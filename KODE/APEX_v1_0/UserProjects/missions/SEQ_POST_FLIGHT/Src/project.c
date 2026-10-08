#include "project.h"

#include "BMI088.h"
#include "BMP388.h"
#include "FreeRTOS.h"
#include "buzzer.h"
#include "circular_buffer.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"

#include "WT901B.h"

#include "data_topic.h"
#include "drivers_config.h"
#include "event_uart.h"
#include "flash_chunk.h"
#include "float3.h"
#include "led.h"
#include "scheduler.h"
#include "stm32f4xx_hal.h"
#include "w25q.h"
#include "waveform.h"

#include "tools.h"
#include "usbd_cdc_if.h"
#include "waveform.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>

static Apex_Flight_Phase_t flight_phase = APEX_FLIGHT_PHASE_INIT;

static event_uart_consumer_t uart_consumer;

static uint8_t uart6_buffer[sizeof(event_uart_msg_t) > WT901B_RX_BUFFER_SIZE ? sizeof(event_uart_msg_t) : WT901B_RX_BUFFER_SIZE]; // Ensure the buffer is at least 4 bytes long

static uint32_t t0, t1;

static size_t addr;

void setup() {

	BUZZER_set_song_bank(&buzzer_song_bank);

	LED_Init(&led0_rgb.red	, DRIVERS_CONFIG_LED0_TIMER, DRIVERS_CONFIG_LED0_CHANNEL_RED);
	LED_Init(&led0_rgb.green, DRIVERS_CONFIG_LED0_TIMER, DRIVERS_CONFIG_LED0_CHANNEL_GREEN);
	LED_Init(&led0_rgb.blue	, DRIVERS_CONFIG_LED0_TIMER, DRIVERS_CONFIG_LED0_CHANNEL_BLUE);
	LED_RGB_SetColor(&led0_rgb, FLOAT3_UNIT_Y);

	HAL_Delay(5000);
	LED_RGB_SetColor(&led0_rgb, FLOAT3_ZERO);
	HAL_Delay(100);
	LED_RGB_SetColor(&led0_rgb, FLOAT3_UNIT_Y);
	

    UART_buffer_init(&huart6, uart6_buffer, sizeof(uart6_buffer));

    uart_mux_set(&uart_mux, UART_MUX_CHANNEL_0);

	event_uart_consumer_init(&uart_consumer, SEQ_DA_GPIO_Port, SEQ_DA_Pin, &uart_mux, UART_MUX_CHANNEL_1, &huart6);

	W25Q_SendCmd(&w25q, W25Q_CHIP_ERASE);
	// W25Q_WaitForReady(&w25q);

	HAL_Delay(1000);

	LED_RGB_SetColor(&led0_rgb, FLOAT3_ZERO);
	HAL_Delay(1000);
	LED_RGB_SetColor(&led0_rgb, FLOAT3_UNIT_Y);

	BUZZER_play_note(&buzzer, buzzer_song_bank.beep_0_freqs, buzzer_song_bank.beep_0_durations, BUZZER_BEEP_SONG_SIZE);

	t0 = HAL_GetTick();
	t1 = HAL_GetTick() + 50;

}

void loop() {


	// event_uart_consumer_run(&uart_consumer);

	// if (uart_consumer.cb.count > 0) {
	// 	event_uart_msg_t msg;
	// 	cb_pop(&uart_consumer.cb, &msg);
		
	// 	LED_RGB_SetColor(&led0_rgb, FLOAT3_UNIT_Y);
	// 	HAL_Delay(1);
	// 	LED_RGB_SetColor(&led0_rgb, FLOAT3_ZERO);
	// }



	Apex_data_sensors_t data_sensors = { .time = HAL_GetTick() };

	uint32_t raw_press, raw_temp, raw_time;

	// get data from sensors
	BMI088_ReadAcc(&bmi088, &data_sensors.acc);
	BMI088_ReadGyr(&bmi088, &data_sensors.gyr);
	
	BMP388_ReadRawPressTempTime(&bmp388, &raw_press, &raw_temp, &raw_time);
	BMP388_CompensateRawPressTemp(&bmp388, raw_press, raw_temp, &data_sensors.baro_press, &data_sensors.baro_temp);

	W25Q_WriteData(&w25q, (uint8_t*)&data_sensors, addr, sizeof(data_sensors));

	HAL_Delay(10);

	// if (HAL_GetTick() - t0 > 100) {
	// 	sx127x_TxSend(&sx127x_1, (uint8_t *)&data_sensors, sizeof(data_sensors));
	// 	t0 = HAL_GetTick();
	// }
	// if (HAL_GetTick() - t1 > 100) {
	// 	sx127x_TxSend(&sx127x_2, (uint8_t *)&data_sensors, sizeof(data_sensors));
	// 	t1 = HAL_GetTick();
	// }



	// switch (flight_phase) {
	// 	case APEX_FLIGHT_PHASE_INIT:
	// 		// Initialization phase logic
	// 		break;
	// 	case APEX_FLIGHT_PHASE_PRE_LAUNCH: {
	// 		static Apex_data_sensors data[NBR_DATA_BEFORE_LAUNCH + NBR_DATA_AFTER_LAUNCH] = { 0 };
	// 		static Apex_data_sensors *data_before = data;
	// 		static Apex_data_sensors *data_after = data + NBR_DATA_BEFORE_LAUNCH;

	// 		static circular_buffer_t data_before_cb;
	// 		static circular_buffer_t data_after_cb;
	// 		cb_init(&data_before_cb, data_before, sizeof(data_t), NBR_DATA_BEFORE_LAUNCH, CB_OVERWRITE_OLDEST);
	// 		cb_init(&data_after_cb, data_after, sizeof(data_t), NBR_DATA_AFTER_LAUNCH, CB_REJECT_NEW);

	// 		static data_sub_t sub = { 0 };
	// 		data_sub_attach(&sub, *(args->data_topic), DATA_ATTACH_FROM_NOW);

	// 		Apex_data_sensors flash_data;

	// 		static size_t nb_data_after = 0;

	// 		static bool sample_done = false;


	// 		if (!sample_done) {
	// 			data_sub_wait_for_data(&sub, osWaitForever);
	// 			data_sub_read(&sub, &(flash_data.acc));

	// 			flash_data.launch_detected = *(args->launch_signal);

	// 			if (!*(args->launch_signal)) {
	// 				// Before launch detection, keep filling the pre-launch circular buffer. Once it's full, the oldest
	// 				// data will be overwritten, ensuring we always have the most recent NBR_DATA_BEFORE_LAUNCH samples
	// 				// leading up to the launch.	
	// 				cb_push(&data_before_cb, &flash_data);
	// 			} else {
	// 				// Keep filling the post-launch circular buffer until it's full. Once it's full, stop accepting new
	// 				// data to preserve the context immediately following the launch.
	// 				cb_push(&data_after_cb, &flash_data);
	// 				nb_data_after++;
	// 				if (nb_data_after >= NBR_DATA_AFTER_LAUNCH) {
	// 					sample_done = true;
	// 				}
	// 			}
	// 		} else {
	// 			flight_phase = APEX_FLIGHT_PHASE_POST_LAUNCH
	// 		}	
	// 		break;
	// 	}
	// 	case APEX_FLIGHT_PHASE_POST_LAUNCH:
	// 		// Post-launch phase logic
	// 		break;
	// 	default:
	// 		// Handle unexpected flight phase
	// 		break;
	// }
}