/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
  /* USER CODE END Header */
  /* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "drivers/ADXL375.h"
#include "drivers/BMI088.h"
#include "drivers/BMP388.h"
#include "drivers/BMP388.h"
#include "drivers/buzzer.h"
#include "drivers/gps.h"
#include "drivers/led.h"
#include "drivers/LSM303AGR.h"
#include "drivers/rfm96w.h"
#include "drivers/usb.h"
#include "drivers/w25q_mem.h"

#include "peripherals/adc.h"
#include "peripherals/gpio.h"
#include "peripherals/dma.h"
#include "peripherals/i2c.h"
#include "peripherals/spi.h"
#include "peripherals/tim.h"
#include "peripherals/usart.h"

#include "utils/scheduler.h"
#include "utils/test.h"

#include "usbd_cdc_if.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// MACHINE machine;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t time_rfm;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_I2C3_Init();
	MX_SPI2_Init();
	MX_TIM2_Init();
	MX_TIM4_Init();
	MX_USART1_UART_Init();
	MX_ADC1_Init();
	MX_SPI1_Init();
	MX_TIM3_Init();
	MX_USB_DEVICE_Init();
	/* USER CODE BEGIN 2 */

	GMS_init(&GMS_memory);

	ADXL375 adxl375;
	BMI088 bmi088;
	BMP388_HandleTypeDef bmp388;
	BUZZER buzzer;
	GPS_t gps;
	LED_RGB led_rgb_0;
	LED_RGB led_rgb_1;
	LSM303AGR lsm303agr;
	RFM96_Chip rfm96w;
	W25Q_Chip w25q_mem;

	COMPONENTS components;
	init_components(&components,
					&adxl375,
					&bmi088,
					&bmp388,
					&buzzer,
					&gps,
					&led_rgb_0,
					&led_rgb_1,
					&lsm303agr,
					&rfm96w,
					&w25q_mem);

	FLASH_STREAM flash_stream;
	init_machine(&machine, &components, &flash_stream);

	init_obj_pools();

	SCHEDULER scheduler;
	SCHEDULER_init(&scheduler);


	// TASK *task = SCHEDULER_add_task_macro(&scheduler, ASYNC_BUZZER_play_note, false);
	// ASYNC_BUZZER_play_note_init(task, &buzzer,
	// 							buzzer_song_bank.beep_0_freqs,
	// 							buzzer_song_bank.beep_0_durations,
	// 							BUZZER_BEEP_SONG_SIZE);

	

	// uint32_t addr;
	// for (int i = 0; i < 16; i++) {
	// 	addr = 4096 * 16 + i * 4096;
	// 	W25Q_EarseSector(&w25q_mem, addr);
	// }


	// W25Q_EarseSector(&w25q_mem, 0);
	// W25Q_WaitForReady(&w25q_mem);


	// uint8_t data[256] = { 0 };

	// for (int i = 0; i < 256; i++) {
	// 	data[i] = i;
	// }
	// uint32_t addr = 0x0012000; 
	// W25Q_PageProgram(&w25q_mem, data, addr, 255);

	// for (int i = 0; i < 32; i++) {
	// 	uint8_t buf[256] = { 0 };
	// 	addr = 256 * i;
	// 	W25Q_ReadData(&w25q_mem, buf, addr, 256);
	// 	__NOP();
	// }

	// uint32_t addr;
	// for (int i = 0; i < 16 * 16 * 256; i++) {
	// 	addr = i * 256;
	// 	W25Q_PageProgram(&w25q_mem, data, addr, 256);
	// }

	// W25Q_EarseSector(&w25q_mem, 0);
	
	// uint8_t tx_buf[5] = { W25Q_64KB_BLOCK_ERASE, 0, 0, 0, 0 };
	// W25Q_TransmitReceive(&w25q_mem, tx_buf, NULL, 5, 0);

	// W25Q_WaitForReady(&w25q_mem);


	// uint8_t data[256] = { 0 };

	// W25Q_ReadData(&w25q_mem, data, 0, 256);

	// __NOP();


	// uint32_t addr;
	// for (int i = 0; i < 16 * 256; i++) {
	// 	addr = 4096 * 16 + i * 256;
	// 	W25Q_PageProgram(&w25q_mem, data, addr, 256);
	// }


	BUZZER_play_note(&buzzer,
					 buzzer_song_bank.beep_0_freqs,
					 buzzer_song_bank.beep_0_durations,
					 BUZZER_BEEP_SONG_SIZE);
	HAL_Delay(1500);

	// TASK* task = SCHEDULER_add_task_macro(&scheduler, ASYNC_LED_RGB_Blink, false);
	// ASYNC_LED_RGB_Blink_init(task, &machine.led_rgb, 

	// LED led0_g;
	// LED_Init(&led0_g, &htim3, TIM_CHANNEL_1);

	// LED_SetBrightness(&led0_g, 127);


	// LED_RGB_SetColor(machine.components->led_rgb_0, 0, 255, 0);

	uint8_t gps_tx_buf[4096] = { 0 };

	char buff[256];
	char acc_tot[20], lat[20], lon[20], alt[10], time[10];

	DATA_ALL_TIMESTAMP_2 data;
	data.dummy_start = 0x55AA;
	data.dummy_end = 0xADDE;

	MACHINE_STATE_2 state = MACHINE_STATE_2_START_TIMER;

	uint8_t nb_acc;
	bool flight_start = false;
	float max_acc = 0.0f;

	uint32_t i;
	uint32_t next_time;

	FLASH_STREAM fs;
	flash_stream_init(&fs, machine.components->flash, sizeof(DATA_ALL_TIMESTAMP_2));

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		SCHEDULER_run(&scheduler);
		add_task_if_flag(&scheduler, &machine);
		// size_t size = sizeof(DATA_ALL_TIMESTAMP);

		HAL_UARTEx_ReceiveToIdle_DMA(machine.components->gps->uart, machine.components->gps->rx_buffer, RX_BUF_SIZE);

		// HAL_UART_Receive(gps.uart, gps_tx_buf, sizeof(gps_tx_buf), 100);
		// CDC_Transmit_FS(gps_tx_buf, sizeof(gps_tx_buf));
		// CDC_Transmit_FS(machine.components->gps->rx_buffer, sizeof(GPS_BUF_SIZE));
			// if (HAL_GetTick() - time0 <= 10000) {
		// if (i < 0x100000) { // 65536 bytes
			  //     CDC_Transmit_FS(gps_tx_buf, sizeof(gps_tx_buf));
		//     i += sizeof(gps_tx_buf);
		// }	
		// test_USB();
			  // HAL_Delay(1);
			// } else {
			// 	__NOP(); // Do nothing after 10 seconds
			// }

		// size_t size = GMS_get_free_size(&GMS_memory);
		// sprintf(buff, "Free blocks size: %u\n", size);
		// CDC_Transmit_FS((uint8_t*)buff, strlen(buff) + 1);
		// HAL_Delay(1);

		switch (state) {
		case MACHINE_STATE_2_START_TIMER: {
			next_time = HAL_GetTick() + 1000 * 60; // Start timer for 1 min
			// next_time = HAL_GetTick(); // Start timer for 1 min
			state = MACHINE_STATE_2_WAIT_TIMER;
			break; }
		case MACHINE_STATE_2_WAIT_TIMER: {
			uint32_t time0 = HAL_GetTick();
			if (HAL_GetTick() >= next_time) {
				state = MACHINE_STATE_2_CHECK_TELEM;
				next_time = HAL_GetTick() + 5000; // Reset timer for 5 sec
				i++;
			}
			break; }
		case MACHINE_STATE_2_CHECK_TELEM: {
			sprintf(buff, "Ping APEX %i\n", i);
			i++;
			RFM96_Print(&rfm96w, buff);
			HAL_Delay(200);
			uint32_t time0 = HAL_GetTick();
			if (HAL_GetTick() >= next_time) {
				state = MACHINE_STATE_2_DO;
			}
			break; }
		case MACHINE_STATE_2_DO: {

			BMI088_ReadAccelerometer(machine.components->bmi);
			BMI088_ReadGyroscope(machine.components->bmi);

			data.bmi088_acc.x = machine.components->bmi->acc_mps2.x;
			data.bmi088_acc.y = machine.components->bmi->acc_mps2.y;
			data.bmi088_acc.z = machine.components->bmi->acc_mps2.z;
			data.bmi088_gyr.x = machine.components->bmi->gyr_rps.x;
			data.bmi088_gyr.y = machine.components->bmi->gyr_rps.y;
			data.bmi088_gyr.z = machine.components->bmi->gyr_rps.z;
			data.latitude = machine.components->gps->dec_latitude;
			data.longitude = machine.components->gps->dec_longitude;
			data.altitude = machine.components->gps->altitude_ft;
			data.gps_time = machine.components->gps->utc_time;

			machine.datas.gps_time += 20000; // GMT+2

			float acc_tot_f = sqrtf(
				data.bmi088_acc.x * data.bmi088_acc.x +
				data.bmi088_acc.y * data.bmi088_acc.y +
				data.bmi088_acc.z * data.bmi088_acc.z);
			
			max_acc = acc_tot_f > max_acc ? acc_tot_f : max_acc;

			if (!flight_start) {
				if (acc_tot_f > 9.81 * 3.0f) {
					nb_acc++;
					if (nb_acc > 3) {
						flight_start = true;
					}
				}
			}

			float_format(acc_tot, acc_tot_f, 5, 10);
			float_format(lat, machine.datas.latitude, 5, 10);
			float_format(lon, machine.datas.longitude, 5, 10);
			float_format(alt, machine.datas.altitude, 5, 10);
			float_format(time, machine.datas.gps_time, 2, 10);

			sprintf(buff, "%s, %s,%s,%s,%s", acc_tot, lon, lat, alt, time);

			RFM96_Print(&rfm96w, buff);

			HAL_Delay(100);
			state = MACHINE_STATE_2_WRIT_FLASH;
			break; }
		case MACHINE_STATE_2_WRIT_FLASH: {
			if (flight_start) {
				flash_stream_write(&fs, (uint8_t*)&data, sizeof(DATA_ALL_TIMESTAMP_2));
			}
			state = MACHINE_STATE_2_DO;
		}
		default: {
			break; }
		}






		// TEST ENVOIE DATA
		// char buff[256] = "Coucou, je m'appelle Malo !";
		// RFM96_Print(&rfm96w, buff);
		// HAL_Delay(100);

		// TEST RECEPTION DATA
		// char buff[256] = { 0 };
		// int len = RFM96_ParsePacket(&rfm96w);
		// if (len > 0) {
		// 	RFM96_Read(&rfm96w, (uint8_t*)buff, len);
		// 	CDC_Transmit_FS((uint8_t*)buff, len);
		// 	HAL_Delay(1);
		// }


		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	*/
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	* in the RCC_OscInitTypeDef structure.
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 10;
	RCC_OscInitStruct.PLL.PLLN = 96;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
		| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line) {
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	   /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
