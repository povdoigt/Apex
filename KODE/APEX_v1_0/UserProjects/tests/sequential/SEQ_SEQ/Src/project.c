#include "project.h"
#include "drivers_config.h"

#include "data_topic.h"
#include "float3.h"
#include "led.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_uart.h"
#include "usart.h"
#include "waveform.h"
#include "vt100.h"


#include "usb_device.h"
#include "usbd_cdc_if.h"


#include <stdint.h>
#include <stdio.h>



void wt901b_acc_callback(void);

static data_sub_t wt901b_acc_sub = { 0 };

void setup(void) {

    LED_Init(&led0_rgb.red  , DRIVERS_CONFIG_LED0_TIMER, DRIVERS_CONFIG_LED0_CHANNEL_RED);
    LED_Init(&led0_rgb.green, DRIVERS_CONFIG_LED0_TIMER, DRIVERS_CONFIG_LED0_CHANNEL_GREEN);
    LED_Init(&led0_rgb.blue , DRIVERS_CONFIG_LED0_TIMER, DRIVERS_CONFIG_LED0_CHANNEL_BLUE);

    LED_RGB_SetColor(&led0_rgb, FLOAT3_ZERO);

    uart_mux_init(&uart_mux, DRIVERS_CONFIG_UART_MUX_GPIO_PORT, DRIVERS_CONFIG_UART_MUX_GPIO_PIN);

    // sx127x_Init(&sx127x_2, sx127x_base_config_2, sx127x_modulation_2, (sx127x_mod_config_t){.fsk_ook = sx127x_FSK_OOK_config});

    WT901B_Init(&wt901b, &huart6);

    data_sub_attach(&wt901b_acc_sub, &wt901b.data_topic, DATA_ATTACH_FROM_OLDEST);

    uart_mux_set(&uart_mux, UART_MUX_CHANNEL_0);

}
static bool flag_tx = false;
static bool flag_rx = false;

void loop(void) {
    
    WT901B_Parse_Frames(&wt901b);
    wt901b_acc_callback();

    if (flag_tx && flag_rx) {
        flag_rx = false;
        flag_tx = false; // Allow transmitting again after receiving a message
        LED_RGB_SetColor(&led0_rgb, FLOAT3_UNIT_Z); // Set LED to blue
        HAL_Delay(100);
        LED_RGB_SetColor(&led0_rgb, FLOAT3_ZERO); // Turn off LED
        CDC_Transmit_FS((uint8_t*)"Switched back to 1st UART\r\n", 28);
        HAL_Delay(10);
        uart_mux_set(&uart_mux, UART_MUX_CHANNEL_0); // Switch back to 1st UART (USB)
    }

    if (HAL_GPIO_ReadPin(SEQ_DA_GPIO_Port, SEQ_DA_Pin) == GPIO_PIN_SET && !flag_tx) {
        uart_mux_set(&uart_mux, UART_MUX_CHANNEL_1); // Switch to 2nd UART (SEQ)
        CDC_Transmit_FS((uint8_t*)"Switched to 2nd UART\r\n", 23);
        HAL_Delay(10);
        HAL_UART_Transmit(&huart6, (uint8_t*)"Hello from APEX\r\n", 18, HAL_MAX_DELAY);
        flag_tx = true;
    }

    HAL_Delay(10);
}


void wt901b_acc_callback(void) {
    uint8_t msg[100];
    if (data_sub_num_to_read(&wt901b_acc_sub) > 0) {
        WT901B_Frame_t frame;
        data_sub_read(&wt901b_acc_sub, &frame); 

        if (frame.type == WT901B_FRAME_ACCEL) {
            snprintf((char*)msg, sizeof(msg), "Accel (g): ax=%.2f ay=%.2f az=%.2f\r\n",
                frame.data.accel.ax_g, frame.data.accel.ay_g, frame.data.accel.az_g);
            CDC_Transmit_FS(msg, strlen((char*)msg)); // Send accel data over USB CDC
        }
    }
}

void uart_seq_callback(uint16_t size) {
    flag_rx = true;
}