#include "project.h"
#include "drivers_config.h"

#include "data_topic.h"
#include "float3.h"
#include "led.h"
#include "main.h"
#include "usart.h"
#include "waveform.h"
#include "vt100.h"
#include "event_uart.h"

// #include "usbd_cdc_if.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>



void wt901b_acc_callback(void);
void seq_callback(void);

static data_sub_t wt901b_acc_sub = { 0 };

static uint8_t uart6_buffer[WT901B_RX_BUFFER_SIZE > sizeof(event_uart_msg_t) ? WT901B_RX_BUFFER_SIZE : sizeof(event_uart_msg_t)]; // Ensure the buffer is at least 4 bytes long

void setup(void) {
    LED_Init(&led0_rgb.red  , DRIVERS_CONFIG_LED0_TIMER, DRIVERS_CONFIG_LED0_CHANNEL_RED);
    LED_Init(&led0_rgb.green, DRIVERS_CONFIG_LED0_TIMER, DRIVERS_CONFIG_LED0_CHANNEL_GREEN);
    LED_Init(&led0_rgb.blue , DRIVERS_CONFIG_LED0_TIMER, DRIVERS_CONFIG_LED0_CHANNEL_BLUE);

    LED_RGB_SetColor(&led0_rgb, FLOAT3_ZERO);

    UART_buffer_init(&huart6, uart6_buffer, sizeof(uart6_buffer));

    uart_mux_init(&uart_mux, DRIVERS_CONFIG_UART_MUX_GPIO_PORT, DRIVERS_CONFIG_UART_MUX_GPIO_PIN);
    uart_mux_set(&uart_mux, UART_MUX_CHANNEL_0);

    event_uart_consumer_init(&event_uart_consumer, SEQ_DA_GPIO_Port, SEQ_DA_Pin, &uart_mux, UART_MUX_CHANNEL_1, &huart6);

    WT901B_Init(&wt901b, &huart6);
    data_sub_attach(&wt901b_acc_sub, &wt901b.data_topic, DATA_ATTACH_FROM_OLDEST);
}

void loop(void) {
    
    WT901B_Parse_Frames(&wt901b);
    wt901b_acc_callback();
    seq_callback();

    event_uart_consumer_run(&event_uart_consumer);

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
            // CDC_Transmit_FS(msg, strlen((char*)msg) + 1); // Send accel data over USB CDC
        }
    }
}

void seq_callback(void) {
    if (event_uart_consumer.cb.count > 0) {
        event_uart_msg_t msg;
        cb_pop(&event_uart_consumer.cb, &msg);
        LED_RGB_SetColor(&led0_rgb, FLOAT3_UNIT_Y);
        HAL_Delay(100);
        LED_RGB_SetColor(&led0_rgb, FLOAT3_ZERO);
        uint8_t buffer[256];
        snprintf((char*)buffer, sizeof(buffer), "SEQ received: timestamp=%lu, type=%d\r\n", (unsigned long)msg.timestamp, msg.type);
        // CDC_Transmit_FS(buffer, strlen((char*)buffer) + 1);
    }
}