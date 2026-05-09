#include "project.h"

#include "WT901B.h"
#include "data_topic.h"
#include "drivers_config.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "vt100.h"
#include <stdint.h>
#include <stdio.h>

void wt901b_acc_callback(void);

static data_sub_t wt901b_acc_sub = { 0 };

void setup(void) {

    // sx127x_Init(&sx127x_2, sx127x_base_config_2, sx127x_modulation_2, (sx127x_mod_config_t){.fsk_ook = sx127x_FSK_OOK_config});

    WT901B_Init(&wt901b, &huart6);

    data_sub_attach(&wt901b_acc_sub, &wt901b.data_topic, DATA_ATTACH_FROM_OLDEST);

}

static uint32_t t0 = 0;

void loop(void) {
    
    WT901B_Parse_Frames(&wt901b);

    wt901b_acc_callback();

    // if (HAL_GetTick() - t0 >= 5000) {
    //     t0 = HAL_GetTick();
    //     HAL_GPIO_TogglePin(UART_SEL_GPIO_Port, UART_SEL_Pin);
    //     CDC_Transmit_FS((uint8_t*)"Change UART\r\n", 16);
    // }

    HAL_Delay(1);
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