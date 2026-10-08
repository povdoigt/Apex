#include "project.h"
#include "drivers_config.h"

#include "led.h"

#include "stm32f4xx_hal.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>


static const uint32_t period = 100;

static uint32_t t0;

static uint8_t buf[14];
static uint16_t len = 0;


void setup(void) {

    LED_Init(&led0_rgb.red  , DRIVERS_CONFIG_LED0_TIMER, DRIVERS_CONFIG_LED0_CHANNEL_RED  );
    LED_Init(&led0_rgb.green, DRIVERS_CONFIG_LED0_TIMER, DRIVERS_CONFIG_LED0_CHANNEL_GREEN);
    LED_Init(&led0_rgb.blue , DRIVERS_CONFIG_LED0_TIMER, DRIVERS_CONFIG_LED0_CHANNEL_BLUE );
    LED_RGB_SetColor(&led0_rgb, (float3_t){0.0f, 0.0f, 0.0f});

    sx127x_status_t status = sx127x_Init(&sx127x_2, sx127x_base_config_2, sx127x_modulation_2, (sx127x_mod_config_t){.fsk_ook = sx127x_FSK_OOK_config_2});

    t0 = HAL_GetTick();
}


void loop(void) {

    if (HAL_GetTick() - t0 >= period) {
        t0 = HAL_GetTick();
        snprintf((char *)buf, sizeof(buf), "Hello World !");
        sx127x_TxSend(&sx127x_2, buf, strlen((char *)buf));
        memset(buf, 0, sizeof(buf));

        // LED_RGB_SetColor(&led0_rgb, (float3_t){0.0f, 1.0f, 0.0f});
        // HAL_Delay(10);
        // LED_RGB_SetColor(&led0_rgb, (float3_t){0.0f, 0.0f, 0.0f});
    }

    sx127x_status_t status = sx127x_RxReceive(&sx127x_2, buf, &len);
    if (status == sx127x_STATUS_OK && len > 0) {
        if (strcmp(buf, "Hello World !") == 0) {
            LED_RGB_SetColor(&led0_rgb, (float3_t){0.0f, 0.0f, 1.0f});
            HAL_Delay(10);
            LED_RGB_SetColor(&led0_rgb, (float3_t){0.0f, 0.0f, 0.0f});
        }
    }
}