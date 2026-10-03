// driver_config.h
// This file contains the configuration for the hardware drivers used in the APEX project.

#ifndef DRIVERS_CONFIG_H
#define DRIVERS_CONFIG_H

#include "main_config.h"

#include "adc.h"
#include "dma.h"
#include "gpio.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

// =======================================================================
// ADXL345 configuration
// =======================================================================
#if (APEX_ENABLE_ADXL345 == 1)

#include "ADXL375.h"
extern adxl375_t ADXL375;

#endif

// =======================================================================
// BMI088 configuration
// =======================================================================
#if (APEX_ENABLE_BMI088 == 1)

#include "BMI088.h"

extern SPI_HandleTypeDef * const DRIVERS_CONFIG_BMI088_SPI_HANDLE;
extern GPIO_TypeDef * const DRIVERS_CONFIG_BMI088_ACC_CS_PORT;
extern GPIO_TypeDef * const DRIVERS_CONFIG_BMI088_GYR_CS_PORT;
extern const uint16_t DRIVERS_CONFIG_BMI088_ACC_CS_PIN;
extern const uint16_t DRIVERS_CONFIG_BMI088_GYR_CS_PIN;

extern const bmi_config_t bmi088_config;
extern bmi088_t bmi088;

#if (APEX_CFG_PROFILE_TEST == 1)

#include "BMI088_seq_test.h"

#endif

#endif


// =======================================================================
// BMP388 configuration
// =======================================================================
#if (APEX_ENABLE_BMP388 == 1)

#include "BMP388.h"
extern BMP388_HandleTypeDef bmp388;

#endif


// =======================================================================
// Buzzer configuration
// =======================================================================
#if (APEX_ENABLE_BUZZER == 1)

#include "buzzer.h"
extern buzzer_t buzzer;

#endif


// =======================================================================
// GPS configuration
// =======================================================================
#if (APEX_ENABLE_GPS == 1)

#include "gps.h"

#endif


// =======================================================================
// LED configuration
// =======================================================================
#if (APEX_ENABLE_LED == 1)

#include "led.h"

extern TIM_HandleTypeDef * const DRIVERS_CONFIG_LED0_TIMER;
extern const uint32_t DRIVERS_CONFIG_LED0_CHANNEL_RED;
extern const uint32_t DRIVERS_CONFIG_LED0_CHANNEL_GREEN;
extern const uint32_t DRIVERS_CONFIG_LED0_CHANNEL_BLUE;

extern led_rgb_t led0_rgb;

#endif


// =======================================================================
// LSM303AGR configuration
// =======================================================================
#if (APEX_ENABLE_LSM303AGR == 1)

#include "LSM303AGR.h"
extern lsm303agr_t lsm303agr;

#endif


// =======================================================================
// SX127x 1 configuration
// =======================================================================
#if (APEX_ENABLE_SX127X_1 == 1)

#include "sx127x.h"

extern const sx127x_LORA_config_t sx127x_LORA_config_1;
extern const sx127x_FSK_OOK_config_t sx127x_FSK_OOK_config_1;
extern const sx127x_base_config_t sx127x_base_config_1;
extern const sx127x_modulation_t sx127x_modulation_1;

extern sx127x_t sx127x_1;

#endif

// =======================================================================
// SX127x 2 configuration
// =======================================================================
#if (APEX_ENABLE_SX127X_2 == 1)

#include "sx127x.h"

extern const sx127x_LORA_config_t sx127x_LORA_config_2;
extern const sx127x_FSK_OOK_config_t sx127x_FSK_OOK_config_2;
extern const sx127x_base_config_t sx127x_base_config_2;
extern const sx127x_modulation_t sx127x_modulation_2;

extern sx127x_t sx127x_2;

#endif

// =======================================================================
// UART Mux configuration
// =======================================================================
#if (APEX_ENABLE_UART_MUX == 1)
#include "uart_mux.h"
extern GPIO_TypeDef * const DRIVERS_CONFIG_UART_MUX_GPIO_PORT;
extern const uint16_t DRIVERS_CONFIG_UART_MUX_GPIO_PIN; 
extern uart_mux_t uart_mux;
#endif


// =======================================================================
// W25Q512 configuration
// =======================================================================
#if (APEX_ENABLE_W25Q512 == 1)

#include "w25q.h"

extern SPI_HandleTypeDef * const DRIVERS_CONFIG_W25Q512_SPI_HANDLE;
extern GPIO_TypeDef * const DRIVERS_CONFIG_W25Q512_CS_PORT;
extern const uint16_t DRIVERS_CONFIG_W25Q512_CS_PIN;

extern W25Q_t w25q;

#endif


// =======================================================================
// WT901B configuration
// =======================================================================
#if (APEX_ENABLE_WT901B == 1)

#include "WT901B.h"

#endif





// =======================================================================
// Initialization functions prototypes
// =======================================================================

typedef struct DRIVERS_CONFIG_init_result_t {
#if (APEX_ENABLE_ADXL345 == 1)
    // No specific initialization result for ADXL375 for now, but will add here later
#endif
#if (APEX_ENABLE_BMI088 == 1)
    BMI_STATE bmi088_init_res;
#endif
#if (APEX_ENABLE_BMP388 == 1)
    // No specific initialization result for BMP388 for now, but will add here later
#endif
#if (APEX_ENABLE_BUZZER == 1)
    // No specific initialization result for buzzer for now, but will add here later
#endif
#if (APEX_ENABLE_GPS == 1)
    // GPS initialization result here
#endif
#if (APEX_ENABLE_LED == 1)
    // LED initialization result here
#endif
#if (APEX_ENABLE_LSM303AGR == 1)
    // No specific initialization result for LSM303AGR for now, but will add here later
#endif
#if (APEX_ENABLE_SX127X_1 == 1)
    sx127x_status_t sx127x_1_init_res;
#endif
#if (APEX_ENABLE_SX127X_2 == 1)
    sx127x_status_t sx127x_2_init_res;
#endif
#if (APEX_ENABLE_W25Q512 == 1)
    W25Q_STATE w25q_init_res;
#endif
} DRIVERS_CONFIG_init_result_t;
void DRIVERS_CONFIG_init_seq(DRIVERS_CONFIG_init_result_t *result);


#endif // DRIVERS_CONFIG_H
