// drivers_config.h
#ifndef DRIVERS_CONFIG_H
#define DRIVERS_CONFIG_H

#include "main_config.h"
#include "stm32f411xe.h"

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
extern bmi088_t bmi088;
extern const bmi_config_t bmi088_config;

/* Data topic pointers set by the background read tasks */
#include "data_topic.h"
extern data_topic_t *bmi088_acc_topic;
extern data_topic_t *bmi088_gyr_topic;

#if (APEX_CFG_PROFILE_TEST == 1)
#if (APEX_CFG_SCHED_RTOS == 1)
#include "BMI088_rtos_test.h"
#endif /* APEX_CFG_SCHED_RTOS */
#endif /* APEX_CFG_PROFILE_TEST */

#endif /* APEX_ENABLE_BMI088 */

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
// I2C Mux configuration
// =======================================================================
#if (APEX_ENABLE_I2C_MUX == 1)
#include "i2c_mux.h"
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
extern sx127x_t sx127x_1;
#endif

// =======================================================================
// SX127x 2 configuration
// =======================================================================
#if (APEX_ENABLE_SX127X_2 == 1)
#include "sx127x.h"
extern const sx127x_LORA_config_t sx127x_LORA_config;
extern const sx127x_FSK_OOK_config_t sx127x_FSK_OOK_config;
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
extern W25Q_t w25q;
#endif

// =======================================================================
// WT901B configuration
// =======================================================================
#if (APEX_ENABLE_WT901B == 1)
#include "WT901B.h"
#endif

// =======================================================================
// Initialization structures and prototypes
// =======================================================================

typedef struct DRIVERS_CONFIG_init_result_t {
#if (APEX_ENABLE_BMI088 == 1)
    BMI_STATE bmi088_init_res;
#endif
} DRIVERS_CONFIG_init_result_t;

void DRIVERS_CONFIG_init_seq(DRIVERS_CONFIG_init_result_t *result);

#endif // DRIVERS_CONFIG_H
