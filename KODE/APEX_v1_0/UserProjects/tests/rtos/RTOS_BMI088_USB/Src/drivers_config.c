#include "drivers_config.h"

// =======================================================================
// ADXL345 configuration
// =======================================================================
#if (APEX_ENABLE_ADXL345 == 1)

#define DRIVERS_CONFIG_ADXL375_SPI_HANDLE   hspi1
#define DRIVERS_CONFIG_ADXL375_CS_PIN       GPIO_PIN_3
#define DRIVERS_CONFIG_ADXL375_CS_PORT      GPIOA
adxl375_t ADXL375;

#endif

// =======================================================================
// BMI088 configuration
// =======================================================================
#if (APEX_ENABLE_BMI088 == 1)

#define DRIVERS_CONFIG_BMI088_SPI_HANDLE    hspi1
#define DRIVERS_CONFIG_BMI088_ACC_CS_PIN    GPIO_PIN_4
#define DRIVERS_CONFIG_BMI088_ACC_CS_PORT   GPIOA
#define DRIVERS_CONFIG_BMI088_GYR_CS_PIN    GPIO_PIN_2
#define DRIVERS_CONFIG_BMI088_GYR_CS_PORT   GPIOB

const bmi_config_t bmi088_config = {
    .acc_range  = BMI_ACC_RANGE_24G,
    .acc_bwp    = BMI_ACC_CONF_BWP_NORMAL,
    .acc_odr    = BMI_ACC_CONF_ODR_100_HZ,
    .acc_pwr    = BMI_ACC_PWR_CONF_ACTIVE,
    .acc_ctrl   = BMI_ACC_PWR_CTRL_ENABLE,

    .gyr_range  = BMI_GYR_RANGE_2000,
    .gyr_bw     = BMI_GYR_BANDWIDTH_BW_23_HZ,
    .gyr_mode   = BMI_GYR_LPM1_MODE_NORMAL,
};

bmi088_t bmi088;

/* Topic pointers — set by TASK_BMI088_ReadAcc / TASK_BMI088_ReadGyr */
data_topic_t *bmi088_acc_topic = NULL;
data_topic_t *bmi088_gyr_topic = NULL;

#endif

// =======================================================================
// BMP388 configuration
// =======================================================================
#if (APEX_ENABLE_BMP388 == 1)
BMP388_HandleTypeDef bmp388;
#endif

// =======================================================================
// Buzzer configuration
// =======================================================================
#if (APEX_ENABLE_BUZZER == 1)
#define DRIVERS_CONFIG_BUZZER_TIM_HANDLE    htim3
#define DRIVERS_CONFIG_BUZZER_CHANNEL       TIM_CHANNEL_4
buzzer_t buzzer;
#endif

// =======================================================================
// GPS configuration
// =======================================================================
#if (APEX_ENABLE_GPS == 1)
// GPS config here
#endif

// =======================================================================
// LED configuration
// =======================================================================
#if (APEX_ENABLE_LED == 1)
// LED config here
#endif

// =======================================================================
// LSM303AGR configuration
// =======================================================================
#if (APEX_ENABLE_LSM303AGR == 1)
lsm303agr_t lsm303agr;
#endif

// =======================================================================
// SX127x 1 configuration
// =======================================================================
#if (APEX_ENABLE_SX127X_1 == 1)
sx127x_t sx127x_1;
#endif

// =======================================================================
// SX127x 2 configuration
// =======================================================================
#if (APEX_ENABLE_SX127X_2 == 1)
sx127x_t sx127x_2;
#endif

// =======================================================================
// W25Q512 configuration
// =======================================================================
#if (APEX_ENABLE_W25Q512 == 1)
W25Q_t w25q;
#endif

// =======================================================================
// Sequential initialisation stub (unused in RTOS mode — kept for linker)
// =======================================================================
void DRIVERS_CONFIG_init_seq(DRIVERS_CONFIG_init_result_t *result) {
    (void)result;
}
