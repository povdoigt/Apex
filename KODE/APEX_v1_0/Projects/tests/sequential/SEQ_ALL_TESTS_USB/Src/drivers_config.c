#include "drivers_config.h"

// =======================================================================
// ADXL345
// =======================================================================
#if (APEX_ENABLE_ADXL345 == 1)
#define DRIVERS_CONFIG_ADXL375_SPI_HANDLE   hspi1
#define DRIVERS_CONFIG_ADXL375_CS_PIN       GPIO_PIN_3
#define DRIVERS_CONFIG_ADXL375_CS_PORT      GPIOA
adxl375_t ADXL375;
#endif

// =======================================================================
// BMI088
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
#endif

// =======================================================================
// BMP388
// =======================================================================
#if (APEX_ENABLE_BMP388 == 1)
BMP388_HandleTypeDef bmp388;
#endif

// =======================================================================
// Buzzer
// =======================================================================
#if (APEX_ENABLE_BUZZER == 1)
#define DRIVERS_CONFIG_BUZZER_TIM_HANDLE    htim3
#define DRIVERS_CONFIG_BUZZER_CHANNEL       TIM_CHANNEL_4
buzzer_t buzzer;
#endif

// =======================================================================
// GPS
// =======================================================================
#if (APEX_ENABLE_GPS == 1)
// ... add GPS configuration here
#endif

// =======================================================================
// LED
// =======================================================================
#if (APEX_ENABLE_LED == 1)
// ... add LED configuration here
#endif

// =======================================================================
// LSM303AGR
// =======================================================================
#if (APEX_ENABLE_LSM303AGR == 1)
lsm303agr_t lsm303agr;
#endif

// =======================================================================
// SX127x 1
// =======================================================================
#if (APEX_ENABLE_SX127X_1 == 1)
sx127x_LORA_config_t sx127x_LORA_config = {
    .implicitHeader  = false,
    .bandwidth       = sx127x_LORA_REG_1D_MODEM_CONFIG1_BW_125KHZ,
    .codingRate      = sx127x_LORA_REG_1D_MODEM_CONFIG1_CR_4_5,
    .spreadingFactor = sx127x_LORA_REG_1E_MODEM_CONFIG2_SF_128CPS,
    .crcEnabled      = true,
};
sx127x_base_config_t sx127x_base_config_1 = {
    .spiHandle     = &hspi1,
    .csPinBank     = CS_LORA_GPIO_Port,
    .csPin         = CS_LORA_Pin,
    .frequency     = 869500000,
    .ocp_current_mA = 240.0,
    .power_dBm     = 20.0,
};
sx127x_modulation_t sx127x_modulation_1 = sx127x_MODULATION_LORA;
sx127x_t sx127x_1;
#endif

// =======================================================================
// SX127x 2
// =======================================================================
#if (APEX_ENABLE_SX127X_2 == 1)
sx127x_base_config_t sx127x_base_config_2 = { 0 };
sx127x_modulation_t  sx127x_modulation_2  = sx127x_MODULATION_FSK;
sx127x_t sx127x_2;
#endif

// =======================================================================
// W25Q512
// =======================================================================
#if (APEX_ENABLE_W25Q512 == 1)
#define DRIVERS_CONFIG_W25Q512_SPI_HANDLE   hspi2
#define DRIVERS_CONFIG_W25Q512_CS_PIN       GPIO_PIN_1
#define DRIVERS_CONFIG_W25Q512_CS_PORT      GPIOC
W25Q_t w25q;
#endif

// =======================================================================
// WT901B
// =======================================================================
#if (APEX_ENABLE_WT901B == 1)
// ... add WT901B configuration here
#endif

// =======================================================================
// Initialization
// =======================================================================
void DRIVERS_CONFIG_init_seq(DRIVERS_CONFIG_init_result_t *result) {
#if (APEX_ENABLE_ADXL345 == 1)
    ADXL375_Init(&ADXL375, &DRIVERS_CONFIG_ADXL375_SPI_HANDLE,
                 DRIVERS_CONFIG_ADXL375_CS_PORT, DRIVERS_CONFIG_ADXL375_CS_PIN);
#endif
#if (APEX_ENABLE_BMI088 == 1)
    result->bmi088_init_res = BMI088_Init(&bmi088, &DRIVERS_CONFIG_BMI088_SPI_HANDLE,
        DRIVERS_CONFIG_BMI088_ACC_CS_PORT, DRIVERS_CONFIG_BMI088_ACC_CS_PIN,
        DRIVERS_CONFIG_BMI088_GYR_CS_PORT, DRIVERS_CONFIG_BMI088_GYR_CS_PIN, &bmi088_config);
#endif
#if (APEX_ENABLE_BMP388 == 1)
    BMP388_Init(&bmp388);
#endif
#if (APEX_ENABLE_BUZZER == 1)
    BUZZER_Init(&buzzer, &DRIVERS_CONFIG_BUZZER_TIM_HANDLE, DRIVERS_CONFIG_BUZZER_CHANNEL);
#endif
#if (APEX_ENABLE_SX127X_1 == 1)
    result->sx127x_1_init_res = sx127x_Init(&sx127x_1, sx127x_base_config_1,
        sx127x_modulation_1, (sx127x_mod_config_t){.lora = sx127x_LORA_config});
#endif
#if (APEX_ENABLE_SX127X_2 == 1)
    result->sx127x_2_init_res = sx127x_Init(&sx127x_2, sx127x_base_config_2,
        sx127x_modulation_2, (sx127x_mod_config_t){ 0 });
#endif
#if (APEX_ENABLE_W25Q512 == 1)
    result->w25q_init_res = W25Q_Init(&w25q, &DRIVERS_CONFIG_W25Q512_SPI_HANDLE,
        DRIVERS_CONFIG_W25Q512_CS_PORT, DRIVERS_CONFIG_W25Q512_CS_PIN);
#endif
}
