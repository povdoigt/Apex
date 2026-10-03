#include "drivers_config.h"

// =======================================================================
// ADXL345 configuration
// =======================================================================
#if (APEX_ENABLE_ADXL345 == 1)

#define DRIVERS_CONFIG_ADXL375_SPI_HANDLE		hspi1
#define DRIVERS_CONFIG_ADXL375_CS_PIN			GPIO_PIN_3
#define DRIVERS_CONFIG_ADXL375_CS_PORT			GPIOA
adxl375_t ADXL375;

// ... add any additional configuration or definitions for the ADXL345 here

#endif


// =======================================================================
// BMI088 configuration
// =======================================================================
#if (APEX_ENABLE_BMI088 == 1)

#define DRIVERS_CONFIG_BMI088_SPI_HANDLE		hspi1
#define DRIVERS_CONFIG_BMI088_ACC_CS_PIN		GPIO_PIN_4
#define DRIVERS_CONFIG_BMI088_ACC_CS_PORT		GPIOA
#define DRIVERS_CONFIG_BMI088_GYR_CS_PIN		GPIO_PIN_2
#define DRIVERS_CONFIG_BMI088_GYR_CS_PORT		GPIOB
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

// ... add any additional configuration or definitions for the BMI088 here

#endif


// =======================================================================
// BMP388 configuration
// =======================================================================
#if (APEX_ENABLE_BMP388 == 1)

BMP388_HandleTypeDef bmp388;

// ... add any additional configuration or definitions for the BMP388 here

#endif


// =======================================================================
// Buzzer configuration
// =======================================================================
#if (APEX_ENABLE_BUZZER == 1)

#define DRIVERS_CONFIG_BUZZER_TIM_HANDLE		htim3
#define DRIVERS_CONFIG_BUZZER_CHANNEL			TIM_CHANNEL_4
buzzer_t buzzer;

// ... add any additional configuration or definitions for the buzzer here

#endif


// =======================================================================
// GPS configuration
// =======================================================================
#if (APEX_ENABLE_GPS == 1)

// ... add any additional configuration or definitions for the GPS here

#endif


// =======================================================================
// LED configuration
// =======================================================================
#if (APEX_ENABLE_LED == 1)

// ... add any additional configuration or definitions for the LEDs here

#endif


// =======================================================================
// LSM303AGR configuration
// =======================================================================
#if (APEX_ENABLE_LSM303AGR == 1)

extern lsm303agr_t lsm303agr;

#endif


// =======================================================================
// SX127x 1 configuration
// =======================================================================
#if (APEX_ENABLE_SX127X_1 == 1)

sx127x_LORA_config_t sx127x_LORA_config = {
	.implicitHeader = false,
	.bandwidth = sx127x_LORA_REG_1D_MODEM_CONFIG1_BW_125KHZ,
	.codingRate = sx127x_LORA_REG_1D_MODEM_CONFIG1_CR_4_5,
	.spreadingFactor = sx127x_LORA_REG_1E_MODEM_CONFIG2_SF_128CPS,
	.crcEnabled = true,
};
sx127x_FSK_OOK_config_t sx127x_FSK_OOK_config = {
	.bitrate = 200000,
	.RxBw = SX127X_FSK_OOK_RxBw_125_0kHz,
	.modShaping = sx127x_FSK_OOK_REG_0A_PA_RAMP_MOD_SHAPING_FSK_BT_0_5,
	.paRamp = sx127x_FSK_OOK_REG_0A_PA_RAMP_PA_RAMP_40US,
	.packetCfg = {
		.preamble_len = 5,
		.sync_on = true,
		.sync_len = 4,
		.sync_word = {0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01},
		.variable_length = true,
		.crc_on = true,
	},
	.mod_config.fsk = {
		.fdev = 60000,
	},
};
sx127x_base_config_t sx127x_base_config_1 = {
	.spiHandle = &hspi1,
	.csPinBank = CS_LORA_GPIO_Port,
	.csPin = CS_LORA_Pin,
	.frequency = 869500000,
	.ocp_current_mA = 240.0,
	.power_dBm = 20.0,
};
sx127x_modulation_t sx127x_modulation_1 = sx127x_MODULATION_LORA;

sx127x_t sx127x_1;

#endif


// =======================================================================
// SX127x 2 configuration
// =======================================================================
#if (APEX_ENABLE_SX127X_2 == 1)

sx127x_base_config_t sx127x_base_config_2 = { 0 };
sx127x_modulation_t sx127x_modulation_2 = sx127x_MODULATION_FSK;

sx127x_t sx127x_2;

#endif


// =======================================================================
// W25Q512 configuration
// =======================================================================
#if (APEX_ENABLE_W25Q512 == 1)

#define DRIVERS_CONFIG_W25Q512_SPI_HANDLE		hspi2
#define DRIVERS_CONFIG_W25Q512_CS_PIN			GPIO_PIN_1
#define DRIVERS_CONFIG_W25Q512_CS_PORT			GPIOC
W25Q_t w25q;

#endif


// =======================================================================
// WT901B configuration
// =======================================================================
#if (APEX_ENABLE_WT901B == 1)

#endif


// =======================================================================
// Initialization functions
// =======================================================================
void DRIVERS_CONFIG_init_seq(DRIVERS_CONFIG_init_result_t *result) {
#if (APEX_ENABLE_ADXL345 == 1)
	ADXL375_Init(&ADXL375, &DRIVERS_CONFIG_ADXL375_SPI_HANDLE, DRIVERS_CONFIG_ADXL375_CS_PORT,
		DRIVERS_CONFIG_ADXL375_CS_PIN);
#endif

#if (APEX_ENABLE_BMI088 == 1)
	result->bmi088_init_res = BMI088_Init(&bmi088, &DRIVERS_CONFIG_BMI088_SPI_HANDLE, DRIVERS_CONFIG_BMI088_ACC_CS_PORT,
		DRIVERS_CONFIG_BMI088_ACC_CS_PIN, DRIVERS_CONFIG_BMI088_GYR_CS_PORT,
		DRIVERS_CONFIG_BMI088_GYR_CS_PIN, &bmi088_config);
#endif

#if (APEX_ENABLE_BMP388 == 1)
	BMP388_Init(&bmp388);
#endif

#if (APEX_ENABLE_BUZZER == 1)
	BUZZER_Init(&buzzer, &DRIVERS_CONFIG_BUZZER_TIM_HANDLE, DRIVERS_CONFIG_BUZZER_CHANNEL);
#endif

#if (GPS_ENABLE_GPS == 1)
	// GPS initialization code here
#endif

#if (APEX_ENABLE_LED == 1)
	// LED initialization code here
#endif

#if (APEX_ENABLE_LSM303AGR == 1)
	// LSM303AGR initialization code here
#endif

#if (APEX_ENABLE_SX127X_1 == 1)
	switch (sx127x_modulation_1) {
		case sx127x_MODULATION_LORA:
			result->sx127x_1_init_res = sx127x_Init(&sx127x_1, sx127x_base_config_1, sx127x_modulation_1,
				(sx127x_mod_config_t){.lora = sx127x_LORA_config});
			break;
		case sx127x_MODULATION_FSK:
		case sx127x_MODULATION_OOK:
			result->sx127x_1_init_res = sx127x_Init(&sx127x_1, sx127x_base_config_1, sx127x_modulation_1,
				(sx127x_mod_config_t){.fsk_ook = sx127x_FSK_OOK_config});
			break;
		default:
			result->sx127x_1_init_res = sx127x_STATUS_ERROR;
	}
#endif

#if (APEX_ENABLE_SX127X_2 == 1)
	switch (sx127x_modulation_2) {
		case sx127x_MODULATION_LORA:
			result->sx127x_2_init_res = sx127x_Init(&sx127x_2, sx127x_base_config_2, sx127x_modulation_2,
				(sx127x_mod_config_t){.lora = sx127x_LORA_config});
			break;
		case sx127x_MODULATION_FSK:
		case sx127x_MODULATION_OOK:
			result->sx127x_2_init_res = sx127x_Init(&sx127x_2, sx127x_base_config_2, sx127x_modulation_2,
				(sx127x_mod_config_t){.fsk_ook = sx127x_FSK_OOK_config});
			break;
		default:
			result->sx127x_2_init_res = sx127x_STATUS_ERROR;
	}
#endif

#if (APEX_ENABLE_W25Q512 == 1)
	result->w25q_init_res = W25Q_Init(&w25q, &DRIVERS_CONFIG_W25Q512_SPI_HANDLE,
		DRIVERS_CONFIG_W25Q512_CS_PORT, DRIVERS_CONFIG_W25Q512_CS_PIN);
#endif

#if (APEX_ENABLE_WT901B == 1)
	// WT901B initialization code here
#endif
}
