#include "drivers_config.h"
#include "led.h"
#include "main.h"
#include "sx127x_common.h"
#include "tim.h"

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
TIM_HandleTypeDef * const DRIVERS_CONFIG_LED0_TIMER = &htim2;
const uint32_t DRIVERS_CONFIG_LED0_CHANNEL_RED = TIM_CHANNEL_3;
const uint32_t DRIVERS_CONFIG_LED0_CHANNEL_GREEN = TIM_CHANNEL_1;
const uint32_t DRIVERS_CONFIG_LED0_CHANNEL_BLUE = TIM_CHANNEL_2;
led_rgb_t led0_rgb;
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

const sx127x_LORA_config_t sx127x_LORA_config_1 = {
	.implicitHeader = false,
	.bandwidth = sx127x_LORA_REG_1D_MODEM_CONFIG1_BW_125KHZ,
	.codingRate = sx127x_LORA_REG_1D_MODEM_CONFIG1_CR_4_5,
	.spreadingFactor = sx127x_LORA_REG_1E_MODEM_CONFIG2_SF_128CPS,
	.crcEnabled = true,
};
const sx127x_FSK_OOK_config_t sx127x_FSK_OOK_config_1 = {
	.bitrate = 200000,
	.RxBw = SX127X_FSK_OOK_RxBw_125_0kHz,
	.modShaping = sx127x_FSK_OOK_REG_0A_PA_RAMP_MOD_SHAPING_FSK_BT_0_5,
	.paRamp = sx127x_FSK_OOK_REG_0A_PA_RAMP_PA_RAMP_40US,				// Default PA ramp time
	.packetCfg = {
		.preamble_len = 5,												// Default preamble length
		.sync_on = true,
		.sync_len = 4,													// Default sync length
		.sync_word = {0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01},	// Default sync word
		.variable_length = true,
		// .payload_len = 63,											// Default payload length
		.crc_on = true,
	},
	.mod_config.fsk = {
		.fdev = 60000,
	},
};
const sx127x_base_config_t sx127x_base_config_1 = {
	.spiHandle = &hspi1,
	.csPinBank = CS_LORA_GPIO_Port,
	.csPin = CS_LORA_Pin,
	.frequency = 869500000,
	.ocp_current_mA = 240.0,
	.power_dBm = 20.0,
};
const sx127x_modulation_t sx127x_modulation_1 = sx127x_MODULATION_FSK;

sx127x_t sx127x_1;

#endif

// =======================================================================
// SX127x 2 configuration
// =======================================================================
#if (APEX_ENABLE_SX127X_2 == 1)

const sx127x_LORA_config_t sx127x_LORA_config_2 = {
	.implicitHeader = false,
	.bandwidth = sx127x_LORA_REG_1D_MODEM_CONFIG1_BW_125KHZ,
	.codingRate = sx127x_LORA_REG_1D_MODEM_CONFIG1_CR_4_5,
	.spreadingFactor = sx127x_LORA_REG_1E_MODEM_CONFIG2_SF_128CPS,
	.crcEnabled = true,
};
const sx127x_FSK_OOK_config_t sx127x_FSK_OOK_config_2 = {
	.bitrate = 200000,
	.RxBw = SX127X_FSK_OOK_RxBw_125_0kHz,
	.modShaping = sx127x_FSK_OOK_REG_0A_PA_RAMP_MOD_SHAPING_FSK_BT_0_5,
	.paRamp = sx127x_FSK_OOK_REG_0A_PA_RAMP_PA_RAMP_40US,				// Default PA ramp time
	.packetCfg = {
		.preamble_len = 5,												// Default preamble length
		.sync_on = true,
		.sync_len = 4,													// Default sync length
		.sync_word = {0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01},	// Default sync word
		.variable_length = true,
		// .payload_len = 63,											// Default payload length
		.crc_on = true,
	},
	.mod_config.fsk = {
		.fdev = 60000,
	},
};
const sx127x_base_config_t sx127x_base_config_2 = {
	.spiHandle = &hspi3,
	.csPinBank = SPI3_CS_GPIO_Port,
	.csPin = SPI3_CS_Pin,
	.frequency = 869500000,
	.ocp_current_mA = 240.0,
	.power_dBm = 20.0,
};
const sx127x_modulation_t sx127x_modulation_2 = sx127x_MODULATION_FSK;

sx127x_t sx127x_2;

#endif

// =======================================================================
// UART Mux configuration
// =======================================================================
#if (APEX_ENABLE_UART_MUX == 1)
GPIO_TypeDef * const DRIVERS_CONFIG_UART_MUX_GPIO_PORT = UART_SEL_GPIO_Port;
const uint16_t DRIVERS_CONFIG_UART_MUX_GPIO_PIN = UART_SEL_Pin;
uart_mux_t uart_mux;
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
