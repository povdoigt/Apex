#ifndef SX127X_COMMON_H
#define SX127X_COMMON_H

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_spi.h"
#include <stdint.h>


// Max number of octets the LORA Rx/Tx FIFO can hold
#define sx127x_FIFO_SIZE 255

// The crystal oscillator frequency of the module (32 MHz)
#define sx127x_FXOSC 32000000

// The Frequency Synthesizer step = sx127x_FXOSC / 2^^19
#define sx127x_FSTEP  ((float)sx127x_FXOSC / (1 << 19))

// The Frequency range of the module (3 bands: 137MHz - 1020MHz)
// Available bands for sx1276/7/8, sx1279 is not supported
#define sx127x_BAND_3_MIN_FREQ  137000000
#define sx127x_BAND_3_MAX_FREQ  175000000
#define sx127x_BAND_2_MIN_FREQ  410000000
#define sx127x_BAND_2_MAX_FREQ  525000000
#define sx127x_BAND_1_MIN_FREQ  862000000
#define sx127x_BAND_1_MAX_FREQ 1020000000

typedef enum sx127x_band_t {
    sx127x_BAND_3 = 0,
    sx127x_BAND_2 = 1,
    sx127x_BAND_1 = 2,
} sx127x_band_t;

// OcpTRIM values for different Ocp currents
#define sx127x_OCP_IMAX_MIN    45
#define sx127x_OCP_IMAX_MAX   240

// Ouput power range
#define sx127x_OUTPUT_POWER_MIN    -4.2f
#define sx127x_OUTPUT_POWER_MAX    20.0f

// Register Write Not Read mask
#define sx127x_REG_WRITE_MSK	0b10000000


// Common Register names (FSK/OOK and LoRa Mode)  
#define sx127x_REG_00_FIFO			0x00
#define sx127x_REG_01_OP_MODE		0x01
// REG 02~05 are mode specific
#define sx127x_REG_06_FRF_MSB   	0x06
#define sx127x_REG_07_FRF_MID   	0x07
#define sx127x_REG_08_FRF_LSB   	0x08
#define sx127x_REG_09_PA_CONFIG   	0x09
// REG 0A are mode specific
#define sx127x_REG_0B_OCP       	0x0b
#define sx127x_REG_0C_LNA       	0x0c
// REG 0D~3F are mode specific
#define sx127x_REG_40_DIO_MAPPING1	0x40
#define sx127x_REG_41_DIO_MAPPING2	0x41
#define sx127x_REG_42_VERSION     	0x42
// REG 43 is reserved
// REG 44 is mode specific
// REG 45~4A are reserved
#define sx127x_REG_4B_TCXO       	0x4b
// REG 4C is reserved
#define sx127x_REG_4D_PA_DAC     	0x4d
// REG 4E~5A are reserved
#define sx127x_REG_5B_FORMER_TEMP	0x5b
// REG 5C is reserved
// REG 5D is mode specific
// 5E~60 are reserved
#define sx127x_REG_61_AGC_REF    	0x61
#define sx127x_REG_62_AGC_THRESH1	0x62
#define sx127x_REG_63_AGC_THRESH2	0x63
#define sx127x_REG_64_AGC_THRESH3	0x64
// REG 65~6F are reserved
#define sx127x_REG_70_PLL_HOP     	0x70





// ====================== 0x09 RegPaConfig ======================
// [7] PaSelect:
typedef enum sx127x_REG_09_PA_CONFIG_PA_SELECT {
	sx127x_REG_09_PA_CONFIG_PA_SELECT_RFO      = 0b00000000,	// Use RFO pin
	sx127x_REG_09_PA_CONFIG_PA_SELECT_PA_BOOST = 0b10000000,	// Use PA_BOOST pin
} sx127x_REG_09_PA_CONFIG_PA_SELECT;
#define sx127x_REG_09_PA_CONFIG_PA_SELECT_MSK 0b10000000
// [6-4] MaxPower: Output power setting (Pmax = 10.8 + 0.6 * MaxPower dBm) 
typedef enum sx127x_REG_09_PA_CONFIG_MAX_POWER {
    sx127x_REG_09_PA_CONFIG_MAX_POWER_0 = 0b00000000,   // Pmax = 10.8 dBm
    sx127x_REG_09_PA_CONFIG_MAX_POWER_1 = 0b00010000,   // Pmax = 11.4 dBm
    sx127x_REG_09_PA_CONFIG_MAX_POWER_2 = 0b00100000,   // Pmax = 12.0 dBm
    sx127x_REG_09_PA_CONFIG_MAX_POWER_3 = 0b00110000,   // Pmax = 12.6 dBm
    sx127x_REG_09_PA_CONFIG_MAX_POWER_4 = 0b01000000,   // Pmax = 13.2 dBm
    sx127x_REG_09_PA_CONFIG_MAX_POWER_5 = 0b01010000,   // Pmax = 13.8 dBm
    sx127x_REG_09_PA_CONFIG_MAX_POWER_6 = 0b01100000,   // Pmax = 14.4 dBm
    sx127x_REG_09_PA_CONFIG_MAX_POWER_7 = 0b01110000,   // Pmax = 15.0 dBm
} sx127x_REG_09_PA_CONFIG_MAX_POWER;
#define sx127x_REG_09_PA_CONFIG_MAX_POWER_MSK 0b01110000
// [3-0] OutputPower: Output power setting (Pout = Pmax - (15 - OutputPower) dBm PA=0 or Pout = 17 - (15 - OutputPower) dBm PA=1)
typedef enum sx127x_REG_09_PA_CONFIG_OUTPUT_POWER {
    sx127x_REG_09_PA_CONFIG_OUTPUT_POWER_0   = 0b00000000,   // Pout = Pmax - 15 dBm (PA=0) or  2 dBm (PA=1)
    sx127x_REG_09_PA_CONFIG_OUTPUT_POWER_1   = 0b00000001,   // Pout = Pmax - 14 dBm (PA=0) or  3 dBm (PA=1)
    sx127x_REG_09_PA_CONFIG_OUTPUT_POWER_2   = 0b00000010,   // Pout = Pmax - 13 dBm (PA=0) or  4 dBm (PA=1)
    sx127x_REG_09_PA_CONFIG_OUTPUT_POWER_3   = 0b00000011,   // Pout = Pmax - 12 dBm (PA=0) or  5 dBm (PA=1)
    sx127x_REG_09_PA_CONFIG_OUTPUT_POWER_4   = 0b00000100,   // Pout = Pmax - 11 dBm (PA=0) or  6 dBm (PA=1)
    sx127x_REG_09_PA_CONFIG_OUTPUT_POWER_5   = 0b00000101,   // Pout = Pmax - 10 dBm (PA=0) or  7 dBm (PA=1)
    sx127x_REG_09_PA_CONFIG_OUTPUT_POWER_6   = 0b00000110,   // Pout = Pmax -  9 dBm (PA=0) or  8 dBm (PA=1)
    sx127x_REG_09_PA_CONFIG_OUTPUT_POWER_7   = 0b00000111,   // Pout = Pmax -  8 dBm (PA=0) or  9 dBm (PA=1)
    sx127x_REG_09_PA_CONFIG_OUTPUT_POWER_8   = 0b00001000,   // Pout = Pmax -  7 dBm (PA=0) or 10 dBm (PA=1)
    sx127x_REG_09_PA_CONFIG_OUTPUT_POWER_9   = 0b00001001,   // Pout = Pmax -  6 dBm (PA=0) or 11 dBm (PA=1)
    sx127x_REG_09_PA_CONFIG_OUTPUT_POWER_10  = 0b00001010,   // Pout = Pmax -  5 dBm (PA=0) or 12 dBm (PA=1)
    sx127x_REG_09_PA_CONFIG_OUTPUT_POWER_11  = 0b00001011,   // Pout = Pmax -  4 dBm (PA=0) or 13 dBm (PA=1)
    sx127x_REG_09_PA_CONFIG_OUTPUT_POWER_12  = 0b00001100,   // Pout = Pmax -  3 dBm (PA=0) or 14 dBm (PA=1)
    sx127x_REG_09_PA_CONFIG_OUTPUT_POWER_13  = 0b00001101,   // Pout = Pmax -  2 dBm (PA=0) or 15 dBm (PA=1)
    sx127x_REG_09_PA_CONFIG_OUTPUT_POWER_14  = 0b00001110,   // Pout = Pmax -  1 dBm (PA=0) or 16 dBm (PA=1)
    sx127x_REG_09_PA_CONFIG_OUTPUT_POWER_15  = 0b00001111,   // Pout = Pmax      dBm (PA=0) or 17 dBm (PA=1)
} sx127x_REG_09_PA_CONFIG_OUTPUT_POWER;
#define sx127x_REG_09_PA_CONFIG_OUTPUT_POWER_MSK 0b00001111





// ====================== 0x0b RegOcp ======================
// [7-6] Unused
// [5] OcpOn:
typedef enum sx127x_REG_0B_OCP_OCP_ON {
	sx127x_REG_0B_OCP_OCP_ON_OFF	= 0b00000000,	// OCP disabled
	sx127x_REG_0B_OCP_OCP_ON_ON		= 0b00100000,	// OCP enabled
} sx127x_REG_0B_OCP_OCP_ON;
#define sx127x_REG_0B_OCP_OCP_ON_MSK 0b00100000
// [4-0] OcpTrim: OCP current trim setting
#define sx127x_REG_0B_OCP_OCP_TRIM_MSK 0b00011111





// ====================== 0x0c RegLna ======================
// [7-5] LnaGain:
typedef enum sx127x_REG_0C_LNA_LNA_GAIN {
	// 000 reserved
	sx127x_REG_0C_LNA_LNA_GAIN_G1	= 0b00100000,	// Gain set to G1 (highest)
	sx127x_REG_0C_LNA_LNA_GAIN_G2	= 0b01000000,	// Gain set to G2
	sx127x_REG_0C_LNA_LNA_GAIN_G3	= 0b01100000,	// Gain set to G3
	sx127x_REG_0C_LNA_LNA_GAIN_G4	= 0b10000000,	// Gain set to G4
	sx127x_REG_0C_LNA_LNA_GAIN_G5	= 0b10100000,	// Gain set to G5
	sx127x_REG_0C_LNA_LNA_GAIN_G6	= 0b11000000,	// Gain set to G6 (lowest)
	// 111 reserved
} sx127x_REG_0C_LNA_LNA_GAIN;
#define sx127x_REG_0C_LNA_LNA_GAIN_MSK 0b11100000
// [4-3] LnaBoostLf:
typedef enum sx127x_REG_0C_LNA_LNA_BOOST_LF {
	sx127x_REG_0C_LNA_LNA_BOOST_LF_DFT	= 0b00000000,	// Normal LNA current
	// 01 reserved
	// 10 reserved
	// 11 reserved
} sx127x_REG_0C_LNA_LNA_BOOST_LF;
// [2] Reserved
// [1-0] LnaBoostHf:
typedef enum sx127x_REG_0C_LNA_LNA_BOOST_HF {
	sx127x_REG_0C_LNA_LNA_BOOST_HF_DFT		= 0b00000000,	// Normal LNA current
	// 01 reserved
	// 10 reserved
	sx127x_REG_0C_LNA_LNA_BOOST_HF_BOOST	= 0b00000011,	// LNA current boosted by 150%
} sx127x_REG_0C_LNA_LNA_BOOST_HF;





// ====================== 0x40 RegDioMapping1 ======================
// [7-6] DIO0 Mapping:
#define sx127x_REG_40_DIO_MAPPING1_DIO0_MSK 0b11000000
// [5-4] DIO1 Mapping:
#define sx127x_REG_40_DIO_MAPPING1_DIO1_MSK 0b00110000
// [3-2] DIO2 Mapping:
#define sx127x_REG_40_DIO_MAPPING1_DIO2_MSK 0b00001100
// [1-0] DIO3 Mapping:
#define sx127x_REG_40_DIO_MAPPING1_DIO3_MSK 0b00000011





// ====================== 0x41 RegDioMapping2 ======================
// [7-6] DIO4 Mapping:
#define sx127x_REG_41_DIO_MAPPING2_DIO4_MSK 0b11000000
// [5-4] DIO5 Mapping:
#define sx127x_REG_41_DIO_MAPPING2_DIO5_MSK 0b00110000
// [3-1] Reserved
// [0] ClkOutOnPayloadReady:
typedef enum sx127x_REG_41_DIO_MAPPING2_MAP_PREAMBLE_DETECT {
	sx127x_REG_41_DIO_MAPPING2_MPD_RSSI				= 0b00000000,	// ClkOut on RSSI
	sx127x_REG_41_DIO_MAPPING2_MPD_PREAMBLE_DETECT	= 0b00000001,	// ClkOut on PreambleDetect
} sx127x_REG_41_DIO_MAPPING2_MAP_PREAMBLE_DETECT;
#define sx127x_REG_41_DIO_MAPPING2_MAP_PREAMBLE_DETECT_MSK 0b00000001






// ====================== 0x42 RegVersion ======================
#define sx127x_VERSION_ID	0x12





// ====================== 0x44 RegPllHop ======================
// [7] FastHopOn:
typedef enum sx127x_REG_70_PLL_HOP_FAST_HOP_ON {
	sx127x_REG_70_PLL_HOP_FAST_HOP_ON_OFF	= 0b00000000,	// Fast Hop disabled
	sx127x_REG_70_PLL_HOP_FAST_HOP_ON_ON	= 0b10000000,	// Fast Hop enabled
} sx127x_REG_70_PLL_HOP_FAST_HOP_ON;
#define sx127x_REG_70_PLL_HOP_FAST_HOP_ON_MSK 0b10000000
// [6-0] Reserved





// ====================== 0x4b RegTcxo ======================
// [7-5] Reserved
// [4] TcxoInputOn:
typedef enum sx127x_REG_4B_TCXO_TCXO_INPUT_ON {
	sx127x_REG_4B_TCXO_TCXO_INPUT_ON_OFF	= 0b00000000,	// TCXO input off
	sx127x_REG_4B_TCXO_TCXO_INPUT_ON_ON		= 0b00010000,	// TCXO input on
} sx127x_REG_4B_TCXO_TCXO_INPUT_ON; 
#define sx127x_REG_4B_TCXO_TCXO_INPUT_ON_MSK 0b00010000
// [3-0] Reserved





// ====================== 0x4d RegPaDac ======================
// [7-3] Reserved and must be set to 0b10000
#define sx127x_REG_4D_PA_DAC_DEFAULT 0b10000000
// [2-0] PaDac:
typedef enum sx127x_REG_4D_PA_DAC_PA_DAC {
	sx127x_REG_4D_PA_DAC_PA_DAC_DFT	= 0x04,	// Default value
	sx127x_REG_4D_PA_DAC_PA_DAC_HP  = 0x07,	// High power +20dBm
} sx127x_REG_4D_PA_DAC_PA_DAC;
#define sx127x_REG_4D_PA_DAC_PA_DAC_MSK 0b00000111
// [3-0] Reserved





// ====================== 0x61 RegAgcRef ======================
// [7-6] Unused
// [5-0] AgcReference:
#define sx127x_REG_61_AGC_REF_AGC_REFERENCE_MSK 0b00111111





// ====================== 0x62 RegAgcThresh1 ======================
// [7-5] Unused
// [4-0] AgcThresh1:
#define sx127x_REG_62_AGC_THRESH1_AGC_THRESH1_MSK 0b00011111





// ====================== 0x63 RegAgcThresh2 ======================
// [7-4] AgcThresh2:
#define sx127x_REG_63_AGC_THRESH2_AGC_THRESH2_MSK 0b11110000
// [3-0] Unused
#define sx127x_REG_63_AGC_THRESH2_AGC_THRESH3_MSK 0b00001111





// ====================== 0x64 RegAgcThresh3 ======================
// [7-4] AgcThresh2:
#define sx127x_REG_63_AGC_THRESH2_AGC_THRESH4_MSK 0b11110000
// [3-0] Unused
#define sx127x_REG_63_AGC_THRESH2_AGC_THRESH5_MSK 0b00001111





// ====================== 0x70 RegPllLf ======================
// [7-6] PllBandwidth:
typedef enum sx127x_REG_70_PLL_HOP_PLL_BANDWIDTH {
	sx127x_REG_70_PLL_HOP_PLL_BANDWIDTH_75KHZ   = 0b00000000,	// 75 kHz
	sx127x_REG_70_PLL_HOP_PLL_BANDWIDTH_150KHZ  = 0b01000000,	// 150 kHz
	sx127x_REG_70_PLL_HOP_PLL_BANDWIDTH_225KHZ  = 0b10000000,	// 225 kHz
	sx127x_REG_70_PLL_HOP_PLL_BANDWIDTH_300KHZ  = 0b11000000,	// 300 kHz
} sx127x_REG_70_PLL_HOP_PLL_BANDWIDTH;
// [5-0] Reserved




// ====================== Data Structures ======================

typedef enum sx127x_status_t {
    sx127x_STATUS_OK       = 0x00,
    sx127x_STATUS_ERROR    = 0x01,
} sx127x_status_t;

typedef enum sx127x_modulation_t {
	sx127x_MODULATION_FSK,
	sx127x_MODULATION_OOK,
	sx127x_MODULATION_LORA,
} sx127x_modulation_t;

typedef struct sx127x_chip_t {
	SPI_HandleTypeDef	*spiHandle;			// SPI Handle
	GPIO_TypeDef 	    *csPinBank;			// Chip Select Pin Bank
	uint16_t 		     csPin;				// Chip Select Pin
	uint32_t			 frequency;			// in Hz
	float				 ocp_current_mA;	// Over Current Protection current in mA
	float 				 power_dBm;			// Output power in dBm
	sx127x_modulation_t	 modulation;		// Current modulation mode
} sx127x_chip_t;








// ================================== Level 1: Basic Read/Write functions ==================================

sx127x_status_t sx127x_RegWrite(sx127x_chip_t *chip, uint8_t reg, uint8_t value);
sx127x_status_t sx127x_RegWriteMulti(sx127x_chip_t *chip, uint8_t reg, const uint8_t *buffer, uint8_t buf_size);
sx127x_status_t sx127x_RegRead(sx127x_chip_t *chip, uint8_t reg, uint8_t *value);
sx127x_status_t sx127x_RegReadMulti(sx127x_chip_t *chip, uint8_t reg, uint8_t *buffer, uint8_t buf_size);





// ================================== Level 2: High level function ==================================

sx127x_status_t sx127x_Init(sx127x_chip_t *chip, SPI_HandleTypeDef *spiHandle,
                            GPIO_TypeDef *csPinBank, uint16_t csPin, uint32_t frequency,
                            float ocp_current_mA, float power_dBm);
sx127x_status_t sx127x_GetBand(sx127x_chip_t *chip, uint8_t *band);
sx127x_status_t sx127x_SetFrequency(sx127x_chip_t *chip, uint32_t frequency);
sx127x_status_t sx127x_SetTxPower(sx127x_chip_t *chip, float power_dBm);
sx127x_status_t sx127x_SetOcp(sx127x_chip_t *chip, float ocp_current_mA);

#endif // SX127X_COMMON_H