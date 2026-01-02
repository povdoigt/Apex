#ifndef SX127X_FSK_OOK_H
#define SX127X_FSK_OOK_H
// include
#include "drivers/sx127x/sx127x_common.h"

#include <stdbool.h>


#define SX127X_FSK_OOK_BITRATE_MIN	  1200	// in bps
#define SX127X_FSK_OOK_BITRATE_MAX	300000	// in bps


#define SX127X_FSK_OOK_FIFO_SIZE		 64	// in bytes
#define SX127X_FSK_OOK_MAX_PAYLOAD_SIZE	255	// in bytes


// Register names (FSK/OOK Mode)
// REG 00~01 are common
#define sx127x_FSK_OOK_REG_02_BITRATE_MSB		0x02
#define sx127x_FSK_OOK_REG_03_BITRATE_LSB		0x03
#define sx127x_FSK_OOK_REG_04_FDEV_MSB			0x04
#define sx127x_FSK_OOK_REG_05_FDEV_LSB			0x05
// REG 06~09 are common
#define sx127x_FSK_OOK_REG_0A_PA_RAMP			0x0a
// REG 0B~0C are common
#define sx127x_FSK_OOK_REG_0D_RX_CONFIG			0x0d
#define sx127x_FSK_OOK_REG_0E_RSSI_CONFIG		0x0e
#define sx127x_FSK_OOK_REG_0F_RSSI_COLLISION	0x0f
#define sx127x_FSK_OOK_REG_10_RSSI_THRESH		0x10
#define sx127x_FSK_OOK_REG_11_RSSI_VALUE		0x11
#define sx127x_FSK_OOK_REG_12_RX_BW				0x12
#define sx127x_FSK_OOK_REG_13_AFC_BW			0x13
#define sx127x_FSK_OOK_REG_14_OOK_PEAK			0x14
#define sx127x_FSK_OOK_REG_15_OOK_FIX			0x15
#define sx127x_FSK_OOK_REG_16_OOK_AVG			0x16
// REG 17~19 are reserved
#define sx127x_FSK_OOK_REG_1A_AFC_FEI			0x1a
#define sx127x_FSK_OOK_REG_1B_AFC_MSB			0x1b
#define sx127x_FSK_OOK_REG_1C_AFC_LSB			0x1c
#define sx127x_FSK_OOK_REG_1D_FEI_MSB			0x1d
#define sx127x_FSK_OOK_REG_1E_FEI_LSB			0x1e
#define sx127x_FSK_OOK_REG_1F_PREAMBLE_DETECT	0x1f
#define sx127x_FSK_OOK_REG_20_RX_TIMEOUT_1		0x20
#define sx127x_FSK_OOK_REG_21_RX_TIMEOUT_2		0x21
#define sx127x_FSK_OOK_REG_22_RX_TIMEOUT_3		0x22
#define sx127x_FSK_OOK_REG_23_RX_DELAY			0x23
#define sx127x_FSK_OOK_REG_24_OSC				0x24
#define sx127x_FSK_OOK_REG_25_PREAMBLE_MSB		0x25
#define sx127x_FSK_OOK_REG_26_PREAMBLE_LSB		0x26
#define sx127x_FSK_OOK_REG_27_SYNC_CONFIG		0x27
#define sx127x_FSK_OOK_REG_28_SYNC_VALUE_1		0x28
#define sx127x_FSK_OOK_REG_29_SYNC_VALUE_2		0x29
#define sx127x_FSK_OOK_REG_2A_SYNC_VALUE_3		0x2a
#define sx127x_FSK_OOK_REG_2B_SYNC_VALUE_4		0x2b
#define sx127x_FSK_OOK_REG_2C_SYNC_VALUE_5		0x2c
#define sx127x_FSK_OOK_REG_2D_SYNC_VALUE_6		0x2d
#define sx127x_FSK_OOK_REG_2E_SYNC_VALUE_7		0x2e
#define sx127x_FSK_OOK_REG_2F_SYNC_VALUE_8		0x2f
#define sx127x_FSK_OOK_REG_30_PACKET_CONFIG1	0x30
#define sx127x_FSK_OOK_REG_31_PACKET_CONFIG2	0x31
#define sx127x_FSK_OOK_REG_32_PAYLOAD_LENGTH	0x32
#define sx127x_FSK_OOK_REG_33_NODE_ADRS			0x33
#define sx127x_FSK_OOK_REG_34_BROADCAST_ADRS	0x34
#define sx127x_FSK_OOK_REG_35_FIFO_THRESH		0x35
#define sx127x_FSK_OOK_REG_36_SEQ_CONFIG1		0x36
#define sx127x_FSK_OOK_REG_37_SEQ_CONFIG2		0x37
#define sx127x_FSK_OOK_REG_38_TIMER_RESOL		0x38
#define sx127x_FSK_OOK_REG_39_TIMER1_COEF		0x39
#define sx127x_FSK_OOK_REG_3A_TIMER2_COEF		0x3a
#define sx127x_FSK_OOK_REG_3B_IMAGE_CAL			0x3b
#define sx127x_FSK_OOK_REG_3C_TEMP				0x3c
#define sx127x_FSK_OOK_REG_3D_LOW_BATTERY		0x3d
#define sx127x_FSK_OOK_REG_3E_IRQ_FLAGS1		0x3e
#define sx127x_FSK_OOK_REG_3F_IRQ_FLAGS2		0x3f
// REG 40~42 are common
// REG 43 is reserved
#define sx127x_FSK_OOK_REG_44_PLL_HOP			0x44
// REG 45~4A are reserved
// REG 4B is common
// REG 4C is reserved
// REG 4D is common
// REG 4E~5A are reserved
// REG 5B is common
// REG 5C is reserved
#define sx127x_FSK_OOK_REG_5D_BIT_RATE_FRAC		0x5d
// REG 5E~60 are reserved
// REG 61~64 are common
// REG 65~6F are reserved
// REG 70 is common





// ====================== 0x01 RegOpMode ======================
// [7] LongRangeMode:
typedef enum sx127x_FSK_OOK_REG_01_OP_MODE_LRM {
    sx127x_FSK_OOK_REG_01_OP_MODE_LRM_FSK_OOK   = 0b00000000,   // FSK/OOK mode
    sx127x_FSK_OOK_REG_01_OP_MODE_LRM_LoRa      = 0b10000000,   // LoRa mode
} sx127x_FSK_OOK_REG_01_OP_MODE_LRM;
#define sx127x_FSK_OOK_REG_01_OP_MODE_LRM_MSK 0b10000000
// [6-5] ModulationType:
typedef enum sx127x_FSK_OOK_REG_01_OP_MODE_MOD_TYPE {
    sx127x_FSK_OOK_REG_01_OP_MODE_MOD_TYPE_FSK  = 0b00000000,   // FSK
    sx127x_FSK_OOK_REG_01_OP_MODE_MOD_TYPE_OOK  = 0b00100000,   // OOK
    // 10 reserved
    // 11 reserved 
} sx127x_FSK_OOK_REG_01_OP_MODE_MOD_TYPE;
#define sx127x_FSK_OOK_REG_01_OP_MODE_MOD_TYPE_MSK 0b01100000
// [4] Reserved
// [3] LowFrequencyModeOn:
typedef enum sx127x_FSK_OOK_REG_01_OP_MODE_LFMO {
    sx127x_FSK_OOK_REG_01_OP_MODE_LFMO_OFF  = 0b00000000,   // HF mode (> 525MHz)
    sx127x_FSK_OOK_REG_01_OP_MODE_LFMO_ON   = 0b00001000,   // LF mode (<= 525MHz)
} sx127x_FSK_OOK_REG_01_OP_MODE_LFMO;
#define sx127x_FSK_OOK_REG_01_OP_MODE_LFMO_MSK 0b00001000
// [2-0] Mode:
typedef enum sx127x_FSK_OOK_REG_01_OP_MODE_MODE {
    sx127x_FSK_OOK_REG_01_OP_MODE_MODE_SLEEP    = 0b00000000,   // Sleep
    sx127x_FSK_OOK_REG_01_OP_MODE_MODE_STDBY    = 0b00000001,   // Standby
    sx127x_FSK_OOK_REG_01_OP_MODE_MODE_FSTX     = 0b00000010,   // FSTX
    sx127x_FSK_OOK_REG_01_OP_MODE_MODE_TX       = 0b00000011,   // Tx
    sx127x_FSK_OOK_REG_01_OP_MODE_MODE_FSRX     = 0b00000100,   // FSRX
    sx127x_FSK_OOK_REG_01_OP_MODE_MODE_RX       = 0b00000101,   // Rx
    // 110 reserved
    // 111 reserved
} sx127x_FSK_OOK_REG_01_OP_MODE_MODE;
#define sx127x_FSK_OOK_REG_01_OP_MODE_MODE_MSK 0b00000111





// ====================== 0x0a RegPaRamp ======================
// [7] Unused
// [6-5] ModulationShaping:
typedef enum sx127x_FSK_OOK_REG_0A_PA_RAMP_MOD_SHAPING {
    sx127x_FSK_OOK_REG_0A_PA_RAMP_MOD_SHAPING_OFF           = 0b00000000,   // No shaping
    sx127x_FSK_OOK_REG_0A_PA_RAMP_MOD_SHAPING_FSK_BT_1_0    = 0b00100000,   // FSK, BT = 1.0
    sx127x_FSK_OOK_REG_0A_PA_RAMP_MOD_SHAPING_FSK_BT_0_5    = 0b01000000,   // FSK, BT = 0.5
    sx127x_FSK_OOK_REG_0A_PA_RAMP_MOD_SHAPING_FSK_BT_0_3    = 0b01100000,   // FSK, BT = 0.3
} sx127x_FSK_OOK_REG_0A_PA_RAMP_MOD_SHAPING;
// For OOK modulation
#define sx127x_FSK_OOK_REG_0A_PA_RAMP_MOD_SHAPING_OOK_FC    sx127x_FSK_OOK_REG_0A_PA_RAMP_MOD_SHAPING_FSK_BT_1_0    // OOK, fc = bit rate
#define sx127x_FSK_OOK_REG_0A_PA_RAMP_MOD_SHAPING_OOK_FC_2  sx127x_FSK_OOK_REG_0A_PA_RAMP_MOD_SHAPING_FSK_BT_0_5    // OOK, fc = 2*bit rate
#define sx127x_FSK_OOK_REG_0A_PA_RAMP_MOD_SHAPING_MSK 0b01100000
// [4] Reserved
// [3-0] PA Ramp:
typedef enum sx127x_FSK_OOK_REG_0A_PA_RAMP_PA_RAMP {
    sx127x_FSK_OOK_REG_0A_PA_RAMP_PA_RAMP_3_4MS    = 0b00000000,
    sx127x_FSK_OOK_REG_0A_PA_RAMP_PA_RAMP_2MS      = 0b00000001,
    sx127x_FSK_OOK_REG_0A_PA_RAMP_PA_RAMP_1MS      = 0b00000010,
    sx127x_FSK_OOK_REG_0A_PA_RAMP_PA_RAMP_500US    = 0b00000011,
    sx127x_FSK_OOK_REG_0A_PA_RAMP_PA_RAMP_250US    = 0b00000100,
    sx127x_FSK_OOK_REG_0A_PA_RAMP_PA_RAMP_125US    = 0b00000101,
    sx127x_FSK_OOK_REG_0A_PA_RAMP_PA_RAMP_100US    = 0b00000110,
    sx127x_FSK_OOK_REG_0A_PA_RAMP_PA_RAMP_62US     = 0b00000111,
    sx127x_FSK_OOK_REG_0A_PA_RAMP_PA_RAMP_50US     = 0b00001000,
    sx127x_FSK_OOK_REG_0A_PA_RAMP_PA_RAMP_40US     = 0b00001001,
    sx127x_FSK_OOK_REG_0A_PA_RAMP_PA_RAMP_31US     = 0b00001010,
    sx127x_FSK_OOK_REG_0A_PA_RAMP_PA_RAMP_25US     = 0b00001011,
    sx127x_FSK_OOK_REG_0A_PA_RAMP_PA_RAMP_20US     = 0b00001100,
    sx127x_FSK_OOK_REG_0A_PA_RAMP_PA_RAMP_15US     = 0b00001101,
    sx127x_FSK_OOK_REG_0A_PA_RAMP_PA_RAMP_12US     = 0b00001110,
    sx127x_FSK_OOK_REG_0A_PA_RAMP_PA_RAMP_10US     = 0b00001111,
} sx127x_FSK_OOK_REG_0A_PA_RAMP_PA_RAMP;
#define sx127x_FSK_OOK_REG_0A_PA_RAMP_PA_RAMP_MSK 0b00001111





// ====================== 0x0d RxConfig ======================
// [7] RestartRxOnCollision:
typedef enum sx127x_FSK_OOK_REG_0D_RX_CONFIG_RROC {
	sx127x_FSK_OOK_REG_0D_RX_CONFIG_RROC_OFF    = 0b00000000,   // No auto restart
	sx127x_FSK_OOK_REG_0D_RX_CONFIG_RROC_ON     = 0b10000000,   // Auto restart
} sx127x_FSK_OOK_REG_0D_RX_CONFIG_RROC;
#define sx127x_FSK_OOK_REG_0D_RX_CONFIG_RROC_MSK 0b10000000
// [6] RestartRxWithoutPllLock: (wt: write trigger)
#define sx127x_FSK_OOK_REG_0D_RX_CONFIG_RROWithoutP_MSK 0b01000000
// [5] RestartRxWithPllLock: (wt: write trigger)
#define sx127x_FSK_OOK_REG_0D_RX_CONFIG_RROWithP_MSK 0b00100000
// [4] AfcAutoOn:
typedef enum sx127x_FSK_OOK_REG_0D_RX_CONFIG_AFCAON {
	sx127x_FSK_OOK_REG_0D_RX_CONFIG_AFCAON_OFF    = 0b00000000,   // AFC disabled
	sx127x_FSK_OOK_REG_0D_RX_CONFIG_AFCAON_ON     = 0b00010000,   // AFC enabled
} sx127x_FSK_OOK_REG_0D_RX_CONFIG_AFCAON;
#define sx127x_FSK_OOK_REG_0D_RX_CONFIG_AFCAON_MSK 0b00010000
// [3] AgcAutoOn:
typedef enum sx127x_FSK_OOK_REG_0D_RX_CONFIG_AGCAON {
	sx127x_FSK_OOK_REG_0D_RX_CONFIG_AGCAON_OFF   = 0b00000000,   // LNA gain set by RegLna
	sx127x_FSK_OOK_REG_0D_RX_CONFIG_AGCAON_ON    = 0b00001000,   // LNA gain set by AGC
} sx127x_FSK_OOK_REG_0D_RX_CONFIG_AGCAON;
#define sx127x_FSK_OOK_REG_0D_RX_CONFIG_AGCAON_MSK 0b00001000
// [2-0] RxTrigger: (see table 24 of the datasheet)
#define sx127x_FSK_OOK_REG_0D_RX_CONFIG_RX_TRIGGER_MSK 0b00000111





// ====================== 0x0e RssiConfig ======================
// [7-3] RssiOffset:
#define sx127x_FSK_OOK_REG_0E_RSSI_CONFIG_RSSI_OFFSET_MSK 0b11111000
// [2-0] RssiSmoothing:
typedef enum sx127x_FSK_OOK_REG_0E_RSSI_CONFIG_RSSI_SMOOTHING {
	sx127x_FSK_OOK_REG_0E_RSSI_CONFIG_RSSI_SMOOTHING_2		= 0b00000000,	// Smoothing over   2 samples
	sx127x_FSK_OOK_REG_0E_RSSI_CONFIG_RSSI_SMOOTHING_4		= 0b00000001,	// Smoothing over   4 samples
	sx127x_FSK_OOK_REG_0E_RSSI_CONFIG_RSSI_SMOOTHING_8		= 0b00000010,	// Smoothing over   8 samples
	sx127x_FSK_OOK_REG_0E_RSSI_CONFIG_RSSI_SMOOTHING_16		= 0b00000011,	// Smoothing over  16 samples
	sx127x_FSK_OOK_REG_0E_RSSI_CONFIG_RSSI_SMOOTHING_32		= 0b00000100,	// Smoothing over  32 samples
	sx127x_FSK_OOK_REG_0E_RSSI_CONFIG_RSSI_SMOOTHING_64		= 0b00000101,	// Smoothing over  64 samples
	sx127x_FSK_OOK_REG_0E_RSSI_CONFIG_RSSI_SMOOTHING_128	= 0b00000110,	// Smoothing over 128 samples
	sx127x_FSK_OOK_REG_0E_RSSI_CONFIG_RSSI_SMOOTHING_256	= 0b00000111,	// Smoothing over 256 samples
} sx127x_FSK_OOK_REG_0E_RSSI_CONFIG_RSSI_SMOOTHING;
#define sx127x_FSK_OOK_REG_0E_RSSI_CONFIG_RSSI_SMOOTHING_MSK 0b00000111





// ====================== 0x12 RxBw ======================
// [7] Unused
// [6-5] Reserved
// [4-3] RxBwMant:
typedef enum sx127x_FSK_OOK_REG_12_RX_BW_MANT {
	sx127x_FSK_OOK_REG_12_RX_BW_MANT_16    = 0b00000000,
	sx127x_FSK_OOK_REG_12_RX_BW_MANT_20    = 0b00001000,
	sx127x_FSK_OOK_REG_12_RX_BW_MANT_24    = 0b00010000,
	// 11 reserved
} sx127x_FSK_OOK_REG_12_RX_BW_MANT;
#define sx127x_FSK_OOK_REG_12_RX_BW_MANT_MSK 0b00011000
// [2-0] RxBwExp:
typedef enum sx127x_FSK_OOK_REG_12_RX_BW_EXP {
	sx127x_FSK_OOK_REG_12_RX_BW_EXP_0    = 0b00000000,
	sx127x_FSK_OOK_REG_12_RX_BW_EXP_1    = 0b00000001,
	sx127x_FSK_OOK_REG_12_RX_BW_EXP_2    = 0b00000010,
	sx127x_FSK_OOK_REG_12_RX_BW_EXP_3    = 0b00000011,
	sx127x_FSK_OOK_REG_12_RX_BW_EXP_4    = 0b00000100,
	sx127x_FSK_OOK_REG_12_RX_BW_EXP_5    = 0b00000101,
	sx127x_FSK_OOK_REG_12_RX_BW_EXP_6    = 0b00000110,
	sx127x_FSK_OOK_REG_12_RX_BW_EXP_7    = 0b00000111,
} sx127x_FSK_OOK_REG_12_RX_BW_EXP;
#define sx127x_FSK_OOK_REG_12_RX_BW_EXP_MSK 0b00000111





// ====================== 0x13 AfcBw ======================
// [7-5] Reserved
// [4-3] RxBwMantAfc:
#define sx127x_FSK_OOK_REG_13_AFC_BW_RX_BW_MANT_AFC_MSK 0b00011000
// [2-0] RxBwExpAfc:
#define sx127x_FSK_OOK_REG_13_AFC_BW_RX_BW_EXP_AFC_MSK 0b00000111





// ====================== 0x14 OokPeak ======================
// [7-6] Reserved
// [5] BitSyncOn:
typedef enum sx127x_FSK_OOK_REG_14_OOK_PEAK_BIT_SYNC_ON {
	sx127x_FSK_OOK_REG_14_OOK_PEAK_BIT_SYNC_ON_OFF    = 0b00000000,   // BitSync disabled (not possible in packet mode)
	sx127x_FSK_OOK_REG_14_OOK_PEAK_BIT_SYNC_ON_ON     = 0b00100000,   // BitSync enabled
} sx127x_FSK_OOK_REG_14_OOK_PEAK_BIT_SYNC_ON;
#define sx127x_FSK_OOK_REG_14_OOK_PEAK_BIT_SYNC_ON_MSK 0b00100000
// [4-3] OokPeakThreshType:
typedef enum sx127x_FSK_OOK_REG_14_OOK_PEAK_THRESH_TYPE {
	sx127x_FSK_OOK_REG_14_OOK_PEAK_THRESH_TYPE_FIXED	= 0b00000000,
	sx127x_FSK_OOK_REG_14_OOK_PEAK_THRESH_TYPE_PEAK		= 0b00001000,
	sx127x_FSK_OOK_REG_14_OOK_PEAK_THRESH_TYPE_AVG		= 0b00010000,
	// 11 reserved
} sx127x_FSK_OOK_REG_14_OOK_PEAK_THRESH_TYPE;
#define sx127x_FSK_OOK_REG_14_OOK_PEAK_THRESH_TYPE_MSK 0b00011000
// [2-0] OokPeakThreshStep:
typedef enum sx127x_FSK_OOK_REG_14_OOK_PEAK_THRESH_STEP {
	sx127x_FSK_OOK_REG_14_OOK_PEAK_THRESH_STEP_0_5	= 0b00000000,	// 0.5 dB
	sx127x_FSK_OOK_REG_14_OOK_PEAK_THRESH_STEP_1_0	= 0b00000001,	// 1.0 dB
	sx127x_FSK_OOK_REG_14_OOK_PEAK_THRESH_STEP_1_5	= 0b00000010,	// 1.5 dB
	sx127x_FSK_OOK_REG_14_OOK_PEAK_THRESH_STEP_2_0	= 0b00000011,	// 2.0 dB
	sx127x_FSK_OOK_REG_14_OOK_PEAK_THRESH_STEP_3_0	= 0b00000100,	// 3.0 dB
	sx127x_FSK_OOK_REG_14_OOK_PEAK_THRESH_STEP_4_0	= 0b00000101,	// 4.0 dB
	sx127x_FSK_OOK_REG_14_OOK_PEAK_THRESH_STEP_5_0	= 0b00000110,	// 5.0 dB
	sx127x_FSK_OOK_REG_14_OOK_PEAK_THRESH_STEP_6_0	= 0b00000111,	// 6.0 dB
} sx127x_FSK_OOK_REG_14_OOK_PEAK_THRESH_STEP;
#define sx127x_FSK_OOK_REG_14_OOK_PEAK_THRESH_STEP_MSK 0b00000111





// ====================== 0x16 OokAvg ======================
// [7-5] OokPeakThreshDec:
typedef enum sx127x_FSK_OOK_REG_16_OOK_AVG_PEAK_THRESH_DEC {
	sx127x_FSK_OOK_REG_16_OOK_AVG_PEAK_THRESH_DEC_1_1	= 0b00000000,	// once per chip
	sx127x_FSK_OOK_REG_16_OOK_AVG_PEAK_THRESH_DEC_1_2	= 0b00100000,	// once every 2 chips
	sx127x_FSK_OOK_REG_16_OOK_AVG_PEAK_THRESH_DEC_1_4	= 0b01000000,	// once every 4 chips
	sx127x_FSK_OOK_REG_16_OOK_AVG_PEAK_THRESH_DEC_1_8	= 0b01100000,	// once every 8 chips
	sx127x_FSK_OOK_REG_16_OOK_AVG_PEAK_THRESH_DEC_2_0	= 0b10000000,	// twice in each chip
	sx127x_FSK_OOK_REG_16_OOK_AVG_PEAK_THRESH_DEC_4_0	= 0b10100000,	// 4 times in each chip
	sx127x_FSK_OOK_REG_16_OOK_AVG_PEAK_THRESH_DEC_8_0	= 0b11000000,	// 8 times in each chip
	sx127x_FSK_OOK_REG_16_OOK_AVG_PEAK_THRESH_DEC_16_0	= 0b11100000,	// 16 times in each chip
} sx127x_FSK_OOK_REG_16_OOK_AVG_PEAK_THRESH_DEC;
#define sx127x_FSK_OOK_REG_16_OOK_AVG_PEAK_THRESH_DEC_MSK 0b11100000
// [4] Reserved
// [3-2] OokAvgOffset:
typedef enum sx127x_FSK_OOK_REG_16_OOK_AVG_OFFSET {
	sx127x_FSK_OOK_REG_16_OOK_AVG_OFFSET_0	= 0b00000000,	// 0.0 dB
	sx127x_FSK_OOK_REG_16_OOK_AVG_OFFSET_2	= 0b00000100,	// 2.0 dB
	sx127x_FSK_OOK_REG_16_OOK_AVG_OFFSET_4	= 0b00001000,	// 4.0 dB
	sx127x_FSK_OOK_REG_16_OOK_AVG_OFFSET_6	= 0b00001100,	// 6.0 dB
} sx127x_FSK_OOK_REG_16_OOK_AVG_OFFSET;
#define sx127x_FSK_OOK_REG_16_OOK_AVG_OFFSET_MSK 0b00001100
// [1-0] OokAvgThreshFilt:
typedef enum sx127x_FSK_OOK_REG_16_OOK_AVG_THRESH_FILT {
	sx127x_FSK_OOK_REG_16_OOK_AVG_THRESH_FILT_32	= 0b00000000,	// fc = chip rate / (32 * pi)
	sx127x_FSK_OOK_REG_16_OOK_AVG_THRESH_FILT_8		= 0b00000001,	// fc = chip rate / (8  * pi)
	sx127x_FSK_OOK_REG_16_OOK_AVG_THRESH_FILT_4		= 0b00000010,	// fc = chip rate / (4  * pi)
	sx127x_FSK_OOK_REG_16_OOK_AVG_THRESH_FILT_2		= 0b00000011,	// fc = chip rate / (2  * pi)
} sx127x_FSK_OOK_REG_16_OOK_AVG_THRESH_FILT;
#define sx127x_FSK_OOK_REG_16_OOK_AVG_THRESH_FILT_MSK 0b00000011





// ====================== 0x1a AfcFei ======================
// [7-5] Unused
// [4] AgcStart: (wt: write trigger)
#define sx127x_FSK_OOK_REG_1A_AFC_FEI_AGC_START_MSK 0b00010000
// [3] Reserved
// [2] Unused
// [1] AfcClear: (wc: write clear)
#define sx127x_FSK_OOK_REG_1A_AFC_FEI_AFC_CLEAR_MSK 0b00000010
// [0] AfcAutoClearOn:
typedef enum sx127x_FSK_OOK_REG_1A_AFC_FEI_AFC_AUTO_CLEAR_ON {
	sx127x_FSK_OOK_REG_1A_AFC_FEI_AFC_AUTO_CLEAR_ON_OFF    = 0b00000000,   // AFC value not cleared after read
	sx127x_FSK_OOK_REG_1A_AFC_FEI_AFC_AUTO_CLEAR_ON_ON     = 0b00000001,   // AFC value cleared after read
} sx127x_FSK_OOK_REG_1A_AFC_FEI_AFC_AUTO_CLEAR_ON;
#define sx127x_FSK_OOK_REG_1A_AFC_FEI_AFC_AUTO_CLEAR_ON_MSK 0b00000001





// ====================== 0x1f PreambleDetect ======================
// [7] PreambleDetectorOn:
typedef enum sx127x_FSK_OOK_REG_1F_PREAMBLE_DETECT_PDON {
	sx127x_FSK_OOK_REG_1F_PREAMBLE_DETECT_PDON_OFF    = 0b00000000,   // Preamble detector disabled
	sx127x_FSK_OOK_REG_1F_PREAMBLE_DETECT_PDON_ON     = 0b10000000,   // Preamble detector enabled
} sx127x_FSK_OOK_REG_1F_PREAMBLE_DETECT_PDON;
#define sx127x_FSK_OOK_REG_1F_PREAMBLE_DETECT_PDON_MSK 0b10000000
// [6-5] PreambleDetectorSize:
typedef enum sx127x_FSK_OOK_REG_1F_PREAMBLE_DETECT_PDSIZE {
	sx127x_FSK_OOK_REG_1F_PREAMBLE_DETECT_PDSIZE_1	= 0b00000000,	// 1 byte
	sx127x_FSK_OOK_REG_1F_PREAMBLE_DETECT_PDSIZE_2	= 0b00100000,	// 2 bytes
	sx127x_FSK_OOK_REG_1F_PREAMBLE_DETECT_PDSIZE_3	= 0b01000000,	// 3 bytes
	// 11 reserved
} sx127x_FSK_OOK_REG_1F_PREAMBLE_DETECT_PDSIZE;
#define sx127x_FSK_OOK_REG_1F_PREAMBLE_DETECT_PDSIZE_MSK 0b01100000
// [4-0] PreambleDetectorTol:
#define sx127x_FSK_OOK_REG_1F_PREAMBLE_DETECT_PDTOL_MSK 0b00011111





// ====================== 0x24 RegOsc ======================
// [7-4] Unused
// [3] RcCalStart: (wt: write trigger)
#define sx127x_FSK_OOK_REG_24_OSC_RC_CAL_START_MSK 0b00001000
// [2-0] ClkOut:
typedef enum sx127x_FSK_OOK_REG_24_OSC_CLK_OUT {
	sx127x_FSK_OOK_REG_24_OSC_CLK_OUT_1		= 0b00000000,	// fXosc / 1
	sx127x_FSK_OOK_REG_24_OSC_CLK_OUT_2		= 0b00000001,	// fXosc / 2
	sx127x_FSK_OOK_REG_24_OSC_CLK_OUT_4		= 0b00000010,	// fXosc / 4
	sx127x_FSK_OOK_REG_24_OSC_CLK_OUT_8		= 0b00000011,	// fXosc / 8
	sx127x_FSK_OOK_REG_24_OSC_CLK_OUT_16	= 0b00000100,	// fXosc / 16
	sx127x_FSK_OOK_REG_24_OSC_CLK_OUT_32	= 0b00000101,	// fXosc / 32
	sx127x_FSK_OOK_REG_24_OSC_CLK_OUT_RC	= 0b00000110,	// RC oscillator
	sx127x_FSK_OOK_REG_24_OSC_CLK_OUT_OFF	= 0b00000111,	// Off
} sx127x_FSK_OOK_REG_24_OSC_CLK_OUT;
#define sx127x_FSK_OOK_REG_24_OSC_CLK_OUT_MSK 0b00000111





// ====================== 0x27 SyncConfig ======================
// [7-6] AutoRestartRxMode:
typedef enum sx127x_FSK_OOK_REG_27_SYNC_CONFIG_ARRXM {
	sx127x_FSK_OOK_REG_27_SYNC_CONFIG_ARRXM_OFF		= 0b00000000,   // Dsiable
	sx127x_FSK_OOK_REG_27_SYNC_CONFIG_ARRXM_ON		= 0b01000000,   // Enable without waiting Pll to re-lock
	sx127x_FSK_OOK_REG_27_SYNC_CONFIG_ARRXM_WAIT	= 0b10000000,   // Enable, waiting Pll to re-lock Pll
	// 11 reserved
} sx127x_FSK_OOK_REG_27_SYNC_CONFIG_ARRXM;
#define sx127x_FSK_OOK_REG_27_SYNC_CONFIG_ARRXM_MSK 0b11000000
// [5] PreamblePolarity:
typedef enum sx127x_FSK_OOK_REG_27_SYNC_CONFIG_PP {
	sx127x_FSK_OOK_REG_27_SYNC_CONFIG_PP_AA	= 0b00000000,   // 0xAA
	sx127x_FSK_OOK_REG_27_SYNC_CONFIG_PP_55	= 0b00100000,   // 0x55
} sx127x_FSK_OOK_REG_27_SYNC_CONFIG_PP;
#define sx127x_FSK_OOK_REG_27_SYNC_CONFIG_PP_MSK 0b00100000
// [4] SyncOn:
typedef enum sx127x_FSK_OOK_REG_27_SYNC_CONFIG_SO {
	sx127x_FSK_OOK_REG_27_SYNC_CONFIG_SO_OFF    = 0b00000000,   // Sync word detector disabled
	sx127x_FSK_OOK_REG_27_SYNC_CONFIG_SO_ON     = 0b00010000,   // Sync word detector enabled
} sx127x_FSK_OOK_REG_27_SYNC_CONFIG_SO;
#define sx127x_FSK_OOK_REG_27_SYNC_CONFIG_SO_MSK 0b00010000
// [3] Reserved
// [2-0] SyncSize: (size = SyncSize + 1, size = SyncSize if ioHomeOn = 1)
#define sx127x_FSK_OOK_REG_27_SYNC_CONFIG_SYNC_SIZE_MSK 0b00000111





// ====================== 0x30 PacketConfig1 ======================
// [7] PacketFormat:
typedef enum sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_PF {
	sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_PF_FIXED    = 0b00000000,   // Fixed length packets
	sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_PF_VARIABLE = 0b10000000,   // Variable length packets
} sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_PF;
#define sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_PF_MSK 0b10000000
// [6-5] DcFree:
typedef enum sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_DCFREE {
	sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_DCFREE_OFF			= 0b00000000,   // No DC-free encoding
	sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_DCFREE_MANCHESTER	= 0b00100000,   // Manchester encoding
	sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_DCFREE_WHITENING	= 0b01000000,   // Whitening
	// 11 reserved
} sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_DCFREE;
#define sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_DCFREE_MSK 0b01100000
// [4] CrcOn:
typedef enum sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_CRCON {
	sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_CRCON_OFF    = 0b00000000,   // CRC disabled
	sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_CRCON_ON     = 0b00010000,   // CRC enabled
} sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_CRCON;
#define sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_CRCON_MSK 0b00010000
// [3] CrcAutoClearOff:
typedef enum sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_CRCAOFF {
	sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_CRCAOFF_CLEAR		= 0b00000000,   // FIFO is cleared if CRC error
	sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_CRCAOFF_NO_CLEAR	= 0b00001000,   // FIFO is not cleared if CRC error (PayloadReady interrupt still asserted)
} sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_CRCAOFF;
#define sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_CRCAOFF_MSK 0b00001000
// [2-1] AddressFiltering:
typedef enum sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_AF {
	sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_AF_OFF           	= 0b00000000,   // No address filtering
	sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_AF_NODE          	= 0b00000010,   // Node address filtering
	sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_AF_NODE_BROADCAST	= 0b00000100,   // Node or broadcast address filtering
	// 11 reserved
} sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_AF;
#define sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_AF_MSK 0b00000110
// [0] CrcWhiteningSeed:
typedef enum sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_CRCWSEED {
	sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_CRCWSEED_CCITT	= 0b00000000,   // CRC-CCITT (0x1D0F complemented)
	sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_CRCWSEED_IBM	= 0b00000001,   // CRC-IBM (0xFFFF not complemented)
} sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_CRCWSEED;
#define sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_CRCWSEED_MSK 0b00000001





// ====================== 0x31 PacketConfig2 ======================
// [7] Unused
// [6] DataMode:
typedef enum sx127x_FSK_OOK_REG_31_PACKET_CONFIG2_DM {
	sx127x_FSK_OOK_REG_31_PACKET_CONFIG2_DM_CONTINUOUS	= 0b00000000,   // Continuous mode
	sx127x_FSK_OOK_REG_31_PACKET_CONFIG2_DM_PACKET		= 0b01000000,   // Packet mode
} sx127x_FSK_OOK_REG_31_PACKET_CONFIG2_DM;
#define sx127x_FSK_OOK_REG_31_PACKET_CONFIG2_DM_MSK 0b01000000
// [5] IoHomeOn:
typedef enum sx127x_FSK_OOK_REG_31_PACKET_CONFIG2_IOHOMEON {
	sx127x_FSK_OOK_REG_31_PACKET_CONFIG2_IOHOMEON_OFF    = 0b00000000,   // IoHomeControl mode disabled
	sx127x_FSK_OOK_REG_31_PACKET_CONFIG2_IOHOMEON_ON     = 0b00100000,   // IoHomeControl mode enabled
} sx127x_FSK_OOK_REG_31_PACKET_CONFIG2_IOHOMEON;
#define sx127x_FSK_OOK_REG_31_PACKET_CONFIG2_IOHOMEON_MSK 0b00100000
// [4] IoHomePowerFrame:
#define sx127x_FSK_OOK_REG_31_PACKET_CONFIG2_IOHOME_POWER_FRAME_MSK 0b00010000
// [3] BeconOn:
typedef enum sx127x_FSK_OOK_REG_31_PACKET_CONFIG2_BEACONON {
	sx127x_FSK_OOK_REG_31_PACKET_CONFIG2_BEACONON_OFF    = 0b00000000,   // Beacon disabled
	sx127x_FSK_OOK_REG_31_PACKET_CONFIG2_BEACONON_ON     = 0b00001000,   // Beacon enabled
} sx127x_FSK_OOK_REG_31_PACKET_CONFIG2_BEACONON;
#define sx127x_FSK_OOK_REG_31_PACKET_CONFIG2_BEACONON_MSK 0b00001000
// [2-0] PayloadLength:
#define sx127x_FSK_OOK_REG_31_PACKET_CONFIG2_PAYLOAD_LENGTH_MSK 0b00000111





// ====================== 0x35 FifoThreash ======================
// [7] TxStartCondition:
typedef enum sx127x_FSK_OOK_REG_35_FIFO_THRESH_TX_START_COND {
	sx127x_FSK_OOK_REG_35_FIFO_THRESH_TX_START_COND_LEVEL	= 0b00000000,   // Start Tx when FIFO level > FIFOThresh
	sx127x_FSK_OOK_REG_35_FIFO_THRESH_TX_START_COND_EMPTY	= 0b10000000,   // Start Tx when FIFO is not empty
} sx127x_FSK_OOK_REG_35_FIFO_THRESH_TX_START_COND;
#define sx127x_FSK_OOK_REG_35_FIFO_THRESH_TX_START_COND_MSK 0b10000000
// [6] Unused
// [5-0] FifoThresh: (IRQ triggered when FIFO level > FifoThresh)
#define sx127x_FSK_OOK_REG_35_FIFO_THRESH_FIFO_THRESH_MSK 0b00111111





// ====================== 0x36 SeqConfig1 ======================
// [7] SequencerStart: (wt: write trigger)
#define sx127x_FSK_OOK_REG_36_SEQ_CONFIG_SEQUENCER_START_MSK 0b10000000
// [6] SequencerStop: (wt: write trigger)
#define sx127x_FSK_OOK_REG_36_SEQ_CONFIG_SEQUENCER_STOP_MSK 0b01000000
// [5] IdleMode:
typedef enum sx127x_FSK_OOK_REG_36_SEQ_CONFIG_IDLE_MODE {
	sx127x_FSK_OOK_REG_36_SEQ_CONFIG_IDLE_MODE_STDBY	= 0b00000000,   // Standby
	sx127x_FSK_OOK_REG_36_SEQ_CONFIG_IDLE_MODE_SLEEP	= 0b00100000,   // Sleep
} sx127x_FSK_OOK_REG_36_SEQ_CONFIG_IDLE_MODE;
#define sx127x_FSK_OOK_REG_36_SEQ_CONFIG_IDLE_MODE_MSK 0b00100000
// [4-3] FromStart:
typedef enum sx127x_FSK_OOK_REG_36_SEQ_CONFIG_FROM_START {
	sx127x_FSK_OOK_REG_36_SEQ_CONFIG_FROM_START_LPS		= 0b00000000,   // to LowPowerSelection
	sx127x_FSK_OOK_REG_36_SEQ_CONFIG_FROM_START_RX 		= 0b00001000,   // to Rx
	sx127x_FSK_OOK_REG_36_SEQ_CONFIG_FROM_START_TX 		= 0b00010000,   // to Tx
	sx127x_FSK_OOK_REG_36_SEQ_CONFIG_FROM_START_TX_FIFO	= 0b00011000,   // to Tx when FifoLevel IRQ
} sx127x_FSK_OOK_REG_36_SEQ_CONFIG_FROM_START;
#define sx127x_FSK_OOK_REG_36_SEQ_CONFIG_FROM_START_MSK 0b00011000
// [2] LowPowerSelection:
typedef enum sx127x_FSK_OOK_REG_36_SEQ_CONFIG_LPS {
	sx127x_FSK_OOK_REG_36_SEQ_CONFIG_LPS_OFF	= 0b00000000,   // Sequencer Off
	sx127x_FSK_OOK_REG_36_SEQ_CONFIG_LPS_IDLE	= 0b00000100,   // to Idle
} sx127x_FSK_OOK_REG_36_SEQ_CONFIG_LPS;
#define sx127x_FSK_OOK_REG_36_SEQ_CONFIG_LPS_MSK 0b00000100
// [1] FromIdle: (on T1)
typedef enum sx127x_FSK_OOK_REG_36_SEQ_CONFIG_FROM_IDLE {
	sx127x_FSK_OOK_REG_36_SEQ_CONFIG_FROM_IDLE_TX	= 0b00000000,   // to Tx
	sx127x_FSK_OOK_REG_36_SEQ_CONFIG_FROM_IDLE_RX	= 0b00000010,   // to Rx
} sx127x_FSK_OOK_REG_36_SEQ_CONFIG_FROM_IDLE;
#define sx127x_FSK_OOK_REG_36_SEQ_CONFIG_FROM_IDLE_MSK 0b00000010
// [0] FromTransmit: (on PacketSent)
typedef enum sx127x_FSK_OOK_REG_36_SEQ_CONFIG_FROM_TX {
	sx127x_FSK_OOK_REG_36_SEQ_CONFIG_FROM_TX_LPS	= 0b00000000,   // to LowPowerSelection
	sx127x_FSK_OOK_REG_36_SEQ_CONFIG_FROM_TX_RX		= 0b00000001,   // to Rx
} sx127x_FSK_OOK_REG_36_SEQ_CONFIG_FROM_TX;
#define sx127x_FSK_OOK_REG_36_SEQ_CONFIG_FROM_TX_MSK 0b00000001





// ====================== 0x37 SeqConfig2 ======================
// [7-5] FromReceive: (on PayloadReady)
typedef enum sx127x_FSK_OOK_REG_37_SEQ_CONFIG_FROM_RX {
	// 000 unused
	sx127x_FSK_OOK_REG_37_SEQ_CONFIG_FROM_RX_PKTRX_PR	= 0b00100000,   // to PacketReceived on PayloadReady
	sx127x_FSK_OOK_REG_37_SEQ_CONFIG_FROM_RX_LPS		= 0b01000000,   // to LowPowerSelection on PayloadReady
	sx127x_FSK_OOK_REG_37_SEQ_CONFIG_FROM_RX_PKTRX_CRC	= 0b01100000,   // to PacketReceived on CRC OK
	sx127x_FSK_OOK_REG_37_SEQ_CONFIG_FROM_RX_OFF_RSSI	= 0b10000000,   // Sequencer Off on RSSI
	sx127x_FSK_OOK_REG_37_SEQ_CONFIG_FROM_RX_OFF_SA		= 0b10100000,   // Sequencer Off on SyncAddress
	sx127x_FSK_OOK_REG_37_SEQ_CONFIG_FROM_RX_OFF_PD		= 0b11000000,   // Sequencer Off on PreambleDetect
	// 111 reserved
} sx127x_FSK_OOK_REG_37_SEQ_CONFIG_FROM_RX;
#define sx127x_FSK_OOK_REG_37_SEQ_CONFIG_FROM_RX_MSK 0b11100000
// [4-3] FromRxTimeout:
typedef enum sx127x_FSK_OOK_REG_37_SEQ_CONFIG_FROM_RX_TIMEOUT {
	sx127x_FSK_OOK_REG_37_SEQ_CONFIG_FROM_RX_TIMEOUT_RX		= 0b00000000,   // to Rx
	sx127x_FSK_OOK_REG_37_SEQ_CONFIG_FROM_RX_TIMEOUT_TX		= 0b00001000,   // to Tx
	sx127x_FSK_OOK_REG_37_SEQ_CONFIG_FROM_RX_TIMEOUT_LPS	= 0b00010000,   // to LowPowerSelection
	sx127x_FSK_OOK_REG_37_SEQ_CONFIG_FROM_RX_TIMEOUT_OFF	= 0b00011000,   // Sequencer Off
} sx127x_FSK_OOK_REG_37_SEQ_CONFIG_FROM_RX_TIMEOUT;
#define sx127x_FSK_OOK_REG_37_SEQ_CONFIG_FROM_RX_TIMEOUT_MSK 0b00011000
// [2-0] FromPacketReceived:
typedef enum sx127x_FSK_OOK_REG_37_SEQ_CONFIG_FROM_PKTRX {
	sx127x_FSK_OOK_REG_37_SEQ_CONFIG_FROM_PKTRX_OFF		= 0b00000000,   // Sequencer Off
	sx127x_FSK_OOK_REG_37_SEQ_CONFIG_FROM_PKTRX_TX		= 0b00000001,   // to Tx on FifoEmpty
	sx127x_FSK_OOK_REG_37_SEQ_CONFIG_FROM_PKTRX_LPS		= 0b00000010,   // to LowPowerSelection
	sx127x_FSK_OOK_REG_37_SEQ_CONFIG_FROM_PKTRX_RX_FS	= 0b00000011,   // to Rx via FreqSynth
	sx127x_FSK_OOK_REG_37_SEQ_CONFIG_FROM_PKTRX_RX		= 0b00000100,   // to Rx
	// 101-111 reserved
} sx127x_FSK_OOK_REG_37_SEQ_CONFIG_FROM_PKTRX;
#define sx127x_FSK_OOK_REG_37_SEQ_CONFIG_FROM_PKTRX_MSK 0b00000111





// ====================== 0x38 TimerResol ======================
// [7-4] Unused
// [3-2] Timer1Resolution:
typedef enum sx127x_FSK_OOK_REG_38_TIMER1_RESOLUTION {
	sx127x_FSK_OOK_REG_38_TIMER1_RESOLUTION_OFF	    = 0b00000000,   // Timer1 disabled
	sx127x_FSK_OOK_REG_38_TIMER1_RESOLUTION_64_US	= 0b00000100,   // Timer1 resolution = 64 us
	sx127x_FSK_OOK_REG_38_TIMER1_RESOLUTION_4_1_MS	= 0b00001000,   // Timer1 resolution = 4.1 ms
	sx127x_FSK_OOK_REG_38_TIMER1_RESOLUTION_262_MS	= 0b00001100,   // Timer1 resolution = 262 ms
} sx127x_FSK_OOK_REG_38_TIMER1_RESOLUTION;
#define sx127x_FSK_OOK_REG_38_TIMER1_RESOLUTION_MSK 0b00001100
// [1-0] Timer2Resolution:
typedef enum sx127x_FSK_OOK_REG_38_TIMER2_RESOLUTION {
	sx127x_FSK_OOK_REG_38_TIMER2_RESOLUTION_OFF	    = 0b00000000,   // Timer2 disabled
	sx127x_FSK_OOK_REG_38_TIMER2_RESOLUTION_64_US	= 0b00000001,   // Timer2 resolution = 64 us
	sx127x_FSK_OOK_REG_38_TIMER2_RESOLUTION_4_1_MS	= 0b00000010,   // Timer2 resolution = 4.1 ms
	sx127x_FSK_OOK_REG_38_TIMER2_RESOLUTION_262_MS	= 0b00000011,   // Timer2 resolution = 262 ms
} sx127x_FSK_OOK_REG_38_TIMER2_RESOLUTION;
#define sx127x_FSK_OOK_REG_38_TIMER2_RESOLUTION_MSK 0b00000011





// ====================== 0x3b ImageCal ======================
// [7] AutoImageCalOn:
typedef enum sx127x_FSK_OOK_REG_3B_IMAGE_CAL_AICO {
	sx127x_FSK_OOK_REG_3B_IMAGE_CAL_AICO_OFF    = 0b00000000,   // Automatic image calibration disabled
	sx127x_FSK_OOK_REG_3B_IMAGE_CAL_AICO_ON     = 0b10000000,   // Automatic image calibration enabled
} sx127x_FSK_OOK_REG_3B_IMAGE_CAL_AICO;
#define sx127x_FSK_OOK_REG_3B_IMAGE_CAL_AICO_MSK 0b10000000
// [6] ImageCalStart: (wt: write trigger)
#define sx127x_FSK_OOK_REG_3B_IMAGE_CAL_ICAL_START_MSK 0b01000000
// [5] ImageCalRunning: (r: read only)
#define sx127x_FSK_OOK_REG_3B_IMAGE_CAL_ICAL_RUNNING_MSK 0b00100000
// [4] Unused
// [3] TempChange: (r: read only)
#define sx127x_FSK_OOK_REG_3B_IMAGE_CAL_TEMP_CHANGE_MSK 0b00001000
// [2-1] TempThreshold:
typedef enum sx127x_FSK_OOK_REG_3B_IMAGE_CAL_TEMP_THRESHOLD {
	sx127x_FSK_OOK_REG_3B_IMAGE_CAL_TEMP_THRESHOLD_0	= 0b00000000,   // 0 °C
	sx127x_FSK_OOK_REG_3B_IMAGE_CAL_TEMP_THRESHOLD_5	= 0b00000010,   // 5 °C
	sx127x_FSK_OOK_REG_3B_IMAGE_CAL_TEMP_THRESHOLD_10	= 0b00000100,   // 10 °C
	sx127x_FSK_OOK_REG_3B_IMAGE_CAL_TEMP_THRESHOLD_20	= 0b00000110,   // 20 °C
} sx127x_FSK_OOK_REG_3B_IMAGE_CAL_TEMP_THRESHOLD;
#define sx127x_FSK_OOK_REG_3B_IMAGE_CAL_TEMP_THRESHOLD_MSK 0b00000110
// [0] TempMonitorOff:
typedef enum sx127x_FSK_OOK_REG_3B_IMAGE_CAL_TMO {
	sx127x_FSK_OOK_REG_3B_IMAGE_CAL_TMO_ON	= 0b00000000,   // Temperature monitoring done
	sx127x_FSK_OOK_REG_3B_IMAGE_CAL_TMO_OFF	= 0b00000001,   // Temperature monitoring disabled
} sx127x_FSK_OOK_REG_3B_IMAGE_CAL_TMO;
#define sx127x_FSK_OOK_REG_3B_IMAGE_CAL_TMO_MSK 0b00000001





// ====================== 0x3d LowBat ======================
// [7-4] Unused
// [3] LowBatOn:
typedef enum sx127x_FSK_OOK_REG_3D_LOW_BAT_LB_ON {
	sx127x_FSK_OOK_REG_3D_LOW_BAT_LB_ON_OFF    = 0b00000000,   // Low battery detector disabled
	sx127x_FSK_OOK_REG_3D_LOW_BAT_LB_ON_ON     = 0b00001000,   // Low battery detector enabled
} sx127x_FSK_OOK_REG_3D_LOW_BAT_LB_ON;
#define sx127x_FSK_OOK_REG_3D_LOW_BAT_LB_ON_MSK 0b00001000
// [2-0] LowBatTrim:
typedef enum sx127x_FSK_OOK_REG_3D_LOW_BAT_LB_TRIM {
	sx127x_FSK_OOK_REG_3D_LOW_BAT_LB_TRIM_1_695V	= 0b00000000,   // 1.695 V
	sx127x_FSK_OOK_REG_3D_LOW_BAT_LB_TRIM_1_764V	= 0b00000001,   // 1.764 V
	sx127x_FSK_OOK_REG_3D_LOW_BAT_LB_TRIM_1_835V	= 0b00000010,   // 1.835 V
	sx127x_FSK_OOK_REG_3D_LOW_BAT_LB_TRIM_1_905V	= 0b00000011,   // 1.905 V
	sx127x_FSK_OOK_REG_3D_LOW_BAT_LB_TRIM_1_976V	= 0b00000100,   // 1.976 V
	sx127x_FSK_OOK_REG_3D_LOW_BAT_LB_TRIM_2_045V	= 0b00000101,   // 2.045 V
	sx127x_FSK_OOK_REG_3D_LOW_BAT_LB_TRIM_2_116V	= 0b00000110,   // 2.116 V
	sx127x_FSK_OOK_REG_3D_LOW_BAT_LB_TRIM_2_185V	= 0b00000111,   // 2.185 V
} sx127x_FSK_OOK_REG_3D_LOW_BAT_LB_TRIM;
#define sx127x_FSK_OOK_REG_3D_LOW_BAT_LB_TRIM_MSK 0b00000111





// ====================== 0x3e IrqFlags1 ======================
// [7] ModeReady: (r: read only)
#define sx127x_FSK_OOK_REG_3E_IRQ_FLAGS1_MODE_READY_MSK			0b10000000
// [6] RxReady: (r: read only)
#define sx127x_FSK_OOK_REG_3E_IRQ_FLAGS1_RX_READY_MSK			0b01000000
// [5] TxReady: (r: read only)
#define sx127x_FSK_OOK_REG_3E_IRQ_FLAGS1_TX_READY_MSK			0b00100000
// [4] PllLock: (r: read only)
#define sx127x_FSK_OOK_REG_3E_IRQ_FLAGS1_PLL_LOCK_MSK			0b00010000
// [3] Rssi: (rwc: read write clear) clear when leaving Rx or on writing 1
#define sx127x_FSK_OOK_REG_3E_IRQ_FLAGS1_RSSI_MSK				0b00001000
// [2] Timeout: (r: read only)
#define sx127x_FSK_OOK_REG_3E_IRQ_FLAGS1_TIMEOUT_MSK			0b00000100
// [1] PreambleDetect: (rwc: read write clear) clear on writing 1
#define sx127x_FSK_OOK_REG_3E_IRQ_FLAGS1_PREAMBLE_DETECT_MSK	0b00000010
// [0] SyncAddressMatch: (r: read only on PacketMode, rwc: read write clear on ContinuousMode) clear on writing 1
#define sx127x_FSK_OOK_REG_3E_IRQ_FLAGS1_SYNC_ADDRESS_MSK		0b00000001





// ====================== 0x3f IrqFlags2 ======================
// [7] FifoFull: (r: read only)
#define sx127x_FSK_OOK_REG_3F_IRQ_FLAGS2_FIFO_FULL_MSK			0b10000000
// [6] FifoEmpty: (r: read only)
#define sx127x_FSK_OOK_REG_3F_IRQ_FLAGS2_FIFO_EMPTY_MSK			0b01000000
// [5] FifoLevel: (r: read only)
#define sx127x_FSK_OOK_REG_3F_IRQ_FLAGS2_FIFO_LEVEL_MSK			0b00100000
// [4] FifoOverrun: (rwc: read write clear) clear on writing 1
#define sx127x_FSK_OOK_REG_3F_IRQ_FLAGS2_FIFO_OVERRUN_MSK		0b00010000
// [3] PacketSent: (r: read only)
#define sx127x_FSK_OOK_REG_3F_IRQ_FLAGS2_PACKET_SENT_MSK		0b00001000
// [2] PayloadReady: (r: read only)
#define sx127x_FSK_OOK_REG_3F_IRQ_FLAGS2_PAYLOAD_READY_MSK		0b00000100
// [1] CrcOk: (r: read only)
#define sx127x_FSK_OOK_REG_3F_IRQ_FLAGS2_CRC_OK_MSK				0b00000010
// [0] LowBat: (rwc: read write clear) clear on writing 1
#define sx127x_FSK_OOK_REG_3F_IRQ_FLAGS2_LOW_BAT_MSK			0b00000001



typedef enum sx127x_FSK_OOK_mode_t {
    SX127X_FSK_OOK_MODE_FSK = 0,
    SX127X_FSK_OOK_MODE_OOK = 1,
} sx127x_FSK_OOK_mode_t;

typedef enum sx127x_FSK_OOK_RxBw_t {
    SX127X_FSK_OOK_RxBw_2_6kHz = 0,
	SX127X_FSK_OOK_RxBw_3_1kHz,
	SX127X_FSK_OOK_RxBw_3_9kHz,
	SX127X_FSK_OOK_RxBw_5_2kHz,
	SX127X_FSK_OOK_RxBw_6_3kHz,
	SX127X_FSK_OOK_RxBw_7_8kHz,
	SX127X_FSK_OOK_RxBw_10_4kHz,
	SX127X_FSK_OOK_RxBw_12_5kHz,
	SX127X_FSK_OOK_RxBw_15_6kHz,
	SX127X_FSK_OOK_RxBw_20_8kHz,
	SX127X_FSK_OOK_RxBw_25_0kHz,
	SX127X_FSK_OOK_RxBw_31_3kHz,
	SX127X_FSK_OOK_RxBw_41_7kHz,
	SX127X_FSK_OOK_RxBw_50_0kHz,
	SX127X_FSK_OOK_RxBw_62_5kHz,
	SX127X_FSK_OOK_RxBw_83_3kHz,
	SX127X_FSK_OOK_RxBw_100_0kHz,
	SX127X_FSK_OOK_RxBw_125_0kHz,
	SX127X_FSK_OOK_RxBw_166_7kHz,
	SX127X_FSK_OOK_RxBw_200_0kHz,
	SX127X_FSK_OOK_RxBw_250_0kHz,
} sx127x_FSK_OOK_RxBw_t;

typedef struct sx127x_FSK_OOK_packet_cfg_t {
    /* Preamble */
    uint16_t	preamble_len;		// RegPreambleMsb/Lsb

    /* Sync word (no SyncAddr / no AddrFilt) */
    bool		sync_on;			// RegSyncConfig.SyncOn
    uint8_t		sync_len;			// 1..8 bytes
    uint8_t		sync_word[8];		// RegSyncValue1..8

    /* Packet format */
    bool		variable_length;	// RegPacketConfig1.PacketFormat
    uint16_t	payload_len;		// fixed: RegPayloadLength; variable: max accepted length
    bool		crc_on;				// RegPacketConfig1.CrcOn
} sx127x_FSK_OOK_packet_cfg_t;

typedef enum sx127x_ook_thresh_type_t {
    SX127X_OOK_THRESH_FIXED,
    SX127X_OOK_THRESH_PEAK,
    SX127X_OOK_THRESH_AVERAGE,
} sx127x_ook_thresh_type_t;

typedef struct sx127x_ook_config_t {
    sx127x_ook_thresh_type_t thresh_type;

	union {
		struct {
			/* FIXED */
			uint8_t	fixed_thresh;	// RegOokFix.OokThreshFixed
		} fixed;

		struct {
			/* PEAK */
			bool peak_sync_on;										// RegOokPeak.OokPeakThreshSync
			sx127x_FSK_OOK_REG_14_OOK_PEAK_THRESH_TYPE peak_type;	// RegOokPeak.OokPeakThreshType
			sx127x_FSK_OOK_REG_14_OOK_PEAK_THRESH_STEP peak_step;	// RegOokPeak.OokPeakThreshStep
		} peak;

		struct {
			/* AVERAGE */
			sx127x_FSK_OOK_REG_16_OOK_AVG_PEAK_THRESH_DEC avg_peak_dec;	// RegOokAvg.OokAvgPeakThreshDec
			sx127x_FSK_OOK_REG_16_OOK_AVG_OFFSET avg_offset;			// RegOokAvg.OokAvgOffset
			sx127x_FSK_OOK_REG_16_OOK_AVG_THRESH_FILT avg_filt;			// RegOokAvg.OokAvgThreshFilt
		} average;
	} params;
} sx127x_ook_config_t;





typedef struct sx127x_FSK_OOK_config_t {
    uint32_t									bitrate;	// in bps
	sx127x_FSK_OOK_RxBw_t						RxBw;		// Rx bandwidth
    sx127x_FSK_OOK_REG_0A_PA_RAMP_MOD_SHAPING	modShaping;	// modulation shaping
    sx127x_FSK_OOK_REG_0A_PA_RAMP_PA_RAMP		paRamp;		// PA ramp time
	sx127x_FSK_OOK_packet_cfg_t					packetCfg;	// packet configuration
	union {
		struct {
			/* FSK specific */
			uint32_t fdev;	// frequency deviation in Hz
		} fsk;

		struct {
			/* OOK specific */
			sx127x_ook_config_t ookCfg;	// OOK configuration
		} ook;
	} mod_config;
} sx127x_FSK_OOK_config_t;


typedef struct sx127x_FSK_OOK_chip_t {
	sx127x_chip_t *base_chip;
	sx127x_FSK_OOK_config_t config;
} sx127x_FSK_OOK_chip_t;


sx127x_status_t sx127x_FSK_Config(sx127x_FSK_OOK_chip_t *chip, sx127x_chip_t *base_chip, sx127x_FSK_OOK_config_t config);
sx127x_status_t sx127x_OOK_Config(sx127x_FSK_OOK_chip_t *chip, sx127x_chip_t *base_chip, sx127x_FSK_OOK_config_t config);

sx127x_status_t sx127x_FSK_OOK_SetMode(sx127x_FSK_OOK_chip_t *chip, sx127x_FSK_OOK_REG_01_OP_MODE_MODE mode);

sx127x_status_t sx127x_FSK_OOK_TxSendFixLen(sx127x_FSK_OOK_chip_t *chip, const uint8_t *data);
sx127x_status_t sx127x_FSK_OOK_TxSend(sx127x_FSK_OOK_chip_t *chip, const uint8_t *data, uint8_t len);

sx127x_status_t sx127x_FSK_OOK_RxReceiveFixLen(sx127x_FSK_OOK_chip_t *chip, uint8_t *data);
sx127x_status_t sx127x_FSK_OOK_RxReceive(sx127x_FSK_OOK_chip_t *chip, uint8_t *data, uint8_t *len);

#endif // SX127X_FSK_OOK_H