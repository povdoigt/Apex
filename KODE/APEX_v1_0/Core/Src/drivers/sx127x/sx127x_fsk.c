#include "drivers/sx127x/sx127x_fsk.h"
#include "drivers/sx127x/sx127x_common.h"
#include "stm32f4xx_hal.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// Helper function to calculate and set bitrate
static sx127x_status_t sx127x_FSK_SetBitrate(sx127x_chip_t *chip, uint32_t bitrate) {
	if (bitrate < SX127X_FSK_BITRATE_MIN || bitrate > SX127X_FSK_BITRATE_MAX) {
		return sx127x_STATUS_ERROR;
	}

	// BitRate = Fxosc / BitRateReg
	// BitRateReg = Fxosc / BitRate
	uint32_t bitrate_reg = (uint32_t)(sx127x_FXOSC / bitrate);

	uint8_t msb = (uint8_t)(bitrate_reg >> 8);
	uint8_t lsb = (uint8_t)(bitrate_reg & 0xFF);

	sx127x_status_t status;
	status = sx127x_RegWrite(chip, sx127x_FSK_OOK_REG_02_BITRATE_MSB, msb);
	if (status != sx127x_STATUS_OK) { return status; }
	status = sx127x_RegWrite(chip, sx127x_FSK_OOK_REG_03_BITRATE_LSB, lsb);
	if (status != sx127x_STATUS_OK) { return status; }

	return sx127x_STATUS_OK;
}

// Helper function to set frequency deviation (FSK only)
static sx127x_status_t sx127x_FSK_SetFreqDev(sx127x_chip_t *chip, int16_t freq_dev_hz) {
	// Fdev = Fstep * FdevReg
	// FdevReg = Fdev / Fstep
	uint32_t fdev_reg = (uint32_t)((float)abs(freq_dev_hz) / sx127x_FSTEP);

	uint8_t msb = (uint8_t)(fdev_reg >> 8);
	uint8_t lsb = (uint8_t)(fdev_reg & 0xFF);

	sx127x_status_t status;
	status = sx127x_RegWrite(chip, sx127x_FSK_OOK_REG_04_FDEV_MSB, msb);
	if (status != sx127x_STATUS_OK) { return status; }
	status = sx127x_RegWrite(chip, sx127x_FSK_OOK_REG_05_FDEV_LSB, lsb);
	if (status != sx127x_STATUS_OK) { return status; }

	return sx127x_STATUS_OK;
}

// Helper function to convert RxBw enum to register values
static void sx127x_FSK_GetRxBwRegValues(sx127x_FSK_OOK_RxBw_t rxbw, uint8_t *mant, uint8_t *exp) {
	// Mapping of RxBw enum to mantissa and exponent values
	// Based on SX127x datasheet Table 17
	const struct {
		uint8_t mant;
		uint8_t exp;
	} rxbw_map[] = {
		{sx127x_FSK_OOK_REG_12_RX_BW_MANT_24, 7},  // 2.6kHz
		{sx127x_FSK_OOK_REG_12_RX_BW_MANT_20, 7},  // 3.1kHz
		{sx127x_FSK_OOK_REG_12_RX_BW_MANT_16, 7},  // 3.9kHz
		{sx127x_FSK_OOK_REG_12_RX_BW_MANT_24, 6},  // 5.2kHz
		{sx127x_FSK_OOK_REG_12_RX_BW_MANT_20, 6},  // 6.3kHz
		{sx127x_FSK_OOK_REG_12_RX_BW_MANT_16, 6},  // 7.8kHz
		{sx127x_FSK_OOK_REG_12_RX_BW_MANT_24, 5},  // 10.4kHz
		{sx127x_FSK_OOK_REG_12_RX_BW_MANT_20, 5},  // 12.5kHz
		{sx127x_FSK_OOK_REG_12_RX_BW_MANT_16, 5},  // 15.6kHz
		{sx127x_FSK_OOK_REG_12_RX_BW_MANT_24, 4},  // 20.8kHz
		{sx127x_FSK_OOK_REG_12_RX_BW_MANT_20, 4},  // 25.0kHz
		{sx127x_FSK_OOK_REG_12_RX_BW_MANT_16, 4},  // 31.3kHz
		{sx127x_FSK_OOK_REG_12_RX_BW_MANT_24, 3},  // 41.7kHz
		{sx127x_FSK_OOK_REG_12_RX_BW_MANT_20, 3},  // 50.0kHz
		{sx127x_FSK_OOK_REG_12_RX_BW_MANT_16, 3},  // 62.5kHz
		{sx127x_FSK_OOK_REG_12_RX_BW_MANT_24, 2},  // 83.3kHz
		{sx127x_FSK_OOK_REG_12_RX_BW_MANT_20, 2},  // 100.0kHz
		{sx127x_FSK_OOK_REG_12_RX_BW_MANT_16, 2},  // 125.0kHz
		{sx127x_FSK_OOK_REG_12_RX_BW_MANT_24, 1},  // 166.7kHz
		{sx127x_FSK_OOK_REG_12_RX_BW_MANT_20, 1},  // 200.0kHz
		{sx127x_FSK_OOK_REG_12_RX_BW_MANT_16, 1}   // 250.0kHz
	};

	if (rxbw <= SX127X_FSK_OOK_RxBw_250_0kHz) {
		*mant = rxbw_map[rxbw].mant;
		*exp = rxbw_map[rxbw].exp;
	} else {
		// Default to 50kHz if invalid
		*mant = sx127x_FSK_OOK_REG_12_RX_BW_MANT_20;
		*exp = 3;
	}
}

// Helper function to change mode in FSK/OOK
static sx127x_status_t sx127x_FSK_ChangeMode(sx127x_chip_t *chip, sx127x_FSK_OOK_REG_01_OP_MODE_MODE mode) {
	if (!chip || chip->modulation != sx127x_MODULATION_FSK_OOK) {
		return sx127x_STATUS_ERROR;
	}
	sx127x_status_t status;
	uint8_t value = 0x00;

	// Read the current RegOpMode
	status = sx127x_RegRead(chip, sx127x_REG_01_OP_MODE, &value);
	if (status != sx127x_STATUS_OK) { return status; }

	// Modify only the mode bits
	value &= ~sx127x_FSK_OOK_REG_01_OP_MODE_MODE_MSK;
	value |= mode;

	// Write back the new RegOpMode
	status = sx127x_RegWrite(chip, sx127x_REG_01_OP_MODE, value);
	if (status != sx127x_STATUS_OK) { return status; }

	return sx127x_STATUS_OK;
}

sx127x_status_t sx127x_FSK_Config(sx127x_chip_t *sx127x_chip, sx127x_FSK_OOK_config_t config) {
	sx127x_status_t status;
	uint8_t value = 0x00;

	if (!sx127x_chip) {
		return sx127x_STATUS_ERROR;
	}

	// Get chip frequency band
	uint8_t band;
	status = sx127x_GetBand(sx127x_chip, &band);
	if (status != sx127x_STATUS_OK) { return status; }

	// Set the mode to sleep (need to be in sleep mode to change modulation)
	status = sx127x_RegRead(sx127x_chip, sx127x_REG_01_OP_MODE, &value);
	if (status != sx127x_STATUS_OK) { return status; }
	value &= ~sx127x_FSK_OOK_REG_01_OP_MODE_MODE_MSK;
	value |= sx127x_FSK_OOK_REG_01_OP_MODE_MODE_SLEEP;
	status = sx127x_RegWrite(sx127x_chip, sx127x_REG_01_OP_MODE, value);
	if (status != sx127x_STATUS_OK) { return status; }

	// Set the RegOpMode for FSK/OOK mode
	value = sx127x_FSK_OOK_REG_01_OP_MODE_LRM_FSK_OOK  // FSK/OOK mode (not LoRa)
		  | (config.mode == SX127X_FSK_OOK_MODE_FSK ? sx127x_FSK_OOK_REG_01_OP_MODE_MOD_TYPE_FSK : sx127x_FSK_OOK_REG_01_OP_MODE_MOD_TYPE_OOK)
		  | (band == sx127x_BAND_1 ? sx127x_FSK_OOK_REG_01_OP_MODE_LFMO_OFF : sx127x_FSK_OOK_REG_01_OP_MODE_LFMO_ON)
		  | sx127x_FSK_OOK_REG_01_OP_MODE_MODE_SLEEP;  // Sleep mode
	status = sx127x_RegWrite(sx127x_chip, sx127x_REG_01_OP_MODE, value);
	if (status != sx127x_STATUS_OK) { return status; }

	// Set bitrate
	status = sx127x_FSK_SetBitrate(sx127x_chip, config.bitrate);
	if (status != sx127x_STATUS_OK) { return status; }

	// Set frequency deviation (FSK only)
	if (config.mode == SX127X_FSK_OOK_MODE_FSK) {
		status = sx127x_FSK_SetFreqDev(sx127x_chip, config.mod_config.fsk.freqDev);
		if (status != sx127x_STATUS_OK) { return status; }
	}

	// Set PA ramp and modulation shaping
	value = config.modShaping | config.paRamp;
	status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_0A_PA_RAMP, value);
	if (status != sx127x_STATUS_OK) { return status; }

	// Set LNA boost
	value = sx127x_REG_0C_LNA_LNA_GAIN_G1           // Gain set to G1 (highest)
		  | sx127x_REG_0C_LNA_LNA_BOOST_LF_DFT      // Normal LNA current (LF)
		  | sx127x_REG_0C_LNA_LNA_BOOST_HF_BOOST;   // LNA current boosted by 150% (HF)
	status = sx127x_RegWrite(sx127x_chip, sx127x_REG_0C_LNA, value);
	if (status != sx127x_STATUS_OK) { return status; }

	// Set RX configuration
	value = sx127x_FSK_OOK_REG_0D_RX_CONFIG_RROC_OFF     // No auto restart on collision
		  | sx127x_FSK_OOK_REG_0D_RX_CONFIG_AFCAON_ON    // AFC enabled
		  | sx127x_FSK_OOK_REG_0D_RX_CONFIG_AGCAON_ON;   // AGC enabled
	status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_0D_RX_CONFIG, value);
	if (status != sx127x_STATUS_OK) { return status; }

	// Set RX bandwidth
	uint8_t rxbw_mant, rxbw_exp;
	sx127x_FSK_GetRxBwRegValues(config.RxBw, &rxbw_mant, &rxbw_exp);
	value = rxbw_mant | rxbw_exp;
	status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_12_RX_BW, value);
	if (status != sx127x_STATUS_OK) { return status; }

	// Set AFC bandwidth (same as RX bandwidth)
	status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_13_AFC_BW, value);
	if (status != sx127x_STATUS_OK) { return status; }

	// Configure OOK demodulator if in OOK mode
	if (config.mode == SX127X_FSK_OOK_MODE_OOK) {
		sx127x_ook_config_t *ook_cfg = &config.mod_config.ook.ookCfg;
		
		if (ook_cfg->thresh_type == SX127X_OOK_THRESH_FIXED) {
			// Set fixed threshold
			value = sx127x_FSK_OOK_REG_14_OOK_PEAK_BIT_SYNC_ON_ON
				  | sx127x_FSK_OOK_REG_14_OOK_PEAK_THRESH_TYPE_FIXED;
			status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_14_OOK_PEAK, value);
			if (status != sx127x_STATUS_OK) { return status; }
			
			status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_15_OOK_FIX, 
									ook_cfg->params.fixed.fixed_thresh);
			if (status != sx127x_STATUS_OK) { return status; }
		} else if (ook_cfg->thresh_type == SX127X_OOK_THRESH_PEAK) {
			// Set peak threshold
			value = sx127x_FSK_OOK_REG_14_OOK_PEAK_BIT_SYNC_ON_ON
				  | ook_cfg->params.peak.peak_type
				  | ook_cfg->params.peak.peak_step;
			status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_14_OOK_PEAK, value);
			if (status != sx127x_STATUS_OK) { return status; }
		} else if (ook_cfg->thresh_type == SX127X_OOK_THRESH_AVERAGE) {
			// Set average threshold
			value = sx127x_FSK_OOK_REG_14_OOK_PEAK_BIT_SYNC_ON_ON
				  | sx127x_FSK_OOK_REG_14_OOK_PEAK_THRESH_TYPE_AVG;
			status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_14_OOK_PEAK, value);
			if (status != sx127x_STATUS_OK) { return status; }

			value = ook_cfg->params.average.avg_peak_dec
				  | ook_cfg->params.average.avg_offset
				  | ook_cfg->params.average.avg_filt;
			status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_16_OOK_AVG, value);
			if (status != sx127x_STATUS_OK) { return status; }
		}
	}

	// Set preamble length
	uint8_t preamble_msb = (uint8_t)(config.packetCfg.preamble_len >> 8);
	uint8_t preamble_lsb = (uint8_t)(config.packetCfg.preamble_len & 0xFF);
	status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_25_PREAMBLE_MSB, preamble_msb);
	if (status != sx127x_STATUS_OK) { return status; }
	status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_26_PREAMBLE_LSB, preamble_lsb);
	if (status != sx127x_STATUS_OK) { return status; }

	// Set sync word configuration
	value = sx127x_FSK_OOK_REG_27_SYNC_CONFIG_ARRXM_OFF  // Auto restart disabled
		  | sx127x_FSK_OOK_REG_27_SYNC_CONFIG_PP_AA      // Preamble polarity 0xAA
		  | (config.packetCfg.sync_on ? sx127x_FSK_OOK_REG_27_SYNC_CONFIG_SO_ON : sx127x_FSK_OOK_REG_27_SYNC_CONFIG_SO_OFF)
		  | ((config.packetCfg.sync_len - 1) & sx127x_FSK_OOK_REG_27_SYNC_CONFIG_SYNC_SIZE_MSK);
	status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_27_SYNC_CONFIG, value);
	if (status != sx127x_STATUS_OK) { return status; }

	// Write sync word bytes
	if (config.packetCfg.sync_on && config.packetCfg.sync_len > 0) {
		uint8_t sync_len = (config.packetCfg.sync_len > 8) ? 8 : config.packetCfg.sync_len;
		for (uint8_t i = 0; i < sync_len; i++) {
			status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_28_SYNC_VALUE_1 + i, 
									config.packetCfg.sync_word[i]);
			if (status != sx127x_STATUS_OK) { return status; }
		}
	}

	// Set packet configuration 1
	value = (config.packetCfg.variable_length ? sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_PF_VARIABLE : sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_PF_FIXED)
		  | sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_DCFREE_OFF  // No DC-free encoding
		  | (config.packetCfg.crc_on ? sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_CRCON_ON : sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_CRCON_OFF)
		  | sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_CRCAOFF_CLEAR  // Clear FIFO on CRC error
		  | sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_AF_OFF;  // No address filtering
	status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_30_PACKET_CONFIG1, value);
	if (status != sx127x_STATUS_OK) { return status; }

	// Set packet configuration 2
	value = sx127x_FSK_OOK_REG_31_PACKET_CONFIG2_DM_PACKET  // Packet mode
		  | sx127x_FSK_OOK_REG_31_PACKET_CONFIG2_IOHOMEON_OFF;  // IoHome disabled
	status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_31_PACKET_CONFIG2, value);
	if (status != sx127x_STATUS_OK) { return status; }

	// Set payload length
	status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_32_PAYLOAD_LENGTH, 
							config.packetCfg.payload_max_len);
	if (status != sx127x_STATUS_OK) { return status; }

	// Set FIFO threshold
	value = sx127x_FSK_OOK_REG_35_FIFO_THRESH_TX_START_COND_EMPTY  // Start Tx when FIFO not empty
		  | (config.fifoThresh & sx127x_FSK_OOK_REG_35_FIFO_THRESH_FIFO_THRESH_MSK);
	status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_35_FIFO_THRESH, value);
	if (status != sx127x_STATUS_OK) { return status; }

	// Update chip modulation mode
	sx127x_chip->modulation = sx127x_MODULATION_FSK_OOK;

	return sx127x_STATUS_OK;
}
