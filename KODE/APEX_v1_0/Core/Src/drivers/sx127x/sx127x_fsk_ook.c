#include "drivers/sx127x/sx127x_fsk_ook.h"
#include "drivers/sx127x/sx127x_common.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

// Helper function to convert RxBw enum to register values
static void sx127x_FSK_OOK_RxBw_to_RegValue(sx127x_FSK_OOK_RxBw_t rxbw, uint8_t *mant, uint8_t *exp) {
	// RxBw = Fxosc / (RxBwMant * 2^(RxBwExp + 2))
	// Based on datasheet Table 19
	switch (rxbw) {
		case SX127X_FSK_OOK_RxBw_2_6kHz:   *mant = sx127x_FSK_OOK_REG_12_RX_BW_MANT_24; *exp = sx127x_FSK_OOK_REG_12_RX_BW_EXP_7; break;
		case SX127X_FSK_OOK_RxBw_3_1kHz:   *mant = sx127x_FSK_OOK_REG_12_RX_BW_MANT_20; *exp = sx127x_FSK_OOK_REG_12_RX_BW_EXP_7; break;
		case SX127X_FSK_OOK_RxBw_3_9kHz:   *mant = sx127x_FSK_OOK_REG_12_RX_BW_MANT_16; *exp = sx127x_FSK_OOK_REG_12_RX_BW_EXP_7; break;
		case SX127X_FSK_OOK_RxBw_5_2kHz:   *mant = sx127x_FSK_OOK_REG_12_RX_BW_MANT_24; *exp = sx127x_FSK_OOK_REG_12_RX_BW_EXP_6; break;
		case SX127X_FSK_OOK_RxBw_6_3kHz:   *mant = sx127x_FSK_OOK_REG_12_RX_BW_MANT_20; *exp = sx127x_FSK_OOK_REG_12_RX_BW_EXP_6; break;
		case SX127X_FSK_OOK_RxBw_7_8kHz:   *mant = sx127x_FSK_OOK_REG_12_RX_BW_MANT_16; *exp = sx127x_FSK_OOK_REG_12_RX_BW_EXP_6; break;
		case SX127X_FSK_OOK_RxBw_10_4kHz:  *mant = sx127x_FSK_OOK_REG_12_RX_BW_MANT_24; *exp = sx127x_FSK_OOK_REG_12_RX_BW_EXP_5; break;
		case SX127X_FSK_OOK_RxBw_12_5kHz:  *mant = sx127x_FSK_OOK_REG_12_RX_BW_MANT_20; *exp = sx127x_FSK_OOK_REG_12_RX_BW_EXP_5; break;
		case SX127X_FSK_OOK_RxBw_15_6kHz:  *mant = sx127x_FSK_OOK_REG_12_RX_BW_MANT_16; *exp = sx127x_FSK_OOK_REG_12_RX_BW_EXP_5; break;
		case SX127X_FSK_OOK_RxBw_20_8kHz:  *mant = sx127x_FSK_OOK_REG_12_RX_BW_MANT_24; *exp = sx127x_FSK_OOK_REG_12_RX_BW_EXP_4; break;
		case SX127X_FSK_OOK_RxBw_25_0kHz:  *mant = sx127x_FSK_OOK_REG_12_RX_BW_MANT_20; *exp = sx127x_FSK_OOK_REG_12_RX_BW_EXP_4; break;
		case SX127X_FSK_OOK_RxBw_31_3kHz:  *mant = sx127x_FSK_OOK_REG_12_RX_BW_MANT_16; *exp = sx127x_FSK_OOK_REG_12_RX_BW_EXP_4; break;
		case SX127X_FSK_OOK_RxBw_41_7kHz:  *mant = sx127x_FSK_OOK_REG_12_RX_BW_MANT_24; *exp = sx127x_FSK_OOK_REG_12_RX_BW_EXP_3; break;
		case SX127X_FSK_OOK_RxBw_50_0kHz:  *mant = sx127x_FSK_OOK_REG_12_RX_BW_MANT_20; *exp = sx127x_FSK_OOK_REG_12_RX_BW_EXP_3; break;
		case SX127X_FSK_OOK_RxBw_62_5kHz:  *mant = sx127x_FSK_OOK_REG_12_RX_BW_MANT_16; *exp = sx127x_FSK_OOK_REG_12_RX_BW_EXP_3; break;
		case SX127X_FSK_OOK_RxBw_83_3kHz:  *mant = sx127x_FSK_OOK_REG_12_RX_BW_MANT_24; *exp = sx127x_FSK_OOK_REG_12_RX_BW_EXP_2; break;
		case SX127X_FSK_OOK_RxBw_100_0kHz: *mant = sx127x_FSK_OOK_REG_12_RX_BW_MANT_20; *exp = sx127x_FSK_OOK_REG_12_RX_BW_EXP_2; break;
		case SX127X_FSK_OOK_RxBw_125_0kHz: *mant = sx127x_FSK_OOK_REG_12_RX_BW_MANT_16; *exp = sx127x_FSK_OOK_REG_12_RX_BW_EXP_2; break;
		case SX127X_FSK_OOK_RxBw_166_7kHz: *mant = sx127x_FSK_OOK_REG_12_RX_BW_MANT_24; *exp = sx127x_FSK_OOK_REG_12_RX_BW_EXP_1; break;
		case SX127X_FSK_OOK_RxBw_200_0kHz: *mant = sx127x_FSK_OOK_REG_12_RX_BW_MANT_20; *exp = sx127x_FSK_OOK_REG_12_RX_BW_EXP_1; break;
		case SX127X_FSK_OOK_RxBw_250_0kHz: *mant = sx127x_FSK_OOK_REG_12_RX_BW_MANT_16; *exp = sx127x_FSK_OOK_REG_12_RX_BW_EXP_1; break;
		default: *mant = sx127x_FSK_OOK_REG_12_RX_BW_MANT_24; *exp = sx127x_FSK_OOK_REG_12_RX_BW_EXP_5; break;
	}
}

sx127x_status_t __sx127x_FSK_OOK_Config(sx127x_FSK_OOK_chip_t *chip, sx127x_chip_t *base_chip, sx127x_FSK_OOK_config_t config, sx127x_modulation_t modulation) {
    if (!chip) { return sx127x_STATUS_ERROR; }
    
    sx127x_status_t status;
    uint8_t value = 0x00;

	chip->base_chip = base_chip;
	chip->base_chip->modulation = modulation;
	chip->config = config;

    // Get chip frequency band
    uint8_t band;
    status = sx127x_GetBand(chip->base_chip, &band);
    if (status != sx127x_STATUS_OK) { return status; }
    
    // Set the mode to sleep (required to switch from LoRa to FSK or configure FSK)
    status = sx127x_FSK_OOK_SetMode(chip, sx127x_FSK_OOK_REG_01_OP_MODE_MODE_SLEEP);
	if (status != sx127x_STATUS_OK) { return status; }
    
    // Set the RegOpMode to FSK/OOK mode
    value = sx127x_FSK_OOK_REG_01_OP_MODE_LRM_FSK_OOK	// FSK/OOK mode
    	  | (modulation == sx127x_MODULATION_FSK ? sx127x_FSK_OOK_REG_01_OP_MODE_MOD_TYPE_FSK : sx127x_FSK_OOK_REG_01_OP_MODE_MOD_TYPE_OOK) // Modulation type
    	  | (band == sx127x_BAND_1 ? sx127x_FSK_OOK_REG_01_OP_MODE_LFMO_OFF : sx127x_FSK_OOK_REG_01_OP_MODE_LFMO_ON)
    	  | sx127x_FSK_OOK_REG_01_OP_MODE_MODE_SLEEP;	// Sleep mode
    status = sx127x_RegWrite(chip->base_chip, sx127x_REG_01_OP_MODE, value);
    if (status != sx127x_STATUS_OK) { return status; }
    
    // Set bitrate (BitRate = Fxosc / BitRate register value)
    // BitRate register value = Fxosc / bitrate
    uint32_t bitrate_reg = (uint32_t)(sx127x_FXOSC / config.bitrate);
    uint8_t bitrate_buf[2] = {
        (uint8_t)((bitrate_reg >> 8) & 0xFF),
        (uint8_t)((bitrate_reg >> 0) & 0xFF) };
    status = sx127x_RegWriteMulti(chip->base_chip, sx127x_FSK_OOK_REG_02_BITRATE_MSB, bitrate_buf, 2);
    if (status != sx127x_STATUS_OK) { return status; }

	// Set RxConfig - enable AGC, disable AFC
	value = sx127x_FSK_OOK_REG_0D_RX_CONFIG_RROC_OFF	// Restart Rx on collision
		  | sx127x_FSK_OOK_REG_0D_RX_CONFIG_AFCAON_OFF	// AFC disabled
		  | sx127x_FSK_OOK_REG_0D_RX_CONFIG_AGCAON_OFF	// AGC enabled
		  | 0x00; // Default
	status = sx127x_RegWrite(chip->base_chip, sx127x_FSK_OOK_REG_0D_RX_CONFIG, value);
	if (status != sx127x_STATUS_OK) { return status; }

	// Set PA ramp and modulation shaping
	value = config.modShaping
		  | config.paRamp;	// Useful only for FSK modulation 
	status = sx127x_RegWrite(chip->base_chip, sx127x_FSK_OOK_REG_0A_PA_RAMP, value);
	if (status != sx127x_STATUS_OK) { return status; }

	// Set RxBw (Receiver bandwidth)
	uint8_t rxbw_mant, rxbw_exp;
	sx127x_FSK_OOK_RxBw_to_RegValue(config.RxBw, &rxbw_mant, &rxbw_exp);
	status = sx127x_RegWrite(chip->base_chip, sx127x_FSK_OOK_REG_12_RX_BW, rxbw_mant | rxbw_exp);
	if (status != sx127x_STATUS_OK) { return status; }

	// Set preamble length
	uint8_t preamble_buf[2] = {
		(uint8_t)((config.packetCfg.preamble_len >> 8) & 0xFF),
		(uint8_t)((config.packetCfg.preamble_len >> 0) & 0xFF) };
	status = sx127x_RegWriteMulti(chip->base_chip, sx127x_FSK_OOK_REG_25_PREAMBLE_MSB, preamble_buf, 2);
	if (status != sx127x_STATUS_OK) { return status; }

	// Set sync word configuration
	value = (config.packetCfg.sync_on ? sx127x_FSK_OOK_REG_27_SYNC_CONFIG_SO_ON : sx127x_FSK_OOK_REG_27_SYNC_CONFIG_SO_OFF)
			| ((config.packetCfg.sync_len - 1) & sx127x_FSK_OOK_REG_27_SYNC_CONFIG_SYNC_SIZE_MSK);
	status = sx127x_RegWrite(chip->base_chip, sx127x_FSK_OOK_REG_27_SYNC_CONFIG, value);
	if (status != sx127x_STATUS_OK) { return status; }

	// Write sync word bytes
	if (config.packetCfg.sync_on && config.packetCfg.sync_len > 0) {
		uint8_t write_len = (config.packetCfg.sync_len > 8) ? 8 : config.packetCfg.sync_len;
		status = sx127x_RegWriteMulti(chip->base_chip, sx127x_FSK_OOK_REG_28_SYNC_VALUE_1, config.packetCfg.sync_word, write_len);
		if (status != sx127x_STATUS_OK) { return status; }
	}

	// Set packet configuration 1
	value = (config.packetCfg.variable_length ? sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_PF_VARIABLE : sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_PF_FIXED)
			| sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_DCFREE_WHITENING
			| (config.packetCfg.crc_on ? sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_CRCON_ON : sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_CRCON_OFF)
			| sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_AF_OFF;
	status = sx127x_RegWrite(chip->base_chip, sx127x_FSK_OOK_REG_30_PACKET_CONFIG1, value);
	if (status != sx127x_STATUS_OK) { return status; }

	config.packetCfg.payload_len = config.packetCfg.variable_length ? 2047 : config.packetCfg.payload_len;

	uint8_t len[2];
	if (config.packetCfg.variable_length) {
		len[0] = 0xFF & sx127x_FSK_OOK_REG_31_PACKET_CONFIG2_PAYLOAD_LENGTH_MSK;
		len[1] = 0xFF;
	} else {
		len[0] = (uint8_t)((config.packetCfg.payload_len >> 8) & sx127x_FSK_OOK_REG_31_PACKET_CONFIG2_PAYLOAD_LENGTH_MSK);
		len[1] = (uint8_t)(config.packetCfg.payload_len & 0xFF);
	}

	// Set packet configuration 2 - packet mode
	value = sx127x_FSK_OOK_REG_31_PACKET_CONFIG2_DM_PACKET
		  | sx127x_FSK_OOK_REG_31_PACKET_CONFIG2_IOHOMEON_OFF
		  | sx127x_FSK_OOK_REG_31_PACKET_CONFIG2_BEACONON_OFF
		  | len[0];
	status = sx127x_RegWrite(chip->base_chip, sx127x_FSK_OOK_REG_31_PACKET_CONFIG2, value);
	if (status != sx127x_STATUS_OK) { return status; }

	value = len[1];
	status = sx127x_RegWrite(chip->base_chip, sx127x_FSK_OOK_REG_32_PAYLOAD_LENGTH, value);
	if (status != sx127x_STATUS_OK) { return status; }

	// // Set RxTimeoutRssi to 0x01 (T_out = 16 / BitRate)
	// status = sx127x_RegWrite(chip->base_chip, sx127x_FSK_OOK_REG_20_RX_TIMEOUT_1, 0x01);
	// if (status != sx127x_STATUS_OK) { return status; }


    return sx127x_STATUS_OK;
}

sx127x_status_t sx127x_FSK_Config(sx127x_FSK_OOK_chip_t *chip, sx127x_chip_t *base_chip, sx127x_FSK_OOK_config_t config) {
	if (!chip) { return sx127x_STATUS_ERROR; }

	sx127x_status_t status;

	status = __sx127x_FSK_OOK_Config(chip, base_chip, config, sx127x_MODULATION_FSK);
	if (status != sx127x_STATUS_OK) { return status; }

	// Set frequency deviation (FSK mode only)
	// Fdev = Fstep * Fdev register value
	// Fdev register value = Fdev / Fstep
	uint16_t fdev_reg = (uint16_t)((float)config.mod_config.fsk.fdev / sx127x_FSTEP);
	uint8_t fdev_buf[2] = {
		(uint8_t)((fdev_reg >> 8) & 0xFF),
		(uint8_t)((fdev_reg >> 0) & 0xFF) };
	status = sx127x_RegWriteMulti(chip->base_chip, sx127x_FSK_OOK_REG_04_FDEV_MSB, fdev_buf, 2);
	if (status != sx127x_STATUS_OK) { return status; }

	return sx127x_STATUS_OK;
}

sx127x_status_t sx127x_OOK_Config(sx127x_FSK_OOK_chip_t *chip, sx127x_chip_t *base_chip, sx127x_FSK_OOK_config_t config) {
    if (!chip) { return sx127x_STATUS_ERROR; }
    
    sx127x_status_t status;
    uint8_t value = 0x00;

	status = __sx127x_FSK_OOK_Config(chip, base_chip, config, sx127x_MODULATION_OOK);
	if (status != sx127x_STATUS_OK) { return status; }
    
    // Set OOK-specific configuration
    switch (config.mod_config.ook.ookCfg.thresh_type) {
        case SX127X_OOK_THRESH_FIXED:
            // Set fixed threshold
            value = sx127x_FSK_OOK_REG_14_OOK_PEAK_THRESH_TYPE_FIXED;
            status = sx127x_RegWrite(chip->base_chip, sx127x_FSK_OOK_REG_14_OOK_PEAK, value);
            if (status != sx127x_STATUS_OK) { return status; }
			value = config.mod_config.ook.ookCfg.params.fixed.fixed_thresh;
            status = sx127x_RegWrite(chip->base_chip, sx127x_FSK_OOK_REG_15_OOK_FIX, value);
            if (status != sx127x_STATUS_OK) { return status; }
            break;
            
        case SX127X_OOK_THRESH_PEAK:
            // Set peak threshold
            value = config.mod_config.ook.ookCfg.params.peak.peak_type
                  | config.mod_config.ook.ookCfg.params.peak.peak_step;
            status = sx127x_RegWrite(chip->base_chip, sx127x_FSK_OOK_REG_14_OOK_PEAK, value);
            if (status != sx127x_STATUS_OK) { return status; }
            break;
            
        case SX127X_OOK_THRESH_AVERAGE:
            // Set average threshold
            value = sx127x_FSK_OOK_REG_14_OOK_PEAK_THRESH_TYPE_AVG;
            status = sx127x_RegWrite(chip->base_chip, sx127x_FSK_OOK_REG_14_OOK_PEAK, value);
            if (status != sx127x_STATUS_OK) { return status; }
            value = config.mod_config.ook.ookCfg.params.average.avg_peak_dec
                  | config.mod_config.ook.ookCfg.params.average.avg_offset
                  | config.mod_config.ook.ookCfg.params.average.avg_filt;
            status = sx127x_RegWrite(chip->base_chip, sx127x_FSK_OOK_REG_16_OOK_AVG, value);
            if (status != sx127x_STATUS_OK) { return status; }
            break;
    }
    
    return sx127x_STATUS_OK;
}






sx127x_status_t sx127x_FSK_OOK_SetMode(sx127x_FSK_OOK_chip_t *chip, sx127x_FSK_OOK_REG_01_OP_MODE_MODE mode) {
	if (!chip || chip->base_chip->modulation == sx127x_MODULATION_LORA) { return sx127x_STATUS_ERROR; }

	sx127x_status_t status;
	uint8_t value;
	status = sx127x_RegRead(chip->base_chip, sx127x_REG_01_OP_MODE, &value);
	if (status != sx127x_STATUS_OK) { return status; }
	value &= ~sx127x_FSK_OOK_REG_01_OP_MODE_MODE_MSK;
	value |= mode;
	status = sx127x_RegWrite(chip->base_chip, sx127x_REG_01_OP_MODE, value);
	if (status != sx127x_STATUS_OK) { return status; }

	return sx127x_STATUS_OK;
}






sx127x_status_t __sx127x_FSK_OOK_TxSend(sx127x_FSK_OOK_chip_t *chip, const uint8_t *data, uint16_t len, bool variable_length) {
	if (!chip || chip->base_chip->modulation == sx127x_MODULATION_LORA || !data) { return sx127x_STATUS_ERROR; }

	sx127x_status_t status;
	uint8_t value;
	uint16_t max_size = SX127X_FSK_OOK_FIFO_SIZE;

	// Set the mode to standby
	status = sx127x_FSK_OOK_SetMode(chip, sx127x_FSK_OOK_REG_01_OP_MODE_MODE_STDBY);
	if (status != sx127x_STATUS_OK) { return status; }

	// Variable length packets: first byte is length
	if (variable_length) {
		max_size--;
		// Set the first byte to length
		status = sx127x_RegWrite(chip->base_chip, sx127x_REG_00_FIFO, (uint8_t)(len & 0xFF));
		if (status != sx127x_STATUS_OK) { return status; }
	}

	if (len > max_size) {
		uint16_t offset = 0;
		uint8_t fifo_half = (SX127X_FSK_OOK_FIFO_SIZE / 2);
		uint8_t chunk_size;
		bool fifo_threash;

		// Prefilled FIFO
		status = sx127x_RegWriteMulti(chip->base_chip, sx127x_REG_00_FIFO, data, max_size);
		if (status != sx127x_STATUS_OK) { return status; }
		len -= max_size;
		offset += max_size;

		// Set FIFO threshold to half FIFO size
		value = sx127x_FSK_OOK_REG_35_FIFO_THRESH_TX_START_COND_LEVEL
			  | (fifo_half & sx127x_FSK_OOK_REG_35_FIFO_THRESH_FIFO_THRESH_MSK);
		status = sx127x_RegWrite(chip->base_chip, sx127x_FSK_OOK_REG_35_FIFO_THRESH, value);
		if (status != sx127x_STATUS_OK) { return status; }

		// Set the mode to transmit
		status = sx127x_FSK_OOK_SetMode(chip, sx127x_FSK_OOK_REG_01_OP_MODE_MODE_TX);
		if (status != sx127x_STATUS_OK) { return status; }

		// Write data to FIFO in chunks
		while (len > 0) {
			// Wait until FIFO threshold is cleared
			fifo_threash = true;
			while (fifo_threash) {
				status = sx127x_RegRead(chip->base_chip, sx127x_FSK_OOK_REG_3F_IRQ_FLAGS2, &value);
				if (status != sx127x_STATUS_OK) { return status; }
				fifo_threash = value & sx127x_FSK_OOK_REG_3F_IRQ_FLAGS2_FIFO_LEVEL_MSK;
			}

			chunk_size = (len > fifo_half) ? fifo_half : (uint8_t)len;
			status = sx127x_RegWriteMulti(chip->base_chip, sx127x_REG_00_FIFO, &data[offset], chunk_size);
			if (status != sx127x_STATUS_OK) { return status; }

			len -= chunk_size;
			offset += chunk_size;
		}
	} else {
		// Set Tx start condition to FIFO not empty
		value = sx127x_FSK_OOK_REG_35_FIFO_THRESH_TX_START_COND_EMPTY;
		status = sx127x_RegWrite(chip->base_chip, sx127x_FSK_OOK_REG_35_FIFO_THRESH, value);
		if (status != sx127x_STATUS_OK) { return status; }

		// Write entire data to FIFO
		status = sx127x_RegWriteMulti(chip->base_chip, sx127x_REG_00_FIFO, data, len);
		if (status != sx127x_STATUS_OK) { return status; }

		// Set the mode to transmit
		status = sx127x_FSK_OOK_SetMode(chip, sx127x_FSK_OOK_REG_01_OP_MODE_MODE_TX);
		if (status != sx127x_STATUS_OK) { return status; }
	}

	// Wait for TxDone flag
	bool tx_done = false;
	while (!tx_done) {
		status = sx127x_RegRead(chip->base_chip, sx127x_FSK_OOK_REG_3F_IRQ_FLAGS2, &value);
		if (status != sx127x_STATUS_OK) { return status; }
		tx_done = value & sx127x_FSK_OOK_REG_3F_IRQ_FLAGS2_PACKET_SENT_MSK;
	}

	return sx127x_STATUS_OK;
}

sx127x_status_t sx127x_FSK_OOK_TxSendFixLen(sx127x_FSK_OOK_chip_t *chip, const uint8_t *data) {
	if (!chip || chip->base_chip->modulation == sx127x_MODULATION_LORA || !data) { return sx127x_STATUS_ERROR; }

	sx127x_status_t status;

	status = __sx127x_FSK_OOK_TxSend(chip, data, chip->config.packetCfg.payload_len, false);
	if (status != sx127x_STATUS_OK) { return status; }

	return sx127x_STATUS_OK;
}

sx127x_status_t sx127x_FSK_OOK_TxSend(sx127x_FSK_OOK_chip_t *chip, const uint8_t *data, uint8_t len) {
	if (!chip || chip->base_chip->modulation == sx127x_MODULATION_LORA || !data) { return sx127x_STATUS_ERROR; }

	sx127x_status_t status;

	len = (len > SX127X_FSK_OOK_MAX_PAYLOAD_SIZE) ? SX127X_FSK_OOK_MAX_PAYLOAD_SIZE : len;

	status = __sx127x_FSK_OOK_TxSend(chip, data, len, true);
	if (status != sx127x_STATUS_OK) { return status; }

	return sx127x_STATUS_OK;
}

sx127x_status_t __sx127x_FSK_OOK_RxReceive_IRQ(sx127x_FSK_OOK_chip_t *chip, uint32_t RxTimeout_delay_ms,
											   bool is_flag_2, uint8_t irq_mask, bool invert_mask) {
	if (!chip || chip->base_chip->modulation == sx127x_MODULATION_LORA) { return sx127x_STATUS_ERROR; }

	sx127x_status_t status;
	uint8_t values[2];

	uint32_t t0 = HAL_GetTick();
	while (true) {
		status = sx127x_RegReadMulti(chip->base_chip, sx127x_FSK_OOK_REG_3E_IRQ_FLAGS1, values, 2);
		if (status != sx127x_STATUS_OK) { return status; }
		if (values[1] & sx127x_FSK_OOK_REG_3F_IRQ_FLAGS2_FIFO_OVERRUN_MSK) {
			return sx127x_STATUS_ERROR;
		}
		if (values[0] & sx127x_FSK_OOK_REG_3E_IRQ_FLAGS1_RX_READY_MSK) {
			// Something happened
			__NOP();
		}

		// Check IRQ flag
		bool condition = values[(uint8_t)is_flag_2] & irq_mask;
		condition = invert_mask ? !condition : condition;
		if (condition) {
			break;
		}
		// Check for timeout
		if ((HAL_GetTick() - t0) > RxTimeout_delay_ms) {
			return sx127x_STATUS_MODEM_TIMEOUT;
		}
	}

	return sx127x_STATUS_OK;
}

sx127x_status_t __sx127x_FSK_OOK_RxReceive(sx127x_FSK_OOK_chip_t *chip, uint8_t *data, uint8_t *len_ptr) {
	if (!chip || chip->base_chip->modulation == sx127x_MODULATION_LORA || !data) { return sx127x_STATUS_ERROR; }

	sx127x_status_t status;
	uint8_t values[2];
	uint16_t len;
	uint16_t i;
	// Set FIFO threshold (chunk) to FIFO size / 2
	uint8_t fifo_half = SX127X_FSK_OOK_FIFO_SIZE / 2;

	// Time to receive one byte in ms
	float bytes_time = 8000.0f / chip->config.bitrate;

	// Set the mcu RxTimeout delay based on bitrate to preamble length + sync word length (add 1 byte time of margin)
	uint32_t preamble_byts = chip->config.packetCfg.preamble_len;
	uint32_t sync_byts = chip->config.packetCfg.sync_on ? (chip->config.packetCfg.sync_len) : 0;
	uint32_t RxTimeout_delay_ms = (uint32_t)ceil((1 + preamble_byts + sync_byts) * bytes_time);
	// RxTimeout_delay_ms = HAL_MAX_DELAY; // Disable timeout for debugging purposes

	len = chip->config.packetCfg.payload_len;	// Default length, will be overwritten if variable length
	i = 0;

	status = sx127x_RegRead(chip->base_chip, sx127x_FSK_OOK_REG_35_FIFO_THRESH, values);
	if (status != sx127x_STATUS_OK) { return status; }
	values[0] &= ~sx127x_FSK_OOK_REG_35_FIFO_THRESH_FIFO_THRESH_MSK;
	values[0] |= (fifo_half & sx127x_FSK_OOK_REG_35_FIFO_THRESH_FIFO_THRESH_MSK);
	status = sx127x_RegWrite(chip->base_chip, sx127x_FSK_OOK_REG_35_FIFO_THRESH, values[0]);
	if (status != sx127x_STATUS_OK) { return status; }

	// Set the mode to Rx
	status = sx127x_FSK_OOK_SetMode(chip, sx127x_FSK_OOK_REG_01_OP_MODE_MODE_RX);
	if (status != sx127x_STATUS_OK) { return status; }

	// Wait for FifoEmpty clear
	status = __sx127x_FSK_OOK_RxReceive_IRQ(chip, RxTimeout_delay_ms, true, sx127x_FSK_OOK_REG_3F_IRQ_FLAGS2_FIFO_EMPTY_MSK, true);
	if (status != sx127x_STATUS_OK) { return status; }

	// If variable length, read first byte as length
	if (len_ptr) {
		status = sx127x_RegRead(chip->base_chip, sx127x_REG_00_FIFO, len_ptr);
		if (status != sx127x_STATUS_OK) { return status; }
		len = 0;
		len = (uint16_t)*len_ptr; // Overwrite len with received length
		if (len == 0 || len > SX127X_FSK_OOK_MAX_PAYLOAD_SIZE) {
			return sx127x_STATUS_MODEM_TIMEOUT;
		}
	}

	// Calculate number of chunks to read and remaining bytes
	uint8_t nb_chunks = len / fifo_half;
	uint8_t remaining = len % fifo_half;

	// Because there is less than chunks_in_fifo space in FIFO and remaining < fifo_half,
	// the number of bytes (remaining + fifo_half) can fit in FIFO.
	// So we adjust nb_chunks and remaining accordingly to read the maximum possible bytes outside of the nb_chunks loop.
	// This way we avoid to clear the PayloadReady flag in the middle of reading.
	uint8_t nb_chunks_adjusted = nb_chunks - (nb_chunks > 0 ? 1 : 0);
	uint8_t remaining_adjusted = remaining + (nb_chunks > 0 ? fifo_half : 0);

	// Reajust RxTimeout for chunk reading
	RxTimeout_delay_ms = (uint32_t)ceil((1 + fifo_half) * bytes_time);

	if (nb_chunks_adjusted > 0) {
		for (uint8_t c = 0; c < nb_chunks_adjusted; c++) {
			// Wait until FIFO threshold is reached
			status = __sx127x_FSK_OOK_RxReceive_IRQ(chip, RxTimeout_delay_ms, true, sx127x_FSK_OOK_REG_3F_IRQ_FLAGS2_FIFO_LEVEL_MSK, false);
			if (status != sx127x_STATUS_OK) { return status; }
			// Read chunk from FIFO
			status = sx127x_RegReadMulti(chip->base_chip, sx127x_REG_00_FIFO, &data[i], fifo_half);
			if (status != sx127x_STATUS_OK) { return status; }
			i += fifo_half;
		}
	}

	// Reajust RxTimeout for remaining bytes reading
	RxTimeout_delay_ms = (uint32_t)ceil((1 + remaining_adjusted) * bytes_time);

	// Wait for PayloadReady flag
	status = __sx127x_FSK_OOK_RxReceive_IRQ(chip, RxTimeout_delay_ms, true, sx127x_FSK_OOK_REG_3F_IRQ_FLAGS2_PAYLOAD_READY_MSK, false);
	if (status != sx127x_STATUS_OK) { return status; }

	// Read remaining bytes
	if (remaining_adjusted > 0) {
		// Change FIFO threshold to remaining_adjusted
		status = sx127x_RegRead(chip->base_chip, sx127x_FSK_OOK_REG_35_FIFO_THRESH, values);
		if (status != sx127x_STATUS_OK) { return status; }
		values[0] &= ~sx127x_FSK_OOK_REG_35_FIFO_THRESH_FIFO_THRESH_MSK;
		values[0] |= ((remaining_adjusted - 1) & sx127x_FSK_OOK_REG_35_FIFO_THRESH_FIFO_THRESH_MSK);
		status = sx127x_RegWrite(chip->base_chip, sx127x_FSK_OOK_REG_35_FIFO_THRESH, values[0]);
		if (status != sx127x_STATUS_OK) { return status; }

		// Wait until FIFO threshold is reached
		status = __sx127x_FSK_OOK_RxReceive_IRQ(chip, RxTimeout_delay_ms, true, sx127x_FSK_OOK_REG_3F_IRQ_FLAGS2_FIFO_LEVEL_MSK, false);
		if (status != sx127x_STATUS_OK) { return status; }

		// Values already available in FIFO
		status = sx127x_RegReadMulti(chip->base_chip, sx127x_REG_00_FIFO, &data[i], remaining_adjusted);
		if (status != sx127x_STATUS_OK) { return status; }
		i += remaining_adjusted;
	}

	return sx127x_STATUS_OK;
}

sx127x_status_t sx127x_FSK_OOK_RxReceiveFixLen(sx127x_FSK_OOK_chip_t *chip, uint8_t *data) {
	if (!chip || chip->base_chip->modulation == sx127x_MODULATION_LORA || !data) { return sx127x_STATUS_ERROR; }

	sx127x_status_t status;

	status = __sx127x_FSK_OOK_RxReceive(chip, data, NULL);
	if (status != sx127x_STATUS_OK) { return status; }

	return sx127x_STATUS_OK;
}

sx127x_status_t sx127x_FSK_OOK_RxReceive(sx127x_FSK_OOK_chip_t *chip, uint8_t *data, uint8_t *len_ptr) {
	if (!chip || chip->base_chip->modulation == sx127x_MODULATION_LORA || !data || !len_ptr) { return sx127x_STATUS_ERROR; }

	sx127x_status_t status;

	status = __sx127x_FSK_OOK_RxReceive(chip, data, len_ptr);
	if (status != sx127x_STATUS_OK) { return status; }

	return sx127x_STATUS_OK;
}
