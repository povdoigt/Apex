#include "drivers/sx127x/sx127x_fsk_ook.h"
#include "drivers/sx127x/sx127x_common.h"
#include <stdint.h>

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

static sx127x_status_t __sx127x_FSK_OOK_Config(sx127x_chip_t *sx127x_chip, sx127x_FSK_OOK_config_t config, sx127x_modulation_t modulation) {
    if (!sx127x_chip || modulation == sx127x_MODULATION_LORA) { return sx127x_STATUS_ERROR; }
    
    sx127x_status_t status;
    uint8_t value = 0x00;
    
    // Get chip frequency band
    uint8_t band;
    status = sx127x_GetBand(sx127x_chip, &band);
    if (status != sx127x_STATUS_OK) { return status; }
    
    // Set the mode to sleep (required to switch from LoRa to FSK or configure FSK)
    status = sx127x_FSK_OOK_ChangeMode(sx127x_chip, sx127x_FSK_OOK_REG_01_OP_MODE_MODE_SLEEP);
	if (status != sx127x_STATUS_OK) { return status; }
    
    // Set the RegOpMode to FSK/OOK mode
    value = sx127x_FSK_OOK_REG_01_OP_MODE_LRM_FSK_OOK	// FSK/OOK mode
    	  | (modulation == sx127x_MODULATION_FSK ? sx127x_FSK_OOK_REG_01_OP_MODE_MOD_TYPE_FSK : sx127x_FSK_OOK_REG_01_OP_MODE_MOD_TYPE_OOK) // Modulation type
    	  | (band == sx127x_BAND_1 ? sx127x_FSK_OOK_REG_01_OP_MODE_LFMO_OFF : sx127x_FSK_OOK_REG_01_OP_MODE_LFMO_ON)
    	  | sx127x_FSK_OOK_REG_01_OP_MODE_MODE_SLEEP;	// Sleep mode
    status = sx127x_RegWrite(sx127x_chip, sx127x_REG_01_OP_MODE, value);
    if (status != sx127x_STATUS_OK) { return status; }
    
    // Set bitrate (BitRate = Fxosc / BitRate register value)
    // BitRate register value = Fxosc / bitrate
    uint32_t bitrate_reg = (uint32_t)(sx127x_FXOSC / config.bitrate);
    uint8_t bitrate_buf[2] = {
        (uint8_t)((bitrate_reg >> 8) & 0xFF),
        (uint8_t)((bitrate_reg >> 0) & 0xFF) };
    status = sx127x_RegWriteMulti(sx127x_chip, sx127x_FSK_OOK_REG_02_BITRATE_MSB, bitrate_buf, 2);
    if (status != sx127x_STATUS_OK) { return status; }

	// Set RxConfig - enable AGC, disable AFC
	value = sx127x_FSK_OOK_REG_0D_RX_CONFIG_RROC_ON		// Restart Rx on collision
		  | sx127x_FSK_OOK_REG_0D_RX_CONFIG_AFCAON_OFF	// AFC disabled
		  | sx127x_FSK_OOK_REG_0D_RX_CONFIG_AGCAON_ON;	// AGC enabled
	status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_0D_RX_CONFIG, value);
	if (status != sx127x_STATUS_OK) { return status; }

	// Set PA ramp and modulation shaping
	value = config.modShaping
		  | config.paRamp;	// Useful only for FSK modulation 
	status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_0A_PA_RAMP, value);
	if (status != sx127x_STATUS_OK) { return status; }

	// Set RxBw (Receiver bandwidth)
	uint8_t rxbw_mant, rxbw_exp;
	sx127x_FSK_OOK_RxBw_to_RegValue(config.RxBw, &rxbw_mant, &rxbw_exp);
	status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_12_RX_BW, rxbw_mant | rxbw_exp);
	if (status != sx127x_STATUS_OK) { return status; }

	// Set preamble length
	uint8_t preamble_buf[2] = {
		(uint8_t)((config.packetCfg.preamble_len >> 8) & 0xFF),
		(uint8_t)((config.packetCfg.preamble_len >> 0) & 0xFF) };
	status = sx127x_RegWriteMulti(sx127x_chip, sx127x_FSK_OOK_REG_25_PREAMBLE_MSB, preamble_buf, 2);
	if (status != sx127x_STATUS_OK) { return status; }

	// Set sync word configuration
	value = (config.packetCfg.sync_on ? sx127x_FSK_OOK_REG_27_SYNC_CONFIG_SO_ON : sx127x_FSK_OOK_REG_27_SYNC_CONFIG_SO_OFF)
			| ((config.packetCfg.sync_len - 1) & sx127x_FSK_OOK_REG_27_SYNC_CONFIG_SYNC_SIZE_MSK);
	status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_27_SYNC_CONFIG, value);
	if (status != sx127x_STATUS_OK) { return status; }

	// Write sync word bytes
	if (config.packetCfg.sync_on && config.packetCfg.sync_len > 0) {
		uint8_t write_len = (config.packetCfg.sync_len > 8) ? 8 : config.packetCfg.sync_len;
		status = sx127x_RegWriteMulti(sx127x_chip, sx127x_FSK_OOK_REG_28_SYNC_VALUE_1, config.packetCfg.sync_word, write_len);
		if (status != sx127x_STATUS_OK) { return status; }
	}

	// Set packet configuration 1
	value = (config.packetCfg.variable_length ? sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_PF_VARIABLE : sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_PF_FIXED)
			| sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_DCFREE_WHITENING
			| (config.packetCfg.crc_on ? sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_CRCON_ON : sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_CRCON_OFF)
			| sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_AF_OFF;
	status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_30_PACKET_CONFIG1, value);
	if (status != sx127x_STATUS_OK) { return status; }

	// Set packet configuration 2 - packet mode
	value = sx127x_FSK_OOK_REG_31_PACKET_CONFIG2_DM_PACKET;
	status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_31_PACKET_CONFIG2, value);
	if (status != sx127x_STATUS_OK) { return status; }

	// Set payload length
	status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_32_PAYLOAD_LENGTH, config.packetCfg.payload_max_len);
	if (status != sx127x_STATUS_OK) { return status; }

	// Set FIFO threshold
	value = sx127x_FSK_OOK_REG_35_FIFO_THRESH_TX_START_COND_LEVEL
			| (config.fifoThresh & sx127x_FSK_OOK_REG_35_FIFO_THRESH_FIFO_THRESH_MSK);
	status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_35_FIFO_THRESH, value);
	if (status != sx127x_STATUS_OK) { return status; }


    return sx127x_STATUS_OK;
}

sx127x_status_t sx127x_FSK_Config(sx127x_chip_t *sx127x_chip, sx127x_FSK_OOK_config_t config) {
	if (!sx127x_chip) { return sx127x_STATUS_ERROR; }

	sx127x_status_t status;

	status = __sx127x_FSK_OOK_Config(sx127x_chip, config, sx127x_MODULATION_FSK);
	if (status != sx127x_STATUS_OK) { return status; }

	// Set frequency deviation (FSK mode only)
	// Fdev = Fstep * Fdev register value
	// Fdev register value = Fdev / Fstep
	uint16_t fdev_reg = (uint16_t)((float)config.mod_config.fsk.fdev / sx127x_FSTEP);
	uint8_t fdev_buf[2] = {
		(uint8_t)((fdev_reg >> 8) & 0xFF),
		(uint8_t)((fdev_reg >> 0) & 0xFF) };
	status = sx127x_RegWriteMulti(sx127x_chip, sx127x_FSK_OOK_REG_04_FDEV_MSB, fdev_buf, 2);
	if (status != sx127x_STATUS_OK) { return status; }

	// Update chip modulation mode
	sx127x_chip->modulation = sx127x_MODULATION_FSK;

	return sx127x_STATUS_OK;
}

sx127x_status_t sx127x_OOK_Config(sx127x_chip_t *sx127x_chip, sx127x_FSK_OOK_config_t config) {
    if (!sx127x_chip) {
        return sx127x_STATUS_ERROR;
    }
    
    sx127x_status_t status;
    uint8_t value = 0x00;

	status = __sx127x_FSK_OOK_Config(sx127x_chip, config, sx127x_MODULATION_OOK);
	if (status != sx127x_STATUS_OK) { return status; }
    
    // Set OOK-specific configuration
    switch (config.mod_config.ook.ookCfg.thresh_type) {
        case SX127X_OOK_THRESH_FIXED:
            // Set fixed threshold
            value = sx127x_FSK_OOK_REG_14_OOK_PEAK_THRESH_TYPE_FIXED;
            status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_14_OOK_PEAK, value);
            if (status != sx127x_STATUS_OK) { return status; }
			value = config.mod_config.ook.ookCfg.params.fixed.fixed_thresh;
            status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_15_OOK_FIX, value);
            if (status != sx127x_STATUS_OK) { return status; }
            break;
            
        case SX127X_OOK_THRESH_PEAK:
            // Set peak threshold
            value = config.mod_config.ook.ookCfg.params.peak.peak_type
                  | config.mod_config.ook.ookCfg.params.peak.peak_step;
            status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_14_OOK_PEAK, value);
            if (status != sx127x_STATUS_OK) { return status; }
            break;
            
        case SX127X_OOK_THRESH_AVERAGE:
            // Set average threshold
            value = sx127x_FSK_OOK_REG_14_OOK_PEAK_THRESH_TYPE_AVG;
            status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_14_OOK_PEAK, value);
            if (status != sx127x_STATUS_OK) { return status; }
            value = config.mod_config.ook.ookCfg.params.average.avg_peak_dec
                  | config.mod_config.ook.ookCfg.params.average.avg_offset
                  | config.mod_config.ook.ookCfg.params.average.avg_filt;
            status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_16_OOK_AVG, value);
            if (status != sx127x_STATUS_OK) { return status; }
            break;
    }
    
    // Update chip modulation mode
    sx127x_chip->modulation = sx127x_MODULATION_OOK;
    
    return sx127x_STATUS_OK;
}

sx127x_status_t sx127x_FSK_OOK_ChangeMode(sx127x_chip_t *sx127x_chip, sx127x_FSK_OOK_REG_01_OP_MODE_MODE mode) {
	if (!sx127x_chip) { return sx127x_STATUS_ERROR; }

	sx127x_status_t status;
	uint8_t value;
	status = sx127x_RegRead(sx127x_chip, sx127x_REG_01_OP_MODE, &value);
	if (status != sx127x_STATUS_OK) { return status; }
	value &= ~sx127x_FSK_OOK_REG_01_OP_MODE_MODE_MSK;
	value |= mode;
	status = sx127x_RegWrite(sx127x_chip, sx127x_REG_01_OP_MODE, value);
	if (status != sx127x_STATUS_OK) { return status; }

	return sx127x_STATUS_OK;
}