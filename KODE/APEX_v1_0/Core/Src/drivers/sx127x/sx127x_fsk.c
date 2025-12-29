#include "drivers/sx127x/sx127x_fsk.h"
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
        case SX127X_FSK_OKK_RxBw_100_0kHz: *mant = sx127x_FSK_OOK_REG_12_RX_BW_MANT_20; *exp = sx127x_FSK_OOK_REG_12_RX_BW_EXP_2; break;
        case SX127X_FSK_OOK_RxBw_125_0kHz: *mant = sx127x_FSK_OOK_REG_12_RX_BW_MANT_16; *exp = sx127x_FSK_OOK_REG_12_RX_BW_EXP_2; break;
        case SX127X_FSK_OOK_RxBw_166_7kHz: *mant = sx127x_FSK_OOK_REG_12_RX_BW_MANT_24; *exp = sx127x_FSK_OOK_REG_12_RX_BW_EXP_1; break;
        case SX127X_FSK_OOK_RxBw_200_0kHz: *mant = sx127x_FSK_OOK_REG_12_RX_BW_MANT_20; *exp = sx127x_FSK_OOK_REG_12_RX_BW_EXP_1; break;
        case SX127X_FSK_OOK_RxBw_250_0kHz: *mant = sx127x_FSK_OOK_REG_12_RX_BW_MANT_16; *exp = sx127x_FSK_OOK_REG_12_RX_BW_EXP_1; break;
        default: *mant = sx127x_FSK_OOK_REG_12_RX_BW_MANT_24; *exp = sx127x_FSK_OOK_REG_12_RX_BW_EXP_5; break;
    }
}

sx127x_status_t sx127x_FSK_Config(sx127x_chip_t *sx127x_chip, sx127x_FSK_OOK_config_t config) {
    if (!sx127x_chip || config.mode != SX127X_FSK_OOK_MODE_FSK) {
        return sx127x_STATUS_ERROR;
    }
    
    sx127x_status_t status;
    uint8_t value = 0x00;
    
    // Get chip frequency band
    uint8_t band;
    status = sx127x_GetBand(sx127x_chip, &band);
    if (status != sx127x_STATUS_OK) { return status; }
    
    // Set the mode to sleep (required to switch from LoRa to FSK or configure FSK)
    status = sx127x_RegRead(sx127x_chip, sx127x_REG_01_OP_MODE, &value);
    if (status != sx127x_STATUS_OK) { return status; }
    value &= ~sx127x_FSK_OOK_REG_01_OP_MODE_MODE_MSK;
    value |= sx127x_FSK_OOK_REG_01_OP_MODE_MODE_SLEEP;
    status = sx127x_RegWrite(sx127x_chip, sx127x_REG_01_OP_MODE, value);
    if (status != sx127x_STATUS_OK) { return status; }
    
    // Set the RegOpMode to FSK mode
    value = sx127x_FSK_OOK_REG_01_OP_MODE_LRM_FSK_OOK                       // FSK/OOK mode
          | sx127x_FSK_OOK_REG_01_OP_MODE_MOD_TYPE_FSK                      // FSK modulation
          | (band == sx127x_BAND_1 ? sx127x_FSK_OOK_REG_01_OP_MODE_LFMO_OFF : sx127x_FSK_OOK_REG_01_OP_MODE_LFMO_ON)
          | sx127x_FSK_OOK_REG_01_OP_MODE_MODE_SLEEP;                       // Sleep mode
    status = sx127x_RegWrite(sx127x_chip, sx127x_REG_01_OP_MODE, value);
    if (status != sx127x_STATUS_OK) { return status; }
    
    // Set bitrate (BitRate = Fxosc / BitRate register value)
    // BitRate register value = Fxosc / bitrate
    uint32_t bitrate_reg = (uint32_t)(sx127x_FXOSC / config.bitrate);
    uint8_t bitrate_msb = (uint8_t)(bitrate_reg >> 8);
    uint8_t bitrate_lsb = (uint8_t)(bitrate_reg & 0xFF);
    status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_02_BITRATE_MSB, bitrate_msb);
    if (status != sx127x_STATUS_OK) { return status; }
    status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_03_BITRATE_LSB, bitrate_lsb);
    if (status != sx127x_STATUS_OK) { return status; }
    
    // Set frequency deviation (FSK mode only)
    // Fdev = Fstep * Fdev register value
    // Fdev register value = Fdev / Fstep
    uint16_t fdev_reg = (uint16_t)(config.mod_config.fsk.freqDev / sx127x_FSTEP);
    uint8_t fdev_msb = (uint8_t)(fdev_reg >> 8);
    uint8_t fdev_lsb = (uint8_t)(fdev_reg & 0xFF);
    status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_04_FDEV_MSB, fdev_msb);
    if (status != sx127x_STATUS_OK) { return status; }
    status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_05_FDEV_LSB, fdev_lsb);
    if (status != sx127x_STATUS_OK) { return status; }
    
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
    
    // Set RxConfig - enable AGC and AFC
    value = sx127x_FSK_OOK_REG_0D_RX_CONFIG_AFCAON_ON   // AFC enabled
          | sx127x_FSK_OOK_REG_0D_RX_CONFIG_AGCAON_ON;  // AGC enabled
    status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_0D_RX_CONFIG, value);
    if (status != sx127x_STATUS_OK) { return status; }
    
    // Set RxBw (Receiver bandwidth)
    uint8_t rxbw_mant, rxbw_exp;
    sx127x_FSK_OOK_RxBw_to_RegValue(config.RxBw, &rxbw_mant, &rxbw_exp);
    value = rxbw_mant | rxbw_exp;
    status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_12_RX_BW, value);
    if (status != sx127x_STATUS_OK) { return status; }
    
    // Set AFC bandwidth (same as RxBw)
    status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_13_AFC_BW, value);
    if (status != sx127x_STATUS_OK) { return status; }
    
    // Set preamble length
    uint8_t preamble_msb = (uint8_t)(config.packetCfg.preamble_len >> 8);
    uint8_t preamble_lsb = (uint8_t)(config.packetCfg.preamble_len & 0xFF);
    status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_25_PREAMBLE_MSB, preamble_msb);
    if (status != sx127x_STATUS_OK) { return status; }
    status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_26_PREAMBLE_LSB, preamble_lsb);
    if (status != sx127x_STATUS_OK) { return status; }
    
    // Set sync word configuration
    value = (config.packetCfg.sync_on ? sx127x_FSK_OOK_REG_27_SYNC_CONFIG_SO_ON : sx127x_FSK_OOK_REG_27_SYNC_CONFIG_SO_OFF)
          | ((config.packetCfg.sync_len - 1) & sx127x_FSK_OOK_REG_27_SYNC_CONFIG_SYNC_SIZE_MSK);
    status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_27_SYNC_CONFIG, value);
    if (status != sx127x_STATUS_OK) { return status; }
    
    // Write sync word bytes
    if (config.packetCfg.sync_on && config.packetCfg.sync_len > 0) {
        for (uint8_t i = 0; i < config.packetCfg.sync_len && i < 8; i++) {
            status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_28_SYNC_VALUE_1 + i, config.packetCfg.sync_word[i]);
            if (status != sx127x_STATUS_OK) { return status; }
        }
    }
    
    // Set packet configuration 1
    value = (config.packetCfg.variable_length ? sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_PF_VARIABLE : sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_PF_FIXED)
          | sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_DCFREE_OFF
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
    
    // Update chip modulation mode
    sx127x_chip->modulation = sx127x_MODULATION_FSK_OOK;
    
    return sx127x_STATUS_OK;
}

sx127x_status_t sx127x_OOK_Config(sx127x_chip_t *sx127x_chip, sx127x_FSK_OOK_config_t config) {
    if (!sx127x_chip || config.mode != SX127X_FSK_OOK_MODE_OOK) {
        return sx127x_STATUS_ERROR;
    }
    
    sx127x_status_t status;
    uint8_t value = 0x00;
    
    // Get chip frequency band
    uint8_t band;
    status = sx127x_GetBand(sx127x_chip, &band);
    if (status != sx127x_STATUS_OK) { return status; }
    
    // Set the mode to sleep (required to switch from LoRa to OOK or configure OOK)
    status = sx127x_RegRead(sx127x_chip, sx127x_REG_01_OP_MODE, &value);
    if (status != sx127x_STATUS_OK) { return status; }
    value &= ~sx127x_FSK_OOK_REG_01_OP_MODE_MODE_MSK;
    value |= sx127x_FSK_OOK_REG_01_OP_MODE_MODE_SLEEP;
    status = sx127x_RegWrite(sx127x_chip, sx127x_REG_01_OP_MODE, value);
    if (status != sx127x_STATUS_OK) { return status; }
    
    // Set the RegOpMode to OOK mode
    value = sx127x_FSK_OOK_REG_01_OP_MODE_LRM_FSK_OOK                       // FSK/OOK mode
          | sx127x_FSK_OOK_REG_01_OP_MODE_MOD_TYPE_OOK                      // OOK modulation
          | (band == sx127x_BAND_1 ? sx127x_FSK_OOK_REG_01_OP_MODE_LFMO_OFF : sx127x_FSK_OOK_REG_01_OP_MODE_LFMO_ON)
          | sx127x_FSK_OOK_REG_01_OP_MODE_MODE_SLEEP;                       // Sleep mode
    status = sx127x_RegWrite(sx127x_chip, sx127x_REG_01_OP_MODE, value);
    if (status != sx127x_STATUS_OK) { return status; }
    
    // Set bitrate (BitRate = Fxosc / BitRate register value)
    uint32_t bitrate_reg = (uint32_t)(sx127x_FXOSC / config.bitrate);
    uint8_t bitrate_msb = (uint8_t)(bitrate_reg >> 8);
    uint8_t bitrate_lsb = (uint8_t)(bitrate_reg & 0xFF);
    status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_02_BITRATE_MSB, bitrate_msb);
    if (status != sx127x_STATUS_OK) { return status; }
    status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_03_BITRATE_LSB, bitrate_lsb);
    if (status != sx127x_STATUS_OK) { return status; }
    
    // Set PA ramp and modulation shaping (OOK uses different shaping values)
    value = config.modShaping | config.paRamp;
    status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_0A_PA_RAMP, value);
    if (status != sx127x_STATUS_OK) { return status; }
    
    // Set LNA boost
    value = sx127x_REG_0C_LNA_LNA_GAIN_G1           // Gain set to G1 (highest)
          | sx127x_REG_0C_LNA_LNA_BOOST_LF_DFT      // Normal LNA current (LF)
          | sx127x_REG_0C_LNA_LNA_BOOST_HF_BOOST;   // LNA current boosted by 150% (HF)
    status = sx127x_RegWrite(sx127x_chip, sx127x_REG_0C_LNA, value);
    if (status != sx127x_STATUS_OK) { return status; }
    
    // Set RxConfig - enable AGC
    value = sx127x_FSK_OOK_REG_0D_RX_CONFIG_AGCAON_ON;  // AGC enabled (AFC not applicable for OOK)
    status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_0D_RX_CONFIG, value);
    if (status != sx127x_STATUS_OK) { return status; }
    
    // Set RxBw (Receiver bandwidth)
    uint8_t rxbw_mant, rxbw_exp;
    sx127x_FSK_OOK_RxBw_to_RegValue(config.RxBw, &rxbw_mant, &rxbw_exp);
    value = rxbw_mant | rxbw_exp;
    status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_12_RX_BW, value);
    if (status != sx127x_STATUS_OK) { return status; }
    
    // Set OOK-specific configuration
    switch (config.mod_config.ook.ookCfg.thresh_type) {
        case SX127X_OOK_THRESH_FIXED:
            // Set fixed threshold
            value = sx127x_FSK_OOK_REG_14_OOK_PEAK_THRESH_TYPE_FIXED;
            status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_14_OOK_PEAK, value);
            if (status != sx127x_STATUS_OK) { return status; }
            status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_15_OOK_FIX, config.mod_config.ook.ookCfg.params.fixed.fixed_thresh);
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
    
    // Set preamble length
    uint8_t preamble_msb = (uint8_t)(config.packetCfg.preamble_len >> 8);
    uint8_t preamble_lsb = (uint8_t)(config.packetCfg.preamble_len & 0xFF);
    status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_25_PREAMBLE_MSB, preamble_msb);
    if (status != sx127x_STATUS_OK) { return status; }
    status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_26_PREAMBLE_LSB, preamble_lsb);
    if (status != sx127x_STATUS_OK) { return status; }
    
    // Set sync word configuration
    value = (config.packetCfg.sync_on ? sx127x_FSK_OOK_REG_27_SYNC_CONFIG_SO_ON : sx127x_FSK_OOK_REG_27_SYNC_CONFIG_SO_OFF)
          | ((config.packetCfg.sync_len - 1) & sx127x_FSK_OOK_REG_27_SYNC_CONFIG_SYNC_SIZE_MSK);
    status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_27_SYNC_CONFIG, value);
    if (status != sx127x_STATUS_OK) { return status; }
    
    // Write sync word bytes
    if (config.packetCfg.sync_on && config.packetCfg.sync_len > 0) {
        for (uint8_t i = 0; i < config.packetCfg.sync_len && i < 8; i++) {
            status = sx127x_RegWrite(sx127x_chip, sx127x_FSK_OOK_REG_28_SYNC_VALUE_1 + i, config.packetCfg.sync_word[i]);
            if (status != sx127x_STATUS_OK) { return status; }
        }
    }
    
    // Set packet configuration 1
    value = (config.packetCfg.variable_length ? sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_PF_VARIABLE : sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_PF_FIXED)
          | sx127x_FSK_OOK_REG_30_PACKET_CONFIG1_DCFREE_OFF
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
    
    // Update chip modulation mode
    sx127x_chip->modulation = sx127x_MODULATION_FSK_OOK;
    
    return sx127x_STATUS_OK;
}
