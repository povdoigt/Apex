#include "drivers/sx127x/sx127x_lora.h"
#include "drivers/sx127x/sx127x_common.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>


sx127x_status_t sx127x_LORA_Init(sx127x_chip_t *chip, sx127x_lora_config_t config) {
	sx127x_status_t status;
	uint8_t value = 0x00;

	// Set the mode to sleep (need to be in sleep mode to set LoRa mode)
	status = sx127x_LORA_ChangeMode(chip, sx127x_LORA_REG_01_OP_MODE_MODE_SLEEP);
	if (status != sx127x_STATUS_OK) { return status; }

	// Set the RegOpMode
	value = 0x00;
	value |= sx127x_LORA_REG_01_OP_MODE_LRM_LoRa;	// LoRa mode
	value |= sx127x_LORA_REG_01_OP_MODE_MODE_SLEEP; // Sleep mode (normaly already set)
	status = sx127x_RegWrite(chip, sx127x_REG_01_OP_MODE, value);
	if (status != sx127x_STATUS_OK) { return status; }

	// Set the mode to standby
	status = sx127x_LORA_ChangeMode(chip, sx127x_LORA_REG_01_OP_MODE_MODE_STDBY);
	if (status != sx127x_STATUS_OK) { return status; }

	// Clear all IRQ flags
	status = sx127x_RegWrite(chip, sx127x_LORA_REG_12_IRQ_FLAGS, 0xFF);
	if (status != sx127x_STATUS_OK) { return status; }
	// DEBUG: Verify IRQ flags are cleared
	status = sx127x_RegRead(chip, sx127x_LORA_REG_12_IRQ_FLAGS, &value);
	if (status != sx127x_STATUS_OK) { return status; }
	if (value != 0x00) { return sx127x_STATUS_ERROR; }

	// Set the fifo rx and tx base address to 0x00
	status = sx127x_RegWrite(chip, sx127x_LORA_REG_0F_FIFO_RX_BASE_ADDR, 0x00);
	if (status != sx127x_STATUS_OK) { return status; }
	status = sx127x_RegWrite(chip, sx127x_LORA_REG_0E_FIFO_TX_BASE_ADDR, 0x80);
	if (status != sx127x_STATUS_OK) { return status; }

	// Set lna boost
	value = 0x00;
	value |= sx127x_REG_0C_LNA_LNA_GAIN_G1; // Gain set to G1 (highest)
	value |= sx127x_REG_0C_LNA_LNA_BOOST_LF_DFT; // Normal LNA current (LF)
	value |= sx127x_REG_0C_LNA_LNA_BOOST_HF_BOOST; // LNA current boosted by 150% (HF)
	status = sx127x_RegWrite(chip, sx127x_REG_0C_LNA, value);
	if (status != sx127x_STATUS_OK) { return status; }

	// Set the Modem Config1
	value = 0x00;
	value |= config.bandwidth;			// Bandwidth
	value |= config.codingRate;		// Coding Rate
	value |= config.implicitHeader;	// Implicit Header mode
	status = sx127x_RegWrite(chip, sx127x_LORA_REG_1D_MODEM_CONFIG1, value);
	if (status != sx127x_STATUS_OK) { return status; }

	// Set the Modem Config2
	value = 0x00;
	value |= config.spreadingFactor;	// Spreading Factor
	value |= sx127x_LORA_REG_1E_MODEM_CONFIG2_TXCM_OFF; // Normal Tx mode
	value |= config.crcEnabled; // CRC enabled
	status = sx127x_RegWrite(chip, sx127x_LORA_REG_1E_MODEM_CONFIG2, value);
	if (status != sx127x_STATUS_OK) { return status; }

	// Set the Modem Config3
	value = 0x00;
	value |= sx127x_LORA_REG_26_MODEM_CONFIG3_LDRO_OFF; // LowDataRateOptimize disabled
	value |= sx127x_LORA_REG_26_MODEM_CONFIG3_AGCAO_ON; // LNA gain set by AGC loop
	status = sx127x_RegWrite(chip, sx127x_LORA_REG_26_MODEM_CONFIG3, value);
	if (status != sx127x_STATUS_OK) { return status; }

	return sx127x_STATUS_OK;
}


sx127x_status_t sx127x_LORA_ChangeMode(sx127x_chip_t *chip, sx127x_LORA_REG_01_OP_MODE_MODE mode) {
	sx127x_status_t status;
	uint8_t value = 0x00;

	// Read the current RegOpMode
	status = sx127x_RegRead(chip, sx127x_REG_01_OP_MODE, &value);
	if (status != sx127x_STATUS_OK) { return status; }

	// Modify only the mode bits
	value &= ~sx127x_LORA_REG_01_OP_MODE_MODE_MSK;
	value |= mode;

	// Write back the new RegOpMode
	status = sx127x_RegWrite(chip, sx127x_REG_01_OP_MODE, value);
	if (status != sx127x_STATUS_OK) { return status; }

	return sx127x_STATUS_OK;
}

sx127x_status_t sx127x_LORA_IsTxDone(sx127x_chip_t *chip, bool *isDone) {
	sx127x_status_t status;
	uint8_t irq_flags;

	// Read the IRQ flags
	status = sx127x_RegRead(chip, sx127x_LORA_REG_12_IRQ_FLAGS, &irq_flags);
	if (status != sx127x_STATUS_OK) { return status; }

	*isDone = irq_flags & sx127x_LORA_IRQ_FLAG_TX_DONE_MSK;

	// Clear the TX done flag
	if (*isDone) {
		status = sx127x_RegWrite(chip, sx127x_LORA_REG_12_IRQ_FLAGS, sx127x_LORA_IRQ_FLAG_TX_DONE_MSK);
		if (status != sx127x_STATUS_OK) { return status; }
	}

	return sx127x_STATUS_OK;
}

sx127x_status_t sx127x_LORA_TxSend(sx127x_chip_t *chip, uint8_t *data, uint8_t len) {
	sx127x_status_t status;

	len = len > 128 ? 128 : len; // Max payload length is 128 bytes

	// Set the mode to standby
	status = sx127x_LORA_ChangeMode(chip, sx127x_LORA_REG_01_OP_MODE_MODE_STDBY);
	if (status != sx127x_STATUS_OK) { return status; }

	// Set the fifo pointer to TxBaseAddr (0x80)
	status = sx127x_RegWrite(chip, sx127x_LORA_REG_0D_FIFO_ADDR_PTR, 0x80);
	if (status != sx127x_STATUS_OK) { return status; }

	// Write the data to the fifo
	status = sx127x_RegWriteMulti(chip, sx127x_REG_00_FIFO, data, len);
	if (status != sx127x_STATUS_OK) { return status; }

	// Set the payload length
	status = sx127x_RegWrite(chip, sx127x_LORA_REG_22_PAYLOAD_LENGTH, len);
	if (status != sx127x_STATUS_OK) { return status; }

	// Set the mode to transmit
	status = sx127x_LORA_ChangeMode(chip, sx127x_LORA_REG_01_OP_MODE_MODE_TX);
	if (status != sx127x_STATUS_OK) { return status; }

	// Check TxDone flag
	bool isDone = false;
	while (!isDone) {
		status = sx127x_LORA_IsTxDone(chip, &isDone);
		if (status != sx127x_STATUS_OK) { return status; }
	}

	return sx127x_STATUS_OK;
}

sx127x_status_t sx127x_LORA_RxReceive(sx127x_chip_t *chip, uint8_t *buff, uint8_t *len) {
	sx127x_status_t status;
	uint8_t packetLength = 0;
	uint8_t value;

	// Check for irq flags
	status = sx127x_RegRead(chip, sx127x_LORA_REG_12_IRQ_FLAGS, &value);
	if (status != sx127x_STATUS_OK) { return status; }
	if (!(value & sx127x_LORA_IRQ_FLAG_RX_DONE_MSK)) {
		// No packet received
		*len = 0;
		return sx127x_STATUS_OK;
	}

	// Clear the RX done flag
	status = sx127x_RegWrite(chip, sx127x_LORA_REG_12_IRQ_FLAGS, sx127x_LORA_IRQ_FLAG_RX_DONE_MSK);
	if (status != sx127x_STATUS_OK) { return status; }
	
	// Check if it's a valid packet with a valid CRC
	if (!(value & sx127x_LORA_IRQ_FLAG_VALID_HEADER_MSK) && (value & sx127x_LORA_IRQ_FLAG_PAYLOAD_CRC_ERROR_MSK)) {
		// Invalid packet
		*len = 0;
		return sx127x_STATUS_OK;
	}

	// read packet length
	status = sx127x_RegRead(chip, sx127x_LORA_REG_13_RX_NB_BYTES, &packetLength);
	if (status != sx127x_STATUS_OK) { return status; }
	*len = packetLength;

	// set FIFO address to current RX address
	status = sx127x_RegRead(chip, sx127x_LORA_REG_10_FIFO_RX_CURRENT_ADDR, &value);
	if (status != sx127x_STATUS_OK) { return status; }
	status = sx127x_RegWrite(chip, sx127x_LORA_REG_0D_FIFO_ADDR_PTR, value);
	if (status != sx127x_STATUS_OK) { return status; }

	// read the data from the fifo
	for (uint8_t i = 0; i < packetLength; i++) {
		status = sx127x_RegRead(chip, sx127x_REG_00_FIFO, &buff[i]);
		if (status != sx127x_STATUS_OK) { return status; }
	}

	// Reset the RxBaseAddr to 0x00
	status = sx127x_RegWrite(chip, sx127x_LORA_REG_0F_FIFO_RX_BASE_ADDR, 0x00);
	if (status != sx127x_STATUS_OK) { return status; }

	return sx127x_STATUS_OK;
}
