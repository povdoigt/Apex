#include "drivers/sx127x/sx127x_lora.h"
#include "drivers/sx127x/sx127x_common.h"
#include "stm32f4xx_hal.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>





static sx127x_status_t __sx127x_LORA_SetMode(sx127x_LORA_chip_t *chip, sx127x_LORA_REG_01_OP_MODE_MODE mode) {
	if (!chip || chip->base_chip->modulation != sx127x_MODULATION_LORA) { return sx127x_STATUS_ERROR; }
	sx127x_status_t status;
	uint8_t value = 0x00;

	// Read the current RegOpMode
	status = sx127x_RegRead(chip->base_chip, sx127x_REG_01_OP_MODE, &value);
	if (status != sx127x_STATUS_OK) { return status; }

	// Modify only the mode bits
	value &= ~sx127x_LORA_REG_01_OP_MODE_MODE_MSK;
	value |= mode;

	// Write back the new RegOpMode
	status = sx127x_RegWrite(chip->base_chip, sx127x_REG_01_OP_MODE, value);
	if (status != sx127x_STATUS_OK) { return status; }

	return sx127x_STATUS_OK;
}

static sx127x_status_t __sx127x_LORA_IsTxDone(sx127x_LORA_chip_t *chip, bool *isDone) {
	if (!chip || chip->base_chip->modulation != sx127x_MODULATION_LORA) { return sx127x_STATUS_ERROR; }
	sx127x_status_t status;
	uint8_t irq_flags;

	// Read the IRQ flags
	status = sx127x_RegRead(chip->base_chip, sx127x_LORA_REG_12_IRQ_FLAGS, &irq_flags);
	if (status != sx127x_STATUS_OK) { return status; }

	*isDone = irq_flags & sx127x_LORA_IRQ_FLAG_TX_DONE_MSK;

	// Clear the TX done flag
	if (*isDone) {
		status = sx127x_RegWrite(chip->base_chip, sx127x_LORA_REG_12_IRQ_FLAGS, sx127x_LORA_IRQ_FLAG_TX_DONE_MSK);
		if (status != sx127x_STATUS_OK) { return status; }
	}

	return sx127x_STATUS_OK;
}


sx127x_status_t sx127x_LORA_Config(sx127x_LORA_chip_t *chip, sx127x_base_chip_t *base_chip, sx127x_LORA_config_t config) {
	sx127x_status_t status;
	uint8_t value = 0x00;

	chip->base_chip = base_chip;

	chip->config = config;
	chip->base_chip->modulation = sx127x_MODULATION_LORA;

	// Get chip frequency band
	uint8_t band;
	status = sx127x_GetBand(chip->base_chip, &band);
	if (status != sx127x_STATUS_OK) { return status; }

	// Set the mode to sleep (need to be in sleep mode to set LoRa mode)
	status = __sx127x_LORA_SetMode(chip, sx127x_LORA_REG_01_OP_MODE_MODE_SLEEP);
	if (status != sx127x_STATUS_OK) { return status; }

	// Set the RegOpMode
	value = sx127x_LORA_REG_01_OP_MODE_LRM_LoRa		// LoRa mode
		  | sx127x_LORA_REG_01_OP_MODE_ASR_LoRa		// LoRa access mode
		  | (band == sx127x_BAND_1 ? sx127x_LORA_REG_01_OP_MODE_LFMO_OFF : sx127x_LORA_REG_01_OP_MODE_LFMO_ON)
		  | sx127x_LORA_REG_01_OP_MODE_MODE_SLEEP;	// Sleep mode (normaly already set)
	status = sx127x_RegWrite(chip->base_chip, sx127x_REG_01_OP_MODE, value);
	if (status != sx127x_STATUS_OK) { return status; }

	// Set the fifo rx and tx base address to 0x00
	status = sx127x_RegWrite(chip->base_chip, sx127x_LORA_REG_0F_FIFO_RX_BASE_ADDR, 0x00);
	if (status != sx127x_STATUS_OK) { return status; }
	status = sx127x_RegWrite(chip->base_chip, sx127x_LORA_REG_0E_FIFO_TX_BASE_ADDR, 0x80);
	if (status != sx127x_STATUS_OK) { return status; }

	// Set the Modem Config1
	value = config.bandwidth		// Bandwidth
		  | config.codingRate		// Coding Rate
		  | config.implicitHeader;	// Implicit Header mode
	status = sx127x_RegWrite(chip->base_chip, sx127x_LORA_REG_1D_MODEM_CONFIG1, value);
	if (status != sx127x_STATUS_OK) { return status; }

	// Set the Modem Config2
	value = config.spreadingFactor						// Spreading Factor
		  | sx127x_LORA_REG_1E_MODEM_CONFIG2_TXCM_OFF	// Normal Tx mode
		  | (config.crcEnabled ? sx127x_LORA_REG_1E_MODEM_CONFIG2_RPC_ON : sx127x_LORA_REG_1E_MODEM_CONFIG2_RPC_OFF); // CRC
	status = sx127x_RegWrite(chip->base_chip, sx127x_LORA_REG_1E_MODEM_CONFIG2, value);
	if (status != sx127x_STATUS_OK) { return status; }

	// Set the Modem Config3
	value = sx127x_LORA_REG_26_MODEM_CONFIG3_LDRO_OFF	// LowDataRateOptimize disabled
		  | sx127x_LORA_REG_26_MODEM_CONFIG3_AGCAO_ON;	// LNA gain set by AGC loop
	status = sx127x_RegWrite(chip->base_chip, sx127x_LORA_REG_26_MODEM_CONFIG3, value);
	if (status != sx127x_STATUS_OK) { return status; }

	return sx127x_STATUS_OK;
}

sx127x_status_t sx127x_LORA_TxSend(sx127x_LORA_chip_t *chip, const uint8_t *data, uint8_t len) {
	if (!chip || chip->base_chip->modulation != sx127x_MODULATION_LORA) { return sx127x_STATUS_ERROR; }
	sx127x_status_t status;

	len = len > 128 ? 128 : len; // Max payload length is 128 bytes

	// Set the mode to standby
	status = __sx127x_LORA_SetMode(chip, sx127x_LORA_REG_01_OP_MODE_MODE_STDBY);
	if (status != sx127x_STATUS_OK) { return status; }

	// Set the fifo pointer to TxBaseAddr (0x80)
	status = sx127x_RegWrite(chip->base_chip, sx127x_LORA_REG_0D_FIFO_ADDR_PTR, 0x80);
	if (status != sx127x_STATUS_OK) { return status; }

	// Write the data to the fifo
	status = sx127x_RegWriteMulti(chip->base_chip, sx127x_REG_00_FIFO, data, len);
	if (status != sx127x_STATUS_OK) { return status; }

	// Set the payload length
	status = sx127x_RegWrite(chip->base_chip, sx127x_LORA_REG_22_PAYLOAD_LENGTH, len);
	if (status != sx127x_STATUS_OK) { return status; }

	// Set the mode to transmit
	status = __sx127x_LORA_SetMode(chip, sx127x_LORA_REG_01_OP_MODE_MODE_TX);
	if (status != sx127x_STATUS_OK) { return status; }

	// Check TxDone flag
	bool isDone = false;
	while (!isDone) {
		status = __sx127x_LORA_IsTxDone(chip, &isDone);
		if (status != sx127x_STATUS_OK) { return status; }
	}

	return sx127x_STATUS_OK;
}

sx127x_status_t sx127x_LORA_RxReceive(sx127x_LORA_chip_t *chip, uint8_t *buff, uint8_t *len) {
	if (!chip || chip->base_chip->modulation != sx127x_MODULATION_LORA) { return sx127x_STATUS_ERROR; }
	sx127x_status_t status;
	uint8_t packetLength = 0;
	uint8_t value;

	// Set the mode to Rx continuous
	status = __sx127x_LORA_SetMode(chip, sx127x_LORA_REG_01_OP_MODE_MODE_RX_CONTINUOUS);
	if (status != sx127x_STATUS_OK) { return status; }

	// Check for irq flags
	uint32_t t0 = HAL_GetTick();
	while (true) {
		status = sx127x_RegRead(chip->base_chip, sx127x_LORA_REG_12_IRQ_FLAGS, &value);
		if (status != sx127x_STATUS_OK) { return status; }
		if (value & sx127x_LORA_IRQ_FLAG_RX_DONE_MSK) {
			break;
		}
		if ((HAL_GetTick() - t0) > 10) {	// Timeout of 10 ms (adjustable)
			// No packet received
			*len = 0;
			return sx127x_STATUS_MODEM_TIMEOUT;
		}
	}

	// Clear the RX done flag
	status = sx127x_RegWrite(chip->base_chip, sx127x_LORA_REG_12_IRQ_FLAGS, sx127x_LORA_IRQ_FLAG_RX_DONE_MSK);
	if (status != sx127x_STATUS_OK) { return status; }
	
	// Check if it's a valid packet with a valid CRC
	if (!(value & sx127x_LORA_IRQ_FLAG_VALID_HEADER_MSK) || (value & sx127x_LORA_IRQ_FLAG_PAYLOAD_CRC_ERROR_MSK)) {
		// Invalid packet
		*len = 0;
		return sx127x_STATUS_OK;
	}

	// Read packet length
	status = sx127x_RegRead(chip->base_chip, sx127x_LORA_REG_13_RX_NB_BYTES, &packetLength);
	if (status != sx127x_STATUS_OK) { return status; }
	*len = packetLength;

	// Set FIFO address to current RX address
	status = sx127x_RegRead(chip->base_chip, sx127x_LORA_REG_10_FIFO_RX_CURRENT_ADDR, &value);
	if (status != sx127x_STATUS_OK) { return status; }
	status = sx127x_RegWrite(chip->base_chip, sx127x_LORA_REG_0D_FIFO_ADDR_PTR, value);
	if (status != sx127x_STATUS_OK) { return status; }

	// Read the data from the fifo
	status = sx127x_RegReadMulti(chip->base_chip, sx127x_REG_00_FIFO, buff, packetLength);
	if (status != sx127x_STATUS_OK) { return status; }

	// Reset the RxBaseAddr to 0x00
	status = sx127x_RegWrite(chip->base_chip, sx127x_LORA_REG_0F_FIFO_RX_BASE_ADDR, 0x00);
	if (status != sx127x_STATUS_OK) { return status; }

	return sx127x_STATUS_OK;
}
