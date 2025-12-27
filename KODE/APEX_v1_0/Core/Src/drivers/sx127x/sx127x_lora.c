#include "drivers/sx127x/sx127x_lora.h"
#include "drivers/sx127x/sx127x_common.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>


sx127x_status_t sx127x_LORA_Init(sx127x_chip_t *chip, sx127x_lora_config_t *config) {
	sx127x_status_t status;
	uint8_t value = 0x00;

	// Set the mode to sleep (need to be in sleep mode to set LoRa mode)
	sx127x_RegRead(chip, sx127x_REG_01_OP_MODE, &value);
	value &= ~sx127x_LORA_REG_01_OP_MODE_MODE_MSK;
	value |= sx127x_LORA_REG_01_OP_MODE_MODE_SLEEP; // Sleep mode
	sx127x_RegWrite(chip, sx127x_REG_01_OP_MODE, value);

	// Set the RegOpMode
	value = 0x00;
	value |= sx127x_LORA_REG_01_OP_MODE_LRM_LoRa;	// LoRa mode
	value |= sx127x_LORA_REG_01_OP_MODE_MODE_SLEEP; // Sleep mode (normaly already set)
	sx127x_RegWrite(chip, sx127x_REG_01_OP_MODE, value);

	// Set the fifo rx and tx base address to 0x00
	sx127x_RegWrite(chip, sx127x_LORA_REG_0F_FIFO_RX_BASE_ADDR, 0x00);
	sx127x_RegWrite(chip, sx127x_LORA_REG_0E_FIFO_TX_BASE_ADDR, 0x00);

	// Set lna boost
	value = 0x00;
	value |= sx127x_REG_0C_LNA_LNA_GAIN_G1; // Gain set to G1 (highest)
	value |= sx127x_REG_0C_LNA_LNA_BOOST_LF_DFT; // Normal LNA current (LF)
	value |= sx127x_REG_0C_LNA_LNA_BOOST_HF_BOOST; // LNA current boosted by 150% (HF)
	sx127x_RegWrite(chip, sx127x_REG_0C_LNA, value);

	// Set the Modem Config1
	value = 0x00;
	value |= config->bandwidth;			// Bandwidth
	value |= config->codingRate;		// Coding Rate
	value |= config->implicitHeader;	// Implicit Header mode
	sx127x_RegWrite(chip, sx127x_LORA_REG_1D_MODEM_CONFIG1, value);

	// Set the Modem Config2
	value = 0x00;
	value |= config->spreadingFactor;	// Spreading Factor
	value |= sx127x_LORA_REG_1E_MODEM_CONFIG2_TXCM_OFF; // Normal Tx mode
	value |= config->crcEnabled; // CRC enabled
	sx127x_RegWrite(chip, sx127x_LORA_REG_1E_MODEM_CONFIG2, value);

	// Set the Modem Config3
	value = 0x00;
	value |= sx127x_LORA_REG_26_MODEM_CONFIG3_LDRO_OFF; // LowDataRateOptimize disabled
	value |= sx127x_LORA_REG_26_MODEM_CONFIG3_AGCAO_ON; // LNA gain set by AGC loop
	sx127x_RegWrite(chip, sx127x_LORA_REG_26_MODEM_CONFIG3, value);

	// Set the output power to 10 dBm
	// ON PEUT MONTER A 13 VOIRE 25
	sx127x_LORA_SetTxPower(sx127x_LORA_chip, 20);


	// Set the mode to5 standby
	sx127x_LORA_LoRaStandBy(sx127x_LORA_chip);
}


void sx127x_LORA_Reset(sx127x_LORA_Chip *sx127x_LORA_chip) {
	HAL_GPIO_WritePin(sx127x_LORA_chip->resetPinBank, sx127x_LORA_chip->resetPin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(sx127x_LORA_chip->resetPinBank, sx127x_LORA_chip->resetPin, GPIO_PIN_SET);
	HAL_Delay(10);
}


uint8_t sx127x_LORA_GetVersion(sx127x_LORA_Chip *sx127x_LORA_chip) {
	uint8_t tx_buf[2] = {sx127x_LORA_LORA_REG_42_VERSION | sx127x_LORA_REG_READ_MASK, 0};
	uint8_t rx_buf[1];
	sx127x_LORA_TransmitReceive(sx127x_LORA_chip, tx_buf, rx_buf, 1, 1);
	return rx_buf[0];
}


void sx127x_LORA_LoRaSleep(sx127x_LORA_Chip *sx127x_LORA_chip) {
	uint8_t tx_buf[2] = {sx127x_LORA_LORA_REG_01_OP_MODE  | sx127x_LORA_REG_WRITE_MASK,
						 sx127x_LORA_LORA_LONG_RANGE_MODE | sx127x_LORA_LORA_MODE_SLEEP};
	sx127x_LORA_TransmitReceive(sx127x_LORA_chip, tx_buf, NULL, 2, 0);
}


void sx127x_LORA_LoRaStandBy(sx127x_LORA_Chip *sx127x_LORA_chip) {
	uint8_t tx_buf[2] = {sx127x_LORA_LORA_REG_01_OP_MODE  | sx127x_LORA_REG_WRITE_MASK,
						 sx127x_LORA_LORA_LONG_RANGE_MODE | sx127x_LORA_LORA_MODE_STDBY};
	sx127x_LORA_TransmitReceive(sx127x_LORA_chip, tx_buf, NULL, 2, 0);
}


void sx127x_LORA_SetFrequency(sx127x_LORA_Chip *sx127x_LORA_chip, double frequency) {
	uint64_t frf = ((uint64_t)frequency / sx127x_LORA_FSTEP);
	uint8_t tx_buf[4] = {sx127x_LORA_LORA_REG_06_FRF_MSB | sx127x_LORA_REG_WRITE_MASK,
						 (uint8_t)(frf >> 16),
						 (uint8_t)(frf >>  8),
						 (uint8_t)(frf >>  0)};
	sx127x_LORA_TransmitReceive(sx127x_LORA_chip, tx_buf, NULL, 4, 0);
}


double sx127x_LORA_GetFrequency(sx127x_LORA_Chip *sx127x_LORA_chip) {
	uint8_t rx_buf[3];
	sx127x_LORA_ReadRegisters(sx127x_LORA_chip, sx127x_LORA_LORA_REG_06_FRF_MSB, rx_buf, 3);
	uint64_t frf = ((uint64_t)rx_buf[0] << 16) |
	               ((uint64_t)rx_buf[1] <<  8) |
				   ((uint64_t)rx_buf[2] <<  0);
	return (double)(frf * sx127x_LORA_FSTEP);
}


void sx127x_LORA_SetTxPower(sx127x_LORA_Chip *sx127x_LORA_chip, int8_t power) {
	if (power > 17) {
		if (power > 20) {
			power = 20;
		}
		power -= 3;
		sx127x_LORA_WriteRegister(sx127x_LORA_chip, sx127x_LORA_LORA_REG_4D_PA_DAC, sx127x_LORA_PA_DAC_ENABLE);
		sx127x_LORA_SetOCP(sx127x_LORA_chip, 140);
	} else {
		if (power < 2) {
			power = 2;
		}
		sx127x_LORA_WriteRegister(sx127x_LORA_chip, sx127x_LORA_LORA_REG_4D_PA_DAC, sx127x_LORA_PA_DAC_DISABLE);
		sx127x_LORA_SetOCP(sx127x_LORA_chip, 100);

	}
	sx127x_LORA_WriteRegister(sx127x_LORA_chip, sx127x_LORA_LORA_REG_09_PA_CONFIG, sx127x_LORA_LORA_PA_SELECT | (power - 2));
}


void sx127x_LORA_SetOCP(sx127x_LORA_Chip *sx127x_LORA_chip, uint8_t mA) {
	uint8_t ocpTrim = 27;
	if (mA <= 120) {
		ocpTrim = (mA - 45) / 5;
	} else if (mA <=240) {
		ocpTrim = (mA + 30) / 10;
	}
	ocpTrim = ocpTrim & 0x1F;
	sx127x_LORA_WriteRegister(sx127x_LORA_chip, sx127x_LORA_LORA_REG_0B_OCP, sx127x_LORA_LORA_OCP_ON | ocpTrim);
}


void sx127x_LORA_BeginPacket(sx127x_LORA_Chip *sx127x_LORA_chip) {
	// Set the mode to standby
	sx127x_LORA_LoRaStandBy(sx127x_LORA_chip);

	// Set the header in explicit mode
	uint8_t current_config = sx127x_LORA_ReadRegister(sx127x_LORA_chip, sx127x_LORA_LORA_REG_1D_MODEM_CONFIG1);
	sx127x_LORA_WriteRegister(sx127x_LORA_chip, sx127x_LORA_LORA_REG_1D_MODEM_CONFIG1,
	                    current_config & ~sx127x_LORA_IMPLICIT_HEADER_MODE_ON);

	// Set the fifo pointer to 0x00
	sx127x_LORA_WriteRegister(sx127x_LORA_chip, sx127x_LORA_LORA_REG_0D_FIFO_ADDR_PTR, 0x00);

	// Set the payload length to 0
	sx127x_LORA_WriteRegister(sx127x_LORA_chip, sx127x_LORA_LORA_REG_22_PAYLOAD_LENGTH, 0x00);
}


void sx127x_LORA_EndPacket(sx127x_LORA_Chip *sx127x_LORA_chip) {
	uint8_t irq_flags;
	uint8_t mode;		// DEBUG : to remove
	uint8_t res;		// DEBUG : to remove

	// DEBUG : read current mode (has to be 0x81 = sx127x_LORA_LORA_LONG_RANGE_MODE | sx127x_LORA_LORA_MODE_STANDBY)
	mode = sx127x_LORA_ReadRegister(sx127x_LORA_chip, sx127x_LORA_LORA_REG_01_OP_MODE);

	// Set the mode to transmit
	sx127x_LORA_WriteRegister(sx127x_LORA_chip, sx127x_LORA_LORA_REG_01_OP_MODE, sx127x_LORA_LORA_LONG_RANGE_MODE | sx127x_LORA_LORA_MODE_TX);

	// DEBUG : read current mode (has to be 0x83 = sx127x_LORA_LORA_LONG_RANGE_MODE | sx127x_LORA_LORA_MODE_TX)
	mode = sx127x_LORA_ReadRegister(sx127x_LORA_chip, sx127x_LORA_LORA_REG_01_OP_MODE);

	// wait for the packet to be sent
	do {
		irq_flags = sx127x_LORA_ReadRegister(sx127x_LORA_chip, sx127x_LORA_LORA_REG_12_IRQ_FLAGS);
		res = irq_flags & sx127x_LORA_LORA_TX_DONE_MASK;
	} while (res == 0);

	// Utile ???
	// clear the tx done flag
	sx127x_LORA_WriteRegister(sx127x_LORA_chip, sx127x_LORA_LORA_REG_12_IRQ_FLAGS, sx127x_LORA_LORA_TX_DONE_MASK);	
}


void sx127x_LORA_Write(sx127x_LORA_Chip *sx127x_LORA_chip, uint8_t *data, uint16_t len) {
	uint8_t current_len, max_len;

	uint8_t irq_flags;
	uint8_t mode;		// DEBUG : to remove

	// DEBUG : read current mode (has to be 0x81 = sx127x_LORA_LORA_LONG_RANGE_MODE | sx127x_LORA_LORA_MODE_STANDBY)
	mode = sx127x_LORA_ReadRegister(sx127x_LORA_chip, sx127x_LORA_LORA_REG_01_OP_MODE);

	// Adjust the length of the packet
	current_len = sx127x_LORA_ReadRegister(sx127x_LORA_chip, sx127x_LORA_LORA_REG_22_PAYLOAD_LENGTH);
	max_len = sx127x_LORA_ReadRegister(sx127x_LORA_chip, sx127x_LORA_LORA_REG_23_MAX_PAYLOAD_LENGTH);
	if (current_len + len > max_len) {
		len = max_len - current_len;
	}

	// Write the data to the fifo
	sx127x_LORA_WriteRegisters(sx127x_LORA_chip, sx127x_LORA_REG_00_FIFO, data, len);
	current_len += len;

	// DEBUG : read the fifo content
	uint8_t fifo_content[256];
	sx127x_LORA_WriteRegister(sx127x_LORA_chip, sx127x_LORA_LORA_REG_0D_FIFO_ADDR_PTR, 0x00);
	sx127x_LORA_ReadRegisters(sx127x_LORA_chip, sx127x_LORA_REG_00_FIFO, fifo_content, current_len);

	// Update the length of the packet
	sx127x_LORA_WriteRegister(sx127x_LORA_chip, sx127x_LORA_LORA_REG_22_PAYLOAD_LENGTH, current_len);
}


void sx127x_LORA_WriteString(sx127x_LORA_Chip *sx127x_LORA_chip, char *data) {
	uint16_t len = strlen(data) + 1;
	char *data_cpy = (char *)malloc(len*sizeof(char));
	strcpy(data_cpy, data);
	data_cpy[len - 1] = '\0';

	sx127x_LORA_Write(sx127x_LORA_chip, (uint8_t *)data_cpy, len);

	free(data_cpy);
}


void sx127x_LORA_Print(sx127x_LORA_Chip *sx127x_LORA_chip, char *data) {
	HAL_Delay(1);
	sx127x_LORA_BeginPacket(sx127x_LORA_chip);
	sx127x_LORA_WriteString(sx127x_LORA_chip, data);
	sx127x_LORA_EndPacket(sx127x_LORA_chip);
}


int sx127x_LORA_ParsePacket(sx127x_LORA_Chip *sx127x_LORA_chip) {
  	int packetLength = 0;
  	uint8_t irqFlags = sx127x_LORA_ReadRegister(sx127x_LORA_chip, sx127x_LORA_LORA_REG_12_IRQ_FLAGS);

	// Set the header in explicit mode
	uint8_t current_config = sx127x_LORA_ReadRegister(sx127x_LORA_chip, sx127x_LORA_LORA_REG_1D_MODEM_CONFIG1);
	sx127x_LORA_WriteRegister(sx127x_LORA_chip, sx127x_LORA_LORA_REG_1D_MODEM_CONFIG1,
	                    current_config & ~sx127x_LORA_IMPLICIT_HEADER_MODE_ON);

	// Utile ???
	// clear the rx done flag
	sx127x_LORA_WriteRegister(sx127x_LORA_chip, sx127x_LORA_LORA_REG_12_IRQ_FLAGS, irqFlags);

	uint8_t current_mode = sx127x_LORA_ReadRegister(sx127x_LORA_chip, sx127x_LORA_LORA_REG_01_OP_MODE);
	if (current_config != (sx127x_LORA_LORA_LONG_RANGE_MODE | sx127x_LORA_LORA_MODE_RXSINGLE)) {
		// not currently in RX mode

		// reset FIFO address
		sx127x_LORA_WriteRegister(sx127x_LORA_chip, sx127x_LORA_LORA_REG_0D_FIFO_ADDR_PTR, 0);

		// put in single RX mode
		sx127x_LORA_WriteRegister(sx127x_LORA_chip, sx127x_LORA_LORA_REG_01_OP_MODE, sx127x_LORA_LORA_LONG_RANGE_MODE | sx127x_LORA_LORA_MODE_RXSINGLE);
	}
	if ((irqFlags & sx127x_LORA_LORA_RX_DONE_MASK) && (irqFlags & sx127x_LORA_LORA_PAYLOAD_CRC_ERROR_MASK) == 0) {

		// read packet length
		packetLength = sx127x_LORA_ReadRegister(sx127x_LORA_chip, sx127x_LORA_LORA_REG_13_RX_NB_BYTES);

		// set FIFO address to current RX address
		uint8_t rx_addr = sx127x_LORA_ReadRegister(sx127x_LORA_chip, sx127x_LORA_LORA_REG_0F_FIFO_RX_BASE_ADDR); 
		sx127x_LORA_WriteRegister(sx127x_LORA_chip, sx127x_LORA_LORA_REG_0D_FIFO_ADDR_PTR, rx_addr);

		// put in standby mode
		sx127x_LORA_LoRaStandBy(sx127x_LORA_chip);
	}

	return packetLength;
}


void sx127x_LORA_Read(sx127x_LORA_Chip *sx127x_LORA_chip, uint8_t *buff, int size) {
	int i;
	for (i = 0; i < size; i++) {
		buff[i] = sx127x_LORA_ReadRegister(sx127x_LORA_chip, sx127x_LORA_REG_00_FIFO);
	}
	buff[i] = '\0';
}


void sx127x_LORA_ReadRegisters(sx127x_LORA_Chip *sx127x_LORA_chip, uint8_t reg, uint8_t *data, uint16_t len) {
	uint8_t tx_buf[1] = {reg | sx127x_LORA_REG_READ_MASK};
	sx127x_LORA_TransmitReceive(sx127x_LORA_chip, tx_buf, data, 1, len);
}


uint8_t sx127x_LORA_ReadRegister(sx127x_LORA_Chip *sx127x_LORA_chip, uint8_t reg) {
	uint8_t data;
	sx127x_LORA_ReadRegisters(sx127x_LORA_chip, reg, &data, 1);
	return data;
}


void sx127x_LORA_WriteRegisters(sx127x_LORA_Chip *sx127x_LORA_chip, uint8_t reg, uint8_t *data, uint16_t len) {
	uint8_t *tx_buf = (uint8_t *)malloc(len + 1);
	tx_buf[0] = reg | sx127x_LORA_REG_WRITE_MASK;
	for (int i = 0; i < len; i++) {
		tx_buf[i + 1] = data[i];
	}
	sx127x_LORA_TransmitReceive(sx127x_LORA_chip, tx_buf, NULL, len + 1, 0);
	free(tx_buf);
}


void sx127x_LORA_WriteRegister(sx127x_LORA_Chip *sx127x_LORA_chip, uint8_t reg, uint8_t data) {
	uint8_t tx_buf[2] = {reg | sx127x_LORA_REG_WRITE_MASK, data};
	sx127x_LORA_TransmitReceive(sx127x_LORA_chip, tx_buf, NULL, 2, 0);
}

// TODO : à changer en sx127x_LORA_SPI_TxRx
void sx127x_LORA_TransmitReceive(sx127x_LORA_Chip *sx127x_LORA_chip, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t tx_size, uint16_t rx_size) {
	HAL_StatusTypeDef statue = HAL_OK;

	HAL_GPIO_WritePin(sx127x_LORA_chip->csPinBank, sx127x_LORA_chip->csPin, GPIO_PIN_RESET);
	statue += HAL_SPI_Transmit(sx127x_LORA_chip->spiHandle, tx_buf, tx_size, HAL_MAX_DELAY);
	if (rx_size > 0 && rx_buf != NULL && statue == HAL_OK) {
		statue += HAL_SPI_Receive(sx127x_LORA_chip->spiHandle, rx_buf, rx_size, HAL_MAX_DELAY);
	}
	HAL_GPIO_WritePin(sx127x_LORA_chip->csPinBank, sx127x_LORA_chip->csPin, GPIO_PIN_SET);
}
