#include "drivers/rfm96w.h"

#include <stdlib.h>
#include <string.h>


void RFM96_Init(RFM96_Chip        *rfm96_chip,
			    SPI_HandleTypeDef *spiHandle,
			    GPIO_TypeDef      *csPinBank,
			    uint16_t           csPin,
				GPIO_TypeDef      *resetPinBank,
				uint16_t           resetPin,
				double			   frequency) {
	rfm96_chip->spiHandle = spiHandle;
	rfm96_chip->csPinBank = csPinBank;
	rfm96_chip->csPin = csPin;
	rfm96_chip->resetPinBank = resetPinBank;
	rfm96_chip->resetPin = resetPin;

	// Reset the chip
	RFM96_Reset(rfm96_chip);

	// Check the version of the chip
	if (RFM96_GetVersion(rfm96_chip) != RFM96_VERSION) {
		// Error
		return;
	}

	// Set the mode to sleep
	RFM96_LoRaSleep(rfm96_chip);

	// Set the frequency
	RFM96_SetFrequency(rfm96_chip, frequency);

	// Set the fifo rx and tx base address to 0x00
	RFM96_WriteRegister(rfm96_chip, RFM96_REG_0F_FIFO_RX_BASE_ADDR, 0x00);
	RFM96_WriteRegister(rfm96_chip, RFM96_REG_0E_FIFO_TX_BASE_ADDR, 0x00);

	// Set lna boost
	uint8_t current_lna = RFM96_ReadRegister(rfm96_chip, RFM96_REG_0C_LNA);
	RFM96_WriteRegister(rfm96_chip, RFM96_REG_0C_LNA, current_lna | RFM96_LNA_BOOST_HF);


	// Set the bandwidth to 125 kHz
	RFM96_WriteRegister(rfm96_chip, RFM96_REG_1D_MODEM_CONFIG1, 0X72);

	// Set the spreading factor to 9
	// RFM96_WriteRegister(rfm96_chip, RFM96_REG_1E_MODEM_CONFIG2, RFM96_SPREADING_FACTOR_128CPS | RFM96_PAYLOAD_CRC_ON);
	RFM96_WriteRegister(rfm96_chip, RFM96_REG_1E_MODEM_CONFIG2, 0X74);

	// Set modem config 3 to auto AGC
	RFM96_WriteRegister(rfm96_chip, RFM96_REG_26_MODEM_CONFIG3, RFM96_AGC_AUTO_ON);

	// Set the output power to 10 dBm
	// ON PEUT MONTER A 13 VOIRE 25
	RFM96_SetTxPower(rfm96_chip, 13);


	// Set the mode to5 standby
	RFM96_LoRaStandBy(rfm96_chip);
}


void RFM96_Reset(RFM96_Chip *rfm96_chip) {
	HAL_GPIO_WritePin(rfm96_chip->resetPinBank, rfm96_chip->resetPin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(rfm96_chip->resetPinBank, rfm96_chip->resetPin, GPIO_PIN_SET);
	HAL_Delay(10);
}


uint8_t RFM96_GetVersion(RFM96_Chip *rfm96_chip) {
	uint8_t tx_buf[2] = {RFM96_REG_42_VERSION | RFM96_REG_READ_MASK, 0};
	uint8_t rx_buf[1];
	RFM96_TransmitReceive(rfm96_chip, tx_buf, rx_buf, 1, 1);
	return rx_buf[0];
}


void RFM96_LoRaSleep(RFM96_Chip *rfm96_chip) {
	uint8_t tx_buf[2] = {RFM96_REG_01_OP_MODE  | RFM96_REG_WRITE_MASK,
						 RFM96_LONG_RANGE_MODE | RFM96_MODE_SLEEP};
	RFM96_TransmitReceive(rfm96_chip, tx_buf, NULL, 2, 0);
}


void RFM96_LoRaStandBy(RFM96_Chip *rfm96_chip) {
	uint8_t tx_buf[2] = {RFM96_REG_01_OP_MODE  | RFM96_REG_WRITE_MASK,
						 RFM96_LONG_RANGE_MODE | RFM96_MODE_STDBY};
	RFM96_TransmitReceive(rfm96_chip, tx_buf, NULL, 2, 0);
}


void RFM96_SetFrequency(RFM96_Chip *rfm96_chip, double frequency) {
	uint64_t frf = ((uint64_t)frequency / RFM96_FSTEP);
	uint8_t tx_buf[4] = {RFM96_REG_06_FRF_MSB | RFM96_REG_WRITE_MASK,
						 (uint8_t)(frf >> 16),
						 (uint8_t)(frf >>  8),
						 (uint8_t)(frf >>  0)};
	RFM96_TransmitReceive(rfm96_chip, tx_buf, NULL, 4, 0);
}


double RFM96_GetFrequency(RFM96_Chip *rfm96_chip) {
	uint8_t rx_buf[3];
	RFM96_ReadRegisters(rfm96_chip, RFM96_REG_06_FRF_MSB, rx_buf, 3);
	uint64_t frf = ((uint64_t)rx_buf[0] << 16) |
	               ((uint64_t)rx_buf[1] <<  8) |
				   ((uint64_t)rx_buf[2] <<  0);
	return (double)(frf * RFM96_FSTEP);
}


void RFM96_SetTxPower(RFM96_Chip *rfm96_chip, int8_t power) {
	if (power > 17) {
		if (power > 20) {
			power = 20;
		}
		power -= 3;
		RFM96_WriteRegister(rfm96_chip, RFM96_REG_4D_PA_DAC, RFM96_PA_DAC_ENABLE);
		RFM96_SetOCP(rfm96_chip, 140);
	} else {
		if (power < 2) {
			power = 2;
		}
		RFM96_WriteRegister(rfm96_chip, RFM96_REG_4D_PA_DAC, RFM96_PA_DAC_DISABLE);
		RFM96_SetOCP(rfm96_chip, 100);

	}
	RFM96_WriteRegister(rfm96_chip, RFM96_REG_09_PA_CONFIG, RFM96_PA_SELECT | (power - 2));
}


void RFM96_SetOCP(RFM96_Chip *rfm96_chip, uint8_t mA) {
	uint8_t ocpTrim = 27;
	if (mA <= 120) {
		ocpTrim = (mA - 45) / 5;
	} else if (mA <=240) {
		ocpTrim = (mA + 30) / 10;
	}
	ocpTrim = ocpTrim & 0x1F;
	RFM96_WriteRegister(rfm96_chip, RFM96_REG_0B_OCP, RFM96_OCP_ON | ocpTrim);
}


void RFM96_BeginPacket(RFM96_Chip *rfm96_chip) {
	// Set the mode to standby
	RFM96_LoRaStandBy(rfm96_chip);

	// Set the header in explicit mode
	uint8_t current_config = RFM96_ReadRegister(rfm96_chip, RFM96_REG_1D_MODEM_CONFIG1);
	RFM96_WriteRegister(rfm96_chip, RFM96_REG_1D_MODEM_CONFIG1,
	                    current_config & ~RFM96_IMPLICIT_HEADER_MODE_ON);

	// Set the fifo pointer to 0x00
	RFM96_WriteRegister(rfm96_chip, RFM96_REG_0D_FIFO_ADDR_PTR, 0x00);

	// Set the payload length to 0
	RFM96_WriteRegister(rfm96_chip, RFM96_REG_22_PAYLOAD_LENGTH, 0x00);
}


void RFM96_EndPacket(RFM96_Chip *rfm96_chip) {
	uint8_t irq_flags;
	uint8_t mode;		// DEBUG : to remove
	uint8_t res;		// DEBUG : to remove

	// DEBUG : read current mode (has to be 0x81 = RFM96_LONG_RANGE_MODE | RFM96_MODE_STANDBY)
	mode = RFM96_ReadRegister(rfm96_chip, RFM96_REG_01_OP_MODE);

	// Set the mode to transmit
	RFM96_WriteRegister(rfm96_chip, RFM96_REG_01_OP_MODE, RFM96_LONG_RANGE_MODE | RFM96_MODE_TX);

	// DEBUG : read current mode (has to be 0x83 = RFM96_LONG_RANGE_MODE | RFM96_MODE_TX)
	mode = RFM96_ReadRegister(rfm96_chip, RFM96_REG_01_OP_MODE);

	// wait for the packet to be sent
	do {
		irq_flags = RFM96_ReadRegister(rfm96_chip, RFM96_REG_12_IRQ_FLAGS);
		res = irq_flags & RFM96_TX_DONE_MASK;
	} while (res == 0);

	// Utile ???
	// clear the tx done flag
	RFM96_WriteRegister(rfm96_chip, RFM96_REG_12_IRQ_FLAGS, RFM96_TX_DONE_MASK);	
}


void RFM96_Write(RFM96_Chip *rfm96_chip, uint8_t *data, uint16_t len) {
	uint8_t current_len, max_len;

	uint8_t irq_flags;
	uint8_t mode;		// DEBUG : to remove

	// DEBUG : read current mode (has to be 0x81 = RFM96_LONG_RANGE_MODE | RFM96_MODE_STANDBY)
	mode = RFM96_ReadRegister(rfm96_chip, RFM96_REG_01_OP_MODE);

	// Adjust the length of the packet
	current_len = RFM96_ReadRegister(rfm96_chip, RFM96_REG_22_PAYLOAD_LENGTH);
	max_len = RFM96_ReadRegister(rfm96_chip, RFM96_REG_23_MAX_PAYLOAD_LENGTH);
	if (current_len + len > max_len) {
		len = max_len - current_len;
	}

	// Write the data to the fifo
	RFM96_WriteRegisters(rfm96_chip, RFM96_REG_00_FIFO, data, len);
	current_len += len;

	// DEBUG : read the fifo content
	uint8_t fifo_content[256];
	RFM96_WriteRegister(rfm96_chip, RFM96_REG_0D_FIFO_ADDR_PTR, 0x00);
	RFM96_ReadRegisters(rfm96_chip, RFM96_REG_00_FIFO, fifo_content, current_len);

	// Update the length of the packet
	RFM96_WriteRegister(rfm96_chip, RFM96_REG_22_PAYLOAD_LENGTH, current_len);
}


void RFM96_WriteString(RFM96_Chip *rfm96_chip, char *data) {
	uint16_t len = strlen(data) + 1;
	char *data_cpy = (char *)malloc(len*sizeof(char));
	strcpy(data_cpy, data);
	data_cpy[len - 1] = '\0';

	RFM96_Write(rfm96_chip, (uint8_t *)data_cpy, len);

	free(data_cpy);
}


void RFM96_Print(RFM96_Chip *rfm96_chip, char *data) {
	HAL_Delay(1);
	RFM96_BeginPacket(rfm96_chip);
	RFM96_WriteString(rfm96_chip, data);
	RFM96_EndPacket(rfm96_chip);
}


int RFM96_ParsePacket(RFM96_Chip *rfm96_chip) {
  	int packetLength = 0;
  	uint8_t irqFlags = RFM96_ReadRegister(rfm96_chip, RFM96_REG_12_IRQ_FLAGS);

	// Set the header in explicit mode
	uint8_t current_config = RFM96_ReadRegister(rfm96_chip, RFM96_REG_1D_MODEM_CONFIG1);
	RFM96_WriteRegister(rfm96_chip, RFM96_REG_1D_MODEM_CONFIG1,
	                    current_config & ~RFM96_IMPLICIT_HEADER_MODE_ON);

	// Utile ???
	// clear the rx done flag
	RFM96_WriteRegister(rfm96_chip, RFM96_REG_12_IRQ_FLAGS, irqFlags);

	uint8_t current_mode = RFM96_ReadRegister(rfm96_chip, RFM96_REG_01_OP_MODE);
	if (current_config != (RFM96_LONG_RANGE_MODE | RFM96_MODE_RXSINGLE)) {
		// not currently in RX mode

		// reset FIFO address
		RFM96_WriteRegister(rfm96_chip, RFM96_REG_0D_FIFO_ADDR_PTR, 0);

		// put in single RX mode
		RFM96_WriteRegister(rfm96_chip, RFM96_REG_01_OP_MODE, RFM96_LONG_RANGE_MODE | RFM96_MODE_RXSINGLE);
	}
	if ((irqFlags & RFM96_RX_DONE_MASK) && (irqFlags & RFM96_PAYLOAD_CRC_ERROR_MASK) == 0) {

		// read packet length
		packetLength = RFM96_ReadRegister(rfm96_chip, RFM96_REG_13_RX_NB_BYTES);

		// set FIFO address to current RX address
		uint8_t rx_addr = RFM96_ReadRegister(rfm96_chip, RFM96_REG_0F_FIFO_RX_BASE_ADDR); 
		RFM96_WriteRegister(rfm96_chip, RFM96_REG_0D_FIFO_ADDR_PTR, rx_addr);

		// put in standby mode
		RFM96_LoRaStandBy(rfm96_chip);
	}

	return packetLength;
}


void RFM96_Read(RFM96_Chip *rfm96_chip, uint8_t *buff, int size) {
	int i;
	for (i = 0; i < size; i++) {
		buff[i] = RFM96_ReadRegister(rfm96_chip, RFM96_REG_00_FIFO);
	}
	buff[i] = '\0';
}


void RFM96_ReadRegisters(RFM96_Chip *rfm96_chip, uint8_t reg, uint8_t *data, uint16_t len) {
	uint8_t tx_buf[1] = {reg | RFM96_REG_READ_MASK};
	RFM96_TransmitReceive(rfm96_chip, tx_buf, data, 1, len);
}


uint8_t RFM96_ReadRegister(RFM96_Chip *rfm96_chip, uint8_t reg) {
	uint8_t data;
	RFM96_ReadRegisters(rfm96_chip, reg, &data, 1);
	return data;
}


void RFM96_WriteRegisters(RFM96_Chip *rfm96_chip, uint8_t reg, uint8_t *data, uint16_t len) {
	uint8_t *tx_buf = (uint8_t *)malloc(len + 1);
	tx_buf[0] = reg | RFM96_REG_WRITE_MASK;
	for (int i = 0; i < len; i++) {
		tx_buf[i + 1] = data[i];
	}
	RFM96_TransmitReceive(rfm96_chip, tx_buf, NULL, len + 1, 0);
	free(tx_buf);
}


void RFM96_WriteRegister(RFM96_Chip *rfm96_chip, uint8_t reg, uint8_t data) {
	uint8_t tx_buf[2] = {reg | RFM96_REG_WRITE_MASK, data};
	RFM96_TransmitReceive(rfm96_chip, tx_buf, NULL, 2, 0);
}

// TODO : à changer en RFM96_SPI_TxRx
void RFM96_TransmitReceive(RFM96_Chip *rfm96_chip, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t tx_size, uint16_t rx_size) {
	HAL_StatusTypeDef statue = HAL_OK;

	HAL_GPIO_WritePin(rfm96_chip->csPinBank, rfm96_chip->csPin, GPIO_PIN_RESET);
	statue += HAL_SPI_Transmit(rfm96_chip->spiHandle, tx_buf, tx_size, HAL_MAX_DELAY);
	if (rx_size > 0 && rx_buf != NULL && statue == HAL_OK) {
		statue += HAL_SPI_Receive(rfm96_chip->spiHandle, rx_buf, rx_size, HAL_MAX_DELAY);
	}
	HAL_GPIO_WritePin(rfm96_chip->csPinBank, rfm96_chip->csPin, GPIO_PIN_SET);
}


// TASK_POOL_CREATE(ASYNC_RFM96_BeginPacket);

// void ASYNC_RFM96_BeginPacket_init(TASK *self, RFM96_Chip *rfm96_chip) {
// 	ASYNC_RFM96_BeginPacket_CONTEXT *context = (ASYNC_RFM96_BeginPacket_CONTEXT *)self->context;
	
// 	context->rfm96_chip = rfm96_chip;
// 	context->state = ASYNC_RFM96_BeginPacket_LORA_STANDBY;
// }

// TASK_RETURN ASYNC_RFM96_BeginPacket(SCHEDULER *scheduler, TASK *self) {
// 	ASYNC_RFM96_BeginPacket_CONTEXT *context = (ASYNC_RFM96_BeginPacket_CONTEXT *)self->context;

// 	switch (context->state) {
// 		case ASYNC_RFM96_BeginPacket_WAIT_READY: {
// 			if (context->is_done) {
// 				context->state = context->next_state;
// 			}
// 			break; }
// 		case ASYNC_RFM96_BeginPacket_LORA_STANDBY: {
// 			context->is_done = false;
// 			context->tx_buf[0] = RFM96_REG_01_OP_MODE | RFM96_REG_WRITE_MASK;
// 			context->tx_buf[1] = RFM96_LONG_RANGE_MODE | RFM96_MODE_STDBY;

// 			TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_SPI_TxRx_DMA_static, false);
// 			ASYNC_SPI_TxRx_DMA_static_init_RFM96(context->tx_buf, NULL, 2, 0);
// 			task->is_done = &(context->is_done);

// 			context->next_state = ASYNC_RFM96_BeginPacket_GET_CONFIG;
// 			context->state = ASYNC_RFM96_BeginPacket_WAIT_READY;
// 			break; }
// 		case ASYNC_RFM96_BeginPacket_GET_CONFIG: {
// 			context->is_done = false;
// 			context->tx_buf[0] = RFM96_REG_1D_MODEM_CONFIG1 | RFM96_REG_READ_MASK;

// 			TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_SPI_TxRx_DMA_static, false);
// 			ASYNC_SPI_TxRx_DMA_static_init_RFM96(context->tx_buf, &(context->rx_buf), 1, 1);
// 			task->is_done = &(context->is_done);

// 			context->next_state = ASYNC_RFM96_BeginPacket_SET_CONFIG;
// 			context->state = ASYNC_RFM96_BeginPacket_WAIT_READY;
// 			break; }
// 		case ASYNC_RFM96_BeginPacket_SET_CONFIG: {
// 			context->is_done = false;
// 			// Set the header in explicit mode
// 			context->tx_buf[0] = RFM96_REG_1D_MODEM_CONFIG1 | RFM96_REG_WRITE_MASK;
// 			context->tx_buf[1] = context->rx_buf & ~RFM96_IMPLICIT_HEADER_MODE_ON;

// 			TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_SPI_TxRx_DMA_static, false);
// 			ASYNC_SPI_TxRx_DMA_static_init_RFM96(context->tx_buf, NULL, 2, 0);
// 			task->is_done = &(context->is_done);

// 			context->next_state = ASYNC_RFM96_BeginPacket_SET_FIFO_ADDR_PTR;
// 			context->state = ASYNC_RFM96_BeginPacket_WAIT_READY;
// 			break; }
// 		case ASYNC_RFM96_BeginPacket_SET_FIFO_ADDR_PTR: {
// 			context->is_done = false;
// 			// Set the fifo pointer to 0x00
// 			context->tx_buf[0] = RFM96_REG_0D_FIFO_ADDR_PTR | RFM96_REG_WRITE_MASK;
// 			context->tx_buf[1] = 0x00;

// 			TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_SPI_TxRx_DMA_static, false);
// 			ASYNC_SPI_TxRx_DMA_static_init_RFM96(context->tx_buf, NULL, 2, 0);
// 			task->is_done = &(context->is_done);

// 			context->next_state = ASYNC_RFM96_BeginPacket_PAYLOAD_LENGTH;
// 			context->state = ASYNC_RFM96_BeginPacket_WAIT_READY;
// 			break; }
// 		case ASYNC_RFM96_BeginPacket_PAYLOAD_LENGTH: {
// 			context->is_done = false;
// 			// Set the payload length to 0
// 			context->tx_buf[0] = RFM96_REG_22_PAYLOAD_LENGTH | RFM96_REG_WRITE_MASK;
// 			context->tx_buf[1] = 0x00;

// 			TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_SPI_TxRx_DMA_static, false);
// 			ASYNC_SPI_TxRx_DMA_static_init_RFM96(context->tx_buf, NULL, 2, 0);
// 			task->is_done = &(context->is_done);

// 			context->next_state = ASYNC_RFM96_BeginPacket_END;
// 			context->state = ASYNC_RFM96_BeginPacket_WAIT_READY;
// 			break; }
// 		case ASYNC_RFM96_BeginPacket_END: {
// 			return TASK_RETURN_STOP;
// 			break; }
// 		}
// 	return TASK_RETURN_IDLE;
// }


// TASK_POOL_CREATE(ASYNC_RFM96_EndPacket);

// void ASYNC_RFM96_EndPacket_init(TASK *self, RFM96_Chip *rfm96_chip) {
// 	ASYNC_RFM96_EndPacket_CONTEXT *context = (ASYNC_RFM96_EndPacket_CONTEXT *)self->context;

// 	context->rfm96_chip = rfm96_chip;
// 	context->state = ASYNC_RFM96_EndPacket_SET_TX_MODE;
// }

// TASK_RETURN ASYNC_RFM96_EndPacket(SCHEDULER *scheduler, TASK *self) {
// 	ASYNC_RFM96_EndPacket_CONTEXT *context = (ASYNC_RFM96_EndPacket_CONTEXT *)self->context;

// 	switch (context->state) {
// 	case ASYNC_RFM96_EndPacket_WAIT_READY: {
// 		if (context->is_done) {
// 			context->state = context->next_state;
// 		}
// 		break; }
// 	case ASYNC_RFM96_EndPacket_SET_TX_MODE: {
// 		context->is_done = false;
// 		// Set the mode to transmit
// 		context->tx_buf[0] = RFM96_REG_01_OP_MODE | RFM96_REG_WRITE_MASK;
// 		context->tx_buf[1] = RFM96_LONG_RANGE_MODE | RFM96_MODE_TX;

// 		TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_SPI_TxRx_DMA_static, false);
// 		ASYNC_SPI_TxRx_DMA_static_init_RFM96(context->tx_buf, NULL, 2, 0);
// 		task->is_done = &(context->is_done);

// 		context->next_state = ASYNC_RFM96_EndPacket_ASK_TX_DONE;
// 		context->state = ASYNC_RFM96_EndPacket_WAIT_READY;
// 		break; }
// 	case ASYNC_RFM96_EndPacket_ASK_TX_DONE: {
// 		context->is_done = false;
// 		// Wait for the packet to be sent
// 		context->tx_buf[0] = RFM96_REG_12_IRQ_FLAGS | RFM96_REG_READ_MASK;

// 		TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_SPI_TxRx_DMA_static, false);
// 		ASYNC_SPI_TxRx_DMA_static_init_RFM96(context->tx_buf, &(context->rx_buf), 1, 1);
// 		task->is_done = &(context->is_done);

// 		context->next_state = ASYNC_RFM96_EndPacket_WAIT_TX_DONE;
// 		context->state = ASYNC_RFM96_EndPacket_WAIT_READY;
// 		break; }
// 	case ASYNC_RFM96_EndPacket_WAIT_TX_DONE: {
// 		if (context->rx_buf & RFM96_TX_DONE_MASK) {
// 			context->state = ASYNC_RFM96_EndPacket_CLEAR_IRQ_FLAGS;
// 		} else {
// 			context->state = ASYNC_RFM96_EndPacket_ASK_TX_DONE;
// 		}
// 		break; }
// 	case ASYNC_RFM96_EndPacket_CLEAR_IRQ_FLAGS: {
// 		// Clear the tx done flag
// 		context->is_done = false;
// 		context->tx_buf[0] = RFM96_REG_12_IRQ_FLAGS | RFM96_REG_WRITE_MASK;
// 		context->tx_buf[1] = RFM96_TX_DONE_MASK; // here, irq_flag is not cleared ??

// 		TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_SPI_TxRx_DMA_static, false);
// 		ASYNC_SPI_TxRx_DMA_static_init_RFM96(context->tx_buf, NULL, 2, 0);
// 		task->is_done = &(context->is_done);

// 		context->next_state = ASYNC_RFM96_EndPacket_END;
// 		context->state = ASYNC_RFM96_EndPacket_WAIT_READY;
// 		break; }
// 	case ASYNC_RFM96_EndPacket_END: {
// 		// End the task
// 		return TASK_RETURN_STOP;
// 		}
// 	}
// 	return TASK_RETURN_IDLE;
// }


// TASK_POOL_CREATE(ASYNC_RFM96_WriteFIFO);

// void ASYNC_RFM96_WriteFIFO_init(TASK *self, RFM96_Chip *rfm96_chip, uint8_t *tx_buf, uint16_t tx_size) {
// 	ASYNC_RFM96_WriteFIFO_CONTEXT *context = (ASYNC_RFM96_WriteFIFO_CONTEXT *)self->context;

// 	context->rfm96_chip = rfm96_chip;
// 	memcpy(context->tx_fifo_buf + 1, tx_buf, tx_size);
// 	context->tx_fifo_size = tx_size;

// 	context->is_done = false;

// 	context->current_len = 0;
// 	context->max_len = 0;

// 	context->state = ASYNC_RFM96_WriteFIFO_GET_CURRENT_LEN;
// }

// TASK_RETURN ASYNC_RFM96_WriteFIFO(SCHEDULER *scheduler, TASK *self) {
// 	ASYNC_RFM96_WriteFIFO_CONTEXT *context = (ASYNC_RFM96_WriteFIFO_CONTEXT *)self->context;

// 	switch (context->state) {
// 	case ASYNC_RFM96_WriteFIFO_WAIT_READY: {
// 		if (context->is_done) {
// 			context->state = context->next_state;
// 		}
// 		break; }
// 	case ASYNC_RFM96_WriteFIFO_GET_CURRENT_LEN: {
// 		context->is_done = false;
// 		// Get the current length of the FIFO
// 		context->tx_buf[0] = RFM96_REG_22_PAYLOAD_LENGTH | RFM96_REG_READ_MASK;

// 		TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_SPI_TxRx_DMA_static, false);
// 		ASYNC_SPI_TxRx_DMA_static_init_RFM96(context->tx_buf, &(context->current_len), 1, 1);
// 		task->is_done = &(context->is_done);

// 		context->next_state = ASYNC_RFM96_WriteFIFO_GET_MAX_LEN;
// 		context->state = ASYNC_RFM96_WriteFIFO_WAIT_READY;
// 		break; }
// 	case ASYNC_RFM96_WriteFIFO_GET_MAX_LEN: {
// 		context->is_done = false;
// 		// Get the max length of the FIFO
// 		context->tx_buf[0] = RFM96_REG_23_MAX_PAYLOAD_LENGTH | RFM96_REG_READ_MASK;

// 		TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_SPI_TxRx_DMA_static, false);
// 		ASYNC_SPI_TxRx_DMA_static_init_RFM96(context->tx_buf, &(context->max_len), 1, 1);
// 		task->is_done = &(context->is_done);

// 		context->next_state = ASYNC_RFM96_WriteFIFO_WRITE_FIFO;
// 		context->state = ASYNC_RFM96_WriteFIFO_WAIT_READY;
// 		break; }
// 	case ASYNC_RFM96_WriteFIFO_WRITE_FIFO: {
// 		uint8_t len = context->tx_fifo_size + context->current_len > context->max_len ? context->max_len - context->current_len : context->tx_fifo_size;
// 		context->current_len += len;

// 		if (len <= 0) {
// 			return TASK_RETURN_STOP; // No more space in FIFO
// 		}
// 		static counter = 0;
// 		counter += 1;
// 		context->is_done = false;

// 		// Write the data to the FIFO
// 		context->tx_fifo_buf[0] = RFM96_REG_00_FIFO | RFM96_REG_WRITE_MASK;
		
// 		TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_SPI_TxRx_DMA_static, false);
// 		ASYNC_SPI_TxRx_DMA_static_init_RFM96(context->tx_fifo_buf, NULL, len + 1, 0);
// 		task->is_done = &(context->is_done);

// 		context->next_state = ASYNC_RFM96_WriteFIFO_UPDATE_LEN;
// 		context->state = ASYNC_RFM96_WriteFIFO_WAIT_READY;
// 		break; }
// 	case ASYNC_RFM96_WriteFIFO_UPDATE_LEN: {
// 		context->is_done = false;
// 		// Update the length of the packet
// 		context->tx_buf[0] = RFM96_REG_22_PAYLOAD_LENGTH | RFM96_REG_WRITE_MASK;
// 		context->tx_buf[1] = context->current_len;

// 		TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_SPI_TxRx_DMA_static, false);
// 		ASYNC_SPI_TxRx_DMA_static_init_RFM96(context->tx_buf, NULL, 2, 0);
// 		task->is_done = &(context->is_done);

// 		context->next_state = ASYNC_RFM96_WriteFIFO_END;
// 		context->state = ASYNC_RFM96_WriteFIFO_WAIT_READY;
// 		break; }
// 	case ASYNC_RFM96_WriteFIFO_END: {
// 		// End the task
// 		return TASK_RETURN_STOP;
// 		}
// 	}
// 	return TASK_RETURN_IDLE;
// }


// TASK_POOL_CREATE(ASYNC_RFM96_SendPacket);

// void ASYNC_RFM96_SendPacket_init(TASK *self, RFM96_Chip *rfm96_chip, uint8_t *tx_buf, uint8_t tx_size) {
// 	ASYNC_RFM96_SendPacket_CONTEXT *context = (ASYNC_RFM96_SendPacket_CONTEXT *)self->context;
	
// 	context->rfm96_chip = rfm96_chip;
// 	context->tx_buf = tx_buf;
// 	context->tx_size = tx_size;

// 	context->is_done = false;

// 	context->state = ASYNC_RFM96_SendPacket_BEGIN_PACKET;
// }

// TASK_RETURN ASYNC_RFM96_SendPacket(SCHEDULER *scheduler, TASK *self) {
// 	ASYNC_RFM96_SendPacket_CONTEXT *context = (ASYNC_RFM96_SendPacket_CONTEXT *)self->context;

// 	switch (context->state) {
// 	case ASYNC_RFM96_SendPacket_WAIT_READY: {
// 		if (context->is_done) {
// 			context->state = context->next_state;
// 		}
// 		break; }
// 	case ASYNC_RFM96_SendPacket_BEGIN_PACKET: {
// 		context->is_done = false;
		
// 		TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_RFM96_BeginPacket, false);
// 		ASYNC_RFM96_BeginPacket_init(task, context->rfm96_chip);
// 		task->is_done = &(context->is_done);

// 		context->next_state = ASYNC_RFM96_SendPacket_WRITE_FIFO;
// 		context->state = ASYNC_RFM96_SendPacket_WAIT_READY;
// 		break; }
// 	case ASYNC_RFM96_SendPacket_WRITE_FIFO: {
// 		context->is_done = false;

// 		TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_RFM96_WriteFIFO, false);
// 		ASYNC_RFM96_WriteFIFO_init(task, context->rfm96_chip, context->tx_buf, context->tx_size);
// 		task->is_done = &(context->is_done);

// 		context->next_state = ASYNC_RFM96_SendPacket_END_PACKET;
// 		context->state = ASYNC_RFM96_SendPacket_WAIT_READY;
// 		break; }
// 	case ASYNC_RFM96_SendPacket_END_PACKET: {
// 		context->is_done = false;

// 		TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_RFM96_EndPacket, false);
// 		ASYNC_RFM96_EndPacket_init(task, context->rfm96_chip);
// 		task->is_done = &(context->is_done);

// 		context->next_state = ASYNC_RFM96_SendPacket_END;
// 		context->state = ASYNC_RFM96_SendPacket_WAIT_READY;
// 		break; }
// 	case ASYNC_RFM96_SendPacket_END: {
// 		// End the task
// 		uint32_t time = HAL_GetTick();
// 		uint32_t time0 = self->time0;
// 		uint32_t elapsed = time - time0;

// 		return TASK_RETURN_STOP;
// 		break; }
// 	}
// 	return TASK_RETURN_IDLE;
// }


// TASK_POOL_CREATE(ASYNC_RFM96_SetRxMode);

// void ASYNC_RFM96_SetRxMode_init(TASK *self, RFM96_Chip *rfm96_chip) {
// 	ASYNC_RFM96_SetRxMode_CONTEXT *context = (ASYNC_RFM96_SetRxMode_CONTEXT *)self->context;

// 	context->rfm96_chip = rfm96_chip;

// 	context->state = ASYNC_RFM96_SetRxMode_Wait;
// }

// TASK_RETURN ASYNC_RFM96_SetRxMode(SCHEDULER *scheduler, TASK *self) {
// 	ASYNC_RFM96_SetRxMode_CONTEXT *context = (ASYNC_RFM96_SetRxMode_CONTEXT *)self->context;

// 	switch (context->state) {
// 	case ASYNC_RFM96_SetRxMode_Wait: {
// 		if (context->is_done[0] && context->is_done[1] && context->is_done[2]) {
// 			context->state = context->next_state;
// 		}
// 		break; }
// 	case ASYNC_RFM96_SetRxMode_GetConfig1: {
// 		context->is_done[0] = false;
// 		context->is_done[1] = true;
// 		context->is_done[2] = true;

// 		context->tx0[0] = RFM96_REG_1D_MODEM_CONFIG1;

// 		TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_SPI_TxRx_DMA_static, false);
// 		ASYNC_SPI_TxRx_DMA_static_init_RFM96(context->tx0, &(context->rx), 1, 1);
// 		task->is_done = context->is_done;

// 		context->state = ASYNC_RFM96_SetRxMode_Wait;
// 		context->next_state = ASYNC_RFM96_SetRxMode_SetConfig1_Mode_FIFO;
// 		break; }
// 	case ASYNC_RFM96_SetRxMode_SetConfig1_Mode_FIFO: {
// 		context->is_done[0] = false;
// 		context->is_done[1] = false;
// 		context->is_done[2] = false;

// 		TASK* task;

// 		context->tx0[0] = RFM96_REG_1D_MODEM_CONFIG1 | RFM96_REG_WRITE_MASK;
// 		context->tx0[1] = context->rx & ~RFM96_IMPLICIT_HEADER_MODE_ON;
// 		task = SCHEDULER_add_task_macro(scheduler, ASYNC_SPI_TxRx_DMA_static, false);
// 		ASYNC_SPI_TxRx_DMA_static_init_RFM96(context->tx0, NULL, 2, 0);

// 		context->tx1[0] = RFM96_REG_01_OP_MODE | RFM96_REG_WRITE_MASK;
// 		context->tx1[1] = RFM96_LONG_RANGE_MODE | RFM96_MODE_RXSINGLE;
// 		task = SCHEDULER_add_task_macro(scheduler, ASYNC_SPI_TxRx_DMA_static, false);
// 		ASYNC_SPI_TxRx_DMA_static_init_RFM96(context->tx1, NULL, 2, 0);

// 		context->tx2[0] = RFM96_REG_0D_FIFO_ADDR_PTR | RFM96_REG_WRITE_MASK;
// 		context->tx2[1] = 0;
// 		task = SCHEDULER_add_task_macro(scheduler, ASYNC_SPI_TxRx_DMA_static, false);
// 		ASYNC_SPI_TxRx_DMA_static_init_RFM96(context->tx1, NULL, 2, 0);

// 		context->state = ASYNC_RFM96_SetRxMode_Wait;
// 		context->next_state = ASYNC_RFM96_SetRxMode_DONE;
// 		break; }
// 	case ASYNC_RFM96_SetRxMode_DONE: {
// 		return TASK_RETURN_STOP;
// 		break; }
// 	}
// 	return TASK_RETURN_IDLE;
// }



// TASK_POOL_CREATE(ASYNC_RFM96_ParsePacket);

// void ASYNC_RFM96_ParsePacket_init(TASK *self, RFM96_Chip *rfm96_chip, uint8_t *data, uint8_t *len) {
// 	ASYNC_RFM96_ParsePacket_CONTEXT *context = (ASYNC_RFM96_ParsePacket_CONTEXT *)self->context;

// 	context->rfm96_chip = rfm96_chip;
// 	context->data = data;
// 	context->len = len;

// 	context->state = ASYNC_RFM96_ParsePacket_GetIRQ;
// }

// TASK_RETURN ASYNC_RFM96_ParsePacket(SCHEDULER *scheduler, TASK *self) {
// 	ASYNC_RFM96_ParsePacket_CONTEXT *context = (ASYNC_RFM96_ParsePacket_CONTEXT *)self->context;

// 	switch (context->state) {
// 	case ASYNC_RFM96_ParsePacket_Wait: {
// 		if (context->is_done) {
// 			context->state = context->next_state;
// 		}
// 		break; }
// 	case ASYNC_RFM96_ParsePacket_GetIRQ: {
// 		context->is_done = false;

// 		context->tx[0] = RFM96_REG_12_IRQ_FLAGS;
// 		TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_SPI_TxRx_DMA_static, false);
// 		ASYNC_SPI_TxRx_DMA_static_init_RFM96(context->tx, &context->rx, 1, 1);
// 		task->is_done = &(context->is_done);

// 		context->state = ASYNC_RFM96_ParsePacket_Wait;
// 		context->next_state = ASYNC_RFM96_ParsePacket_ResetIRQ;
// 		break; }
// 	case ASYNC_RFM96_ParsePacket_ResetIRQ: {
// 		context->is_done = false;

// 		context->tx[0] = RFM96_REG_12_IRQ_FLAGS | RFM96_REG_WRITE_MASK;
// 		context->tx[1] = context->rx;
// 		TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_SPI_TxRx_DMA_static, false);
// 		ASYNC_SPI_TxRx_DMA_static_init_RFM96(context->tx, NULL, 2, 0);
// 		task->is_done = &(context->is_done);

// 		context->state = ASYNC_RFM96_ParsePacket_Wait;
// 		context->next_state = ASYNC_RFM96_ParsePacket_GetLen;
// 		break; }
// 	case ASYNC_RFM96_ParsePacket_GetLen: {
// 		context->is_done = false;

// 		context->tx[0] = RFM96_REG_13_RX_NB_BYTES;
// 		TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_SPI_TxRx_DMA_static, false);
// 		ASYNC_SPI_TxRx_DMA_static_init_RFM96(context->tx, context->len, 1, 1);
// 		task->is_done = &(context->is_done);

// 		context->state = ASYNC_RFM96_ParsePacket_Wait;
// 		context->next_state = ASYNC_RFM96_ParsePacket_IsPacketAvailable;
// 		break; }
// 	case ASYNC_RFM96_ParsePacket_IsPacketAvailable: {
// 		if (context->len) {
// 			context->i = 0;
// 			context->state = ASYNC_RFM96_ParsePacket_GetRxAddr;
// 		} else {
// 			context->state = ASYNC_RFM96_ParsePacket_Done;
// 		}
// 		break; }
// 	case ASYNC_RFM96_ParsePacket_GetRxAddr: {
// 		context->is_done = false;

// 		context->tx[0] = RFM96_REG_0F_FIFO_RX_BASE_ADDR;
// 		TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_SPI_TxRx_DMA_static, false);
// 		ASYNC_SPI_TxRx_DMA_static_init_RFM96(context->tx, context->rx, 1, 1);
// 		task->is_done = &(context->is_done);

// 		context->state = ASYNC_RFM96_ParsePacket_Wait;
// 		context->next_state = ASYNC_RFM96_ParsePacket_SetRxAddr;
// 		break; }
// 	case ASYNC_RFM96_ParsePacket_SetRxAddr: {
// 		context->is_done = false;

// 		context->tx[0] = RFM96_REG_0F_FIFO_RX_BASE_ADDR;
// 		context->tx[1] = context->rx;
// 		TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_SPI_TxRx_DMA_static, false);
// 		ASYNC_SPI_TxRx_DMA_static_init_RFM96(context->tx, NULL, 2, 0);

// 		context->state = ASYNC_RFM96_ParsePacket_Wait;
// 		context->next_state = ASYNC_RFM96_ParsePacket_GetData;
// 		break; }
// 	case ASYNC_RFM96_ParsePacket_GetData: {
// 		context->is_done = false;

// 		context->tx[0] = RFM96_REG_0F_FIFO_RX_BASE_ADDR;
// 		TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_SPI_TxRx_DMA_static, false);
// 		ASYNC_SPI_TxRx_DMA_static_init_RFM96(context->tx, context->data + context->i, 1, 1);

// 		context->i++;
// 		if (context->i == *context->len) {
// 			context->state = ASYNC_RFM96_ParsePacket_Done;
// 		}

// 		break; }
// 	case ASYNC_RFM96_ParsePacket_Done: {
// 		return TASK_RETURN_STOP;
// 		break; }
// 	}
// 	return TASK_RETURN_IDLE;
// }
