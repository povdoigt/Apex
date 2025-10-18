/**
 *******************************************
 * @file    w25q_mem.c
 * @author  Dmitriy Semenov / Crazy_Geeks
 * @version 0.1b
 * @date    12-August-2021
 * @brief   Source file for W25Qxxx lib
 * @note    https://github.com/Crazy-Geeks/STM32-W25Q-QSPI
 *******************************************
 *
 * @note https://ru.mouser.com/datasheet/2/949/w25q256jv_spi_revg_08032017-1489574.pdf
 * @note https://www.st.com/resource/en/application_note/DM00227538-.pdf
 */

 /**
  * @addtogroup W25Q_Driver
  * @{
  */

#include "drivers/w25q_mem.h"
#include <stdlib.h>
#include <string.h>

#include "usbd_cdc_if.h"


void W25Q_Init(W25Q_Chip			*w25q_chip,
			   SPI_HandleTypeDef	*hspi,
			   GPIO_TypeDef			*csPinBank,
			   uint16_t              csPin,
			   uint32_t              id) {
	w25q_chip->hspi = hspi;
	w25q_chip->csPinBank = csPinBank;
	w25q_chip->csPin = csPin;
	w25q_chip->statue = W25Q_OK;

	w25q_chip->ASYNC_busy = false;

	for (uint8_t i = 0; i < 24; i++) {
		w25q_chip->status_bits[i] = false;
	}

	uint8_t id_buf[3];
	W25Q_ReadID(w25q_chip, id_buf);

	if (w25q_chip->statue == W25Q_OK) {
		if (id_buf[0] != W25Q_MANUFACTURER_ID) {
			w25q_chip->statue = W25Q_CHIP_ERR;
		} else if (id != (uint32_t)((id_buf[1] << 8) | id_buf[2])) {
			w25q_chip->statue = W25Q_PARAM_ERR;
		} else {
			W25Q_ReadStatusReg(w25q_chip);
    		W25Q_WriteStatuReg1(w25q_chip, 0x00);
		}
	}
}

bool W25Q_IsReady(W25Q_Chip *w25q_chip) {
	W25Q_ReadStatusReg(w25q_chip);
	return !w25q_chip->status_bits[W25Q_BUSY_BIT];
}

void W25Q_WaitForReady(W25Q_Chip *w25q_chip) {
	while (!W25Q_IsReady(w25q_chip)) {}
}

void W25Q_ReadID(W25Q_Chip *w25q_chip, uint8_t *id) {
	uint8_t txBuf[1] = { W25Q_READ_JEDEC_ID };
	uint8_t rxBuf[3];

	W25Q_TransmitReceive2(w25q_chip, txBuf, rxBuf, 1, 3);

	id[0] = rxBuf[0];
	id[1] = rxBuf[1];
	id[2] = rxBuf[2];
}

void W25Q_ReadStatusReg__(W25Q_Chip *w25q_chip) {

	uint8_t txBuf[3] = { W25Q_READ_SR1, W25Q_READ_SR2, W25Q_READ_SR3 };
	uint8_t rxBuf[3];

	W25Q_TransmitReceive2(w25q_chip, txBuf + 0, rxBuf + 0, 1, 1);
	W25Q_TransmitReceive2(w25q_chip, txBuf + 1, rxBuf + 1, 1, 1);
	W25Q_TransmitReceive2(w25q_chip, txBuf + 2, rxBuf + 2, 1, 1);

	for (uint8_t i = 0; i < 8; i++) {
		w25q_chip->status_bits[i + 0] = (rxBuf[0] >> i) & 0x01;
		w25q_chip->status_bits[i + 8] = (rxBuf[1] >> i) & 0x01;
		w25q_chip->status_bits[i + 16] = (rxBuf[2] >> i) & 0x01;
	}
}

void W25Q_ReadStatusReg(W25Q_Chip *w25q_chip) {

	uint8_t txBuf[3] = { W25Q_READ_SR1, W25Q_READ_SR2, W25Q_READ_SR3 };
	uint8_t rxBuf[6];

	W25Q_TransmitReceive(w25q_chip, txBuf + 0, rxBuf + 0, 1, 1);
	W25Q_TransmitReceive(w25q_chip, txBuf + 1, rxBuf + 2, 1, 1);
	W25Q_TransmitReceive(w25q_chip, txBuf + 2, rxBuf + 4, 1, 1);

	for (uint8_t i = 0; i < 8; i++) {
		w25q_chip->status_bits[i + 0] = (rxBuf[1] >> i) & 0x01;
		w25q_chip->status_bits[i + 8] = (rxBuf[3] >> i) & 0x01;
		w25q_chip->status_bits[i + 16] = (rxBuf[5] >> i) & 0x01;
	}
}

void W25Q_WriteStatuReg1(W25Q_Chip *w25q_chip, uint8_t data) {
	uint8_t txBuf[2] = { W25Q_WRITE_SR1, data };

	W25Q_TransmitReceive(w25q_chip, txBuf, NULL, 2, 0);	
}

void W25Q_WriteStatuReg2(W25Q_Chip *w25q_chip, uint8_t data) {
	uint8_t txBuf[2] = { W25Q_WRITE_SR2, data };

	W25Q_TransmitReceive(w25q_chip, txBuf, NULL, 2, 0);	
}

void W25Q_WriteStatuReg3(W25Q_Chip *w25q_chip, uint8_t data) {
	uint8_t txBuf[2] = { W25Q_WRITE_SR3, data };

	W25Q_TransmitReceive(w25q_chip, txBuf, NULL, 2, 0);
}

void W25Q_EarseSector(W25Q_Chip *w25q_chip, uint32_t addr) {
	W25Q_WaitForReady(w25q_chip);
	if (w25q_chip->status_bits[W25Q_WEL_BIT] == 0) {
		W25Q_WriteEnable(w25q_chip);
	}

	uint8_t txBuf[4] = { W25Q_SECTOR_ERASE,
						// (uint8_t)(addr >> 24),
						(uint8_t)(addr >> 16),
						(uint8_t)(addr >> 8 ),
						(uint8_t)(addr >> 0 ) };
	W25Q_TransmitReceive(w25q_chip, txBuf, NULL, 4, 0);
}

void W25Q_EarseAll(W25Q_Chip *w25q_chip) {
	W25Q_WaitForReady(w25q_chip);
	if (w25q_chip->status_bits[W25Q_WEL_BIT] == 0) {
		W25Q_WriteEnable(w25q_chip);
	}

	uint8_t txBuf[1] = { W25Q_CHIP_ERASE };
	W25Q_TransmitReceive(w25q_chip, txBuf, NULL, 1, 0);
}

void W25Q_WriteEnable(W25Q_Chip *w25q_chip) {
	uint8_t txBuf[1] = { W25Q_WRITE_ENABLE };
	W25Q_TransmitReceive(w25q_chip, txBuf, NULL, 1, 0);
}

void W25Q_WriteDisable(W25Q_Chip *w25q_chip) {
	uint8_t txBuf[1] = { W25Q_WRITE_DISABLE };
	W25Q_TransmitReceive(w25q_chip, txBuf, NULL, 1, 0);
}

void W25Q_PageProgram(W25Q_Chip *w25q_chip, uint8_t *data, uint32_t addr, uint16_t data_size) {
	W25Q_WaitForReady(w25q_chip);
	if (w25q_chip->status_bits[W25Q_WEL_BIT] == 0) {
		W25Q_WriteEnable(w25q_chip);
	}

	data_size = data_size > W25Q_MEM_PAGE_SIZE ? W25Q_MEM_PAGE_SIZE : data_size;
	uint16_t tx_size = data_size + 4;
	// uint8_t *tx_buf = malloc(sizeof(uint8_t) * tx_size);
	uint8_t tx_buf[256 + 4];
	tx_buf[0] = W25Q_PAGE_PROGRAM;		// Command
	// tx_buf[1] = (uint8_t)(addr >> 24);	// Address
	// tx_buf[2] = (uint8_t)(addr >> 16);	// Address
	// tx_buf[3] = (uint8_t)(addr >> 8);	// Address
	// tx_buf[4] = (uint8_t)(addr >> 0);	// Address
	tx_buf[1] = (uint8_t)(addr >> 16);	// Address
	tx_buf[2] = (uint8_t)(addr >> 8);	// Address
	tx_buf[3] = (uint8_t)(addr >> 0);	// Address

	// for (uint16_t i = 0; i < data_size; i++) {
	// 	tx_buf[i + 4] = data[i];
	// }
	memcpy(tx_buf + 4, data, data_size);

	W25Q_TransmitReceive(w25q_chip, tx_buf, NULL, tx_size, 0);

	// free(tx_buf);
}

void W25Q_WriteData(W25Q_Chip *w25q_chip, uint8_t *data, uint32_t addr, uint32_t data_size) {
	uint32_t flash_size = W25Q_MEM_FLASH_SIZE * 1000000; // MBytes to bytes
	uint32_t data_size_left = (data_size + addr) > flash_size ? flash_size - addr : data_size;
	uint32_t current_addr = addr;
	uint8_t *current_data = data;

	while (data_size_left > 0) {
		uint32_t relative_addr = current_addr % W25Q_MEM_PAGE_SIZE;
		uint16_t data_size_page = (data_size_left + relative_addr) > W25Q_MEM_PAGE_SIZE ? W25Q_MEM_PAGE_SIZE - relative_addr : data_size_left;
		W25Q_PageProgram(w25q_chip, current_data, current_addr, data_size_page);

		data_size_left -= data_size_page;
		current_addr += data_size_page;
		current_data += data_size_page;
	}
}

void W25Q_ReadData(W25Q_Chip *w25q_chip, uint8_t *data_buf, uint32_t addr, uint32_t data_size) {
	W25Q_WaitForReady(w25q_chip);

	for (uint16_t i = 0; i < data_size; i++) {
		data_buf[i] = 0xaa;
	}

	uint8_t tx_buf[4] = { W25Q_READ_DATA,			// Command
						//   (uint8_t)(addr >> 24),	// Address
						  (uint8_t)(addr >> 16),	// Address
						  (uint8_t)(addr >> 8 ),	// Address
						  (uint8_t)(addr >> 0 ) };	// Address

	W25Q_TransmitReceive2(w25q_chip, tx_buf, data_buf, 4, data_size);
}

void W25Q_TransmitReceive2(W25Q_Chip *w25q_chip, uint8_t *tx_buf, uint8_t* rx_buf, uint16_t tx_len, uint16_t rx_len) {
	uint16_t len = tx_len + rx_len;
	uint8_t *tx_buf_full = malloc(sizeof(uint8_t) * len);
	uint8_t *rx_buf_full = malloc(sizeof(uint8_t) * len);

	memcpy(tx_buf_full, tx_buf, tx_len);

	HAL_GPIO_WritePin(w25q_chip->csPinBank, w25q_chip->csPin, GPIO_PIN_RESET);
	if (rx_buf) {
		HAL_SPI_TransmitReceive(w25q_chip->hspi, tx_buf_full, rx_buf_full, len, HAL_MAX_DELAY);
	} else {
		HAL_SPI_Transmit(w25q_chip->hspi, tx_buf_full, tx_len, HAL_MAX_DELAY);
	}
	HAL_GPIO_WritePin(w25q_chip->csPinBank, w25q_chip->csPin, GPIO_PIN_SET);

	memcpy(rx_buf, rx_buf_full + tx_len, rx_len);

	free(tx_buf_full);
	free(rx_buf_full);
}

void W25Q_TransmitReceive(W25Q_Chip *w25q_chip, uint8_t *tx_buf, uint8_t* rx_buf, uint16_t tx_len, uint16_t rx_len) {
	uint16_t len = tx_len + rx_len;
	HAL_StatusTypeDef status;
	HAL_GPIO_WritePin(w25q_chip->csPinBank, w25q_chip->csPin, GPIO_PIN_RESET);
	if (rx_buf) {
		status = HAL_SPI_TransmitReceive(w25q_chip->hspi, tx_buf, rx_buf, len, HAL_MAX_DELAY);
	} else {
		status = HAL_SPI_Transmit(w25q_chip->hspi, tx_buf, tx_len, HAL_MAX_DELAY);
	}
	HAL_GPIO_WritePin(w25q_chip->csPinBank, w25q_chip->csPin, GPIO_PIN_SET);
}


TASK_POOL_CREATE(ASYNC_W25Q_ReadStatusReg);

void ASYNC_W25Q_ReadStatusReg_init(TASK *self, W25Q_Chip *w25q_chip) {
	ASYNC_W25Q_ReadStatusReg_CONTEXT *context = (ASYNC_W25Q_ReadStatusReg_CONTEXT*)self->context;

	context->w25q_chip = w25q_chip;

	context->state = ASYNC_W25Q_START;
	// can skip the waiting state if the chip is busy because this
	// task is not going to change the internal state of the chip

	memset(context->dma_complete, false, sizeof(bool) * 3);

	context->tx_buf[0] = W25Q_READ_SR1;
	context->tx_buf[1] = W25Q_READ_SR2;
	context->tx_buf[2] = W25Q_READ_SR3;
}

TASK_RETURN ASYNC_W25Q_ReadStatusReg(SCHEDULER *scheduler, TASK *self) {
	ASYNC_W25Q_ReadStatusReg_CONTEXT *context = (ASYNC_W25Q_ReadStatusReg_CONTEXT*)self->context;

	switch (context->state) {
	case ASYNC_W25Q_WAIT_W25Q: { break;} // what ever if w25q is taken by another task
	case ASYNC_W25Q_START: {	// if SR1, SR2 or SR3, then read
		TASK *task;
		for (uint8_t i = 0; i < 3; i++) {	
			task = SCHEDULER_add_task_macro(scheduler, ASYNC_SPI_TxRx_DMA, false);
			ASYNC_SPI_TxRx_DMA_init_W25Q(context->tx_buf + i, context->rx_buf + i, 1, 1);
			task->is_done = &(context->dma_complete[i]);
		}
		context->state = ASYNC_W25Q_WAIT;
		break;}
	case ASYNC_W25Q_WAIT: {
		if (context->dma_complete[0] &&
		    context->dma_complete[1] &&
			context->dma_complete[2]) {
			context->state = ASYNC_W25Q_END;
		}
		break;}
	case ASYNC_W25Q_END: {
		for (uint8_t i = 0; i < 8; i++) {
			context->w25q_chip->status_bits[i +  0] = (context->rx_buf[0] >> i) & 0x01; // SR1
			context->w25q_chip->status_bits[i +  8] = (context->rx_buf[1] >> i) & 0x01; // SR2
			context->w25q_chip->status_bits[i + 16] = (context->rx_buf[2] >> i) & 0x01; // SR3
		}
		return TASK_RETURN_STOP;
		break;}
	}
	return TASK_RETURN_IDLE;
}


TASK_POOL_CREATE(ASYNC_W25Q_WriteEnable);

void ASYNC_W25Q_WriteEnable_init(TASK *self, W25Q_Chip *w25q_chip) {
	ASYNC_W25Q_WriteEnable_CONTEXT *context = (ASYNC_W25Q_WriteEnable_CONTEXT*)self->context;
	context->w25q_chip = w25q_chip;
}

TASK_RETURN ASYNC_W25Q_WriteEnable(SCHEDULER *scheduler, TASK *self) {
	ASYNC_W25Q_WriteEnable_CONTEXT *context = (ASYNC_W25Q_WriteEnable_CONTEXT*)self->context;

	uint8_t tx_buf[1] = { W25Q_WRITE_ENABLE };
	TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_SPI_TxRx_DMA, false);
	ASYNC_SPI_TxRx_DMA_init_W25Q(tx_buf, NULL, 1, 0);
	task->is_done = self->is_done;
	self->is_done = NULL;

	return TASK_RETURN_STOP;
}


TASK_POOL_CREATE(ASYNC_W25Q_WaitForReady);

void ASYNC_W25Q_WaitForReady_init(TASK *self, W25Q_Chip *w25q_chip) {
	ASYNC_W25Q_WaitForReady_CONTEXT *context = (ASYNC_W25Q_WaitForReady_CONTEXT*)self->context;

	context->w25q_chip = w25q_chip;
	context->state = ASYNC_W25Q_START;
	// can skip the waiting state if the chip is busy because this
	// task is not going to change the internal state of the chip
}

TASK_RETURN ASYNC_W25Q_WaitForReady(SCHEDULER *scheduler, TASK *self) {
	ASYNC_W25Q_WaitForReady_CONTEXT *context = (ASYNC_W25Q_WaitForReady_CONTEXT*)self->context;

	switch (context->state) {
	case ASYNC_W25Q_WAIT_W25Q: { break; } // what ever if w25q is taken by another task
	case ASYNC_W25Q_START: {
		context->read_reg_status_done = false;
		TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_W25Q_ReadStatusReg, false);
		ASYNC_W25Q_ReadStatusReg_init(task, context->w25q_chip);
		task->is_done = &(context->read_reg_status_done);
		context->state = ASYNC_W25Q_WAIT;
		break;}
	case ASYNC_W25Q_WAIT: {
		if (context->read_reg_status_done) {
			if (context->w25q_chip->status_bits[W25Q_BUSY_BIT] == 0) {
				context->state = ASYNC_W25Q_END;
			} else {
				context->state = ASYNC_W25Q_START;
			}
		}
		break;}
	case ASYNC_W25Q_END: {
		return TASK_RETURN_STOP;
		break;}
	}
	return TASK_RETURN_IDLE;
}


TASK_POOL_CREATE(ASYNC_W25Q_EraseSector);

void ASYNC_W25Q_EraseSector_init(TASK *self, W25Q_Chip *w25q_chip, uint32_t addr) {
	ASYNC_W25Q_EraseSector_CONTEXT *context = (ASYNC_W25Q_EraseSector_CONTEXT*)self->context;

	context->w25q_chip = w25q_chip;
	context->addr = addr;

	context->tx_buf[0] = W25Q_SECTOR_ERASE;
	context->tx_buf[1] = (uint8_t)(context->addr >> 24);	// Address
	context->tx_buf[2] = (uint8_t)(context->addr >> 16);	// Address
	context->tx_buf[3] = (uint8_t)(context->addr >>  8);	// Address
	context->tx_buf[4] = (uint8_t)(context->addr >>  0);	// Address

	context->state = ASYNC_W25Q_WaitAndProceed_WAIT_W25Q;
}

TASK_RETURN ASYNC_W25Q_EraseSector(SCHEDULER *scheduler, TASK *self) {
	ASYNC_W25Q_EraseSector_CONTEXT *context = (ASYNC_W25Q_EraseSector_CONTEXT*)self->context;

	switch (context->state) {
	case ASYNC_W25Q_WaitAndProceed_WAIT_W25Q: {
		if (!(context->w25q_chip->ASYNC_busy)) {
			context->w25q_chip->ASYNC_busy = true;
			context->state = ASYNC_W25Q_WaitAndProceed_START;
		}
		break;}
	case ASYNC_W25Q_WaitAndProceed_START: {
		context->is_ready = false;
		TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_W25Q_WaitForReady, false);
		ASYNC_W25Q_WaitForReady_init(task, context->w25q_chip);
		task->is_done = &(context->is_ready);
		context->state = ASYNC_W25Q_WaitAndProceed_WAIT_READY;
		break;}
	case ASYNC_W25Q_WaitAndProceed_WAIT_READY: {
		if (context->is_ready) {
			if (context->w25q_chip->status_bits[W25Q_WEL_BIT] == 0) {
				context->is_ready = false;
				TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_W25Q_WriteEnable, false);
				ASYNC_W25Q_WriteEnable_init(task, context->w25q_chip);
				task->is_done = &(context->is_ready);
				context->state = ASYNC_W25Q_WaitAndProceed_WAIT_WEL;
			} else {
				context->state = ASYNC_W25Q_WaitAndProceed_TxRx;
			}
		}
		break;}
	case ASYNC_W25Q_WaitAndProceed_WAIT_WEL: {
		if (context->is_ready) {
			context->state = ASYNC_W25Q_WaitAndProceed_TxRx;
		}
		break;}
	case ASYNC_W25Q_WaitAndProceed_TxRx: {
		context->is_ready = false;
		TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_SPI_TxRx_DMA, false);
		ASYNC_SPI_TxRx_DMA_init_W25Q(context->tx_buf, NULL, 5, 0);
		task->is_done = &(context->is_ready);
		context->state = ASYNC_W25Q_WaitAndProceed_END;
		break;}
	case ASYNC_W25Q_WaitAndProceed_END: {
		if (context->is_ready) {
			context->w25q_chip->ASYNC_busy = false;
			return TASK_RETURN_STOP;
		}
		break;}
	}
	return TASK_RETURN_IDLE;
}


TASK_POOL_CREATE(ASYNC_W25Q_EraseAll);

void ASYNC_W25Q_EraseAll_init(TASK *self, W25Q_Chip *w25q_chip) {
	ASYNC_W25Q_EraseAll_CONTEXT *context = (ASYNC_W25Q_EraseAll_CONTEXT*)self->context;

	context->w25q_chip = w25q_chip;

	context->tx_buf[0] = W25Q_CHIP_ERASE;

	context->state = ASYNC_W25Q_WaitAndProceed_WAIT_W25Q;
}

TASK_RETURN ASYNC_W25Q_EraseAll(SCHEDULER *scheduler, TASK *self) {
	ASYNC_W25Q_EraseAll_CONTEXT *context = (ASYNC_W25Q_EraseAll_CONTEXT*)self->context;

	switch (context->state) {
	case ASYNC_W25Q_WaitAndProceed_WAIT_W25Q: {
		if (!(context->w25q_chip->ASYNC_busy)) {
			context->w25q_chip->ASYNC_busy = true;
			context->state = ASYNC_W25Q_WaitAndProceed_START;
		}
		break;}
	case ASYNC_W25Q_WaitAndProceed_START: {
		context->is_ready = false;
		TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_W25Q_WaitForReady, false);
		ASYNC_W25Q_WaitForReady_init(task, context->w25q_chip);
		task->is_done = &(context->is_ready);
		context->state = ASYNC_W25Q_WaitAndProceed_WAIT_READY;
		break;}
	case ASYNC_W25Q_WaitAndProceed_WAIT_READY: {
		if (context->is_ready) {
			if (context->w25q_chip->status_bits[W25Q_WEL_BIT] == 0) {
				context->is_ready = false;
				TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_W25Q_WriteEnable, false);
				ASYNC_W25Q_WriteEnable_init(task, context->w25q_chip);
				task->is_done = &(context->is_ready);
				context->state = ASYNC_W25Q_WaitAndProceed_WAIT_WEL;
			} else {
				context->state = ASYNC_W25Q_WaitAndProceed_TxRx;
			}
		}
		break;}
	case ASYNC_W25Q_WaitAndProceed_WAIT_WEL: {
		if (context->is_ready) {
			context->state = ASYNC_W25Q_WaitAndProceed_TxRx;
		}
		break;}
	case ASYNC_W25Q_WaitAndProceed_TxRx: {
		context->is_ready = false;
		TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_SPI_TxRx_DMA, false);
		ASYNC_SPI_TxRx_DMA_init_W25Q(context->tx_buf, NULL, 1, 0);
		task->is_done = &(context->is_ready);
		context->state = ASYNC_W25Q_WaitAndProceed_END;
		break;}
	case ASYNC_W25Q_WaitAndProceed_END: {
		if (context->is_ready) {
			context->w25q_chip->ASYNC_busy = false;
			return TASK_RETURN_STOP;
		}
		break;}
	}
	return TASK_RETURN_IDLE;
}


TASK_POOL_CREATE(ASYNC_W25Q_ReadData_Smol);

void ASYNC_W25Q_ReadData_Smol_init(TASK					*self,
								   W25Q_Chip			*w25q_chip,
								   uint8_t	            *data,
								   uint16_t 		     data_size,
								   uint32_t				 addr) {
	ASYNC_W25Q_ReadData_Smol_CONTEXT *context = (ASYNC_W25Q_ReadData_Smol_CONTEXT*)self->context;

	context->w25q_chip = w25q_chip;

	context->data = data;
	context->data_size = data_size;
	context->addr = addr;

	context->tx_buf[0] = W25Q_READ_DATA;
	// context->tx_buf[1] = (uint8_t)(context->addr >> 24);	// Address
	// context->tx_buf[2] = (uint8_t)(context->addr >> 16);	// Address
	// context->tx_buf[3] = (uint8_t)(context->addr >>  8);	// Address
	// context->tx_buf[4] = (uint8_t)(context->addr >>  0);	// Address

	context->tx_buf[1] = (uint8_t)(context->addr >> 16);	// Address
	context->tx_buf[2] = (uint8_t)(context->addr >>  8);	// Address
	context->tx_buf[3] = (uint8_t)(context->addr >>  0);	// Address

	context->state = ASYNC_W25Q_WaitAndProceed_WAIT_W25Q;
}

TASK_RETURN ASYNC_W25Q_ReadData_Smol(SCHEDULER *scheduler, TASK *self) {
	ASYNC_W25Q_ReadData_Smol_CONTEXT *context = (ASYNC_W25Q_ReadData_Smol_CONTEXT*)self->context;

	switch (context->state) {
	case ASYNC_W25Q_WAIT_W25Q: {
		if (!(context->w25q_chip->ASYNC_busy)) {
			context->w25q_chip->ASYNC_busy = true;
			context->state = ASYNC_W25Q_WaitAndProceed_START;
		}
		break;}
	case ASYNC_W25Q_WaitAndProceed_START: {
		context->is_ready = false;
		TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_W25Q_WaitForReady, false);
		ASYNC_W25Q_WaitForReady_init(task, context->w25q_chip);
		task->is_done = &(context->is_ready);
		context->state = ASYNC_W25Q_WaitAndProceed_WAIT_READY;
		break;}
	case ASYNC_W25Q_WaitAndProceed_WAIT_READY: {
		if (context->is_ready) {
			context->state = ASYNC_W25Q_WaitAndProceed_TxRx;
		}
		break;}
	case ASYNC_W25Q_WaitAndProceed_WAIT_WEL: {break;} // case never happens
	case ASYNC_W25Q_WaitAndProceed_TxRx: {
		context->is_ready = false;
		TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_SPI_TxRx_DMA, false);
		ASYNC_SPI_TxRx_DMA_init_W25Q(context->tx_buf, context->data, 4, context->data_size);
		task->is_done = &(context->is_ready);
		context->state = ASYNC_W25Q_WaitAndProceed_END;
		break;}
	case ASYNC_W25Q_WaitAndProceed_END: {
		if (context->is_ready) {
			context->w25q_chip->ASYNC_busy = false;
			return TASK_RETURN_STOP;
		}
		break;}
	}
	return TASK_RETURN_IDLE;
}


TASK_POOL_CREATE(ASYNC_W25Q_ReadData);

void ASYNC_W25Q_ReadData_init(TASK				*self,
							  W25Q_Chip			*w25q_chip,
							  uint8_t	 		*data,
							  uint16_t 			 data_size,
							  uint32_t			 addr) {
	ASYNC_W25Q_ReadData_CONTEXT *context = (ASYNC_W25Q_ReadData_CONTEXT*)self->context;

	context->w25q_chip = w25q_chip;
	context->data = data;
	context->data_size = data_size;
	context->based_addr = addr;

	context->last_data_size = data_size;

	context->state = ASYNC_W25Q_START;
}

TASK_RETURN ASYNC_W25Q_ReadData(SCHEDULER *scheduler, TASK *self) {
	ASYNC_W25Q_ReadData_CONTEXT *context = (ASYNC_W25Q_ReadData_CONTEXT*)self->context;

	switch (context->state) {
	case ASYNC_W25Q_WAIT_W25Q: { break; } // case never happens
	case ASYNC_W25Q_START: {
		context->is_ready = false;

		uint32_t addr = context->based_addr + (context->data_size - context->last_data_size);

		// Limit the data read size to 256 bytes in order to not saturate the SPI bus and the memory allocator
		size_t size_to_copy = W25Q_MEM_PAGE_SIZE * 16;
		size_to_copy = context->last_data_size > size_to_copy ? size_to_copy : context->last_data_size; // Limit to the last data size

		size_t offset = context->data_size - context->last_data_size;
		uint8_t *data_block = context->data + offset;
		context->last_data_size -= size_to_copy;

		TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_W25Q_ReadData_Smol, false);
		ASYNC_W25Q_ReadData_Smol_init(task, context->w25q_chip, data_block, size_to_copy, addr);
		task->is_done = &(context->is_ready);
		context->state = ASYNC_W25Q_WAIT;
		break;}
	case ASYNC_W25Q_WAIT: {
		if (context->is_ready) {
			if (context->last_data_size) {
				context->state = ASYNC_W25Q_START;
			} else {
				context->state = ASYNC_W25Q_END;
			}
		}
		break; }
	case ASYNC_W25Q_END: {
		return TASK_RETURN_STOP;
		break; }
	}
	return TASK_RETURN_IDLE;
}


TASK_POOL_CREATE(ASYNC_W25Q_PageProgram);

void ASYNC_W25Q_PageProgram_init(TASK				*self,
							     W25Q_Chip  		*w25q_chip,
								 uint8_t   			*data,
								 uint16_t 		 	 data_size,
							     uint32_t    		 addr) {
	ASYNC_W25Q_PageProgram_CONTEXT *context = (ASYNC_W25Q_PageProgram_CONTEXT*)self->context;
	
	context->w25q_chip = w25q_chip;

	size_t flash_size = W25Q_MEM_FLASH_SIZE * 1000000; // MBytes to bytes

	context->addr = addr;
	data_size = data_size > W25Q_MEM_PAGE_SIZE ? W25Q_MEM_PAGE_SIZE : data_size;
	data_size = flash_size < addr ? 0 : data_size;
	data_size = data_size + addr > flash_size ? flash_size - addr : data_size;
	context->data_size = data_size;

	uint8_t tx_buf_start[5] = { W25Q_PAGE_PROGRAM,
								// (uint8_t)(context->addr >> 24),		// Address
								(uint8_t)(context->addr >> 16),		// Address
								(uint8_t)(context->addr >>  8),		// Address
								(uint8_t)(context->addr >>  0) };	// Address

	context->tx_buf = GMS_alloc(&GMS_memory, data_size + 4);
	memcpy(context->tx_buf, tx_buf_start, 4);
	memcpy(context->tx_buf + 4, data, data_size);

	context->state = ASYNC_W25Q_WaitAndProceed_WAIT_W25Q;
}

TASK_RETURN ASYNC_W25Q_PageProgram(SCHEDULER *scheduler, TASK *self) {
	ASYNC_W25Q_PageProgram_CONTEXT *context = (ASYNC_W25Q_PageProgram_CONTEXT*)self->context;	

	switch (context->state) {
	case ASYNC_W25Q_WaitAndProceed_WAIT_W25Q: {
		if (context->data_size == 0) { // No data
			GMS_free(&GMS_memory, context->tx_buf);
			return TASK_RETURN_STOP;
		}
		if (!context->w25q_chip->ASYNC_busy) {
			context->state = ASYNC_W25Q_WaitAndProceed_START;
			context->w25q_chip->ASYNC_busy = true;
		}
		break; }
	case ASYNC_W25Q_WaitAndProceed_START: {
		context->is_ready = false;
		TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_W25Q_WaitForReady, false);
		ASYNC_W25Q_WaitForReady_init(task, context->w25q_chip);
		task->is_done = &(context->is_ready);
		context->state = ASYNC_W25Q_WaitAndProceed_WAIT_READY;
		break;}
	case ASYNC_W25Q_WaitAndProceed_WAIT_READY: {
		if (context->is_ready) {
			if (context->w25q_chip->status_bits[W25Q_WEL_BIT] == 0) {
				context->is_ready = false;
				TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_W25Q_WriteEnable, false);
				ASYNC_W25Q_WriteEnable_init(task, context->w25q_chip);
				task->is_done = &(context->is_ready);
				context->state = ASYNC_W25Q_WaitAndProceed_WAIT_WEL;
			} else {
				context->state = ASYNC_W25Q_WaitAndProceed_TxRx;
			}
		}
		break;}
	case ASYNC_W25Q_WaitAndProceed_WAIT_WEL: {
		if (context->is_ready) {
			context->state = ASYNC_W25Q_WaitAndProceed_TxRx;
		}
		break;}
	case ASYNC_W25Q_WaitAndProceed_TxRx: {
		context->is_ready = false;
		TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_SPI_TxRx_DMA, false);
		ASYNC_SPI_TxRx_DMA_init_W25Q(context->tx_buf, NULL, context->data_size + 4, 0);
		task->is_done = &(context->is_ready);
		context->state = ASYNC_W25Q_WaitAndProceed_END;
		break;}
	case ASYNC_W25Q_WaitAndProceed_END: {
		if (context->is_ready) {
			context->w25q_chip->ASYNC_busy = false;
			GMS_free(&GMS_memory, context->tx_buf);
			return TASK_RETURN_STOP;
		}
		break;}
	}
	return TASK_RETURN_IDLE;
}


TASK_POOL_CREATE(ASYNC_W25Q_WriteData);

void ASYNC_W25Q_WriteData_init(TASK					*self,
							   W25Q_Chip 			*w25q_chip,
							   uint8_t	 			*data,
							   size_t	 		 	 data_size,
							   uint32_t			 	 addr) {
	ASYNC_W25Q_WriteData_CONTEXT *context = (ASYNC_W25Q_WriteData_CONTEXT*)self->context;

	context->w25q_chip = w25q_chip;
	context->data = data;
	context->data_size = data_size;
	context->based_addr = addr;

	context->last_data_size = data_size;

	context->state = ASYNC_W25Q_START;
}

TASK_RETURN ASYNC_W25Q_WriteData(SCHEDULER *scheduler, TASK *self) {
	ASYNC_W25Q_WriteData_CONTEXT *context = (ASYNC_W25Q_WriteData_CONTEXT*)self->context;	

	switch (context->state) {
	case ASYNC_W25Q_WAIT_W25Q: { break; } // case never happens
	case ASYNC_W25Q_START: {
		context->is_ready = false;

		uint32_t addr = context->based_addr + (context->data_size - context->last_data_size);

		size_t offset = addr % W25Q_MEM_PAGE_SIZE;
		size_t size_to_copy = W25Q_MEM_PAGE_SIZE - offset;
		size_to_copy = context->last_data_size > size_to_copy ? size_to_copy : context->last_data_size;

		size_t offset_data = context->data_size - context->last_data_size;
		uint8_t *data_block = context->data + offset_data;
		context->last_data_size -= size_to_copy;

		TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_W25Q_PageProgram, false);
		ASYNC_W25Q_PageProgram_init(task, context->w25q_chip, data_block, size_to_copy, addr);
		task->is_done = &(context->is_ready);
		context->state = ASYNC_W25Q_WAIT;
		break; }
	case ASYNC_W25Q_WAIT: {
		if (context->is_ready) {
			if (context->last_data_size) {
				context->state = ASYNC_W25Q_START;
			} else {
				context->state = ASYNC_W25Q_END;
			}
		}
		break; }
	case ASYNC_W25Q_END: {
		return TASK_RETURN_STOP;
		break; }
	}
	return TASK_RETURN_IDLE;
}


TASK_POOL_CREATE(ASYNC_W25Q_ScanIfSectorErased);

void ASYNC_W25Q_ScanIfSectorErased_init(TASK *self, W25Q_Chip *w25q_chip, uint32_t addr, bool *is_erased) {
	ASYNC_W25Q_ScanIfSectorErased_CONTEXT *context = (ASYNC_W25Q_ScanIfSectorErased_CONTEXT*)self->context;

	context->w25q_chip = w25q_chip;
	context->addr = addr / 4096 * 4096; // align to sector start

	context->is_erased = is_erased;
	*context->is_erased = true;

	context->state = ASYNC_W25Q_START;
}

TASK_RETURN ASYNC_W25Q_ScanIfSectorErased(SCHEDULER *scheduler, TASK *self) {
	ASYNC_W25Q_ScanIfSectorErased_CONTEXT *context = (ASYNC_W25Q_ScanIfSectorErased_CONTEXT*)self->context;

	switch (context->state) {
	case ASYNC_W25Q_WAIT_W25Q: { break; } // case never happens
	case ASYNC_W25Q_START: {
		context->is_ready = false;
		TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_W25Q_ReadData, false);
		ASYNC_W25Q_ReadData_init(task, context->w25q_chip, context->data, 4096, context->addr);
		task->is_done = &(context->is_ready);
		context->state = ASYNC_W25Q_WAIT;
		break; }
	case ASYNC_W25Q_WAIT: {
		if (context->is_ready) {
			context->state = ASYNC_W25Q_END;
		}
		break; }
	case ASYNC_W25Q_END: {
		for (uint16_t i = 0; i < W25Q_MEM_PAGE_SIZE; i++) {
			if (context->data[i] != 0xFF) {
				*(context->is_erased) = false;
				break;
			}
		}
		return TASK_RETURN_STOP;
		break; }
	}
	return TASK_RETURN_IDLE;
}


TASK_POOL_CREATE(ASYNC_W25Q_ScanIfBlockErased);

void ASYNC_W25Q_ScanIfBlockErased_init(TASK *self, W25Q_Chip *w25q_chip, uint32_t addr, bool *sectors, bool *is_erased) {
	ASYNC_W25Q_ScanIfBlockErased_CONTEXT *context = (ASYNC_W25Q_ScanIfBlockErased_CONTEXT*)self->context;

	context->w25q_chip = w25q_chip;
	context->addr = addr / (4096 * 16) * (4096 * 16); // align to block start
	context->i = 0;

	context->sectors = sectors;
	memset(context->sectors, true, sizeof(bool) * 16); // 16 sectors in a block
	context->is_erased = is_erased;
	*context->is_erased = true;

	context->state = ASYNC_W25Q_START;
}

TASK_RETURN ASYNC_W25Q_ScanIfBlockErased(SCHEDULER *scheduler, TASK *self) {
	ASYNC_W25Q_ScanIfBlockErased_CONTEXT *context = (ASYNC_W25Q_ScanIfBlockErased_CONTEXT*)self->context;

	switch (context->state) {
	case ASYNC_W25Q_WAIT_W25Q: { break; } // case never happens
	case ASYNC_W25Q_START: {
		context->is_ready = false;
		TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_W25Q_ScanIfSectorErased, false);
		ASYNC_W25Q_ScanIfSectorErased_init(task, context->w25q_chip, context->addr, context->sectors + context->i);
		task->is_done = &(context->is_ready);
		context->state = ASYNC_W25Q_WAIT;
		break; }
	case ASYNC_W25Q_WAIT: {
		if (context->is_ready) {
			*(context->is_erased) &= context->sectors[context->i];
			if (context->i == 16) {
				context->state = ASYNC_W25Q_END;
			} else {
				context->i++;
				context->addr += 4096;
				context->state = ASYNC_W25Q_START;
			}
		}
		break; }
	case ASYNC_W25Q_END: {
		return TASK_RETURN_STOP;
		break; }
	}
	return TASK_RETURN_IDLE;
}


TASK_POOL_CREATE(ASYNC_W25Q_ScanIfChipErased);

void ASYNC_W25Q_ScanIfChipErased_init(TASK *self, W25Q_Chip *w25q_chip, bool *blocks, bool *is_erased) {
	ASYNC_W25Q_ScanIfChipErased_CONTEXT *context = (ASYNC_W25Q_ScanIfChipErased_CONTEXT*)self->context;

	context->w25q_chip = w25q_chip;

	context->is_erased = is_erased;
	*context->is_erased = true;

	context->blocks = blocks;
	memset(context->blocks, true, sizeof(bool) * 1024); // 1024 blocks
	context->addr = 0; // start from the beginning
	context->i = 0;

	context->state = ASYNC_W25Q_START;
}

TASK_RETURN ASYNC_W25Q_ScanIfChipErased(SCHEDULER *scheduler, TASK *self) {
	ASYNC_W25Q_ScanIfChipErased_CONTEXT *context = (ASYNC_W25Q_ScanIfChipErased_CONTEXT*)self->context;

	switch (context->state) {
	case ASYNC_W25Q_WAIT_W25Q: { break; } // case never happens
	case ASYNC_W25Q_START: {
		context->is_ready = false;
		TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_W25Q_ScanIfBlockErased, false);
		ASYNC_W25Q_ScanIfBlockErased_init(task, context->w25q_chip, context->addr, context->sectors, context->blocks + context->i);
		task->is_done = &(context->is_ready);
		context->state = ASYNC_W25Q_WAIT;
		break; }
	case ASYNC_W25Q_WAIT: {
		if (context->is_ready) {
			*(context->is_erased) &= context->blocks[context->i];
			// if (context->i == 1024) {
			if (context->i == 512) {
				context->state = ASYNC_W25Q_END;
			} else {
				context->i++;
				context->addr += 4096 * 16;
				context->state = ASYNC_W25Q_START;

				// char str[64];
				// sprintf(str, "Scanning block %d/%d\r\n", context->i, 1024);
				// CDC_Transmit_FS((uint8_t*)str, strlen(str));
			}
		}
		break; }
	case ASYNC_W25Q_END: {
		return TASK_RETURN_STOP;
		break; }
	}
	return TASK_RETURN_IDLE;
}


TASK_POOL_CREATE(ASYNC_W25Q_USB_ScanMemory);

void ASYNC_W25Q_USB_ScanMemory_init(TASK *self, W25Q_Chip *w25q_chip) {
	ASYNC_W25Q_USB_ScanMemory_CONTEXT *context = (ASYNC_W25Q_USB_ScanMemory_CONTEXT*)self->context;

	context->w25q_chip = w25q_chip;
	context->is_erased = false;

	context->addr = 0; // start from the beginning

	context->state = ASYNC_W25Q_START;
}

TASK_RETURN ASYNC_W25Q_USB_ScanMemory(SCHEDULER *scheduler, TASK *self) {
	ASYNC_W25Q_USB_ScanMemory_CONTEXT *context = (ASYNC_W25Q_USB_ScanMemory_CONTEXT*)self->context;

	switch (context->state) {
	case ASYNC_W25Q_WAIT_W25Q: { break; } // case never happens
	case ASYNC_W25Q_START: {
		context->is_ready = false;
		TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_W25Q_ReadData, false);
		ASYNC_W25Q_ReadData_init(task, context->w25q_chip, context->data, 4096, context->addr);
		task->is_done = &(context->is_ready);
		context->state = ASYNC_W25Q_WAIT;
		break; }
	case ASYNC_W25Q_WAIT: {
		if (context->is_ready) {
			context->state = ASYNC_W25Q_END;
		}
		break; }
	case ASYNC_W25Q_END: {
		HAL_Delay(10);
		CDC_Transmit_FS(context->data, 4096);
		context->addr += 4096;
		if (context->addr % 0x2000000 == 0) {
			HAL_Delay(1000); // set a delay to avoid flooding the USB
			__NOP();
		}
		// TODO : CHANGER A 64 MO
		if (context->addr >= 0x1000000) { // A CHANGER (OU PAS)
		// if (context->addr >= 0x2000) {
			HAL_Delay(1);
			char str[64];
			// sprintf(str, "End of memory scan\r\n");
			// CDC_Transmit_FS((uint8_t*)str, strlen(str));
			return TASK_RETURN_STOP; // end of memory
		} else {
			context->state = ASYNC_W25Q_START; // continue scanning
		}
		break; }
	}
	return TASK_RETURN_IDLE;
}


