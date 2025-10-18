/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.h
  * @brief   This file contains all the function prototypes for
  *          the i2c.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "utils/scheduler.h"
#include "utils/gms.h"

/* USER CODE END Includes */

extern I2C_HandleTypeDef hi2c3;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_I2C3_Init(void);

/* USER CODE BEGIN Prototypes */

#define ASYNC_I2C_Tx_or_Rx_DMA_NUMBER 10

// typedef enum ASYNC_WAITER_STATE {
// 	ASYNC_START,
// 	ASYNC_WAIT,
// 	ASYNC_END
// } ASYNC_WAITER_STATE;

typedef struct ASYNC_I2C_Tx_or_Rx_DMA_CONTEXT {
	I2C_HandleTypeDef *hi2c;

	bool has_started;

    uint8_t dev_address;
	uint8_t *buf;
	uint16_t size;

	uint32_t next_time;
} ASYNC_I2C_Tx_or_Rx_DMA_CONTEXT;

extern TASK_POOL_CREATE(ASYNC_I2C_Tx_or_Rx_DMA);

void ASYNC_I2C_Tx_or_Rx_DMA_init(TASK				*self,
							     I2C_HandleTypeDef	*hi2c,
                                 uint8_t             dev_address,
							     uint8_t			*buf,
							     uint16_t            size);

TASK_RETURN ASYNC_I2C_Tx_DMA(SCHEDULER* scheduler, TASK* self);
TASK_RETURN ASYNC_I2C_Tx_DMA(SCHEDULER* scheduler, TASK* self);



// =======================================================================

#define ASYNC_I2C_Mem_Tx_or_Rx_DMA_NUMBER 10


typedef struct ASYNC_I2C_Mem_Tx_or_Rx_DMA_CONTEXT {
	I2C_HandleTypeDef *hi2c;

	bool has_started;

    uint8_t dev_address;
    uint16_t mem_address;
    uint16_t mem_address_size; // I2C_MEMADD_SIZE_8BIT or I2C_MEMADD_SIZE_16BIT
	uint8_t *buf;
	uint16_t size;

	uint32_t next_time;
} ASYNC_I2C_Mem_Tx_or_Rx_DMA_CONTEXT;

extern TASK_POOL_CREATE(ASYNC_I2C_Mem_Tx_or_Rx_DMA);

void ASYNC_I2C_Mem_Tx_or_Rx_DMA_init(TASK				*self,
                                     I2C_HandleTypeDef	*hi2c,
                                     uint8_t             dev_address,
                                     uint16_t            mem_address,
                                     uint16_t            mem_address_size,
                                     uint8_t			*buf,
                                     uint16_t            size);

TASK_RETURN ASYNC_I2C_Mem_Tx_DMA(SCHEDULER* scheduler, TASK* self);
TASK_RETURN ASYNC_I2C_Mem_Rx_DMA(SCHEDULER* scheduler, TASK* self);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */

