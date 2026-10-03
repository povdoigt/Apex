/**
 *******************************************************************************
 * @file    w25q_tasks.c
 * @brief   Task bodies of the W25Q driver. @see w25q_tasks.h
 *
 * @details
 * Every body is business logic and a `return`. Compare with the previous
 * shape, repeated five times:
 *
 *     void TASK_W25Q_SendCmd(void *argument) {
 *         TASK_W25Q_SendCmd_ARGS *args = (TASK_W25Q_SendCmd_ARGS *)argument;
 *         *(args->result) = W25Q_SendCmd_RTOS(args->chip, args->cmd);
 *         if (args->done_flags) { osEventFlagsSet(args->done_flags, 1); }
 *         osThreadExit_Cstm();
 *     }
 *
 * The cast, the result write-through, the completion signal and the manual
 * exit are all gone. The manual exit mattered most: forgetting it on one error
 * path used to leak the slot for good, or run off the end of the thread.
 *
 * Note the one `goto` that survives, in W25Q_Read_task. It releases the chip
 * semaphore, which is a real resource this body owns. The framework removes
 * plumbing gotos, not ownership ones.
 *******************************************************************************
 */

#include "w25q_tasks.h"

#if (APEX_CFG_SCHED_RTOS == 1)

/* ========================================================================== */
/*                        Level 1: command primitives                         */
/* ========================================================================== */

/* No pool is defined here: the driver ships code, the application ships RAM.
   See the TASK_POOL block in the application source. */

TASK_DEFINE(W25Q_SendCmd) {
    return W25Q_SendCmd_RTOS(args->chip, args->cmd);
}

TASK_DEFINE(W25Q_SendCmdAddr) {
    return W25Q_SendCmdAddr_RTOS(args->chip, args->cmd, args->addr);
}

/* ========================================================================== */
/*                         Level 2: device logic                              */
/* ========================================================================== */

TASK_DEFINE(W25Q_Init) {
    args->chip->hspi    = args->hspi;
    args->chip->cs_bank = args->cs_bank;
    args->chip->cs_pin  = args->cs_pin;

    args->chip->sem_id = osSemaphoreNew(1u, 1u, &(const osSemaphoreAttr_t){
        .name    = "w25q_bus",
        .cb_mem  = &args->chip->sem,
        .cb_size = sizeof(args->chip->sem),
    });
    if (args->chip->sem_id == NULL) {
        return W25Q_SEM_ERR;
    }

    uint8_t id[3] = { 0 };
    W25Q_STATE st = W25Q_ReadID_RTOS(args->chip, id);
    if (st != W25Q_OK) {
        return st;
    }

    return (id[0] == W25Q_MANUFACTURER_ID) ? W25Q_OK : W25Q_CHIP_ERR;
}

TASK_DEFINE(W25Q_Write) {
    const uint32_t flash_size = W25Q_MEM_FLASH_SIZE * 1000000u / 8u;

    /* Clamping and walking mutate `args` freely: they are this slot's private
       copy, never the caller's struct and never shared with a sibling. */
    if (args->addr >= flash_size) {
        return W25Q_PARAM_ERR;
    }
    if ((args->len + args->addr) > flash_size) {
        args->len = flash_size - args->addr;
    }

    while (args->len > 0u) {
        uint32_t page_off = args->addr % W25Q_MEM_PAGE_SIZE;
        uint32_t room     = W25Q_MEM_PAGE_SIZE - page_off;
        uint16_t chunk    = (uint16_t)((args->len < room) ? args->len : room);

        W25Q_STATE st = W25Q_PageProgram_RTOS(args->chip, args->buf, args->addr, chunk);
        if (st != W25Q_OK) {
            return st;          /* was: *result = st; goto exit_flag; */
        }

        args->len  -= chunk;
        args->addr += chunk;
        args->buf  += chunk;
    }

    return W25Q_OK;
}

TASK_DEFINE(W25Q_Read) {
    const uint32_t flash_size = W25Q_MEM_FLASH_SIZE * 1000000u / 8u;
    W25Q_STATE     st;

    if (osSemaphoreAcquire(args->chip->sem_id, osWaitForever) != osOK) {
        return W25Q_SEM_ERR;
    }

    st = W25Q_WaitForReady_RTOS_NoLock(args->chip);
    if (st != W25Q_OK) {
        goto release;
    }

    if (args->len > flash_size) {
        args->len = flash_size;
    }

    const uint8_t cmd[5] = {
        W25Q_READ_DATA_4B,
        (uint8_t)(args->addr >> 24),
        (uint8_t)(args->addr >> 16),
        (uint8_t)(args->addr >>  8),
        (uint8_t)(args->addr >>  0),
    };

    st = W25Q_SPI_Begin_RTOS(args->chip);
    if (st != W25Q_OK) {
        goto release;
    }

    st = W25Q_SPI_Tx_RTOS(args->chip, cmd, sizeof(cmd));
    if (st == W25Q_OK) {
        st = W25Q_SPI_Rx_RTOS(args->chip, args->buf, args->len);
    }

    /* End the transaction whatever happened, but do not let a successful end
       mask a transfer error. */
    W25Q_STATE end_st = W25Q_SPI_End_RTOS(args->chip);
    if (st == W25Q_OK) {
        st = end_st;
    }

release:
    /* Ownership cleanup, not framework plumbing: this one stays. */
    if (osSemaphoreRelease(args->chip->sem_id) != osOK) {
        return W25Q_SEM_ERR;
    }
    return st;
}

#endif /* APEX_CFG_SCHED_RTOS == 1 */
