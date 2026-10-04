/**
 *******************************************************************************
 * @file    w25q_rtos.h
 * @brief   RTOS layer of the W25Q driver: semaphore-guarded primitives (DMA SPI)
 *          and task bodies for the static scheduler.
 *
 * @details
 * Everything that needs FreeRTOS or the scheduler API lives here. w25q.h / w25q.c
 * stay purely sequential (blocking HAL).
 *
 *   Level 0  SPI transaction (DMA, RTOS-aware)      -> private, w25q_rtos.c
 *   Level 1  command primitives, guarded by the chip semaphore
 *   Level 2  device logic (Init / PageProgram / Write / Read)
 *   Tasks    TASK_DECLARE'd wrappers around levels 1 and 2
 *
 * Each primitive exists in two flavours:
 *   - `W25Q_xxx_RTOS`         takes the chip semaphore for the whole operation;
 *   - `W25Q_xxx_RTOS_NoLock`  assumes the caller already owns the semaphore.
 *
 * Timeouts: `timeout_ms` bounds every wait of the operation separately -
 * acquiring the chip semaphore (W25Q_LOCK_TIMEOUT), then each wait for BUSY
 * to clear (W25Q_BUSY_TIMEOUT). 0 fails at once if the chip or the semaphore
 * is not free; osWaitForever waits without bound. The `_NoLock` flavours take
 * no semaphore, so only the BUSY waits apply to them.
 * It is a bound per wait, not per call: a primitive may take up to about
 * 2 x timeout_ms (semaphore, then BUSY), and W25Q_Write repeats that per page.
 * Bound the whole job with the timeout of task_join() instead.
 *
 * Waiting for BUSY: polled back to back for the first W25Q_RTOS_SPIN_MS, then
 * once per tick (osDelay(1)). A page program (~0.7 ms) completes while spinning;
 * an erase (tens of ms to 2 s) leaves the CPU to other tasks between polls.
 *
 * The chip semaphore is created once, by the first W25Q_Init, and never
 * re-created: the W25Q_t must therefore be zero-initialised before (static
 * storage), and that first Init must not run concurrently with any other use
 * of the chip. A later re-Init takes the semaphore like any other operation.
 *
 * Task arguments are copied into the slot at spawn time, so a body may mutate
 * `args` freely (W25Q_Write_task walks its buffer that way). The status a body
 * returns is published by the framework before the slot is released; recover it
 * with W25Q_RET().
 *
 * @note No task pool is defined here: the driver ships code, the application
 *       ships RAM (TASK_POOL block in the application source).
 *******************************************************************************
 */

#ifndef W25Q_RTOS_H
#define W25Q_RTOS_H

#include "w25q.h"

#if (APEX_CFG_SCHED_RTOS == 1)

#include "scheduler.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                              Configuration                                 */
/* ========================================================================== */

/**
 * @brief Busy-wait budget of WaitForReady, in ms, before it sleeps between polls.
 *
 * Measured with the 1 ms HAL tick, so the real spin lasts between
 * (W25Q_RTOS_SPIN_MS - 1) and W25Q_RTOS_SPIN_MS ms. 2 keeps a page program
 * (tPP ~0.7 ms typ.) entirely in the spin phase.
 */
#ifndef W25Q_RTOS_SPIN_MS
#define W25Q_RTOS_SPIN_MS   2u
#endif

/* ========================================================================== */
/*                        Level 1: command primitives                         */
/* ========================================================================== */

W25Q_STATE W25Q_WaitForReady_RTOS_base(W25Q_t *chip, uint32_t timeout_ms, bool lock_sem);
W25Q_STATE W25Q_SendCmd_RTOS_base(W25Q_t *chip, uint8_t cmd, uint32_t timeout_ms, bool lock_sem);
W25Q_STATE W25Q_SendCmdAddr_RTOS_base(W25Q_t *chip, uint8_t cmd, uint32_t addr, uint32_t timeout_ms, bool lock_sem);
W25Q_STATE W25Q_ReadStatus_RTOS_base(W25Q_t *chip, uint8_t sr_index, uint32_t timeout_ms, bool lock_sem);
W25Q_STATE W25Q_WriteStatus_RTOS_base(W25Q_t *chip, uint8_t sr_index, uint8_t value, W25Q_SR_WRITE mode, uint32_t timeout_ms, bool lock_sem);
W25Q_STATE W25Q_ReadID_RTOS_base(W25Q_t *chip, uint8_t *id, uint32_t timeout_ms, bool lock_sem);

#define W25Q_WaitForReady_RTOS_NoLock(chip, timeout_ms)                         W25Q_WaitForReady_RTOS_base(chip, timeout_ms, false)
#define W25Q_SendCmd_RTOS_NoLock(chip, cmd, timeout_ms)                         W25Q_SendCmd_RTOS_base(chip, cmd, timeout_ms, false)
#define W25Q_SendCmdAddr_RTOS_NoLock(chip, cmd, addr, timeout_ms)               W25Q_SendCmdAddr_RTOS_base(chip, cmd, addr, timeout_ms, false)
#define W25Q_ReadStatus_RTOS_NoLock(chip, sr_index)                             W25Q_ReadStatus_RTOS_base(chip, sr_index, 0u, false)
#define W25Q_WriteStatus_RTOS_NoLock(chip, sr_index, value, mode, timeout_ms)   W25Q_WriteStatus_RTOS_base(chip, sr_index, value, mode, timeout_ms, false)
#define W25Q_ReadID_RTOS_NoLock(chip, id)                                       W25Q_ReadID_RTOS_base(chip, id, 0u, false)

#define W25Q_WaitForReady_RTOS(chip, timeout_ms)                                W25Q_WaitForReady_RTOS_base(chip, timeout_ms, true)
#define W25Q_SendCmd_RTOS(chip, cmd, timeout_ms)                                W25Q_SendCmd_RTOS_base(chip, cmd, timeout_ms, true)
#define W25Q_SendCmdAddr_RTOS(chip, cmd, addr, timeout_ms)                      W25Q_SendCmdAddr_RTOS_base(chip, cmd, addr, timeout_ms, true)
#define W25Q_ReadStatus_RTOS(chip, sr_index, timeout_ms)                        W25Q_ReadStatus_RTOS_base(chip, sr_index, timeout_ms, true)
#define W25Q_WriteStatus_RTOS(chip, sr_index, value, mode, timeout_ms)          W25Q_WriteStatus_RTOS_base(chip, sr_index, value, mode, timeout_ms, true)
#define W25Q_ReadID_RTOS(chip, id, timeout_ms)                                  W25Q_ReadID_RTOS_base(chip, id, timeout_ms, true)

/* ========================================================================== */
/*                         Level 2: device logic                              */
/* ========================================================================== */

/** @brief Program at most one page; the caller must not cross a page boundary. */
W25Q_STATE W25Q_PageProgram_RTOS(W25Q_t *chip, const uint8_t *data, uint32_t addr, uint16_t len, uint32_t timeout_ms);

/* ========================================================================== */
/*                                   Tasks                                    */
/* ========================================================================== */

/** @brief Arguments of the W25Q_SendCmd task. */
typedef struct {
    W25Q_t  *chip;  /**< Target chip.      */
    uint8_t  cmd;   /**< Opcode to issue.  */
    uint32_t timeout_ms; /**< Max wait (ms) per wait (semaphore, BUSY); see header. */
} W25Q_SendCmd_args_t;

/** @brief Issue a single opcode. Returns a W25Q_STATE. */
TASK_DECLARE(W25Q_SendCmd, W25Q_SendCmd_args_t, 640);

/** @brief Arguments of the W25Q_SendCmdAddr task. */
typedef struct {
    W25Q_t   *chip;  /**< Target chip.             */
    uint8_t   cmd;   /**< Opcode to issue.         */
    uint32_t  addr;  /**< 32-bit address operand.  */
    uint32_t  timeout_ms; /**< Max wait (ms) per wait (semaphore, BUSY); see header. */
} W25Q_SendCmdAddr_args_t;

/**
 * @brief Issue an opcode with an address operand (sector/block erase...).
 *
 * Several instances can run at once (two erases = two spawns on distinct join
 * bits); the chip semaphore serialises the bus underneath.
 */
TASK_DECLARE(W25Q_SendCmdAddr, W25Q_SendCmdAddr_args_t, 640);

/** @brief Arguments of the W25Q_Init task. */
typedef struct {
    W25Q_t        *chip;    /**< Chip to bring up.                                  */
    W25Q_config_t  config;      /**< Bus + status-register configuration (see w25q.h). */
    uint32_t       timeout_ms;  /**< Max wait (ms) per wait (semaphore, BUSY); see header. */
} W25Q_Init_args_t;

/**
 * @brief Bring up a chip: create its semaphore (first call only), then, holding
 *        it, store the config, check the ID and apply the status-register config.
 *
 * Can be re-run on a live chip (config change, recovery after a reset): it then
 * waits for the semaphore like any other operation. Rejected parameters leave
 * the chip untouched.
 */
TASK_DECLARE(W25Q_Init, W25Q_Init_args_t, 640);

/** @brief Arguments of the W25Q_Write task. */
typedef struct {
    W25Q_t         *chip;  /**< Target chip.                       */
    const uint8_t  *buf;   /**< Source data.                       */
    uint32_t        addr;  /**< Destination address.               */
    uint32_t        len;   /**< Byte count; clamped to chip size.  */
    uint32_t        timeout_ms; /**< Max wait (ms) per wait (semaphore, BUSY); see header. */
} W25Q_Write_args_t;

/** @brief Page-program a buffer of arbitrary length across page boundaries. */
TASK_DECLARE(W25Q_Write, W25Q_Write_args_t, 768);

/** @brief Arguments of the W25Q_Read task. */
typedef struct {
    W25Q_t    *chip;  /**< Target chip.                       */
    uint8_t   *buf;   /**< Destination buffer.                */
    uint32_t   addr;  /**< Source address.                    */
    uint32_t   len;   /**< Byte count; clamped to chip size.  */
    uint32_t   timeout_ms; /**< Max wait (ms) per wait (semaphore, BUSY); see header. */
} W25Q_Read_args_t;

/** @brief Read a contiguous span into a caller-owned buffer. */
TASK_DECLARE(W25Q_Read, W25Q_Read_args_t, 640);

/** @brief Recover a driver status from a task return value (task_ret_t is int32_t). */
#define W25Q_RET(ret_)   ((W25Q_STATE)(ret_))

#ifdef __cplusplus
}
#endif

#endif /* APEX_CFG_SCHED_RTOS == 1 */
#endif /* W25Q_RTOS_H */
