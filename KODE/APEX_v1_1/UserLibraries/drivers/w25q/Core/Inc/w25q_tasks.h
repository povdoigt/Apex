/**
 *******************************************************************************
 * @file    w25q_tasks.h
 * @brief   Task layer of the W25Q driver, ported to the static scheduler.
 *
 * @details
 * Replaces the `/ * Niveau 1/2 : ... TASK * /` section of w25q.h.
 *
 * Two things left these argument structs, and they account for most of the
 * removed lines:
 *
 *   - `W25Q_STATE *result` : a body now returns its status. The framework
 *     publishes it to the destination given at spawn time, before the slot is
 *     released, so the value is readable as soon as task_join() succeeds.
 *
 *   - `osEventFlagsId_t done_flags` : completion is signalled through the
 *     joiner's own thread flags. No event group to create, no StaticEventGroup_t
 *     to place, no lifetime to manage.
 *
 * What remains in each struct is business input only.
 *
 * @note A body may freely mutate `args`: the framework copied them into the
 *       slot at spawn time, so the caller's struct is untouched and two
 *       concurrent instances never share them. W25Q_WriteData_task relies on
 *       this to walk its buffer.
 *******************************************************************************
 */

#ifndef W25Q_TASKS_H
#define W25Q_TASKS_H

#include "scheduler.h"
#include "w25q.h"

#if (APEX_CFG_SCHED_RTOS == 1)

/* ========================================================================== */
/*                        Level 1: command primitives                         */
/* ========================================================================== */

/** @brief Arguments of the W25Q_SendCmd task. */
typedef struct {
    W25Q_t  *chip;  /**< Target chip.      */
    uint8_t  cmd;   /**< Opcode to issue.  */
} W25Q_SendCmd_args_t;

/** @brief Issue a single opcode. Returns a W25Q_STATE. */
TASK_DECLARE(W25Q_SendCmd, W25Q_SendCmd_args_t, 512);

/** @brief Arguments of the W25Q_SendCmdAddr task. */
typedef struct {
    W25Q_t   *chip;  /**< Target chip.                 */
    uint8_t   cmd;   /**< Opcode to issue.             */
    uint32_t  addr;  /**< 32-bit address operand.      */
} W25Q_SendCmdAddr_args_t;

/**
 * @brief Issue an opcode with an address operand (block erase, and so on).
 *
 * The usual case for running several instances at once: erasing two blocks in
 * parallel is two spawns of this task on distinct join bits. The chip
 * semaphore serialises the bus accesses underneath.
 */
TASK_DECLARE(W25Q_SendCmdAddr, W25Q_SendCmdAddr_args_t, 512);

/* ========================================================================== */
/*                         Level 2: device logic                              */
/* ========================================================================== */

/** @brief Arguments of the W25Q_Init task. */
typedef struct {
    W25Q_t            *chip;     /**< Chip to bring up.            */
    SPI_HandleTypeDef *hspi;     /**< SPI bus it hangs on.         */
    GPIO_TypeDef      *cs_bank;  /**< Chip-select port.            */
    uint16_t           cs_pin;   /**< Chip-select pin.             */
} W25Q_Init_args_t;

/** @brief Bring up a chip: wire the bus, create its semaphore, read its ID. */
TASK_DECLARE(W25Q_Init, W25Q_Init_args_t, 512);

/** @brief Arguments of the W25Q_Write task. */
typedef struct {
    W25Q_t         *chip;  /**< Target chip.                        */
    const uint8_t  *buf;   /**< Source data.                        */
    uint32_t        addr;  /**< Destination address.                */
    uint32_t        len;   /**< Byte count; clamped to chip size.   */
} W25Q_Write_args_t;

/** @brief Page-program a buffer of arbitrary length across page boundaries. */
TASK_DECLARE(W25Q_Write, W25Q_Write_args_t, 768);

/** @brief Arguments of the W25Q_Read task. */
typedef struct {
    W25Q_t    *chip;  /**< Target chip.                        */
    uint8_t   *buf;   /**< Destination buffer.                 */
    uint32_t   addr;  /**< Source address.                     */
    uint32_t   len;   /**< Byte count; clamped to chip size.   */
} W25Q_Read_args_t;

/** @brief Read a contiguous span into a caller-owned buffer. */
TASK_DECLARE(W25Q_Read, W25Q_Read_args_t, 512);

/* ========================================================================== */
/*                              Convenience                                   */
/* ========================================================================== */

/**
 * @brief Recover a driver status from a task return value.
 *
 * task_ret_t is a plain int32_t, so a driver enum round-trips unchanged; this
 * only documents the intent at the call site.
 */
#define W25Q_RET(ret_)   ((W25Q_STATE)(ret_))

#endif /* APEX_CFG_SCHED_RTOS == 1 */
#endif /* W25Q_TASKS_H */
