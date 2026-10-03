/**
 *******************************************************************************
 * @file    example_flash_selftest.c
 * @brief   Worked example: the main usage principles of the static scheduler.
 *
 * @details
 * Ported from TASK_W25Q_ReadWriteTest, which needed six event groups, six
 * StaticEventGroup_t control blocks, six osEventFlagsNew() calls and six
 * osEventFlagsClear() calls before it could start working.
 *
 * It demonstrates, in order:
 *   1. the application-side RAM manifest (TASK_POOL),
 *   2. an ordinary sequential job: spawn, join, read the status,
 *   3. two instances of the same task running at once, joined together,
 *   4. a persistent task, spawned and never joined,
 *   5. prewarming a latency-critical pool at boot.
 *******************************************************************************
 */

#include <string.h>

#include "scheduler.h"
#include "w25q_tasks.h"

/* ========================================================================== */
/* 1. RAM manifest                                                            */
/*                                                                            */
/* Every task pool of the program, in one auditable block. The driver declared */
/* what each body needs; the application decides how many exist. A task not    */
/* listed here does not link, so nothing can silently reserve RAM.             */
/* ========================================================================== */

TASK_POOL(W25Q_Init,        1);
TASK_POOL(W25Q_Read,        1);
TASK_POOL(W25Q_Write,       1);
TASK_POOL(W25Q_SendCmd,     1);
TASK_POOL(W25Q_SendCmdAddr, 2);   /* two erases in flight at once */

/* A measured high-water mark would justify TASK_POOL_SZ instead:              */
/* TASK_POOL_SZ(W25Q_Write, 1, 1024);                                          */

/* ========================================================================== */
/* Join bits are claimed manually, and they are local to this thread           */
/* ========================================================================== */

/* Bit 0 belongs to the framework (garage). Bits 1..31 are ours to hand out,
   and only collide with bits used by *this* thread. */
#define JOIN_ID      1u
#define JOIN_ERASE_A 2u
#define JOIN_ERASE_B 3u
#define JOIN_IO      4u

#define TEST_ADDR    0x00000000u
#define TEST_LEN     64u

/* ========================================================================== */
/* 2. A sequential job                                                        */
/* ========================================================================== */

/**
 * @brief Read the chip ID, waiting for the result.
 *
 * The three lines below are the whole pattern: spawn, check the handle, join.
 * The status is in `st` as soon as the join succeeds, because the framework
 * publishes the return value before releasing the slot.
 */
static W25Q_STATE selftest_read_id(W25Q_t *chip) {
    task_ret_t st;

    task_h_t h = W25Q_SendCmd_spawn(
        &(W25Q_SendCmd_args_t){ .chip = chip, .cmd = W25Q_READ_JEDEC_ID },
        &(task_attr_t){ .priority = osPriorityNormal,
                        .ret      = &st,
                        .join_bit = JOIN_ID });

    if (!task_h_valid(h)) {
        return W25Q_CHIP_ERR;          /* pool exhausted: degrade, never wait */
    }

    if (task_join(h, 200u) != osOK) {
        /* Timeout does NOT cancel the job: it is still running, still holds
           the bus semaphore. Treat this as a fault, not as a retry point. */
        return W25Q_CHIP_ERR;
    }

    return W25Q_RET(st);
}

/* ========================================================================== */
/* 3. Two instances of one task, concurrently                                 */
/* ========================================================================== */

/**
 * @brief Erase two blocks in parallel.
 *
 * Both jobs run on the same task, on two instances of its pool, at the same
 * time. They are told apart only by their join bit; `task_join_all` waits for
 * both and rejects a duplicated bit at run time.
 *
 * The bus is not a race: the chip semaphore inside the driver serialises the
 * SPI accesses. What overlaps is the erase latency, which is where the time
 * actually goes.
 */
static W25Q_STATE selftest_erase_two(W25Q_t *chip, uint32_t addr_a, uint32_t addr_b) {
    task_ret_t st_a, st_b;

    task_h_t h[2];

    h[0] = W25Q_SendCmdAddr_spawn(
        &(W25Q_SendCmdAddr_args_t){ .chip = chip,
                                    .cmd  = W25Q_SECTOR_ERASE_4B,
                                    .addr = addr_a },
        &(task_attr_t){ .priority = osPriorityNormal,
                        .ret      = &st_a,
                        .join_bit = JOIN_ERASE_A });

    h[1] = W25Q_SendCmdAddr_spawn(
        &(W25Q_SendCmdAddr_args_t){ .chip = chip,
                                    .cmd  = W25Q_SECTOR_ERASE_4B,
                                    .addr = addr_b },
        &(task_attr_t){ .priority = osPriorityNormal,
                        .ret      = &st_b,
                        .join_bit = JOIN_ERASE_B });

    if (!task_h_valid(h[0]) || !task_h_valid(h[1])) {
        return W25Q_CHIP_ERR;
    }

    /* A block erase takes seconds on a W25Q: size the timeout accordingly. */
    if (task_join_all(h, 2u, 10000u) != osOK) {
        return W25Q_CHIP_ERR;
    }

    if (W25Q_RET(st_a) != W25Q_OK) { return W25Q_RET(st_a); }
    return W25Q_RET(st_b);
}

/* ========================================================================== */
/* Write-then-verify, reusing one join bit for successive jobs                */
/* ========================================================================== */

/**
 * @brief Write a pattern and read it back.
 *
 * JOIN_IO is reused for both jobs, which is safe because they are sequential:
 * the framework clears the bit at each spawn, so a flag left over by an
 * earlier job that nobody joined cannot satisfy this join.
 */
static W25Q_STATE selftest_write_verify(W25Q_t *chip) {
    uint8_t    pattern[TEST_LEN];
    uint8_t    readback[TEST_LEN];
    task_ret_t st;

    for (uint32_t i = 0; i < TEST_LEN; i++) {
        pattern[i] = (uint8_t)(i ^ 0xA5u);
    }

    task_h_t hw = W25Q_Write_spawn(
        &(W25Q_Write_args_t){ .chip = chip, .buf = pattern,
                              .addr = TEST_ADDR, .len = TEST_LEN },
        &(task_attr_t){ .priority = osPriorityNormal,
                        .ret = &st, .join_bit = JOIN_IO });

    if (!task_h_valid(hw) || task_join(hw, 1000u) != osOK) {
        return W25Q_CHIP_ERR;
    }
    if (W25Q_RET(st) != W25Q_OK) {
        return W25Q_RET(st);
    }

    /* `pattern` is still intact: W25Q_Write_task walked its own private copy
       of the arguments, not ours. */

    task_h_t hr = W25Q_Read_spawn(
        &(W25Q_Read_args_t){ .chip = chip, .buf = readback,
                             .addr = TEST_ADDR, .len = TEST_LEN },
        &(task_attr_t){ .priority = osPriorityNormal,
                        .ret = &st, .join_bit = JOIN_IO });

    if (!task_h_valid(hr) || task_join(hr, 1000u) != osOK) {
        return W25Q_CHIP_ERR;
    }
    if (W25Q_RET(st) != W25Q_OK) {
        return W25Q_RET(st);
    }

    return (memcmp(pattern, readback, TEST_LEN) == 0) ? W25Q_OK : W25Q_CHIP_ERR;
}

/* ========================================================================== */
/* 4. A persistent task                                                       */
/* ========================================================================== */

/** @brief Arguments of the health monitor. */
typedef struct {
    const task_desc_t *watched;  /**< Pool to keep an eye on. */
    uint32_t           period_ms;
} FlashMonitor_args_t;

/**
 * @brief Periodic supervision; never returns.
 *
 * Declared persistent, so the framework refuses a join bit on it: waiting for
 * a task that never finishes would hang the joiner, and that is now a spawn
 * error rather than a silent deadlock.
 */
TASK_DECLARE_PERSISTENT(FlashMonitor, FlashMonitor_args_t, 512);

TASK_DEFINE(FlashMonitor) {
    uint32_t next = osKernelGetTickCount();

    for (;;) {
        /* A pool permanently saturated is the symptom of an under-sized
           instance count, which shows up here rather than as a mystery
           TASK_H_INVALID in flight. */
        if (task_busy_count(args->watched) == args->watched->n_instances) {
            /* raise a housekeeping flag, bump a counter, blink a LED... */
        }

        next += args->period_ms;
        (void)osDelayUntil(next);
    }
    /* Unreachable; the framework would release the slot if it were reached. */
}

TASK_POOL(FlashMonitor, 1);

/* ========================================================================== */
/* 5. Putting it together                                                     */
/* ========================================================================== */

/**
 * @brief Full self-test, to be called from any thread.
 *
 * @note Called from a thread, after osKernelStart(): spawn and join both rely
 *       on the calling thread's own flags.
 */
W25Q_STATE flash_selftest(W25Q_t *chip, SPI_HandleTypeDef *hspi,
                          GPIO_TypeDef *cs_bank, uint16_t cs_pin) {
    task_ret_t st;
    W25Q_STATE w;

    /* Pay the first-spawn stack setup of the write path now rather than in
       flight. Optional, and worth it only for latency-critical pools. */
    (void)task_prewarm(&TASK_W25Q_Write);

    /* Bring the chip up. */
    task_h_t h = W25Q_Init_spawn(
        &(W25Q_Init_args_t){ .chip = chip, .hspi = hspi,
                             .cs_bank = cs_bank, .cs_pin = cs_pin },
        &(task_attr_t){ .priority = osPriorityNormal,
                        .ret = &st, .join_bit = JOIN_IO });

    if (!task_h_valid(h) || task_join(h, 500u) != osOK) {
        return W25Q_CHIP_ERR;
    }
    if (W25Q_RET(st) != W25Q_OK) {
        return W25Q_RET(st);
    }

    /* Start supervision. No join bit, so it is detached by construction. */
    (void)FlashMonitor_spawn(
        &(FlashMonitor_args_t){ .watched = &TASK_W25Q_SendCmdAddr,
                                .period_ms = 1000u },
        &(task_attr_t){ .priority = osPriorityLow });

    w = selftest_read_id(chip);
    if (w != W25Q_OK) { return w; }

    w = selftest_erase_two(chip, 0x00000000u, 0x00010000u);
    if (w != W25Q_OK) { return w; }

    return selftest_write_verify(chip);
}
