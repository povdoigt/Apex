/**
 *******************************************************************************
 * @file    BMI088_rtos.h
 * @brief   RTOS layer of the BMI088 driver: semaphore-guarded primitives (DMA
 *          SPI), sensor logic, and task bodies for the static scheduler.
 *
 * @details
 * Everything that needs FreeRTOS or the scheduler API lives here. BMI088.h /
 * BMI088.c stay purely sequential (blocking HAL).
 *
 *   Level 0  SPI transaction (DMA, shared bus)           -> private, BMI088_rtos.c
 *   Level 1  register access, guarded by the sensor semaphore
 *   Level 2  sensor logic (Init / ApplyConfig / ReadAcc / ReadGyr / ReadTemp)
 *   Tasks    BMI088_Init and BMI088_ApplyConfig (one-shot), BMI088_Acquire
 *            (periodic acquisition, publishes on data_topic)
 *
 * Each primitive exists in two flavours:
 *   - `BMI088_xxx_RTOS`         takes the sensor semaphore for the whole operation;
 *   - `BMI088_xxx_RTOS_NoLock`  assumes the caller already owns it.
 *
 * Locking: the SPI bus is shared (SPI1: ADXL375, SX127x) and is serialised by
 * the per-bus semaphore of spi.c, one transaction at a time. The sensor
 * semaphore sits above it and makes a multi-transaction sequence (init,
 * configuration, reset) atomic with respect to the other users of this
 * BMI088; it also guards the cached configuration and conversion factors.
 *
 * Timeouts: `timeout_ms` bounds the wait for the sensor semaphore
 * (BMI_LOCK_TIMEOUT). Waiting for the SPI bus itself is not bounded yet
 * (SPI_Begin_DMA_RTOS waits forever). Delays required by the datasheet
 * (resets, power modes) are slept with osDelay; the 2 us gap between two
 * transactions is a DWT busy-wait.
 *
 * The sensor semaphore is created once, by the first init, and never
 * re-created: the bmi088_t must be zero-initialised (static storage), and that
 * first init must not run concurrently with any other use of the sensor.
 *
 * Stacks: sized for a Debug (-O0) build. The deepest chain (Init -> Bringup
 * -> ApplyConfig -> Verify -> read -> SPI DMA -> semaphore wait) takes about
 * 660 B of frames, plus ~200 B of exception frame and FPU context saved on
 * the task stack. An overflow lands on the pool's TCB array, just below the
 * stacks, and corrupts kernel lists. Release builds may shrink them with
 * TASK_POOL_SZ, from task_stack_headroom() measurements.
 *
 * @note No task pool is defined here: the driver ships code, the application
 *       ships RAM (TASK_POOL block in the application source).
 *******************************************************************************
 */

#ifndef BMI088_RTOS_H
#define BMI088_RTOS_H

#include "BMI088.h"

#if (APEX_CFG_SCHED_RTOS == 1)

#include "scheduler.h"
#include "data_topic.h"
#include "tools.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                        Level 1: register access                            */
/* ========================================================================== */

BMI_STATE BMI088_ReadMultiple_RTOS_base(bmi088_t *imu, bool is_gyr, uint8_t reg, uint8_t *data, uint16_t len, uint32_t timeout_ms, bool lock_sem);
BMI_STATE BMI088_WriteRegister_RTOS_base(bmi088_t *imu, bool is_gyr, uint8_t reg, uint8_t value, uint32_t timeout_ms, bool lock_sem);
BMI_STATE BMI088_ReadID_RTOS_base(bmi088_t *imu, uint8_t *acc_id, uint8_t *gyr_id, uint32_t timeout_ms, bool lock_sem);
BMI_STATE BMI088_SoftReset_RTOS_base(bmi088_t *imu, bool is_gyr, uint32_t timeout_ms, bool lock_sem);

#define BMI088_ReadMultiple_RTOS_NoLock(imu, is_gyr, reg, data, len)        BMI088_ReadMultiple_RTOS_base(imu, is_gyr, reg, data, len, 0u, false)
#define BMI088_ReadRegister_RTOS_NoLock(imu, is_gyr, reg, value)            BMI088_ReadMultiple_RTOS_base(imu, is_gyr, reg, value, 1u, 0u, false)
#define BMI088_WriteRegister_RTOS_NoLock(imu, is_gyr, reg, value)           BMI088_WriteRegister_RTOS_base(imu, is_gyr, reg, value, 0u, false)
#define BMI088_ReadID_RTOS_NoLock(imu, acc_id, gyr_id)                      BMI088_ReadID_RTOS_base(imu, acc_id, gyr_id, 0u, false)
#define BMI088_SoftReset_RTOS_NoLock(imu, is_gyr)                           BMI088_SoftReset_RTOS_base(imu, is_gyr, 0u, false)

#define BMI088_ReadMultiple_RTOS(imu, is_gyr, reg, data, len, timeout_ms)   BMI088_ReadMultiple_RTOS_base(imu, is_gyr, reg, data, len, timeout_ms, true)
#define BMI088_ReadRegister_RTOS(imu, is_gyr, reg, value, timeout_ms)       BMI088_ReadMultiple_RTOS_base(imu, is_gyr, reg, value, 1u, timeout_ms, true)
#define BMI088_WriteRegister_RTOS(imu, is_gyr, reg, value, timeout_ms)      BMI088_WriteRegister_RTOS_base(imu, is_gyr, reg, value, timeout_ms, true)
#define BMI088_ReadID_RTOS(imu, acc_id, gyr_id, timeout_ms)                 BMI088_ReadID_RTOS_base(imu, acc_id, gyr_id, timeout_ms, true)
#define BMI088_SoftReset_RTOS(imu, is_gyr, timeout_ms)                      BMI088_SoftReset_RTOS_base(imu, is_gyr, timeout_ms, true)

/* ========================================================================== */
/*                         Level 2: sensor logic                              */
/* ========================================================================== */

/**
 * @brief Bring up the sensor: create its semaphore (first call only), then,
 *        holding it, store the bus, reset, check the IDs and apply `reg`.
 *
 * Thread context only. Can be re-run on a live sensor (recovery): it then
 * waits for the semaphore like any other operation. Rejected parameters leave
 * the handle untouched.
 *
 * @warning ACC data read as zeros for ~55 ms afterwards; an acquisition
 *          running meanwhile publishes them (see BMI088.h header).
 */
BMI_STATE BMI088_Init_RTOS(bmi088_t *imu, const bmi_config_t *config, uint32_t timeout_ms);

/**
 * @brief Write, read back and cache a register configuration (atomic).
 * @warning ACC data still reflect the previous setting for ~55 ms afterwards
 *          (old-range raw x new factor); an acquisition running meanwhile
 *          publishes them (see BMI088.h header).
 */
BMI_STATE BMI088_ApplyConfig_RTOS(bmi088_t *imu, const bmi_reg_config_t *reg, uint32_t timeout_ms);

/** @brief One transaction each; callable directly from an application task. */
BMI_STATE BMI088_ReadAcc_RTOS(bmi088_t *imu, float3_t *accel, uint32_t timeout_ms);
BMI_STATE BMI088_ReadGyr_RTOS(bmi088_t *imu, float3_t *gyro, uint32_t timeout_ms);
BMI_STATE BMI088_ReadTemp_RTOS(bmi088_t *imu, float *temp_c, uint32_t timeout_ms);

/* ========================================================================== */
/*                                   Tasks                                    */
/* ========================================================================== */

/** @brief Arguments of the BMI088_Init task. */
typedef struct {
    bmi088_t       *imu;         /**< Sensor to bring up (zero-initialised). */
    bmi_config_t    config;      /**< Bus + register configuration.          */
    uint32_t        timeout_ms;  /**< Max wait for the sensor semaphore.     */
} BMI088_Init_args_t;

/** @brief Run BMI088_Init_RTOS in a job. Returns a BMI_STATE. */
TASK_DECLARE(BMI088_Init, BMI088_Init_args_t, 1024);

/** @brief Arguments of the BMI088_ApplyConfig task. */
typedef struct {
    bmi088_t           *imu;         /**< Initialised sensor.                */
    bmi_reg_config_t    reg;         /**< Register configuration to apply.   */
    uint32_t            timeout_ms;  /**< Max wait for the sensor semaphore. */
} BMI088_ApplyConfig_args_t;

/** @brief Run BMI088_ApplyConfig_RTOS in a job. Returns a BMI_STATE. */
TASK_DECLARE(BMI088_ApplyConfig, BMI088_ApplyConfig_args_t, 1024);

/**
 * @brief Counters of the acquisition task. Written by the task only; read
 *        them from anywhere (32-bit reads are atomic on Cortex-M4).
 */
typedef struct {
    volatile uint32_t   cycles;      /**< Periods completed.                     */
    volatile uint32_t   errors;      /**< Reads that failed.                     */
    volatile uint32_t   overruns;    /**< Periods missed (task woke up late).    */
    volatile BMI_STATE  last_error;  /**< Status of the last failed read.        */
} bmi_acq_stats_t;

/** @brief Arguments of the BMI088_Acquire task. */
typedef struct {
    bmi088_t           *imu;           /**< Initialised sensor.                          */
    uint32_t            period_ms;     /**< Acquisition period, >= 1 (tick = 1 ms).       */
    data_topic_t       *acc_topic;     /**< float3_ts_t samples, or NULL to skip the ACC. */
    data_topic_t       *gyr_topic;     /**< float3_ts_t samples, or NULL to skip the GYR. */
    data_topic_t       *temp_topic;    /**< float_ts_t samples, or NULL to skip the temp. */
    uint32_t            temp_divider;  /**< Temperature read every N periods (>= 1).      */
    bmi_acq_stats_t    *stats;         /**< Counters, or NULL.                            */
    volatile bool      *stop;          /**< Job ends when *stop becomes true; NULL: never.*/
    uint32_t            timeout_ms;    /**< Max wait for the sensor semaphore, per read.  */
} BMI088_Acquire_args_t;

/**
 * @brief Periodic acquisition: every `period_ms`, read the selected sensors and
 *        publish timestamped samples (ts = kernel tick, ms) on their topics.
 *
 * Paced by osDelayUntil, so the period does not drift. A late wake-up counts
 * one overrun per missed period and restarts the schedule from now, rather
 * than firing a burst of catch-up reads. A failed read is counted and skipped;
 * the loop goes on. Returns BMI_OK once *stop is seen, BMI_INVALID_ARG at once
 * on bad arguments. The topics belong to the application (storage, init).
 */
TASK_DECLARE(BMI088_Acquire, BMI088_Acquire_args_t, 1024);

/** @brief Recover a driver status from a task return value (task_ret_t is int32_t). */
#define BMI_RET(ret_)   ((BMI_STATE)(ret_))

#ifdef __cplusplus
}
#endif

#endif /* APEX_CFG_SCHED_RTOS == 1 */
#endif /* BMI088_RTOS_H */
