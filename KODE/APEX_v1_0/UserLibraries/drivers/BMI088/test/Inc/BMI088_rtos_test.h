/**
 * @file BMI088_rtos_test.h
 * @brief BMI088 RTOS driver test suite.
 *
 * @details
 * Complete RTOS test suite covering all Level-1 RTOS primitives and
 * concurrency/race-condition scenarios relevant to the embedded real-time context.
 *
 * Test organisation
 * -----------------
 *  Level-1 RTOS Primitives (T0–T7)
 *   T0  ReadRegister_RTOS_base(lock=true)  — ACC chip ID
 *   T1  ReadRegister_RTOS_base(lock=true)  — GYR chip ID
 *   T2  WriteRegister_RTOS_base + ReadRegister_RTOS_base — ACC_CONF roundtrip
 *   T3  ReadMultiple_RTOS_base(lock=true)  — 6-byte ACC burst from 0x12
 *   T4  ReadID_RTOS_base(lock=true)        — both chip IDs in one call
 *   T5  SoftReset_RTOS_base ACC(lock=true) — reset, re-enable, chip-ID check
 *   T6  SoftReset_RTOS_base GYR(lock=true) — reset, chip-ID check
 *   T7  _NoLock bracket — manual sem acquire→NoLock call→release, correctness
 *
 *  Race Conditions & RTOS Concurrency (T8–T12)
 *   T8  Concurrent ACC+GYR reads   — 2 tasks fire simultaneously on the same SPI bus
 *   T9  Semaphore serialization    — holder task blocks waiter until it releases the sem
 *   T10 TASK_BMI088_ReadAcc topic  — data_topic coherence, subscriber sampling
 *   T11 TASK_BMI088_ReadGyr topic  — mirror of T10 for gyroscope
 *   T12 ApplyConfig_RTOS under load — reconfigure while publish tasks run
 *
 * @note T0–T9 execute the sensor operation in dedicated RTOS helper tasks
 *       (not directly in the orchestrator thread), to match nominal usage.
 *
 * @note Call BMI088_rtos_test_set_context() before running any test.
 *       Call BMI088_rtos_test_init_pools() in setup() before osKernelStart().
 */

#ifndef BMI088_RTOS_TEST_H
#define BMI088_RTOS_TEST_H

#include "BMI088.h"
#include "data_topic.h"
#include "test.h"
#include "scheduler.h"

#include "cmsis_os2.h"

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Suite constants
 * ========================================================================= */

#define BMI088_RTOS_TEST_N_TESTS    13

extern TEST_case_table_t BMI088_rtos_test_cases[BMI088_RTOS_TEST_N_TESTS];

/* =========================================================================
 * Context injection
 * ========================================================================= */

/**
 * @brief Inject the BMI088 handle and nominal config before running tests.
 *
 * @param bmi    Pointer to the already-initialised BMI088 handle.
 * @param config Pointer to the nominal configuration.
 * @param acc_topic Pointer to the data_topic_t pointer for accelerometer data.
 *                  Set to NULL if TASK_BMI088_ReadAcc is not running.
 * @param gyr_topic Pointer to the data_topic_t pointer for gyroscope data.
 *                  Set to NULL if TASK_BMI088_ReadGyr is not running.
 */
void BMI088_rtos_test_set_context(bmi088_t          *bmi,
                                  const bmi_config_t *config,
                                  data_topic_t      **acc_topic,
                                  data_topic_t      **gyr_topic);

/* =========================================================================
 * Pool initialisation — call once in setup() before osKernelStart()
 * ========================================================================= */

/**
 * @brief Create all RTOS memory pools used by the concurrency tests.
 *        Must be called between osKernelInitialize() and osKernelStart().
 */
void BMI088_rtos_test_init_pools(void);

/* =========================================================================
 * Helper RTOS task definitions for concurrency tests
 *
 *  These tasks are internal to the test module but must have their pools
 *  declared here because TASK_POOL_CONFIGURE expands extern declarations.
 * ========================================================================= */

/* ---- T0..T7 : per-test worker tasks --- */

/**
 * @brief Result carrier written by every T0..T7 worker task.
 *        Accessed through a pointer stored in the args struct, so the pool's
 *        deep-copy of the args does not prevent the orchestrator from reading
 *        back the results.
 */
typedef struct BMI_L1Worker_Shared {
    volatile BMI_STATE st_main;
    volatile BMI_STATE st_aux;
    volatile uint8_t   v0;
    volatile uint8_t   v1;
    volatile uint8_t   raw[6];
    volatile bool      ok;
} BMI_L1Worker_Shared;

/**
 * @brief Common arguments layout for all T0..T7 worker tasks.
 *        Each test aliases this type under its own name so the pool macros
 *        bind correctly (pool uses task_name##_ARGS for sizing).
 */
typedef struct BMI_L1Task_ARGS {
    bmi088_t              *imu;      /**< IMU handle (pointer deep-copied by pool) */
    const bmi_config_t    *cfg;      /**< Nominal config (pointer deep-copied)     */
    osEventFlagsId_t       flags;    /**< Bit 0 set by worker when complete        */
    BMI_L1Worker_Shared   *shared;   /**< Result struct — pointer, NOT value       */
} BMI_L1Task_ARGS;

/* One typedef alias per test so each pool gets its own type. */
typedef BMI_L1Task_ARGS TASK_BMI_T0_ARGS;
typedef BMI_L1Task_ARGS TASK_BMI_T1_ARGS;
typedef BMI_L1Task_ARGS TASK_BMI_T2_ARGS;
typedef BMI_L1Task_ARGS TASK_BMI_T3_ARGS;
typedef BMI_L1Task_ARGS TASK_BMI_T4_ARGS;
typedef BMI_L1Task_ARGS TASK_BMI_T5_ARGS;
typedef BMI_L1Task_ARGS TASK_BMI_T6_ARGS;
typedef BMI_L1Task_ARGS TASK_BMI_T7_ARGS;

TASK_POOL_CONFIGURE(TASK_BMI_T0, 1, 384); void TASK_BMI_T0(void *argument);
TASK_POOL_CONFIGURE(TASK_BMI_T1, 1, 384); void TASK_BMI_T1(void *argument);
TASK_POOL_CONFIGURE(TASK_BMI_T2, 1, 512); void TASK_BMI_T2(void *argument);
TASK_POOL_CONFIGURE(TASK_BMI_T3, 1, 384); void TASK_BMI_T3(void *argument);
TASK_POOL_CONFIGURE(TASK_BMI_T4, 1, 384); void TASK_BMI_T4(void *argument);
TASK_POOL_CONFIGURE(TASK_BMI_T5, 1, 512); void TASK_BMI_T5(void *argument);
TASK_POOL_CONFIGURE(TASK_BMI_T6, 1, 512); void TASK_BMI_T6(void *argument);
TASK_POOL_CONFIGURE(TASK_BMI_T7, 1, 384); void TASK_BMI_T7(void *argument);

/* ---- T0..T7 concurrency contender ---- */

/**
 * @brief Contender task for T0..T7 concurrency: fired simultaneously with
 *        the main worker.  Reads the chip ID of the opposite sensor,
 *        contesting the shared SPI bus semaphore.
 *        Results are written through pointers (pool deep-copies the pointer
 *        value, not the pointed data).  Sets @p flag_bit when complete.
 */
typedef struct TASK_BMI_Contender_ARGS {
    bmi088_t         *imu;         /**< Shared IMU handle                    */
    bool              is_gyr;      /**< true → read GYR id, false → ACC id   */
    BMI_STATE        *result;      /**< Written via pointer — orchestrator var */
    uint8_t          *chip_id;     /**< Written via pointer — orchestrator var */
    osEventFlagsId_t  flags;       /**< Shared event flags group             */
    uint32_t          flag_bit;    /**< Bit to set on completion (0x02U)     */
} TASK_BMI_Contender_ARGS;
TASK_POOL_CONFIGURE(TASK_BMI_Contender, 1, 256);
void TASK_BMI_Contender(void *argument);

/* ---- T8 : concurrent read --- */

typedef struct TASK_BMI_TestConcAcc_ARGS {
    bmi088_t            *imu;
    BMI_STATE           *result;
    uint8_t             *chip_id;
    osEventFlagsId_t     flags;       /**< Bit 0 set when task is done */
} TASK_BMI_TestConcAcc_ARGS;
TASK_POOL_CONFIGURE(TASK_BMI_TestConcAcc, 1, 384);
void TASK_BMI_TestConcAcc(void *argument);

typedef struct TASK_BMI_TestConcGyr_ARGS {
    bmi088_t            *imu;
    BMI_STATE           *result;
    uint8_t             *chip_id;
    osEventFlagsId_t     flags;       /**< Bit 1 set when task is done */
} TASK_BMI_TestConcGyr_ARGS;
TASK_POOL_CONFIGURE(TASK_BMI_TestConcGyr, 1, 384);
void TASK_BMI_TestConcGyr(void *argument);

/* ---- T9 : semaphore serialization --- */

/** Shared state for T9 — written by both helper tasks, read by the test fn. */
typedef struct BMI_SemTest_Shared {
    volatile bool    holder_done;     /**< Set after holder completes its NoLock op */
    volatile bool    holder_id_ok;    /**< Holder chip-ID read was correct */
    volatile bool    waiter_done;     /**< Set after waiter completes its _RTOS op */
    volatile bool    waiter_id_ok;    /**< Waiter chip-ID read was correct */
    volatile uint32_t waiter_start_tick; /**< HAL tick when waiter called _RTOS */
    volatile uint32_t waiter_end_tick;   /**< HAL tick when waiter returned */
} BMI_SemTest_Shared;

typedef struct TASK_BMI_TestSemHolder_ARGS {
    bmi088_t            *imu;
    BMI_SemTest_Shared  *shared;
    osEventFlagsId_t     flags;       /**< Bit 0 set when done */
    uint32_t             hold_ms;     /**< How long to hold the semaphore (ms) */
} TASK_BMI_TestSemHolder_ARGS;
TASK_POOL_CONFIGURE(TASK_BMI_TestSemHolder, 1, 384);
void TASK_BMI_TestSemHolder(void *argument);

typedef struct TASK_BMI_TestSemWaiter_ARGS {
    bmi088_t            *imu;
    BMI_SemTest_Shared  *shared;
    osEventFlagsId_t     flags;       /**< Bit 1 set when done */
} TASK_BMI_TestSemWaiter_ARGS;
TASK_POOL_CONFIGURE(TASK_BMI_TestSemWaiter, 1, 384);
void TASK_BMI_TestSemWaiter(void *argument);

/* =========================================================================
 * Test function declarations  (T0 – T12)
 * ========================================================================= */

void BMI088_rtos_test_t0_read_reg_acc_id         (TEST_case_t *tc);
void BMI088_rtos_test_t1_read_reg_gyr_id         (TEST_case_t *tc);
void BMI088_rtos_test_t2_write_read_acc_conf     (TEST_case_t *tc);
void BMI088_rtos_test_t3_read_multiple_acc_burst (TEST_case_t *tc);
void BMI088_rtos_test_t4_read_id_both            (TEST_case_t *tc);
void BMI088_rtos_test_t5_soft_reset_acc          (TEST_case_t *tc);
void BMI088_rtos_test_t6_soft_reset_gyr          (TEST_case_t *tc);
void BMI088_rtos_test_t7_nolock_bracket          (TEST_case_t *tc);
void BMI088_rtos_test_t8_concurrent_reads        (TEST_case_t *tc);
void BMI088_rtos_test_t9_sem_serialization       (TEST_case_t *tc);
void BMI088_rtos_test_t10_acc_topic_coherence    (TEST_case_t *tc);
void BMI088_rtos_test_t11_gyr_topic_coherence    (TEST_case_t *tc);
void BMI088_rtos_test_t12_apply_config_under_load(TEST_case_t *tc);

#ifdef __cplusplus
}
#endif

#endif /* BMI088_RTOS_TEST_H */
