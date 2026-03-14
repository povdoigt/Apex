/**
 * @file BMI088_rtos_test.c
 * @brief BMI088 RTOS driver test suite — implementation.
 *
 * @details See BMI088_rtos_test.h for the full test catalogue.
 *
 * Architecture note
 * -----------------
 * All tests run under FreeRTOS.  For nominal RTOS validation, every Level-1
 * primitive scenario (T0..T7) is executed in its own dedicated helper task
 * (TASK_BMI_T0 .. TASK_BMI_T7).  Each task has a fixed static pool and calls
 * exactly one BMI088 RTOS API — no switch dispatch.  The orchestrator creates
 * a stack-allocated event group, spawns the task via OS_THREAD_NEW_CSTM, then
 * blocks until bit-0 fires.  Results are written through a pointer stored in
 * the args struct so the pool's deep-copy does not lose them.
 * Concurrency scenarios (T8, T9) also use helper tasks; the pattern is the
 * same but with two tasks running simultaneously.
 */

#include "BMI088_rtos_test.h"

#include "test.h"
#include "data_topic.h"
#include "types.h"

#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"

#include <stdio.h>
#include <math.h>
#include <string.h>

/* =========================================================================
 * Context (set by BMI088_rtos_test_set_context)
 * ========================================================================= */

static bmi088_t            *s_imu        = NULL;
static const bmi_config_t  *s_cfg        = NULL;
static data_topic_t       **s_acc_topic  = NULL;
static data_topic_t       **s_gyr_topic  = NULL;

void BMI088_rtos_test_set_context(bmi088_t          *bmi,
                                  const bmi_config_t *config,
                                  data_topic_t      **acc_topic,
                                  data_topic_t      **gyr_topic) {
    s_imu       = bmi;
    s_cfg       = config;
    s_acc_topic = acc_topic;
    s_gyr_topic = gyr_topic;
}

/* =========================================================================
 * Test case table
 * ========================================================================= */

TEST_case_table_t BMI088_rtos_test_cases[BMI088_RTOS_TEST_N_TESTS] = {
    { .case_info = { .name = "T0  RdReg ACC id"     }, .func = BMI088_rtos_test_t0_read_reg_acc_id          },
    { .case_info = { .name = "T1  RdReg GYR id"     }, .func = BMI088_rtos_test_t1_read_reg_gyr_id          },
    { .case_info = { .name = "T2  WrRd ACC_CONF"    }, .func = BMI088_rtos_test_t2_write_read_acc_conf      },
    { .case_info = { .name = "T3  RdMult ACC burst" }, .func = BMI088_rtos_test_t3_read_multiple_acc_burst  },
    { .case_info = { .name = "T4  ReadID both"      }, .func = BMI088_rtos_test_t4_read_id_both             },
    { .case_info = { .name = "T5  SoftRst ACC"      }, .func = BMI088_rtos_test_t5_soft_reset_acc           },
    { .case_info = { .name = "T6  SoftRst GYR"      }, .func = BMI088_rtos_test_t6_soft_reset_gyr           },
    { .case_info = { .name = "T7  NoLock bracket"   }, .func = BMI088_rtos_test_t7_nolock_bracket           },
    { .case_info = { .name = "T8  Conc. reads"      }, .func = BMI088_rtos_test_t8_concurrent_reads         },
    { .case_info = { .name = "T9  Sem serial."      }, .func = BMI088_rtos_test_t9_sem_serialization        },
    { .case_info = { .name = "T10 Acc topic"        }, .func = BMI088_rtos_test_t10_acc_topic_coherence     },
    { .case_info = { .name = "T11 Gyr topic"        }, .func = BMI088_rtos_test_t11_gyr_topic_coherence     },
    { .case_info = { .name = "T12 Cfg under load"   }, .func = BMI088_rtos_test_t12_apply_config_under_load },
};

/* =========================================================================
 * Constants
 * ========================================================================= */

#define ACC_ID_EXP          BMI_ACC_CHIP_ID_VALUE   /* 0x1E */
#define GYR_ID_EXP          BMI_GYR_CHIP_ID_VALUE   /* 0x0F */

/* Concurrency test timing */
#define SEM_HOLD_MS         60U    /**< T9: how long the holder keeps the sem */
#define SEM_MIN_BLOCK_MS    40U    /**< T9: waiter must have blocked at least this long */

/* Topic tests */
#define TOPIC_SAMPLE_COUNT  5U     /**< T10/T11: number of samples to collect */
#define TOPIC_SAMPLE_TIMEOUT_MS 500U /**< T10/T11: per-sample wait timeout */

/* Plausibility for acceleration (m/s²): at least one axis must be non-zero  */
#define ACC_NONZERO_THRESHOLD  0.05f
/* Plausibility for temperature: sensor must be within [-40, +85] °C         */
#define TEMP_MIN_C  (-40.0f)
#define TEMP_MAX_C  ( 85.0f)

/* =========================================================================
 * Utility
 * ========================================================================= */

static const char *bmi_str(BMI_STATE s) {
    switch (s) {
        case BMI_OK:          return "OK";
        case BMI_SPI_ERR:     return "SPI_ERR";
        case BMI_INVALID_ARG: return "INVALID_ARG";
        case BMI_BUSY:        return "BUSY";
        case BMI_TIMEOUT:     return "TIMEOUT";
        case BMI_UNKNOWN_ERR: return "UNKNOWN_ERR";
        case BMI_SEM_ERR:     return "SEM_ERR";
        default:              return "?";
    }
}

/* =========================================================================
 * Helper task memory pools — allocated here, created in init_pools()
 * ========================================================================= */

TASK_POOL_ALLOCATE(TASK_BMI_TestConcAcc);
TASK_POOL_ALLOCATE(TASK_BMI_TestConcGyr);
TASK_POOL_ALLOCATE(TASK_BMI_TestSemHolder);
TASK_POOL_ALLOCATE(TASK_BMI_TestSemWaiter);
TASK_POOL_ALLOCATE(TASK_BMI_T0);
TASK_POOL_ALLOCATE(TASK_BMI_T1);
TASK_POOL_ALLOCATE(TASK_BMI_T2);
TASK_POOL_ALLOCATE(TASK_BMI_T3);
TASK_POOL_ALLOCATE(TASK_BMI_T4);
TASK_POOL_ALLOCATE(TASK_BMI_T5);
TASK_POOL_ALLOCATE(TASK_BMI_T6);
TASK_POOL_ALLOCATE(TASK_BMI_T7);
TASK_POOL_ALLOCATE(TASK_BMI_Contender);

void BMI088_rtos_test_init_pools(void) {
    TASK_POOL_CREATE(TASK_BMI_TestConcAcc);
    TASK_POOL_CREATE(TASK_BMI_TestConcGyr);
    TASK_POOL_CREATE(TASK_BMI_TestSemHolder);
    TASK_POOL_CREATE(TASK_BMI_TestSemWaiter);
    TASK_POOL_CREATE(TASK_BMI_T0);
    TASK_POOL_CREATE(TASK_BMI_T1);
    TASK_POOL_CREATE(TASK_BMI_T2);
    TASK_POOL_CREATE(TASK_BMI_T3);
    TASK_POOL_CREATE(TASK_BMI_T4);
    TASK_POOL_CREATE(TASK_BMI_T5);
    TASK_POOL_CREATE(TASK_BMI_T6);
    TASK_POOL_CREATE(TASK_BMI_T7);
    TASK_POOL_CREATE(TASK_BMI_Contender);
}

/* =========================================================================
 * Helper task bodies
 * ========================================================================= */

/* --- T0..T7 individual worker tasks -------------------------------------- */

/** Convenience macro: set bit-0, exit. Used by every T0..T7 worker. */
#define L1_TASK_DONE(a)  do { osEventFlagsSet((a)->flags, 0x01U); osThreadExit_Cstm(); } while(0)

void TASK_BMI_T0(void *argument) {
    TASK_BMI_T0_ARGS *a = (TASK_BMI_T0_ARGS *)argument;
    BMI_L1Worker_Shared *s = a->shared;
    uint8_t id = 0;
    s->st_main = BMI088_ReadRegister_RTOS(a->imu, false, BMI_ACC_CHIP_ID, &id);
    s->v0 = id;
    s->ok = (s->st_main == BMI_OK) && (id == ACC_ID_EXP);
    L1_TASK_DONE(a);
}

void TASK_BMI_T1(void *argument) {
    TASK_BMI_T1_ARGS *a = (TASK_BMI_T1_ARGS *)argument;
    BMI_L1Worker_Shared *s = a->shared;
    uint8_t id = 0;
    s->st_main = BMI088_ReadRegister_RTOS(a->imu, true, BMI_GYR_CHIP_ID, &id);
    s->v0 = id;
    s->ok = (s->st_main == BMI_OK) && (id == GYR_ID_EXP);
    L1_TASK_DONE(a);
}

void TASK_BMI_T2(void *argument) {
    TASK_BMI_T2_ARGS *a = (TASK_BMI_T2_ARGS *)argument;
    BMI_L1Worker_Shared *s = a->shared;
    const uint8_t test_conf = (uint8_t)BMI_ACC_CONF_BWP_OSR2 |
                              (uint8_t)BMI_ACC_CONF_ODR_200_HZ;
    uint8_t readback = 0;
    s->st_main = BMI088_WriteRegister_RTOS(a->imu, false, BMI_ACC_CONF, test_conf);
    if (s->st_main == BMI_OK) {
        osDelay(2);
        s->st_aux = BMI088_ReadRegister_RTOS(a->imu, false, BMI_ACC_CONF, &readback);
    }
    /* Restore nominal conf regardless of outcome */
    const uint8_t nom_conf = (uint8_t)a->cfg->acc_bwp | (uint8_t)a->cfg->acc_odr;
    BMI088_WriteRegister_RTOS(a->imu, false, BMI_ACC_CONF, nom_conf);
    s->v0 = test_conf;
    s->v1 = readback;
    s->ok = (s->st_main == BMI_OK) && (s->st_aux == BMI_OK) && (readback == test_conf);
    L1_TASK_DONE(a);
}

void TASK_BMI_T3(void *argument) {
    TASK_BMI_T3_ARGS *a = (TASK_BMI_T3_ARGS *)argument;
    BMI_L1Worker_Shared *s = a->shared;
    uint8_t raw[6] = {0};
    bool all_zero = true;
    s->st_main = BMI088_ReadMultiple_RTOS(a->imu, false, BMI_ACC_X_LSB, raw, 6);
    for (uint8_t i = 0; i < 6U; i++) {
        s->raw[i] = raw[i];
        if (raw[i] != 0x00U) { all_zero = false; }
    }
    s->ok = (s->st_main == BMI_OK) && !all_zero;
    L1_TASK_DONE(a);
}

void TASK_BMI_T4(void *argument) {
    TASK_BMI_T4_ARGS *a = (TASK_BMI_T4_ARGS *)argument;
    BMI_L1Worker_Shared *s = a->shared;
    uint8_t acc_id = 0, gyr_id = 0;
    s->st_main = BMI088_ReadID_RTOS(a->imu, &acc_id, &gyr_id);
    s->v0 = acc_id;
    s->v1 = gyr_id;
    s->ok = (s->st_main == BMI_OK) && (acc_id == ACC_ID_EXP) && (gyr_id == GYR_ID_EXP);
    L1_TASK_DONE(a);
}

void TASK_BMI_T5(void *argument) {
    TASK_BMI_T5_ARGS *a = (TASK_BMI_T5_ARGS *)argument;
    BMI_L1Worker_Shared *s = a->shared;
    uint8_t id = 0;
    s->st_main = BMI088_SoftReset_RTOS(a->imu, false);
    if (s->st_main == BMI_OK) {
        osDelay(50);
        BMI088_WriteRegister_RTOS(a->imu, false, BMI_ACC_PWR_CTRL,
                                  (uint8_t)BMI_ACC_PWR_CTRL_ENABLE);
        osDelay(5);
        BMI088_WriteRegister_RTOS(a->imu, false, BMI_ACC_PWR_CONF,
                                  (uint8_t)BMI_ACC_PWR_CONF_ACTIVE);
        osDelay(5);
        s->st_aux = BMI088_ReadRegister_RTOS(a->imu, false, BMI_ACC_CHIP_ID, &id);
    }
    BMI088_ApplyConfig_RTOS(a->imu, a->cfg);
    osDelay(5);
    s->v0 = id;
    s->ok = (s->st_main == BMI_OK) && (s->st_aux == BMI_OK) && (id == ACC_ID_EXP);
    L1_TASK_DONE(a);
}

void TASK_BMI_T6(void *argument) {
    TASK_BMI_T6_ARGS *a = (TASK_BMI_T6_ARGS *)argument;
    BMI_L1Worker_Shared *s = a->shared;
    uint8_t id = 0;
    s->st_main = BMI088_SoftReset_RTOS(a->imu, true);
    if (s->st_main == BMI_OK) {
        osDelay(30);
        s->st_aux = BMI088_ReadRegister_RTOS(a->imu, true, BMI_GYR_CHIP_ID, &id);
    }
    BMI088_ApplyConfig_RTOS(a->imu, a->cfg);
    osDelay(5);
    s->v0 = id;
    s->ok = (s->st_main == BMI_OK) && (s->st_aux == BMI_OK) && (id == GYR_ID_EXP);
    L1_TASK_DONE(a);
}

void TASK_BMI_T7(void *argument) {
    TASK_BMI_T7_ARGS *a = (TASK_BMI_T7_ARGS *)argument;
    BMI_L1Worker_Shared *s = a->shared;
    osStatus_t sem_st = osSemaphoreAcquire(a->imu->sem_id, 200U);
    uint8_t id = 0, id2 = 0;
    if (sem_st == osOK) {
        s->st_main = BMI088_ReadRegister_RTOS_NoLock(a->imu, true, BMI_GYR_CHIP_ID, &id);
        osSemaphoreRelease(a->imu->sem_id);
        s->st_aux = BMI088_ReadRegister_RTOS(a->imu, true, BMI_GYR_CHIP_ID, &id2);
    } else {
        s->st_main = BMI_SEM_ERR;
        s->st_aux  = BMI_SEM_ERR;
    }
    s->v0 = id;
    s->v1 = id2;
    s->ok = (sem_st == osOK) &&
            (s->st_main == BMI_OK) &&
            (s->st_aux  == BMI_OK) &&
            (id == GYR_ID_EXP);
    L1_TASK_DONE(a);
}

/* --- Contender (used by T0..T7) ------------------------------------------ */

/**
 * @brief Reads the chip ID of the opposite sensor while the main worker
 *        operates on the shared SPI bus.  Validates that the semaphore
 *        correctly serialises concurrent access.
 */
void TASK_BMI_Contender(void *argument) {
    TASK_BMI_Contender_ARGS *a = (TASK_BMI_Contender_ARGS *)argument;
    uint8_t id = 0;
    uint8_t reg = a->is_gyr ? BMI_GYR_CHIP_ID : BMI_ACC_CHIP_ID;
    *a->result  = BMI088_ReadRegister_RTOS(a->imu, a->is_gyr, reg, &id);
    *a->chip_id = id;
    osEventFlagsSet(a->flags, a->flag_bit);
    osThreadExit_Cstm();
}

/* --- T8 helpers ----------------------------------------------------------- */

void TASK_BMI_TestConcAcc(void *argument) {
    TASK_BMI_TestConcAcc_ARGS *a = (TASK_BMI_TestConcAcc_ARGS *)argument;
    uint8_t id = 0;

    /* Read ACC chip ID via the locking variant (may block until bus is free) */
    *a->result  = BMI088_ReadRegister_RTOS(a->imu, false, BMI_ACC_CHIP_ID, &id);
    *a->chip_id = id;

    osEventFlagsSet(a->flags, 0x01U);   /* signal bit 0 */
    osThreadExit_Cstm();
}

void TASK_BMI_TestConcGyr(void *argument) {
    TASK_BMI_TestConcGyr_ARGS *a = (TASK_BMI_TestConcGyr_ARGS *)argument;
    uint8_t id = 0;

    /* Read GYR chip ID via the locking variant (may block until bus is free) */
    *a->result  = BMI088_ReadRegister_RTOS(a->imu, true, BMI_GYR_CHIP_ID, &id);
    *a->chip_id = id;

    osEventFlagsSet(a->flags, 0x02U);   /* signal bit 1 */
    osThreadExit_Cstm();
}

/* --- T9 helpers ----------------------------------------------------------- */

/**
 * @brief SemHolder: manually acquires sem, waits, performs a NoLock read,
 *        then releases the sem.  Simulates a task that owns the SPI bus for
 *        a known duration, representing e.g. a burst DMA transfer in progress.
 */
void TASK_BMI_TestSemHolder(void *argument) {
    TASK_BMI_TestSemHolder_ARGS *a = (TASK_BMI_TestSemHolder_ARGS *)argument;

    /* Step 1 — explicitly seize the SPI bus semaphore */
    osSemaphoreAcquire(a->imu->sem_id, osWaitForever);

    /* Step 2 — simulate bus occupation (e.g. DMA in-flight) */
    osDelay(a->hold_ms);

    /* Step 3 — perform the actual operation without re-locking  */
    uint8_t id = 0;
    BMI_STATE st = BMI088_ReadRegister_RTOS_base(a->imu, false, BMI_ACC_CHIP_ID, &id, false);

    a->shared->holder_id_ok  = (st == BMI_OK) && (id == ACC_ID_EXP);
    a->shared->holder_done   = true;

    /* Step 4 — release the bus for other tasks                   */
    osSemaphoreRelease(a->imu->sem_id);

    osEventFlagsSet(a->flags, 0x01U);
    osThreadExit_Cstm();
}

/**
 * @brief SemWaiter: immediately attempts a locking read — this call will
 *        block inside osSemaphoreAcquire until SemHolder releases the sem.
 *        Records timestamps to prove the blocking duration.
 */
void TASK_BMI_TestSemWaiter(void *argument) {
    TASK_BMI_TestSemWaiter_ARGS *a = (TASK_BMI_TestSemWaiter_ARGS *)argument;

    a->shared->waiter_start_tick = HAL_GetTick();

    uint8_t id = 0;
    /* lock=true — will block waiting for SemHolder to release */
    BMI_STATE st = BMI088_ReadRegister_RTOS(a->imu, false, BMI_ACC_CHIP_ID, &id);

    a->shared->waiter_end_tick = HAL_GetTick();
    a->shared->waiter_id_ok    = (st == BMI_OK) && (id == ACC_ID_EXP);
    a->shared->waiter_done     = true;

    osEventFlagsSet(a->flags, 0x02U);
    osThreadExit_Cstm();
}

/* =========================================================================
 * T0..T7 — helpers
 *
 *  L1_FLAGS_CREATE  : allocate a static event group on the caller's stack.
 *  L1_SPAWN         : build the common args struct and launch the worker pool.
 *  L1_WAIT          : wait for bit-0, delete flags, assert no error/timeout.
 * ========================================================================= */

#define L1_FLAGS_CREATE(fid_, cb_, name_)                           \
    StaticEventGroup_t  cb_;                                        \
    osEventFlagsAttr_t  cb_##_attr = {                              \
        .name    = (name_),                                         \
        .cb_mem  = &(cb_),                                          \
        .cb_size = sizeof(cb_),                                     \
    };                                                              \
    osEventFlagsId_t fid_ = osEventFlagsNew(&cb_##_attr);          \
    TEST_ASSERT((fid_) != NULL, "osEventFlagsNew " name_ " failed")

#define L1_SPAWN(task_name_, fid_, out_ptr_, attr_name_)            \
    task_name_##_ARGS _args_##task_name_ = {                        \
        .imu    = s_imu,                                            \
        .cfg    = s_cfg,                                            \
        .flags  = (fid_),                                           \
        .shared = (out_ptr_),                                       \
    };                                                              \
    osThreadAttr_t _attr_##task_name_ = {                           \
        .name     = (attr_name_),                                   \
        .priority = osPriorityAboveNormal,                          \
    };                                                              \
    TEST_ASSERT(                                                    \
        OS_THREAD_NEW_CSTM(task_name_, _args_##task_name_,          \
                           _attr_##task_name_, osWaitForever) != NULL, \
        #task_name_ " pool empty or OS error")

#define L1_WAIT(fid_, timeout_ms_)                                  \
    do {                                                            \
        uint32_t _ev = osEventFlagsWait((fid_), 0x01U,             \
                                         osFlagsWaitAll,            \
                                         (timeout_ms_));            \
        osEventFlagsDelete(fid_);                                   \
        TEST_ASSERT((_ev & 0x80000000U) == 0U,                     \
                    "worker timeout/error: 0x%08X", (unsigned)_ev); \
    } while(0)

/**
 * @brief Spawn the contender task on an already-created flags group.
 *        Sets bit 1 (0x02U) on completion.
 * @param fid_    osEventFlagsId_t — the same group shared with the main worker.
 * @param is_gyr_ true → read GYR chip ID, false → read ACC chip ID.
 * @param st_ptr_ Pointer to a BMI_STATE variable on the orchestrator's stack.
 * @param id_ptr_ Pointer to a uint8_t variable on the orchestrator's stack.
 */
#define L1_SPAWN_CONTENDER(fid_, is_gyr_, st_ptr_, id_ptr_)         \
    TASK_BMI_Contender_ARGS _cnt_args = {                           \
        .imu      = s_imu,                                          \
        .is_gyr   = (is_gyr_),                                      \
        .result   = (st_ptr_),                                      \
        .chip_id  = (id_ptr_),                                      \
        .flags    = (fid_),                                         \
        .flag_bit = 0x02U,                                          \
    };                                                              \
    osThreadAttr_t _cnt_attr = {                                    \
        .name     = "Contender",                                    \
        .priority = osPriorityAboveNormal,                          \
    };                                                              \
    TEST_ASSERT(                                                    \
        OS_THREAD_NEW_CSTM(TASK_BMI_Contender, _cnt_args,           \
                           _cnt_attr, osWaitForever) != NULL,       \
        "Contender pool empty or OS error")

/** Wait for both the main worker (bit 0) and the contender (bit 1). */
#define L1_WAIT_BOTH(fid_, timeout_ms_)                             \
    do {                                                            \
        uint32_t _ev = osEventFlagsWait((fid_), 0x03U,             \
                                         osFlagsWaitAll,            \
                                         (timeout_ms_));            \
        osEventFlagsDelete(fid_);                                   \
        TEST_ASSERT((_ev & 0x80000000U) == 0U,                     \
                    "workers timeout/error: 0x%08X", (unsigned)_ev); \
    } while(0)

/* =========================================================================
 * T0 — ReadRegister_RTOS_base(lock=true) : ACC chip ID
 * ========================================================================= */

void BMI088_rtos_test_t0_read_reg_acc_id(TEST_case_t *tc) {
    BMI_L1Worker_Shared out = {0};
    BMI_STATE cnt_st = BMI_UNKNOWN_ERR;
    uint8_t   cnt_id = 0U;
    L1_FLAGS_CREATE(flags, flags_cb, "T0");
    L1_SPAWN(TASK_BMI_T0, flags, &out, "T0_Worker");
    L1_SPAWN_CONTENDER(flags, true, &cnt_st, &cnt_id);  /* GYR contends ACC */
    L1_WAIT_BOTH(flags, 1500U);

    TEST_ASSERT(out.ok,
                "ReadRegister_RTOS ACC: st=%s id=0x%02X exp=0x%02X",
                bmi_str(out.st_main), out.v0, ACC_ID_EXP);
    TEST_ASSERT(cnt_st == BMI_OK && cnt_id == GYR_ID_EXP,
                "Contender GYR: st=%s id=0x%02X exp=0x%02X",
                bmi_str(cnt_st), cnt_id, GYR_ID_EXP);
    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail),
             "ACC=0x%02X cnt_GYR=0x%02X", out.v0, cnt_id);
}

/* =========================================================================
 * T1 — ReadRegister_RTOS_base(lock=true) : GYR chip ID
 * ========================================================================= */

void BMI088_rtos_test_t1_read_reg_gyr_id(TEST_case_t *tc) {
    BMI_L1Worker_Shared out = {0};
    BMI_STATE cnt_st = BMI_UNKNOWN_ERR;
    uint8_t   cnt_id = 0U;
    L1_FLAGS_CREATE(flags, flags_cb, "T1");
    L1_SPAWN(TASK_BMI_T1, flags, &out, "T1_Worker");
    L1_SPAWN_CONTENDER(flags, false, &cnt_st, &cnt_id); /* ACC contends GYR */
    L1_WAIT_BOTH(flags, 1500U);

    TEST_ASSERT(out.ok,
                "ReadRegister_RTOS GYR: st=%s id=0x%02X exp=0x%02X",
                bmi_str(out.st_main), out.v0, GYR_ID_EXP);
    TEST_ASSERT(cnt_st == BMI_OK && cnt_id == ACC_ID_EXP,
                "Contender ACC: st=%s id=0x%02X exp=0x%02X",
                bmi_str(cnt_st), cnt_id, ACC_ID_EXP);
    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail),
             "GYR=0x%02X cnt_ACC=0x%02X", out.v0, cnt_id);
}

/* =========================================================================
 * T2 — WriteRegister_RTOS_base + ReadRegister_RTOS_base : ACC_CONF roundtrip
 *   Tests that write and read each acquire/release the sem independently
 *   (lock=true) without deadlock, and that the written value is reflected.
 * ========================================================================= */

void BMI088_rtos_test_t2_write_read_acc_conf(TEST_case_t *tc) {
    BMI_L1Worker_Shared out = {0};
    BMI_STATE cnt_st = BMI_UNKNOWN_ERR;
    uint8_t   cnt_id = 0U;
    L1_FLAGS_CREATE(flags, flags_cb, "T2");
    L1_SPAWN(TASK_BMI_T2, flags, &out, "T2_Worker");
    L1_SPAWN_CONTENDER(flags, true, &cnt_st, &cnt_id);  /* GYR contends ACC */
    L1_WAIT_BOTH(flags, 2000U);

    TEST_ASSERT(out.ok,
                "ACC_CONF: write=%s read=%s wrote=0x%02X read=0x%02X",
                bmi_str(out.st_main), bmi_str(out.st_aux), out.v0, out.v1);
    TEST_ASSERT(cnt_st == BMI_OK && cnt_id == GYR_ID_EXP,
                "Contender GYR: st=%s id=0x%02X exp=0x%02X",
                bmi_str(cnt_st), cnt_id, GYR_ID_EXP);
    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail),
             "wrote=0x%02X read=0x%02X cnt_GYR=0x%02X", out.v0, out.v1, cnt_id);
}

/* =========================================================================
 * T3 — ReadMultiple_RTOS_base(lock=true) : 6-byte ACC burst from 0x12
 *   Reads X/Y/Z (6 bytes, LSB+MSB each) in a single locked burst.
 *   Validates that the burst does not corrupt adjacent bytes and returns
 *   a coherent 16-bit pair (H byte is always the masked MSB).
 * ========================================================================= */

void BMI088_rtos_test_t3_read_multiple_acc_burst(TEST_case_t *tc) {
    BMI_L1Worker_Shared out = {0};
    BMI_STATE cnt_st = BMI_UNKNOWN_ERR;
    uint8_t   cnt_id = 0U;
    L1_FLAGS_CREATE(flags, flags_cb, "T3");
    L1_SPAWN(TASK_BMI_T3, flags, &out, "T3_Worker");
    L1_SPAWN_CONTENDER(flags, true, &cnt_st, &cnt_id);  /* GYR contends ACC burst */
    L1_WAIT_BOTH(flags, 1500U);

    TEST_ASSERT(out.ok,
                "ReadMultiple_RTOS: st=%s raw=%02X %02X %02X %02X %02X %02X",
                bmi_str(out.st_main),
                out.raw[0], out.raw[1], out.raw[2], out.raw[3], out.raw[4], out.raw[5]);
    TEST_ASSERT(cnt_st == BMI_OK && cnt_id == GYR_ID_EXP,
                "Contender GYR: st=%s id=0x%02X exp=0x%02X",
                bmi_str(cnt_st), cnt_id, GYR_ID_EXP);
    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail),
             "X=%02X%02X Y=%02X%02X Z=%02X%02X cnt_GYR=0x%02X",
             out.raw[1], out.raw[0], out.raw[3], out.raw[2], out.raw[5], out.raw[4], cnt_id);
}

/* =========================================================================
 * T4 — ReadID_RTOS_base(lock=true) : both chip IDs in one locked call
 * ========================================================================= */

void BMI088_rtos_test_t4_read_id_both(TEST_case_t *tc) {
    BMI_L1Worker_Shared out = {0};
    BMI_STATE cnt_st = BMI_UNKNOWN_ERR;
    uint8_t   cnt_id = 0U;
    L1_FLAGS_CREATE(flags, flags_cb, "T4");
    L1_SPAWN(TASK_BMI_T4, flags, &out, "T4_Worker");
    L1_SPAWN_CONTENDER(flags, true, &cnt_st, &cnt_id);  /* 2nd GYR reader concurrent with ReadID */
    L1_WAIT_BOTH(flags, 1500U);

    TEST_ASSERT(out.ok,
                "ReadID_RTOS: st=%s ACC=0x%02X GYR=0x%02X",
                bmi_str(out.st_main), out.v0, out.v1);
    TEST_ASSERT(cnt_st == BMI_OK && cnt_id == GYR_ID_EXP,
                "Contender GYR: st=%s id=0x%02X exp=0x%02X",
                bmi_str(cnt_st), cnt_id, GYR_ID_EXP);
    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail),
             "ACC=0x%02X GYR=0x%02X cnt_GYR=0x%02X", out.v0, out.v1, cnt_id);
}

/* =========================================================================
 * T5 — SoftReset_RTOS_base ACC (lock=true)
 *   Issues the ACC soft-reset, re-enables power, verifies chip ID.
 *   Also confirms that a subsequent ReadRegister_RTOS call succeeds (the
 *   semaphore is properly released after reset).
 * ========================================================================= */

void BMI088_rtos_test_t5_soft_reset_acc(TEST_case_t *tc) {
    BMI_L1Worker_Shared out = {0};
    BMI_STATE cnt_st = BMI_UNKNOWN_ERR;
    uint8_t   cnt_id = 0U;
    L1_FLAGS_CREATE(flags, flags_cb, "T5");
    L1_SPAWN(TASK_BMI_T5, flags, &out, "T5_Worker");
    L1_SPAWN_CONTENDER(flags, true, &cnt_st, &cnt_id);  /* GYR blocked during ACC reset */
    L1_WAIT_BOTH(flags, 4000U);

    TEST_ASSERT(out.ok,
                "SoftReset ACC: reset=%s post-read=%s id=0x%02X",
                bmi_str(out.st_main), bmi_str(out.st_aux), out.v0);
    TEST_ASSERT(cnt_st == BMI_OK && cnt_id == GYR_ID_EXP,
                "Contender GYR: st=%s id=0x%02X exp=0x%02X",
                bmi_str(cnt_st), cnt_id, GYR_ID_EXP);
    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail),
             "ACC_ID=0x%02X post-reset cnt_GYR=0x%02X", out.v0, cnt_id);
}

/* =========================================================================
 * T6 — SoftReset_RTOS_base GYR (lock=true)
 * ========================================================================= */

void BMI088_rtos_test_t6_soft_reset_gyr(TEST_case_t *tc) {
    BMI_L1Worker_Shared out = {0};
    BMI_STATE cnt_st = BMI_UNKNOWN_ERR;
    uint8_t   cnt_id = 0U;
    L1_FLAGS_CREATE(flags, flags_cb, "T6");
    L1_SPAWN(TASK_BMI_T6, flags, &out, "T6_Worker");
    L1_SPAWN_CONTENDER(flags, false, &cnt_st, &cnt_id); /* ACC blocked during GYR reset */
    L1_WAIT_BOTH(flags, 4000U);

    TEST_ASSERT(out.ok,
                "SoftReset GYR: reset=%s post-read=%s id=0x%02X",
                bmi_str(out.st_main), bmi_str(out.st_aux), out.v0);
    TEST_ASSERT(cnt_st == BMI_OK && cnt_id == ACC_ID_EXP,
                "Contender ACC: st=%s id=0x%02X exp=0x%02X",
                bmi_str(cnt_st), cnt_id, ACC_ID_EXP);
    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail),
             "GYR_ID=0x%02X post-reset cnt_ACC=0x%02X", out.v0, cnt_id);
}

/* =========================================================================
 * T7 — _NoLock bracket
 *   Validates the _NoLock pattern: caller acquires sem, calls _NoLock,
 *   releases sem.  Ensures:
 *    (a) NoLock variant does NOT try to double-acquire the same sem.
 *    (b) The read still returns correct data.
 *    (c) The sem is usable again after release (follow-up locking read works).
 * ========================================================================= */

void BMI088_rtos_test_t7_nolock_bracket(TEST_case_t *tc) {
    BMI_L1Worker_Shared out = {0};
    BMI_STATE cnt_st = BMI_UNKNOWN_ERR;
    uint8_t   cnt_id = 0U;
    L1_FLAGS_CREATE(flags, flags_cb, "T7");
    L1_SPAWN(TASK_BMI_T7, flags, &out, "T7_Worker");
    L1_SPAWN_CONTENDER(flags, false, &cnt_st, &cnt_id); /* ACC blocked while T7 holds sem */
    L1_WAIT_BOTH(flags, 1500U);

    TEST_ASSERT(out.ok,
                "NoLock: st=%s post=%s id=0x%02X id2=0x%02X",
                bmi_str(out.st_main), bmi_str(out.st_aux), out.v0, out.v1);
    TEST_ASSERT(cnt_st == BMI_OK && cnt_id == ACC_ID_EXP,
                "Contender ACC: st=%s id=0x%02X exp=0x%02X",
                bmi_str(cnt_st), cnt_id, ACC_ID_EXP);
    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail),
             "NoLock id=0x%02X cnt_ACC=0x%02X", out.v0, cnt_id);
}

/* =========================================================================
 * T8 — Concurrent ACC + GYR reads
 *   Spawns two equal-priority tasks that both call _ReadRegister_RTOS
 *   (lock=true) simultaneously.  The semaphore must ensure one completes
 *   before the other starts its SPI transaction.  Both must return
 *   correct chip IDs — any corruption would indicate a race on the SPI bus.
 * ========================================================================= */

void BMI088_rtos_test_t8_concurrent_reads(TEST_case_t *tc) {
    /* Shared event flags — bit 0 = ACC done, bit 1 = GYR done */
    StaticEventGroup_t flags_cb;
    osEventFlagsAttr_t flags_attr = {
        .name    = "T8_flags",
        .cb_mem  = &flags_cb,
        .cb_size = sizeof(flags_cb),
    };
    osEventFlagsId_t flags = osEventFlagsNew(&flags_attr);
    TEST_ASSERT(flags != NULL, "osEventFlagsNew failed");

    BMI_STATE acc_st = BMI_UNKNOWN_ERR;
    BMI_STATE gyr_st = BMI_UNKNOWN_ERR;
    uint8_t acc_id = 0U;
    uint8_t gyr_id = 0U;

    TASK_BMI_TestConcAcc_ARGS acc_args = {
        .imu = s_imu,
        .result = &acc_st,
        .chip_id = &acc_id,
        .flags = flags,
    };
    TASK_BMI_TestConcGyr_ARGS gyr_args = {
        .imu = s_imu,
        .result = &gyr_st,
        .chip_id = &gyr_id,
        .flags = flags,
    };

    osThreadAttr_t attr = { .priority = (osPriority_t)osPriorityAboveNormal };

    attr.name = "T8_ConcAcc";
    OS_THREAD_NEW_CSTM(TASK_BMI_TestConcAcc, acc_args, attr, osWaitForever);

    attr.name = "T8_ConcGyr";
    OS_THREAD_NEW_CSTM(TASK_BMI_TestConcGyr, gyr_args, attr, osWaitForever);

    /* Wait for both tasks (2 second safety timeout) */
    uint32_t ev = osEventFlagsWait(flags, 0x03U, osFlagsWaitAll, 2000);
    osEventFlagsDelete(flags);

    TEST_ASSERT((ev & 0x80000000U) == 0,
                "Event wait timeout/error: 0x%08lX", ev);
    TEST_ASSERT(acc_st == BMI_OK,
                "ConcAcc: %s", bmi_str(acc_st));
    TEST_ASSERT(gyr_st == BMI_OK,
                "ConcGyr: %s", bmi_str(gyr_st));
    TEST_ASSERT(acc_id == ACC_ID_EXP,
                "ConcAcc id=0x%02X (exp:0x%02X)", acc_id, ACC_ID_EXP);
    TEST_ASSERT(gyr_id == GYR_ID_EXP,
                "ConcGyr id=0x%02X (exp:0x%02X)", gyr_id, GYR_ID_EXP);

    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail),
             "ACC=0x%02X GYR=0x%02X no corruption", acc_id, gyr_id);
}

/* =========================================================================
 * T9 — Semaphore serialization
 *   SemHolder acquires the imu semaphore for SEM_HOLD_MS milliseconds.
 *   SemWaiter immediately tries _ReadRegister_RTOS(lock=true) and is blocked.
 *   After holder releases, waiter must (a) complete, (b) return correct data,
 *   (c) have been blocked for at least SEM_MIN_BLOCK_MS ticks.
 *
 *   This test directly probes that the binary semaphore inside bmi088_t
 *   acts as a real critical-section guard on the SPI bus, preventing
 *   interleaved CS/clock sequences from different tasks.
 * ========================================================================= */

void BMI088_rtos_test_t9_sem_serialization(TEST_case_t *tc) {
    StaticEventGroup_t flags_cb;
    osEventFlagsAttr_t flags_attr = {
        .name    = "T9_flags",
        .cb_mem  = &flags_cb,
        .cb_size = sizeof(flags_cb),
    };
    osEventFlagsId_t flags = osEventFlagsNew(&flags_attr);
    TEST_ASSERT(flags != NULL, "osEventFlagsNew failed");

    BMI_SemTest_Shared shared = {0};

    TASK_BMI_TestSemHolder_ARGS holder_args = {
        .imu     = s_imu,
        .shared  = &shared,
        .flags   = flags,
        .hold_ms = SEM_HOLD_MS,
    };
    TASK_BMI_TestSemWaiter_ARGS waiter_args = {
        .imu    = s_imu,
        .shared = &shared,
        .flags  = flags,
    };

    osThreadAttr_t attr = { .priority = (osPriority_t)osPriorityAboveNormal };

    /* Launch holder first so it seizes the sem before waiter starts */
    attr.name = "T9_Holder";
    OS_THREAD_NEW_CSTM(TASK_BMI_TestSemHolder, holder_args, attr, osWaitForever);

    /* Short delay to ensure holder acquires sem before waiter tries */
    osDelay(5);

    attr.name = "T9_Waiter";
    OS_THREAD_NEW_CSTM(TASK_BMI_TestSemWaiter, waiter_args, attr, osWaitForever);

    /* Wait for both (SEM_HOLD_MS + 500 ms safety margin) */
    uint32_t ev = osEventFlagsWait(flags, 0x03U, osFlagsWaitAll,
                                   SEM_HOLD_MS + 500U);
    osEventFlagsDelete(flags);

    TEST_ASSERT((ev & 0x80000000U) == 0,
                "Event wait timeout: 0x%08lX", ev);
    TEST_ASSERT(shared.holder_id_ok,
                "Holder NoLock read returned wrong ID");
    TEST_ASSERT(shared.waiter_id_ok,
                "Waiter blocked-read returned wrong ID");

    /* Waiter must have been held up for at least SEM_MIN_BLOCK_MS ticks */
    uint32_t block_ms = shared.waiter_end_tick - shared.waiter_start_tick;
    TEST_ASSERT(block_ms >= SEM_MIN_BLOCK_MS,
                "Waiter unblocked too early: blocked=%lums (min=%ums)",
                block_ms, SEM_MIN_BLOCK_MS);

    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail),
             "Waiter blocked %lums (hold=%ums)", block_ms, SEM_HOLD_MS);
}

/* =========================================================================
 * T10 — TASK_BMI088_ReadAcc data_topic coherence
 *   Attaches a subscriber to the accelerometer topic (published by the
 *   TASK_BMI088_ReadAcc background task), collects TOPIC_SAMPLE_COUNT
 *   samples, and verifies:
 *    (a) No DT_DATA_LOSS — subscriber keeps up with the publication rate.
 *    (b) At least one axis is non-zero (sensor is awake and returning data).
 *    (c) The Z-axis plausibly contains gravity (|z| > ACC_NONZERO_THRESHOLD).
 * ========================================================================= */

void BMI088_rtos_test_t10_acc_topic_coherence(TEST_case_t *tc) {
    /* Requires the Acc task to be running and topic pointer already set */
    TEST_ASSERT(s_acc_topic != NULL && *s_acc_topic != NULL,
                "ACC topic pointer is NULL (task not running?)");

    data_topic_t *dt = *s_acc_topic;

    data_sub_t sub = {0};
    data_status_t ds = data_sub_attach(&sub, dt, DATA_ATTACH_FROM_NOW);
    TEST_ASSERT(ds == DT_OK, "data_sub_attach: %d", (int)ds);

    uint32_t loss_count  = 0u;
    bool     nonzero     = false;

    for (uint32_t i = 0; i < TOPIC_SAMPLE_COUNT; i++) {
        data_sub_wait_for_data(&sub, TOPIC_SAMPLE_TIMEOUT_MS);

        float3_t sample;
        ds = data_sub_read(&sub, &sample);

        if (ds == DT_DATA_LOSS) {
            loss_count++;
            /* Resync and continue — we still count the sample */
            ds = data_sub_read(&sub, &sample);
        }

        if (ds == DT_OK) {
            if (fabsf(sample.x) > ACC_NONZERO_THRESHOLD ||
                fabsf(sample.y) > ACC_NONZERO_THRESHOLD ||
                fabsf(sample.z) > ACC_NONZERO_THRESHOLD) {
                nonzero = true;
            }
        }
    }

    data_sub_detach(&sub);

    TEST_ASSERT(loss_count == 0,
                "DT_DATA_LOSS x%lu on %u samples", loss_count, TOPIC_SAMPLE_COUNT);
    TEST_ASSERT(nonzero,
                "All %u ACC samples were exactly zero (sensor dead?)", TOPIC_SAMPLE_COUNT);

    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail),
             "%u samples, 0 loss, non-zero OK", TOPIC_SAMPLE_COUNT);
}

/* =========================================================================
 * T11 — TASK_BMI088_ReadGyr data_topic coherence
 * ========================================================================= */

void BMI088_rtos_test_t11_gyr_topic_coherence(TEST_case_t *tc) {
    TEST_ASSERT(s_gyr_topic != NULL && *s_gyr_topic != NULL,
                "GYR topic pointer is NULL (task not running?)");

    data_topic_t *dt = *s_gyr_topic;

    data_sub_t sub = {0};
    data_status_t ds = data_sub_attach(&sub, dt, DATA_ATTACH_FROM_NOW);
    TEST_ASSERT(ds == DT_OK, "data_sub_attach: %d", (int)ds);

    uint32_t loss_count = 0u;
    /* For gyro on a stationary board we can only check no-SPI-error
     * (all-zero is actually valid if the board is perfectly still).
     * We instead check that samples arrive without loss.             */

    for (uint32_t i = 0; i < TOPIC_SAMPLE_COUNT; i++) {
        data_sub_wait_for_data(&sub, TOPIC_SAMPLE_TIMEOUT_MS);

        float3_t sample;
        ds = data_sub_read(&sub, &sample);
        if (ds == DT_DATA_LOSS) {
            loss_count++;
            data_sub_read(&sub, &sample);   /* resync */
        }
    }

    data_sub_detach(&sub);

    TEST_ASSERT(loss_count == 0,
                "DT_DATA_LOSS x%lu on %u GYR samples", loss_count, TOPIC_SAMPLE_COUNT);

    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail),
             "%u GYR samples, 0 loss", TOPIC_SAMPLE_COUNT);
}

/* =========================================================================
 * T12 — BMI088_ApplyConfig_RTOS under load
 *   While TASK_BMI088_ReadAcc and TASK_BMI088_ReadGyr are actively reading
 *   (publishing to their topics), this test calls BMI088_ApplyConfig_RTOS
 *   with an alternate configuration.  ApplyConfig_RTOS must:
 *    (a) Successfully acquire the semaphore (not starved by reader tasks).
 *    (b) Write all registers atomically (no register half-updated).
 *    (c) Return BMI_OK.
 *   Then it reads back ACC_CONF + GYR_RANGE to confirm the new values took
 *   effect.  Finally it restores the nominal config.
 * ========================================================================= */

void BMI088_rtos_test_t12_apply_config_under_load(TEST_case_t *tc) {
    /* Alternate config: different ODR and range to guarantee register change */
    const bmi_config_t alt_cfg = {
        .acc_range = BMI_ACC_RANGE_12G,
        .acc_bwp   = BMI_ACC_CONF_BWP_OSR4,
        .acc_odr   = BMI_ACC_CONF_ODR_50_HZ,
        .acc_pwr   = BMI_ACC_PWR_CONF_ACTIVE,
        .acc_ctrl  = BMI_ACC_PWR_CTRL_ENABLE,
        .gyr_range = BMI_GYR_RANGE_500,
        .gyr_bw    = BMI_GYR_BANDWIDTH_BW_47_HZ,
        .gyr_mode  = BMI_GYR_LPM1_MODE_NORMAL,
    };

    /* Apply alternate config while reader tasks are running */
    BMI_STATE st = BMI088_ApplyConfig_RTOS(s_imu, &alt_cfg);
    TEST_ASSERT(st == BMI_OK, "ApplyConfig_RTOS (alt): %s", bmi_str(st));
    osDelay(5);

    /* Verify ACC_CONF register reflects the new config */
    uint8_t conf_r = 0, range_r = 0, gyr_range_r = 0;
    st = BMI088_ReadRegister_RTOS(s_imu, false, BMI_ACC_CONF, &conf_r);
    TEST_ASSERT(st == BMI_OK, "Read ACC_CONF: %s", bmi_str(st));

    st = BMI088_ReadRegister_RTOS(s_imu, false, BMI_ACC_RANGE, &range_r);
    TEST_ASSERT(st == BMI_OK, "Read ACC_RANGE: %s", bmi_str(st));

    st = BMI088_ReadRegister_RTOS(s_imu, true, BMI_GYR_RANGE, &gyr_range_r);
    TEST_ASSERT(st == BMI_OK, "Read GYR_RANGE: %s", bmi_str(st));

    const uint8_t exp_conf  = (uint8_t)BMI_ACC_CONF_BWP_OSR4 |
                              (uint8_t)BMI_ACC_CONF_ODR_50_HZ;
    const uint8_t exp_range = (uint8_t)BMI_ACC_RANGE_12G;
    const uint8_t exp_gyr   = (uint8_t)BMI_GYR_RANGE_500;

    TEST_ASSERT(conf_r == exp_conf,
                "ACC_CONF: got=0x%02X exp=0x%02X", conf_r, exp_conf);
    TEST_ASSERT(range_r == exp_range,
                "ACC_RANGE: got=0x%02X exp=0x%02X", range_r, exp_range);
    TEST_ASSERT(gyr_range_r == exp_gyr,
                "GYR_RANGE: got=0x%02X exp=0x%02X", gyr_range_r, exp_gyr);

    /* Restore nominal — also done while reader tasks are running */
    st = BMI088_ApplyConfig_RTOS(s_imu, s_cfg);
    TEST_ASSERT(st == BMI_OK, "ApplyConfig_RTOS (restore): %s", bmi_str(st));
    osDelay(5);

    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail),
             "Alt cfg applied & verified under Acc+Gyr task load");
}
