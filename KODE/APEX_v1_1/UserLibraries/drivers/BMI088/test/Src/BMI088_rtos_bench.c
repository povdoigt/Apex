#include "BMI088_rtos_bench.h"

/* Compile seulement pour le projet de banc : il definit les memes pools que
   BMI088_rtos_test.c, les deux ne peuvent pas etre lies ensemble. */
#if (APEX_CFG_SCHED_RTOS == 1) && (APEX_CFG_PROFILE_TEST == 1) && (APEX_ENABLE_BMI088 == 1) && (APEX_CFG_BMI088_BENCH == 1)

#include "bench.h"
#include "spi.h"

#include <stdio.h>
#include <string.h>

/* ========================================================================
 * Parametres
 * ======================================================================== */
#define N_SPL            200U       /* Echantillons des micro-mesures            */
#define N_SEQ            20U        /* Echantillons des sequences (A3)           */
#define N_RESET          5U         /* Soft resets par capteur (A3)              */
#define N_INIT           3U         /* Init completes par la tache (A3)          */
#define OP_TIMEOUT_MS    1000U      /* Attente max du semaphore capteur          */
#define JOIN_TIMEOUT_MS  5000U
#define HAL_TIMEOUT_MS   100U       /* HAL bloquant : large, -O0 compris         */
#define PRIO_JOB         osPriorityAboveNormal
#define PRIO_ACQ         osPriorityAboveNormal
#define PRIO_PROBE       osPriorityHigh
#define PRIO_SPIN        osPriorityBelowNormal
#define LOAD_REF_MS      100U       /* Duree de la mesure de reference (B1)      */
#define ACQ_RUN_MS       1000U      /* Duree d'acquisition par periode (B1)      */
#define JIT_RUN_MS       500U       /* Duree d'acquisition par periode (B2)      */
#define TEMP_DIV         10U        /* Temperature 1 periode sur 10 (B)          */

#define TOPIC_CAP        16U        /* Topics de l'acquisition (B)               */

/* Codes propres au banc (hors BMI_STATE). */
#define RT_SPAWN_FAIL    ((task_ret_t)-1)
#define RT_JOIN_TIMEOUT  ((task_ret_t)-2)
#define RT_VERIFY_FAIL   ((task_ret_t)-3)
#define RT_NOT_SET       ((task_ret_t)-99)

#define MAX_RESULTS      64U

/* ========================================================================
 * Taches propres au banc
 * ======================================================================== */
typedef struct {
    volatile uint32_t *count;
    volatile bool     *stop;
} BMIB_Spin_args_t;

/* Compte tant qu'il a le CPU : sa vitesse mesure le CPU laisse aux taches basses. */
TASK_DECLARE(BMIB_Spin, BMIB_Spin_args_t, 512);

typedef struct {
    data_topic_t      *topic;
    uint32_t          *t_wake;    /* DWT a chaque reveil sur donnee          */
    uint32_t           max;
    volatile uint32_t *n;
    volatile bool     *stop;
} BMIB_Probe_args_t;

/* Abonne de priorite haute : horodate (DWT) chaque reveil sur publication. */
TASK_DECLARE(BMIB_Probe, BMIB_Probe_args_t, 768);

TASK_DEFINE(BMIB_Spin) {
    while (!*args->stop) {
        (*args->count)++;
    }
    return 0;
}

TASK_DEFINE(BMIB_Probe) {
    static data_sub_t sub;
    float3_ts_t s;
    if (data_sub_attach(&sub, args->topic, DATA_ATTACH_FROM_NOW) != DT_OK) return RT_SPAWN_FAIL;
    while (!*args->stop) {
        if (data_sub_wait_for_data(&sub, 20u) != osOK) continue;
        const uint32_t t = BENCH_now();
        bool got = false;
        while (data_sub_read(&sub, &s) != DT_EMPTY) got = true;
        if (got && *args->n < args->max) {
            args->t_wake[(*args->n)++] = t;
        }
    }
    (void)data_sub_detach(&sub);
    return 0;
}

/* ========================================================================
 * Pools : toute la RAM des taches du banc
 * ======================================================================== */
TASK_POOL(BMI088_Init,        1);
TASK_POOL(BMI088_ApplyConfig, 1);
TASK_POOL(BMI088_Acquire,     1);
TASK_POOL(BMIB_Spin,          1);
TASK_POOL(BMIB_Probe,         1);

static const task_desc_t *const bench_pools[] = {
    &TASK_BMI088_Init, &TASK_BMI088_ApplyConfig, &TASK_BMI088_Acquire,
    &TASK_BMIB_Spin,   &TASK_BMIB_Probe,
};
#define N_POOLS (sizeof(bench_pools) / sizeof(bench_pools[0]))

/* ========================================================================
 * Etat et buffers – tous statiques
 * ======================================================================== */
static bmi088_t           *imu = NULL;
static const bmi_config_t *cfg = NULL;

static uint32_t spl_a[N_SPL];
static uint32_t spl_b[N_SPL];
static uint32_t spl_c[N_SPL];

static uint8_t  tx_buf[BMI_BURST_MAX + 2u];
static uint8_t  rx_buf[BMI_BURST_MAX + 2u];

static task_ret_t job_ret;

static BENCH_result_t results[MAX_RESULTS];
static size_t         n_results;

/* Acquisition (B) */
static float3_ts_t       acc_store[TOPIC_CAP];
static float3_ts_t       gyr_store[TOPIC_CAP];
static float_ts_t        temp_store[TOPIC_CAP];
static data_topic_t      acc_topic, gyr_topic, temp_topic;
static bmi_acq_stats_t   acq_stats;
static volatile bool     acq_stop;

static const uint32_t acq_periods[] = { 1u, 2u, 5u, 10u };
#define N_PERIODS (sizeof(acq_periods) / sizeof(acq_periods[0]))

typedef struct {
    uint32_t period_ms;
    uint32_t dur_ms;
    uint32_t cycles;
    uint32_t overruns;
    uint32_t errors;
    float    cpu_free_pct;  /* CPU laisse aux taches de priorite inferieure */
} acq_result_t;

static acq_result_t acq_results[N_PERIODS];
static size_t       n_acq;
static float        load_ref_pct;   /* reference : osDelay seul (100 %)   */

/* Premiere erreur rencontree, et leur nombre : le banc continue malgre tout. */
static struct {
    const char *where;
    int32_t     code;
    uint32_t    count;
} bench_err;

static void note_err(const char *where, int32_t code) {
    if (bench_err.count++ == 0u) {
        bench_err.where = where;
        bench_err.code  = code;
    }
}

#define CHECK(where_, expr_)                                                   \
    do {                                                                       \
        int32_t st_ = (int32_t)(expr_);                                        \
        if (st_ != 0) note_err((where_), st_);                                 \
    } while (0)

static void add(const char *group, const char *name, uint32_t bytes, uint32_t *samples, uint32_t n) {
    if (n_results < MAX_RESULTS) {
        BENCH_summarize(&results[n_results++], group, name, bytes, samples, n);
    }
}

/* ========================================================================
 * Helpers : taches du driver
 * ======================================================================== */
static task_ret_t run_init(void) {
    job_ret = RT_NOT_SET;
    task_h_t h = BMI088_Init_spawn(
        &(BMI088_Init_args_t){ .imu = imu, .config = *cfg, .timeout_ms = OP_TIMEOUT_MS },
        &(task_attr_t){ .priority = PRIO_JOB, .ret = &job_ret, .join_bit = 1u });
    if (!task_h_valid(h)) return RT_SPAWN_FAIL;
    if (task_join(h, JOIN_TIMEOUT_MS) != osOK) return RT_JOIN_TIMEOUT;
    return job_ret;
}

static task_ret_t run_apply(const bmi_reg_config_t *reg) {
    job_ret = RT_NOT_SET;
    task_h_t h = BMI088_ApplyConfig_spawn(
        &(BMI088_ApplyConfig_args_t){ .imu = imu, .reg = *reg, .timeout_ms = OP_TIMEOUT_MS },
        &(task_attr_t){ .priority = PRIO_JOB, .ret = &job_ret, .join_bit = 1u });
    if (!task_h_valid(h)) return RT_SPAWN_FAIL;
    if (task_join(h, JOIN_TIMEOUT_MS) != osOK) return RT_JOIN_TIMEOUT;
    return job_ret;
}

/* ========================================================================
 * A1 – Couche SPI1 RTOS
 *
 * CS du GYR bas, premier octet 0x80 (lecture a partir de GYR_CHIP_ID) : le
 * gyro renvoie ses registres 0x00.. en auto-increment, lecture seule, sans
 * effet quelle que soit la taille.
 * ======================================================================== */
static const uint16_t xfer_sizes[] = { 2u, 3u, 4u, 7u, 8u, BMI_BURST_MAX + 1u };
static const char *const xfer_dma_names[] = {
    "TxRx DMA 2 o (ecriture reg)", "TxRx DMA 3 o (lecture reg ACC)", "TxRx DMA 4 o (temperature)",
    "TxRx DMA 7 o (GYR XYZ)",      "TxRx DMA 8 o (ACC XYZ)",         "TxRx DMA 33 o (burst max)" };
static const char *const xfer_hal_names[] = {
    "TxRx HAL bloquant 2 o", "TxRx HAL bloquant 3 o", "TxRx HAL bloquant 4 o",
    "TxRx HAL bloquant 7 o", "TxRx HAL bloquant 8 o", "TxRx HAL bloquant 33 o" };
#define N_XFER (sizeof(xfer_sizes) / sizeof(xfer_sizes[0]))

static void bench_a1_spi(void) {
    static const char G[] = "A1 Couche SPI1 RTOS (DMA) vs HAL bloquant";
    SPI_HandleTypeDef *hspi   = imu->config.bus.hspi;
    GPIO_TypeDef      *cs_bnk = imu->config.bus.cs_gyr_bank;
    uint16_t           cs_pin = imu->config.bus.cs_gyr_pin;
    uint32_t t0, t1;

    memset(tx_buf, 0, sizeof(tx_buf));
    tx_buf[0] = (uint8_t)(BMI_GYR_CHIP_ID | BMI_READ_MASK);

    /* Le verrou du capteur est garde tout le groupe : on parle au bus en direct. */
    if (osSemaphoreAcquire(imu->sem_id, OP_TIMEOUT_MS) != osOK) {
        note_err("A1 verrou capteur", BMI_LOCK_TIMEOUT);
        return;
    }

    for (uint32_t i = 0; i < N_SPL; i++) {
        t0 = BENCH_now();
        t1 = BENCH_now();
        spl_a[i] = t1 - t0;
    }
    add(G, "DWT (mesure a vide)", 0u, spl_a, N_SPL);

    for (uint32_t i = 0; i < N_SPL; i++) {
        t0 = BENCH_now();
        CHECK("A1 Begin", SPI_Begin_DMA_RTOS(hspi, cs_bnk, cs_pin));
        CHECK("A1 End",   SPI_End_DMA_RTOS(hspi, cs_bnk, cs_pin));
        t1 = BENCH_now();
        spl_a[i] = t1 - t0;
    }
    add(G, "Begin + End (bus + CS)", 0u, spl_a, N_SPL);

    for (uint32_t k = 0; k < N_XFER; k++) {
        for (uint32_t i = 0; i < N_SPL; i++) {
            CHECK("A1 Begin", SPI_Begin_DMA_RTOS(hspi, cs_bnk, cs_pin));
            t0 = BENCH_now();
            CHECK("A1 TxRx DMA", SPI_TransmitReceive_DMA_RTOS(hspi, tx_buf, rx_buf, xfer_sizes[k]));
            t1 = BENCH_now();
            CHECK("A1 End", SPI_End_DMA_RTOS(hspi, cs_bnk, cs_pin));
            BMI088_DelayUs(BMI_IDLE_NORMAL_US);
            spl_a[i] = t1 - t0;
        }
        add(G, xfer_dma_names[k], xfer_sizes[k], spl_a, N_SPL);
        if (rx_buf[1] != BMI_GYR_CHIP_ID_VALUE) note_err("A1 GYR_CHIP_ID relu (DMA)", RT_VERIFY_FAIL);
    }

    for (uint32_t k = 0; k < N_XFER; k++) {
        for (uint32_t i = 0; i < N_SPL; i++) {
            CHECK("A1 Begin", SPI_Begin_DMA_RTOS(hspi, cs_bnk, cs_pin));
            t0 = BENCH_now();
            CHECK("A1 TxRx HAL", HAL_SPI_TransmitReceive(hspi, tx_buf, rx_buf, xfer_sizes[k], HAL_TIMEOUT_MS));
            t1 = BENCH_now();
            CHECK("A1 End", SPI_End_DMA_RTOS(hspi, cs_bnk, cs_pin));
            BMI088_DelayUs(BMI_IDLE_NORMAL_US);
            spl_a[i] = t1 - t0;
        }
        add(G, xfer_hal_names[k], xfer_sizes[k], spl_a, N_SPL);
        if (rx_buf[1] != BMI_GYR_CHIP_ID_VALUE) note_err("A1 GYR_CHIP_ID relu (HAL)", RT_VERIFY_FAIL);
    }

    for (uint32_t i = 0; i < N_SPL; i++) {
        t0 = BENCH_now();
        BMI088_DelayUs(BMI_IDLE_NORMAL_US);
        t1 = BENCH_now();
        spl_a[i] = t1 - t0;
    }
    add(G, "Temps mort tIDLE 2 us (DelayUs)", 0u, spl_a, N_SPL);

    (void)osSemaphoreRelease(imu->sem_id);
}

/* ========================================================================
 * A2 – Primitives (appel direct depuis le thread du banc)
 * ======================================================================== */
static void bench_a2_primitives(void) {
    static const char G[] = "A2 Primitives";
    uint32_t t0, t1;
    uint8_t  v, a, g;
    float3_t f3;
    float    tc;

    for (uint32_t i = 0; i < N_SPL; i++) {
        t0 = BENCH_now();
        (void)osSemaphoreAcquire(imu->sem_id, OP_TIMEOUT_MS);
        (void)osSemaphoreRelease(imu->sem_id);
        t1 = BENCH_now();
        spl_a[i] = t1 - t0;
    }
    add(G, "Verrou capteur pris + rendu", 0u, spl_a, N_SPL);

    for (uint32_t i = 0; i < N_SPL; i++) {
        t0 = BENCH_now();
        CHECK("A2 ReadRegister ACC", BMI088_ReadRegister_RTOS(imu, false, BMI_ACC_CHIP_ID, &v, OP_TIMEOUT_MS));
        t1 = BENCH_now();
        spl_a[i] = t1 - t0;
        t0 = BENCH_now();
        CHECK("A2 ReadRegister GYR", BMI088_ReadRegister_RTOS(imu, true, BMI_GYR_CHIP_ID, &v, OP_TIMEOUT_MS));
        t1 = BENCH_now();
        spl_b[i] = t1 - t0;
    }
    add(G, "ReadRegister ACC (verrou)", 0u, spl_a, N_SPL);
    add(G, "ReadRegister GYR (verrou)", 0u, spl_b, N_SPL);

    if (osSemaphoreAcquire(imu->sem_id, OP_TIMEOUT_MS) == osOK) {
        for (uint32_t i = 0; i < N_SPL; i++) {
            t0 = BENCH_now();
            CHECK("A2 ReadRegister NoLock", BMI088_ReadRegister_RTOS_NoLock(imu, false, BMI_ACC_CHIP_ID, &v));
            t1 = BENCH_now();
            spl_a[i] = t1 - t0;
        }
        (void)osSemaphoreRelease(imu->sem_id);
        add(G, "ReadRegister ACC (NoLock)", 0u, spl_a, N_SPL);
    } else {
        note_err("A2 verrou capteur", BMI_LOCK_TIMEOUT);
    }

    /* Reecrit la plage en vigueur : aucun effet sur le capteur. */
    const uint8_t range = (uint8_t)imu->config.reg.acc_range;
    for (uint32_t i = 0; i < N_SPL; i++) {
        t0 = BENCH_now();
        CHECK("A2 WriteRegister", BMI088_WriteRegister_RTOS(imu, false, BMI_ACC_RANGE, range, OP_TIMEOUT_MS));
        t1 = BENCH_now();
        spl_a[i] = t1 - t0;
    }
    add(G, "WriteRegister ACC_RANGE", 0u, spl_a, N_SPL);

    for (uint32_t i = 0; i < N_SPL; i++) {
        t0 = BENCH_now();
        CHECK("A2 ReadID", BMI088_ReadID_RTOS(imu, &a, &g, OP_TIMEOUT_MS));
        t1 = BENCH_now();
        spl_a[i] = t1 - t0;
    }
    add(G, "ReadID (2 transactions)", 0u, spl_a, N_SPL);
    if (a != BMI_ACC_CHIP_ID_VALUE || g != BMI_GYR_CHIP_ID_VALUE) note_err("A2 IDs", RT_VERIFY_FAIL);

    for (uint32_t i = 0; i < N_SPL; i++) {
        t0 = BENCH_now();
        CHECK("A2 ReadAcc", BMI088_ReadAcc_RTOS(imu, &f3, OP_TIMEOUT_MS));
        t1 = BENCH_now();
        spl_a[i] = t1 - t0;
        t0 = BENCH_now();
        CHECK("A2 ReadGyr", BMI088_ReadGyr_RTOS(imu, &f3, OP_TIMEOUT_MS));
        t1 = BENCH_now();
        spl_b[i] = t1 - t0;
        t0 = BENCH_now();
        CHECK("A2 ReadTemp", BMI088_ReadTemp_RTOS(imu, &tc, OP_TIMEOUT_MS));
        t1 = BENCH_now();
        spl_c[i] = t1 - t0;
    }
    add(G, "ReadAcc_RTOS", 6u, spl_a, N_SPL);
    add(G, "ReadGyr_RTOS", 6u, spl_b, N_SPL);
    add(G, "ReadTemp_RTOS", 2u, spl_c, N_SPL);

    /* Part CPU du decodage (entiers -> float x facteur), sans bus. */
    for (uint32_t i = 0; i < N_SPL; i++) {
        rx_buf[0] = (uint8_t)i;
        t0 = BENCH_now();
        BMI088_DecodeXYZ(rx_buf, imu->acc_conv, &f3);
        t1 = BENCH_now();
        spl_a[i] = t1 - t0;
    }
    add(G, "DecodeXYZ seul (CPU)", 0u, spl_a, N_SPL);

    /* Couche sequentielle (HAL bloquant), verrou tenu pour ne gener personne. */
    if (osSemaphoreAcquire(imu->sem_id, OP_TIMEOUT_MS) == osOK) {
        for (uint32_t i = 0; i < N_SPL; i++) {
            t0 = BENCH_now();
            CHECK("A2 ReadAcc seq", BMI088_ReadAcc(imu, &f3));
            t1 = BENCH_now();
            spl_a[i] = t1 - t0;
            t0 = BENCH_now();
            CHECK("A2 ReadGyr seq", BMI088_ReadGyr(imu, &f3));
            t1 = BENCH_now();
            spl_b[i] = t1 - t0;
        }
        (void)osSemaphoreRelease(imu->sem_id);
        add(G, "ReadAcc sequentiel (HAL bloquant)", 6u, spl_a, N_SPL);
        add(G, "ReadGyr sequentiel (HAL bloquant)", 6u, spl_b, N_SPL);
    } else {
        note_err("A2 verrou capteur (seq)", BMI_LOCK_TIMEOUT);
    }
}

/* ========================================================================
 * A3 – Sequences multi-transactions
 * ======================================================================== */
static void bench_a3_sequences(void) {
    static const char G[] = "A3 Sequences (reset, config, init)";
    uint32_t t0;

    CHECK("A3 prewarm ApplyConfig", task_prewarm(&TASK_BMI088_ApplyConfig));
    CHECK("A3 prewarm Init",        task_prewarm(&TASK_BMI088_Init));

    /* Config deja en vigueur : aucune attente de changement de mode. */
    for (uint32_t i = 0; i < N_SEQ; i++) {
        t0 = BENCH_now();
        CHECK("A3 ApplyConfig", BMI088_ApplyConfig_RTOS(imu, &cfg->reg, OP_TIMEOUT_MS));
        spl_a[i] = BENCH_now() - t0;
    }
    add(G, "ApplyConfig_RTOS direct", 0u, spl_a, N_SEQ);

    for (uint32_t i = 0; i < N_SEQ; i++) {
        t0 = BENCH_now();
        CHECK("A3 Tache ApplyConfig", run_apply(&cfg->reg));
        spl_a[i] = BENCH_now() - t0;
    }
    add(G, "Tache ApplyConfig (spawn + join)", 0u, spl_a, N_SEQ);

    for (uint32_t i = 0; i < N_RESET; i++) {
        t0 = BENCH_now();
        CHECK("A3 SoftReset ACC", BMI088_SoftReset_RTOS(imu, false, OP_TIMEOUT_MS));
        spl_a[i] = BENCH_now() - t0;
        t0 = BENCH_now();
        CHECK("A3 SoftReset GYR", BMI088_SoftReset_RTOS(imu, true, OP_TIMEOUT_MS));
        spl_b[i] = BENCH_now() - t0;
        /* Apres reset, l'ACC est en suspend : la config repasse par les attentes 5 ms. */
        t0 = BENCH_now();
        CHECK("A3 ApplyConfig post-reset", BMI088_ApplyConfig_RTOS(imu, &cfg->reg, OP_TIMEOUT_MS));
        spl_c[i] = BENCH_now() - t0;
    }
    add(G, "SoftReset ACC (+ retour SPI)", 0u, spl_a, N_RESET);
    add(G, "SoftReset GYR", 0u, spl_b, N_RESET);
    add(G, "ApplyConfig apres reset", 0u, spl_c, N_RESET);

    for (uint32_t i = 0; i < N_INIT; i++) {
        t0 = BENCH_now();
        CHECK("A3 Tache Init", run_init());
        spl_a[i] = BENCH_now() - t0;
    }
    add(G, "Tache Init complete", 0u, spl_a, N_INIT);
}

/* ========================================================================
 * A4 – Cycle d'acquisition sans tache
 * ======================================================================== */
static void bench_a4_cycle(void) {
    static const char G[] = "A4 Cycle d'acquisition (thread du banc)";
    uint32_t t0;
    float3_ts_t sa, sg;
    float_ts_t  st;
    data_sub_t  sub;

    data_topic_init(&acc_topic,  acc_store,  sizeof(float3_ts_t), TOPIC_CAP, CB_OVERWRITE_OLDEST);
    data_topic_init(&gyr_topic,  gyr_store,  sizeof(float3_ts_t), TOPIC_CAP, CB_OVERWRITE_OLDEST);
    data_topic_init(&temp_topic, temp_store, sizeof(float_ts_t),  TOPIC_CAP, CB_OVERWRITE_OLDEST);

    for (uint32_t i = 0; i < N_SPL; i++) {
        t0 = BENCH_now();
        CHECK("A4 ReadAcc", BMI088_ReadAcc_RTOS(imu, &sa.data, OP_TIMEOUT_MS));
        CHECK("A4 ReadGyr", BMI088_ReadGyr_RTOS(imu, &sg.data, OP_TIMEOUT_MS));
        spl_a[i] = BENCH_now() - t0;
    }
    add(G, "ReadAcc + ReadGyr", 12u, spl_a, N_SPL);

    for (uint32_t i = 0; i < N_SPL; i++) {
        t0 = BENCH_now();
        (void)data_topic_publish(&acc_topic, &sa);
        spl_a[i] = BENCH_now() - t0;
    }
    add(G, "data_topic_publish, 0 abonne", 0u, spl_a, N_SPL);

    memset(&sub, 0, sizeof(sub));
    if (data_sub_attach(&sub, &acc_topic, DATA_ATTACH_FROM_NOW) == DT_OK) {
        for (uint32_t i = 0; i < N_SPL; i++) {
            t0 = BENCH_now();
            (void)data_topic_publish(&acc_topic, &sa);
            spl_a[i] = BENCH_now() - t0;
        }
        (void)data_sub_detach(&sub);
        add(G, "data_topic_publish, 1 abonne", 0u, spl_a, N_SPL);
    }

    /* Corps d'une periode de BMI088_Acquire (ACC + GYR + publications). */
    for (uint32_t i = 0; i < N_SPL; i++) {
        t0 = BENCH_now();
        sa.ts = sg.ts = osKernelGetTickCount();
        if (BMI088_ReadAcc_RTOS(imu, &sa.data, OP_TIMEOUT_MS) == BMI_OK) (void)data_topic_publish(&acc_topic, &sa);
        if (BMI088_ReadGyr_RTOS(imu, &sg.data, OP_TIMEOUT_MS) == BMI_OK) (void)data_topic_publish(&gyr_topic, &sg);
        spl_a[i] = BENCH_now() - t0;
    }
    add(G, "Periode ACC+GYR+publication", 12u, spl_a, N_SPL);

    for (uint32_t i = 0; i < N_SPL; i++) {
        t0 = BENCH_now();
        sa.ts = sg.ts = st.ts = osKernelGetTickCount();
        if (BMI088_ReadAcc_RTOS(imu, &sa.data, OP_TIMEOUT_MS) == BMI_OK) (void)data_topic_publish(&acc_topic, &sa);
        if (BMI088_ReadGyr_RTOS(imu, &sg.data, OP_TIMEOUT_MS) == BMI_OK) (void)data_topic_publish(&gyr_topic, &sg);
        if (BMI088_ReadTemp_RTOS(imu, &st.data, OP_TIMEOUT_MS) == BMI_OK) (void)data_topic_publish(&temp_topic, &st);
        spl_a[i] = BENCH_now() - t0;
    }
    add(G, "Periode ACC+GYR+TEMP+publication", 14u, spl_a, N_SPL);
}

/* ========================================================================
 * B – Tache BMI088_Acquire en fonctionnement
 *
 * B1 : un spinner a PRIO_SPIN compte tant qu'il a le CPU. Sa vitesse pendant
 *      un osDelay du thread du banc sert de reference (100 %) ; pendant
 *      l'acquisition, le rapport donne la part de CPU qu'elle laisse.
 * B2 : un abonne a PRIO_PROBE (au-dessus de l'acquisition) horodate chaque
 *      publication ACC : l'ecart entre deux reveils donne la gigue.
 * ======================================================================== */
static volatile uint32_t spin_count;
static volatile bool     spin_stop;
static uint32_t          probe_t[N_SPL];
static volatile uint32_t probe_n;
static volatile bool     probe_stop;

static task_h_t acq_spawn(uint32_t period_ms) {
    memset((void *)&acq_stats, 0, sizeof(acq_stats));
    acq_stop = false;
    job_ret  = RT_NOT_SET;
    return BMI088_Acquire_spawn(&(BMI088_Acquire_args_t){
                                    .imu          = imu,
                                    .period_ms    = period_ms,
                                    .acc_topic    = &acc_topic,
                                    .gyr_topic    = &gyr_topic,
                                    .temp_topic   = &temp_topic,
                                    .temp_divider = TEMP_DIV,
                                    .stats        = &acq_stats,
                                    .stop         = &acq_stop,
                                    .timeout_ms   = OP_TIMEOUT_MS,
                                },
                                &(task_attr_t){ .priority = PRIO_ACQ, .ret = &job_ret, .join_bit = 3u });
}

static void acq_end(task_h_t h, const char *where) {
    acq_stop = true;
    if (task_join(h, JOIN_TIMEOUT_MS) != osOK) {
        note_err(where, RT_JOIN_TIMEOUT);
    } else {
        CHECK(where, job_ret);
    }
}

static void bench_b1_acquire_load(void) {
    static task_ret_t spin_ret;
    uint32_t c0, t0;

    spin_count = 0u;
    spin_stop  = false;
    n_acq      = 0u;

    task_h_t hs = BMIB_Spin_spawn(&(BMIB_Spin_args_t){ .count = &spin_count, .stop = &spin_stop },
                                  &(task_attr_t){ .priority = PRIO_SPIN, .ret = &spin_ret, .join_bit = 2u });
    if (!task_h_valid(hs)) {
        note_err("B1 spawn spinner", RT_SPAWN_FAIL);
        return;
    }

    c0 = spin_count; t0 = BENCH_now();
    (void)osDelay(LOAD_REF_MS);
    const float ref = (float)(spin_count - c0) / (float)(BENCH_now() - t0);
    load_ref_pct = 100.0f;

    CHECK("B1 prewarm Acquire", task_prewarm(&TASK_BMI088_Acquire));
    for (size_t k = 0; k < N_PERIODS; k++) {
        task_h_t h = acq_spawn(acq_periods[k]);
        if (!task_h_valid(h)) {
            note_err("B1 spawn Acquire", RT_SPAWN_FAIL);
            continue;
        }
        c0 = spin_count; t0 = BENCH_now();
        (void)osDelay(ACQ_RUN_MS);
        const uint32_t dc = spin_count - c0, dt = BENCH_now() - t0;
        acq_end(h, "B1 Acquire");

        acq_results[n_acq++] = (acq_result_t){
            .period_ms    = acq_periods[k],
            .dur_ms       = ACQ_RUN_MS,
            .cycles       = acq_stats.cycles,
            .overruns     = acq_stats.overruns,
            .errors       = acq_stats.errors,
            .cpu_free_pct = (ref > 0.0f && dt > 0u) ? 100.0f * ((float)dc / (float)dt) / ref : 0.0f,
        };
        if (acq_stats.errors != 0u) note_err("B1 erreurs de lecture", acq_stats.last_error);
    }

    spin_stop = true;
    if (task_join(hs, JOIN_TIMEOUT_MS) != osOK) {
        note_err("B1 join spinner", RT_JOIN_TIMEOUT);
    }
}

static const char *const jit_names[N_PERIODS] = {
    "Periode 1 ms : ecart entre reveils", "Periode 2 ms : ecart entre reveils",
    "Periode 5 ms : ecart entre reveils", "Periode 10 ms : ecart entre reveils" };

static void bench_b2_jitter(void) {
    static const char G[] = "B2 Gigue de periode (abonne prio haute)";
    static task_ret_t probe_ret;

    for (size_t k = 0; k < N_PERIODS; k++) {
        probe_n    = 0u;
        probe_stop = false;
        task_h_t hp = BMIB_Probe_spawn(&(BMIB_Probe_args_t){ .topic = &acc_topic, .t_wake = probe_t, .max = N_SPL,
                                                             .n = &probe_n, .stop = &probe_stop },
                                       &(task_attr_t){ .priority = PRIO_PROBE, .ret = &probe_ret, .join_bit = 4u });
        if (!task_h_valid(hp)) {
            note_err("B2 spawn probe", RT_SPAWN_FAIL);
            return;
        }
        task_h_t h = acq_spawn(acq_periods[k]);
        if (!task_h_valid(h)) {
            note_err("B2 spawn Acquire", RT_SPAWN_FAIL);
        } else {
            uint32_t run = acq_periods[k] * (N_SPL + 2u);
            (void)osDelay(run < JIT_RUN_MS ? run : JIT_RUN_MS);
            acq_end(h, "B2 Acquire");
        }
        probe_stop = true;
        if (task_join(hp, JOIN_TIMEOUT_MS) != osOK) note_err("B2 join probe", RT_JOIN_TIMEOUT);

        uint32_t n = 0;
        for (uint32_t i = 1; i < probe_n; i++) spl_a[n++] = probe_t[i] - probe_t[i - 1u];
        add(G, jit_names[k], 0u, spl_a, n);
    }
}

/* ========================================================================
 * API
 * ======================================================================== */
BMI_STATE BMI088_rtos_bench_init_imu(bmi088_t *dev, const bmi_config_t *config, uint32_t timeout_ms) {
    static task_ret_t ret;
    ret = RT_NOT_SET;
    task_h_t h = BMI088_Init_spawn(&(BMI088_Init_args_t){ .imu = dev, .config = *config, .timeout_ms = timeout_ms },
                                   &(task_attr_t){ .priority = osPriorityNormal, .ret = &ret, .join_bit = 1u });
    if (!task_h_valid(h)) return BMI_SEM_ERR;
    if (task_join(h, JOIN_TIMEOUT_MS) != osOK) return BMI_TIMEOUT;
    return BMI_RET(ret);
}

void BMI088_rtos_bench_run(bmi088_t *dev, const bmi_config_t *config) {
    imu       = dev;
    cfg       = config;
    n_results = 0u;
    n_acq     = 0u;
    memset(&bench_err, 0, sizeof(bench_err));

    BENCH_init();

    bench_a1_spi();
    bench_a2_primitives();
    bench_a3_sequences();
    bench_a4_cycle();
    bench_b1_acquire_load();
    bench_b2_jitter();
}

void BMI088_rtos_bench_print(void (*print)(const char *), BMI_STATE init_st) {
    char line[160];
    uint32_t sck_khz = 0u;

    if (imu != NULL) {
        const SPI_HandleTypeDef *hspi = imu->config.bus.hspi;
        /* SPI1 est le seul SPI sur APB2 de ce composant. */
        uint32_t pclk  = (hspi->Instance == SPI1) ? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq();
        uint32_t presc = 2u << (hspi->Init.BaudRatePrescaler >> SPI_CR1_BR_Pos);
        sck_khz = pclk / presc / 1000u;
    }

    print("BMI088 RTOS - banc de performance\r\n");
#ifdef DEBUG
    print("Build : Debug (-O0)\r\n");
#else
    print("Build : Release (-Os)\r\n");
#endif
    snprintf(line, sizeof(line), "SYSCLK %lu MHz, SCK %lu kHz, init capteur : %s (code %d)\r\n",
             (unsigned long)(SystemCoreClock / 1000000u), (unsigned long)sck_khz,
             (init_st == BMI_OK) ? "OK" : "ECHEC", (int)init_st);
    print(line);
    if (bench_err.count == 0u) {
        print("Erreurs : aucune\r\n");
    } else {
        snprintf(line, sizeof(line), "Erreurs : %lu, premiere : %s (code %ld)\r\n",
                 (unsigned long)bench_err.count, bench_err.where, (long)bench_err.code);
        print(line);
    }

    BENCH_print_table(results, n_results, print);

    snprintf(line, sizeof(line),
             "\r\n== B1 Tache BMI088_Acquire (ACC+GYR, TEMP 1/%u), %u ms par periode\r\n"
             "%10s %10s %10s %10s %10s %10s\r\n",
             (unsigned)TEMP_DIV, (unsigned)ACQ_RUN_MS, "periode ms", "cycles", "Hz", "retards", "erreurs",
             "CPU libre");
    print(line);
    snprintf(line, sizeof(line), "%10s %10s %10s %10s %10s %9.1f%%\r\n", "ref", "-", "-", "-", "-",
             (double)load_ref_pct);
    print(line);
    for (size_t i = 0; i < n_acq; i++) {
        const acq_result_t *r = &acq_results[i];
        snprintf(line, sizeof(line), "%10lu %10lu %10.1f %10lu %10lu %9.1f%%\r\n",
                 (unsigned long)r->period_ms, (unsigned long)r->cycles,
                 (double)(1000.0f * (float)r->cycles / (float)r->dur_ms),
                 (unsigned long)r->overruns, (unsigned long)r->errors, (double)r->cpu_free_pct);
        print(line);
    }

    snprintf(line, sizeof(line), "\r\n== C Marge de pile (instance 0)\r\n%-34s %10s %10s %10s\r\n",
             "Pool", "pile o", "libre o", "utilise o");
    print(line);
    for (size_t i = 0; i < N_POOLS; i++) {
        const task_desc_t *d = bench_pools[i];
        uint32_t free_b = task_stack_headroom(d, 0u);
        if (free_b == 0u) {
            snprintf(line, sizeof(line), "%-34s %10lu %10s %10s\r\n",
                     d->name, (unsigned long)d->stack_bytes, "-", "non cree");
        } else {
            snprintf(line, sizeof(line), "%-34s %10lu %10lu %10lu\r\n",
                     d->name, (unsigned long)d->stack_bytes, (unsigned long)free_b,
                     (unsigned long)(d->stack_bytes - free_b));
        }
        print(line);
    }

    BENCH_print_csv(results, n_results, print);
}

#endif /* APEX_CFG_SCHED_RTOS && APEX_CFG_PROFILE_TEST && APEX_ENABLE_BMI088 && APEX_CFG_BMI088_BENCH */
