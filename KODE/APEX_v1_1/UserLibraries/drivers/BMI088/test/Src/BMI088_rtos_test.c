#include "BMI088_rtos_test.h"

/* Compile seulement en profil test RTOS avec le BMI088 : la suite definit les
   pools des taches du driver, qu'un projet mission definit lui-meme. Exclue
   quand le banc de performance (BMI088_rtos_bench.c, memes pools) est actif. */
#if (APEX_CFG_SCHED_RTOS == 1) && (APEX_CFG_PROFILE_TEST == 1) && (APEX_ENABLE_BMI088 == 1) && (APEX_CFG_BMI088_BENCH != 1)

#include "BMI088_test_common.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

// Pointer need to be set
static bmi088_t           *imu = NULL;
static const bmi_config_t *cfg = NULL;

void BMI088_rtos_test_set_context(bmi088_t *imu_ctx, const bmi_config_t *config) {
    imu = imu_ctx;
    cfg = config;
}

TEST_case_table_t BMI088_rtos_test_cases[BMI088_rtos_test_N_TESTS] = {
    /* A. Suite sequentielle transposee */
    { .case_info = { .name = "T0 Chip IDs"            } , .func = BMI088_rtos_test_t0_chip_ids                  },
    { .case_info = { .name = "T1 ACC SoftReset"       } , .func = BMI088_rtos_test_t1_acc_soft_reset            },
    { .case_info = { .name = "T2 GYR SoftReset"       } , .func = BMI088_rtos_test_t2_gyr_soft_reset            },
    { .case_info = { .name = "T3 ACC Config R/W"      } , .func = BMI088_rtos_test_t3_acc_config_rw             },
    { .case_info = { .name = "T4 GYR Config R/W"      } , .func = BMI088_rtos_test_t4_gyr_config_rw             },
    { .case_info = { .name = "T5 ACC SelfTest"        } , .func = BMI088_rtos_test_t5_acc_self_test             },
    { .case_info = { .name = "T6 GYR BIST"            } , .func = BMI088_rtos_test_t6_gyr_bist                  },
    { .case_info = { .name = "T7 ACC Temperature"     } , .func = BMI088_rtos_test_t7_acc_temperature           },
    /* B. Init et parametres */
    { .case_info = { .name = "T8 Re-init / Init Args" } , .func = BMI088_rtos_test_t8_reinit                    },
    { .case_info = { .name = "T9 Uninit Sensor"       } , .func = BMI088_rtos_test_t9_uninit                    },
    { .case_info = { .name = "T10 Invalid Args"       } , .func = BMI088_rtos_test_t10_invalid_args             },
    /* C. Semaphore du capteur */
    { .case_info = { .name = "T11 NoLock Under Lock"  } , .func = BMI088_rtos_test_t11_nolock_held              },
    { .case_info = { .name = "T12 Lock Timeout"       } , .func = BMI088_rtos_test_t12_lock_timeout             },
    /* D. Tache BMI088_Acquire */
    { .case_info = { .name = "T13 Acquire Nominal"    } , .func = BMI088_rtos_test_t13_acquire_nominal          },
    { .case_info = { .name = "T14 Acquire Invalid"    } , .func = BMI088_rtos_test_t14_acquire_invalid          },
    { .case_info = { .name = "T15 Acquire Stall"      } , .func = BMI088_rtos_test_t15_acquire_stall            },
    { .case_info = { .name = "T16 Acquire Lock Errors"} , .func = BMI088_rtos_test_t16_acquire_lock_errors      },
    { .case_info = { .name = "T17 Acquire Subset/Stop"} , .func = BMI088_rtos_test_t17_acquire_subset_stop      },
    /* E. Concurrence */
    { .case_info = { .name = "T18 Concurrent Readers" } , .func = BMI088_rtos_test_t18_concurrent_readers       },
    { .case_info = { .name = "T19 Re-init Acquiring"  } , .func = BMI088_rtos_test_t19_reinit_during_acquire    },
    { .case_info = { .name = "T20 Range Chg Acquiring"} , .func = BMI088_rtos_test_t20_range_change_during_acquire },
    { .case_info = { .name = "T21 Reset Atomic"       } , .func = BMI088_rtos_test_t21_reset_atomic             },
    /* F. Ressources */
    { .case_info = { .name = "T22 Stack Headroom"     } , .func = BMI088_rtos_test_t22_stack_headroom           },
};


/* ========================================================================
 * Parametres
 * ======================================================================== */
#define OP_TIMEOUT_MS       1000U         /* Attente max du semaphore par operation */
#define JOIN_TIMEOUT_MS     5000U
#define CLEANUP_TIMEOUT_MS  5000U
#define JOIN_BITS_MASK      0x7FFFFFFEu   /* bits 1..30 */
#define STACK_MARGIN_BYTES  128U

#define PRIO_ACQ            osPriorityAboveNormal
#define PRIO_HOLD           osPriorityHigh

/* Bits de join du thread de test (un par job simultane). */
#define BIT_SYNC            1u
#define BIT_HOLD            2u
#define BIT_ACQ             3u
#define BIT_READER0         4u            /* 4, 5, 6 */
#define BIT_RESET           7u

/* Topics de l'acquisition : capacite > nombre d'echantillons d'un cas. */
#define TOPIC_CAP           128U
#define TEMP_CAP            32U

/* Apres une re-init ou un changement de plage, l'ACC rend encore pendant un
   temps des donnees de l'ancien reglage (ou des zeros apres un reset). T19 et
   T20 mesurent ce temps ; au-dela de SETTLE_MAX_MS, le cas echoue. */
#define SETTLE_MAX_MS       100U

#define N_READERS           3u
#define READER_ITER         100u

/* Codes propres aux helpers de la suite (hors BMI_STATE). */
#define RT_SPAWN_FAIL       ((task_ret_t)-1)
#define RT_JOIN_TIMEOUT     ((task_ret_t)-2)
#define RT_NOT_SET          ((task_ret_t)-99)

/* ========================================================================
 * Etat partage – tout est statique (voir BMI088_rtos_test.h)
 * ======================================================================== */
static bmi088_t          imu_uninit;      /* T9 – jamais initialise            */

static task_ret_t        R_sync;          /* ret des helpers synchrones        */
static task_ret_t        R_hold;
static task_ret_t        R_acq;
static task_ret_t        R_reset;
static task_ret_t        R_rd[N_READERS];
static volatile bool     hold_taken;      /* le job Hold detient le semaphore  */

/* Acquisition */
static float3_ts_t       acc_store[TOPIC_CAP];
static float3_ts_t       gyr_store[TOPIC_CAP];
static float_ts_t        temp_store[TEMP_CAP];
static data_topic_t      acc_topic, gyr_topic, temp_topic;
static data_sub_t        acc_sub, gyr_sub, temp_sub;
static bool              topics_ready;

static bmi_acq_stats_t   acq_stats;
static volatile bool     acq_stop;
static task_h_t          acq_h;

static float3_ts_t       acc_rx[TOPIC_CAP];
static float3_ts_t       gyr_rx[TOPIC_CAP];
static float_ts_t        temp_rx[TEMP_CAP];

/* Remise en etat demandee par un cas, faite par le nettoyage. */
static bool              restore_cfg;
static bool              restore_selftest;

/* ========================================================================
 * Taches de test
 * ======================================================================== */

/* Hold : prend le semaphore du capteur, le garde ms millisecondes, le rend. */
typedef struct { bmi088_t *imu; uint32_t ms; } BMIT_Hold_args_t;
TASK_DECLARE(BMIT_Hold, BMIT_Hold_args_t, 640);
TASK_DEFINE(BMIT_Hold) {
    if (osSemaphoreAcquire(args->imu->sem_id, osWaitForever) != osOK) {
        return BMI_SEM_ERR;
    }
    hold_taken = true;
    (void)osDelay(args->ms);
    hold_taken = false;
    return (osSemaphoreRelease(args->imu->sem_id) == osOK) ? BMI_OK : BMI_SEM_ERR;
}

/* Reader : boucle ID / ACC / GYR / TEMP et juge chaque resultat. */
typedef struct {
    uint32_t   n_ok;
    uint32_t   bad_id;
    uint32_t   bad_val;
    uint32_t   n_err;
    BMI_STATE  first_err;
} bmit_reader_res_t;

typedef struct { bmi088_t *imu; uint32_t n; bmit_reader_res_t *res; } BMIT_Reader_args_t;
TASK_DECLARE(BMIT_Reader, BMIT_Reader_args_t, 1024);
TASK_DEFINE(BMIT_Reader) {
    bmit_reader_res_t *r = args->res;
    for (uint32_t i = 0; i < args->n; i++) {
        uint8_t a = 0, g = 0;
        float3_t v;
        float t;
        BMI_STATE st[4];

        st[0] = BMI088_ReadID_RTOS(args->imu, &a, &g, OP_TIMEOUT_MS);
        if (st[0] == BMI_OK && (a != BMI_ACC_CHIP_ID_EXP || g != BMI_GYR_CHIP_ID_EXP)) r->bad_id++;
        st[1] = BMI088_ReadAcc_RTOS(args->imu, &v, OP_TIMEOUT_MS);
        if (st[1] == BMI_OK) {
            float n = bmi_norm(&v);
            if (n < ACC_G_MIN || n > ACC_G_MAX) r->bad_val++;
        }
        st[2] = BMI088_ReadGyr_RTOS(args->imu, &v, OP_TIMEOUT_MS);
        if (st[2] == BMI_OK && bmi_norm(&v) > GYR_REST_MAX) r->bad_val++;
        st[3] = BMI088_ReadTemp_RTOS(args->imu, &t, OP_TIMEOUT_MS);
        if (st[3] == BMI_OK && (t < TEMP_MIN_C || t > TEMP_MAX_C)) r->bad_val++;

        for (int k = 0; k < 4; k++) {
            if (st[k] == BMI_OK) {
                r->n_ok++;
            } else {
                if (r->n_err++ == 0u) r->first_err = st[k];
            }
        }
    }
    return (r->n_err == 0u) ? BMI_OK : r->first_err;
}

/* Reset : BMI088_SoftReset_RTOS (version verrouillee) dans un job. */
typedef struct { bmi088_t *imu; bool is_gyr; } BMIT_Reset_args_t;
TASK_DECLARE(BMIT_Reset, BMIT_Reset_args_t, 1024);
TASK_DEFINE(BMIT_Reset) {
    return BMI088_SoftReset_RTOS(args->imu, args->is_gyr, OP_TIMEOUT_MS);
}

/* ========================================================================
 * Pools de la suite
 * ======================================================================== */
TASK_POOL(BMI088_Init,        1);
TASK_POOL(BMI088_ApplyConfig, 1);
TASK_POOL(BMI088_Acquire,     1);
TASK_POOL(BMIT_Hold,          1);
TASK_POOL(BMIT_Reader,        N_READERS);   /* T18 : 3 lecteurs */
TASK_POOL(BMIT_Reset,         1);

static const task_desc_t *const bmi_pools[] = {
    &TASK_BMI088_Init, &TASK_BMI088_ApplyConfig, &TASK_BMI088_Acquire,
    &TASK_BMIT_Hold,   &TASK_BMIT_Reader,        &TASK_BMIT_Reset,
};

/* ========================================================================
 * Utilitaires internes
 * ======================================================================== */

static const char *rstr(task_ret_t r) {
    switch (r) {
        case RT_SPAWN_FAIL:   return "SPAWN_FAIL";
        case RT_JOIN_TIMEOUT: return "JOIN_TIMEOUT";
        case RT_NOT_SET:      return "NOT_SET";
        default:              return bmi_str(BMI_RET(r));
    }
}

static uint32_t sem_count(void) {
    return osSemaphoreGetCount(imu->sem_id);
}

static uint32_t pools_busy(void) {
    uint32_t n = 0u;
    for (size_t i = 0; i < sizeof(bmi_pools) / sizeof(bmi_pools[0]); i++) {
        n += task_busy_count(bmi_pools[i]);
    }
    return n;
}

static bool wait_hold(bool taken, uint32_t ms) {
    for (uint32_t t = 0; t < ms && hold_taken != taken; t++) {
        (void)osDelay(1u);
    }
    return hold_taken == taken;
}

static task_h_t spawn_hold(uint32_t ms) {
    R_hold = RT_NOT_SET;
    hold_taken = false;
    return BMIT_Hold_spawn(&(BMIT_Hold_args_t){ .imu = imu, .ms = ms },
                           &(task_attr_t){ .priority = PRIO_HOLD, .ret = &R_hold, .join_bit = BIT_HOLD });
}

/* Helpers synchrones : un job sur le bit 1, joint avec JOIN_TIMEOUT_MS. */
#define SYNC_ATTR  (&(task_attr_t){ .priority = osPriorityNormal, .ret = &R_sync, .join_bit = BIT_SYNC })

static task_ret_t join_sync(task_h_t h) {
    if (!task_h_valid(h)) return RT_SPAWN_FAIL;
    if (task_join(h, JOIN_TIMEOUT_MS) != osOK) return RT_JOIN_TIMEOUT;
    return R_sync;
}

static task_ret_t rt_init_on(bmi088_t *dev, const bmi_config_t *c, osPriority_t prio) {
    R_sync = RT_NOT_SET;
    return join_sync(BMI088_Init_spawn(&(BMI088_Init_args_t){ .imu = dev, .config = *c, .timeout_ms = OP_TIMEOUT_MS },
                                       &(task_attr_t){ .priority = prio, .ret = &R_sync, .join_bit = BIT_SYNC }));
}

static task_ret_t rt_apply_on(bmi088_t *dev, const bmi_reg_config_t *reg, uint32_t timeout_ms) {
    R_sync = RT_NOT_SET;
    return join_sync(BMI088_ApplyConfig_spawn(&(BMI088_ApplyConfig_args_t){
        .imu = dev, .reg = *reg, .timeout_ms = timeout_ms }, SYNC_ATTR));
}

static task_ret_t rt_apply(const bmi_reg_config_t *reg) {
    return rt_apply_on(imu, reg, OP_TIMEOUT_MS);
}

static bool cfg_equal(const bmi_config_t *a, const bmi_config_t *b) {
    return a->bus.hspi        == b->bus.hspi        &&
           a->bus.cs_acc_bank == b->bus.cs_acc_bank && a->bus.cs_acc_pin == b->bus.cs_acc_pin &&
           a->bus.cs_gyr_bank == b->bus.cs_gyr_bank && a->bus.cs_gyr_pin == b->bus.cs_gyr_pin &&
           a->reg.acc_range   == b->reg.acc_range   && a->reg.acc_bwp    == b->reg.acc_bwp    &&
           a->reg.acc_odr     == b->reg.acc_odr     && a->reg.acc_pwr    == b->reg.acc_pwr    &&
           a->reg.acc_ctrl    == b->reg.acc_ctrl    && a->reg.gyr_range  == b->reg.gyr_range  &&
           a->reg.gyr_bw      == b->reg.gyr_bw      && a->reg.gyr_mode   == b->reg.gyr_mode;
}

/* ------------------------------------------------------------------------
 * Acquisition
 * ------------------------------------------------------------------------ */

static void topics_init_once(void) {
    if (topics_ready) return;
    data_topic_init(&acc_topic,  acc_store,  sizeof(float3_ts_t), TOPIC_CAP, CB_OVERWRITE_OLDEST);
    data_topic_init(&gyr_topic,  gyr_store,  sizeof(float3_ts_t), TOPIC_CAP, CB_OVERWRITE_OLDEST);
    data_topic_init(&temp_topic, temp_store, sizeof(float_ts_t),  TEMP_CAP,  CB_OVERWRITE_OLDEST);
    topics_ready = true;
}

static void subs_detach_all(void) {
    if (acc_sub.attached)  (void)data_sub_detach(&acc_sub);
    if (gyr_sub.attached)  (void)data_sub_detach(&gyr_sub);
    if (temp_sub.attached) (void)data_sub_detach(&temp_sub);
}

typedef struct {
    bmi088_t *dev;
    uint32_t  period_ms;
    bool      acc, gyr, temp;
    uint32_t  temp_div;
    uint32_t  timeout_ms;
} acq_req_t;

/* Lance BMI088_Acquire (bit BIT_ACQ) ; un abonne FROM_NOW par topic choisi. */
static bool acq_start(const acq_req_t *q) {
    topics_init_once();
    subs_detach_all();
    memset((void *)&acq_stats, 0, sizeof(acq_stats));
    acq_stop = false;
    R_acq    = RT_NOT_SET;

    if (q->acc)  (void)data_sub_attach(&acc_sub,  &acc_topic,  DATA_ATTACH_FROM_NOW);
    if (q->gyr)  (void)data_sub_attach(&gyr_sub,  &gyr_topic,  DATA_ATTACH_FROM_NOW);
    if (q->temp) (void)data_sub_attach(&temp_sub, &temp_topic, DATA_ATTACH_FROM_NOW);

    acq_h = BMI088_Acquire_spawn(&(BMI088_Acquire_args_t){
                                     .imu          = q->dev,
                                     .period_ms    = q->period_ms,
                                     .acc_topic    = q->acc  ? &acc_topic  : NULL,
                                     .gyr_topic    = q->gyr  ? &gyr_topic  : NULL,
                                     .temp_topic   = q->temp ? &temp_topic : NULL,
                                     .temp_divider = q->temp_div,
                                     .stats        = &acq_stats,
                                     .stop         = &acq_stop,
                                     .timeout_ms   = q->timeout_ms,
                                 },
                                 &(task_attr_t){ .priority = PRIO_ACQ, .ret = &R_acq, .join_bit = BIT_ACQ });
    return task_h_valid(acq_h);
}

/* Leve le drapeau d'arret et joint la tache. */
static osStatus_t acq_finish(uint32_t timeout_ms) {
    acq_stop = true;
    return task_join(acq_h, timeout_ms);
}

static uint32_t drain3(data_sub_t *sub, float3_ts_t *out, uint32_t max, bool *loss) {
    uint32_t n = 0;
    while (n < max && sub->attached) {
        data_status_t s = data_sub_read(sub, &out[n]);
        if (s == DT_EMPTY || s == DT_BAD_ARG) break;
        if (s == DT_DATA_LOSS) *loss = true;
        n++;
    }
    return n;
}

static uint32_t drain1(data_sub_t *sub, float_ts_t *out, uint32_t max, bool *loss) {
    uint32_t n = 0;
    while (n < max && sub->attached) {
        data_status_t s = data_sub_read(sub, &out[n]);
        if (s == DT_EMPTY || s == DT_BAD_ARG) break;
        if (s == DT_DATA_LOSS) *loss = true;
        n++;
    }
    return n;
}

static bool acc_plausible(const float3_t *v) {
    float n = bmi_norm(v);
    return n >= ACC_G_MIN && n <= ACC_G_MAX;
}

/* ------------------------------------------------------------------------
 * Nettoyage
 * ------------------------------------------------------------------------ */

/* Remet l'environnement a zero apres un cas, qu'il ait reussi ou non :
   arrete l'acquisition, attend la fin des jobs, restaure la configuration
   si le cas l'a demande, verifie que le semaphore du capteur est libre. */
static void bmi_cleanup(TEST_case_t *tc) {
    acq_stop = true;

    uint32_t t = 0;
    while (pools_busy() != 0u && t < CLEANUP_TIMEOUT_MS) {
        (void)osDelay(1u);
        t++;
    }
    uint32_t left = pools_busy();

    subs_detach_all();
    (void)osThreadFlagsClear(JOIN_BITS_MASK);
    (void)osThreadSetPriority(osThreadGetId(), osPriorityNormal);

    uint32_t count = sem_count();
    if (count == 0u && left == 0u) {
        (void)osSemaphoreRelease(imu->sem_id);   /* repare pour les cas suivants */
    }

    BMI_STATE rs = BMI_OK;
    if (restore_selftest) {
        rs = BMI088_WriteRegister_RTOS(imu, false, BMI_ACC_SELF_TEST, (uint8_t)BMI_ACC_SELF_TEST_OFF, OP_TIMEOUT_MS);
        (void)osDelay(ACC_ST_SETTLE_MS);
    }
    if (restore_cfg) {
        BMI_STATE r2 = BMI088_ApplyConfig_RTOS(imu, &cfg->reg, OP_TIMEOUT_MS);
        if (rs == BMI_OK) rs = r2;
        (void)osDelay(2u);
    }
    restore_selftest = false;
    restore_cfg      = false;

    if (tc->result == R_PASS) {
        if (left != 0u) {
            tc->result = R_FAIL;
            snprintf(tc->detail, sizeof(tc->detail),
                     "Nettoyage : %lu job(s) encore actifs apres %u ms",
                     (unsigned long)left, (unsigned)CLEANUP_TIMEOUT_MS);
        } else if (count != 1u) {
            tc->result = R_FAIL;
            snprintf(tc->detail, sizeof(tc->detail),
                     "Semaphore capteur non rendu en fin de cas (count=%lu)", (unsigned long)count);
        } else if (rs != BMI_OK) {
            tc->result = R_FAIL;
            snprintf(tc->detail, sizeof(tc->detail), "Restauration de la config nominale: %s", bmi_str(rs));
        }
    }
}

/* Definit un cas : le corps ecrit avec TEST_ASSERT, suivi d'un nettoyage
   systematique, meme quand une assertion a rendu la main plus tot. */
#define BMI_CASE(fn_)                                                          \
    static void fn_##_body(TEST_case_t *tc);                                   \
    void fn_(TEST_case_t *tc) {                                                \
        tc->result    = R_FAIL;                                                \
        tc->detail[0] = '\0';                                                  \
        if (imu == NULL || cfg == NULL || imu->sem_id == NULL) {               \
            snprintf(tc->detail, sizeof(tc->detail),                           \
                     "Contexte absent : init_imu / set_context non appeles");  \
            return;                                                            \
        }                                                                      \
        fn_##_body(tc);                                                        \
        bmi_cleanup(tc);                                                       \
    }                                                                          \
    static void fn_##_body(TEST_case_t *tc)

#define PASS(fmt_, ...)                                                        \
    do {                                                                       \
        snprintf(tc->detail, sizeof(tc->detail), fmt_, ##__VA_ARGS__);        \
        tc->result = R_PASS;                                                   \
    } while (0)

/* ========================================================================
 * Initialisation
 * ======================================================================== */

BMI_STATE BMI088_rtos_test_init_imu(bmi088_t *dev, const bmi_config_t *config, uint32_t timeout_ms) {
    static task_ret_t ret;
    ret = RT_NOT_SET;
    task_h_t h = BMI088_Init_spawn(&(BMI088_Init_args_t){ .imu = dev, .config = *config, .timeout_ms = timeout_ms },
                                   &(task_attr_t){ .priority = osPriorityNormal, .ret = &ret, .join_bit = BIT_SYNC });
    if (!task_h_valid(h)) return BMI_SEM_ERR;
    if (task_join(h, JOIN_TIMEOUT_MS) != osOK) return BMI_TIMEOUT;
    return BMI_RET(ret);
}


/* ======================= A. Suite sequentielle transposee =============== */

BMI_CASE(BMI088_rtos_test_t0_chip_ids) {
    uint8_t acc_id = 0, gyr_id = 0;
    BMI_STATE st = BMI088_ReadID_RTOS(imu, &acc_id, &gyr_id, OP_TIMEOUT_MS);
    TEST_ASSERT(st == BMI_OK, "ReadID: %s", bmi_str(st));
    TEST_ASSERT(acc_id == BMI_ACC_CHIP_ID_EXP && gyr_id == BMI_GYR_CHIP_ID_EXP,
                "ACC=0x%02X(exp:0x%02X) GYR=0x%02X(exp:0x%02X)",
                acc_id, BMI_ACC_CHIP_ID_EXP, gyr_id, BMI_GYR_CHIP_ID_EXP);
    PASS("ACC_ID=0x%02X GYR_ID=0x%02X", acc_id, gyr_id);
}

/* T1 / T2 : meme deroulement, un capteur ou l'autre. */
static void soft_reset_case(TEST_case_t *tc, bool is_gyr) {
    restore_cfg = true;
    const uint8_t reg = is_gyr ? BMI_GYR_CHIP_ID : BMI_ACC_CHIP_ID;
    const uint8_t exp = is_gyr ? BMI_GYR_CHIP_ID_EXP : BMI_ACC_CHIP_ID_EXP;

    BMI_STATE st = BMI088_SoftReset_RTOS(imu, is_gyr, OP_TIMEOUT_MS);
    TEST_ASSERT(st == BMI_OK, "SoftReset: %s", bmi_str(st));

    uint8_t id = 0;
    st = BMI088_ReadRegister_RTOS(imu, is_gyr, reg, &id, OP_TIMEOUT_MS);
    TEST_ASSERT(st == BMI_OK, "ReadID post-reset: %s", bmi_str(st));
    TEST_ASSERT(id == exp, "ID=0x%02X apres reset (exp:0x%02X)", id, exp);

    /* Le reset remet la config a ses valeurs par defaut : on la reapplique. */
    task_ret_t r = rt_apply(&cfg->reg);
    TEST_ASSERT(r == BMI_OK, "Tache ApplyConfig apres reset: %s", rstr(r));
    restore_cfg = false;
    PASS("ID=0x%02X apres reset, config reappliquee (tache)", id);
}

BMI_CASE(BMI088_rtos_test_t1_acc_soft_reset) {
    soft_reset_case(tc, false);
}

BMI_CASE(BMI088_rtos_test_t2_gyr_soft_reset) {
    soft_reset_case(tc, true);
}

BMI_CASE(BMI088_rtos_test_t3_acc_config_rw) {
    const bmi_reg_config_t test_cfg = bmi_test_cfg_acc(&cfg->reg);
    restore_cfg = true;

    task_ret_t r = rt_apply(&test_cfg);
    TEST_ASSERT(r == BMI_OK, "Tache ApplyConfig: %s", rstr(r));
    (void)osDelay(2u);

    uint8_t conf_r = 0, range_r = 0;
    BMI_STATE st = BMI088_ReadRegister_RTOS(imu, false, BMI_ACC_CONF, &conf_r, OP_TIMEOUT_MS);
    TEST_ASSERT(st == BMI_OK, "ReadConf: %s", bmi_str(st));
    st = BMI088_ReadRegister_RTOS(imu, false, BMI_ACC_RANGE, &range_r, OP_TIMEOUT_MS);
    TEST_ASSERT(st == BMI_OK, "ReadRange: %s", bmi_str(st));

    TEST_ASSERT(conf_r == BMI_TEST_ACC_CONF_EXP && range_r == BMI_TEST_ACC_RANGE_EXP,
                "CONF: got=0x%02X exp=0x%02X  RANGE: got=0x%02X exp=0x%02X",
                conf_r, BMI_TEST_ACC_CONF_EXP, range_r, BMI_TEST_ACC_RANGE_EXP);
    TEST_ASSERT(imu->acc_conv == BMI088_AccSensitivity(BMI_ACC_RANGE_12G), "acc_conv non mis a jour");
    PASS("ACC_CONF=0x%02X ACC_RANGE=0x%02X OK", conf_r, range_r);
}

BMI_CASE(BMI088_rtos_test_t4_gyr_config_rw) {
    const bmi_reg_config_t test_cfg = bmi_test_cfg_gyr(&cfg->reg);
    restore_cfg = true;

    BMI_STATE st = BMI088_ApplyConfig_RTOS(imu, &test_cfg, OP_TIMEOUT_MS);
    TEST_ASSERT(st == BMI_OK, "ApplyConfig_RTOS: %s", bmi_str(st));
    (void)osDelay(2u);

    uint8_t range_r = 0, bw_r = 0;
    st = BMI088_ReadRegister_RTOS(imu, true, BMI_GYR_RANGE, &range_r, OP_TIMEOUT_MS);
    TEST_ASSERT(st == BMI_OK, "ReadRange: %s", bmi_str(st));
    st = BMI088_ReadRegister_RTOS(imu, true, BMI_GYR_BANDWIDTH, &bw_r, OP_TIMEOUT_MS);
    TEST_ASSERT(st == BMI_OK, "ReadBW: %s", bmi_str(st));

    range_r &= BMI_GYR_RANGE_MASK;
    bw_r    &= BMI_GYR_BANDWIDTH_BW_MASK;
    TEST_ASSERT(range_r == BMI_TEST_GYR_RANGE_EXP && bw_r == BMI_TEST_GYR_BW_EXP,
                "RANGE: got=0x%02X exp=0x%02X  BW: got=0x%02X exp=0x%02X",
                range_r, BMI_TEST_GYR_RANGE_EXP, bw_r, BMI_TEST_GYR_BW_EXP);
    TEST_ASSERT(imu->gyr_conv == BMI088_GyrSensitivity(BMI_GYR_RANGE_500), "gyr_conv non mis a jour");
    PASS("GYR_RANGE=0x%02X GYR_BW=0x%02X OK", range_r, bw_r);
}

BMI_CASE(BMI088_rtos_test_t5_acc_self_test) {
    const bmi_reg_config_t st_cfg = bmi_test_cfg_selftest(&cfg->reg);
    restore_cfg      = true;
    restore_selftest = true;

    BMI_STATE st = BMI088_ApplyConfig_RTOS(imu, &st_cfg, OP_TIMEOUT_MS);
    TEST_ASSERT(st == BMI_OK, "ApplyConfig: %s", bmi_str(st));
    (void)osDelay(2u);

    st = BMI088_WriteRegister_RTOS(imu, false, BMI_ACC_SELF_TEST, (uint8_t)BMI_ACC_SELF_TEST_POS, OP_TIMEOUT_MS);
    TEST_ASSERT(st == BMI_OK, "WriteSTpos: %s", bmi_str(st));
    (void)osDelay(ACC_ST_SETTLE_MS);

    uint8_t raw_p[6] = {0};
    st = BMI088_ReadMultiple_RTOS(imu, false, BMI_ACC_X_LSB, raw_p, sizeof(raw_p), OP_TIMEOUT_MS);
    TEST_ASSERT(st == BMI_OK, "ReadAcc(pos): %s", bmi_str(st));

    st = BMI088_WriteRegister_RTOS(imu, false, BMI_ACC_SELF_TEST, (uint8_t)BMI_ACC_SELF_TEST_NEG, OP_TIMEOUT_MS);
    TEST_ASSERT(st == BMI_OK, "WriteSTneg: %s", bmi_str(st));
    (void)osDelay(ACC_ST_SETTLE_MS);

    uint8_t raw_n[6] = {0};
    st = BMI088_ReadMultiple_RTOS(imu, false, BMI_ACC_X_LSB, raw_n, sizeof(raw_n), OP_TIMEOUT_MS);
    TEST_ASSERT(st == BMI_OK, "ReadAcc(neg): %s", bmi_str(st));

    int32_t d[3];
    TEST_ASSERT(bmi_selftest_delta(raw_p, raw_n, d), "dX=%ld dY=%ld dZ=%ld (min=%d LSB)",
                (long)d[0], (long)d[1], (long)d[2], ACC_ST_MIN_LSB);
    PASS("dX=%ld dY=%ld dZ=%ld >= %d LSB OK", (long)d[0], (long)d[1], (long)d[2], ACC_ST_MIN_LSB);
}

BMI_CASE(BMI088_rtos_test_t6_gyr_bist) {
    BMI_STATE st = BMI088_WriteRegister_RTOS(imu, true, BMI_GYR_SELF_TEST, GYR_BIST_TRIG, OP_TIMEOUT_MS);
    TEST_ASSERT(st == BMI_OK, "TriggerBIST: %s", bmi_str(st));

    uint32_t t_start = osKernelGetTickCount();
    uint8_t bist_reg = 0;
    do {
        (void)osDelay(GYR_BIST_POLL_MS);
        st = BMI088_ReadRegister_RTOS(imu, true, BMI_GYR_SELF_TEST, &bist_reg, OP_TIMEOUT_MS);
        TEST_ASSERT(st == BMI_OK, "PollBIST: %s", bmi_str(st));
    } while (!(bist_reg & GYR_BIST_RDY) && (osKernelGetTickCount() - t_start) < GYR_BIST_TIMEOUT);

    TEST_ASSERT(bist_reg & GYR_BIST_RDY, "Timeout %u ms, reg=0x%02X", GYR_BIST_TIMEOUT, bist_reg);
    TEST_ASSERT(bmi_bist_ok(bist_reg), "rate_ok=%d bist_fail=%d (reg=0x%02X)",
                (bist_reg & GYR_BIST_OK) ? 1 : 0, (bist_reg & GYR_BIST_FAIL) ? 1 : 0, bist_reg);
    PASS("rate_ok=1 bist_fail=0 (reg=0x%02X)", bist_reg);
}

BMI_CASE(BMI088_rtos_test_t7_acc_temperature) {
    float temp_c = 0.0f;
    BMI_STATE st = BMI088_ReadTemp_RTOS(imu, &temp_c, OP_TIMEOUT_MS);
    TEST_ASSERT(st == BMI_OK, "ReadTemp: %s", bmi_str(st));
    char ts[16];
    bmi_fmt_temp(ts, sizeof(ts), temp_c);
    TEST_ASSERT(temp_c >= TEMP_MIN_C && temp_c <= TEMP_MAX_C,
                "T=%s C hors plage [%.0f, %.0f]", ts, TEMP_MIN_C, TEMP_MAX_C);
    PASS("T=%s C dans [%.0f, %.0f] OK", ts, TEMP_MIN_C, TEMP_MAX_C);
}


/* ======================= B. Init et parametres ========================== */

BMI_CASE(BMI088_rtos_test_t8_reinit) {
    const osSemaphoreId_t sem_before = imu->sem_id;
    const float acc_conv = imu->acc_conv;
    const float gyr_conv = imu->gyr_conv;

    task_ret_t r = rt_init_on(imu, cfg, osPriorityNormal);
    TEST_ASSERT(r == BMI_OK, "Re-init: %s", rstr(r));
    TEST_ASSERT(imu->sem_id == sem_before, "Semaphore recree par la re-init");
    TEST_ASSERT(cfg_equal(&imu->config, cfg), "Config en cache differente de la config demandee");
    TEST_ASSERT(imu->acc_conv == acc_conv && imu->gyr_conv == gyr_conv, "Facteurs de conversion modifies");

    /* Init rejetees : le handle ne doit pas bouger. */
    bmi_config_t bad = *cfg;
    task_ret_t r_null = rt_init_on(NULL, cfg, osPriorityNormal);
    bad.bus.cs_acc_bank = NULL;
    task_ret_t r_cs = rt_init_on(imu, &bad, osPriorityNormal);
    bad = *cfg;
    bad.bus.hspi = NULL;
    task_ret_t r_spi = rt_init_on(imu, &bad, osPriorityNormal);
    bad = *cfg;
    bad.reg.acc_odr = (bmi_acc_conf_odr_t)0x00;   /* sous ODR 12.5 Hz : hors plage */
    task_ret_t r_reg = rt_init_on(imu, &bad, osPriorityNormal);

    TEST_ASSERT(r_null == BMI_INVALID_ARG, "Init imu NULL: %s (attendu INVALID_ARG)", rstr(r_null));
    TEST_ASSERT(r_cs == BMI_INVALID_ARG, "Init cs_acc NULL: %s (attendu INVALID_ARG)", rstr(r_cs));
    TEST_ASSERT(r_spi == BMI_INVALID_ARG, "Init hspi NULL: %s (attendu INVALID_ARG)", rstr(r_spi));
    TEST_ASSERT(r_reg == BMI_INVALID_ARG, "Init ODR invalide: %s (attendu INVALID_ARG)", rstr(r_reg));
    TEST_ASSERT(cfg_equal(&imu->config, cfg), "Handle modifie par une init rejetee");

    uint8_t a = 0, g = 0;
    BMI_STATE st = BMI088_ReadID_RTOS(imu, &a, &g, OP_TIMEOUT_MS);
    TEST_ASSERT(st == BMI_OK && a == BMI_ACC_CHIP_ID_EXP && g == BMI_GYR_CHIP_ID_EXP,
                "ReadID apres: %s ACC=0x%02X GYR=0x%02X", bmi_str(st), a, g);
    PASS("Re-init OK (meme semaphore), 4 inits invalides rejetees");
}

BMI_CASE(BMI088_rtos_test_t9_uninit) {
    memset(&imu_uninit, 0, sizeof(imu_uninit));
    imu_uninit.config = imu->config;   /* un chemin fautif parlerait au vrai capteur, sans planter */

    uint8_t  a = 0, g = 0, v = 0;
    float3_t f3;
    float    t;
    BMI_STATE s_id   = BMI088_ReadID_RTOS(&imu_uninit, &a, &g, 10u);
    BMI_STATE s_rd   = BMI088_ReadRegister_RTOS(&imu_uninit, false, BMI_ACC_CHIP_ID, &v, 10u);
    BMI_STATE s_wr   = BMI088_WriteRegister_RTOS(&imu_uninit, false, BMI_ACC_RANGE, (uint8_t)cfg->reg.acc_range, 10u);
    BMI_STATE s_acc  = BMI088_ReadAcc_RTOS(&imu_uninit, &f3, 10u);
    BMI_STATE s_gyr  = BMI088_ReadGyr_RTOS(&imu_uninit, &f3, 10u);
    BMI_STATE s_tmp  = BMI088_ReadTemp_RTOS(&imu_uninit, &t, 10u);
    BMI_STATE s_app  = BMI088_ApplyConfig_RTOS(&imu_uninit, &cfg->reg, 10u);
    task_ret_t s_tap = rt_apply_on(&imu_uninit, &cfg->reg, 10u);

    TEST_ASSERT(s_id  == BMI_SEM_ERR, "ReadID: %s (attendu SEM_ERR)", bmi_str(s_id));
    TEST_ASSERT(s_rd  == BMI_SEM_ERR, "ReadRegister: %s (attendu SEM_ERR)", bmi_str(s_rd));
    TEST_ASSERT(s_wr  == BMI_SEM_ERR, "WriteRegister: %s (attendu SEM_ERR)", bmi_str(s_wr));
    TEST_ASSERT(s_acc == BMI_SEM_ERR, "ReadAcc: %s (attendu SEM_ERR)", bmi_str(s_acc));
    TEST_ASSERT(s_gyr == BMI_SEM_ERR, "ReadGyr: %s (attendu SEM_ERR)", bmi_str(s_gyr));
    TEST_ASSERT(s_tmp == BMI_SEM_ERR, "ReadTemp: %s (attendu SEM_ERR)", bmi_str(s_tmp));
    TEST_ASSERT(s_app == BMI_SEM_ERR, "ApplyConfig: %s (attendu SEM_ERR)", bmi_str(s_app));
    TEST_ASSERT(s_tap == BMI_SEM_ERR, "Tache ApplyConfig: %s (attendu SEM_ERR)", rstr(s_tap));

    /* L'acquisition sur un capteur non initialise compte ses erreurs et ne publie rien. */
    const uint32_t pub0 = topics_ready ? acc_topic.pub_seq : 0u;
    TEST_ASSERT(acq_start(&(acq_req_t){ .dev = &imu_uninit, .period_ms = 10u, .acc = true,
                                        .timeout_ms = 10u }), "Spawn Acquire refuse");
    (void)osDelay(50u);
    osStatus_t j = acq_finish(200u);
    TEST_ASSERT(j == osOK && R_acq == BMI_OK, "Acquire: join %d, ret %s", (int)j, rstr(R_acq));
    TEST_ASSERT(acq_stats.errors >= 3u && acq_stats.last_error == BMI_SEM_ERR,
                "Acquire: %lu erreurs, derniere %s", (unsigned long)acq_stats.errors, bmi_str(acq_stats.last_error));
    TEST_ASSERT(acc_topic.pub_seq == pub0, "Acquire a publie %lu echantillon(s)",
                (unsigned long)(acc_topic.pub_seq - pub0));
    TEST_ASSERT(imu_uninit.sem_id == NULL, "Semaphore cree hors Init");

    BMI_STATE st = BMI088_ReadID_RTOS(imu, &a, &g, OP_TIMEOUT_MS);
    TEST_ASSERT(st == BMI_OK && a == BMI_ACC_CHIP_ID_EXP, "ReadID vrai capteur: %s", bmi_str(st));
    PASS("Non init -> SEM_ERR (x8), Acquire: %lu erreurs, rien publie", (unsigned long)acq_stats.errors);
}

BMI_CASE(BMI088_rtos_test_t10_invalid_args) {
    uint8_t  buf[BMI_BURST_MAX + 1u];
    uint8_t  a, g;
    float3_t f3;
    uint32_t c_min = UINT32_MAX;
    bmi_reg_config_t bad = cfg->reg;
    bad.acc_range = (bmi_acc_range_t)7;
    const bmi_reg_config_t before = imu->config.reg;

#define CHK_ARG(expr_, label_)                                                         \
    do {                                                                               \
        BMI_STATE s_ = (expr_);                                                        \
        uint32_t  c_ = sem_count();                                                    \
        if (c_ < c_min) c_min = c_;                                                    \
        TEST_ASSERT(s_ == BMI_INVALID_ARG, "%s: %s (attendu INVALID_ARG)", label_, bmi_str(s_)); \
    } while (0)

    CHK_ARG(BMI088_ReadMultiple_RTOS(NULL, false, BMI_ACC_X_LSB, buf, 6u, OP_TIMEOUT_MS),  "ReadMultiple imu NULL");
    CHK_ARG(BMI088_ReadMultiple_RTOS(imu, false, BMI_ACC_X_LSB, NULL, 6u, OP_TIMEOUT_MS),  "ReadMultiple data NULL");
    CHK_ARG(BMI088_ReadMultiple_RTOS(imu, false, BMI_ACC_X_LSB, buf, 0u, OP_TIMEOUT_MS),   "ReadMultiple len 0");
    CHK_ARG(BMI088_ReadMultiple_RTOS(imu, true, BMI_GYR_CHIP_ID, buf, BMI_BURST_MAX + 1u, OP_TIMEOUT_MS), "ReadMultiple len max+1");
    CHK_ARG(BMI088_WriteRegister_RTOS(NULL, false, BMI_ACC_RANGE, 0u, OP_TIMEOUT_MS),       "WriteRegister imu NULL");
    CHK_ARG(BMI088_ReadID_RTOS(imu, NULL, &g, OP_TIMEOUT_MS),                               "ReadID acc NULL");
    CHK_ARG(BMI088_ReadID_RTOS(imu, &a, NULL, OP_TIMEOUT_MS),                               "ReadID gyr NULL");
    CHK_ARG(BMI088_ReadAcc_RTOS(imu, NULL, OP_TIMEOUT_MS),                                  "ReadAcc NULL");
    CHK_ARG(BMI088_ReadGyr_RTOS(NULL, &f3, OP_TIMEOUT_MS),                                  "ReadGyr imu NULL");
    CHK_ARG(BMI088_ReadTemp_RTOS(imu, NULL, OP_TIMEOUT_MS),                                 "ReadTemp NULL");
    CHK_ARG(BMI088_SoftReset_RTOS(NULL, true, OP_TIMEOUT_MS),                               "SoftReset imu NULL");
    CHK_ARG(BMI088_ApplyConfig_RTOS(NULL, &cfg->reg, OP_TIMEOUT_MS),                        "ApplyConfig imu NULL");
    CHK_ARG(BMI088_ApplyConfig_RTOS(imu, NULL, OP_TIMEOUT_MS),                              "ApplyConfig reg NULL");
    CHK_ARG(BMI088_ApplyConfig_RTOS(imu, &bad, OP_TIMEOUT_MS),                              "ApplyConfig range 7");
#undef CHK_ARG

    task_ret_t r = rt_apply_on(imu, &bad, OP_TIMEOUT_MS);
    TEST_ASSERT(r == BMI_INVALID_ARG, "Tache ApplyConfig range 7: %s (attendu INVALID_ARG)", rstr(r));
    TEST_ASSERT(c_min == 1u, "Semaphore pris par un appel rejete (count min=%lu)", (unsigned long)c_min);
    TEST_ASSERT(memcmp(&before, &imu->config.reg, sizeof(before)) == 0, "Config en cache modifiee");

    /* Bornes valides : burst de BMI_BURST_MAX octets (GYR 0x00.., lecture seule). */
    BMI_STATE st = BMI088_ReadMultiple_RTOS(imu, true, BMI_GYR_CHIP_ID, buf, BMI_BURST_MAX, OP_TIMEOUT_MS);
    TEST_ASSERT(st == BMI_OK && buf[0] == BMI_GYR_CHIP_ID_EXP, "Burst %u o: %s, 1er=0x%02X",
                (unsigned)BMI_BURST_MAX, bmi_str(st), buf[0]);
    PASS("15 appels invalides rejetes sans verrou, burst %u o OK", (unsigned)BMI_BURST_MAX);
}


/* ======================= C. Semaphore du capteur ======================== */

BMI_CASE(BMI088_rtos_test_t11_nolock_held) {
    osStatus_t acq = osSemaphoreAcquire(imu->sem_id, 100u);
    TEST_ASSERT(acq == osOK, "Prise du semaphore: %d", (int)acq);

    uint8_t  a = 0, g = 0, range = 0xFF, raw[6];
    float3_t v;
    BMI_STATE l0 = BMI088_ReadID_RTOS(imu, &a, &g, 0u);
    uint32_t t0 = osKernelGetTickCount();
    BMI_STATE l20 = BMI088_ReadAcc_RTOS(imu, &v, 20u);
    uint32_t el = osKernelGetTickCount() - t0;

    a = g = 0;
    BMI_STATE nl_id = BMI088_ReadID_RTOS_NoLock(imu, &a, &g);
    BMI_STATE nl_rd = BMI088_ReadRegister_RTOS_NoLock(imu, false, BMI_ACC_RANGE, &range);
    BMI_STATE nl_wr = (nl_rd == BMI_OK)
                    ? BMI088_WriteRegister_RTOS_NoLock(imu, false, BMI_ACC_RANGE, range)   /* reecrit la meme valeur */
                    : nl_rd;
    BMI_STATE nl_bu = BMI088_ReadMultiple_RTOS_NoLock(imu, true, BMI_GYR_RATE_X_LSB, raw, sizeof(raw));
    uint32_t count_held = sem_count();

    (void)osSemaphoreRelease(imu->sem_id);

    TEST_ASSERT(l0 == BMI_LOCK_TIMEOUT, "ReadID(0 ms) semaphore pris: %s (attendu LOCK_TIMEOUT)", bmi_str(l0));
    TEST_ASSERT(l20 == BMI_LOCK_TIMEOUT, "ReadAcc(20 ms) semaphore pris: %s (attendu LOCK_TIMEOUT)", bmi_str(l20));
    TEST_ASSERT(el >= 18u && el <= 40u, "ReadAcc(20 ms) a attendu %lu ms", (unsigned long)el);
    TEST_ASSERT(nl_id == BMI_OK && a == BMI_ACC_CHIP_ID_EXP && g == BMI_GYR_CHIP_ID_EXP,
                "ReadID_NoLock: %s ACC=0x%02X GYR=0x%02X", bmi_str(nl_id), a, g);
    TEST_ASSERT(nl_rd == BMI_OK && (range & BMI_ACC_RANGE_MASK) == (uint8_t)cfg->reg.acc_range,
                "ReadRegister_NoLock: %s, range=0x%02X", bmi_str(nl_rd), range);
    TEST_ASSERT(nl_wr == BMI_OK, "WriteRegister_NoLock: %s", bmi_str(nl_wr));
    TEST_ASSERT(nl_bu == BMI_OK, "ReadMultiple_NoLock: %s", bmi_str(nl_bu));
    TEST_ASSERT(count_held == 0u, "count=%lu pendant la detention (attendu 0)", (unsigned long)count_held);
    PASS("Verrouille: LOCK_TIMEOUT (0 et %lu ms), _NoLock OK", (unsigned long)el);
}

BMI_CASE(BMI088_rtos_test_t12_lock_timeout) {
    task_h_t hh = spawn_hold(200u);
    TEST_ASSERT(task_h_valid(hh), "Spawn Hold refuse");
    TEST_ASSERT(wait_hold(true, 100u), "Le job Hold n'a pas pris le semaphore");

    float3_t v = {0};
    uint32_t t0 = osKernelGetTickCount();
    BMI_STATE st_short = BMI088_ReadAcc_RTOS(imu, &v, 50u);
    uint32_t el_short = osKernelGetTickCount() - t0;

    task_ret_t ap = rt_apply_on(imu, &cfg->reg, 20u);

    t0 = osKernelGetTickCount();
    BMI_STATE st_long = BMI088_ReadAcc_RTOS(imu, &v, 1000u);
    uint32_t el_long = osKernelGetTickCount() - t0;

    osStatus_t j = task_join(hh, JOIN_TIMEOUT_MS);

    TEST_ASSERT(st_short == BMI_LOCK_TIMEOUT, "ReadAcc(50 ms): %s (attendu LOCK_TIMEOUT)", bmi_str(st_short));
    TEST_ASSERT(el_short >= 48u && el_short <= 80u, "ReadAcc(50 ms) a attendu %lu ms", (unsigned long)el_short);
    TEST_ASSERT(ap == BMI_LOCK_TIMEOUT, "Tache ApplyConfig(20 ms): %s (attendu LOCK_TIMEOUT)", rstr(ap));
    TEST_ASSERT(st_long == BMI_OK && acc_plausible(&v), "ReadAcc(1000 ms): %s, |a|=%.2f",
                bmi_str(st_long), (double)bmi_norm(&v));
    TEST_ASSERT(el_long < 200u, "ReadAcc(1000 ms) a attendu %lu ms (> duree de detention)", (unsigned long)el_long);
    TEST_ASSERT(j == osOK && R_hold == BMI_OK, "Job Hold: join %d, ret %s", (int)j, rstr(R_hold));
    PASS("LOCK_TIMEOUT apres %lu ms, OK apres %lu ms d'attente", (unsigned long)el_short, (unsigned long)el_long);
}


/* ======================= D. Tache BMI088_Acquire ======================== */

BMI_CASE(BMI088_rtos_test_t13_acquire_nominal) {
    const uint32_t period = 10u, run_ms = 500u, div = 5u;
    TEST_ASSERT(acq_start(&(acq_req_t){ .dev = imu, .period_ms = period, .acc = true, .gyr = true, .temp = true,
                                        .temp_div = div, .timeout_ms = OP_TIMEOUT_MS }), "Spawn Acquire refuse");
    (void)osDelay(run_ms);
    osStatus_t j = acq_finish(3u * period + 20u);
    TEST_ASSERT(j == osOK && R_acq == BMI_OK, "Acquire: join %d, ret %s", (int)j, rstr(R_acq));

    const uint32_t cycles = acq_stats.cycles;
    TEST_ASSERT(cycles >= run_ms / period - 2u && cycles <= run_ms / period + 2u,
                "%lu cycles en %lu ms (attendu ~%lu)", (unsigned long)cycles, (unsigned long)run_ms,
                (unsigned long)(run_ms / period));
    TEST_ASSERT(acq_stats.errors == 0u, "%lu erreurs, derniere %s", (unsigned long)acq_stats.errors,
                bmi_str(acq_stats.last_error));
    TEST_ASSERT(acq_stats.overruns == 0u, "%lu retards", (unsigned long)acq_stats.overruns);

    bool loss = false;
    uint32_t na = drain3(&acc_sub, acc_rx, TOPIC_CAP, &loss);
    uint32_t ng = drain3(&gyr_sub, gyr_rx, TOPIC_CAP, &loss);
    uint32_t nt = drain1(&temp_sub, temp_rx, TEMP_CAP, &loss);
    TEST_ASSERT(!loss, "Perte de donnees sur un topic");
    TEST_ASSERT(na == cycles && ng == cycles && nt == (cycles + div - 1u) / div,
                "Publies ACC=%lu GYR=%lu TEMP=%lu pour %lu cycles", (unsigned long)na, (unsigned long)ng,
                (unsigned long)nt, (unsigned long)cycles);

    float a_min = 1e9f, a_max = 0.0f, g_max = 0.0f;
    for (uint32_t i = 0; i < na; i++) {
        TEST_ASSERT(acc_rx[i].ts == gyr_rx[i].ts, "Cycle %lu : ts ACC %lu != GYR %lu", (unsigned long)i,
                    (unsigned long)acc_rx[i].ts, (unsigned long)gyr_rx[i].ts);
        if (i > 0u) {
            uint32_t d = acc_rx[i].ts - acc_rx[i - 1u].ts;
            TEST_ASSERT(d + 1u >= period && d <= period + 1u, "Ecart ts %lu ms au cycle %lu",
                        (unsigned long)d, (unsigned long)i);
        }
        float an = bmi_norm(&acc_rx[i].data), gn = bmi_norm(&gyr_rx[i].data);
        if (an < a_min) a_min = an;
        if (an > a_max) a_max = an;
        if (gn > g_max) g_max = gn;
    }
    uint32_t span = acc_rx[na - 1u].ts - acc_rx[0].ts;
    TEST_ASSERT(span + 2u >= (na - 1u) * period && span <= (na - 1u) * period + 2u,
                "Derive : %lu ms pour %lu periodes", (unsigned long)span, (unsigned long)(na - 1u));
    TEST_ASSERT(a_min >= ACC_G_MIN && a_max <= ACC_G_MAX, "|a| dans [%.2f, %.2f] (attendu ~%.2f)",
                (double)a_min, (double)a_max, (double)ACC_G_NOMINAL);
    TEST_ASSERT(g_max <= GYR_REST_MAX, "|w| max %.2f (carte immobile ?)", (double)g_max);
    for (uint32_t i = 0; i < nt; i++) {
        TEST_ASSERT(temp_rx[i].data >= TEMP_MIN_C && temp_rx[i].data <= TEMP_MAX_C, "T=%.1f hors plage",
                    (double)temp_rx[i].data);
        TEST_ASSERT(temp_rx[i].ts == acc_rx[i * div].ts, "TEMP %lu : ts %lu hors grille 1/%lu", (unsigned long)i,
                    (unsigned long)temp_rx[i].ts, (unsigned long)div);
    }
    PASS("%lu cycles/%lu ms, |a| %.2f..%.2f, |w|<%.2f, %lu temp", (unsigned long)cycles, (unsigned long)run_ms,
         (double)a_min, (double)a_max, (double)g_max, (unsigned long)nt);
}

BMI_CASE(BMI088_rtos_test_t14_acquire_invalid) {
    topics_init_once();
    const uint32_t pub0 = acc_topic.pub_seq;
    static const char *const label[3] = { "imu NULL", "periode 0", "aucun topic" };
    BMI088_Acquire_args_t a[3];
    for (int k = 0; k < 3; k++) {
        a[k] = (BMI088_Acquire_args_t){ .imu = imu, .period_ms = 10u, .acc_topic = &acc_topic,
                                        .stop = &acq_stop, .timeout_ms = OP_TIMEOUT_MS };
    }
    a[0].imu       = NULL;
    a[1].period_ms = 0u;
    a[2].acc_topic = NULL;

    acq_stop = false;   /* un garde-fou absent ferait tourner la boucle : le nettoyage l'arrete */
    for (int k = 0; k < 3; k++) {
        R_sync = RT_NOT_SET;
        task_h_t h = BMI088_Acquire_spawn(&a[k], SYNC_ATTR);
        TEST_ASSERT(task_h_valid(h), "Spawn %s refuse", label[k]);
        osStatus_t j = task_join(h, 100u);
        TEST_ASSERT(j == osOK, "%s : job toujours actif apres 100 ms", label[k]);
        TEST_ASSERT(R_sync == BMI_INVALID_ARG, "%s: %s (attendu INVALID_ARG)", label[k], rstr(R_sync));
    }
    TEST_ASSERT(acc_topic.pub_seq == pub0, "%lu echantillon(s) publie(s)", (unsigned long)(acc_topic.pub_seq - pub0));
    PASS("imu NULL, periode 0, aucun topic -> INVALID_ARG immediat");
}

BMI_CASE(BMI088_rtos_test_t15_acquire_stall) {
    const uint32_t stall = 30u, after = 50u;
    TEST_ASSERT(acq_start(&(acq_req_t){ .dev = imu, .period_ms = 1u, .acc = true, .gyr = true,
                                        .timeout_ms = 500u }), "Spawn Acquire refuse");
    (void)osDelay(50u);

    const uint32_t c_a = acq_stats.cycles, o_a = acq_stats.overruns, t_a = osKernelGetTickCount();
    task_h_t hh = spawn_hold(stall);
    TEST_ASSERT(task_h_valid(hh), "Spawn Hold refuse");
    osStatus_t jh = task_join(hh, JOIN_TIMEOUT_MS);
    (void)osDelay(after);
    const uint32_t c_b = acq_stats.cycles, o_b = acq_stats.overruns, t_b = osKernelGetTickCount();

    osStatus_t j = acq_finish(50u);
    TEST_ASSERT(jh == osOK && R_hold == BMI_OK, "Job Hold: join %d, ret %s", (int)jh, rstr(R_hold));
    TEST_ASSERT(j == osOK && R_acq == BMI_OK, "Acquire: join %d, ret %s", (int)j, rstr(R_acq));

    const uint32_t win = t_b - t_a, dc = c_b - c_a, dov = o_b - o_a;
    TEST_ASSERT(acq_stats.errors == 0u, "%lu erreurs (%s)", (unsigned long)acq_stats.errors,
                bmi_str(acq_stats.last_error));
    TEST_ASSERT(dov + 10u >= stall && dov <= stall + 10u, "%lu retards comptes pour %lu ms de blocage",
                (unsigned long)dov, (unsigned long)stall);
    TEST_ASSERT(dc + dov + 5u >= win && dc + dov <= win + 5u, "cycles %lu + retards %lu != fenetre %lu ms",
                (unsigned long)dc, (unsigned long)dov, (unsigned long)win);
    TEST_ASSERT(dc + stall <= win + 10u, "%lu cycles en %lu ms : rafale de rattrapage", (unsigned long)dc,
                (unsigned long)win);
    PASS("Blocage %lu ms : %lu retards, %lu cycles/%lu ms, sans rattrapage", (unsigned long)stall,
         (unsigned long)dov, (unsigned long)dc, (unsigned long)win);
}

BMI_CASE(BMI088_rtos_test_t16_acquire_lock_errors) {
    const uint32_t period = 5u, hold = 30u, after = 50u;
    TEST_ASSERT(acq_start(&(acq_req_t){ .dev = imu, .period_ms = period, .acc = true,
                                        .timeout_ms = 2u }), "Spawn Acquire refuse");
    (void)osDelay(30u);

    TEST_ASSERT(acq_stats.errors == 0u, "%lu erreurs avant le blocage", (unsigned long)acq_stats.errors);
    task_h_t hh = spawn_hold(hold);
    TEST_ASSERT(task_h_valid(hh), "Spawn Hold refuse");
    osStatus_t jh = task_join(hh, JOIN_TIMEOUT_MS);
    const uint32_t err = acq_stats.errors;
    const BMI_STATE last = acq_stats.last_error;
    const uint32_t pub1 = acc_topic.pub_seq;
    (void)osDelay(after);
    const uint32_t pub2 = acc_topic.pub_seq;
    const uint32_t err2 = acq_stats.errors;

    osStatus_t j = acq_finish(50u);
    TEST_ASSERT(jh == osOK && R_hold == BMI_OK, "Job Hold: join %d, ret %s", (int)jh, rstr(R_hold));
    TEST_ASSERT(j == osOK && R_acq == BMI_OK, "Acquire: join %d, ret %s", (int)j, rstr(R_acq));
    TEST_ASSERT(err >= hold / period / 2u && err <= hold / period + 2u,
                "%lu erreurs pendant %lu ms de blocage (periode %lu ms)", (unsigned long)err,
                (unsigned long)hold, (unsigned long)period);
    TEST_ASSERT(last == BMI_LOCK_TIMEOUT, "Derniere erreur %s (attendu LOCK_TIMEOUT)", bmi_str(last));
    TEST_ASSERT(err2 <= err + 1u, "Erreurs apres liberation : %lu -> %lu", (unsigned long)err, (unsigned long)err2);
    TEST_ASSERT(pub2 - pub1 + 2u >= after / period, "Reprise : %lu publications en %lu ms",
                (unsigned long)(pub2 - pub1), (unsigned long)after);
    PASS("%lu LOCK_TIMEOUT pendant le blocage, reprise %lu pub/%lu ms", (unsigned long)err,
         (unsigned long)(pub2 - pub1), (unsigned long)after);
}

BMI_CASE(BMI088_rtos_test_t17_acquire_subset_stop) {
    const uint32_t period = 20u, run_ms = 100u;
    topics_init_once();
    const uint32_t acc0 = acc_topic.pub_seq, gyr0 = gyr_topic.pub_seq, tmp0 = temp_topic.pub_seq;

    TEST_ASSERT(acq_start(&(acq_req_t){ .dev = imu, .period_ms = period, .gyr = true,
                                        .timeout_ms = OP_TIMEOUT_MS }), "Spawn Acquire refuse");
    (void)osDelay(run_ms);

    uint32_t t0 = osKernelGetTickCount();
    osStatus_t j = acq_finish(period + 50u);
    uint32_t el = osKernelGetTickCount() - t0;

    TEST_ASSERT(j == osOK && R_acq == BMI_OK, "Acquire: join %d, ret %s", (int)j, rstr(R_acq));
    TEST_ASSERT(el <= period + 5u, "Arret en %lu ms (periode %lu ms)", (unsigned long)el, (unsigned long)period);
    TEST_ASSERT(acc_topic.pub_seq == acc0 && temp_topic.pub_seq == tmp0, "Publication sur un topic NULL");
    const uint32_t ng = gyr_topic.pub_seq - gyr0;
    TEST_ASSERT(ng >= run_ms / period && ng <= run_ms / period + 2u, "%lu publications GYR en %lu ms",
                (unsigned long)ng, (unsigned long)run_ms);
    TEST_ASSERT(acq_stats.errors == 0u && acq_stats.cycles == ng, "%lu erreurs, %lu cycles pour %lu pub",
                (unsigned long)acq_stats.errors, (unsigned long)acq_stats.cycles, (unsigned long)ng);

    bool loss = false;
    uint32_t n = drain3(&gyr_sub, gyr_rx, TOPIC_CAP, &loss);
    TEST_ASSERT(n == ng && !loss, "Lus %lu / %lu, perte %d", (unsigned long)n, (unsigned long)ng, (int)loss);
    for (uint32_t i = 0; i < n; i++) {
        TEST_ASSERT(bmi_norm(&gyr_rx[i].data) <= GYR_REST_MAX, "|w|=%.2f (carte immobile ?)",
                    (double)bmi_norm(&gyr_rx[i].data));
    }
    PASS("GYR seul : %lu pub, ACC/TEMP intacts, arret en %lu ms", (unsigned long)ng, (unsigned long)el);
}


/* ======================= E. Concurrence ================================= */

BMI_CASE(BMI088_rtos_test_t18_concurrent_readers) {
    static const osPriority_t prio[N_READERS] = { osPriorityBelowNormal, osPriorityNormal, osPriorityAboveNormal };
    static bmit_reader_res_t res[N_READERS];
    task_h_t h[N_READERS];

    memset(res, 0, sizeof(res));
    for (uint32_t i = 0; i < N_READERS; i++) {
        R_rd[i] = RT_NOT_SET;
        h[i] = BMIT_Reader_spawn(&(BMIT_Reader_args_t){ .imu = imu, .n = READER_ITER, .res = &res[i] },
                                 &(task_attr_t){ .priority = prio[i], .ret = &R_rd[i],
                                                 .join_bit = (uint8_t)(BIT_READER0 + i) });
        TEST_ASSERT(task_h_valid(h[i]), "Spawn Reader %lu refuse", (unsigned long)i);
    }
    osStatus_t j = task_join_all(h, N_READERS, JOIN_TIMEOUT_MS);
    TEST_ASSERT(j == osOK, "join_all: %d", (int)j);

    uint32_t total = 0;
    for (uint32_t i = 0; i < N_READERS; i++) {
        TEST_ASSERT(R_rd[i] == BMI_OK && res[i].n_err == 0u, "Reader %lu : %lu erreurs, 1re %s", (unsigned long)i,
                    (unsigned long)res[i].n_err, bmi_str(res[i].first_err));
        TEST_ASSERT(res[i].bad_id == 0u, "Reader %lu : %lu IDs faux", (unsigned long)i, (unsigned long)res[i].bad_id);
        TEST_ASSERT(res[i].bad_val == 0u, "Reader %lu : %lu valeurs implausibles", (unsigned long)i,
                    (unsigned long)res[i].bad_val);
        total += res[i].n_ok;
    }
    PASS("%u lecteurs x %u iter (ID/ACC/GYR/TEMP), %lu lectures exactes", (unsigned)N_READERS,
         (unsigned)READER_ITER, (unsigned long)total);
}

/* Bilan des echantillons ACC d'un segment [from, to[ (horodatage). Un
   echantillon hors norme est classe : zero (registre remis a zero par un
   reset), perime (donnee brute de l'ancienne plage : |a| x stale_scale vaut
   1 g), ou autre (corruption : toujours un echec). */
typedef struct {
    uint32_t n, bad, zero, stale, other;
    uint32_t last_bad_ts;
    uint32_t other_ts;
    float    other_norm;
} seg_t;

static void eval_segment(const float3_ts_t *s, uint32_t n, uint32_t from, uint32_t to, float stale_scale, seg_t *r) {
    memset(r, 0, sizeof(*r));
    for (uint32_t i = 0; i < n; i++) {
        const uint32_t ts = s[i].ts;
        if ((int32_t)(ts - from) < 0 || (int32_t)(ts - to) >= 0) continue;
        r->n++;
        if (acc_plausible(&s[i].data)) continue;

        const float nm = bmi_norm(&s[i].data);
        r->bad++;
        r->last_bad_ts = ts;
        if (nm < 0.01f * ACC_G_NOMINAL) {
            r->zero++;
        } else if (stale_scale > 0.0f && nm * stale_scale >= ACC_G_MIN && nm * stale_scale <= ACC_G_MAX) {
            r->stale++;
        } else if (r->other++ == 0u) {
            r->other_ts   = ts;
            r->other_norm = nm;
        }
    }
}

/* Temps de stabilisation : dernier echantillon hors norme apres la fin de la
   reconfiguration (0 si aucun). */
static uint32_t settle_ms(const seg_t *r, uint32_t t_end) {
    return (r->bad != 0u && (int32_t)(r->last_bad_ts - t_end) > 0) ? r->last_bad_ts - t_end : 0u;
}

/* Nombre d'echantillons du segment posterieurs au dernier hors norme. */
static uint32_t good_after(const float3_ts_t *s, uint32_t n, const seg_t *r, uint32_t from, uint32_t to) {
    uint32_t k = 0;
    const uint32_t after = (r->bad != 0u) ? r->last_bad_ts + 1u : from;
    for (uint32_t i = 0; i < n; i++) {
        if ((int32_t)(s[i].ts - after) >= 0 && (int32_t)(s[i].ts - to) < 0) k++;
    }
    return k;
}

BMI_CASE(BMI088_rtos_test_t19_reinit_during_acquire) {
    const uint32_t period = 4u;
    const osSemaphoreId_t sem_before = imu->sem_id;
    TEST_ASSERT(acq_start(&(acq_req_t){ .dev = imu, .period_ms = period, .acc = true, .gyr = true,
                                        .timeout_ms = 500u }), "Spawn Acquire refuse");
    (void)osDelay(30u);

    const uint32_t t_a = osKernelGetTickCount();
    task_ret_t r = rt_init_on(imu, cfg, osPriorityHigh);
    const uint32_t t_b = osKernelGetTickCount();
    (void)osDelay(SETTLE_MAX_MS + 40u);
    const uint32_t t_end = osKernelGetTickCount();

    osStatus_t j = acq_finish(50u);
    TEST_ASSERT(r == BMI_OK, "Re-init: %s", rstr(r));
    TEST_ASSERT(imu->sem_id == sem_before, "Semaphore recree par la re-init");
    TEST_ASSERT(j == osOK && R_acq == BMI_OK, "Acquire: join %d, ret %s", (int)j, rstr(R_acq));
    TEST_ASSERT(acq_stats.errors == 0u, "%lu erreurs de lecture, derniere %s", (unsigned long)acq_stats.errors,
                bmi_str(acq_stats.last_error));

    bool loss = false;
    uint32_t na = drain3(&acc_sub, acc_rx, TOPIC_CAP, &loss);
    uint32_t ng = drain3(&gyr_sub, gyr_rx, TOPIC_CAP, &loss);
    TEST_ASSERT(!loss && na == ng, "Lus ACC=%lu GYR=%lu, perte %d", (unsigned long)na, (unsigned long)ng, (int)loss);

    /* Avant la re-init : tout a 1 g. Apres : zeros (ACC qui redemarre) ou
       donnees de la meme plage, puis 1 g. */
    const uint32_t t0 = acc_rx[0].ts;
    seg_t before, after;
    eval_segment(acc_rx, na, t0, t_a - period, 0.0f, &before);
    eval_segment(acc_rx, na, t_a - period, t_end + 1u, 1.0f, &after);
    const uint32_t settle = settle_ms(&after, t_b);
    const uint32_t good   = good_after(acc_rx, na, &after, t_a - period, t_end + 1u);

    TEST_ASSERT(before.bad == 0u, "%lu/%lu ech. hors norme avant la re-init", (unsigned long)before.bad,
                (unsigned long)before.n);
    TEST_ASSERT(after.other == 0u, "ts=%lu |a|=%.2f : ni zero ni 1 g apres re-init", (unsigned long)after.other_ts,
                (double)after.other_norm);
    TEST_ASSERT(settle <= SETTLE_MAX_MS, "ACC stabilise %lu ms apres la re-init (max %u)", (unsigned long)settle,
                (unsigned)SETTLE_MAX_MS);
    TEST_ASSERT(good >= 10u, "Seulement %lu ech. valides apres stabilisation", (unsigned long)good);
    for (uint32_t i = 0; i < ng; i++) {
        if ((int32_t)(gyr_rx[i].ts - t_b) > 0) {
            TEST_ASSERT(bmi_norm(&gyr_rx[i].data) <= GYR_REST_MAX, "GYR ts=%lu |w|=%.2f apres re-init",
                        (unsigned long)gyr_rx[i].ts, (double)bmi_norm(&gyr_rx[i].data));
        }
    }
    PASS("Re-init %lu ms ; ACC stable +%lu ms (%lu zeros), 0 erreur", (unsigned long)(t_b - t_a),
         (unsigned long)settle, (unsigned long)after.zero);
}

BMI_CASE(BMI088_rtos_test_t20_range_change_during_acquire) {
    const uint32_t period = 5u;
    bmi_reg_config_t fine = cfg->reg;
    fine.acc_range = (cfg->reg.acc_range == BMI_ACC_RANGE_3G) ? BMI_ACC_RANGE_24G : BMI_ACC_RANGE_3G;
    restore_cfg = true;
    const float conv_nom = imu->acc_conv;

    TEST_ASSERT(acq_start(&(acq_req_t){ .dev = imu, .period_ms = period, .acc = true,
                                        .timeout_ms = 500u }), "Spawn Acquire refuse");
    (void)osDelay(40u);

    const uint32_t t1a = osKernelGetTickCount();
    task_ret_t r1 = rt_apply(&fine);
    const uint32_t t1b = osKernelGetTickCount();
    const float conv_fine = imu->acc_conv;
    (void)osDelay(SETTLE_MAX_MS + 40u);

    const uint32_t t2a = osKernelGetTickCount();
    task_ret_t r2 = rt_apply(&cfg->reg);
    const uint32_t t2b = osKernelGetTickCount();
    (void)osDelay(SETTLE_MAX_MS + 40u);
    const uint32_t t_end = osKernelGetTickCount();

    osStatus_t j = acq_finish(50u);
    TEST_ASSERT(r1 == BMI_OK && r2 == BMI_OK, "ApplyConfig: %s puis %s", rstr(r1), rstr(r2));
    TEST_ASSERT(conv_fine == BMI088_AccSensitivity(fine.acc_range), "acc_conv non mis a jour");
    TEST_ASSERT(j == osOK && R_acq == BMI_OK, "Acquire: join %d, ret %s", (int)j, rstr(R_acq));
    TEST_ASSERT(acq_stats.errors == 0u, "%lu erreurs, derniere %s", (unsigned long)acq_stats.errors,
                bmi_str(acq_stats.last_error));

    bool loss = false;
    uint32_t na = drain3(&acc_sub, acc_rx, TOPIC_CAP, &loss);
    TEST_ASSERT(!loss && na > 0u, "Perte de donnees (%lu lus)", (unsigned long)na);

    /* Donnee perimee = brut de l'ancienne plage x facteur de la nouvelle :
       |a| x (facteur ancien / facteur nouveau) vaut alors 1 g. */
    const uint32_t t0 = acc_rx[0].ts;
    seg_t s0, s1, s2;
    eval_segment(acc_rx, na, t0, t1a - period, 0.0f, &s0);
    eval_segment(acc_rx, na, t1a - period, t2a - period, conv_nom / conv_fine, &s1);
    eval_segment(acc_rx, na, t2a - period, t_end + 1u, conv_fine / conv_nom, &s2);
    const uint32_t st1 = settle_ms(&s1, t1b), st2 = settle_ms(&s2, t2b);
    const uint32_t g1  = good_after(acc_rx, na, &s1, t1a - period, t2a - period);
    const uint32_t g2  = good_after(acc_rx, na, &s2, t2a - period, t_end + 1u);

    TEST_ASSERT(s0.bad == 0u, "%lu/%lu ech. hors norme avant le changement", (unsigned long)s0.bad,
                (unsigned long)s0.n);
    TEST_ASSERT(s1.other == 0u && s2.other == 0u, "ts=%lu |a|=%.2f : ni 1 g ni ancienne plage (desappariement)",
                (unsigned long)(s1.other ? s1.other_ts : s2.other_ts),
                (double)(s1.other ? s1.other_norm : s2.other_norm));
    TEST_ASSERT(st1 <= SETTLE_MAX_MS && st2 <= SETTLE_MAX_MS, "Stabilisation %lu / %lu ms (max %u)",
                (unsigned long)st1, (unsigned long)st2, (unsigned)SETTLE_MAX_MS);
    TEST_ASSERT(g1 >= 5u && g2 >= 5u, "Ech. valides apres stabilisation : %lu / %lu", (unsigned long)g1,
                (unsigned long)g2);
    PASS("Stabilisation +%lu / +%lu ms, %lu+%lu ech. perimes (ancienne plage)", (unsigned long)st1,
         (unsigned long)st2, (unsigned long)s1.stale, (unsigned long)s2.stale);
}

BMI_CASE(BMI088_rtos_test_t21_reset_atomic) {
    restore_cfg = true;
    R_reset = RT_NOT_SET;

    /* Priorite haute : le job prend le semaphore avant que le thread ne lise. */
    task_h_t hr = BMIT_Reset_spawn(&(BMIT_Reset_args_t){ .imu = imu, .is_gyr = true },
                                   &(task_attr_t){ .priority = osPriorityAboveNormal, .ret = &R_reset,
                                                   .join_bit = BIT_RESET });
    TEST_ASSERT(task_h_valid(hr), "Spawn Reset refuse");

    uint32_t n = 0, n_err = 0, n_bad = 0, el_first = 0;
    BMI_STATE first_err = BMI_OK;
    while (task_running(hr) && n < 1000u) {
        uint8_t a = 0, g = 0;
        uint32_t t0 = osKernelGetTickCount();
        BMI_STATE st = BMI088_ReadID_RTOS(imu, &a, &g, 100u);
        if (n == 0u) el_first = osKernelGetTickCount() - t0;
        if (st != BMI_OK) {
            if (n_err++ == 0u) first_err = st;
        } else if (g != BMI_GYR_CHIP_ID_EXP || a != BMI_ACC_CHIP_ID_EXP) {
            n_bad++;
        }
        n++;
    }
    osStatus_t j = task_join(hr, JOIN_TIMEOUT_MS);

    TEST_ASSERT(j == osOK && R_reset == BMI_OK, "Job Reset: join %d, ret %s", (int)j, rstr(R_reset));
    TEST_ASSERT(n >= 1u, "Reset fini avant la 1re lecture : test non concluant");
    TEST_ASSERT(n_err == 0u, "%lu erreurs, 1re %s", (unsigned long)n_err, bmi_str(first_err));
    TEST_ASSERT(n_bad == 0u, "%lu/%lu IDs faux pendant le reset", (unsigned long)n_bad, (unsigned long)n);
    TEST_ASSERT(el_first + 5u >= BMI_GYR_SOFTRESET_DELAY_MS, "1re lecture servie en %lu ms (reset %u ms)",
                (unsigned long)el_first, (unsigned)BMI_GYR_SOFTRESET_DELAY_MS);
    PASS("%lu lecture(s) pendant le reset, 1re servie apres %lu ms, IDs exacts", (unsigned long)n,
         (unsigned long)el_first);
}


/* ======================= F. Ressources ================================== */

BMI_CASE(BMI088_rtos_test_t22_stack_headroom) {
    uint32_t    min_free = UINT32_MAX;
    const char *min_name = "?";
    uint8_t     min_inst = 0;
    unsigned    n_meas   = 0;

    for (size_t p = 0; p < sizeof(bmi_pools) / sizeof(bmi_pools[0]); p++) {
        const task_desc_t *d = bmi_pools[p];
        for (uint8_t i = 0; i < d->n_instances; i++) {
            if (d->slots[i].id == NULL) continue;   /* instance jamais creee */
            uint32_t free_b = task_stack_headroom(d, i);
            n_meas++;
            if (free_b < min_free) { min_free = free_b; min_name = d->name; min_inst = i; }
        }
    }

    TEST_ASSERT(n_meas > 0u, "Aucune instance creee : lancer T22 apres les autres cas");
    TEST_ASSERT(min_free >= STACK_MARGIN_BYTES, "%s[%u] : %lu B libres < %u", min_name, min_inst,
                (unsigned long)min_free, (unsigned)STACK_MARGIN_BYTES);
    PASS("Min %lu B libres (%s[%u]), %u instances", (unsigned long)min_free, min_name, min_inst, n_meas);
}

#endif /* RTOS && PROFILE_TEST && BMI088 && !BENCH */
