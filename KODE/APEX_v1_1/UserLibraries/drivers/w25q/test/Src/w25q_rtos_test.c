#include "w25q_rtos_test.h"

/* Compile seulement en profil test RTOS avec la W25Q : la suite definit les
   pools des taches du driver, qu'un projet mission definit lui-meme. Exclue
   quand le banc de performance (w25q_rtos_bench.c, memes pools) est actif. */
#if (APEX_CFG_SCHED_RTOS == 1) && (APEX_CFG_PROFILE_TEST == 1) && (APEX_ENABLE_W25Q512 == 1) && (APEX_CFG_W25Q_BENCH != 1)

#include "w25q_test_common.h"

#include <stdio.h>
#include <string.h>

// Pointer need to be set
static W25Q_t *w25q = NULL;

void W25Q_rtos_test_set_context(W25Q_t *w25q_ctx) {
    w25q = w25q_ctx;
}

TEST_case_table_t W25Q_rtos_test_cases[W25Q_rtos_test_N_TESTS] = {
    /* A. Communication */
    { .case_info = { .name = "T0 ID Check"            } , .func = W25Q_rtos_test_t0_id_check           },
    { .case_info = { .name = "T1 Read Status"         } , .func = W25Q_rtos_test_t1_read_status        },
    /* B. Configuration */
    { .case_info = { .name = "T2 Cfg To Status"       } , .func = W25Q_rtos_test_t2_cfg_to_status      },
    { .case_info = { .name = "T3 Cfg Applied"         } , .func = W25Q_rtos_test_t3_cfg_applied        },
    { .case_info = { .name = "T4 Cfg Invalid"         } , .func = W25Q_rtos_test_t4_cfg_invalid        },
    { .case_info = { .name = "T5 Cfg Keep"            } , .func = W25Q_rtos_test_t5_cfg_keep           },
    /* C. Primitives commande */
    { .case_info = { .name = "T6 Cmd Invalid"         } , .func = W25Q_rtos_test_t6_cmd_invalid        },
    { .case_info = { .name = "T7 Write Enable Latch"  } , .func = W25Q_rtos_test_t7_wel                },
    { .case_info = { .name = "T8 Addr Mode Tracking"  } , .func = W25Q_rtos_test_t8_addr_mode_tracking },
    /* D. Effacement secteur */
    { .case_info = { .name = "T9 Erase Verify"        } , .func = W25Q_rtos_test_t9_erase_verify       },
    /* E. Lecture / ecriture */
    { .case_info = { .name = "T10 Aligned R/W"        } , .func = W25Q_rtos_test_t10_aligned_rw        },
    { .case_info = { .name = "T11 Cross-page R/W"     } , .func = W25Q_rtos_test_t11_cross_page_rw     },
    { .case_info = { .name = "T12 Cross-sector R/W"   } , .func = W25Q_rtos_test_t12_cross_sector_rw   },
    { .case_info = { .name = "T13 Unaligned R/W"      } , .func = W25Q_rtos_test_t13_unaligned_rw      },
    { .case_info = { .name = "T14 Multi-sector R/W"   } , .func = W25Q_rtos_test_t14_multi_sector_rw   },
    { .case_info = { .name = "T15 AND without Erase"  } , .func = W25Q_rtos_test_t15_and_behavior      },
    { .case_info = { .name = "T16 R/W in 3B Mode"     } , .func = W25Q_rtos_test_t16_rw_3b_mode        },
    /* F. Effacement etendu */
    { .case_info = { .name = "T17 Erase Addr Length"  } , .func = W25Q_rtos_test_t17_erase_addr_len    },
    { .case_info = { .name = "T18 Block Erase 32 KB"  } , .func = W25Q_rtos_test_t18_erase_32kb        },
    { .case_info = { .name = "T19 Block Erase 64 KB"  } , .func = W25Q_rtos_test_t19_erase_64kb        },
    { .case_info = { .name = "T20 Sector Isolation"   } , .func = W25Q_rtos_test_t20_sector_isolation  },
    { .case_info = { .name = "T21 Busy Timeout"       } , .func = W25Q_rtos_test_t21_busy_timeout      },
    /* G. Cas limites R/W */
    { .case_info = { .name = "T22 Write Size=0"       } , .func = W25Q_rtos_test_t22_write_zero_size   },
    { .case_info = { .name = "T23 Write Near End"     } , .func = W25Q_rtos_test_t23_write_end_clamp   },
    { .case_info = { .name = "T24 Read Near End"      } , .func = W25Q_rtos_test_t24_read_end_clamp    },
    { .case_info = { .name = "T25 Addr Out Of Range"  } , .func = W25Q_rtos_test_t25_addr_out_of_range },
    /* H. Reset */
    { .case_info = { .name = "T26 Soft Reset"         } , .func = W25Q_rtos_test_t26_soft_reset        },
    /* I. Semaphore de la puce */
    { .case_info = { .name = "T27 Lock Error Paths"   } , .func = W25Q_rtos_test_t27_lock_error_paths  },
    { .case_info = { .name = "T28 NoLock Under Lock"  } , .func = W25Q_rtos_test_t28_nolock_held       },
    { .case_info = { .name = "T29 Lock Timeout"       } , .func = W25Q_rtos_test_t29_lock_timeout      },
    { .case_info = { .name = "T30 Uninit Chip"        } , .func = W25Q_rtos_test_t30_uninit_chip       },
    { .case_info = { .name = "T31 Re-init Concurrent" } , .func = W25Q_rtos_test_t31_reinit_concurrent },
    /* J. Integration scheduler */
    { .case_info = { .name = "T32 Args Copy/Detached" } , .func = W25Q_rtos_test_t32_args_detached     },
    { .case_info = { .name = "T33 Join Timeout"       } , .func = W25Q_rtos_test_t33_join_timeout      },
    /* K. Concurrence */
    { .case_info = { .name = "T34 Concurrent Writes"  } , .func = W25Q_rtos_test_t34_concurrent_writes },
    { .case_info = { .name = "T35 Concurrent Erases"  } , .func = W25Q_rtos_test_t35_concurrent_erases },
    { .case_info = { .name = "T36 Read During Write"  } , .func = W25Q_rtos_test_t36_read_during_write },
    { .case_info = { .name = "T37 Wait Shares Lock"   } , .func = W25Q_rtos_test_t37_wait_shares_lock  },
    /* L. Ressources */
    { .case_info = { .name = "T38 Stack Headroom"     } , .func = W25Q_rtos_test_t38_stack_headroom    },
};


/* ========================================================================
 * Zones propres a la suite RTOS (les zones communes sont dans
 * w25q_test_common.h, sous 0x040000).
 * ======================================================================== */
#define ADDR_K1       0x040000UL  /* T34 - 4 ecrivains, 1 secteur chacun (-> 0x043FFF) */
#define ADDR_K2       0x044000UL  /* T35 - 2 effacements concurrents      (-> 0x045FFF) */
#define ADDR_K3_W     0x046000UL  /* T36 - ecriture longue, 2 secteurs    (-> 0x047FFF) */
#define ADDR_K3_R     0x048000UL  /* T36 - zone relue pendant l'ecriture               */
#define ADDR_J_ARGS   0x049000UL  /* T32 - zone visee par les arguments d'origine      */
#define ADDR_J_ARGS2  0x04A000UL  /* T32 - zone des arguments modifies : reste vierge  */
#define ADDR_I_RE     0x04B000UL  /* T31 - ecriture pendant re-init, 2 secteurs        */
#define ADDR_BUSY64   0x050000UL  /* T27/T33/T37 - bloc 64 KB efface pour occuper la puce */

#define N_WRITERS     4u
#define SIZE_K        1024U       /* T34/T36 - taille par ecrivain / lecture           */
#define K1_OFFSET     37U         /* T34 - decalage : 5 pages par ecrivain             */

/* Join d'un job unique : couvre un effacement 64 KB et plusieurs attentes de
   W25Q_TEST_TIMEOUT_MS. Nettoyage : tous les jobs d'un cas en echec. */
#define JOIN_TIMEOUT_MS     15000U
#define CLEANUP_TIMEOUT_MS  20000U
#define JOIN_BITS_MASK      0x7FFFFFFEu   /* bits 1..30 */
#define STACK_MARGIN_BYTES  128U

/* Codes propres aux helpers de la suite (hors W25Q_STATE). */
#define RT_SPAWN_FAIL       ((task_ret_t)-1)
#define RT_JOIN_TIMEOUT     ((task_ret_t)-2)
#define RT_NOT_SET          ((task_ret_t)-99)

/* ========================================================================
 * Buffers et etat partages – tous statiques (voir w25q_rtos_test.h)
 * ======================================================================== */
static uint8_t buf_a[PAGE_BYTES];
static uint8_t buf_b[PAGE_BYTES];
static uint8_t buf_isol_a[SIZE_ISOL];
static uint8_t buf_isol_b[SIZE_ISOL];
static uint8_t buf_isol_rx[SIZE_ISOL];
static uint8_t buf_large[SIZE_LARGE];     /* T14/T31/T36 – write multi-secteurs */
static uint8_t buf_large_rx[SIZE_LARGE];  /* T14/T31/T36 – read  multi-secteurs */
static uint8_t buf_mode3[SIZE_ADDR_MODE]; /* T16 – relu en mode 3B              */
static uint8_t buf_mode4[SIZE_ADDR_MODE]; /* T16 – relu apres restauration      */
static uint8_t buf_20h_pat[16];           /* T17 – motif avant 20h              */
static uint8_t buf_20h_rx[16];            /* T17 – relu apres 20h               */
static uint8_t buf_k_tx[N_WRITERS][SIZE_K];
static uint8_t buf_k_rx[SIZE_K];
static uint8_t buf_byte;                  /* T15 – octet ecrit / relu           */

static W25Q_t  w25q_uninit;               /* T30 – jamais initialisee           */

static task_ret_t        R_sync;          /* ret des helpers synchrones         */
static task_ret_t        R[N_WRITERS];    /* ret des jobs lances par les cas    */
static volatile bool     hold_taken;      /* T29/T32 – le job Hold detient le semaphore */

/* ========================================================================
 * Taches de test
 * ======================================================================== */

/* Hold : prend le semaphore de la puce, le garde ms millisecondes, le rend. */
typedef struct { W25Q_t *chip; uint32_t ms; } W25QT_Hold_args_t;
TASK_DECLARE(W25QT_Hold, W25QT_Hold_args_t, 512);
TASK_DEFINE(W25QT_Hold) {
    if (osSemaphoreAcquire(args->chip->sem_id, osWaitForever) != osOK) {
        return W25Q_SEM_ERR;
    }
    hold_taken = true;
    (void)osDelay(args->ms);
    hold_taken = false;
    return (osSemaphoreRelease(args->chip->sem_id) == osOK) ? W25Q_OK : W25Q_SEM_ERR;
}

/* Poll : W25Q_WaitForReady_RTOS (version verrouillee) dans un job. */
typedef struct { W25Q_t *chip; uint32_t timeout_ms; } W25QT_Poll_args_t;
TASK_DECLARE(W25QT_Poll, W25QT_Poll_args_t, 640);
TASK_DEFINE(W25QT_Poll) {
    return W25Q_WaitForReady_RTOS(args->chip, args->timeout_ms);
}

/* ========================================================================
 * Pools de la suite
 * ======================================================================== */
TASK_POOL(W25Q_Init,        1);
TASK_POOL(W25Q_SendCmd,     1);
TASK_POOL(W25Q_SendCmdAddr, 2);           /* T35 : 2 effacements           */
TASK_POOL(W25Q_Write,       N_WRITERS);   /* T34 : 4 ecrivains             */
TASK_POOL(W25Q_Read,        2);           /* T36 : lecture + ecriture      */
TASK_POOL(W25QT_Hold,       1);
TASK_POOL(W25QT_Poll,       1);

static const task_desc_t *const w25q_pools[] = {
    &TASK_W25Q_Init,  &TASK_W25Q_SendCmd, &TASK_W25Q_SendCmdAddr,
    &TASK_W25Q_Write, &TASK_W25Q_Read,    &TASK_W25QT_Hold, &TASK_W25QT_Poll,
};

/* ========================================================================
 * Utilitaires internes
 * ======================================================================== */

static const char *rstr(task_ret_t r) {
    switch (r) {
        case RT_SPAWN_FAIL:   return "SPAWN_FAIL";
        case RT_JOIN_TIMEOUT: return "JOIN_TIMEOUT";
        case RT_NOT_SET:      return "NOT_SET";
        default:              return state_str(W25Q_RET(r));
    }
}

static uint32_t sem_count(void) {
    return osSemaphoreGetCount(w25q->sem_id);
}

static uint32_t pools_busy(void) {
    uint32_t n = 0u;
    for (size_t i = 0; i < sizeof(w25q_pools) / sizeof(w25q_pools[0]); i++) {
        n += task_busy_count(w25q_pools[i]);
    }
    return n;
}

/* Attend la fin d'un job sans le joindre (job detache). */
static bool wait_done(task_h_t h, uint32_t ms) {
    for (uint32_t t = 0; t < ms && task_running(h); t++) {
        (void)osDelay(1u);
    }
    return !task_running(h);
}

static bool wait_hold(bool taken, uint32_t ms) {
    for (uint32_t t = 0; t < ms && hold_taken != taken; t++) {
        (void)osDelay(1u);
    }
    return hold_taken == taken;
}

/* Helpers synchrones : un job sur le bit 1, joint avec JOIN_TIMEOUT_MS. */
#define SYNC_ATTR  (&(task_attr_t){ .priority = osPriorityNormal, .ret = &R_sync, .join_bit = 1u })

static task_ret_t join_sync(task_h_t h) {
    if (!task_h_valid(h)) return RT_SPAWN_FAIL;
    if (task_join(h, JOIN_TIMEOUT_MS) != osOK) return RT_JOIN_TIMEOUT;
    return R_sync;
}

static task_ret_t rt_init(const W25Q_config_t *cfg) {
    return join_sync(W25Q_Init_spawn(&(W25Q_Init_args_t){
        .chip = w25q, .config = *cfg, .timeout_ms = W25Q_TEST_TIMEOUT_MS }, SYNC_ATTR));
}

static task_ret_t rt_write(const uint8_t *buf, uint32_t addr, uint32_t len) {
    return join_sync(W25Q_Write_spawn(&(W25Q_Write_args_t){
        .chip = w25q, .buf = buf, .addr = addr, .len = len, .timeout_ms = W25Q_TEST_TIMEOUT_MS }, SYNC_ATTR));
}

static task_ret_t rt_read_on(W25Q_t *chip, uint8_t *buf, uint32_t addr, uint32_t len, uint32_t timeout_ms) {
    return join_sync(W25Q_Read_spawn(&(W25Q_Read_args_t){
        .chip = chip, .buf = buf, .addr = addr, .len = len, .timeout_ms = timeout_ms }, SYNC_ATTR));
}

static task_ret_t rt_read(uint8_t *buf, uint32_t addr, uint32_t len) {
    return rt_read_on(w25q, buf, addr, len, W25Q_TEST_TIMEOUT_MS);
}

static task_ret_t rt_cmd(uint8_t cmd) {
    return join_sync(W25Q_SendCmd_spawn(&(W25Q_SendCmd_args_t){
        .chip = w25q, .cmd = cmd, .timeout_ms = W25Q_TEST_TIMEOUT_MS }, SYNC_ATTR));
}

static task_ret_t rt_cmd_addr(uint8_t cmd, uint32_t addr) {
    return join_sync(W25Q_SendCmdAddr_spawn(&(W25Q_SendCmdAddr_args_t){
        .chip = w25q, .cmd = cmd, .addr = addr, .timeout_ms = W25Q_TEST_TIMEOUT_MS }, SYNC_ATTR));
}

/* Relit SR1, SR2, SR3 dans w25q->status_reg. */
static W25Q_STATE read_all_status(void) {
    W25Q_STATE st = W25Q_OK;
    for (uint8_t sr = 1; sr <= 3 && st == W25Q_OK; sr++) st = W25Q_ReadStatus_RTOS(w25q, sr, W25Q_TEST_TIMEOUT_MS);
    return st;
}

static W25Q_STATE erase_sector(uint32_t addr) {
    return W25Q_SendCmdAddr_RTOS(w25q, W25Q_SECTOR_ERASE_4B, addr, W25Q_TEST_TIMEOUT_MS);
}

/* Ecrit un motif sur un secteur deja programme puis l'efface avec 20h (longueur
   d'adresse selon ADS) : retourne true si le secteur vise est bien vierge. */
static bool erase_20h_hits(uint32_t addr, task_ret_t *st_out) {
    for (uint32_t i = 0; i < sizeof(buf_20h_pat); i++) buf_20h_pat[i] = (uint8_t)(0x4D ^ i);
    *st_out = rt_write(buf_20h_pat, addr, sizeof(buf_20h_pat));
    if (*st_out == W25Q_OK) *st_out = W25Q_SendCmdAddr_RTOS(w25q, W25Q_SECTOR_ERASE, addr, W25Q_TEST_TIMEOUT_MS);
    if (*st_out == W25Q_OK) *st_out = rt_read(buf_20h_rx, addr, sizeof(buf_20h_rx));
    if (*st_out != W25Q_OK) return false;
    uint32_t first_addr; uint8_t first_got;
    return verify_uniform(buf_20h_rx, sizeof(buf_20h_rx), 0xFF, addr, &first_addr, &first_got);
}

/* Remet l'environnement a zero apres un cas, qu'il ait reussi ou non, et
   verifie qu'aucun job ne tourne et que le semaphore de la puce est libre. */
static void w25q_cleanup(TEST_case_t *tc) {
    uint32_t t = 0;
    while (pools_busy() != 0u && t < CLEANUP_TIMEOUT_MS) {
        (void)osDelay(1u);
        t++;
    }
    uint32_t left = pools_busy();

    (void)osThreadFlagsClear(JOIN_BITS_MASK);
    (void)osThreadSetPriority(osThreadGetId(), osPriorityNormal);

    uint32_t count = sem_count();
    if (count == 0u && left == 0u) {
        (void)osSemaphoreRelease(w25q->sem_id);   /* repare pour les cas suivants */
    }

    if (tc->result == R_PASS) {
        if (left != 0u) {
            tc->result = R_FAIL;
            snprintf(tc->detail, sizeof(tc->detail),
                     "Nettoyage : %lu job(s) encore actifs apres %u ms",
                     (unsigned long)left, (unsigned)CLEANUP_TIMEOUT_MS);
        } else if (count != 1u) {
            tc->result = R_FAIL;
            snprintf(tc->detail, sizeof(tc->detail),
                     "Semaphore puce non rendu en fin de cas (count=%lu)", (unsigned long)count);
        }
    }
}

/* Definit un cas : le corps ecrit avec TEST_ASSERT, suivi d'un nettoyage
   systematique, meme quand une assertion a rendu la main plus tot. */
#define W25Q_CASE(fn_)                                                         \
    static void fn_##_body(TEST_case_t *tc);                                   \
    void fn_(TEST_case_t *tc) {                                                \
        tc->result    = R_FAIL;                                                \
        tc->detail[0] = '\0';                                                  \
        if (w25q == NULL || w25q->sem_id == NULL) {                            \
            snprintf(tc->detail, sizeof(tc->detail),                           \
                     "Contexte absent : init_chip / set_context non appeles"); \
            return;                                                            \
        }                                                                      \
        fn_##_body(tc);                                                        \
        w25q_cleanup(tc);                                                      \
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

W25Q_STATE W25Q_rtos_test_init_chip(W25Q_t *chip, const W25Q_config_t *config, uint32_t timeout_ms) {
    static task_ret_t ret;
    task_h_t h = W25Q_Init_spawn(&(W25Q_Init_args_t){ .chip = chip, .config = *config, .timeout_ms = timeout_ms },
                                 &(task_attr_t){ .priority = osPriorityNormal, .ret = &ret, .join_bit = 1u });
    if (!task_h_valid(h)) return W25Q_SEM_ERR;
    if (task_join(h, JOIN_TIMEOUT_MS) != osOK) return W25Q_BUSY_TIMEOUT;
    return W25Q_RET(ret);
}


/* ======================= A. Communication ============================== */

W25Q_CASE(W25Q_rtos_test_t0_id_check) {
    uint8_t id[3] = {0};
    W25Q_STATE st = W25Q_ReadID_RTOS(w25q, id, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "ReadID: %s", rstr(st));

    uint16_t dev = (uint16_t)((id[1] << 8) | id[2]);
    TEST_ASSERT(id[0] == W25Q_MANUFACTURER_ID, "Manuf=0x%02X != 0x%02X", id[0], W25Q_MANUFACTURER_ID);
    TEST_ASSERT(dev == W25Q_V_FULL_DEVICE_ID, "Dev=0x%04X != 0x%04X", dev, W25Q_V_FULL_DEVICE_ID);

    PASS("Manuf=0x%02X DevID=0x%04X", id[0], dev);
}

W25Q_CASE(W25Q_rtos_test_t1_read_status) {
    W25Q_STATE st;
    st = W25Q_ReadStatus_RTOS(w25q, 1, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "ReadSR1: %s", rstr(st));
    st = W25Q_ReadStatus_RTOS(w25q, 2, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "ReadSR2: %s", rstr(st));
    st = W25Q_ReadStatus_RTOS(w25q, 3, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "ReadSR3: %s", rstr(st));
    TEST_ASSERT(!W25Q_STATUS_REG(w25q, W25Q_SR1_BUSY_BIT), "BUSY=1 au repos, SR1=0x%02X",
                (uint8_t)w25q->status_reg);

    uint32_t cached = w25q->status_reg;
    st = W25Q_ReadStatus_RTOS(w25q, 0, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_PARAM_ERR, "ReadStatus(0): %s (attendu PARAM_ERR)", rstr(st));
    st = W25Q_ReadStatus_RTOS(w25q, 4, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_PARAM_ERR, "ReadStatus(4): %s (attendu PARAM_ERR)", rstr(st));
    TEST_ASSERT(w25q->status_reg == cached, "status_reg modifie par un index invalide");

    PASS("SR1=0x%02X SR2=0x%02X SR3=0x%02X, index 0/4 rejetes",
         (uint8_t)(w25q->status_reg >>  0),
         (uint8_t)(w25q->status_reg >>  8),
         (uint8_t)(w25q->status_reg >> 16));
}


/* ======================= B. Configuration ============================== */

W25Q_CASE(W25Q_rtos_test_t2_cfg_to_status) {
    uint32_t mask, bits;

    W25Q_reg_config_t keep = { 0 };
    W25Q_STATE st = W25Q_ConfigToStatus(&keep, &mask, &bits);
    TEST_ASSERT(st == W25Q_OK, "Config KEEP: %s", rstr(st));
    TEST_ASSERT(mask == 0 && bits == 0, "Config KEEP: mask=0x%06lX bits=0x%06lX (attendu 0)",
                (unsigned long)mask, (unsigned long)bits);

    W25Q_reg_config_t full = {
        .sr_write             = W25Q_SR_WRITE_VOLATILE,
        .block_protect        = W25Q_CFG_BP(5),
        .top_bottom           = W25Q_CFG_TB_BOTTOM,
        .complement           = W25Q_CFG_CMP_ON,
        .quad_enable          = W25Q_CFG_QE_OFF,
        .addr_mode            = W25Q_CFG_ADS_4B,
        .power_up_addr_mode   = W25Q_CFG_ADP_4B,
        .write_protect_scheme = W25Q_CFG_WPS_STATUS_BITS,
        .drive_strength       = W25Q_CFG_DRV_50,
    };
    const uint32_t exp_mask = (0xFUL << W25Q_SR1_BP0_BIT) | (1UL << W25Q_SR1_TB_BIT)
                            | (1UL << W25Q_SR2_CMP_BIT)   | (1UL << W25Q_SR2_QE_BIT)
                            | (1UL << W25Q_SR3_ADP_BIT)   | (1UL << W25Q_SR3_WPS_BIT)
                            | (0x3UL << W25Q_SR3_DRV0_BIT);
    const uint32_t exp_bits = (5UL << W25Q_SR1_BP0_BIT)   | (1UL << W25Q_SR1_TB_BIT)
                            | (1UL << W25Q_SR2_CMP_BIT)   | (1UL << W25Q_SR3_ADP_BIT)
                            | (0x2UL << W25Q_SR3_DRV0_BIT);
    st = W25Q_ConfigToStatus(&full, &mask, &bits);
    TEST_ASSERT(st == W25Q_OK, "Config complete: %s", rstr(st));
    TEST_ASSERT(mask == exp_mask, "mask=0x%06lX != 0x%06lX", (unsigned long)mask, (unsigned long)exp_mask);
    TEST_ASSERT(bits == exp_bits, "bits=0x%06lX != 0x%06lX", (unsigned long)bits, (unsigned long)exp_bits);
    TEST_ASSERT(!(mask & (1UL << W25Q_SR3_ADS_BIT)), "ADS dans le masque d'ecriture");

    PASS("KEEP->0, complete->mask=0x%06lX bits=0x%06lX", (unsigned long)mask, (unsigned long)bits);
}

W25Q_CASE(W25Q_rtos_test_t3_cfg_applied) {
    const W25Q_config_t ref = w25q->config;
    task_ret_t st = rt_init(&ref);
    TEST_ASSERT(st == W25Q_OK, "Re-init: %s", rstr(st));

    uint32_t mask, bits;
    st = W25Q_ConfigToStatus(&ref.reg, &mask, &bits);
    TEST_ASSERT(st == W25Q_OK, "ConfigToStatus: %s", rstr(st));
    st = read_all_status();
    TEST_ASSERT(st == W25Q_OK, "ReadStatus: %s", rstr(st));
    TEST_ASSERT((w25q->status_reg & mask) == bits, "SR=0x%06lX, attendu 0x%06lX sur mask 0x%06lX",
                (unsigned long)w25q->status_reg, (unsigned long)bits, (unsigned long)mask);

    int ads = W25Q_STATUS_REG(w25q, W25Q_SR3_ADS_BIT);
    if (ref.reg.addr_mode != W25Q_CFG_ADS_KEEP) {
        int want = (ref.reg.addr_mode == W25Q_CFG_ADS_4B);
        TEST_ASSERT(ads == want, "ADS=%d != config (%d)", ads, want);
    }

    PASS("SR=0x%06lX conforme (mask 0x%06lX), ADS=%d",
         (unsigned long)w25q->status_reg, (unsigned long)mask, ads);
}

W25Q_CASE(W25Q_rtos_test_t4_cfg_invalid) {
    const W25Q_config_t ref = w25q->config;
    W25Q_config_t cfg;

    cfg = ref;
    cfg.reg.addr_mode = (W25Q_CFG_ADS)(W25Q_CFG_ADS_4B + 1);
    task_ret_t st = rt_init(&cfg);
    TEST_ASSERT(st == W25Q_PARAM_ERR, "addr_mode invalide: %s (attendu PARAM_ERR)", rstr(st));

    cfg = ref;
    cfg.reg.block_protect = (uint8_t)(W25Q_CFG_BP(15) + 1u);
    st = rt_init(&cfg);
    TEST_ASSERT(st == W25Q_PARAM_ERR, "block_protect invalide: %s (attendu PARAM_ERR)", rstr(st));

    cfg = ref;
    cfg.bus.hspi = NULL;
    st = rt_init(&cfg);
    TEST_ASSERT(st == W25Q_PARAM_ERR, "hspi NULL: %s (attendu PARAM_ERR)", rstr(st));

    /* Un rejet ne doit pas ecraser la config en place */
    TEST_ASSERT(w25q->config.bus.hspi == ref.bus.hspi && w25q->config.reg.addr_mode == ref.reg.addr_mode,
                "chip->config modifiee par un Init rejete");
    uint8_t id[3] = {0};
    st = W25Q_ReadID_RTOS(w25q, id, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "ReadID apres rejet: %s", rstr(st));
    PASS("3 configs invalides -> PARAM_ERR, config intacte");
}

W25Q_CASE(W25Q_rtos_test_t5_cfg_keep) {
    const W25Q_config_t ref = w25q->config;
    W25Q_STATE st = read_all_status();
    TEST_ASSERT(st == W25Q_OK, "ReadStatus avant: %s", rstr(st));
    uint32_t before = w25q->status_reg;

    W25Q_config_t cfg = { .bus = ref.bus };   /* reg : tout a W25Q_CFG_KEEP */
    task_ret_t st_keep = rt_init(&cfg);
    uint32_t after = w25q->status_reg;
    task_ret_t st_ref = rt_init(&ref);

    TEST_ASSERT(st_keep == W25Q_OK, "Init KEEP: %s", rstr(st_keep));
    TEST_ASSERT((before & ~SR_VOLATILE_MASK) == (after & ~SR_VOLATILE_MASK),
                "SR modifies: avant=0x%06lX apres=0x%06lX", (unsigned long)before, (unsigned long)after);
    TEST_ASSERT(st_ref == W25Q_OK, "Restauration config: %s", rstr(st_ref));
    PASS("SR inchanges (0x%06lX)", (unsigned long)after);
}


/* ======================= C. Primitives commande ======================== */

W25Q_CASE(W25Q_rtos_test_t6_cmd_invalid) {
    const uint8_t bad = 0x00;   /* absent de W25Q_CMD_FLAGS */
    W25Q_STATE st = W25Q_SendCmd_RTOS(w25q, bad, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_PARAM_ERR, "SendCmd(0x00): %s (attendu PARAM_ERR)", rstr(st));
    st = W25Q_SendCmdAddr_RTOS(w25q, bad, ADDR_SEC0, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_PARAM_ERR, "SendCmdAddr(0x00): %s (attendu PARAM_ERR)", rstr(st));
    st = W25Q_WriteStatus_RTOS(w25q, 0, 0x00, W25Q_SR_WRITE_VOLATILE, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_PARAM_ERR, "WriteStatus(0): %s (attendu PARAM_ERR)", rstr(st));
    st = W25Q_WriteStatus_RTOS(w25q, 4, 0x00, W25Q_SR_WRITE_VOLATILE, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_PARAM_ERR, "WriteStatus(4): %s (attendu PARAM_ERR)", rstr(st));

    uint8_t id[3] = {0};
    st = W25Q_ReadID_RTOS(w25q, id, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK && id[0] == W25Q_MANUFACTURER_ID,
                "ReadID apres rejets: %s, Manuf=0x%02X", rstr(st), id[0]);
    PASS("SendCmd/SendCmdAddr/WriteStatus -> PARAM_ERR");
}

W25Q_CASE(W25Q_rtos_test_t7_wel) {
    task_ret_t st = rt_cmd(W25Q_WRITE_ENABLE);
    TEST_ASSERT(st == W25Q_OK, "WRITE_ENABLE: %s", rstr(st));
    st = W25Q_ReadStatus_RTOS(w25q, 1, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "ReadSR1: %s", rstr(st));
    TEST_ASSERT(W25Q_STATUS_REG(w25q, W25Q_SR1_WEL_BIT), "WEL=0 apres 06h, SR1=0x%02X",
                (uint8_t)w25q->status_reg);

    st = rt_cmd(W25Q_WRITE_DISABLE);
    TEST_ASSERT(st == W25Q_OK, "WRITE_DISABLE: %s", rstr(st));
    st = W25Q_ReadStatus_RTOS(w25q, 1, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "ReadSR1: %s", rstr(st));
    TEST_ASSERT(!W25Q_STATUS_REG(w25q, W25Q_SR1_WEL_BIT), "WEL=1 apres 04h, SR1=0x%02X",
                (uint8_t)w25q->status_reg);

    PASS("06h -> WEL=1, 04h -> WEL=0 (tache SendCmd)");
}

W25Q_CASE(W25Q_rtos_test_t8_addr_mode_tracking) {
    const W25Q_config_t ref = w25q->config;
    W25Q_config_t cfg = ref;

    /* Toutes les operations d'abord, puis restauration, puis verifications :
       un TEST_ASSERT ne doit pas laisser la puce dans un mode non configure. */
    cfg.reg.addr_mode = W25Q_CFG_ADS_3B;
    task_ret_t st3 = rt_init(&cfg);
    int ads3 = W25Q_STATUS_REG(w25q, W25Q_SR3_ADS_BIT);

    /* SR3 reecrit tel quel avec ADS force a 1 : le bit est en lecture seule */
    uint8_t sr3 = (uint8_t)(w25q->status_reg >> 16);
    W25Q_STATE ws = W25Q_WriteStatus_RTOS(w25q, 3, (uint8_t)(sr3 | (1u << (W25Q_SR3_ADS_BIT - 16))),
                                          W25Q_SR_WRITE_VOLATILE, W25Q_TEST_TIMEOUT_MS);
    int ws_tracked = W25Q_STATUS_REG(w25q, W25Q_SR3_ADS_BIT);
    W25Q_STATE ws_rd = W25Q_ReadStatus_RTOS(w25q, 3, W25Q_TEST_TIMEOUT_MS);
    int ws_read = W25Q_STATUS_REG(w25q, W25Q_SR3_ADS_BIT);

    /* Mode 3 octets : 20h limite a 16 MB, 21h (4-byte) non limite */
    W25Q_STATE oor3 = W25Q_SendCmdAddr_RTOS(w25q, W25Q_SECTOR_ERASE, ADDR_ABOVE_16MB, W25Q_TEST_TIMEOUT_MS);
    W25Q_STATE op4 = W25Q_SendCmdAddr_RTOS(w25q, W25Q_SECTOR_ERASE_4B, ADDR_ABOVE_16MB, W25Q_TEST_TIMEOUT_MS);
    W25Q_STATE op4_wait = W25Q_WaitForReady_RTOS(w25q, W25Q_TEST_TIMEOUT_MS);

    /* SendCmd seul (sans Init, qui relit SR3) */
    W25Q_STATE sc_b7 = W25Q_SendCmd_RTOS(w25q, W25Q_ENABLE_4B_MODE, W25Q_TEST_TIMEOUT_MS);
    int b7_tracked = W25Q_STATUS_REG(w25q, W25Q_SR3_ADS_BIT);
    W25Q_STATE rs_b7 = W25Q_ReadStatus_RTOS(w25q, 3, W25Q_TEST_TIMEOUT_MS);
    int b7_read = W25Q_STATUS_REG(w25q, W25Q_SR3_ADS_BIT);
    W25Q_STATE sc_e9 = W25Q_SendCmd_RTOS(w25q, W25Q_DISABLE_4B_MODE, W25Q_TEST_TIMEOUT_MS);
    int e9_tracked = W25Q_STATUS_REG(w25q, W25Q_SR3_ADS_BIT);
    W25Q_STATE rs_e9 = W25Q_ReadStatus_RTOS(w25q, 3, W25Q_TEST_TIMEOUT_MS);
    int e9_read = W25Q_STATUS_REG(w25q, W25Q_SR3_ADS_BIT);

    task_ret_t st_ref = rt_init(&ref);

    TEST_ASSERT(st3 == W25Q_OK, "Init 3B: %s", rstr(st3));
    TEST_ASSERT(ads3 == 0, "ADS=%d apres config 3B", ads3);
    TEST_ASSERT(ws == W25Q_OK && ws_rd == W25Q_OK, "WriteStatus SR3: %s, ReadStatus: %s",
                rstr(ws), rstr(ws_rd));
    TEST_ASSERT(ws_tracked == 0 && ws_read == 0, "ADS apres WriteStatus: suivi=%d relu=%d (attendu 0)",
                ws_tracked, ws_read);
    TEST_ASSERT(oor3 == W25Q_PARAM_ERR, "20h en 3B >16MB: %s (attendu PARAM_ERR)", rstr(oor3));
    TEST_ASSERT(op4 == W25Q_OK && op4_wait == W25Q_OK, "21h en 3B >16MB: %s, attente: %s (attendu OK)",
                rstr(op4), rstr(op4_wait));
    TEST_ASSERT(sc_b7 == W25Q_OK && rs_b7 == W25Q_OK, "SendCmd(B7h): %s, ReadStatus: %s",
                rstr(sc_b7), rstr(rs_b7));
    TEST_ASSERT(b7_tracked == 1 && b7_read == 1, "ADS apres B7h: suivi=%d relu=%d (attendu 1)",
                b7_tracked, b7_read);
    TEST_ASSERT(sc_e9 == W25Q_OK && rs_e9 == W25Q_OK, "SendCmd(E9h): %s, ReadStatus: %s",
                rstr(sc_e9), rstr(rs_e9));
    TEST_ASSERT(e9_tracked == 0 && e9_read == 0, "ADS apres E9h: suivi=%d relu=%d (attendu 0)",
                e9_tracked, e9_read);
    TEST_ASSERT(st_ref == W25Q_OK, "Restauration config: %s", rstr(st_ref));
    PASS("ADS suivi (Init/WriteStatus/B7h/E9h), 20h/21h en 3B OK");
}


/* ======================= D. Effacement secteur ========================= */

W25Q_CASE(W25Q_rtos_test_t9_erase_verify) {
    task_ret_t st = rt_cmd_addr(W25Q_SECTOR_ERASE_4B, ADDR_SEC0);
    TEST_ASSERT(st == W25Q_OK, "SectorErase: %s", rstr(st));
    st = rt_read(buf_a, ADDR_SEC0, PAGE_BYTES);
    TEST_ASSERT(st == W25Q_OK, "Read: %s", rstr(st));
    uint32_t first_addr; uint8_t first_got;
    TEST_ASSERT(verify_uniform(buf_a, PAGE_BYTES, 0xFF, ADDR_SEC0, &first_addr, &first_got),
                "Non-0xFF @0x%06lX got=0x%02X", (unsigned long)first_addr, first_got);
    PASS("%uB=0xFF apres erase @0x%06lX (tache SendCmdAddr)", (unsigned)PAGE_BYTES, (unsigned long)ADDR_SEC0);
}


/* ======================= E. Lecture / ecriture ========================= */

W25Q_CASE(W25Q_rtos_test_t10_aligned_rw) {
    task_ret_t st = erase_sector(ADDR_SEC1);
    TEST_ASSERT(st == W25Q_OK, "Erase: %s", rstr(st));
    for (uint32_t i = 0; i < PAGE_BYTES; i++) buf_a[i] = (uint8_t)(0xA5 ^ i);
    st = rt_write(buf_a, ADDR_SEC1, PAGE_BYTES);
    TEST_ASSERT(st == W25Q_OK, "Write: %s", rstr(st));
    st = rt_read(buf_b, ADDR_SEC1, PAGE_BYTES);
    TEST_ASSERT(st == W25Q_OK, "Read: %s", rstr(st));
    uint32_t first_addr; uint8_t first_exp, first_got;
    uint32_t mm = count_mm(buf_a, buf_b, PAGE_BYTES, ADDR_SEC1, &first_addr, &first_exp, &first_got);
    TEST_ASSERT(mm == 0, "%lu mm, 1er @0x%06lX exp=0x%02X got=0x%02X",
                (unsigned long)mm, (unsigned long)first_addr, first_exp, first_got);
    PASS("256B @0x%06lX motif 0xA5^i OK", (unsigned long)ADDR_SEC1);
}

W25Q_CASE(W25Q_rtos_test_t11_cross_page_rw) {
    task_ret_t st = erase_sector(ADDR_SEC2);
    TEST_ASSERT(st == W25Q_OK, "Erase: %s", rstr(st));
    for (uint32_t i = 0; i < SIZE_CROSS_PAGE; i++) buf_a[i] = (uint8_t)(0xC3 ^ i);
    st = rt_write(buf_a, ADDR_CROSS_PAGE, SIZE_CROSS_PAGE);
    TEST_ASSERT(st == W25Q_OK, "Write: %s", rstr(st));
    st = rt_read(buf_b, ADDR_CROSS_PAGE, SIZE_CROSS_PAGE);
    TEST_ASSERT(st == W25Q_OK, "Read: %s", rstr(st));
    uint32_t first_addr; uint8_t first_exp, first_got;
    uint32_t mm = count_mm(buf_a, buf_b, SIZE_CROSS_PAGE, ADDR_CROSS_PAGE, &first_addr, &first_exp, &first_got);
    TEST_ASSERT(mm == 0, "%lu mm, 1er @0x%06lX exp=0x%02X got=0x%02X",
                (unsigned long)mm, (unsigned long)first_addr, first_exp, first_got);
    PASS("128B @0x%06lX cheval page7/8 OK", (unsigned long)ADDR_CROSS_PAGE);
}

W25Q_CASE(W25Q_rtos_test_t12_cross_sector_rw) {
    task_ret_t st;
    st = erase_sector(ADDR_SEC3);
    TEST_ASSERT(st == W25Q_OK, "Erase sec3: %s", rstr(st));
    st = erase_sector(ADDR_SEC4);
    TEST_ASSERT(st == W25Q_OK, "Erase sec4: %s", rstr(st));
    for (uint32_t i = 0; i < SIZE_CROSS_SECTOR; i++) buf_a[i] = (uint8_t)(0x55 ^ i);
    st = rt_write(buf_a, ADDR_CROSS_SECTOR, SIZE_CROSS_SECTOR);
    TEST_ASSERT(st == W25Q_OK, "Write: %s", rstr(st));
    st = rt_read(buf_b, ADDR_CROSS_SECTOR, SIZE_CROSS_SECTOR);
    TEST_ASSERT(st == W25Q_OK, "Read: %s", rstr(st));
    uint32_t first_addr; uint8_t first_exp, first_got;
    uint32_t mm = count_mm(buf_a, buf_b, SIZE_CROSS_SECTOR, ADDR_CROSS_SECTOR, &first_addr, &first_exp, &first_got);
    TEST_ASSERT(mm == 0, "%lu mm, 1er @0x%06lX exp=0x%02X got=0x%02X",
                (unsigned long)mm, (unsigned long)first_addr, first_exp, first_got);
    PASS("32B @0x%06lX cheval sec3/4 OK", (unsigned long)ADDR_CROSS_SECTOR);
}

W25Q_CASE(W25Q_rtos_test_t13_unaligned_rw) {
    task_ret_t st = erase_sector(ADDR_SEC8);
    TEST_ASSERT(st == W25Q_OK, "Erase: %s", rstr(st));

    for (uint32_t i = 0; i < SIZE_UNALIGNED; i++) buf_a[i] = (uint8_t)(0x3C ^ i);
    st = rt_write(buf_a, ADDR_UNALIGNED, SIZE_UNALIGNED);
    TEST_ASSERT(st == W25Q_OK, "Write: %s", rstr(st));

    st = rt_read(buf_b, ADDR_SEC8, 200U);
    TEST_ASSERT(st == W25Q_OK, "Read: %s", rstr(st));

    uint32_t first_addr; uint8_t first_got;
    TEST_ASSERT(verify_uniform(buf_b, 50U, 0xFF, ADDR_SEC8, &first_addr, &first_got),
                "Zone avant corrompue @0x%06lX got=0x%02X", (unsigned long)first_addr, first_got);

    uint32_t first_addr2; uint8_t first_exp2, first_got2;
    uint32_t mm = count_mm(buf_a, buf_b + 50U, SIZE_UNALIGNED, ADDR_UNALIGNED, &first_addr2, &first_exp2, &first_got2);
    TEST_ASSERT(mm == 0, "%lu mm @0x%06lX exp=0x%02X got=0x%02X",
                (unsigned long)mm, (unsigned long)first_addr2, first_exp2, first_got2);

    TEST_ASSERT(verify_uniform(buf_b + 150U, 50U, 0xFF, ADDR_UNALIGNED + SIZE_UNALIGNED, &first_addr, &first_got),
                "Zone apres corrompue @0x%06lX got=0x%02X", (unsigned long)first_addr, first_got);

    PASS("100B @0x%06lX (offset+50), pre/post=0xFF OK", (unsigned long)ADDR_UNALIGNED);
}

W25Q_CASE(W25Q_rtos_test_t14_multi_sector_rw) {
    task_ret_t st;
    st = erase_sector(ADDR_MULTI);
    TEST_ASSERT(st == W25Q_OK, "Erase sec0: %s", rstr(st));
    st = erase_sector(ADDR_MULTI + SECTOR_BYTES);
    TEST_ASSERT(st == W25Q_OK, "Erase sec1: %s", rstr(st));
    for (uint32_t i = 0; i < SIZE_LARGE; i++) {
        buf_large[i] = (uint8_t)(i & 0xFFU);
    }
    st = rt_write(buf_large, ADDR_MULTI, SIZE_LARGE);
    TEST_ASSERT(st == W25Q_OK, "Write: %s", rstr(st));
    st = rt_read(buf_large_rx, ADDR_MULTI, SIZE_LARGE);
    TEST_ASSERT(st == W25Q_OK, "Read: %s", rstr(st));
    uint32_t first_addr; uint8_t first_exp, first_got;
    uint32_t mm = count_mm(buf_large, buf_large_rx, SIZE_LARGE, ADDR_MULTI,
                           &first_addr, &first_exp, &first_got);
    TEST_ASSERT(mm == 0, "%lu mm, 1er @0x%06lX exp=0x%02X got=0x%02X",
                (unsigned long)mm, (unsigned long)first_addr, first_exp, first_got);
    PASS("%uB @0x%06lX (17 pages, 2 sec) OK", (unsigned)SIZE_LARGE, (unsigned long)ADDR_MULTI);
}

W25Q_CASE(W25Q_rtos_test_t15_and_behavior) {
    task_ret_t st = erase_sector(ADDR_SEC7);
    TEST_ASSERT(st == W25Q_OK, "Erase: %s", rstr(st));

    buf_byte = 0x0F;
    st = rt_write(&buf_byte, ADDR_SEC7, 1);
    TEST_ASSERT(st == W25Q_OK, "Write 0x0F: %s", rstr(st));

    buf_byte = 0xF0;
    st = rt_write(&buf_byte, ADDR_SEC7, 1);   /* pas d'effacement */
    TEST_ASSERT(st == W25Q_OK, "Write 0xF0: %s", rstr(st));

    buf_byte = 0xFF;
    st = rt_read(&buf_byte, ADDR_SEC7, 1);
    TEST_ASSERT(st == W25Q_OK, "Read: %s", rstr(st));

    const uint8_t expected = 0x0F & 0xF0;  /* = 0x00 */
    TEST_ASSERT(buf_byte == expected, "exp=0x%02X got=0x%02X (devrait etre AND)", expected, buf_byte);
    PASS("0xFF->0x0F->0x00 (0x0F & 0xF0) OK");
}

W25Q_CASE(W25Q_rtos_test_t16_rw_3b_mode) {
    const W25Q_config_t ref = w25q->config;
    W25Q_config_t cfg = ref;
    const uint32_t half = SIZE_ADDR_MODE / 2u;

    task_ret_t st = erase_sector(ADDR_ABOVE_16MB);
    TEST_ASSERT(st == W25Q_OK, "Erase >16MB: %s", rstr(st));
    for (uint32_t i = 0; i < SIZE_ADDR_MODE; i++) buf_a[i] = (uint8_t)(0xE1 ^ i);
    st = rt_write(buf_a, ADDR_ABOVE_16MB, half);
    TEST_ASSERT(st == W25Q_OK, "Write 1re moitie: %s", rstr(st));

    /* Operations en 3B, restauration, puis verifications */
    cfg.reg.addr_mode = W25Q_CFG_ADS_3B;
    task_ret_t st3 = rt_init(&cfg);
    task_ret_t wr3 = rt_write(buf_a + half, ADDR_ABOVE_16MB + half, half);
    task_ret_t rd3 = rt_read(buf_mode3, ADDR_ABOVE_16MB, SIZE_ADDR_MODE);
    task_ret_t st_ref = rt_init(&ref);
    task_ret_t rd4 = rt_read(buf_mode4, ADDR_ABOVE_16MB, SIZE_ADDR_MODE);

    TEST_ASSERT(st3 == W25Q_OK, "Init 3B: %s", rstr(st3));
    TEST_ASSERT(wr3 == W25Q_OK, "Write >16MB en 3B: %s", rstr(wr3));
    TEST_ASSERT(rd3 == W25Q_OK, "Read >16MB en 3B: %s", rstr(rd3));
    TEST_ASSERT(st_ref == W25Q_OK, "Restauration config: %s", rstr(st_ref));
    TEST_ASSERT(rd4 == W25Q_OK, "Read >16MB apres restauration: %s", rstr(rd4));
    uint32_t first_addr; uint8_t first_exp, first_got;
    uint32_t mm = count_mm(buf_a, buf_mode3, SIZE_ADDR_MODE, ADDR_ABOVE_16MB, &first_addr, &first_exp, &first_got);
    TEST_ASSERT(mm == 0, "Lecture en 3B: %lu mm @0x%07lX exp=0x%02X got=0x%02X",
                (unsigned long)mm, (unsigned long)first_addr, first_exp, first_got);
    mm = count_mm(buf_a, buf_mode4, SIZE_ADDR_MODE, ADDR_ABOVE_16MB, &first_addr, &first_exp, &first_got);
    TEST_ASSERT(mm == 0, "Lecture apres restauration: %lu mm @0x%07lX exp=0x%02X got=0x%02X",
                (unsigned long)mm, (unsigned long)first_addr, first_exp, first_got);
    PASS("%uB @0x%07lX ecrits 4B+3B, relus identiques", (unsigned)SIZE_ADDR_MODE, (unsigned long)ADDR_ABOVE_16MB);
}


/* ======================= F. Effacement etendu ========================== */

W25Q_CASE(W25Q_rtos_test_t17_erase_addr_len) {
    const W25Q_config_t ref = w25q->config;
    W25Q_config_t cfg = ref;

    task_ret_t st = erase_sector(ADDR_SEC11);
    TEST_ASSERT(st == W25Q_OK, "Erase sec11: %s", rstr(st));
    st = erase_sector(ADDR_SEC12);
    TEST_ASSERT(st == W25Q_OK, "Erase sec12: %s", rstr(st));

    /* Operations dans les deux modes, restauration, puis verifications */
    cfg.reg.addr_mode = W25Q_CFG_ADS_3B;
    task_ret_t st3 = rt_init(&cfg);
    task_ret_t er3;
    bool hit3 = erase_20h_hits(ADDR_SEC11, &er3);
    cfg.reg.addr_mode = W25Q_CFG_ADS_4B;
    task_ret_t st4 = rt_init(&cfg);
    task_ret_t er4;
    bool hit4 = erase_20h_hits(ADDR_SEC12, &er4);
    task_ret_t st_ref = rt_init(&ref);

    TEST_ASSERT(st3 == W25Q_OK, "Init 3B: %s", rstr(st3));
    TEST_ASSERT(er3 == W25Q_OK && hit3, "20h en 3B @0x%06lX: %s, secteur %s",
                (unsigned long)ADDR_SEC11, rstr(er3), hit3 ? "efface" : "non efface");
    TEST_ASSERT(st4 == W25Q_OK, "Init 4B: %s", rstr(st4));
    TEST_ASSERT(er4 == W25Q_OK && hit4, "20h en 4B @0x%06lX: %s, secteur %s",
                (unsigned long)ADDR_SEC12, rstr(er4), hit4 ? "efface" : "non efface");
    TEST_ASSERT(st_ref == W25Q_OK, "Restauration config: %s", rstr(st_ref));
    PASS("20h: 3 octets en 3B, 4 octets en 4B, bon secteur efface");
}

W25Q_CASE(W25Q_rtos_test_t18_erase_32kb) {
    /* Pre-condition : s'assure que la zone n'est pas deja vierge */
    task_ret_t st = erase_sector(ADDR_BLK32);
    TEST_ASSERT(st == W25Q_OK, "Pre-erase sector: %s", rstr(st));
    for (uint32_t i = 0; i < PAGE_BYTES; i++) {
        buf_a[i] = (uint8_t)(0xAB ^ i);
    }
    st = rt_write(buf_a, ADDR_BLK32, PAGE_BYTES);
    TEST_ASSERT(st == W25Q_OK, "Write motif: %s", rstr(st));
    /* Efface tout le bloc 32 KB (0x010000-0x017FFF) */
    st = rt_cmd_addr(W25Q_32KB_BLOCK_ERASE, ADDR_BLK32);
    TEST_ASSERT(st == W25Q_OK, "32KB Erase: %s", rstr(st));
    st = rt_read(buf_b, ADDR_BLK32, PAGE_BYTES);
    TEST_ASSERT(st == W25Q_OK, "Read: %s", rstr(st));
    uint32_t first_addr; uint8_t first_got;
    TEST_ASSERT(verify_uniform(buf_b, PAGE_BYTES, 0xFF, ADDR_BLK32, &first_addr, &first_got),
                "Non-0xFF @0x%06lX got=0x%02X", (unsigned long)first_addr, first_got);
    PASS("256B=0xFF apres 32KB erase @0x%06lX (tache)", (unsigned long)ADDR_BLK32);
}

W25Q_CASE(W25Q_rtos_test_t19_erase_64kb) {
    task_ret_t st = erase_sector(ADDR_BLK64);
    TEST_ASSERT(st == W25Q_OK, "Pre-erase sector: %s", rstr(st));
    for (uint32_t i = 0; i < PAGE_BYTES; i++) buf_a[i] = (uint8_t)(0xDE ^ i);
    st = rt_write(buf_a, ADDR_BLK64, PAGE_BYTES);
    TEST_ASSERT(st == W25Q_OK, "Write motif: %s", rstr(st));
    st = rt_cmd_addr(W25Q_64KB_BLOCK_ERASE_4B, ADDR_BLK64);
    TEST_ASSERT(st == W25Q_OK, "64KB Erase: %s", rstr(st));
    st = rt_read(buf_b, ADDR_BLK64, PAGE_BYTES);
    TEST_ASSERT(st == W25Q_OK, "Read: %s", rstr(st));
    uint32_t first_addr; uint8_t first_got;
    TEST_ASSERT(verify_uniform(buf_b, PAGE_BYTES, 0xFF, ADDR_BLK64, &first_addr, &first_got),
                "Non-0xFF @0x%06lX got=0x%02X", (unsigned long)first_addr, first_got);
    PASS("256B=0xFF apres 64KB erase @0x%06lX (tache)", (unsigned long)ADDR_BLK64);
}

W25Q_CASE(W25Q_rtos_test_t20_sector_isolation) {
    for (uint32_t i = 0; i < SIZE_ISOL; i++) { buf_isol_a[i] = 0xAA; buf_isol_b[i] = 0x55; }

    task_ret_t st;
    st = erase_sector(ADDR_SEC5);
    TEST_ASSERT(st == W25Q_OK, "Erase sec5: %s", rstr(st));
    st = erase_sector(ADDR_SEC6);
    TEST_ASSERT(st == W25Q_OK, "Erase sec6: %s", rstr(st));
    st = rt_write(buf_isol_a, ADDR_ISOL_END5, SIZE_ISOL);
    TEST_ASSERT(st == W25Q_OK, "Write 0xAA: %s", rstr(st));
    st = rt_write(buf_isol_b, ADDR_ISOL_BEG6, SIZE_ISOL);
    TEST_ASSERT(st == W25Q_OK, "Write 0x55: %s", rstr(st));
    st = erase_sector(ADDR_SEC5);  /* re-efface sec5 seul */
    TEST_ASSERT(st == W25Q_OK, "Re-erase sec5: %s", rstr(st));

    st = rt_read(buf_isol_rx, ADDR_ISOL_END5, SIZE_ISOL);
    TEST_ASSERT(st == W25Q_OK, "Read sec5: %s", rstr(st));
    uint32_t first_addr; uint8_t first_got;
    TEST_ASSERT(verify_uniform(buf_isol_rx, SIZE_ISOL, 0xFF, ADDR_ISOL_END5, &first_addr, &first_got),
                "Sec5 non efface @0x%06lX got=0x%02X", (unsigned long)first_addr, first_got);

    st = rt_read(buf_isol_rx, ADDR_ISOL_BEG6, SIZE_ISOL);
    TEST_ASSERT(st == W25Q_OK, "Read sec6: %s", rstr(st));
    uint32_t first_addr2; uint8_t first_exp2, first_got2;
    uint32_t mm = count_mm(buf_isol_b, buf_isol_rx, SIZE_ISOL, ADDR_ISOL_BEG6, &first_addr2, &first_exp2, &first_got2);
    TEST_ASSERT(mm == 0, "Sec6 corrompu: %lu mm @0x%06lX exp=0x%02X got=0x%02X",
                (unsigned long)mm, (unsigned long)first_addr2, first_exp2, first_got2);
    PASS("Sec5=0xFF efface, sec6=0x55 intact");
}

W25Q_CASE(W25Q_rtos_test_t21_busy_timeout) {
    /* L'effacement rend la main sans attendre la fin (pas de WAIT_AFTER) */
    W25Q_STATE st = erase_sector(ADDR_SEC10);
    TEST_ASSERT(st == W25Q_OK, "SectorErase: %s", rstr(st));
    W25Q_STATE st_short = W25Q_WaitForReady_RTOS(w25q, 1U);   /* erase 4 KB : ~45 ms typ. */
    W25Q_STATE st_long  = W25Q_WaitForReady_RTOS(w25q, W25Q_TEST_TIMEOUT_MS);

    TEST_ASSERT(st_short == W25Q_BUSY_TIMEOUT, "WaitForReady(1 ms): %s (attendu TIMEOUT)", rstr(st_short));
    TEST_ASSERT(st_long == W25Q_OK, "WaitForReady(%u ms): %s", (unsigned)W25Q_TEST_TIMEOUT_MS, rstr(st_long));
    TEST_ASSERT(!W25Q_STATUS_REG(w25q, W25Q_SR1_BUSY_BIT), "BUSY=1 apres WaitForReady OK");
    PASS("1 ms -> TIMEOUT, %u ms -> OK", (unsigned)W25Q_TEST_TIMEOUT_MS);
}


/* ======================= G. Cas limites R/W ============================ */

W25Q_CASE(W25Q_rtos_test_t22_write_zero_size) {
    task_ret_t st = erase_sector(ADDR_SEC9);
    TEST_ASSERT(st == W25Q_OK, "Erase: %s", rstr(st));
    for (uint32_t i = 0; i < PAGE_BYTES; i++) buf_a[i] = (uint8_t)(0x96 ^ i);
    st = rt_write(buf_a, ADDR_SEC9, 0U);
    TEST_ASSERT(st == W25Q_OK, "Write(size=0): %s (attendu OK)", rstr(st));
    st = rt_read(buf_b, ADDR_SEC9, PAGE_BYTES);
    TEST_ASSERT(st == W25Q_OK, "Read: %s", rstr(st));
    uint32_t first_addr; uint8_t first_got;
    TEST_ASSERT(verify_uniform(buf_b, PAGE_BYTES, 0xFF, ADDR_SEC9, &first_addr, &first_got),
                "Zone modifiee @0x%06lX got=0x%02X (attendu 0xFF)", (unsigned long)first_addr, first_got);
    PASS("W25Q_OK, zone intacte (0xFF)");
}

W25Q_CASE(W25Q_rtos_test_t23_write_end_clamp) {
    task_ret_t st = erase_sector(ADDR_NEAR_END);
    TEST_ASSERT(st == W25Q_OK, "Erase: %s", rstr(st));
    for (uint32_t i = 0; i < PAGE_BYTES; i++) buf_a[i] = (uint8_t)(0x7E ^ i);
    /* Tente d'ecrire 256 B alors qu'il ne reste que 128 B disponibles */
    st = rt_write(buf_a, ADDR_NEAR_END, PAGE_BYTES);
    TEST_ASSERT(st == W25Q_OK, "Write: %s (attendu OK)", rstr(st));
    st = rt_read(buf_b, ADDR_NEAR_END, 128U);
    TEST_ASSERT(st == W25Q_OK, "Read: %s", rstr(st));
    uint32_t first_addr; uint8_t first_exp, first_got;
    uint32_t mm = count_mm(buf_a, buf_b, 128U, ADDR_NEAR_END, &first_addr, &first_exp, &first_got);
    TEST_ASSERT(mm == 0, "%lu mm @0x%06lX exp=0x%02X got=0x%02X",
                (unsigned long)mm, (unsigned long)first_addr, first_exp, first_got);
    PASS("128/256B clampees @0x%06lX OK", (unsigned long)ADDR_NEAR_END);
}

W25Q_CASE(W25Q_rtos_test_t24_read_end_clamp) {
    task_ret_t st = erase_sector(ADDR_NEAR_END);
    TEST_ASSERT(st == W25Q_OK, "Erase: %s", rstr(st));
    for (uint32_t i = 0; i < PAGE_BYTES; i++) buf_b[i] = SENTINEL;
    /* Demande 256 B alors qu'il n'en reste que 128 avant la fin du flash */
    st = rt_read(buf_b, ADDR_NEAR_END, PAGE_BYTES);
    TEST_ASSERT(st == W25Q_OK, "Read: %s (attendu OK)", rstr(st));
    uint32_t first_addr; uint8_t first_got;
    TEST_ASSERT(verify_uniform(buf_b, 128U, 0xFF, ADDR_NEAR_END, &first_addr, &first_got),
                "Partie valide non-0xFF @0x%06lX got=0x%02X", (unsigned long)first_addr, first_got);
    TEST_ASSERT(verify_uniform(buf_b + 128U, 128U, SENTINEL, 128U, &first_addr, &first_got),
                "Buffer ecrase apres la limite, off=%lu got=0x%02X", (unsigned long)first_addr, first_got);
    PASS("128/256B lus @0x%06lX, reste du buffer intact", (unsigned long)ADDR_NEAR_END);
}

W25Q_CASE(W25Q_rtos_test_t25_addr_out_of_range) {
    /* addr == taille du flash : hors plage, doit etre rejete sans toucher le bus */
    task_ret_t st = rt_write(buf_a, W25Q_FLASH_SIZE_BYTES, PAGE_BYTES);
    TEST_ASSERT(st == W25Q_PARAM_ERR, "Write hors plage: %s (attendu PARAM_ERR)", rstr(st));
    st = rt_read(buf_b, W25Q_FLASH_SIZE_BYTES, PAGE_BYTES);
    TEST_ASSERT(st == W25Q_PARAM_ERR, "Read hors plage: %s (attendu PARAM_ERR)", rstr(st));
    /* Le driver doit rester utilisable apres un rejet (CS relache, bus libre) */
    uint8_t id[3] = {0};
    st = W25Q_ReadID_RTOS(w25q, id, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "ReadID apres rejet: %s", rstr(st));
    PASS("addr=taille flash -> PARAM_ERR (R et W)");
}


/* ======================= H. Reset ====================================== */

W25Q_CASE(W25Q_rtos_test_t26_soft_reset) {
    task_ret_t st = rt_cmd(W25Q_ENABLE_RESET);
    TEST_ASSERT(st == W25Q_OK, "ENABLE_RESET: %s", rstr(st));
    st = rt_cmd(W25Q_RESET);
    TEST_ASSERT(st == W25Q_OK, "RESET: %s", rstr(st));
    /* Le driver doit avoir resynchronise son cache : ADS suivi = ADS relu */
    int ads_tracked = W25Q_STATUS_REG(w25q, W25Q_SR3_ADS_BIT);
    st = read_all_status();
    TEST_ASSERT(st == W25Q_OK, "ReadStatus post-reset: %s", rstr(st));
    TEST_ASSERT(ads_tracked == W25Q_STATUS_REG(w25q, W25Q_SR3_ADS_BIT),
                "ADS suivi=%d != relu=%d apres reset", ads_tracked, W25Q_STATUS_REG(w25q, W25Q_SR3_ADS_BIT));
    uint8_t id[3] = {0};
    st = W25Q_ReadID_RTOS(w25q, id, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "ReadID post-reset: %s", rstr(st));
    uint16_t dev = (uint16_t)((id[1] << 8) | id[2]);
    TEST_ASSERT(id[0] == W25Q_MANUFACTURER_ID && dev == W25Q_V_FULL_DEVICE_ID,
                "Post-reset: Manuf=0x%02X Dev=0x%04X", id[0], dev);
    int ads = W25Q_STATUS_REG(w25q, W25Q_SR3_ADS_BIT);
    int adp = W25Q_STATUS_REG(w25q, W25Q_SR3_ADP_BIT);
    /* Le reset efface ADS et les ecritures volatiles : re-applique la config */
    const W25Q_config_t ref = w25q->config;
    task_ret_t st_cfg = rt_init(&ref);
    TEST_ASSERT(ads == adp, "Post-reset: ADS=%d != ADP=%d", ads, adp);
    TEST_ASSERT(st_cfg == W25Q_OK, "Re-init post-reset: %s", rstr(st_cfg));
    PASS("ID OK post-reset, ADS=ADP=%d, config re-appliquee", ads);
}


/* ======================= I. Semaphore de la puce ======================= */

W25Q_CASE(W25Q_rtos_test_t27_lock_error_paths) {
    W25Q_STATE st = W25Q_SendCmdAddr_RTOS(w25q, W25Q_64KB_BLOCK_ERASE_4B, ADDR_BUSY64, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "Erase 64KB: %s", rstr(st));

    /* Puce occupee (~150 ms typ.) : chaque appel expire sur BUSY avec le
       semaphore pris, et doit le rendre. Mesures d'abord, verifications ensuite. */
    W25Q_STATE s_wait = W25Q_WaitForReady_RTOS(w25q, 1u);
    uint32_t   c_wait = sem_count();
    W25Q_STATE s_addr = W25Q_SendCmdAddr_RTOS(w25q, W25Q_SECTOR_ERASE_4B, ADDR_BUSY64, 1u);
    uint32_t   c_addr = sem_count();
    W25Q_STATE s_pp   = W25Q_PageProgram_RTOS(w25q, buf_a, ADDR_BUSY64, PAGE_BYTES, 1u);
    uint32_t   c_pp   = sem_count();
    task_ret_t s_rd   = rt_read_on(w25q, buf_b, ADDR_BUSY64, PAGE_BYTES, 1u);
    uint32_t   c_rd   = sem_count();
    W25Q_STATE s_end  = W25Q_WaitForReady_RTOS(w25q, W25Q_TEST_TIMEOUT_MS);

    TEST_ASSERT(s_wait == W25Q_BUSY_TIMEOUT && c_wait == 1u, "WaitForReady: %s, count=%lu",
                rstr(s_wait), (unsigned long)c_wait);
    TEST_ASSERT(s_addr == W25Q_BUSY_TIMEOUT && c_addr == 1u, "SendCmdAddr: %s, count=%lu",
                rstr(s_addr), (unsigned long)c_addr);
    TEST_ASSERT(s_pp == W25Q_BUSY_TIMEOUT && c_pp == 1u, "PageProgram: %s, count=%lu",
                rstr(s_pp), (unsigned long)c_pp);
    TEST_ASSERT(s_rd == W25Q_BUSY_TIMEOUT && c_rd == 1u, "Tache Read: %s, count=%lu",
                rstr(s_rd), (unsigned long)c_rd);
    TEST_ASSERT(s_end == W25Q_OK, "Attente fin d'effacement: %s", rstr(s_end));
    PASS("4 chemins BUSY_TIMEOUT, semaphore rendu a chaque fois");
}

W25Q_CASE(W25Q_rtos_test_t28_nolock_held) {
    osStatus_t a = osSemaphoreAcquire(w25q->sem_id, 100u);
    TEST_ASSERT(a == osOK, "Prise du semaphore: %d", (int)a);

    uint8_t id[3] = {0};
    W25Q_STATE l0 = W25Q_ReadID_RTOS(w25q, id, 0u);
    uint32_t t0 = osKernelGetTickCount();
    W25Q_STATE l20 = W25Q_ReadID_RTOS(w25q, id, 20u);
    uint32_t el = osKernelGetTickCount() - t0;

    W25Q_STATE nl_id   = W25Q_ReadID_RTOS_NoLock(w25q, id);
    W25Q_STATE nl_sr   = W25Q_ReadStatus_RTOS_NoLock(w25q, 1);
    W25Q_STATE nl_cmd  = W25Q_SendCmd_RTOS_NoLock(w25q, W25Q_WRITE_DISABLE, W25Q_TEST_TIMEOUT_MS);
    W25Q_STATE nl_wait = W25Q_WaitForReady_RTOS_NoLock(w25q, W25Q_TEST_TIMEOUT_MS);
    uint32_t count_held = sem_count();

    (void)osSemaphoreRelease(w25q->sem_id);

    TEST_ASSERT(l0 == W25Q_LOCK_TIMEOUT, "ReadID(0 ms) semaphore pris: %s (attendu LOCK_TIMEOUT)", rstr(l0));
    TEST_ASSERT(l20 == W25Q_LOCK_TIMEOUT, "ReadID(20 ms) semaphore pris: %s (attendu LOCK_TIMEOUT)", rstr(l20));
    TEST_ASSERT(el >= 18u && el <= 40u, "ReadID(20 ms) a attendu %lu ms", (unsigned long)el);
    TEST_ASSERT(nl_id == W25Q_OK && id[0] == W25Q_MANUFACTURER_ID, "ReadID_NoLock: %s, Manuf=0x%02X",
                rstr(nl_id), id[0]);
    TEST_ASSERT(nl_sr == W25Q_OK, "ReadStatus_NoLock: %s", rstr(nl_sr));
    TEST_ASSERT(nl_cmd == W25Q_OK, "SendCmd_NoLock(04h): %s", rstr(nl_cmd));
    TEST_ASSERT(nl_wait == W25Q_OK, "WaitForReady_NoLock: %s", rstr(nl_wait));
    TEST_ASSERT(count_held == 0u, "count=%lu pendant la detention (attendu 0)", (unsigned long)count_held);
    PASS("Verrouille: LOCK_TIMEOUT (0 et %lu ms), _NoLock OK", (unsigned long)el);
}

W25Q_CASE(W25Q_rtos_test_t29_lock_timeout) {
    uint8_t id[3] = {0};
    R[0] = RT_NOT_SET;
    hold_taken = false;
    task_h_t hh = W25QT_Hold_spawn(&(W25QT_Hold_args_t){ .chip = w25q, .ms = 200u },
                                   &(task_attr_t){ .priority = osPriorityAboveNormal, .ret = &R[0], .join_bit = 2u });
    TEST_ASSERT(task_h_valid(hh), "Spawn Hold refuse");
    TEST_ASSERT(wait_hold(true, 100u), "Le job Hold n'a pas pris le semaphore");

    uint32_t t0 = osKernelGetTickCount();
    W25Q_STATE st_short = W25Q_ReadID_RTOS(w25q, id, 50u);
    uint32_t el_short = osKernelGetTickCount() - t0;

    task_ret_t rd = rt_read_on(w25q, buf_b, ADDR_SEC0, 16u, 20u);

    t0 = osKernelGetTickCount();
    W25Q_STATE st_long = W25Q_ReadID_RTOS(w25q, id, 1000u);
    uint32_t el_long = osKernelGetTickCount() - t0;

    osStatus_t j = task_join(hh, JOIN_TIMEOUT_MS);

    TEST_ASSERT(st_short == W25Q_LOCK_TIMEOUT, "ReadID(50 ms): %s (attendu LOCK_TIMEOUT)", rstr(st_short));
    TEST_ASSERT(el_short >= 48u && el_short <= 80u, "ReadID(50 ms) a attendu %lu ms", (unsigned long)el_short);
    TEST_ASSERT(rd == W25Q_LOCK_TIMEOUT, "Tache Read(20 ms): %s (attendu LOCK_TIMEOUT)", rstr(rd));
    TEST_ASSERT(st_long == W25Q_OK && id[0] == W25Q_MANUFACTURER_ID, "ReadID(1000 ms): %s, Manuf=0x%02X",
                rstr(st_long), id[0]);
    TEST_ASSERT(el_long < 200u, "ReadID(1000 ms) a attendu %lu ms (> duree de detention)", (unsigned long)el_long);
    TEST_ASSERT(j == osOK && R[0] == W25Q_OK, "Job Hold: join %d, ret %s", (int)j, rstr(R[0]));
    PASS("LOCK_TIMEOUT apres %lu ms, OK apres %lu ms d'attente", (unsigned long)el_short, (unsigned long)el_long);
}

W25Q_CASE(W25Q_rtos_test_t30_uninit_chip) {
    uint8_t id[3] = {0};
    memset(&w25q_uninit, 0, sizeof(w25q_uninit));
    w25q_uninit.config.bus = w25q->config.bus;   /* un chemin fautif parlerait a la vraie puce, sans planter */

    W25Q_STATE s_id  = W25Q_ReadID_RTOS(&w25q_uninit, id, 10u);
    W25Q_STATE s_sr  = W25Q_ReadStatus_RTOS(&w25q_uninit, 1, 10u);
    W25Q_STATE s_cmd = W25Q_SendCmd_RTOS(&w25q_uninit, W25Q_WRITE_DISABLE, 10u);
    task_ret_t s_rd  = rt_read_on(&w25q_uninit, buf_b, ADDR_SEC0, 16u, 10u);

    W25Q_config_t cfg = w25q->config;
    task_ret_t s_null = join_sync(W25Q_Init_spawn(&(W25Q_Init_args_t){
        .chip = NULL, .config = cfg, .timeout_ms = W25Q_TEST_TIMEOUT_MS }, SYNC_ATTR));
    cfg.bus.cs_bank = NULL;
    task_ret_t s_cs = rt_init(&cfg);

    TEST_ASSERT(s_id == W25Q_SEM_ERR, "ReadID non init: %s (attendu SEM_ERR)", rstr(s_id));
    TEST_ASSERT(s_sr == W25Q_SEM_ERR, "ReadStatus non init: %s (attendu SEM_ERR)", rstr(s_sr));
    TEST_ASSERT(s_cmd == W25Q_SEM_ERR, "SendCmd non init: %s (attendu SEM_ERR)", rstr(s_cmd));
    TEST_ASSERT(s_rd == W25Q_SEM_ERR, "Tache Read non init: %s (attendu SEM_ERR)", rstr(s_rd));
    TEST_ASSERT(w25q_uninit.sem_id == NULL, "Semaphore cree hors Init");
    TEST_ASSERT(s_null == W25Q_PARAM_ERR, "Init chip NULL: %s (attendu PARAM_ERR)", rstr(s_null));
    TEST_ASSERT(s_cs == W25Q_PARAM_ERR, "Init cs_bank NULL: %s (attendu PARAM_ERR)", rstr(s_cs));
    TEST_ASSERT(w25q->config.bus.cs_bank != NULL, "chip->config modifiee par un Init rejete");

    W25Q_STATE st = W25Q_ReadID_RTOS(w25q, id, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK && id[0] == W25Q_MANUFACTURER_ID, "ReadID vraie puce: %s", rstr(st));
    PASS("Non init -> SEM_ERR (x4), Init NULL/cs NULL -> PARAM_ERR");
}

W25Q_CASE(W25Q_rtos_test_t31_reinit_concurrent) {
    task_ret_t st = erase_sector(ADDR_I_RE);
    TEST_ASSERT(st == W25Q_OK, "Erase sec0: %s", rstr(st));
    st = erase_sector(ADDR_I_RE + SECTOR_BYTES);
    TEST_ASSERT(st == W25Q_OK, "Erase sec1: %s", rstr(st));
    for (uint32_t i = 0; i < SIZE_LARGE; i++) buf_large[i] = (uint8_t)(0x6B ^ i ^ (i >> 8));

    const osSemaphoreId_t sem_before = w25q->sem_id;
    const W25Q_config_t ref = w25q->config;
    R[0] = RT_NOT_SET;
    R[1] = RT_NOT_SET;

    /* L'ecriture demarre aussitot (priorite superieure) puis rend la main sur
       son premier transfert DMA : l'Init est lance pendant qu'elle tourne. */
    task_h_t h[2];
    h[0] = W25Q_Write_spawn(&(W25Q_Write_args_t){ .chip = w25q, .buf = buf_large, .addr = ADDR_I_RE,
                                                  .len = SIZE_LARGE, .timeout_ms = W25Q_TEST_TIMEOUT_MS },
                            &(task_attr_t){ .priority = osPriorityAboveNormal, .ret = &R[0], .join_bit = 1u });
    TEST_ASSERT(task_h_valid(h[0]), "Spawn Write refuse");
    bool write_running = task_running(h[0]);
    h[1] = W25Q_Init_spawn(&(W25Q_Init_args_t){ .chip = w25q, .config = ref, .timeout_ms = W25Q_TEST_TIMEOUT_MS },
                           &(task_attr_t){ .priority = osPriorityHigh, .ret = &R[1], .join_bit = 2u });
    TEST_ASSERT(task_h_valid(h[1]), "Spawn Init refuse");

    osStatus_t j = task_join_all(h, 2u, JOIN_TIMEOUT_MS);
    TEST_ASSERT(j == osOK, "join_all: %d", (int)j);
    TEST_ASSERT(write_running, "Ecriture finie avant le lancement de l'Init : test non concluant");
    TEST_ASSERT(R[0] == W25Q_OK, "Write: %s", rstr(R[0]));
    TEST_ASSERT(R[1] == W25Q_OK, "Re-init: %s", rstr(R[1]));
    TEST_ASSERT(w25q->sem_id == sem_before, "Semaphore recree par le re-init");

    st = rt_read(buf_large_rx, ADDR_I_RE, SIZE_LARGE);
    TEST_ASSERT(st == W25Q_OK, "Read: %s", rstr(st));
    uint32_t first_addr; uint8_t first_exp, first_got;
    uint32_t mm = count_mm(buf_large, buf_large_rx, SIZE_LARGE, ADDR_I_RE, &first_addr, &first_exp, &first_got);
    TEST_ASSERT(mm == 0, "%lu mm, 1er @0x%06lX exp=0x%02X got=0x%02X",
                (unsigned long)mm, (unsigned long)first_addr, first_exp, first_got);
    PASS("Write 17 pages + re-init concurrent OK, meme semaphore");
}


/* ======================= J. Integration scheduler ====================== */

W25Q_CASE(W25Q_rtos_test_t32_args_detached) {
    task_ret_t st = erase_sector(ADDR_J_ARGS);
    TEST_ASSERT(st == W25Q_OK, "Erase: %s", rstr(st));
    st = erase_sector(ADDR_J_ARGS2);
    TEST_ASSERT(st == W25Q_OK, "Erase 2: %s", rstr(st));
    for (uint32_t i = 0; i < PAGE_BYTES; i++) buf_a[i] = (uint8_t)(0x2D ^ i);

    /* Le semaphore est tenu : le Write reste bloque apres son spawn. */
    R[0] = RT_NOT_SET;
    R[1] = RT_NOT_SET;
    hold_taken = false;
    task_h_t hh = W25QT_Hold_spawn(&(W25QT_Hold_args_t){ .chip = w25q, .ms = 50u },
                                   &(task_attr_t){ .priority = osPriorityAboveNormal, .ret = &R[1], .join_bit = 2u });
    TEST_ASSERT(task_h_valid(hh), "Spawn Hold refuse");
    TEST_ASSERT(wait_hold(true, 100u), "Le job Hold n'a pas pris le semaphore");

    W25Q_Write_args_t a = { .chip = w25q, .buf = buf_a, .addr = ADDR_J_ARGS,
                            .len = PAGE_BYTES, .timeout_ms = W25Q_TEST_TIMEOUT_MS };
    task_h_t hw = W25Q_Write_spawn(&a, &(task_attr_t){ .priority = osPriorityAboveNormal, .ret = &R[0],
                                                       .join_bit = TASK_JOIN_NONE });
    TEST_ASSERT(task_h_valid(hw), "Spawn Write detache refuse");
    a.buf  = buf_b;          /* apres le spawn : sans effet sur le job */
    a.addr = ADDR_J_ARGS2;
    a.len  = 1u;
    bool blocked = task_running(hw);

    bool done = wait_done(hw, JOIN_TIMEOUT_MS);
    osStatus_t j = task_join(hh, JOIN_TIMEOUT_MS);

    TEST_ASSERT(blocked, "Write deja fini alors que le semaphore etait tenu");
    TEST_ASSERT(done, "Write detache toujours actif apres %u ms", (unsigned)JOIN_TIMEOUT_MS);
    TEST_ASSERT(R[0] == W25Q_OK, "ret du job detache: %s", rstr(R[0]));
    TEST_ASSERT(j == osOK && R[1] == W25Q_OK, "Job Hold: join %d, ret %s", (int)j, rstr(R[1]));
    TEST_ASSERT(a.len == 1u && a.addr == ADDR_J_ARGS2, "Struct de l'appelant modifiee par le job (len=%lu)",
                (unsigned long)a.len);

    st = rt_read(buf_b, ADDR_J_ARGS, PAGE_BYTES);
    TEST_ASSERT(st == W25Q_OK, "Read: %s", rstr(st));
    uint32_t first_addr; uint8_t first_exp, first_got;
    uint32_t mm = count_mm(buf_a, buf_b, PAGE_BYTES, ADDR_J_ARGS, &first_addr, &first_exp, &first_got);
    TEST_ASSERT(mm == 0, "Zone d'origine: %lu mm @0x%06lX exp=0x%02X got=0x%02X",
                (unsigned long)mm, (unsigned long)first_addr, first_exp, first_got);
    st = rt_read(buf_b, ADDR_J_ARGS2, 16u);
    TEST_ASSERT(st == W25Q_OK, "Read 2: %s", rstr(st));
    TEST_ASSERT(verify_uniform(buf_b, 16u, 0xFF, ADDR_J_ARGS2, &first_addr, &first_got),
                "Zone des args modifies ecrite @0x%06lX got=0x%02X", (unsigned long)first_addr, first_got);
    PASS("Copie au spawn, ret publie sans join, appelant intact");
}

W25Q_CASE(W25Q_rtos_test_t33_join_timeout) {
    W25Q_STATE st = W25Q_SendCmdAddr_RTOS(w25q, W25Q_64KB_BLOCK_ERASE_4B, ADDR_BUSY64, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "Erase 64KB: %s", rstr(st));

    memset(buf_b, 0x00, PAGE_BYTES);
    R[0] = RT_NOT_SET;
    task_h_t h = W25Q_Read_spawn(&(W25Q_Read_args_t){ .chip = w25q, .buf = buf_b, .addr = ADDR_BUSY64,
                                                      .len = PAGE_BYTES, .timeout_ms = W25Q_TEST_TIMEOUT_MS },
                                 &(task_attr_t){ .priority = osPriorityNormal, .ret = &R[0], .join_bit = 1u });
    TEST_ASSERT(task_h_valid(h), "Spawn Read refuse");

    osStatus_t j1      = task_join(h, 5u);
    bool       running = task_running(h);
    task_ret_t r_mid   = R[0];
    osStatus_t j2      = task_join(h, JOIN_TIMEOUT_MS);

    TEST_ASSERT(j1 == osErrorTimeout, "join(5 ms): %d (attendu osErrorTimeout)", (int)j1);
    TEST_ASSERT(running, "Job fini en 5 ms alors que la puce efface");
    TEST_ASSERT(r_mid == RT_NOT_SET, "ret publie avant la fin du job: %s", rstr(r_mid));
    TEST_ASSERT(j2 == osOK, "join final: %d", (int)j2);
    TEST_ASSERT(R[0] == W25Q_OK, "Read: %s", rstr(R[0]));
    uint32_t first_addr; uint8_t first_got;
    TEST_ASSERT(verify_uniform(buf_b, PAGE_BYTES, 0xFF, ADDR_BUSY64, &first_addr, &first_got),
                "Non-0xFF @0x%06lX got=0x%02X", (unsigned long)first_addr, first_got);
    PASS("join(5 ms) expire, job poursuivi, 2e join OK, zone effacee");
}


/* ======================= K. Concurrence ================================ */

W25Q_CASE(W25Q_rtos_test_t34_concurrent_writes) {
    static const osPriority_t prio[N_WRITERS] = {
        osPriorityBelowNormal, osPriorityNormal, osPriorityAboveNormal, osPriorityHigh,
    };
    task_h_t h[N_WRITERS];
    task_ret_t st;

    for (uint32_t i = 0; i < N_WRITERS; i++) {
        st = erase_sector(ADDR_K1 + i * SECTOR_BYTES);
        TEST_ASSERT(st == W25Q_OK, "Erase zone %lu: %s", (unsigned long)i, rstr(st));
        for (uint32_t j = 0; j < SIZE_K; j++) buf_k_tx[i][j] = (uint8_t)((0x11u * (i + 1u)) ^ j);
        R[i] = RT_NOT_SET;
    }

    for (uint32_t i = 0; i < N_WRITERS; i++) {
        h[i] = W25Q_Write_spawn(&(W25Q_Write_args_t){ .chip = w25q, .buf = buf_k_tx[i],
                                                      .addr = ADDR_K1 + i * SECTOR_BYTES + K1_OFFSET,
                                                      .len = SIZE_K, .timeout_ms = W25Q_TEST_TIMEOUT_MS },
                                &(task_attr_t){ .priority = prio[i], .ret = &R[i], .join_bit = (uint8_t)(1u + i) });
        TEST_ASSERT(task_h_valid(h[i]), "Spawn Write %lu refuse", (unsigned long)i);
    }
    osStatus_t j = task_join_all(h, N_WRITERS, JOIN_TIMEOUT_MS);
    TEST_ASSERT(j == osOK, "join_all: %d", (int)j);

    for (uint32_t i = 0; i < N_WRITERS; i++) {
        uint32_t addr = ADDR_K1 + i * SECTOR_BYTES + K1_OFFSET;
        TEST_ASSERT(R[i] == W25Q_OK, "Write %lu: %s", (unsigned long)i, rstr(R[i]));
        st = rt_read(buf_k_rx, addr, SIZE_K);
        TEST_ASSERT(st == W25Q_OK, "Read %lu: %s", (unsigned long)i, rstr(st));
        uint32_t first_addr; uint8_t first_exp, first_got;
        uint32_t mm = count_mm(buf_k_tx[i], buf_k_rx, SIZE_K, addr, &first_addr, &first_exp, &first_got);
        TEST_ASSERT(mm == 0, "Ecrivain %lu: %lu mm @0x%06lX exp=0x%02X got=0x%02X", (unsigned long)i,
                    (unsigned long)mm, (unsigned long)first_addr, first_exp, first_got);
    }
    PASS("%u ecrivains x %uB (4 priorites), donnees exactes", (unsigned)N_WRITERS, (unsigned)SIZE_K);
}

W25Q_CASE(W25Q_rtos_test_t35_concurrent_erases) {
    task_h_t h[2];
    task_ret_t st;

    for (uint32_t i = 0; i < 2u; i++) {
        uint32_t addr = ADDR_K2 + i * SECTOR_BYTES;
        st = erase_sector(addr);
        TEST_ASSERT(st == W25Q_OK, "Pre-erase %lu: %s", (unsigned long)i, rstr(st));
        for (uint32_t k = 0; k < 16u; k++) buf_a[k] = (uint8_t)(0x93 ^ k ^ i);
        st = rt_write(buf_a, addr, 16u);
        TEST_ASSERT(st == W25Q_OK, "Write motif %lu: %s", (unsigned long)i, rstr(st));
        R[i] = RT_NOT_SET;
    }

    for (uint32_t i = 0; i < 2u; i++) {
        h[i] = W25Q_SendCmdAddr_spawn(&(W25Q_SendCmdAddr_args_t){ .chip = w25q, .cmd = W25Q_SECTOR_ERASE_4B,
                                                                  .addr = ADDR_K2 + i * SECTOR_BYTES,
                                                                  .timeout_ms = W25Q_TEST_TIMEOUT_MS },
                                      &(task_attr_t){ .priority = osPriorityAboveNormal, .ret = &R[i],
                                                      .join_bit = (uint8_t)(1u + i) });
        TEST_ASSERT(task_h_valid(h[i]), "Spawn erase %lu refuse", (unsigned long)i);
    }
    osStatus_t j = task_join_all(h, 2u, JOIN_TIMEOUT_MS);
    TEST_ASSERT(j == osOK, "join_all: %d", (int)j);

    for (uint32_t i = 0; i < 2u; i++) {
        uint32_t addr = ADDR_K2 + i * SECTOR_BYTES;
        TEST_ASSERT(R[i] == W25Q_OK, "Erase %lu: %s", (unsigned long)i, rstr(R[i]));
        st = rt_read(buf_b, addr, 16u);   /* attend la fin du dernier effacement */
        TEST_ASSERT(st == W25Q_OK, "Read %lu: %s", (unsigned long)i, rstr(st));
        uint32_t first_addr; uint8_t first_got;
        TEST_ASSERT(verify_uniform(buf_b, 16u, 0xFF, addr, &first_addr, &first_got),
                    "Secteur %lu non efface @0x%06lX got=0x%02X", (unsigned long)i,
                    (unsigned long)first_addr, first_got);
    }
    PASS("2 effacements concurrents, 2 secteurs vierges");
}

W25Q_CASE(W25Q_rtos_test_t36_read_during_write) {
    task_ret_t st;
    st = erase_sector(ADDR_K3_W);
    TEST_ASSERT(st == W25Q_OK, "Erase W0: %s", rstr(st));
    st = erase_sector(ADDR_K3_W + SECTOR_BYTES);
    TEST_ASSERT(st == W25Q_OK, "Erase W1: %s", rstr(st));
    st = erase_sector(ADDR_K3_R);
    TEST_ASSERT(st == W25Q_OK, "Erase R: %s", rstr(st));

    for (uint32_t j = 0; j < SIZE_K; j++) buf_k_tx[0][j] = (uint8_t)(0x5E ^ j);
    st = rt_write(buf_k_tx[0], ADDR_K3_R, SIZE_K);
    TEST_ASSERT(st == W25Q_OK, "Write zone lue: %s", rstr(st));
    for (uint32_t i = 0; i < SIZE_LARGE; i++) buf_large[i] = (uint8_t)(0xC7 ^ i ^ (i >> 8));

    R[0] = RT_NOT_SET;
    task_h_t hw = W25Q_Write_spawn(&(W25Q_Write_args_t){ .chip = w25q, .buf = buf_large, .addr = ADDR_K3_W,
                                                         .len = SIZE_LARGE, .timeout_ms = W25Q_TEST_TIMEOUT_MS },
                                   &(task_attr_t){ .priority = osPriorityNormal, .ret = &R[0], .join_bit = 1u });
    TEST_ASSERT(task_h_valid(hw), "Spawn Write refuse");

    uint32_t n_reads = 0, n_bad = 0;
    task_ret_t rd_err = W25Q_OK;
    while (task_running(hw) && n_reads < 50u) {
        memset(buf_k_rx, 0, SIZE_K);
        R[1] = RT_NOT_SET;
        task_h_t hr = W25Q_Read_spawn(&(W25Q_Read_args_t){ .chip = w25q, .buf = buf_k_rx, .addr = ADDR_K3_R,
                                                           .len = SIZE_K, .timeout_ms = W25Q_TEST_TIMEOUT_MS },
                                      &(task_attr_t){ .priority = osPriorityAboveNormal, .ret = &R[1], .join_bit = 2u });
        if (!task_h_valid(hr))                    { rd_err = RT_SPAWN_FAIL;   break; }
        if (task_join(hr, JOIN_TIMEOUT_MS) != osOK) { rd_err = RT_JOIN_TIMEOUT; break; }
        if (R[1] != W25Q_OK)                      { rd_err = R[1];            break; }
        uint32_t first_addr; uint8_t first_exp, first_got;
        if (count_mm(buf_k_tx[0], buf_k_rx, SIZE_K, ADDR_K3_R, &first_addr, &first_exp, &first_got) != 0u) n_bad++;
        n_reads++;
    }
    osStatus_t j = task_join(hw, JOIN_TIMEOUT_MS);

    TEST_ASSERT(rd_err == W25Q_OK, "Lecture %lu: %s", (unsigned long)n_reads, rstr(rd_err));
    TEST_ASSERT(n_reads >= 1u, "Ecriture finie avant la 1re lecture : test non concluant");
    TEST_ASSERT(n_bad == 0u, "%lu/%lu lectures corrompues", (unsigned long)n_bad, (unsigned long)n_reads);
    TEST_ASSERT(j == osOK && R[0] == W25Q_OK, "Write: join %d, ret %s", (int)j, rstr(R[0]));

    st = rt_read(buf_large_rx, ADDR_K3_W, SIZE_LARGE);
    TEST_ASSERT(st == W25Q_OK, "Read ecriture: %s", rstr(st));
    uint32_t first_addr; uint8_t first_exp, first_got;
    uint32_t mm = count_mm(buf_large, buf_large_rx, SIZE_LARGE, ADDR_K3_W, &first_addr, &first_exp, &first_got);
    TEST_ASSERT(mm == 0, "Ecriture: %lu mm @0x%06lX exp=0x%02X got=0x%02X",
                (unsigned long)mm, (unsigned long)first_addr, first_exp, first_got);
    PASS("%lu lectures exactes pendant l'ecriture de 17 pages", (unsigned long)n_reads);
}

W25Q_CASE(W25Q_rtos_test_t37_wait_shares_lock) {
    W25Q_STATE st = W25Q_SendCmdAddr_RTOS(w25q, W25Q_64KB_BLOCK_ERASE_4B, ADDR_BUSY64, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "Erase 64KB: %s", rstr(st));

    R[0] = RT_NOT_SET;
    task_h_t hp = W25QT_Poll_spawn(&(W25QT_Poll_args_t){ .chip = w25q, .timeout_ms = W25Q_TEST_TIMEOUT_MS },
                                   &(task_attr_t){ .priority = osPriorityNormal, .ret = &R[0], .join_bit = 1u });
    TEST_ASSERT(task_h_valid(hp), "Spawn Poll refuse");

    /* Meme priorite que le job : chaque attente DMA de l'un laisse passer l'autre. */
    uint32_t n_ok = 0, n_lock = 0, n_other = 0;
    bool busy_seen = false;
    while (task_running(hp) && (n_ok + n_lock + n_other) < 100000u) {
        W25Q_STATE s = W25Q_ReadStatus_RTOS(w25q, 1, 50u);
        if (s == W25Q_OK) {
            n_ok++;
            if (W25Q_STATUS_REG(w25q, W25Q_SR1_BUSY_BIT)) busy_seen = true;
        } else if (s == W25Q_LOCK_TIMEOUT) {
            n_lock++;
        } else {
            n_other++;
        }
    }
    osStatus_t j = task_join(hp, JOIN_TIMEOUT_MS);

    TEST_ASSERT(j == osOK && R[0] == W25Q_OK, "Job Poll: join %d, ret %s", (int)j, rstr(R[0]));
    TEST_ASSERT(n_lock == 0u && n_other == 0u, "ReadStatus: %lu LOCK_TIMEOUT, %lu autres erreurs",
                (unsigned long)n_lock, (unsigned long)n_other);
    TEST_ASSERT(n_ok >= 1u && busy_seen, "%lu lectures pendant l'attente, BUSY vu: %d",
                (unsigned long)n_ok, (int)busy_seen);
    PASS("%lu lectures de SR1 pendant le WaitForReady du job", (unsigned long)n_ok);
}


/* ======================= L. Ressources ================================= */

W25Q_CASE(W25Q_rtos_test_t38_stack_headroom) {
    uint32_t    min_free = UINT32_MAX;
    const char *min_name = "?";
    uint8_t     min_inst = 0;
    unsigned    n_meas   = 0;

    for (size_t p = 0; p < sizeof(w25q_pools) / sizeof(w25q_pools[0]); p++) {
        const task_desc_t *d = w25q_pools[p];
        for (uint8_t i = 0; i < d->n_instances; i++) {
            if (d->slots[i].id == NULL) continue;   /* instance jamais creee */
            uint32_t free_b = task_stack_headroom(d, i);
            n_meas++;
            if (free_b < min_free) { min_free = free_b; min_name = d->name; min_inst = i; }
        }
    }

    TEST_ASSERT(n_meas > 0u, "Aucune instance creee : lancer T38 apres les autres cas");
    TEST_ASSERT(min_free >= STACK_MARGIN_BYTES, "%s[%u] : %lu B libres < %u", min_name, min_inst,
                (unsigned long)min_free, (unsigned)STACK_MARGIN_BYTES);
    PASS("Min %lu B libres (%s[%u]), %u instances", (unsigned long)min_free, min_name, min_inst, n_meas);
}

#endif /* RTOS && PROFILE_TEST && W25Q512 */
