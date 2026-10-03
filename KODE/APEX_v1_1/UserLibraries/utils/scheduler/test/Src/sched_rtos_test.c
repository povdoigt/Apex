#include "sched_rtos_test.h"

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include <string.h>

/* ========================================================================
 * Table des cas de test
 * ======================================================================== */

TEST_case_table_t SCHED_rtos_test_cases[SCHED_rtos_test_N_TESTS] = {
    { .case_info = { .name = "T0 Spawn/join/ret"        }, .func = SCHED_rtos_test_t0_spawn_join            },
    { .case_info = { .name = "T1 Args copied"           }, .func = SCHED_rtos_test_t1_args_copied           },
    { .case_info = { .name = "T2 Lazy creation"         }, .func = SCHED_rtos_test_t2_lazy_creation         },
    { .case_info = { .name = "T3 Detached + ret"        }, .func = SCHED_rtos_test_t3_detached              },
    { .case_info = { .name = "T4 Priorities"            }, .func = SCHED_rtos_test_t4_priorities            },
    { .case_info = { .name = "T5 join_all"              }, .func = SCHED_rtos_test_t5_join_all              },
    { .case_info = { .name = "T6 Nested jobs"           }, .func = SCHED_rtos_test_t6_nested                },
    { .case_info = { .name = "T7 Prewarm"               }, .func = SCHED_rtos_test_t7_prewarm               },
    { .case_info = { .name = "T8 Busy count/headroom"   }, .func = SCHED_rtos_test_t8_busy_headroom         },
    { .case_info = { .name = "T9 Persistent"            }, .func = SCHED_rtos_test_t9_persistent            },
    { .case_info = { .name = "T10 Spawn rejects"        }, .func = SCHED_rtos_test_t10_spawn_rejects        },
    { .case_info = { .name = "T11 Join rejects"         }, .func = SCHED_rtos_test_t11_join_rejects         },
    { .case_info = { .name = "T12 Pool exhaustion"      }, .func = SCHED_rtos_test_t12_pool_exhaustion      },
    { .case_info = { .name = "T13 Stale handle"         }, .func = SCHED_rtos_test_t13_stale_handle         },
    { .case_info = { .name = "T14 Endurance x20000"     }, .func = SCHED_rtos_test_t14_endurance            },
    { .case_info = { .name = "T15 Join bit 30 (E1)"     }, .func = SCHED_rtos_test_t15_join_bit_30          },
    { .case_info = { .name = "T16 Join timeout (E2)"    }, .func = SCHED_rtos_test_t16_join_timeout         },
    { .case_info = { .name = "T17 Both done (E8)"       }, .func = SCHED_rtos_test_t17_both_done            },
    { .case_info = { .name = "T18 join_all retry (E4)"  }, .func = SCHED_rtos_test_t18_join_all_retry       },
    { .case_info = { .name = "T19 Join after reuse(E5)" }, .func = SCHED_rtos_test_t19_join_after_reuse     },
    { .case_info = { .name = "T20 Bit in flight (E6)"   }, .func = SCHED_rtos_test_t20_bit_in_flight        },
    { .case_info = { .name = "T21 Prio bounds (F3)"     }, .func = SCHED_rtos_test_t21_prio_bounds          },
    { .case_info = { .name = "T22 Wake no inversion"    }, .func = SCHED_rtos_test_t22_wake_no_inversion    },
    { .case_info = { .name = "T23 Spawn latency"        }, .func = SCHED_rtos_test_t23_spawn_latency        },
    { .case_info = { .name = "T24 Joiner sleeps"        }, .func = SCHED_rtos_test_t24_joiner_sleeps        },
    { .case_info = { .name = "T25 Concurrent spawners"  }, .func = SCHED_rtos_test_t25_concurrent_spawners  },
    { .case_info = { .name = "T26 ISR rejects (H1)"     }, .func = SCHED_rtos_test_t26_isr_rejects          },
};

/* ========================================================================
 * Etat partage
 *
 * Tout ce qu'un job ecrit (ret, sondes, compteurs) vit en statique, jamais
 * sur la pile du cas : un TEST_ASSERT en echec rend la main avant la fin des
 * jobs, qui ecriraient sinon dans une pile morte.
 * ======================================================================== */

/* Handler installe par T10 et T26 seulement : un mauvais usage est compte puis
   refuse, au lieu d'arreter le programme sur configASSERT. */
static volatile uint32_t misuse_count;

static void count_misuse(const char *what) {
    (void)what;
    misuse_count++;
}

#define GATE_MAX        8u
#define N_RET           6u
#define JOIN_BITS_MASK  0x7FFFFFFEu   /* bits 1..30 */

typedef struct {
    uint32_t     cyc;    /* DWT->CYCCNT a l'instant de la mesure */
    osPriority_t prio;   /* priorite vue par le corps            */
    osThreadId_t id;     /* thread qui a execute le corps        */
} probe_t;

static osSemaphoreId_t   gate_sem;
static StaticSemaphore_t gate_cb;

static task_ret_t        R[N_RET];
static volatile probe_t  probe;
static volatile uint32_t counter;
static volatile bool     counter_stop;

static inline uint32_t cyc_now(void) {
    return DWT->CYCCNT;
}

static inline uint32_t cyc_to_us(uint32_t cyc) {
    return cyc / (SystemCoreClock / 1000000u);
}

/* ========================================================================
 * Corps de test
 * ======================================================================== */

/* Echo : attend delay_ms puis rend value. */
typedef struct { int32_t value; uint32_t delay_ms; } SchedT_Echo_args_t;
TASK_DECLARE(SchedT_Echo, SchedT_Echo_args_t, 768);
TASK_DEFINE(SchedT_Echo) {
    if (args->delay_ms != 0u) {
        (void)osDelay(args->delay_ms);
    }
    return args->value;
}

/* Gate : bloque jusqu'a ce que le test libere gate_sem, puis rend value. */
typedef struct { int32_t value; } SchedT_Gate_args_t;
TASK_DECLARE(SchedT_Gate, SchedT_Gate_args_t, 768);
TASK_DEFINE(SchedT_Gate) {
    (void)osSemaphoreAcquire(gate_sem, osWaitForever);
    return args->value;
}

/* Probe : attend delay_ms, puis note l'instant, sa priorite et son thread.
   Plusieurs pools partagent ce corps pour disposer de slots vierges. */
typedef struct { volatile probe_t *out; uint32_t delay_ms; } SchedT_Probe_args_t;

static task_ret_t probe_body(const SchedT_Probe_args_t *a) {
    if (a->delay_ms != 0u) {
        (void)osDelay(a->delay_ms);
    }
    a->out->cyc  = cyc_now();
    a->out->prio = osThreadGetPriority(osThreadGetId());
    a->out->id   = osThreadGetId();
    return 0;
}

TASK_DECLARE(SchedT_Probe, SchedT_Probe_args_t, 768);
TASK_DEFINE(SchedT_Probe) { return probe_body(args); }

TASK_DECLARE(SchedT_Lazy, SchedT_Probe_args_t, 768);
TASK_DEFINE(SchedT_Lazy) { return probe_body(args); }

TASK_DECLARE(SchedT_Warm, SchedT_Probe_args_t, 768);
TASK_DEFINE(SchedT_Warm) { return probe_body(args); }

TASK_DECLARE(SchedT_Cold, SchedT_Probe_args_t, 768);
TASK_DEFINE(SchedT_Cold) { return probe_body(args); }

/* Spin : occupe le CPU pendant ms millisecondes, sans jamais bloquer. */
typedef struct { uint32_t ms; } SchedT_Spin_args_t;
TASK_DECLARE(SchedT_Spin, SchedT_Spin_args_t, 768);
TASK_DEFINE(SchedT_Spin) {
    TickType_t t0 = xTaskGetTickCount();
    while ((xTaskGetTickCount() - t0) < args->ms) {
    }
    return 0;
}

/* Forever : tache persistante, ne rend jamais la main. */
typedef struct { uint32_t period_ms; } SchedT_Forever_args_t;
TASK_DECLARE_PERSISTENT(SchedT_Forever, SchedT_Forever_args_t, 512);
TASK_DEFINE(SchedT_Forever) {
    for (;;) {
        (void)osDelay(args->period_ms);
    }
}

/* Nested : lance deux Echo sur ses propres bits 1 et 2, rend leur somme. */
typedef struct { int32_t a; int32_t b; } SchedT_Nested_args_t;
TASK_DECLARE(SchedT_Nested, SchedT_Nested_args_t, 1024);
TASK_DEFINE(SchedT_Nested) {
    task_ret_t ra = 0, rb = 0;
    task_h_t   h[2];

    h[0] = SchedT_Echo_spawn(&(SchedT_Echo_args_t){ .value = args->a, .delay_ms = 5u },
                             &(task_attr_t){ .priority = osPriorityNormal, .ret = &ra, .join_bit = 1u });
    h[1] = SchedT_Echo_spawn(&(SchedT_Echo_args_t){ .value = args->b, .delay_ms = 10u },
                             &(task_attr_t){ .priority = osPriorityNormal, .ret = &rb, .join_bit = 2u });

    /* Toujours joindre ce qui a ete lance : ra et rb sont sur cette pile. */
    if (!task_h_valid(h[0]) || !task_h_valid(h[1])) {
        if (task_h_valid(h[0])) { (void)task_join(h[0], osWaitForever); }
        if (task_h_valid(h[1])) { (void)task_join(h[1], osWaitForever); }
        return INT32_MIN;
    }
    if (task_join_all(h, 2u, osWaitForever) != osOK) {
        return INT32_MIN;
    }
    return ra + rb;
}

/* Foreign : tente de joindre, depuis son propre thread, un handle qui ne lui
   appartient pas ; rend le statut obtenu. */
typedef struct { task_h_t h; } SchedT_Foreign_args_t;
TASK_DECLARE(SchedT_Foreign, SchedT_Foreign_args_t, 768);
TASK_DEFINE(SchedT_Foreign) {
    return (task_ret_t)task_join(args->h, 0u);
}

/* Respawn : depuis son propre thread, lance un Gate et le joint. */
typedef struct { int32_t value; } SchedT_Respawn_args_t;
TASK_DECLARE(SchedT_Respawn, SchedT_Respawn_args_t, 1024);
TASK_DEFINE(SchedT_Respawn) {
    task_ret_t r = 0;
    task_h_t   h = SchedT_Gate_spawn(&(SchedT_Gate_args_t){ .value = args->value },
                                     &(task_attr_t){ .priority = osPriorityNormal, .ret = &r, .join_bit = 1u });
    if (!task_h_valid(h)) {
        return INT32_MIN;
    }
    (void)task_join(h, osWaitForever);   /* r est sur cette pile */
    return r;
}

/* Counter : incremente counter tant que counter_stop est faux. */
typedef struct { uint32_t unused; } SchedT_Counter_args_t;
TASK_DECLARE(SchedT_Counter, SchedT_Counter_args_t, 768);
TASK_DEFINE(SchedT_Counter) {
    while (!counter_stop) {
        counter++;
    }
    return 0;
}

/* Spammer : enchaine spawn/join d'Echo avec une valeur propre a ce spammer.
   Rend le nombre de ret faux, -1 si un spawn reste impossible, -2 si un join
   echoue. */
typedef struct { uint16_t id; uint16_t iters; } SchedT_Spammer_args_t;
TASK_DECLARE(SchedT_Spammer, SchedT_Spammer_args_t, 1024);
TASK_DEFINE(SchedT_Spammer) {
    int32_t bad = 0;

    for (uint32_t i = 0; i < args->iters; i++) {
        int32_t    value = ((int32_t)args->id << 16) | (int32_t)i;
        task_ret_t r     = -1;
        task_h_t   h     = TASK_H_INVALID;

        for (uint32_t tries = 0; tries < 100u && !task_h_valid(h); tries++) {
            h = SchedT_Echo_spawn(&(SchedT_Echo_args_t){ .value = value },
                                  &(task_attr_t){ .priority = osPriorityNormal, .ret = &r, .join_bit = 1u });
            if (!task_h_valid(h)) {
                (void)osDelay(1u);   /* pool plein : l'autre spammer l'occupe */
            }
        }
        if (!task_h_valid(h)) {
            return -1;
        }
        if (task_join(h, osWaitForever) != osOK) {
            return -2;
        }
        if (r != value) {
            bad++;
        }
    }
    return bad;
}

/* ========================================================================
 * Pools de la suite
 * ======================================================================== */

TASK_POOL(SchedT_Echo,    4);
TASK_POOL(SchedT_Gate,    3);
TASK_POOL(SchedT_Probe,   1);
TASK_POOL(SchedT_Lazy,    1);
TASK_POOL(SchedT_Warm,    2);
TASK_POOL(SchedT_Cold,    1);
TASK_POOL(SchedT_Spin,    1);
TASK_POOL(SchedT_Forever, 1);
TASK_POOL(SchedT_Nested,  1);
TASK_POOL(SchedT_Foreign, 1);
TASK_POOL(SchedT_Respawn, 1);
TASK_POOL(SchedT_Counter, 1);
TASK_POOL(SchedT_Spammer, 2);

/* Pools dont les jobs se terminent ; SchedT_Forever en est exclu. */
static const task_desc_t *const finite_pools[] = {
    &TASK_SchedT_Echo,    &TASK_SchedT_Gate,    &TASK_SchedT_Probe,
    &TASK_SchedT_Lazy,    &TASK_SchedT_Warm,    &TASK_SchedT_Cold,
    &TASK_SchedT_Spin,    &TASK_SchedT_Nested,  &TASK_SchedT_Foreign,
    &TASK_SchedT_Respawn, &TASK_SchedT_Counter, &TASK_SchedT_Spammer,
};

/* ========================================================================
 * Outils
 * ======================================================================== */

static uint32_t finite_busy(void) {
    uint32_t n = 0u;
    for (size_t i = 0; i < sizeof(finite_pools) / sizeof(finite_pools[0]); i++) {
        n += task_busy_count(finite_pools[i]);
    }
    return n;
}

/* Attend qu'un job soit fini, sans le joindre (son flag reste leve). */
static bool wait_done(task_h_t h, uint32_t ms) {
    for (uint32_t t = 0; t < ms && task_running(h); t++) {
        (void)osDelay(1u);
    }
    return !task_running(h);
}

/* Attend qu'un pool compte exactement n jobs. */
static bool wait_busy(const task_desc_t *desc, uint8_t n, uint32_t ms) {
    for (uint32_t t = 0; t < ms && task_busy_count(desc) != n; t++) {
        (void)osDelay(1u);
    }
    return task_busy_count(desc) == n;
}

/* Remet l'environnement a zero apres un cas, qu'il ait reussi ou non. */
static void sched_cleanup(TEST_case_t *tc) {
    counter_stop = true;
    for (uint32_t i = 0; i < GATE_MAX; i++) {
        (void)osSemaphoreRelease(gate_sem);
    }

    uint32_t t = 0;
    while (finite_busy() != 0u && t < 1000u) {
        (void)osDelay(1u);
        t++;
    }
    uint32_t left = finite_busy();

    while (osSemaphoreAcquire(gate_sem, 0u) == osOK) {
    }
    (void)osThreadFlagsClear(JOIN_BITS_MASK);
    (void)osThreadSetPriority(osThreadGetId(), osPriorityNormal);

    memset(R, 0, sizeof(R));
    probe.cyc    = 0u;
    probe.prio   = osPriorityNone;
    probe.id     = NULL;
    counter      = 0u;
    counter_stop = false;

    if (left != 0u && tc->result == R_PASS) {
        tc->result = R_FAIL;
        snprintf(tc->detail, sizeof(tc->detail),
                 "Nettoyage : %lu job(s) encore actifs apres 1 s", (unsigned long)left);
    }
}

/* Definit un cas : le corps ecrit avec TEST_ASSERT, suivi d'un nettoyage
   systematique, meme quand une assertion a rendu la main plus tot. */
#define SCHED_CASE(fn_)                                                        \
    static void fn_##_body(TEST_case_t *tc);                                   \
    void fn_(TEST_case_t *tc) {                                                \
        tc->result    = R_FAIL;                                                \
        tc->detail[0] = '\0';                                                  \
        fn_##_body(tc);                                                        \
        sched_cleanup(tc);                                                     \
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

void SCHED_rtos_test_init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT       = 0u;
    DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;

    if (gate_sem == NULL) {
        gate_sem = osSemaphoreNew(GATE_MAX, 0u, &(const osSemaphoreAttr_t){
            .name    = "sched_test_gate",
            .cb_mem  = &gate_cb,
            .cb_size = sizeof(gate_cb),
        });
    }
}

/* ========================================================================
 * T0 – Spawn, join, lecture de ret
 * ======================================================================== */
SCHED_CASE(SCHED_rtos_test_t0_spawn_join) {
    TEST_ASSERT(gate_sem != NULL, "SCHED_rtos_test_init() non appele ou echoue");

    task_h_t h = SchedT_Echo_spawn(&(SchedT_Echo_args_t){ .value = 42 },
                                   &(task_attr_t){ .priority = osPriorityNormal, .ret = &R[0], .join_bit = 1u });
    TEST_ASSERT(task_h_valid(h), "Spawn refuse");

    osStatus_t st = task_join(h, 100u);
    TEST_ASSERT(st == osOK, "task_join -> %d != osOK", (int)st);
    TEST_ASSERT(R[0] == 42, "ret = %ld != 42", (long)R[0]);
    TEST_ASSERT(!task_running(h), "task_running vrai apres join");

    st = task_join(h, 5u);
    TEST_ASSERT(st == osErrorTimeout, "2e join -> %d != osErrorTimeout", (int)st);

    PASS("ret 42 publie ; 2e join expire comme documente");
}

/* ========================================================================
 * T1 – Arguments copies au spawn
 * ======================================================================== */
SCHED_CASE(SCHED_rtos_test_t1_args_copied) {
    SchedT_Gate_args_t a = { .value = 7 };

    task_h_t h = SchedT_Gate_spawn(&a, &(task_attr_t){ .priority = osPriorityNormal, .ret = &R[0], .join_bit = 1u });
    TEST_ASSERT(task_h_valid(h), "Spawn refuse");

    a.value = 99;   /* le job est bloque sur la porte : il n'a encore rien lu */
    (void)osSemaphoreRelease(gate_sem);

    osStatus_t st = task_join(h, 100u);
    TEST_ASSERT(st == osOK, "task_join -> %d != osOK", (int)st);
    TEST_ASSERT(R[0] == 7, "ret = %ld != 7 (copie privee non utilisee)", (long)R[0]);

    PASS("Le job a lu sa copie (7), pas la variable modifiee (99)");
}

/* ========================================================================
 * T2 – Creation paresseuse
 * ======================================================================== */
SCHED_CASE(SCHED_rtos_test_t2_lazy_creation) {
    const task_slot_t *s = &TASK_SchedT_Lazy.slots[0];
    TEST_ASSERT(s->id == NULL, "Slot deja cree avant le 1er spawn (suite relancee ?)");

    task_h_t h = SchedT_Lazy_spawn(&(SchedT_Probe_args_t){ .out = &probe },
                                   &(task_attr_t){ .priority = osPriorityNormal, .join_bit = 1u });
    TEST_ASSERT(task_h_valid(h), "1er spawn refuse");
    TEST_ASSERT(task_join(h, 100u) == osOK, "1er join echoue");

    osThreadId_t id1 = s->id;
    TEST_ASSERT(id1 != NULL, "Thread non cree par le 1er spawn");
    TEST_ASSERT(probe.id == id1, "Corps execute par un autre thread que le slot");

    h = SchedT_Lazy_spawn(&(SchedT_Probe_args_t){ .out = &probe },
                          &(task_attr_t){ .priority = osPriorityNormal, .join_bit = 1u });
    TEST_ASSERT(task_h_valid(h), "2e spawn refuse");
    TEST_ASSERT(task_join(h, 100u) == osOK, "2e join echoue");
    TEST_ASSERT(s->id == id1 && probe.id == id1, "Thread recree au 2e spawn");

    PASS("Cree au 1er spawn, reutilise ensuite");
}

/* ========================================================================
 * T3 – Job detache avec ret
 * ======================================================================== */
SCHED_CASE(SCHED_rtos_test_t3_detached) {
    task_h_t h = SchedT_Echo_spawn(&(SchedT_Echo_args_t){ .value = 33, .delay_ms = 20u },
                                   &(task_attr_t){ .priority = osPriorityNormal, .ret = &R[0] });
    TEST_ASSERT(task_h_valid(h), "Spawn refuse");
    TEST_ASSERT(task_running(h), "task_running faux juste apres le spawn");
    TEST_ASSERT(wait_done(h, 200u), "Job detache toujours en cours apres 200 ms");
    TEST_ASSERT(R[0] == 33, "ret = %ld != 33", (long)R[0]);

    PASS("ret publie sans join, task_running retombe");
}

/* ========================================================================
 * T4 – Priorite de travail et de garage
 * ======================================================================== */
SCHED_CASE(SCHED_rtos_test_t4_priorities) {
    task_h_t h = SchedT_Probe_spawn(&(SchedT_Probe_args_t){ .out = &probe },
                                    &(task_attr_t){ .priority = osPriorityAboveNormal, .join_bit = 1u });
    TEST_ASSERT(task_h_valid(h), "Spawn AboveNormal refuse");
    TEST_ASSERT(task_join(h, 100u) == osOK, "Join AboveNormal echoue");
    TEST_ASSERT(probe.prio == osPriorityAboveNormal, "Corps a %d != AboveNormal", (int)probe.prio);

    osPriority_t parked = osThreadGetPriority(TASK_SchedT_Probe.slots[0].id);
    TEST_ASSERT(parked == TASK_PARKED_PRIORITY, "Garage a %d != %d", (int)parked, (int)TASK_PARKED_PRIORITY);

    h = SchedT_Probe_spawn(&(SchedT_Probe_args_t){ .out = &probe },
                           &(task_attr_t){ .priority = osPriorityBelowNormal, .join_bit = 1u });
    TEST_ASSERT(task_h_valid(h), "Spawn BelowNormal refuse");
    TEST_ASSERT(task_join(h, 100u) == osOK, "Join BelowNormal echoue");
    TEST_ASSERT(probe.prio == osPriorityBelowNormal, "Herite %d au lieu de BelowNormal", (int)probe.prio);

    PASS("AboveNormal puis garage %d puis BelowNormal", (int)TASK_PARKED_PRIORITY);
}

/* ========================================================================
 * T5 – task_join_all
 * ======================================================================== */
SCHED_CASE(SCHED_rtos_test_t5_join_all) {
    static const uint32_t delays[3] = { 30u, 10u, 20u };
    task_h_t h[3];

    for (uint32_t i = 0; i < 3u; i++) {
        h[i] = SchedT_Echo_spawn(&(SchedT_Echo_args_t){ .value = (int32_t)(10u * (i + 1u)), .delay_ms = delays[i] },
                                 &(task_attr_t){ .priority = osPriorityNormal, .ret = &R[i], .join_bit = (uint8_t)(i + 1u) });
        TEST_ASSERT(task_h_valid(h[i]), "Spawn %lu refuse", (unsigned long)i);
    }

    osStatus_t st = task_join_all(h, 3u, 200u);
    TEST_ASSERT(st == osOK, "task_join_all -> %d != osOK", (int)st);
    TEST_ASSERT(R[0] == 10 && R[1] == 20 && R[2] == 30,
                "ret = %ld/%ld/%ld != 10/20/30", (long)R[0], (long)R[1], (long)R[2]);

    PASS("3 jobs (30/10/20 ms) joints ensemble");
}

/* ========================================================================
 * T6 – Sous-jobs
 * ======================================================================== */
SCHED_CASE(SCHED_rtos_test_t6_nested) {
    task_h_t h = SchedT_Nested_spawn(&(SchedT_Nested_args_t){ .a = 5, .b = 7 },
                                     &(task_attr_t){ .priority = osPriorityNormal, .ret = &R[0], .join_bit = 1u });
    TEST_ASSERT(task_h_valid(h), "Spawn refuse");

    osStatus_t st = task_join(h, 300u);
    TEST_ASSERT(st == osOK, "task_join -> %d != osOK", (int)st);
    TEST_ASSERT(R[0] == 12, "ret = %ld != 12 (INT32_MIN = sous-job en echec)", (long)R[0]);

    PASS("Le job a joint ses 2 sous-jobs : 5 + 7 = 12");
}

/* ========================================================================
 * T7 – task_prewarm
 * ======================================================================== */
SCHED_CASE(SCHED_rtos_test_t7_prewarm) {
    const task_desc_t *d = &TASK_SchedT_Warm;

    osStatus_t st = task_prewarm(d);
    TEST_ASSERT(st == osOK, "task_prewarm -> %d != osOK", (int)st);
    TEST_ASSERT(d->slots[0].id != NULL && d->slots[1].id != NULL, "Instance non creee");
    TEST_ASSERT(task_busy_count(d) == 0u, "busy_count = %u apres prewarm", task_busy_count(d));

    osPriority_t p = osThreadGetPriority(d->slots[0].id);
    TEST_ASSERT(p == TASK_PARKED_PRIORITY, "Prechauffe a %d != garage", (int)p);

    osThreadId_t id0 = d->slots[0].id;
    st = task_prewarm(d);
    TEST_ASSERT(st == osOK && d->slots[0].id == id0, "2e prewarm a recree un thread");

    task_h_t h = SchedT_Warm_spawn(&(SchedT_Probe_args_t){ .out = &probe },
                                   &(task_attr_t){ .priority = osPriorityNormal, .join_bit = 1u });
    TEST_ASSERT(task_h_valid(h), "Spawn refuse");
    TEST_ASSERT(task_join(h, 100u) == osOK, "Join echoue");
    TEST_ASSERT(probe.id == id0, "Spawn n'a pas utilise le thread prechauffe");

    st = task_prewarm(NULL);
    TEST_ASSERT(st == osErrorParameter, "task_prewarm(NULL) -> %d", (int)st);

    PASS("2 instances creees au garage, reutilisees");
}

/* ========================================================================
 * T8 – task_busy_count et task_stack_headroom
 * ======================================================================== */
SCHED_CASE(SCHED_rtos_test_t8_busy_headroom) {
    const task_desc_t *d = &TASK_SchedT_Gate;
    task_h_t h[2];

    for (uint32_t i = 0; i < 2u; i++) {
        h[i] = SchedT_Gate_spawn(&(SchedT_Gate_args_t){ .value = (int32_t)i },
                                 &(task_attr_t){ .priority = osPriorityNormal, .ret = &R[i], .join_bit = (uint8_t)(i + 1u) });
        TEST_ASSERT(task_h_valid(h[i]), "Spawn %lu refuse", (unsigned long)i);
    }
    TEST_ASSERT(task_busy_count(d) == 2u, "busy_count = %u != 2", task_busy_count(d));

    uint32_t hr = task_stack_headroom(d, 0u);
    TEST_ASSERT(hr > 0u && hr < d->stack_bytes, "headroom = %lu hors ]0, %lu[",
                (unsigned long)hr, (unsigned long)d->stack_bytes);
    TEST_ASSERT(task_stack_headroom(d, d->n_instances) == 0u, "headroom hors bornes != 0");
    TEST_ASSERT(task_stack_headroom(NULL, 0u) == 0u, "headroom(NULL) != 0");
    TEST_ASSERT(task_busy_count(NULL) == 0u, "busy_count(NULL) != 0");

    (void)osSemaphoreRelease(gate_sem);
    (void)osSemaphoreRelease(gate_sem);
    TEST_ASSERT(task_join_all(h, 2u, 100u) == osOK, "join_all echoue");
    TEST_ASSERT(task_busy_count(d) == 0u, "busy_count = %u != 0 apres join", task_busy_count(d));

    PASS("busy 2 -> 0 ; marge pile slot 0 : %lu/%lu o", (unsigned long)hr, (unsigned long)d->stack_bytes);
}

/* ========================================================================
 * T9 – Tache persistante
 * ======================================================================== */
SCHED_CASE(SCHED_rtos_test_t9_persistent) {
    const task_desc_t *d = &TASK_SchedT_Forever;
    TEST_ASSERT(task_busy_count(d) == 0u, "Tache persistante deja lancee (suite relancee ?)");

    task_h_t h = SchedT_Forever_spawn(&(SchedT_Forever_args_t){ .period_ms = 100u },
                                      &(task_attr_t){ .priority = osPriorityBelowNormal });
    TEST_ASSERT(task_h_valid(h), "Spawn detache refuse");

    (void)osDelay(250u);
    TEST_ASSERT(task_running(h), "La tache persistante s'est arretee");
    TEST_ASSERT(task_busy_count(d) == 1u, "busy_count = %u != 1", task_busy_count(d));

    task_h_t h2 = SchedT_Forever_spawn(&(SchedT_Forever_args_t){ .period_ms = 100u },
                                       &(task_attr_t){ .priority = osPriorityBelowNormal });
    TEST_ASSERT(!task_h_valid(h2), "2e spawn accepte sur un pool de 1 occupe");

    PASS("Slot occupe en permanence, 2e spawn refuse");
}

/* ========================================================================
 * T10 – Spawns refuses
 * ======================================================================== */
SCHED_CASE(SCHED_rtos_test_t10_spawn_rejects) {
    const task_desc_t *d  = &TASK_SchedT_Echo;
    SchedT_Echo_args_t a  = { .value = 1 };
    const task_attr_t  ok = { .priority = osPriorityNormal };

    misuse_count = 0u;
    task_set_misuse_handler(count_misuse);
    TEST_ASSERT(!task_h_valid(task_spawn_(NULL, &a, &ok)), "desc NULL accepte");
    TEST_ASSERT(!task_h_valid(task_spawn_(d, &a, NULL)), "attr NULL accepte");
    TEST_ASSERT(!task_h_valid(SchedT_Echo_spawn(NULL, &ok)), "args NULL accepte (D5)");

    TEST_ASSERT(!task_h_valid(SchedT_Echo_spawn(&a, &(task_attr_t){ .priority = osPriorityNone })),
                "osPriorityNone accepte");
    TEST_ASSERT(!task_h_valid(SchedT_Echo_spawn(&a, &(task_attr_t){ .priority = osPriorityError })),
                "osPriorityError accepte");
    TEST_ASSERT(!task_h_valid(SchedT_Echo_spawn(&a, &(task_attr_t){ .priority = osPriorityISR })),
                "osPriorityISR (56) accepte (F3)");

    TEST_ASSERT(!task_h_valid(SchedT_Echo_spawn(&a, &(task_attr_t){ .priority = osPriorityNormal, .join_bit = 31u })),
                "join_bit 31 accepte (E1)");
    TEST_ASSERT(!task_h_valid(SchedT_Echo_spawn(&a, &(task_attr_t){ .priority = osPriorityNormal, .join_bit = 255u })),
                "join_bit 255 accepte");
    TEST_ASSERT(!task_h_valid(SchedT_Forever_spawn(&(SchedT_Forever_args_t){ .period_ms = 10u },
                                                   &(task_attr_t){ .priority = osPriorityNormal, .join_bit = 1u })),
                "join_bit sur tache persistante accepte");

    task_set_misuse_handler(NULL);
    TEST_ASSERT(misuse_count == 9u, "%lu mauvais usages signales au lieu de 9", (unsigned long)misuse_count);
    TEST_ASSERT(task_busy_count(d) == 0u, "Un refus a consomme un slot (busy %u)", task_busy_count(d));

    /* Aucun refus ne doit avoir laisse le bit 1 marque comme utilise. */
    task_h_t h = SchedT_Echo_spawn(&a, &(task_attr_t){ .priority = osPriorityNormal, .ret = &R[0], .join_bit = 1u });
    TEST_ASSERT(task_h_valid(h), "Bit 1 bloque apres les refus");
    TEST_ASSERT(task_join(h, 100u) == osOK && R[0] == 1, "Spawn valide apres refus en echec");

    PASS("9 spawns invalides refuses, aucun slot ni bit consomme");
}

/* ========================================================================
 * T11 – Joins refuses
 * ======================================================================== */
SCHED_CASE(SCHED_rtos_test_t11_join_rejects) {
    osStatus_t st = task_join(TASK_H_INVALID, 0u);
    TEST_ASSERT(st == osErrorParameter, "join(INVALID) -> %d", (int)st);

    task_h_t hd = SchedT_Echo_spawn(&(SchedT_Echo_args_t){ .value = 1, .delay_ms = 10u },
                                    &(task_attr_t){ .priority = osPriorityNormal });
    TEST_ASSERT(task_h_valid(hd), "Spawn detache refuse");
    st = task_join(hd, 0u);
    TEST_ASSERT(st == osErrorParameter, "join(detache) -> %d", (int)st);

    /* Join depuis un autre thread que le spawner. */
    task_h_t hg = SchedT_Gate_spawn(&(SchedT_Gate_args_t){ .value = 5 },
                                    &(task_attr_t){ .priority = osPriorityNormal, .ret = &R[0], .join_bit = 1u });
    TEST_ASSERT(task_h_valid(hg), "Spawn Gate refuse");

    task_h_t hf = SchedT_Foreign_spawn(&(SchedT_Foreign_args_t){ .h = hg },
                                       &(task_attr_t){ .priority = osPriorityNormal, .ret = &R[1], .join_bit = 2u });
    TEST_ASSERT(task_h_valid(hf), "Spawn Foreign refuse");
    TEST_ASSERT(task_join(hf, 100u) == osOK, "Join Foreign echoue");
    TEST_ASSERT(R[1] == (task_ret_t)osErrorParameter, "join etranger -> %ld != osErrorParameter", (long)R[1]);

    /* join_all : entrees invalides. */
    task_h_t dup[2] = { hg, hg };
    task_h_t mix[2] = { hg, hd };
    TEST_ASSERT(task_join_all(NULL, 1u, 0u) == osErrorParameter, "join_all(NULL) accepte");
    TEST_ASSERT(task_join_all(dup, 0u, 0u) == osErrorParameter, "join_all(count 0) accepte");
    TEST_ASSERT(task_join_all(dup, 2u, 0u) == osErrorParameter, "join_all(bit duplique) accepte");
    TEST_ASSERT(task_join_all(mix, 2u, 0u) == osErrorParameter, "join_all(detache) accepte");

    (void)osSemaphoreRelease(gate_sem);
    st = task_join(hg, 100u);
    TEST_ASSERT(st == osOK && R[0] == 5, "Le vrai joiner n'obtient pas le job (%d, ret %ld)", (int)st, (long)R[0]);

    PASS("INVALID, detache, etranger et 4 join_all invalides refuses");
}

/* ========================================================================
 * T12 – Pool epuise
 * ======================================================================== */
SCHED_CASE(SCHED_rtos_test_t12_pool_exhaustion) {
    const task_desc_t *d = &TASK_SchedT_Gate;
    task_h_t h[4];

    for (uint32_t i = 0; i < 3u; i++) {
        h[i] = SchedT_Gate_spawn(&(SchedT_Gate_args_t){ .value = (int32_t)(i + 1u) },
                                 &(task_attr_t){ .priority = osPriorityNormal, .ret = &R[i], .join_bit = (uint8_t)(i + 1u) });
        TEST_ASSERT(task_h_valid(h[i]), "Spawn %lu refuse avant saturation", (unsigned long)i);
    }

    uint32_t   t0   = cyc_now();
    task_h_t   over = SchedT_Gate_spawn(&(SchedT_Gate_args_t){ .value = 99 },
                                        &(task_attr_t){ .priority = osPriorityNormal, .ret = &R[3], .join_bit = 4u });
    uint32_t   us   = cyc_to_us(cyc_now() - t0);
    TEST_ASSERT(!task_h_valid(over), "Spawn accepte sur pool plein");
    TEST_ASSERT(us < 100u, "Refus en %lu us : le spawn a bloque", (unsigned long)us);
    TEST_ASSERT(task_busy_count(d) == 3u, "busy_count = %u != 3", task_busy_count(d));

    (void)osSemaphoreRelease(gate_sem);
    TEST_ASSERT(wait_busy(d, 2u, 100u), "Aucun slot libere apres ouverture de la porte");

    h[3] = SchedT_Gate_spawn(&(SchedT_Gate_args_t){ .value = 4 },
                             &(task_attr_t){ .priority = osPriorityNormal, .ret = &R[3], .join_bit = 4u });
    TEST_ASSERT(task_h_valid(h[3]), "Spawn refuse apres liberation d'un slot");

    for (uint32_t i = 0; i < 3u; i++) {
        (void)osSemaphoreRelease(gate_sem);
    }
    osStatus_t st = task_join_all(h, 4u, 200u);
    TEST_ASSERT(st == osOK, "join_all -> %d != osOK", (int)st);
    TEST_ASSERT(R[0] == 1 && R[1] == 2 && R[2] == 3 && R[3] == 4,
                "ret = %ld/%ld/%ld/%ld", (long)R[0], (long)R[1], (long)R[2], (long)R[3]);

    PASS("4e spawn refuse en %lu us, accepte apres liberation", (unsigned long)us);
}

/* ========================================================================
 * T13 – Handle perime
 * ======================================================================== */
SCHED_CASE(SCHED_rtos_test_t13_stale_handle) {
    task_h_t h1 = SchedT_Echo_spawn(&(SchedT_Echo_args_t){ .value = 1 },
                                    &(task_attr_t){ .priority = osPriorityNormal, .ret = &R[0], .join_bit = 1u });
    TEST_ASSERT(task_h_valid(h1) && task_join(h1, 100u) == osOK, "1er job en echec");

    task_h_t h2 = SchedT_Echo_spawn(&(SchedT_Echo_args_t){ .value = 2, .delay_ms = 20u },
                                    &(task_attr_t){ .priority = osPriorityNormal, .ret = &R[1], .join_bit = 2u });
    TEST_ASSERT(task_h_valid(h2), "2e spawn refuse");
    TEST_ASSERT(h2.slot == h1.slot, "Le 2e job n'a pas repris le meme slot");
    TEST_ASSERT(task_running(h2), "Nouveau job non vu en cours");
    TEST_ASSERT(!task_running(h1), "Handle perime vu en cours");
    TEST_ASSERT(task_join(h2, 100u) == osOK && R[1] == 2, "2e job en echec");

    PASS("Meme slot, generations %lu -> %lu", (unsigned long)h1.gen, (unsigned long)h2.gen);
}

/* ========================================================================
 * T14 – Endurance
 * ======================================================================== */
SCHED_CASE(SCHED_rtos_test_t14_endurance) {
    const uint32_t n  = 20000u;
    uint32_t       t0 = osKernelGetTickCount();

    for (uint32_t i = 0; i < n; i++) {
        task_h_t h = SchedT_Echo_spawn(&(SchedT_Echo_args_t){ .value = (int32_t)i },
                                       &(task_attr_t){ .priority = osPriorityNormal, .ret = &R[0], .join_bit = 1u });
        TEST_ASSERT(task_h_valid(h), "Spawn refuse au cycle %lu", (unsigned long)i);
        osStatus_t st = task_join(h, 50u);
        TEST_ASSERT(st == osOK, "Join -> %d au cycle %lu", (int)st, (unsigned long)i);
        TEST_ASSERT(R[0] == (task_ret_t)i, "ret = %ld au cycle %lu", (long)R[0], (unsigned long)i);
    }

    uint32_t ms = osKernelGetTickCount() - t0;
    TEST_ASSERT(task_busy_count(&TASK_SchedT_Echo) == 0u, "Slot qui fuit apres endurance");

    PASS("%lu cycles en %lu ms (%lu us/cycle)", (unsigned long)n, (unsigned long)ms,
         (unsigned long)((ms * 1000u) / n));
}

/* ========================================================================
 * T15 – Bit de join 30 (E1)
 * ======================================================================== */
SCHED_CASE(SCHED_rtos_test_t15_join_bit_30) {
    task_h_t h = SchedT_Echo_spawn(&(SchedT_Echo_args_t){ .value = 30, .delay_ms = 5u },
                                   &(task_attr_t){ .priority = osPriorityNormal, .ret = &R[0], .join_bit = TASK_JOIN_BIT_MAX });
    TEST_ASSERT(task_h_valid(h), "Spawn sur le bit %u refuse", TASK_JOIN_BIT_MAX);

    osStatus_t st = task_join(h, 100u);
    TEST_ASSERT(st == osOK, "Join bit 30 -> %d != osOK", (int)st);
    TEST_ASSERT(R[0] == 30, "ret = %ld != 30", (long)R[0]);

    PASS("Bit 30 joint normalement (31 refuse en T10)");
}

/* ========================================================================
 * T16 – Timeout 0 et timeout court (E2)
 * ======================================================================== */
SCHED_CASE(SCHED_rtos_test_t16_join_timeout) {
    task_h_t h = SchedT_Gate_spawn(&(SchedT_Gate_args_t){ .value = 16 },
                                   &(task_attr_t){ .priority = osPriorityNormal, .ret = &R[0], .join_bit = 1u });
    TEST_ASSERT(task_h_valid(h), "Spawn refuse");

    osStatus_t st = task_join(h, 0u);
    TEST_ASSERT(st == osErrorTimeout, "join(h, 0) -> %d != osErrorTimeout", (int)st);

    uint32_t t0 = osKernelGetTickCount();
    st          = task_join(h, 20u);
    uint32_t dt = osKernelGetTickCount() - t0;
    TEST_ASSERT(st == osErrorTimeout, "join(h, 20) -> %d != osErrorTimeout", (int)st);
    TEST_ASSERT(dt >= 20u && dt <= 22u, "join(h, 20) a dure %lu ms", (unsigned long)dt);

    (void)osSemaphoreRelease(gate_sem);
    st = task_join(h, 100u);
    TEST_ASSERT(st == osOK && R[0] == 16, "Join final -> %d, ret %ld", (int)st, (long)R[0]);

    PASS("Timeout 0 et 20 ms (%lu ms mesures) -> osErrorTimeout", (unsigned long)dt);
}

/* ========================================================================
 * T17 – Deux jobs finis avant le premier join (E8)
 * ======================================================================== */
SCHED_CASE(SCHED_rtos_test_t17_both_done) {
    task_h_t a = SchedT_Echo_spawn(&(SchedT_Echo_args_t){ .value = 1 },
                                   &(task_attr_t){ .priority = osPriorityNormal, .ret = &R[0], .join_bit = 1u });
    task_h_t b = SchedT_Echo_spawn(&(SchedT_Echo_args_t){ .value = 2 },
                                   &(task_attr_t){ .priority = osPriorityNormal, .ret = &R[1], .join_bit = 2u });
    TEST_ASSERT(task_h_valid(a) && task_h_valid(b), "Spawn refuse");
    TEST_ASSERT(wait_done(a, 50u) && wait_done(b, 50u), "Jobs non termines");

    uint32_t   t0 = cyc_now();
    osStatus_t sa = task_join(a, 50u);
    osStatus_t sb = task_join(b, 50u);
    uint32_t   us = cyc_to_us(cyc_now() - t0);

    TEST_ASSERT(sa == osOK, "join(A) -> %d", (int)sa);
    TEST_ASSERT(sb == osOK, "join(B) -> %d (bloque malgre son bit leve)", (int)sb);
    TEST_ASSERT(us < 1000u, "Les 2 joins ont pris %lu us", (unsigned long)us);
    TEST_ASSERT(R[0] == 1 && R[1] == 2, "ret = %ld/%ld", (long)R[0], (long)R[1]);

    PASS("2 joins immediats en %lu us", (unsigned long)us);
}

/* ========================================================================
 * T18 – join_all relance apres timeout (E4)
 * ======================================================================== */
SCHED_CASE(SCHED_rtos_test_t18_join_all_retry) {
    task_h_t h[2];
    h[0] = SchedT_Echo_spawn(&(SchedT_Echo_args_t){ .value = 1 },
                             &(task_attr_t){ .priority = osPriorityNormal, .ret = &R[0], .join_bit = 1u });
    h[1] = SchedT_Gate_spawn(&(SchedT_Gate_args_t){ .value = 2 },
                             &(task_attr_t){ .priority = osPriorityNormal, .ret = &R[1], .join_bit = 2u });
    TEST_ASSERT(task_h_valid(h[0]) && task_h_valid(h[1]), "Spawn refuse");
    TEST_ASSERT(wait_done(h[0], 50u), "Job A non termine");

    osStatus_t st = task_join_all(h, 2u, 20u);
    TEST_ASSERT(st == osErrorTimeout, "1er join_all -> %d != osErrorTimeout", (int)st);

    (void)osSemaphoreRelease(gate_sem);
    st = task_join_all(h, 2u, 100u);
    TEST_ASSERT(st == osOK, "2e join_all -> %d (bit de A perdu ?)", (int)st);
    TEST_ASSERT(R[0] == 1 && R[1] == 2, "ret = %ld/%ld", (long)R[0], (long)R[1]);

    PASS("Timeout puis relance reussie");
}

/* ========================================================================
 * T19 – Join apres reutilisation du slot par un autre thread (E5)
 * ======================================================================== */
SCHED_CASE(SCHED_rtos_test_t19_join_after_reuse) {
    const task_desc_t *d = &TASK_SchedT_Gate;

    task_h_t a = SchedT_Gate_spawn(&(SchedT_Gate_args_t){ .value = 1 },
                                   &(task_attr_t){ .priority = osPriorityNormal, .ret = &R[0], .join_bit = 1u });
    TEST_ASSERT(task_h_valid(a), "Spawn A refuse");
    (void)osSemaphoreRelease(gate_sem);
    TEST_ASSERT(wait_done(a, 50u), "Job A non termine");

    /* Un autre thread reprend le slot de A et attend son propre job. */
    task_h_t r = SchedT_Respawn_spawn(&(SchedT_Respawn_args_t){ .value = 2 },
                                      &(task_attr_t){ .priority = osPriorityAboveNormal, .ret = &R[1], .join_bit = 2u });
    TEST_ASSERT(task_h_valid(r), "Spawn Respawn refuse");
    TEST_ASSERT(wait_busy(d, 1u, 50u), "Le slot n'a pas ete repris");
    TEST_ASSERT(d->slots[0].busy && a.slot == &d->slots[0], "Reprise sur un autre slot que celui de A");

    osStatus_t st = task_join(a, 10u);
    TEST_ASSERT(st == osOK, "join(A) -> %d au lieu de osOK", (int)st);
    TEST_ASSERT(R[0] == 1, "ret A = %ld", (long)R[0]);

    (void)osSemaphoreRelease(gate_sem);
    st = task_join(r, 200u);
    TEST_ASSERT(st == osOK && R[1] == 2, "Respawn -> %d, ret %ld", (int)st, (long)R[1]);

    PASS("join(A) reussi alors que son slot sert a un autre thread");
}

/* ========================================================================
 * T20 – Bit encore utilise (E6)
 * ======================================================================== */
SCHED_CASE(SCHED_rtos_test_t20_bit_in_flight) {
    task_h_t a = SchedT_Gate_spawn(&(SchedT_Gate_args_t){ .value = 11 },
                                   &(task_attr_t){ .priority = osPriorityNormal, .ret = &R[0], .join_bit = 1u });
    TEST_ASSERT(task_h_valid(a), "Spawn A refuse");

    osStatus_t st = task_join(a, 5u);
    TEST_ASSERT(st == osErrorTimeout, "join(A, 5) -> %d", (int)st);

    task_h_t b = SchedT_Echo_spawn(&(SchedT_Echo_args_t){ .value = 22, .delay_ms = 10u },
                                   &(task_attr_t){ .priority = osPriorityNormal, .ret = &R[1], .join_bit = 1u });
    TEST_ASSERT(!task_h_valid(b), "Respawn accepte sur un bit encore utilise");

    (void)osSemaphoreRelease(gate_sem);
    TEST_ASSERT(wait_done(a, 50u), "Job A non termine");
    TEST_ASSERT(R[0] == 11, "ret A = %ld", (long)R[0]);

    b = SchedT_Echo_spawn(&(SchedT_Echo_args_t){ .value = 22, .delay_ms = 10u },
                          &(task_attr_t){ .priority = osPriorityNormal, .ret = &R[1], .join_bit = 1u });
    TEST_ASSERT(task_h_valid(b), "Respawn refuse alors que A est fini");

    /* A est fini et jamais joint, B a repris son bit : joindre A attendrait B. */
    st = task_join(a, 0u);
    TEST_ASSERT(st == osErrorResource, "join(A perime) -> %d != osErrorResource", (int)st);

    st = task_join(b, 100u);
    TEST_ASSERT(st == osOK, "join(B) -> %d", (int)st);
    TEST_ASSERT(!task_running(b), "join(B) rendu avant la fin de B (flag de A)");
    TEST_ASSERT(R[1] == 22, "ret B = %ld", (long)R[1]);

    PASS("Bit refuse pendant A, join(B) attend bien B");
}

/* ========================================================================
 * T21 – Bornes de priorite (F3)
 * ======================================================================== */
SCHED_CASE(SCHED_rtos_test_t21_prio_bounds) {
    for (uint32_t i = 0; i < 2u; i++) {
        task_h_t h = SchedT_Probe_spawn(&(SchedT_Probe_args_t){ .out = &probe },
                                        &(task_attr_t){ .priority = osPriorityRealtime7, .join_bit = 1u });
        TEST_ASSERT(task_h_valid(h), "Spawn Realtime7 n%lu refuse", (unsigned long)(i + 1u));
        TEST_ASSERT(task_join(h, 100u) == osOK, "Join Realtime7 n%lu echoue", (unsigned long)(i + 1u));
        TEST_ASSERT(probe.prio == osPriorityRealtime7, "Corps a %d != Realtime7", (int)probe.prio);
    }

    task_h_t h = SchedT_Probe_spawn(&(SchedT_Probe_args_t){ .out = &probe },
                                    &(task_attr_t){ .priority = osPriorityIdle, .join_bit = 1u });
    TEST_ASSERT(task_h_valid(h), "Spawn Idle refuse");
    TEST_ASSERT(task_join(h, 100u) == osOK, "Join Idle echoue");
    TEST_ASSERT(probe.prio == osPriorityIdle, "Corps a %d != Idle", (int)probe.prio);

    PASS("Realtime7 x2 (slot deja cree) et Idle acceptes");
}

/* ========================================================================
 * T22 – Reveil du joiner sans inversion (F1)
 * ======================================================================== */
SCHED_CASE(SCHED_rtos_test_t22_wake_no_inversion) {
    (void)osThreadSetPriority(osThreadGetId(), osPriorityHigh);

    /* Job Realtime : 10 ms de sommeil, puis note l'instant de sa fin. */
    task_h_t hw = SchedT_Probe_spawn(&(SchedT_Probe_args_t){ .out = &probe, .delay_ms = 10u },
                                     &(task_attr_t){ .priority = osPriorityRealtime, .join_bit = 1u });
    /* Thread AboveNormal qui monopolise le CPU pendant 50 ms. */
    task_h_t hs = SchedT_Spin_spawn(&(SchedT_Spin_args_t){ .ms = 50u },
                                    &(task_attr_t){ .priority = osPriorityAboveNormal, .join_bit = 2u });
    TEST_ASSERT(task_h_valid(hw) && task_h_valid(hs), "Spawn refuse");

    osStatus_t st   = task_join(hw, 200u);
    uint32_t   woke = cyc_now();
    TEST_ASSERT(st == osOK, "join(job) -> %d", (int)st);

    uint32_t us = cyc_to_us(woke - probe.cyc);
    TEST_ASSERT(task_join(hs, 200u) == osOK, "join(spin) echoue");
    TEST_ASSERT(us < 1000u, "Joiner reveille %lu us apres la fin du job (inversion)", (unsigned long)us);

    PASS("Joiner reveille %lu us apres la fin du job", (unsigned long)us);
}

/* ========================================================================
 * T23 – Latence spawn -> premiere instruction (F2)
 * ======================================================================== */
SCHED_CASE(SCHED_rtos_test_t23_spawn_latency) {
    TEST_ASSERT(TASK_SchedT_Cold.slots[0].id == NULL, "Pool Cold deja cree (suite relancee ?)");

    /* Corps AboveNormal : il preempte le test des que le garage s'ouvre. */
    uint32_t t0 = cyc_now();
    task_h_t h  = SchedT_Cold_spawn(&(SchedT_Probe_args_t){ .out = &probe },
                                    &(task_attr_t){ .priority = osPriorityAboveNormal, .join_bit = 1u });
    TEST_ASSERT(task_h_valid(h), "Spawn a froid refuse");
    uint32_t cold_us = cyc_to_us(probe.cyc - t0);
    TEST_ASSERT(task_join(h, 100u) == osOK, "Join a froid echoue");

    uint32_t min_us = UINT32_MAX, max_us = 0u;
    for (uint32_t i = 0; i < 100u; i++) {
        t0 = cyc_now();
        h  = SchedT_Cold_spawn(&(SchedT_Probe_args_t){ .out = &probe },
                               &(task_attr_t){ .priority = osPriorityAboveNormal, .join_bit = 1u });
        TEST_ASSERT(task_h_valid(h), "Spawn a chaud refuse");
        uint32_t us = cyc_to_us(probe.cyc - t0);
        TEST_ASSERT(task_join(h, 100u) == osOK, "Join a chaud echoue");
        if (us < min_us) { min_us = us; }
        if (us > max_us) { max_us = us; }
    }
    TEST_ASSERT(max_us < 100u, "Chaud max %lu us >= 100 us", (unsigned long)max_us);

    PASS("Froid %lu us, chaud %lu..%lu us", (unsigned long)cold_us, (unsigned long)min_us, (unsigned long)max_us);
}

/* ========================================================================
 * T24 – Le joiner dort vraiment
 * ======================================================================== */
SCHED_CASE(SCHED_rtos_test_t24_joiner_sleeps) {
    /* BelowNormal : ne tourne que si le test (Normal) est bloque. */
    task_h_t hc = SchedT_Counter_spawn(&(SchedT_Counter_args_t){ 0 },
                                       &(task_attr_t){ .priority = osPriorityBelowNormal, .join_bit = 2u });
    task_h_t he = SchedT_Echo_spawn(&(SchedT_Echo_args_t){ .value = 24, .delay_ms = 100u },
                                    &(task_attr_t){ .priority = osPriorityNormal, .ret = &R[0], .join_bit = 1u });
    TEST_ASSERT(task_h_valid(hc) && task_h_valid(he), "Spawn refuse");

    uint32_t   c0 = counter;
    osStatus_t st = task_join(he, 300u);
    uint32_t   c1 = counter;

    counter_stop = true;
    TEST_ASSERT(task_join(hc, 100u) == osOK, "Compteur non arrete");
    TEST_ASSERT(st == osOK && R[0] == 24, "join -> %d, ret %ld", (int)st, (long)R[0]);
    TEST_ASSERT((c1 - c0) > 100000u, "Compteur +%lu seulement : le joiner n'a pas dormi",
                (unsigned long)(c1 - c0));

    PASS("Compteur basse priorite +%lu pendant le join de 100 ms", (unsigned long)(c1 - c0));
}

/* ========================================================================
 * T25 – Spawners concurrents (G1)
 * ======================================================================== */
SCHED_CASE(SCHED_rtos_test_t25_concurrent_spawners) {
    task_h_t h[2];
    h[0] = SchedT_Spammer_spawn(&(SchedT_Spammer_args_t){ .id = 1u, .iters = 1000u },
                                &(task_attr_t){ .priority = osPriorityAboveNormal, .ret = &R[0], .join_bit = 1u });
    h[1] = SchedT_Spammer_spawn(&(SchedT_Spammer_args_t){ .id = 2u, .iters = 1000u },
                                &(task_attr_t){ .priority = osPriorityBelowNormal, .ret = &R[1], .join_bit = 2u });
    TEST_ASSERT(task_h_valid(h[0]) && task_h_valid(h[1]), "Spawn refuse");

    osStatus_t st = task_join_all(h, 2u, 10000u);
    TEST_ASSERT(st == osOK, "join_all -> %d", (int)st);
    TEST_ASSERT(R[0] == 0 && R[1] == 0,
                "Erreurs spammers : %ld / %ld (-1 spawn, -2 join, >0 ret faux)", (long)R[0], (long)R[1]);

    PASS("2 x 1000 spawn/join concurrents, aucun ret croise");
}

/* ========================================================================
 * T26 – Appels depuis une ISR (H1)
 * ======================================================================== */
static volatile struct {
    bool       ran;
    bool       spawn_valid;
    osStatus_t join;
    osStatus_t join_all;
    osStatus_t prewarm;
} isr_res;

/* Arme par T26 uniquement. Le handler ne reference que ce pointeur : dans un
   projet qui n'execute pas la suite, l'edition de liens (--gc-sections)
   ecarte les pools et les corps de test. */
static void (*volatile isr_hook)(void);

/* SPI5 n'est pas utilise sur APEX : son vecteur sert de point d'entree ISR. */
void SPI5_IRQHandler(void) {
    if (isr_hook != NULL) {
        isr_hook();
    }
}

static void isr_body(void) {
    task_h_t inv = TASK_H_INVALID;

    isr_res.spawn_valid = task_h_valid(SchedT_Echo_spawn(&(SchedT_Echo_args_t){ .value = 1 },
                                                         &(task_attr_t){ .priority = osPriorityNormal }));
    isr_res.join        = task_join(inv, 0u);
    isr_res.join_all    = task_join_all(&inv, 1u, 0u);
    isr_res.prewarm     = task_prewarm(&TASK_SchedT_Echo);
    isr_res.ran         = true;
}

SCHED_CASE(SCHED_rtos_test_t26_isr_rejects) {
    isr_res.ran = false;
    misuse_count = 0u;
    task_set_misuse_handler(count_misuse);
    isr_hook    = isr_body;

    /* Priorite 6 : sous configMAX_SYSCALL_INTERRUPT_PRIORITY (5). */
    NVIC_SetPriority(SPI5_IRQn, 6u);
    NVIC_EnableIRQ(SPI5_IRQn);
    NVIC_SetPendingIRQ(SPI5_IRQn);
    __DSB();
    __ISB();
    (void)osDelay(1u);
    NVIC_DisableIRQ(SPI5_IRQn);
    isr_hook = NULL;
    task_set_misuse_handler(NULL);

    TEST_ASSERT(isr_res.ran, "L'ISR de test ne s'est pas executee");
    TEST_ASSERT(!isr_res.spawn_valid, "Spawn accepte en ISR");
    TEST_ASSERT(misuse_count == 1u, "Spawn en ISR non signale (%lu)", (unsigned long)misuse_count);
    TEST_ASSERT(isr_res.join == osErrorISR, "join en ISR -> %d != osErrorISR", (int)isr_res.join);
    TEST_ASSERT(isr_res.join_all == osErrorISR, "join_all en ISR -> %d", (int)isr_res.join_all);
    TEST_ASSERT(isr_res.prewarm == osErrorISR, "prewarm en ISR -> %d", (int)isr_res.prewarm);
    TEST_ASSERT(task_busy_count(&TASK_SchedT_Echo) == 0u, "Slot consomme par le spawn en ISR");

    PASS("Spawn, join, join_all et prewarm refuses en ISR");
}
