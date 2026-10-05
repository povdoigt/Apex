#include "dt_rtos_test.h"

/* Compile seulement en build RTOS de test : les pools de la suite et le
   vecteur TIM5 n'existent pas ailleurs. */
#if (APEX_CFG_SCHED_RTOS == 1) && (APEX_CFG_PROFILE_TEST == 1)

#include "dt_seq_test.h"
#include "test_irq.h"

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <stdio.h>
#include <string.h>

/* ========================================================================
 * Table des cas de test
 * ======================================================================== */

TEST_case_table_t DT_rtos_test_cases[DT_rtos_test_N_TESTS] = {
    /* A. Suite sequentielle (T0-T24) : rejouee telle quelle sous RTOS par le
          projet, avec sa propre table DT_seq_test_cases. */
    /* B. Publication et notification */
    { .case_info = { .name = "T14 Abonne prioritaire"  }, .func = DT_rtos_test_t14_priority_subscriber  },
    { .case_info = { .name = "T15 Wait sans donnee"    }, .func = DT_rtos_test_t15_wait_no_data         },
    { .case_info = { .name = "T16 Wait reveil"         }, .func = DT_rtos_test_t16_wait_wakeup          },
    { .case_info = { .name = "T17 Deux publieurs"      }, .func = DT_rtos_test_t17_two_publishers       },
    { .case_info = { .name = "T18 Attach pendant pub"  }, .func = DT_rtos_test_t18_attach_during_publish },
    { .case_info = { .name = "T19 Liste pendant notif" }, .func = DT_rtos_test_t19_list_during_notify   },
    /* C. Lecture */
    { .case_info = { .name = "T20 Lecture dechiree ISR"}, .func = DT_rtos_test_t20_torn_read_isr        },
    { .case_info = { .name = "T21 Pertes signalees"    }, .func = DT_rtos_test_t21_loss_accounting      },
    /* D. circular_buffer */
    { .case_info = { .name = "T22 cb_pop concurrents"  }, .func = DT_rtos_test_t22_cb_pop_concurrent    },
    { .case_info = { .name = "T23 cb_push ISR"         }, .func = DT_rtos_test_t23_cb_push_isr          },
    /* E. Interruptions et cycle de vie */
    { .case_info = { .name = "T24 Publish ISR + wait"  }, .func = DT_rtos_test_t24_publish_isr_wait     },
    { .case_info = { .name = "T25 Free / re-init"      }, .func = DT_rtos_test_t25_free_reinit          },
    { .case_info = { .name = "T26 Attach/detach rafale"}, .func = DT_rtos_test_t26_attach_churn         },
    /* F. Avant le demarrage du noyau */
    { .case_info = { .name = "T27 Publish avant noyau" }, .func = DT_rtos_test_t27_publish_before_kernel },
    /* G. Attente infinie et liste face a une interruption */
    { .case_info = { .name = "T28 Wait osWaitForever"  }, .func = DT_rtos_test_t28_wait_forever         },
    { .case_info = { .name = "T29 Attach/detach vs ISR"}, .func = DT_rtos_test_t29_churn_vs_isr         },
    { .case_info = { .name = "T30 2 taches, 1 liste"   }, .func = DT_rtos_test_t30_two_tasks_one_list   },
    { .case_info = { .name = "T31 Abonne fantome"      }, .func = DT_rtos_test_t31_dangling_subscriber  },
};

/* ========================================================================
 * Etat partage
 *
 * Tout ce que touche une tache ou l'ISR (topics, abonnes, resultats) vit en
 * statique, jamais sur la pile du cas : un TEST_ASSERT en echec rend la main
 * avant la fin des taches, qui ecriraient sinon dans une pile morte.
 * ======================================================================== */

#define JOIN_BITS_MASK  0x7FFFFFFEu   /* bits 1..30 */
#define CAP_A           8u            /* topics A et B */
#define CAP_F           4u            /* topic des trames */
#define CAP_CB          64u           /* T22 */
#define CAP_CB_ISR      32u           /* T23 */
#define N_RES           4u

typedef struct {
    uint32_t seq;
    uint32_t pad[30];                 /* = seq, sur toute la trame */
    uint32_t seq_end;                 /* = seq */
} frame_t;                            /* 128 o */

typedef struct {
    uint32_t count;     /* donnees recues                          */
    uint32_t last;      /* derniere valeur recue                   */
    uint32_t bad;       /* statut ou valeur inattendus             */
    uint32_t losses;    /* DT_DATA_LOSS recus                      */
    uint32_t skipped;   /* valeurs sautees (somme des trous)       */
    uint32_t timeouts;  /* attentes expirees                       */
    uint32_t t_start;   /* tick de debut                           */
    uint32_t t_end;     /* tick de fin                             */
    uint64_t sum;       /* somme des valeurs recues                */
} dtt_res_t;

static data_topic_t      topic_a, topic_b, topic_f;
static uint32_t          store_a[CAP_A], store_b[CAP_A];
static frame_t           store_f[CAP_F];
static data_sub_t        sub_a, sub_b, sub_c, sub_f;
static data_sub_t        sub_n[3];

static circular_buffer_t cb_t;
static uint32_t          store_cb[CAP_CB];

static volatile dtt_res_t res[N_RES];
static task_ret_t         ret[N_RES];
static volatile bool      stop;

/* Resultats ponctuels ecrits par une tache (T18, T19). */
static volatile struct {
    osStatus_t    wait;
    data_status_t attach, detach, read_a, read_b;
    uint32_t      num_b, value_a, value_b;
} probe;

/* ISR (TIM5, voir test_irq.h) */
static volatile uint32_t      isr_n;
static volatile uint32_t      isr_rejected;
static volatile bool          isr_probe_done;
static volatile osStatus_t    isr_wait_status;
static volatile data_status_t isr_attach_status;
static volatile uint32_t      isr_pops;
static volatile uint64_t      isr_sum;
static frame_t                isr_frame;   /* hors pile : la pile MSP est petite */

/* ========================================================================
 * Crochets d'interruption (appeles par TIM5_IRQHandler, test_irq.c)
 * ======================================================================== */

/* T17 : un troisieme publieur, en interruption (bit 31 : valeur de l'ISR). */
static void hook_spam_pub(void) {
    const uint32_t v = 0x80000000u | isr_n;
    (void)data_topic_publish(&topic_a, &v);
    isr_n = isr_n + 1u;
}

/* T22 : un troisieme consommateur, en interruption. */
static void hook_cb_pop(void) {
    uint32_t v;
    if (cb_pop(&cb_t, &v) == CB_OK) {
        isr_pops = isr_pops + 1u;
        isr_sum  = isr_sum + v;
    }
}

/* T28 : 1, 2, 3... sur topic_a. */
static void hook_one_pub(void) {
    const uint32_t v = isr_n + 1u;
    (void)data_topic_publish(&topic_a, &v);
    isr_n = v;
}

/* T29 : 1, 2, 3... alternativement sur topic_a (impairs) et topic_b (pairs). */
static void hook_alt_pub(void) {
    const uint32_t v = isr_n + 1u;
    (void)data_topic_publish(((v & 1u) != 0u) ? &topic_a : &topic_b, &v);
    isr_n = v;
}

/* T20 : une trame complete par interruption. */
static void hook_frame_pub(void) {
    const uint32_t n = isr_n;
    isr_frame.seq = n;
    for (uint32_t k = 0; k < 30u; k++) {
        isr_frame.pad[k] = n;
    }
    isr_frame.seq_end = n;
    (void)data_topic_publish(&topic_f, &isr_frame);
    isr_n = n + 1u;
}

/* T23 : 0, 1, 2... dans un buffer REJECT_NEW, refus comptes. */
static void hook_cb_push(void) {
    const uint32_t x = isr_n;
    if (cb_push(&cb_t, &x) == CB_FULL) {
        isr_rejected++;
    }
    isr_n = x + 1u;
}

/* T24 : 1, 2, 3... sur topic_a ; la premiere fois, sonde les API interdites. */
static void hook_seq_pub(void) {
    if (!isr_probe_done) {
        isr_wait_status   = data_sub_wait_for_data(&sub_a, 0u);
        isr_attach_status = data_sub_attach(&sub_c, &topic_b, DATA_ATTACH_FROM_NOW);
        isr_probe_done    = true;
    }
    const uint32_t v = isr_n + 1u;
    (void)data_topic_publish(&topic_a, &v);
    isr_n = v;
}

/* ========================================================================
 * Corps des taches
 * ======================================================================== */

/* Echo (T14) : wait puis read n fois, valeurs attendues 1..n. */
typedef struct { data_sub_t *sub; uint32_t n; uint32_t timeout_ms; volatile dtt_res_t *r; } DTT_Echo_args_t;
TASK_DECLARE(DTT_Echo, DTT_Echo_args_t, 1024);
TASK_DEFINE(DTT_Echo) {
    for (uint32_t k = 1u; k <= args->n && !stop; k++) {
        if (data_sub_wait_for_data(args->sub, args->timeout_ms) != osOK) {
            args->r->timeouts++;
            return 1;
        }
        uint32_t v = 0u;
        data_status_t s = data_sub_read(args->sub, &v);
        if (s != DT_OK || v != k) {
            args->r->bad++;
        }
        args->r->last = v;
        args->r->count++;
    }
    return 0;
}

/* DelayedPub (T16, T25) : publie `value` apres `delay_ms`. */
typedef struct { data_topic_t *topic; uint32_t delay_ms; uint32_t value; } DTT_DelayedPub_args_t;
TASK_DECLARE(DTT_DelayedPub, DTT_DelayedPub_args_t, 1024);
TASK_DEFINE(DTT_DelayedPub) {
    (void)osDelay(args->delay_ms);
    return (task_ret_t)data_topic_publish(args->topic, &args->value);
}

/* Spam (T17) : n publications sans pause. */
typedef struct { data_topic_t *topic; uint32_t n; volatile dtt_res_t *r; } DTT_Spam_args_t;
TASK_DECLARE(DTT_Spam, DTT_Spam_args_t, 1024);
TASK_DEFINE(DTT_Spam) {
    args->r->t_start = osKernelGetTickCount();
    for (uint32_t i = 0u; i < args->n; i++) {
        if (data_topic_publish(args->topic, &i) == DT_OK) {
            args->r->count++;
        } else {
            args->r->bad++;
        }
    }
    args->r->t_end = osKernelGetTickCount();
    return 0;
}

/* Attacher (T18) : reveille par une publication sur topic_a, attache sub_b. */
typedef struct { uint32_t timeout_ms; } DTT_Attacher_args_t;
TASK_DECLARE(DTT_Attacher, DTT_Attacher_args_t, 1024);
TASK_DEFINE(DTT_Attacher) {
    probe.wait = data_sub_wait_for_data(&sub_a, args->timeout_ms);
    if (probe.wait != osOK) return 1;

    probe.attach = data_sub_attach(&sub_b, &topic_a, DATA_ATTACH_FROM_NOW);
    probe.num_b  = data_sub_num_to_read(&sub_b);
    uint32_t v = 0xDEADu;
    probe.read_b  = data_sub_read(&sub_b, &v);
    probe.value_b = v;
    v = 0u;
    probe.read_a  = data_sub_read(&sub_a, &v);
    probe.value_a = v;
    return 0;
}

/* Mover (T19) : reveille sur topic_a, se detache et passe sur topic_b. */
typedef struct { uint32_t timeout_ms; } DTT_Mover_args_t;
TASK_DECLARE(DTT_Mover, DTT_Mover_args_t, 1024);
TASK_DEFINE(DTT_Mover) {
    probe.wait = data_sub_wait_for_data(&sub_a, args->timeout_ms);
    if (probe.wait != osOK) return 1;
    probe.detach = data_sub_detach(&sub_a);
    probe.attach = data_sub_attach(&sub_a, &topic_b, DATA_ATTACH_FROM_NOW);
    return 0;
}

/* SeqPub (T21) : publie 1..n, une valeur par periode. */
typedef struct { data_topic_t *topic; uint32_t n; uint32_t period_ms; } DTT_SeqPub_args_t;
TASK_DECLARE(DTT_SeqPub, DTT_SeqPub_args_t, 1024);
TASK_DEFINE(DTT_SeqPub) {
    for (uint32_t i = 1u; i <= args->n && !stop; i++) {
        (void)data_topic_publish(args->topic, &i);
        (void)osDelay(args->period_ms);
    }
    return 0;
}

/* LossSub (T21, T24) : lit 1..n en verifiant que chaque trou est annonce
   par DT_DATA_LOSS. S'arrete a n, sur `stop`, ou apres 3 timeouts. */
typedef struct {
    data_sub_t        *sub;
    uint32_t           n;            /* 0 : jusqu'a `stop` */
    uint32_t           read_delay_ms;
    uint32_t           timeout_ms;
    volatile dtt_res_t *r;
} DTT_LossSub_args_t;
TASK_DECLARE(DTT_LossSub, DTT_LossSub_args_t, 1024);
TASK_DEFINE(DTT_LossSub) {
    volatile dtt_res_t *r = args->r;
    uint32_t prev = 0u;

    while ((args->n == 0u || prev < args->n) && !stop) {
        if (data_sub_wait_for_data(args->sub, args->timeout_ms) != osOK) {
            if (stop) break;                /* fin demandee pendant l'attente */
            if (++r->timeouts > 3u) break;
            continue;
        }
        uint32_t      v;
        data_status_t s;
        while ((s = data_sub_read(args->sub, &v)) != DT_EMPTY) {
            if (s == DT_OK) {
                if (v != prev + 1u) r->bad++;
            } else if (s == DT_DATA_LOSS) {
                r->losses++;
                if (v <= prev + 1u) r->bad++;
                else                r->skipped += v - prev - 1u;
            } else {
                r->bad++;
                break;
            }
            prev = v;
            r->count++;
            if (args->read_delay_ms != 0u) {
                (void)osDelay(args->read_delay_ms);
            }
        }
    }
    r->last = prev;
    return 0;
}

/* Popper (T22) : vide cb_t jusqu'a `stop` et buffer vide. */
typedef struct { volatile dtt_res_t *r; } DTT_Popper_args_t;
TASK_DECLARE(DTT_Popper, DTT_Popper_args_t, 1024);
TASK_DEFINE(DTT_Popper) {
    for (;;) {
        uint32_t    v;
        cb_status_t s = cb_pop(&cb_t, &v);
        if (s == CB_OK) {
            args->r->count++;
            args->r->sum += v;
        } else if (s == CB_EMPTY) {
            if (stop) break;
            osThreadYield();
        } else {
            args->r->bad++;
        }
    }
    return 0;
}

/* AltPub (T26) : publie sur topic_a et topic_b jusqu'a `stop`. */
typedef struct { uint32_t unused; } DTT_AltPub_args_t;
TASK_DECLARE(DTT_AltPub, DTT_AltPub_args_t, 1024);
TASK_DEFINE(DTT_AltPub) {
    for (uint32_t i = 1u; !stop; i++) {
        (void)data_topic_publish(&topic_a, &i);
        (void)data_topic_publish(&topic_b, &i);
        if ((i & 3u) == 0u) {
            (void)osDelay(1u);
        }
    }
    return 0;
}

/* Churn (T26) : attache / lit / detache sub_n[0] en alternant les topics. */
typedef struct { uint32_t rounds; volatile dtt_res_t *r; } DTT_Churn_args_t;
TASK_DECLARE(DTT_Churn, DTT_Churn_args_t, 1024);
TASK_DEFINE(DTT_Churn) {
    data_topic_t *const topics[2] = { &topic_a, &topic_b };
    for (uint32_t k = 0u; k < args->rounds && !stop; k++) {
        for (uint32_t t = 0u; t < 2u; t++) {
            if (data_sub_attach(&sub_n[0], topics[t], DATA_ATTACH_FROM_NOW) != DT_OK) {
                args->r->bad++;
                continue;
            }
            if (data_sub_wait_for_data(&sub_n[0], 20u) != osOK) {
                args->r->timeouts++;
            } else {
                uint32_t v;
                data_status_t s = data_sub_read(&sub_n[0], &v);
                if (s != DT_OK && s != DT_DATA_LOSS) args->r->bad++;
                else                                 args->r->count++;
            }
            if (data_sub_detach(&sub_n[0]) != DT_OK) {
                args->r->bad++;
            }
        }
    }
    return 0;
}

/* ListLow (T30) : attache / detache sub_n[1] sur topic_a en boucle, jusqu'a `stop`. */
typedef struct { volatile dtt_res_t *r; } DTT_ListLow_args_t;
TASK_DECLARE(DTT_ListLow, DTT_ListLow_args_t, 1024);
TASK_DEFINE(DTT_ListLow) {
    while (!stop) {
        if (data_sub_attach(&sub_n[1], &topic_a, DATA_ATTACH_FROM_NOW) != DT_OK) args->r->bad++;
        if (data_sub_detach(&sub_n[1]) != DT_OK)                                  args->r->bad++;
        args->r->count++;
    }
    return 0;
}

/* ListHigh (T30) : reveillee par l'ISR (sub_c sur topic_b), donc a n'importe
   quelle instruction de ListLow, attache / detache sub_n[2] sur topic_a. */
typedef struct { uint32_t rounds; volatile dtt_res_t *r; } DTT_ListHigh_args_t;
TASK_DECLARE(DTT_ListHigh, DTT_ListHigh_args_t, 1024);
TASK_DEFINE(DTT_ListHigh) {
    for (uint32_t k = 0u; k < args->rounds && !stop; k++) {
        if (data_sub_wait_for_data(&sub_c, 20u) != osOK) {
            args->r->timeouts++;
            continue;
        }
        uint32_t v;
        (void)data_sub_read(&sub_c, &v);
        if (data_sub_attach(&sub_n[2], &topic_a, DATA_ATTACH_FROM_NOW) != DT_OK) args->r->bad++;
        if (data_sub_detach(&sub_n[2]) != DT_OK)                                  args->r->bad++;
        args->r->count++;
    }
    return 0;
}

/* ========================================================================
 * Pools de la suite
 * ======================================================================== */

TASK_POOL(DTT_Echo,       1);
TASK_POOL(DTT_DelayedPub, 1);
TASK_POOL(DTT_Spam,       2);
TASK_POOL(DTT_Attacher,   1);
TASK_POOL(DTT_Mover,      1);
TASK_POOL(DTT_SeqPub,     1);
TASK_POOL(DTT_LossSub,    3);
TASK_POOL(DTT_Popper,     2);
TASK_POOL(DTT_AltPub,     1);
TASK_POOL(DTT_Churn,      1);
TASK_POOL(DTT_ListLow,    1);
TASK_POOL(DTT_ListHigh,   1);

static const task_desc_t *const all_pools[] = {
    &TASK_DTT_Echo,     &TASK_DTT_DelayedPub, &TASK_DTT_Spam,   &TASK_DTT_Attacher,
    &TASK_DTT_Mover,    &TASK_DTT_SeqPub,     &TASK_DTT_LossSub, &TASK_DTT_Popper,
    &TASK_DTT_AltPub,   &TASK_DTT_Churn,      &TASK_DTT_ListLow, &TASK_DTT_ListHigh,
};

/* ========================================================================
 * Outils
 * ======================================================================== */

static uint32_t pools_busy(void) {
    uint32_t n = 0u;
    for (size_t i = 0; i < sizeof(all_pools) / sizeof(all_pools[0]); i++) {
        n += task_busy_count(all_pools[i]);
    }
    return n;
}

static inline uint32_t now_ms(void) {
    return osKernelGetTickCount();
}

#define ATTR(prio_, i_)  (&(task_attr_t){ .priority = (prio_), .ret = &ret[(i_)], .join_bit = (uint8_t)((i_) + 1u) })

/* Initialise un topic de uint32_t de capacite CAP_A. */
static data_status_t topic_u32_init(data_topic_t *t, uint32_t *store) {
    memset(store, 0, CAP_A * sizeof(uint32_t));
    return data_topic_init(t, store, sizeof(uint32_t), CAP_A, CB_OVERWRITE_OLDEST);
}

/* Remet l'environnement a zero apres un cas, qu'il ait reussi ou non. */
static void dt_cleanup(TEST_case_t *tc) {
    TEST_irq_stop();
    stop = true;

    uint32_t t = 0u;
    while (pools_busy() != 0u && t < 2000u) {
        (void)osDelay(1u);
        t++;
    }
    const uint32_t left = pools_busy();

    /* free detache tous les abonnes (et supprime leurs semaphores). */
    data_topic_free(&topic_a);
    data_topic_free(&topic_b);
    data_topic_free(&topic_f);
    data_sub_t *const subs[] = { &sub_a, &sub_b, &sub_c, &sub_f, &sub_n[0], &sub_n[1], &sub_n[2] };
    for (size_t i = 0; i < sizeof(subs) / sizeof(subs[0]); i++) {
        if (subs[i]->attached) {
            (void)data_sub_detach(subs[i]);
        }
    }

    (void)osThreadFlagsClear(JOIN_BITS_MASK);
    (void)osThreadSetPriority(osThreadGetId(), osPriorityNormal);

    memset((void *)res, 0, sizeof(res));
    memset((void *)&probe, 0, sizeof(probe));
    memset(ret, 0, sizeof(ret));
    isr_n             = 0u;
    isr_rejected      = 0u;
    isr_pops          = 0u;
    isr_sum           = 0u;
    isr_probe_done    = false;
    isr_wait_status   = osOK;
    isr_attach_status = DT_OK;
    stop              = false;

    if (left != 0u && tc->result == R_PASS) {
        tc->result = R_FAIL;
        snprintf(tc->detail, sizeof(tc->detail),
                 "Nettoyage : %lu tache(s) encore active(s) apres 2 s", (unsigned long)left);
    }
}

/* Definit un cas : le corps ecrit avec TEST_ASSERT, suivi d'un nettoyage
   systematique, meme quand une assertion a rendu la main plus tot. */
#define DT_CASE(fn_)                                                           \
    static void fn_##_body(TEST_case_t *tc);                                   \
    void fn_(TEST_case_t *tc) {                                                \
        tc->result    = R_FAIL;                                                \
        tc->detail[0] = '\0';                                                  \
        fn_##_body(tc);                                                        \
        dt_cleanup(tc);                                                        \
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

void DT_rtos_test_init(void) {
    TEST_irq_stop();
    stop = false;
}

/* ========================================================================
 * B. Publication et notification
 * ======================================================================== */

DT_CASE(DT_rtos_test_t14_priority_subscriber) {
#define N14 50u
    TEST_ASSERT(topic_u32_init(&topic_a, store_a) == DT_OK, "init topic_a");
    TEST_ASSERT(data_sub_attach(&sub_a, &topic_a, DATA_ATTACH_FROM_NOW) == DT_OK, "attach sub_a");

    task_h_t h = DTT_Echo_spawn(&(DTT_Echo_args_t){ .sub = &sub_a, .n = N14, .timeout_ms = 200u, .r = &res[0] },
                                ATTR(osPriorityHigh, 0));
    TEST_ASSERT(task_h_valid(h), "Spawn Echo refuse");

    uint32_t late = 0u, first_late = 0u;
    for (uint32_t i = 1u; i <= N14; i++) {
        (void)data_topic_publish(&topic_a, &i);
        /* L'abonne est plus prioritaire : il a deja lu i quand publish rend la main. */
        if (res[0].last != i) {
            if (late++ == 0u) first_late = i;
        }
        (void)osDelay(1u);
    }

    TEST_ASSERT(task_join(h, 500u) == osOK, "Echo pas termine (lus=%lu)", (unsigned long)res[0].count);
    TEST_ASSERT(ret[0] == 0 && res[0].timeouts == 0u, "Echo : timeout (ret=%ld)", (long)ret[0]);
    TEST_ASSERT(res[0].bad == 0u, "%lu lecture(s) avec statut/valeur inattendu", (unsigned long)res[0].bad);
    TEST_ASSERT(late == 0u, "%lu publication(s) lue(s) en retard (1re : %lu)",
                (unsigned long)late, (unsigned long)first_late);
    TEST_ASSERT(res[0].count == N14, "%lu lectures != %u", (unsigned long)res[0].count, N14);
    PASS("%u publications lues immediatement par l'abonne High", N14);
#undef N14
}

DT_CASE(DT_rtos_test_t15_wait_no_data) {
    TEST_ASSERT(topic_u32_init(&topic_a, store_a) == DT_OK, "init topic_a");
    TEST_ASSERT(data_sub_attach(&sub_a, &topic_a, DATA_ATTACH_FROM_NOW) == DT_OK, "attach sub_a");
    TEST_ASSERT(sub_a.sem != NULL && uxSemaphoreGetCount(sub_a.sem) == 0u, "Semaphore absent ou non vide a l'attache");

    uint32_t v = 42u, out = 0u;
    (void)data_topic_publish(&topic_a, &v);
    data_status_t s = data_sub_read(&sub_a, &out);
    TEST_ASSERT(s == DT_OK && out == 42u, "read 42: s=%d out=%lu", s, (unsigned long)out);
    TEST_ASSERT(uxSemaphoreGetCount(sub_a.sem) == 1u, "Jeton attendu dans le semaphore (scenario du point 2)");

    uint32_t t0 = now_ms();
    osStatus_t w = data_sub_wait_for_data(&sub_a, 50u);
    uint32_t dt = now_ms() - t0;
    TEST_ASSERT(w == osErrorTimeout, "wait(50) sans donnee rend %d != osErrorTimeout", (int)w);
    TEST_ASSERT(dt >= 50u && dt <= 52u, "wait(50) a dure %lu ms", (unsigned long)dt);
    s = data_sub_read(&sub_a, &out);
    TEST_ASSERT(s == DT_EMPTY, "read apres timeout rend %d != DT_EMPTY", s);

    t0 = now_ms();
    w  = data_sub_wait_for_data(&sub_a, 0u);
    const uint32_t dt0 = now_ms() - t0;
    TEST_ASSERT(w == osErrorTimeout && dt0 <= 1u, "wait(0) sans donnee : %d en %lu ms", (int)w, (unsigned long)dt0);

    v = 43u;
    (void)data_topic_publish(&topic_a, &v);
    w = data_sub_wait_for_data(&sub_a, 0u);
    TEST_ASSERT(w == osOK, "wait(0) avec donnee rend %d != osOK", (int)w);
    s = data_sub_read(&sub_a, &out);
    TEST_ASSERT(s == DT_OK && out == 43u, "read 43: s=%d out=%lu", s, (unsigned long)out);

    TEST_ASSERT(data_sub_detach(&sub_a) == DT_OK, "detach");
    TEST_ASSERT(sub_a.sem == NULL, "Semaphore pas supprime au detach");
    w = data_sub_wait_for_data(&sub_a, 0u);
    TEST_ASSERT(w == osErrorParameter, "wait sur detache rend %d != osErrorParameter", (int)w);
    w = data_sub_wait_for_data(NULL, 0u);
    TEST_ASSERT(w == osErrorParameter, "wait(NULL) rend %d != osErrorParameter", (int)w);
    PASS("Jeton residuel ignore : timeout en %lu ms, puis DT_EMPTY", (unsigned long)dt);
}

DT_CASE(DT_rtos_test_t16_wait_wakeup) {
    TEST_ASSERT(topic_u32_init(&topic_a, store_a) == DT_OK, "init topic_a");
    TEST_ASSERT(data_sub_attach(&sub_a, &topic_a, DATA_ATTACH_FROM_NOW) == DT_OK, "attach sub_a");

    uint32_t t0 = now_ms();
    task_h_t h = DTT_DelayedPub_spawn(&(DTT_DelayedPub_args_t){ .topic = &topic_a, .delay_ms = 30u, .value = 7u },
                                      ATTR(osPriorityBelowNormal, 0));
    TEST_ASSERT(task_h_valid(h), "Spawn DelayedPub refuse");

    osStatus_t w  = data_sub_wait_for_data(&sub_a, 500u);
    uint32_t   dt = now_ms() - t0;
    TEST_ASSERT(w == osOK, "wait(500) rend %d != osOK", (int)w);
    TEST_ASSERT(dt >= 28u && dt <= 33u, "Reveil apres %lu ms, attendu ~30", (unsigned long)dt);

    uint32_t out = 0u;
    data_status_t s = data_sub_read(&sub_a, &out);
    TEST_ASSERT(s == DT_OK && out == 7u, "read: s=%d out=%lu attendu 7", s, (unsigned long)out);
    TEST_ASSERT(task_join(h, 100u) == osOK && ret[0] == DT_OK, "DelayedPub : join ou publish en echec");

    t0 = now_ms();
    w  = data_sub_wait_for_data(&sub_a, 20u);
    uint32_t dt2 = now_ms() - t0;
    TEST_ASSERT(w == osErrorTimeout, "wait(20) sans publieur rend %d", (int)w);
    TEST_ASSERT(dt2 >= 20u && dt2 <= 22u, "wait(20) a dure %lu ms", (unsigned long)dt2);
    PASS("Reveil a %lu ms (attendu 30), timeout 20 ms en %lu ms", (unsigned long)dt, (unsigned long)dt2);
}

DT_CASE(DT_rtos_test_t17_two_publishers) {
#define N17 20000u
    TEST_ASSERT(topic_u32_init(&topic_a, store_a) == DT_OK, "init topic_a");
    TEST_ASSERT(data_sub_attach(&sub_a, &topic_a, DATA_ATTACH_FROM_NOW) == DT_OK, "attach sub_a");

    task_h_t h[2];
    for (uint32_t i = 0u; i < 2u; i++) {
        h[i] = DTT_Spam_spawn(&(DTT_Spam_args_t){ .topic = &topic_a, .n = N17, .r = &res[i] },
                              ATTR(osPriorityBelowNormal, i));
        TEST_ASSERT(task_h_valid(h[i]), "Spawn Spam %lu refuse", (unsigned long)i);
    }
    /* Troisieme publieur en interruption : il preempte les deux taches a
       n'importe quelle instruction, la ou le time slicing ne les interrompt
       qu'au tick (revue du 05/10 : sans lui, le test passait meme sans
       section critique). */
    TEST_irq_start(20000u, hook_spam_pub);
    const osStatus_t joined = task_join_all(h, 2u, 5000u);
    TEST_irq_stop();
    const uint32_t isr_pub = isr_n;
    TEST_ASSERT(joined == osOK, "Publieurs pas termines en 5 s");

    TEST_ASSERT(res[0].count == N17 && res[1].count == N17, "publish OK : %lu / %lu",
                (unsigned long)res[0].count, (unsigned long)res[1].count);
    TEST_ASSERT(res[0].t_start < res[1].t_end && res[1].t_start < res[0].t_end,
                "Executions sans chevauchement [%lu,%lu] [%lu,%lu] : test non concluant",
                (unsigned long)res[0].t_start, (unsigned long)res[0].t_end,
                (unsigned long)res[1].t_start, (unsigned long)res[1].t_end);
    TEST_ASSERT(isr_pub > 1000u, "ISR : %lu publications seulement", (unsigned long)isr_pub);
    TEST_ASSERT(topic_a.pub_seq == 2u * N17 + isr_pub, "pub_seq=%lu != %lu : increments perdus",
                (unsigned long)topic_a.pub_seq, (unsigned long)(2u * N17 + isr_pub));
    TEST_ASSERT(data_sub_num_to_read(&sub_a) == 2u * N17 + isr_pub, "num_to_read=%lu",
                (unsigned long)data_sub_num_to_read(&sub_a));
    PASS("pub_seq=%lu exact (2 x %u + %lu en ISR), entrelaces sur %lu ms",
         (unsigned long)topic_a.pub_seq, N17, (unsigned long)isr_pub,
         (unsigned long)((res[0].t_end > res[1].t_end ? res[0].t_end : res[1].t_end) -
                         (res[0].t_start < res[1].t_start ? res[0].t_start : res[1].t_start)));
#undef N17
}

DT_CASE(DT_rtos_test_t18_attach_during_publish) {
    TEST_ASSERT(topic_u32_init(&topic_a, store_a) == DT_OK, "init topic_a");   /* stockage a 0 */
    TEST_ASSERT(data_sub_attach(&sub_a, &topic_a, DATA_ATTACH_FROM_NOW) == DT_OK, "attach sub_a");

    task_h_t h = DTT_Attacher_spawn(&(DTT_Attacher_args_t){ .timeout_ms = 500u }, ATTR(osPriorityHigh, 0));
    TEST_ASSERT(task_h_valid(h), "Spawn Attacher refuse");

    uint32_t v = 1u;
    (void)data_topic_publish(&topic_a, &v);   /* reveille Attacher, qui tourne avant le retour */
    TEST_ASSERT(task_join(h, 500u) == osOK && ret[0] == 0, "Attacher : wait=%d ret=%ld",
                (int)probe.wait, (long)ret[0]);

    TEST_ASSERT(probe.attach == DT_OK, "attach pendant la publication rend %d", probe.attach);
    TEST_ASSERT(probe.num_b == 0u, "Nouvel abonne : num_to_read=%lu != 0", (unsigned long)probe.num_b);
    TEST_ASSERT(probe.read_b == DT_EMPTY, "Nouvel abonne : read rend %d (valeur %lu) au lieu de DT_EMPTY",
                probe.read_b, (unsigned long)probe.value_b);
    TEST_ASSERT(probe.read_a == DT_OK && probe.value_a == 1u, "sub_a : s=%d v=%lu attendu 1",
                probe.read_a, (unsigned long)probe.value_a);

    /* Le thread de test reprend sub_b (sa tache est finie). */
    v = 2u;
    (void)data_topic_publish(&topic_a, &v);
    uint32_t out = 0u;
    data_status_t s = data_sub_read(&sub_b, &out);
    TEST_ASSERT(s == DT_OK && out == 2u, "sub_b ensuite : s=%d out=%lu attendu 2", s, (unsigned long)out);
    PASS("Attache pendant publish : DT_EMPTY, puis 2 recu");
}

DT_CASE(DT_rtos_test_t19_list_during_notify) {
    TEST_ASSERT(topic_u32_init(&topic_a, store_a) == DT_OK, "init topic_a");
    TEST_ASSERT(topic_u32_init(&topic_b, store_b) == DT_OK, "init topic_b");
    /* topic_a : B puis A (A en tete de liste) ; topic_b : C */
    TEST_ASSERT(data_sub_attach(&sub_b, &topic_a, DATA_ATTACH_FROM_NOW) == DT_OK, "attach B");
    TEST_ASSERT(data_sub_attach(&sub_a, &topic_a, DATA_ATTACH_FROM_NOW) == DT_OK, "attach A");
    TEST_ASSERT(data_sub_attach(&sub_c, &topic_b, DATA_ATTACH_FROM_NOW) == DT_OK, "attach C");
    TEST_ASSERT(topic_a.subs == &sub_a && sub_a.next == &sub_b, "Ordre de liste inattendu");

    task_h_t h = DTT_Mover_spawn(&(DTT_Mover_args_t){ .timeout_ms = 500u }, ATTR(osPriorityHigh, 0));
    TEST_ASSERT(task_h_valid(h), "Spawn Mover refuse");

    uint32_t v = 1u;
    (void)data_topic_publish(&topic_a, &v);
    TEST_ASSERT(task_join(h, 500u) == osOK && ret[0] == 0, "Mover : wait=%d ret=%ld",
                (int)probe.wait, (long)ret[0]);
    TEST_ASSERT(probe.detach == DT_OK && probe.attach == DT_OK, "Mover : detach=%d attach=%d",
                probe.detach, probe.attach);

    TEST_ASSERT(uxSemaphoreGetCount(sub_b.sem) == 1u, "B n'a pas ete notifie");
    TEST_ASSERT(uxSemaphoreGetCount(sub_c.sem) == 0u, "C (autre topic) a ete notifie");
    TEST_ASSERT(topic_a.sub_count == 1u && topic_a.subs == &sub_b && sub_b.prev == NULL && sub_b.next == NULL,
                "Liste A incoherente (sub_count=%lu)", (unsigned long)topic_a.sub_count);
    TEST_ASSERT(topic_b.sub_count == 2u && topic_b.subs == &sub_a && sub_a.next == &sub_c && sub_c.prev == &sub_a,
                "Liste B incoherente (sub_count=%lu)", (unsigned long)topic_b.sub_count);
    PASS("B notifie, C non ; listes A={B}, B={A,C}");
}

/* ========================================================================
 * C. Lecture
 * ======================================================================== */

DT_CASE(DT_rtos_test_t20_torn_read_isr) {
    memset(store_f, 0, sizeof(store_f));
    TEST_ASSERT(data_topic_init(&topic_f, store_f, sizeof(frame_t), CAP_F, CB_OVERWRITE_OLDEST) == DT_OK,
                "init topic_f");
    TEST_ASSERT(data_sub_attach(&sub_f, &topic_f, DATA_ATTACH_FROM_NOW) == DT_OK, "attach sub_f");

    uint32_t count = 0u, torn = 0u, bad = 0u, losses = 0u, prev = 0u;
    uint32_t rng = 0x9E3779B9u;
    bool     have_prev = false;
    frame_t  f;

    TEST_irq_start(20000u, hook_frame_pub);
    const uint32_t t_end = now_ms() + 300u;
    while ((int32_t)(now_ms() - t_end) < 0) {
        /* Lire au bord : attendre que le topic soit plein vu de l'abonne, la
           trame lue est alors la plus ancienne, celle que la prochaine
           publication ecrase. Pause aleatoire avant la lecture : sans elle,
           la lecture partirait toujours juste apres une publication, en phase
           avec l'ISR, et ne serait presque jamais interrompue (revue du 05/10 :
           le test passait parfois avec des sections critiques vides). */
        if (data_sub_num_to_read(&sub_f) < CAP_F) continue;
        rng = rng * 1664525u + 1013904223u;
        for (volatile uint32_t d = 0u; d < ((rng >> 8) % 400u); d++) {
        }
        data_status_t s = data_sub_read(&sub_f, &f);
        if (s == DT_EMPTY) continue;
        if (s != DT_OK && s != DT_DATA_LOSS) { bad++; continue; }

        bool intact = (f.seq == f.seq_end);
        for (uint32_t k = 0u; k < 30u && intact; k++) {
            intact = (f.pad[k] == f.seq);
        }
        if (!intact) torn++;

        if (have_prev) {
            if (s == DT_OK && f.seq != prev + 1u)                     bad++;
            if (s == DT_DATA_LOSS) { if (f.seq <= prev + 1u) bad++; else losses++; }
        }
        prev      = f.seq;
        have_prev = true;
        count++;
    }
    TEST_irq_stop();

    TEST_ASSERT(isr_n > 4000u, "ISR TIM5 : %lu publications seulement", (unsigned long)isr_n);
    TEST_ASSERT(torn == 0u, "%lu trame(s) dechiree(s) sur %lu lues", (unsigned long)torn, (unsigned long)count);
    TEST_ASSERT(bad == 0u, "%lu lecture(s) hors sequence ou statut inattendu", (unsigned long)bad);
    TEST_ASSERT(count > 1000u, "Seulement %lu trames lues", (unsigned long)count);
    TEST_ASSERT(losses > 0u, "Aucune perte : l'abonne n'a jamais ete depasse, test non concluant");
    PASS("%lu trames lues / %lu publiees en ISR, 0 dechiree, %lu pertes signalees",
         (unsigned long)count, (unsigned long)isr_n, (unsigned long)losses);
}

DT_CASE(DT_rtos_test_t21_loss_accounting) {
#define N21 300u
    TEST_ASSERT(topic_u32_init(&topic_a, store_a) == DT_OK, "init topic_a");
    for (uint32_t i = 0u; i < 3u; i++) {
        TEST_ASSERT(data_sub_attach(&sub_n[i], &topic_a, DATA_ATTACH_FROM_NOW) == DT_OK, "attach sub_n[%lu]",
                    (unsigned long)i);
    }

    static const osPriority_t prio[3]  = { osPriorityHigh, osPriorityBelowNormal, osPriorityLow };
    static const uint32_t     delay[3] = { 0u, 2u, 5u };
    task_h_t h[4];
    for (uint32_t i = 0u; i < 3u; i++) {
        h[i] = DTT_LossSub_spawn(&(DTT_LossSub_args_t){ .sub = &sub_n[i], .n = N21, .read_delay_ms = delay[i],
                                                        .timeout_ms = 100u, .r = &res[i] },
                                 ATTR(prio[i], i));
        TEST_ASSERT(task_h_valid(h[i]), "Spawn LossSub %lu refuse", (unsigned long)i);
    }
    h[3] = DTT_SeqPub_spawn(&(DTT_SeqPub_args_t){ .topic = &topic_a, .n = N21, .period_ms = 1u },
                            ATTR(osPriorityAboveNormal, 3));
    TEST_ASSERT(task_h_valid(h[3]), "Spawn SeqPub refuse");
    TEST_ASSERT(task_join_all(h, 4u, 3000u) == osOK, "Taches pas terminees en 3 s");

    for (uint32_t i = 0u; i < 3u; i++) {
        volatile dtt_res_t *r = &res[i];
        TEST_ASSERT(r->bad == 0u, "Abonne %lu : %lu trou(s) non annonce(s) ou statut inattendu",
                    (unsigned long)i, (unsigned long)r->bad);
        TEST_ASSERT(r->last == N21, "Abonne %lu : derniere valeur %lu != %u", (unsigned long)i,
                    (unsigned long)r->last, N21);
        TEST_ASSERT(r->count + r->skipped == N21, "Abonne %lu : lus %lu + sautes %lu != %u", (unsigned long)i,
                    (unsigned long)r->count, (unsigned long)r->skipped, N21);
        TEST_ASSERT(r->timeouts == 0u, "Abonne %lu : %lu timeout(s)", (unsigned long)i, (unsigned long)r->timeouts);
    }
    TEST_ASSERT(res[0].losses == 0u && res[0].count == N21, "Abonne High : %lu perte(s), %lu lus",
                (unsigned long)res[0].losses, (unsigned long)res[0].count);
    TEST_ASSERT(res[1].losses > 0u && res[2].losses > 0u, "Abonnes lents sans perte (%lu, %lu) : test non concluant",
                (unsigned long)res[1].losses, (unsigned long)res[2].losses);
    PASS("High %lu/300 ; lents %lu+%lu sautes (%lu pertes), %lu+%lu (%lu)",
         (unsigned long)res[0].count,
         (unsigned long)res[1].count, (unsigned long)res[1].skipped, (unsigned long)res[1].losses,
         (unsigned long)res[2].count, (unsigned long)res[2].skipped, (unsigned long)res[2].losses);
#undef N21
}

/* ========================================================================
 * D. circular_buffer sous concurrence
 * ======================================================================== */

DT_CASE(DT_rtos_test_t22_cb_pop_concurrent) {
#define M22 20000u
    TEST_ASSERT(cb_init(&cb_t, store_cb, sizeof(uint32_t), CAP_CB, CB_REJECT_NEW) == CB_OK, "cb_init");

    task_h_t h[2];
    for (uint32_t i = 0u; i < 2u; i++) {
        h[i] = DTT_Popper_spawn(&(DTT_Popper_args_t){ .r = &res[i] }, ATTR(osPriorityBelowNormal, i));
        TEST_ASSERT(task_h_valid(h[i]), "Spawn Popper %lu refuse", (unsigned long)i);
    }

    /* Troisieme consommateur en interruption : il preempte les cb_pop des
       taches a n'importe quelle instruction (revue du 05/10 : avec les deux
       taches seules, le test passait meme sans section critique). */
    TEST_irq_start(20000u, hook_cb_pop);

    uint32_t full_waits = 0u;
    for (uint32_t i = 1u; i <= M22; i++) {
        while (cb_push(&cb_t, &i) == CB_FULL) {
            full_waits++;
            (void)osDelay(1u);
        }
    }
    for (uint32_t t = 0u; t < 1000u && cb_count(&cb_t) != 0u; t++) {
        (void)osDelay(1u);
    }
    stop = true;
    const osStatus_t joined = task_join_all(h, 2u, 1000u);
    TEST_irq_stop();
    TEST_ASSERT(joined == osOK, "Consommateurs pas termines");

    const uint64_t expected_sum = (uint64_t)M22 * (M22 + 1u) / 2u;
    const uint32_t pops = res[0].count + res[1].count + isr_pops;
    const uint64_t sum  = res[0].sum + res[1].sum + isr_sum;
    TEST_ASSERT(res[0].bad == 0u && res[1].bad == 0u, "cb_pop a rendu un statut inattendu");
    TEST_ASSERT(cb_count(&cb_t) == 0u, "count=%lu en fin (attendu 0)", (unsigned long)cb_count(&cb_t));
    TEST_ASSERT(pops == M22, "%lu retraits != %u pousses", (unsigned long)pops, M22);
    TEST_ASSERT(sum == expected_sum, "Somme retiree differente : valeur perdue ou dupliquee");
    TEST_ASSERT(res[0].count > 0u && res[1].count > 0u && isr_pops > 0u, "Un consommateur n'a pas travaille (%lu / %lu / ISR %lu)",
                (unsigned long)res[0].count, (unsigned long)res[1].count, (unsigned long)isr_pops);
    PASS("%u valeurs retirees une fois (%lu / %lu / ISR %lu), %lu attentes plein",
         M22, (unsigned long)res[0].count, (unsigned long)res[1].count, (unsigned long)isr_pops,
         (unsigned long)full_waits);
#undef M22
}

DT_CASE(DT_rtos_test_t23_cb_push_isr) {
    TEST_ASSERT(cb_init(&cb_t, store_cb, sizeof(uint32_t), CAP_CB_ISR, CB_REJECT_NEW) == CB_OK, "cb_init");

    uint32_t popped = 0u, next = 0u, gaps = 0u, bad = 0u, incoherent = 0u;

    TEST_irq_start(20000u, hook_cb_push);
    const uint32_t t_end = now_ms() + 300u;
    while ((int32_t)(now_ms() - t_end) < 0) {
        uint32_t v;
        while (cb_pop(&cb_t, &v) == CB_OK) {
            if (v < next) bad++;                 /* dupliquee ou desordonnee */
            else          gaps += v - next;      /* refusee par l'ISR        */
            next = v + 1u;
            popped++;
        }

        /* count doit toujours correspondre a head / tail. */
        cb_critical_t c = cb_critical_enter();
        const size_t calc = (cb_t.head + cb_t.capacity - cb_t.tail) % cb_t.capacity;
        const bool   ok   = (cb_t.count == calc) || (cb_t.count == cb_t.capacity && calc == 0u);
        cb_critical_exit(c);
        if (!ok) incoherent++;

        /* Laisser le buffer se remplir de temps en temps : refus cote ISR. */
        if ((popped & 0x3FFu) == 0u) {
            (void)osDelay(3u);
        }
    }
    TEST_irq_stop();

    uint32_t v;
    while (cb_pop(&cb_t, &v) == CB_OK) {
        if (v < next) bad++;
        else          gaps += v - next;
        next = v + 1u;
        popped++;
    }

    const uint32_t pushed   = isr_n;
    const uint32_t rejected = isr_rejected;
    TEST_ASSERT(pushed > 4000u, "ISR TIM5 : %lu appels seulement", (unsigned long)pushed);
    TEST_ASSERT(incoherent == 0u, "%lu incoherence(s) count / head / tail", (unsigned long)incoherent);
    TEST_ASSERT(bad == 0u, "%lu valeur(s) dupliquee(s) ou desordonnee(s)", (unsigned long)bad);
    TEST_ASSERT(popped + rejected == pushed, "depiles %lu + refus %lu != pousses %lu",
                (unsigned long)popped, (unsigned long)rejected, (unsigned long)pushed);
    TEST_ASSERT(gaps + (pushed - next) == rejected, "trous %lu + fin %lu != refus %lu",
                (unsigned long)gaps, (unsigned long)(pushed - next), (unsigned long)rejected);
    TEST_ASSERT(rejected > 0u, "Aucun refus : buffer jamais plein, test non concluant");
    PASS("%lu poussees en ISR : %lu depilees, %lu refusees, count coherent",
         (unsigned long)pushed, (unsigned long)popped, (unsigned long)rejected);
}

/* ========================================================================
 * E. Interruptions et cycle de vie
 * ======================================================================== */

DT_CASE(DT_rtos_test_t24_publish_isr_wait) {
    TEST_ASSERT(topic_u32_init(&topic_a, store_a) == DT_OK, "init topic_a");
    TEST_ASSERT(topic_u32_init(&topic_b, store_b) == DT_OK, "init topic_b");
    TEST_ASSERT(data_sub_attach(&sub_a, &topic_a, DATA_ATTACH_FROM_NOW) == DT_OK, "attach sub_a");

    task_h_t h = DTT_LossSub_spawn(&(DTT_LossSub_args_t){ .sub = &sub_a, .n = 0u, .read_delay_ms = 0u,
                                                          .timeout_ms = 10u, .r = &res[0] },
                                   ATTR(osPriorityHigh, 0));
    TEST_ASSERT(task_h_valid(h), "Spawn LossSub refuse");

    TEST_irq_start(1000u, hook_seq_pub);
    (void)osDelay(200u);
    TEST_irq_stop();
    (void)osDelay(5u);          /* l'abonne lit la derniere publication */
    stop = true;
    TEST_ASSERT(task_join(h, 100u) == osOK, "Abonne pas termine");

    const uint32_t published = isr_n;
    TEST_ASSERT(isr_probe_done, "L'ISR n'a jamais tourne");
    TEST_ASSERT(isr_wait_status == osErrorISR, "wait depuis l'ISR rend %d != osErrorISR", (int)isr_wait_status);
    TEST_ASSERT(isr_attach_status == DT_BAD_ARG, "attach depuis l'ISR rend %d != DT_BAD_ARG", isr_attach_status);
    TEST_ASSERT(sub_c.attached == 0, "attach depuis l'ISR a quand meme attache l'abonne");
    TEST_ASSERT(published >= 150u, "ISR : %lu publications en 200 ms", (unsigned long)published);
    TEST_ASSERT(res[0].timeouts == 0u, "%lu attente(s) expiree(s) malgre l'ISR a 1 kHz", (unsigned long)res[0].timeouts);
    TEST_ASSERT(res[0].bad == 0u && res[0].losses == 0u, "bad=%lu pertes=%lu",
                (unsigned long)res[0].bad, (unsigned long)res[0].losses);
    TEST_ASSERT(res[0].count == published && res[0].last == published, "recus %lu (dernier %lu) / publies %lu",
                (unsigned long)res[0].count, (unsigned long)res[0].last, (unsigned long)published);
    PASS("%lu publications en ISR toutes recues, wait/attach refuses en ISR", (unsigned long)published);
}

DT_CASE(DT_rtos_test_t25_free_reinit) {
    TEST_ASSERT(topic_u32_init(&topic_a, store_a) == DT_OK, "init topic_a");
    TEST_ASSERT(data_sub_attach(&sub_a, &topic_a, DATA_ATTACH_FROM_NOW) == DT_OK, "attach sub_a");
    TEST_ASSERT(data_sub_attach(&sub_b, &topic_a, DATA_ATTACH_FROM_NOW) == DT_OK, "attach sub_b");
    TEST_ASSERT(sub_a.sem != NULL && sub_b.sem != NULL, "Semaphores non crees");
    uint32_t v = 1u;
    (void)data_topic_publish(&topic_a, &v);

    data_topic_free(&topic_a);
    TEST_ASSERT(sub_a.attached == 0 && sub_b.attached == 0, "Abonnes encore attaches apres free");
    TEST_ASSERT(sub_a.sem == NULL && sub_b.sem == NULL, "Semaphores non supprimes par free");
    TEST_ASSERT(data_sub_wait_for_data(&sub_a, 0u) == osErrorParameter, "wait apres free");
    TEST_ASSERT(data_topic_publish(&topic_a, &v) == DT_BAD_ARG, "publish apres free accepte");

    TEST_ASSERT(topic_u32_init(&topic_a, store_a) == DT_OK, "re-init topic_a");
    TEST_ASSERT(data_sub_attach(&sub_a, &topic_a, DATA_ATTACH_FROM_NOW) == DT_OK, "re-attach sub_a");
    TEST_ASSERT(data_sub_wait_for_data(&sub_a, 0u) == osErrorTimeout, "Semaphore neuf non vide");

    task_h_t h = DTT_DelayedPub_spawn(&(DTT_DelayedPub_args_t){ .topic = &topic_a, .delay_ms = 10u, .value = 77u },
                                      ATTR(osPriorityBelowNormal, 0));
    TEST_ASSERT(task_h_valid(h), "Spawn DelayedPub refuse");
    osStatus_t w = data_sub_wait_for_data(&sub_a, 200u);
    TEST_ASSERT(w == osOK, "wait apres re-init rend %d", (int)w);
    uint32_t out = 0u;
    data_status_t s = data_sub_read(&sub_a, &out);
    TEST_ASSERT(s == DT_OK && out == 77u, "read apres re-init : s=%d out=%lu", s, (unsigned long)out);
    TEST_ASSERT(task_join(h, 100u) == osOK, "DelayedPub pas termine");

    TEST_ASSERT(data_sub_detach(&sub_a) == DT_OK, "detach");
    TEST_ASSERT(data_sub_detach(&sub_a) == DT_BAD_ARG, "Double detach accepte");
    PASS("free : semaphores supprimes ; re-init : reveil et lecture 77 OK");
}

DT_CASE(DT_rtos_test_t26_attach_churn) {
#define R26 300u
    TEST_ASSERT(topic_u32_init(&topic_a, store_a) == DT_OK, "init topic_a");
    TEST_ASSERT(topic_u32_init(&topic_b, store_b) == DT_OK, "init topic_b");
    TEST_ASSERT(data_sub_attach(&sub_b, &topic_a, DATA_ATTACH_FROM_NOW) == DT_OK, "attach sub_b (permanent)");
    TEST_ASSERT(data_sub_attach(&sub_c, &topic_b, DATA_ATTACH_FROM_NOW) == DT_OK, "attach sub_c (permanent)");

    task_h_t hp = DTT_AltPub_spawn(&(DTT_AltPub_args_t){ 0 }, ATTR(osPriorityAboveNormal, 0));
    TEST_ASSERT(task_h_valid(hp), "Spawn AltPub refuse");
    task_h_t hc = DTT_Churn_spawn(&(DTT_Churn_args_t){ .rounds = R26, .r = &res[1] }, ATTR(osPriorityHigh, 1));
    TEST_ASSERT(task_h_valid(hc), "Spawn Churn refuse");

    TEST_ASSERT(task_join(hc, 10000u) == osOK, "Churn pas termine en 10 s (%lu lectures)", (unsigned long)res[1].count);
    stop = true;
    TEST_ASSERT(task_join(hp, 500u) == osOK, "AltPub pas termine");

    TEST_ASSERT(res[1].bad == 0u, "%lu attach/read/detach en echec", (unsigned long)res[1].bad);
    TEST_ASSERT(res[1].timeouts == 0u, "%lu attente(s) expiree(s)", (unsigned long)res[1].timeouts);
    TEST_ASSERT(res[1].count == 2u * R26, "%lu lectures != %u", (unsigned long)res[1].count, 2u * R26);
    TEST_ASSERT(sub_n[0].attached == 0, "sub_n[0] encore attache");
    TEST_ASSERT(topic_a.sub_count == 1u && topic_a.subs == &sub_b && sub_b.prev == NULL && sub_b.next == NULL,
                "Liste A incoherente (sub_count=%lu)", (unsigned long)topic_a.sub_count);
    TEST_ASSERT(topic_b.sub_count == 1u && topic_b.subs == &sub_c && sub_c.prev == NULL && sub_c.next == NULL,
                "Liste B incoherente (sub_count=%lu)", (unsigned long)topic_b.sub_count);
    TEST_ASSERT(uxSemaphoreGetCount(sub_b.sem) == 1u && uxSemaphoreGetCount(sub_c.sem) == 1u,
                "Abonne permanent jamais notifie");
    PASS("%u x 2 attach/wait/read/detach, %lu publications, listes intactes",
         R26, (unsigned long)topic_a.pub_seq);
#undef R26
}

/* ========================================================================
 * F. Avant le demarrage du noyau
 * ======================================================================== */

/* Etat releve par DT_rtos_test_pre_kernel(), avant osKernelStart(). */
static data_topic_t topic_pk;
static uint32_t     store_pk[CAP_A];
static data_sub_t   sub_pk;
static volatile struct {
    bool          ran;                /* le crochet a ete appele          */
    bool          not_started;        /* ... noyau effectivement arrete   */
    data_status_t init, attach;
    data_status_t pub[3];
    osStatus_t    wait;               /* wait_for_data avant le noyau     */
    uint32_t      num;                /* num_to_read apres 3 publications */
} pk;
static bool pk_checked;

void DT_rtos_test_pre_kernel(void) {
    pk.ran         = true;
    pk.not_started = (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED);
    pk.init        = data_topic_init(&topic_pk, store_pk, sizeof(uint32_t), CAP_A, CB_OVERWRITE_OLDEST);
    pk.attach      = data_sub_attach(&sub_pk, &topic_pk, DATA_ATTACH_FROM_NOW);
    for (uint32_t i = 0u; i < 3u; i++) {
        const uint32_t v = 101u + i;
        /* Avant correctif, la notification passait par xSemaphoreGiveFromISR, dont
           le configASSERT sur PRIGROUP bloque la carte tant que le noyau n'a pas
           demarre : on n'arrivait jamais jusqu'au rapport. */
        pk.pub[i] = data_topic_publish(&topic_pk, &v);
    }
    pk.wait = data_sub_wait_for_data(&sub_pk, 0u);
    pk.num  = data_sub_num_to_read(&sub_pk);
}

DT_CASE(DT_rtos_test_t27_publish_before_kernel) {
    if (pk_checked) {
        /* L'etat pre-noyau n'existe qu'une fois par demarrage. */
        snprintf(tc->detail, sizeof(tc->detail), "Etat pre-noyau deja verifie depuis ce demarrage");
        tc->result = R_SKIP;
        return;
    }
    pk_checked = true;

    TEST_ASSERT(pk.ran, "setup_pre_kernel() n'a pas appele DT_rtos_test_pre_kernel()");
    TEST_ASSERT(pk.not_started, "Crochet appele noyau demarre : test non concluant");
    TEST_ASSERT(pk.init == DT_OK && pk.attach == DT_OK, "Avant noyau : init=%d attach=%d", pk.init, pk.attach);
    TEST_ASSERT(pk.pub[0] == DT_OK && pk.pub[1] == DT_OK && pk.pub[2] == DT_OK,
                "Avant noyau : publish %d/%d/%d", pk.pub[0], pk.pub[1], pk.pub[2]);
    TEST_ASSERT(pk.wait == osError, "wait avant noyau rend %d != osError", (int)pk.wait);
    TEST_ASSERT(pk.num == 3u, "Avant noyau : num_to_read=%lu != 3", (unsigned long)pk.num);

    /* Noyau demarre : les donnees pre-noyau sont la, sans attente. */
    TEST_ASSERT(data_sub_wait_for_data(&sub_pk, 0u) == osOK, "wait(0) ne voit pas les donnees pre-noyau");
    for (uint32_t i = 0u; i < 3u; i++) {
        uint32_t out = 0u;
        data_status_t s = data_sub_read(&sub_pk, &out);
        TEST_ASSERT(s == DT_OK && out == 101u + i, "read %lu : s=%d out=%lu", (unsigned long)i, s, (unsigned long)out);
    }

    /* Et la notification fonctionne ensuite normalement (semaphore cree avant le noyau). */
    task_h_t h = DTT_DelayedPub_spawn(&(DTT_DelayedPub_args_t){ .topic = &topic_pk, .delay_ms = 10u, .value = 104u },
                                      ATTR(osPriorityBelowNormal, 0));
    TEST_ASSERT(task_h_valid(h), "Spawn DelayedPub refuse");
    osStatus_t w = data_sub_wait_for_data(&sub_pk, 200u);
    TEST_ASSERT(task_join(h, 100u) == osOK && ret[0] == DT_OK, "DelayedPub : join ou publish en echec");
    TEST_ASSERT(w == osOK, "wait apres demarrage rend %d", (int)w);
    uint32_t out = 0u;
    data_status_t s = data_sub_read(&sub_pk, &out);
    TEST_ASSERT(s == DT_OK && out == 104u, "read 104 : s=%d out=%lu", s, (unsigned long)out);

    data_topic_free(&topic_pk);
    PASS("3 publications avant noyau sans blocage, lues apres ; reveil OK ensuite");
}

/* ========================================================================
 * G. Attente infinie et liste face a une interruption
 * ======================================================================== */

DT_CASE(DT_rtos_test_t28_wait_forever) {
    TEST_ASSERT(topic_u32_init(&topic_a, store_a) == DT_OK, "init topic_a");
    TEST_ASSERT(data_sub_attach(&sub_a, &topic_a, DATA_ATTACH_FROM_NOW) == DT_OK, "attach sub_a");

    /* Reveil par une tache. */
    uint32_t t0 = now_ms();
    task_h_t h = DTT_DelayedPub_spawn(&(DTT_DelayedPub_args_t){ .topic = &topic_a, .delay_ms = 20u, .value = 5u },
                                      ATTR(osPriorityBelowNormal, 0));
    TEST_ASSERT(task_h_valid(h), "Spawn DelayedPub refuse");
    osStatus_t w = data_sub_wait_for_data(&sub_a, osWaitForever);
    const uint32_t dt_task = now_ms() - t0;
    TEST_ASSERT(w == osOK, "wait(osWaitForever) rend %d", (int)w);
    TEST_ASSERT(dt_task >= 18u && dt_task <= 25u, "reveil par tache apres %lu ms, attendu ~20", (unsigned long)dt_task);
    uint32_t out = 0u;
    data_status_t s = data_sub_read(&sub_a, &out);
    TEST_ASSERT(s == DT_OK && out == 5u, "read : s=%d out=%lu attendu 5", s, (unsigned long)out);
    TEST_ASSERT(task_join(h, 100u) == osOK, "DelayedPub pas termine");

    /* Reveil par une interruption (une publication toutes les 20 ms). */
    t0 = now_ms();
    TEST_irq_start(50u, hook_one_pub);
    w = data_sub_wait_for_data(&sub_a, osWaitForever);
    const uint32_t dt_isr = now_ms() - t0;
    TEST_irq_stop();
    TEST_ASSERT(w == osOK, "wait(osWaitForever) face a l'ISR rend %d", (int)w);
    TEST_ASSERT(dt_isr >= 18u && dt_isr <= 25u, "reveil par ISR apres %lu ms, attendu ~20", (unsigned long)dt_isr);
    s = data_sub_read(&sub_a, &out);
    TEST_ASSERT(s == DT_OK && out == 1u, "read : s=%d out=%lu attendu 1", s, (unsigned long)out);
    PASS("osWaitForever reveille par tache (%lu ms) et par ISR (%lu ms)", (unsigned long)dt_task, (unsigned long)dt_isr);
}

DT_CASE(DT_rtos_test_t29_churn_vs_isr) {
#define R29 1500u
    TEST_ASSERT(topic_u32_init(&topic_a, store_a) == DT_OK, "init topic_a");
    TEST_ASSERT(topic_u32_init(&topic_b, store_b) == DT_OK, "init topic_b");
    TEST_ASSERT(data_sub_attach(&sub_b, &topic_a, DATA_ATTACH_FROM_NOW) == DT_OK, "attach sub_b (permanent)");
    TEST_ASSERT(data_sub_attach(&sub_c, &topic_b, DATA_ATTACH_FROM_NOW) == DT_OK, "attach sub_c (permanent)");

    /* L'ISR parcourt les listes (notification) pendant que la tache les modifie. */
    TEST_irq_start(20000u, hook_alt_pub);
    task_h_t hc = DTT_Churn_spawn(&(DTT_Churn_args_t){ .rounds = R29, .r = &res[1] }, ATTR(osPriorityHigh, 1));
    const bool spawned = task_h_valid(hc);
    const osStatus_t joined = spawned ? task_join(hc, 10000u) : osError;
    TEST_irq_stop();
    const uint32_t published = isr_n;

    TEST_ASSERT(spawned, "Spawn Churn refuse");
    TEST_ASSERT(joined == osOK, "Churn pas termine en 10 s (%lu lectures)", (unsigned long)res[1].count);
    TEST_ASSERT(res[1].bad == 0u, "%lu attach/read/detach en echec", (unsigned long)res[1].bad);
    TEST_ASSERT(res[1].timeouts == 0u, "%lu attente(s) expiree(s)", (unsigned long)res[1].timeouts);
    TEST_ASSERT(res[1].count == 2u * R29, "%lu lectures != %u", (unsigned long)res[1].count, 2u * R29);
    TEST_ASSERT(published > 1000u, "ISR : %lu publications seulement", (unsigned long)published);
    TEST_ASSERT(sub_n[0].attached == 0, "sub_n[0] encore attache");
    TEST_ASSERT(topic_a.sub_count == 1u && topic_a.subs == &sub_b && sub_b.prev == NULL && sub_b.next == NULL,
                "Liste A incoherente (sub_count=%lu)", (unsigned long)topic_a.sub_count);
    TEST_ASSERT(topic_b.sub_count == 1u && topic_b.subs == &sub_c && sub_c.prev == NULL && sub_c.next == NULL,
                "Liste B incoherente (sub_count=%lu)", (unsigned long)topic_b.sub_count);
    TEST_ASSERT(uxSemaphoreGetCount(sub_b.sem) == 1u && uxSemaphoreGetCount(sub_c.sem) == 1u,
                "Abonne permanent jamais notifie");
    TEST_ASSERT(data_sub_num_to_read(&sub_b) + data_sub_num_to_read(&sub_c) == published,
                "Abonnes permanents : %lu + %lu != %lu publications", (unsigned long)data_sub_num_to_read(&sub_b),
                (unsigned long)data_sub_num_to_read(&sub_c), (unsigned long)published);
    PASS("%u x 2 attach/wait/read/detach face a %lu publications ISR, listes intactes", R29, (unsigned long)published);
#undef R29
}

DT_CASE(DT_rtos_test_t30_two_tasks_one_list) {
#define R30 2000u
    TEST_ASSERT(topic_u32_init(&topic_a, store_a) == DT_OK, "init topic_a");
    TEST_ASSERT(topic_u32_init(&topic_b, store_b) == DT_OK, "init topic_b");
    TEST_ASSERT(data_sub_attach(&sub_b, &topic_a, DATA_ATTACH_FROM_NOW) == DT_OK, "attach sub_b (permanent)");
    TEST_ASSERT(data_sub_attach(&sub_c, &topic_b, DATA_ATTACH_FROM_NOW) == DT_OK, "attach sub_c (reveil de High)");

    task_h_t hl = DTT_ListLow_spawn(&(DTT_ListLow_args_t){ .r = &res[0] }, ATTR(osPriorityBelowNormal, 0));
    task_h_t hh = DTT_ListHigh_spawn(&(DTT_ListHigh_args_t){ .rounds = R30, .r = &res[1] }, ATTR(osPriorityHigh, 1));
    const bool spawned = task_h_valid(hl) && task_h_valid(hh);

    /* L'ISR publie alternativement sur topic_a (sa liste est parcourue) et sur
       topic_b (reveil de High, qui preempte Low au milieu d'une operation). */
    TEST_irq_start(20000u, hook_alt_pub);
    const osStatus_t joined_high = task_h_valid(hh) ? task_join(hh, 10000u) : osError;
    stop = true;
    const osStatus_t joined_low = task_h_valid(hl) ? task_join(hl, 1000u) : osError;
    TEST_irq_stop();

    TEST_ASSERT(spawned, "Spawn ListLow / ListHigh refuse");
    TEST_ASSERT(joined_high == osOK && joined_low == osOK, "Taches pas terminees (High %d, Low %d)",
                (int)joined_high, (int)joined_low);
    TEST_ASSERT(res[0].bad == 0u && res[1].bad == 0u, "attach/detach en echec : Low %lu, High %lu",
                (unsigned long)res[0].bad, (unsigned long)res[1].bad);
    TEST_ASSERT(res[1].timeouts == 0u && res[1].count == R30, "High : %lu tours, %lu timeouts",
                (unsigned long)res[1].count, (unsigned long)res[1].timeouts);
    TEST_ASSERT(res[0].count > 1000u, "Low : %lu tours seulement, test non concluant", (unsigned long)res[0].count);
    TEST_ASSERT(sub_n[1].attached == 0 && sub_n[2].attached == 0, "abonne encore attache en fin");
    TEST_ASSERT(topic_a.sub_count == 1u && topic_a.subs == &sub_b && sub_b.prev == NULL && sub_b.next == NULL,
                "Liste A incoherente (sub_count=%lu)", (unsigned long)topic_a.sub_count);
    PASS("%u modifs High (reveil ISR) au milieu de %lu modifs Low : liste A intacte", R30, (unsigned long)res[0].count);
#undef R30
}

DT_CASE(DT_rtos_test_t31_dangling_subscriber) {
    TEST_ASSERT(topic_u32_init(&topic_a, store_a) == DT_OK, "init topic_a");
    /* Liste : sub_c (tete) -> sub_n[0] (futur fantome) -> sub_b (queue). */
    TEST_ASSERT(data_sub_attach(&sub_b, &topic_a, DATA_ATTACH_FROM_NOW) == DT_OK, "attach sub_b");
    TEST_ASSERT(data_sub_attach(&sub_n[0], &topic_a, DATA_ATTACH_FROM_NOW) == DT_OK, "attach sub_n[0]");
    TEST_ASSERT(data_sub_attach(&sub_c, &topic_a, DATA_ATTACH_FROM_NOW) == DT_OK, "attach sub_c");

    /* La tache de sub_n[0] a rendu la main sans detach, et sa memoire sert
       maintenant a autre chose : contenu quelconque, pointeurs compris. */
    static data_sub_t saved, garbage;
    saved = sub_n[0];
    memset(&garbage, 0xA5, sizeof(garbage));
    sub_n[0] = garbage;

    uint32_t v = 1u;
    const data_status_t pub = data_topic_publish(&topic_a, &v);       /* tache : ne doit pas planter */
    const uint32_t faults_task = topic_a.list_faults;
    const UBaseType_t tok_c = uxSemaphoreGetCount(sub_c.sem);
    const UBaseType_t tok_b = uxSemaphoreGetCount(sub_b.sem);

    isr_n = 1u;                                                       /* l'ISR publie 2, 3, ... */
    TEST_irq_start(1000u, hook_one_pub);
    (void)osDelay(10u);
    TEST_irq_stop();
    const uint32_t isr_pub    = isr_n - 1u;
    const uint32_t faults_isr = topic_a.list_faults - faults_task;
    const uint32_t lag_b      = data_sub_num_to_read(&sub_b);

    /* Attache d'un nouvel abonne (le parcours de controle s'arrete sur le
       fantome), puis detach du voisin du fantome : rien n'est ecrit dedans. */
    const data_status_t att = data_sub_attach(&sub_n[1], &topic_a, DATA_ATTACH_FROM_NOW);
    const uint32_t before_det = topic_a.list_faults;
    const data_status_t det = data_sub_detach(&sub_c);
    const bool untouched = (memcmp(&sub_n[0], &garbage, sizeof(garbage)) == 0);
    const bool fault_det = (topic_a.list_faults == before_det + 1u);

    /* Remise en etat avant toute assertion : le nettoyage detache le reste. */
    sub_n[0] = saved;

    TEST_ASSERT(pub == DT_OK, "publish avec un abonne fantome rend %d", pub);
    TEST_ASSERT(faults_task == 1u, "anomalie comptee a la publication : %lu, attendu 1", (unsigned long)faults_task);
    TEST_ASSERT(tok_c == 1u && tok_b == 0u, "jetons : tete %lu (attendu 1), au-dela du fantome %lu (attendu 0)",
                (unsigned long)tok_c, (unsigned long)tok_b);
    TEST_ASSERT(isr_pub >= 5u && faults_isr == isr_pub, "ISR : %lu publications, %lu anomalies comptees",
                (unsigned long)isr_pub, (unsigned long)faults_isr);
    TEST_ASSERT(lag_b == 1u + isr_pub, "au-dela du fantome, num_to_read=%lu attendu %lu (donnees lisibles)",
                (unsigned long)lag_b, (unsigned long)(1u + isr_pub));
    TEST_ASSERT(att == DT_OK, "attache d'un nouvel abonne refusee (%d)", att);
    TEST_ASSERT(det == DT_OK && untouched, "detach du voisin : s=%d, fantome %s", det, untouched ? "intact" : "ECRIT");
    TEST_ASSERT(fault_det, "detach du voisin : anomalie non comptee");
    PASS("fantome : 1 + %lu notifications arretees sans plantage, rien ecrit dedans", (unsigned long)isr_pub);
}

#endif /* APEX_CFG_SCHED_RTOS && APEX_CFG_PROFILE_TEST */
