#include "dt_rtos_stress.h"

/* Compile seulement en build RTOS de test : pools de taches et TIM5. */
#if (APEX_CFG_SCHED_RTOS == 1) && (APEX_CFG_PROFILE_TEST == 1)

#include "data_topic.h"
#include "data_packet.h"
#include "circular_buffer.h"
#include "scheduler.h"
#include "test_irq.h"
#include "bench.h"

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include <string.h>

/* ========================================================================
 * Echantillons : chaque element porte ses mots de controle
 *
 * Horodatage : HAL_GetTick() partout (ISR, taches, instant des paquets),
 * une seule base de temps pour la fenetre du packer.
 * ======================================================================== */

#define STR_MIX  0xA5A5A5A5u

typedef struct { uint32_t ts; uint32_t seq; uint32_t inv; uint32_t mix; } str_s16_t;   /* acc, gyr : 16 o */
typedef struct { uint32_t ts; uint32_t seq; uint32_t inv;               } str_s12_t;   /* baro     : 12 o */
typedef struct {                                                                        /* paquet   : 40 o */
    uint32_t ts, flags;
    uint32_t acc[3];          /* seq, inv, mix */
    uint32_t gyr[3];          /* seq, inv, mix */
    uint32_t baro[2];         /* seq, inv      */
} str_pkt_t;

static bool str_ok3(const uint32_t w[3]) { return w[1] == ~w[0] && w[2] == (w[0] ^ STR_MIX); }
static bool str_ok2(const uint32_t w[2]) { return w[1] == ~w[0]; }

static bool str_zero(const uint32_t *w, size_t n) {
    for (size_t i = 0; i < n; i++) {
        if (w[i] != 0u) {
            return false;
        }
    }
    return true;
}

/* ========================================================================
 * Topics, abonnes et comptes
 * ======================================================================== */

#define ACC_CAP   16u
#define GYR_CAP   16u
#define BARO_CAP   8u
#define PKT_CAP    8u
#define WINDOW_MS 20u                 /* fenetre du packer : +-10 ms */

static data_topic_t  t_acc, t_gyr, t_baro;
static str_s16_t     acc_store[ACC_CAP], gyr_store[GYR_CAP];
static str_s12_t     baro_store[BARO_CAP];
static data_packer_t packer;
static str_pkt_t     pkt_store[PKT_CAP];

static data_sub_t nav_acc, nav_gyr, log_acc, log_gyr, log_baro, tlm_acc, tlm_gyr, tlm_baro, churn_sub, radio_sub;

static volatile uint32_t acc_pub, gyr_pub, baro_pub;
static volatile bool     st_stop;
static str_s16_t         isr_acc;                 /* hors pile : la pile MSP est petite */

/* Comptabilite d'un abonne sur un topic. */
typedef struct {
    uint32_t read;      /* donnees lues                    */
    uint32_t skipped;   /* donnees sautees (somme des trous) */
    uint32_t losses;    /* DT_DATA_LOSS recus               */
    uint32_t bad;       /* trou non annonce, controle faux  */
    uint32_t prev;      /* derniere sequence lue            */
    uint32_t max_lag;   /* plus grand num_to_read vu        */
} str_acct_t;

enum { A_NAV_ACC, A_NAV_GYR, A_LOG_ACC, A_LOG_GYR, A_LOG_BARO, A_TLM_ACC, A_TLM_GYR, A_TLM_BARO, A_N };
static const char *const acct_name[A_N] = {
    "nav / acc", "nav / gyr", "logger / acc", "logger / gyr", "logger / baro",
    "telemetrie / acc", "telemetrie / gyr", "telemetrie / baro",
};
static str_acct_t acct[A_N];

enum { K_GYR, K_BARO, K_NAV, K_LOG, K_TLM, K_CHURN, K_PACK, K_RADIO, K_N };
static const char *const task_name[K_N] = {
    "STR_GyrPub", "STR_BaroPub", "STR_Nav", "STR_Logger", "STR_Telemetry", "STR_Churn", "STR_Packer", "STR_Radio",
};

static struct {
    bool     ran, pass;
    uint32_t duration_ms;
    uint32_t isr_n;
    uint32_t churn_ops, churn_bad, churn_reads;
    uint32_t pkt_built, pkt_read, pkt_losses, pkt_bad, pkt_with[3];
    uint32_t checks, check_bad;
    uint32_t headroom[K_N];
    char     verdict[120];
} er;

static void str_account(str_acct_t *a, data_status_t s, uint32_t seq, bool intact) {
    if (s != DT_OK && s != DT_DATA_LOSS) {
        a->bad++;
        return;
    }
    if (!intact) a->bad++;
    if (s == DT_OK) {
        if (seq != a->prev + 1u) a->bad++;
    } else {
        a->losses++;
        if (seq <= a->prev + 1u) a->bad++;
        else                     a->skipped += seq - a->prev - 1u;
    }
    a->prev = seq;
    a->read++;
}

static void str_drain16(data_sub_t *sub, str_acct_t *a) {
    const uint32_t lag = data_sub_num_to_read(sub);
    if (lag > a->max_lag) a->max_lag = lag;
    str_s16_t s;
    data_status_t st;
    while ((st = data_sub_read(sub, &s)) != DT_EMPTY) {
        if (st == DT_BAD_ARG) {
            a->bad++;
            return;
        }
        str_account(a, st, s.seq, s.inv == ~s.seq && s.mix == (s.seq ^ STR_MIX));
    }
}

static void str_drain12(data_sub_t *sub, str_acct_t *a) {
    const uint32_t lag = data_sub_num_to_read(sub);
    if (lag > a->max_lag) a->max_lag = lag;
    str_s12_t s;
    data_status_t st;
    while ((st = data_sub_read(sub, &s)) != DT_EMPTY) {
        if (st == DT_BAD_ARG) {
            a->bad++;
            return;
        }
        str_account(a, st, s.seq, s.inv == ~s.seq);
    }
}

/* ========================================================================
 * Publieurs
 * ======================================================================== */

/* acc : une publication par interruption TIM5 (1 kHz). */
static void str_hook_acc(void) {
    const uint32_t n = acc_pub + 1u;
    isr_acc.ts  = HAL_GetTick();
    isr_acc.seq = n;
    isr_acc.inv = ~n;
    isr_acc.mix = n ^ STR_MIX;
    (void)data_topic_publish(&t_acc, &isr_acc);
    acc_pub = n;
}

typedef struct { uint32_t period_ms; } STR_Period_args_t;

TASK_DECLARE(STR_GyrPub, STR_Period_args_t, 1024);
TASK_DEFINE(STR_GyrPub) {
    uint32_t next = osKernelGetTickCount();
    while (!st_stop) {
        const uint32_t n = gyr_pub + 1u;
        const str_s16_t s = { .ts = HAL_GetTick(), .seq = n, .inv = ~n, .mix = n ^ STR_MIX };
        (void)data_topic_publish(&t_gyr, &s);
        gyr_pub = n;
        next += args->period_ms;
        (void)osDelayUntil(next);
    }
    return 0;
}

TASK_DECLARE(STR_BaroPub, STR_Period_args_t, 1024);
TASK_DEFINE(STR_BaroPub) {
    uint32_t next = osKernelGetTickCount();
    while (!st_stop) {
        const uint32_t n = baro_pub + 1u;
        const str_s12_t s = { .ts = HAL_GetTick(), .seq = n, .inv = ~n };
        (void)data_topic_publish(&t_baro, &s);
        baro_pub = n;
        next += args->period_ms;
        (void)osDelayUntil(next);
    }
    return 0;
}

/* ========================================================================
 * Abonnes
 * ======================================================================== */

/* nav : reveillee par acc, lit tout acc et tout gyr ; ne doit rien perdre. */
TASK_DECLARE(STR_Nav, STR_Period_args_t, 1024);
TASK_DEFINE(STR_Nav) {
    while (!st_stop) {
        if (data_sub_wait_for_data(&nav_acc, args->period_ms) == osOK) {
            str_drain16(&nav_acc, &acct[A_NAV_ACC]);
        }
        str_drain16(&nav_gyr, &acct[A_NAV_GYR]);
    }
    return 0;
}

/* logger (10 ms) et telemetrie (100 ms) : lecture periodique de tout. */
typedef struct { uint32_t period_ms; data_sub_t *acc, *gyr, *baro; str_acct_t *a_acc, *a_gyr, *a_baro; } STR_Poll_args_t;
TASK_DECLARE(STR_Poll, STR_Poll_args_t, 1024);
TASK_DEFINE(STR_Poll) {
    while (!st_stop) {
        str_drain16(args->acc, args->a_acc);
        str_drain16(args->gyr, args->a_gyr);
        str_drain12(args->baro, args->a_baro);
        (void)osDelay(args->period_ms);
    }
    return 0;
}

/* Rafale : un abonne qui s'attache a acc toutes les 50 ms, lit, se detache. */
TASK_DECLARE(STR_Churn, STR_Period_args_t, 1024);
TASK_DEFINE(STR_Churn) {
    while (!st_stop) {
        if (data_sub_attach(&churn_sub, &t_acc, DATA_ATTACH_FROM_OLDEST) != DT_OK) {
            er.churn_bad++;
        } else {
            for (uint32_t k = 0u; k < 4u; k++) {
                str_s16_t s;
                const data_status_t st = data_sub_read(&churn_sub, &s);
                if (st == DT_EMPTY) break;
                if (st == DT_BAD_ARG || s.inv != ~s.seq || s.mix != (s.seq ^ STR_MIX)) er.churn_bad++;
                else                                                                    er.churn_reads++;
            }
            if (data_sub_detach(&churn_sub) != DT_OK) {
                er.churn_bad++;
            }
        }
        er.churn_ops++;
        (void)osDelay(args->period_ms);
    }
    return 0;
}

/* Packer (10 Hz) : acc, gyr et baro dans un paquet de 40 o. */
TASK_DECLARE(STR_Packer, STR_Period_args_t, 1024);
TASK_DEFINE(STR_Packer) {
    while (!st_stop) {
        const uint32_t flags = data_packer_build_publish(&packer, HAL_GetTick());
        er.pkt_built++;
        for (uint32_t i = 0u; i < 3u; i++) {
            if ((flags & (1u << i)) != 0u) er.pkt_with[i]++;
        }
        (void)osDelay(args->period_ms);
    }
    return 0;
}

/* Radio : relit chaque paquet et le verifie champ par champ. */
static void str_check_packets(void) {
    str_pkt_t p;
    data_status_t st;
    while ((st = data_sub_read(&radio_sub, &p)) != DT_EMPTY) {
        if (st == DT_BAD_ARG) {
            er.pkt_bad++;
            return;
        }
        if (st == DT_DATA_LOSS) er.pkt_losses++;
        bool ok = (p.flags & ~0x7u) == 0u;
        ok = ok && (((p.flags & 0x1u) != 0u) ? str_ok3(p.acc)  : str_zero(p.acc, 3u));
        ok = ok && (((p.flags & 0x2u) != 0u) ? str_ok3(p.gyr)  : str_zero(p.gyr, 3u));
        ok = ok && (((p.flags & 0x4u) != 0u) ? str_ok2(p.baro) : str_zero(p.baro, 2u));
        if (!ok) er.pkt_bad++;
        er.pkt_read++;
    }
}

TASK_DECLARE(STR_Radio, STR_Period_args_t, 1024);
TASK_DEFINE(STR_Radio) {
    while (!st_stop) {
        if (data_sub_wait_for_data(&radio_sub, args->period_ms) == osOK) {
            str_check_packets();
        }
    }
    return 0;
}

TASK_POOL(STR_GyrPub,  1);
TASK_POOL(STR_BaroPub, 1);
TASK_POOL(STR_Nav,     1);
TASK_POOL(STR_Poll,    2);
TASK_POOL(STR_Churn,   1);
TASK_POOL(STR_Packer,  1);
TASK_POOL(STR_Radio,   1);

/* ========================================================================
 * Invariants d'un topic, sous section critique
 * ======================================================================== */

static bool str_topic_ok(const data_topic_t *t, uint32_t published, size_t subs_min, size_t subs_max) {
    cb_critical_t c = cb_critical_enter();
    const circular_buffer_t *cb = &t->cb;
    const size_t calc = (cb->head + cb->capacity - cb->tail) % cb->capacity;
    bool ok = (cb->count <= cb->capacity) &&
              ((cb->count == calc) || (cb->count == cb->capacity && calc == 0u)) &&
              (t->pub_seq == published) &&
              (t->sub_count >= subs_min) && (t->sub_count <= subs_max) &&
              (t->list_faults == 0u);
    size_t n = 0u;
    for (size_t i = 0u; i < DATA_TOPIC_MAX_SUBS; i++) {
        const data_sub_t *p = t->subs[i];
        if (p == NULL) continue;
        n++;
        if (p->topic != t || !p->attached) ok = false;
    }
    if (n != t->sub_count) ok = false;
    cb_critical_exit(c);
    return ok;
}

/* ========================================================================
 * Endurance
 * ======================================================================== */

#define ATTR_K(prio_, k_)  (&(task_attr_t){ .priority = (prio_), .ret = &rets[(k_)], .join_bit = (uint8_t)((k_) + 1u) })

static void str_fail(const char *what) {
    if (er.verdict[0] == '\0') {
        snprintf(er.verdict, sizeof(er.verdict), "%s", what);
    }
    er.pass = false;
}

bool DT_stress_endurance_run(uint32_t duration_s) {
    static task_ret_t rets[K_N];
    memset(&er, 0, sizeof(er));
    memset(acct, 0, sizeof(acct));
    er.ran  = true;
    er.pass = true;
    acc_pub = gyr_pub = baro_pub = 0u;
    st_stop = false;

    /* Topics et abonnes, avant tout publieur : la comptabilite part de 1. */
    (void)data_topic_init(&t_acc,  acc_store,  sizeof(str_s16_t), ACC_CAP,  CB_OVERWRITE_OLDEST);
    (void)data_topic_init(&t_gyr,  gyr_store,  sizeof(str_s16_t), GYR_CAP,  CB_OVERWRITE_OLDEST);
    (void)data_topic_init(&t_baro, baro_store, sizeof(str_s12_t), BARO_CAP, CB_OVERWRITE_OLDEST);
    data_sub_t *const subs[] = { &nav_acc, &nav_gyr, &log_acc, &log_gyr, &log_baro, &tlm_acc, &tlm_gyr, &tlm_baro,
                                 &churn_sub, &radio_sub };
    for (size_t i = 0; i < sizeof(subs) / sizeof(subs[0]); i++) {
        memset(subs[i], 0, sizeof(*subs[i]));
    }
    bool ok = data_sub_attach(&nav_acc,  &t_acc,  DATA_ATTACH_FROM_NOW) == DT_OK &&
              data_sub_attach(&nav_gyr,  &t_gyr,  DATA_ATTACH_FROM_NOW) == DT_OK &&
              data_sub_attach(&log_acc,  &t_acc,  DATA_ATTACH_FROM_NOW) == DT_OK &&
              data_sub_attach(&log_gyr,  &t_gyr,  DATA_ATTACH_FROM_NOW) == DT_OK &&
              data_sub_attach(&log_baro, &t_baro, DATA_ATTACH_FROM_NOW) == DT_OK &&
              data_sub_attach(&tlm_acc,  &t_acc,  DATA_ATTACH_FROM_NOW) == DT_OK &&
              data_sub_attach(&tlm_gyr,  &t_gyr,  DATA_ATTACH_FROM_NOW) == DT_OK &&
              data_sub_attach(&tlm_baro, &t_baro, DATA_ATTACH_FROM_NOW) == DT_OK;
    memset(&packer, 0, sizeof(packer));
    data_topic_t *src[3] = { &t_acc, &t_gyr, &t_baro };
    ok = ok && data_packer_init(&packer, WINDOW_MS, 3u, src, PKT_CAP, pkt_store) == DT_OK &&
               packer.packet_size == sizeof(str_pkt_t) &&
               data_sub_attach(&radio_sub, &packer.topic, DATA_ATTACH_FROM_NOW) == DT_OK;
    if (!ok) {
        str_fail("Initialisation des topics / abonnes / packer en echec");
        return false;
    }

    task_h_t h[K_N];
    h[K_GYR]   = STR_GyrPub_spawn(&(STR_Period_args_t){ .period_ms = 2u }, ATTR_K(osPriorityHigh, K_GYR));
    h[K_BARO]  = STR_BaroPub_spawn(&(STR_Period_args_t){ .period_ms = 20u }, ATTR_K(osPriorityAboveNormal, K_BARO));
    h[K_NAV]   = STR_Nav_spawn(&(STR_Period_args_t){ .period_ms = 10u }, ATTR_K(osPriorityAboveNormal1, K_NAV));
    h[K_LOG]   = STR_Poll_spawn(&(STR_Poll_args_t){ .period_ms = 10u, .acc = &log_acc, .gyr = &log_gyr, .baro = &log_baro,
                                                    .a_acc = &acct[A_LOG_ACC], .a_gyr = &acct[A_LOG_GYR],
                                                    .a_baro = &acct[A_LOG_BARO] },
                                ATTR_K(osPriorityBelowNormal, K_LOG));
    h[K_TLM]   = STR_Poll_spawn(&(STR_Poll_args_t){ .period_ms = 100u, .acc = &tlm_acc, .gyr = &tlm_gyr, .baro = &tlm_baro,
                                                    .a_acc = &acct[A_TLM_ACC], .a_gyr = &acct[A_TLM_GYR],
                                                    .a_baro = &acct[A_TLM_BARO] },
                                ATTR_K(osPriorityLow, K_TLM));
    h[K_CHURN] = STR_Churn_spawn(&(STR_Period_args_t){ .period_ms = 50u }, ATTR_K(osPriorityNormal, K_CHURN));
    h[K_PACK]  = STR_Packer_spawn(&(STR_Period_args_t){ .period_ms = 100u }, ATTR_K(osPriorityBelowNormal, K_PACK));
    h[K_RADIO] = STR_Radio_spawn(&(STR_Period_args_t){ .period_ms = 200u }, ATTR_K(osPriorityLow, K_RADIO));
    for (size_t k = 0; k < K_N; k++) {
        if (!task_h_valid(h[k])) {
            str_fail("Spawn d'une tache refuse");
        }
    }

    /* acc en interruption, puis verification des invariants chaque seconde. */
    const uint32_t t0 = osKernelGetTickCount();
    TEST_irq_start(1000u, str_hook_acc);
    for (uint32_t s = 0u; s < duration_s && er.pass; s++) {
        (void)osDelay(1000u);
        er.checks++;
        if (!str_topic_ok(&t_acc, acc_pub, 4u, 5u) || !str_topic_ok(&t_gyr, gyr_pub, 4u, 4u) ||
            !str_topic_ok(&t_baro, baro_pub, 3u, 3u) || !str_topic_ok(&packer.topic, packer.topic.pub_seq, 1u, 1u)) {
            er.check_bad++;
            str_fail("Invariant de topic viole (count / head / tail / pub_seq / liste)");
        }
        for (size_t i = 0; i < A_N; i++) {
            if (acct[i].bad != 0u) str_fail("Donnee corrompue ou trou non annonce");
        }
        if (er.churn_bad != 0u) str_fail("Rafale d'attache : echec");
        if (er.pkt_bad != 0u)   str_fail("Paquet incoherent");
    }

    st_stop = true;
    TEST_irq_stop();
    er.isr_n       = acc_pub;
    er.duration_ms = osKernelGetTickCount() - t0;
    if (task_join_all(h, K_N, 2000u) != osOK) {
        str_fail("Taches pas terminees 2 s apres l'arret");
    }

    /* Publieurs arretes : vidange finale, puis comptes exacts. */
    str_drain16(&nav_acc, &acct[A_NAV_ACC]);   str_drain16(&nav_gyr, &acct[A_NAV_GYR]);
    str_drain16(&log_acc, &acct[A_LOG_ACC]);   str_drain16(&log_gyr, &acct[A_LOG_GYR]);
    str_drain12(&log_baro, &acct[A_LOG_BARO]);
    str_drain16(&tlm_acc, &acct[A_TLM_ACC]);   str_drain16(&tlm_gyr, &acct[A_TLM_GYR]);
    str_drain12(&tlm_baro, &acct[A_TLM_BARO]);
    str_check_packets();

    static const uint8_t topic_of[A_N] = { 0, 1, 0, 1, 2, 0, 1, 2 };
    const uint32_t published[3] = { acc_pub, gyr_pub, baro_pub };
    for (size_t i = 0; i < A_N; i++) {
        const uint32_t pub = published[topic_of[i]];
        if (acct[i].bad != 0u)                       str_fail("Donnee corrompue ou trou non annonce");
        if (acct[i].prev != pub)                     str_fail("Derniere donnee lue != derniere publiee");
        if (acct[i].read + acct[i].skipped != pub)   str_fail("lus + sautes != publies");
    }
    if (acct[A_NAV_ACC].losses != 0u || acct[A_NAV_GYR].losses != 0u) str_fail("nav a perdu des donnees");
    if (er.pkt_bad != 0u)                                             str_fail("Paquet incoherent");
    if (er.pkt_read == 0u || er.pkt_with[0] == 0u || er.pkt_with[1] == 0u) str_fail("Chaine data_packet inactive");
    if (er.churn_bad != 0u || er.churn_ops == 0u)                     str_fail("Rafale d'attache : echec");
    if (acc_pub < (duration_s * 1000u * 9u) / 10u)                    str_fail("ISR acc en dessous de 1 kHz");

    const task_desc_t *const pools[K_N] = { &TASK_STR_GyrPub, &TASK_STR_BaroPub, &TASK_STR_Nav, &TASK_STR_Poll,
                                            &TASK_STR_Poll, &TASK_STR_Churn, &TASK_STR_Packer, &TASK_STR_Radio };
    static const uint8_t inst[K_N] = { 0, 0, 0, 0, 1, 0, 0, 0 };
    for (size_t k = 0; k < K_N; k++) {
        er.headroom[k] = task_stack_headroom(pools[k], inst[k]);
        if (er.headroom[k] < 256u) str_fail("Marge de pile < 256 o sur une tache");
    }

    /* Rendre la memoire propre. */
    for (size_t i = 0; i < sizeof(subs) / sizeof(subs[0]); i++) {
        if (subs[i]->attached) (void)data_sub_detach(subs[i]);
    }
    data_packer_free(&packer);
    data_topic_free(&t_acc);
    data_topic_free(&t_gyr);
    data_topic_free(&t_baro);

    if (er.pass) {
        snprintf(er.verdict, sizeof(er.verdict), "PASS");
    }
    return er.pass;
}

void DT_stress_endurance_print(void (*print)(const char *)) {
    char line[160];
    if (!er.ran) {
        print("Endurance : non executee\r\n");
        return;
    }
    snprintf(line, sizeof(line), "\r\n===== Endurance (%lu s) : %s =====\r\n",
             (unsigned long)(er.duration_ms / 1000u), er.verdict);
    print(line);
    snprintf(line, sizeof(line), "Publies : acc %lu (ISR 1 kHz), gyr %lu (500 Hz), baro %lu (50 Hz) ; %lu controles d'invariants, %lu viole(s)\r\n",
             (unsigned long)acc_pub, (unsigned long)gyr_pub, (unsigned long)baro_pub,
             (unsigned long)er.checks, (unsigned long)er.check_bad);
    print(line);
    print("  # abonne / topic          lus   sautes   pertes  corrompus  retard max\r\n");
    for (size_t i = 0; i < A_N; i++) {
        snprintf(line, sizeof(line), "  %u %-20s %8lu %8lu %8lu %8lu %8lu\r\n", (unsigned)i, acct_name[i],
                 (unsigned long)acct[i].read, (unsigned long)acct[i].skipped, (unsigned long)acct[i].losses,
                 (unsigned long)acct[i].bad, (unsigned long)acct[i].max_lag);
        print(line);
    }
    snprintf(line, sizeof(line), "Rafale d'attache : %lu cycles, %lu lectures, %lu echecs\r\n",
             (unsigned long)er.churn_ops, (unsigned long)er.churn_reads, (unsigned long)er.churn_bad);
    print(line);
    snprintf(line, sizeof(line), "data_packet : %lu paquets (acc %lu, gyr %lu, baro %lu), %lu relus, %lu pertes, %lu incoherents\r\n",
             (unsigned long)er.pkt_built, (unsigned long)er.pkt_with[0], (unsigned long)er.pkt_with[1],
             (unsigned long)er.pkt_with[2], (unsigned long)er.pkt_read, (unsigned long)er.pkt_losses,
             (unsigned long)er.pkt_bad);
    print(line);
    print("Pile jamais utilisee (o, sur 1024) :");
    for (size_t k = 0; k < K_N; k++) {
        snprintf(line, sizeof(line), " %s=%lu", task_name[k] + 4, (unsigned long)er.headroom[k]);
        print(line);
    }
    print("\r\n");
}

/* ========================================================================
 * Durees des sections critiques (DWT)
 * ======================================================================== */

#define TM_N      200u
#define TM_SLOTS  8u

static uint32_t       tm_samples[TM_N];
static BENCH_result_t tm_res[24];
static size_t         tm_n;
static uint32_t       tm_store[(TM_SLOTS * 256u) / sizeof(uint32_t)];
static uint32_t       tm_elem[256u / sizeof(uint32_t)];

void DT_stress_timing_run(void) {
    static const uint32_t sizes[] = { 4u, 16u, 64u, 128u, 256u };
    static const char *const g_masked = "Interruptions masquees : borne du temps masque";
    BENCH_init();
    tm_n = 0u;

    for (size_t k = 0; k < sizeof(sizes) / sizeof(sizes[0]); k++) {
        const uint32_t size = sizes[k];
        circular_buffer_t cb;
        (void)cb_init(&cb, tm_store, size, TM_SLOTS, CB_OVERWRITE_OLDEST);
        for (uint32_t i = 0u; i < TM_N; i++) {
            const cb_critical_t c = cb_critical_enter();
            const uint32_t t0 = BENCH_now();
            (void)cb_push(&cb, tm_elem);
            tm_samples[i] = BENCH_now() - t0;
            cb_critical_exit(c);
        }
        BENCH_summarize(&tm_res[tm_n++], g_masked, "cb_push", size, tm_samples, TM_N);

        for (uint32_t i = 0u; i < TM_N; i++) {
            (void)cb_push(&cb, tm_elem);
            const cb_critical_t c = cb_critical_enter();
            const uint32_t t0 = BENCH_now();
            (void)cb_pop(&cb, tm_elem);
            tm_samples[i] = BENCH_now() - t0;
            cb_critical_exit(c);
        }
        BENCH_summarize(&tm_res[tm_n++], g_masked, "cb_pop", size, tm_samples, TM_N);

        data_topic_t t;
        data_sub_t s = { 0 };
        (void)data_topic_init(&t, tm_store, size, TM_SLOTS, CB_OVERWRITE_OLDEST);
        (void)data_sub_attach(&s, &t, DATA_ATTACH_FROM_NOW);
        for (uint32_t i = 0u; i < TM_N; i++) {
            (void)data_topic_publish(&t, tm_elem);
            const cb_critical_t c = cb_critical_enter();
            const uint32_t t0 = BENCH_now();
            (void)data_sub_read(&s, tm_elem);
            tm_samples[i] = BENCH_now() - t0;
            cb_critical_exit(c);
        }
        BENCH_summarize(&tm_res[tm_n++], g_masked, "data_sub_read", size, tm_samples, TM_N);
        (void)data_sub_detach(&s);
        data_topic_free(&t);
    }

    /* Publication complete, notification comprise, interruptions actives. */
    static const char *const names[] = { "publish, 0 abonne", "publish, 1 abonne", "publish, 4 abonnes" };
    static const uint32_t nsubs[] = { 0u, 1u, 4u };
    for (size_t k = 0; k < 3u; k++) {
        data_topic_t t;
        data_sub_t s[4];
        memset(s, 0, sizeof(s));
        (void)data_topic_init(&t, tm_store, 16u, TM_SLOTS, CB_OVERWRITE_OLDEST);
        for (uint32_t j = 0u; j < nsubs[k]; j++) {
            (void)data_sub_attach(&s[j], &t, DATA_ATTACH_FROM_NOW);
        }
        for (uint32_t i = 0u; i < TM_N; i++) {
            const uint32_t t0 = BENCH_now();
            (void)data_topic_publish(&t, tm_elem);
            tm_samples[i] = BENCH_now() - t0;
        }
        BENCH_summarize(&tm_res[tm_n++], "Publication complete (16 o, interruptions actives)", names[k], 16u,
                        tm_samples, TM_N);
        for (uint32_t j = 0u; j < nsubs[k]; j++) {
            (void)data_sub_detach(&s[j]);
        }
        data_topic_free(&t);
    }
}

void DT_stress_timing_print(void (*print)(const char *)) {
    print("\r\n===== Durees (DWT, build Debug -O0) =====\r\n");
    BENCH_print_table(tm_res, tm_n, print);
}

#endif /* APEX_CFG_SCHED_RTOS && APEX_CFG_PROFILE_TEST */
