#include "dp_seq_test.h"
#include "test_irq.h"

#include "main.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/* ========================================================================
 * Table des cas de test
 * ======================================================================== */

TEST_case_table_t DP_seq_test_cases[DP_seq_test_N_TESTS] = {
    { .case_info = { .name = "T0 Init / free"       }, .func = DP_seq_test_t0_init_args       },
    { .case_info = { .name = "T1 Paquet nominal"    }, .func = DP_seq_test_t1_basic_packet    },
    { .case_info = { .name = "T2 Fenetre"           }, .func = DP_seq_test_t2_time_window     },
    { .case_info = { .name = "T3 Copie, champs a 0" }, .func = DP_seq_test_t3_copy_publish    },
    { .case_info = { .name = "T4 Source liberee"    }, .func = DP_seq_test_t4_source_freed    },
    { .case_info = { .name = "T5 Concurrence ISR"   }, .func = DP_seq_test_t5_isr_concurrency },
    { .case_info = { .name = "T6 Rejet borne"       }, .func = DP_seq_test_t6_bounded_discard },
};

/* ========================================================================
 * Sources, packer et paquets
 *
 * Tout est statique : le packer (32 abonnes) est trop gros pour une pile de
 * tache, et l'ISR de T5 touche aux topics.
 * ======================================================================== */

typedef struct { uint32_t ts; uint32_t v[3];  } dpt_a_t;     /* 16 o, charge 12 o */
typedef struct { uint32_t ts; uint32_t v;     } dpt_b_t;     /*  8 o, charge  4 o */
typedef struct { uint32_t ts; uint32_t v[15]; } dpt_c_t;     /* 64 o, charge 60 o */

typedef struct { uint32_t ts, flags; uint32_t a[3]; uint32_t b; uint32_t c[15]; } dpt_pkt_abc_t;
typedef struct { uint32_t ts, flags; uint32_t a[3]; uint32_t b;                } dpt_pkt_ab_t;
typedef struct { uint32_t ts, flags; uint32_t a[3];                            } dpt_pkt_a_t;

#define DPT_SRC_CAP   8u
#define DPT_PKT_CAP   4u

static data_topic_t  dpt_a, dpt_b, dpt_c, dpt_big;
static dpt_a_t       dpt_a_store[DPT_SRC_CAP];
static dpt_b_t       dpt_b_store[DPT_SRC_CAP];
static dpt_c_t       dpt_c_store[DPT_SRC_CAP];
static data_packer_t dpt_packer;
static uint32_t      dpt_packets[(DPT_PKT_CAP * sizeof(dpt_pkt_abc_t)) / sizeof(uint32_t)];
static data_sub_t    dpt_consumer;

static void dpt_setup(size_t c_capacity) {
    memset(dpt_a_store, 0, sizeof(dpt_a_store));
    memset(dpt_b_store, 0, sizeof(dpt_b_store));
    memset(dpt_c_store, 0, sizeof(dpt_c_store));
    memset(dpt_packets, 0, sizeof(dpt_packets));
    memset(&dpt_packer, 0, sizeof(dpt_packer));
    memset(&dpt_consumer, 0, sizeof(dpt_consumer));
    (void)data_topic_init(&dpt_a, dpt_a_store, sizeof(dpt_a_t), DPT_SRC_CAP, CB_OVERWRITE_OLDEST);
    (void)data_topic_init(&dpt_b, dpt_b_store, sizeof(dpt_b_t), DPT_SRC_CAP, CB_OVERWRITE_OLDEST);
    (void)data_topic_init(&dpt_c, dpt_c_store, sizeof(dpt_c_t), c_capacity, CB_OVERWRITE_OLDEST);
}

static void dpt_teardown(void) {
    TEST_irq_stop();
    if (dpt_consumer.attached) {
        (void)data_sub_detach(&dpt_consumer);
    }
    data_packer_free(&dpt_packer);
    data_topic_free(&dpt_a);
    data_topic_free(&dpt_b);
    data_topic_free(&dpt_c);
}

static void dpt_pub_a(uint32_t ts, uint32_t v) {
    const dpt_a_t s = { .ts = ts, .v = { v, v, v } };
    (void)data_topic_publish(&dpt_a, &s);
}

static void dpt_pub_b(uint32_t ts, uint32_t v) {
    const dpt_b_t s = { .ts = ts, .v = v };
    (void)data_topic_publish(&dpt_b, &s);
}

static bool dpt_all(const uint32_t *w, size_t n, uint32_t value) {
    for (size_t i = 0; i < n; i++) {
        if (w[i] != value) {
            return false;
        }
    }
    return true;
}

/* ========================================================================
 * T0 – Arguments de data_packer_init, cycle init / free / init
 * ======================================================================== */
void DP_seq_test_t0_init_args(TEST_case_t *tc) {
    tc->result = R_FAIL;
    dpt_setup(DPT_SRC_CAP);

    data_topic_t *src[3] = { &dpt_a, &dpt_b, &dpt_c };
    TEST_ASSERT(data_packer_packet_size(src, 3u) == sizeof(dpt_pkt_abc_t), "packet_size=%u attendu %u",
                (unsigned)data_packer_packet_size(src, 3u), (unsigned)sizeof(dpt_pkt_abc_t));

    TEST_ASSERT(data_packer_init(NULL, 10u, 3u, src, DPT_PKT_CAP, dpt_packets) == DT_BAD_ARG, "packer NULL accepte");
    TEST_ASSERT(data_packer_init(&dpt_packer, 10u, 3u, NULL, DPT_PKT_CAP, dpt_packets) == DT_BAD_ARG, "topics NULL accepte");
    TEST_ASSERT(data_packer_init(&dpt_packer, 10u, 0u, src, DPT_PKT_CAP, dpt_packets) == DT_BAD_ARG, "0 topic accepte");
    TEST_ASSERT(data_packer_init(&dpt_packer, 10u, DATA_PACKET_MAX_TOPICS + 1u, src, DPT_PKT_CAP, dpt_packets) == DT_BAD_ARG,
                "33 topics acceptes");
    TEST_ASSERT(data_packer_init(&dpt_packer, 10u, 3u, src, DPT_PKT_CAP, NULL) == DT_BAD_ARG, "storage NULL accepte");
    TEST_ASSERT(data_packer_init(&dpt_packer, 10u, 3u, src, 0u, dpt_packets) == DT_BAD_ARG, "capacite 0 acceptee");

    data_topic_t *with_null[2] = { &dpt_a, NULL };
    TEST_ASSERT(data_packer_init(&dpt_packer, 10u, 2u, with_null, DPT_PKT_CAP, dpt_packets) == DT_BAD_ARG, "topic NULL accepte");
    data_topic_t uninit = { 0 };
    data_topic_t *with_uninit[2] = { &dpt_a, &uninit };
    TEST_ASSERT(data_packer_init(&dpt_packer, 10u, 2u, with_uninit, DPT_PKT_CAP, dpt_packets) == DT_BAD_ARG,
                "topic non initialise accepte");

    static uint32_t big_store[4u * 17u];          /* elements de 68 o > DATA_PACKET_MAX_ELEM_SIZE */
    (void)data_topic_init(&dpt_big, big_store, 17u * sizeof(uint32_t), 4u, CB_OVERWRITE_OLDEST);
    data_topic_t *with_big[1] = { &dpt_big };
    TEST_ASSERT(data_packer_init(&dpt_packer, 10u, 1u, with_big, DPT_PKT_CAP, dpt_packets) == DT_BAD_ARG,
                "element de 68 o accepte");
    data_topic_free(&dpt_big);

    data_topic_t *many[5] = { &dpt_c, &dpt_c, &dpt_c, &dpt_c, &dpt_c };   /* 8 + 5 * 60 = 308 o > 256 */
    TEST_ASSERT(data_packer_packet_size(many, 5u) == 0u, "paquet de 308 o : packet_size != 0");
    TEST_ASSERT(data_packer_init(&dpt_packer, 10u, 5u, many, DPT_PKT_CAP, dpt_packets) == DT_BAD_ARG,
                "paquet de 308 o accepte");

    TEST_ASSERT(dpt_a.sub_count == 0u && dpt_b.sub_count == 0u && dpt_c.sub_count == 0u,
                "abonne laisse sur une source apres refus (%u/%u/%u)",
                (unsigned)dpt_a.sub_count, (unsigned)dpt_b.sub_count, (unsigned)dpt_c.sub_count);

    dpt_packer.subs[5].attached = 1;              /* packer pas remis a zero */
    TEST_ASSERT(data_packer_init(&dpt_packer, 10u, 3u, src, DPT_PKT_CAP, dpt_packets) == DT_BAD_ARG,
                "packer non remis a zero accepte");
    dpt_packer.subs[5].attached = 0;

    TEST_ASSERT(data_packer_init(&dpt_packer, 10u, 3u, src, DPT_PKT_CAP, dpt_packets) == DT_OK, "init valide refuse");
    TEST_ASSERT(dpt_a.sub_count == 1u && dpt_b.sub_count == 1u && dpt_c.sub_count == 1u, "abonnes du packer absents");
    TEST_ASSERT(dpt_packer.packet_size == sizeof(dpt_pkt_abc_t), "packet_size=%u", (unsigned)dpt_packer.packet_size);
    TEST_ASSERT(data_packer_init(&dpt_packer, 10u, 3u, src, DPT_PKT_CAP, dpt_packets) == DT_BAD_ARG,
                "re-init sans free acceptee");
    data_packer_free(&dpt_packer);
    TEST_ASSERT(dpt_a.sub_count == 0u && dpt_b.sub_count == 0u && dpt_c.sub_count == 0u, "free n'a pas detache");
    TEST_ASSERT(data_packer_build_publish(&dpt_packer, 0u) == 0u, "build sur packer libere != 0");
    TEST_ASSERT(data_packer_init(&dpt_packer, 10u, 3u, src, DPT_PKT_CAP, dpt_packets) == DT_OK, "re-init apres free refusee");

    dpt_teardown();
    snprintf(tc->detail, sizeof(tc->detail), "10 refus sans abonne laisse ; init / free / init OK");
    tc->result = R_PASS;
}

/* ========================================================================
 * T1 – Paquet nominal
 * ======================================================================== */
void DP_seq_test_t1_basic_packet(TEST_case_t *tc) {
    tc->result = R_FAIL;
    dpt_setup(DPT_SRC_CAP);

    data_topic_t *src[3] = { &dpt_a, &dpt_b, &dpt_c };
    TEST_ASSERT(data_packer_init(&dpt_packer, 10u, 3u, src, DPT_PKT_CAP, dpt_packets) == DT_OK, "init");
    TEST_ASSERT(data_sub_attach(&dpt_consumer, &dpt_packer.topic, DATA_ATTACH_FROM_NOW) == DT_OK, "attach consommateur");

    const dpt_a_t a = { .ts = 1000u, .v = { 11u, 12u, 13u } };
    const dpt_b_t b = { .ts = 1001u, .v = 21u };
    dpt_c_t c = { .ts = 999u };
    for (uint32_t k = 0u; k < 15u; k++) {
        c.v[k] = 31u + k;
    }
    (void)data_topic_publish(&dpt_a, &a);
    (void)data_topic_publish(&dpt_b, &b);
    (void)data_topic_publish(&dpt_c, &c);

    const uint32_t flags = data_packer_build_publish(&dpt_packer, 1000u);
    TEST_ASSERT(flags == 0x7u, "flags=0x%lx attendu 0x7", (unsigned long)flags);
    dpt_pkt_abc_t p;
    memset(&p, 0xFF, sizeof(p));
    data_status_t s = data_sub_read(&dpt_consumer, &p);
    TEST_ASSERT(s == DT_OK, "lecture du paquet : %d", s);
    TEST_ASSERT(p.ts == 1000u && p.flags == 0x7u, "paquet : ts=%lu flags=0x%lx", (unsigned long)p.ts, (unsigned long)p.flags);
    TEST_ASSERT(memcmp(p.a, a.v, sizeof(a.v)) == 0 && p.b == b.v && memcmp(p.c, c.v, sizeof(c.v)) == 0,
                "charges utiles fausses");
    TEST_ASSERT(data_sub_num_to_read(&dpt_packer.subs[0]) == 0u && data_sub_num_to_read(&dpt_packer.subs[1]) == 0u &&
                data_sub_num_to_read(&dpt_packer.subs[2]) == 0u, "sources non consommees");

    dpt_teardown();
    snprintf(tc->detail, sizeof(tc->detail), "3 sources (16/8/64 o) : flags 0x7, horodatage et charges exacts");
    tc->result = R_PASS;
}

/* ========================================================================
 * T2 – Fenetre temporelle
 * ======================================================================== */
void DP_seq_test_t2_time_window(TEST_case_t *tc) {
    tc->result = R_FAIL;
    dpt_setup(DPT_SRC_CAP);

    data_topic_t *src[3] = { &dpt_a, &dpt_b, &dpt_c };
    TEST_ASSERT(data_packer_init(&dpt_packer, 10u, 3u, src, DPT_PKT_CAP, dpt_packets) == DT_OK, "init");
    TEST_ASSERT(data_sub_attach(&dpt_consumer, &dpt_packer.topic, DATA_ATTACH_FROM_NOW) == DT_OK, "attach consommateur");

    /* A : 980, 990 trop vieilles, 996 dans la fenetre, 1003 et 1010 restent.
       B : 1006 trop jeune (gardee). C : vide. */
    static const uint32_t ts_a[] = { 980u, 990u, 996u, 1003u, 1010u };
    for (size_t i = 0; i < 5u; i++) {
        dpt_pub_a(ts_a[i], ts_a[i]);
    }
    dpt_pub_b(1006u, 1006u);

    uint32_t flags = data_packer_build_publish(&dpt_packer, 1000u);
    dpt_pkt_abc_t p;
    TEST_ASSERT(flags == 0x1u, "t=1000 : flags=0x%lx attendu 0x1", (unsigned long)flags);
    TEST_ASSERT(data_sub_read(&dpt_consumer, &p) == DT_OK, "t=1000 : lecture");
    TEST_ASSERT(p.a[0] == 996u && p.b == 0u && dpt_all(p.c, 15u, 0u), "t=1000 : a=%lu b=%lu (attendu 996, 0, C a zero)",
                (unsigned long)p.a[0], (unsigned long)p.b);
    TEST_ASSERT(data_sub_num_to_read(&dpt_packer.subs[0]) == 2u, "A : %lu restantes, attendu 2 (1003, 1010)",
                (unsigned long)data_sub_num_to_read(&dpt_packer.subs[0]));
    TEST_ASSERT(data_sub_num_to_read(&dpt_packer.subs[1]) == 1u, "B : la trop jeune a ete consommee");

    flags = data_packer_build_publish(&dpt_packer, 1006u);
    TEST_ASSERT(flags == 0x3u, "t=1006 : flags=0x%lx attendu 0x3", (unsigned long)flags);
    TEST_ASSERT(data_sub_read(&dpt_consumer, &p) == DT_OK && p.a[0] == 1003u && p.b == 1006u,
                "t=1006 : a=%lu b=%lu attendu 1003, 1006", (unsigned long)p.a[0], (unsigned long)p.b);

    /* Bornes : |t - ts| = T/2 = 5 dans la fenetre, 6 hors. 1010 (A) est trop vieille a t=2000. */
    dpt_pub_a(1995u, 1995u);
    dpt_pub_b(2005u, 2005u);
    dpt_c_t c = { .ts = 2006u };
    (void)data_topic_publish(&dpt_c, &c);
    flags = data_packer_build_publish(&dpt_packer, 2000u);
    TEST_ASSERT(flags == 0x3u, "t=2000 : flags=0x%lx attendu 0x3 (C trop jeune)", (unsigned long)flags);
    TEST_ASSERT(data_sub_read(&dpt_consumer, &p) == DT_OK && p.a[0] == 1995u && p.b == 2005u,
                "t=2000 : a=%lu b=%lu attendu 1995, 2005", (unsigned long)p.a[0], (unsigned long)p.b);
    TEST_ASSERT(data_sub_num_to_read(&dpt_packer.subs[2]) == 1u, "C : la trop jeune a ete consommee");
    dpt_teardown();

    /* Horodatage qui repasse par 0 : t = 2, echantillons a -16 (trop vieux) et -2 (4 ms avant). */
    dpt_setup(DPT_SRC_CAP);
    data_topic_t *only_a[1] = { &dpt_a };
    TEST_ASSERT(data_packer_init(&dpt_packer, 10u, 1u, only_a, DPT_PKT_CAP, dpt_packets) == DT_OK, "init (A seul)");
    TEST_ASSERT(data_sub_attach(&dpt_consumer, &dpt_packer.topic, DATA_ATTACH_FROM_NOW) == DT_OK, "attach (A seul)");
    dpt_pub_a(0xFFFFFFF0u, 1u);
    dpt_pub_a(0xFFFFFFFEu, 2u);
    flags = data_packer_build_publish(&dpt_packer, 2u);
    dpt_pkt_a_t pa;
    TEST_ASSERT(flags == 0x1u, "passage par 0 : flags=0x%lx attendu 0x1", (unsigned long)flags);
    TEST_ASSERT(data_sub_read(&dpt_consumer, &pa) == DT_OK && pa.a[0] == 2u && pa.ts == 2u,
                "passage par 0 : a=%lu attendu 2", (unsigned long)pa.a[0]);

    dpt_teardown();
    snprintf(tc->detail, sizeof(tc->detail), "trop vieilles jetees, trop jeunes gardees, bornes T/2, passage par 2^32");
    tc->result = R_PASS;
}

/* ========================================================================
 * T3 – Paquets publies par copie, champs absents a zero
 * ======================================================================== */
void DP_seq_test_t3_copy_publish(TEST_case_t *tc) {
    tc->result = R_FAIL;
    dpt_setup(DPT_SRC_CAP);

    data_topic_t *src[2] = { &dpt_a, &dpt_b };
    TEST_ASSERT(data_packer_init(&dpt_packer, 10u, 2u, src, 3u, dpt_packets) == DT_OK, "init (capacite 3)");
    TEST_ASSERT(data_sub_attach(&dpt_consumer, &dpt_packer.topic, DATA_ATTACH_FROM_NOW) == DT_OK, "attach consommateur");

    for (uint32_t k = 0u; k < 10u; k++) {
        dpt_pub_a(100u + k, k);
        if ((k & 1u) == 0u) {
            dpt_pub_b(100u + k, 1000u + k);
        }
        const uint32_t flags = data_packer_build_publish(&dpt_packer, 100u + k);
        TEST_ASSERT(flags == (((k & 1u) == 0u) ? 0x3u : 0x1u), "paquet %lu : flags=0x%lx", (unsigned long)k, (unsigned long)flags);
    }

    /* Abonne en retard de 10 paquets sur 3 : perte, puis 7, 8, 9 intacts. */
    for (uint32_t k = 7u; k <= 9u; k++) {
        dpt_pkt_ab_t p;
        memset(&p, 0xFF, sizeof(p));
        const data_status_t s = data_sub_read(&dpt_consumer, &p);
        TEST_ASSERT(s == ((k == 7u) ? DT_DATA_LOSS : DT_OK), "paquet %lu : statut %d", (unsigned long)k, s);
        TEST_ASSERT(p.ts == 100u + k && dpt_all(p.a, 3u, k), "paquet %lu : ts=%lu a=%lu", (unsigned long)k,
                    (unsigned long)p.ts, (unsigned long)p.a[0]);
        if ((k & 1u) == 0u) {
            TEST_ASSERT(p.flags == 0x3u && p.b == 1000u + k, "paquet %lu : flags=0x%lx b=%lu", (unsigned long)k,
                        (unsigned long)p.flags, (unsigned long)p.b);
        } else {
            /* Le slot contenait le paquet k-3 : son B ne doit pas survivre. */
            TEST_ASSERT(p.flags == 0x1u && p.b == 0u, "paquet %lu : B absent mais b=%lu (reste du paquet %lu ?)",
                        (unsigned long)k, (unsigned long)p.b, (unsigned long)(k - 3u));
        }
    }

    dpt_teardown();
    snprintf(tc->detail, sizeof(tc->detail), "10 paquets dans 3 : LOSS puis 7,8,9 intacts, champ absent a zero");
    tc->result = R_PASS;
}

/* ========================================================================
 * T4 – Source liberee pendant l'usage
 * ======================================================================== */
void DP_seq_test_t4_source_freed(TEST_case_t *tc) {
    tc->result = R_FAIL;
    dpt_setup(DPT_SRC_CAP);

    data_topic_t *src[2] = { &dpt_a, &dpt_b };
    TEST_ASSERT(data_packer_init(&dpt_packer, 10u, 2u, src, DPT_PKT_CAP, dpt_packets) == DT_OK, "init");
    TEST_ASSERT(data_sub_attach(&dpt_consumer, &dpt_packer.topic, DATA_ATTACH_FROM_NOW) == DT_OK, "attach consommateur");

    dpt_pub_a(500u, 1u);
    dpt_pub_b(500u, 2u);
    TEST_ASSERT(data_packer_build_publish(&dpt_packer, 500u) == 0x3u, "avant free : flags != 0x3");

    data_topic_free(&dpt_b);
    TEST_ASSERT(!dpt_packer.subs[1].attached, "abonne du packer toujours attache a la source liberee");
    dpt_pub_a(501u, 3u);
    uint32_t flags = data_packer_build_publish(&dpt_packer, 501u);
    TEST_ASSERT(flags == 0x1u, "apres free : flags=0x%lx attendu 0x1", (unsigned long)flags);

    /* Source re-initialisee : le packer ne s'y rattache pas tout seul. */
    (void)data_topic_init(&dpt_b, dpt_b_store, sizeof(dpt_b_t), DPT_SRC_CAP, CB_OVERWRITE_OLDEST);
    dpt_pub_a(502u, 4u);
    dpt_pub_b(502u, 5u);
    flags = data_packer_build_publish(&dpt_packer, 502u);
    TEST_ASSERT(flags == 0x1u, "source re-initialisee : flags=0x%lx attendu 0x1", (unsigned long)flags);

    dpt_pkt_ab_t p;
    TEST_ASSERT(data_sub_read(&dpt_consumer, &p) == DT_OK && p.flags == 0x3u, "paquet 500");
    TEST_ASSERT(data_sub_read(&dpt_consumer, &p) == DT_OK && p.flags == 0x1u && p.a[0] == 3u && p.b == 0u, "paquet 501");
    TEST_ASSERT(data_sub_read(&dpt_consumer, &p) == DT_OK && p.flags == 0x1u && p.a[0] == 4u && p.b == 0u, "paquet 502");

    dpt_teardown();
    snprintf(tc->detail, sizeof(tc->detail), "source liberee : flag a 0 sans erreur, les autres continuent");
    tc->result = R_PASS;
}

/* ========================================================================
 * T5 – Concurrence avec une interruption
 * ======================================================================== */

typedef struct { uint32_t ts, flags; uint32_t a[3]; uint32_t c[15]; } dpt_pkt_ac_t;

static data_sub_t        dpt_isr_sub;
static dpt_c_t           dpt_isr_sample;      /* hors pile : la pile MSP est petite */
static dpt_pkt_ac_t      dpt_isr_pkt;
static volatile bool     dpt_isr_reader;      /* l'ISR relit aussi le topic des paquets */
static volatile uint32_t dpt_isr_n, dpt_isr_reads, dpt_isr_resync, dpt_isr_torn;

/* Paquet coherent : chaque champ present a tous ses mots egaux (et A non
   nul : numero de construction), chaque champ absent est a zero. */
static bool dpt_pkt_ac_ok(const dpt_pkt_ac_t *p) {
    if ((p->flags & ~0x3u) != 0u) return false;
    const bool a_ok = ((p->flags & 0x1u) != 0u) ? (p->a[0] != 0u && dpt_all(p->a, 3u, p->a[0])) : dpt_all(p->a, 3u, 0u);
    const bool c_ok = ((p->flags & 0x2u) != 0u) ? dpt_all(p->c, 15u, p->c[0]) : dpt_all(p->c, 15u, 0u);
    return a_ok && c_ok;
}

static void dpt_hook(void) {
    const uint32_t n = dpt_isr_n;
    if (!dpt_isr_reader || (n & 1u) == 0u) {
        dpt_isr_sample.ts = HAL_GetTick();
        for (uint32_t k = 0u; k < 15u; k++) {
            dpt_isr_sample.v[k] = n;
        }
        (void)data_topic_publish(&dpt_c, &dpt_isr_sample);
    } else {
        /* Abonne qui ne consomme jamais : chaque lecture le trouve depasse et
           le recale sur le plus ancien paquet, c'est-a-dire sur le slot que la
           prochaine publication du packer va remplacer. */
        const data_status_t s = data_sub_peek(&dpt_isr_sub, &dpt_isr_pkt, 0u);
        if (s == DT_OK || s == DT_DATA_LOSS) {
            dpt_isr_reads++;
            if (s == DT_DATA_LOSS) dpt_isr_resync++;
            if (!dpt_pkt_ac_ok(&dpt_isr_pkt)) dpt_isr_torn++;
        }
    }
    dpt_isr_n = n + 1u;
}

void DP_seq_test_t5_isr_concurrency(TEST_case_t *tc) {
    tc->result = R_FAIL;
    dpt_setup(4u);                                /* source C de 4 : le packer se fait depasser */
    memset(&dpt_isr_sub, 0, sizeof(dpt_isr_sub));
    dpt_isr_n = dpt_isr_reads = dpt_isr_resync = dpt_isr_torn = 0u;

    /* A : publiee par la boucle principale avant chaque paquet (toujours
       presente) ; C : publiee par l'ISR. */
    data_topic_t *src[2] = { &dpt_a, &dpt_c };
    TEST_ASSERT(data_packer_init(&dpt_packer, 1000u, 2u, src, DPT_PKT_CAP, dpt_packets) == DT_OK, "init");
    TEST_ASSERT(data_sub_attach(&dpt_consumer, &dpt_packer.topic, DATA_ATTACH_FROM_NOW) == DT_OK, "attach consommateur");
    TEST_ASSERT(data_sub_attach(&dpt_isr_sub, &dpt_packer.topic, DATA_ATTACH_FROM_NOW) == DT_OK, "attach abonne ISR");

    uint32_t built = 0u, reads = 0u, torn = 0u, with_c = 0u;
    dpt_pkt_ac_t p;

    /* Phase 1 : paquets construits sans pause, relus par l'ISR en retard de
       `capacity` paquets, donc dans le slot en cours de remplacement. */
    dpt_isr_reader = true;
    TEST_irq_start(20000u, dpt_hook);
    uint32_t t_end = HAL_GetTick() + 150u;
    while ((int32_t)(HAL_GetTick() - t_end) < 0) {
        dpt_pub_a(HAL_GetTick(), built + 1u);
        if ((data_packer_build_publish(&dpt_packer, HAL_GetTick()) & 0x2u) != 0u) with_c++;
        built++;
        while (data_sub_read(&dpt_consumer, &p) != DT_EMPTY) {
            reads++;
            if (!dpt_pkt_ac_ok(&p)) torn++;
        }
    }

    /* Phase 2 : l'ISR ne fait que publier C ; le packer attend que C ait fait
       plus d'un tour, puis lit l'echantillon que la publication suivante
       ecrase. */
    dpt_isr_reader = false;
    uint32_t seen = dpt_c.pub_seq;
    uint32_t rng = 0x2545F491u;
    t_end = HAL_GetTick() + 150u;
    while ((int32_t)(HAL_GetTick() - t_end) < 0) {
        while ((uint32_t)(dpt_c.pub_seq - seen) < 6u && (int32_t)(HAL_GetTick() - t_end) < 0) {
        }
        seen = dpt_c.pub_seq;
        /* Pause aleatoire : sans elle, la construction partirait toujours juste
           apres une publication de l'ISR, en phase avec elle, et ne serait
           presque jamais interrompue par la suivante. */
        rng = rng * 1664525u + 1013904223u;
        for (volatile uint32_t d = 0u; d < ((rng >> 8) % 600u); d++) {
        }
        dpt_pub_a(HAL_GetTick(), built + 1u);
        if ((data_packer_build_publish(&dpt_packer, HAL_GetTick()) & 0x2u) != 0u) with_c++;
        built++;
        while (data_sub_read(&dpt_consumer, &p) != DT_EMPTY) {
            reads++;
            if (!dpt_pkt_ac_ok(&p)) torn++;
        }
    }
    TEST_irq_stop();
    (void)data_sub_detach(&dpt_isr_sub);
    dpt_teardown();

    TEST_ASSERT(dpt_isr_n > 4000u, "ISR TIM5 : %lu appels seulement", (unsigned long)dpt_isr_n);
    TEST_ASSERT(torn == 0u && dpt_isr_torn == 0u, "Paquets incoherents : %lu / %lu relus, ISR %lu / %lu",
                (unsigned long)torn, (unsigned long)reads, (unsigned long)dpt_isr_torn, (unsigned long)dpt_isr_reads);
    TEST_ASSERT(built > 1000u && reads > 1000u && with_c > 500u, "Seulement %lu paquets (%lu avec C), %lu relus",
                (unsigned long)built, (unsigned long)with_c, (unsigned long)reads);
    TEST_ASSERT(dpt_isr_reads > 500u && dpt_isr_resync > 0u, "ISR : %lu lectures, %lu recalages : test non concluant",
                (unsigned long)dpt_isr_reads, (unsigned long)dpt_isr_resync);

    snprintf(tc->detail, sizeof(tc->detail), "%lu paquets (%lu avec C), %lu relus + %lu par l'ISR : 0 incoherent",
             (unsigned long)built, (unsigned long)with_c, (unsigned long)reads, (unsigned long)dpt_isr_reads);
    tc->result = R_PASS;
}

/* ========================================================================
 * T6 – Rejet borne face a un flot d'echantillons trop vieux
 * ======================================================================== */

static volatile uint32_t dpt_old_n;

static void dpt_hook_old(void) {
    dpt_isr_sample.ts = HAL_GetTick() - 10000u;   /* toujours hors fenetre */
    (void)data_topic_publish(&dpt_c, &dpt_isr_sample);
    dpt_old_n++;
}

void DP_seq_test_t6_bounded_discard(TEST_case_t *tc) {
    tc->result = R_FAIL;
    dpt_setup(4u);
    dpt_old_n = 0u;
    data_topic_t *src[2] = { &dpt_a, &dpt_c };
    TEST_ASSERT(data_packer_init(&dpt_packer, 1000u, 2u, src, DPT_PKT_CAP, dpt_packets) == DT_OK, "init");

    /* L'ISR publie sur C des echantillons trop vieux plus vite que le packer
       ne les rejette : sans borne, la boucle de rejet ne rendrait jamais la
       main (revue du 08/10, N4). Sous RTOS, la
       notification alourdit l'ISR : cadence reduite. */
#if (APEX_CFG_SCHED_RTOS == 1)
    TEST_irq_start(40000u, dpt_hook_old);
#else
    TEST_irq_start(100000u, dpt_hook_old);
#endif
    uint32_t built = 0u, with_c = 0u, max_ms = 0u;
    const uint32_t t_start = HAL_GetTick();
    while ((built < 500u || (HAL_GetTick() - t_start) < 50u) && built < 200000u) {
        dpt_pub_a(HAL_GetTick(), built + 1u);
        const uint32_t t0 = HAL_GetTick();
        if ((data_packer_build_publish(&dpt_packer, t0) & 0x2u) != 0u) with_c++;
        const uint32_t dt = HAL_GetTick() - t0;
        if (dt > max_ms) max_ms = dt;
        built++;
    }
    const uint32_t total_ms = HAL_GetTick() - t_start;
    TEST_irq_stop();
    const uint32_t published = dpt_old_n;
    dpt_teardown();

    TEST_ASSERT(published > 1000u, "ISR TIM5 : %lu publications seulement", (unsigned long)published);
    TEST_ASSERT(with_c == 0u, "%lu paquets avec C alors que tout C est trop vieux", (unsigned long)with_c);
    TEST_ASSERT(max_ms <= 2u, "un appel a dure %lu ms", (unsigned long)max_ms);
    snprintf(tc->detail, sizeof(tc->detail), "%lu paquets en %lu ms face a %lu echantillons trop vieux (ISR), appel <= %lu ms",
             (unsigned long)built, (unsigned long)total_ms, (unsigned long)published, (unsigned long)max_ms);
    tc->result = R_PASS;
}
