#include "dt_seq_test.h"
#include "test_irq.h"

#include "main.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/* Registre du topic == exactement les abonnes `expected` (ordre indifferent,
   chacun dans un seul slot, tous attaches a ce topic), et sub_count == n. */
static bool dts_subs_are(const data_topic_t *t, data_sub_t *const expected[], size_t n) {
    if (t->sub_count != n) {
        return false;
    }
    size_t used = 0u;
    for (size_t i = 0; i < DATA_TOPIC_MAX_SUBS; i++) {
        const data_sub_t *p = t->subs[i];
        if (p == NULL) {
            continue;
        }
        used++;
        bool known = false;
        for (size_t k = 0; k < n; k++) {
            known = known || (p == expected[k]);
        }
        if (!known || !p->attached || p->topic != t) {
            return false;
        }
    }
    for (size_t k = 0; k < n; k++) {
        size_t hits = 0u;
        for (size_t i = 0; i < DATA_TOPIC_MAX_SUBS; i++) {
            hits += (t->subs[i] == expected[k]) ? 1u : 0u;
        }
        if (hits != 1u) {
            return false;
        }
    }
    return used == n;
}

/* ========================================================================
 * Table des cas de test
 * ======================================================================== */

TEST_case_table_t DT_seq_test_cases[DT_seq_test_N_TESTS] = {
    { .case_info = { .name = "T0  NULL args"        }, .func = DT_seq_test_t0_null_args         },
    { .case_info = { .name = "T1  Pub/Read FIFO"    }, .func = DT_seq_test_t1_publish_read_fifo },
    { .case_info = { .name = "T2  Empty read"       }, .func = DT_seq_test_t2_empty_read        },
    { .case_info = { .name = "T3  Attach OLDEST"    }, .func = DT_seq_test_t3_attach_from_oldest},
    { .case_info = { .name = "T4  Attach NOW"       }, .func = DT_seq_test_t4_attach_from_now   },
    { .case_info = { .name = "T5  num_to_read"      }, .func = DT_seq_test_t5_num_to_read       },
    { .case_info = { .name = "T6  Two subscribers"  }, .func = DT_seq_test_t6_two_subscribers   },
    { .case_info = { .name = "T7  Data loss"        }, .func = DT_seq_test_t7_data_loss         },
    { .case_info = { .name = "T8  Sync"             }, .func = DT_seq_test_t8_sync              },
    { .case_info = { .name = "T9  Detach/Reattach"  }, .func = DT_seq_test_t9_detach_reattach   },
    { .case_info = { .name = "T10 Peek non-dest."   }, .func = DT_seq_test_t10_peek_non_destructive },
    { .case_info = { .name = "T11 REJECT_NEW refuse"}, .func = DT_seq_test_t11_reject_new_policy},
    { .case_info = { .name = "T12 Zero-copy ptr"    }, .func = DT_seq_test_t12_zero_copy_ptr    },
    { .case_info = { .name = "T13 Free detache"     }, .func = DT_seq_test_t13_free_detaches    },
    { .case_info = { .name = "T14 Bornes de perte"  }, .func = DT_seq_test_t14_loss_boundary    },
    { .case_info = { .name = "T15 Perte par peek(i)"}, .func = DT_seq_test_t15_loss_by_peek_idx },
    { .case_info = { .name = "T16 Perte par _ptr"   }, .func = DT_seq_test_t16_loss_by_ptr      },
    { .case_info = { .name = "T17 Peek hors donnees"}, .func = DT_seq_test_t17_peek_beyond_while_lagging },
    { .case_info = { .name = "T18 Detach milieu"    }, .func = DT_seq_test_t18_detach_middle    },
    { .case_info = { .name = "T19 pub_seq 2^32"     }, .func = DT_seq_test_t19_pub_seq_wrap     },
    { .case_info = { .name = "T20 OLDEST + cap 1"   }, .func = DT_seq_test_t20_oldest_after_wraps_cap1 },
    { .case_info = { .name = "T21 Tailles+temoins"  }, .func = DT_seq_test_t21_elem_sizes_guards },
    { .case_info = { .name = "T22 Attache, perime"  }, .func = DT_seq_test_t22_attach_refusals  },
    { .case_info = { .name = "T23 ISR dechirure"    }, .func = DT_seq_test_t23_isr_torn_read    },
    { .case_info = { .name = "T24 ISR pertes"       }, .func = DT_seq_test_t24_isr_loss_accounting },
    { .case_info = { .name = "T25 Detach perime"    }, .func = DT_seq_test_t25_stale_detach     },
    { .case_info = { .name = "T26 Detach perime inv"}, .func = DT_seq_test_t26_stale_detach_reverse },
    { .case_info = { .name = "T27 Abonne deplace"   }, .func = DT_seq_test_t27_sub_moved          },
    { .case_info = { .name = "T28 Registre plein"   }, .func = DT_seq_test_t28_registry_full      },
    { .case_info = { .name = "T29 Curseur incoherent"}, .func = DT_seq_test_t29_cursor_check      },
};

/* ========================================================================
 * T0 – Protection NULL / arguments invalides
 * ======================================================================== */
void DT_seq_test_t0_null_args(TEST_case_t *tc) {
    tc->result = R_FAIL;

    data_status_t s;

    /* data_topic_publish avec pointeurs NULL */
    uint32_t storage[4];
    data_topic_t topic;
    s = data_topic_init(NULL, storage, sizeof(uint32_t), 4, CB_OVERWRITE_OLDEST);
    TEST_ASSERT(s == DT_BAD_ARG, "init(NULL,...) retourne %d != DT_BAD_ARG", s);
    s = data_topic_init(&topic, NULL, sizeof(uint32_t), 4, CB_OVERWRITE_OLDEST);
    TEST_ASSERT(s == DT_BAD_ARG, "init(topic,NULL,...) retourne %d != DT_BAD_ARG", s);
    s = data_topic_publish(&topic, &(uint32_t){1u});
    TEST_ASSERT(s == DT_BAD_ARG, "publish sur topic non initialise retourne %d != DT_BAD_ARG", s);

    s = data_topic_init(&topic, storage, sizeof(uint32_t), 4, CB_OVERWRITE_OLDEST);
    TEST_ASSERT(s == DT_OK, "init valide retourne %d != DT_OK", s);

    s = data_topic_publish(NULL, &(uint32_t){1u});
    TEST_ASSERT(s == DT_BAD_ARG, "publish(NULL,val) retourne %d != DT_BAD_ARG", s);

    s = data_topic_publish(&topic, NULL);
    TEST_ASSERT(s == DT_BAD_ARG, "publish(topic,NULL) retourne %d != DT_BAD_ARG", s);

    /* data_sub_attach avec pointeurs NULL */
    data_sub_t sub = {0};
    s = data_sub_attach(NULL, &topic, DATA_ATTACH_FROM_NOW);
    TEST_ASSERT(s == DT_BAD_ARG, "attach(NULL,topic) retourne %d != DT_BAD_ARG", s);

    s = data_sub_attach(&sub, NULL, DATA_ATTACH_FROM_NOW);
    TEST_ASSERT(s == DT_BAD_ARG, "attach(sub,NULL) retourne %d != DT_BAD_ARG", s);

    /* data_sub_detach sur abonné non attaché ou NULL */
    s = data_sub_detach(NULL);
    TEST_ASSERT(s == DT_BAD_ARG, "detach(NULL) retourne %d != DT_BAD_ARG", s);
    /* sub.attached == 0 */
    s = data_sub_detach(&sub);
    TEST_ASSERT(s == DT_BAD_ARG, "detach(non-attache) retourne %d != DT_BAD_ARG", s);

    /* data_sub_sync sur NULL ou non attaché */
    s = data_sub_sync(NULL, DATA_ATTACH_FROM_NOW);
    TEST_ASSERT(s == DT_BAD_ARG, "sync(NULL) retourne %d != DT_BAD_ARG", s);
    s = data_sub_sync(&sub, DATA_ATTACH_FROM_NOW);
    TEST_ASSERT(s == DT_BAD_ARG, "sync(non-attache) retourne %d != DT_BAD_ARG", s);

    /* data_sub_read avec NULL */
    uint32_t out;
    s = data_sub_read(NULL, &out);
    TEST_ASSERT(s == DT_BAD_ARG, "read(NULL,out) retourne %d != DT_BAD_ARG", s);

    /* data_sub_read sur non attaché */
    s = data_sub_read(&sub, &out);
    TEST_ASSERT(s == DT_BAD_ARG, "read(non-attache,out) retourne %d != DT_BAD_ARG", s);

    /* Sorties NULL, sur un abonne attache */
    s = data_sub_attach(&sub, &topic, DATA_ATTACH_FROM_NOW);
    TEST_ASSERT(s == DT_OK, "attach valide retourne %d != DT_OK", s);
    s = data_sub_read(&sub, NULL);
    TEST_ASSERT(s == DT_BAD_ARG, "read(sub,NULL) retourne %d != DT_BAD_ARG", s);
    s = data_sub_peek(&sub, NULL, 0u);
    TEST_ASSERT(s == DT_BAD_ARG, "peek(sub,NULL) retourne %d != DT_BAD_ARG", s);
    s = data_sub_read_ptr(&sub, NULL);
    TEST_ASSERT(s == DT_BAD_ARG, "read_ptr(sub,NULL) retourne %d != DT_BAD_ARG", s);
    s = data_sub_peek_ptr(&sub, NULL, 0u);
    TEST_ASSERT(s == DT_BAD_ARG, "peek_ptr(sub,NULL) retourne %d != DT_BAD_ARG", s);
    s = data_sub_sync(&sub, (data_attach_mode_t)7);
    TEST_ASSERT(s == DT_BAD_ARG, "sync(mode invalide) retourne %d != DT_BAD_ARG", s);
    data_sub_detach(&sub);

    data_topic_free(&topic);
    snprintf(tc->detail, sizeof(tc->detail), "Tous les NULL/bad-arg correctement rejetes");
    tc->result = R_PASS;
}

/* ========================================================================
 * T1 – Publish / read FIFO basique
 * ======================================================================== */
void DT_seq_test_t1_publish_read_fifo(TEST_case_t *tc) {
    tc->result = R_FAIL;

    uint32_t storage[4];
    data_topic_t topic;
    data_topic_init(&topic, storage, sizeof(uint32_t), 4, CB_OVERWRITE_OLDEST);

    data_sub_t sub = {0};
    data_status_t s = data_sub_attach(&sub, &topic, DATA_ATTACH_FROM_NOW);
    TEST_ASSERT(s == DT_OK, "attach retourne %d != DT_OK", s);

    uint32_t in[3] = {10u, 20u, 30u};
    for (int i = 0; i < 3; i++) {
        s = data_topic_publish(&topic, &in[i]);
        TEST_ASSERT(s == DT_OK, "publish[%d] retourne %d != DT_OK", i, s);
    }
    TEST_ASSERT(topic.pub_seq == 3u, "pub_seq=%u != 3", (unsigned)topic.pub_seq);
    TEST_ASSERT(data_sub_num_to_read(&sub) == 3u,
              "num_to_read=%u != 3 apres 3 publish", (unsigned)data_sub_num_to_read(&sub));

    for (int i = 0; i < 3; i++) {
        uint32_t out = 0u;
        s = data_sub_read(&sub, &out);
        TEST_ASSERT(s   == DT_OK,  "read[%d] retourne %d != DT_OK", i, s);
        TEST_ASSERT(out == in[i],  "read[%d]=%u attendu %u", i, (unsigned)out, (unsigned)in[i]);
    }
    TEST_ASSERT(data_sub_num_to_read(&sub) == 0u,
              "num_to_read=%u != 0 apres 3 lectures", (unsigned)data_sub_num_to_read(&sub));

    data_sub_detach(&sub);
    data_topic_free(&topic);
    snprintf(tc->detail, sizeof(tc->detail), "FIFO {10,20,30} OK, pub_seq=3, num_to_read=0");
    tc->result = R_PASS;
}

/* ========================================================================
 * T2 – DT_EMPTY quand aucune donnée disponible
 * ======================================================================== */
void DT_seq_test_t2_empty_read(TEST_case_t *tc) {
    tc->result = R_FAIL;

    uint32_t storage[4];
    data_topic_t topic;
    data_topic_init(&topic, storage, sizeof(uint32_t), 4, CB_OVERWRITE_OLDEST);

    data_sub_t sub = {0};
    data_sub_attach(&sub, &topic, DATA_ATTACH_FROM_NOW);

    TEST_ASSERT(data_sub_num_to_read(&sub) == 0u,
              "num_to_read=%u != 0 antes de cualquier pub", (unsigned)data_sub_num_to_read(&sub));

    uint32_t out = 0xDEADBEEFu;
    data_status_t s = data_sub_read(&sub, &out);
    TEST_ASSERT(s == DT_EMPTY, "read sur vide retourne %d != DT_EMPTY", s);
    TEST_ASSERT(out == 0xDEADBEEFu, "out modifié alors que DT_EMPTY (out=0x%08X)", (unsigned)out);

    s = data_sub_peek(&sub, &out, 0u);
    TEST_ASSERT(s == DT_EMPTY, "peek sur vide retourne %d != DT_EMPTY", s);
    TEST_ASSERT(out == 0xDEADBEEFu, "out modifie par peek alors que DT_EMPTY (out=0x%08X)", (unsigned)out);

    data_sub_detach(&sub);
    data_topic_free(&topic);
    snprintf(tc->detail, sizeof(tc->detail), "DT_EMPTY pour read et peek, out non modifie");
    tc->result = R_PASS;
}

/* ========================================================================
 * T3 – DATA_ATTACH_FROM_OLDEST récupère l'historique existant
 * ======================================================================== */
void DT_seq_test_t3_attach_from_oldest(TEST_case_t *tc) {
    tc->result = R_FAIL;

    uint32_t storage[4];
    data_topic_t topic;
    data_topic_init(&topic, storage, sizeof(uint32_t), 4, CB_OVERWRITE_OLDEST);

    uint32_t in[3] = {10u, 20u, 30u};
    for (int i = 0; i < 3; i++) {
        data_topic_publish(&topic, &in[i]);
    }

    /* Attache APRES les 3 publications */
    data_sub_t sub = {0};
    data_status_t s = data_sub_attach(&sub, &topic, DATA_ATTACH_FROM_OLDEST);
    TEST_ASSERT(s == DT_OK, "attach FROM_OLDEST retourne %d != DT_OK", s);
    TEST_ASSERT(data_sub_num_to_read(&sub) == 3u,
              "num_to_read=%u != 3 apres ATTACH_FROM_OLDEST", (unsigned)data_sub_num_to_read(&sub));

    for (int i = 0; i < 3; i++) {
        uint32_t out = 0u;
        s = data_sub_read(&sub, &out);
        TEST_ASSERT(s   == DT_OK,  "read[%d] retourne %d != DT_OK", i, s);
        TEST_ASSERT(out == in[i],  "read[%d]=%u attendu %u", i, (unsigned)out, (unsigned)in[i]);
    }
    s = data_sub_read(&sub, &(uint32_t){0u});
    TEST_ASSERT(s == DT_EMPTY, "4eme read retourne %d != DT_EMPTY", s);

    data_sub_detach(&sub);
    data_topic_free(&topic);
    snprintf(tc->detail, sizeof(tc->detail), "FROM_OLDEST: num=3, {10,20,30} lus, DT_EMPTY OK");
    tc->result = R_PASS;
}

/* ========================================================================
 * T4 – DATA_ATTACH_FROM_NOW ignore l'historique existant
 * ======================================================================== */
void DT_seq_test_t4_attach_from_now(TEST_case_t *tc) {
    tc->result = R_FAIL;

    uint32_t storage[4];
    data_topic_t topic;
    data_topic_init(&topic, storage, sizeof(uint32_t), 4, CB_OVERWRITE_OLDEST);

    uint32_t in[3] = {10u, 20u, 30u};
    for (int i = 0; i < 3; i++) {
        data_topic_publish(&topic, &in[i]);
    }

    /* Attache APRES les 3 publications en mode FROM_NOW */
    data_sub_t sub = {0};
    data_sub_attach(&sub, &topic, DATA_ATTACH_FROM_NOW);
    TEST_ASSERT(data_sub_num_to_read(&sub) == 0u,
              "num_to_read=%u != 0 apres ATTACH_FROM_NOW", (unsigned)data_sub_num_to_read(&sub));

    data_status_t s = data_sub_read(&sub, &(uint32_t){0u});
    TEST_ASSERT(s == DT_EMPTY, "read avant any new pub retourne %d != DT_EMPTY", s);

    /* Publie une nouvelle valeur : l'abonné doit la voir */
    uint32_t v40 = 40u;
    data_topic_publish(&topic, &v40);
    TEST_ASSERT(data_sub_num_to_read(&sub) == 1u,
              "num_to_read=%u != 1 apres pub 40", (unsigned)data_sub_num_to_read(&sub));

    uint32_t out = 0u;
    s = data_sub_read(&sub, &out);
    TEST_ASSERT(s   == DT_OK,  "read apres pub 40 retourne %d != DT_OK", s);
    TEST_ASSERT(out == 40u,    "read=%u attendu 40", (unsigned)out);

    data_sub_detach(&sub);
    data_topic_free(&topic);
    snprintf(tc->detail, sizeof(tc->detail), "FROM_NOW: historique ignore, pub 40 lue OK");
    tc->result = R_PASS;
}

/* ========================================================================
 * T5 – data_sub_num_to_read suit le compteur de publications
 * ======================================================================== */
void DT_seq_test_t5_num_to_read(TEST_case_t *tc) {
    tc->result = R_FAIL;

    uint32_t storage[8];
    data_topic_t topic;
    data_topic_init(&topic, storage, sizeof(uint32_t), 8, CB_OVERWRITE_OLDEST);

    data_sub_t sub = {0};
    data_sub_attach(&sub, &topic, DATA_ATTACH_FROM_NOW);

    uint32_t v;
    v = 1u; data_topic_publish(&topic, &v);
    TEST_ASSERT(data_sub_num_to_read(&sub) == 1u, "[1pub] num=%u != 1",
              (unsigned)data_sub_num_to_read(&sub));

    v = 2u; data_topic_publish(&topic, &v);
    TEST_ASSERT(data_sub_num_to_read(&sub) == 2u, "[2pub] num=%u != 2",
              (unsigned)data_sub_num_to_read(&sub));

    v = 3u; data_topic_publish(&topic, &v);
    TEST_ASSERT(data_sub_num_to_read(&sub) == 3u, "[3pub] num=%u != 3",
              (unsigned)data_sub_num_to_read(&sub));

    /* Chaque read décrémente d'un */
    for (uint32_t expected = 2u; expected != (uint32_t)-1u; expected--) {
        data_sub_read(&sub, &(uint32_t){0u});
        uint32_t n = data_sub_num_to_read(&sub);
        TEST_ASSERT(n == expected, "apres read: num=%u attendu %u",
                  (unsigned)n, (unsigned)expected);
        if (expected == 0u) break;
    }

    data_sub_detach(&sub);
    data_topic_free(&topic);
    snprintf(tc->detail, sizeof(tc->detail), "num_to_read: 1->2->3 puis 2->1->0 apres reads OK");
    tc->result = R_PASS;
}

/* ========================================================================
 * T6 – Indépendance de deux abonnés
 * ======================================================================== */
void DT_seq_test_t6_two_subscribers(TEST_case_t *tc) {
    tc->result = R_FAIL;

    uint32_t storage[4];
    data_topic_t topic;
    data_topic_init(&topic, storage, sizeof(uint32_t), 4, CB_OVERWRITE_OLDEST);

    data_sub_t sub1 = {0}, sub2 = {0};
    data_sub_attach(&sub1, &topic, DATA_ATTACH_FROM_NOW);
    data_sub_attach(&sub2, &topic, DATA_ATTACH_FROM_NOW);
    TEST_ASSERT(topic.sub_count == 2u, "sub_count=%u != 2", (unsigned)topic.sub_count);

    uint32_t in[3] = {10u, 20u, 30u};
    for (int i = 0; i < 3; i++) data_topic_publish(&topic, &in[i]);

    /* sub1 lit les 2 premiers */
    for (int i = 0; i < 2; i++) {
        uint32_t out = 0u;
        data_status_t s = data_sub_read(&sub1, &out);
        TEST_ASSERT(s   == DT_OK,  "[sub1] read[%d] retourne %d != DT_OK", i, s);
        TEST_ASSERT(out == in[i],  "[sub1] read[%d]=%u attendu %u",
                  i, (unsigned)out, (unsigned)in[i]);
    }

    /* sub2 ne doit pas avoir avancé */
    TEST_ASSERT(data_sub_num_to_read(&sub2) == 3u,
              "[sub2] num=%u != 3 (avance par sub1?)", (unsigned)data_sub_num_to_read(&sub2));
    TEST_ASSERT(data_sub_num_to_read(&sub1) == 1u,
              "[sub1] num=%u != 1 apres 2 reads", (unsigned)data_sub_num_to_read(&sub1));

    /* sub2 lit les 3 éléments indépendamment */
    for (int i = 0; i < 3; i++) {
        uint32_t out = 0u;
        data_status_t s = data_sub_read(&sub2, &out);
        TEST_ASSERT(s   == DT_OK,  "[sub2] read[%d] retourne %d != DT_OK", i, s);
        TEST_ASSERT(out == in[i],  "[sub2] read[%d]=%u attendu %u",
                  i, (unsigned)out, (unsigned)in[i]);
    }
    TEST_ASSERT(data_sub_num_to_read(&sub2) == 0u,
              "[sub2] num=%u != 0 apres 3 reads", (unsigned)data_sub_num_to_read(&sub2));

    data_sub_detach(&sub1);
    data_sub_detach(&sub2);
    data_topic_free(&topic);
    snprintf(tc->detail, sizeof(tc->detail),
             "sub1={10,20} lu, sub2={10,20,30} intact, tails indep.");
    tc->result = R_PASS;
}


/* ========================================================================
 * T7 – DT_DATA_LOSS quand un abonné est dépassé
 * ======================================================================== */
void DT_seq_test_t7_data_loss(TEST_case_t *tc) {
    tc->result = R_FAIL;

#define CAP7 3u
    uint32_t storage[CAP7];
    data_topic_t topic;
    data_topic_init(&topic, storage, sizeof(uint32_t), CAP7, CB_OVERWRITE_OLDEST);

    data_sub_t sub = {0};
    data_sub_attach(&sub, &topic, DATA_ATTACH_FROM_NOW);

    /* Publie 1..5 : 1 et 2 sont écrasés avant d'être lus */
    for (uint32_t i = 1u; i <= 5u; i++) {
        data_topic_publish(&topic, &i);
    }
    TEST_ASSERT(data_sub_num_to_read(&sub) == 5u,
              "num_to_read=%u != 5 (> cap=%u) avant lecture",
              (unsigned)data_sub_num_to_read(&sub), (unsigned)CAP7);

    /* data_sub_read signale la perte et rend la plus ancienne valeur restante */
    uint32_t out = 0u;
    data_status_t s = data_sub_read(&sub, &out);
    TEST_ASSERT(s   == DT_DATA_LOSS, "1er read retourne %d != DT_DATA_LOSS", s);
    TEST_ASSERT(out == 3u,           "1er read=%u attendu 3 (plus ancienne restante)", (unsigned)out);

    /* La perte n'est signalée qu'une fois, la suite se lit normalement */
    TEST_ASSERT(data_sub_num_to_read(&sub) == 2u,
              "num_to_read=%u != 2 apres la lecture de 3", (unsigned)data_sub_num_to_read(&sub));
    for (uint32_t expected = 4u; expected <= 5u; expected++) {
        out = 0u;
        s = data_sub_read(&sub, &out);
        TEST_ASSERT(s   == DT_OK,    "read(%u) retourne %d != DT_OK", (unsigned)expected, s);
        TEST_ASSERT(out == expected, "read=%u attendu %u", (unsigned)out, (unsigned)expected);
    }
    s = data_sub_read(&sub, &out);
    TEST_ASSERT(s == DT_EMPTY, "read apres 5 retourne %d != DT_EMPTY", s);

    /* Même chose par peek : la perte est signalée par le premier accès */
    for (uint32_t i = 10u; i <= 14u; i++) {
        data_topic_publish(&topic, &i);
    }
    out = 0u;
    s = data_sub_peek(&sub, &out, 0u);
    TEST_ASSERT(s == DT_DATA_LOSS && out == 12u, "peek: s=%d out=%u attendu DT_DATA_LOSS/12", s, (unsigned)out);
    out = 0u;
    s = data_sub_read(&sub, &out);
    TEST_ASSERT(s == DT_OK && out == 12u, "read apres peek: s=%d out=%u attendu DT_OK/12", s, (unsigned)out);

    data_sub_detach(&sub);
    data_topic_free(&topic);
    snprintf(tc->detail, sizeof(tc->detail),
             "read: LOSS+3 puis 4,5 ; peek signale la perte une fois (12)");
    tc->result = R_PASS;
#undef CAP7
}

/* ========================================================================
 * T8 – data_sub_sync : FROM_NOW (tête) et FROM_OLDEST (plus ancien)
 * ======================================================================== */
void DT_seq_test_t8_sync(TEST_case_t *tc) {
    tc->result = R_FAIL;

    uint32_t storage[4];
    data_topic_t topic;
    data_topic_init(&topic, storage, sizeof(uint32_t), 4, CB_OVERWRITE_OLDEST);

    data_sub_t sub = {0};
    data_sub_attach(&sub, &topic, DATA_ATTACH_FROM_NOW);

    uint32_t in[3] = {10u, 20u, 30u};
    for (int i = 0; i < 3; i++) data_topic_publish(&topic, &in[i]);
    TEST_ASSERT(data_sub_num_to_read(&sub) == 3u,
              "num_to_read=%u != 3 avant sync", (unsigned)data_sub_num_to_read(&sub));

    /* FROM_NOW : l'abonné saute tout l'historique non lu */
    data_status_t s = data_sub_sync(&sub, DATA_ATTACH_FROM_NOW);
    TEST_ASSERT(s == DT_OK, "sync NOW retourne %d != DT_OK", s);
    TEST_ASSERT(data_sub_num_to_read(&sub) == 0u,
              "num_to_read=%u != 0 apres sync NOW", (unsigned)data_sub_num_to_read(&sub));
    TEST_ASSERT(sub.tail     == topic.cb.head,
              "sub.tail=%u != cb.head=%u apres sync NOW",
              (unsigned)sub.tail, (unsigned)topic.cb.head);
    TEST_ASSERT(sub.last_seq == topic.pub_seq,
              "sub.last_seq=%u != pub_seq=%u apres sync NOW",
              (unsigned)sub.last_seq, (unsigned)topic.pub_seq);

    s = data_sub_read(&sub, &(uint32_t){0u});
    TEST_ASSERT(s == DT_EMPTY, "read apres sync NOW retourne %d != DT_EMPTY", s);

    /* Publie 40 : lisible normalement après sync */
    uint32_t v40 = 40u;
    data_topic_publish(&topic, &v40);
    uint32_t out = 0u;
    s = data_sub_read(&sub, &out);
    TEST_ASSERT(s   == DT_OK, "read post-sync retourne %d != DT_OK", s);
    TEST_ASSERT(out == 40u,   "read post-sync=%u attendu 40", (unsigned)out);

    /* FROM_OLDEST : repart de la plus ancienne donnée présente, 10..40 */
    s = data_sub_sync(&sub, DATA_ATTACH_FROM_OLDEST);
    TEST_ASSERT(s == DT_OK, "sync OLDEST retourne %d != DT_OK", s);
    TEST_ASSERT(data_sub_num_to_read(&sub) == 4u,
              "num_to_read=%u != 4 apres sync OLDEST", (unsigned)data_sub_num_to_read(&sub));
    uint32_t expected[4] = {10u, 20u, 30u, 40u};
    for (int i = 0; i < 4; i++) {
        out = 0u;
        s = data_sub_read(&sub, &out);
        TEST_ASSERT(s == DT_OK && out == expected[i], "read[%d]: s=%d out=%u attendu %u",
                  i, s, (unsigned)out, (unsigned)expected[i]);
    }

    data_sub_detach(&sub);
    data_topic_free(&topic);
    snprintf(tc->detail, sizeof(tc->detail),
             "NOW: num=0, tail=head, 40 lu ; OLDEST: 10..40 relus");
    tc->result = R_PASS;
}

/* ========================================================================
 * T9 – data_sub_detach / sub_count / re-attachement
 * ======================================================================== */
void DT_seq_test_t9_detach_reattach(TEST_case_t *tc) {
    tc->result = R_FAIL;

    uint32_t storage[4];
    data_topic_t topic;
    data_topic_init(&topic, storage, sizeof(uint32_t), 4, CB_OVERWRITE_OLDEST);
    uint32_t storage_b[4];
    data_topic_t topic_b;
    data_topic_init(&topic_b, storage_b, sizeof(uint32_t), 4, CB_OVERWRITE_OLDEST);

    data_sub_t sub1 = {0}, sub2 = {0};
    data_sub_attach(&sub1, &topic, DATA_ATTACH_FROM_NOW);
    data_sub_attach(&sub2, &topic, DATA_ATTACH_FROM_NOW);
    TEST_ASSERT(topic.sub_count == 2u, "sub_count=%u != 2 apres 2 attach",
              (unsigned)topic.sub_count);

    /* Déjà attaché : même topic accepté sans effet, autre topic refusé */
    data_status_t s = data_sub_attach(&sub1, &topic, DATA_ATTACH_FROM_OLDEST);
    TEST_ASSERT(s == DT_OK && topic.sub_count == 2u,
              "re-attach meme topic: s=%d sub_count=%u", s, (unsigned)topic.sub_count);
    s = data_sub_attach(&sub1, &topic_b, DATA_ATTACH_FROM_NOW);
    TEST_ASSERT(s == DT_BAD_ARG, "attach a un 2e topic retourne %d != DT_BAD_ARG", s);
    TEST_ASSERT(sub1.topic == &topic && topic_b.sub_count == 0u, "sub1 deplace vers le 2e topic");

    /* Détache sub1 */
    s = data_sub_detach(&sub1);
    TEST_ASSERT(s == DT_OK, "detach sub1 retourne %d != DT_OK", s);
    TEST_ASSERT(topic.sub_count == 1u, "sub_count=%u != 1 apres detach",
              (unsigned)topic.sub_count);
    TEST_ASSERT(sub1.attached == 0, "sub1.attached=%d != 0", sub1.attached);

    /* sub1 détaché ne peut pas lire */
    s = data_sub_read(&sub1, &(uint32_t){0u});
    TEST_ASSERT(s == DT_BAD_ARG, "read sur sub1 detache retourne %d != DT_BAD_ARG", s);

    /* sub2 voit correctement la publication */
    uint32_t v99 = 99u;
    data_topic_publish(&topic, &v99);
    uint32_t out = 0u;
    s = data_sub_read(&sub2, &out);
    TEST_ASSERT(s   == DT_OK, "sub2 read retourne %d != DT_OK", s);
    TEST_ASSERT(out == 99u,   "sub2 read=%u attendu 99", (unsigned)out);

    /* Re-attache sub1 */
    s = data_sub_attach(&sub1, &topic, DATA_ATTACH_FROM_NOW);
    TEST_ASSERT(s == DT_OK, "re-attach sub1 retourne %d != DT_OK", s);
    TEST_ASSERT(topic.sub_count == 2u, "sub_count=%u != 2 apres re-attach",
              (unsigned)topic.sub_count);
    TEST_ASSERT(data_sub_num_to_read(&sub1) == 0u,
              "sub1 num_to_read=%u != 0 apres re-attach FROM_NOW",
              (unsigned)data_sub_num_to_read(&sub1));

    data_sub_detach(&sub1);
    data_sub_detach(&sub2);
    data_topic_free(&topic);
    data_topic_free(&topic_b);
    snprintf(tc->detail, sizeof(tc->detail),
             "Detach: sub_count=1, sub2 lit 99, re-attach OK, 2e topic refuse");
    tc->result = R_PASS;
}

/* ========================================================================
 * T10 – data_sub_peek : rang relatif au curseur, non destructif
 * ======================================================================== */
void DT_seq_test_t10_peek_non_destructive(TEST_case_t *tc) {
    tc->result = R_FAIL;

    uint32_t storage[4];
    data_topic_t topic;
    data_topic_init(&topic, storage, sizeof(uint32_t), 4, CB_OVERWRITE_OLDEST);

    /* 1 et 2 publiés AVANT l'attache : le curseur ne part pas du slot 0 */
    uint32_t v;
    v = 1u; data_topic_publish(&topic, &v);
    v = 2u; data_topic_publish(&topic, &v);

    data_sub_t sub = {0};
    data_sub_attach(&sub, &topic, DATA_ATTACH_FROM_NOW);

    uint32_t in[3] = {10u, 20u, 30u};
    for (int i = 0; i < 3; i++) data_topic_publish(&topic, &in[i]);

    size_t tail_before = sub.tail;

    /* peek(i) = i-ème donnée non lue par CET abonné */
    for (uint32_t i = 0; i < 3u; i++) {
        uint32_t out = 0u;
        data_status_t s = data_sub_peek(&sub, &out, i);
        TEST_ASSERT(s   == DT_OK, "peek[%u] retourne %d != DT_OK", (unsigned)i, s);
        TEST_ASSERT(out == in[i], "peek[%u]=%u attendu %u",
                  (unsigned)i, (unsigned)out, (unsigned)in[i]);
    }
    uint32_t out = 0xDEADBEEFu;
    data_status_t s = data_sub_peek(&sub, &out, 3u);
    TEST_ASSERT(s == DT_EMPTY, "peek[3] (au-dela des non lus) retourne %d != DT_EMPTY", s);
    TEST_ASSERT(out == 0xDEADBEEFu, "peek[3] a modifie out (0x%08X)", (unsigned)out);

    /* Non destructif : tail et num_to_read inchangés */
    TEST_ASSERT(sub.tail == tail_before,
              "sub.tail=%u modifie par peek (etait %u)",
              (unsigned)sub.tail, (unsigned)tail_before);
    TEST_ASSERT(data_sub_num_to_read(&sub) == 3u,
              "num_to_read=%u != 3 apres peeks", (unsigned)data_sub_num_to_read(&sub));

    /* data_sub_read suivant retourne le premier élément du FIFO, et peek(0)
       suit le curseur */
    out = 0u;
    s = data_sub_read(&sub, &out);
    TEST_ASSERT(s   == DT_OK, "read apres peeks retourne %d != DT_OK", s);
    TEST_ASSERT(out == 10u,   "read apres peeks=%u attendu 10 (tete FIFO)", (unsigned)out);
    out = 0u;
    s = data_sub_peek(&sub, &out, 0u);
    TEST_ASSERT(s == DT_OK && out == 20u, "peek[0] apres read: s=%d out=%u attendu 20", s, (unsigned)out);

    data_sub_detach(&sub);
    data_topic_free(&topic);
    snprintf(tc->detail, sizeof(tc->detail),
             "peek[0..2]={10,20,30}, peek[3] vide, tail intact, read->10");
    tc->result = R_PASS;
}

/* ========================================================================
 * T11 – CB_REJECT_NEW refusé sur un topic, OVERWRITE ne bloque jamais
 * ======================================================================== */
void DT_seq_test_t11_reject_new_policy(TEST_case_t *tc) {
    tc->result = R_FAIL;

    uint32_t storage[2];
    data_topic_t topic;
    data_sub_t sub = {0};

    data_status_t s = data_topic_init(&topic, storage, sizeof(uint32_t), 2, CB_REJECT_NEW);
    TEST_ASSERT(s == DT_BAD_ARG, "init REJECT_NEW retourne %d != DT_BAD_ARG", s);
    s = data_topic_publish(&topic, &(uint32_t){1u});
    TEST_ASSERT(s == DT_BAD_ARG, "publish sur topic refuse retourne %d != DT_BAD_ARG", s);
    s = data_sub_attach(&sub, &topic, DATA_ATTACH_FROM_NOW);
    TEST_ASSERT(s == DT_BAD_ARG, "attach sur topic refuse retourne %d != DT_BAD_ARG", s);

    /* Le scénario qui bloquait REJECT_NEW : remplir, tout lire, republier */
    s = data_topic_init(&topic, storage, sizeof(uint32_t), 2, CB_OVERWRITE_OLDEST);
    TEST_ASSERT(s == DT_OK, "init OVERWRITE retourne %d != DT_OK", s);
    data_sub_attach(&sub, &topic, DATA_ATTACH_FROM_NOW);

    uint32_t v, out;
    for (uint32_t round = 0; round < 5u; round++) {
        for (uint32_t k = 0; k < 2u; k++) {
            v = 10u * round + k;
            s = data_topic_publish(&topic, &v);
            TEST_ASSERT(s == DT_OK, "tour %u: publish %u retourne %d", (unsigned)round, (unsigned)v, s);
        }
        for (uint32_t k = 0; k < 2u; k++) {
            out = 0xFFFFFFFFu;
            s = data_sub_read(&sub, &out);
            TEST_ASSERT(s == DT_OK && out == 10u * round + k, "tour %u: read s=%d out=%u attendu %u",
                      (unsigned)round, s, (unsigned)out, (unsigned)(10u * round + k));
        }
    }
    TEST_ASSERT(topic.pub_seq == 10u, "pub_seq=%u != 10", (unsigned)topic.pub_seq);

    data_sub_detach(&sub);
    data_topic_free(&topic);
    snprintf(tc->detail, sizeof(tc->detail),
             "REJECT_NEW refuse ; OVERWRITE: 5 tours plein/vide sans blocage");
    tc->result = R_PASS;
}

/* ========================================================================
 * T12 – Accès zéro copie : peek_ptr / read_ptr
 * ======================================================================== */
void DT_seq_test_t12_zero_copy_ptr(TEST_case_t *tc) {
    tc->result = R_FAIL;

    uint32_t storage[4];
    data_topic_t topic;
    data_topic_init(&topic, storage, sizeof(uint32_t), 4, CB_OVERWRITE_OLDEST);

    data_sub_t sub = {0};
    data_sub_attach(&sub, &topic, DATA_ATTACH_FROM_NOW);
    for (uint32_t i = 1u; i <= 3u; i++) data_topic_publish(&topic, &i);

    /* peek_ptr(0) = prochaine donnée à lire (la plus ancienne non lue), pas la dernière publiée */
    const uint32_t *p = NULL;
    data_status_t s = data_sub_peek_ptr(&sub, (const void **)&p, 0u);
    TEST_ASSERT(s == DT_OK && p != NULL && *p == 1u, "peek_ptr(0): s=%d *p=%u attendu 1",
              s, p ? (unsigned)*p : 0u);
    s = data_sub_peek_ptr(&sub, (const void **)&p, 2u);
    TEST_ASSERT(s == DT_OK && *p == 3u, "peek_ptr(2): s=%d *p=%u attendu 3", s, (unsigned)*p);
    TEST_ASSERT(data_sub_num_to_read(&sub) == 3u, "peek_ptr a consomme (num=%u)",
              (unsigned)data_sub_num_to_read(&sub));

    /* read_ptr rend 1 et avance */
    s = data_sub_read_ptr(&sub, (const void **)&p);
    TEST_ASSERT(s == DT_OK && *p == 1u, "read_ptr: s=%d *p=%u attendu 1", s, (unsigned)*p);
    TEST_ASSERT((const void *)p >= (const void *)storage && (const void *)p < (const void *)(storage + 4),
              "read_ptr ne pointe pas dans le stockage du topic");
    s = data_sub_peek_ptr(&sub, (const void **)&p, 0u);
    TEST_ASSERT(s == DT_OK && *p == 2u, "peek_ptr(0) apres read_ptr: *p=%u attendu 2", (unsigned)*p);

    /* Vide : le pointeur de sortie n'est pas modifié */
    data_sub_sync(&sub, DATA_ATTACH_FROM_NOW);
    const uint32_t *sentinel = (const uint32_t *)0x1u;
    p = sentinel;
    s = data_sub_read_ptr(&sub, (const void **)&p);
    TEST_ASSERT(s == DT_EMPTY && p == sentinel, "read_ptr sur vide: s=%d, p modifie", s);

    data_sub_detach(&sub);
    data_topic_free(&topic);
    snprintf(tc->detail, sizeof(tc->detail),
             "peek_ptr(0)=1 (plus ancien), peek_ptr(2)=3, read_ptr=1 puis 2");
    tc->result = R_PASS;
}

/* ========================================================================
 * T13 – data_topic_free détache les abonnés, le topic se réinitialise
 * ======================================================================== */
void DT_seq_test_t13_free_detaches(TEST_case_t *tc) {
    tc->result = R_FAIL;

    uint32_t storage[4];
    data_topic_t topic;
    data_topic_init(&topic, storage, sizeof(uint32_t), 4, CB_OVERWRITE_OLDEST);

    data_sub_t sub1 = {0}, sub2 = {0};
    data_sub_attach(&sub1, &topic, DATA_ATTACH_FROM_NOW);
    data_sub_attach(&sub2, &topic, DATA_ATTACH_FROM_NOW);
    uint32_t v = 5u;
    data_topic_publish(&topic, &v);

    data_topic_free(&topic);

    TEST_ASSERT(sub1.attached == 0 && sub2.attached == 0,
              "abonnes encore attaches apres free (%d, %d)", sub1.attached, sub2.attached);
    TEST_ASSERT(dts_subs_are(&topic, NULL, 0u),
              "registre non vide apres free (sub_count=%u)", (unsigned)topic.sub_count);
    TEST_ASSERT(data_sub_num_to_read(&sub1) == 0u, "num_to_read=%u != 0 apres free",
              (unsigned)data_sub_num_to_read(&sub1));

    uint32_t out = 0u;
    data_status_t s = data_sub_read(&sub1, &out);
    TEST_ASSERT(s == DT_BAD_ARG, "read apres free retourne %d != DT_BAD_ARG", s);
    s = data_topic_publish(&topic, &v);
    TEST_ASSERT(s == DT_BAD_ARG, "publish apres free retourne %d != DT_BAD_ARG", s);
    s = data_sub_attach(&sub1, &topic, DATA_ATTACH_FROM_NOW);
    TEST_ASSERT(s == DT_BAD_ARG, "attach apres free retourne %d != DT_BAD_ARG", s);

    /* Réinitialisation : tout repart proprement */
    s = data_topic_init(&topic, storage, sizeof(uint32_t), 4, CB_OVERWRITE_OLDEST);
    TEST_ASSERT(s == DT_OK, "re-init retourne %d != DT_OK", s);
    s = data_sub_attach(&sub1, &topic, DATA_ATTACH_FROM_NOW);
    TEST_ASSERT(s == DT_OK, "attach apres re-init retourne %d != DT_OK", s);
    v = 6u;
    data_topic_publish(&topic, &v);
    s = data_sub_read(&sub1, &out);
    TEST_ASSERT(s == DT_OK && out == 6u, "read apres re-init: s=%d out=%u attendu 6", s, (unsigned)out);

    data_sub_detach(&sub1);
    data_topic_free(&topic);
    snprintf(tc->detail, sizeof(tc->detail),
             "free: 2 abonnes detaches, topic refuse, re-init + lecture OK");
    tc->result = R_PASS;
}

/* ========================================================================
 * Outils des cas limites et de stress (T14-T24)
 * ======================================================================== */

#define DTS_GUARD       16u
#define DTS_GUARD_BYTE  0x5Au

static void dts_guard_fill(uint8_t *area, size_t inner) {
    memset(area, DTS_GUARD_BYTE, DTS_GUARD);
    memset(area + DTS_GUARD + inner, DTS_GUARD_BYTE, DTS_GUARD);
}

static bool dts_guard_ok(const uint8_t *area, size_t inner) {
    for (size_t i = 0; i < DTS_GUARD; i++) {
        if (area[i] != DTS_GUARD_BYTE || area[DTS_GUARD + inner + i] != DTS_GUARD_BYTE) {
            return false;
        }
    }
    return true;
}

static uint8_t dts_pattern(uint32_t n, size_t i) {
    const uint32_t x = n * 2246822519u + (uint32_t)i * 3266489917u + 7u;
    return (uint8_t)(x ^ (x >> 11) ^ (x >> 23));
}

static void dts_fill(uint8_t *dst, uint32_t n, size_t size) {
    for (size_t i = 0; i < size; i++) {
        dst[i] = dts_pattern(n, i);
    }
}

static bool dts_check(const uint8_t *src, uint32_t n, size_t size) {
    for (size_t i = 0; i < size; i++) {
        if (src[i] != dts_pattern(n, i)) {
            return false;
        }
    }
    return true;
}

/* Publie first..last (uint32_t) ; rend le nombre de refus. */
static uint32_t dts_publish_range(data_topic_t *t, uint32_t first, uint32_t last) {
    uint32_t refused = 0u;
    for (uint32_t v = first; v <= last; v++) {
        if (data_topic_publish(t, &v) != DT_OK) {
            refused++;
        }
    }
    return refused;
}

/* Attente active : T23/T24 tournent aussi bien en sequentiel qu'en RTOS. */
static void dts_busy_wait_ms(uint32_t ms) {
    const uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < ms) {
    }
}

/* ========================================================================
 * T14 – Bornes de la perte
 * ======================================================================== */
void DT_seq_test_t14_loss_boundary(TEST_case_t *tc) {
    tc->result = R_FAIL;

    uint32_t storage[4];
    data_topic_t topic;
    data_status_t s = data_topic_init(&topic, storage, sizeof(uint32_t), 4u, CB_OVERWRITE_OLDEST);
    TEST_ASSERT(s == DT_OK, "init retourne %d", s);
    data_sub_t sub = {0};
    TEST_ASSERT(data_sub_attach(&sub, &topic, DATA_ATTACH_FROM_NOW) == DT_OK, "attach");

    /* lag == capacite : tout est encore present. */
    dts_publish_range(&topic, 1u, 4u);
    TEST_ASSERT(data_sub_num_to_read(&sub) == 4u, "lag=cap : num_to_read=%u", (unsigned)data_sub_num_to_read(&sub));
    for (uint32_t want = 1u; want <= 4u; want++) {
        uint32_t out = 0u;
        s = data_sub_read(&sub, &out);
        TEST_ASSERT(s == DT_OK && out == want, "lag=cap : read s=%d out=%u attendu DT_OK/%u", s, (unsigned)out, (unsigned)want);
    }

    /* lag == capacite + 1 : exactement une donnee perdue (5). */
    dts_publish_range(&topic, 5u, 9u);
    TEST_ASSERT(data_sub_num_to_read(&sub) == 5u, "lag=cap+1 : num_to_read=%u", (unsigned)data_sub_num_to_read(&sub));
    uint32_t out = 0u;
    s = data_sub_read(&sub, &out);
    TEST_ASSERT(s == DT_DATA_LOSS && out == 6u, "lag=cap+1 : read s=%d out=%u attendu DT_DATA_LOSS/6", s, (unsigned)out);
    for (uint32_t want = 7u; want <= 9u; want++) {
        out = 0u;
        s = data_sub_read(&sub, &out);
        TEST_ASSERT(s == DT_OK && out == want, "apres perte : s=%d out=%u attendu DT_OK/%u", s, (unsigned)out, (unsigned)want);
    }
    s = data_sub_read(&sub, &out);
    TEST_ASSERT(s == DT_EMPTY, "fin : read retourne %d != DT_EMPTY", s);

    data_sub_detach(&sub);
    data_topic_free(&topic);
    snprintf(tc->detail, sizeof(tc->detail), "lag=4 : 1..4 sans perte ; lag=5 : LOSS/6 puis 7,8,9");
    tc->result = R_PASS;
}

/* ========================================================================
 * T15 – Perte constatee par peek(idx > 0)
 * ======================================================================== */
void DT_seq_test_t15_loss_by_peek_idx(TEST_case_t *tc) {
    tc->result = R_FAIL;

    uint32_t storage[4];
    data_topic_t topic;
    data_topic_init(&topic, storage, sizeof(uint32_t), 4u, CB_OVERWRITE_OLDEST);
    data_sub_t sub = {0};
    TEST_ASSERT(data_sub_attach(&sub, &topic, DATA_ATTACH_FROM_NOW) == DT_OK, "attach");

    dts_publish_range(&topic, 1u, 7u);              /* 1, 2, 3 perdus ; 4..7 presents */
    uint32_t out = 0u;
    data_status_t s = data_sub_peek(&sub, &out, 2u);
    TEST_ASSERT(s == DT_DATA_LOSS && out == 6u, "peek(2) : s=%d out=%u attendu DT_DATA_LOSS/6", s, (unsigned)out);
    TEST_ASSERT(data_sub_num_to_read(&sub) == 4u, "pas recale : num_to_read=%u", (unsigned)data_sub_num_to_read(&sub));

    s = data_sub_peek(&sub, &out, 0u);
    TEST_ASSERT(s == DT_OK && out == 4u, "peek(0) : s=%d out=%u attendu DT_OK/4", s, (unsigned)out);
    s = data_sub_peek(&sub, &out, 3u);
    TEST_ASSERT(s == DT_OK && out == 7u, "peek(3) : s=%d out=%u attendu DT_OK/7", s, (unsigned)out);
    out = 0xDEADBEEFu;
    s = data_sub_peek(&sub, &out, 4u);
    TEST_ASSERT(s == DT_EMPTY && out == 0xDEADBEEFu, "peek(4) : s=%d, sortie touchee", s);
    for (uint32_t want = 4u; want <= 7u; want++) {
        s = data_sub_read(&sub, &out);
        TEST_ASSERT(s == DT_OK && out == want, "read : s=%d out=%u attendu DT_OK/%u", s, (unsigned)out, (unsigned)want);
    }

    data_sub_detach(&sub);
    data_topic_free(&topic);
    snprintf(tc->detail, sizeof(tc->detail), "peek(2) : LOSS/6 et recalage ; peek(0)=4, peek(3)=7, puis 4..7 lus");
    tc->result = R_PASS;
}

/* ========================================================================
 * T16 – Perte constatee par read_ptr et peek_ptr
 * ======================================================================== */
void DT_seq_test_t16_loss_by_ptr(TEST_case_t *tc) {
    tc->result = R_FAIL;

    uint32_t storage[4];
    data_topic_t topic;
    data_topic_init(&topic, storage, sizeof(uint32_t), 4u, CB_OVERWRITE_OLDEST);
    data_sub_t sub = {0};
    TEST_ASSERT(data_sub_attach(&sub, &topic, DATA_ATTACH_FROM_NOW) == DT_OK, "attach");

    dts_publish_range(&topic, 1u, 6u);              /* 1, 2 perdus ; 3..6 presents */
    const uint32_t *p = NULL;
    data_status_t s = data_sub_read_ptr(&sub, (const void **)&p);
    TEST_ASSERT(s == DT_DATA_LOSS && p != NULL && *p == 3u, "read_ptr : s=%d *p=%u attendu DT_DATA_LOSS/3",
                s, p ? (unsigned)*p : 0u);
    TEST_ASSERT(p >= storage && p < storage + 4, "read_ptr ne pointe pas dans le stockage");
    s = data_sub_peek_ptr(&sub, (const void **)&p, 0u);
    TEST_ASSERT(s == DT_OK && *p == 4u, "peek_ptr(0) : s=%d *p=%u attendu DT_OK/4", s, (unsigned)*p);

    dts_publish_range(&topic, 7u, 12u);             /* lag 9 > 4 : 9..12 presents */
    s = data_sub_peek_ptr(&sub, (const void **)&p, 1u);
    TEST_ASSERT(s == DT_DATA_LOSS && *p == 10u, "peek_ptr(1) : s=%d *p=%u attendu DT_DATA_LOSS/10", s, (unsigned)*p);
    TEST_ASSERT(data_sub_num_to_read(&sub) == 4u, "pas recale : num_to_read=%u", (unsigned)data_sub_num_to_read(&sub));
    s = data_sub_read_ptr(&sub, (const void **)&p);
    TEST_ASSERT(s == DT_OK && *p == 9u, "read_ptr apres recalage : s=%d *p=%u attendu DT_OK/9", s, (unsigned)*p);

    data_sub_detach(&sub);
    data_topic_free(&topic);
    snprintf(tc->detail, sizeof(tc->detail), "read_ptr LOSS/3, peek_ptr(1) LOSS/10, recalage puis 9");
    tc->result = R_PASS;
}

/* ========================================================================
 * T17 – peek au-dela des donnees presentes pendant un retard
 * ======================================================================== */
void DT_seq_test_t17_peek_beyond_while_lagging(TEST_case_t *tc) {
    tc->result = R_FAIL;

    uint32_t storage[4];
    data_topic_t topic;
    data_topic_init(&topic, storage, sizeof(uint32_t), 4u, CB_OVERWRITE_OLDEST);
    data_sub_t sub = {0};
    TEST_ASSERT(data_sub_attach(&sub, &topic, DATA_ATTACH_FROM_NOW) == DT_OK, "attach");

    dts_publish_range(&topic, 1u, 7u);              /* lag 7, 4 presents */
    uint32_t out = 0xDEADBEEFu;
    data_status_t s = data_sub_peek(&sub, &out, 4u);
    TEST_ASSERT(s == DT_EMPTY && out == 0xDEADBEEFu, "peek(4) : s=%d out=0x%08X attendu DT_EMPTY, sortie intacte",
                s, (unsigned)out);
    s = data_sub_peek(&sub, &out, 100u);
    TEST_ASSERT(s == DT_EMPTY && out == 0xDEADBEEFu, "peek(100) : s=%d, sortie touchee", s);
    TEST_ASSERT(data_sub_num_to_read(&sub) == 7u, "recale par un peek vide : num_to_read=%u",
                (unsigned)data_sub_num_to_read(&sub));
    s = data_sub_read(&sub, &out);
    TEST_ASSERT(s == DT_DATA_LOSS && out == 4u, "read : s=%d out=%u attendu DT_DATA_LOSS/4", s, (unsigned)out);

    data_sub_detach(&sub);
    data_topic_free(&topic);
    snprintf(tc->detail, sizeof(tc->detail), "peek(4)/peek(100) vides sans recalage (num=7), read -> LOSS/4");
    tc->result = R_PASS;
}

/* ========================================================================
 * T18 – Detach dans n'importe quel slot du registre
 * ======================================================================== */
void DT_seq_test_t18_detach_middle(TEST_case_t *tc) {
    tc->result = R_FAIL;

    uint32_t storage[4];
    data_topic_t topic;
    data_topic_init(&topic, storage, sizeof(uint32_t), 4u, CB_OVERWRITE_OLDEST);
    data_sub_t s0 = {0}, s1 = {0}, s2 = {0}, s3 = {0};
    data_sub_t *const all[4] = { &s0, &s1, &s2, &s3 };
    for (size_t i = 0; i < 4u; i++) {
        TEST_ASSERT(data_sub_attach(all[i], &topic, DATA_ATTACH_FROM_NOW) == DT_OK, "attach s%u", (unsigned)i);
    }
    TEST_ASSERT(dts_subs_are(&topic, (data_sub_t *const[]){ &s3, &s2, &s1, &s0 }, 4u), "registre initial != {s0..s3}");

    TEST_ASSERT(data_sub_detach(&s1) == DT_OK, "detach s1 (milieu)");
    TEST_ASSERT(dts_subs_are(&topic, (data_sub_t *const[]){ &s3, &s2, &s0 }, 3u), "apres detach s1 : registre != {s3,s2,s0}");
    TEST_ASSERT(!s1.attached && s1.topic == NULL, "s1 mal detache");

    uint32_t v = 42u, out;
    data_topic_publish(&topic, &v);
    data_sub_t *const rest[3] = { &s3, &s2, &s0 };
    for (size_t i = 0; i < 3u; i++) {
        out = 0u;
        data_status_t s = data_sub_read(rest[i], &out);
        TEST_ASSERT(s == DT_OK && out == 42u, "abonne restant %u : s=%d out=%u", (unsigned)i, s, (unsigned)out);
    }
    TEST_ASSERT(data_sub_read(&s1, &out) == DT_BAD_ARG, "s1 detache lit encore");

    TEST_ASSERT(data_sub_detach(&s3) == DT_OK, "detach s3 (tete)");
    TEST_ASSERT(dts_subs_are(&topic, (data_sub_t *const[]){ &s2, &s0 }, 2u), "apres detach s3 : registre != {s2,s0}");
    TEST_ASSERT(data_sub_detach(&s0) == DT_OK, "detach s0 (queue)");
    TEST_ASSERT(dts_subs_are(&topic, (data_sub_t *const[]){ &s2 }, 1u), "apres detach s0 : registre != {s2}");
    TEST_ASSERT(data_sub_detach(&s2) == DT_OK, "detach s2 (seul)");
    TEST_ASSERT(dts_subs_are(&topic, NULL, 0u), "registre non vide en fin (sub_count=%u)", (unsigned)topic.sub_count);

    /* Re-attache dans un autre ordre, puis deux detach du milieu. */
    data_sub_t *const order[4] = { &s1, &s3, &s0, &s2 };
    for (size_t i = 0; i < 4u; i++) {
        TEST_ASSERT(data_sub_attach(order[i], &topic, DATA_ATTACH_FROM_NOW) == DT_OK, "re-attach %u", (unsigned)i);
    }
    TEST_ASSERT(dts_subs_are(&topic, (data_sub_t *const[]){ &s2, &s0, &s3, &s1 }, 4u), "re-attache : registre != {s0..s3}");
    TEST_ASSERT(data_sub_detach(&s0) == DT_OK && data_sub_detach(&s3) == DT_OK, "detach s0, s3");
    TEST_ASSERT(dts_subs_are(&topic, (data_sub_t *const[]){ &s2, &s1 }, 2u), "registre != {s2,s1}");
    v = 43u;
    data_topic_publish(&topic, &v);
    TEST_ASSERT(data_sub_read(&s2, &out) == DT_OK && out == 43u, "s2 ne recoit pas 43");
    TEST_ASSERT(data_sub_read(&s1, &out) == DT_OK && out == 43u, "s1 ne recoit pas 43");

    data_sub_detach(&s2);
    data_sub_detach(&s1);
    data_topic_free(&topic);
    snprintf(tc->detail, sizeof(tc->detail), "detach de chaque position : registre et sub_count exacts");
    tc->result = R_PASS;
}

/* ========================================================================
 * T19 – Debordement de pub_seq
 * ======================================================================== */
void DT_seq_test_t19_pub_seq_wrap(TEST_case_t *tc) {
    tc->result = R_FAIL;

    uint32_t storage[16];
    data_topic_t topic;
    data_topic_init(&topic, storage, sizeof(uint32_t), 16u, CB_OVERWRITE_OLDEST);
    topic.pub_seq = UINT32_MAX - 5u;                /* comme apres 2^32 - 6 publications */

    data_sub_t sub = {0}, sub2 = {0};
    TEST_ASSERT(data_sub_attach(&sub, &topic, DATA_ATTACH_FROM_NOW) == DT_OK, "attach");
    dts_publish_range(&topic, 1u, 12u);
    TEST_ASSERT(topic.pub_seq == 6u, "pub_seq=%u apres passage par 0, attendu 6", (unsigned)topic.pub_seq);
    TEST_ASSERT(data_sub_num_to_read(&sub) == 12u, "num_to_read=%u attendu 12", (unsigned)data_sub_num_to_read(&sub));
    for (uint32_t want = 1u; want <= 12u; want++) {
        uint32_t out = 0u;
        data_status_t s = data_sub_read(&sub, &out);
        TEST_ASSERT(s == DT_OK && out == want, "read : s=%d out=%u attendu DT_OK/%u", s, (unsigned)out, (unsigned)want);
    }

    TEST_ASSERT(data_sub_attach(&sub2, &topic, DATA_ATTACH_FROM_OLDEST) == DT_OK, "attach FROM_OLDEST");
    TEST_ASSERT(data_sub_num_to_read(&sub2) == 12u, "FROM_OLDEST : num_to_read=%u attendu 12",
                (unsigned)data_sub_num_to_read(&sub2));
    uint32_t out = 0u;
    data_status_t s = data_sub_read(&sub2, &out);
    TEST_ASSERT(s == DT_OK && out == 1u, "FROM_OLDEST : read s=%d out=%u attendu DT_OK/1", s, (unsigned)out);
    data_sub_detach(&sub2);
    data_sub_detach(&sub);
    data_topic_free(&topic);

    /* Perte constatee de part et d'autre du passage par 0. */
    uint32_t small[4];
    data_topic_init(&topic, small, sizeof(uint32_t), 4u, CB_OVERWRITE_OLDEST);
    topic.pub_seq = UINT32_MAX - 2u;
    data_sub_t sub3 = {0};
    TEST_ASSERT(data_sub_attach(&sub3, &topic, DATA_ATTACH_FROM_NOW) == DT_OK, "attach (perte)");
    dts_publish_range(&topic, 1u, 7u);
    s = data_sub_read(&sub3, &out);
    TEST_ASSERT(s == DT_DATA_LOSS && out == 4u, "perte au passage par 0 : s=%d out=%u attendu DT_DATA_LOSS/4", s, (unsigned)out);
    for (uint32_t want = 5u; want <= 7u; want++) {
        s = data_sub_read(&sub3, &out);
        TEST_ASSERT(s == DT_OK && out == want, "apres perte : s=%d out=%u attendu DT_OK/%u", s, (unsigned)out, (unsigned)want);
    }
    data_sub_detach(&sub3);
    data_topic_free(&topic);

    snprintf(tc->detail, sizeof(tc->detail), "pub_seq repasse par 0 : num_to_read, FROM_OLDEST et perte exacts");
    tc->result = R_PASS;
}

/* ========================================================================
 * T20 – FROM_OLDEST apres plusieurs tours, topic de capacite 1
 * ======================================================================== */
void DT_seq_test_t20_oldest_after_wraps_cap1(TEST_case_t *tc) {
    tc->result = R_FAIL;

    uint32_t storage[4];
    data_topic_t topic;
    data_topic_init(&topic, storage, sizeof(uint32_t), 4u, CB_OVERWRITE_OLDEST);
    dts_publish_range(&topic, 1u, 10u);
    data_sub_t sub = {0};
    TEST_ASSERT(data_sub_attach(&sub, &topic, DATA_ATTACH_FROM_OLDEST) == DT_OK, "attach FROM_OLDEST");
    TEST_ASSERT(data_sub_num_to_read(&sub) == 4u, "num_to_read=%u attendu 4", (unsigned)data_sub_num_to_read(&sub));
    uint32_t out = 0u;
    data_status_t s;
    for (uint32_t want = 7u; want <= 10u; want++) {
        s = data_sub_read(&sub, &out);
        TEST_ASSERT(s == DT_OK && out == want, "OLDEST : s=%d out=%u attendu DT_OK/%u", s, (unsigned)out, (unsigned)want);
    }
    TEST_ASSERT(data_sub_read(&sub, &out) == DT_EMPTY, "OLDEST : pas vide en fin");
    data_sub_detach(&sub);
    data_topic_free(&topic);

    /* Capacite 1. */
    uint32_t one[1];
    TEST_ASSERT(data_topic_init(&topic, one, sizeof(uint32_t), 1u, CB_OVERWRITE_OLDEST) == DT_OK, "init cap 1");
    data_sub_t a = {0}, b = {0};
    TEST_ASSERT(data_sub_attach(&a, &topic, DATA_ATTACH_FROM_NOW) == DT_OK, "attach a");
    dts_publish_range(&topic, 1u, 1u);
    s = data_sub_read(&a, &out);
    TEST_ASSERT(s == DT_OK && out == 1u, "cap 1 : s=%d out=%u attendu DT_OK/1", s, (unsigned)out);
    dts_publish_range(&topic, 2u, 3u);
    s = data_sub_read(&a, &out);
    TEST_ASSERT(s == DT_DATA_LOSS && out == 3u, "cap 1 : s=%d out=%u attendu DT_DATA_LOSS/3", s, (unsigned)out);
    dts_publish_range(&topic, 4u, 4u);
    s = data_sub_peek(&a, &out, 0u);
    TEST_ASSERT(s == DT_OK && out == 4u, "cap 1 : peek s=%d out=%u attendu DT_OK/4", s, (unsigned)out);
    TEST_ASSERT(data_sub_attach(&b, &topic, DATA_ATTACH_FROM_OLDEST) == DT_OK, "attach b");
    TEST_ASSERT(data_sub_num_to_read(&b) == 1u, "cap 1 : b num_to_read=%u", (unsigned)data_sub_num_to_read(&b));
    s = data_sub_read(&b, &out);
    TEST_ASSERT(s == DT_OK && out == 4u, "cap 1 : b s=%d out=%u attendu DT_OK/4", s, (unsigned)out);
    dts_publish_range(&topic, 5u, 5u);
    TEST_ASSERT(data_sub_read(&a, &out) == DT_DATA_LOSS && out == 5u, "cap 1 : a n'a pas lu 4 : LOSS/5 attendu (out=%u)",
                (unsigned)out);
    TEST_ASSERT(data_sub_read(&b, &out) == DT_OK && out == 5u, "cap 1 : b doit lire 5");

    data_sub_detach(&a);
    data_sub_detach(&b);
    data_topic_free(&topic);
    snprintf(tc->detail, sizeof(tc->detail), "OLDEST apres 10/4 : 7..10 sans perte ; cap 1 : pertes, peek, 2 abonnes");
    tc->result = R_PASS;
}

/* ========================================================================
 * T21 – Elements de 13 et 128 octets, avec canaris
 * ======================================================================== */
void DT_seq_test_t21_elem_sizes_guards(TEST_case_t *tc) {
    tc->result = R_FAIL;

#define T21_CAP   3u
#define T21_MAXE  128u
    static const size_t sizes[] = { 13u, 128u };
    static uint8_t area[DTS_GUARD + T21_CAP * T21_MAXE + DTS_GUARD];
    static uint8_t outb[DTS_GUARD + T21_MAXE + DTS_GUARD];
    static uint8_t elem[T21_MAXE];

    for (size_t k = 0; k < sizeof(sizes) / sizeof(sizes[0]); k++) {
        const size_t es = sizes[k], inner = T21_CAP * es;
        uint8_t *const out = outb + DTS_GUARD;
        memset(area, 0, sizeof(area));
        dts_guard_fill(area, inner);
        dts_guard_fill(outb, es);

        data_topic_t topic;
        data_status_t s = data_topic_init(&topic, area + DTS_GUARD, es, T21_CAP, CB_OVERWRITE_OLDEST);
        TEST_ASSERT(s == DT_OK, "[%u o] init retourne %d", (unsigned)es, s);
        data_sub_t sub = {0};
        TEST_ASSERT(data_sub_attach(&sub, &topic, DATA_ATTACH_FROM_NOW) == DT_OK, "[%u o] attach", (unsigned)es);

        for (uint32_t n = 0u; n < 2u; n++) {
            dts_fill(elem, n, es);
            data_topic_publish(&topic, elem);
        }
        for (uint32_t n = 0u; n < 2u; n++) {
            memset(out, 0, es);
            s = data_sub_read(&sub, out);
            TEST_ASSERT(s == DT_OK && dts_check(out, n, es), "[%u o] read %u : s=%d ou contenu faux", (unsigned)es, (unsigned)n, s);
            TEST_ASSERT(dts_guard_ok(outb, es), "[%u o] temoin de sortie ecrase", (unsigned)es);
        }
        for (uint32_t n = 2u; n <= 6u; n++) {         /* 5 publications dans 3 : 2 et 3 perdus */
            dts_fill(elem, n, es);
            data_topic_publish(&topic, elem);
            TEST_ASSERT(dts_guard_ok(area, inner), "[%u o] temoin du stockage ecrase (publication %u)", (unsigned)es, (unsigned)n);
        }
        memset(out, 0, es);
        s = data_sub_peek(&sub, out, 1u);
        TEST_ASSERT(s == DT_DATA_LOSS && dts_check(out, 5u, es), "[%u o] peek(1) : s=%d ou contenu != 5", (unsigned)es, s);
        for (uint32_t n = 4u; n <= 6u; n++) {
            memset(out, 0, es);
            s = data_sub_read(&sub, out);
            TEST_ASSERT(s == DT_OK && dts_check(out, n, es), "[%u o] read %u : s=%d ou contenu faux", (unsigned)es, (unsigned)n, s);
            TEST_ASSERT(dts_guard_ok(outb, es), "[%u o] temoin de sortie ecrase", (unsigned)es);
        }
        TEST_ASSERT(dts_guard_ok(area, inner), "[%u o] temoin du stockage ecrase", (unsigned)es);
        data_sub_detach(&sub);
        data_topic_free(&topic);
    }
#undef T21_CAP
#undef T21_MAXE

    snprintf(tc->detail, sizeof(tc->detail), "13 et 128 o : lectures, perte, peek exacts ; temoins intacts");
    tc->result = R_PASS;
}

/* ========================================================================
 * T22 – Attache : mode invalide, slot perime
 * ======================================================================== */
void DT_seq_test_t22_attach_refusals(TEST_case_t *tc) {
    tc->result = R_FAIL;

    uint32_t storage[4];
    data_topic_t topic;
    data_topic_init(&topic, storage, sizeof(uint32_t), 4u, CB_OVERWRITE_OLDEST);
    data_sub_t sub1 = {0}, sub2 = {0}, sub3 = {0};
    TEST_ASSERT(data_sub_attach(&sub1, &topic, DATA_ATTACH_FROM_NOW) == DT_OK, "attach sub1");
    TEST_ASSERT(data_sub_attach(&sub2, &topic, DATA_ATTACH_FROM_NOW) == DT_OK, "attach sub2");

    /* Mode invalide (revue du 05/10, B3). */
    data_status_t s = data_sub_attach(&sub3, &topic, (data_attach_mode_t)7);
    TEST_ASSERT(s == DT_BAD_ARG && !sub3.attached && topic.sub_count == 2u, "mode 7 : s=%d attached=%d sub_count=%u",
                s, sub3.attached, (unsigned)topic.sub_count);

    /* sub1 remis a zero sans detach, comme une variable locale reutilisee : son
       slot est perime. La publication atteint toujours sub2, puis la
       re-attache de sub1 ne cree pas de doublon et compte l'anomalie (sous
       RTOS, la publication a deja libere le slot perime : meme etat final). */
    memset(&sub1, 0, sizeof(sub1));
    uint32_t v = 5u, out = 0u;
    s = data_topic_publish(&topic, &v);
    TEST_ASSERT(s == DT_OK, "publish retourne %d", s);
    s = data_sub_read(&sub2, &out);
    TEST_ASSERT(s == DT_OK && out == 5u, "sub2 : s=%d out=%u attendu 5", s, (unsigned)out);
    s = data_sub_attach(&sub1, &topic, DATA_ATTACH_FROM_NOW);
    TEST_ASSERT(s == DT_OK, "re-attache de sub1 remis a zero : s=%d", s);
    TEST_ASSERT(dts_subs_are(&topic, (data_sub_t *const[]){ &sub1, &sub2 }, 2u), "registre != {sub1,sub2} (sub_count=%u)",
                (unsigned)topic.sub_count);
    TEST_ASSERT(topic.list_faults == 1u, "list_faults=%u attendu 1", (unsigned)topic.list_faults);

    /* Idem pour sub2, re-attache FROM_OLDEST : il relit l'historique. */
    memset(&sub2, 0, sizeof(sub2));
    s = data_sub_attach(&sub2, &topic, DATA_ATTACH_FROM_OLDEST);
    TEST_ASSERT(s == DT_OK && dts_subs_are(&topic, (data_sub_t *const[]){ &sub1, &sub2 }, 2u) && topic.list_faults == 2u,
                "re-attache de sub2 remis a zero : s=%d sub_count=%u list_faults=%u", s, (unsigned)topic.sub_count,
                (unsigned)topic.list_faults);
    out = 0u;
    s = data_sub_read(&sub2, &out);
    TEST_ASSERT(s == DT_OK && out == 5u, "sub2 FROM_OLDEST : s=%d out=%u attendu 5", s, (unsigned)out);
    v = 6u;
    data_topic_publish(&topic, &v);
    data_status_t r1 = data_sub_read(&sub1, &out);
    const uint32_t out1 = out;
    data_status_t r2 = data_sub_read(&sub2, &out);
    TEST_ASSERT(r1 == DT_OK && out1 == 6u && r2 == DT_OK && out == 6u, "publication suivante : sub1 %d/%u, sub2 %d/%u",
                r1, (unsigned)out1, r2, (unsigned)out);

    /* Ecrasement partiel : seul `attached` est remis a zero, `topic` reste.
       Le slot reste invalide : pas de doublon a la re-attache. */
    sub1.attached = 0;
    s = data_sub_attach(&sub1, &topic, DATA_ATTACH_FROM_NOW);
    TEST_ASSERT(s == DT_OK && dts_subs_are(&topic, (data_sub_t *const[]){ &sub1, &sub2 }, 2u) && topic.list_faults == 3u,
                "re-attache apres ecrasement partiel : s=%d sub_count=%u list_faults=%u", s, (unsigned)topic.sub_count,
                (unsigned)topic.list_faults);

    TEST_ASSERT(data_sub_detach(&sub1) == DT_OK && data_sub_detach(&sub2) == DT_OK, "detach");
    TEST_ASSERT(dts_subs_are(&topic, NULL, 0u), "registre non vide en fin (sub_count=%u)", (unsigned)topic.sub_count);
    data_topic_free(&topic);
    snprintf(tc->detail, sizeof(tc->detail), "mode 7 refuse ; abonne remis a zero encore inscrit : slot repris sans doublon, anomalie comptee");
    tc->result = R_PASS;
}

/* ========================================================================
 * T23 / T24 – Publieur en interruption (TIM5, test_irq.h)
 *
 * Tout ce que touche l'ISR est statique, et le timer est arrete avant toute
 * assertion.
 * ======================================================================== */

typedef struct {
    uint32_t seq;
    uint32_t pad[30];                 /* = seq, sur toute la trame */
    uint32_t seq_end;                 /* = seq */
} dts_frame_t;                        /* 128 o */

static data_topic_t      dts_topic;
static data_sub_t        dts_sub;
static dts_frame_t       dts_frames[4];
static uint32_t          dts_values[8];
static dts_frame_t       dts_isr_frame;    /* hors pile : la pile MSP est petite */
static volatile uint32_t dts_isr_n;

/* T23 : une trame complete par interruption. */
static void dts_hook_frame(void) {
    const uint32_t n = dts_isr_n;
    dts_isr_frame.seq = n;
    for (uint32_t k = 0u; k < 30u; k++) {
        dts_isr_frame.pad[k] = n;
    }
    dts_isr_frame.seq_end = n;
    (void)data_topic_publish(&dts_topic, &dts_isr_frame);
    dts_isr_n = n + 1u;
}

/* T24 : 1, 2, 3... */
static void dts_hook_value(void) {
    const uint32_t v = dts_isr_n + 1u;
    (void)data_topic_publish(&dts_topic, &v);
    dts_isr_n = v;
}

/* ========================================================================
 * T23 – Lectures jamais dechirees
 * ======================================================================== */
void DT_seq_test_t23_isr_torn_read(TEST_case_t *tc) {
    tc->result = R_FAIL;

    memset(dts_frames, 0, sizeof(dts_frames));
    memset(&dts_sub, 0, sizeof(dts_sub));
    dts_isr_n = 0u;
    TEST_ASSERT(data_topic_init(&dts_topic, dts_frames, sizeof(dts_frame_t), 4u, CB_OVERWRITE_OLDEST) == DT_OK, "init");
    TEST_ASSERT(data_sub_attach(&dts_sub, &dts_topic, DATA_ATTACH_FROM_NOW) == DT_OK, "attach");

    uint32_t count = 0u, torn = 0u, bad = 0u, losses = 0u, prev = 0u;
    uint32_t rng = 0x9E3779B9u;
    bool have_prev = false;
    dts_frame_t f;

    TEST_irq_start(20000u, dts_hook_frame);
    const uint32_t t_end = HAL_GetTick() + 300u;
    while ((int32_t)(HAL_GetTick() - t_end) < 0) {
        /* Lire au bord : topic plein vu de l'abonne, la trame lue est celle que
           la prochaine publication ecrase. Pause aleatoire pour ne pas lire en
           phase avec l'ISR (juste apres chaque publication). */
        if (data_sub_num_to_read(&dts_sub) < 4u) continue;
        rng = rng * 1664525u + 1013904223u;
        for (volatile uint32_t d = 0u; d < ((rng >> 8) % 400u); d++) {
        }
        const data_status_t s = data_sub_read(&dts_sub, &f);
        if (s == DT_EMPTY) continue;
        if (s != DT_OK && s != DT_DATA_LOSS) { bad++; continue; }

        bool intact = (f.seq == f.seq_end);
        for (uint32_t k = 0u; k < 30u && intact; k++) {
            intact = (f.pad[k] == f.seq);
        }
        if (!intact) torn++;
        if (have_prev) {
            if (s == DT_OK && f.seq != prev + 1u) bad++;
            if (s == DT_DATA_LOSS) { if (f.seq <= prev + 1u) bad++; else losses++; }
        }
        prev      = f.seq;
        have_prev = true;
        count++;
    }
    TEST_irq_stop();
    const uint32_t published = dts_isr_n;
    data_sub_detach(&dts_sub);
    data_topic_free(&dts_topic);

    TEST_ASSERT(published > 4000u, "ISR TIM5 : %lu publications seulement", (unsigned long)published);
    TEST_ASSERT(torn == 0u, "%lu trame(s) dechiree(s) sur %lu lues", (unsigned long)torn, (unsigned long)count);
    TEST_ASSERT(bad == 0u, "%lu lecture(s) hors sequence ou statut inattendu", (unsigned long)bad);
    TEST_ASSERT(count > 1000u, "Seulement %lu trames lues", (unsigned long)count);
    TEST_ASSERT(losses > 0u, "Aucune perte : l'abonne n'a jamais ete depasse, test non concluant");

    snprintf(tc->detail, sizeof(tc->detail), "%lu trames lues / %lu publiees en ISR, 0 dechiree, %lu pertes signalees",
             (unsigned long)count, (unsigned long)published, (unsigned long)losses);
    tc->result = R_PASS;
}

/* ========================================================================
 * T24 – Comptabilite des pertes face a un publieur en interruption
 * ======================================================================== */
void DT_seq_test_t24_isr_loss_accounting(TEST_case_t *tc) {
    tc->result = R_FAIL;

    memset(&dts_sub, 0, sizeof(dts_sub));
    dts_isr_n = 0u;
    TEST_ASSERT(data_topic_init(&dts_topic, dts_values, sizeof(uint32_t), 8u, CB_OVERWRITE_OLDEST) == DT_OK, "init");
    TEST_ASSERT(data_sub_attach(&dts_sub, &dts_topic, DATA_ATTACH_FROM_NOW) == DT_OK, "attach");

    uint32_t count = 0u, skipped = 0u, losses = 0u, bad = 0u, prev = 0u, v;

    TEST_irq_start(20000u, dts_hook_value);
    const uint32_t t_end = HAL_GetTick() + 300u;
    bool running = true;
    for (;;) {
        if (running && (int32_t)(HAL_GetTick() - t_end) >= 0) {
            TEST_irq_stop();                          /* puis vidange de ce qui reste */
            running = false;
        }
        const data_status_t s = data_sub_read(&dts_sub, &v);
        if (s == DT_EMPTY) {
            if (!running) break;
            continue;
        }
        if (s == DT_OK) {
            if (v != prev + 1u) bad++;
        } else if (s == DT_DATA_LOSS) {
            losses++;
            if (v <= prev + 1u) bad++;
            else                skipped += v - prev - 1u;
        } else {
            bad++;
            break;
        }
        prev = v;
        count++;
        if ((count & 31u) == 0u) {
            dts_busy_wait_ms(1u);                     /* 20 publications > capacite 8 */
        }
    }
    TEST_irq_stop();
    const uint32_t published = dts_isr_n;
    data_sub_detach(&dts_sub);
    data_topic_free(&dts_topic);

    TEST_ASSERT(published > 4000u, "ISR TIM5 : %lu publications seulement", (unsigned long)published);
    TEST_ASSERT(bad == 0u, "%lu trou(s) non annonce(s) ou statut inattendu", (unsigned long)bad);
    TEST_ASSERT(prev == published, "derniere valeur lue %lu != derniere publiee %lu", (unsigned long)prev, (unsigned long)published);
    TEST_ASSERT(count + skipped == published, "lus %lu + sautes %lu != publies %lu",
                (unsigned long)count, (unsigned long)skipped, (unsigned long)published);
    TEST_ASSERT(losses > 0u, "Aucune perte : test non concluant");

    snprintf(tc->detail, sizeof(tc->detail), "%lu publiees en ISR = %lu lues + %lu sautees (%lu pertes signalees)",
             (unsigned long)published, (unsigned long)count, (unsigned long)skipped, (unsigned long)losses);
    tc->result = R_PASS;
}

/* ========================================================================
 * T25 – Detach apres une re-initialisation fautive du topic
 * ======================================================================== */
void DT_seq_test_t25_stale_detach(TEST_case_t *tc) {
    tc->result = R_FAIL;

    uint32_t storage[4];
    data_topic_t topic;
    data_topic_init(&topic, storage, sizeof(uint32_t), 4u, CB_OVERWRITE_OLDEST);
    data_sub_t old1 = {0}, old2 = {0}, fresh = {0};
    TEST_ASSERT(data_sub_attach(&old1, &topic, DATA_ATTACH_FROM_NOW) == DT_OK, "attach old1");
    TEST_ASSERT(data_sub_attach(&old2, &topic, DATA_ATTACH_FROM_NOW) == DT_OK, "attach old2");

    /* Erreur d'usage : re-init avec les abonnes encore attaches (il fallait free). */
    TEST_ASSERT(data_topic_init(&topic, storage, sizeof(uint32_t), 4u, CB_OVERWRITE_OLDEST) == DT_OK, "re-init");
    TEST_ASSERT(data_sub_attach(&fresh, &topic, DATA_ATTACH_FROM_NOW) == DT_OK, "attach fresh");

    data_status_t s2 = data_sub_detach(&old2);
    data_status_t s1 = data_sub_detach(&old1);
    TEST_ASSERT(s2 == DT_OK && s1 == DT_OK && !old1.attached && !old2.attached, "detach des anciens : %d / %d", s2, s1);
    TEST_ASSERT(dts_subs_are(&topic, (data_sub_t *const[]){ &fresh }, 1u),
                "registre du topic reinitialise abime (sub_count=%u)", (unsigned)topic.sub_count);
    TEST_ASSERT(topic.list_faults == 2u, "list_faults=%u attendu 2", (unsigned)topic.list_faults);

    uint32_t v = 7u, out = 0u;
    data_topic_publish(&topic, &v);
    data_status_t s = data_sub_read(&fresh, &out);
    TEST_ASSERT(s == DT_OK && out == 7u, "fresh : s=%d out=%u attendu 7", s, (unsigned)out);
    TEST_ASSERT(data_sub_detach(&fresh) == DT_OK && dts_subs_are(&topic, NULL, 0u),
                "detach fresh : registre non vide (sub_count=%u)", (unsigned)topic.sub_count);
    TEST_ASSERT(topic.list_faults == 2u, "detach normal compte une anomalie (%u)", (unsigned)topic.list_faults);

    data_topic_free(&topic);
    snprintf(tc->detail, sizeof(tc->detail), "detach apres re-init fautive : nouveau registre intact, 2 anomalies comptees");
    tc->result = R_PASS;
}

/* ========================================================================
 * T26 – Re-initialisation fautive, detach des anciens dans l'ordre inverse
 * ======================================================================== */
void DT_seq_test_t26_stale_detach_reverse(TEST_case_t *tc) {
    tc->result = R_FAIL;

    uint32_t storage[4];
    data_topic_t topic;
    data_topic_init(&topic, storage, sizeof(uint32_t), 4u, CB_OVERWRITE_OLDEST);
    data_sub_t old1 = {0}, old2 = {0}, fresh = {0}, late = {0};
    TEST_ASSERT(data_sub_attach(&old1, &topic, DATA_ATTACH_FROM_NOW) == DT_OK, "attach old1");
    TEST_ASSERT(data_sub_attach(&old2, &topic, DATA_ATTACH_FROM_NOW) == DT_OK, "attach old2");

    /* Erreur d'usage : re-init avec les abonnes encore attaches (il fallait free). */
    TEST_ASSERT(data_topic_init(&topic, storage, sizeof(uint32_t), 4u, CB_OVERWRITE_OLDEST) == DT_OK, "re-init");
    TEST_ASSERT(data_sub_attach(&fresh, &topic, DATA_ATTACH_FROM_NOW) == DT_OK, "attach fresh");

    data_status_t s1 = data_sub_detach(&old1);      /* le plus ancien d'abord */
    data_status_t s2 = data_sub_detach(&old2);
    TEST_ASSERT(s1 == DT_OK && s2 == DT_OK && !old1.attached && !old2.attached, "detach des anciens : %d / %d", s1, s2);
    TEST_ASSERT(topic.sub_count == 1u, "sub_count=%u attendu 1 (fresh seul)", (unsigned)topic.sub_count);
    TEST_ASSERT(topic.list_faults == 2u, "list_faults=%u attendu 2", (unsigned)topic.list_faults);

    /* Le topic doit toujours accepter un nouvel abonne. */
    data_status_t s = data_sub_attach(&late, &topic, DATA_ATTACH_FROM_NOW);
    TEST_ASSERT(s == DT_OK && topic.sub_count == 2u, "attache apres detach perimes : s=%d sub_count=%u", s,
                (unsigned)topic.sub_count);

    uint32_t v = 9u, out1 = 0u, out2 = 0u;
    data_topic_publish(&topic, &v);
    data_status_t r1 = data_sub_read(&fresh, &out1);
    data_status_t r2 = data_sub_read(&late, &out2);
    TEST_ASSERT(r1 == DT_OK && out1 == 9u && r2 == DT_OK && out2 == 9u, "lectures : fresh %d/%u, late %d/%u",
                r1, (unsigned)out1, r2, (unsigned)out2);
    TEST_ASSERT(data_sub_detach(&fresh) == DT_OK && data_sub_detach(&late) == DT_OK && topic.sub_count == 0u,
                "detach normaux : sub_count=%u", (unsigned)topic.sub_count);
    TEST_ASSERT(topic.list_faults == 2u, "detach normaux comptent une anomalie (%u)", (unsigned)topic.list_faults);

    data_topic_free(&topic);
    snprintf(tc->detail, sizeof(tc->detail), "ordre inverse : sub_count exact, attache suivante acceptee, 2 anomalies");
    tc->result = R_PASS;
}

/* ========================================================================
 * T27 – Abonne remis a zero puis rattache a un autre topic
 * ======================================================================== */
void DT_seq_test_t27_sub_moved(TEST_case_t *tc) {
    tc->result = R_FAIL;

    uint32_t store_a[4], store_b[4];
    data_topic_t ta, tb;
    data_topic_init(&ta, store_a, sizeof(uint32_t), 4u, CB_OVERWRITE_OLDEST);
    data_topic_init(&tb, store_b, sizeof(uint32_t), 4u, CB_OVERWRITE_OLDEST);
    data_sub_t a1 = {0}, a2 = {0}, a3 = {0}, a4 = {0}, b1 = {0};
    TEST_ASSERT(data_sub_attach(&a1, &ta, DATA_ATTACH_FROM_NOW) == DT_OK &&
                data_sub_attach(&a2, &ta, DATA_ATTACH_FROM_NOW) == DT_OK &&
                data_sub_attach(&a3, &ta, DATA_ATTACH_FROM_NOW) == DT_OK &&
                data_sub_attach(&b1, &tb, DATA_ATTACH_FROM_NOW) == DT_OK, "attaches initiales");

    /* Erreur d'usage : a2 remis a zero sans detach, puis rattache a tb. Son
       slot dans ta est perime. */
    memset(&a2, 0, sizeof(a2));
    TEST_ASSERT(data_sub_attach(&a2, &tb, DATA_ATTACH_FROM_NOW) == DT_OK, "a2 vers tb");
    TEST_ASSERT(dts_subs_are(&tb, (data_sub_t *const[]){ &b1, &a2 }, 2u), "registre tb != {b1,a2}");
    const data_sub_t a2_before = a2;

    /* Operations sur ta : le slot perime est libere une fois (anomalie
       comptee), a2 n'est jamais ecrit au titre de ta, tb n'est pas touche. */
    uint32_t v = 11u, out = 0u;
    data_topic_publish(&ta, &v);
    TEST_ASSERT(data_sub_attach(&a4, &ta, DATA_ATTACH_FROM_NOW) == DT_OK, "attache de a4 sur ta");
    TEST_ASSERT(dts_subs_are(&ta, (data_sub_t *const[]){ &a1, &a3, &a4 }, 3u), "registre ta != {a1,a3,a4} (sub_count=%u)",
                (unsigned)ta.sub_count);
    TEST_ASSERT(ta.list_faults == 1u, "ta : list_faults=%u attendu 1", (unsigned)ta.list_faults);
    TEST_ASSERT(memcmp(&a2, &a2_before, sizeof(a2)) == 0, "a2 ecrit par une operation sur ta");
    TEST_ASSERT(tb.list_faults == 0u && dts_subs_are(&tb, (data_sub_t *const[]){ &b1, &a2 }, 2u), "tb modifie par ta");

    /* Chaque abonne lit son propre topic. */
    TEST_ASSERT(data_sub_read(&a1, &out) == DT_OK && out == 11u, "a1 ne lit pas 11");
    TEST_ASSERT(data_sub_read(&a3, &out) == DT_OK && out == 11u, "a3 ne lit pas 11");
    TEST_ASSERT(data_sub_read(&a4, &out) == DT_EMPTY, "a4 (FROM_NOW) lit une donnee anterieure");
    v = 22u;
    data_topic_publish(&tb, &v);
    TEST_ASSERT(data_sub_read(&b1, &out) == DT_OK && out == 22u, "b1 ne lit pas 22");
    TEST_ASSERT(data_sub_read(&a2, &out) == DT_OK && out == 22u, "a2 ne lit pas 22 sur tb");
    TEST_ASSERT(data_sub_read(&a1, &out) == DT_EMPTY, "a1 lit une publication de tb");
    v = 33u;
    data_topic_publish(&ta, &v);
    TEST_ASSERT(data_sub_read(&a4, &out) == DT_OK && out == 33u, "a4 ne lit pas 33");
    TEST_ASSERT(data_sub_read(&a2, &out) == DT_EMPTY, "a2 lit une publication de ta");

    /* free de ta alors que a1, remis a zero, est passe sur tb : a1 n'est pas
       touche et reste attache a tb ; a3 et a4 sont detaches. */
    memset(&a1, 0, sizeof(a1));
    TEST_ASSERT(data_sub_attach(&a1, &tb, DATA_ATTACH_FROM_NOW) == DT_OK, "a1 vers tb");
    const data_sub_t a1_before = a1;
    data_topic_free(&ta);
    TEST_ASSERT(memcmp(&a1, &a1_before, sizeof(a1)) == 0 && a1.attached && a1.topic == &tb, "free de ta a ecrit dans a1");
    TEST_ASSERT(!a3.attached && !a4.attached, "free de ta n'a pas detache a3 / a4");
    TEST_ASSERT(ta.list_faults == 2u, "ta : list_faults=%u attendu 2", (unsigned)ta.list_faults);
    v = 44u;
    data_topic_publish(&tb, &v);
    TEST_ASSERT(data_sub_read(&a1, &out) == DT_OK && out == 44u, "a1 ne lit pas 44 sur tb");
    TEST_ASSERT(data_sub_read(&a2, &out) == DT_OK && out == 44u, "a2 ne lit pas 44 sur tb");
    TEST_ASSERT(data_sub_read(&b1, &out) == DT_OK && out == 44u, "b1 ne lit pas 44 sur tb");

    TEST_ASSERT(data_sub_detach(&a1) == DT_OK && data_sub_detach(&a2) == DT_OK && data_sub_detach(&b1) == DT_OK, "detach");
    TEST_ASSERT(dts_subs_are(&tb, NULL, 0u) && tb.list_faults == 0u, "tb : registre non vide ou %u anomalies en fin",
                (unsigned)tb.list_faults);
    data_topic_free(&tb);
    snprintf(tc->detail, sizeof(tc->detail), "slots perimes liberes (attach, free), abonnes deplaces jamais ecrits, tb intact");
    tc->result = R_PASS;
}

/* ========================================================================
 * T28 – Registre plein
 * ======================================================================== */
void DT_seq_test_t28_registry_full(TEST_case_t *tc) {
    tc->result = R_FAIL;

    uint32_t storage[4];
    data_topic_t topic;
    data_topic_init(&topic, storage, sizeof(uint32_t), 4u, CB_OVERWRITE_OLDEST);
    static data_sub_t subs[DATA_TOPIC_MAX_SUBS + 1u];   /* hors pile */
    memset(subs, 0, sizeof(subs));
    data_sub_t *const extra = &subs[DATA_TOPIC_MAX_SUBS];

    for (size_t i = 0; i < DATA_TOPIC_MAX_SUBS; i++) {
        TEST_ASSERT(data_sub_attach(&subs[i], &topic, DATA_ATTACH_FROM_NOW) == DT_OK, "attach %u", (unsigned)i);
    }
    TEST_ASSERT(topic.sub_count == DATA_TOPIC_MAX_SUBS, "sub_count=%u", (unsigned)topic.sub_count);
    data_status_t s = data_sub_attach(extra, &topic, DATA_ATTACH_FROM_NOW);
    TEST_ASSERT(s == DT_NO_SLOT && !extra->attached && topic.sub_count == DATA_TOPIC_MAX_SUBS,
                "attache de trop : s=%d attached=%d sub_count=%u", s, extra->attached, (unsigned)topic.sub_count);

    uint32_t v = 7u, out = 0u;
    data_topic_publish(&topic, &v);
    for (size_t i = 0; i < DATA_TOPIC_MAX_SUBS; i++) {
        TEST_ASSERT(data_sub_read(&subs[i], &out) == DT_OK && out == 7u, "abonne %u ne lit pas 7", (unsigned)i);
    }

    /* Un slot libere par detach est repris. */
    TEST_ASSERT(data_sub_detach(&subs[3]) == DT_OK, "detach 3");
    TEST_ASSERT(data_sub_attach(extra, &topic, DATA_ATTACH_FROM_NOW) == DT_OK, "attache apres detach");
    TEST_ASSERT(data_sub_attach(&subs[3], &topic, DATA_ATTACH_FROM_NOW) == DT_NO_SLOT, "plein de nouveau : DT_NO_SLOT attendu");

    /* Un slot perime (abonne remis a zero sans detach) est recupere par une
       attache quand le registre est plein. */
    memset(&subs[0], 0, sizeof(subs[0]));
    s = data_sub_attach(&subs[3], &topic, DATA_ATTACH_FROM_NOW);
    TEST_ASSERT(s == DT_OK && topic.sub_count == DATA_TOPIC_MAX_SUBS && topic.list_faults == 1u,
                "recuperation du slot perime : s=%d sub_count=%u list_faults=%u", s, (unsigned)topic.sub_count,
                (unsigned)topic.list_faults);
    v = 8u;
    data_topic_publish(&topic, &v);
    for (size_t i = 1; i <= DATA_TOPIC_MAX_SUBS; i++) {
        TEST_ASSERT(data_sub_read(&subs[i], &out) == DT_OK && out == 8u, "abonne %u ne lit pas 8", (unsigned)i);
    }

    for (size_t i = 1; i <= DATA_TOPIC_MAX_SUBS; i++) {
        TEST_ASSERT(data_sub_detach(&subs[i]) == DT_OK, "detach final %u", (unsigned)i);
    }
    TEST_ASSERT(dts_subs_are(&topic, NULL, 0u), "registre non vide en fin (sub_count=%u)", (unsigned)topic.sub_count);
    data_topic_free(&topic);
    snprintf(tc->detail, sizeof(tc->detail), "%u slots : DT_NO_SLOT au-dela, slot libere repris, slot perime recupere",
             (unsigned)DATA_TOPIC_MAX_SUBS);
    tc->result = R_PASS;
}

/* ========================================================================
 * T29 – Curseur incoherent (repli de pub_seq - last_seq)
 * ======================================================================== */
void DT_seq_test_t29_cursor_check(TEST_case_t *tc) {
    tc->result = R_FAIL;

    uint32_t storage[3];
    data_topic_t topic;
    data_topic_init(&topic, storage, sizeof(uint32_t), 3u, CB_OVERWRITE_OLDEST);
    data_sub_t sub = {0};
    TEST_ASSERT(data_sub_attach(&sub, &topic, DATA_ATTACH_FROM_NOW) == DT_OK, "attach");
    dts_publish_range(&topic, 1u, 2u);

    /* 2^32 + 2 publications sans lecture donneraient le meme retard (2) mais
       un head decale de 2^32 mod 3 = 1 slot : on reproduit ce decalage sur le
       curseur. Sans controle, read rendrait DT_OK sur un slot faux. */
    sub.tail = (sub.tail + 2u) % 3u;
    uint32_t out = 0u;
    data_status_t s = data_sub_read(&sub, &out);
    TEST_ASSERT(s == DT_DATA_LOSS && out == 1u, "curseur incoherent : read s=%d out=%u (attendu DT_DATA_LOSS/1)", s, (unsigned)out);
    s = data_sub_read(&sub, &out);
    TEST_ASSERT(s == DT_OK && out == 2u, "apres recalage : s=%d out=%u (attendu DT_OK/2)", s, (unsigned)out);
    TEST_ASSERT(data_sub_read(&sub, &out) == DT_EMPTY, "donnee en trop apres recalage");

    /* Meme chose constatee par peek(idx > 0). */
    dts_publish_range(&topic, 3u, 4u);              /* stockage : 2, 3, 4 */
    sub.tail = (sub.tail + 1u) % 3u;
    s = data_sub_peek(&sub, &out, 1u);
    TEST_ASSERT(s == DT_DATA_LOSS && out == 3u, "peek(1) curseur incoherent : s=%d out=%u (attendu DT_DATA_LOSS/3)", s, (unsigned)out);
    for (uint32_t want = 2u; want <= 4u; want++) {
        s = data_sub_read(&sub, &out);
        TEST_ASSERT(s == DT_OK && out == want, "apres peek : read s=%d out=%u (attendu DT_OK/%u)", s, (unsigned)out, (unsigned)want);
    }
    TEST_ASSERT(data_sub_read(&sub, &out) == DT_EMPTY, "donnee en trop en fin");

    data_sub_detach(&sub);
    data_topic_free(&topic);
    snprintf(tc->detail, sizeof(tc->detail), "curseur decale (repli 2^32, cap 3) : DT_DATA_LOSS + recalage, read et peek");
    tc->result = R_PASS;
}
