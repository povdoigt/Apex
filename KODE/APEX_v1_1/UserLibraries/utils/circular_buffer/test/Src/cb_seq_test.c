#include "cb_seq_test.h"
#include "test_irq.h"

#include "main.h"

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <limits.h>
#include <stdbool.h>

/* ========================================================================
 * Table des cas de test
 * ======================================================================== */

TEST_case_table_t CB_seq_test_cases[CB_seq_test_N_TESTS] = {
    { .case_info = { .name = "T0 NULL args"       }, .func = CB_seq_test_t0_null_args         },
    { .case_info = { .name = "T1 FIFO order"      }, .func = CB_seq_test_t1_fifo_order        },
    { .case_info = { .name = "T2 Empty read"      }, .func = CB_seq_test_t2_empty_read        },
    { .case_info = { .name = "T3 Full REJECT_NEW" }, .func = CB_seq_test_t3_full_reject       },
    { .case_info = { .name = "T4 Full OVERWRITE"  }, .func = CB_seq_test_t4_full_overwrite    },
    { .case_info = { .name = "T5 Reset"           }, .func = CB_seq_test_t5_reset             },
    { .case_info = { .name = "T6 Peek absolute"   }, .func = CB_seq_test_t6_peek_absolute     },
    { .case_info = { .name = "T7 Peek relative"   }, .func = CB_seq_test_t7_peek_relative     },
    { .case_info = { .name = "T8 Wrap-around"     }, .func = CB_seq_test_t8_wraparound        },
    { .case_info = { .name = "T9 Float elem_size" }, .func = CB_seq_test_t9_float_elemsize    },
    { .case_info = { .name = "T10 Fill/Drain x2"  }, .func = CB_seq_test_t10_fill_drain_cycle },
    { .case_info = { .name = "T11 Capacite 1"     }, .func = CB_seq_test_t11_capacity_one     },
    { .case_info = { .name = "T12 Ecrasements"    }, .func = CB_seq_test_t12_multi_overwrite  },
    { .case_info = { .name = "T13 Sortie de plein"}, .func = CB_seq_test_t13_reject_recovery  },
    { .case_info = { .name = "T14 Tailles+temoins"}, .func = CB_seq_test_t14_elem_sizes_guards},
    { .case_info = { .name = "T15 Acces pointeur" }, .func = CB_seq_test_t15_ptr_accessors    },
    { .case_info = { .name = "T16 Wrap extremes"  }, .func = CB_seq_test_t16_wrap_extremes    },
    { .case_info = { .name = "T17 Args invalides" }, .func = CB_seq_test_t17_bad_args         },
    { .case_info = { .name = "T18 Push en place"  }, .func = CB_seq_test_t18_push_in_place    },
    { .case_info = { .name = "T19 Aleatoire/modele"}, .func = CB_seq_test_t19_random_model    },
    { .case_info = { .name = "T20 Push ISR"       }, .func = CB_seq_test_t20_isr_push         },
    { .case_info = { .name = "T21 Copies vs ISR"  }, .func = CB_seq_test_t21_isr_no_torn_copy },
};

#define CAP6 4u
#define CAP7 5u

/* ========================================================================
 * T0 – Protection NULL / arguments invalides
 * ======================================================================== */
void CB_seq_test_t0_null_args(TEST_case_t *tc) {
    tc->result = R_FAIL;

    /* cb_init avec arguments invalides – doit retourner CB_BAD_ARG sans crasher */
    cb_status_t s;
    s = cb_init(NULL, NULL, sizeof(uint32_t), 4, CB_REJECT_NEW);
    TEST_ASSERT(s == CB_BAD_ARG, "cb_init(NULL,NULL,...) retourne %d != CB_BAD_ARG", s);

    uint8_t storage[4 * sizeof(uint32_t)];
    s = cb_init(NULL, storage, sizeof(uint32_t), 4, CB_REJECT_NEW);
    TEST_ASSERT(s == CB_BAD_ARG, "cb_init(NULL,storage,...) retourne %d != CB_BAD_ARG", s);

    circular_buffer_t cb;
    s = cb_init(&cb, NULL, sizeof(uint32_t), 4, CB_REJECT_NEW);
    TEST_ASSERT(s == CB_BAD_ARG, "cb_init(cb,NULL,...) retourne %d != CB_BAD_ARG", s);
    s = cb_init(&cb, storage, 0,             4, CB_REJECT_NEW);
    TEST_ASSERT(s == CB_BAD_ARG, "cb_init(cb,storage,0,...) retourne %d != CB_BAD_ARG", s);
    s = cb_init(&cb, storage, sizeof(uint32_t), 0, CB_REJECT_NEW);
    TEST_ASSERT(s == CB_BAD_ARG, "cb_init(cb,...,cap=0,...) retourne %d != CB_BAD_ARG", s);

    /* cb_push / cb_pop avec NULL cb */
    uint32_t val = 42;

    s = cb_push(NULL, &val);
    TEST_ASSERT(s == CB_BAD_ARG, "cb_push(NULL,val) retourne %d != CB_BAD_ARG", s);

    s = cb_pop(NULL, &val);
    TEST_ASSERT(s == CB_BAD_ARG, "cb_pop(NULL,out) retourne %d != CB_BAD_ARG", s);

    /* cb_push avec elem NULL sur buffer valide */
    s = cb_init(&cb, storage, sizeof(uint32_t), 4, CB_REJECT_NEW);
    TEST_ASSERT(s == CB_OK, "cb_init valide retourne %d != CB_OK", s);
    s = cb_push(&cb, NULL);
    TEST_ASSERT(s == CB_BAD_ARG, "cb_push(cb,NULL) retourne %d != CB_BAD_ARG", s);
    s = cb_free(&cb);
    TEST_ASSERT(s == CB_OK, "cb_free retourne %d != CB_OK", s);

    snprintf(tc->detail, sizeof(tc->detail), "Tous les NULL/bad-arg correctement rejetes");
    tc->result = R_PASS;
}

/* ========================================================================
 * T1 – Ordre FIFO basique
 * ======================================================================== */
void CB_seq_test_t1_fifo_order(TEST_case_t *tc) {
    tc->result = R_FAIL;

    uint32_t storage[3];
    circular_buffer_t cb;
    cb_status_t s = cb_init(&cb, storage, sizeof(uint32_t), 3, CB_REJECT_NEW);
    TEST_ASSERT(s == CB_OK, "cb_init retourne %d != CB_OK", s);

    uint32_t in[3] = {10u, 20u, 30u};

    for (int i = 0; i < 3; i++) {
        s = cb_push(&cb, &in[i]);
        TEST_ASSERT(s == CB_OK, "push[%d] retourne %d != CB_OK", i, s);
    }
    TEST_ASSERT(cb.count == 3u, "count=%u != 3 apres 3 push", (unsigned)cb.count);

    uint32_t out;
    for (int i = 0; i < 3; i++) {
        s = cb_pop(&cb, &out);
        TEST_ASSERT(s == CB_OK,       "pop[%d] retourne %d != CB_OK", i, s);
        TEST_ASSERT(out == in[i],     "pop[%d]=%u attendu %u", i, (unsigned)out, (unsigned)in[i]);
    }
    TEST_ASSERT(cb.count == 0u, "count=%u != 0 apres 3 pop", (unsigned)cb.count);

    s = cb_free(&cb);
    TEST_ASSERT(s == CB_OK, "cb_free retourne %d != CB_OK", s);
    snprintf(tc->detail, sizeof(tc->detail), "FIFO {10,20,30} OK, count=0 en fin");
    tc->result = R_PASS;
}

/* ========================================================================
 * T2 – Lecture sur buffer vide
 * ======================================================================== */
void CB_seq_test_t2_empty_read(TEST_case_t *tc) {
    tc->result = R_FAIL;

    uint32_t storage[4];
    circular_buffer_t cb;
    cb_status_t s = cb_init(&cb, storage, sizeof(uint32_t), 4, CB_REJECT_NEW);
    TEST_ASSERT(s == CB_OK, "cb_init retourne %d != CB_OK", s);

    uint32_t out = 0xDEADBEEFu;
    s = cb_pop(&cb, &out);

    TEST_ASSERT(s == CB_EMPTY,          "cb_pop sur vide retourne %d != CB_EMPTY", s);
    TEST_ASSERT(out == 0xDEADBEEFu,     "cb_pop ne doit pas ecrire sur out (out=0x%08X)", (unsigned)out);
    TEST_ASSERT(cb.count == 0u,         "count=%u != 0", (unsigned)cb.count);

    s = cb_free(&cb);
    TEST_ASSERT(s == CB_OK, "cb_free retourne %d != CB_OK", s);
    snprintf(tc->detail, sizeof(tc->detail), "CB_EMPTY retourne, out intact, count=0");
    tc->result = R_PASS;
}

/* ========================================================================
 * T3 – Buffer plein avec CB_REJECT_NEW
 * ======================================================================== */
void CB_seq_test_t3_full_reject(TEST_case_t *tc) {
    tc->result = R_FAIL;

    uint32_t storage[3];
    circular_buffer_t cb;
    uint32_t val;
    cb_status_t s = cb_init(&cb, storage, sizeof(uint32_t), 3, CB_REJECT_NEW);
    TEST_ASSERT(s == CB_OK, "cb_init retourne %d != CB_OK", s);

    for (uint32_t i = 1u; i <= 3u; i++) {
        s = cb_push(&cb, &i);
        TEST_ASSERT(s == CB_OK, "push #%u retourne %d != CB_OK", (unsigned)i, s);
    }
    TEST_ASSERT(cb.count == 3u, "count=%u != 3 avant push de trop", (unsigned)cb.count);

    val = 99u;
    s = cb_push(&cb, &val);
    TEST_ASSERT(s == CB_FULL, "4eme push retourne %d != CB_FULL", s);
    TEST_ASSERT(cb.count == 3u, "count=%u != 3 apres refus", (unsigned)cb.count);

    /* Verifie que les 3 premiers elements sont intacts */
    for (uint32_t i = 1u; i <= 3u; i++) {
        uint32_t out = 0u;
        s = cb_pop(&cb, &out);
        TEST_ASSERT(s  == CB_OK, "pop #%u retourne %d != CB_OK", (unsigned)i, s);
        TEST_ASSERT(out == i,    "pop #%u=%u attendu %u", (unsigned)i, (unsigned)out, (unsigned)i);
    }

    s = cb_free(&cb);
    TEST_ASSERT(s == CB_OK, "cb_free retourne %d != CB_OK", s);
    snprintf(tc->detail, sizeof(tc->detail), "CB_FULL ok, {1,2,3} intacts, 99 rejete");
    tc->result = R_PASS;
}

/* ========================================================================
 * T4 – Buffer plein avec CB_OVERWRITE_OLDEST
 * ======================================================================== */
void CB_seq_test_t4_full_overwrite(TEST_case_t *tc) {
    tc->result = R_FAIL;

    uint32_t storage[3];
    circular_buffer_t cb;
    cb_status_t s = cb_init(&cb, storage, sizeof(uint32_t), 3, CB_OVERWRITE_OLDEST);
    TEST_ASSERT(s == CB_OK, "cb_init retourne %d != CB_OK", s);
    for (uint32_t i = 1u; i <= 3u; i++) {
        s = cb_push(&cb, &i);
        TEST_ASSERT(s == CB_OK, "push #%u retourne %d != CB_OK", (unsigned)i, s);
    }

    uint32_t val = 4u;
    s = cb_push(&cb, &val);
    TEST_ASSERT(s == CB_OVERWROTE_OLDEST, "4eme push retourne %d != CB_OVERWROTE_OLDEST", s);
    TEST_ASSERT(cb.count == 3u, "count=%u != 3 apres overwrite", (unsigned)cb.count);

    /* Apres overwrite de 1 par 4 : FIFO doit etre {2, 3, 4} */
    uint32_t expected[3] = {2u, 3u, 4u};
    for (int i = 0; i < 3; i++) {
        uint32_t out = 0u;
        s = cb_pop(&cb, &out);
        TEST_ASSERT(s   == CB_OK,          "pop[%d] retourne %d != CB_OK", i, s);
        TEST_ASSERT(out == expected[i],     "pop[%d]=%u attendu %u", i, (unsigned)out, (unsigned)expected[i]);
    }

    s = cb_free(&cb);
    TEST_ASSERT(s == CB_OK, "cb_free retourne %d != CB_OK", s);
    snprintf(tc->detail, sizeof(tc->detail), "CB_OVERWROTE_OLDEST ok, FIFO={2,3,4} apres push 4");
    tc->result = R_PASS;
}

/* ========================================================================
 * T5 – cb_reset vide logiquement le buffer
 * ======================================================================== */
void CB_seq_test_t5_reset(TEST_case_t *tc) {
    tc->result = R_FAIL;

    uint32_t storage[4];
    circular_buffer_t cb;
    cb_status_t s = cb_init(&cb, storage, sizeof(uint32_t), 4, CB_REJECT_NEW);
    TEST_ASSERT(s == CB_OK, "cb_init retourne %d != CB_OK", s);

    uint32_t val;
    val = 11u; cb_push(&cb, &val);
    val = 22u; cb_push(&cb, &val);
    TEST_ASSERT(cb.count == 2u, "count=%u != 2 avant reset", (unsigned)cb.count);

    s = cb_reset(&cb);
    TEST_ASSERT(s == CB_OK, "cb_reset retourne %d != CB_OK", s);
    TEST_ASSERT(cb.count == 0u, "count=%u != 0 apres reset", (unsigned)cb.count);

    uint32_t out = 0xDEADBEEFu;
    s = cb_pop(&cb, &out);
    TEST_ASSERT(s == CB_EMPTY, "cb_pop apres reset retourne %d != CB_EMPTY", s);

    /* Le buffer doit etre reutilisable apres reset */
    val = 55u;
    s = cb_push(&cb, &val);
    TEST_ASSERT(s == CB_OK, "push apres reset retourne %d != CB_OK", s);
    s = cb_pop(&cb, &out);
    TEST_ASSERT(s  == CB_OK && out == 55u,
              "pop apres reset : s=%d out=%u (attendu 55)", s, (unsigned)out);

    s = cb_free(&cb);
    TEST_ASSERT(s == CB_OK, "cb_free retourne %d != CB_OK", s);
    snprintf(tc->detail, sizeof(tc->detail), "Reset: count=0, CB_EMPTY, reutilisable OK");
    tc->result = R_PASS;
}

/* ========================================================================
 * T6 – cb_peek : comportement wrap permissif sur l'index absolu
 *
 * Logique testee : cb_peek(cb, idx) est equivalent a
 *   storage[ idx % capacity ]  (wrap permissif, pas de garde).
 *
 * Scenarios :
 *   A) idx in [0, cap-1]  : acces directs aux slots physiques.
 *   B) idx == capacity    : wrap -> slot 0.
 *   C) idx == capacity+1  : wrap -> slot 1.
 *   D) idx == 2*capacity  : double wrap -> slot 0 a nouveau.
 *   E) idx >> capacity (grand index) : modulo exact.
 *   F) non destructif : count inchange apres tous les peek.
 * ======================================================================== */
void CB_seq_test_t6_peek_absolute(TEST_case_t *tc) {
    tc->result = R_FAIL;

    /* capacity = 4, on remplit les 4 slots : storage[0..3] = {10,20,30,40} */
#define CAP6 4u
    uint32_t storage[CAP6];
    circular_buffer_t cb;
    cb_status_t s = cb_init(&cb, storage, sizeof(uint32_t), CAP6, CB_REJECT_NEW);
    TEST_ASSERT(s == CB_OK, "cb_init retourne %d != CB_OK", s);

    for (uint32_t i = 0u; i < CAP6; i++) {
        uint32_t v = (i + 1u) * 10u;   /* 10, 20, 30, 40 */
        cb_push(&cb, &v);
    }
    /* head == 0 (wrap), tail == 0 : slots physiques [0..3] = {10,20,30,40} */

    /* --- A) acces normal in [0, cap-1] --- */
    uint32_t expected_slot[CAP6] = {10u, 20u, 30u, 40u};
    for (size_t i = 0u; i < CAP6; i++) {
        uint32_t out = 0u;
        s = cb_peek(&cb, i, &out);
        TEST_ASSERT(s   == CB_OK,              "[A] peek[%u] retourne %d", (unsigned)i, s);
        TEST_ASSERT(out == expected_slot[i],    "[A] peek[%u]=%u attendu %u",
                  (unsigned)i, (unsigned)out, (unsigned)expected_slot[i]);
    }

    /* --- B) idx == capacity  ->  idx % 4 == 0  -> slot 0 = 10 --- */
    {
        uint32_t out = 0u;
        s = cb_peek(&cb, CAP6, &out);
        TEST_ASSERT(s   == CB_OK,  "[B] peek[cap] retourne %d", s);
        TEST_ASSERT(out == 10u,    "[B] peek[cap]=%u attendu 10", (unsigned)out);
    }

    /* --- C) idx == capacity+1  ->  idx % 4 == 1  -> slot 1 = 20 --- */
    {
        uint32_t out = 0u;
        s = cb_peek(&cb, CAP6 + 1u, &out);
        TEST_ASSERT(s   == CB_OK,  "[C] peek[cap+1] retourne %d", s);
        TEST_ASSERT(out == 20u,    "[C] peek[cap+1]=%u attendu 20", (unsigned)out);
    }

    /* --- D) idx == 2*capacity  ->  idx % 4 == 0  -> slot 0 = 10 --- */
    {
        uint32_t out = 0u;
        s = cb_peek(&cb, 2u * CAP6, &out);
        TEST_ASSERT(s   == CB_OK,  "[D] peek[2*cap] retourne %d", s);
        TEST_ASSERT(out == 10u,    "[D] peek[2*cap]=%u attendu 10", (unsigned)out);
    }

    /* --- E) grand index : idx == 17  ->  17 % 4 == 1  -> slot 1 = 20 --- */
    {
        uint32_t out = 0u;
        s = cb_peek(&cb, 17u, &out);
        TEST_ASSERT(s   == CB_OK,  "[E] peek[17] retourne %d", s);
        TEST_ASSERT(out == 20u,    "[E] peek[17]=%u attendu 20 (17%%4=1)", (unsigned)out);
    }

    /* --- F) non destructif --- */
    TEST_ASSERT(cb.count == CAP6, "[F] count=%u != %u apres tous les peek",
              (unsigned)cb.count, (unsigned)CAP6);

    s = cb_free(&cb);
    TEST_ASSERT(s == CB_OK, "cb_free retourne %d != CB_OK", s);
    snprintf(tc->detail, sizeof(tc->detail),
             "peek wrap-permissif: idx%%cap OK (B,C,D,E), non-destructif OK");
    tc->result = R_PASS;
#undef CAP6
}

/* ========================================================================
 * T7 – cb_peek_relative : couverture complete du wrap permissif
 *
 * Logique : index_physique = (origin + offset) % capacity  (modulo signe)
 *   - wrap_add gere les negatifs via  ((x % n) + n) % n.
 *
 * Etat initial :
 *   capacity=5, push {10,20,30,40,50} => storage[0..4]={10,20,30,40,50}
 *   head=0 (wrap), tail=0.
 *
 * Scenarios :
 *   A) offset positif [0..4]  depuis origin=0  -> slots directs.
 *   B) offset == +cap         depuis origin=0  -> (0+5)%5=0 -> 10.
 *   C) offset == +cap+2       depuis origin=0  -> (0+7)%5=2 -> 30.
 *   D) offset == -(cap)       depuis origin=0  -> (0-5)%5=0 -> 10.
 *   E) offset == -1           depuis origin=0  -> (0-1+5)%5=4 -> 50.
 *   F) offset == -2           depuis origin=0  -> (0-2+5)%5=3 -> 40.
 *   G) origin arbitraire=2, offset=+3 -> (2+3)%5=0 -> 10.
 *   H) origin arbitraire=2, offset=-3 -> (2-3+5)%5=4 -> 50.
 *   I) grand offset positif  -> origin=1, offset=+11 -> (1+11)%5=2 -> 30.
 *   J) grand offset negatif  -> origin=1, offset=-11 -> (1-11+15)%5=0 -> 10
 *      (wrap_add : ((1-11) % 5 + 5) % 5 = ((-10)%5+5)%5 = (0+5)%5 = 0).
 *   K) non destructif : count inchange.
 * ======================================================================== */
void CB_seq_test_t7_peek_relative(TEST_case_t *tc) {
    tc->result = R_FAIL;

#define CAP7 5u
    uint32_t storage[CAP7];
    circular_buffer_t cb;
    cb_status_t s = cb_init(&cb, storage, sizeof(uint32_t), CAP7, CB_REJECT_NEW);
    TEST_ASSERT(s == CB_OK, "cb_init retourne %d != CB_OK", s);

    /* Remplit les 5 slots : storage[i] = (i+1)*10 */
    for (uint32_t i = 0u; i < CAP7; i++) {
        uint32_t v = (i + 1u) * 10u;   /* 10, 20, 30, 40, 50 */
        cb_push(&cb, &v);
    }
    /* head=0 (wrap), tail=0. storage[0..4] = {10,20,30,40,50} */

    /* --- A) offsets positifs normaux depuis origin=0 --- */
    uint32_t slot_val[CAP7] = {10u, 20u, 30u, 40u, 50u};
    for (int off = 0; off < (int)CAP7; off++) {
        uint32_t out = 0u;
        s = cb_peek_relative(&cb, 0u, off, &out);
        TEST_ASSERT(s   == CB_OK,          "[A] peek_rel(0,+%d) retourne %d", off, s);
        TEST_ASSERT(out == slot_val[off],   "[A] peek_rel(0,+%d)=%u attendu %u",
                  off, (unsigned)out, (unsigned)slot_val[off]);
    }

    /* --- B) offset == +CAP7  ->  (0+5)%5=0  -> storage[0]=10 --- */
    {
        uint32_t out = 0u;
        s = cb_peek_relative(&cb, 0u, (int)CAP7, &out);
        TEST_ASSERT(s   == CB_OK, "[B] peek_rel(0,+cap) retourne %d", s);
        TEST_ASSERT(out == 10u,   "[B] peek_rel(0,+cap)=%u attendu 10", (unsigned)out);
    }

    /* --- C) offset == +CAP7+2  ->  (0+7)%5=2  -> storage[2]=30 --- */
    {
        uint32_t out = 0u;
        s = cb_peek_relative(&cb, 0u, (int)CAP7 + 2, &out);
        TEST_ASSERT(s   == CB_OK, "[C] peek_rel(0,+cap+2) retourne %d", s);
        TEST_ASSERT(out == 30u,   "[C] peek_rel(0,+cap+2)=%u attendu 30", (unsigned)out);
    }

    /* --- D) offset == -cap  ->  (0-5+5)%5=0  -> storage[0]=10 --- */
    {
        uint32_t out = 0u;
        s = cb_peek_relative(&cb, 0u, -(int)CAP7, &out);
        TEST_ASSERT(s   == CB_OK, "[D] peek_rel(0,-cap) retourne %d", s);
        TEST_ASSERT(out == 10u,   "[D] peek_rel(0,-cap)=%u attendu 10", (unsigned)out);
    }

    /* --- E) offset == -1  ->  (0-1+5)%5=4  -> storage[4]=50 --- */
    {
        uint32_t out = 0u;
        s = cb_peek_relative(&cb, 0u, -1, &out);
        TEST_ASSERT(s   == CB_OK, "[E] peek_rel(0,-1) retourne %d", s);
        TEST_ASSERT(out == 50u,   "[E] peek_rel(0,-1)=%u attendu 50", (unsigned)out);
    }

    /* --- F) offset == -2  ->  (0-2+5)%5=3  -> storage[3]=40 --- */
    {
        uint32_t out = 0u;
        s = cb_peek_relative(&cb, 0u, -2, &out);
        TEST_ASSERT(s   == CB_OK, "[F] peek_rel(0,-2) retourne %d", s);
        TEST_ASSERT(out == 40u,   "[F] peek_rel(0,-2)=%u attendu 40", (unsigned)out);
    }

    /* --- G) origin=2, offset=+3  ->  (2+3)%5=0  -> storage[0]=10 --- */
    {
        uint32_t out = 0u;
        s = cb_peek_relative(&cb, 2u, +3, &out);
        TEST_ASSERT(s   == CB_OK, "[G] peek_rel(2,+3) retourne %d", s);
        TEST_ASSERT(out == 10u,   "[G] peek_rel(2,+3)=%u attendu 10", (unsigned)out);
    }

    /* --- H) origin=2, offset=-3  ->  (2-3+5)%5=4  -> storage[4]=50 --- */
    {
        uint32_t out = 0u;
        s = cb_peek_relative(&cb, 2u, -3, &out);
        TEST_ASSERT(s   == CB_OK, "[H] peek_rel(2,-3) retourne %d", s);
        TEST_ASSERT(out == 50u,   "[H] peek_rel(2,-3)=%u attendu 50", (unsigned)out);
    }

    /* --- I) grand offset positif : origin=1, offset=+11
     *   (1+11)%5 = 12%5 = 2  -> storage[2]=30 --- */
    {
        uint32_t out = 0u;
        s = cb_peek_relative(&cb, 1u, +11, &out);
        TEST_ASSERT(s   == CB_OK, "[I] peek_rel(1,+11) retourne %d", s);
        TEST_ASSERT(out == 30u,   "[I] peek_rel(1,+11)=%u attendu 30 ((1+11)%%5=2)",
                  (unsigned)out);
    }

    /* --- J) grand offset negatif : origin=1, offset=-11
     *   wrap_add: ((1-11)%5 + 5)%5 = ((-10)%5+5)%5 = (0+5)%5 = 0  -> storage[0]=10
     *   (note: (-10)%5 == 0 en C, pas de reste negatif ici)         --- */
    {
        uint32_t out = 0u;
        s = cb_peek_relative(&cb, 1u, -11, &out);
        TEST_ASSERT(s   == CB_OK, "[J] peek_rel(1,-11) retourne %d", s);
        TEST_ASSERT(out == 10u,   "[J] peek_rel(1,-11)=%u attendu 10 ((1-11)%%5=0)",
                  (unsigned)out);
    }

    /* --- K) non destructif --- */
    TEST_ASSERT(cb.count == CAP7, "[K] count=%u != %u apres tous les peek_relative",
              (unsigned)cb.count, (unsigned)CAP7);

    s = cb_free(&cb);
    TEST_ASSERT(s == CB_OK, "cb_free retourne %d != CB_OK", s);
    snprintf(tc->detail, sizeof(tc->detail),
             "peek_rel wrap-permissif: A-J OK (pos/neg/grand/origin arb.), non-dest.");
    tc->result = R_PASS;
#undef CAP7
}

/* ========================================================================
 * T8 – Wrap-around (head et tail traversent la frontiere capacite)
 * ======================================================================== */
void CB_seq_test_t8_wraparound(TEST_case_t *tc) {
    tc->result = R_FAIL;

    uint32_t storage[4];
    circular_buffer_t cb;
    cb_status_t s = cb_init(&cb, storage, sizeof(uint32_t), 4, CB_REJECT_NEW);
    TEST_ASSERT(s == CB_OK, "cb_init retourne %d != CB_OK", s);

    /* Remplit le buffer : slots [0..3] = {1,2,3,4}, head wraps a 0 */
    for (uint32_t i = 1u; i <= 4u; i++) {
        cb_push(&cb, &i);
    }

    /* Pop 2 elements : tail passe de 0 a 2 */
    uint32_t out;
    cb_pop(&cb, &out);
    TEST_ASSERT(out == 1u, "pop #1 = %u attendu 1", (unsigned)out);
    cb_pop(&cb, &out);
    TEST_ASSERT(out == 2u, "pop #2 = %u attendu 2", (unsigned)out);
    TEST_ASSERT(cb.count == 2u && cb.tail == 2u && cb.head == 0u,
              "Etat apres 2 pop: count=%u tail=%u head=%u",
              (unsigned)cb.count, (unsigned)cb.tail, (unsigned)cb.head);

    /* Push 5 et 6 : head passe par le slot 0 puis 1 (wrap effectif) */
    uint32_t v5 = 5u, v6 = 6u;
    cb_push(&cb, &v5);
    cb_push(&cb, &v6);
    TEST_ASSERT(cb.count == 4u, "count=%u != 4 apres 2 push supplementaires", (unsigned)cb.count);

    /* Le FIFO doit sortir {3, 4, 5, 6} */
    uint32_t expected[4] = {3u, 4u, 5u, 6u};
    for (int i = 0; i < 4; i++) {
        s = cb_pop(&cb, &out);
        TEST_ASSERT(s   == CB_OK,          "pop[%d] retourne %d != CB_OK", i, s);
        TEST_ASSERT(out == expected[i],     "pop[%d]=%u attendu %u", i, (unsigned)out, (unsigned)expected[i]);
    }

    TEST_ASSERT(cb.count == 0u, "count=%u != 0 apres vidange complete", (unsigned)cb.count);

    s = cb_free(&cb);
    TEST_ASSERT(s == CB_OK, "cb_free retourne %d != CB_OK", s);
    snprintf(tc->detail, sizeof(tc->detail), "Wrap-around: FIFO={3,4,5,6} correct, count=0");
    tc->result = R_PASS;
}

/* ========================================================================
 * T9 – elem_size variable : type float
 * ======================================================================== */
void CB_seq_test_t9_float_elemsize(TEST_case_t *tc) {
    tc->result = R_FAIL;

    float storage[4];
    circular_buffer_t cb;
    cb_status_t s = cb_init(&cb, storage, sizeof(float), 4, CB_REJECT_NEW);
    TEST_ASSERT(s == CB_OK, "cb_init retourne %d != CB_OK", s);

    float in[3] = {1.5f, 2.5f, 3.5f};
    for (int i = 0; i < 3; i++) {
        s = cb_push(&cb, &in[i]);
        TEST_ASSERT(s == CB_OK, "push float[%d] retourne %d != CB_OK", i, s);
    }

    for (int i = 0; i < 3; i++) {
        float out = 0.0f;
        s = cb_pop(&cb, &out);
        TEST_ASSERT(s == CB_OK,                  "pop float[%d] retourne %d != CB_OK", i, s);
        TEST_ASSERT(fabsf(out - in[i]) < 1e-6f,  "pop float[%d]=%.6f attendu %.6f", i, (double)out, (double)in[i]);
    }

    TEST_ASSERT(cb.count == 0u, "count=%u != 0 apres pop complet", (unsigned)cb.count);

    s = cb_free(&cb);
    TEST_ASSERT(s == CB_OK, "cb_free retourne %d != CB_OK", s);
    snprintf(tc->detail, sizeof(tc->detail), "elem_size=float: {1.5,2.5,3.5} push/pop OK");
    tc->result = R_PASS;
}

/* ========================================================================
 * T10 – Cycle remplissage / vidange repete
 * ======================================================================== */
void CB_seq_test_t10_fill_drain_cycle(TEST_case_t *tc) {
    tc->result = R_FAIL;

#define CB_TEST_CAP 8u

    uint32_t storage[CB_TEST_CAP];
    circular_buffer_t cb;
    cb_status_t s = cb_init(&cb, storage, sizeof(uint32_t), CB_TEST_CAP, CB_REJECT_NEW);
    TEST_ASSERT(s == CB_OK, "cb_init retourne %d != CB_OK", s);

    /* --- Cycle 1 : valeurs 0..7 --- */
    for (uint32_t i = 0u; i < CB_TEST_CAP; i++) {
        s = cb_push(&cb, &i);
        TEST_ASSERT(s == CB_OK, "[C1] push[%u] retourne %d != CB_OK", (unsigned)i, s);
    }
    TEST_ASSERT(cb.count == CB_TEST_CAP, "[C1] count=%u != %u", (unsigned)cb.count, (unsigned)CB_TEST_CAP);

    for (uint32_t i = 0u; i < CB_TEST_CAP; i++) {
        uint32_t out = 0xFFu;
        s = cb_pop(&cb, &out);
        TEST_ASSERT(s   == CB_OK, "[C1] pop[%u] retourne %d != CB_OK", (unsigned)i, s);
        TEST_ASSERT(out == i,     "[C1] pop[%u]=%u attendu %u", (unsigned)i, (unsigned)out, (unsigned)i);
    }
    TEST_ASSERT(cb.count == 0u, "[C1] count=%u != 0 apres vidange", (unsigned)cb.count);

    /* --- Cycle 2 : valeurs 8..15 --- */
    for (uint32_t i = 0u; i < CB_TEST_CAP; i++) {
        uint32_t v = i + CB_TEST_CAP;
        s = cb_push(&cb, &v);
        TEST_ASSERT(s == CB_OK, "[C2] push[%u] retourne %d != CB_OK", (unsigned)v, s);
    }

    for (uint32_t i = 0u; i < CB_TEST_CAP; i++) {
        uint32_t out = 0xFFu;
        uint32_t expected = i + CB_TEST_CAP;
        s = cb_pop(&cb, &out);
        TEST_ASSERT(s   == CB_OK,         "[C2] pop[%u] retourne %d != CB_OK", (unsigned)i, s);
        TEST_ASSERT(out == expected,       "[C2] pop[%u]=%u attendu %u", (unsigned)i, (unsigned)out, (unsigned)expected);
    }
    TEST_ASSERT(cb.count == 0u, "[C2] count=%u != 0 apres vidange", (unsigned)cb.count);

#undef CB_TEST_CAP

    s = cb_free(&cb);
    TEST_ASSERT(s == CB_OK, "cb_free retourne %d != CB_OK", s);
    snprintf(tc->detail, sizeof(tc->detail), "2 cycles fill/drain cap=8 OK, count=0 final");
    tc->result = R_PASS;
}

/* ========================================================================
 * Outils des cas limites et de stress (T11-T21)
 * ======================================================================== */

#define CBT_GUARD       16u     /* octets temoins de chaque cote */
#define CBT_GUARD_BYTE  0xA5u

static void cbt_guard_fill(uint8_t *area, size_t inner) {
    memset(area, CBT_GUARD_BYTE, CBT_GUARD);
    memset(area + CBT_GUARD + inner, CBT_GUARD_BYTE, CBT_GUARD);
}

static bool cbt_guard_ok(const uint8_t *area, size_t inner) {
    for (size_t i = 0; i < CBT_GUARD; i++) {
        if (area[i] != CBT_GUARD_BYTE || area[CBT_GUARD + inner + i] != CBT_GUARD_BYTE) {
            return false;
        }
    }
    return true;
}

/* Octet i de l'element numero n : chaque element a un contenu propre. */
static uint8_t cbt_pattern(uint32_t n, size_t i) {
    const uint32_t x = n * 2654435761u + (uint32_t)i * 40503u + 1u;
    return (uint8_t)(x ^ (x >> 13) ^ (x >> 24));
}

static void cbt_fill(uint8_t *dst, uint32_t n, size_t size) {
    for (size_t i = 0; i < size; i++) {
        dst[i] = cbt_pattern(n, i);
    }
}

static bool cbt_check(const uint8_t *src, uint32_t n, size_t size) {
    for (size_t i = 0; i < size; i++) {
        if (src[i] != cbt_pattern(n, i)) {
            return false;
        }
    }
    return true;
}

static bool cbt_all(const uint8_t *src, uint8_t byte, size_t size) {
    for (size_t i = 0; i < size; i++) {
        if (src[i] != byte) {
            return false;
        }
    }
    return true;
}

/* Generateur congruentiel, graine fixe : tirages reproductibles. */
static uint32_t cbt_rand(uint32_t *state) {
    *state = *state * 1664525u + 1013904223u;
    return *state >> 8;
}

/* Reference de (origin + offset) mod cap, en 64 bits signes. */
static size_t cbt_ref_wrap(size_t origin, int offset, size_t cap) {
    const int64_t c = (int64_t)cap;
    const int64_t o = (int64_t)(origin % cap);
    const int64_t r = (((int64_t)offset % c) + c) % c;
    return (size_t)((o + r) % c);
}

/* Attente active : T20/T21 tournent aussi bien en sequentiel qu'en RTOS. */
static void cbt_busy_wait_ms(uint32_t ms) {
    const uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < ms) {
    }
}

/* ========================================================================
 * T11 – Capacite 1
 * ======================================================================== */
void CB_seq_test_t11_capacity_one(TEST_case_t *tc) {
    tc->result = R_FAIL;

    uint32_t storage[1];
    circular_buffer_t cb;
    uint32_t v, out;
    cb_status_t s = cb_init(&cb, storage, sizeof(uint32_t), 1u, CB_REJECT_NEW);
    TEST_ASSERT(s == CB_OK, "[REJECT] cb_init retourne %d", s);

    for (uint32_t k = 1u; k <= 5u; k++) {
        v = k;
        s = cb_push(&cb, &v);
        TEST_ASSERT(s == CB_OK, "[REJECT] cycle %u : push retourne %d", (unsigned)k, s);
        v = 100u + k;
        s = cb_push(&cb, &v);
        TEST_ASSERT(s == CB_FULL && cb_count(&cb) == 1u, "[REJECT] cycle %u : 2e push s=%d count=%u",
                    (unsigned)k, s, (unsigned)cb_count(&cb));
        out = 0u;
        s = cb_pop(&cb, &out);
        TEST_ASSERT(s == CB_OK && out == k, "[REJECT] cycle %u : pop s=%d out=%u", (unsigned)k, s, (unsigned)out);
        s = cb_pop(&cb, &out);
        TEST_ASSERT(s == CB_EMPTY, "[REJECT] cycle %u : pop sur vide retourne %d", (unsigned)k, s);
        TEST_ASSERT(cb.head == 0u && cb.tail == 0u, "[REJECT] cycle %u : head=%u tail=%u",
                    (unsigned)k, (unsigned)cb.head, (unsigned)cb.tail);
    }

    s = cb_init(&cb, storage, sizeof(uint32_t), 1u, CB_OVERWRITE_OLDEST);
    TEST_ASSERT(s == CB_OK, "[OVERWRITE] cb_init retourne %d", s);
    v = 1u;
    s = cb_push(&cb, &v);
    TEST_ASSERT(s == CB_OK, "[OVERWRITE] 1er push retourne %d", s);
    for (uint32_t k = 2u; k <= 6u; k++) {
        v = k;
        s = cb_push(&cb, &v);
        TEST_ASSERT(s == CB_OVERWROTE_OLDEST && cb_count(&cb) == 1u, "[OVERWRITE] push %u : s=%d count=%u",
                    (unsigned)k, s, (unsigned)cb_count(&cb));
        out = 0u;
        s = cb_peek(&cb, 0u, &out);
        TEST_ASSERT(s == CB_OK && out == k, "[OVERWRITE] peek apres push %u : s=%d out=%u", (unsigned)k, s, (unsigned)out);
    }
    out = 0u;
    s = cb_pop(&cb, &out);
    TEST_ASSERT(s == CB_OK && out == 6u, "[OVERWRITE] pop : s=%d out=%u attendu 6", s, (unsigned)out);
    s = cb_pop(&cb, &out);
    TEST_ASSERT(s == CB_EMPTY, "[OVERWRITE] pop sur vide retourne %d", s);

    snprintf(tc->detail, sizeof(tc->detail), "cap=1 : REJECT 5 cycles, OVERWRITE garde la derniere (6)");
    tc->result = R_PASS;
}

/* ========================================================================
 * T12 – Ecrasements successifs
 * ======================================================================== */
void CB_seq_test_t12_multi_overwrite(TEST_case_t *tc) {
    tc->result = R_FAIL;

    uint32_t storage[3];
    circular_buffer_t cb;
    cb_status_t s = cb_init(&cb, storage, sizeof(uint32_t), 3u, CB_OVERWRITE_OLDEST);
    TEST_ASSERT(s == CB_OK, "cb_init retourne %d", s);

    for (uint32_t v = 1u; v <= 10u; v++) {
        s = cb_push(&cb, &v);
        const cb_status_t want = (v <= 3u) ? CB_OK : CB_OVERWROTE_OLDEST;
        TEST_ASSERT(s == want, "push %u retourne %d attendu %d", (unsigned)v, s, want);
        TEST_ASSERT(cb_count(&cb) == ((v < 3u) ? v : 3u), "push %u : count=%u", (unsigned)v, (unsigned)cb_count(&cb));
    }
    for (uint32_t want = 8u; want <= 10u; want++) {
        uint32_t out = 0u;
        s = cb_pop(&cb, &out);
        TEST_ASSERT(s == CB_OK && out == want, "pop : s=%d out=%u attendu %u", s, (unsigned)out, (unsigned)want);
    }
    s = cb_pop(&cb, &(uint32_t){0u});
    TEST_ASSERT(s == CB_EMPTY, "4e pop retourne %d != CB_EMPTY", s);

    /* 1000 pushes : le tail fait des centaines de tours par le chemin d'ecrasement. */
    uint32_t overwrote = 0u;
    for (uint32_t v = 1u; v <= 1000u; v++) {
        s = cb_push(&cb, &v);
        if (s == CB_OVERWROTE_OLDEST) {
            overwrote++;
        } else {
            TEST_ASSERT(s == CB_OK, "[1000] push %u retourne %d", (unsigned)v, s);
        }
    }
    TEST_ASSERT(overwrote == 997u, "[1000] %u ecrasements, attendu 997", (unsigned)overwrote);
    for (uint32_t want = 998u; want <= 1000u; want++) {
        uint32_t out = 0u;
        s = cb_pop(&cb, &out);
        TEST_ASSERT(s == CB_OK && out == want, "[1000] pop : s=%d out=%u attendu %u", s, (unsigned)out, (unsigned)want);
    }
    TEST_ASSERT(cb_count(&cb) == 0u, "[1000] count=%u en fin", (unsigned)cb_count(&cb));

    snprintf(tc->detail, sizeof(tc->detail), "1..10 : 7 ecrasements, reste 8,9,10 ; 1000 pushes : reste 998..1000");
    tc->result = R_PASS;
}

/* ========================================================================
 * T13 – Sortie de l'etat plein (REJECT_NEW)
 * ======================================================================== */
void CB_seq_test_t13_reject_recovery(TEST_case_t *tc) {
    tc->result = R_FAIL;

    uint32_t storage[4];
    circular_buffer_t cb;
    cb_status_t s = cb_init(&cb, storage, sizeof(uint32_t), 4u, CB_REJECT_NEW);
    TEST_ASSERT(s == CB_OK, "cb_init retourne %d", s);

    uint32_t next_in = 1u, next_out = 1u;
    for (; next_in <= 4u; next_in++) {
        s = cb_push(&cb, &next_in);
        TEST_ASSERT(s == CB_OK, "remplissage : push %u retourne %d", (unsigned)next_in, s);
    }
    for (uint32_t k = 0u; k < 50u; k++) {
        uint32_t v = 0xFFFFu;
        s = cb_push(&cb, &v);
        TEST_ASSERT(s == CB_FULL, "tour %u : push sur plein retourne %d", (unsigned)k, s);
        uint32_t out = 0u;
        s = cb_pop(&cb, &out);
        TEST_ASSERT(s == CB_OK && out == next_out, "tour %u : pop s=%d out=%u attendu %u",
                    (unsigned)k, s, (unsigned)out, (unsigned)next_out);
        next_out++;
        s = cb_push(&cb, &next_in);
        TEST_ASSERT(s == CB_OK, "tour %u : push apres pop retourne %d", (unsigned)k, s);
        next_in++;
    }
    while (next_out < next_in) {
        uint32_t out = 0u;
        s = cb_pop(&cb, &out);
        TEST_ASSERT(s == CB_OK && out == next_out, "vidange : s=%d out=%u attendu %u", s, (unsigned)out, (unsigned)next_out);
        next_out++;
    }
    TEST_ASSERT(cb_count(&cb) == 0u, "count=%u apres vidange", (unsigned)cb_count(&cb));

    snprintf(tc->detail, sizeof(tc->detail), "50 tours plein/pop/push/refus, FIFO 1..54 conserve");
    tc->result = R_PASS;
}

/* ========================================================================
 * T14 – Tailles d'element 1, 3, 13, 128 octets, avec canaris
 * ======================================================================== */
void CB_seq_test_t14_elem_sizes_guards(TEST_case_t *tc) {
    tc->result = R_FAIL;

#define T14_CAP   5u
#define T14_MAXE  128u
    static const size_t sizes[] = { 1u, 3u, 13u, 128u };
    static uint8_t area[CBT_GUARD + T14_CAP * T14_MAXE + CBT_GUARD];
    static uint8_t outb[CBT_GUARD + T14_MAXE + CBT_GUARD];
    static uint8_t elem[T14_MAXE];

    for (size_t k = 0; k < sizeof(sizes) / sizeof(sizes[0]); k++) {
        const size_t es    = sizes[k];
        const size_t inner = T14_CAP * es;
        uint8_t *const st  = area + CBT_GUARD;
        uint8_t *const out = outb + CBT_GUARD;
        memset(area, 0, sizeof(area));
        cbt_guard_fill(area, inner);
        cbt_guard_fill(outb, es);

        circular_buffer_t cb;
        cb_status_t s = cb_init(&cb, st, es, T14_CAP, CB_OVERWRITE_OLDEST);
        TEST_ASSERT(s == CB_OK, "[%u o] cb_init retourne %d", (unsigned)es, s);

        uint32_t oldest = 0u, count = 0u;   /* modele : numero du plus ancien, nombre present */
        for (uint32_t n = 0u; n < 23u; n++) {
            cbt_fill(elem, n, es);
            const cb_status_t want = (count == T14_CAP) ? CB_OVERWROTE_OLDEST : CB_OK;
            s = cb_push(&cb, elem);
            TEST_ASSERT(s == want, "[%u o] push %u retourne %d attendu %d", (unsigned)es, (unsigned)n, s, want);
            if (want == CB_OVERWROTE_OLDEST) {
                oldest++;
            } else {
                count++;
            }
            TEST_ASSERT(cbt_guard_ok(area, inner), "[%u o] temoin du stockage ecrase par le push %u", (unsigned)es, (unsigned)n);

            if ((n % 3u) == 2u) {
                memset(out, 0, es);
                s = cb_pop(&cb, out);
                TEST_ASSERT(s == CB_OK && cbt_check(out, oldest, es), "[%u o] pop apres push %u : s=%d, contenu != element %u",
                            (unsigned)es, (unsigned)n, s, (unsigned)oldest);
                TEST_ASSERT(cbt_guard_ok(outb, es), "[%u o] temoin de sortie ecrase par pop", (unsigned)es);
                oldest++;
                count--;
            }
        }
        TEST_ASSERT(cb_count(&cb) == count, "[%u o] count=%u modele=%u", (unsigned)es, (unsigned)cb_count(&cb), (unsigned)count);
        for (uint32_t i = 0u; i < count; i++) {
            memset(out, 0, es);
            s = cb_peek_relative(&cb, cb.tail, (int)i, out);
            TEST_ASSERT(s == CB_OK && cbt_check(out, oldest + i, es), "[%u o] peek_relative(tail,+%u) != element %u",
                        (unsigned)es, (unsigned)i, (unsigned)(oldest + i));
            TEST_ASSERT(cbt_guard_ok(outb, es), "[%u o] temoin de sortie ecrase par peek", (unsigned)es);
        }
        TEST_ASSERT(cbt_guard_ok(area, inner), "[%u o] temoin du stockage ecrase", (unsigned)es);
    }
#undef T14_CAP
#undef T14_MAXE

    snprintf(tc->detail, sizeof(tc->detail), "1/3/13/128 o : 23 push + pops, contenus exacts, temoins intacts");
    tc->result = R_PASS;
}

/* ========================================================================
 * T15 – Acces pointeur
 * ======================================================================== */
void CB_seq_test_t15_ptr_accessors(TEST_case_t *tc) {
    tc->result = R_FAIL;

    uint32_t storage[4];
    circular_buffer_t cb;
    cb_status_t s = cb_init(&cb, storage, sizeof(uint32_t), 4u, CB_REJECT_NEW);
    TEST_ASSERT(s == CB_OK, "cb_init retourne %d", s);
    for (uint32_t i = 0u; i < 4u; i++) {
        const uint32_t v = (i + 1u) * 10u;
        s = cb_push(&cb, &v);
        TEST_ASSERT(s == CB_OK, "push %u retourne %d", (unsigned)v, s);
    }

    for (size_t idx = 0u; idx < 12u; idx++) {
        const uint32_t *p = (const uint32_t *)cb_peek_ptr(&cb, idx);
        TEST_ASSERT(p == &storage[idx % 4u], "cb_peek_ptr(%u) ne pointe pas sur le slot %u",
                    (unsigned)idx, (unsigned)(idx % 4u));
    }
    for (size_t origin = 0u; origin < 10u; origin++) {
        for (int off = -9; off <= 9; off++) {
            const size_t slot = cbt_ref_wrap(origin, off, 4u);
            const uint32_t *p = (const uint32_t *)cb_peek_relative_ptr(&cb, origin, off);
            TEST_ASSERT(p == &storage[slot], "peek_relative_ptr(%u,%d) ne pointe pas sur le slot %u",
                        (unsigned)origin, off, (unsigned)slot);
            uint32_t out = 0u;
            s = cb_peek_relative(&cb, origin, off, &out);
            TEST_ASSERT(s == CB_OK && out == *p, "peek_relative(%u,%d) : s=%d copie %u != *ptr %u",
                        (unsigned)origin, off, s, (unsigned)out, (unsigned)*p);
        }
    }

    TEST_ASSERT(cb_peek_ptr(NULL, 0u) == NULL, "cb_peek_ptr(NULL,0) != NULL");
    TEST_ASSERT(cb_peek_relative_ptr(NULL, 0u, 0) == NULL, "cb_peek_relative_ptr(NULL,0,0) != NULL");
    circular_buffer_t zero = { 0 };
    TEST_ASSERT(cb_peek_ptr(&zero, 0u) == NULL && cb_peek_relative_ptr(&zero, 0u, 1) == NULL,
                "buffer jamais initialise : pointeur non NULL");
    TEST_ASSERT(cb_count(&cb) == 4u, "les acces pointeur ont modifie count (%u)", (unsigned)cb_count(&cb));

    snprintf(tc->detail, sizeof(tc->detail), "peek_ptr x12, peek_relative_ptr 10x19 = copies ; NULL si invalide");
    tc->result = R_PASS;
}

/* ========================================================================
 * T16 – cb_wrap aux extremes
 * ======================================================================== */
void CB_seq_test_t16_wrap_extremes(TEST_case_t *tc) {
    tc->result = R_FAIL;

    static uint8_t storage[1000];
    static const size_t caps[] = { 1u, 2u, 3u, 5u, 7u, 8u, 1000u };
    for (size_t i = 0; i < sizeof(storage); i++) {
        storage[i] = (uint8_t)(i * 7u + 3u);
    }

    uint32_t checks = 0u;
    for (size_t k = 0; k < sizeof(caps) / sizeof(caps[0]); k++) {
        const size_t cap = caps[k];
        circular_buffer_t cb;
        cb_status_t s = cb_init(&cb, storage, 1u, cap, CB_OVERWRITE_OLDEST);
        TEST_ASSERT(s == CB_OK, "cap=%u : cb_init retourne %d", (unsigned)cap, s);

        const size_t origins[] = { 0u, 1u, cap - 1u, cap, cap + 1u, SIZE_MAX, SIZE_MAX - 1u, SIZE_MAX / 2u };
        const int offsets[]    = { 0, 1, -1, (int)cap, -(int)cap, INT_MAX, INT_MAX - 1, INT_MIN, INT_MIN + 1,
                                   12345, -12345 };
        for (size_t a = 0; a < sizeof(origins) / sizeof(origins[0]); a++) {
            for (size_t b = 0; b < sizeof(offsets) / sizeof(offsets[0]); b++) {
                const size_t want = cbt_ref_wrap(origins[a], offsets[b], cap);
                const uint8_t *p = (const uint8_t *)cb_peek_relative_ptr(&cb, origins[a], offsets[b]);
                TEST_ASSERT(p == &storage[want], "cap=%u origin=%lu offset=%d : slot %ld attendu %u",
                            (unsigned)cap, (unsigned long)origins[a], offsets[b],
                            (p != NULL) ? (long)(p - storage) : -1L, (unsigned)want);
                uint8_t out = 0u;
                s = cb_peek_relative(&cb, origins[a], offsets[b], &out);
                TEST_ASSERT(s == CB_OK && out == storage[want], "cap=%u origin=%lu offset=%d : copie fausse",
                            (unsigned)cap, (unsigned long)origins[a], offsets[b]);
                checks++;
            }
        }
    }

    snprintf(tc->detail, sizeof(tc->detail), "%lu combinaisons (INT_MIN, INT_MAX, SIZE_MAX...) = reference 64 bits",
             (unsigned long)checks);
    tc->result = R_PASS;
}

/* ========================================================================
 * T17 – Arguments invalides restants
 * ======================================================================== */
void CB_seq_test_t17_bad_args(TEST_case_t *tc) {
    tc->result = R_FAIL;

    uint32_t storage[4];
    circular_buffer_t cb;
    uint32_t out = 0xDEADBEEFu, v = 1u;
    cb_status_t s = cb_init(&cb, storage, sizeof(uint32_t), 4u, CB_REJECT_NEW);
    TEST_ASSERT(s == CB_OK, "cb_init retourne %d", s);
    s = cb_push(&cb, &v);
    TEST_ASSERT(s == CB_OK, "push retourne %d", s);

    s = cb_pop(&cb, NULL);
    TEST_ASSERT(s == CB_BAD_ARG && cb_count(&cb) == 1u, "pop(cb,NULL) : s=%d count=%u", s, (unsigned)cb_count(&cb));
    s = cb_peek(NULL, 0u, &out);
    TEST_ASSERT(s == CB_BAD_ARG, "peek(NULL,...) retourne %d", s);
    s = cb_peek(&cb, 0u, NULL);
    TEST_ASSERT(s == CB_BAD_ARG, "peek(cb,0,NULL) retourne %d", s);
    s = cb_peek_relative(NULL, 0u, 0, &out);
    TEST_ASSERT(s == CB_BAD_ARG, "peek_relative(NULL,...) retourne %d", s);
    s = cb_peek_relative(&cb, 0u, 0, NULL);
    TEST_ASSERT(s == CB_BAD_ARG, "peek_relative(cb,...,NULL) retourne %d", s);
    s = cb_reset(NULL);
    TEST_ASSERT(s == CB_BAD_ARG, "reset(NULL) retourne %d", s);
    s = cb_free(NULL);
    TEST_ASSERT(s == CB_BAD_ARG, "free(NULL) retourne %d", s);
    TEST_ASSERT(cb_count(NULL) == 0u, "count(NULL) != 0");

    circular_buffer_t other;
    s = cb_init(&other, storage, sizeof(uint32_t), 4u, (cb_overflow_policy_t)7);
    TEST_ASSERT(s == CB_BAD_ARG, "politique 7 acceptee (%d)", s);
    s = cb_init(&other, storage, (SIZE_MAX / 2u) + 1u, 2u, CB_REJECT_NEW);
    TEST_ASSERT(s == CB_BAD_ARG, "elem_size * capacity > SIZE_MAX accepte (%d)", s);
    s = cb_init(&other, storage, 2u, (SIZE_MAX / 2u) + 1u, CB_REJECT_NEW);
    TEST_ASSERT(s == CB_BAD_ARG, "capacity * elem_size > SIZE_MAX accepte (%d)", s);

    circular_buffer_t zero = { 0 };
    s = cb_push(&zero, &v);
    TEST_ASSERT(s == CB_BAD_ARG, "push sur buffer jamais initialise retourne %d", s);
    s = cb_pop(&zero, &out);
    TEST_ASSERT(s == CB_BAD_ARG, "pop sur buffer jamais initialise retourne %d", s);
    s = cb_peek(&zero, 0u, &out);
    TEST_ASSERT(s == CB_BAD_ARG, "peek sur buffer jamais initialise retourne %d", s);
    s = cb_peek_relative(&zero, 0u, 0, &out);
    TEST_ASSERT(s == CB_BAD_ARG, "peek_relative sur buffer jamais initialise retourne %d", s);
    TEST_ASSERT(cb_count(&zero) == 0u, "count d'un buffer a zero != 0");
    TEST_ASSERT(out == 0xDEADBEEFu, "une sortie a ete ecrite malgre CB_BAD_ARG (0x%08X)", (unsigned)out);

    snprintf(tc->detail, sizeof(tc->detail), "NULL, buffer a zero, politique 7, debordement SIZE_MAX : refuses");
    tc->result = R_PASS;
}

/* ========================================================================
 * T18 – Push d'une donnee deja en place
 * ======================================================================== */
void CB_seq_test_t18_push_in_place(TEST_case_t *tc) {
    tc->result = R_FAIL;

    uint32_t storage[3];
    circular_buffer_t cb;
    cb_status_t s = cb_init(&cb, storage, sizeof(uint32_t), 3u, CB_REJECT_NEW);
    TEST_ASSERT(s == CB_OK, "cb_init retourne %d", s);

    for (uint32_t k = 1u; k <= 7u; k++) {
        uint32_t *slot = (uint32_t *)cb_peek_ptr(&cb, cb.head);
        TEST_ASSERT(slot != NULL, "tour %u : cb_peek_ptr(head) NULL", (unsigned)k);
        *slot = 700u + k;
        s = cb_push(&cb, slot);
        TEST_ASSERT(s == CB_OK, "tour %u : push en place retourne %d", (unsigned)k, s);
        uint32_t out = 0u;
        s = cb_pop(&cb, &out);
        TEST_ASSERT(s == CB_OK && out == 700u + k, "tour %u : s=%d relu %u", (unsigned)k, s, (unsigned)out);
    }

    snprintf(tc->detail, sizeof(tc->detail), "7 tours (head fait 2 tours) : push depuis le slot head, relu");
    tc->result = R_PASS;
}

/* ========================================================================
 * T19 – Test aleatoire contre un modele de reference
 *
 * Modele independant de head / tail : le k-ieme push accepte depuis
 * init / reset va au slot k % capacite ; le plus ancien present est le
 * `first`-ieme ; le contenu d'un slot est celui du dernier element qui y a
 * ete ecrit (un pop ne l'efface pas, un reset non plus).
 * ======================================================================== */
void CB_seq_test_t19_random_model(TEST_case_t *tc) {
    tc->result = R_FAIL;

#define T19_MAXCAP  16u
#define T19_MAXE    12u
#define T19_OPS     10000u
#define T19_NEVER   0xFFFFFFFFu
    static const size_t caps[]  = { 1u, 2u, 3u, 7u, 8u, 16u };
    static const size_t sizes[] = { 1u, 4u, 12u };
    static const cb_overflow_policy_t pols[] = { CB_OVERWRITE_OLDEST, CB_REJECT_NEW };
    static uint8_t area[CBT_GUARD + T19_MAXCAP * T19_MAXE + CBT_GUARD];
    static uint8_t outb[CBT_GUARD + T19_MAXE + CBT_GUARD];
    uint8_t elem[T19_MAXE];
    uint32_t slot_seq[T19_MAXCAP];
    uint32_t rng = 0x12345678u;
    uint32_t ops = 0u;

    for (size_t ic = 0; ic < sizeof(caps) / sizeof(caps[0]); ic++) {
    for (size_t is = 0; is < sizeof(sizes) / sizeof(sizes[0]); is++) {
    for (size_t ip = 0; ip < sizeof(pols) / sizeof(pols[0]); ip++) {
        const size_t cap = caps[ic], es = sizes[is], inner = cap * es;
        const cb_overflow_policy_t pol = pols[ip];
        uint8_t *const st  = area + CBT_GUARD;
        uint8_t *const out = outb + CBT_GUARD;
        memset(area, 0xEE, sizeof(area));
        cbt_guard_fill(area, inner);
        cbt_guard_fill(outb, es);

        circular_buffer_t cb;
        cb_status_t s = cb_init(&cb, st, es, cap, pol);
        TEST_ASSERT(s == CB_OK, "cap=%u e=%u : cb_init retourne %d", (unsigned)cap, (unsigned)es, s);

        for (size_t i = 0; i < T19_MAXCAP; i++) {
            slot_seq[i] = T19_NEVER;
        }
        uint32_t pushed = 0u;   /* pushes acceptes depuis init / reset */
        uint32_t first  = 0u;   /* rang du plus ancien present          */
        uint32_t seq    = 0u;   /* numero du prochain element           */

        for (uint32_t k = 0u; k < T19_OPS; k++, ops++) {
            const uint32_t r = cbt_rand(&rng) % 1000u;
            const uint32_t count = pushed - first;

            if (r < 500u) {                                 /* push */
                cbt_fill(elem, seq, es);
                cb_status_t want = CB_OK;
                if (count == cap && pol == CB_REJECT_NEW) {
                    want = CB_FULL;
                } else {
                    if (count == cap) {
                        want = CB_OVERWROTE_OLDEST;
                        first++;
                    }
                    slot_seq[pushed % cap] = seq;
                    pushed++;
                }
                s = cb_push(&cb, elem);
                TEST_ASSERT(s == want, "cap=%u e=%u pol=%d op %u : push %d attendu %d",
                            (unsigned)cap, (unsigned)es, pol, (unsigned)k, s, want);
                seq++;
            } else if (r < 850u) {                          /* pop */
                memset(out, 0x11, es);
                s = cb_pop(&cb, out);
                if (count == 0u) {
                    TEST_ASSERT(s == CB_EMPTY && cbt_all(out, 0x11u, es), "cap=%u e=%u pol=%d op %u : pop vide s=%d",
                                (unsigned)cap, (unsigned)es, pol, (unsigned)k, s);
                } else {
                    TEST_ASSERT(s == CB_OK && cbt_check(out, slot_seq[first % cap], es),
                                "cap=%u e=%u pol=%d op %u : pop s=%d, contenu faux",
                                (unsigned)cap, (unsigned)es, pol, (unsigned)k, s);
                    first++;
                }
            } else if (r < 995u) {                          /* peek absolu ou relatif */
                size_t slot;
                memset(out, 0x11, es);
                if (r < 925u) {
                    const size_t idx = (size_t)(cbt_rand(&rng) % (3u * cap + 1u));
                    slot = idx % cap;
                    s = cb_peek(&cb, idx, out);
                } else {
                    const size_t origin = (size_t)cbt_rand(&rng);
                    const int offset = (int)(cbt_rand(&rng) % 2001u) - 1000;
                    slot = cbt_ref_wrap(origin, offset, cap);
                    s = cb_peek_relative(&cb, origin, offset, out);
                }
                const bool content_ok = (slot_seq[slot] == T19_NEVER) ? cbt_all(out, 0xEEu, es)
                                                                       : cbt_check(out, slot_seq[slot], es);
                TEST_ASSERT(s == CB_OK && content_ok, "cap=%u e=%u pol=%d op %u : peek slot %u s=%d, contenu faux",
                            (unsigned)cap, (unsigned)es, pol, (unsigned)k, (unsigned)slot, s);
            } else {                                        /* reset */
                s = cb_reset(&cb);
                TEST_ASSERT(s == CB_OK, "cap=%u e=%u pol=%d op %u : reset retourne %d",
                            (unsigned)cap, (unsigned)es, pol, (unsigned)k, s);
                pushed = 0u;
                first  = 0u;
            }

            TEST_ASSERT(cb_count(&cb) == pushed - first, "cap=%u e=%u pol=%d op %u : count=%u modele=%u",
                        (unsigned)cap, (unsigned)es, pol, (unsigned)k, (unsigned)cb_count(&cb), (unsigned)(pushed - first));
            TEST_ASSERT(cbt_guard_ok(area, inner) && cbt_guard_ok(outb, es), "cap=%u e=%u pol=%d op %u : temoin ecrase",
                        (unsigned)cap, (unsigned)es, pol, (unsigned)k);
        }
    }
    }
    }
#undef T19_MAXCAP
#undef T19_MAXE
#undef T19_OPS
#undef T19_NEVER

    snprintf(tc->detail, sizeof(tc->detail), "%lu operations aleatoires, 36 configurations : conformes au modele",
             (unsigned long)ops);
    tc->result = R_PASS;
}

/* ========================================================================
 * T20 / T21 – Concurrence avec une interruption (TIM5, test_irq.h)
 *
 * Tout ce que touche l'ISR est statique. Le timer est arrete avant toute
 * assertion : un TEST_ASSERT en echec ne laisse jamais l'ISR tourner.
 * ======================================================================== */

#define CBT_ISR_CAP   32u

typedef struct {
    uint32_t w[16];                 /* 64 o, tous les mots = numero de l'element */
} cbt_blob_t;

static circular_buffer_t cbt_isr_cb;
static uint32_t          cbt_isr_store[CBT_ISR_CAP];
static cbt_blob_t        cbt_blob_store[4];
static cbt_blob_t        cbt_isr_blob;      /* hors pile : la pile MSP est petite */
static volatile uint32_t cbt_isr_n;
static volatile uint32_t cbt_isr_rejected;
static volatile uint32_t cbt_isr_overwrote;

/* T20 : 0, 1, 2... dans un buffer REJECT_NEW, refus comptes. */
static void cbt_hook_push_u32(void) {
    const uint32_t x = cbt_isr_n;
    if (cb_push(&cbt_isr_cb, &x) == CB_FULL) {
        cbt_isr_rejected++;
    }
    cbt_isr_n = x + 1u;
}

/* T21 : un element de 64 o par interruption, ecrasements comptes. */
static void cbt_hook_push_blob(void) {
    const uint32_t n = cbt_isr_n;
    for (uint32_t k = 0u; k < 16u; k++) {
        cbt_isr_blob.w[k] = n;
    }
    if (cb_push(&cbt_isr_cb, &cbt_isr_blob) == CB_OVERWROTE_OLDEST) {
        cbt_isr_overwrote++;
    }
    cbt_isr_n = n + 1u;
}

static bool cbt_blob_ok(const cbt_blob_t *b) {
    for (uint32_t k = 1u; k < 16u; k++) {
        if (b->w[k] != b->w[0]) {
            return false;
        }
    }
    return true;
}

/* ========================================================================
 * T20 – cb_push depuis une interruption
 * ======================================================================== */
void CB_seq_test_t20_isr_push(TEST_case_t *tc) {
    tc->result = R_FAIL;

    cbt_isr_n        = 0u;
    cbt_isr_rejected = 0u;
    TEST_ASSERT(cb_init(&cbt_isr_cb, cbt_isr_store, sizeof(uint32_t), CBT_ISR_CAP, CB_REJECT_NEW) == CB_OK, "cb_init");

    uint32_t popped = 0u, next = 0u, gaps = 0u, bad = 0u, incoherent = 0u, v;

    TEST_irq_start(20000u, cbt_hook_push_u32);
    const uint32_t t_end = HAL_GetTick() + 300u;
    while ((int32_t)(HAL_GetTick() - t_end) < 0) {
        while (cb_pop(&cbt_isr_cb, &v) == CB_OK) {
            if (v < next) bad++;                    /* dupliquee ou desordonnee */
            else          gaps += v - next;         /* refusee par l'ISR        */
            next = v + 1u;
            popped++;
        }
        /* count doit toujours correspondre a head / tail. */
        cb_critical_t c = cb_critical_enter();
        const size_t calc = (cbt_isr_cb.head + cbt_isr_cb.capacity - cbt_isr_cb.tail) % cbt_isr_cb.capacity;
        const bool   ok   = (cbt_isr_cb.count == calc) || (cbt_isr_cb.count == cbt_isr_cb.capacity && calc == 0u);
        cb_critical_exit(c);
        if (!ok) incoherent++;

        /* Laisser le buffer se remplir de temps en temps : refus cote ISR. */
        if ((popped & 0x3FFu) == 0u) {
            cbt_busy_wait_ms(3u);
        }
    }
    TEST_irq_stop();

    while (cb_pop(&cbt_isr_cb, &v) == CB_OK) {
        if (v < next) bad++;
        else          gaps += v - next;
        next = v + 1u;
        popped++;
    }

    const uint32_t pushed   = cbt_isr_n;
    const uint32_t rejected = cbt_isr_rejected;
    TEST_ASSERT(pushed > 4000u, "ISR TIM5 : %lu appels seulement", (unsigned long)pushed);
    TEST_ASSERT(incoherent == 0u, "%lu incoherence(s) count / head / tail", (unsigned long)incoherent);
    TEST_ASSERT(bad == 0u, "%lu valeur(s) dupliquee(s) ou desordonnee(s)", (unsigned long)bad);
    TEST_ASSERT(popped + rejected == pushed, "depiles %lu + refus %lu != pousses %lu",
                (unsigned long)popped, (unsigned long)rejected, (unsigned long)pushed);
    TEST_ASSERT(gaps + (pushed - next) == rejected, "trous %lu + fin %lu != refus %lu",
                (unsigned long)gaps, (unsigned long)(pushed - next), (unsigned long)rejected);
    TEST_ASSERT(rejected > 0u, "Aucun refus : buffer jamais plein, test non concluant");

    snprintf(tc->detail, sizeof(tc->detail), "%lu poussees en ISR : %lu depilees, %lu refusees, count coherent",
             (unsigned long)pushed, (unsigned long)popped, (unsigned long)rejected);
    tc->result = R_PASS;
}

/* ========================================================================
 * T21 – Copies jamais dechirees face a une ISR
 * ======================================================================== */
void CB_seq_test_t21_isr_no_torn_copy(TEST_case_t *tc) {
    tc->result = R_FAIL;

    cbt_isr_n         = 0u;
    cbt_isr_overwrote = 0u;
    memset(cbt_blob_store, 0, sizeof(cbt_blob_store));
    TEST_ASSERT(cb_init(&cbt_isr_cb, cbt_blob_store, sizeof(cbt_blob_t), 4u, CB_OVERWRITE_OLDEST) == CB_OK, "cb_init");

    uint32_t copies = 0u, torn = 0u, pops = 0u, order = 0u, prev = 0u, iter = 0u;
    bool have_prev = false;
    uint32_t rng = 0xC0FFEEu;
    cbt_blob_t b;

    TEST_irq_start(20000u, cbt_hook_push_blob);
    const uint32_t t_end = HAL_GetTick() + 300u;
    while ((int32_t)(HAL_GetTick() - t_end) < 0) {
        if (cb_peek(&cbt_isr_cb, (size_t)(cbt_rand(&rng) & 3u), &b) == CB_OK) {
            copies++;
            if (!cbt_blob_ok(&b)) torn++;
        }
        if (cb_pop(&cbt_isr_cb, &b) == CB_OK) {
            copies++;
            pops++;
            if (!cbt_blob_ok(&b)) torn++;
            if (have_prev && b.w[0] <= prev) order++;
            prev      = b.w[0];
            have_prev = true;
        }
        /* Se laisser depasser : 1 ms = 20 elements > capacite 4. */
        if ((++iter & 0xFFu) == 0u) {
            cbt_busy_wait_ms(1u);
        }
    }
    TEST_irq_stop();

    TEST_ASSERT(cbt_isr_n > 4000u, "ISR TIM5 : %lu appels seulement", (unsigned long)cbt_isr_n);
    TEST_ASSERT(torn == 0u, "%lu copie(s) dechiree(s) sur %lu", (unsigned long)torn, (unsigned long)copies);
    TEST_ASSERT(order == 0u, "%lu pop(s) hors ordre", (unsigned long)order);
    TEST_ASSERT(pops > 1000u, "Seulement %lu pops", (unsigned long)pops);
    TEST_ASSERT(cbt_isr_overwrote > 0u, "Aucun ecrasement cote ISR : test non concluant");

    snprintf(tc->detail, sizeof(tc->detail), "%lu copies de 64 o, 0 dechiree ; %lu pops ordonnes, %lu ecrasements ISR",
             (unsigned long)copies, (unsigned long)pops, (unsigned long)cbt_isr_overwrote);
    tc->result = R_PASS;
}
