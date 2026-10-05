#include "project.h"

#include "vt100.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/* ========================================================================
 * Campagne de stress de la gestion de donnees (revue du 05/10/2026)
 *   1. Les quatre suites (RTOS, et sequentielles rejouees sous RTOS)
 *      executees STRESS_PASSES fois : chaque echec est compte par cas.
 *   2. Endurance "facon mission" pendant STRESS_ENDURANCE_S secondes.
 *   3. Durees des sections critiques (DWT).
 * Rien n'est imprime avant la fin : le rapport complet attend le terminal.
 * ======================================================================== */

#define STRESS_PASSES        50u
#define STRESS_ENDURANCE_S   600u

typedef struct {
    const char        *name;
    TEST_case_table_t *table;
    size_t             n;
    uint16_t           fails[32];
    uint16_t           skips[32];
} suite_t;

static suite_t suites[] = {
    { .name = "DT RTOS",  .table = DT_rtos_test_cases, .n = DT_rtos_test_N_TESTS },
    { .name = "DT seq",   .table = DT_seq_test_cases,  .n = DT_seq_test_N_TESTS  },
    { .name = "CB seq",   .table = CB_seq_test_cases,  .n = CB_seq_test_N_TESTS  },
    { .name = "DP seq",   .table = DP_seq_test_cases,  .n = DP_seq_test_N_TESTS  },
};
#define N_SUITES (sizeof(suites) / sizeof(suites[0]))

static const bool all_enabled[32] = {
    true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true,
    true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true,
};

static char     first_fail[224];
static uint32_t total_fails;

/* ========================================================================
 * setup_pre_kernel() – avant osKernelStart(), aucune tache ne tourne
 * ======================================================================== */
void setup_pre_kernel(void) {
    DT_rtos_test_pre_kernel();
}

/* ========================================================================
 * setup() – execution unique dans le thread applicatif, kernel demarre
 * ======================================================================== */
void setup(void) {
    _Static_assert(DT_seq_test_N_TESTS <= 32 && CB_seq_test_N_TESTS <= 32 &&
                   DP_seq_test_N_TESTS <= 32 && DT_rtos_test_N_TESTS <= 32, "fails[] / all_enabled too short");

    DT_rtos_test_init();

    /* 1. Suites en boucle. */
    const uint32_t t0 = osKernelGetTickCount();
    for (uint32_t pass = 1u; pass <= STRESS_PASSES; pass++) {
        for (size_t s = 0; s < N_SUITES; s++) {
            suite_t *su = &suites[s];
            TEST_configure_cases(su->table, su->n, all_enabled);
            TEST_perform_cases(su->table, su->n);
            for (size_t i = 0; i < su->n; i++) {
                const TEST_case_t *c = &su->table[i].case_info;
                if (c->result == R_FAIL) {
                    su->fails[i]++;
                    total_fails++;
                    if (first_fail[0] == '\0') {
                        snprintf(first_fail, sizeof(first_fail), "passe %lu, %s %s : %s",
                                 (unsigned long)pass, su->name, c->name, c->detail);
                    }
                } else if (c->result == R_SKIP) {
                    su->skips[i]++;
                }
            }
        }
    }
    const uint32_t t_suites = osKernelGetTickCount() - t0;

    /* 2. Endurance, 3. Durees. */
    const bool endurance_ok = DT_stress_endurance_run(STRESS_ENDURANCE_S);
    DT_stress_timing_run();
    const unsigned long stack_free = (unsigned long)uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t);

    /* Rapport. */
    TEST_wait_host();
    TEST_usb_print(VT100_SCREEN_CLEAR);
    char line[200];
    snprintf(line, sizeof(line), "===== Stress data_topic / circular_buffer / data_packet =====\r\n"
                                 "1. Suites x %u (%lu s) : %lu echec(s)\r\n",
             STRESS_PASSES, (unsigned long)(t_suites / 1000u), (unsigned long)total_fails);
    TEST_usb_print(line);
    for (size_t s = 0; s < N_SUITES; s++) {
        const suite_t *su = &suites[s];
        uint32_t fails = 0u;
        for (size_t i = 0; i < su->n; i++) {
            fails += su->fails[i];
        }
        snprintf(line, sizeof(line), "  %-8s %2u cas x %u passes : %lu echec(s)\r\n",
                 su->name, (unsigned)su->n, STRESS_PASSES, (unsigned long)fails);
        TEST_usb_print(line);
        for (size_t i = 0; i < su->n; i++) {
            if (su->fails[i] != 0u) {
                snprintf(line, sizeof(line), "    %-26s %u/%u echecs\r\n", su->table[i].case_info.name,
                         (unsigned)su->fails[i], STRESS_PASSES);
                TEST_usb_print(line);
            }
            if (su->skips[i] != 0u) {
                snprintf(line, sizeof(line), "    %-26s %u/%u SKIP (attendu : etat pre-noyau unique)\r\n",
                         su->table[i].case_info.name, (unsigned)su->skips[i], STRESS_PASSES);
                TEST_usb_print(line);
            }
        }
    }
    if (first_fail[0] != '\0') {
        TEST_usb_print("  Premier echec : ");
        TEST_usb_print(first_fail);
        TEST_usb_print("\r\n");
    }

    DT_stress_endurance_print(TEST_usb_print);
    DT_stress_timing_print(TEST_usb_print);

    snprintf(line, sizeof(line), "\r\nVerdict : suites %s, endurance %s\r\n"
                                 "Pile du thread de test jamais utilisee : %lu octets\r\nEND_OF_REPORT\r\n",
             (total_fails == 0u) ? "PASS" : "FAIL", endurance_ok ? "PASS" : "FAIL", stack_free);
    TEST_usb_print(line);
}
