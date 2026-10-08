/* Host harness: runs the sequential suites on the PC (gcc), to iterate fast
   on pure logic. The interrupt-driven cases cannot pass here (no TIM5): they
   are reported but expected to fail with "ISR TIM5 : 0 appels". */
#include "test.h"
#include "test_irq.h"
#include "cb_seq_test.h"
#include "dt_seq_test.h"
#ifdef WITH_DP
#include "dp_seq_test.h"
#endif

#include <stdio.h>
#include <string.h>
#include <time.h>

uint32_t HAL_GetTick(void) {
    return (uint32_t)((clock() * 1000ull) / CLOCKS_PER_SEC);
}

void TEST_irq_start(uint32_t rate_hz, void (*hook)(void)) { (void)rate_hz; (void)hook; }
void TEST_irq_stop(void) {}
uint32_t TEST_irq_count(void) { return 0u; }

void TEST_configure_cases(TEST_case_table_t table[], size_t n_cases, const bool enable[]) {
    for (size_t i = 0; i < n_cases; i++) table[i].case_info.result = enable[i] ? R_PASS : R_SKIP;
}

void TEST_perform_cases(TEST_case_table_t table[], size_t n_cases) {
    for (size_t i = 0; i < n_cases; i++) {
        if (table[i].case_info.result != R_SKIP) table[i].func(&table[i].case_info);
    }
}

static int run(const char *name, TEST_case_table_t *t, size_t n) {
    bool en[64];
    for (size_t i = 0; i < n; i++) en[i] = true;
    TEST_configure_cases(t, n, en);
    TEST_perform_cases(t, n);
    int fail = 0;
    printf("== %s\n", name);
    for (size_t i = 0; i < n; i++) {
        const char *tag = t[i].case_info.result == R_PASS ? "PASS" : t[i].case_info.result == R_FAIL ? "FAIL" : "SKIP";
        if (t[i].case_info.result == R_FAIL) fail++;
        printf("  [%s] %-24s %s\n", tag, t[i].case_info.name, t[i].case_info.detail);
    }
    return fail;
}

int main(void) {
    int fail = 0;
    fail += run("CB seq", CB_seq_test_cases, CB_seq_test_N_TESTS);
    fail += run("DT seq", DT_seq_test_cases, DT_seq_test_N_TESTS);
#ifdef WITH_DP
    fail += run("DP seq", DP_seq_test_cases, DP_seq_test_N_TESTS);
#endif
    printf("%d FAIL\n", fail);
    return 0;
}
