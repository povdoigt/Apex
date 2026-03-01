#include "test.h"

#include "vt100.h"

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

void TEST_configure_cases(TEST_case_table_t table[], size_t n_cases, const bool enable[]) {
    for (size_t i = 0; i < n_cases; i++) {
        table[i].case_info.result = enable[i] ? R_PASS : R_SKIP;
    }
}

void TEST_perform_cases(TEST_case_table_t table[], size_t n_cases) {
    for (size_t i = 0; i < n_cases; i++) {
        if (table[i].case_info.result != R_SKIP) {
            table[i].func(&table[i].case_info);
        }
    }
}

void TEST_print_case_result(const TEST_case_table_t *table, size_t n_cases, void (*print_func)(const char *),
                            const char suite_name[32], const char suite_desc[128]) {
    char log_buf[256];

    uint32_t n_pass = 0, n_fail = 0;
    for (size_t i = 0; i < n_cases; i++) {
        if (table[i].case_info.result == R_PASS) {
            n_pass++;
        } else if (table[i].case_info.result == R_FAIL) {
            n_fail++;
        }
    }

    print_func(VT100_SCREEN_CLEAR);

    snprintf(log_buf, sizeof(log_buf), VT100_FG_CYAN "===== %s Test Suite =====" VT100_RESET "\r\n"
                                                     "%s (%d test cases)\r\n\r\n",
            suite_name, suite_desc, n_cases);
    print_func(log_buf);

    for (size_t i = 0; i < n_cases; i++) {
        const char *col = (table[i].case_info.result == R_PASS) ? VT100_FG_GREEN
                        : (table[i].case_info.result == R_FAIL) ? VT100_FG_RED
                        : VT100_FG_YELLOW;
        const char *tag = (table[i].case_info.result == R_PASS) ? "PASS"
                        : (table[i].case_info.result == R_FAIL) ? "FAIL" : "SKIP";
        snprintf(log_buf, sizeof(log_buf),
                    "  %s[%s]" VT100_RESET " %-32s %s\r\n",
                    col, tag, table[i].case_info.name, table[i].case_info.detail);
        print_func(log_buf);
    }

    const char *vcol = (n_fail == 0) ? VT100_BG_GREEN VT100_FG_BLACK
                                     : VT100_BG_RED   VT100_FG_BLACK;
    snprintf(log_buf, sizeof(log_buf),
                "\r\n%s  %lu/%d PASS   %lu FAIL  " VT100_RESET "\r\n",
                vcol, (unsigned long)n_pass, n_cases, (unsigned long)n_fail);
    print_func(log_buf);
}