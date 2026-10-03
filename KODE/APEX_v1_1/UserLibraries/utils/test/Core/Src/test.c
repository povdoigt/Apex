#include "test.h"

#include "vt100.h"

#include "main.h"
#include "cmsis_os2.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceFS;

// Sleep when the RTOS runs (other threads keep going), busy-wait otherwise.
static void test_delay_ms(uint32_t ms) {
    if (osKernelGetState() == osKernelRunning) {
        (void)osDelay(ms);
    } else {
        HAL_Delay(ms);
    }
}

void TEST_usb_print(const char *s) {
    size_t len = strlen(s);
    if (len == 0u) {
        return;
    }

    // A previous transfer may still be in flight: retry while busy, bounded.
    for (uint32_t tries = 0; tries < 100u; tries++) {
        if (CDC_Transmit_FS((uint8_t *)s, (uint16_t)len) != USBD_BUSY) {
            break;
        }
        test_delay_ms(1);
    }

    // The transfer reads `s` asynchronously: wait for its end before returning.
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    for (uint32_t tries = 0; hcdc != NULL && hcdc->TxState != 0u && tries < 100u; tries++) {
        test_delay_ms(1);
    }
}

void TEST_wait_host(void) {
    while (!cdc_port_open) {
        test_delay_ms(10);
    }
    test_delay_ms(50);
}

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

void TEST_get_pass_fail_count(const TEST_case_table_t table[], size_t n_cases, uint32_t *n_pass, uint32_t *n_fail) {
    uint32_t pass = 0, fail = 0;
    for (size_t i = 0; i < n_cases; i++) {
        if (table[i].case_info.result == R_PASS) {
            pass++;
        } else if (table[i].case_info.result == R_FAIL) {
            fail++;
        }
    }
    if (n_pass) *n_pass += pass;
    if (n_fail) *n_fail += fail;
}

void TEST_print_case_result(const TEST_case_table_t *table, size_t n_cases, void (*print_func)(const char *),
                            const char suite_name[32], const char suite_desc[128]) {
    char log_buf[256];

    uint32_t n_pass = 0, n_fail = 0;
    TEST_get_pass_fail_count(table, n_cases, &n_pass, &n_fail);

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