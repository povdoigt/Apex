#ifndef TEST_H
#define TEST_H

#include <stddef.h>
#include <stdbool.h>

typedef enum TEST_result_t {
    R_PASS = 0,
    R_FAIL,
    R_SKIP
} TEST_result_t;

typedef struct TEST_case_t {
	const char		name[32];
	TEST_result_t	result;
	char			detail[128];
} TEST_case_t;

// Test functions prototypes: each function takes a pointer to a TEST_case_t
// struct, which it fills with the test result and details.
typedef void (*TEST_func_t)(TEST_case_t *tc);

typedef struct TEST_case_table_t {
    TEST_case_t case_info;
    TEST_func_t func;
} TEST_case_table_t;

void TEST_configure_cases(TEST_case_table_t table[], size_t n_cases, const bool enable[]);

void TEST_perform_cases(TEST_case_table_t table[], size_t n_cases);

void TEST_print_case_result(const TEST_case_table_t *table, size_t n_cases, void (*print_func)(const char *),
                            const char suite_name[32], const char suite_desc[128]);

#endif // TEST_H