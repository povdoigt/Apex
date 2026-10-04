/**
 *******************************************************************************
 * @file    bench.h
 * @brief   On-target micro-benchmark helpers: DWT cycle counter + statistics.
 *
 * @details
 * Measure with BENCH_now() around the code under test, store one sample per
 * iteration in a caller-owned buffer, then reduce it with BENCH_summarize().
 * The summary keeps min / median / p99 / max / mean in CPU cycles, so nothing
 * is printed while measuring: print the result table once the run is over,
 * USB traffic would otherwise disturb the figures.
 *
 * Resolution: one CPU cycle (10.4 ns at 96 MHz). The 32-bit counter wraps
 * after 2^32 cycles (44.7 s at 96 MHz): a single sample must stay below that.
 *
 * Interrupts (SysTick, HAL timebase, USB SOF, run-time stats timer) are left
 * enabled: they show in max / p99, while the median stays clean.
 *******************************************************************************
 */

#ifndef BENCH_H
#define BENCH_H

#include <stddef.h>
#include <stdint.h>

#include "stm32f4xx.h"

/** @brief Summary of one measured quantity. All times are in CPU cycles. */
typedef struct {
    const char *group;   /**< Section header in the report (rows of a group are contiguous). */
    const char *name;    /**< Row label.                                    */
    uint32_t    n;       /**< Number of samples.                            */
    uint32_t    bytes;   /**< Payload bytes per sample, 0 if not relevant.  */
    uint32_t    min;
    uint32_t    p50;
    uint32_t    p99;
    uint32_t    max;
    uint32_t    mean;
} BENCH_result_t;

/** @brief Enable the DWT cycle counter. Idempotent. */
void BENCH_init(void);

/** @brief Current cycle count. */
static inline uint32_t BENCH_now(void) {
    return DWT->CYCCNT;
}

/** @brief Convert cycles to microseconds at the current core clock. */
float BENCH_cyc_to_us(uint32_t cyc);

/**
 * @brief Reduce a sample buffer into a summary.
 *
 * Sorts `samples` in place (insertion sort: meant for a few hundred samples).
 * With n == 0 the summary is zeroed apart from the labels.
 */
void BENCH_summarize(BENCH_result_t *r, const char *group, const char *name,
                     uint32_t bytes, uint32_t *samples, uint32_t n);

/**
 * @brief Print a result table: one header per group, times in microseconds,
 *        throughput (KiB/s, on the median) when `bytes` is set.
 */
void BENCH_print_table(const BENCH_result_t *r, size_t n, void (*print)(const char *));

/** @brief Print the same results as CSV, between BEGIN / END marker lines. */
void BENCH_print_csv(const BENCH_result_t *r, size_t n, void (*print)(const char *));

#endif /* BENCH_H */
