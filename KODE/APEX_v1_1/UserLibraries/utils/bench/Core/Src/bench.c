/**
 *******************************************************************************
 * @file    bench.c
 * @brief   On-target micro-benchmark helpers. @see bench.h
 *******************************************************************************
 */

#include "bench.h"

#include <stdio.h>
#include <string.h>

void BENCH_init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;
}

float BENCH_cyc_to_us(uint32_t cyc) {
    return (float)cyc * 1.0e6f / (float)SystemCoreClock;
}

static void sort_u32(uint32_t *a, uint32_t n) {
    for (uint32_t i = 1; i < n; i++) {
        uint32_t v = a[i];
        uint32_t j = i;
        while (j > 0u && a[j - 1u] > v) {
            a[j] = a[j - 1u];
            j--;
        }
        a[j] = v;
    }
}

/* Nearest-rank percentile on a sorted buffer, pct in 1..100. */
static uint32_t percentile(const uint32_t *sorted, uint32_t n, uint32_t pct) {
    uint32_t rank = (n * pct + 99u) / 100u;
    return sorted[(rank > 0u) ? rank - 1u : 0u];
}

void BENCH_summarize(BENCH_result_t *r, const char *group, const char *name,
                     uint32_t bytes, uint32_t *samples, uint32_t n) {
    memset(r, 0, sizeof(*r));
    r->group = group;
    r->name  = name;
    r->bytes = bytes;
    r->n     = n;
    if (n == 0u) {
        return;
    }

    sort_u32(samples, n);

    uint64_t sum = 0;
    for (uint32_t i = 0; i < n; i++) {
        sum += samples[i];
    }
    r->min  = samples[0];
    r->max  = samples[n - 1u];
    r->p50  = percentile(samples, n, 50u);
    r->p99  = percentile(samples, n, 99u);
    r->mean = (uint32_t)(sum / n);
}

/* KiB/s on the median, 0 when not applicable. */
static float throughput_kib_s(const BENCH_result_t *r) {
    if (r->bytes == 0u || r->p50 == 0u) {
        return 0.0f;
    }
    return ((float)r->bytes / 1024.0f) / (BENCH_cyc_to_us(r->p50) * 1.0e-6f);
}

void BENCH_print_table(const BENCH_result_t *r, size_t n, void (*print)(const char *)) {
    char line[192];
    const char *group = NULL;

    for (size_t i = 0; i < n; i++) {
        if (group == NULL || strcmp(group, r[i].group) != 0) {
            group = r[i].group;
            snprintf(line, sizeof(line),
                     "\r\n== %s\r\n%-34s %5s %6s %10s %10s %10s %10s %10s %9s\r\n",
                     group, "Mesure", "n", "octets",
                     "min us", "med us", "p99 us", "max us", "moy us", "KiB/s");
            print(line);
        }
        snprintf(line, sizeof(line),
                 "%-34s %5lu %6lu %10.2f %10.2f %10.2f %10.2f %10.2f %9.1f\r\n",
                 r[i].name, (unsigned long)r[i].n, (unsigned long)r[i].bytes,
                 BENCH_cyc_to_us(r[i].min), BENCH_cyc_to_us(r[i].p50),
                 BENCH_cyc_to_us(r[i].p99), BENCH_cyc_to_us(r[i].max),
                 BENCH_cyc_to_us(r[i].mean), throughput_kib_s(&r[i]));
        print(line);
    }
}

void BENCH_print_csv(const BENCH_result_t *r, size_t n, void (*print)(const char *)) {
    char line[192];

    print("\r\n--- CSV BEGIN ---\r\n");
    print("group,name,n,bytes,min_us,p50_us,p99_us,max_us,mean_us,kib_s\r\n");
    for (size_t i = 0; i < n; i++) {
        snprintf(line, sizeof(line), "%s,%s,%lu,%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f\r\n",
                 r[i].group, r[i].name, (unsigned long)r[i].n, (unsigned long)r[i].bytes,
                 BENCH_cyc_to_us(r[i].min), BENCH_cyc_to_us(r[i].p50),
                 BENCH_cyc_to_us(r[i].p99), BENCH_cyc_to_us(r[i].max),
                 BENCH_cyc_to_us(r[i].mean), throughput_kib_s(&r[i]));
        print(line);
    }
    print("--- CSV END ---\r\n");
}
