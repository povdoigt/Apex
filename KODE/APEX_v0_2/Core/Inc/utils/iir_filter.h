#include "circular_buffer.h"
#include "stm32f4xx_hal.h"

#ifndef IIR_FILTER_H
#define IIR_FILTER_H

typedef struct IIR_FILTER {
    float *a; // Coefficients for the feedback loop
    float *b; // Coefficients for the feedforward loop
    CIRCULAR_BUFFER *x; // Input buffer
    CIRCULAR_BUFFER *y; // Output buffer
    size_t order;       // Order of the filter
} IIR_FILTER;

void IIR_FILTER_init_no_alloc(IIR_FILTER *filter, size_t order,
                              float *a_coef, float *b_coef,
                              CIRCULAR_BUFFER *x, CIRCULAR_BUFFER *y);

void IIR_FILTER_init(IIR_FILTER *filter, size_t order, float *a_coef, float *b_coef);

void IIR_FILTER_free(IIR_FILTER *filter);

float IIR_FILTER_process(IIR_FILTER *filter, float input);

#endif // IIR_FILTER_H
