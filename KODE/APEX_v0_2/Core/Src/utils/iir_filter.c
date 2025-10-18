#include "utils/iir_filter.h"
#include "utils/gms.h"
#include <string.h>


void IIR_FILTER_alloc(IIR_FILTER *filter, size_t order) {
    filter->a = (float *)GMS_alloc(&GMS_memory, sizeof(float) * (order + 1));
    filter->b = (float *)GMS_alloc(&GMS_memory, sizeof(float) * (order + 1));
    filter->x = (CIRCULAR_BUFFER *)GMS_alloc(&GMS_memory, sizeof(CIRCULAR_BUFFER));
    filter->y = (CIRCULAR_BUFFER *)GMS_alloc(&GMS_memory, sizeof(CIRCULAR_BUFFER));
}

void IIR_FILTER_init_no_alloc(IIR_FILTER *filter, size_t order,
                               float *a_coef, float *b_coef,
                               CIRCULAR_BUFFER *x, CIRCULAR_BUFFER *y) {
    filter->order = order;
    filter->a = a_coef;
    filter->b = b_coef;
    filter->x = x;
    filter->y = y;

    filter->a[order] = 0.0f; // Set the last coefficient of a to 0.0f

    CIRCULAR_BUFFER_init(filter->x, order + 1, sizeof(float));
    CIRCULAR_BUFFER_init(filter->y, order + 1, sizeof(float));

    float zero = 0.0f;
    for (size_t i = 0; i < order + 1; i++) {
        CIRCULAR_BUFFER_push(filter->x, (uint8_t *)&zero);
        CIRCULAR_BUFFER_push(filter->y, (uint8_t *)&zero);
    }
}


void IIR_FILTER_init(IIR_FILTER *filter, size_t order, float *a_coef, float *b_coef) {
    IIR_FILTER_alloc(filter, order);
    filter->order = order;
    IIR_FILTER_init_no_alloc(filter, order, a_coef, b_coef, filter->x, filter->y);
    memset(filter->a, a_coef, sizeof(float) * (order));
    memset(filter->b, b_coef, sizeof(float) * (order + 1));
    filter->a[order] = 0.0f; // Set the last coefficient of a to 0.0f
}

void IIR_FILTER_free(IIR_FILTER *filter) {
    CIRCULAR_BUFFER_free(filter->x);
    CIRCULAR_BUFFER_free(filter->y);
    GMS_free(&GMS_memory, filter->a);
    GMS_free(&GMS_memory, filter->b);
    GMS_free(&GMS_memory, filter->x);
    GMS_free(&GMS_memory, filter->y);
}

float IIR_FILTER_process(IIR_FILTER *filter, float input) {
    CIRCULAR_BUFFER_push(&filter->x[0], (uint8_t *)&input);

    float output = 0.0f;
    for (size_t i = 0; i < filter->order + 1; i++) {
        float a_coef, b_coef, x_val, y_val;
        CIRCULAR_BUFFER_get_from_tail(filter->a, (uint8_t *)&a_coef, i);
        CIRCULAR_BUFFER_get_from_tail(filter->b, (uint8_t *)&b_coef, i);
        CIRCULAR_BUFFER_get_from_tail(filter->x, (uint8_t *)&x_val, i);
        CIRCULAR_BUFFER_get_from_tail(filter->y, (uint8_t *)&y_val, i);
        output += b_coef * x_val - a_coef * y_val;
    }
    CIRCULAR_BUFFER_push(filter->y, (uint8_t *)&output);

    return output;
}
