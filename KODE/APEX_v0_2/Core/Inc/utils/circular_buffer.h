#include "stm32f4xx_hal.h"

#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

typedef struct CIRCULAR_BUFFER {
    uint8_t *buffer;    // Pointer to the buffer
    size_t head;        // Index of the head of the buffer
    size_t tail;        // Index of the tail of the buffer
    size_t capacity;    // Maximum size of the buffer in elements
    size_t max_size;    // Maximum size of the buffer in bytes
    size_t data_size;   // Size of each data element in the buffer
} CIRCULAR_BUFFER;

#define CIRCULAR_BUFFER_SIZE(data_size, capacity) (capacity * data_size)

void CIRCULAR_BUFFER_init(CIRCULAR_BUFFER *cb, uint8_t *buffer, size_t data_size, size_t capacity);

void CIRCULAR_BUFFER_push(CIRCULAR_BUFFER *cb, void *data);

void CIRCULAR_BUFFER_pop(CIRCULAR_BUFFER *cb, void *data);

void CIRCULAR_BUFFER_set_from_tail(CIRCULAR_BUFFER *cb, void *data, int i);

void CIRCULAR_BUFFER_set_from_head(CIRCULAR_BUFFER *cb, void *data, int i);

void CIRCULAR_BUFFER_get_from_tail(CIRCULAR_BUFFER *cb, void *data, int i);

void CIRCULAR_BUFFER_get_from_head(CIRCULAR_BUFFER *cb, void *data, int i);

#endif // CIRCULAR_BUFFER_H
