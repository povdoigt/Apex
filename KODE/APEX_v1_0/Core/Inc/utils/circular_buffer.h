#include "stm32f4xx_hal.h"

#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

typedef struct CIRCULAR_BUFFER_WRITTER {
    uint8_t *buffer;    // Pointer to the buffer
    size_t head;        // Index of the head of the buffer
    size_t capacity;    // Maximum size of the buffer in elements
    size_t max_size;    // Maximum size of the buffer in bytes
    size_t data_size;   // Size of each data element in the buffer
} CIRCULAR_BUFFER_WRITTER;

typedef struct CIRCULAR_BUFFER_READER {
    CIRCULAR_BUFFER_WRITTER *cbw;
    size_t tail;
} CIRCULAR_BUFFER_READER;

#define CIRCULAR_BUFFER_SIZE(data_size, capacity) (capacity * data_size)

void CIRCULAR_BUFFER_init(CIRCULAR_BUFFER_WRITTER *cbw, uint8_t *buffer, size_t data_size, size_t capacity);

void CIRCULAR_BUFFER_push(CIRCULAR_BUFFER_WRITTER *cbw, void *data);

void CIRCULAR_BUFFER_pop(CIRCULAR_BUFFER_READER *cbr, void *data);

void CIRCULAR_BUFFER_set_from(CIRCULAR_BUFFER_WRITTER *cbw, void *data, size_t origine, int i);

void CIRCULAR_BUFFER_get_from(CIRCULAR_BUFFER_READER *cbr, void *data, size_t origine, int i);

// void CIRCULAR_BUFFER_set_from_tail(CIRCULAR_BUFFER *cb, void *data, int i);

// void CIRCULAR_BUFFER_set_from_head(CIRCULAR_BUFFER *cb, void *data, int i);

// void CIRCULAR_BUFFER_get_from_tail(CIRCULAR_BUFFER *cb, void *data, int i);

// void CIRCULAR_BUFFER_get_from_head(CIRCULAR_BUFFER *cb, void *data, int i);

#endif // CIRCULAR_BUFFER_H
