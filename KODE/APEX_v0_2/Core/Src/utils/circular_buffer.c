#include "utils/circular_buffer.h"


void CIRCULAR_BUFFER_one_byte_push(CIRCULAR_BUFFER *cb, uint8_t data);
uint8_t CIRCULAR_BUFFER_one_byte_pop(CIRCULAR_BUFFER *cb);
size_t CIRCULAR_BUFFER_get_relative_index(CIRCULAR_BUFFER *cb, size_t origine, int i);
void CIRCULAR_BUFFER_set_from(CIRCULAR_BUFFER *cb, void *data, size_t origine, int i);
void CIRCULAR_BUFFER_get_from(CIRCULAR_BUFFER *cb, void *data, size_t origine, int i);


void CIRCULAR_BUFFER_init(CIRCULAR_BUFFER *cb, uint8_t *buffer, size_t data_size, size_t capacity) {
    cb->buffer = buffer;
    cb->head = 0;
    cb->tail = 0;
    cb->capacity = capacity;
    cb->max_size = data_size * capacity;
    cb->data_size = data_size;
}

void CIRCULAR_BUFFER_one_byte_push(CIRCULAR_BUFFER *cb, uint8_t data) {
    cb->buffer[cb->head] = data;
    cb->head = (cb->head + 1) % cb->max_size;
}

uint8_t CIRCULAR_BUFFER_one_byte_pop(CIRCULAR_BUFFER *cb) {
    uint8_t data = cb->buffer[cb->tail];
    cb->tail = (cb->tail + 1) % cb->max_size;
    return data;
}

void CIRCULAR_BUFFER_push(CIRCULAR_BUFFER *cb, void *data) {
    for (size_t i = 0; i < cb->data_size; i++) {
        CIRCULAR_BUFFER_one_byte_push(cb, *(uint8_t *)((uintptr_t)data + i));
    }
}

void CIRCULAR_BUFFER_pop(CIRCULAR_BUFFER *cb, void *data) {
    for (size_t i = 0; i < cb->data_size; i++) {
        *(uint8_t *)((uintptr_t)data + i) = CIRCULAR_BUFFER_one_byte_pop(cb);
    }
}

size_t CIRCULAR_BUFFER_get_relative_index(CIRCULAR_BUFFER *cb, size_t origine, int i) {
    int i0 = ((int)origine + i * ((int)(cb->data_size))) % ((int)(cb->max_size));
    if (i0 < 0) {
        i0 += cb->max_size;
    }
    return (size_t)i0;
}

void CIRCULAR_BUFFER_set_from(CIRCULAR_BUFFER *cb, void *data, size_t origine, int i) {
    size_t index = CIRCULAR_BUFFER_get_relative_index(cb, origine, i);
    for (size_t j = 0; j < cb->data_size; j++) {
        cb->buffer[index + j] = *(uint8_t *)((uintptr_t)data + j);
    }
}

void CIRCULAR_BUFFER_get_from(CIRCULAR_BUFFER *cb, void *data, size_t origine, int i) {
    size_t index = CIRCULAR_BUFFER_get_relative_index(cb, origine, i);
    for (size_t j = 0; j < cb->data_size; j++) {
        *(uint8_t *)((uintptr_t)data + j) = cb->buffer[index + j];
    }
}

void CIRCULAR_BUFFER_set_from_tail(CIRCULAR_BUFFER *cb, void *data, int i) {
    CIRCULAR_BUFFER_set_from(cb, data, cb->tail, i);
}

void CIRCULAR_BUFFER_set_from_head(CIRCULAR_BUFFER *cb, void *data, int i) {
    CIRCULAR_BUFFER_set_from(cb, data, cb->head, i);
}

void CIRCULAR_BUFFER_get_from_tail(CIRCULAR_BUFFER *cb, void *data, int i) {
    CIRCULAR_BUFFER_get_from(cb, data, cb->tail, i);
}

void CIRCULAR_BUFFER_get_from_head(CIRCULAR_BUFFER *cb, void *data, int i) {
    CIRCULAR_BUFFER_get_from(cb, data, cb->head, i);
}
