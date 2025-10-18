#include "utils/circular_buffer.h"


void CIRCULAR_BUFFER_one_byte_push(CIRCULAR_BUFFER_WRITTER *cbw, uint8_t data);
uint8_t CIRCULAR_BUFFER_one_byte_pop(CIRCULAR_BUFFER_READER *cbr);
size_t CIRCULAR_BUFFER_get_relative_index(CIRCULAR_BUFFER_WRITTER *cbw, size_t origine, int i);
void CIRCULAR_BUFFER_set_from(CIRCULAR_BUFFER_WRITTER *cbw, void *data, size_t origine, int i);
void CIRCULAR_BUFFER_get_from(CIRCULAR_BUFFER_READER *cbr, void *data, size_t origine, int i);


void CIRCULAR_BUFFER_init(CIRCULAR_BUFFER_WRITTER *cbw, uint8_t *buffer, size_t data_size, size_t capacity) {
    cbw->buffer = buffer;
    cbw->head = 0;
    cbw->capacity = capacity;
    cbw->max_size = data_size * capacity;
    cbw->data_size = data_size;
}

void CIRCULAR_BUFFER_one_byte_push(CIRCULAR_BUFFER_WRITTER *cbw, uint8_t data) {
    cbw->buffer[cbw->head] = data;
    cbw->head = (cbw->head + 1) % cbw->max_size;
}

uint8_t CIRCULAR_BUFFER_one_byte_pop(CIRCULAR_BUFFER_READER *cbr) {
    uint8_t data = cbr->cbw->buffer[cbr->tail];
    cbr->tail = (cbr->tail + 1) % cbr->cbw->max_size;
    return data;
}

void CIRCULAR_BUFFER_push(CIRCULAR_BUFFER_WRITTER *cbw, void *data) {
    for (size_t i = 0; i < cbw->data_size; i++) {
        CIRCULAR_BUFFER_one_byte_push(cbw, *(uint8_t *)((uintptr_t)data + i));
    }
}

void CIRCULAR_BUFFER_pop(CIRCULAR_BUFFER_READER *cbr, void *data) {
    for (size_t i = 0; i < cbr->cbw->data_size; i++) {
        *(uint8_t *)((uintptr_t)data + i) = CIRCULAR_BUFFER_one_byte_pop(cbr);
    }
}

size_t CIRCULAR_BUFFER_get_relative_index(CIRCULAR_BUFFER_WRITTER *cbw, size_t origine, int i) {
    int i0 = ((int)origine + i * ((int)(cbw->data_size))) % ((int)(cbw->max_size));
    if (i0 < 0) {
        i0 += cbw->max_size;
    }
    return (size_t)i0;
}

void CIRCULAR_BUFFER_set_from(CIRCULAR_BUFFER_WRITTER *cbw, void *data, size_t origine, int i) {
    size_t index = CIRCULAR_BUFFER_get_relative_index(cbw, origine, i);
    for (size_t j = 0; j < cbw->data_size; j++) {
        cbw->buffer[index + j] = *(uint8_t *)((uintptr_t)data + j);
    }
}

void CIRCULAR_BUFFER_get_from(CIRCULAR_BUFFER_READER *cbr, void *data, size_t origine, int i) {
    size_t index = CIRCULAR_BUFFER_get_relative_index(cbr->cbw, origine, i);
    for (size_t j = 0; j < cbr->cbw->data_size; j++) {
        *(uint8_t *)((uintptr_t)data + j) = cbr->cbw->buffer[index + j];
    }
}

// void CIRCULAR_BUFFER_set_from_tail(CIRCULAR_BUFFER *cb, void *data, int i) {
//     CIRCULAR_BUFFER_set_from(cb, data, cb->tail, i);
// }

// void CIRCULAR_BUFFER_set_from_head(CIRCULAR_BUFFER *cb, void *data, int i) {
//     CIRCULAR_BUFFER_set_from(cb, data, cb->head, i);
// }

// void CIRCULAR_BUFFER_get_from_tail(CIRCULAR_BUFFER *cb, void *data, int i) {
//     CIRCULAR_BUFFER_get_from(cb, data, cb->tail, i);
// }

// void CIRCULAR_BUFFER_get_from_head(CIRCULAR_BUFFER *cb, void *data, int i) {
//     CIRCULAR_BUFFER_get_from(cb, data, cb->head, i);
// }
