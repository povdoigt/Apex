#include "utils/data_publisher.h"

void DATA_PUB_init(DATA_PUB_WRITTER *dpw, uint8_t *buffer, size_t data_size, size_t capacity) {
    CIRCULAR_BUFFER_init(&dpw, buffer, data_size, capacity);
}

void DATA_PUB_push(DATA_PUB_WRITTER *dpw, void *data) {
    CIRCULAR_BUFFER_push(&dpr->dpw, data);
    dpr->new_data_num += dpr->new_data_num < dpr->dpw.capacity ? 1 : 0;
}

void DATA_PUB_pop(DATA_PUB_READ *dpr, void *data) {
    CIRCULAR_BUFFER_pop(&dpr, data);
    // TODO: maybe need to change to read always
    data_pub->new_data_num -= data_pub->new_data_num > 0 ? 1 : 0;
}
