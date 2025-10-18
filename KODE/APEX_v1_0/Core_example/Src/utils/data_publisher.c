#include "utils/data_publisher.h"

void DATA_PUB_init(DATA_PUB *data_pub, uint8_t *buffer, size_t data_size, size_t capacity) {
    CIRCULAR_BUFFER_init(&data_pub->ring_data, buffer, data_size, capacity);
    data_pub->new_data_num = 0;
}

void DATA_PUB_push(DATA_PUB *data_pub, void *data) {
    CIRCULAR_BUFFER_push(&data_pub->ring_data, data);
    data_pub->new_data_num += data_pub->new_data_num < data_pub->ring_data.capacity ? 1 : 0;
}

void DATA_PUB_pop(DATA_PUB *data_pub, void *data) {
    CIRCULAR_BUFFER_pop(&data_pub->ring_data, data);
    // TODO: maybe need to change to read always
    data_pub->new_data_num -= data_pub->new_data_num > 0 ? 1 : 0;
}
