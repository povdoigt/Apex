#include "data_packet.h"
#include "circular_buffer.h"
#include "data_topic.h"

#include <string.h>

void data_packer_init(data_packer_t *packer, uint32_t window_ms, size_t num_topics,
                      data_topic_t **topics, size_t cb_capacity, void *storage) {
    packer->T = window_ms;
    packer->num_topics = num_topics;

    size_t data_packet_size = sizeof(data_ts_packet_generic_t);
    for (size_t i = 0; i < num_topics; i++) {
        packer->subs[i] = (data_sub_t){ 0 };
        data_sub_attach(&packer->subs[i], topics[i], DATA_ATTACH_FROM_NOW);
        data_packet_size += topics[i]->cb.elem_size;
    }
    data_topic_init(&packer->topic, storage, data_packet_size, cb_capacity, CB_OVERWRITE_OLDEST);
}

data_packer_status_t data_packer_check(data_packer_t *packer, data_sub_t *sub, uint32_t current_time_ms, void *out_ptr) {
    const data_ts_generic_t *data_ptr = NULL;
    
    data_status_t status = data_sub_peek_last_ptr(sub, (const void **)&data_ptr);
    while(status != DT_EMPTY && data_ptr->ts < current_time_ms - packer->T / 2) {
        data_sub_read_ptr(sub, (const void **)&data_ptr); // read and discard the old data
        status = data_sub_peek_last_ptr(sub, (const void **)&data_ptr);
    }

    if (status == DT_EMPTY) {
        return PACKER_EMPTY;
    } else if (data_ptr->ts > current_time_ms + packer->T / 2) {
        return PACKER_TOO_YOUNG;
    } else {
        memcpy(out_ptr, data_ptr->data, sub->topic->cb.elem_size - sizeof(data_ts_generic_t));
        data_sub_read_ptr(sub, (const void **)&data_ptr); // just to advance the subscriber's tail
        return PACKER_VALID;
    }
}

uint32_t data_packer_build_publish(data_packer_t *packer, uint32_t current_time_ms) {
    data_ts_packet_generic_t *packet = (data_ts_packet_generic_t *)cb_peek_relative_ptr(&packer->topic.cb, packer->topic.cb.head, 1);
    packet->ts = current_time_ms;
    packet->flags = 0;
    size_t offset = 0;
    for (size_t i = 0; i < packer->num_topics; i++) {
        uint8_t *data = packet->data + offset;
        data_packer_status_t status = data_packer_check(packer, packer->subs + i, packet->ts, data);
        if (status == PACKER_VALID) {
            packet->flags |= (1 << i);
        }
        offset += packer->subs[i].topic->cb.elem_size - sizeof(data_ts_generic_t);
    }
    data_topic_publish(&packer->topic, packet);
    return packet->flags;
}