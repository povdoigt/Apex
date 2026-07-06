#ifndef DATA_PACKET_H
#define DATA_PACKET_H

#include "data_topic.h"
#include "main_config.h"

#include <stdint.h>
#include <stddef.h>

#if (APEX_CFG_SCHED_RTOS == 1)
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define DATA_PACKET_MAX_TOPICS 32

typedef struct data_ts_generic_t {
    uint32_t ts; // Timestamp in milliseconds (HAL)
    uint8_t data[]; // Data payload (variable length)
} data_ts_generic_t;

typedef struct data_ts_packet_generic_t {
    uint32_t ts; // Timestamp in milliseconds (HAL)
    uint32_t flags; // Flags for validity of each data field (bit 0 = topic 0, bit 1 = topic 1, etc.)
    uint8_t data[]; // Data payload (variable length)
} data_ts_packet_generic_t;

typedef struct data_packer_t {
    uint32_t T; // Window time size in milliseconds
    size_t num_topics; // Number of topics
    data_topic_t topic; // Publisher for the packed data
    data_sub_t subs[DATA_PACKET_MAX_TOPICS];
} data_packer_t;

typedef enum data_packer_status_t{
    PACKER_EMPTY     = 0,  // Data topic is empty, no data to read
    PACKER_TOO_YOUNG = 1,  // Data is too young, outside the window
    PACKER_VALID     = 2,  // Data is valid and within the window
} data_packer_status_t;

void data_packer_init(data_packer_t *packer, uint32_t window_ms, size_t num_topics,
                      data_topic_t **topics, size_t cb_capacity, void *storage);

uint32_t data_packer_build_publish(data_packer_t *packer, uint32_t current_time_ms);

#ifdef __cplusplus
}
#endif

#endif /* DATA_PACKET_H */
