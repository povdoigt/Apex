#ifndef QUEUE_H
#define QUEUE_H

#include "obj_pool.h"
#include "stm32f4xx_hal.h"


struct QUEUE_WRAPPER {
    struct QUEUE_WRAPPER *queue_next;
    uint8_t obj[];
};
typedef struct QUEUE_WRAPPER QUEUE_WRAPPER;


typedef struct QUEUE {
    QUEUE_WRAPPER *head;
    QUEUE_WRAPPER *tail;
    size_t obj_size;
    uint8_t pool[];
} QUEUE;


#define QUEUE_WRAPPER_SIZE(obj_size) (sizeof(QUEUE_WRAPPER) + obj_size)
#define QUEUE_SIZE(obj_size, capacity) (sizeof(QUEUE) + OBJ_WRAPPER_SIZE(QUEUE_WRAPPER_SIZE(obj_size)) * (capacity))


void QUEUE_init(QUEUE *queue, size_t obj_size, size_t capacity);

void *QUEUE_push(QUEUE *queue, void *obj);
void *QUEUE_pop(QUEUE *queue);

#endif