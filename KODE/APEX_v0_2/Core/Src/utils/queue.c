#include "utils/queue.h"
#include <string.h>

void QUEUE_init(QUEUE *queue, size_t obj_size, size_t capacity) {
    queue->head = NULL;
    queue->tail = NULL;
    queue->obj_size = obj_size;

    OBJ_POOL *pool = (OBJ_POOL *)queue->pool;
    OBJ_POOL_init(pool, QUEUE_WRAPPER_SIZE(obj_size), capacity);
}

void *QUEUE_push(QUEUE *queue, void *obj) {
    OBJ_POOL *pool = (OBJ_POOL *)queue->pool;
    
    QUEUE_WRAPPER *queue_wrap = (QUEUE_WRAPPER *)OBJ_POOL_alloc(pool);
    if (queue_wrap == NULL) {
        return NULL; // Queue is full
    }

    queue_wrap->queue_next = NULL;
    memcpy(queue_wrap->obj, obj, queue->obj_size); // !!!

    if (queue->head != NULL) {
        // queue->tail->queue_next = queue_wrap; // tail ou head ????
        queue->head->queue_next = queue_wrap; // !!!!
    }
    queue->head = queue_wrap;

    if (queue->tail == NULL) {
        queue->tail = queue_wrap;
    }

    return queue_wrap->obj;
}

void *QUEUE_pop(QUEUE *queue) {
    if (queue->tail == NULL) {
        return NULL;
    }

    void *obj = queue->tail->obj;
    QUEUE_WRAPPER *queue_next = (QUEUE_WRAPPER *)(queue->tail->queue_next);

    OBJ_POOL *pool = (OBJ_POOL *)queue->pool;
    OBJ_POOL_free(pool, queue->tail);
    if (queue->tail == queue->head) {
        queue->head = NULL;
        queue->tail = NULL;
    } else {
        queue->tail = queue_next; // if queue_next == NULL, queue->tail will be NULL
    }

    return obj;
}