#ifndef OBJ_POOL_H
#define OBJ_POOL_H


#include "stm32f4xx_hal.h"


typedef struct OBJ_WRAPPER {
    void *next;
    uint8_t obj[];
} OBJ_WRAPPER;

typedef struct OBJ_POOL {
    size_t obj_size;
    size_t capacity;
    size_t count;
    OBJ_WRAPPER *free_obj_wraps;
    uint8_t obj_wraps[];
} OBJ_POOL;


#define OBJ_WRAPPER_SIZE(obj_size) (sizeof(OBJ_WRAPPER) + (obj_size))
#define OBJ_POOL_SIZE(obj_size, capacity) (sizeof(OBJ_POOL) + OBJ_WRAPPER_SIZE(obj_size) * (capacity))


void OBJ_POOL_init(OBJ_POOL *pool, size_t obj_size, size_t capacity);
void *OBJ_POOL_alloc(OBJ_POOL *pool);
void OBJ_POOL_free(OBJ_POOL *pool, void *obj);
void *OBJ_POOL_get(OBJ_POOL *pool, size_t idx);


#endif // OBJ_POOL_H