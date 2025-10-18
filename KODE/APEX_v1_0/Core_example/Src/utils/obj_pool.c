#include "utils/obj_pool.h"



void OBJ_POOL_init(OBJ_POOL *pool, size_t obj_size, size_t capacity) {
    pool->obj_size = obj_size;
    pool->capacity = capacity;
    pool->count = 0;
    pool->free_obj_wraps = (OBJ_WRAPPER *)pool->obj_wraps;

    OBJ_WRAPPER *prev = (OBJ_WRAPPER *)pool->obj_wraps;
    for (size_t i = 1; i < capacity; i++) {
        OBJ_WRAPPER *obj_wrap = (OBJ_WRAPPER *)((uintptr_t)(pool->obj_wraps) + i * OBJ_WRAPPER_SIZE(obj_size));
        prev->next = obj_wrap;
        prev = obj_wrap;
    }
    prev->next = NULL;
}

void *OBJ_POOL_alloc(OBJ_POOL *pool) {
    if (pool->free_obj_wraps == NULL) {
        return NULL;
    }

    OBJ_WRAPPER *obj_wrap = pool->free_obj_wraps;
    pool->free_obj_wraps = obj_wrap->next;
    pool->count++;
    return obj_wrap->obj;
}

void OBJ_POOL_free(OBJ_POOL *pool, void *obj) {
    OBJ_WRAPPER *obj_wrap = (OBJ_WRAPPER *)((uintptr_t)obj - offsetof(OBJ_WRAPPER, obj));

    obj_wrap->next = pool->free_obj_wraps;
    pool->free_obj_wraps = obj_wrap;

    pool->count--;
}

void *OBJ_POOL_get(OBJ_POOL *pool, size_t idx) {
    if (idx >= pool->capacity) {
        return NULL;
    }

    return (void *)((uintptr_t)(pool->obj_wraps) + idx * OBJ_WRAPPER_SIZE(pool->obj_size) + offsetof(OBJ_WRAPPER, obj));
}
