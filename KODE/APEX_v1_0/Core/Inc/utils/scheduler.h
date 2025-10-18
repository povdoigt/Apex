#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <stdbool.h>
#include <stddef.h>

#include "cmsis_os2.h"
#include "freertos_mpool.h"
#include "stm32f4xx_hal.h"

#include "utils/obj_pool.h"
#include "utils/circular_buffer.h"


#define MAX_TASK 30


#define TASK_POOL_CONFIGURE(task_name, number, stack_size)  \
    enum { task_name##_NUMBER = number };                   \
    enum { task_name##_STACK_SIZE = stack_size };           \
    TASK_POOL_OBJ_DECLARE(task_name);                       \
    TASK_POOL_DECLARE(task_name)

#define TASK_POOL_OBJ_DECLARE(task_name)                                    \
    typedef struct task_name##_POOL_OBJ {                                   \
        char name[configMAX_TASK_NAME_LEN];                                 \
        task_name##_ARGS args;                                              \
        StaticTask_t control_block;                                         \
        StackType_t stack[task_name##_STACK_SIZE / sizeof(StackType_t)];    \
    } task_name##_POOL_OBJ

#define TASK_POOL_OBJ_t(task_name) task_name##_POOL_OBJ

#define TASK_POOL_DECLARE(task_name)                                                                        \
    extern StaticMemPool_t task_name##_POOL_CB;                                                             \
    extern uint8_t task_name##_POOL[MEMPOOL_ARR_SIZE(task_name##_NUMBER, sizeof(task_name##_POOL_OBJ))];    \
    extern osMemoryPoolId_t task_name##_POOL_ID;

#define TASK_POOL_ALLOCATE(task_name)                                                               \
    StaticMemPool_t task_name##_POOL_CB;                                                            \
    uint8_t task_name##_POOL[MEMPOOL_ARR_SIZE(task_name##_NUMBER, sizeof(task_name##_POOL_OBJ))];   \
    osMemoryPoolId_t task_name##_POOL_ID = NULL

#define TASK_POOL_CREATE(task_name)                 \
    osMemoryPoolAttr_t task_name##_POOL_ATTR = {    \
        .name = #task_name "_POOL",                 \
        .cb_mem = &task_name##_POOL_CB,             \
        .cb_size = sizeof(task_name##_POOL_CB),     \
        .mp_mem = &task_name##_POOL,                \
        .mp_size = sizeof(task_name##_POOL),        \
    };                                              \
    task_name##_POOL_ID = (osMemoryPoolId_t) osMemoryPoolNew(          \
        task_name##_NUMBER,                         \
        sizeof(task_name##_POOL_OBJ),               \
        &task_name##_POOL_ATTR                      \
    )
    

osThreadId_t osThreadNew_Cstm(osThreadFunc_t func, osMemoryPoolId_t pool_id,
                              osThreadAttr_t *attr, void *args,
                              size_t name_size, size_t args_size, size_t cb_size, size_t stack_size,
                              size_t name_offset, size_t args_offset, size_t cb_offset, size_t stack_offset,
                              uint32_t timeout);

void osThreadExit_Cstm();

#define OS_THREAD_NEW_CSTM(task_name, _args, _attr, timeout)        \
    osThreadNew_Cstm(                                               \
        task_name,                                                  \
        task_name##_POOL_ID,                                        \
        &_attr,                                                     \
        &_args,                                                     \
        sizeof(((TASK_POOL_OBJ_t(task_name) *)0)->name),            \
        sizeof(((TASK_POOL_OBJ_t(task_name) *)0)->args),            \
        sizeof(((TASK_POOL_OBJ_t(task_name) *)0)->control_block),   \
        sizeof(((TASK_POOL_OBJ_t(task_name) *)0)->stack),           \
        offsetof(TASK_POOL_OBJ_t(task_name), name),                 \
        offsetof(TASK_POOL_OBJ_t(task_name), args),                 \
        offsetof(TASK_POOL_OBJ_t(task_name), control_block),        \
        offsetof(TASK_POOL_OBJ_t(task_name), stack),                \
        timeout                                                     \
    )


struct KEY_TASK_POOL_OBJ {
    osThreadId_t thread_id;
    osMemoryPoolId_t pool_id;
    void *obj;
    struct KEY_TASK_POOL_OBJ *next;
};
typedef struct KEY_TASK_POOL_OBJ KEY_TASK_POOL_OBJ;

typedef struct TASK_POOL_OBJ_TABLE {
    KEY_TASK_POOL_OBJ *head;
    size_t count;
    KEY_TASK_POOL_OBJ elems[MAX_TASK];
} TASK_POOL_OBJ_TABLE;

extern TASK_POOL_OBJ_TABLE task_pool_obj_table;




void TASK_POOL_OBJ_TABLE_init(TASK_POOL_OBJ_TABLE *table);

void TASK_POOL_OBJ_TABLE_add(TASK_POOL_OBJ_TABLE *table, osThreadId_t thread_id, osMemoryPoolId_t pool_id, void *obj);

KEY_TASK_POOL_OBJ *TASK_POOL_OBJ_TABLE_find(TASK_POOL_OBJ_TABLE *table, osThreadId_t thread_id);

void TASK_POOL_OBJ_TABLE_remove(TASK_POOL_OBJ_TABLE *table, KEY_TASK_POOL_OBJ *key);



typedef struct TASK_Idle_ARGS { } TASK_Idle_ARGS;

TASK_POOL_CONFIGURE(TASK_Idle, 1, 512);

void TASK_Idle(void *argument);

void Init_cleanup(void);


#endif // SCHEDULER_H