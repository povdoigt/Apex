
#include <complex.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "utils/scheduler.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"



// ================== SCHEUDLER ==================


static osMessageQueueId_t msg_queue_cleanup_id;
static StaticQueue_t msg_queue_cleanup_cb;
static uint8_t msg_queue_cleanup_buffer[sizeof(osThreadId_t) * MAX_TASK];

TASK_POOL_OBJ_TABLE task_pool_obj_table = {0};


osThreadId_t osThreadNew_Cstm(osThreadFunc_t func, osMemoryPoolId_t pool_id,
                              osThreadAttr_t *attr, void *args,
                              size_t name_size, size_t args_size, size_t cb_size, size_t stack_size,
                              size_t name_offset, size_t args_offset, size_t cb_offset, size_t stack_offset,
                              uint32_t timeout) {
    void *obj = (void *)osMemoryPoolAlloc(pool_id, timeout);
    if (obj == NULL) {
        return NULL;
    }
    char         *obj_name  = (char         *)((uint8_t *)obj + name_offset );
    void         *obj_args  = (void         *)((uint8_t *)obj + args_offset );
    StaticTask_t *obj_cb    = (StaticTask_t *)((uint8_t *)obj + cb_offset   );
    StackType_t  *obj_stack = (StackType_t  *)((uint8_t *)obj + stack_offset);

    osThreadAttr_t default_attr = { 0 };
    if (attr != NULL) {
        if (attr->name != NULL) {
            strncpy(obj_name, attr->name, configMAX_TASK_NAME_LEN);
            if (strlen(attr->name) >= configMAX_TASK_NAME_LEN) {
                obj_name[configMAX_TASK_NAME_LEN - 1] = '\0';
            }
        } else {
            obj_name[0] = '\0';
        }
    } else {
        attr = &default_attr;
    }

    attr->name       = obj_name;
    attr->cb_mem     = obj_cb;
    attr->cb_size    = cb_size;
    attr->stack_mem  = obj_stack;
    attr->stack_size = stack_size;
    
    if (args != NULL) {
        memcpy(obj_args, args, args_size);
    } else {
        memset(obj_args, 0, args_size);
    }

    osThreadId_t task = osThreadNew(func, obj_args, attr);

    if (task == NULL) {
        osMemoryPoolFree(pool_id, obj);
        return NULL;
    } else {
        TASK_POOL_OBJ_TABLE_add(&task_pool_obj_table, task, pool_id, obj);
    }

    return task;
}

void osThreadExit_Cstm() {
    osThreadId_t thread_id = osThreadGetId();
    if (thread_id != NULL) {
        osMessageQueuePut(msg_queue_cleanup_id, &thread_id, 0, 0);
        osThreadExit();
    }
}

TASK_POOL_ALLOCATE(TASK_Idle);

void TASK_Idle(void *argument) {
    for (;;) {
        osThreadId_t task_id;
        if (osMessageQueueGet(msg_queue_cleanup_id, &task_id, NULL, osWaitForever) == osOK) {
            if (task_id != NULL) {
                if (osThreadGetState(task_id) == osThreadTerminated) {
                    KEY_TASK_POOL_OBJ *key = TASK_POOL_OBJ_TABLE_find(&task_pool_obj_table, task_id);
                    if (key != NULL) {
                        osMemoryPoolFree(key->pool_id, key->obj);
                        TASK_POOL_OBJ_TABLE_remove(&task_pool_obj_table, key);
                    }
                } else {
                    // The task is not terminated yet, requeue it
                    osMessageQueuePut(msg_queue_cleanup_id, &task_id, 0, 0);
                }
            }
        }
    }
}


void TASK_POOL_OBJ_TABLE_init(TASK_POOL_OBJ_TABLE *table) {
    table->head = table->elems;
    table->count = 0;

    KEY_TASK_POOL_OBJ *last = table->elems;
    for (size_t i = 1; i < MAX_TASK; i++) {
        last->next = &table->elems[i];
        last = last->next;
    }
}

void TASK_POOL_OBJ_TABLE_add(TASK_POOL_OBJ_TABLE *table, osThreadId_t thread_id, osMemoryPoolId_t pool_id, void *obj) {
    if (table->count >= MAX_TASK) {
        return;
    }
    // KEY_TASK_POOL_OBJ *new_key = table->head;
    KEY_TASK_POOL_OBJ *new_key = NULL;
    for (size_t i = 0; i < MAX_TASK; i++) {
        if (table->elems[i].thread_id == NULL) {
            new_key = &(table->elems[i]);
            break;
        }
    }
    if (new_key == NULL) {
        return;
    }

    new_key->thread_id = thread_id;
    new_key->pool_id = pool_id;
    new_key->obj = obj;

    table->count++;
}

KEY_TASK_POOL_OBJ *TASK_POOL_OBJ_TABLE_find(TASK_POOL_OBJ_TABLE *table, osThreadId_t thread_id) {
    for (size_t i = 0; i < MAX_TASK; i++) {
        if ((table->elems)[i].pool_id == thread_id) {
            return table->elems + i;
        }
    }
    return NULL;
}

void TASK_POOL_OBJ_TABLE_remove(TASK_POOL_OBJ_TABLE *table, KEY_TASK_POOL_OBJ *key) {
    if (table->count == 0 || key == NULL) {
        return;
    }

    key->thread_id = 0;
    key->pool_id = 0;
    key->obj = NULL;

    // // Rebuild the free list
    // key->next = table->head;
    // table->head = key;

    table->count--;
}

void Init_cleanup(void) {

    osMessageQueueAttr_t msg_queue_cleanup_attr = {
        .name = "msg_queue_cleanup",
        .cb_mem = &msg_queue_cleanup_cb,
        .cb_size = sizeof(msg_queue_cleanup_cb),
        .mq_mem = msg_queue_cleanup_buffer,
        .mq_size = sizeof(msg_queue_cleanup_buffer),
    };

    msg_queue_cleanup_id = osMessageQueueNew(MAX_TASK, sizeof(osThreadId_t), &msg_queue_cleanup_attr);

    TASK_POOL_CREATE(TASK_Idle);

    TASK_POOL_OBJ_TABLE_init(&task_pool_obj_table);

    osThreadAttr_t attr = {
        .name = "TASK_Idle",
        .priority = (osPriority_t)osPriorityLow,
    };
    OS_THREAD_NEW_CSTM(TASK_Idle, (TASK_Idle_ARGS) {}, attr, osWaitForever);
}