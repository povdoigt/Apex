
#include <stdlib.h>
#include <string.h>

#include "utils/scheduler.h"


// ================== SCHEUDLER ==================


void SCHEDULER_init(SCHEDULER *scheduler) {
    OBJ_POOL_init((OBJ_POOL *)scheduler->task_pool, sizeof(TASK), MAX_TASKS_NBR);
    CIRCULAR_BUFFER_init(&scheduler->low_prio_queue, scheduler->low_prio_queue_buffer,
                         sizeof(TASK*), MAX_TASKS_NBR);
    CIRCULAR_BUFFER_init(&scheduler->high_prio_queue, scheduler->high_prio_queue_buffer,
                         sizeof(TASK*), MAX_TASKS_NBR);
    scheduler->low_prio_count = 0;
    scheduler->high_prio_count = 0;
}

TASK *SCHEDULER_add_task(SCHEDULER *scheduler, task_func_t func, bool high_prio, OBJ_POOL *context_pool) {
    TASK *task = (TASK *)OBJ_POOL_alloc((OBJ_POOL *)scheduler->task_pool);
    if (task) {
        task->func = (__task_func_t)func;
        task->time0 = HAL_GetTick();
        task->time1 = 0;
        task->high_prio = high_prio;
        task->context = OBJ_POOL_alloc(context_pool);
        task->context_pool = context_pool;
        task->is_done = NULL;

        if (!task->context) {
            // If context allocation failed, free the task and return NULL
            OBJ_POOL_free((OBJ_POOL *)scheduler->task_pool, task);
            return NULL;
        }

        if (high_prio) {
            CIRCULAR_BUFFER_push(&scheduler->high_prio_queue, &task);
            scheduler->high_prio_count++;
        } else {
            CIRCULAR_BUFFER_push(&scheduler->low_prio_queue, &task);
            scheduler->low_prio_count++;
        }
    }
    return task;
}

void SCHEDULER_kill_task(SCHEDULER *scheduler, TASK *task) {
    UNUSED(scheduler);
    // uint32_t time = HAL_GetTick() - task->time0; // debug purpose
    if (task->is_done) {
        *(task->is_done) = true;
    }
    OBJ_POOL_free(task->context_pool, task->context);
    OBJ_POOL_free((OBJ_POOL *)scheduler->task_pool, task);
}

void SCHEDULER_run_task(SCHEDULER *scheduler, TASK *task) {
    task->time1 = HAL_GetTick();
    TASK_RETURN task_ret = task->func(scheduler, task);
    switch (task_ret) {
    case TASK_RETURN_IDLE: {
        if (task->high_prio) {
            CIRCULAR_BUFFER_push(&scheduler->high_prio_queue, &task);
            scheduler->high_prio_count++;
        } else {
            CIRCULAR_BUFFER_push(&scheduler->low_prio_queue, &task);
            scheduler->low_prio_count++;
        }
        break; }
    case TASK_RETURN_STOP: {
        SCHEDULER_kill_task(scheduler, task);
        break; }
    }
}

void SCHEDULER_run(SCHEDULER *scheduler) {
    size_t high_prio_count = scheduler->high_prio_count;
    size_t low_prio_count = scheduler->low_prio_count;

    for (size_t i = 0; i < high_prio_count; i++) {
        TASK *task;
        CIRCULAR_BUFFER_pop(&scheduler->high_prio_queue, &task);
        if (task) {
            scheduler->high_prio_count--;
            SCHEDULER_run_task(scheduler, task);
        }
    }
    for (size_t i = 0; i < low_prio_count; i++) {
        TASK *task;
        CIRCULAR_BUFFER_pop(&scheduler->low_prio_queue, &task);
        if (task) {
            scheduler->low_prio_count--;
            SCHEDULER_run_task(scheduler, task);
        }
    }
}
