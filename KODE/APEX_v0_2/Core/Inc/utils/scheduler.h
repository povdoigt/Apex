#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <stdbool.h>

#include "stm32f4xx_hal.h"

#include "utils/obj_pool.h"
#include "utils/circular_buffer.h"


#define MAX_TASKS_NBR   30

typedef enum TASK_RETURN {
    TASK_RETURN_IDLE,
    TASK_RETURN_STOP
} TASK_RETURN;


// fonction a executer (avec le scheduler et le contexte)
typedef TASK_RETURN(*__task_func_t)(void*, void*);

// Contient les informations necessaires pour executer une tache
// (fonction, index, etat) sans prendre en compte le contexte
typedef struct TASK {
    __task_func_t    func;       // fonction a executer (avec le scheduler et sa tache référente)
    uint32_t         time0;      // temps de debut de la tache
    uint32_t         time1;      // temps de debut de dernier cycle
    bool             high_prio;  // si la tache est haute priorite ou non
    uint8_t         *context;  // contexte de la tache
    OBJ_POOL        *context_pool;  // pool de contexte de la tache
    bool            *is_done;    // pointeur vers un booléen pour transmettre l'etat de la tache
} TASK;



// Contient toutes les informations du scheduler
typedef struct SCHEDULER {
    uint8_t task_pool[OBJ_POOL_SIZE(sizeof(TASK), MAX_TASKS_NBR)];
    uint8_t low_prio_queue_buffer[CIRCULAR_BUFFER_SIZE(sizeof(TASK*), MAX_TASKS_NBR)];
    uint8_t high_prio_queue_buffer[CIRCULAR_BUFFER_SIZE(sizeof(TASK*), MAX_TASKS_NBR)];
    CIRCULAR_BUFFER low_prio_queue;
    CIRCULAR_BUFFER high_prio_queue;
    size_t low_prio_count;
    size_t high_prio_count;
} SCHEDULER;

// Le vrai type de fonction a executer (avec le scheduler et le contexte)
typedef TASK_RETURN(*task_func_t)(SCHEDULER*, TASK*);


#define TASK_POOL_CREATE(task_name) \
    uint8_t task_name##_POOL[OBJ_POOL_SIZE(sizeof(task_name##_CONTEXT), task_name##_NUMBER)]


#define TASK_POOL_init(task_name) \
    OBJ_POOL_init((OBJ_POOL *)task_name##_POOL, sizeof(task_name##_CONTEXT), task_name##_NUMBER)

// ================== SCHEUDLER ==================



void SCHEDULER_init(SCHEDULER *scheduler);

TASK *SCHEDULER_add_task(SCHEDULER *scheduler, task_func_t func, bool high_prio, OBJ_POOL *context_pool);

void SCHEDULER_kill_task(SCHEDULER *scheduler, TASK *task);

void SCHEDULER_run_task(SCHEDULER *scheduler, TASK *task);

void SCHEDULER_run(SCHEDULER *scheduler);


#define SCHEDULER_add_task_macro(scheduler, task_name, high_prio) \
    SCHEDULER_add_task(scheduler, task_name, high_prio, (OBJ_POOL *)task_name##_POOL)

#endif