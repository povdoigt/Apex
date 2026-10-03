/**
 *******************************************************************************
 * @file    scheduler.c
 * @brief   Implementation of the static task framework. @see scheduler.h
 *
 * @details
 * ## Where the ordering matters
 *
 * Two sequences in this file are load-bearing and are the reason the framework
 * exists at all rather than being open-coded in every task body.
 *
 * 1. On spawn: reserve the slot and the join bit, clearing any stale flag on
 *    that bit, in one locked section; then publish the arguments, then raise
 *    the priority, and only then release the garage. Releasing first would let
 *    the body run against the previous tenant's priority.
 *
 * 2. On completion: publish the return value, then, in one locked section,
 *    release the slot and the join bit, signal the joiner, and drop to the
 *    parked priority. Signalling *before* dropping keeps a high-priority
 *    joiner from waiting behind every medium-priority thread; doing it all
 *    under the lock keeps a concurrent spawn from seeing the slot free while
 *    its priority is still about to be lowered.
 *
 * ## Concurrency model
 *
 * Slot and join-bit bookkeeping run under osKernelLock() (scheduler
 * suspended, interrupts still enabled), and only non-blocking kernel calls are
 * made inside it. Thread creation deliberately happens outside that lock.
 *
 * ## Why joins do not use osThreadFlagsWait()
 *
 * In the CMSIS-RTOS2 wrapper shipped with FreeRTOS 10.3.1, osThreadFlagsWait()
 * returns only when a notification is *pending*. Bits latched by a
 * notification that an earlier wait already consumed are ignored: with two
 * jobs finished before the first join, the second join would block with its
 * bit set. It also reports a zero timeout as osErrorResource, and with
 * osFlagsWaitAll it discards the bits it saw when it times out. join_wait()
 * therefore reads the latched bits before sleeping, and clears them only on
 * success.
 *******************************************************************************
 */

#include <string.h>

#include "scheduler.h"

#if (configNUM_THREAD_LOCAL_STORAGE_POINTERS <= TASK_TLS_INDEX)
#error "scheduler: configNUM_THREAD_LOCAL_STORAGE_POINTERS must exceed TASK_TLS_INDEX"
#endif

/* ========================================================================== */
/*                            Internal helpers                                */
/* ========================================================================== */

/**
 * @brief Enter a short critical section, tolerating a not-yet-running kernel.
 * @return Token to hand back to sched_unlock().
 */
static int32_t sched_lock(void) {
    return osKernelLock();
}

/**
 * @brief Leave the critical section opened by sched_lock().
 * @param token Value returned by sched_lock(); negative values are errors and
 *              mean no lock was taken, so nothing must be restored.
 */
static void sched_unlock(int32_t token) {
    if (token >= 0) {
        (void)osKernelRestoreLock(token);
    }
}

/**
 * @brief Check that the caller may use the API.
 * @retval osOK       Thread context, kernel started.
 * @retval osErrorISR Called from an interrupt.
 * @retval osError    Kernel not started yet.
 */
static osStatus_t sched_context(void) {
    if (xPortIsInsideInterrupt() != pdFALSE) {
        return osErrorISR;
    }
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
        return osError;
    }
    return osOK;
}

/**
 * @brief Join bits of a thread still owed a signal by a running job.
 *
 * Kept in the thread's own TLS slot, as a plain bitmask cast to a pointer, so
 * it costs no RAM and needs no registry. Call under sched_lock().
 */
static uint32_t inflight_get(osThreadId_t thread) {
    return (uint32_t)(uintptr_t)pvTaskGetThreadLocalStoragePointer(
        (TaskHandle_t)thread, TASK_TLS_INDEX);
}

/** @brief Store a thread's in-flight join bits. Call under sched_lock(). */
static void inflight_put(osThreadId_t thread, uint32_t bits) {
    vTaskSetThreadLocalStoragePointer((TaskHandle_t)thread, TASK_TLS_INDEX,
                                      (void *)(uintptr_t)bits);
}

/** @brief Index of a slot within its pool. */
static inline uint8_t slot_index(const task_desc_t *desc, const task_slot_t *s) {
    return (uint8_t)(s - desc->slots);
}

/** @brief Argument buffer belonging to a slot. */
static inline void *slot_args(const task_desc_t *desc, const task_slot_t *s) {
    return desc->args_base + (size_t)slot_index(desc, s) * desc->args_stride;
}

/** @brief Stack belonging to a slot. */
static inline StackType_t *slot_stack(const task_desc_t *desc, const task_slot_t *s) {
    return desc->stack_base
         + (size_t)slot_index(desc, s) * (desc->stack_bytes / sizeof(StackType_t));
}

/**
 * @brief Reserve the first free slot of a pool, and the caller's join bit.
 *
 * Both reservations happen under one lock, so two concurrent spawners cannot
 * pick the same slot, and a join bit still owed a signal by an earlier job
 * cannot be handed out again. The caller finishes populating the slot before
 * starting the job.
 *
 * @param joiner   Calling thread.
 * @param join_bit Bit to reserve, or TASK_JOIN_NONE.
 * @return Reserved slot, or NULL if every instance is busy or the bit is in
 *         flight.
 */
static task_slot_t *slot_acquire(const task_desc_t *desc, osThreadId_t joiner,
                                 uint8_t join_bit) {
    task_slot_t *found = NULL;
    uint32_t     bit   = (join_bit != TASK_JOIN_NONE) ? (1u << join_bit) : 0u;
    int32_t      token = sched_lock();

    if (bit == 0u || (inflight_get(joiner) & bit) == 0u) {
        for (uint8_t i = 0; i < desc->n_instances; i++) {
            if (!desc->slots[i].busy) {
                found = &desc->slots[i];
                break;
            }
        }
    }

    if (found != NULL) {
        found->busy     = true;
        found->joiner   = (bit != 0u) ? joiner : NULL;
        found->join_bit = join_bit;

        if (bit != 0u) {
            inflight_put(joiner, inflight_get(joiner) | bit);
            /* Drop the flag a finished-but-never-joined job may have left, so
               the next join on this bit waits for the new job. Under the lock
               because osThreadFlagsClear() is a read-modify-write. */
            (void)osThreadFlagsClear(bit);
        }
    }

    sched_unlock(token);
    return found;
}

/**
 * @brief Hand a reserved slot back without having run anything.
 *
 * Used on the spawn failure path and by task_prewarm(). The generation is
 * *not* bumped: no handle was ever issued for this reservation.
 */
static void slot_abort(task_slot_t *s) {
    int32_t token = sched_lock();

    if (s->join_bit != TASK_JOIN_NONE) {
        inflight_put(s->joiner, inflight_get(s->joiner) & ~(1u << s->join_bit));
    }
    s->ret      = NULL;
    s->joiner   = NULL;
    s->join_bit = TASK_JOIN_NONE;
    s->busy     = false;

    sched_unlock(token);
}

/**
 * @brief Release a slot after its job returned, and signal the joiner.
 *
 * Bumping the generation is what makes stale handles detectable: a handle
 * issued for the previous tenant no longer matches.
 */
static void slot_finish(task_slot_t *s, task_ret_t result) {
    /* Publish before releasing: once the slot is free, another job may own the
       destination pointer. */
    if (s->ret != NULL) {
        *s->ret = result;
    }

    int32_t token = sched_lock();

    osThreadId_t joiner   = s->joiner;
    uint8_t      join_bit = s->join_bit;

    s->gen++;
    s->ret      = NULL;
    s->joiner   = NULL;
    s->join_bit = TASK_JOIN_NONE;
    s->busy     = false;

    if (join_bit != TASK_JOIN_NONE && joiner != NULL) {
        uint32_t bit = 1u << join_bit;
        inflight_put(joiner, inflight_get(joiner) & ~bit);
        (void)osThreadFlagsSet(joiner, bit);
    }

    /* Signal first, then drop: the joiner is already ready when this thread
       loses its working priority. Inside the lock, the drop cannot be
       preempted by a spawn that has just raised it for the next tenant;
       the context switch it may request is deferred to the unlock. */
    (void)osThreadSetPriority(osThreadGetId(), TASK_PARKED_PRIORITY);

    sched_unlock(token);
}

/**
 * @brief Convert a millisecond timeout to ticks, keeping osWaitForever.
 *
 * Computed in 64 bits: pdMS_TO_TICKS() overflows past ~71 min at 1 kHz.
 */
static TickType_t ms_to_ticks(uint32_t ms) {
    if (ms == osWaitForever) {
        return portMAX_DELAY;
    }
    uint64_t ticks = ((uint64_t)ms * configTICK_RATE_HZ) / 1000u;
    return (ticks >= portMAX_DELAY) ? (portMAX_DELAY - 1u) : (TickType_t)ticks;
}

/**
 * @brief Wait until every bit of `mask` is set in the caller's flags.
 *
 * Bits are cleared on success only, so a timeout consumes nothing and the
 * wait can be repeated. @see the file header for why osThreadFlagsWait() is
 * not used.
 *
 * @retval osOK           All bits were set; they are now cleared.
 * @retval osErrorTimeout At least one bit still missing after the timeout.
 */
static osStatus_t join_wait(uint32_t mask, uint32_t timeout_ms) {
    const TickType_t timeout = ms_to_ticks(timeout_ms);
    const TickType_t start   = xTaskGetTickCount();

    for (;;) {
        uint32_t value = 0u;

        /* Zero-tick wait: never blocks, always reports the latched bits. */
        (void)xTaskNotifyWait(0u, 0u, &value, 0u);

        if ((value & mask) == mask) {
            int32_t token = sched_lock();
            (void)osThreadFlagsClear(mask);
            sched_unlock(token);
            return osOK;
        }

        TickType_t remaining = portMAX_DELAY;
        if (timeout != portMAX_DELAY) {
            TickType_t elapsed = xTaskGetTickCount() - start;
            if (elapsed >= timeout) {
                return osErrorTimeout;
            }
            remaining = timeout - elapsed;
        }

        /* Sleep until the next notification. One that arrived since the check
           above is still pending, so this returns at once: none is lost. */
        (void)xTaskNotifyWait(0u, 0u, &value, remaining);
    }
}

/* ========================================================================== */
/*                                 Runner                                     */
/* ========================================================================== */

/**
 * @brief Thread entry shared by every task instance.
 *
 * Reconciles the kernel's contract (`void f(void *)`, never returns) with the
 * body's contract (typed arguments, may return), and owns the two ordering
 * sequences documented at the top of this file. The slot pointer arrives as
 * the thread argument and lives in this frame for the life of the thread,
 * which is why no registry or thread-local lookup is needed.
 */
static void task_runner(void *slot_ptr) {
    task_slot_t *s = (task_slot_t *)slot_ptr;

    for (;;) {
        /* Garage. A spawn may have set the flag before we got here; thread
           flags latch, so no wake-up can be missed. */
        uint32_t flags = osThreadFlagsWait(TASK_FLAG_START, osFlagsWaitAll, osWaitForever);

        if ((flags & osFlagsError) != 0u) {
            /* Cannot happen with osWaitForever, but never spin on an error. */
            (void)osDelay(1);
            continue;
        }

        const task_desc_t *desc = s->desc;
        task_ret_t result = desc->entry(slot_args(desc, s));

        /* A persistent body never reaches this point; if one ever returns we
           still release the slot rather than fall off the end of the thread. */
        slot_finish(s, result);
    }
}

/**
 * @brief Create the thread backing a slot, parked and not yet started.
 *
 * @param prio Priority to create at. The runner blocks immediately, so this
 *             only matters for the instant before the garage is reached.
 * @return osOK, or osError if the kernel refused the creation.
 */
static osStatus_t slot_create(const task_desc_t *desc, task_slot_t *s, osPriority_t prio) {
    const osThreadAttr_t attr = {
        .name       = desc->name,
        .cb_mem     = &s->tcb,
        .cb_size    = sizeof(s->tcb),
        .stack_mem  = slot_stack(desc, s),
        .stack_size = desc->stack_bytes,   /* CMSIS takes BYTES, not words. */
        .priority   = prio,
    };

    s->desc = desc;
    s->id   = osThreadNew(task_runner, s, &attr);

    return (s->id != NULL) ? osOK : osError;
}

/* ========================================================================== */
/*                                  API                                       */
/* ========================================================================== */

task_h_t task_spawn_(const task_desc_t *desc, void *args, const task_attr_t *attr) {
    if (sched_context() != osOK) {
        return TASK_H_INVALID;
    }
    if (desc == NULL || attr == NULL) {
        return TASK_H_INVALID;
    }
    /* Without arguments the body would run on the previous tenant's ones. */
    if (desc->args_size != 0u && args == NULL) {
        return TASK_H_INVALID;
    }
    /* osPriorityISR equals configMAX_PRIORITIES here: vTaskPrioritySet()
       would trip its configASSERT on it. */
    if (attr->priority < osPriorityIdle
        || (uint32_t)attr->priority >= (uint32_t)configMAX_PRIORITIES) {
        return TASK_H_INVALID;
    }
    if (attr->join_bit > TASK_JOIN_BIT_MAX) {
        return TASK_H_INVALID;
    }
    /* Joining a task that never returns would block the joiner forever. */
    if (attr->join_bit != TASK_JOIN_NONE && (desc->flags & TASK_F_PERSISTENT) != 0u) {
        return TASK_H_INVALID;
    }

    task_slot_t *s = slot_acquire(desc, osThreadGetId(), attr->join_bit);
    if (s == NULL) {
        return TASK_H_INVALID;   /* Pool exhausted or bit in flight: degrade. */
    }

    s->desc = desc;
    s->ret  = attr->ret;

    if (desc->args_size != 0u) {
        memcpy(slot_args(desc, s), args, desc->args_size);
    }

    /* Lazy creation: the first spawn of this slot builds its thread. Storage is
       already reserved, so this only initialises the stack frame. */
    if (s->id == NULL) {
        if (slot_create(desc, s, attr->priority) != osOK) {
            slot_abort(s);
            return TASK_H_INVALID;
        }
    } else {
        (void)osThreadSetPriority(s->id, attr->priority);
    }

    task_h_t h = {
        .slot     = s,
        .joiner   = s->joiner,
        .gen      = s->gen,
        .join_bit = attr->join_bit,
    };

    /* Last: release the garage, once arguments and priority are in place. */
    (void)osThreadFlagsSet(s->id, TASK_FLAG_START);

    return h;
}

osStatus_t task_join(task_h_t h, uint32_t timeout_ms) {
    osStatus_t ctx = sched_context();
    if (ctx != osOK) {
        return ctx;
    }
    if (h.slot == NULL || h.join_bit == TASK_JOIN_NONE || h.join_bit > TASK_JOIN_BIT_MAX) {
        return osErrorParameter;
    }
    /* Thread flags are private to their owner: only the spawner can wait. The
       joiner comes from the handle, never from the slot, which may already
       belong to another job. */
    if (h.joiner != osThreadGetId()) {
        return osErrorParameter;
    }

    return join_wait(1u << h.join_bit, timeout_ms);
}

osStatus_t task_join_all(const task_h_t *h, size_t count, uint32_t timeout_ms) {
    osStatus_t ctx = sched_context();
    if (ctx != osOK) {
        return ctx;
    }
    if (h == NULL || count == 0u) {
        return osErrorParameter;
    }

    osThreadId_t self = osThreadGetId();
    uint32_t     mask = 0u;

    for (size_t i = 0; i < count; i++) {
        if (h[i].slot == NULL || h[i].join_bit == TASK_JOIN_NONE
            || h[i].join_bit > TASK_JOIN_BIT_MAX || h[i].joiner != self) {
            return osErrorParameter;
        }
        uint32_t bit = 1u << h[i].join_bit;
        if ((mask & bit) != 0u) {
            return osErrorParameter;   /* Two jobs sharing a bit are indistinct. */
        }
        mask |= bit;
    }

    return join_wait(mask, timeout_ms);
}

bool task_h_valid(task_h_t h) {
    return h.slot != NULL;
}

bool task_running(task_h_t h) {
    if (h.slot == NULL || sched_context() != osOK) {
        return false;
    }

    int32_t token   = sched_lock();
    bool    running = h.slot->busy && (h.slot->gen == h.gen);
    sched_unlock(token);

    return running;
}

osStatus_t task_prewarm(const task_desc_t *desc) {
    if (xPortIsInsideInterrupt() != pdFALSE) {
        return osErrorISR;
    }
    if (desc == NULL) {
        return osErrorParameter;
    }

    osStatus_t status = osOK;

    for (uint8_t i = 0; i < desc->n_instances; i++) {
        task_slot_t *s    = &desc->slots[i];
        bool         mine = false;

        /* Reserve before creating, so a concurrent spawn cannot pick and
           create the same slot meanwhile. A slot already busy is either
           created or being created by its spawner. */
        int32_t token = sched_lock();
        if (!s->busy && s->id == NULL) {
            s->busy = true;
            mine    = true;
        }
        sched_unlock(token);

        if (mine) {
            /* Created parked: the runner blocks on its garage flag, so the
               slot is free again for the next spawn once handed back. */
            if (slot_create(desc, s, TASK_PARKED_PRIORITY) != osOK) {
                status = osError;
            }
            slot_abort(s);
        }
    }

    return status;
}

uint32_t task_stack_headroom(const task_desc_t *desc, uint8_t inst) {
    if (desc == NULL || inst >= desc->n_instances || xPortIsInsideInterrupt() != pdFALSE) {
        return 0u;
    }
    osThreadId_t id = desc->slots[inst].id;
    return (id != NULL) ? osThreadGetStackSpace(id) : 0u;
}

uint8_t task_busy_count(const task_desc_t *desc) {
    if (desc == NULL || xPortIsInsideInterrupt() != pdFALSE) {
        return 0u;
    }

    uint8_t count = 0u;
    int32_t token = sched_lock();

    for (uint8_t i = 0; i < desc->n_instances; i++) {
        if (desc->slots[i].busy) {
            count++;
        }
    }

    sched_unlock(token);
    return count;
}
