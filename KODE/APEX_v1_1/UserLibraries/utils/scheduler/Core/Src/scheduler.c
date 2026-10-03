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
 * 1. On spawn: clear the stale join flag, then publish the arguments, then
 *    raise the priority, and only then release the garage. Releasing first
 *    would let the body run against the previous tenant's priority.
 *
 * 2. On completion: publish the return value, snapshot the joiner, restore the
 *    parked priority, release the slot, and *then* signal. Nothing may touch
 *    the slot after release, because a concurrent spawn may already have
 *    handed it to another job — hence the snapshot.
 *
 * ## Concurrency model
 *
 * Slot allocation and release run under osKernelLock(), a few instructions
 * long. Thread creation deliberately happens outside that lock. Everything
 * else is per-thread state or thread flags, which the kernel makes atomic.
 *******************************************************************************
 */

#include <string.h>

#include "scheduler.h"

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
 * @brief Reserve the first free slot of a pool.
 *
 * Marks it busy under lock so two concurrent spawners cannot pick the same
 * one. The caller finishes populating it before starting the job.
 *
 * @return Reserved slot, or NULL if every instance is busy.
 */
static task_slot_t *slot_acquire(const task_desc_t *desc) {
    task_slot_t *found = NULL;
    int32_t      token = sched_lock();

    for (uint8_t i = 0; i < desc->n_instances; i++) {
        if (!desc->slots[i].busy) {
            found = &desc->slots[i];
            found->busy = true;
            break;
        }
    }

    sched_unlock(token);
    return found;
}

/**
 * @brief Hand a reserved slot back without having run anything.
 *
 * Used only on the spawn failure path. The generation is *not* bumped: no
 * handle was ever issued for this reservation.
 */
static void slot_abort(task_slot_t *s) {
    int32_t token = sched_lock();
    s->busy = false;
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

    /* Snapshot what we still need after the slot stops being ours. */
    osThreadId_t joiner   = s->joiner;
    uint8_t      join_bit = s->join_bit;

    /* Always restore, on every exit path, so the next tenant of this slot
       cannot inherit this job's priority. */
    (void)osThreadSetPriority(osThreadGetId(), TASK_PARKED_PRIORITY);

    int32_t token = sched_lock();
    s->gen++;
    s->ret      = NULL;
    s->joiner   = NULL;
    s->join_bit = TASK_JOIN_NONE;
    s->busy     = false;
    sched_unlock(token);

    /* From here on, `s` may already belong to another job: use the snapshot. */
    if (join_bit != TASK_JOIN_NONE && joiner != NULL) {
        (void)osThreadFlagsSet(joiner, 1u << join_bit);
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
    if (desc == NULL || attr == NULL) {
        return TASK_H_INVALID;
    }
    if (attr->priority < osPriorityIdle || attr->priority > osPriorityISR) {
        return TASK_H_INVALID;
    }
    if (attr->join_bit > TASK_JOIN_BIT_MAX) {
        return TASK_H_INVALID;
    }
    /* Joining a task that never returns would block the joiner forever. */
    if (attr->join_bit != TASK_JOIN_NONE && (desc->flags & TASK_F_PERSISTENT) != 0u) {
        return TASK_H_INVALID;
    }

    osThreadId_t self = osThreadGetId();
    if (attr->join_bit != TASK_JOIN_NONE && self == NULL) {
        return TASK_H_INVALID;   /* No thread context: nobody could join. */
    }

    task_slot_t *s = slot_acquire(desc);
    if (s == NULL) {
        return TASK_H_INVALID;   /* Pool exhausted. Caller must degrade. */
    }

    s->desc     = desc;
    s->ret      = attr->ret;
    s->joiner   = (attr->join_bit != TASK_JOIN_NONE) ? self : NULL;
    s->join_bit = attr->join_bit;

    /* Drop any flag left by an earlier job that nobody joined — e.g. one whose
       task_join() timed out. Without this, the next join on the same bit would
       return immediately and read a result that is not yet written. Only the
       calling thread's own flags can be cleared, which is exactly whose bit
       this is. */
    if (attr->join_bit != TASK_JOIN_NONE) {
        (void)osThreadFlagsClear(1u << attr->join_bit);
    }

    if (desc->args_size != 0u && args != NULL) {
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

    task_h_t h = { .slot = s, .gen = s->gen, .join_bit = attr->join_bit };

    /* Last: release the garage, once arguments and priority are in place. */
    (void)osThreadFlagsSet(s->id, TASK_FLAG_START);

    return h;
}

osStatus_t task_join(task_h_t h, uint32_t timeout_ms) {
    if (h.slot == NULL || h.join_bit == TASK_JOIN_NONE) {
        return osErrorParameter;
    }
    /* Thread flags are private to their owner: only the spawner can wait. */
    if (h.slot->joiner != NULL && h.slot->joiner != osThreadGetId()) {
        return osErrorParameter;
    }

    uint32_t flags = osThreadFlagsWait(1u << h.join_bit, osFlagsWaitAll, timeout_ms);

    if ((flags & osFlagsError) != 0u) {
        return (flags == (uint32_t)osFlagsErrorTimeout) ? osErrorTimeout : osError;
    }
    return osOK;
}

osStatus_t task_join_all(const task_h_t *h, size_t count, uint32_t timeout_ms) {
    if (h == NULL || count == 0u) {
        return osErrorParameter;
    }

    uint32_t mask = 0u;
    for (size_t i = 0; i < count; i++) {
        if (h[i].slot == NULL || h[i].join_bit == TASK_JOIN_NONE) {
            return osErrorParameter;
        }
        uint32_t bit = 1u << h[i].join_bit;
        if ((mask & bit) != 0u) {
            return osErrorParameter;   /* Two jobs sharing a bit are indistinct. */
        }
        mask |= bit;
    }

    uint32_t flags = osThreadFlagsWait(mask, osFlagsWaitAll, timeout_ms);

    if ((flags & osFlagsError) != 0u) {
        return (flags == (uint32_t)osFlagsErrorTimeout) ? osErrorTimeout : osError;
    }
    return osOK;
}

bool task_h_valid(task_h_t h) {
    return h.slot != NULL;
}

bool task_running(task_h_t h) {
    if (h.slot == NULL) {
        return false;
    }

    int32_t token   = sched_lock();
    bool    running = h.slot->busy && (h.slot->gen == h.gen);
    sched_unlock(token);

    return running;
}

osStatus_t task_prewarm(const task_desc_t *desc) {
    if (desc == NULL) {
        return osErrorParameter;
    }

    osStatus_t status = osOK;

    for (uint8_t i = 0; i < desc->n_instances; i++) {
        task_slot_t *s = &desc->slots[i];
        if (s->id == NULL) {
            /* Created parked and left free: the runner blocks on its garage
               flag, so the slot is still available to the next spawn. */
            if (slot_create(desc, s, TASK_PARKED_PRIORITY) != osOK) {
                status = osError;
            }
        }
    }

    return status;
}

uint32_t task_stack_headroom(const task_desc_t *desc, uint8_t inst) {
    if (desc == NULL || inst >= desc->n_instances) {
        return 0u;
    }
    osThreadId_t id = desc->slots[inst].id;
    return (id != NULL) ? osThreadGetStackSpace(id) : 0u;
}

uint8_t task_busy_count(const task_desc_t *desc) {
    if (desc == NULL) {
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
