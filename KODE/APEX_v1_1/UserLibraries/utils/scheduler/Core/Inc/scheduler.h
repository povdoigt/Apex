/**
 *******************************************************************************
 * @file    scheduler.h
 * @brief   Fully static task framework on top of CMSIS-RTOS 2 / FreeRTOS.
 *
 * @details
 * All task storage (TCB, stack, argument buffer) is reserved at link time.
 * No heap, no memory pool, no run-time registry. Set
 * `configSUPPORT_DYNAMIC_ALLOCATION 0` so that the linker enforces this.
 *
 * ## Lifecycle: parking, not create/destroy
 *
 * A task instance is created once, on its first spawn, and is then *parked*:
 * it blocks on a thread flag, runs its body when spawned, and returns to the
 * garage. Because the stack is statically reserved, destroying the task would
 * return no RAM; parking therefore has the same footprint with none of the
 * teardown machinery (no reaper task, no lookup table, no race on free).
 *
 * Creation is lazy: the first spawn of a given *slot* creates its thread. Use
 * task_prewarm() to pay that cost at boot instead of in flight.
 *
 * ## Three declaration sites
 *
 *   TASK_DECLARE   in a header  - the contract: argument type, stack size
 *   TASK_DEFINE    in a source  - the body, written as plain business logic
 *   TASK_POOL      in the app   - the RAM: how many instances exist
 *
 * A driver ships DECLARE + DEFINE (it knows what its code needs); the
 * application ships POOL (it owns the RAM budget). Every task pool in the
 * program is therefore visible in one auditable block.
 *
 * ## Return value and join
 *
 * A body has the signature `task_ret_t body(args_t *args)` and may return at
 * any point; the framework publishes the value to the destination given at
 * spawn time, then releases the slot. Synchronisation uses the joiner's own
 * thread flags: no kernel object, no RAM, one bit per outstanding job.
 *
 * @note All public functions must be called from thread context, after
 *       osKernelStart() (task_prewarm() excepted). None of them is ISR-safe:
 *       called from an ISR they refuse and do nothing.
 *******************************************************************************
 */

#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "task.h"

/* ========================================================================== */
/*                              Configuration                                 */
/* ========================================================================== */

/**
 * @brief Priority a slot holds while parked.
 *
 * Visible only during the short window between a job releasing its slot and
 * the runner re-entering the garage; the joiner has already been signalled by
 * then. It should sit below every working priority, otherwise an idle slot can
 * delay a periodic task during that window.
 */
#ifndef TASK_PARKED_PRIORITY
#define TASK_PARKED_PRIORITY    osPriorityLow
#endif

/**
 * @brief FreeRTOS thread-local-storage index used by the framework.
 *
 * Each thread that joins keeps there the set of its join bits still in flight,
 * so a bit cannot be reused while an earlier job on it may still signal it.
 * Requires `configNUM_THREAD_LOCAL_STORAGE_POINTERS > TASK_TLS_INDEX`.
 */
#ifndef TASK_TLS_INDEX
#define TASK_TLS_INDEX          0
#endif

/** @brief Upper bound on instances per pool (keeps the slot index one byte). */
#define TASK_MAX_INSTANCES      32u

/** @brief Smallest stack a body may declare, in bytes. */
#define TASK_MIN_STACK_BYTES    ((uint32_t)configMINIMAL_STACK_SIZE * sizeof(StackType_t))

/* ========================================================================== */
/*                              Thread flags                                  */
/* ========================================================================== */

/**
 * @brief Flag bit 0, reserved by the framework to release a parked slot.
 *
 * Join bits are numbered 1..30 and belong to the *joiner*, not to the job, so
 * a worker can itself join sub-jobs without colliding with its own garage bit.
 */
#define TASK_FLAG_START         (1u << 0)

/**
 * @brief Lowest / highest join bit a caller may claim.
 *
 * Bit 31 is excluded: the CMSIS-RTOS2 FreeRTOS wrapper rejects it
 * (MAX_BITS_TASK_NOTIFY), so a job on it could never signal its joiner.
 */
#define TASK_JOIN_BIT_MIN       1u
#define TASK_JOIN_BIT_MAX       30u

/** @brief Value of `task_attr_t::join_bit` meaning "detached, signal nobody". */
#define TASK_JOIN_NONE          0u

/* ========================================================================== */
/*                                  Types                                     */
/* ========================================================================== */

/**
 * @brief Value returned by a task body.
 *
 * Signed so driver status enums (W25Q_STATE, BMI_STATE, ...) round-trip
 * unchanged. The framework never inspects it.
 */
typedef int32_t task_ret_t;

/** @brief Descriptor flags, set by the TASK_DECLARE variant used. */
typedef enum {
    TASK_F_NONE       = 0u,
    /** Body never returns (acquisition loop, LED animation). Such a task may
     *  not be joined: task_spawn() rejects a non-zero join_bit on it. */
    TASK_F_PERSISTENT = (1u << 0),
} task_flags_t;

struct task_desc_t;

/**
 * @brief Per-instance mutable state. One per instance, in .bss.
 *
 * Zero-initialised, which is exactly the "free, never created" state: no boot
 * time initialisation is needed or performed.
 */
typedef struct task_slot_t {
    StaticTask_t              tcb;       /**< FreeRTOS control block.          */
    const struct task_desc_t *desc;      /**< Owning descriptor (set on spawn).*/
    osThreadId_t              id;        /**< NULL until the thread exists.    */
    osThreadId_t              joiner;    /**< Thread to signal on completion.  */
    task_ret_t               *ret;       /**< Where to publish the result.     */
    uint32_t                  gen;       /**< Bumped on release; stales handles*/
    uint8_t                   join_bit;  /**< 0 = detached, else 1..30.        */
    bool                      busy;      /**< Slot currently holds a job.      */
} task_slot_t;

/**
 * @brief Immutable, link-time description of a task. Lives in flash.
 *
 * Storage geometry is private to the translation unit that instantiates the
 * pool: no header ever mentions the instance count, so two units cannot
 * disagree about it.
 */
typedef struct task_desc_t {
    const char  *name;          /**< Thread name, for debuggers and traces.   */
    task_ret_t (*entry)(void *);/**< Type-erasing adapter around the body.    */
    task_slot_t *slots;         /**< Array of n_instances slots.              */
    StackType_t *stack_base;    /**< Base of the stack array.                 */
    uint8_t     *args_base;     /**< Base of the argument array.              */
    uint32_t     stack_bytes;   /**< Stack size per instance, in bytes.        */
    uint16_t     args_size;     /**< Bytes copied on spawn.                   */
    uint16_t     args_stride;   /**< Distance between argument buffers.       */
    uint8_t      n_instances;   /**< Pool depth.                              */
    uint8_t      flags;         /**< task_flags_t bitmask.                    */
} task_desc_t;

/**
 * @brief Spawn-time attributes.
 *
 * Designed so a compound literal with only the fields you care about is
 * correct: an unset `join_bit` is 0, which means detached.
 */
typedef struct task_attr_t {
    osPriority_t priority;  /**< Working priority for this run.               */
    task_ret_t  *ret;       /**< Destination for the return value, or NULL.
                                 Must stay valid until the job really ends,
                                 even past a task_join() timeout.              */
    uint8_t      join_bit;  /**< Joiner flag bit 1..30, or TASK_JOIN_NONE.    */
} task_attr_t;

/**
 * @brief Opaque reference to one running job.
 *
 * Carries the generation counter seen at spawn time, so a handle kept past the
 * end of its job cannot be mistaken for the next tenant of the same slot. The
 * joiner and join bit are carried here too, which lets task_join() work
 * without touching the slot at all.
 */
typedef struct task_h_t {
    task_slot_t *slot;
    osThreadId_t joiner;
    uint32_t     gen;
    uint8_t      join_bit;
} task_h_t;

/** @brief The handle returned when a spawn fails. */
#define TASK_H_INVALID          ((task_h_t){ .slot = NULL, .joiner = NULL, .gen = 0, .join_bit = 0 })

/* ========================================================================== */
/*                          Declaration macros                                */
/* ========================================================================== */

/** @cond INTERNAL */
/* Parameter names carry a trailing underscore: the preprocessor substitutes a
   parameter token even after a '.', so a parameter called `name` would corrupt
   the designated initialiser `.name` below. */
#if defined(__GNUC__)
#define TASK_MAYBE_UNUSED_      __attribute__((unused))
#else
#define TASK_MAYBE_UNUSED_
#endif

#define TASK_DECLARE_IMPL_(name_, args_type_, stackb_, flags_)                 \
    typedef args_type_ name_##_task_args_t;                                    \
    enum { name_##_TASK_STACK_BYTES = (uint32_t)(stackb_) };                   \
    enum { name_##_TASK_FLAGS       = (uint8_t)(flags_) };                     \
    extern const task_desc_t TASK_##name_;                                     \
    extern task_ret_t name_##_task_entry(void *raw_args_);                     \
    static inline task_h_t name_##_spawn(name_##_task_args_t *spawn_args_,     \
                                         const task_attr_t   *spawn_attr_) {   \
        return task_spawn_(&TASK_##name_, spawn_args_, spawn_attr_);           \
    }                                                                          \
    task_ret_t name_##_task_body(name_##_task_args_t *args TASK_MAYBE_UNUSED_)
/** @endcond */

/**
 * @brief Declare a task contract in a header.
 *
 * @param name        Task identifier. The descriptor is named TASK_<name>.
 * @param args_type   Already-declared struct type holding the parameters.
 * @param stack_bytes Stack per instance, in bytes; multiple of 4.
 *
 * The argument struct is declared separately, as an ordinary type, and holds
 * business inputs only: no result pointer, no done-flag handle.
 *
 * @code
 * typedef struct {
 *     W25Q_t   *chip;
 *     uint8_t  *buf;
 *     uint32_t  addr;
 *     uint32_t  len;
 * } W25Q_Write_args_t;
 *
 * TASK_DECLARE(W25Q_Write, W25Q_Write_args_t, 768);
 * @endcode
 */
#define TASK_DECLARE(name_, args_type_, stack_bytes_)                          \
    TASK_DECLARE_IMPL_(name_, args_type_, stack_bytes_, TASK_F_NONE)

/**
 * @brief Declare a task whose body never returns.
 *
 * Same as TASK_DECLARE but marks the task persistent: its slot stays busy for
 * the lifetime of the program, and task_spawn() refuses a join bit on it
 * (waiting for a task that never finishes would deadlock the joiner).
 */
#define TASK_DECLARE_PERSISTENT(name_, args_type_, stack_bytes_)               \
    TASK_DECLARE_IMPL_(name_, args_type_, stack_bytes_, TASK_F_PERSISTENT)

/**
 * @brief Define a task body in a source file.
 *
 * Inside the block, `args` is a correctly typed pointer to the private copy of
 * the arguments, and `return` is the only exit mechanism needed: the framework
 * publishes the value, restores the priority and releases the slot.
 *
 * @code
 * TASK_DEFINE(W25Q_Write) {
 *     if (args->chip == NULL) return W25Q_PARAM_ERR;
 *     return W25Q_WriteData(args->chip, args->buf, args->addr, args->len);
 * }
 * @endcode
 */
#define TASK_DEFINE(name_)                                                     \
    task_ret_t name_##_task_entry(void *raw_args_) {                           \
        return name_##_task_body((name_##_task_args_t *)raw_args_);            \
    }                                                                          \
    task_ret_t name_##_task_body(name_##_task_args_t *args TASK_MAYBE_UNUSED_)

/* ========================================================================== */
/*                          Instantiation macros                              */
/* ========================================================================== */

/**
 * @brief Instantiate a task pool, using the stack size from TASK_DECLARE.
 *
 * Purely declarative: it defines storage and a constant descriptor. Nothing
 * runs, nothing can fail at boot. Place every pool of the program in one block
 * so the whole static RAM budget is readable at a glance.
 *
 * @code
 * TASK_POOL(W25Q_Write,     2);
 * TASK_POOL(BMI088_ReadAcc, 1);
 * TASK_POOL(DetectLaunch,   1);
 * @endcode
 */
#define TASK_POOL(name_, n_)                                                   \
    TASK_POOL_SZ(name_, n_, name_##_TASK_STACK_BYTES)

/**
 * @brief Instantiate a task pool with an overridden stack size.
 *
 * Use when a measured high-water mark justifies departing from the size the
 * driver declared. @see task_stack_headroom().
 */
#define TASK_POOL_SZ(name_, n_, stackb_)                                       \
    _Static_assert((n_) >= 1 && (n_) <= TASK_MAX_INSTANCES,                    \
                   #name_ ": instance count out of range");                    \
    _Static_assert((stackb_) >= TASK_MIN_STACK_BYTES,                          \
                   #name_ ": stack below configMINIMAL_STACK_SIZE");           \
    _Static_assert((stackb_) % sizeof(StackType_t) == 0,                       \
                   #name_ ": stack size must be a multiple of the word size"); \
    static task_slot_t name_##_task_slots[(n_)];                               \
    static StackType_t name_##_task_stacks[(n_)]                               \
                                          [(stackb_) / sizeof(StackType_t)]    \
        __attribute__((aligned(8)));                                           \
    static union {                                                             \
        name_##_task_args_t args;                                              \
        uint64_t            align_;                                            \
    } name_##_task_argbuf[(n_)];                                               \
    _Static_assert(sizeof(name_##_task_argbuf[0]) <= UINT16_MAX,               \
                   #name_ ": argument struct larger than 65535 bytes");        \
    const task_desc_t TASK_##name_ = {                                         \
        .name        = #name_,                                                 \
        .entry       = name_##_task_entry,                                     \
        .slots       = name_##_task_slots,                                     \
        .stack_base  = &name_##_task_stacks[0][0],                             \
        .args_base   = (uint8_t *)name_##_task_argbuf,                         \
        .stack_bytes = (uint32_t)(stackb_),                                    \
        .args_size   = (uint16_t)sizeof(name_##_task_args_t),                  \
        .args_stride = (uint16_t)sizeof(name_##_task_argbuf[0]),               \
        .n_instances = (uint8_t)(n_),                                          \
        .flags       = (uint8_t)name_##_TASK_FLAGS,                            \
    }

/* ========================================================================== */
/*                                   API                                      */
/* ========================================================================== */

/**
 * @brief Start a job on a free instance of a task.
 *
 * TASK_DECLARE() generates one inline spawner per task, `<name>_spawn()`, so
 * the call site is an ordinary function call: arguments are type-checked by
 * the compiler, and compound literals may be written inline (a function-like
 * macro could not accept them - the preprocessor splits on commas inside
 * braces).
 *
 * Never blocks. Returns TASK_H_INVALID if the pool is exhausted, so flight
 * code must handle that path explicitly instead of waiting.
 *
 * `priority` has no default: osPriorityNone is rejected, which forces every
 * spawn to state the priority it runs at. That is deliberate - an implicit
 * priority is exactly what makes a rate-monotonic argument unverifiable.
 *
 * @code
 * W25Q_STATE st;
 * task_h_t h = W25Q_Write_spawn(
 *     &(W25Q_Write_args_t){ .chip = &w25q, .buf = data,
 *                           .addr = a, .len = sizeof(data) },
 *     &(task_attr_t){ .priority = osPriorityAboveNormal,
 *                     .ret = &st, .join_bit = 1 });
 * if (!task_h_valid(h)) { / * pool exhausted: degrade * / }
 * @endcode
 *
 * @param desc Task descriptor; `<name>_spawn()` supplies &TASK_<name>.
 * A join bit stays reserved for the calling thread until the job behind it
 * ends, whether or not it was joined: after a task_join() timeout, a new spawn
 * on the same bit is refused while the old job is still running.
 *
 * @param desc Task descriptor; `<name>_spawn()` supplies &TASK_<name>.
 * @param args Arguments, copied into the slot, so a caller-stack literal is
 *             safe. Must not be NULL when the argument struct is non-empty.
 * @param attr Spawn attributes.
 * @retval TASK_H_INVALID  Pool exhausted, thread creation failed, called from
 *                         an ISR or before osKernelStart(), NULL args, or
 *                         invalid attributes (priority outside
 *                         osPriorityIdle..configMAX_PRIORITIES-1, join bit
 *                         outside 1..30 or still in flight, join bit requested
 *                         on a persistent task).
 */
task_h_t task_spawn_(const task_desc_t *desc, void *args, const task_attr_t *attr);

/**
 * @brief Wait for one job to finish.
 *
 * Must be called by the thread that spawned it, since the flag being waited on
 * belongs to that thread. The return value of the body is written to
 * `attr->ret` before the slot is released, so it is readable as soon as this
 * call succeeds. A handle can be joined successfully only once: the flag is
 * consumed by the first successful join.
 *
 * On timeout the job is *not* cancelled and keeps running: stopping a task at
 * an arbitrary point would leave peripherals and buffers half-written. Treat
 * osErrorTimeout as a condition to degrade explicitly; `attr->ret` will still
 * be written when the job ends.
 *
 * @param h          Handle from task_spawn(), spawned with a non-zero join bit.
 * @param timeout_ms Milliseconds (0 polls), or osWaitForever.
 * @retval osOK             Job finished.
 * @retval osErrorTimeout   Still running.
 * @retval osErrorParameter Detached handle, or caller is not the joiner.
 * @retval osErrorISR       Called from an ISR.
 */
osStatus_t task_join(task_h_t h, uint32_t timeout_ms);

/**
 * @brief Wait for several jobs to finish.
 *
 * All handles must have been spawned by the calling thread on distinct join
 * bits. Waits for every one of them. On timeout no flag is consumed, so the
 * call can simply be repeated.
 *
 * @param h          Array of handles.
 * @param count      Number of handles.
 * @param timeout_ms Milliseconds (0 polls), or osWaitForever.
 * @retval osOK             All jobs finished.
 * @retval osErrorTimeout   At least one still running.
 * @retval osErrorParameter A detached handle, a handle spawned by another
 *                          thread, or a duplicated join bit.
 * @retval osErrorISR       Called from an ISR.
 */
osStatus_t task_join_all(const task_h_t *h, size_t count, uint32_t timeout_ms);

/**
 * @brief Test whether a spawn succeeded.
 * @note This says the handle was issued, not that the job is still running;
 *       use task_running() for that.
 */
bool task_h_valid(task_h_t h);

/**
 * @brief Report whether the job behind a handle is still running.
 *
 * Generation-checked, so a handle whose slot has since been reused by another
 * job reports false rather than reporting on the new tenant.
 */
bool task_running(task_h_t h);

/**
 * @brief Create every instance of a pool now, parked and idle.
 *
 * Optional. Moves the cost of first-time stack initialisation from the first
 * in-flight spawn to boot; call it for latency-critical pools only. Unlike the
 * rest of the API it may also be called before osKernelStart().
 *
 * @retval osOK       All instances exist, or are being created by a concurrent
 *                    spawn (already-created ones are left alone).
 * @retval osError    At least one thread could not be created.
 * @retval osErrorISR Called from an ISR.
 */
osStatus_t task_prewarm(const task_desc_t *desc);

/**
 * @brief Smallest number of free stack bytes ever observed on an instance.
 *
 * Wraps osThreadGetStackSpace(). Read it on the bench, at the end of a run, to
 * size TASK_POOL_SZ() from measurement rather than from guesswork.
 *
 * @param inst Instance index, 0 .. n_instances-1.
 * @return Free bytes, or 0 if the instance does not exist yet.
 */
uint32_t task_stack_headroom(const task_desc_t *desc, uint8_t inst);

/**
 * @brief Number of instances of a pool currently holding a job.
 *
 * Intended for a supervisor task: a pool permanently saturated is the symptom
 * that its instance count is under-sized.
 */
uint8_t task_busy_count(const task_desc_t *desc);

#endif /* SCHEDULER_H */
