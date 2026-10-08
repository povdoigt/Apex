#include "circular_buffer.h"
#include <string.h>

/* --------------------------------------------------------------------------
 *   Fonctions internes (non exportées)
 * -------------------------------------------------------------------------- */

// Index suivant, avec retour à 0 en fin de stockage.
static inline size_t cb_next(size_t idx, size_t capacity) {
    return (idx + 1u == capacity) ? 0u : idx + 1u;
}

// (origin + offset) modulo capacity, sans débordement : origin peut dépasser
// capacity et offset être négatif ("wrap permissif").
static inline size_t cb_wrap(size_t origin, int offset, size_t capacity) {
    size_t base = origin % capacity;
    size_t step;
    if (offset >= 0) {
        step = (size_t)offset % capacity;
    } else {
        /* -(offset + 1) + 1 évite de calculer -INT_MIN */
        size_t back = ((size_t)(-(offset + 1)) + 1u) % capacity;
        step = (back == 0u) ? 0u : capacity - back;
    }
    /* base < capacity et step < capacity */
    return (base >= capacity - step) ? base - (capacity - step) : base + step;
}

static inline uint8_t *cb_slot(const circular_buffer_t *cb, size_t idx) {
    return cb->storage + (idx * cb->elem_size);
}

/* --------------------------------------------------------------------------
 *   Initialisation / reset
 * -------------------------------------------------------------------------- */

cb_status_t cb_init(circular_buffer_t *cb,
                    void *storage, size_t elem_size, size_t capacity,
                    cb_overflow_policy_t policy) {
    if (!cb || !storage || elem_size == 0u || capacity == 0u) return CB_BAD_ARG;
    if (policy != CB_OVERWRITE_OLDEST && policy != CB_REJECT_NEW) return CB_BAD_ARG;
    /* elem_size * capacity doit tenir dans un size_t : sinon cb_slot déborde. */
    if (capacity > SIZE_MAX / elem_size) return CB_BAD_ARG;

    cb->storage = (uint8_t *)storage;
    cb->elem_size = elem_size;
    cb->capacity  = capacity;
    cb->head = 0u;
    cb->tail = 0u;
    cb->count = 0u;
    cb->policy = policy;

    return CB_OK;
}

cb_status_t cb_reset(circular_buffer_t *cb) {
    if (!cb) return CB_BAD_ARG;
    cb_critical_t c = cb_critical_enter();
    cb->head = 0u;
    cb->tail = 0u;
    cb->count = 0u;
    cb_critical_exit(c);
    return CB_OK;
}

cb_status_t cb_free(circular_buffer_t *cb) {
    if (!cb) return CB_BAD_ARG;
    return CB_OK;
}

/* --------------------------------------------------------------------------
 *   Opérations principales
 * -------------------------------------------------------------------------- */

cb_status_t cb_push(circular_buffer_t *cb, const void *elem) {
    if (!cb || !elem || !cb->storage) return CB_BAD_ARG;

    cb_status_t status = CB_OK;
    cb_critical_t c = cb_critical_enter();

    /* Cas plein */
    if (cb->count == cb->capacity) {
        if (cb->policy == CB_REJECT_NEW) {
            cb_critical_exit(c);
            return CB_FULL;
        }
        /* Overwrite oldest: on avance le tail, count reste saturé */
        cb->tail = cb_next(cb->tail, cb->capacity);
        status = CB_OVERWROTE_OLDEST;
    } else {
        cb->count++;
    }

    /* Copie de l’élément au head */
    uint8_t *dst = cb_slot(cb, cb->head);
    if (dst != (const uint8_t *)elem) { /* donnée déjà en place : memcpy sur elle-même interdit */
        memcpy(dst, elem, cb->elem_size);
    }
    cb->head = cb_next(cb->head, cb->capacity);

    cb_critical_exit(c);
    return status;
}

cb_status_t cb_pop(circular_buffer_t *cb, void *out) {
    if (!cb || !out || !cb->storage) return CB_BAD_ARG;

    cb_critical_t c = cb_critical_enter();

    /* count testé sous la section critique : deux consommateurs ne peuvent
       pas retirer tous les deux le dernier élément. */
    if (cb->count == 0u) {
        cb_critical_exit(c);
        return CB_EMPTY;
    }

    memcpy(out, cb_slot(cb, cb->tail), cb->elem_size);
    cb->tail = cb_next(cb->tail, cb->capacity);
    cb->count--;

    cb_critical_exit(c);
    return CB_OK;
}

size_t cb_count(const circular_buffer_t *cb) {
    if (!cb) return 0u;
    return cb->count;   /* lecture d'un mot, atomique sur Cortex-M */
}

/* --------------------------------------------------------------------------
 *   Accès pointeur (bas niveau, sans copie)
 * -------------------------------------------------------------------------- */

const void *cb_peek_relative_ptr(circular_buffer_t *cb,
                                 size_t origin, int offset) {
    if (!cb || !cb->storage || cb->capacity == 0u) return NULL;
    return cb_slot(cb, cb_wrap(origin, offset, cb->capacity));
}

const void *cb_peek_ptr(circular_buffer_t *cb, size_t idx) {
    if (!cb || !cb->storage || cb->capacity == 0u) return NULL;
    return cb_slot(cb, idx % cb->capacity);
}

/* --------------------------------------------------------------------------
 *   Accès lecture (haut niveau, avec copie)
 * -------------------------------------------------------------------------- */

cb_status_t cb_peek_relative(circular_buffer_t *cb,
                             size_t origin, int offset, void *out) {
    if (!cb || !out || !cb->storage || cb->capacity == 0u) return CB_BAD_ARG;

    cb_critical_t c = cb_critical_enter();
    memcpy(out, cb_slot(cb, cb_wrap(origin, offset, cb->capacity)), cb->elem_size);
    cb_critical_exit(c);
    return CB_OK;
}

cb_status_t cb_peek(circular_buffer_t *cb, size_t idx, void *out) {
    if (!cb || !out || !cb->storage || cb->capacity == 0u) return CB_BAD_ARG;

    cb_critical_t c = cb_critical_enter();
    memcpy(out, cb_slot(cb, idx % cb->capacity), cb->elem_size);
    cb_critical_exit(c);
    return CB_OK;
}
