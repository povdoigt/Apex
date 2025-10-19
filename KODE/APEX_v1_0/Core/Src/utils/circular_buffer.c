#include "utils/circular_buffer.h"
#include <string.h>

/* --------------------------------------------------------------------------
 *   Fonctions internes (non exportées)
 * -------------------------------------------------------------------------- */

// Effectue une addition modulo avec un offset pouvant être négatif.
// Permet de gérer les index circulaires.
static inline size_t wrap_add(size_t base, int offset, size_t mod) {
    int result = (int)base + offset;
    int wrapped = (result % (int)mod + (int)mod) % (int)mod;
    return (size_t)wrapped;
}

/* --------------------------------------------------------------------------
 *   Initialisation / reset
 * -------------------------------------------------------------------------- */

void cb_init(circular_buffer_t *cb,
             void *storage, size_t elem_size, size_t capacity,
             cb_overflow_policy_t policy) {
    if (!cb || !storage || elem_size == 0u || capacity == 0u) return;

    cb->storage = (uint8_t *)storage;
    cb->elem_size = elem_size;
    cb->capacity  = capacity;
    cb->head = 0u;
    cb->tail = 0u;
    cb->count = 0u;
    cb->policy = policy;
}

void cb_reset(circular_buffer_t *cb) {
    cb->head = 0u;
    cb->tail = 0u;
    cb->count = 0u;
}

/* --------------------------------------------------------------------------
 *   Opérations principales
 * -------------------------------------------------------------------------- */

cb_status_t cb_push(circular_buffer_t *cb, const void *elem) {
    if (!cb || !elem) return CB_BAD_ARG;

    cb_status_t status = CB_OK;
    /* Cas plein */
    if (cb->count == cb->capacity) {
        if (cb->policy == CB_REJECT_NEW) {
            return CB_FULL;
        }
        /* Overwrite oldest: on avance le tail */
        cb->tail = wrap_add(cb->tail, 1, cb->capacity);
        status = CB_OVERWROTE_OLDEST;
        /* count reste saturé */
    } else {
        cb->count++;
    }

    /* Copie de l’élément au head */
    uint8_t *dst = cb->storage + (cb->head * cb->elem_size);
    memcpy(dst, elem, cb->elem_size);
    cb->head = wrap_add(cb->head, 1, cb->capacity);

    return status;
}

cb_status_t cb_pop(circular_buffer_t *cb, void *out) {
    if (!cb)             return CB_BAD_ARG;
    if (cb->count == 0u) return CB_EMPTY;
    if (!out)            return CB_BAD_ARG;

    const uint8_t *src = cb->storage + (cb->tail * cb->elem_size);
    memcpy(out, src, cb->elem_size);

    cb->tail = wrap_add(cb->tail, 1, cb->capacity);
    cb->count--;
    return CB_OK;
}

/* --------------------------------------------------------------------------
 *   Accès pointeur (bas niveau, sans copie)
 * -------------------------------------------------------------------------- */

/**
 * @brief Retourne un pointeur constant vers un élément à un index absolu (wrap permissif).
 */
const void *cb_peek_ptr(const circular_buffer_t *cb, size_t idx) {
    if (!cb || !cb->storage) return NULL;

    /* wrap permissif : idx peut être supérieur à capacity */
    size_t index = idx % cb->capacity;
    return cb->storage + (index * cb->elem_size);
}

/**
 * @brief Retourne un pointeur constant vers un élément relatif à une origine.
 * @param origin Index de base (souvent cb->tail)
 * @param offset Décalage relatif (positif ou négatif, wrap automatique)
 */
const void *cb_peek_relative_ptr(const circular_buffer_t *cb,
                                 size_t origin, int offset) {
    if (!cb || !cb->storage) return NULL;

    int idx = (int)origin + offset;
    int mod = (idx % (int)cb->capacity + (int)cb->capacity) % (int)cb->capacity;

    return cb->storage + ((size_t)mod * cb->elem_size);
}

/* --------------------------------------------------------------------------
 *   Accès lecture (haut niveau, avec copie)
 * -------------------------------------------------------------------------- */

cb_status_t cb_peek(const circular_buffer_t *cb, size_t idx, void *out) {
    if (!cb || !out) return CB_BAD_ARG;

    const void *src = cb_peek_ptr(cb, idx);
    if (!src) return CB_BAD_ARG;

    memcpy(out, src, cb->elem_size);
    return CB_OK;
}

cb_status_t cb_peek_relative(const circular_buffer_t *cb,
                             size_t origin, int offset, void *out) {
    if (!cb || !out) return CB_BAD_ARG;

    const void *src = cb_peek_relative_ptr(cb, origin, offset);
    if (!src) return CB_BAD_ARG;

    memcpy(out, src, cb->elem_size);
    return CB_OK;
}
