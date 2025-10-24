#include "utils/data_topic.h"
#include <string.h>
#include <stdbool.h>

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

static inline void dt_lock(data_topic_t *dt) {
    osMutexAcquire(dt->cb.mutex_id, osWaitForever);
}

static inline void dt_unlock(data_topic_t *dt) {
    osMutexRelease(dt->cb.mutex_id);
}

/* --------------------------------------------------------------------------
 *   Topic : initialisation et publication
 * -------------------------------------------------------------------------- */

void data_topic_init(data_topic_t *topic,
                     void *storage, size_t elem_size, size_t capacity,
                     cb_overflow_policy_t policy) {
    if (!topic) return;

    cb_init(&(topic->cb), storage, elem_size, capacity, policy);
    dt_lock(topic);
    topic->pub_seq = 0u;
    topic->sub_count = 0u;
    topic->subs = NULL;
    dt_unlock(topic);
}

data_status_t data_topic_publish(data_topic_t *topic, const void *elem) {
    if (!topic || !elem) return DT_BAD_ARG;

    cb_status_t s = cb_push(&(topic->cb), elem);
    if (s == CB_FULL) return DT_FULL;

    /* Publication validée */
    topic->pub_seq++;
    return DT_OK;
}

/* --------------------------------------------------------------------------
 *   Subscriber : attachement / détachement / synchronisation
 * -------------------------------------------------------------------------- */

data_status_t data_sub_attach(data_sub_t *sub,
                              data_topic_t *topic,
                              data_attach_mode_t mode) {
    if (!sub || !topic) return DT_BAD_ARG;
    if (sub->attached) return DT_OK; /* déjà attaché */

    sub->topic = topic;
    sub->attached = 1;
    sub->last_seq = topic->pub_seq;

    if (mode == DATA_ATTACH_FROM_OLDEST) {
        sub->tail = topic->cb.tail;
        /* Ajuste last_seq pour correspondre à la plus ancienne donnée encore présente */
        sub->last_seq -= topic->cb.count;
    } else {
        sub->tail = topic->cb.head;
    }

    dt_lock(topic);

    sub->prev = NULL;
    sub->next = NULL;

    data_sub_t *old_head = topic->subs;
    if (old_head != NULL) {
        old_head->prev = sub;
        sub->next = old_head;
    }
    topic->subs = sub;

    topic->sub_count++;
    dt_unlock(topic);

    return DT_OK;
}

data_status_t data_sub_detach(data_sub_t *sub) {
    if (!sub || !sub->attached || !sub->topic) return DT_BAD_ARG;

    data_topic_t *topic = sub->topic;

    dt_lock(topic);

    /* Retire de la liste chainée */

    if (sub->next != NULL) {
        sub->next->prev = sub->prev;
    }
    if (sub->prev != NULL) {
        sub->prev->next = sub->next;
    } else {
        topic->subs = sub->next;
    }

    if (topic->sub_count > 0u) {
        topic->sub_count--;
    }
    dt_unlock(topic);

    sub->attached = 0;
    sub->topic = NULL;
    sub->tail = 0u;
    sub->last_seq = 0u;

    return DT_OK;
}

data_status_t data_sub_sync(data_sub_t *sub) {
    if (!sub || !sub->attached || !sub->topic) return DT_BAD_ARG;

    data_topic_t *topic = sub->topic;
    sub->tail = topic->cb.head;
    sub->last_seq = topic->pub_seq;

    return DT_OK;
}

/* --------------------------------------------------------------------------
 *   Subscriber : logique commune (paramétrable)
 * -------------------------------------------------------------------------- */

uint32_t data_sub_num_to_read(const data_sub_t *sub) {
    if (!sub || !sub->attached || !sub->topic) return 0u;

    uint32_t delta = sub->topic->pub_seq - sub->last_seq;
    return delta;
}

data_status_t data_sub_peek_relative_ptr(data_sub_t *sub, const void **out_ptr, size_t origin, int offset) {
    if (!sub || !sub->attached) return DT_BAD_ARG;
    if (!out_ptr) return DT_BAD_ARG;

    if (sub->topic->cb.count == 0u) return DT_EMPTY;

    uint32_t delta = data_sub_num_to_read(sub);
    if (delta == 0) return DT_EMPTY;

    data_status_t result = DT_OK;

    /* Overflow : plus de messages publiés que la capacité ne peut en garder */
    if (delta > sub->topic->cb.capacity) {
        data_sub_sync(sub);
        result = DT_DATA_LOSS;
    }

    const void *src = cb_peek_relative_ptr(&(sub->topic->cb), origin, offset);
    if (!src) return DT_BAD_ARG;
    
    *out_ptr = src;

    return result;
}

data_status_t data_sub_peek_ptr(data_sub_t *sub, const void **out_ptr, int idx) {
    return data_sub_peek_relative_ptr(sub, out_ptr, 0, idx);
}

data_status_t data_sub_read_ptr(data_sub_t *sub, const void **out_ptr) {
    data_status_t status = data_sub_peek_relative_ptr(sub, out_ptr, sub->tail, 0);
    if (status != DT_BAD_ARG && status != DT_EMPTY) {
        // Avance la position de l’abonné
        sub->tail = wrap_add(sub->tail, 1, sub->topic->cb.capacity);
        sub->last_seq++;
    }
    return status;
}

data_status_t data_sub_peek_relative(data_sub_t *sub, void *out_elem, size_t origin, int offset) {
    if (!out_elem) return DT_BAD_ARG;

    const void *src_ptr = NULL;
    data_status_t status = data_sub_peek_relative_ptr(sub, &src_ptr, origin, offset);
    if (status != DT_BAD_ARG && status != DT_EMPTY) {
        memcpy(out_elem, src_ptr, sub->topic->cb.elem_size);
    }
    return status;
}

data_status_t data_sub_peek(data_sub_t *sub, void *out_elem, int idx) {
    return data_sub_peek_relative(sub, out_elem, 0, idx);
}

data_status_t data_sub_read(data_sub_t *sub, void *out_elem) {
    if (!out_elem) return DT_BAD_ARG;

    const void *src_ptr = NULL;
    data_status_t status = data_sub_read_ptr(sub, &src_ptr);
    if (status != DT_BAD_ARG && status != DT_EMPTY) {
        memcpy(out_elem, src_ptr, sub->topic->cb.elem_size);
    }
    return status;
}
