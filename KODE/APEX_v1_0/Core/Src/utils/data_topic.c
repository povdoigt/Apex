#include "utils/data_topic.h"
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
 *   Topic : initialisation et publication
 * -------------------------------------------------------------------------- */

void data_topic_init(data_topic_t *topic,
                     void *storage, size_t elem_size, size_t capacity,
                     cb_overflow_policy_t policy) {
    if (!topic) return;

    cb_init(&topic->ring, storage, elem_size, capacity, policy);
    topic->pub_seq = 0u;
    topic->subscriber_count = 0u;
}

data_status_t data_topic_publish(data_topic_t *topic, const void *elem) {
    if (!topic || !elem) return DT_BAD_ARG;

    cb_status_t s = cb_push(&topic->ring, elem);
    if (s == CB_FULL) return DT_FULL;

    /* Publication validée */
    topic->pub_seq++;
    return DT_OK;
}

/* --------------------------------------------------------------------------
 *   Subscriber : attachement / détachement
 * -------------------------------------------------------------------------- */

data_status_t data_subscriber_attach(data_subscriber_t *sub,
                                     data_topic_t *topic,
                                     data_attach_mode_t mode) {
    if (!sub || !topic) return DT_BAD_ARG;
    if (sub->attached) return DT_OK; /* déjà attaché */

    sub->topic = topic;
    sub->attached = 1;
    sub->last_seq = topic->pub_seq;

    if (mode == DATA_ATTACH_FROM_OLDEST) {
        sub->tail = topic->ring.tail;
        /* Ajuste last_seq pour correspondre à la plus ancienne donnée encore présente */
        sub->last_seq -= topic->ring.count;
    } else {
        sub->tail = topic->ring.head;
    }

    topic->subscriber_count++;
    return DT_OK;
}

data_status_t data_subscriber_detach(data_subscriber_t *sub) {
    if (!sub || !sub->attached || !sub->topic) return DT_BAD_ARG;

    data_topic_t *topic = sub->topic;
    if (topic->subscriber_count > 0u) {
        topic->subscriber_count--;
    }

    sub->attached = 0;
    sub->topic = NULL;
    sub->tail = 0u;
    sub->last_seq = 0u;

    return DT_OK;
}

/* --------------------------------------------------------------------------
 *   Subscriber : lecture et détection de nouveautés
 * -------------------------------------------------------------------------- */

uint32_t data_subscriber_num_new(const data_subscriber_t *sub) {
    if (!sub || !sub->attached) return 0;

    // Calcul du delta entre la dernière séquence lue et la séquence de publication actuelle
    return sub->topic->pub_seq - sub->last_seq;
}

data_status_t data_subscriber_read(data_subscriber_t *sub, void *out_elem){
    if (!sub || !out_elem || !sub->attached) return DT_BAD_ARG;

    data_topic_t *topic = sub->topic;
    circular_buffer_t *cb = &(topic->ring);

    if (cb->count == 0u) return DT_EMPTY;

    uint32_t delta = data_subscriber_num_new(sub);
    if (delta == 0) return DT_EMPTY; /* aucune nouvelle donnée */

    data_status_t result = DT_OK;

    /* Si delta > capacity → overflow, on réaligne le subscriber */
    if (delta > cb->capacity) {
        sub->tail = cb->tail;
        sub->last_seq = topic->pub_seq - cb->count;
        result = DT_DATA_LOSS;
    }

    /* Lecture de la donnée pointée par tail (copie unique) */
    const void *src = cb_peek_relative_ptr(cb, sub->tail, 0);
    if (!src) return DT_BAD_ARG;

    memcpy(out_elem, src, cb->elem_size);

    /* Avancement logique */
    sub->tail = wrap_add(sub->tail, 1, cb->capacity);
    sub->last_seq++;

    return result;
}
