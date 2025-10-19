#ifndef DATA_TOPIC_H
#define DATA_TOPIC_H

#include "utils/circular_buffer.h"

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 *   Types et constantes
 * -------------------------------------------------------------------------- */

typedef enum {
    DATA_ATTACH_FROM_NOW = 0,
    DATA_ATTACH_FROM_OLDEST
} data_attach_mode_t;

typedef enum {
    DT_OK = 0,
    DT_EMPTY,
    DT_DATA_LOSS,
    DT_FULL,
    DT_BAD_ARG
} data_status_t;

/* --------------------------------------------------------------------------
 *   Structures principales
 * -------------------------------------------------------------------------- */

/**
 * @brief Topic de données : encapsule un buffer circulaire et son état logique.
 */
typedef struct {
    circular_buffer_t ring;     /**< Buffer associé (avec mémoire externe). */
    uint32_t          pub_seq;  /**< Compteur global de publications. */
    size_t            subscriber_count; /**< Nombre d’abonnés actuellement attachés. */
} data_topic_t;

/**
 * @brief Abonné à un topic : position locale et référence vers le topic.
 */
typedef struct {
    data_topic_t *topic;
    size_t        tail;
    uint32_t      last_seq;
    int           attached;
} data_subscriber_t;

/* --------------------------------------------------------------------------
 *   API Topic
 * -------------------------------------------------------------------------- */

void data_topic_init(data_topic_t *topic,
                     void *storage, size_t elem_size, size_t capacity,
                     cb_overflow_policy_t policy);

data_status_t data_topic_publish(data_topic_t *topic, const void *elem);

/* --------------------------------------------------------------------------
 *   API Subscriber
 * -------------------------------------------------------------------------- */

data_status_t data_subscriber_attach(data_subscriber_t *sub,
                                     data_topic_t *topic,
                                     data_attach_mode_t mode);

/**
 * @brief Donne le nombre de nouvelles données non lues par l’abonné.
 *
 * @param sub Abonné à interroger.
 * @return Nombre de nouvelles données disponibles (0 si aucun ou erreur).
 */
uint32_t data_subscriber_num_new(const data_subscriber_t *sub);

data_status_t data_subscriber_read(data_subscriber_t *sub, void *out_elem);

/**
 * @brief Détache proprement un abonné du topic.
 *
 * @param sub Abonné à détacher.
 * @return DT_OK si succès, DT_BAD_ARG sinon.
 */
data_status_t data_subscriber_detach(data_subscriber_t *sub);

#ifdef __cplusplus
}
#endif

#endif /* DATA_TOPIC_H */
