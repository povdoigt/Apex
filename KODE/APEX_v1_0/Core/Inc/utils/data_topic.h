#ifndef DATA_TOPIC_H
#define DATA_TOPIC_H

#include "utils/circular_buffer.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 *   Types et constantes
 * -------------------------------------------------------------------------- */

/**
 * @brief Mode d’attachement d’un abonné à un topic.
 */
typedef enum {
    DATA_ATTACH_FROM_NOW = 0,   /**< L’abonné commencera à lire les publications futures. */
    DATA_ATTACH_FROM_OLDEST     /**< L’abonné commencera depuis la donnée la plus ancienne encore disponible. */
} data_attach_mode_t;

/**
 * @brief Codes de statut utilisés par le module data_topic.
 */
typedef enum {
    DT_OK = 0,          /**< Opération réussie. */
    DT_EMPTY,           /**< Aucune donnée disponible à la lecture. */
    DT_DATA_LOSS,       /**< L’abonné a été dépassé : des données ont été perdues. */
    DT_FULL,            /**< Le topic a refusé une publication (mode REJECT_NEW). */
    DT_BAD_ARG          /**< Paramètre invalide. */
} data_status_t;

/* --------------------------------------------------------------------------
 *   Structures principales
 * -------------------------------------------------------------------------- */

/**
 * @brief Représente un topic de données.
 *
 * Le topic encapsule un circular_buffer existant et gère la séquence
 * des publications ainsi que le suivi du nombre d’abonnés.
 * Il ne possède pas de mémoire propre pour les données.
 */
typedef struct {
    circular_buffer_t cb;      /**< Buffer circulaire associé. */
    uint32_t          pub_seq; /**< Compteur global de publications. */
    size_t            subscriber_count; /**< Nombre d’abonnés actuellement attachés. */
} data_topic_t;

/**
 * @brief Représente un abonné à un topic.
 *
 * Chaque abonné conserve sa propre position de lecture dans le buffer.
 * Les lectures sont non destructives pour les autres abonnés.
 */
typedef struct {
    data_topic_t *topic;  /**< Référence vers le topic associé. */
    size_t        tail;   /**< Position locale de lecture (index dans le buffer). */
    uint32_t      last_seq; /**< Dernière séquence lue (comparée à pub_seq). */
    int           attached; /**< 0 = détaché, 1 = attaché. */
} data_subscriber_t;

/* --------------------------------------------------------------------------
 *   API Topic
 * -------------------------------------------------------------------------- */

/**
 * @brief Initialise un topic à partir d’un buffer circulaire existant.
 *
 * @param topic     Structure du topic à initialiser.
 * @param storage   Mémoire externe du buffer.
 * @param elem_size Taille d’un élément (en octets).
 * @param capacity  Nombre maximal d’éléments.
 * @param policy    Politique de dépassement (voir cb_overflow_policy_t).
 */
void data_topic_init(data_topic_t *topic,
                     void *storage, size_t elem_size, size_t capacity,
                     cb_overflow_policy_t policy);

/**
 * @brief Publie une nouvelle donnée dans le topic.
 *
 * Copie la donnée dans le buffer et incrémente le compteur de publication.
 * @return DT_OK, DT_FULL ou DT_BAD_ARG selon le cas.
 */
data_status_t data_topic_publish(data_topic_t *topic, const void *elem);

/* --------------------------------------------------------------------------
 *   API Subscriber : attachement / détachement / synchronisation
 * -------------------------------------------------------------------------- */

/**
 * @brief Attache un abonné à un topic.
 *
 * @param sub   Abonné à initialiser.
 * @param topic Topic auquel s’attacher.
 * @param mode  Mode d’attache (FROM_NOW ou FROM_OLDEST).
 */
data_status_t data_sub_attach(data_subscriber_t *sub,
                              data_topic_t *topic,
                              data_attach_mode_t mode);

/**
 * @brief Détache un abonné du topic.
 */
data_status_t data_sub_detach(data_subscriber_t *sub);

/**
 * @brief Réaligne un abonné sur la tête du topic.
 *
 * Utilisé en cas de perte de données ou de resynchronisation volontaire.
 */
data_status_t data_sub_sync(data_subscriber_t *sub);

/* --------------------------------------------------------------------------
 *   API Subscriber : logique commune et peek (bas niveau)
 * -------------------------------------------------------------------------- */

/**
 * @brief Retourne le nombre de publications non lues par un abonné.
 */
uint32_t data_sub_num_to_read(const data_subscriber_t *sub);

/**
 * @brief Lecture non destructive (pointeur direct) depuis un index relatif.
 *
 * @param sub     Abonné concerné.
 * @param out_ptr Pointeur de sortie vers la donnée (zéro copie).
 * @param origin  Index de base à partir duquel s’applique l’offset.
 * @param offset  Décalage relatif (peut être négatif, wrap permissif).
 *
 * @return DT_OK, DT_EMPTY, DT_DATA_LOSS ou DT_BAD_ARG.
 */
data_status_t data_sub_peek_relative_ptr(data_subscriber_t *sub,
                                         const void **out_ptr,
                                         size_t origin, int offset);

/**
 * @brief Lecture non destructive (pointeur direct) à partir d’un index absolu.
 *
 * @param sub     Abonné concerné.
 * @param out_ptr Pointeur de sortie vers la donnée (zéro copie).
 * @param idx     Index absolu (origine 0, wrap permissif).
 */
data_status_t data_sub_peek_ptr(data_subscriber_t *sub,
                                const void **out_ptr, int idx);

/* --------------------------------------------------------------------------
 *   API Subscriber : lecture "ptr" et "copy"
 * -------------------------------------------------------------------------- */

/**
 * @brief Lecture destructive : avance la position locale.
 *
 * @param sub     Abonné concerné.
 * @param out_ptr Pointeur de sortie vers la donnée interne.
 */
data_status_t data_sub_read_ptr(data_subscriber_t *sub, const void **out_ptr);

/**
 * @brief Lecture non destructive avec copie (relative).
 *
 * @param sub     Abonné concerné.
 * @param out_elem Buffer de sortie.
 * @param origin  Index de base.
 * @param offset  Décalage relatif.
 */
data_status_t data_sub_peek_relative(data_subscriber_t *sub,
                                     void *out_elem,
                                     size_t origin, int offset);

/**
 * @brief Lecture non destructive avec copie (absolue).
 *
 * @param sub     Abonné concerné.
 * @param out_elem Buffer de sortie.
 * @param idx     Index absolu (origine 0).
 */
data_status_t data_sub_peek(data_subscriber_t *sub, void *out_elem, int idx);

/**
 * @brief Lecture destructive avec copie locale.
 *
 * @param sub     Abonné concerné.
 * @param out_elem Buffer de sortie.
 */
data_status_t data_sub_read(data_subscriber_t *sub, void *out_elem);

#ifdef __cplusplus
}
#endif

#endif /* DATA_TOPIC_H */
