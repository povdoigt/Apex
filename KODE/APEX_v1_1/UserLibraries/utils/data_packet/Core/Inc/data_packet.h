#ifndef DATA_PACKET_H
#define DATA_PACKET_H

#include "data_topic.h"
#include "main_config.h"

#include <stdint.h>
#include <stddef.h>

#if (APEX_CFG_SCHED_RTOS == 1)
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 *   Modèle
 *
 *   Un packer regroupe dans un même paquet horodaté la donnée de chacun de
 *   ses topics sources la plus proche d'un instant t (fenêtre de T ms
 *   centrée sur t), puis publie ce paquet sur son propre topic.
 *
 *   Concurrence
 *   - Les sources sont lues avec copie (data_sub_peek / data_sub_read) :
 *     leurs publieurs (tâches plus prioritaires, interruptions) peuvent
 *     tourner pendant la construction sans jamais livrer d'échantillon
 *     déchiré.
 *   - Le paquet est construit dans `staging`, puis publié par copie : un
 *     abonné du packer ne voit jamais un paquet à moitié écrit, même quand
 *     il est en retard d'exactement `capacity` paquets.
 *   - data_packer_build_publish est appelé par une seule tâche (ou la seule
 *     boucle principale) : le packer et ses abonnés lui appartiennent.
 * -------------------------------------------------------------------------- */

#define DATA_PACKET_MAX_TOPICS 32   /* flags : un bit par topic source */

/** Plus grand élément accepté d'un topic source (horodatage compris). */
#ifndef DATA_PACKET_MAX_ELEM_SIZE
#define DATA_PACKET_MAX_ELEM_SIZE    64u
#endif

/** Plus grand paquet accepté (horodatage, flags et charges utiles). */
#ifndef DATA_PACKET_MAX_PACKET_SIZE
#define DATA_PACKET_MAX_PACKET_SIZE  256u
#endif

typedef struct data_ts_generic_t {
    uint32_t ts; // Timestamp in milliseconds (HAL)
    uint8_t data[]; // Data payload (variable length)
} data_ts_generic_t;

typedef struct data_ts_packet_generic_t {
    uint32_t ts; // Timestamp in milliseconds (HAL)
    uint32_t flags; // Flags for validity of each data field (bit 0 = topic 0, bit 1 = topic 1, etc.)
    uint8_t data[]; // Data payload (variable length)
} data_ts_packet_generic_t;

typedef struct data_packer_t {
    uint32_t T; // Window time size in milliseconds
    size_t num_topics; // Number of topics
    size_t packet_size; // Bytes of one packet (elem_size of `topic`)
    data_topic_t topic; // Publisher for the packed data
    data_sub_t subs[DATA_PACKET_MAX_TOPICS];
    size_t payload_size[DATA_PACKET_MAX_TOPICS]; // Bytes of payload per topic (elem_size - timestamp)
    /* Paquet en construction, publié par copie (voir "Concurrence"). */
    uint32_t staging[(DATA_PACKET_MAX_PACKET_SIZE + 3u) / 4u];
    /* Copie de l'élément source examiné. */
    uint32_t elem[(DATA_PACKET_MAX_ELEM_SIZE + 3u) / 4u];
} data_packer_t;

typedef enum data_packer_status_t{
    PACKER_EMPTY     = 0,  // Data topic is empty, no data to read
    PACKER_TOO_YOUNG = 1,  // Data is too young, outside the window
    PACKER_VALID     = 2,  // Data is valid and within the window
} data_packer_status_t;

/**
 * @brief Initialise un packer : attache un abonné FROM_NOW à chaque topic
 *        source et initialise le topic des paquets sur `storage`.
 *
 * @param packer      Packer à zéro (statique ou `= {0}`) ou libéré par
 *                    data_packer_free : un abonné encore attaché est refusé.
 * @param window_ms   Largeur T de la fenêtre, centrée sur l'instant du paquet.
 * @param num_topics  1..DATA_PACKET_MAX_TOPICS.
 * @param topics      Topics sources initialisés ; chaque élément commence par
 *                    un horodatage uint32_t (data_ts_generic_t) et fait au
 *                    plus DATA_PACKET_MAX_ELEM_SIZE octets.
 * @param cb_capacity Capacité du topic des paquets.
 * @param storage     cb_capacity * data_packer_packet_size(...) octets.
 * @return DT_OK, ou DT_BAD_ARG (argument invalide, élément ou paquet trop
 *         grand, attache refusée) : rien n'est alors laissé attaché.
 */
data_status_t data_packer_init(data_packer_t *packer, uint32_t window_ms, size_t num_topics,
                               data_topic_t **topics, size_t cb_capacity, void *storage);

/**
 * @brief Taille d'un paquet pour ces topics sources : à multiplier par la
 *        capacité pour dimensionner `storage`. 0 si un topic est invalide.
 */
size_t data_packer_packet_size(data_topic_t **topics, size_t num_topics);

/**
 * @brief Détache les abonnés du packer et libère son topic. Le packer peut
 *        ensuite être réinitialisé. Réservé à la tâche propriétaire.
 */
void data_packer_free(data_packer_t *packer);

/**
 * @brief Construit le paquet de l'instant `current_time_ms` et le publie.
 *
 * Durée bornée : au plus `capacity` échantillons trop vieux sont rejetés par
 * source et par appel, même si leur publieur en produit plus vite.
 *
 * @return Les flags du paquet publié (bit i : topic i présent), 0 si le
 *         packer n'est pas initialisé.
 */
uint32_t data_packer_build_publish(data_packer_t *packer, uint32_t current_time_ms);

#ifdef __cplusplus
}
#endif

#endif /* DATA_PACKET_H */
