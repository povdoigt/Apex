#ifndef DATA_TOPIC_H
#define DATA_TOPIC_H

#include "circular_buffer.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"

#if (APEX_CFG_SCHED_RTOS == 1)
#include "FreeRTOS.h"
#include "semphr.h"
#endif

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 *   Modèle
 *
 *   Un topic est un historique circulaire des `capacity` dernières
 *   publications. Chaque abonné a son propre curseur : une lecture n'enlève
 *   rien aux autres abonnés, et le publieur n'attend jamais personne. Un
 *   abonné trop lent pour suivre est dépassé : les éléments écrasés avant
 *   qu'il les lise sont perdus pour lui, et sa lecture suivante renvoie
 *   DT_DATA_LOSS (avec la plus ancienne donnée encore présente).
 *
 *   Concurrence (RTOS et séquentiel)
 *   - Tout l'état partagé (stockage, pub_seq, liste des abonnés, curseur et
 *     copie d'une lecture) est protégé par la section critique courte du
 *     circular_buffer (voir cb_critical_enter). Une publication est donc vue
 *     entièrement ou pas du tout, et une copie n'est jamais déchirée.
 *   - data_topic_publish est utilisable depuis une interruption (priorité
 *     NVIC numériquement >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY).
 *   - Les lectures avec copie (read, peek) sont utilisables depuis une
 *     interruption ; attach, detach, sync et wait sont réservés aux tâches.
 *   - Un abonné appartient à une seule tâche : deux tâches ne lisent pas sur
 *     le même data_sub_t.
 *   - Durée de vie : le topic garde l'adresse de chaque abonné attaché. Un
 *     data_sub_t doit être détaché (data_sub_detach) avant que sa mémoire
 *     ne serve à autre chose : variable locale d'une tâche, détacher avant
 *     que le corps ne rende la main. Un abonné oublié dans la liste est lu à
 *     chaque publication. Filets, qui ne remplacent pas le detach :
 *     data_sub_attach refuse un abonné encore chaîné (la liste ne se
 *     referme pas sur elle-même) ; sous RTOS, un abonné dont la mémoire a
 *     été réécrite est reconnu (son sémaphore n'est plus le sien), la
 *     notification s'arrête sur lui au lieu de suivre des pointeurs
 *     quelconques, detach et free n'écrivent jamais dedans, et
 *     topic->list_faults s'incrémente. Les abonnés situés au-delà ne sont
 *     alors plus réveillés (leurs données restent lisibles).
 *   - Les accès "_ptr" (zéro copie) rendent une adresse dans le stockage du
 *     topic. Son contenu reste valide tant que le publieur n'a pas fait le
 *     tour du buffer, soit au moins (capacity - num_to_read) publications.
 *     À réserver au cas où le publieur ne peut pas tourner pendant la
 *     lecture (même tâche, ou build séquentiel sans publication en ISR).
 *
 *   Notification (RTOS)
 *   Chaque abonné a un sémaphore binaire statique, créé à l'attache. Le
 *   publieur écrit la donnée et incrémente pub_seq, puis libère les
 *   sémaphores : un abonné réveillé trouve toujours la donnée déjà là.
 *   data_sub_wait_for_data revérifie num_to_read à chaque réveil, un jeton
 *   resté dans le sémaphore ne la fait donc pas rendre la main à vide.
 *   Avant osKernelStart(), init, attach et publish sont permis (depuis
 *   setup_pre_kernel() par exemple) : la publication ne notifie personne,
 *   aucune tâche ne pouvant attendre, et le premier wait la voit.
 * -------------------------------------------------------------------------- */

/* --------------------------------------------------------------------------
 *   Types et constantes
 * -------------------------------------------------------------------------- */

/**
 * @brief Position de départ d’un abonné (attache) ou de recalage (sync).
 */
typedef enum {
    DATA_ATTACH_FROM_NOW = 0,   /**< Ne lire que les publications futures. */
    DATA_ATTACH_FROM_OLDEST     /**< Repartir de la plus ancienne donnée encore disponible. */
} data_attach_mode_t;

/**
 * @brief Codes de statut utilisés par le module data_topic.
 */
typedef enum {
    DT_OK = 0,          /**< Opération réussie. */
    DT_EMPTY,           /**< Aucune donnée disponible à la lecture. */
    DT_DATA_LOSS,       /**< Donnée lue, mais l’abonné avait été dépassé : des données ont été perdues avant elle. */
    DT_BAD_ARG          /**< Paramètre invalide, topic non initialisé ou abonné non attaché. */
} data_status_t;

/* --------------------------------------------------------------------------
 *   Structures principales
 * -------------------------------------------------------------------------- */

/**
 * @brief Représente un topic de données.
 *
 * Le topic encapsule un circular_buffer existant et gère la séquence
 * des publications ainsi que le suivi des abonnés.
 * Il ne possède pas de mémoire propre pour les données.
 *
 * Champs internes : `pub_seq` et `sub_count` peuvent être lus (un mot,
 * lecture atomique), le reste ne doit pas être modifié hors de l’API.
 */
typedef struct data_topic_t {
    circular_buffer_t    cb;        /**< Buffer circulaire associé. */
    uint32_t             pub_seq;   /**< Compteur global de publications. */
    size_t               sub_count; /**< Nombre d’abonnés actuellement attachés. */
    struct data_sub_t   *subs;      /**< Liste chainée des abonnés. */
    uint32_t             list_faults; /**< Abonnés incohérents rencontrés dans la liste (voir "Durée de vie") : 0 en usage normal, à remonter en télémétrie. */
} data_topic_t;

/**
 * @brief Représente un abonné à un topic.
 *
 * Chaque abonné conserve sa propre position de lecture dans le buffer.
 * Les lectures sont non destructives pour les autres abonnés.
 * Un abonné doit être initialisé à zéro avant sa première attache
 * (`data_sub_t sub = {0};` ou variable statique).
 */
struct data_sub_t {
    data_topic_t        *topic;     /**< Référence vers le topic associé. */
    size_t               tail;      /**< Slot de la prochaine donnée à lire. */
    uint32_t             last_seq;  /**< Nombre de publications déjà consommées (comparé à pub_seq). */
    int                  attached;  /**< 0 = détaché, 1 = attaché. */
    struct data_sub_t   *prev;      /**< Pointeur vers l’abonné précédent (liste chainée). */
    struct data_sub_t   *next;      /**< Pointeur vers l’abonné suivant (liste chainée). */
#if (APEX_CFG_SCHED_RTOS == 1)
    StaticSemaphore_t    sem_cm;    /**< Mémoire statique du sémaphore de notification. */
    SemaphoreHandle_t    sem;       /**< Sémaphore binaire, NULL hors attache. */
#endif
};
typedef struct data_sub_t data_sub_t;

/* --------------------------------------------------------------------------
 *   API Topic
 * -------------------------------------------------------------------------- */

/**
 * @brief Initialise un topic à partir d’une mémoire externe.
 *
 * @param topic     Structure du topic à initialiser.
 * @param storage   Mémoire externe du buffer (elem_size * capacity octets).
 * @param elem_size Taille d’un élément (en octets).
 * @param capacity  Nombre maximal d’éléments conservés.
 * @param policy    Doit valoir CB_OVERWRITE_OLDEST. CB_REJECT_NEW est refusé :
 *                  les abonnés ne dépilent jamais le buffer, un topic en
 *                  REJECT_NEW serait plein pour toujours après `capacity`
 *                  publications.
 * @return DT_OK, ou DT_BAD_ARG (paramètre invalide ou politique refusée).
 *
 * @note Ne pas réinitialiser un topic qui a encore des abonnés : appeler
 *       data_topic_free d’abord, qui les détache.
 */
data_status_t data_topic_init(data_topic_t *topic,
                              void *storage, size_t elem_size, size_t capacity,
                              cb_overflow_policy_t policy);

/**
 * @brief Détache tous les abonnés puis remet le topic à zéro.
 *
 * Les abonnés détachés renvoient ensuite DT_BAD_ARG, et le topic refuse
 * publish et attach jusqu’au prochain data_topic_init. Réservé aux tâches, et
 * aucune tâche ne doit être bloquée dans data_sub_wait_for_data sur ce topic.
 */
void data_topic_free(data_topic_t *topic);

/**
 * @brief Publie une nouvelle donnée dans le topic.
 *
 * Copie la donnée (écrasant la plus ancienne si le buffer est plein),
 * incrémente pub_seq, puis réveille les abonnés. Ne bloque jamais.
 * Utilisable depuis une interruption.
 *
 * @return DT_OK, ou DT_BAD_ARG (pointeur NULL, topic non initialisé).
 */
data_status_t data_topic_publish(data_topic_t *topic, const void *elem);

/* --------------------------------------------------------------------------
 *   API Subscriber : attachement / détachement / synchronisation
 * -------------------------------------------------------------------------- */

/**
 * @brief Attache un abonné à un topic.
 *
 * @param sub   Abonné, initialisé à zéro ou détaché.
 * @param topic Topic auquel s’attacher.
 * @param mode  Mode d’attache (FROM_NOW ou FROM_OLDEST).
 * @return DT_OK (y compris si déjà attaché à ce même topic, la position
 *         n’est alors pas modifiée), DT_BAD_ARG sinon (mode invalide, déjà
 *         attaché à un autre topic, topic non initialisé, appel depuis une
 *         ISR, ou abonné remis à zéro alors qu'il figure encore dans la liste
 *         du topic : voir "Durée de vie" en tête de fichier).
 */
data_status_t data_sub_attach(data_sub_t *sub,
                              data_topic_t *topic,
                              data_attach_mode_t mode);

/**
 * @brief Détache un abonné du topic.
 *
 * Aucune tâche ne doit être bloquée dans data_sub_wait_for_data sur cet
 * abonné (en pratique : c’est sa tâche propriétaire qui le détache).
 */
data_status_t data_sub_detach(data_sub_t *sub);

/**
 * @brief Recale explicitement la position de lecture d’un abonné.
 *
 * @param mode DATA_ATTACH_FROM_NOW : saute tout ce qui n’a pas été lu
 *             (num_to_read devient 0, "je repars de maintenant").
 *             DATA_ATTACH_FROM_OLDEST : repart de la plus ancienne donnée
 *             encore présente (garde le plus d’historique possible).
 *
 * La resynchronisation automatique après une perte (DT_DATA_LOSS) se fait
 * toujours en FROM_OLDEST.
 */
data_status_t data_sub_sync(data_sub_t *sub, data_attach_mode_t mode);

/* --------------------------------------------------------------------------
 *   API Subscriber : lecture
 * -------------------------------------------------------------------------- */

/**
 * @brief Nombre de publications non lues par un abonné.
 *
 * Valeur brute pub_seq - last_seq : si elle dépasse la capacité du topic,
 * l’abonné a été dépassé et sa prochaine lecture renverra DT_DATA_LOSS.
 * Renvoie 0 pour un abonné NULL ou détaché.
 */
uint32_t data_sub_num_to_read(const data_sub_t *sub);

/**
 * @brief Lecture destructive avec copie : copie la prochaine donnée et avance.
 *
 * @return DT_OK, DT_DATA_LOSS (donnée copiée, mais des données plus anciennes
 *         ont été perdues ; l’abonné a été recalé sur la plus ancienne
 *         donnée encore présente, qui est celle copiée), DT_EMPTY (rien à
 *         lire, out_elem intact) ou DT_BAD_ARG.
 */
data_status_t data_sub_read(data_sub_t *sub, void *out_elem);

/**
 * @brief Lecture non destructive avec copie.
 *
 * @param idx Rang parmi les données non lues : 0 = la prochaine que
 *            data_sub_read rendrait, 1 = la suivante, etc.
 * @return Comme data_sub_read ; DT_EMPTY si idx >= nombre de données
 *         disponibles. Une perte est signalée (et l’abonné recalé) par le
 *         premier accès qui la constate, peek ou read.
 */
data_status_t data_sub_peek(data_sub_t *sub, void *out_elem, uint32_t idx);

/**
 * @brief Lecture destructive sans copie : rend l’adresse de la prochaine
 *        donnée dans le stockage du topic, puis avance.
 *
 * Voir la note "_ptr" en tête de fichier : l’adresse n’est protégée contre
 * aucune réécriture concurrente.
 */
data_status_t data_sub_read_ptr(data_sub_t *sub, const void **out_ptr);

/**
 * @brief Lecture non destructive sans copie, `idx` comme pour data_sub_peek.
 *
 * Même réserve que data_sub_read_ptr. `data_sub_peek_ptr(sub, &p, 0)`
 * donne la prochaine donnée à lire (la plus ancienne non lue).
 */
data_status_t data_sub_peek_ptr(data_sub_t *sub, const void **out_ptr, uint32_t idx);

#if (APEX_CFG_SCHED_RTOS == 1)
/* --------------------------------------------------------------------------
 *   API Subscriber : attente (RTOS)
 * -------------------------------------------------------------------------- */

/**
 * @brief Bloque jusqu’à ce que l’abonné ait au moins une donnée à lire.
 *
 * @param timeout_ms Délai maximal, en ticks du noyau (1 tick = 1 ms sur APEX,
 *                   configTICK_RATE_HZ = 1000), 0 pour tester sans attendre,
 *                   ou osWaitForever.
 * @retval osOK             num_to_read > 0 au retour.
 * @retval osErrorTimeout   Délai écoulé sans donnée.
 * @retval osErrorParameter Abonné NULL ou détaché.
 * @retval osErrorResource  Abonné sans sémaphore.
 * @retval osErrorISR       Appel depuis une interruption.
 * @retval osError          Noyau non démarré ou scheduler suspendu.
 */
osStatus_t data_sub_wait_for_data(data_sub_t *sub, uint32_t timeout_ms);

#endif

#ifdef __cplusplus
}
#endif

#endif /* DATA_TOPIC_H */
