#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include "main_config.h"

#include <stdint.h>
#include <stddef.h>

#include "stm32f4xx.h"          /* CMSIS core : __get_PRIMASK, __disable_irq */

#if (APEX_CFG_SCHED_RTOS == 1)
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 *   Concurrence
 *
 *   Chaque opération qui touche à l'état du buffer (push, pop, reset, peek
 *   avec copie) s'exécute dans une section critique courte : quelques
 *   affectations et un memcpy de elem_size octets. Le buffer est donc
 *   utilisable en même temps depuis plusieurs tâches et depuis des
 *   interruptions, sans objet noyau :
 *
 *   - RTOS : masquage par BASEPRI (taskENTER_CRITICAL_FROM_ISR). Une ISR qui
 *     utilise le buffer doit avoir une priorité NVIC numériquement
 *     >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, comme pour toute API
 *     FreeRTOS "FromISR". Les interruptions plus prioritaires ne sont pas
 *     masquées et ne doivent pas toucher au buffer.
 *   - Séquentiel : masquage global par PRIMASK.
 *
 *   La durée de la section critique est proportionnelle à elem_size : garder
 *   des éléments petits (quelques centaines d'octets au plus).
 *
 *   cb_init n'est pas protégé : un buffer s'initialise avant d'être partagé.
 *   Les accès "pointeur" (cb_peek_ptr, cb_peek_relative_ptr) rendent une
 *   adresse dans le stockage : son contenu peut être réécrit par un push
 *   concurrent dès le retour de la fonction.
 * -------------------------------------------------------------------------- */

/** @brief État sauvegardé par cb_critical_enter(), à rendre à cb_critical_exit(). */
typedef uint32_t cb_critical_t;

/**
 * @brief Entre dans la section critique du buffer (imbricable).
 *
 * Valide en tâche, en interruption et avant le démarrage du noyau. En RTOS,
 * c'est une simple sauvegarde puis élévation de BASEPRI : aucune fonction
 * FreeRTOS ne doit être appelée entre enter et exit.
 * Partagée avec data_topic, qui protège son propre état avec.
 */
static inline cb_critical_t cb_critical_enter(void) {
#if (APEX_CFG_SCHED_RTOS == 1)
    return (cb_critical_t)taskENTER_CRITICAL_FROM_ISR();
#else
    cb_critical_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
#endif
}

/** @brief Sort de la section critique en restaurant l'état d'avant l'entrée. */
static inline void cb_critical_exit(cb_critical_t saved) {
#if (APEX_CFG_SCHED_RTOS == 1)
    taskEXIT_CRITICAL_FROM_ISR(saved);
#else
    __set_PRIMASK(saved);
#endif
}

/* --------------------------------------------------------------------------
 *   Types et configuration
 * -------------------------------------------------------------------------- */

/**
 * @brief Politique à appliquer en cas de dépassement de capacité.
 */
typedef enum {
    CB_OVERWRITE_OLDEST = 0,  /**< Écrase la donnée la plus ancienne si plein. */
    CB_REJECT_NEW             /**< Rejette la nouvelle donnée si plein.       */
} cb_overflow_policy_t;

/**
 * @brief Codes de statut renvoyés par les fonctions du circular buffer.
 */
typedef enum {
    CB_OK = 0,                /**< Opération réussie. */
    CB_EMPTY,                 /**< Aucune donnée à lire. */
    CB_FULL,                  /**< Tampon plein, nouvelle écriture refusée. */
    CB_OVERWROTE_OLDEST,      /**< Donnée la plus ancienne écrasée. */
    CB_BAD_ARG                /**< Paramètre invalide (pointeur NULL, etc.). */
} cb_status_t;

/**
 * @brief Structure d’un tampon circulaire générique.
 *
 * La mémoire de stockage n’est pas allouée par le buffer.
 * L’appelant doit fournir un pointeur vers un bloc de taille :
 * `elem_size * capacity` octets.
 *
 * Les champs sont internes : utiliser cb_count() plutôt que lire `count`.
 */
typedef struct circular_buffer_t {
    uint8_t *storage;               /**< Mémoire externe du buffer. */
    size_t   elem_size;             /**< Taille d’un élément (en octets). */
    size_t   capacity;              /**< Nombre maximal d’éléments. */
    size_t   head;                  /**< Prochaine position d’écriture. */
    size_t   tail;                  /**< Position du plus ancien élément. */
    size_t   count;                 /**< Nombre d’éléments actuellement stockés. */
    cb_overflow_policy_t policy;    /**< Politique en cas de dépassement. */
} circular_buffer_t;

/**
 * @brief Calcule la taille mémoire nécessaire pour stocker `n` éléments du type donné.
 */
#define CIRCULAR_BUFFER_BYTES(type, n) ((size_t)(sizeof(type) * (n)))

/* --------------------------------------------------------------------------
 *   Initialisation / reset
 * -------------------------------------------------------------------------- */

/**
 * @brief Initialise un tampon circulaire sur une mémoire externe.
 * @param cb        Pointeur vers la structure à initialiser.
 * @param storage   Mémoire fournie (doit être allouée par l’appelant).
 * @param elem_size Taille d’un élément (en octets).
 * @param capacity  Nombre maximal d’éléments.
 * @param policy    Politique en cas de dépassement.
 * @return CB_OK, ou CB_BAD_ARG si un paramètre est invalide (pointeur NULL,
 *         taille ou capacité nulle, politique inconnue, elem_size * capacity
 *         au-delà de SIZE_MAX).
 *
 * @note Ne crée aucun objet noyau : appeler cb_init sur un buffer déjà
 *       initialisé le vide simplement. Ne pas le faire pendant qu'un autre
 *       contexte l'utilise.
 */
cb_status_t cb_init(circular_buffer_t *cb,
                    void *storage, size_t elem_size, size_t capacity,
                    cb_overflow_policy_t policy);

/**
 * @brief Vide le buffer sans modifier la mémoire de stockage.
 */
cb_status_t cb_reset(circular_buffer_t *cb);

/**
 * @brief Libère les ressources du buffer.
 *
 * Le buffer ne possède plus aucune ressource (pas de mutex) : la fonction ne
 * fait que valider l'argument. Conservée pour la symétrie avec cb_init.
 */
cb_status_t cb_free(circular_buffer_t *cb);

/* --------------------------------------------------------------------------
 *   Écriture / lecture destructive
 * -------------------------------------------------------------------------- */

/**
 * @brief Ajoute une donnée au buffer (copie complète).
 *
 * Si le buffer est plein :
 * - en mode CB_OVERWRITE_OLDEST, l’élément le plus ancien est écrasé,
 * - en mode CB_REJECT_NEW, l’écriture est refusée.
 *
 * Utilisable depuis une interruption.
 *
 * @note Ne pas construire l’élément directement dans le slot `head`
 *       (cb_peek_ptr) : buffer plein, ce slot contient l’élément le plus
 *       ancien, encore lisible par les autres contextes pendant la
 *       construction, et détruit même si le push est ensuite refusé
 *       (CB_REJECT_NEW). Construire dans une variable puis pousser.
 */
cb_status_t cb_push(circular_buffer_t *cb, const void *elem);

/**
 * @brief Retire et copie la donnée la plus ancienne (FIFO).
 *
 * Utilisable depuis une interruption.
 */
cb_status_t cb_pop(circular_buffer_t *cb, void *out);

/**
 * @brief Nombre d’éléments actuellement stockés (0 si cb est NULL).
 *
 * Valeur instantanée : un push ou un pop concurrent peut la changer juste
 * après le retour.
 */
size_t cb_count(const circular_buffer_t *cb);

/* --------------------------------------------------------------------------
 *   Accès pointeur (sans copie)
 * -------------------------------------------------------------------------- */

/**
 * @brief Retourne un pointeur constant vers un élément à un index absolu.
 *
 * @note Comportement "wrap permissif" :
 *       si `idx >= capacity`, l’indice est ramené automatiquement par modulo.
 *       Aucun contrôle n’est effectué sur la validité temporelle de la donnée.
 */
const void *cb_peek_ptr(circular_buffer_t *cb, size_t idx);

/**
 * @brief Retourne un pointeur constant vers un élément relatif à une origine.
 *
 * @param cb      Pointeur vers le buffer.
 * @param origin  Index de base (souvent cb->tail).
 * @param offset  Décalage relatif (peut être positif ou négatif).
 *
 * @note Le comportement est également "wrap permissif".
 */
const void *cb_peek_relative_ptr(circular_buffer_t *cb,
                                 size_t origin, int offset);

/* --------------------------------------------------------------------------
 *   Accès lecture (avec copie)
 * -------------------------------------------------------------------------- */

/**
 * @brief Copie la donnée à un index absolu dans le buffer.
 *
 * Équivalent à : memcpy(out, cb_peek_ptr(cb, idx), elem_size), la copie
 * étant faite sous la section critique du buffer.
 */
cb_status_t cb_peek(circular_buffer_t *cb, size_t idx, void *out);

/**
 * @brief Copie la donnée à un offset relatif à une origine donnée.
 *
 * Équivalent à : memcpy(out, cb_peek_relative_ptr(cb, origin, offset), elem_size),
 * la copie étant faite sous la section critique du buffer.
 */
cb_status_t cb_peek_relative(circular_buffer_t *cb,
                             size_t origin, int offset, void *out);

#ifdef __cplusplus
}
#endif

#endif /* CIRCULAR_BUFFER_H */
