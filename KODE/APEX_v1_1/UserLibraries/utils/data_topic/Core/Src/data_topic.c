#include "data_topic.h"
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#if (APEX_CFG_SCHED_RTOS == 1)
#include "task.h"
#endif

/* --------------------------------------------------------------------------
 *   Fonctions internes (non exportées)
 *
 *   Toutes les fonctions dt_*_locked s'appellent sous cb_critical_enter().
 * -------------------------------------------------------------------------- */

static inline bool dt_in_isr(void) {
    return __get_IPSR() != 0u;
}

static inline size_t dt_next(size_t idx, size_t capacity) {
    return (idx + 1u == capacity) ? 0u : idx + 1u;
}

/* Place le curseur d'un abonné selon `mode`. */
static void dt_place_locked(data_sub_t *sub, const data_topic_t *topic, data_attach_mode_t mode) {
    if (mode == DATA_ATTACH_FROM_OLDEST) {
        sub->tail     = topic->cb.tail;
        sub->last_seq = topic->pub_seq - (uint32_t)topic->cb.count;
    } else {
        sub->tail     = topic->cb.head;
        sub->last_seq = topic->pub_seq;
    }
}

/* L'abonné d'un slot est-il validement attaché à CE topic ? attach l'inscrit
   et le marque attaché dans la même section critique, detach et free le
   désinscrivent et le démarquent de même : en usage normal, tout slot occupé
   passe ce contrôle. Sous RTOS, son sémaphore doit aussi être celui de sa
   propre structure (créé avant l'inscription, supprimé après la
   désinscription). Un abonné remis à zéro, une mémoire réutilisée ou un
   abonné rattaché à un autre topic ne le passe pas. */
static inline bool dt_sub_valid(const data_sub_t *sub, const data_topic_t *topic) {
#if (APEX_CFG_SCHED_RTOS == 1)
    if ((const void *)sub->sem != (const void *)&sub->sem_cm) {
        return false;
    }
#endif
    return (sub->attached != 0) && (sub->topic == topic);
}

/* Slot du registre qui désigne `sub` (NULL : premier slot libre), ou
   DATA_TOPIC_MAX_SUBS s'il n'y en a pas. */
static size_t dt_slot_locked(const data_topic_t *topic, const data_sub_t *sub) {
    for (size_t i = 0u; i < DATA_TOPIC_MAX_SUBS; i++) {
        if (topic->subs[i] == sub) {
            return i;
        }
    }
    return DATA_TOPIC_MAX_SUBS;
}

static void dt_unregister_locked(data_topic_t *topic, size_t slot) {
    topic->subs[slot] = NULL;
    if (topic->sub_count > 0u) {
        topic->sub_count--;
    }
}

/* Libère le slot `slot` si son abonné n'est pas valide (voir dt_sub_valid).
   L'abonné n'est jamais écrit : sa mémoire appartient peut-être déjà à
   quelqu'un d'autre. L'anomalie est comptée une fois, au moment où le slot
   est libéré. */
static void dt_drop_invalid_locked(data_topic_t *topic, size_t slot) {
    const data_sub_t *sub = topic->subs[slot];
    if (sub != NULL && !dt_sub_valid(sub, topic)) {
        dt_unregister_locked(topic, slot);
        topic->list_faults++;
    }
}

/* Détache `sub` côté abonné. Son sémaphore éventuel reste à libérer, hors
   section critique (dt_release_sem). */
static void dt_reset_sub_locked(data_sub_t *sub) {
    sub->attached = 0;
    sub->topic    = NULL;
    sub->tail     = 0u;
    sub->last_seq = 0u;
}

#if (APEX_CFG_SCHED_RTOS == 1)

/* Le sémaphore d'un abonné n'est supprimé qu'une fois l'abonné désinscrit du
   registre : aucune notification ne peut plus le viser (voir dt_notify). */
static void dt_release_sem(data_sub_t *sub) {
    SemaphoreHandle_t sem = sub->sem;
    sub->sem = NULL;
    if (sem != NULL) {
        vSemaphoreDelete(sem);
    }
}

/* Réveille les abonnés d'un topic, la donnée étant déjà validée.
 *
 * Le registre n'est modifié par une tâche (attach, detach, free) que sous
 * section critique. Il est donc stable pendant le parcours :
 * - en interruption : aucune tâche ne tourne pendant le parcours ;
 * - en tâche : le scheduler est suspendu, aucune autre tâche ne tourne.
 *   xSemaphoreGive sans attente est explicitement permis scheduler suspendu
 *   (configASSERT de xQueueGenericSend) ; les abonnés réveillés plus
 *   prioritaires prennent la main à xTaskResumeAll, après le parcours.
 * Seule une interruption plus prioritaire qui publie sur le même topic peut
 * libérer un slot invalide pendant le parcours : dt_drop_invalid_locked
 * refait son contrôle sous section critique.
 *
 * Noyau pas encore démarré : aucune tâche n'attend, il n'y a rien à
 * réveiller, et data_sub_wait_for_data regarde num_to_read avant de bloquer
 * (la donnée sera vue au premier appel). Aucune API FreeRTOS ne doit être
 * appelée à ce moment-là : xSemaphoreGiveFromISR passe par
 * vPortValidateInterruptPriority, dont le configASSERT sur PRIGROUP échoue
 * tant que xPortStartScheduler n'a pas tourné, et bloque la carte (constaté
 * sur cible, revue du 05/10/2026, B1).
 *
 * Un slot dont l'abonné n'est pas valide n'est pas notifié (ce serait
 * appeler xSemaphoreGive sur un contenu quelconque) : il est libéré et compté
 * (voir dt_drop_invalid_locked), et les autres slots sont notifiés. */
static void dt_notify(data_topic_t *topic) {
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
        return;
    }

    const bool in_isr = dt_in_isr();
    BaseType_t woken  = pdFALSE;
    if (!in_isr) {
        vTaskSuspendAll();
    }
    for (size_t i = 0u; i < DATA_TOPIC_MAX_SUBS; i++) {
        data_sub_t *sub = topic->subs[i];
        if (sub == NULL) {
            continue;
        }
        if (dt_sub_valid(sub, topic)) {
            if (in_isr) {
                (void)xSemaphoreGiveFromISR(sub->sem, &woken);
            } else {
                (void)xSemaphoreGive(sub->sem);
            }
        } else {
            cb_critical_t c = cb_critical_enter();
            dt_drop_invalid_locked(topic, i);
            cb_critical_exit(c);
        }
    }
    if (in_isr) {
        portYIELD_FROM_ISR(woken);
    } else {
        (void)xTaskResumeAll();
    }
}

#endif /* APEX_CFG_SCHED_RTOS */

/* Lecture commune : copie (out) et/ou adresse (out_ptr) de la donnée de rang
 * `idx` parmi les non lues, puis avance d'un cran si `consume`. */
static data_status_t dt_access(data_sub_t *sub, uint32_t idx,
                               void *out, const void **out_ptr, bool consume) {
    if (!sub || (!out && !out_ptr)) return DT_BAD_ARG;

    data_topic_t *topic = sub->topic;
    if (!topic) return DT_BAD_ARG;

    cb_critical_t c = cb_critical_enter();

    /* Revérifié sous la section critique : un detach a pu passer entre-temps. */
    if (!sub->attached || sub->topic != topic) {
        cb_critical_exit(c);
        return DT_BAD_ARG;
    }

    const circular_buffer_t *cb = &topic->cb;
    const uint32_t stored = (uint32_t)cb->count;
    const uint32_t lag    = topic->pub_seq - sub->last_seq;
    const uint32_t avail  = (lag < stored) ? lag : stored;

    if (idx >= avail) {
        cb_critical_exit(c);
        return DT_EMPTY;
    }

    /* Curseur cohérent ? Tant que lag <= stored, la prochaine donnée à lire
       est lag crans derrière head : sub->tail == (head - lag) mod capacity.
       Un écart veut dire que lag est faux : plus de 2^32 publications sans
       lecture (pub_seq - last_seq a fait un tour, voir "Limites"), ou
       curseur abîmé. L'abonné est alors traité comme dépassé, ce qu'il est
       forcément dans le premier cas. */
    const bool consistent = (lag <= stored) &&
                            (sub->tail == (cb->head + cb->capacity - (size_t)lag) % cb->capacity);

    data_status_t status = DT_OK;
    if (!consistent) {
        /* Dépassé : les plus anciennes données non lues ont été écrasées.
           On repart de la plus ancienne encore présente. */
        dt_place_locked(sub, topic, DATA_ATTACH_FROM_OLDEST);
        status = DT_DATA_LOSS;
    }

    const size_t   slot = (sub->tail + idx) % cb->capacity;
    const uint8_t *src  = cb->storage + (slot * cb->elem_size);
    if (out)     memcpy(out, src, cb->elem_size);
    if (out_ptr) *out_ptr = src;

    if (consume) {
        sub->tail = dt_next(sub->tail, cb->capacity);
        sub->last_seq++;
    }

    cb_critical_exit(c);
    return status;
}

/* --------------------------------------------------------------------------
 *   Topic : initialisation et publication
 * -------------------------------------------------------------------------- */

data_status_t data_topic_init(data_topic_t *topic,
                              void *storage, size_t elem_size, size_t capacity,
                              cb_overflow_policy_t policy) {
    if (!topic) return DT_BAD_ARG;

    topic->pub_seq     = 0u;
    topic->sub_count   = 0u;
    topic->list_faults = 0u;
    for (size_t i = 0u; i < DATA_TOPIC_MAX_SUBS; i++) {
        topic->subs[i] = NULL;
    }
    topic->cb.storage  = NULL;  /* refusé par publish/attach si l'init échoue */

    if (policy != CB_OVERWRITE_OLDEST) return DT_BAD_ARG;
    if (cb_init(&(topic->cb), storage, elem_size, capacity, policy) != CB_OK) {
        return DT_BAD_ARG;
    }
    return DT_OK;
}

void data_topic_free(data_topic_t *topic) {
    if (!topic) return;

    /* Un slot à la fois : le sémaphore d'un abonné est supprimé hors section
       critique, une fois l'abonné désinscrit. Un abonné invalide n'est pas
       touché (voir dt_drop_invalid_locked). */
    for (size_t i = 0u; i < DATA_TOPIC_MAX_SUBS; i++) {
        cb_critical_t c = cb_critical_enter();
        data_sub_t *sub = topic->subs[i];
        const bool valid = (sub != NULL) && dt_sub_valid(sub, topic);
        if (valid) {
            dt_unregister_locked(topic, i);
            dt_reset_sub_locked(sub);
        } else {
            dt_drop_invalid_locked(topic, i);
        }
        cb_critical_exit(c);

#if (APEX_CFG_SCHED_RTOS == 1)
        if (valid) {
            dt_release_sem(sub);
        }
#endif
    }

    (void)cb_free(&(topic->cb));

    /* storage à NULL : publish et attach refusent ensuite le topic, jusqu'au
       prochain data_topic_init. */
    cb_critical_t c = cb_critical_enter();
    topic->cb.storage = NULL;
    topic->cb.head    = 0u;
    topic->cb.tail    = 0u;
    topic->cb.count   = 0u;
    topic->pub_seq    = 0u;
    topic->sub_count  = 0u;
    cb_critical_exit(c);
}

data_status_t data_topic_publish(data_topic_t *topic, const void *elem) {
    if (!topic || !elem) return DT_BAD_ARG;

    /* Écriture et compteur validés ensemble : un abonné voit la publication
       entière ou pas du tout. */
    cb_critical_t c = cb_critical_enter();
    if (!topic->cb.storage) {
        cb_critical_exit(c);
        return DT_BAD_ARG;
    }
    (void)cb_push(&(topic->cb), elem);   /* OVERWRITE_OLDEST : ne refuse jamais */
    topic->pub_seq++;
    cb_critical_exit(c);

#if (APEX_CFG_SCHED_RTOS == 1)
    dt_notify(topic);
#endif

    return DT_OK;
}

/* --------------------------------------------------------------------------
 *   Subscriber : attachement / détachement / synchronisation
 * -------------------------------------------------------------------------- */

data_status_t data_sub_attach(data_sub_t *sub,
                              data_topic_t *topic,
                              data_attach_mode_t mode) {
    if (!sub || !topic || !topic->cb.storage) return DT_BAD_ARG;
    if (mode != DATA_ATTACH_FROM_NOW && mode != DATA_ATTACH_FROM_OLDEST) return DT_BAD_ARG;
    if (sub->attached) return (sub->topic == topic) ? DT_OK : DT_BAD_ARG;
#if (APEX_CFG_SCHED_RTOS == 1)
    if (dt_in_isr()) return DT_BAD_ARG;

    /* Créé vide, avant l'inscription : data_sub_wait_for_data regarde
       num_to_read avant d'attendre. Tant que `attached` vaut 0, la
       notification ne touche pas à ce sémaphore, même si un slot périmé
       désigne déjà cet abonné. */
    sub->sem = xSemaphoreCreateBinaryStatic(&sub->sem_cm);
    if (sub->sem == NULL) return DT_BAD_ARG;
#endif

    data_status_t status = DT_OK;
    cb_critical_t c = cb_critical_enter();
    if (!topic->cb.storage) {
        status = DT_BAD_ARG;                /* data_topic_free est passé entre-temps */
    } else {
        /* Les slots invalides sont libérés d'abord, dont un éventuel slot
           périmé de cet abonné (remis à zéro sans detach : `attached` vaut 0
           ici) : pas de doublon. Puis le premier slot libre est pris. */
        for (size_t i = 0u; i < DATA_TOPIC_MAX_SUBS; i++) {
            dt_drop_invalid_locked(topic, i);
        }
        const size_t slot = dt_slot_locked(topic, NULL);
        if (slot < DATA_TOPIC_MAX_SUBS) {
            topic->subs[slot] = sub;
            topic->sub_count++;
        } else {
            status = DT_NO_SLOT;
        }
        if (status == DT_OK) {
            dt_place_locked(sub, topic, mode);
            sub->topic    = topic;
            sub->attached = 1;
        }
    }
    cb_critical_exit(c);

#if (APEX_CFG_SCHED_RTOS == 1)
    if (status != DT_OK) {
        dt_release_sem(sub);
    }
#endif
    return status;
}

data_status_t data_sub_detach(data_sub_t *sub) {
    if (!sub || !sub->attached || !sub->topic) return DT_BAD_ARG;
#if (APEX_CFG_SCHED_RTOS == 1)
    if (dt_in_isr()) return DT_BAD_ARG;
#endif

    data_topic_t *topic = sub->topic;

    cb_critical_t c = cb_critical_enter();
    if (!sub->attached || sub->topic != topic) {
        cb_critical_exit(c);
        return DT_BAD_ARG;
    }
    const size_t slot = dt_slot_locked(topic, sub);
    if (slot < DATA_TOPIC_MAX_SUBS) {
        dt_unregister_locked(topic, slot);
    } else {
        topic->list_faults++;               /* absent : topic ré-initialisé avec cet abonné attaché */
    }
    dt_reset_sub_locked(sub);
    cb_critical_exit(c);

#if (APEX_CFG_SCHED_RTOS == 1)
    dt_release_sem(sub);
#endif

    return DT_OK;
}

data_status_t data_sub_sync(data_sub_t *sub, data_attach_mode_t mode) {
    if (!sub || !sub->attached || !sub->topic) return DT_BAD_ARG;
    if (mode != DATA_ATTACH_FROM_NOW && mode != DATA_ATTACH_FROM_OLDEST) return DT_BAD_ARG;

    data_topic_t *topic = sub->topic;

    cb_critical_t c = cb_critical_enter();
    if (!sub->attached || sub->topic != topic) {
        cb_critical_exit(c);
        return DT_BAD_ARG;
    }
    dt_place_locked(sub, topic, mode);
    cb_critical_exit(c);

    return DT_OK;
}

/* --------------------------------------------------------------------------
 *   Subscriber : lecture
 * -------------------------------------------------------------------------- */

uint32_t data_sub_num_to_read(const data_sub_t *sub) {
    if (!sub || !sub->attached) return 0u;

    const data_topic_t *topic = sub->topic;
    if (!topic) return 0u;

    return topic->pub_seq - sub->last_seq;
}

data_status_t data_sub_read(data_sub_t *sub, void *out_elem) {
    if (!out_elem) return DT_BAD_ARG;
    return dt_access(sub, 0u, out_elem, NULL, true);
}

data_status_t data_sub_peek(data_sub_t *sub, void *out_elem, uint32_t idx) {
    if (!out_elem) return DT_BAD_ARG;
    return dt_access(sub, idx, out_elem, NULL, false);
}

data_status_t data_sub_read_ptr(data_sub_t *sub, const void **out_ptr) {
    if (!out_ptr) return DT_BAD_ARG;
    return dt_access(sub, 0u, NULL, out_ptr, true);
}

data_status_t data_sub_peek_ptr(data_sub_t *sub, const void **out_ptr, uint32_t idx) {
    if (!out_ptr) return DT_BAD_ARG;
    return dt_access(sub, idx, NULL, out_ptr, false);
}

#if (APEX_CFG_SCHED_RTOS == 1)

/* --------------------------------------------------------------------------
 *   API Subscriber : attente
 * -------------------------------------------------------------------------- */

osStatus_t data_sub_wait_for_data(data_sub_t *sub, uint32_t timeout_ms) {
    if (dt_in_isr()) return osErrorISR;
    if (!sub || !sub->attached) return osErrorParameter;
    if (!sub->sem) return osErrorResource;
    if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING) return osError;

    /* osWaitForever == portMAX_DELAY : attente infinie. */
    const TickType_t total = (TickType_t)timeout_ms;
    const TickType_t start = xTaskGetTickCount();

    for (;;) {
        /* La donnée est validée avant le réveil : si elle est là, inutile
           d'attendre, et un jeton resté dans le sémaphore ne compte pas. */
        if (data_sub_num_to_read(sub) > 0u) return osOK;

        TickType_t wait = portMAX_DELAY;
        if (total != portMAX_DELAY) {
            const TickType_t elapsed = xTaskGetTickCount() - start;
            if (elapsed >= total) return osErrorTimeout;
            wait = total - elapsed;
        }

        if (xSemaphoreTake(sub->sem, wait) != pdTRUE) {
            return (data_sub_num_to_read(sub) > 0u) ? osOK : osErrorTimeout;
        }
    }
}

#endif
