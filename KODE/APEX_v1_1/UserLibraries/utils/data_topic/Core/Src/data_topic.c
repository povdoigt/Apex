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

/* Un abonné joignable par la liste est-il cohérent ? Sous RTOS, son sémaphore
   est toujours celui créé dans sa propre structure : attach le crée avant de
   le chaîner, detach et free ne le suppriment qu'après l'avoir retiré. Tout
   autre contenu (abonné remis à zéro, mémoire d'une tâche terminée sans
   detach puis réutilisée) rend le nœud inutilisable, et son `next` avec. */
static inline bool dt_sub_sane(const data_sub_t *sub) {
#if (APEX_CFG_SCHED_RTOS == 1)
    return (const void *)sub->sem == (const void *)&sub->sem_cm;
#else
    (void)sub;
    return true;
#endif
}

/* Le nœud est-il un abonné cohérent, attaché à CE topic ? Un abonné remis à
   zéro puis rattaché ailleurs reste cohérent, mais n'appartient plus à cette
   liste : on ne doit ni le parcourir ni l'écrire au titre de ce topic. */
static inline bool dt_sub_member(const data_sub_t *sub, const data_topic_t *topic) {
    return dt_sub_sane(sub) && sub->attached && sub->topic == topic;
}

/* Retire un abonné de la liste ("safe unlink"). Un voisin n'est réécrit que
   s'il appartient à ce topic et désigne bien `sub` en retour, la tête que si
   elle vaut `sub` : un voisin incohérent (mémoire peut-être déjà réutilisée),
   un voisin passé dans une autre liste, ou un lien périmé (liste coupée,
   topic réinitialisé avec ses abonnés) n'est jamais écrit. La liste est alors
   coupée à cet endroit, et l'anomalie comptée. En usage normal, tous les
   contrôles passent. */
static void dt_unlink_locked(data_sub_t *sub, data_topic_t *topic) {
    data_sub_t *const next = sub->next;
    data_sub_t *const prev = sub->prev;
    const bool next_ok = (next == NULL) || (dt_sub_member(next, topic) && next->prev == sub);
    bool fault   = !next_ok;
    bool in_list = false;          /* décroché de son prédécesseur (ou de la tête) */

    if (next != NULL && next_ok) {
        next->prev = prev;
    }
    if (prev == NULL) {
        if (topic->subs == sub) {
            topic->subs = next_ok ? next : NULL;
            in_list = true;
        } else {
            fault = true;
        }
    } else if (dt_sub_member(prev, topic) && prev->next == sub) {
        prev->next = next_ok ? next : NULL;
        in_list = true;
    } else {
        fault = true;
    }
    if (fault) {
        topic->list_faults++;
    }
    /* sub_count ne baisse que si l'abonné était bien dans la liste : un lien
       périmé ne doit pas réduire le compte des abonnés actuels. Resté trop
       haut (abonné au-delà d'un nœud incohérent), il ne fait qu'élargir les
       parcours bornés. */
    if (in_list && topic->sub_count > 0u) {
        topic->sub_count--;
    }

    sub->prev     = NULL;
    sub->next     = NULL;
    sub->attached = 0;
    sub->topic    = NULL;
    sub->tail     = 0u;
    sub->last_seq = 0u;
}

/* L'abonné figure-t-il déjà dans la liste du topic ? Parcours borné par
   sub_count : une liste plus longue que son compteur est corrompue, et la
   réponse est alors "oui", pour refuser l'attache plutôt que boucler. Le
   parcours s'arrête sur un nœud qui n'appartient pas (ou plus) au topic,
   dont le `next` n'est pas fiable. */
static bool dt_linked_locked(const data_topic_t *topic, const data_sub_t *sub) {
    size_t n = 0u;
    for (const data_sub_t *p = topic->subs; p != NULL; p = p->next) {
        if (p == sub || ++n > topic->sub_count) {
            return true;
        }
        if (!dt_sub_member(p, topic)) {
            return false;
        }
    }
    return false;
}

#if (APEX_CFG_SCHED_RTOS == 1)

/* Le sémaphore d'un abonné n'est supprimé qu'une fois l'abonné retiré de la
   liste : aucune notification ne peut plus le viser (voir dt_notify). */
static void dt_release_sem(data_sub_t *sub) {
    SemaphoreHandle_t sem = sub->sem;
    sub->sem = NULL;
    if (sem != NULL) {
        vSemaphoreDelete(sem);
    }
}

/* Réveille les abonnés d'un topic, la donnée étant déjà validée.
 *
 * La liste n'est modifiée que par attach/detach/free, en tâche et sous
 * section critique. Elle est donc stable :
 * - en interruption : aucune tâche ne tourne pendant le parcours ;
 * - en tâche : le scheduler est suspendu, aucune autre tâche ne tourne.
 *   xSemaphoreGive sans attente est explicitement permis scheduler suspendu
 *   (configASSERT de xQueueGenericSend) ; les abonnés réveillés plus
 *   prioritaires prennent la main à xTaskResumeAll, après le parcours.
 *
 * Noyau pas encore démarré : aucune tâche n'attend, il n'y a rien à
 * réveiller, et data_sub_wait_for_data regarde num_to_read avant de bloquer
 * (la donnée sera vue au premier appel). Aucune API FreeRTOS ne doit être
 * appelée à ce moment-là : xSemaphoreGiveFromISR passe par
 * vPortValidateInterruptPriority, dont le configASSERT sur PRIGROUP échoue
 * tant que xPortStartScheduler n'a pas tourné, et bloque la carte (constaté
 * sur cible, revue du 05/10/2026, B1).
 *
 * Un nœud incohérent (voir dt_sub_sane) arrête le parcours : le notifier
 * reviendrait à appeler xSemaphoreGive sur un contenu quelconque, et à suivre
 * un `next` quelconque. Les abonnés situés au-delà ne sont plus réveillés
 * (leurs données restent lisibles), et topic->list_faults le signale. */
static void dt_count_fault(data_topic_t *topic) {
    cb_critical_t c = cb_critical_enter();
    topic->list_faults++;
    cb_critical_exit(c);
}

static void dt_notify(data_topic_t *topic) {
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
        return;
    }

    bool fault = false;
    if (dt_in_isr()) {
        BaseType_t woken = pdFALSE;
        for (data_sub_t *sub = topic->subs; sub != NULL; sub = sub->next) {
            if (!dt_sub_member(sub, topic)) {
                fault = true;
                break;
            }
            (void)xSemaphoreGiveFromISR(sub->sem, &woken);
        }
        if (fault) {
            dt_count_fault(topic);
        }
        portYIELD_FROM_ISR(woken);
        return;
    }

    vTaskSuspendAll();
    for (data_sub_t *sub = topic->subs; sub != NULL; sub = sub->next) {
        if (!dt_sub_member(sub, topic)) {
            fault = true;
            break;
        }
        (void)xSemaphoreGive(sub->sem);
    }
    (void)xTaskResumeAll();
    if (fault) {
        dt_count_fault(topic);
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

    data_status_t status = DT_OK;
    if (lag > stored) {
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
    topic->subs        = NULL;
    topic->list_faults = 0u;
    topic->cb.storage  = NULL;  /* refusé par publish/attach si l'init échoue */

    if (policy != CB_OVERWRITE_OLDEST) return DT_BAD_ARG;
    if (cb_init(&(topic->cb), storage, elem_size, capacity, policy) != CB_OK) {
        return DT_BAD_ARG;
    }
    return DT_OK;
}

void data_topic_free(data_topic_t *topic) {
    if (!topic) return;

    /* Un abonné à la fois : son sémaphore est supprimé hors section critique,
       une fois l'abonné retiré de la liste. Un abonné incohérent en tête (voir
       dt_sub_sane) n'est pas touché : le reste de la liste est abandonné. */
    for (;;) {
        cb_critical_t c = cb_critical_enter();
        data_sub_t *sub = topic->subs;
        const bool sane = (sub != NULL) && dt_sub_member(sub, topic);
        if (sane) {
            dt_unlink_locked(sub, topic);
            if (topic->subs == sub) {
                topic->subs = NULL;     /* tête aux liens incohérents : on s'arrête là */
            }
        } else if (sub != NULL) {
            topic->subs = NULL;
            topic->list_faults++;
        }
        cb_critical_exit(c);

        if (!sane) break;
#if (APEX_CFG_SCHED_RTOS == 1)
        dt_release_sem(sub);
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
#endif

    /* Garde-fou : un abonné encore chaîné mais remis à zéro sans detach
       (variable locale d'une tâche qui a rendu la main sans se détacher puis a
       été relancée, structure réinitialisée...) refermerait la liste sur
       elle-même, et dt_notify bouclerait sans fin, scheduler suspendu ou en
       interruption. */
    cb_critical_t c = cb_critical_enter();
    const bool linked = dt_linked_locked(topic, sub);
    cb_critical_exit(c);
    if (linked) return DT_BAD_ARG;

#if (APEX_CFG_SCHED_RTOS == 1)
    /* Créé vide : data_sub_wait_for_data regarde num_to_read avant d'attendre. */
    sub->sem = xSemaphoreCreateBinaryStatic(&sub->sem_cm);
    if (sub->sem == NULL) return DT_BAD_ARG;
#endif

    c = cb_critical_enter();
    if (!topic->cb.storage) {
        /* data_topic_free est passé entre les deux sections critiques. */
        cb_critical_exit(c);
#if (APEX_CFG_SCHED_RTOS == 1)
        dt_release_sem(sub);
#endif
        return DT_BAD_ARG;
    }
    dt_place_locked(sub, topic, mode);
    sub->topic = topic;
    sub->prev  = NULL;
    sub->next  = topic->subs;
    if (topic->subs != NULL) {
        if (dt_sub_member(topic->subs, topic)) {
            topic->subs->prev = sub;
        } else {
            topic->list_faults++;   /* tête incohérente : jamais écrite */
        }
    }
    topic->subs = sub;
    topic->sub_count++;
    sub->attached = 1;
    cb_critical_exit(c);

    return DT_OK;
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
    dt_unlink_locked(sub, topic);
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
