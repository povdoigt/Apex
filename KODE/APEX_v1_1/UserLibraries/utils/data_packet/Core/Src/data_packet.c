#include "data_packet.h"
#include "circular_buffer.h"
#include "data_topic.h"

#include <string.h>

/* Taille d'un élément source utilisable par un packer, 0 sinon. */
static size_t data_packer_elem_size(const data_topic_t *topic) {
    if (topic == NULL || topic->cb.storage == NULL) {
        return 0u;
    }
    const size_t elem_size = topic->cb.elem_size;
    if (elem_size < sizeof(data_ts_generic_t) || elem_size > DATA_PACKET_MAX_ELEM_SIZE) {
        return 0u;
    }
    return elem_size;
}

size_t data_packer_packet_size(data_topic_t **topics, size_t num_topics) {
    if (topics == NULL || num_topics == 0u || num_topics > DATA_PACKET_MAX_TOPICS) {
        return 0u;
    }
    size_t packet_size = sizeof(data_ts_packet_generic_t);
    for (size_t i = 0; i < num_topics; i++) {
        const size_t elem_size = data_packer_elem_size(topics[i]);
        if (elem_size == 0u) {
            return 0u;
        }
        packet_size += elem_size - sizeof(data_ts_generic_t);
    }
    return (packet_size <= DATA_PACKET_MAX_PACKET_SIZE) ? packet_size : 0u;
}

data_status_t data_packer_init(data_packer_t *packer, uint32_t window_ms, size_t num_topics,
                               data_topic_t **topics, size_t cb_capacity, void *storage) {
    if (packer == NULL) {
        return DT_BAD_ARG;
    }
    const size_t packet_size = data_packer_packet_size(topics, num_topics);
    if (packet_size == 0u) {
        return DT_BAD_ARG;
    }
    /* Packer à zéro ou libéré : un abonné encore attaché (ou un topic de
       paquets encore suivi) serait perdu par la réinitialisation. */
    if (packer->topic.subs != NULL) {
        return DT_BAD_ARG;
    }
    for (size_t i = 0; i < DATA_PACKET_MAX_TOPICS; i++) {
        if (packer->subs[i].attached) {
            return DT_BAD_ARG;
        }
    }

    if (data_topic_init(&packer->topic, storage, packet_size, cb_capacity, CB_OVERWRITE_OLDEST) != DT_OK) {
        return DT_BAD_ARG;
    }
    for (size_t i = 0; i < num_topics; i++) {
        if (data_sub_attach(&packer->subs[i], topics[i], DATA_ATTACH_FROM_NOW) != DT_OK) {
            while (i-- > 0u) {
                (void)data_sub_detach(&packer->subs[i]);
            }
            data_topic_free(&packer->topic);
            return DT_BAD_ARG;
        }
        /* Taille gardée ici : un abonné détaché n'a plus de topic à interroger. */
        packer->payload_size[i] = topics[i]->cb.elem_size - sizeof(data_ts_generic_t);
    }
    packer->T           = window_ms;
    packer->num_topics  = num_topics;
    packer->packet_size = packet_size;
    return DT_OK;
}

void data_packer_free(data_packer_t *packer) {
    if (packer == NULL) {
        return;
    }
    for (size_t i = 0; i < DATA_PACKET_MAX_TOPICS; i++) {
        if (packer->subs[i].attached) {
            (void)data_sub_detach(&packer->subs[i]);
        }
    }
    /* Détache aussi les abonnés du topic des paquets. */
    data_topic_free(&packer->topic);
    packer->num_topics  = 0u;
    packer->packet_size = 0u;
}

static data_packer_status_t data_packer_check(data_packer_t *packer, size_t i, uint32_t current_time_ms, uint8_t *out) {
    data_sub_t *sub = &packer->subs[i];
    data_ts_generic_t *sample = (data_ts_generic_t *)packer->elem;
    const void *consumed;
    const int32_t half = (int32_t)(packer->T / 2u);

    /* Trop vieille : (t - ts) > T/2 en arithmetique signee.
     * Le cast (int32_t) de la difference gere a la fois le wrap du uint32
     * et le signe : une mesure future (ts > t) donne un ecart negatif,
     * donc n'est jamais consideree comme trop vieille.
     * DT_DATA_LOSS rend quand meme une donnee valide (la plus ancienne
     * restante) ; tout autre statut (vide, abonne detache) arrete la.
     * Chaque echantillon est copie avant d'etre examine : son publieur peut
     * reecrire le slot a tout moment. S'il en publie assez entre la copie et
     * l'avance du curseur pour provoquer une perte, l'avance saute un autre
     * echantillon que celui copie : un echantillon perdu de plus, jamais une
     * donnee melangee. */
    for (;;) {
        const data_status_t status = data_sub_peek(sub, sample, 0u);
        if (status != DT_OK && status != DT_DATA_LOSS) {
            return PACKER_EMPTY;
        }
        if ((int32_t)(current_time_ms - sample->ts) <= half) {
            break;
        }
        (void)data_sub_read_ptr(sub, &consumed); // discard the old data (pointer never dereferenced)
    }

    if ((int32_t)(sample->ts - current_time_ms) > half) {
        /* Trop jeune : (ts - t) > T/2 en arithmetique signee. */
        return PACKER_TOO_YOUNG;
    }
    memcpy(out, sample->data, packer->payload_size[i]);
    (void)data_sub_read_ptr(sub, &consumed); // just to advance the subscriber's tail
    return PACKER_VALID;
}

uint32_t data_packer_build_publish(data_packer_t *packer, uint32_t current_time_ms) {
    if (packer == NULL || packer->packet_size == 0u) {
        return 0u;   /* packer non initialise ou libere */
    }
    /* Construit dans `staging`, jamais dans le stockage du topic : un abonne
       en retard de `capacity` paquets lit encore le slot que la publication
       va ecraser. */
    data_ts_packet_generic_t *packet = (data_ts_packet_generic_t *)packer->staging;
    packet->ts = current_time_ms;
    packet->flags = 0u;
    size_t offset = 0u;
    for (size_t i = 0; i < packer->num_topics; i++) {
        uint8_t *data = packet->data + offset;
        if (data_packer_check(packer, i, current_time_ms, data) == PACKER_VALID) {
            packet->flags |= (1u << i);
        } else {
            memset(data, 0, packer->payload_size[i]);   /* champ absent : a zero */
        }
        offset += packer->payload_size[i];
    }
    if (data_topic_publish(&packer->topic, packet) != DT_OK) {
        return 0u;
    }
    return packet->flags;
}
