#ifndef DT_RTOS_STRESS_H
#define DT_RTOS_STRESS_H

#include "main_config.h"

#include <stdbool.h>
#include <stdint.h>

#if (APEX_CFG_SCHED_RTOS == 1)

/* ========================================================================
 * Endurance et mesures de data_topic / circular_buffer / data_packet
 *
 * A appeler depuis le thread applicatif (kernel demarre, osPriorityNormal),
 * comme la suite RTOS. Le .c n'est compile qu'avec APEX_CFG_SCHED_RTOS et
 * APEX_CFG_PROFILE_TEST a 1, et declare ses propres pools de taches. Rien
 * n'est imprime pendant les mesures : les resultats sont gardes, puis
 * imprimes par les fonctions *_print une fois le terminal ouvert.
 * ======================================================================== */

/* Endurance "facon mission", `duration_s` secondes :
 *   publieurs : acc 1 kHz en interruption (TIM5), gyr 500 Hz (tache High),
 *               baro 50 Hz (tache AboveNormal) ;
 *   abonnes   : nav (AboveNormal1, attend acc, lit acc + gyr, ne doit rien
 *               perdre), logger (BelowNormal, toutes les 10 ms), telemetrie
 *               (Low, toutes les 100 ms, perd par construction), rafale
 *               d'attache / detache sur acc toutes les 50 ms (Normal) ;
 *   data_packet : un packer (BelowNormal, 10 Hz) empaquette acc / gyr / baro,
 *               une tache radio (Low) relit et verifie chaque paquet.
 * Chaque element lu est verifie (mots de controle), chaque trou doit etre
 * annonce par DT_DATA_LOSS, et lus + sautes == publies pour chaque abonne.
 * Le thread appelant verifie chaque seconde les invariants des topics sous
 * section critique. Rend true si tout est conforme. */
bool DT_stress_endurance_run(uint32_t duration_s);
void DT_stress_endurance_print(void (*print)(const char *));

/* Duree des sections critiques (cycles DWT, convertis en us) : cb_push,
 * cb_pop et data_sub_read pour des elements de 4 a 256 octets, mesures
 * interruptions masquees (borne haute du temps passe masque), puis
 * data_topic_publish complet (notification de 0, 1 ou 4 abonnes). */
void DT_stress_timing_run(void);
void DT_stress_timing_print(void (*print)(const char *));

#endif /* APEX_CFG_SCHED_RTOS */

#endif /* DT_RTOS_STRESS_H */
