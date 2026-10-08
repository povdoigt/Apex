#ifndef BMI088_RTOS_BENCH_H
#define BMI088_RTOS_BENCH_H

#include "BMI088_rtos.h"

#if (APEX_CFG_SCHED_RTOS == 1)

/* ========================================================================
 * Banc de performance de la couche RTOS du driver BMI088
 *
 * Mesures (DWT, en cycles CPU, resumees en min / mediane / p99 / max / moy) :
 *   A1  Couche SPI1 RTOS : cout fixe Begin/End, transfert full-duplex DMA
 *       aux tailles utilisees par le driver, compare au HAL bloquant ;
 *       temps mort tIDLE (BMI088_DelayUs).
 *   A2  Primitives : verrou capteur, ReadRegister (avec / sans verrou),
 *       WriteRegister, ReadID, ReadAcc / ReadGyr / ReadTemp, decodage seul,
 *       et lectures de la couche sequentielle (HAL bloquant) pour comparer.
 *   A3  Sequences : soft resets, ApplyConfig direct et par sa tache, Init.
 *   A4  Cycle d'acquisition sans tache (ACC + GYR [+ TEMP], publication) :
 *       donne le debit maximal theorique.
 *   B1  Tache BMI088_Acquire a 1 / 2 / 5 / 10 ms : cycles, retards,
 *       erreurs, CPU laisse a une tache de priorite inferieure.
 *   B2  Gigue de periode vue par un abonne de priorite haute.
 *   C   Marge de pile de chaque pool apres le banc.
 *
 * Contexte d'execution :
 *   - Depuis un thread a osPriorityNormal, kernel demarre, carte immobile.
 *   - Le .c declare ses pools (TASK_POOL) : il ne doit pas etre lie avec
 *     BMI088_rtos_test.c. Il ne compile qu'avec APEX_CFG_BMI088_BENCH a 1
 *     dans main_config.h, ce qui exclut la suite de tests RTOS.
 *   - Rien n'est imprime pendant les mesures : appeler
 *     BMI088_rtos_bench_print() une fois le terminal ouvert.
 * ======================================================================== */

/* Initialise le capteur par la tache BMI088_Init (pool du banc). Cree le
   semaphore au premier appel ; le bmi088_t doit etre a zero. */
BMI_STATE BMI088_rtos_bench_init_imu(bmi088_t *imu, const bmi_config_t *config, uint32_t timeout_ms);

/* Execute tout le banc et garde les resultats en RAM. */
void BMI088_rtos_bench_run(bmi088_t *imu, const bmi_config_t *config);

/* Imprime le rapport : tableaux, acquisition, piles, puis CSV. */
void BMI088_rtos_bench_print(void (*print)(const char *), BMI_STATE init_st);

#endif /* APEX_CFG_SCHED_RTOS == 1 */
#endif /* BMI088_RTOS_BENCH_H */
