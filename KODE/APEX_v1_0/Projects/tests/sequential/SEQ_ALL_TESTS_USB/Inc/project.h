#ifndef PROJECT_H
#define PROJECT_H

#include "main_config.h"
#include "drivers_config.h"
#include "tests_config.h"

// =======================================================================
// Inclusion conditionnelle des headers de test selon tests_config.h
// =======================================================================
#if (APEX_TEST_ENABLE_CB == 1)
#include "cb_seq_test.h"
#endif

#if (APEX_TEST_ENABLE_DT == 1)
#include "dt_seq_test.h"
#endif

#if (APEX_TEST_ENABLE_BMI088 == 1)
#include "BMI088_seq_test.h"
#endif

#if (APEX_TEST_ENABLE_W25Q == 1)
#include "w25q_seq_test.h"
#endif

// Setup : exécuté une seule fois après l'init des périphériques.
void setup(void);

// Loop : appelé en boucle après setup(). Rien à faire ici.
void loop(void);

#endif // PROJECT_H
