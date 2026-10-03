// tests_config.h
// Sélecteur de suites de tests unitaires séquentiels.
// Mettre à 1 les suites à exécuter, à 0 pour les désactiver.
//
// ⚠ Les tests de drivers matériels (BMI088, W25Q…) nécessitent que le
//   module correspondant soit aussi activé dans main_config.h
//   (APEX_ENABLE_<MODULE> = 1) et que le matériel soit physiquement
//   branché. Un #error est généré si cette règle n'est pas respectée.

#ifndef TESTS_CONFIG_H
#define TESTS_CONFIG_H

#include "main_config.h"

// =======================================================================
// Suites – utils (RAM uniquement, aucun matériel requis)
// =======================================================================
#define APEX_TEST_ENABLE_CB             1   // circular_buffer  (11 cas)
#define APEX_TEST_ENABLE_DT             1   // data_topic        (12 cas)

// =======================================================================
// Suites – drivers (matériel requis – voir main_config.h)
// =======================================================================
#define APEX_TEST_ENABLE_BMI088         1   // BMI088 IMU        (8 cas)  → APEX_ENABLE_BMI088
#define APEX_TEST_ENABLE_W25Q           1   // W25Q512 flash     (15 cas) → APEX_ENABLE_W25Q512

// =======================================================================
// Contrôles de cohérence (compile-time)
// Chaque suite driver nécessite le module matériel correspondant.
// =======================================================================
#if (APEX_TEST_ENABLE_BMI088 == 1) && (APEX_ENABLE_BMI088 == 0)
#error "TESTS_CONFIG: APEX_TEST_ENABLE_BMI088=1 mais APEX_ENABLE_BMI088=0 dans main_config.h"
#endif

#if (APEX_TEST_ENABLE_W25Q == 1) && (APEX_ENABLE_W25Q512 == 0)
#error "TESTS_CONFIG: APEX_TEST_ENABLE_W25Q=1 mais APEX_ENABLE_W25Q512=0 dans main_config.h"
#endif

// =======================================================================
// Garde : au moins une suite doit être activée
// =======================================================================
#if (APEX_TEST_ENABLE_CB    + \
     APEX_TEST_ENABLE_DT    + \
     APEX_TEST_ENABLE_BMI088 + \
     APEX_TEST_ENABLE_W25Q) == 0
#error "TESTS_CONFIG: Aucune suite de tests activée. Activez au moins une suite."
#endif


#endif // TESTS_CONFIG_H
