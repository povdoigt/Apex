#include "config/main_config.h"

// Check if only one of the schedule profiles is set to 1
#if (APEX_CFG_SCHED_SEQ + APEX_CFG_SCHED_RTOS) != 1
#error "ERROR CONFIG: SCHEDULE PROFILE - Only one schedule profile can be selected. \
Please set either APEX_CFG_SCHED_SEQ or APEX_CFG_SCHED_RTOS to 1."
#endif

// Check if only one of the profile selections is set to 1 
#if (APEX_CFG_PROFILE_TEST + APEX_CFG_PROFILE_MISSION) != 1
#error "ERROR CONFIG: PROFILE SELECTION - Only one profile can be selected. \
Please set either APEX_CFG_PROFILE_TEST or APEX_CFG_PROFILE_MISSION to 1."
#endif

// If test profile is selected, check that only one test profile is set to 1
#if APEX_CFG_PROFILE_TEST == 1
#if (APEX_CFG_UTEST) != 1
#error "ERROR CONFIG: TEST PROFILE - Only one test profile can be selected."
#endif
#endif

// If mission profile is selected, check that only one mission profile is set to 1
#if APEX_CFG_PROFILE_MISSION == 1
#if (APEX_CFG_MISSION_DEFAULT) != 1
#error "ERROR CONFIG: MISSION PROFILE - Only one mission profile can be selected."
#endif
#endif