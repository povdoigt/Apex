#include "main_config.h"

#if (APEX_CFG_SCHED_SEQ + APEX_CFG_SCHED_RTOS) != 1
#error "ERROR CONFIG: SCHEDULE PROFILE - Only one schedule profile can be selected."
#endif

#if (APEX_CFG_PROFILE_TEST + APEX_CFG_PROFILE_MISSION) != 1
#error "ERROR CONFIG: PROFILE SELECTION - Only one profile can be selected."
#endif

#if APEX_CFG_PROFILE_MISSION == 1
#if (APEX_CFG_MISSION_DEFAULT) != 1
#error "ERROR CONFIG: MISSION PROFILE - Only one mission profile can be selected."
#endif
#endif
