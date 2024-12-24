//*****************************************************************************
//
//! @file am_hal_sysctrl_ton_config.h
//!
//! @brief Internal api definition for Ton Control
//!
//
//*****************************************************************************

//*****************************************************************************
//
// ${copyright}
//
// This is part of revision ${version} of the AmbiqSuite Development Package.
//
//*****************************************************************************
//! @cond SYSCTRL_TON_CONFIG
#ifndef AM_HAL_SYSCTRL_TON_CONFIG_H
#define AM_HAL_SYSCTRL_TON_CONFIG_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "am_hal_pwrctrl.h"

//*****************************************************************************
//
//! TON setting values for each power rail and level
//
//*****************************************************************************
#define SYSCTRL_VDDCACTLOWTONTRIM       (0x0E)
#define SYSCTRL_VDDCACTHIGHTONTRIM      (0x1F)
#define SYSCTRL_VDDFACTLOWTONTRIM       (0x15)
#define SYSCTRL_VDDFACTHIGHTONTRIM      (0x1F)
#define SYSCTRL_VDDCLVACTLOWTONTRIM     (0x0B)
#define SYSCTRL_VDDCLVACTHIGHTONTRIM    (0x0B)


//*****************************************************************************
//
//! Definition of TON Value Levels
//
//*****************************************************************************
typedef enum
{
    SYSCTRL_TON_LEVEL_NA = 0,
    SYSCTRL_TON_LEVEL_LOW,
    SYSCTRL_TON_LEVEL_HIGH,
} am_hal_sysctrl_ton_levels_t;

//*****************************************************************************
//
//! Definition of CPU/GPU Power State enumeration for SIMOBUCK TON control
//
//*****************************************************************************
typedef enum
{
    SYSCTRL_GPU_TON_POWER_STATE_OFF,
    SYSCTRL_GPU_TON_POWER_STATE_LP,
    SYSCTRL_GPU_TON_POWER_STATE_HP,
    SYSCTRL_GPU_TON_POWER_STATE_MAX,
} am_hal_sysctrl_gpu_ton_power_state;

typedef enum
{
    SYSCTRL_CPU_TON_POWER_STATE_OFF = 0,
    SYSCTRL_CPU_TON_POWER_STATE_LP,
    SYSCTRL_CPU_TON_POWER_STATE_HP,
    SYSCTRL_CPU_TON_POWER_STATE_MAX,
} am_hal_sysctrl_cpu_ton_power_state;

//*****************************************************************************
//
//! @brief Initialize Ton config handling during power up.
//
//*****************************************************************************
void am_hal_sysctrl_ton_config_init();

#ifdef __cplusplus
}
#endif

#endif //AM_HAL_SYSCTRL_TON_CONFIG_H

//! @endcond SYSCTRL_TON_CONFIG
