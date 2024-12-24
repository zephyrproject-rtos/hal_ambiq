//*****************************************************************************
//
//! @file am_hal_sysctrl.h
//!
//! @brief Functions for interfacing with the M4F system control registers
//!
//! @addtogroup sysctrl4 SYSCTRL - System Control
//! @ingroup apollo510L_hal
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// ${copyright}
//
// This is part of revision ${version} of the AmbiqSuite Development Package.
//
//*****************************************************************************
#ifndef AM_HAL_SYSCTRL_H
#define AM_HAL_SYSCTRL_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! @name Definitions for sleep mode parameter
//! @{
//
//*****************************************************************************
#define AM_HAL_SYSCTRL_SLEEP_DEEP       true
#define AM_HAL_SYSCTRL_SLEEP_NORMAL     false
//! @}

//*****************************************************************************
//
//! Definition of Global Power State enumeration
//
//*****************************************************************************
typedef enum
{
  AM_HAL_SYSCTRL_WAKE,
  AM_HAL_SYSCTRL_NORMALSLEEP,
  AM_HAL_SYSCTRL_DEEPSLEEP
} am_hal_sysctrl_power_state_e;

#define SYNC_READ       0x47FF0000

//*****************************************************************************
//
//! Write flush - This function will hold the bus until all queued write
//! operations on System Bus have completed, thereby guaranteeing that all
//! writes to APB have been flushed.
//
//*****************************************************************************
#define am_hal_sysctrl_sysbus_write_flush()     AM_REGVAL(SYNC_READ)

//*****************************************************************************
//
//! Write flush - This function will return once all queued write
//! operations have completed, thereby guaranteeing that all
//! writes have been flushed.
//! This works across all the buses - AXI and APB
//
//*****************************************************************************
#define am_hal_sysctrl_bus_write_flush()        am_hal_cachectrl_dcache_invalidate(NULL, true)

//*****************************************************************************
//
// External function definitions
//
//*****************************************************************************
//*****************************************************************************
//
//! @brief Place the core into sleep or deepsleep.
//!
//! @param bSleepDeep - False for Normal or True Deep sleep.
//!
//! This function puts the MCU to sleep or deepsleep depending on bSleepDeep.
//!
//! Valid values for bSleepDeep are:
//!
//!     AM_HAL_SYSCTRL_SLEEP_NORMAL
//!     AM_HAL_SYSCTRL_SLEEP_DEEP
//
//*****************************************************************************
extern void am_hal_sysctrl_sleep(bool bSleepDeep);
#ifdef __cplusplus
}
#endif

#endif // AM_HAL_SYSCTRL_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

