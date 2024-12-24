//*****************************************************************************
//
//! @file am_hal_debug.h
//!
//! @brief Functions for general debug operations.
//!
//! @addtogroup debug
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

#ifndef AM_HAL_DEBUG_H
#define AM_HAL_DEBUG_H

#ifdef __cplusplus
extern "C"
{
#endif


//*****************************************************************************
//
// Function prototypes
//
//*****************************************************************************

//*****************************************************************************
//
//! am_hal_debug_enable()
//! Perform steps necessary for general enabling for debug.
//
//*****************************************************************************
extern uint32_t am_hal_debug_enable(void);


//*****************************************************************************
//
//! am_hal_debug_disable()
//!Perform steps necessary to disable from debug.
//
//*****************************************************************************
extern uint32_t am_hal_debug_disable(void);


//*****************************************************************************
//
//! Debug power.
//! This function will power up the debug domain if not already powered.
//! It will also power down the domain if it were forced to power up.
//!
//! @param  bPowerUp - If true, determines whether the debug domain is already
//!                    powered up. If not, will power it up.
//!                    If false, will power down if it was initially powered
//!                    down and subsequently forced to power up.
//
//*****************************************************************************
extern uint32_t am_hal_debug_power(bool bPowerup);


//*****************************************************************************
//
//! Enable debug tracing.
//!
//! This function enables tracing, e.g. DEMCR->TRCENA.
//! TRCENA is required when using:
//!     DWT (Data Watchpoint and Trace)
//!     ITM (Instrumentation Trace Macrocell)
//!     PMU (Performance Monitoring Unit)
//! Arm recommends TRCENA when using:
//!     ETM (Embedded Trace Macrocell)- See section D1.2.37 DEMCR of Arm v8-M
//!                                     Architecture Reference Manual DDI0553B.y
//
//*****************************************************************************
extern uint32_t am_hal_debug_trace_enable(void);

//*****************************************************************************
//
//! Disable debug tracing.
//!
//! This function disables tracing, e.g. DEMCR->TRCENA.
//
//*****************************************************************************
extern uint32_t am_hal_debug_trace_disable(void);

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_DEBUG_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

