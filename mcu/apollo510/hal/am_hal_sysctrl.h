//*****************************************************************************
//
//! @file am_hal_sysctrl.h
//!
//! @brief Functions for interfacing with the M4F system control registers
//!
//! @addtogroup sysctrl4 SYSCTRL - System Control
//! @ingroup apollo510_hal
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2025, Ambiq Micro, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision release_sdk5p0p0-5f68a8286b of the AmbiqSuite Development Package.
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
// Global Variables
//
//*****************************************************************************
extern bool g_bFrcBuckAct;

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

//*****************************************************************************
//
//! @brief Control the buck state in deepsleep
//!
//! @param bFrcBuckAct - True for forcing buck active in deepsleep
//!                    - False for not forcing buck active in deepsleep
//!
//! If you want to manually force the buck stay active in deepsleep mode,
//! am_hal_sysctrl_force_buck_active_in_deepsleep must
//! be called for setting g_bAppFrcBuckAct to true before
//! calling am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP).
//! For PCM2.1 and later versions, if anyone of spotmgr and
//! am_hal_sysctrl_force_buck_active_in_deepsleep forced buck stay active, buck
//! will stay active in deepsleep.
//
//*****************************************************************************
extern void am_hal_sysctrl_force_buck_active_in_deepsleep(bool bFrcBuckAct);

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

