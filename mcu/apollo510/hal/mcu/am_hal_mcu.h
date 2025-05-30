//*****************************************************************************
//
//! @file am_hal_mcu.h
//!
//! @brief Functions for accessing and configuring MCU specific HAL modules
//!
//! @addtogroup mcu MCU Hal Modules
//! @ingroup apollo510_hal
//! @{

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

#ifndef AM_HAL_MCU_H
#define AM_HAL_MCU_H

#include "hal/mcu/am_hal_mcu_sysctrl.h"
#include "hal/mcu/am_hal_mcu_interrupt.h"

#define am_hal_get_core_id()    (DSP_MUTEX0_MUTEX0_CPU)

#ifdef __IAR_SYSTEMS_ICC__
#define am_count_num_leading_zeros(n)                     __CLZ(n)
#else
#define am_count_num_leading_zeros(n)                     __builtin_clz(n)
#endif

//
// The Arm6 compiler defines both GNUC and ARMCC_VERSION, so check Arm first.
// Though not necessary, Arm5 and Arm6 can be differentiated with:
// #if (defined (__ARMCC_VERSION)) && (__ARMCC_VERSION >= 6000000)
//
#if (defined (__ARMCC_VERSION))
#define COMPILER_VERSION                    ("ARMCC " STRINGIZE_VAL(__ARMCC_VERSION))
#elif defined(__GNUC__)
#define COMPILER_VERSION                    ("GCC " __VERSION__)
#elif defined(__IAR_SYSTEMS_ICC__)
#define COMPILER_VERSION                    __VERSION__
#else
#error "Unknown Compiler"
#endif

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus
}
#endif

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
#endif // AM_HAL_MCU_H

