//*****************************************************************************
//
//! @file am_util_delay.c
//!
//! @brief A few useful delay functions.
//!
//! Functions for fixed delays.
//!
//! @addtogroup delay Delay Functionality
//! @ingroup utils
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

#include  <stdint.h>
#include <stdbool.h>
#include "am_mcu_apollo.h"
#include "am_util_delay.h"

//*****************************************************************************
//
// Delays for a desired amount of loops.
//
//*****************************************************************************
void
am_util_delay_cycles(uint32_t ui32Iterations)
{
    //
    // Call the BOOTROM cycle delay function
    //
#if defined(AM_PART_APOLLO4_API) || defined(AM_PART_APOLLO5_API)
    am_hal_delay_us( ui32Iterations);
#else
    am_hal_flash_delay(ui32Iterations);
#endif // AM_PART_APOLLO4_API || AM_PART_APOLLO5_API
}

//*****************************************************************************
//
// Delays for a desired amount of milliseconds.
//
//*****************************************************************************
void
am_util_delay_ms(uint32_t ui32MilliSeconds)
{
#if defined(AM_PART_APOLLO4_API) || defined(AM_PART_APOLLO5_API)
    am_hal_delay_us( ui32MilliSeconds * 1000);
#else // AM_PART_APOLLO4_API || AM_PART_APOLLO5_API
    uint32_t ui32Loops, ui32HFRC;
#if AM_APOLLO3_CLKGEN
    am_hal_clkgen_status_t sClkgenStatus;
    am_hal_clkgen_status_get(&sClkgenStatus);
    ui32HFRC = sClkgenStatus.ui32SysclkFreq;
#else // AM_APOLLO3_CLKGEN
    ui32HFRC = am_hal_clkgen_sysclk_get();
#endif // AM_APOLLO3_CLKGEN
    ui32Loops = ui32MilliSeconds * (ui32HFRC / 3000);

    //
    // Call the BOOTROM cycle delay function
    //
    am_hal_flash_delay(ui32Loops);
#endif // AM_PART_APOLLO4_API || AM_PART_APOLLO5_API
}

//*****************************************************************************
//
// Delays for a desired amount of microseconds.
//
//*****************************************************************************
void
am_util_delay_us(uint32_t ui32MicroSeconds)
{
#if defined(AM_PART_APOLLO4_API) || defined(AM_PART_APOLLO5_API)
    am_hal_delay_us( ui32MicroSeconds );
#else // AM_PART_APOLLO4_API || AM_PART_APOLLO5_API
    uint32_t ui32Loops, ui32HFRC;

#if AM_APOLLO3_CLKGEN
    am_hal_clkgen_status_t sClkgenStatus;
    am_hal_clkgen_status_get(&sClkgenStatus);
    ui32HFRC = sClkgenStatus.ui32SysclkFreq;
#else // AM_APOLLO3_CLKGEN
    ui32HFRC = am_hal_clkgen_sysclk_get();
#endif // AM_APOLLO3_CLKGEN
    ui32Loops = ui32MicroSeconds * (ui32HFRC / 3000000);

    //
    // Call the BOOTROM cycle delay function
    //
    am_hal_flash_delay(ui32Loops);
#endif // AM_PART_APOLLO4_API || AM_PART_APOLLO5_API
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

