//*****************************************************************************
//
//! @file am_hal_mcu_interrupt.h
//!
//! @brief Helper functions supporting interrupts and NVIC operation.
//!
//! These functions may be used for NVIC-level interrupt configuration.
//!
//! @addtogroup mcu_interrupt Interrupt - ARM NVIC support functions
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
#ifndef AM_HAL_MCU_INTERRUPT_H
#define AM_HAL_MCU_INTERRUPT_H

#ifdef __cplusplus
extern "C"
{
#endif

//
//! Define the last peripheral interrupt as AM_HAL_INTERRUPT_MAX.
//! The total number of interrupts in the vector table is therefore
//! (AM_HAL_INTERRUPT_MAX + 1 + 16).
//
#define AM_HAL_INTERRUPT_MAX                (MAX_IRQn - 1)

//
//! For Apollo4 RevB, a workaround requires masking all interrupts.
//! This is accomplished by setting a priority of 0, but assumes
//! any other IRQs have a lower priority (>0).
//! This will assign a default priority for IRQs.
//
#define AM_IRQ_PRIORITY_DEFAULT             4

//*****************************************************************************
//
//! @brief Globally disable interrupt service routines
//!
//! This function prevents interrupt signals from the NVIC from triggering ISR
//! entry in the CPU. This will effectively stop incoming interrupt sources
//! from triggering their corresponding ISRs.
//!
//! @note Any external interrupt signal that occurs while the master interrupt
//! disable is active will still reach the "pending" state in the NVIC, but it
//! will not be allowed to reach the "active" state or trigger the
//! corresponding ISR. Instead, these interrupts are essentially "queued" until
//! the next time the master interrupt enable instruction is executed. At that
//! time, the interrupt handlers will be executed in order of decreasing
//! priority.
//!
//! @return 1 if interrupts were previously disabled, 0 otherwise.
//
//*****************************************************************************
extern uint32_t am_hal_interrupt_master_disable(void);

//*****************************************************************************
//
//! @brief Globally enable interrupt service routines
//!
//! This function allows interrupt signals from the NVIC to trigger ISR entry
//! in the CPU. This function must be called if interrupts are to be serviced
//! in software.
//!
//! @return 1 if interrupts were previously disabled, 0 otherwise.
//
//*****************************************************************************
extern uint32_t am_hal_interrupt_master_enable(void);

//*****************************************************************************
//
//! @brief Sets the master interrupt state based on the input.
//!
//! @param ui32Level - Desired PRIMASK value.
//!
//! This function directly writes the PRIMASK register in the ARM core. A value
//! of 1 will disable interrupts, while a value of zero will enable them.
//!
//! @note This function may be used along with am_hal_interrupt_master_disable() to
//! implement a nesting critical section. To do this, call
//! am_hal_interrupt_master_disable() to start the critical section, and save
//! its ret value. To complete the critical section, call
//! am_hal_interrupt_master_set() using the saved return value as \e
//! ui32Level. This will safely restore PRIMASK to the value it
//! contained just before the start of the critical section.
//
//
//*****************************************************************************
extern void am_hal_interrupt_master_set(uint32_t ui32Level);

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_MCU_INTERRUPT_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

