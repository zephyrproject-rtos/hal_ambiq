//*****************************************************************************
//
//! @file am_hal_itm.h
//!
//! @brief Functions for operating the instrumentation trace macrocell
//!
//! @addtogroup itm4 ITM - Instrumentation Trace Macrocell
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

#ifndef AM_HAL_ITM_H
#define AM_HAL_ITM_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! Various ITM fields and values that may not be defined in core_cm55.h
//
//*****************************************************************************
#define ITM_LAR_KEYVAL  0xC5ACCE55

//*****************************************************************************
//
// Sync Packet Defines
//
//*****************************************************************************
#define AM_HAL_ITM_SYNC_REG             23
#define AM_HAL_ITM_SYNC_VAL             0xF8F8F8F8

//*****************************************************************************
//
// PrintF Setup
//
//*****************************************************************************
#define AM_HAL_ITM_PRINT_NUM_BYTES      1
#define AM_HAL_ITM_PRINT_NUM_REGS       1
extern uint32_t am_hal_itm_print_registers[AM_HAL_ITM_PRINT_NUM_REGS];

//*****************************************************************************
//
// External function definitions
//
//*****************************************************************************
//*****************************************************************************
//
// Sets certain ITM parameters if something other than defaults are required.
//
//! @param ui32ItmBaud - Set to one of the following:
//!         AM_HAL_TPIU_BAUD_DEFAULT    (1M)
//!         AM_HAL_TPIU_BAUD_2M
//!         AM_HAL_TPIU_BAUD_1M
//!         AM_HAL_TPIU_BAUD_115200
//!         AM_HAL_TPIU_BAUD_230400
//!         AM_HAL_TPIU_BAUD_460800
//!         AM_HAL_TPIU_BAUD_500000
//!         AM_HAL_TPIU_BAUD_57600
//!
//! @note The ui32ItmBaud parameter was formerly the parameter (and now
//!       deprecated) to the TPIU function, am_hal_tpiu_enable().
//
//*****************************************************************************
extern uint32_t am_hal_itm_parameters_set(uint32_t ui32ItmBaud);

//*****************************************************************************
//
//! @brief Enables the ITM
//!
//! This function enables the ARM ITM.
//
//*****************************************************************************
extern uint32_t am_hal_itm_enable(void);

//*****************************************************************************
//
//! @brief Disables the ITM
//!
//! This function completely disables the ARM ITM.
//
//*****************************************************************************
extern uint32_t am_hal_itm_disable(void);

//*****************************************************************************
//
//! @brief Wait until the ITM/TPIU pipeline is clear.
//!
//! @note This function is not a status check. Instead, it waits for the
//! SWO/ITM/TPIU print pipeline to be flushed, which can take significant
//! time to complete or in some cases, to timeout.
//!
//! @return AM_HAL_STATUS_SUCCESS = ITM and TPIU pipelines are flushed.
//
//*****************************************************************************
extern uint32_t am_hal_itm_tpiu_pipeline_flush(void);

//*****************************************************************************
//
//! @brief Checks if itm is busy and provides a delay to flush the fifo
//!
//! This function determines whether ITM is busy.
//!
//! @return True if not busy, false if busy (timed out or other error).
//
//*****************************************************************************
extern bool am_hal_itm_not_busy(void);

//*****************************************************************************
//
//! @brief Sends a Sync Packet.
//!
//! Sends a sync packet. This can be useful for external software should it
//! become out of sync with the ITM stream.
//
//*****************************************************************************
extern void am_hal_itm_sync_send(void);

//*****************************************************************************
//
//! @brief Enables tracing on a given set of ITM ports
//!
//! @param ui8portNum - Set ports to be enabled
//!
//! Enables tracing on the ports referred to by \e ui8portNum by writing the
//! associated bit in the Trace Privilege Register in the ITM. The value for
//! ui8portNum should be the logical OR one or more of the following values:
//!
//! \e ITM_PRIVMASK_0_7 - enable ports 0 through 7
//! \e ITM_PRIVMASK_8_15 - enable ports 8 through 15
//! \e ITM_PRIVMASK_16_23 - enable ports 16 through 23
//! \e ITM_PRIVMASK_24_31 - enable ports 24 through 31
//
//*****************************************************************************
extern void am_hal_itm_trace_port_enable(uint8_t ui8portNum);

//*****************************************************************************
//
//! @brief Disable tracing on the given ITM stimulus port.
//!
//! @param ui8portNum
//!
//! Disables tracing on the ports referred to by \e ui8portNum by writing the
//! associated bit in the Trace Privilege Register in the ITM. The value for
//! ui8portNum should be the logical OR one or more of the following values:
//!
//! \e ITM_PRIVMASK_0_7 - disable ports 0 through 7
//! \e ITM_PRIVMASK_8_15 - disable ports 8 through 15
//! \e ITM_PRIVMASK_16_23 - disable ports 16 through 23
//! \e ITM_PRIVMASK_24_31 - disable ports 24 through 31
//
//*****************************************************************************
extern void am_hal_itm_trace_port_disable(uint8_t ui8portNum);

//*****************************************************************************
//
//! @brief Poll the given ITM stimulus register until not busy.
//!
//! @param ui32StimReg - stimulus register
//!
//! @return true if not busy, false if busy (timed out or other error).
//
//*****************************************************************************
extern bool am_hal_itm_stimulus_not_busy(uint32_t ui32StimReg);

//*****************************************************************************
//
//! @brief Writes a 32-bit value to the given ITM stimulus register.
//!
//! @param ui32StimReg - stimulus register
//! @param ui32Value - value to be written.
//!
//! Write a word to the desired stimulus register.
//
//*****************************************************************************
extern void am_hal_itm_stimulus_reg_word_write(uint32_t ui32StimReg,
                                               uint32_t ui32Value);

//*****************************************************************************
//
//! @brief Writes a short to the given ITM stimulus register.
//!
//! @param ui32StimReg - stimulus register
//! @param ui16Value - short to be written.
//!
//! Write a short to the desired stimulus register.
//
//*****************************************************************************
extern void am_hal_itm_stimulus_reg_short_write(uint32_t ui32StimReg,
                                                uint16_t ui16Value);

//*****************************************************************************
//
//! @brief Writes a byte to the given ITM stimulus register.
//!
//! @param ui32StimReg - stimulus register
//! @param ui8Value - byte to be written.
//!
//! Write a byte to the desired stimulus register.
//
//*****************************************************************************
extern void am_hal_itm_stimulus_reg_byte_write(uint32_t ui32StimReg,
                                               uint8_t ui8Value);
//*****************************************************************************
//
//! @brief Poll the print stimulus registers until not busy.
//!
//! @return true if not busy, false if busy (timed out or other error).
//
//*****************************************************************************
extern bool am_hal_itm_print_not_busy(void);

//*****************************************************************************
//
//! @brief Prints a char string out of the ITM.
//!
//! @param pcString pointer to the character sting
//!
//! This function prints a sting out of the ITM.
//
//*****************************************************************************
extern void am_hal_itm_print(char *pcString);

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_ITM_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

