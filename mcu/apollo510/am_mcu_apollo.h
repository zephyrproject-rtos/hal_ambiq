//*****************************************************************************
//
//! @file am_mcu_apollo.h
//!
//! @brief Top Include for Apollo510 class devices.
//!
//! This file provides all the includes necessary for an apollo device.
//!
//! @addtogroup hal mcu
//
//! @defgroup apollo510_hal apollo510
//! @ingroup hal
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

#ifndef AM_MCU_APOLLO_H
#define AM_MCU_APOLLO_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! AM_PART_APOLLO5_API indicates that this device uses the Apollo5 API.
//
//*****************************************************************************
#define AM_PART_APOLLO510
#define AM_PART_APOLLO5_API     1

//*****************************************************************************
//
//! Define AM_CMSIS_REGS to indicate that CMSIS registers are supported.
//
//*****************************************************************************
#define AM_CMSIS_REGS           1

//*****************************************************************************
//
// C99
//
//*****************************************************************************
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

//*****************************************************************************
//
// Apollo CMSIS peripheral registers
//
//*****************************************************************************
#include <arm_cmse.h>
#include "apollo510.h"

//*****************************************************************************
//
// Global HAL
//
//*****************************************************************************
//
// Define this macro to disable and remove parameter validation in functions
// throughout the HAL.
//
//#define AM_HAL_DISABLE_API_VALIDATION

//*****************************************************************************
//
// Registers
//
//*****************************************************************************
#include "regs/am_reg_base_addresses.h"
#include "regs/am_reg_macros.h"
#include "regs/am_reg.h"
#include "regs/am_reg_jedec.h"

//*****************************************************************************
//
// HAL
//
//*****************************************************************************
#include "hal/am_hal_global.h"
#include "hal/am_hal_pin.h"
#include "hal/am_hal_status.h"
#include "hal/am_hal_sysctrl.h"

//
// HAL MCU includes
//
#include "hal/mcu/am_hal_bootrom_helper.h"
#include "hal/mcu/am_hal_cachectrl.h"
#include "hal/mcu/am_hal_clkgen.h"
#include "hal/mcu/am_hal_syspll.h"
#include "hal/am_hal_clkmgr.h"
#include "hal/mcu/am_hal_card_host.h"
#include "hal/mcu/am_hal_card.h"
#include "hal/mcu/am_hal_cmdq.h"
#include "hal/mcu/am_hal_debug.h"
#include "hal/mcu/am_hal_dsi.h"
#include "hal/mcu/am_hal_iom.h"
#include "hal/mcu/am_hal_ios.h"
#include "hal/mcu/am_hal_itm.h"
#include "hal/mcu/am_hal_mcu.h"
#include "hal/mcu/am_hal_mcuctrl.h"
#include "hal/mcu/am_hal_mcu_sysctrl.h"
#include "hal/mcu/am_hal_mpu.h"
#include "hal/mcu/am_hal_mram.h"
#include "hal/mcu/am_hal_mram_recovery.h"
#include "hal/mcu/am_hal_mspi.h"
#include "hal/mcu/am_hal_reset.h"
#include "hal/mcu/am_hal_rtc.h"
#include "hal/mcu/am_hal_sdhc.h"
#include "hal/mcu/am_hal_secure_ota.h"
#include "hal/mcu/am_hal_systick.h"
#include "hal/mcu/am_hal_tpiu.h"
#include "hal/mcu/am_hal_uart.h"
#include "hal/mcu/am_hal_uart_stream.h"

//
// HAL common includes
//
#include "hal/am_hal_access.h"
#include "hal/am_hal_adc.h"
#include "hal/am_hal_audadc.h"
#include "hal/am_hal_dcu.h"
#include "hal/am_hal_gpio.h"
#include "hal/am_hal_i2s.h"
#include "hal/am_hal_info.h"
#include "hal/am_hal_infoc.h"
#include "hal/am_hal_pdm.h"
#include "hal/am_hal_pwrctrl.h"
#include "hal/am_hal_spotmgr.h"
#include "hal/am_hal_queue.h"
#include "hal/am_hal_security.h"
#include "hal/am_hal_stimer.h"
#include "hal/am_hal_timer.h"
#include "hal/am_hal_usb.h"
#include "hal/am_hal_usbcharger.h"
#include "hal/am_hal_utils.h"
#include "hal/am_hal_wdt.h"

//
// INFO includes
//
#include "regs/am_mcu_apollo510_mraminfo0.h"
#include "regs/am_mcu_apollo510_mraminfo1.h"
#include "regs/am_mcu_apollo510_otpinfo0.h"
#include "regs/am_mcu_apollo510_otpinfo1.h"
#include "regs/am_mcu_apollo510_otpinfoc.h"

#ifdef __cplusplus
}
#endif

#endif // AM_MCU_APOLLO_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
