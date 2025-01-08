//*****************************************************************************
//
//! @file am_apollo3.h
//!
//! @brief Top DTS Include for Apollo3 class devices.
//!
//! This file provides all bus addresses of an apollo device for device tree.
//!
//! @addtogroup hal
//
//! @defgroup apollo3
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
//*****************************************************************************

#ifndef AM_APOLLO3_H
#define AM_APOLLO3_H

#define APOLLO3_BROWNOUT_IRQ            0
#define APOLLO3_WDT_IRQ                 1
#define APOLLO3_RTC_IRQ                 2
#define APOLLO3_VCOMP_IRQ               3
#define APOLLO3_IOSLAVE_IRQ             4
#define APOLLO3_IOSLAVEACC_IRQ          5
#define APOLLO3_IOMSTR0_IRQ             6
#define APOLLO3_IOMSTR1_IRQ             7
#define APOLLO3_IOMSTR2_IRQ             8
#define APOLLO3_IOMSTR3_IRQ             9
#define APOLLO3_IOMSTR4_IRQ             10
#define APOLLO3_IOMSTR5_IRQ             11
#define APOLLO3_BLE_IRQ                 12
#define APOLLO3_GPIO_IRQ                13
#define APOLLO3_CTIMER_IRQ              14
#define APOLLO3_UART0_IRQ               15
#define APOLLO3_UART1_IRQ               16
#define APOLLO3_SCARD_IRQ               17
#define APOLLO3_ADC_IRQ                 18
#define APOLLO3_PDM_IRQ                 19
#define APOLLO3_MSPI0_IRQ               20
#define APOLLO3_STIMER_IRQ              22
#define APOLLO3_STIMER_CMPR0_IRQ        23
#define APOLLO3_STIMER_CMPR1_IRQ        24
#define APOLLO3_STIMER_CMPR2_IRQ        25
#define APOLLO3_STIMER_CMPR3_IRQ        26
#define APOLLO3_STIMER_CMPR4_IRQ        27
#define APOLLO3_STIMER_CMPR5_IRQ        28
#define APOLLO3_STIMER_CMPR6_IRQ        29
#define APOLLO3_STIMER_CMPR7_IRQ        30
#define APOLLO3_CLKGEN_IRQ              31
#define APOLLO3_MAX_IRQ                 32

#define APOLLO3_ADC                     50010000
#define APOLLO3_APBDMA                  40011000
#define APOLLO3_BLEIF                   5000c000
#define APOLLO3_CACHECTRL               40018000
#define APOLLO3_CLKGEN                  40004000
#define APOLLO3_CTIMER0                 40008000
#define APOLLO3_CTIMER1                 40008020
#define APOLLO3_CTIMER2                 40008040
#define APOLLO3_CTIMER3                 40008060
#define APOLLO3_CTIMER4                 40008080
#define APOLLO3_CTIMER5                 400080a0
#define APOLLO3_CTIMER6                 400080c0
#define APOLLO3_CTIMER7                 400080e0
#define APOLLO3_STIMER                  40008140
#define APOLLO3_GPIO                    40010000
#define APOLLO3_IOM0                    50004000
#define APOLLO3_IOM1                    50005000
#define APOLLO3_IOM2                    50006000
#define APOLLO3_IOM3                    50007000
#define APOLLO3_IOM4                    50008000
#define APOLLO3_IOM5                    50009000
#define APOLLO3_IOS                     50000000
#define APOLLO3_MCUCTRL                 40020000
#define APOLLO3_MSPI                    50014000
#define APOLLO3_PDM                     50011000
#define APOLLO3_PWRCTRL                 40021000
#define APOLLO3_RSTGEN                  40000000
#define APOLLO3_RTC                     40004200
#define APOLLO3_SCARD                   40080000
#define APOLLO3_SECURITY                40030000
#define APOLLO3_UART0                   4001c000
#define APOLLO3_UART1                   4001d000
#define APOLLO3_VCOMP                   4000c000
#define APOLLO3_WDT                     40024000

#define APOLLO3_ADC_BASE                0x50010000UL
#define APOLLO3_APBDMA_BASE             0x40011000UL
#define APOLLO3_BLEIF_BASE              0x5000c000UL
#define APOLLO3_CACHECTRL_BASE          0x40018000UL
#define APOLLO3_CLKGEN_BASE             0x40004000UL
#define APOLLO3_CTIMER0_BASE            0x40008000UL
#define APOLLO3_CTIMER1_BASE            0x40008020UL
#define APOLLO3_CTIMER2_BASE            0x40008040UL
#define APOLLO3_CTIMER3_BASE            0x40008060UL
#define APOLLO3_CTIMER4_BASE            0x40008080UL
#define APOLLO3_CTIMER5_BASE            0x400080a0UL
#define APOLLO3_CTIMER6_BASE            0x400080c0UL
#define APOLLO3_CTIMER7_BASE            0x400080e0UL
#define APOLLO3_STIMER_BASE             0x40008140UL
#define APOLLO3_GPIO_BASE               0x40010000UL
#define APOLLO3_IOM0_BASE               0x50004000UL
#define APOLLO3_IOM1_BASE               0x50005000UL
#define APOLLO3_IOM2_BASE               0x50006000UL
#define APOLLO3_IOM3_BASE               0x50007000UL
#define APOLLO3_IOM4_BASE               0x50008000UL
#define APOLLO3_IOM5_BASE               0x50009000UL
#define APOLLO3_IOS_BASE                0x50000000UL
#define APOLLO3_MCUCTRL_BASE            0x40020000UL
#define APOLLO3_MSPI_BASE               0x50014000UL
#define APOLLO3_PDM_BASE                0x50011000UL
#define APOLLO3_PWRCTRL_BASE            0x40021000UL
#define APOLLO3_RSTGEN_BASE             0x40000000UL
#define APOLLO3_RTC_BASE                0x40004200UL
#define APOLLO3_SCARD_BASE              0x40080000UL
#define APOLLO3_SECURITY_BASE           0x40030000UL
#define APOLLO3_UART0_BASE              0x4001c000UL
#define APOLLO3_UART1_BASE              0x4001d000UL
#define APOLLO3_VCOMP_BASE              0x4000c000UL
#define APOLLO3_WDT_BASE                0x40024000UL

#define APOLLO3_ADC_SIZE                0x294UL
#define APOLLO3_APBDMA_SIZE             0x44UL
#define APOLLO3_BLEIF_SIZE              0x414UL
#define APOLLO3_CACHECTRL_SIZE          0x60UL
#define APOLLO3_CLKGEN_SIZE             0x110UL
#define APOLLO3_CTIMER0_SIZE            0x20UL
#define APOLLO3_CTIMER1_SIZE            0x20UL
#define APOLLO3_CTIMER2_SIZE            0x20UL
#define APOLLO3_CTIMER3_SIZE            0x20UL
#define APOLLO3_CTIMER4_SIZE            0x20UL
#define APOLLO3_CTIMER5_SIZE            0x20UL
#define APOLLO3_CTIMER6_SIZE            0x20UL
#define APOLLO3_CTIMER7_SIZE            0x20UL
#define APOLLO3_STIMER_SIZE             0x1d0UL
#define APOLLO3_GPIO_SIZE               0x220UL
#define APOLLO3_IOM0_SIZE               0x414UL
#define APOLLO3_IOM1_SIZE               0x414UL
#define APOLLO3_IOM2_SIZE               0x414UL
#define APOLLO3_IOM3_SIZE               0x414UL
#define APOLLO3_IOM4_SIZE               0x414UL
#define APOLLO3_IOM5_SIZE               0x414UL
#define APOLLO3_IOS_SIZE                0x220UL
#define APOLLO3_MCUCTRL_SIZE            0x3d8UL
#define APOLLO3_MSPI_SIZE               0x2c8UL
#define APOLLO3_PDM_SIZE                0x294UL
#define APOLLO3_PWRCTRL_SIZE            0x30UL
#define APOLLO3_RSTGEN_SIZE             0x210UL
#define APOLLO3_RTC_SIZE                0x110UL
#define APOLLO3_SCARD_SIZE              0x104UL
#define APOLLO3_SECURITY_SIZE           0x90UL
#define APOLLO3_UART0_SIZE              0x48UL
#define APOLLO3_UART1_SIZE              0x48UL
#define APOLLO3_VCOMP_SIZE              0x210UL
#define APOLLO3_WDT_SIZE                0x210UL

#define APOLLO3_TCM                     10000000
#define APOLLO3_TCM_BASE                0x10000000UL
#define APOLLO3_TCM_SIZE                0x10000UL

#define APOLLO3_SRAM                    10010000
#define APOLLO3_SRAM_BASE               0x10010000UL
#define APOLLO3_SRAM_SIZE               0x50000UL

#define APOLLO3_FLASH                   c000
#define APOLLO3_FLASH_BASE              0x0000c000UL
#define APOLLO3_FLASH_SIZE              0xf4000UL

#define APOLLO3_MSPI_XIP_APERTURE       04000000
#define APOLLO3_MSPI_XIP_APERTURE_BASE  0x04000000UL
#define APOLLO3_MSPI_XIP_APERTURE_SIZE  0x04000000UL

#define APOLLO3_MSPI_XIPMM_APERTURE      51000000
#define APOLLO3_MSPI_XIPMM_APERTURE_BASE 0x51000000UL
#define APOLLO3_MSPI_XIPMM_APERTURE_SIZE 0x01000000UL

#endif // AM_APOLLO3_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
