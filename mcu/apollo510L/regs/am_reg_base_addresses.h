//*****************************************************************************
//
//  am_reg_base_addresses.h
//! @file am_reg_base_addresses.h
//!
//! @brief Register defines for all module base addresses
//
//*****************************************************************************

//*****************************************************************************
//
// ${copyright}
//
// This is part of revision ${version} of the AmbiqSuite Development Package.
//
//*****************************************************************************
#ifndef AM_REG_BASE_ADDRESSES_H
#define AM_REG_BASE_ADDRESSES_H

#include "stdint.h"

// #### INTERNAL BEGIN ####
// Information about the memory map comes from the Confluence page Cayenne
// Specifications (Camaro)->CPU Subsystem, Apr 2024.
// #### INTERNAL END ####
//
// The following table is based on maximum memory sizing of Apollo510L,
// specific SKUs may vary.
// ****************************************************************************
// RAM and NVRAM Memory Map:
//  ITCM:                            None
//  NVM (MRAM):                      2MB        0x00400000 - 0x005FFFFF
//  MSPIx External memory:          varies      0x60000000 - 0x8FFFFFFF
//  DTCM:                            256KB      0x20000000 - 0x2003FFFF
//  System SRAM:                   1.768MB      0x20080000 - 0x2023FFFF
// ****************************************************************************

//
// DTCM address space.
//
#define DTCM_BASEADDR                           (0x20000000UL)
#define DTCM_MAX_SIZE                           (256UL * 1024UL)

//
// The Apollo5 maximum SSRAM space is 1.768MB.
//
#define SSRAM_BASEADDR                          (0x20080000)
#define SSRAM_MAX_SIZE                          (1792UL * 1024UL)

// ****************************************************************************
// MRAM Address Space
// ****************************************************************************
#define MRAM_BASEADDR                           (0x00400000UL)
#define MRAM_MAX_SIZE                           (2048UL * 1024UL)

// ****************************************************************************
// INFO base addresses
// ****************************************************************************
#define AM_REG_INFO0_BASEADDR           0x42000000
#define AM_REG_INFO1_BASEADDR           0x42002000
// #### INTERNAL BEGIN ####
//#define AM_REG_INFOC_BASEADDR           0x400C2000
// #### INTERNAL END ####
#define AM_REG_OTP_INFO0_BASEADDR       0x42004000
#define AM_REG_OTP_INFO1_BASEADDR       0x42006000
#define AM_REG_OTP_INFOC_BASEADDR       0x400C2000

//
// INFO sizes
//
#define AM_REG_INFO0_SIZE               2048
#define AM_REG_INFO1_SIZE               6144
#define AM_REG_INFOC_SIZE               8192
#define AM_REG_OTP_INFO0_SIZE           256
#define AM_REG_OTP_INFO1_SIZE           2816
#define AM_REG_OTP_INFOC_SIZE           1024


// ****************************************************************************
// MSPI aperture address ranges
// ****************************************************************************
// #### INTERNAL BEGIN ####
// Information about the MSPI aperatures comes from the Confluence page Cayenne
// Specifications (Camaro)->CPU Subsystem, Apr 2024.
// #### INTERNAL END ####
//
// Please note that the END_ADDR as used here is actually the (end address + 1).
//
#define MSPI0_XIP_BASEADDR                      (0x60000000UL)
#define MSPI1_XIP_BASEADDR                      (0x80000000UL)
#define MSPI2_XIP_BASEADDR                      (0x84000000UL)

#define MSPI0_APERTURE_START_ADDR               (0x60000000UL)
#define MSPI0_APERTURE_END_ADDR                 (0x70000000UL)
#define MSPI1_APERTURE_START_ADDR               (0x80000000UL)
#define MSPI1_APERTURE_END_ADDR                 (0x84000000UL)
#define MSPI2_APERTURE_START_ADDR               (0x84000000UL)
#define MSPI2_APERTURE_END_ADDR                 (0x8C000000UL)

#endif // AM_REG_BASE_ADDRESSES_H
