//*****************************************************************************
//
//! @file am_hal_bootrom_helper.c
//!
//! @brief BootROM Helper Function Table
//!
//! @addtogroup bootrom4 Bootrom Functionality
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

#include <stdint.h>
#include <stdbool.h>
#include "am_mcu_apollo.h"
#include "am_hal_bootrom_helper.h"

//*****************************************************************************
//
//  Global data
//
// ****************************************************************************
//
//! Bootrom helper function look-up table.
//
const am_hal_bootrom_helper_t g_am_hal_bootrom_helper =
{
         ((int  (*)(uint32_t, uint32_t *, uint32_t, uint32_t))                      (0x0200ff0c + 1)),  // nv_program_info_area
         ((int  (*)(uint32_t, uint32_t, uint32_t, uint32_t, uint32_t))              (0x0200ff20 + 1)),  // nv_program_main2
         ((uint32_t (*)(uint32_t *))                                                (0x0200ff28 + 1)),  // br_util_read_word
         ((void (*)(uint32_t *, uint32_t))                                          (0x0200ff2c + 1)),  // br_util_write_word
         ((bool (*) (void *, uint32_t, uint32_t))                                   (0x0200ff40 + 1)),  // valid_address_range
         ((void (*)(uint32_t ))                                                     (0x0200ff50 + 1)),  // br_util_delay_cycles
         ((int  (*)(uint32_t, uint32_t*, uint32_t, uint32_t))                       (0x0200ff54 + 1)),  // otp_program_info_area
         ((uint32_t (*)(void))                                                      (0x0200ff5c + 1)),  // br_util_rom_version
};

// *****************************************************************************
//
// Program INFO0 (customer).
//
// *****************************************************************************
int
nv_program_info_area(uint32_t  value,
                     uint32_t *pSrc,
                     uint32_t  Offset,
                     uint32_t  NumberOfWords)
{
    int iRet = g_am_hal_bootrom_helper.nv_program_info_area(value, pSrc, Offset, NumberOfWords);

    return iRet;
}

// *****************************************************************************
//
// Program main NVRAM.
//
// *****************************************************************************
int
nv_program_main2(uint32_t  value,
                 uint32_t  Program_nWipe,
                 uint32_t  Addr_WipeData,
                 uint32_t  WordOffset,
                 uint32_t  NumberOfWords)
{
    int iRet = g_am_hal_bootrom_helper.nv_program_main2(value, Program_nWipe, Addr_WipeData, WordOffset, NumberOfWords);

    return iRet;
}
//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
