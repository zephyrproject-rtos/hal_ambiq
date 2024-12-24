//*****************************************************************************
//
//! @file am_hal_mram.c
//!
//! @brief MRAM and ROM Helper Function Access
//!
//! @addtogroup mram4 MRAM Functionality
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

#define AM_HAL_MRAM_WIPE                   0
#define AM_HAL_MRAM_PROGRAM                1


//*****************************************************************************
//
// This programs up to N words of the Main MRAM
//
//*****************************************************************************
uint32_t
am_hal_mram_main_words_program(uint32_t ui32ProgramKey, uint32_t *pui32Src,
                               uint32_t *pui32Dst, uint32_t ui32NumWords)
{
    uint32_t    ui32Status;

// #### INTERNAL BEGIN ####
    // Apollo5 helper function that efficiently performs one of two
    //  functions.  For this function we'll be using function 1.
    //  0) Writes a 32-bit pattern repeatedly.
    //     parm0: ui32ProgramKey, parm1: 0, parm2: 32-bit fill value,
    //     parm3: pui32Dst, parm4: ui32NumWords.
    //  1) Program data from a source.
    //     parm0: ui32ProgramKey, parm1: 1, parm2: pui32Src,
    //     parm3: pui32Dst, parm4: ui32NumWords.
// #### INTERNAL END ####
    //
    // Check that the pui32Dst is word aligned
    //
    if ( (uint32_t)pui32Dst & 0x3 )
    {
        return AM_HAL_MRAM_INVLD_ADDR_ALIGNMENT;
    }

    //
    // This helper function requires a word offset rather than an actual address.
    //
    pui32Dst = (uint32_t*)(((uint32_t)pui32Dst - AM_HAL_MRAM_ADDR) >> 2);

    AM_CRITICAL_BEGIN

    //
    // Enable the ROM for helper function.
    //
    am_hal_pwrctrl_rom_enable();

    ui32Status = nv_program_main2(ui32ProgramKey, AM_HAL_MRAM_PROGRAM, (uint32_t)pui32Src, (uint32_t)pui32Dst, ui32NumWords);

    //
    // Disable the ROM.
    //
    am_hal_pwrctrl_rom_disable();

    AM_CRITICAL_END

    //
    // Return the status.
    //
    if ( ui32Status == AM_HAL_MRAM_SUCCESS )
    {
        ui32Status = AM_HAL_STATUS_SUCCESS;
    }
    else
    {
        ui32Status |= AM_HAL_MRAM_ERROR;
    }

    return ui32Status;

} // am_hal_mram_main_words_program()

//*****************************************************************************
//
// This programs up to N words of the Main MRAM
//
//*****************************************************************************
uint32_t
am_hal_mram_main_program(uint32_t ui32ProgramKey, uint32_t *pui32Src,
                         uint32_t *pui32Dst, uint32_t ui32NumWords)
{
    //
    // Check for 16 byte aligned pui32Dst & ui32NumWords
    //
    if ( ((uint32_t)pui32Dst & 0xf) || (ui32NumWords & 0x3) )
    {
        return AM_HAL_MRAM_INVLD_ADDR_ALIGNMENT;
    }

    return am_hal_mram_main_words_program(ui32ProgramKey, pui32Src,
                                          pui32Dst, ui32NumWords);
} // am_hal_mram_main_program()

//*****************************************************************************
//
// This Fills up to N words of the Main MRAM
//
//*****************************************************************************
uint32_t
am_hal_mram_main_fill(uint32_t ui32ProgramKey, uint32_t ui32Value,
                      uint32_t *pui32Dst, uint32_t ui32NumWords)
{
    uint32_t    ui32Status;

    //
    // Check for 16 byte aligned pui32Dst & ui32NumWords
    //
    if ( ((uint32_t)pui32Dst & 0xf) || (ui32NumWords & 0x3) )
    {
        return AM_HAL_MRAM_INVLD_ADDR_ALIGNMENT;
    }

// #### INTERNAL BEGIN ####
    // Apollo5 helper function that efficiently performs one of two
    //  functions. For this function we'll be using function 0.
    //  0) Writes a 32-bit pattern repeatedly.
    //     parm0: ui32ProgramKey, parm1: 0, parm2: 32-bit fill value,
    //     parm3: pui32Dst, parm4: ui32NumWords.
    //  1) Program data from a source.
    //     parm0: ui32ProgramKey, parm1: 1, parm2: pui32Src,
    //     parm3: pui32Dst, parm4: ui32NumWords.
// #### INTERNAL END ####
    //
    // This helper function requires a word offset rather than an actual address.
    //
    pui32Dst = (uint32_t*)(((uint32_t)pui32Dst - AM_HAL_MRAM_ADDR) >> 2);

    AM_CRITICAL_BEGIN

    //
    // Enable the ROM for helper functions.
    //
    am_hal_pwrctrl_rom_enable();

    ui32Status = nv_program_main2(ui32ProgramKey, AM_HAL_MRAM_WIPE, (uint32_t)ui32Value, (uint32_t)pui32Dst, ui32NumWords);

    //
    // Disable the ROM.
    //
    am_hal_pwrctrl_rom_disable();

    AM_CRITICAL_END

    //
    // Return the status.
    //
    if ( ui32Status == AM_HAL_MRAM_SUCCESS )
    {
        ui32Status = AM_HAL_STATUS_SUCCESS;
    }
    else
    {
        ui32Status |= AM_HAL_MRAM_ERROR;
    }

    return ui32Status;

} // am_hal_mram_main_fill()

//*****************************************************************************
//
// Initialize MRAM for DeepSleep.
//
//*****************************************************************************
uint32_t
am_hal_mram_ds_init(void)
{

    return AM_HAL_STATUS_SUCCESS;

} // am_hal_mram_ds_init()


//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
