//*****************************************************************************
//
//! @file am_hal_utils.c
//!
//! @brief HAL Utility Functions
//!
//! @addtogroup utils4 Utils - HAL Utility Functions
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
//! Mode - the result of a change request.
//
//*****************************************************************************
typedef enum
{
    AM_HAL_BURST_MODE,
    AM_HAL_NORMAL_MODE,
} am_hal_burst_mode_e;

//*****************************************************************************
//
//! @brief Return current burst mode state
//!
//! Implemented as a macro, this function returns the current burst mode state.
//!     AM_HAL_BURST_MODE
//!     AM_HAL_NORMAL_MODE
//
//*****************************************************************************
#define am_hal_burst_mode_status()  AM_HAL_NORMAL_MODE

//*****************************************************************************
//
//! @brief Given an integer number of microseconds, convert to a value representing
//! the number of cycles that will provide that amount
//! of delay.  This macro is designed to take into account some of the call
//! overhead and latencies.
//! The BOOTROM_CYCLES_US macro assumes:
//!  - Burst or normal mode operation.
//!  - If cache is not enabled, use BOOTROM_CYCLES_US_NOCACHE() instead.
//!
//! @name BootROM_CYCLES
//! @{
//
//*****************************************************************************
#define CYCLESPERITER                 ((float)AM_HAL_CLKGEN_FREQ_MAX_MHZ / 3)
#define BOOTROM_CYCLES_US(n)          ((uint32_t)((n) * CYCLESPERITER) + 0)
#define BOOTROM_CYCLES_US_NOCACHE(n)  ( (n == 0) ? 0 : (uint32_t)(n * CYCLESPERITER) - 5)
//! @}

//*****************************************************************************
//
//!  @brief Delay function with accurate cycle.
//!
//!  @param ui32Iterations The number of iterations.
//!
//!  This function is located in DTCM, which means instruction fetch bypasses
//!  D-Cache. Each iteration spends 3 CPU cycles.
//
//*****************************************************************************
#if defined(__IAR_SYSTEMS_ICC__)
__ramfunc void
#else
void __attribute__((naked, section(".ramfunc")))
#endif
br_util_delay_cycles(uint32_t ui32Cycles)
{
    __asm
    (
        "SUBS    R0, #1\n"
        "BNE     br_util_delay_cycles\n"
        "BX      LR\n"
    );
}

//*****************************************************************************
//
// Use the bootrom to implement a spin loop.
//
//*****************************************************************************
void
am_hal_delay_us(uint32_t ui32us)
{
// #### INTERNAL BEGIN ####
#ifdef APOLLO5_FPGA
    register uint32_t ui32Iterations = 1;

    //
    // Compensate for about 2us of overhead.
    //
    if (ui32us >= 2)
    {
        ui32Iterations = BOOTROM_CYCLES_US(ui32us - 2);
    }

// Some notes about FPGA target speeds.
// - The SOF designated as 24MHz.
// - HFRC can be measured on designated pins by doing 2 things:
//   1. Configuring CLKGEN->CLKOUT with CLKGEN_CLKOUT_CKSEL_HFRC and
//      CLKGEN_CLKOUT_CKEN_EN.
//   2. Configuring the GPIO with FNCSEL=CLKOUT.
    if ( g_ui32FPGAfreqMHz == 0 )
    {
        g_ui32FPGAfreqMHz = APOLLO5_FPGA;
    }

    //
    // Check for LP (96MHz) vs. HP (250MHz) mode and create the adjustment accordingly.
    //
    if (PWRCTRL->MCUPERFREQ_b.MCUPERFSTATUS == AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE1)
    {
        //The current apollo5b fpga hp mode is only 20Mhz
        if ( ui32us >= 6 )
        {
            ui32Iterations = BOOTROM_CYCLES_US(ui32us - 6);
            ui32Iterations = (uint32_t)(ui32Iterations * 1.0f * AM_HAL_CLKGEN_FREQ_HP250_HZ / AM_HAL_CLKGEN_FREQ_MAX_HZ);
        }
        else if ( ui32us > 0 )
        {
            ui32Iterations = 1;
        }
        else
        {
            return;
        }
    }
    else
    {
        //
        // Compensate for specific frequency FPGA builds.
        //
        if ( g_ui32FPGAfreqMHz == 48 )
        {
            if ( ui32us >= 3 )
            {
                ui32Iterations = BOOTROM_CYCLES_US(ui32us - 2);
            }
            else if ( ui32us > 0 )
            {
                ui32Iterations = 1;
            }
            else
            {
                return;
            }
        }
        else if ( g_ui32FPGAfreqMHz == 24 )
        {
            if ( ui32us >= 3 )
            {
                ui32Iterations = BOOTROM_CYCLES_US(ui32us - 2);
            }
            else if ( ui32us > 0 )
            {
                ui32Iterations = 1;
            }
            else
            {
                return;
            }
        }
        else if ( g_ui32FPGAfreqMHz == 12 )
        {
            if ( ui32us >= 9 )
            {
                //
                // Compensate for about 2us of overhead.
                //
                ui32Iterations = BOOTROM_CYCLES_US(ui32us - 9);
            }
            else if ( ui32us > 0 )
            {
                ui32Iterations = 1;
            }
            else
            {
                return;
            }
        }
        else if ( g_ui32FPGAfreqMHz == 8 )
        {
            if ( ui32us >= 9 )
            {
                //
                // Compensate for about 2us of overhead.
                //
                ui32Iterations = BOOTROM_CYCLES_US(ui32us - 9);
            }
            else if ( ui32us > 0 )
            {
                ui32Iterations = 1;
            }
            else
            {
                return;
            }
        }
        else if ( g_ui32FPGAfreqMHz == 6 )
        {
            if ( ui32us >= 8 )
            {
                //
                // Compensate for about 2us of overhead.
                //
                ui32Iterations = BOOTROM_CYCLES_US(ui32us - 8);
            }
            else if ( ui32us > 0 )
            {
                ui32Iterations = 1;
            }
            else
            {
                return;
            }
        }
    }

    if ( ui32Iterations == 0 )
    {
        return;
    }

    br_util_delay_cycles(ui32Iterations);
#else
// #### INTERNAL END ####

    register uint32_t ui32Iterations = BOOTROM_CYCLES_US(ui32us);
    register uint32_t ui32CycleCntAdj;

    //
    // Check for LP (96MHz) vs. HP (192MHz/250MHz) mode and create the adjustment accordingly.
    //
    if (PWRCTRL->MCUPERFREQ_b.MCUPERFSTATUS == AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE1)
    {
        //
        // Use float instead of uint32_t, otherwise saturation may occur.
        //
        ui32Iterations = (uint32_t)(ui32Iterations * 1.0f * AM_HAL_CLKGEN_FREQ_HP250_MHZ / AM_HAL_CLKGEN_FREQ_MAX_MHZ);

        //
        // There's an additional shift to account for.
        //
        ui32CycleCntAdj = ((13 * AM_HAL_CLKGEN_FREQ_HP250_MHZ / AM_HAL_CLKGEN_FREQ_MAX_MHZ) + 40) / 3;
    }
    else
    {
        ui32CycleCntAdj = ((13 * 1) + 32) / 3;
    }

    //
    // Allow for the overhead of the burst-mode check and these comparisons
    // by eliminating an appropriate number of iterations.
    //

    if ( ui32Iterations > ui32CycleCntAdj )
    {
        ui32Iterations -= ui32CycleCntAdj;
        br_util_delay_cycles(ui32Iterations);
    }

// #### INTERNAL BEGIN ####
#endif // APOLLO5_FPGA
// #### INTERNAL END ####
} // am_hal_delay_us()

//*****************************************************************************
//
// Delays for a desired amount of cycles while also waiting for a
// status to change a value.
//
//*****************************************************************************
uint32_t
am_hal_delay_us_status_change(uint32_t ui32usMaxDelay, uint32_t ui32Address,
                              uint32_t ui32Mask, uint32_t ui32Value)
{
    while ( 1 )
    {
        //
        // Check the status
        //
        if ( ( AM_REGVAL(ui32Address) & ui32Mask ) == ui32Value )
        {
            return AM_HAL_STATUS_SUCCESS;
        }

        if (ui32usMaxDelay--)
        {
            //
            // Call the ROM helper cycle function to delay for about 1 microsecond.
            //
            am_hal_delay_us(1);
        }
        else
        {
            break;
        }
    }

    return AM_HAL_STATUS_TIMEOUT;

} // am_hal_delay_us_status_change()

//*****************************************************************************
//
// Delays for a desired amount of cycles while also waiting for a
// status to equal OR not-equal to a value.
//
//*****************************************************************************
uint32_t
am_hal_delay_us_status_check(uint32_t ui32usMaxDelay, uint32_t ui32Address,
                             uint32_t ui32Mask, uint32_t ui32Value,
                             bool bIsEqual)
{
    while ( 1 )
    {
        //
        // Check the status
        //
        if ( bIsEqual )
        {
            if ( ( AM_REGVAL(ui32Address) & ui32Mask ) == ui32Value )
            {
                return AM_HAL_STATUS_SUCCESS;
            }
        }
        else
        {
            if ( ( AM_REGVAL(ui32Address) & ui32Mask ) != ui32Value )
            {
                return AM_HAL_STATUS_SUCCESS;
            }
        }

        if ( ui32usMaxDelay-- )
        {
            //
            // Call the ROM helper cycle function to delay for about 1 microsecond.
            //
            am_hal_delay_us(1);
        }
        else
        {
            break;
        }
    }

    return AM_HAL_STATUS_TIMEOUT;

} // am_hal_delay_us_status_check()

//*****************************************************************************
//
//!  @brief Read a word
//!
//!  @param pAddr The address to read.
//!
//!  This function is located in DTCM, which means instruction fetch bypasses
//!  D-Cache.
//
//*****************************************************************************
#if defined(__IAR_SYSTEMS_ICC__)
#pragma diag_suppress = Pe940   // Suppress IAR compiler warning about missing
                                // return statement on a non-void function
__ramfunc uint32_t
#else
uint32_t __attribute__((naked, section(".dtcm_text")))
#endif
internal_hal_read_word(uint32_t *pAddr)
{
    __asm
    (
        "LDR     r0,[r0,#0]\n"
        "BX      LR\n"
    );
}
#if defined(__IAR_SYSTEMS_ICC__)
#pragma diag_default = Pe940    // Restore IAR compiler warning
#endif

//*****************************************************************************
//
//!  @brief Read words
//!
//!  @param pSrcAddr The starting address to read from.
//!  @param pDstAddr The starting address to write to.
//!  @param numWords The number of words to read.
//!
//!  This function is located in DTCM, which means instruction fetch bypasses
//!  D-Cache.
//
//*****************************************************************************
#if defined(__IAR_SYSTEMS_ICC__)
__ramfunc void
#else
void __attribute__((naked, section(".dtcm_text")))
#endif
internal_hal_read_words(uint32_t *pSrcAddr, uint32_t *pDstAddr, uint32_t numWords)
{
    __asm
    (
        "LDR     r3,[r0], #4\n"
        "STR     r3,[r1], #4\n"
        "SUBS    r2, #1\n"
        "BNE     internal_hal_read_words\n"
        "BX      LR\n"
    );
}

//*****************************************************************************
//
//!  @brief Read words
//!
//!  @param pSrcAddr The starting address to read from.
//!  @param pDstAddr The starting address to write to.
//!  @param numWords The number of words to read.
//!
//!  This function is located in DTCM, which means instruction fetch bypasses
//!  D-Cache.
//
//*****************************************************************************
void am_hal_read_words(uint32_t *pSrcAddr, uint32_t *pDstAddr, uint32_t numWords)
{
    internal_hal_read_words(pSrcAddr, pDstAddr, numWords);
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
