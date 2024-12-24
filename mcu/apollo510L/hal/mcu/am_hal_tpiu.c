//*****************************************************************************
//
//! @file am_hal_tpiu.c
//!
//! @brief Support functions for the Arm TPIU module
//!
//! Provides support functions for configuring the Arm TPIU module
//!
//! @addtogroup tpiu4 TPIU - Trace Port Interface Unit
//! @ingroup apollo5b_hal
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


//*****************************************************************************
//
// Global Variables
//
//*****************************************************************************
//
// TPIU counters
//
static bool    g_bTpiu_DebugEnabled = false;
static uint8_t g_ui8TpiuEnCnt       = 0;


//*****************************************************************************
//
// Local functions
//
//*****************************************************************************
// #### INTERNAL BEGIN ####
#if 0
static bool    g_bTpiuFlush         = false;
static uint32_t
tpiu_flush(void)
{
    uint32_t ui32Status;

    if ( g_bTpiuFlush )
    {
        return AM_HAL_STATUS_SUCCESS;
    }

    //
    // Generate a flush.
    // This function should only be called prior the first programming
    // of the TPIU, see the following Arm notes.
    //
    // Section 2.10 Programming the TPIU-M for trace capture:
    // "Arm recommends that debug tools perform a flush by writing
    //  TPIU_FFCR.FOnMan = 1 whenever the TPIU-M is reprogrammed."
    // "The TPIU clears the bit to 0 when the flush completes."
    //
    // "Arm recommends that the TPIU is only reprogrammed before any data has
    //  been presented on either (AMBA Trace Bus) ATB slave port and either:
    //  - After both ATRESETn and TRESETn have been applied.
    //  - After a flush has been completed using FFCR.FOnMan.
    //  If this is not followed, reprogramming can cause either momentary or permanent
    //  data corruption that might require ATRESETn and TRESETn to be applied."
    //
    g_bTpiuFlush = true;

    TPI->FFCR |= _VAL2FLD(TPI_FFCR_FOnMan, 1);
    ui32Status = am_hal_delay_us_status_change(100000,
                                               (uint32_t)&TPI->FFCR,
                                               TPI_FFCR_FOnMan_Msk,
                                               0);

    return ui32Status;
} // tpiu_flush()
#endif // 0
// #### INTERNAL END ####

//*****************************************************************************
//
// TPIU configuration function
//
//*****************************************************************************
uint32_t
am_hal_tpiu_config(uint32_t ui32DbgTpiuClksel,  // MCUCTRL_DBGCTRL_DBGTPIUCLKSEL_HFRC_48MHz
                   uint32_t ui32FFCR,           // 0: Disable continuous formatting (EnFCont)
                   uint32_t ui32CSPSR,          // TPI_CSPSR_CWIDTH_1BIT
                   uint32_t ui32PinProtocol,    // TPI_SPPR_TXMODE_UART
                   uint32_t ui32SWOscaler)
{
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;

    AM_CRITICAL_BEGIN

    //
    // Enable the TPIU clock source in MCU control.
    //
    MCUCTRL->DBGCTRL_b.DBGTPIUCLKSEL = ui32DbgTpiuClksel;

    //
    // Perform general debug/tracing configuration.
    //
    if ( g_bTpiu_DebugEnabled == false )
    {
        am_hal_debug_enable();
        g_bTpiu_DebugEnabled = true;
    }

// #### INTERNAL BEGIN ####
    //
    // Perform a TPIU flush before reconfiguring.
    //
//  ui32Status = tpiu_flush();
// #### INTERNAL END ####
    if ( ui32Status == AM_HAL_STATUS_SUCCESS )
    {

        //
        // TPIU formatter & flush control register.
        // Disable continuous formatting (EnFCont bit1).
        //
        TPI->FFCR = ui32FFCR;

        //
        // Set the Current Parallel Port Size (note - only 1 bit can be set).
        //
        TPI->CSPSR = ui32CSPSR;


        //
        // Set the scaler value.
        //
        TPI->ACPR = _VAL2FLD(TPI_ACPR_SWOSCALER, ui32SWOscaler);

        //
        // Set the Pin Protocol.
        //
        TPI->SPPR = _VAL2FLD( TPI_SPPR_TXMODE, ui32PinProtocol);
    }

    AM_CRITICAL_END

    return ui32Status;
} // am_hal_tpiu_config()


//*****************************************************************************
//
// Enable the TPIU
//
// This function enables the Arm TPIU by setting the TPIU clock source.
//
// Arm v8-M Architecture Reference Manual, DDI0553B.y section B14.4 states
// "The TPIU is not directly affected by DEMCR.TRCENA being set to 0."
//
//*****************************************************************************
uint32_t
am_hal_tpiu_enable(uint32_t ui32DeprecatedItmBaud)
{
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;

    AM_CRITICAL_BEGIN

// #### INTERNAL BEGIN ####
//  g_bTpiuFlush = true;
// #### INTERNAL END ####
    //
    // Bump the counter on each enable call.
    //
    g_ui8TpiuEnCnt++;

    //
    // The deprecated ItmBaud argument is an ITM-specific parameter and is only
    // maintained here for backward compatibility. On entry, it should typically
    // be set to 0. Otherwise, pass it along to the ITM HAL.
    //
    if ( (ui32DeprecatedItmBaud != AM_HAL_TPIU_BAUD_DEFAULT) &&
         (ui32DeprecatedItmBaud != 0) )
    {
        am_hal_itm_parameters_set(ui32DeprecatedItmBaud);
    }

    if ( g_bTpiu_DebugEnabled == false )
    {
        //
        // Perform general debug/tracing configuration.
        //
        ui32Status = am_hal_debug_enable();
        g_bTpiu_DebugEnabled = true;
    }

    AM_CRITICAL_END

    return ui32Status;

} // am_hal_tpiu_enable()

//*****************************************************************************
//
// Disable the TPIU
//
// This function enables the Arm TPIU by disabling the TPIU clock source.
//
//*****************************************************************************
uint32_t
am_hal_tpiu_disable(void)
{
    uint32_t ui32Status;

    AM_CRITICAL_BEGIN

    if ( g_ui8TpiuEnCnt != 0 )
    {
        --g_ui8TpiuEnCnt;
    }

    if ( g_ui8TpiuEnCnt > 0 )
    {
        //
        // TPIU needs to remain enabled for now.
        //
        ui32Status = AM_HAL_STATUS_IN_USE;
    }
    else
    {
        //
        // Invalidate any previous TPIU configuration
        //
        g_bTpiu_DebugEnabled = false;

// #### INTERNAL BEGIN ####
        //
        // Perform a TPIU flush before reconfiguring.
        //
//      ui32Status = tpiu_flush();
// #### INTERNAL END ####
        //
        // Shut down the general debug/tracing configuration.
        //
        ui32Status = am_hal_debug_disable();
        if ( ui32Status == AM_HAL_STATUS_IN_USE )
        {
            ui32Status = AM_HAL_STATUS_SUCCESS;
        }
    }

    AM_CRITICAL_END

    return ui32Status;

} // am_hal_tpiu_disable()

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
