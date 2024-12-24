//*****************************************************************************
//
//! @file am_hal_reset.c
//!
//! @brief Hardware abstraction layer for the Reset Generator module.
//!
//! @addtogroup rstgen2 Reset - Reset Generator (RSTGEN)
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

#include "am_mcu_apollo.h"

uint32_t gAmHalResetStatus = 0;

//*****************************************************************************
//
//  am_hal_reset_enable()
//  Enable and configure the Reset controller.
//
//*****************************************************************************
uint32_t
am_hal_reset_configure(am_hal_reset_configure_e eConfigure)
{
    uint32_t ui32Val;
    bool     bEnable = false;

    switch ( eConfigure )
    {
// #### INTERNAL BEGIN ####
#if 0   // FALCSW-427 8/1/22 Remove ability to set BODHREN
        case AM_HAL_RESET_BROWNOUT_HIGH_ENABLE:
            bEnable = true;
            ui32Val = RSTGEN_CFG_BODHREN_Msk;
            break;
#endif
// #### INTERNAL END ####
        case AM_HAL_RESET_BROWNOUT_HIGH_ENABLE:
            //
            // Apollo5 operates at 1.8v/1.9v, thus selecting
            // AM_HAL_RESET_BROWNOUT_HIGH_ENABLE is invalid.
            //
            return AM_HAL_STATUS_INVALID_ARG;

        case AM_HAL_RESET_WDT_RESET_ENABLE:
            bEnable = true;
            ui32Val = RSTGEN_CFG_WDREN_Msk;
            break;

        case AM_HAL_RESET_BROWNOUT_HIGH_DISABLE:
            bEnable = false;
            ui32Val = RSTGEN_CFG_BODHREN_Msk;
            break;

        case AM_HAL_RESET_WDT_RESET_DISABLE:
            bEnable = false;
            ui32Val = RSTGEN_CFG_WDREN_Msk;
            break;

        case AM_HAL_RESET_SECURE_WDT_ENABLE:
            bEnable = true;
            ui32Val = RSTGEN_CFG_SECUREWDREN_Msk;
            break;

        case AM_HAL_RESET_SECURE_WDT_DISABLE:
            bEnable = false;
            ui32Val = RSTGEN_CFG_SECUREWDREN_Msk;
            break;

        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }

    AM_CRITICAL_BEGIN
    if ( bEnable )
    {
        RSTGEN->CFG |= ui32Val;
    }
    else
    {
        RSTGEN->CFG &= ~ui32Val;
    }
    AM_CRITICAL_END

    //
    // Return success status.
    //
    return AM_HAL_STATUS_SUCCESS;

} // am_hal_reset_configure()

//*****************************************************************************
//
//  am_hal_reset_control()
//  Perform various reset functions including assertion of software resets.
//
//*****************************************************************************
uint32_t
am_hal_reset_control(am_hal_reset_control_e eControl, void *pArgs)
{
    switch ( eControl )
    {
        case AM_HAL_RESET_CONTROL_SWPOR:
            //
            // Perform a Power On Reset level reset.
            // Write the POR key to the software POR register.
            //
            RSTGEN->SWPOR =
                   _VAL2FLD(RSTGEN_SWPOR_SWPORKEY, RSTGEN_SWPOR_SWPORKEY_KEYVALUE);
            // Spin till reset happens.
            while (1) ;
            break;

        case AM_HAL_RESET_CONTROL_SWPOI:
            //
            // Perform a Power On Initialization level reset.
            // Write the POI key to the software POI register.
            //
            RSTGEN->SWPOI =
                _VAL2FLD(RSTGEN_SWPOI_SWPOIKEY, RSTGEN_SWPOI_SWPOIKEY_KEYVALUE);
            // Spin till reset happens.
            while (1) ;
            break;

        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Return success status.
    //
    return AM_HAL_STATUS_SUCCESS;

} // am_hal_reset_control()

//*****************************************************************************
//
//  am_hal_reset_status_get()
//  Return status of the reset generator.
//  Application MUST call this API at least once before going to deepsleep
//  Otherwise this API will not provide correct reset status
//
//*****************************************************************************
uint32_t
am_hal_reset_status_get(am_hal_reset_status_t *psStatus)
{
    if ( psStatus == NULL )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Retrieve the reset generator status bits
    // Need to read the status only the very first time
    //
    if ( !gAmHalResetStatus )
    {
        gAmHalResetStatus = RSTGEN->STAT;
// #### INTERNAL BEGIN ####
        if (MCUCTRL->BOOTLOADER_b.SECBOOTFEATURE != MCUCTRL_BOOTLOADER_SECBOOTFEATURE_ENABLED)
        {
            // Non-SBL part
            gAmHalResetStatus = AM_REGVAL(0x4000000C);
        }
// #### INTERNAL END ####
    }
    gAmHalResetStatus &= AM_HAL_RESET_STATUS_MASK;

    psStatus->eStatus           = (am_hal_reset_status_e)gAmHalResetStatus;
    psStatus->bEXTStat          = gAmHalResetStatus & AM_HAL_RESET_STATUS_EXTERNAL;
    psStatus->bPOAStat          = gAmHalResetStatus & AM_HAL_RESET_STATUS_POA;
    psStatus->bBODStat          = gAmHalResetStatus & AM_HAL_RESET_STATUS_BOD;
    psStatus->bSWPORStat        = gAmHalResetStatus & AM_HAL_RESET_STATUS_SWPOR;
    psStatus->bSWPOIStat        = gAmHalResetStatus & AM_HAL_RESET_STATUS_SWPOI;
    psStatus->bDBGRStat         = gAmHalResetStatus & AM_HAL_RESET_STATUS_DEBUGGER;
    psStatus->bWDTStat          = gAmHalResetStatus & AM_HAL_RESET_STATUS_WDT;
    psStatus->bBOUnregStat      = gAmHalResetStatus & AM_HAL_RESET_STATUS_BOUNREG;
    psStatus->bBOCOREStat       = gAmHalResetStatus & AM_HAL_RESET_STATUS_BOCORE;
    psStatus->bBOMEMStat        = gAmHalResetStatus & AM_HAL_RESET_STATUS_BOMEM;
    psStatus->bBOHPMEMStat      = gAmHalResetStatus & AM_HAL_RESET_STATUS_BOHPMEM;
    psStatus->bAIRCRStat        = gAmHalResetStatus & AM_HAL_RESET_STATUS_AIRCR;
    psStatus->bSecureWdtRst     = gAmHalResetStatus & AM_HAL_RESET_STATUS_SECURE_WDR;
    psStatus->bCM4TimeoutStat   = gAmHalResetStatus & AM_HAL_RESET_STATUS_CM4_TIMEOUT;

    //
    // Return status.
    // If the Reset Status is 0 - this implies application did not capture the snapshot
    // before deepsleep, and hence the result is invalid
    //
    return (gAmHalResetStatus ? AM_HAL_STATUS_SUCCESS : AM_HAL_STATUS_FAIL);

} // am_hal_reset_status_get()

//*****************************************************************************
//
//  am_hal_reset_interrupt_enable()
//  Use this function to enable the reset generator interrupts.
//
//*****************************************************************************
uint32_t
am_hal_reset_interrupt_enable(uint32_t ui32IntMask)
{
    AM_CRITICAL_BEGIN
    RSTGEN->INTEN |= ui32IntMask;
    AM_CRITICAL_END

    //
    // Return success status.
    //
    return AM_HAL_STATUS_SUCCESS;

} // am_hal_reset_interrupt_enable()

//*****************************************************************************
//
//  am_hal_reset_interrupt_disable()
//  Disable selected RSTGEN Interrupts.
//
//*****************************************************************************
uint32_t
am_hal_reset_interrupt_disable(uint32_t ui32IntMask)
{
    AM_CRITICAL_BEGIN
    RSTGEN->INTEN &= ~ui32IntMask;
    AM_CRITICAL_END

    //
    // Return success status.
    //
    return AM_HAL_STATUS_SUCCESS;

} // am_hal_reset_interrupt_disable()

//*****************************************************************************
//
//  am_hal_reset_interrupt_clear()
//  Reset generator interrupt clear
//
//*****************************************************************************
uint32_t
am_hal_reset_interrupt_clear(uint32_t ui32IntMask)
{
    RSTGEN->INTEN = ui32IntMask;
    *(volatile uint32_t*)(&RSTGEN->INTSTAT);
    //
    // Return success status.
    //
    return AM_HAL_STATUS_SUCCESS;

} // am_hal_reset_interrupt_clear()

//*****************************************************************************
//
//  am_hal_reset_interrupt_status_get()
//  Get interrupt status of reset generator.
//
//*****************************************************************************
uint32_t
am_hal_reset_interrupt_status_get(bool bEnabledOnly,
                                  uint32_t *pui32IntStatus)
{
    if ( pui32IntStatus == NULL )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Retrieve the reset generator status bits
    //
    *pui32IntStatus = RSTGEN->INTSTAT;

    //
    // Return success status.
    //
    return AM_HAL_STATUS_SUCCESS;

} // am_hal_reset_interrupt_status_get()

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
