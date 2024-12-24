//*****************************************************************************
//
//! @file am_hal_mram_recovery.c
//!
//! @brief MRAM Recovery API.
//!
//! @addtogroup MRAM Recovery Functionality
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

#include "am_mcu_apollo.h"
#include "am_hal_mram_recovery.h"

//
// Application Initiated MRAM Recovery
//
uint32_t am_hal_mram_recovery_init_app_recovery(uint32_t ui32Key, bool bReset)
{
    //
    // Check for valid key
    //
    if ( ui32Key != AM_HAL_MRAM_RECOVERY_KEY )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Write the key to the OTA Pointer register.
    //
    MCUCTRL->OTAPOINTER = ui32Key;

    //
    // Do a POI
    //
    if ( bReset )
    {
        RSTGEN->SWPOI = RSTGEN_SWPOI_SWPOIKEY_KEYVALUE;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//
// Read the MRAM Recovery Status
//
uint32_t am_hal_mram_recovery_read_status(am_hal_mram_recovery_status_t *psStatus)
{   
    if ( psStatus == NULL )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Mask & store the status bits.
    //
    psStatus->ui32MramRcvStatus = RSTGEN->STAT & AM_HAL_MRAM_RECOVERY_RSTGEN_STATUS_Msk;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
