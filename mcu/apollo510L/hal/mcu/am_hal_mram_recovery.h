//*****************************************************************************
//
//! @file am_hal_mram_recovery.h
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

#ifndef AM_HAL_MRAM_RECOVERY_H
#define AM_HAL_MRAM_RECOVERY_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

#define AM_HAL_MRAM_RECOVERY_KEY            ( 0xAFD1363A )

//
// MRAM Recovery Status Mask 
//
#define AM_HAL_MRAM_RECOVERY_RSTGEN_STATUS_Msk    ( RSTGEN_STAT_RCVYINPROGRESS_Msk  |   \
                                                    RSTGEN_STAT_RCVYUARTLOAD_Msk    |   \
                                                    RSTGEN_STAT_RCVYSPILOAD_Msk     |   \
                                                    RSTGEN_STAT_RCVYINITIATED_Msk   |   \
                                                    RSTGEN_STAT_SBLWIREDOTA_Msk     |   \
                                                    RSTGEN_STAT_SBLWIREDLOAD_Msk    |   \
                                                    RSTGEN_STAT_SBLNVOTA_Msk        |   \
                                                    RSTGEN_STAT_SBLNVIMAGELOAD_Msk  |   \
                                                    RSTGEN_STAT_OEMWIREDOTA_Msk     |   \
                                                    RSTGEN_STAT_OEMWIREDLOAD_Msk    |   \
                                                    RSTGEN_STAT_OEMNVOTA_Msk        |   \
                                                    RSTGEN_STAT_OEMNVIMAGELOAD_Msk  |   \
                                                    RSTGEN_STAT_OEMRCVYFAIL_Msk     |   \
                                                    RSTGEN_STAT_OEMRCVYSUCCESS_Msk  |   \
                                                    RSTGEN_STAT_SBLRCVYFAIL_Msk     |   \
                                                    RSTGEN_STAT_SBLRCVYSUCCESS_Msk )
#define AM_HAL_MRAM_RECOVERY_RSTGEN_STATUS_Pos    ( RSTGEN_STAT_RCVYINPROGRESS_Pos )

//**************************************
//
//! MRAM Recovery Status Structure
//
//**************************************
typedef struct
{
    union
    {
        struct
        {
            uint16_t rsvd : 15;
            bool    bRecoveryInProgress : 1;
            bool    bUartLoad : 1;
            bool    bSpiLoad : 1;
            uint8_t ui8RecoveryInitiated : 2;
            bool    bSblWiredOta : 1;
            bool    bSblWiredOtaLoad : 1;
            bool    bSblNvOta : 1;
            bool    bSblNvImageLoad : 1;
            bool    bOemWiredOta : 1;
            bool    bOemWiredLoad : 1;
            bool    bOemNvOta : 1;
            bool    bOemNvImageLoad : 1;
            bool    bOemRecoveryFail : 1;
            bool    bOemRecoverySuccess     : 1;
            bool    bSblRecoveryFail : 1;
            bool    bSblRecoverySuccess : 1;
        }MramRcvStatus_b;
        uint32_t ui32MramRcvStatus;
    };
}am_hal_mram_recovery_status_t;
//*****************************************************************************
//
//! @brief Writes the Application Initiated MRAM Recovery key to OTA pointer.
//!        Need to call POI for initiating MRAM Recovery.
//!
//! @param ui32Key - The key to write to the OTA Pointer register.
//!        bReset  - Set to true if a reset is needed after writing the key
//!
//! @return 0 for success, non-zero for failure.
//
//*****************************************************************************
extern uint32_t am_hal_mram_recovery_init_app_recovery(uint32_t ui32Key, bool bReset);

//*****************************************************************************
//
//! @brief Reads the MRAM Recovery Status register.
//!
//! @param pui32Status - Pointer to the location to store the status.
//!
//! @return 0 for success, non-zero for failure.
//
//*****************************************************************************
extern uint32_t am_hal_mram_recovery_read_status(am_hal_mram_recovery_status_t *psStatus);


#ifdef __cplusplus
}
#endif

#endif // AM_HAL_MRAM_RECOVERY_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
