//*****************************************************************************
//
//! @file am_hal_mcuctrl.c
//!
//! @brief Functions for interfacing with the MCUCTRL.
//!
//! @addtogroup mcuctrl4 MCUCTRL - MCU Control
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

//*****************************************************************************
//
// Global Variables.
//
//*****************************************************************************
//
//! Lookup table for memory sizes as derived from the SKU register.
//! Apollo510L has 256KB DTCM, 1.7M SSRAM, no ITCM(0KB).
//!  0: ITCM    size in KB
//!  1: DTCM    size in KB
//!  2: SSRAM   size in KB
//
static const uint16_t
g_am_hal_mcuctrl_sku_ram_size[AM_HAL_MCUCTRL_SKU_SSRAM_SIZE_N][3] =
{
    {0, 256, 1024},    //! 0x0:  0KB ITCM + 256KB DTCM + 1024KB SSRAM
    {0, 256, 768},     //! 0x1:  0KB ITCM + 256KB DTCM +  768KB SSRAM
};

//
//! Lookup table for MRAM sizes as derived from the SKU register.
//
static const uint16_t
g_am_hal_mcuctrl_sku_mram_size[AM_HAL_MCUCTRL_SKU_MRAM_SIZE_N] =
{
     1024,
     2048,
     3072,
     4096
};

// ****************************************************************************
// MCUCTRL XTALHSCAP Globals for Cooper Device
// Refer to App Note Apollo4 Blue 32MHz Crystal Calibration
// ****************************************************************************
uint32_t g_ui32xtalhscap2trim = XTALHSCAP2TRIM_DEFAULT;
uint32_t g_ui32xtalhscaptrim = XTALHSCAPTRIM_DEFAULT;

// ****************************************************************************
//
//  device_info_get()
//  Gets all relevant device information.
//
// ****************************************************************************
#define JEDEC_RD_DELAY

static void
device_info_get(am_hal_mcuctrl_device_t *psDevice)
{
    //
    // Read the Part Number.
    //
    psDevice->ui32ChipPN = MCUCTRL->CHIPPN;

    //
    // Read the Chip ID0.
    //
    psDevice->ui32ChipID0 = MCUCTRL->CHIPID0;

    //
    // Read the Chip ID1.
    //
    psDevice->ui32ChipID1 = MCUCTRL->CHIPID1;

    //
    // Read the Chip Revision.
    //
    psDevice->ui32ChipRev = MCUCTRL->CHIPREV;

    //
    // Read the Chip VENDOR ID.
    //
    psDevice->ui32VendorID = MCUCTRL->VENDORID;

    //
    // Read the SKU.
    //
    psDevice->ui32SKU = MCUCTRL->SKU;

    //
    // Qualified from Part Number.
    //
    psDevice->ui32Qualified = 1;

    //
    // MRAM size as derived from the SKU register.
    //
    psDevice->ui32MRAMSize = g_am_hal_mcuctrl_sku_mram_size[MCUCTRL->SKU_b.SKUMRAMSIZE] * 1024;

    //
    // ITCM size as derived from the SKU register.
    //
    psDevice->ui32ITCMSize = g_am_hal_mcuctrl_sku_ram_size[MCUCTRL->SKU_b.SKUSRAMSIZE][0] * 1024;

    //
    // DTCM size as derived from the SKU register.
    //
    psDevice->ui32DTCMSize = g_am_hal_mcuctrl_sku_ram_size[MCUCTRL->SKU_b.SKUSRAMSIZE][1] * 1024;

    //
    // Shared SRAM size as derived from the SKU register.
    //
    psDevice->ui32SSRAMSize = g_am_hal_mcuctrl_sku_ram_size[MCUCTRL->SKU_b.SKUSRAMSIZE][2] * 1024;

    //
    // Now, let's look at the JEDEC info.
    // The full partnumber is 12 bits total, but is scattered across 2 registers.
    // Bits [11:8] are 0xE.
    // Bits [7:4] are 0xE for Apollo, 0xD for Apollo2.
    // Bits [3:0] are defined differently for Apollo and Apollo2.
    //   For Apollo, the low nibble is 0x0.
    //   For Apollo2, the low nibble indicates flash and SRAM size.
    //
    psDevice->ui32JedecPN  = JEDEC->PID0_b.PNL8 << 0;
    JEDEC_RD_DELAY
    psDevice->ui32JedecPN |= JEDEC->PID1_b.PNH4 << 8;
    JEDEC_RD_DELAY

    //
    // JEPID is the JEP-106 Manufacturer ID Code, which is assigned to Ambiq as
    //  0x1B, with parity bit is 0x9B.  It is 8 bits located across 2 registers.
    //
    psDevice->ui32JedecJEPID  = JEDEC->PID1_b.JEPIDL << 0;
    JEDEC_RD_DELAY
    psDevice->ui32JedecJEPID |= JEDEC->PID2_b.JEPIDH << 4;
    JEDEC_RD_DELAY

    //
    // CHIPREV is 8 bits located across 2 registers.
    //
    psDevice->ui32JedecCHIPREV  = JEDEC->PID2_b.CHIPREVH4 << 4;
    JEDEC_RD_DELAY
    psDevice->ui32JedecCHIPREV |= JEDEC->PID3_b.CHIPREVL4 << 0;
    JEDEC_RD_DELAY

    //
    // Let's get the Coresight ID (32-bits across 4 registers)
    // For Apollo and Apollo2, it's expected to be 0xB105100D.
    //
    psDevice->ui32JedecCID  = JEDEC->CID3_b.CID << 24;
    JEDEC_RD_DELAY
    psDevice->ui32JedecCID |= JEDEC->CID2_b.CID << 16;
    JEDEC_RD_DELAY
    psDevice->ui32JedecCID |= JEDEC->CID1_b.CID <<  8;
    JEDEC_RD_DELAY
    psDevice->ui32JedecCID |= JEDEC->CID0_b.CID <<  0;
    JEDEC_RD_DELAY

} // device_info_get()

// ****************************************************************************
//
//  am_hal_mcuctrl_control()
//  Apply various specific commands/controls on the MCUCTRL module.
//
// ****************************************************************************
uint32_t
am_hal_mcuctrl_control(am_hal_mcuctrl_control_e eControl, void *pArgs)
{
    volatile uint32_t ui32Reg;

    switch ( eControl )
    {
        case AM_HAL_MCUCTRL_CONTROL_EXTCLK32K_ENABLE:
            //
            // Configure the bits in XTALCTRL that enable external 32KHz clock.
            //
// #### INTERNAL BEGIN ####
#if 1   // Replace multiple bitfield accesses with an equivalent, single register write.
// #### INTERNAL END ####
            ui32Reg  = MCUCTRL->XTALCTRL;
            ui32Reg &= ~(MCUCTRL_XTALCTRL_XTALPDNB_Msk                      |
                         MCUCTRL_XTALCTRL_XTALCOMPPDNB_Msk                  |
                         MCUCTRL_XTALCTRL_XTALCOMPBYPASS_Msk                |
                         MCUCTRL_XTALCTRL_XTALCOREDISFB_Msk                 |
                         MCUCTRL_XTALCTRL_XTALSWE_Msk);
            ui32Reg |= _VAL2FLD(MCUCTRL_XTALCTRL_XTALPDNB,       MCUCTRL_XTALCTRL_XTALPDNB_PWRUPCORE)       |
                       _VAL2FLD(MCUCTRL_XTALCTRL_XTALCOMPPDNB,   MCUCTRL_XTALCTRL_XTALCOMPPDNB_PWRUPCOMP)   |
                       _VAL2FLD(MCUCTRL_XTALCTRL_XTALCOMPBYPASS, MCUCTRL_XTALCTRL_XTALCOMPBYPASS_USECOMP)   |
                       _VAL2FLD(MCUCTRL_XTALCTRL_XTALCOREDISFB,  MCUCTRL_XTALCTRL_XTALCOREDISFB_EN)         |
                       _VAL2FLD(MCUCTRL_XTALCTRL_XTALSWE,        MCUCTRL_XTALCTRL_XTALSWE_OVERRIDE_EN);
            MCUCTRL->XTALCTRL = ui32Reg;
// #### INTERNAL BEGIN ####
#else
            MCUCTRL->XTALCTRL_b.XTALPDNB = MCUCTRL_XTALCTRL_XTALPDNB_PWRUPCORE;
            MCUCTRL->XTALCTRL_b.XTALCOMPPDNB = MCUCTRL_XTALCTRL_XTALCOMPPDNB_PWRUPCOMP;
            MCUCTRL->XTALCTRL_b.XTALCOMPBYPASS = MCUCTRL_XTALCTRL_XTALCOMPBYPASS_BYPCOMP;
            MCUCTRL->XTALCTRL_b.XTALCOREDISFB = MCUCTRL_XTALCTRL_XTALCOREDISFB_DIS;
            MCUCTRL->XTALCTRL_b.XTALSWE = MCUCTRL_XTALCTRL_XTALSWE_OVERRIDE_EN;
#endif
// #### INTERNAL END ####
            break;

        case AM_HAL_MCUCTRL_CONTROL_EXTCLK32K_DISABLE:
            //
            // Configure the bits in XTALCTRL that disable external 32KHz
            // clock, thus re-configuring for the crystal.
            //
// #### INTERNAL BEGIN ####
#if 1   // Replace multiple bitfield accesses with an equivalent, single register write.
// #### INTERNAL END ####
            ui32Reg  = MCUCTRL->XTALCTRL;
            ui32Reg &= ~(MCUCTRL_XTALCTRL_XTALPDNB_Msk                      |
                         MCUCTRL_XTALCTRL_XTALCOMPPDNB_Msk                  |
                         MCUCTRL_XTALCTRL_XTALCOMPBYPASS_Msk                |
                         MCUCTRL_XTALCTRL_XTALCOREDISFB_Msk                 |
                         MCUCTRL_XTALCTRL_XTALSWE_Msk);
            ui32Reg |= _VAL2FLD(MCUCTRL_XTALCTRL_XTALPDNB,       MCUCTRL_XTALCTRL_XTALPDNB_PWRUPCORE)       |
                       _VAL2FLD(MCUCTRL_XTALCTRL_XTALCOMPPDNB,   MCUCTRL_XTALCTRL_XTALCOMPPDNB_PWRUPCOMP)   |
                       _VAL2FLD(MCUCTRL_XTALCTRL_XTALCOMPBYPASS, MCUCTRL_XTALCTRL_XTALCOMPBYPASS_USECOMP)   |
                       _VAL2FLD(MCUCTRL_XTALCTRL_XTALCOREDISFB,  MCUCTRL_XTALCTRL_XTALCOREDISFB_EN)         |
                       _VAL2FLD(MCUCTRL_XTALCTRL_XTALSWE,        MCUCTRL_XTALCTRL_XTALSWE_OVERRIDE_DIS);
            MCUCTRL->XTALCTRL = ui32Reg;
// #### INTERNAL BEGIN ####
#else
            MCUCTRL->XTALCTRL_b.XTALPDNB = MCUCTRL_XTALCTRL_XTALPDNB_PWRUPCORE;
            MCUCTRL->XTALCTRL_b.XTALCOMPPDNB = MCUCTRL_XTALCTRL_XTALCOMPPDNB_PWRUPCOMP;
            MCUCTRL->XTALCTRL_b.XTALCOMPBYPASS = MCUCTRL_XTALCTRL_XTALCOMPBYPASS_USECOMP;
            MCUCTRL->XTALCTRL_b.XTALCOREDISFB = MCUCTRL_XTALCTRL_XTALCOREDISFB_EN;
            MCUCTRL->XTALCTRL_b.XTALSWE = MCUCTRL_XTALCTRL_XTALSWE_OVERRIDE_DIS;
#endif
// #### INTERNAL END ####
            break;

        case AM_HAL_MCUCTRL_CONTROL_EXTCLK32M_KICK_START:
// #### INTERNAL BEGIN ####
#if 0 // This reg was removed from drop8
// Set the default trim code for CAP1/CAP2, it impacts frequency accuracy and should be retrimmed
            ui32Reg = _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSCAP2TRIM, g_ui32xtalhscap2trim)    |
                      _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSCAPTRIM, g_ui32xtalhscaptrim)      |
// Set the transconductance of crystal to maximum, it accelerate the startup sequence
                      _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSDRIVETRIM, 3)                      |
// Choose the power of clock driver to be the cleanest one
                      _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSDRIVERSTRENGTH, 0)                 |
// Tune the bias generator
                      _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSIBIASCOMP2TRIM, 3)                 |
                      _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSIBIASCOMPTRIM, 15)                 |
// Set the bias of crystal to maximum
                      _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSIBIASTRIM, 127)                    |
                      _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSRSTRIM, 0)                         |
                      _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSSPARE, 0);
            MCUCTRL->XTALHSTRIMS = ui32Reg;
#endif
// #### INTERNAL END ####
            ui32Reg  = MCUCTRL->XTALHSCTRL;
            ui32Reg &= ~(MCUCTRL_XTALHSCTRL_XTALHSCOMPPDNB_Msk              |
                         MCUCTRL_XTALHSCTRL_XTALHSPDNPNIMPROVE_Msk          |
                         MCUCTRL_XTALHSCTRL_XTALHSPADOUTEN_Msk);
// Enable the option to output clock on PAD GPIO46 connecting to Cooper crystal input
            ui32Reg |= _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSCOMPPDNB, 1)       |
                       _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSPDNPNIMPROVE, 0)   |
                       _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSPADOUTEN, 1);
            MCUCTRL->XTALHSCTRL = ui32Reg;
            //
            // Turn on xtalhs_pdnb & xtalhs_injection_enable
            //

            // Turn on crystal oscillator
            MCUCTRL->XTALHSCTRL_b.XTALHSPDNB = 1;
            // inject HFRC clock to accelerate the startup sequence
            MCUCTRL->XTALHSCTRL_b.XTALHSINJECTIONENABLE = 1;

            //
            // Turn on xtalhs_ibst_enable
            //
            // Maximize the bias current to accelerate the startup sequence
            MCUCTRL->XTALHSCTRL_b.XTALHSIBSTENABLE = 1;

            //
            // Turn off xtalhs_injection_enable
            //
            // Wait 5us to make the setting above effective
            am_hal_delay_us(5);
            // Turn off the clock injection
            MCUCTRL->XTALHSCTRL_b.XTALHSINJECTIONENABLE = 0;

            // Apply external source
            if ( (pArgs) && (*((bool *)pArgs) == true) )
            {
                ui32Reg = MCUCTRL->XTALHSCTRL;
                ui32Reg &=  ~(MCUCTRL_XTALHSCTRL_XTALHSPDNB_Msk                 |
                              MCUCTRL_XTALHSCTRL_XTALHSIBSTENABLE_Msk           |
                              MCUCTRL_XTALHSCTRL_XTALHSEXTERNALCLOCK_Msk);
                ui32Reg |=  _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSPDNB, 0)          |
                            _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSIBSTENABLE, 0)    |
                            _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSEXTERNALCLOCK, 1);
                MCUCTRL->XTALHSCTRL = ui32Reg;
            }

            break;

        case AM_HAL_MCUCTRL_CONTROL_EXTCLK32M_NORMAL:
// #### INTERNAL BEGIN ####
#if 0 // This reg was removed from drop8
            ui32Reg = _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSCAP2TRIM, g_ui32xtalhscap2trim)    |
                      _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSCAPTRIM, g_ui32xtalhscaptrim)      |
                      _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSDRIVETRIM, 3)                      |
                      _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSDRIVERSTRENGTH, 0)                 |
                      _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSIBIASCOMP2TRIM, 3)                 |
                      _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSIBIASCOMPTRIM, 15)                 |
                      _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSIBIASTRIM, 127)                    |
                      _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSRSTRIM, 0)                         |
                      _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSSPARE, 0);
            MCUCTRL->XTALHSTRIMS = ui32Reg;
#endif
// #### INTERNAL END ####
            ui32Reg  = MCUCTRL->XTALHSCTRL;
            ui32Reg &= ~(MCUCTRL_XTALHSCTRL_XTALHSCOMPPDNB_Msk                  |
                         MCUCTRL_XTALHSCTRL_XTALHSPDNPNIMPROVE_Msk              |
                         MCUCTRL_XTALHSCTRL_XTALHSPADOUTEN_Msk);
            ui32Reg |= _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSCOMPPDNB, 1)           |
                       _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSPDNPNIMPROVE, 1)       |
                       _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSPADOUTEN, 1);
            MCUCTRL->XTALHSCTRL = ui32Reg;
            //
            // Turn on xtalhs_pdnb
            //
            MCUCTRL->XTALHSCTRL_b.XTALHSPDNB = 1;
            // Apply external source
            ui32Reg = MCUCTRL->XTALHSCTRL;
            // Apply external source
            if ( (pArgs) && (*((bool *)pArgs) == true) )
            {
                ui32Reg &=  ~(MCUCTRL_XTALHSCTRL_XTALHSPDNB_Msk                 |
                              MCUCTRL_XTALHSCTRL_XTALHSEXTERNALCLOCK_Msk);
                ui32Reg |=  _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSPDNB, 0)          |
                            _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSEXTERNALCLOCK, 1);
            }
            else
            {
                ui32Reg &=  ~(MCUCTRL_XTALHSCTRL_XTALHSPDNPNIMPROVE_Msk         |
                              MCUCTRL_XTALHSCTRL_XTALHSIBSTENABLE_Msk);
                ui32Reg |=  _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSPDNPNIMPROVE, 0)  |
                            _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSIBSTENABLE, 1);
            }

            MCUCTRL->XTALHSCTRL = ui32Reg;
            break;

        case AM_HAL_MCUCTRL_CONTROL_EXTCLK32M_DISABLE:
// #### INTERNAL BEGIN ####
#if 0 // This reg was removed from drop8
            ui32Reg = _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSCAP2TRIM, g_ui32xtalhscap2trim)    |
                      _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSCAPTRIM, g_ui32xtalhscaptrim)      |
                      _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSDRIVETRIM, 0)                      |
                      _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSDRIVERSTRENGTH, 7)                 |
                      _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSIBIASCOMP2TRIM, 3)                 |
                      _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSIBIASCOMPTRIM, 8)                  |
                      _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSIBIASTRIM, 24)                     |
                      _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSRSTRIM, 0)                         |
                      _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSSPARE, 0);
            MCUCTRL->XTALHSTRIMS = ui32Reg;
#endif
// #### INTERNAL END ####
            ui32Reg  = MCUCTRL->XTALHSCTRL;
            ui32Reg &= ~(MCUCTRL_XTALHSCTRL_XTALHSPDNB_Msk                  |
                         MCUCTRL_XTALHSCTRL_XTALHSEXTERNALCLOCK_Msk         |
                         MCUCTRL_XTALHSCTRL_XTALHSCOMPPDNB_Msk              |
                         MCUCTRL_XTALHSCTRL_XTALHSIBSTENABLE_Msk            |
                         MCUCTRL_XTALHSCTRL_XTALHSPDNPNIMPROVE_Msk          |
                         MCUCTRL_XTALHSCTRL_XTALHSPADOUTEN_Msk);
            ui32Reg |= _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSPDNB, 0)           |
                       _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSEXTERNALCLOCK, 0)  |
                       _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSCOMPPDNB, 1)       |
                       _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSIBSTENABLE, 0)     |
                       _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSPDNPNIMPROVE, 0)   |
                       _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSPADOUTEN, 1);
            MCUCTRL->XTALHSCTRL = ui32Reg;
            break;

        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Return success status.
    //
    return AM_HAL_STATUS_SUCCESS;

} // am_hal_mcuctrl_control()

// ****************************************************************************
//
//  am_hal_mcuctrl_status_get()
// This function returns  current status of the MCUCTRL as obtained from
// various registers of the MCUCTRL block.
//
// ****************************************************************************
uint32_t
am_hal_mcuctrl_status_get(am_hal_mcuctrl_status_t *psStatus)
{
    uint32_t ui32Status;

    if ( psStatus == NULL )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    psStatus->bDebuggerLockout =
        _FLD2VAL(MCUCTRL_DEBUGGER_LOCKOUT, MCUCTRL->DEBUGGER);

    psStatus->bADCcalibrated =
        _FLD2VAL(MCUCTRL_ADCCAL_ADCCALIBRATED, MCUCTRL->ADCCAL);

    psStatus->bBattLoadEnabled =
        _FLD2VAL(MCUCTRL_ADCBATTLOAD_BATTLOAD, MCUCTRL->ADCBATTLOAD);

    ui32Status = MCUCTRL->BOOTLOADER;

// ##### INTERNAL BEGIN #####
    // TODO: FIXME - These are with a Vanilla part.  Need to set for DM LCS part.
// ##### INTERNAL END #####
    psStatus->bSecBootOnColdRst =
        (_FLD2VAL(MCUCTRL_BOOTLOADER_SECBOOT, ui32Status) != MCUCTRL_BOOTLOADER_SECBOOT_ERROR);
    psStatus->bSecBootOnWarmRst =
        (_FLD2VAL(MCUCTRL_BOOTLOADER_SECBOOTONRST, ui32Status) != MCUCTRL_BOOTLOADER_SECBOOTONRST_ERROR);

    return AM_HAL_STATUS_SUCCESS;

} // am_hal_mcuctrl_status_get()

// ****************************************************************************
//
//  am_hal_mcuctrl_info_get()
//  Get information of the given MCUCTRL item.
//
// ****************************************************************************
uint32_t
am_hal_mcuctrl_info_get(am_hal_mcuctrl_infoget_e eInfoGet, void *pInfo)
{
    if ( pInfo == NULL )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    am_hal_mcuctrl_feature_t *psFeature = (am_hal_mcuctrl_feature_t *)pInfo;

    switch ( eInfoGet )
    {
        case AM_HAL_MCUCTRL_INFO_FEATURES_AVAIL:
            //
            // Decode ITCM, DTCM, SSRAM sizes.
            //
            switch(MCUCTRL->SKU_b.SKUSRAMSIZE)
            {
                case 0:
                    psFeature->eDTCMSize        = AM_HAL_MCUCTRL_DTCM_256K;
                    psFeature->eSharedSRAMSize  = AM_HAL_MCUCTRL_SSRAM_1M;
                    break;
                case 1:
                    psFeature->eDTCMSize        = AM_HAL_MCUCTRL_DTCM_256K;
                    psFeature->eSharedSRAMSize  = AM_HAL_MCUCTRL_SSRAM_2M;
                    break;
                case 2:
                    psFeature->eDTCMSize        = AM_HAL_MCUCTRL_DTCM_512K;
                    psFeature->eSharedSRAMSize  = AM_HAL_MCUCTRL_SSRAM_2M;
                    break;
                case 3:
                    psFeature->eDTCMSize        = AM_HAL_MCUCTRL_DTCM_512K;
                    psFeature->eSharedSRAMSize  = AM_HAL_MCUCTRL_SSRAM_3M;
                    break;
            }
            //
            // Decode MRAM sizes.
            //
            psFeature->eMRAMSize        = (am_hal_mcuctrl_mram_e)MCUCTRL->SKU_b.SKUMRAMSIZE;
            psFeature->bSupportHPMode   = (MCUCTRL->SKU_b.SKUTURBOSPOT > 0);
            psFeature->bMIPIDSI         = (MCUCTRL->SKU_b.SKUMIPIDSI > 0);
            psFeature->bGPU             = (MCUCTRL->SKU_b.SKUGFX > 0);
            psFeature->bUSB             = (MCUCTRL->SKU_b.SKUUSB > 0);
            psFeature->bSecBootFeature  = (MCUCTRL->SKU_b.SKUSECURESPOT > 0);
            psFeature->bFPU             = (MCUCTRL->SKU_b.SKUFPU > 0);
            psFeature->eMVECfg          = (am_hal_mcuctrl_mve_e)MCUCTRL->SKU_b.SKUMVE;
            psFeature->bDISP            = (MCUCTRL->SKU_b.SKUDISP > 0);
            psFeature->bRadioDisable    = (MCUCTRL->SKU_b.SKURADIODISABLE > 0);
            psFeature->bRadioBTBLE      = (MCUCTRL->SKU_b.SKURADIOBTBLE > 0);
            psFeature->bRadioBLE        = (MCUCTRL->SKU_b.SKURADIOBLE > 0);
            psFeature->bCM4DEBUG        = (MCUCTRL->SKU_b.SKUCM4DEBUG > 0);

            break;

        case AM_HAL_MCUCTRL_INFO_DEVICEID:
            device_info_get((am_hal_mcuctrl_device_t *)pInfo);
            break;

        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Return success status.
    //
    return AM_HAL_STATUS_SUCCESS;

} // am_hal_mcuctrl_info_get()

//*****************************************************************************
//
// am_hal_mcuctrl_finecnt_get()
// Read Radio Fine Counter with 0.5µs precision
//
//*****************************************************************************
uint32_t
am_hal_mcuctrl_finecnt_get(void)
{
    uint32_t ui32CurrVal;

    ui32CurrVal = MCUCTRL->RADIOFINECNT;
    return ui32CurrVal;
} // am_hal_mcuctrl_finecnt_get()

//*****************************************************************************
//
// am_hal_mcuctrl_clkncnt_get()
// Read Radio CLKN Counter with a 312.5μs precision
//
//*****************************************************************************
uint32_t
am_hal_mcuctrl_clkncnt_get(void)
{
    uint32_t ui32CurrVal;

    ui32CurrVal = MCUCTRL->RADIOCLKNCNT;
    return ui32CurrVal;
} // am_hal_mcuctrl_clkncnt_get()

//*****************************************************************************
//
// am_hal_mcuctrl_i3c_phy_pulldown_enable()
// I3CPHY Pulldown Swiitch Control
//
//*****************************************************************************
void
am_hal_mcuctrl_i3c_phy_pulldown_enable(bool bEnable)
{
    if ( bEnable )
    {
        MCUCTRL->I3CPHYCTRL = MCUCTRL_I3CPHYCTRL_I3CPHYPULLDN_PULLDNEN;
    }
    else
    {
        MCUCTRL->I3CPHYCTRL = MCUCTRL_I3CPHYCTRL_I3CPHYPULLDN_PULLDNDIS;
    }
} // am_hal_mcuctrl_i3c_phy_pulldown_enable

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
