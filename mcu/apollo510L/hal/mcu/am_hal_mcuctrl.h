//*****************************************************************************
//
//! @file am_hal_mcuctrl.h
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
#ifndef AM_HAL_MCUCTRL_H
#define AM_HAL_MCUCTRL_H

#ifdef __cplusplus
extern "C"
{
#endif

#define MCUCTRL_CHIPPN_PARTNUM_PN_M             0xFF000000
#define MCUCTRL_CHIPPN_PARTNUM_PN_S             24

//**********************************************************
//! MCUCTRL XTALHSCAP defaults for Cooper
//! Refer to App Note Apollo4 Blue 32MHz Crystal Calibration
//**********************************************************
#define XTALHSCAP2TRIM_DEFAULT  44
#define XTALHSCAPTRIM_DEFAULT   4

#define APOLLO5_B1                                                     \
  ((MCUCTRL->CHIPREV  &                                                \
    (MCUCTRL_CHIPREV_REVMAJ_Msk | MCUCTRL_CHIPREV_REVMIN_Msk)) ==      \
      (_VAL2FLD(MCUCTRL_CHIPREV_REVMAJ, MCUCTRL_CHIPREV_REVMAJ_B) |    \
        _VAL2FLD(MCUCTRL_CHIPREV_REVMIN, MCUCTRL_CHIPREV_REVMIN_REV1)))

#define APOLLO5_GE_B1                                                       \
        ((MCUCTRL->CHIPREV  &                                               \
           (MCUCTRL_CHIPREV_REVMAJ_Msk | MCUCTRL_CHIPREV_REVMIN_Msk)) >=    \
           (_VAL2FLD(MCUCTRL_CHIPREV_REVMAJ, MCUCTRL_CHIPREV_REVMAJ_B) |    \
            _VAL2FLD(MCUCTRL_CHIPREV_REVMIN, MCUCTRL_CHIPREV_REVMIN_REV1)))

#define APOLLO5_GT_B1                                                       \
        ((MCUCTRL->CHIPREV  &                                               \
           (MCUCTRL_CHIPREV_REVMAJ_Msk | MCUCTRL_CHIPREV_REVMIN_Msk))  >    \
           (_VAL2FLD(MCUCTRL_CHIPREV_REVMAJ, MCUCTRL_CHIPREV_REVMAJ_B) |    \
            _VAL2FLD(MCUCTRL_CHIPREV_REVMIN, MCUCTRL_CHIPREV_REVMIN_REV1)))

#define APOLLO5_B0                                                     \
  ((MCUCTRL->CHIPREV  &                                                \
    (MCUCTRL_CHIPREV_REVMAJ_Msk | MCUCTRL_CHIPREV_REVMIN_Msk)) ==      \
      (_VAL2FLD(MCUCTRL_CHIPREV_REVMAJ, MCUCTRL_CHIPREV_REVMAJ_B) |    \
        _VAL2FLD(MCUCTRL_CHIPREV_REVMIN, MCUCTRL_CHIPREV_REVMIN_REV0)))

#define APOLLO5_GE_B0                                                       \
        ((MCUCTRL->CHIPREV  &                                               \
           (MCUCTRL_CHIPREV_REVMAJ_Msk | MCUCTRL_CHIPREV_REVMIN_Msk)) >=    \
           (_VAL2FLD(MCUCTRL_CHIPREV_REVMAJ, MCUCTRL_CHIPREV_REVMAJ_B) |    \
            _VAL2FLD(MCUCTRL_CHIPREV_REVMIN, MCUCTRL_CHIPREV_REVMIN_REV0)))

#define APOLLO5_A0                                                     \
  ((MCUCTRL->CHIPREV  &                                                \
    (MCUCTRL_CHIPREV_REVMAJ_Msk | MCUCTRL_CHIPREV_REVMIN_Msk)) ==      \
      (_VAL2FLD(MCUCTRL_CHIPREV_REVMAJ, MCUCTRL_CHIPREV_REVMAJ_A) |    \
        _VAL2FLD(MCUCTRL_CHIPREV_REVMIN, MCUCTRL_CHIPREV_REVMIN_REV0)))

#define APOLLO5_GE_A0                                                       \
        ((MCUCTRL->CHIPREV  &                                               \
           (MCUCTRL_CHIPREV_REVMAJ_Msk | MCUCTRL_CHIPREV_REVMIN_Msk)) >=    \
           (_VAL2FLD(MCUCTRL_CHIPREV_REVMAJ, MCUCTRL_CHIPREV_REVMAJ_A) |    \
            _VAL2FLD(MCUCTRL_CHIPREV_REVMIN, MCUCTRL_CHIPREV_REVMIN_REV0)))


//*****************************************************************************
//
// MCUCTRL specific definitions.
//
//*****************************************************************************
// Define the size of fields derived from the PARTNUM register
#define AM_HAL_MCUCTRL_CHIPPN_NV_SIZE_N     (_FLD2VAL(MCUCTRL_CHIPPN_MRAMSIZE, 0xFFFFFFFF) + 1)
#define AM_HAL_MCUCTRL_CHIPPN_SRAM_SIZE_N   (_FLD2VAL(MCUCTRL_CHIPPN_SRAMSIZE, 0xFFFFFFFF) + 1)
// #### INTERNAL BEGIN ####
#if 0
#define AM_HAL_MCUCTRL_CHIPPN_TCM_SIZE_N    AM_HAL_MCUCTRL_CHIPPN_SRAM_SIZE_N
#endif
// #### INTERNAL END ####

// Define the size of fields derived from the SKU register
#define AM_HAL_MCUCTRL_SKU_SSRAM_SIZE_N     (_FLD2VAL(MCUCTRL_SKU_SKUSRAMSIZE, 0xFFFFFFFF) + 1)
#define AM_HAL_MCUCTRL_SKU_MRAM_SIZE_N      (_FLD2VAL(MCUCTRL_SKU_SKUMRAMSIZE, 0xFFFFFFFF) + 1)

#define AM_HAL_MCUCTRL_CRYSTAL_IBST_DURATION    500

//*****************************************************************************
//
// MCUCTRL enumerations
//
//*****************************************************************************
//**************************************
//! MCUCTRL control operations
//**************************************
typedef enum
{
    AM_HAL_MCUCTRL_CONTROL_EXTCLK32K_ENABLE,
    AM_HAL_MCUCTRL_CONTROL_EXTCLK32K_DISABLE,
    AM_HAL_MCUCTRL_CONTROL_EXTCLK32M_KICK_START,
    AM_HAL_MCUCTRL_CONTROL_EXTCLK32M_NORMAL,
    AM_HAL_MCUCTRL_CONTROL_EXTCLK32M_DISABLE
} am_hal_mcuctrl_control_e;

//**************************************
//
//! MCUCTRL info get
//
//**************************************
typedef enum
{
    AM_HAL_MCUCTRL_INFO_FEATURES_AVAIL,
    AM_HAL_MCUCTRL_INFO_DEVICEID
} am_hal_mcuctrl_infoget_e;

//*****************************************************************************
//
//! MCUCTRL SKU/Feature Enums
//
//*****************************************************************************
typedef enum
{
    AM_HAL_MCUCTRL_ITCM_128K,
    AM_HAL_MCUCTRL_ITCM_256K
} am_hal_mcuctrl_itcm_e;

typedef enum
{
    AM_HAL_MCUCTRL_DTCM_256K,
    AM_HAL_MCUCTRL_DTCM_512K
} am_hal_mcuctrl_dtcm_e;

typedef enum
{
    AM_HAL_MCUCTRL_SSRAM_1M,
    AM_HAL_MCUCTRL_SSRAM_2M,
    AM_HAL_MCUCTRL_SSRAM_3M,
} am_hal_mcuctrl_ssram_e;
#define AM_HAL_MCUCTRL_SSRAM_MAX    AM_HAL_MCUCTRL_SSRAM_3M

//
//! MRAM Size Setting
//
typedef enum
{
    AM_HAL_MCUCTRL_MRAM_1M,
    AM_HAL_MCUCTRL_MRAM_2M,
    AM_HAL_MCUCTRL_MRAM_3M,
    AM_HAL_MCUCTRL_MRAM_4M,
} am_hal_mcuctrl_mram_e;

#define AM_HAL_MCUCTRL_MRAM_MAX     AM_HAL_MCUCTRL_MRAM_4M

//
//! MVE Configuration.
//
typedef enum
{
    AM_HAL_MCUCTRL_NO_MVE,
    AM_HAL_MCUCTRL_INT_MVE,
    AM_HAL_MCUCTRL_FLT_MVE
} am_hal_mcuctrl_mve_e;

//*****************************************************************************
//
// MCUCTRL data structures
//
//*****************************************************************************
//**************************************
//
//! MCUCTRL device structure
//
//**************************************
typedef struct
{
    //
    //! Device part number. (BCD format)
    //
    uint32_t ui32ChipPN;

    //
    //! Unique Chip ID 0.
    //
    uint32_t ui32ChipID0;

    //
    //! Unique Chip ID 1.
    //
    uint32_t ui32ChipID1;

    //
    //! Chip Revision.
    //
    uint32_t ui32ChipRev;

    //
    //! Vendor ID.
    //
    uint32_t ui32VendorID;

    //
    //! SKU.
    //
    uint32_t ui32SKU;

    //
    //! Qualified chip.
    //
    uint32_t ui32Qualified;

    //
    //! NVM Size.
    //
    uint32_t ui32FlashSize;

    //
    //! ITCM Size
    //
    uint32_t ui32ITCMSize;

    //
    //! DTCM Size
    //
    uint32_t ui32DTCMSize;

    //
    //! SSRAM Size.
    //
    uint32_t ui32SSRAMSize;

    //
    //! MRAM Size.
    //
    uint32_t ui32MRAMSize;

    //
    // JEDEC chip info
    //
    uint32_t ui32JedecPN;
    uint32_t ui32JedecJEPID;
    uint32_t ui32JedecCHIPREV;
    uint32_t ui32JedecCID;
} am_hal_mcuctrl_device_t;

//**************************************
//
//! MCUCTRL status structure
//
//**************************************
typedef struct
{
    bool        bDebuggerLockout;   // DEBUGGER
    bool        bADCcalibrated;     // ADCCAL
    bool        bBattLoadEnabled;   // ADCBATTLOAD
    uint8_t     bSecBootOnWarmRst;  // BOOTLOADER
    uint8_t     bSecBootOnColdRst;  // BOOTLOADER
} am_hal_mcuctrl_status_t;

//**************************************
//
//! MCUCTRL features available structure
//
//**************************************
typedef struct
{
    am_hal_mcuctrl_dtcm_e       eDTCMSize;          // DTCM Size
    am_hal_mcuctrl_ssram_e      eSharedSRAMSize;    // SSRAM Size
    am_hal_mcuctrl_mram_e       eMRAMSize;          // MRAM Size
    bool                        bDISP;              // DISP
    bool                        bSupportHPMode;     // HP Mode
    bool                        bMIPIDSI;           // MIPI/DSI
    bool                        bGPU;               // GPU
    bool                        bUSB;               // USB
    bool                        bSecBootFeature;    // Secure Boot
    bool                        bRadioDisable;      // SKURADIODISABLE
    bool                        bRadioBTBLE;        // SKURADIOBTBLE
    bool                        bRadioBLE;          // SKURADIOBLE
    bool                        bCM4DEBUG;          // CM4 SWD
    bool                        bFPU;               // FPU
    am_hal_mcuctrl_mve_e        eMVECfg;            // MVE
} am_hal_mcuctrl_feature_t;

//**********************************************************
//
//! MCUCTRL XTALHSCAP Globals for Cooper Device
//! Refer to App Note Apollo4 Blue 32MHz Crystal Calibration
//
//**********************************************************
extern uint32_t g_ui32xtalhscap2trim;
extern uint32_t g_ui32xtalhscaptrim;

// ****************************************************************************
//
//! @brief Apply various specific commands/controls on the MCUCTRL module.
//!
//! This function is used to apply various controls to MCUCTRL.
//!
//! @param eControl - One of the following:
//!     AM_HAL_MCUCTRL_CONTROL_EXTCLK32K_ENABLE
//!     AM_HAL_MCUCTRL_CONTROL_EXTCLK32K_DISABLE
//! @param pArgs - Pointer to arguments for Control Switch Case
//!
//! @return status      - generic or interface specific status.
//
// ****************************************************************************
extern uint32_t am_hal_mcuctrl_control(am_hal_mcuctrl_control_e eControl,
                                       void *pArgs);

// ****************************************************************************
//
//! @brief MCUCTRL status function
//!
//! This function returns current status of the MCUCTRL as obtained from
//! various registers of the MCUCTRL block.
//!
//! @param psStatus - ptr to a status structure to receive the current statuses.
//!
//! @return status      - generic or interface specific status.
//
// ****************************************************************************
extern uint32_t am_hal_mcuctrl_status_get(am_hal_mcuctrl_status_t *psStatus);

// ****************************************************************************
//
//! @brief Get information of the given MCUCTRL item.
//!
//! This function returns a data structure of information regarding the given
//! MCUCTRL parameter.
//!
//! @param eInfoGet - One of the following:         Return structure type:
//!     AM_HAL_MCUCTRL_INFO_DEVICEID,               psDevice
//!     AM_HAL_MCUCTRL_INFO_FAULT_STATUS            psFault
//!
//! @param pInfo - A pointer to a structure to receive the return data,
//! the type of which is dependent on the eInfo parameter.
//!
//! @return status      - generic or interface specific status.
//
// ****************************************************************************
extern uint32_t am_hal_mcuctrl_info_get(am_hal_mcuctrl_infoget_e eInfoGet,
                                        void *pInfo);

//*****************************************************************************
//
//! @brief Get the count of Radio Fine Counter
//!
//! @return - the count of Radio Fine Counter with 0.5µs precision
//
//*****************************************************************************
extern uint32_t am_hal_mcuctrl_finecnt_get(void);

//*****************************************************************************
//
//! @brief Get the count of Radio CLKN Counter
//!
//! @return - the count of Radio CLKN Counter with 312.5μs precision
//
//*****************************************************************************
extern uint32_t am_hal_mcuctrl_clkncnt_get(void);

//*****************************************************************************
//
//! @brief I3C PHY pull down switch
//!
//! @param bEnable -  TRUE: Enable  I3C PHY pull down.
//!                  FALSE: Disable I3C PHY pull down.
//!
//
//*****************************************************************************
extern void am_hal_mcuctrl_i3c_phy_pulldown_enable(bool bEnable);

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_MCUCTRL_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

