//*****************************************************************************
//
//! @file am_hal_pwrctrl.c
//!
//! @brief Functions for enabling and disabling power domains.
//!
//! @addtogroup pwrctrl4 PWRCTRL - Power Control
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
// #### INTERNAL BEGIN ####
//#define TEMPCO_DBG 1
#if TEMPCO_DBG
extern uint32_t am_util_stdio_printf(const char *pui8Fmt, ...);
#endif
// #### INTERNAL END ####

//*****************************************************************************
//
// Local defines
//
//*****************************************************************************

//
//! Maximum number of checks to memory power status before declaring error
// (5 x 1usec = 5usec).
//
#define AM_HAL_PWRCTRL_MAX_WAIT_US          5
// #### INTERNAL BEGIN ####
// As per design inputs from CAB-1160, max time we may need to wait should account
// for 32b write - which is 40us x32
// Making this timeout large enough - 100ms to even work for FPGA
// #### INTERNAL END ####
#define AM_HAL_PWRCTRL_MAX_WAIT_OTP_US      100000
#define AM_HAL_PWRCTRL_MAX_BOOTROM_COUNT    10000

#define AM_HAL_PWRCTRL_MEMPWRSTATUS_MASK    ( PWRCTRL_MEMPWRSTATUS_PWRSTTCM_Msk        |    \
                                              PWRCTRL_MEMPWRSTATUS_PWRSTNVM0_Msk       |    \
                                              PWRCTRL_MEMPWRSTATUS_PWRSTROM_Msk )

//
// Power trims
//
#define ADJUST_POWER_SETTING           true
#define RESTORE_POWER_SETTING          false
#define CORELDOTEMPCOTRIM_VAL          2

#define MEMLDOACTIVETRIM_DELTA_0        5
#define TVRGFVREFTRIM_DELTA_0           24

#define MEMLDOACTIVETRIM_DELTA_1        23

#define TVRGFVREFTRIM_VAL_0             127
#define TVRGFTEMPCOTRIM_VAL_0           31

#define VDDCACTLOWTONTRIM_VAL           21
#define VDDFACTLOWTONTRIM_VAL           31

//
//! Trim revision 6 is required for the following pwrctrl adjustments:
//!  - Sourcing MCUH from the VDDC_LV rail
//!  - Reliable application of TempCo
//!  - Applying Crypto boosts
// #### INTERNAL BEGIN ####
//  TRIMREV_VDDC_LV see FALCSW-286
//  TRIMREV_TEMPCO see FALCSW-253
//  TRIMREV_BOOST_ADJUST see FALCSW-348.
// #### INTERNAL END ####
//
#define TRIMREV_PWRCTRL         6

#define AM_HAL_PWRCTRL_GPU_VOLTADJ_WAIT  1
#define AM_HAL_PWRCTRL_GPU_PWRADJ_WAIT   6
#define AM_HAL_PWRCTRL_GPU_PWRON_WAIT    1

#define CORE_SLEEP_DURATION            50  //in us

static uint32_t nvic_en[16];
static uint32_t origBasePri, orgPriMask;
static bool g_bSTIntEnabled = false;

//*****************************************************************************
//
//! @name Define max values of some useful fields
//! @{
//
// ****************************************************************************
#define MAX_ACTTRIMVDDF         _FLD2VAL(MCUCTRL_SIMOBUCK12_ACTTRIMVDDF, 0xFFFFFFFF)    // Buck VDDF
#define MAX_MEMLDOACTIVETRIM    _FLD2VAL(MCUCTRL_LDOREG2_MEMLDOACTIVETRIM, 0xFFFFFFFF)  // LDO VDDF
#define MAX_LPTRIMVDDF          _FLD2VAL(MCUCTRL_SIMOBUCK12_LPTRIMVDDF, 0xFFFFFFFF)     // VDDF LP
#define MAX_MEMLPLDOTRIM        _FLD2VAL(MCUCTRL_LDOREG2_MEMLPLDOTRIM, 0xFFFFFFFF)      // MEM LP LDO
#define MAX_BUCKVDDCTRIM        _FLD2VAL(MCUCTRL_VREFGEN2_TVRGVREFTRIM, 0xFFFFFFFF)     // Buck VDDC
//! @}

// ****************************************************************************
//
// Global variables.
//
// ****************************************************************************

//*****************************************************************************
//
//! @name Global State Variables for the VDDF and VDDC boosting
//! @{
//
// ****************************************************************************
am_hal_pwrctrl_mcu_mode_e       g_eCurMcuPwrMode    = AM_HAL_PWRCTRL_MCU_MODE_LOW_POWER;
am_hal_pwrctrl_rom_select_e     g_eCurROMPwrMode    = AM_HAL_PWRCTRL_ROM_ALWAYS_ON;

uint32_t g_ui32TrimVer                      = 0xFFFFFFFF;
uint32_t g_ui32origSimobuckVDDStrim         = 0xFFFFFFFF;
uint32_t g_ui32VDDCBoostReqCnt              = 0;
uint32_t g_ui32VDDFBoostReqLevel1Cnt        = 0;
uint32_t g_ui32VDDFBoostReqLevel2Cnt        = 0;
//! @}

//*****************************************************************************
//
//! @name Save factory trim values.
//! @{
//
// ****************************************************************************
static bool     g_bOrigTrimsStored          = false;
static uint32_t g_orig_ACTTRIMVDDF          = 0;
static uint32_t g_orig_MEMLDOACTIVETRIM     = 0;
static uint32_t g_orig_LPTRIMVDDF           = 0;
static uint32_t g_orig_MEMLPLDOTRIM         = 0;
static uint32_t g_orig_TVRGVREFTRIM         = 0;

//! @}

#if AM_HAL_TEMPCO_LP
//*****************************************************************************
//
//! @name Saved handle and slot number for TempCo.
//! @{
//
// ****************************************************************************
static void    *g_TempcoADCHandle;
static uint32_t g_ui32TempcoADCslot;
static uint16_t g_ui16TempcoTEMP_code;
static bool     g_bTempcoValid           = false;
static float    g_pfTempMeasured;
//
//! @}
//
#endif // AM_HAL_TEMPCO_LP

//*****************************************************************************
//
// This table will be populated with all available INFO1 values and
// will be used for easy lookup after OTP is powered down.
//
//*****************************************************************************
am_hal_pwrctrl_info1_regs_t g_sINFO1regs;

//*****************************************************************************
//
//! @name DEVPWREN and DEVPWRSTATUS Mask Macros
//! @{
//! The below DEVPWREN and DEVPWRSTATUS masks are used to check if a peripheral
//!    has been disabled properly
//!
//! The original check of ((PWRCTRL->DEVPWRSTATUS & ui32PeriphStatus) == 0)
//!     will fail when more than one enable in the same domain is set and the
//!     user tries disable only one.
// ****************************************************************************

#define PWRCTRL_HCPB_DEVPWREN_MASK       ( \
    _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM0, PWRCTRL_DEVPWREN_PWRENIOM0_EN) | \
    _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM1, PWRCTRL_DEVPWREN_PWRENIOM1_EN) | \
    _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM2, PWRCTRL_DEVPWREN_PWRENIOM2_EN) | \
    _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM3, PWRCTRL_DEVPWREN_PWRENIOM3_EN))

#define PWRCTRL_HCPC_DEVPWREN_MASK       ( \
    _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM4, PWRCTRL_DEVPWREN_PWRENIOM4_EN) | \
    _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM5, PWRCTRL_DEVPWREN_PWRENIOM5_EN))

#define PWRCTRL_HCPA_DEVPWREN_MASK       ( \
    _VAL2FLD(PWRCTRL_DEVPWREN_PWRENUART0, PWRCTRL_DEVPWREN_PWRENUART0_EN) | \
    _VAL2FLD(PWRCTRL_DEVPWREN_PWRENUART1, PWRCTRL_DEVPWREN_PWRENUART1_EN))

#define PWRCTRL_MSPI_DEVPWREN_MASK       ( \
    _VAL2FLD(PWRCTRL_DEVPWREN_PWRENMSPI0, PWRCTRL_DEVPWREN_PWRENMSPI0_EN) | \
    _VAL2FLD(PWRCTRL_DEVPWREN_PWRENMSPI1, PWRCTRL_DEVPWREN_PWRENMSPI1_EN) | \
    _VAL2FLD(PWRCTRL_DEVPWREN_PWRENMSPI2, PWRCTRL_DEVPWREN_PWRENMSPI2_EN))

#define PWRCTRL_AUD_DEVPWREN_MASK       ( \
    _VAL2FLD(PWRCTRL_AUDSSPWREN_PWRENPDM0, PWRCTRL_AUDSSPWREN_PWRENPDM0_EN)     | \
    _VAL2FLD(PWRCTRL_AUDSSPWREN_PWRENI2S0, PWRCTRL_AUDSSPWREN_PWRENI2S0_EN))

#define PWRCTRL_HCPB_DEVPWRSTATUS_MASK      ( \
    PWRCTRL_DEVPWRSTATUS_PWRSTIOM0_Msk | \
    PWRCTRL_DEVPWRSTATUS_PWRSTIOM1_Msk | \
    PWRCTRL_DEVPWRSTATUS_PWRSTIOM2_Msk | \
    PWRCTRL_DEVPWRSTATUS_PWRSTIOM3_Msk)

#define PWRCTRL_HCPC_DEVPWRSTATUS_MASK      ( \
    PWRCTRL_DEVPWRSTATUS_PWRSTIOM4_Msk | \
    PWRCTRL_DEVPWRSTATUS_PWRSTIOM5_Msk)

#define PWRCTRL_HCPA_DEVPWRSTATUS_MASK          ( \
    PWRCTRL_DEVPWRSTATUS_PWRSTUART0_Msk | \
    PWRCTRL_DEVPWRSTATUS_PWRSTUART1_Msk)

#define PWRCTRL_MSPI_DEVPWRSTATUS_MASK          ( \
    PWRCTRL_DEVPWRSTATUS_PWRSTMSPI0_Msk | \
    PWRCTRL_DEVPWRSTATUS_PWRSTMSPI1_Msk | \
    PWRCTRL_DEVPWRSTATUS_PWRSTMSPI2_Msk)

#define PWRCTRL_AUD_DEVPWRSTATUS_MASK      ( \
    PWRCTRL_AUDSSPWRSTATUS_PWRSTPDM0_Msk   | \
    PWRCTRL_AUDSSPWRSTATUS_PWRSTI2S0_Msk)
//! @}

// **********************************************
//! Define the peripheral control structure.
// **********************************************
struct am_pwr_s
{
    uint32_t    ui32PwrEnRegAddr;
    uint32_t    ui32PeriphEnable;
    uint32_t    ui32PwrStatReqAddr;
    uint32_t    ui32PeriphStatus;
};

//
//! Peripheral control data structure
//
#ifndef AM_HAL_PWRCTRL_RAM_TABLE
const struct am_pwr_s am_hal_pwrctrl_peripheral_control[AM_HAL_PWRCTRL_PERIPH_MAX] =
{
// #### INTERNAL BEGIN ####
#if 0 // IOS0 is removed from RTL
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOS0, PWRCTRL_DEVPWREN_PWRENIOS0_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTIOS0_Msk
    },
#endif
// #### INTERNAL END ####
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOSFD0, PWRCTRL_DEVPWREN_PWRENIOSFD0_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTIOSFD0_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOSFD1, PWRCTRL_DEVPWREN_PWRENIOSFD1_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTIOSFD1_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM0, PWRCTRL_DEVPWREN_PWRENIOM0_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_HCPB_DEVPWRSTATUS_MASK
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM1, PWRCTRL_DEVPWREN_PWRENIOM1_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_HCPB_DEVPWRSTATUS_MASK
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM2, PWRCTRL_DEVPWREN_PWRENIOM2_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_HCPB_DEVPWRSTATUS_MASK
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM3, PWRCTRL_DEVPWREN_PWRENIOM3_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_HCPB_DEVPWRSTATUS_MASK
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM4, PWRCTRL_DEVPWREN_PWRENIOM4_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_HCPC_DEVPWRSTATUS_MASK
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM5, PWRCTRL_DEVPWREN_PWRENIOM5_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_HCPC_DEVPWRSTATUS_MASK
    },
// #### INTERNAL BEGIN ####
#if 0 // IOM6 & IOM7 are removed from RTL
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM6, PWRCTRL_DEVPWREN_PWRENIOM6_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_HCPC_DEVPWRSTATUS_MASK
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM7, PWRCTRL_DEVPWREN_PWRENIOM7_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_HCPC_DEVPWRSTATUS_MASK
    },
#endif
// #### INTERNAL END ####
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENUART0, PWRCTRL_DEVPWREN_PWRENUART0_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_HCPA_DEVPWRSTATUS_MASK
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENUART1, PWRCTRL_DEVPWREN_PWRENUART1_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_HCPA_DEVPWRSTATUS_MASK
    },
// #### INTERNAL BEGIN ####
#if 0 // UART2 & UART3 are removed from RTL
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENUART2, PWRCTRL_DEVPWREN_PWRENUART2_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_HCPA_DEVPWRSTATUS_MASK
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENUART3, PWRCTRL_DEVPWREN_PWRENUART3_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_HCPA_DEVPWRSTATUS_MASK
    },
#endif
// #### INTERNAL END ####
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENADC, PWRCTRL_DEVPWREN_PWRENADC_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTADC_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENMSPI0, PWRCTRL_DEVPWREN_PWRENMSPI0_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTMSPI0_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENMSPI1, PWRCTRL_DEVPWREN_PWRENMSPI1_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTMSPI1_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENMSPI2, PWRCTRL_DEVPWREN_PWRENMSPI2_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTMSPI2_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENGFX, PWRCTRL_DEVPWREN_PWRENGFX_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTGFX_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENDISP, PWRCTRL_DEVPWREN_PWRENDISP_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTDISP_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENDISPPHY, PWRCTRL_DEVPWREN_PWRENDISPPHY_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTDISPPHY_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENCRYPTO, PWRCTRL_DEVPWREN_PWRENCRYPTO_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTCRYPTO_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENSDIO0, PWRCTRL_DEVPWREN_PWRENSDIO0_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTSDIO0_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENSDIO1, PWRCTRL_DEVPWREN_PWRENSDIO1_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTSDIO1_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENUSB, PWRCTRL_DEVPWREN_PWRENUSB_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTUSB_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENUSBPHY, PWRCTRL_DEVPWREN_PWRENUSBPHY_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTUSBPHY_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENDBG, PWRCTRL_DEVPWREN_PWRENDBG_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTDBG_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENOTP, PWRCTRL_DEVPWREN_PWRENOTP_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTOTP_Msk
    },
// #### INTERNAL BEGIN ####
#if 0 // am_hal_pwrctrl.c:  As of 26SEPT, PWRENAUDPB, PWRENAUDREC, PWRSTAUDPB, PWRSTAUDREC no longer exist.
    {
        AM_REGADDR(PWRCTRL, AUDSSPWREN),
        _VAL2FLD(PWRCTRL_AUDSSPWREN_PWRENAUDREC, PWRCTRL_AUDSSPWREN_PWRENAUDREC_EN),
        AM_REGADDR(PWRCTRL, AUDSSPWRSTATUS),
        PWRCTRL_AUDSSPWRSTATUS_PWRSTAUDREC_Msk
    },
    {
        AM_REGADDR(PWRCTRL, AUDSSPWREN),
        _VAL2FLD(PWRCTRL_AUDSSPWREN_PWRENAUDPB, PWRCTRL_AUDSSPWREN_PWRENAUDPB_EN),
        AM_REGADDR(PWRCTRL, AUDSSPWRSTATUS),
        PWRCTRL_AUDSSPWRSTATUS_PWRSTAUDPB_Msk
    },
#endif
// #### INTERNAL END ####
    {
        AM_REGADDR(PWRCTRL, AUDSSPWREN),
        _VAL2FLD(PWRCTRL_AUDSSPWREN_PWRENPDM0, PWRCTRL_AUDSSPWREN_PWRENPDM0_EN),
        AM_REGADDR(PWRCTRL, AUDSSPWRSTATUS),
        PWRCTRL_AUDSSPWRSTATUS_PWRSTPDM0_Msk
    },
    {
        AM_REGADDR(PWRCTRL, AUDSSPWREN),
        _VAL2FLD(PWRCTRL_AUDSSPWREN_PWRENI2S0, PWRCTRL_AUDSSPWREN_PWRENI2S0_EN),
        AM_REGADDR(PWRCTRL, AUDSSPWRSTATUS),
        PWRCTRL_AUDSSPWRSTATUS_PWRSTI2S0_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENI3C, PWRCTRL_DEVPWREN_PWRENI3C_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTI3C_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENI3CPHY, PWRCTRL_DEVPWREN_PWRENI3CPHY_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTI3CPHY_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENNETAOL, PWRCTRL_DEVPWREN_PWRENNETAOL_ON),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTNETAOL_Msk
    },

// #### INTERNAL BEGIN ####
#if 0 // I2S1 and AUDADC are removed from RTL
    {
        AM_REGADDR(PWRCTRL, AUDSSPWREN),
        _VAL2FLD(PWRCTRL_AUDSSPWREN_PWRENI2S1, PWRCTRL_AUDSSPWREN_PWRENI2S1_EN),
        AM_REGADDR(PWRCTRL, AUDSSPWRSTATUS),
        PWRCTRL_AUDSSPWRSTATUS_PWRSTI2S1_Msk
    },
    {
        AM_REGADDR(PWRCTRL, AUDSSPWREN),
        _VAL2FLD(PWRCTRL_AUDSSPWREN_PWRENAUDADC, PWRCTRL_AUDSSPWREN_PWRENAUDADC_EN),
        AM_REGADDR(PWRCTRL, AUDSSPWRSTATUS),
        PWRCTRL_AUDSSPWRSTATUS_PWRSTAUDADC_Msk
    },
#endif
// #### INTERNAL END ####
};

//*****************************************************************************
//
// Mask off all interrupts
//
//*****************************************************************************
static inline uint32_t
am_critical_mask_all_begin(void)
{
    uint32_t basePrioGrouping = ((uint32_t)((SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) >> SCB_AIRCR_PRIGROUP_Pos));

    orgPriMask = am_hal_interrupt_master_disable();
    origBasePri = __get_BASEPRI();
    __set_BASEPRI((basePrioGrouping >= (8 - __NVIC_PRIO_BITS))?(1 << (basePrioGrouping + 1)):(1 << (8 - __NVIC_PRIO_BITS)));
    for (uint32_t i = 0; i < 16; i++)
    {
        nvic_en[i] = NVIC->ISER[i];
        NVIC->ICER[i] = nvic_en[i];
    }
    //
    // Disable SysTick Int if it was enabled.
    //
    if (SysTick->CTRL & SysTick_CTRL_TICKINT_Msk)
    {
        g_bSTIntEnabled = true;
        SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
    }

    return AM_HAL_STATUS_SUCCESS;

} // am_critical_mask_all_begin()

//*****************************************************************************
//
//  Restore the interrupts
//
//*****************************************************************************
static inline void
am_critical_mask_all_end(void)
{
    uint32_t ui32STVal = 0;
    bool bSetSTPend = false;

    for (uint32_t i = 0; i < 16; i++ )
    {
        NVIC->ISER[i] = nvic_en[i];
    }
    __set_BASEPRI(origBasePri);

    if (g_bSTIntEnabled)
    {
        //
        // Address potential race condition where Systick interrupt could have
        // been fired - while we disabled it
        //
        // Current snapshot for Systick
        //
        ui32STVal = SysTick->VAL;

        //
        // Need to read COUNTFLAG before writing to CTRL, as it would get reset
        //
        bSetSTPend = SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk;

        //
        // Enable SysTick Int
        //
        SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;

        //
        // Read Systick again to see if it wrapped
        //
        if (SysTick->VAL > ui32STVal)
        {
              bSetSTPend = true;
        }

        //
        // Set PENDSTSET
        //
        if (bSetSTPend)
        {
            SCB->ICSR |= SCB_ICSR_PENDSTSET_Msk;
        }
    }

    am_hal_interrupt_master_set(orgPriMask);
} // am_critical_mask_all_end()

//*****************************************************************************
//
//! @brief  Return the pwr_ctrl entry for a given ePeripheral
//!
//! @param  pwr_ctrl address where the power entry is copied
//! @param  ePeripheral the peripheral to copy
//!
//! @return Returns AM_HAL_STATUS_SUCCESS on success
//
//*****************************************************************************
static inline uint32_t
am_get_pwrctrl(struct am_pwr_s *pwr_ctrl, uint32_t ePeripheral)
{
    if ( pwr_ctrl == NULL || ePeripheral >= AM_HAL_PWRCTRL_PERIPH_MAX )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    *pwr_ctrl = am_hal_pwrctrl_peripheral_control[ePeripheral];

    return AM_HAL_STATUS_SUCCESS;
} // am_get_pwrctrl()
#else
//*****************************************************************************
//
//! @brief  Return the pwr_ctrl entry for a given ePeripheral
//!
//! @param  pwr_ctrl address where the power entry is generated
//! @param  ePeripheral the peripheral for which to generate:
//!
//! @return Returns AM_HAL_STATUS_SUCCESS on success
//
//*****************************************************************************
static uint32_t
am_get_pwrctrl(struct am_pwr_s *pwr_ctrl, uint32_t ePeripheral)
{
    int shift_pos;

    if (pwr_ctrl == NULL || ePeripheral >= AM_HAL_PWRCTRL_PERIPH_MAX)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    if (ePeripheral < AM_HAL_PWRCTRL_PERIPH_PDM0)
    {
        pwr_ctrl->ui32PwrEnRegAddr = AM_REGADDR(PWRCTRL, DEVPWREN);
        pwr_ctrl->ui32PwrStatReqAddr = AM_REGADDR(PWRCTRL, DEVPWRSTATUS);
        pwr_ctrl->ui32PeriphEnable = 1 << ePeripheral;
        pwr_ctrl->ui32PeriphStatus = 1 << ePeripheral;
    }
    else
    {
        shift_pos = (ePeripheral - AM_HAL_PWRCTRL_PERIPH_PDM0) + 2;
        if (ePeripheral > AM_HAL_PWRCTRL_PERIPH_I2S1)
        {
            shift_pos += 2;
        }

        pwr_ctrl->ui32PwrEnRegAddr =  AM_REGADDR(PWRCTRL, AUDSSPWREN);
        pwr_ctrl->ui32PwrStatReqAddr = AM_REGADDR(PWRCTRL, AUDSSPWRSTATUS);
        pwr_ctrl->ui32PeriphEnable = 1 << shift_pos;
        pwr_ctrl->ui32PeriphStatus = 1 << shift_pos;
    }

    return AM_HAL_STATUS_SUCCESS;
} // am_get_pwrctrl()
#endif // AM_HAL_PWRCTRL_RAM_TABLE

//*****************************************************************************
//
// Default configurations definitions
//
//*****************************************************************************
// #### INTERNAL BEGIN ####
#warning "TODO - FIXME: What are the proper default memory configurations?"
// #### INTERNAL END ####
const am_hal_pwrctrl_mcu_memory_config_t    g_DefaultMcuMemCfg =
{
    .eROMMode           = AM_HAL_PWRCTRL_ROM_AUTO,
    .eDTCMCfg           = AM_HAL_PWRCTRL_DTCM256K,
    .eRetainDTCM        = AM_HAL_PWRCTRL_MEMRETCFG_TCMPWDSLP_RETAIN,
    .eNVMCfg            = AM_HAL_PWRCTRL_NVM,
    .eNVMPowerDown      = AM_HAL_PWRCTRL_NVM_DEEP_POWER_DOWN,
    .bKeepNVMOnInDeepSleep         = false,
};

const am_hal_pwrctrl_sram_memcfg_t          g_DefaultSRAMCfg =
{
    .eSRAMCfg           = AM_HAL_PWRCTRL_SRAM_1P75M,
    .eActiveWithMCU     = AM_HAL_PWRCTRL_SRAM_NONE,
    .eActiveWithGFX     = AM_HAL_PWRCTRL_SRAM_NONE,
    .eActiveWithDISP    = AM_HAL_PWRCTRL_SRAM_NONE,
    .eSRAMRetain        = AM_HAL_PWRCTRL_SRAM_1P75M
};

//*****************************************************************************
//
// Function to determine the chip's TRIM version.
//
// return Status code.
//
// pui32TrimVer: The uint32_t that will receive the trim version number.
//               If no valid trim version found, *pui32TrimVer returns as 0.
//
//
//*****************************************************************************
static uint32_t
TrimVersionGet(uint32_t *pui32TrimVer)
{
    uint32_t ui32Ret;

    //
    // Get the TRIM version and set the global variable.
    // This only needs to be done and verified once.
    //
    if ( g_ui32TrimVer == 0xFFFFFFFF )
    {
        ui32Ret = am_hal_info1_read(AM_HAL_INFO_INFOSPACE_CURRENT_INFO1,
                                    AM_REG_OTP_INFO1_TRIM_REV_O / 4,
                                    1, &g_ui32TrimVer);

        if ( (g_ui32TrimVer == 0x00000000)  ||
#ifdef AM_PART_APOLLO5A
             (g_ui32TrimVer == 0xFFFFFFFF)  ||
#endif // AM_PART_APOLLO5A
            (ui32Ret != AM_HAL_STATUS_SUCCESS) )
        {
            //
            // Invalid trim value. Set the global to indicate version 0.
            //
            g_ui32TrimVer = 0;
        }
    }

    if ( pui32TrimVer )
    {
        *pui32TrimVer = g_ui32TrimVer;
        return AM_HAL_STATUS_SUCCESS;
    }
    else
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

} // TrimVersionGet()

// ****************************************************************************
//
//  am_hal_pwrctrl_mcu_mode_status()
//
// ****************************************************************************
uint32_t
am_hal_pwrctrl_mcu_mode_status(am_hal_pwrctrl_mcu_mode_e *peCurrentPowerMode)
{
#ifndef AM_HAL_DISABLE_API_VALIDATION
    if ( peCurrentPowerMode == 0 )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    *peCurrentPowerMode = g_eCurMcuPwrMode;
    return AM_HAL_STATUS_SUCCESS;

} // am_hal_pwrctrl_mcu_mode_status()

// ****************************************************************************
//
//  am_hal_pwrctrl_mcu_mode_select()
//  Select the MCU power mode.
//
// ****************************************************************************
uint32_t
am_hal_pwrctrl_mcu_mode_select(am_hal_pwrctrl_mcu_mode_e ePowerMode)
{
    uint32_t ui32Status;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    if ( (ePowerMode != AM_HAL_PWRCTRL_MCU_MODE_LOW_POWER)      &&
         (ePowerMode != AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE1) &&
         (ePowerMode != AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE2) )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
#if !defined(APOLLO5_FPGA)
    //
    // We must be using SIMOBUCK in order to go to HP mode.
    //
    if ( (ePowerMode == AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE1)   &&
         (PWRCTRL->VRSTATUS_b.SIMOBUCKST != PWRCTRL_VRSTATUS_SIMOBUCKST_ACT) )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
#endif
#endif // AM_HAL_DISABLE_API_VALIDATION

    //
    // Check if the request mode is already selected.  If so, return SUCCESS.
    //
    if ( ePowerMode == g_eCurMcuPwrMode )
    {
        return AM_HAL_STATUS_SUCCESS;
    }

    AM_CRITICAL_BEGIN

    g_eCurMcuPwrMode = ePowerMode;

    //
    // Set the MCUCTRL MCU power mode.
    //
    switch (ePowerMode)
    {
        case AM_HAL_PWRCTRL_MCU_MODE_LOW_POWER:
        case AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE1:
            MCUCTRL->SKUOVERRIDE_b.SKUTURBOSPOT = MCUCTRL_SKU_SKUTURBOSPOT_HP1;
            break;
        case AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE2:
            MCUCTRL->SKUOVERRIDE_b.SKUTURBOSPOT = MCUCTRL_SKU_SKUTURBOSPOT_HP2;
            break;
    }

    //
    // Set the MCU power mode.
    //
    PWRCTRL->MCUPERFREQ_b.MCUPERFREQ = ePowerMode;

    //
    // Wait for the ACK
    //
    ui32Status = AM_HAL_STATUS_TIMEOUT;
#ifdef APOLLO5_FPGA
    for ( uint32_t i = 0; i < 60; i++ )
    {
        if ( PWRCTRL->MCUPERFREQ_b.MCUPERFACK > 0 )
        {
            ui32Status = AM_HAL_STATUS_SUCCESS;
            break;
        }
        am_hal_delay_us(1);
    }
#else
    for ( uint32_t i = 0; i < 5; i++ )
    {
        if ( PWRCTRL->MCUPERFREQ_b.MCUPERFACK > 0 )
        {
            ui32Status = AM_HAL_STATUS_SUCCESS;
            break;
        }
        am_hal_delay_us(1);
    }
#endif // APOLLO5_FPGA

    AM_CRITICAL_END

    //
    // Check for timeout.
    //
    if ( ui32Status != AM_HAL_STATUS_SUCCESS )
    {
        //
        // Caution: Reaching this point means the device is in an unpredictable
        //          state and may not be able to recover.
        //
        return ui32Status;
    }

    //
    // Check the MCU power mode status and return SUCCESS/FAIL.
    //
    if ( PWRCTRL->MCUPERFREQ_b.MCUPERFSTATUS == ePowerMode )
    {
        return AM_HAL_STATUS_SUCCESS;
    }
    else
    {
        //
        // Caution: Reaching this point means the device is in an unpredictable
        //          state and may not be able to recover.
        //
        return AM_HAL_STATUS_FAIL;
    }

} // am_hal_pwrctrl_mcu_mode_select()

// ****************************************************************************
//
//  am_hal_pwrctrl_mcu_memory_config()
//  Configure the MCU memory.
//
// ****************************************************************************
uint32_t
am_hal_pwrctrl_mcu_memory_config(am_hal_pwrctrl_mcu_memory_config_t *psConfig)
{
    uint32_t      ui32Status;
    uint32_t      ui32PwrStatus = 0;
    uint32_t      ui32PwrEn = 0;
    bool          bForceAxiClkEn = false;

    //
    // Configure the ROM power mode.
    //
    g_eCurROMPwrMode = psConfig->eROMMode;

    switch (psConfig->eROMMode)
    {
        case AM_HAL_PWRCTRL_ROM_AUTO:
            //
            // Initialize the ROM to off.  Let the HAL access function control the power.
            //
            ui32PwrEn |= _VAL2FLD(PWRCTRL_MEMPWREN_PWRENROM, 0);
            ui32PwrStatus |= _VAL2FLD(PWRCTRL_MEMPWRSTATUS_PWRSTROM, 0);
            break;
        case AM_HAL_PWRCTRL_ROM_ALWAYS_ON:
            //
            // Initialize the ROM to be always on.
            //
            ui32PwrEn |= _VAL2FLD(PWRCTRL_MEMPWREN_PWRENROM, 1);
            ui32PwrStatus |= _VAL2FLD(PWRCTRL_MEMPWRSTATUS_PWRSTROM, 1);
            break;
    }

    //
    // Configure the MCU Tightly Coupled Memory.
    //
    ui32PwrEn |= _VAL2FLD(PWRCTRL_MEMPWREN_PWRENTCM, psConfig->eDTCMCfg);
    ui32PwrStatus |= _VAL2FLD(PWRCTRL_MEMPWRSTATUS_PWRSTTCM, psConfig->eDTCMCfg);

    //
    // Configure the Non-Volatile Memory.
    //
    switch (psConfig->eNVMCfg)
    {
        case AM_HAL_PWRCTRL_NVM_NONE:
            //
            // Power down all NVMs.
            //
            ui32PwrEn |= _VAL2FLD(PWRCTRL_MEMPWREN_PWRENNVM, 0);
            ui32PwrStatus |= _VAL2FLD(PWRCTRL_MEMPWRSTATUS_PWRSTNVM0, 0);
            break;
        case AM_HAL_PWRCTRL_NVM:
            //
            // Power up NVM0, power down NVM1.
            //
            ui32PwrEn |= _VAL2FLD(PWRCTRL_MEMPWREN_PWRENNVM, 1);
            ui32PwrStatus |= _VAL2FLD(PWRCTRL_MEMPWRSTATUS_PWRSTNVM0, 1);
            break;
    }

    //
    // Configure the Non-Volatile Memory Power Down.
    //
    PWRCTRL->MEMRETCFG_b.NVMPWDCFG = psConfig->eNVMPowerDown;


    DIAG_SUPPRESS_VOLATILE_ORDER()

    //
    // Special logic needed, when powering on NVM1 by itself
    // Need to force AXI Clock during the transition
    // If powering on both banks "at same time", no special processing needed
    //
// #### INTERNAL BEGIN ####
    // CAYNSWS-1544/CAB-1209
// #### INTERNAL END ####
    if ((psConfig->eNVMCfg == AM_HAL_PWRCTRL_NVM) &&
        ((PWRCTRL->MEMPWRSTATUS & PWRCTRL_MEMPWRSTATUS_PWRSTNVM0_Msk) == PWRCTRL_MEMPWRSTATUS_PWRSTNVM0_Msk))
    {
        MCUCTRL->FORCEAXICLKEN_b.FRCAXICLKEN = 1;
        bForceAxiClkEn = true;
    }

    PWRCTRL->MEMPWREN = ui32PwrEn;

    //
    // Wait for Status
    //
    ui32Status = am_hal_delay_us_status_check(AM_HAL_PWRCTRL_MAX_WAIT_US,
                                              (uint32_t)&PWRCTRL->MEMPWRSTATUS,
                                              AM_HAL_PWRCTRL_MEMPWRSTATUS_MASK,
                                              ui32PwrStatus,
                                              true);
    if (bForceAxiClkEn)
    {
        MCUCTRL->FORCEAXICLKEN_b.FRCAXICLKEN = 0;
    }

    //
    // Check for timeout.
    //
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return ui32Status;
    }

    //
    // Check the MCU power mode status and return SUCCESS/FAIL.
    //
    if ((PWRCTRL->MEMPWRSTATUS_b.PWRSTTCM != PWRCTRL->MEMPWREN_b.PWRENTCM)          ||
        (PWRCTRL->MEMPWRSTATUS_b.PWRSTNVM0 != PWRCTRL->MEMPWREN_b.PWRENNVM)         ||
        (PWRCTRL->MEMPWRSTATUS_b.PWRSTROM != PWRCTRL->MEMPWREN_b.PWRENROM)          ||
        (PWRCTRL->DEVPWRSTATUS_b.PWRSTOTP != PWRCTRL->DEVPWREN_b.PWRENOTP))
    {
        return AM_HAL_STATUS_FAIL;
    }

    DIAG_DEFAULT_VOLATILE_ORDER()

    //
    // Configure the Non-Volatile Memory retention.
    //
    if (psConfig->bKeepNVMOnInDeepSleep)
    {
        PWRCTRL->MEMRETCFG_b.NVMPWDSLP = PWRCTRL_MEMRETCFG_NVMPWDSLP_DIS;
    }
    else
    {
        PWRCTRL->MEMRETCFG_b.NVMPWDSLP = PWRCTRL_MEMRETCFG_NVMPWDSLP_EN;
    }

    //
    // Configure the MCU Tightly Coupled Memory retention.
    //
    if ( psConfig->eRetainDTCM == AM_HAL_PWRCTRL_MEMRETCFG_TCMPWDSLP_NORETAIN )
    {
        PWRCTRL->MEMRETCFG_b.TCMPWDSLP = PWRCTRL_MEMRETCFG_TCMPWDSLP_NORETAIN;
    }
    else if ( psConfig->eRetainDTCM == AM_HAL_PWRCTRL_MEMRETCFG_TCMPWDSLP_RETAIN )
    {
        PWRCTRL->MEMRETCFG_b.TCMPWDSLP = PWRCTRL_MEMRETCFG_TCMPWDSLP_RETAIN;
    }
    else
    {
        return AM_HAL_STATUS_OUT_OF_RANGE;
    }

    return AM_HAL_STATUS_SUCCESS;

} // am_hal_pwrctrl_mcu_memory_config()

// ****************************************************************************
//
//  am_hal_pwrctrl_mcu_memory_config_get()
//  Get the MCU Memory configuration.
//
// ****************************************************************************
uint32_t
am_hal_pwrctrl_mcu_memory_config_get(am_hal_pwrctrl_mcu_memory_config_t *psConfig)
{
    //
    // Get the MCU Tightly Coupled Memory configuration.
    //
    psConfig->eDTCMCfg =
        (am_hal_pwrctrl_dtcm_select_e)PWRCTRL->MEMPWREN_b.PWRENTCM;

    //
    // Get the Non-Volatile Memory configuration.
    //
    psConfig->eNVMCfg =
        (am_hal_pwrctrl_nvm_select_e)(PWRCTRL->MEMPWREN_b.PWRENNVM);

    //
    // Configure the Non-Volatile Memory retention.
    //
    psConfig->bKeepNVMOnInDeepSleep =
        (PWRCTRL->MEMRETCFG_b.NVMPWDSLP == PWRCTRL_MEMRETCFG_NVMPWDSLP_DIS);

    //
    // Configure the MCU Tightly Coupled Memory retention.
    //
    if (PWRCTRL->MEMRETCFG_b.TCMPWDSLP == PWRCTRL_MEMRETCFG_TCMPWDSLP_RETAIN)
    {
        psConfig->eRetainDTCM = AM_HAL_PWRCTRL_MEMRETCFG_TCMPWDSLP_RETAIN;
    }
    else if (PWRCTRL->MEMRETCFG_b.TCMPWDSLP == PWRCTRL_MEMRETCFG_TCMPWDSLP_NORETAIN)
    {
        psConfig->eRetainDTCM = AM_HAL_PWRCTRL_MEMRETCFG_TCMPWDSLP_NORETAIN;
    }
    else
    {
        return AM_HAL_STATUS_OUT_OF_RANGE;
    }

    return AM_HAL_STATUS_SUCCESS;
} // am_hal_pwrctrl_mcu_memory_config_get()

// ****************************************************************************
//
//  am_hal_pwrctrl_rom_enable()
//  Enable the ROM power domain if AUTO mode is selected.
//
// ****************************************************************************
uint32_t
am_hal_pwrctrl_rom_enable(void)
{
    uint32_t    count;

    if ( AM_HAL_PWRCTRL_ROM_AUTO == g_eCurROMPwrMode )
    {
        PWRCTRL->MEMPWREN_b.PWRENROM = PWRCTRL_MEMPWREN_PWRENROM_EN;

        //
        // Wait for Status
        //
// #### INTERNAL BEGIN ####
        // TODO - We cannot call am_hal_delay_us_status_check now as it will also call the BootROM power enable/disable
        //        functions as they are no reentrant.
        //ui32Status = am_hal_delay_us_status_check(AM_HAL_PWRCTRL_MAX_WAIT_US,
        //                                          (uint32_t)&PWRCTRL->MEMPWRSTATUS,
        //                                          AM_HAL_PWRCTRL_MEMPWRSTATUS_MASK,
        // TODO - Need to calculate expected power status due to misalignment of MEMPWREN and MEMPWRSTATUS fields.
        //                                          PWRCTRL->MEMPWREN,
        //                                          true);
#warning "TODO - Fixme!!! Replace this with TCM based delay function when it is ready."
// #### INTERNAL END ####
        for (count = 0; count < AM_HAL_PWRCTRL_MAX_BOOTROM_COUNT; count++)
        {
            if (PWRCTRL->MEMPWRSTATUS_b.PWRSTROM == 1)
            {
                break;
            }
        }

        //
        // Check for timeout.
        //
        if (count == AM_HAL_PWRCTRL_MAX_BOOTROM_COUNT)
        {
            return AM_HAL_STATUS_TIMEOUT;
        }
    }

    return AM_HAL_STATUS_SUCCESS;
} // am_hal_pwrctrl_rom_enable()

// ****************************************************************************
//
//  am_hal_pwrctrl_rom_disable()
//  Enable the ROM power domain if AUTO mode is selected.
//
// ****************************************************************************
uint32_t
am_hal_pwrctrl_rom_disable(void)
{
    uint32_t    ui32Count;

    if ( AM_HAL_PWRCTRL_ROM_AUTO == g_eCurROMPwrMode )
    {
        PWRCTRL->MEMPWREN_b.PWRENROM = PWRCTRL_MEMPWREN_PWRENROM_DIS;

        //
        // Wait for Status
        //
// #### INTERNAL BEGIN ####
        // TODO - We cannot call am_hal_delay_us_status_check now as it will also call the BootROM power enable/disable
        //        functions as they are no reentrant.
        //ui32Status = am_hal_delay_us_status_check(AM_HAL_PWRCTRL_MAX_WAIT_US,
        //                                          (uint32_t)&PWRCTRL->MEMPWRSTATUS,
        //                                          AM_HAL_PWRCTRL_MEMPWRSTATUS_MASK,
        // TODO - Need to calculate expected power status due to misalignment of MEMPWREN and MEMPWRSTATUS fields.
        //                                          PWRCTRL->MEMPWREN,
        //                                          true);
#warning "TODO - Fixme!!! Replace this with TCM based delay function when it is ready."
// #### INTERNAL END ####
        for (ui32Count = 0; ui32Count < AM_HAL_PWRCTRL_MAX_BOOTROM_COUNT; ui32Count++)
        {
            if (PWRCTRL->MEMPWRSTATUS_b.PWRSTROM == 0)
            {
                break;
            }
        }

        //
        // Check for timeout.
        //
        if (ui32Count == AM_HAL_PWRCTRL_MAX_BOOTROM_COUNT)
        {
            return AM_HAL_STATUS_TIMEOUT;
        }
    }

    return AM_HAL_STATUS_SUCCESS;

} // am_hal_pwrctrl_rom_disable()

// ****************************************************************************
//
//  am_hal_pwrctrl_sram_config()
//  Configure the Shared RAM.
//
// ****************************************************************************
uint32_t
am_hal_pwrctrl_sram_config(am_hal_pwrctrl_sram_memcfg_t *psConfig)
{
    uint32_t      ui32Status;

    //
    // Configure the Shared RAM.
    //
    PWRCTRL->SSRAMPWREN_b.PWRENSSRAM = psConfig->eSRAMCfg;

    DIAG_SUPPRESS_VOLATILE_ORDER()

    //
    // Wait for Status
    //
    ui32Status = am_hal_delay_us_status_check(AM_HAL_PWRCTRL_MAX_WAIT_US,
                                              (uint32_t)&PWRCTRL->SSRAMPWRST,
                                              PWRCTRL_SSRAMPWRST_SSRAMPWRST_Msk,
                                              PWRCTRL->SSRAMPWREN,
                                              true);

    //
    // Check for error.
    //
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return ui32Status;
    }

    //
    // Check the Shared RAM power mode status.
    //
    if (PWRCTRL->SSRAMPWRST_b.SSRAMPWRST != PWRCTRL->SSRAMPWREN_b.PWRENSSRAM)
    {
        return AM_HAL_STATUS_FAIL;
    }
    DIAG_DEFAULT_VOLATILE_ORDER()

    //
    // Configure the Shared RAM domain active based on the states of the MCU,
    // graphics, and display.
    //
    PWRCTRL->SSRAMRETCFG_b.SSRAMACTMCU  = psConfig->eActiveWithMCU;
    PWRCTRL->SSRAMRETCFG_b.SSRAMACTGFX  = psConfig->eActiveWithGFX;
// #### INTERNAL BEGIN ####
#if 0 // DISP is removed from 20240319 RTL. May be added back later.
    PWRCTRL->SSRAMRETCFG_b.SSRAMACTDISP = psConfig->eActiveWithDISP;
#endif
// #### INTERNAL END ####

    //
    // Configure the Shared RAM retention.
    //
    switch ( psConfig->eSRAMRetain )
    {
        case AM_HAL_PWRCTRL_SRAM_NONE:
            PWRCTRL->SSRAMRETCFG_b.SSRAMPWDSLP = PWRCTRL_SSRAMRETCFG_SSRAMPWDSLP_ALL;
            break;
        case AM_HAL_PWRCTRL_SRAM_1M:        // Retain lower 1M, pwr dwn upper 0.75M
            PWRCTRL->SSRAMRETCFG_b.SSRAMPWDSLP = PWRCTRL_SSRAMRETCFG_SSRAMPWDSLP_GROUP1;
            break;
        case AM_HAL_PWRCTRL_SRAM_0P75M:    // Retain dwn upper 0.75M, pwr dwn lower 1M
            PWRCTRL->SSRAMRETCFG_b.SSRAMPWDSLP = PWRCTRL_SSRAMRETCFG_SSRAMPWDSLP_GROUP0;
            break;
        case AM_HAL_PWRCTRL_SRAM_1P75M:    // Retain all SSRAM, power down none
            PWRCTRL->SSRAMRETCFG_b.SSRAMPWDSLP = PWRCTRL_SSRAMRETCFG_SSRAMPWDSLP_NONE;
            break;
    }

    return AM_HAL_STATUS_SUCCESS;
} // am_hal_pwrctrl_sram_config()

// ****************************************************************************
//
//  am_hal_pwrctrl_sram_config_get()
//  Get the current Shared RAM configuration.
//
// ****************************************************************************
uint32_t
am_hal_pwrctrl_sram_config_get(am_hal_pwrctrl_sram_memcfg_t *psConfig)
{
    //
    // Get the Shared RAM configuration.
    //
    psConfig->eSRAMCfg = (am_hal_pwrctrl_sram_select_e)PWRCTRL->SSRAMPWREN_b.PWRENSSRAM;

    //
    // Get the SRAM active configurations based for each of MCU, graphics, and display.
    //
    psConfig->eActiveWithMCU  = (am_hal_pwrctrl_sram_select_e)PWRCTRL->SSRAMRETCFG_b.SSRAMACTMCU;
    psConfig->eActiveWithGFX  = (am_hal_pwrctrl_sram_select_e)PWRCTRL->SSRAMRETCFG_b.SSRAMACTGFX;
// #### INTERNAL BEGIN ####
#if 0 // DISP is removed from 20240319 RTL. May be added back later.
    psConfig->eActiveWithDISP = (am_hal_pwrctrl_sram_select_e)PWRCTRL->SSRAMRETCFG_b.SSRAMACTDISP;
#endif
// #### INTERNAL END ####

    //uint32_t val = (PWRCTRL_SSRAMRETCFG_SSRAMPWDSLP_GROUP1 || PWRCTRL_SSRAMRETCFG_SSRAMPWDSLP_GROUP2);
    //
    // Get the SRAM retention configuration.
    //
    if (PWRCTRL->SSRAMRETCFG_b.SSRAMPWDSLP == PWRCTRL_SSRAMRETCFG_SSRAMPWDSLP_ALL)
    {
        psConfig->eSRAMRetain = AM_HAL_PWRCTRL_SRAM_NONE;
    }
    else if (PWRCTRL->SSRAMRETCFG_b.SSRAMPWDSLP == PWRCTRL_SSRAMRETCFG_SSRAMPWDSLP_NONE)
    {
        psConfig->eSRAMRetain = AM_HAL_PWRCTRL_SRAM_1P75M;
    }
    else if (PWRCTRL->SSRAMRETCFG_b.SSRAMPWDSLP == PWRCTRL_SSRAMRETCFG_SSRAMPWDSLP_GROUP0)
    {
        psConfig->eSRAMRetain = AM_HAL_PWRCTRL_SRAM_0P75M;
    }
    else if (PWRCTRL->SSRAMRETCFG_b.SSRAMPWDSLP == PWRCTRL_SSRAMRETCFG_SSRAMPWDSLP_GROUP1)
    {
        psConfig->eSRAMRetain = AM_HAL_PWRCTRL_SRAM_1M;
    }
    else
    {
        return AM_HAL_STATUS_OUT_OF_RANGE;
    }

    return AM_HAL_STATUS_SUCCESS;
} // am_hal_pwrctrl_sram_config_get()

//******************************************************************************
//
// Function that waits for crypto peripheral to stabilize before or after
// powerup/powerdown.
//
//******************************************************************************
#define CRYPTO_WAIT_USEC        100
static uint32_t
crypto_quiesce(void)
{
    uint32_t ui32Status;

    //
    // We do not know if the application has already set CRYPTO->HOSTPOWERDOWN
    // Wait for CRYPTOPWRDOWREADY, assuming it has been set
    //
    ui32Status = am_hal_delay_us_status_change(CRYPTO_WAIT_USEC,
                                                (uint32_t)&MCUCTRL->MRAMCRYPTOPWRCTRL,
                                                MCUCTRL_MRAMCRYPTOPWRCTRL_CRYPTOPWRDOWREADY_Msk,
                                                MCUCTRL_MRAMCRYPTOPWRCTRL_CRYPTOPWRDOWREADY_Msk);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        //
        // if this fails - it could be because CRYPTO->HOSTPOWERDOWN was never written to
        //
        //
        // Wait for crypto block to go idle.
        //
        ui32Status = am_hal_delay_us_status_change(CRYPTO_WAIT_USEC,
                                                   (uint32_t)&CRYPTO->HOSTCCISIDLE,
                                                   CRYPTO_HOSTCCISIDLE_HOSTCCISIDLE_Msk,
                                                   CRYPTO_HOSTCCISIDLE_HOSTCCISIDLE_Msk);
        if (AM_HAL_STATUS_SUCCESS == ui32Status)
        {

            //
            // Inform Crypto of imminent Power down
            //
            CRYPTO->HOSTPOWERDOWN_b.HOSTPOWERDOWN = 1;
            //
            // Wait for CRYPTOPWRDOWREADY
            //
            ui32Status = am_hal_delay_us_status_change(CRYPTO_WAIT_USEC,
                                                        (uint32_t)&MCUCTRL->MRAMCRYPTOPWRCTRL,
                                                        MCUCTRL_MRAMCRYPTOPWRCTRL_CRYPTOPWRDOWREADY_Msk,
                                                        MCUCTRL_MRAMCRYPTOPWRCTRL_CRYPTOPWRDOWREADY_Msk);
        }
    }

    return ui32Status;

} // crypto_quiesce()

// ****************************************************************************
//
//  am_hal_pwrctrl_periph_enable()
//  Enable power for a peripheral.
//
// ****************************************************************************
uint32_t
am_hal_pwrctrl_periph_enable(am_hal_pwrctrl_periph_e ePeripheral)
{
    uint32_t      ui32Status;
    struct am_pwr_s pwr_ctrl;

    ui32Status = am_get_pwrctrl(&pwr_ctrl, ePeripheral);

    if ( AM_HAL_STATUS_SUCCESS != ui32Status )
    {
        return ui32Status;
    }

    if ( AM_REGVAL(pwr_ctrl.ui32PwrEnRegAddr) & pwr_ctrl.ui32PeriphEnable )
    {
        //
        // We're already enabled, nothing to do.
        //
        return AM_HAL_STATUS_SUCCESS;
    }

    AM_CRITICAL_BEGIN

    //
    // Enable power control for the given device.
    //
    AM_REGVAL(pwr_ctrl.ui32PwrEnRegAddr) |= pwr_ctrl.ui32PeriphEnable;
    AM_CRITICAL_END

    ui32Status = am_hal_delay_us_status_check(AM_HAL_PWRCTRL_MAX_WAIT_US,
                                              pwr_ctrl.ui32PwrStatReqAddr,
                                              pwr_ctrl.ui32PeriphStatus,
                                              pwr_ctrl.ui32PeriphStatus,
                                              true);

    //
    // Check for timeout.
    //
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return ui32Status;
    }

    //
    // Crypto peripheral needs more time to power up after the normal status bit
    // is set. We'll wait for the IDLE signal from the NVM as our signal that
    // crypto is ready.
    //
    if (ePeripheral == AM_HAL_PWRCTRL_PERIPH_CRYPTO)
    {
        ui32Status = am_hal_delay_us_status_change(CRYPTO_WAIT_USEC,
                                                   (uint32_t)&CRYPTO->NVMISIDLE,
                                                   CRYPTO_NVMISIDLE_NVMISIDLEREG_Msk,
                                                   CRYPTO_NVMISIDLE_NVMISIDLEREG_Msk);

        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return ui32Status;
        }
    }

    if ( ePeripheral == AM_HAL_PWRCTRL_PERIPH_OTP)
    {
        // #### INTERNAL BEGIN ####
        // CAYNSWS-2001 2/26/24 It was found that after an OTP power up, additional
        // delay was required before a reliable read from OTP could occur.
        // Experimentation with Keil6 compiled HAL and using hello_world's Trim
        // Rev print showed that 45us was not enough delay, but that 50us worked.
        // For the time-being, at least for SDK6, we'll double that to 100us.
        // Note that the 100us delay was agreed to in a Teams chat between
        // Ron, Jayesh, and Eric Mao.
        // #### INTERNAL END ####
        //
        // Perform a short delay after OTP power up.
        //
        am_hal_delay_us(100);
    }

    //
    // Check the device status.
    //
    if ( (AM_REGVAL(pwr_ctrl.ui32PwrStatReqAddr) &
          pwr_ctrl.ui32PeriphStatus) != 0)
    {
        return AM_HAL_STATUS_SUCCESS;
    }
    else
    {
        return AM_HAL_STATUS_FAIL;
    }

} // am_hal_pwrctrl_periph_enable()

// ****************************************************************************
//
//  am_hal_pwrctrl_periph_disable_msk_check()
//  Function checks the PWRCTRL->DEVPWREN
//
//  The original check of ((PWRCTRL->DEVPWRSTATUS & ui32PeriphStatus) == 0)
//      will fail when more than one enable in the same domain is set and the
//      user tries disable only one.
//
// ****************************************************************************
static uint32_t
pwrctrl_periph_disable_msk_check(am_hal_pwrctrl_periph_e ePeripheral)
{
    uint32_t      ui32Status;
    struct am_pwr_s pwr_ctrl;

    ui32Status = am_get_pwrctrl(&pwr_ctrl, ePeripheral);

    if ( AM_HAL_STATUS_SUCCESS != ui32Status )
    {
        return ui32Status;
    }

    switch (pwr_ctrl.ui32PeriphStatus)
    {
        case (PWRCTRL_HCPA_DEVPWRSTATUS_MASK):
            if (((AM_REGVAL(pwr_ctrl.ui32PwrEnRegAddr) & PWRCTRL_HCPA_DEVPWREN_MASK) != 0) &&
                ((AM_REGVAL(pwr_ctrl.ui32PwrEnRegAddr) & pwr_ctrl.ui32PeriphEnable) == 0))
            {
                ui32Status = AM_HAL_STATUS_SUCCESS;
            }
            break;

        case (PWRCTRL_HCPB_DEVPWRSTATUS_MASK):
            if (((AM_REGVAL(pwr_ctrl.ui32PwrEnRegAddr) & PWRCTRL_HCPB_DEVPWREN_MASK) != 0) &&
                ((AM_REGVAL(pwr_ctrl.ui32PwrEnRegAddr) & pwr_ctrl.ui32PeriphEnable) == 0))
            {
                ui32Status = AM_HAL_STATUS_SUCCESS;
            }
            break;

        case (PWRCTRL_HCPC_DEVPWRSTATUS_MASK):
            if (((AM_REGVAL(pwr_ctrl.ui32PwrEnRegAddr) & PWRCTRL_HCPC_DEVPWREN_MASK) != 0) &&
                ((AM_REGVAL(pwr_ctrl.ui32PwrEnRegAddr) & pwr_ctrl.ui32PeriphEnable) == 0))
            {
                ui32Status = AM_HAL_STATUS_SUCCESS;
            }
            break;

        case (PWRCTRL_MSPI_DEVPWRSTATUS_MASK):
            if (((AM_REGVAL(pwr_ctrl.ui32PwrEnRegAddr) & PWRCTRL_MSPI_DEVPWREN_MASK) != 0) &&
                ((AM_REGVAL(pwr_ctrl.ui32PwrEnRegAddr) & pwr_ctrl.ui32PeriphEnable) == 0))
            {
                ui32Status = AM_HAL_STATUS_SUCCESS;
            }
            break;

        case (PWRCTRL_AUD_DEVPWRSTATUS_MASK):
            if (((AM_REGVAL(pwr_ctrl.ui32PwrEnRegAddr) & PWRCTRL_AUD_DEVPWREN_MASK) != 0) &&
                ((AM_REGVAL(pwr_ctrl.ui32PwrEnRegAddr) & pwr_ctrl.ui32PeriphEnable) == 0))
            {
                ui32Status = AM_HAL_STATUS_SUCCESS;
            }
            break;

        default:
            break;
    }

    return ui32Status;
} // pwrctrl_periph_disable_msk_check()

// ****************************************************************************
//
//  am_hal_pwrctrl_periph_disable()
//  Disable power for a peripheral.
//
// ****************************************************************************
uint32_t
am_hal_pwrctrl_periph_disable(am_hal_pwrctrl_periph_e ePeripheral)
{
    uint32_t      ui32Status;
    struct am_pwr_s pwr_ctrl;

    ui32Status = am_get_pwrctrl(&pwr_ctrl, ePeripheral);

    if ( AM_HAL_STATUS_SUCCESS != ui32Status )
    {
        return ui32Status;
    }

    if ( !(AM_REGVAL(pwr_ctrl.ui32PwrEnRegAddr) & pwr_ctrl.ui32PeriphEnable) )
    {
        //
        // We're already disabled, nothing to do.
        //
        return AM_HAL_STATUS_SUCCESS;
    }

    if (ePeripheral == AM_HAL_PWRCTRL_PERIPH_OTP)
    {
        // Need to wait for PTM status idle to ensure any outstanding OTP writes are done
        // #### INTERNAL BEGIN ####
        // CAYNSWS-1520
        // #### INTERNAL END ####

        ui32Status = am_hal_delay_us_status_check(AM_HAL_PWRCTRL_MAX_WAIT_OTP_US,
                                                  (uint32_t)&OTP->PTMSTAT,
                                                  OTP_PTMSTAT_BUSY_Msk,
                                                  0,
                                                  true);

        //
        // Check for timeout.
        //
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return ui32Status;
        }
    }

    AM_CRITICAL_BEGIN

    //
    // Disable power domain for the given device.
    //
    AM_REGVAL(pwr_ctrl.ui32PwrEnRegAddr) &= ~pwr_ctrl.ui32PeriphEnable;
    AM_CRITICAL_END

    //
    //  This check will fail when more than one enable in the same domain
    //      is set and the user tries to disable only one.
    //
    ui32Status = am_hal_delay_us_status_check(AM_HAL_PWRCTRL_MAX_WAIT_US,
                                              pwr_ctrl.ui32PwrStatReqAddr,
                                              pwr_ctrl.ui32PeriphStatus,
                                              pwr_ctrl.ui32PeriphStatus,
                                              false);




    //
    // Check for success.
    //
    if (AM_HAL_STATUS_SUCCESS == ui32Status)
    {
        return ui32Status;
    }
    else
    {
        return pwrctrl_periph_disable_msk_check(ePeripheral);
    }

} // am_hal_pwrctrl_periph_disable()

// ****************************************************************************
//
//  am_hal_pwrctrl_periph_enabled()
//  Determine whether a peripheral is currently enabled.
//
// ****************************************************************************
uint32_t
am_hal_pwrctrl_periph_enabled(am_hal_pwrctrl_periph_e ePeripheral,
                              bool *bEnabled)
{
    uint32_t      ui32Status;
    struct am_pwr_s pwr_ctrl;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    if ( bEnabled == NULL )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    //
    // Initialize bEnabled to false in case an error is encountered.
    //
    *bEnabled = false;

    ui32Status = am_get_pwrctrl(&pwr_ctrl, ePeripheral);

    if ( AM_HAL_STATUS_SUCCESS != ui32Status )
    {
        return ui32Status;
    }

    *bEnabled = ((AM_REGVAL(pwr_ctrl.ui32PwrStatReqAddr) &
                  pwr_ctrl.ui32PeriphStatus) != 0);

    return AM_HAL_STATUS_SUCCESS;

} // am_hal_pwrctrl_periph_enabled()

// ****************************************************************************
//
//  am_hal_pwrctrl_status_get()
//  Get the current powercontrol status registers.
//
// ****************************************************************************
uint32_t
am_hal_pwrctrl_status_get(am_hal_pwrctrl_status_t *psStatus)
{
    //
    // Device Power ON Status
    //
    psStatus->ui32Device = PWRCTRL->DEVPWRSTATUS;

    //
    // Audio Subsystem ON Status
    //
    psStatus->ui32AudioSS = PWRCTRL->AUDSSPWRSTATUS;

    //
    // MCU Memory Power ON Status
    //
    psStatus->ui32Memory = PWRCTRL->MEMPWRSTATUS;

    //
    // Power ON Status for MCU Core
    //
    psStatus->ui32System = PWRCTRL->SYSPWRSTATUS;

    //
    // Shared SRAM Power ON Status
    //
    psStatus->ui32SSRAM = PWRCTRL->SSRAMPWRST;

    //
    // Voltage Regulators status
    //
    psStatus->ui32VRStatus = PWRCTRL->VRSTATUS;

    //
    // Power Status Register for ADC Block
    //
    psStatus->ui32ADC = PWRCTRL->ADCSTATUS;

    return AM_HAL_STATUS_SUCCESS;

} // am_hal_pwrctrl_status_get()

// ****************************************************************************
//
//  Collect and store all INFO1 data, after which OTP can be turned off.
//
// ****************************************************************************
static uint32_t
pwrctrl_INFO1_populate(void)
{
    //
    // To optimizize power, we'll capture the INFO1 registers of interest here
    //  and power down OTP. The registers will then always be available.
    //

    uint32_t ui32Status;
    uint32_t info1buf[12];

//
// Helper macros for pwrctrl_INFO1_populate().
//  CHK_OFFSET_DELTA: Helper macro to assure continguousness of registers.
//  RD_INFO1: Macro to call am_hal_info1_read() and check return status.
//
#define CHK_OFFSET_DELTA(offh, offl, n)     STATIC_ASSERT((((offh - offl) / 4) + 1) != n)

#define RD_INFO1(infospace, wdoffset, numwds, pData)                    \
    ui32Status = am_hal_info1_read(infospace, wdoffset, numwds, pData); \
    if ( ui32Status != AM_HAL_STATUS_SUCCESS )                          \
    {                                                                   \
        return ui32Status;                                              \
    }

    if ( (MCUCTRL->SHADOWVALID_b.INFO1SELOTP != MCUCTRL_SHADOWVALID_INFO1SELOTP_VALID)  ||
         (PWRCTRL->DEVPWRSTATUS_b.PWRSTOTP   != PWRCTRL_DEVPWRSTATUS_PWRSTOTP_ON) )
    {
// #### INTERNAL BEGIN ####
        //
        // It might seem that we should mark the data as invalid here. However,
        // it's possible that somehow, this function was called twice. In that
        // case, we just want to return a fail but leave the data intact.
        //
        // g_sINFO1regs.ui32INFO1GlobalValid = 0;
// #### INTERNAL END ####
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

    CHK_OFFSET_DELTA(AM_REG_INFO1_SBL_VERSION_1_O, AM_REG_INFO1_SBL_VERSION_0_O, 2 );
    RD_INFO1(AM_HAL_INFO_INFOSPACE_MRAM_INFO1,    (AM_REG_INFO1_SBL_VERSION_0_O / 4), 2, &info1buf[0]);
    g_sINFO1regs.ui32SBLVer0                = info1buf[0];
    g_sINFO1regs.ui32SBLVer1                = info1buf[1];

    RD_INFO1(AM_HAL_INFO_INFOSPACE_CURRENT_INFO1, (AM_REG_OTP_INFO1_MAINPTR_O / 4), 1, &info1buf[0]);
    g_sINFO1regs.ui32MAINPTR                = info1buf[0];
    RD_INFO1(AM_HAL_INFO_INFOSPACE_CURRENT_INFO1, (AM_REG_OTP_INFO1_SBLOTA_O / 4), 1, &info1buf[0]);
    g_sINFO1regs.ui32SBLOTA                 = info1buf[0];

    CHK_OFFSET_DELTA( AM_REG_OTP_INFO1_SOCID7_O, AM_REG_OTP_INFO1_SOCID0_O, 8 );
    RD_INFO1(AM_HAL_INFO_INFOSPACE_OTP_INFO1,     (AM_REG_OTP_INFO1_SOCID0_O / 4), 8, &info1buf[0]);
    g_sINFO1regs.ui32SOCID0                 = info1buf[0];
    g_sINFO1regs.ui32SOCID1                 = info1buf[1];
    g_sINFO1regs.ui32SOCID2                 = info1buf[2];
    g_sINFO1regs.ui32SOCID3                 = info1buf[3];
    g_sINFO1regs.ui32SOCID4                 = info1buf[4];
    g_sINFO1regs.ui32SOCID5                 = info1buf[5];
    g_sINFO1regs.ui32SOCID6                 = info1buf[6];
    g_sINFO1regs.ui32SOCID7                 = info1buf[7];

    CHK_OFFSET_DELTA( AM_REG_OTP_INFO1_TEMP_CAL_ADC_OFFSET_O, AM_REG_OTP_INFO1_TEMP_CAL_ATE_O, 3);
    RD_INFO1(AM_HAL_INFO_INFOSPACE_CURRENT_INFO1, (AM_REG_OTP_INFO1_TEMP_CAL_ATE_O  / 4), 3, &info1buf[0]);
    g_sINFO1regs.ui32TEMP_CAL_ATE           = info1buf[0];
    g_sINFO1regs.ui32TEMP_CAL_MEASURED      = info1buf[1];
    g_sINFO1regs.ui32TEMP_CAL_ADC_OFFSET    = info1buf[2];

    CHK_OFFSET_DELTA(AM_REG_OTP_INFO1_ADC_OFFSET_ERR_O, AM_REG_OTP_INFO1_ADC_GAIN_ERR_O, 2 );
    RD_INFO1(AM_HAL_INFO_INFOSPACE_CURRENT_INFO1, (AM_REG_OTP_INFO1_ADC_GAIN_ERR_O  / 4), 2, &info1buf[0]);
    g_sINFO1regs.ui32ADC_GAIN_ERR           = info1buf[0];
    g_sINFO1regs.ui32ADC_OFFSET_ERR         = info1buf[1];

    CHK_OFFSET_DELTA(AM_REG_OTP_INFO1_AUDADC_B1_HG_INTERCEPT_O, AM_REG_OTP_INFO1_AUDADC_A0_LG_OFFSET_O, 12 );
    RD_INFO1(AM_HAL_INFO_INFOSPACE_CURRENT_INFO1, (AM_REG_OTP_INFO1_AUDADC_A0_LG_OFFSET_O  / 4), 12, &info1buf[0]);
    g_sINFO1regs.ui32AUDADC_A0_LG_OFFSET    = info1buf[0];
    g_sINFO1regs.ui32AUDADC_A0_HG_SLOPE     = info1buf[1];
    g_sINFO1regs.ui32AUDADC_A0_HG_INTERCEPT = info1buf[2];
    g_sINFO1regs.ui32AUDADC_A1_LG_OFFSET    = info1buf[3];
    g_sINFO1regs.ui32AUDADC_A1_HG_SLOPE     = info1buf[4];
    g_sINFO1regs.ui32AUDADC_A1_HG_INTERCEPT = info1buf[5];
    g_sINFO1regs.ui32AUDADC_B0_LG_OFFSET    = info1buf[6];
    g_sINFO1regs.ui32AUDADC_B0_HG_SLOPE     = info1buf[7];
    g_sINFO1regs.ui32AUDADC_B0_HG_INTERCEPT = info1buf[8];
    g_sINFO1regs.ui32AUDADC_B1_LG_OFFSET    = info1buf[9];
    g_sINFO1regs.ui32AUDADC_B1_HG_SLOPE     = info1buf[10];
    g_sINFO1regs.ui32AUDADC_B1_HG_INTERCEPT = info1buf[11];

    //
    // All done, mark the data as valid
    //
    g_sINFO1regs.ui32INFO1GlobalValid = INFO1GLOBALVALID;

    return ui32Status;

} // pwrctrl_INFO1_populate()

// ****************************************************************************
//
//  am_hal_pwrctrl_low_power_init()
//  Initialize the device for low power operation.
//
// ****************************************************************************
uint32_t
am_hal_pwrctrl_low_power_init(void)
{
    uint32_t ui32TrimVer = 0;

    //
    // Set the default CPDLPSTATE configuration in active mode.
    // Please refer to M55 TRM 5.17.1 for CPDLPSTATE definitions.
    //
    am_hal_pwrctrl_pwrmodctl_cpdlp_t sDefaultCpdlpConfig =
    {
        .eRlpConfig = AM_HAL_PWRCTRL_RLP_ON,
        .eElpConfig = AM_HAL_PWRCTRL_ELP_ON,
        .eClpConfig = AM_HAL_PWRCTRL_CLP_ON
    };
    am_hal_pwrctrl_pwrmodctl_cpdlp_config(sDefaultCpdlpConfig);

// #### INTERNAL BEGIN ####
#warning "TODO - FIXME: WICCONTROL register is removed from RevB. Need to double check"
#if 0
    //
    // Set the default EWIC/IWIC configuration.
    //
    MCUCTRL->WICCONTROL =
        _VAL2FLD(MCUCTRL_WICCONTROL_USEIWIC, 1) |
        _VAL2FLD(MCUCTRL_WICCONTROL_DEEPISWIC, 1);
#endif
// #### INTERNAL END ####

    //
    // Set the default memory configuration.
    //
    am_hal_pwrctrl_mcu_memory_config((am_hal_pwrctrl_mcu_memory_config_t *)&g_DefaultMcuMemCfg);
    am_hal_pwrctrl_sram_config((am_hal_pwrctrl_sram_memcfg_t *)&g_DefaultSRAMCfg);

    // #### INTERNAL BEGIN ####
    // CAYNSWS-2671 6/24/24
    //  For Cayenne revB, Crypto must be powered down in order to power down OTP.
    //  For revA and 510L, it's not clear whether Crypto powerdown is required,
    //  but to keep everything in sync, power down Crypto nonetheless.
    // #### INTERNAL END ####
    //
    // For power optimization, power down OTP.
    // Since OTP is being powered down, first power down Crypto.
    //
    // If INFO1 is currently selected as OTP, collect and save INFO1 data
    // for use by HAL functions. This will eliminate the need to have OTP
    // powered. If INFO1 is MRAM, any INFO1 data needed by HAL functions
    // will simply be read directly via am_hal_info1_read().
    //
    pwrctrl_INFO1_populate();

    //
    // Power down Crypto.
    //
    am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_CRYPTO);

    //
    // Power down OTP.
    //
    am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_OTP);


    //
    // Now that we have the INFO1 data saved, power down the OTP
    //
    am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_OTP);

    //
    // Enable clock gate optimizations for Apollo4.
    //
    // #### INTERNAL BEGIN ####
    //  Bits Equivalent to CLKGEN->MISC_b.CLKGENMISCSPARES = 0x3F
    // #### INTERNAL END ####
    CLKGEN->MISC |=
        _VAL2FLD(CLKGEN_MISC_CM4DAXICLKGATEEN, 1)          |   // [18] DAXI CLK
        _VAL2FLD(CLKGEN_MISC_GFXCLKCLKGATEEN, 1)        |   // [19] GFX CLK
        _VAL2FLD(CLKGEN_MISC_GFXAXICLKCLKGATEEN, 1)     |   // [20] GFX AXI CLK
        _VAL2FLD(CLKGEN_MISC_APBDMACPUCLKCLKGATEEN, 1)  |   // [21] APB DMA CPU CLK
        _VAL2FLD(CLKGEN_MISC_ETMTRACECLKCLKGATEEN, 1)   |   // [22] ETM TRACE CLK
        _VAL2FLD(CLKGEN_MISC_HFRCFUNCCLKGATEEN, 1);         // [23] HFRC_FUNC_CLK

    //
    //  Enable the I-Cache and D-Cache.
    //
    // #### INTERNAL BEGIN ####
    #warning Move out cache enabling from here to respective applications.
    // #### INTERNAL END ####
    am_hal_cachectrl_icache_enable();
    am_hal_cachectrl_dcache_enable(true);

    //
    // Additional required settings
    //
    // #### INTERNAL BEGIN ####
    // HSP20-429 PWRONCLKENDISP Needed for DC workaround, disables revB clock enable
    //           during reset, basically reverting to revA behavior.
    // #### INTERNAL END ####
// #### INTERNAL BEGIN ####
#if 0 // DISP is removed from 20240319 RTL. May be added back later.
    CLKGEN->MISC_b.PWRONCLKENDISP = 1;
#endif
// #### INTERNAL END ####

#if 1
// #### INTERNAL BEGIN ####
#if 0 // DISP is removed from 20240319 RTL. May be added back later.
    // #### INTERNAL BEGIN ####
    // FALCSW-176.
    // #### INTERNAL END ####
    // Increases the reference recovery time between scans in LPMODE1 from 5us to 10us.
    MCUCTRL->AUDADCPWRDLY_b.AUDADCPWR1 = 4;
#endif
// #### INTERNAL END ####

    //
    // Store the factory values for various trims.
    //
    if ( g_bOrigTrimsStored == false )
    {
// #### INTERNAL BEGIN ####
#warning "Fixme!!! Need to update to reflect the changes in MCUCTRL register fields."
// #### INTERNAL END ####
        g_orig_MEMLDOACTIVETRIM     = MCUCTRL->LDOREG2_b.MEMLDOACTIVETRIM;
        g_orig_MEMLPLDOTRIM         = MCUCTRL->LDOREG2_b.MEMLDOLPTRIM;
        g_orig_TVRGVREFTRIM         = MCUCTRL->VREFGEN2_b.TVRGCVREFTRIM;
        g_bOrigTrimsStored          = true;
    }
#endif

    //
    // Get trim version
    //
    TrimVersionGet(&ui32TrimVer);

// #### INTERNAL BEGIN ####
    //
    // Workaround for SWO no printf on FPGA image.
    //
    PWRCTRL->DEVPWREN_b.PWRENDBG = PWRCTRL_DEVPWREN_PWRENDBG_EN;
// #### INTERNAL END ####

    return AM_HAL_STATUS_SUCCESS;

} // am_hal_pwrctrl_low_power_init()

void
buck_ldo_override_init(void)
{
    //
    // Force SIMOBUCK into active mode. SIMOBUCKOVER must be set last.
    // This override, even though enabled - is not effective till we go to DeepSleep
    //
    MCUCTRL->VRCTRL_b.SIMOBUCKPDNB   = 1;
    MCUCTRL->VRCTRL_b.SIMOBUCKRSTB   = 1;
    MCUCTRL->VRCTRL_b.SIMOBUCKACTIVE = 1;
    MCUCTRL->VRCTRL_b.SIMOBUCKOVER   = 1;

#if AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL
    //
    // Force LDOs into active mode and to run in parallel with SIMO.
    //
    //
    // Core LDO. Set CORELDOOVER last
    //
    MCUCTRL->VRCTRL_b.CORELDOCOLDSTARTEN  = 0;
    MCUCTRL->VRCTRL |=
        MCUCTRL_VRCTRL_CORELDOACTIVE_Msk        |
        MCUCTRL_VRCTRL_CORELDOACTIVEEARLY_Msk   |
        MCUCTRL_VRCTRL_CORELDOPDNB_Msk;
    MCUCTRL->VRCTRL_b.CORELDOOVER         = 1;
    //
    // Mem LDO. Set MEMLDOOVER last
    //
    MCUCTRL->VRCTRL_b.MEMLDOCOLDSTARTEN   = 0;
    MCUCTRL->VRCTRL |=
        MCUCTRL_VRCTRL_MEMLDOACTIVE_Msk         |
        MCUCTRL_VRCTRL_MEMLDOACTIVEEARLY_Msk    |
        MCUCTRL_VRCTRL_MEMLDOPDNB_Msk;
    MCUCTRL->VRCTRL_b.MEMLDOOVER          = 1;
#endif // AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL
} // buck_ldo_override_init()

// #### INTERNAL BEGIN ####
#if 0 // Not needed with latest updates
#if AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL
// ****************************************************************************
//
//  coreldo_enable()
//  Function to set CORELDOTEMPCOTRIM and force CORELDO to be active
//
// ****************************************************************************
void
coreldo_enable(void)
{
    //
    // Enable coreldo
    // There are timing constraints involved in getting this done,
    // so optimize as much as possible.
    //
// #### INTERNAL2 BEGIN ####
    // 10/29/21 Ivan and Jamie: coldstarten is used to limit inrush current of
    // the LDO on initial power up. Technically it's not needed when used for
    // backup / running in parallel, but we should set the same as rev B.
// #### INTERNAL2 END ####
    MCUCTRL->VRCTRL_b.CORELDOCOLDSTARTEN  = 0;
    MCUCTRL->VRCTRL |=
        MCUCTRL_VRCTRL_CORELDOACTIVE_Msk        |
        MCUCTRL_VRCTRL_CORELDOACTIVEEARLY_Msk   |
        MCUCTRL_VRCTRL_CORELDOPDNB_Msk;
    MCUCTRL->VRCTRL_b.CORELDOOVER         = 1;
} // coreldo_enable()

// ****************************************************************************
//
//  coreldo_disable()
//  Function to turn off coreldo
//
// ****************************************************************************
void
coreldo_disable(void)
{
    //
    // Disable coreldo
    //
    MCUCTRL->VRCTRL_b.CORELDOOVER         = 0;
    MCUCTRL->VRCTRL &=
        ~(MCUCTRL_VRCTRL_CORELDOACTIVE_Msk      |
          MCUCTRL_VRCTRL_CORELDOACTIVEEARLY_Msk |
          MCUCTRL_VRCTRL_CORELDOPDNB_Msk);
} // coreldo_disable()

// ****************************************************************************
//
//  memldo_enable()
//  Function to force MEMLDO to be active
//
// ****************************************************************************
void
memldo_enable(void)
{
    //
    // Enable memldo
    // There are timing constraints involved in getting this done,
    // so optimize as much as possible.
    // However, MEMLDOOVER must be the last bit set.
    //
// #### INTERNAL2 BEGIN ####
    // 10/29/21 See "coldstarten" comment above.
// #### INTERNAL2 END ####
    MCUCTRL->VRCTRL_b.MEMLDOCOLDSTARTEN   = 0;
    MCUCTRL->VRCTRL |=
        MCUCTRL_VRCTRL_MEMLDOACTIVE_Msk         |
        MCUCTRL_VRCTRL_MEMLDOACTIVEEARLY_Msk    |
        MCUCTRL_VRCTRL_MEMLDOPDNB_Msk;
    MCUCTRL->VRCTRL_b.MEMLDOOVER          = 1;
} // memldo_enable()

// ****************************************************************************
//
//  memldo_disable()
//  Function to turn off memldo
//
// ****************************************************************************
void
memldo_disable(void)
{
    //
    // disable memldo
    //
    MCUCTRL->VRCTRL_b.MEMLDOOVER          = 0;
    MCUCTRL->VRCTRL &=
        ~(MCUCTRL_VRCTRL_MEMLDOACTIVE_Msk         |
          MCUCTRL_VRCTRL_MEMLDOACTIVEEARLY_Msk    |
          MCUCTRL_VRCTRL_MEMLDOPDNB_Msk);
} // memldo_disable()
#endif // AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL

void
simobuck_ldos_force_active(void)
{
    //
    // Force SIMOBUCK into active mode. SIMOBUCKOVER must be set last.
    //
    MCUCTRL->VRCTRL_b.SIMOBUCKPDNB   = 1;
    MCUCTRL->VRCTRL_b.SIMOBUCKRSTB   = 1;
    MCUCTRL->VRCTRL_b.SIMOBUCKACTIVE = 1;
    MCUCTRL->VRCTRL_b.SIMOBUCKOVER   = 1;

#if AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL
    //
    // Also force LDOs to active in deepsleep
    //
    MCUCTRL->VRCTRL_b.CORELDOPDNB    = 1;
    MCUCTRL->VRCTRL_b.CORELDOACTIVE  = 1;
    MCUCTRL->VRCTRL_b.CORELDOOVER    = 1;
    MCUCTRL->VRCTRL_b.MEMLDOPDNB     = 1;
    MCUCTRL->VRCTRL_b.MEMLDOACTIVE   = 1;
    MCUCTRL->VRCTRL_b.MEMLDOOVER     = 1;
#endif // AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL
} // simobuck_ldos_force_active()

void
simobuck_ldos_force_disable(void)
{
    MCUCTRL->VRCTRL_b.SIMOBUCKOVER   = 0;
    MCUCTRL->VRCTRL_b.SIMOBUCKACTIVE = 0;
    MCUCTRL->VRCTRL_b.SIMOBUCKRSTB   = 0;
    MCUCTRL->VRCTRL_b.SIMOBUCKPDNB   = 0;

#if AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL
    MCUCTRL->VRCTRL_b.CORELDOOVER    = 0;
    MCUCTRL->VRCTRL_b.MEMLDOOVER     = 0;
    MCUCTRL->VRCTRL_b.CORELDOACTIVE  = 0;
    MCUCTRL->VRCTRL_b.CORELDOPDNB    = 0;
    MCUCTRL->VRCTRL_b.MEMLDOACTIVE   = 0;
    MCUCTRL->VRCTRL_b.MEMLDOPDNB     = 0;
#endif // AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL
} // simobuck_ldos_force_disable()
#endif // if 0
// #### INTERNAL END ####

// Dynamically turn on and off the overrides for buck and LDO
// Override configs are already set once in buck_ldo_override_init
void
buck_ldo_update_override(bool bEnable)
{
    MCUCTRL->VRCTRL_b.SIMOBUCKOVER   = bEnable;
#if AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL
    MCUCTRL->VRCTRL_b.CORELDOOVER    = bEnable;
    MCUCTRL->VRCTRL_b.MEMLDOOVER     = bEnable;
#endif // AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL
} // buck_ldo_update_override()

// ****************************************************************************
//
//  am_hal_pwrctrl_control()
//  Additional miscellaneous power controls.
//
// ****************************************************************************
uint32_t
am_hal_pwrctrl_control(am_hal_pwrctrl_control_e eControl, void *pArgs)
{
    uint32_t ui32ReturnStatus = AM_HAL_STATUS_SUCCESS;

    switch ( eControl )
    {
        case AM_HAL_PWRCTRL_CONTROL_SIMOBUCK_INIT:
            {
                //
                // Set Set VDDC active low and high TON trim.
                //
                MCUCTRL->VRCTRL_b.SIMOBUCKPDNB   = 1;
                MCUCTRL->VRCTRL_b.SIMOBUCKRSTB   = 1;
                MCUCTRL->VRCTRL_b.SIMOBUCKACTIVE = 1;
                MCUCTRL->VRCTRL_b.SIMOBUCKOVER   = 1;

                //
                // Force LDOs into active mode and to run in parallel with SIMO.
                // Coreldo override
                //
                MCUCTRL->VRCTRL_b.CORELDOCOLDSTARTEN  = 0;
                MCUCTRL->VRCTRL |=
                    MCUCTRL_VRCTRL_CORELDOACTIVE_Msk        |
                    MCUCTRL_VRCTRL_CORELDOACTIVEEARLY_Msk   |
                    MCUCTRL_VRCTRL_CORELDOPDNB_Msk;
                MCUCTRL->VRCTRL_b.CORELDOOVER         = 1;

                //
                // Memldo override
                //
                MCUCTRL->VRCTRL_b.MEMLDOCOLDSTARTEN   = 0;
                MCUCTRL->VRCTRL |=
                    MCUCTRL_VRCTRL_MEMLDOACTIVE_Msk         |
                    MCUCTRL_VRCTRL_MEMLDOACTIVEEARLY_Msk    |
                    MCUCTRL_VRCTRL_MEMLDOPDNB_Msk;
                MCUCTRL->VRCTRL_b.MEMLDOOVER          = 1;

                // #### INTERNAL BEGIN ####
                // *(uint32_t*)0x4004033c=0xf; from validation team
                // #### INTERNAL END ####
                //
                // Enable SIMOBUCK compensations
                //
                MCUCTRL->SIMOBUCK0 |=
                    MCUCTRL_SIMOBUCK0_VDDCRXCOMPEN_Msk |
                    MCUCTRL_SIMOBUCK0_VDDFRXCOMPEN_Msk |
                    MCUCTRL_SIMOBUCK0_VDDSRXCOMPEN_Msk |
                    MCUCTRL_SIMOBUCK0_VDDCLVRXCOMPEN_Msk;

#warning "TODO - FIXME: SIMOBUCKEN is removed from apollo5b.h. "
                //
                // Enable the SIMOBUCK
                //
                PWRCTRL->VRCTRL |= 0x1;
            }
            break;

            // Use am_hal_pwrctrl_control(AM_HAL_PWRCTRL_CONTROL_CRYPTO_POWERDOWN) if using crypto
        case AM_HAL_PWRCTRL_CONTROL_CRYPTO_POWERDOWN:
            {
                uint32_t    ui32Status;
                bool        bEnabled;

                //
                // Check if CRYPTO block is powered on.
                //
                bEnabled = false;
                am_hal_pwrctrl_periph_enabled(AM_HAL_PWRCTRL_PERIPH_CRYPTO, &bEnabled);
                if ( bEnabled )
                {

                    //
                    // Prepare Crypto for PowerDown & wait for it to be ready
                    //
                    ui32Status = crypto_quiesce();

                    if (AM_HAL_STATUS_SUCCESS != ui32Status)
                    {
                        return ui32Status;
                    }
                    //
                    // Power down the crypto block
                    //
                    ui32Status = am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_CRYPTO);
                    if (AM_HAL_STATUS_SUCCESS != ui32Status)
                    {
                        return ui32Status;
                    }
                }
            }
            break;

        case AM_HAL_PWRCTRL_CONTROL_XTAL_PWDN_DEEPSLEEP:
            //
            // This optimization is optional. Enable it IFF the 32KHz crystal is
            // not required during deep sleep. If enabled it will save ~0.8uA.
            //
            MCUCTRL->XTALGENCTRL_b.XTALBIASTRIM   = 0x20;

// #### INTERNAL BEGIN ####
            // TODO FIXME: Is this intended to be a write to the field or the
            //      entire register? The workaround code from validation simply
            //      wrote the value 1 to the register.
            // Answer: Assuming the question is in reference to the following write
            //         to XTALCTRL, it is indeed writing the value 0x1 to the register.
            //         It's just more explicit than a plain, hard-coded value. - REH
// #### INTERNAL END ####
            MCUCTRL->XTALCTRL =
                _VAL2FLD(MCUCTRL_XTALCTRL_XTALICOMPTRIM,  0 )                                       |
                _VAL2FLD(MCUCTRL_XTALCTRL_XTALIBUFTRIM,   0 )                                       |
                _VAL2FLD(MCUCTRL_XTALCTRL_XTALCOMPPDNB,   MCUCTRL_XTALCTRL_XTALCOMPPDNB_PWRDNCOMP ) |
                _VAL2FLD(MCUCTRL_XTALCTRL_XTALPDNB,       MCUCTRL_XTALCTRL_XTALPDNB_PWRDNCORE )     |
                _VAL2FLD(MCUCTRL_XTALCTRL_XTALCOMPBYPASS, MCUCTRL_XTALCTRL_XTALCOMPBYPASS_USECOMP ) |
                _VAL2FLD(MCUCTRL_XTALCTRL_XTALCOREDISFB,  MCUCTRL_XTALCTRL_XTALCOREDISFB_EN )       |
                _VAL2FLD(MCUCTRL_XTALCTRL_XTALSWE,        MCUCTRL_XTALCTRL_XTALSWE_OVERRIDE_EN);
            break;

        case AM_HAL_PWRCTRL_CONTROL_DIS_PERIPHS_ALL:
            PWRCTRL->DEVPWREN =
// #### INTERNAL BEGIN ####
#if 0   // A2SD-2003 deprecate I3C enable/status registers/fields
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENI3C1,    PWRCTRL_DEVPWREN_PWRENI3C1_DIS)         |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENI3C0,    PWRCTRL_DEVPWREN_PWRENI3C0_DIS)         |
#endif
// #### INTERNAL END ####
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENI3C,     PWRCTRL_DEVPWREN_PWRENI3C_DIS)          |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENDBG,     PWRCTRL_DEVPWREN_PWRENDBG_DIS)          |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENUSBPHY,  PWRCTRL_DEVPWREN_PWRENUSBPHY_DIS)       |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENUSB,     PWRCTRL_DEVPWREN_PWRENUSB_DIS)          |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENSDIO1,   PWRCTRL_DEVPWREN_PWRENSDIO1_DIS)        |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENSDIO0,   PWRCTRL_DEVPWREN_PWRENSDIO0_DIS)        |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENCRYPTO,  PWRCTRL_DEVPWREN_PWRENCRYPTO_DIS)       |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENDISPPHY, PWRCTRL_DEVPWREN_PWRENDISPPHY_DIS)      |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENDISP,    PWRCTRL_DEVPWREN_PWRENDISP_DIS)         |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENGFX,     PWRCTRL_DEVPWREN_PWRENGFX_DIS)          |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENMSPI2,   PWRCTRL_DEVPWREN_PWRENMSPI2_DIS)        |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENMSPI1,   PWRCTRL_DEVPWREN_PWRENMSPI1_DIS)        |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENMSPI0,   PWRCTRL_DEVPWREN_PWRENMSPI0_DIS)        |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENADC,     PWRCTRL_DEVPWREN_PWRENADC_DIS)          |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENUART1,   PWRCTRL_DEVPWREN_PWRENUART1_DIS)        |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENUART0,   PWRCTRL_DEVPWREN_PWRENUART0_DIS)        |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENI3CPHY,  PWRCTRL_DEVPWREN_PWRENI3CPHY_DIS)       |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM5,    PWRCTRL_DEVPWREN_PWRENIOM5_DIS)         |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM4,    PWRCTRL_DEVPWREN_PWRENIOM4_DIS)         |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM3,    PWRCTRL_DEVPWREN_PWRENIOM3_DIS)         |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM2,    PWRCTRL_DEVPWREN_PWRENIOM2_DIS)         |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM1,    PWRCTRL_DEVPWREN_PWRENIOM1_DIS)         |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM0,    PWRCTRL_DEVPWREN_PWRENIOM0_DIS);
// #### INTERNAL BEGIN ####
#if 0 // Probably not needed, no Coremark difference.
            //
            // Update only certain fields of this register.
            //
            MCUCTRL->AUDADCPWRCTRL &=
                ~(MCUCTRL_AUDADCPWRCTRL_AUDADCPWRCTRLSWE_Msk    |
                  MCUCTRL_AUDADCPWRCTRL_VDDAUDADCRESETN_Msk     |
                  MCUCTRL_AUDADCPWRCTRL_VDDAUDADCDIGISOLATE_Msk |
                  MCUCTRL_AUDADCPWRCTRL_VDDAUDADCSARISOLATE_Msk |
                  MCUCTRL_AUDADCPWRCTRL_AUDADCBPSEN_Msk         |
                  MCUCTRL_AUDADCPWRCTRL_AUDADCAPSEN_Msk);
            MCUCTRL->AUDADCPWRCTRL |=
                _VAL2FLD(MCUCTRL_AUDADCPWRCTRL_AUDADCPWRCTRLSWE,    MCUCTRL_AUDADCPWRCTRL_AUDADCPWRCTRLSWE_OVERRIDE_DIS)    |
                _VAL2FLD(MCUCTRL_AUDADCPWRCTRL_VDDAUDADCRESETN,     MCUCTRL_AUDADCPWRCTRL_VDDAUDADCRESETN_ASSERT)           |
                _VAL2FLD(MCUCTRL_AUDADCPWRCTRL_VDDAUDADCDIGISOLATE, MCUCTRL_AUDADCPWRCTRL_VDDAUDADCDIGISOLATE_DIS)          |
                _VAL2FLD(MCUCTRL_AUDADCPWRCTRL_VDDAUDADCSARISOLATE, MCUCTRL_AUDADCPWRCTRL_VDDAUDADCSARISOLATE_DIS)          |
                _VAL2FLD(MCUCTRL_AUDADCPWRCTRL_AUDADCBPSEN,         MCUCTRL_AUDADCPWRCTRL_AUDADCBPSEN_DIS)                  |
                _VAL2FLD(MCUCTRL_AUDADCPWRCTRL_AUDADCAPSEN,         MCUCTRL_AUDADCPWRCTRL_AUDADCAPSEN_DIS);

            //
            // Update only certain fields of this register.
            //
            MCUCTRL->ADCPWRCTRL &=
                ~(MCUCTRL_ADCPWRCTRL_VDDADCRESETN_Msk       |
                  MCUCTRL_ADCPWRCTRL_VDDADCDIGISOLATE_Msk   |
                  MCUCTRL_ADCPWRCTRL_VDDADCSARISOLATE_Msk   |
                  MCUCTRL_ADCPWRCTRL_ADCBPSEN_Msk);
            MCUCTRL->ADCPWRCTRL |=
                _VAL2FLD(MCUCTRL_ADCPWRCTRL_VDDADCRESETN, MCUCTRL_ADCPWRCTRL_VDDADCRESETN_ASSERT)       |
                _VAL2FLD(MCUCTRL_ADCPWRCTRL_VDDADCDIGISOLATE, MCUCTRL_ADCPWRCTRL_VDDADCDIGISOLATE_DIS)  |
                _VAL2FLD(MCUCTRL_ADCPWRCTRL_VDDADCSARISOLATE, MCUCTRL_ADCPWRCTRL_VDDADCSARISOLATE_DIS)  |
                _VAL2FLD(MCUCTRL_ADCPWRCTRL_ADCBPSEN, MCUCTRL_ADCPWRCTRL_ADCBPSEN_DIS);
#endif
#if 0 // Probably not needed, should be handled in the appl.
    //
    // Configure the Shared RAM domain active state settings.
    //
    PWRCTRL->SSRAMRETCFG_b.SSRAMACTDISP = AM_HAL_PWRCTRL_SRAM_NONE;
    PWRCTRL->SSRAMRETCFG_b.SSRAMACTGFX  = AM_HAL_PWRCTRL_SRAM_NONE;
    PWRCTRL->SSRAMRETCFG_b.SSRAMACTDSP  = AM_HAL_PWRCTRL_SRAM_NONE;
    PWRCTRL->SSRAMRETCFG_b.SSRAMACTMCU  = AM_HAL_PWRCTRL_SRAM_NONE;
    PWRCTRL->SSRAMRETCFG_b.SSRAMPWDSLP  = AM_HAL_PWRCTRL_SRAM_1M;
#endif
// #### INTERNAL END ####
            break;

// #### INTERNAL BEGIN ####
#if 0 // Note: As of Apollo5b 26SEPT, PWRENAUDPB, PWRENAUDREC, PWRSTAUDPB, PWRSTAUDREC no longer exist.
        case IF_USED_THIS_WOULD_BE_A_NEW_CASE:
            PWRCTRL->MEMPWREN =
                _VAL2FLD(PWRCTRL_MEMPWREN_PWRENCACHEB2, PWRCTRL_AUDSSPWRSTATUS_PWRSTAUDADC_ON)  |
                _VAL2FLD(PWRCTRL_MEMPWREN_PWRENCACHEB0, PWRCTRL_MEMPWREN_PWRENCACHEB0_EN)       |
                _VAL2FLD(PWRCTRL_MEMPWREN_PWRENNVM0, PWRCTRL_MEMPWREN_PWRENNVM0_EN)             |
                _VAL2FLD(PWRCTRL_MEMPWREN_PWRENDTCM, PWRCTRL_MEMPWREN_PWRENDTCM_TCM384K);

            PWRCTRL->SSRAMPWREN =
                _VAL2FLD(PWRCTRL_SSRAMPWREN_PWRENSSRAM, PWRCTRL_SSRAMPWREN_PWRENSSRAM_ALL);

            PWRCTRL->AUDSSPWREN =
                _VAL2FLD(PWRCTRL_AUDSSPWREN_PWRENDSPA,   PWRCTRL_AUDSSPWREN_PWRENDSPA_DIS)      |
                _VAL2FLD(PWRCTRL_AUDSSPWREN_PWRENAUDADC, PWRCTRL_AUDSSPWREN_PWRENAUDADC_DIS)    |
                _VAL2FLD(PWRCTRL_AUDSSPWREN_PWRENI2S1,   PWRCTRL_AUDSSPWREN_PWRENI2S1_DIS)      |
                _VAL2FLD(PWRCTRL_AUDSSPWREN_PWRENI2S0,   PWRCTRL_AUDSSPWREN_PWRENI2S0_DIS)      |
                _VAL2FLD(PWRCTRL_AUDSSPWREN_PWRENPDM3,   PWRCTRL_AUDSSPWREN_PWRENPDM3_DIS)      |
                _VAL2FLD(PWRCTRL_AUDSSPWREN_PWRENPDM2,   PWRCTRL_AUDSSPWREN_PWRENPDM2_DIS)      |
                _VAL2FLD(PWRCTRL_AUDSSPWREN_PWRENPDM1,   PWRCTRL_AUDSSPWREN_PWRENPDM1_DIS)      |
                _VAL2FLD(PWRCTRL_AUDSSPWREN_PWRENPDM0,   PWRCTRL_AUDSSPWREN_PWRENPDM0_DIS)      |
                _VAL2FLD(PWRCTRL_AUDSSPWREN_PWRENAUDPB,  PWRCTRL_AUDSSPWREN_PWRENAUDPB_DIS)     |
                _VAL2FLD(PWRCTRL_AUDSSPWREN_PWRENAUDREC, PWRCTRL_AUDSSPWREN_PWRENAUDREC_DIS);
            break;
#endif
// #### INTERNAL END ####
#if AM_HAL_TEMPCO_LP
        case AM_HAL_PWRCTRL_CONTROL_TEMPCO_GETMEASTEMP:
            if ( pArgs )
            {
                *((float*)pArgs) = g_pfTempMeasured;
            }
#endif // AM_HAL_TEMPCO_LP

        case AM_HAL_PWRCTRL_CONTROL_DEEPERSLEEP_ENABLE:
            PWRCTRL->CPUPWRCTRL_b.DEEPERSLEEPEN = 1;
            break;

        case AM_HAL_PWRCTRL_CONTROL_DEEPERSLEEP_DISABLE:
            PWRCTRL->CPUPWRCTRL_b.DEEPERSLEEPEN = 0;
            break;

        default:
            ui32ReturnStatus = AM_HAL_STATUS_INVALID_ARG;
            break;
    }

    //
    // Return success status.
    //
    return ui32ReturnStatus;

} // am_hal_pwrctrl_control()

// ****************************************************************************
// Function to restore factory trims.
// ****************************************************************************
static void
restore_factory_trims(void)
{
    if ( g_bOrigTrimsStored )
    {
        //
        // Restore the original factory trim values
        //
// #### INTERNAL BEGIN ####
#warning "Fixe me!!! Need to update based on MCUCTRL register field changes."
// #### INTERNAL END ####
        MCUCTRL->LDOREG2_b.MEMLDOACTIVETRIM = g_orig_MEMLDOACTIVETRIM;
        MCUCTRL->LDOREG2_b.MEMLDOLPTRIM     = g_orig_MEMLPLDOTRIM;
        MCUCTRL->VREFGEN2_b.TVRGCVREFTRIM    = g_orig_TVRGVREFTRIM;
    }
} // restore_factory_trims()

//*****************************************************************************
//
// Restore original power settings
//
// This function restores default power trims, reverting relative changes that
// were done as part of am_hal_pwrctrl_low_power_init, SIMOBUCK init, and
// dynamic updates such as are made with Temperature Compensation (TempCo)
// and/or by enabling Crypto.
//
// Important:
// - This function must be called before transition to a new application, such
//   as the case of a secondary bootloader transistioning to an application.
// - If previously enabled, TempCo must be disabled before this function is
//   called.
//
//*****************************************************************************
uint32_t
am_hal_pwrctrl_settings_restore(void)
{
    uint32_t ui32Ret;

// #### INTERNAL BEGIN ####
    // Switch from HP to LP mode (if in HP mode)
    // Switch to LDO mode (if not already in LDO mode)
    // Delay 20us for rails to settle
    // Disable LDOs in parallel (if override was set to enabled this in simobuck mode)
    // Remove VDDS/VDDF short
    // Turn on Crypto if not already on.
// #### INTERNAL END ####
    //
    // Ensure - we're in LP mode
    //
    if (PWRCTRL->MCUPERFREQ_b.MCUPERFREQ != AM_HAL_PWRCTRL_MCU_MODE_LOW_POWER)
    {
        // Device needs to be in LP mode before restore is called
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

    //
    // Need to revert the trim changes, and turn to LDO mode
    //
    AM_CRITICAL_BEGIN

    //
    // Switch to LDO mode (if not already in LDO mode)
    // Delay 20us for rails to settle
    //
    if (PWRCTRL->VRSTATUS_b.SIMOBUCKST == PWRCTRL_VRSTATUS_SIMOBUCKST_ACT)
    {
#if AM_HAL_PWRCTL_SHORT_VDDF_TO_VDDS
        if (g_ui32origSimobuckVDDStrim != 0xFFFFFFFF)
        {
            MCUCTRL->SIMOBUCK13_b.ACTTRIMVDDS = g_ui32origSimobuckVDDStrim;
        }

        //
        // Remove VDDS/VDDF short
        //
        MCUCTRL->PWRSW1_b.SHORTVDDFVDDSORVAL  = 0;
        MCUCTRL->PWRSW1_b.SHORTVDDFVDDSOREN   = 0;
#endif

// #### INTERNAL BEGIN ####
#if 0 // This is not needed, as LDO's should already be in parallel
#if AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL
#if AM_HAL_PWRCTRL_LDOS_FOR_VDDC
        // #### INTERNAL2 BEGIN ####
        // A2SD-2248 Activate LDOs in parallel when disabling SIMOBUCK.
        // #### INTERNAL2 END ####
        simobuck_ldos_force_active();  // Force LDOs in parallel with simobuck
#else
        coreldo_disable();
        memldo_disable();
#endif // AM_HAL_PWRCTRL_LDOS_FOR_VDDC
#endif
#endif // if 0
#warning "Fix me!!! - VRCTRL is not defined in PWRCTRL registers."
#warning "Fixme!!! - VRCTRL is NOW defined in PWRCTRL registers. Do we still want to enable SIMOBUCKEN??"
// #### INTERNAL END ####
        //PWRCTRL->VRCTRL_b.SIMOBUCKEN = 0;
        //
        // Need to remove overrides
        //
        buck_ldo_update_override(false);
    }

    //
    // Re-enable Crypto if not already on.
    // The enable function will check whether it's enabled or not.
    //
    ui32Ret = am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_CRYPTO);

    //
    // Restore original factory trims.
    // This will apply whether or not AM_HAL_TEMPCO_LP is activated.
    //
    restore_factory_trims();

    AM_CRITICAL_END

    return ui32Ret;

} // am_hal_pwrctrl_settings_restore()

#if AM_HAL_TEMPCO_LP
// ****************************************************************************
//
//  am_hal_pwrctrl_tempco_init()
//  ui32UpdateInterval - Time in seconds. 10 is recommended.
//
//  User must also call am_hal_adc_initialize().
//
// ****************************************************************************
uint32_t
am_hal_pwrctrl_tempco_init(void *pADCHandle,
                           uint32_t ui32ADCslot)
{
    uint32_t                    ui32Retval;
    am_hal_adc_slot_config_t    sSlotCfg;
    uint32_t ui32TrimVer;

    uint32_t ui32Temp[3];

    //
    // First make sure the temperature calibration is valid.
    //
    ui32Retval = am_hal_info1_read(AM_HAL_INFO_INFOSPACE_CURRENT_INFO1,
                                   AM_REG_OTP_INFO1_TEMP_CAL_ATE_O / 4,
                                   3, &ui32Temp[0]);

    if ( (ui32Temp[0] == 0x00000000)    ||
         (ui32Temp[1] == 0x00000000)    ||
         (ui32Temp[2] == 0x00000000)    ||
#ifdef AM_PART_APOLLO5A
         (ui32Temp[0] == 0xFFFFFFFF)    ||
         (ui32Temp[1] == 0xFFFFFFFF)    ||
         (ui32Temp[2] == 0xFFFFFFFF)    ||
#endif // AM_PART_APOLLO5A
        (ui32Retval != AM_HAL_STATUS_SUCCESS) )
    {
        //
        // Invalidate the application of TempCo.
        //
        g_bTempcoValid = false;
        return AM_HAL_STATUS_HW_ERR;
    }

    //
    // Make sure this device can reliably support TempCo
    //
    TrimVersionGet(&ui32TrimVer);
// #### INTERNAL BEGIN ####
#if 0 // Set to 1 to temporarily debug on a non-TempCo ready part
    ui32TrimVer = TRIMREV_PWRCTRL;
#endif
// #### INTERNAL END ####
    if ( ui32TrimVer < TRIMREV_PWRCTRL )
    {
        //
        // Invalidate that original trims have been saved, which
        // invalidates the application of TempCo.
        //
        //
        // Invalidate the application of TempCo.
        //
        g_bTempcoValid = false;
        return AM_HAL_STATUS_FAIL;
    }
    else
    {
        g_bTempcoValid = true;
    }

    //
    // Save the ADC handle and the slot number.
    //
    g_TempcoADCHandle   = pADCHandle;
    g_ui32TempcoADCslot = ui32ADCslot;

    //
    // At this point the ADC is expected to be initialized, powered, and configured.
    // Configure the temperature slot.
    //
    sSlotCfg.eMeasToAvg     = AM_HAL_ADC_SLOT_AVG_1;
    sSlotCfg.ui32TrkCyc     = 32;
    sSlotCfg.ePrecisionMode = AM_HAL_ADC_SLOT_12BIT;
    sSlotCfg.eChannel       = AM_HAL_ADC_SLOT_CHSEL_TEMP;
    sSlotCfg.bWindowCompare = false;
    sSlotCfg.bEnabled       = true;
    ui32Retval = am_hal_adc_configure_slot(g_TempcoADCHandle, g_ui32TempcoADCslot, &sSlotCfg);
    if ( ui32Retval != AM_HAL_STATUS_SUCCESS )
    {
       return ui32Retval;
    }

    return AM_HAL_STATUS_SUCCESS;

} // am_hal_pwrctrl_tempco_init()

//
// TempCo trims lookup tables
// The VDD table is arranged row-by-row with 3 values as:
//  0: Min temperature
//  1: Max temperature
//  2: Trim code adjust
//

//
//! Trim adjust table for VDDF
//
const static int8_t
g_VDDF_trimstbl[][3] =
{
    { -20, -11,   0},
    { -11,  -2,  -1},
    {  -2,   8,  -2},
    {   8,  17,  -3},
    {  17,  26,  -4},
    {  26,  35,  -5},
    {  35,  44,  -6},
    {  44,  53,  -7},
    {  53,  60,  -8},   // Last actual table entry
    { 127, 127,   0}    // End of table: Probably bogus temp, do no adjustment
};

//
//! Trim adjust table for VDDFLP
//
const static int8_t
g_VDDFLP_trimstbl[][3] =
{
    { -20, -11,   0},
    { -11,  -2,  -1},
    {  -2,   8,  -2},
    {   8,  17,  -3},
    {  17,  26,  -4},
    {  26,  35,  -5},
    {  35,  44,  -6},
    {  44,  53,  -7},
    {  53,  60,  -8},   // Last actual table entry
    { 127, 127,   0}    // End of table: Probably bogus temp, do no adjustment
};

//
//! Trim adjust table for memlpldo
//
const static int8_t
g_memlpldo_trimstbl[][3] =
{
    { -18, -14,   8},
    { -14, -10,   7},
    { -10,  -6,   6},
    {  -6,  -2,   5},
    {  -2,   2,   4},
    {   2,   6,   3},
    {   6,  10,   2},
    {  10,  14,   1},
    {  14,  18,   0},
    {  18,  22,  -1},
    {  22,  26,  -2},
    {  26,  42,  -3},
    {  42,  60,  -4},   // Last actual table entry
    { 127, 127,   0}    // End of table: Probably bogus temp, do no adjustment
};

//
//! Helper macro to round a float down.
//
#define FTOI_RNDDN(fval)        ( (fval < 0.00F) ? (int)fval - 1 : (int)fval )

// ****************************************************************************
// Function to lookup a trim given a temperature and a pointer
// to the appropriate lookup table.
// ****************************************************************************
static int8_t
lookup_trim(int8_t i8Temp, const int8_t pi8Tbl[][3])
{
    uint32_t ux;
    if ( i8Temp < pi8Tbl[0][0] )
    {
        //
        // Return the trim for the lowest temperature in the table
        //
        return pi8Tbl[0][2];
    }
    else
    {
        //
        // Lookup the trim
        //
        ux = 0;
        while ( pi8Tbl[ux][0] < 127 )
        {
            if ( i8Temp <= pi8Tbl[ux][1] )
            {
                return pi8Tbl[ux][2];
            }
            ux++;
        }

// #### INTERNAL BEGIN ####
        //return pi8Tbl[ux][2];
// #### INTERNAL END ####
        //
        // The temperature is very high, so snap to the default trims
        ///
        return 0;
    }

} // lookup_trim()

// ****************************************************************************
// Function to validate and apply trim changes.
// ****************************************************************************
static void
tempco_set_trims(int32_t i32VDDFtrim,
                 int32_t i32VDDFLPtrim,
                 int32_t i32MemlpLDOtrim)
{
    int32_t i32SimoVDDFact, i32Memldoact, i32SimoVDDFlp, i32Memlpldo;

    if ( !g_bOrigTrimsStored )
    {
        return;
    }

    i32SimoVDDFact = g_orig_ACTTRIMVDDF      + i32VDDFtrim;
    i32Memldoact   = g_orig_MEMLDOACTIVETRIM + i32VDDFtrim;
    i32SimoVDDFlp  = g_orig_LPTRIMVDDF       + i32VDDFLPtrim;
    i32Memlpldo    = g_orig_MEMLPLDOTRIM     + i32MemlpLDOtrim;

    //
    // If Crypto is currently off, adjust the VDDF active trim.
    //
    AM_CRITICAL_BEGIN

    if ( i32SimoVDDFact < 0 )
    {
        i32SimoVDDFact = 0;
    }
    else if ( i32SimoVDDFact > MAX_ACTTRIMVDDF )
    {
        i32SimoVDDFact = MAX_ACTTRIMVDDF;
    }

    if ( i32Memldoact < 0 )
    {
        i32Memldoact = 0;
    }
    else if ( i32Memldoact > MAX_MEMLDOACTIVETRIM )
    {
        i32Memldoact = MAX_MEMLDOACTIVETRIM;
    }

    if ( i32SimoVDDFlp < 0 )
    {
        i32SimoVDDFlp = 0;
    }
    else if ( i32SimoVDDFlp > MAX_LPTRIMVDDF )
    {
        i32SimoVDDFlp = MAX_LPTRIMVDDF;
    }

    if ( i32Memlpldo  < 0 )
    {
        i32Memlpldo  = 0;
    }
    else if ( i32Memlpldo > MAX_MEMLPLDOTRIM )
    {
        i32Memlpldo = MAX_MEMLPLDOTRIM;
    }

    //
    // Now set the new values
    //
    MCUCTRL->LDOREG2_b.MEMLDOACTIVETRIM = i32Memldoact;
    MCUCTRL->LDOREG2_b.MEMLPLDOTRIM     = i32Memlpldo;

    AM_CRITICAL_END
} // tempco_set_trims()

// ****************************************************************************
// Validate samples from temperature sensor, and average them.
// ****************************************************************************
static uint32_t
adc_get_temperature_volts(float *pfADCTempVolts,
                          uint32_t ui32NumSamples,
                          am_hal_adc_sample_t sSample[])
{
    uint32_t ux;
    float fSum;

    //
    // ui32NumSamples temperature samples have been obtained.
    // Make sure at least 2 of the samples are slightly different.
    //
    ux = 0;
    while ( ux < (ui32NumSamples - 1) )
    {
        if ( sSample[ux].ui32Sample != sSample[ux + 1].ui32Sample )
        {
            break;
        }
        ux++;
    }

    if ( ux == (ui32NumSamples - 1) )
    {
        //
        // This sample is not reliable, return with error.
        //
        return AM_HAL_STATUS_FAIL;
    }

    //
    // The measured temperature can be considered reliable.
    // Get an average of the temperature.
    //
    ux = 0;
    fSum = 0.0F;
    while ( ux < ui32NumSamples )
    {
        //
        // Convert and scale the temperature sample into its corresponding voltage.
        //
        g_ui16TempcoTEMP_code = AM_HAL_ADC_FIFO_SAMPLE(sSample[ux].ui32Sample);
        fSum += (float)g_ui16TempcoTEMP_code * AM_HAL_ADC_VREF / 4096.0f; // 12-bit sample
        ux++;
    }

    *pfADCTempVolts = fSum / ((float)ui32NumSamples);

    return AM_HAL_STATUS_SUCCESS;

} // adc_get_temperature_volts()

// ****************************************************************************
//
//  am_hal_pwrctrl_tempco_sample_handler()
//
//  This function to be called with a minimum periodicity as recommended
//  by Ambiq engineering.
//
// ****************************************************************************
uint32_t
am_hal_pwrctrl_tempco_sample_handler(uint32_t ui32NumSamples, am_hal_adc_sample_t sSamples[])
{
    uint32_t ui32Retval;
    float fVT[3];
    float fADCTempVolts, fADCTempDegreesC;
    int32_t i32VDDFtrim, i32VDDFLPtrim, i32MemlpLDOtrim;

    if ( (g_bTempcoValid == false) || (ui32NumSamples < AM_HAL_TEMPCO_NUMSAMPLES) )
    {
        return AM_HAL_STATUS_FAIL;
    }

    //
    // Query the ADC for the current temperature of the chip.
    //
    ui32Retval = adc_get_temperature_volts(&fADCTempVolts, ui32NumSamples, sSamples);
    if ( ui32Retval != AM_HAL_STATUS_SUCCESS )
    {
        //
        // This sample is not reliable.
        // Snap to the highest trim code settings from the tables and return.
// #### INTERNAL BEGIN ####
        // Note - these settings per Jamie email on 5/15/22.
// #### INTERNAL END ####
        //
        tempco_set_trims(g_VDDF_trimstbl[0][2],
                         g_VDDFLP_trimstbl[0][2],
                         g_memlpldo_trimstbl[0][2]);
// #### INTERNAL BEGIN ####
#if TEMPCO_DBG // Debug only
        am_util_stdio_printf("ERROR in adc_get_temperature_volts().\n");
#endif
// #### INTERNAL END ####
        return ui32Retval;
    }

    //
    // Now call the HAL routine to convert volts to degrees Celsius.
    //
    fVT[0] = fADCTempVolts;
    fVT[1] = 0.0f;
    fVT[2] = -123.456;
    ui32Retval = am_hal_adc_control(g_TempcoADCHandle, AM_HAL_ADC_REQ_TEMP_CELSIUS_GET, fVT);
    if ( ui32Retval == AM_HAL_STATUS_SUCCESS )
    {
        fADCTempDegreesC = fVT[1];  // Get the temperature
        g_pfTempMeasured = fADCTempDegreesC;
    }
    else
    {
        //
        // Error, restore default values.
        //
        tempco_set_trims(0, 0, 0);
        g_pfTempMeasured = 0.0F;
        return ui32Retval;
    }

    //
    // The temperature in degC is stored in fADCTempDegreesC
    //
    int8_t  i8Temp;
    int32_t i32Temp;
    i32Temp = FTOI_RNDDN(fADCTempDegreesC);

    //
    // Get the integer value of the temperature.
    // Allow for the temperature sensor specified at +-3C.
    //
    i8Temp = (int8_t)(i32Temp - 3);

    //
    // Look up the 3 trim adjustments.
    //
    i32VDDFtrim     = (int32_t)lookup_trim(i8Temp, g_VDDF_trimstbl);
    i32VDDFLPtrim   = (int32_t)lookup_trim(i8Temp, g_VDDFLP_trimstbl);
    i32MemlpLDOtrim = (int32_t)lookup_trim(i8Temp, g_memlpldo_trimstbl);

    //
    // Now, set the trims appropriately.
    //
// #### INTERNAL BEGIN ####
#if TEMPCO_DBG // Debug only
    am_util_stdio_printf("TEMP=%.2F C, AdjTemp=%4d, VDDFtrim=%4d, VDDFLPtrim=%4d, LDOtrim=%4d\n", fADCTempDegreesC, i8Temp, i32VDDFtrim, i32VDDFLPtrim, i32MemlpLDOtrim);
#endif
// #### INTERNAL END ####
    tempco_set_trims(i32VDDFtrim, i32VDDFLPtrim, i32MemlpLDOtrim);

    return AM_HAL_STATUS_SUCCESS;
} // am_hal_pwrctrl_tempco_sample_handler()

#endif // AM_HAL_TEMPCO_LP

// ****************************************************************************
//
//  am_hal_pwrctrl_pwrmodctl_cpdlp_config()
//  sCpdlpConfig - CPDLPSTATE config.
//
// ****************************************************************************
uint32_t
am_hal_pwrctrl_pwrmodctl_cpdlp_config(am_hal_pwrctrl_pwrmodctl_cpdlp_t sCpdlpConfig)
{
    uint32_t ui32CpdlpConfig = 0;

    //
    // If cache is in use, powering off cache is not allowed.
    //
    if (((SCB->CCR & SCB_CCR_IC_Msk) || (SCB->CCR & SCB_CCR_DC_Msk))
        && (sCpdlpConfig.eRlpConfig == AM_HAL_PWRCTRL_RLP_OFF))
    {
        return AM_HAL_STATUS_FAIL;
    }

    ui32CpdlpConfig |= (sCpdlpConfig.eRlpConfig << PWRMODCTL_CPDLPSTATE_RLPSTATE_Pos);
    ui32CpdlpConfig |= (sCpdlpConfig.eElpConfig << PWRMODCTL_CPDLPSTATE_ELPSTATE_Pos);
    ui32CpdlpConfig |= (sCpdlpConfig.eClpConfig << PWRMODCTL_CPDLPSTATE_CLPSTATE_Pos);

    PWRMODCTL->CPDLPSTATE = ui32CpdlpConfig;

    return AM_HAL_STATUS_SUCCESS;
} // am_hal_pwrctrl_pwrmodctl_cpdlp_config()

// ****************************************************************************
//
//  am_hal_pwrctrl_pwrmodctl_cpdlp_get()
//  psCpdlpConfig - CPDLPSTATE config.
//
// ****************************************************************************
void
am_hal_pwrctrl_pwrmodctl_cpdlp_get(am_hal_pwrctrl_pwrmodctl_cpdlp_t * psCpdlpConfig)
{
    uint32_t ui32CpdlpConfig = 0;

    ui32CpdlpConfig = PWRMODCTL->CPDLPSTATE;

    psCpdlpConfig->eRlpConfig = (am_hal_pwrctrl_pwrmodctl_rlp_e) ((ui32CpdlpConfig & PWRMODCTL_CPDLPSTATE_RLPSTATE_Msk) >> PWRMODCTL_CPDLPSTATE_RLPSTATE_Pos);
    psCpdlpConfig->eElpConfig = (am_hal_pwrctrl_pwrmodctl_elp_e) ((ui32CpdlpConfig & PWRMODCTL_CPDLPSTATE_ELPSTATE_Msk) >> PWRMODCTL_CPDLPSTATE_ELPSTATE_Pos);
    psCpdlpConfig->eClpConfig = (am_hal_pwrctrl_pwrmodctl_clp_e) ((ui32CpdlpConfig & PWRMODCTL_CPDLPSTATE_CLPSTATE_Msk) >> PWRMODCTL_CPDLPSTATE_CLPSTATE_Pos);
} // am_hal_pwrctrl_pwrmodctl_cpdlp_get()

// ****************************************************************************
//
//  am_hal_pwrctrl_wait_pll_lock_for_hp2()
//
// ****************************************************************************
void
am_hal_pwrctrl_wait_pll_lock_for_hp2(bool bWaitPllockForHp2)
{
    //
    // wait for hp2 or not.
    //
    PWRCTRL->MCUPERFREQ_b.WAITFORHP2 = bWaitPllockForHp2;
} // am_hal_pwrctrl_wait_pll_lock_for_hp2()

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
