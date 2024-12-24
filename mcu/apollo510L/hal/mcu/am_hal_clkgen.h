//*****************************************************************************
//
//! @file am_hal_clkgen.h
//!
//! @brief Functions for interfacing with the CLKGEN.
//!
//! @addtogroup clkgen4 CLKGEN - Clock Generator
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
#ifndef AM_HAL_CLKGEN_H
#define AM_HAL_CLKGEN_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! @name System Clock max frequency
//! @{
//! Defines the maximum clock frequency for this device.
//! These macros provide a definition of the maximum clock frequency.
//
//*****************************************************************************
#ifdef APOLLO5_FPGA
#define AM_HAL_CLKGEN_FREQ_MAX_HZ           (APOLLO5_FPGA * 1000000)
#define AM_HAL_CLKGEN_FREQ_HP192_HZ         (16 * 1000000)
#define AM_HAL_CLKGEN_FREQ_HP250_HZ         (20 * 1000000)
#else // APOLLO5_FPGA
#define AM_HAL_CLKGEN_FREQ_MAX_HZ           96000000
// #### INTERNAL BEGIN ####
// CAYNSWS-1744 Apollo5b remove CPUHPFREQSEL, the only valid HP frequency is 250MHz.
// #### INTERNAL END ####
#define AM_HAL_CLKGEN_FREQ_HP192_HZ         192000000
#define AM_HAL_CLKGEN_FREQ_HP250_HZ         250000000
#endif // APOLLO5_FPGA

#define AM_HAL_CLKGEN_FREQ_MAX_KHZ      (AM_HAL_CLKGEN_FREQ_MAX_HZ / 1000)
#define AM_HAL_CLKGEN_FREQ_HP250_KHZ    (AM_HAL_CLKGEN_FREQ_HP250_HZ / 1000)
#define AM_HAL_CLKGEN_FREQ_MAX_MHZ      (AM_HAL_CLKGEN_FREQ_MAX_HZ / 1000000)
#define AM_HAL_CLKGEN_FREQ_HP250_MHZ    (AM_HAL_CLKGEN_FREQ_HP250_HZ / 1000000)
#define AM_HAL_CLKGEN_CORESEL_MAXDIV    1
//! @}

//
//! Control operations.
//
typedef enum
{
    AM_HAL_CLKGEN_CONTROL_RTC_SEL_XTAL,
    AM_HAL_CLKGEN_CONTROL_RTC_SEL_LFRC,
    AM_HAL_CLKGEN_CONTROL_HFADJ_ENABLE,
    AM_HAL_CLKGEN_CONTROL_HFADJ_DISABLE,
    AM_HAL_CLKGEN_CONTROL_I3CCLKEN_ENABLE,
    AM_HAL_CLKGEN_CONTROL_I3CCLKEN_DISABLE,
    AM_HAL_CLKGEN_CONTROL_PLLVCOEN_ENABLE,
    AM_HAL_CLKGEN_CONTROL_PLLVCOEN_NORMALOP,
    AM_HAL_CLKGEN_CONTROL_PLLDIVEN_ENABLE,
    AM_HAL_CLKGEN_CONTROL_PLLDIVEN_NORMALOP,
    AM_HAL_CLKGEN_CONTROL_SELECT_EXTREF,
    AM_HAL_CLKGEN_CONTROL_SELECT_XT,
    AM_HAL_CLKGEN_CONTROL_DISPCTRLCLK_ENABLE,
    AM_HAL_CLKGEN_CONTROL_DISPCTRLCLK_DISABLE,
    AM_HAL_CLKGEN_CONTROL_GFXCORECLK_ENABLE,
    AM_HAL_CLKGEN_CONTROL_GFXCORECLK_DISABLE
} am_hal_clkgen_control_e;

//
//! Current RTC oscillator.
//
typedef enum
{
    AM_HAL_CLKGEN_STATUS_RTCOSC_XTAL,
    AM_HAL_CLKGEN_STATUS_RTCOSC_LFRC,
} am_hal_clkgen_status_rtcosc_e;

//
//! Clock Generation CLKOUT
//
typedef enum
{
    // XTAL
    AM_HAL_CLKGEN_CLKOUT_XTAL_32768     = CLKGEN_CLKOUT_CKSEL_XT_DIV2,      // XTAL         = 32KHz
    AM_HAL_CLKGEN_CLKOUT_XTAL_16384     = CLKGEN_CLKOUT_CKSEL_XT_DIV4,      // XTAL / 4     = 16KHz
    AM_HAL_CLKGEN_CLKOUT_XTAL_8192      = CLKGEN_CLKOUT_CKSEL_XT_DIV8,      // XTAL / 8     = 8KHz
    AM_HAL_CLKGEN_CLKOUT_XTAL_4096      = CLKGEN_CLKOUT_CKSEL_XT_DIV16,     // XTAL / 16    = 4KHz
    AM_HAL_CLKGEN_CLKOUT_XTAL_2048      = CLKGEN_CLKOUT_CKSEL_XT_DIV32,     // XTAL / 32    = 2KHz
    AM_HAL_CLKGEN_CLKOUT_XTAL_1024      = CLKGEN_CLKOUT_CKSEL_XT_DIV256,    // XTAL / 256   = 1KHz
    AM_HAL_CLKGEN_CLKOUT_XTAL_128       = CLKGEN_CLKOUT_CKSEL_XT_DIV8K,     // XTAL / 256   = 128 Hz
    AM_HAL_CLKGEN_CLKOUT_XTAL_4         = CLKGEN_CLKOUT_CKSEL_XT_DIV64K,    // XTAL / 8192  =  4 Hz
    AM_HAL_CLKGEN_CLKOUT_XTAL_0_5       = CLKGEN_CLKOUT_CKSEL_XT_DIV2M,     // XTAL / 65536 =  0.5 Hz

    // RTC
    AM_HAL_CLKGEN_CLKOUT_RTC_1Hz        = CLKGEN_CLKOUT_CKSEL_RTC_1Hz,      // RTC          = 1Hz
    AM_HAL_CLKGEN_CLKOUT_RTC_100Hz      = CLKGEN_CLKOUT_CKSEL_RTC_CG_100Hz, // RTC          = 100Hz

    // LFRC
    AM_HAL_CLKGEN_CLKOUT_LFRC           = CLKGEN_CLKOUT_CKSEL_LFRC,         // LFRC         = 1KHz
    AM_HAL_CLKGEN_CLKOUT_LFRC_512       = CLKGEN_CLKOUT_CKSEL_LFRC_DIV2,    // LFRC / 2     = MCU clock
    AM_HAL_CLKGEN_CLKOUT_LFRC_32        = CLKGEN_CLKOUT_CKSEL_LFRC_DIV32,   // LFRC / 32    = PLLVCO 250MHz
    AM_HAL_CLKGEN_CLKOUT_LFRC_2         = CLKGEN_CLKOUT_CKSEL_LFRC_DIV512,  // LFRC / 512   = PLLVCO 125MHz

    // PLL
    AM_HAL_CLKGEN_CLKOUT_PLL_POSTDIV    = CLKGEN_CLKOUT_CKSEL_PLLPOSTDIV,   // PLL            PLL POSTDIV
    AM_HAL_CLKGEN_CLKOUT_PLL_OUT2       = CLKGEN_CLKOUT_CKSEL_PLLFOUT2,     // PLL            PLLFOUT2
    AM_HAL_CLKGEN_CLKOUT_PLL_OUT3       = CLKGEN_CLKOUT_CKSEL_PLLFOUT3,     // PLL            PLLFOUT3
    AM_HAL_CLKGEN_CLKOUT_PLL_OUT4       = CLKGEN_CLKOUT_CKSEL_PLLFOUT4,     // PLL            PLLFOUT4

    // RF XTAL
    AM_HAL_CLKGEN_CLKOUT_RF_XTAL_48M    = CLKGEN_CLKOUT_CKSEL_RFXTAL48,     // RF XTAL       = 48MHz
    AM_HAL_CLKGEN_CLKOUT_RF_XTAL_24M    = CLKGEN_CLKOUT_CKSEL_RFXTAL24,     // RF XTAL       = 24MHz
    AM_HAL_CLKGEN_CLKOUT_RF_XTAL_12M    = CLKGEN_CLKOUT_CKSEL_RFXTAL12,     // RF XTAL       = 12MHz

    // Secure RTC clocks
    AM_HAL_CLKGEN_CLKOUT_SECURE_RTC_1HZ   = CLKGEN_CLKOUT_CKSEL_SECURE_RTC_1Hz,      // Secure RTC 1Hz
    AM_HAL_CLKGEN_CLKOUT_SECURE_RTC_100HZ = CLKGEN_CLKOUT_CKSEL_SECURE_RTC_CG_100Hz, // Secure RTC 100Hz

    //AO CLK
    AM_HAL_CLKGEN_CLKOUT_AO_CLK_DIV16     = CLKGEN_CLKOUT_CKSEL_AO_DIV_CLK,  //AO_CLK div16
} am_hal_clkgen_clkout_e;

#define AM_HAL_CLKGEN_CLKOUT_MAX        CLKGEN_CLKOUT_CKSEL_SECURE_RTC_CG_100Hz         // Highest valid CKSEL enum value

// #### INTERNAL BEGIN ####
//
// XTAL 24MHz control
//
/*typedef enum
{
    AM_HAL_CLKGEN_XTAL24MCTRL_XTAL24MDS_0 = 0x0,      // Drive strength
    AM_HAL_CLKGEN_XTAL24MCTRL_XTAL24MDS_1,
    AM_HAL_CLKGEN_XTAL24MCTRL_XTAL24MDS_2,
    AM_HAL_CLKGEN_XTAL24MCTRL_XTAL24MDS_3,
    AM_HAL_CLKGEN_XTAL24MCTRL_XTAL24MDS_4,
    AM_HAL_CLKGEN_XTAL24MCTRL_XTAL24MDS_5,
    AM_HAL_CLKGEN_XTAL24MCTRL_XTAL24MDS_6,
    AM_HAL_CLKGEN_XTAL24MCTRL_XTAL24MDS_7,
} am_hal_clkgen_xtal24mctrl_e;*/

//
// Display clock control
//
/*typedef enum
{
    AM_HAL_CLKGEN_DISPCLKCTRL_DCCLKEN,
    AM_HAL_CLKGEN_DISPCLKCTRL_DISPCLKSEL_OFF,
    AM_HAL_CLKGEN_DISPCLKCTRL_DISPCLKSEL_HFRC48,
    AM_HAL_CLKGEN_DISPCLKCTRL_DISPCLKSEL_HFRC96,
    AM_HAL_CLKGEN_DISPCLKCTRL_DISPCLKSEL_DPHYPLL,
    AM_HAL_CLKGEN_DISPCLKCTRL_PLLCLKEN,
    AM_HAL_CLKGEN_DISPCLKCTRL_PLLCLKSEL_OFF,
    AM_HAL_CLKGEN_DISPCLKCTRL_PLLCLKSEL_HFRC12,
    AM_HAL_CLKGEN_DISPCLKCTRL_PLLCLKSEL_HFRC24,
    AM_HAL_CLKGEN_DISPCLKCTRL_PLLCLKSEL_HFXT,
} am_hal_clkgen_dispclkctrl_e*/;

//
// ClkGen Interrupts
//
/*typedef enum
{
    AM_HAL_CLKGEN_INTERRUPT_OF          = CLKGEN_INTRPTEN_OF_Msk,
    AM_HAL_CLKGEN_INTERRUPT_ACC         = CLKGEN_INTRPTEN_ACC_Msk,
    AM_HAL_CLKGEN_INTERRUPT_ACF         = CLKGEN_INTRPTEN_ACF_Msk
} am_hal_clkgen_interrupt_e;*/
// #### INTERNAL END ####

//
//! Status structure.
//
typedef struct
{
    //
    // ui32SysclkFreq
    //!  Returns the current system clock frequency, in hertz.
    //
    uint32_t    ui32SysclkFreq;

    //
    // eRTCOSC
    //
    //!  Returns the current RTC oscillator as one of:
    //!  AM_HAL_CLKGEN_STATUS_RTCOSC_LFRC
    //!  AM_HAL_CLKGEN_STATUS_RTCOSC_XTAL
    //
    uint32_t    eRTCOSC;

    //
    // bXtalFailure
    //!  true = XTAL has failed (is enabled but not oscillating).  Also if the
    //!         LFRC is selected as the oscillator in OCTRL.RTCOSEL.
    //
    bool        bXtalFailure;

    //
    // enable status for all the peripheral clocks.
    //  1: enable
    //  0: disable
    //
    //uint32_t    ui32Clockenstat;
    //uint32_t    ui32Clocken2stat;
    //uint32_t    ui32Clocken3stat;
} am_hal_clkgen_status_t;

// #### INTERNAL BEGIN ####
#if 0   // FAL-778 RevB0 removes OCTRL.STOPRC, OCTRL.STOPXT, and STATUS (RTCOSC, OSCF)
//
// Clkgen status and returns.
//
typedef enum
{
    AM_HAL_CLKGEN_STATUS_SYSCLK,
    AM_HAL_CLKGEN_STATUS_OSC_RTC,
    AM_HAL_CLKGEN_STATUS_XTAL_FAILURE
} am_hal_clkgen_status_e;
#endif
// #### INTERNAL END ####
// ****************************************************************************
//
//! @brief Apply various specific commands/controls on the CLKGEN module.
//!
//! This function is used to apply various controls on CLKGEN.
//!
//! @note IMPORTANT! This function MUST be called very early in execution of
//!       an application with the parameter AM_HAL_CLKGEN_CONTROL_SYSCLK_MAX
//!       in order to set Apollo5 to its required operating frequency.
//!
//! @param eControl - One of the following:
//!     AM_HAL_CLKGEN_CONTROL_SYSCLK_MAX
//!     AM_HAL_CLKGEN_CONTROL_RTC_SEL_XTAL
//!     AM_HAL_CLKGEN_CONTROL_RTC_SEL_LFRC
//!     AM_HAL_CLKGEN_CONTROL_HFADJ_ENABLE
//!     AM_HAL_CLKGEN_CONTROL_HFADJ_DISABLE
//! @param pArgs - Pointer to arguments for Control Switch Case
//!
//! @return status      - generic or interface specific status.
//!
//! @note After starting the XTAL, a 2 second warm-up delay is required.
//
// ****************************************************************************
extern uint32_t am_hal_clkgen_control(am_hal_clkgen_control_e eControl,
                                      void *pArgs);

// ****************************************************************************
//
//! @brief Get CLKGEN status.
//!
//! This function returns the current value of various CLKGEN statuses.
//!
//! @param psStatus - ptr to a status structure to receive the current statuses.
//!
//! @return status      - generic or interface specific status.
//!
//! @note After selection of the RTC Oscillator, a 2 second delay is required
//! before the new oscillator takes effect.  Therefore the CLKGEN.STATUS.OMODE
//! bit will not reflect the new status until after the 2s wait period.
//
// ****************************************************************************
extern uint32_t am_hal_clkgen_status_get(am_hal_clkgen_status_t *psStatus);

// ****************************************************************************
//
//! @brief Enable CLKOUT.
//!
//! This function is used to enable and select a CLKOUT frequency.
//!
//! @param bEnable: true to enable, false to disable.
//! @param eClkSelect - One of the following:
//!     AM_HAL_CLKGEN_CLKOUT_XTAL_32768
//!     AM_HAL_CLKGEN_CLKOUT_XTAL_16384
//!     AM_HAL_CLKGEN_CLKOUT_XTAL_8192
//!     AM_HAL_CLKGEN_CLKOUT_XTAL_4096
//!     AM_HAL_CLKGEN_CLKOUT_XTAL_2048
//!     AM_HAL_CLKGEN_CLKOUT_XTAL_1024
//!     AM_HAL_CLKGEN_CLKOUT_XTAL_128
//!     AM_HAL_CLKGEN_CLKOUT_XTAL_4
//!     AM_HAL_CLKGEN_CLKOUT_XTAL_0_5
//!
//!     AM_HAL_CLKGEN_CLKOUT_RTC_1Hz
//!     AM_HAL_CLKGEN_CLKOUT_RTC_100Hz
//!
//!     AM_HAL_CLKGEN_CLKOUT_LFRC
//!     AM_HAL_CLKGEN_CLKOUT_LFRC_512
//!     AM_HAL_CLKGEN_CLKOUT_LFRC_32
//!     AM_HAL_CLKGEN_CLKOUT_LFRC_2
//!
//!     AM_HAL_CLKGEN_CLKOUT_PLL_POSTDIV
//!     AM_HAL_CLKGEN_CLKOUT_PLL_OUT2
//!     AM_HAL_CLKGEN_CLKOUT_PLL_OUT3
//!     AM_HAL_CLKGEN_CLKOUT_PLL_OUT4
//!
//!     AM_HAL_CLKGEN_CLKOUT_RF_XTAL_48M
//!     AM_HAL_CLKGEN_CLKOUT_RF_XTAL_24M
//!     AM_HAL_CLKGEN_CLKOUT_RF_XTAL_12M
//!
//!     AM_HAL_CLKGEN_CLKOUT_SECURE_RTC_1HZ
//!     AM_HAL_CLKGEN_CLKOUT_SECURE_RTC_100HZ
//!
//! @return status      - generic or interface specific status.
//
// ****************************************************************************
extern uint32_t am_hal_clkgen_clkout_enable(bool bEnable,
                                            am_hal_clkgen_clkout_e eClkSelect);
// #### INTERNAL BEGIN ####
// ****************************************************************************
//
//! @brief Enable XTAL 24MHz.
//!
//! This function is used to enable XTAL 24MHz and select drive strength.
//!
//! @param bEnable: true to enable, false to disable.
//! @param eDSSelect - One of the following:
//!     AM_HAL_CLKGEN_XTAL24MCTRL_XTAL24MDS_0
//!     AM_HAL_CLKGEN_XTAL24MCTRL_XTAL24MDS_1
//!     AM_HAL_CLKGEN_XTAL24MCTRL_XTAL24MDS_2
//!     AM_HAL_CLKGEN_XTAL24MCTRL_XTAL24MDS_3
//!     AM_HAL_CLKGEN_XTAL24MCTRL_XTAL24MDS_4
//!     AM_HAL_CLKGEN_XTAL24MCTRL_XTAL24MDS_5
//!     AM_HAL_CLKGEN_XTAL24MCTRL_XTAL24MDS_6
//!     AM_HAL_CLKGEN_XTAL24MCTRL_XTAL24MDS_7
//!
//! @return status      - generic or interface specific status.
//
// ****************************************************************************
//extern uint32_t am_hal_clkgen_xtal24mctrl_enable(bool bEnable,
                                               //am_hal_clkgen_xtal24mctrl_e eDSSelect);

// ****************************************************************************
//
//! @brief Enable DCCLK.
//!
//! This function is used to enable DCCLK and select clock source.
//!
//! @param bEnable: true to enable, false to disable.
//! @param eDSSelect - One of the following:
//!     CLKGEN_DISPCLKCTRL_DISPCLKSEL_OFF
//!     CLKGEN_DISPCLKCTRL_DISPCLKSEL_HFRC48
//!     CLKGEN_DISPCLKCTRL_DISPCLKSEL_HFRC96
//!     CLKGEN_DISPCLKCTRL_DISPCLKSEL_DPHYPLL
//!
//! @return status      - generic or interface specific status.
//
// ****************************************************************************
//extern uint32_t am_hal_clkgen_dcclk_enable(bool bEnable,
                                           //CLKGEN_DISPCLKCTRL_DISPCLKSEL_Enum eDispclkSelect);

// ****************************************************************************
//
//! @brief Enable PLLCLK.
//!
//! This function is used to enable PLLCLK and select pll reference clock.
//!
//! @param bEnable: true to enable, false to disable.
//! @param eDSSelect - One of the following:
//!     CLKGEN_DISPCLKCTRL_PLLCLKSEL_OFF
//!     CLKGEN_DISPCLKCTRL_PLLCLKSEL_HFRC12
//!     CLKGEN_DISPCLKCTRL_PLLCLKSEL_HFRC24
//!     CLKGEN_DISPCLKCTRL_PLLCLKSEL_HFXT
//!
//! @return status      - generic or interface specific status.
//
// ****************************************************************************
//extern uint32_t am_hal_clkgen_pllclk_enable(bool bEnable,
                                          //CLKGEN_DISPCLKCTRL_PLLCLKSEL_Enum ePllclkSelect);

// ****************************************************************************
//
//! @brief Enable selected CLKGEN Interrupts.
//!
//! Use this function to enable the interrupts.
//!
//! @param ui32IntMask - One or more of the following bitmasks.
//!     AM_HAL_CLKGEN_INTERRUPT_OF
//!     AM_HAL_CLKGEN_INTERRUPT_ACC
//!     AM_HAL_CLKGEN_INTERRUPT_ACF
//!
//! @return status      - generic or interface specific status.
//
// ****************************************************************************
//extern uint32_t am_hal_clkgen_interrupt_enable(am_hal_clkgen_interrupt_e ui32IntMask);

// ****************************************************************************
//
//! @brief Disable selected CLKGEN Interrupts.
//!
//! Use this function to disable the CLKGEN interrupts.
//!
//! @param ui32IntMask - One or more of the following bitmasks.
//!     AM_HAL_CLKGEN_INTERRUPT_OF
//!     AM_HAL_CLKGEN_INTERRUPT_ACC
//!     AM_HAL_CLKGEN_INTERRUPT_ACF
//!
//! @return status      - generic or interface specific status.
//
// ****************************************************************************
//extern uint32_t am_hal_clkgen_interrupt_disable(am_hal_clkgen_interrupt_e ui32IntMask);

//*****************************************************************************
//
//! @brief IOM interrupt clear
//!
//! @param ui32IntMask  - interface specific interrupt mask.
//!
//! This function clears the interrupts for the given peripheral.
//!
//! The following are valid clear bits, any of which can be ORed together.
//!     AM_HAL_CLKGEN_INTERRUPT_OF
//!     AM_HAL_CLKGEN_INTERRUPT_ACC
//!     AM_HAL_CLKGEN_INTERRUPT_ACF
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
//extern uint32_t am_hal_clkgen_interrupt_clear(am_hal_clkgen_interrupt_e ui32IntMask);

// ****************************************************************************
//
//! @brief Return CLKGEN interrupts.
//!
//! Use this function to get all CLKGEN interrupts, or only the interrupts
//! that are enabled.
//!
//! @return status
//
// ****************************************************************************
//extern uint32_t am_hal_clkgen_interrupt_status_get(bool bEnabledOnly,
//                                                   uint32_t *pui32IntStatus);

// ****************************************************************************
//
//! @brief Sets the interrupt status.
//!
//! This function sets the CLKGEN interrupts.
//!
//! @param ui32IntMask - One or more of the following bitmasks.
//!     AM_HAL_CLKGEN_INTERRUPT_OF
//!     AM_HAL_CLKGEN_INTERRUPT_ACC
//!     AM_HAL_CLKGEN_INTERRUPT_ACF
//!
//! @return status
//
// ****************************************************************************
//extern uint32_t am_hal_clkgen_interrupt_set(am_hal_clkgen_interrupt_e ui32IntMask);
// #### INTERNAL END ####
#ifdef __cplusplus
}
#endif

#endif // AM_HAL_CLKGEN_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
