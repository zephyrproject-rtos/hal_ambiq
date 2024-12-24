//*****************************************************************************
//
//! @file am_hal_pwrctrl.h
//!
//! @brief Functions for enabling and disabling power domains.
//!
//! @addtogroup pwrctrl4 PWRCTRL - Power Control
//! @ingroup apollo510L_hal
//! @{

//*****************************************************************************

//*****************************************************************************
//
// ${copyright}
//
// This is part of revision ${version} of the AmbiqSuite Development Package.
//
//*****************************************************************************

#ifndef AM_HAL_PWRCTRL_H
#define AM_HAL_PWRCTRL_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! @name Delays (in uS) required for VDDF/VDDC trim enhancements.
//! @{
//
//*****************************************************************************
#define AM_HAL_PWRCTRL_VDDF_BOOST_DELAY     20
#define AM_HAL_PWRCTRL_MEMLDO_BOOST_DELAY   20
#define AM_HAL_PWRCTRL_VDDC_BOOST_DELAY     20
#define AM_HAL_PWRCTRL_GOTOLDO_DELAY        20
//! @}

//*****************************************************************************
//
//! Option for coreldo and memldo to operate in parallel with simobuck
//!  AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL
//!      0 = Do not turn on LDOs in parallel with simobuck.
//!      1 = Turn on LDOs in parallel with simobuck and set their voltage levels
//!          ~35mV lower than minimum buck voltages.
//  Default: 1
//  NOTE: FOR Apollo5b - this must be 1
//
//*****************************************************************************
#define AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL    1  // Enabled for Apollo5b silicon

#if (AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL == 0)
#error Apollo5b requires AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL.
#endif

//*****************************************************************************
//
//! Option to assist VDDC by activating LDOs when disabling SIMOBUCK.
//!  AM_HAL_PWRCTRL_LDOS_FOR_VDDC
//!      0 = Do not assist VDDC.
//!      1 = Activate LDOs in parallel when disabling SIMOBUCK.
//!  Default: 1
// #### INTERNAL BEGIN ####
// A2SD-2248 AM_HAL_PWRCTRL_LDOS_FOR_VDDC. This option is dependent on
// AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL.
// #### INTERNAL END ####
//
//*****************************************************************************
#define AM_HAL_PWRCTRL_LDOS_FOR_VDDC                    0
#if ( (AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL == 0) && (AM_HAL_PWRCTRL_LDOS_FOR_VDDC != 0) )
#warning AM_HAL_PWRCTRL_LDOS_FOR_VDDC requires AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL.
#endif

//*****************************************************************************
//
//! Option to connect MCU core to VDDC_LV for increased power efficiency. Ambiq
//! recommends this option be enabled for all new applications
//
// #### INTERNAL BEGIN ####
// See FB-350 for discussion and detail concerning this option.
// #### INTERNAL END ####
//
//! Default: 1 for RevC
//*****************************************************************************
#define AM_HAL_PWRCTRL_CORE_PWR_OPTIMAL_EFFICIENCY      0

//*****************************************************************************
//
//! Option for the TempCo power minimum power.
//
//*****************************************************************************
#define AM_HAL_TEMPCO_LP                                0
#if AM_HAL_TEMPCO_LP
#define AM_HAL_TEMPCO_DELAYUS   30

//
//! Define the recommended number of ADC samples required for accurate
//! temperature measurement. This number of samples must be passed along
//! to the HAL function, am_hal_pwrctrl_tempco_sample_handler().
//
#define AM_HAL_TEMPCO_NUMSAMPLES  5
#endif // AM_HAL_TEMPCO_LP

//*****************************************************************************
//
//! @name Performace mode enums.
//! @{
//
//*****************************************************************************
//
//! Peripheral MCU Mode power enum.
//
typedef enum
{
    AM_HAL_PWRCTRL_MCU_MODE_LOW_POWER        = PWRCTRL_MCUPERFREQ_MCUPERFSTATUS_LP, // 96 MHz
    AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE1 = PWRCTRL_MCUPERFREQ_MCUPERFSTATUS_HP1, // 192 MHz
    AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE2 = PWRCTRL_MCUPERFREQ_MCUPERFSTATUS_HP2, // 250 MHz
} am_hal_pwrctrl_mcu_mode_e;

//
//! vddf boost mode enum.
//
typedef enum
{
    AM_HAL_PWRCTRL_VDDF_BOOST_DEFAULT,
    AM_HAL_PWRCTRL_VDDF_BOOST_LEVEL1,
    AM_HAL_PWRCTRL_VDDF_BOOST_LEVEL2,
} am_hal_pwrctrl_vddf_boost_level_e;

//! @}

//*****************************************************************************
//
//! Peripheral power enum.
//
//*****************************************************************************
typedef enum
{
    AM_HAL_PWRCTRL_PERIPH_IOSFD0,
    AM_HAL_PWRCTRL_PERIPH_IOSFD1,
    AM_HAL_PWRCTRL_PERIPH_IOM0,
    AM_HAL_PWRCTRL_PERIPH_IOM1,
    AM_HAL_PWRCTRL_PERIPH_IOM2,
    AM_HAL_PWRCTRL_PERIPH_IOM3,
    AM_HAL_PWRCTRL_PERIPH_IOM4,
    AM_HAL_PWRCTRL_PERIPH_IOM5,
    AM_HAL_PWRCTRL_PERIPH_UART0,
    AM_HAL_PWRCTRL_PERIPH_UART1,
    AM_HAL_PWRCTRL_PERIPH_ADC,
    AM_HAL_PWRCTRL_PERIPH_MSPI0,
    AM_HAL_PWRCTRL_PERIPH_MSPI1,
    AM_HAL_PWRCTRL_PERIPH_MSPI2,
    AM_HAL_PWRCTRL_PERIPH_GFX,
    AM_HAL_PWRCTRL_PERIPH_DISP,
    AM_HAL_PWRCTRL_PERIPH_DISPPHY,
    AM_HAL_PWRCTRL_PERIPH_CRYPTO,
    AM_HAL_PWRCTRL_PERIPH_SDIO0,
    AM_HAL_PWRCTRL_PERIPH_SDIO1,
    AM_HAL_PWRCTRL_PERIPH_USB,
    AM_HAL_PWRCTRL_PERIPH_USBPHY,
    AM_HAL_PWRCTRL_PERIPH_DEBUG,
    AM_HAL_PWRCTRL_PERIPH_OTP,
    AM_HAL_PWRCTRL_PERIPH_PDM0,
    AM_HAL_PWRCTRL_PERIPH_I2S0,
    AM_HAL_PWRCTRL_PERIPH_I3C,
    AM_HAL_PWRCTRL_PERIPH_I3CPHY,
    AM_HAL_PWRCTRL_PERIPH_NETAOL,
    AM_HAL_PWRCTRL_PERIPH_MAX
} am_hal_pwrctrl_periph_e;

//*****************************************************************************
//
// MCU memory control settings.
//
//*****************************************************************************

//
//! DTCM enable settings.
//
typedef enum
{
    AM_HAL_PWRCTRL_DTCM_NONE       = PWRCTRL_MEMPWREN_PWRENTCM_NONE,
    AM_HAL_PWRCTRL_DTCM128K        = PWRCTRL_MEMPWREN_PWRENTCM_DTCM128K,
    AM_HAL_PWRCTRL_DTCM256K        = PWRCTRL_MEMPWREN_PWRENTCM_DTCM256K,
} am_hal_pwrctrl_dtcm_select_e;

//
//! DTCM retain settings.
//
typedef enum
{
  AM_HAL_PWRCTRL_MEMRETCFG_TCMPWDSLP_RETAIN = PWRCTRL_MEMRETCFG_TCMPWDSLP_RETAIN,
  AM_HAL_PWRCTRL_MEMRETCFG_TCMPWDSLP_NORETAIN = PWRCTRL_MEMRETCFG_TCMPWDSLP_NORETAIN,
} am_hal_pwrctrl_dtcm_retain_e;

//
//! NVM enable settings.
//
typedef enum
{
    AM_HAL_PWRCTRL_NVM_NONE       = 0,
    AM_HAL_PWRCTRL_NVM            = 1,
} am_hal_pwrctrl_nvm_select_e;

//
//! NVM power down modes settings.
//
typedef enum
{
    AM_HAL_PWRCTRL_NVM_RETENTION_POWER_DOWN       = 0,
    AM_HAL_PWRCTRL_NVM_DEEP_POWER_DOWN            = 1,
    AM_HAL_PWRCTRL_NVM_FULL_POWER_DOWN            = 2,
} am_hal_pwrctrl_nvm_power_down_select_e;

//
//!ROM settings.
//
typedef enum
{
    AM_HAL_PWRCTRL_ROM_ALWAYS_ON,               // Turn on ROM and always leave it on.
    AM_HAL_PWRCTRL_ROM_AUTO                     // Allow HAL to manage ROM power state.
} am_hal_pwrctrl_rom_select_e;

//
//! MCU memory configuration structure.
//
typedef struct
{
    //
    //! ROM mode selection.
    //
    am_hal_pwrctrl_rom_select_e         eROMMode;

    //
    //! DTCM configuration
    //
    am_hal_pwrctrl_dtcm_select_e        eDTCMCfg;
    am_hal_pwrctrl_dtcm_retain_e        eRetainDTCM;

    //
    //! NVM configuration.
    //
    am_hal_pwrctrl_nvm_select_e         eNVMCfg;

    //
    //! NVM power down modes configuration.
    //
    am_hal_pwrctrl_nvm_power_down_select_e eNVMPowerDown;

    bool                                bKeepNVMOnInDeepSleep;
} am_hal_pwrctrl_mcu_memory_config_t;

//
//! Miscellaneous power controls.
//
typedef enum
{
    AM_HAL_PWRCTRL_CONTROL_SIMOBUCK_INIT,       // Enable the SIMOBUCK
    AM_HAL_PWRCTRL_CONTROL_CRYPTO_POWERDOWN,    // Power down Crypto
    AM_HAL_PWRCTRL_CONTROL_XTAL_PWDN_DEEPSLEEP, // Allow the crystal to power down during deepsleep
    AM_HAL_PWRCTRL_CONTROL_DIS_PERIPHS_ALL,     // Power down all peripherals
#if AM_HAL_TEMPCO_LP
    AM_HAL_PWRCTRL_CONTROL_TEMPCO_GETMEASTEMP,  // TempCo, return the measured temperature
#endif // AM_HAL_TEMPCO_LP
    AM_HAL_PWRCTRL_CONTROL_DEEPERSLEEP_ENABLE,  // Enable mcu to go to Deeper-sleep
    AM_HAL_PWRCTRL_CONTROL_DEEPERSLEEP_DISABLE, // Select mcu to go to Deep-sleep
} am_hal_pwrctrl_control_e;

//*****************************************************************************
//
//! Shared memory control settings.
//
//*****************************************************************************
typedef enum
{
    AM_HAL_PWRCTRL_SRAM_NONE         = PWRCTRL_SSRAMPWREN_PWRENSSRAM_NONE,
    AM_HAL_PWRCTRL_SRAM_1M           = PWRCTRL_SSRAMPWREN_PWRENSSRAM_GROUP0,
    AM_HAL_PWRCTRL_SRAM_0P75M        = PWRCTRL_SSRAMPWREN_PWRENSSRAM_GROUP1,
    AM_HAL_PWRCTRL_SRAM_1P75M        = PWRCTRL_SSRAMPWREN_PWRENSSRAM_ALL,
} am_hal_pwrctrl_sram_select_e;

typedef struct
{
    //
    //! SRAM banks to enable.
    //
    am_hal_pwrctrl_sram_select_e        eSRAMCfg;

    //
    //! For each of the eActiveWithXxx settings:
    //!  AM_HAL_PWRCTRL_SRAM_NONE    = This component has no association with SSRAM.
    //!  AM_HAL_PWRCTRL_SRAM_1M_GRP0 = Group1 ignored by this component. Group0 is active when this component is powered on.
    //!  AM_HAL_PWRCTRL_SRAM_1M_GRP1 = Group0 ignored by this component. Group1 is active when this component is powered on.
    //!  AM_HAL_PWRCTRL_SRAM_ALL     = All SRAM active when this component is powered on.
    //

    //
    //! Activate SRAM when the MCU is active.
    //
    am_hal_pwrctrl_sram_select_e        eActiveWithMCU;

    //
    //! Activate SRAM when the Graphics is active.
    //
    am_hal_pwrctrl_sram_select_e        eActiveWithGFX;

    //
    //! Activate SRAM when the DISPLAY is active.
    //
    am_hal_pwrctrl_sram_select_e        eActiveWithDISP;

    //
    //! Retain SRAM in deep sleep.
    //! For SRAM retention:
    //!  AM_HAL_PWRCTRL_SRAM_NONE    = Retain all SRAM in deepsleep.
    //!  AM_HAL_PWRCTRL_SRAM_1M_GRP0 = Retain Group0, power down Group1 in deepsleep.
    //!  AM_HAL_PWRCTRL_SRAM_1M_GRP1 = Retain Group1, power down Group0 in deepsleep.
    //!  AM_HAL_PWRCTRL_SRAM_ALL     = Retain all SRAM in deepsleep.
    //
    am_hal_pwrctrl_sram_select_e        eSRAMRetain;

} am_hal_pwrctrl_sram_memcfg_t;

//*****************************************************************************
//
//! System power status structure
//
//*****************************************************************************
typedef struct
{
    //
    //! DEVPWRSTATUS - Device Power ON Status
    //
    uint32_t ui32Device;

    //
    //! AUDSSPWRSTATUS - Audio Subsystem ON Status
    //
    uint32_t ui32AudioSS;

    //
    //! MEMPWRSTATUS - MCU Memory Power ON Status
    //
    uint32_t ui32Memory;

    //
    //! SYSPWRSTATUS - Power ON Status for MCU Core
    //
    uint32_t ui32System;

    //
    //! SSRAMPWRST - Shared SRAM Power ON Status
    //
    uint32_t ui32SSRAM;

    //
    //! AUDSSPWRSTATUS - Audio Subsystem Power ON Status
    //
    uint32_t ui32Audio;

    //
    //! VRSTATUS - Voltage Regulators status
    //
    uint32_t ui32VRStatus;

    //
    //! ADCSTATUS - Power Status Register for ADC Block
    //
    uint32_t ui32ADC;

    //
    //! AUDADCSTATUS - Power Status Register for audio ADC Block
    //
    uint32_t ui32AudioADC;
} am_hal_pwrctrl_status_t;

//*****************************************************************************
//
//! INFO1 register structure
//
//*****************************************************************************
typedef struct
{
    uint32_t ui32INFO1GlobalValid;

    uint32_t ui32SBLVer0;                   // 00 Apollo5b MRAM
    uint32_t ui32SBLVer1;                   // 04 Apollo5b MRAM

    uint32_t ui32MAINPTR;                   // 10
    uint32_t ui32SBLOTA;                    // 18 Apollo5b
// #### INTERNAL BEGIN ####
//  uint32_t ui32resvd14;                   // 14
//  uint32_t ui32SBLOTA;                    // 18 Apollo5b
//  uint32_t ui32resvd1C;                   // 1C
// #### INTERNAL END ####

    uint32_t ui32SOCID0;                    // 20 OTP
    uint32_t ui32SOCID1;                    // 24 OTP
    uint32_t ui32SOCID2;                    // 28 OTP
    uint32_t ui32SOCID3;                    // 2C OTP
    uint32_t ui32SOCID4;                    // 30 OTP
    uint32_t ui32SOCID5;                    // 34 OTP
    uint32_t ui32SOCID6;                    // 38 OTP
    uint32_t ui32SOCID7;                    // 3C OTP

// #### INTERNAL BEGIN ####
//  uint32_t ui32PATCH_TRACKER0;            // 40
//  uint32_t ui32PATCH_TRACKER1;            // 44
//  uint32_t ui32PATCH_TRACKER2;            // 48
//  uint32_t ui32PATCH_TRACKER3;            // 4C
//  uint32_t ui32SBR_SDCERT_ADDR;           // 50
//  uint32_t ui32resvd54;                   // 54
//  uint32_t ui32SBR_IPT_ADDR;              // 58
//  uint32_t ui32SBR_OPT_ADDR;              // 5C
//  uint32_t ui32TRIM_SBR_OTP;              // 60 Apollo5b
// #### INTERNAL END ####
    uint32_t ui32TEMP_CAL_ATE;              // 00
    uint32_t ui32TEMP_CAL_MEASURED;         // 04
    uint32_t ui32TEMP_CAL_ADC_OFFSET;       // 08
// #### INTERNAL BEGIN ####
//  uint32_t ui32CHIPSUBREV;                // 0C Apollo5b
//  uint32_t ui32TRIM_REV;                  // 10
//  uint32_t ui32FT1_GDR1;                  // 14 Apollo5b
//  uint32_t ui32FT2_GDR1;                  // 18 Apollo5b
//  uint32_t ui32LVT_TRIMCODE;              // 1C Apollo5b
//  uint32_t ui32EHVT_TRIMCODE;             // 20 Apollo5b
//  uint32_t ui32AUDADC_BINNING;            // 24
// #### INTERNAL END ####
    uint32_t ui32ADC_GAIN_ERR;              // 28
    uint32_t ui32ADC_OFFSET_ERR;            // 2C

    uint32_t ui32AUDADC_A0_LG_OFFSET;       // 40
    uint32_t ui32AUDADC_A0_HG_SLOPE;        // 44
    uint32_t ui32AUDADC_A0_HG_INTERCEPT;    // 48
    uint32_t ui32AUDADC_A1_LG_OFFSET;       // 4C
    uint32_t ui32AUDADC_A1_HG_SLOPE;        // 50
    uint32_t ui32AUDADC_A1_HG_INTERCEPT;    // 54
    uint32_t ui32AUDADC_B0_LG_OFFSET;       // 58
    uint32_t ui32AUDADC_B0_HG_SLOPE;        // 5C
    uint32_t ui32AUDADC_B0_HG_INTERCEPT;    // 60
    uint32_t ui32AUDADC_B1_LG_OFFSET;       // 64
    uint32_t ui32AUDADC_B1_HG_SLOPE;        // 68
    uint32_t ui32AUDADC_B1_HG_INTERCEPT;    // 6C
} am_hal_pwrctrl_info1_regs_t;

#define INFO1DATAENTRIES    (sizeof(am_hal_pwrctrl_info1_regs_t) / 4)

//
// Define a value used in ui32INFO1GlobalValid to indicate
// that INFO1 has been saved globally.
//
#define INFO1GLOBALVALID          0x1F01600D

extern am_hal_pwrctrl_info1_regs_t g_sINFO1regs;

//*****************************************************************************
//
//! RLPSTATE, powerup state for PDRAMS power domain.
//
//*****************************************************************************
typedef enum
{
    AM_HAL_PWRCTRL_RLP_ON,
    AM_HAL_PWRCTRL_RLP_RESERVED,
    AM_HAL_PWRCTRL_RLP_RESERVED1,
    AM_HAL_PWRCTRL_RLP_OFF
} am_hal_pwrctrl_pwrmodctl_rlp_e;

//*****************************************************************************
//
//! ELPSTATE, type of low-power state for PDEPU.
//
//*****************************************************************************
typedef enum
{
    AM_HAL_PWRCTRL_ELP_ON,
    AM_HAL_PWRCTRL_ELP_ON_CLK_OFF,
    AM_HAL_PWRCTRL_ELP_RET,
    AM_HAL_PWRCTRL_ELP_OFF
} am_hal_pwrctrl_pwrmodctl_elp_e;

//*****************************************************************************
//
//! CLPSTATE, type of low-power state for PDCORE.
//
//*****************************************************************************
typedef enum
{
    AM_HAL_PWRCTRL_CLP_ON,
    AM_HAL_PWRCTRL_CLP_ON_CLK_OFF,
    AM_HAL_PWRCTRL_CLP_RET,
    AM_HAL_PWRCTRL_CLP_OFF
} am_hal_pwrctrl_pwrmodctl_clp_e;

//*****************************************************************************
//
//! CPDLPSTATE structure
//
//*****************************************************************************
typedef struct
{
    //
    //!RLPSTATE
    //
    am_hal_pwrctrl_pwrmodctl_rlp_e eRlpConfig;

    //
    //!ELPSTATE
    //
    am_hal_pwrctrl_pwrmodctl_elp_e eElpConfig;

    //
    //!CLPSTATE
    //
    am_hal_pwrctrl_pwrmodctl_clp_e eClpConfig;
} am_hal_pwrctrl_pwrmodctl_cpdlp_t;

//*****************************************************************************
//
//! @name Default configurations
//! @{
//
//*****************************************************************************
extern const am_hal_pwrctrl_mcu_memory_config_t      g_DefaultMcuMemCfg;
extern const am_hal_pwrctrl_sram_memcfg_t            g_DefaultSRAMCfg;
//! @}

//*****************************************************************************
//
//! @name Global Variables
//! @{
//
// ****************************************************************************
extern uint32_t g_ui32TrimVer;
//! @}


// ****************************************************************************
//
//! @brief Return the current MCU performance mode.
//!
//! @param peCurrentPowerMode is a ptr to a variable to save the current status.
//!
//! @return AM_HAL_STATUS_SUCCESS or applicable PWRCTRL errors.
//
// ****************************************************************************
extern uint32_t am_hal_pwrctrl_mcu_mode_status(am_hal_pwrctrl_mcu_mode_e *peCurrentPowerMode);

//*****************************************************************************
//
//! @brief Change the MCU performance mode.
//!
//! @param ePowerMode is the performance mode for the MCU
//!
//! Select the power mode for the MCU.
//!
//! @return AM_HAL_STATUS_SUCCESS or applicable PWRCTRL errors.
//
//*****************************************************************************
extern uint32_t am_hal_pwrctrl_mcu_mode_select(am_hal_pwrctrl_mcu_mode_e ePowerMode);


//*****************************************************************************
//
//! @brief Configure the power settings for the MCU memory.
//!
//! @param psConfig is a structure describing the desired memory configuration.
//!
//! Use this function to enable, disable, or change the sleep configuration of
//! MCU memory.
//!
//! @return AM_HAL_STATUS_SUCCESS or applicable PWRCTRL errors
//
//*****************************************************************************
extern uint32_t am_hal_pwrctrl_mcu_memory_config(am_hal_pwrctrl_mcu_memory_config_t *psConfig);

//*****************************************************************************
//
//! @brief Read the power settings for the MCU memory.
//!
//! @param psConfig is a structure describing the desired memory configuration.
//!
//! Use this function to check the current settings for the MCU memories.
//!
//! @return AM_HAL_STATUS_SUCCESS or applicable PWRCTRL errors
//
//*****************************************************************************
extern uint32_t am_hal_pwrctrl_mcu_memory_config_get(am_hal_pwrctrl_mcu_memory_config_t *psConfig);

//*****************************************************************************
//
//! @brief Enable the ROM power domain.
//!
//! @param none.
//!
//! Use this function to enable the ROM power domain.  If the global ROM power
//! state is AM_HAL_PWRCTRL_ROM_ALWAYS_ON, then this function will have no effect.
//!
//! @return AM_HAL_STATUS_SUCCESS or applicable PWRCTRL errors
//
//*****************************************************************************
extern uint32_t am_hal_pwrctrl_rom_enable(void);

//*****************************************************************************
//
//! @brief Disable the ROM power domain.
//!
//! @param none.
//!
//! Use this function to disable the ROM power domain.  If the global ROM power
//! state is AM_HAL_PWRCTRL_ROM_ALWAYS_ON, then this function will have no effect.
//!
//! @return AM_HAL_STATUS_SUCCESS or applicable PWRCTRL errors
//
//*****************************************************************************
extern uint32_t am_hal_pwrctrl_rom_disable(void);

//*****************************************************************************
//
//! @brief Configure the power settings for the Shared RAM.
//!
//! @param psConfig is a structure describing the desired memory configuration.
//!
//! Use this function to enable, disable, or change the sleep configuration of
//! MCU memory.
//!
//! @return AM_HAL_STATUS_SUCCESS or applicable PWRCTRL errors
//
//*****************************************************************************
extern uint32_t am_hal_pwrctrl_sram_config(am_hal_pwrctrl_sram_memcfg_t *psConfig);

//*****************************************************************************
//
//! @brief Get the power settings for the Shared RAM.
//!
//! @param psConfig is a structure describing the desired memory configuration.
//!
//! Use this function to check the current settings for the MCU memories.
//!
//! @return AM_HAL_STATUS_SUCCESS or applicable PWRCTRL errors
//
//*****************************************************************************
extern uint32_t am_hal_pwrctrl_sram_config_get(am_hal_pwrctrl_sram_memcfg_t *psConfig);

//*****************************************************************************
//
//! @brief Enable power to a peripheral.
//!
//! @param ePeripheral - The peripheral to enable.
//!
//! This function enables power to the peripheral and waits for a
//! confirmation from the hardware.
//!
//! @return AM_HAL_STATUS_SUCCESS or applicable PWRCTRL errors
//
//*****************************************************************************
extern uint32_t am_hal_pwrctrl_periph_enable(am_hal_pwrctrl_periph_e ePeripheral);

//*****************************************************************************
//
//! @brief Disable power to a peripheral.
//!
//! @param ePeripheral - The peripheral to disable.
//!
//! This function disables power to the peripheral and waits for a
//! confirmation from the hardware.
//!
//! @return AM_HAL_STATUS_SUCCESS or applicable PWRCTRL errors
//
//*****************************************************************************
extern uint32_t am_hal_pwrctrl_periph_disable(am_hal_pwrctrl_periph_e ePeripheral);

//*****************************************************************************
//
//! @brief Determine whether a peripheral is currently enabled.
//!
//! @param ePeripheral - The peripheral to enable.
//! @param bEnabled - Pointer to a ui32 that will return as 1 or 0.
//!
//! This function determines to the caller whether a given peripheral is
//! currently enabled or disabled.
//!
//! @return AM_HAL_STATUS_SUCCESS or applicable PWRCTRL errors
//
//*****************************************************************************
extern uint32_t am_hal_pwrctrl_periph_enabled(am_hal_pwrctrl_periph_e ePeripheral,
                                              bool *bEnabled);

//*****************************************************************************
//
//! @brief Get the current powercontrol status registers.
//!
//! @param psStatus returns a structure containing power status information.
//!
//! This function can be used to determine the current status of a wide variety
//! of system components.
//!
//! @return AM_HAL_STATUS_SUCCESS or applicable PWRCTRL errors
//
//*****************************************************************************
extern uint32_t am_hal_pwrctrl_status_get(am_hal_pwrctrl_status_t *psStatus);

// #### INTERNAL BEGIN ####
// ****************************************************************************
// Brief version of
// am_hal_pwrctrl_periph_enabled(AM_HAL_PWRCTRL_PERIPH_OTP, &bEnabled);
// ****************************************************************************
#define am_hal_pwrctrl_is_OTP_enabled()     \
    (PWRCTRL->DEVPWRSTATUS_b.PWRSTOTP ? true : false)
// #### INTERNAL END ####
//*****************************************************************************
//
//! @brief Initialize system for low power configuration.
//!
//! This function implements various low power initialization and optimizations.
//! - See also am_hal_pwrctrl_control() for other power saving techniques.
//!
//! @return AM_HAL_STATUS_SUCCESS or applicable PWRCTRL errors
//
//*****************************************************************************
extern uint32_t am_hal_pwrctrl_low_power_init(void);

//*****************************************************************************
//
//! @brief Miscellaneous power saving controls
//!
//! @param eControl - Power saving type, one of the following:
//!     AM_HAL_PWRCTRL_CONTROL_SIMOBUCK_INIT        - Enable the SIMOBUCK
//!     AM_HAL_PWRCTRL_CONTROL_XTAL_PWDN_DEEPSLEEP  - Allow the crystal to power
//!                                                   down during deepsleep
//!
//! @param pArgs - Pointer to arguments for Control Switch Case
//!
//! @return AM_HAL_STATUS_SUCCESS or applicable PWRCTRL errors
//
//*****************************************************************************
extern uint32_t am_hal_pwrctrl_control(am_hal_pwrctrl_control_e eControl, void *pArgs);

//*****************************************************************************
//
//! @brief Restore original Power settings
//!
//! This function restores default power trims reverting relative
//! changes done as part of low_power_init and SIMOBUCK init.
//! User needs to make sure device is running in Low Power mode before calling
//! this function.
//!
//! @return AM_HAL_STATUS_SUCCESS or applicable PWRCTRL errors
//
//*****************************************************************************
extern uint32_t am_hal_pwrctrl_settings_restore(void);

#if AM_HAL_TEMPCO_LP
//*****************************************************************************
//
//! @brief Initialize the TempCo workaround.
//!
//! @param pADCHandle - Pointer to ADC Handle
//! @param ui32ADCslot - ADc Slot to initialize
//!
//! @return AM_HAL_STATUS_SUCCESS or applicable PWRCTRL errors
//
//*****************************************************************************
extern uint32_t am_hal_pwrctrl_tempco_init(void *pADCHandle,
                                           uint32_t ui32ADCslot);

// ****************************************************************************
//
//! @brief This function to be called from the ADC or timer ISR.
//!
//! @param ui32NumSamples - Number of sSamples to process
//! @param sSamples - Array of ADC Samples
//!
//! @return AM_HAL_STATUS_SUCCESS or applicable PWRCTRL errors
//
// ****************************************************************************
extern uint32_t am_hal_pwrctrl_tempco_sample_handler(uint32_t ui32NumSamples,
                                                     am_hal_adc_sample_t sSamples[]);
#endif // AM_HAL_TEMPCO_LP

//*****************************************************************************
//
//! @brief Configure CPDLPSTATE.
//!
//! @param sCpdlpConfig - CPDLPSTATE configurations.
//!
//! @return AM_HAL_STATUS_SUCCESS or applicable PWRCTRL errors
//
//*****************************************************************************
extern uint32_t am_hal_pwrctrl_pwrmodctl_cpdlp_config(am_hal_pwrctrl_pwrmodctl_cpdlp_t sCpdlpConfig);

//*****************************************************************************
//
//! @brief Get CPDLPSTATE configurations.
//!
//! @param psCpdlpConfig - Pointer of CPDLPSTATE configurations structure.
//!
//! @return None.
//
//*****************************************************************************
extern void am_hal_pwrctrl_pwrmodctl_cpdlp_get(am_hal_pwrctrl_pwrmodctl_cpdlp_t * psCpdlpConfig);

//*****************************************************************************
//
//! @brief Decides CPU will enter HP1 and wait until PLL_LOCK and switch to HP2.
//!
//! @param bWaitPllockForHp2 - true: CPU will NOT enter HP1. It will Wait for PLL_LOCK and then directly enter HP2 mode.
//!                            false:CPU will enter HP1 until PLL lock and then switch to HP2.
//!
//! @return None.
//
//*****************************************************************************
extern void am_hal_pwrctrl_wait_pll_lock_for_hp2(bool bWaitPllockForHp2);
#ifdef __cplusplus
}
#endif

#endif // AM_HAL_PWRCTRL_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
