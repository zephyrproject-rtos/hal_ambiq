//*****************************************************************************
//
//! @file am_hal_sysctrl.c
//!
//! @brief Functions for interfacing with the M4F system control registers
//!
//! @addtogroup sysctrl5 SYSCTRL - System Control
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
//  Globals
//
//*****************************************************************************

extern void buck_ldo_update_override(bool bEnable);

//*****************************************************************************
//
// Place the core into sleep or deepsleep.
//
// This function puts the MCU to sleep or deepsleep depending on bSleepDeep.
//
// Valid values for bSleepDeep are:
//     AM_HAL_SYSCTRL_SLEEP_NORMAL
//     AM_HAL_SYSCTRL_SLEEP_DEEP
//
//*****************************************************************************

void
am_hal_sysctrl_sleep(bool bSleepDeep)
{
    bool bBuckIntoLPinDS = false, bSimobuckAct = false, bRecoverVRCTRL = false;
    uint32_t ui32VRCTRLCache = 0;
    am_hal_pwrctrl_pwrmodctl_cpdlp_t sActCpdlpConfig;

    //
    // Disable interrupts and save the previous interrupt state.
    //
    AM_CRITICAL_BEGIN

    //
    // Get the current CPDLPSTATE configuration in active mode
    //
    am_hal_pwrctrl_pwrmodctl_cpdlp_get(&sActCpdlpConfig);

    //
    // Get current mode.
    //
    bSimobuckAct = ( PWRCTRL->VRSTATUS_b.SIMOBUCKST == PWRCTRL_VRSTATUS_SIMOBUCKST_ACT );

    //
    // If the user selected DEEPSLEEP and OTP & ROM are off, attempt to enter
    // DEEP SLEEP.
    // CPU cannot go to deepsleep if either OTP or ROM is still powered on
    // #### INTERNAL BEGIN ####
    // CAB-926
    // #### INTERNAL END ####
    //
    if ((bSleepDeep == AM_HAL_SYSCTRL_SLEEP_DEEP)
        && (!PWRCTRL->DEVPWRSTATUS_b.PWRSTOTP && !PWRCTRL->MEMPWRSTATUS_b.PWRSTROM))
    {
        //
        // Set the CPDLPSTATE configuration in deepsleep mode
        //
        am_hal_pwrctrl_pwrmodctl_cpdlp_t sDSCpdlpConfig =
        {
            .eRlpConfig = sActCpdlpConfig.eRlpConfig,
            .eElpConfig = AM_HAL_PWRCTRL_ELP_RET,
            .eClpConfig = AM_HAL_PWRCTRL_CLP_RET
        };
        am_hal_pwrctrl_pwrmodctl_cpdlp_config(sDSCpdlpConfig);
        //
        // Check if SIMOBUCK needs to stay in Active mode in DeepSleep
        //
        if ( bSimobuckAct )
        {
            //
            // Check if SIMOBUCK would go into LP mode in DeepSleep
            //
            if ( !(PWRCTRL->DEVPWRSTATUS &
                    (PWRCTRL_DEVPWRSTATUS_PWRSTDBG_Msk      |
                    PWRCTRL_DEVPWRSTATUS_PWRSTUSBPHY_Msk   |
                    PWRCTRL_DEVPWRSTATUS_PWRSTUSB_Msk      |
                    PWRCTRL_DEVPWRSTATUS_PWRSTSDIO1_Msk    |
                    PWRCTRL_DEVPWRSTATUS_PWRSTSDIO0_Msk    |
                    PWRCTRL_DEVPWRSTATUS_PWRSTCRYPTO_Msk   |
                  /*PWRCTRL_DEVPWRSTATUS_PWRSTDISPPHY_Msk  |
                    PWRCTRL_DEVPWRSTATUS_PWRSTDISP_Msk     |*/
                    PWRCTRL_DEVPWRSTATUS_PWRSTGFX_Msk      |
                    PWRCTRL_DEVPWRSTATUS_PWRSTMSPI2_Msk    |
                    PWRCTRL_DEVPWRSTATUS_PWRSTMSPI1_Msk    |
                    PWRCTRL_DEVPWRSTATUS_PWRSTMSPI0_Msk    |
                    PWRCTRL_DEVPWRSTATUS_PWRSTADC_Msk      |
                  /*PWRCTRL_DEVPWRSTATUS_PWRSTUART3_Msk    |
                    PWRCTRL_DEVPWRSTATUS_PWRSTUART2_Msk    |*/
                    PWRCTRL_DEVPWRSTATUS_PWRSTUART1_Msk    |
                    PWRCTRL_DEVPWRSTATUS_PWRSTUART0_Msk    |
                  /*PWRCTRL_DEVPWRSTATUS_PWRSTIOM7_Msk     |
                    PWRCTRL_DEVPWRSTATUS_PWRSTIOM6_Msk     |*/
                    PWRCTRL_DEVPWRSTATUS_PWRSTIOM5_Msk     |
                    PWRCTRL_DEVPWRSTATUS_PWRSTIOM4_Msk     |
                    PWRCTRL_DEVPWRSTATUS_PWRSTIOM3_Msk     |
                    PWRCTRL_DEVPWRSTATUS_PWRSTIOM2_Msk     |
                    PWRCTRL_DEVPWRSTATUS_PWRSTIOM1_Msk     |
                    PWRCTRL_DEVPWRSTATUS_PWRSTIOM0_Msk     |
                    PWRCTRL_DEVPWRSTATUS_PWRSTIOSFD0_Msk   |
                    PWRCTRL_DEVPWRSTATUS_PWRSTIOSFD1_Msk   |
                 /* PWRCTRL_DEVPWRSTATUS_PWRSTIOS0_Msk     |*/
                    PWRCTRL_DEVPWRSTATUS_PWRSTOTP_Msk ))      &&
                !(PWRCTRL->MEMPWRSTATUS &
                  PWRCTRL_MEMPWRSTATUS_PWRSTROM_Msk ))
            {
                bBuckIntoLPinDS = true;

                //
                // Remove overrides to allow buck to go in LP mode
                //
                buck_ldo_update_override(false);
            }
        }

        //
        // Check and confirm Simobuck/LDO is forced active if PLL is enabled
        //
        // #### INTERNAL BEGIN ####
        // Note: We are implementing forced Simobuck/LDO active handling that
        // is decoupled with LDO-IN-PARALLEL feature since LDO-IN-PARALLEL is
        // planned to be removed. Hence, buck-ldo override mechanism above
        // is not utilized.
        // #### INTERNAL END ####
        if ( MCUCTRL->PLLCTL0_b.SYSPLLPDB == MCUCTRL_PLLCTL0_SYSPLLPDB_ENABLE )
        {
            //
            // If neither of Buck nor MemLDO is forced active, force power
            // active accordingly.
            //
            bool bBuckForced  = MCUCTRL->VRCTRL_b.SIMOBUCKOVER && MCUCTRL->VRCTRL_b.SIMOBUCKACTIVE;
            bool bMemLdoForced   = MCUCTRL->VRCTRL_b.MEMLDOOVER && MCUCTRL->VRCTRL_b.MEMLDOACTIVE;
            if ( !bBuckForced && !bMemLdoForced )
            {
                ui32VRCTRLCache = MCUCTRL->VRCTRL;
                bRecoverVRCTRL = true;

                if ( bSimobuckAct )
                {
                    MCUCTRL->VRCTRL_b.SIMOBUCKACTIVE = 1;
                    MCUCTRL->VRCTRL_b.SIMOBUCKOVER = 1;
                }
                else
                {
                    MCUCTRL->VRCTRL_b.MEMLDOACTIVE = 1;
                    MCUCTRL->VRCTRL_b.MEMLDOOVER = 1;
                }
            }
        }

        //
        // Prepare the core for deepsleep (write 1 to the DEEPSLEEP bit).
        //
        SCB->SCR |= _VAL2FLD(SCB_SCR_SLEEPDEEP, 1);
    }
    else
    {
        //
        // Set the CPDLPSTATE configuration in normal sleep mode
        //
        am_hal_pwrctrl_pwrmodctl_cpdlp_t sNSCpdlpConfig;
        if (PWRCTRL->CPUPWRCTRL_b.SLEEPMODE) // ARM sleep
        {
            sNSCpdlpConfig.eRlpConfig = sActCpdlpConfig.eRlpConfig;
            sNSCpdlpConfig.eElpConfig = AM_HAL_PWRCTRL_ELP_ON_CLK_OFF;
            sNSCpdlpConfig.eClpConfig = AM_HAL_PWRCTRL_CLP_ON_CLK_OFF;
        }
        else // Ambiq sleep
        {
            sNSCpdlpConfig.eRlpConfig = sActCpdlpConfig.eRlpConfig;
            sNSCpdlpConfig.eElpConfig = AM_HAL_PWRCTRL_ELP_ON_CLK_OFF; // or can leave at 0x0 as we will turn the clocks off at the source
            sNSCpdlpConfig.eClpConfig = AM_HAL_PWRCTRL_CLP_ON_CLK_OFF; // or can leave at 0x0 as we will turn the clocks off at the source
        }
        am_hal_pwrctrl_pwrmodctl_cpdlp_config(sNSCpdlpConfig);

        //
        // Prepare the core for normal sleep (write 0 to the DEEPSLEEP bit).
        //
        SCB->SCR &= ~_VAL2FLD(SCB_SCR_SLEEPDEEP, 1);
    }

    //
    // Before executing WFI, flush APB writes.
    //
    am_hal_sysctrl_sysbus_write_flush();

    //
    // Execute the sleep instruction.
    //
    __WFI();

    //
    // Upon wake, execute the Instruction Sync Barrier instruction.
    //
    __ISB();

    if ( bRecoverVRCTRL )
    {
        //
        // Recover VRCTRL settings if it has been changed before sleep.
        //
        MCUCTRL->VRCTRL = ui32VRCTRLCache;
    }

    if ( bBuckIntoLPinDS )
    {
        //
        // Re-enable overrides
        //
        buck_ldo_update_override(true);
    }

    //
    // Restore the CPDLPSTATE
    //
    am_hal_pwrctrl_pwrmodctl_cpdlp_config(sActCpdlpConfig);

    //
    // Restore the interrupt state.
    //
    AM_CRITICAL_END
}

//*****************************************************************************
//
// Enable the floating point module.
//
//*****************************************************************************
void
am_hal_sysctrl_fpu_enable(void)
{
    //
    //! Enable the EPU. See section 6.3 of the CM55 TRM.

    //
    SCB->CPACR |= (0xF << 20);
    __DSB();
    __ISB();
}

//*****************************************************************************
//
// Disable the floating point module.
//
//*****************************************************************************
void
am_hal_sysctrl_fpu_disable(void)
{
    //
    // Disable access to the FPU in both privileged and user modes.
    // NOTE: Write 0s to all reserved fields in this register.
    //
    SCB->CPACR |= ~(0xF << 20);
    __DSB();
    __ISB();
}

//*****************************************************************************
//
// Enable stacking of FPU registers on exception entry.
//
//*****************************************************************************
void
am_hal_sysctrl_fpu_stacking_enable(bool bLazy)
{
    uint32_t ui32fpccr;

    //
    // Set the requested FPU stacking mode in ISRs.
    //
    AM_CRITICAL_BEGIN
#define SYSCTRL_FPCCR_LAZY  (FPU_FPCCR_ASPEN_Msk | FPU_FPCCR_LSPEN_Msk)
    ui32fpccr  = FPU->FPCCR;
    ui32fpccr &= ~SYSCTRL_FPCCR_LAZY;
    ui32fpccr |= (bLazy ? SYSCTRL_FPCCR_LAZY : FPU_FPCCR_ASPEN_Msk);
    FPU->FPCCR = ui32fpccr;
    AM_CRITICAL_END
}

//*****************************************************************************
//
// Disable FPU register stacking on exception entry.
//
//*****************************************************************************
void
am_hal_sysctrl_fpu_stacking_disable(void)
{
    //
    // Completely disable FPU context save on entry to ISRs.
    //
    AM_CRITICAL_BEGIN
    FPU->FPCCR &= ~SYSCTRL_FPCCR_LAZY;
    AM_CRITICAL_END
}

//*****************************************************************************
//
// Issue a system wide reset using the AIRCR bit in the M4 system ctrl.
//
//*****************************************************************************
void
am_hal_sysctrl_aircr_reset(void)
{
    //
    // Set the system reset bit in the AIRCR register
    //
    __NVIC_SystemReset();
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
