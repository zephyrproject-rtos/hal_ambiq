//*****************************************************************************
//
//! @file am_hal_timer.c
//!
//! @brief
//!
//! @addtogroup timer Timer Functionality
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
#warning TODO:Need delete it when add spot manager.
#define AM_HAL_INTERNAL_TIMER_NUM_A     15
// #### INTERNAL END ####
//
// Not declared as static as this function can be used from within HAL
//
uint32_t
internal_timer_config(uint32_t ui32TimerNumber,
                      am_hal_timer_config_t *psTimerConfig)
{
    uint32_t ui32ConfigCtrl, ui32ConfigMode;
    uint32_t ui32TimerLimit = psTimerConfig->ui32PatternLimit;
    uint32_t ui32Compare0 = psTimerConfig->ui32Compare0;
    uint32_t ui32Compare1 = psTimerConfig->ui32Compare1;

    if ((ui32TimerNumber == AM_HAL_INTERNAL_TIMER_NUM_B) || (ui32TimerNumber == AM_HAL_INTERNAL_TIMER_NUM_A))
    {
        //
        // Timer AM_HAL_INTERNAL_TIMER_NUM_B and AM_HAL_INTERNAL_TIMER_NUM_A are used by HAL.
        //
        return AM_HAL_STATUS_IN_USE;
    }

    //
    // Mode/function specific error checking.
    //
    switch ( psTimerConfig->eFunction )
    {
        case AM_HAL_TIMER_FN_EDGE:
            //
            // If Compare1 is used, then compare0 must be set to a higher value than compare1.
            //
            if ((ui32Compare1 != 0xFFFFFFFF) && (ui32Compare0 <= ui32Compare1))
            {
                return AM_HAL_STATUS_INVALID_OPERATION;
            }
            break;

        case AM_HAL_TIMER_FN_PWM:
            break;

        case AM_HAL_TIMER_FN_UPCOUNT:
            break;

        case AM_HAL_TIMER_FN_SINGLEPATTERN:
        case AM_HAL_TIMER_FN_REPEATPATTERN:
            //
            // Check that requested pattern is not too long.
            //
            if (psTimerConfig->ui32PatternLimit > 63)
            {
                return AM_HAL_STATUS_INVALID_OPERATION;
            }

            break;

// #### INTERNAL BEGIN ####
        // TODO - Remove deprecated TIMER functions.
        //case AM_HAL_TIMER_FN_CONTINUOUS:
        //case AM_HAL_TIMER_FN_DOWNCOUNT:
        //case AM_HAL_TIMER_FN_EVENTTIMER:
// #### INTERNAL END ####
        default:
            return AM_HAL_STATUS_INVALID_OPERATION;
    }

    //
    //
    // Build up a value in SRAM before we start writing to the timer control
    // registers.
    //
    ui32ConfigCtrl  = _VAL2FLD(TIMER_CTRL0_TMR0CLK,     psTimerConfig->eInputClock);
    ui32ConfigCtrl |= _VAL2FLD(TIMER_CTRL0_TMR0FN,      psTimerConfig->eFunction);
    ui32ConfigCtrl |= _VAL2FLD(TIMER_CTRL0_TMR0POL1,    psTimerConfig->bInvertOutput1);
    ui32ConfigCtrl |= _VAL2FLD(TIMER_CTRL0_TMR0POL0,    psTimerConfig->bInvertOutput0);
    ui32ConfigCtrl |= _VAL2FLD(TIMER_CTRL0_TMR0TMODE,   psTimerConfig->eTriggerType);
    ui32ConfigCtrl |= _VAL2FLD(TIMER_CTRL0_TMR0LMT,     ui32TimerLimit);
    ui32ConfigCtrl |= _VAL2FLD(TIMER_CTRL0_TMR0EN, 0);

    ui32ConfigMode  = _VAL2FLD(TIMER_MODE0_TMR0TRIGSEL, psTimerConfig->eTriggerSource);

    //
    // Disable the timer.
    //
    TIMERn(ui32TimerNumber)->CTRL0_b.TMR0EN = 0;

    //
    // Apply the settings from the configuration structure.
    //
    TIMERn(ui32TimerNumber)->CTRL0 = ui32ConfigCtrl;
    TIMERn(ui32TimerNumber)->MODE0 = ui32ConfigMode;
    TIMERn(ui32TimerNumber)->TMR0CMP0 = ui32Compare0;
    TIMERn(ui32TimerNumber)->TMR0CMP1 = ui32Compare1;

    //
    // Clear the timer to make sure it has the appropriate starting value.
    //
    TIMERn(ui32TimerNumber)->CTRL0_b.TMR0CLR = 1;

    return AM_HAL_STATUS_SUCCESS;

} // internal_timer_config()

uint32_t
am_hal_timer_config(uint32_t ui32TimerNumber,
                    am_hal_timer_config_t *psTimerConfig)
{
    if ((ui32TimerNumber == AM_HAL_INTERNAL_TIMER_NUM_B) || (ui32TimerNumber == AM_HAL_INTERNAL_TIMER_NUM_A))
    {
        //
        // Timer AM_HAL_INTERNAL_TIMER_NUM_B and AM_HAL_INTERNAL_TIMER_NUM_A are used by HAL.
        //
        return AM_HAL_STATUS_IN_USE;
    }
    return internal_timer_config(ui32TimerNumber, psTimerConfig);
}

//
// Initialize a timer configuration structure with default values.
//
uint32_t
am_hal_timer_default_config_set(am_hal_timer_config_t *psTimerConfig)
{

    psTimerConfig->eInputClock = AM_HAL_TIMER_CLOCK_HFRC_DIV16;
    psTimerConfig->eFunction = AM_HAL_TIMER_FN_EDGE;
    psTimerConfig->ui32Compare0 = 0xFFFFFFFF;
    psTimerConfig->ui32Compare1 = 0xFFFFFFFF;
    psTimerConfig->bInvertOutput0 = false;
    psTimerConfig->bInvertOutput1 = false;
    psTimerConfig->eTriggerType = AM_HAL_TIMER_TRIGGER_DIS;
    psTimerConfig->eTriggerSource = AM_HAL_TIMER_TRIGGER_TMR0_OUT1;
    psTimerConfig->ui32PatternLimit = 0;

    return AM_HAL_STATUS_SUCCESS;
}

//
// Reset the timer to the default state.
//
uint32_t
am_hal_timer_reset_config(uint32_t ui32TimerNumber)
{
    if ((ui32TimerNumber == AM_HAL_INTERNAL_TIMER_NUM_B) || (ui32TimerNumber == AM_HAL_INTERNAL_TIMER_NUM_A))
    {
        //
        // Timer AM_HAL_INTERNAL_TIMER_NUM_B and AM_HAL_INTERNAL_TIMER_NUM_A are used by HAL.
        //
        return AM_HAL_STATUS_IN_USE;
    }
    //
    // Disable Interrupts.
    //
    am_hal_timer_interrupt_disable(3 << (ui32TimerNumber * 2));

    //
    // Enable the Global enable.
    //
    am_hal_timer_enable_sync(1 << ui32TimerNumber);

    //
    // Disable the Timer.
    //
    am_hal_timer_disable(ui32TimerNumber);

    //
    // Reset the Timer specific registers.
    //
    TIMERn(ui32TimerNumber)->CTRL0 = 0;
    TIMERn(ui32TimerNumber)->TIMER0 = 0;
    TIMERn(ui32TimerNumber)->TMR0CMP0 = 0;
    TIMERn(ui32TimerNumber)->TMR0CMP1 = 0;
    TIMERn(ui32TimerNumber)->MODE0 = 0;

    am_hal_timer_interrupt_clear(3 << (ui32TimerNumber *2));

    return AM_HAL_STATUS_SUCCESS;
}

//
// Enable a single TIMER
//
uint32_t
am_hal_timer_enable(uint32_t ui32TimerNumber)
{
    if ((ui32TimerNumber == AM_HAL_INTERNAL_TIMER_NUM_B) || (ui32TimerNumber == AM_HAL_INTERNAL_TIMER_NUM_A))
    {
        //
        // Timer AM_HAL_INTERNAL_TIMER_NUM_B and AM_HAL_INTERNAL_TIMER_NUM_A are used by HAL.
        //
        return AM_HAL_STATUS_IN_USE;
    }
    AM_CRITICAL_BEGIN;
    //
    // Enable the timer in both the individual enable register and the global
    // sync register.
    //
    TIMER->GLOBEN |= 1 << ui32TimerNumber;

    //
    // Toggle the clear bit (required by the hardware), and then enable the timer.
    //
    TIMERn(ui32TimerNumber)->CTRL0_b.TMR0CLR = 1;
    TIMERn(ui32TimerNumber)->CTRL0_b.TMR0CLR = 0;
    TIMERn(ui32TimerNumber)->CTRL0_b.TMR0EN = 1;

    AM_CRITICAL_END;

    return AM_HAL_STATUS_SUCCESS;
}

//
// Disable a single TIMER
//
uint32_t
am_hal_timer_disable(uint32_t ui32TimerNumber)
{
    if ((ui32TimerNumber == AM_HAL_INTERNAL_TIMER_NUM_B) || (ui32TimerNumber == AM_HAL_INTERNAL_TIMER_NUM_A))
    {
        //
        // Timer AM_HAL_INTERNAL_TIMER_NUM_B and AM_HAL_INTERNAL_TIMER_NUM_A are used by HAL.
        //
        return AM_HAL_STATUS_IN_USE;
    }

    AM_CRITICAL_BEGIN;

    //
    // Disable the timer.
    //
    TIMERn(ui32TimerNumber)->CTRL0_b.TMR0EN = 0;

    AM_CRITICAL_END;

    return AM_HAL_STATUS_SUCCESS;
}

//
// Enable a group of TIMERS all at once
//
uint32_t
am_hal_timer_enable_sync(uint32_t ui32TimerMask)
{
    //
    // Disable the timers in the global sync register, make sure they are all
    // individually enabled, and then re-enable them in the global sync
    // register.
    //
    AM_CRITICAL_BEGIN;

    TIMER->GLOBEN &= ~(ui32TimerMask);

    for (uint32_t i = 0; i < AM_REG_NUM_TIMERS; i++)
    {
        if ((1 << i) & ui32TimerMask)
        {
            //
            // Toggle the clear bit (required by the hardware), and then enable the timer.
            //
            TIMERn(i)->CTRL0_b.TMR0CLR = 1;
            TIMERn(i)->CTRL0_b.TMR0CLR = 0;
            //
            // Enable the timer.
            //
            TIMERn(i)->CTRL0_b.TMR0EN = 1;
        }
    }

    TIMER->GLOBEN |= ui32TimerMask;

    AM_CRITICAL_END;

    return AM_HAL_STATUS_SUCCESS;
}

//
// Disable a group of TIMERS all at once
//
uint32_t
am_hal_timer_disable_sync(uint32_t ui32TimerMask)
{
    //
    // Disable the timers in the global sync register, make sure they are all
    // individually disabled, and then re-enable them in the global sync
    // register.
    //
    AM_CRITICAL_BEGIN;

    TIMER->GLOBEN &= ~(ui32TimerMask);

    for (uint32_t i = 0; i < AM_REG_NUM_TIMERS; i++)
    {
        if ((1 << i) & ui32TimerMask)
        {
            //
            // Disable the timer.
            //
            TIMERn(i)->CTRL0_b.TMR0EN = 0;
        }
    }

    AM_CRITICAL_END;

    return AM_HAL_STATUS_SUCCESS;
}

//
// Clear a single TIMER and start the timer.
//
uint32_t
am_hal_timer_clear(uint32_t ui32TimerNumber)
{
    if ((ui32TimerNumber == AM_HAL_INTERNAL_TIMER_NUM_B) || (ui32TimerNumber == AM_HAL_INTERNAL_TIMER_NUM_A))
    {
        //
        // Timer AM_HAL_INTERNAL_TIMER_NUM_B and AM_HAL_INTERNAL_TIMER_NUM_A are used by HAL.
        //
        return AM_HAL_STATUS_IN_USE;
    }

    AM_CRITICAL_BEGIN;

    //
    // Disable the timer.
    //
    TIMERn(ui32TimerNumber)->CTRL0_b.TMR0EN = 0;

    //
    // Clear the timer.
    //
    TIMERn(ui32TimerNumber)->CTRL0_b.TMR0CLR = 1;
    TIMERn(ui32TimerNumber)->CTRL0_b.TMR0CLR = 0;

    //
    // Enable the timer.
    //
    TIMERn(ui32TimerNumber)->CTRL0_b.TMR0EN = 1;

    AM_CRITICAL_END;

    return AM_HAL_STATUS_SUCCESS;
}

//
// Clear a single TIMER but don't start it.
//
uint32_t
am_hal_timer_clear_stop(uint32_t ui32TimerNumber)
{
    if ((ui32TimerNumber == AM_HAL_INTERNAL_TIMER_NUM_B) || (ui32TimerNumber == AM_HAL_INTERNAL_TIMER_NUM_A))
    {
        //
        // Timer AM_HAL_INTERNAL_TIMER_NUM_B and AM_HAL_INTERNAL_TIMER_NUM_A are used by HAL.
        //
        return AM_HAL_STATUS_IN_USE;
    }
    AM_CRITICAL_BEGIN;

    //
    // Disable the timer.
    //
    TIMERn(ui32TimerNumber)->CTRL0_b.TMR0EN = 0;

    //
    // Clear the timer.
    //
    TIMERn(ui32TimerNumber)->CTRL0_b.TMR0CLR = 1;

    TIMERn(ui32TimerNumber)->CTRL0_b.TMR0CLR = 0;

    AM_CRITICAL_END;

    return AM_HAL_STATUS_SUCCESS;
}

//
// Read the current value of a timer.
//
uint32_t
am_hal_timer_read(uint32_t ui32TimerNumber)
{
    uint32_t      ui32TimerAddr = (uint32_t)&TIMERn(ui32TimerNumber)->TIMER0;
    uint32_t      ui32TimerVals[3];

    if ((ui32TimerNumber == AM_HAL_INTERNAL_TIMER_NUM_B) || (ui32TimerNumber == AM_HAL_INTERNAL_TIMER_NUM_A))
    {
        //
        // Timer AM_HAL_INTERNAL_TIMER_NUM_B and AM_HAL_INTERNAL_TIMER_NUM_A are used by HAL.
        //
        return AM_HAL_STATUS_IN_USE;
    }

    //
    // Read the register into ui32TimerVals[].
    //
    am_hal_triple_read(ui32TimerAddr, ui32TimerVals);

    //
    // Now determine which of the three values is the correct value.
    // If the first 2 match, then the values are both correct and we're done.
    // Otherwise, the third value is taken to be the correct value.
    //
    if ( ui32TimerVals[0] == ui32TimerVals[1] )
    {
        //
        // If the first two values match, then neither one was a bad read.
        // We'll take this as the current time.
        //
        return ui32TimerVals[1];
    }
    else
    {
        return ui32TimerVals[2];
    }
}

//
// Configure timer pin output.
//
uint32_t
am_hal_timer_output_config(uint32_t ui32PadNum,
                           uint32_t eOutputType)
{
    uint32_t volatile *outcfg;
    uint32_t ui32OutcfgValue, ui32OutcfgMsk, ui32CfgShf, ui32OutcfgFnc;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    if ( (ui32PadNum >= AM_HAL_GPIO_MAX_PADS) || (eOutputType > AM_HAL_TIMER_OUTPUT_STIMER7) )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
#endif

    ui32CfgShf = ui32PadNum % 4 * 8;
    ui32OutcfgMsk = 0x7F << ui32CfgShf;
    ui32OutcfgFnc = eOutputType << ui32CfgShf;

    //
    // Begin critical section.
    //
    AM_CRITICAL_BEGIN

    outcfg = &(TIMER->OUTCFG0) + (ui32PadNum >> 2);
    ui32OutcfgValue = *outcfg;
    ui32OutcfgValue &= ~ui32OutcfgMsk;
    ui32OutcfgValue |=  ui32OutcfgFnc;
    *outcfg = ui32OutcfgValue;

    //
    // Done with critical section.
    //
    AM_CRITICAL_END

    return AM_HAL_STATUS_SUCCESS;
} // am_hal_timer_output_config()

//
// Set the COMPARE0 value for a single timer.
//
uint32_t
am_hal_timer_compare0_set(uint32_t ui32TimerNumber,
                          uint32_t ui32CompareValue)
{
    if ((ui32TimerNumber == AM_HAL_INTERNAL_TIMER_NUM_B) || (ui32TimerNumber == AM_HAL_INTERNAL_TIMER_NUM_A))
    {
        //
        // Timer AM_HAL_INTERNAL_TIMER_NUM_B and AM_HAL_INTERNAL_TIMER_NUM_A are used by HAL.
        //
        return AM_HAL_STATUS_IN_USE;
    }
    //
    // Apply the Compare0 value without disabling the timer.
    //
    TIMERn(ui32TimerNumber)->TMR0CMP0 = ui32CompareValue;

    return AM_HAL_STATUS_SUCCESS;
}

//
// Set the COMPARE1 value for a single timer.
//
uint32_t
am_hal_timer_compare1_set(uint32_t ui32TimerNumber,
                          uint32_t ui32CompareValue)
{
    if ((ui32TimerNumber == AM_HAL_INTERNAL_TIMER_NUM_B) || (ui32TimerNumber == AM_HAL_INTERNAL_TIMER_NUM_A))
    {
        //
        // Timer AM_HAL_INTERNAL_TIMER_NUM_B and AM_HAL_INTERNAL_TIMER_NUM_A are used by HAL.
        //
        return AM_HAL_STATUS_IN_USE;
    }
    //
    // Apply the Compare1 value without disabling the timer.
    //
    TIMERn(ui32TimerNumber)->TMR0CMP1 = ui32CompareValue;

    return AM_HAL_STATUS_SUCCESS;
}

//
// Enable timer interrupts.
//
uint32_t
am_hal_timer_interrupt_enable(uint32_t ui32InterruptMask)
{
    TIMER->INTEN |= ui32InterruptMask;

    return AM_HAL_STATUS_SUCCESS;
}

//
// Disable timer interrupts.
//
uint32_t
am_hal_timer_interrupt_disable(uint32_t ui32InterruptMask)
{
    TIMER->INTEN &= ~(ui32InterruptMask);

    return AM_HAL_STATUS_SUCCESS;
}

//
// Get the timer interrupt status.
//
uint32_t
am_hal_timer_interrupt_status_get(bool bEnabledOnly, uint32_t *pui32IntStatus)
{
    DIAG_SUPPRESS_VOLATILE_ORDER()

    if (bEnabledOnly)
    {
        *pui32IntStatus = TIMER->INTSTAT & TIMER->INTEN;
    }
    else
    {
        *pui32IntStatus = TIMER->INTSTAT;
    }

    return AM_HAL_STATUS_SUCCESS;

    DIAG_DEFAULT_VOLATILE_ORDER()
}

//
// Clear timer interrupts.
//
uint32_t
am_hal_timer_interrupt_clear(uint32_t ui32InterruptMask)
{
    TIMER->INTCLR = ui32InterruptMask;
    *(volatile uint32_t*)(&TIMER->INTSTAT);
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
