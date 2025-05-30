//*****************************************************************************
//
//! @file am_hal_rtc.c
//!
//! @brief Functions for interfacing with the Real-Time Clock (RTC).
//!
//! @addtogroup rtc4 RTC - Real-Time Clock
//! @ingroup apollo510_hal
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2025, Ambiq Micro, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision release_sdk5p0p0-5f68a8286b of the AmbiqSuite Development Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "am_mcu_apollo.h"

//*****************************************************************************
//
// Local defines
//
//*****************************************************************************
//! A safe delay for RTC reg read
#define RTC_CDC_WA_DELAY_US     10  // if 32K, 10us = 33%; if LFRC 10us = 1%

//*****************************************************************************
//
//! These macros are used inside the #am_hal_rtc_alarm_interval_set function
//! when 10ms and 100ms repeated alarm intervals are desired.
//
//*****************************************************************************
#define AM_HAL_RTC_ALM100_DEFAULT           0x00
#define AM_HAL_RTC_ALM100_10TH              0xF0
#define AM_HAL_RTC_ALM100_100TH             0xFF

//*****************************************************************************
//
//! Converts a Binary Coded Decimal (BCD) byte to its Decimal form.
//
//*****************************************************************************
static uint8_t
bcd_to_dec(uint8_t ui8BCDByte)
{
    return (((ui8BCDByte & 0xF0) >> 4) * 10) + (ui8BCDByte & 0x0F);
}

//*****************************************************************************
//
//! Converts a Decimal byte to its Binary Coded Decimal (BCD) form.
//
//*****************************************************************************
static uint8_t
dec_to_bcd(uint8_t ui8DecimalByte)
{
    return (((ui8DecimalByte / 10) << 4) | (ui8DecimalByte % 10));
}

//*****************************************************************************
//
//! @brief Validates the user defined date & time
//!
//! @param *pTime - A pointer to the time structure.
//!
//! Validates the input required to have the time set to a specific period
//!
//! @return returns true if the pTime structure input values are with in range
//
//*****************************************************************************
static bool
time_input_validate(am_hal_rtc_time_t *pTime)
{

    bool bValidateStatus = true;

    if (pTime->ui32Hundredths > 99)
    {
        bValidateStatus = false;
        return bValidateStatus;
    }

    if (pTime->ui32Second >= 60)
    {
        bValidateStatus = false;
        return bValidateStatus;
    }

    if (pTime->ui32Minute >= 60)
    {
        bValidateStatus = false;
        return bValidateStatus;
    }

    if (pTime->ui32Hour >= 24)
    {
        bValidateStatus = false;
        return bValidateStatus;
    }

    if (pTime->ui32DayOfMonth < 1 || pTime->ui32DayOfMonth > 31)
    {
        bValidateStatus = false;
        return bValidateStatus;
    }

    if (pTime->ui32Month < 1 || pTime->ui32Month > 12)
    {
        bValidateStatus = false;
        return bValidateStatus;
    }

    if (pTime->ui32Year > 100)
    {
        bValidateStatus = false;
        return bValidateStatus;
    }

    if ( pTime->ui32Weekday > 6)
    {
        bValidateStatus = false;
        return bValidateStatus;
    }

    return bValidateStatus;

}

//*****************************************************************************
//
// Configures the RTC
//
// Configures the oscillator and 12/24hr settings for the RTC. See the \e
// am_hal_rtc_config_t structure for more information.
//
//*****************************************************************************
uint32_t
am_hal_rtc_config(const am_hal_rtc_config_t *psConfig)
{
    uint32_t ui32Oscillator;

    //
    // Determine the correct oscillator setting.
    //
    switch (psConfig->eOscillator)
    {
        case AM_HAL_RTC_OSC_XT:
            ui32Oscillator = CLKGEN_OCTRL_OSEL_XT_512Hz;
            break;

        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Only write the oscillator register if we need to.
    //
    if (CLKGEN->OCTRL_b.OSEL != ui32Oscillator)
    {
        CLKGEN->OCTRL_b.OSEL = ui32Oscillator;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Enable/Start the RTC oscillator.
//
// Starts the RTC oscillator.
//
//*****************************************************************************
uint32_t
am_hal_rtc_osc_enable(void)
{
    //
    // Start the RTC Oscillator.
    //
    RTC->RTCCTL_b.RSTOP = RTC_RTCCTL_RSTOP_RUN;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Disable/Stop the RTC oscillator.
//
// Stops the RTC oscillator.
//
//*****************************************************************************
uint32_t
am_hal_rtc_osc_disable(void)
{
    //
    // Stop the RTC Oscillator.
    //
    RTC->RTCCTL_b.RSTOP = RTC_RTCCTL_RSTOP_STOP;

    return AM_HAL_STATUS_SUCCESS;
}

void
am_hal_rtc_counter_clear(void)
{
    //
    // Enable writing to the counters.
    //
    RTC->RTCCTL_b.WRTC = RTC_RTCCTL_WRTC_EN;

    //
    // Write the RTCLOW register.
    //
    RTC->CTRLOW = 0x0;

    //
    // Write the RTCUP register.
    //
    RTC->CTRUP = 0x0;

    //
    // Disable writing to the counters.
    //
    RTC->RTCCTL_b.WRTC = RTC_RTCCTL_WRTC_DIS;
}

//*****************************************************************************
//
// Set the Real Time Clock counter registers.
//
// Sets the RTC counter registers to the supplied values.
//
//*****************************************************************************
uint32_t
am_hal_rtc_time_set(am_hal_rtc_time_t *pTime)
{
    //
    // Validate the user defined date and time setting
    //
    if (!time_input_validate(pTime))
    {
        return AM_HAL_STATUS_FAIL;
    }

    //
    // Enable writing to the counters.
    //
    RTC->RTCCTL_b.WRTC = RTC_RTCCTL_WRTC_EN;

    //
    // Write the RTCLOW register.
    //
    RTC->CTRLOW =
        _VAL2FLD(RTC_CTRLOW_CTRHR,  dec_to_bcd(pTime->ui32Hour))         |
        _VAL2FLD(RTC_CTRLOW_CTRMIN, dec_to_bcd(pTime->ui32Minute))       |
        _VAL2FLD(RTC_CTRLOW_CTRSEC, dec_to_bcd(pTime->ui32Second))       |
        _VAL2FLD(RTC_CTRLOW_CTR100, dec_to_bcd(pTime->ui32Hundredths));

    //
    // Write the RTCUP register.
    //
    RTC->CTRUP =
        _VAL2FLD(RTC_CTRUP_CB,      (pTime->ui32CenturyBit))                |
        _VAL2FLD(RTC_CTRUP_CTRWKDY, (pTime->ui32Weekday))                |
        _VAL2FLD(RTC_CTRUP_CTRYR,   dec_to_bcd((pTime->ui32Year)))       |
        _VAL2FLD(RTC_CTRUP_CTRMO,   dec_to_bcd((pTime->ui32Month)))      |
        _VAL2FLD(RTC_CTRUP_CTRDATE, dec_to_bcd((pTime->ui32DayOfMonth)));

    //
    // Disable writing to the counters.
    //
    RTC->RTCCTL_b.WRTC = RTC_RTCCTL_WRTC_DIS;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Poll the RTC source clock's edge and wait before register read
//
//*****************************************************************************
static inline void
poll_rtc_clksrc_edge_and_wait()
{
    uint32_t ui32PreTimerCnt = 0;

#if RTC_CDC_WA_USE_APP_STMR
    //
    // capture the clock edge
    //
    ui32PreTimerCnt = am_hal_stimer_counter_get();
    while (ui32PreTimerCnt == am_hal_stimer_counter_get());
#else
    //
    // Setup the timer with same clock source as RTL (LFRC or XT) with the highest frequency option
    //
    TIMERn(AM_HAL_INTERNAL_TIMER_NUM_B)->CTRL0_b.TMR0EN = 0;
    TIMERn(AM_HAL_INTERNAL_TIMER_NUM_B)->CTRL0 = _VAL2FLD(TIMER_CTRL0_TMR0CLK, \
                                              CLKGEN->OCTRL & CLKGEN_OCTRL_OSEL_Msk ?  \
                                              TIMER_CTRL0_TMR0CLK_LFRC : TIMER_CTRL0_TMR0CLK_XT)        |
                                      _VAL2FLD(TIMER_CTRL0_TMR0FN,      AM_HAL_TIMER_FN_EDGE)           |
                                      _VAL2FLD(TIMER_CTRL0_TMR0POL1,    false)                          |
                                      _VAL2FLD(TIMER_CTRL0_TMR0POL0,    false)                          |
                                      _VAL2FLD(TIMER_CTRL0_TMR0TMODE,   AM_HAL_TIMER_TRIGGER_DIS)       |
                                      _VAL2FLD(TIMER_CTRL0_TMR0LMT,     0)                              |
                                      _VAL2FLD(TIMER_CTRL0_TMR0EN, 0);
    TIMERn(AM_HAL_INTERNAL_TIMER_NUM_B)->TMR0CMP0 = 0xFFFFFFFF;
    TIMERn(AM_HAL_INTERNAL_TIMER_NUM_B)->TMR0CMP1 = 0xFFFFFFFF;

    //
    // Toggle the clear bit (required by the hardware), and then enable the timer.
    //
    TIMER->GLOBEN |= 1 <<  AM_HAL_INTERNAL_TIMER_NUM_B;
    TIMERn(AM_HAL_INTERNAL_TIMER_NUM_B)->CTRL0_b.TMR0CLR = 1;
    TIMERn(AM_HAL_INTERNAL_TIMER_NUM_B)->CTRL0_b.TMR0CLR = 0;
    TIMERn(AM_HAL_INTERNAL_TIMER_NUM_B)->CTRL0_b.TMR0EN = 1;

    //
    // capture the clock edge
    //
    ui32PreTimerCnt = TIMERn(AM_HAL_INTERNAL_TIMER_NUM_B)->TIMER0;
    while (ui32PreTimerCnt == TIMERn(AM_HAL_INTERNAL_TIMER_NUM_B)->TIMER0);

    //
    // Disable the timer
    //
    TIMERn(AM_HAL_INTERNAL_TIMER_NUM_B)->CTRL0_b.TMR0EN = 0;
#endif

    //
    // A safe delay for RTC reg read
    //
    am_hal_delay_us(RTC_CDC_WA_DELAY_US);
}

//*****************************************************************************
//
// Get the Real Time Clock current time.
//
// Gets the RTC's current time
//
//*****************************************************************************
uint32_t
am_hal_rtc_time_get(am_hal_rtc_time_t *pTime)
{
    uint32_t ui32RTCLow, ui32RTCUp, ui32Value;

    AM_CRITICAL_BEGIN

    //
    // poll the RTC clock source edge and wait before read the registers
    //
    poll_rtc_clksrc_edge_and_wait();

    //
    // Read the upper and lower RTC registers.
    //
    ui32RTCLow = RTC->CTRLOW;
    ui32RTCUp  = RTC->CTRUP;

    AM_CRITICAL_END

    //
    // Check for errors. If we didn't successfully read the time, return with
    // an error.
    //
    pTime->ui32ReadError = _FLD2VAL(RTC_CTRUP_CTERR, ui32RTCUp);

    if (pTime->ui32ReadError)
    {
        return AM_HAL_STATUS_FAIL;
    }

    //
    // Break out the lower word.
    //
    ui32Value               = _FLD2VAL(RTC_CTRLOW_CTRHR, ui32RTCLow);
    pTime->ui32Hour         = bcd_to_dec(ui32Value);

    ui32Value               = _FLD2VAL(RTC_CTRLOW_CTRMIN, ui32RTCLow);
    pTime->ui32Minute       = bcd_to_dec(ui32Value);

    ui32Value               = _FLD2VAL(RTC_CTRLOW_CTRSEC, ui32RTCLow);
    pTime->ui32Second       = bcd_to_dec(ui32Value);

    ui32Value               = _FLD2VAL(RTC_CTRLOW_CTR100, ui32RTCLow);
    pTime->ui32Hundredths   = bcd_to_dec(ui32Value);

    //
    // Break out the upper word.
    //

    pTime->ui32CenturyBit      = _FLD2VAL(RTC_CTRUP_CB, ui32RTCUp);

    ui32Value               = _FLD2VAL(RTC_CTRUP_CTRWKDY, ui32RTCUp);
    pTime->ui32Weekday      = bcd_to_dec(ui32Value);

    ui32Value               = _FLD2VAL(RTC_CTRUP_CTRYR, ui32RTCUp);
    pTime->ui32Year         = bcd_to_dec(ui32Value);

    ui32Value               = _FLD2VAL(RTC_CTRUP_CTRMO, ui32RTCUp);
    pTime->ui32Month        = bcd_to_dec(ui32Value);

    ui32Value               = _FLD2VAL(RTC_CTRUP_CTRDATE, ui32RTCUp);
    pTime->ui32DayOfMonth   = bcd_to_dec(ui32Value);

    //
    // Was there a read error?
    //
    if (pTime->ui32ReadError)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

//*****************************************************************************
//
// Selects the clock source for the RTC.
//
// This function selects the clock source for the RTC.
//
// Valid values for ui32OSC are:
//
//     AM_HAL_RTC_OSC_LFRC
//     AM_HAL_RTC_OSC_XT
//
// @note After selection of the RTC oscillator, a 2 second delay occurs before
// the new setting is reflected in status. Therefore the CLKGEN.STATUS.OMODE
// bit will not reflect the new status until after the 2s wait period.
//
//
//*****************************************************************************
void
am_hal_rtc_osc_select(uint32_t ui32OSC)
{
    if ( ui32OSC == AM_HAL_RTC_OSC_XT )
    {
        //
        // Clear bit to 0 for XTAL
        //
        CLKGEN->OCTRL &= ~CLKGEN_OCTRL_OSEL_Msk;
    }
    else if ( ui32OSC == AM_HAL_RTC_OSC_LFRC )
    {
        //
        // Set bit to 1 for LFRC
        //
        CLKGEN->OCTRL |= CLKGEN_OCTRL_OSEL_Msk;
    }
}

//*****************************************************************************
//
// Sets the alarm repeat interval.
//
// Valid values for ui32RepeatInterval:
//
//     AM_HAL_RTC_ALM_RPT_DIS
//     AM_HAL_RTC_ALM_RPT_YR
//     AM_HAL_RTC_ALM_RPT_MTH
//     AM_HAL_RTC_ALM_RPT_WK
//     AM_HAL_RTC_ALM_RPT_DAY
//     AM_HAL_RTC_ALM_RPT_HR
//     AM_HAL_RTC_ALM_RPT_MIN
//     AM_HAL_RTC_ALM_RPT_SEC
//     AM_HAL_RTC_ALM_RPT_10TH
//     AM_HAL_RTC_ALM_RPT_100TH
//
//*****************************************************************************
void
am_hal_rtc_alarm_interval_set(uint32_t ui32RepeatInterval)
{
    uint32_t ui32RptInt, ui32Alm100, ui32Value;

    switch ( ui32RepeatInterval )
    {
        //
        // If repeat every 10th set RPT and ALM100 field accordingly
        //
        case AM_HAL_RTC_ALM_RPT_10TH:
            ui32RptInt = AM_HAL_RTC_ALM_RPT_SEC;
            ui32Alm100 = AM_HAL_RTC_ALM100_10TH;
            break;
        //
        // If repeat every 100th set RPT and ALM100 field accordingly
        //
        case AM_HAL_RTC_ALM_RPT_100TH:
            ui32RptInt = AM_HAL_RTC_ALM_RPT_SEC;
            ui32Alm100 = AM_HAL_RTC_ALM100_100TH;
            break;

        //
        // Otherwise set RPT as value passed.  ALM100 values need to be 0xnn
        // in this setting where n = 0-9.
        //
        default:
            //
            // Get the current value of the ALM100 field.
            //
            ui32Value = RTC->ALMLOW_b.ALM100;

            //
            // If ALM100 was previous EVERY_10TH or EVERY_100TH reset to zero
            // otherwise keep previous setting.
            //
            ui32Alm100 = ui32Value >= 0xF0 ? 0 : ui32Value;

            //
            // Set RPT value to value passed.
            //
            ui32RptInt = ui32RepeatInterval;
            break;
    }

    //
    // Write the interval to the register.
    //
    RTC->RTCCTL_b.RPT = ui32RptInt;

    //
    // Write the Alarm 100 bits in the ALM100 register.
    //
    RTC->ALMLOW_b.ALM100 = ui32Alm100;
}

//*****************************************************************************
//
// Sets the RTC's Alarm.
//
// Set the Real Time Clock Alarm Parameters.
//
//*****************************************************************************
uint32_t
am_hal_rtc_alarm_set(am_hal_rtc_time_t *pTime,
                     am_hal_rtc_alarm_repeat_e eRepeatInterval)
{
    uint8_t ui8Value = 0;

    //
    // Write the interval to the register.
    //
    RTC->RTCCTL_b.RPT = eRepeatInterval > 0x7 ? 0x7 : eRepeatInterval;

    //
    // Check if the interval is 10th or every 100th and track it in ui8Value.
    //
    if (eRepeatInterval == AM_HAL_RTC_ALM_RPT_10TH)
    {
        ui8Value = 0xF0;
    }
    else if (eRepeatInterval == AM_HAL_RTC_ALM_RPT_100TH)
    {
        ui8Value = 0xFF;
    }

    //
    // Write the ALMUP register.
    //
    RTC->ALMUP =
        _VAL2FLD(RTC_ALMUP_ALMWKDY, (pTime->ui32Weekday))                   |
        _VAL2FLD(RTC_ALMUP_ALMMO,   dec_to_bcd((pTime->ui32Month)))         |
        _VAL2FLD(RTC_ALMUP_ALMDATE, dec_to_bcd((pTime->ui32DayOfMonth)));

    //
    // Write the ALMLOW register.
    //
    RTC->ALMLOW =
        _VAL2FLD(RTC_ALMLOW_ALMHR,  dec_to_bcd(pTime->ui32Hour))            |
        _VAL2FLD(RTC_ALMLOW_ALMMIN, dec_to_bcd(pTime->ui32Minute))          |
        _VAL2FLD(RTC_ALMLOW_ALMSEC, dec_to_bcd(pTime->ui32Second))          |
        _VAL2FLD(RTC_ALMLOW_ALM100, dec_to_bcd(pTime->ui32Hundredths) | ui8Value);

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Get the Real Time Clock Alarm Parameters
//
// Retrieves the RTC's current alarm time and repeat interval settings.
//
//*****************************************************************************
uint32_t
am_hal_rtc_alarm_get(am_hal_rtc_time_t *pTime,
                     am_hal_rtc_alarm_repeat_e *peRepeatInterval)
{
    uint32_t ui32ALMLow, ui32ALMUp, ui32Value;
    uint32_t ui32RegValue;

    //
    // Read the upper and lower RTC registers.
    //
    ui32ALMLow = RTC->ALMLOW;
    ui32ALMUp  = RTC->ALMUP;

    if (pTime != NULL)
    {
        //
        // Break out the lower word.
        //
        ui32Value = ((ui32ALMLow & RTC_ALMLOW_ALMHR_Msk) >> RTC_ALMLOW_ALMHR_Pos);
        pTime->ui32Hour = bcd_to_dec(ui32Value);

        ui32Value = ((ui32ALMLow & RTC_ALMLOW_ALMMIN_Msk) >> RTC_ALMLOW_ALMMIN_Pos);
        pTime->ui32Minute = bcd_to_dec(ui32Value);

        ui32Value = ((ui32ALMLow & RTC_ALMLOW_ALMSEC_Msk) >> RTC_ALMLOW_ALMSEC_Pos);
        pTime->ui32Second = bcd_to_dec(ui32Value);

        ui32Value = ((ui32ALMLow & RTC_ALMLOW_ALM100_Msk) >> RTC_ALMLOW_ALM100_Pos);
        pTime->ui32Hundredths = bcd_to_dec(ui32Value);

        //
        // Break out the upper word.
        //
        pTime->ui32CenturyBit = 0;
        pTime->ui32ReadError = 0;

        ui32Value = ((ui32ALMUp & RTC_ALMUP_ALMWKDY_Msk) >> RTC_ALMUP_ALMWKDY_Pos);
        pTime->ui32Weekday = bcd_to_dec(ui32Value);

        pTime->ui32Year = 0;

        ui32Value = ((ui32ALMUp & RTC_ALMUP_ALMMO_Msk) >> RTC_ALMUP_ALMMO_Pos);
        pTime->ui32Month = bcd_to_dec(ui32Value);

        ui32Value = ((ui32ALMUp & RTC_ALMUP_ALMDATE_Msk) >> RTC_ALMUP_ALMDATE_Pos);
        pTime->ui32DayOfMonth = bcd_to_dec(ui32Value);
    }

    if (peRepeatInterval != NULL)
    {
        //
        // Read the repeat interval back from the register.
        //
        ui32RegValue = RTC->RTCCTL_b.RPT;

        //
        // We have to do some special handling here, since the repeate behavior
        // isn't entirely controlled by a single register.
        //
        if (ui32RegValue == RTC_RTCCTL_RPT_SEC)
        {
            ui32RegValue = RTC->ALMLOW_b.ALM100;

            if (ui32RegValue == 0xF0)
            {
                ui32RegValue = AM_HAL_RTC_ALM_RPT_10TH;
            }
            else if (ui32RegValue == 0xFF)
            {
                ui32RegValue = AM_HAL_RTC_ALM_RPT_100TH;
            }
            else
            {
                ui32RegValue = RTC_RTCCTL_RPT_SEC;
            }
        }

        *peRepeatInterval = (am_hal_rtc_alarm_repeat_e)ui32RegValue;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Enable selected RTC interrupts.
//
// Enables the RTC interrupts. \e ui32InterruptMask should be set to a
// logical OR of one or more of the following:
//
//     AM_HAL_RTC_INT_ALM
//
//*****************************************************************************
uint32_t
am_hal_rtc_interrupt_enable(uint32_t ui32InterruptMask)
{
    RTC->INTEN |= ui32InterruptMask;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Return the enabled RTC interrupts.
//
// Returns the enabled RTC interrupts. \e pui32InterruptMask will be set to a
// logical OR of one or more of the following:
//
//     AM_HAL_RTC_INT_ALM
//
//
//*****************************************************************************
uint32_t
am_hal_rtc_interrupt_enable_get(uint32_t *pui32InterruptMask)
{
    *pui32InterruptMask = RTC->INTEN;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Disable selected RTC interrupts.
//
// Disables the RTC interrupts.
//
// ui32Interrupt should be an OR of the following:
//
//     AM_HAL_RTC_INT_ALM
//
//*****************************************************************************
uint32_t
am_hal_rtc_interrupt_disable(uint32_t ui32InterruptMask)
{
    RTC->INTEN &= ~ui32InterruptMask;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Clear selected RTC interrupts.
//
// Clears the RTC interrupts.
//
// ui32Interrupt should be an OR of the following:
//
//     AM_HAL_RTC_INT_ALM
//
//*****************************************************************************
uint32_t
am_hal_rtc_interrupt_clear(uint32_t ui32InterruptMask)
{
    RTC->INTCLR = ui32InterruptMask;
    *(volatile uint32_t*)(&RTC->INTSTAT);
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Sets the selected RTC interrupts.
//
// Sets the RTC interrupts causing them to immediately trigger.
//
// ui32Interrupt should be an OR of the following:
//
//     AM_HAL_RTC_INT_ALM
//
//*****************************************************************************
uint32_t
am_hal_rtc_interrupt_set(uint32_t ui32InterruptMask)
{
    RTC->INTSET = ui32InterruptMask;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Returns the RTC interrupt status.
//
// This function will write the current RTC interrupt status to \e
// pui32InterruptMask. The interrupt status value will be the logical OR of
// one or more of the following:
//
//     AM_HAL_RTC_INT_ALM
//
//*****************************************************************************
uint32_t
am_hal_rtc_interrupt_status_get(bool bEnabledOnly,
                                uint32_t *pui32InterruptMask)
{
    //
    // Get the interrupt status.
    //
    if ( bEnabledOnly )
    {
        uint32_t ui32RetVal;
        ui32RetVal = RTC->INTSTAT;
        ui32RetVal &= RTC->INTEN;
        *pui32InterruptMask = ui32RetVal & (AM_HAL_RTC_INT_ALM);
    }
    else
    {
        *pui32InterruptMask = RTC->INTSTAT & (AM_HAL_RTC_INT_ALM);
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
