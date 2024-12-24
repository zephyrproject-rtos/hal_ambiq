//*****************************************************************************
//
//! @file am_hal_dsi.c
//!
//! @brief Hardware abstraction for the Display Serial Interface
//!
//! @addtogroup dsi DSI - Display Serial Interface
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
//! VDD18 control callback function
//
//*****************************************************************************
am_hal_dsi_external_vdd18_callback external_vdd18_callback;

//*****************************************************************************
//
// Register callback function
//
//*****************************************************************************
uint32_t
am_hal_dsi_register_external_vdd18_callback(const am_hal_dsi_external_vdd18_callback cb)
{
    if (cb != NULL)
    {
        external_vdd18_callback = cb;
    }
    else
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Configure DSI frequency and timing
//
//*****************************************************************************
uint32_t
am_hal_dsi_timing(uint32_t ui32FreqTrim)
{
    uint32_t ui32FreqTrimLsb6;
    ui32FreqTrimLsb6 = ui32FreqTrim & 0x3F;
    //
    // DPHY CLK/DATA timing parameters
    //
    if (ui32FreqTrimLsb6 == 0x10)
    {
        DSI->DPHYPARAM_b.HSPREP = 0x05;
        DSI->DPHYPARAM_b.HSZERO = 0x08;
        DSI->DPHYPARAM_b.HSTRAIL = 0x06;
        DSI->DPHYPARAM_b.HSEXIT = 0x14;
        DSI->CLKLANETIMPARM_b.HSPREP = 0x04;
        DSI->CLKLANETIMPARM_b.HSZERO = 0x18;
        DSI->CLKLANETIMPARM_b.HSTRAIL = 0x06;
        DSI->CLKLANETIMPARM_b.HSEXIT = 0x0F;
    }
    else if ((ui32FreqTrimLsb6 >= 0x0B) && (ui32FreqTrimLsb6 <= 0x0F))
    {
        DSI->DPHYPARAM_b.HSPREP = 0x04;
        DSI->DPHYPARAM_b.HSZERO = 0x08;
        DSI->DPHYPARAM_b.HSTRAIL = 0x05;
        DSI->DPHYPARAM_b.HSEXIT = 0x14;
        DSI->CLKLANETIMPARM_b.HSPREP = 0x04;
        DSI->CLKLANETIMPARM_b.HSZERO = 0x1C;
        DSI->CLKLANETIMPARM_b.HSTRAIL = 0x05;
        DSI->CLKLANETIMPARM_b.HSEXIT = 0x0F;
    }
    else if ((ui32FreqTrimLsb6 >= 0x08) && (ui32FreqTrimLsb6 <= 0x0A))
    {
        DSI->DPHYPARAM_b.HSPREP = 0x03;
        DSI->DPHYPARAM_b.HSZERO = 0x06;
        DSI->DPHYPARAM_b.HSTRAIL = 0x04;
        DSI->DPHYPARAM_b.HSEXIT = 0x0E;
        DSI->CLKLANETIMPARM_b.HSPREP = 0x03;
        DSI->CLKLANETIMPARM_b.HSZERO = 0x11;
        DSI->CLKLANETIMPARM_b.HSTRAIL = 0x04;
        DSI->CLKLANETIMPARM_b.HSEXIT = 0x0F;
    }
    else if ((ui32FreqTrimLsb6 >= 0x05) && (ui32FreqTrimLsb6 <= 0x07))
    {
        DSI->DPHYPARAM_b.HSPREP = 0x02;
        DSI->DPHYPARAM_b.HSZERO = 0x04;
        DSI->DPHYPARAM_b.HSTRAIL = 0x03;
        DSI->DPHYPARAM_b.HSEXIT = 0x0A;
        DSI->CLKLANETIMPARM_b.HSPREP = 0x02;
        DSI->CLKLANETIMPARM_b.HSZERO = 0x0B;
        DSI->CLKLANETIMPARM_b.HSTRAIL = 0x03;
        DSI->CLKLANETIMPARM_b.HSEXIT = 0x0F;
    }
    else
    {
        DSI->DPHYPARAM_b.HSPREP = 0x01;
        DSI->DPHYPARAM_b.HSZERO = 0x03;
        DSI->DPHYPARAM_b.HSTRAIL = 0x02;
        DSI->DPHYPARAM_b.HSEXIT = 0x0A;
        DSI->CLKLANETIMPARM_b.HSPREP = 0x01;
        DSI->CLKLANETIMPARM_b.HSZERO = 0x05;
        DSI->CLKLANETIMPARM_b.HSTRAIL = 0x02;
        DSI->CLKLANETIMPARM_b.HSEXIT = 0x0F;
    }
    //
    // PLL settings
    //
    DSI->AFETRIM1 &= _VAL2FLD(DSI_AFETRIM1_AFETRIM1, ~0x0000007F);
    DSI->AFETRIM1 |= _VAL2FLD(DSI_AFETRIM1_AFETRIM1, ui32FreqTrim);

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Initialize the DSI
//
//*****************************************************************************
uint32_t
am_hal_dsi_para_config(uint8_t ui8LanesNum, uint8_t ui8DBIBusWidth, uint32_t ui32FreqTrim, bool bSendUlpsPattern)
{

    //
    // ui32DSIFuncPrg_REG (DATA_WIDTH, RESERVED, FMT_VIDEO, CH_NO_CM, CH_NO_VM, DATA_LANE_CNT)
    //                     [15:13]     [12:10]   [9:7]      [6:5]     [4:3]     [2:0]
    //
    uint32_t ui32DSIFuncPrg = 0;

    //
    // check number of lanes parameters
    //
    switch (ui8LanesNum)
    {
        case 1:
            //
            // One data lane mode
            //
            ui32DSIFuncPrg |= _VAL2FLD(DSI_DSIFUNCPRG_DATALANES, DSI_DSIFUNCPRG_DATALANES_DATAL1);
        break;

        case 2:
            //
            // Two data lanes mode
            //
            ui32DSIFuncPrg |= _VAL2FLD(DSI_DSIFUNCPRG_DATALANES, DSI_DSIFUNCPRG_DATALANES_DATAL2);
        break;

        default:
            //
            // Other modes are not yet supported.
            //
            return AM_HAL_STATUS_OUT_OF_RANGE;
    }

    //
    // check DBI bus width parameter
    //
    switch (ui8DBIBusWidth)
    {
        case 8:
            //
            // The 8bit Interface of DBI-Type B output formats
            //
            ui32DSIFuncPrg |= _VAL2FLD(DSI_DSIFUNCPRG_REGNAME, DSI_DSIFUNCPRG_REGNAME_8BIT);
        break;

        case 9:
            //
            // The 9bit Interface of DBI-Type B output formats
            //
            ui32DSIFuncPrg |= _VAL2FLD(DSI_DSIFUNCPRG_REGNAME, DSI_DSIFUNCPRG_REGNAME_9BIT);
        break;

        case 16:
            //
            // There are two options for 16 bit interface of DBI-Type B output formats. We use option 1 as the default.
            //
// #### INTERNAL BEGIN ####
            //ui32DSIFuncPrg |= _VAL2FLD(DSI_DSIFUNCPRG_REGNAME, DSI_DSIFUNCPRG_REGNAME_16BIT0);
// #### INTERNAL END ####
            ui32DSIFuncPrg |= _VAL2FLD(DSI_DSIFUNCPRG_REGNAME, DSI_DSIFUNCPRG_REGNAME_16BIT1);

        break;

        default:
            //
            // Other modes are not yet supported.
            //
            return AM_HAL_STATUS_OUT_OF_RANGE;
    }

    DSI->RSTENBDFE = _VAL2FLD(DSI_RSTENBDFE_ENABLE, 0);

    //
    // write into DSI functional programming register
    //
    DSI->DSIFUNCPRG = ui32DSIFuncPrg;

// #### INTERNAL BEGIN ####
#ifdef APOLLO5_FPGA
    //
    // enable the internal cln_RxStopState
    //
    DSI->DSIFUNCPRG |= (1UL << 16);
#endif
// #### INTERNAL END ####

    //
    // write into HIGH SPEED RECEIVE TIMEOUT REGISTER
    //
    DSI->HSTXTIMEOUT = _VAL2FLD(DSI_HSTXTIMEOUT_MAXDURTOCNT, 0x00FFFFFF);

    //
    //write into LOW POWER RECEIVE TIMEOUT REGISTER
    //
    DSI->LPRXTO = _VAL2FLD(DSI_LPRXTO_TOCHKRVS, 0xFFFF);

    //
    // write into TURN AROUND TIMEOUT REGISTER
    //
    DSI->TURNARNDTO = _VAL2FLD(DSI_TURNARNDTO_TIMOUT, 0x1F);

    //
    // write into DEVICE RESET TIMER REGISTER
    //
    DSI->DEVICERESETTIMER = _VAL2FLD(DSI_DEVICERESETTIMER_TIMOUT, 0xFF);

    //
    // write into HIGH TO LOW SWITCH COUNT REGISTER
    //
    DSI->DATALANEHILOSWCNT = _VAL2FLD(DSI_DATALANEHILOSWCNT_DATALHLSWCNT, 0xFFFF);
    DSI->INITCNT = _VAL2FLD(DSI_INITCNT_MSTR, 0x7d0);
    DSI->LPBYTECLK = _VAL2FLD(DSI_LPBYTECLK_VALBYTECLK, 0x3);
    DSI->CLKEOT = _VAL2FLD(DSI_CLKEOT_CLOCK, 1);
    DSI->CLKEOT |= _VAL2FLD(DSI_CLKEOT_EOT, 1);
    am_hal_dsi_timing(ui32FreqTrim);
    DSI->AFETRIM2 = _VAL2FLD(DSI_AFETRIM2_AFETRIM2, 0x10000000);
    if (ui8LanesNum == 1)
    {
        DSI->AFETRIM2 |= _VAL2FLD(DSI_AFETRIM2_AFETRIM2, 0x00480000); // trim_2<22> and trim_2<19> need to be set for DSI TX in 1-lane configuration.
    }
    else if (ui8LanesNum == 2)
    {
        DSI->AFETRIM2 |= _VAL2FLD(DSI_AFETRIM2_AFETRIM2, 0x00400000); // clear power down bit for Data lane 1 to support DSI TX in 2lanes configuration.
    }
    DSI->AFETRIM1 |= _VAL2FLD(DSI_AFETRIM1_AFETRIM1, 0x00002000); // trim_1<13> needs to be set

    if (bSendUlpsPattern)
    {
        DSI->AFETRIM3 |= _VAL2FLD(DSI_AFETRIM3_AFETRIM3, 0x00030000);
    }
    DSI->AFETRIM0 |= _VAL2FLD(DSI_AFETRIM0_AFETRIM0, 0x00020000); // trim_0<17> needs to be set for B1 and later version.

    //
    // enable DSI TX and DPHY
    //
    DSI->RSTENBDFE = _VAL2FLD(DSI_RSTENBDFE_ENABLE, 1);
    DSI->DEVICEREADY |= _VAL2FLD(DSI_DEVICEREADY_READY, 1);

    //
    // Wait for DPHY init
    //
    if (AM_HAL_STATUS_SUCCESS != am_hal_delay_us_status_change(1000, (uint32_t)&DSI->INTRSTAT, DSI_INTRSTAT_INITDONE_Msk, DSI_INTRSTAT_INITDONE_Msk))
    {
       return AM_HAL_STATUS_TIMEOUT;
    }

    //
    // Check the low contention status bit
    //
    if ( DSI->INTRSTAT_b.LOWC == 1 )
    {
        //
        // write 1 to clear low contention
        //
        DSI->INTRSTAT_b.LOWC = 1;
    }

    if (bSendUlpsPattern)
    {
        //
        // ULPS Exit sequence
        //
        DSI->DEVICEREADY_b.ULPS = DSI_DEVICEREADY_ULPS_LOW_POWER;
        am_hal_delay_us(10);
        DSI->DEVICEREADY_b.ULPS = DSI_DEVICEREADY_ULPS_EXIT;
        DSI->AFETRIM3 &= _VAL2FLD(DSI_AFETRIM3_AFETRIM3, ~0x00030000);
        am_hal_delay_us(1010);
        DSI->DEVICEREADY_b.ULPS = DSI_DEVICEREADY_ULPS_This;
    }
    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Initialize power and clock of DSI
//
//*****************************************************************************
uint32_t
am_hal_dsi_init(void)
{
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_DISPCTRLCLK_ENABLE, NULL);
    am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_DISPPHY);
#warning "Temporary code for CRM module"
    CRM->APBDISPPHYCRM_b.APBDISPPHYCRMRSTN = 1;
    CRM->APBDISPPHYCRM_b.APBDISPPHYRSTN = 1;
    CRM->APBDISPPHYCRM_b.APBDISPPHYCLKEN = 1;

    CRM->DPHYPLLREFCRM_b.DPHYPLLREFCRMRSTN = 0;
    CRM->DPHYPLLREFCRM_b.DPHYPLLREFCLKEN = 0;
// #### INTERNAL BEGIN ####
#ifdef APOLLO5_FPGA
    // New build started with 20MHz clock in place of PLLFOUT2 as a choice for dphy_pll_ref_clk.
    // assign dphy_pll_ref_src_clk = { EXTREF_CLK_i, fpga_pll_20MHz, PLLPOSTDIV, RF_XTAL_48MHz_i, HFRC_48MHz_opcg };
    // assign dphy_pll_ref_src_clk = { EXTREF_CLK_i, PLLFOUT2, PLLPOSTDIV, RF_XTAL_48MHz_i, HFRC_48MHz_opcg };
    // More information please refer to jira FBRDSW-491
    CRM->DPHYPLLREFCRM_b.DPHYPLLREFCLKSEL = CRM_DPHYPLLREFCRM_DPHYPLLREFCLKSEL_PLLFOUT2;
    CRM->DPHYPLLREFCRM_b.DPHYPLLREFCLKDIV = 0;
#else
    CRM->DPHYPLLREFCRM_b.DPHYPLLREFCLKSEL = CRM_DPHYPLLREFCRM_DPHYPLLREFCLKSEL_HFRC_48MHz;
    CRM->DPHYPLLREFCRM_b.DPHYPLLREFCLKDIV = 3;
#endif
// #### INTERNAL END ####
    CRM->DPHYPLLREFCRM_b.DPHYPLLREFCRMRSTN = 1;
    CRM->DPHYPLLREFCRM_b.DPHYPLLREFCLKEN = 1;

    DSI->RSTENBDFE = _VAL2FLD(DSI_RSTENBDFE_ENABLE, 0);

    //
    // vdd18 enable
    //
    if (external_vdd18_callback)
    {
        external_vdd18_callback(true);
    }

    CRM->APBDISPCRM_b.APBDISPCRMRSTN = 1;
    CRM->APBDISPCRM_b.APBDISPRSTN = 1;
    CRM->APBDISPCRM_b.APBDISPCLKEN = 1;

    CRM->DISPCLKCRM_b.DISPCLKCRMRSTN = 0;
    CRM->DISPCLKCRM_b.DISPCLKCLKEN = 0;
    CRM->DISPCLKCRM_b.DISPCLKCLKSEL = CRM_DISPCLKCRM_DISPCLKCLKSEL_HFRC_192MHz;
    CRM->DISPCLKCRM_b.DISPCLKCLKDIV = 1;
    CRM->DISPCLKCRM_b.DISPCLKCRMRSTN = 1;
    CRM->DISPCLKCRM_b.DISPCLKCLKEN = 1;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Turn off power and clock of DSI
//
//*****************************************************************************
uint32_t
am_hal_dsi_deinit(bool bCheckStopState)
{
    //
    // Please don't check the stop state when DSI in ULPS
    //
    if (bCheckStopState)
    {
        //
        // Wait MIPI-DSI lane 0 return stop state before deinit it.
        //
        if (AM_HAL_STATUS_TIMEOUT == am_hal_dsi_wait_stop_state(0))
        {
            //
            // Return AM_HAL_STATUS_IN_USE if DSI is working.
            //
            return AM_HAL_STATUS_IN_USE;
        }
    }
#warning "Temporary code for CRM module"
    CRM->DPHYPLLREFCRM_b.DPHYPLLREFCRMRSTN = 0;
    CRM->DPHYPLLREFCRM_b.DPHYPLLREFCLKEN = 0;

    CRM->APBDISPPHYCRM_b.APBDISPPHYCRMRSTN = 0;
    CRM->APBDISPPHYCRM_b.APBDISPPHYRSTN = 0;
    CRM->APBDISPPHYCRM_b.APBDISPPHYCLKEN = 0;

    //
    // vdd18 disable
    //
    if (external_vdd18_callback)
    {
        external_vdd18_callback(false);
    }

    CRM->DISPCLKCRM_b.DISPCLKCRMRSTN = 0;
    CRM->DISPCLKCRM_b.DISPCLKCLKEN = 0;

    CRM->APBDISPCRM_b.APBDISPCRMRSTN = 0;
    CRM->APBDISPCRM_b.APBDISPRSTN = 0;
    CRM->APBDISPCRM_b.APBDISPCLKEN = 0;

    am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_DISPPHY);
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_DISPCTRLCLK_DISABLE, NULL);

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Enter ULPS mode
//
//*****************************************************************************
uint32_t
am_hal_dsi_ulps_entry(void)
{
    DSI->DEVICEREADY_b.ULPS = DSI_DEVICEREADY_ULPS_LOW_POWER;
    DSI->AFETRIM0 |= _VAL2FLD(DSI_AFETRIM0_AFETRIM0, 0x00000800); //  trim_0<11> needs to be set - contention detector disabled
    am_hal_delay_us(10);
    DSI->AFETRIM1 |= _VAL2FLD(DSI_AFETRIM1_AFETRIM1, 0x00000200); //  trim_1<9> needs to be set
    DSI->AFETRIM2 |= _VAL2FLD(DSI_AFETRIM2_AFETRIM2, 0x0000001C); //  trim_2<2>, trim_2<3> & trim_2<4> need to be set
    DSI->AFETRIM3 |= _VAL2FLD(DSI_AFETRIM3_AFETRIM3, 0x00038000); //  trim_3<15>, trim_3<16>, and trim_3<17> need to be set

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Exit ULPS mode
//
//*****************************************************************************
uint32_t
am_hal_dsi_ulps_exit(void)
{
    DSI->AFETRIM3 &= _VAL2FLD(DSI_AFETRIM3_AFETRIM3, ~0x00038000); //  trim_3<15>, trim_3<16>, and trim_3<17> need to be cleared
    DSI->AFETRIM2 &= _VAL2FLD(DSI_AFETRIM2_AFETRIM2, ~0x0000001C); //  trim_2<2>, trim_2<3> & trim_2<4> need to be cleared
    DSI->AFETRIM1 &= _VAL2FLD(DSI_AFETRIM1_AFETRIM1, ~0x00000200); //  trim_1<9> needs to be cleared
    DSI->DEVICEREADY_b.ULPS = DSI_DEVICEREADY_ULPS_EXIT;
    am_hal_delay_us(1010);
    DSI->DEVICEREADY_b.ULPS = DSI_DEVICEREADY_ULPS_This;
    DSI->AFETRIM0 &= _VAL2FLD(DSI_AFETRIM0_AFETRIM0, ~0x00000800); //  trim_0<11> needs to be cleared - contention detector enable

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// DSI napping
//
//*****************************************************************************
uint32_t
am_hal_dsi_napping(bool bSendUlpsPattern)
{
    if (bSendUlpsPattern)
    {
        am_hal_dsi_ulps_entry();
    }

    return am_hal_dsi_deinit(!bSendUlpsPattern);
}

//*****************************************************************************
//
// DSI wakeup
//
//*****************************************************************************
uint32_t
am_hal_dsi_wakeup(uint8_t ui8LanesNum, uint8_t ui8DBIBusWidth, uint32_t ui32FreqTrim, bool bSendUlpsPattern)
{
    am_hal_dsi_init();

    if ( am_hal_dsi_para_config(ui8LanesNum, ui8DBIBusWidth, ui32FreqTrim, bSendUlpsPattern) != 0 )
    {
        return AM_HAL_STATUS_FAIL;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// DSI set return packet size (bytes)
//
//*****************************************************************************
uint32_t
am_hal_dsi_set_return_size(uint8_t ui8DataLen, bool bHS)
{
    DSI->MAXRETPACSZE = _VAL2FLD(DSI_MAXRETPACSZE_HSLP, (uint32_t) (!bHS)) | _VAL2FLD(DSI_MAXRETPACSZE_COUNTVAL, (uint32_t)ui8DataLen);
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// DSI wait the stop state
//
//*****************************************************************************
uint32_t
am_hal_dsi_wait_stop_state(uint8_t ui8Lane)
{
// #### INTERNAL BEGIN ####
    //
    // This function can synchronize the stop state of DSI to the CPU. For more
    // Information, please refer to jira FBRDSW-351.
    //
// #### INTERNAL END ####
    //
    // For backward compatibility, we defined the argument ui8Lane equals 0,1 and 2, 
    // which represent data-lane 0, data-lane 1, and the clock lane, respectively.
    //
    if ( ui8Lane > 2 )
    {
        return AM_HAL_STATUS_OUT_OF_RANGE;
    }
    else if( ui8Lane == 2 )
    {
        ui8Lane = 0;
    }
    else
    {
        ui8Lane += 1;
    }
    //
    // Wait lane 0 or lane 1 in stop state.
    //

    if ( AM_HAL_STATUS_SUCCESS != am_hal_delay_us_status_change(1000, (uint32_t)&MCUCTRL->DISPSTATUS, 0x1U << ui8Lane, 0x1U << ui8Lane) )
    {
         return AM_HAL_STATUS_TIMEOUT;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
