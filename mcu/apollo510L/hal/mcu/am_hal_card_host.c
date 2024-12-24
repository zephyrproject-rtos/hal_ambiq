//*****************************************************************************
//
//! @file am_hal_card_host.c
//!
//! @brief Functions for interfacing with the SDHC or SPI SD/MMC/SDIO card host.
//!
//! @addtogroup card_host Card Host for SD/MMC/eMMC/SDIO
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
#include "am_util_stdio.h"
#include "am_util_debug.h"

//
// Internal MACROs
//
#define AM_HAL_CARD_HOST_DEBUG(fmt, ...) am_util_debug_printf("[CARD HOST] line %04d - "fmt, __LINE__, ##__VA_ARGS__)

//
// Internal functions
//

//*****************************************************************************
//
// Initialize the SDHC as a Host
// Sets up the power, clock, and capabilities
//
//*****************************************************************************
static uint32_t am_hal_sdhc_host_init(am_hal_card_host_t *pHost)
{
    //
    // Must initialize the SDHC state firstly
    //
    if (am_hal_sdhc_initialize(pHost->ui32Module, &pHost->pHandle) != AM_HAL_STATUS_SUCCESS)
    {
        return AM_HAL_STATUS_FAIL;
    }

    //
    // Must power the SDHC peripheral firstly
    //
    if (am_hal_sdhc_power_control(pHost->pHandle, AM_HAL_SYSCTRL_WAKE, false) != AM_HAL_STATUS_SUCCESS)
    {
        return AM_HAL_STATUS_FAIL;
    }

    //
    // Enable the clock to SDHC peripheral
    //
    if (am_hal_sdhc_enable(pHost->pHandle) != AM_HAL_STATUS_SUCCESS)
    {
        return AM_HAL_STATUS_FAIL;
    }

    //
    // Get the capabilities and version information
    // save them in the card host for later use
    //
    if (am_hal_sdhc_setup_host(pHost->pHandle, pHost) !=  AM_HAL_STATUS_SUCCESS)
    {
        return AM_HAL_STATUS_FAIL;
    }

    //
    // Set the 1.8V as the default bus voltage
    //
    if (am_hal_sdhc_set_bus_voltage(pHost->pHandle, pHost->eBusVoltage) != AM_HAL_STATUS_SUCCESS)
    {
        return AM_HAL_STATUS_FAIL;
    }

    //
    // Use the lowest frequency as the default clock speed
    //
    if (am_hal_sdhc_set_bus_clock(pHost->pHandle, pHost->ui32MinClock) != AM_HAL_STATUS_SUCCESS)
    {
        return AM_HAL_STATUS_FAIL;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Denitializes the SDHC as a Host
// Disables the SDHC and power capabilities
//
//*****************************************************************************
static uint32_t am_hal_sdhc_host_deinit(void *pHandle)
{
    if (am_hal_sdhc_disable(pHandle) != AM_HAL_STATUS_SUCCESS)
    {
        return AM_HAL_STATUS_FAIL;
    }

    if (am_hal_sdhc_power_control(pHandle, AM_HAL_SYSCTRL_NORMALSLEEP, false) != AM_HAL_STATUS_SUCCESS)
    {
        return AM_HAL_STATUS_FAIL;
    }

    if (am_hal_sdhc_deinitialize(pHandle) != AM_HAL_STATUS_SUCCESS)
    {
        return AM_HAL_STATUS_FAIL;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// A wrapper for the retries of the am_hal_sdhc_execute_cmd function
//
//*****************************************************************************
static uint32_t am_hal_sdhc_host_execute_cmd(void *pHandle, am_hal_card_cmd_t *pCmd, am_hal_card_cmd_data_t *pCmdData)
{
    uint32_t ui32Status = 0;
    uint8_t ui8Retries = 3;

    while (ui8Retries--)
    {
        ui32Status = am_hal_sdhc_execute_cmd(pHandle, pCmd, pCmdData);
        if ((ui32Status & 0xFFFF) == AM_HAL_STATUS_SUCCESS)
        {
            return ui32Status;
        }
    }

    AM_HAL_CARD_HOST_DEBUG("Failed to send the %d command after retry three times\n", pCmd->ui8Idx);

    return ui32Status;
}

//*****************************************************************************
//
// A wrapper for the am_hal_sdhc_power_control function
//
//*****************************************************************************
static uint32_t am_hal_card_host_power_control(void *pHandle, bool bOnOff)
{
  return am_hal_sdhc_power_control(pHandle, bOnOff ? AM_HAL_SYSCTRL_WAKE : AM_HAL_SYSCTRL_NORMALSLEEP, true);
}

static am_hal_card_host_ops_t g_sdhc_host_ops =
{
    .init = am_hal_sdhc_host_init,
    .deinit = am_hal_sdhc_host_deinit,
    .pwr_ctrl   = am_hal_card_host_power_control,
    .execute_cmd = am_hal_sdhc_host_execute_cmd,
    .set_bus_voltage = am_hal_sdhc_set_bus_voltage,
    .set_bus_width = am_hal_sdhc_set_bus_width,
    .set_bus_clock = am_hal_sdhc_set_bus_clock,
    .set_uhs_mode = am_hal_sdhc_set_uhs_mode,
    .set_txrx_delay = am_hal_sdhc_set_txrx_delay,
    .get_cd = am_hal_sdhc_get_cd,
    .get_wr_protect = am_hal_sdhc_get_wr_protect,
    .card_busy = am_hal_sdhc_card_busy,
};

static am_hal_card_host_t g_sdhc_card_host =
{
    .pHandle    = NULL,
    .bInited    = false,
    .eBusVoltage = AM_HAL_HOST_BUS_VOLTAGE_1_8,
    .ui32Module = 0x0,
    .ops        = &g_sdhc_host_ops,
};

static am_hal_card_host_t g_sdhc_card_host1 =
{
    .pHandle    = NULL,
    .bInited    = false,
    .eBusVoltage = AM_HAL_HOST_BUS_VOLTAGE_1_8,
    .ui32Module = 0x1,
    .ops        = &g_sdhc_host_ops,
};

// #### INTERNAL BEGIN ####

//
// SPI card host instance example
//
static uint32_t am_hal_spi_host_init(am_hal_card_host_t* pHost)
{
    return AM_HAL_STATUS_FAIL;
}

static uint32_t am_hal_spi_host_execute_cmd(void *pHandle, am_hal_card_cmd_t *pCmd, am_hal_card_cmd_data_t *pCmdData)
{
    return AM_HAL_STATUS_FAIL;
}

static am_hal_card_host_ops_t g_spi_host_ops =
{
    .init        = am_hal_spi_host_init,
    .execute_cmd = am_hal_spi_host_execute_cmd,
};

static am_hal_card_host_t g_spi_card_host =
{
    .pHandle    = NULL,
    .bInited    = false,
    .ui32Module = 0x0,
    .ops        = &g_spi_host_ops,
};

// #### INTERNAL END ####

//
// SD/MMC/SDIO host instances
//
static am_hal_card_host_t *g_CardHosts[AM_HAL_CARD_HOST_NUM] =
{
    &g_sdhc_card_host,
    &g_sdhc_card_host1,
    // #### INTERNAL BEGIN ####
    &g_spi_card_host,
    // #### INTERNAL END ####
};

//
// Public functions
//

//
// Initialize the card host instance and return it if it can be initialized
// successfully.
//
am_hal_card_host_t *am_hal_get_card_host(am_hal_host_inst_index_e eIndex, bool bReInit)
{
    am_hal_card_host_t *pHost;

    if (eIndex >= AM_HAL_CARD_HOST_NUM)
    {
        return NULL;
    }

    pHost = g_CardHosts[eIndex];

    if (pHost->bInited && !bReInit)
    {
        return pHost;
    }

    if (pHost->bInited)
    {
        pHost->ops->deinit(pHost->pHandle);
    }

    if (pHost->ops->init(pHost) != AM_HAL_STATUS_SUCCESS)
    {
        return NULL;
    }

    pHost->bInited = true;

    return pHost;
}

//
// Set the default transfer mode of the SDHC host controller. if command's transfer type is not
// specified, it will be used.
//
void am_hal_card_host_set_xfer_mode(am_hal_card_host_t *pHost, am_hal_host_xfer_mode_e eXferMode)
{
    pHost->eXferMode = eXferMode;
}

void am_hal_card_host_set_txrx_delay(am_hal_card_host_t *pHost, uint8_t ui8TxRxDelays[2])
{
    if (pHost != NULL && pHost->bInited && pHost->ops->set_txrx_delay)
    {
        pHost->ops->set_txrx_delay(pHost->pHandle, ui8TxRxDelays);
    }
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
