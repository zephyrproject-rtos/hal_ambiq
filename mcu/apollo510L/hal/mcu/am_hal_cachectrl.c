//*****************************************************************************
//
//! @file am_hal_cachectrl.c
//!
//! @brief Functions for interfacing with the CACHE controller.
//!
//! @addtogroup cachectrl4 CACHE - Cache Control
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
// Default configurations definitions
//
//*****************************************************************************
am_hal_cachectrl_prefetch_t    g_PrefetchConfig =
{
    .ui8MaxOutTransactions          = 6,
    .ui8MaxLookAhead                = 6,
    .ui8MinLookAhead                = 4
};

//Enable the Instruction cache for operation.
uint32_t
am_hal_cachectrl_icache_enable(void)
{
    //
    // Check if cache was powered on, if not, return error.
    //
    if (PWRMODCTL->CPDLPSTATE & PWRMODCTL_CPDLPSTATE_RLPSTATE_Msk)
    {
        return AM_HAL_STATUS_FAIL;
    }
    //
    // Call CMSIS function to enable I-Cache.
    //
    SCB_EnableICache();

    return AM_HAL_STATUS_SUCCESS;
}

//Disable the Instruction cache.
uint32_t
am_hal_cachectrl_icache_disable(void)
{
    //
    // Check if cache was powered on, if not, return error.
    //
    if (PWRMODCTL->CPDLPSTATE & PWRMODCTL_CPDLPSTATE_RLPSTATE_Msk)
    {
        return AM_HAL_STATUS_FAIL;
    }
    //
    // Call CMSIS function to disable the I-Cache.
    //
    SCB_DisableICache();
    return AM_HAL_STATUS_SUCCESS;
}

//Enable the Data cache for operation.
uint32_t
am_hal_cachectrl_dcache_enable(bool bClean)
{
    //
    // Check if cache was powered on, if not, return error.
    //
    if (PWRMODCTL->CPDLPSTATE & PWRMODCTL_CPDLPSTATE_RLPSTATE_Msk)
    {
        return AM_HAL_STATUS_FAIL;
    }
    //
    //  Set the default CM55 Pre-Fetch Control for RevB.
    //
    MEMSYSCTL->PFCR = _VAL2FLD(MEMSYSCTL_PFCR_MAX_OS, g_PrefetchConfig.ui8MaxOutTransactions)    |
                      _VAL2FLD(MEMSYSCTL_PFCR_MAX_LA, g_PrefetchConfig.ui8MaxLookAhead)          |
                      _VAL2FLD(MEMSYSCTL_PFCR_MIN_LA, g_PrefetchConfig.ui8MinLookAhead)          |
                      _VAL2FLD(MEMSYSCTL_PFCR_ENABLE, 1);

    //
    // Call CMSIS function to enable the D-Cache.
    //
    SCB_EnableDCache();

    //
    // Check whether to clean the D-Cache or not.
    //
    if (bClean)
    {
        SCB_CleanDCache();
    }

    return AM_HAL_STATUS_SUCCESS;
}

//Disable the Data cache.
uint32_t
am_hal_cachectrl_dcache_disable(void)
{
    //
    // Check if cache was powered on, if not, return error.
    //
    if (PWRMODCTL->CPDLPSTATE & PWRMODCTL_CPDLPSTATE_RLPSTATE_Msk)
    {
        return AM_HAL_STATUS_FAIL;
    }
    //
    // Call CMSIS function to disable the D-Cache.
    //
    SCB_DisableDCache();
    return AM_HAL_STATUS_SUCCESS;
}

//Invalidate the Insruction cache.
uint32_t
am_hal_cachectrl_icache_invalidate(am_hal_cachectrl_range_t *pRange)
{
    //
    // If I-Cache was not enabled, only do DSB and ISB operations.
    //
    if ((SCB->CCR & (uint32_t)SCB_CCR_IC_Msk) == 0)
    {
        __DSB();
        __ISB();

        return AM_HAL_STATUS_SUCCESS;
    }
    //
    // If no address range was provided, invalidate the entire icache.
    //
    if (NULL == pRange)
    {
        SCB_InvalidateICache();
    }
    else
    {
        SCB_InvalidateICache_by_Addr((void *)pRange->ui32StartAddr, pRange->ui32Size);
    }

    return AM_HAL_STATUS_SUCCESS;
}

//Invalidate the Data cache.
uint32_t
am_hal_cachectrl_dcache_invalidate(am_hal_cachectrl_range_t *pRange,
                                   bool bClean)
{
    //
    // If D-Cache was not enabled, only do DSB and ISB operations.
    //
    if ((SCB->CCR & (uint32_t)SCB_CCR_DC_Msk) == 0)
    {
        __DSB();
        __ISB();

        return AM_HAL_STATUS_SUCCESS;
    }
    //
    // If the address range is NULL then invalidate the entire D-Cache.
    //
    if (NULL == pRange)
    {
        if (bClean)
        {
            SCB_CleanInvalidateDCache();
        }
        else
        {
            SCB_InvalidateDCache();
        }
    }
    else
    {
        if (bClean)
        {
            SCB_CleanInvalidateDCache_by_Addr((void *)pRange->ui32StartAddr, pRange->ui32Size);
        }
        else
        {
            SCB_InvalidateDCache_by_Addr((void *)pRange->ui32StartAddr, pRange->ui32Size);
        }
    }

    return AM_HAL_STATUS_SUCCESS;
}

// CACHECTRL control function for configuration changes.
uint32_t
am_hal_cachectrl_control(am_hal_cachectrl_req_e eReq, void *pArgs)
{
    uint32_t status = AM_HAL_STATUS_SUCCESS;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Validate the parameters
    //
    if (eReq >= AM_HAL_CACHECTRL_REQ_MAX)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    switch (eReq)
    {
        case AM_HAL_CACHECTRL_REQ_PREFETCH_CFG:
            if (pArgs)
            {
                am_hal_cachectrl_prefetch_t *pConfig = (am_hal_cachectrl_prefetch_t *)pArgs;
                //
                //  Update the CM55 Pre-Fetch Control for RevB.
                //
                g_PrefetchConfig = *pConfig;
                if (SCB->CCR | (uint32_t)SCB_CCR_DC_Msk)  // Is D-Cache enabled?
                {
                    // Update the Pre-fetch configuration.
                    MEMSYSCTL->PFCR = _VAL2FLD(MEMSYSCTL_PFCR_MAX_OS, g_PrefetchConfig.ui8MaxOutTransactions)    |
                                      _VAL2FLD(MEMSYSCTL_PFCR_MAX_LA, g_PrefetchConfig.ui8MaxLookAhead)          |
                                      _VAL2FLD(MEMSYSCTL_PFCR_MIN_LA, g_PrefetchConfig.ui8MinLookAhead)          |
                                      _VAL2FLD(MEMSYSCTL_PFCR_ENABLE, 1);
                }
                status = AM_HAL_STATUS_SUCCESS;
            }
            else
            {
                status = AM_HAL_STATUS_INVALID_ARG;
            }
        break;

        default:
            status = AM_HAL_STATUS_INVALID_ARG;

    }
    return status;
}

//Clean the Data cache.
uint32_t
am_hal_cachectrl_dcache_clean(am_hal_cachectrl_range_t *pRange)
{
    //
    // If D-Cache was not enabled, only do DSB and ISB operations.
    //
    if ((SCB->CCR & (uint32_t)SCB_CCR_DC_Msk) == 0)
    {
        __DSB();
        __ISB();

        return AM_HAL_STATUS_SUCCESS;
    }
    //
    // If the address range is NULL then clean the entire D-Cache.
    //
    if (NULL == pRange)
    {
        SCB_CleanDCache();
    }
    else
    {
        SCB_CleanDCache_by_Addr((void *)pRange->ui32StartAddr, pRange->ui32Size);
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
