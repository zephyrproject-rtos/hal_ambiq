//*****************************************************************************
//
//! @file am_hal_dcu.c
//!
//! @brief Functions for DCU functions
//!
//! @addtogroup dcu DCU - Debug Control Unit
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
#include "am_mcu_apollo.h"

#define CRYPTO_CC_IS_IDLE()     while (CRYPTO->HOSTCCISIDLE_b.HOSTCCISIDLE == 0)

// Raw offset for 3b value corresponding to DCU value 1
uint32_t gStartOff    = 0; // DCU value 1 corresponds to b[2:0]
uint64_t gDcuMask     = AM_HAL_DCURAW_MASK;
uint64_t gDcuEnable   = AM_HAL_DCURAW_ENABLE;
uint64_t gDcuDisable  = AM_HAL_DCURAW_DISABLE;
volatile uint32_t *gpDcuEnable = &CRYPTO->HOSTDCUEN2;
volatile uint32_t *gpDcuLock   = &CRYPTO->HOSTDCULOCK2;

typedef union
{
    uint64_t u64;
    uint32_t u32[2];
} am_hal_64b_dcu_t;

//*****************************************************************************
//
//! @brief Get the Current RAW DCU Mask
//
//*****************************************************************************
static uint64_t
get_raw_dcu_mask(uint32_t ui32DcuMask, uint8_t threeBitVal)
{
    uint32_t i = AM_HAL_DCU_NUMDCU;
    uint64_t ui64Mask = 0;
    uint32_t offset = gStartOff;
    ui32DcuMask >>= 1; // DCU value 0 is not defined
    while ( --i )
    {
        if (ui32DcuMask & 0x1)
        {
            ui64Mask |= (uint64_t)threeBitVal << offset;
        }
        offset += 3;
        ui32DcuMask >>= 1;
    }
    return ui64Mask;
}

//*****************************************************************************
//
//! @brief Get the Current DCU Mask
//
//*****************************************************************************
static uint32_t
get_ui32_dcu_mask(uint64_t ui64DcuMask, uint8_t threeBitVal)
{
    uint32_t i = AM_HAL_DCU_NUMDCU;
    uint32_t ui32Mask = 0;
    ui64DcuMask >>= gStartOff;
    while ( --i )
    {
        if ((ui64DcuMask & AM_HAL_DCURAWVAL_MASK) == threeBitVal)
        {
            ui32Mask |= (1 << (AM_HAL_DCU_NUMDCU - i - 1));
        }
        ui64DcuMask >>= 3;
    }
    return ui32Mask;
}

//*****************************************************************************
//
//! @brief  Read DCU Lock
//!
//! @param  pui64Val -  Pointer to double word for returned data
//!
//! This will retrieve the DCU Lock information
//!
//! @return Returns AM_HAL_STATUS_SUCCESS on success
//
//*****************************************************************************
static
uint32_t am_hal_dcu_raw_lock_status_get(uint64_t *pui64Val)
{
    am_hal_64b_dcu_t value;
    value.u32[0] = AM_REGVAL(gpDcuLock);
    value.u32[1] = AM_REGVAL(gpDcuLock + 1);
    *pui64Val = value.u64;
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief  Read DCU Lock
//!
//! @param  pui32Val -  Pointer to word for returned data (Qualified DCU Mask)
//!
//! This will retrieve the DCU Lock information
//!
//! @return Returns AM_HAL_STATUS_SUCCESS on success
//
//*****************************************************************************
uint32_t am_hal_dcu_lock_status_get(uint32_t *pui32Val)
{
    uint64_t ui64Lock;
    uint32_t ui32Status;
    if ((PWRCTRL->DEVPWRSTATUS_b.PWRSTCRYPTO == 0) || (CRYPTO->HOSTCCISIDLE_b.HOSTCCISIDLE == 0))
    {
        // Crypto is not accessible
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
    ui32Status = am_hal_dcu_raw_lock_status_get(&ui64Lock);
    *pui32Val = get_ui32_dcu_mask(ui64Lock, AM_HAL_DCURAWVAL_MASK);
    return ui32Status;
}

//*****************************************************************************
//
//! @brief  Write DCU Lock
//!
//! @param  val -  double word for lock values
//!
//! This will lock the DCU from further changes
//!
//! @return Returns AM_HAL_STATUS_SUCCESS on success
//
//*****************************************************************************
static
uint32_t am_hal_dcu_raw_lock(uint64_t ui64Mask)
{
// #### INTERNAL BEGIN ####
    //
    // Prior to the following change, the GCC 10.3.1 compiler was issuing a
    // "may be used uninitialized in this function" warning. The warning came
    // about from the inlining of copy_words() in this function. After inlining,
    // the compiler apparently was looking at the copy_words() pSrc argument as
    // an uninitialized local variable and thus issuing the warning.
    //
    // Two methods were devised to eliminate the warning. One was used as the
    // final solution (with approval between Ron and Christian).
    // Here is the 2nd, which still uses copy_words().
    // Store the 64-bit value into its word components in a local array.
    // Then the ptr to that array can be safely passed to copy_words().
    //uint32_t ui32tmp[2];
    //ui32tmp[0] = (uint32_t)(ui64Mask >> 0);
    //ui32tmp[1] = (uint32_t)(ui64Mask >> 32);
    //copy_words((uint32_t *)gpDcuLock, ui32tmp, sizeof(uint64_t) / 4);
// #### INTERNAL END ####
// #### INTERNAL BEGIN ####
    // Finally, in order to double-check the correct word order of the uint64_t,
    // the following code was added to hello_world and the debugger used to
    // verify actual memory placement.
    //uint32_t ui32tmp[2];                        // Global variable
    //uint64_t ui64value = 0x123456789abcdef0;    // Global variable
    // This demonstrates how the 64-bit data is saved in memory.
    // After running the following code from the debugger, dump both ui32tmp
    // and ui64value in bytes or words, they should look identical in memory.
    //am_util_stdio_printf("ui64value = 0x%08x%08x\n", (uint32_t)(ui64value >> 32), (uint32_t)(ui64value >> 0);
    //ui32tmp[0] = (uint32_t)(ui64value >> 0);
    //ui32tmp[1] = (uint32_t)(ui64value >> 32);
    //am_util_stdio_printf("ui64value = 0x%08x%08x\n", ui32tmp[1], ui32tmp[0]  );
    //am_util_stdio_printf("ui64 test done.\n\n");
// #### INTERNAL END ####
    //
    // copy_words((uint32_t *)gpDcuLock, (uint32_t *)&ui64Mask, sizeof(uint64_t) / 4);
    //
    // In order to avoid a GCC compiler warning, we'll explicitly handle the
    // call to copy_words() by replacing it with a couple of direct writes.
    // Note that the writes are done in the same reverse order as copy_words()
    // would do it.
    //
    gpDcuLock[1] = (uint32_t)(ui64Mask >> 32);
    gpDcuLock[0] = (uint32_t)(ui64Mask >> 0);
    CRYPTO_CC_IS_IDLE();
    return AM_HAL_STATUS_SUCCESS;
} // am_hal_dcu_raw_lock()

//*****************************************************************************
//
//! @brief  Write DCU Lock (Qualified Values)
//!
//! @param  ui32Mask -  Mask for lock values
//!
//! This will lock the DCU from further changes
//!
//! @return Returns AM_HAL_STATUS_SUCCESS on success
//
//*****************************************************************************
uint32_t am_hal_dcu_lock(uint32_t ui32Mask)
{
    uint64_t ui64Lock;
    if ((PWRCTRL->DEVPWRSTATUS_b.PWRSTCRYPTO == 0) || (CRYPTO->HOSTCCISIDLE_b.HOSTCCISIDLE == 0))
    {
        // Crypto is not accessible
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
    ui64Lock = get_raw_dcu_mask(ui32Mask, AM_HAL_DCURAWVAL_MASK);
    return am_hal_dcu_raw_lock(ui64Lock);
}

//*****************************************************************************
//
//! @brief  Read DCU Enables
//!
//! @param  pui64Val -  Pointer to double word for returned data
//!
//! This will get the current DCU Enable settings
//!
//! @return Returns AM_HAL_STATUS_SUCCESS on success
//
//*****************************************************************************
static
uint32_t am_hal_dcu_raw_get(uint64_t *pui64Val)
{
    am_hal_64b_dcu_t value;
    value.u32[0] = AM_REGVAL(gpDcuEnable);
    value.u32[1] = AM_REGVAL(gpDcuEnable + 1);
    *pui64Val = value.u64;
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief  Read DCU Enables (Qualified Values)
//!
//! @param  pui32Val -  Pointer to Mask for returned data
//!
//! This will get the current DCU Enable settings
//!
//! @return Returns AM_HAL_STATUS_SUCCESS on success
//
//*****************************************************************************
uint32_t am_hal_dcu_get(uint32_t *pui32Val)
{
    uint64_t ui64Enable;
    uint32_t ui32Status;
    if ((PWRCTRL->DEVPWRSTATUS_b.PWRSTCRYPTO == 0) || (CRYPTO->HOSTCCISIDLE_b.HOSTCCISIDLE == 0))
    {
        // Crypto is not accessible
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
    ui32Status = am_hal_dcu_raw_get(&ui64Enable);
    *pui32Val = get_ui32_dcu_mask(ui64Enable, AM_HAL_DCURAWVAL_ENABLE);
    return ui32Status;
}

//*****************************************************************************
//
//! @brief  Update DCU Enable
//!
//! @param  ui64Mask -  DCU controls to be modified
//! @param  bEnable - Whether to enable or disable
//!
//! This will update the DCU Enable settings, if not locked
//!
//! @return Returns AM_HAL_STATUS_SUCCESS on success
//
//*****************************************************************************
static
uint32_t am_hal_dcu_raw_update(bool bEnable, uint64_t ui64Mask)
{
    am_hal_64b_dcu_t dcuVal;
    am_hal_64b_dcu_t dcuLock;
    dcuLock.u32[0] = AM_REGVAL(gpDcuLock);
    dcuLock.u32[1] = AM_REGVAL(gpDcuLock + 1);
    if (ui64Mask & dcuLock.u64)
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
    dcuVal.u32[0] = AM_REGVAL(gpDcuEnable);
    dcuVal.u32[1] = AM_REGVAL(gpDcuEnable + 1);
    if (bEnable)
    {
        dcuVal.u64 = (dcuVal.u64 & ~ui64Mask) | (gDcuEnable & ui64Mask);
    }
    else
    {
        dcuVal.u64 = (dcuVal.u64 & ~ui64Mask) | (gDcuDisable & ui64Mask);
    }
    AM_REGVAL(gpDcuEnable) = dcuVal.u32[0];
    AM_REGVAL(gpDcuEnable + 1) = dcuVal.u32[1];
    CRYPTO_CC_IS_IDLE();
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Update DCU Enable (Qualified Values)
//
// This will update the DCU Enable settings, if not locked
//
//*****************************************************************************
uint32_t am_hal_dcu_update(bool bEnable, uint32_t ui32Mask)
{
    uint64_t ui64Mask;
    if ((PWRCTRL->DEVPWRSTATUS_b.PWRSTCRYPTO == 0) || (CRYPTO->HOSTCCISIDLE_b.HOSTCCISIDLE == 0))
    {
        // Crypto is not accessible
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
    ui64Mask = get_raw_dcu_mask(ui32Mask, AM_HAL_DCURAWVAL_MASK);
    return am_hal_dcu_raw_update(bEnable, ui64Mask);
}

//*****************************************************************************
//
// DCU Disable - Using MCUCTRL Override
//
// This will update the MCUCTRL DCU Disable Override settings
// This can only further lock things if the corresponding DCU Enable was open
//
//*****************************************************************************
uint32_t am_hal_dcu_mcuctrl_override(uint32_t ui32Mask)
{
    MCUCTRL->DEBUGGER = ui32Mask;
    return AM_HAL_STATUS_SUCCESS;
}

// #### INTERNAL BEGIN ####
//*****************************************************************************
//
// Update DCU Scope
//
// This will update the DCU scope for further operations
//
//*****************************************************************************
uint32_t am_hal_dcu_set_scope(bool bAmbiq)
{
    if (bAmbiq)
    {
        gpDcuEnable = &CRYPTO->HOSTDCUEN0;
        gpDcuLock   = &CRYPTO->HOSTDCULOCK0;
        gDcuMask    = AM_HAL_DCURAW_MASK_AMBIQ;
        gDcuEnable  = AM_HAL_DCURAW_ENABLE_AMBIQ;
        gDcuDisable = AM_HAL_DCURAW_DISABLE_AMBIQ;
        gStartOff   = 3; // DCU value 1 corresponds to b[5:3]
    }
    else
    {
        gpDcuEnable = &CRYPTO->HOSTDCUEN2;
        gpDcuLock   = &CRYPTO->HOSTDCULOCK2;
        gDcuMask    = AM_HAL_DCURAW_MASK;
        gDcuEnable  = AM_HAL_DCURAW_ENABLE;
        gDcuDisable = AM_HAL_DCURAW_DISABLE;
        gStartOff   = 0;
    }
    return AM_HAL_STATUS_SUCCESS;
}
// #### INTERNAL END ####

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
