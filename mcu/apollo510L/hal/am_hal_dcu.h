//*****************************************************************************
//
//! @file am_hal_dcu.h
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

#ifndef AM_HAL_DCU_H
#define AM_HAL_DCU_H

#ifdef __cplusplus
extern "C"
{
#endif

#define AM_HAL_DCU_NUMDCU               21 // Number of valid bits

// Qualified DCU Controls
#define AM_HAL_DCU_CPUDBG_INVASIVE          0x00000002UL
#define AM_HAL_DCU_CPUDBG_NON_INVASIVE      0x00000004UL
#define AM_HAL_DCU_CPUTRC_ETM               0x00000008UL
#define AM_HAL_DCU_CPUTRC_DWT_SWO           0x00000010UL
#define AM_HAL_DCU_CPUTRC_PERFCNT           0x00000020UL
#define AM_HAL_DCU_CACHEDBG                 0x00000040UL
#define AM_HAL_DCU_CM4DBG                   0x00000080UL
#define AM_HAL_DCU_CPUDBG_S_INVASIVE        0x00000200UL
#define AM_HAL_DCU_CPUDBG_S_NON_INVASIVE    0x00000400UL
#define AM_HAL_DCU_SWD                      0x00000800UL
#define AM_HAL_DCU_TRACE                    0x00004000UL

// #### INTERNAL BEGIN ####
#define AM_HAL_DCU_AMBIQ_BROM_DBG           0x00008000UL
#define AM_HAL_DCU_AMBIQ_DBG_OBSERVE        0x00010000UL
#define AM_HAL_DCU_AMBIQ_TESTMODE           0x00020000UL
#define AM_HAL_DCU_AMBIQ_ANATEST            0x00040000UL
#define AM_HAL_DCU_AMBIQ_INFO1_SIGN         0x00080000UL
#define AM_HAL_DCU_AMBIQ_INFO1_PROG         0x00100000UL
// #### INTERNAL END ####

// Following macros define Raw DCU Values as in HOSTDCU registers
// These are provided only for reference
// The HAL APIs work only with Qualified values (AM_HAL_DCU_* above)

// 3 bit encoding for individual DCU fields
#define AM_HAL_DCURAWVAL_ENABLE            0x5
#define AM_HAL_DCURAWVAL_DISABLE           0x2
#define AM_HAL_DCURAWVAL_MASK              0x7

// Raw DCU control masks
#define AM_HAL_DCURAW_CPUDBG_INVASIVE          0x0000000000000007ULL
#define AM_HAL_DCURAW_CPUDBG_NON_INVASIVE      0x0000000000000038ULL
#define AM_HAL_DCURAW_CPUTRC_DWT_SWO           0x0000000000000E00ULL
#define AM_HAL_DCURAW_CPUTRC_PERFCNT           0x0000000000007000ULL
#define AM_HAL_DCURAW_CACHEDBG                 0x0000000000038000ULL
#define AM_HAL_DCURAW_CM4DBG                   0x00000000001C0000ULL
#define AM_HAL_DCURAW_CPUDBG_S_INVASIVE        0x0000000007000000ULL
#define AM_HAL_DCURAW_CPUDBG_S_NON_INVASIVE    0x0000000038000000ULL
#define AM_HAL_DCURAW_SWD                      0x00000001C0000000ULL
#define AM_HAL_DCURAW_TRACE                    0x0000038000000000ULL

// #### INTERNAL BEGIN ####
#define AM_HAL_DCURAW_AMBIQ_CPUDBG_INVASIVE          0x0000000000000038ULL
#define AM_HAL_DCURAW_AMBIQ_CPUDBG_NON_INVASIVE      0x00000000000001C0ULL
#define AM_HAL_DCURAW_AMBIQ_CPUTRC_DWT_SWO           0x0000000000007000ULL
#define AM_HAL_DCURAW_AMBIQ_CPUTRC_PERFCNT           0x0000000000038000ULL
#define AM_HAL_DCURAW_AMBIQ_CACHEDBG                 0x00000000001C0000ULL
#define AM_HAL_DCURAW_AMBIQ_CM4DBG                   0x0000000000700000ULL
#define AM_HAL_DCURAW_AMBIQ_CPUDBG_S_INVASIVE        0x0000000038000000ULL
#define AM_HAL_DCURAW_AMBIQ_CPUDBG_S_NON_INVASIVE    0x00000001C0000000ULL
#define AM_HAL_DCURAW_AMBIQ_SWD                      0x0000000E00000000ULL
#define AM_HAL_DCURAW_AMBIQ_TRACE                    0x00001C0000000000ULL
#define AM_HAL_DCURAW_AMBIQ_BROM_DBG                 0x0000E00000000000ULL
#define AM_HAL_DCURAW_AMBIQ_DBG_OBSERVE              0x0007000000000000ULL
#define AM_HAL_DCURAW_AMBIQ_TESTMODE                 0x0038000000000000ULL
#define AM_HAL_DCURAW_AMBIQ_ANATEST                  0x01C0000000000000ULL
#define AM_HAL_DCURAW_AMBIQ_INFO1_SIGN               0x0E00000000000000ULL
#define AM_HAL_DCURAW_AMBIQ_INFO1_PROG               0x7000000000000000ULL
// #### INTERNAL END ####

// All possible controls
// #### INTERNAL BEGIN ####
#define AM_HAL_DCURAW_MASK_AMBIQ       0x7FFFFF8FF81FFFF8ULL
#define AM_HAL_DCURAW_ENABLE_AMBIQ     0x5B6DB68B6816DB68ULL
#define AM_HAL_DCURAW_DISABLE_AMBIQ    0x2492490490092490ULL
// #### INTERNAL END ####
#define AM_HAL_DCURAW_MASK             0x00000381FF1FFFFFULL
#define AM_HAL_DCURAW_ENABLE           0x000002816D16DB6DULL
#define AM_HAL_DCURAW_DISABLE          0x0000010092092492ULL

// Convenience wrappers
//! @param  ui32Mask -  DCU controls to be modified - OR'ing of AM_HAL_DCU_* masks defined above
#define am_hal_dcu_enable(ui32Mask)         am_hal_dcu_update(true, (ui32Mask))
//! @param  ui32Mask -  DCU controls to be modified - OR'ing of AM_HAL_DCU_* masks defined above
#define am_hal_dcu_disable(ui32Mask)        am_hal_dcu_update(false, (ui32Mask))

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
extern uint32_t am_hal_dcu_lock_status_get(uint32_t *pui32Val);

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
extern uint32_t am_hal_dcu_lock(uint32_t ui32Mask);

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
extern uint32_t am_hal_dcu_get(uint32_t *pui32Val);

//*****************************************************************************
//
//! @brief  Update DCU Enable (Qualified Values)
//!
//! @param  ui32Mask -  DCU controls to be modified
//! @param  bEnable - Whether to enable or disable
//!
//! This will update the DCU Enable settings, if not locked
//!
//! @return Returns AM_HAL_STATUS_SUCCESS on success
//
//*****************************************************************************
extern uint32_t am_hal_dcu_update(bool bEnable, uint32_t ui32Mask);

//*****************************************************************************
//
//! @brief  DCU Disable - Using MCUCTRL Override
//!
//! @param  ui32Mask -  DCU controls to be modified (Qualified Values)
//!
//! This will update the MCUCTRL DCU Disable Override settings
//! This can only further lock things if the corresponding DCU Enable was open
//!
//! @return Returns AM_HAL_STATUS_SUCCESS on success
//
//*****************************************************************************
extern uint32_t am_hal_dcu_mcuctrl_override(uint32_t ui32Mask);

// #### INTERNAL BEGIN ####
//*****************************************************************************
//
//! @brief  Update DCU Scope
//!
//! @param  bAmbiq - Whether to operate on OEM or Ambiq set
//!
//! This will update the DCU scope for further operations
//!
//! @return Returns AM_HAL_STATUS_SUCCESS on success
//
//*****************************************************************************
extern uint32_t am_hal_dcu_set_scope(bool bAmbiq);
// #### INTERNAL END ####

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_DCU_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

