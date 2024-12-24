//*****************************************************************************
//
//! @file am_hal_status.h
//!
//! @brief Global status return codes for HAL interface.
//!
//! @addtogroup status3 Status - Global Status Return Codes.
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
#ifndef AM_HAL_STATUS_H
#define AM_HAL_STATUS_H

#ifdef __cplusplus
extern "C"
{
#endif

//
//! Global Status Returns
//
typedef enum
{
AM_HAL_STATUS_SUCCESS,
AM_HAL_STATUS_FAIL,
AM_HAL_STATUS_INVALID_HANDLE,
AM_HAL_STATUS_IN_USE,
AM_HAL_STATUS_TIMEOUT,
AM_HAL_STATUS_OUT_OF_RANGE,
AM_HAL_STATUS_INVALID_ARG,
AM_HAL_STATUS_INVALID_OPERATION,
AM_HAL_STATUS_MEM_ERR,
AM_HAL_STATUS_HW_ERR,
AM_HAL_STATUS_MODULE_SPECIFIC_START = 0x08000000,
} am_hal_status_e;

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_STATUS_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

