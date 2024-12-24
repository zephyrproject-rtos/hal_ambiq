//*****************************************************************************
//
//! @file am_hal_infoc.h
//!
//! @brief Functions for INFOC functions
//!
//! @addtogroup infoc INFOC - One-Time Programmable
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

#ifndef AM_HAL_INFOC_H
#define AM_HAL_INFOC_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! @brief  Read INFOC word
//!
//! @param  offset -  word aligned offset in INFOC to be read
//! @param  pVal -  Pointer to word for returned data
//!
//! This will retrieve the INFOC information
//!
//! @note The caller is responsible for powering up OTP before calling this
//! function.
//!
//! The current power status of OTP can be determined via
//! am_hal_pwrctrl_periph_enabled(AM_HAL_PWRCTRL_PERIPH_OTP, &bEnabled);
//!
//! OTP is enabled or disabled via:
//! am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_OTP);
//! am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_OTP);
//!
//! @return Returns AM_HAL_STATUS_SUCCESS on success
//
//*****************************************************************************
extern uint32_t am_hal_infoc_read_word(uint32_t offset, uint32_t *pVal);

//*****************************************************************************
//
//! @brief  Write INFOC word
//!
//! @param  offset -  word aligned offset in INFOC to be read
//! @param  value -  value to be written
//!
//! This will write a word to the supplied offset in the INFOC
//!
//! @note The caller is responsible for powering up OTP before calling this
//! function.
//!
//! The current power status of OTP can be determined via
//! am_hal_pwrctrl_periph_enabled(AM_HAL_PWRCTRL_PERIPH_OTP, &bEnabled);
//!
//! OTP is enabled or disabled via:
//! am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_OTP);
//! am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_OTP);
//!
//! @return Returns AM_HAL_STATUS_SUCCESS on success
//
//*****************************************************************************
extern uint32_t am_hal_infoc_write_word(uint32_t offset, uint32_t value);

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_INFOC_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

