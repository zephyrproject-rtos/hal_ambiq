//*****************************************************************************
//
//! @file am_reg.h
//!
//! @brief Apollo510L register macros
//!
//! @addtogroup reg_macros_5 Apollo510L Register Macros
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
#ifndef AM_REG_H
#define AM_REG_H

//*****************************************************************************
//
//! @brief SYSPLL
//! Instance finder. (1 instance(s) available)
//
//*****************************************************************************
#define AM_REG_SYSPLL_NUM_MODULES                       1

//*****************************************************************************
//
//! @brief CLKGEN
//! Instance finder. (1 instance(s) available)
//
//*****************************************************************************
#define AM_REG_CLKGEN_NUM_MODULES                    1
#define AM_REG_CLKGENn(n) \
    (CLKGEN_BASE + 0x00000000 * n)

//*****************************************************************************
//
//! @brief CM55IPC
//! Instance finder. (1 instance(s) available)
//
//*****************************************************************************
#define AM_REG_CM55IPC_NUM_MODULES                   1
#define AM_REG_CM55IPCn(n) \
    (CM55IPC_BASE + 0x00000000 * n)

//*****************************************************************************
//
//! @brief CRM
//! Instance finder. (1 instance(s) available)
//
//*****************************************************************************
#define AM_REG_CRM_NUM_MODULES                       1
#define AM_REG_CRMn(n) \
    (CRM_BASE + 0x00000000 * n)

//*****************************************************************************
//
//! @brief RSTGEN
//! Instance finder. (1 instance(s) available)
//
//*****************************************************************************
#define AM_REG_RSTGEN_NUM_MODULES                    1
#define AM_REG_RSTGENn(n) \
    (RSTGEN_BASE + 0x00000000 * n)

//*****************************************************************************
//
//! @brief RTC
//! Instance finder. (1 instance(s) available)
//
//*****************************************************************************
#define AM_REG_RTC_NUM_MODULES                       1
#define AM_REG_RTCn(n) \
    (RTC_BASE + 0x00000000 * n)

//*****************************************************************************
//
//! @brief SECURITY
//! Instance finder. (1 instance(s) available)
//
//*****************************************************************************
#define AM_REG_SECURITY_NUM_MODULES                  1
#define AM_REG_SECURITYn(n) \
    (SECURITY_BASE + 0x00001000 * n)

//*****************************************************************************
//
//! @brief WDT
//! Instance finder. (1 instance(s) available)
//
//*****************************************************************************
#define AM_REG_WDT_NUM_MODULES                       1
#define AM_REG_WDTn(n) \
    (WDT_BASE + 0x00000000 * n)

//*****************************************************************************
//
//! @brief I2S
//! Instance finder. (2 instance(s) available)
//
//*****************************************************************************
#define AM_REG_I2S_NUM_MODULES                       2
#define AM_REG_I2Sn(n) \
    (I2S0_BASE + 0x00001000 * n)

//*****************************************************************************
//
//! @brief PDM
//! Instance finder. (4 instance(s) available)
//
//*****************************************************************************
#define AM_REG_PDM_NUM_MODULES                       4
#define AM_REG_PDMn(n) \
    (PDM0_BASE + 0x00001000 * n)

//*****************************************************************************
//
//! @brief ADC
//! Instance finder. (1 instance(s) available)
//
//*****************************************************************************
#define AM_REG_ADC_NUM_MODULES                       1
#define AM_REG_ADCn(n) \
    (ADC_BASE + 0x00000000 * n)

//*****************************************************************************
//
//! @brief CRYPTO
//! Instance finder. (1 instance(s) available)
//
//*****************************************************************************
#define AM_REG_CRYPTO_NUM_MODULES                    1
#define AM_REG_CRYPTOn(n) \
    (CRYPTO_BASE + 0x00000000 * n)

//*****************************************************************************
//
//! @brief FPIO
//! Instance finder. (1 instance(s) available)
//
//*****************************************************************************
#define AM_REG_FPIO_NUM_MODULES                      1
#define AM_REG_FPIOn(n) \
    (FPIO_BASE + 0x00000000 * n)

//*****************************************************************************
//
//! @brief GPIO
//! Instance finder. (1 instance(s) available)
//
//*****************************************************************************
#define AM_REG_GPIO_NUM_MODULES                      1
#define AM_REG_GPIOn(n) \
    (GPIO_BASE + 0x00000004 * n)

//*****************************************************************************
//
//! @brief IOM
//! Instance finder. (6 instance(s) available)
//
//*****************************************************************************
#define AM_REG_IOM_NUM_MODULES                       6
#define AM_REG_IOMn(n) \
    (IOM0_BASE + 0x00001000 * n)

//*****************************************************************************
//
//! @brief IOSLAVEFD
//! Instance finder. (2 instance(s) available)
//
//*****************************************************************************
#define AM_REG_IOSLAVEFD_NUM_MODULES                 2
#define AM_REG_IOSLAVEFDn(n) \
    (IOSLAVEFD0_BASE + 0x00001000 * n)

//*****************************************************************************
//
//! @brief MCUCTRL
//! Instance finder. (1 instance(s) available)
//
//*****************************************************************************
#define AM_REG_MCUCTRL_NUM_MODULES                   1
#define AM_REG_MCUCTRLn(n) \
    (MCUCTRL_BASE + 0x00000000 * n)

//*****************************************************************************
//
//! @brief MSPI
//! Instance finder. (3 instance(s) available)
//
//*****************************************************************************
#define AM_REG_MSPI_NUM_MODULES                      3
#define AM_REG_MSPIn(n) \
    (MSPI0_BASE + 0x00001000 * n)

//*****************************************************************************
//
//! @brief OTP
//! Instance finder. (1 instance(s) available)
//
//*****************************************************************************
#define AM_REG_OTP_NUM_MODULES                       1
#define AM_REG_OTPn(n) \
    (OTP_BASE + 0x00000000 * n)

//*****************************************************************************
//
//! @brief PWRCTRL
//! Instance finder. (1 instance(s) available)
//
//*****************************************************************************
#define AM_REG_PWRCTRL_NUM_MODULES                   1
#define AM_REG_PWRCTRLn(n) \
    (PWRCTRL_BASE + 0x00000000 * n)

//*****************************************************************************
//
//! @brief SDIO
//! Instance finder. (2 instance(s) available)
//
//*****************************************************************************
#define AM_REG_SDIO_NUM_MODULES                      2
#define AM_REG_SDIOn(n) \
    (SDIO0_BASE + 0x00001000 * n)

//*****************************************************************************
//
//! @brief SSC
//! Instance finder. (1 instance(s) available)
//
//*****************************************************************************
#define AM_REG_SSC_NUM_MODULES                       1
#define AM_REG_SSCn(n) \
    (SSC_BASE + 0x00000000 * n)

//*****************************************************************************
//
//! @brief STIMER
//! Instance finder. (1 instance(s) available)
//
//*****************************************************************************
#define AM_REG_STIMER_NUM_MODULES                    1
#define AM_REG_STIMERn(n) \
    (STIMER_BASE + 0x00000000 * n)

//*****************************************************************************
//
//! @brief TIMER
//! Instance finder. (1 instance(s) available)
//
//*****************************************************************************
#define AM_REG_TIMER_NUM_MODULES                     1
#define AM_REG_TIMERn(n) \
    (TIMER_BASE + 0x00000020 * n)

//*****************************************************************************
//
//! @brief UART
//! Instance finder. (2 instance(s) available)
//
//*****************************************************************************
#define AM_REG_UART_NUM_MODULES                      2
#define AM_REG_UARTn(n) \
    (UART0_BASE + 0x00001000 * n)

//*****************************************************************************
//
//! @brief VCOMP
//! Instance finder. (1 instance(s) available)
//
//*****************************************************************************
#define AM_REG_VCOMP_NUM_MODULES                     1
#define AM_REG_VCOMPn(n) \
    (VCOMP_BASE + 0x00000000 * n)

//*****************************************************************************
//
//! @brief DC
//! Instance finder. (1 instance(s) available)
//
//*****************************************************************************
#define AM_REG_DC_NUM_MODULES                        1
#define AM_REG_DCn(n) \
    (DC_BASE + 0x00000000 * n)

//*****************************************************************************
//
//! @brief DSI
//! Instance finder. (1 instance(s) available)
//
//*****************************************************************************
#define AM_REG_DSI_NUM_MODULES                       1
#define AM_REG_DSIn(n) \
    (DSI_BASE + 0x00000000 * n)

//*****************************************************************************
//
//! @brief GPU
//! Instance finder. (1 instance(s) available)
//
//*****************************************************************************
#define AM_REG_GPU_NUM_MODULES                       1
#define AM_REG_GPUn(n) \
    (GPU_BASE + 0x00000000 * n)

//*****************************************************************************
//
//! @brief I3C
//! Instance finder. (1 instance(s) available)
//
//*****************************************************************************
#define AM_REG_I3C_NUM_MODULES                       1
#define AM_REG_I3Cn(n) \
    (I3C_BASE + 0x00001000 * n)

//*****************************************************************************
//
//! @brief USB
//! Instance finder. (1 instance(s) available)
//
//*****************************************************************************
#define AM_REG_USB_NUM_MODULES                       1
#define AM_REG_USBn(n) \
    (USB_BASE + 0x00000004 * n)

//*****************************************************************************
//
//! @brief USBPHY
//! Instance finder. (1 instance(s) available)
//
//*****************************************************************************
#define AM_REG_USBPHY_NUM_MODULES                    1
#define AM_REG_USBPHYn(n) \
    (USBPHY_BASE + 0x00000004 * n)

#endif // AM_REG_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
