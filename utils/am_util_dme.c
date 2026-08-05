//*****************************************************************************
//
//! @file am_util_dme.c
//!
//! @brief DMA-350 utility helpers built on the DME HAL.
//!
//! @addtogroup dma_350_utils DMA-350 Utility Functions
//! @ingroup utils
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! Copyright (c) 2026, Ambiq Micro, Inc.
//! All rights reserved.
//!
//! Redistribution and use in source and binary forms, with or without
//! modification, are permitted provided that the following conditions are met:
//!
//! 1. Redistributions of source code must retain the above copyright notice,
//! this list of conditions and the following disclaimer.
//!
//! 2. Redistributions in binary form must reproduce the above copyright
//! notice, this list of conditions and the following disclaimer in the
//! documentation and/or other materials provided with the distribution.
//!
//! 3. Neither the name of the copyright holder nor the names of its
//! contributors may be used to endorse or promote products derived from this
//! software without specific prior written permission.
//!
//! Third party software included in this distribution is subject to the
//! additional license terms as defined in the /docs/licenses directory.
//!
//! THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//! AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//! IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//! ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
//! LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//! CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//! SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//! INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//! CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//! ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//! POSSIBILITY OF SUCH DAMAGE.
//!
//! This is part of revision v5.2.0-zephyr-685438d73f of the AmbiqSuite Development Package.
//
//*****************************************************************************

#include "am_mcu_apollo.h"
#include "am_util_dme.h"
#include <stddef.h>

#define AM_UTIL_DME_TIMEOUT_MIN_US    1000U
#define AM_UTIL_DME_MAX_TRANSIZE      4U

//*****************************************************************************
//
//! @brief Common 2D copy configuration structure.
//
//*****************************************************************************
typedef struct
{
    uint32_t ui32SrcAddr;
    uint32_t ui32DstAddr;
    uint32_t ui32SrcXSize;
    uint32_t ui32SrcYSize;
    uint32_t ui32DstXSize;
    uint32_t ui32DstYSize;
    uint32_t ui32SrcLineWidth;
    uint32_t ui32DstLineWidth;
    am_util_dme_2d_copy_mode_e eCopyMode;
    am_hal_dme_transize_e eTransSize;
} am_util_dme_common_2d_copy_config_t;

//*****************************************************************************
//
//! @brief Assert that the layout of the common 2D copy configuration structure
//!        is compatible with both am_util_dme_memcpy_2d_config_t and
//!        am_util_dme_wrapcpy_2d_config_t.
//
//*****************************************************************************
#define AM_UTIL_DME_ASSERT_COMMON_2D_COPY_LAYOUT(sType)                                                   \
    _Static_assert(sizeof(am_util_dme_common_2d_copy_config_t) == sizeof(sType),                          \
                   #sType " must remain layout-compatible with am_util_dme_common_2d_copy_config_t");    \
    _Static_assert(offsetof(am_util_dme_common_2d_copy_config_t, ui32SrcAddr) == offsetof(sType, ui32SrcAddr),         \
                   #sType " ui32SrcAddr layout mismatch");                                                \
    _Static_assert(offsetof(am_util_dme_common_2d_copy_config_t, ui32DstAddr) == offsetof(sType, ui32DstAddr),         \
                   #sType " ui32DstAddr layout mismatch");                                                \
    _Static_assert(offsetof(am_util_dme_common_2d_copy_config_t, ui32SrcXSize) == offsetof(sType, ui32SrcXSize),       \
                   #sType " ui32SrcXSize layout mismatch");                                               \
    _Static_assert(offsetof(am_util_dme_common_2d_copy_config_t, ui32SrcYSize) == offsetof(sType, ui32SrcYSize),       \
                   #sType " ui32SrcYSize layout mismatch");                                               \
    _Static_assert(offsetof(am_util_dme_common_2d_copy_config_t, ui32DstXSize) == offsetof(sType, ui32DstXSize),       \
                   #sType " ui32DstXSize layout mismatch");                                               \
    _Static_assert(offsetof(am_util_dme_common_2d_copy_config_t, ui32DstYSize) == offsetof(sType, ui32DstYSize),       \
                   #sType " ui32DstYSize layout mismatch");                                               \
    _Static_assert(offsetof(am_util_dme_common_2d_copy_config_t, ui32SrcLineWidth) == offsetof(sType, ui32SrcLineWidth), \
                   #sType " ui32SrcLineWidth layout mismatch");                                           \
    _Static_assert(offsetof(am_util_dme_common_2d_copy_config_t, ui32DstLineWidth) == offsetof(sType, ui32DstLineWidth), \
                   #sType " ui32DstLineWidth layout mismatch");                                           \
    _Static_assert(offsetof(am_util_dme_common_2d_copy_config_t, eCopyMode) == offsetof(sType, eCopyMode),             \
                   #sType " eCopyMode layout mismatch");                                                  \
    _Static_assert(offsetof(am_util_dme_common_2d_copy_config_t, eTransSize) == offsetof(sType, eTransSize),           \
                   #sType " eTransSize layout mismatch")
AM_UTIL_DME_ASSERT_COMMON_2D_COPY_LAYOUT(am_util_dme_memcpy_2d_config_t);
AM_UTIL_DME_ASSERT_COMMON_2D_COPY_LAYOUT(am_util_dme_wrapcpy_2d_config_t);
#undef AM_UTIL_DME_ASSERT_COMMON_2D_COPY_LAYOUT

//*****************************************************************************
//
//! @brief DME configuration templates
//
//*****************************************************************************
// Configuration template for memory copy operations
static const am_hal_dme_ch_config_t g_am_util_dma_350_memcpy_config_default =
{
    .mask.mask_b.regclear_msk = 1,
    .mask.mask_b.ctrl_msk = 1,
    .mask.mask_b.srcaddr_msk = 1,
    .mask.mask_b.desaddr_msk = 1,
    .mask.mask_b.xsize_msk = 1,
    .mask.mask_b.xsizehi_msk = 1,
    .mask.mask_b.xaddrinc_msk = 1,
    .mask.mask_b.srctranscfg_msk = 1,
    .mask.mask_b.destranscfg_msk = 1,
    .intren.u32 = 0,
    .ctrl.donepauseen = 0,
    .ctrl.regreloadtype = AM_HAL_DME_RELOADTYPE_DISABLED,
    .ctrl.chprio = AM_HAL_DME_CHPRIO_0,
    .ctrl.xtype = AM_HAL_DME_XTYPE_CONTINUE,
    .ctrl.ytype = AM_HAL_DME_YTYPE_DISABLE,
    .ctrl.donetype = AM_HAL_DME_DONETYPE_END_OF_COMMAND,
    .ctrl.transize = AM_HAL_DME_TRANSIZE_1BYTE,
    .srcaddr = 0,
    .desaddr = 0,
    .xsize.u32 = 0,
    .xsizehi.u32 = 0,
    .srctranscfg.maxburstlen = 15,
    .srctranscfg.privattr = AM_HAL_DME_PRIVATTR_UNPRIVILEGED,
    .srctranscfg.nonsecattr = AM_HAL_DME_NONSECATTR_NONSECURE,
    .srctranscfg.shareattr = AM_HAL_DME_SHRATTR_DEFAULT,
    .srctranscfg.memattrlo = AM_HAL_DME_MEMATTR_LO_DEFAULT,
    .srctranscfg.memattrhi = AM_HAL_DME_MEMATTR_HI_DEFAULT,
    .destranscfg.maxburstlen = 15,
    .destranscfg.privattr = AM_HAL_DME_PRIVATTR_UNPRIVILEGED,
    .destranscfg.nonsecattr = AM_HAL_DME_NONSECATTR_NONSECURE,
    .destranscfg.shareattr = AM_HAL_DME_SHRATTR_DEFAULT,
    .destranscfg.memattrlo = AM_HAL_DME_MEMATTR_LO_DEFAULT,
    .destranscfg.memattrhi = AM_HAL_DME_MEMATTR_HI_DEFAULT,
    .xaddrinc.u32 = 0,
    .xaddrinc.src = 1,
    .xaddrinc.des = 1,
    .yaddrstride.u32 = 0,
    .fillval = 0,
    .ysize.u32 = 0,
    .tmpltcfg.u32 = 0,
    .srctmplt.u32 = 0,
    .destmplt.u32 = 0,
    .linkattr.u32 = 0,
    .linkattr.shareattr = AM_HAL_DME_SHRATTR_DEFAULT,
    .linkattr.memattrlo = AM_HAL_DME_MEMATTR_LO_DEFAULT,
    .linkattr.memattrhi = AM_HAL_DME_MEMATTR_HI_DEFAULT,
    .autocfg.u32 = 0,
    .linkaddr.u32 = 0,
};

// Configuration template for 2D memory copy operations
static const am_hal_dme_ch_config_t g_am_util_dma_350_copy_2d_config_default =
{
    .mask.mask_b.regclear_msk = 1,
    .mask.mask_b.ctrl_msk = 1,
    .mask.mask_b.srcaddr_msk = 1,
    .mask.mask_b.desaddr_msk = 1,
    .mask.mask_b.xsize_msk = 1,
    .mask.mask_b.xsizehi_msk = 1,
    .mask.mask_b.xaddrinc_msk = 1,
    .mask.mask_b.yaddrstride_msk = 1,
    .mask.mask_b.ysize_msk = 1,
    .mask.mask_b.srctranscfg_msk = 1,
    .mask.mask_b.destranscfg_msk = 1,
    .intren.u32 = 0,
    .ctrl.donepauseen = 0,
    .ctrl.regreloadtype = AM_HAL_DME_RELOADTYPE_DISABLED,
    .ctrl.chprio = AM_HAL_DME_CHPRIO_0,
    .ctrl.xtype = AM_HAL_DME_XTYPE_CONTINUE,
    .ctrl.ytype = AM_HAL_DME_YTYPE_CONTINUE,
    .ctrl.donetype = AM_HAL_DME_DONETYPE_END_OF_COMMAND,
    .ctrl.transize = AM_HAL_DME_TRANSIZE_1BYTE,
    .srcaddr = 0,
    .desaddr = 0,
    .xsize.u32 = 0,
    .xsizehi.u32 = 0,
    .srctranscfg.maxburstlen = 15,
    .srctranscfg.privattr = AM_HAL_DME_PRIVATTR_UNPRIVILEGED,
    .srctranscfg.nonsecattr = AM_HAL_DME_NONSECATTR_NONSECURE,
    .srctranscfg.shareattr = AM_HAL_DME_SHRATTR_DEFAULT,
    .srctranscfg.memattrlo = AM_HAL_DME_MEMATTR_LO_DEFAULT,
    .srctranscfg.memattrhi = AM_HAL_DME_MEMATTR_HI_DEFAULT,
    .destranscfg.maxburstlen = 15,
    .destranscfg.privattr = AM_HAL_DME_PRIVATTR_UNPRIVILEGED,
    .destranscfg.nonsecattr = AM_HAL_DME_NONSECATTR_NONSECURE,
    .destranscfg.shareattr = AM_HAL_DME_SHRATTR_DEFAULT,
    .destranscfg.memattrlo = AM_HAL_DME_MEMATTR_LO_DEFAULT,
    .destranscfg.memattrhi = AM_HAL_DME_MEMATTR_HI_DEFAULT,
    .xaddrinc.u32 = 0,
    .xaddrinc.src = 1,
    .xaddrinc.des = 1,
    .yaddrstride.u32 = 0,
    .fillval = 0,
    .ysize.u32 = 0,
    .tmpltcfg.u32 = 0,
    .srctmplt.u32 = 0,
    .destmplt.u32 = 0,
    .linkattr.u32 = 0,
    .linkattr.shareattr = AM_HAL_DME_SHRATTR_DEFAULT,
    .linkattr.memattrlo = AM_HAL_DME_MEMATTR_LO_DEFAULT,
    .linkattr.memattrhi = AM_HAL_DME_MEMATTR_HI_DEFAULT,
    .autocfg.u32 = 0,
    .linkaddr.u32 = 0,
};

// Configuration template for wrapped-copy operations with fill-value
static const am_hal_dme_ch_config_t g_am_util_dma_350_wrapcpy_config_default_fillval =
{
    .mask.mask_b.regclear_msk = 1,
    .mask.mask_b.ctrl_msk = 1,
    .mask.mask_b.desaddr_msk = 1,
    .mask.mask_b.xsize_msk = 1,
    .mask.mask_b.xsizehi_msk = 1,
    .mask.mask_b.xaddrinc_msk = 1,
    .mask.mask_b.destranscfg_msk = 1,
    .mask.mask_b.fillval_msk = 1,
    .intren.u32 = 0,
    .ctrl.donepauseen = 0,
    .ctrl.regreloadtype = AM_HAL_DME_RELOADTYPE_DISABLED,
    .ctrl.chprio = AM_HAL_DME_CHPRIO_0,
    .ctrl.xtype = AM_HAL_DME_XTYPE_FILL,
    .ctrl.ytype = AM_HAL_DME_YTYPE_DISABLE,
    .ctrl.donetype = AM_HAL_DME_DONETYPE_END_OF_COMMAND,
    .ctrl.transize = AM_HAL_DME_TRANSIZE_1BYTE,
    .srcaddr = 0,
    .desaddr = 0,
    .xsize.u32 = 0,
    .xsizehi.u32 = 0,
    .destranscfg.maxburstlen = 15,
    .destranscfg.privattr = AM_HAL_DME_PRIVATTR_UNPRIVILEGED,
    .destranscfg.nonsecattr = AM_HAL_DME_NONSECATTR_NONSECURE,
    .destranscfg.shareattr = AM_HAL_DME_SHRATTR_DEFAULT,
    .destranscfg.memattrlo = AM_HAL_DME_MEMATTR_LO_DEFAULT,
    .destranscfg.memattrhi = AM_HAL_DME_MEMATTR_HI_DEFAULT,
    .xaddrinc.u32 = 0,
    .xaddrinc.des = 1,
    .yaddrstride.u32 = 0,
    .fillval = 0,
    .ysize.u32 = 0,
    .tmpltcfg.u32 = 0,
    .srctmplt.u32 = 0,
    .destmplt.u32 = 0,
    .linkattr.u32 = 0,
    .linkattr.shareattr = AM_HAL_DME_SHRATTR_DEFAULT,
    .linkattr.memattrlo = AM_HAL_DME_MEMATTR_LO_DEFAULT,
    .linkattr.memattrhi = AM_HAL_DME_MEMATTR_HI_DEFAULT,
    .autocfg.u32 = 0,
    .linkaddr.u32 = 0,
};

//*****************************************************************************
//
//! @brief Internal helper to read a 32-bit X dimension count from the split
//!        XSIZE and XSIZEHI registers.
//!
//! @param psConfig - Pointer to the configuration structure.
//! @param bSourceSide - Selects the source count when true, destination count
//!                      otherwise.
//!
//! @return The 32-bit X dimension count.
//*****************************************************************************
static inline uint32_t
am_util_dme_get_xsize_count(const am_hal_dme_ch_config_t *psConfig,
                            bool bSourceSide)
{
    if (bSourceSide)
    {
        return (((uint32_t)psConfig->xsizehi.src) << 16) | psConfig->xsize.src;
    }

    return (((uint32_t)psConfig->xsizehi.des) << 16) | psConfig->xsize.des;
}

//*****************************************************************************
//
//! @brief Internal helper to write split XSIZE and XSIZEHI register fields.
//!
//! @param psConfig - Pointer to the configuration structure.
//! @param ui32SrcXEleCount - Source X dimension count in elements.
//! @param ui32DstXEleCount - Destination X dimension count in elements.
//!
//*****************************************************************************
static inline void
am_util_dme_set_xsize_count(am_hal_dme_ch_config_t *psConfig,
                            uint32_t ui32SrcXEleCount,
                            uint32_t ui32DstXEleCount)
{
    psConfig->xsize.src = ui32SrcXEleCount & 0xFFFFU;
    psConfig->xsize.des = ui32DstXEleCount & 0xFFFFU;
    psConfig->xsizehi.src = ui32SrcXEleCount >> 16;
    psConfig->xsizehi.des = ui32DstXEleCount >> 16;
}

//*****************************************************************************
//
//! @brief Internal helper function to get the transfer size for a memory copy.
//!
//! @param ui32SrcAddr - Source address.
//! @param ui32DstAddr - Destination address.
//! @param ui32SrcLength - Source length in bytes.
//! @param ui32DstLength - Destination length in bytes.
//!
//! @return Transfer size encoding selected from common address and length
//!         alignment.
//*****************************************************************************
static inline uint32_t
am_util_dme_get_copy_transize(uint32_t ui32SrcAddr,
                              uint32_t ui32DstAddr,
                              uint32_t ui32SrcLength,
                              uint32_t ui32DstLength)
{
    uint32_t ui32CommonMask = ui32SrcAddr | ui32DstAddr | ui32SrcLength | ui32DstLength;
    uint32_t ui32Transize;

    if (ui32CommonMask == 0U)
    {
        return AM_UTIL_DME_MAX_TRANSIZE;
    }

    ui32Transize = __CLZ(__RBIT(ui32CommonMask));

    return (ui32Transize > AM_UTIL_DME_MAX_TRANSIZE) ? AM_UTIL_DME_MAX_TRANSIZE : ui32Transize;
}

//*****************************************************************************
//
//! @brief Internal helper function to validate a common 2D copy configuration.
//!
//! @param psWrapCpyConfig - Pointer to a layout-compatible 2D copy
//!                          configuration structure.
//!
//! @return AM_HAL_STATUS_SUCCESS when the configuration is valid.
//!         AM_HAL_STATUS_INVALID_ARG if one or more arguments fail validation.
//*****************************************************************************
static inline uint32_t
am_util_dme_wrapcpy_2d_validate(const am_util_dme_wrapcpy_2d_config_t *psWrapCpyConfig)
{
    uint32_t ui32ElemMask;
    uint32_t ui32SrcXEleCount;
    uint32_t ui32DstXEleCount;
    uint32_t ui32SrcLineEleCount;
    uint32_t ui32DstLineEleCount;

    // Require a valid wrapped 2D copy configuration pointer.
    if (psWrapCpyConfig == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    // Reject invalid TRANSIZE
    if ((psWrapCpyConfig->eTransSize > AM_HAL_DME_TRANSIZE_16BYTE))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    // Require element-aligned base addresses.
    ui32ElemMask = (1UL << psWrapCpyConfig->eTransSize) - 1U;
    if ((((uintptr_t)psWrapCpyConfig->ui32SrcAddr & ui32ElemMask) != 0U) ||
        (((uintptr_t)psWrapCpyConfig->ui32DstAddr & ui32ElemMask) != 0U))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    // Require element-aligned line width and transfer width
    if (((psWrapCpyConfig->ui32SrcLineWidth & ui32ElemMask) != 0U) ||
        ((psWrapCpyConfig->ui32DstLineWidth & ui32ElemMask) != 0U) ||
        ((psWrapCpyConfig->ui32SrcXSize & ui32ElemMask) != 0U) ||
        ((psWrapCpyConfig->ui32DstXSize & ui32ElemMask) != 0U))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    // Calculate element counts
    ui32SrcXEleCount = (psWrapCpyConfig->ui32SrcXSize >> psWrapCpyConfig->eTransSize);
    ui32DstXEleCount = (psWrapCpyConfig->ui32DstXSize >> psWrapCpyConfig->eTransSize);
    ui32SrcLineEleCount = (psWrapCpyConfig->ui32SrcLineWidth >> psWrapCpyConfig->eTransSize);
    ui32DstLineEleCount = (psWrapCpyConfig->ui32DstLineWidth >> psWrapCpyConfig->eTransSize);

    // Ensure Y direction loop counts within hardware limits
    if ((psWrapCpyConfig->ui32SrcYSize > 0xFFFFU) ||
        (psWrapCpyConfig->ui32DstYSize > 0xFFFFU) ||
        (ui32SrcLineEleCount > 0xFFFFU) ||
        (ui32DstLineEleCount > 0xFFFFU))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    // Ensure X direction element count is less then line element count
    if ((ui32SrcXEleCount > ui32SrcLineEleCount) ||
        (ui32DstXEleCount > ui32DstLineEleCount))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    // Reject empty source  or destination ranges
    if ((ui32SrcXEleCount == 0U)    ||
        (ui32DstXEleCount == 0U)    ||
        (ui32SrcLineEleCount == 0U) ||
        (ui32DstLineEleCount == 0U) ||
        (psWrapCpyConfig->ui32SrcYSize == 0U) ||
        (psWrapCpyConfig->ui32DstYSize == 0U))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    // Reject invalid CopyMode
    if (psWrapCpyConfig->eCopyMode > AM_UTIL_DME_2D_COPY_MODE_FLIP_ANTI_DIAGONAL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Internal helper function to generate a memory copy configuration.
//!
//! @param psConfig - Pointer to the configuration structure to initialize.
//! @param ui32SrcAddr - Source buffer address.
//! @param ui32DstAddr - Destination buffer address.
//! @param ui32Length - Copy length in bytes.
//!
//! @return AM_HAL_STATUS_SUCCESS on success.
//!         AM_HAL_STATUS_INVALID_ARG if one or more arguments fail validation.
//
//*****************************************************************************
static inline uint32_t
am_util_dme_memcpy_config_generate_internal(am_hal_dme_ch_config_t *psConfig,
                                            uint32_t ui32SrcAddr,
                                            uint32_t ui32DstAddr,
                                            uint32_t ui32Length)
{
    uint32_t ui32Transize;
    uint32_t ui32TransferEleCount;

    if ((psConfig == NULL) ||
        (ui32Length == 0U) ||
        (ui32SrcAddr == 0U) ||
        (ui32DstAddr == 0U))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    // Assign default configuration
    *psConfig = g_am_util_dma_350_memcpy_config_default;
    ui32Transize = am_util_dme_get_copy_transize(ui32SrcAddr, ui32DstAddr, ui32Length, ui32Length);
    ui32TransferEleCount = (ui32Length >> ui32Transize);

    psConfig->ctrl.transize = ui32Transize;
    psConfig->srcaddr = ui32SrcAddr;
    psConfig->desaddr = ui32DstAddr;
    psConfig->xsize.u32 = ((ui32TransferEleCount & 0xFFFFU) << 16) | (ui32TransferEleCount & 0xFFFFU);
    if (ui32TransferEleCount & 0xFFFF0000U)
    {
        psConfig->xsizehi.u32 = (ui32TransferEleCount >> 16) | (ui32TransferEleCount & 0xFFFF0000U);
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Internal helper to prepare common 2D transfer geometry.
//!
//! @param psConfig - Pointer to the DME channel configuration to initialize.
//! @param psCommonConfig - Pointer to the shared 2D copy configuration.
//! @param bEnableWrap - When true, enables wrapped X/Y transfer behavior.
//!
//! @return AM_HAL_STATUS_SUCCESS when the configuration is valid and the
//!         channel configuration is prepared.
//!         AM_HAL_STATUS_INVALID_ARG if one or more arguments fail validation.
//*****************************************************************************
static inline uint32_t
am_util_dme_2d_copy_config_generate_internal(am_hal_dme_ch_config_t *psConfig,
                                             const am_util_dme_common_2d_copy_config_t *psCommonConfig,
                                             bool bEnableWrap)
{
    uint32_t ui32TransformedSrcAddr;
    uint32_t ui32SrcXEleCount;
    uint32_t ui32SrcYSize;
    uint32_t ui32DstXEleCount;
    uint32_t ui32DstYSize;
    uint32_t ui32DstLineEleCount;
    int16_t i16SrcInc;
    int16_t i16SrcStride;
    uint32_t ui32Status;

    if ((psConfig == NULL) ||
        (psCommonConfig == NULL) ||
        (psCommonConfig->ui32SrcAddr == 0U) ||
        (psCommonConfig->ui32DstAddr == 0U))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    ui32Status = am_util_dme_wrapcpy_2d_validate((const am_util_dme_wrapcpy_2d_config_t *)psCommonConfig);
    if (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        return ui32Status;
    }

    // Keep the destination in top-left raster order and transform the source
    // view instead. The DME X loop is the inner loop and Y is the outer loop.
    ui32SrcXEleCount = psCommonConfig->ui32SrcXSize >> psCommonConfig->eTransSize;
    ui32SrcYSize = psCommonConfig->ui32SrcYSize;
    ui32DstXEleCount = psCommonConfig->ui32DstXSize >> psCommonConfig->eTransSize;
    ui32DstYSize = psCommonConfig->ui32DstYSize;
    ui32DstLineEleCount = psCommonConfig->ui32DstLineWidth >> psCommonConfig->eTransSize;
    switch (psCommonConfig->eCopyMode)
    {
        case AM_UTIL_DME_2D_COPY_MODE_ORIGINAL:
            ui32TransformedSrcAddr = psCommonConfig->ui32SrcAddr;
            i16SrcInc = 1;
            i16SrcStride = psCommonConfig->ui32SrcLineWidth >> psCommonConfig->eTransSize;
            break;

        case AM_UTIL_DME_2D_COPY_MODE_FLIP_HORIZONTAL:
            ui32TransformedSrcAddr = psCommonConfig->ui32SrcAddr + ((ui32SrcXEleCount - 1U) << psCommonConfig->eTransSize);
            i16SrcInc = -1;
            i16SrcStride = psCommonConfig->ui32SrcLineWidth >> psCommonConfig->eTransSize;
            break;

        case AM_UTIL_DME_2D_COPY_MODE_FLIP_VERTICAL:
            i16SrcInc = 1;
            i16SrcStride = 0 - (psCommonConfig->ui32SrcLineWidth >> psCommonConfig->eTransSize);
            ui32TransformedSrcAddr = psCommonConfig->ui32SrcAddr +
                                     (((uint32_t)(psCommonConfig->ui32SrcLineWidth >> psCommonConfig->eTransSize) *
                                       (ui32SrcYSize - 1U)) << psCommonConfig->eTransSize);
            break;

        case AM_UTIL_DME_2D_COPY_MODE_FLIP_DIAGONAL:
            ui32TransformedSrcAddr = psCommonConfig->ui32SrcAddr;
            i16SrcInc = psCommonConfig->ui32SrcLineWidth >> psCommonConfig->eTransSize;
            i16SrcStride = 1;
            ui32SrcXEleCount = psCommonConfig->ui32SrcYSize;
            ui32SrcYSize = psCommonConfig->ui32SrcXSize >> psCommonConfig->eTransSize;
            break;

        case AM_UTIL_DME_2D_COPY_MODE_FLIP_ANTI_DIAGONAL:
            i16SrcInc = 0 - (psCommonConfig->ui32SrcLineWidth >> psCommonConfig->eTransSize);
            i16SrcStride = -1;
            ui32TransformedSrcAddr = psCommonConfig->ui32SrcAddr +
                                     ((((uint32_t)(psCommonConfig->ui32SrcLineWidth >> psCommonConfig->eTransSize) *
                                        (ui32SrcYSize - 1U)) + (ui32SrcXEleCount - 1U)) << psCommonConfig->eTransSize);
            ui32SrcXEleCount = psCommonConfig->ui32SrcYSize;
            ui32SrcYSize = psCommonConfig->ui32SrcXSize >> psCommonConfig->eTransSize;
            break;

        case AM_UTIL_DME_2D_COPY_MODE_ROTATE_90_CW:
            i16SrcInc = 0 - (psCommonConfig->ui32SrcLineWidth >> psCommonConfig->eTransSize);
            i16SrcStride = 1;
            ui32TransformedSrcAddr = psCommonConfig->ui32SrcAddr +
                                     (((uint32_t)(psCommonConfig->ui32SrcLineWidth >> psCommonConfig->eTransSize) *
                                       (ui32SrcYSize - 1U)) << psCommonConfig->eTransSize);
            ui32SrcXEleCount = psCommonConfig->ui32SrcYSize;
            ui32SrcYSize = psCommonConfig->ui32SrcXSize >> psCommonConfig->eTransSize;
            break;

        case AM_UTIL_DME_2D_COPY_MODE_ROTATE_180_CW:
            i16SrcInc = -1;
            i16SrcStride = 0 - (psCommonConfig->ui32SrcLineWidth >> psCommonConfig->eTransSize);
            ui32TransformedSrcAddr = psCommonConfig->ui32SrcAddr +
                                     ((((uint32_t)(psCommonConfig->ui32SrcLineWidth >> psCommonConfig->eTransSize) *
                                        (ui32SrcYSize - 1U)) + (ui32SrcXEleCount - 1U)) << psCommonConfig->eTransSize);
            break;

        case AM_UTIL_DME_2D_COPY_MODE_ROTATE_270_CW:
        default:
            ui32TransformedSrcAddr = psCommonConfig->ui32SrcAddr + ((ui32SrcXEleCount - 1U) << psCommonConfig->eTransSize);
            i16SrcInc = psCommonConfig->ui32SrcLineWidth >> psCommonConfig->eTransSize;
            i16SrcStride = -1;
            ui32SrcXEleCount = psCommonConfig->ui32SrcYSize;
            ui32SrcYSize = psCommonConfig->ui32SrcXSize >> psCommonConfig->eTransSize;
            break;
    }

    if (!bEnableWrap)
    {
        if (ui32SrcXEleCount > ui32DstXEleCount)
        {
            ui32SrcXEleCount = ui32DstXEleCount;
        }
        ui32DstXEleCount = ui32SrcXEleCount;

        if (ui32SrcYSize > psCommonConfig->ui32DstYSize)
        {
            ui32SrcYSize = psCommonConfig->ui32DstYSize;
        }
        ui32DstYSize = ui32SrcYSize;
    }
    else
    {
        ui32DstYSize = psCommonConfig->ui32DstYSize;
    }

    *psConfig = g_am_util_dma_350_copy_2d_config_default;
    if (bEnableWrap)
    {
        psConfig->ctrl.xtype = AM_HAL_DME_XTYPE_WRAP;
        psConfig->ctrl.ytype = AM_HAL_DME_YTYPE_WRAP;
    }
    psConfig->ctrl.transize = psCommonConfig->eTransSize;
    psConfig->srcaddr = ui32TransformedSrcAddr;
    am_util_dme_set_xsize_count(psConfig, ui32SrcXEleCount, ui32DstXEleCount);
    psConfig->xaddrinc.src = i16SrcInc;
    psConfig->ysize.src = ui32SrcYSize & 0xFFFFU;
    psConfig->yaddrstride.src = i16SrcStride;
    psConfig->desaddr = psCommonConfig->ui32DstAddr;
    psConfig->xaddrinc.des = 1;
    psConfig->ysize.des = ui32DstYSize;
    psConfig->yaddrstride.des = ui32DstLineEleCount;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Internal helper function to generate a 2D memory copy configuration.
//!
//! @param psConfig - Pointer to the configuration structure to initialize.
//! @param psCopyConfig - Pointer to the copy configuration structure.
//!
//! @return AM_HAL_STATUS_SUCCESS when the configuration is written to
//!                               @p psConfig.
//!         AM_HAL_STATUS_INVALID_ARG if one or more arguments fail validation.
//!         If the transformed source image exceeds the destination boundary,
//!         only the top-left region that fits is generated.
//*****************************************************************************
static inline uint32_t
am_util_dme_memcpy_2d_config_generate_internal(am_hal_dme_ch_config_t *psConfig,
                                               am_util_dme_memcpy_2d_config_t *psCopyConfig)
{
    return am_util_dme_2d_copy_config_generate_internal(psConfig,
                                                        (const am_util_dme_common_2d_copy_config_t *)psCopyConfig,
                                                        false);
}

//*****************************************************************************
//
//! @brief Internal helper function to generate a wrapped-copy configuration.
//!
//! @param psConfig - Pointer to the configuration structure to initialize.
//! @param psWrapCpyConfig - Pointer to the wrapped-copy configuration structure.
//!
//! @return AM_HAL_STATUS_SUCCESS when the configuration is written to
//!                               @p psConfig.
//!         AM_HAL_STATUS_INVALID_ARG if one or more arguments fail validation.
//*****************************************************************************
static uint32_t
am_util_dme_wrapcpy_config_generate_internal(am_hal_dme_ch_config_t *psConfig,
                                             am_util_dme_1d_wrapcpy_config_t *psWrapCpyConfig)
{
    if ((psConfig == NULL) ||
        (psWrapCpyConfig == NULL) ||
        (psWrapCpyConfig->ui32DstLen == 0U) ||
        (psWrapCpyConfig->ui32SrcLen == 0U) ||
        (psWrapCpyConfig->ui32SrcAddr == 0U) ||
        (psWrapCpyConfig->ui32DstAddr == 0U))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    // Use the dedicated fill-value path only when the source window is a
    // native 1/2/4-byte repeat and the destination is aligned to that width.
    bool bUseFillVal = ((psWrapCpyConfig->ui32SrcLen == 1) || (psWrapCpyConfig->ui32SrcLen == 2) || (psWrapCpyConfig->ui32SrcLen == 4)) &&
                       ((psWrapCpyConfig->ui32DstAddr % psWrapCpyConfig->ui32SrcLen) == 0U) &&
                       ((psWrapCpyConfig->ui32DstLen % psWrapCpyConfig->ui32SrcLen) == 0U);

    // Generate wrapped-copy configuration.
    uint32_t ui32TranSize;
    uint32_t ui32TransferEleCount;
    if (bUseFillVal)
    {
        *psConfig = g_am_util_dma_350_wrapcpy_config_default_fillval;
        ui32TranSize = __CLZ(__RBIT(psWrapCpyConfig->ui32SrcLen));
        psConfig->fillval = *((uint32_t *)psWrapCpyConfig->ui32SrcAddr);
    }
    else
    {
        uint32_t ui32SrcEleCount;
        *psConfig = g_am_util_dma_350_memcpy_config_default;
        ui32TranSize = am_util_dme_get_copy_transize(psWrapCpyConfig->ui32DstAddr,
                                                     psWrapCpyConfig->ui32SrcAddr,
                                                     psWrapCpyConfig->ui32SrcLen,
                                                     psWrapCpyConfig->ui32DstLen);
        ui32SrcEleCount = (psWrapCpyConfig->ui32SrcLen >> ui32TranSize);
        psConfig->ctrl.xtype = AM_HAL_DME_XTYPE_WRAP;
        psConfig->srcaddr = psWrapCpyConfig->ui32SrcAddr;
        psConfig->xsize.src = ui32SrcEleCount;
        if (ui32SrcEleCount & 0xFFFF0000U)
        {
            psConfig->xsizehi.src = (ui32SrcEleCount >> 16) & 0xFFFFU;
        }
    }
    psConfig->desaddr = psWrapCpyConfig->ui32DstAddr;
    psConfig->ctrl.transize = ui32TranSize;
    ui32TransferEleCount = (psWrapCpyConfig->ui32DstLen >> ui32TranSize);
    psConfig->xsize.des = ui32TransferEleCount;
    if (ui32TransferEleCount & 0xFFFF0000U)
    {
        psConfig->xsizehi.des = (ui32TransferEleCount >> 16);
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Internal helper function to generate a 2D wrapped-copy
//!        configuration.
//!
//! @param psConfig - Pointer to the configuration structure to initialize.
//! @param psWrapCpyConfig - Pointer to the 2D wrapped-copy configuration
//!                          structure.
//!
//! @return AM_HAL_STATUS_SUCCESS when the configuration is written to
//!                               @p psConfig.
//!         AM_HAL_STATUS_INVALID_ARG if one or more arguments fail validation.
//*****************************************************************************
static uint32_t
am_util_dme_gencfg_wrapcpy_2d_internal(am_hal_dme_ch_config_t *psConfig,
                                       am_util_dme_wrapcpy_2d_config_t *psWrapCpyConfig)
{
    return am_util_dme_2d_copy_config_generate_internal(psConfig,
                                                        (const am_util_dme_common_2d_copy_config_t *)psWrapCpyConfig,
                                                        true);
}

//*****************************************************************************
//
//! @brief Internal helper function to generate a strided-copy configuration.
//!
//! @param psConfig - Pointer to the configuration structure to initialize.
//! @param psStridedCopyConfig - Pointer to the strided-copy configuration
//!                              structure.
//!
//! @return AM_HAL_STATUS_SUCCESS when the configuration is written to
//!                               @p psConfig.
//!         AM_HAL_STATUS_INVALID_ARG if one or more arguments fail validation.
//
//*****************************************************************************
static uint32_t
am_util_dme_gencfg_strided_copy_internal(am_hal_dme_ch_config_t *psConfig,
                                         am_util_dme_strided_copy_config_t *psStridedCopyConfig)
{
    uint32_t ui32Transize;
    uint32_t ui32TransferEleCount;
    int32_t i32SrcStrideEleCount;
    int32_t i32DstStrideEleCount;

    // Validate strided-copy geometry and address parameters.
    if ((psConfig == NULL) ||
        (psStridedCopyConfig == NULL) ||
        (psStridedCopyConfig->ui32SetSize == 0U) ||
        (psStridedCopyConfig->ui32SetCount == 0U) ||
        (psStridedCopyConfig->ui32SrcAddr == 0U) ||
        (psStridedCopyConfig->ui32DstAddr == 0U) ||
        (psStridedCopyConfig->ui32SetCount > 0xFFFFU) ||
        (psStridedCopyConfig->i32SrcStride == 0) ||
        (psStridedCopyConfig->i32DstStride == 0))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    // Select the largest safe transfer size, or byte transfers when reversing.
    if (psStridedCopyConfig->bInvertDst)
    {
        ui32Transize = AM_HAL_DME_TRANSIZE_1BYTE;
    }
    else
    {
        ui32Transize = am_util_dme_get_copy_transize(psStridedCopyConfig->ui32SrcAddr |
                                                     (uint32_t)psStridedCopyConfig->i32SrcStride,
                                                     psStridedCopyConfig->ui32DstAddr |
                                                     (uint32_t)psStridedCopyConfig->i32DstStride,
                                                     psStridedCopyConfig->ui32SetSize,
                                                     psStridedCopyConfig->ui32SetSize);
    }

    // Convert byte counts and strides into TRANSIZE-scaled element counts.
    ui32TransferEleCount = psStridedCopyConfig->ui32SetSize >> ui32Transize;
    i32SrcStrideEleCount = psStridedCopyConfig->i32SrcStride / (int32_t)(1U << ui32Transize);
    i32DstStrideEleCount = psStridedCopyConfig->i32DstStride / (int32_t)(1U << ui32Transize);

    // Validate stride counts against the 16-bit Y stride fields.
    if ((i32SrcStrideEleCount < INT16_MIN) ||
        (i32SrcStrideEleCount > 0xFFFF)    ||
        (i32DstStrideEleCount < INT16_MIN) ||
        (i32DstStrideEleCount > 0xFFFF))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    // Program DME as a 2D transfer with M-byte X size and N-line Y size.
    *psConfig = g_am_util_dma_350_copy_2d_config_default;
    psConfig->ctrl.transize = ui32Transize;
    psConfig->srcaddr = psStridedCopyConfig->ui32SrcAddr;
    if (psStridedCopyConfig->bInvertDst)
    {
        psConfig->srcaddr += psStridedCopyConfig->ui32SetSize - 1U;
        psConfig->xaddrinc.src = -1;
    }
    else
    {
        psConfig->xaddrinc.src = 1;
    }
    psConfig->desaddr = psStridedCopyConfig->ui32DstAddr;
    am_util_dme_set_xsize_count(psConfig, ui32TransferEleCount, ui32TransferEleCount);
    psConfig->xaddrinc.des = 1;
    psConfig->ysize.src = psStridedCopyConfig->ui32SetCount;
    psConfig->ysize.des = psStridedCopyConfig->ui32SetCount;
    psConfig->yaddrstride.src = (uint16_t)i32SrcStrideEleCount;
    psConfig->yaddrstride.des = (uint16_t)i32DstStrideEleCount;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Generate a byte-swap configuration.
//!
//! @param psConfig - Pointer to the configuration structure to initialize.
//! @param ui32SrcAddr - Source address.
//! @param ui32DstAddr - Destination address.
//! @param ui32Length - Length in bytes.
//! @param ui32GroupSize - Group size of each byte-swapped unit.
//!
//! @return AM_HAL_STATUS_SUCCESS when the configuration is written to
//!                               @p psConfig.
//!         AM_HAL_STATUS_INVALID_ARG if one or more arguments fail validation.
//*****************************************************************************
uint32_t
am_util_dme_gencfg_byteswap_internal(am_hal_dme_ch_config_t *psConfig,
                                     uint32_t ui32SrcAddr,
                                     uint32_t ui32DstAddr,
                                     uint32_t ui32Length,
                                     uint32_t ui32GroupSize)
{
    uint32_t ui32RepeatCount;

    if ((psConfig == NULL) ||
        (ui32Length == 0U) ||
        (ui32GroupSize == 0U) ||
        (ui32SrcAddr == 0U) ||
        (ui32DstAddr == 0U) ||
        (ui32GroupSize > ui32Length) ||
        ((ui32Length % ui32GroupSize) != 0U))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    ui32RepeatCount = ui32Length / ui32GroupSize;

    // Validate repeat count is within YSIZE hardware limit
    if (ui32RepeatCount > 0xFFFFU)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    // Generate configuration for byte-swap operation.
    *psConfig = g_am_util_dma_350_copy_2d_config_default;
    psConfig->srcaddr = ui32SrcAddr + (ui32GroupSize - 1U);
    psConfig->desaddr = ui32DstAddr;
    am_util_dme_set_xsize_count(psConfig, ui32GroupSize, ui32GroupSize);
    psConfig->ysize.src = ui32RepeatCount;
    psConfig->ysize.des = ui32RepeatCount;
    psConfig->xaddrinc.src = -1;
    psConfig->xaddrinc.des = 1;
    psConfig->yaddrstride.src = ui32GroupSize;
    psConfig->yaddrstride.des = ui32GroupSize;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Generate a memory copy configuration
//
//*****************************************************************************
uint32_t
am_util_dme_gencfg_memcpy(am_hal_dme_ch_config_t *psConfig,
                          uint32_t ui32SrcAddr,
                          uint32_t ui32DstAddr,
                          uint32_t ui32Length,
                          bool     bEnableLinkAddr)
{
    uint32_t ui32Status;

    ui32Status = am_util_dme_memcpy_config_generate_internal(psConfig, ui32SrcAddr, ui32DstAddr, ui32Length);
    if (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        return ui32Status;
    }

    // Enable link-attribute and link-address fields in the mask if requested
    if (bEnableLinkAddr)
    {
        psConfig->mask.mask_b.linkattr_msk = 1;
        psConfig->mask.mask_b.linkaddr_msk = 1;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Generate a 2D memory copy configuration
//
//*****************************************************************************
uint32_t
am_util_dme_gencfg_memcpy_2d(am_hal_dme_ch_config_t *psConfig,
                             am_util_dme_memcpy_2d_config_t *psCopyConfig,
                             bool bEnableLinkAddr)
{
    uint32_t ui32Status;

    ui32Status = am_util_dme_memcpy_2d_config_generate_internal(psConfig, psCopyConfig);
    if (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        return ui32Status;
    }

    if (bEnableLinkAddr)
    {
        psConfig->mask.mask_b.linkattr_msk = 1;
        psConfig->mask.mask_b.linkaddr_msk = 1;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Generate a wrapped-copy configuration
//
//*****************************************************************************
uint32_t
am_util_dme_gencfg_wrapcpy(am_hal_dme_ch_config_t *psConfig,
                           am_util_dme_1d_wrapcpy_config_t *psWrapCpyConfig,
                           bool bEnableLinkAddr)
{
    uint32_t ui32Status;

    ui32Status = am_util_dme_wrapcpy_config_generate_internal(psConfig, psWrapCpyConfig);
    if (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        return ui32Status;
    }

    if (bEnableLinkAddr)
    {
        psConfig->mask.mask_b.linkattr_msk = 1;
        psConfig->mask.mask_b.linkaddr_msk = 1;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Generate a 2D wrapped-copy configuration
//
//*****************************************************************************
uint32_t
am_util_dme_gencfg_wrapcpy_2d(am_hal_dme_ch_config_t *psConfig,
                              am_util_dme_wrapcpy_2d_config_t *psWrapCpyConfig,
                              bool bEnableLinkAddr)
{
    uint32_t ui32Status;

    ui32Status = am_util_dme_gencfg_wrapcpy_2d_internal(psConfig, psWrapCpyConfig);
    if (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        return ui32Status;
    }

    if (bEnableLinkAddr)
    {
        psConfig->mask.mask_b.linkattr_msk = 1;
        psConfig->mask.mask_b.linkaddr_msk = 1;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Generate a strided-copy configuration.
//
//*****************************************************************************
uint32_t
am_util_dme_gencfg_strided_copy(am_hal_dme_ch_config_t *psConfig,
                                am_util_dme_strided_copy_config_t *psStridedCopyConfig,
                                bool bEnableLinkAddr)
{
    uint32_t ui32Status;

    // Generate and validate the strided-copy command configuration.
    ui32Status = am_util_dme_gencfg_strided_copy_internal(psConfig, psStridedCopyConfig);
    if (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        return ui32Status;
    }

    // Enable command-link fields when the caller will build a descriptor.
    if (bEnableLinkAddr)
    {
        psConfig->mask.mask_b.linkattr_msk = 1;
        psConfig->mask.mask_b.linkaddr_msk = 1;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Generate a byte-swap configuration.
//
//*****************************************************************************
uint32_t
am_util_dme_gencfg_byteswap(am_hal_dme_ch_config_t *psConfig,
                            uint32_t ui32SrcAddr,
                            uint32_t ui32DstAddr,
                            uint32_t ui32Length,
                            uint32_t ui32GroupSize,
                            bool bEnableLinkAddr)
{
    uint32_t ui32Status;
    ui32Status = am_util_dme_gencfg_byteswap_internal(psConfig, ui32SrcAddr, ui32DstAddr, ui32Length, ui32GroupSize);
    if (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        return ui32Status;
    }

    if (bEnableLinkAddr)
    {
        psConfig->mask.mask_b.linkattr_msk = 1;
        psConfig->mask.mask_b.linkaddr_msk = 1;
    }
    return AM_HAL_STATUS_SUCCESS;
}


//*****************************************************************************
//
// Copy a memory range. Zero-length copies are rejected as invalid arguments.
//
//*****************************************************************************
uint32_t
am_util_dme_memcpy(am_hal_dme_ch_access_token_t *pToken,
                   uint32_t ui32SrcAddr,
                   uint32_t ui32DstAddr,
                   uint32_t ui32Length)
{
    uint32_t ui32Status;
    am_hal_dme_ch_config_t sConfig;

    uint32_t ui32TimeoutUs = ui32Length < AM_UTIL_DME_TIMEOUT_MIN_US ? AM_UTIL_DME_TIMEOUT_MIN_US : ui32Length;

    if (pToken == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    ui32Status = am_util_dme_memcpy_config_generate_internal(&sConfig, ui32SrcAddr, ui32DstAddr, ui32Length);
    if (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        return ui32Status;
    }

    // Clear DME channel state before starting to configure the channel
    ui32Status = am_hal_dme_ch_clear(pToken);
    if (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        return ui32Status;
    }

    // Configure the DME channel
    ui32Status = am_hal_dme_ch_config(pToken, &sConfig);
    if (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        return ui32Status;
    }
    return am_hal_dme_command_execute(pToken, ui32TimeoutUs);
}

//*****************************************************************************
//
// Copy a 2D memory range
//
//*****************************************************************************
uint32_t
am_util_dme_memcpy_2d(am_hal_dme_ch_access_token_t *pToken,
                      am_util_dme_memcpy_2d_config_t *psCopyConfig)
{
    uint32_t ui32Status;
    am_hal_dme_ch_config_t sConfig;
    uint32_t ui32TimeoutUs;

    if (pToken == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    // Generate configuration first so invalid copy parameters fail before the
    // channel state is touched.
    ui32Status = am_util_dme_memcpy_2d_config_generate_internal(&sConfig, psCopyConfig);
    if (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        return ui32Status;
    }

    ui32TimeoutUs = psCopyConfig->ui32DstXSize * psCopyConfig->ui32DstYSize;
    if (ui32TimeoutUs < AM_UTIL_DME_TIMEOUT_MIN_US)
    {
        ui32TimeoutUs = AM_UTIL_DME_TIMEOUT_MIN_US;
    }

    // Clear DME channel state before starting to configure the channel
    ui32Status = am_hal_dme_ch_clear(pToken);
    if (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        return ui32Status;
    }

    ui32Status = am_hal_dme_ch_config(pToken, &sConfig);
    if (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        return ui32Status;
    }

    return am_hal_dme_command_execute(pToken, ui32TimeoutUs);
}

//*****************************************************************************
//
// Wrapped 1D copy from source to destination range
//
//*****************************************************************************
uint32_t
am_util_dme_wrapcpy(am_hal_dme_ch_access_token_t *pToken,
                    am_util_dme_1d_wrapcpy_config_t *psWrapCpyConfig)
{
    uint32_t ui32Status;
    am_hal_dme_ch_config_t sConfig;
    uint32_t ui32TimeoutUs;

    if (pToken == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    ui32Status = am_util_dme_wrapcpy_config_generate_internal(&sConfig, psWrapCpyConfig);
    if (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        return ui32Status;
    }

    // Clear DME channel state before starting to configure the channel
    ui32Status = am_hal_dme_ch_clear(pToken);
    if (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        return ui32Status;
    }
    ui32Status = am_hal_dme_ch_config(pToken, &sConfig);
    if (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        return ui32Status;
    }
    ui32TimeoutUs = psWrapCpyConfig->ui32DstLen < AM_UTIL_DME_TIMEOUT_MIN_US ? AM_UTIL_DME_TIMEOUT_MIN_US : psWrapCpyConfig->ui32DstLen;
    return am_hal_dme_command_execute(pToken, ui32TimeoutUs);
}

//*****************************************************************************
//
// Wrapped 2D copy from source to destination range
//
//*****************************************************************************
uint32_t
am_util_dme_wrapcpy_2d(am_hal_dme_ch_access_token_t *pToken,
                       am_util_dme_wrapcpy_2d_config_t *psWrapCpyConfig)
{
    uint32_t ui32Status;
    am_hal_dme_ch_config_t sConfig;
    uint32_t ui32TimeoutUs;

    if (pToken == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    // Generate configuration and execute the wrapped-copy operation
    ui32Status = am_util_dme_gencfg_wrapcpy_2d_internal(&sConfig, psWrapCpyConfig);
    if (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        return ui32Status;
    }

    // Calculate timeout based on the destination size
    ui32TimeoutUs = psWrapCpyConfig->ui32DstXSize * psWrapCpyConfig->ui32DstYSize;
    if (ui32TimeoutUs < AM_UTIL_DME_TIMEOUT_MIN_US)
    {
        ui32TimeoutUs = AM_UTIL_DME_TIMEOUT_MIN_US;
    }

    // Clear DME channel state before starting to configure the channel
    ui32Status = am_hal_dme_ch_clear(pToken);
    if (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        return ui32Status;
    }

    // Configure the DME channel
    ui32Status = am_hal_dme_ch_config(pToken, &sConfig);
    if (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        return ui32Status;
    }

    return am_hal_dme_command_execute(pToken, ui32TimeoutUs);
}

//*****************************************************************************
//
// Strided copy from source to destination.
//
//*****************************************************************************
uint32_t
am_util_dme_strided_copy(am_hal_dme_ch_access_token_t *pToken,
                         am_util_dme_strided_copy_config_t *psStridedCopyConfig)
{
    uint32_t ui32Status;
    am_hal_dme_ch_config_t sConfig;
    uint32_t ui32TimeoutUs;

    if (pToken == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    // Generate and validate the strided-copy command configuration.
    ui32Status = am_util_dme_gencfg_strided_copy_internal(&sConfig, psStridedCopyConfig);
    if (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        return ui32Status;
    }

    // Calculate timeout from the total destination byte count.
    ui32TimeoutUs = psStridedCopyConfig->ui32SetSize * psStridedCopyConfig->ui32SetCount;
    if (ui32TimeoutUs < AM_UTIL_DME_TIMEOUT_MIN_US)
    {
        ui32TimeoutUs = AM_UTIL_DME_TIMEOUT_MIN_US;
    }

    // Clear DME channel state before starting to configure the channel
    ui32Status = am_hal_dme_ch_clear(pToken);
    if (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        return ui32Status;
    }

    ui32Status = am_hal_dme_ch_config(pToken, &sConfig);
    if (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        return ui32Status;
    }

    return am_hal_dme_command_execute(pToken, ui32TimeoutUs);
}

//*****************************************************************************
//
// Byte-swap copy from source to destination buffer.
//
//*****************************************************************************
uint32_t
am_util_dme_byteswap(am_hal_dme_ch_access_token_t *pToken,
                     uint32_t ui32SrcAddr,
                     uint32_t ui32DstAddr,
                     uint32_t ui32Length,
                     uint32_t ui32GroupSize)
{
    uint32_t ui32Status;
    am_hal_dme_ch_config_t sConfig;
    uint32_t ui32TimeoutUs;

    if (pToken == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    // Generate configuration
    ui32Status = am_util_dme_gencfg_byteswap_internal(&sConfig, ui32SrcAddr, ui32DstAddr, ui32Length, ui32GroupSize);
    if (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        return ui32Status;
    }

    // Clear DME channel state before starting to configure the channel
    ui32Status = am_hal_dme_ch_clear(pToken);
    if (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        return ui32Status;
    }

    // Configure the DME channel
    ui32Status = am_hal_dme_ch_config(pToken, &sConfig);
    if (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        return ui32Status;
    }

    // Execute the byte-swap operation.
    ui32TimeoutUs = ui32Length < AM_UTIL_DME_TIMEOUT_MIN_US ? AM_UTIL_DME_TIMEOUT_MIN_US : ui32Length;
    return am_hal_dme_command_execute(pToken, ui32TimeoutUs);
}

//*****************************************************************************
//
//! @}
//
//*****************************************************************************
