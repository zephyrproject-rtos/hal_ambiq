//*****************************************************************************
//
//! @file am_hal_sdhc.c
//!
//! @brief Functions for interfacing with the SDHC.
//!
//! @addtogroup sdhc SDHC host controller
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

#include <stdbool.h>
#include <stdint.h>

#include "am_mcu_apollo.h"

#include "am_util_stdio.h"
#include "am_util_debug.h"

//*****************************************************************************
//
// Private Types.
//
//*****************************************************************************

#define AM_HAL_MAGIC_SDHC 0x8313B0
#define AM_HAL_SDHC_CHK_HANDLE(h)                                                                                      \
    ((h) && ((am_hal_handle_prefix_t *)(h))->s.bInit && (((am_hal_handle_prefix_t *)(h))->s.magic == AM_HAL_MAGIC_SDHC))

#define AM_HAL_SDHC_DEBUG(fmt, ...) am_util_debug_printf("[SDHC] line %04d - "fmt, __LINE__, ##__VA_ARGS__)

//
//! SD Host Register Types
//
typedef struct
{
    bool bValid;
    uint32_t regHOSTCTRL1;
    uint32_t regCLOCKCTRL;
    uint32_t regINTENABLE;
    uint32_t regINTSIG;
    uint32_t regAUTO;
} am_hal_sdhc_register_state_t;

//
//! SDHC State structure.
//
typedef struct
{
    //
    //! Handle validation prefix.
    //
    am_hal_handle_prefix_t prefix;

    //
    //! Physical module number.
    //
    uint32_t ui32Module;
    uint32_t ui32HostSDMABufSize;
    uint8_t  ui8BaseClockFreq;

    //
    //! Link to the card host
    //
    am_hal_card_host_t *pHost;

    //
    //! Save the error count
    //
    bool bCmdErr;
    bool bDataErr;
    uint32_t ui32DataErrCnt;
    uint32_t ui32CmdErrCnt;

    //
    //! Store the data transfer infomation
    //
    uint32_t *pui32Buf;
    uint32_t ui32DataLen;
    uint32_t ui32BlkCnt;
    uint32_t ui32BlkNum;
    uint32_t ui32BlkSize;
    uint32_t ui32BlksPerSDMA;
    uint32_t ui32ClkOffDelay;
    am_hal_data_dir_e eDataDir;
    bool bAsyncCmdIsDone;

    //
    //! Power Save-Restore register state
    //
    am_hal_sdhc_register_state_t registerState;
} am_hal_sdhc_state_t;

//! SDIO interrupts.
static const IRQn_Type sdio_interrupts[] =
{
    SDIO0_IRQn,
    SDIO1_IRQn,
};

//*****************************************************************************
//
// Global Variables.
//
//*****************************************************************************
static am_hal_sdhc_state_t g_SDHCState[AM_REG_SDIO_NUM_MODULES];

//*****************************************************************************
//
// Internal Functions.
//
//*****************************************************************************

uint32_t am_hal_sdhc_software_reset(SDIO_Type *pSDHC, am_hal_sdhc_sw_reset_e eSoftwareReset)
{
    uint32_t ui32Mask;
    uint32_t ui32Timeout;

    if ( !pSDHC->PRESENT_b.CARDINSERTED )
    {
        return AM_HAL_STATUS_FAIL;
    }

    ui32Mask = 0x1 << (SDIO0_CLOCKCTRL_SWRSTDAT_Pos - eSoftwareReset);

    pSDHC->CLOCKCTRL |= ui32Mask;

    ui32Timeout = 150;
    do
    {
        if ( ui32Timeout == 0 )
        {
            return AM_HAL_STATUS_FAIL;
        }
        ui32Timeout--;
        am_hal_delay_us(1000);  // 1ms delay
    } while (pSDHC->CLOCKCTRL & ui32Mask);

    return AM_HAL_STATUS_SUCCESS;
}

//
// SDHC send command function
//
static inline uint32_t am_hal_sdhc_check_cmd_inhibit(SDIO_Type *pSDHC, am_hal_card_cmd_t *pCmd, am_hal_card_cmd_data_t *pCmdData)
{
    uint32_t ui32CmdInhibitMask;

    //
    // Check if CMD and DAT Lines are busy
    //
    ui32CmdInhibitMask = SDIO0_PRESENT_CMDINHCMD_Msk | SDIO0_PRESENT_CMDINHDAT_Msk;

    if ( !pCmdData || pCmdData->bNotUseDataLine )
    {
        ui32CmdInhibitMask &= ~SDIO0_PRESENT_CMDINHDAT_Msk;
    }

    if ( am_hal_delay_us_status_check(100, (uint32_t)&pSDHC->PRESENT, ui32CmdInhibitMask,
        ui32CmdInhibitMask, false) == AM_HAL_STATUS_TIMEOUT )
    {
        AM_HAL_SDHC_DEBUG("%s : CMD and DAT line is busy\n", __FUNCTION__);
        pCmd->eError = AM_HAL_CMD_ERR_TIMEOUT;
        return AM_HAL_STATUS_TIMEOUT;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//
// Prepare the command register value
//
static uint32_t am_hal_sdhc_prepare_cmd(SDIO_Type *pSDHC, am_hal_card_cmd_t *pCmd, am_hal_card_cmd_data_t *pCmdData)
{
    uint32_t ui32CmdReg = 0x0;
    //
    // Response Type Select
    //
    if ( !(pCmd->ui32RespType & MMC_RSP_PRESENT) )
    {
        ui32CmdReg = SDIO0_TRANSFER_RESPTYPESEL_NORESPONSE;
    }
    else if ( pCmd->ui32RespType & MMC_RSP_136 )
    {
        ui32CmdReg |= SDIO0_TRANSFER_RESPTYPESEL_LEN136 << SDIO0_TRANSFER_RESPTYPESEL_Pos;
    }
    else if ( pCmd->ui32RespType & MMC_RSP_BUSY )
    {
        ui32CmdReg |= SDIO0_TRANSFER_RESPTYPESEL_LEN48CHKBUSY << SDIO0_TRANSFER_RESPTYPESEL_Pos;
    }
    else
    {
        ui32CmdReg |= SDIO0_TRANSFER_RESPTYPESEL_LEN48 << SDIO0_TRANSFER_RESPTYPESEL_Pos;
    }

    //
    // Data Present Select
    //
    if ( pCmdData != NULL )
    {
        ui32CmdReg |= SDIO0_TRANSFER_DATAPRSNTSEL_Msk;
    }

    //
    // Command CRC Check Enable
    //
    if ( pCmd->ui32RespType & MMC_RSP_CRC )
    {
        ui32CmdReg |= SDIO0_TRANSFER_CMDCRCCHKEN_Msk;
    }

    //
    // Command Index Check Enable
    //
    if ( pCmd->ui32RespType & MMC_RSP_OPCODE )
    {
        ui32CmdReg |= SDIO0_TRANSFER_CMDIDXCHKEN_Msk;
    }

    ui32CmdReg |= (pCmd->ui8Idx << SDIO0_TRANSFER_CMDIDX_Pos);

    return ui32CmdReg;
}

//
// ADMA2 functions - maximum 127*16 = 2032 Blocks can be used by upper layer block-oriented APIs
// redefining 'AM_HAL_ADMA_TABLE_NO_ENTRIES' to get the much larger data transfer.
//

#define AM_HAL_ADMA_MAX_LEN             65535
#define AM_HAL_ADMA_MAX_BLKS_PER_ENTRY  127     //  (AM_HAL_ADMA_MAX_LEN/512)

//
//! Maximum block number for one ADMA2 is limited to 127*AM_HAL_ADMA_TABLE_NO_ENTRIES.
//! enlarging the entry number if want to transfer more blocks
//! in one ADMA2 transaction.
//
#define AM_HAL_ADMA_TABLE_NO_ENTRIES 16  // 127*16*512 = 1016KB

//
//! @name Decriptor table defines
//! @{
//
#define AM_HAL_ADMA_DESC_ATTR_VALID     (0x1 << 0)
#define AM_HAL_ADMA_DESC_ATTR_END       (0x1 << 1)
#define AM_HAL_ADMA_DESC_ATTR_INT       (0x1 << 2)
#define AM_HAL_ADMA_DESC_ATTR_ACT0      (0x1 << 3)
#define AM_HAL_ADMA_DESC_ATTR_ACT1      (0x1 << 4)
#define AM_HAL_ADMA_DESC_ATTR_ACT2      (0x1 << 5)

#define AM_HAL_ADMA_DESC_TRANSFER_DATA  AM_HAL_ADMA_DESC_ATTR_ACT2
#define AM_HAL_ADMA_DESC_LINK_DESC      (AM_HAL_ADMA_DESC_ATTR_ACT1 | AM_HAL_ADMA_DESC_ATTR_ACT2)
//! @}

typedef uint32_t dma_addr_t;

typedef struct
{
    uint8_t  ui8Attr;
    uint8_t  ui8Reserved;
    uint16_t ui16Len;
    uint32_t ui32AddrLow;
} am_hal_sdhc_adma_desc_t;

AM_SHARED_RW static am_hal_sdhc_adma_desc_t adma_desc_table[AM_REG_SDIO_NUM_MODULES][AM_HAL_ADMA_TABLE_NO_ENTRIES] __attribute__((aligned(32)));

static void am_hal_sdhc_prepare_sdhci_adma_desc(uint32_t ui32Module, uint32_t ui32Idx, dma_addr_t ui32DmaAddr, uint16_t ui16Len, bool bEnd)
{
    am_hal_sdhc_adma_desc_t *pDesc;
    uint8_t ui8Attr;

    pDesc = &adma_desc_table[ui32Module][ui32Idx];

    ui8Attr = AM_HAL_ADMA_DESC_ATTR_VALID | AM_HAL_ADMA_DESC_TRANSFER_DATA;
    if ( bEnd )
    {
        ui8Attr |= AM_HAL_ADMA_DESC_ATTR_END;
    }

    pDesc->ui8Attr = ui8Attr;
    pDesc->ui16Len = ui16Len;
    pDesc->ui8Reserved = 0;
    pDesc->ui32AddrLow = ui32DmaAddr;
}

static void am_hal_sdhc_prepare_adma_table(uint32_t ui32Module, am_hal_card_cmd_data_t *pCmdData)
{
    bool bEnd;
    uint32_t i;
    int32_t i32BlkCnt;
    uint32_t ui32XferBytes;
    uint32_t ui32IovLen;
    uint8_t ui8IovCnt = 0;
    uint8_t ui8IovIndex = 0;
    dma_addr_t ui32DmaAddr;

    ui8IovCnt = pCmdData->ui8IovCnt;
    if ( ui8IovCnt )
    {
        i = 0;
        while (ui8IovCnt > 0)
        {
            ui32DmaAddr = (dma_addr_t)(pCmdData->pIov[ui8IovIndex].pIovBase);
            ui32IovLen = pCmdData->pIov[ui8IovIndex].ui32IovLen;
            while (ui32IovLen > 0)
            {
                if ( ui8IovCnt > 1 )
                {
                    bEnd = false;
                }
                else
                {
                    bEnd = (ui32IovLen > AM_HAL_ADMA_MAX_BLKS_PER_ENTRY * pCmdData->ui32BlkSize) ? false : true;
                }

                if ( ui32IovLen > AM_HAL_ADMA_MAX_BLKS_PER_ENTRY * pCmdData->ui32BlkSize )
                {
                    ui32XferBytes = AM_HAL_ADMA_MAX_BLKS_PER_ENTRY * pCmdData->ui32BlkSize;
                }
                else
                {
                    ui32XferBytes = ui32IovLen;
                }
                am_hal_sdhc_prepare_sdhci_adma_desc(ui32Module, i, ui32DmaAddr, ui32XferBytes, bEnd);
                ui32DmaAddr += ui32XferBytes;
                ui32IovLen  -= ui32XferBytes;
                i++;
            }
            ui8IovIndex++;
            ui8IovCnt--;
        }
    }
    else
    {
        i = 0;
        i32BlkCnt = (pCmdData->ui32BlkCnt == 0) ? 1 : pCmdData->ui32BlkCnt;
        ui32DmaAddr = (dma_addr_t)(pCmdData->pui8Buf);
        while (i32BlkCnt > 0)
        {
            ui32XferBytes = (i32BlkCnt > AM_HAL_ADMA_MAX_BLKS_PER_ENTRY) ?
                AM_HAL_ADMA_MAX_BLKS_PER_ENTRY*pCmdData->ui32BlkSize : i32BlkCnt*pCmdData->ui32BlkSize;
            bEnd = (i32BlkCnt > AM_HAL_ADMA_MAX_BLKS_PER_ENTRY) ? false : true;
            am_hal_sdhc_prepare_sdhci_adma_desc(ui32Module, i, ui32DmaAddr, ui32XferBytes, bEnd);
            i++;
            ui32DmaAddr += ui32XferBytes;
            i32BlkCnt -= AM_HAL_ADMA_MAX_BLKS_PER_ENTRY;
        }
    }
}

//
// Prepare transfer mode register and set the block size, block count, SDMA and host control registers (for DMA)
//
static uint32_t am_hal_sdhc_prepare_xfer(am_hal_sdhc_state_t *pSDHCState, SDIO_Type *pSDHC, am_hal_card_cmd_t *pCmd, am_hal_card_cmd_data_t *pCmdData)
{
    uint32_t ui32ModeReg = 0;
    uint32_t ui32BlkReg  = 0;

    if ( pCmdData != NULL )
    {
        pSDHCState->pui32Buf = (uint32_t *)(pCmdData->pui8Buf);
        pSDHCState->ui32DataLen = (pCmdData->ui32BlkCnt == 0) ? pCmdData->ui32BlkSize : (pCmdData->ui32BlkCnt * pCmdData->ui32BlkSize);
        pSDHCState->eDataDir = pCmdData->dir;
        pSDHCState->ui32BlkSize = pCmdData->ui32BlkSize;
        pSDHCState->ui32BlkCnt = pCmdData->ui32BlkCnt;
        pSDHCState->ui32BlkNum = pCmdData->ui32BlkCnt;

        //
        // Transfer direction - 0x1 Read
        //
        if ( pCmdData->dir == AM_HAL_DATA_DIR_READ )
        {
            ui32ModeReg |= SDIO0_TRANSFER_DXFERDIRSEL_Msk;
        }

        //
        // Transfer DMA setting
        //
        if ( pCmdData->eXferMode == AM_HAL_HOST_XFER_DEFAULT )
        {
            pCmdData->eXferMode = pSDHCState->pHost->eXferMode;
        }

        switch (pCmdData->eXferMode)
        {
            case AM_HAL_HOST_XFER_SDMA:
                ui32ModeReg |= SDIO0_TRANSFER_DMAEN_Msk;
                pSDHC->HOSTCTRL1_b.DMASELECT = SDIO0_HOSTCTRL1_DMASELECT_SDMA;
                pSDHC->SDMA = (dma_addr_t)(pCmdData->pui8Buf);
                pSDHCState->ui32BlksPerSDMA = pSDHCState->ui32HostSDMABufSize / pCmdData->ui32BlkSize;
                break;
            case AM_HAL_HOST_XFER_ADMA:
                ui32ModeReg |= SDIO0_TRANSFER_DMAEN_Msk;
                pSDHC->HOSTCTRL1_b.DMASELECT = SDIO0_HOSTCTRL1_DMASELECT_ADMA232;
                am_hal_sdhc_prepare_adma_table(pSDHCState->ui32Module, pCmdData);
                pSDHC->ADMALOWD = (dma_addr_t)(&adma_desc_table[pSDHCState->ui32Module][0]);
                break;
            default:
                break;
        }

        //
        // Auto Command setting
        //
        if ( pCmd->bAutoCMD12 )
        {
            ui32ModeReg |= SDIO0_TRANSFER_ACMDEN_CMD12ENABLE << SDIO0_TRANSFER_ACMDEN_Pos;
        }
        else if ( pCmd->bAutoCMD23 )
        {
            ui32ModeReg |= SDIO0_TRANSFER_ACMDEN_CMD23ENABLE << SDIO0_TRANSFER_ACMDEN_Pos;

#ifdef AM_DEBUG_PRINTF
            if (pCmdData->eXferMode == AM_HAL_HOST_XFER_SDMA)
            {
                AM_HAL_SDHC_DEBUG("SDMA can't be used if enabling CMD23\n");
            }
#endif

            pSDHC->SDMA = pCmdData->ui32BlkCnt;
        }

        //
        // Set the block count and size
        //
        ui32BlkReg |= pCmdData->ui32BlkSize << SDIO0_BLOCK_TRANSFERBLOCKSIZE_Pos;
        if ( pCmdData->ui32BlkCnt > 1 )
        {
            ui32ModeReg |= SDIO0_TRANSFER_BLKSEL_Msk | SDIO0_TRANSFER_BLKCNTEN_Msk;
            ui32BlkReg |= pCmdData->ui32BlkCnt << SDIO0_BLOCK_BLKCNT_Pos;
        }

        pSDHC->BLOCK = ui32BlkReg;

        //
        // Set the data timeout
        //
        pSDHC->CLOCKCTRL |= (0xe << SDIO0_CLOCKCTRL_TIMEOUTCNT_Pos);
    }

    return ui32ModeReg;
}

//
// Get the command response after sending out the command
//
static void am_hal_sdhc_get_cmd_response(SDIO_Type *pSDHC, am_hal_card_cmd_t *pCmd)
{
    uint32_t ui32RegResp[4];

    pCmd->eError = AM_HAL_CMD_ERR_NONE;
    if ( pCmd->ui32RespType == MMC_RSP_NONE )
    {
        return;
    }

    if ( pCmd->ui32RespType & MMC_RSP_136 )
    {
        ui32RegResp[0] = pSDHC->RESPONSE0;
        ui32RegResp[1] = pSDHC->RESPONSE1;
        ui32RegResp[2] = pSDHC->RESPONSE2;
        ui32RegResp[3] = pSDHC->RESPONSE3;
        pCmd->ui32Resp[0] = (ui32RegResp[3] << 8) | (ui32RegResp[2] >> 24);
        pCmd->ui32Resp[1] = (ui32RegResp[2] << 8) | (ui32RegResp[1] >> 24);
        pCmd->ui32Resp[2] = (ui32RegResp[1] << 8) | (ui32RegResp[0] >> 24);
        pCmd->ui32Resp[3] = (ui32RegResp[0] << 8);
    }
    else
    {
        pCmd->ui32Resp[0] = pSDHC->RESPONSE0;
    }
}

//
// Sending the command by writing the argument and command registers
//
static uint32_t am_hal_sdhc_send_cmd(am_hal_sdhc_state_t *pSDHCState, SDIO_Type *pSDHC, am_hal_card_cmd_t *pCmd, am_hal_card_cmd_data_t *pCmdData)
{
    uint32_t ui32CmdReg;
    am_hal_cachectrl_range_t sRange;

    if ( am_hal_sdhc_check_cmd_inhibit(pSDHC, pCmd, pCmdData) == AM_HAL_STATUS_TIMEOUT )
    {
        pCmd->eError = AM_HAL_CMD_ERR_INHIBIT;
        pSDHCState->bCmdErr = true;
        pSDHCState->ui32CmdErrCnt++;
        return AM_HAL_STATUS_TIMEOUT;
    }

    //
    // Prepare the command register value
    //
    ui32CmdReg = am_hal_sdhc_prepare_cmd(pSDHC, pCmd, pCmdData);

    //
    // Prepare the transfer mode register value
    //
    ui32CmdReg |= am_hal_sdhc_prepare_xfer(pSDHCState, pSDHC, pCmd, pCmdData);

    //
    // Invalidate adma buffer if located at SSRAM region
    //
    if ( (uint32_t)adma_desc_table >= SSRAM_BASEADDR )
    {
        if ( pSDHCState->ui32Module )
        {
            sRange.ui32StartAddr = (uint32_t)(&adma_desc_table[1]);
        }
        else
        {
            sRange.ui32StartAddr = (uint32_t)(&adma_desc_table[0]);
        }

        sRange.ui32Size = AM_HAL_ADMA_TABLE_NO_ENTRIES*sizeof(am_hal_sdhc_adma_desc_t);
        am_hal_cachectrl_dcache_clean(&sRange);
    }

    //
    // Write the argument and command register
    //
    pSDHC->ARGUMENT1 = pCmd->ui32Arg;
    pSDHC->TRANSFER  = ui32CmdReg;

    return AM_HAL_STATUS_SUCCESS;

}

static inline am_hal_card_cmd_err_e am_hal_sdhc_check_cmd_error_type(uint32_t ui32IntStatus)
{
    if (ui32IntStatus & SDIO0_INTSTAT_COMMANDINDEXERROR_Msk)
    {
        return AM_HAL_CMD_ERR_INDEX;
    }
    else if (ui32IntStatus & SDIO0_INTSTAT_COMMANDENDBITERROR_Msk)
    {
        return AM_HAL_CMD_ERR_ENDBIT;
    }
    else if (ui32IntStatus & SDIO0_INTSTAT_COMMANDCRCERROR_Msk)
    {
        return AM_HAL_CMD_ERR_CRC;
    }
    else if (ui32IntStatus & SDIO0_INTSTAT_COMMANDTIMEOUTERROR_Msk)
    {
        return AM_HAL_CMD_ERR_NO_RESPONSE;
    }
    else
    {
        return AM_HAL_CMD_ERR_TIMEOUT;
    }
}

//
// Wait the command done by checking command completion interrupt status
//
#define AM_HAL_WAIT_CMD_DONE_TIMEOUT 2

static uint32_t inline am_hal_sdhc_wait_cmd_done(am_hal_sdhc_state_t *pSDHCState, SDIO_Type *pSDHC, am_hal_card_cmd_t *pCmd)
{
    uint32_t ui32Status;
    uint32_t ui32IntStatus;
    uint32_t ui32BitMask = SDIO0_INTSTAT_COMMANDCOMPLETE_Msk;

    ui32Status = am_hal_delay_us_status_check(
        AM_HAL_WAIT_CMD_DONE_TIMEOUT*1000, (uint32_t)&pSDHC->INTSTAT,
        ui32BitMask,
        ui32BitMask,
        true);

    if ( AM_HAL_STATUS_SUCCESS != ui32Status )
    {
        ui32IntStatus = pSDHC->INTSTAT;
        pCmd->eError = am_hal_sdhc_check_cmd_error_type(ui32IntStatus);
        pSDHCState->bCmdErr = true;
        pSDHCState->ui32CmdErrCnt++;
        AM_HAL_SDHC_DEBUG("wait CMD completion INT timeout, Error Type : %d\n", pCmd->eError);
        return AM_HAL_STATUS_TIMEOUT;
    }

    pSDHC->INTSTAT = ui32BitMask;

    return AM_HAL_STATUS_SUCCESS;
}

//
// Do the PIO block transfer
//
static void am_hal_sdhc_pio_xfer_data(am_hal_sdhc_state_t *pSDHCState)
{
    uint32_t ui32PreBufReadyMask;

    ui32PreBufReadyMask =  pSDHCState->eDataDir == AM_HAL_DATA_DIR_READ ?
        SDIO0_PRESENT_BUFRDEN_Msk : SDIO0_PRESENT_BUFWREN_Msk;

    SDIO_Type *pSDHC = SDHCn(pSDHCState->ui32Module);

    while (pSDHC->PRESENT & ui32PreBufReadyMask)
    {
        if ( pSDHCState->eDataDir == AM_HAL_DATA_DIR_READ )
        {
            for (int i = 0; i < pSDHCState->ui32BlkSize; i += 4)
            {
                *pSDHCState->pui32Buf++ = pSDHC->BUFFER;
            }
        }
        else
        {
            for (int i = 0; i < pSDHCState->ui32BlkSize; i += 4)
            {
                 pSDHC->BUFFER = *pSDHCState->pui32Buf++;
            }
        }
        pSDHCState->ui32BlkCnt--;
    }
}

//
// Do the SDMA block transfer
//
static void am_hal_sdhc_sdma_xfer_data(am_hal_sdhc_state_t *pSDHCState)
{
    SDIO_Type *pSDHC = SDHCn(pSDHCState->ui32Module);

    //
    // Load the next DMA address
    //
    pSDHCState->ui32BlkCnt -= pSDHCState->ui32BlksPerSDMA;
    pSDHCState->pui32Buf += pSDHCState->ui32HostSDMABufSize / sizeof(uint32_t);
    pSDHC->SDMA = (dma_addr_t)(pSDHCState->pui32Buf);
}

static inline am_hal_card_data_err_e am_hal_sdhc_check_data_error_type(uint32_t ui32IntStatus)
{
    if (ui32IntStatus & SDIO0_INTSTAT_ADMAERROR_Msk)
    {
        return AM_HAL_DATA_ERR_ADMAERROR;
    }
    else if (ui32IntStatus & SDIO0_INTSTAT_DATACRCERROR_Msk)
    {
        return AM_HAL_DATA_ERR_DATACRCERROR;
    }
    else if (ui32IntStatus & SDIO0_INTSTAT_DATATIMEOUTERROR_Msk)
    {
        return AM_HAL_DATA_ERR_DATATIMEOUTERROR;
    }
    else if (ui32IntStatus & SDIO0_INTSTAT_DATAENDBITERROR_Msk)
    {
        return AM_HAL_DATA_ERR_DATAENDBITERROR;
    }
    else
    {
        return AM_HAL_DATA_ERR_TIMEOUT;
    }
}

#define DYNAMIC_SWITCH_SDCLK_FEATURE

//
// Xfer timeout value depends on the card performance. 80000 is an empirical value.
//
#define XFER_DATA_TIMEOUT   80000

//
// Transfer the block data to the card
//
static uint32_t am_hal_sdhc_xfer_data(am_hal_sdhc_state_t *pSDHCState,
                                      am_hal_card_cmd_data_t *pCmdData)
{
    bool bXferDone;
    uint32_t ui32BufReadyMask;
    uint32_t ui32IntStatus;
    uint32_t ui32Timeout = XFER_DATA_TIMEOUT;

    SDIO_Type *pSDHC = SDHCn(pSDHCState->ui32Module);

#ifdef AM_DEBUG_PRINTF
    am_hal_card_host_t *pHost = pSDHCState->pHost;
#endif

    if ( pCmdData->ui32BlkCnt != 0 )
    {
        ui32Timeout = XFER_DATA_TIMEOUT*pCmdData->ui32BlkCnt;
    }

    AM_HAL_SDHC_DEBUG("Xfer Timeout is %d\n", ui32Timeout);
    AM_HAL_SDHC_DEBUG("Xfer BLK Cnt is %d\n", pSDHCState->ui32BlkCnt);
    AM_HAL_SDHC_DEBUG("Xfer DataLen is %d\n", pSDHCState->ui32DataLen);
    AM_HAL_SDHC_DEBUG("Xfer Mode  is %d\n", pHost->eXferMode);
    AM_HAL_SDHC_DEBUG("Xfer speed is %d\n", pHost->ui32Clock);
    AM_HAL_SDHC_DEBUG("Xfer Width is %d\n", pHost->eBusWidth);

    ui32BufReadyMask = pSDHCState->eDataDir == AM_HAL_DATA_DIR_READ ?
        SDIO0_INTSTAT_BUFFERREADREADY_Msk : SDIO0_INTSTAT_BUFFERWRITEREADY_Msk;

    bXferDone = false;
    ui32IntStatus = 0;
    while ( !(ui32IntStatus & SDIO0_INTSTAT_TRANSFERCOMPLETE_Msk) && (ui32Timeout > 0) )
    {
        ui32IntStatus = pSDHC->INTSTAT;

        if ( ui32IntStatus & SDIO0_INTSTAT_TRANSFERCOMPLETE_Msk )
        {
            //
            // Transfer completed
            //
            pSDHC->INTSTAT = SDIO0_INTSTAT_TRANSFERCOMPLETE_Msk;
            pSDHCState->ui32BlkCnt = 0;
            bXferDone = true;
            AM_HAL_SDHC_DEBUG("Xfer Completed\n");
        }
        else if ( ui32IntStatus & SDIO0_INTSTAT_DMAINTERRUPT_Msk )
        {
            //
            // Transfer SDMA data
            //
            pSDHC->INTSTAT = SDIO0_INTSTAT_DMAINTERRUPT_Msk;
            AM_HAL_SDHC_DEBUG("Xfer SDMA DMA INTR\n");
            am_hal_sdhc_sdma_xfer_data(pSDHCState);
        }
        else if ( ui32IntStatus & ui32BufReadyMask )
        {
            //
            // Transfer PIO data if PIO buffer is ready
            //
            pSDHC->INTSTAT = ui32BufReadyMask;
            AM_HAL_SDHC_DEBUG("Xfer PIO\n");
            am_hal_sdhc_pio_xfer_data(pSDHCState);
        }
        else if (ui32IntStatus & (SDIO0_INTSTAT_ADMAERROR_Msk    |
                         SDIO0_INTSTAT_DATACRCERROR_Msk          |
                         SDIO0_INTSTAT_DATATIMEOUTERROR_Msk      |
                         SDIO0_INTSTAT_DATAENDBITERROR_Msk))
        {
            pSDHC->INTSTAT = ui32IntStatus;
            AM_HAL_SDHC_DEBUG("Xfer Data Error - 0x%x\n", ui32IntStatus);
            pCmdData->eDataError = am_hal_sdhc_check_data_error_type(ui32IntStatus);
            pSDHCState->bDataErr = true;
            pSDHCState->ui32DataErrCnt++;
            ui32Timeout = 1;
        }

        if ( ui32Timeout-- > 0 )
        {
            am_hal_delay_us(5);
        }
        // AM_HAL_SDHC_DEBUG("INT STATUS 0x%x\n", ui32IntStatus);
    }

#ifdef DYNAMIC_SWITCH_SDCLK_FEATURE

    //
    // Disable the SDCLK after the xfer is done
    //
    pSDHC->CLOCKCTRL_b.SDCLKEN = 0x0;
    am_hal_delay_us(pSDHCState->ui32ClkOffDelay);
    AM_HAL_SDHC_DEBUG("Disable the SDCLK\n");
#endif

    if ( !bXferDone && ui32Timeout == 0 )
    {
        return (pSDHCState->ui32BlkNum - pSDHCState->ui32BlkCnt) << 16 | AM_HAL_STATUS_TIMEOUT;
    }
    else
    {
        return (pSDHCState->ui32BlkNum - pSDHCState->ui32BlkCnt) << 16 | AM_HAL_STATUS_SUCCESS;
    }
}

//*****************************************************************************
//
// External Functions.
//
//*****************************************************************************

//*****************************************************************************
//
// SDHC initialization function
//
//*****************************************************************************
uint32_t am_hal_sdhc_initialize(uint32_t ui32Module, void **ppHandle)
{

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check that the request module is in range.
    //
    if ( ui32Module >= AM_REG_SDIO_NUM_MODULES )
    {
        return AM_HAL_STATUS_OUT_OF_RANGE;
    }

    //
    // Check for valid arguements.
    //
    if ( !ppHandle )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Check if the handle is unallocated.
    //
    if ( g_SDHCState[ui32Module].prefix.s.bInit )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    //
    // Initialize the handle.
    //
    g_SDHCState[ui32Module].prefix.s.bInit = true;
    g_SDHCState[ui32Module].prefix.s.magic = AM_HAL_MAGIC_SDHC;
    g_SDHCState[ui32Module].ui32Module = ui32Module;

    //
    // Return the handle.
    //
    *ppHandle = (void *)&g_SDHCState[ui32Module];

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// SDHC Deinitialize function
//
//*****************************************************************************
uint32_t am_hal_sdhc_deinitialize(void *pHandle)
{
    am_hal_sdhc_state_t *pSDHCState = (am_hal_sdhc_state_t *)pHandle;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_SDHC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    //
    // Reset the handle.
    //
    pSDHCState->prefix.s.bInit = false;
    pSDHCState->ui32Module = 0;

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// SDHC power control function
//
// This function updates the peripheral to a given power state.
//
//*****************************************************************************
uint32_t am_hal_sdhc_power_control(void *pHandle, am_hal_sysctrl_power_state_e ePowerState, bool bRetainState)
{
    am_hal_sdhc_state_t *pSDHCState = (am_hal_sdhc_state_t *)pHandle;
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_SDHC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

#endif // AM_HAL_DISABLE_API_VALIDATION

    //
    // Decode the requested power state and update SDHC operation accordingly.
    //
    switch (ePowerState)
    {
        case AM_HAL_SYSCTRL_WAKE:

            if ( bRetainState && !pSDHCState->registerState.bValid )
            {
                return AM_HAL_STATUS_INVALID_OPERATION;
            }

            //
            // Clock request.
            //
            ui32Status = am_hal_clkmgr_clock_request(AM_HAL_CLKMGR_CLK_ID_HFRC, (am_hal_clkmgr_user_id_e)(AM_HAL_CLKMGR_USER_ID_SDIO0 + pSDHCState->ui32Module));
            if ( ui32Status != AM_HAL_STATUS_SUCCESS )
            {
                return ui32Status;
            }

            //
            // Enable power control.
            //
            am_hal_pwrctrl_periph_enable((am_hal_pwrctrl_periph_e)(AM_HAL_PWRCTRL_PERIPH_SDIO0 + pSDHCState->ui32Module));

            //
            // Enable the clock to SDIO peripheral
            //
            switch (pSDHCState->ui32Module)
            {
                case 0:
                    MCUCTRL->SDIO0CTRL |= (MCUCTRL_SDIO0CTRL_SDIO0SYSCLKEN_Msk | MCUCTRL_SDIO0CTRL_SDIO0XINCLKEN_Msk);
                    break;
                case 1:
                    MCUCTRL->SDIO1CTRL |= (MCUCTRL_SDIO1CTRL_SDIO1SYSCLKEN_Msk | MCUCTRL_SDIO1CTRL_SDIO1XINCLKEN_Msk);
                    break;
                default:
                    break;
            }

            if ( bRetainState )
            {
                //
                // Restore SDHC registers
                //
                SDHCn(pSDHCState->ui32Module)->HOSTCTRL1 = pSDHCState->registerState.regHOSTCTRL1;
                SDHCn(pSDHCState->ui32Module)->CLOCKCTRL = pSDHCState->registerState.regCLOCKCTRL;

                //
                // Wait util the internal clock to be stablized
                //
                uint32_t ui32Timeout = 100;
                while ( SDHCn(pSDHCState->ui32Module)->CLOCKCTRL_b.CLKSTABLE == 0 )
                {
                    if ( ui32Timeout == 0 )
                    {
                        AM_HAL_SDHC_DEBUG("Internal clock can not be stablized\n");
                        return AM_HAL_STATUS_FAIL;
                    }
                    ui32Timeout--;
                    am_hal_delay_us(10);
                }

                SDHCn(pSDHCState->ui32Module)->INTENABLE = pSDHCState->registerState.regINTENABLE;
                SDHCn(pSDHCState->ui32Module)->INTSIG = pSDHCState->registerState.regINTSIG;
                SDHCn(pSDHCState->ui32Module)->AUTO = pSDHCState->registerState.regAUTO;
                pSDHCState->registerState.bValid = false;
            }
            break;

        case AM_HAL_SYSCTRL_NORMALSLEEP:
        case AM_HAL_SYSCTRL_DEEPSLEEP:
            if ( bRetainState )
            {
                //
                // Save SDHC Registers
                //
                pSDHCState->registerState.regHOSTCTRL1 = SDHCn(pSDHCState->ui32Module)->HOSTCTRL1;
                pSDHCState->registerState.regCLOCKCTRL = SDHCn(pSDHCState->ui32Module)->CLOCKCTRL;
                pSDHCState->registerState.regINTENABLE = SDHCn(pSDHCState->ui32Module)->INTENABLE;
                pSDHCState->registerState.regINTSIG = SDHCn(pSDHCState->ui32Module)->INTSIG;
                pSDHCState->registerState.regAUTO = SDHCn(pSDHCState->ui32Module)->AUTO;
                pSDHCState->registerState.bValid = true;
            }

            //
            // Disable all the interrupts.
            am_hal_sdhc_intr_status_disable(pHandle, 0xFFFFFFFF);

            //
            // Disable the clock to SDIO peripheral
            //
            switch (pSDHCState->ui32Module)
            {
                case 0:
                        MCUCTRL->SDIO0CTRL &= ~(MCUCTRL_SDIO0CTRL_SDIO0SYSCLKEN_Msk | MCUCTRL_SDIO0CTRL_SDIO0XINCLKEN_Msk);
                    break;
                case 1:
                        MCUCTRL->SDIO1CTRL &= ~(MCUCTRL_SDIO1CTRL_SDIO1SYSCLKEN_Msk | MCUCTRL_SDIO1CTRL_SDIO1XINCLKEN_Msk);
                    break;
                default:
                    break;
            }

            //
            // Disable power control.
            //
            am_hal_pwrctrl_periph_disable((am_hal_pwrctrl_periph_e)(AM_HAL_PWRCTRL_PERIPH_SDIO0 + pSDHCState->ui32Module));

            //
            // Clock release.
            //
            ui32Status = am_hal_clkmgr_clock_release(AM_HAL_CLKMGR_CLK_ID_HFRC, (am_hal_clkmgr_user_id_e)(AM_HAL_CLKMGR_USER_ID_SDIO0 + pSDHCState->ui32Module));
            if ( ui32Status != AM_HAL_STATUS_SUCCESS )
            {
                return ui32Status;
            }
            break;

        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// SDHC setup card host function
//
// This function updates pHost related settings by checking the capabilites of underlying
// SDHC host controller. These settings are important for validate the arguments to the card
// block read, write, erase, speed, bus width.
//
//*****************************************************************************
uint32_t am_hal_sdhc_setup_host(void *pHandle, am_hal_card_host_t *pHost)
{
    SDIO_Type *pSDHC;
    am_hal_sdhc_state_t *pSDHCState = (am_hal_sdhc_state_t *)pHandle;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_SDHC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

    if ( !pSDHCState->prefix.s.bEnable )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

#endif // AM_HAL_DISABLE_API_VALIDATION

    //
    // Link to the card host
    //
    pSDHCState->pHost = pHost;

    pSDHC = SDHCn(pSDHCState->ui32Module);

    pSDHCState->ui8BaseClockFreq = pSDHC->CAPABILITIES0_b.SDCLKFREQ;

    pHost->ui8Version = pSDHC->SLOTSTAT_b.SPECVER;
    pHost->ui32MaxClock = pSDHCState->ui8BaseClockFreq * 1000000;
    pHost->ui32MinClock = pHost->ui32MaxClock / 256;
    pHost->ui32Clock =  0x0;

    //
    // Default transfer mode is ADMA
    //
    pHost->eXferMode = AM_HAL_HOST_XFER_ADMA;
    pHost->eBusWidth = AM_HAL_HOST_BUS_WIDTH_1;
    pHost->eUHSMode = AM_HAL_HOST_UHS_NONE;


    pHost->ui32MaxADMA2BlkNums = AM_HAL_ADMA_TABLE_NO_ENTRIES*AM_HAL_ADMA_MAX_BLKS_PER_ENTRY;

    //
    // Default 4K SDMA boundary size
    //
    pSDHCState->ui32HostSDMABufSize = 4096*(0x1 << pSDHC->BLOCK_b.HOSTSDMABUFSZ);
    pSDHCState->bAsyncCmdIsDone = true;

    //
    // Set delay time to 4 sdio clock cycles when card initialization
    // SDIO initial clock is 375KHz, so the delay time caculation as follow:
    // pSDHCState->ui32ClkOffDelay = (uint32_t)((1/375000)*1000000*4 + 1).
    //
    pSDHCState->ui32ClkOffDelay = 11;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// SDHC detects the card function
//
// This function detects the present of card in the slot.
//
//*****************************************************************************
bool am_hal_sdhc_get_cd(void *pHandle)
{
    am_hal_sdhc_state_t *pSDHCState = (am_hal_sdhc_state_t *)pHandle;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_SDHC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

    if ( !pSDHCState->prefix.s.bEnable )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    return SDHCn(pSDHCState->ui32Module)->PRESENT_b.CARDINSERTED;
}

//*****************************************************************************
//
// SDHC detects the card write protection
//
// This function detects the card write protection in the slot.
//
//*****************************************************************************
bool am_hal_sdhc_get_wr_protect(void *pHandle)
{
    am_hal_sdhc_state_t *pSDHCState = (am_hal_sdhc_state_t *)pHandle;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_SDHC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

    if ( !pSDHCState->prefix.s.bEnable )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    return SDHCn(pSDHCState->ui32Module)->PRESENT_b.WRPROTSW;
}

//*****************************************************************************
//
// SDHC sets the SDIO bus IO volage
//
// This function sets the bus voltage needed to communiate with the card.
//
//*****************************************************************************
uint32_t am_hal_sdhc_set_bus_voltage(void *pHandle, am_hal_host_bus_voltage_e eBusVoltage)
{
    am_hal_sdhc_state_t *pSDHCState = (am_hal_sdhc_state_t *)pHandle;
    am_hal_card_host_t *pHost;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_SDHC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

    if ( !pSDHCState->prefix.s.bEnable )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

#endif // AM_HAL_DISABLE_API_VALIDATION

    pHost = pSDHCState->pHost;

    switch (eBusVoltage)
    {
        case AM_HAL_HOST_BUS_VOLTAGE_1_8:
            SDHCn(pSDHCState->ui32Module)->HOSTCTRL1 |= SDIO0_HOSTCTRL1_VOLTSELECT_1_8V << SDIO0_HOSTCTRL1_VOLTSELECT_Pos;
            break;
        case AM_HAL_HOST_BUS_VOLTAGE_3_0:
            SDHCn(pSDHCState->ui32Module)->HOSTCTRL1 |= SDIO0_HOSTCTRL1_VOLTSELECT_3_0V << SDIO0_HOSTCTRL1_VOLTSELECT_Pos;
            break;
        case AM_HAL_HOST_BUS_VOLTAGE_3_3:
            SDHCn(pSDHCState->ui32Module)->HOSTCTRL1 |= SDIO0_HOSTCTRL1_VOLTSELECT_3_3V << SDIO0_HOSTCTRL1_VOLTSELECT_Pos;
            break;
    }
    SDHCn(pSDHCState->ui32Module)->HOSTCTRL1 |= SDIO0_HOSTCTRL1_SDBUSPOWER_Msk;

    pHost->eBusVoltage = eBusVoltage;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// SDHC sets the SDIO bus width
//
// This function sets the bus width needed to communiate with the card.
//
//*****************************************************************************
uint32_t am_hal_sdhc_set_bus_width(void *pHandle, am_hal_host_bus_width_e eBusWidth)
{
    am_hal_sdhc_state_t *pSDHCState = (am_hal_sdhc_state_t *)pHandle;
    am_hal_card_host_t *pHost;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_SDHC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

    if ( !pSDHCState->prefix.s.bEnable )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

#endif // AM_HAL_DISABLE_API_VALIDATION

    pHost = pSDHCState->pHost;

    if ( eBusWidth == AM_HAL_HOST_BUS_WIDTH_8 )
    {
        SDHCn(pSDHCState->ui32Module)->HOSTCTRL1_b.XFERWIDTH = 0x1;
    }
    else
    {
        SDHCn(pSDHCState->ui32Module)->HOSTCTRL1_b.XFERWIDTH = 0x0;
        SDHCn(pSDHCState->ui32Module)->HOSTCTRL1_b.DATATRANSFERWIDTH = (eBusWidth == AM_HAL_HOST_BUS_WIDTH_4) ?  1 : 0;
    }

    pHost->eBusWidth = eBusWidth;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// SDHC sets the SDIO bus clock speed
//
// This function sets the bus clock speed needed to communiate with the card.
//
//*****************************************************************************
uint32_t am_hal_sdhc_set_bus_clock(void *pHandle, uint32_t ui32Clock)
{
    uint32_t ui32Divider;

    SDIO_Type *pSDHC;
    am_hal_card_host_t *pHost;
    am_hal_sdhc_state_t *pSDHCState = (am_hal_sdhc_state_t *)pHandle;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_SDHC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

    if ( !pSDHCState->prefix.s.bEnable )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

#endif // AM_HAL_DISABLE_API_VALIDATION

    pSDHC = SDHCn(pSDHCState->ui32Module);
    pHost = pSDHCState->pHost;

    //
    // Set delay time to 4 sdio clock cycles: Minimum time is 1us
    //
    pSDHCState->ui32ClkOffDelay = (uint32_t) (4000000 / ui32Clock + 1);

    //
    // Find the nearest the clock divider
    //
    for (ui32Divider = 1; ui32Divider <= 256; ui32Divider *= 2)
    {
        if ( (pHost->ui32MaxClock / ui32Divider) <= ui32Clock )
        {
            break;
        }
    }
    ui32Clock = pHost->ui32MaxClock / ui32Divider;

    //
    // Same clock setting and do nothing
    //
    if ( pHost->ui32Clock == ui32Clock )
    {
        return AM_HAL_STATUS_SUCCESS;
    }

    ui32Divider >>= 1;

    //
    // Should stop the clock before changing the clock setting
    //
    pSDHC->CLOCKCTRL &= ~(SDIO0_CLOCKCTRL_CLKEN_Msk | SDIO0_CLOCKCTRL_SDCLKEN_Msk);

    //
    // Set the divider
    //
    pSDHC->CLOCKCTRL_b.FREQSEL = ui32Divider;

    //
    // Now enable the internal clock
    //
    pSDHC->CLOCKCTRL_b.CLKEN = 0x1;

    //
    // Wait util the internal clock to be stablized
    //
    uint32_t ui32Timeout = 1000;
    while ( pSDHC->CLOCKCTRL_b.CLKSTABLE == 0 )
    {
        if ( ui32Timeout == 0 )
        {
            AM_HAL_SDHC_DEBUG("Internal clock can not be stablized\n");
            return AM_HAL_STATUS_FAIL;
        }
        ui32Timeout--;
        am_hal_delay_us(10);
    }

    //
    // Now enable the SDCLK
    //
    pSDHC->CLOCKCTRL_b.SDCLKEN = 0x1;

    pHost->ui32Clock = ui32Clock;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// SDHC sets the SDIO UHS Mode
//
// This function sets the bus clock speed needed to communiate with the card.
//
//*****************************************************************************
uint32_t am_hal_sdhc_set_uhs_mode(void *pHandle, am_hal_host_uhs_mode_e eUHSMode)
{
    SDIO_Type *pSDHC;
    am_hal_card_host_t *pHost;
    am_hal_sdhc_state_t *pSDHCState = (am_hal_sdhc_state_t *)pHandle;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_SDHC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

    if ( !pSDHCState->prefix.s.bEnable )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    pSDHC = SDHCn(pSDHCState->ui32Module);
    pHost = pSDHCState->pHost;

    if ( pHost->eUHSMode == eUHSMode )
    {
        return AM_HAL_STATUS_SUCCESS;
    }

    pSDHC->AUTO_b.UHSMODESEL = eUHSMode - AM_HAL_HOST_UHS_NONE;
    pHost->eUHSMode = eUHSMode;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// SDHC checks the SDIO bus busy DAT0 line
//
// This function checks if DAT0 line is busy or not.
//
//*****************************************************************************
uint32_t am_hal_sdhc_card_busy(void *pHandle, uint32_t ui32TimeoutMS)
{
    uint32_t ui32Dat0BusyMask;
    SDIO_Type *pSDHC;
    am_hal_sdhc_state_t *pSDHCState = (am_hal_sdhc_state_t *)pHandle;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_SDHC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

    if ( !pSDHCState->prefix.s.bEnable )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    pSDHC = SDHCn(pSDHCState->ui32Module);

    ui32Dat0BusyMask = 0x1 << SDIO0_PRESENT_DAT30LINE_Pos;

    uint32_t ui32Status;
    ui32Status = am_hal_delay_us_status_check(
        ui32TimeoutMS*1000, (uint32_t)&pSDHC->PRESENT,
        ui32Dat0BusyMask,
        ui32Dat0BusyMask,
        true);

    if ( AM_HAL_STATUS_SUCCESS != ui32Status )
    {
        AM_HAL_SDHC_DEBUG("%s : wait DAT0 busy timeout\n", __FUNCTION__);
    }

#ifdef DYNAMIC_SWITCH_SDCLK_FEATURE
    //
    // Disable the SDCLK
    //
    pSDHC->CLOCKCTRL_b.SDCLKEN = 0x0;
    am_hal_delay_us(pSDHCState->ui32ClkOffDelay);
    AM_HAL_SDHC_DEBUG("Disable the SDCLK\n");
#endif

    return ui32Status;
}

//*****************************************************************************
//
// SDHC Set TxRx Delays
//
//*****************************************************************************
void am_hal_sdhc_set_txrx_delay(void *pHandle, uint8_t ui8TxRxDelays[2])
{
    am_hal_sdhc_state_t *pSDHCState = (am_hal_sdhc_state_t *)pHandle;

    switch (pSDHCState->ui32Module)
    {
        case 0:
            // Adjust TX CLK delay
            MCUCTRL->SDIO0CTRL_b.SDIO0ITAPCHGWIN = 1;

            MCUCTRL->SDIO0CTRL_b.SDIO0OTAPDLYSEL = ui8TxRxDelays[0];
            MCUCTRL->SDIO0CTRL_b.SDIO0OTAPDLYENA = 1;

            // Adjust RX CLK delay
            MCUCTRL->SDIO0CTRL_b.SDIO0ITAPDLYSEL = ui8TxRxDelays[1];
            MCUCTRL->SDIO0CTRL_b.SDIO0ITAPDLYENA = 1;

            MCUCTRL->SDIO0CTRL_b.SDIO0ITAPCHGWIN = 0;
            break;
        case 1:
            // Adjust TX CLK delay
            MCUCTRL->SDIO1CTRL_b.SDIO1ITAPCHGWIN = 1;

            MCUCTRL->SDIO1CTRL_b.SDIO1OTAPDLYSEL = ui8TxRxDelays[0];
            MCUCTRL->SDIO1CTRL_b.SDIO1OTAPDLYENA = 1;

            // Adjust RX CLK delay
            MCUCTRL->SDIO1CTRL_b.SDIO1ITAPDLYSEL = ui8TxRxDelays[1];
            MCUCTRL->SDIO1CTRL_b.SDIO1ITAPDLYENA = 1;

            MCUCTRL->SDIO1CTRL_b.SDIO1ITAPCHGWIN = 0;
            break;
        default:
            break;
    }
}

//
// SDHC Enable function
//
uint32_t am_hal_sdhc_enable(void *pHandle)
{
    am_hal_sdhc_state_t *pSDHCState = (am_hal_sdhc_state_t *)pHandle;
    SDIO_Type *pSDHC;
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_SDHC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

    if ( pSDHCState->prefix.s.bEnable )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    pSDHC = SDHCn(pSDHCState->ui32Module);

    //
    // Note SW_RESET_ALL is *only* used here before initializing other registers
    //
    if ( (ui32Status = am_hal_sdhc_software_reset(pSDHC, AM_HAL_SDHC_SW_RESET_ALL)) != AM_HAL_STATUS_SUCCESS )
    {
        AM_HAL_SDHC_DEBUG("Software Reset ALL failed\n");
        return ui32Status;
    }

    //
    // Enable all interrupts
    //
    SDHCn(pSDHCState->ui32Module)->INTENABLE = (uint32_t)-1;

    //
    // Disable all interrupts SIGNAL
    //
    SDHCn(pSDHCState->ui32Module)->INTSIG = 0x0;

    //
    // Enable SDIO IRQ only after host is initialized successfully
    //
    NVIC_SetPriority(sdio_interrupts[pSDHCState->ui32Module], AM_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(sdio_interrupts[pSDHCState->ui32Module]);

    //
    // Reset CMD and Data error count
    //
    pSDHCState->bCmdErr = false;
    pSDHCState->bDataErr = false;
    pSDHCState->ui32DataErrCnt = 0;
    pSDHCState->ui32CmdErrCnt = 0;

    pSDHCState->prefix.s.bEnable = true;

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//
// SDHC Disable function
//
uint32_t am_hal_sdhc_disable(void *pHandle)
{
    am_hal_sdhc_state_t *pSDHCState = (am_hal_sdhc_state_t *)pHandle;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_SDHC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    if ( !pSDHCState->prefix.s.bEnable )
    {
        return AM_HAL_STATUS_SUCCESS;
    }

    pSDHCState->prefix.s.bEnable = false;

    //
    // Disable SDIO IRQ
    //
    NVIC_DisableIRQ(sdio_interrupts[pSDHCState->ui32Module]);

    //
    // Disable all interrupts SIGNAL
    //
    SDHCn(pSDHCState->ui32Module)->INTSIG = 0x0;

    //
    // Disable all interrupts
    //
    SDHCn(pSDHCState->ui32Module)->INTENABLE = 0x0;

    //
    // Clear all interrupt status
    //
    SDHCn(pSDHCState->ui32Module)->INTSTAT = (uint32_t)-1;

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// SDHC send command function
//
// This function sends a command to SD/MMC/eMMC/SDIO card and gets the response. if this command
// is using synchronous transfer mode, it will be blocked until the data in the buffer has been
// transmited or received. if this command is using asynchronous transfer mode, it will return immediately
// after sending the command, data transfer completion done event will be notified by registered callback
// function in the ISR.
//
//*****************************************************************************
uint32_t am_hal_sdhc_execute_cmd(void *pHandle, am_hal_card_cmd_t *pCmd, am_hal_card_cmd_data_t *pCmdData)
{
    am_hal_sdhc_state_t *pSDHCState = (am_hal_sdhc_state_t *)pHandle;
    SDIO_Type *pSDHC = NULL;
    uint32_t ui32Status  = AM_HAL_STATUS_SUCCESS;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_SDHC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

    if ( !pSDHCState->prefix.s.bEnable )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    pSDHC = SDHCn(pSDHCState->ui32Module);

    if ( !pSDHCState->bAsyncCmdIsDone )
    {
        AM_HAL_SDHC_DEBUG("Return because the previous async command is still ongoing\n");
        return AM_HAL_STATUS_IN_USE;
    }

    //
    // Reset the CMD and DATA internal circuit for safe
    //
    if (pSDHCState->bCmdErr)
    {
        am_hal_sdhc_software_reset(pSDHC, AM_HAL_SDHC_SW_RESET_CMD_LINE);
        pSDHCState->bCmdErr = false;
        AM_HAL_SDHC_DEBUG("Software reset the Cmd Error\n");
    }

    if (pSDHCState->bDataErr)
    {
        am_hal_sdhc_software_reset(pSDHC, AM_HAL_SDHC_SW_RESET_DATA_LINE);
        pSDHCState->bDataErr = false;
        AM_HAL_SDHC_DEBUG("Software reset the Data Error\n");
    }

    //
    // Clear all interrupts's status
    //
    pSDHC->INTENABLE_b.CARDINTERRUPTSTATUSENABLE = 0x0;
    pSDHC->INTSTAT = ((uint32_t)-1);

    //
    // Disable all interrupts firstly except card insert and removal
    //
    pSDHC->INTSIG = ( SDIO0_INTSIG_CARDINSERTEN_Msk | SDIO0_INTSIG_CARDREMOVALEN_Msk | SDIO0_INTSIG_CARDINTEN_Msk);

#ifdef DYNAMIC_SWITCH_SDCLK_FEATURE
    //
    // Enable the SDCLK
    //
    pSDHC->CLOCKCTRL_b.SDCLKEN = 0x1;

    AM_HAL_SDHC_DEBUG("Enable the SDCLK\n");
#endif

    if ( (ui32Status = am_hal_sdhc_send_cmd(pSDHCState, pSDHC, pCmd, pCmdData)) != AM_HAL_STATUS_SUCCESS )
    {
        return ui32Status;
    }

    if ( (ui32Status = am_hal_sdhc_wait_cmd_done(pSDHCState, pSDHC, pCmd)) != AM_HAL_STATUS_SUCCESS )
    {
        return ui32Status;
    }

    am_hal_sdhc_get_cmd_response(pSDHC, pCmd);

    if ( pCmd->bASync )
    {
        AM_CRITICAL_BEGIN
        pSDHC->INTSIG = ((uint32_t)-1);
        pSDHCState->bAsyncCmdIsDone = false;
        AM_CRITICAL_END
    }

    if ( pCmdData == NULL || pCmd->bASync )
    {
#ifdef DYNAMIC_SWITCH_SDCLK_FEATURE
        if (pCmdData == NULL && !pCmd->bCheckBusyCmd)
        {
            pSDHC->CLOCKCTRL_b.SDCLKEN = 0x0;
            am_hal_delay_us(pSDHCState->ui32ClkOffDelay);
            AM_HAL_SDHC_DEBUG("Disable the SDCLK\n");
        }
#endif
        return AM_HAL_STATUS_SUCCESS;
    }

    return am_hal_sdhc_xfer_data(pSDHCState, pCmdData);
}

//
// SDHC normal/error status interrupts enable function
//
uint32_t am_hal_sdhc_intr_status_enable(void *pHandle, uint32_t ui32IntMask)
{
    am_hal_sdhc_state_t *pSDHCState = (am_hal_sdhc_state_t *)pHandle;
    uint32_t ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_SDHC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

    if ( !pSDHCState->prefix.s.bEnable )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    ui32Module = pSDHCState->ui32Module;

    //
    // Set the interrupt enables according to the mask.
    //
    SDHCn(ui32Module)->INTENABLE |= ui32IntMask;

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//
// SDHC normal/error status interrupts disable function
//
uint32_t am_hal_sdhc_intr_status_disable(void *pHandle, uint32_t ui32IntMask)
{
    am_hal_sdhc_state_t *pSDHCState = (am_hal_sdhc_state_t *)pHandle;
    uint32_t ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_SDHC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

    if ( !pSDHCState->prefix.s.bEnable )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    ui32Module = pSDHCState->ui32Module;
    //
    // Clear the interrupt enables according to the mask.
    //
    SDHCn(ui32Module)->INTENABLE &= ~ui32IntMask;

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//
// SDHC normal/error interrupt status get function
//
uint32_t am_hal_sdhc_intr_status_get(void *pHandle, uint32_t *pui32Status, bool bEnabledOnly)
{
    am_hal_sdhc_state_t *pSDHCState = (am_hal_sdhc_state_t *)pHandle;
    uint32_t ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_SDHC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

    if ( !pSDHCState->prefix.s.bEnable )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    ui32Module = pSDHCState->ui32Module;

    //
    // if requested, only return the interrupts that are enabled.
    //
    if ( bEnabledOnly )
    {
        uint32_t ui32RetVal = SDHCn(ui32Module)->INTSTAT;
        *pui32Status = ui32RetVal & SDHCn(ui32Module)->INTENABLE;
    }
    else
    {
        *pui32Status = SDHCn(ui32Module)->INTSTAT;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//
// SDHC normal/error interrupt status clear function
//
uint32_t am_hal_sdhc_intr_status_clear(void *pHandle, uint32_t ui32IntMask)
{
    am_hal_sdhc_state_t *pSDHCState = (am_hal_sdhc_state_t *)pHandle;
    uint32_t ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_SDHC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

    if ( !pSDHCState->prefix.s.bEnable )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    ui32Module = pSDHCState->ui32Module;

    SDHCn(ui32Module)->INTSTAT = ui32IntMask;
    *(volatile uint32_t*)(&SDHCn(ui32Module)->INTSTAT);
    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//
// SDHC normal interrupt signal enable function
//
uint32_t am_hal_sdhc_intr_signal_enable(void *pHandle, uint32_t ui32IntMask)
{
    am_hal_sdhc_state_t *pSDHCState = (am_hal_sdhc_state_t *)pHandle;
    uint32_t ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_SDHC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

    if ( !pSDHCState->prefix.s.bEnable )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    ui32Module = pSDHCState->ui32Module;

    //
    // Set the interrupt enables according to the mask.
    //
    SDHCn(ui32Module)->INTSIG |= ui32IntMask;

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//
// SDHC normal status signal disable function
//
uint32_t am_hal_sdhc_intr_signal_disable(void *pHandle, uint32_t ui32IntMask)
{
    am_hal_sdhc_state_t *pSDHCState = (am_hal_sdhc_state_t *)pHandle;
    uint32_t ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_SDHC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

    if ( !pSDHCState->prefix.s.bEnable )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    ui32Module = pSDHCState->ui32Module;
    //
    // Clear the interrupt enables according to the mask.
    //
    SDHCn(ui32Module)->INTSIG &= ~ui32IntMask;

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//
// SDHC interrupt service routine
//
uint32_t am_hal_sdhc_interrupt_service(void *pHandle, uint32_t ui32IntStatus)
{
    am_hal_sdhc_state_t *pSDHCState = (am_hal_sdhc_state_t *)pHandle;
    am_hal_card_host_t *pHost = pSDHCState->pHost;
    SDIO_Type *pSDHC = NULL;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_SDHC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

    if ( !pSDHCState->prefix.s.bEnable )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    pSDHC = SDHCn(pSDHCState->ui32Module);

    //
    // Asynchronous PIO write
    //
    if ( ui32IntStatus & SDIO0_INTSTAT_BUFFERREADREADY_Msk ||
        ui32IntStatus & SDIO0_INTSTAT_BUFFERWRITEREADY_Msk )
    {
        am_hal_sdhc_pio_xfer_data(pSDHCState);
        // AM_HAL_SDHC_DEBUG("%d\n", pSDHCState->ui32BlkCnt);
    }

    //
    // Asynchronous SDMA interrupt
    //
    if ( ui32IntStatus & SDIO0_INTSTAT_DMAINTERRUPT_Msk )
    {
        am_hal_sdhc_sdma_xfer_data(pSDHCState);
        // AM_HAL_SDHC_DEBUG("ISR - DMA Xfer BlkCnt %d\n", pSDHCState->ui32BlkCnt);
        pHost->Evt.eType = AM_HAL_EVT_SDMA_DONE;
        pHost->Evt.pCtx = pSDHCState->pHost;
        pHost->Evt.ui32BlkCnt = pSDHCState->ui32BlksPerSDMA;
        if ( pSDHCState->pHost->pfunEvtCallback )
        {
            pSDHCState->pHost->pfunEvtCallback(&pHost->Evt);
        }
    }

    //
    // Asynchronous PIO read, write, SDMA and ADMA xfer completion
    //
    if ( ui32IntStatus & SDIO0_INTSTAT_TRANSFERCOMPLETE_Msk )
    {
        pHost->Evt.eType = AM_HAL_EVT_XFER_COMPLETE;
        pHost->Evt.pCtx = pSDHCState->pHost;
        pHost->Evt.ui32BlkCnt = pSDHCState->ui32BlkCnt;
        if ( pSDHCState->pHost->pfunEvtCallback )
        {
            pSDHCState->pHost->pfunEvtCallback(&pHost->Evt);
        }
        AM_HAL_SDHC_DEBUG("ISR - Xfer Completion BlkCnt %d\n", pSDHCState->ui32BlkNum);
        pSDHCState->bAsyncCmdIsDone = true;

#ifdef DYNAMIC_SWITCH_SDCLK_FEATURE
        //
        // Disable the SDCLK after the xfer is done
        //
        pSDHC->CLOCKCTRL_b.SDCLKEN = 0x0;
        am_hal_delay_us(pSDHCState->ui32ClkOffDelay);
        AM_HAL_SDHC_DEBUG("Disable the SDCLK\n");
#endif

    }

    if (ui32IntStatus & (SDIO0_INTSTAT_ADMAERROR_Msk         |
                         SDIO0_INTSTAT_DATACRCERROR_Msk      |
                         SDIO0_INTSTAT_DATATIMEOUTERROR_Msk  |
                         SDIO0_INTSTAT_DATAENDBITERROR_Msk))
    {
        pHost->Evt.eType = AM_HAL_EVT_DAT_ERR;
        pHost->Evt.pCtx = pSDHCState->pHost;
        pHost->Evt.ui32BlkCnt = pSDHCState->ui32BlkCnt;
        pHost->AsyncCmdData.eDataError = am_hal_sdhc_check_data_error_type(ui32IntStatus);
        pSDHCState->bDataErr = true;
        pSDHCState->ui32DataErrCnt++;
        if ( pSDHCState->pHost->pfunEvtCallback )
        {
            pSDHCState->pHost->pfunEvtCallback(&pHost->Evt);
        }
        AM_HAL_SDHC_DEBUG("Xfer ERR INT 0x%x\n", ui32IntStatus);
        pSDHCState->bAsyncCmdIsDone = true;

#ifdef DYNAMIC_SWITCH_SDCLK_FEATURE
        //
        // Disable the SDCLK after the xfer is done
        //
        pSDHC->CLOCKCTRL_b.SDCLKEN = 0x0;
        am_hal_delay_us(pSDHCState->ui32ClkOffDelay);
        AM_HAL_SDHC_DEBUG("Disable the SDCLK\n");
#endif

    }

    //
    // SDIO Card interrupt
    //
    if (ui32IntStatus & SDIO0_INTSIG_CARDINTEN_Msk)
    {
        pHost->Evt.eType = AM_HAL_EVT_CARD_INT;
        pHost->Evt.pCtx = pSDHCState->pHost;

        if ( pSDHCState->pHost->pfunEvtCallback )
        {
            pSDHCState->pHost->pfunEvtCallback(&pHost->Evt);
        }

        AM_HAL_SDHC_DEBUG("SDIO Card INT 0x%x\n", ui32IntStatus);
        am_hal_sdhc_intr_status_disable(pHost->pHandle, SDIO0_INTENABLE_CARDINTERRUPTSTATUSENABLE_Msk);
    }

    //
    // SD Card insert or removal interrupt
    //
    if (ui32IntStatus & ( SDIO0_INTSTAT_CARDREMOVAL_Msk |
                          SDIO0_INTSTAT_CARDINSERTION_Msk))
    {
        uint32_t ui32PresentState = SDHCn(pSDHCState->ui32Module)->PRESENT;

        if ( ui32IntStatus & SDIO0_INTSTAT_CARDREMOVAL_Msk )
        {
            pHost->Evt.eType = AM_HAL_EVT_CARD_NOT_PRESENT;
        }

       if ( (ui32IntStatus & SDIO0_INTSTAT_CARDINSERTION_Msk) && (ui32PresentState & SDIO0_PRESENT_CARDINSERTED_Msk) )
        {
            pHost->Evt.eType = AM_HAL_EVT_CARD_PRESENT;
        }
        else
        {
            pHost->Evt.eType = AM_HAL_EVT_CARD_NOT_PRESENT;
        }

        if ( pSDHCState->pHost->pfunEvtCallback )
        {
            pSDHCState->pHost->pfunEvtCallback(&pHost->Evt);
        }

        AM_HAL_SDHC_DEBUG("SD Card INT 0x%x\n", ui32IntStatus);
    }

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
