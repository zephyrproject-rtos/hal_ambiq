/*
 * Copyright (c) 2001-2019, Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause OR Arm’s non-OSI source license
 */

/*
 * Apollo510 (Cortex-M55) DCACHE / CC312 DMA coherency
 * ----------------------------------------------------
 * On Apollo510 a write-back data cache (DCACHE, 32-byte line) sits
 * in front of the system memory that the CC312 DMA reads from and
 * writes to. Two things must be honored on the internal-DMA ChaCha
 * path:
 *
 *   1. Every system-memory buffer handed to the engine must be
 *      aligned to the DCACHE line size (CC_DCACHE_LINE_BYTES, 32)
 *      and its backing allocation padded to a multiple of it. Cache
 *      maintenance works on whole lines; a buffer sharing a line
 *      with unrelated data would have that neighbor clobbered.
 *
 *   2. Before the transfer is kicked, the input buffer is cleaned
 *      (so CPU writes reach memory for the engine to read) and the
 *      output buffer is clean+invalidated (so no stale dirty line
 *      later evicts over the result), using the CMSIS SCB
 *      DCACHE-by-address intrinsics. After CC_HalWaitInterrupt
 *      returns, any driver state that must be stored (e.g.
 *      StoreChachaState) should be called before cache maintenance;
 *      not all operations require this step. A small dummy DMA read
 *      (ChachaDmaReadFlush) then drains the engine's last write to
 *      memory, followed by output invalidation so the CPU re-reads
 *      the engine's result rather than a stale cached copy.
 *
 * Only ProcessChacha (internal DMA) needs this. SetDataBuffersInfo
 * (driver_common.c) is shared and only records addresses, and
 * chacha_driver_ext_dma.c leaves DMA/coherency to the host
 * application.
 */

#include <stdint.h>
#include <stdio.h>
#include <stddef.h>
#include "cc_pal_mutex.h"
#include "cc_pal_abort.h"
#include "chacha_driver.h"
#include "driver_defs.h"
#include "cc_hal.h"
#include "cc_hal_plat.h"
#include "cc_sram_map.h"
#include "cc_regs.h"
#include "dx_crys_kernel.h"
#include "cc_util_pm.h"

/* CMSIS-Core (Cortex-M55) for the SCB DCACHE maintenance intrinsics. */
#include "apollo510.h"


extern CC_PalMutex CCSymCryptoMutex;

/* chacha mode, poly1305 disabled, 256 bit key, 20 rounds, 64 bit iv, do not reset the block counter (overwritten by the context) */
#define CHACHA_CONTROL_REG_VAL        (1 << DX_CHACHA_CONTROL_REG_INIT_FROM_HOST_BIT_SHIFT)
#define CHACHA_CONTROL_REG_USE_IV_96  (1 << DX_CHACHA_CONTROL_REG_USE_IV_96BIT_BIT_SHIFT)

/* Local DCACHE-line-aligned sink for the post-operation DMA-read flush. */
static uint32_t gChachaDmaFlushSink[CC_DCACHE_LINE_BYTES / sizeof(uint32_t)]
        __attribute__((aligned(CC_DCACHE_LINE_BYTES)));

/******************************************************************************
*               PRIVATE FUNCTIONS
******************************************************************************/

static drvError_t InitChacha(ChachaContext_t *chachaCtx)
{
    uint32_t irrVal = 0;

    /* verify user context pointer */
    if ( chachaCtx == NULL ) {
        return CHACHA_DRV_INVALID_USER_CONTEXT_POINTER_ERROR;
    }

    /* verify chacha valid input addr type */
    if ( (chachaCtx->inputDataAddrType != SRAM_ADDR) &&
         (chachaCtx->inputDataAddrType != DLLI_ADDR) ) {
        return CHACHA_DRV_ILLEGAL_INPUT_ADDR_MEM_ERROR;
    }

    /* verify chacha valid output addr type */
    if ( (chachaCtx->outputDataAddrType != SRAM_ADDR) &&
         (chachaCtx->outputDataAddrType != DLLI_ADDR) ) {
        return CHACHA_DRV_ILLEGAL_OUTPUT_ADDR_MEM_ERROR;
    }

    /* make sure sym engines are ready to use */
    CC_HAL_WAIT_ON_CRYPTO_BUSY();

    /* clear all interrupts before starting the engine */
    CC_HalClearInterruptBit(0xFFFFFFFFUL);

    /* mask dma interrupts which are not required */
    irrVal = CC_HAL_READ_REGISTER(CC_REG_OFFSET(HOST_RGF, HOST_IMR));
    CC_REG_FLD_SET(HOST_RGF, HOST_IMR, SRAM_TO_DIN_MASK, irrVal, 1);
    CC_REG_FLD_SET(HOST_RGF, HOST_IMR, DOUT_TO_SRAM_MASK, irrVal, 1);
    CC_REG_FLD_SET(HOST_RGF, HOST_IMR, MEM_TO_DIN_MASK, irrVal, 1);
    CC_REG_FLD_SET(HOST_RGF, HOST_IMR, DOUT_TO_MEM_MASK, irrVal, 1);
    CC_REG_FLD_SET(HOST_RGF, HOST_IMR, SYM_DMA_COMPLETED_MASK, irrVal, 0);
    CC_HalMaskInterrupt(irrVal);

    /* configure DIN-CHACHA-DOUT */
    CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, CRYPTO_CTL) ,CONFIG_DIN_CHACHA_DOUT_VAL);

    return CHACHA_DRV_OK;
}

static drvError_t LoadChachaState(ChachaContext_t *chachaCtx)
{
        /* verify user context pointer */
        if (chachaCtx == NULL) {
                return CHACHA_DRV_INVALID_USER_CONTEXT_POINTER_ERROR;
        }

        /* write the initial counter value according to mode */
        if (chachaCtx->nonceSize == NONCE_SIZE_64) {
                CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, CHACHA_BLOCK_CNT_MSB), chachaCtx->blockCounterMsb);
                CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, CHACHA_IV_0), chachaCtx->nonceBuf[0]);
                CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, CHACHA_IV_1), chachaCtx->nonceBuf[1]);
        }
        else if (chachaCtx->nonceSize == NONCE_SIZE_96) {
                CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, CHACHA_BLOCK_CNT_MSB), chachaCtx->nonceBuf[0]);
                CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, CHACHA_IV_0), chachaCtx->nonceBuf[1]);
                CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, CHACHA_IV_1), chachaCtx->nonceBuf[2]);
        }
        else {
                return CHACHA_DRV_ILLEGAL_NONCE_SIZE_ERROR;
        }
        CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, CHACHA_BLOCK_CNT_LSB), chachaCtx->blockCounterLsb);

        return CHACHA_DRV_OK;
}


static drvError_t StoreChachaState(ChachaContext_t *chachaCtx)
{
        /* verify user context pointer */
        if (chachaCtx == NULL) {
                return CHACHA_DRV_INVALID_USER_CONTEXT_POINTER_ERROR;
        }

        /* read the initial counter value according to mode */
        if (chachaCtx->nonceSize == NONCE_SIZE_64) {
                chachaCtx->blockCounterMsb = CC_HAL_READ_REGISTER(CC_REG_OFFSET(HOST_RGF, CHACHA_BLOCK_CNT_MSB));
                chachaCtx->nonceBuf[0] = CC_HAL_READ_REGISTER(CC_REG_OFFSET(HOST_RGF, CHACHA_IV_0));
                chachaCtx->nonceBuf[1] = CC_HAL_READ_REGISTER(CC_REG_OFFSET(HOST_RGF, CHACHA_IV_1));
        }
        else if (chachaCtx->nonceSize == NONCE_SIZE_96) {
                chachaCtx->nonceBuf[0] = CC_HAL_READ_REGISTER(CC_REG_OFFSET(HOST_RGF, CHACHA_BLOCK_CNT_MSB));
                chachaCtx->nonceBuf[1] = CC_HAL_READ_REGISTER(CC_REG_OFFSET(HOST_RGF, CHACHA_IV_0));
                chachaCtx->nonceBuf[2] = CC_HAL_READ_REGISTER(CC_REG_OFFSET(HOST_RGF, CHACHA_IV_1));
        }
        else {
                return CHACHA_DRV_ILLEGAL_NONCE_SIZE_ERROR;
        }
        chachaCtx->blockCounterLsb = CC_HAL_READ_REGISTER(CC_REG_OFFSET(HOST_RGF, CHACHA_BLOCK_CNT_LSB));

        return CHACHA_DRV_OK;
}

static drvError_t LoadChachaKey(ChachaContext_t *chachaCtx)
{
        int enrtyNum = 0;

        /* verify user context pointer */
        if (chachaCtx == NULL) {
                return CHACHA_DRV_INVALID_USER_CONTEXT_POINTER_ERROR;
        }

        for (enrtyNum = 0; enrtyNum < CHACHA_256_BIT_KEY_SIZE_WORDS; ++enrtyNum) {
                CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, CHACHA_KEY0) + (sizeof(uint32_t) * enrtyNum), chachaCtx->keyBuf[enrtyNum]);
        }

        return CHACHA_DRV_OK;
}

/*
 * Drain the engine's last DMA write to memory.
 *
 * After the data DMA completes, the CC312 write path may still hold
 * the final line(s). Issuing a small dummy DMA read from the output
 * tail into a throwaway sink forces that write to land in memory
 * before the CPU invalidates and re-reads the result. The throwaway
 * data itself is discarded; the ChaCha state has already been
 * captured by StoreChachaState, so the incidental pass of the sink
 * read through the core does not affect the operation's result.
 */
static drvError_t ChachaDmaReadFlush(uint32_t srcAddr)
{
        uint32_t irrVal = 0;

        /* clear all interrupts and re-mask everything except SYM DMA completion */
        CC_HalClearInterruptBit(0xFFFFFFFFUL);
        irrVal = CC_HAL_READ_REGISTER(CC_REG_OFFSET(HOST_RGF, HOST_IMR));
        CC_REG_FLD_SET(HOST_RGF, HOST_IMR, SRAM_TO_DIN_MASK, irrVal, 1);
        CC_REG_FLD_SET(HOST_RGF, HOST_IMR, DOUT_TO_SRAM_MASK, irrVal, 1);
        CC_REG_FLD_SET(HOST_RGF, HOST_IMR, MEM_TO_DIN_MASK, irrVal, 1);
        CC_REG_FLD_SET(HOST_RGF, HOST_IMR, DOUT_TO_MEM_MASK, irrVal, 1);
        CC_REG_FLD_SET(HOST_RGF, HOST_IMR, SYM_DMA_COMPLETED_MASK, irrVal, 0);
        CC_HalMaskInterrupt(irrVal);

        /* destination: throwaway sink; source: output tail; one cache line */
        CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, DST_LLI_WORD0) ,(uint32_t)(uintptr_t)gChachaDmaFlushSink);
        CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, DST_LLI_WORD1) ,CC_DCACHE_LINE_BYTES);
        CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, SRC_LLI_WORD0) ,srcAddr);
        CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, SRC_LLI_WORD1) ,CC_DCACHE_LINE_BYTES);

        /* wait for the flush DMA to complete */
        irrVal = 0;
        CC_REG_FLD_SET(HOST_RGF, HOST_IRR, SYM_DMA_COMPLETED, irrVal, 1);
        return CC_HalWaitInterrupt(irrVal);
}

/******************************************************************************
*               PUBLIC FUNCTIONS
******************************************************************************/
drvError_t ProcessChacha(ChachaContext_t *chachaCtx, CCBuffInfo_t *pInputBuffInfo, CCBuffInfo_t *pOutputBuffInfo, uint32_t inDataSize)
{
    uint32_t irrVal = 0;
    uint32_t chachaCtrl = CHACHA_CONTROL_REG_VAL;
    drvError_t drvRc = CHACHA_DRV_OK;
    uint32_t regVal = 0;
    uint32_t inputDataAddr, outputDataAddr;

    /* check input parameters */
    if ( (pInputBuffInfo == NULL) || (pOutputBuffInfo == NULL)) {
         return CHACHA_DRV_INVALID_USER_DATA_BUFF_POINTER_ERROR;
    }

    /* verify user context pointer */
    if ( chachaCtx == NULL ) {
        return CHACHA_DRV_INVALID_USER_CONTEXT_POINTER_ERROR;
    }
    if (((chachaCtx->inputDataAddrType == SRAM_ADDR) && (inDataSize >= CC_SRAM_MAX_SIZE)) ||
        ((chachaCtx->inputDataAddrType == DLLI_ADDR) && (inDataSize >= DLLI_MAX_BUFF_SIZE))) {
        return CHACHA_DRV_ILLEGAL_MEM_SIZE_ERROR;
    }

    /* lock mutex for more chacha hw operation */
    drvRc = CC_PalMutexLock(&CCSymCryptoMutex, CC_INFINITE);
    if (drvRc != 0) {
        CC_PalAbort("Fail to acquire mutex\n");
    }

    /* increase CC counter at the beginning of each operation */
    drvRc = CC_IS_WAKE;
    if (drvRc != 0) {
        CC_PalAbort("Fail to increase PM counter\n");
    }

    /* enable clock */
    CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, CHACHA_CLK_ENABLE) ,SET_CLOCK_ENABLE);
    CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, DMA_CLK_ENABLE) ,SET_CLOCK_ENABLE);

    drvRc = InitChacha(chachaCtx);
    if (drvRc != CHACHA_DRV_OK) {
            goto ProcessExit;
    }

    /* write the initial counter value */
    drvRc = LoadChachaState(chachaCtx);
    if (drvRc != CHACHA_DRV_OK) {
            goto ProcessExit;
    }

    /* load key */
    drvRc = LoadChachaKey(chachaCtx);
    if (drvRc != CHACHA_DRV_OK) {
            goto ProcessExit;
    }

    /* configure the CHACHA mode */
    if (chachaCtx->nonceSize == NONCE_SIZE_96) {
        chachaCtrl |= CHACHA_CONTROL_REG_USE_IV_96;
    }

    CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, CHACHA_CONTROL_REG), chachaCtrl);

    inputDataAddr = pInputBuffInfo->dataBuffAddr;
    outputDataAddr = pOutputBuffInfo->dataBuffAddr;

    /* configure the HW with the correct data buffer attributes (secure/non-secure) */
    CC_REG_FLD_SET(HOST_RGF, AHBM_HNONSEC, AHB_READ_HNONSEC, regVal, pInputBuffInfo->dataBuffNs);
    CC_REG_FLD_SET(HOST_RGF, AHBM_HNONSEC, AHB_WRITE_HNONSEC, regVal, pOutputBuffInfo->dataBuffNs);
    CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, AHBM_HNONSEC) ,regVal);

    /* configure destination address and size */
    /* and set dout bit in irr */
    if (chachaCtx->outputDataAddrType == DLLI_ADDR) {
        CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, DST_LLI_WORD0) ,outputDataAddr);
        CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, DST_LLI_WORD1) ,inDataSize);
    } else {
        CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, SRAM_DEST_ADDR) ,outputDataAddr);
        CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, DOUT_SRAM_BYTES_LEN) ,inDataSize);
    }

    /*
     * DCACHE clean before the DMA is kicked (concern #2). Clean the
     * input so the engine reads committed data; clean+invalidate the
     * output so no stale dirty line later evicts over the result.
     * System-memory buffers only - SRAM_ADDR buffers live in
     * CC-internal SRAM, behind no CPU cache. Buffers must be
     * CC_DCACHE_LINE_BYTES aligned/padded (concern #1).
     */
    if (SCB->CCR & SCB_CCR_DC_Msk) {
        if (chachaCtx->inputDataAddrType == DLLI_ADDR) {
            SCB_CleanInvalidateDCache_by_Addr((void *)(uintptr_t)inputDataAddr, (int32_t)inDataSize);
        }
        if (chachaCtx->outputDataAddrType == DLLI_ADDR) {
            SCB_CleanInvalidateDCache_by_Addr((void *)(uintptr_t)outputDataAddr, (int32_t)inDataSize);
        }
    }

    /* configure source address and size */
    if (chachaCtx->inputDataAddrType == DLLI_ADDR) {
        CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, SRC_LLI_WORD0) ,inputDataAddr);
        CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, SRC_LLI_WORD1) ,inDataSize);
    } else {
        CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, SRAM_SRC_ADDR) ,inputDataAddr);
        CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, DIN_SRAM_BYTES_LEN) ,inDataSize);
    }

    /* set dma completion bit in irr */
    CC_REG_FLD_SET(HOST_RGF, HOST_IRR, SYM_DMA_COMPLETED, irrVal, 1);
    drvRc = CC_HalWaitInterrupt(irrVal);
    if (drvRc != CHACHA_DRV_OK) {
        goto ProcessExit;
    }
    /* get machine state */
    drvRc = StoreChachaState(chachaCtx);
    if (drvRc != CHACHA_DRV_OK) {
            goto ProcessExit;
    }

    /*
     * DCACHE invalidate after the DMA (concern #2). Drain the
     * engine's last write to memory with a dummy DMA read, then
     * invalidate the output region so the CPU re-reads the engine's
     * result instead of a stale cached copy.
     */
    if (chachaCtx->outputDataAddrType == DLLI_ADDR) {
        drvRc = ChachaDmaReadFlush(outputDataAddr);
        if (drvRc != CHACHA_DRV_OK) {
                goto ProcessExit;
        }
        if (SCB->CCR & SCB_CCR_DC_Msk) {
            SCB_InvalidateDCache_by_Addr((void *)(uintptr_t)outputDataAddr, (int32_t)inDataSize);
        }
    }

    ProcessExit:
    CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, CHACHA_CLK_ENABLE) ,SET_CLOCK_DISABLE);
    CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, DMA_CLK_ENABLE) ,SET_CLOCK_DISABLE);

    /* decrease CC counter at the end of each operation */
    if (CC_IS_IDLE != 0) {
        CC_PalAbort("Fail to decrease PM counter\n");
    }

    /* release mutex */
    if (CC_PalMutexUnlock(&CCSymCryptoMutex) != 0) {
        CC_PalAbort("Fail to release mutex\n");
    }

    return drvRc;
}


