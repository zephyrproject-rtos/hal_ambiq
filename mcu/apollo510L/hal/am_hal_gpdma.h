//*****************************************************************************
//
//! @file am_hal_gpdma.h
//!
//! @brief Functions for Memcopy DMA Engine
//!
//! @addtogroup gpdma4 GPDMA - General Purpose Memcopy DMA
//! @ingroup apollo510L_hal
//! @{

//*****************************************************************************

//*****************************************************************************
//
// ${copyright}
//
// This is part of revision ${version} of the AmbiqSuite Development Package.
//
//*****************************************************************************

#ifndef AM_HAL_GPDMA_H
#define AM_HAL_GPDMA_H

//*****************************************************************************
//
//! General Purpose DMA Config
//
//*****************************************************************************
typedef struct
{
    bool     bHighPriority;
    uint32_t burstSize;
} am_hal_gpdma_config_t;

//*****************************************************************************
//
//! General Purpose DMA Transfer
//
//*****************************************************************************
typedef struct
{
    bool bSrcStatic; // address is static (non-incrementing), e.g. for FIFO
    bool bDstStatic; // address is static (non-incrementing), e.g. for FIFO
    uint32_t srcAddress;
    uint32_t dstAddress;
    uint32_t numBytes;
} am_hal_gpdma_transfer_t;

//*****************************************************************************
//
//! General Purpose DMA State
//
//*****************************************************************************
typedef enum
{
    AM_HAL_GPDMA_STATUS_TIP,
    AM_HAL_GPDMA_STATUS_ERR,
    AM_HAL_GPDMA_STATUS_IDLE,
} am_hal_gpdma_state_e;

//*****************************************************************************
//
//! General Purpose DMA Status
//
//*****************************************************************************
typedef struct
{
    am_hal_gpdma_state_e  state;
    uint32_t               numPendingBytes;
} am_hal_gpdma_status_t;

#ifdef __cplusplus
extern "C"
{
#endif

//
//! Transfer callback function prototype
//
typedef void (*am_hal_transfer_callback_t)(void *pCallbackCtxt, uint32_t transferStatus);

//*****************************************************************************
//
//! @brief GPDMA initialization function
//!
//! @param channelId - channelId.
//! @param pHandle  - returns the handle for the GPDMA instance.
//!
//! This function accepts a GPDMA channelId, allocates the interface and then
//! returns a handle to be used by the remaining interface functions.
//!
//! @return status   - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_gpdma_initialize(uint32_t channelId, void **pHandle);

//*****************************************************************************
//
//! @brief GPDMA configuration function
//!
//! @param pHandle      - handle for the GPDMA.
//! @param pConfig      - pointer to the GPDMA specific configuration.
//!
//! This function configures the interface settings for the GPDMA.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_gpdma_config(void *pHandle, am_hal_gpdma_config_t *pConfig);

//*****************************************************************************
//
//! @brief General Purpose DMA Transfer
//!
//! @param pHandle       - pointer the IOM instance handle.
//! @param pTransfer - pointer to IOM transaction.
//! @param pfnCallback   - pointer to the callback for transaction (could be NULL)
//! @param pCallbackCtxt - pointer to the context to the callback (could be NULL)
//!
//! This function adds a function to the internal high priority transaction queue
//!
//! @return HAL status of the operation.
//
//*****************************************************************************
extern uint32_t am_hal_gpdma_transfer(void *pHandle,
                                      am_hal_gpdma_transfer_t *pTransfer,
                                      am_hal_transfer_callback_t pfnCallback,
                                      void *pCallbackCtxt);

//*****************************************************************************
//
//! @brief General Purpose DMA Get Status
//!
//! @param pHandle       - pointer the IOM instance handle.
//! @param pStatus
//!
//! @return HAL status of the operation.
//
//*****************************************************************************
extern uint32_t am_hal_gpdma_get_status(void *pHandle, am_hal_gpdma_status_t *pStatus);

//*****************************************************************************
//
//! @brief General Purpose DMA Uninitialize
//!
//! @param pHandle       - pointer the IOM instance handle.
//!
//! @return HAL status of the operation.
//
//*****************************************************************************
extern uint32_t am_hal_gpdma_uninitialize(void *pHandle);

//*****************************************************************************
//
//! @brief General Purpose DMA Interrupt Enable
//!
//! @param pHandle       - pointer the IOM instance handle.
//! @param ui32IntMask
//!
//! @return HAL status of the operation.
//
//*****************************************************************************
extern uint32_t am_hal_gpdma_interrupt_enable(void *pHandle, uint32_t ui32IntMask);

//*****************************************************************************
//
//! @brief General Purpose DMA Interrupt Disable
//!
//! @param pHandle       - pointer the IOM instance handle.
//! @param ui32IntMask
//!
//! @return HAL status of the operation.
//
//*****************************************************************************
extern uint32_t am_hal_gpdma_interrupt_disable(void *pHandle, uint32_t ui32IntMask);

//*****************************************************************************
//
//! @brief General Purpose DMA Interrupt Clear
//!
//! @param pHandle       - pointer the IOM instance handle.
//! @param ui32IntMask
//!
//! @return HAL status of the operation.
//
//*****************************************************************************
extern uint32_t am_hal_gpdma_interrupt_clear(void *pHandle, uint32_t ui32IntMask);

//*****************************************************************************
//
//! @brief General Purpose DMA Status Get
//!
//! @param pHandle       - pointer the IOM instance handle.
//! @param bEnabledOnly
//! @param pui32IntStatus
//!
//! @return HAL status of the operation.
//
//*****************************************************************************
extern uint32_t am_hal_gpdma_interrupt_status_get(void *pHandle, bool bEnabledOnly,
                                                  uint32_t *pui32IntStatus);

//*****************************************************************************
//
//! @brief General Purpose DMA Interrupt Service
//!
//! @param pHandle       - pointer the IOM instance handle.
//! @param ui32IntMask
//!
//! @return HAL status of the operation.
//
//*****************************************************************************
extern uint32_t am_hal_gpdma_interrupt_service(void *pHandle, uint32_t ui32IntMask);

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_GPDMA_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

