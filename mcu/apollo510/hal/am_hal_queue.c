//*****************************************************************************
//
//! @file am_hal_queue.c
//!
//! @brief Functions for implementing a queue system.
//!
//! @addtogroup queue4 Queue Implementation
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

#include <stdint.h>
#include <stdbool.h>
#include "am_mcu_apollo.h"

//*****************************************************************************
//
// Initializes a queue.
//
// This function initializes the members of a queue structure and attaches it
// to an array of memory that it can use for storage. This function should be
// called before the queue is used.
//
// In this example, we are creating a queue that can hold 1024 32-bit
// integers. The integers themselves will be stored in the array named
// pui32WorkingSpace, while information about the queue itself will be stored
// in sDataQueue.
//
//*****************************************************************************
void
am_hal_queue_init(am_hal_queue_t *psQueue, void *pvData, uint32_t ui32ItemSize,
                  uint32_t ui32ArraySize)
{
    psQueue->ui32WriteIndex = 0;
    psQueue->ui32ReadIndex = 0;
    psQueue->ui32Length = 0;
    psQueue->ui32Capacity = ui32ArraySize;
    psQueue->ui32ItemSize = ui32ItemSize;
    psQueue->pui8Data = (uint8_t *) pvData;
}

//*****************************************************************************
//
// Adds an item to the Queue
//
// This function will copy the data pointed to by pvSource into the queue. The
// \e ui32NumItems term specifies the number of items to be copied from \e
// pvSource. The size of an "item" depends on how the queue was initialized.
// Please see am_hal_queue_init() for more information on this.
//
//*****************************************************************************
bool
am_hal_queue_item_add(am_hal_queue_t *psQueue, const void *pvSource, uint32_t ui32NumItems)
{
    uint8_t *pui8Source;
    uint32_t ui32Bytes = ui32NumItems * psQueue->ui32ItemSize;
    bool bSuccess = false;

    pui8Source = (uint8_t *) pvSource;

    AM_CRITICAL_BEGIN

    //
    // Check to make sure that the buffer isn't already full
    //
    if ( am_hal_queue_space_left(psQueue) >= ui32Bytes )
    {
        //
        // Loop over the bytes in the source array.
        //
        for ( uint32_t i = 0; i < ui32Bytes; i++ )
        {
            //
            // Write the value to the buffer, but only if the source pointer is
            // valid.
            //
            if (pvSource)
            {
                psQueue->pui8Data[psQueue->ui32WriteIndex] = pui8Source[i];
            }

            //
            // Advance the write index, making sure to wrap if necessary.
            //
            psQueue->ui32WriteIndex = ((psQueue->ui32WriteIndex + 1) %
                                       psQueue->ui32Capacity);
        }

        //
        // Update the length value appropriately.
        //
        psQueue->ui32Length += ui32Bytes;

        //
        // Report a success.
        //
        bSuccess = true;
    }
    else
    {
        //
        // The buffer can't fit the amount of data requested. Return a
        // failure.
        //
        bSuccess = false;
    }

    AM_CRITICAL_END

    return bSuccess;
}

//*****************************************************************************
//
// Removes an item from the Queue
//
// This function will copy the data from the queue into the memory pointed to
// by pvDest. The \e ui32NumItems term specifies the number of items to be
// copied from the queue. The size of an "item" depends on how the queue was
// initialized.  Please see am_hal_queue_init() for more information on this.
//
//*****************************************************************************
bool
am_hal_queue_item_get(am_hal_queue_t *psQueue, void *pvDest, uint32_t ui32NumItems)
{
    uint8_t *pui8Dest;
    uint32_t ui32Bytes = ui32NumItems * psQueue->ui32ItemSize;
    bool bSuccess = false;

    pui8Dest = (uint8_t *) pvDest;

    AM_CRITICAL_BEGIN

    //
    // Check to make sure that the buffer isn't empty
    //
    if ( am_hal_queue_data_left(psQueue) >= ui32Bytes )
    {
        //
        // Loop over the bytes in the destination array.
        //
        for ( uint32_t i = 0; i < ui32Bytes; i++ )
        {
            //
            // Grab the next value from the buffer, but only if the
            // destination pointer is valid.
            //
            if (pvDest)
            {
                pui8Dest[i] = psQueue->pui8Data[psQueue->ui32ReadIndex];
            }

            //
            // Advance the read index, wrapping if needed.
            //
            psQueue->ui32ReadIndex = ((psQueue->ui32ReadIndex + 1) %
                                      psQueue->ui32Capacity);
        }

        //
        // Adjust the length value to reflect the change.
        //
        psQueue->ui32Length -= ui32Bytes;

        //
        // Report a success.
        //
        bSuccess = true;
    }
    else
    {
        //
        // If the buffer didn't have enough data, just return false.
        //
        bSuccess = false;
    }

    AM_CRITICAL_END

    return bSuccess;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
