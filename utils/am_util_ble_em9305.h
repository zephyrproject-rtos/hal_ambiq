//*****************************************************************************
//
//! @file am_util_ble_em9305.h
//!
//! @brief EM9305 BLE functions not covered by the HAL.
//!
//! This file contains functions for interacting with the Apollo5 BLE hardware
//! that are not already covered by the HAL. Most of these commands either
//! adjust RF settings or facilitate RF testing operations.
//!
//! @addtogroup em9305_util EM9305 - BLE Functions
//! @ingroup utils
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

#ifndef AM_UTIL_BLE_EM9305_H
#define AM_UTIL_BLE_EM9305_H

//*****************************************************************************
//
// External function declarations.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif



//*****************************************************************************
//
//! @brief Reset EM9305 BLE controller
//!
//! @param pHandle - pointer to BLE Handle
//!
//! @return Status Code
//
//*****************************************************************************
extern uint32_t am_util_ble_hci_reset(void *pHandle);

//*****************************************************************************
//
//! @brief Set BLE sleep enable/disable for the BLE core.
//! enable = 'true' set sleep enable, enable = 'false' set sleep disable
//!
//! @param pHandle - pointer to BLE Handle
//! @param sleep_option - enable/disable sleep
//!
//! @return Status Code
//
//*****************************************************************************
extern uint32_t am_util_ble_sleep_set(void* pHandle, bool enable);

//*****************************************************************************
//
//! @brief set the tx power of BLE
//!
//! @param pHandle - pointer to BLE Handle
//! @param ui32TxPower - enum txPowerLevel_t defined in hci_drv_EM9305.h
//!
//! @return Status Code
//
//*****************************************************************************
extern uint32_t am_util_ble_tx_power_set(void* pHandle, uint8_t ui32TxPower);

//*****************************************************************************
//
//! @brief End test.
//!
//! @param pHandle - pointer to BLE Handle
//! @param recvpackets - pointer for RX Packets
//!
//! @return Status Code
//
//*****************************************************************************
uint32_t
am_util_ble_end_test(void *pHandle, uint32_t *recvpackets);

//*****************************************************************************
//
//! @brief To do directly transmit test where the DUT generates test reference
//! packets at a fixed interval.
//
//! @param channel - Transmit channel, ranges 0x00 - 0x27
//! @param test_data_len - Length of payload in each packet, range 0x00-0xFF
//! @param pkt_payload -  Packet payload, ranges 0x00 - 0x07
//!
//! @return Status Code
//
//*****************************************************************************
extern uint32_t am_util_ble_transmit_test(void *pHandle, uint8_t channel, uint8_t test_data_len, uint8_t pkt_payload);
//*****************************************************************************
//
//! @brief to do directly output modulation signal.
//! channel ranges from 0 to 0x27, pattern from 0 to 7.
//!
//! @param pHandle - pointer to BLE Handle
//! @param channel - channel number
//! @param pattern - pattern for TX test
//!
//! @return Status Code
//
//*****************************************************************************
extern uint32_t am_util_ble_trasmitter_test_ex(void *pHandle, uint8_t tx_test_mode, uint8_t channel, uint8_t test_data_len, uint8_t pkt_payload, uint8_t pyh, uint8_t tx_power);

//*****************************************************************************
//
//! @brief Receiver test.
//! change channel ranges from 0 to 0x27, return received packets in 100ms.
//!
//! @param pHandle - pointer to BLE Handle
//! @param channel - channel number
//!
//! @return Status Code
//
//*****************************************************************************
extern uint32_t am_util_ble_receiver_test_ex(void *pHandle,
                                             uint8_t channel);

extern uint32_t am_util_ble_set_bd_addr(void *pHandle, uint8_t* bd_addr);
extern uint32_t am_util_ble_get_w2pass_info(void *pHandle, char *info);
#ifdef __cplusplus
}
#endif

#endif // AM_UTIL_BLE_EM9305_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

