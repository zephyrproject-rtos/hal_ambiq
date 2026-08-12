//*****************************************************************************
//
//! @file am_devices_em9305.h
//!
//! @brief An implementation of the Apollo interface to EM9305 using the IOM.
//!
//! @addtogroup em9305 EM9305 BLE Device Driver
//! @ingroup devices
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2023, Ambiq Micro, Inc.
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
//*****************************************************************************

#ifndef AM_DEVICES_EM9305_H
#define AM_DEVICES_EM9305_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

//*****************************************************************************
//
//! Type definitions and constants.
//
//*****************************************************************************

//
//! Firmware image record: one contiguous region of the EM9305 NVM binary.
//
typedef struct {
  const uint8_t *data;   //!< Pointer to image data
  uint32_t length;       //!< Length of this record in bytes
  uint32_t address;      //!< Target NVM address
} ImageRecord;

//
//! NVM page descriptor used for per-page erase operations.
//
typedef struct {
  uint8_t page;  //!< Page number within the area
  uint8_t area;  //!< NVM area (0 = main, 1 = info)
} NvmPage;

#define EM9305_STS_CHK_CNT_MAX 10     //!< Check EM9305 status count
#define WAIT_EM9305_RDY_TIMEOUT 12000 //!< EM9305 timeout value (1.2 sec)

//
//! Maximum single-packet HCI payload sizes for the EM9305 SPI transport.
//!
//! Host -> Controller (TX) worst case is an HCI Command:
//!   1 H4 type + 3 byte HCI Command header + 255 byte parameter = 259.
//!
//! Controller -> Host (RX) worst case is an HCI Event:
//!   1 H4 type + 2 byte HCI Event header + 255 byte parameter = 258.
//!
//! HCI ACL/ISO frames under LE-only configurations fit comfortably within
//! the same bounds (with LE Data Length Extension the LL PDU payload is
//! capped at 251 bytes).
//
#define EM9305_HCI_MAX_TX_LEN 259     //!< Max host->controller HCI packet (Command)
#define EM9305_HCI_MAX_RX_LEN 258     //!< Max controller->host HCI packet (Event)

#define EM9305_BUFFER_SIZE    EM9305_HCI_MAX_TX_LEN

#define EM9305_SPI_HEADER_TX 0x42     //!< SPI TX header byte
#define EM9305_SPI_HEADER_RX 0x81     //!< SPI RX header byte
#define EM9305_STS1_READY_VALUE 0xC0  //!< SPI Ready byte

//
//! Status codes for EM9305 operations
//
typedef enum
{
    AM_DEVICES_EM9305_STATUS_SUCCESS,
    AM_DEVICES_EM9305_STATUS_ERROR,
    AM_DEVICES_EM9305_RX_FULL,
    AM_DEVICES_EM9305_TX_BUSY,
    AM_DEVICES_EM9305_NO_DATA_TX,
    AM_DEVICES_EM9305_NOT_READY,
    AM_DEVICES_EM9305_DATA_LENGTH_ERROR,
    AM_DEVICES_EM9305_DATA_TRANSFER_ERROR,
    AM_DEVICES_EM9305_CMD_TRANSFER_ERROR,
    AM_DEVICES_EM9305_CHECKSUM_ERROR,
    AM_DEVICES_EM9305_TX_PARTIAL,
} am_devices_em9305_status_t;

//
//! BLE controller function callback for SPI transceive
//! Note: Uses void* to match apollox_blue.h typedef
//
typedef int (*bt_spi_transceive_fun)(void *tx, uint32_t tx_len, void *rx, uint32_t rx_len);

//
//! Callback used to start/stop the 30 kHz signal required to enter EM9305
//! configuration mode for firmware update. The signal source (e.g. CTIMER PWM)
//! and target pin are board-specific and are owned by the HCI driver layer
//! so that the device driver stays portable.
//
typedef void (*em9305_cm_pwm_ctrl_fun)(bool enable);

//
//! BLE controller callback structure
//
typedef struct
{
    /**
     *************************************************************************************
     * @brief Reset the BLE controller via RESET GPIO.
     *
     *************************************************************************************
     */
    void (*reset)(void);

    /**
     *************************************************************************************
    * @brief Full-duplex SPI exchange used for both TX and RX (HCI commands,
    *        responses, NVM read/write).
    *
    *************************************************************************************
    */
    bt_spi_transceive_fun transceive;

    /**
     *************************************************************************************
     * @brief When true, skip bundled-firmware auto-update during init.
     *
     * Use for early board bring-up (e.g. EXTREF wake) when a later HCI init pass
     * will perform the update with full board hooks (CLKREQ, CM PWM, etc.).
     *
     *************************************************************************************
     */
    bool skip_fw_update;
} am_devices_em9305_callback_t;

//*****************************************************************************
//
//! @brief Initialize the BLE controller driver.
//!
//! Brings the EM9305 out of reset and waits for the active-state vendor event.
//! If normal boot fails (e.g. NVM left corrupt by an interrupted FW update),
//! a forced configuration-mode reflash of the bundled image is attempted
//! automatically when @a skip_fw_update is false and CM PWM is registered.
//!
//! When the NVM version matches the bundled image, a CRC integrity check is
//! performed on every init; mismatches trigger a full reprogram.
//!
//! @param cb pointer of BLE Controller callback
//!
//! @return Status of initialization
//
//*****************************************************************************
uint32_t am_devices_em9305_init(am_devices_em9305_callback_t *cb);

//*****************************************************************************
//
//! @brief Read BLE firmware version from NVM info page 1.
//!
//! @param image_ver pointer to receive 32-bit version (A.B.C.D packed in BE
//! order per legacy print).
//!
//! @return AM_DEVICES_EM9305_STATUS_SUCCESS on success, or another
//! am_devices_em9305_status_t code.
//
//*****************************************************************************
uint32_t am_devices_em9305_get_fw_version(uint32_t *image_ver);

//*****************************************************************************
//
//! @brief Deinitialize the BLE controller driver.
//!
//! This function puts the EM9305 in reset state and clears internal state.
//!
//! @return Status of deinitialization
//
//*****************************************************************************
uint32_t am_devices_em9305_deinit(void);

//*****************************************************************************
//
//! @brief Set the reset state of EM9305.
//!
//! @param data - reset state (true = set, false = clear)
//
//*****************************************************************************
void am_devices_em9305_set_reset_state(bool data);

//*****************************************************************************
//
//! @brief Get the reset state of EM9305.
//!
//! @return Current reset state
//
//*****************************************************************************
bool am_devices_em9305_get_reset_state(void);

//*****************************************************************************
//
//! @brief Send data to EM9305 via SPI.
//!
//! @param pui8Values - pointer to data buffer
//! @param ui32NumBytes - number of bytes to send
//! @param transceive - SPI transceive function pointer
//!
//! @return Status of transmission
//
//*****************************************************************************
int am_devices_em9305_blocking_write(uint8_t *pui8Values, uint16_t ui32NumBytes,
                                      bt_spi_transceive_fun transceive);

//*****************************************************************************
//
//! @brief Check if EM9305 received the active state entered event.
//!
//! @param data - pointer to received data
//! @param len - length of received data
//!
//! @return true if active state entered event was received
//
//*****************************************************************************
bool am_devices_em9305_check_active_state_event(uint8_t *data, uint16_t len);

//*****************************************************************************
//
//! @brief Register GPIO operations.
//!
//! This function should be called by the driver layer to register GPIO
//! operations before calling am_devices_em9305_init().
//!
//! @param set_reset - function to set reset pin
//! @param get_reset - function to get reset pin state
//! @param get_irq - function to get IRQ pin state
//! @param cs_set - function to set CS pin
//! @param cs_release - function to release CS pin
//
//*****************************************************************************
void am_devices_em9305_register_gpio_ops(void (*set_reset)(bool),
                                          bool (*get_reset)(void),
                                          bool (*get_irq)(void),
                                          void (*cs_set)(void),
                                          void (*cs_release)(void));

//*****************************************************************************
//
//! @brief Get SPI TX in progress status.
//!
//! @return true if SPI TX is in progress
//
//*****************************************************************************
bool am_devices_em9305_get_spi_tx_status(void);

//*****************************************************************************
//
//! @brief Reset the EM9305 controller.
//!
//! This function performs a hardware reset sequence on the EM9305 controller.
//
//*****************************************************************************
void am_devices_em9305_controller_reset(void);

//*****************************************************************************
//
//! @brief Register the CM (configuration mode) GPIO set function.
//!
//! Must be called before am_devices_em9305_init() if firmware update is used.
//!
//! @param set_cm - function to drive cm GPIO (true=assert, false=deassert)
//
//*****************************************************************************
void am_devices_em9305_register_cm_gpio(void (*set_cm)(bool));

//*****************************************************************************
//
//! @brief Register the CM (configuration mode) PWM start/stop callback.
//!
//! The EM9305 enters configuration mode when it samples a 30 kHz signal on
//! the CM pad during boot. The signal source is board-specific (e.g. CTIMER
//! PWM routed to a CT-capable pad) so the driver delegates the start/stop to
//! a callback registered by the HCI driver layer.
//!
//! Must be called before am_devices_em9305_init() if firmware update is to be
//! performed; if not registered, the firmware-update path returns an error.
//!
//! @param cm_pwm - function to start (true) or stop (false) the 30 kHz signal
//
//*****************************************************************************
void am_devices_em9305_register_cm_pwm_ops(em9305_cm_pwm_ctrl_fun cm_pwm);

//*****************************************************************************
//
//! @brief Enable or disable the EM9305 sleep option.
//!
//! Sends the vendor-specific HCI_VSC_SET_SLEEP_OPTION command (opcode 0xFC49)
//! to the BLE controller. When sleep is enabled the controller may enter low
//! power state between BLE activity, which is required for low-power BLE use
//! cases.
//!
//! @param enable - true to enable controller sleep, false to disable
//!
//! @return AM_DEVICES_EM9305_STATUS_SUCCESS on success
//
//*****************************************************************************
uint32_t am_devices_em9305_sleep_set(bool enable);

//*****************************************************************************
//
//! @brief Update EM9305 firmware if the bundled image is newer.
//!
//! When @a force is false and the NVM version already matches @a image_ver,
//! enters configuration mode and CRC-verifies every image record before
//! skipping the flash.  Corrupt NVM (e.g. from an interrupted update) is
//! reprogrammed even when the version word matches.  Always leaves
//! configuration mode with a controller reset.
//!
//! @param pFwImage     Array of ImageRecord pointers (from ble_fw_image_em9305.h)
//! @param record_size  Number of records in pFwImage
//! @param erase_pages  Array of NvmPage descriptors to erase
//! @param erase_size   Number of entries in erase_pages
//! @param image_ver    Version word of the bundled image
//! @param force        true to update even if already at same version
//!
//! @return AM_DEVICES_EM9305_STATUS_SUCCESS on success
//
//*****************************************************************************
uint32_t am_devices_em9305_update_fw(ImageRecord **pFwImage, uint8_t record_size,
                                     NvmPage *erase_pages, uint32_t erase_size,
                                     uint32_t image_ver, bool force);

//*****************************************************************************
//
//! @brief Program the EM9305 HF crystal trim value into NVM info page 2.
//!
//! The trim value occupies bits 12:7 of REG_RF_XO_SEQ_DIG.  The controller
//! uses the record in NVM info page 3 by default; a valid record in info
//! page 2 overrides it, so the custom trim is written to page 2.  This
//! enters configuration mode and resets the controller, then waits for it to
//! return to active state.
//!
//! @param trim_value   6-bit trim value to program.
//! @param force_update when false, skip the update if the live register
//!                     already holds the requested trim value.
//!
//! @return AM_DEVICES_EM9305_STATUS_SUCCESS on success.
//
//*****************************************************************************
uint32_t am_devices_em9305_crystal_trim_set(uint8_t trim_value, bool force_update);

#ifdef __cplusplus
}
#endif

#endif // AM_DEVICES_EM9305_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
