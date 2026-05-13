//*****************************************************************************
//
//! @file am_devices_em9305.c
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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include "am_mcu_apollo.h"
#include "am_devices_em9305.h"
#include "ble_fw_image_em9305.h"

LOG_MODULE_REGISTER(am_devices_em9305, CONFIG_LOG_DEFAULT_LEVEL);

int am_devices_em9305_blocking_write(uint8_t *pui8Values, uint16_t ui32NumBytes, bt_spi_transceive_fun transceive);
bool am_devices_em9305_get_spi_tx_status(void);

/* EM9305 NVM / HCI VSC definitions (aligned with AmbiqSuite am_devices_em9305)
 */
#define EM9305_NVM_INFO_PAGE1_START_ADDR 0x402000U
#define EM9305_FW_VER_NUM_LEN 4U
#define EM9305_FW_VER_INVALID 0xFFFFFFFFU
#define HCI_VSC_READ_AT_ADDRESS_OPCODE 0xFD01U
#define HCI_VSC_WRITE_AT_ADDRESS_OPCODE 0xFD03U
#define HCI_VSC_NVM_ERASE_NVM_MAIN_OPCODE 0xFD06U
#define HCI_VSC_NVM_ERASE_PAGE_OPCODE 0xFD07U
#define HCI_VSC_NVM_ERASE_PAGE_CMD_LENGTH 2U
#define HCI_VSC_NVM_ERASE_NVM_MAIN_CMD_LENGTH 0U
#define AM_DEVICES_EM9305_HCI_CMD_PKT 0x01U
#define EM9305_CMD_IRQ_WAIT_MS 5000U
#define EM9305_ACTIVE_STATE_TIMEOUT_MS 5000U
#define EM9305_FW_UPDATE_WRITE_PACKET_SIZE 128U
#define EM9305_NVM_INFO_READ_LEN 248U
#define EM9305_NVM_INFO_AREA 1U
#define EM9305_NVM_INFO_AREA_PAGE_1 1U

//*****************************************************************************
//
// Global variables.
//
//*****************************************************************************

//! Active state entered event signature
static uint8_t active_state_entered_evt[] = {0x04, 0xFF, 0x01, 0x01};

//! SPI lock when a transmission is in progress
static bool spiTxInProgress;

//! EM9305 status ok flag
static volatile bool Em9305status_ok;

//! EM9305 callback structure
static am_devices_em9305_callback_t g_Em9305cb;

//! SPI transceive used for vendor HCI during init (NVM read)
static bt_spi_transceive_fun g_transceive;

//! GPIO operations structure (to be initialized by the driver layer)
static struct
{
    void (*set_reset)(bool state);
    bool (*get_reset)(void);
    bool (*get_irq)(void);
    void (*cs_set)(void);
    void (*cs_release)(void);
    void (*set_cm)(bool state);
} g_gpio_ops;

//! Firmware update image records (set by am_devices_em9305_update_fw)
static ImageRecord **g_fw_image_records;
static uint8_t g_fw_image_record_size;
static NvmPage *g_fw_erase_pages;
static uint32_t g_fw_erase_pages_size;

//*****************************************************************************
//
//! @brief Set the reset state of EM9305.
//!
//! @param data - reset state (true = set, false = clear)
//
//*****************************************************************************
void am_devices_em9305_set_reset_state(bool data)
{
    if (g_gpio_ops.set_reset)
    {
        g_gpio_ops.set_reset(data);
    }
}

//*****************************************************************************
//
//! @brief Get the reset state of EM9305.
//!
//! @return Current reset state
//
//*****************************************************************************
bool am_devices_em9305_get_reset_state(void)
{
    if (g_gpio_ops.get_reset)
    {
        return g_gpio_ops.get_reset();
    }
    return false;
}

//*****************************************************************************
//
//! @brief Get the IRQ pin state.
//!
//! @return IRQ pin state
//
//*****************************************************************************
static bool irq_pin_state(void)
{
    if (g_gpio_ops.get_irq)
    {
        return g_gpio_ops.get_irq();
    }
    return false;
}

//*****************************************************************************
//
//! @brief Set the CS pin (active high for EM9305).
//
//*****************************************************************************
static void bt_em9305_cs_set(void)
{
    if (g_gpio_ops.cs_set)
    {
        g_gpio_ops.cs_set();
    }
}

//*****************************************************************************
//
//! @brief Release the CS pin.
//
//*****************************************************************************
static void bt_em9305_cs_release(void)
{
    if (g_gpio_ops.cs_release)
    {
        g_gpio_ops.cs_release();
    }
}

//*****************************************************************************
//
//! @brief Read one or more HCI packets from EM9305 over SPI (Apollo5 protocol).
//
//*****************************************************************************
static int em9305_spi_rcv(uint8_t *data, uint16_t size_max, uint16_t *len)
{
    uint8_t sCommand[2] = {EM9305_SPI_HEADER_RX, 0x00};
    uint8_t ui8RxBytes = 0;
    int ret;

    *len = 0;
    if (am_devices_em9305_get_spi_tx_status())
    {
        return AM_DEVICES_EM9305_TX_BUSY;
    }
    if (!irq_pin_state())
    {
        return AM_DEVICES_EM9305_NO_DATA_TX;
    }
    if (!g_transceive)
    {
        return AM_DEVICES_EM9305_STATUS_ERROR;
    }
    do {
        uint8_t sStas[2] = {0};
        for (uint32_t i = 0; i < EM9305_STS_CHK_CNT_MAX; i++) 
        {
            bt_em9305_cs_set();
            ret = g_transceive(sCommand, 2, sStas, 2);
            if (ret != AM_HAL_STATUS_SUCCESS) 
            {
                return AM_DEVICES_EM9305_CMD_TRANSFER_ERROR;
            }
            if ((sStas[0] == EM9305_STS1_READY_VALUE) && (sStas[1] != 0x00)) 
            {
                break;
            }
            bt_em9305_cs_release();
        }

        if ((sStas[0] != EM9305_STS1_READY_VALUE) || (sStas[1] == 0x00)) 
        {
            bt_em9305_cs_release();
            /* (0x00,0x00) and (0xC0,0x00) are common while the link wakes; avoid ERR spam. */
            if ((sStas[0] == 0U && sStas[1] == 0U) || (sStas[0] == EM9305_STS1_READY_VALUE && sStas[1] == 0U))
            {
                LOG_DBG("EM9305 not ready yet (0x%02x 0x%02x)", sStas[0], sStas[1]);
            } 
            else
            {
                LOG_WRN("EM9305 unexpected status (0x%02x 0x%02x)", sStas[0], sStas[1]);
            }
            return AM_DEVICES_EM9305_NOT_READY;
        }

        ui8RxBytes = sStas[1];
        if (irq_pin_state() && (ui8RxBytes != 0))
        {
            if ((*len + ui8RxBytes) > size_max)
            {
                bt_em9305_cs_release();
                LOG_ERR("HCI RX packet too large");
                return AM_DEVICES_EM9305_DATA_LENGTH_ERROR;
            }
            ret = g_transceive(NULL, 0, data + *len, ui8RxBytes);
            if (ret != AM_HAL_STATUS_SUCCESS)
            {
                bt_em9305_cs_release();
                LOG_ERR("SPI RX failed (%d)", ret);
                return AM_DEVICES_EM9305_DATA_TRANSFER_ERROR;
            }
            *len += ui8RxBytes;
      }
      bt_em9305_cs_release();
    } while (irq_pin_state());

    return AM_DEVICES_EM9305_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Send HCI vendor command and wait for matching command complete.
//
//*****************************************************************************
static uint32_t em9305_command_write(const uint8_t *cmd, uint16_t cmd_len, uint8_t *resp, uint16_t resp_max, uint16_t *resp_len)
{
    int err;
    uint32_t t0;

    if (!g_transceive)
    {
        return AM_DEVICES_EM9305_STATUS_ERROR;
    }
    err = am_devices_em9305_blocking_write((uint8_t *)cmd, cmd_len, g_transceive);
    if (err != AM_DEVICES_EM9305_STATUS_SUCCESS)
    {
        return (uint32_t)err;
    }
    t0 = k_uptime_get_32();
    while (!irq_pin_state()) 
    {
        if ((k_uptime_get_32() - t0) > EM9305_CMD_IRQ_WAIT_MS) 
        {
            LOG_ERR("EM9305 command response timeout");
            return AM_DEVICES_EM9305_CMD_TRANSFER_ERROR;
        }
        k_busy_wait(200);
    }

    for (uint32_t attempt = 0; attempt < 64U; attempt++) 
    {
        *resp_len = 0;
        err = em9305_spi_rcv(resp, resp_max, resp_len);
        if (err != AM_DEVICES_EM9305_STATUS_SUCCESS)
        {
            return (uint32_t)err;
        }
        if (*resp_len < 6U) 
        {
            continue;
        }
        if ((resp[0] == 0x04U) && (resp[1] == 0x0EU) && (resp[4] == cmd[1]) && (resp[5] == cmd[2])) 
        {
            return AM_DEVICES_EM9305_STATUS_SUCCESS;
        }
    }

    LOG_ERR("EM9305 command complete not matched");
    return AM_DEVICES_EM9305_CMD_TRANSFER_ERROR;
}

//*****************************************************************************
//
//! @brief HCI VSC read at address (NVM / register).
//
//*****************************************************************************
static uint32_t read_data_cmd(uint32_t address, uint8_t *data_out, uint8_t data_len)
{
    uint8_t cmd[9];
    static uint8_t resp[EM9305_BUFFER_SIZE];
    uint16_t resp_len = 0;
    uint32_t st;

    cmd[0] = AM_DEVICES_EM9305_HCI_CMD_PKT;
    sys_put_le16(HCI_VSC_READ_AT_ADDRESS_OPCODE, &cmd[1]);
    cmd[3] = 4U + 1U;
    sys_put_le32(address, &cmd[4]);
    cmd[8] = data_len;

    st = em9305_command_write(cmd, sizeof(cmd), resp, sizeof(resp), &resp_len);
    if (st != AM_DEVICES_EM9305_STATUS_SUCCESS) 
    {
        return st;
    }

    if (resp_len < (7U + data_len))
    {
        return AM_DEVICES_EM9305_CMD_TRANSFER_ERROR;
    }

    if (resp[6] != 0U)
    {
        return AM_DEVICES_EM9305_CMD_TRANSFER_ERROR;
    }

    memcpy(data_out, &resp[7], data_len);
    return AM_DEVICES_EM9305_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Read BLE firmware version from NVM (same mechanism as AmbiqSuite).
//
//*****************************************************************************
uint32_t am_devices_em9305_get_fw_version(uint32_t *image_ver)
{
  if (!image_ver)
  {
      return AM_DEVICES_EM9305_STATUS_ERROR;
  }
  return read_data_cmd(EM9305_NVM_INFO_PAGE1_START_ADDR, (uint8_t *)image_ver, EM9305_FW_VER_NUM_LEN);
}

//*****************************************************************************
//
//! @brief Wait for EM9305 to be ready.
//
//*****************************************************************************
static void bt_em9305_wait_ready(void)
{
    uint16_t i;

    for (i = 0; i < WAIT_EM9305_RDY_TIMEOUT; i++)
    {
        if (irq_pin_state())
        {
            break;
        }
        k_busy_wait(100);
    }

    if (i >= WAIT_EM9305_RDY_TIMEOUT)
    {
        LOG_WRN("EM9305 ready timeout after %d ms", WAIT_EM9305_RDY_TIMEOUT * 100 / 1000);
    }
}

//*****************************************************************************
//
//! @brief End EM9305 TX transaction.
//
//*****************************************************************************
static void am_devices_em9305_tx_ends(void)
{
    // Deselect the EM9305
    bt_em9305_cs_release();
    // Indicates that the SPI transfer is finished
    spiTxInProgress = false;
}

//*****************************************************************************
//
//! @brief Start EM9305 TX transaction and get available buffer size.
//!
//! @param transceive - SPI transceive function pointer
//!
//! @return Available buffer size in EM9305
//
//*****************************************************************************
static uint8_t am_devices_em9305_tx_starts(bt_spi_transceive_fun transceive)
{
    uint8_t sCommand[2] = {EM9305_SPI_HEADER_TX, 0x00};
    uint8_t sStas[2] = {0, 0};
    int xact_st;

    // Indicates that a SPI transfer is in progress
    spiTxInProgress = true;

    // Select the EM9305
    bt_em9305_cs_set();

    // Wait for EM9305 ready
    bt_em9305_wait_ready();

    // Check ready again
    if (!irq_pin_state())
    {
        bt_em9305_cs_release();
        spiTxInProgress = false;
        LOG_ERR("wait em9305 ready timeout");
        return 0;
    }

    for (uint32_t i = 0; i < EM9305_STS_CHK_CNT_MAX; i++)
    {
        bt_em9305_cs_set();
        xact_st = transceive(sCommand, 2, sStas, 2);
        if (xact_st != AM_HAL_STATUS_SUCCESS) 
        {
            LOG_ERR("%s: SPI transceive failed (%d)", __func__, xact_st);
            am_devices_em9305_tx_ends();
            return 0;
        }

        if ((sStas[0] == EM9305_STS1_READY_VALUE) && (sStas[1] != 0x00))
        {
            return sStas[1];
        }
        bt_em9305_cs_release();
    }

    am_devices_em9305_tx_ends();
    return 0;
}

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
                                      bt_spi_transceive_fun transceive)
{
    uint32_t ui32ErrorStatus = AM_DEVICES_EM9305_STATUS_SUCCESS;
    int ret = -ENOTSUP;
    static uint8_t data[EM9305_BUFFER_SIZE];
    uint8_t em9305BufSize = 0;

    if (ui32NumBytes <= EM9305_BUFFER_SIZE)
    {
        for (uint32_t i = 0; i < ui32NumBytes;)
        {
            em9305BufSize = am_devices_em9305_tx_starts(transceive);
            if (em9305BufSize == 0x00)
            {
                ui32ErrorStatus = AM_DEVICES_EM9305_RX_FULL;
                LOG_ERR("EM9305_RX_FULL");
                am_devices_em9305_tx_ends();
                break;
            }
            uint32_t len = (em9305BufSize < (ui32NumBytes - i)) ? em9305BufSize : (ui32NumBytes - i);
            // Check again if there is room to send more data
            if ((len > 0) && (em9305BufSize))
            {
                memcpy(data, pui8Values + i, len);
                i += len;

                // Write to the IOM
                // Transmit the message
                ret = transceive(data, len, NULL, 0);

                if (ret != AM_HAL_STATUS_SUCCESS)
                {
                    ui32ErrorStatus = AM_DEVICES_EM9305_DATA_TRANSFER_ERROR;
                    LOG_ERR("%s: ret= %d", __func__, ret);
                }
            }
            am_devices_em9305_tx_ends();
        }
    }
    else
    {
        ui32ErrorStatus = AM_DEVICES_EM9305_DATA_LENGTH_ERROR;
        LOG_ERR("%s: error (STATUS ERROR) Packet Too Large", __func__);
    }

    return ui32ErrorStatus;
}

//*****************************************************************************
//
// Firmware update helper: HCI VSC write to NVM address
//
//*****************************************************************************
static uint32_t fw_write_data_cmd(uint32_t address, const uint8_t *data, uint8_t data_len)
{
  /* HCI pkt: 1 type + 2 opcode + 1 len + 4 addr + data (max info page write) */
  static uint8_t cmd[8U + EM9305_NVM_INFO_READ_LEN];
  static uint8_t resp[EM9305_BUFFER_SIZE];
  uint16_t resp_len = 0;
  uint32_t st;
  uint8_t total_len = (uint8_t)(4U + data_len);

  cmd[0] = AM_DEVICES_EM9305_HCI_CMD_PKT;
  sys_put_le16(HCI_VSC_WRITE_AT_ADDRESS_OPCODE, &cmd[1]);
  cmd[3] = total_len;
  sys_put_le32(address, &cmd[4]);
  memcpy(&cmd[8], data, data_len);

  st = em9305_command_write(cmd, (uint16_t)(8U + data_len), resp, sizeof(resp), &resp_len);
  if (st != AM_DEVICES_EM9305_STATUS_SUCCESS)
  {
      return st;
  }
  if (resp_len < 7U || resp[6] != 0U)
  {
      LOG_ERR("EM9305 write cmd failed, status=0x%02x", (resp_len >= 7U) ? resp[6] : 0xFFU);
      return AM_DEVICES_EM9305_CMD_TRANSFER_ERROR;
  }

  return AM_DEVICES_EM9305_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Firmware update helper: HCI VSC erase one NVM page
//
//*****************************************************************************
static uint32_t fw_erase_page_cmd(uint8_t area, uint8_t page)
{
    uint8_t cmd[6];
    static uint8_t resp[EM9305_BUFFER_SIZE];
    uint16_t resp_len = 0;
    uint32_t st;

    cmd[0] = AM_DEVICES_EM9305_HCI_CMD_PKT;
    sys_put_le16(HCI_VSC_NVM_ERASE_PAGE_OPCODE, &cmd[1]);
    cmd[3] = HCI_VSC_NVM_ERASE_PAGE_CMD_LENGTH;
    cmd[4] = area;
    cmd[5] = page;

    st = em9305_command_write(cmd, sizeof(cmd), resp, sizeof(resp), &resp_len);
    if (st != AM_DEVICES_EM9305_STATUS_SUCCESS)
    {
        return st;
    }
    if (resp_len < 7U || resp[6] != 0U)
    {
        LOG_ERR("EM9305 erase page area=%u page=%u failed, status=0x%02x", area, page, (resp_len >= 7U) ? resp[6] : 0xFFU);
        return AM_DEVICES_EM9305_CMD_TRANSFER_ERROR;
    }
    return AM_DEVICES_EM9305_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Firmware update helper: HCI VSC erase entire NVM main area
//
//*****************************************************************************
static uint32_t fw_erase_nvm_main(void)
{
    uint8_t cmd[4];
    static uint8_t resp[EM9305_BUFFER_SIZE];
    uint16_t resp_len = 0;
    uint32_t st;

    cmd[0] = AM_DEVICES_EM9305_HCI_CMD_PKT;
    sys_put_le16(HCI_VSC_NVM_ERASE_NVM_MAIN_OPCODE, &cmd[1]);
    cmd[3] = HCI_VSC_NVM_ERASE_NVM_MAIN_CMD_LENGTH;

    LOG_INF("EM9305: Erasing NVM main area...");
    st = em9305_command_write(cmd, sizeof(cmd), resp, sizeof(resp), &resp_len);
    if (st != AM_DEVICES_EM9305_STATUS_SUCCESS)
    {
        return st;
    }
    if (resp_len < 7U || resp[6] != 0U)
    {
        LOG_ERR("EM9305 erase NVM main failed, status=0x%02x", (resp_len >= 7U) ? resp[6] : 0xFFU);
        return AM_DEVICES_EM9305_CMD_TRANSFER_ERROR;
    }
    return AM_DEVICES_EM9305_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Firmware update helper: Enter configuration mode.
// Uses Apollo5 CTIMER 15 to generate a hardware 30kHz PWM on GPIO15 (CT15).
// No CPU busy-wait: the timer runs autonomously while we poll with k_msleep.
//
// Timing: HFRC_DIV64 = 1.5 MHz -> period = 50 counts (33.3 µs = 30 kHz),
//         compare1 = 25 (50 % duty cycle).
//
//*****************************************************************************
static uint32_t fw_enter_cm_mode(void)
{
    static const uint8_t entered_cm_evt[] = {0x04, 0xFF, 0x01, 0x03};
    static uint8_t buf[EM9305_BUFFER_SIZE];
    uint16_t plen;
    uint32_t t0;
    uint32_t st = AM_DEVICES_EM9305_STATUS_ERROR;
    uint8_t retry;

    LOG_INF("EM9305: Entering configuration mode...");

    /* --- Configure CTIMER 11 for 30kHz PWM (timers 14 and 15 are reserved) ---
    * HFRC_DIV64: 96 MHz / 64 = 1.5 MHz
    * Compare0 = 50  ??period = 50 / 1.5 MHz = 33.3 µs = 30 kHz
    * Compare1 = 25  ??50 % duty cycle
    */
    am_hal_timer_config_t timer_cfg;

    am_hal_timer_default_config_set(&timer_cfg);
    timer_cfg.eFunction    = AM_HAL_TIMER_FN_PWM;
    timer_cfg.eInputClock  = AM_HAL_TIMER_CLOCK_HFRC_DIV64; /* 1.5 MHz */
    timer_cfg.ui32Compare0 = 50U;
    timer_cfg.ui32Compare1 = 25U;
    am_hal_timer_config(11U, &timer_cfg);

    /* Route Timer 11 OUT0 to GPIO pad 15 via TIMER OUTCFG register */
    am_hal_timer_output_config(15U, AM_HAL_TIMER_OUTPUT_TMR11_OUT0);

    /* Switch GPIO pad 15 to CT function (FNCSEL = 6 = CT15_P15). */
    am_hal_gpio_pincfg_t ct15_cfg = am_hal_gpio_pincfg_output;
    ct15_cfg.GP.cfg_b.uFuncSel = 6U;
    am_hal_gpio_pinconfig(15U, ct15_cfg);

    /* Start 30kHz clock BEFORE the first reset pulse so EM9305 sees the
    * signal from the moment it begins booting ??matches AmbiqSuite SDK
    * generate_square_wave() which starts immediately after em9305_pulse_gpio_en(). */
    am_hal_timer_enable(11U);

#define EM9305_CM_MAX_RETRIES  5U
#define EM9305_CM_PER_RETRY_MS 200U

    for (retry = 0; retry <= EM9305_CM_MAX_RETRIES; retry++)
    {
        if (retry > 0U)
        {
            LOG_WRN("EM9305: CM mode attempt %u timed out, retrying...", retry);
        }
        /* Pulse EN/RST: assert LOW for 10 ms, then release HIGH ??EM9305 reboots */
        g_gpio_ops.set_reset(true);
        k_msleep(10);
        g_gpio_ops.set_reset(false);
        /* Poll for CM-entered event; 1 ms sleep keeps the scheduler running */
        t0 = k_uptime_get_32();
        while ((k_uptime_get_32() - t0) < EM9305_CM_PER_RETRY_MS)
        {
            if (irq_pin_state())
            {
                plen = 0;
                int rv = em9305_spi_rcv(buf, sizeof(buf), &plen);
                if (rv == AM_DEVICES_EM9305_STATUS_SUCCESS && plen >= 4U)
                {
                    if (memcmp(buf, entered_cm_evt, sizeof(entered_cm_evt)) == 0)
                    {
                        LOG_INF("EM9305: Configuration mode entered (attempt %u)", retry + 1U);
                        st = AM_DEVICES_EM9305_STATUS_SUCCESS;
                        goto cm_done;
                    }
                }
            }
            k_msleep(1);
        }
    }

cm_done:
    /* Stop the 30kHz clock; restore GPIO pad 15 to push-pull output LOW */
    am_hal_timer_disable(11U);
    am_hal_gpio_pinconfig(15U, am_hal_gpio_pincfg_output);
    am_hal_gpio_state_write(15U, AM_HAL_GPIO_OUTPUT_CLEAR);
    if (st != AM_DEVICES_EM9305_STATUS_SUCCESS)
    {
        LOG_ERR("EM9305: Timeout waiting for CM mode event after %u attempts", EM9305_CM_MAX_RETRIES + 1U);
    }

    return st;
}

//*****************************************************************************
//
// Firmware update: write image records to NVM
//
//*****************************************************************************
static uint32_t fw_write_image(void)
{
    uint32_t st = AM_DEVICES_EM9305_STATUS_SUCCESS;
    LOG_INF("EM9305: Writing firmware image...");

    for (uint8_t i = 0; i < g_fw_image_record_size; i++) 
    {
        uint32_t addr = g_fw_image_records[i]->address;
        const uint8_t *data = g_fw_image_records[i]->data;
        uint32_t remaining = g_fw_image_records[i]->length;

        while (remaining > 0U)
        {
          uint8_t chunk = (uint8_t)((remaining > EM9305_FW_UPDATE_WRITE_PACKET_SIZE) ? EM9305_FW_UPDATE_WRITE_PACKET_SIZE : remaining);
          st = fw_write_data_cmd(addr, data, chunk);
          if (st != AM_DEVICES_EM9305_STATUS_SUCCESS)
          {
              LOG_ERR("EM9305: write error at addr 0x%08x", addr);
              return st;
          }
          addr += chunk;
          data += chunk;
          remaining -= chunk;
        }
    }

    LOG_INF("EM9305: Firmware write complete");
    return AM_DEVICES_EM9305_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Firmware update: update version word in NVM info page 1
//
//*****************************************************************************
static uint32_t fw_update_version(uint32_t image_ver)
{
    uint32_t st;
    static uint8_t read_out_data[EM9305_NVM_INFO_READ_LEN];

    st = read_data_cmd(EM9305_NVM_INFO_PAGE1_START_ADDR, read_out_data, (uint8_t)EM9305_NVM_INFO_READ_LEN);
    if (st != AM_DEVICES_EM9305_STATUS_SUCCESS)
    {
        LOG_ERR("EM9305: read NVM info page1 failed");
        return st;
    }
    st = fw_erase_page_cmd((uint8_t)EM9305_NVM_INFO_AREA, (uint8_t)EM9305_NVM_INFO_AREA_PAGE_1);
    if (st != AM_DEVICES_EM9305_STATUS_SUCCESS)
    {
        LOG_ERR("EM9305: erase NVM info page1 failed");
        return st;
    }
    /* Overwrite first 4 bytes with new version */
    memcpy(read_out_data, &image_ver, EM9305_FW_VER_NUM_LEN);
    st = fw_write_data_cmd(EM9305_NVM_INFO_PAGE1_START_ADDR, read_out_data, (uint8_t)EM9305_NVM_INFO_READ_LEN);
    if (st != AM_DEVICES_EM9305_STATUS_SUCCESS)
    {
        LOG_ERR("EM9305: write NVM info page1 failed");
    }
    return st;
}

//*****************************************************************************
//
//! @brief Register the CM GPIO set function for firmware update.
//
//*****************************************************************************
void am_devices_em9305_register_cm_gpio(void (*set_cm)(bool))
{
    g_gpio_ops.set_cm = set_cm;
}

//*****************************************************************************
//
//! @brief Update EM9305 firmware if bundled image is newer than current.
//
//*****************************************************************************
uint32_t am_devices_em9305_update_fw(ImageRecord **pFwImage, uint8_t record_size, NvmPage *erase_pages, uint32_t erase_size, uint32_t image_ver, bool force)
{
    uint32_t st;
    uint32_t current_ver = 0;

    /* Store references for use by helper functions */
    g_fw_image_records = pFwImage;
    g_fw_image_record_size = record_size;
    g_fw_erase_pages = erase_pages;
    g_fw_erase_pages_size = erase_size;

    /* Read current FW version */
    st = am_devices_em9305_get_fw_version(&current_ver);
    if (st != AM_DEVICES_EM9305_STATUS_SUCCESS)
    {
        LOG_WRN("EM9305: version read failed, proceeding with update");
        current_ver = EM9305_FW_VER_INVALID;
    }
    if (!force && (current_ver == image_ver))
    {
        LOG_INF("EM9305: FW already up-to-date (0x%08x)", (unsigned int)image_ver);
        return AM_DEVICES_EM9305_STATUS_SUCCESS;
    }
    if (!force && (current_ver != EM9305_FW_VER_INVALID) && (current_ver > image_ver))
    {
        LOG_INF("EM9305: current FW 0x%08x >= image 0x%08x, skipping update", (unsigned int)current_ver, (unsigned int)image_ver);
        return AM_DEVICES_EM9305_STATUS_SUCCESS;
    }
    LOG_INF("EM9305: Updating FW from 0x%08x to 0x%08x", (unsigned int)current_ver, (unsigned int)image_ver);
    /* Enter configuration mode */
    st = fw_enter_cm_mode();
    if (st != AM_DEVICES_EM9305_STATUS_SUCCESS)
    {
        return st;
    }
    /* Erase NVM main area */
    st = fw_erase_nvm_main();
    if (st != AM_DEVICES_EM9305_STATUS_SUCCESS)
    {
        goto exit_cm;
    }
    /* Write image records */
    st = fw_write_image();
    if (st != AM_DEVICES_EM9305_STATUS_SUCCESS)
    {
        goto exit_cm;
    }
    /* Update version in NVM info page */
    st = fw_update_version(image_ver);
    if (st != AM_DEVICES_EM9305_STATUS_SUCCESS)
    {
        LOG_ERR("EM9305: version update failed");
        goto exit_cm;
    }
    LOG_INF("EM9305: FW update successful");

exit_cm:
    /* Deassert CM GPIO */
    if (g_gpio_ops.set_cm)
    {
        g_gpio_ops.set_cm(false);
    }

    return st;
}

//*****************************************************************************
//
//! @brief Initialize the BLE controller driver.
//!
//! @param cb pointer of BLE Controller callback
//!
//! @return Status of initialization
//
//*****************************************************************************
uint32_t am_devices_em9305_init(am_devices_em9305_callback_t *cb)
{
    uint8_t buf[EM9305_BUFFER_SIZE];
    uint32_t t0;

    if (!cb || !cb->write || !cb->reset || !cb->transceive)
    {
        return AM_DEVICES_EM9305_STATUS_ERROR;
    }

    // Register the callback functions
    g_Em9305cb.write = cb->write;
    g_Em9305cb.reset = cb->reset;
    g_Em9305cb.transceive = cb->transceive;
    g_transceive = cb->transceive;

    Em9305status_ok = false;
    g_Em9305cb.reset();

    t0 = k_uptime_get_32();
    while (!Em9305status_ok)
    {
        if ((k_uptime_get_32() - t0) > EM9305_ACTIVE_STATE_TIMEOUT_MS)
        {
            LOG_ERR("EM9305 active state timeout");
            return AM_DEVICES_EM9305_STATUS_ERROR;
        }

        if (irq_pin_state())
        {
            uint16_t plen = 0;
            int rv = em9305_spi_rcv(buf, EM9305_BUFFER_SIZE, &plen);
            if (rv == AM_DEVICES_EM9305_STATUS_SUCCESS && plen > 0)
            {
                am_devices_em9305_check_active_state_event(buf, plen);
            } 
            else if (rv == AM_DEVICES_EM9305_NO_DATA_TX)
            {
                k_msleep(1);
            }
        } 
        else
        {
            k_msleep(1);
        }
    }

    uint32_t image_version = 0;
    uint32_t ui32Status = am_devices_em9305_get_fw_version(&image_version);

    if ((ui32Status == AM_DEVICES_EM9305_STATUS_SUCCESS) && (image_version != EM9305_FW_VER_INVALID))
    {
      LOG_INF("BLE FW Ver: %u.%u.%u.%u", (unsigned int)((image_version & 0xFF000000U) >> 24), (unsigned int)((image_version & 0xFF0000U) >> 16), 
                                        (unsigned int)((image_version & 0xFF00U) >> 8), (unsigned int)(image_version & 0xFFU));
    }
    else
    {
      LOG_WRN("BLE FW version read failed (status %u)", (unsigned int)ui32Status);
    }

    /* Auto-update only after a successful version read (avoids treating
    * uninitialized 0x00000000 as "older than bundle" on SPI/HCI failure).
    */
    if ((ui32Status == AM_DEVICES_EM9305_STATUS_SUCCESS) && ((image_version == EM9305_FW_VER_INVALID) || (image_version < ble_fw_image_bin_ver)))
    {
      LOG_INF("EM9305: bundled FW 0x%08x > device FW 0x%08x, updating...", (unsigned int)ble_fw_image_bin_ver, (unsigned int)image_version);
      uint32_t fw_st = am_devices_em9305_update_fw( image_records, IMAGE_RECORDS_SIZE, erase_pages, erase_pages_size, ble_fw_image_bin_ver, false);

      if (fw_st == AM_DEVICES_EM9305_STATUS_SUCCESS)
      {
        LOG_INF("EM9305: FW update done, resetting controller");

        /* Reset and wait for active state again */
        Em9305status_ok = false;
        g_Em9305cb.reset();

        t0 = k_uptime_get_32();
        while (!Em9305status_ok)
        {
          if ((k_uptime_get_32() - t0) > EM9305_ACTIVE_STATE_TIMEOUT_MS)
          {
            LOG_ERR("EM9305 active state timeout after FW update");
            return AM_DEVICES_EM9305_STATUS_ERROR;
          }
          if (irq_pin_state())
          {
            uint16_t plen = 0;
            int rv = em9305_spi_rcv(buf, EM9305_BUFFER_SIZE, &plen);

            if (rv == AM_DEVICES_EM9305_STATUS_SUCCESS && plen > 0)
            {
              am_devices_em9305_check_active_state_event(buf, plen);
            } 
            else if (rv == AM_DEVICES_EM9305_NO_DATA_TX) 
            {
              k_msleep(1);
            }
          }
          else
          {
            k_msleep(1);
          }
        }
      }
      else
      {
        LOG_ERR("EM9305: FW update failed (status %u)", (unsigned int)fw_st);
        return fw_st;
      }
    }

    return AM_DEVICES_EM9305_STATUS_SUCCESS;
}

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
bool am_devices_em9305_check_active_state_event(uint8_t *data, uint16_t len)
{
    bool ret = false;

    if (memcmp(data, active_state_entered_evt, sizeof(active_state_entered_evt)) == 0)
    {
        LOG_INF("EM9305 enter active state");
        Em9305status_ok = true;
        ret = true;
    }
    return ret;
}

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
                                          void (*cs_release)(void))
{
    g_gpio_ops.set_reset = set_reset;
    g_gpio_ops.get_reset = get_reset;
    g_gpio_ops.get_irq = get_irq;
    g_gpio_ops.cs_set = cs_set;
    g_gpio_ops.cs_release = cs_release;
}

//*****************************************************************************
//
//! @brief Get SPI TX in progress status.
//!
//! @return true if SPI TX is in progress
//
//*****************************************************************************
bool am_devices_em9305_get_spi_tx_status(void)
{
    return spiTxInProgress;
}

//*****************************************************************************
//
//! @brief Deinitialize the BLE controller driver.
//!
//! This function puts the EM9305 in reset state and clears internal state.
//!
//! @return Status of deinitialization
//
//*****************************************************************************
uint32_t am_devices_em9305_deinit(void)
{
    // Hold the controller in reset state
    am_devices_em9305_set_reset_state(true);

    // Give some time for the reset to take effect
    k_sleep(K_MSEC(10));

    // Clear internal state variables
    spiTxInProgress = false;
    Em9305status_ok = false;

    // Clear callback structure
    g_Em9305cb.write = NULL;
    g_Em9305cb.reset = NULL;
    g_Em9305cb.transceive = NULL;
    g_transceive = NULL;

    LOG_INF("EM9305 deinitialized");

    return AM_DEVICES_EM9305_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Reset the EM9305 controller.
//!
//! This function performs a hardware reset sequence on the EM9305 controller.
//
//*****************************************************************************
void am_devices_em9305_controller_reset(void)
{
    // Reset the controller
    am_devices_em9305_set_reset_state(false);

    // Take controller out of reset
    k_sleep(K_MSEC(2));
    am_devices_em9305_set_reset_state(true);
    k_sleep(K_MSEC(2));
    am_devices_em9305_set_reset_state(false);
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
