//*****************************************************************************
//
//! @file am_hal_cc312_chacha_test.c
//!
//! @brief Functional test for the CC312 ChaCha20 HAL (am_hal_cc312_chacha).
//!
//! Runs on Apollo510 target hardware. Validates the hardware-accelerated
//! ChaCha20 stream cipher against the RFC 8439 (Section 2.4.2) known-answer
//! test vector, plus a decrypt round-trip and a split/streaming case that
//! exercises block-counter continuation across calls (which depends on the
//! store-state + DMA-coherency path being correct).
//!
//! All DMA buffers are 32-byte (DCACHE line) aligned and padded, per the
//! am_hal_cc312_chacha DMA contract.
//
//*****************************************************************************

//*****************************************************************************
//
// ${copyright}
//
// This is part of revision ${version} of the AmbiqSuite Development Package.
//
//*****************************************************************************

#include "am_mcu_apollo.h"
#include "am_util.h"
#include "am_hal_cc312_chacha.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

//*****************************************************************************
//
// RFC 8439 Section 2.4.2 known-answer test vector.
//
//*****************************************************************************

//
//! 256-bit key: 00 01 02 ... 1f.
//
static const uint8_t g_chacha_key[CHACHA_KEY_SIZE] =
{
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
    0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f
};

//
//! 96-bit nonce: 00 00 00 00 00 00 00 4a 00 00 00 00.
//
static const uint8_t g_chacha_nonce[CHACHA_NONCE_96_SIZE] =
{
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4a, 0x00, 0x00, 0x00, 0x00
};

//
//! Initial block counter.
//
#define CHACHA_TEST_INITIAL_COUNTER     1

//
//! Plaintext (114 bytes).
//
static const uint8_t g_chacha_plaintext[114] =
    "Ladies and Gentlemen of the class of '99: If I could offer you only "
    "one tip for the future, sunscreen would be it.";

//
//! Expected ciphertext (RFC 8439 Section 2.4.2), verified against a reference
//! ChaCha20 implementation.
//
static const uint8_t g_chacha_expected_ct[114] =
{
    0x6e, 0x2e, 0x35, 0x9a, 0x25, 0x68, 0xf9, 0x80, 0x41, 0xba, 0x07, 0x28,
    0xdd, 0x0d, 0x69, 0x81, 0xe9, 0x7e, 0x7a, 0xec, 0x1d, 0x43, 0x60, 0xc2,
    0x0a, 0x27, 0xaf, 0xcc, 0xfd, 0x9f, 0xae, 0x0b, 0xf9, 0x1b, 0x65, 0xc5,
    0x52, 0x47, 0x33, 0xab, 0x8f, 0x59, 0x3d, 0xab, 0xcd, 0x62, 0xb3, 0x57,
    0x16, 0x39, 0xd6, 0x24, 0xe6, 0x51, 0x52, 0xab, 0x8f, 0x53, 0x0c, 0x35,
    0x9f, 0x08, 0x61, 0xd8, 0x07, 0xca, 0x0d, 0xbf, 0x50, 0x0d, 0x6a, 0x61,
    0x56, 0xa3, 0x8e, 0x08, 0x8a, 0x22, 0xb6, 0x5e, 0x52, 0xbc, 0x51, 0x4d,
    0x16, 0xcc, 0xf8, 0x06, 0x81, 0x8c, 0xe9, 0x1a, 0xb7, 0x79, 0x37, 0x36,
    0x5a, 0xf9, 0x0b, 0xbf, 0x74, 0xa3, 0x5b, 0xe6, 0xb4, 0x0b, 0x8e, 0xed,
    0xf2, 0x78, 0x5e, 0x42, 0x87, 0x4d
};

#define CHACHA_TEST_LEN                 ((uint32_t)sizeof(g_chacha_plaintext))

//
//! Cache-line-aligned, padded work buffers for DMA.
//
#define CHACHA_TEST_BUF_BYTES           128     // >= 114, multiple of 32
static uint8_t g_chacha_in[CHACHA_TEST_BUF_BYTES]  __attribute__((aligned(AM_HAL_CC312_DMA_ALIGNMENT)));
static uint8_t g_chacha_out[CHACHA_TEST_BUF_BYTES] __attribute__((aligned(AM_HAL_CC312_DMA_ALIGNMENT)));

//*****************************************************************************
//
//! @brief Run the CC312 ChaCha20 HAL test suite.
//!
//! @return Number of failed sub-tests (0 == all passed).
//
//*****************************************************************************
uint32_t
am_hal_cc312_chacha_test_run(void)
{
    am_hal_cc312_chacha_context_t ctx;
    uint32_t status;
    uint32_t failures = 0;

    //
    // --- Test 1: single-shot encryption against the RFC 8439 vector. ---
    //
    am_hal_cc312_chacha_context_init(&ctx);
    status  = am_hal_cc312_chacha_setkey(&ctx, g_chacha_key, CHACHA_KEY_SIZE * 8);
    status |= am_hal_cc312_chacha_set_nonce(&ctx, g_chacha_nonce,
                                            AM_HAL_CHACHA_NONCE_SIZE_96,
                                            CHACHA_TEST_INITIAL_COUNTER);

    memset(g_chacha_in, 0, sizeof(g_chacha_in));
    memset(g_chacha_out, 0, sizeof(g_chacha_out));
    memcpy(g_chacha_in, g_chacha_plaintext, CHACHA_TEST_LEN);

    status |= am_hal_chacha_crypt(&ctx, CHACHA_TEST_LEN, g_chacha_in, g_chacha_out);

    if ((status != AM_HAL_STATUS_SUCCESS) ||
        (memcmp(g_chacha_out, g_chacha_expected_ct, CHACHA_TEST_LEN) != 0))
    {
        am_util_stdio_printf("CHACHA TEST 1 (encrypt KAT): FAIL (status=0x%08X)\n", status);
        failures++;
    }
    else
    {
        am_util_stdio_printf("CHACHA TEST 1 (encrypt KAT): PASS\n");
    }
    am_hal_cc312_chacha_free(&ctx);

    //
    // --- Test 2: decrypt round-trip (ciphertext -> plaintext). ---
    //
    am_hal_cc312_chacha_context_init(&ctx);
    status  = am_hal_cc312_chacha_setkey(&ctx, g_chacha_key, CHACHA_KEY_SIZE * 8);
    status |= am_hal_cc312_chacha_set_nonce(&ctx, g_chacha_nonce,
                                            AM_HAL_CHACHA_NONCE_SIZE_96,
                                            CHACHA_TEST_INITIAL_COUNTER);

    memset(g_chacha_in, 0, sizeof(g_chacha_in));
    memset(g_chacha_out, 0, sizeof(g_chacha_out));
    memcpy(g_chacha_in, g_chacha_expected_ct, CHACHA_TEST_LEN);

    status |= am_hal_chacha_crypt(&ctx, CHACHA_TEST_LEN, g_chacha_in, g_chacha_out);

    if ((status != AM_HAL_STATUS_SUCCESS) ||
        (memcmp(g_chacha_out, g_chacha_plaintext, CHACHA_TEST_LEN) != 0))
    {
        am_util_stdio_printf("CHACHA TEST 2 (decrypt round-trip): FAIL (status=0x%08X)\n", status);
        failures++;
    }
    else
    {
        am_util_stdio_printf("CHACHA TEST 2 (decrypt round-trip): PASS\n");
    }
    am_hal_cc312_chacha_free(&ctx);

    //
    // --- Test 3: split/streaming encryption (block-counter continuation). ---
    //     First a whole 64-byte block, then the 50-byte remainder. The
    //     hardware advances the counter and store-state writes it back, so the
    //     second call must continue the same keystream.
    //
    am_hal_cc312_chacha_context_init(&ctx);
    status  = am_hal_cc312_chacha_setkey(&ctx, g_chacha_key, CHACHA_KEY_SIZE * 8);
    status |= am_hal_cc312_chacha_set_nonce(&ctx, g_chacha_nonce,
                                            AM_HAL_CHACHA_NONCE_SIZE_96,
                                            CHACHA_TEST_INITIAL_COUNTER);

    memset(g_chacha_in, 0, sizeof(g_chacha_in));
    memset(g_chacha_out, 0, sizeof(g_chacha_out));
    memcpy(g_chacha_in, g_chacha_plaintext, CHACHA_TEST_LEN);

    status |= am_hal_chacha_crypt(&ctx, CHACHA_BLOCK_SIZE,
                                  g_chacha_in, g_chacha_out);
    status |= am_hal_chacha_crypt(&ctx, CHACHA_TEST_LEN - CHACHA_BLOCK_SIZE,
                                  g_chacha_in + CHACHA_BLOCK_SIZE,
                                  g_chacha_out + CHACHA_BLOCK_SIZE);

    if ((status != AM_HAL_STATUS_SUCCESS) ||
        (memcmp(g_chacha_out, g_chacha_expected_ct, CHACHA_TEST_LEN) != 0))
    {
        am_util_stdio_printf("CHACHA TEST 3 (split streaming): FAIL (status=0x%08X)\n", status);
        failures++;
    }
    else
    {
        am_util_stdio_printf("CHACHA TEST 3 (split streaming): PASS\n");
    }
    am_hal_cc312_chacha_free(&ctx);

    am_util_stdio_printf("CHACHA HAL TEST: %s (%u failure(s))\n",
                         (failures == 0) ? "ALL PASS" : "FAILURES", failures);

    return failures;
}
