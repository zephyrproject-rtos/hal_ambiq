//*****************************************************************************
//
//! @file am_hal_cc312_chacha.h
//!
//! @brief Hardware abstraction for CryptoCell-312 ChaCha20 stream cipher
//!
//! @addtogroup cc312_chacha_ap510 CC312 ChaCha - Stream Cipher
//! @ingroup apollo510_hal
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

#ifndef AM_HAL_CC312_CHACHA_H
#define AM_HAL_CC312_CHACHA_H

#include "am_hal_cc312.h"

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// ChaCha cipher constants.
//
//*****************************************************************************
#define CHACHA_BLOCK_SIZE               64  //!< ChaCha block size (bytes).
#define CHACHA_KEY_SIZE                 32  //!< ChaCha key size (256-bit, bytes).
#define CHACHA_KEY_SIZE_WORDS           (CHACHA_KEY_SIZE >> 2)  //!< Key size in 32-bit words.
#define CHACHA_NONCE_64_SIZE            8   //!< 64-bit nonce size (bytes).
#define CHACHA_NONCE_96_SIZE            12  //!< 96-bit nonce size (bytes).
#define CHACHA_NONCE_MAX_SIZE_WORDS     3   //!< Max nonce buffer size (96-bit) in words.

//*****************************************************************************
//
// ChaCha operation direction.
//
//! ChaCha is a symmetric stream cipher; encrypt and decrypt are the same
//! keystream-XOR operation. The direction is retained for API parity with the
//! AES driver and for callers that track intent.
//
//*****************************************************************************
typedef enum
{
    AM_HAL_CHACHA_ENCRYPT          = 0,             //!< Encryption.
    AM_HAL_CHACHA_DECRYPT          = 1,             //!< Decryption.
    AM_HAL_CHACHA_DIR_RESERVE32B   = 0x7FFFFFFF     //!< Reserved for 32-bit enum.
}
am_hal_cc312_chacha_direction_e;

//*****************************************************************************
//
// ChaCha nonce (IV) size selection.
//
//*****************************************************************************
typedef enum
{
    AM_HAL_CHACHA_NONCE_SIZE_64    = 0,             //!< 64-bit nonce (64-bit block counter).
    AM_HAL_CHACHA_NONCE_SIZE_96    = 1,             //!< 96-bit nonce (32-bit block counter).
    AM_HAL_CHACHA_NONCE_RESERVE32B = 0x7FFFFFFF     //!< Reserved for 32-bit enum.
}
am_hal_cc312_chacha_nonce_size_e;

//*****************************************************************************
//
// ChaCha context structure.
//
//! Declared with DCACHE-line alignment so the structure never shares a cache
//! line with an unrelated DMA buffer (see am_hal_cc312_chacha_process()).
//
//*****************************************************************************
//! ChaCha context data used by CC312 HAL.
typedef struct
{
    uint32_t keyBuf[CHACHA_KEY_SIZE_WORDS];             //!< 256-bit key buffer.
    uint32_t nonceBuf[CHACHA_NONCE_MAX_SIZE_WORDS];     //!< Nonce/IV buffer.
    uint32_t blockCounterLsb;                           //!< Block counter [31:0].
    uint32_t blockCounterMsb;                           //!< Block counter [63:32] (64-bit nonce mode).
    am_hal_cc312_chacha_nonce_size_e nonceSize;         //!< Nonce size selector.
    am_hal_cc312_chacha_direction_e dir;                //!< Encrypt/decrypt direction.
    am_hal_cc312_dma_addr_type_e inputDataAddrType;     //!< Input buffer address type.
    am_hal_cc312_dma_addr_type_e outputDataAddrType;    //!< Output buffer address type.
} __attribute__((aligned(AM_HAL_CC312_DMA_ALIGNMENT)))
am_hal_cc312_chacha_context_t;

//*****************************************************************************
//
//! @brief Enable ChaCha and DMA clocks required for CC312 ChaCha operations.
//
//*****************************************************************************
extern void am_hal_cc312_chacha_enable_clocks(void);

//*****************************************************************************
//
//! @brief Disable ChaCha and DMA clocks used by CC312 ChaCha operations.
//
//*****************************************************************************
extern void am_hal_cc312_chacha_disable_clocks(void);

extern void am_hal_cc312_chacha_set_control(am_hal_cc312_chacha_context_t *ctx);

//*****************************************************************************
//
//! @brief Initialize ChaCha context to a known state.
//!
//! @param ctx - Pointer to ChaCha context structure.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_chacha_context_init(am_hal_cc312_chacha_context_t *ctx);

//*****************************************************************************
//
//! @brief Free (clear) ChaCha context.
//!
//! @param ctx - Pointer to ChaCha context structure.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_chacha_free(am_hal_cc312_chacha_context_t *ctx);

//*****************************************************************************
//
//! @brief Set the ChaCha 256-bit key in the context.
//!
//! @param ctx      - Pointer to ChaCha context structure.
//! @param key      - Pointer to key bytes.
//! @param key_bits - Key size in bits (must be 256).
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_chacha_setkey(am_hal_cc312_chacha_context_t *ctx,
                                           const uint8_t *key,
                                           uint32_t key_bits);

//*****************************************************************************
//
//! @brief Set the ChaCha nonce and initial block counter in the context.
//!
//! @param ctx             - Pointer to ChaCha context structure.
//! @param nonce           - Pointer to nonce bytes (8 or 12 bytes).
//! @param nonce_size      - Nonce size selector (64-bit or 96-bit).
//! @param initial_counter - Initial block counter value (low 32 bits).
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_chacha_set_nonce(am_hal_cc312_chacha_context_t *ctx,
                                              const uint8_t *nonce,
                                              am_hal_cc312_chacha_nonce_size_e nonce_size,
                                              uint32_t initial_counter);

//*****************************************************************************
//
//! @brief Initialize ChaCha hardware registers for the current context.
//!
//! @param ctx - Pointer to ChaCha context structure.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_chacha_init(am_hal_cc312_chacha_context_t *ctx);

//*****************************************************************************
//
//! @brief Load context key material into ChaCha hardware key registers.
//!
//! @param ctx - Pointer to ChaCha context structure.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_chacha_load_key(am_hal_cc312_chacha_context_t *ctx);

//*****************************************************************************
//
//! @brief Load nonce/counter state from context into ChaCha hardware registers.
//!
//! @param ctx - Pointer to ChaCha context structure.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_chacha_load_state(am_hal_cc312_chacha_context_t *ctx);

//*****************************************************************************
//
//! @brief Store nonce/counter state from ChaCha hardware registers into context.
//!
//! @param ctx - Pointer to ChaCha context structure.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_chacha_store_state(am_hal_cc312_chacha_context_t *ctx);

//*****************************************************************************
//
//! @brief ChaCha20 buffer encryption/decryption.
//!
//! Processes a buffer of data using the CC312 ChaCha20 engine. The block
//! counter in @p ctx is advanced by the hardware and stored back so successive
//! calls continue the keystream. @p length need not be a multiple of the block
//! size for the final call.
//!
//! In DLLI mode (the default for both endpoints) @p input and @p output must
//! be AM_HAL_CC312_DMA_ALIGNMENT-aligned and their allocations padded up to a
//! multiple of AM_HAL_CC312_DMA_ALIGNMENT: the cache maintenance and the DLLI
//! dummy-read flush operate on whole cache lines. Misaligned buffers are
//! rejected; short allocations cannot be detected here. @p length must also be
//! less than AM_HAL_CC312_DLLI_MAX_BUFF_SIZE, which is the largest transfer the
//! DMA can express. Split longer payloads into successive calls; the stored
//! block counter keeps the keystream continuous.
//!
//! @param ctx    - Pointer to ChaCha context (key and nonce must be set).
//! @param length - Number of bytes to process.
//! @param input  - Pointer to input data.
//! @param output - Pointer to output buffer.
//!
//! @return AM_HAL_STATUS_INVALID_ARG if a DLLI buffer is misaligned.
//! @return AM_HAL_STATUS_OUT_OF_RANGE if @p length exceeds the DLLI limit.
//! @return Standard HAL status code otherwise.
//
//*****************************************************************************
extern uint32_t am_hal_chacha_crypt(am_hal_cc312_chacha_context_t *ctx,
                                    uint32_t length,
                                    const uint8_t *input,
                                    uint8_t *output);

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_CC312_CHACHA_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
