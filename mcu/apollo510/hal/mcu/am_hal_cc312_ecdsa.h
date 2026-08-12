//*****************************************************************************
//
//! @file am_hal_cc312_ecdsa.h
//!
//! @brief Hardware abstraction for CryptoCell-312 ECDSA sign / verify
//!
//! Purpose: Register-level port of the CC312 runtime ECDSA sign and verify
//! (ec_wrst_dsa.c, pka_ec_wrst_dsa_verify.c) on top of the PKA and EC layers.
//!
//! The ephemeral key (k) is supplied by the caller: ECDSA security depends on k
//! being uniformly random in [1, n-1] and never reused. Generate it from the
//! CC312 TRNG (or another CSPRNG) for production use; deterministic k is useful
//! for known-answer testing.
//!
//! @addtogroup cc312_ecdsa_ap510 CC312 ECDSA - Sign / Verify
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

#ifndef AM_HAL_CC312_ECDSA_H
#define AM_HAL_CC312_ECDSA_H

#include "am_hal_cc312_ecc.h"

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! @brief Generate an ECDSA signature (r, s) over a message hash.
//!
//! @param pDomain       - EC domain (e.g. secp256r1).
//! @param privKey       - Private key d (little-endian words, order size).
//! @param ephemKey      - Ephemeral key k (little-endian words, order size);
//!                        must be uniformly random in [1, n-1] for security.
//! @param hash          - Message hash, big-endian bytes.
//! @param hashSizeBytes - Length of @p hash in bytes.
//! @param sigR          - Output signature r (little-endian words, order size).
//! @param sigS          - Output signature s (little-endian words, order size).
//!
//! @return AM_HAL_STATUS_SUCCESS, or an error (e.g. a degenerate k - retry with
//!         a fresh k).
//
//*****************************************************************************
extern uint32_t am_hal_cc312_ecdsa_sign(const am_hal_cc312_ecc_domain_t *pDomain,
                                        const uint32_t *privKey,
                                        const uint32_t *ephemKey,
                                        const uint8_t *hash,
                                        uint32_t hashSizeBytes,
                                        uint32_t *sigR,
                                        uint32_t *sigS);

//*****************************************************************************
//
//! @brief Generate an ECDSA signature, drawing the ephemeral key from the PUF.
//!
//! Generates k uniformly in [1, n-1] from the on-chip PUF entropy source
//! (am_hal_puf_get_entropy) by rejection sampling, then signs. The PUF entropy
//! source must be initialized (am_hal_puf_entropy_init()) and the Crypto module
//! powered before calling.
//!
//! @param pDomain       - EC domain (e.g. secp256r1).
//! @param privKey       - Private key d (little-endian words, order size).
//! @param hash          - Message hash, big-endian bytes.
//! @param hashSizeBytes - Length of @p hash in bytes.
//! @param sigR          - Output signature r (little-endian words, order size).
//! @param sigS          - Output signature s (little-endian words, order size).
//!
//! @return AM_HAL_STATUS_SUCCESS or an error code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_ecdsa_sign_rnd(const am_hal_cc312_ecc_domain_t *pDomain,
                                            const uint32_t *privKey,
                                            const uint8_t *hash,
                                            uint32_t hashSizeBytes,
                                            uint32_t *sigR,
                                            uint32_t *sigS);

//*****************************************************************************
//
//! @brief Verify an ECDSA signature (r, s) over a message hash.
//!
//! @param pDomain       - EC domain (e.g. secp256r1).
//! @param pubKeyX       - Public key Q.x (little-endian words, modulus size).
//! @param pubKeyY       - Public key Q.y (little-endian words, modulus size).
//! @param hash          - Message hash, big-endian bytes.
//! @param hashSizeBytes - Length of @p hash in bytes.
//! @param sigR          - Signature r (little-endian words, order size).
//! @param sigS          - Signature s (little-endian words, order size).
//!
//! @return AM_HAL_STATUS_SUCCESS if the signature is valid, otherwise an error
//!         (AM_HAL_STATUS_FAIL for an invalid signature).
//
//*****************************************************************************
extern uint32_t am_hal_cc312_ecdsa_verify(const am_hal_cc312_ecc_domain_t *pDomain,
                                          const uint32_t *pubKeyX,
                                          const uint32_t *pubKeyY,
                                          const uint8_t *hash,
                                          uint32_t hashSizeBytes,
                                          const uint32_t *sigR,
                                          const uint32_t *sigS);

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_CC312_ECDSA_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
