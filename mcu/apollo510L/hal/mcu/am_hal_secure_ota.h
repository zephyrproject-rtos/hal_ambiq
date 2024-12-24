//*****************************************************************************
//
//! @file am_hal_secure_ota.h
//!
//! @brief Functions for secure over-the-air.
//!
//! @addtogroup secure_ota Secure OTA Functionality
//! @ingroup apollo510L_hal
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

#ifndef AM_HAL_SECURE_OTA_H
#define AM_HAL_SECURE_OTA_H
//
//! @name Ambiq Standard Image Format related definitions
//! Magic Numbers
//! @{
//
#define AM_IMAGE_MAGIC_SBL                0xA3
// #### INTERNAL BEGIN ####
#define AM_IMAGE_MAGIC_ICV_CHAIN          0xAC
//#define AM_IMAGE_MAGIC_AM3P               0x3A
#define AM_IMAGE_MAGIC_PATCH              0xAF
#define AM_IMAGE_MAGIC_AMB_RT_KEYBANK     0xAE
//#define AM_IMAGE_MAGIC_INFO1_UPDATE       0xA0
// #### INTERNAL END ####
#define AM_IMAGE_MAGIC_SECURE             0xC0
#define AM_IMAGE_MAGIC_OEM_CHAIN          0xCC
#define AM_IMAGE_MAGIC_NONSECURE          0xCB
#define AM_IMAGE_MAGIC_INFO0              0xCF
#define AM_IMAGE_MAGIC_CONTAINER          0xC1
#define AM_IMAGE_MAGIC_KEYREVOKE          0xCE
#define AM_IMAGE_MAGIC_DOWNLOAD           0xCD
//! @}

typedef enum
{
    AM_PROG_INFO0_ACTIVE = 0, // program active info0 ( mram/ OTP)
    AM_PROG_INFO0_OTP,  // program info0 opt
    AM_PROG_INFO0_MRAM, //program info0 mram
    AM_PROG_INFO0_BOTH // program both info0 opt and info0 mram
} am_info0_prog_dest_e;

//
//! OTA Image HDR Info
//
typedef struct
{
    union
    {
        uint32_t     ui32;
        struct
        {
            uint32_t    blobSize     : 22;
            uint32_t    rsvd         : 4;
            uint32_t    crcCheck     : 1;
            uint32_t    enc          : 1;
            uint32_t    authCheck    : 1;
            uint32_t    ccIncluded   : 1;
            uint32_t    ambiq        : 1;
            uint32_t    rsvd1        : 1;
        } s;
    } w0;
    uint32_t        crc;
    union
    {
        uint32_t    ui32;
        uint32_t    ui32Resv;
        struct
        {
            uint32_t    authKeyIdx   : 8;
            uint32_t    encKeyIdx    : 8;
            uint32_t    authAlgo     : 4;
            uint32_t    encAlgo      : 4;
            uint32_t    resvd        : 8;
        } s;
    } w2;
    union
    {
        uint32_t ui32;
    } w3;
} am_image_hdr_common_t;

//
//! OTA Image Encryption 2
//
typedef union
{
    uint32_t        ui32Words[12];
    uint8_t         ui8Bytes[48];
    struct
    {
        uint32_t        kek[4];
        uint32_t        iv[4];
        uint32_t        rsvd[4];
    } aesCbc128;
    struct
    {
        uint32_t        kek[4];
        uint32_t        iv[4];
        uint32_t        rsvd[4];
    } aesCtr128;
#if 0
    struct
    {
        uint32_t        kek[4];
        uint32_t        tag[4];
        uint32_t        nonce[3];
        uint32_t        rsvd[1];
    } aesCcm128;
#endif
} am_image_enc_info_t;

//
//! OTA Image Authentication Info
//
typedef union
{
    uint8_t        ui8Signature[384];
    uint32_t       ui32Signature[96];
} am_image_auth_info_t;

//
//! OTA Image Option 0
//
typedef union
{
    uint32_t   ui32;
    uint8_t    ui8[4];
    struct
    {
        uint32_t    magicNum    : 8;
        uint32_t    ccSize      : 11;
        uint32_t    rsvd        : 13;
    } s;
    struct
    {
        uint32_t    magicNum    : 8;
        uint32_t    ota         : 1; // To initiate SBL based updates
        uint32_t    sblOta      : 1; // To initiate SBL self OTA using SBL
        uint32_t    brOta       : 1; // To initiate BootROM based updates
        uint32_t    rsvd        : 21;
    } wired;
// #### INTERNAL BEGIN ####
    struct
    {
        uint32_t    magicNum    : 8;
        uint32_t    wordOffset  : 8;
        uint32_t    sizeWords   : 8;
        uint32_t    rsvd        : 8;
    } rtKeyBank;
// #### INTERNAL END ####
} am_image_opt0_t;

//
//! OTA Image Option 1
//
typedef union
{
    uint32_t   ui32;
    struct
    {
        uint32_t    rsvd        : 2;
        uint32_t    loadAddrMsb : 30;
    } fw;
    struct
    {
        uint32_t    offset      : 12;
        uint32_t    size        : 12;
        uint32_t    progDest    : 2; //info0 program destination ( mram, otp, active, both)
        uint32_t    rsvd        : 6;
    } info0;
// #### INTERNAL BEGIN ####
    struct
    {
        uint32_t    offset      : 16;
        uint32_t    rsvd        : 16;
    } patch;
    struct
    {
        uint32_t   size : 16;
        uint32_t   rsvd : 16;
    } sbl;
// #### INTERNAL END ####
    struct
    {
        uint32_t    rsvd        : 2;
        uint32_t    loadAddrMsb : 30;
    } wired;
} am_image_opt1_t;

//
//! OTA Image Option 2
//
typedef union
{
    uint32_t   ui32;
    uint32_t   ui32Key;
// #### INTERNAL BEGIN ####
    struct
    {
        uint32_t    authKeyIdx   : 8;
        uint32_t    encKeyIdx    : 8;
        uint32_t    authAlgo     : 4;
        uint32_t    encAlgo      : 4;
        uint32_t    resvd        : 8;
    } sbl;
// #### INTERNAL END ####
} am_image_opt2_t;

//
//! OTA Image Option 3
//
typedef union
{
    uint32_t   ui32;
} am_image_opt3_t;

//
//! OTA Image Info
//
typedef struct
{
   am_image_opt0_t  opt0;
   am_image_opt1_t  opt1;
   am_image_opt2_t  opt2;
   am_image_opt3_t  opt3;
} am_image_opt_info_t;

// #### INTERNAL BEGIN ####
// #### INTERNAL END ####

//
//! Maximum number of OTAs
//
#define AM_HAL_SECURE_OTA_MAX_OTA           8

typedef struct
{
    uint32_t upgrade[AM_HAL_SECURE_OTA_MAX_OTA + 1];
} am_hal_otadesc_t;

// Reserved magic numbers allowed to be used by customer's own bootloader
//#define AM_IMAGE_MAGIC_CUST(x)   ((((x) & 0xF0) == 0xC0) && ((x) != 0xC0) && ((x) != 0xCC) && ((x) != 0xCB) && ((x) != 0xCF))
#define AM_IMAGE_MAGIC_CUST(x)   (((x) == AM_IMAGE_MAGIC_INFO0)         ||  \
                                  ((x) == AM_IMAGE_MAGIC_SECURE)        ||  \
                                  ((x) == AM_IMAGE_MAGIC_NONSECURE)     ||  \
                                  ((x) == AM_IMAGE_MAGIC_DOWNLOAD)      ||  \
                                  ((x) == AM_IMAGE_MAGIC_KEYREVOKE)     ||  \
                                  ((x) == AM_IMAGE_MAGIC_OEM_CHAIN)     ||  \
                                  ((x) == AM_IMAGE_MAGIC_CONTAINER))
// #### INTERNAL BEGIN ####
#define AM_IMAGE_MAGIC_AMBQ(x)   (((x) == AM_IMAGE_MAGIC_SBL)           ||  \
                                  ((x) == AM_IMAGE_MAGIC_PATCH)         ||  \
                                  ((x) == AM_IMAGE_MAGIC_AMB_RT_KEYBANK) ||  \
                                  ((x) == AM_IMAGE_MAGIC_ICV_CHAIN))

#define AM_IMAGE_MAGIC_KNOWN(x)  (AM_IMAGE_MAGIC_CUST((x)) || AM_IMAGE_MAGIC_AMBQ((x)))
// #### INTERNAL END ####

//
//! OTA Upgrade related definitions
//
#define AM_HAL_SECURE_OTA_OTA_LIST_END_MARKER           0xFFFFFFFF

// OTA Protocol between OTA application and SecureBoot
// OTAPOINTER will be initialized as follows:
// Most significant 30 bits will correspond to most significant 30 bits of OTA Descriptor
// Least Significant bit (bit 0) should be initialized to 1 to indicate a valid OTA Descriptor
// bit 1 should be initialized to 1 to indicate that the list contains an SBL OTA
// OTA Descriptor points to a list of entries, each corresponding to an OTA blob, list terminating in 0xFFFFFFFF
// Each list entry word comprises of following:
// Most significant 30 bits will correspond to most significant 30 bits of OTA blob pointer
// Least Significant 2 bits should be initialized to 1 to indicate a valid OTA Pending
// After Secboot processes an OTA, it clears the least significant bit (bit 0)
// bit 1 indicates the status of the OTA - 0 for Success, 1 for Failure

// Store the b1 to store the return status - pass/fail
// b0 is used to indicate pending/done
#define AM_HAL_OTA_VALID_MASK                0x3
// #### INTERNAL BEGIN ####
/* Apollo5 memory map
ITCM 0x00000000
     0x00003FFF
MRAM 0x00400000
     0x007FFFFF
RAM  0x20000000
     0x2037FFFF
MSPI 0x60000000
     0x6FFFFFFF
     0x80000000
     0x8FFFFFFF
So assuming we never use MSPI for OTA...bits 24, 25, 26, 27, 28, 30, 31 are available (will always be 0 in address)
*/
// #### INTERNAL END ####
// bits 24, 25, 26, 27, 28, 30, 31 will always be 0 in address - Along with bits 0, 1 (as it needs to be word aligned)
#define AM_HAL_OTA_ADDR_MASK           0x20FFFFFC
// Use bit 24, 25, 26, 27 to store the return status, if failure

//
//! This batch is used as bitmask - more than one can be set - Used only for OTAPOINTER register status
//! only bits 24, 25, 26, 27 can be used (expand to 30, 31 if needed)
//
#define AM_HAL_OTADESC_STATUS_MASK      0x0F000001
//
//! OTA Descriptor Status
//
typedef enum
{
    AM_HAL_OTADESC_STATUS_SUCCESS           =          0x00000000,
    AM_HAL_OTADESC_STATUS_PENDING           =          0x00000001,
    AM_HAL_OTADESC_STATUS_INVALID_OTAPTR    = (int32_t)0x08000000,
    AM_HAL_OTADESC_STATUS_INVALID_OTADESC   =          0x04000000,
    AM_HAL_OTADESC_STATUS_MAX_OTA_EXCEED    =          0x02000000,
} am_hal_otadesc_status_e;

// only bits 24, 25, 26, 27 can be used (expanded to 30, 31 if needed) to store the return status, if failure
#define AM_HAL_OTA_STATUS_MASK               0x0F000000
//
//! OTA Status
//
typedef enum
{
    //
    // Note: Enums that use bit31 must be treated as signed decimal values in
    //       order to avoid compiler warnings. We'll cast those as int32_t.
    //
    AM_HAL_OTA_STATUS_SUCCESS               =          0x00000000,
    AM_HAL_OTA_STATUS_IN_PROGRESS           =          0x00000001,
    AM_HAL_OTA_STATUS_FAILURE               =          0x00000002,
    AM_HAL_OTA_STATUS_PENDING               =          0x00000003,
    // Remaining values are used as numbers - only bits 24, 25, 26, 27 can be used (expanded to 30, 31 if needed) - Along with setting the FAILURE indication as value 2 in bits 0,1
    AM_HAL_OTA_STATUS_INVALID_OWNER         = (int32_t)0x01000002,
    AM_HAL_OTA_STATUS_AUTH_POLICY           =          0x02000002,
    AM_HAL_OTA_STATUS_ENC_POLICY            = (int32_t)0x03000002,
    AM_HAL_OTA_STATUS_CRC                   =          0x04000002,
    AM_HAL_OTA_STATUS_AUTH                  = (int32_t)0x05000002,
    AM_HAL_OTA_STATUS_ENC                   = (int32_t)0x06000002,
    AM_HAL_OTA_STATUS_CC                    =          0x07000002,
    AM_HAL_OTA_STATUS_MAGIC                 = (int32_t)0x08000002,
    AM_HAL_OTA_STATUS_RECURSE               =          0x09000002,
    AM_HAL_OTA_STATUS_ERR                   = (int32_t)0x0A000002,
    AM_HAL_OTA_STATUS_INVALID_IMAGE         = (int32_t)0x0B000002,
    AM_HAL_OTA_STATUS_FAIL                  =          0x0C000002,      // Only for Patch
    AM_HAL_OTA_STATUS_VALIDATION            =          0x0D000002,
    AM_HAL_OTA_STATUS_INVALID_OPERATION     =          0x0E000002,      // Only for Wired
    AM_HAL_OTA_STATUS_INTERRUPTED           = (int32_t)0x0F000002,
} am_hal_ota_status_e;

#define AM_HAL_OTA_GET_BLOB_PTR(ptr)         (((uint32_t)(ptr) & ~(AM_HAL_OTA_VALID_MASK | AM_HAL_OTA_STATUS_MASK)))
#define AM_HAL_OTA_IS_VALID(ptr)             (((uint32_t)(ptr) & AM_HAL_OTA_VALID_MASK) == AM_HAL_OTA_VALID_MASK)
#define AM_HAL_OTA_IS_IN_PROGRESS(ptr)       (((uint32_t)(ptr) & AM_HAL_OTA_VALID_MASK) == AM_HAL_OTA_DONE_STATUS_IN_PROGRESS)

#define AM_HAL_OTA_DONE_STATUS_MASK            0x3
#define AM_HAL_OTA_DONE_STATUS_IN_PROGRESS     0x1
#define AM_HAL_OTA_DONE_STATUS_FAILURE         0x2
#define AM_HAL_OTA_DONE_STATUS_SUCCESS         0x0

//
//! Per Image OTA Status information
//
typedef struct
{
    uint32_t            *pImage;
    am_hal_ota_status_e status;
} am_hal_ota_status_t;

//
//! 3 stage Certificate Chain
//
typedef struct
{
    void       *pRootCert;
    void       *pKeyCert;
    void       *pContentCert;
} am_hal_certchain_t;

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! @brief  Initialize OTA state
//!
//! Initializes the OTA state. This should be called before doing any other operation
//!
//! @param  ui32ProgramKey - The Flash programming key
//! @param  pOtaDesc should be start of a flash page designated for OTA Descriptor
//!
//! This call will erase the flash page, which will then be incrementally
//! populated as OTA's are added.  It will also initialize the OTAPOINTER to point
//! to this descriptor, marking it as invalid at the same time
//!
//! @return Returns AM_HAL_STATUS_SUCCESS on success
//
//*****************************************************************************
extern uint32_t am_hal_ota_init(uint32_t ui32ProgramKey, am_hal_otadesc_t *pOtaDesc);

//*****************************************************************************
//
//! @brief  Add a new image for OTA
//!
//! Adds a new image to the OTA Descriptor.
//!
//! @param  ui32ProgamKey - The Flash programming key
//! @param  imageMagic image magic# identifying type of image being added to OTA descr
//! @param  pImage should point to the start of new image to be added to descr
//!
//! This will program the next available entry in OTA descriptor. It will also set
//! appropriate state in the OTA pointer register
//!
//! @return Returns AM_HAL_STATUS_SUCCESS on success
//
//*****************************************************************************
extern uint32_t am_hal_ota_add(uint32_t ui32ProgamKey, uint8_t imageMagic, uint32_t *pImage);

//*****************************************************************************
//
//! @brief  Get Current OTA Descriptor state
//!
//! @param  maxOta Determines the size of the following buffer
//! @param  pStatus - Return Parameter - populated by this function indicating the OTA
//! status of various OTA's
//! @param  pOtaDescStatus - Return Parameter - populated by this function indicating the overall
//! OTA descriptor processin status
//!
//! This will retrieve the current OTA status of various images added to the OTA descr
//!
//! @return Returns AM_HAL_STATUS_SUCCESS on success
//
//*****************************************************************************
extern uint32_t am_hal_get_ota_status(uint32_t maxOta, am_hal_ota_status_t *pStatus, uint32_t *pOtaDescStatus);

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_SECURE_OTA_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

