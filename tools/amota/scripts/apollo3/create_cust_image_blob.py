#!/usr/bin/env python3

# *****************************************************************************
#
#    create_cust_image_blob.py
#
#    Utility to create image blobs for Apollo3 Secure Boot
#
# *****************************************************************************

# *****************************************************************************
#
#    ${copyright}
#
#  This is part of revision the Ambiq AMOTA tooling release.
#
# *****************************************************************************

import argparse
import sys
from Crypto.Cipher import AES
import array
import hashlib
import hmac
import os
import binascii
import importlib

from am_defines import *
#from keys_info import keyTblAes, keyTblHmac, minAesKeyIdx, maxAesKeyIdx, minHmacKeyIdx, maxHmacKeyIdx, INFO_KEY

#### INTERNAL BEGIN ####
error = 0
forceLen = 0
rsa = 0
#### INTERNAL END ####

#******************************************************************************
#
# Generate the image blob as per command line parameters
#
#******************************************************************************
def process(loadaddress, appFile, magicNum, crcI, crcB, authI, authB, protection, authKeyIdx, output, encKeyIdx, version, erasePrev, child0, child1, authalgo, encalgo, keyFile):

    app_binarray = bytearray()
    # Open the file, and read it into an array of integers.
    with appFile as f_app:
        app_binarray.extend(f_app.read())
        f_app.close()

    filenames = keyFile.split('.')
    keys = importlib.import_module(filenames[0])

    encVal = 0
    if (encalgo != 0):
        encVal = 1
        if ((encKeyIdx < keys.minAesKeyIdx) or (encKeyIdx > keys.maxAesKeyIdx)):
            am_print("Invalid encKey Idx ", encKeyIdx, level=AM_PRINT_LEVEL_ERROR);
            return
        if (encalgo == 2):
            if (encKeyIdx & 0x1):
                am_print("Invalid encKey Idx ", encKeyIdx, level=AM_PRINT_LEVEL_ERROR);
                return
            keySize = 32
        else:
            keySize = 16
    if (authalgo != 0):
        if ((authKeyIdx < keys.minHmacKeyIdx) or (authKeyIdx > keys.maxHmacKeyIdx) or (authKeyIdx & 0x1)):
            am_print("Invalid authKey Idx ", authKeyIdx, level=AM_PRINT_LEVEL_ERROR);
            return

    if (magicNum == AM_IMAGE_MAGIC_MAIN):
        hdr_length  = AM_IMAGEHDR_SIZE_MAIN;   #fixed header length
    elif ((magicNum == AM_IMAGE_MAGIC_CHILD) or (magicNum == AM_IMAGE_MAGIC_CUSTPATCH) or (magicNum == AM_IMAGE_MAGIC_NONSECURE) or (magicNum == AM_IMAGE_MAGIC_INFO0)):
        hdr_length  = AM_IMAGEHDR_SIZE_AUX;   #fixed header length
#### INTERNAL BEGIN ####
    elif ((magicNum == AM_IMAGE_MAGIC_SBL)):
        hdr_length  = AM_IMAGEHDR_SIZE_MAIN;   #fixed header length
    elif ((magicNum == AM_IMAGE_MAGIC_AM3P) or (magicNum == AM_IMAGE_MAGIC_PATCH)):
        hdr_length  = AM_IMAGEHDR_SIZE_AUX;   #fixed header length
    elif (magicNum == AM_IMAGE_MAGIC_SBL + 1): # Just for test
        hdr_length  = AM_IMAGEHDR_SIZE_MAIN;   #fixed header length
#### INTERNAL END ####
    else:
        am_print("magic number", hex(magicNum), " not supported", level=AM_PRINT_LEVEL_ERROR)
        return
    am_print("Header Size = ", hex(hdr_length))

    #generate mutable byte array for the header
    hdr_binarray = bytearray([0x00]*hdr_length);

    orig_app_length  = (len(app_binarray))
    am_print("original app_size ",hex(orig_app_length), "(",orig_app_length,")")

    am_print("load_address ",hex(loadaddress), "(",loadaddress,")")
    if (loadaddress & 0x3):
        am_print("load address needs to be word aligned", level=AM_PRINT_LEVEL_ERROR)
        return

    if (loadaddress & (FLASH_PAGE_SIZE - 1)):
        am_print("WARNING!!! - load address is not page aligned", level=AM_PRINT_LEVEL_ERROR)

    if (magicNum == AM_IMAGE_MAGIC_INFO0):
        if (orig_app_length & 0x3):
            am_print("INFO0 blob length needs to be multiple of 4", level=AM_PRINT_LEVEL_ERROR)
            return
        if ((loadaddress + orig_app_length) > INFO_SIZE_BYTES):
            am_print("INFO0 Offset and length exceed size", level=AM_PRINT_LEVEL_ERROR)
            return

    if (encVal == 1):
        block_size = AM_SECBOOT_AESCBC_BLOCK_SIZE_BYTES
        app_binarray = pad_to_block_size(app_binarray, block_size, 1)
    else:
        # Add Padding
        app_binarray = pad_to_block_size(app_binarray, 4, 0)

    app_length  = (len(app_binarray))
    am_print("app_size ",hex(app_length), "(",app_length,")")

    # Create Image blobs
#### INTERNAL BEGIN ####
    global forceLen
    global error
    am_print("error = ", error)
    if (error & 0x8):
        fill_word(app_binarray, 0, 0xDEADBEEF)
    if (error & 0x10):
        fill_word(app_binarray, 4, 0xDEADBEEF)
    am_print("forceLen = ", forceLen)
    if (forceLen != 0):
        forceLen = forceLen - app_length - hdr_length - 256
        app_binarray.extend(bytearray([0x00]*forceLen))
#### INTERNAL END ####

    # w0
    blobLen = hdr_length + app_length
#### INTERNAL BEGIN ####
    if (error & 0x20):
        blobLen = forceLen
    if (error & 0x4):
        magicNum = magicNum ^ 0xA5A5A5A5;
#### INTERNAL END ####
    w0 = (magicNum << 24) | ((encVal & 0x1) << 23) | blobLen

#### INTERNAL BEGIN ####
    # Temporary - to make sure the BootROM populates correct SP & RV
    if (magicNum == AM_IMAGE_MAGIC_SBL + 1):
        # w0 = SP
        hdr_binarray[0]  = app_binarray[0]
        hdr_binarray[1]  = app_binarray[1]
        hdr_binarray[2]  = app_binarray[2]
        hdr_binarray[3]  = app_binarray[3]

        # w1 = RV
        hdr_binarray[4]  = app_binarray[4]
        hdr_binarray[5]  = app_binarray[5]
        hdr_binarray[6]  = app_binarray[6]
        hdr_binarray[7]  = app_binarray[7]
#### INTERNAL END ####
    am_print("w0 =", hex(w0))
    fill_word(hdr_binarray, 0, w0)

    # w2
    securityVal = ((authI << 1) | crcI) << 4 | (authB << 1) | crcB
    am_print("Security Value ", hex(securityVal))
    w2 = ((securityVal << 24) & 0xff000000) | ((authalgo) & 0xf) | ((authKeyIdx << 4) & 0xf0) | ((encalgo << 8) & 0xf00) | ((encKeyIdx << 12) & 0xf000)
    fill_word(hdr_binarray, 8, w2)
    am_print("w2 = ",hex(w2))


    if (magicNum == AM_IMAGE_MAGIC_INFO0):
        # Insert the INFO0 size and offset
        addrWord = ((orig_app_length>>2) << 16) | ((loadaddress>>2) & 0xFFFF)
        versionKeyWord = keys.INFO_KEY
    else:
        # Insert the application binary load address.
        addrWord = loadaddress | (protection & 0x3)
        # Initialize versionKeyWord
        versionKeyWord = (version & 0x7FFF) | ((erasePrev & 0x1) << 15)

    am_print("addrWord = ",hex(addrWord))
    fill_word(hdr_binarray, AM_IMAGEHDR_OFFSET_ADDR, addrWord)

    am_print("versionKeyWord = ",hex(versionKeyWord))
    fill_word(hdr_binarray, AM_IMAGEHDR_OFFSET_VERKEY, versionKeyWord)

    # Initialize child (Child Ptr/ Feature key)
    am_print("child0/feature = ",hex(child0))
    fill_word(hdr_binarray, AM_IMAGEHDR_OFFSET_CHILDPTR, child0)
    am_print("child1 = ",hex(child1))
    fill_word(hdr_binarray, AM_IMAGEHDR_OFFSET_CHILDPTR + 4, child1)

    authKeyIdx = authKeyIdx - keys.minHmacKeyIdx
    if (authB != 0): # Authentication needed
        am_print("Boot Authentication Enabled")
#        am_print("Key used for HMAC")
#        am_print([hex(keys.keyTblHmac[authKeyIdx*AM_SECBOOT_KEYIDX_BYTES + n]) for n in range (0, AM_HMAC_SIG_SIZE)])
        # Initialize the clear image HMAC
        sigClr = compute_hmac(keys.keyTblHmac[authKeyIdx*AM_SECBOOT_KEYIDX_BYTES:(authKeyIdx*AM_SECBOOT_KEYIDX_BYTES+AM_HMAC_SIG_SIZE)], (hdr_binarray[AM_IMAGEHDR_START_HMAC:hdr_length] + app_binarray))
        am_print("HMAC Clear")
        am_print([hex(n) for n in sigClr])
        # Fill up the HMAC
        for x in range(0, AM_HMAC_SIG_SIZE):
            hdr_binarray[AM_IMAGEHDR_OFFSET_SIGCLR + x]  = sigClr[x]
#### INTERNAL BEGIN ####
        if (error & 0x40):
            hdr_binarray[AM_IMAGEHDR_OFFSET_SIGCLR] = hdr_binarray[AM_IMAGEHDR_OFFSET_SIGCLR] ^ 0xA5;
#### INTERNAL END ####

    # All the header fields part of the encryption are now final
    if (encVal == 1):
        am_print("Encryption Enabled")
        encKeyIdx = encKeyIdx - keys.minAesKeyIdx
        ivValAes = os.urandom(AM_SECBOOT_AESCBC_BLOCK_SIZE_BYTES)
        am_print("Initialization Vector")
        am_print([hex(ivValAes[n]) for n in range (0, AM_SECBOOT_AESCBC_BLOCK_SIZE_BYTES)])
        keyAes = os.urandom(keySize)
        am_print("AES Key used for encryption")
        am_print([hex(keyAes[n]) for n in range (0, keySize)])
        # Encrypted Part
        am_print("Encrypting blob of size " , (hdr_length - AM_IMAGEHDR_START_ENCRYPT + app_length))
        enc_binarray = encrypt_app_aes((hdr_binarray[AM_IMAGEHDR_START_ENCRYPT:hdr_length] + app_binarray), keyAes, ivValAes)
#        am_print("Key used for encrypting AES Key")
#        am_print([hex(keys.keyTblAes[encKeyIdx*keySize + n]) for n in range (0, keySize)])
        # Encrypted Key
        enc_key = encrypt_app_aes(keyAes, keys.keyTblAes[encKeyIdx*keySize:encKeyIdx*keySize + keySize], ivVal0)
        am_print("Encrypted Key")
        am_print([hex(enc_key[n]) for n in range (0, keySize)])
        # Fill up the IV
        for x in range(0, AM_SECBOOT_AESCBC_BLOCK_SIZE_BYTES):
            hdr_binarray[AM_IMAGEHDR_OFFSET_IV + x]  = ivValAes[x]
        # Fill up the Encrypted Key
        for x in range(0, keySize):
            hdr_binarray[AM_IMAGEHDR_OFFSET_KEK + x]  = enc_key[x]
    else:
        enc_binarray = hdr_binarray[AM_IMAGEHDR_START_ENCRYPT:hdr_length] + app_binarray


    if (authI != 0): # Install Authentication needed
        am_print("Install Authentication Enabled")
#        am_print("Key used for HMAC")
#        am_print([hex(keys.keyTblHmac[authKeyIdx*AM_SECBOOT_KEYIDX_BYTES + n]) for n in range (0, AM_HMAC_SIG_SIZE)])
        # Initialize the top level HMAC
        sig = compute_hmac(keys.keyTblHmac[authKeyIdx*AM_SECBOOT_KEYIDX_BYTES:(authKeyIdx*AM_SECBOOT_KEYIDX_BYTES+AM_HMAC_SIG_SIZE)], (hdr_binarray[AM_IMAGEHDR_START_HMAC_INST:AM_IMAGEHDR_START_ENCRYPT] + enc_binarray))
        am_print("Generated Signature")
        am_print([hex(n) for n in sig])
        # Fill up the HMAC
        for x in range(0, AM_HMAC_SIG_SIZE):
            hdr_binarray[AM_IMAGEHDR_OFFSET_SIG + x]  = sig[x]
#### INTERNAL BEGIN ####
        if (error & 0x2):
            hdr_binarray[AM_IMAGEHDR_OFFSET_SIG] = hdr_binarray[AM_IMAGEHDR_OFFSET_SIG] ^ 0xA5;
#### INTERNAL END ####
    # compute the CRC for the blob - this is done on a clear image
    crc = crc32(hdr_binarray[AM_IMAGEHDR_START_CRC:hdr_length] + app_binarray)
    am_print("crc =  ",hex(crc));
#### INTERNAL BEGIN ####
    if (error & 0x1):
        crc = crc ^ 0xA5A5A5A5;
#### INTERNAL END ####
    w1 = crc
    fill_word(hdr_binarray, AM_IMAGEHDR_OFFSET_CRC, w1)

    # now output all three binary arrays in the proper order
#### INTERNAL BEGIN ####
    if (magicNum == AM_IMAGE_MAGIC_SBL):
        output_manuf = output + '_manuf.bin'
        am_print("Writing to file ", output_manuf)
        # For Raw image enc bit should be clear
        w0 = (magicNum << 24) | blobLen
        fill_word(hdr_binarray, 0, w0)
        with open(output_manuf, mode = 'wb') as out_manuf:
            out_manuf.write(hdr_binarray[0:hdr_length])
            out_manuf.write(app_binarray)
        # Restore enc bit
        w0 = (magicNum << 24) | ((encVal & 0x1) << 23) | blobLen
        fill_word(hdr_binarray, 0, w0)
#### INTERNAL END ####
    output = output + '.bin'
    am_print("Writing to file ", output)
    with open(output, mode = 'wb') as out:
        out.write(hdr_binarray[0:AM_IMAGEHDR_START_ENCRYPT])
        out.write(enc_binarray)

#### INTERNAL BEGIN ####
        global rsa
        if (rsa == 1):
            sign = compute_rsa_sign("prv.txt", hdr_binarray[0:AM_IMAGEHDR_START_ENCRYPT]+enc_binarray)
            am_print ("RSA Signature")
            am_print([hex(n) for n in sign])

            verified = verify_rsa_sign("pub.txt", hdr_binarray[0:AM_IMAGEHDR_START_ENCRYPT]+enc_binarray, sign)
            assert verified, 'Signature verification failed'

            if (error & 0x80):
                sign = bytes(0xA5) + sign
            out.write(sign)
#### INTERNAL END ####


def parse_arguments():
    parser = argparse.ArgumentParser(description =
                     'Generate Apollo3 Image Blob')

    parser.add_argument('--bin', dest='appFile', type=argparse.FileType('rb'),
                        help='binary file (blah.bin)')

    parser.add_argument('--load-address', dest='loadaddress', type=auto_int, default=hex(AM_SECBOOT_DEFAULT_NONSECURE_MAIN),
                        help='Load address of the binary.')

    parser.add_argument('--magic-num', dest='magic_num', default=hex(AM_IMAGE_MAGIC_NONSECURE),
                        type=lambda x: x.lower(),
#                        type = str.lower,
                        choices = [
#### INTERNAL BEGIN ####
                                hex(AM_IMAGE_MAGIC_SBL),
                                hex(AM_IMAGE_MAGIC_AM3P),
#### INTERNAL END ####
                                hex(AM_IMAGE_MAGIC_MAIN),
                                hex(AM_IMAGE_MAGIC_CHILD),
#### INTERNAL BEGIN ####
                                hex(AM_IMAGE_MAGIC_PATCH),
#### INTERNAL END ####
                                hex(AM_IMAGE_MAGIC_CUSTPATCH),
                                hex(AM_IMAGE_MAGIC_NONSECURE),
                                hex(AM_IMAGE_MAGIC_INFO0)
                                ],
                        help = 'Magic Num ('
#### INTERNAL BEGIN ####
                                + str(hex(AM_IMAGE_MAGIC_SBL)) + ': SBL, '
                                + str(hex(AM_IMAGE_MAGIC_AM3P)) + ': AM3P, '
#### INTERNAL END ####
                                + str(hex(AM_IMAGE_MAGIC_MAIN)) + ': Main, '
                                + str(hex(AM_IMAGE_MAGIC_CHILD)) + ': Child, '
#### INTERNAL BEGIN ####
                                + str(hex(AM_IMAGE_MAGIC_PATCH)) + ': Patch, '
#### INTERNAL END ####
                                + str(hex(AM_IMAGE_MAGIC_CUSTPATCH)) + ': CustOTA, '
                                + str(hex(AM_IMAGE_MAGIC_NONSECURE)) + ': NonSecure, '
                                + str(hex(AM_IMAGE_MAGIC_INFO0)) + ': Info0) '
                                '- default[Main]'
                                )

    parser.add_argument('-o', dest = 'output', default='outimage',
                        help = 'Output filename (without the extension)')

    parser.add_argument('--authkey', dest = 'authkey', type=auto_int, default=(AM_SECBOOT_MIN_KEYIDX_INFO0), choices = range(AM_SECBOOT_MIN_KEYIDX_INFO0, AM_SECBOOT_MAX_KEYIDX_INFO0 + 1),
                        help = 'Authentication Key Idx? (' + str(AM_SECBOOT_MIN_KEYIDX_INFO0) + ' to ' + str(AM_SECBOOT_MAX_KEYIDX_INFO0) + ')')

    parser.add_argument('--kek', dest = 'kek', type=auto_int, default=(AM_SECBOOT_MIN_KEYIDX_INFO0), choices = range(AM_SECBOOT_MIN_KEYIDX_INFO0, AM_SECBOOT_MAX_KEYIDX_INFO0 + 1),
                        help = 'KEK Index? (' + str(AM_SECBOOT_MIN_KEYIDX_INFO0) + ' to ' + str(AM_SECBOOT_MAX_KEYIDX_INFO0) + ')')

    parser.add_argument('--authalgo', dest = 'authalgo', type=auto_int, default=0, choices=range(0, AM_SECBOOT_AUTH_ALGO_MAX+1),
                        help = helpAuthAlgo)

    parser.add_argument('--encalgo', dest = 'encalgo', type=auto_int, default=0, choices = range(0, AM_SECBOOT_ENC_ALGO_MAX+1),
                        help = helpEncAlgo)

    parser.add_argument('--child0', dest = 'child0', type=auto_int, default=hex(0xFFFFFFFF),
                        help = 'child (blobPtr#0 for Main / feature key for AM3P)')

    parser.add_argument('--child1', dest = 'child1', type=auto_int, default=hex(0xFFFFFFFF),
                        help = 'child (blobPtr#1 for Main)')

    parser.add_argument('--version', dest = 'version', type=auto_int, default=0,
                        help = 'version (15 bit)')

    parser.add_argument('--crcI', dest = 'crcI', type=auto_int, default=1, choices=[0,1],
                        help = 'Install CRC check enabled (Default = Y)?')

    parser.add_argument('--crcB', dest = 'crcB', type=auto_int, default=0, choices=[0,1],
                        help = 'Boot CRC check enabled (Default = N)?')

    parser.add_argument('--authI', dest = 'authI', type=auto_int, default=0, choices=[0,1],
                        help = 'Install Authentication check enabled (Default = N)?')

    parser.add_argument('--authB', dest = 'authB', type=auto_int, default=0, choices=[0,1],
                        help = 'Boot Authentication check enabled (Default = N)?')

    parser.add_argument('--erasePrev', dest = 'erasePrev', type=auto_int, default=0, choices=[0,1],
                        help = 'erasePrev (Valid only for main)')

    parser.add_argument('-p', dest = 'protection', type=auto_int, default=0, choices = [0x0, 0x1, 0x2, 0x3],
                        help = 'protection info 2 bit C W')

    parser.add_argument('-k', type=str, dest='keyFile', nargs='?', default='keys_info.py',
                        help='key file in specified format [default = keys_info.py]')

#### INTERNAL BEGIN ####
    parser.add_argument('--rsa', dest = 'rsa', type=auto_int, default=0, choices = [0, 1],
                        help = 'Test Only - Add RSA Signature at the end (Public key in pub.txt)')

    parser.add_argument('--len', dest = 'len', type=auto_int, default=0,
                        help = 'Test Only - Inflate the image to specified length')

    parser.add_argument('--error', dest = 'errorVal', type=auto_int, default=0,
                        help = 'Test Only - Introduce intentional errors mask (0x1 = CRC, 0x2 = Authentication, 0x4 = Magic#, 0x8 = SP, 0x10 = RV, 0x20 = length, 0x40 = Auth Clear, 0x80 = RSA Sign')
#### INTERNAL END ####
    parser.add_argument('--loglevel', dest='loglevel', type=auto_int, default=AM_PRINT_LEVEL_INFO,
                        choices = range(AM_PRINT_LEVEL_MIN, AM_PRINT_LEVEL_MAX+1),
                        help=helpPrintLevel)


    args = parser.parse_args()
    args.magic_num = int(args.magic_num, 16)

#### INTERNAL BEGIN ####
    global forceLen
    global error
    global rsa
    forceLen = args.len
    error = args.errorVal
    rsa = args.rsa
#### INTERNAL END ####

    return args

#******************************************************************************
#
# Main function.
#
#******************************************************************************
def main():
    # Read the arguments.
    args = parse_arguments()
    am_set_print_level(args.loglevel)


    process(args.loadaddress, args.appFile, args.magic_num, args.crcI, args.crcB, args.authI, args.authB, args.protection, args.authkey, args.output, args.kek, args.version, args.erasePrev, args.child0, args.child1, args.authalgo, args.encalgo, args.keyFile)

if __name__ == '__main__':
    main()
