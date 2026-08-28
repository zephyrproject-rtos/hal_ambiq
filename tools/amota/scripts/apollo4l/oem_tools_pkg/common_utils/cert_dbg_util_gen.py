#!/usr/local/bin/python3
#
# Copyright (c) 2001-2019, Arm Limited and Contributors. All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause OR Arm’s non-OSI source license
#

# This file contains the general functions that are used in both certificates

import configparser
import sys
from global_defines import *
from cert_basic_utilities import *
from cert_dbg_util_data import *


# The function GetRSASignature calculates the RSA signature
def GetRSASignature(logFile, dataIn, authKeyIdx, plugin):

    signature = plugin.get_signature(authKeyIdx, dataIn)

    return CertRSASignature(ReverseBytesinBinString(signature))
# End of GetRSASignature

# This class holds the secondary N HASH value
class CertPubKeyHASHData:

    # Constructor
    def __init__(self, HASHData):
        self.PubHashData = HASHData

    # The method returns the signature size
    def __len__(self):
        return len(self.PubHashData)

    # This method returns the binary signature

    def VarsToBinString(self):
        DataBinStr = str()

        for i in range(SHA_256_HASH_SIZE_IN_BYTES):
            byte = self.PubHashData[i]
            DataBinStr = DataBinStr + byte2string(byte)

        return DataBinStr

# End of CertPubKeyHASHData

# The function GetPubKeyHash returns the hash of the (N|Np).
def GetPubKeyHash(logFile, RSAPubKeyFileName, CryptoDLL_handle):
    hashData = create_string_buffer(SHA_256_HASH_SIZE_IN_BYTES)

    result = CryptoDLL_handle.SBU_GetHashOfNAndNpFromPubKey(str.encode(RSAPubKeyFileName), hashData, SHA_256_HASH_SIZE_IN_BYTES)
    if result != 0:
        print_and_log (logFile, "Error in hash(public key | Np)")
        sys.exit(1)

    # Create the public key object and return it in binary format
    return CertPubKeyHASHData(hashData)
# End of GetPubKeyHash

