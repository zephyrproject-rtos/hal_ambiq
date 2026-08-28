#!/usr/bin/env python3

# *****************************************************************************
#
#    create_cust_image_blob.py
#
#    Generate customer blobs.
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
from configparser import ConfigParser, ExtendedInterpolation
import binascii
import subprocess
import shlex
import os
import sys
import ctypes
import itertools

from pathlib import Path

from am_defines import *
import apollo4l_keys
import key_table

from arm_utils import global_defines
from arm_utils.cert_basic_utilities import *
from arm_utils.cert_dbg_util_gen import *

from oem_tools_pkg.cert_utils.cert_gen_utils import plugin_manager
from oem_tools_pkg.cert_utils.cert_gen_utils.plugin import signature_interface
from oem_tools_pkg.common_utils import cryptolayer

PRIVATE_KEY_FILE = "private_key.pem"
PUBLIC_KEY_FILE = "public_key.pub"
TEMP_BINARY_FILE = "temp_binary.bin"
TEMP_OUTPUT_FILE = "temp_output.bin"
SIGNATURE_FILE = "image_signature.bin"
PLAINTEXT_OUTPUT = "plaintext.bin"

TEMP_KEY_FILE = "temp_key_file.bin"

AES_TEST_KEY = os.urandom(16)
AES_TEST_IV = os.urandom(16)

AES_TEST_KEK = apollo4l_keys.KEK
AES_TEST_KEK_IV = AES_TEST_IV
signature_plugin_path = "oem_tools_pkg.cert_utils.cert_gen_utils.plugin.signature_interface.LocalInterface"
signature_plugin_cfg = os.path.join(os.path.dirname(os.path.abspath(__file__)), "oem_tools_pkg/cert_utils/cert_gen_utils/plugin/am_local_keys_plugin.cfg")

config_header = '''\
#******************************************************************************
#
# Configuration file for create_cust_image_blob.py
#
# Run "create_cust_image_blob.py --help" for more information about the options
# below.
#
# All numerical values below may be expressed in either decimal or hexadecimal
# "0x" notation.
#
# To re-generate this file using all default values, run
# "create_cust_image_blob.py --create-config"
#
#******************************************************************************
'''

def main():
    options = get_options()

    if options:
        b = BootImage()

        if 'app_file' in options and options['app_file'] is not None:
            b.load_binary(options['app_file'])

        output_bytes = b.create_image(**options)

        output_file = options['output']
        with open(output_file, 'wb') as f:
            f.write(output_bytes)

def get_options():
    args = parse_arguments()

    cmd_defaults = {
        "ambiq_owned": 0,
        "chip": 'apollo4l',
        "app_file": None,
        "load_address": 0x18000,
        "enc_algo": 0,
        "auth_algo": 0,
        "auth_key": 0,
        "kek_index": 0,
        "image_type": 'firmware',
        "ota": 0,
        "sbl_ota": 0,
        "offset": 0,
        "certificate": None,
        "alt_certificate": None,
        "output": 'outimage',
        "key_table": 'keys.ini',
        "wired_chunk_size": 256 * 1024,
        "TEST_len": 0,
        "TEST_errorVal": 0,
        "loglevel": 2,
        "config_file": None,
        "create_config": False,
    }

    config_options = dict()

    args_options = dict()
    for name, value in vars(args).items():
        if value is not None:
            args_options[name] = value

    if args.create_config:
        print("Creating config file.")

        # Grab the settings based on the defaults set by argparse. We need to
        # do a little manipulation here because configparser needs all values
        # in "str" format.
        config_dict = dict()
        for key, value in vars(args).items():
            if key in ["create_config", "config_file"]:
                # Ignore the "create_config" option. We don't need it in the
                # config file.
                pass
            elif isinstance(value, int):
                config_dict[key] = '0x{:X}'.format(value)
            else:
                config_dict[key] = str(value)

        config = configparser.ConfigParser()
        config['Settings'] = config_dict

        with open('example.ini', 'w') as configfile:
            configfile.write(config_header)
            config.write(configfile)

        # Don't actually run the program if we were only asked to create a
        # config file.
        return None

    elif args.config_file:
        print("Using settings from {}.".format(args.config_file))

        # Read in the settings from the supplied config file.
        config = configparser.ConfigParser()
        config.read(args.config_file)
        config_dict = dict(config['Settings'])

        # Convert to a dictionary. This also requires some special handling,
        # because the config file treats everything as a string. This step
        # should convert any numbers or "None" fields to the correct python
        # representation.
        for key, value in config_dict.items():
            if value == 'None':
                config_options[key] = None
            elif isinstance(value, str):
                try:
                    config_options[key] = int(value, 0)
                except ValueError:
                    config_options[key] = value
            else:
                config_options[key] = value

    options = dict()
    options.update(cmd_defaults)
    options.update(config_options)
    options.update(args_options)

    #for n, v in options.items():
        #print(f"{n}: {v}")

    return options

def parse_arguments():
    parser = argparse.ArgumentParser(description = 'Generate Bootloader Image Blob')

#### INTERNAL BEGIN ####
    parser.add_argument('--ambiq-owned', dest='ambiq_owned', type=auto_int,
                        help='Ambiq-owned image?')
#### INTERNAL END ####

#### INTERNAL BEGIN ####
    parser.add_argument('--chip-type', dest='chip', type=str,
                        choices = ['apollo4l'],
                        help='Chip Type: apollo4l (default = apollo4l)')
#### INTERNAL END ####

    parser.add_argument('--bin', dest='app_file', type=str,
                        help='binary file (blah.bin)')

    parser.add_argument('--load-address', dest='load_address', type=auto_int,
                        help='Load address of the binary.')

    parser.add_argument('--enc-algo', dest='enc_algo', type=int, choices=BootImage.enc_algorithms,
                        help='Encryption algorithm to use.')

    parser.add_argument('--auth-algo', dest='auth_algo', type=int, choices=BootImage.auth_algorithms,
                        help='Authentication algorithm to use.')

    parser.add_argument('--auth-key', dest='auth_key', type=auto_int,
                        help='Authentication key.')

    parser.add_argument('--kek-index', dest='kek_index', type=auto_int,
                        help='Key-encryption-key index.')

    parser.add_argument('-t', '--image-type', dest='image_type', type=str, choices=BootImage.image_types,
                        help='Type of OTA file to generate')

    parser.add_argument('--ota', dest='ota', type=int,
                        help='Set the OTA bit. You should use this if you are wrapping an OTA blob instead of a raw binary.')

    parser.add_argument('--sbl-ota', dest='sbl_ota', type=int,
                        help='Sets the SBL OTA bit for wired update images. Set this bit alongside OTA if you are performing an SBL OTA.')

    parser.add_argument('--offset', dest='offset', type=auto_int,
                        help='Offset (for info0 or patch images)')

    parser.add_argument('--certificate', dest='certificate', type=str,
                        help='Certificate file.')

#### INTERNAL BEGIN ####
    parser.add_argument('--alt-certificate', dest='alt_certificate', type=str,
                        help='Alternate certificate file for ICV chain updates. Corresponds to the alternate-location SBL.')
#### INTERNAL END ####

    parser.add_argument('-o', '--output', dest = 'output',
                        help = 'Output filename (without the extension)')

    parser.add_argument('--key-table', dest = 'key_table',
                        help = 'Key table configuration file.')

    parser.add_argument('--wired-chunk-size', dest='wired_chunk_size', type=auto_int,
                        help='Size of the download chunks for wired update (saves SRAM)')

#### INTERNAL BEGIN ####
    parser.add_argument('--len', dest = 'TEST_len', type=auto_int,
                        help = 'Test Only - Inflate the image to specified length')

    parser.add_argument('--error', dest = 'TEST_errorVal', type=auto_int,
                        help = 'Test Only - Introduce intentional errors mask (0x1 = CRC, 0x4 = Magic#, 0x8 = SP, 0x10 = RV, 0x20 = length')
#### INTERNAL END ####
    parser.add_argument('--loglevel', dest='loglevel', type=auto_int,
                        choices = range(AM_PRINT_LEVEL_MIN, AM_PRINT_LEVEL_MAX+1),
                        help=helpPrintLevel)

    parser.add_argument('-c', '--config-file', dest='config_file', type=str,
                        help = 'Use the config file to set our options.')

    parser.add_argument('--create-config', dest='create_config', action="store_true",
                        help = 'Save the listed options to a configuration file.')


    args = parser.parse_args()

    return args

class BitField:
    def __init__(self, word, lsb, size):
        self.word = word
        self.lsb = lsb
        self.size = size
        self.mask = (1 << size) - 1

    def convert(self, value):
        return (value & self.mask) << self.lsb

class BootImage:

    enc_algorithms = [
        0,
        1
    ]

    auth_algorithms = [
        0,
        1
    ]

    magic_numbers = {
#### INTERNAL BEGIN ####
        'patch': AM_IMAGE_MAGIC_PATCH,
#### INTERNAL END ####
        'secure-firmware': AM_IMAGE_MAGIC_SECURE,
        'firmware': AM_IMAGE_MAGIC_NONSECURE,
        'info0': AM_IMAGE_MAGIC_INFO0,
        'container': AM_IMAGE_MAGIC_CONTAINER,
        'wired': AM_IMAGE_MAGIC_DOWNLOAD,
#### INTERNAL BEGIN ####
        'icv_chain': 0xAC,
        'amb_rt_keybank': 0xAE,
#### INTERNAL END ####
        'oem_chain': 0xCC,
        'keyrevoke': 0xCE,
    }

    image_types = magic_numbers.keys()

    # Where to store each piece of information in the image header.
    header_offsets = {
        'ambiq':      BitField(0, 30, 1),
        'ccIncluded': BitField(0, 29, 1),
        'authCheck':  BitField(0, 28, 1),
        'enc':        BitField(0, 27, 1),
        'crcCheck':   BitField(0, 26, 1),
        'blobSize':   BitField(0, 0,  23),

        'crc':        BitField(1, 0,  32),

        'encAlgo':    BitField(2, 20, 4),
        'authAlgo':   BitField(2, 16, 4),
        'encKeyIdx':  BitField(2, 8,  8),
        'authKeyIdx': BitField(2, 0,  8),
    }

    # How to set the OPT fields
    opt_offsets = {
        'firmware': {
            'magicNum':    BitField(0, 0, 8),
            'ccSize':      BitField(0, 8, 12),

            'loadAddrMsb': BitField(1, 0, 32),
        },

        'secure-firmware': {
            'magicNum':    BitField(0, 0, 8),
            'ccSize':      BitField(0, 8, 12),

            'loadAddrMsb': BitField(1, 0, 32),
        },

        'wired': {
            'magicNum':    BitField(0, 0, 8),
            'ota':         BitField(0, 8, 1),
            'sblOta':      BitField(0, 9, 1),

            'loadAddrMsb': BitField(1, 0, 32),

            'ProgramKey':  BitField(2, 0, 32),
        },

        'info0': {
            'magicNum':    BitField(0, 0,  8),

            'offset':      BitField(1, 0,  12),
            'size':        BitField(1, 12, 12),

            'info0Key':    BitField(2, 0,  32),
        },

#### INTERNAL BEGIN ####
        'patch': {
            'magicNum':    BitField(0, 0, 8),

            'offset':      BitField(1, 0, 16),
        },

        'icv_chain': {
            'magicNum':        BitField(0, 0, 8),
            'ccSize':          BitField(0, 8, 12),
        },

        'amb_rt_keybank': {
            'magicNum':        BitField(0, 0, 8),
            'offset':          BitField(0, 8, 8),
            'size':            BitField(0, 16, 8),
        },

#### INTERNAL END ####

        'oem_chain': {
            'magicNum':        BitField(0, 0, 8),
            'ccSize':          BitField(0, 8, 12),
        },

        'keyrevoke': {
            'magicNum':        BitField(0, 0, 8),
        },

        'container': {
            'magicNum':        BitField(0, 0, 8),
        }
    }

    def __init__(self, image=None):
        # Header information.
        self.ambiq = 0
        self.ccIncluded = 0
        self.authCheck = 0
        self.enc = 0
        self.crcCheck = 0
        self.blobSize = 0
        self.crc = 0
        self.authAlgo = 0
        self.encAlgo = 0
        self.encKeyIdx = 0
        self.authKeyIdx = 0

        # Required for authenticated images
        self.signature = None

        # Required for encrypted images.
        self.encryption_info = None

        # The actual image and its metadata
        self.magicNum = 0
        self.image_size = 0
        self.image_type = None
        self.image_blob = bytes()
        self.loadAddrMsb = 0
        self.ccSize = 0

        if image:
            self.set_image(image)

    def set_image(self, binfile):
        self.image_blob = binfile
        self.image_size = len(self.image_blob)

    def load_binary(self, binfile):

        with open(binfile, "rb") as binfile_object:
            self.set_image(bytes(binfile_object.read()))

    def set_default_options(self, **kwargs):
        # Set some default options.
        options = {
#### INTERNAL BEGIN ####
            "ambiq_owned": False,
#### INTERNAL END ####
            "crc_check": True,
            "enc_algo": 0,
            "auth_algo": 0,
            "kek_index": 0,
            "auth_key": 0,
            "image_type": 'firmware',
            "load_address": 0x18000,
            "ota": 0,
            "offset": 0,
            "info_key": 0,
            "certificate": None,
#### INTERNAL BEGIN ####
            "alt_certificate": None,
#### INTERNAL END ####
            "image0": None,
            "image1": None,
            "cert0": None,
            "cert1": None,
            "key_table": None,
            "wired_chunk_size": 0x40000,
        }

        # Read in the options passed by the caller.
        options.update(kwargs)

        return options

    def create_image(self, **kwargs):
        options = self.set_default_options(**kwargs)
        self.set_attributes(**options)

        if self.image_type == "oem_chain":
            return self.create_cert_chain(**options)
        #### INTERNAL BEGIN ####
        elif self.image_type == "icv_chain":
            return self.create_cert_chain(**options)
        elif self.image_type == "amb_rt_keybank":
            return self.create_rt_keybank(**options)
        #### INTERNAL END ####
        elif self.image_type == "keyrevoke" or self.image_type == "container":
            return self.create_magic_only(**options)
        else:
            return self.create_ota_image(**options)

    def create_magic_only(self, **options):
        with open(options["app_file"], "rb") as f:
            binary = f.read()

        # Write the OPT fields, and add them to the image.
        opt_definition = self.opt_offsets[self.image_type]
        opt_fields = bitfields_from_dict(vars(self), opt_definition, length=4)
        binary = opt_fields + binary

        # Encrypt
        binary = self.apply_encryption(binary)

        # Sign
        binary = self.apply_signature(binary)

        # Complete the binary with the image header.
        binary = self.add_header(binary)

        return binary

    def create_rt_keybank(self, **options):
        # Start the binary using the input file.
        with open(options["app_file"], "rb") as f:
            binary = f.read()

        self.size = len(binary) // 4
        self.offset = options["offset"]

        # Write the OPT fields, and add them to the image.
        opt_definition = self.opt_offsets[self.image_type]
        opt_fields = bitfields_from_dict(vars(self), opt_definition, length=4)
        binary = opt_fields + binary

        # Encrypt
        binary = self.apply_encryption(binary)

        # Sign
        binary = self.apply_signature(binary)

        # Complete the binary with the image header.
        binary = self.add_header(binary)

        return binary


    def create_cert_chain(self, **options):
        binary = []

#### INTERNAL BEGIN ####
        # Make sure to set the ambiq bit correctly.
        if self.image_type == "icv_chain":
            self.ambiq = 1
#### INTERNAL END ####

        # Start the image by reading in the three certificates.
        with open(options["certificate"], "rb") as f:
            content_cert = f.read()

        with open(options["root_cert"], "rb") as f:
            root_cert = f.read()

        with open(options["key_cert"], "rb") as f:
            key_cert = f.read()

        self.ccSize = len(content_cert)
        self.ccIncluded = 1

        binary = content_cert + key_cert + root_cert

#### INTERNAL BEGIN ####
        if self.image_type == "icv_chain":
            with open(options["alt_certificate"], "rb") as f:
                alt_cert = f.read()
                binary = binary + alt_cert
#### INTERNAL END ####
        # Write the OPT fields, and add them to the image.
        opt_definition = self.opt_offsets[self.image_type]
        opt_fields = bitfields_from_dict(vars(self), opt_definition, length=4)
        binary = opt_fields + binary

        # Encrypt
        binary = self.apply_encryption(binary)

        # Sign
        binary = self.apply_signature(binary)

        # Complete the binary with the image header.
        binary = self.add_header(binary)

        return binary

    def create_ota_image(self, **options):

        # Make sure the OTA image length is a multiple of 4 bytes.
        self.image_blob = pad_binary(self.image_blob)

        # If we can create a single-chunk OTA image, we should do so.
        # Otherwise, we'll need to split the image into sections so the OTA
        # tool can perform multiple downloads.
        if self.image_type == 'wired' and len(self.image_blob) > options['wired_chunk_size']:
            return self.create_segmented_wired_image(**options)
        else:
            return self.create_simple_ota_image(**options)

    def create_segmented_wired_image(self, **options):

        # Helper function to split the binary into manageable chunks. It
        # yields a tuple containing the binary data, the offset
        # address from the main binary, and a boolean identifying the first
        # segment.
        #
        # These tuples are specifically designed to work with the
        # "generate_ota_chunk" function below.
        def split_binary(B, chunk_size):
            first = True
            for n in range(0, len(B), chunk_size):
                yield (B[n:n + chunk_size], n, first)
                first = False

        # Second helper function to convert each segment into a full OTA blob.
        def generate_ota_chunk(segment, offset, first_segment=False):

            # Each chunk is actually its own wired image.
            chunk_image = BootImage()
            chunk_image.set_image(segment)

            # Take the options for the chunk from the overall options.
            chunk_options = dict()
            chunk_options.update(options)
            chunk_options['load_address'] = options['load_address'] + offset

            # The segments other than the first segement need to be altered
            # slightly so they don't trigger an OTA.
            if first_segment == False:
                chunk_options['ota'] = 0
                chunk_options['sblOta'] = 0

            return chunk_image.create_image(**chunk_options)

        # Split the binary into segments
        segments = split_binary(self.image_blob, options['wired_chunk_size'])

        # Generate "wired update" images for each segment.
        chunks = (generate_ota_chunk(*x) for x in segments)

        # Return the total list of bytes.
        return bytearray(itertools.chain.from_iterable(chunks))

    def create_simple_ota_image(self, **options):
        binary = self.image_blob
        # Add the certificate if necessary.
        if options['certificate']:
            with open(options['certificate'], "rb") as cert_file:
                cert_bytes = bytearray(cert_file.read())
                self.ccSize = len(cert_bytes)
                self.ccIncluded = 1
                binary = cert_bytes + binary

        # Write the OPT fields, and add them to the image.
        opt_definition = self.opt_offsets[self.image_type]
        opt_fields = bitfields_from_dict(vars(self), opt_definition, length=4)
        binary = opt_fields + binary

        # Encrypt
        binary = self.apply_encryption(binary)

        # Sign
        binary = self.apply_signature(binary)

        # Write the Header.
        binary = self.add_header(binary)

        return binary

    def apply_encryption(self, binary):
        if self.enc:
            orig_binary = binary
            kek = self.get_aes_key(self.encKeyIdx)
            kek_iv = AES_TEST_IV
            enc_binary, wrapped_key = self.encrypt_image(binary, AES_TEST_KEY, AES_TEST_IV, kek, kek_iv)

            binary = wrapped_key + AES_TEST_IV + bytearray(0x0 for x in range(16)) + enc_binary

            if testmode:
                print('Encryption Key: {}'.format(' '.join('{:02X}'.format(x) for x in AES_TEST_KEY)))
                print('Encryption IV:  {}'.format(' '.join('{:02X}'.format(x) for x in AES_TEST_IV)))
                unencrypted_binary = wrapped_key + AES_TEST_IV + bytearray(0x0 for x in range(16)) + orig_binary
                with open(PLAINTEXT_OUTPUT, 'wb') as f:
                    f.write(unencrypted_binary)

        return binary

    def apply_signature(self, binary):
        if self.authCheck:
            plugin = plugin_manager.load_plugin(signature_plugin_cfg, signature_plugin_path)
            options = get_options()
            plugin.set_keys_table(options['key_table'])
            signature = plugin.get_signature(self.authKeyIdx, binary)

            if len(signature) != 384:
                print('Error: signature not the correct length')
                sys.exit(1)

            binary = signature + bytes(0 for x in range(len(signature), 384)) + binary

        return binary

    def get_pubkey(self):
        if self.authCheck:

            RSAPubKey = cryptolayer.Common.get_n_and_np_from_keypair(self.keys[self.authKeyIdx].filename, self.keys[self.authKeyIdx].pass_file)

            log_file = 'pk-log.txt'

            with open("pubKey.bin", "wb") as pubkey_file:
                pubkey_file.write(RSAPubKey)

            pubKey_size = os.path.getsize("pubKey.bin")

            file = open("pubKey.bin", 'rb')
            pubKey_bin = file.read()
            file.close()

        return pubKey_bin

    def set_attributes(self, **options):

        # Set the image type and magic number
        self.image_type = options['image_type']
        self.magicNum = self.magic_numbers[self.image_type]

        # Metadata based on image type.
        if self.image_type in ['firmware', 'secure-firmware']:
            self.loadAddrMsb = options['load_address']
        elif self.image_type == 'wired':
            self.loadAddrMsb = options['load_address']
            self.ota = options['ota']
            self.sblOta = options['sbl_ota']
            self.ProgramKey = apollo4l_keys.AM_HAL_FLASH_PROGRAM_KEY
        elif self.image_type == 'info0':
            self.offset = options['offset']
            self.size = len(self.image_blob)
            self.info0Key = apollo4l_keys.INFO_KEY
#### INTERNAL BEGIN ####
        elif self.image_type == 'patch':
            self.offset = options['offset']
            self.size = len(self.image_blob)
#### INTERNAL END ####

        # Set the OTA metadata
#### INTERNAL BEGIN ####
        if options['ambiq_owned']:
            self.ambiq = 1
#### INTERNAL END ####

        if options['crc_check']:
            self.crcCheck = 1

        if options['enc_algo']:
            self.enc = 1
            self.encAlgo = options['enc_algo']
            self.encKeyIdx = options['kek_index']

        if options['auth_algo']:
            self.authCheck = 1
            self.authAlgo = options['auth_algo']
            self.authKeyIdx = options['auth_key']

        self.keys = key_table.import_key_table(options["key_table"])
        self.load_keys(options["key_table"])

    def add_header(self, binary_array, crc_check=True):
        # At this point, we've added everything to the image except the 16-byte
        # header, so we can calculate the blob size.
        self.blobSize = len(binary_array) + 16

        # Create an array to hold the standard header information for the image.
        header_array = [0x0 for n in range(4)]

        # Set all of the fields in the header based on their corresponding
        # member variables.
        for name, _ in self.header_offsets.items():
            self._set_bitfield_by_name(header_array, name, self.header_offsets)

        # Add the second half of the header information to the binary array.
        binary_array = (bytearray(words_to_bytes(header_array[2:])) + binary_array)

        # Calculate a CRC on the image if required. We have to do this here
        # because part of the header block is included in the CRC. Make sure
        # that the CRC field is correctly updated in the first half of the
        # header.
        if crc_check:
            self.crc = binascii.crc32(binary_array) & 0xFFFFFFFF
            self._set_bitfield_by_name(header_array, 'crc', self.header_offsets)

        # Add the first half of the header to the image.
        binary_array = (bytearray(words_to_bytes(header_array[0:2])) + binary_array)

        return binary_array

    def sign_byte_array_openssl(self, B, key, auth_algo=0):
        signature = None
        try:
            with open(TEMP_BINARY_FILE, "wb") as temp_binary:
                temp_binary.write(B)

            subprocess.run(shlex.split("openssl dgst -sha256 -sign {} -passin file:{} -out {} {}"
                                       .format(key.filename, key.pass_file, SIGNATURE_FILE, TEMP_BINARY_FILE)))

            with open(SIGNATURE_FILE, "rb") as signature_file:
                signature =  bytes(signature_file.read())
        finally:
            os.remove(TEMP_BINARY_FILE)
            os.remove(SIGNATURE_FILE)

        return signature

    def encrypt_image(self, binary_array, key, iv, kek=AES_TEST_KEK, kek_iv=AES_TEST_KEK_IV):
        encrypted_bin = self.encrypt_bytes_aes_ctr(binary_array, key, iv)
        wrapped_key = self.encrypt_bytes_aes_ctr(key, kek, kek_iv)

        return (encrypted_bin, wrapped_key)

    def encrypt_bytes_aes_ctr(self, B, key, iv):

        output_bytes = None

        with open(TEMP_BINARY_FILE, "wb") as tempfile:
            tempfile.write(B)

        try:
            aes_args = {
                "in": TEMP_BINARY_FILE,
                "out": TEMP_OUTPUT_FILE,
                "key": ''.join('{:02X}'.format(x) for x in key),
                "iv": ''.join('{:02X}'.format(x) for x in iv),
            }

            openssl_command = "openssl enc -aes-128-ctr -in {in} -out {out} -K {key} -iv {iv} -p -nopad".format(**aes_args)
            #print(openssl_command)
            subprocess.run(shlex.split(openssl_command))

            with open(TEMP_OUTPUT_FILE, "rb") as temp_output_file:
                output_bytes = bytes(temp_output_file.read())

        finally:
            os.remove(TEMP_BINARY_FILE)
            os.remove(TEMP_OUTPUT_FILE)

        return output_bytes

    def load_keys(self, config_file_name):

        config = configparser.ConfigParser()
        config.read(config_file_name)

        kb0 = config['Symmetric Keys']['kb0']
        kb1 = config['Symmetric Keys']['kb1']
        kb2 = config['Symmetric Keys']['kb2']
        kb3 = config['Symmetric Keys']['kb3']

        B = bytearray()
        with open(kb0, 'rb') as kbfile:
            B = B + kbfile.read()
        with open(kb1, 'rb') as kbfile:
            B = B + kbfile.read()
        with open(kb2, 'rb') as kbfile:
            B = B + kbfile.read()
        with open(kb3, 'rb') as kbfile:
            B = B + kbfile.read()

        self.aes_key_bank = B

        with open(config['Symmetric Keys']['kcp'], 'rb') as kcpfile:
            self.kcp = kcpfile.read()
        with open(config['Symmetric Keys']['kce'], 'rb') as kcefile:
            self.kce = kcefile.read()

    def get_aes_key(self, key_index):
        if key_index == 0:
            return self.kcp
        elif key_index == 1:
            return self.kce
        else:
            key_bank_index = key_index & 0x7F
            key_start = key_bank_index * 16
            key_end = key_start + 16
            return self.aes_key_bank[key_start:key_end]

    def _set_bitfield_by_name(self, word_array, bf_name, bf_definitions):
        field = bf_definitions[bf_name]
        word_array[field.word] = (word_array[field.word] |
                                  field.convert(getattr(self, bf_name)))

    def __str__(self):
        S = ''
        for var in sorted(dir(self)):
            if var == 'image_blob':
                S += 'image_blob:\n'
                S += binary_to_string(self.image_blob)
            elif not var.startswith('__') and not callable(getattr(self, var)):
                S += ('{}: {}\n'.format(var, getattr(self, var)))
        return S

# input_dict: Dictionary containing field names and desired values.
# bf_definitions: Dictionary where bitfield parameters are listed by name.
def bitfields_from_dict(input_dict, bf_definitions, length=0, fill=0x0):
    word_array = [fill for x in range(length)]

    for name, field in bf_definitions.items():
        value = input_dict[name]
        word_array[field.word] = (word_array[field.word] | field.convert(value))

    return bytearray(words_to_bytes(word_array))


def pad_binary(b, pad_size=4):
    remainder = len(b) % pad_size
    if remainder:
        return b + bytearray(0 for n in range(pad_size - remainder))
    else:
        return b

def binary_to_string(B):
    binstring = ' '.join('{:02X}'.format(b) for b in B)

    if len(binstring) > 70:
        return '    ' + binstring[0:70] + '...'
    else:
        return '    ' + binstring[0:70]

def words_to_bytes(W):
    for word in W:
        yield (word & 0x000000FF)
        yield (word & 0x0000FF00) >> 8
        yield (word & 0x00FF0000) >> 16
        yield (word & 0xFF000000) >> 24

testmode = False

if __name__ == '__main__':
    main()
