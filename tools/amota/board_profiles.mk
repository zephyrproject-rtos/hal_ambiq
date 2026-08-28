#******************************************************************************
#
# Board profiles for unified Zephyr AMOTA packaging.
#
# Set BOARD on the command line, e.g.:
#   make BOARD=apollo510b_evb
#
# Override BUILD_DIR when the west output directory does not follow the default
# bld_<board>_peripheral_hr naming convention. Relative paths are resolved from
# the AMOTA tool directory.
#
# NOTE: AMOTA GATT service is not yet supported on apollo3_evb and
# apollo3p_evb in Zephyr. Packaging for those boards is provided for
# tooling parity only; OTA upgrade on-device will not work until support
# is added.
#
#******************************************************************************

SUPPORTED_BOARDS := apollo3_evb apollo3p_evb apollo4p_blue_kxr_evb apollo510b_evb apollo510dL_evb apollo330mP_evb

ifeq ($(filter $(BOARD),$(SUPPORTED_BOARDS)),)
$(error Unsupported BOARD=$(BOARD). Supported: $(SUPPORTED_BOARDS))
endif

OUTPUT_NAME := OTA_update_$(BOARD)_peripheral_hr

# Apollo3 Blue EVB (AMOTA not supported on-device yet)
ifeq ($(BOARD),apollo3_evb)
FAMILY := apollo3
SCRIPT_FAMILY := apollo3
AMOTA_UNSUPPORTED := 1
DEFAULT_BUILD_DIR_NAME := bld_apollo3_evb_peripheral_hr
LOAD_ADDRESS := 0xc000
endif

# Apollo3P Blue EVB (AMOTA not supported on-device yet)
ifeq ($(BOARD),apollo3p_evb)
FAMILY := apollo3
SCRIPT_FAMILY := apollo3
AMOTA_UNSUPPORTED := 1
DEFAULT_BUILD_DIR_NAME := bld_apollo3p_evb_peripheral_hr
LOAD_ADDRESS := 0xc000
endif

# Apollo4P Blue KXR EVB
ifeq ($(BOARD),apollo4p_blue_kxr_evb)
FAMILY := apollo4
SCRIPT_FAMILY := apollo4l
DEFAULT_BUILD_DIR_NAME := bld_ap4p_blue_kxr_evb_peripheral_hr
FIRMWARE_INI := $(AMOTA_DIR)/configs/firmware_apollo4p_blue_kxr_evb.ini
STARTER_BIN := starter_binary_apollo4_blue.bin
TEMP_BIN := temp_binary_apollo4_blue.bin
endif

# Apollo510B EVB
ifeq ($(BOARD),apollo510b_evb)
FAMILY := apollo510
SCRIPT_FAMILY := apollo510
DEFAULT_BUILD_DIR_NAME := bld_ap510b_peripheral_hr_lp
FIRMWARE_INI := $(AMOTA_DIR)/configs/firmware_apollo510b_evb.ini
STARTER_BIN := starter_binary_apollo5_blue.bin
TEMP_BIN := temp_binary_apollo5_blue.bin
endif

# Apollo510DL EVB
ifeq ($(BOARD),apollo510dL_evb)
FAMILY := apollo510
SCRIPT_FAMILY := apollo510L
DEFAULT_BUILD_DIR_NAME := bld_ap510dL_evb_peripheral_hr
FIRMWARE_INI := $(AMOTA_DIR)/configs/firmware_apollo510dL_evb.ini
STARTER_BIN := starter_binary_apollo5_blue.bin
TEMP_BIN := temp_binary_apollo5_blue.bin
endif

# Apollo330MP EVB
ifeq ($(BOARD),apollo330mP_evb)
FAMILY := apollo510
SCRIPT_FAMILY := apollo330P
DEFAULT_BUILD_DIR_NAME := bld_ap330mP_evb_peripheral_hr
FIRMWARE_INI := $(AMOTA_DIR)/configs/firmware_apollo330mP_evb.ini
STARTER_BIN := starter_binary_apollo5_blue.bin
TEMP_BIN := temp_binary_apollo5_blue.bin
endif

SCRIPT_DIR := $(AMOTA_DIR)/scripts/$(SCRIPT_FAMILY)
ifneq ($(wildcard $(ZEPHYR_BASE)/$(DEFAULT_BUILD_DIR_NAME)),)
DEFAULT_BUILD_DIR := $(ZEPHYR_BASE)/$(DEFAULT_BUILD_DIR_NAME)
else
DEFAULT_BUILD_DIR := $(WORKSPACE)/$(DEFAULT_BUILD_DIR_NAME)
endif
BUILD_DIR ?= $(DEFAULT_BUILD_DIR)
APPBIN ?= $(abspath $(BUILD_DIR)/zephyr/zephyr.bin)
