#******************************************************************************
#
# Unified Makefile to prepare AMOTA update binaries for Zephyr applications.
#
# Location: modules/hal/ambiq/tools/amota
#
# Usage:
#   make BOARD=apollo510b_evb
#   make BOARD=apollo4p_blue_kxr_evb
#   make all-built
#
# Optional overrides:
#   BUILD_DIR=<west build directory>
#   APPBIN=<path to zephyr.bin>
#
# Target sample: samples/bluetooth/peripheral_hr
# Output file:   OTA_update_<board>_peripheral_hr.bin
#
#******************************************************************************

BOARD ?= apollo510b_evb

AMOTA_DIR := $(abspath .)
WORKSPACE := $(abspath ../../../../..)
ZEPHYR_BASE := $(abspath $(WORKSPACE)/zephyr)

# Prefer the west workspace virtualenv when present.
VENV_PYTHON_WIN := $(WORKSPACE)/.venv/Scripts/python.exe
VENV_PYTHON_UNIX := $(WORKSPACE)/.venv/bin/python
ifneq ($(wildcard $(VENV_PYTHON_WIN)),)
DEFAULT_PYTHON := $(VENV_PYTHON_WIN)
else ifneq ($(wildcard $(VENV_PYTHON_UNIX)),)
DEFAULT_PYTHON := $(VENV_PYTHON_UNIX)
else
DEFAULT_PYTHON := python
endif
PYTHON ?= $(DEFAULT_PYTHON)

ifeq ($(OS),Windows_NT)
NULL_DEVICE := NUL
else
NULL_DEVICE := /dev/null
endif

include board_profiles.mk

BUILT_BOARDS := apollo4p_blue_kxr_evb apollo510b_evb apollo510dL_evb apollo330mP_evb

SCRIPT_FAMILIES := apollo3 apollo4l apollo510 apollo510L apollo330P

CLEAN_OTA_BINS := \
	$(wildcard OTA_update_*.bin) \
	$(wildcard update_*.bin)

CLEAN_SCRIPT_ARTIFACTS := \
	$(foreach d,$(SCRIPT_FAMILIES),$(wildcard scripts/$(d)/starter_binary_*.bin)) \
	$(foreach d,$(SCRIPT_FAMILIES),$(wildcard scripts/$(d)/temp_*.bin)) \
	$(foreach d,$(SCRIPT_FAMILIES),$(wildcard scripts/$(d)/temp_main_nosecure_ota.bin)) \
	$(foreach d,$(SCRIPT_FAMILIES),$(wildcard scripts/$(d)/keys_info.py))

CLEAN_LOGS := \
	$(foreach d,$(SCRIPT_FAMILIES),$(wildcard scripts/$(d)/oem_tools_pkg/oem_asset_prov_utils/oem_key_request/key_request_cert.log))

.PHONY: all clean realclean list-boards help all-built check-deps

all: $(APPBIN) check-deps
ifeq ($(AMOTA_UNSUPPORTED),1)
	@echo "WARNING: AMOTA is not yet supported on $(BOARD) in Zephyr."
	@echo "         The OTA package is generated for tooling only."
endif
	@echo "Generating AMOTA package for BOARD=$(BOARD)"
ifeq ($(FAMILY),apollo3)
	cp $(SCRIPT_DIR)/keys_info0.py $(SCRIPT_DIR)/keys_info.py
	cd $(SCRIPT_DIR) && $(PYTHON) create_cust_image_blob.py \
		--bin $(APPBIN) --load-address $(LOAD_ADDRESS) --magic-num 0xcb \
		-o temp_main_nosecure_ota --version 0x0
	$(PYTHON) ota_binary_converter.py \
		--appbin $(SCRIPT_DIR)/temp_main_nosecure_ota.bin \
		-o $(OUTPUT_NAME)
	@rm -rf $(SCRIPT_DIR)/temp_main_nosecure_ota.bin
else
	cp $(APPBIN) $(SCRIPT_DIR)/$(STARTER_BIN)
	cd $(SCRIPT_DIR) && $(PYTHON) create_cust_image_blob.py -c $(FIRMWARE_INI)
	$(PYTHON) ota_binary_converter.py \
		--appbin $(SCRIPT_DIR)/$(TEMP_BIN) -o $(OUTPUT_NAME)
	@rm -rf $(SCRIPT_DIR)/$(STARTER_BIN) $(SCRIPT_DIR)/$(TEMP_BIN)
endif
	@echo "Output: $(OUTPUT_NAME).bin"

check-deps:
	@$(PYTHON) -c "from Crypto.Cipher import AES; import yaml; import cryptography" 2>$(NULL_DEVICE) || \
	$(PYTHON) -c "import sys; from Crypto.Cipher import AES; import yaml; import cryptography" || \
	(echo "Missing Python packages. Run: $(PYTHON) -m pip install -r requirements.txt" && exit 1)

$(APPBIN):
	$(error $(APPBIN) not found. Build the application first, or pass BUILD_DIR=<west build directory> / APPBIN=<path to zephyr.bin>)

clean:
ifneq ($(strip $(CLEAN_OTA_BINS)),)
	rm -rf $(CLEAN_OTA_BINS)
endif
ifneq ($(strip $(CLEAN_SCRIPT_ARTIFACTS)),)
	rm -rf $(CLEAN_SCRIPT_ARTIFACTS)
endif
	@echo "Clean complete."

realclean: clean
ifneq ($(strip $(CLEAN_LOGS)),)
	rm -rf $(CLEAN_LOGS)
endif
	-rm -rf __pycache__
	$(foreach d,$(SCRIPT_FAMILIES),rm -rf scripts/$(d)/__pycache__;)
	@echo "Realclean complete."

all-built: $(BUILT_BOARDS)

$(BUILT_BOARDS):
	@echo "=== Building $@ ==="
	$(MAKE) BOARD=$@

.PHONY: $(BUILT_BOARDS)

list-boards:
	@echo "Supported BOARD values:"
	@echo "  apollo3_evb          (AMOTA not supported on-device yet)"
	@echo "  apollo3p_evb         (AMOTA not supported on-device yet)"
	@echo "  apollo4p_blue_kxr_evb"
	@echo "  apollo510b_evb"
	@echo "  apollo510dL_evb"
	@echo "  apollo330mP_evb"

help: list-boards
	@echo ""
	@echo "Examples:"
	@echo "  make BOARD=apollo510b_evb"
	@echo "  make BOARD=apollo510b_evb BUILD_DIR=../../../../../zephyr/bld_ap510b_peripheral_gatt_write"
	@echo "  make BOARD=apollo510b_evb APPBIN=../../../../../zephyr/bld_custom/zephyr/zephyr.bin"
	@echo "  make all-built"
	@echo "  make clean"
	@echo "  make realclean"
