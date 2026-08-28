Steps of generating AMOTA update file for Zephyr:

1. Build your target Bluetooth sample for the board:

   west build -b <board> samples/bluetooth/[target_bluetooth_example] \
     -d bld_<board>_[target_bluetooth_example]

   See board_profiles.mk for the expected build directory name per board.

2. Install Python dependencies:

   python -m pip install -r requirements.txt

3. From modules/hal/ambiq/tools/amota, run make with the BOARD option:

   make BOARD=apollo510b_evb
   make BOARD=apollo4p_blue_kxr_evb
   make all-built

   Override BUILD_DIR if your west output directory name differs. Relative
   paths are resolved from modules/hal/ambiq/tools/amota:

   make BOARD=apollo510b_evb BUILD_DIR=../../../../../zephyr/bld_target_example

   Or pass APPBIN directly for a custom build output:

   make BOARD=apollo510b_evb APPBIN=../../../../../zephyr/bld_custom/zephyr/zephyr.bin

   The all-built target packages all supported boards from already-built west
   output directories. It does not run west build.

4. The final AMOTA OTA package is written as:
   OTA_update_<board>_[target_bluetooth_example].bin

All image-blob scripts and keys are bundled under scripts/ in this directory.
No external tooling installation is required.

Notes:
  - apollo510b, apollo510L, and apollo330P boards are fully supported
  - apollo3/apollo4L script folders are included for packaging parity; AMOTA
    on-device is not supported for apollo3/apollo3p in Zephyr yet

5. Load the generated binary into the Ambiq OTA mobile app. The target device
   must be running firmware with CONFIG_BT_AMOTA enabled (for example,
   samples/bluetooth/[target_bluetooth_example] or peripheral_amota).

Supported boards:
  apollo4p_blue_kxr_evb
  apollo510b_evb
  apollo510dL_evb
  apollo330mP_evb

Boards with packaging support only (Zephyr-based AMOTA not supported on-device yet):
  apollo3_evb
  apollo3p_evb

Bundled script directories:
  scripts/apollo3/      - Apollo3 image blob tools
  scripts/apollo4l/     - Apollo4L image blob tools
  scripts/apollo510/    - Apollo510 image blob tools
  scripts/apollo510L/   - Apollo510L image blob tools
  scripts/apollo330P/   - Apollo330P image blob tools

Run "make list-boards" or "make help" for a quick reference.
