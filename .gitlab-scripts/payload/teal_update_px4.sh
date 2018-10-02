#!/bin/sh
#
# This script installs the bootloader and initial PX4 firmware, then proceeds
# to update to the version of the PX4 firmware (i.e. px4fmu-v4_default.px4).
#
# @author Darin Stoker
# @date August 3, 2017
#
#

# Root check
if [ "$(id -u)" != 0 ]; then
  echo "ERROR- This script must be run as root" 2>&1
  exit 1
fi

# Add dir to PATH
export PATH=$PATH:/opt/teal/sbin

# Variables
PX4_BIN=""
FLASH_BOOTLOADER="false"

# Functions
printUsage () {
    echo "Usage: $0 <px4_binary> (--flash-bootloader)" 2>&1
}

# Check parameters
if [ $1 ]; then
    PX4_BIN="$1"
else
    echo "px4_binary not specified." 2>&1
    printUsage
    exit 1
fi

if [ $2 ]; then
    if [ "$2" != "--flash-bootloader" ]; then
        echo "Flag not recognized." 2>&1
        printUsage
        exit 1
    else
        FLASH_BOOTLOADER="true"
    fi
fi

# Flash bootloader (if required)
if [ "${FLASH_BOOTLOADER}" = "true" ]; then
    echo "FLASHING BOOTLOADER ONTO STM32:"
    echo "Resetting the PX4."
    ./reset-fc.sh

    echo "Flashing the STM32."
    cd teal-dfu-stuff
    ./dfu-flash-pixracer.sh
    cd ..
    echo "Finished STM32 flash."

    echo "Sleep 10 to wait for FC to come back up."
    sleep 10
fi

# Update PX4
echo "UPDATING THE PX4 FIRMWARE:"
echo "Resetting the PX4."
echo "~ Task: 1 ~ Progress: 0.1 ~"
./reset-fc.sh
echo "~ Task: 1 ~ Progress: 0.3 ~"
echo "Updating PX4 firmware."

./px_uploader.py --port=/dev/ttyACM0 ${PX4_BIN} || exit $?
echo "~ Task: 1 ~ Progress: 0.8 ~"
sleep 4
echo "~ Task: 1 ~ Progress: 0.9 ~"
./reset-fc.sh
wait
echo "Finished PX4 firmware update."
