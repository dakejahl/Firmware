#!/bin/sh

echo "This will read the firmware from the STM32 flight controller (Pixracer)"
echo "using the USB DFU mode"

if [ "$(id -u)" != 0 ]; then
  echo "ERROR- This script must be run as root" 1>&2
  exit 1
fi

# configure the GPIO to control reset
if [ ! -d /sys/class/gpio/gpio1023 ]; then
  echo '1023' > /sys/class/gpio/export
  echo 'out' > /sys/class/gpio/gpio1023/direction
fi
FC_RESET=/sys/class/gpio/gpio1023/value

# configure the GPIO to control bootmode
if [ ! -d /sys/class/gpio/gpio1019 ]; then
  echo '1019' > /sys/class/gpio/export
  echo 'out' > /sys/class/gpio/gpio1019/direction
fi
FC_BOOTMODE=/sys/class/gpio/gpio1019/value

# Put the STM32 into bootload mode
echo "Putting STM32 into system bootloader mode"
echo '0' > $FC_RESET
# Set the bootmode pin high
echo '1' > $FC_BOOTMODE
sleep 1
# Take the STM32 out of reset
echo '1' > $FC_RESET
sleep 1

# read the code
sudo dfu-util -d 0483:df11 -c 1 -i 0 -a 0 -s 0x08000000:2097152 -U teal-ef.bin

# Take the STM32 out of bootload mode
echo "Taking STM32 out of system bootloader mode"
echo '0' > $FC_RESET
# Set the bootmode pin low
echo '0' > $FC_BOOTMODE
sleep 1
# Take the STM32 out of reset
echo '1' > $FC_RESET
