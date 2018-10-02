#!/bin/sh

#reset the STM32 flight controller

# configure the GPIO to control reset
FC_RESET=/sys/devices/teal/teal-gpio/fc_reset

# Put the STM32 into reset
echo "STM32-> reset"
echo '0' > $FC_RESET
sleep 1
# Take the STM32 out of reset
echo '1' > $FC_RESET
