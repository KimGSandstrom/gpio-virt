#!/bin/bash

if [ $(whoami) != "root" ]
  then echo "Please run as root"; exit 1;
fi

if [ "$#" -ne 1 ]; then
    echo -e "Usage: $0 <GPIO_PIN_NUMBER>\nExample: $0 PN.01"
    exit 1
fi

# on gpiochip1
# PBB.01  GPIO08  Pin 16 GPIO/AO DMIC
# PBB.00  GPIO09  Pin 32 GPIO/AO DMIC

# on gpiochip0
# PP.04   GPIO17  Pin 22 GPIO
# PN.01   GPIO27  Pin 15 GPIO/PWM
# PH.00   GPIO35  Pin 18 GPIO/PWM

gpioPinNumber=$1
gpioValuePath="/sys/class/gpio/$gpioPinNumber/value"

# Export the GPIO pin
echo $gpioPinNumber > /sys/class/gpio/export
if [ $? -ne 0 ]; then
    echo "Failed to export GPIO"
    exit 1
fi

# Set the GPIO direction to output
echo "out" > /sys/class/gpio/$gpioPinNumber/direction
if [ $? -ne 0 ]; then
    echo "Failed to set GPIO direction"
    exit 1
fi

# Toggle the GPIO pin
echo 1 > $gpioValuePath
sleep 0.5 # Sleep for half a second
echo 0 > $gpioValuePath

# Unexport the GPIO pin
echo $gpioPinNumber > /sys/class/gpio/unexport
if [ $? -ne 0 ]; then
    echo "Failed to unexport GPIO"
    exit 1
fi

exit 0

