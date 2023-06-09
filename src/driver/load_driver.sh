#!/bin/bash
#Bash script to load FIBRE_TEST PCIe driver.  Must be run with sudo rights. 

# Copyright 2017
# Original Author: Huide Zhou <prettyage.new@gmail.com>


# Load the FIBRE_TEST PCIe driver
if [ `lsmod | grep -o ft_driver` ]; then
    echo "remove previous FIBRE_TEST driver first."
    rmmod ft_driver.ko
fi
echo "loading FIBRE_TEST driver."
insmod ft_driver.ko

#Find what major device number was assigned from /proc/devices
majorNum=$( awk '{ if ($2 ~ /pcie_ft1/) print $1}' /proc/devices )

if [ -z "$majorNum" ]; then
    echo "Unable to find the FIBRE_TEST device!"
    echo "Did the driver correctly load?"
else
    #Remove any stale device file
    if [ -e "/dev/pcie_ft1" ]; then
        rm -r /dev/pcie_ft1
    fi

    #Create a new one with full read/write permissions for everyone
    mknod -m 666 /dev/pcie_ft1 c $majorNum 0

    echo "Success!"
fi


