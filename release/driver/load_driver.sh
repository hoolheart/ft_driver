#!/bin/bash
#Bash script to load FIBRE_TEST PCIe driver.  Must be run with sudo rights. 

# Copyright 2017

# Load the FIBRE_TEST PCIe driver
if [ `lsmod | grep -o ft_driver` ]; then
    echo "remove previous FIBRE_TEST driver first."
    rmmod ft_driver.ko
fi
echo "loading FIBRE_TEST driver."
insmod ft_driver.ko

#Find what major device number was assigned from /proc/devices
majorNum=$( awk '{ if ($2 ~ /pcie_ft/) print $1}' /proc/devices )

if [ -z "$majorNum" ]; then
    echo "Unable to find the FIBRE_TEST device!"
    echo "Did the driver correctly load?"
else
    #change file mode
    chmod 666 /dev/pcie_ft*
    echo "Success to load driver"
    chmod +x config
    ./config
fi


