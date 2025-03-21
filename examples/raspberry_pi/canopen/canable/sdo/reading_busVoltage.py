# Copyright: (c) 2021-present, SOLO motor controllers project
# MIT License (see LICENSE file for more details)

# Title: SoloPy
# Author: SOLOMotorControllers
# Date: 2025
# Code version: 4.1.0
# Availability: https://github.com/Solo-FL/SoloPy/tree/main/SoloPy
# This Library is made by SOLOMotorControllers.COM
# please visit:  https://www.SOLOMotorControllers.com/

# EXAMPLE of how read the SOLO Battery or Supply Input Voltage, 
# every second we print the value of it.

import SoloPy as solo

import time

#tested with CANable V2.0 USB-can coverter, after connection the usb of CANable check cmd:" dmesg | tail -n 20 " show usage of /dev/ttyACM0
mySolo = solo.SoloMotorControllersCanopen(0, solo.CanBusBaudRate.RATE_1000, solo.CanCommunicationInterface.CANABLE, 'slcan', "/dev/ttyACM0")


# loop actions
while True:
    # reading
    busVoltage, error = mySolo.get_bus_voltage()

    # print
    print("Read from SOLO: " + str(busVoltage))
    print("Error: " + str(error))

    time.sleep(1)
