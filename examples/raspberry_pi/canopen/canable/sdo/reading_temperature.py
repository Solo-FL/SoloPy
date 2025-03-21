# Copyright: (c) 2021-present, SOLO motor controllers project
# MIT License (see LICENSE file for more details)

# Title: SoloPy
# Author: SOLOMotorControllers
# Date: 2025
# Code version: 4.1.0
# Availability: https://github.com/Solo-FL/SoloPy/tree/main/SoloPy
# This Library is made by SOLOMotorControllers.COM
# please visit:  https://www.SOLOMotorControllers.com/

# EXAMPLE of how read the SOLO board temperature,
# every second we print the value of the temperature

import SoloPy as solo

import time

#tested with CANable V2.0 USB-can coverter, after connection the usb of CANable check cmd:" dmesg | tail -n 20 " show usage of /dev/ttyACM0
mySolo = solo.SoloMotorControllersCanopen(0, solo.CanBusBaudRate.RATE_1000, solo.CanCommunicationInterface.CANABLE, 'slcan', "/dev/ttyACM0")

# loop actions
while True:
    # reading
    temperature, error = mySolo.get_board_temperature()

    # print
    print("Read from SOLO: " + str(temperature))
    print("Error: " + str(error))

    time.sleep(1)
