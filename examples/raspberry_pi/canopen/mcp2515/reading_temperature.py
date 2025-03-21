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

# RUN IT BEFORE TEST THE CODE ON RASPBERRY PI:
# sudo ip link set can0 up type can bitrate 1000000
# instanciate a SOLO object:
mySolo = solo.SoloMotorControllersCanopen(0, solo.CanBusBaudRate.RATE_1000)

# loop actions
while True:
    # reading
    temperature, error = mySolo.get_board_temperature()

    # print
    print("Read from SOLO: " + str(temperature))
    print("Error: " + str(error))

    time.sleep(1)
