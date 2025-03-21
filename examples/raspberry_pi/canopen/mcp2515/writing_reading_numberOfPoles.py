# Copyright: (c) 2021-present, SOLO motor controllers project
# MIT License (see LICENSE file for more details)

# Title: SoloPy
# Author: SOLOMotorControllers
# Date: 2025
# Code version: 4.0.0
# Availability: https://github.com/Solo-FL/SoloPy/tree/main/SoloPy
# This Library is made by SOLOMotorControllers.COM
# please visit:  https://www.SOLOMotorControllers.com/

# EXAMPLE of how to set the Motor number of poles,
# every second we repit the setting and the reading of it

import SoloPy as solo

import time

# RUN IT BEFORE TEST THE CODE ON RASPBERRY PI:
# sudo ip link set can0 up type can bitrate 1000000
# instanciate a SOLO object:
mySolo = solo.SOLOMotorControllersCanopen(0, solo.CanBusBaudRate.RATE_1000)

# Motor's Number of Poles
NumberOfPoles_write = 4
NumberOfPoles_read = 0

# loop actions
while True:
    # Setting
    mySolo.set_motor_poles_counts(NumberOfPoles_write)

    # Reading
    NumberOfPoles_read, error = mySolo.get_motor_poles_counts()

    # Print
    print("Read from SOLO " + str(NumberOfPoles_read))

    time.sleep(1)
