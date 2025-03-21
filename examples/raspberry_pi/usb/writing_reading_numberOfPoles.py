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

# instanciate a SOLO object:
# check with SOLO motion terminal that you are able to connect to your device and make sure the port name in the code is the correct one 
mySolo = solo.SoloMotorControllerUart("/dev/ttyACM0", 0, solo.UartBaudRate.RATE_937500)

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
