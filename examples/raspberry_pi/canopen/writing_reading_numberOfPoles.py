# Copyright: (c) 2021, 2022, 2023 SOLO motor controllers project
# GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

# Title: SoloPy
# Author: SOLOMotorControllers
# Date: 2023
# Code version: 3.0.0
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
mySolo = solo.SOLOMotorControllersCanopen(0, solo.CAN_BUS_BAUD_RATE.RATE_1000)

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
