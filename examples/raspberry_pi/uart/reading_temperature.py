# Copyright: (c) 2021, 2022, 2023 SOLO motor controllers project
# GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

# Title: SoloPy
# Author: SOLOMotorControllers
# Date: 2023
# Code version: 3.0.0
# Availability: https://github.com/Solo-FL/SoloPy/tree/main/SoloPy
# This Library is made by SOLOMotorControllers.COM
# please visit:  https://www.SOLOMotorControllers.com/

# EXAMPLE of how read the SOLO board temperature,
# every second we print the value of the temperature

import SoloPy as solo

import time

# instanciate a SOLO object:
mySolo = solo.SoloMotorControllerUart("/dev/ttyS0", 0, solo.UART_BAUD_RATE.RATE_937500)

# loop actions
while True:
    # reading
    temperature, error = mySolo.get_board_temperature()

    # print
    print("Read from SOLO: " + str(temperature))
    print("Error: " + str(error))

    time.sleep(1)

#ensure close the serial
mySolo.disconnect() 