# Copyright: (c) 2021, 2022, 2023 SOLO motor controllers project
# GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

# Title: SoloPy
# Author: SOLOMotorControllers
# Date: 2023
# Code version: 3.1.0
# Availability: https://github.com/Solo-FL/SoloPy/tree/main/SoloPy
# This Library is made by SOLOMotorControllers.COM
# please visit:  https://www.SOLOMotorControllers.com/

# EXAMPLE of how read the SOLO Battery or Supply Input Voltage, 
# every second we print the value of it.

import SoloPy as solo

import time

# instanciate a SOLO object:
# check with SOLO motion terminal that you are able to connect to your device and make sure the port name in the code is the correct one 
mySolo = solo.SoloMotorControllerUart("/dev/ttyACM0", 0, solo.UART_BAUD_RATE.RATE_937500)

# loop actions
while True:
    # reading
    busVoltage, error = mySolo.get_bus_voltage()

    # print
    print("Read from SOLO: " + str(busVoltage))
    print("Error: " + str(error))

    time.sleep(1)

#ensure close the serial
mySolo.disconnect() 