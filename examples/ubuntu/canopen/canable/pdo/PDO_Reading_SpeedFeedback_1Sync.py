# Copyright: (c) 2021-present, SOLO motor controllers project
# MIT License (see LICENSE file for more details)

# Title: SoloPy
# Author: SOLOMotorControllers
# Date: 2025
# Code version: 4.1.0
# Availability: https://github.com/Solo-FL/SoloPy/tree/main/SoloPy
# This Library is made by SOLOMotorControllers.COM
# please visit:  https://www.SOLOMotorControllers.com/

# know more reading the CAN manual or Library Documentation 
# Getting the Speed Feedback after 1 SYNC Message

import SoloPy as solo
from SoloPy import PdoParameterConfig
from SoloPy import PdoParameterName

import time

#tested with CANable V2.0 USB-can coverter, after connection the usb of CANable check cmd:" dmesg | tail -n 20 " show usage of /dev/ttyACM0
mySolo = solo.SoloMotorControllersCanopen(0, solo.CanBusBaudRate.RATE_1000, solo.CanCommunicationInterface.CANABLE, 'slcan', "/dev/ttyACM0")

# 1 time needed CONFIGURATION:
config = PdoParameterConfig()
config.parameterName = PdoParameterName.SPEED_FEEDBACK
config.parameterCobId = 0x281
config.isRrtParameterEnable = True
config.isPdoParameterEnable = True
config.syncParameterCount = 1
# send the configuration to SOLO
result, error = mySolo.set_pdo_parameter_config(config)
print("Set pdo parameter config: " + str(result))
print("Error: " + str(error))

time.sleep(0.05)

# send the sync message
mySolo.send_pdo_sync()
time.sleep(0.05)

# read the value
getValue, error = mySolo.get_pdo_speed_feedack()

# print
print("Read from SOLO: " + str(getValue))
print("Error: " + str(error))
