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
# Setting the Speed Reference Immediately

import SoloPy as solo
from SoloPy import PdoParameterConfig
from SoloPy import PdoParameterName

import time

# tested with CANable V2.0 USB-can coverter, device manager show CANable at COM69 
mySolo = solo.SoloMotorControllersCanopen(0, solo.CanBusBaudRate.RATE_1000, solo.CanCommunicationInterface.CANABLE, 'slcan', 'COM69')

# 1 time needed CONFIGURATION:
config = PdoParameterConfig()
config.parameterName = PdoParameterName.SPEED_REFERENCE
config.parameterCobId = 0x201
config.isRrtParameterEnable = True
config.isPdoParameterEnable = True
config.syncParameterCount = 0
# send the configuration to SOLO
result, error = mySolo.set_pdo_parameter_config(config)
print("Set pdo parameter config: " + str(result))
print("Error: " + str(error))

time.sleep(0.05)

# set the value
mySolo.set_pdo_speed_reference(200)

# read the value
getValue, error = mySolo.get_speed_reference()
# print
print("Read from SOLO: " + str(getValue))
print("Error: " + str(error))