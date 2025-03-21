# Copyright: (c) 2021-present, SOLO motor controllers project
# MIT License (see LICENSE file for more details)

# Title: SoloPy
# Author: SOLOMotorControllers
# Date: 2025
# Code version: 4.1.0
# Availability: https://github.com/Solo-FL/SoloPy/tree/main/SoloPy
# This Library is made by SOLOMotorControllers.COM
# please visit:  https://www.SOLOMotorControllers.com/

# In this example we want:
# STEP 1: to print the command mode of SOLO and the error status of the reading operation
# STEP 2: if we read the command mode without error we want to change the command mode of SOLO.

import SoloPy as solo

import time

# RUN IT BEFORE TEST THE CODE ON RASPBERRY PI:
# sudo ip link set can0 up type can bitrate 1000000
# instanciate a SOLO object:
mySolo = solo.SoloMotorControllersCanopen(0, solo.CanBusBaudRate.RATE_1000)

# loop actions
while True:
    time.sleep(1)

    # STEP 1
    # commandMode : is the Command Mode reading from SOLO device 
    # error : after the execution of the fuction will have the error status of the execution
    commandMode, error = mySolo.get_command_mode()

    # we print the info:
    print("COMMAND MODE READ: " + str(commandMode) + " Error: " + str(error))

    # STEP 2
    # if we have no error we want to change the command mode of SOLO
    # we can compare error with SOLOMotorControllersError enum or int value. Equal code: 
    #    error == SOLOMotorControllers::SOLOMotorControllersError::noErrorDetected
    #    error == 0
    if error == solo.Error.NO_ERROR_DETECTED:
        # we check the commandMode readed value.
        # we can compare commandMode with CommandMode enum or int value. Equal code:
        #    commandMode == solo.CommandMode.ANALOGUE
        #    commandMode == 0

        if commandMode == solo.CommandMode.ANALOGUE:
            # setIsSuccesfull : set return if the set was succesfull
            # SOLOMotorControllers::CommandMode::digital : is the command mode i want to set to SOLO.
            # error : after the execution of the fuction will have the error status of the execution
            setIsSuccesfull, error = mySolo.set_command_mode(solo.CommandMode.DIGITAL)

            # we can call the function without check the return:
            # mySolo.set_command_mode(solo.CommandMode.DIGITAL)

            # we print the info:
            print("COMMAND MODE SET: " + str(commandMode) + " Error: " + str(error))
        else:
            # in this situation we want to set analogue as command mode in SOLO
            # we choose the alternative code with less herror and status controlling:
            setIsSuccesfull, error = mySolo.set_command_mode(solo.CommandMode.ANALOGUE)

            # we can call the function without check the return:
            # mySolo.set_command_mode(solo.CommandMode.ANALOGUE)

            # we print the info:
            print("COMMAND MODE SET: " + str(commandMode) + " Error: " + str(error))
