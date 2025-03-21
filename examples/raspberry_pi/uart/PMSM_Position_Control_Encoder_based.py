# Copyright: (c) 2021-present, SOLO motor controllers project
# MIT License (see LICENSE file for more details)

# Title: SoloPy
# Author: SOLOMotorControllers
# Date: 2025
# Code version: 4.0.0
# Availability: https://github.com/Solo-FL/SoloPy/tree/main/SoloPy
# This Library is made by SOLOMotorControllers.COM
# please visit:  https://www.SOLOMotorControllers.com/

# The Motor used for Testings: teknic m-2310P-LN-04K
# Read more about this code here:https:#www.solomotorcontrollers.com/position-control-brushless-arduino-and-solo/

import SoloPy as solo

import time

# For this Test, make sure you have calibrated your Encoder before
# to know more please read: https:#www.solomotorcontrollers.com/how-to-connect-calibrate-incremental-encoder-with-solo/

# instanciate a SOLO object:
mySolo = solo.SoloMotorControllerUart("/dev/ttyS0", 0, solo.UartBaudRate.RATE_937500)

# Desired Switching or PWM Frequency at Output
pwmFrequency = 20

# Motor's Number of Poles
numberOfPoles = 8

# Motor's Number of Encoder Lines (PPR pre-quad)
numberOfEncoderLines = 1000

# Speed controller Kp
speedControllerKp = 0.1

# Speed controller Ki
speedControllerKi = 0.008

# Position controller Kp
positionControllerKp = 0.12

# Position controller Ki
positionControllerKi = 0.02

# Current Limit of the Motor
currentLimit = 7.5

# Battery or Bus Voltage
busVoltage = 0

# Desired Speed Limit[RPM]
desiredSpeedLimit = 1000

# Desired Position Reference
desiredPositionReference = 0

# Motor speed feedback
actualMotorSpeed = 0

# Motor position feedback
actualMotorPosition = 0

# wait here till communication is established
print("Trying to Connect To SOLO")
communication_is_working = False
while communication_is_working is False:
    time.sleep(1)
    communication_is_working, error = mySolo.communication_is_working()
print("Communication Established succuessfully!")

# Initial Configuration of the device and the Motor
mySolo.set_output_pwm_frequency_khz(pwmFrequency)
mySolo.set_current_limit(currentLimit)
mySolo.set_motor_poles_counts(numberOfPoles)
mySolo.set_incremental_encoder_lines(numberOfEncoderLines)
mySolo.set_command_mode(solo.CommandMode.DIGITAL)
mySolo.set_motor_type(solo.MotorType.BLDC_PMSM)

# run the motor identification to Auto-tune the current controller gains Kp and Ki needed for Torque Loop
# run ID. always after selecting the Motor Type!
# ID. doesn't need to be called everytime, only one time after wiring up the Motor will be enough
# the ID. values will be remembered by SOLO after power recycling
mySolo.motor_parameters_identification(solo.Action.START)
print("Identifying the Motor")
# wait at least for 2sec till ID. is done
time.sleep(2)

# Operate while using Quadrature Incremental Encoders
mySolo.set_feedback_control_mode(solo.FeedbackControlMode.ENCODERS)
mySolo.set_control_mode(solo.ControlMode.POSITION_MODE)
# Speed Controller Tunings
mySolo.set_speed_controller_kp(speedControllerKp)
mySolo.set_speed_controller_ki(speedControllerKi)
# Position Controller Tunings
mySolo.set_position_controller_kp(positionControllerKp)
mySolo.set_position_controller_ki(positionControllerKi)

# loop actions
while True:
    # set a desired Speed Limit for trajectory in RPM
    desiredSpeedLimit = 1200
    mySolo.set_speed_limit(desiredSpeedLimit)
    # set a positive desired Position Reference in terms of pulses
    mySolo.set_position_reference(+10500)
    time.sleep(1)
    actualMotorSpeed, error = mySolo.get_speed_feedback()
    print("Motor Speed [RPM]: " + str(actualMotorSpeed))
    # wait till motor reaches to the reference
    time.sleep(8)
    actualMotorPosition = mySolo.get_position_counts_feedback()
    print("Number of Pulses passed: " + str(actualMotorPosition))

    # set a desired Speed Limit for trajectory in RPM
    desiredSpeedLimit = 2500
    mySolo.set_speed_limit(desiredSpeedLimit)
    # set a positive desired Position Reference in terms of pulses
    mySolo.set_position_reference(-320000)
    time.sleep(1)
    actualMotorSpeed, error = mySolo.get_speed_feedback()
    print("Motor Speed [RPM]: " + str(actualMotorSpeed))
    # wait till motor reaches to the reference
    time.sleep(8)
    actualMotorPosition = mySolo.get_position_counts_feedback()
    print("Number of Pulses passed: " + str(actualMotorPosition))
