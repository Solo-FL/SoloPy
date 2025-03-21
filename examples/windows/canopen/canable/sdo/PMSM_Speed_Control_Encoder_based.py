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

import SoloPy as solo

import time

#tested with CANable V2.0 USB-can coverter, device manager show CANable at COM69 
mySolo = solo.SOLOMotorControllersCanopen(0, solo.CanBusBaudRate.RATE_1000, solo.CanCommunicationInterface.CANABLE, 'slcan', 'COM69')


# Desired Switching or PWM Frequency at Output
pwmFrequency = 20

# Motor's Number of Poles
numberOfPoles = 8

# Current Limit of the Motor
currentLimit = 7.0

# Motor's Number of Encoder Lines (PPR pre-quad)
numberOfEncoderLines = 1000

# Speed controller Kp
speedControllerKp = 0.15

# Speed controller Ki
speedControllerKi = 0.005

# Battery or Bus Voltage
busVoltage = 0

# Motor Torque feedback
actualMotorTorque = 0

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
mySolo.set_feedback_control_mode(solo.FeedbackControlMode.ENCODERS)
mySolo.set_speed_controller_kp(speedControllerKp)
mySolo.set_speed_controller_ki(speedControllerKi)
mySolo.set_control_mode(solo.ControlMode.SPEED_MODE)

# run the motor identification to Auto-tune the current controller gains Kp and Ki needed for Torque Loop
# run ID. always after selecting the Motor Type!
# ID. doesn't need to be called everytime, only one time after wiring up the Motor will be enough
# the ID. values will be remembered by SOLO after power recycling
mySolo.motor_parameters_identification(solo.Action.START)
print("Identifying the Motor")
# wait at least for 2sec till ID. is done
time.sleep(2)

# loop actions
while True:
    # set the Direction on C.W.
    mySolo.set_motor_direction(solo.Direction.CLOCKWISE)
    # set an arbitrary Positive speed reference[RPM]
    mySolo.set_speed_reference(1500)
    time.sleep(3)
    actualMotorSpeed, error = mySolo.get_speed_feedback()
    print("Motor Speed: " + str(actualMotorSpeed))
    actualMotorTorque, error = mySolo.get_quadrature_current_iq_feedback()
    print("Measured Iq/Torque[A]: " + str(actualMotorTorque))
    time.sleep(0.3)

    # set the Direction on C.C.W.
    mySolo.set_motor_direction(solo.Direction.COUNTERCLOCKWISE)
    # set an arbitrary Positive speed reference[RPM]
    mySolo.set_speed_reference(3000)
    time.sleep(3)
    actualMotorSpeed, error = mySolo.get_speed_feedback()
    print("Motor Speed: " + str(actualMotorSpeed))
    actualMotorTorque, error = mySolo.get_quadrature_current_iq_feedback()
    print("Measured Iq/Torque[A]: " + str(actualMotorTorque))
    time.sleep(0.3)
