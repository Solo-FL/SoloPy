# Copyright: (c) 2021, 2022, 2023 SOLO motor controllers project
# GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

# Title: SoloPy
# Author: SOLOMotorControllers
# Date: 2023
# Code version: 3.1.0
# Availability: https://github.com/Solo-FL/SoloPy/tree/main/SoloPy
# This Library is made by SOLOMotorControllers.COM
# please visit:  https://www.SOLOMotorControllers.com/

# The Motor used for Testings: teknic m-2310P-LN-04K
# Read more about this code here: https:#www.solomotorcontrollers.com/sensorless-control-brushless-motor-arduino-solo-digital-mode-uart/

import SoloPy as solo

import time

# instanciate a SOLO object:
# check with SOLO motion terminal that you are able to connect to your device and make sure the port name in the code is the correct one 
mySolo = solo.SoloMotorControllerUart("/dev/ttyACM0", 0, solo.UART_BAUD_RATE.RATE_937500)

# Desired Switching or PWM Frequency at Output
pwmFrequency = 75

# Motor's Number of Poles
numberOfPoles = 8

# Speed controller Kp
speedControllerKp = 0.04

# Speed controller Ki
speedControllerKi = 0.006

# Current Limit of the Motor
currentLimit = 16.55

# Battery of Bus Voltage
busVoltage = 0

# Desired Speed Reference
desiredMotorSpeed = 0

# Motor speed feedback
actualMotorSpeed = 0

# In this example, make sure you put SOLO into Closed-Loop by
# pressing the Piano Switch NO# 5 DOWN. in SOLO UNO

# wait here till communication is established
print("Trying to Connect To SOLO")
communication_is_working = False
while communication_is_working is False:
    time.sleep(1)
    communication_is_working, error = mySolo.communication_is_working()
print("Communication Established succuessfully!")

# Initial Configurations
mySolo.set_output_pwm_frequency_khz(pwmFrequency)
mySolo.set_current_limit(currentLimit)
mySolo.set_command_mode(solo.COMMAND_MODE.DIGITAL)
mySolo.set_motor_type(solo.MOTOR_TYPE.BLDC_PMSM)

# run the motor identification to Auto-tune the current controller gains Kp and Ki needed for Torque Loop
# run ID. always after selecting the Motor Type!
# ID. doesn't need to be called everytime, only one time after wiring up the Motor will be enough
# the ID. values will be remembered by SOLO after power recycling
mySolo.motor_parameters_identification(solo.ACTION.START)
print("Identifying the Motor")
# wait at least for 2sec till ID. is done
time.sleep(2)

# Operate in Sensor-less Mode
mySolo.set_feedback_control_mode(solo.FEEDBACK_CONTROL_MODE.SENSOR_LESS)

# Control The Speed
mySolo.set_control_mode(solo.CONTROL_MODE.SPEED_MODE)

# Controller Tunings
mySolo.set_speed_controller_kp(speedControllerKp)
mySolo.set_speed_controller_ki(speedControllerKi)

# loop actions
while True:
    # set the Direction on C.W.
    mySolo.set_motor_direction(solo.DIRECTION.CLOCKWISE)

    # set a new reference for speed [RPM]
    desiredMotorSpeed = 5000
    mySolo.set_speed_reference(desiredMotorSpeed)

    # wait till motor reaches to the reference
    time.sleep(2)

    actualMotorSpeed, error = mySolo.get_speed_feedback()
    print("Motor Speed: " + str(actualMotorSpeed))

    # set the Direction on C.C.W. 
    mySolo.set_motor_direction(solo.DIRECTION.COUNTERCLOCKWISE)

    # set a new reference for speed [RPM]
    desiredMotorSpeed = 1500
    mySolo.set_speed_reference(desiredMotorSpeed)

    # wait till motor reaches to the reference 
    time.sleep(2)

    actualMotorSpeed, error = mySolo.get_speed_feedback()
    print("Motor Speed: " + str(actualMotorSpeed))

    # stop the motor
    desiredMotorSpeed = 0
    mySolo.set_speed_reference(desiredMotorSpeed)
    time.sleep(2)
