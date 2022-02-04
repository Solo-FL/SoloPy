#   Title: Speed Control of a Brushless Motor with RASPBERRY-PI and SOLO using HALL sensors
#   Author: SOLOMOTORCONTROLLERS
#   Date: 2021
#   Code version: 2.0.0
#   Availability: https://github.com/Solo-FL/SoloPy/
#   Please make sure you are applying the right wiring between SOLO and your RASPBERRY-PI
#   The Code below has been tested on RASPBERRY-PI 4B
#   The Motor used for Testings: DB56C036030-A
# _________________________________________________________________________________________________

#Importing SoloPy
import SoloPy as solo

import time
import sys

# For this Test, make sure you have calibrated your HALL sensors before
# to know more please read: https: // www.solomotorcontrollers.com/hall-sensors-to-solo-for-controlling-speed-torque-brushless-motor/

# In this example, make sure you put SOLO into Closed-Loop by
# pressing the Piano Switch NO  # 5 DOWN. in SOLO UNO

# The Piano Switch Setup on SOLO UNO are as below
#  PIN 5 Down: closed-loop
#  PIN 3 UP(Not in DFU mode)

# Initialize the SOLO object 
mySolo = solo.SoloMotorController()

# wait here till communication is established
while mySolo.serial_is_working() == False:
    time.sleep(1)
print("Communication Established succuessfully!")

# Initial Configurations
mySolo.set_command_mode(solo.COMMAND_MODE.DIGITAL)
mySolo.set_motor_type(solo.MOTOR_TYPE.BLDC_PMSM)
mySolo.set_output_pwm_frequency_khz(22)
mySolo.set_current_limit(12.5)

# run the motor identification
# run ID. always after selecting the Motor Type!
print("\n Identifying the Motor")
mySolo.motor_parameters_identification(solo.ACTION.START)
# wait at least for 2sec till ID. is done
time.sleep(3)

mySolo.set_control_mode(solo.CONTROL_MODE.SPEED_MODE)
mySolo.set_feedback_control_mode(solo.FEEDBACK_CONTROL_MODE.HALL_SENSORS)
mySolo.set_motor_poles_counts(8)

# Speed Controller Tunings
mySolo.set_speed_controller_kp(0.6)
mySolo.set_speed_controller_ki(0.008)
# Initial Configurations end

# loop actions
while True:
    mySolo.set_motor_direction(solo.DIRECTION.CLOCKWISE)
    mySolo.set_speed_reference(1000)

    # wait till motor reaches to the reference
    time.sleep(1)

    print("Actual Motor Iq: "+ str(mySolo.get_quadrature_current_iq_feedback()))
    print("Motor Speed: " + str(mySolo.get_speed_feedback()))
    time.sleep(3)

    mySolo.set_speed_reference(0)
    time.sleep(1)
    
    print("Actual Motor Iq: "+ str(mySolo.get_quadrature_current_iq_feedback()))
    print("Motor Speed: " + str(mySolo.get_speed_feedback()))
    time.sleep(3)
    
    mySolo.set_motor_direction(solo.DIRECTION.COUNTERCLOCKWISE)
    mySolo.set_speed_reference(30000)

    # wait till motor reaches to the reference
    time.sleep(1)

    print("Actual Motor Iq: "+ str(mySolo.get_quadrature_current_iq_feedback()))
    print("Motor Speed: " + str(mySolo.get_speed_feedback()))
    time.sleep(3)

    mySolo.set_speed_reference(0)
    time.sleep(1)
    
    print("Actual Motor Iq: "+ str(mySolo.get_quadrature_current_iq_feedback()))
    print("Motor Speed: " + str(mySolo.get_speed_feedback()))
    time.sleep(3)
