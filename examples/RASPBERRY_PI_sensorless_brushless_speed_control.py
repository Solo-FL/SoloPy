#     Title: Controlling the speed of a Brushless Motor using RASPBERRY-PI in Sensorless Mode
#     Author: SOLOMOTORCONTROLLERS.COM
#     Date: 2021
#     Code version: 2.0.0
#     Availability: https://github.com/Solo-FL/SoloPy/
#     Please make sure you are applying the right wiring between SOLO and your RASPBERRY-PI
#     The Code below has been tested on RASPBERRY-PI 4B
#     The Motor used for Testings: teknic m-2310P-LN-04K
# _________________________________________________________________________________________________

#Importing SoloPy
import SoloPy as solo

import time
import sys

# In this example, make sure you put SOLO into Closed-Loop by
# pressing the Piano Switch NO# 5 DOWN. in SOLO UNO

# Initialize the SOLO object 
mySolo = solo.SoloMotorController()

# wait here till communication is established
while mySolo.serial_is_working() == False:
    time.sleep(1)
print("Communication Established succuessfully!")

# Initial Configurations
mySolo.set_command_mode(solo.COMMAND_MODE.DIGITAL)
mySolo.set_motor_type(solo.MOTOR_TYPE.BLDC_PMSM)
mySolo.set_output_pwm_frequency_khz(75)
mySolo.set_current_limit(16.55)

# run the motor identification
# run ID. always after selecting the Motor Type!
print("Identifying the Motor")
mySolo.motor_parameters_identification(solo.ACTION.START)
# wait at least for 2sec till ID. is done
time.sleep(3)

mySolo.set_control_mode(solo.CONTROL_MODE.SPEED_MODE)
mySolo.set_feedback_control_mode(solo.FEEDBACK_CONTROL_MODE.SENSOR_LESS)
mySolo.set_motor_poles_counts(8)

# Speed Controller Tunings
mySolo.set_speed_controller_kp(0.04)
mySolo.set_speed_controller_ki(0.008)
# Initial Configurations end

# loop actions
while True:
    mySolo.set_motor_direction(solo.DIRECTION.CLOCKWISE)
    mySolo.set_speed_reference(5000)

    # wait till motor reaches to the reference
    time.sleep(3)

    print("Motor Speed: "+ str(mySolo.get_speed_feedback()))
    mySolo.set_motor_direction(solo.DIRECTION.COUNTERCLOCKWISE)
    mySolo.set_speed_reference(1500)

    # wait till motor reaches to the reference
    time.sleep(3)

    print("Motor Speed: "+ str(mySolo.get_speed_feedback()))
    mySolo.set_speed_reference(0)
    time.sleep(3)
