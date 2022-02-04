#    Title: How to Drive Fast Drone or RC car Brushless Motors using WINDOWS (USB port COM3) and SOLO in Sensoless Mode
#    Author: SOLOMOTORCONTROLLERS
#    Date: 2021
#    Code version: 2.0.0
#    Availability: https://github.com/Solo-FL/SoloPy/
#    The Motor used for Testings: 4150KV, 4x4 SCT 550

#Importing SoloPy
import SoloPy as solo

import time
import sys

# In this example, make sure you put SOLO into Closed-Loop by
# pressing the Piano Switch NO  # 5 DOWN. in SOLO UNO

def __loop():

  mySolo.set_motor_direction(solo.DIRECTION.CLOCKWISE)
  mySolo.set_speed_reference(15000)

  # wait till motor reaches to the reference
  time.sleep(1)

  print("Motor Speed: "+ str(mySolo.get_speed_feedback()))
  time.sleep(5)

  # stop the motor
  mySolo.set_speed_reference(0)
  time.sleep(2)
  
  mySolo.set_motor_direction(solo.DIRECTION.COUNTERCLOCKWISE)
  mySolo.set_speed_reference(30000)

  # wait till motor reaches to the reference
  time.sleep(1)

  print("Motor Speed: "+ str(mySolo.get_speed_feedback()))
  time.sleep(5)

  # stop the motor
  mySolo.set_speed_reference(0)
  time.sleep(2)

def __setup():
    global mySolo

    # Initialize the SOLO object 
    mySolo = solo.SoloMotorController(port="COM3")
    
    # wait here till communication is established
    while mySolo.serial_is_working() == False:
        time.sleep(1)
    print("Communication Established succuessfully!")

    # Initial Configurations
    mySolo.set_command_mode(solo.COMMAND_MODE.DIGITAL)
    mySolo.set_motor_type(solo.MOTOR_TYPE.BLDC_PMSM_ULTRAFAST)
    mySolo.set_output_pwm_frequency_khz(79)
    mySolo.set_current_limit(32.0)

    # run the motor identification
    # run ID. always after selecting the Motor Type!
    print("\n Identifying the Motor")
    mySolo.motor_parameters_identification(solo.ACTION.START)
    # wait at least for 2sec till ID. is done
    time.sleep(3)

    mySolo.set_control_mode(solo.CONTROL_MODE.SPEED_MODE)
    mySolo.set_feedback_control_mode(solo.FEEDBACK_CONTROL_MODE.SENSOR_LESS)
    mySolo.set_motor_poles_counts(2)

    # Speed Controller Tunings
    mySolo.set_speed_controller_kp(0.3)
    mySolo.set_speed_controller_ki(0.005)

    while True:
        __loop()


def do_work():
    __setup()


if __name__ == "__main__":
    do_work()
