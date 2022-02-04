#
#   Title: Position Control of a Brushless Motor with WINDOWS (USB port COM3) and SOLO
#   Author: SOLOMOTORCONTROLLERS
#   Date: 2021
#   Code version: 2.0.0
#   Availability: https://github.com/Solo-FL/SoloPy/
#   The Motor used for Testings: teknic m-2310P-LN-04K
#_________________________________________________________________________________________________

# Importing SoloPy
import SoloPy as solo

import time
import sys

# For this Test, make sure you have calibrated your Encoder before
# to know more please read: https: // www.solomotorcontrollers.com/how-to-connect-calibrate-incremental-encoder-with-solo/

# Initialize the SOLO object 
mySolo = solo.SoloMotorController(port="COM3")

# wait here till communication is established
while mySolo.serial_is_working() == False:
    time.sleep(1)
print("Communication Established succuessfully!")

# Initial Configurations
mySolo.set_command_mode(solo.COMMAND_MODE.DIGITAL)
mySolo.set_motor_type(solo.MOTOR_TYPE.BLDC_PMSM)
mySolo.set_output_pwm_frequency_khz(70)
mySolo.set_current_limit(15.0)

# run the motor identification
# run ID. always after selecting the Motor Type!
print("\n Identifying the Motor")
mySolo.motor_parameters_identification(solo.ACTION.START)
# wait at least for 2sec till ID. is done
time.sleep(3)

mySolo.set_control_mode(solo.CONTROL_MODE.POSITION_MODE)
mySolo.set_feedback_control_mode(solo.FEEDBACK_CONTROL_MODE.ENCODERS)
mySolo.get_incremental_encoder_lines(1000)
mySolo.set_motor_poles_counts(8)

# Speed Controller Tunings
mySolo.set_speed_controller_kp(0.15)
mySolo.set_speed_controller_ki(0.03)

#Position Controller Tunings
mySolo.set_position_controller_kp(1.2)
mySolo.set_position_controller_ki(0.02)
# Initial Configurations end

# loop actions
while True:
  # set a desired Speed Limit for trajectory in RPM
  mySolo.set_speed_limit(5000);
  
  # set a positive desired Position Reference 
  mySolo.set_position_reference(500000);

  # wait till motor reaches to the reference 
  time.sleep(3); 
  print("Number of Pulses passed: " + str(mySolo.get_position_counts_feedback()))

  # set a desired Speed Limit for trajectory in RPM
  mySolo.set_speed_limit(1500)
  
  # set a negative desired Position Reference 
  mySolo.set_position_reference(-32559)

  # wait till motor reaches to the reference 
  time.sleep(6)
  print("Number of Pulses passed: "+ str(mySolo.get_position_counts_feedback()))
