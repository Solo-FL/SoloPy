#
#   Title: Position Control of a Brushless Motor with RASPBERRY-PI and SOLO
#   Author: SOLOMOTORCONTROLLERS
#   Date: 2021
#   Code version: 1.0.0
#   Availability: https://github.com/Solo-FL/SOLO-motor-controllers-PYTHON-RASPBERRY-PI-library/
#   Please make sure you are applying the right wiring between SOLO and your RASPBERRY-PI
#   The Code below has been tested on RASPBERRY-PI 4B
#   The Motor used for Testings: teknic m-2310P-LN-04K
#_________________________________________________________________________________________________

# Importing SOLO PYTHON RASPBERRY-PIlibrary
from SoloPy import solo_motor_controller as Solo
import time
import sys


# the device address of SOLO
__solo_address = 0

# For this Test, make sure you have calibrated your Encoder before
# to know more please read: https: // www.solomotorcontrollers.com/how-to-connect-calibrate-incremental-encoder-with-solo/

# ____________________________________________________________________
# High Speed High Performance Baudrate (Recommended)
# Use this baudrate to have the best and real performance
# of SOLO under all conditions 937500;
baudrate = 937500

# Low Speed Low Performance Baudrate
# Use this baudrate only for devices that don't support
# 937500 or 921600 baudrates.
#baudrate = 115200
# _____________________________________________________________________

# Desired Switching or PWM Frequency at Output in kHz
pwm_frequency = 70

# Motor's Number of Poles
numberOfPoles = 8

# Motor's Number of Encoder Lines(PPR pre-quad)
numberOfEncoder_lines = 1000

# Select the Normal BLDC_PMSM motor type
motor_type = 1

# Speed controller Kp
speed_controller_kp = 0.15

# Speed controller Ki
speed_controller_ki = 0.03

# Position controller Kp
position_controller_kp = 1.2

# Position controller Ki
position_controller_ki = 0.02

# Current Limit of the Motor
current_limit = 15.0

# Battery or Bus Voltage
bus_voltage = 0

# Desired Speed Limit[RPM]
desired_speed_limit = 3000

# Desired Position Reference
desired_position_reference = 0

# Motor speed feedback
actual_motor_speed = 0

# Motor position feedback
actual_motor_position = 0


def __loop():
  # set a desired Speed Limit for trajectory in RPM
  desired_speed_limit = 5000;
  __solo_driver.set_speed_limit(desired_speed_limit);
  
  # set a positive desired Position Reference 
  desired_position_reference =+500000;
  __solo_driver.set_desired_position(desired_position_reference);

  # wait till motor reaches to the reference 
  time.sleep(3); 

  actual_motor_position = __solo_driver.get_encoder_position()
  print("\n Number of Pulses passed: \n",actual_motor_position)


  # set a desired Speed Limit for trajectory in RPM
  desired_speed_limit = 1500
  __solo_driver.set_speed_limit(desired_speed_limit)
  
  # set a negative desired Position Reference 
  desired_position_reference =-32559
  __solo_driver.set_desired_position(desired_position_reference)

  # wait till motor reaches to the reference 
  time.sleep(6)

  actual_motor_position = __solo_driver.get_encoder_position()
  print("\n Number of Pulses passed: \n",actual_motor_position)

def __setup():
    # Initialize the SOLO object using the device address of SOLO at 0
    global __solo_driver
    __solo_driver = solo.SoloMotorController(__solo_address, baudrate)

    time.sleep(2)

    bus_voltage = __solo_driver.get_bus_voltage()
    while bus_voltage <= 0:
        # wait here till communication is established
        bus_voltage = __solo_driver.get_bus_voltage()
        print("\n Trying to Connect To SOLO")
        time.sleep(1)

    print("\n Communication Established succuessfully!")

    # Initial Configurations
    __solo_driver.set_pwm_frequency(pwm_frequency)
    __solo_driver.set_current_limit(current_limit)
    __solo_driver.set_encoder_lines(numberOfEncoder_lines)

    # select Digital Mode
    __solo_driver.set_command_mode(1)

    __solo_driver.set_motor_type(motor_type)

    # run the motor identification
    # run ID. always after selecting the Motor Type!
    __solo_driver.set_identification(1)

    print("\n Identifying the Motor")

    # wait at least for 2sec till ID. is done
    time.sleep(3)

   #Operate while using Quadrature Encoder
    __solo_driver.set_speed_control_mode(1)

   #Control The Position
    __solo_driver.set_control_mode(2)
    
    # Set the Number of Poles 
    __solo_driver.set_number_of_poles(numberOfPoles)

    # Speed Controller Tunings
    __solo_driver.set_speed_controller_Kp(speed_controller_kp)
    __solo_driver.set_speed_controller_Ki(speed_controller_ki)

    #Position Controller Tunings
    __solo_driver.set_position_controller_kp(position_controller_kp)
    __solo_driver.set_position_controller_ki(position_controller_ki)

    while True:
        __loop()


def do_work():
    __setup()


if __name__ == "__main__":
    do_work()
