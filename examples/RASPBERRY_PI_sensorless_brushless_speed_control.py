#     Title: Controlling the speed of a Brushless Motor using RASPBERRY-PI in Sensorless Mode
#     Author: SOLOMOTORCONTROLLERS.COM
#     Date: 2021
#     Code version: 1.0.0
#     Availability: https://github.com/Solo-FL/SoloPy/
#     Please make sure you are applying the right wiring between SOLO and your RASPBERRY-PI
#     The Code below has been tested on RASPBERRY-PI 4B
#     The Motor used for Testings: teknic m-2310P-LN-04K
# _________________________________________________________________________________________________

# Importing SOLOPY
from SoloPy import solo_motor_controller as solo
import time
import sys



# the device address of SOLO
__solo_address = 0

# In this example, make sure you put SOLO into Closed-Loop by
# pressing the Piano Switch NO# 5 DOWN. in SOLO UNO

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
pwm_frequency = 75

# Motor's Number of Poles
numberOfPoles = 8

# Select the Normal BLDC_PMSM motor type
motor_type = 1

# Speed controller Kp
speed_controller_kp = 0.04

# Speed controller Ki
speed_controller_ki = 0.008

# Current Limit of the Motor
current_limit = 16.55

# Battery of Bus Voltage
bus_voltage = 0

# Desired Speed Reference
desired_motor_speed = 0

# Motor speed feedback
actual_motor_speed = 0


def __loop():

    # set the Direction on C.W.
    __solo_driver.set_direction(0)

    # set a new reference for speed
    desired_motor_speed = 5000
    __solo_driver.set_speed_reference(desired_motor_speed)

    # wait till motor reaches to the reference
    time.sleep(3)

    actual_motor_speed = __solo_driver.get_speed()
    print("\n Motor Speed: \n", actual_motor_speed)

    # set the Direction on C.C.W.
    __solo_driver.set_direction(1)

    # set a new reference for speed
    desired_motor_speed = 1500
    __solo_driver.set_speed_reference(desired_motor_speed)

    # wait till motor reaches to the reference
    time.sleep(3)

    actual_motor_speed = __solo_driver.get_speed()
    print("\n Motor Speed: \n", actual_motor_speed)

    # stop the motor
    desired_motor_speed = 0
    __solo_driver.set_speed_reference(desired_motor_speed)
    time.sleep(3)


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

    # select Digital Mode
    __solo_driver.set_command_mode(1)

    __solo_driver.set_motor_type(motor_type)

    # run the motor identification
    # run ID. always after selecting the Motor Type!
    __solo_driver.set_identification(1)

    print("\n Identifying the Motor")

    # wait at least for 2sec till ID. is done
    time.sleep(3)

    # Operate in Sensor-less Mode
    __solo_driver.set_speed_control_mode(0)

    # Control The Speed
    __solo_driver.set_control_mode(0)

    # Set the Number of Poles 
    __solo_driver.set_number_of_poles(numberOfPoles)
    
    # Speed Controller Tunings
    __solo_driver.set_speed_controller_Kp(speed_controller_kp)
    __solo_driver.set_speed_controller_Ki(speed_controller_ki)

    while True:
        __loop()


def do_work():
    __setup()


if __name__ == "__main__":
    do_work()
