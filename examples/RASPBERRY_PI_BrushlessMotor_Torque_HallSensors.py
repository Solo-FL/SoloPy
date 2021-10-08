#   Title: Torque Control of a Brushless Motor with RASPBERRY-PI and SOLO using HALL sensors
#   Author: SOLOMOTORCONTROLLERS
#   Date: 2021
#   Code version: 1.0.0
#   Availability: https://github.com/Solo-FL/SoloPy/
#   Please make sure you are applying the right wiring between SOLO and your RASPBERRY-PI
#   The Code below has been tested on RASPBERRY-PI 4B
#   The Motor used for Testings: DB56C036030-A
# _________________________________________________________________________________________________

# Importing PYTHON RASPBERRY-PI library
from SoloPy import solo_motor_controller as solo
import time
import sys


# the device address of SOLO
__solo_address = 0

# For this Test, make sure you have calibrated your HALL sensors before
# to know more please read: https: // www.solomotorcontrollers.com/hall-sensors-to-solo-for-controlling-speed-torque-brushless-motor/

# In this example, make sure you put SOLO into Closed-Loop by
# pressing the Piano Switch NO  # 5 DOWN. in SOLO UNO

# The Piano Switch Setup on SOLO UNO are as below
#  PIN 5 Down: closed-loop
#  PIN 3 UP(Not in DFU mode)

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
pwm_frequency = 22

# Motor's Number of Poles
numberOfPoles = 8

# Select the Normal BLDC_PMSM motor type
motor_type = 1

# Current Limit of the Motor
current_limit = 12.5

# Define Desired Torque referrrence
desired_torque_iq = 0.0

# Define Desired Speed referrrence
desired_motor_speed = 0

# Battery or Bus Voltage
bus_voltage = 0

# Motor speed feedback
actual_motor_speed = 0

# Motor Iq(torque) feedback
actual_motor_torque_iq = 0.0


def __loop():

    # set the Direction on C.W.
    __solo_driver.set_direction(0)
    
    #set desired Iq ( torque ) in Amps
    desired_torque_iq = 2.258
    __solo_driver.set_torque_reference(desired_torque_iq)

    # wait till motor reaches to the reference
    time.sleep(1)

    actual_motor_torque_iq = __solo_driver.get_quadrature_current()
    print("\n Actual Motor Iq: \n", actual_motor_torque_iq)
    
    actual_motor_speed = __solo_driver.get_speed()
    print("\n Motor Speed: \n", actual_motor_speed)
    
    # wait 
    time.sleep(3)
    
    # stop the motor
    desired_torque_iq = 0.0
    __solo_driver.set_torque_reference(desired_torque_iq)
    time.sleep(1)
    
    actual_motor_torque_iq = __solo_driver.get_quadrature_current()
    print("\n Actual Motor Iq: \n", actual_motor_torque_iq)
    
    actual_motor_speed = __solo_driver.get_speed()
    print("\n Motor Speed: \n", actual_motor_speed)
    
    # set the Direction on C.C.W.
    __solo_driver.set_direction(1)
    
    #set desired Iq ( torque ) in Amps
    desired_torque_iq = 3.512
    __solo_driver.set_torque_reference(desired_torque_iq)

    # wait till motor reaches to the reference
    time.sleep(1)

    actual_motor_torque_iq = __solo_driver.get_quadrature_current()
    print("\n Actual Motor Iq: \n", actual_motor_torque_iq)
    
    actual_motor_speed = __solo_driver.get_speed()
    print("\n Motor Speed: \n", actual_motor_speed)
    
    # wait 
    time.sleep(3)

    # stop the motor
    desired_torque_iq = 0.0
    __solo_driver.set_torque_reference(desired_torque_iq)
    time.sleep(1)
    
    actual_motor_torque_iq = __solo_driver.get_quadrature_current()
    print("\n Actual Motor Iq: \n", actual_motor_torque_iq)
    
    actual_motor_speed = __solo_driver.get_speed()
    print("\n Motor Speed: \n", actual_motor_speed)
    
    # wait 
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
    # Torque Controller Gains are tuned after Identification automatically
    __solo_driver.set_identification(1)

    print("\n Identifying the Motor")

    # wait at least for 2sec till ID. is done
    time.sleep(3)

    # Operate in Hall Sensor Mode
    __solo_driver.set_speed_control_mode(2)

    # Control The Torque
    __solo_driver.set_control_mode(1)

    # Set the Number of Poles 
    __solo_driver.set_number_of_poles(numberOfPoles)
    

    while True:
        __loop()


def do_work():
    __setup()


if __name__ == "__main__":
    do_work()
