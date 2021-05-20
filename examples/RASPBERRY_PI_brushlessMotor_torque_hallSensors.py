#   Title: Torque Control of a Brushless Motor with RASPBERRY-PI and SOLO using HALL sensors
#   Author: SOLOMOTORCONTROLLERS
#   Date: 2021
#   Code version: 1.0.0
#   Availability: https://github.com/Solo-FL/SOLO-motor-controllers-PYTHON-RASPBERRY-PI-library/
#   Please make sure you are applying the right wiring between SOLO and your RASPBERRY-PI
#   The Code below has been tested on RASPBERRY-PI
#   The Motor used for Testings: DB56C036030-A
#   
#
# _________________________________________________________________________________________________

# Not Completed


# Importing PYTHON RASPBERRY-PI library
import solo_motor_controller as solo
import time
import sys
sys.path.append("../src")


# the device address of SOLO
__solo_address = 0

# For this Test, make sure you have calibrated your HALL sensors before
# to know more please read: https: // www.solomotorcontrollers.com/hall-sensors-to-solo-for-controlling-speed-torque-brushless-motor/

# In this example, make sure you put SOLO into Closed-Loop by
# pressing the Piano Switch NO  # 5 DOWN. in SOLO UNO

# The Piano Switch Setup on SOLO UNO are as below since SOLO will be commandaed in Analogue Mode with PWM:
#  PIN 5 Down: closed-loop
#  PIN 4 UP: Torque Mode(Analogue mode)
#  PIN 1 Down and PIN 2 UP:  BLDC_PMSM motor type(Analogue mode)
#  PIN 3 UP(Not in DFU mode)

# ____________________________________________________________________
# High Speed High Performance Baudrate (Recommended)
# Use this baudrate to have the best and real performance
# of SOLO under all conditions 937500;
#__baudrate = 937500

# Low Speed Low Performance Baudrate
# Use this baudrate only for devices that don't support
# 937500 or 921600 baudrates.
baudrate = 115200
# _____________________________________________________________________

AnalogueCommandMode = 0
PMSM_BLDC_Normal = 1
UsingHallSensors = 2
DigitalCommandMode = 1
ControlType_Torque = 1
ControlType_Speed = 0

# Desired Switching or PWM Frequency at Output
pwm_frequency = 16

# Motor's Number of Poles
numberOfPoles = 8

# Current Limit of the Motor
current_limit = 12.5

# Define Desired Torque referrrence
desired_torque_iq = 1.5

# Converted value to PWM duty cycle for Iq
desired_dutyCycle_iq = 0

# Converted value to PWM duty cycle for currentLimit
desired_dutyCycle_current_limit = 0

# Define the Max Measurable current in SOLO UNO for 100 % duty Cycle
max_measurable_current_solo_uno = 32.0

# Battery or Bus Voltage
bus_voltage = 0

# Desired Speed Limit[RPM]
desired_speed_limit = 3000

# Motor speed feedback
actual_motor_speed = 0

# Motor Iq(torque) feedback
actual_motor_torque_iq = 0


def __loop():

    # set the Direction on C.W.
    __solo_driver.set_direction(0)

    # set a new reference for speed[RPM]
    desired_motor_speed = 10000
    __solo_driver.set_speed_reference(desired_motor_speed)

    # wait till motor reaches to the reference
    time.sleep(5000)

    actual_motor_speed = __solo_driver.get_speed()
    print("\n Motor Speed: \n", actual_motor_speed)

    # set the Direction on C.C.W.
    __solo_driver.set_direction()

    # set a new reference for speed[RPM]
    desired_motor_speed = 30000
    __solo_driver.set_speed_reference(desired_motor_speed)

    # wait till motor reaches to the reference
    time.sleep(5000)

    actual_motor_speed = __solo_driver.get_speed()
    print("\n Motor Speed: \n", actual_motor_speed)

    # stop the motor
    desired_motor_speed = 0
    __solo_driver.set_speed_reference(desired_motor_speed)
    time.sleep(2000)


def __setup():
    # Initialize the SOLO object using the device address of SOLO at 0
    global __solo_driver
    __solo_driver = solo.SoloMotorController(__solo_address, baudrate)

    time.sleep(2000)

    bus_voltage = __solo_driver.get_bus_voltage()
    while bus_voltage <= 0:
        # wait here till communication is established
        bus_voltage = __solo_driver.get_bus_voltage()
        print("\n Trying to Connect To SOLO")
        time.sleep(1000)

    print("\n Communication Established succuessfully!")

    # Initial Configurations
    __solo_driver.set_pwm_frequency(pwm_frequency)
    __solo_driver.set_current_limit(current_limit)

    # select Digital Mode
    __solo_driver.set_command_mode(DigitalCommandMode)

    __solo_driver.set_motor_type(PMSM_BLDC_Normal)

    # Operate while using Hall sensors
    __solo_driver.set_speed_control_mode(UsingHallSensors)

    # run the motor identification
    # run ID. always after selecting the Motor Type!
    __solo_driver.set_identification(1)

    print("\n Identifying the Motor")

    # wait at least for 2sec till ID. is done
    time.sleep(2000)

   # Go back to Analogue Mode to accept PWM as Reference for Torque and Speed
    __solo_driver.set_speed_control_mode(AnalogueCommandMode)

    while True:
        __loop()


def do_work():
    __setup()


if __name__ == "__main__":
    do_work()
