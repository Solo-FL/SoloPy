# EXAMPLE of how to set the Motor number of poles,
# every second we repit the setting and the reading of it

import time
import sys
sys.path.append("../src")

#Importing SoloPy
import solo_motor_controller as solo

# the device address of SOLO
__solo_address = 0 

# Motor's Number of Poles
__numberOfPoles_write = 4
numberOfPoles_read = 0


def __loop():
    #Setting
    __solo_driver.set_number_of_poles(__numberOfPoles_write)

    #Reading
    number_of_poles = __solo_driver.get_number_of_poles()

    # Print
    print("\n Read from SOLO -> Number Of Poles: \n")
    print(number_of_poles)

    time.sleep(1)


def __setup():
    #Initialize the SOLO object using the device address of SOLO at 0
    global __solo_driver
    __solo_driver = solo.SoloMotorController(__solo_address)
    while True:
        __loop()

def do_work():
    __setup()


if __name__ == "__main__":
    do_work()




