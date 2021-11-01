# EXAMPLE of how read the SOLO Battery or Supply Input Voltage, with USB (USB port COM3) in Windows machine
# every second we print the value of the BusVoltage

import time
import sys

#Importing SoloPy
from SoloPy import solo_motor_controller as solo

# the device address of SOLO
__solo_address = 0
 
busVoltage = 0

def __loop():
    #Reading
    busVoltage = __solo_driver.get_bus_voltage()

    # Print
    print("\n Read from SOLO -> BusVoltage: \n")
    print(busVoltage)

    time.sleep(1)



def __setup():
 #Initialize the SOLO object using the device address of SOLO at 0
    global __solo_driver
    __solo_driver = solo.SoloMotorController(__solo_address,937500, "COM3")
    while True:
        __loop()

def do_work():
    __setup()


if __name__ == "__main__":
    do_work()
