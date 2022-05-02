# EXAMPLE of how read the SOLO Battery or Supply Input Voltage, with USB (USB port COM3) in Windows machine
# every second we print the value of the BusVoltage

#Importing SoloPy
import SoloPy as solo
import time

mySolo = solo.SoloMotorController(port="COM3")
while True:
    print("Read from SOLO -> Board Temperature: " + str(mySolo.get_board_temperature()))
    time.sleep(1)