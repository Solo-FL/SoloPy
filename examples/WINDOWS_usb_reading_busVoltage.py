# EXAMPLE of how read the SOLO Battery or Supply Input Voltage, with USB (USB port COM3) in Windows machine
# every second we print the value of the BusVoltage

#Importing SoloPy
import SoloPy as solo
import time

mySolo = solo.SoloMotorController(port="COM3")
while True:
    print("\n Read from SOLO -> BusVoltage: \n" + str(mySolo.get_bus_voltage()))
    time.sleep(1)
