# EXAMPLE of how read the SOLO Battery or Supply Input Voltage,
# every second we print the value of the BusVoltage

#Importing SoloPy
import SoloPy as solo
import time

mySolo = solo.SoloMotorController()
while True:
    print("Read from SOLO -> BusVoltage: " + str(mySolo.get_bus_voltage()))
    time.sleep(1)
