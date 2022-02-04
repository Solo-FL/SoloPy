# EXAMPLE of how to set the Motor number of poles,
# every second we repit the setting and the reading of it

#Importing SoloPy
import SoloPy as solo
import time

# Initialize the SOLO object 
mySolo = solo.SoloMotorController(port="COM3")

while True:
    mySolo.set_motor_poles_counts(4)
    print("Read from SOLO -> Number Of Poles: " + str(mySolo.get_motor_poles_counts()))
    time.sleep(1)
