# EXAMPLE of how read the SOLO board temperature,
# every second we print the value of the temperature

#Importing SoloPy
import SoloPy as solo
import time

# Initialize the SOLO object 
mySolo = solo.SoloMotorController()

while True:
    print("Read from SOLO -> Board Temperature: " + str(mySolo.get_board_temperature()))
    time.sleep(1)
