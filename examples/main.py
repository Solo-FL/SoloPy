#Importing SoloPy
import SoloPy as solo

# Initialize the SOLO object 
mySolo = solo.SoloMotorController()

# Setting examples
mySolo.set_pwm_frequency(20)
mySolo.set_number_of_poles(4)
mySolo.set_encoder_lines(2000)

# Reading example
print("Read from SOLO -> BusVoltage: " + str(mySolo.get_bus_voltage()))
print("Read from SOLO -> Temperature: " + str(mySolo.get_board_temperature()))
print("Read from SOLO -> VoltageA: "+  str(mySolo.get_phase_a_voltage()))
print("Read from SOLO -> PWMFrequency: "+  str(mySolo.get_output_pwm_frequency_khz()))
print("Read from SOLO -> NumberOfPoles: "+  str(mySolo.get_motor_poles_counts()))
print("Read from SOLO -> EncoderLines: "+  str(mySolo.get_incremental_encoder_lines()))
