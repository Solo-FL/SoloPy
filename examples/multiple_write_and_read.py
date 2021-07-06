# Importing SoloPy
import solo_motor_controller as solo
import time
import sys
sys.path.append("../src")

# **********************************************/
# You can instanciate up to 254 SOLOs in a network
# Each SOLO in the network should have a unique address


__solo_address1 = 0  # the device address of SOLO
__solo_address2 = 1  # the device address of SOLO
# *********************************************/

__pWMFrequency_write = 20  # Desired Switching Frequency at Output
__numberOfPoles_write = 4  # Set the Motor's Number of Poles
__encoderLines_write = 2000  # Set PPR for the Encoder

busVoltage_read = 0  # Battery or Supply Input Voltage
temperature_read = 0  # SOLO board Temperature
voltageA_read = 0  # Phase A voltage reading (3phase)
inductance_read = 0  # Motor Phase Inductance
pWMFrequency_read = 0  # Read Switching Frequency of SOLO
numberOfPoles_read = 0  # Read the Motor's Number of Poles
encoderLines_read = 0  # Read the PPR set for the Encoder


def __loop():
    # Setting Some Parameters
    __solo_driver1.set_pwm_frequency(__pWMFrequency_write)
    __solo_driver1.set_number_of_poles(__numberOfPoles_write)
    __solo_driver1.set_encoder_lines(__encoderLines_write)

    # Reading Some Parameters
    busVoltage_read = __solo_driver1.get_bus_voltage()
    temperature_read = __solo_driver1.get_temperature()
    voltageA_read = __solo_driver1.get_voltage_a()
    inductance_read = __solo_driver1.get_inductance()
    pWMFrequency_read = __solo_driver1.get_Pwm_frequency()
    numberOfPoles_read = __solo_driver1.get_number_of_poles()
    encoderLines_read = __solo_driver1.get_encoder_lines()

    print("\n List Of some parameters read from SOLO")

    # Print
    print("\n Read from SOLO -> BusVoltage: \n")
    print(busVoltage_read)

    print("\n Read from SOLO -> Temperature: \n")
    print(temperature_read)

    print("\n Read from SOLO -> VoltageA: \n")
    print(voltageA_read)

    print("\n Read from SOLO -> Inductance: \n")
    print(inductance_read)

    print("\n Read from SOLO -> PWMFrequency: \n")
    print(pWMFrequency_read)

    print("\n Read from SOLO -> NumberOfPoles: \n")
    print(numberOfPoles_read)

    print("\n Read from SOLO -> EncoderLines: \n")
    print(encoderLines_read)

    time.sleep(1)


def __setup():
 # Initialize the SOLO object using the device address of SOLO at 0
    global __solo_driver1
    __solo_driver1 = solo.SoloMotorController(__solo_address1)
    while True:
        __loop()


def do_work():
    __setup()


if __name__ == "__main__":
    do_work()
