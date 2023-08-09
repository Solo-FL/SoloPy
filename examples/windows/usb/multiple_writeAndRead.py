# Copyright: (c) 2021, 2022, 2023 SOLO motor controllers project
# GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

# Title: SoloPy
# Author: SOLOMotorControllers
# Date: 2023
# Code version: 3.0.0
# Availability: https://github.com/Solo-FL/SoloPy/tree/main/SoloPy
# This Library is made by SOLOMotorControllers.COM
# please visit:  https://www.SOLOMotorControllers.com/

import SoloPy as solo

import time

# instanciate a SOLO object:
mySolo = solo.SoloMotorControllerUart("COM3", 0, solo.UART_BAUD_RATE.RATE_937500)

PWMFrequency_write = 20  # Desired Switching Frequency at Output
numberOfPoles_write = 4  # Set the Motor's Number of Poles
encoderLines_write = 2000  # Set PPR for the Encoder

busVoltage = 0  # Battery or Supply Input Voltage
temperature = 0  # SOLO board temperature
voltageA = 0  # Phase A voltage reading (3phase)
inductance = 0  # Motor Phase inductance
PWMFrequency_read = 0  # Read Switching Frequency of SOLO
numberOfPoles_read = 0  # Read the Motor's Number of Poles
encoderLines_read = 0  # Read the PPR set for the Encoder

# Setting Some Parameters
mySolo.set_output_pwm_frequency_khz(PWMFrequency_write)
mySolo.set_motor_poles_counts(numberOfPoles_write)
mySolo.set_incremental_encoder_lines(encoderLines_write)

# loop actions
while True:
    # Reading Some Parameters
    busVoltage, error = mySolo.get_bus_voltage()
    temperature, error = mySolo.get_board_temperature()
    voltageA, error = mySolo.get_phase_a_voltage()
    inductance, error = mySolo.get_motor_inductance()
    PWMFrequency_read, error = mySolo.get_output_pwm_frequency_khz()
    numberOfPoles_read, error = mySolo.get_motor_poles_counts()
    encoderLines_read, error = mySolo.get_incremental_encoder_lines()

    print("List Of some parameters read from SOLO")
    print(busVoltage)
    print(temperature)
    print(voltageA)
    print(inductance)
    print(PWMFrequency_read)
    print(numberOfPoles_read)
    print(encoderLines_read)

    time.sleep(1)

#ensure close the serial
mySolo.disconnect() 