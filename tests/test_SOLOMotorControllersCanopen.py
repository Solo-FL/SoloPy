# Copyright: (c) 2021, 2022, 2023 SOLO motor controllers project
# GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

# Title: SoloPy
# Author: SOLOMotorControllers
# Date: 2023
# Code version: 3.1.0
# Availability: https://github.com/Solo-FL/SoloPy/tree/main/SoloPy
# This Library is made by SOLOMotorControllers.COM
# please visit:  https://www.SOLOMotorControllers.com/

#RUN THE TESTP:
#pytest test_SOLOMotorControllersCanopen.py
  
#RUN THE TEST SYNTETIC LOG:
#pytest -q test_SOLOMotorControllersCanopen.py

import SoloPy as solo
import random

#RUN IT BEFORE TEST THE CODE ON RASPBERRY PI:
#sudo ip link set can0 up type can bitrate 1000000
mySolo = solo.SOLOMotorControllersCanopen(0, solo.CAN_BUS_BAUD_RATE.RATE_1000)

def test_connection():
    result, error = mySolo.connection_is_working()
    assert result == True

def tests():
    assert True == general_setget_test(mySolo.set_command_mode, mySolo.get_command_mode, solo.COMMAND_MODE.DIGITAL)
    assert True == general_setget_test_round(mySolo.set_current_limit, mySolo.get_current_limit, round(random.uniform(1.1, 10.5), 2))
    assert True == general_setget_test_round(mySolo.set_torque_reference_iq, mySolo.get_torque_reference_iq, round(random.uniform(1.1, 10.5), 2))
    assert True == general_setget_test_round(mySolo.set_speed_reference, mySolo.get_speed_reference, round(random.uniform(1, 1000), 0))
    assert True == general_setget_test_round(mySolo.set_power_reference, mySolo.get_power_reference, round(random.uniform(1.1, 10.5), 2))
    assert True == general_setget_test_round(mySolo.set_output_pwm_frequency_khz, mySolo.get_output_pwm_frequency_khz, round(random.uniform(8, 79), 0), 0)
    assert True == general_setget_test_round(mySolo.set_speed_controller_ki, mySolo.get_speed_controller_ki, round(random.uniform(1.1, 10.5), 2))
    assert True == general_setget_test(mySolo.set_motor_direction, mySolo.get_motor_direction, solo.DIRECTION.CLOCKWISE)
    #NEED RESTART assert True == general_setget_test_round(mySolo.set_motor_resistance, mySolo.get_motor_resistance, round(random.uniform(1.1, 10.5), 2))
    #NEED RESTART assert True == general_setget_test_round(mySolo.set_motor_inductance, mySolo.get_motor_inductance, round(random.uniform(1.1, 10.5), 2))
    assert True == general_setget_test_round(mySolo.set_motor_poles_counts, mySolo.get_motor_poles_counts, round(random.uniform(2, 20), 0))
    assert True == general_setget_test_round(mySolo.set_incremental_encoder_lines, mySolo.get_incremental_encoder_lines, round(random.uniform(1, 10), 0))
    assert True == general_setget_test_round(mySolo.set_speed_limit, mySolo.get_speed_limit, round(random.uniform(1, 100), 0))
    assert True == general_setget_test(mySolo.set_feedback_control_mode, mySolo.get_feedback_control_mode, solo.FEEDBACK_CONTROL_MODE.ENCODERS)
    assert True == general_setget_test(mySolo.set_motor_type, mySolo.get_motor_type, solo.MOTOR_TYPE.BLDC_PMSM)
    assert True == general_setget_test(mySolo.set_control_mode, mySolo.get_control_mode, solo.CONTROL_MODE.POSITION_MODE)
    #NEED RESTART assert True == general_setget_test_round(mySolo.set_current_controller_kp, mySolo.get_current_controller_kp, round(random.uniform(1.1, 10.5), 2))
    #NEED RESTART assert True == general_setget_test_round(mySolo.set_current_controller_ki, mySolo.get_current_controller_ki, round(random.uniform(1.1, 10.5), 2))
    assert True == general_setget_test_round(mySolo.set_magnetizing_current_id_reference, mySolo.get_magnetizing_current_id_reference, round(random.uniform(1.1, 10.5), 2))
    assert True == general_setget_test_round(mySolo.set_position_reference, mySolo.get_position_reference, round(random.uniform(1, 100), 0))
    assert True == general_setget_test_round(mySolo.set_position_controller_kp, mySolo.get_position_controller_kp, round(random.uniform(1.1, 10.5), 2))
    assert True == general_setget_test_round(mySolo.set_position_controller_ki, mySolo.get_position_controller_ki, round(random.uniform(1.1, 10.5), 2))
    assert True == general_setget_test_round(mySolo.set_observer_gain_bldc_pmsm, mySolo.get_observer_gain_bldc_pmsm, round(random.uniform(1.1, 10.5), 2))
    assert True == general_setget_test_round(mySolo.set_observer_gain_bldc_pmsm_ultrafast, mySolo.get_observer_gain_bldc_pmsm_ultrafast, round(random.uniform(1.1, 10.5), 2))
    assert True == general_setget_test_round(mySolo.set_observer_gain_dc, mySolo.get_observer_gain_dc, round(random.uniform(1.1, 10.5), 2))
    assert True == general_setget_test_round(mySolo.set_filter_gain_bldc_pmsm, mySolo.get_filter_gain_bldc_pmsm, round(random.uniform(1.1, 10.5), 2))
    assert True == general_setget_test_round(mySolo.set_filter_gain_bldc_pmsm_ultrafast, mySolo.get_filter_gain_bldc_pmsm_ultrafast, round(random.uniform(1.1, 10.5), 2))
    assert True == general_setget_test_round(mySolo.set_encoder_hall_ccw_offset, mySolo.get_encoder_hall_ccw_offset, round(random.uniform(0.1, 1), 2))
    assert True == general_setget_test_round(mySolo.set_encoder_hall_cw_offset, mySolo.get_encoder_hall_cw_offset, round(random.uniform(0.1, 1), 2))
    assert True == general_setget_test_round(mySolo.set_speed_acceleration_value, mySolo.get_speed_acceleration_value, round(random.uniform(1, 10), 0), 0)
    assert True == general_setget_test_round(mySolo.set_speed_deceleration_value, mySolo.get_speed_deceleration_value, round(random.uniform(1, 10), 0), 0)
    assert True == general_setget_test_round(mySolo.set_analogue_speed_resolution_division_coefficient, mySolo.get_analogue_speed_resolution_division_coefficient, round(random.uniform(1.1, 5.5), 2))
    assert True == general_setget_test(mySolo.set_motion_profile_mode, mySolo.get_motion_profile_mode, solo.MOTION_PROFILE_MODE.TIME_BASE_STCURVE)
    assert True == general_setget_test_round(mySolo.set_motion_profile_variable1, mySolo.get_motion_profile_variable1, round(random.uniform(1.1, 5.1), 2))
    assert True == general_setget_test_round(mySolo.set_motion_profile_variable2, mySolo.get_motion_profile_variable2, round(random.uniform(1.1, 5.1), 2))
    assert True == general_setget_test_round(mySolo.set_motion_profile_variable3, mySolo.get_motion_profile_variable3, round(random.uniform(1.1, 5.1), 2))
    assert True == general_setget_test_round(mySolo.set_motion_profile_variable4, mySolo.get_motion_profile_variable4, round(random.uniform(1.1, 5.1), 2))
    assert True == general_setget_test_round(mySolo.set_motion_profile_variable5, mySolo.get_motion_profile_variable5, round(random.uniform(1.1, 5.1), 2))


def general_setget_test_round(functionSet, functionGet, valueToSet, rnd=2):
    isSet, error = functionSet(valueToSet)
    getValue, error = functionGet()
    print(functionSet, valueToSet , round(getValue, rnd))
    return isSet == True and valueToSet == round(getValue, rnd)

def general_setget_test(functionSet, functionGet, valueToSet):
    isSet, error = functionSet(valueToSet)
    getValue, error = functionGet()
    print(valueToSet , getValue)
    return isSet == True and valueToSet == getValue

# def test_current_limit():
#     setValue = 11.5
#     isSet, error = solo.set_current_limit(setValue)
#     assert isSet == True
#     getValue, error = solo.get_current_limit()
#     assert setValue == getValue