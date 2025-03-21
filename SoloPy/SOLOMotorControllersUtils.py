## @package SOLOMotorControllersUtils.py
#  @author  SOLOMotorControllers
#  @brief   This file contains all the utility common functions
#           Availability: https://github.com/Solo-FL/SoloPy/tree/main/SoloPy
#  @date    Date: 2025
#  @version 4.0.0

## @attention
# Copyright: (c) 2021-present, SOLO motor controllers project
# MIT License (see LICENSE file for more details)

import math
from typing import List, Tuple, Union
from SoloPy.SOLOMotorControllers import *
from SoloPy import ConstantCommon


def convert_to_data(number: Union[int, float], data_type: DataType) -> Tuple[int]:
    """
    Converts a number to a list of data bytes based on the specified data type.

    Args:
        number (int or float): The number to convert.
        data_type (DataType): The type of data conversion.

    Returns:
        Tuple[int]: The converted data as a list of bytes.
    """
    data = []
    if data_type == DataType.SFXT:
        dec = math.ceil(number * 131072)
        if dec < 0:
            dec = 0xFFFFFFFF - abs(dec)
        data = [(dec >> i & 0xff) for i in (24, 16, 8, 0)]
    elif data_type in (DataType.UINT32, DataType.INT32):
        dec = number
        data = [(dec >> i & 0xff) for i in (24, 16, 8, 0)]
    return data


def convert_from_data(data: Tuple[int], data_type: DataType) -> Union[int, float]:
    """
    Converts data bytes to a number based on the specified data type.

    Args:
        data (Tuple[int]): The data to convert.
        data_type (DataType): The type of data conversion.

    Returns:
        int or float: The converted number.
    """
    if data_type == DataType.SFXT:
        return convert_to_float(data)
    if data_type == DataType.INT32:
        return convert_to_int(data)
    if data_type == DataType.UINT32:
        return convert_to_long(data)
    return 0

def convert_to_float(data: Tuple[int]) -> float:
    """
    Converts data bytes to a float number.

    Args:
        data (Tuple[int]): The data to convert.

    Returns:
        float: The converted float number.
    """
    dec = int.from_bytes(data, byteorder='big', signed=False)
    if dec <= 0x7FFE0000:
        value = dec / 131072.0
        return float(f"{value:f}")
    dec = 0xFFFFFFFF - dec + 1
    value = (dec / 131072.0) * -1
    return float(f"{value:f}")


def convert_to_long(data: Tuple[int]) -> int:
    """
    Converts data bytes to a long integer.

    Args:
        data (Tuple[int]): The data to convert.

    Returns:
        int: The converted long integer.
    """
    return int.from_bytes(data, byteorder='big', signed=False)


def convert_to_int(data: Tuple[int]) -> int:
    """
    Converts data bytes to an integer.

    Args:
        data (Tuple[int]): The data to convert.

    Returns:
        int: The converted integer.
    """
    return int.from_bytes(data, byteorder='big', signed=True)


def extract_data(data: Tuple[int]) -> Tuple[int]:
    """
    Extracts specific bytes from the provided data list.

    Args:
        data (Tuple[int]): The data list to extract from.

    Returns:
        Tuple[int]: The extracted bytes.
    """
    return [data[7], data[6], data[5], data[4]]


def get_data(cmd: Tuple[int]) -> Tuple[int]:
    """
    Extracts specific command data.

    Args:
        cmd (Tuple[int]): The command list.

    Returns:
        Tuple[int]: The extracted command data.
    """
    return [cmd[2], cmd[3], cmd[4], cmd[5]]

def validate_input_range(value: Union[int, float], min_val: Union[int, float], max_val: Union[int, float]) -> Tuple[bool, Error, str]:
    """
    Validates if the given value is within a specified range.

    Args:
        value (int or float): The value to validate.
        min_val (int or float): Minimum acceptable value.
        max_val (int or float): Maximum acceptable value.

    Returns:
        Tuple[bool, Error, str]: Validation status, error code, and error message if any.
    """
    if not min_val <= value <= max_val:
        return False, Error.OUT_OF_RANGE_SETTING, ConstantCommon.INPUT_OUT_OF_RANGE
    return True, Error.NO_ERROR_DETECTED, ""


def set_guard_time_input_validation(guardtime: int) -> Tuple[bool, Error, str]:
    """Validates guard time input."""
    return validate_input_range(guardtime, 0, 65535)


def set_life_time_factor_input_validation(lifetime_factor: int) -> Tuple[bool, Error, str]:
    """Validates lifetime factor input."""
    return validate_input_range(lifetime_factor, 0, 255)


def set_producer_heartbeat_time_input_validation(producer_heartbeat_time: int) -> Tuple[bool, Error, str]:
    """Validates producer heartbeat time input."""
    return validate_input_range(producer_heartbeat_time, 0, 65535)


def set_device_address_input_validation(device_address: int) -> Tuple[bool, Error, str]:
    """Validates device address input."""
    return validate_input_range(device_address, 0, 254)

def set_current_limit_input_validation(current_limit: float) -> Tuple[bool, Error, str]:
    """Validates current limit input."""
    return validate_input_range(current_limit, 0.0, 200.0)


def set_torque_reference_iq_input_validation(torque_reference_iq: float) -> Tuple[bool, Error, str]:
    """Validates torque reference Iq input."""
    return validate_input_range(torque_reference_iq, 0.0, 200.0)


def set_speed_reference_input_validation(speed_reference: int) -> Tuple[bool, Error, str]:
    """Validates speed reference input."""
    return validate_input_range(speed_reference, 0, 200000)


def set_power_reference_input_validation(power_reference: float) -> Tuple[bool, Error, str]:
    """Validates power reference input."""
    return validate_input_range(power_reference, 0.0, 100.0)


def set_output_pwm_frequency_khz_input_validation(output_pwm_frequency_khz: int) -> Tuple[bool, Error, str]:
    """Validates output PWM frequency input."""
    return validate_input_range(output_pwm_frequency_khz, 8, 80)


def set_speed_controller_kp_input_validation(speed_controller_kp: float) -> Tuple[bool, Error, str]:
    """Validates speed controller Kp input."""
    return validate_input_range(speed_controller_kp, 0.0, 300.0)


def set_speed_controller_ki_input_validation(speed_controller_ki: float) -> Tuple[bool, Error, str]:
    """Validates speed controller Ki input."""
    return validate_input_range(speed_controller_ki, 0.0, 300.0)


def set_motor_resistance_input_validation(motor_resistance: float) -> Tuple[bool, Error, str]:
    """Validates motor resistance input."""
    return validate_input_range(motor_resistance, 0.001, 100.0)


def set_motor_inductance_input_validation(motor_inductance: float) -> Tuple[bool, Error, str]:
    """Validates motor inductance input."""
    return validate_input_range(motor_inductance, 0.0, 0.2)


def set_motor_poles_counts_input_validation(motor_poles_counts: int) -> Tuple[bool, Error, str]:
    """Validates motor poles counts input."""
    return validate_input_range(motor_poles_counts, 1, 80)


def set_incremental_encoder_lines_input_validation(incremental_encoder_lines: int) -> Tuple[bool, Error, str]:
    """Validates incremental encoder lines input."""
    return validate_input_range(incremental_encoder_lines, 1, 40000)


def set_speed_limit_input_validation(speed_limit: int) -> Tuple[bool, Error, str]:
    """Validates speed limit input."""
    return validate_input_range(speed_limit, 1, 200000)


def set_current_controller_kp_input_validation(current_controller_kp: float) -> Tuple[bool, Error, str]:
    """Validates current controller Kp input."""
    return validate_input_range(current_controller_kp, 0.0, 16000.0)


def set_current_controller_ki_input_validation(current_controller_ki: float) -> Tuple[bool, Error, str]:
    """Validates current controller Ki input."""
    return validate_input_range(current_controller_ki, 0.0, 16000.0)


def set_magnetizing_current_id_reference_input_validation(magnetizing_current_id_reference: float) -> Tuple[bool, Error, str]:
    """Validates magnetizing current Id reference input."""
    return validate_input_range(magnetizing_current_id_reference, 0.0, 200.0)


def set_position_reference_input_validation(position_reference: int) -> Tuple[bool, Error, str]:
    """Validates position reference input."""
    return validate_input_range(position_reference, -2147483647, 2147483647)


def set_position_controller_kp_input_validation(position_controller_kp: float) -> Tuple[bool, Error, str]:
    """Validates position controller Kp input."""
    return validate_input_range(position_controller_kp, 0.0, 16000.0)


def set_position_controller_ki_input_validation(position_controller_ki: float) -> Tuple[bool, Error, str]:
    """Validates position controller Ki input."""
    return validate_input_range(position_controller_ki, 0.0, 16000.0)


def set_zsft_injection_amplitude_input_validation(amplitude: float) -> Tuple[bool, Error, str]:
    """Validates ZSFT injection amplitude input."""
    return validate_input_range(amplitude, 0.0, 0.55)


def set_zsft_polarity_amplitude_input_validation(amplitude: float) -> Tuple[bool, Error, str]:
    """Validates ZSFT polarity amplitude input."""
    return validate_input_range(amplitude, 0.0, 0.55)


def set_zsft_injection_frequency_input_validation(frequency: int) -> Tuple[bool, Error, str]:
    """Validates ZSFT injection frequency input."""
    return validate_input_range(frequency, 0, 10)


def set_observer_gain_dc_input_validation(observer_gain: float) -> Tuple[bool, Error, str]:
    """Validates observer gain DC input."""
    return validate_input_range(observer_gain, 0.01, 1000.0)


def set_filter_gain_bldc_pmsm_input_validation(filter_gain: float) -> Tuple[bool, Error, str]:
    """Validates filter gain for BLDC/PMSM input."""
    return validate_input_range(filter_gain, 0.01, 16000.0)


def set_sensorless_transition_speed_input_validation(sensorless_transition_speed: int) -> Tuple[bool, Error, str]:
    """Validates sensorless transition speed input."""
    return validate_input_range(sensorless_transition_speed, 1, 5000)


def set_encoder_hall_ccw_offset_input_validation(encoder_hall_offset: float) -> Tuple[bool, Error, str]:
    """Validates encoder hall CCW offset input."""
    return validate_input_range(encoder_hall_offset, 0.0, 1.0)


def set_encoder_hall_cw_offset_input_validation(encoder_hall_offset: float) -> Tuple[bool, Error, str]:
    """Validates encoder hall CW offset input."""
    return validate_input_range(encoder_hall_offset, 0.0, 1.0)


def set_speed_acceleration_value_input_validation(speed_acceleration_value: float) -> Tuple[bool, Error, str]:
    """Validates speed acceleration value input."""
    return validate_input_range(speed_acceleration_value, 0.0, 1600.0)


def set_speed_deceleration_value_input_validation(speed_deceleration_value: float) -> Tuple[bool, Error, str]:
    """Validates speed deceleration value input."""
    return validate_input_range(speed_deceleration_value, 0.0, 1600.0)


def set_command_mode_input_validation(mode: CommandMode) -> Tuple[bool, Error, str]:
    """Validates command mode input."""
    if not isinstance(mode, CommandMode):
        if not (isinstance(mode, int) and mode in [0, 1]):
            return False, Error.OUT_OF_RANGE_SETTING, ConstantCommon.INPUT_NEED_ENUM
    return True, Error.NO_ERROR_DETECTED, ""


def set_motor_parameters_identification_input_validation(identification: Action) -> Tuple[bool, Error, str]:
    """Validates motor parameters identification input."""
    if not isinstance(identification, Action):
        if not (isinstance(identification, int) and identification in [0, 1]):
            return False, Error.OUT_OF_RANGE_SETTING, ConstantCommon.INPUT_NEED_ENUM
    return True, Error.NO_ERROR_DETECTED, ""


def set_motor_direction_input_validation(motor_direction: Direction) -> Tuple[bool, Error, str]:
    """Validates motor direction input."""
    if not isinstance(motor_direction, Direction):
        if not (isinstance(motor_direction, int) and motor_direction in [0, 1]):
            return False, Error.OUT_OF_RANGE_SETTING, ConstantCommon.INPUT_NEED_ENUM
    return True, Error.NO_ERROR_DETECTED, ""


def set_feedback_control_mode_input_validation(mode: FeedbackControlMode) -> Tuple[bool, Error, str]:
    """Validates feedback control mode input."""
    if not isinstance(mode, FeedbackControlMode):
        if not (isinstance(mode, int) and mode in [0, 1, 2, 3]):
            return False, Error.OUT_OF_RANGE_SETTING, ConstantCommon.INPUT_NEED_ENUM
    return True, Error.NO_ERROR_DETECTED, ""


def set_motor_type_input_validation(motor_type: MotorType) -> Tuple[bool, Error, str]:
    """Validates motor type input."""
    if not isinstance(motor_type, MotorType):
        if not (isinstance(motor_type, int) and motor_type in [0, 1, 2, 3]):
            return False, Error.OUT_OF_RANGE_SETTING, ConstantCommon.INPUT_NEED_ENUM
    return True, Error.NO_ERROR_DETECTED, ""


def set_control_mode_input_validation(control_mode: ControlMode) -> Tuple[bool, Error, str]:
    """Validates control mode input."""
    if not isinstance(control_mode, ControlMode):
        if not (isinstance(control_mode, int) and control_mode in [0, 1, 2]):
            return False, Error.OUT_OF_RANGE_SETTING, ConstantCommon.INPUT_NEED_ENUM
    return True, Error.NO_ERROR_DETECTED, ""


def set_uart_baudrate_input_validation(baudrate: UartBaudRate) -> Tuple[bool, Error, str]:
    """Validates UART baudrate input."""
    if not isinstance(baudrate, UartBaudRate):
        if not (isinstance(baudrate, int) and baudrate in [0, 1]):
            return False, Error.OUT_OF_RANGE_SETTING, ConstantCommon.INPUT_NEED_ENUM
    return True, Error.NO_ERROR_DETECTED, ""


def sensor_calibration_input_validation(calibration_action: PositionSensorCalibrationAction) -> Tuple[bool, Error, str]:
    """Validates sensor calibration action input."""
    if not isinstance(calibration_action, PositionSensorCalibrationAction):
        if not (isinstance(calibration_action, int) and calibration_action in [0, 1, 2]):
            return False, Error.OUT_OF_RANGE_SETTING, ConstantCommon.INPUT_NEED_ENUM
    return True, Error.NO_ERROR_DETECTED, ""


def motor_parameters_identification_input_validation(identification: Action) -> Tuple[bool, Error, str]:
    """Validates motor parameters identification input."""
    if not isinstance(identification, Action):
        if not (isinstance(identification, int) and identification in [0, 1]):
            return False, Error.OUT_OF_RANGE_SETTING, ConstantCommon.INPUT_NEED_ENUM
    return True, Error.NO_ERROR_DETECTED, ""


def drive_disable_enable_input_validation(action: DisableEnable) -> Tuple[bool, Error, str]:
    """Validates drive disable/enable input."""
    if not isinstance(action, DisableEnable):
        if not (isinstance(action, int) and action in [0, 1]):
            return False, Error.OUT_OF_RANGE_SETTING, ConstantCommon.INPUT_NEED_ENUM
    return True, Error.NO_ERROR_DETECTED, ""


def set_can_bus_baudrate_input_validation(canbus_baudrate: CanBusBaudRate) -> Tuple[bool, Error, str]:
    """Validates CAN bus baudrate input."""
    if not isinstance(canbus_baudrate, CanBusBaudRate):
        if not (isinstance(canbus_baudrate, int) and canbus_baudrate in [1000, 500, 250, 125, 100]):
            return False, Error.OUT_OF_RANGE_SETTING, ConstantCommon.INPUT_NEED_ENUM
    return True, Error.NO_ERROR_DETECTED, ""

def set_analogue_speed_resolution_division_coefficient_input_validation(division_coefficient: float) -> Tuple[bool, Error, str]:
    """Validates analogue speed resolution division coefficient input."""
    return validate_input_range(division_coefficient, 0.0001, 10000.0)


def set_motion_profile_mode_input_validation(motion_profile_mode: MotionProfileMode) -> Tuple[bool, Error, str]:
    """Validates motion profile mode input."""
    if not isinstance(motion_profile_mode, MotionProfileMode):
        if not (isinstance(motion_profile_mode, int) and motion_profile_mode in [0, 1, 2]):
            return False, Error.OUT_OF_RANGE_SETTING, ConstantCommon.INPUT_OUT_OF_RANGE
    return True, Error.NO_ERROR_DETECTED, ""


def set_motion_profile_variable1_input_validation(motion_profile_variable1: float) -> Tuple[bool, Error, str]:
    """Validates motion profile variable 1 input."""
    return validate_input_range(motion_profile_variable1, 0.0, 16000.0)


def set_motion_profile_variable2_input_validation(motion_profile_variable2: float) -> Tuple[bool, Error, str]:
    """Validates motion profile variable 2 input."""
    return validate_input_range(motion_profile_variable2, 0.0, 16000.0)


def set_motion_profile_variable3_input_validation(motion_profile_variable3: float) -> Tuple[bool, Error, str]:
    """Validates motion profile variable 3 input."""
    return validate_input_range(motion_profile_variable3, 0.0, 16000.0)


def set_motion_profile_variable4_input_validation(motion_profile_variable4: float) -> Tuple[bool, Error, str]:
    """Validates motion profile variable 4 input."""
    return validate_input_range(motion_profile_variable4, 0.0, 16000.0)


def set_motion_profile_variable5_input_validation(motion_profile_variable5: float) -> Tuple[bool, Error, str]:
    """Validates motion profile variable 5 input."""
    return validate_input_range(motion_profile_variable5, 0.0, 16000.0)


def set_digital_output_state_input_validation(channel: Channel) -> Tuple[bool, Error, str]:
    """Validates digital output state input."""
    if not isinstance(channel, Channel):
        if not (isinstance(channel, int) and channel in [0, 1, 2, 3]):
            return False, Error.OUT_OF_RANGE_SETTING, ConstantCommon.INPUT_OUT_OF_RANGE
    return True, Error.NO_ERROR_DETECTED, ""


def set_regeneration_current_limit_input_validation(current: float) -> Tuple[bool, Error, str]:
    """Validates regeneration current limit input."""
    return validate_input_range(current, 0.0, 200.0)


def set_position_sensor_digital_filter_level_input_validation(level: int) -> Tuple[bool, Error, str]:
    """Validates position sensor digital filter level input."""
    return validate_input_range(level, 0, 255)
