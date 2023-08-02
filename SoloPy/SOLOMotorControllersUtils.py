# Copyright: (c) 2021, 2022, 2023 SOLO motor controllers project
# GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

# Title: SoloPy
# Author: SOLOMotorControllers
# Date: 2023
# Code version: 3.1.3
# Availability: https://github.com/Solo-FL/SoloPy/tree/main/SoloPy
# This Library is made by SOLOMotorControllers.COM
# please visit:  https://www.SOLOMotorControllers.com/

from SoloPy.SOLOMotorControllers import *
import SoloPy.ConstantCommon as ConstantCommon


def convert_to_data(number, dataType: DATA_TYPE) -> list:
    data = []
    if (dataType == DATA_TYPE.SFXT):
        data = SFXT_to_data(number)

    if ( dataType == DATA_TYPE.INT32):
        data = INT32_to_data(number)
        
    if (dataType == DATA_TYPE.UINT32):
        data = UINT32_to_data(number)

    return data

def SFXT_to_data(number) -> list:
    data = []
    dec = math.ceil(number * 131072)
    if dec < 0:
        dec *= -1
        dec = 0xFFFFFFFF - dec
    data = [(dec >> i & 0xff) for i in (24, 16, 8, 0)]
    return data

def UINT32_to_data(number) -> list:
    dec = number
    data = [(dec >> i & 0xff) for i in (24, 16, 8, 0)]
    return data

def INT32_to_data(number) -> list:
    i32Value = number * 1;
    if (i32Value < 0):
        i32Value = 4294967295 - abs(i32Value) + 1;
    return UINT32_to_data(i32Value)

def convert_from_data(data, dataType: DATA_TYPE):
    if (dataType == DATA_TYPE.SFXT):
        return convert_to_float(data)

    if (dataType == DATA_TYPE.INT32):
        return convert_to_int(data)

    if (dataType == DATA_TYPE.UINT32):
        return convert_to_long(data)

def convert_to_float(data) -> float:
    dec = 0
    dec = int.from_bytes(
        [data[0], data[1], data[2], data[3]], byteorder='big', signed=False)
    if (dec <= 0x7FFE0000):
        value = (float)(dec / 131072.0)
        return float(format(value, 'f'))  # .8f
    else:
        dec = 0xFFFFFFFF - dec + 1
        value = ((float)(dec / 131072.0)) * -1
        return float(format(value, '.8f'))  # .8f

def convert_to_long(data) -> int:
    dec = 0
    dec = int.from_bytes(
        [data[0], data[1], data[2], data[3]], byteorder='big', signed=False)
    return dec

def convert_to_int(data) -> int:
    dec = convert_to_long(data)
    if (dec > 2147483647):
        dec = (4294967295 - dec + 1)*-1
    return dec

def ExtractData(Data: list) -> list:
    ExtractedData = [0, 0, 0, 0]
    if(len(Data)>=8):
        ExtractedData[0] = Data[7]
        ExtractedData[1] = Data[6]
        ExtractedData[2] = Data[5]
        ExtractedData[3] = Data[4]
    return ExtractedData

def get_data(cmd: list) -> list:
    return [cmd[2], cmd[3], cmd[4], cmd[5]]

# -- input Validation

def set_guard_time_input_validation(guardtime: int) -> list:
    if (guardtime < 0 or guardtime > 65535):
        return False, ERROR.OUT_OF_RANGE_SETTING, ConstantCommon.InputOutOfRange
    return True, ERROR.NO_ERROR_DETECTED, ""


def set_life_time_factor_input_validation(lifeTimeFactor: int) -> list:
    if (lifeTimeFactor < 0 or lifeTimeFactor > 255):
        return False, ERROR.OUT_OF_RANGE_SETTING, ConstantCommon.InputOutOfRange
    return True, ERROR.NO_ERROR_DETECTED, ""


def set_producer_heartbeat_time_input_validation(producerHeartbeatTime: int) -> list:
    if (producerHeartbeatTime < 0 or producerHeartbeatTime > 65535):
        return False, ERROR.OUT_OF_RANGE_SETTING, ConstantCommon.InputOutOfRange
    return True, ERROR.NO_ERROR_DETECTED, ""


def set_device_address_input_validation(deviceAddress: int) -> list:
    if (deviceAddress < 0 or deviceAddress > 254):
        return False, ERROR.OUT_OF_RANGE_SETTING, ConstantCommon.InputOutOfRange
    return True, ERROR.NO_ERROR_DETECTED, ""


def set_current_limit_input_validation(currentLimit: float) -> list:
    if (currentLimit < 0 or currentLimit > 32):
        return False, ERROR.OUT_OF_RANGE_SETTING, ConstantCommon.InputOutOfRange
    return True, ERROR.NO_ERROR_DETECTED, ""


def set_torque_reference_iq_input_validation(torqueReferenceIq: float) -> list:
    if (torqueReferenceIq < 0 or torqueReferenceIq > 32):
        return False, ERROR.OUT_OF_RANGE_SETTING, ConstantCommon.InputOutOfRange
    return True, ERROR.NO_ERROR_DETECTED, ""


def set_speed_reference_input_validation(speedReference: int) -> list:
    if (speedReference < 0 or speedReference > 30000):
        return False, ERROR.OUT_OF_RANGE_SETTING, ConstantCommon.InputOutOfRange
    return True, ERROR.NO_ERROR_DETECTED, ""


def set_power_reference_input_validation(powerReference: float) -> list:
    if (powerReference < 0 or powerReference > 100):
        return False, ERROR.OUT_OF_RANGE_SETTING, ConstantCommon.InputOutOfRange
    return True, ERROR.NO_ERROR_DETECTED, ""


def set_output_pwm_frequency_khz_input_validation(outputPwmFrequencyKhz: int) -> list:
    if (outputPwmFrequencyKhz < 8 or outputPwmFrequencyKhz > 80):
        return False, ERROR.OUT_OF_RANGE_SETTING, ConstantCommon.InputOutOfRange
    return True, ERROR.NO_ERROR_DETECTED, ""


def set_speed_controller_kp_input_validation(speedControllerKp: float) -> list:
    if (speedControllerKp < 0 or speedControllerKp > 300):
        return False, ERROR.OUT_OF_RANGE_SETTING, ConstantCommon.InputOutOfRange
    return True, ERROR.NO_ERROR_DETECTED, ""


def set_speed_controller_ki_input_validation(speedControllerKi: float) -> list:
    if (speedControllerKi < 0 or speedControllerKi > 300):
        return False, ERROR.OUT_OF_RANGE_SETTING, ConstantCommon.InputOutOfRange
    return True, ERROR.NO_ERROR_DETECTED, ""


def set_motor_resistance_input_validation(motorResistance: float) -> list:
    if (motorResistance < 0.001 or motorResistance > 50):
        return False, ERROR.OUT_OF_RANGE_SETTING, ConstantCommon.InputOutOfRange
    return True, ERROR.NO_ERROR_DETECTED, ""


def set_motor_inductance_input_validation(motorInductance: float) -> list:
    if (motorInductance < 0.00001 or motorInductance > 0.2):
        return False, ERROR.OUT_OF_RANGE_SETTING, ConstantCommon.InputOutOfRange
    return True, ERROR.NO_ERROR_DETECTED, ""


def set_motor_poles_counts_input_validation(motorPolesCounts: int) -> list:
    if (motorPolesCounts < 1 or motorPolesCounts > 80):
        return False, ERROR.OUT_OF_RANGE_SETTING, ConstantCommon.InputOutOfRange
    return True, ERROR.NO_ERROR_DETECTED, ""


def set_incremental_encoder_lines_input_validation(incrementalEncoderLines: int) -> list:
    if (incrementalEncoderLines < 1 or incrementalEncoderLines > 40000):
        return False, ERROR.OUT_OF_RANGE_SETTING, ConstantCommon.InputOutOfRange
    return True, ERROR.NO_ERROR_DETECTED, ""


def set_speed_limit_input_validation(speedLimit: int) -> list:
    if (speedLimit < 1 or speedLimit > 30000):
        return False, ERROR.OUT_OF_RANGE_SETTING, ConstantCommon.InputOutOfRange
    return True, ERROR.NO_ERROR_DETECTED, ""


def set_current_controller_kp_input_validation(currentControllerKp: float) -> list:
    if (currentControllerKp < 0 or currentControllerKp > 16000):
        return False, ERROR.OUT_OF_RANGE_SETTING, ConstantCommon.InputOutOfRange
    return True, ERROR.NO_ERROR_DETECTED, ""


def set_current_controller_ki_input_validation(currentControllerKi: float) -> list:
    if (currentControllerKi < 0 or currentControllerKi > 16000):
        return False, ERROR.OUT_OF_RANGE_SETTING, ConstantCommon.InputOutOfRange
    return True, ERROR.NO_ERROR_DETECTED, ""


def set_magnetizing_current_id_reference_input_validation(magnetizingCurrentIdReference: float) -> list:
    if (magnetizingCurrentIdReference < 0 or magnetizingCurrentIdReference > 32):
        return False, ERROR.OUT_OF_RANGE_SETTING, ConstantCommon.InputOutOfRange
    return True, ERROR.NO_ERROR_DETECTED, ""


def set_position_reference_input_validation(positionReference: int) -> list:
    if (positionReference < -2147483647 or positionReference > 2147483647):
        return False, ERROR.OUT_OF_RANGE_SETTING
    return True, ERROR.NO_ERROR_DETECTED, ""


def set_position_controller_kp_input_validation(positionControllerKp: float) -> list:
    if (positionControllerKp < 0 or positionControllerKp > 16000):
        return False, ERROR.OUT_OF_RANGE_SETTING, ConstantCommon.InputOutOfRange
    return True, ERROR.NO_ERROR_DETECTED, ""


def set_position_controller_ki_input_validation(positionControllerKi: float) -> list:
    if (positionControllerKi < 0 or positionControllerKi > 16000):
        return False, ERROR.OUT_OF_RANGE_SETTING, ConstantCommon.InputOutOfRange
    return True, ERROR.NO_ERROR_DETECTED, ""


def set_observer_gain_bldc_pmsm_input_validation(observerGain: float) -> list:
    if (observerGain < 0.01 or observerGain > 1000):
        return False, ERROR.OUT_OF_RANGE_SETTING, ConstantCommon.InputOutOfRange
    return True, ERROR.NO_ERROR_DETECTED, ""


def set_observer_gain_bldc_pmsm_ultrafast_input_validation(observerGain: float) -> list:
    if (observerGain < 0.01 or observerGain > 1000):
        return False, ERROR.OUT_OF_RANGE_SETTING, ConstantCommon.InputOutOfRange
    return True, ERROR.NO_ERROR_DETECTED, ""


def set_observer_gain_dc_input_validation(observerGain: float) -> list:
    if (observerGain < 0.01 or observerGain > 1000):
        return False, ERROR.OUT_OF_RANGE_SETTING, ConstantCommon.InputOutOfRange
    return True, ERROR.NO_ERROR_DETECTED, ""


def set_filter_gain_bldc_pmsm_input_validation(filterGain: float) -> list:
    if (filterGain < 0.01 or filterGain > 16000):
        return False, ERROR.OUT_OF_RANGE_SETTING, ConstantCommon.InputOutOfRange
    return True, ERROR.NO_ERROR_DETECTED, ""


def set_filter_gain_bldc_pmsm_ultrafast_input_validation(filterGain: float) -> list:
    if (filterGain < 0.01 or filterGain > 16000):
        return False, ERROR.OUT_OF_RANGE_SETTING, ConstantCommon.InputOutOfRange
    return True, ERROR.NO_ERROR_DETECTED, ""


def set_encoder_hall_ccw_offset_input_validation(encoderHallOffset: float) -> list:
    if (encoderHallOffset <= 0 or encoderHallOffset >= 1):
        return False, ERROR.OUT_OF_RANGE_SETTING, ConstantCommon.InputOutOfRange
    return True, ERROR.NO_ERROR_DETECTED, ""


def set_encoder_hall_cw_offset_input_validation(encoderHallOffset: float) -> list:
    if (encoderHallOffset <= 0 or encoderHallOffset >= 1):
        return False, ERROR.OUT_OF_RANGE_SETTING, ConstantCommon.InputOutOfRange
    return True, ERROR.NO_ERROR_DETECTED, ""


def set_speed_acceleration_value_input_validation(speedAccelerationValue: float) -> list:
    if (speedAccelerationValue < 0 or speedAccelerationValue > 1600):
        return False, ERROR.OUT_OF_RANGE_SETTING, ConstantCommon.InputOutOfRange
    return True, ERROR.NO_ERROR_DETECTED, ""


def set_speed_deceleration_value_input_validation(speedDecelerationValue: float) -> list:
    if (speedDecelerationValue < 0 or speedDecelerationValue > 1600):
        return False, ERROR.OUT_OF_RANGE_SETTING, ConstantCommon.InputOutOfRange
    return True, ERROR.NO_ERROR_DETECTED, ""


def set_command_mode_input_validation(mode: COMMAND_MODE) -> list:
    if not isinstance(mode, COMMAND_MODE):
        if ((type(mode) is int and (mode == 0 or mode == 1)) is False):
            return False, ERROR.OUT_OF_RANGE_SETTING, ConstantCommon.InputNeedEnum
    return True, ERROR.NO_ERROR_DETECTED, ""


def set_motor_parameters_identification_input_validation(identification: ACTION) -> list:
    if not isinstance(identification, ACTION):
        if ((type(identification) is int and (identification == 0 or identification == 1)) is False):
            return False, ERROR.OUT_OF_RANGE_SETTING, ConstantCommon.InputNeedEnum
    return True, ERROR.NO_ERROR_DETECTED, ""


def set_motor_direction_input_validation(motor_direction: DIRECTION) -> list:
    if not isinstance(motor_direction, DIRECTION):
        if ((type(motor_direction) is int and (motor_direction == 0 or motor_direction == 1)) is False):
            return False, ERROR.OUT_OF_RANGE_SETTING, ConstantCommon.InputNeedEnum
    return True, ERROR.NO_ERROR_DETECTED, ""


def set_feedback_control_mode_input_validation(mode: FEEDBACK_CONTROL_MODE) -> list:
    if not isinstance(mode, FEEDBACK_CONTROL_MODE):
        if ((type(mode) is int and (mode == 0 or mode == 1 or mode == 2)) is False):
            return False, ERROR.OUT_OF_RANGE_SETTING, ConstantCommon.InputNeedEnum
    return True, ERROR.NO_ERROR_DETECTED, ""


def set_motor_type_input_validation(motor_type: MOTOR_TYPE) -> list:
    if not isinstance(motor_type, MOTOR_TYPE):
        if ((type(motor_type) is int and (motor_type == 0 or motor_type == 1 or motor_type == 2 or motor_type == 3)) is False):
            return False, ERROR.OUT_OF_RANGE_SETTING, ConstantCommon.InputNeedEnum
    return True, ERROR.NO_ERROR_DETECTED, ""


def set_control_mode_input_validation(control_mode: CONTROL_MODE) -> list:
    if not isinstance(control_mode, CONTROL_MODE):
        if ((type(control_mode) is int and (control_mode == 0 or control_mode == 1 or control_mode == 2)) is False):
            return False, ERROR.OUT_OF_RANGE_SETTING, ConstantCommon.InputNeedEnum
    return True, ERROR.NO_ERROR_DETECTED, ""


def set_uart_baudrate_input_validation(baudrate: UART_BAUD_RATE) -> list:
    if not isinstance(baudrate, UART_BAUD_RATE):
        if ((type(baudrate) is int and (baudrate == 0 or baudrate == 1)) is False):
            return False, ERROR.OUT_OF_RANGE_SETTING, ConstantCommon.InputNeedEnum
    return True, ERROR.NO_ERROR_DETECTED, ""


def sensor_calibration_input_validation(calibration_action: POSITION_SENSOR_CALIBRATION_ACTION) -> list:
    if not isinstance(calibration_action, POSITION_SENSOR_CALIBRATION_ACTION):
        if ((type(calibration_action) is int and (calibration_action == 0 or calibration_action == 1 or calibration_action == 2)) is False):
            return False, ERROR.OUT_OF_RANGE_SETTING, ConstantCommon.InputNeedEnum
    return True, ERROR.NO_ERROR_DETECTED, ""


def motor_parameters_identification_input_validation(identification: ACTION) -> list:
    if not isinstance(identification, ACTION):
        if ((type(identification) is int and (identification == 0 or identification == 1)) is False):
            return False, ERROR.OUT_OF_RANGE_SETTING, ConstantCommon.InputNeedEnum
    return True, ERROR.NO_ERROR_DETECTED, ""


def set_can_bus_baudrate_input_validation(canbus_baudrate: CAN_BUS_BAUD_RATE) -> list:
    if not isinstance(canbus_baudrate, CAN_BUS_BAUD_RATE):
        if ((type(canbus_baudrate) is int and (canbus_baudrate == 1000 or canbus_baudrate == 500 or canbus_baudrate == 250 or canbus_baudrate == 125 or canbus_baudrate == 100)) is False):
            return False, ERROR.OUT_OF_RANGE_SETTING, ConstantCommon.InputNeedEnum
    return True, ERROR.NO_ERROR_DETECTED, ""
    
def set_analogue_speed_resolution_division_coefficient_input_validation(division_coefficient: float) -> list:
    if (division_coefficient < 0.0001 or division_coefficient > 10000):
        return False, ERROR.OUT_OF_RANGE_SETTING, ConstantCommon.InputOutOfRange
    return True, ERROR.NO_ERROR_DETECTED, ""
    
def set_motion_profile_mode_input_validation(motion_profile_mode: MOTION_PROFILE_MODE) -> list:
    if not isinstance(motion_profile_mode, MOTION_PROFILE_MODE):
        if ((type(motion_profile_mode) is int and (motion_profile_mode == 0 or motion_profile_mode == 1 or motion_profile_mode == 2)) is False):
            return False, ERROR.OUT_OF_RANGE_SETTING, ConstantCommon.InputOutOfRange
    return True, ERROR.NO_ERROR_DETECTED, ""
    
def set_motion_profile_variable1_input_validation(motion_profile_variable1: float) -> list:
    if (motion_profile_variable1 < 0 or motion_profile_variable1 > 16000):
        return False, ERROR.OUT_OF_RANGE_SETTING, ConstantCommon.InputOutOfRange
    return True, ERROR.NO_ERROR_DETECTED, ""
    
def set_motion_profile_variable2_input_validation(motion_profile_variable2: float) -> list:
    if (motion_profile_variable2 < 0 or motion_profile_variable2 > 16000):
        return False, ERROR.OUT_OF_RANGE_SETTING, ConstantCommon.InputOutOfRange
    return True, ERROR.NO_ERROR_DETECTED, ""
    
def set_motion_profile_variable3_input_validation(motion_profile_variable3: float) -> list:
    if (motion_profile_variable3 < 0 or motion_profile_variable3 > 16000):
        return False, ERROR.OUT_OF_RANGE_SETTING, ConstantCommon.InputOutOfRange
    return True, ERROR.NO_ERROR_DETECTED, ""
    
def set_motion_profile_variable4_input_validation(motion_profile_variable4: float) -> list:
    if (motion_profile_variable4 < 0 or motion_profile_variable4 > 16000):
        return False, ERROR.OUT_OF_RANGE_SETTING, ConstantCommon.InputOutOfRange
    return True, ERROR.NO_ERROR_DETECTED, ""
    
def set_motion_profile_variable5_input_validation(motion_profile_variable5: float) -> list:
    if (motion_profile_variable5 < 0 or motion_profile_variable5 > 16000):
        return False, ERROR.OUT_OF_RANGE_SETTING, ConstantCommon.InputOutOfRange
    return True, ERROR.NO_ERROR_DETECTED, ""