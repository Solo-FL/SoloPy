## @package SOLOMotorControllers.py
#  @author  SOLOMotorControllers
#  @brief   This file contains all the functions prototypes for the Solo Drivers
#           Availability: https://github.com/Solo-FL/SoloPy/tree/main/SoloPy
#  @date    Date: 2025
#  @version 4.0.0

## @attention
# Copyright: (c) 2021-present, SOLO motor controllers project
# MIT License (see LICENSE file for more details)

from enum import Enum
#from interface import Interface
from typing import Tuple

##
# @brief  Error enumeration definition
#
class Error(Enum):
    ## no error detected
    NO_ERROR_DETECTED = 0
    ## general error
    GENERAL_ERROR = 1
    ## command is not valid
    NO_PROCESSED_COMMAND = 2
    ## setting is out of range
    OUT_OF_RANGE_SETTING = 3
    ## trial attempt overflow
    PACKET_FAILURE_TRIAL_ATTEMPTS_OVERFLOW = 4
    ## receive time error
    RECIEVE_TIMEOUT_ERROR = 5
    ## abort object
    ABORT_OBJECT = 6
    ## abort value
    ABORT_VALUE = 7
    ## can initialization error
    CAN_INITIALIZATION_ERROR = 8
    ## can initialazation not implemented error
    CAN_INTERFACE_NOT_IMPLEMENTED_ERROR = 9
    ## can operation error
    CAN_OPERATION_ERROR = 10
    ## can timeout error
    CAN_TIMEOUT_ERROR = 11
    ## PDO configuration id out of range
    PDO_PARAMETER_ID_OUT_OF_RANGE = 12
	## PDO configuration sync out of range
    PDO_SYNC_OUT_OF_RANGE = 13
	## PDO no specific CobId for the specified pdo
    PDO_MISSING_COBID = 14
	## PDO RTR specific command not allowed
    PDO_RTR_COMMAND_NOT_ALLOWED = 15

##
# @brief  Command Mode enumeration definitio
#
class CommandMode(Enum):
    ## Analogue Mode
    ANALOGUE = 0
    ## Digital Mode
    DIGITAL = 1
    ## Analogue with Digital Speed Gain Mode
    ANALOGUE_WITH_DIGITAL_SPEED_GAIN = 2
    ## Command Mode Error
    COMMAND_MODE_ERROR = -1

##
# @brief  Direction enumeration definition
#
class Direction(Enum):
    ## clockwise direction
    CLOCKWISE = 0
    ## counter-clockwise direction
    COUNTERCLOCKWISE = 1

##
# @brief  Feedback Control Mode enumeration definition
#
class FeedbackControlMode(Enum):
    ## sensorless mode
    SENSORLESS_HSO = 0
    ## encoders mode
    ENCODERS = 1
    ## hall sensors mode
    HALL_SENSORS = 2
    ## sensorless ZSFT mode
    SENSORLESS_ZSFT = 3

##
# @brief  Control Mode enumeration definition
#
class ControlMode(Enum):
    ## speed mode
    SPEED_MODE = 0
    ## torque mode
    TORQUE_MODE = 1
    ## position mode
    POSITION_MODE = 2

##
# @brief  Motor Type enumeration definition
#
class MotorType(Enum):
    ## dc motor
    DC = 0
    ## brushless dc motor
    BLDC_PMSM = 1
    ## acim motor
    ACIM = 2
    ## brushless dc motor fast
    BLDC_PMSM_ULTRAFAST = 3

##
# @brief  Uart Baudrate enumeration definition
#
class UartBaudRate(Enum):  # [bits/s]
    ## baud rate 937500
    RATE_937500 = 0
    ## baud rate 115200
    RATE_115200 = 1

##
# @brief  Canbus Baudrate enumeration definition
#
class CanBusBaudRate(Enum):  # [bits/s]
    ## Baudrate 1000 kbits/s
    RATE_1000 = 1000
    ## Baudrate 500 kbits/s
    RATE_500 = 500
    ## Baudrate 250 kbits/s
    RATE_250 = 250
    ## Baudrate 125 kbits/s
    RATE_125 = 125
    ## Baudrate 100 kbits/s
    RATE_100 = 100

##
# @brief  Action enumeration definition
#
class Action (Enum):
    ## stop
    STOP = 0
    ## st
    START = 1

##
# @brief  Disable/Enable enumeration definition
#
class DisableEnable (Enum):
    ## Disable
    DISABLE = 0
    ## Enable
    ENABLE = 1
    ## Disable/Enable Error
    DISABLE_ENABLE_ERROR = -1

##
# @brief  Position Sensor Calibration Action enumeration definition
#
class PositionSensorCalibrationAction(Enum):
    ## stop colibration
    STOP_CALIBRATION = 0
    ## incremental encoder start calibration
    INCREMENTAL_ENCODER_START_CALIBRATION = 1
    ## hall sensor start calibration
    HALL_SENSOR_START_CALIBRATION = 2

##
# @brief  Data Type enumeration definition
#
class DataType(Enum):
    ## uint32
    UINT32 = 0
    ## int32
    INT32 = 1
    ## sfxt
    SFXT = 2

##
# @brief  Motion Profile Mode enumeration definition
#
class MotionProfileMode(Enum):
    ## step ramp service
    STEP_RAMP_RESPNSE = 0
    ## time based st curve
    TIME_BASE_STCURVE = 1
    ## time optimal st curve
    TIME_OPTIMAL_STCURVE = 2

##
# @brief  Digital IO State enumeration definition
#
class DigitalIoState(Enum):
    ## Low IO State
    LOW_IO_STATE = 0
    ## High IO State
    HIGH_IO_STATE = 1



##
# @brief  Channel enumeration definition
#
class Channel(Enum):
    ## channel0
    CHANNEL0 = 0
    ## channel1
    CHANNEL1 = 1
    ## channel2
    CHANNEL2 = 2
    ## channel3
    CHANNEL3 = 3

#class SOLOMotorControllers(Interface):
class SOLOMotorControllers:

    def set_device_address(self, device_address: int) -> Tuple[bool, Error]:
        pass

    def set_command_mode(self, mode: CommandMode) -> Tuple[bool, Error]:
        pass

    def set_current_limit(self, current_limit: float) -> Tuple[bool, Error]:
        pass

    def set_torque_reference_iq(self, torque_reference_iq: float) -> Tuple[bool, Error]:
        pass

    def set_speed_reference(self, speed_reference: int) -> Tuple[bool, Error]:
        pass

    def set_power_reference(self, power_reference: float) -> Tuple[bool, Error]:
        pass

    def motor_parameters_identification(self, identification: Action) -> Tuple[bool, Error]:
        pass

    def set_drive_disable_enable(self, action : DisableEnable) -> Tuple[bool, Error]:
        pass

    def set_output_pwm_frequency_khz(self, output_pwm_frequency_khz: int) -> Tuple[bool, Error]:
        pass

    def set_speed_controller_kp(self, speed_controller_kp: float) -> Tuple[bool, Error]:
        pass

    def set_speed_controller_ki(self, speed_controller_ki: float) -> Tuple[bool, Error]:
        pass

    def set_motor_direction(self, motor_direction: Direction) -> Tuple[bool, Error]:
        pass

    def set_motor_resistance(self, motor_resistance: float) -> Tuple[bool, Error]:
        pass

    def set_motor_inductance(self, motor_inductance: float) -> Tuple[bool, Error]:
        pass

    def set_motor_poles_counts(self, motor_poles_counts: int) -> Tuple[bool, Error]:
        pass

    def set_incremental_encoder_lines(self, incremental_encoder_lines: int) -> Tuple[bool, Error]:
        pass

    def set_speed_limit(self, speed_limit: int) -> Tuple[bool, Error]:
        pass

    def set_feedback_control_mode(self, feedback_control_mode: FeedbackControlMode) -> Tuple[bool, Error]:
        pass

    def reset_factory(self) -> Tuple[bool, Error]:
        pass

    def set_motor_type(self, motor_type: MotorType) -> Tuple[bool, Error]:
        pass

    def set_control_mode(self, control_mode: ControlMode) -> Tuple[bool, Error]:
        pass

    def set_current_controller_kp(self, current_controller_kp: float) -> Tuple[bool, Error]:
        pass

    def set_current_controller_ki(self, current_controller_ki: float) -> Tuple[bool, Error]:
        pass

    def set_magnetizing_current_id_reference(self, magnetizing_current_id_reference: float) -> Tuple[bool, Error]:
        pass

    def set_position_reference(self, position_reference: int) -> Tuple[bool, Error]:
        pass

    def set_position_controller_kp(self, position_controller_kp: float) -> Tuple[bool, Error]:
        pass

    def set_position_controller_ki(self, position_controller_ki: float) -> Tuple[bool, Error]:
        pass

    def overwrite_error_register(self) -> Tuple[bool, Error]:
        pass

    def set_zsft_injection_amplitude(self, zsft_injection_amplitude: float) -> Tuple[bool, Error]:
        pass

    def set_zsft_polarity_amplitude(self, zsft_polarity_amplitude: float) -> Tuple[bool, Error]:
        pass

    def set_observer_gain_dc(self, observer_gain: float) -> Tuple[bool, Error]:
        pass

    def set_zsft_injection_frequency(self, zsft_injection_frequency: int) -> Tuple[bool, Error]:
        pass

    def set_sensorless_transition_speed(self, sensorless_transition_speed: int) -> Tuple[bool, Error]:
        pass

    def set_uart_baudrate(self, baudrate: UartBaudRate) -> Tuple[bool, Error]:
        pass

    def sensor_calibration(self, calibration_action: PositionSensorCalibrationAction) -> Tuple[bool, Error]:
        pass

    def set_encoder_hall_ccw_offset(self, encoder_hall_offset: float) -> Tuple[bool, Error]:
        pass

    def set_encoder_hall_cw_offset(self, encoder_hall_offset: float) -> Tuple[bool, Error]:
        pass

    def set_speed_acceleration_value(self, speed_acceleration_value: float) -> Tuple[bool, Error]:
        pass

    def set_speed_deceleration_value(self, speed_deceleration_value: float) -> Tuple[bool, Error]:
        pass

    def set_can_bus_baudrate(self, canbus_baudrate: CanBusBaudRate) -> Tuple[bool, Error]:
        pass

    def set_analogue_speed_resolution_division_coefficient(self, division_coefficient: float) -> Tuple[bool, Error]:
        pass

    def set_motion_profile_mode(self, motion_profile_mode: MotionProfileMode) -> Tuple[bool, Error]:
        pass

    def set_motion_profile_variable1(self, motion_profile_variable1: float) -> Tuple[bool, Error]:
        pass

    def set_motion_profile_variable2(self, motion_profile_variable2: float) -> Tuple[bool, Error]:
        pass

    def set_motion_profile_variable3(self, motion_profile_variable3: float) -> Tuple[bool, Error]:
        pass

    def set_motion_profile_variable4(self, motion_profile_variable4: float) -> Tuple[bool, Error]:
        pass

    def set_motion_profile_variable5(self, motion_profile_variable5: float) -> Tuple[bool, Error]:
        pass

    def set_digital_output_state(self, channel: Channel, state: DigitalIoState) -> Tuple[bool, Error]:
        pass

    def get_device_address(self) -> Tuple[int, Error]:
        pass

    def get_phase_a_voltage(self) -> Tuple[float, Error]:
        pass

    def get_phase_b_voltage(self) -> Tuple[float, Error]:
        pass

    def get_phase_a_current(self) -> Tuple[float, Error]:
        pass

    def get_phase_b_current(self) -> Tuple[float, Error]:
        pass

    def get_bus_voltage(self) -> Tuple[float, Error]:
        pass

    def get_dc_motor_current_im(self) -> Tuple[float, Error]:
        pass

    def get_dc_motor_voltage_vm(self) -> Tuple[float, Error]:
        pass

    def get_speed_controller_kp(self) -> Tuple[float, Error]:
        pass

    def get_speed_controller_ki(self) -> Tuple[float, Error]:
        pass

    def get_output_pwm_frequency_khz(self) -> Tuple[int, Error]:
        pass

    def get_current_limit(self) -> Tuple[float, Error]:
        pass

    def get_quadrature_current_iq_feedback(self) -> Tuple[float, Error]:
        pass

    def get_magnetizing_current_id_feedback(self) -> Tuple[float, Error]:
        pass

    def get_motor_poles_counts(self) -> Tuple[int, Error]:
        pass

    def get_incremental_encoder_lines(self) -> Tuple[int, Error]:
        pass

    def get_current_controller_kp(self) -> Tuple[float, Error]:
        pass

    def get_current_controller_ki(self) -> Tuple[float, Error]:
        pass

    def get_board_temperature(self) -> Tuple[float, Error]:
        pass

    def get_motor_resistance(self) -> Tuple[float, Error]:
        pass

    def get_motor_inductance(self) -> Tuple[float, Error]:
        pass

    def get_speed_feedback(self) -> Tuple[int, Error]:
        pass

    def get_motor_type(self) -> Tuple[MotorType, Error]:
        pass

    def get_feedback_control_mode(self) -> Tuple[FeedbackControlMode, Error]:
        pass

    def get_command_mode(self) -> Tuple[CommandMode, Error]:
        pass

    def get_control_mode(self) -> Tuple[ControlMode, Error]:
        pass

    def get_speed_limit(self) -> Tuple[int, Error]:
        pass

    def get_position_controller_kp(self) -> Tuple[float, Error]:
        pass

    def get_position_controller_ki(self) -> Tuple[float, Error]:
        pass

    def get_position_counts_feedback(self) -> Tuple[int, Error]:
        pass

    def get_error_register(self) -> Tuple[int, Error]:
        pass

    def get_device_firmware_version(self) -> Tuple[int, Error]:
        pass

    def get_device_hardware_version(self) -> Tuple[int, Error]:
        pass

    def get_torque_reference_iq(self) -> Tuple[float, Error]:
        pass

    def get_speed_reference(self) -> Tuple[int, Error]:
        pass

    def get_magnetizing_current_id_reference(self) -> Tuple[float, Error]:
        pass

    def get_position_reference(self) -> Tuple[int, Error]:
        pass

    def get_power_reference(self) -> Tuple[float, Error]:
        pass

    def get_motor_direction(self) -> Tuple[UartBaudRate, Error]:
        pass

    def get_zsft_injection_amplitude(self) -> Tuple[float, Error]:
        pass

    def get_zsft_polarity_amplitude(self) -> Tuple[float, Error]:
        pass

    def get_zsft_injection_frequency(self) -> Tuple[float, Error]:
        pass

    def get_filter_gain_bldc_pmsm(self) -> Tuple[float, Error]:
        pass

    def get_sensorless_transition_speed(self) -> Tuple[float, Error]:
        pass

    def get_3phase_motor_angle(self) -> Tuple[float, Error]:
        pass

    def get_encoder_hall_ccw_offset(self) -> Tuple[float, Error]:
        pass

    def get_encoder_hall_cw_offset(self) -> Tuple[float, Error]:
        pass

    def get_uart_baudrate(self) -> Tuple[UartBaudRate, Error]:
        pass

    def get_analogue_speed_resolution_division_coefficient(self) -> Tuple[float, Error]:
        pass

    def get_speed_acceleration_value(self) -> Tuple[float, Error]:
        pass

    def get_speed_deceleration_value(self) -> Tuple[float, Error]:
        pass

    def communication_is_working(self) -> Tuple[bool, Error]:
        pass

    def get_encoder_index_counts(self) -> Tuple[int, Error]:
        pass

    def get_motion_profile_mode(self) -> Tuple[MotionProfileMode, Error]:
        pass

    def get_motion_profile_variable1(self) -> Tuple[float, Error]:
        pass

    def get_motion_profile_variable2(self) -> Tuple[float, Error]:
        pass

    def get_motion_profile_variable3(self) -> Tuple[float, Error]:
        pass

    def get_motion_profile_variable4(self) -> Tuple[float, Error]:
        pass

    def get_motion_profile_variable5(self) -> Tuple[float, Error]:
        pass

    def get_digital_outputs_register(self) -> Tuple[int, Error]:
        pass

    def get_drive_disable_enable(self) -> Tuple[DisableEnable, Error]:
        pass

    def get_regeneration_current_limit(self) -> Tuple[int, Error]:
        pass

    def get_position_sensor_digital_filter_level(self) -> Tuple[int, Error]:
        pass

    def get_pt1000_sensor_voltage(self) -> Tuple[int, Error]:
        pass

    def get_analogue_input(self, channel: Channel) -> Tuple[int, Error]:
        pass
