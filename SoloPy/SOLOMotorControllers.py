# Copyright: (c) 2021, 2022, 2023 SOLO motor controllers project
# GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

# Title: SoloPy
# Author: SOLOMotorControllers
# Date: 2023
# Code version: 3.0.0
# Availability: https://github.com/Solo-FL/SoloPy/tree/main/SoloPy
# This Library is made by SOLOMotorControllers.COM
# please visit:  https://www.SOLOMotorControllers.com/

import time
import math
from enum import Enum
from interface import implements, Interface


class ERROR(Enum):
    NO_ERROR_DETECTED = 0
    GENERAL_ERROR = 1
    NO_PROCESSED_COMMAND = 2
    OUT_OF_RANGE_SETTING = 3
    PACKET_FAILURE_TRIAL_ATTEMPTS_OVERFLOW = 4
    RECIEVE_TIMEOUT_ERROR = 5
    ABORT_OBJECT = 6
    ABORT_VALUE = 7
    CAN_INITIALIZATION_ERROR = 8
    CAN_INTERFACE_NOT_IMPLEMENTED_ERROR = 9
    CAN_OPERATION_ERROR = 10
    CAN_TIMEOUT_ERROR = 11


class COMMAND_MODE(Enum):
    ANALOGUE = 0
    DIGITAL = 1


class DIRECTION(Enum):
    CLOCKWISE = 0
    COUNTERCLOCKWISE = 1


class FEEDBACK_CONTROL_MODE(Enum):
    SENSOR_LESS = 0
    ENCODERS = 1
    HALL_SENSORS = 2


class CONTROL_MODE(Enum):
    SPEED_MODE = 0
    TORQUE_MODE = 1
    POSITION_MODE = 2


class MOTOR_TYPE(Enum):
    DC = 0
    BLDC_PMSM = 1
    ACIM = 2
    BLDC_PMSM_ULTRAFAST = 3


class UART_BAUD_RATE(Enum):  # [bits/s]
    RATE_937500 = 0
    RATE_115200 = 1


class CAN_BUS_BAUD_RATE(Enum):  # [bits/s]
    RATE_1000 = 1000
    RATE_500 = 500
    RATE_250 = 250
    RATE_125 = 125
    RATE_100 = 100


class ACTION (Enum):
    STOP = 0
    START = 1


class POSITION_SENSOR_CALIBRATION_ACTION(Enum):
    STOP_CALIBRATION = 0
    INCREMENTAL_ENCODER_START_CALIBRATION = 1
    HALL_SENSOR_START_CALIBRATION = 2


class DATA_TYPE(Enum):
    UINT32 = 0
    INT32 = 1
    SFXT = 2


class SOLOMotorControllers(Interface):

    def set_device_address(self, device_address: int) -> bool:
        pass

    def set_command_mode(self, mode: COMMAND_MODE) -> bool:
        pass

    def set_current_limit(self, current_limit: float) -> bool:
        pass

    def set_torque_reference_iq(self, torque_reference_iq: float) -> bool:
        pass

    def set_speed_reference(self, speed_reference: int) -> bool:
        pass

    def set_power_reference(self, power_reference: float) -> bool:
        pass

    def motor_parameters_identification(self, identification: ACTION) -> bool:
        pass

    def emergency_stop(self) -> bool:
        pass

    def set_output_pwm_frequency_khz(self, output_pwm_frequency_khz: int) -> bool:
        pass

    def set_speed_controller_kp(self, speed_controller_kp: float) -> bool:
        pass

    def set_speed_controller_ki(self, speed_controller_ki: float) -> bool:
        pass

    def set_motor_direction(self, motor_direction: DIRECTION) -> bool:
        pass

    def set_motor_resistance(self, motor_resistance: float) -> bool:
        pass

    def set_motor_inductance(self, motor_inductance: float) -> bool:
        pass

    def set_motor_poles_counts(self, motor_poles_counts: int) -> bool:
        pass

    def set_incremental_encoder_lines(self, incremental_encoder_lines: int) -> bool:
        pass

    def set_speed_limit(self, speed_limit: int) -> bool:
        pass

    def set_feedback_control_mode(self, mode: FEEDBACK_CONTROL_MODE) -> bool:
        pass

    def reset_factory(self) -> bool:
        pass

    def set_motor_type(self, motor_type: MOTOR_TYPE) -> bool:
        pass

    def set_control_mode(self, control_mode: CONTROL_MODE) -> bool:
        pass

    def set_current_controller_kp(self, current_controller_kp: float) -> bool:
        pass

    def set_current_controller_ki(self, current_controller_ki: float) -> bool:
        pass

    def set_magnetizing_current_id_reference(self, magnetizing_current_id_reference: float) -> bool:
        pass

    def set_position_reference(self, position_reference: int) -> bool:
        pass

    def set_position_controller_kp(self, position_controller_kp: float) -> bool:
        pass

    def set_position_controller_ki(self, position_controller_ki: float) -> bool:
        pass

    def overwrite_error_register(self) -> bool:
        pass

    def set_observer_gain_bldc_pmsm(self, observer_gain: float) -> bool:
        pass

    def set_observer_gain_bldc_pmsm_ultrafast(self, observer_gain: float) -> bool:
        pass

    def set_observer_gain_dc(self, observer_gain: float) -> bool:
        pass

    def set_filter_gain_bldc_pmsm(self, filter_gain: float) -> bool:
        pass

    def set_filter_gain_bldc_pmsm_ultrafast(self, filter_gain: float) -> bool:
        pass

    def set_uart_baudrate(self, baudrate: UART_BAUD_RATE) -> bool:
        pass

    def sensor_calibration(self, calibration_action: POSITION_SENSOR_CALIBRATION_ACTION) -> bool:
        pass

    def set_encoder_hall_ccw_offset(self, encoder_hall_offset: float) -> bool:
        pass

    def set_encoder_hall_cw_offset(self, encoder_hall_offset: float) -> bool:
        pass

    def set_speed_acceleration_value(self, speed_acceleration_value: float) -> bool:
        pass

    def set_speed_deceleration_value(self, speed_deceleration_value: float) -> bool:
        pass

    def set_can_bus_baudrate(self, canbus_baudrate: CAN_BUS_BAUD_RATE) -> bool:
        pass

    def get_device_address(self) -> int:
        pass

    def get_phase_a_voltage(self) -> float:
        pass

    def get_phase_b_voltage(self) -> float:
        pass

    def get_phase_a_current(self) -> float:
        pass

    def get_phase_b_current(self) -> float:
        pass

    def get_bus_voltage(self) -> float:
        pass

    def get_dc_motor_current_im(self) -> float:
        pass

    def get_dc_motor_voltage_vm(self) -> float:
        pass

    def get_speed_controller_kp(self) -> float:
        pass

    def get_speed_controller_ki(self) -> float:
        pass

    def get_output_pwm_frequency_khz(self) -> int:
        pass

    def get_current_limit(self) -> float:
        pass

    def get_quadrature_current_iq_feedback(self) -> float:
        pass

    def get_magnetizing_current_id_feedback(self) -> float:
        pass

    def get_motor_poles_counts(self) -> int:
        pass

    def get_incremental_encoder_lines(self) -> int:
        pass

    def get_current_controller_kp(self) -> float:
        pass

    def get_current_controller_ki(self) -> float:
        pass

    def get_board_temperature(self) -> float:
        pass

    def get_motor_resistance(self) -> float:
        pass

    def get_motor_inductance(self) -> float:
        pass

    def get_speed_feedback(self) -> int:
        pass

    def get_motor_type(self) -> MOTOR_TYPE:
        pass

    def get_feedback_control_mode(self) -> FEEDBACK_CONTROL_MODE:
        pass

    def get_command_mode(self) -> COMMAND_MODE:
        pass

    def get_control_mode(self) -> CONTROL_MODE:
        pass

    def get_speed_limit(self) -> int:
        pass

    def get_position_controller_kp(self) -> float:
        pass

    def get_position_controller_ki(self) -> float:
        pass

    def get_position_counts_feedback(self) -> int:
        pass

    def get_error_register(self) -> int:
        pass

    def get_device_firmware_version(self) -> int:
        pass

    def get_device_hardware_version(self) -> int:
        pass

    def get_torque_reference_iq(self) -> float:
        pass

    def get_speed_reference(self) -> int:
        pass

    def get_magnetizing_current_id_reference(self) -> float:
        pass

    def get_position_reference(self) -> int:
        pass

    def get_power_reference(self) -> float:
        pass

    def get_motor_direction(self) -> DIRECTION:
        pass

    def get_observer_gain_bldc_pmsm(self) -> float:
        pass

    def get_observer_gain_bldc_pmsm_ultrafast(self) -> float:
        pass

    def get_observer_gain_dc(self) -> float:
        pass

    def get_filter_gain_bldc_pmsm(self) -> float:
        pass

    def get_filter_gain_bldc_pmsm_ultrafast(self) -> float:
        pass

    def get_3phase_motor_angle(self) -> float:
        pass

    def get_encoder_hall_ccw_offset(self) -> float:
        pass

    def get_encoder_hall_cw_offset(self) -> float:
        pass

    def get_uart_baudrate(self) -> UART_BAUD_RATE:
        pass

    def get_speed_acceleration_value(self) -> float:
        pass

    def get_speed_deceleration_value(self) -> float:
        pass

    def communication_is_working(self) -> bool:
        pass

    def get_encoder_index_counts(self) -> int:
        pass
