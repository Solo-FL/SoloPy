## @package SOLOMotorControllers.py
#  @author  SOLOMotorControllers
#  @brief   This file contains all the functions prototypes for the Solo Drivers 
#           Availability: https://github.com/Solo-FL/SoloPy/tree/main/SoloPy
#  @date    Date: 2023
#  @version 3.1.1

## @attention
# Copyright: (c) 2021-2023 SOLO motor controllers project
# GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

import time
import math
from enum import Enum
from interface import implements, Interface

##
# @brief  Error enumeration definition
#
class ERROR(Enum):
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

##
# @brief  Command Mode enumeration definitio
#
class COMMAND_MODE(Enum):
    ## Analogue Mode
    ANALOGUE = 0
    ## Digital Mode
    DIGITAL = 1

##
# @brief  Direction enumeration definition
#
class DIRECTION(Enum):
    ## clockwise direction
    CLOCKWISE = 0
    ## counter-clockwise direction
    COUNTERCLOCKWISE = 1

##
# @brief  Feedback Control Mode enumeration definition
#
class FEEDBACK_CONTROL_MODE(Enum):
    ## sensorless mode
    SENSOR_LESS = 0
    ## encoders mode
    ENCODERS = 1
    ##hall sensors mode
    HALL_SENSORS = 2

##
# @brief  Control Mode enumeration definition
#
class CONTROL_MODE(Enum):
    ## speed mode
    SPEED_MODE = 0
    ## torque mode
    TORQUE_MODE = 1
    ## position mode
    POSITION_MODE = 2

##
# @brief  Motor Type enumeration definition
#
class MOTOR_TYPE(Enum):
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
class UART_BAUD_RATE(Enum):  # [bits/s]
    ## baud rate 937500
    RATE_937500 = 0
    ## baud rate 115200
    RATE_115200 = 1

##
# @brief  Canbus Baudrate enumeration definition
#
class CAN_BUS_BAUD_RATE(Enum):  # [bits/s]
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
class ACTION (Enum):
    ## stop
    STOP = 0
    ## st
    START = 1

##
# @brief  Position Sensor Calibration Action enumeration definition
#
class POSITION_SENSOR_CALIBRATION_ACTION(Enum):
    ## stop colibration
    STOP_CALIBRATION = 0
    ## incremental encoder start calibration
    INCREMENTAL_ENCODER_START_CALIBRATION = 1
    ## hall sensor start calibration
    HALL_SENSOR_START_CALIBRATION = 2

##
# @brief  Data Type enumeration definition
#
class DATA_TYPE(Enum):
    ## uint32
    UINT32 = 0
    ## int32
    INT32 = 1
    ## sfxt
    SFXT = 2
    
## 
#@brief  Motion Profile Mode enumeration definition
#
class MOTION_PROFILE_MODE(Enum):
    ## step ramp service
    STEP_RAMP_RESPNSE = 0   
    ## time based st curve
    TIME_BASE_STCURVE = 1    
    ## time optimal st curve
    TIME_OPTIMAL_STCURVE = 2   


class SOLOMotorControllers(Interface):

    def set_device_address(self, device_address: int) -> list:
        pass

    def set_command_mode(self, mode: COMMAND_MODE) -> list:
        pass

    def set_current_limit(self, current_limit: float) -> list:
        pass

    def set_torque_reference_iq(self, torque_reference_iq: float) -> list:
        pass

    def set_speed_reference(self, speed_reference: int) -> list:
        pass

    def set_power_reference(self, power_reference: float) -> list:
        pass

    def motor_parameters_identification(self, identification: ACTION) -> list:
        pass

    def emergency_stop(self) -> list:
        pass

    def set_output_pwm_frequency_khz(self, output_pwm_frequency_khz: int) -> list:
        pass

    def set_speed_controller_kp(self, speed_controller_kp: float) -> list:
        pass

    def set_speed_controller_ki(self, speed_controller_ki: float) -> list:
        pass

    def set_motor_direction(self, motor_direction: DIRECTION) -> list:
        pass

    def set_motor_resistance(self, motor_resistance: float) -> list:
        pass

    def set_motor_inductance(self, motor_inductance: float) -> list:
        pass

    def set_motor_poles_counts(self, motor_poles_counts: int) -> list:
        pass

    def set_incremental_encoder_lines(self, incremental_encoder_lines: int) -> list:
        pass

    def set_speed_limit(self, speed_limit: int) -> list:
        pass

    def set_feedback_control_mode(self, mode: FEEDBACK_CONTROL_MODE) -> list:
        pass

    def reset_factory(self) -> list:
        pass

    def set_motor_type(self, motor_type: MOTOR_TYPE) -> list:
        pass

    def set_control_mode(self, control_mode: CONTROL_MODE) -> list:
        pass

    def set_current_controller_kp(self, current_controller_kp: float) -> list:
        pass

    def set_current_controller_ki(self, current_controller_ki: float) -> list:
        pass

    def set_magnetizing_current_id_reference(self, magnetizing_current_id_reference: float) -> list:
        pass

    def set_position_reference(self, position_reference: int) -> list:
        pass

    def set_position_controller_kp(self, position_controller_kp: float) -> list:
        pass

    def set_position_controller_ki(self, position_controller_ki: float) -> list:
        pass

    def overwrite_error_register(self) -> list:
        pass

    def set_observer_gain_bldc_pmsm(self, observer_gain: float) -> list:
        pass

    def set_observer_gain_bldc_pmsm_ultrafast(self, observer_gain: float) -> list:
        pass

    def set_observer_gain_dc(self, observer_gain: float) -> list:
        pass

    def set_filter_gain_bldc_pmsm(self, filter_gain: float) -> list:
        pass

    def set_filter_gain_bldc_pmsm_ultrafast(self, filter_gain: float) -> list:
        pass

    def set_uart_baudrate(self, baudrate: UART_BAUD_RATE) -> list:
        pass

    def sensor_calibration(self, calibration_action: POSITION_SENSOR_CALIBRATION_ACTION) -> list:
        pass

    def set_encoder_hall_ccw_offset(self, encoder_hall_offset: float) -> list:
        pass

    def set_encoder_hall_cw_offset(self, encoder_hall_offset: float) -> list:
        pass

    def set_speed_acceleration_value(self, speed_acceleration_value: float) -> list:
        pass

    def set_speed_deceleration_value(self, speed_deceleration_value: float) -> list:
        pass

    def set_can_bus_baudrate(self, canbus_baudrate: CAN_BUS_BAUD_RATE) -> list:
        pass
        
    def set_analogue_speed_resolution_division_coefficient(self, division_coefficient: float) -> list:
        pass
    def set_motion_profile_mode(self, motion_profile_mode: MOTION_PROFILE_MODE) -> list:
        pass
    def set_motion_profile_variable1(self, motion_profile_variable1: float) -> list:
        pass
    def set_motion_profile_variable2(self, motion_profile_variable2: float) -> list:
        pass
    def set_motion_profile_variable3(self, motion_profile_variable3: float) -> list:
        pass
    def set_motion_profile_variable4(self, motion_profile_variable4: float) -> list:
        pass
    def set_motion_profile_variable5(self, motion_profile_variable5: float) -> list: 
        pass
                                                                               
    def get_device_address(self) -> list:                                       
        pass                                                                   
                                                                               
    def get_phase_a_voltage(self) -> list:
        pass

    def get_phase_b_voltage(self) -> list:
        pass

    def get_phase_a_current(self) -> list:
        pass

    def get_phase_b_current(self) -> list:
        pass

    def get_bus_voltage(self) -> list:
        pass

    def get_dc_motor_current_im(self) -> list:
        pass

    def get_dc_motor_voltage_vm(self) -> list:
        pass

    def get_speed_controller_kp(self) -> list:
        pass

    def get_speed_controller_ki(self) -> list:
        pass

    def get_output_pwm_frequency_khz(self) -> list:
        pass

    def get_current_limit(self) -> list:
        pass

    def get_quadrature_current_iq_feedback(self) -> list:
        pass

    def get_magnetizing_current_id_feedback(self) -> list:
        pass

    def get_motor_poles_counts(self) -> list:
        pass

    def get_incremental_encoder_lines(self) -> list:
        pass

    def get_current_controller_kp(self) -> list:
        pass

    def get_current_controller_ki(self) -> list:
        pass

    def get_board_temperature(self) -> list:
        pass

    def get_motor_resistance(self) -> list:
        pass

    def get_motor_inductance(self) -> list:
        pass

    def get_speed_feedback(self) -> list:
        pass

    def get_motor_type(self) -> list:
        pass

    def get_feedback_control_mode(self) -> list:
        pass

    def get_command_mode(self) -> list:
        pass

    def get_control_mode(self) -> list:
        pass

    def get_speed_limit(self) -> list:
        pass

    def get_position_controller_kp(self) -> list:
        pass

    def get_position_controller_ki(self) -> list:
        pass

    def get_position_counts_feedback(self) -> list:
        pass

    def get_error_register(self) -> list:
        pass

    def get_device_firmware_version(self) -> list:
        pass

    def get_device_hardware_version(self) -> list:
        pass

    def get_torque_reference_iq(self) -> list:
        pass

    def get_speed_reference(self) -> list:
        pass

    def get_magnetizing_current_id_reference(self) -> list:
        pass

    def get_position_reference(self) -> list:
        pass

    def get_power_reference(self) -> list:
        pass

    def get_motor_direction(self) -> list:
        pass

    def get_observer_gain_bldc_pmsm(self) -> list:
        pass

    def get_observer_gain_bldc_pmsm_ultrafast(self) -> list:
        pass

    def get_observer_gain_dc(self) -> list:
        pass

    def get_filter_gain_bldc_pmsm(self) -> list:
        pass

    def get_filter_gain_bldc_pmsm_ultrafast(self) -> list:
        pass

    def get_3phase_motor_angle(self) -> list:
        pass

    def get_encoder_hall_ccw_offset(self) -> list:
        pass

    def get_encoder_hall_cw_offset(self) -> list:
        pass

    def get_uart_baudrate(self) -> list:
        pass
        
    def get_analogue_speed_resolution_division_coefficient(self) -> list:
        pass
                                                                                                                                   
    def get_speed_acceleration_value(self) -> list:
        pass

    def get_speed_deceleration_value(self) -> list:
        pass

    def communication_is_working(self) -> list:
        pass

    def get_encoder_index_counts(self) -> list:
        pass
    
    def get_motion_profile_mode(self) -> list:
        pass
        
    def get_motion_profile_variable1(self) -> list:
        pass
        
    def get_motion_profile_variable2(self) -> list:
        pass

    def get_motion_profile_variable3(self) -> list:
        pass

    def get_motion_profile_variable4(self) -> list:
        pass

    def get_motion_profile_variable5(self) -> list:
        pass
