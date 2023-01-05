# Copyright: (c) 2021, 2022, 2023 SOLO motor controllers project
# GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

# Title: SoloPy
# Author: SOLOMotorControllers
# Date: 2023
# Code version: 3.0.0
# Availability: https://github.com/Solo-FL/SoloPy/tree/main/SoloPy
# This Library is made by SOLOMotorControllers.COM
# please visit:  https://www.SOLOMotorControllers.com/

import can

import logging
from importlib import reload

import SoloPy.ConstantCanopen as ConstantCanopen
import SoloPy.ConstantCommon as ConstantCommon
from SoloPy.SOLOMotorControllers import *
from SoloPy.SOLOMotorControllersUtils import *


class SOLOMotorControllersCanopen(implements(SOLOMotorControllers)):

    def __init__(self,
                 address = 0,
                 baudrate = CAN_BUS_BAUD_RATE.RATE_500,
                 timeout_count = 6000,
                 channel = 'can0',
                 bustype = 'socketcan',
                 loggerLevel=logging.INFO
                 ) -> None:
        if (address == 0):  # address 0 is reserved for the host
            address = 1
        self._address = address
        self._timeout_count = timeout_count
        self._baudrate = baudrate.value*100
        self._channel = channel
        self._bustype = bustype

        # logger init
        logging.shutdown()
        reload(logging)
        self._logger = logging.getLogger('SoloPy')
        self._logger.setLevel(loggerLevel)
        ch = logging.StreamHandler()
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        ch.setFormatter(formatter)
        self._logger.addHandler(ch)
        # logger init end
        self._logger.debug('SoloMotorController INIT')

    def CANOpenTransmit(self, address: int, Object: int, informatrionToSend: list):
        result = None
        error = ERROR.NO_ERROR_DETECTED

        try:
            bus = can.interface.Bus(
                channel=self._channel, bustype=self._bustype, bitrate= self._baudrate )
            msg = can.Message(
                arbitration_id=(0x600 + address),
                data=[0x22,
                    Object.to_bytes(2,byteorder='big')[1],  # LSB Object Index
                    Object.to_bytes(2,byteorder='big')[0],  # MSB Object Index
                    0x00,                                   # Sub Index
                    informatrionToSend[3],
                    informatrionToSend[2],
                    informatrionToSend[1],
                    informatrionToSend[0]],
                is_extended_id=False)

            bus.send(msg)
            informationReceived = bus.recv(1)
            # Abort Checking
            if      ( informationReceived.arbitration_id == (0x580 + address)                   # Check COB-ID
                and ( informationReceived.data[0] == 0x80 )                                     # Check Byte1  
                and ( informationReceived.data[1] == Object.to_bytes(2,byteorder='big')[1] )    # Check Object Index(LSB)
                and ( informationReceived.data[2] == Object.to_bytes(2,byteorder='big')[0] )):  # Check Object Index(MSB)                 
                    if  (   (informationReceived.data[4] == 0x06)
                        and (informationReceived.data[5] == 0x02)
                        and (informationReceived.data[6] == 0x00)
                        and (informationReceived.data[7] == 0x00)):
                        error = ERROR.ABORT_OBJECT
                        result = False

                    elif (   (informationReceived.data[4] == 0x06)
                        and (informationReceived.data[5] == 0x09)
                        and (informationReceived.data[6] == 0x00)
                        and (informationReceived.data[7] == 0x30)):
                        error = ERROR.ABORT_VALUE
                        result = False
            # End Abort Checking
            
            # Check ACK
            if      ( informationReceived.arbitration_id == (0x580 + address)   # Check COB-ID
                and ( informationReceived.data[0] == 0x60)                        # Check Byte1  
                and ( informationReceived.data[1] == Object.to_bytes(2,byteorder='big')[1])    # Check Object Index(LSB)
                and ( informationReceived.data[2] == Object.to_bytes(2,byteorder='big')[0])):  # Check Object Index(MSB)
                error = ERROR.NO_ERROR_DETECTED
                result = True
            # End Check ACK

        except can.exceptions.CanInitializationError :
            error = ERROR.CAN_INITIALIZATION_ERROR
            result = False
        except can.exceptions.CanInterfaceNotImplementedError :
            error = ERROR.CAN_INTERFACE_NOT_IMPLEMENTED_ERROR
            result = False
        except can.exceptions.CanOperationError :
            error = ERROR.CAN_OPERATION_ERROR
            result = False
        except can.exceptions.CanTimeoutError :
            error = ERROR.CAN_TIMEOUT_ERROR
            result = False
        finally:
            return result, error

    def Generic_Canbus_Write(self, _ID: int, _Data: list):
        result = None
        error = ERROR.NO_ERROR_DETECTED

        try:
            bus = can.interface.Bus(
                channel=self._channel, bustype=self._bustype, bitrate= self._baudrate )
            msg = can.Message(
                arbitration_id=_ID,
                data = _Data,
                is_extended_id=False)

            bus.send(msg)

        except can.exceptions.CanInitializationError :
            error = ERROR.CAN_INITIALIZATION_ERROR
            result = False
        except can.exceptions.CanInterfaceNotImplementedError :
            error = ERROR.CAN_INTERFACE_NOT_IMPLEMENTED_ERROR
            result = False
        except can.exceptions.CanOperationError :
            error = ERROR.CAN_OPERATION_ERROR
            result = False
        except  can.exceptions.CanTimeoutError :
            error = ERROR.CAN_TIMEOUT_ERROR
            result = False
        finally:
            return result, error

    def CANOpenReceive(self, address: int, Object: int, informatrionToSend: list) -> list:
        try:
            result = True
            error = ERROR.NO_ERROR_DETECTED
            bus = can.interface.Bus(
                channel=self._channel, bustype=self._bustype, bitrate=self._baudrate)

            msg = can.Message(
                arbitration_id=(0x600 + address),
                data=[0x40,
                    Object.to_bytes(2,byteorder='big')[1],  # LSB Object Index
                    Object.to_bytes(2,byteorder='big')[0],  # MSB Object Index
                    0x00,                                   # Sub Index
                    informatrionToSend[3],
                    informatrionToSend[2],
                    informatrionToSend[1],
                    informatrionToSend[0]],
                is_extended_id=False)

            bus.send(msg)
            informationReceived = bus.recv(1)  # Timeout in seconds.
            if (informationReceived == None):    # No unit response
                result = False
                error = ERROR.GENERAL_ERROR
            
            # Abort Checking
            if ( informationReceived != None
                and informationReceived.arbitration_id == (0x580 + address)                   # Check COB-ID
                and ( informationReceived.data[0] == 0x80 )                                     # Check Byte1  
                and ( informationReceived.data[1] == Object.to_bytes(2,byteorder='big')[1] )    # Check Object Index(LSB)
                and ( informationReceived.data[2] == Object.to_bytes(2,byteorder='big')[0] )):  # Check Object Index(MSB)                 
                    if  (   (informationReceived.data[4] == 0x06)
                        and (informationReceived.data[5] == 0x02)
                        and (informationReceived.data[6] == 0x00)
                        and (informationReceived.data[7] == 0x00)):
                        error = ERROR.ABORT_OBJECT
                        result = False

                    elif (   (informationReceived.data[4] == 0x06)
                        and (informationReceived.data[5] == 0x09)
                        and (informationReceived.data[6] == 0x00)
                        and (informationReceived.data[7] == 0x30)):
                        error = ERROR.ABORT_VALUE
                        result = False
            # End Abort Checking

        except can.exceptions.CanInitializationError :
            error = ERROR.CAN_INITIALIZATION_ERROR
            result = False
        except can.exceptions.CanInterfaceNotImplementedError :
            error = ERROR.CAN_INTERFACE_NOT_IMPLEMENTED_ERROR
            result = False
        except can.exceptions.CanOperationError :
            error = ERROR.CAN_OPERATION_ERROR
            result = False
        except  can.exceptions.CanTimeoutError :
            error = ERROR.CAN_TIMEOUT_ERROR
            result = False
        finally :
            if (result):
                return result, error, ExtractData(list(informationReceived.data))
            else:
                return result, error, None
    
    def Generic_Canbus_Read(self) -> list:
        try:
            result = True
            error = ERROR.NO_ERROR_DETECTED
            bus = can.interface.Bus(
                channel=self._channel, bustype=self._bustype, bitrate=self._baudrate)

            informationReceived = bus.recv(1)  # Timeout in seconds.
            if (informationReceived == None):    # No unit response
                result = False
                error = ERROR.GENERAL_ERROR

        except can.exceptions.CanInitializationError :
            error = ERROR.CAN_INITIALIZATION_ERROR
            result = False
        except can.exceptions.CanInterfaceNotImplementedError :
            error = ERROR.CAN_INTERFACE_NOT_IMPLEMENTED_ERROR
            result = False
        except can.exceptions.CanOperationError :
            error = ERROR.CAN_OPERATION_ERROR
            result = False
        except  can.exceptions.CanTimeoutError :
            error = ERROR.CAN_TIMEOUT_ERROR
            result = False
        finally :
            if (result):
                return result, error, ExtractData(list(informationReceived.data)),informationReceived.dlc,informationReceived.arbitration_id
            else:
                return result, error, None
            
# -----------------------------------------------
# ---------------------Write---------------------
# -----------------------------------------------

    def set_guard_time(self, guard_time: int) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_guard_time_input_validation(guard_time)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        informationToSend = convert_to_data(guard_time, DATA_TYPE.UINT32)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_GuardTime, informationToSend)
        return result, error

    def set_life_time_factor(self, life_time_factor: int) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_life_time_factor_input_validation(life_time_factor)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        informationToSend = convert_to_data(life_time_factor, DATA_TYPE.UINT32)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_LifeTimeFactor, informationToSend)
        return result, error

    def set_producer_heartbeat_time(self, producer_heartbeat_time: int) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_producer_heartbeat_time_input_validation(producer_heartbeat_time)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        informationToSend = convert_to_data(
            producer_heartbeat_time, DATA_TYPE.UINT32)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_ProducerHeartbeatTime, informationToSend)
        return result, error

    def set_device_address(self, device_address: int) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_device_address_input_validation(device_address)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        informationToSend = convert_to_data(device_address, DATA_TYPE.UINT32)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_SetDeviceAddress, informationToSend)
        self._address = device_address
        return result, error 

    def set_command_mode(self, mode: COMMAND_MODE) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_command_mode_input_validation(mode)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error

        if (type(mode) is int):
            mode = COMMAND_MODE(mode)
        
        mode = mode.value
        informationToSend = convert_to_data(mode, DATA_TYPE.UINT32)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_CommandMode, informationToSend)
        return result, error

    def set_current_limit(self, current_limit: float) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_current_limit_input_validation(current_limit)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        current_limit = float(current_limit)
        informationToSend = convert_to_data(current_limit, DATA_TYPE.SFXT)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_CurrentLimit, informationToSend)
        return result, error

    def set_torque_reference_iq(self, torque_reference_iq: float) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_torque_reference_iq_input_validation(torque_reference_iq)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        torque_reference_iq = float(torque_reference_iq)
        informationToSend = convert_to_data(torque_reference_iq, DATA_TYPE.SFXT)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_TorqueReferenceIq, informationToSend)
        return result, error

    def set_speed_reference(self, speed_reference: int) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_speed_reference_input_validation(speed_reference)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        speed_reference = int(speed_reference)
        informationToSend = convert_to_data(speed_reference, DATA_TYPE.UINT32)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_SpeedReference, informationToSend)
        return result, error

    def set_power_reference(self, power_reference: float) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_power_reference_input_validation(power_reference)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        power_reference = float(power_reference)
        informationToSend = convert_to_data(power_reference, DATA_TYPE.SFXT)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_PowerReference, informationToSend)
        return result, error

    def motor_parameters_identification(self, identification: ACTION) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = motor_parameters_identification_input_validation(identification)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error

        if (type(identification) is int):
            identification = COMMAND_MODE(identification)

        identification = identification.value
        informationToSend = convert_to_data(identification, DATA_TYPE.UINT32)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_MotorParametersIdentification, informationToSend)
        return result, error 

    def emergency_stop(self) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_EmergencyStop, informationToSend)
        self._logger.info(
            "SOLO should be manually power recycled to get back into normal operation ")
        return result, error

    def set_output_pwm_frequency_khz(self, output_pwm_frequency_khz: int) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_output_pwm_frequency_khz_input_validation(output_pwm_frequency_khz)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        output_pwm_frequency_khz = int(output_pwm_frequency_khz)
        informationToSend = convert_to_data(
            output_pwm_frequency_khz, DATA_TYPE.UINT32)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_OutputPwmFrequencyKhz, informationToSend)
        return result, error

    def set_speed_controller_kp(self, speed_controller_kp: float) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_speed_controller_kp_input_validation(speed_controller_kp)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        speed_controller_kp = float(speed_controller_kp)
        informationToSend = convert_to_data(speed_controller_kp, DATA_TYPE.SFXT)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_SpeedControllerKp, informationToSend)
        return result, error
    
    def set_speed_controller_ki(self, speed_controller_ki: float) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_speed_controller_ki_input_validation(speed_controller_ki)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        speed_controller_ki = float(speed_controller_ki)
        informationToSend = convert_to_data(
            speed_controller_ki, DATA_TYPE.SFXT)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_SpeedControllerKi, informationToSend)
        return result, error 

    def set_motor_direction(self, motor_direction: DIRECTION) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_motor_direction_input_validation(motor_direction)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error

        if (type(motor_direction) is int):
            motor_direction = COMMAND_MODE(motor_direction)

        motor_direction = motor_direction.value
        informationToSend = convert_to_data(motor_direction, DATA_TYPE.UINT32)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_MotorDirection, informationToSend)
        return result, error

    def set_motor_resistance(self, motor_resistance: float) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_motor_resistance_input_validation(motor_resistance)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        motor_resistance = float(motor_resistance)
        informationToSend = convert_to_data(motor_resistance, DATA_TYPE.SFXT)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_MotorResistance, informationToSend)
        return result, error

    def set_motor_inductance(self, motor_inductance: float) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_motor_inductance_input_validation(motor_inductance)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        motor_inductance = float(motor_inductance)
        informationToSend = convert_to_data(motor_inductance, DATA_TYPE.SFXT)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_MotorInductance, informationToSend)
        return result, error

    def set_motor_poles_counts(self, motor_poles_counts: int) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_motor_poles_counts_input_validation(motor_poles_counts)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        motor_poles_counts = int(motor_poles_counts)
        informationToSend = convert_to_data(motor_poles_counts, DATA_TYPE.UINT32)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_MotorPolesCounts, informationToSend)
        return result, error

    def set_incremental_encoder_lines(self, incremental_encoder_lines: int) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_incremental_encoder_lines_input_validation(incremental_encoder_lines)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        incremental_encoder_lines = int(incremental_encoder_lines)
        informationToSend = convert_to_data(
            incremental_encoder_lines, DATA_TYPE.UINT32)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_IncrementalEncoderLines, informationToSend)
        return result, error

    def set_speed_limit(self, speed_limit: int) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_speed_limit_input_validation(speed_limit)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        speed_limit = int(speed_limit)
        informationToSend = convert_to_data(speed_limit, DATA_TYPE.UINT32)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_SpeedLimit, informationToSend)
        return result, error

    def set_feedback_control_mode(self, mode: FEEDBACK_CONTROL_MODE) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_feedback_control_mode_input_validation(mode)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error

        if (type(mode) is int):
            mode = COMMAND_MODE(mode)

        mode = mode.value
        informationToSend = convert_to_data(mode, DATA_TYPE.UINT32)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_FeedbackControlMode, informationToSend)
        return result, error

    def reset_factory(self) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        informationToSend = [0x00, 0x00, 0x00, 0x01]
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_ResetFactory, informationToSend)
        return result, error

    def set_motor_type(self, motor_type: MOTOR_TYPE) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_motor_type_input_validation(motor_type)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error

        if (type(motor_type) is int):
            motor_type = COMMAND_MODE(motor_type)

        motor_type = motor_type.value
        informationToSend = convert_to_data(motor_type, DATA_TYPE.UINT32)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_MotorType, informationToSend)
        return result, error

    def set_control_mode(self, control_mode: CONTROL_MODE) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_control_mode_input_validation(control_mode)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error

        if (type(control_mode) is int):
            control_mode = CONTROL_MODE(control_mode)
        
        control_mode = control_mode.value
        informationToSend = convert_to_data(control_mode, DATA_TYPE.UINT32)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_ControlMode, informationToSend)
        return result, error

    def set_current_controller_kp(self, current_controller_kp: float) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_current_controller_kp_input_validation(current_controller_kp)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        current_controller_kp = float(current_controller_kp)
        informationToSend = convert_to_data(
            current_controller_kp, DATA_TYPE.SFXT)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_CurrentControllerKp, informationToSend)
        return result, error

    def set_current_controller_ki(self, current_controller_ki: float) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_current_controller_ki_input_validation(current_controller_ki)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        current_controller_ki = float(current_controller_ki)
        informationToSend = convert_to_data(
            current_controller_ki, DATA_TYPE.SFXT)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_CurrentControllerKi, informationToSend)
        return result, error 

    def set_magnetizing_current_id_reference(self, magnetizing_current_id_reference: float) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_magnetizing_current_id_reference_input_validation(magnetizing_current_id_reference)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        magnetizing_current_id_reference = float(magnetizing_current_id_reference)
        informationToSend = convert_to_data(
            magnetizing_current_id_reference, DATA_TYPE.SFXT)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_MagnetizingCurrentIdReference, informationToSend)
        return result, error

    def set_position_reference(self, position_reference: int) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_position_reference_input_validation(position_reference)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        position_reference = int(position_reference)
        informationToSend = convert_to_data(position_reference, DATA_TYPE.INT32)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_PositionReference, informationToSend)
        return result, error

    def set_position_controller_kp(self, position_controller_kp: float) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_position_controller_kp_input_validation(position_controller_kp)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        position_controller_kp = float(position_controller_kp)
        informationToSend = convert_to_data(
            position_controller_kp, DATA_TYPE.SFXT)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_PositionControllerKp, informationToSend)
        return result, error

    def set_position_controller_ki(self, position_controller_ki: float) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_position_controller_ki_input_validation(position_controller_ki)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        position_controller_ki = float(position_controller_ki)
        informationToSend = convert_to_data(
            position_controller_ki, DATA_TYPE.SFXT)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_PositionControllerKi, informationToSend)
        return result, error

    # def reset_position_to_zero(self) -> bool:
    #     error = ERROR.NO_PROCESSED_COMMAND
    #     informationToSend = [0x00, 0x00, 0x00, 0x01]
    #     result, error = self.CANOpenTransmit(
    #         self._address, ConstantCanopen.Object_ResetPositionToZero, informationToSend)
    #     return result, error 

    def overwrite_error_register(self) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_OverwriteErrorRegister, informationToSend)
        return result, error

    def set_observer_gain_bldc_pmsm(self, observer_gain: float) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_observer_gain_bldc_pmsm_input_validation(observer_gain)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        observer_gain = float(observer_gain)
        informationToSend = convert_to_data(observer_gain, DATA_TYPE.SFXT)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_ObserverGainBldcPmsm, informationToSend)
        return result, error

    def set_observer_gain_bldc_pmsm_ultrafast(self, observer_gain: float) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_observer_gain_bldc_pmsm_ultrafast_input_validation(observer_gain)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        observer_gain = float(observer_gain)
        informationToSend = convert_to_data(observer_gain, DATA_TYPE.SFXT)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_ObserverGainBldcPmsmUltrafast, informationToSend)
        return result, error

    def set_observer_gain_dc(self, observer_gain: float) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_observer_gain_dc_input_validation(observer_gain)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        observer_gain = float(observer_gain)
        informationToSend = convert_to_data(observer_gain, DATA_TYPE.SFXT)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_ObserverGainDc, informationToSend)
        return result, error

    def set_filter_gain_bldc_pmsm(self, filter_gain: float) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_filter_gain_bldc_pmsm_input_validation(filter_gain)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        filter_gain = float(filter_gain)
        informationToSend = convert_to_data(filter_gain, DATA_TYPE.SFXT)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_FilterGainBldcPmsm, informationToSend)
        return result, error 

    def set_filter_gain_bldc_pmsm_ultrafast(self, filter_gain: float) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_filter_gain_bldc_pmsm_ultrafast_input_validation(filter_gain)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        filter_gain = float(filter_gain)
        informationToSend = convert_to_data(filter_gain, DATA_TYPE.SFXT)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_FilterGainBldcPmsmUltrafast, informationToSend)
        return result, error 

    def set_uart_baudrate(self, baudrate: UART_BAUD_RATE) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_uart_baudrate_input_validation(baudrate)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error

        if (type(baudrate) is int):
            baudrate = COMMAND_MODE(baudrate)
            
        baudrate = baudrate.value
        informationToSend = convert_to_data(baudrate, DATA_TYPE.UINT32)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_UartBaudrate, informationToSend)
        return result, error 

    def sensor_calibration(self, calibration_action: POSITION_SENSOR_CALIBRATION_ACTION) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = sensor_calibration_input_validation(calibration_action)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error

        if (type(calibration_action) is int):
            calibration_action = COMMAND_MODE(calibration_action)

        calibration_action = calibration_action.value
        informationToSend = convert_to_data(
            calibration_action, DATA_TYPE.UINT32)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_SensorCalibration, informationToSend)
        return result, error 

    def set_encoder_hall_ccw_offset(self, encoder_hall_offset: float) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_encoder_hall_ccw_offset_input_validation(encoder_hall_offset)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        encoder_hall_offset = float(encoder_hall_offset)
        informationToSend = convert_to_data(encoder_hall_offset, DATA_TYPE.SFXT)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_EncoderHallCcwOffset, informationToSend)
        return result, error

    def set_encoder_hall_cw_offset(self, encoder_hall_offset: float) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_encoder_hall_cw_offset_input_validation(encoder_hall_offset)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        encoder_hall_offset = float(encoder_hall_offset)
        informationToSend = convert_to_data(encoder_hall_offset, DATA_TYPE.SFXT)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_EncoderHallCwOffset, informationToSend)
        return result, error

    def set_speed_acceleration_value(self, speed_acceleration_value: float) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_speed_acceleration_value_input_validation(speed_acceleration_value)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        speed_acceleration_value = float(speed_acceleration_value)
        informationToSend = convert_to_data(
            speed_acceleration_value, DATA_TYPE.SFXT)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_SpeedAccelerationValue, informationToSend)
        return result, error

    def set_speed_deceleration_value(self, speed_deceleration_value: float) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_speed_deceleration_value_input_validation(speed_deceleration_value)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        speed_deceleration_value = float(speed_deceleration_value)
        informationToSend = convert_to_data(
            speed_deceleration_value, DATA_TYPE.SFXT)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_SpeedDecelerationValue, informationToSend)
        return result, error

    def set_can_bus_baudrate(self, canbus_baudrate: CAN_BUS_BAUD_RATE) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_can_bus_baudrate_input_validation(canbus_baudrate)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error

        if (type(canbus_baudrate) is int):
            canbus_baudrate = COMMAND_MODE(canbus_baudrate)
        
        canbus_baudrate = canbus_baudrate.value
        informationToSend = convert_to_data(canbus_baudrate, DATA_TYPE.UINT32)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_CanbusBaudrate, informationToSend)
        return result, error
# ----------------------------------------------
# ---------------------Read---------------------
# ----------------------------------------------

    def get_read_error_register(self) -> int:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_ReadErrorRegister, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.UINT32), error
        return -1, error

    def get_guard_time(self) -> int:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_GuardTime, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.UINT32), error
        return -1, error

    def get_life_time_factor(self) -> int:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_LifeTimeFactor, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.UINT32), error
        return -1, error

    def get_producer_heartbeat_time(self) -> int:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_ProducerHeartbeatTime, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.UINT32), error
        return -1, error

    def get_device_address(self) -> int:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_SetDeviceAddress, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.UINT32), error
        return -1, error

    def get_phase_a_voltage(self) -> float:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_PhaseAVoltage, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    def get_phase_b_voltage(self) -> float:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_PhaseBVoltage, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    def get_phase_a_current(self) -> float:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_PhaseACurrent, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    def get_phase_b_current(self) -> float:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_PhaseBCurrent, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    def get_bus_voltage(self) -> float:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_BusVoltage, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    def get_dc_motor_current_im(self) -> float:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_DcMotorCurrentIm, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    def get_dc_motor_voltage_vm(self) -> float:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_DcMotorVoltageVm, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    def get_speed_controller_kp(self) -> float:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_SpeedControllerKp, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    def get_speed_controller_ki(self) -> float:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_SpeedControllerKi, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    def get_output_pwm_frequency_khz(self) -> int:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_OutputPwmFrequencyKhz, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return round(convert_from_data(informationReceived, DATA_TYPE.UINT32)/ 1000,0), error
        return -1, error

    def get_current_limit(self) -> float:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_CurrentLimit, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    def get_quadrature_current_iq_feedback(self) -> float:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_QuadratureCurrentIqFeedback, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    def get_magnetizing_current_id_feedback(self) -> float:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_MagnetizingCurrentIdFeedback, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    def get_motor_poles_counts(self) -> int:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_MotorPolesCounts, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.UINT32), error
        return -1, error

    def get_incremental_encoder_lines(self) -> int:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_IncrementalEncoderLines, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.UINT32), error
        return -1, error

    def get_current_controller_kp(self) -> float:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_CurrentControllerKp, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    def get_current_controller_ki(self) -> float:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_CurrentControllerKi, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    def get_board_temperature(self) -> float:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_BoardTemperature, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    def get_motor_resistance(self) -> float:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_MotorResistance, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    def get_motor_inductance(self) -> float:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_MotorInductance, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    def get_speed_feedback(self) -> int:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_SpeedFeedback, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.UINT32), error
        return -1, error

    def get_motor_type(self) -> MOTOR_TYPE:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_MotorType, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return MOTOR_TYPE(convert_from_data(informationReceived, DATA_TYPE.UINT32)), error
        return -1, error

    def get_feedback_control_mode(self) -> FEEDBACK_CONTROL_MODE:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_FeedbackControlMode, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return FEEDBACK_CONTROL_MODE(convert_from_data(informationReceived, DATA_TYPE.UINT32)), error
        return -1, error

    def get_command_mode(self) -> COMMAND_MODE:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_CommandMode, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return COMMAND_MODE(convert_from_data(informationReceived, DATA_TYPE.UINT32)), error
        return -1, error

    def get_control_mode(self) -> CONTROL_MODE:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_ControlMode, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return CONTROL_MODE(convert_from_data(informationReceived, DATA_TYPE.UINT32)), error
        return -1, error

    def get_speed_limit(self) -> int:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_SpeedLimit, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.UINT32), error
        return -1, error

    def get_position_controller_kp(self) -> float:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_PositionControllerKp, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    def get_position_controller_ki(self) -> float:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_PositionControllerKi, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    def get_position_counts_feedback(self) -> int:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_PositionCountsFeedback, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.UINT32), error
        return -1, error

    def get_error_register(self) -> int:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_OverwriteErrorRegister, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.UINT32), error
        return -1, error

    def get_device_firmware_version(self) -> int:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_DeviceFirmwareVersion, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.UINT32), error
        return -1, error

    def get_device_hardware_version(self) -> int:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_DeviceHardwareVersion, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.UINT32), error
        return -1, error

    def get_torque_reference_iq(self) -> float:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_TorqueReferenceIq, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    def get_speed_reference(self) -> int:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_SpeedReference, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.UINT32), error
        return -1, error

    def get_magnetizing_current_id_reference(self) -> float:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_MagnetizingCurrentIdReference, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    def get_position_reference(self) -> int:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_PositionReference, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.UINT32), error
        return -1, error

    def get_power_reference(self) -> float:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_PowerReference, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    def get_motor_direction(self) -> DIRECTION:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_MotorDirection, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return DIRECTION(convert_from_data(informationReceived, DATA_TYPE.UINT32)), error
        return -1, error

    def get_observer_gain_bldc_pmsm(self) -> float:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_ObserverGainBldcPmsm, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    def get_observer_gain_bldc_pmsm_ultrafast(self) -> float:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_ObserverGainBldcPmsmUltrafast, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    def get_observer_gain_dc(self) -> float:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_ObserverGainDc, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    def get_filter_gain_bldc_pmsm(self) -> float:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_FilterGainBldcPmsm, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    def get_filter_gain_bldc_pmsm_ultrafast(self) -> float:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_FilterGainBldcPmsmUltrafast, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    def get_3phase_motor_angle(self) -> float:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_3PhaseMotorAngle, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    def get_encoder_hall_ccw_offset(self) -> float:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_EncoderHallCcwOffset, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    def get_encoder_hall_cw_offset(self) -> float:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_EncoderHallCwOffset, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    def get_uart_baudrate(self) -> UART_BAUD_RATE:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_UartBaudrate, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return UART_BAUD_RATE(convert_from_data(informationReceived, DATA_TYPE.UINT32)), error
        return -1, error

    def get_speed_acceleration_value(self) -> float:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_SpeedAccelerationValue, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    def get_speed_deceleration_value(self) -> float:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_SpeedDecelerationValue, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    def communication_is_working(self) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        temperature, error = self.get_board_temperature()
        time.sleep(0.2)
        temperature, error = self.get_board_temperature()
        if (error == ERROR.NO_ERROR_DETECTED):
            return True, error
        return False, error

    def get_encoder_index_counts(self) -> int:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_EncoderIndexCounts, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.UINT32), error
        return -1, error
