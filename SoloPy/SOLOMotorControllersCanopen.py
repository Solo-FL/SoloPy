## @package SOLOMotorControllersCanopen.py
#  @author  SOLOMotorControllers
#  @brief   This file contains all the functions for the Solo Canopen Drivers 
#           Availability: https://github.com/Solo-FL/SoloPy/tree/main/SoloPy
#  @date    Date: 2023
#  @version 3.1.3

## @attention
# Copyright: (c) 2021-2023 SOLO motor controllers project
# GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

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
            bus.shutdown()
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
            bus.shutdown()
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
            bus.shutdown()
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
            bus.shutdown()
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

    def set_guard_time(self, guard_time: int) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_guard_time_input_validation(guard_time)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        informationToSend = convert_to_data(guard_time, DATA_TYPE.UINT32)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_GuardTime, informationToSend)
        return result, error

    def set_life_time_factor(self, life_time_factor: int) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_life_time_factor_input_validation(life_time_factor)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        informationToSend = convert_to_data(life_time_factor, DATA_TYPE.UINT32)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_LifeTimeFactor, informationToSend)
        return result, error

    def set_producer_heartbeat_time(self, producer_heartbeat_time: int) -> list:
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

    ##
    #@brief  This command sets the desired device address for a SOLO unit
    #            .The method refers to the Object Dictionary: 0x3001
    #@param  device_address  address want to set for board       
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_device_address(self, device_address: int) -> list:
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

    ##
    #@brief  This command sets the mode of the operation of SOLO
    #        in terms of operating in Analogue mode or Digital
    #          .The method refers to the Object Dictionary: 0x3002
    #@param  mode  enum that specify mode of the operation of SOLO      
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_command_mode(self, mode: COMMAND_MODE) -> list:
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

    ##
    #@brief  This command defines the maximum allowed current into the motor in terms of Amps
    #           .The method refers to the Object Dictionary: 0x3003
    #@param  current_limit  a float value [Amps]     
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_current_limit(self, current_limit: float) -> list:
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

    ##
    #@brief  This command sets the amount of desired current that acts in torque generation
    #          .The method refers to the Object Dictionary: 0x3004
    #@param  torque_reference_iq  a float [Amps]      
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_torque_reference_iq(self, torque_reference_iq: float) -> list:
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

    ##
    #@brief  This command defines the speed reference for SOLO once it’s in Digital Speed Mode
    #          .The method refers to the Object Dictionary: 0x3005
    #@param  speed_reference  a long value [RPM]      
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_speed_reference(self, speed_reference: int) -> list:
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

    ##
    #@brief  This command defines the amount of power percentage during only
    #        Open-loop mode for 3-phase motors
    #          .The method refers to the Object Dictionary: 0x3006
    #@param  power_reference  a float value between 0 to 100       
    #@retval bool 0 fail / 1 for succes
    def set_power_reference(self, power_reference: float) -> list:
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

    ##
    #@brief  By putting 1 in the DATA section of a packet sent with this command, SOLO will start
    #        identifying the electrical parameters of the Motor connected
    #          .The method refers to the Object Dictionary: 0x3007
    #@param  powerReference  enum that specify Start or Stop of something in SOLO      
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def motor_parameters_identification(self, identification: ACTION) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = motor_parameters_identification_input_validation(identification)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error

        if (type(identification) is int):
            identification = ACTION(identification)

        identification = identification.value
        informationToSend = convert_to_data(identification, DATA_TYPE.UINT32)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_MotorParametersIdentification, informationToSend)
        return result, error 

    ##
    #@brief  This command if the DATA is set at zero will stop the whole power and switching system
    #        connected to the motor and it will cut the current floating into the Motor from SOLO 
    #          .The method refers to the Object Dictionary: 0x3008     
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def emergency_stop(self) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_EmergencyStop, informationToSend)
        self._logger.info(
            "SOLO should be manually power recycled to get back into normal operation ")
        return result, error

    ##
    #@brief  This command sets the output switching frequency of the whole power unit on the Motor
    #          .The method refers to the Object Dictionary: 0x3009
    #@param  output_pwm_frequency_khz  switching frequencies [kHz]      
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_output_pwm_frequency_khz(self, output_pwm_frequency_khz: int) -> list:
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

    ##
    #@brief  This command sets the Speed controller Kp Gain, and it will
    #        be functional only in Digital Closed-loop mode
    #          .The method refers to the Object Dictionary: 0x300A
    #@param  speed_controller_kp  a float value between 0 to 300     
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_speed_controller_kp(self, speed_controller_kp: float) -> list:
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
    
    ##
    #@brief  This command sets the Speed controller Ki gain, and it will
    #        be functional only in Digital Closed-loop mode
    #          .The method refers to the Object Dictionary: 0x300B
    #@param  speed_controller_ki  a float value between 0 to 300      
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]   
    def set_speed_controller_ki(self, speed_controller_ki: float) -> list:
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

    ##
    #@brief  This commands sets the direction of the rotation of the motor
    #        either to ClockWise rotation or to Counter Clockwise Rotation
    #          .The method refers to the Object Dictionary: 0x300C
    #@param  motor_direction  enum that specify the direction of the rotation of the motor    
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_motor_direction(self, motor_direction: DIRECTION) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_motor_direction_input_validation(motor_direction)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error

        if (type(motor_direction) is int):
            motor_direction = DIRECTION(motor_direction)

        motor_direction = motor_direction.value
        informationToSend = convert_to_data(motor_direction, DATA_TYPE.UINT32)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_MotorDirection, informationToSend)
        return result, error

    ##
    #@brief  This command sets the amount of the Phase or Armature resistance
    #        for 3-phase or DC Brushed motors respectively
    #          .The method refers to the Object Dictionary: 0x300D
    #@param  motor_resistance  a float value [Ohm]    
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_motor_resistance(self, motor_resistance: float) -> list:
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

    ##
    #@brief  This command sets the amount of the Phase or Armature Inductance
    #        for 3-phase or DC Brushed motors respectively
    #          .The method refers to the Object Dictionary: 0x300E
    #@param  motor_inductance  a float value [Henry]   
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_motor_inductance(self, motor_inductance: float) -> list:
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

    ##
    #@brief  This command sets the number of the Poles of a 3-phase motor commissioned with SOLO
    #          .The method refers to the Object Dictionary: 0x300F
    #@param  motor_poles_counts  a long value between 1 to 254     
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_motor_poles_counts(self, motor_poles_counts: int) -> list:
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

    ##
    #@brief  This command sets the pre-quad number of physical lines of an 
    #        incremental encoder engraved on its disk
    #          .The method refers to the Object Dictionary: 0x3010
    #@param  incremental_encoder_lines  a long value [pre-quad]    
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_incremental_encoder_lines(self, incremental_encoder_lines: int) -> list:
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

    ##
    #@brief  This command sets the allowed speed during trajectory following
    #        in closed-loop position controlling mode
    #          .The method refers to the Object Dictionary: 0x3011
    #@param  speed_limit  a long value [RPM]     
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_speed_limit(self, speed_limit: int) -> list:
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

    ##
    #@brief  This command sets the type of the feedback control SOLO has to operate
    #          .The method refers to the Object Dictionary: 0x3013
    #@param  mode  enum that specify the type of the feedback control SOLO   
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_feedback_control_mode(self, mode: FEEDBACK_CONTROL_MODE) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_feedback_control_mode_input_validation(mode)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error

        if (type(mode) is int):
            mode = FEEDBACK_CONTROL_MODE(mode)

        mode = mode.value
        informationToSend = convert_to_data(mode, DATA_TYPE.UINT32)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_FeedbackControlMode, informationToSend)
        return result, error

    ##
    #@brief  This command resets SOLO to its factory setting to all the default parameters 
    #          .The method refers to the Object Dictionary: 0x3014  
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def reset_factory(self) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        informationToSend = [0x00, 0x00, 0x00, 0x01]
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_ResetFactory, informationToSend)
        return result, error

    ##
    #@brief  This command sets the Motor type that is connected to SOLO in Digital Mode
    #          .The method refers to the Object Dictionary: 0x3015
    #@param  motor_type  enum that specify the Motor type that is connected to SOLO in Digital Mode 
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_motor_type(self, motor_type: MOTOR_TYPE) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_motor_type_input_validation(motor_type)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error

        if (type(motor_type) is int):
            motor_type = MOTOR_TYPE(motor_type)

        motor_type = motor_type.value
        informationToSend = convert_to_data(motor_type, DATA_TYPE.UINT32)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_MotorType, informationToSend)
        return result, error

    ##
    #@brief  This command sets the Control Mode in terms of Torque,
    #        Speed or Position only in Digital Mode
    #          .The method refers to the Object Dictionary: 0x3016
    #@param  control_mode  enum that specify the Control Mode in terms of Torque,
    #                      Speed or Position only in Digital Mode  
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_control_mode(self, control_mode: CONTROL_MODE) -> list:
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

    ##
    #@brief  This command sets the value for Current Controller Kp or proportional gain
    #          .The method refers to the Object Dictionary: 0x3017
    #@param  current_controller_kp  a float value between 0 to 16000   
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_current_controller_kp(self, current_controller_kp: float) -> list:
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

    ##
    #@brief  This command sets the value for Current Controller Ki or integral gain
    #          .The method refers to the Object Dictionary: 0x3018
    #@param  current_controller_ki  a float value between 0 to 16000   
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_current_controller_ki(self, current_controller_ki: float) -> list:
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

    ##
    #@brief  depending on the Motor type: in case of BLDC or PMSM motors Sets the Field
    #        Weakening current reference to help the motor reaching speeds higher than
    #        nominal values and in case of AC Induction Motors Sets the desired magnetizing
    #        current (Id) required for controlling ACIM motors in FOC in Amps 
    #          .The method refers to the Object Dictionary: 0x301A
    #@param  magnetizing_current_id_reference  a float value [Amps]    
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_magnetizing_current_id_reference(self, magnetizing_current_id_reference: float) -> list:
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

    ##
    #@brief  This command sets the desired Position reference in terms of quadrature
    #        pulses while SOLO operates with the Incremental Encoders or in terms of
    #        pulses while while SOLO operates with Hall sensors
    #          .The method refers to the Object Dictionary: 0x301B
    #@param  position_reference  a long value [Quad-Pulse]   
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_position_reference(self, position_reference: int) -> list:
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

    ##
    #@brief  This command sets the value for Position Controller Kp or proportional gain 
    #          .The method refers to the Object Dictionary: 0x301C
    #@param  position_controller_kp  a float value between 0 to 16000   
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_position_controller_kp(self, position_controller_kp: float) -> list:
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

    ##
    #@brief  This command sets the value for Position Controller Ki or integrator gain
    #          .The method refers to the Object Dictionary: 0x301D
    #@param  position_controller_ki  a float value between 0 to 16000     
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_position_controller_ki(self, position_controller_ki: float) -> list:
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
        
    ##
    #@brief  This command resets the position counter back to zero
    #          .The method refers to the Object Dictionary: 0x301F     
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def reset_position_to_zero(self) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_ResetPositionToZero, informationToSend)
        return result, error

    ##
    #@brief  This command overwrites the reported errors in Error Register
    #        reported with command code of "0xA1"   
    #          .The method refers to the Object Dictionary: 0x3020
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def overwrite_error_register(self) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_OverwriteErrorRegister, informationToSend)
        return result, error

    ##
    #@brief  This command sets the observer gain for the Non-linear observer
    #        that estimates the speed and angle of a BLDC or PMSM once the 
    #        motor type is selected as normal BLDC-PMSM
    #          .The method refers to the Object Dictionary: 0x3021
    #@param  observer_gain  a float value between 0.01 to 1000    
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_observer_gain_bldc_pmsm(self, observer_gain: float) -> list:
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

    ##
    #@brief  This command sets the observer gain for the Non-linear observer that
    #        estimates the speed and angle of a BLDC or PMSM once the motor type
    #        is selected as ultra-fast BLDC-PMSM
    #          .The method refers to the Object Dictionary: 0x3022
    #@param  observer_gain  a float value between 0.01 to 1000    
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_observer_gain_bldc_pmsm_ultrafast(self, observer_gain: float) -> list:
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

    ##
    #@brief  This command sets the observer gain for the Non-linear observer
    #        that estimates the speed of a DC brushed once the motor type 
    #        is selected as DC brushed
    #          .The method refers to the Object Dictionary: 0x3023
    #@param  observer_gain  a float value between 0.01 to 1000    
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_observer_gain_dc(self, observer_gain: float) -> list:
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

    ##
    #@brief  This command sets how fast the observer should operate once
    #        SOLO is in sensorless mode with normal BLDC-PMSM selected as the Motor type
    #          .The method refers to the Object Dictionary: 0x3024
    #@param  filter_gain  a float value between 0.01 to 16000   
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_filter_gain_bldc_pmsm(self, filter_gain: float) -> list:
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

    ##
    #@brief  This command sets how fast the observer should operate once SOLO
    #           is in sensorless mode with ultra-fast BLDC-PMSM selected as the Motor type
    #           .The method refers to the Object Dictionary: 0x3025
    #@param  filterGain  a float value between 0.01 to 16000   
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_filter_gain_bldc_pmsm_ultrafast(self, filter_gain: float) -> list:
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

    ##
    #@brief  This command sets the baud-rate of the UART line
    #          .The method refers to the Object Dictionary: 0x3026
    #@param  baudrate  enum that specify the baud-rate of the UART line     
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_uart_baudrate(self, baudrate: UART_BAUD_RATE) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_uart_baudrate_input_validation(baudrate)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error

        if (type(baudrate) is int):
            baudrate = UART_BAUD_RATE(baudrate)
            
        baudrate = baudrate.value
        informationToSend = convert_to_data(baudrate, DATA_TYPE.UINT32)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_UartBaudrate, informationToSend)
        return result, error 

    ##
    #@brief  This command starts or stops the process of sensor calibration
    #          .The method refers to the Object Dictionary: 0x3027
    #@param  calibration_action  enum that specify the process of sensor calibration   
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def sensor_calibration(self, calibration_action: POSITION_SENSOR_CALIBRATION_ACTION) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = sensor_calibration_input_validation(calibration_action)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error

        if (type(calibration_action) is int):
            calibration_action = POSITION_SENSOR_CALIBRATION_ACTION(calibration_action)

        calibration_action = calibration_action.value
        informationToSend = convert_to_data(
            calibration_action, DATA_TYPE.UINT32)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_SensorCalibration, informationToSend)
        return result, error 

    ##
    #@brief  This command sets the per-unit offset identified after sensor calibration
    #        for Encoder or Hall sensors in C.C.W direction
    #          .The method refers to the Object Dictionary: 0x3028
    #@param  encoder_hall_offset  a float value between 0.0 to 1.0   
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_encoder_hall_ccw_offset(self, encoder_hall_offset: float) -> list:
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

    ##
    #@brief  This command sets the per-unit offset identified after sensor calibration
    #        for Encoder or Hall sensors in C.W direction
    #          .The method refers to the Object Dictionary: 0x3029
    #@param  encoder_hall_offset  a float value between 0.0 to 1.0     
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_encoder_hall_cw_offset(self, encoder_hall_offset: float) -> list:
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

    ##
    #@brief  This command defines the acceleration value of the Speed for speed controller
    #        both in Analogue and Digital modes in Revolution per square seconds
    #          .The method refers to the Object Dictionary: 0x302A
    #@param  speed_acceleration_value  a float value [Rev/S^2]  
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_speed_acceleration_value(self, speed_acceleration_value: float) -> list:
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

    ##
    #@brief  This command defines the deceleration value of the Speed for speed controller
    #        both in Analogue and Digital modes in Revolution per square seconds
    #          .The method refers to the Object Dictionary: 0x302B
    #@param  speed_deceleration_value  a float value [Rev/S^2]     
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_speed_deceleration_value(self, speed_deceleration_value: float) -> list:
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

    ##
    #@brief  This command sets the baud rate of CAN bus in CANOpen network
    #          .The method refers to the Object Dictionary: 0x302C
    #@param  canbus_baudrate  enum that specify the baud rate of CAN bus in CANOpen network    
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_can_bus_baudrate(self, canbus_baudrate: CAN_BUS_BAUD_RATE) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_can_bus_baudrate_input_validation(canbus_baudrate)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error

        if (type(canbus_baudrate) is int):
            canbus_baudrate = CAN_BUS_BAUD_RATE(canbus_baudrate)
        
        canbus_baudrate = canbus_baudrate.value
        informationToSend = convert_to_data(canbus_baudrate, DATA_TYPE.UINT32)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_CanbusBaudrate, informationToSend)
        return result, error
        
    ##
    #@brief  This command defines the resolution of the speed at S/T input
    #          while SOLO operates in Analogue mode
    #           .The method refers to the Object Dictionary: 0x303E
    #@param  division_coefficient  a long value     
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_analogue_speed_resolution_division_coefficient(self, division_coefficient: float) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_analogue_speed_resolution_division_coefficient_input_validation(division_coefficient)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error

        division_coefficient = float(division_coefficient)
        
        informationToSend = convert_to_data(division_coefficient, DATA_TYPE.SFXT)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_ASRDC, informationToSend)
        return result, error

    ##
    #@brief  This command defines the type of the Motion Profile that is 
    #          being used in Speed or Position Modes
    #           .The method refers to the Object Dictionary: 0x3040
    #@param  motion_profile_mode enum that specify the type of the Motion Profile    
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_motion_profile_mode(self, motion_profile_mode: MOTION_PROFILE_MODE) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_motion_profile_mode_input_validation(motion_profile_mode)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error

        if (type(motion_profile_mode) is int):
            motion_profile_mode = MOTION_PROFILE_MODE(motion_profile_mode)
        
        motion_profile_mode = motion_profile_mode.value
        
        informationToSend = convert_to_data(motion_profile_mode, DATA_TYPE.UINT32)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_MotionProfileMode, informationToSend)
        return result, error
        
    ##
    #@brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles 
    #           .The method refers to the Object Dictionary: 0x3041
    #@param  motion_profile_variable1 a long value     
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_motion_profile_variable1(self, motion_profile_variable1: float) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_motion_profile_variable1_input_validation(motion_profile_variable1)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error

        motion_profile_variable1 = float(motion_profile_variable1)
        
        informationToSend = convert_to_data(motion_profile_variable1, DATA_TYPE.SFXT)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_MotionProfileVariable1, informationToSend)
        return result, error
        
    ##
    #@brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
    #           .The method refers to the Object Dictionary: 0x3042
    #@param  motion_profile_variable2 a long value     
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_motion_profile_variable2(self, motion_profile_variable2: float) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_motion_profile_variable2_input_validation(motion_profile_variable2)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error

        motion_profile_variable2 = float(motion_profile_variable2)
        
        informationToSend = convert_to_data(motion_profile_variable2, DATA_TYPE.SFXT)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_MotionProfileVariable2, informationToSend)
        return result, error
        
    ##
    #@brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
    #          .The method refers to the Object Dictionary: 0x3043
    #@param  motion_profile_variable3 a long value     
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_motion_profile_variable3(self, motion_profile_variable3: float) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_motion_profile_variable3_input_validation(motion_profile_variable3)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error

        motion_profile_variable3 = float(motion_profile_variable3)
        
        informationToSend = convert_to_data(motion_profile_variable3, DATA_TYPE.SFXT)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_MotionProfileVariable3, informationToSend)
        return result, error
        
    ##
    #@brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
    #           .The method refers to the Object Dictionary: 0x3044
    #@param  motion_profile_variable4 a long value     
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_motion_profile_variable4(self, motion_profile_variable4: float) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_motion_profile_variable4_input_validation(motion_profile_variable4)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error

        motion_profile_variable4 = float(motion_profile_variable4)
        
        informationToSend = convert_to_data(motion_profile_variable4, DATA_TYPE.SFXT)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_MotionProfileVariable4, informationToSend)
        return result, error
        
    ##
    #@brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
    #           .The method refers to the Object Dictionary: 0x3045
    #@param  motion_profile_variable5 a long value     
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_motion_profile_variable5(self, motion_profile_variable5: float) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_motion_profile_variable5_input_validation(motion_profile_variable5)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error

        motion_profile_variable5 = float(motion_profile_variable5)
        
        informationToSend = convert_to_data(motion_profile_variable5, DATA_TYPE.SFXT)
        result, error = self.CANOpenTransmit(
            self._address, ConstantCanopen.Object_MotionProfileVariable5, informationToSend)
        return result, error
# ----------------------------------------------
# ---------------------Read---------------------
# ----------------------------------------------

    def get_read_error_register(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_ReadErrorRegister, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.UINT32), error
        return -1, error

    def get_guard_time(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_GuardTime, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.UINT32), error
        return -1, error

    def get_life_time_factor(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_LifeTimeFactor, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.UINT32), error
        return -1, error

    def get_producer_heartbeat_time(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_ProducerHeartbeatTime, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.UINT32), error
        return -1, error

    ##
    #@brief  This command reads the device address connected on the line 
    #          .The method refers to the Object Dictionary: 0x3001   
    #@retval List of [long device address connected on the line, ERROR class/enumeration]
    def get_device_address(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_SetDeviceAddress, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.UINT32), error
        return -1, error

    ##
    #@brief  This command reads the phase-A voltage of the motor connected to the
    #        "A" pin output of SOLO for 3-phase Motors   
    #          .The method refers to the Object Dictionary: 0x302D
    #@retval List of [float phase-A voltage of the motor [Volts], ERROR class/enumeration]
    def get_phase_a_voltage(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_PhaseAVoltage, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the phase-B voltage of the motor connected to the
    #        "B" pin output of SOLO for 3-phase Motors  
    #          .The method refers to the Object Dictionary: 0x302E
    #@retval List of [float 0 phase-A voltage of the motor [Volts], ERROR class/enumeration]
    def get_phase_b_voltage(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_PhaseBVoltage, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the phase-A current of the motor connected to the
    #        "A" pin output of SOLO for 3-phase Motors 
    #          .The method refers to the Object Dictionary: 0x302F
    #@retval List of [float phase-A current of the motor [Amps], ERROR class/enumeration]
    def get_phase_a_current(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_PhaseACurrent, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the phase-B current of the motor connected to the
    #        "B" pin output of SOLO for 3-phase Motors  
    #          .The method refers to the Object Dictionary: 0x3030
    #@retval List of [float phase-B current of the motor [Amps], ERROR class/enumeration]
    def get_phase_b_current(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_PhaseBCurrent, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the input BUS voltage   
    #          .The method refers to the Object Dictionary: 0x3031 
    #@retval List of [float  BUS voltage [Volts], ERROR class/enumeration]
    def get_bus_voltage(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_BusVoltage, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the current inside the DC brushed motor connected to
    #        "B" and "C" outputs of SOLO  
    #          .The method refers to the Object Dictionary: 0x3032
    #@retval List of [float between [Amps], ERROR class/enumeration]
    def get_dc_motor_current_im(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_DcMotorCurrentIm, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the voltage of the DC brushed motor connected to
    #        "B" and "C" outputs of SOLO  
    #          .The method refers to the Object Dictionary: 0x3033
    #@retval List of [float [Volts], ERROR class/enumeration]
    def get_dc_motor_voltage_vm(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_DcMotorVoltageVm, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the value of the Speed controller Kp gain, 
    #        set for Digital mode operations   
    #          .The method refers to the Object Dictionary: 0x300A
    #@retval List of [float between 0 to 16000, ERROR class/enumeration]
    def get_speed_controller_kp(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_SpeedControllerKp, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the value of the Speed controller Ki gain,
    #        set for Digital mode operations   
    #          .The method refers to the Object Dictionary: 0x300B
    #@retval List of [float between 0 to 16000, ERROR class/enumeration]
    def get_speed_controller_ki(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_SpeedControllerKi, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the output switching frequency of SOLO in Hertz 
    #          .The method refers to the Object Dictionary: 0x3009
    #@retval List of [long [Hz], ERROR class/enumeration]
    def get_output_pwm_frequency_khz(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_OutputPwmFrequencyKhz, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return round(convert_from_data(informationReceived, DATA_TYPE.UINT32)/ 1000,0), error
        return -1, error

    ##
    #@brief  This command reads the value of the current limit set for SOLO in
    #        closed-loop digital operation mode   
    #          .The method refers to the Object Dictionary: 0x3003
    #@retval List of [float [Amps], ERROR class/enumeration]
    def get_current_limit(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_CurrentLimit, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the actual monetary value of “Iq” that is
    #        the current acts in torque generation in FOC mode for 3-phase motors 
    #          .The method refers to the Object Dictionary: 0x3034 
    #@retval List of [float [Amps], ERROR class/enumeration]
    def get_quadrature_current_iq_feedback(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_QuadratureCurrentIqFeedback, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the actual monetary value of Id that is the
    #        direct current acting in FOC  
    #          .The method refers to the Object Dictionary: 0x3035
    #@retval List of [float [Amps], ERROR class/enumeration]
    def get_magnetizing_current_id_feedback(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_MagnetizingCurrentIdFeedback, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the number of Poles set for 3-phase motors 
    #          .The method refers to the Object Dictionary: 0x300F 
    #@retval List of [long between 1 to 254, ERROR class/enumeration]
    def get_motor_poles_counts(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_MotorPolesCounts, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.UINT32), error
        return -1, error

    ##
    #@brief  This command reads the number of physical Incremental encoder lines set on SOLO   
    #          .The method refers to the Object Dictionary: 0x3010
    #@retval List of [long between 1 to 200000, ERROR class/enumeration]
    def get_incremental_encoder_lines(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_IncrementalEncoderLines, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.UINT32), error
        return -1, error

    ##
    #@brief  This command reads the amount of value set for Current controller
    #        Kp or proportional gain  
    #          .The method refers to the Object Dictionary: 0x3017
    #@retval List of [float between 0 to 16000, ERROR class/enumeration]
    def get_current_controller_kp(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_CurrentControllerKp, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the amount of value set for Current controller
    #        Ki or integrator gain    
    #          .The method refers to the Object Dictionary: 0x3018
    #@retval List of [float between 0 to 16000, ERROR class/enumeration]
    def get_current_controller_ki(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_CurrentControllerKi, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the momentary temperature of the board in centigrade  
    #          .The method refers to the Object Dictionary: 0x3039
    #@retval List of [float [°C], ERROR class/enumeration]
    def get_board_temperature(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_BoardTemperature, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the Phase or Armature resistance of
    #        the 3-phase or DC brushed motor connected to SOLO respectively  
    #          .The method refers to the Object Dictionary: 0x300D
    #@retval List of [float [Ohms], ERROR class/enumeration]
    def get_motor_resistance(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_MotorResistance, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the Phase or Armature Inductance of 
    #        the 3-phase or DC brushed motor connected to SOLO respectively 
    #          .The method refers to the Object Dictionary: 0x300E  
    #@retval List of [float [Henry], ERROR class/enumeration]
    def get_motor_inductance(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_MotorInductance, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    ##
    #@brief  his command reads the actual speed of the motor measured or estimated by SOLO in
    #        sensorless or sensor-based modes respectively   
    #          .The method refers to the Object Dictionary: 0x3036
    #@retval List of [long [RPM], ERROR class/enumeration]
    def get_speed_feedback(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_SpeedFeedback, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.INT32), error
        return -1, error

    ##
    #@brief  This command reads the Motor type selected for Digital or Analogue mode operations 
    #          .The method refers to the Object Dictionary: 0x3015  
    #@retval List of [long between 0 to 3, ERROR class/enumeration]
    def get_motor_type(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_MotorType, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return MOTOR_TYPE(convert_from_data(informationReceived, DATA_TYPE.UINT32)), error
        return -1, error

    ##
    #@brief  This command reads the feedback control mode selected on SOLO both
    #        for Analogue and Digital operations    
    #          .The method refers to the Object Dictionary: 0x3013
    #@retval List of [long between 0 to 2, ERROR class/enumeration]
    def get_feedback_control_mode(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_FeedbackControlMode, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return FEEDBACK_CONTROL_MODE(convert_from_data(informationReceived, DATA_TYPE.UINT32)), error
        return -1, error

    ##
    #@brief  This command reads the actual commanding mode that SOLO is operating 
    #          .The method refers to the Object Dictionary: 0x3002
    #@retval List of [long between 0 or 1, ERROR class/enumeration]
    def get_command_mode(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_CommandMode, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return COMMAND_MODE(convert_from_data(informationReceived, DATA_TYPE.UINT32)), error
        return -1, error

    ##
    #@brief  This command reads the Control Mode type in terms of Torque,
    #        Speed or Position in both Digital and Analogue modes  
    #          .The method refers to the Object Dictionary: 0x3013
    #@retval List of [long between 0 to 2, ERROR class/enumeration]
    def get_control_mode(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_ControlMode, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return CONTROL_MODE(convert_from_data(informationReceived, DATA_TYPE.UINT32)), error
        return -1, error

    ##
    #@brief  This command reads the value of the speed limit set on SOLO  
    #          .The method refers to the Object Dictionary: 0x3011
    #@retval List of [long [RPM], ERROR class/enumeration]
    def get_speed_limit(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_SpeedLimit, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.UINT32), error
        return -1, error

    ##
    #@brief  This command reads the amount of value set for Position
    #        controller Kp or proportional gain   
    #          .The method refers to the Object Dictionary: 0x301C
    #@retval List of [float between 0 to 16000, ERROR class/enumeration]
    def get_position_controller_kp(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_PositionControllerKp, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the amount of value set for Position
    #        controller Ki or integrator gain   
    #          .The method refers to the Object Dictionary: 0x301D
    #@retval List of [float between 0 to 16000, ERROR class/enumeration]
    def get_position_controller_ki(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_PositionControllerKi, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the number of counted pulses from the
    #        Incremental Encoder or Hall sensors  
    #          .The method refers to the Object Dictionary: 0x3037
    #@retval List of [long [Quad-Pulses], ERROR class/enumeration]
    def get_position_counts_feedback(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_PositionCountsFeedback, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.INT32), error
        return -1, error

    ##
    #@brief  This command reads the error register which is a 32 bit register with
    #        each bit corresponding to specific errors   
    #          .The method refers to the Object Dictionary: 0x3020
    #@retval List of [long , ERROR class/enumeration]
    def get_error_register(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_OverwriteErrorRegister, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.UINT32), error
        return -1, error

    ##
    #@brief  This command reads the Firmware version existing currently on the SOLO unit   
    #          .The method refers to the Object Dictionary: 0x303A
    #@retval List of [long, ERROR class/enumeration]
    def get_device_firmware_version(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_DeviceFirmwareVersion, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.UINT32), error
        return -1, error

    ##
    #@brief  This command reads the Hardware version of the SOLO unit connected    
    #          .The method refers to the Object Dictionary: 0x303B 
    #@retval List of [long, ERROR class/enumeration]
    def get_device_hardware_version(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_DeviceHardwareVersion, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.UINT32), error
        return -1, error

    ##
    #@brief  This command reads the amount of desired Torque reference (Iq or IM)
    #        already set for the Motor to follow in Digital Closed-loop Torque control mode  
    #          .The method refers to the Object Dictionary: 0x3004 
    #@retval List of [float [Amps], ERROR class/enumeration]
    def get_torque_reference_iq(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_TorqueReferenceIq, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the amount of desired Speed reference already set for
    #        the Motor to follow in Digital Closed-loop Speed control mode   
    #          .The method refers to the Object Dictionary: 0x3005
    #@retval List of [long [RPM], ERROR class/enumeration]
    def get_speed_reference(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_SpeedReference, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.UINT32), error
        return -1, error

    ##
    #@brief  This command reads the amount of desired Id (direct current) or
    #        Magnetizing current reference already set for the Motor to follow
    #        in Digital Closed-loop Speed control mode for ACIM motors  
    #          .The method refers to the Object Dictionary: 0x301A
    #@retval List of [float [Amps], ERROR class/enumeration]
    def get_magnetizing_current_id_reference(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_MagnetizingCurrentIdReference, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the desired position reference set for the Motor
    #        to follow in Digital Closed-loop Position mode in terms of quadrature pulses 
    #          .The method refers to the Object Dictionary: 0x301B 
    #@retval List of [long [Quad-Pulses], ERROR class/enumeration]
    def get_position_reference(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_PositionReference, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.INT32), error
        return -1, error

    ##
    #@brief  This command reads the desired Power reference for SOLO to apply in 
    #        Digital Open-loop speed control mode for 3-phase motors in terms of percentage
    #          .The method refers to the Object Dictionary: 0x3006
    #@retval List of [float [%], ERROR class/enumeration]
    def get_power_reference(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_PowerReference, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    ##
    #@brief  This commands reads the desired direction of rotation set for the Motor   
    #          .The method refers to the Object Dictionary: 0x300C
    #@retval List of [long 0 Counter ClockWise / 1 ClockWise, ERROR class/enumeration]
    def get_motor_direction(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_MotorDirection, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return DIRECTION(convert_from_data(informationReceived, DATA_TYPE.UINT32)), error
        return -1, error

    ##
    #@brief  This command reads the value of Sensorless Observer Gain for Normal BLDC-PMSM Motors  
    #          .The method refers to the Object Dictionary: 0x3021
    #@retval List of [float between 0.01 to 1000, ERROR class/enumeration]
    def get_observer_gain_bldc_pmsm(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_ObserverGainBldcPmsm, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the value of Sensorless Observer Gain for Normal BLDC-PMSM Motors  
    #          .The method refers to the Object Dictionary: 0x3022
    #@retval List of [float between 0.01 to 1000, ERROR class/enumeration]
    def get_observer_gain_bldc_pmsm_ultrafast(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_ObserverGainBldcPmsmUltrafast, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the value of Sensorless Observer Gain for DC Motor  
    #          .The method refers to the Object Dictionary: 0x3023
    #@retval List of [float between 0.01 to 1000, ERROR class/enumeration]
    def get_observer_gain_dc(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_ObserverGainDc, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the value of Sensorless Observer
    #        Filter Gain for Normal BLDC-PMSM Motors    
    #          .The method refers to the Object Dictionary: 0x3024
    #@retval List of [float between 0.01 to 16000, ERROR class/enumeration]
    def get_filter_gain_bldc_pmsm(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_FilterGainBldcPmsm, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the value of Sensorless Observer
    #        Filter Gain for Ultra Fast BLDC-PMSM Motors  
    #          .The method refers to the Object Dictionary: 0x3025
    #@retval List of [float between 0.01 to 16000, ERROR class/enumeration]
    def get_filter_gain_bldc_pmsm_ultrafast(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_FilterGainBldcPmsmUltrafast, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the measured or estimated per-unit angle of the 3-phase motors   
    #          .The method refers to the Object Dictionary: 0x3038
    #@retval List of [float [Per Unit], ERROR class/enumeration]
    def get_3phase_motor_angle(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_3PhaseMotorAngle, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the per-unit Encoder or Hall sensor offset in C.C.W direction    
    #          .The method refers to the Object Dictionary: 0x3028
    #@retval List of [float [Per Unit], ERROR class/enumeration]
    def get_encoder_hall_ccw_offset(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_EncoderHallCcwOffset, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the per-unit Encoder or Hall sensor offset in C.C.W direction 
    #          .The method refers to the Object Dictionary: 0x3029  
    #@retval List of [float [Per Unit], ERROR class/enumeration]
    def get_encoder_hall_cw_offset(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_EncoderHallCwOffset, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads Baud Rate selected on SOLO unit to communicate through UART line   
    #          .The method refers to the Object Dictionary: 0x3026
    #@retval List of [long [Bits/s], ERROR class/enumeration]
    def get_uart_baudrate(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_UartBaudrate, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return UART_BAUD_RATE(convert_from_data(informationReceived, DATA_TYPE.UINT32)), error
        return -1, error

    ##
    #@brief  This command reads the acceleration value of the Speed for
    #        speed controller both in Analogue and Digital modes
    #        in Revolution per square seconds  
    #          .The method refers to the Object Dictionary: 0x302A
    #@retval List of [float [Rev/S^2], ERROR class/enumeration]
    def get_speed_acceleration_value(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_SpeedAccelerationValue, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the deceleration value of the Speed for
    #        speed controller both in Analogue and Digital modes
    #        in Revolution per square seconds  
    #          .The method refers to the Object Dictionary: 0x302B
    #@retval List of [float [Rev/S^2], ERROR class/enumeration]
    def get_speed_deceleration_value(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_SpeedDecelerationValue, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error
        
    ##
    #@brief  This command reads the Analogue Speed Resolution Division Coefficient (ASRDC)
    #           .The method refers to the Object Dictionary: 0x303E
    #          while SOLO operates in Analogue mode     
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def get_analogue_speed_resolution_division_coefficient(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_ASRDC, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1, error

    ##
    #@brief  This command test if the communication is working 
    #@retval  List of [ bool 0 not working / 1 for working, ERROR class/enumeration]
    def communication_is_working(self) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        temperature, error = self.get_board_temperature()
        time.sleep(0.2)
        temperature, error = self.get_board_temperature()
        if (error == ERROR.NO_ERROR_DETECTED):
            return True, error
        return False, error

    ##
    #@brief  This Command reads the number of counted index pulses 
    #        seen on the Incremental Encoder’s output  
    #          .The method refers to the Object Dictionary: 0x303D 
    #@retval List of [long [Pulses], ERROR class/enumeration]
    def get_encoder_index_counts(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_EncoderIndexCounts, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.UINT32), error
        return -1, error

    ##
    #@brief  This command reads the type of the Embedded Motion profile active in the controller 
    #          being used in Speed or Position Modes    
    #           .The method refers to the Object Dictionary: 0x303F
    #@retval List of [Motion profile , ERROR class/enumeration]
    def get_motion_profile_mode(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_MotionProfileMode, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return MOTION_PROFILE_MODE(convert_from_data(informationReceived, DATA_TYPE.UINT32)), error
        return -1, error
        
    ##
    #@brief  This command reads the value of the Motion Profile Variable1 set inside the controller  
    #           .The method refers to the Object Dictionary: 0x3040
    #@retval List of [Motion Profile Variable1, ERROR class/enumeration]
    def get_motion_profile_variable1(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_MotionProfileVariable1, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error
        
    ##
    #@brief  This command reads the value of the Motion Profile Variable2 set inside the controller 
    #           .The method refers to the Object Dictionary: 0x3041
    #@retval List of [Motion Profile Variable2, ERROR class/enumeration]
    def get_motion_profile_variable2(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_MotionProfileVariable2, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error
        
    ##
    #@brief  This command reads the value of the Motion Profile Variable3 set inside the controller
    #           .The method refers to the Object Dictionary: 0x3042
    #@retval List of [Motion Profile Variable3, ERROR class/enumeration]
    def get_motion_profile_variable3(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_MotionProfileVariable3, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error
        
    ##
    #@brief  This command reads the value of the Motion Profile Variable4 set inside the controller 
    #           .The method refers to the Object Dictionary: 0x3043
    #@retval List of [Motion Profile Variable4, ERROR class/enumeration]
    def get_motion_profile_variable4(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_MotionProfileVariable4, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error
        
    ##
    #@brief  This command reads the value of the Motion Profile Variable5 set inside the controller 
    #           .The method refers to the Object Dictionary: 0x3044
    #@retval List of [Motion Profile Variable5, ERROR class/enumeration]
    def get_motion_profile_variable5(self) -> list:
        informationToSend = [0x00, 0x00, 0x00, 0x00]
        informationReceived = []
        error = ERROR.NO_PROCESSED_COMMAND
        result, error, informationReceived = self.CANOpenReceive(
            self._address, ConstantCanopen.Object_MotionProfileVariable5, informationToSend)
        if (error == ERROR.NO_ERROR_DETECTED) and (result is True):
            return convert_from_data(informationReceived, DATA_TYPE.SFXT), error
        return -1.0, error
