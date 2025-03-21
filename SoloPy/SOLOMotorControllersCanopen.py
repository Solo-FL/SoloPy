## @package SOLOMotorControllersCanopen.py
#  @author  SOLOMotorControllers
#  @brief   This file contains all the functions for the Solo Canopen Drivers
#           Availability: https://github.com/Solo-FL/SoloPy/tree/main/SoloPy
#  @date    Date: 2025
#  @version 4.0.0

## @attention
# Copyright: (c) 2021-present, SOLO motor controllers project
# MIT License (see LICENSE file for more details)

import logging
from importlib import reload
#from interface import implements
import can
import time

from SoloPy.Mcp2515 import *
from SoloPy.Canable import *

from SoloPy import ConstantCanopen
from SoloPy.SOLOMotorControllers import *
from SoloPy.SOLOMotorControllersUtils import *

##
# @brief  Hardware Interface for Canopen Communication
#
class CanCommunicationInterface(Enum):
    ## MCP2515 Interface
    MCP2515          				        = 0
    ## CANABLE Interface
    CANABLE          				        = 1

##
# @brief  Pdo Parameter Name enumeration
#
class PdoParameterName(Enum):
    ## target position [RPDO]
    POSITION_REFERENCE 				        = 0
    ## target velocity [RPDO]
    SPEED_REFERENCE 					        = 1
    ## target torque [RPDO]
    TORQUE_REFERENCE_IQ 				        = 2
    ## target direct current [RPDO]
    MAGNETIZING_CURRENT_ID_REFERENCE 	        = 3
    ## control mode [RPDO]
    CONTROL_MODE 					        = 4
    ## motor direction [RPDO]
    MOTOR_DIRECTION 					        = 5
    ## feedback position [TPDO]
    POSITION_COUNTS_FEEDBACK			        = 6
    ## feedback velocity [TPDO]
    SPEED_FEEDBACK 					        = 7
    ## feedback lq [TPDO]
    QUADRATURE_CURRENT_IQ_FEEDBACK		        = 8
    ## feedback ld [TPDO]
    MAGNETIZING_CURRENT_ID_FEEDBACK 	        = 9
    ## error register [TPDO]
    ERROR_REGISTER 					        = 10
    ## board temperature [TPDO]
    BOARD_TEMPERATURE 				        = 11

##
# @brief a struct that include all the Parameter used during PDO configuration
#
class PdoParameterConfig:
    parameterName           : PdoParameterName
    parameterCobId          : int
    isPdoParameterEnable    : bool
    isRrtParameterEnable    : bool
    syncParameterCount      : int

#class SOLOMotorControllersCanopen(implements(SOLOMotorControllers)):
class SOLOMotorControllersCanopen:
    def __init__(
            self,
            address = 0,
            baudrate = CanBusBaudRate.RATE_1000,
            interface = CanCommunicationInterface.MCP2515,
            bustype = 'socketcan',
            channel = 'can0',
            timeout_count = 2000,
            logger_level = logging.INFO
            ) -> None:
                # logger init
        logging.shutdown()
        reload(logging)
        self._logger = logging.getLogger('SoloPy')
        self._logger.setLevel(logger_level)
        ch = logging.StreamHandler()
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        ch.setFormatter(formatter)
        self._logger.addHandler(ch)
        # logger init end
        self._logger.debug('SoloMotorController INIT')

        if address == 0:  # address 0 is reserved for the host
            address = 1
        self._address = address
        # self._interace = interface
        self._timeout_count = timeout_count
        self.pdoParameterObjectByPdoParameterName = [0] * ConstantCanopen.PDO_PARAMETERNAME_COUNT
        self.pdoParameterCobIdByPdoParameterName = [0] * ConstantCanopen.PDO_PARAMETERNAME_COUNT
        self._interface = None
        self.init_pdo_config()

        if interface == CanCommunicationInterface.MCP2515:
            self._interface = Mcp2515(channel, bustype, baudrate)
        elif interface == CanCommunicationInterface.CANABLE:
            try:
                self._interface = Canable(channel, bustype, baudrate, self._logger)
            except Exception as e:
                self._logger.error("Exception on Canable Init")
                self._logger.error( e, exc_info=True)
                

    def canopen_transmit(self, address: int, _object: int,  sub_index: int, informatrion_to_send: list) -> Tuple[bool, Error]:
        result = None
        error = Error.NO_ERROR_DETECTED

        try:
            result, error = self._interface.canopen_transmit(address, _object,  sub_index, informatrion_to_send)            
        except can.exceptions.CanInitializationError :
            error = Error.CAN_INITIALIZATION_ERROR
            result = False
        except can.exceptions.CanInterfaceNotImplementedError :
            error = Error.CAN_INTERFACE_NOT_IMPLEMENTED_ERROR
            result = False
        except can.exceptions.CanOperationError :
            error = Error.CAN_OPERATION_ERROR
            result = False
        except can.exceptions.CanTimeoutError :
            error = Error.CAN_TIMEOUT_ERROR
            result = False

        return result, error

    def pdo_transmit(self, address: int, _informatrion_to_send: list) -> Tuple[bool, Error]:
        result = False
        error = Error.NO_PROCESSED_COMMAND
        try:
            result, error = self._interface.pdo_transmit(address, _informatrion_to_send)
        except can.exceptions.CanInitializationError :
            error = Error.CAN_INITIALIZATION_ERROR
            result = False
        except can.exceptions.CanInterfaceNotImplementedError :
            error = Error.CAN_INTERFACE_NOT_IMPLEMENTED_ERROR
            result = False
        except can.exceptions.CanOperationError :
            error = Error.CAN_OPERATION_ERROR
            result = False
        except can.exceptions.CanTimeoutError :
            error = Error.CAN_TIMEOUT_ERROR
            result = False

        return result, error


    def pdo_receive(self, address: int) -> Tuple[bool, int, list]:
        result = True
        error = Error.NO_ERROR_DETECTED
        data = list

        try:
            result, error, data = self._interface.pdo_receive(address)
        except can.exceptions.CanInitializationError :
            error = Error.CAN_INITIALIZATION_ERROR
            result = False
        except can.exceptions.CanInterfaceNotImplementedError :
            error = Error.CAN_INTERFACE_NOT_IMPLEMENTED_ERROR
            result = False
        except can.exceptions.CanOperationError :
            error = Error.CAN_OPERATION_ERROR
            result = False
        except  can.exceptions.CanTimeoutError :
            error = Error.CAN_TIMEOUT_ERROR
            result = False

        if result is True:
            dummy = [0, 0, 0, 0]
            data[0:0] = dummy
            return result, error, extract_data(data)
        return result, error, None

    def generic_canbus_write(self, _id: int, _data: list):
        result = None
        error = Error.NO_ERROR_DETECTED

        try:
            bus = can.interface.Bus(
                channel=self._channel, bustype=self._bustype, bitrate= self._baudrate )
            msg = can.Message(
                arbitration_id=_id,
                data = _data,
                is_extended_id=False)

            bus.send(msg)

        except can.exceptions.CanInitializationError :
            error = Error.CAN_INITIALIZATION_ERROR
            result = False
        except can.exceptions.CanInterfaceNotImplementedError :
            error = Error.CAN_INTERFACE_NOT_IMPLEMENTED_ERROR
            result = False
        except can.exceptions.CanOperationError :
            error = Error.CAN_OPERATION_ERROR
            result = False
        except  can.exceptions.CanTimeoutError :
            error = Error.CAN_TIMEOUT_ERROR
            result = False

        return result, error

    def canopen_receive(self, address: int, _object: int, sub_index: int, informatrion_to_send: list) -> Tuple[bool, Error, bytearray]:
        try:
            result = True
            error = Error.NO_ERROR_DETECTED
            information_received = [0, 0, 0, 0, 0, 0, 0, 0]
            result, error, information_received = self._interface.canopen_receive(address, _object,  sub_index, informatrion_to_send)
        except can.exceptions.CanInitializationError :
            error = Error.CAN_INITIALIZATION_ERROR
            result = False
        except can.exceptions.CanInterfaceNotImplementedError :
            error = Error.CAN_INTERFACE_NOT_IMPLEMENTED_ERROR
            result = False
        except can.exceptions.CanOperationError :
            error = Error.CAN_OPERATION_ERROR
            result = False
        except  can.exceptions.CanTimeoutError :
            error = Error.CAN_TIMEOUT_ERROR
            result = False

        if result is True:
            return result, error, extract_data(information_received)

        return result, error, None

    def generic_canbus_read(self) -> list:
        try:
            result = True
            error = Error.NO_ERROR_DETECTED
            bus = can.interface.Bus(
                channel=self._channel, bustype=self._bustype, bitrate=self._baudrate)

            information_received = bus.recv(1)  # Timeout in seconds.
            if not information_received:    # No unit response
                result = False
                error = Error.GENERAL_ERROR

        except can.exceptions.CanInitializationError :
            error = Error.CAN_INITIALIZATION_ERROR
            result = False
        except can.exceptions.CanInterfaceNotImplementedError :
            error = Error.CAN_INTERFACE_NOT_IMPLEMENTED_ERROR
            result = False
        except can.exceptions.CanOperationError :
            error = Error.CAN_OPERATION_ERROR
            result = False
        except  can.exceptions.CanTimeoutError :
            error = Error.CAN_TIMEOUT_ERROR
            result = False

        if result is True:
            return result, error, extract_data(list(information_received.data)),information_received.dlc,information_received.arbitration_id

        return result, error, None

# -----------------------------------------------
# ---------------------Write---------------------
# -----------------------------------------------

    def set_guard_time(self, guard_time: int) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_guard_time_input_validation(guard_time)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        information_to_send = convert_to_data(guard_time, DataType.UINT32)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_GUARDTIME, 0x00, information_to_send)
        return result, error

    def set_life_time_factor(self, life_time_factor: int) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_life_time_factor_input_validation(life_time_factor)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        information_to_send = convert_to_data(life_time_factor, DataType.UINT32)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_LIFETIME_FACTOR, 0x00, information_to_send)
        return result, error

    def set_producer_heartbeat_time(self, producer_heartbeat_time: int) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_producer_heartbeat_time_input_validation(producer_heartbeat_time)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        information_to_send = convert_to_data(
            producer_heartbeat_time, DataType.UINT32)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_PRODUCER_HEARTBEAT_TIME, 0x00, information_to_send)
        return result, error

    ##
    # @brief  This command determine the validity of count of SYNC message
    # @param  parameterName	enum that specifies the name of the PDO parameter that wants to set CobId value
    # @param  parameterCobbId	CobId value
    # @retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    #
    def set_pdo_parameter_cobid_input_validation(self, parameterName: PdoParameterName, parameterCobbId: int) -> Tuple[bool, Error]:
        error = Error.NO_ERROR_DETECTED
        if parameterName.value < PdoParameterName.POSITION_COUNTS_FEEDBACK.value:
            if (parameterCobbId >= ConstantCanopen.RPDO_MIN_COBIB and parameterCobbId <= ConstantCanopen.RPDO_MAX_COBIB):
                return True, error
        else :
            if (parameterCobbId >= ConstantCanopen.TPDO_MIN_COBIB and parameterCobbId <= ConstantCanopen.TPDO_MAX_COBIB):
                return True, error

        error = Error.PDO_PARAMETER_ID_OUT_OF_RANGE
        return False, error

    ##
    # @brief  This command set the intended long value for a PDO command
    # @param  parameterName	enum that specifies the name of the PDO parameter that wants to write its value
    # @param  value	the value that wants to be set for the PDO parameter
    # @param  type	enum DataType that determine type of data
    # @retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    #
    def set_pdo_parameter_value(self, parameterName: PdoParameterName, value: int, _type = DataType.UINT32) -> Tuple[bool, Error]:
        informatrion_to_send = [0x00,0x00,0x00,0x00]
        error = Error.NO_PROCESSED_COMMAND
        if _type == DataType.UINT32:
            informatrion_to_send = convert_to_data(value, DataType.UINT32)
        elif _type == DataType.SFXT:
            informatrion_to_send = convert_to_data(value, DataType.SFXT)

        adr, error = self.get_pdo_parameter_cobid(parameterName)
        if error == Error.PDO_MISSING_COBID:
            return False, error

        return self.pdo_transmit(adr, informatrion_to_send)

    ##
    # @brief  This command send a SYNC message on bus
    # @retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    #
    def send_pdo_sync(self) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND

        try:
            result, error = self._interface.send_pdo_sync()
        except can.exceptions.CanInitializationError :
            error = Error.CAN_INITIALIZATION_ERROR
            result = False
        except can.exceptions.CanInterfaceNotImplementedError :
            error = Error.CAN_INTERFACE_NOT_IMPLEMENTED_ERROR
            result = False
        except can.exceptions.CanOperationError :
            error = Error.CAN_OPERATION_ERROR
            result = False
        except can.exceptions.CanTimeoutError :
            error = Error.CAN_TIMEOUT_ERROR
            result = False

        return result, error

    ##
    # @brief  This command send a RTR for the intended PDO object
    # @param  parameterName	enum that specifies the name of the PDO parameter that wants to send RTR
    # @retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    #
    def send_pdo_rtr(self, parameterName: PdoParameterName) -> Tuple[bool, Error]:
        error = Error.NO_ERROR_DETECTED
        res, error = self.pdo_rtr_valid_parameter(parameterName)
        if error != Error.NO_ERROR_DETECTED:
            return res, error

        adr, error = self.get_pdo_parameter_cobid(parameterName)
        if error != Error.NO_ERROR_DETECTED:
            return False, error

        try:
            result, error = self._interface.send_pdo_rtr(adr)

        except can.exceptions.CanInitializationError :
            error = Error.CAN_INITIALIZATION_ERROR
            result = False
        except can.exceptions.CanInterfaceNotImplementedError :
            error = Error.CAN_INTERFACE_NOT_IMPLEMENTED_ERROR
            result = False
        except can.exceptions.CanOperationError :
            error = Error.CAN_OPERATION_ERROR
            result = False
        except can.exceptions.CanTimeoutError :
            error = Error.CAN_TIMEOUT_ERROR
            result = False

        return result, error

    ##
    # @brief  This command determine the validity of count of SYNC message
    # @param  parameter_count	count of SYNC message
    # @retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    #
    def set_sync_parameter_count_input_validation(self, parameter_count: int) -> Tuple[bool , Error]:
        error = Error.NO_ERROR_DETECTED
        if (parameter_count >= 0 and parameter_count < 12 or parameter_count == 0xFF):
            return True, error
        error = Error.PDO_SYNC_OUT_OF_RANGE
        return False, error

    ##
    # @brief  This command returns the CobId value for the intended PDO parameter name
    # @param  parameterName	enum that specifies the name of the PDO parameter that wants to get parameter CobId
    # @retval List of [long [CobId], Error class/enumeration]
    #
    def get_pdo_parameter_cobid(self, parameterName: PdoParameterName) -> Tuple[int, Error]:
        error = Error.NO_ERROR_DETECTED
        pdoParameterCobId = self.pdoParameterCobIdByPdoParameterName[parameterName.value]
        if pdoParameterCobId == 0:
            error = Error.PDO_MISSING_COBID
        return pdoParameterCobId, error

    ##
    # @brief  This command set PDO configs for the intended PDO object
    # @param  config	enum that specifies PDO parameter configs for the intended PDO object
    # @retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    #
    def set_pdo_parameter_config(self, config: PdoParameterConfig) -> Tuple[bool, Error]:
        informatrion_to_send = [0x00,0x00,0x00,0x00]
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error = self.set_pdo_parameter_cobid_input_validation(config.parameterName, config.parameterCobId)
        if input_validate is False:
            error = Error.PDO_PARAMETER_ID_OUT_OF_RANGE
            return False, error
        input_validate, error = self.set_sync_parameter_count_input_validation(config.syncParameterCount)
        if input_validate is False:
            error = Error.PDO_SYNC_OUT_OF_RANGE
            return False, error

        informatrion_to_send[0] = (config.isPdoParameterEnable << 7) | (config.isRrtParameterEnable << 6)
        informatrion_to_send[1] = 0
        informatrion_to_send[2] = config.parameterCobId >> 8
        informatrion_to_send[3] = config.parameterCobId % 256
        result, error = self.canopen_transmit(self._address, int(self.pdoParameterObjectByPdoParameterName[config.parameterName.value]), 0x01, informatrion_to_send)
        if result is True:
            self.pdoParameterCobIdByPdoParameterName[config.parameterName.value] = config.parameterCobId

            informatrion_to_send[0] = 0
            informatrion_to_send[1] = 0
            informatrion_to_send[2] = 0
            informatrion_to_send[3] = config.syncParameterCount
            if config.syncParameterCount == 0 and self.pdoParameterObjectByPdoParameterName[config.parameterName.value] < self.pdoParameterObjectByPdoParameterName[PdoParameterName.POSITION_COUNTS_FEEDBACK.value]:
                informatrion_to_send[3] = 0xFF

            result, error = self.canopen_transmit(self._address, int(self.pdoParameterObjectByPdoParameterName[config.parameterName.value]), 0x02, informatrion_to_send)
            return result, error
        return False, error

    ##
    # @brief  This command returns the long value of a PDO command
    # @param  parameterName	enum that specifies the name of the parameter that wants to read its value
    # @retval List of [long [PDO Value], Error class/enumeration]
    #
    def get_pdo_parameter_value_long(self, parameterName: PdoParameterName) -> Tuple[int, Error]:
        information_received = [0x00,0x00,0x00,0x00]
        error = Error.NO_PROCESSED_COMMAND

        adr, error = self.get_pdo_parameter_cobid(parameterName)
        if error != Error.NO_ERROR_DETECTED:
            return -1, error

        result, error, information_received = self.pdo_receive(adr)
        if result == 1 and error == Error.NO_ERROR_DETECTED:
            return convert_to_long(information_received), error
        return -1, error

    ##
    # @brief  This command returns the float value of a PDO command
    # @param  parameterName	enum that specifies the name of the PDO parameter that wants to read its value
    # @retval List of [float [PDO Value], Error class/enumeration]
    #
    def get_pdo_parameter_value_float(self, parameterName: PdoParameterName) -> Tuple[float, Error]:
        information_received = [0x00,0x00,0x00,0x00]
        error = Error.NO_ERROR_DETECTED

        adr, error = self.get_pdo_parameter_cobid(parameterName)
        if error != Error.NO_ERROR_DETECTED:
            return False

        result, error, information_received = self.pdo_receive(adr)
        if result == 1 and error == Error.NO_ERROR_DETECTED:
            return convert_to_float(information_received), error
        return -1.0, error

    ##
    # @brief  This command checks the validity of allowing RTR  for the intended PDO parameter
    # @param  parameterName	enum that specifies the name of the PDO parameter that wants to check RTR validity
    # @retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    #
    def pdo_rtr_valid_parameter(self, parameterName: PdoParameterName) -> Tuple[bool, Error]:
        if parameterName.value >= PdoParameterName.POSITION_COUNTS_FEEDBACK.value:
            error = Error.NO_ERROR_DETECTED
            return True, error
        error = Error.PDO_RTR_COMMAND_NOT_ALLOWED
        return False, error

    ##
    # @brief  This command initializes all PDO parameter names addresses in array
    # @retval void
    #
    def init_pdo_config(self):
        i = 0
        for i in range(0, ConstantCanopen.PDO_PARAMETERNAME_COUNT):
            self.pdoParameterCobIdByPdoParameterName[i] = 0

        self.pdoParameterObjectByPdoParameterName[PdoParameterName.POSITION_REFERENCE.value]                = 0x1414
        self.pdoParameterObjectByPdoParameterName[PdoParameterName.SPEED_REFERENCE.value]                   = 0x1415
        self.pdoParameterObjectByPdoParameterName[PdoParameterName.TORQUE_REFERENCE_IQ.value]                = 0x1416
        self.pdoParameterObjectByPdoParameterName[PdoParameterName.MAGNETIZING_CURRENT_ID_REFERENCE.value]    = 0x1417
        self.pdoParameterObjectByPdoParameterName[PdoParameterName.CONTROL_MODE.value]                      = 0x1418
        self.pdoParameterObjectByPdoParameterName[PdoParameterName.MOTOR_DIRECTION.value]                   = 0x1419
        self.pdoParameterObjectByPdoParameterName[PdoParameterName.POSITION_COUNTS_FEEDBACK.value]           = 0x1814
        self.pdoParameterObjectByPdoParameterName[PdoParameterName.SPEED_FEEDBACK.value]                    = 0x1815
        self.pdoParameterObjectByPdoParameterName[PdoParameterName.QUADRATURE_CURRENT_IQ_FEEDBACK.value]      = 0x1816
        self.pdoParameterObjectByPdoParameterName[PdoParameterName.MAGNETIZING_CURRENT_ID_FEEDBACK.value]     = 0x1817
        self.pdoParameterObjectByPdoParameterName[PdoParameterName.ERROR_REGISTER.value]                    = 0x1818
        self.pdoParameterObjectByPdoParameterName[PdoParameterName.BOARD_TEMPERATURE.value]                 = 0x1819

    ##
    #@brief  This command sets the desired device address for a SOLO unit
    #            .The method refers to the Object Dictionary: 0x3001
    #@param  device_address  address want to set for board
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_device_address(self, device_address: int) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_device_address_input_validation(device_address)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        information_to_send = convert_to_data(device_address, DataType.UINT32)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_SET_DEVICE_ADDRESS, 0x00, information_to_send)
        self._address = device_address
        return result, error

    ##
    #@brief  This command sets the mode of the operation of SOLO
    #        in terms of operating in Analogue mode or Digital
    #          .The method refers to the Object Dictionary: 0x3002
    #@param  mode  enum that specify mode of the operation of SOLO
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_command_mode(self, mode: CommandMode) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_command_mode_input_validation(mode)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error

        if isinstance(mode, int):
            mode = CommandMode(mode)

        mode = mode.value
        information_to_send = convert_to_data(mode, DataType.UINT32)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_COMMAND_MODE, 0x00, information_to_send)
        return result, error

    ##
    #@brief  This command defines the maximum allowed current into the motor in terms of Amps
    #           .The method refers to the Object Dictionary: 0x3003
    #@param  current_limit  a float value [Amps]
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_current_limit(self, current_limit: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_current_limit_input_validation(current_limit)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        current_limit = float(current_limit)
        information_to_send = convert_to_data(current_limit, DataType.SFXT)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_CURRENT_LIMIT, 0x00, information_to_send)
        return result, error

    ##
    #@brief  This command sets the amount of desired current that acts in torque generation
    #          .The method refers to the Object Dictionary: 0x3004
    #@param  torque_reference_iq  a float [Amps]
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_torque_reference_iq(self, torque_reference_iq: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_torque_reference_iq_input_validation(torque_reference_iq)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        torque_reference_iq = float(torque_reference_iq)
        information_to_send = convert_to_data(torque_reference_iq, DataType.SFXT)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_TORQUE_REFERENCE_IQ, 0x00, information_to_send)
        return result, error

    ##
    #@brief  This command defines the speed reference for SOLO once it’s in Digital Speed Mode
    #          .The method refers to the Object Dictionary: 0x3005
    #@param  speed_reference  a long value [RPM]
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_speed_reference(self, speed_reference: int) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_speed_reference_input_validation(speed_reference)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        speed_reference = int(speed_reference)
        information_to_send = convert_to_data(speed_reference, DataType.UINT32)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_SPEED_REFERENCE, 0x00, information_to_send)
        return result, error

    ##
    #@brief  This command defines the amount of power percentage during only
    #        Open-loop mode for 3-phase motors
    #          .The method refers to the Object Dictionary: 0x3006
    #@param  power_reference  a float value between 0 to 100
    #@retval bool 0 fail / 1 for succes
    def set_power_reference(self, power_reference: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_power_reference_input_validation(power_reference)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        power_reference = float(power_reference)
        information_to_send = convert_to_data(power_reference, DataType.SFXT)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_POWER_REFERENCE, 0x00, information_to_send)
        return result, error

    ##
    #@brief  By putting 1 in the DATA section of a packet sent with this command, SOLO will start
    #        identifying the electrical parameters of the Motor connected
    #          .The method refers to the Object Dictionary: 0x3007
    #@param  identification  enum that specify Start or Stop of something in SOLO
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def motor_parameters_identification(self, identification: Action) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = motor_parameters_identification_input_validation(identification)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error

        if isinstance(identification, int):
            identification = Action(identification)

        identification = identification.value
        information_to_send = convert_to_data(identification, DataType.UINT32)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_MOTOR_PARAMETERS_IDENTIFICATION, 0x00, information_to_send)
        return result, error

    ##
    #@brief  This command if the DATA is set at zero will stop the whole power and switching system
    #        connected to the motor and it will cut the current floating into the Motor from SOLO
    #          .The method refers to the Object Dictionary: 0x3009
    #@param  action  enum that specify Disable or Enable of something in SOLO
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_drive_disable_enable(self, action : DisableEnable) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = drive_disable_enable_input_validation(action)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error

        if isinstance(action, int):
            action = DisableEnable(action)

        action = action.value
        information_to_send = convert_to_data(action, DataType.UINT32)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_DRIVE_DISABLE_ENABLE, 0x00, information_to_send)
        return result, error

    ##
    #@brief  This command sets the output switching frequency of the whole power unit on the Motor
    #          .The method refers to the Object Dictionary: 0x3009
    #@param  output_pwm_frequency_khz  switching frequencies [kHz]
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_output_pwm_frequency_khz(self, output_pwm_frequency_khz: int) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_output_pwm_frequency_khz_input_validation(output_pwm_frequency_khz)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        output_pwm_frequency_khz = int(output_pwm_frequency_khz)
        information_to_send = convert_to_data(
            output_pwm_frequency_khz, DataType.UINT32)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_OUTPUT_PWMFREQUENCY_KHZ, 0x00, information_to_send)
        return result, error

    ##
    #@brief  This command sets the Speed controller Kp Gain, and it will
    #        be functional only in Digital Closed-loop mode
    #          .The method refers to the Object Dictionary: 0x300A
    #@param  speed_controller_kp  a float value between 0 to 300
   #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_speed_controller_kp(self, speed_controller_kp: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_speed_controller_kp_input_validation(speed_controller_kp)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        speed_controller_kp = float(speed_controller_kp)
        information_to_send = convert_to_data(speed_controller_kp, DataType.SFXT)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_SPEED_CONTROLLER_KP, 0x00, information_to_send)
        return result, error

    ##
    #@brief  This command sets the Speed controller Ki gain, and it will
    #        be functional only in Digital Closed-loop mode
    #          .The method refers to the Object Dictionary: 0x300B
    #@param  speed_controller_ki  a float value between 0 to 300
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_speed_controller_ki(self, speed_controller_ki: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_speed_controller_ki_input_validation(speed_controller_ki)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        speed_controller_ki = float(speed_controller_ki)
        information_to_send = convert_to_data(
            speed_controller_ki, DataType.SFXT)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_SPEED_CONTROLLER_KI, 0x00, information_to_send)
        return result, error
    
    ##
    #@brief  This commands sets the direction of the rotation of the motor
    #        either to ClockWise rotation or to Counter Clockwise Rotation
    #          .The method refers to the Object Dictionary: 0x300C
    #@param  motor_direction  enum that specify the direction of the rotation of the motor    
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_motor_direction(self, motor_direction: Direction) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_motor_direction_input_validation(motor_direction)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error

        if isinstance(motor_direction, int):
            motor_direction = Direction(motor_direction)

        motor_direction = motor_direction.value
        information_to_send = convert_to_data(motor_direction, DataType.UINT32)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_MOTOR_DIRECTION, 0x00, information_to_send)
        return result, error

    ##
    #@brief  This command sets the amount of the Phase or Armature resistance
    #        for 3-phase or DC Brushed motors respectively
    #          .The method refers to the Object Dictionary: 0x300D
    #@param  motor_resistance  a float value between 0.001 t0 100.0 [Ohm]
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_motor_resistance(self, motor_resistance: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_motor_resistance_input_validation(motor_resistance)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        motor_resistance = float(motor_resistance)
        information_to_send = convert_to_data(motor_resistance, DataType.SFXT)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_MOTOR_RESISTANCE, 0x00, information_to_send)
        return result, error

    ##
    #@brief  This command sets the amount of the Phase or Armature Inductance
    #        for 3-phase or DC Brushed motors respectively
    #          .The method refers to the Object Dictionary: 0x300E
    #@param  motor_inductance  a float value between 0.0 t0 0.2 [Henry]
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_motor_inductance(self, motor_inductance: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_motor_inductance_input_validation(motor_inductance)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        motor_inductance = float(motor_inductance)
        information_to_send = convert_to_data(motor_inductance, DataType.SFXT)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_MOTOR_INDUCTANCE, 0x00, information_to_send)
        return result, error

    ##
    #@brief  This command sets the number of the Poles of a 3-phase motor commissioned with SOLO
    #          .The method refers to the Object Dictionary: 0x300F
    #@param  motor_poles_counts  a long value between 1 to 254
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_motor_poles_counts(self, motor_poles_counts: int) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_motor_poles_counts_input_validation(motor_poles_counts)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        motor_poles_counts = int(motor_poles_counts)
        information_to_send = convert_to_data(motor_poles_counts, DataType.UINT32)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_MOTOR_POLES_COUNTS, 0x00, information_to_send)
        return result, error

    ##
    #@brief  This command sets the pre-quad number of physical lines of an
    #        incremental encoder engraved on its disk
    #          .The method refers to the Object Dictionary: 0x3010
    #@param  incremental_encoder_lines  a long value [pre-quad]
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_incremental_encoder_lines(self, incremental_encoder_lines: int) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_incremental_encoder_lines_input_validation(incremental_encoder_lines)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        incremental_encoder_lines = int(incremental_encoder_lines)
        information_to_send = convert_to_data(
            incremental_encoder_lines, DataType.UINT32)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_INCREMENTAL_ENCODER_LINES, 0x00, information_to_send)
        return result, error

    ##
    #@brief  This command sets the allowed speed during trajectory following
    #        in closed-loop position controlling mode
    #          .The method refers to the Object Dictionary: 0x3011
    #@param  speed_limit  a long value [RPM]
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_speed_limit(self, speed_limit: int) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_speed_limit_input_validation(speed_limit)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        speed_limit = int(speed_limit)
        information_to_send = convert_to_data(speed_limit, DataType.UINT32)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_SPEED_LIMIT, 0x00, information_to_send)
        return result, error

    ##
    #@brief  This command sets the type of the feedback control SOLO has to operate
    #          .The method refers to the Object Dictionary: 0x3013
    #@param  feedback_control_mode  enum that specify the type of the feedback control SOLO
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_feedback_control_mode(self, feedback_control_mode: FeedbackControlMode) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_feedback_control_mode_input_validation(feedback_control_mode)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error

        if isinstance(feedback_control_mode, int):
            feedback_control_mode = FeedbackControlMode(feedback_control_mode)

        feedback_control_mode = feedback_control_mode.value
        information_to_send = convert_to_data(feedback_control_mode, DataType.UINT32)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_FEEDBACK_CONTROL_MODE, 0x00, information_to_send)
        return result, error

    ##
    #@brief  This command resets SOLO to its factory setting to all the default parameters
   #          .The method refers to the Object Dictionary: 0x3014
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def reset_factory(self) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        information_to_send = [0x00, 0x00, 0x00, 0x01]
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_RESET_FACTORY, 0x00, information_to_send)
        return result, error

    ##
    #@brief  This command sets the Motor type that is connected to SOLO in Digital Mode
    #          .The method refers to the Object Dictionary: 0x3015
    #@param  motor_type  enum that specify the Motor type that is connected to SOLO in Digital Mode
   #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_motor_type(self, motor_type: MotorType) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_motor_type_input_validation(motor_type)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error

        if isinstance(motor_type, int):
            motor_type = MotorType(motor_type)

        motor_type = motor_type.value
        information_to_send = convert_to_data(motor_type, DataType.UINT32)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_MOTOR_TYPE, 0x00, information_to_send)
        return result, error

    ##
    #@brief  This command sets the Control Mode in terms of Torque,
    #        Speed or Position only in Digital Mode
    #          .The method refers to the Object Dictionary: 0x3016
    #@param  control_mode  enum that specify the Control Mode in terms of Torque,
    #                      Speed or Position only in Digital Mode
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_control_mode(self, control_mode: ControlMode) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_control_mode_input_validation(control_mode)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error

        if isinstance(control_mode, int):
            control_mode = ControlMode(control_mode)

        control_mode = control_mode.value
        information_to_send = convert_to_data(control_mode, DataType.UINT32)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_CONTROL_MODE, 0x00, information_to_send)
        return result, error

    ##
    #@brief  This command sets the value for Current Controller Kp or proportional gain
    #          .The method refers to the Object Dictionary: 0x3017
    #@param  current_controller_kp  a float value between 0 to 16000
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_current_controller_kp(self, current_controller_kp: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_current_controller_kp_input_validation(current_controller_kp)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        current_controller_kp = float(current_controller_kp)
        information_to_send = convert_to_data(
            current_controller_kp, DataType.SFXT)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_CURRENT_CONTROLLER_KP, 0x00, information_to_send)
        return result, error

    ##
    #@brief  This command sets the value for Current Controller Ki or integral gain
    #          .The method refers to the Object Dictionary: 0x3018
    #@param  current_controller_ki  a float value between 0 to 16000
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_current_controller_ki(self, current_controller_ki: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_current_controller_ki_input_validation(current_controller_ki)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        current_controller_ki = float(current_controller_ki)
        information_to_send = convert_to_data(
            current_controller_ki, DataType.SFXT)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_CURRENT_CONTROLLER_KI, 0x00, information_to_send)
        return result, error 

    ##
    #@brief  depending on the Motor type: in case of BLDC or PMSM motors Sets the Field
    #        Weakening current reference to help the motor reaching speeds higher than
    #        nominal values and in case of AC Induction Motors Sets the desired magnetizing
    #        current (Id) required for controlling ACIM motors in FOC in Amps
    #          .The method refers to the Object Dictionary: 0x301A
    #@param  magnetizing_current_id_reference  a float value [Amps]
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_magnetizing_current_id_reference(self, magnetizing_current_id_reference: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_magnetizing_current_id_reference_input_validation(magnetizing_current_id_reference)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        magnetizing_current_id_reference = float(magnetizing_current_id_reference)
        information_to_send = convert_to_data(
            magnetizing_current_id_reference, DataType.SFXT)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_MAGNETIZING_CURRENTID_REFERENCE, 0x00, information_to_send)
        return result, error

    ##
    #@brief  This command sets the desired Position reference in terms of quadrature
    #        pulses while SOLO operates with the Incremental Encoders or in terms of
    #        pulses while while SOLO operates with Hall sensors
    #          .The method refers to the Object Dictionary: 0x301B
    #@param  position_reference  a long value [Quad-Pulse]
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_position_reference(self, position_reference: int) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_position_reference_input_validation(position_reference)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        position_reference = int(position_reference)
        information_to_send = convert_to_data(position_reference, DataType.INT32)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_POSITION_REFERENCE, 0x00, information_to_send)
        return result, error

    ##
    #@brief  This command sets the value for Position Controller Kp or proportional gain
    #          .The method refers to the Object Dictionary: 0x301C
    #@param  position_controller_kp  a float value between 0 to 16000
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_position_controller_kp(self, position_controller_kp: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_position_controller_kp_input_validation(position_controller_kp)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        position_controller_kp = float(position_controller_kp)
        information_to_send = convert_to_data(
            position_controller_kp, DataType.SFXT)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_POSITION_CONTROLLER_KP, 0x00, information_to_send)
        return result, error

    ##
    #@brief  This command sets the value for Position Controller Ki or integrator gain
    #          .The method refers to the Object Dictionary: 0x301D
    #@param  position_controller_ki  a float value between 0 to 16000
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_position_controller_ki(self, position_controller_ki: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_position_controller_ki_input_validation(position_controller_ki)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        position_controller_ki = float(position_controller_ki)
        information_to_send = convert_to_data(
            position_controller_ki, DataType.SFXT)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_POSITION_CONTROLLER_KI, 0x00, information_to_send)
        return result, error
        
    ##
    #@brief  This command resets the position counter back to zero
    #          .The method refers to the Object Dictionary: 0x301F
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def reset_position_to_zero(self) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_RESET_POSITION_TO_ZERO, 0x00, information_to_send)
        return result, error

    ##
    #@brief  This command overwrites the reported errors in Error Register
    #        reported with command code of "0xA1"
    #          .The method refers to the Object Dictionary: 0x3020
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def overwrite_error_register(self) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_OVERWRITE_ERROR_REGISTER, 0x00, information_to_send)
        return result, error

    ##
    #@brief  Once in Zero Speed Full Torque algorithm (ZSFT) for controlling the speed of a BLDC or PMSM
    #            in sensorless fashion, this parameter defines the strength of signal injection into the motor, the
    #            user has to make sure this value is not selected too high or too low
    #          .The method refers to the Object Dictionary: 0x3021
    #@param  zsft_injection_amplitude  a float value between 0.0 to 0.55
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_zsft_injection_amplitude(self, zsft_injection_amplitude: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_zsft_injection_amplitude_input_validation(zsft_injection_amplitude)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        zsft_injection_amplitude = float(zsft_injection_amplitude)
        information_to_send = convert_to_data(zsft_injection_amplitude, DataType.SFXT)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_ZSFT_INJECTION_AMPLITUDE, 0x00, information_to_send)
        return result, error

    ##
    #@brief  Once in Zero Speed Full Torque algorithm (ZSFT) for controlling the speed of a BLDC or PMSM
    #             in sensorless fashion, this parameter defines the strength of signal injection into the motor to
    #               identify the polarity of the Motor at the startup
    #          .The method refers to the Object Dictionary: 0x3022
    #@param  zsft_polarity_amplitude  a float value between 0.0 to 0.55
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_zsft_polarity_amplitude(self, zsft_polarity_amplitude: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_zsft_polarity_amplitude_input_validation(zsft_polarity_amplitude)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        zsft_polarity_amplitude = float(zsft_polarity_amplitude)
        information_to_send = convert_to_data(zsft_polarity_amplitude, DataType.SFXT)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_ZSFT_POLARITY_AMPLITUDE, 0x00, information_to_send)
        return result, error

    ##
    #@brief  This command sets the observer gain for the Non-linear observer
    #        that estimates the speed of a DC brushed once the motor type
    #        is selected as DC brushed
    #          .The method refers to the Object Dictionary: 0x3023
    #@param  observer_gain  a float value between 0.01 to 1000
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_observer_gain_dc(self, observer_gain: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_observer_gain_dc_input_validation(observer_gain)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        observer_gain = float(observer_gain)
        information_to_send = convert_to_data(observer_gain, DataType.SFXT)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_OBSERVER_GAIN_DC, 0x00, information_to_send)
        return result, error

    ##
    #@brief  This command defines the frequency of signal injection into the Motor in
    #           runtime, by selecting zero the full injection frequency will be applied which allows to reach to
    #           higher speeds, however for some motors, it’s better to increase this value
    #          .The method refers to the Object Dictionary: 0x3024
    #@param  zsft_injection_frequency  a long value between 0 to 10
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_zsft_injection_frequency(self, zsft_injection_frequency: int) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_zsft_injection_frequency_input_validation(zsft_injection_frequency)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        zsft_injection_frequency = int(zsft_injection_frequency)
        information_to_send = convert_to_data(zsft_injection_frequency, DataType.UINT32)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_ZSFT_INJECTION_FREQUENCY, 0x00, information_to_send)
        return result, error

    ##
    #@brief  Once in Sensorless speed or torque controlling of a BLDC or PMSM motors, this parameter
    #				defines the speed in which the Low speed algorithm has to switch to high speed algorithm
    #           .The method refers to the Object Dictionary: 0x3025
    #@param  sensorless_transition_speed  a long value between 1 to 5000
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_sensorless_transition_speed(self, sensorless_transition_speed: int) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_sensorless_transition_speed_input_validation(sensorless_transition_speed)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        sensorless_transition_speed = int(sensorless_transition_speed)
        information_to_send = convert_to_data(sensorless_transition_speed, DataType.UINT32)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_SENSORLESS_TRANSITION_SPEED, 0x00, information_to_send)
        return result, error
    

    ##
    #@brief  This command sets the baud-rate of the UART line
    #          .The method refers to the Object Dictionary: 0x3026
    #@param  baudrate  enum that specify the baud-rate of the UART line
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_uart_baudrate(self, baudrate: UartBaudRate) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_uart_baudrate_input_validation(baudrate)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error

        if isinstance(baudrate, int):
            baudrate = UartBaudRate(baudrate)

        baudrate = baudrate.value
        information_to_send = convert_to_data(baudrate, DataType.UINT32)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_UART_BAUDRATE, 0x00, information_to_send)
        return result, error
    
    ##
    #@brief  This command starts or stops the process of sensor calibration
    #          .The method refers to the Object Dictionary: 0x3027
    #@param  calibration_action  enum that specify the process of sensor calibration
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def sensor_calibration(self, calibration_action: PositionSensorCalibrationAction) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = sensor_calibration_input_validation(calibration_action)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error

        if isinstance(calibration_action, int):
            calibration_action = PositionSensorCalibrationAction(calibration_action)

        calibration_action = calibration_action.value
        information_to_send = convert_to_data(
            calibration_action, DataType.UINT32)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_SENSOR_CALIBRATION, 0x00, information_to_send)
        return result, error

    ##
    #@brief  This command sets the per-unit offset identified after sensor calibration
    #        for Encoder or Hall sensors in C.C.W direction
    #          .The method refers to the Object Dictionary: 0x3028
    #@param  encoder_hall_offset  a float value between 0.0 to 1.0
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_encoder_hall_ccw_offset(self, encoder_hall_offset: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_encoder_hall_ccw_offset_input_validation(encoder_hall_offset)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        encoder_hall_offset = float(encoder_hall_offset)
        information_to_send = convert_to_data(encoder_hall_offset, DataType.SFXT)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_ENCODER_HALL_CCW_OFFSET, 0x00, information_to_send)
        return result, error

    ##
    #@brief  This command sets the per-unit offset identified after sensor calibration
    #        for Encoder or Hall sensors in C.W direction
    #          .The method refers to the Object Dictionary: 0x3029
    #@param  encoder_hall_offset  a float value between 0.0 to 1.0
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_encoder_hall_cw_offset(self, encoder_hall_offset: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_encoder_hall_cw_offset_input_validation(encoder_hall_offset)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        encoder_hall_offset = float(encoder_hall_offset)
        information_to_send = convert_to_data(encoder_hall_offset, DataType.SFXT)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_ENCODER_HALL_CW_OFFSET, 0x00, information_to_send)
        return result, error

    ##
    #@brief  This command defines the acceleration value of the Speed for speed controller
    #        both in Analogue and Digital modes in Revolution per square seconds
    #          .The method refers to the Object Dictionary: 0x302A
    #@param  speed_acceleration_value  a float value [Rev/S^2]
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_speed_acceleration_value(self, speed_acceleration_value: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_speed_acceleration_value_input_validation(speed_acceleration_value)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        speed_acceleration_value = float(speed_acceleration_value)
        information_to_send = convert_to_data(
            speed_acceleration_value, DataType.SFXT)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_SPEED_ACCELERATION_VALUE, 0x00, information_to_send)
        return result, error

    ##
    #@brief  This command defines the deceleration value of the Speed for speed controller
    #        both in Analogue and Digital modes in Revolution per square seconds
    #          .The method refers to the Object Dictionary: 0x302B
    #@param  speed_deceleration_value  a float value [Rev/S^2]
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_speed_deceleration_value(self, speed_deceleration_value: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_speed_deceleration_value_input_validation(speed_deceleration_value)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        speed_deceleration_value = float(speed_deceleration_value)
        information_to_send = convert_to_data(
            speed_deceleration_value, DataType.SFXT)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_SPEED_DECELERATION_VALUE, 0x00, information_to_send)
        return result, error

    ##
    #@brief  This command sets the baud rate of CAN bus in CANOpen network
    #          .The method refers to the Object Dictionary: 0x302C
    #@param  canbus_baudrate  enum that specify the baud rate of CAN bus in CANOpen network
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_can_bus_baudrate(self, canbus_baudrate: CanBusBaudRate) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_can_bus_baudrate_input_validation(canbus_baudrate)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error

        if isinstance(canbus_baudrate, int):
            canbus_baudrate = CanBusBaudRate(canbus_baudrate)

        canbus_baudrate = canbus_baudrate.value
        information_to_send = convert_to_data(canbus_baudrate, DataType.UINT32)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_CANBUS_BAUDRATE, 0x00, information_to_send)
        return result, error

    ##
    #@brief  This command defines the resolution of the speed at S/T input
    #          while SOLO operates in Analogue mode
    #           .The method refers to the Object Dictionary: 0x303E
    #@param  division_coefficient  a long value
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_analogue_speed_resolution_division_coefficient(self, division_coefficient: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_analogue_speed_resolution_division_coefficient_input_validation(division_coefficient)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error

        division_coefficient = float(division_coefficient)
        information_to_send = convert_to_data(division_coefficient, DataType.SFXT)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_ASRDC, 0x00, information_to_send)
        return result, error

    ##
    #@brief  This command defines the type of the Motion Profile that is
    #          being used in Speed or Position Modes
    #           .The method refers to the Object Dictionary: 0x3040
    #@param  motion_profile_mode enum that specify the type of the Motion Profile
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_motion_profile_mode(self, motion_profile_mode: MotionProfileMode) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_motion_profile_mode_input_validation(motion_profile_mode)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error

        if isinstance(motion_profile_mode, int):
            motion_profile_mode = MotionProfileMode(motion_profile_mode)

        motion_profile_mode = motion_profile_mode.value
        information_to_send = convert_to_data(motion_profile_mode, DataType.UINT32)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_MOTION_PROFILE_MODE, 0x00, information_to_send)
        return result, error

    ##
    #@brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
    #           .The method refers to the Object Dictionary: 0x3041
    #@param  motion_profile_variable1 a float value
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_motion_profile_variable1(self, motion_profile_variable1: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_motion_profile_variable1_input_validation(motion_profile_variable1)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error

        motion_profile_variable1 = float(motion_profile_variable1)
        information_to_send = convert_to_data(motion_profile_variable1, DataType.SFXT)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_MOTION_PROFILE_VARIABLE1, 0x00, information_to_send)
        return result, error

    ##
    #@brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
    #           .The method refers to the Object Dictionary: 0x3042
    #@param  motion_profile_variable2 a float value
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_motion_profile_variable2(self, motion_profile_variable2: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_motion_profile_variable2_input_validation(motion_profile_variable2)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error

        motion_profile_variable2 = float(motion_profile_variable2)
        information_to_send = convert_to_data(motion_profile_variable2, DataType.SFXT)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_MOTION_PROFILE_VARIABLE2, 0x00, information_to_send)
        return result, error

    ##
    #@brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
    #          .The method refers to the Object Dictionary: 0x3043
    #@param  motion_profile_variable3 a float value
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_motion_profile_variable3(self, motion_profile_variable3: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_motion_profile_variable3_input_validation(motion_profile_variable3)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error

        motion_profile_variable3 = float(motion_profile_variable3)
        information_to_send = convert_to_data(motion_profile_variable3, DataType.SFXT)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_MOTION_PROFILE_VARIABLE3, 0x00, information_to_send)
        return result, error

    ##
    #@brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
    #           .The method refers to the Object Dictionary: 0x3044
    #@param  motion_profile_variable4 a float value
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_motion_profile_variable4(self, motion_profile_variable4: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_motion_profile_variable4_input_validation(motion_profile_variable4)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error

        motion_profile_variable4 = float(motion_profile_variable4)
        information_to_send = convert_to_data(motion_profile_variable4, DataType.SFXT)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_MOTION_PROFILE_VARIABLE4, 0x00, information_to_send)
        return result, error

    ##
    #@brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
    #           .The method refers to the Object Dictionary: 0x3045
    #@param  motion_profile_variable5 a float value     
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_motion_profile_variable5(self, motion_profile_variable5: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_motion_profile_variable5_input_validation(motion_profile_variable5)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error

        motion_profile_variable5 = float(motion_profile_variable5)
        
        information_to_send = convert_to_data(motion_profile_variable5, DataType.SFXT)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_MOTION_PROFILE_VARIABLE5, 0x00, information_to_send)
        return result, error

    ##
    #@brief     This command Set the Digiatal Ouput pin Status
    #           .The method refers to the Object Dictionary: 0x3048
    #@param  channel SOLOMotorControllers.Channel
    #@param  state	 SOLOMotorControllers.DigitalIoState
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    #
    def set_digital_output_state(self, channel: Channel, state: DigitalIoState) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_digital_output_state_input_validation(channel)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error

        last_out_register = get_digital_outputs_register()
        if state == 1:
            last_out_register = last_out_register | (1 << channel)
        else:
            last_out_register = last_out_register & (~(1 << channel))

        information_to_send = convert_to_data(last_out_register, DataType.SFXT)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_DIGITAL_OUTPUTS_REGISTER, 0x00, information_to_send)
        return result, error

    ##
    #@brief  This command defines the maximum allowed regeneration current sent back from the Motor to
    #				the Power Supply during decelerations
    #           .The method refers to the Object Dictionary: 0x304B
    #@param  current a float value
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_regeneration_current_limit(self, current: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_regeneration_current_limit_input_validation(current)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error

        current = float(current)
        information_to_send = convert_to_data(current, DataType.SFXT)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_REGENERATION_CURRENT_LIMIT, 0x00, information_to_send)
        return result, error

    ##
    #@brief  This value defines the the sampling window of qualification digital filter applied to the output of
    #			the position sensor before being processed by DSP
    #           .The method refers to the Object Dictionary: 0x304C
    #@param  level a long value
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration
    def set_position_sensor_digital_filter_level(self, level: int) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_position_sensor_digital_filter_level_input_validation(level)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error

        level = int(level)
        information_to_send = convert_to_data(level, DataType.UINT32)
        result, error = self.canopen_transmit(
            self._address, ConstantCanopen.OBJECT_POSITION_SENSOR_DIGITAL_FILTER_LEVEL, 0x00, information_to_send)
        return result, error

    ##
    # @brief  This PDO command sets the desired Position reference in terms of quadrature
    #         pulses while SOLO operates with the Incremental Encoders or in terms of
    #         pulses while while SOLO operates with Hall sensors
    #				.The method refers to the Object Dictionary: 0x1414
    # @param position_reference  a long value between -2,147,483,647 to 2,147,483,647
    # @retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    #
    def set_pdo_position_reference(self, position_reference: int) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND

        res, error, str = set_position_reference_input_validation(position_reference)
        if error != Error.NO_ERROR_DETECTED:
            return res, error
        return self.set_pdo_parameter_value(PdoParameterName.POSITION_REFERENCE, position_reference)

    ##
    # @brief  This PDO command defines the speed reference for SOLO once it’s in Digital Speed Mode
    #				.The method refers to the Object Dictionary: 0x1415
    # @param speed_reference  a long value defining the speed (only positive)
    # @retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    #
    def set_pdo_speed_reference(self, speed_reference: int) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        res, error, str = set_speed_reference_input_validation(speed_reference)
        if error != Error.NO_ERROR_DETECTED:
            return res, error

        return self.set_pdo_parameter_value(PdoParameterName.SPEED_REFERENCE,speed_reference)

    ##
    # @brief  This PDO command sets the amount of desired current that acts in torque generation
    #				.The method refers to the Object Dictionary: 0x1416
    # @param TORQUE_REFERENCE_IQ  a float value between 0 to 32
    # @retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    #
    def set_pdo_torque_reference_iq(self, TORQUE_REFERENCE_IQ: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        res, error, str = set_torque_reference_iq_input_validation(TORQUE_REFERENCE_IQ)
        if error != Error.NO_ERROR_DETECTED:
            return False, error

        return self.set_pdo_parameter_value(PdoParameterName.TORQUE_REFERENCE_IQ,TORQUE_REFERENCE_IQ, DataType.SFXT)

    ##
    # @brief  this PDO command depending on the Motor type: in case of BLDC or PMSM motors Sets the Field
    #         Weakening current reference to help the motor reaching speeds higher than
    #         nominal values and in case of AC Induction Motors Sets the desired magnetizing
    #        current (Id) required for controlling ACIM motors in FOC in Amps
    #				.The method refers to the Object Dictionary: 0x1417
    # @param magnetizing_current_id_reference  a float value between 0 to 32
    # @retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    #
    def set_pdo_magnetizing_current_id_reference(self, magnetizing_current_id_reference: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        res, error, str = set_magnetizing_current_id_reference_input_validation(magnetizing_current_id_reference)
        if error != Error.NO_ERROR_DETECTED:
            return res, error

        return self.set_pdo_parameter_value(PdoParameterName.MAGNETIZING_CURRENT_ID_REFERENCE,magnetizing_current_id_reference, DataType.SFXT)

    ##
    # @brief  This PDO command sets the Control Mode in terms of Torque,
    #         Speed or Position only in Digital Mode
    #			.The method refers to the Object Dictionary: 0x1418
    # @param control_mode  enum that specify the Control Mode in terms of Torque,
    #                       Speed or Position only in Digital Mode
    # @retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    #
    def set_pdo_control_mode(self, control_mode: ControlMode) -> Tuple[bool, Error]:
        return self.set_pdo_parameter_value(PdoParameterName.CONTROL_MODE, control_mode.value)

    ##
    # @brief  This PDO command sets the direction of the rotation of the motor
    #         either to ClockWise rotation or to Counter Clockwise Rotation
    #				.The method refers to the Object Dictionary: 0x1419
    # @param motor_direction  enum that specify the direction of the rotation of the motor
    # @retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    #
    def set_pdo_motor_direction(self, motor_direction: Direction) -> Tuple[bool, Error]:
        return self.set_pdo_parameter_value(PdoParameterName.MOTOR_DIRECTION, motor_direction.value)


# ----------------------------------------------
# ---------------------Read---------------------
# ----------------------------------------------

    def get_read_error_register(self) -> Tuple[int, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_READ_ERROR_REGISTER, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.UINT32), error
        return -1, error

    def get_guard_time(self) -> Tuple[int, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_GUARDTIME, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.UINT32), error
        return -1, error

    def get_life_time_factor(self) -> Tuple[int, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_LIFETIME_FACTOR, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.UINT32), error
        return -1, error

    def get_producer_heartbeat_time(self) -> Tuple[int, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_PRODUCER_HEARTBEAT_TIME, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.UINT32), error
        return -1, error

    ##
    #@brief  This command reads the device address connected on the line
    #          .The method refers to the Object Dictionary: 0x3001
    #@retval List of [long device address connected on the line, Error class/enumeration]
    def get_device_address(self) -> Tuple[int, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_SET_DEVICE_ADDRESS, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.UINT32), error
        return -1, error

    ##
    #@brief  This command reads the phase-A voltage of the motor connected to the
    #        "A" pin output of SOLO for 3-phase Motors
    #          .The method refers to the Object Dictionary: 0x302D
    #@retval List of [float phase-A voltage of the motor [Volts], Error class/enumeration]
    def get_phase_a_voltage(self) -> Tuple[float, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_PHASEA_VOLTAGE, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the phase-B voltage of the motor connected to the
    #        "B" pin output of SOLO for 3-phase Motors
    #          .The method refers to the Object Dictionary: 0x302E
    #@retval List of [float 0 phase-A voltage of the motor [Volts], Error class/enumeration]
    def get_phase_b_voltage(self) -> Tuple[float, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_PHASEB_VOLTAGE, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the phase-A current of the motor connected to the
    #        "A" pin output of SOLO for 3-phase Motors
    #          .The method refers to the Object Dictionary: 0x302F
    #@retval List of [float phase-A current of the motor [Amps], Error class/enumeration]
    def get_phase_a_current(self) -> Tuple[float, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_PHASEA_CURRENT, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the phase-B current of the motor connected to the
    #        "B" pin output of SOLO for 3-phase Motors
    #          .The method refers to the Object Dictionary: 0x3030
    #@retval List of [float phase-B current of the motor [Amps], Error class/enumeration]
    def get_phase_b_current(self) -> Tuple[float, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_PHASEB_CURRENT, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the input BUS voltage
    #          .The method refers to the Object Dictionary: 0x3031
    #@retval List of [float  BUS voltage [Volts], Error class/enumeration]
    def get_bus_voltage(self) -> Tuple[float, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_BUS_VOLTAGE, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the current inside the DC brushed motor connected to
    #        "B" and "C" outputs of SOLO
    #          .The method refers to the Object Dictionary: 0x3032
    #@retval List of [float between [Amps], Error class/enumeration]
    def get_dc_motor_current_im(self) -> Tuple[float, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_DC_MOTOR_CURRENT_IM, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the voltage of the DC brushed motor connected to
    #        "B" and "C" outputs of SOLO
    #          .The method refers to the Object Dictionary: 0x3033
    #@retval List of [float [Volts], Error class/enumeration]
    def get_dc_motor_voltage_vm(self) -> Tuple[float, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_DC_MOTOR_VOLTAGE_VM, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the value of the Speed controller Kp gain,
    #        set for Digital mode operations
    #          .The method refers to the Object Dictionary: 0x300A
    #@retval List of [float between 0 to 16000, Error class/enumeration]
    def get_speed_controller_kp(self) -> Tuple[float, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_SPEED_CONTROLLER_KP, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the value of the Speed controller Ki gain,
    #        set for Digital mode operations
    #          .The method refers to the Object Dictionary: 0x300B
    #@retval List of [float between 0 to 16000, Error class/enumeration]
    def get_speed_controller_ki(self) -> Tuple[float, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_SPEED_CONTROLLER_KI, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the output switching frequency of SOLO in Hertz
    #          .The method refers to the Object Dictionary: 0x3009
    #@retval List of [long [KHz], Error class/enumeration]
    def get_output_pwm_frequency_khz(self) -> Tuple[int, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_OUTPUT_PWMFREQUENCY_KHZ, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.UINT32), error
        return -1, error

    ##
    #@brief  This command reads the value of the current limit set for SOLO in
    #        closed-loop digital operation mode
    #          .The method refers to the Object Dictionary: 0x3003
    #@retval List of [float [Amps], Error class/enumeration]
    def get_current_limit(self) -> Tuple[float, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_CURRENT_LIMIT, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the actual monetary value of “Iq” that is
    #        the current acts in torque generation in FOC mode for 3-phase motors
    #          .The method refers to the Object Dictionary: 0x3034
    #@retval List of [float [Amps], Error class/enumeration]
    def get_quadrature_current_iq_feedback(self) -> Tuple[float, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_QUADRATURE_CURRENT_IQ_FEEDBACK, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the actual monetary value of Id that is the
    #        direct current acting in FOC
    #          .The method refers to the Object Dictionary: 0x3035
    #@retval List of [float [Amps], Error class/enumeration]
    def get_magnetizing_current_id_feedback(self) -> Tuple[float, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_MAGNETIZING_CURRENT_ID_FEEDBACK, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the number of Poles set for 3-phase motors
    #          .The method refers to the Object Dictionary: 0x300F
    #@retval List of [long between 1 to 254, Error class/enumeration]
    def get_motor_poles_counts(self) -> Tuple[int, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_MOTOR_POLES_COUNTS, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.UINT32), error
        return -1, error

    ##
    #@brief  This command reads the number of physical Incremental encoder lines set on SOLO
    #          .The method refers to the Object Dictionary: 0x3010
    #@retval List of [long between 1 to 200000, Error class/enumeration]
    def get_incremental_encoder_lines(self) -> Tuple[int, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_INCREMENTAL_ENCODER_LINES, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.UINT32), error
        return -1, error

    ##
    #@brief  This command reads the amount of value set for Current controller
    #        Kp or proportional gain
    #          .The method refers to the Object Dictionary: 0x3017
    #@retval List of [float between 0 to 16000, Error class/enumeration]
    def get_current_controller_kp(self) -> Tuple[float, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_CURRENT_CONTROLLER_KP, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the amount of value set for Current controller
    #        Ki or integrator gain
    #          .The method refers to the Object Dictionary: 0x3018
    #@retval List of [float between 0 to 16000, Error class/enumeration]
    def get_current_controller_ki(self) -> Tuple[float, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_CURRENT_CONTROLLER_KI, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the momentary temperature of the board in centigrade
    #          .The method refers to the Object Dictionary: 0x3039
    #@retval List of [float [°C], Error class/enumeration]
    def get_board_temperature(self) -> Tuple[float, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_BOARD_TEMPERATURE, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the Phase or Armature resistance of
    #        the 3-phase or DC brushed motor connected to SOLO respectively
    #          .The method refers to the Object Dictionary: 0x300D
    #@retval List of [float [Ohms], Error class/enumeration]
    def get_motor_resistance(self) -> Tuple[float, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_MOTOR_RESISTANCE, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the Phase or Armature Inductance of
    #        the 3-phase or DC brushed motor connected to SOLO respectively
    #          .The method refers to the Object Dictionary: 0x300E
    #@retval List of [float [Henry], Error class/enumeration]
    def get_motor_inductance(self) -> Tuple[float, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_MOTOR_INDUCTANCE, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.SFXT), error
        return -1.0, error

    ##
    #@brief  his command reads the actual speed of the motor measured or estimated by SOLO in
    #        sensorless or sensor-based modes respectively
    #          .The method refers to the Object Dictionary: 0x3036
    #@retval List of [long [RPM], Error class/enumeration]
    def get_speed_feedback(self) -> Tuple[int, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_SPEED_FEEDBACK, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.UINT32), error
        return -1, error

    ##
    #@brief  This command reads the Motor type selected for Digital or Analogue mode operations
    #          .The method refers to the Object Dictionary: 0x3015
    #@retval List of [long between 0 to 3, Error class/enumeration]
    def get_motor_type(self) -> Tuple[MotorType, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_MOTOR_TYPE, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return MotorType(convert_from_data(information_received, DataType.UINT32)), error
        return -1, error

    ##
    #@brief  This command reads the feedback control mode selected on SOLO both
    #        for Analogue and Digital operations
    #          .The method refers to the Object Dictionary: 0x3013
    #@retval List of [long between 0 to 2, Error class/enumeration]
    def get_feedback_control_mode(self) -> Tuple[FeedbackControlMode, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_FEEDBACK_CONTROL_MODE, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return FeedbackControlMode(convert_from_data(information_received, DataType.UINT32)), error
        return -1, error

    ##
    #@brief  This command reads the actual commanding mode that SOLO is operating
    #          .The method refers to the Object Dictionary: 0x3002
    #@retval List of [long between 0 or 1, Error class/enumeration]
    def get_command_mode(self) -> Tuple[CommandMode, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_COMMAND_MODE, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return CommandMode(convert_from_data(information_received, DataType.UINT32)), error
        return -1, error

    ##
    #@brief  This command reads the Control Mode type in terms of Torque,
    #        Speed or Position in both Digital and Analogue modes
    #          .The method refers to the Object Dictionary: 0x3013
    #@retval List of [long between 0 to 2, Error class/enumeration]
    def get_control_mode(self) -> Tuple[ControlMode, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_CONTROL_MODE, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return ControlMode(convert_from_data(information_received, DataType.UINT32)), error
        return -1, error

    ##
    #@brief  This command reads the value of the speed limit set on SOLO
    #          .The method refers to the Object Dictionary: 0x3011
    #@retval List of [long [RPM], Error class/enumeration]
    def get_speed_limit(self) -> Tuple[int, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_SPEED_LIMIT, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.UINT32), error
        return -1, error

    ##
    #@brief  This command reads the amount of value set for Position
    #        controller Kp or proportional gain
    #          .The method refers to the Object Dictionary: 0x301C
    #@retval List of [float between 0 to 16000, Error class/enumeration]
    def get_position_controller_kp(self) -> Tuple[float, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_POSITION_CONTROLLER_KP, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the amount of value set for Position
    #        controller Ki or integrator gain
    #          .The method refers to the Object Dictionary: 0x301D
    #@retval List of [float between 0 to 16000, Error class/enumeration]
    def get_position_controller_ki(self) -> Tuple[float, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_POSITION_CONTROLLER_KI, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the number of counted pulses from the
    #        Incremental Encoder or Hall sensors
    #          .The method refers to the Object Dictionary: 0x3037
    #@retval List of [long [Quad-Pulses], Error class/enumeration]
    def get_position_counts_feedback(self) -> Tuple[int, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_POSITION_COUNTS_FEEDBACK, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.UINT32), error
        return -1, error

    ##
    #@brief  This command reads the error register which is a 32 bit register with
    #        each bit corresponding to specific errors
    #          .The method refers to the Object Dictionary: 0x3020
    #@retval List of [long , Error class/enumeration]
    def get_error_register(self) -> Tuple[int, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_OVERWRITE_ERROR_REGISTER, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.UINT32), error
        return -1, error

    ##
    #@brief  This command reads the Firmware version existing currently on the SOLO unit
    #          .The method refers to the Object Dictionary: 0x303A
    #@retval List of [long, Error class/enumeration]
    def get_device_firmware_version(self) -> Tuple[int, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_DEVICE_FIRMWARE_VERSION, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.UINT32), error
        return -1, error

    ##
    #@brief  This command reads the Hardware version of the SOLO unit connected
    #          .The method refers to the Object Dictionary: 0x303B
    #@retval List of [long, Error class/enumeration]
    def get_device_hardware_version(self) -> Tuple[int, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_DEVICE_HARDWARE_VERSION, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.UINT32), error
        return -1, error

    ##
    #@brief  This command reads the amount of desired Torque reference (Iq or IM)
    #        already set for the Motor to follow in Digital Closed-loop Torque control mode
    #          .The method refers to the Object Dictionary: 0x3004
    #@retval List of [float [Amps], Error class/enumeration]
    def get_torque_reference_iq(self) -> Tuple[float, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_TORQUE_REFERENCE_IQ, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the amount of desired Speed reference already set for
    #        the Motor to follow in Digital Closed-loop Speed control mode
    #          .The method refers to the Object Dictionary: 0x3005
    #@retval List of [long [RPM], Error class/enumeration]
    def get_speed_reference(self) -> Tuple[int, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_SPEED_REFERENCE, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.UINT32), error
        return -1, error

    ##
    #@brief  This command reads the amount of desired Id (direct current) or
    #        Magnetizing current reference already set for the Motor to follow
    #        in Digital Closed-loop Speed control mode for ACIM motors
    #          .The method refers to the Object Dictionary: 0x301A
    #@retval List of [float [Amps], Error class/enumeration]
    def get_magnetizing_current_id_reference(self) -> Tuple[float, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_MAGNETIZING_CURRENTID_REFERENCE, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the desired position reference set for the Motor
    #        to follow in Digital Closed-loop Position mode in terms of quadrature pulses
    #          .The method refers to the Object Dictionary: 0x301B 
    #@retval List of [long [Quad-Pulses], Error class/enumeration]
    def get_position_reference(self) -> Tuple[int, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_POSITION_REFERENCE, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.UINT32), error
        return -1, error

    ##
    #@brief  This command reads the desired Power reference for SOLO to apply in
    #        Digital Open-loop speed control mode for 3-phase motors in terms of percentage
    #          .The method refers to the Object Dictionary: 0x3006
    #@retval List of [float [%], Error class/enumeration]
    def get_power_reference(self) -> Tuple[float, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_POWER_REFERENCE, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.SFXT), error
        return -1.0, error

    ##
    #@brief  This commands reads the desired direction of rotation set for the Motor
    #          .The method refers to the Object Dictionary: 0x300C
    #@retval List of [long 0 Counter ClockWise / 1 ClockWise, Error class/enumeration]
    def get_motor_direction(self) -> Tuple[Direction, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_MOTOR_DIRECTION, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return Direction(convert_from_data(information_received, DataType.UINT32)), error
        return -1, error

    ##
    #@brief  This command reads the value of Sensorless Zero Speed Full Torque Injection Amplitude
    #          .The method refers to the Object Dictionary: 0x3021
    #@retval List of [float between 0.0 to 0.55, Error class/enumeration]
    def get_zsft_injection_amplitude(self) -> Tuple[float, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_ZSFT_INJECTION_AMPLITUDE, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.SFXT), error
        return -1.0, error
    
    ##
    #@brief  This command reads the value of Sensorless Zero Speed Full Torque Polarity Amplitude
    #          .The method refers to the Object Dictionary: 0x3022
    #@retval List of [float between 0.0 to 0.55, Error class/enumeration]
    def get_zsft_polarity_amplitude(self) -> Tuple[float, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_ZSFT_POLARITY_AMPLITUDE, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the value of Sensorless Zero Speed Full Torque Injection Frequency
    #          .The method refers to the Object Dictionary: 0x3024
    #@retval List of [long between 0 to 10, Error class/enumeration]
    def get_zsft_injection_frequency(self) -> Tuple[float, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_ZSFT_INJECTION_FREQUENCY, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.UINT32), error
        return -1.0, error

    ##
    #@brief  This command reads the value of Sensorless Observer Gain for DC Motor
    #          .The method refers to the Object Dictionary: 0x3023
    #@retval List of [float between 0.01 to 1000, Error class/enumeration]
    def get_observer_gain_dc(self) -> Tuple[float, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_OBSERVER_GAIN_DC, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the value of Sensorless Observer
    #        Filter Gain for Normal BLDC-PMSM Motors
    #          .The method refers to the Object Dictionary: 0x3024
    #@retval List of [float between 0.01 to 16000, Error class/enumeration]
    def get_filter_gain_bldc_pmsm(self) -> Tuple[float, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_FILTER_GAIN_BLDC_PMSM, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the value of Sensorless Transition Speed
    #          .The method refers to the Object Dictionary: 0x3025
    #@retval List of [long between 1 to 5000, Error class/enumeration]
    def get_sensorless_transition_speed(self) -> Tuple[float, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_SENSORLESS_TRANSITION_SPEED, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.UINT32), error
        return -1.0, error

    ##
    #@brief  This command reads the measured or estimated per-unit angle of the 3-phase motors
    #          .The method refers to the Object Dictionary: 0x3038
    #@retval List of [float [Per Unit], Error class/enumeration]
    def get_3phase_motor_angle(self) -> Tuple[float, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_3PHASE_MOTOR_ANGLE, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the per-unit Encoder or Hall sensor offset in C.C.W direction
    #          .The method refers to the Object Dictionary: 0x3028
    #@retval List of [float [Per Unit], Error class/enumeration]
    def get_encoder_hall_ccw_offset(self) -> Tuple[float, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_ENCODER_HALL_CCW_OFFSET, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the per-unit Encoder or Hall sensor offset in C.C.W direction
    #          .The method refers to the Object Dictionary: 0x3029
    #@retval List of [float [Per Unit], Error class/enumeration]
    def get_encoder_hall_cw_offset(self) -> Tuple[float, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_ENCODER_HALL_CW_OFFSET, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads Baud Rate selected on SOLO unit to communicate through UART line
    #          .The method refers to the Object Dictionary: 0x3026
    #@retval List of [long [Bits/s], Error class/enumeration]
    def get_uart_baudrate(self) -> Tuple[UartBaudRate, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_UART_BAUDRATE, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return UartBaudRate(convert_from_data(information_received, DataType.UINT32)), error
        return -1, error

    ##
    #@brief  This command reads the acceleration value of the Speed for
    #        speed controller both in Analogue and Digital modes
    #        in Revolution per square seconds
    #          .The method refers to the Object Dictionary: 0x302A
    #@retval List of [float [Rev/S^2], Error class/enumeration]
    def get_speed_acceleration_value(self) -> Tuple[float, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_SPEED_ACCELERATION_VALUE, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the deceleration value of the Speed for
    #        speed controller both in Analogue and Digital modes
    #        in Revolution per square seconds
    #          .The method refers to the Object Dictionary: 0x302B
    #@retval List of [float [Rev/S^2], Error class/enumeration]
    def get_speed_deceleration_value(self) -> Tuple[float, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_SPEED_DECELERATION_VALUE, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the Analogue Speed Resolution Division Coefficient (ASRDC)
    #           .The method refers to the Object Dictionary: 0x303E
    #          while SOLO operates in Analogue mode
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def get_analogue_speed_resolution_division_coefficient(self) -> Tuple[float, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_ASRDC, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.SFXT), error
        return -1, error

    ##
    #@brief  This command test if the communication is working
    #@retval  List of [ bool 0 not working / 1 for working, Error class/enumeration]
    def communication_is_working(self) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        temperature, error = self.get_board_temperature()
        time.sleep(0.2)
        temperature, error = self.get_board_temperature()
        if error == Error.NO_ERROR_DETECTED:
            return True, error
        return False, error

    ##
    #@brief  This Command reads the number of counted index pulses
    #        seen on the Incremental Encoder’s output
    #          .The method refers to the Object Dictionary: 0x303D
    #@retval List of [long [Pulses], Error class/enumeration]
    def get_encoder_index_counts(self) -> Tuple[int, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_ENCODER_INDEX_COUNTS, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.UINT32), error
        return -1, error

    ##
    #@brief  This command reads the type of the Embedded Motion profile active in the controller
    #          being used in Speed or Position Modes
    #           .The method refers to the Object Dictionary: 0x303F
    #@retval List of [Motion profile , Error class/enumeration]
    def get_motion_profile_mode(self) -> Tuple[MotionProfileMode, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_MOTION_PROFILE_MODE, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return MotionProfileMode(convert_from_data(information_received, DataType.UINT32)), error
        return -1, error

    ##
    #@brief  This command reads the value of the Motion Profile Variable1 set inside the controller
    #           .The method refers to the Object Dictionary: 0x3040
    #@retval List of [Motion Profile Variable1, Error class/enumeration]
    def get_motion_profile_variable1(self) -> Tuple[float, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_MOTION_PROFILE_VARIABLE1, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the value of the Motion Profile Variable2 set inside the controller
    #           .The method refers to the Object Dictionary: 0x3041
    #@retval List of [Motion Profile Variable2, Error class/enumeration]
    def get_motion_profile_variable2(self) -> Tuple[float, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_MOTION_PROFILE_VARIABLE2, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the value of the Motion Profile Variable3 set inside the controller
    #           .The method refers to the Object Dictionary: 0x3042
    #@retval List of [Motion Profile Variable3, Error class/enumeration]
    def get_motion_profile_variable3(self) -> Tuple[float, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_MOTION_PROFILE_VARIABLE3, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the value of the Motion Profile Variable4 set inside the controller
    #           .The method refers to the Object Dictionary: 0x3043
    #@retval List of [Motion Profile Variable4, Error class/enumeration]
    def get_motion_profile_variable4(self) -> Tuple[float, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_MOTION_PROFILE_VARIABLE4, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the value of the Motion Profile Variable5 set inside the controller
    #           .The method refers to the Object Dictionary: 0x3044
    #@retval List of [Motion Profile Variable5, Error class/enumeration]
    def get_motion_profile_variable5(self) -> Tuple[float, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_MOTION_PROFILE_VARIABLE5, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the value of the Digital Outputs Register as a 32 bits register, where each
    #               bit represent the state of each output
    #          .The method refers to the Object Dictionary: 0x3048
    #@retval List of [ int, Error class/enumeration]
    def get_digital_outputs_register(self) -> Tuple[int, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_DIGITAL_OUTPUTS_REGISTER, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.UINT32), error
        return -1.0, error

    ##
    #@brief  This command reads the current state of the controller
    #           .The method refers to the Object Dictionary: 0x3008
    #@retval List of [Disable/Enable , Error class/enumeration]
    def get_drive_disable_enable(self) -> Tuple[DisableEnable, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_DRIVE_DISABLE_ENABLE, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return DisableEnable(convert_from_data(information_received, DataType.UINT32)), error
        return -1, error

    ##
    #@brief  This command reads the value of the Regeneration Current Limit
    #           .The method refers to the Object Dictionary: 0x304B
    #@retval List of [float, Error class/enumeration]
    def get_regeneration_current_limit(self) -> Tuple[float, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_REGENERATION_CURRENT_LIMIT, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.SFXT), error
        return -1.0, error

    ##
    #@brief  This command reads the value of the Position Sensor Digital Filter Level
    #           .The method refers to the Object Dictionary: 0x304C
    #@retval List of [int, Error class/enumeration]
    def get_position_sensor_digital_filter_level(self) -> Tuple[int, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_POSITION_SENSOR_DIGITAL_FILTER_LEVEL, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.UINT32), error
        return -1.0, error

    ##
    #@brief  This command reads the value of the Digital Input Register as a 32 bits register
    #           .The method refers to the Object Dictionary: 0x3049
    #@retval List of [int, Error class/enumeration]
    def get_digital_input_register(self) -> Tuple[int, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_DIGITAL_INPUT_REGISTER, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.UINT32), error
        return -1.0, error

    ##
    #@brief  This command reads the value of the voltage sensed at the output of PT1000 temperature
    #			sensor amplifier, this command can be used only on devices that come with PT1000 input
    #           .The method refers to the Object Dictionary: 0x3047
    #@retval List of [int, Error class/enumeration]
    def get_pt1000_sensor_voltage(self) -> Tuple[int, Error]:
        information_to_send = [0x00, 0x00, 0x00, 0x00]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_PT1000_SENSOR_VOLTAGE, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.UINT32), error
        return -1.0, error

    ##
    #@brief  This command reads the quantized value of an Analogue Input as a number between 0 to 4095
    #           .The method refers to the Object Dictionary: 0x304A
    #@param  channel  an enum that specify the Channel of Analogue Input
    #@retval List of [int, Error class/enumeration]
    def get_analogue_input(self, channel: Channel) -> Tuple[int, Error]:
        information_to_send = [0x00, 0x00, 0x00, int(channel)]
        information_received = []
        error = Error.NO_PROCESSED_COMMAND
        result, error, information_received = self.canopen_receive(
            self._address, ConstantCanopen.OBJECT_ANALOGUE_INPUT, 0x00, information_to_send)
        if (error == Error.NO_ERROR_DETECTED and result is True):
            return convert_from_data(information_received, DataType.UINT32), error
        return -1.0, error

    ##
    # @brief  this PDO command give the first in the baffer position of the Motor
    #         to follow in Digital Closed-loop Position mode in terms of quadrature pulses
    #				.The method refers to the Object Dictionary: 0x1814
    # @retval List of [long, Error class/enumeration]
    #
    def get_pdo_position_counts_feedback(self) -> Tuple[int, Error]:
        return self.get_pdo_parameter_value_long(PdoParameterName.POSITION_COUNTS_FEEDBACK)

    ##
    # @brief  this PDO command give the first in the baffer speed of the motor measured or estimated by SOLO in
    #		    sensorless or sensor-based modes respectively
    #				.The method refers to the Object Dictionary: 0x1815
    # @retval List of [long, Error class/enumeration]
    #
    def get_pdo_speed_feedack(self) -> Tuple[int, Error]:
        return self.get_pdo_parameter_value_long(PdoParameterName.SPEED_FEEDBACK)

    ##
    # @brief  This PDO command give the first in the baffer monetary value of “Iq” that is
    #         the current acts in torque generation in FOC mode for 3-phase motors
    #				.The method refers to the Object Dictionary: 0x1816
    # @retval List of [float, Error class/enumeration]
    #
    def get_pdo_quadrature_current_iq_feedback(self) -> Tuple[float, Error]:
        return self.get_pdo_parameter_value_float(PdoParameterName.QUADRATURE_CURRENT_IQ_FEEDBACK)

    ##
    # @brief  This PDO command give the first in the baffer monetary value of Id that is the
    #         direct current acting in FOC
    #			.The method refers to the Object Dictionary: 0x1817
    # @retval List of [float, Error class/enumeration]
    #
    def get_pdo_magnetizing_current_id_feedback(self) -> Tuple[float, Error]:
        return self.get_pdo_parameter_value_float(PdoParameterName.MAGNETIZING_CURRENT_ID_FEEDBACK)

    ##
    # @brief  This PDO command reads the error register which is a 32 bit register with
    #         each bit corresponding to specific errors
    #				.The method refers to the Object Dictionary: 0x1818
    # @retval List of [long, Error class/enumeration]
    #
    def get_pdo_error_register(self) -> Tuple[int, Error]:
        return self.get_pdo_parameter_value_long(PdoParameterName.ERROR_REGISTER)

    ##
    # @brief  This PDO command reads the momentary temperature of the board in centigrade
    #				.The method refers to the Object Dictionary: 0x1819
    # @retval List of [float, Error class/enumeration]
    #
    def get_pdo_board_temperature(self) -> Tuple[float, Error]:
        return self.get_pdo_parameter_value_float(PdoParameterName.BOARD_TEMPERATURE)
