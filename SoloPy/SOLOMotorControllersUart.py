## @package SOLOMotorControllersUart.py
#  @author  SOLOMotorControllers
#  @brief   This file contains all the functions for the Solo Uart Drivers 
#           Availability: https://github.com/Solo-FL/SoloPy/tree/main/SoloPy
#  @date    Date: 2023
#  @version 3.1.1

## @attention
# Copyright: (c) 2021-2023 SOLO motor controllers project
# GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

import serial
import string

import logging
from importlib import reload

import SoloPy.ConstantUart as ConstantUart
from SoloPy.SOLOMotorControllers import *
from SoloPy.SOLOMotorControllersUtils import *


class SoloMotorControllerUart(implements(SOLOMotorControllers)):

    def __init__(
            self,
            port="/dev/ttyS0",
            address=0,
            baudrate=UART_BAUD_RATE.RATE_937500,
            timeout=3,
            loggerLevel=logging.INFO):

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

        self._address = address
        if (baudrate == UART_BAUD_RATE.RATE_937500):
                self._baudrate = 937500
        if (baudrate == UART_BAUD_RATE.RATE_115200):
                self._baudrate = 115200
        self._port = port
        self._timeout = timeout
        self._ser = None
        self.serial_open()
        # TODO try solving serial error

    def __del__(self):
        self._logger.debug('SoloMotorController DEL')
        self.serial_close()

    def __exec_cmd(self, cmd: list) -> list:
        self.serial_open()

        _cmd = [ConstantUart.INITIATOR, ConstantUart.INITIATOR,
                cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], ConstantUart.CRC, ConstantUart.ENDING]

        _readPacket = []

        messageLog = "WRITE: " + str([
            hex(ConstantUart.INITIATOR), hex(ConstantUart.INITIATOR),
            hex(cmd[0]), hex(cmd[1]), hex(cmd[2]), hex(
                cmd[3]), hex(cmd[4]), hex(cmd[5]),
            hex(ConstantUart.CRC), hex(ConstantUart.ENDING)])
        self._logger.debug(messageLog)

        for attempts in range(5):
            try:
                self._ser.write(_cmd)

                # Time to ensure the writing, reducing it can make the comunication instable
                time.sleep(0.1)

                # read up to ten bytes (timeout)
                while self._ser.in_waiting> 0:
                    _readPacket = self._ser.read(10)

                    messageLog = "READ: " + \
                        str([hex(i) for i in _readPacket]) + \
                        " size: " + str(len(_readPacket))
                    self._logger.debug(messageLog)

                if (len(_readPacket) == 0):
                    self._ser.flush()

                if (_readPacket and
                        _readPacket[0] == _cmd[0] and
                        _readPacket[1] == _cmd[1] and
                        (_readPacket[2] == _cmd[2] or _cmd[2] == 0xFF) and
                        _readPacket[8] == _cmd[8] and
                        _readPacket[9] == _cmd[9]):
                    if (_readPacket[3] == _cmd[3]):
                        cmd[0] = _readPacket[2]
                        cmd[1] = _readPacket[3]
                        cmd[2] = _readPacket[4]
                        cmd[3] = _readPacket[5]
                        cmd[4] = _readPacket[6]
                        cmd[5] = _readPacket[7]
                        return True, ERROR.NO_ERROR_DETECTED
                    else:
                        continue
            except Exception as e:
                self._logger.debug('__exec_cmd Exception')
                self._logger.debug(e, exc_info=True)
            time.sleep(0.1) 

        cmd[0] = 0xEE
        cmd[1] = 0xEE
        cmd[2] = 0xEE
        cmd[3] = 0xEE
        cmd[4] = 0xEE
        cmd[5] = 0xEE
        self.serial_close()
        return False, ERROR.GENERAL_ERROR

    # #############################Support##############################
    def serial_open(self) -> bool:
        if(self._ser is None):
            self._logger.debug("serial_open start")
            try:
                self._ser = serial.Serial(
                    self._port, self._baudrate, timeout=self._timeout, writeTimeout=self._timeout)
                self._ser_status = 1

                # Time sleep for ensure serial initialization
                time.sleep(self._timeout)

                self._logger.debug("Serial init")
                self._ser.bytesize = serial.EIGHTBITS
                self._ser.parity = serial.PARITY_NONE
                self._ser.stopbits = serial.STOPBITS_ONE
                time.sleep(0.2)

                self._logger.debug("Serial flush")
                self._ser.reset_input_buffer()
                self._ser.reset_output_buffer()
                time.sleep(self._timeout)

            except Exception as e:
                self._logger.error(
                    "serial_open: Exception during the serial inizialisation")
                # self._logger.error( e, exc_info=True)
                # raise e
                return False

        if (not self._ser.is_open):  #pyserial automatic open
            self._logger.debug("Serial open")
            self._ser.open()
            time.sleep(self._timeout)

            self._logger.debug("Serial flush")
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()
            time.sleep(self._timeout*2)

        self._logger.debug("serial_open end")
        return True

    def serial_error_handler(self) -> bool:
        self._logger.debug('SEH start')
        try:
            try:
                if (self._ser.is_open):
                    self._logger.debug('SEH: serial is open')
                    if (self._ser.inWaiting() > 0):
                        self._logger.debug('SEH: clean buffers')
                        self._ser.flushInput()
                        self._ser.flushOutput()

                    self._logger.debug('SEH: serial close')
                    self._ser.close()
                    self._ser_status = 0

            except Exception as e:
                pass

            self._logger.debug('SEH: serial init')
            res = self.serial_open()
            self._logger.debug('SEH end')
            return res

        except Exception as e:
            self._logger.error("SEH: Exception")
            # self._logger.error( e, exc_info=True)
            self._ser_status = -1
            self._logger.debug('SEH end')
            return False

    def serial_close(self) -> bool:
        self._logger.debug("serial_close start")

        if self._ser is None:
            self._logger.debug("serial_close: Serial not exist")
            return True
        elif not self._ser.is_open:
            self._logger.debug("serial_close: Serial is already close")
            return True
        
        if self._ser.is_open:
            try:
                self._ser.close()
                self._ser = None
                time.sleep(self._timeout*2)
                self._logger.debug("serial_close: Serial closed")
                return True
            except Exception as e:
                self._logger.error("serial_close: Exception on Serial Closure")
                # self._logger.error( e, exc_info=True)
                return False

        return False

    def serial_status(self) -> string:
        status = ''
        if not(self._ser is None) and self._ser.is_open:
            status += 'SPS: Serial Open'
            try:
                status += '\nSPS: Serial out buffer size: ' + \
                str(self._ser.out_waiting)
                status += '\nSPS: Serial in buffer size: ' + \
                str(self._ser.in_waiting)
                self._logger.debug(status)
            except Exception as e:
                self._logger.error('exception on printing the status')
                # self._logger.error( e, exc_info=True)

        return status

    def serial_is_working(self) -> bool:
        if self.get_phase_a_voltage() == -1:
            return False
        return True

    # #############################Write############################# #


    ##
    #@brief  This command sets the desired device address for a SOLO unit
    #          .The method refers to the Uart Write command: 0x01
    #@param  device_address  address want to set for board       
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_device_address(self, device_address: int) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_device_address_input_validation(device_address)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error

        cmd = [self._address, ConstantUart.WriteDeviceAddress,
               0x00, 0x00, 0x00, device_address]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command sets the mode of the operation of SOLO
    #        in terms of operating in Analogue mode or Digital
    #          .The method refers to the Uart Write command: 0x02
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
        data = convert_to_data(mode, DATA_TYPE.UINT32)
        cmd = [self._address, ConstantUart.WriteCommandMode,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)
        
    ##
    #@brief  This command defines the maximum allowed current into the motor in terms of Amps
    #          .The method refers to the Uart Write command: 0x03
    #@param  current_limit  a float value [Amps]     
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_current_limit(self, current_limit: float) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_current_limit_input_validation(current_limit)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error

        current_limit = float(current_limit)
        data = convert_to_data(current_limit, DATA_TYPE.SFXT)
        cmd = [self._address, ConstantUart.WriteCurrentLimit,
               data[0], data[1], data[2], data[3]]

        return self.__exec_cmd(cmd)

    ##
    #@brief  This command sets the amount of desired current that acts in torque generation
    #          .The method refers to the Uart Write command: 0x04
    #@param  torque_reference_iq  a float [Amps]      
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_torque_reference_iq(self, torque_reference_iq: float) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_torque_reference_iq_input_validation(torque_reference_iq)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error

        torque_reference_iq = float(torque_reference_iq)
        data = convert_to_data(torque_reference_iq, DATA_TYPE.SFXT)
        cmd = [self._address, ConstantUart.WriteTorqueReferenceIq,
               data[0], data[1], data[2], data[3]]

        return self.__exec_cmd(cmd)

    ##
    #@brief  This command defines the speed reference for SOLO once it’s in Digital Speed Mode
    #          .The method refers to the Uart Write command: 0x05
    #@param  speed_reference  a long value [RPM]      
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_speed_reference(self, speed_reference: int) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_speed_reference_input_validation(speed_reference)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        speed_reference = int(speed_reference)
        data = convert_to_data(speed_reference, DATA_TYPE.UINT32)
        cmd = [self._address, ConstantUart.WriteSpeedReference,
               data[0], data[1], data[2], data[3]]

        return self.__exec_cmd(cmd)

    ##
    #@brief  This command defines the amount of power percentage during only
    #        Open-loop mode for 3-phase motors
    #          .The method refers to the Uart Write command: 0x06
    #@param  power_reference  a float value between 0 to 100       
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_power_reference(self, power_reference: float) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_power_reference_input_validation(power_reference)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        power_reference = float(power_reference)
        data = convert_to_data(power_reference, DATA_TYPE.SFXT)
        cmd = [self._address, ConstantUart.WritePowerReference,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  By putting 1 in the DATA section of a packet sent with this command, SOLO will start
    #        identifying the electrical parameters of the Motor connected
    #          .The method refers to the Uart Write command: 0x07
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
        data = convert_to_data(identification, DATA_TYPE.UINT32)
        cmd = [self._address, ConstantUart.WriteMotorParametersIdentification,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command if the DATA is set at zero will stop the whole power and switching system
    #        connected to the motor and it will cut the current floating into the Motor from SOLO 
    #          .The method refers to the Uart Write command: 0x08     
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def emergency_stop(self) -> list:
        cmd = [self._address, ConstantUart.WriteEmergencyStop,
               0x00, 0x00, 0x00, 0x00]
        self._logger.info(
            "SOLO should be manually power recycled to get back into normal operation ")
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command sets the output switching frequency of the whole power unit on the Motor
    #          .The method refers to the Uart Write command: 0x09
    #@param  output_pwm_frequency_khz  switching frequencies [kHz]      
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_output_pwm_frequency_khz(self, output_pwm_frequency_khz: int) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_output_pwm_frequency_khz_input_validation(output_pwm_frequency_khz)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        output_pwm_frequency_khz = int(output_pwm_frequency_khz)
        data = convert_to_data(output_pwm_frequency_khz, DATA_TYPE.UINT32)
        cmd = [self._address, ConstantUart.WriteOutputPwmFrequencyKhz,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command sets the Speed controller Kp Gain, and it will
    #        be functional only in Digital Closed-loop mode
    #          .The method refers to the Uart Write command: 0x0A
    #@param  speed_controller_kp  a float value between 0 to 300     
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_speed_controller_kp(self, speed_controller_kp: float) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_speed_controller_kp_input_validation(speed_controller_kp)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        speed_controller_kp = float(speed_controller_kp)
        data = convert_to_data(speed_controller_kp, DATA_TYPE.SFXT)
        cmd = [self._address, ConstantUart.WriteSpeedControllerKp,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command sets the Speed controller Ki gain, and it will
    #        be functional only in Digital Closed-loop mode
    #          .The method refers to the Uart Write command: 0x0B
    #@param  speed_controller_ki  a float value between 0 to 300      
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_speed_controller_ki(self, speed_controller_ki: float) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_speed_controller_ki_input_validation(speed_controller_ki)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        speed_controller_ki = float(speed_controller_ki)
        data = convert_to_data(speed_controller_ki, DATA_TYPE.SFXT)
        cmd = [self._address, ConstantUart.WriteSpeedControllerKi,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This commands sets the direction of the rotation of the motor
    #        either to ClockWise rotation or to Counter Clockwise Rotation
    #          .The method refers to the Uart Write command: 0x0C
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
        data = convert_to_data(motor_direction, DATA_TYPE.UINT32)
        cmd = [self._address, ConstantUart.WriteMotorDirection,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command sets the amount of the Phase or Armature resistance
    #        for 3-phase or DC Brushed motors respectively
    #          .The method refers to the Uart Write command: 0x0D
    #@param  motor_resistance  a float value [Ohm]    
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_motor_resistance(self, motor_resistance: float) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_motor_resistance_input_validation(motor_resistance)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        motor_resistance = float(motor_resistance)
        data = convert_to_data(motor_resistance, DATA_TYPE.SFXT)
        cmd = [self._address, ConstantUart.WriteMotorResistance,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command sets the amount of the Phase or Armature Inductance
    #        for 3-phase or DC Brushed motors respectively
    #          .The method refers to the Uart Write command: 0x0E
    #@param  motor_inductance  a float value [Henry]   
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_motor_inductance(self, motor_inductance: float) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_motor_inductance_input_validation(motor_inductance)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        motor_inductance = float(motor_inductance)
        data = convert_to_data(motor_inductance, DATA_TYPE.SFXT)
        cmd = [self._address, ConstantUart.WriteMotorInductance,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command sets the number of the Poles of a 3-phase motor commissioned with SOLO
    #          .The method refers to the Uart Write command: 0x0F
    #@param  motor_poles_counts  a long value between 1 to 254     
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_motor_poles_counts(self, motor_poles_counts: int) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_motor_poles_counts_input_validation(motor_poles_counts)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        motor_poles_counts = int(motor_poles_counts)
        data = convert_to_data(motor_poles_counts, DATA_TYPE.UINT32)
        cmd = [self._address, ConstantUart.WriteMotorPolesCounts,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command sets the pre-quad number of physical lines of an 
    #        incremental encoder engraved on its disk
    #          .The method refers to the Uart Write command: 0x10
    #@param  incremental_encoder_lines  a long value [pre-quad]    
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_incremental_encoder_lines(self, incremental_encoder_lines: int) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_incremental_encoder_lines_input_validation(incremental_encoder_lines)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        incremental_encoder_lines = int(incremental_encoder_lines)
        data = convert_to_data(incremental_encoder_lines, DATA_TYPE.UINT32)
        cmd = [self._address, ConstantUart.WriteIncrementalEncoderLines,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command sets the allowed speed during trajectory following
    #        in closed-loop position controlling mode
    #          .The method refers to the Uart Write command: 0x11
    #@param  speed_limit  a long value [RPM]     
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_speed_limit(self, speed_limit: int) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_speed_limit_input_validation(speed_limit)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        speed_limit = int(speed_limit)
        data = convert_to_data(speed_limit, DATA_TYPE.UINT32)
        cmd = [self._address, ConstantUart.WriteSpeedLimit,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command resets the device address of any connected SOLO to zero  
    #          .The method refers to the Uart Write command: 0x12  
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def reset_address(self) -> list:
        cmd = [0xFF, ConstantUart.WriteResetAddress, 0x00, 0x00, 0x00, 0xFF]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command sets the type of the feedback control SOLO has to operate
    #          .The method refers to the Uart Write command: 0x13
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
        data = convert_to_data(mode, DATA_TYPE.UINT32)
        cmd = [self._address, ConstantUart.WriteFeedbackControlMode,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command resets SOLO to its factory setting to all the default parameters 
    #          .The method refers to the Uart Write command: 0x14    
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def reset_factory(self) -> list:
        cmd = [self._address, ConstantUart.WriteResetFactory,
               0x00, 0x00, 0x00, 0x01]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command sets the Motor type that is connected to SOLO in Digital Mode
    #          .The method refers to the Uart Write command: 0x15
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
        data = convert_to_data(motor_type, DATA_TYPE.UINT32)
        cmd = [self._address, ConstantUart.WriteMotorType,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command sets the Control Mode in terms of Torque,
    #        Speed or Position only in Digital Mode
    #          .The method refers to the Uart Write command: 0x16
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
        data = convert_to_data(control_mode, DATA_TYPE.UINT32)
        cmd = [self._address, ConstantUart.WriteControlMode,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command sets the value for Current Controller Kp or proportional gain
    #          .The method refers to the Uart Write command: 0x17
    #@param  current_controller_kp  a float value between 0 to 16000   
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_current_controller_kp(self, current_controller_kp: float) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_current_controller_kp_input_validation(current_controller_kp)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        current_controller_kp = float(current_controller_kp)
        data = convert_to_data(current_controller_kp, DATA_TYPE.SFXT)
        cmd = [self._address, ConstantUart.WriteCurrentControllerKp,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command sets the value for Current Controller Ki or integral gain
    #          .The method refers to the Uart Write command: 0x18
    #@param  current_controller_ki  a float value between 0 to 16000   
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_current_controller_ki(self, current_controller_ki: float) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_current_controller_ki_input_validation(current_controller_ki)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        current_controller_ki = float(current_controller_ki)
        data = convert_to_data(current_controller_ki, DATA_TYPE.SFXT)
        cmd = [self._address, ConstantUart.WriteCurrentControllerKi,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    # @@ we don't have this function in arduino uart
    # def set_monitoring_mode(self, mode: int) -> list:
    #     if (mode < 0 or mode > 2):
    #         self._logger.info(ConstantCommon.InputOutOfRange)
    #         return False
    #     mode = int(mode)
    #     data = convert_to_data(mode, DATA_TYPE.UINT32)
    #     cmd = [self._address, ConstantUart.WriteMonitoringMode,
    #            data[0], data[1], data[2], data[3]]
    #     return self.__exec_cmd(cmd)

    ##
    #@brief  depending on the Motor type: in case of BLDC or PMSM motors Sets the Field
    #        Weakening current reference to help the motor reaching speeds higher than
    #        nominal values and in case of AC Induction Motors Sets the desired magnetizing
    #        current (Id) required for controlling ACIM motors in FOC in Amps 
    #          .The method refers to the Uart Write command: 0x1A
    #@param  magnetizing_current_id_reference  a float value [Amps]    
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_magnetizing_current_id_reference(self, magnetizing_current_id_reference: float) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_magnetizing_current_id_reference_input_validation(magnetizing_current_id_reference)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        magnetizing_current_id_reference = float(magnetizing_current_id_reference)
        data = convert_to_data(magnetizing_current_id_reference, DATA_TYPE.SFXT)
        cmd = [self._address, ConstantUart.WriteMagnetizingCurrentIdReference,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command sets the desired Position reference in terms of quadrature
    #        pulses while SOLO operates with the Incremental Encoders or in terms of
    #        pulses while while SOLO operates with Hall sensors
    #          .The method refers to the Uart Write command: 0x1B
    #@param  position_reference  a long value [Quad-Pulse]   
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_position_reference(self, position_reference: int) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_position_reference_input_validation(position_reference)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        position_reference = int(position_reference)
        data = convert_to_data(position_reference, DATA_TYPE.INT32)
        cmd = [self._address, ConstantUart.WritePositionReference,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command sets the value for Position Controller Kp or proportional gain 
    #          .The method refers to the Uart Write command: 0x1C
    #@param  position_controller_kp  a float value between 0 to 16000   
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_position_controller_kp(self, position_controller_kp: float) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_position_controller_kp_input_validation(position_controller_kp)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        position_controller_kp = float(position_controller_kp)
        data = convert_to_data(position_controller_kp, DATA_TYPE.SFXT)
        cmd = [self._address, ConstantUart.WritePositionControllerKp,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command sets the value for Position Controller Ki or integrator gain
    #          .The method refers to the Uart Write command: 0x1D
    #@param  position_controller_ki  a float value between 0 to 16000     
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_position_controller_ki(self, position_controller_ki: float) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_position_controller_ki_input_validation(position_controller_ki)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        position_controller_ki = float(position_controller_ki)
        data = convert_to_data(position_controller_ki, DATA_TYPE.SFXT)
        cmd = [self._address, ConstantUart.WritePositionControllerKi,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command resets the position counter back to zero
    #          .The method refers to the Uart Write command: 0x1F    
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def reset_position_to_zero(self) -> list:
        cmd = [self._address, ConstantUart.WriteResetPositionToZero,
                0x00, 0x00, 0x00, 0x01]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command overwrites the reported errors in Error Register
    #        reported with command code of "0xA1"   
    #          .The method refers to the Uart Write command: 0x20
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def overwrite_error_register(self) -> list:
        cmd = [self._address, ConstantUart.WriteOverwriteErrorRegister,
               0x00, 0x00, 0x00, 0x00]
        return self.__exec_cmd(cmd)

    # SOG => Sensorless Observer Gain
    
    ##
    #@brief  This command sets the observer gain for the Non-linear observer
    #        that estimates the speed and angle of a BLDC or PMSM once the 
    #        motor type is selected as normal BLDC-PMSM
    #          .The method refers to the Uart Write command: 0x21
    #@param  observer_gain  a float value between 0.01 to 1000    
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_observer_gain_bldc_pmsm(self, observer_gain: float) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_observer_gain_bldc_pmsm_input_validation(observer_gain)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        observer_gain = float(observer_gain)
        data = convert_to_data(observer_gain, DATA_TYPE.SFXT)
        cmd = [self._address, ConstantUart.WriteObserverGainBldcPmsm,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command sets the observer gain for the Non-linear observer that
    #        estimates the speed and angle of a BLDC or PMSM once the motor type
    #        is selected as ultra-fast BLDC-PMSM
    #          .The method refers to the Uart Write command: 0x22
    #@param  observer_gain  a float value between 0.01 to 1000    
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_observer_gain_bldc_pmsm_ultrafast(self, observer_gain: float) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_observer_gain_bldc_pmsm_ultrafast_input_validation(observer_gain)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        observer_gain = float(observer_gain)
        data = convert_to_data(observer_gain, DATA_TYPE.SFXT)
        cmd = [self._address, ConstantUart.WriteObserverGainBldcPmsmUltrafast,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command sets the observer gain for the Non-linear observer
    #        that estimates the speed of a DC brushed once the motor type 
    #        is selected as DC brushed
    #          .The method refers to the Uart Write command: 0x23
    #@param  observer_gain  a float value between 0.01 to 1000    
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_observer_gain_dc(self, observer_gain: float) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_observer_gain_dc_input_validation(observer_gain)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        observer_gain = float(observer_gain)
        data = convert_to_data(observer_gain, DATA_TYPE.SFXT)
        cmd = [self._address, ConstantUart.WriteObserverGainDc,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command sets how fast the observer should operate once
    #        SOLO is in sensorless mode with normal BLDC-PMSM selected as the Motor type
    #          .The method refers to the Uart Write command: 0x24
    #@param  filter_gain  a float value between 0.01 to 16000   
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_filter_gain_bldc_pmsm(self, filter_gain: float) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_filter_gain_bldc_pmsm_input_validation(filter_gain)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        filter_gain = float(filter_gain)
        data = convert_to_data(filter_gain, DATA_TYPE.SFXT)
        cmd = [self._address, ConstantUart.WriteFilterGainBldcPmsm,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command sets how fast the observer should operate once SOLO
    #           is in sensorless mode with ultra-fast BLDC-PMSM selected as the Motor type
    #           .The method refers to the Uart Write command: 0x25
    #@param  filterGain  a float value between 0.01 to 16000   
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_filter_gain_bldc_pmsm_ultrafast(self, filter_gain: float) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_filter_gain_bldc_pmsm_ultrafast_input_validation(filter_gain)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        filter_gain = float(filter_gain)
        data = convert_to_data(filter_gain, DATA_TYPE.SFXT)
        cmd = [self._address, ConstantUart.WriteFilterGainBldcPmsmUltrafast,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command sets the baud-rate of the UART line
    #          .The method refers to the Uart Write command: 0x26
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
        data = convert_to_data(baudrate, DATA_TYPE.UINT32)
        cmd = [self._address, ConstantUart.WriteUartBaudRate,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command starts or stops the process of sensor calibration
    #          .The method refers to the Uart Write command: 0x27
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
        data = convert_to_data(calibration_action, DATA_TYPE.UINT32)
        cmd = [self._address, ConstantUart.WriteSensorCalibration,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command sets the per-unit offset identified after sensor calibration
    #        for Encoder or Hall sensors in C.C.W direction
    #          .The method refers to the Uart Write command: 0x28
    #@param  encoder_hall_offset  a float value between 0.0 to 1.0   
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_encoder_hall_ccw_offset(self, encoder_hall_offset: float) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_encoder_hall_ccw_offset_input_validation(encoder_hall_offset)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        encoder_hall_offset = float(encoder_hall_offset)
        data = convert_to_data(encoder_hall_offset, DATA_TYPE.SFXT)
        cmd = [self._address, ConstantUart.WriteEncoderHallCcwOffset,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command sets the per-unit offset identified after sensor calibration
    #        for Encoder or Hall sensors in C.W direction
    #          .The method refers to the Uart Write command: 0x29
    #@param  encoder_hall_offset  a float value between 0.0 to 1.0     
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_encoder_hall_cw_offset(self, encoder_hall_offset: float) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_encoder_hall_cw_offset_input_validation(encoder_hall_offset)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        encoder_hall_offset = float(encoder_hall_offset)
        data = convert_to_data(encoder_hall_offset, DATA_TYPE.SFXT)
        cmd = [self._address, ConstantUart.WriteEncoderHallCwOffset,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command defines the acceleration value of the Speed for speed controller
    #        both in Analogue and Digital modes in Revolution per square seconds
    #          .The method refers to the Uart Write command: 0x2A
    #@param  speed_acceleration_value  a float value [Rev/S^2]  
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_speed_acceleration_value(self, speed_acceleration_value: float) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_speed_acceleration_value_input_validation(speed_acceleration_value)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        speed_acceleration_value = float(speed_acceleration_value)
        data = convert_to_data(speed_acceleration_value, DATA_TYPE.SFXT)
        cmd = [self._address, ConstantUart.WriteSpeedAccelerationValue,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command defines the deceleration value of the Speed for speed controller
    #        both in Analogue and Digital modes in Revolution per square seconds
    #          .The method refers to the Uart Write command: 0x2B
    #@param  speed_deceleration_value  a float value [Rev/S^2]     
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_speed_deceleration_value(self, speed_deceleration_value: float) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_speed_deceleration_value_input_validation(speed_deceleration_value)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        speed_deceleration_value = float(speed_deceleration_value)
        data = convert_to_data(speed_deceleration_value, DATA_TYPE.SFXT)
        cmd = [self._address, ConstantUart.WriteSpeedDecelerationValue,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command sets the baud rate of CAN bus in CANOpen network
    #          .The method refers to the Uart Write command: 0x2C
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
        data = convert_to_data(canbus_baudrate, DATA_TYPE.UINT32)
        cmd = [self._address, ConstantUart.WriteCanBusBaudRate,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)
        
    ##
    #@brief  This command defines the resolution of the speed at S/T input
    #          while SOLO operates in Analogue mode
    #          .The method refers to the Uart Write command: 0x2D
    #@param  division_coefficient  a float value     
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_analogue_speed_resolution_division_coefficient(self, division_coefficient: float) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_analogue_speed_resolution_division_coefficient_input_validation(division_coefficient)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        division_coefficient = float(division_coefficient)
        data = convert_to_data(division_coefficient, DATA_TYPE.SFXT)
        cmd = [self._address, ConstantUart.WriteASRDC,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command defines the type of the Motion Profile that is 
    #          being used in Speed or Position Modes
    #          .The method refers to the Uart Write command: 0x30
    #@param  motion_profile_mode enum that specify the type of the Motion Profile    
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    #
    def set_motion_profile_mode(self, motion_profile_mode: MOTION_PROFILE_MODE) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_motion_profile_mode_input_validation(motion_profile_mode)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        
        if (type(motion_profile_mode) is int):
            motion_profile_mode = MOTION_PROFILE_MODE(motion_profile_mode)

        motion_profile_mode = motion_profile_mode.value
        data = convert_to_data(motion_profile_mode, DATA_TYPE.UINT32)
        cmd = [self._address, ConstantUart.WriteMotionProfileMode,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)
        
    ##
    #@brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
    #          .The method refers to the Uart Write command: 0x31  
    #@param  motion_profile_variable1 a float value     
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_motion_profile_variable1(self, motion_profile_variable1: float) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_motion_profile_variable1_input_validation(motion_profile_variable1)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        motion_profile_variable1 = float(motion_profile_variable1)
        data = convert_to_data(motion_profile_variable1, DATA_TYPE.SFXT)
        cmd = [self._address, ConstantUart.WriteMotionProfileVariable1,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)
        
    ##
    #@brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
    #          .The method refers to the Uart Write command: 0x32 
    #@param  motion_profile_variable2 a float value     
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_motion_profile_variable2(self, motion_profile_variable2: float) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_motion_profile_variable2_input_validation(motion_profile_variable2)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        motion_profile_variable2 = float(motion_profile_variable2)
        data = convert_to_data(motion_profile_variable2, DATA_TYPE.SFXT)
        cmd = [self._address, ConstantUart.WriteMotionProfileVariable2,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)
        
    ##
    #@brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
    #          .The method refers to the Uart Write command: 0x33
    #@param  motion_profile_variable3 a float value     
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_motion_profile_variable3(self, motion_profile_variable3: float) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_motion_profile_variable3_input_validation(motion_profile_variable3)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        motion_profile_variable3 = float(motion_profile_variable3)
        data = convert_to_data(motion_profile_variable3, DATA_TYPE.SFXT)
        cmd = [self._address, ConstantUart.WriteMotionProfileVariable3,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)
        
    ##
    #@brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
    #          .The method refers to the Uart Write command: 0x34
    #@param  motion_profile_variable4 a float value     
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_motion_profile_variable4(self, motion_profile_variable4: float) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_motion_profile_variable4_input_validation(motion_profile_variable4)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        motion_profile_variable4 = float(motion_profile_variable4)
        data = convert_to_data(motion_profile_variable4, DATA_TYPE.SFXT)
        cmd = [self._address, ConstantUart.WriteMotionProfileVariable4,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)
        
    ##
    #@brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
    #          .The method refers to the Uart Write command: 0x35
    #@param  motion_profile_variable5 a float value     
    #@retval List of [bool 0 fail / 1 for success, ERROR class/enumeration]
    def set_motion_profile_variable5(self, motion_profile_variable5: float) -> list:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_motion_profile_variable5_input_validation(motion_profile_variable5)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error
        motion_profile_variable5 = float(motion_profile_variable5)
        data = convert_to_data(motion_profile_variable5, DATA_TYPE.SFXT)
        cmd = [self._address, ConstantUart.WriteMotionProfileVariable5,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

# ----------------------------------------------
# ---------------------Read---------------------
# ----------------------------------------------

    ##
    #@brief  This command reads the device address connected on the line 
    #          .The method refers to the Uart Read command: 0x81   
    #@retval  List of [long device address connected on the line, ERROR class/enumeration]
    def get_device_address(self) -> list:
        cmd = [0xFF, ConstantUart.ReadDeviceAddress, 0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.UINT32), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  This command reads the phase-A voltage of the motor connected to the
    #        "A" pin output of SOLO for 3-phase Motors   
    #          .The method refers to the Uart Read command: 0x82
    #@retval  List of [float phase-A voltage of the motor [Volts], ERROR class/enumeration]
    def get_phase_a_voltage(self) -> list:
        cmd = [self._address, ConstantUart.ReadPhaseAVoltage, 0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  This command reads the phase-B voltage of the motor connected to the
    #        "B" pin output of SOLO for 3-phase Motors  
    #          .The method refers to the Uart Read command: 0x83
    #@retval  List of [float 0 phase-A voltage of the motor [Volts], ERROR class/enumeration]
    def get_phase_b_voltage(self) -> list:
        cmd = [self._address, ConstantUart.ReadPhaseBVoltage, 0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  This command reads the phase-A current of the motor connected to the
    #        "A" pin output of SOLO for 3-phase Motors 
    #          .The method refers to the Uart Read command: 0x84
    #@retval  List of [float phase-A current of the motor [Amps], ERROR class/enumeration]
    def get_phase_a_current(self) -> list:
        cmd = [self._address, ConstantUart.ReadPhaseACurrent, 0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  This command reads the phase-B current of the motor connected to the
    #        "B" pin output of SOLO for 3-phase Motors  
    #          .The method refers to the Uart Read command: 0x85
    #@retval  List of [float phase-B current of the motor [Amps], ERROR class/enumeration]
    def get_phase_b_current(self) -> list:
        cmd = [self._address, ConstantUart.ReadPhaseBCurrent, 0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  This command reads the input BUS voltage   
    #          .The method refers to the Uart Read command: 0x86 
    #@retval  List of [float  BUS voltage [Volts], ERROR class/enumeration]
    def get_bus_voltage(self) -> list:
        cmd = [self._address, ConstantUart.ReadBusVoltage, 0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  This command reads the current inside the DC brushed motor connected to
    #        "B" and "C" outputs of SOLO  
    #          .The method refers to the Uart Read command: 0x87
    #@retval  List of [float between [Amps], ERROR class/enumeration]
    def get_dc_motor_current_im(self) -> list:
        cmd = [self._address, ConstantUart.ReadDcMotorCurrentIm,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  This command reads the voltage of the DC brushed motor connected to
    #        "B" and "C" outputs of SOLO  
    #          .The method refers to the Uart Read command: 0x88
    #@retval  List of [float [Volts], ERROR class/enumeration]
    def get_dc_motor_voltage_vm(self) -> list:
        cmd = [self._address, ConstantUart.ReadDcMotorVoltageVm,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  This command reads the value of the Speed controller Kp gain, 
    #        set for Digital mode operations   
    #          .The method refers to the Uart Read command: 0x89
    #@retval  List of [float between 0 to 16000, ERROR class/enumeration]
    def get_speed_controller_kp(self) -> list:
        cmd = [self._address, ConstantUart.ReadSpeedControllerKp,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  This command reads the value of the Speed controller Ki gain,
    #        set for Digital mode operations   
    #          .The method refers to the Uart Read command: 0x8A
    #@retval  List of [float between 0 to 16000, ERROR class/enumeration]
    def get_speed_controller_ki(self) -> list:
        cmd = [self._address, ConstantUart.ReadSpeedControllerKi,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  This command reads the output switching frequency of SOLO in Hertz 
    #          .The method refers to the Uart Read command: 0x8B  
    #@retval  List of [long [Hz], ERROR class/enumeration]
    def get_output_pwm_frequency_khz(self) -> list:
        cmd = [self._address, ConstantUart.ReadOutputPwmFrequencyHz,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return (convert_from_data(data, DATA_TYPE.UINT32) / 1000), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  This command reads the value of the current limit set for SOLO in
    #        closed-loop digital operation mode   
    #          .The method refers to the Uart Read command: 0x8C
    #@retval  List of [float [Amps], ERROR class/enumeration]
    def get_current_limit(self) -> list:
        cmd = [self._address, ConstantUart.ReadCurrentLimit,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  This command reads the actual monetary value of “Iq” that is
    #        the current acts in torque generation in FOC mode for 3-phase motors 
    #          .The method refers to the Uart Read command: 0x8D 
    #@retval  List of [float [Amps], ERROR class/enumeration]
    def get_quadrature_current_iq_feedback(self) -> list:
        cmd = [self._address, ConstantUart.ReadQuadratureCurrentIqFeedback,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  This command reads the actual monetary value of Id that is the
    #        direct current acting in FOC  
    #          .The method refers to the Uart Read command: 0x8E
    #@retval  List of [float [Amps], ERROR class/enumeration]
    def get_magnetizing_current_id_feedback(self) -> list:
        cmd = [self._address, ConstantUart.ReadMagnetizingCurrentIdFeedback,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  This command reads the number of Poles set for 3-phase motors 
    #          .The method refers to the Uart Read command: 0x8F  
    #@retval  List of [long between 1 to 254, ERROR class/enumeration]
    def get_motor_poles_counts(self) -> list:
        cmd = [self._address, ConstantUart.ReadMotorPolesCounts,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.UINT32), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  This command reads the number of physical Incremental encoder lines set on SOLO   
    #          .The method refers to the Uart Read command: 0x90
    #@retval  List of [long between 1 to 200000, ERROR class/enumeration]
    def get_incremental_encoder_lines(self) -> list:
        cmd = [self._address, ConstantUart.ReadIncrementalEncoderLines,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.UINT32), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  This command reads the amount of value set for Current controller
    #        Kp or proportional gain  
    #          .The method refers to the Uart Read command: 0x91
    #@retval  List of [float between 0 to 16000, ERROR class/enumeration]
    def get_current_controller_kp(self) -> list:
        cmd = [self._address, ConstantUart.ReadCurrentControllerKp,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  This command reads the amount of value set for Current controller
    #        Ki or integrator gain    
    #          .The method refers to the Uart Read command: 0x92
    #@retval  List of [float between 0 to 16000, ERROR class/enumeration]
    def get_current_controller_ki(self) -> list:
        cmd = [self._address, ConstantUart.ReadCurrentControllerKi,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT) * 0.00005, ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  This command reads the momentary temperature of the board in centigrade  
    #          .The method refers to the Uart Read command: 0x93
    #@retval  List of [float [°C], ERROR class/enumeration]
    def get_board_temperature(self) -> list:
        cmd = [self._address, ConstantUart.ReadBoardTemperature,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  This command reads the Phase or Armature resistance of
    #        the 3-phase or DC brushed motor connected to SOLO respectively  
    #          .The method refers to the Uart Read command: 0x94 
    #@retval  List of [float [Ohms], ERROR class/enumeration]
    def get_motor_resistance(self) -> list:
        cmd = [self._address, ConstantUart.ReadMotorResistance,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT) * 0.00005, ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  This command reads the Phase or Armature Inductance of 
    #        the 3-phase or DC brushed motor connected to SOLO respectively 
    #          .The method refers to the Uart Read command: 0x95  
    #@retval  List of [float [Henry], ERROR class/enumeration]
    def get_motor_inductance(self) -> list:
        cmd = [self._address, ConstantUart.ReadMotorInductance,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  his command reads the actual speed of the motor measured or estimated by SOLO in
    #        sensorless or sensor-based modes respectively   
    #          .The method refers to the Uart Read command: 0x96
    #@retval  List of [long [RPM], ERROR class/enumeration]
    def get_speed_feedback(self) -> list:
        cmd = [self._address, ConstantUart.ReadSpeedFeedback,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.INT32), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  This command reads the Motor type selected for Digital or Analogue mode operations 
    #          .The method refers to the Uart Read command: 0x97  
    #@retval  List of [long between 0 to 3, ERROR class/enumeration]
    def get_motor_type(self) -> list:
        cmd = [self._address, ConstantUart.ReadMotorType,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return MOTOR_TYPE(convert_from_data(data, DATA_TYPE.UINT32)), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  This command reads the feedback control mode selected on SOLO both
    #        for Analogue and Digital operations    
    #          .The method refers to the Uart Read command: 0x99
    #@retval  List of [long between 0 to 2, ERROR class/enumeration]
    def get_feedback_control_mode(self) -> list:
        cmd = [self._address, ConstantUart.ReadFeedbackControlMode,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return FEEDBACK_CONTROL_MODE(convert_from_data(data, DATA_TYPE.UINT32)), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  This command reads the actual commanding mode that SOLO is operating 
    #          .The method refers to the Uart Read command: 0x9A 
    #@retval  List of [long between 0 or 1, ERROR class/enumeration]
    def get_command_mode(self) -> list:
        cmd = [self._address, ConstantUart.ReadCommandMode,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return COMMAND_MODE(convert_from_data(data, DATA_TYPE.UINT32)), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  This command reads the Control Mode type in terms of Torque,
    #        Speed or Position in both Digital and Analogue modes  
    #          .The method refers to the Uart Read command: 0x9B
    #@retval  List of [long between 0 to 2, ERROR class/enumeration]
    def get_control_mode(self) -> list:
        cmd = [self._address, ConstantUart.ReadControlMode,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return CONTROL_MODE(convert_from_data(data, DATA_TYPE.UINT32)), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  This command reads the value of the speed limit set on SOLO  
    #          .The method refers to the Uart Read command: 0x9C
    #@retval  List of [long [RPM], ERROR class/enumeration]
    def get_speed_limit(self) -> list:
        cmd = [self._address, ConstantUart.ReadSpeedLimit,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.UINT32), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error


    ##
    #@brief  This command reads the amount of value set for Position
    #        controller Kp or proportional gain   
    #          .The method refers to the Uart Read command: 0x9D
    #@retval  List of [float between 0 to 16000, ERROR class/enumeration]
    def get_position_controller_kp(self) -> list:
        cmd = [self._address, ConstantUart.ReadPositionControllerKp,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  This command reads the amount of value set for Position
    #        controller Ki or integrator gain   
    #          .The method refers to the Uart Read command: 0x9E
    #@retval  List of [float between 0 to 16000, ERROR class/enumeration]
    def get_position_controller_ki(self) -> list:
        cmd = [self._address, ConstantUart.ReadPositionControllerKi,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  This command reads the number of counted pulses from the
    #        Incremental Encoder or Hall sensors  
    #          .The method refers to the Uart Read command: 0xA0
    #@retval  List of [long [Quad-Pulses], ERROR class/enumeration]
    def get_position_counts_feedback(self) -> list:
        cmd = [self._address, ConstantUart.ReadPositionCountsFeedback,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.INT32), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  This command reads the error register which is a 32 bit register with
    #        each bit corresponding to specific errors   
    #          .The method refers to the Uart Read command: 0xA1
    #@retval  List of [long , ERROR class/enumeration]
    def get_error_register(self) -> list:
        cmd = [self._address, ConstantUart.ReadErrorRegister,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.UINT32), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  This command reads the Firmware version existing currently on the SOLO unit   
    #          .The method refers to the Uart Read command: 0xA2
    #@retval  List of [long, ERROR class/enumeration]
    def get_device_firmware_version(self) -> list:
        cmd = [self._address, ConstantUart.ReadDeviceFirmwareVersion,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.UINT32), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  This command reads the Hardware version of the SOLO unit connected    
    #          .The method refers to the Uart Read command: 0xA3 
    #@retval  List of [long, ERROR class/enumeration]
    def get_device_hardware_version(self) -> list:
        cmd = [self._address, ConstantUart.ReadDeviceHardwareVersion,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.UINT32), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  This command reads the amount of desired Torque reference (Iq or IM)
    #        already set for the Motor to follow in Digital Closed-loop Torque control mode  
    #          .The method refers to the Uart Read command: 0xA4 
    #@retval  List of [float [Amps], ERROR class/enumeration]
    def get_torque_reference_iq(self) -> list:
        cmd = [self._address, ConstantUart.ReadTorqueReferenceIq,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  This command reads the amount of desired Speed reference already set for
    #        the Motor to follow in Digital Closed-loop Speed control mode   
    #          .The method refers to the Uart Read command: 0xA5
    #@retval  List of [long [RPM], ERROR class/enumeration]
    def get_speed_reference(self) -> list:
        cmd = [self._address, ConstantUart.ReadSpeedReference,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.UINT32), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  This command reads the amount of desired Id (direct current) or
    #        Magnetizing current reference already set for the Motor to follow
    #        in Digital Closed-loop Speed control mode for ACIM motors  
    #          .The method refers to the Uart Read command: 0xA6
    #@retval  List of [float [Amps], ERROR class/enumeration]
    def get_magnetizing_current_id_reference(self) -> list:
        cmd = [self._address, ConstantUart.ReadMagnetizingCurrentIdReference,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  This command reads the desired position reference set for the Motor
    #        to follow in Digital Closed-loop Position mode in terms of quadrature pulses 
    #          .The method refers to the Uart Read command: 0xA7 
    #@retval  List of [long [Quad-Pulses], ERROR class/enumeration]
    def get_position_reference(self) -> list:
        cmd = [self._address, ConstantUart.ReadPositionReference,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.INT32), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  This command reads the desired Power reference for SOLO to apply in 
    #        Digital Open-loop speed control mode for 3-phase motors in terms of percentage
    #          .The method refers to the Uart Read command: 0xA8
    #@retval  List of [float [%], ERROR class/enumeration]
    def get_power_reference(self) -> list:
        cmd = [self._address, ConstantUart.ReadPowerReference,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  This commands reads the desired direction of rotation set for the Motor   
    #          .The method refers to the Uart Read command: 0xA9
    #@retval  List of [long 0 Counter ClockWise / 1 ClockWise, ERROR class/enumeration]
    def get_motor_direction(self) -> list:
        cmd = [self._address, ConstantUart.ReadMotorDirection,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return DIRECTION(convert_from_data(data, DATA_TYPE.UINT32)), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  This command reads the value of Sensorless Observer Gain for Normal BLDC-PMSM Motors  
    #          .The method refers to the Uart Read command: 0xAA 
    #@retval  List of [float between 0.01 to 1000, ERROR class/enumeration]
    def get_observer_gain_bldc_pmsm(self) -> list:
        cmd = [self._address, ConstantUart.ReadObserverGainBldcPmsm,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  This command reads the value of Sensorless Observer Gain for Normal BLDC-PMSM Motors  
    #          .The method refers to the Uart Read command: 0xAB
    #@retval  List of [float between 0.01 to 1000, ERROR class/enumeration]
    def get_observer_gain_bldc_pmsm_ultrafast(self) -> list:
        cmd = [self._address, ConstantUart.ReadObserverGainBldcPmsmUltrafast,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  This command reads the value of Sensorless Observer Gain for DC Motor  
    #          .The method refers to the Uart Read command: 0xAC 
    #@retval  List of [float between 0.01 to 1000, ERROR class/enumeration]
    def get_observer_gain_dc(self) -> list:
        cmd = [self._address, ConstantUart.ReadObserverGainDc,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  This command reads the value of Sensorless Observer
    #        Filter Gain for Normal BLDC-PMSM Motors    
    #          .The method refers to the Uart Read command: 0xAD
    #@retval  List of [float between 0.01 to 16000, ERROR class/enumeration]
    def get_filter_gain_bldc_pmsm(self) -> list:
        cmd = [self._address, ConstantUart.ReadFilterGainBldcPmsm,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  This command reads the value of Sensorless Observer
    #        Filter Gain for Ultra Fast BLDC-PMSM Motors  
    #          .The method refers to the Uart Read command: 0xAE 
    #@retval  List of [float between 0.01 to 16000, ERROR class/enumeration]
    def get_filter_gain_bldc_pmsm_ultrafast(self) -> list:
        cmd = [self._address, ConstantUart.ReadFilterGainBldcPmsmUltrafast,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  This command reads the measured or estimated per-unit angle of the 3-phase motors   
    #          .The method refers to the Uart Read command: 0xB0
    #@retval  List of [float [Per Unit], ERROR class/enumeration]
    def get_3phase_motor_angle(self) -> list:
        cmd = [self._address, ConstantUart.Read3PhaseMotorAngle,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  This command reads the per-unit Encoder or Hall sensor offset in C.C.W direction    
    #          .The method refers to the Uart Read command: 0xB1
    #@retval  List of [float [Per Unit], ERROR class/enumeration]
    def get_encoder_hall_ccw_offset(self) -> list:
        cmd = [self._address, ConstantUart.ReadEncoderHallCcwOffset,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  This command reads the per-unit Encoder or Hall sensor offset in C.C.W direction 
    #          .The method refers to the Uart Read command: 0xB2  
    #@retval  List of [float [Per Unit], ERROR class/enumeration]
    def get_encoder_hall_cw_offset(self) -> list:
        cmd = [self._address, ConstantUart.ReadEncoderHallCwOffset,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  This command reads Baud Rate selected on SOLO unit to communicate through UART line   
    #          .The method refers to the Uart Read command: 0xB3
    #@retval  List of [long [Bits/s], ERROR class/enumeration]
    def get_uart_baudrate(self) -> list:
        cmd = [self._address, ConstantUart.ReadUartBaudRate,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return UART_BAUD_RATE(convert_from_data(data, DATA_TYPE.UINT32)), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  This command reads the acceleration value of the Speed for
    #        speed controller both in Analogue and Digital modes
    #        in Revolution per square seconds  
    #          .The method refers to the Uart Read command: 0xB4
    #@retval  List of [float [Rev/S^2], ERROR class/enumeration]
    def get_speed_acceleration_value(self) -> list:
        cmd = [self._address, ConstantUart.ReadSpeedAccelerationValue,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  This command reads the deceleration value of the Speed for
    #        speed controller both in Analogue and Digital modes
    #        in Revolution per square seconds  
    #          .The method refers to the Uart Read command: 0xB5
    #@retval  List of [float [Rev/S^2], ERROR class/enumeration]
    def get_speed_deceleration_value(self) -> list:
        cmd = [self._address, ConstantUart.ReadSpeedDecelerationValue,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
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
    #          .The method refers to the Uart Read command: 0xB8 
    #@retval  List of [long [Pulses], ERROR class/enumeration]
    def get_encoder_index_counts(self) -> list:
        cmd = [self._address, ConstantUart.ReadEncoderIndexCounts,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.UINT32), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_can_bus_baudrate(self) -> list:
        cmd = [self._address, ConstantUart.ReadCanBusBaudRate,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return CAN_BUS_BAUD_RATE(convert_from_data(data, DATA_TYPE.UINT32)), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error
            
    ##
    #@brief  This command reads the Analogue Speed Resolution Division Coefficient (ASRDC)
    #          while SOLO operates in Analogue mode
    #          .The method refers to the Uart Write command: 0xB7     
    #@retval  List of [Analogue Speed Resolution Division Coefficient, ERROR class/enumeration]
    def get_analogue_speed_resolution_division_coefficient(self) -> list:
        cmd = [self._address, ConstantUart.ReadASRDC,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    ##
    #@brief  This command reads the type of the Embedded Motion profile active in the controller 
    #          being used in Speed or Position Modes
    #          .The method refers to the Uart Write command: 0xBB    
    #@retval  List of [int value of Motion profile, ERROR class/enumeration]
    def get_motion_profile_mode(self) -> list:
        cmd = [self._address, ConstantUart.ReadMotionProfileMode,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return MOTION_PROFILE_MODE(convert_from_data(data, DATA_TYPE.UINT32)), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error
        
    ##
    #@brief  This command reads the value of the Motion Profile Variable1 set inside the controller 
    #          .The method refers to the Uart Write command: 0xBC      
    #@retval  List of [Motion Profile Variable1, ERROR class/enumeration]
    def get_motion_profile_variable1(self) -> list:
        cmd = [self._address, ConstantUart.ReadMotionProfileVariable1,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error
        
    ##
    #@brief  This command reads the value of the Motion Profile Variable2 set inside the controller 
    #          .The method refers to the Uart Write command: 0xBD    
    #@retval  List of [Motion Profile Variable2, ERROR class/enumeration]
    def get_motion_profile_variable2(self) -> list:
        cmd = [self._address, ConstantUart.ReadMotionProfileVariable2,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error
        
    ##
    #@brief  This command reads the value of the Motion Profile Variable3 set inside the controller 
    #          .The method refers to the Uart Write command: 0xBE   
    #@retval  List of [Motion Profile Variable3, ERROR class/enumeration]
    def get_motion_profile_variable3(self) -> list:
        cmd = [self._address, ConstantUart.ReadMotionProfileVariable3,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error
        
    ##
    #@brief  This command reads the value of the Motion Profile Variable4 set inside the controller 
    #          .The method refers to the Uart Write command: 0xBF    
    #@retval List of [ Motion Profile Variable4, ERROR class/enumeration]
    def get_motion_profile_variable4(self) -> list:
        cmd = [self._address, ConstantUart.ReadMotionProfileVariable4,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error
        
    ##
    #@brief  This command reads the value of the Motion Profile Variable5 set inside the controller 
    #          .The method refers to the Uart Write command: 0xC0   
    #@retval List of [ Motion Profile Variable5, ERROR class/enumeration]
    def get_motion_profile_variable5(self) -> list:
        cmd = [self._address, ConstantUart.ReadMotionProfileVariable5,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error
