## @package SOLOMotorControllersUart.py
#  @author  SOLOMotorControllers
#  @brief   This file contains all the functions for the Solo Uart Drivers 
#           Availability: https://github.com/Solo-FL/SoloPy/tree/main/SoloPy
#  @date    Date: 2025
#  @version 4.0.0

## @attention
# Copyright: (c) 2021-present, SOLO motor controllers project
# MIT License (see LICENSE file for more details)

import logging
import string
import time
from importlib import reload
#from interface import implements
import serial

import SoloPy.ConstantUart as ConstantUart
from SoloPy.SOLOMotorControllers import *
from SoloPy.SOLOMotorControllersUtils import *

#class SoloMotorControllerUart(implements(SOLOMotorControllers)):
class SoloMotorControllerUart:

    def __init__(
            self,
            port="/dev/ttyS0",
            address=0,
            baudrate=UartBaudRate.RATE_937500,
            timeout=100/1000,
            autoConnect = True,
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
        if baudrate == UartBaudRate.RATE_937500:
            self._baudrate = 937500
        if baudrate == UartBaudRate.RATE_115200:
            self._baudrate = 115200
        self._port = port
        self._timeout = timeout
        self._ser_status = 0
        self._ser = None
        if autoConnect:
            self.connect()
        # TODO try solving serial error

    def __del__(self):
        self._logger.debug('SoloMotorController DEL')
        if self._ser_status == 1:
            self._logger.debug('Serial close')
            try:
                self._ser.close()
            except Exception as e:
                self._logger.error("Exception on Serial Closure")
                self._logger.error( e, exc_info=True)

    def __exec_cmd(self, cmd: list) -> Tuple[bool, Error]:
        if self._ser_status == -1:
            self.connect()
        if self._ser_status == 1:
            try:
                message_log = "Serial connection is: " + str(self._ser.isOpen())
                self._logger.debug(message_log)
                if not self._ser.isOpen():
                    self._logger.debug("Serial open")
                    self._ser.open()
                    self._logger.debug("Serial flush")
                    self._ser.flush()
                    self._ser.flushInput()
                    self._ser_status = 1

                _cmd = [ConstantUart.INITIATOR, ConstantUart.INITIATOR,
                        cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], 
                        ConstantUart.CRC, ConstantUart.ENDING]

                _read_packet = []

                message_log = "WRITE: " + str([
                    hex(ConstantUart.INITIATOR), hex(ConstantUart.INITIATOR),
                    hex(cmd[0]), hex(cmd[1]), hex(cmd[2]), hex(
                        cmd[3]), hex(cmd[4]), hex(cmd[5]),
                    hex(ConstantUart.CRC), hex(ConstantUart.ENDING)])
                self._logger.debug(message_log)

                self._ser.write(_cmd)

                # Time to ensure the writing, reducing it can make the comunication instable
                time.sleep(0.1)

                # read up to ten bytes (timeout)
                _read_packet = self._ser.read(10)
                message_log = "READ: " + \
                    str([hex(i) for i in _read_packet]) + \
                    " size: " + str(len(_read_packet))
                self._logger.debug(message_log)
                if len(_read_packet) == 0:
                    self._ser.flush()

                if (_read_packet and
                        _read_packet[0] == _cmd[0] and
                        _read_packet[1] == _cmd[1] and
                        (_read_packet[2] == _cmd[2] or _cmd[2] == 0xFF) and
                        _read_packet[3] == _cmd[3] and
                        _read_packet[8] == _cmd[8] and
                        _read_packet[9] == _cmd[9]):
                    cmd[0] = _read_packet[2]
                    cmd[1] = _read_packet[3]
                    cmd[2] = _read_packet[4]
                    cmd[3] = _read_packet[5]
                    cmd[4] = _read_packet[6]
                    cmd[5] = _read_packet[7]
                else:
                    cmd[0] = 0xEE
                    cmd[1] = 0xEE
                    cmd[2] = 0xEE
                    cmd[3] = 0xEE
                    cmd[4] = 0xEE
                    cmd[5] = 0xEE

                if (cmd[2] == ConstantUart.Error and
                    cmd[3] == ConstantUart.Error and
                    cmd[4] == ConstantUart.Error and
                        cmd[5] == ConstantUart.Error):
                    return False, Error.GENERAL_ERROR
                else:
                    return True, Error.NO_ERROR_DETECTED
            except Exception as e:
                self._logger.debug('__exec_cmd Exception')
                self._logger.debug(e, exc_info=True)
                self.serial_error_handler()
                return False, Error.GENERAL_ERROR

        self._logger.info("Serial status not ready")
        # self.serial_open()
        return False, Error.GENERAL_ERROR

    # #############################Support############################# #
    def connect(self) -> bool:
        self._logger.debug("connect function start")
        try:
            self._ser = serial.Serial(
                self._port, self._baudrate, timeout=self._timeout, writeTimeout=self._timeout)
            self._ser_status = 1

            # Time sleep for ensure serial initialization
            time.sleep(0.2)

        except serial.serialutil.SerialException as e:
            self._logger.error(
                "connect: Exception during the serial inizialisation")
            self._logger.error( e, exc_info=True)
            self._ser_status = -1
            # raise e
            return False

        if self._ser_status == 1:
            self._logger.debug("Serial init")
            self._ser.bytesize = serial.EIGHTBITS
            self._ser.parity = serial.PARITY_NONE
            self._ser.stopbits = serial.STOPBITS_ONE
            time.sleep(0.2)
            if not self._ser.isOpen():
                self._logger.debug("Serial open")
                self._ser.open()

            self._ser.flush()
            self._ser.flushInput()
            self._logger.info("connect: success")
            self._logger.debug("connect function end")
            return True

        self._logger.debug("serial_open end")
        return False

    def serial_error_handler(self) -> bool:
        """Handles errors in serial port."""
        self._logger.debug('SEH start')
        try:
            try:
                if self._ser.isOpen():
                    self._logger.debug('SEH: serial is open')
                    if self._ser.inWaiting() > 0:
                        self._logger.debug('SEH: clean buffers')
                        self._ser.flushInput()
                        self._ser.flushOutput()

                    self._logger.debug('SEH: serial close')
                    self._ser.close()
                    self._ser_status = 0

            except Exception as e:
                self._logger.error( e, exc_info=True)

            self._logger.debug('SEH: serial init')
            res = self.serial_open()
            self._logger.debug('SEH end')
            return res

        except Exception as e:
            self._logger.error("SEH: Exception")
            self._logger.error( e, exc_info=True)
            self._ser_status = -1
            self._logger.debug('SEH end')
            return False

    def serial_close(self) -> bool:
        """Close the serial port."""
        self._logger.debug("serial_close start")

        if self._ser_status == 1:
            try:
                self._ser.close()
                self._logger.info("serial_close: Serial closed")
                self._ser_status = 0
                return True
            except Exception as e:
                self._logger.error("serial_close: Exception on Serial Closure")
                self._logger.error( e, exc_info=True)
                self._ser_status = -1
                return False

        if self._ser_status == 0:
            self._logger.info("serial_close: Serial is already close")
        if self._ser_status == -1:
            self._logger.info("serial_close: Serial is in error")
        self._logger.debug("serial_close end")
        return False

    def serial_status(self) -> string:
        """Determine status of serial port."""
        status = ''
        if self._ser_status == 1:
            status += 'SPS: Serial Open'
            try:
                status += '\nSPS: Serial comunication open: ' + \
                    str(self._ser.isOpen())
                if self._ser.isOpen():
                    status += '\nSPS: Serial input buffer size: ' + \
                        str(self._ser.out_waiting)
                    status += '\nSPS: Serial output buffer size: ' + \
                        str(self._ser.inWaiting())
            except Exception as e:
                self._logger.error('exception on printing the status')
                self._logger.error( e, exc_info=True)

        if self._ser_status == 0:
            status += 'SPS: Serial Close'

        if self._ser_status == -1:
            status += 'SPS: Serial on Error'
        return status

    def serial_is_working(self) -> bool:
        """Determine Communication on serial port."""
        if self.get_phase_a_voltage() == -1:
            return False
        return True

    # #############################Write############################# #


    ##
    #@brief  This command sets the desired device address for a SOLO unit
    #          .The method refers to the Uart Write command: 0x01
    #@param  device_address  address want to set for board
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_device_address(self, device_address: int) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_device_address_input_validation(device_address)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error

        cmd = [self._address, ConstantUart.WRITE_DEVICE_ADDRESS,
               0x00, 0x00, 0x00, device_address]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command sets the mode of the operation of SOLO
    #        in terms of operating in Analogue mode or Digital
    #          .The method refers to the Uart Write command: 0x02
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
        data = convert_to_data(mode, DataType.UINT32)
        cmd = [self._address, ConstantUart.WRITE_COMMAND_MODE,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command defines the maximum allowed current into the motor in terms of Amps
    #          .The method refers to the Uart Write command: 0x03
    #@param  current_limit  a float value [Amps]
    # #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_current_limit(self, current_limit: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_current_limit_input_validation(current_limit)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error

        current_limit = float(current_limit)
        data = convert_to_data(current_limit, DataType.SFXT)
        cmd = [self._address, ConstantUart.WRITE_CURRENT_LIMIT,
               data[0], data[1], data[2], data[3]]

        return self.__exec_cmd(cmd)

    ##
    #@brief  This command sets the amount of desired current that acts in torque generation
    #          .The method refers to the Uart Write command: 0x04
    #@param  torque_reference_iq  a float [Amps]
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_torque_reference_iq(self, torque_reference_iq: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_torque_reference_iq_input_validation(torque_reference_iq)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error

        torque_reference_iq = float(torque_reference_iq)
        data = convert_to_data(torque_reference_iq, DataType.SFXT)
        cmd = [self._address, ConstantUart.WRITE_TORQUE_REFERENCE_IQ,
               data[0], data[1], data[2], data[3]]

        return self.__exec_cmd(cmd)

    ##
    #@brief  This command defines the speed reference for SOLO once it’s in Digital Speed Mode
    #          .The method refers to the Uart Write command: 0x05
    #@param  speed_reference  a long value [RPM]
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_speed_reference(self, speed_reference: int) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_speed_reference_input_validation(speed_reference)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        speed_reference = int(speed_reference)
        data = convert_to_data(speed_reference, DataType.UINT32)
        cmd = [self._address, ConstantUart.WRITE_SPEED_REFERENCE,
               data[0], data[1], data[2], data[3]]

        return self.__exec_cmd(cmd)

    ##
    #@brief  This command defines the amount of power percentage during only
    #        Open-loop mode for 3-phase motors
    #          .The method refers to the Uart Write command: 0x06
    #@param  power_reference  a float value between 0 to 100
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_power_reference(self, power_reference: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_power_reference_input_validation(power_reference)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        power_reference = float(power_reference)
        data = convert_to_data(power_reference, DataType.SFXT)
        cmd = [self._address, ConstantUart.WRITE_POWER_REFERENCE,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  By putting 1 in the DATA section of a packet sent with this command, SOLO will start
    #        identifying the electrical parameters of the Motor connected
    #          .The method refers to the Uart Write command: 0x07
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
        data = convert_to_data(identification, DataType.UINT32)
        cmd = [self._address, ConstantUart.WRITE_MOTOR_PARAMETERS_IDENTIFICATION,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command if the DATA is set at zero will stop the whole power and switching system
    #        connected to the motor and it will cut the current floating into the Motor from SOLO
    #          .The method refers to the Uart Write command: 0x08
    #@param  action  enum that specify Disable or Enable of something in SOLO
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_drive_disable_enable(self, action : DisableEnable) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = drive_disable_enable_input_validation(action)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error

        if type(action) is int:
            action = DisableEnable(action)

        action = action.value
        data = convert_to_data(action, DataType.UINT32)
        cmd = [self._address, ConstantUart.WRITE_DRIVE_DISABLE_ENABLE,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command Disables or Enables the Controller resulting in deactivation or activation of the
    #           switching at the output, by disabling the drive, the effect of the Controller on the Motor will be
    #            almost eliminated ( except for body diodes of the Mosfets) allowing freewheeling
    #          .The method refers to the Uart Write command: 0x09
    #@param  output_pwm_frequency_khz  switching frequencies [kHz]      
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_output_pwm_frequency_khz(self, output_pwm_frequency_khz: int) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_output_pwm_frequency_khz_input_validation(output_pwm_frequency_khz)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        output_pwm_frequency_khz = int(output_pwm_frequency_khz)
        data = convert_to_data(output_pwm_frequency_khz, DataType.UINT32)
        cmd = [self._address, ConstantUart.WRITE_OUTPUT_PWM_FREQUENCY_KHZ,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command sets the Speed controller Kp Gain, and it will
    #        be functional only in Digital Closed-loop mode
    #          .The method refers to the Uart Write command: 0x0A
    #@param  speed_controller_kp  a float value between 0 to 300
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_speed_controller_kp(self, speed_controller_kp: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_speed_controller_kp_input_validation(speed_controller_kp)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        speed_controller_kp = float(speed_controller_kp)
        data = convert_to_data(speed_controller_kp, DataType.SFXT)
        cmd = [self._address, ConstantUart.WRITE_SPEED_CONTROLLER_KP,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command sets the Speed controller Ki gain, and it will
    #        be functional only in Digital Closed-loop mode
    #          .The method refers to the Uart Write command: 0x0B
    #@param  speed_controller_ki  a float value between 0 to 300
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_speed_controller_ki(self, speed_controller_ki: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_speed_controller_ki_input_validation(speed_controller_ki)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        speed_controller_ki = float(speed_controller_ki)
        data = convert_to_data(speed_controller_ki, DataType.SFXT)
        cmd = [self._address, ConstantUart.WRITE_SPEED_CONTROLLER_KI,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This commands sets the direction of the rotation of the motor
    #        either to ClockWise rotation or to Counter Clockwise Rotation
    #          .The method refers to the Uart Write command: 0x0C
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
        data = convert_to_data(motor_direction, DataType.UINT32)
        cmd = [self._address, ConstantUart.WRITE_MOTOR_DIRECTION,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command sets the amount of the Phase or Armature resistance
    #        for 3-phase or DC Brushed motors respectively
    #          .The method refers to the Uart Write command: 0x0D
    #@param  motor_resistance  a float value between 0.001 t0 100.0 [Ohm]
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_motor_resistance(self, motor_resistance: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_motor_resistance_input_validation(motor_resistance)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        motor_resistance = float(motor_resistance)
        data = convert_to_data(motor_resistance, DataType.SFXT)
        cmd = [self._address, ConstantUart.WRITE_MOTOR_RESISTANCE,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command sets the amount of the Phase or Armature Inductance
    #        for 3-phase or DC Brushed motors respectively
    #          .The method refers to the Uart Write command: 0x0E
    #@param  motor_inductance  a float value between 0.0 t0 0.2 [Henry]
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_motor_inductance(self, motor_inductance: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_motor_inductance_input_validation(motor_inductance)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        motor_inductance = float(motor_inductance)
        data = convert_to_data(motor_inductance, DataType.SFXT)
        cmd = [self._address, ConstantUart.WRITE_MOTOR_INDUCTANCE,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command sets the number of the Poles of a 3-phase motor commissioned with SOLO
    #          .The method refers to the Uart Write command: 0x0F
    #@param  motor_poles_counts  a long value between 1 to 254
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_motor_poles_counts(self, motor_poles_counts: int) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_motor_poles_counts_input_validation(motor_poles_counts)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        motor_poles_counts = int(motor_poles_counts)
        data = convert_to_data(motor_poles_counts, DataType.UINT32)
        cmd = [self._address, ConstantUart.WRITE_MOTOR_POLES_COUNTS,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command sets the pre-quad number of physical lines of an
    #        incremental encoder engraved on its disk
    #          .The method refers to the Uart Write command: 0x10
    #@param  incremental_encoder_lines  a long value [pre-quad]
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_incremental_encoder_lines(self, incremental_encoder_lines: int) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_incremental_encoder_lines_input_validation(incremental_encoder_lines)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        incremental_encoder_lines = int(incremental_encoder_lines)
        data = convert_to_data(incremental_encoder_lines, DataType.UINT32)
        cmd = [self._address, ConstantUart.WRITE_INCREMENTAL_ENCODER_LINES,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command sets the allowed speed during trajectory following
    #        in closed-loop position controlling mode
    #          .The method refers to the Uart Write command: 0x11
    #@param  speed_limit  a long value [RPM]
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_speed_limit(self, speed_limit: int) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_speed_limit_input_validation(speed_limit)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        speed_limit = int(speed_limit)
        data = convert_to_data(speed_limit, DataType.UINT32)
        cmd = [self._address, ConstantUart.WRITE_SPEED_LIMIT,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command resets the device address of any connected SOLO to zero
    #          .The method refers to the Uart Write command: 0x12
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def reset_address(self) -> Tuple[bool, Error]:
        cmd = [0xFF, ConstantUart.WRITE_RESET_ADDRESS, 0x00, 0x00, 0x00, 0xFF]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command sets the type of the feedback control SOLO has to operate
    #          .The method refers to the Uart Write command: 0x13
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
        data = convert_to_data(feedback_control_mode, DataType.UINT32)
        cmd = [self._address, ConstantUart.WRITE_FEEDBACK_CONTROL_MODE,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command resets SOLO to its factory setting to all the default parameters
    #          .The method refers to the Uart Write command: 0x14
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def reset_factory(self) -> Tuple[bool, Error]:
        cmd = [self._address, ConstantUart.WRITE_RESET_FACTORY,
               0x00, 0x00, 0x00, 0x01]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command sets the Motor type that is connected to SOLO in Digital Mode
    #          .The method refers to the Uart Write command: 0x15
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
        data = convert_to_data(motor_type, DataType.UINT32)
        cmd = [self._address, ConstantUart.WRITE_MOTOR_TYPE,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command sets the Control Mode in terms of Torque,
    #        Speed or Position only in Digital Mode
    #          .The method refers to the Uart Write command: 0x16
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
        data = convert_to_data(control_mode, DataType.UINT32)
        cmd = [self._address, ConstantUart.WRITE_CONTROL_MODE,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command sets the value for Current Controller Kp or proportional gain
    #          .The method refers to the Uart Write command: 0x17
    #@param  current_controller_kp  a float value between 0 to 16000
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_current_controller_kp(self, current_controller_kp: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_current_controller_kp_input_validation(current_controller_kp)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        current_controller_kp = float(current_controller_kp)
        data = convert_to_data(current_controller_kp, DataType.SFXT)
        cmd = [self._address, ConstantUart.WRITE_CURRENT_CONTROLLER_KP,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command sets the value for Current Controller Ki or integral gain
    #          .The method refers to the Uart Write command: 0x18
    #@param  current_controller_ki  a float value between 0 to 16000
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_current_controller_ki(self, current_controller_ki: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_current_controller_ki_input_validation(current_controller_ki)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        current_controller_ki = float(current_controller_ki)
        data = convert_to_data(current_controller_ki, DataType.SFXT)
        cmd = [self._address, ConstantUart.WRITE_CURRENT_CONTROLLER_KI,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    # @@ we don't have this function in arduino uart
    # def set_monitoring_mode(self, mode: int) -> Tuple[bool, Error]:
    #     if mode < 0 or mode > 2):
    #         self._logger.info(ConstantCommon.INPUT_OUT_OF_RANGE)
    #         return False
    #     mode = int(mode)
    #     data = convert_to_data(mode, DataType.UINT32)
    #     cmd = [self._address, ConstantUart.WRITE_MONITORING_MODE,
    #            data[0], data[1], data[2], data[3]]
    #     return self.__exec_cmd(cmd)

    ##
    #@brief  depending on the Motor type: in case of BLDC or PMSM motors Sets the Field
    #        Weakening current reference to help the motor reaching speeds higher than
    #        nominal values and in case of AC Induction Motors Sets the desired magnetizing
    #        current (Id) required for controlling ACIM motors in FOC in Amps
    #          .The method refers to the Uart Write command: 0x1A
    #@param  magnetizing_current_id_reference  a float value [Amps]
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_magnetizing_current_id_reference(self, magnetizing_current_id_reference: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_magnetizing_current_id_reference_input_validation(magnetizing_current_id_reference)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        magnetizing_current_id_reference = float(magnetizing_current_id_reference)
        data = convert_to_data(magnetizing_current_id_reference, DataType.SFXT)
        cmd = [self._address, ConstantUart.WRITE_MAGNETIZING_CURRENT_REFERENCE,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command sets the desired Position reference in terms of quadrature
    #        pulses while SOLO operates with the Incremental Encoders or in terms of
    #        pulses while while SOLO operates with Hall sensors
    #          .The method refers to the Uart Write command: 0x1B
    #@param  position_reference  a long value [Quad-Pulse]
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_position_reference(self, position_reference: int) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_position_reference_input_validation(position_reference)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        position_reference = int(position_reference)
        data = convert_to_data(position_reference, DataType.INT32)
        cmd = [self._address, ConstantUart.WRITE_POSITION_REFERENCE,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command sets the value for Position Controller Kp or proportional gain
    #          .The method refers to the Uart Write command: 0x1C
    #@param  position_controller_kp  a float value between 0 to 16000
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_position_controller_kp(self, position_controller_kp: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_position_controller_kp_input_validation(position_controller_kp)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        position_controller_kp = float(position_controller_kp)
        data = convert_to_data(position_controller_kp, DataType.SFXT)
        cmd = [self._address, ConstantUart.WRITE_POSITION_CONTROLLER_KP,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command sets the value for Position Controller Ki or integrator gain
    #          .The method refers to the Uart Write command: 0x1D
    #@param  position_controller_ki  a float value between 0 to 16000
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_position_controller_ki(self, position_controller_ki: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_position_controller_ki_input_validation(position_controller_ki)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        position_controller_ki = float(position_controller_ki)
        data = convert_to_data(position_controller_ki, DataType.SFXT)
        cmd = [self._address, ConstantUart.WRITE_POSITION_CONTROLLER_KI,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command resets the position counter back to zero
    #          .The method refers to the Uart Write command: 0x1F
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def reset_position_to_zero(self) -> Tuple[bool, Error]:
        cmd = [self._address, ConstantUart.WRITE_RESET_POSITION_TO_ZERO,
                0x00, 0x00, 0x00, 0x01]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command overwrites the reported errors in Error Register
    #        reported with command code of "0xA1"
    #          .The method refers to the Uart Write command: 0x20
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def overwrite_error_register(self) -> Tuple[bool, Error]:
        cmd = [self._address, ConstantUart.WRITE_OVERWRITE_ERROR_REGISTER,
               0x00, 0x00, 0x00, 0x00]
        return self.__exec_cmd(cmd)

    # SOG => Sensorless Observer Gain

    ##
    #@brief  Once in Zero Speed Full Torque algorithm (ZSFT) for controlling the speed of a BLDC or PMSM
    #            in sensorless fashion, this parameter defines the strength of signal injection into the motor, the
    #            user has to make sure this value is not selected too high or too low
    #          .The method refers to the Uart Write command: 0x21
    #@param  zsft_injection_amplitude  a float value between 0.0 to 0.55
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_zsft_injection_amplitude(self, zsft_injection_amplitude: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_zsft_injection_amplitude_input_validation(zsft_injection_amplitude)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        zsft_injection_amplitude = float(zsft_injection_amplitude)
        data = convert_to_data(zsft_injection_amplitude, DataType.SFXT)
        cmd = [self._address, ConstantUart.WRITE_ZSFT_INJECTION_AMPLITUDE,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)
    
    ##
    #@brief  Once in Zero Speed Full Torque algorithm (ZSFT) for controlling the speed of a BLDC or PMSM
    #             in sensorless fashion, this parameter defines the strength of signal injection into the motor to
    #               identify the polarity of the Motor at the startup
    #          .The method refers to the Uart Write command: 0x22
    #@param  zsft_polarity_amplitude  a float value between 0.0 to 0.55
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_zsft_polarity_amplitude(self, zsft_polarity_amplitude: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_zsft_polarity_amplitude_input_validation(zsft_polarity_amplitude)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        zsft_polarity_amplitude = float(zsft_polarity_amplitude)
        data = convert_to_data(zsft_polarity_amplitude, DataType.SFXT)
        cmd = [self._address, ConstantUart.WRITE_ZSFT_POLARITY_AMPLITUDE,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)
    
    ##
    #@brief  This command sets the observer gain for the Non-linear observer
    #        that estimates the speed of a DC brushed once the motor type
    #        is selected as DC brushed
    #          .The method refers to the Uart Write command: 0x23
    #@param  observer_gain  a float value between 0.01 to 1000
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_observer_gain_dc(self, observer_gain: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_observer_gain_dc_input_validation(observer_gain)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        observer_gain = float(observer_gain)
        data = convert_to_data(observer_gain, DataType.SFXT)
        cmd = [self._address, ConstantUart.WRITE_OBSERVER_GAIN_DC,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command defines the frequency of signal injection into the Motor in
    #           runtime, by selecting zero the full injection frequency will be applied which allows to reach to
    #           higher speeds, however for some motors, it’s better to increase this value
    #          .The method refers to the Uart Write command: 0x24
    #@param  zsft_injection_frequency  a long value between 0 to 10
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_zsft_injection_frequency(self, zsft_injection_frequency: int) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_zsft_injection_frequency_input_validation(zsft_injection_frequency)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        zsft_injection_frequency = int(zsft_injection_frequency)
        data = convert_to_data(zsft_injection_frequency, DataType.UINT32)
        cmd = [self._address, ConstantUart.WRITE_ZSFT_INJECTION_FREQUENCY,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  Once in Sensorless speed or torque controlling of a BLDC or PMSM motors, this parameter
    #				defines the speed in which the Low speed algorithm has to switch to high speed algorithm
    #           .The method refers to the Uart Write command: 0x25
    #@param  sensorless_transition_speed  a long value between 1 to 5000
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_sensorless_transition_speed(self, sensorless_transition_speed: int) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_sensorless_transition_speed_input_validation(sensorless_transition_speed)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        sensorless_transition_speed = int(sensorless_transition_speed)
        data = convert_to_data(sensorless_transition_speed, DataType.UINT32)
        cmd = [self._address, ConstantUart.WRITE_SENSORLESS_TRANSITION_SPEED,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command sets the baud-rate of the UART line
    #          .The method refers to the Uart Write command: 0x26
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
        data = convert_to_data(baudrate, DataType.UINT32)
        cmd = [self._address, ConstantUart.WRITE_UART_BAUDRATE,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command starts or stops the process of sensor calibration
    #          .The method refers to the Uart Write command: 0x27
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
        data = convert_to_data(calibration_action, DataType.UINT32)
        cmd = [self._address, ConstantUart.WRITE_SENSOR_CALIBRATION,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command sets the per-unit offset identified after sensor calibration
    #        for Encoder or Hall sensors in C.C.W direction
    #          .The method refers to the Uart Write command: 0x28
    #@param  encoder_hall_offset  a float value between 0.0 to 1.0
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_encoder_hall_ccw_offset(self, encoder_hall_offset: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_encoder_hall_ccw_offset_input_validation(encoder_hall_offset)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        encoder_hall_offset = float(encoder_hall_offset)
        data = convert_to_data(encoder_hall_offset, DataType.SFXT)
        cmd = [self._address, ConstantUart.WRITE_ENCODER_HALL_CCW_OFFSET,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command sets the per-unit offset identified after sensor calibration
    #        for Encoder or Hall sensors in C.W direction
    #          .The method refers to the Uart Write command: 0x29
    #@param  encoder_hall_offset  a float value between 0.0 to 1.0
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_encoder_hall_cw_offset(self, encoder_hall_offset: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_encoder_hall_cw_offset_input_validation(encoder_hall_offset)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        encoder_hall_offset = float(encoder_hall_offset)
        data = convert_to_data(encoder_hall_offset, DataType.SFXT)
        cmd = [self._address, ConstantUart.WRITE_ENCODER_HALL_CW_OFFSET,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command defines the acceleration value of the Speed for speed controller
    #        both in Analogue and Digital modes in Revolution per square seconds
    #          .The method refers to the Uart Write command: 0x2A
    #@param  speed_acceleration_value  a float value [Rev/S^2]
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_speed_acceleration_value(self, speed_acceleration_value: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_speed_acceleration_value_input_validation(speed_acceleration_value)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        speed_acceleration_value = float(speed_acceleration_value)
        data = convert_to_data(speed_acceleration_value, DataType.SFXT)
        cmd = [self._address, ConstantUart.WRITE_SPEED_ACCELERATION_VALUE,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command defines the deceleration value of the Speed for speed controller
    #        both in Analogue and Digital modes in Revolution per square seconds
    #          .The method refers to the Uart Write command: 0x2B
    #@param  speed_deceleration_value  a float value [Rev/S^2]
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_speed_deceleration_value(self, speed_deceleration_value: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_speed_deceleration_value_input_validation(speed_deceleration_value)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        speed_deceleration_value = float(speed_deceleration_value)
        data = convert_to_data(speed_deceleration_value, DataType.SFXT)
        cmd = [self._address, ConstantUart.WRITE_SPEED_DECELERATION_VALUE,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command sets the baud rate of CAN bus in CANOpen network
    #          .The method refers to the Uart Write command: 0x2C
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
        data = convert_to_data(canbus_baudrate, DataType.UINT32)
        cmd = [self._address, ConstantUart.WRITE_CAN_BUS_BAUDRATE,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command defines the resolution of the speed at S/T input
    #          while SOLO operates in Analogue mode
    #          .The method refers to the Uart Write command: 0x2D
    #@param  division_coefficient  a float value
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_analogue_speed_resolution_division_coefficient(self, division_coefficient: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_analogue_speed_resolution_division_coefficient_input_validation(division_coefficient)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        division_coefficient = float(division_coefficient)
        data = convert_to_data(division_coefficient, DataType.SFXT)
        cmd = [self._address, ConstantUart.WRITE_ASRDC,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command defines the type of the Motion Profile that is
    #          being used in Speed or Position Modes
    #          .The method refers to the Uart Write command: 0x30
    #@param  motion_profile_mode enum that specify the type of the Motion Profile
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    #
    def set_motion_profile_mode(self, motion_profile_mode: MotionProfileMode) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_motion_profile_mode_input_validation(motion_profile_mode)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        
        if isinstance(motion_profile_mode, int):
            motion_profile_mode = MotionProfileMode(motion_profile_mode)

        motion_profile_mode = motion_profile_mode.value
        data = convert_to_data(motion_profile_mode, DataType.UINT32)
        cmd = [self._address, ConstantUart.WRITE_MOTION_PROFILE_MODE,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
    #          .The method refers to the Uart Write command: 0x31
    #@param  motion_profile_variable1 a float value
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_motion_profile_variable1(self, motion_profile_variable1: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_motion_profile_variable1_input_validation(motion_profile_variable1)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        motion_profile_variable1 = float(motion_profile_variable1)
        data = convert_to_data(motion_profile_variable1, DataType.SFXT)
        cmd = [self._address, ConstantUart.WRITE_MOTION_PROFILE_VARIABLE1,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
    #          .The method refers to the Uart Write command: 0x32
    #@param  motion_profile_variable2 a float value
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_motion_profile_variable2(self, motion_profile_variable2: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_motion_profile_variable2_input_validation(motion_profile_variable2)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        motion_profile_variable2 = float(motion_profile_variable2)
        data = convert_to_data(motion_profile_variable2, DataType.SFXT)
        cmd = [self._address, ConstantUart.WRITE_MOTION_PROFILE_VARIABLE2,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
    #          .The method refers to the Uart Write command: 0x33
    #@param  motion_profile_variable3 a float value
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_motion_profile_variable3(self, motion_profile_variable3: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_motion_profile_variable3_input_validation(motion_profile_variable3)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        motion_profile_variable3 = float(motion_profile_variable3)
        data = convert_to_data(motion_profile_variable3, DataType.SFXT)
        cmd = [self._address, ConstantUart.WRITE_MOTION_PROFILE_VARIABLE3,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
    #          .The method refers to the Uart Write command: 0x34
    #@param  motion_profile_variable4 a float value
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_motion_profile_variable4(self, motion_profile_variable4: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_motion_profile_variable4_input_validation(motion_profile_variable4)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        motion_profile_variable4 = float(motion_profile_variable4)
        data = convert_to_data(motion_profile_variable4, DataType.SFXT)
        cmd = [self._address, ConstantUart.WRITE_MOTION_PROFILE_VARIABLE4,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
    #          .The method refers to the Uart Write command: 0x35
    #@param  motion_profile_variable5 a float value
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_motion_profile_variable5(self, motion_profile_variable5: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_motion_profile_variable5_input_validation(motion_profile_variable5)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        motion_profile_variable5 = float(motion_profile_variable5)
        data = convert_to_data(motion_profile_variable5, DataType.SFXT)
        cmd = [self._address, ConstantUart.WRITE_MOTION_PROFILE_VARIABLE5,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command Set the Digiatal Ouput pin Status
    #          .The method refers to the Uart Write command: 0x38
    #@param  channel    SOLOMotorControllers.Channel
    #@param  state	    .SOLOMotorControllers.DigitalIoState
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    #
    def set_digital_output_state(self, channel: Channel, state: DigitalIoState) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_digital_output_state_input_validation(channel)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        
        if isinstance(channel, int):
            channel = Channel(channel)

        last_out_register, error = get_digital_outputs_register()
        if state == 1:
            last_out_register = last_out_register | (1 << channel)
        else:
            last_out_register = last_out_register & (~(1 << channel))

        data = convert_to_data(last_out_register, DataType.UINT32)
        cmd = [self._address, ConstantUart.WRITE_DIGITAL_OUTPUTS_REGISTER,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This command defines the maximum allowed regeneration current sent back from the Motor to
    #				the Power Supply during decelerations
    #          .The method refers to the Uart Write command: 0x39
    #@param  current a float value
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_regeneration_current_limit(self, current: float) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_regeneration_current_limit_input_validation(current)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        current = float(current)
        data = convert_to_data(current, DataType.SFXT)
        cmd = [self._address, ConstantUart.WRITE_REGENERATION_CURRENT_LIMIT,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##
    #@brief  This value defines the the sampling window of qualification digital filter applied to the output of
    #			the position sensor before being processed by DSP
    #          .The method refers to the Uart Write command: 0x3A
    #@param  level a long value
    #@retval List of [bool 0 fail / 1 for success, Error class/enumeration]
    def set_position_sensor_digital_filter_level(self, level: int) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        input_validate, error, log_msg = set_position_sensor_digital_filter_level_input_validation(level)
        if input_validate is False:
            self._logger.info(log_msg)
            return False, error
        level = int(level)
        data = convert_to_data(level, DataType.UINT32)
        cmd = [self._address, ConstantUart.WRITE_POSITION_SENSOR_DIGITAL_FILTER_LEVEL,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

# ----------------------------------------------
# ---------------------Read---------------------
# ----------------------------------------------

    ##
    #@brief  This command reads the device address connected on the line
    #          .The method refers to the Uart Read command: 0x81
    #@retval  List of [long device address connected on the line, Error class/enumeration]
    def get_device_address(self) -> Tuple[int, Error]:
        cmd = [0xFF, ConstantUart.READ_DEVICE_ADDRESS, 0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.UINT32), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the phase-A voltage of the motor connected to the
    #        "A" pin output of SOLO for 3-phase Motors
    #          .The method refers to the Uart Read command: 0x82
    #@retval  List of [float phase-A voltage of the motor [Volts], Error class/enumeration]
    def get_phase_a_voltage(self) -> Tuple[float, Error]:
        cmd = [self._address, ConstantUart.READ_PHASEA_VOLTAGE, 0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.SFXT), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the phase-B voltage of the motor connected to the
    #        "B" pin output of SOLO for 3-phase Motors
    #          .The method refers to the Uart Read command: 0x83
    #@retval  List of [float 0 phase-A voltage of the motor [Volts], Error class/enumeration]
    def get_phase_b_voltage(self) -> Tuple[float, Error]:
        cmd = [self._address, ConstantUart.READ_PHASEB_VOLTAGE, 0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.SFXT), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the phase-A current of the motor connected to the
    #        "A" pin output of SOLO for 3-phase Motors
    #          .The method refers to the Uart Read command: 0x84
    #@retval  List of [float phase-A current of the motor [Amps], Error class/enumeration]
    def get_phase_a_current(self) -> Tuple[float, Error]:
        cmd = [self._address, ConstantUart.READ_PHASEA_CURRENT, 0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.SFXT), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the phase-B current of the motor connected to the
    #        "B" pin output of SOLO for 3-phase Motors
    #          .The method refers to the Uart Read command: 0x85
    #@retval  List of [float phase-B current of the motor [Amps], Error class/enumeration]
    def get_phase_b_current(self) -> Tuple[float, Error]:
        cmd = [self._address, ConstantUart.READ_PHASEB_CURRENT, 0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.SFXT), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the input BUS voltage
    #          .The method refers to the Uart Read command: 0x86
    #@retval  List of [float  BUS voltage [Volts], Error class/enumeration]
    def get_bus_voltage(self) -> Tuple[float, Error]:
        cmd = [self._address, ConstantUart.READ_BUS_VOLTAGE, 0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.SFXT), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the current inside the DC brushed motor connected to
    #        "B" and "C" outputs of SOLO
    #          .The method refers to the Uart Read command: 0x87
    #@retval  List of [float between [Amps], Error class/enumeration]
    def get_dc_motor_current_im(self) -> Tuple[float, Error]:
        cmd = [self._address, ConstantUart.READ_DC_MOTOR_CURRENT_IM,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.SFXT), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the voltage of the DC brushed motor connected to
    #        "B" and "C" outputs of SOLO
    #          .The method refers to the Uart Read command: 0x88
    #@retval  List of [float [Volts], Error class/enumeration]
    def get_dc_motor_voltage_vm(self) -> Tuple[float, Error]:
        cmd = [self._address, ConstantUart.READ_DC_MOTOR_VOLTAGE_VM,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.SFXT), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the value of the Speed controller Kp gain,
    #        set for Digital mode operations
    #          .The method refers to the Uart Read command: 0x89
    #@retval  List of [float between 0 to 16000, Error class/enumeration]
    def get_speed_controller_kp(self) -> Tuple[float, Error]:
        cmd = [self._address, ConstantUart.READ_SPEED_CONTROLLER_KP,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.SFXT), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the value of the Speed controller Ki gain,
    #        set for Digital mode operations
    #          .The method refers to the Uart Read command: 0x8A
    #@retval  List of [float between 0 to 16000, Error class/enumeration]
    def get_speed_controller_ki(self) -> Tuple[float, Error]:
        cmd = [self._address, ConstantUart.READ_SPEED_CONTROLLER_KI,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.SFXT), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the output switching frequency of SOLO in Hertz
    #          .The method refers to the Uart Read command: 0x8B
    #@retval  List of [long [Hz], Error class/enumeration]
    def get_output_pwm_frequency_khz(self) -> Tuple[int, Error]:
        cmd = [self._address, ConstantUart.READ_OUTPUT_PWM_FREQUENCY_HZ,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.UINT32), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the value of the current limit set for SOLO in
    #        closed-loop digital operation mode
    #          .The method refers to the Uart Read command: 0x8C
    #@retval  List of [float [Amps], Error class/enumeration]
    def get_current_limit(self) -> Tuple[float, Error]:
        cmd = [self._address, ConstantUart.READ_CURRENT_LIMIT,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.SFXT), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the actual monetary value of “Iq” that is
    #        the current acts in torque generation in FOC mode for 3-phase motors
    #          .The method refers to the Uart Read command: 0x8D
    #@retval  List of [float [Amps], Error class/enumeration]
    def get_quadrature_current_iq_feedback(self) -> Tuple[float, Error]:
        cmd = [self._address, ConstantUart.READ_QUADRATURE_CURRENT_IQ_FEEDBACK,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.SFXT), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the actual monetary value of Id that is the
    #        direct current acting in FOC
    #          .The method refers to the Uart Read command: 0x8E
    #@retval  List of [float [Amps], Error class/enumeration]
    def get_magnetizing_current_id_feedback(self) -> Tuple[float, Error]:
        cmd = [self._address, ConstantUart.READ_MAGNETIZING_CURRENT_ID_FEEDBACK,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.SFXT), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the number of Poles set for 3-phase motors
    #          .The method refers to the Uart Read command: 0x8F
    #     #@retval  List of [long between 1 to 254, Error class/enumeration]
    def get_motor_poles_counts(self) -> Tuple[int, Error]:
        cmd = [self._address, ConstantUart.READ_MOTOR_POLES_COUNTS,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.UINT32), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the number of physical Incremental encoder lines set on SOLO
    #          .The method refers to the Uart Read command: 0x90
    #@retval  List of [long between 1 to 200000, Error class/enumeration]
    def get_incremental_encoder_lines(self) -> Tuple[int, Error]:
        cmd = [self._address, ConstantUart.READ_INCREMENTAL_ENCODER_LINES,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.UINT32), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the amount of value set for Current controller
    #        Kp or proportional gain
    #          .The method refers to the Uart Read command: 0x91
    #@retval  List of [float between 0 to 16000, Error class/enumeration]
    def get_current_controller_kp(self) -> Tuple[float, Error]:
        cmd = [self._address, ConstantUart.READ_CURRENT_CONTROLLER_KP,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.SFXT), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the amount of value set for Current controller
    #        Ki or integrator gain
    #          .The method refers to the Uart Read command: 0x92
    #@retval  List of [float between 0 to 16000, Error class/enumeration]
    def get_current_controller_ki(self) -> Tuple[float, Error]:
        cmd = [self._address, ConstantUart.READ_CURRENT_CONTROLLER_KI,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.SFXT), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the momentary temperature of the board in centigrade
    #          .The method refers to the Uart Read command: 0x93
    #@retval  List of [float [°C], Error class/enumeration]
    def get_board_temperature(self) -> Tuple[float, Error]:
        cmd = [self._address, ConstantUart.READ_BOARD_TEMPERATURE,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.SFXT), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the Phase or Armature resistance of
    #        the 3-phase or DC brushed motor connected to SOLO respectively
    #          .The method refers to the Uart Read command: 0x94
    #@retval  List of [float [Ohms], Error class/enumeration]
    def get_motor_resistance(self) -> Tuple[float, Error]:
        cmd = [self._address, ConstantUart.READ_MOTOR_RESISTANCE,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.SFXT), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the Phase or Armature Inductance of
    #        the 3-phase or DC brushed motor connected to SOLO respectively
    #          .The method refers to the Uart Read command: 0x95
    #@retval  List of [float [Henry], Error class/enumeration]
    def get_motor_inductance(self) -> Tuple[float, Error]:
        cmd = [self._address, ConstantUart.READ_MOTOR_INDUCTANCE,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.SFXT), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  his command reads the actual speed of the motor measured or estimated by SOLO in
    #        sensorless or sensor-based modes respectively
    #          .The method refers to the Uart Read command: 0x96
    #@retval  List of [long [RPM], Error class/enumeration]
    def get_speed_feedback(self) -> Tuple[int, Error]:
        cmd = [self._address, ConstantUart.READ_SPEED_FEEDBACK,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.INT32), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the Motor type selected for Digital or Analogue mode operations
    #          .The method refers to the Uart Read command: 0x97
    #@retval  List of [long between 0 to 3, Error class/enumeration]
    def get_motor_type(self) -> Tuple[MotorType, Error]:
        cmd = [self._address, ConstantUart.READ_MOTOR_TYPE,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return MotorType(convert_from_data(data, DataType.UINT32)), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the feedback control mode selected on SOLO both
    #        for Analogue and Digital operations
    #          .The method refers to the Uart Read command: 0x99
    #@retval  List of [long between 0 to 2, Error class/enumeration]
    def get_feedback_control_mode(self) -> Tuple[FeedbackControlMode, Error]:
        cmd = [self._address, ConstantUart.READ_FEEDBACK_CONTROL_MODE,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return FeedbackControlMode(convert_from_data(data, DataType.UINT32)), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the actual commanding mode that SOLO is operating
    #          .The method refers to the Uart Read command: 0x9A
    #@retval  List of [long between 0 or 1, Error class/enumeration]
    def get_command_mode(self) -> Tuple[CommandMode, Error]:
        cmd = [self._address, ConstantUart.READ_COMMAND_MODE,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return CommandMode(convert_from_data(data, DataType.UINT32)), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the Control Mode type in terms of Torque,
    #        Speed or Position in both Digital and Analogue modes
    #          .The method refers to the Uart Read command: 0x9B
    #@retval  List of [long between 0 to 2, Error class/enumeration]
    def get_control_mode(self) -> Tuple[ControlMode, Error]:
        cmd = [self._address, ConstantUart.READ_CONTROL_MODE,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return ControlMode(convert_from_data(data, DataType.UINT32)), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the value of the speed limit set on SOLO
    #          .The method refers to the Uart Read command: 0x9C
    #@retval  List of [long [RPM], Error class/enumeration]
    def get_speed_limit(self) -> Tuple[int, Error]:
        cmd = [self._address, ConstantUart.READ_SPEED_LIMIT,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.UINT32), Error.NO_ERROR_DETECTED
        return -1, error


    ##
    #@brief  This command reads the amount of value set for Position
    #        controller Kp or proportional gain
    #          .The method refers to the Uart Read command: 0x9D
    #@retval  List of [float between 0 to 16000, Error class/enumeration]
    def get_position_controller_kp(self) -> Tuple[float, Error]:
        cmd = [self._address, ConstantUart.READ_POSITION_CONTROLLER_KP,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.SFXT), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the amount of value set for Position
    #        controller Ki or integrator gain
    #          .The method refers to the Uart Read command: 0x9E
    #@retval  List of [float between 0 to 16000, Error class/enumeration]
    def get_position_controller_ki(self) -> Tuple[float, Error]:
        cmd = [self._address, ConstantUart.READ_POSITION_CONTROLLER_KI,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.SFXT), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the number of counted pulses from the
    #        Incremental Encoder or Hall sensors
    #          .The method refers to the Uart Read command: 0xA0
    #@retval  List of [long [Quad-Pulses], Error class/enumeration]
    def get_position_counts_feedback(self) -> Tuple[int, Error]:
        cmd = [self._address, ConstantUart.READ_POSITION_COUNTS_FEEDBACK,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.INT32), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the error register which is a 32 bit register with
    #        each bit corresponding to specific errors
    #          .The method refers to the Uart Read command: 0xA1
    #@retval  List of [long , Error class/enumeration]
    def get_error_register(self) -> Tuple[int, Error]:
        cmd = [self._address, ConstantUart.READ_ERROR_REGISTER,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.UINT32), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the Firmware version existing currently on the SOLO unit
    #          .The method refers to the Uart Read command: 0xA2
    #@retval  List of [long, Error class/enumeration]
    def get_device_firmware_version(self) -> Tuple[int, Error]:
        cmd = [self._address, ConstantUart.READ_DEVICE_FIRMWARE_VERSION,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.UINT32), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the Hardware version of the SOLO unit connected
    #          .The method refers to the Uart Read command: 0xA3
    #@retval  List of [long, Error class/enumeration]
    def get_device_hardware_version(self) -> Tuple[int, Error]:
        cmd = [self._address, ConstantUart.READ_DEVICE_HARDWARE_VERSION,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.UINT32), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the amount of desired Torque reference (Iq or IM)
    #        already set for the Motor to follow in Digital Closed-loop Torque control mode
    #          .The method refers to the Uart Read command: 0xA4
   #@retval  List of [float [Amps], Error class/enumeration]
    def get_torque_reference_iq(self) -> Tuple[float, Error]:
        cmd = [self._address, ConstantUart.READ_TORQUE_REFERENCE_IQ,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.SFXT), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the amount of desired Speed reference already set for
    #        the Motor to follow in Digital Closed-loop Speed control mode
    #          .The method refers to the Uart Read command: 0xA5
    #@retval  List of [long [RPM], Error class/enumeration]
    def get_speed_reference(self) -> Tuple[int, Error]:
        cmd = [self._address, ConstantUart.READ_SPEED_REFERENCE,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.UINT32), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the amount of desired Id (direct current) or
    #        Magnetizing current reference already set for the Motor to follow
    #        in Digital Closed-loop Speed control mode for ACIM motors
    #          .The method refers to the Uart Read command: 0xA6
    #@retval  List of [float [Amps], Error class/enumeration]
    def get_magnetizing_current_id_reference(self) -> Tuple[float, Error]:
        cmd = [self._address, ConstantUart.READ_MAGNETIZING_CURRENT_ID_REFERENCE,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.SFXT), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the desired position reference set for the Motor
    #        to follow in Digital Closed-loop Position mode in terms of quadrature pulses
    #          .The method refers to the Uart Read command: 0xA7
    #@retval  List of [long [Quad-Pulses], Error class/enumeration]
    def get_position_reference(self) -> Tuple[int, Error]:
        cmd = [self._address, ConstantUart.READ_POSITION_REFERENCE,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.INT32), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the desired Power reference for SOLO to apply in
    #        Digital Open-loop speed control mode for 3-phase motors in terms of percentage
    #          .The method refers to the Uart Read command: 0xA8
    #@retval  List of [float [%], Error class/enumeration]
    def get_power_reference(self) -> Tuple[float, Error]:
        cmd = [self._address, ConstantUart.READ_POWER_REFERENCE,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.SFXT), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This commands reads the desired direction of rotation set for the Motor
    #          .The method refers to the Uart Read command: 0xA9
    #@retval  List of [long 0 Counter ClockWise / 1 ClockWise, Error class/enumeration]
    def get_motor_direction(self) -> Tuple[Direction, Error]:
        cmd = [self._address, ConstantUart.READ_MOTOR_DIRECTION,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return Direction(convert_from_data(data, DataType.UINT32)), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the value of Sensorless Zero Speed Full Torque Injection Amplitude
    #          .The method refers to the Uart Read command: 0xAA
    #@retval  List of [float between 0.0 to 0.55, Error class/enumeration]
    def get_zsft_injection_amplitude(self) -> Tuple[float, Error]:
        cmd = [self._address, ConstantUart.READ_ZSFT_INJECTION_AMPLITUDE,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.SFXT), Error.NO_ERROR_DETECTED
        return -1, error
        
    ##
    #@brief  This command reads the value of Sensorless Zero Speed Full Torque Polarity Amplitude
    #          .The method refers to the Uart Read command: 0xAB
    #@retval  List of [float between 0.0 to 0.55, Error class/enumeration]
    def get_zsft_polarity_amplitude(self) -> Tuple[float, Error]:
        cmd = [self._address, ConstantUart.READ_ZSFT_POLARITY_AMPLITUDE,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.SFXT), Error.NO_ERROR_DETECTED
        return -1, error
        
    
    ##
    #@brief  This command reads the value of Sensorless Observer Gain for DC Motor
    #          .The method refers to the Uart Read command: 0xAC
    #@retval  List of [float between 0.01 to 1000, Error class/enumeration]
    def get_observer_gain_dc(self) -> Tuple[float, Error]:
        cmd = [self._address, ConstantUart.READ_OBSERVER_GAIN_DC,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.SFXT), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the value of Sensorless Zero Speed Full Torque Injection Frequency
    #          .The method refers to the Uart Read command: 0xAD
    #@retval  List of [long between 0 to 10, Error class/enumeration]
    def get_zsft_injection_frequency(self) -> Tuple[int, Error]:
        cmd = [self._address, ConstantUart.READ_ZSFT_INJECTION_FREQUENCY,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.UINT32), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the value of Sensorless Observer
    #        Filter Gain for Normal BLDC-PMSM Motors
    #          .The method refers to the Uart Read command: 0xAD
    #@retval  List of [float between 0.01 to 16000, Error class/enumeration]
    def get_filter_gain_bldc_pmsm(self) -> Tuple[float, Error]:
        cmd = [self._address, ConstantUart.READ_FILTER_GAIN_BLDC_PMSM,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.SFXT), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the value of Sensorless Transition Speed
    #          .The method refers to the Uart Read command: 0xAE
    #@retval  List of [long between 1 to 5000, Error class/enumeration]
    def get_sensorless_transition_speed(self) -> Tuple[float, Error]:
        cmd = [self._address, ConstantUart.READ_SENSORLESS_TRANSITION_SPEED,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.UINT32), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the measured or estimated per-unit angle of the 3-phase motors
    #          .The method refers to the Uart Read command: 0xB0
    #@retval  List of [float [Per Unit], Error class/enumeration]
    def get_3phase_motor_angle(self) -> Tuple[float, Error]:
        cmd = [self._address, ConstantUart.READ_3PHASE_MOTOR_ANGLE,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.SFXT), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the per-unit Encoder or Hall sensor offset in C.C.W direction
    #          .The method refers to the Uart Read command: 0xB1
    #@retval  List of [float [Per Unit], Error class/enumeration]
    def get_encoder_hall_ccw_offset(self) -> Tuple[float, Error]:
        cmd = [self._address, ConstantUart.READ_ENCODER_HALL_CCW_OFFSET,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.SFXT), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the per-unit Encoder or Hall sensor offset in C.C.W direction
    #          .The method refers to the Uart Read command: 0xB2
    #@retval  List of [float [Per Unit], Error class/enumeration]
    def get_encoder_hall_cw_offset(self) -> Tuple[float, Error]:
        cmd = [self._address, ConstantUart.READ_ENCODER_HALL_CW_OFFSET,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.SFXT), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads Baud Rate selected on SOLO unit to communicate through UART line
    #          .The method refers to the Uart Read command: 0xB3
    #@retval  List of [long [Bits/s], Error class/enumeration]
    def get_uart_baudrate(self) -> Tuple[UartBaudRate, Error]:
        cmd = [self._address, ConstantUart.READ_UART_BAUD_RATE,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return UartBaudRate(convert_from_data(data, DataType.UINT32)), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the acceleration value of the Speed for
    #        speed controller both in Analogue and Digital modes
    #        in Revolution per square seconds
    #          .The method refers to the Uart Read command: 0xB4
    #@retval  List of [float [Rev/S^2], Error class/enumeration]
    def get_speed_acceleration_value(self) -> Tuple[float, Error]:
        cmd = [self._address, ConstantUart.READ_SPEED_ACCELERATION_VALUE,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.SFXT), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the deceleration value of the Speed for
    #        speed controller both in Analogue and Digital modes
    #        in Revolution per square seconds
    #          .The method refers to the Uart Read command: 0xB5
    #@retval  List of [float [Rev/S^2], Error class/enumeration]
    def get_speed_deceleration_value(self) -> Tuple[float, Error]:
        cmd = [self._address, ConstantUart.READ_SPEED_DECELERATION_VALUE,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.SFXT), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command test if the communication is working
   #@retval  List of [ bool 0 not working / 1 for working, Error class/enumeration]
    def communication_is_working(self) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        _temperature, error = self.get_board_temperature()
        time.sleep(0.2)
        _temperature, error = self.get_board_temperature()
        if error == Error.NO_ERROR_DETECTED:
            return True, error
        return False, error

    ##
    #@brief  This Command reads the number of counted index pulses
    #        seen on the Incremental Encoder’s output
    #          .The method refers to the Uart Read command: 0xB8
    #@retval  List of [long [Pulses], Error class/enumeration]
    def get_encoder_index_counts(self) -> Tuple[int, Error]:
        cmd = [self._address, ConstantUart.READ_ENCODER_INDEX_COUNTS,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.UINT32), Error.NO_ERROR_DETECTED
        return -1, error

    def get_can_bus_baudrate(self) -> Tuple[CanBusBaudRate, Error]:
        cmd = [self._address, ConstantUart.READ_CANBUS_BAUD_RATE,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return CanBusBaudRate(convert_from_data(data, DataType.UINT32)), Error.NO_ERROR_DETECTED
        return -1, error
            
    ##
    #@brief  This command reads the Analogue Speed Resolution Division Coefficient (ASRDC)
    #          while SOLO operates in Analogue mode
    #          .The method refers to the Uart Write command: 0xB7
    #@retval  List of [Analogue Speed Resolution Division Coefficient, Error class/enumeration]
    def get_analogue_speed_resolution_division_coefficient(self) -> Tuple[float, Error]:
        cmd = [self._address, ConstantUart.READ_ASRDC,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.SFXT), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the type of the Embedded Motion profile active in the controller
   #          being used in Speed or Position Modes
    #          .The method refers to the Uart Write command: 0xBB
    #@retval  List of [int value of Motion profile, Error class/enumeration]
    def get_motion_profile_mode(self) -> Tuple[MotionProfileMode, Error]:
        cmd = [self._address, ConstantUart.READ_MOTION_PROFILE_MODE,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return MotionProfileMode(convert_from_data(data, DataType.UINT32)), Error.NO_ERROR_DETECTED
        return -1, error
        
    ##
    #@brief  This command reads the value of the Motion Profile Variable1 set inside the controller
    #          .The method refers to the Uart Write command: 0xBC
    #@retval  List of [Motion Profile Variable1, Error class/enumeration]
    def get_motion_profile_variable1(self) -> Tuple[float, Error]:
        cmd = [self._address, ConstantUart.READ_MOTION_PROFILE_VARIABLE1,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.SFXT), Error.NO_ERROR_DETECTED
        return -1, error
        
    ##
    #@brief  This command reads the value of the Motion Profile Variable2 set inside the controller
    #          .The method refers to the Uart Write command: 0xBD
    #@retval  List of [Motion Profile Variable2, Error class/enumeration]
    def get_motion_profile_variable2(self) -> Tuple[float, Error]:
        cmd = [self._address, ConstantUart.READ_MOTION_PROFILE_VARIABLE2,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.SFXT), Error.NO_ERROR_DETECTED
        return -1, error
        
    ##
    #@brief  This command reads the value of the Motion Profile Variable3 set inside the controller
    #          .The method refers to the Uart Write command: 0xBE
    #@retval  List of [Motion Profile Variable3, Error class/enumeration]
    def get_motion_profile_variable3(self) -> Tuple[float, Error]:
        cmd = [self._address, ConstantUart.READ_MOTION_PROFILE_VARIABLE3,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.SFXT), Error.NO_ERROR_DETECTED
        return -1, error
        
    ##
    #@brief  This command reads the value of the Motion Profile Variable4 set inside the controller
    #          .The method refers to the Uart Write command: 0xBF
    #@retval List of [ Motion Profile Variable4, Error class/enumeration]
    def get_motion_profile_variable4(self) -> Tuple[float, Error]:
        cmd = [self._address, ConstantUart.READ_MOTION_PROFILE_VARIABLE4,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.SFXT), Error.NO_ERROR_DETECTED
        return -1, error
        
    ##
    #@brief  This command reads the value of the Motion Profile Variable5 set inside the controller
    #          .The method refers to the Uart Write command: 0xC0
    #@retval List of [ Motion Profile Variable5, Error class/enumeration]
    def get_motion_profile_variable5(self) -> Tuple[float, Error]:
        cmd = [self._address, ConstantUart.READ_MOTION_PROFILE_VARIABLE5,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.SFXT), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the value of the Digital Outputs Register as a 32 bits register, where each
    #               bit represent the state of each output
    #          .The method refers to the Uart Read command: 0xC4
    #@retval List of [ int, Error class/enumeration]
    def get_digital_outputs_register(self) -> Tuple[int, Error]:
        cmd = [self._address, ConstantUart.READ_DIGITAL_OUTPUT_REGISTER,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.UINT32), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the current state of the controller
    #          .The method refers to the Uart Write command: 0xC7
    #@retval  List of [int value of Drive Disable/Enable, Error class/enumeration]
    def get_drive_disable_enable(self) -> Tuple[DisableEnable, Error]:
        cmd = [self._address, ConstantUart.READ_DRIVE_DISABLE_ENABLE,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return DisableEnable(convert_from_data(data, DataType.UINT32)), Error.NO_ERROR_DETECTED
        return -1, error
        
    ##
    #@brief  This command reads the value of the Regeneration Current Limit
    #          .The method refers to the Uart Write command: 0xC8  
    #@retval List of [ float, Error class/enumeration]
    def get_regeneration_current_limit(self) -> Tuple[float, Error]:
        cmd = [self._address, ConstantUart.READ_REGENERATION_CURRENT_LIMIT,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.SFXT), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the value of the Position Sensor Digital Filter Level
    #          .The method refers to the Uart Write command: 0xC9
    #@retval List of [ long, Error class/enumeration]
    def get_position_sensor_digital_filter_level(self) -> Tuple[int, Error]:
        cmd = [self._address, ConstantUart.READ_POSITION_SENSOR_DIGITAL_FILTER_LEVEL,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.UINT32), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the value of the Digital Input Register as a 32 bits register
    #          .The method refers to the Uart Write command: 0xC5
    #@retval List of [ long, Error class/enumeration]
    def get_digital_input_register(self) -> Tuple[int, Error]:
        cmd = [self._address, ConstantUart.READ_DIGITAL_INPUT_REGISTER,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.UINT32), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the value of the voltage sensed at the output of PT1000 temperature
    #			sensor amplifier, this command can be used only on devices that come with PT1000 input
    #          .The method refers to the Uart Write command: 0xC3
    #@retval List of [ long, Error class/enumeration]
    def get_pt1000_sensor_voltage(self) -> Tuple[int, Error]:
        cmd = [self._address, ConstantUart.READ_PT1000_SENSOR_VOLTAGE,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.UINT32), Error.NO_ERROR_DETECTED
        return -1, error

    ##
    #@brief  This command reads the quantized value of an Analogue Input as a number between 0 to 4095
    #           .The method refers to the Uart Write command: 0xC6
    #@param  channel  an enum that specify the Channel of Analogue Input
    #@retval List of [int, Error class/enumeration]
    def get_analogue_input(self, channel: Channel) -> Tuple[int, Error]:
        cmd = [self._address, ConstantUart.READ_ANALOQUE_INPUT,
               0x00, 0x00, 0x00, int(channel)]
        result, error = self.__exec_cmd(cmd)
        if result is True:
            data = get_data(cmd)
            return convert_from_data(data, DataType.UINT32), Error.NO_ERROR_DETECTED
        return -1, error
