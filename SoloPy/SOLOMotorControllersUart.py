# Copyright: (c) 2021, 2022, 2023 SOLO motor controllers project
# GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

# Title: SoloPy
# Author: SOLOMotorControllers
# Date: 2023
# Code version: 3.0.0
# Availability: https://github.com/Solo-FL/SoloPy/tree/main/SoloPy
# This Library is made by SOLOMotorControllers.COM
# please visit:  https://www.SOLOMotorControllers.com/

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

        self._version = "SoloPy v3.0"

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
        self._ser_status = 0
        self._ser = None
        self.serial_open()
        # TODO try solving serial error

    def __del__(self):
        self._logger.debug('SoloMotorController DEL')
        if (self._ser_status == 1):
            self._logger.debug('Serial close')
            try:
                self._ser.close()
            except Exception as e:
                self._logger.error("Exception on Serial Closure")
                # self._logger.error( e, exc_info=True)

    def __exec_cmd(self, cmd: list) -> bool:
        if (self._ser_status == 1):
            try:
                messageLog = "Serial connection is: " + str(self._ser.isOpen())
                self._logger.debug(messageLog)
                if (not self._ser.isOpen()):
                    self._logger.debug("Serial open")
                    self._ser.open()
                    self._logger.debug("Serial flush")
                    self._ser.flush()
                    self._ser.flushInput()
                    self._ser_status = 1

                _cmd = [ConstantUart.INITIATOR, ConstantUart.INITIATOR,
                        cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], ConstantUart.CRC, ConstantUart.ENDING]

                _readPacket = []

                messageLog = "WRITE: " + str([
                    hex(ConstantUart.INITIATOR), hex(ConstantUart.INITIATOR),
                    hex(cmd[0]), hex(cmd[1]), hex(cmd[2]), hex(
                        cmd[3]), hex(cmd[4]), hex(cmd[5]),
                    hex(ConstantUart.CRC), hex(ConstantUart.ENDING)])
                self._logger.debug(messageLog)

                self._ser.write(_cmd)

                # Time to ensure the writing, reducing it can make the comunication instable
                time.sleep(0.1)

                # read up to ten bytes (timeout)
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
                        _readPacket[3] == _cmd[3] and
                        _readPacket[8] == _cmd[8] and
                        _readPacket[9] == _cmd[9]):
                    cmd[0] = _readPacket[2]
                    cmd[1] = _readPacket[3]
                    cmd[2] = _readPacket[4]
                    cmd[3] = _readPacket[5]
                    cmd[4] = _readPacket[6]
                    cmd[5] = _readPacket[7]
                else:
                    cmd[0] = 0xEE
                    cmd[1] = 0xEE
                    cmd[2] = 0xEE
                    cmd[3] = 0xEE
                    cmd[4] = 0xEE
                    cmd[5] = 0xEE

                if (cmd[2] == ConstantUart.ERROR and
                    cmd[3] == ConstantUart.ERROR and
                    cmd[4] == ConstantUart.ERROR and
                        cmd[5] == ConstantUart.ERROR):
                    return False, ERROR.GENERAL_ERROR
                else:
                    return True, ERROR.NO_ERROR_DETECTED
            except Exception as e:
                self._logger.debug('__exec_cmd Exception')
                self._logger.debug(e, exc_info=True)
                self.serial_error_handler()
                return False, ERROR.GENERAL_ERROR

        self._logger.info("Serial status not ready")
        self.serial_open()
        return False, ERROR.GENERAL_ERROR

    # #############################Support############################# #
    def version(self) -> string:
        return self._version

    def serial_open(self) -> bool:
        self._logger.debug("serial_open start")
        try:
            self._ser = serial.Serial(
                self._port, self._baudrate, timeout=self._timeout, writeTimeout=self._timeout)
            self._ser_status = 1

            # Time sleep for ensure serial initialization
            time.sleep(0.2)

        except Exception as e:
            self._logger.error(
                "serial_open: Exception during the serial inizialisation")
            # self._logger.error( e, exc_info=True)
            self._ser_status = -1
            # raise e
            return False

        if (self._ser_status == 1):
            self._logger.debug("Serial init")
            self._ser.bytesize = serial.EIGHTBITS
            self._ser.parity = serial.PARITY_NONE
            self._ser.stopbits = serial.STOPBITS_ONE
            time.sleep(0.2)
            if (not self._ser.isOpen()):
                self._logger.debug("Serial open")
                self._ser.open()

            self._ser.flush()
            self._ser.flushInput()
            self._logger.info("serial_open: success")
            self._logger.debug("serial_open end")
            return True

        self._logger.debug("serial_open end")
        return False

    def serial_error_handler(self) -> bool:
        self._logger.debug('SEH start')
        try:
            try:
                if (self._ser.isOpen()):
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

        if (self._ser_status == 1):
            try:
                self._ser.close()
                self._logger.info("serial_close: Serial closed")
                self._ser_status = 0
                return True
            except Exception as e:
                self._logger.error("serial_close: Exception on Serial Closure")
                # self._logger.error( e, exc_info=True)
                self._ser_status = -1
                return False

        if (self._ser_status == 0):
            self._logger.info("serial_close: Serial is already close")
        if (self._ser_status == -1):
            self._logger.info("serial_close: Serial is in error")
        self._logger.debug("serial_close end")
        return False

    def serial_status(self) -> string:
        status = ''
        if (self._ser_status == 1):
            status += 'SPS: Serial Open'
            try:
                status += '\nSPS: Serial comunication open: ' + \
                    str(self._ser.isOpen())
                if (self._ser.isOpen()):
                    status += '\nSPS: Serial input buffer size: ' + \
                        str(self._ser.out_waiting)
                    status += '\nSPS: Serial output buffer size: ' + \
                        str(self._ser.inWaiting())
            except Exception as e:
                self._logger.error('exception on printing the status')
                # self._logger.error( e, exc_info=True)

        if (self._ser_status == 0):
            status += 'SPS: Serial Close'

        if (self._ser_status == -1):
            status += 'SPS: Serial on Error'
        return status

    def serial_is_working(self) -> bool:
        if (self.get_phase_a_voltage() == -1):
            return False
        return True

    # #############################Write############################# #

    def set_device_address(self, device_address: int) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_device_address_input_validation(device_address)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error

        cmd = [self._address, ConstantUart.WriteDeviceAddress,
               0x00, 0x00, 0x00, device_address]
        return self.__exec_cmd(cmd)

    def set_command_mode(self, mode: COMMAND_MODE) -> bool:
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

    def set_current_limit(self, current_limit: float) -> bool:
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

    def set_torque_reference_iq(self, torque_reference_iq: float) -> bool:
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

    def set_speed_reference(self, speed_reference: int) -> bool:
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

    def set_power_reference(self, power_reference: float) -> bool:
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

    def motor_parameters_identification(self, identification: ACTION) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = motor_parameters_identification_input_validation(identification)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error

        if (type(identification) is int):
            identification = COMMAND_MODE(identification)

        identification = identification.value
        data = convert_to_data(identification, DATA_TYPE.UINT32)
        cmd = [self._address, ConstantUart.WriteMotorParametersIdentification,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def emergency_stop(self) -> bool:
        cmd = [self._address, ConstantUart.WriteEmergencyStop,
               0x00, 0x00, 0x00, 0x00]
        self._logger.info(
            "SOLO should be manually power recycled to get back into normal operation ")
        return self.__exec_cmd(cmd)

    def set_output_pwm_frequency_khz(self, output_pwm_frequency_khz: int) -> bool:
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

    def set_speed_controller_kp(self, speed_controller_kp: float) -> bool:
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

    def set_speed_controller_ki(self, speed_controller_ki: float) -> bool:
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

    def set_motor_direction(self, motor_direction: DIRECTION) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_motor_direction_input_validation(motor_direction)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error

        if (type(motor_direction) is int):
            motor_direction = COMMAND_MODE(motor_direction)

        motor_direction = motor_direction.value
        data = convert_to_data(motor_direction, DATA_TYPE.UINT32)
        cmd = [self._address, ConstantUart.WriteMotorDirection,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_motor_resistance(self, motor_resistance: float) -> bool:
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

    def set_motor_inductance(self, motor_inductance: float) -> bool:
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

    def set_motor_poles_counts(self, motor_poles_counts: int) -> bool:
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

    def set_incremental_encoder_lines(self, incremental_encoder_lines: int) -> bool:
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

    def set_speed_limit(self, speed_limit: int) -> bool:
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

    def reset_address(self) -> bool:
        cmd = [0xFF, ConstantUart.WriteResetAddress, 0x00, 0x00, 0x00, 0xFF]
        return self.__exec_cmd(cmd)

    def set_feedback_control_mode(self, mode: FEEDBACK_CONTROL_MODE) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_feedback_control_mode_input_validation(mode)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error

        if (type(mode) is int):
            mode = COMMAND_MODE(mode)

        mode = mode.value
        data = convert_to_data(mode, DATA_TYPE.UINT32)
        cmd = [self._address, ConstantUart.WriteFeedbackControlMode,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def reset_factory(self) -> bool:
        cmd = [self._address, ConstantUart.WriteResetFactory,
               0x00, 0x00, 0x00, 0x01]
        return self.__exec_cmd(cmd)

    def set_motor_type(self, motor_type: MOTOR_TYPE) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_motor_type_input_validation(motor_type)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error

        if (type(motor_type) is int):
            motor_type = COMMAND_MODE(motor_type)

        motor_type = motor_type.value
        data = convert_to_data(motor_type, DATA_TYPE.UINT32)
        cmd = [self._address, ConstantUart.WriteMotorType,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_control_mode(self, control_mode: CONTROL_MODE) -> bool:
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

    def set_current_controller_kp(self, current_controller_kp: float) -> bool:
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

    def set_current_controller_ki(self, current_controller_ki: float) -> bool:
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
    # def set_monitoring_mode(self, mode: int) -> bool:
    #     if (mode < 0 or mode > 2):
    #         self._logger.info(ConstantCommon.InputOutOfRange)
    #         return False
    #     mode = int(mode)
    #     data = convert_to_data(mode, DATA_TYPE.UINT32)
    #     cmd = [self._address, ConstantUart.WriteMonitoringMode,
    #            data[0], data[1], data[2], data[3]]
    #     return self.__exec_cmd(cmd)

    def set_magnetizing_current_id_reference(self, magnetizing_current_id_reference: float) -> bool:
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

    def set_position_reference(self, position_reference: int) -> bool:
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

    def set_position_controller_kp(self, position_controller_kp: float) -> bool:
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

    def set_position_controller_ki(self, position_controller_ki: float) -> bool:
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

    # def reset_position_to_zero(self) -> bool:
    #     cmd = [self._address, ConstantUart.WriteResetPositionToZero,
    #            0x00, 0x00, 0x00, 0x01]
    #     return self.__exec_cmd(cmd)

    def overwrite_error_register(self) -> bool:
        cmd = [self._address, ConstantUart.WriteOverwriteErrorRegister,
               0x00, 0x00, 0x00, 0x00]
        return self.__exec_cmd(cmd)

    # SOG => Sensorless Observer Gain
    def set_observer_gain_bldc_pmsm(self, observer_gain: float) -> bool:
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

    def set_observer_gain_bldc_pmsm_ultrafast(self, observer_gain: float) -> bool:
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

    def set_observer_gain_dc(self, observer_gain: float) -> bool:
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

    def set_filter_gain_bldc_pmsm(self, filter_gain: float) -> bool:
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

    def set_filter_gain_bldc_pmsm_ultrafast(self, filter_gain: float) -> bool:
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

    def set_uart_baudrate(self, baudrate: UART_BAUD_RATE) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_uart_baudrate_input_validation(baudrate)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error

        if (type(baudrate) is int):
            baudrate = COMMAND_MODE(baudrate)

        baudrate = baudrate.value
        data = convert_to_data(baudrate, DATA_TYPE.UINT32)
        cmd = [self._address, ConstantUart.WriteUartBaudRate,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def sensor_calibration(self, calibration_action: POSITION_SENSOR_CALIBRATION_ACTION) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = sensor_calibration_input_validation(calibration_action)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error

        if (type(calibration_action) is int):
            calibration_action = COMMAND_MODE(calibration_action)

        calibration_action = calibration_action.value
        data = convert_to_data(calibration_action, DATA_TYPE.UINT32)
        cmd = [self._address, ConstantUart.WriteSensorCalibration,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_encoder_hall_ccw_offset(self, encoder_hall_offset: float) -> bool:
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

    def set_encoder_hall_cw_offset(self, encoder_hall_offset: float) -> bool:
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

    def set_speed_acceleration_value(self, speed_acceleration_value: float) -> bool:
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

    def set_speed_deceleration_value(self, speed_deceleration_value: float) -> bool:
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

    def set_can_bus_baudrate(self, canbus_baudrate: CAN_BUS_BAUD_RATE) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        InputValidate, error, logMsg = set_can_bus_baudrate_input_validation(canbus_baudrate)
        if (InputValidate is False):
            self._logger.info(logMsg)
            return False, error

        if (type(canbus_baudrate) is int):
            canbus_baudrate = COMMAND_MODE(canbus_baudrate)

        canbus_baudrate = canbus_baudrate.value
        data = convert_to_data(canbus_baudrate, DATA_TYPE.UINT32)
        cmd = [self._address, ConstantUart.WriteCanBusBaudRate,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)
# ----------------------------------------------
# ---------------------Read---------------------
# ----------------------------------------------

    def get_device_address(self) -> int:
        cmd = [0xFF, ConstantUart.ReadDeviceAddress, 0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.UINT32), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_phase_a_voltage(self) -> float:
        cmd = [self._address, ConstantUart.ReadPhaseAVoltage, 0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_phase_b_voltage(self) -> float:
        cmd = [self._address, ConstantUart.ReadPhaseBVoltage, 0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_phase_a_current(self) -> float:
        cmd = [self._address, ConstantUart.ReadPhaseACurrent, 0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_phase_b_current(self) -> float:
        cmd = [self._address, ConstantUart.ReadPhaseBCurrent, 0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_bus_voltage(self) -> float:
        cmd = [self._address, ConstantUart.ReadBusVoltage, 0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_dc_motor_current_im(self) -> float:
        cmd = [self._address, ConstantUart.ReadDcMotorCurrentIm,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_dc_motor_voltage_vm(self) -> float:
        cmd = [self._address, ConstantUart.ReadDcMotorVoltageVm,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_speed_controller_kp(self) -> float:
        cmd = [self._address, ConstantUart.ReadSpeedControllerKp,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_speed_controller_ki(self) -> float:
        cmd = [self._address, ConstantUart.ReadSpeedControllerKi,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_output_pwm_frequency_khz(self) -> int:
        cmd = [self._address, ConstantUart.ReadOutputPwmFrequencyHz,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return (convert_from_data(data, DATA_TYPE.UINT32) / 1000), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_current_limit(self) -> float:
        cmd = [self._address, ConstantUart.ReadCurrentLimit,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_quadrature_current_iq_feedback(self) -> float:
        cmd = [self._address, ConstantUart.ReadQuadratureCurrentIqFeedback,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_magnetizing_current_id_feedback(self) -> float:
        cmd = [self._address, ConstantUart.ReadMagnetizingCurrentIdFeedback,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_motor_poles_counts(self) -> int:
        cmd = [self._address, ConstantUart.ReadMotorPolesCounts,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.UINT32), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_incremental_encoder_lines(self) -> int:
        cmd = [self._address, ConstantUart.ReadIncrementalEncoderLines,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.UINT32), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_current_controller_kp(self) -> float:
        cmd = [self._address, ConstantUart.ReadCurrentControllerKp,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_current_controller_ki(self) -> float:
        cmd = [self._address, ConstantUart.ReadCurrentControllerKi,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT) * 0.00005, ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_board_temperature(self) -> float:
        cmd = [self._address, ConstantUart.ReadBoardTemperature,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_motor_resistance(self) -> float:
        cmd = [self._address, ConstantUart.ReadMotorResistance,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT) * 0.00005, ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_motor_inductance(self) -> float:
        cmd = [self._address, ConstantUart.ReadMotorInductance,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_speed_feedback(self) -> int:
        cmd = [self._address, ConstantUart.ReadSpeedFeedback,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.INT32), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_motor_type(self) -> MOTOR_TYPE:
        cmd = [self._address, ConstantUart.ReadMotorType,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return MOTOR_TYPE(convert_from_data(data, DATA_TYPE.UINT32)), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_feedback_control_mode(self) -> FEEDBACK_CONTROL_MODE:
        cmd = [self._address, ConstantUart.ReadFeedbackControlMode,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return FEEDBACK_CONTROL_MODE(convert_from_data(data, DATA_TYPE.UINT32)), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_command_mode(self) -> COMMAND_MODE:
        cmd = [self._address, ConstantUart.ReadCommandMode,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return COMMAND_MODE(convert_from_data(data, DATA_TYPE.UINT32)), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_control_mode(self) -> CONTROL_MODE:
        cmd = [self._address, ConstantUart.ReadControlMode,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return CONTROL_MODE(convert_from_data(data, DATA_TYPE.UINT32)), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_speed_limit(self) -> int:
        cmd = [self._address, ConstantUart.ReadSpeedLimit,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.UINT32), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_position_controller_kp(self) -> float:
        cmd = [self._address, ConstantUart.ReadPositionControllerKp,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_position_controller_ki(self) -> float:
        cmd = [self._address, ConstantUart.ReadPositionControllerKi,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_position_counts_feedback(self) -> int:
        cmd = [self._address, ConstantUart.ReadPositionCountsFeedback,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.INT32), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_error_register(self) -> int:
        cmd = [self._address, ConstantUart.ReadErrorRegister,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.UINT32), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_device_firmware_version(self) -> int:
        cmd = [self._address, ConstantUart.ReadDeviceFirmwareVersion,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.UINT32), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_device_hardware_version(self) -> int:
        cmd = [self._address, ConstantUart.ReadDeviceHardwareVersion,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.UINT32), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_torque_reference_iq(self) -> float:
        cmd = [self._address, ConstantUart.ReadTorqueReferenceIq,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_speed_reference(self) -> int:
        cmd = [self._address, ConstantUart.ReadSpeedReference,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.UINT32), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_magnetizing_current_id_reference(self) -> float:
        cmd = [self._address, ConstantUart.ReadMagnetizingCurrentIdReference,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_position_reference(self) -> int:
        cmd = [self._address, ConstantUart.ReadPositionReference,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.INT32), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_power_reference(self) -> float:
        cmd = [self._address, ConstantUart.ReadPowerReference,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_motor_direction(self) -> DIRECTION:
        cmd = [self._address, ConstantUart.ReadMotorDirection,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return DIRECTION(convert_from_data(data, DATA_TYPE.UINT32)), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_observer_gain_bldc_pmsm(self) -> float:
        cmd = [self._address, ConstantUart.ReadObserverGainBldcPmsm,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_observer_gain_bldc_pmsm_ultrafast(self) -> float:
        cmd = [self._address, ConstantUart.ReadObserverGainBldcPmsmUltrafast,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_observer_gain_dc(self) -> float:
        cmd = [self._address, ConstantUart.ReadObserverGainDc,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_filter_gain_bldc_pmsm(self) -> float:
        cmd = [self._address, ConstantUart.ReadFilterGainBldcPmsm,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_filter_gain_bldc_pmsm_ultrafast(self) -> float:
        cmd = [self._address, ConstantUart.ReadFilterGainBldcPmsmUltrafast,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_3phase_motor_angle(self) -> float:
        cmd = [self._address, ConstantUart.Read3PhaseMotorAngle,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_encoder_hall_ccw_offset(self) -> float:
        cmd = [self._address, ConstantUart.ReadEncoderHallCcwOffset,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_encoder_hall_cw_offset(self) -> float:
        cmd = [self._address, ConstantUart.ReadEncoderHallCwOffset,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_uart_baudrate(self) -> UART_BAUD_RATE:
        cmd = [self._address, ConstantUart.ReadUartBaudRate,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return UART_BAUD_RATE(convert_from_data(data, DATA_TYPE.UINT32)), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_speed_acceleration_value(self) -> float:
        cmd = [self._address, ConstantUart.ReadSpeedAccelerationValue,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_speed_deceleration_value(self) -> float:
        cmd = [self._address, ConstantUart.ReadSpeedDecelerationValue,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.SFXT), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def communication_is_working(self) -> bool:
        error = ERROR.NO_PROCESSED_COMMAND
        temperature, error = self.get_board_temperature()
        time.sleep(0.2)
        temperature, error = self.get_board_temperature()
        if (error == ERROR.NO_ERROR_DETECTED):
            return True, error
        return False, error

    def get_encoder_index_counts(self) -> int:
        cmd = [self._address, ConstantUart.ReadEncoderIndexCounts,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return convert_from_data(data, DATA_TYPE.UINT32), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error

    def get_can_bus_baudrate(self) -> CAN_BUS_BAUD_RATE:
        cmd = [self._address, ConstantUart.ReadCanBusBaudRate,
               0x00, 0x00, 0x00, 0x00]
        result, error = self.__exec_cmd(cmd)
        if (result):
            data = get_data(cmd)
            return CAN_BUS_BAUD_RATE(convert_from_data(data, DATA_TYPE.UINT32)), ERROR.NO_ERROR_DETECTED
        else:
            return -1, error
