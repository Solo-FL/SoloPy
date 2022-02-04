import serial
import string
import SoloPy.constant as constant
import math
import time
import logging
from importlib import reload
from enum import Enum


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
    BLDC_PMSM  = 1
    ACIM = 2
    BLDC_PMSM_ULTRAFAST = 3 

class UART_BAUD_RATE(Enum):
    RATE_937500 = 0
    RATE_115200 =1

class CAN_BUS_BAUD_RATE(Enum):
    RATE_1000 = 0
    RATE_500 = 1
    RATE_250 = 2
    RATE_125 = 3
    RATE_100 = 4

class DATA_TYPE(Enum):
    UINT32 = 0
    INT32 = 1
    SFXT  = 2

class ACTION (Enum):
    STOP = 0
    START = 1

class POSITION_SENSOR_CALIBRATION_ACTION (Enum):
    STOP_CALIBRATION = 0
    INCREMENTAL_ENCODER_START_CALIBRATION = 1
    HALL_SENSOR_START_CALIBRATION  = 2 


class SoloMotorController:

    def __init__(
        self,
        port="/dev/ttyS0",
        address=0,
        baudrate=937500,
        timeout=3,
        loggerLevel=logging.INFO):

        self._version = "SoloPy v2.4"
        
        # logger init
        logging.shutdown()
        reload(logging)
        self._logger = logging.getLogger('SoloPy')
        self._logger.setLevel(loggerLevel)
        ch = logging.StreamHandler()
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        ch.setFormatter(formatter)
        self._logger.addHandler(ch)
        # logger init end
        self._logger.debug('SoloMotorController INIT')

        self._address = address
        self._baudrate = baudrate
        self._port = port
        self._timeout = timeout
        self._ser_status = 0
        self._ser = None
        self.serial_open()
        # TODO try solving serial error
    
    def __del__(self):
        self._logger.debug('SoloMotorController DEL')
        if(self._ser_status == 1):
            self._logger.debug('Serial close')
            try:
                self._ser.close()
            except Exception as e:
                self._logger.error("Exception on Serial Closure")
                # self._logger.error( e , exc_info=True)

    def __exec_cmd(self, cmd: list) -> bool:
        if(self._ser_status == 1):
            try:
                messageLog = "Serial connection is: " + str(self._ser.isOpen())
                self._logger.debug(messageLog)
                if(not self._ser.isOpen()):
                    self._logger.debug("Serial open")
                    self._ser.open()
                    self._logger.debug("Serial flush")
                    self._ser.flush()
                    self._ser.flushInput()
                    self._ser_status = 1
                    
                _cmd = [constant.INITIATOR, constant.INITIATOR,
                        cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], constant.CRC, constant.ENDING]

                _readPacket = []

                messageLog = "WRITE: " + str([
                    hex(constant.INITIATOR), hex(constant.INITIATOR),
                    hex(cmd[0]), hex(cmd[1]), hex(cmd[2]), hex(cmd[3]), hex(cmd[4]), hex(cmd[5]),
                    hex(constant.CRC), hex(constant.ENDING)])
                self._logger.debug(messageLog)
                
                self._ser.write(_cmd)

                # Time to ensure the writing, reducing it can make the comunication instable
                time.sleep(0.1)

                # read up to ten bytes (timeout)
                _readPacket = self._ser.read(10)
                messageLog = "READ: " + str([hex(i) for i in _readPacket]) + " size: " + str(len(_readPacket))
                self._logger.debug(messageLog)
                if(len(_readPacket) == 0):
                    self._ser.flush()

                if (_readPacket and 
                        _readPacket[0] == _cmd[0] and
                        _readPacket[1] == _cmd[1] and
                        (_readPacket[2] == _cmd[2] or _cmd[2]== 0xFF) and
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

                if (cmd[2] == constant.ERROR and
                    cmd[3] == constant.ERROR and
                    cmd[4] == constant.ERROR and
                    cmd[5] == constant.ERROR):
                    return False
                else:
                    return True
            except Exception as e:
                self._logger.debug('__exec_cmd Exception')
                self._logger.debug(e, exc_info=True)
                self.serial_error_handler()
                return False

        self._logger.info("Serial status not ready")
        return False

    def __convert_from_data(self, data, dataType : DATA_TYPE):
        if(dataType == DATA_TYPE.SFXT):
            return self.__convert_to_float(data)

        if(dataType == DATA_TYPE.INT32):
            return self.__convert_to_int(data)

        if(dataType == DATA_TYPE.UINT32):
            return self.__convert_to_long(data)

    def __convert_to_float(self, data) -> float:
        dec = 0
        dec = int.from_bytes(
            [data[0], data[1], data[2], data[3]], byteorder='big', signed=False)
        if(dec <= 0x7FFE0000):
            value = (float)(dec/131072.0)
            return float(format(value,'f')) #.8f
        else:
            dec = 0xFFFFFFFF - dec + 1
            value =((float)(dec/131072.0)) * -1
            return float(format(value,'f')) #.8f

    def __convert_to_long(self, data) -> int:
        dec = 0
        dec = int.from_bytes(
            [data[0], data[1], data[2], data[3]], byteorder='big', signed=False)
        return dec
        
    def __convert_to_int(self, data) -> int:
        dec = 0
        dec = int.from_bytes(
            [data[0], data[1], data[2], data[3]], byteorder='big', signed=True)
        return dec

    def __convert_to_data(self, number, dataType : DATA_TYPE ) -> list:
        data = []
        if(dataType == DATA_TYPE.SFXT):
            dec = math.ceil(number * 131072)
            if dec < 0:
                dec *= -1
                dec = 0xFFFFFFFF - dec
            data = [(dec >> i & 0xff) for i in (24, 16, 8, 0)] 

        if(dataType == DATA_TYPE.UINT32 or dataType == DATA_TYPE.INT32):
            dec = number
            data = [(dec >> i & 0xff) for i in (24, 16, 8, 0)]
            
        return data

    def __get_data(self, cmd: list) -> list:
        return [cmd[2], cmd[3], cmd[4], cmd[5]]

    def set_device_address(self, address: int) -> bool:
        cmd = [self._address, constant.WriteDeviceAddress, 0x00, 0x00, 0x00, address]
        if(address < 0 or address > 254):
            return False
        return self.__exec_cmd(cmd)

    # #############################Support############################# #
    def version(self) -> string:
        return self._version

    def serial_open(self) -> bool:
        self._logger.debug("serial_open start")
        try:
            self._ser = serial.Serial(self._port, self._baudrate, timeout=self._timeout, writeTimeout=self._timeout)
            self._ser_status = 1

            # Time sleep for ensure serial initialization
            time.sleep(0.2)
            
        except Exception as e:
            self._logger.error("serial_open: Exception during the serial inizialisation")
            # self._logger.error( e , exc_info=True)
            self._ser_status = -1
            # raise e
            return False

        if(self._ser_status == 1):
            self._logger.debug("Serial init")
            self._ser.bytesize = serial.EIGHTBITS
            self._ser.parity = serial.PARITY_NONE
            self._ser.stopbits = serial.STOPBITS_ONE
            time.sleep(0.2)
            if(not self._ser.isOpen()):
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
                if(self._ser.isOpen()):
                    self._logger.debug('SEH: serial is open')
                    if(self._ser.inWaiting() > 0):
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
            # self._logger.error( e , exc_info=True)
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
                # self._logger.error( e , exc_info=True)
                self._ser_status = -1
                return False

        if(self._ser_status == 0):
            self._logger.info("serial_close: Serial is already close")
        if(self._ser_status == -1):
            self._logger.info("serial_close: Serial is in error")
        self._logger.debug("serial_close end")
        return False

    def serial_status(self) -> string:
        status = ''
        if(self._ser_status == 1):
            status += 'SPS: Serial Open'
            try:
                status += '\nSPS: Serial comunication open: ' + str(self._ser.isOpen())
                if(self._ser.isOpen()):
                    status += '\nSPS: Serial input buffer size: ' + str(self._ser.out_waiting)
                    status += '\nSPS: Serial output buffer size: ' + str(self._ser.inWaiting())
            except Exception as e:
                self._logger.error('exception on printing the status')
                # self._logger.error( e , exc_info=True)

        if(self._ser_status == 0):
            status += 'SPS: Serial Close'
        
        if(self._ser_status == -1):
            status += 'SPS: Serial on Error'
        return status
        
    def serial_is_working(self) -> bool:
        if(self.get_phase_a_voltage() == -1):
            return False
        return True

    # #############################Write############################# #

    def set_command_mode(self, mode: COMMAND_MODE) -> bool:
        if not isinstance(mode, COMMAND_MODE):
            if(type(mode) is int and (mode == 0 or mode == 1)):
                mode = COMMAND_MODE(mode)
            else:
                self._logger.info(constant.InputNeedEnum)
                return False
        mode = mode.value
        data = self.__convert_to_data(mode, DATA_TYPE.UINT32)
        cmd = [self._address, constant.WriteCommandMode,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_current_limit(self, limit_number: float) -> bool:
        if (limit_number < 0 or limit_number > 32):
            return False
        limit_number = float(limit_number)
        data = self.__convert_to_data(limit_number, DATA_TYPE.SFXT)
        cmd = [self._address, constant.WriteCurrentLimit,
               data[0], data[1], data[2], data[3]]

        return self.__exec_cmd(cmd)

    def set_torque_reference_iq(self, torque_number: float) -> bool:
        if (torque_number < 0 or torque_number > 32):
            self._logger.info(constant.InputOutOfRange)
            return False
        torque_number = float(torque_number)
        data = self.__convert_to_data(torque_number, DATA_TYPE.SFXT)
        cmd = [self._address, constant.WriteTorqueReferenceIq,
               data[0], data[1], data[2], data[3]]

        return self.__exec_cmd(cmd)

    def set_speed_reference(self, speed_number: int) -> bool:
        if (speed_number < 0 or speed_number > 30000):
            self._logger.info(constant.InputOutOfRange)
            return False
        speed_number = int(speed_number)
        data = self.__convert_to_data(speed_number, DATA_TYPE.UINT32)
        cmd = [self._address, constant.WriteSpeedReference,
               data[0], data[1], data[2], data[3]]

        return self.__exec_cmd(cmd)

    def set_power_reference(self, power_number: float) -> bool:
        if (power_number < 0 or power_number > 100):
            self._logger.info(constant.InputOutOfRange)
            return False
        power_number = float(power_number)
        data = self.__convert_to_data(power_number, DATA_TYPE.SFXT)
        cmd = [self._address, constant.WritePowerReference,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def motor_parameters_identification(self, identification: ACTION) -> bool:
        if not isinstance(identification, ACTION):
            if(type(identification) is int and (identification == 0 or identification == 1)):
                identification = ACTION(identification)
            else:
                self._logger.info(constant.InputNeedEnum)
                return False
        identification = identification.value
        data = self.__convert_to_data(identification, DATA_TYPE.UINT32)
        cmd = [self._address, constant.WriteMotorParametersIdentification,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def emergency_stop(self) -> bool:
        cmd = [self._address, constant.WriteEmergencyStop,
               0x00, 0x00, 0x00, 0x00]
        self._logger.info("SOLO should be manually power recycled to get back into normal operation ")
        return self.__exec_cmd(cmd)

    def set_output_pwm_frequency_khz(self, pwm: int) -> bool:
        if (pwm < 8 or pwm > 80):
            self._logger.info(constant.InputOutOfRange)
            return False
        pwm = int(pwm)
        data = self.__convert_to_data(pwm, DATA_TYPE.UINT32)
        cmd = [self._address, constant.WriteOutputPwmFrequencyKhz,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_speed_controller_kp(self, kp: float) -> bool:
        if (kp < 0 or kp > 300):
            self._logger.info(constant.InputOutOfRange)
            return False
        kp = float(kp)
        data = self.__convert_to_data(kp, DATA_TYPE.SFXT)
        cmd = [self._address, constant.WriteSpeedControllerKp,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_speed_controller_ki(self, ki: float) -> bool:
        if (ki < 0 or ki > 300):
            self._logger.info(constant.InputOutOfRange)
            return False
        ki = float(ki)
        data = self.__convert_to_data(ki, DATA_TYPE.SFXT)
        cmd = [self._address, constant.WriteSpeedControllerKi,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_motor_direction(self, dir: DIRECTION) -> bool:
        if not isinstance(dir, DIRECTION):
            if(type(dir) is int and (dir == 0 or dir == 1)):
                dir = DIRECTION(dir)
            else:
                self._logger.info(constant.InputNeedEnum)
                return False
        dir = dir.value
        data = self.__convert_to_data(dir, DATA_TYPE.UINT32)
        cmd = [self._address, constant.WriteMotorDirection,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_motor_resistance(self, res: float) -> bool:
        if (res < 0.001 or res > 50):
            self._logger.info(constant.InputOutOfRange)
            return False
        res = float(res)
        data = self.__convert_to_data(res, DATA_TYPE.SFXT)
        cmd = [self._address, constant.WriteMotorResistance,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_motor_inductance(self, ind: float) -> bool:
        if (ind < 0.00001 or ind > 0.2):
            self._logger.info(constant.InputOutOfRange)
            return False
        ind = float(ind)
        data = self.__convert_to_data(ind, DATA_TYPE.SFXT)
        cmd = [self._address, constant.WriteMotorInductance,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_motor_poles_counts(self, poles: int) -> bool:
        if (poles < 1 or poles > 80):
            self._logger.info(constant.InputOutOfRange)
            return False
        poles = int(poles)
        data = self.__convert_to_data(poles, DATA_TYPE.UINT32)
        cmd = [self._address, constant.WriteMotorPolesCounts,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_incremental_encoder_lines(self, enc: int) -> bool:
        if (enc < 1 or enc > 40000):
            self._logger.info(constant.InputOutOfRange)
            return False
        enc = int(enc)
        data = self.__convert_to_data(enc, DATA_TYPE.UINT32)
        cmd = [self._address, constant.WriteIncrementalEncoderLines,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_speed_limit(self, speed: int) -> bool:
        if (speed < 1 or speed > 30000):
            self._logger.info(constant.InputOutOfRange)
            return False
        speed = int(speed)
        data = self.__convert_to_data(speed, DATA_TYPE.UINT32)
        cmd = [self._address, constant.WriteSpeedLimit,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def reset_address(self) -> bool:
        cmd = [0xFF, constant.WriteResetAddress, 0x00, 0x00, 0x00, 0xFF]
        return self.__exec_cmd(cmd)

    def set_feedback_control_mode(self, mode: FEEDBACK_CONTROL_MODE) -> bool:
        if not isinstance(mode, FEEDBACK_CONTROL_MODE):
            if(type(mode) is int and (mode >= 0 and mode <= 2)):
                mode = FEEDBACK_CONTROL_MODE(mode)
            else:
                self._logger.info(constant.InputNeedEnum)
                return False
        mode = mode.value
        data = self.__convert_to_data(mode, DATA_TYPE.UINT32)
        cmd = [self._address, constant.WriteFeedbackControlMode,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def reset_factory(self) -> bool:
        cmd = [self._address, constant.WriteResetFactory,
               0x00, 0x00, 0x00, 0x01]
        return self.__exec_cmd(cmd)

    def set_motor_type(self, motorType: MOTOR_TYPE) -> bool:
        if not isinstance(motorType, MOTOR_TYPE):
            if(type(motorType) is int and (motorType >= 0 and motorType <= 3)):
                motorType = MOTOR_TYPE(motorType)
            else:
                self._logger.info(constant.InputNeedEnum)
                return False
        motorType = motorType.value
        data = self.__convert_to_data(motorType, DATA_TYPE.UINT32)
        cmd = [self._address, constant.WriteMotorType,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_control_mode(self, mode: CONTROL_MODE) -> bool:
        if not isinstance(mode, CONTROL_MODE):
            if(type(mode) is int and (mode >= 0 and mode <= 2)):
                mode = CONTROL_MODE(mode)
            else:
                self._logger.info(constant.InputNeedEnum)
                return False
        mode = mode.value
        data = self.__convert_to_data(mode, DATA_TYPE.UINT32)
        cmd = [self._address, constant.WriteControlMode,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_current_controller_kp(self, kp: float) -> bool:
        if (kp < 0 or kp > 16000):
            self._logger.info(constant.InputOutOfRange)
            return False
        kp = float(kp)
        data = self.__convert_to_data(kp, DATA_TYPE.SFXT)
        cmd = [self._address, constant.WriteCurrentControllerKp,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_current_controller_ki(self, ki: float) -> bool:
        if (ki < 0 or ki > 16000):
            self._logger.info(constant.InputOutOfRange)
            return False
        ki = float(ki)
        data = self.__convert_to_data(ki, DATA_TYPE.SFXT)
        cmd = [self._address, constant.WriteCurrentControllerKi,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_monitoring_mode(self, mode: int) -> bool:
        if (mode < 0 or mode > 2):
            self._logger.info(constant.InputOutOfRange)
            return False
        mode = int(mode)
        data = self.__convert_to_data(mode, DATA_TYPE.UINT32)
        cmd = [self._address, constant.WriteMonitoringMode,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_magnetizing_current_id_reference(self, mg: float) -> bool:
        if (mg < 0 or mg > 32):
            self._logger.info(constant.InputOutOfRange)
            return False
        mg = float(mg)
        data = self.__convert_to_data(mg, DATA_TYPE.SFXT)
        cmd = [self._address, constant.WriteMagnetizingCurrentIdReference,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_position_reference(self, pos: int) -> bool:
        if (pos < -2147483647 or pos > 2147483647):
            self._logger.info(constant.InputOutOfRange)
            return False
        pos = int(pos)
        data = self.__convert_to_data(pos, DATA_TYPE.INT32)
        cmd = [self._address, constant.WritePositionReference,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_position_controller_kp(self, kp: float) -> bool:
        if (kp < 0 or kp > 16000):
            self._logger.info(constant.InputOutOfRange)
            return False
        kp = float(kp)
        data = self.__convert_to_data(kp, DATA_TYPE.SFXT)
        cmd = [self._address, constant.WritePositionControllerKp,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_position_controller_ki(self, ki: float) -> bool:
        if (ki < 0 or ki > 16000):
            self._logger.info(constant.InputOutOfRange)
            return False
        ki = float(ki)
        data = self.__convert_to_data(ki, DATA_TYPE.SFXT)
        cmd = [self._address, constant.WritePositionControllerKi,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def reset_position_to_zero(self) -> bool:
        cmd = [self._address, constant.WriteResetPositionToZero,
               0x00, 0x00, 0x00, 0x01]
        return self.__exec_cmd(cmd)

    def overwrite_error_register(self) -> bool:
        cmd = [self._address, constant.WriteOverwriteErrorRegister,
               0x00, 0x00, 0x00, 0x00]
        return self.__exec_cmd(cmd)

    # SOG => Sensorless Observer Gain
    def set_observer_gain_bldc_pmsm(self, G: float) -> bool:
        if (G < 0.01 or G > 1000):
            self._logger.info(constant.InputOutOfRange)
            return False
        G = float(G)
        data = self.__convert_to_data(G, DATA_TYPE.SFXT)
        cmd = [self._address, constant.WriteObserverGainBldcPmsm,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_observer_gain_bldc_pmsm_ultrafast(self, G: float) -> bool:
        if (G < 0.01 or G > 1000):
            self._logger.info(constant.InputOutOfRange)
            return False
        G = float(G)
        data = self.__convert_to_data(G, DATA_TYPE.SFXT)
        cmd = [self._address, constant.WriteObserverGainBldcPmsmUltrafast,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_observer_gain_dc(self, G: float) -> bool:
        if (G < 0.01 or G > 1000):
            self._logger.info(constant.InputOutOfRange)
            return False
        G = float(G)
        data = self.__convert_to_data(G, DATA_TYPE.SFXT)
        cmd = [self._address, constant.WriteObserverGainDc,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    # SOFG = > Sensorless Observer Filter Gain
    def set_filter_gain_bldc_pmsm(self, G: float) -> bool:
        if (G < 0.01 or G > 16000):
            self._logger.info(constant.InputOutOfRange)
            return False
        G = float(G)
        data = self.__convert_to_data(G, DATA_TYPE.SFXT)
        cmd = [self._address, constant.WriteFilterGainBldcPmsm,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_filter_gain_bldc_pmsm_ultrafast(self, G: float) -> bool:
        if (G < 0.01 or G > 16000):
            self._logger.info(constant.InputOutOfRange)
            return False
        G = float(G)
        data = self.__convert_to_data(G, DATA_TYPE.SFXT)
        cmd = [self._address, constant.WriteFilterGainBldcPmsmUltrafast,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_uart_baudrate(self, baudrate: UART_BAUD_RATE) -> bool:
        if not isinstance(baudrate, UART_BAUD_RATE):
            if(type(baudrate) is int and (baudrate == 0 or baudrate == 1)):
                baudrate = UART_BAUD_RATE(baudrate)
            else:
                self._logger.info(constant.InputNeedEnum)
                return False
        baudrate = baudrate.value
        data = self.__convert_to_data(baudrate, DATA_TYPE.UINT32)
        cmd = [self._address, constant.WriteUartBaudRate,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def sensor_calibration(self, calibrationAction: POSITION_SENSOR_CALIBRATION_ACTION) -> bool:
        if not isinstance(calibrationAction, POSITION_SENSOR_CALIBRATION_ACTION):
            if(type(calibrationAction) is int and (calibrationAction >= 0 and calibrationAction <= 2)):
                calibrationAction = POSITION_SENSOR_CALIBRATION_ACTION(calibrationAction)
            else:
                self._logger.info(constant.InputNeedEnum)
                return False
        calibrationAction = calibrationAction.value
        data = self.__convert_to_data(calibrationAction, DATA_TYPE.UINT32)
        cmd = [self._address, constant.WriteSensorCalibration,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_encoder_hall_ccw_offset(self, offset: float) -> bool:
        if (offset <= 0 or offset >= 1):
            self._logger.info(constant.InputOutOfRange)
            return False
        offset = float(offset)
        data = self.__convert_to_data(offset, DATA_TYPE.SFXT)
        cmd = [self._address, constant.WriteEncoderHallCcwOffset,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_encoder__hall_cw_offset(self, offset: float) -> bool:
        if (offset <= 0 or offset >= 1):
            self._logger.info(constant.InputOutOfRange)
            return False
        offset = float(offset)
        data = self.__convert_to_data(offset, DATA_TYPE.SFXT)
        cmd = [self._address, constant.WriteEncoderHallCwOffset,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_speed_acceleration_value(self, accelerationValue: float) -> bool:
        if (accelerationValue < 0 or accelerationValue > 1600):
            self._logger.info(constant.InputOutOfRange)
            return False
        accelerationValue = float(accelerationValue)
        data = self.__convert_to_data(accelerationValue, DATA_TYPE.SFXT)
        cmd = [self._address, constant.WriteSpeedAccelerationValue,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_speed_deceleration_value(self, decelerationValue: float) -> bool:
        if (decelerationValue < 0 or decelerationValue > 1600):
            self._logger.info(constant.InputOutOfRange)
            return False
        decelerationValue = float(decelerationValue)
        data = self.__convert_to_data(decelerationValue, DATA_TYPE.SFXT)
        cmd = [self._address, constant.WriteSpeedDecelerationValue,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_can_bus_baudrate(self, baudrate: CAN_BUS_BAUD_RATE) -> bool:
        if not isinstance(baudrate, CAN_BUS_BAUD_RATE):
            if(type(baudrate) is int and (baudrate >= 0 and baudrate <= 4)):
                baudrate = CAN_BUS_BAUD_RATE(baudrate)
            else:
                self._logger.info(constant.InputNeedEnum)
                return False
        baudrate = baudrate.value
        data = self.__convert_to_data(baudrate, DATA_TYPE.UINT32)
        cmd = [self._address, constant.WriteCanBusBaudRate,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    # #############################Read############################# #

    def get_device_address(self) -> int:
        cmd = [0xFF, constant.ReadDeviceAddress, 0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_from_data(data, DATA_TYPE.UINT32)
        else:
            return -1

    def get_phase_a_voltage(self) -> float:
        cmd = [self._address, constant.ReadPhaseAVoltage, 0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_from_data(data, DATA_TYPE.SFXT)
        else:
            return -1

    def get_phase_b_voltage(self) -> float:
        cmd = [self._address, constant.ReadPhaseBVoltage, 0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_from_data(data, DATA_TYPE.SFXT)
        else:
            return -1

    def get_phase_a_current(self) -> float:
        cmd = [self._address, constant.ReadPhaseACurrent, 0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_from_data(data, DATA_TYPE.SFXT)
        else:
            return -1

    def get_phase_b_current(self) -> float:
        cmd = [self._address, constant.ReadPhaseBCurrent, 0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_from_data(data, DATA_TYPE.SFXT)
        else:
            return -1

    def get_bus_voltage(self) -> float:
        cmd = [self._address, constant.ReadBusVoltage, 0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_from_data(data, DATA_TYPE.SFXT)
        else:
            return -1

    def get_dc_motor_current_im(self) -> float:
        cmd = [self._address, constant.ReadDcMotorCurrentIm, 0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_from_data(data, DATA_TYPE.SFXT)
        else:
            return -1

    def get_dc_motor_voltage_vm(self) -> float:
        cmd = [self._address, constant.ReadDcMotorVoltageVm, 0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_from_data(data, DATA_TYPE.SFXT)
        else:
            return -1

    def get_speed_controller_kp(self) -> float:
        cmd = [self._address, constant.ReadSpeedControllerKp,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_from_data(data, DATA_TYPE.SFXT)
        else:
            return -1

    def get_speed_controller_ki(self) -> float:
        cmd = [self._address, constant.ReadSpeedControllerKi,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_from_data(data, DATA_TYPE.SFXT)
        else:
            return -1

    def get_output_pwm_frequency_khz(self) -> int:
        cmd = [self._address, constant.ReadOutputPwmFrequencyHz,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            # PWM reading is in Hz
            return (self.__convert_from_data(data, DATA_TYPE.UINT32)/1000)
        else:
            return -1

    def get_current_limit(self) -> float:
        cmd = [self._address, constant.ReadCurrentLimit,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_from_data(data, DATA_TYPE.SFXT)
        else:
            return -1

    def get_quadrature_current_iq_feedback(self) -> float:
        cmd = [self._address, constant.ReadQuadratureCurrentIqFeedback,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_from_data(data, DATA_TYPE.SFXT)
        else:
            return -1

    def get_magnetizing_current_id_feedback(self) -> float:
        cmd = [self._address, constant.ReadMagnetizingCurrentIdFeedback,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_from_data(data, DATA_TYPE.SFXT)
        else:
            return -1

    def get_motor_poles_counts(self) -> int:
        cmd = [self._address, constant.ReadMotorPolesCounts,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_from_data(data, DATA_TYPE.UINT32)
        else:
            return -1

    def get_incremental_encoder_lines(self) -> int:
        cmd = [self._address, constant.ReadIncrementalEncoderLines,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_from_data(data, DATA_TYPE.UINT32)
        else:
            return -1

    def get_current_controller_kp(self) -> float:
        cmd = [self._address, constant.ReadCurrentControllerKp,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_from_data(data, DATA_TYPE.SFXT) 
        else:
            return -1

    def get_current_controller_ki(self) -> float:
        cmd = [self._address, constant.ReadCurrentControllerKi,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_from_data(data, DATA_TYPE.SFXT) * 0.00005
        else:
            return -1

    def get_board_temperature(self) -> float:
        cmd = [self._address, constant.ReadBoardTemperature,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_from_data(data, DATA_TYPE.SFXT)
        else:
            return -1

    def get_motor_resistance(self) -> float:
        cmd = [self._address, constant.ReadMotorResistance,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_from_data(data, DATA_TYPE.SFXT) * 0.00005
        else:
            return -1

    def get_motor_inductance(self) -> float:
        cmd = [self._address, constant.ReadMotorInductance,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_from_data(data, DATA_TYPE.SFXT)
        else:
            return -1

    def get_speed_feedback(self) -> int:
        cmd = [self._address, constant.ReadSpeedFeedback,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_from_data(data, DATA_TYPE.INT32)
        else:
            return -1

    def get_motor_type(self) -> MOTOR_TYPE:
        cmd = [self._address, constant.ReadMotorType,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return MOTOR_TYPE(self.__convert_from_data(data, DATA_TYPE.UINT32))
        else:
            return -1

    def get_feedback_control_mode(self) -> FEEDBACK_CONTROL_MODE:
        cmd = [self._address, constant.ReadFeedbackControlMode,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return FEEDBACK_CONTROL_MODE(self.__convert_from_data(data, DATA_TYPE.UINT32))
        else:
            return -1

    def get_command_mode(self) -> COMMAND_MODE:
        cmd = [self._address, constant.ReadCommandMode,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return COMMAND_MODE(self.__convert_from_data(data, DATA_TYPE.UINT32))
        else:
            return -1

    def get_control_mode(self) -> CONTROL_MODE:
        cmd = [self._address, constant.ReadControlMode,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return CONTROL_MODE(self.__convert_from_data(data, DATA_TYPE.UINT32))
        else:
            return -1

    def get_speed_limit(self) -> int:
        cmd = [self._address, constant.ReadSpeedLimit,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_from_data(data, DATA_TYPE.UINT32)
        else:
            return -1

    def get_position_controller_kp(self) -> float:
        cmd = [self._address, constant.ReadPositionControllerKp,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_from_data(data, DATA_TYPE.SFXT)
        else:
            return -1

    def get_position_controller_ki(self) -> float:
        cmd = [self._address, constant.ReadPositionControllerKi,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_from_data(data, DATA_TYPE.SFXT)
        else:
            return -1

    def get_position_counts_feedback(self) -> int:
        cmd = [self._address, constant.ReadPositionCountsFeedback,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_from_data(data, DATA_TYPE.INT32)
        else:
            return -1

    def get_error_register(self) -> int:
        cmd = [self._address, constant.ReadErrorRegister,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_from_data(data, DATA_TYPE.UINT32)
        else:
            return -1
        
    def get_device_firmware_version(self) -> int:
        cmd = [self._address, constant.ReadDeviceFirmwareVersion,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)

            return self.__convert_from_data(data, DATA_TYPE.UINT32)
        else:
            return -1

    def get_device_hardware_version(self) -> int:
        cmd = [self._address, constant.ReadDeviceHardwareVersion,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_from_data(data, DATA_TYPE.UINT32)
        else:
            return -1

    def get_torque_reference_iq(self) -> float:
        cmd = [self._address, constant.ReadTorqueReferenceIq,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_from_data(data, DATA_TYPE.SFXT)
        else:
            return -1

    def get_speed_reference(self) -> int:
        cmd = [self._address, constant.ReadSpeedReference,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_from_data(data, DATA_TYPE.UINT32)
        else:
            return -1

    def get_magnetizing_current_id_reference(self) -> float:
        cmd = [self._address, constant.ReadMagnetizingCurrentIdReference,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_from_data(data, DATA_TYPE.SFXT)
        else:
            return -1

    def get_position_reference(self) -> int:
        cmd = [self._address, constant.ReadPositionReference,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_from_data(data, DATA_TYPE.INT32)
        else:
            return -1

    def get_power_reference(self) -> float:
        cmd = [self._address, constant.ReadPowerReference,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_from_data(data, DATA_TYPE.SFXT)
        else:
            return -1

    def get_motor_direction(self) -> DIRECTION:
        cmd = [self._address, constant.ReadMotorDirection,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return DIRECTION(self.__convert_from_data(data, DATA_TYPE.UINT32))
        else:
            return -1

    def get_observer_gain_bldc_pmsm(self) -> float:
        cmd = [self._address, constant.ReadObserverGainBldcPmsm,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_from_data(data, DATA_TYPE.SFXT)
        else:
            return -1

    def get_observer_gain_bldc_pmsm_ultrafast(self) -> float:
        cmd = [self._address, constant.ReadObserverGainBldcPmsmUltrafast,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_from_data(data, DATA_TYPE.SFXT)
        else:
            return -1

    def get_observer_gain_dc(self) -> float:
        cmd = [self._address, constant.ReadObserverGainDc,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_from_data(data, DATA_TYPE.SFXT)
        else:
            return -1

    def get_filter_gain_bldc_pmsm(self) -> float:
        cmd = [self._address, constant.ReadFilterGainBldcPmsm,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_from_data(data, DATA_TYPE.SFXT)
        else:
            return -1

    def get_filter_gain_bldc_pmsm_ultrafast(self) -> float:
        cmd = [self._address, constant.ReadFilterGainBldcPmsmUltrafast,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_from_data(data, DATA_TYPE.SFXT)
        else:
            return -1

    def get_uart_baudrate(self) -> UART_BAUD_RATE:
        cmd = [self._address, constant.ReadUartBaudRate,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return UART_BAUD_RATE(self.__convert_from_data(data, DATA_TYPE.UINT32))
        else:
            return -1

    def get_3phase_motor_angle(self) -> float:
        cmd = [self._address, constant.Read3PhaseMotorAngle,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_from_data(data, DATA_TYPE.SFXT)
        else:
            return -1

    def get_encoder_hall_ccw_offset(self) -> float:
        cmd = [self._address, constant.ReadEncoderHallCcwOffset,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_from_data(data, DATA_TYPE.SFXT)
        else:
            return -1

    def get_encoder__hall_cw_offset(self) -> float:
        cmd = [self._address, constant.ReadEncoderHallCwOffset,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_from_data(data, DATA_TYPE.SFXT)
        else:
            return -1

    def get_speed_acceleration_value(self) -> float:
        cmd = [self._address, constant.ReadSpeedAccelerationValue,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_from_data(data, DATA_TYPE.SFXT)
        else:
            return -1

    def get_speed_deceleration_value(self) -> float:
        cmd = [self._address, constant.ReadSpeedDecelerationValue,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_from_data(data, DATA_TYPE.SFXT)
        else:
            return -1

    def get_can_bus_baudrate(self) -> CAN_BUS_BAUD_RATE:
        cmd = [self._address, constant.ReadCanBusBaudRate,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return CAN_BUS_BAUD_RATE(self.__convert_from_data(data, DATA_TYPE.UINT32))
        else:
            return -1
 
    def get_encoder_index_counts (self) -> int:
        cmd = [self._address, constant.ReadEncoderIndexCounts,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_from_data(data, DATA_TYPE.UINT32)
        else:
            return -1
