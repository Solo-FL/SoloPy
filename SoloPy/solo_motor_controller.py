import serial
import SoloPy.constant as constant
import math
import time
import logging
from importlib import reload


class SoloMotorController:

     def __init__(self, address, baudrate=937500, port="/dev/ttyS0", timeout=3 , loggerLevel = logging.INFO):
        self._version = "SoloPy v1.1"
        
        #logger init
        logging.shutdown()
        reload(logging)
        self._logger = logging.getLogger('SoloPy')
        self._logger.setLevel(loggerLevel)
        ch = logging.StreamHandler()
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        ch.setFormatter(formatter)
        self._logger.addHandler(ch)
        #logger init end
        self._logger.debug('SoloMotorController INIT')

        self._address = address
        self._baudrate = baudrate
        self._port = port
        self._timeout = timeout
        self._ser_status = 0
        self._ser = None
        self.serial_open()
        #TODO try solving serial error
    
    def __del__(self):
        self._logger.debug('SoloMotorController DEL')
        try:
            if(self._ser_status==1):
                self._logger.debug('Serial close')
                self._ser.close()
        except Exception as e:
            self._logger.error("Exception on Serial Closure")
            #self._logger.error( e , exc_info=True)    

    def __exec_cmd(self, cmd: list) -> bool:
        if(self._ser_status==1):
            try:
                messageLog = "Serial connection is: " + str(self._ser.isOpen()) 
                self._logger.debug(messageLog)
                if(self._ser.isOpen() == False):
                    self._logger.debug("Serial open")  
                    self._ser.open()
                    self._logger.debug("Serial flush")  
                    self._ser.flush()
                    self._ser.flushInput()
                    self._ser_status = 1
                    
                _cmd = [constant.INITIATOR, constant.INITIATOR,
                        cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], constant.CRC, constant.ENDING]

                _readPacket = []   

                messageLog =  "WRITE: " + str( [hex(constant.INITIATOR), hex(constant.INITIATOR),hex(cmd[0]), hex(cmd[1]), hex(cmd[2]), hex(cmd[3]), hex(cmd[4]), hex(cmd[5]),hex(constant.CRC), hex(constant.ENDING)])
                self._logger.debug(messageLog) 
                
                self._ser.write(_cmd)

                #Time to ensure the writing, reducing it can make the comunication instable
                time.sleep(0.1)

                # read up to ten bytes (timeout)
                _readPacket = self._ser.read(10)
                messageLog = "READ: " + str( [hex(i) for i in _readPacket]) + " size: "  + str(len(_readPacket))
                self._logger.debug( messageLog) 
                if(len(_readPacket)==0):
                    self._ser.flush()

                if (_readPacket and _readPacket[0] == _cmd[0] and _readPacket[1] == _cmd[1]
                    and _readPacket[2] == _cmd[2] and _readPacket[3] == _cmd[3]
                        and _readPacket[8] == _cmd[8] and _readPacket[9] == _cmd[9]):
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

                if (cmd[2] == constant.ERROR and cmd[3] == constant.ERROR and cmd[4] == constant.ERROR and cmd[5] == constant.ERROR):
                    return False
                else:
                    return True
            except Exception as e:
                self._logger.debug('__exec_cmd Exception')
                self._logger.debug( e , exc_info=True)
                self.serial_error_handler()
                return False

        self._logger.info("Serial status not ready") 
        return False       

    def __convert_to_float(self, data) -> float:
        dec = 0
        dec = int.from_bytes(
            [data[0], data[1], data[2], data[3]], byteorder='big', signed=False)
        if(dec <= 0x7FFE0000):
            return (float)(dec/131072.0)
        else:
            dec = 0xFFFFFFFF - dec + 1
            return ((float)(dec/131072.0)) * -1

    def __convert_to_long(self, data) -> int:
        dec = 0
        dec = int.from_bytes(
            [data[0], data[1], data[2], data[3]], byteorder='big', signed=False)
        if(dec <= 0x7FFFFFFF):
            return dec
        else:
            dec = 0xFFFFFFFF - dec + 1
            return dec * -1

    def __convert_to_data(self, number) -> list:
        data = []
        if(type(number) is int):
            dec = number
            if dec < 0:
                dec *= -1
                dec = 0xFFFFFFFF - dec + 1
            data = [(dec >> i & 0xff) for i in (24, 16, 8, 0)]
        elif (type(number) is float):
            dec = math.ceil(number * 131072)
            if dec < 0:
                dec *= -1
                dec = 0xFFFFFFFF - dec
            data = [(dec >> i & 0xff) for i in (24, 16, 8, 0)]
        return data

    def __get_data(self, cmd: list) -> list:
        return [cmd[2], cmd[3], cmd[4], cmd[5]]

    def set_address(self, address: int) -> bool:
        cmd = [self._address, constant.WriteAddress, 0x00, 0x00, 0x00, address]
        if(address < 0 or address > 254):
            return False
        return self.__exec_cmd(cmd)

    ##############################Support################################################## """
    def version(self) -> string:
        return self._version

    def serial_open(self) -> bool:
        self._logger.debug("serial_open start")
        try:  
            self._ser = serial.Serial(self._port, self._baudrate, timeout=self._timeout, writeTimeout =self._timeout)
            self._ser_status = 1

            #Time sleep for ensure serial initialization
            time.sleep(0.2)
            
        except Exception as e:
            self._logger.error("serial_open: Exception during the serial inizialisation")
            #self._logger.error( e , exc_info=True)
            self._ser_status= -1
            #raise e
            return False

        if(self._ser_status==1):
            self._logger.debug("Serial init")  
            self._ser.bytesize=serial.EIGHTBITS
            self._ser.parity=serial.PARITY_NONE
            self._ser.stopbits=serial.STOPBITS_ONE  
            time.sleep(0.2)
            if(self._ser.isOpen() == False):
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
                    if(self._ser.inWaiting()>0):
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
            #self._logger.error( e , exc_info=True)
            self._ser_status = -1
            self._logger.debug('SEH end')
            return False 

    def serial_close(self) -> bool:
        self._logger.debug("serial_close start")

        if (self._ser_status==1):
            try:
                self._ser.close()
                self._logger.info("serial_close: Serial closed")
                self._ser_status = 0
                return True
            except Exception as e:
                self._logger.error("serial_close: Exception on Serial Closure")
                #self._logger.error( e , exc_info=True)
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
        if(self._ser_status==1):
            status += 'SPS: Serial Open'
            try:
                status += '\nSPS: Serial comunication open: ' + str(self._ser.isOpen())
                if(self._ser.isOpen()):
                    status += '\nSPS: Serial input buffer size: '+ str(self._ser.out_waiting)
                    status += '\nSPS: Serial output buffer size: '+ str(self._ser.inWaiting())
            except Exception as e:
                self._logger.error('exception on printing the status')
                #self._logger.error( e , exc_info=True)

        if(self._ser_status== 0):
            status += 'SPS: Serial Close'
        
        if(self._ser_status== -1):
            status += 'SPS: Serial on Error'
        return status
        
    def serial_is_working(self) -> bool:
        if(self.get_voltage_a()==-1):
            return False
        return True

    ##############################Write##################################################

    def set_command_mode(self, mode: int) -> bool:
        if (mode < 0 or mode > 1):
            self._logger.info(constant.InputOutOfRange)
            return False
        mode = int(mode)
        data = self.__convert_to_data(mode)
        cmd = [self._address, constant.WriteCommandMode,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_current_limit(self, limit_number: float) -> bool:
        if (limit_number < 0.2 or limit_number > 32):
            return False
        limit_number = float(limit_number)
        data = self.__convert_to_data(limit_number)

        cmd = [self._address, constant.WriteCurrentLimit,
               data[0], data[1], data[2], data[3]]

        return self.__exec_cmd(cmd)

    def set_torque_reference(self, torque_number: float) -> bool:
        if (torque_number < 0 or torque_number > 32):
            self._logger.info(constant.InputOutOfRange) 
            return False
        torque_number = float(torque_number)
        data = self.__convert_to_data(torque_number)
        cmd = [self._address, constant.WriteTorqueReference,
               data[0], data[1], data[2], data[3]]

        return self.__exec_cmd(cmd)

    def set_speed_reference(self, speed_number: int) -> bool:
        if (speed_number < 0 or speed_number > 30000):
            self._logger.info(constant.InputOutOfRange)
            return False
        speed_number = int(speed_number)
        data = self.__convert_to_data(speed_number)
        cmd = [self._address, constant.WriteSpeedReference,
               data[0], data[1], data[2], data[3]]

        return self.__exec_cmd(cmd)

    def set_power_reference(self, power_number: float) -> bool:
        if (power_number < 0 or power_number > 100):
            self._logger.info(constant.InputOutOfRange)
            return False
        power_number = float(power_number)
        data = self.__convert_to_data(power_number)
        cmd = [self._address, constant.WritePowerReference,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_identification(self, identification: int) -> bool:
        if (identification < 0 or identification > 1):
            self._logger.info(constant.InputOutOfRange)
            return False
        identification = int(identification)
        data = self.__convert_to_data(identification)
        cmd = [self._address, constant.WriteIdentification,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def stop_system(self) -> bool:
        cmd = [self._address, constant.WriteIdentification,
               0x00, 0x00, 0x00, 0x00]
        return self.__exec_cmd(cmd)

    def set_pwm_frequency(self, pwm: int) -> bool:
        if (pwm < 8 or pwm > 80):
            self._logger.info(constant.InputOutOfRange) 
            return False
        pwm = int(pwm)
        data = self.__convert_to_data(pwm)
        cmd = [self._address, constant.WritePWMFrequency,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_speed_controller_Kp(self, kp: float) -> bool:
        if (kp < 0 or kp > 300):
            self._logger.info(constant.InputOutOfRange) 
            return False
        kp = float(kp)
        data = self.__convert_to_data(kp)
        cmd = [self._address, constant.WriteSpeedControllerKp,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_speed_controller_Ki(self, ki: float) -> bool:
        if (ki < 0 or ki > 300):
            self._logger.info(constant.InputOutOfRange) 
            return False
        ki = float(ki)
        data = self.__convert_to_data(ki)
        cmd = [self._address, constant.WriteSpeedControllerKi,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_direction(self, dir: int) -> bool:
        if (dir < 0 or dir > 1):
            self._logger.info(constant.InputOutOfRange)
            return False
        dir = int(dir)
        data = self.__convert_to_data(dir)
        cmd = [self._address, constant.WriteDirection,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_resistance(self, res: float) -> bool:
        if (res < 0.001 or res > 50):
            self._logger.info(constant.InputOutOfRange) 
            return False
        res = float(res)
        data = self.__convert_to_data(res)
        cmd = [self._address, constant.WriteResistance,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_inductance(self, ind: float) -> bool:
        if (ind < 0.00001 or ind > 0.2):
            self._logger.info(constant.InputOutOfRange) 
            return False
        ind = float(ind)
        data = self.__convert_to_data(ind)
        cmd = [self._address, constant.WriteInductance,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_number_of_poles(self, poles: int) -> bool:
        if (poles < 1 or poles > 80):
            self._logger.info(constant.InputOutOfRange) 
            return False
        poles = int(poles)
        data = self.__convert_to_data(poles)
        cmd = [self._address, constant.WriteNumberOfPoles,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_encoder_lines(self, enc: int) -> bool:
        if (enc < 1 or enc > 40000):
            self._logger.info(constant.InputOutOfRange) 
            return False
        enc = int(enc)
        data = self.__convert_to_data(enc)
        cmd = [self._address, constant.WriteEncoderLines,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_speed_limit(self, speed: int) -> bool:
        if (speed < 1 or speed > 30000):
            self._logger.info(constant.InputOutOfRange) 
            return False
        speed = int(speed)
        data = self.__convert_to_data(speed)
        cmd = [self._address, constant.WriteSpeedLimit,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def reset_address(self) -> bool:
        cmd = [0xFF, constant.WriteResetAddress, 0x00, 0x00, 0x00, 0xFF]
        return self.__exec_cmd(cmd)

    def set_speed_control_mode(self, mode: int) -> bool:
        if (mode < 0 or mode > 2):
            self._logger.info(constant.InputOutOfRange) 
            return False
        mode = int(mode)
        cmd = [self._address, constant.WriteSpeedControlMode,
               0x00, 0x00, 0x00, mode]
        return self.__exec_cmd(cmd)

    def reset_to_factory(self) -> bool:
        cmd = [self._address, constant.WriteResetToFactory,
               0x00, 0x00, 0x00, 0x01]
        return self.__exec_cmd(cmd)

    def set_motor_type(self, type: int) -> bool:
        if (type < 0 or type > 3):
            self._logger.info(constant.InputOutOfRange) 
            return False
        type = int(type)
        data = self.__convert_to_data(type)
        cmd = [self._address, constant.WriteMotorType,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_control_mode(self, mode: int) -> bool:
        if (mode < 0 or mode > 2):
            self._logger.info(constant.InputOutOfRange) 
            return False
        mode = int(mode)
        data = self.__convert_to_data(mode)
        cmd = [self._address, constant.WriteControlMode,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_current_controller_kp(self, kp: float) -> bool:
        if (kp < 0 or kp > 16000):
            self._logger.info(constant.InputOutOfRange) 
            return False
        kp = float(kp)
        data = self.__convert_to_data(kp)
        cmd = [self._address, constant.WriteCurrentControllerKp,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_current_controller_ki(self, ki: float) -> bool:
        if (ki < 0 or ki > 16000):
            self._logger.info(constant.InputOutOfRange) 
            return False
        ki = float(ki)
        data = self.__convert_to_data(ki)
        cmd = [self._address, constant.WriteCurrentControllerKi,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_monitoring_mode(self, mode: int) -> bool:
        if (mode < 0 or mode > 2):
            self._logger.info(constant.InputOutOfRange) 
            return False
        mode = int(mode)
        cmd = [self._address, constant.WriteMonitoringMode, 0x00, 0x00, 0x00, mode]
        return self.__exec_cmd(cmd)

    def set_magnetizing_current_reference(self, mg: float) -> bool:
        if (mg < 0 or mg > 32):
            self._logger.info(constant.InputOutOfRange) 
            return False
        mg = float(mg)
        data = self.__convert_to_data(mg)
        cmd = [self._address, constant.WriteMagnetizingCurrentReference,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_desired_position(self, pos: int) -> bool:
        if (pos < -2147483647 or pos > 2147483647):
            self._logger.info(constant.InputOutOfRange) 
            return False
        pos = int(pos)
        data = self.__convert_to_data(pos)
        cmd = [self._address, constant.WriteDesiredPosition,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_position_controller_kp(self, kp: float) -> bool:
        if (kp < 0 or kp > 16000):
            self._logger.info(constant.InputOutOfRange) 
            return False
        kp = float(kp)
        data = self.__convert_to_data(kp)
        cmd = [self._address, constant.WritePositionControllerKp,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_position_controller_ki(self, ki: float) -> bool:
        if (ki < 0 or ki > 16000):
            self._logger.info(constant.InputOutOfRange) 
            return False
        ki = float(ki)
        data = self.__convert_to_data(ki)
        cmd = [self._address, constant.WritePositionControllerKi,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def reset_position_to_zero(self) -> bool:
        cmd = [self._address, constant.WriteResetPositionToZero,
               0x00, 0x00, 0x00, 0x01]
        return self.__exec_cmd(cmd)

    def overwrite_errors(self) -> bool:
        cmd = [self._address, constant.WriteOverwriteTheErrors,
               0x00, 0x00, 0x00, 0x00]
        return self.__exec_cmd(cmd)

    # SOG => Sensorless Observer Gain
    def set_sog_normalBrushless_motor(self, G: float) -> bool:
        if (G < 0.01 or G > 1000):
            self._logger.info(constant.InputOutOfRange) 
            return False
        G = float(G)
        data = self.__convert_to_data(G)
        cmd = [self._address, constant.WriteGainNormalBrushless,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_sog_ultrafastBrushless_motor(self, G: float) -> bool:
        if (G < 0.01 or G > 1000):
            self._logger.info(constant.InputOutOfRange) 
            return False
        G = float(G)
        data = self.__convert_to_data(G)
        cmd = [self._address, constant.WriteGainUltraFastBrushless,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_sog_dc_motor(self, G: float) -> bool:
        if (G < 0.01 or G > 1000):
            self._logger.info(constant.InputOutOfRange) 
            return False
        G = float(G)
        data = self.__convert_to_data(G)
        cmd = [self._address, constant.WriteGainDC,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    # SOFG = > Sensorless Observer Filter Gain
    def set_sofg_normalBrushless_motor(self, G: float) -> bool:
        if (G < 0.01 or G > 16000):
            self._logger.info(constant.InputOutOfRange) 
            return False
        G = float(G)
        data = self.__convert_to_data(G)
        cmd = [self._address, constant.WriteFilterGainNormalBrushless,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_sofg_ultrafastBrushless_motor(self, G: float) -> bool:
        if (G < 0.01 or G > 16000):
            self._logger.info(constant.InputOutOfRange) 
            return False
        G = float(G)
        data = self.__convert_to_data(G)
        cmd = [self._address, constant.WriteFilterGainUltraFastBrushless,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_uart_baudrate(self, baudrate: int):
        if (baudrate != 0 or baudrate != 1):
            self._logger.info(constant.InputOutOfRange) 
            return False
        baudrate = int(baudrate)
        data = self.__convert_to_data(baudrate)
        cmd = [self._address, constant.WriteUartBaudRate,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def start_enc_hallCalibration(self, cal: int) -> bool:
        if (cal < 0 or cal > 2):
            self._logger.info(constant.InputOutOfRange) 
            return False
        cal = int(cal)
        data = self.__convert_to_data(cal)
        cmd = [self._address, constant.WriteStartENCHallCalibration,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_enc_hallCCW_offset(self, offset: float) -> bool:
        if (offset <= 0 or offset >= 1):
            self._logger.info(constant.InputOutOfRange) 
            return False
        offset = float(offset)
        data = self.__convert_to_data(offset)
        cmd = [self._address, constant.WriteSetENCHallCCWOffset,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_enc_HallCW_offset(self, offset: float) -> bool:
        if (offset <= 0 or offset >= 1):
            self._logger.info(constant.InputOutOfRange) 
            return False
        offset = float(offset)
        data = self.__convert_to_data(offset)
        cmd = [self._address, constant.WriteSetENCHallCWOffset,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_speed_acceleration_value(self, accelerationValue: float) -> bool:
        if (accelerationValue < 0 or accelerationValue > 1600):
            self._logger.info(constant.InputOutOfRange) 
            return False
        accelerationValue = float(accelerationValue)
        data = self.__convert_to_data(accelerationValue)
        cmd = [self._address, constant.WriteSpeedAccelerationValue,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_speed_deceleration_value(self, decelerationValue: float) -> bool:
        if (decelerationValue < 0 or decelerationValue > 1600):
            self._logger.info(constant.InputOutOfRange) 
            return False
        decelerationValue = float(decelerationValue)
        data = self.__convert_to_data(decelerationValue)
        cmd = [self._address, constant.WriteSpeedDecelerationValue,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_can_bus_baudrate(self, baudrate: int) -> bool:
        if (baudrate < 0 or baudrate > 4):
            self._logger.info(constant.InputOutOfRange) 
            return False
        baudrate = int(baudrate)
        data = self.__convert_to_data(baudrate)
        cmd = [self._address, constant.WriteCanBusBaudRate,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    ##############################Read##################################################

    def get_address(self) -> int:
        cmd = [self._address, constant.ReadAddress, 0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_long(data)
        else:
            return -1

    def get_voltage_a(self) -> float:
        cmd = [self._address, constant.ReadVoltageA, 0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_float(data)
        else:
            return -1

    def get_voltage_b(self) -> float:
        cmd = [self._address, constant.ReadVoltageB, 0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_float(data)
        else:
            return -1

    def get_current_a(self) -> float:
        cmd = [self._address, constant.ReadCurrentA, 0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_float(data)
        else:
            return -1

    def get_current_b(self) -> float:
        cmd = [self._address, constant.ReadCurrentB, 0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_float(data)
        else:
            return -1

    def get_bus_voltage(self) -> float:
        cmd = [self._address, constant.ReadBusVoltage, 0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_float(data)
        else:
            return -1

    def get_motor_current(self) -> float:
        cmd = [self._address, constant.ReadIM, 0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_float(data)
        else:
            return -1

    def get_motor_voltage(self) -> float:
        cmd = [self._address, constant.ReadVM, 0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_float(data)
        else:
            return -1

    def get_speed_controller_kp(self) -> float:
        cmd = [self._address, constant.ReadSpeedControllerKp,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_float(data)
        else:
            return -1

    def get_speed_controller_ki(self) -> float:
        cmd = [self._address, constant.ReadSpeedControllerKi,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_float(data)
        else:
            return -1

    def get_Pwm_frequency(self) -> int:
        cmd = [self._address, constant.ReadPWMFrequency,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            # PWM reading is in Hz
            return (self.__convert_to_long(data)/1000)
        else:
            return -1

    def get_current_limit(self) -> float:
        cmd = [self._address, constant.ReadCurrentLimit,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_float(data)
        else:
            return -1

    def get_quadrature_current(self) -> float:
        cmd = [self._address, constant.ReadQuadratureCurrent,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_float(data)
        else:
            return -1

    def get_direct_current(self) -> float:
        cmd = [self._address, constant.ReadDirectCurrent,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_float(data)
        else:
            return -1

    def get_number_of_poles(self) -> int:
        cmd = [self._address, constant.ReadNumberOfPoles,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_long(data)
        else:
            return -1

    def get_encoder_lines(self) -> int:
        cmd = [self._address, constant.ReadEncoderLine,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_long(data)
        else:
            return -1

    def get_current_controller_kp(self) -> float:
        cmd = [self._address, constant.ReadCurrentControllerKp,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_float(data)
        else:
            return -1

    def get_current_controller_ki(self) -> float:
        cmd = [self._address, constant.ReadCurrentControllerKi,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_float(data)
        else:
            return -1

    def get_temperature(self) -> float:
        cmd = [self._address, constant.ReadTemperature,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_float(data)
        else:
            return -1

    def get_resistance(self) -> float:
        cmd = [self._address, constant.ReadResistance,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_float(data)
        else:
            return -1

    def get_inductance(self) -> float:
        cmd = [self._address, constant.ReadInductance,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_float(data)
        else:
            return -1

    def get_speed(self) -> int:
        cmd = [self._address, constant.ReadSpeed,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_long(data)
        else:
            return -1

    def get_motor_type(self) -> int:
        cmd = [self._address, constant.ReadMotorType,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_long(data)
        else:
            return -1

    def get_speed_control_mode(self) -> int:
        cmd = [self._address, constant.ReadSpeedControlMode,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_long(data)
        else:
            return -1

    def get_command_mode(self) -> int:
        cmd = [self._address, constant.ReadCommandMode,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_long(data)
        else:
            return -1

    def get_control_mode(self) -> int:
        cmd = [self._address, constant.ReadControlMode,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_long(data)
        else:
            return -1

    def get_speed_limit(self) -> int:
        cmd = [self._address, constant.ReadSpeedLimit,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_long(data)
        else:
            return -1

    def get_position_controller_kp(self) -> float:
        cmd = [self._address, constant.ReadPositionControllerKp,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_float(data)
        else:
            return -1

    def get_position_controller_ki(self) -> float:
        cmd = [self._address, constant.ReadPositionControllerKi,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_float(data)
        else:
            return -1

    def get_encoder_position(self) -> int:
        cmd = [self._address, constant.ReadEncoderPosition,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_long(data)
        else:
            return -1

    def get_error_register(self) -> int:
        cmd = [self._address, constant.ReadErrorRegister,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_long(data)
        else:
            return -1
        
    def get_firmware_version(self) -> int:
        cmd = [self._address, constant.ReadFirmwareVersion,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)

            return self.__convert_to_long(data)
        else:
            return -1

    def get_hardware_version(self) -> int:
        cmd = [self._address, constant.ReadHardwareVersion,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_long(data)
        else:
            return -1

    def get_torque_reference(self) -> float:
        cmd = [self._address, constant.ReadTorque,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_float(data)
        else:
            return -1

    def get_speed_reference(self) -> int:
        cmd = [self._address, constant.ReadSpeedReference,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_long(data)
        else:
            return -1

    def get_magnetizing_current(self) -> float:
        cmd = [self._address, constant.ReadMagnetizingCurrent,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_float(data)
        else:
            return -1

    def get_position_reference(self) -> int:
        cmd = [self._address, constant.ReadPositionReference,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_long(data)
        else:
            return -1

    def get_power_reference(self) -> float:
        cmd = [self._address, constant.ReadPowerReference,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_float(data)
        else:
            return -1

    def get_direction_rotation(self) -> int:
        cmd = [self._address, constant.ReadDirectionRotation,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_long(data)
        else:
            return -1

    def get_sog_normalBrushless_motor(self) -> float:
        cmd = [self._address, constant.ReadGainNormalBrushless,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_float(data)
        else:
            return -1

    def get_sog_ultraFastBrushless_motor(self) -> float:
        cmd = [self._address, constant.ReadGainUltraFastBrushless,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_float(data)
        else:
            return -1

    def get_sog_dc_motor(self) -> float:
        cmd = [self._address, constant.ReadGainDC,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_float(data)
        else:
            return -1

    def get_sofg_normalBrushless_motor(self) -> float:
        cmd = [self._address, constant.ReadFilterGainNormalBrushless,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_float(data)
        else:
            return -1

    def get_sofg_ultraFastBrushless_motor(self) -> float:
        cmd = [self._address, constant.ReadFilterGainUltraFastBrushless,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_float(data)
        else:
            return -1

    def get_uart_baudrate(self) -> int:
        cmd = [self._address, constant.ReadUartBaudRate,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_long(data)
        else:
            return -1

    def get_3Phase_motorAngle(self) -> float:
        cmd = [self._address, constant.Read3PhaseMotorAngle,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_float(data)
        else:
            return -1 

    def get_enc_HallCCW_offset(self) -> float:
        cmd = [self._address, constant.ReadENCHallCCWOffset,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_float(data)
        else:
            return -1

    def get_enc_hallCW_offset(self) -> float:
        cmd = [self._address, constant.ReadENCHallCWOffset,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_float(data)
        else:
            return -1

    def get_speed_acceleration_value(self) -> float:
        cmd = [self._address, constant.ReadSpeedAccelerationValue,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_float(data)
        else:
            return -1

    def get_speed_deceleration_value(self) -> float:
        cmd = [self._address, constant.ReadSpeedDecelerationValue,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_float(data)
        else:
            return -1

    def get_can_bus_baudrate(self) -> int:
        cmd = [self._address, constant.ReadCanBusBaudRate,
               0x00, 0x00, 0x00, 0x00]
        if(self.__exec_cmd(cmd)):
            data = self.__get_data(cmd)
            return self.__convert_to_long(data)
        else:
            return -1
