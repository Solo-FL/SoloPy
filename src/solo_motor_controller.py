import serial
import constant


class SoloMotorController:

    def __init__(self, address):
        self._address = address

    def __exec_cmd(self, cmd: list) -> bool:
        _cmd = [constant.INITIATOR, constant.INITIATOR,
                cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], constant.CRC, constant.ENDING]

        _readPacket = []

        with serial.Serial('/dev/ttyS0', 9600, timeout=1) as ser:
            ser.write(_cmd)
            _readPacket = ser.read(10)        # read up to ten bytes (timeout)

            if (_readPacket[0] == _cmd[0] and _readPacket[1] == _cmd[1]
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

    def __convert_to_float(self, data) -> float:
        dec = 0
        dec = (data[0]/16)*268435456 + (data[0] % 16)*16777216 + (data[1]/16)*1048576 + (data[1] %
                                                                                         16)*65536 + (data[2]/16)*4096 + (data[2] % 16)*256 + (data[3]/16)*16 + (data[3] % 16)*1
        if(dec <= 0x7FFE0000):
            return (float)(dec/131072.0)
        else:
            dec = 0xFFFFFFFF - dec + 1
            return ((float)(dec/131072.0)) * -1

    def __convert_to_long(self, data) -> int:
        dec = 0
        dec = (data[0]/16)*268435456 + (data[0] % 16)*16777216 + (data[1]/16)*1048576 + (data[1] %
                                                                                         16)*65536 + (data[2]/16)*4096 + (data[2] % 16)*256 + (data[3]/16)*16 + (data[3] % 16)*1
        if(dec <= 2147483647):  # 0x7FFFFFFF
            return dec
        else:
            dec = 4294967295 - dec + 1  # 0xFFFFFFFF
            return dec * -1

    def __convert_to_data(self, number) -> list:
        data = []
        if(type(number) is int):
            dec = number
            if dec < 0:
                dec *= -1
                dec = 0xFFFFFFFF - dec + 1

            data[0] = dec/16777216
            dec = dec % 16777216
            data[1] = dec/65536
            dec = dec % 65536
            data[2] = dec/256
            dec = dec % 256
            data[3] = dec

        elif (type(number) is float):
            dec = (int)(number * 131072)
            if dec < 0:
                dec *= -1
                dec = 0xFFFFFFFF - dec

            data[0] = dec/16777216
            dec = dec % 16777216
            data[1] = dec/65536
            dec = dec % 65536
            data[2] = dec/256
            dec = dec % 256
            data[3] = dec

        return data

    def __get_data(self, cmd: list) -> list:
        return [cmd[2], cmd[3], cmd[4], cmd[5]]

    def set_address(self, address: str) -> bool:
        cmd = [self._address, constant.WriteAddress, 0x00, 0x00, 0x00, address]

        if(address < 0 or address > 254):
            return False

        return self.__exec_cmd(cmd)

    def set_command_mode(self, mode: bool) -> bool:
        cmd = [self._address, constant.WriteCommandMode, 0x00, 0x00, 0x00, mode]
        return self.__exec_cmd(cmd)

    def set_current_limit(self, limit_number: float) -> bool:
        if (limit_number < 0.2 or limit_number > 32):
            return False
        data = self.__convert_to_data(limit_number)
        cmd = [self._address, constant.WriteCurrentLimit,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_torque_reference(self, torque_number: float) -> bool:
        if (torque_number < 0.2 or torque_number > 32):
            return False
        data = self.__convert_to_data(torque_number)
        cmd = [self._address, constant.WriteTorqueReference,
               data[0], data[1], data[2], data[3]]

        return self.__exec_cmd(cmd)

    def set_speed_reference(self, speed_number: int) -> bool:
        if (speed_number < 0 and speed_number > 30000):
            return False
        data = self.__convert_to_data(speed_number)
        cmd = [self._address, constant.WriteSpeedReference,
               data[0], data[1], data[2], data[3]]

        return self.__exec_cmd(cmd)

    def set_power_reference(self, power_number: float) -> bool:
        if (power_number < 0 or power_number > 100):
            return False
        data = self.__convert_to_data(power_number)
        cmd = [self._address, constant.WritePowerReference,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_identification(self, identification: bool) -> bool:
        cmd = [self._address, constant.WriteIdentification,
               0x00, 0x00, 0x00, identification]
        return self.__exec_cmd(cmd)

    def stop_system(self) -> bool:
        cmd = [self._address, constant.WriteIdentification,
               0x00, 0x00, 0x00, 0x00]
        return self.__exec_cmd(cmd)

    def set_pwm_frequency(self, pwm: int) -> bool:
        if (pwm < 8 or pwm > 80):
            return False
        data = self.__convert_to_data(pwm)
        cmd = [self._address, constant.WritePWMFrequency,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_speed_controller_Kp(self, kp: float) -> bool:
        if (kp < 0 or kp > 300):
            return False
        data = self.__convert_to_data(kp)
        cmd = [self._address, constant.WriteSpeedControllerKp,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_speed_controller_Ki(self, ki: float) -> bool:
        if (ki < 0 or ki > 300):
            return False
        data = self.__convert_to_data(ki)
        cmd = [self._address, constant.WriteSpeedControllerKi,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_direction(self, dir: bool) -> bool:
        cmd = [self._address, constant.WriteDirection, 0x00, 0x00, 0x00, dir]
        return self.__exec_cmd(cmd)

    def set_resistance(self, res: float) -> bool:
        if (res < 0.001 or res > 50):
            return False
        data = self.__convert_to_data(res)
        cmd = [self._address, constant.WriteResistance,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_inductance(self, ind: float) -> bool:
        if (ind < 0.00001 or ind > 0.2):
            return False
        data = self.__convert_to_data(ind)
        cmd = [self._address, constant.WriteInductance,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_number_of_poles(self, poles: int) -> bool:
        if (poles < 1 or poles > 80):
            return False
        data = self.__convert_to_data(poles)
        cmd = [self._address, constant.WriteNumberOfPoles,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_encoder_lines(self, enc: int) -> bool:
        if (enc < 1 or enc > 40000):
            return False
        data = self.__convert_to_data(enc)
        cmd = [self._address, constant.WriteEncoderLines,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_speed_limit(self, speed: int) -> bool:
        if (speed < 1 or speed > 30000):
            return False
        data = self.__convert_to_data(speed)
        cmd = [self._address, constant.WriteSpeedLimit,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def reset_address(self) -> bool:
        cmd = [0xFF, constant.WriteResetAddress, 0x00, 0x00, 0x00, 0xFF]
        return self.__exec_cmd(cmd)

    def set_speed_control_mode(self, mode: bool) -> bool:
        cmd = [self._address, constant.WriteSpeedControlMode,
               0x00, 0x00, 0x00, mode]
        return self.__exec_cmd(cmd)

    def reset_to_factory(self) -> bool:
        cmd = [self._address, constant.WriteResetToFactory,
               0x00, 0x00, 0x00, 0x01]
        return self.__exec_cmd(cmd)

    def set_motor_type(self, type: int) -> bool:
        if (type < 0 or type > 3):
            return False
        data = self.__convert_to_data(type)
        cmd = [self._address, constant.WriteSpeedLimit,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_control_mode(self, mode: int) -> bool:
        if (mode < 0 or mode > 2):
            return False
        data = self.__convert_to_data(mode)
        cmd = [self._address, constant.WriteControlMode,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_current_controller_kp(self, kp: float) -> bool:
        if (kp < 0 or kp > 16000):
            return False
        data = self.__convert_to_data(kp)
        cmd = [self._address, constant.WriteCurrentControllerKp,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_current_controller_ki(self, ki: float) -> bool:
        if (ki < 0 or ki > 16000):
            return False
        data = self.__convert_to_data(ki)
        cmd = [self._address, constant.WriteCurrentControllerKi,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_monitoring_mode(self, mode: bool) -> bool:
        cmd = [self._address, constant.WriteMonitoringMode, 0x00, 0x00, 0x00, mode]
        return self.__exec_cmd(cmd)

    def set_magnetizing_current_reference(self, mg: float) -> bool:
        if (mg < 0 or mg > 32):
            return False
        data = self.__convert_to_data(mg)
        cmd = [self._address, constant.WriteMagnetizingCurrentReference,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_desired_position(self, pos: int) -> bool:
        if (pos < -2147483647 or pos > 2147483647):
            return False
        data = self.__convert_to_data(pos)
        cmd = [self._address, constant.WriteDesiredPosition,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_position_controller_kp(self, kp: float) -> bool:
        if (kp < 0 or kp > 16000):
            return False
        data = self.__convert_to_data(kp)
        cmd = [self._address, constant.WritePositionControllerKp,
               data[0], data[1], data[2], data[3]]
        return self.__exec_cmd(cmd)

    def set_position_controller_ki(self, ki: float) -> bool:
        if (ki < 0 or ki > 16000):
            return False
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


##############################Read##################################################

    def get_address(self, addr: int) -> int:
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
