## @package Canable.py
#  @author  SOLOMotorControllers
#  @brief   This file contains all the functions for Canable communicaton 
#           Availability: https://github.com/Solo-FL/SoloPy/tree/main/SoloPy
#  @date    Date: 2025
#  @version 4.0.0

## @attention
# Copyright: (c) 2021-present, SOLO motor controllers project
# MIT License (see LICENSE file for more details)

import logging
#from interface import implements
import can
from SoloPy.CanInterface import *
from SoloPy.SOLOMotorControllers import *
from typing import Tuple

#class Canable(implements(CanInterface)):
class Canable:
    def __init__(self, channel = 'COM3', bustype = 'slcan', baudrate = CanBusBaudRate.RATE_1000, logger=None):
        self._logger = logger
        self._channel = channel
        self._bustype = bustype
        self._baudrate = baudrate.value * 1000
        self._bus = can.interface.Bus(
                        channel=self._channel, bustype=self._bustype, bitrate=self._baudrate)
        self._reader = can.BufferedReader()
        self._notifier = can.Notifier(self._bus, [self._reader])
        self._buff = []

    def __del__(self):
        try:
            self._bus.shutdown()
        except Exception as e:
            self._logger.error("Exception on Canable Del")


    def canopen_transmit(self, address: int, _object: int,  sub_index: int, informatrion_to_send: list) -> Tuple[bool, Error]:
        error = Error.GENERAL_ERROR
        result = False
        msg = can.Message(
            arbitration_id=(0x600 + address),
            data=[0x22,
                _object.to_bytes(2,byteorder='big')[1],  # LSB _object Index
                _object.to_bytes(2,byteorder='big')[0],  # MSB _object Index
                sub_index,                                   # Sub Index
                informatrion_to_send[3],
                informatrion_to_send[2],
                informatrion_to_send[1],
                informatrion_to_send[0]],
            is_extended_id=False)
        self._bus.send(msg)
        result, error, information_received = self.check_received_data(0x580 + address, 8)
        #information_received = self._bus.recv(1)
        # No unit response
        if not information_received:
            result = False
            error = Error.GENERAL_ERROR
            return result, error
        # Abort Checking
        if      ( information_received.arbitration_id == (0x580 + address)                   # Check COB-ID
                and ( information_received.data[0] == 0x80 )                                     # Check Byte1
                and ( information_received.data[1] == _object.to_bytes(2,byteorder='big')[1] )    # Check _object Index(LSB)
                and ( information_received.data[2] == _object.to_bytes(2,byteorder='big')[0] )):  # Check _object Index(MSB)
            if  (   (information_received.data[4] == 0x06)
                and (information_received.data[5] == 0x02)
                and (information_received.data[6] == 0x00)
                and (information_received.data[7] == 0x00)):
                error = Error.ABORT_OBJECT
                result = False

            elif (   (information_received.data[4] == 0x06)
                and (information_received.data[5] == 0x09)
                and (information_received.data[6] == 0x00)
                and (information_received.data[7] == 0x30)):
                error = Error.ABORT_VALUE
                result = False
        # End Abort Checking

        # Check ACK
        if      ( information_received.arbitration_id == (0x580 + address)   # Check COB-ID
            and ( information_received.data[0] == 0x60)                        # Check Byte1
            and ( information_received.data[1] == _object.to_bytes(2,byteorder='big')[1])    # Check _object Index(LSB)
            and ( information_received.data[2] == _object.to_bytes(2,byteorder='big')[0])):  # Check _object Index(MSB)
            error = Error.NO_ERROR_DETECTED
            result = True
        # End Check ACK

        return result, error

    def canopen_receive(self, address: int, _object: int, sub_index: int, informatrion_to_send: list) -> Tuple[bool, Error, list]:
        result = False
        error = Error.GENERAL_ERROR

        msg = can.Message(
            arbitration_id=(0x600 + address),
            data=[0x40,
                _object.to_bytes(2,byteorder='big')[1],  # LSB _object Index
                _object.to_bytes(2,byteorder='big')[0],  # MSB _object Index
                sub_index,                                   # Sub Index
                informatrion_to_send[3],
                informatrion_to_send[2],
                informatrion_to_send[1],
                informatrion_to_send[0]],
            is_extended_id=False)

        try:
            self._bus.send(msg)
            result, error, information_received = self.check_received_data(0x580 + address, 8)
            #information_received = self._bus.recv(1)  # Timeout in seconds.
            if not information_received:    # No unit response
                information_received = [0, 0, 0, 0, 0, 0, 0, 0]
                result = False
                error = Error.GENERAL_ERROR
                return result, error, None

            # Abort Checking
            if ( information_received
                    and information_received.arbitration_id == (0x580 + address)                   # Check COB-ID
                    and ( information_received.data[0] == 0x80 )                                     # Check Byte1  
                    and ( information_received.data[1] == _object.to_bytes(2,byteorder='big')[1] )    # Check _object Index(LSB)
                    and ( information_received.data[2] == _object.to_bytes(2,byteorder='big')[0] )):  # Check _object Index(MSB)                 
                if  (   (information_received.data[4] == 0x06)
                    and (information_received.data[5] == 0x02)
                    and (information_received.data[6] == 0x00)
                    and (information_received.data[7] == 0x00)):
                    error = Error.ABORT_OBJECT
                    result = False
                    return result, error, information_received.data

                if (   (information_received.data[4] == 0x06)
                    and (information_received.data[5] == 0x09)
                    and (information_received.data[6] == 0x00)
                    and (information_received.data[7] == 0x30)):
                    error = Error.ABORT_VALUE
                    result = False
                    return result, error, information_received.data
            # End Abort Checking

        except Exception as e:
            logging.error(e)
            information_received = [0, 0, 0, 0, 0, 0, 0, 0]
            result = False
            error = Error.GENERAL_ERROR
            return result, error, information_received

        result = True
        error = Error.NO_ERROR_DETECTED
        return result, error, information_received.data

    def pdo_transmit(self, address: int, informatrion_to_send: list) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        result = False

        try:
            msg =   [   informatrion_to_send[3],
                informatrion_to_send[2],
                informatrion_to_send[1],
                informatrion_to_send[0]
            ]

            msg = can.Message(
                arbitration_id = address,
                data = msg,
                is_extended_id=False)

            self._bus.send(msg)
        except Exception as e:
            logging.error(e)
            return False, Error.GENERAL_ERROR

        result = True
        error = Error.NO_ERROR_DETECTED
        return result, error

    def pdo_receive(self, address: int) -> Tuple[bool, int, list]:
        error = Error.NO_PROCESSED_COMMAND
        result = False
        information_received = []

        try:
            #reader = can.BufferedReader()
            #notifier = can.Notifier(self._bus, [reader])
            #information_received = reader.get_message(1)
            result, error, information_received = self.check_received_data(address, 4)
            #information_received = self._bus.recv(1)  # Timeout in seconds.
            if not information_received:    # No unit response
                result = False
                error = Error.GENERAL_ERROR
                return result, error, None

            # Abort Checking
            if (information_received
                    and information_received.arbitration_id == address  # Check COB-ID
                    and information_received.data[0] == 0x80) :
                error = Error.ABORT_OBJECT
                result = False
                return result, error, information_received.data
        except Exception as e:
            logging.error(e)
            return False, Error.GENERAL_ERROR, None

        result = True
        error = Error.NO_ERROR_DETECTED
        return result, error, information_received.data

    def send_pdo_sync(self) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        result = False

        try:
            msg = can.Message(
                    arbitration_id = 0x80,
                    data = [],
                    is_extended_id=False)
            self._bus.send(msg)
        except Exception as e:
            logging.error(e)
            return False, Error.GENERAL_ERROR

        result = True
        error = Error.NO_ERROR_DETECTED
        return result, error

    def send_pdo_rtr(self, address: int) -> Tuple[bool, Error]:
        error = Error.NO_PROCESSED_COMMAND
        result = False

        try:
            msg = can.Message(
                    arbitration_id = address,
                    data = [],
                    is_remote_frame=True,
                    is_extended_id=False)
            self._bus.send(msg)
        except Exception as e:
            logging.error(e)
            return False, Error.GENERAL_ERROR

        result = True
        error = Error.NO_ERROR_DETECTED
        return result, error

    def generic_canbus_write(self, _id: int, _data: list):
        pass

    def generic_canbus_read(self) -> list:
        pass

    def check_received_data(self, address: int, lenght: int) -> Tuple[bool, Error, list]:
        information_received = self._reader.get_message(0.01)

        while information_received:
            self._buff.append(information_received)
            information_received = self._reader.get_message(0.001)

        # Iterate over a copy of the buffer to avoid modifying it while iterating
        for message in self._buff[:]:
            if(message and message.arbitration_id == address and message.dlc == lenght):
                self._buff.remove(message)
                return True, Error.NO_ERROR_DETECTED, message

        return False, Error.GENERAL_ERROR, None

