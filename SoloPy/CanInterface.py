## @package CanInterface.py
#  @author  SOLOMotorControllers
#  @brief   This file contains all the functions prototypes for the CAN communication
#           Availability: https://github.com/Solo-FL/SoloPy/tree/main/SoloPy
#  @date    Date: 2025
#  @version 4.0.0

## @attention
# Copyright: (c) 2021-present, SOLO motor controllers project
# MIT License (see LICENSE file for more details)

#from interface import Interface
from SoloPy.SOLOMotorControllers import Error
from typing import Tuple

#class CanInterface(Interface):
class CanInterface:
    def canopen_transmit(self, address: int, _object: int,  sub_index: int, informatrion_to_send: list) -> Tuple[bool, Error]:
        pass

    def canopen_receive(self, address: int, _object: int, sub_index: int, informatrion_to_send: list) -> Tuple[bool, Error, list]:
        pass

    def pdo_transmit(self, address: int, informatrion_to_send: list) -> Tuple[bool, Error]:
        pass

    def pdo_receive(self, address: int) -> Tuple[bool, int, list]:
        pass

    def generic_canbus_write(self, _id: int, _data: list):
        pass

    def generic_canbus_read(self) -> list:
        pass
