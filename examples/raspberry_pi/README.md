# RASPBERRY PI INFO #
On RASPBERRY PI is possible to interface using:
- USB 
- UART
- UART (using a USB to UART converter)
- CANopen (CANopen with canable and MCP2515 shild)

## CONFIGURATIONS ##
usb:
- can be usefull to check with motion terminal the name of the port

canable:
- can be usefull to find the canable port by using the the command: ls /dev/ttyACM*


## NOTES ##
The USB/USB to UART library improvemt point: 
- power reaciling solo during code execution can lead to connectivity issue (to be validated over multiple iterations)

The canable improvment point:
- In case the port of canable is not correct, the program will generate a bloking error. In case of this error the program shuld be able to continue. 

UART protocol with RASPBERRY PI Note
=============
In order to enable UART Protocol on Raspberry Pi you need to follow this one time process. 

1. In Raspberry Pi, enter following command in Terminal to enable UART

.. code-block::

   $ sudo raspi-config

2. Select -> Interfacing Options

3. After selecting Interfacing option, select Serial option to enable UART

4. Then it will ask for the login shell to be accessible over Serial, select No shown as follows.

5. At the end, it will ask for enabling Hardware Serial port, select Yes,

6. Finally, our UART is enabled for Serial Communication on RX and TX pins of Raspberry Pi 3.

7. Then, reboot the Raspberry Pi.


CanOpen protocol with RASPBERRY PI Note
=============
In order to enable CanOpen Protocol on Raspberry Pi you need to follow this process one time

1. turn SPI on in raspberry pi: 

.. code-block::

   $ sudo raspi-config   

then go to interfaces 

then go to SPI and turn on 

then go to reboot

2. type in terminal

.. code-block::

   $ sudo apt-get update

3. then type this command:

.. code-block::

   $ sudo nano /boot/config.txt

4- scroll down and add these lines:

.. code-block::

   dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25

5- then reboot RASPBERRY PI


**Every time** you reboot RASPBERRY PI you need to follow this process

1- type this command 

.. code-block::

   $ sudo ip link set can0 up type can bitrate 1000000

**Notes** 

- The bit-rate has to be the same as the one used in the code

- The following CAN transceiver module  `"PiCAN2" <https://copperhilltech.com/pican-2-can-bus-interface-for-raspberry-pi/>`__ has been used to test the library 
