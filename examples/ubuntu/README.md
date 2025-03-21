# LINUX BASED INFO #
On LINUX is possible to interface using:
- USB 
- UART (using a USB to UART converter)
- CANopen (CANopen with canable)

## CONFIGURATIONS ##
usb:
- can be usefull to check with motion terminal the name of the port

canable:
- can be usefull to find the canable port by using the the command: ls /dev/ttyACM*


## NOTES ##
The USB/ USB to UART library improvemt point: 
- power reaciling solo during code execution can lead to connectivity issue (to be validated over multiple iterations)

The canable improvment point:
- In case the port of canable is not correct, the program will generate a bloking error. In case of this error the program shuld be able to continue. 


USB Communication with Windows OS or Linux Note
==========================
1. Connect Solo to Raspberry Pi or Pc

2. Open terminal in linux(cmd in windows) and enter the following command

.. code-block::

   $ python -m serial.tools.list_ports -v

3. this will show you the port name that SOLO is connected to (in linux it's sth like '/dev/ttyAMC0' and in windows it's sth like 'COM8'


