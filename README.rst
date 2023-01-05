|License|

==================================================
SoloPy
==================================================

SoloPy is a Python library published by SOLO Motor Controllers to control, command
or read all the parameters that are stored or existing in the command set of SOLO devices.
More information about SoloPy on `the SOLO website <https://www.solomotorcontrollers.com/>`_.
For List of All the Available Methods Read the `DataSheet <https://www.solomotorcontrollers.com/resources/specs-datasheets/>`__

SoloPy allows SOLO devices to be used in this conditions: 

- USB Communication with Windows OS or Linux

- UART protocol with RASPBERRY PI or any similar controller

- CanOpen protocol with RASPBERRY PI or any similar controller


Installation
=============
Please make sure you have installed **Python 3** and **pip**

Install Solopy From Pip

.. code-block::

   $ pip install solopy


Update
=============
If you have installed the library and you want to update it on Linux, RASPBERRY PI or Windows

.. code-block::

   $ pip install --upgrade SoloPy 
   


USB Communication with Windows OS or Linux Note
==========================
1. Connect Solo to Raspberry Pi or Pc

2. Open terminal in linux(cmd in windows) and enter the following command

.. code-block::

   $ python -m serial.tools.list_ports -v

3. this will show you the port name that SOLO is connected to (in linux it's sth like '/dev/ttyAMC0' and in windows it's sth like 'COM8'


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

- The following CAN transciever module  `"PiCAN2" <https://copperhilltech.com/pican-2-can-bus-interface-for-raspberry-pi/>`__ has been used to test the library: 


Dependencies
=============
`Python 3 <https://www.python.org/downloads/>`__

`python-interface <https://github.com/ssanderson/python-interface>`__

for UART `pyserial <https://github.com/pyserial/pyserial>`__

for CanOpen `Python-Can <https://pypi.org/project/python-can/>`__


Authors
=============

SoloPy is created by SOLO Motor Controllers team


License
=============

GNU General Public License v3.0 or later

See `COPYING <COPYING>`_ to see the full text.

.. |License| image:: https://img.shields.io/badge/license-GPL%20v3.0-brightgreen.svg
   :target: COPYING
   :alt: Repository License