.. |License| image:: https://img.shields.io/badge/license-MIT-blue.svg
   :target: https://opensource.org/licenses/MIT

==================================================
SoloPy - Python Library for SOLO Motor Controllers
==================================================

**SoloPy** is a Python library published by **SOLO Motor Controllers** to control, command,
or read all the parameters that are stored or exist in the command set of SOLO devices.

More information about SoloPy can be found on `the SOLO website <https://www.solomotorcontrollers.com/>`_.

For a list of all available methods, refer to the `DataSheet <https://www.solomotorcontrollers.com/resources/specs-datasheets/>`__.


Features
=============

**SoloPy** allows SOLO devices to be used under the following conditions:

- **Windows OS** using **USB** or **CANopen** (CANopen with Canable)
- **Linux OS** using **USB** or **CANopen** (CANopen with Canable)
- **Raspberry Pi** or any similar controller using **UART protocol, USB, and CANopen** (CANopen with Canable or MCP2515 Shield)


Installation
=============

Make sure you have **Python 3** and **pip** installed.

To install SoloPy from PyPI, run:

.. code-block:: bash

   pip install SoloPy

More information is available in each example's README.


Updating
=============

If you have already installed the library and want to update it on **Linux, Raspberry Pi, or Windows**, run:

.. code-block:: bash

   pip install --upgrade SoloPy 



Dependencies
=============

SoloPy requires the following dependencies:

- `Python 3 <https://www.python.org/downloads/>`__
- `python-interface <https://github.com/ssanderson/python-interface>`__
- For **UART communication**: `pyserial <https://github.com/pyserial/pyserial>`__
- For **CANopen communication**: `Python-Can <https://pypi.org/project/python-can/>`__



Authors
=============

SoloPy is created and maintained by the **SOLO Motor Controllers** team.


License
=============

SoloPy is released under the **MIT License**.

See `COPYING <COPYING>`_ for the full license text.
