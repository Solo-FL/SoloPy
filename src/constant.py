ReadData = 0x00  # 0x00000000
INITIATOR = 0xFF  # 0xFFFF
BroadcastAddress = 0xFF
ENDING = 0xFE
ERROR = 0xEE  # 0xEEEEEEEE
CRC = 0x00
WriteAddress = 0x01
WriteCommandMode = 0x02
WriteCurrentLimit = 0x03
WriteTorqueReference = 0x04
WriteSpeedReference = 0x05
WritePowerReference = 0x06
WriteIdentification = 0x07
WriteStopSystem = 0x08
WritePWMFrequency = 0x09
WriteSpeedControllerKp = 0x0A
WriteSpeedControllerKi = 0x0B
WriteDirection = 0x0C
WriteResistance = 0x0D
WriteInductance = 0x0E
WriteNumberOfPoles = 0x0F
WriteEncoderLines = 0x10
WriteSpeedLimit = 0x11
WriteResetAddress = 0x12
WriteSpeedControlMode = 0x13
WriteResetToFactory = 0x14
WriteMotorType = 0x15
WriteControlMode = 0x16
WriteCurrentControllerKp = 0x17
WriteCurrentControllerKi = 0x18
WriteMonitoringMode = 0x19
WriteMagnetizingCurrentReference = 0x1A
WriteDesiredPosition = 0x1B
WritePositionControllerKp = 0x1C
WritePositionControllerKi = 0x1D
WriteResetPositionToZero = 0x1F  # Home
WriteOverwriteTheErrors = 0x20
# Set Sensorless Observer Gain for Normal Brushless Motor
WriteGainNormalBrushless = 0x21
# Set Sensorless Observer Gain for Ultra-Fast Brushless Motor
WriteGainUltraFastBrushless = 0x22
WriteGainDC = 0x23  # Set Sensorless Observer Gain for DC Motor
# Set Sensorless Observer Filter Gain for Normal Brushless Motor
WriteFilterGainNormalBrushless = 0x24
# Set Sensorless Observer Filter Gain for ultra-fast Brushless Motor
WriteFilterGainUltraFastBrushless = 0x25
WriteUartBaudRate = 0x26  # Set UART line baud-rate - 937500 / 115200 [ bits/s]
WriteStartENCHallCalibration =  0x27
WriteSetENCHallCCWOffset =  0x28
WriteSetENCHallCWOffset =  0x29
ReadAddress = 0x81
ReadVoltageA = 0x82
ReadVoltageB = 0x83
ReadCurrentA = 0x84
ReadCurrentB = 0x85
ReadBusVoltage = 0x86
ReadIM = 0x87
ReadVM = 0x88
ReadSpeedControllerKp = 0x89
ReadSpeedControllerKi = 0x8A
ReadPWMFrequency = 0x8B
ReadCurrentLimit = 0x8C
ReadQuadratureCurrent = 0x8D
ReadDirectCurrent = 0x8E  # Magnetizing
ReadNumberOfPoles = 0x8F
ReadEncoderLine = 0x90
ReadCurrentControllerKp = 0x91
ReadCurrentControllerKi = 0x92
ReadTemperature = 0x93
ReadResistance = 0x94
ReadInductance = 0x95
ReadSpeed = 0x96
ReadMotorType = 0x97
# TODO: 0x98 !?
ReadSpeedControlMode = 0x99
ReadCommandMode = 0x9A
ReadControlMode = 0x9B
ReadSpeedLimit = 0x9C
ReadPositionControllerKp = 0x9D
ReadPositionControllerKi = 0x9E
# TODO: 0x9F !?
ReadEncoderPosition = 0xA0
ReadErrorRegister = 0xA1
ReadFirmwareVersion = 0xA2
ReadHardwareVersion = 0xA3
ReadTorque = 0xA4  # Read Torque /“Iq” Reference
ReadSpeedReference = 0xA5  # Read Speed Reference
ReadMagnetizingCurrent = 0xA6  # Read Magnetizing Current / “Id” Reference
ReadPositionReference = 0xA7
ReadPowerReference = 0xA8
ReadDirectionRotation = 0xA9
# Read the Non-linear observer Gain for Normal Brushless motor in Sensorless mode
ReadGainNormalBrushless = 0xAA
# Read the Non-linear observer Gain for Ultra-fast Brushless motor in Sensorless mode
ReadGainUltraFastBrushless = 0xAB
ReadGainDC = 0xAC  # Read the Non-linear observer Gain for DC motor in Sensorless mode
# Read the Non-linear observer Filter Gain for Normal Brushless motor in Sensorless mode
ReadFilterGainNormalBrushless = 0xAD
# Read the Non-linear Filter Gain for Ultra-fast Brushless motor in Sensorless mode
ReadFilterGainUltraFastBrushless = 0xAE
Read3PhaseMotorAngle = 0xB0 # Read Estimated or Measured Rotor Angle
ReadENCHallCCWOffset = 0xB1
ReadENCHallCWOffset  = 0xB2
ReadUartBaudRate = 0xB3  # 0 / 1 ( 937500 / 115200 [bits/s] )
