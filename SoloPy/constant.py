ReadData = 0x00  # 0x00000000
INITIATOR = 0xFF  # 0xFFFF
BroadcastAddress = 0xFF
ENDING = 0xFE
ERROR = 0xEE  # 0xEEEEEEEE
CRC = 0x00
WriteDeviceAddress = 0x01
WriteCommandMode = 0x02
WriteCurrentLimit = 0x03
WriteTorqueReferenceIq = 0x04
WriteSpeedReference = 0x05
WritePowerReference = 0x06
WriteMotorParametersIdentification = 0x07
WriteEmergencyStop = 0x08
WriteOutputPwmFrequencyKhz = 0x09
WriteSpeedControllerKp = 0x0A
WriteSpeedControllerKi = 0x0B
WriteMotorDirection = 0x0C
WriteMotorResistance = 0x0D
WriteMotorInductance = 0x0E
WriteMotorPolesCounts = 0x0F
WriteIncrementalEncoderLines = 0x10
WriteSpeedLimit = 0x11
WriteResetAddress = 0x12
WriteFeedbackControlMode = 0x13
WriteResetFactory = 0x14
WriteMotorType = 0x15
WriteControlMode = 0x16
WriteCurrentControllerKp = 0x17
WriteCurrentControllerKi = 0x18
WriteMonitoringMode = 0x19
WriteMagnetizingCurrentIdReference = 0x1A
WritePositionReference = 0x1B
WritePositionControllerKp = 0x1C
WritePositionControllerKi = 0x1D
WriteResetPositionToZero = 0x1F  # Home
WriteOverwriteErrorRegister = 0x20
# Set Sensorless Observer Gain for Normal BLDC-PMSM Motors
WriteObserverGainBldcPmsm = 0x21
# Set Sensorless Observer Gain for Ultra-Fast Brushless Motor
WriteObserverGainBldcPmsmUltrafast = 0x22
# Set Sensorless Observer Gain for DC Motor
WriteObserverGainDc = 0x23  
# Set Sensorless Observer Filter Gain for Normal Brushless Motor
WriteFilterGainBldcPmsm = 0x24
# Set Sensorless Observer Filter Gain for ultra-fast Brushless Motor
WriteFilterGainBldcPmsmUltrafast = 0x25
# Set UART line baud-rate - 937500 / 115200 [ bits/s]
WriteUartBaudRate = 0x26  
WriteSensorCalibration = 0x27
WriteEncoderHallCcwOffset = 0x28
WriteEncoderHallCwOffset = 0x29
WriteSpeedAccelerationValue = 0x2A
WriteSpeedDecelerationValue = 0x2B
WriteCanBusBaudRate = 0x2C

ReadDeviceAddress = 0x81
ReadPhaseAVoltage = 0x82
ReadPhaseBVoltage = 0x83
ReadPhaseACurrent = 0x84
ReadPhaseBCurrent = 0x85
ReadBusVoltage = 0x86
ReadDcMotorCurrentIm = 0x87
ReadDcMotorVoltageVm = 0x88
ReadSpeedControllerKp = 0x89
ReadSpeedControllerKi = 0x8A
ReadOutputPwmFrequencyHz = 0x8B
ReadCurrentLimit = 0x8C
ReadQuadratureCurrentIqFeedback = 0x8D
ReadMagnetizingCurrentIdFeedback = 0x8E  # Magnetizing 
ReadMotorPolesCounts = 0x8F
ReadIncrementalEncoderLines = 0x90
ReadCurrentControllerKp = 0x91
ReadCurrentControllerKi = 0x92
ReadBoardTemperature = 0x93
ReadMotorResistance = 0x94
ReadMotorInductance = 0x95
ReadSpeedFeedback = 0x96
ReadMotorType = 0x97
# TODO: 0x98
ReadFeedbackControlMode = 0x99
ReadCommandMode = 0x9A
ReadControlMode = 0x9B
ReadSpeedLimit = 0x9C
ReadPositionControllerKp = 0x9D
ReadPositionControllerKi = 0x9E
# TODO: 0x9F 
ReadPositionCountsFeedback = 0xA0
ReadErrorRegister = 0xA1
ReadDeviceFirmwareVersion = 0xA2
ReadDeviceHardwareVersion = 0xA3
ReadTorqueReferenceIq = 0xA4  # Read Torque /“Iq” Reference
ReadSpeedReference = 0xA5  # Read Speed Reference
ReadMagnetizingCurrentIdReference = 0xA6  # Read Magnetizing Current / “Id” Reference
ReadPositionReference = 0xA7
ReadPowerReference = 0xA8
ReadMotorDirection = 0xA9
# Read Sensorless Observer Gain for Normal BLDC-PMSM Motors
ReadObserverGainBldcPmsm = 0xAA
# Read Sensorless Observer Gain for Ultra-fast BLDC-PMSM Motors
ReadObserverGainBldcPmsmUltrafast = 0xAB
# Read Observer Gain for DC Motor
ReadObserverGainDc = 0xAC  
# Read Sensorless Observer Filter Gain for Normal BLDC-PMSM Motors
ReadFilterGainBldcPmsm  = 0xAD
# Read Sensorless Observer Filter Gain for Ultra Fast BLDC-PMSM Motors
ReadFilterGainBldcPmsmUltrafast= 0xAE
# Read Motor’s Angle
Read3PhaseMotorAngle = 0xB0  
ReadEncoderHallCcwOffset = 0xB1
ReadEncoderHallCwOffset = 0xB2
ReadUartBaudRate = 0xB3  # 0 / 1 ( 937500 / 115200 [bits/s] )
ReadSpeedAccelerationValue = 0xB4
ReadSpeedDecelerationValue = 0xB5
ReadCanBusBaudRate = 0xB6
ReadEncoderIndexCounts = 0xB8

InputOutOfRange = "input out of range, set function will not be executed"
InputNeedEnum = "the input is an enum, function will not be executed"
