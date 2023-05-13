# Copyright: (c) 2021, 2022, 2023 SOLO motor controllers project
# GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

# Title: SoloPy
# Author: SOLOMotorControllers
# Date: 2023
# Code version: 3.1.0
# Availability: https://github.com/Solo-FL/SoloPy/tree/main/SoloPy
# This Library is made by SOLOMotorControllers.COM
# please visit:  https://www.SOLOMotorControllers.com/

Object_ReadErrorRegister = 0x1001
Object_GuardTime = 0x100C
Object_LifeTimeFactor = 0x100D
Object_ProducerHeartbeatTime = 0x1017
# ***** SOLO UNO CANOPEN Objects
Object_SetDeviceAddress = 0x3001
Object_CommandMode = 0x3002
Object_CurrentLimit = 0x3003
Object_TorqueReferenceIq = 0x3004
Object_SpeedReference = 0x3005
Object_PowerReference = 0x3006
Object_MotorParametersIdentification = 0x3007
Object_EmergencyStop = 0x3008
Object_OutputPwmFrequencyKhz = 0x3009
Object_SpeedControllerKp = 0x300A
Object_SpeedControllerKi = 0x300B
Object_MotorDirection = 0x300C
Object_MotorResistance = 0x300D
Object_MotorInductance = 0x300E
Object_MotorPolesCounts = 0x300F
Object_IncrementalEncoderLines = 0x3010
Object_SpeedLimit = 0x3011
# Reserved Object                                      0x3012
Object_FeedbackControlMode = 0x3013
Object_ResetFactory = 0x3014
Object_MotorType = 0x3015
Object_ControlMode = 0x3016
Object_CurrentControllerKp = 0x3017
Object_CurrentControllerKi = 0x3018
# Reserved Object                                      0x3019
Object_MagnetizingCurrentIdReference = 0x301A
Object_PositionReference = 0x301B
Object_PositionControllerKp = 0x301C
Object_PositionControllerKi = 0x301D
# Reserved Object                                      0x301E
Object_ResetPositionToZero = 0x301F
Object_OverwriteErrorRegister = 0x3020
Object_ObserverGainBldcPmsm = 0x3021
Object_ObserverGainBldcPmsmUltrafast = 0x3022
Object_ObserverGainDc = 0x3023
Object_FilterGainBldcPmsm = 0x3024
Object_FilterGainBldcPmsmUltrafast = 0x3025
Object_UartBaudrate = 0x3026
Object_SensorCalibration = 0x3027
Object_EncoderHallCcwOffset = 0x3028
Object_EncoderHallCwOffset = 0x3029
Object_SpeedAccelerationValue = 0x302A
Object_SpeedDecelerationValue = 0x302B
Object_CanbusBaudrate = 0x302C
Object_PhaseAVoltage = 0x302D
Object_PhaseBVoltage = 0x302E
Object_PhaseACurrent = 0x302F
Object_PhaseBCurrent = 0x3030
Object_BusVoltage = 0x3031
Object_DcMotorCurrentIm = 0x3032
Object_DcMotorVoltageVm = 0x3033
Object_QuadratureCurrentIqFeedback = 0x3034
Object_MagnetizingCurrentIdFeedback = 0x3035
Object_SpeedFeedback = 0x3036
Object_PositionCountsFeedback = 0x3037
Object_3PhaseMotorAngle = 0x3038
Object_BoardTemperature = 0x3039
Object_DeviceFirmwareVersion = 0x303A
Object_DeviceHardwareVersion = 0x303B
Object_EncoderIndexCounts = 0x303D
Object_ASRDC =                                   0x303E
Object_MotionProfileMode =                       0x303F
Object_MotionProfileVariable1 =                  0x3040
Object_MotionProfileVariable2 =                  0x3041
Object_MotionProfileVariable3 =                  0x3042
Object_MotionProfileVariable4 =                  0x3043
Object_MotionProfileVariable5 =                  0x3044
