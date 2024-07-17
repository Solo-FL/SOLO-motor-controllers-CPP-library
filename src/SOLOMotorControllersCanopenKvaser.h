/**
 *******************************************************************************
 * @file    SOLOMotorControllersCanopenKvaser.h
 * @authors SOLO Motor Controllers
 * @brief   This file contains all the base functions prototypes for the Solo Drivers
 *          Availability: https://github.com/Solo-FL/SOLO-motor-controllers-CPP-library
 * 
 * @date    Date: 2024
 * @version 1.3.0
 * *******************************************************************************    
 * @attention
 * Copyright: (c) 2021-2024, SOLO motor controllers project
 * MIT License (see LICENSE file for more details)
 ******************************************************************************* 
 */

#pragma once

#include <stdint.h>
#include "SOLOMotorControllers.h"
#include "SOLOMotorControllersUtils.h"

#include "canlib.h"
#include "Kvaser.h"
#include <map>

#ifndef SOLOMOTORCONTROLLERSCANOPEN_H       //Avoid loading SOLOMotorControllersCanopen.h more than once
#define SOLOMOTORCONTROLLERSCANOPEN_H

//SOLO Object Index
/** @addtogroup NMT_Control_Objects NMT Control Objects
  * @{
  */ 
#define Object_ReadErrorRegister                       0x1001
#define Object_GuardTime                               0x100C
#define Object_LifeTimeFactor                          0x100D
#define Object_ProducerHeartbeatTime                   0x1017
/**
  * @}
  */

/** @addtogroup SOLO_UNO_CANOPEN_Objects SOLO UNO CANOPEN Objects
  * @{
  */
#define Object_SetDeviceAddress                        0x3001
#define Object_CommandMode                             0x3002
#define Object_CurrentLimit                            0x3003
#define Object_TorqueReferenceIq                       0x3004
#define Object_SpeedReference                          0x3005
#define Object_PowerReference                          0x3006
#define Object_MotorParametersIdentification           0x3007
#define Object_DriveDisableEnable	                   0x3008
#define Object_OutputPwmFrequency                      0x3009
#define Object_SpeedControllerKp                       0x300A
#define Object_SpeedControllerKi                       0x300B
#define Object_MotorDirection                          0x300C
#define Object_MotorResistance                         0x300D
#define Object_MotorInductance                         0x300E
#define Object_MotorPolesCounts                        0x300F
#define Object_IncrementalEncoderLines                 0x3010
#define Object_SpeedLimit                              0x3011
//Reserved Object                                      0x3012
#define Object_FeedbackControlMode                     0x3013
#define Object_ResetFactory                            0x3014
#define Object_MotorType                               0x3015
#define Object_ControlMode                             0x3016
#define Object_CurrentControllerKp                     0x3017
#define Object_CurrentControllerKi                     0x3018
//Reserved Object                                      0x3019
#define Object_MagnetizingCurrentIdReference           0x301A
#define Object_PositionReference                       0x301B
#define Object_PositionControllerKp                    0x301C
#define Object_PositionControllerKi                    0x301D
//Reserved Object                                      0x301E
#define Object_ResetPositionToZero                     0x301F
#define Object_OverwriteErrorRegister                  0x3020
#define Object_ZsftInjectionAmplitude	               0x3021
#define Object_ZsftPolarityAmplitude		           0x3022
#define Object_ObserverGainDc                          0x3023
#define Object_ZsftInjectionFrequency	               0x3024
#define Object_SensorlessTransactionSpeed	           0x3025
#define Object_UartBaudrate                            0x3026
#define Object_SensorCalibration                       0x3027
#define Object_EncoderHallCcwOffset                    0x3028
#define Object_EncoderHallCwOffset                     0x3029
#define Object_SpeedAccelerationValue                  0x302A
#define Object_SpeedDecelerationValue                  0x302B
#define Object_CanbusBaudrate                          0x302C
#define Object_PhaseAVoltage                           0x302D
#define Object_PhaseBVoltage                           0x302E
#define Object_PhaseACurrent                           0x302F
#define Object_PhaseBCurrent                           0x3030
#define Object_BusVoltage                              0x3031
#define Object_DcMotorCurrentIm                        0x3032
#define Object_DcMotorVoltageVm                        0x3033
#define Object_QuadratureCurrentIqFeedback             0x3034
#define Object_MagnetizingCurrentIdFeedback            0x3035
#define Object_SpeedFeedback                           0x3036  
#define Object_PositionCountsFeedback                  0x3037
#define Object_3PhaseMotorAngle                        0x3038
#define Object_BoardTemperature                        0x3039
#define Object_DeviceFirmwareVersion                   0x303A
#define Object_DeviceHardwareVersion                   0x303B
#define Object_EncoderIndexCounts                      0x303D
#define Object_ASRDC                                   0x303E
#define Object_MotionProfileMode                       0x303F
#define Object_MotionProfileVariable1                  0x3040
#define Object_MotionProfileVariable2                  0x3041
#define Object_MotionProfileVariable3                  0x3042
#define Object_MotionProfileVariable4                  0x3043
#define Object_MotionProfileVariable5                  0x3044
#define Object_PT1000SensorVoltage		               0x3047
#define Object_DigitalOutputRegister                  0x3048
#define Object_DigitalInputRegister		         	   0x3049
#define Object_AnalogueInput			         	   0x304A
#define Object_RegenerationCurrentLimit                0x304B
#define Object_PositionSensorDigitalFilterLevel	       0x304C

/**
  * @}
  */

#define PdoParameterNameCount							12

#define RPDO_MIN_COBIB									0x200
#define RPDO_MAX_COBIB									0x27F
#define TPDO_MIN_COBIB									0x280
#define TPDO_MAX_COBIB									0x2FF

/**
 * @brief  Pdo Parameter Name enumeration
 */
enum PdoParameterName
{
	positionReference 				= 0,			/*!< target position [RPDO] */
	speedReference 					= 1,			/*!< target velocity [RPDO] */
	torqueReferenceIq 				= 2,			/*!< target torque [RPDO] */
	magnetizingCurrentIdReference 	= 3,			/*!< target direct current [RPDO] */
	controlMode 					= 4,			/*!< control mode [RPDO] */
	motorDirection 					= 5,			/*!< motor direction [RPDO] */
	positionCountsFeedback			= 6,			/*!< feedback position [TPDO] */
	speedFeedback 					= 7,			/*!< feedback velocity [TPDO] */
	quadratureCurrentIqFeedback		= 8,			/*!< feedback lq [TPDO] */
	magnetizingCurrentIdFeedback 	= 9,			/*!< feedback ld [TPDO] */
	errorRegister 					= 10,			/*!< error register [TPDO] */
	boardTemperature 				= 11			/*!< board temperature [TPDO] */
};

/**
 * @brief a struct that include all the Parameter used during PDO configuration
 */
typedef struct 
{
	PdoParameterName 		parameterName;
	int						parameterCobId;
	bool					isPdoParameterEnable;	
	bool					isRrtParameterEnable; 	
	int 					syncParameterCount;								
} PdoParameterConfig;

/**
 * @brief a class for handle canopen communication
 */
class SOLOMotorControllersCanopenKvaser : public SOLOMotorControllers
{

private:
	uint8_t Address;
	canHandle hnd;		
	Kvaser *kvaser;
	SOLOMotorControllersUtils *soloUtils;
	long timeout;
	long canBaudrate;
	int pdoParameterObjectByPdoParameterName[PdoParameterNameCount];

public:
	int pdoParameterCobIdByPdoParameterName[PdoParameterNameCount];
	SOLOMotorControllersCanopenKvaser(UINT8 deviceAddress = 0, 
			SOLOMotorControllers::CanbusBaudrate baudrate = SOLOMotorControllers::CanbusBaudrate::rate1000,
	 		long millisecondsTimeout = 200, bool autoConnect = true);

	~SOLOMotorControllersCanopenKvaser();
	static int lastError;

public:
	bool Connect(UINT8 deviceAddress, 
		SOLOMotorControllers::CanbusBaudrate baudrate = SOLOMotorControllers::CanbusBaudrate::rate1000,
		long millisecondsTimeout = 200);

	bool Connect();

	void Disconnect();
	
/** @addtogroup CANOpen_Write_Functions_Specific Specific CANOpen Write Functions
  * @{
  */
	bool SetGuardTime(long guardtime, int &error = lastError);
	bool SetLifeTimeFactor(long lifeTimeFactor, int &error = lastError);
	bool SetProducerHeartbeatTime(long producerHeartbeatTime, int &error = lastError);
	bool SetPdoParameterConfig(PdoParameterConfig config, int &error = lastError);
/**
  * @}
  */

/** @addtogroup CANOpen_PDO_Functions CANOpen Functions for Work with PDO Objects
  * @{
  */
	bool 	SetPdoParameterCobbIdInputValidation(PdoParameterName parameterName, int parameterCobbId, int &error);
	bool 	SetSyncParameterCountInputValidation(uint8_t parameterCount, int &error);
	long 	GetPdoParameterValueLong(PdoParameterName parameterName, int &error);
	float 	GetPdoParameterValueFloat(PdoParameterName parameterName, int &error);
	bool 	PdoRtrValidParameter(PdoParameterName parameterName, int &error);
	void 	InitPdoConfig();
	long 	GetPdoParameterCobId(PdoParameterName parameterName, int &error);
	bool 	SetPdoParameterValue(PdoParameterName parameterName, long value, int &error);
	bool 	SetPdoParameterValue(PdoParameterName parameterName, float value, int &error);
	
	bool SendPdoSync(int &error = lastError);
	bool SendPdoRtr(PdoParameterName parameterName, int &error = lastError);
	

	bool SetPdoPositionReference(long positionReference, int &error = lastError);
	bool SetPdoSpeedReference(long speedReference, int &error = lastError);
	bool SetPdoTorqueReferenceIq(float torqueReferenceIq, int &error = lastError);
	bool SetPdoMagnetizingCurrentIdReference(float magnetizingCurrentIdReference, int &error = lastError);
	bool SetPdoControlMode(SOLOMotorControllers::ControlMode controlMode, int &error = lastError);
	bool SetPdoMotorDirection(SOLOMotorControllers::Direction motorDirection, int &error = lastError);

	long 	GetPdoPositionCountsFeedback(int &error = lastError);
	long	GetPdoSpeedFeedback (int &error = lastError);
	float 	GetPdoQuadratureCurrentIqFeedback(int &error = lastError);
	float 	GetPdoMagnetizingCurrentIdFeedback(int &error = lastError);
	long 	GetPdoErrorRegister(int &error = lastError);
	float 	GetPdoBoardTemperature(int &error = lastError);
	PdoParameterConfig 	GetPdoParameterConfig(PdoParameterName parameterName, int &error = lastError);
	bool 	UpdatePdoParameterCobIdByPdoParameterName();

/**
  * @}
  */

/** @addtogroup CANOpen_Write_Functions Standard CANOpen Write Functions
  * @{
  */
 	bool SetDeviceAddress(unsigned char deviceAddress, int &error = lastError);
	bool SetCommandMode(SOLOMotorControllers::CommandMode mode, int &error = lastError);
	bool SetCurrentLimit(float currentLimit, int &error = lastError);
	bool SetTorqueReferenceIq(float torqueReferenceIq, int &error = lastError);
	bool SetSpeedReference(long speedReference, int &error = lastError);
	bool SetPowerReference(float powerReference, int &error = lastError);
	bool MotorParametersIdentification(SOLOMotorControllers::Action identification, int &error = lastError);
	bool SetDriveDisableEnable(DisableEnable action, int &error = lastError);
	bool SetOutputPwmFrequencyKhz(long outputPwmFrequencyKhz, int &error = lastError);
	bool SetSpeedControllerKp(float speedControllerKp, int &error = lastError);
	bool SetSpeedControllerKi(float speedControllerKi, int &error = lastError);
	bool SetMotorDirection(SOLOMotorControllers::Direction motorDirection, int &error = lastError);
	bool SetMotorResistance(float motorResistance, int &error = lastError);
	bool SetMotorInductance(float motorInductance, int &error = lastError);
	bool SetMotorPolesCounts(long motorPolesCounts, int &error = lastError);
	bool SetIncrementalEncoderLines(long incrementalEncoderLines, int &error = lastError);
	bool SetSpeedLimit(long speedLimit, int &error = lastError);
	bool SetFeedbackControlMode(SOLOMotorControllers::FeedbackControlMode mode, int &error = lastError);
	bool ResetFactory(int &error = lastError);
	bool ResetDeviceAddress(int &error = lastError);
	bool SetMotorType(SOLOMotorControllers::MotorType motorType, int &error = lastError);
	bool SetControlMode(SOLOMotorControllers::ControlMode controlMode, int &error = lastError);
	bool SetCurrentControllerKp(float currentControllerKp, int &error = lastError);
	bool SetCurrentControllerKi(float currentControllerKi, int &error = lastError);
	bool SetMonitoringMode(bool mode, int &error = lastError);
	bool SetMagnetizingCurrentIdReference(float magnetizingCurrentIdReference, int &error = lastError);
	bool SetPositionReference(long positionReference, int &error = lastError);
	bool SetPositionControllerKp(float positionControllerKp, int &error = lastError);
	bool SetPositionControllerKi(float positionControllerKi, int &error = lastError);
	bool ResetPositionToZero(int &error = lastError);
	bool OverwriteErrorRegister(int &error = lastError);
	bool SetZsftInjectionAmplitude(float zsftInjectionAmplitude, int &error = lastError);
	bool SetZsftPolarityAmplitude(float zsftPolarityAmplitude, int &error = lastError);
	bool SetObserverGainDc(float observerGain, int &error = lastError);
	bool SetZsftInjectionFrequency(long zsftInjectionFrequency, int &error = lastError);
	bool SetSensorlessTransitionSpeed(long sensorlessTransitionSpeed, int &error = lastError);
	bool SetUartBaudrate(SOLOMotorControllers::UartBaudrate baudrate, int &error = lastError);
	bool SensorCalibration(SOLOMotorControllers::PositionSensorCalibrationAction calibrationAction, int &error = lastError);
	bool SetEncoderHallCcwOffset(float encoderHallOffset, int &error = lastError);
	bool SetEncoderHallCwOffset(float encoderHallOffset, int &error = lastError);
	bool SetSpeedAccelerationValue(float speedAccelerationValue, int &error = lastError);
	bool SetSpeedDecelerationValue(float speedDecelerationValue, int &error = lastError);
	bool SetCanbusBaudrate(CanbusBaudrate canbusBaudrate, int &error = lastError);
	bool SetAnalogueSpeedResolutionDivisionCoefficient(long divisionCoefficient, int &error = lastError);
    bool SetMotionProfileMode( MotionProfileMode motionProfileMode, int &error = lastError);
    bool SetMotionProfileVariable1(float MotionProfileVariable1, int &error = lastError);
    bool SetMotionProfileVariable2(float MotionProfileVariable2, int &error = lastError);
    bool SetMotionProfileVariable3(float MotionProfileVariable3, int &error = lastError);
    bool SetMotionProfileVariable4(float MotionProfileVariable4, int &error = lastError);
    bool SetMotionProfileVariable5(float MotionProfileVariable5, int &error = lastError);
	bool SetRegenerationCurrentLimit(float current, int &error = lastError);
	bool SetPositionSensorDigitalFilterLevel(long level, int &error = lastError);
	bool SetDigitalOutputState(Channel channel, DigitalIoState state, int &error = lastError);
/**
  * @}
  */

/** @addtogroup CANOpen_Read_Functions Specific CANOpen Read Functions
  * @{
  */
	long	GetGuardTime(int &error = lastError);
	long	GetLifeTimeFactor(int &error = lastError);
	long	GetProducerHeartbeatTime(int &error = lastError);
/**
  * @}
  */

/** @addtogroup CANOpen_Read_Functions Standard CANOpen Read Functions
  * @{
  */
	long	GetReadErrorRegister(int &error = lastError);
	long	GetDeviceAddress(int &error = lastError);
	float	GetPhaseAVoltage(int &error = lastError);
	float	GetPhaseBVoltage(int &error = lastError);
	float	GetPhaseACurrent(int &error = lastError);
	float	GetPhaseBCurrent(int &error = lastError);
	float	GetBusVoltage(int &error = lastError);
	float	GetDcMotorCurrentIm(int &error = lastError);
	float	GetDcMotorVoltageVm(int &error = lastError);
	float	GetSpeedControllerKp(int &error = lastError);
	float	GetSpeedControllerKi(int &error = lastError);
	long	GetOutputPwmFrequencyKhz(int &error = lastError);
	float	GetCurrentLimit(int &error = lastError);
	float	GetQuadratureCurrentIqFeedback(int &error = lastError);
	float	GetMagnetizingCurrentIdFeedback(int &error = lastError);
	long	GetMotorPolesCounts(int &error = lastError);
	long	GetIncrementalEncoderLines(int &error = lastError);
	float	GetCurrentControllerKp(int &error = lastError);
	float	GetCurrentControllerKi(int &error = lastError);
	float	GetBoardTemperature(int &error = lastError);
	float	GetMotorResistance(int &error = lastError);
	float	GetMotorInductance(int &error = lastError);
	long	GetSpeedFeedback(int &error = lastError);
	MotorType	GetMotorType(int &error = lastError);
	FeedbackControlMode	GetFeedbackControlMode(int &error = lastError);
	CommandMode	GetCommandMode(int &error = lastError);
	ControlMode	GetControlMode(int &error = lastError);
	long	GetSpeedLimit(int &error = lastError);
	float	GetPositionControllerKp(int &error = lastError);
	float	GetPositionControllerKi(int &error = lastError);
	long	GetPositionCountsFeedback(int &error = lastError);
	long	GetErrorRegister(int &error = lastError);
	long	GetDeviceFirmwareVersion(int &error = lastError);
	long	GetDeviceHardwareVersion(int &error = lastError);
	float	GetTorqueReferenceIq(int &error = lastError);
	long	GetSpeedReference(int &error = lastError);
	float 	GetMagnetizingCurrentIdReference(int &error = lastError);
	long  	GetPositionReference(int &error = lastError);
	float 	GetPowerReference(int &error = lastError);
	Direction  	GetMotorDirection(int &error = lastError);
	float 	GetZsftInjectionAmplitude(int &error = lastError);
	float 	GetZsftPolarityAmplitude(int &error = lastError);
	float 	GetObserverGainDc(int &error = lastError);
	long 	GetZsftInjectionFrequency(int &error = lastError);
	long 	GetSensorlessTransitionSpeed(int &error = lastError);
	float 	Get3PhaseMotorAngle(int &error = lastError);
	float 	GetEncoderHallCcwOffset(int &error = lastError);
	float 	GetEncoderHallCwOffset(int &error = lastError);
	UartBaudrate  	GetUartBaudrate(int &error = lastError);
	float 	GetSpeedAccelerationValue(int &error = lastError);
	float	GetSpeedDecelerationValue(int &error = lastError);
	long  	GetAnalogueSpeedResolutionDivisionCoefficient(int &error = lastError);
	bool  	CommunicationIsWorking(int &error = lastError);
	long  	GetEncoderIndexCounts(int &error = lastError);
	MotionProfileMode 	GetMotionProfileMode(int &error = lastError);
    float 	GetMotionProfileVariable1(int &error = lastError);
    float 	GetMotionProfileVariable2(int &error = lastError);
    float 	GetMotionProfileVariable3(int &error = lastError);
    float 	GetMotionProfileVariable4(int &error = lastError);
    float 	GetMotionProfileVariable5(int &error = lastError);
	DigitalIoState 	GetDigitalOutputState(Channel channel, int &error = lastError);
	long 	GetDigitalOutputsRegister(int &error);
	DisableEnable 	GetDriveDisableEnable(int &error = lastError);
	long 	GetPT1000SensorVoltage(int &error = lastError);
	void 	GeneralCanbusRead(uint16_t *ID , uint8_t *DLC, uint8_t *Data);
	void 	GeneralCanbusWrite(uint16_t ID, uint8_t *DLC, uint8_t *Data, int &error = lastError);
	float 	GetRegenerationCurrentLimit(int &error = lastError);
	long 	GetPositionSensorDigitalFilterLevel(int &error = lastError);
	long 	GetDigitalInputRegister(int &error = lastError);
	DigitalIoState 	GetAnalogueInput(Channel channel, int &error = lastError);
};
/**
  * @}
  */

#endif