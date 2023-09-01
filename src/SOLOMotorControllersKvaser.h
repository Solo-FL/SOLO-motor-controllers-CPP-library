/**
 *******************************************************************************
 * @file    SOLOMotorControllersKvaser.h
 * @authors SOLO Motor Controllers
 * @brief   This file contains all the base functions prototypes for the Solo Drivers
 *          Availability: https://github.com/Solo-FL/SOLO-motor-controllers-CPP-library
 * 
 * @date    Date: 2023
 * @version 1.2.0
 * *******************************************************************************    
 * @attention
 * Copyright: (c) 2021-2023, SOLO motor controllers project
 * GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)
 ******************************************************************************* 
 */

#pragma once

#include <stdint.h>
#include "SOLOMotorControllers.h"
#include "SOLOMotorControllersUtils.h"

#include "canlib.h"
//#include "Kvaser.h"
#include "CommunicationInterface.h"
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
#define Object_EmergencyStop                           0x3008
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
#define Object_ObserverGainBldcPmsm                    0x3021
#define Object_ObserverGainBldcPmsmUltrafast           0x3022
#define Object_ObserverGainDc                          0x3023
#define Object_FilterGainBldcPmsm                      0x3024
#define Object_FilterGainBldcPmsmUltrafast             0x3025
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
class SOLOMotorControllersKvaser : public SOLOMotorControllers
{

private:
	uint8_t Address;
	//canHandle hnd;		
	CommunicationInterface* kvaser;
	SOLOMotorControllersUtils *soloUtils;
	long timeout;
	long canBaudrate;
	int pdoParameterObjectByPdoParameterName[PdoParameterNameCount];

public:
	int pdoParameterCobIdByPdoParameterName[PdoParameterNameCount];
	SOLOMotorControllersKvaser(CommunicationInterface* ci,
            UINT8 deviceAddress = 0,
            SOLOMotorControllers::CanbusBaudrate baudrate = SOLOMotorControllers::CanbusBaudrate::rate1000,
	 		long millisecondsTimeout = 200, bool autoConnect = true);

	~SOLOMotorControllersKvaser();

private:
	float 	ConvertToFloat(unsigned char data[]);
	long  	ConvertToLong(unsigned char data[]);
	void  	ConvertToData(float f, unsigned char data[]);
	void  	ConvertToData(long l, unsigned char data[]);
	void  	ExtractData(unsigned char _Data[], unsigned char _ExtractedData[]);

public:
	bool Connect(UINT8 deviceAddress, 
		SOLOMotorControllers::CanbusBaudrate baudrate = SOLOMotorControllers::CanbusBaudrate::rate1000,
		long millisecondsTimeout = 200);

	bool Connect() override { return kvaser->Connect(); };
	void Disconnect() override { return kvaser->Disconnect(); };

	//void Disconnect();
	
/** @addtogroup CANOpen_Write_Functions_Specific Specific CANOpen Write Functions
  * @{
  */
	//----------Write  SOLOMotorControllersCanopen----------
	bool SetGuardTime(long guardtime, int& error);
	bool SetGuardTime(long guardtime);
	bool SetLifeTimeFactor(long lifeTimeFactor, int& error);
	bool SetLifeTimeFactor(long lifeTimeFactor);
	bool SetProducerHeartbeatTime(long producerHeartbeatTime, int& error);
	bool SetProducerHeartbeatTime(long producerHeartbeatTime);
	bool SetPdoParameterConfig(PdoParameterConfig config, int &error);

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
	bool 	SetPdoParameterValue(PdoParameterName parameterName, long value);
	bool 	SetPdoParameterValue(PdoParameterName parameterName, float value, int &error);
	bool 	SetPdoParameterValue(PdoParameterName parameterName, float value);
	
	bool SendPdoSync(int &error);
	bool SendPdoSync();
	bool SendPdoRtr(PdoParameterName parameterName, int &error);
	bool SendPdoRtr(PdoParameterName parameterName);
	

	bool SetPdoPositionReference(long positionReference, int& error);
	bool SetPdoPositionReference(long positionReference);
	bool SetPdoSpeedReference(long speedReference, int &error);
	bool SetPdoSpeedReference(long speedReference);
	bool SetPdoTorqueReferenceIq(float torqueReferenceIq, int& error);
	bool SetPdoTorqueReferenceIq(float torqueReferenceIq);
	bool SetPdoMagnetizingCurrentIdReference(float magnetizingCurrentIdReference, int& error);
	bool SetPdoMagnetizingCurrentIdReference(float magnetizingCurrentIdReference);
	bool SetPdoControlMode(SOLOMotorControllers::ControlMode controlMode, int& error);
	bool SetPdoControlMode(SOLOMotorControllers::ControlMode controlMode);
	bool SetPdoMotorDirection(SOLOMotorControllers::Direction motorDirection, int& error);
	bool SetPdoMotorDirection(SOLOMotorControllers::Direction motorDirection);

	long 	GetPdoPositionCountsFeedback(int& error);
	long 	GetPdoPositionCountsFeedback();
	long	GetPdoSpeedFeedback (int &error);
	long 	GetPdoSpeedFeedback ();
	float 	GetPdoQuadratureCurrentIqFeedback(int& error);
	float 	GetPdoQuadratureCurrentIqFeedback();
	float 	GetPdoMagnetizingCurrentIdFeedback(int& error);
	float 	GetPdoMagnetizingCurrentIdFeedback();
	long 	GetPdoErrorRegister(int& error);
	long 	GetPdoErrorRegister();
	float 	GetPdoBoardTemperature(int& error);
	float 	GetPdoBoardTemperature();
	PdoParameterConfig 	GetPdoParameterConfig(PdoParameterName parameterName, int &error);
	bool 	UpdatePdoParameterCobIdByPdoParameterName();

/**
  * @}
  */

/** @addtogroup CANOpen_Write_Functions Standard CANOpen Write Functions
  * @{
  */
	//----------Write  SOLOMotorControllers----------   
	bool SetDeviceAddress(unsigned char deviceAddress, int& error) override;
	bool SetDeviceAddress(unsigned char deviceAddress) override;
	bool SetCommandMode(SOLOMotorControllers::CommandMode mode, int& error) override;
	bool SetCommandMode(SOLOMotorControllers::CommandMode mode) override;
	bool SetCurrentLimit(float currentLimit, int& error) override;
	bool SetCurrentLimit(float currentLimit) override;
	bool SetTorqueReferenceIq(float torqueReferenceIq, int& error) override;
	bool SetTorqueReferenceIq(float torqueReferenceIq) override;
	bool SetSpeedReference(long speedReference, int& error) override;
	bool SetSpeedReference(long speedReference) override;
	bool SetPowerReference(float powerReference, int& error) override;
	bool SetPowerReference(float powerReference) override;
	bool MotorParametersIdentification(SOLOMotorControllers::Action identification, int& error) override;
	bool MotorParametersIdentification(SOLOMotorControllers::Action identification) override;
	bool EmergencyStop(int& error) override;
	bool EmergencyStop() override;
	bool SetOutputPwmFrequencyKhz(long outputPwmFrequencyKhz, int& error) override;
	bool SetOutputPwmFrequencyKhz(long outputPwmFrequencyKhz) override;
	bool SetSpeedControllerKp(float speedControllerKp, int& error) override;
	bool SetSpeedControllerKp(float speedControllerKp) override;
	bool SetSpeedControllerKi(float speedControllerKi, int& error) override;
	bool SetSpeedControllerKi(float speedControllerKi) override;
	bool SetMotorDirection(SOLOMotorControllers::Direction motorDirection, int& error) override;
	bool SetMotorDirection(SOLOMotorControllers::Direction motorDirection) override;
	bool SetMotorResistance(float motorResistance, int& error) override;
	bool SetMotorResistance(float motorResistance) override;
	bool SetMotorInductance(float motorInductance, int& error) override;
	bool SetMotorInductance(float motorInductance) override;
	bool SetMotorPolesCounts(long motorPolesCounts, int& error) override;
	bool SetMotorPolesCounts(long motorPolesCounts) override;
	bool SetIncrementalEncoderLines(long incrementalEncoderLines, int& error) override;
	bool SetIncrementalEncoderLines(long incrementalEncoderLines) override;
	bool SetSpeedLimit(long speedLimit, int& error) override;
	bool SetSpeedLimit(long speedLimit) override;
	bool SetFeedbackControlMode(SOLOMotorControllers::FeedbackControlMode mode, int& error) override;
	bool SetFeedbackControlMode(SOLOMotorControllers::FeedbackControlMode mode) override;
	bool ResetFactory(int& error) override;
	bool ResetFactory() override;
	bool ResetDeviceAddress(int& error) override;
	bool ResetDeviceAddress() override;
	bool SetMotorType(SOLOMotorControllers::MotorType motorType, int& error) override;
	bool SetMotorType(SOLOMotorControllers::MotorType motorType) override;
	bool SetControlMode(SOLOMotorControllers::ControlMode controlMode, int& error) override;
	bool SetControlMode(SOLOMotorControllers::ControlMode controlMode) override;
	bool SetCurrentControllerKp(float currentControllerKp, int& error) override;
	bool SetCurrentControllerKp(float currentControllerKp) override;
	bool SetCurrentControllerKi(float currentControllerKi, int& error) override;
	bool SetCurrentControllerKi(float currentControllerKi) override;
	bool SetMonitoringMode(bool mode, int& error) override;
	bool SetMonitoringMode(bool mode) override;
	bool SetMagnetizingCurrentIdReference(float magnetizingCurrentIdReference, int& error) override;
	bool SetMagnetizingCurrentIdReference(float magnetizingCurrentIdReference) override;
	bool SetPositionReference(long positionReference, int& error) override;
	bool SetPositionReference(long positionReference) override;
	bool SetPositionControllerKp(float positionControllerKp, int& error) override;
	bool SetPositionControllerKp(float positionControllerKp) override;
	bool SetPositionControllerKi(float positionControllerKi, int& error) override;
	bool SetPositionControllerKi(float positionControllerKi) override;
	bool ResetPositionToZero(int& error) override; //Home
	bool ResetPositionToZero() override;
	bool OverwriteErrorRegister(int& error) override;
	bool OverwriteErrorRegister() override;
	bool SetObserverGainBldcPmsm(float observerGain, int& error) override;
	bool SetObserverGainBldcPmsm(float observerGain) override;
	bool SetObserverGainBldcPmsmUltrafast(float observerGain, int& error) override;
	bool SetObserverGainBldcPmsmUltrafast(float observerGain) override;
	bool SetObserverGainDc(float observerGain, int& error) override;
	bool SetObserverGainDc(float observerGain) override;
	bool SetFilterGainBldcPmsm(float filterGain, int& error) override;
	bool SetFilterGainBldcPmsm(float filterGain) override;
	bool SetFilterGainBldcPmsmUltrafast(float filterGain, int& error) override;
	bool SetFilterGainBldcPmsmUltrafast(float filterGain) override;
	bool SetUartBaudrate(SOLOMotorControllers::UartBaudrate baudrate, int& error) override;
	bool SetUartBaudrate(SOLOMotorControllers::UartBaudrate baudrate) override;
	bool SensorCalibration(SOLOMotorControllers::PositionSensorCalibrationAction calibrationAction, int& error) override;
	bool SensorCalibration(SOLOMotorControllers::PositionSensorCalibrationAction calibrationAction) override;
	bool SetEncoderHallCcwOffset(float encoderHallOffset, int& error) override;
	bool SetEncoderHallCcwOffset(float encoderHallOffset) override;
	bool SetEncoderHallCwOffset(float encoderHallOffset, int& error) override;
	bool SetEncoderHallCwOffset(float encoderHallOffset) override;
	bool SetSpeedAccelerationValue(float speedAccelerationValue, int& error) override;
	bool SetSpeedAccelerationValue(float speedAccelerationValue) override;
	bool SetSpeedDecelerationValue(float speedDecelerationValue, int& error) override;
	bool SetSpeedDecelerationValue(float speedDecelerationValue) override;
	bool SetCanbusBaudrate(CanbusBaudrate canbusBaudrate, int& error) override;
	bool SetCanbusBaudrate(CanbusBaudrate canbusBaudrate) override;
	bool SetAnalogueSpeedResolutionDivisionCoefficient(long divisionCoefficient, int &error) override;
    bool SetAnalogueSpeedResolutionDivisionCoefficient(long divisionCoefficient) override;
    bool SetMotionProfileMode( MotionProfileMode motionProfileMode, int &error) override;
    bool SetMotionProfileMode( MotionProfileMode motionProfileMode) override;
    bool SetMotionProfileVariable1(float MotionProfileVariable1, int &error) override;
    bool SetMotionProfileVariable1(float MotionProfileVariable1) override;
    bool SetMotionProfileVariable2(float MotionProfileVariable2, int &error) override;
    bool SetMotionProfileVariable2(float MotionProfileVariable2) override;
    bool SetMotionProfileVariable3(float MotionProfileVariable3, int &error) override;
    bool SetMotionProfileVariable3(float MotionProfileVariable3) override;
    bool SetMotionProfileVariable4(float MotionProfileVariable4, int &error) override;
    bool SetMotionProfileVariable4(float MotionProfileVariable4) override;
    bool SetMotionProfileVariable5(float MotionProfileVariable5, int &error) override;
    bool SetMotionProfileVariable5(float MotionProfileVariable5) override;
/**
  * @}
  */

/** @addtogroup CANOpen_Read_Functions Specific CANOpen Read Functions
  * @{
  */
	long	GetGuardTime(int& error);
	long	GetGuardTime();
	long	GetLifeTimeFactor(int& error);
	long	GetLifeTimeFactor();
	long	GetProducerHeartbeatTime(int& error);
	long	GetProducerHeartbeatTime();
/**
  * @}
  */

/** @addtogroup CANOpen_Read_Functions Standard CANOpen Read Functions
  * @{
  */
	//----------Read SOLOMotorControllers----------
	long	GetReadErrorRegister(int& error);
	long	GetReadErrorRegister();
	long	GetDeviceAddress(int& error) override;
	long	GetDeviceAddress() override;
	float	GetPhaseAVoltage(int& error) override;
	float	GetPhaseAVoltage() override;
	float	GetPhaseBVoltage(int& error) override;
	float	GetPhaseBVoltage() override;
	float	GetPhaseACurrent(int& error) override;
	float	GetPhaseACurrent() override;
	float	GetPhaseBCurrent(int& error) override;
	float	GetPhaseBCurrent() override;
	float	GetBusVoltage(int& error) override; //Battery Voltage
	float	GetBusVoltage() override;
	float	GetDcMotorCurrentIm(int& error) override;
	float	GetDcMotorCurrentIm() override;
	float	GetDcMotorVoltageVm(int& error) override;
	float	GetDcMotorVoltageVm() override;
	float	GetSpeedControllerKp(int& error) override;
	float	GetSpeedControllerKp() override;
	float	GetSpeedControllerKi(int& error) override;
	float	GetSpeedControllerKi() override;
	long	GetOutputPwmFrequencyKhz(int& error) override;
	long	GetOutputPwmFrequencyKhz() override;
	float	GetCurrentLimit(int& error) override;
	float	GetCurrentLimit() override;
	float	GetQuadratureCurrentIqFeedback(int& error) override;
	float	GetQuadratureCurrentIqFeedback() override;
	float	GetMagnetizingCurrentIdFeedback(int& error) override; //Magnetizing
	float	GetMagnetizingCurrentIdFeedback() override;
	long	GetMotorPolesCounts(int& error) override;
	long	GetMotorPolesCounts() override;
	long	GetIncrementalEncoderLines(int& error) override;
	long	GetIncrementalEncoderLines() override;
	float	GetCurrentControllerKp(int& error) override;
	float	GetCurrentControllerKp() override;
	float	GetCurrentControllerKi(int& error) override;
	float	GetCurrentControllerKi() override;
	float	GetBoardTemperature(int& error) override;
	float	GetBoardTemperature() override;
	float	GetMotorResistance(int& error) override;
	float	GetMotorResistance() override;
	float	GetMotorInductance(int& error) override;
	float	GetMotorInductance() override;
	long	GetSpeedFeedback(int& error) override;
	long 	GetSpeedFeedback() override;
	long	GetMotorType(int& error) override;
	long	GetMotorType() override;
	long	GetFeedbackControlMode(int& error) override;
	long	GetFeedbackControlMode() override;
	long	GetCommandMode(int& error) override;
	long	GetCommandMode() override;
	long	GetControlMode(int& error) override;
	long	GetControlMode() override;
	long	GetSpeedLimit(int& error) override;
	long	GetSpeedLimit() override;
	float	GetPositionControllerKp(int& error) override;
	float	GetPositionControllerKp() override;
	float	GetPositionControllerKi(int& error) override;
	float	GetPositionControllerKi() override;
	long	GetPositionCountsFeedback(int& error) override;
	long	GetPositionCountsFeedback() override;
	long	GetErrorRegister(int& error) override;
	long	GetErrorRegister() override;
	long	GetDeviceFirmwareVersion(int& error) override;
	long	GetDeviceFirmwareVersion() override;
	long	GetDeviceHardwareVersion(int& error) override;
	long	GetDeviceHardwareVersion() override;
	float	GetTorqueReferenceIq(int& error) override;
	float	GetTorqueReferenceIq() override;
	long	GetSpeedReference(int& error) override;
	long 	GetSpeedReference() override;
	float 	GetMagnetizingCurrentIdReference(int& error) override;
	float 	GetMagnetizingCurrentIdReference() override;
	long  	GetPositionReference(int& error) override;
	long  	GetPositionReference() override;
	float 	GetPowerReference(int& error) override;
	float 	GetPowerReference() override;
	long  	GetMotorDirection(int& error) override;
	long  	GetMotorDirection() override;
	float 	GetObserverGainBldcPmsm(int& error) override;
	float 	GetObserverGainBldcPmsm() override;
	float 	GetObserverGainBldcPmsmUltrafast(int& error) override;
	float 	GetObserverGainBldcPmsmUltrafast() override;
	float 	GetObserverGainDc(int& error) override;
	float 	GetObserverGainDc() override;
	float 	GetFilterGainBldcPmsm(int& error) override;
	float 	GetFilterGainBldcPmsm() override;
	float 	GetFilterGainBldcPmsmUltrafast(int& error) override;
	float 	GetFilterGainBldcPmsmUltrafast() override;
	float 	Get3PhaseMotorAngle(int& error) override; // Read Estimated or Measured Rotor Angle
	float 	Get3PhaseMotorAngle() override;
	float 	GetEncoderHallCcwOffset(int& error) override;
	float 	GetEncoderHallCcwOffset() override;
	float 	GetEncoderHallCwOffset(int& error) override;
	float 	GetEncoderHallCwOffset() override;
	long  	GetUartBaudrate(int& error) override;
	long  	GetUartBaudrate() override;
	float 	GetSpeedAccelerationValue(int& error) override;
	float 	GetSpeedAccelerationValue() override;
	float	GetSpeedDecelerationValue(int& error) override;
	float 	GetSpeedDecelerationValue() override;
	long  	GetAnalogueSpeedResolutionDivisionCoefficient(int &error) override;
    long  	GetAnalogueSpeedResolutionDivisionCoefficient() override;
	bool  	CommunicationIsWorking(int& error) override;
	bool  	CommunicationIsWorking() override;
	long  	GetEncoderIndexCounts(int& error) override;
	long  	GetEncoderIndexCounts() override;
	long 	GetMotionProfileMode(int &error) override;
    long 	GetMotionProfileMode() override;
    float 	GetMotionProfileVariable1(int &error) override;
    float 	GetMotionProfileVariable1() override;
    float 	GetMotionProfileVariable2(int &error) override;
    float 	GetMotionProfileVariable2() override;
    float 	GetMotionProfileVariable3(int &error) override;
    float 	GetMotionProfileVariable3() override;
    float 	GetMotionProfileVariable4(int &error) override;
    float 	GetMotionProfileVariable4() override;
    float 	GetMotionProfileVariable5(int &error) override;
    float 	GetMotionProfileVariable5() override;
	void 	GeneralCanbusRead(uint16_t *ID , uint8_t *DLC, uint8_t *Data);
	void 	GeneralCanbusWrite(uint16_t ID, uint8_t *DLC, uint8_t *Data, int &error);
};
/**
  * @}
  */

#endif
