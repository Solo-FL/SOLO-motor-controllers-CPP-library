// Copyright: (c) 2021, SOLO motor controllers project
// GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

/*
*    Title: SOLO Motor Controllers DLL
*    Author: SOLOMotorControllers
*    Date: 2022
*    Code version: 0.0.0
*    Availability: https://github.com/Solo-FL/SOLO-motor-controllers-CPP-library
This Library is made by SOLOMotorControllers.com
To learn more please visit:  https://www.SOLOMotorControllers.com/
*/

#pragma once

#include <Windows.h>
#include <tchar.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C"
{
#endif

class SOLOMotorControllers {

private:
	UINT8 addr = 0;
	bool isConnected = false;
	HANDLE hComm;
	UINT32 baudrate;
	UINT32 millisecondsTimeout;
	UINT32 packetFailureTrialAttempts = 1;

	char   ComPortName[10];
	BOOL   Status;
	DWORD  dNoOFBytestoWrite;              // No of bytes to write into the port
	DWORD  dNoOfBytesWritten = 0;          // No of bytes written to the port

	char  SerialBuffer[256];               // Buffer Containing Rxed Data
	DWORD NoBytesRecieved;
	char  TempChar;

public:

	//SOLOMotorControllers();

	SOLOMotorControllers(unsigned char _addr = 0, long _baudrate = 115200, long _millisecondsTimeout = 50, int _packetFailureTrialAttempts = 3);

	~SOLOMotorControllers();

	enum SOLOMotorControllersError
	{
		noErrorDetected = 0,
		generalError = -1,
		noProcessedCommand = -2,
		outOfRengeSetting = -3,
		packetFailureTrialAttemptsOverflow = -4
	};

	enum CommandMode
	{
		analogue = 0,
		digital = 1
	};
	enum Direction
	{
		clockwise = 0,
		counterclockwise = 1
	};
	enum FeedbackControlMode
	{
		sensorLess = 0,
		encoders = 1,
		hallSensors = 2
	};
	enum ControlMode
	{
		speedMode = 0,
		torqueMode = 1,
		positionMode = 2
	};
	enum MotorType
	{
		dc = 0,
		bldcPmsm = 1,
		acim = 2,
		bldcPmsmUltrafast = 3
	};
	enum UartBaudrate
	{
		rate937500 = 0,
		rate115200 = 1
	};
	enum CanbusBaudrate
	{
		rate1000 = 0,
		rate500 = 1,
		rate250 = 2,
		rate125 = 3,
		rate100 = 4
	};
	enum Action
	{
		stop = 0,
		start = 1
	};
	enum PositionSensorCalibrationAction
	{
		stopCalibration = 0,
		incrementalEncoderStartCalibration = 1,
		hallSensorStartCalibration = 2
	};

bool serialSetup(unsigned char _addr, char* _portName, long _baudrate, long _millisecondsTimeout = 200, int _packetFailureTrialAttempts = 5);

void Disconnect();

bool Test();

bool ExeCMD(unsigned char* cmd, int& error);

float ConvertToFloat(unsigned char* data);

void ConvertToData(float f, unsigned char* data);

long ConvertToLong(unsigned char* data);

void ConvertToData(long l, unsigned char* data);

void SplitData(unsigned char* data, unsigned char* cmd);

//----------Write----------

bool SetDeviceAddress(unsigned char deviceAddress, int& error);

bool SetDeviceAddress(unsigned char deviceAddress);

bool SetCommandMode(CommandMode mode, int& error);

bool SetCommandMode(CommandMode mode);

bool SetCurrentLimit(float currentLimit, int& error);

bool SetCurrentLimit(float currentLimit);

bool SetTorqueReferenceIq(float torqueReferenceIq, int& error);

bool SetTorqueReferenceIq(float torqueReferenceIq);

bool SetSpeedReference(long speedReference, int& error);

bool SetSpeedReference(long speedReference);

bool SetPowerReference(float powerReference, int& error);

bool SetPowerReference(float powerReference);

bool MotorParametersIdentification(Action identification, int& error);

bool MotorParametersIdentification(Action identification);

bool EmergencyStop(int& error);

bool EmergencyStop();

bool SetOutputPwmFrequencyKhz(long outputPwmFrequencyKhz, int& error);

bool SetOutputPwmFrequencyKhz(long outputPwmFrequencyKhz);

bool SetSpeedControllerKp(float speedControllerKp, int& error);

bool SetSpeedControllerKp(float speedControllerKp);

bool SetSpeedControllerKi(float speedControllerKi, int& error);

bool SetSpeedControllerKi(float speedControllerKi);

bool SetMotorDirection(Direction motorDirection, int& error);

bool SetMotorDirection(Direction motorDirection);

bool SetMotorResistance(float motorResistance, int& error);

bool SetMotorResistance(float motorResistance);

bool SetMotorInductance(float motorInductance, int& error);

bool SetMotorInductance(float motorInductance);

bool SetMotorPolesCounts(long motorPolesCounts, int& error);

bool SetMotorPolesCounts(long motorPolesCounts);

bool SetIncrementalEncoderLines(long incrementalEncoderLines, int& error);

bool SetIncrementalEncoderLines(long incrementalEncoderLines);

bool SetSpeedLimit(long speedLimit, int& error);

bool SetSpeedLimit(long speedLimit);

bool ResetDeviceAddress(int& error);

bool ResetDeviceAddress();

bool SetFeedbackControlMode(FeedbackControlMode mode, int& error);

bool SetFeedbackControlMode(FeedbackControlMode mode);

bool ResetFactory(int& error);

bool ResetFactory();

bool SetMotorType(MotorType motorType, int& error);

bool SetMotorType(MotorType motorType);

bool SetControlMode(ControlMode controlMode, int& error);

bool SetControlMode(ControlMode controlMode);

bool SetCurrentControllerKp(float currentControllerKp, int& error);

bool SetCurrentControllerKp(float currentControllerKp);

bool SetCurrentControllerKi(float currentControllerKi, int& error);

bool SetCurrentControllerKi(float currentControllerKi);

bool SetMonitoringMode(bool mode, int& error);

bool SetMonitoringMode(bool mode);

bool SetMagnetizingCurrentIdReference(float magnetizingCurrentIdReference, int& error);

bool SetMagnetizingCurrentIdReference(float magnetizingCurrentIdReference);

bool SetPositionReference(long positionReference, int& error);

bool SetPositionReference(long positionReference);

bool SetPositionControllerKp(float positionControllerKp, int& error);

bool SetPositionControllerKp(float positionControllerKp);

bool SetPositionControllerKi(float positionControllerKi, int& error);

bool SetPositionControllerKi(float positionControllerKi);

bool ResetPositionToZero(int& error); //Home

bool ResetPositionToZero();

bool OverwriteErrorRegister(int& error);

bool OverwriteErrorRegister();

bool SetObserverGainBldcPmsm(float observerGain, int& error);

bool SetObserverGainBldcPmsm(float observerGain);

bool SetObserverGainBldcPmsmUltrafast(float observerGain, int& error);

bool SetObserverGainBldcPmsmUltrafast(float observerGain);

bool SetObserverGainDc(float observerGain, int& error);

bool SetObserverGainDc(float observerGain);

bool SetFilterGainBldcPmsm(float filterGain, int& error);

bool SetFilterGainBldcPmsm(float filterGain);

bool SetFilterGainBldcPmsmUltrafast(float filterGain, int& error);

bool SetFilterGainBldcPmsmUltrafast(float filterGain);

bool SetUartBaudrate(UartBaudrate baudrate, int& error);

bool SetUartBaudrate(UartBaudrate baudrate);

bool SensorCalibration(PositionSensorCalibrationAction calibrationAction, int& error);

bool SensorCalibration(PositionSensorCalibrationAction calibrationAction);

bool SetEncoderHallCcwOffset(float encoderHallOffset, int& error);

bool SetEncoderHallCcwOffset(float encoderHallOffset);

bool SetEncoderHallCwOffset(float encoderHallOffset, int& error);

bool SetEncoderHallCwOffset(float encoderHallOffset);

bool SetSpeedAccelerationValue(float speedAccelerationValue, int& error);

bool SetSpeedAccelerationValue(float speedAccelerationValue);

bool SetSpeedDecelerationValue(float speedDecelerationValue, int& error);

bool SetSpeedDecelerationValue(float speedDecelerationValue);

bool SetCanbusBoudrate(CanbusBaudrate canbusBoudrate, int& error);

bool SetCanbusBoudrate(CanbusBaudrate canbusBoudrate);

//----------Read----------
long  GetDeviceAddress(int& error);

long  GetDeviceAddress();

float GetPhaseAVoltage(int& error);

float GetPhaseAVoltage();

float GetPhaseBVoltage(int& error);

float GetPhaseBVoltage();

float GetPhaseACurrent(int& error);

float GetPhaseACurrent();

float GetPhaseBCurrent(int& error);

float GetPhaseBCurrent();

float GetBusVoltage(int& error); //Battery Voltage

float GetBusVoltage();

float GetDcMotorCurrentIm(int& error);

float GetDcMotorCurrentIm();

float GetDcMotorVoltageVm(int& error);

float GetDcMotorVoltageVm();

float GetSpeedControllerKp(int& error);

float GetSpeedControllerKp();

float GetSpeedControllerKi(int& error);

float GetSpeedControllerKi();

long  GetOutputPwmFrequencyKhz(int& error);

long  GetOutputPwmFrequencyKhz();

float GetCurrentLimit(int& error);

float GetCurrentLimit();

float GetQuadratureCurrentIqFeedback(int& error);

float GetQuadratureCurrentIqFeedback();

float GetMagnetizingCurrentIdFeedback(int& error); //Magnetizing

float GetMagnetizingCurrentIdFeedback();

long  GetMotorPolesCounts(int& error);

long  GetMotorPolesCounts();

long  GetIncrementalEncoderLines(int& error);

long  GetIncrementalEncoderLines();

float GetCurrentControllerKp(int& error);

float GetCurrentControllerKp();

float GetCurrentControllerKi(int& error);

float GetCurrentControllerKi();

float GetBoardTemperature(int& error);

float GetBoardTemperature();

float GetMotorResistance(int& error);

float GetMotorResistance();

float GetMotorInductance(int& error);

float GetMotorInductance();

long  GetSpeedFeedback(int& error);

long  GetSpeedFeedback();

long  GetMotorType(int& error);

long  GetMotorType();

long  GetFeedbackControlMode(int& error);

long  GetFeedbackControlMode();

long  GetCommandMode(int& error);

long  GetCommandMode();

long  GetControlMode(int& error);

long  GetControlMode();

long  GetSpeedLimit(int& error);

long  GetSpeedLimit();

float GetPositionControllerKp(int& error);

float GetPositionControllerKp();

float GetPositionControllerKi(int& error);

float GetPositionControllerKi();

long  GetPositionCountsFeedback(int& error);

long  GetPositionCountsFeedback();

long  GetErrorRegister(int& error);

long  GetErrorRegister();

long  GetDeviceFirmwareVersion(int& error);

long  GetDeviceFirmwareVersion();

long  GetDeviceHardwareVersion(int& error);

long  GetDeviceHardwareVersion();

float GetTorqueReferenceIq(int& error);

float GetTorqueReferenceIq();

long  GetSpeedReference(int& error);

long  GetSpeedReference();

float GetMagnetizingCurrentIdReference(int& error);

float GetMagnetizingCurrentIdReference();

long  GetPositionReference(int& error);

long  GetPositionReference();

float GetPowerReference(int& error);

float GetPowerReference();

long  GetMotorDirection(int& error);

long  GetMotorDirection();

float GetObserverGainBldcPmsm(int& error);

float GetObserverGainBldcPmsm();

float GetObserverGainBldcPmsmUltrafast(int& error);

float GetObserverGainBldcPmsmUltrafast();

float GetObserverGainDc(int& error);

float GetObserverGainDc();

float GetFilterGainBldcPmsm(int& error);

float GetFilterGainBldcPmsm();

float GetFilterGainBldcPmsmUltrafast(int& error);

float GetFilterGainBldcPmsmUltrafast();

float Get3PhaseMotorAngle(int& error); // Read Estimated or Measured Rotor Angle

float Get3PhaseMotorAngle();

float GetEncoderHallCcwOffset(int& error);

float GetEncoderHallCcwOffset();

float GetEncoderHallCwOffset(int& error);

float GetEncoderHallCwOffset();

long  GetUartBaudrate(int& error);

long  GetUartBaudrate();

float GetSpeedAccelerationValue(int& error);

float GetSpeedAccelerationValue();

float GetSpeedDecelerationValue(int& error);

float GetSpeedDecelerationValue();

long  GetEncoderIndexCounts(int& error);

long  GetEncoderIndexCounts();

bool serialIsWorking(int& error);

bool serialIsWorking();

};

#ifdef __cplusplus
} // __cplusplus defined.
#endif
