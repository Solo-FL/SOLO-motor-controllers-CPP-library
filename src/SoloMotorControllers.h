// Copyright: (c) 2021-2022, SOLO motor controllers project
// GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

/*
*    Title: SOLO Motor Controllers DLL
*    Author: SOLOMotorControllers
*    Date: 2022
*    Code version: 1.0.0
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

	public:

		enum Error
		{
			noErrorDetected = 0,
			generalError = 1,
			noProcessedCommand = 2,
			outOfRengeSetting = 3,
			packetFailureTrialAttemptsOverflow = 4,
			recieveTimeOutError = 5,
			Abort_Object = 6,
			Abort_Value = 7,
			MCP2515_Transmit_ArbitrationLost = 8,
			MCP2515_Transmit_Error = 9,
			objectNotInitialize = 10
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

		virtual bool Connect() = 0;

		virtual void Disconnect() = 0;

		//----------Write----------

		virtual bool SetDeviceAddress(unsigned char deviceAddress, int& error) = 0;

		virtual bool SetDeviceAddress(unsigned char deviceAddress) = 0;

		virtual bool SetCommandMode(CommandMode mode, int& error) = 0;

		virtual bool SetCommandMode(CommandMode mode) = 0;

		virtual bool SetCurrentLimit(float currentLimit, int& error) = 0;

		virtual bool SetCurrentLimit(float currentLimit) = 0;

		virtual bool SetTorqueReferenceIq(float torqueReferenceIq, int& error) = 0;

		virtual bool SetTorqueReferenceIq(float torqueReferenceIq) = 0;

		virtual bool SetSpeedReference(long speedReference, int& error) = 0;

		virtual bool SetSpeedReference(long speedReference) = 0;

		virtual bool SetPowerReference(float powerReference, int& error) = 0;

		virtual bool SetPowerReference(float powerReference) = 0;

		virtual bool MotorParametersIdentification(Action identification, int& error) = 0;

		virtual bool MotorParametersIdentification(Action identification) = 0;

		virtual bool EmergencyStop(int& error) = 0;

		virtual bool EmergencyStop() = 0;

		virtual bool SetOutputPwmFrequencyKhz(long outputPwmFrequencyKhz, int& error) = 0;

		virtual bool SetOutputPwmFrequencyKhz(long outputPwmFrequencyKhz) = 0;

		virtual bool SetSpeedControllerKp(float speedControllerKp, int& error) = 0;

		virtual bool SetSpeedControllerKp(float speedControllerKp) = 0;

		virtual bool SetSpeedControllerKi(float speedControllerKi, int& error) = 0;

		virtual bool SetSpeedControllerKi(float speedControllerKi) = 0;

		virtual bool SetMotorDirection(Direction motorDirection, int& error) = 0;

		virtual bool SetMotorDirection(Direction motorDirection) = 0;

		virtual bool SetMotorResistance(float motorResistance, int& error) = 0;

		virtual bool SetMotorResistance(float motorResistance) = 0;

		virtual bool SetMotorInductance(float motorInductance, int& error) = 0;

		virtual bool SetMotorInductance(float motorInductance) = 0;

		virtual bool SetMotorPolesCounts(long motorPolesCounts, int& error) = 0;

		virtual bool SetMotorPolesCounts(long motorPolesCounts) = 0;

		virtual bool SetIncrementalEncoderLines(long incrementalEncoderLines, int& error) = 0;

		virtual bool SetIncrementalEncoderLines(long incrementalEncoderLines) = 0;

		virtual bool SetSpeedLimit(long speedLimit, int& error) = 0;

		virtual bool SetSpeedLimit(long speedLimit) = 0;

		virtual bool ResetDeviceAddress(int& error) = 0;

		virtual bool ResetDeviceAddress() = 0;

		virtual bool SetFeedbackControlMode(FeedbackControlMode mode, int& error) = 0;

		virtual bool SetFeedbackControlMode(FeedbackControlMode mode) = 0;

		virtual bool ResetFactory(int& error) = 0;

		virtual bool ResetFactory() = 0;

		virtual bool SetMotorType(MotorType motorType, int& error) = 0;

		virtual bool SetMotorType(MotorType motorType) = 0;

		virtual bool SetControlMode(ControlMode controlMode, int& error) = 0;

		virtual bool SetControlMode(ControlMode controlMode) = 0;

		virtual bool SetCurrentControllerKp(float currentControllerKp, int& error) = 0;

		virtual bool SetCurrentControllerKp(float currentControllerKp) = 0;

		virtual bool SetCurrentControllerKi(float currentControllerKi, int& error) = 0;

		virtual bool SetCurrentControllerKi(float currentControllerKi) = 0;

		virtual bool SetMonitoringMode(bool mode, int& error) = 0;

		virtual bool SetMonitoringMode(bool mode) = 0;

		virtual bool SetMagnetizingCurrentIdReference(float magnetizingCurrentIdReference, int& error) = 0;

		virtual bool SetMagnetizingCurrentIdReference(float magnetizingCurrentIdReference) = 0;

		virtual bool SetPositionReference(long positionReference, int& error) = 0;

		virtual bool SetPositionReference(long positionReference) = 0;

		virtual bool SetPositionControllerKp(float positionControllerKp, int& error) = 0;

		virtual bool SetPositionControllerKp(float positionControllerKp) = 0;

		virtual bool SetPositionControllerKi(float positionControllerKi, int& error) = 0;

		virtual bool SetPositionControllerKi(float positionControllerKi) = 0;

		virtual bool ResetPositionToZero(int& error) = 0; //Home

		virtual bool ResetPositionToZero() = 0;

		virtual bool OverwriteErrorRegister(int& error) = 0;

		virtual bool OverwriteErrorRegister() = 0;

		virtual bool SetObserverGainBldcPmsm(float observerGain, int& error) = 0;

		virtual bool SetObserverGainBldcPmsm(float observerGain) = 0;

		virtual bool SetObserverGainBldcPmsmUltrafast(float observerGain, int& error) = 0;

		virtual bool SetObserverGainBldcPmsmUltrafast(float observerGain) = 0;

		virtual bool SetObserverGainDc(float observerGain, int& error) = 0;

		virtual bool SetObserverGainDc(float observerGain) = 0;

		virtual bool SetFilterGainBldcPmsm(float filterGain, int& error) = 0;

		virtual bool SetFilterGainBldcPmsm(float filterGain) = 0;

		virtual bool SetFilterGainBldcPmsmUltrafast(float filterGain, int& error) = 0;

		virtual bool SetFilterGainBldcPmsmUltrafast(float filterGain) = 0;

		virtual bool SetUartBaudrate(UartBaudrate baudrate, int& error) = 0;

		virtual bool SetUartBaudrate(UartBaudrate baudrate) = 0;

		virtual bool SensorCalibration(PositionSensorCalibrationAction calibrationAction, int& error) = 0;

		virtual bool SensorCalibration(PositionSensorCalibrationAction calibrationAction) = 0;

		virtual bool SetEncoderHallCcwOffset(float encoderHallOffset, int& error) = 0;

		virtual bool SetEncoderHallCcwOffset(float encoderHallOffset) = 0;

		virtual bool SetEncoderHallCwOffset(float encoderHallOffset, int& error) = 0;

		virtual bool SetEncoderHallCwOffset(float encoderHallOffset) = 0;

		virtual bool SetSpeedAccelerationValue(float speedAccelerationValue, int& error) = 0;

		virtual bool SetSpeedAccelerationValue(float speedAccelerationValue) = 0;

		virtual bool SetSpeedDecelerationValue(float speedDecelerationValue, int& error) = 0;

		virtual bool SetSpeedDecelerationValue(float speedDecelerationValue) = 0;

		virtual bool SetCanbusBaudrate(CanbusBaudrate canbusBoudrate, int& error) = 0;

		virtual bool SetCanbusBaudrate(CanbusBaudrate canbusBoudrate) = 0;

		////----------Read----------
		virtual long  GetDeviceAddress(int& error) = 0;

		virtual long  GetDeviceAddress() = 0;

		virtual float GetPhaseAVoltage(int& error) = 0;

		virtual float GetPhaseAVoltage() = 0;

		virtual float GetPhaseBVoltage(int& error) = 0;

		virtual float GetPhaseBVoltage() = 0;

		virtual float GetPhaseACurrent(int& error) = 0;

		virtual float GetPhaseACurrent() = 0;

		virtual float GetPhaseBCurrent(int& error) = 0;

		virtual float GetPhaseBCurrent() = 0;

		virtual float GetBusVoltage(int& error) = 0; //Battery Voltage

		virtual float GetBusVoltage() = 0;

		virtual float GetDcMotorCurrentIm(int& error) = 0;

		virtual float GetDcMotorCurrentIm() = 0;

		virtual float GetDcMotorVoltageVm(int& error) = 0;

		virtual float GetDcMotorVoltageVm() = 0;

		virtual float GetSpeedControllerKp(int& error) = 0;

		virtual float GetSpeedControllerKp() = 0;

		virtual float GetSpeedControllerKi(int& error) = 0;

		virtual float GetSpeedControllerKi() = 0;

		virtual long  GetOutputPwmFrequencyKhz(int& error) = 0;

		virtual long  GetOutputPwmFrequencyKhz() = 0;

		virtual float GetCurrentLimit(int& error) = 0;

		virtual float GetCurrentLimit() = 0;

		virtual float GetQuadratureCurrentIqFeedback(int& error) = 0;

		virtual float GetQuadratureCurrentIqFeedback() = 0;

		virtual float GetMagnetizingCurrentIdFeedback(int& error) = 0; //Magnetizing

		virtual float GetMagnetizingCurrentIdFeedback() = 0;

		virtual long  GetMotorPolesCounts(int& error) = 0;

		virtual long  GetMotorPolesCounts() = 0;

		virtual long  GetIncrementalEncoderLines(int& error) = 0;

		virtual long  GetIncrementalEncoderLines() = 0;

		virtual float GetCurrentControllerKp(int& error) = 0;

		virtual float GetCurrentControllerKp() = 0;

		virtual float GetCurrentControllerKi(int& error) = 0;

		virtual float GetCurrentControllerKi() = 0;

		virtual float GetBoardTemperature(int& error) = 0;

		virtual float GetBoardTemperature() = 0;

		virtual float GetMotorResistance(int& error) = 0;

		virtual float GetMotorResistance() = 0;

		virtual float GetMotorInductance(int& error) = 0;

		virtual float GetMotorInductance() = 0;

		virtual long  GetSpeedFeedback(int& error) = 0;

		virtual long  GetSpeedFeedback() = 0;

		virtual long  GetMotorType(int& error) = 0;

		virtual long  GetMotorType() = 0;

		virtual long  GetFeedbackControlMode(int& error) = 0;

		virtual long  GetFeedbackControlMode() = 0;

		virtual long  GetCommandMode(int& error) = 0;

		virtual long  GetCommandMode() = 0;

		virtual long  GetControlMode(int& error) = 0;

		virtual long  GetControlMode() = 0;

		virtual long  GetSpeedLimit(int& error) = 0;

		virtual long  GetSpeedLimit() = 0;

		virtual float GetPositionControllerKp(int& error) = 0;

		virtual float GetPositionControllerKp() = 0;

		virtual float GetPositionControllerKi(int& error) = 0;

		virtual float GetPositionControllerKi() = 0;

		virtual long  GetPositionCountsFeedback(int& error) = 0;

		virtual long  GetPositionCountsFeedback() = 0;

		virtual long  GetErrorRegister(int& error) = 0;

		virtual long  GetErrorRegister() = 0;

		virtual long  GetDeviceFirmwareVersion(int& error) = 0;

		virtual long  GetDeviceFirmwareVersion() = 0;

		virtual long  GetDeviceHardwareVersion(int& error) = 0;

		virtual long  GetDeviceHardwareVersion() = 0;

		virtual float GetTorqueReferenceIq(int& error) = 0;

		virtual float GetTorqueReferenceIq() = 0;

		virtual long  GetSpeedReference(int& error) = 0;

		virtual long  GetSpeedReference() = 0;

		virtual float GetMagnetizingCurrentIdReference(int& error) = 0;

		virtual float GetMagnetizingCurrentIdReference() = 0;

		virtual long  GetPositionReference(int& error) = 0;

		virtual long  GetPositionReference() = 0;

		virtual float GetPowerReference(int& error) = 0;

		virtual float GetPowerReference() = 0;

		virtual long  GetMotorDirection(int& error) = 0;

		virtual long  GetMotorDirection() = 0;

		virtual float GetObserverGainBldcPmsm(int& error) = 0;

		virtual float GetObserverGainBldcPmsm() = 0;

		virtual float GetObserverGainBldcPmsmUltrafast(int& error) = 0;

		virtual float GetObserverGainBldcPmsmUltrafast() = 0;

		virtual float GetObserverGainDc(int& error) = 0;

		virtual float GetObserverGainDc() = 0;

		virtual float GetFilterGainBldcPmsm(int& error) = 0;

		virtual float GetFilterGainBldcPmsm() = 0;

		virtual float GetFilterGainBldcPmsmUltrafast(int& error) = 0;

		virtual float GetFilterGainBldcPmsmUltrafast() = 0;

		virtual float Get3PhaseMotorAngle(int& error) = 0; // Read Estimated or Measured Rotor Angle

		virtual float Get3PhaseMotorAngle() = 0;

		virtual float GetEncoderHallCcwOffset(int& error) = 0;

		virtual float GetEncoderHallCcwOffset() = 0;

		virtual float GetEncoderHallCwOffset(int& error) = 0;

		virtual float GetEncoderHallCwOffset() = 0;

		virtual long  GetUartBaudrate(int& error) = 0;

		virtual long  GetUartBaudrate() = 0;

		virtual float GetSpeedAccelerationValue(int& error) = 0;

		virtual float GetSpeedAccelerationValue() = 0;

		virtual float GetSpeedDecelerationValue(int& error) = 0;

		virtual float GetSpeedDecelerationValue() = 0;

		virtual long  GetEncoderIndexCounts(int& error) = 0;

		virtual long  GetEncoderIndexCounts() = 0;

		virtual bool CommunicationIsWorking(int& error) = 0;

		virtual bool CommunicationIsWorking() = 0;

	};

#ifdef __cplusplus
} // __cplusplus defined.
#endif