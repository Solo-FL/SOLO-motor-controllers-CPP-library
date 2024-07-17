/**
 *******************************************************************************
 * @file    SOLOMotorControllers.h
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

#include <Windows.h>
#include <tchar.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C"
{
#endif

	class SOLOMotorControllers {

	public:

    /**
	 * @brief  Error enumeration definition
	 */
		enum Error
		{
			noErrorDetected = 0,					/*!< no error detected */
			generalError = 1,						/*!< general error */
			noProcessedCommand = 2,					/*!< command is not valid */
			outOfRengeSetting = 3,					/*!< setting is out of range */
			packetFailureTrialAttemptsOverflow = 4,	/*!< trial attempt overflow */
			recieveTimeOutError = 5,				/*!< receive time error */
			abortObject = 6,						/*!< abort object */
			abortValue = 7,							/*!< abort value */
			mcp2515TransmitArbitrationLost = 8,		/*!< MCP2515 transmit arbitration lost */
			mcp2515TransmitError = 9,				/*!< MCP2515 transmit error */
			objectNotInitialize = 10,				/*!< Kvaser object not initialize */
			canEmptyBuffer = 11,					/*!< Kvaser buffer have no data for the defined COBID */
			pdoParameterIdOutOfRange = 12,			/*!< PDO configuration id out of range */
			pdoSyncOutOfRange = 13,					/*!< PDO configuration sync out of range */
			pdoMissingCobId = 14,					/*!< PDO no specific CobId for the specified pdo*/
			pdoRtrCommandNotAllowed = 15,			/*!< PDO RTR specific command not allowed*/
		};

    /**
	 * @brief  Command Mode enumeration definition
	 */
		enum CommandMode
		{
			analogue = 0,							/*!< Analogue Mode */
			digital = 1,							/*!< Digital Mode */
			analogueWithDigitalSpeedGain = 2,		/*!<Analogue Mode with Speed controller gains taken from the Digital setup*/
			commandModeError = -1					/*!< error */
		};

    /**
	 * @brief  Direction enumeration definition
	 */
		enum Direction
		{
			counterclockwise = 0,					/*!< counter-clockwise direction */
			clockwise = 1,							/*!< clockwise direction */
			directionError = -1						/*!< error */
		};

	/**
	 * @brief  Feedback Control Mode enumeration definition
	 */
		enum FeedbackControlMode
		{
			sensorLessHso = 0,						/*!< sensorless mode */
			encoders = 1,							/*!< encoders mode */
			hallSensors = 2,						/*!< hall sensors mode */
			sensorLessZsft = 3,						/*!< Sensor-less Zero Speed Full Torque feedback Mode (ZSFT) */
			feedbackControlModeError = -1			/*!< error */
		};

    /**
	 * @brief  Control Mode enumeration definition
	 */
		enum ControlMode
		{
			speedMode = 0,							/*!< speed mode */
			torqueMode = 1,							/*!< torque mode */
			positionMode = 2,						/*!< position mode */
			controlModeError = -1					/*!< error */
		};

    /**
	 * @brief  Motor Type enumeration definition
	 */
		enum MotorType
		{
			dc = 0,									/*!< dc motor */
			bldcPmsm = 1,							/*!< brushless dc motor  */
			acim = 2,								/*!< acim motor */
			bldcPmsmUltrafast = 3,					/*!< brushless dc motor fast */
			motorTypeError = -1						/*!< error */
		};
	
    /**
	 * @brief  Uart Baudrate enumeration definition
	 */
		enum UartBaudrate
		{
			rate937500 = 0,							/*!< baud rate 937500 */
			rate115200 = 1,							/*!< baud rate 115200 */
			uartBaudrateError = -1					/*!< error */
		};

    /**
	 * @brief  Canbus Baudrate enumeration definition
	 */
		enum CanbusBaudrate
		{
			rate1000 = 0,							/*!< Baudrate 1000 kbits/s */
			rate500 = 1,							/*!< Baudrate 500 kbits/s */
			rate250 = 2,							/*!< Baudrate 250 kbits/s */
			rate125 = 3,							/*!< Baudrate 125 kbits/s */
			rate100 = 4,							/*!< Baudrate 100 kbits/s */
			canbusBaudrateError = -1				/*!< error */
		};

    /**
	 * @brief  Action enumeration definition
	 */
		enum Action
		{
			stop = 0,								/*!< stop */
			start = 1,								/*!< start */
			actionError = -1						/*!< error */
		};

	/**
	 * @brief  Disable/Enable enumeration definition
	 */
		enum DisableEnable
		{
			disable = 0,								/*!< Disable */
			enable = 1,									/*!< Enable */
			disableEnableError = -1						/*!< error */
		};

    /**
	 * @brief  Position Sensor Calibration Action enumeration definition
	 */
		enum PositionSensorCalibrationAction
		{
			stopCalibration = 0,						/*!< stop colibration */
			incrementalEncoderStartCalibration = 1,		/*!< incremental encoder start calibration */
			hallSensorStartCalibration = 2,				/*!< hall sensor start calibration */
			positionSensorCalibrationActionError = -1	/*!< error */
		};
		
	/** 
	 * @brief  Motion Profile Mode enumeration definition
	 */
		enum MotionProfileMode
		{
			stepRampResponse = 0,       /*!< step ramp service */
			timeBasedStcurve = 1,       /*!< time based st curve */
			timeOptimalStcurve = 2,	  	/*!< time optimal st curve */
			motionProfileModeError = -1	/*!< error */
		};

	/** 
	 * @brief  Digital Input and Output State enumeration definition
	 */
		enum DigitalIoState
		{
			low = 0,       	/*!< GPIO Low State */
			high = 1,     	/*!< GPIO High State */
			digitalIoStateError = -1		/*!< error */
		};

	/** 
	 * @brief  Channel enumeration definition
	 */
		enum Channel
		{
			channel0 = 0,       	/*!< Channel 0 */
			channel1 = 1,       	/*!< Channel 1 */
			channel2 = 2,			/*!< Channel 2 */
			channel3 = 3,			/*!< Channel 3 */
			channelError = -1		/*!< error */
		};

		static int lastError;

		virtual bool Connect() = 0;

		virtual void Disconnect() = 0;

		//----------Write----------

		virtual bool SetDeviceAddress(unsigned char deviceAddress, int& error = lastError) = 0;

		virtual bool SetCommandMode(CommandMode mode, int& error = lastError) = 0;

		virtual bool SetCurrentLimit(float currentLimit, int& error = lastError) = 0;

		virtual bool SetTorqueReferenceIq(float torqueReferenceIq, int& error = lastError) = 0;

		virtual bool SetSpeedReference(long speedReference, int& error = lastError) = 0;

		virtual bool SetPowerReference(float powerReference, int& error = lastError) = 0;

		virtual bool MotorParametersIdentification(Action identification, int& error = lastError) = 0;

		virtual bool SetDriveDisableEnable(DisableEnable action, int& error = lastError) = 0;

		virtual bool SetOutputPwmFrequencyKhz(long outputPwmFrequencyKhz, int& error = lastError) = 0;

		virtual bool SetSpeedControllerKp(float speedControllerKp, int& error = lastError) = 0;

		virtual bool SetSpeedControllerKi(float speedControllerKi, int& error = lastError) = 0;

		virtual bool SetMotorDirection(Direction motorDirection, int& error = lastError) = 0;

		virtual bool SetMotorResistance(float motorResistance, int& error = lastError) = 0;

		virtual bool SetMotorInductance(float motorInductance, int& error = lastError) = 0;

		virtual bool SetMotorPolesCounts(long motorPolesCounts, int& error = lastError) = 0;

		virtual bool SetIncrementalEncoderLines(long incrementalEncoderLines, int& error = lastError) = 0;

		virtual bool SetSpeedLimit(long speedLimit, int& error = lastError) = 0;

		virtual bool ResetDeviceAddress(int& error = lastError) = 0;

		virtual bool ResetPositionToZero(int& error = lastError) = 0;

		virtual bool SetFeedbackControlMode(FeedbackControlMode mode, int& error = lastError) = 0;

		virtual bool ResetFactory(int& error = lastError) = 0;

		virtual bool SetMotorType(MotorType motorType, int& error = lastError) = 0;

		virtual bool SetControlMode(ControlMode controlMode, int& error = lastError) = 0;

		virtual bool SetCurrentControllerKp(float currentControllerKp, int& error = lastError) = 0;

		virtual bool SetCurrentControllerKi(float currentControllerKi, int& error = lastError) = 0;

		virtual bool SetMonitoringMode(bool mode, int& error = lastError) = 0;

		virtual bool SetMagnetizingCurrentIdReference(float magnetizingCurrentIdReference, int& error = lastError) = 0;

		virtual bool SetPositionReference(long positionReference, int& error = lastError) = 0;

		virtual bool SetPositionControllerKp(float positionControllerKp, int& error = lastError) = 0;

		virtual bool SetPositionControllerKi(float positionControllerKi, int& error = lastError) = 0;

		virtual bool OverwriteErrorRegister(int& error = lastError) = 0;

		virtual bool SetZsftInjectionAmplitude(float zsftInjectionAmplitude, int& error = lastError) = 0;

		virtual bool SetZsftPolarityAmplitude(float zsftPolarityAmplitude, int& error = lastError) = 0;

		virtual bool SetObserverGainDc(float observerGain, int& error = lastError) = 0;

		virtual bool SetZsftInjectionFrequency(long zsftInjectionFrequency, int& error = lastError) = 0;

		virtual bool SetSensorlessTransitionSpeed(long sensorlessTransitionSpeed, int& error = lastError) = 0;

		virtual bool SetUartBaudrate(UartBaudrate baudrate, int& error = lastError) = 0;

		virtual bool SensorCalibration(PositionSensorCalibrationAction calibrationAction, int& error = lastError) = 0;

		virtual bool SetEncoderHallCcwOffset(float encoderHallOffset, int& error = lastError) = 0;

		virtual bool SetEncoderHallCwOffset(float encoderHallOffset, int& error = lastError) = 0;

		virtual bool SetSpeedAccelerationValue(float speedAccelerationValue, int& error = lastError) = 0;

		virtual bool SetSpeedDecelerationValue(float speedDecelerationValue, int& error = lastError) = 0;

		virtual bool SetCanbusBaudrate(CanbusBaudrate canbusBoudrate, int& error = lastError) = 0;
		
		virtual bool SetAnalogueSpeedResolutionDivisionCoefficient(long divisionCoefficient, int& error = lastError) = 0;
				
		virtual bool SetMotionProfileMode( MotionProfileMode motionProfileMode, int& error = lastError) = 0;
				
		virtual bool SetMotionProfileVariable1(float MotionProfileVariable1, int& error = lastError) = 0;
				
		virtual bool SetMotionProfileVariable2(float MotionProfileVariable2, int& error = lastError) = 0;
				
		virtual bool SetMotionProfileVariable3(float MotionProfileVariable3, int& error = lastError) = 0;
				
		virtual bool SetMotionProfileVariable4(float MotionProfileVariable4, int& error = lastError) = 0;
				
		virtual bool SetMotionProfileVariable5(float MotionProfileVariable5, int& error = lastError) = 0;

		virtual bool SetDigitalOutputState(Channel channel, DigitalIoState state, int& error = lastError) = 0;

		virtual bool SetRegenerationCurrentLimit(float current, int& error = lastError) = 0;

		virtual bool SetPositionSensorDigitalFilterLevel(long level, int& error = lastError) = 0;
		
		////----------Read----------
		virtual long  GetDeviceAddress(int& error = lastError) = 0;

		virtual float GetPhaseAVoltage(int& error = lastError) = 0;

		virtual float GetPhaseBVoltage(int& error = lastError) = 0;

		virtual float GetPhaseACurrent(int& error = lastError) = 0;

		virtual float GetPhaseBCurrent(int& error = lastError) = 0;

		virtual float GetBusVoltage(int& error = lastError) = 0; //Battery Voltage

		virtual float GetDcMotorCurrentIm(int& error = lastError) = 0;

		virtual float GetDcMotorVoltageVm(int& error = lastError) = 0;

		virtual float GetSpeedControllerKp(int& error = lastError) = 0;

		virtual float GetSpeedControllerKi(int& error = lastError) = 0;

		virtual long  GetOutputPwmFrequencyKhz(int& error = lastError) = 0;

		virtual float GetCurrentLimit(int& error = lastError) = 0;

		virtual float GetQuadratureCurrentIqFeedback(int& error = lastError) = 0;

		virtual float GetMagnetizingCurrentIdFeedback(int& error = lastError) = 0; //Magnetizing

		virtual long  GetMotorPolesCounts(int& error = lastError) = 0;

		virtual long  GetIncrementalEncoderLines(int& error = lastError) = 0;

		virtual float GetCurrentControllerKp(int& error = lastError) = 0;

		virtual float GetCurrentControllerKi(int& error = lastError) = 0;

		virtual float GetBoardTemperature(int& error = lastError) = 0;

		virtual float GetMotorResistance(int& error = lastError) = 0;

		virtual float GetMotorInductance(int& error = lastError) = 0;

		virtual long  GetSpeedFeedback(int& error = lastError) = 0;

		virtual MotorType  GetMotorType(int& error = lastError) = 0;

		virtual FeedbackControlMode  GetFeedbackControlMode(int& error = lastError) = 0;

		virtual CommandMode  GetCommandMode(int& error = lastError) = 0;

		virtual ControlMode  GetControlMode(int& error = lastError) = 0;

		virtual long  GetSpeedLimit(int& error = lastError) = 0;

		virtual float GetPositionControllerKp(int& error = lastError) = 0;

		virtual float GetPositionControllerKi(int& error = lastError) = 0;

		virtual long  GetPositionCountsFeedback(int& error = lastError) = 0;

		virtual long  GetErrorRegister(int& error = lastError) = 0;

		virtual long  GetDeviceFirmwareVersion(int& error = lastError) = 0;

		virtual long  GetDeviceHardwareVersion(int& error = lastError) = 0;

		virtual float GetTorqueReferenceIq(int& error = lastError) = 0;

		virtual long  GetSpeedReference(int& error = lastError) = 0;

		virtual float GetMagnetizingCurrentIdReference(int& error = lastError) = 0;

		virtual long  GetPositionReference(int& error = lastError) = 0;

		virtual float GetPowerReference(int& error = lastError) = 0;

		virtual Direction  GetMotorDirection(int& error = lastError) = 0;

		virtual float GetZsftInjectionAmplitude(int& error = lastError) = 0;

		virtual float GetZsftPolarityAmplitude(int& error = lastError) = 0;

		virtual float GetObserverGainDc(int& error = lastError) = 0;

		virtual long GetZsftInjectionFrequency(int& error = lastError) = 0;

		virtual long GetSensorlessTransitionSpeed(int& error = lastError) = 0;

		virtual float Get3PhaseMotorAngle(int& error = lastError) = 0; // Read Estimated or Measured Rotor Angle

		virtual float GetEncoderHallCcwOffset(int& error = lastError) = 0;

		virtual float GetEncoderHallCwOffset(int& error = lastError) = 0;

		virtual UartBaudrate  GetUartBaudrate(int& error = lastError) = 0;

		virtual float GetSpeedAccelerationValue(int& error = lastError) = 0;

		virtual float GetSpeedDecelerationValue(int& error = lastError) = 0;

		virtual long  GetEncoderIndexCounts(int& error = lastError) = 0;

		virtual bool CommunicationIsWorking(int& error = lastError) = 0;
		
		virtual long  GetAnalogueSpeedResolutionDivisionCoefficient(int& error = lastError) = 0;
				
		virtual MotionProfileMode GetMotionProfileMode(int& error = lastError) = 0;
				
		virtual float GetMotionProfileVariable1(int& error = lastError) = 0;
				
		virtual float GetMotionProfileVariable2(int& error = lastError) = 0;
				
		virtual float GetMotionProfileVariable3(int& error = lastError) = 0;
				
		virtual float GetMotionProfileVariable4(int& error = lastError) = 0;
				
		virtual float GetMotionProfileVariable5(int& error = lastError) = 0;

		virtual DigitalIoState GetDigitalOutputState(Channel chaneel, int& error = lastError) = 0;
		
		virtual DisableEnable GetDriveDisableEnable(int& error = lastError) = 0;

		virtual float GetRegenerationCurrentLimit(int& error = lastError) = 0;

		virtual long GetPositionSensorDigitalFilterLevel(int& error = lastError) = 0;

		virtual long GetDigitalInputRegister(int& error = lastError) = 0;

		virtual long GetPT1000SensorVoltage(int& error = lastError) = 0;

		virtual DigitalIoState GetAnalogueInput(Channel channel, int& error = lastError) = 0;
	};

#ifdef __cplusplus
} // __cplusplus defined.
#endif