/**
 *******************************************************************************
 * @file    SOLOMotorControllersSerial.h
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

#include "SOLOMotorControllers.h"
#include "SOLOMotorControllersUtils.h"

#ifdef __cplusplus
extern "C"
{
#endif

/** @defgroup Serial_Commands Serial Commands
  * @brief all serial command hex code
  * @{
  */
#define ReadData                            0x00 // 0x00000000
#define INITIATOR                           0xFF //0xFFFF
#define BroadcastAddress                    0xFF
#define ENDING                              0xFE
#define ERR									0xEE //0xEEEEEEEE
#define CRC                                 0x00
#define WriteDeviceAddres                   0x01
#define WriteCommandMode                    0x02
#define WriteCurrentLimit                   0x03
#define WriteTorqueReferenceIq              0x04
#define WriteSpeedReference                 0x05
#define WritePowerReference                 0x06
#define WriteMotorParametersIdentification  0x07
#define WriteDriveDisableEnable		     	0x08
#define WriteOutputPwmFrequencyKhz          0x09
#define WriteSpeedControllerKp              0x0A
#define WriteSpeedControllerKi              0x0B
#define WriteMotorDirection                 0x0C
#define WriteMotorResistance                0x0D
#define WriteMotorInductance                0x0E
#define WriteMotorPolesCounts               0x0F
#define WriteIncrementalEncoderLines        0x10
#define WriteSpeedLimit                     0x11
#define WriteResetDeviceAddress             0x12
#define WriteFeedbackControlMode            0x13
#define WriteResetFactory                   0x14
#define WriteMotorType                      0x15
#define WriteControlMode                    0x16
#define WriteCurrentControllerKp            0x17
#define WriteCurrentControllerKi            0x18
#define WriteMonitoringMode                 0x19
#define WriteMagnetizingCurrentIdReference  0x1A
#define WritePositionReference              0x1B
#define WritePositionControllerKp           0x1C
#define WritePositionControllerKi           0x1D
#define WriteResetPositionToZero            0x1F //Home
#define WriteOverwriteErrorRegister         0x20
#define WriteZSFTInjectionAmplitude       	0x21
#define WriteZSFTPolarityAmplitude  		0x22
#define WriteObserverGainDc                 0x23 
#define WriteZSFTInjectionFrequency       	0x24
#define WriteSensorlessTransitionSpeed	    0x25
#define WriteUartBaudrate                   0x26 //Set UART line baud-rate - 937500 / 115200 [ bits/s]
#define WriteSensorCalibration              0x27
#define WriteEncoderHallCcwOffset           0x28
#define WriteEncoderHallCwOffset            0x29
#define WriteSpeedAccelerationValue         0x2A
#define WriteSpeedDecelerationValue         0x2B
#define WriteCanbusBaudrate                 0x2C
#define WriteASRDC                          0x2D
#define WriteMotionProfileMode              0x30
#define WriteMotionProfileVariable1         0x31
#define WriteMotionProfileVariable2         0x32
#define WriteMotionProfileVariable3         0x33
#define WriteMotionProfileVariable4         0x34
#define WriteMotionProfileVariable5         0x35
#define WriteDigitalOutputsRegister         0x38
#define WriteRegenerationCurrentLimit      	0x39
#define WritePositionSensorDigitalFilterLevel	0x3A

#define ReadDeviceAddress                   0x81
#define ReadPhaseAVoltage                   0x82
#define ReadPhaseBVoltage                   0x83
#define ReadPhaseACurrent                   0x84
#define ReadPhaseBCurrent                   0x85
#define ReadBusVoltage                      0x86
#define ReadDcMotorCurrentIm                0x87
#define ReadDcMotorVoltageVm                0x88
#define ReadSpeedControllerKp               0x89
#define ReadSpeedControllerKi               0x8A
#define ReadOutputPwmFrequencyHz            0x8B
#define ReadCurrentLimit                    0x8C
#define ReadQuadratureCurrentIqFeedback     0x8D
#define ReadMagnetizingCurrentIdFeedback    0x8E //Magnetizing
#define ReadMotorPolesCounts                0x8F
#define ReadIncrementalEncoderLines         0x90
#define ReadCurrentControllerKp             0x91
#define ReadCurrentControllerKi             0x92
#define ReadBoardTemperature                0x93
#define ReadMotorResistance                 0x94
#define ReadMotorInductance                 0x95
#define ReadSpeedFeedback                   0x96
#define ReadMotorType                       0x97
#define ReadFeedbackControlMode             0x99
#define ReadCommandMode                     0x9A
#define ReadControlMode                     0x9B
#define ReadSpeedLimit                      0x9C
#define ReadPositionControllerKp            0x9D
#define ReadPositionControllerKi            0x9E
#define ReadPositionCountsFeedback          0xA0
#define ReadErrorRegister                   0xA1
#define ReadDeviceFirmwareVersion           0xA2
#define ReadDeviceHardwareVersion           0xA3
#define ReadTorqueReferenceIq               0xA4 // Read Torque /�Iq� Reference
#define ReadSpeedReference                  0xA5 // Read Speed Reference
#define ReadMagnetizingCurrentIdReference   0xA6 // Read Magnetizing Current / �Id� Reference
#define ReadPositionReference               0xA7
#define ReadPowerReference                  0xA8
#define ReadMotorDirection                  0xA9
#define ReadZSFTInjectionAmplitude       	0xAA
#define ReadZSFTPolarityAmplitude   		0xAB
#define ReadObserverGainDc                  0xAC
#define ReadZSFTInjectionFrequency        	0xAD
#define ReadSensorlessTransitionSpeed	   	0xAE
#define Read3PhaseMotorAngle                0xB0
#define ReadEncoderHallCcwOffset            0xB1
#define ReadEncoderHallCwOffset             0xB2
#define ReadUartBaudrate                    0xB3 // 0 / 1 ( 937500 / 115200 [bits/s] )
#define ReadSpeedAccelerationValue          0xB4
#define ReadSpeedDecelerationValue          0xB5
#define ReadEncoderIndexCounts              0xB8
#define ReadASRDC                           0xB7
#define ReadEncoderIndexCounts              0xB8
#define ReadMotionProfileMode               0xBB
#define ReadMotionProfileVariable1          0xBC
#define ReadMotionProfileVariable2          0xBD
#define ReadMotionProfileVariable3          0xBE
#define ReadMotionProfileVariable4          0xBF
#define ReadMotionProfileVariable5          0xC0
#define ReadPT1000SensorVoltage         	0xC3
#define ReadDigitalOutputRegister         	0xC4
#define ReadDigitalInputRegister         	0xC5
#define ReadAnalogueInput		         	0xC6
#define ReadDriveDisableEnable		      	0xC7
#define ReadRegenerationCurrentLimit      	0xC8
#define ReadPositionSensorDigitalFilterLevel	0x3A
/**
  * @}
  */

/**
 * @brief a class for handle serial communication
 * */
	class SOLOMotorControllersSerial : public SOLOMotorControllers {

	private:
		UINT8 addr = 0;
		char* portName;
		bool isConnected = false;
		HANDLE hSerial;
		UINT32 uartBaudrate;
		UINT32 timeout;
		UINT32 trialCount;

		char   ComPortName[10];
		BOOL   Status;
		DWORD  dNoOFBytestoWrite;              // No of bytes to write into the port
		DWORD  dNoOfBytesWritten = 0;          // No of bytes written to the port

		char  SerialBuffer[256];               // Buffer Containing Rxed Data
		DWORD NoBytesRecieved;
		char  TempChar;

		SOLOMotorControllersUtils* soloUtils;

	public:

		SOLOMotorControllersSerial(char* COMPortName, UINT8 deviceAddress = 0,
 				SOLOMotorControllers::UartBaudrate baudrate = SOLOMotorControllersSerial::UartBaudrate::rate115200,
 				long millisecondsTimeout = 5, int packetFailureTrialAttempts = 2, bool autoConnect = true);

		~SOLOMotorControllersSerial();

		static int lastError;

	private:

		bool ExeCMD(unsigned char* cmd, int& error = lastError);

		void SplitData(unsigned char* data, unsigned char* cmd);

	public:

		bool Connect(char* COMPortName, UINT8 deviceAddress = 0,
 				SOLOMotorControllers::UartBaudrate baudrate = SOLOMotorControllersSerial::UartBaudrate::rate115200,
 				long millisecondsTimeout = 50, int packetFailureTrialAttempts = 3);

		bool Connect();

		void Disconnect();

		bool Test();

    /** @addtogroup Serial_Write_Functions Standard Serial Write Functions
     * @{
     */
		bool SetDeviceAddress(unsigned char deviceAddress, int& error = lastError);

		bool SetCommandMode(CommandMode mode, int& error = lastError);

		bool SetCurrentLimit(float currentLimit, int& error = lastError);

		bool SetTorqueReferenceIq(float torqueReferenceIq, int& error = lastError);

		bool SetSpeedReference(long speedReference, int& error = lastError);

		bool SetPowerReference(float powerReference, int& error = lastError);

		bool MotorParametersIdentification(Action identification, int& error = lastError);

		bool SetDriveDisableEnable(DisableEnable action, int& error = lastError);

		bool SetOutputPwmFrequencyKhz(long outputPwmFrequencyKhz, int& error = lastError);

		bool SetSpeedControllerKp(float speedControllerKp, int& error = lastError);

		bool SetSpeedControllerKi(float speedControllerKi, int& error = lastError);

		bool SetMotorDirection(Direction motorDirection, int& error = lastError);

		bool SetMotorResistance(float motorResistance, int& error = lastError);

		bool SetMotorInductance(float motorInductance, int& error = lastError);

		bool SetMotorPolesCounts(long motorPolesCounts, int& error = lastError);

		bool SetIncrementalEncoderLines(long incrementalEncoderLines, int& error = lastError);

		bool SetSpeedLimit(long speedLimit, int& error = lastError);

		bool ResetDeviceAddress(int& error = lastError);

		bool SetFeedbackControlMode(FeedbackControlMode mode, int& error = lastError);

		bool ResetFactory(int& error = lastError);

		bool SetMotorType(MotorType motorType, int& error = lastError);

		bool SetControlMode(ControlMode controlMode, int& error = lastError);

		bool SetCurrentControllerKp(float currentControllerKp, int& error = lastError);

		bool SetCurrentControllerKi(float currentControllerKi, int& error = lastError);

		bool SetMonitoringMode(bool mode, int& error = lastError);

		bool SetMagnetizingCurrentIdReference(float magnetizingCurrentIdReference, int& error = lastError);

		bool SetPositionReference(long positionReference, int& error = lastError);

		bool SetPositionControllerKp(float positionControllerKp, int& error = lastError);

		bool SetPositionControllerKi(float positionControllerKi, int& error = lastError);

		bool ResetPositionToZero(int& error = lastError); //Home

		bool OverwriteErrorRegister(int& error = lastError);

		bool SetZsftInjectionAmplitude(float zsftInjectionAmplitude, int& error = lastError);

		bool SetZsftPolarityAmplitude(float zsftPolarityAmplitude, int& error = lastError);

		bool SetObserverGainDc(float observerGain, int& error = lastError);

		bool SetZsftInjectionFrequency(long zsftInjectionFrequency, int& error = lastError);

		bool SetSensorlessTransitionSpeed(long sensorlessTransitionSpeed, int& error = lastError);

		bool SetUartBaudrate(UartBaudrate baudrate, int& error = lastError);

		bool SensorCalibration(PositionSensorCalibrationAction calibrationAction, int& error = lastError);

		bool SetEncoderHallCcwOffset(float encoderHallOffset, int& error = lastError);

		bool SetEncoderHallCwOffset(float encoderHallOffset, int& error = lastError);

		bool SetSpeedAccelerationValue(float speedAccelerationValue, int& error = lastError);

		bool SetSpeedDecelerationValue(float speedDecelerationValue, int& error = lastError);

		bool SetCanbusBaudrate(CanbusBaudrate canbusBoudrate, int& error = lastError);
		
		bool SetAnalogueSpeedResolutionDivisionCoefficient(long divisionCoefficient, int &error);
				
        bool SetMotionProfileMode( MotionProfileMode motionProfileMode, int &error);
				
        bool SetMotionProfileVariable1(float MotionProfileVariable1, int &error);
				
        bool SetMotionProfileVariable2(float MotionProfileVariable2, int &error);
				
        bool SetMotionProfileVariable3(float MotionProfileVariable3, int &error);
				
        bool SetMotionProfileVariable4(float MotionProfileVariable4, int &error);
				
        bool SetMotionProfileVariable5(float MotionProfileVariable5, int &error);

		bool SetDigitalOutputState(Channel channel, DigitalIoState state, int &error);

		bool SetRegenerationCurrentLimit(float current, int &error);

		bool SetPositionSensorDigitalFilterLevel(long level, int &error);
		
	/**
     * @}
     */

    /** @addtogroup Serial_Read_Functions Standard Serial Read Functions
     * @{
     */
		////----------Read----------
		long  GetDeviceAddress(int& error = lastError);

		float GetPhaseAVoltage(int& error = lastError);

		float GetPhaseBVoltage(int& error = lastError);

		float GetPhaseACurrent(int& error = lastError);

		float GetPhaseBCurrent(int& error = lastError);

		float GetBusVoltage(int& error = lastError); //Battery Voltage

		float GetDcMotorCurrentIm(int& error = lastError);

		float GetDcMotorVoltageVm(int& error = lastError);

		float GetSpeedControllerKp(int& error = lastError);

		float GetSpeedControllerKi(int& error = lastError);

		long  GetOutputPwmFrequencyKhz(int& error = lastError);

		float GetCurrentLimit(int& error = lastError);

		float GetQuadratureCurrentIqFeedback(int& error = lastError);

		float GetMagnetizingCurrentIdFeedback(int& error = lastError); //Magnetizing

		long  GetMotorPolesCounts(int& error = lastError);

		long  GetIncrementalEncoderLines(int& error = lastError);

		float GetCurrentControllerKp(int& error = lastError);

		float GetCurrentControllerKi(int& error = lastError);

		float GetBoardTemperature(int& error = lastError);

		float GetMotorResistance(int& error = lastError);

		float GetMotorInductance(int& error = lastError);

		long  GetSpeedFeedback(int& error = lastError);

		MotorType  GetMotorType(int& error = lastError);

		FeedbackControlMode  GetFeedbackControlMode(int& error = lastError);

		CommandMode  GetCommandMode(int& error = lastError);

		ControlMode  GetControlMode(int& error = lastError);

		long  GetSpeedLimit(int& error = lastError);

		float GetPositionControllerKp(int& error = lastError);

		float GetPositionControllerKi(int& error = lastError);

		long  GetPositionCountsFeedback(int& error = lastError);

		long  GetErrorRegister(int& error = lastError);

		long  GetDeviceFirmwareVersion(int& error = lastError);

		long  GetDeviceHardwareVersion(int& error = lastError);

		float GetTorqueReferenceIq(int& error = lastError);

		long  GetSpeedReference(int& error = lastError);

		float GetMagnetizingCurrentIdReference(int& error = lastError);

		long  GetPositionReference(int& error = lastError);

		float GetPowerReference(int& error = lastError);

		Direction  GetMotorDirection(int& error = lastError);

		float GetZsftInjectionAmplitude(int& error = lastError);

		float GetZsftPolarityAmplitude(int& error = lastError);

		float GetObserverGainDc(int& error = lastError);

		long  GetZsftInjectionFrequency(int& error = lastError);

		long  GetSensorlessTransitionSpeed(int& error = lastError);

		float Get3PhaseMotorAngle(int& error = lastError); // Read Estimated or Measured Rotor Angle

		float GetEncoderHallCcwOffset(int& error = lastError);

		float GetEncoderHallCwOffset(int& error = lastError);

		UartBaudrate  GetUartBaudrate(int& error = lastError);
		
		long  GetAnalogueSpeedResolutionDivisionCoefficient(int &error);
		
		float GetSpeedAccelerationValue(int& error = lastError);

		float GetSpeedDecelerationValue(int& error = lastError);

		long  GetEncoderIndexCounts(int& error = lastError);

		bool CommunicationIsWorking(int& error = lastError);
		
		MotionProfileMode GetMotionProfileMode(int &error);
				
        float GetMotionProfileVariable1(int &error);
				
        float GetMotionProfileVariable2(int &error);
				
        float GetMotionProfileVariable3(int &error);
				
        float GetMotionProfileVariable4(int &error);
				
        float GetMotionProfileVariable5(int &error);

		DigitalIoState GetDigitalOutputState(Channel channel, int &error);

		long GetDigitalOutputsRegister(int &error);

		long GetPT1000SensorVoltage(int &error);
		
		DisableEnable GetDriveDisableEnable(int &error);

		float GetRegenerationCurrentLimit(int &error);

		long GetPositionSensorDigitalFilterLevel(int &error);

		long GetDigitalInputRegister(int &error);

		DigitalIoState GetAnalogueInput(Channel channel, int &error);
	/**
     * @}
     */

	};

#ifdef __cplusplus
} // __cplusplus defined.
#endif
