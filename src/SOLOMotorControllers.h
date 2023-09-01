/**
*******************************************************************************
* @file    SOLOMotorControllers.h
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

#include <Windows.h>
#include <tchar.h>
#include <stdio.h>

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
            Abort_Object = 6,						/*!< abort object */
            Abort_Value = 7,						/*!< abort value */
            MCP2515_Transmit_ArbitrationLost = 8,	/*!< MCP2515 transmit arbitration lost */
            MCP2515_Transmit_Error = 9,				/*!< MCP2515 transmit error */
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
            digital = 1								/*!< Digital Mode */
        };

    /**
     * @brief  Direction enumeration definition
     */
    enum Direction
        {
            clockwise = 0,							/*!< clockwise direction */
            counterclockwise = 1					/*!< counter-clockwise direction */
        };

    /**
     * @brief  Feedback Control Mode enumeration definition
     */
    enum FeedbackControlMode
        {
            sensorLess = 0,							/*!< sensorless mode */
            encoders = 1,							/*!< encoders mode */
            hallSensors = 2							/*!< hall sensors mode */
        };

    /**
     * @brief  Control Mode enumeration definition
     */
    enum ControlMode
        {
            speedMode = 0,							/*!< speed mode */
            torqueMode = 1,							/*!< torque mode */
            positionMode = 2						/*!< position mode */
        };

    /**
     * @brief  Motor Type enumeration definition
     */
    enum MotorType
        {
            dc = 0,									/*!< dc motor */
            bldcPmsm = 1,							/*!< brushless dc motor  */
            acim = 2,								/*!< acim motor */
            bldcPmsmUltrafast = 3					/*!< brushless dc motor fast */
        };
	
    /**
     * @brief  Uart Baudrate enumeration definition
     */
    enum UartBaudrate
        {
            rate937500 = 0,							/*!< baud rate 937500 */
            rate115200 = 1							/*!< baud rate 115200 */
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
            rate100 = 4								/*!< Baudrate 100 kbits/s */
        };

    /**
     * @brief  Action enumeration definition
     */
    enum Action
        {
            stop = 0,								/*!< stop */
            start = 1								/*!< start */
        };

    /**
     * @brief  Position Sensor Calibration Action enumeration definition
     */
    enum PositionSensorCalibrationAction
        {
            stopCalibration = 0,					/*!< stop colibration */
            incrementalEncoderStartCalibration = 1,	/*!< incremental encoder start calibration */
            hallSensorStartCalibration = 2			/*!< hall sensor start calibration */
        };
		
    /** 
     * @brief  Motion Profile Mode enumeration definition
     */
    enum MotionProfileMode
        {
            StepRampResponse = 0,       /*!< step ramp service */
            TimeBasedStcurve = 1,       /*!< time based st curve */
            TimeOptimalStcurve = 2	  	/*!< time optimal st curve */
        };

    virtual ~SOLOMotorControllers() = default;

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
		
    virtual bool SetAnalogueSpeedResolutionDivisionCoefficient(long divisionCoefficient, int &error) = 0;
		
    virtual bool SetAnalogueSpeedResolutionDivisionCoefficient(long divisionCoefficient) = 0;
		
    virtual bool SetMotionProfileMode( MotionProfileMode motionProfileMode, int &error) = 0;
		
    virtual bool SetMotionProfileMode( MotionProfileMode motionProfileMode) = 0;
		
    virtual bool SetMotionProfileVariable1(float MotionProfileVariable1, int &error) = 0;
		
    virtual bool SetMotionProfileVariable1(float MotionProfileVariable1) = 0;
		
    virtual bool SetMotionProfileVariable2(float MotionProfileVariable2, int &error) = 0;
		
    virtual bool SetMotionProfileVariable2(float MotionProfileVariable2) = 0;
		
    virtual bool SetMotionProfileVariable3(float MotionProfileVariable3, int &error) = 0;
		
    virtual bool SetMotionProfileVariable3(float MotionProfileVariable3) = 0;
		
    virtual bool SetMotionProfileVariable4(float MotionProfileVariable4, int &error) = 0;
		
    virtual bool SetMotionProfileVariable4(float MotionProfileVariable4) = 0;
		
    virtual bool SetMotionProfileVariable5(float MotionProfileVariable5, int &error) = 0;
		
    virtual bool SetMotionProfileVariable5(float MotionProfileVariable5) = 0;

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
		
    virtual long  GetAnalogueSpeedResolutionDivisionCoefficient(int &error) = 0;
		
    virtual long  GetAnalogueSpeedResolutionDivisionCoefficient() = 0;
		
    virtual long GetMotionProfileMode(int &error) = 0;
		
    virtual long GetMotionProfileMode() = 0;
		
    virtual float GetMotionProfileVariable1(int &error) = 0;
		
    virtual float GetMotionProfileVariable1() = 0;
		
    virtual float GetMotionProfileVariable2(int &error) = 0;
		
    virtual float GetMotionProfileVariable2() = 0;
		
    virtual float GetMotionProfileVariable3(int &error) = 0;
		
    virtual float GetMotionProfileVariable3() = 0;
		
    virtual float GetMotionProfileVariable4(int &error) = 0;
		
    virtual float GetMotionProfileVariable4() = 0;
		
    virtual float GetMotionProfileVariable5(int &error) = 0;
		
    virtual float GetMotionProfileVariable5() = 0;
};

