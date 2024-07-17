/**
 *******************************************************************************
 * @file    SOLOMotorControllersCanopenKvaser.cpp
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

#include "SOLOMotorControllersCanopenKvaser.h"

//DEBUG
// #include "stdio.h"
// #include <iostream>
// using std::cout;
// using std::endl;
// using std::hex; 

int SOLOMotorControllersCanopenKvaser::lastError = 0;

SOLOMotorControllersCanopenKvaser::SOLOMotorControllersCanopenKvaser(UINT8 deviceAddress, 
		SOLOMotorControllers::CanbusBaudrate baudrate, long millisecondsTimeout, bool autoConnect)
	:Address(deviceAddress)
	, timeout(millisecondsTimeout)
{
	if (Address == 0) //Address 0 is reserved for the host 
	{
		Address = 1;
	}

	switch (baudrate)
	{
	case SOLOMotorControllers::CanbusBaudrate::rate1000:
		canBaudrate = canBITRATE_1M;
		break;
	case SOLOMotorControllers::CanbusBaudrate::rate500:
		canBaudrate = canBITRATE_500K;
		break;
	case SOLOMotorControllers::CanbusBaudrate::rate250:
		canBaudrate = canBITRATE_250K;
		break;
	case SOLOMotorControllers::CanbusBaudrate::rate125:
		canBaudrate = canBITRATE_125K;
		break;
	case SOLOMotorControllers::CanbusBaudrate::rate100:
		canBaudrate = canBITRATE_100K;
		break;
	default:
		canBaudrate = canBITRATE_1M;
		break;
	}
	soloUtils = new SOLOMotorControllersUtils();
	kvaser = new Kvaser();
	InitPdoConfig();
	if(autoConnect)
	{
		SOLOMotorControllersCanopenKvaser::Connect();
	}	
}

SOLOMotorControllersCanopenKvaser::~SOLOMotorControllersCanopenKvaser()
{
	Disconnect();
}

bool SOLOMotorControllersCanopenKvaser::Connect(UINT8 deviceAddress, 
		SOLOMotorControllers::CanbusBaudrate baudrate, long millisecondsTimeout)
{
	Address = deviceAddress;
	switch (baudrate)
	{
	case SOLOMotorControllers::CanbusBaudrate::rate1000:
		canBaudrate = canBITRATE_1M;
		break;
	case SOLOMotorControllers::CanbusBaudrate::rate500:
		canBaudrate = canBITRATE_500K;
		break;
	case SOLOMotorControllers::CanbusBaudrate::rate250:
		canBaudrate = canBITRATE_250K;
		break;
	case SOLOMotorControllers::CanbusBaudrate::rate125:
		canBaudrate = canBITRATE_125K;
		break;
	case SOLOMotorControllers::CanbusBaudrate::rate100:
		canBaudrate = canBITRATE_100K;
		break;
	default:
		canBaudrate = canBITRATE_1M;
		break;
	}
	timeout = millisecondsTimeout;

	return SOLOMotorControllersCanopenKvaser::Connect();
}

bool SOLOMotorControllersCanopenKvaser::Connect()
{
	canStatus stat;
	canInitializeLibrary();
	hnd = canOpenChannel(0, 0);
	if (hnd < 0)
	{
		return false;
	}
	stat = canSetBusParams(hnd, canBaudrate, 0, 0, 0, 0, 0);
	if(stat == canOK){
		canBusOn(hnd);
		return true;
	}
	else
	{
		canClose(hnd);
		return false;
	}
}

void SOLOMotorControllersCanopenKvaser::Disconnect()
{
	canClose(hnd);
}

bool SOLOMotorControllersCanopenKvaser::SetGuardTime(long guardtime, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetGuardTimeInputValidation(guardtime,error))
    {
        return false;
    }
    soloUtils->ConvertToData(guardtime, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_GuardTime, 0x00, informatrionToSend, error);
}

bool SOLOMotorControllersCanopenKvaser::SetLifeTimeFactor(long lifeTimeFactor, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetLifeTimeFactorInputValidation(lifeTimeFactor,error))
    {
        return false;
    }
    soloUtils->ConvertToData(lifeTimeFactor, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_LifeTimeFactor, 0x00, informatrionToSend, error);
}

bool SOLOMotorControllersCanopenKvaser::SetProducerHeartbeatTime(long producerHeartbeatTime, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetProducerHeartbeatTimeInputValidation(producerHeartbeatTime,error))
    {
        return false;
    }
    soloUtils->ConvertToData(producerHeartbeatTime, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_ProducerHeartbeatTime, 0x00, informatrionToSend, error);
}

/**
  * @brief  This command determine the validity of count of SYNC message
  * @param[in]  parameterName	enum that specifies the name of the PDO parameter that wants to set CobId value
  * @param[in]  parameterCobbId	CobId value
  * @param[out]  error   pointer to an integer that specifies the result of the function       
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetPdoParameterCobbIdInputValidation(PdoParameterName parameterName, int parameterCobbId, int &error)
{
	if(parameterName < PdoParameterName::positionCountsFeedback)
	{
		if(parameterCobbId >= RPDO_MIN_COBIB && parameterCobbId <= RPDO_MAX_COBIB){
			return true;
		}
	}
	else 
	{
		if(parameterCobbId >= TPDO_MIN_COBIB && parameterCobbId <= TPDO_MAX_COBIB){
			return true;
		}
	}
	error = Error::pdoParameterIdOutOfRange;
	return false;
}

/**
  * @brief  This command determine the validity of count of SYNC message
  * @param[in]  parameterCount	count of SYNC message 
  * @param[out]  error   pointer to an integer that specifies the result of the function       
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetSyncParameterCountInputValidation(uint8_t parameterCount, int &error)
{
	if((parameterCount >= 0 && parameterCount < 12) || parameterCount == 0xFF)
		return true;
	error = Error::pdoSyncOutOfRange;
	return 0;
}

/**
  * @brief  This command set PDO configs for the intended PDO object 
  * @param[in]  config	enum that specifies PDO parameter configs for the intended PDO object
  * @param[out]  error   pointer to an integer that specifies the result of the function       
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetPdoParameterConfig(PdoParameterConfig config, int &error)
{
	//std::cout << "SetPdoParameterConfiguration \n";
	uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
	error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!SetPdoParameterCobbIdInputValidation(config.parameterName, config.parameterCobId, error))
    {
		error = SOLOMotorControllers::Error::pdoParameterIdOutOfRange;
		//std::cout << "SetPdoParameterConfiguration - Error SetPdoParameterCobbIdInputValidation \n";
        return false;
    }
	if(!SetSyncParameterCountInputValidation(config.syncParameterCount,error))
    {
		error = SOLOMotorControllers::Error::pdoSyncOutOfRange;
		//std::cout << "SetPdoParameterConfiguration - Error SetSyncParameterCountInputValidation \n";
        return false;
    }

	informatrionToSend[0] = (config.isPdoParameterEnable << 7) | (config.isRrtParameterEnable << 6);
	informatrionToSend[1] = 0;
	informatrionToSend[2] = config.parameterCobId >> 8;
	informatrionToSend[3] = config.parameterCobId % 256;
	boolean isSuccess = kvaser->CANOpenTransmit(hnd, Address, pdoParameterObjectByPdoParameterName[config.parameterName], 0x01, informatrionToSend, error);
	//std::cout << "SetPdoParameterConfiguration [1] return: "<< (int)isSuccess <<" error: "<< error<< " \n";
	if(isSuccess)
	{
		//std::cout << "SetPdoParameterConfiguration parameterName: "<<config.parameterName<< " ID: "<< config.parameterCobId <<"\n";
		pdoParameterCobIdByPdoParameterName[config.parameterName] = config.parameterCobId;
		
		informatrionToSend[0] = 0;
		informatrionToSend[1] = 0;
		informatrionToSend[2] = 0;
		informatrionToSend[3] = config.syncParameterCount;
		if(config.syncParameterCount == 0 && pdoParameterObjectByPdoParameterName[config.parameterName] < pdoParameterObjectByPdoParameterName[PdoParameterName::positionCountsFeedback]){
			informatrionToSend[3] = 0xFF;
		}
		
		isSuccess = kvaser->CANOpenTransmit(hnd, Address, pdoParameterObjectByPdoParameterName[config.parameterName], 0x02, informatrionToSend, error);
		//std::cout << "SetPdoParameterConfiguration [2] return: "<< (int)isSuccess <<" error: "<< error<< " \n";
		
		return isSuccess;
	}
	return false;
}

/**
  * @brief  This command gets PDO configs for the intended PDO object 
  * @param[in]  parameterName	enum that specifies the name of the PDO parameter that wants to get its config
  * @param[out]  error   pointer to an integer that specifies the result of the function       
  * @retval PdoParameterConfig enum @ref PdoParameterConfig
  */
PdoParameterConfig SOLOMotorControllersCanopenKvaser::GetPdoParameterConfig(PdoParameterName parameterName, int &error)
{
	PdoParameterConfig config;
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, parameterName, 0x1, informationToSend, informationReceived, error))
	{
		config.parameterName = parameterName;
		config.parameterCobId = (informationReceived[2] << 8) + informationReceived[3];
		config.isPdoParameterEnable = informationReceived[0] & 0x80;
		config.isRrtParameterEnable = informationReceived[0] & 0x40;

		if (kvaser->CANOpenReceive(hnd, Address, parameterName, 0x2, informationToSend, informationReceived, error))
		{
			config.syncParameterCount = informationReceived[3];
			return config;
		}
		else
		{
			config.parameterCobId = 0;
			config.isPdoParameterEnable = 0;
			config.isRrtParameterEnable = 0;
			config.syncParameterCount = 0;
			error = SOLOMotorControllers::Error::generalError;
			return config;
		}
	}
	error = SOLOMotorControllers::Error::generalError;
	return config;
}

/**
  * @brief  This command send a SYNC message on bus
  * @param[out]  error   pointer to an integer that specifies the result of the function       
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SendPdoSync(int &error)
{
	return kvaser->SendPdoSync(hnd, error);
}

/**
  * @brief  This command send a RTR for the intended PDO object 
  * @param[in]  parameterName	enum that specifies the name of the PDO parameter that wants to send RTR
  * @param[out]  error   pointer to an integer that specifies the result of the function       
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SendPdoRtr(PdoParameterName parameterName, int &error)
{
	SOLOMotorControllersCanopenKvaser::PdoRtrValidParameter(parameterName, error);
	if(error != SOLOMotorControllers::Error::noErrorDetected){
		return false;
	}

	int adr = GetPdoParameterCobId(parameterName, error);
	if(error != SOLOMotorControllers::Error::noErrorDetected){
		return false;
	}

	return kvaser->SendPdoRtr(hnd, adr, error);
}

/**
  * @brief  This command checks the validity of allowing RTR  for the intended PDO parameter
  * @param[in]  parameterName	enum that specifies the name of the PDO parameter that wants to check RTR validity
  * @param[out]  error   pointer to an integer that specifies the result of the function       
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::PdoRtrValidParameter(PdoParameterName parameterName, int &error)
{
	if(parameterName >= PdoParameterName::positionCountsFeedback ){
		error = SOLOMotorControllers::Error::noErrorDetected;
		return true;
	}
	error = SOLOMotorControllers::Error::pdoRtrCommandNotAllowed;
	return false;
}

/**
  * @brief  This command returns the CobId value for the intended PDO parameter name
  * @param[in]  parameterName	enum that specifies the name of the PDO parameter that wants to get parameter CobId
  * @param[out]  error   pointer to an integer that specifies the result of the function       
  * @retval long
  */
long SOLOMotorControllersCanopenKvaser::GetPdoParameterCobId(PdoParameterName parameterName, int &error)
{
	int pdoParameterCobId =  pdoParameterCobIdByPdoParameterName[parameterName];
	if(pdoParameterCobId==0)
	{
		error = SOLOMotorControllers::Error::pdoMissingCobId;
	}
	//std::cout << "parameterName: "<< parameterName << " CobId: "<< pdoParameterCobId <<"\n"; 
	return pdoParameterCobId;
}

/**
  * @brief  This command initializes all PDO parameter names addresses in @ref pdoParameterObjectByPdoParameterName array    
  * @retval void
  */
void SOLOMotorControllersCanopenKvaser::InitPdoConfig()
{
	for(int i = 0; i < PdoParameterNameCount; i++){
		pdoParameterCobIdByPdoParameterName[i] = 0;
	}
	
	pdoParameterObjectByPdoParameterName[PdoParameterName::positionReference] = 0x1414;
	pdoParameterObjectByPdoParameterName[PdoParameterName::speedReference] = 0x1415;
	pdoParameterObjectByPdoParameterName[PdoParameterName::torqueReferenceIq] = 0x1416;
	pdoParameterObjectByPdoParameterName[PdoParameterName::magnetizingCurrentIdReference] = 0x1417;
	pdoParameterObjectByPdoParameterName[PdoParameterName::controlMode] = 0x1418;
	pdoParameterObjectByPdoParameterName[PdoParameterName::motorDirection] = 0x1419;
	pdoParameterObjectByPdoParameterName[PdoParameterName::positionCountsFeedback] = 0x1814;
	pdoParameterObjectByPdoParameterName[PdoParameterName::speedFeedback] = 0x1815;
	pdoParameterObjectByPdoParameterName[PdoParameterName::quadratureCurrentIqFeedback] = 0x1816;
	pdoParameterObjectByPdoParameterName[PdoParameterName::magnetizingCurrentIdFeedback] = 0x1817;
	pdoParameterObjectByPdoParameterName[PdoParameterName::errorRegister] = 0x1818;
	pdoParameterObjectByPdoParameterName[PdoParameterName::boardTemperature] = 0x1819;
}

/**
  * @brief  This command update all CobId values for PDO parameters       
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::UpdatePdoParameterCobIdByPdoParameterName()
{
	int error;
	for(int i = 0; i < PdoParameterNameCount; i++){
		pdoParameterCobIdByPdoParameterName[i] = GetPdoParameterConfig((PdoParameterName)pdoParameterObjectByPdoParameterName[i], error).parameterCobId;
	}
	return true;
}

/**
  * @brief  This command set the intended long value for a PDO command
  * @param[in]  parameterName	enum that specifies the name of the PDO parameter that wants to write its value
  * @param[in]  value	the value that wants to be set for the PDO parameter 
  * @param[out]  error   pointer to an integer that specifies the result of the function       
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetPdoParameterValue(PdoParameterName parameterName, long value,
		int &error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	soloUtils->ConvertToData(value, informatrionToSend);

	int adr = GetPdoParameterCobId(parameterName, error);
	if(error == SOLOMotorControllers::Error::pdoMissingCobId){
		return false;
	}

	return kvaser->PDOTransmit(hnd, adr, informatrionToSend, error);
}

/**
  * @brief  This command set the intended float value for a PDO command
  * @param[in]  parameterName	enum that specifies the name of the PDO parameter that wants to write its value
  * @param[in]  value	float value that wants to be set for the PDO parameter 
  * @param[out]  error   pointer to an integer that specifies the result of the function       
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetPdoParameterValue(PdoParameterName parameterName, float value,
		int &error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	soloUtils->ConvertToData(value, informatrionToSend);

	int adr = GetPdoParameterCobId(parameterName, error);
	if(error == SOLOMotorControllers::Error::pdoMissingCobId){
		return false;
	}

	return kvaser->PDOTransmit(hnd, adr, informatrionToSend, error);
}

/**
  * @brief  This command returns the long value of a PDO command
  * @param[in]  parameterName	enum that specifies the name of the parameter that wants to read its value
  * @param[out]  error   pointer to an integer that specifies the result of the function       
  * @retval long
  */
long SOLOMotorControllersCanopenKvaser::GetPdoParameterValueLong(PdoParameterName parameterName,
		int &error)
{
	uint8_t informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noErrorDetected;

	int adr = GetPdoParameterCobId(parameterName, error);
	if(error != SOLOMotorControllers::Error::noErrorDetected){
		return false;
	}

	if (kvaser->PDOReceive(hnd, adr,informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}

/**
  * @brief  This command returns the float value of a PDO command
  * @param[in]  parameterName	enum that specifies the name of the PDO parameter that wants to read its value
  * @param[out]  error   pointer to an integer that specifies the result of the function      
  * @retval float
  */
float SOLOMotorControllersCanopenKvaser::GetPdoParameterValueFloat(PdoParameterName parameterName,
		int &error)
{
	uint8_t informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noErrorDetected;

	int adr = GetPdoParameterCobId(parameterName, error);
	if(error != SOLOMotorControllers::Error::noErrorDetected){
		return false;
	}

	if (kvaser->PDOReceive(hnd, adr,informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1;
}

/**
  * @brief  This command sets the desired device address for a SOLO unit
  *				.The method refers to the Object Dictionary: 0x3001
  * @param[in]  deviceAddress  address want to set for board
  * @param[out]  error   pointer to an integer that specify result of function       
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetDeviceAddress(unsigned char deviceAddress, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetDeviceAddressInputValidation(deviceAddress, error))
	{
		return false;
	}
	soloUtils->ConvertToData((long)deviceAddress, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_SetDeviceAddress, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the mode of the operation of SOLO
  *         in terms of operating in Analogue mode or Digital
  *				.The method refers to the Object Dictionary: 0x3002
  * @param[in] mode  enum that specify mode of the operation of SOLO  
  * @param[out]  error   pointer to an integer that specify result of function       
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetCommandMode(SOLOMotorControllers::CommandMode mode, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	soloUtils->ConvertToData((long)mode, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_CommandMode, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command defines the maximum allowed current into the motor in terms of Amps
  *				.The method refers to the Object Dictionary: 0x3003
  * @param[in] currentLimit  a float value between 0 to 32
  * @param[out]  error   pointer to an integer that specify result of function       
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetCurrentLimit(float currentLimit, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetCurrentLimitInputValidation(currentLimit, error))
	{
		return false;
	}
	soloUtils->ConvertToData(currentLimit, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_CurrentLimit, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the amount of desired current that acts in torque generation
  *				.The method refers to the Object Dictionary: 0x3004
  * @param[in] torqueReferenceIq  a float value between 0 to 32
  * @param[out]  error   pointer to an integer that specify result of function       
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetTorqueReferenceIq(float torqueReferenceIq, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetTorqueReferenceIqInputValidation(torqueReferenceIq, error))
	{
		return false;
	}
	soloUtils->ConvertToData(torqueReferenceIq, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_TorqueReferenceIq, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command defines the speed reference for SOLO once it’s in Digital Speed Mode
				.The method refers to the Object Dictionary: 0x3005
  * @param[in] speedReference  a long value between 0 to 30000
  * @param[out]  error   pointer to an integer that specify result of function       
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetSpeedReference(long speedReference, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetSpeedReferenceInputValidation(speedReference, error))
	{
		return false;
	}
	soloUtils->ConvertToData(speedReference, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_SpeedReference, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command defines the amount of power percentage during only
  *         Open-loop mode for 3-phase motors
				.The method refers to the Object Dictionary: 0x3006
  * @param[in] powerReference  a float value between 0 to 100
  * @param[out]  error   pointer to an integer that specify result of function       
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetPowerReference(float powerReference, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetPowerReferenceInputValidation(powerReference, error))
	{
		return false;
	}
	soloUtils->ConvertToData(powerReference, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_PowerReference, 0x00, informatrionToSend, error);

}

/**
  * @brief  By putting 1 in the DATA section of a packet sent with this command, SOLO will start
            identifying the electrical parameters of the Motor connected
				.The method refers to the Object Dictionary: 0x3007
  * @param[in] powerReference  enum that specify Start or Stop of something in SOLO 
  * @param[out]  error   pointer to an integer that specify result of function       
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::MotorParametersIdentification(SOLOMotorControllers::Action identification, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	soloUtils->ConvertToData((long)identification, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_MotorParametersIdentification, 0x00, informatrionToSend, error);
}

/**
  * @brief  This command Disables or Enables the Controller resulting in deactivation or activation of the
  *            	switching at the output, by disabling the drive, the effect of the Controller on the Motor will be
  *           	almost eliminated ( except for body diodes of the Mosfets) allowing freewheeling
  *            	.The method refers to the Uart Write command: 0x3008
  * @param[in]  action  enum that specify Disable or Enable of something in SOLO  
  * @param[out]  error   pointer to an integer that specify result of function       
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetDriveDisableEnable(DisableEnable action, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	soloUtils->ConvertToData((long)action, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_DriveDisableEnable, 0x00, informatrionToSend, error);
}

/**
  * @brief  This command sets the output switching frequency of the whole power unit on the Motor
				.The method refers to the Object Dictionary: 0x3009
  * @param[in] outputPwmFrequencyKhz  switching frequencies in kHz. a long value between 8 to 80 
  * @param[out]  error   pointer to an integer that specify result of function     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetOutputPwmFrequencyKhz(long outputPwmFrequencyKhz, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetOutputPwmFrequencyKhzInputValidation(outputPwmFrequencyKhz, error))
	{
		return false;
	}
	soloUtils->ConvertToData(outputPwmFrequencyKhz, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_OutputPwmFrequency, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the Speed controller Kp Gain, and it will
  *         be functional only in Digital Closed-loop mode
				.The method refers to the Object Dictionary: 0x300A
  * @param[in] speedControllerKp  a float value between 0 to 300 
  * @param[out]  error   pointer to an integer that specify result of function     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetSpeedControllerKp(float speedControllerKp, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetSpeedControllerKpInputValidation(speedControllerKp, error))
	{
		return false;
	}
	soloUtils->ConvertToData(speedControllerKp, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_SpeedControllerKp, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the Speed controller Ki gain, and it will
  *         be functional only in Digital Closed-loop mode
				.The method refers to the Object Dictionary: 0x300B
  * @param[in] speedControllerKi  a float value between 0 to 300 
  * @param[out]  error   pointer to an integer that specify result of function     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetSpeedControllerKi(float speedControllerKi, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetSpeedControllerKiInputValidation(speedControllerKi, error))
	{
		return false;
	}
	soloUtils->ConvertToData(speedControllerKi, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_SpeedControllerKi, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the direction of the rotation of the motor
  *         either to ClockWise rotation or to Counter Clockwise Rotation
				.The method refers to the Object Dictionary: 0x300C
  * @param[in] motorDirection  enum that specify the direction of the rotation of the motor 
  * @param[out]  error   pointer to an integer that specify result of function     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetMotorDirection(SOLOMotorControllers::Direction motorDirection, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	soloUtils->ConvertToData((long)motorDirection, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_MotorDirection, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the amount of the Phase or Armature resistance
  *         for 3-phase or DC Brushed motors respectively
				.The method refers to the Object Dictionary: 0x300D
  * @param[in] motorResistance  a float value [Ohm] between 0.0001 to 25.0
  * @param[out]  error   pointer to an integer that specify result of function     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetMotorResistance(float motorResistance, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetMotorResistanceInputValidation(motorResistance, error))
	{
		return false;
	}
	soloUtils->ConvertToData(motorResistance, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_MotorResistance, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the amount of the Phase or Armature Inductance
  *         for 3-phase or DC Brushed motors respectively
  *				.The method refers to the Object Dictionary: 0x300E
  * @param[in] motorInductance  a float value [Henry] between 0.0000001 to 0.5
  * @param[out]  error   pointer to an integer that specify result of function     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetMotorInductance(float motorInductance, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetMotorInductanceInputValidation(motorInductance, error))
	{
		return false;
	}
	soloUtils->ConvertToData(motorInductance, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_MotorInductance, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the number of the Poles of a 3-phase motor commissioned with SOLO
				.The method refers to the Object Dictionary: 0x300F
  * @param[in] motorPolesCounts  a long value between 1 to 254   
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetMotorPolesCounts(long motorPolesCounts, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetMotorPolesCountsInputValidation(motorPolesCounts, error))
	{
		return false;
	}

	soloUtils->ConvertToData(motorPolesCounts, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_MotorPolesCounts, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the pre-quad number of physical lines of an 
  *         incremental encoder engraved on its disk
				.The method refers to the Object Dictionary: 0x3010
  * @param[in] incrementalEncoderLines  a long value between 1 to 200000   
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetIncrementalEncoderLines(long incrementalEncoderLines, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetIncrementalEncoderLinesInputValidation(incrementalEncoderLines, error))
	{
		return false;
	}
	soloUtils->ConvertToData(incrementalEncoderLines, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_IncrementalEncoderLines, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the allowed speed during trajectory following
  *         in closed-loop position controlling mode
				.The method refers to the Object Dictionary: 0x3011
  * @param[in] speedLimit  a long value between 0 to 30000   
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetSpeedLimit(long speedLimit, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetSpeedLimitInputValidation(speedLimit, error))
	{
		return false;
	}
	soloUtils->ConvertToData(speedLimit, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_SpeedLimit, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the type of the feedback control SOLO has to operate
				.The method refers to the Object Dictionary: 0x3013
  * @param[in] mode  enum that specify the type of the feedback control SOLO 
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetFeedbackControlMode(SOLOMotorControllers::FeedbackControlMode mode, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	soloUtils->ConvertToData((long)mode, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_FeedbackControlMode, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command resets SOLO to its factory setting to all the default parameters  
				.The method refers to the Object Dictionary: 0x3014
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::ResetFactory(int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x01 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	return kvaser->CANOpenTransmit(hnd, Address, Object_ResetFactory, 0x00, informatrionToSend, error);

}

bool SOLOMotorControllersCanopenKvaser::ResetDeviceAddress(int& error)
{
	return false;
}

/**
  * @brief  This command resets the position counter back to zero if in the DATA part of the packet the
  *				value of “0x00000001” is placed and sent  
  *				.The method refers to the Object Dictionary: 0x301F
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::ResetPositionToZero(int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x01 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	return kvaser->CANOpenTransmit(hnd, Address, Object_ResetPositionToZero, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the Motor type that is connected to SOLO in Digital Mode
				.The method refers to the Object Dictionary: 0x3015
  * @param[in] motorType  enum that specify the Motor type that is connected to SOLO in Digital Mode
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetMotorType(SOLOMotorControllers::MotorType motorType, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	soloUtils->ConvertToData((long)motorType, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_MotorType, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the Control Mode in terms of Torque,
  *         Speed or Position only in Digital Mode
				.The method refers to the Object Dictionary: 0x3016
  * @param[in] controlMode  enum that specify the Control Mode in terms of Torque,
  *                       Speed or Position only in Digital Mode
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetControlMode(SOLOMotorControllers::ControlMode controlMode, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	soloUtils->ConvertToData((long)controlMode, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_ControlMode, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the value for Current Controller Kp or proportional gain
				.The method refers to the Object Dictionary: 0x3017
  * @param[in] currentControllerKp  a float value between 0 to 16000  
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetCurrentControllerKp(float currentControllerKp, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetCurrentControllerKpInputValidation(currentControllerKp, error))
	{
		return false;
	}
	soloUtils->ConvertToData(currentControllerKp, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_CurrentControllerKp, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the value for Current Controller Ki or integral gain
				.The method refers to the Object Dictionary: 0x3018
  * @param[in] motorInductance  a float value between 0 to 16000  
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetCurrentControllerKi(float currentControllerKi, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetCurrentControllerKiInputValidation(currentControllerKi, error))
	{
		return false;
	}
	soloUtils->ConvertToData(currentControllerKi, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_CurrentControllerKi, 0x00, informatrionToSend, error);

}

bool SOLOMotorControllersCanopenKvaser::SetMonitoringMode(bool mode, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	return false;
}

/**
  * @brief  depending on the Motor type: in case of BLDC or PMSM motors Sets the Field
  *         Weakening current reference to help the motor reaching speeds higher than
  *         nominal values and in case of AC Induction Motors Sets the desired magnetizing
  *         current (Id) required for controlling ACIM motors in FOC in Amps 
				.The method refers to the Object Dictionary: 0x301A
  * @param[in] magnetizingCurrentIdReference  a float value between 0 to 32   
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetMagnetizingCurrentIdReference(float magnetizingCurrentIdReference, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetMagnetizingCurrentIdReferenceInputValidation(magnetizingCurrentIdReference, error))
	{
		return false;
	}
	soloUtils->ConvertToData(magnetizingCurrentIdReference, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_MagnetizingCurrentIdReference, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the desired Position reference in terms of quadrature
  *         pulses while SOLO operates with the Incremental Encoders or in terms of
  *         pulses while while SOLO operates with Hall sensors
				.The method refers to the Object Dictionary: 0x301B
  * @param[in] positionReference  a long value between -2,147,483,647 to 2,147,483,647   
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetPositionReference(long positionReference, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetPositionReferenceInputValidation(positionReference, error))
	{
		return false;
	}
	soloUtils->ConvertToData(positionReference, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_PositionReference, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the value for Position Controller Kp or proportional gain
				.The method refers to the Object Dictionary: 0x301C
  * @param[in] positionControllerKp  a float value between 0 to 16000  
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetPositionControllerKp(float positionControllerKp, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetPositionControllerKpInputValidation(positionControllerKp, error))
	{
		return false;
	}
	soloUtils->ConvertToData(positionControllerKp, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_PositionControllerKp, 0x00, informatrionToSend, error);
}

/**
  * @brief  This command sets the value for Position Controller Ki or integrator gain
				.The method refers to the Object Dictionary: 0x301D
  * @param[in] positionControllerKi  a float value between 0 to 16000   
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetPositionControllerKi(float positionControllerKi, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetPositionControllerKiInputValidation(positionControllerKi, error))
	{
		return false;
	}
	soloUtils->ConvertToData(positionControllerKi, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_PositionControllerKi, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command overwrites the reported errors in Error Register
  *         reported with command code of "0xA1"  
				.The method refers to the Object Dictionary: 0x3020
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::OverwriteErrorRegister(int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	return kvaser->CANOpenTransmit(hnd, Address, Object_OverwriteErrorRegister, 0x00, informatrionToSend, error);

}

/**
  * @brief  Once in Zero Speed Full Torque algorithm (ZSFT) for controlling the speed of a BLDC or PMSM
              in sensorless fashion, this parameter defines the strength of signal injection into the motor, the
              user has to make sure this value is not selected too high or too low
				.The method refers to the Object Dictionary: 0x3021
  * @param[in] amplitude  a float value between 0.0 to 0.55  
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetZsftInjectionAmplitude(float zsftInjectionAmplitude, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetZsftInjectionAmplitudeValidation(zsftInjectionAmplitude, error))
	{
		return false;
	}
	soloUtils->ConvertToData(zsftInjectionAmplitude, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_ZsftInjectionAmplitude, 0x00, informatrionToSend, error);

}

/**
  * @brief  Once in Zero Speed Full Torque algorithm (ZSFT) for controlling the speed of a BLDC or PMSM
  *             in sensorless fashion, this parameter defines the strength of signal injection into the motor to
  *            identify the polarity of the Motor at the startup
  *				.The method refers to the Object Dictionary: 0x3022
  * @param[in] amplitude  a float value between 0.0 to 0.55   
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetZsftPolarityAmplitude(float zsftPolarityAmplitude, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetZsftPolarityAmplitudeValidation(zsftPolarityAmplitude, error))
	{
		return false;
	}
	soloUtils->ConvertToData(zsftPolarityAmplitude, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_ZsftPolarityAmplitude, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the observer gain for the Non-linear observer
  *         that estimates the speed of a DC brushed once the motor type 
  *         is selected as DC brushed
				.The method refers to the Object Dictionary: 0x3023
  * @param[in] observerGain  a float value between 0.01 to 1000   
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetObserverGainDc(float observerGain, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetObserverGainDcInputValidation(observerGain, error))
	{
		return false;
	}
	soloUtils->ConvertToData(observerGain, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_ObserverGainDc, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command defines the frequency of signal injection into the Motor in
				runtime, by selecting zero the full injection frequency will be applied which allows to reach to
				higher speeds, however for some motors, it’s better to increase this value
				.The method refers to the Object Dictionary: 0x3024
  * @param[in] filterGain  a long value between 0 to 10 
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetZsftInjectionFrequency(long zsftInjectionFrequency, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetZsftInjectionFrequencyInputValidation(zsftInjectionFrequency, error))
	{
		return false;
	}
	soloUtils->ConvertToData(zsftInjectionFrequency, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_ZsftInjectionFrequency, 0x00, informatrionToSend, error);

}

/**
  * @brief  Once in Sensorless speed or torque controlling of a BLDC or PMSM motors, this parameter
  *				defines the speed in which the Low speed algorithm has to switch to high speed algorithm
  *				.The method refers to the Object Dictionary: 0x3025
  * @param[in] speed  a long value between 1 to 5000  
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetSensorlessTransitionSpeed(long sensorlessTransitionSpeed, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetSensorlessTransitionSpeedInputValidation(sensorlessTransitionSpeed, error))
	{
		return false;
	}
	soloUtils->ConvertToData(sensorlessTransitionSpeed, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_SensorlessTransactionSpeed, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the baud-rate of the UART line
				.The method refers to the Object Dictionary: 0x3026
  * @param[in] baudrate  enum that specify the baud-rate of the UART line 
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetUartBaudrate(SOLOMotorControllers::UartBaudrate baudrate, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	soloUtils->ConvertToData((long)baudrate, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_UartBaudrate, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command starts or stops the process of sensor calibration
				.The method refers to the Object Dictionary: 0x3027
  * @param[in] calibrationAction  enum that specify the process of sensor calibration 
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SensorCalibration(SOLOMotorControllers::PositionSensorCalibrationAction calibrationAction, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	soloUtils->ConvertToData((long)calibrationAction, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_SensorCalibration, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the per-unit offset identified after sensor calibration
  *         for Encoder or Hall sensors in C.C.W direction
				.The method refers to the Object Dictionary: 0x3028
  * @param[in] encoderHallOffset  a float value between 0.0 to 1.0  
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetEncoderHallCcwOffset(float encoderHallOffset, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetEncoderHallCcwOffsetInputValidation(encoderHallOffset, error))
	{
		return false;
	}
	soloUtils->ConvertToData(encoderHallOffset, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_EncoderHallCcwOffset, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the per-unit offset identified after sensor calibration
  *         for Encoder or Hall sensors in C.W direction
				.The method refers to the Object Dictionary: 0x3029
  * @param[in] encoderHallOffset  a float value between 0.0 to 1.0   
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetEncoderHallCwOffset(float encoderHallOffset, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetEncoderHallCwOffsetInputValidation(encoderHallOffset, error))
	{
		return false;
	}
	soloUtils->ConvertToData(encoderHallOffset, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_EncoderHallCwOffset, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command defines the acceleration value of the Speed for speed controller
  *         both in Analogue and Digital modes in Revolution per square seconds
				.The method refers to the Object Dictionary: 0x302A
  * @param[in] speedAccelerationValue  a float value between 0 to 1600  
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetSpeedAccelerationValue(float speedAccelerationValue, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetSpeedAccelerationValueInputValidation(speedAccelerationValue, error))
	{
		return false;
	}
	soloUtils->ConvertToData(speedAccelerationValue, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_SpeedAccelerationValue, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command defines the deceleration value of the Speed for speed controller
  *         both in Analogue and Digital modes in Revolution per square seconds
				.The method refers to the Object Dictionary: 0x302B
  * @param[in] speedDecelerationValue  a float value between 0 to 1600   
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetSpeedDecelerationValue(float speedDecelerationValue, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetSpeedDecelerationValueInputValidation(speedDecelerationValue, error))
	{
		return false;
	}
	soloUtils->ConvertToData(speedDecelerationValue, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_SpeedDecelerationValue, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the baud rate of CAN bus in CANOpen network
				.The method refers to the Object Dictionary: 0x302C
  * @param[in] canbusBaudrate  enum that specify the baud rate of CAN bus in CANOpen network 
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetCanbusBaudrate(CanbusBaudrate canbusBaudrate, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	soloUtils->ConvertToData((long)canbusBaudrate, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_CanbusBaudrate, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command defines the resolution of the speed at S/T input
  *           while SOLO operates in Analogue mode
  *           .The method refers to the Object Dictionary: 0x303E
  * @param[in] divisionCoefficient  a long value    
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetAnalogueSpeedResolutionDivisionCoefficient(long divisionCoefficient, int &error)
{	
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetAnalogueSpeedResolutionDivisionCoefficientInputValidation(divisionCoefficient, error))
	{
		return false;
	}
    soloUtils->ConvertToData((long) divisionCoefficient, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_ASRDC, 0x00, informatrionToSend, error);
}

/**
  * @brief  This command defines the type of the Motion Profile that is 
  *           being used in Speed or Position Modes
  *           .The method refers to the Object Dictionary: 0x3040
  * @param[in] motionProfileMode enum that specify the type of the Motion Profile   
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetMotionProfileMode( MotionProfileMode motionProfileMode, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    soloUtils->ConvertToData((long) motionProfileMode, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_MotionProfileMode, 0x00, informatrionToSend, error);
}

/**
  * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles 
				.The method refers to the Object Dictionary: 0x3041
  * @param[in] MotionProfileVariable1 a float value   
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetMotionProfileVariable1(float MotionProfileVariable1, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetMotionProfileVariable1InputValidation(MotionProfileVariable1, error))
	{
		return false;
	}
    soloUtils->ConvertToData((float) MotionProfileVariable1, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_MotionProfileVariable1, 0x00, informatrionToSend, error);
}

/**
  * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles 
				.The method refers to the Object Dictionary: 0x3042
  * @param[in] MotionProfileVariable2 a float value   
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetMotionProfileVariable2(float MotionProfileVariable2, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetMotionProfileVariable2InputValidation(MotionProfileVariable2, error))
	{
		return false;
	}
    soloUtils->ConvertToData((float) MotionProfileVariable2, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_MotionProfileVariable2, 0x00, informatrionToSend, error);
}

/**
  * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles 
				.The method refers to the Object Dictionary: 0x3043
  * @param[in] MotionProfileVariable3 a float value   
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetMotionProfileVariable3(float MotionProfileVariable3, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetMotionProfileVariable3InputValidation(MotionProfileVariable3, error))
	{
		return false;
	}
    soloUtils->ConvertToData((float) MotionProfileVariable3, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_MotionProfileVariable3, 0x00, informatrionToSend, error);
}

/**
  * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles 
				.The method refers to the Object Dictionary: 0x3044
  * @param[in] MotionProfileVariable4 a float value   
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetMotionProfileVariable4(float MotionProfileVariable4, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetMotionProfileVariable4InputValidation(MotionProfileVariable4, error))
	{
		return false;
	}
    soloUtils->ConvertToData((float) MotionProfileVariable4, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_MotionProfileVariable4, 0x00, informatrionToSend, error);
}

/**
  * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles 
				.The method refers to the Object Dictionary: 0x3045
  * @param[in] MotionProfileVariable5 a float value   
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetMotionProfileVariable5(float MotionProfileVariable5, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetMotionProfileVariable5InputValidation(MotionProfileVariable5, error))
	{
		return false;
	}
    soloUtils->ConvertToData((float) MotionProfileVariable5, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_MotionProfileVariable5, 0x00, informatrionToSend, error);
}

/**
  * @brief  This is a 32 bits register providing the possibility of writing 0 or 1 into digital outputs provided at
  *				hardware, by setting each bit 0 the respective output will be kept LOW and by writing 1 the
  *				respective output will be kept at HIGH
  *				.The method refers to the Object Dictionary: 0x3048
  * @param[in] channel enum that specify the Channel of GPIO Outputs 
  * @param[in] state enum that specify the type of the State of GPIO Outputs   
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetDigitalOutputState(Channel channel, DigitalIoState state, int &error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
	long lastOutRegister;

	lastOutRegister = GetDigitalOutputsRegister(error);
	if(error = 0)
		return error;

	if(state == 1)
		lastOutRegister = lastOutRegister | (1 << channel);
	else
		lastOutRegister = lastOutRegister & (~(1 << channel));

    soloUtils->ConvertToData(lastOutRegister, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_DigitalOutputRegister, 0x00, informatrionToSend, error);
}

/**
  * @brief  This command defines the maximum allowed regeneration current sent back from the Motor to
  *				the Power Supply during decelerations
  *           .The method refers to the Uart Write command: 0x304B
  * @param[in]  current a float value  
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetRegenerationCurrentLimit(float current, int& error)
{
	uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetRegenerationCurrentLimitValidation(current, error))
	{
		return false;
	}
    soloUtils->ConvertToData((float) current, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_RegenerationCurrentLimit, 0x00, informatrionToSend, error);
}

/**
  * @brief  This value defines the the sampling window of qualification digital filter applied to the output of
  *			the position sensor before being processed by DSP
  *           .The method refers to the Uart Write command: 0x304C
  * @param[in]  level a long value  
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetPositionSensorDigitalFilterLevel(long level, int &error)
{
	uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetPositionSensorDigitalFilterLevelValidation(level, error))
	{
		return false;
	}
    soloUtils->ConvertToData((long) level, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_PositionSensorDigitalFilterLevel, 0x00, informatrionToSend, error);
}

/**
  * @brief  This PDO command sets the desired Position reference in terms of quadrature
  *         pulses while SOLO operates with the Incremental Encoders or in terms of
  *         pulses while while SOLO operates with Hall sensors
  *				.The method refers to the Object Dictionary: 0x1414
  * @param[in] positionReference  a long value between -2,147,483,647 to 2,147,483,647   
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetPdoPositionReference(long positionReference, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetPositionReferenceInputValidation(positionReference, error))
	{
		return false;
	}
	return SOLOMotorControllersCanopenKvaser::SetPdoParameterValue(PdoParameterName::positionReference,positionReference, error);
}

/**
  * @brief  This PDO command defines the speed reference for SOLO once it’s in Digital Speed Mode
  *				.The method refers to the Object Dictionary: 0x1415
  * @param[in] speedReference  a long value defining the speed (only positive)
  * @param[out]  error   pointer that specify result of function       
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetPdoSpeedReference(long speedReference, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetSpeedReferenceInputValidation(speedReference, error))
	{
		return false;
	}
	
	return SOLOMotorControllersCanopenKvaser::SetPdoParameterValue(PdoParameterName::speedReference,speedReference, error);
}

/**
  * @brief  This PDO command sets the amount of desired current that acts in torque generation
  *				.The method refers to the Object Dictionary: 0x1416
  * @param[in] torqueReferenceIq  a float value between 0 to 32
  * @param[out]  error   pointer to an integer that specify result of function       
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetPdoTorqueReferenceIq(float torqueReferenceIq, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetTorqueReferenceIqInputValidation(torqueReferenceIq, error))
	{
		return false;
	}
	return SOLOMotorControllersCanopenKvaser::SetPdoParameterValue(PdoParameterName::torqueReferenceIq,torqueReferenceIq, error);
}

/**
  * @brief  this PDO command depending on the Motor type: in case of BLDC or PMSM motors Sets the Field
  *         Weakening current reference to help the motor reaching speeds higher than
  *         nominal values and in case of AC Induction Motors Sets the desired magnetizing
  *         current (Id) required for controlling ACIM motors in FOC in Amps 
  *				.The method refers to the Object Dictionary: 0x1417
  * @param[in] magnetizingCurrentIdReference  a float value between 0 to 32   
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetPdoMagnetizingCurrentIdReference(float magnetizingCurrentIdReference, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetMagnetizingCurrentIdReferenceInputValidation(magnetizingCurrentIdReference, error))
	{
		return false;
	}
	return SOLOMotorControllersCanopenKvaser::SetPdoParameterValue(PdoParameterName::magnetizingCurrentIdReference,magnetizingCurrentIdReference, error);
}

/**
  * @brief  This PDO command sets the Control Mode in terms of Torque,
  *         Speed or Position only in Digital Mode
  *				.The method refers to the Object Dictionary: 0x1418
  * @param[in] controlMode  enum that specify the Control Mode in terms of Torque,
  *                       Speed or Position only in Digital Mode
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetPdoControlMode(SOLOMotorControllers::ControlMode controlMode, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersCanopenKvaser::SetPdoParameterValue(PdoParameterName::controlMode,(long)controlMode, error);
}

/**
  * @brief  This PDO command sets the direction of the rotation of the motor
  *         either to ClockWise rotation or to Counter Clockwise Rotation
  *				.The method refers to the Object Dictionary: 0x1419
  * @param[in] motorDirection  enum that specify the direction of the rotation of the motor 
  * @param[out]  error   pointer to an integer that specify result of function     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersCanopenKvaser::SetPdoMotorDirection(SOLOMotorControllers::Direction motorDirection, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersCanopenKvaser::SetPdoParameterValue(PdoParameterName::motorDirection,(long)motorDirection, error);
}

////---------------------Read---------------------
long SOLOMotorControllersCanopenKvaser::GetReadErrorRegister(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (kvaser->CANOpenReceive(hnd,Address, Object_ReadErrorRegister, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}

long SOLOMotorControllersCanopenKvaser::GetGuardTime(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_GuardTime, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}

long SOLOMotorControllersCanopenKvaser::GetLifeTimeFactor(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_LifeTimeFactor, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}

long SOLOMotorControllersCanopenKvaser::GetProducerHeartbeatTime(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_ProducerHeartbeatTime, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}

/**
  * @brief  This command reads the device address connected on the line 
				.The method refers to the Object Dictionary: 0x3001
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long device address connected on the line
  */
long SOLOMotorControllersCanopenKvaser::GetDeviceAddress(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_SetDeviceAddress, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}

/**
  * @brief  This command reads the phase-A voltage of the motor connected to the
  *         "A" pin output of SOLO for 3-phase Motors 
				.The method refers to the Object Dictionary: 0x302D
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float phase-A voltage of the motor between -60 to 60
  */
float SOLOMotorControllersCanopenKvaser::GetPhaseAVoltage(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_PhaseAVoltage, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the phase-B voltage of the motor connected to the
  *         "B" pin output of SOLO for 3-phase Motors  
				.The method refers to the Object Dictionary: 0x302E
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float 0 phase-A voltage of the motor between -60 to 60
  */
float SOLOMotorControllersCanopenKvaser::GetPhaseBVoltage(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_PhaseBVoltage, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the phase-A current of the motor connected to the
  *         "A" pin output of SOLO for 3-phase Motors
				.The method refers to the Object Dictionary: 0x302F
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval phase-A current of the motor etween -32 to 32 Amps
  */
float SOLOMotorControllersCanopenKvaser::GetPhaseACurrent(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_PhaseACurrent, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the phase-B current of the motor connected to the
  *         "B" pin output of SOLO for 3-phase Motors 
				.The method refers to the Object Dictionary: 0x3030
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float phase-B current of the motor etween -32 to 32 Amps
  */
float SOLOMotorControllersCanopenKvaser::GetPhaseBCurrent(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_PhaseBCurrent, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the input BUS voltage  
				.The method refers to the Object Dictionary: 0x3031
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float  BUS voltage between 0 to 60
  */
float SOLOMotorControllersCanopenKvaser::GetBusVoltage(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_BusVoltage, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the current inside the DC brushed motor connected to
  *         "B" and "C" outputs of SOLO 
				.The method refers to the Object Dictionary: 0x3032
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between -32 to 32
  */
float SOLOMotorControllersCanopenKvaser::GetDcMotorCurrentIm(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_DcMotorCurrentIm, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the voltage of the DC brushed motor connected to
  *         "B" and "C" outputs of SOLO 
				.The method refers to the Object Dictionary: 0x3033
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between -60 to 60
  */
float SOLOMotorControllersCanopenKvaser::GetDcMotorVoltageVm(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_DcMotorVoltageVm, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the value of the Speed controller Kp gain, 
  *         set for Digital mode operations  
				.The method refers to the Object Dictionary: 0x300A
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 16000
  */
float SOLOMotorControllersCanopenKvaser::GetSpeedControllerKp(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_SpeedControllerKp, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the value of the Speed controller Ki gain,
  *         set for Digital mode operations  
				.The method refers to the Object Dictionary: 0x300B
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 16000
  */
float SOLOMotorControllersCanopenKvaser::GetSpeedControllerKi(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_SpeedControllerKi, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the output switching frequency of SOLO in KHz  
  *				.The method refers to the Object Dictionary: 0x3009
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long between 8 to 80 Hz
  */
long SOLOMotorControllersCanopenKvaser::GetOutputPwmFrequencyKhz(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_OutputPwmFrequency, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}

/**
  * @brief  This command reads the value of the current limit set for SOLO in
  *         closed-loop digital operation mode   
				.The method refers to the Object Dictionary: 0x3003
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 32
  */
float SOLOMotorControllersCanopenKvaser::GetCurrentLimit(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_CurrentLimit, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the actual monetary value of “Iq” that is
  *         the current acts in torque generation in FOC mode for 3-phase motors
				.The method refers to the Object Dictionary: 0x3034
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between -64 to 64
  */
float SOLOMotorControllersCanopenKvaser::GetQuadratureCurrentIqFeedback(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_QuadratureCurrentIqFeedback, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the actual monetary value of Id that is the
  *         direct current acting in FOC 
				.The method refers to the Object Dictionary: 0x3035
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between -64 to 64
  */
float SOLOMotorControllersCanopenKvaser::GetMagnetizingCurrentIdFeedback(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_MagnetizingCurrentIdFeedback, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the number of Poles set for 3-phase motors 
				.The method refers to the Object Dictionary: 0x300F
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long between 1 to 254
  */
long SOLOMotorControllersCanopenKvaser::GetMotorPolesCounts(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_MotorPolesCounts, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}

/**
  * @brief  This command reads the number of physical Incremental encoder lines set on SOLO   
				.The method refers to the Object Dictionary: 0x3010
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long between 1 to 200000
  */
long SOLOMotorControllersCanopenKvaser::GetIncrementalEncoderLines(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_IncrementalEncoderLines, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}

/**
  * @brief  This command reads the amount of value set for Current controller
  *         Kp or proportional gain 
				.The method refers to the Object Dictionary: 0x3017
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 16000
  */
float SOLOMotorControllersCanopenKvaser::GetCurrentControllerKp(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_CurrentControllerKp, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the amount of value set for Current controller
  *         Ki or integrator gain  
				.The method refers to the Object Dictionary: 0x3018
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 16000
  */
float SOLOMotorControllersCanopenKvaser::GetCurrentControllerKi(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_CurrentControllerKi, 0x22, informationToSend, informationReceived, error)) {
		float readValue = soloUtils->ConvertToFloat(informationReceived);
		return readValue;
	}
	return -1.0;
}

/**
  * @brief  This command reads the momentary temperature of the board in centigrade 
				.The method refers to the Object Dictionary: 0x3039
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between -30 to 150 Celsius
  */
float SOLOMotorControllersCanopenKvaser::GetBoardTemperature(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_BoardTemperature, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the Phase or Armature resistance of
  *         the 3-phase or DC brushed motor connected to SOLO respectively  
  *				.The method refers to the Object Dictionary: 0x300D
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float [Ohm] between 0 to 100
  */
float SOLOMotorControllersCanopenKvaser::GetMotorResistance(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_MotorResistance, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the Phase or Armature Inductance of 
  *         the 3-phase or DC brushed motor connected to SOLO respectively
  *				.The method refers to the Object Dictionary: 0x300E
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 0.2
  */
float SOLOMotorControllersCanopenKvaser::GetMotorInductance(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_MotorInductance, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  this command reads the actual speed of the motor measured or estimated by SOLO in
            sensorless or sensor-based modes respectively  
				.The method refers to the Object Dictionary: 0x3036
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long between -30000 to 30000 RPM
  */
long SOLOMotorControllersCanopenKvaser::GetSpeedFeedback(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_SpeedFeedback, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}

/**
  * @brief  This command reads the Motor type selected for Digital or Analogue mode operations 
				.The method refers to the Object Dictionary: 0x3015
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval enum @ref MotorType
  */
SOLOMotorControllers::MotorType SOLOMotorControllersCanopenKvaser::GetMotorType(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_MotorType, 0x22, informationToSend, informationReceived, error)) {
		return ((SOLOMotorControllers::MotorType)soloUtils->ConvertToLong(informationReceived));
	}
	return SOLOMotorControllers::MotorType::motorTypeError;
}

/**
  * @brief  This command reads the feedback control mode selected on SOLO both
  *         for Analogue and Digital operations  
				.The method refers to the Object Dictionary: 0x3013
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval enum @ref FeedbackControlMode
  */
SOLOMotorControllers::FeedbackControlMode SOLOMotorControllersCanopenKvaser::GetFeedbackControlMode(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_FeedbackControlMode, 0x22, informationToSend, informationReceived, error)) {
		return ((SOLOMotorControllers::FeedbackControlMode)soloUtils->ConvertToLong(informationReceived));
	}
	return SOLOMotorControllers::FeedbackControlMode::feedbackControlModeError;
}

/**
  * @brief  This command reads the actual commanding mode that SOLO is operating 
				.The method refers to the Object Dictionary: 0x3002
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval enum @ref CommandMode
  */
SOLOMotorControllers::CommandMode SOLOMotorControllersCanopenKvaser::GetCommandMode(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_CommandMode, 0x22, informationToSend, informationReceived, error)) {
		return ((SOLOMotorControllers::CommandMode)soloUtils->ConvertToLong(informationReceived));
	}
	return SOLOMotorControllers::CommandMode::commandModeError;
}

/**
  * @brief  This command reads the Control Mode type in terms of Torque,
  *         Speed or Position in both Digital and Analogue modes 
				.The method refers to the Object Dictionary: 0x3013
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval enum @ref ControlMode
  */
SOLOMotorControllers::ControlMode SOLOMotorControllersCanopenKvaser::GetControlMode(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_ControlMode, 0x22, informationToSend, informationReceived, error)) {
		return ((SOLOMotorControllers::ControlMode)soloUtils->ConvertToLong(informationReceived));
	}
	return SOLOMotorControllers::ControlMode::controlModeError;
}

/**
  * @brief  This command reads the value of the speed limit set on SOLO 
				.The method refers to the Object Dictionary: 0x3011
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long between 0 to 30000
  */
long SOLOMotorControllersCanopenKvaser::GetSpeedLimit(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_SpeedLimit, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}

/**
  * @brief  This command reads the amount of value set for Position
  *         controller Kp or proportional gain  
				.The method refers to the Object Dictionary: 0x301C
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 16000
  */
float SOLOMotorControllersCanopenKvaser::GetPositionControllerKp(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_PositionControllerKp, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the amount of value set for Position
  *         controller Ki or integrator gain  
				.The method refers to the Object Dictionary: 0x301D
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 16000
  */
float SOLOMotorControllersCanopenKvaser::GetPositionControllerKi(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_PositionControllerKi, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the number of counted pulses from the
  *         Incremental Encoder or Hall sensors 
				.The method refers to the Object Dictionary: 0x3037
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long between -2,147,483,647 to 2,147,483,647
  */
long SOLOMotorControllersCanopenKvaser::GetPositionCountsFeedback(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_PositionCountsFeedback, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}

/**
  * @brief  This command reads the error register which is a 32 bit register with
  *         each bit corresponding to specific errors  
				.The method refers to the Object Dictionary: 0x3020
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long 
  */
long SOLOMotorControllersCanopenKvaser::GetErrorRegister(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_OverwriteErrorRegister, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}

/**
  * @brief  This command reads the Firmware version existing currently on the SOLO unit  
				.The method refers to the Object Dictionary: 0x303A
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long
  */
long SOLOMotorControllersCanopenKvaser::GetDeviceFirmwareVersion(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_DeviceFirmwareVersion, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}

/**
  * @brief  This command reads the Hardware version of the SOLO unit connected  
				.The method refers to the Object Dictionary: 0x303B
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long
  */
long SOLOMotorControllersCanopenKvaser::GetDeviceHardwareVersion(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_DeviceHardwareVersion, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}

/**
  * @brief  This command reads the amount of desired Torque reference (Iq or IM)
  *         already set for the Motor to follow in Digital Closed-loop Torque control mode 	
				.The method refers to the Object Dictionary: 0x3004
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 32
  */
float SOLOMotorControllersCanopenKvaser::GetTorqueReferenceIq(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_TorqueReferenceIq, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the amount of desired Speed reference already set for
  *         the Motor to follow in Digital Closed-loop Speed control mode  
				.The method refers to the Object Dictionary: 0x3005
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long between 0 to 30000
  */
long SOLOMotorControllersCanopenKvaser::GetSpeedReference(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_SpeedReference, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}

/**
  * @brief  This command the actual monetary value of Id that is the
  *         direct current acting in FOC  
  *         in Digital Closed-loop Speed control mode for ACIM motors 
				.The method refers to the Object Dictionary: 0x301A
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 32
  */
float SOLOMotorControllersCanopenKvaser::GetMagnetizingCurrentIdReference(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_MagnetizingCurrentIdReference, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the desired position reference set for the Motor
  *         to follow in Digital Closed-loop Position mode in terms of quadrature pulses 
				.The method refers to the Object Dictionary: 0x301B
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long between -2,147,483,647 to 2,147,483,647 Quad-Pulses
  */
long SOLOMotorControllersCanopenKvaser::GetPositionReference(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_PositionReference, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}

/**
  * @brief  This command reads the desired Power reference for SOLO to apply in 
  *         Digital Open-loop speed control mode for 3-phase motors in terms of percentage
				.The method refers to the Object Dictionary: 0x3006
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 100 %
  */
float SOLOMotorControllersCanopenKvaser::GetPowerReference(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_PowerReference, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This commands reads the desired direction of rotation set for the Motor 
				.The method refers to the Object Dictionary: 0x300C
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval enum @ref Direction
  */
SOLOMotorControllers::Direction SOLOMotorControllersCanopenKvaser::GetMotorDirection(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_MotorDirection, 0x22, informationToSend, informationReceived, error)) {
		return ((SOLOMotorControllers::Direction)soloUtils->ConvertToLong(informationReceived));
	}
	return SOLOMotorControllers::Direction::directionError;
}

/**
  * @brief  This command reads the value of Sensorless Zero Speed Full Torque Injection Amplitude 
  *				.The method refers to the Object Dictionary: 0x3021
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0.01 to 1000
  */
float SOLOMotorControllersCanopenKvaser::GetZsftInjectionAmplitude(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_ZsftInjectionAmplitude, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the value of Sensorless Zero Speed Full Torque Polarity Amplitude 
				.The method refers to the Object Dictionary: 0x3022
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0.0 to 0.55
  */
float SOLOMotorControllersCanopenKvaser::GetZsftPolarityAmplitude(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_ZsftPolarityAmplitude, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the value of Sensorless Observer Gain for DC Motor  
				.The method refers to the Object Dictionary: 0x3023
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0.01 to 1000
  */
float SOLOMotorControllersCanopenKvaser::GetObserverGainDc(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_ObserverGainDc, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the value of Sensorless Zero Speed Full Torque Injection Frequency
				.The method refers to the Object Dictionary: 0x3024
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long between 0 to 10
  */
long SOLOMotorControllersCanopenKvaser::GetZsftInjectionFrequency(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_ZsftInjectionFrequency, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the value of Sensorless Transition Speed
				.The method refers to the Object Dictionary: 0x3025
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long between 1 to 5000
  */
long SOLOMotorControllersCanopenKvaser::GetSensorlessTransitionSpeed(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_SensorlessTransactionSpeed, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the measured or estimated per-unit angle of the 3-phase motors  
				.The method refers to the Object Dictionary: 0x3038
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between -2 to 2
  */
float SOLOMotorControllersCanopenKvaser::Get3PhaseMotorAngle(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_3PhaseMotorAngle, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the per-unit Encoder or Hall sensor offset in C.C.W direction 
				.The method refers to the Object Dictionary: 0x3028
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 1.0
  */
float SOLOMotorControllersCanopenKvaser::GetEncoderHallCcwOffset(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_EncoderHallCcwOffset, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the per-unit Encoder or Hall sensor offset in C.C.W direction  
				.The method refers to the Object Dictionary: 0x3029
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 1.0
  */
float SOLOMotorControllersCanopenKvaser::GetEncoderHallCwOffset(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_EncoderHallCwOffset, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads Baud Rate selected on SOLO unit to communicate through UART line  	
				.The method refers to the Object Dictionary: 0x3026
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval enum @ref UartBaudrate
  */
SOLOMotorControllers::UartBaudrate SOLOMotorControllersCanopenKvaser::GetUartBaudrate(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_UartBaudrate, 0x22, informationToSend, informationReceived, error)) {
		return ((SOLOMotorControllers::UartBaudrate)soloUtils->ConvertToLong(informationReceived));
	}
	return SOLOMotorControllers::UartBaudrate::uartBaudrateError;
}

/**
  * @brief  This command reads the acceleration value of the Speed for
  *         speed controller both in Analogue and Digital modes
  *         in Revolution per square seconds  
				.The method refers to the Object Dictionary: 0x302A
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 1600 Rev/S^2
  */
float SOLOMotorControllersCanopenKvaser::GetSpeedAccelerationValue(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_SpeedAccelerationValue, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the deceleration value of the Speed for
  *         speed controller both in Analogue and Digital modes
  *         in Revolution per square seconds 
				.The method refers to the Object Dictionary: 0x302B
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 1600 Rev/S^2
  */
float SOLOMotorControllersCanopenKvaser::GetSpeedDecelerationValue(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_SpeedDecelerationValue, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the Analogue Speed Resolution Division Coefficient (ASRDC)
				.The method refers to the Object Dictionary: 0x303E
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long
  */
long SOLOMotorControllersCanopenKvaser::GetAnalogueSpeedResolutionDivisionCoefficient(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
	if (kvaser->CANOpenReceive(hnd, Address, Object_ASRDC, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
    return -1 ;
}

/**
  * @brief  This Command reads the number of counted index pulses 
  *         seen on the Incremental Encoder’s output  
				.The method refers to the Object Dictionary: 0x303D
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long between 0 to 2,147,483,647
  */
long SOLOMotorControllersCanopenKvaser::GetEncoderIndexCounts(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_EncoderIndexCounts, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;

}

/**
  * @brief  This command test if the communication is working   
  * @retval bool 0 not working / 1 for working
  */
bool SOLOMotorControllersCanopenKvaser::CommunicationIsWorking(int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	float temperature = SOLOMotorControllersCanopenKvaser::GetBoardTemperature(error);
	if (error == SOLOMotorControllers::Error::noErrorDetected) {
		return true;
	}
	return false;
}

/**
  * @brief  This command reads the type of the Embedded Motion profile active in the controller 
				.The method refers to the Object Dictionary: 0x303F
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval enum @ref MotionProfileMode
  */
SOLOMotorControllers::MotionProfileMode SOLOMotorControllersCanopenKvaser::GetMotionProfileMode(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
	if (kvaser->CANOpenReceive(hnd, Address, Object_MotionProfileMode, 0x22, informationToSend, informationReceived, error)) {
		return ((SOLOMotorControllers::MotionProfileMode)soloUtils->ConvertToLong(informationReceived));
	}
    return SOLOMotorControllers::MotionProfileMode::motionProfileModeError;
}

/**
  * @brief  This command reads the value of the Motion Profile Variable1 set inside the controller 
				.The method refers to the Object Dictionary: 0x3040
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float
  */
float SOLOMotorControllersCanopenKvaser::GetMotionProfileVariable1(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
	if (kvaser->CANOpenReceive(hnd, Address, Object_MotionProfileVariable1, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
    return -1 ;
}

/**
  * @brief  This command reads the value of the Motion Profile Variable2 set inside the controller 
				.The method refers to the Object Dictionary: 0x3041
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float
  */
float SOLOMotorControllersCanopenKvaser::GetMotionProfileVariable2(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
	if (kvaser->CANOpenReceive(hnd, Address, Object_MotionProfileVariable2, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
    return -1 ;
}

/**
  * @brief  This command reads the value of the Motion Profile Variable3 set inside the controller 
				.The method refers to the Object Dictionary: 0x3042
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float
  */
float SOLOMotorControllersCanopenKvaser::GetMotionProfileVariable3(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
	if (kvaser->CANOpenReceive(hnd, Address, Object_MotionProfileVariable3, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
    return -1 ;
}

/**
  * @brief  This command reads the value of the Motion Profile Variable4 set inside the controller
				.The method refers to the Object Dictionary: 0x3043
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float
  */
float SOLOMotorControllersCanopenKvaser::GetMotionProfileVariable4(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
	if (kvaser->CANOpenReceive(hnd, Address, Object_MotionProfileVariable4, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
    return -1 ;
}

/**
  * @brief  This command reads the value of the Motion Profile Variable5 set inside the controller 
				.The method refers to the Object Dictionary: 0x3044
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float
  */
float SOLOMotorControllersCanopenKvaser::GetMotionProfileVariable5(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
	if (kvaser->CANOpenReceive(hnd, Address, Object_MotionProfileVariable5, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
    return -1 ; 
}

/**
  * @brief  This command reads the value of the Digital Outputs Register as a 32 bits register 
  *           .The method refers to the Uart Read command: 0x3048
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval enum @ref DigitalIoState
  */
SOLOMotorControllers::DigitalIoState SOLOMotorControllersCanopenKvaser::GetDigitalOutputState(Channel channel, int &error)
{
    long lastOutRegister;
	lastOutRegister = GetDigitalOutputsRegister(error);
	if(error = SOLOMotorControllers::Error::noErrorDetected)
		return SOLOMotorControllers::DigitalIoState::digitalIoStateError;
	return (SOLOMotorControllers::DigitalIoState)((lastOutRegister >> channel) & 0x00000001);
}

long SOLOMotorControllersCanopenKvaser::GetDigitalOutputsRegister(int &error)
{
	uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
	if (kvaser->CANOpenReceive(hnd, Address, Object_DigitalOutputRegister, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
    return -1 ; 
}

/**
  * @brief  This command reads the current state of the controller
  *           .The method refers to the Uart Read command: 0x3008
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval enum @ref DisableEnable
  */
SOLOMotorControllers::DisableEnable SOLOMotorControllersCanopenKvaser::GetDriveDisableEnable(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
	if (kvaser->CANOpenReceive(hnd, Address, Object_DriveDisableEnable, 0x22, informationToSend, informationReceived, error)) {
		return ((SOLOMotorControllers::DisableEnable)soloUtils->ConvertToLong(informationReceived));
	}
    return SOLOMotorControllers::DisableEnable::disableEnableError;  
}

/**
  * @brief  This command reads the value of the Regeneration Current Limit
  *           .The method refers to the Uart Read command: 0x304B
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float
  */
float SOLOMotorControllersCanopenKvaser::GetRegenerationCurrentLimit(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
	if (kvaser->CANOpenReceive(hnd, Address, Object_RegenerationCurrentLimit, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
    return -1 ;  
}

/**
  * @brief  This command reads the value of the Position Sensor Digital Filter Level
  *           .The method refers to the Uart Read command: 0x304C
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long
  */
long SOLOMotorControllersCanopenKvaser::GetPositionSensorDigitalFilterLevel(int &error)
{
	uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
	if (kvaser->CANOpenReceive(hnd, Address, Object_PositionSensorDigitalFilterLevel, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
    return -1 ; 
}

/**
  * @brief  This command reads the value of the Digital Input Register as a 32 bits register
  *           .The method refers to the Uart Read command: 0x3049
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long
  */
 long SOLOMotorControllersCanopenKvaser::GetDigitalInputRegister(int &error)
{
	uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
	if (kvaser->CANOpenReceive(hnd, Address, Object_DigitalInputRegister, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
    return -1 ; 
}

/**
  * @brief  This command reads the value of the voltage sensed at the output of PT1000 temperature
  *			sensor amplifier, this command can be used only on devices that come with PT1000 input
  *           .The method refers to the Uart Read command: 0x3047
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long
  */
long SOLOMotorControllersCanopenKvaser::GetPT1000SensorVoltage(int &error)
{
	uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
	if (kvaser->CANOpenReceive(hnd, Address, Object_PT1000SensorVoltage, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
    return -1 ; 
}

/**
  * @brief  This command reads the quantized value of an Analogue Input as a number between 0 to 4095
  *				depending on the maximum voltage input possible at the analogue inputs for the controller
  *           .The method refers to the Uart Read command: 0x304A
  * @param[in]  channel  an enum that specify the Channel of Analogue Input 
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval enum @ref DigitalIoState
  */
SOLOMotorControllers::DigitalIoState SOLOMotorControllersCanopenKvaser::GetAnalogueInput(Channel channel, int &error)
{
	uint8_t  informationToSend  [4] = {0x00,0x00,0x00,(uint8_t)channel};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
	if (kvaser->CANOpenReceive(hnd, Address, Object_AnalogueInput, 0x22, informationToSend, informationReceived, error)) {
		return ((SOLOMotorControllers::DigitalIoState)soloUtils->ConvertToLong(informationReceived));
	}
    return SOLOMotorControllers::DigitalIoState::digitalIoStateError; 
}

void SOLOMotorControllersCanopenKvaser::GeneralCanbusRead(uint16_t *ID , uint8_t *DLC, uint8_t *Data)
{
	kvaser->CANOpenGenericReceive(hnd, ID , DLC, Data);
}

void SOLOMotorControllersCanopenKvaser::GeneralCanbusWrite(uint16_t ID, uint8_t *DLC, uint8_t *Data, int &error)
{
	kvaser->CANOpenGenericTransmit(hnd, ID , DLC, Data, error);
}

/**
  * @brief  this PDO command give the first in the baffer position of the Motor
  *         to follow in Digital Closed-loop Position mode in terms of quadrature pulses 
  *				.The method refers to the Object Dictionary: 0x1814
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long between -2,147,483,647 to 2,147,483,647 Quad-Pulses
  */
long SOLOMotorControllersCanopenKvaser::GetPdoPositionCountsFeedback(int& error)
{
	return GetPdoParameterValueLong(PdoParameterName::positionCountsFeedback, error);
}

/**
  * @brief  this PDO command give the first in the baffer speed of the motor measured or estimated by SOLO in
  *		    sensorless or sensor-based modes respectively  
  *				.The method refers to the Object Dictionary: 0x1815
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long value that rappresent the speed feeback in RPM (positive and negative value) 
  */
long SOLOMotorControllersCanopenKvaser::GetPdoSpeedFeedback (int &error){
	return GetPdoParameterValueLong(PdoParameterName::speedFeedback, error);
}

/**
  * @brief  This PDO command give the first in the baffer monetary value of “Iq” that is
  *         the current acts in torque generation in FOC mode for 3-phase motors
  *				.The method refers to the Object Dictionary: 0x1816
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 32
  */
float SOLOMotorControllersCanopenKvaser::GetPdoQuadratureCurrentIqFeedback(int& error)
{
	return GetPdoParameterValueFloat(PdoParameterName::quadratureCurrentIqFeedback, error);
}

/**
  * @brief  This PDO command give the first in the baffer monetary value of Id that is the
  *         direct current acting in FOC  
				.The method refers to the Object Dictionary: 0x1817
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 32
  */
float SOLOMotorControllersCanopenKvaser::GetPdoMagnetizingCurrentIdFeedback(int& error)
{
	return GetPdoParameterValueFloat(PdoParameterName::magnetizingCurrentIdFeedback, error);
}

/**
  * @brief  This PDO command reads the error register which is a 32 bit register with
  *         each bit corresponding to specific errors  
  *				.The method refers to the Object Dictionary: 0x1818
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long 
  */
long SOLOMotorControllersCanopenKvaser::GetPdoErrorRegister(int& error)
{
	return GetPdoParameterValueLong(PdoParameterName::errorRegister, error);
}


/**
  * @brief  This PDO command reads the momentary temperature of the board in centigrade 
  *				.The method refers to the Object Dictionary: 0x1819
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between -30 to 150 Celsius
  */
float SOLOMotorControllersCanopenKvaser::GetPdoBoardTemperature(int& error)
{
	return GetPdoParameterValueFloat(PdoParameterName::boardTemperature, error);
}