/**
 *******************************************************************************
 * @file    SOLOMotorControllersKvaser.cpp
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

#include "SOLOMotorControllersKvaser.h"

//DEBUG
// #include "stdio.h"
// #include <iostream>
// using std::cout;
// using std::endl;
// using std::hex; 

SOLOMotorControllersKvaser::SOLOMotorControllersKvaser(CommunicationInterface* ci,
        UINT8 deviceAddress, SOLOMotorControllers::CanbusBaudrate baudrate,
        long millisecondsTimeout, bool autoConnect)
	:Address(deviceAddress)
	, timeout(millisecondsTimeout)
    , comIf(ci)
{
	if (Address == 0) //Address 0 is reserved for the host 
	{
		Address = 1;
	}
	soloUtils = new SOLOMotorControllersUtils();
	InitPdoConfig();
}

SOLOMotorControllersKvaser::~SOLOMotorControllersKvaser()
{
	comIf->Disconnect();
}

bool SOLOMotorControllersKvaser::Connect(UINT8 deviceAddress, 
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

	return comIf->Connect();
}

bool SOLOMotorControllersKvaser::SetGuardTime(long guardtime, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetGuardTimeInputValidation(guardtime,error))
    {
        return false;
    }
    soloUtils->ConvertToData(guardtime, informatrionToSend);
	return comIf->CANOpenTransmit(Address, Object_GuardTime, 0x00, informatrionToSend, error);
}

bool SOLOMotorControllersKvaser::SetGuardTime(long guardtime)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersKvaser::SetGuardTime(guardtime,error);
}

bool SOLOMotorControllersKvaser::SetLifeTimeFactor(long lifeTimeFactor, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetLifeTimeFactorInputValidation(lifeTimeFactor,error))
    {
        return false;
    }
    soloUtils->ConvertToData(lifeTimeFactor, informatrionToSend);
	return comIf->CANOpenTransmit(Address, Object_LifeTimeFactor, 0x00, informatrionToSend, error);
}

bool SOLOMotorControllersKvaser::SetLifeTimeFactor(long lifeTimeFactor)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersKvaser::SetLifeTimeFactor(lifeTimeFactor,error);
}
bool SOLOMotorControllersKvaser::SetProducerHeartbeatTime(long producerHeartbeatTime, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    if(!soloUtils->SetProducerHeartbeatTimeInputValidation(producerHeartbeatTime,error))
    {
        return false;
    }
    soloUtils->ConvertToData(producerHeartbeatTime, informatrionToSend);
	return comIf->CANOpenTransmit(Address, Object_ProducerHeartbeatTime, 0x00, informatrionToSend, error);
}

bool SOLOMotorControllersKvaser::SetProducerHeartbeatTime(long producerHeartbeatTime)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersKvaser::SetProducerHeartbeatTime(producerHeartbeatTime,error);
}

/**
  * @brief  This command determine the validity of count of SYNC message
  * @param[in]  parameterName	enum that specifies the name of the PDO parameter that wants to set CobId value
  * @param[in]  parameterCobbId	CobId value
  * @param[out]  error   pointer to an integer that specifies the result of the function       
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetPdoParameterCobbIdInputValidation(PdoParameterName parameterName, int parameterCobbId, int &error)
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
bool SOLOMotorControllersKvaser::SetSyncParameterCountInputValidation(uint8_t parameterCount, int &error)
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
bool SOLOMotorControllersKvaser::SetPdoParameterConfig(PdoParameterConfig config, int &error)
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
	boolean isSuccess = comIf->CANOpenTransmit(Address, pdoParameterObjectByPdoParameterName[config.parameterName], 0x01, informatrionToSend, error);
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
		
		isSuccess = comIf->CANOpenTransmit(Address, pdoParameterObjectByPdoParameterName[config.parameterName], 0x02, informatrionToSend, error);
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
PdoParameterConfig SOLOMotorControllersKvaser::GetPdoParameterConfig(PdoParameterName parameterName, int &error)
{
	PdoParameterConfig config;
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, parameterName, 0x1, informationToSend, informationReceived, error))
	{
		config.parameterName = parameterName;
		config.parameterCobId = (informationReceived[2] << 8) + informationReceived[3];
		config.isPdoParameterEnable = informationReceived[0] & 0x80;
		config.isRrtParameterEnable = informationReceived[0] & 0x40;

		if (comIf->CANOpenReceive(Address, parameterName, 0x2, informationToSend, informationReceived, error))
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
bool SOLOMotorControllersKvaser::SendPdoSync(int &error)
{
	return comIf->SendPdoSync(error);
}

/**
  * @brief  This command send a SYNC message on bus
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SendPdoSync()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return comIf->SendPdoSync(error);
}

/**
  * @brief  This command send a RTR for the intended PDO object 
  * @param[in]  parameterName	enum that specifies the name of the PDO parameter that wants to send RTR
  * @param[out]  error   pointer to an integer that specifies the result of the function       
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SendPdoRtr(PdoParameterName parameterName, int &error)
{
	SOLOMotorControllersKvaser::PdoRtrValidParameter(parameterName, error);
	if(error != SOLOMotorControllers::Error::noErrorDetected){
		return false;
	}

	int adr = GetPdoParameterCobId(parameterName, error);
	if(error != SOLOMotorControllers::Error::noErrorDetected){
		return false;
	}

	return comIf->SendPdoRtr(adr, error);
}

/**
  * @brief  This command send a RTR for the intended PDO object 
  * @param[in]  parameterName	enum that specifies the name of the PDO parameter that wants to send RTR      
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SendPdoRtr(PdoParameterName parameterName)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SendPdoRtr(parameterName, error);
}

/**
  * @brief  This command checks the validity of allowing RTR  for the intended PDO parameter
  * @param[in]  parameterName	enum that specifies the name of the PDO parameter that wants to check RTR validity
  * @param[out]  error   pointer to an integer that specifies the result of the function       
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::PdoRtrValidParameter(PdoParameterName parameterName, int &error)
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
long SOLOMotorControllersKvaser::GetPdoParameterCobId(PdoParameterName parameterName, int &error)
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
void SOLOMotorControllersKvaser::InitPdoConfig()
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
bool SOLOMotorControllersKvaser::UpdatePdoParameterCobIdByPdoParameterName()
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
bool SOLOMotorControllersKvaser::SetPdoParameterValue(PdoParameterName parameterName, long value,
		int &error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	soloUtils->ConvertToData(value, informatrionToSend);

	int adr = GetPdoParameterCobId(parameterName, error);
	if(error == SOLOMotorControllers::Error::pdoMissingCobId){
		return false;
	}

	return comIf->PDOTransmit(adr, informatrionToSend, error);
}

/**
  * @brief  This command set the intended long value for a PDO command
  * @param[in]  parameterName	enum that specifies the name of the PDO parameter that wants to write its value
  * @param[in]  value	long value that wants to be set for the PDO parameter       
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetPdoParameterValue(PdoParameterName parameterName, long value)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetPdoParameterValue(parameterName, value, error);
}

/**
  * @brief  This command set the intended float value for a PDO command
  * @param[in]  parameterName	enum that specifies the name of the PDO parameter that wants to write its value
  * @param[in]  value	float value that wants to be set for the PDO parameter 
  * @param[out]  error   pointer to an integer that specifies the result of the function       
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetPdoParameterValue(PdoParameterName parameterName, float value,
		int &error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	soloUtils->ConvertToData(value, informatrionToSend);

	int adr = GetPdoParameterCobId(parameterName, error);
	if(error == SOLOMotorControllers::Error::pdoMissingCobId){
		return false;
	}

	return comIf->PDOTransmit(adr, informatrionToSend, error);
}

/**
  * @brief  This command set the intended float value for a PDO command
  * @param[in]  parameterName	enum that specifies the name of the PDO parameter that wants to write its value
  * @param[in]  value	the value that wants to be set for the PDO parameter     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetPdoParameterValue(PdoParameterName parameterName, float value)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetPdoParameterValue(parameterName, value, error);
}

/**
  * @brief  This command returns the long value of a PDO command
  * @param[in]  parameterName	enum that specifies the name of the parameter that wants to read its value
  * @param[out]  error   pointer to an integer that specifies the result of the function       
  * @retval long
  */
long SOLOMotorControllersKvaser::GetPdoParameterValueLong(PdoParameterName parameterName,
		int &error)
{
	uint8_t informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noErrorDetected;

	int adr = GetPdoParameterCobId(parameterName, error);
	if(error != SOLOMotorControllers::Error::noErrorDetected){
		return false;
	}

	if (comIf->PDOReceive(adr,informationReceived, error)) {
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
float SOLOMotorControllersKvaser::GetPdoParameterValueFloat(PdoParameterName parameterName,
		int &error)
{
	uint8_t informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noErrorDetected;

	int adr = GetPdoParameterCobId(parameterName, error);
	if(error != SOLOMotorControllers::Error::noErrorDetected){
		return false;
	}

	if (comIf->PDOReceive(adr,informationReceived, error)) {
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
bool SOLOMotorControllersKvaser::SetDeviceAddress(unsigned char deviceAddress, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetDeviceAddressInputValidation(deviceAddress, error))
	{
		return false;
	}
	soloUtils->ConvertToData((long)deviceAddress, informatrionToSend);
	return comIf->CANOpenTransmit(Address, Object_SetDeviceAddress, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the desired device address for a SOLO unit
  *				.The method refers to the Object Dictionary: 0x3001
  * @param[in]  deviceAddress  address want to set for board      
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetDeviceAddress(unsigned char deviceAddress)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetDeviceAddress(deviceAddress, error);
}

/**
  * @brief  This command sets the mode of the operation of SOLO
  *         in terms of operating in Analogue mode or Digital
  *				.The method refers to the Object Dictionary: 0x3002
  * @param[in] mode  enum that specify mode of the operation of SOLO  
  * @param[out]  error   pointer to an integer that specify result of function       
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetCommandMode(SOLOMotorControllers::CommandMode mode, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	soloUtils->ConvertToData((long)mode, informatrionToSend);
	return comIf->CANOpenTransmit(Address, Object_CommandMode, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the mode of the operation of SOLO
  *         in terms of operating in Analogue mode or Digital
  *				.The method refers to the Object Dictionary: 0x3002
  * @param[in] mode  enum that specify mode of the operation of SOLO      
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetCommandMode(SOLOMotorControllers::CommandMode mode)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetCommandMode(mode, error);
}

/**
  * @brief  This command defines the maximum allowed current into the motor in terms of Amps
  *				.The method refers to the Object Dictionary: 0x3003
  * @param[in] currentLimit  a float value between 0 to 32
  * @param[out]  error   pointer to an integer that specify result of function       
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetCurrentLimit(float currentLimit, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetCurrentLimitInputValidation(currentLimit, error))
	{
		return false;
	}
	soloUtils->ConvertToData(currentLimit, informatrionToSend);
	return comIf->CANOpenTransmit(Address, Object_CurrentLimit, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command defines the maximum allowed current into the motor in terms of Amps
  *				.The method refers to the Object Dictionary: 0x3003
  * @param[in] currentLimit  a float value between 0 to 32      
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetCurrentLimit(float currentLimit)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetCurrentLimit(currentLimit, error);
}

/**
  * @brief  This command sets the amount of desired current that acts in torque generation
  *				.The method refers to the Object Dictionary: 0x3004
  * @param[in] torqueReferenceIq  a float value between 0 to 32
  * @param[out]  error   pointer to an integer that specify result of function       
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetTorqueReferenceIq(float torqueReferenceIq, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetTorqueReferenceIqInputValidation(torqueReferenceIq, error))
	{
		return false;
	}
	soloUtils->ConvertToData(torqueReferenceIq, informatrionToSend);
	return comIf->CANOpenTransmit(Address, Object_TorqueReferenceIq, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the amount of desired current that acts in torque generation
  *				.The method refers to the Object Dictionary: 0x3004
  * @param[in] torqueReferenceIq  a float value between 0 to 32      
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetTorqueReferenceIq(float torqueReferenceIq)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetTorqueReferenceIq(torqueReferenceIq, error);
}

/**
  * @brief  This command defines the speed reference for SOLO once it’s in Digital Speed Mode
				.The method refers to the Object Dictionary: 0x3005
  * @param[in] speedReference  a long value between 0 to 30000
  * @param[out]  error   pointer to an integer that specify result of function       
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetSpeedReference(long speedReference, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetSpeedReferenceInputValidation(speedReference, error))
	{
		return false;
	}
	soloUtils->ConvertToData(speedReference, informatrionToSend);
	return comIf->CANOpenTransmit(Address, Object_SpeedReference, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command defines the speed reference for SOLO once it’s in Digital Speed Mode
				.The method refers to the Object Dictionary: 0x3005
  * @param[in] speedReference  a long value between 0 to 30000      
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetSpeedReference(long speedReference)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetSpeedReference(speedReference, error);
}

/**
  * @brief  This command defines the amount of power percentage during only
  *         Open-loop mode for 3-phase motors
				.The method refers to the Object Dictionary: 0x3006
  * @param[in] powerReference  a float value between 0 to 100
  * @param[out]  error   pointer to an integer that specify result of function       
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetPowerReference(float powerReference, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetPowerReferenceInputValidation(powerReference, error))
	{
		return false;
	}
	soloUtils->ConvertToData(powerReference, informatrionToSend);
	return comIf->CANOpenTransmit(Address, Object_PowerReference, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command defines the amount of power percentage during only
  *         Open-loop mode for 3-phase motors
				.The method refers to the Object Dictionary: 0x3006
  * @param[in] powerReference  a float value between 0 to 100      
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetPowerReference(float powerReference)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetPowerReference(powerReference, error);
}

/**
  * @brief  By putting 1 in the DATA section of a packet sent with this command, SOLO will start
            identifying the electrical parameters of the Motor connected
				.The method refers to the Object Dictionary: 0x3007
  * @param[in] powerReference  enum that specify Start or Stop of something in SOLO 
  * @param[out]  error   pointer to an integer that specify result of function       
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::MotorParametersIdentification(SOLOMotorControllers::Action identification, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	soloUtils->ConvertToData((long)identification, informatrionToSend);
	return comIf->CANOpenTransmit(Address, Object_MotorParametersIdentification, 0x00, informatrionToSend, error);

}

/**
  * @brief  By putting 1 in the DATA section of a packet sent with this command, SOLO will start
            identifying the electrical parameters of the Motor connected
				.The method refers to the Object Dictionary: 0x3007
  * @param[in] powerReference  enum that specify Start or Stop of something in SOLO   
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::MotorParametersIdentification(SOLOMotorControllers::Action identification)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::MotorParametersIdentification(identification, error);
}

/**
  * @brief  This command if the DATA is set at zero will stop the whole power and switching system
            connected to the motor and it will cut the current floating into the Motor from SOLO
				.The method refers to the Object Dictionary: 0x3008
  * @param[out]  error   pointer to an integer that specify result of function       
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::EmergencyStop(int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	return comIf->CANOpenTransmit(Address, Object_EmergencyStop, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command if the DATA is set at zero will stop the whole power and switching system
            connected to the motor and it will cut the current floating into the Motor from SOLO  
				.The method refers to the Object Dictionary: 0x3008
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::EmergencyStop()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::EmergencyStop(error);
}

/**
  * @brief  This command sets the output switching frequency of the whole power unit on the Motor
				.The method refers to the Object Dictionary: 0x3009
  * @param[in] outputPwmFrequencyKhz  switching frequencies in kHz. a long value between 8 to 80 
  * @param[out]  error   pointer to an integer that specify result of function     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetOutputPwmFrequencyKhz(long outputPwmFrequencyKhz, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetOutputPwmFrequencyKhzInputValidation(outputPwmFrequencyKhz, error))
	{
		return false;
	}
	soloUtils->ConvertToData(outputPwmFrequencyKhz, informatrionToSend);
	return comIf->CANOpenTransmit(Address, Object_OutputPwmFrequency, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the output switching frequency of the whole power unit on the Motor
				.The method refers to the Object Dictionary: 0x3009
  * @param[in] outputPwmFrequencyKhz  switching frequencies in kHz. a long value between 8 to 80      
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetOutputPwmFrequencyKhz(long outputPwmFrequencyKhz)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetOutputPwmFrequencyKhz(outputPwmFrequencyKhz, error);
}

/**
  * @brief  This command sets the Speed controller Kp Gain, and it will
  *         be functional only in Digital Closed-loop mode
				.The method refers to the Object Dictionary: 0x300A
  * @param[in] speedControllerKp  a float value between 0 to 300 
  * @param[out]  error   pointer to an integer that specify result of function     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetSpeedControllerKp(float speedControllerKp, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetSpeedControllerKpInputValidation(speedControllerKp, error))
	{
		return false;
	}
	soloUtils->ConvertToData(speedControllerKp, informatrionToSend);
	return comIf->CANOpenTransmit(Address, Object_SpeedControllerKp, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the Speed controller Kp Gain, and it will
  *         be functional only in Digital Closed-loop mode
				.The method refers to the Object Dictionary: 0x300A
  * @param[in] speedControllerKp  a float value between 0 to 300     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetSpeedControllerKp(float speedControllerKp)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetSpeedControllerKp(speedControllerKp, error);
}

/**
  * @brief  This command sets the Speed controller Ki gain, and it will
  *         be functional only in Digital Closed-loop mode
				.The method refers to the Object Dictionary: 0x300B
  * @param[in] speedControllerKi  a float value between 0 to 300 
  * @param[out]  error   pointer to an integer that specify result of function     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetSpeedControllerKi(float speedControllerKi, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetSpeedControllerKiInputValidation(speedControllerKi, error))
	{
		return false;
	}
	soloUtils->ConvertToData(speedControllerKi, informatrionToSend);
	return comIf->CANOpenTransmit(Address, Object_SpeedControllerKi, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the Speed controller Ki gain, and it will
  *         be functional only in Digital Closed-loop mode
				.The method refers to the Object Dictionary: 0x300B
  * @param[in] speedControllerKi  a float value between 0 to 300     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetSpeedControllerKi(float speedControllerKi)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetSpeedControllerKi(speedControllerKi, error);
}

/**
  * @brief  This command sets the direction of the rotation of the motor
  *         either to ClockWise rotation or to Counter Clockwise Rotation
				.The method refers to the Object Dictionary: 0x300C
  * @param[in] motorDirection  enum that specify the direction of the rotation of the motor 
  * @param[out]  error   pointer to an integer that specify result of function     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetMotorDirection(SOLOMotorControllers::Direction motorDirection, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	soloUtils->ConvertToData((long)motorDirection, informatrionToSend);
	return comIf->CANOpenTransmit(Address, Object_MotorDirection, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the direction of the rotation of the motor
  *         either to ClockWise rotation or to Counter Clockwise Rotation
				.The method refers to the Object Dictionary: 0x300C
  * @param[in] motorDirection  enum that specify the direction of the rotation of the motor    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetMotorDirection(SOLOMotorControllers::Direction motorDirection)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetMotorDirection(motorDirection, error);
}

/**
  * @brief  This command sets the amount of the Phase or Armature resistance
  *         for 3-phase or DC Brushed motors respectively
				.The method refers to the Object Dictionary: 0x300D
  * @param[in] motorResistance  a float value between 0.001 to 50
  * @param[out]  error   pointer to an integer that specify result of function     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetMotorResistance(float motorResistance, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetMotorResistanceInputValidation(motorResistance, error))
	{
		return false;
	}
	soloUtils->ConvertToData(motorResistance, informatrionToSend);
	return comIf->CANOpenTransmit(Address, Object_MotorResistance, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the amount of the Phase or Armature resistance
  *         for 3-phase or DC Brushed motors respectively
				.The method refers to the Object Dictionary: 0x300D
  * @param[in] motorResistance  a float value between 0.001 to 50    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetMotorResistance(float motorResistance)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetMotorResistance(motorResistance, error);
}

/**
  * @brief  This command sets the amount of the Phase or Armature Inductance
  *         for 3-phase or DC Brushed motors respectively
				.The method refers to the Object Dictionary: 0x300E
  * @param[in] motorInductance  a float value between 0.00005 to 0.2
  * @param[out]  error   pointer to an integer that specify result of function     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetMotorInductance(float motorInductance, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetMotorInductanceInputValidation(motorInductance, error))
	{
		return false;
	}
	soloUtils->ConvertToData(motorInductance, informatrionToSend);
	return comIf->CANOpenTransmit(Address, Object_MotorInductance, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the amount of the Phase or Armature Inductance
  *         for 3-phase or DC Brushed motors respectively
				.The method refers to the Object Dictionary: 0x300E
  * @param[in] motorInductance  a float value between 0.00005 to 0.2    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetMotorInductance(float motorInductance)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetMotorInductance(motorInductance, error);
}

/**
  * @brief  This command sets the number of the Poles of a 3-phase motor commissioned with SOLO
				.The method refers to the Object Dictionary: 0x300F
  * @param[in] motorPolesCounts  a long value between 1 to 254   
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetMotorPolesCounts(long motorPolesCounts, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetMotorPolesCountsInputValidation(motorPolesCounts, error))
	{
		return false;
	}

	soloUtils->ConvertToData(motorPolesCounts, informatrionToSend);
	return comIf->CANOpenTransmit(Address, Object_MotorPolesCounts, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the number of the Poles of a 3-phase motor commissioned with SOLO
				.The method refers to the Object Dictionary: 0x300F
  * @param[in] motorPolesCounts  a long value between 1 to 254    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetMotorPolesCounts(long motorPolesCounts)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetMotorPolesCounts(motorPolesCounts, error);
}

/**
  * @brief  This command sets the pre-quad number of physical lines of an 
  *         incremental encoder engraved on its disk
				.The method refers to the Object Dictionary: 0x3010
  * @param[in] incrementalEncoderLines  a long value between 1 to 200000   
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetIncrementalEncoderLines(long incrementalEncoderLines, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetIncrementalEncoderLinesInputValidation(incrementalEncoderLines, error))
	{
		return false;
	}
	soloUtils->ConvertToData(incrementalEncoderLines, informatrionToSend);
	return comIf->CANOpenTransmit(Address, Object_IncrementalEncoderLines, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the pre-quad number of physical lines of an 
  *         incremental encoder engraved on its disk
				.The method refers to the Object Dictionary: 0x3010
  * @param[in] incrementalEncoderLines  a long value between 1 to 200000    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetIncrementalEncoderLines(long incrementalEncoderLines)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetIncrementalEncoderLines(incrementalEncoderLines, error);
}

/**
  * @brief  This command sets the allowed speed during trajectory following
  *         in closed-loop position controlling mode
				.The method refers to the Object Dictionary: 0x3011
  * @param[in] speedLimit  a long value between 0 to 30000   
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetSpeedLimit(long speedLimit, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetSpeedLimitInputValidation(speedLimit, error))
	{
		return false;
	}
	soloUtils->ConvertToData(speedLimit, informatrionToSend);
	return comIf->CANOpenTransmit(Address, Object_SpeedLimit, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the allowed speed during trajectory following
  *         in closed-loop position controlling mode
				.The method refers to the Object Dictionary: 0x3011
  * @param[in] speedLimit  a long value between 0 to 30000    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetSpeedLimit(long speedLimit)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetSpeedLimit(speedLimit, error);
}

/**
  * @brief  This command sets the type of the feedback control SOLO has to operate
				.The method refers to the Object Dictionary: 0x3013
  * @param[in] mode  enum that specify the type of the feedback control SOLO 
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetFeedbackControlMode(SOLOMotorControllers::FeedbackControlMode mode, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	soloUtils->ConvertToData((long)mode, informatrionToSend);
	return comIf->CANOpenTransmit(Address, Object_FeedbackControlMode, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the type of the feedback control SOLO has to operate
				.The method refers to the Object Dictionary: 0x3013
  * @param[in] mode  enum that specify the type of the feedback control SOLO  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetFeedbackControlMode(SOLOMotorControllers::FeedbackControlMode mode)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetFeedbackControlMode(mode, error);
}

/**
  * @brief  This command resets SOLO to its factory setting to all the default parameters  
				.The method refers to the Object Dictionary: 0x3014
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::ResetFactory(int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x01 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	return comIf->CANOpenTransmit(Address, Object_ResetFactory, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command resets SOLO to its factory setting to all the default parameters  
				.The method refers to the Object Dictionary: 0x3014
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::ResetFactory()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::ResetFactory(error);
}

bool SOLOMotorControllersKvaser::ResetDeviceAddress(int& error)
{
	return false;
}
bool SOLOMotorControllersKvaser::ResetDeviceAddress()
{
	return false;
}

/**
  * @brief  This command sets the Motor type that is connected to SOLO in Digital Mode
				.The method refers to the Object Dictionary: 0x3015
  * @param[in] motorType  enum that specify the Motor type that is connected to SOLO in Digital Mode
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetMotorType(SOLOMotorControllers::MotorType motorType, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	soloUtils->ConvertToData((long)motorType, informatrionToSend);
	return comIf->CANOpenTransmit(Address, Object_MotorType, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the Motor type that is connected to SOLO in Digital Mode
				.The method refers to the Object Dictionary: 0x3015
  * @param[in] motorType  enum that specify the Motor type that is connected to SOLO in Digital Mode
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetMotorType(SOLOMotorControllers::MotorType motorType)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetMotorType(motorType, error);
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
bool SOLOMotorControllersKvaser::SetControlMode(SOLOMotorControllers::ControlMode controlMode, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	soloUtils->ConvertToData((long)controlMode, informatrionToSend);
	return comIf->CANOpenTransmit(Address, Object_ControlMode, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the Control Mode in terms of Torque,
  *         Speed or Position only in Digital Mode
				.The method refers to the Object Dictionary: 0x3016
  * @param[in] controlMode  enum that specify the Control Mode in terms of Torque,
  *                       Speed or Position only in Digital Mode
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetControlMode(SOLOMotorControllers::ControlMode controlMode)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetControlMode(controlMode, error);
}

/**
  * @brief  This command sets the value for Current Controller Kp or proportional gain
				.The method refers to the Object Dictionary: 0x3017
  * @param[in] currentControllerKp  a float value between 0 to 16000  
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetCurrentControllerKp(float currentControllerKp, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetCurrentControllerKpInputValidation(currentControllerKp, error))
	{
		return false;
	}
	soloUtils->ConvertToData(currentControllerKp, informatrionToSend);
	return comIf->CANOpenTransmit(Address, Object_CurrentControllerKp, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the value for Current Controller Kp or proportional gain
				.The method refers to the Object Dictionary: 0x3017
  * @param[in] currentControllerKp  a float value between 0 to 16000   
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetCurrentControllerKp(float currentControllerKp)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetCurrentControllerKp(currentControllerKp, error);
}

/**
  * @brief  This command sets the value for Current Controller Ki or integral gain
				.The method refers to the Object Dictionary: 0x3018
  * @param[in] motorInductance  a float value between 0 to 16000  
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetCurrentControllerKi(float currentControllerKi, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetCurrentControllerKiInputValidation(currentControllerKi, error))
	{
		return false;
	}
	soloUtils->ConvertToData(currentControllerKi, informatrionToSend);
	return comIf->CANOpenTransmit(Address, Object_CurrentControllerKi, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the value for Current Controller Ki or integral gain
				.The method refers to the Object Dictionary: 0x3018
  * @param[in] motorInductance  a float value between 0 to 16000    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetCurrentControllerKi(float currentControllerKi)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetCurrentControllerKi(currentControllerKi, error);
}
bool SOLOMotorControllersKvaser::SetMonitoringMode(bool mode, int& error)
{
	return false;
}
bool SOLOMotorControllersKvaser::SetMonitoringMode(bool mode)
{
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
bool SOLOMotorControllersKvaser::SetMagnetizingCurrentIdReference(float magnetizingCurrentIdReference, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetMagnetizingCurrentIdReferenceInputValidation(magnetizingCurrentIdReference, error))
	{
		return false;
	}
	soloUtils->ConvertToData(magnetizingCurrentIdReference, informatrionToSend);
	return comIf->CANOpenTransmit(Address, Object_MagnetizingCurrentIdReference, 0x00, informatrionToSend, error);

}

/**
  * @brief  depending on the Motor type: in case of BLDC or PMSM motors Sets the Field
  *         Weakening current reference to help the motor reaching speeds higher than
  *         nominal values and in case of AC Induction Motors Sets the desired magnetizing
  *         current (Id) required for controlling ACIM motors in FOC in Amps 
				.The method refers to the Object Dictionary: 0x301A
  * @param[in] magnetizingCurrentIdReference  a float value between 0 to 32    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetMagnetizingCurrentIdReference(float magnetizingCurrentIdReference)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetMagnetizingCurrentIdReference(magnetizingCurrentIdReference, error);
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
bool SOLOMotorControllersKvaser::SetPositionReference(long positionReference, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetPositionReferenceInputValidation(positionReference, error))
	{
		return false;
	}
	soloUtils->ConvertToData(positionReference, informatrionToSend);
	return comIf->CANOpenTransmit(Address, Object_PositionReference, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the desired Position reference in terms of quadrature
  *         pulses while SOLO operates with the Incremental Encoders or in terms of
  *         pulses while while SOLO operates with Hall sensors
				.The method refers to the Object Dictionary: 0x301B
  * @param[in] positionReference  a long value between -2,147,483,647 to 2,147,483,647    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetPositionReference(long positionReference)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetPositionReference(positionReference, error);
}

/**
  * @brief  This command sets the value for Position Controller Kp or proportional gain
				.The method refers to the Object Dictionary: 0x301C
  * @param[in] positionControllerKp  a float value between 0 to 16000  
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetPositionControllerKp(float positionControllerKp, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetPositionControllerKpInputValidation(positionControllerKp, error))
	{
		return false;
	}
	soloUtils->ConvertToData(positionControllerKp, informatrionToSend);
	return comIf->CANOpenTransmit(Address, Object_PositionControllerKp, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the value for Position Controller Kp or proportional gain
				.The method refers to the Object Dictionary: 0x301C
  * @param[in] positionControllerKp  a float value between 0 to 16000   
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetPositionControllerKp(float positionControllerKp)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetPositionControllerKp(positionControllerKp, error);
}

/**
  * @brief  This command sets the value for Position Controller Ki or integrator gain
				.The method refers to the Object Dictionary: 0x301D
  * @param[in] positionControllerKi  a float value between 0 to 16000   
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetPositionControllerKi(float positionControllerKi, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetPositionControllerKiInputValidation(positionControllerKi, error))
	{
		return false;
	}
	soloUtils->ConvertToData(positionControllerKi, informatrionToSend);
	return comIf->CANOpenTransmit(Address, Object_PositionControllerKi, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the value for Position Controller Ki or integrator gain
				.The method refers to the Object Dictionary: 0x301D
  * @param[in] positionControllerKi  a float value between 0 to 16000    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetPositionControllerKi(float positionControllerKi)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetPositionControllerKi(positionControllerKi, error);
}

/**
  * @brief  This command resets the position counter back to zero  
				.The method refers to the Object Dictionary: 0x301F
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::ResetPositionToZero(int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x01 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	return comIf->CANOpenTransmit(Address, Object_ResetPositionToZero, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command resets the position counter back to zero    
				.The method refers to the Object Dictionary: 0x301F
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::ResetPositionToZero()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::ResetPositionToZero(error);
}

/**
  * @brief  This command overwrites the reported errors in Error Register
  *         reported with command code of "0xA1"  
				.The method refers to the Object Dictionary: 0x3020
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::OverwriteErrorRegister(int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	return comIf->CANOpenTransmit(Address, Object_OverwriteErrorRegister, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command overwrites the reported errors in Error Register
  *         reported with command code of "0xA1" 
				.The method refers to the Object Dictionary: 0x3020
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::OverwriteErrorRegister()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::OverwriteErrorRegister(error);
}

/**
  * @brief  This command sets the observer gain for the Non-linear observer
  *         that estimates the speed and angle of a BLDC or PMSM once the 
  *         motor type is selected as normal BLDC-PMSM
				.The method refers to the Object Dictionary: 0x3021
  * @param[in] observerGain  a float value between 0.01 to 1000  
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetObserverGainBldcPmsm(float observerGain, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetObserverGainBldcPmsmInputValidation(observerGain, error))
	{
		return false;
	}
	soloUtils->ConvertToData(observerGain, informatrionToSend);
	return comIf->CANOpenTransmit(Address, Object_ObserverGainBldcPmsm, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the observer gain for the Non-linear observer
  *         that estimates the speed and angle of a BLDC or PMSM once the 
  *         motor type is selected as normal BLDC-PMSM
				.The method refers to the Object Dictionary: 0x3021
  * @param[in] observerGain  a float value between 0.01 to 1000   
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetObserverGainBldcPmsm(float observerGain)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetObserverGainBldcPmsm(observerGain, error);
}

/**
  * @brief  This command sets the observer gain for the Non-linear observer that
  *         estimates the speed and angle of a BLDC or PMSM once the motor type
  *         is selected as ultra-fast BLDC-PMSM
				.The method refers to the Object Dictionary: 0x3022
  * @param[in] observerGain  a float value between 0.01 to 1000   
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetObserverGainBldcPmsmUltrafast(float observerGain, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetObserverGainBldcPmsmUltrafastInputValidation(observerGain, error))
	{
		return false;
	}
	soloUtils->ConvertToData(observerGain, informatrionToSend);
	return comIf->CANOpenTransmit(Address, Object_ObserverGainBldcPmsmUltrafast, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the observer gain for the Non-linear observer that
  *         estimates the speed and angle of a BLDC or PMSM once the motor type
  *         is selected as ultra-fast BLDC-PMSM
				.The method refers to the Object Dictionary: 0x3022
  * @param[in] observerGain  a float value between 0.01 to 1000    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetObserverGainBldcPmsmUltrafast(float observerGain)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetObserverGainBldcPmsmUltrafast(observerGain, error);
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
bool SOLOMotorControllersKvaser::SetObserverGainDc(float observerGain, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetObserverGainDcInputValidation(observerGain, error))
	{
		return false;
	}
	soloUtils->ConvertToData(observerGain, informatrionToSend);
	return comIf->CANOpenTransmit(Address, Object_ObserverGainDc, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the observer gain for the Non-linear observer
  *         that estimates the speed of a DC brushed once the motor type 
  *         is selected as DC brushed
				.The method refers to the Object Dictionary: 0x3023
  * @param[in] observerGain  a float value between 0.01 to 1000    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetObserverGainDc(float observerGain)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetObserverGainDc(observerGain, error);
}

/**
  * @brief  This command sets how fast the observer should operate once
  *         SOLO is in sensorless mode with normal BLDC-PMSM selected as the Motor type
				.The method refers to the Object Dictionary: 0x3024
  * @param[in] filterGain  a float value between 0.01 to 16000 
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetFilterGainBldcPmsm(float filterGain, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetFilterGainBldcPmsmInputValidation(filterGain, error))
	{
		return false;
	}
	soloUtils->ConvertToData(filterGain, informatrionToSend);
	return comIf->CANOpenTransmit(Address, Object_FilterGainBldcPmsm, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets how fast the observer should operate once
  *         SOLO is in sensorless mode with normal BLDC-PMSM selected as the Motor type
				.The method refers to the Object Dictionary: 0x3024
  * @param[in] filterGain  a float value between 0.01 to 16000   
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetFilterGainBldcPmsm(float filterGain)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetFilterGainBldcPmsm(filterGain, error);
}

/**
  * @brief  This command sets how fast the observer should operate once SOLO
  *         is in sensorless mode with ultra-fast BLDC-PMSM selected as the Motor type
				.The method refers to the Object Dictionary: 0x3025
  * @param[in] filterGain  a float value between 0.01 to 16000  
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetFilterGainBldcPmsmUltrafast(float filterGain, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetFilterGainBldcPmsmUltrafastInputValidation(filterGain, error))
	{
		return false;
	}
	soloUtils->ConvertToData(filterGain, informatrionToSend);
	return comIf->CANOpenTransmit(Address, Object_FilterGainBldcPmsmUltrafast, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets how fast the observer should operate once SOLO
  *         is in sensorless mode with ultra-fast BLDC-PMSM selected as the Motor type
				.The method refers to the Object Dictionary: 0x3025
  * @param[in] filterGain  a float value between 0.01 to 16000    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetFilterGainBldcPmsmUltrafast(float filterGain)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetFilterGainBldcPmsmUltrafast(filterGain, error);
}

/**
  * @brief  This command sets the baud-rate of the UART line
				.The method refers to the Object Dictionary: 0x3026
  * @param[in] baudrate  enum that specify the baud-rate of the UART line 
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetUartBaudrate(SOLOMotorControllers::UartBaudrate baudrate, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	soloUtils->ConvertToData((long)baudrate, informatrionToSend);
	return comIf->CANOpenTransmit(Address, Object_UartBaudrate, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the baud-rate of the UART line
				.The method refers to the Object Dictionary: 0x3026
  * @param[in] baudrate  enum that specify the baud-rate of the UART line    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetUartBaudrate(SOLOMotorControllers::UartBaudrate baudrate)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetUartBaudrate(baudrate, error);
}

/**
  * @brief  This command starts or stops the process of sensor calibration
				.The method refers to the Object Dictionary: 0x3027
  * @param[in] calibrationAction  enum that specify the process of sensor calibration 
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SensorCalibration(SOLOMotorControllers::PositionSensorCalibrationAction calibrationAction, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	soloUtils->ConvertToData((long)calibrationAction, informatrionToSend);
	return comIf->CANOpenTransmit(Address, Object_SensorCalibration, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command starts or stops the process of sensor calibration
				.The method refers to the Object Dictionary: 0x3027
  * @param[in] calibrationAction  enum that specify the process of sensor calibration     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SensorCalibration(SOLOMotorControllers::PositionSensorCalibrationAction calibrationAction)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SensorCalibration(calibrationAction, error);
}

/**
  * @brief  This command sets the per-unit offset identified after sensor calibration
  *         for Encoder or Hall sensors in C.C.W direction
				.The method refers to the Object Dictionary: 0x3028
  * @param[in] encoderHallOffset  a float value between 0.0 to 1.0  
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetEncoderHallCcwOffset(float encoderHallOffset, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetEncoderHallCcwOffsetInputValidation(encoderHallOffset, error))
	{
		return false;
	}
	soloUtils->ConvertToData(encoderHallOffset, informatrionToSend);
	return comIf->CANOpenTransmit(Address, Object_EncoderHallCcwOffset, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the per-unit offset identified after sensor calibration
  *         for Encoder or Hall sensors in C.C.W direction
				.The method refers to the Object Dictionary: 0x3028
  * @param[in] encoderHallOffset  a float value between 0.0 to 1.0    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetEncoderHallCcwOffset(float encoderHallOffset)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetEncoderHallCcwOffset(encoderHallOffset, error);
}

/**
  * @brief  This command sets the per-unit offset identified after sensor calibration
  *         for Encoder or Hall sensors in C.W direction
				.The method refers to the Object Dictionary: 0x3029
  * @param[in] encoderHallOffset  a float value between 0.0 to 1.0   
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetEncoderHallCwOffset(float encoderHallOffset, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetEncoderHallCwOffsetInputValidation(encoderHallOffset, error))
	{
		return false;
	}
	soloUtils->ConvertToData(encoderHallOffset, informatrionToSend);
	return comIf->CANOpenTransmit(Address, Object_EncoderHallCwOffset, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the per-unit offset identified after sensor calibration
  *         for Encoder or Hall sensors in C.W direction
				.The method refers to the Object Dictionary: 0x3029
  * @param[in] encoderHallOffset  a float value between 0.0 to 1.0     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetEncoderHallCwOffset(float encoderHallOffset)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetEncoderHallCwOffset(encoderHallOffset, error);
}

/**
  * @brief  This command defines the acceleration value of the Speed for speed controller
  *         both in Analogue and Digital modes in Revolution per square seconds
				.The method refers to the Object Dictionary: 0x302A
  * @param[in] speedAccelerationValue  a float value between 0 to 1600  
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetSpeedAccelerationValue(float speedAccelerationValue, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetSpeedAccelerationValueInputValidation(speedAccelerationValue, error))
	{
		return false;
	}
	soloUtils->ConvertToData(speedAccelerationValue, informatrionToSend);
	return comIf->CANOpenTransmit(Address, Object_SpeedAccelerationValue, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command defines the acceleration value of the Speed for speed controller
  *         both in Analogue and Digital modes in Revolution per square seconds
				.The method refers to the Object Dictionary: 0x302A
  * @param[in] speedAccelerationValue  a float value between 0 to 1600   
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetSpeedAccelerationValue(float speedAccelerationValue)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetSpeedAccelerationValue(speedAccelerationValue, error);
}

/**
  * @brief  This command defines the deceleration value of the Speed for speed controller
  *         both in Analogue and Digital modes in Revolution per square seconds
				.The method refers to the Object Dictionary: 0x302B
  * @param[in] speedDecelerationValue  a float value between 0 to 1600   
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetSpeedDecelerationValue(float speedDecelerationValue, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetSpeedDecelerationValueInputValidation(speedDecelerationValue, error))
	{
		return false;
	}
	soloUtils->ConvertToData(speedDecelerationValue, informatrionToSend);
	return comIf->CANOpenTransmit(Address, Object_SpeedDecelerationValue, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command defines the deceleration value of the Speed for speed controller
  *         both in Analogue and Digital modes in Revolution per square seconds
				.The method refers to the Object Dictionary: 0x302B
  * @param[in] speedDecelerationValue  a float value between 0 to 1600    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetSpeedDecelerationValue(float speedDecelerationValue)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetSpeedDecelerationValue(speedDecelerationValue, error);
}

/**
  * @brief  This command sets the baud rate of CAN bus in CANOpen network
				.The method refers to the Object Dictionary: 0x302C
  * @param[in] canbusBaudrate  enum that specify the baud rate of CAN bus in CANOpen network 
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetCanbusBaudrate(CanbusBaudrate canbusBaudrate, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	soloUtils->ConvertToData((long)canbusBaudrate, informatrionToSend);
	return comIf->CANOpenTransmit(Address, Object_CanbusBaudrate, 0x00, informatrionToSend, error);

}

/**
  * @brief  This command sets the baud rate of CAN bus in CANOpen network
				.The method refers to the Object Dictionary: 0x302C
  * @param[in] canbusBaudrate  enum that specify the baud rate of CAN bus in CANOpen network    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetCanbusBaudrate(CanbusBaudrate canbusBaudrate)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetCanbusBaudrate(canbusBaudrate, error);
}

/**
  * @brief  This command defines the resolution of the speed at S/T input
  *           while SOLO operates in Analogue mode
  *           .The method refers to the Object Dictionary: 0x303E
  * @param[in] divisionCoefficient  a long value    
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetAnalogueSpeedResolutionDivisionCoefficient(long divisionCoefficient, int &error)
{	
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetAnalogueSpeedResolutionDivisionCoefficientInputValidation(divisionCoefficient, error))
	{
		return false;
	}
    soloUtils->ConvertToData((long) divisionCoefficient, informatrionToSend);
	return comIf->CANOpenTransmit(Address, Object_ASRDC, 0x00, informatrionToSend, error);
}

/**
  * @brief  This command defines the resolution of the speed at S/T input
  *           while SOLO operates in Analogue mode
  *           .The method refers to the Object Dictionary: 0x303E
  * @param[in] divisionCoefficient  a long value     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetAnalogueSpeedResolutionDivisionCoefficient(long divisionCoefficient)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersKvaser::SetAnalogueSpeedResolutionDivisionCoefficient(divisionCoefficient, error);
}

/**
  * @brief  This command defines the type of the Motion Profile that is 
  *           being used in Speed or Position Modes
  *           .The method refers to the Object Dictionary: 0x3040
  * @param[in] motionProfileMode enum that specify the type of the Motion Profile   
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetMotionProfileMode( MotionProfileMode motionProfileMode, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
    soloUtils->ConvertToData((long) motionProfileMode, informatrionToSend);
	return comIf->CANOpenTransmit(Address, Object_MotionProfileMode, 0x00, informatrionToSend, error);
}

/**
  * @brief  This command defines the type of the Motion Profile that is 
  *           being used in Speed or Position Modes
  *           .The method refers to the Object Dictionary: 0x3040
  * @param[in] motionProfileMode enum that specify the type of the Motion Profile    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetMotionProfileMode( MotionProfileMode motionProfileMode)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersKvaser::SetMotionProfileMode(motionProfileMode, error);
}

/**
  * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles 
				.The method refers to the Object Dictionary: 0x3041
  * @param[in] MotionProfileVariable1 a float value   
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetMotionProfileVariable1(float MotionProfileVariable1, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetMotionProfileVariable1InputValidation(MotionProfileVariable1, error))
	{
		return false;
	}
    soloUtils->ConvertToData((float) MotionProfileVariable1, informatrionToSend);
	return comIf->CANOpenTransmit(Address, Object_MotionProfileVariable1, 0x00, informatrionToSend, error);
}

/**
  * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
				.The method refers to the Object Dictionary: 0x3041
  * @param[in] MotionProfileVariable1 a float value     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetMotionProfileVariable1(float MotionProfileVariable1)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersKvaser::SetMotionProfileVariable1(MotionProfileVariable1, error);
}

/**
  * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles 
				.The method refers to the Object Dictionary: 0x3042
  * @param[in] MotionProfileVariable2 a float value   
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetMotionProfileVariable2(float MotionProfileVariable2, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetMotionProfileVariable2InputValidation(MotionProfileVariable2, error))
	{
		return false;
	}
    soloUtils->ConvertToData((float) MotionProfileVariable2, informatrionToSend);
	return comIf->CANOpenTransmit(Address, Object_MotionProfileVariable2, 0x00, informatrionToSend, error);
}

/**
  * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles 
				.The method refers to the Object Dictionary: 0x3042
  * @param[in] MotionProfileVariable2 a float value     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetMotionProfileVariable2(float MotionProfileVariable2)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersKvaser::SetMotionProfileVariable2(MotionProfileVariable2, error);
}

/**
  * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles 
				.The method refers to the Object Dictionary: 0x3043
  * @param[in] MotionProfileVariable3 a float value   
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetMotionProfileVariable3(float MotionProfileVariable3, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetMotionProfileVariable3InputValidation(MotionProfileVariable3, error))
	{
		return false;
	}
    soloUtils->ConvertToData((float) MotionProfileVariable3, informatrionToSend);
	return comIf->CANOpenTransmit(Address, Object_MotionProfileVariable3, 0x00, informatrionToSend, error);
}

/**
  * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
				.The method refers to the Object Dictionary: 0x3043
  * @param[in] MotionProfileVariable3 a float value     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetMotionProfileVariable3(float MotionProfileVariable3)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersKvaser::SetMotionProfileVariable3(MotionProfileVariable3, error);
}

/**
  * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles 
				.The method refers to the Object Dictionary: 0x3044
  * @param[in] MotionProfileVariable4 a float value   
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetMotionProfileVariable4(float MotionProfileVariable4, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetMotionProfileVariable4InputValidation(MotionProfileVariable4, error))
	{
		return false;
	}
    soloUtils->ConvertToData((float) MotionProfileVariable4, informatrionToSend);
	return comIf->CANOpenTransmit(Address, Object_MotionProfileVariable4, 0x00, informatrionToSend, error);
}

/**
  * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles 
				.The method refers to the Object Dictionary: 0x3044
  * @param[in] MotionProfileVariable4 a float value     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetMotionProfileVariable4(float MotionProfileVariable4)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersKvaser::SetMotionProfileVariable4(MotionProfileVariable4, error);
}

/**
  * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles 
				.The method refers to the Object Dictionary: 0x3045
  * @param[in] MotionProfileVariable5 a float value   
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetMotionProfileVariable5(float MotionProfileVariable5, int &error)
{
    uint8_t informatrionToSend[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetMotionProfileVariable5InputValidation(MotionProfileVariable5, error))
	{
		return false;
	}
    soloUtils->ConvertToData((float) MotionProfileVariable5, informatrionToSend);
	return comIf->CANOpenTransmit(Address, Object_MotionProfileVariable5, 0x00, informatrionToSend, error);
}

/**
  * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
				.The method refers to the Object Dictionary: 0x3045
  * @param[in] MotionProfileVariable5 a float value     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetMotionProfileVariable5(float MotionProfileVariable5)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersKvaser::SetMotionProfileVariable5(MotionProfileVariable5, error);
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
bool SOLOMotorControllersKvaser::SetPdoPositionReference(long positionReference, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetPositionReferenceInputValidation(positionReference, error))
	{
		return false;
	}
	return SOLOMotorControllersKvaser::SetPdoParameterValue(PdoParameterName::positionReference,positionReference, error);
}

/**
  * @brief  This PDO command sets the desired Position reference in terms of quadrature
  *         pulses while SOLO operates with the Incremental Encoders or in terms of
  *         pulses while while SOLO operates with Hall sensors
  *				.The method refers to the Object Dictionary: 0x1414
  * @param[in] positionReference  a long value between -2,147,483,647 to 2,147,483,647    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetPdoPositionReference(long positionReference)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetPdoPositionReference(positionReference, error);
}

/**
  * @brief  This PDO command defines the speed reference for SOLO once it’s in Digital Speed Mode
  *				.The method refers to the Object Dictionary: 0x1415
  * @param[in] speedReference  a long value defining the speed (only positive)
  * @param[out]  error   pointer that specify result of function       
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetPdoSpeedReference(long speedReference, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetSpeedReferenceInputValidation(speedReference, error))
	{
		return false;
	}
	
	return SOLOMotorControllersKvaser::SetPdoParameterValue(PdoParameterName::speedReference,speedReference, error);
}

/**
  * @brief  This PDO command defines the speed reference for SOLO once it’s in Digital Speed Mode
  *				.The method refers to the Object Dictionary: 0x1415
  * @param[in] speedReference  a long value defining the speed (only positive)     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetPdoSpeedReference(long speedReference)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetPdoSpeedReference(speedReference, error);
}

/**
  * @brief  This PDO command sets the amount of desired current that acts in torque generation
  *				.The method refers to the Object Dictionary: 0x1416
  * @param[in] torqueReferenceIq  a float value between 0 to 32
  * @param[out]  error   pointer to an integer that specify result of function       
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetPdoTorqueReferenceIq(float torqueReferenceIq, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetTorqueReferenceIqInputValidation(torqueReferenceIq, error))
	{
		return false;
	}
	return SOLOMotorControllersKvaser::SetPdoParameterValue(PdoParameterName::torqueReferenceIq,torqueReferenceIq, error);
}

/**
  * @brief  This PDO command sets the amount of desired current that acts in torque generation
  *				.The method refers to the Object Dictionary: 0x1416
  * @param[in] torqueReferenceIq  a float value between 0 to 32      
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetPdoTorqueReferenceIq(float torqueReferenceIq)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetPdoTorqueReferenceIq(torqueReferenceIq, error);
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
bool SOLOMotorControllersKvaser::SetPdoMagnetizingCurrentIdReference(float magnetizingCurrentIdReference, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetMagnetizingCurrentIdReferenceInputValidation(magnetizingCurrentIdReference, error))
	{
		return false;
	}
	return SOLOMotorControllersKvaser::SetPdoParameterValue(PdoParameterName::magnetizingCurrentIdReference,magnetizingCurrentIdReference, error);
}

/**
  * @brief  this PDO command depending on the Motor type: in case of BLDC or PMSM motors Sets the Field
  *         Weakening current reference to help the motor reaching speeds higher than
  *         nominal values and in case of AC Induction Motors Sets the desired magnetizing
  *         current (Id) required for controlling ACIM motors in FOC in Amps 
  *				.The method refers to the Object Dictionary: 0x1417
  * @param[in] magnetizingCurrentIdReference  a float value between 0 to 32    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetPdoMagnetizingCurrentIdReference(float magnetizingCurrentIdReference)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetPdoMagnetizingCurrentIdReference(magnetizingCurrentIdReference, error);
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
bool SOLOMotorControllersKvaser::SetPdoControlMode(SOLOMotorControllers::ControlMode controlMode, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetPdoParameterValue(PdoParameterName::controlMode,(long)controlMode, error);
}

/**
  * @brief  This PDO command sets the Control Mode in terms of Torque,
  *         Speed or Position only in Digital Mode
  *				.The method refers to the Object Dictionary: 0x1418
  * @param[in] controlMode  enum that specify the Control Mode in terms of Torque,
  *                       Speed or Position only in Digital Mode
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetPdoControlMode(SOLOMotorControllers::ControlMode controlMode)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetPdoControlMode(controlMode, error);
}

/**
  * @brief  This PDO command sets the direction of the rotation of the motor
  *         either to ClockWise rotation or to Counter Clockwise Rotation
  *				.The method refers to the Object Dictionary: 0x1419
  * @param[in] motorDirection  enum that specify the direction of the rotation of the motor 
  * @param[out]  error   pointer to an integer that specify result of function     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetPdoMotorDirection(SOLOMotorControllers::Direction motorDirection, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetPdoParameterValue(PdoParameterName::motorDirection,(long)motorDirection, error);
}

/**
  * @brief  This PDO commands sets the direction of the rotation of the motor
  *         either to ClockWise rotation or to Counter Clockwise Rotation
  *				.The method refers to the Object Dictionary: 0x1419
  * @param[in] motorDirection  enum that specify the direction of the rotation of the motor    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersKvaser::SetPdoMotorDirection(SOLOMotorControllers::Direction motorDirection)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetPdoMotorDirection(motorDirection, error);
}

////---------------------Read---------------------
long SOLOMotorControllersKvaser::GetReadErrorRegister(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (comIf->CANOpenReceive(Address, Object_ReadErrorRegister, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}
long SOLOMotorControllersKvaser::GetReadErrorRegister()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetReadErrorRegister(error);
}
long SOLOMotorControllersKvaser::GetGuardTime(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_GuardTime, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}
long SOLOMotorControllersKvaser::GetGuardTime()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetGuardTime(error);
}
long SOLOMotorControllersKvaser::GetLifeTimeFactor(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_LifeTimeFactor, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}
long SOLOMotorControllersKvaser::GetLifeTimeFactor()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetLifeTimeFactor(error);
}
long SOLOMotorControllersKvaser::GetProducerHeartbeatTime(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_ProducerHeartbeatTime, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}
long SOLOMotorControllersKvaser::GetProducerHeartbeatTime()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetProducerHeartbeatTime(error);
}

/**
  * @brief  This command reads the device address connected on the line 
				.The method refers to the Object Dictionary: 0x3001
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long device address connected on the line
  */
long SOLOMotorControllersKvaser::GetDeviceAddress(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_SetDeviceAddress, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}

/**
  * @brief  This command reads the device address connected on the line  
				.The method refers to the Object Dictionary: 0x3001
  * @retval long device address connected on the line
  */
long SOLOMotorControllersKvaser::GetDeviceAddress()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetDeviceAddress(error);
}

/**
  * @brief  This command reads the phase-A voltage of the motor connected to the
  *         "A" pin output of SOLO for 3-phase Motors 
				.The method refers to the Object Dictionary: 0x302D
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float phase-A voltage of the motor between -60 to 60
  */
float SOLOMotorControllersKvaser::GetPhaseAVoltage(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_PhaseAVoltage, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the phase-A voltage of the motor connected to the
  *         "A" pin output of SOLO for 3-phase Motors  
				.The method refers to the Object Dictionary: 0x302D
  * @retval float phase-A voltage of the motor between -60 to 60
  */
float SOLOMotorControllersKvaser::GetPhaseAVoltage()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetPhaseAVoltage(error);
}

/**
  * @brief  This command reads the phase-B voltage of the motor connected to the
  *         "B" pin output of SOLO for 3-phase Motors  
				.The method refers to the Object Dictionary: 0x302E
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float 0 phase-A voltage of the motor between -60 to 60
  */
float SOLOMotorControllersKvaser::GetPhaseBVoltage(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_PhaseBVoltage, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the phase-B voltage of the motor connected to the
  *         "B" pin output of SOLO for 3-phase Motors   
				.The method refers to the Object Dictionary: 0x302E
  * @retval float 0 phase-A voltage of the motor between -60 to 60
  */
float SOLOMotorControllersKvaser::GetPhaseBVoltage()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetPhaseBVoltage(error);
}

/**
  * @brief  This command reads the phase-A current of the motor connected to the
  *         "A" pin output of SOLO for 3-phase Motors
				.The method refers to the Object Dictionary: 0x302F
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval phase-A current of the motor etween -32 to 32 Amps
  */
float SOLOMotorControllersKvaser::GetPhaseACurrent(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_PhaseACurrent, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the phase-A current of the motor connected to the
  *         "A" pin output of SOLO for 3-phase Motors  
				.The method refers to the Object Dictionary: 0x302F
  * @retval phase-A current of the motor etween -32 to 32 Amps
  */
float SOLOMotorControllersKvaser::GetPhaseACurrent()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetPhaseACurrent(error);
}

/**
  * @brief  This command reads the phase-B current of the motor connected to the
  *         "B" pin output of SOLO for 3-phase Motors 
				.The method refers to the Object Dictionary: 0x3030
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float phase-B current of the motor etween -32 to 32 Amps
  */
float SOLOMotorControllersKvaser::GetPhaseBCurrent(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_PhaseBCurrent, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the phase-B current of the motor connected to the
  *         "B" pin output of SOLO for 3-phase Motors  
				.The method refers to the Object Dictionary: 0x3030
  * @retval float phase-B current of the motor etween -32 to 32 Amps
  */
float SOLOMotorControllersKvaser::GetPhaseBCurrent()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetPhaseBCurrent(error);
}

/**
  * @brief  This command reads the input BUS voltage  
				.The method refers to the Object Dictionary: 0x3031
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float  BUS voltage between 0 to 60
  */
float SOLOMotorControllersKvaser::GetBusVoltage(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_BusVoltage, 0x22, informationToSend, informationReceived, error)) {
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
float SOLOMotorControllersKvaser::GetBusVoltage()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetBusVoltage(error);
}

/**
  * @brief  This command reads the current inside the DC brushed motor connected to
  *         "B" and "C" outputs of SOLO 
				.The method refers to the Object Dictionary: 0x3032
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between -32 to 32
  */
float SOLOMotorControllersKvaser::GetDcMotorCurrentIm(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_DcMotorCurrentIm, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the current inside the DC brushed motor connected to
  *         "B" and "C" outputs of SOLO  
				.The method refers to the Object Dictionary: 0x3032
  * @retval float between -32 to 32
  */
float SOLOMotorControllersKvaser::GetDcMotorCurrentIm()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetDcMotorCurrentIm(error);
}

/**
  * @brief  This command reads the voltage of the DC brushed motor connected to
  *         "B" and "C" outputs of SOLO 
				.The method refers to the Object Dictionary: 0x3033
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between -60 to 60
  */
float SOLOMotorControllersKvaser::GetDcMotorVoltageVm(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_DcMotorVoltageVm, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the voltage of the DC brushed motor connected to
  *         "B" and "C" outputs of SOLO 
				.The method refers to the Object Dictionary: 0x3033
  * @retval float between -60 to 60
  */
float SOLOMotorControllersKvaser::GetDcMotorVoltageVm()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetDcMotorVoltageVm(error);
}

/**
  * @brief  This command reads the value of the Speed controller Kp gain, 
  *         set for Digital mode operations  
				.The method refers to the Object Dictionary: 0x300A
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 16000
  */
float SOLOMotorControllersKvaser::GetSpeedControllerKp(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_SpeedControllerKp, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the value of the Speed controller Kp gain, 
  *         set for Digital mode operations  
				.The method refers to the Object Dictionary: 0x300A
  * @retval float between 0 to 16000
  */
float SOLOMotorControllersKvaser::GetSpeedControllerKp()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetSpeedControllerKp(error);
}

/**
  * @brief  This command reads the value of the Speed controller Ki gain,
  *         set for Digital mode operations  
				.The method refers to the Object Dictionary: 0x300B
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 16000
  */
float SOLOMotorControllersKvaser::GetSpeedControllerKi(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_SpeedControllerKi, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the value of the Speed controller Ki gain,
  *         set for Digital mode operations  
				.The method refers to the Object Dictionary: 0x300B
  * @retval float between 0 to 16000
  */
float SOLOMotorControllersKvaser::GetSpeedControllerKi()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetSpeedControllerKi(error);
}

/**
  * @brief  This command reads the output switching frequency of SOLO in Hertz  
				.The method refers to the Object Dictionary: 0x3009
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long between 8000 to 80000 Hz
  */
long SOLOMotorControllersKvaser::GetOutputPwmFrequencyKhz(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_OutputPwmFrequency, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived) / 1000L); //PWM reading is in Hz
	}
	return -1;
}

/**
  * @brief  This command reads the output switching frequency of SOLO in Hertz 
				.The method refers to the Object Dictionary: 0x3009
  * @retval long between 8000 to 80000 Hz
  */
long SOLOMotorControllersKvaser::GetOutputPwmFrequencyKhz()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetOutputPwmFrequencyKhz(error);
}

/**
  * @brief  This command reads the value of the current limit set for SOLO in
  *         closed-loop digital operation mode   
				.The method refers to the Object Dictionary: 0x3003
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 32
  */
float SOLOMotorControllersKvaser::GetCurrentLimit(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_CurrentLimit, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the value of the current limit set for SOLO in
  *         closed-loop digital operation mode    
				.The method refers to the Object Dictionary: 0x3003
  * @retval float between 0 to 32
  */
float SOLOMotorControllersKvaser::GetCurrentLimit()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetCurrentLimit(error);
}

/**
  * @brief  This command reads the actual monetary value of “Iq” that is
  *         the current acts in torque generation in FOC mode for 3-phase motors
				.The method refers to the Object Dictionary: 0x3034
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between -64 to 64
  */
float SOLOMotorControllersKvaser::GetQuadratureCurrentIqFeedback(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_QuadratureCurrentIqFeedback, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the actual monetary value of “Iq” that is
  *         the current acts in torque generation in FOC mode for 3-phase motors 
				.The method refers to the Object Dictionary: 0x3034
  * @retval float between -64 to 64
  */
float SOLOMotorControllersKvaser::GetQuadratureCurrentIqFeedback()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetQuadratureCurrentIqFeedback(error);
}

/**
  * @brief  This command reads the actual monetary value of Id that is the
  *         direct current acting in FOC 
				.The method refers to the Object Dictionary: 0x3035
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between -64 to 64
  */
float SOLOMotorControllersKvaser::GetMagnetizingCurrentIdFeedback(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_MagnetizingCurrentIdFeedback, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the actual monetary value of Id that is the
  *         direct current acting in FOC  
				.The method refers to the Object Dictionary: 0x3035
  * @retval float between -64 to 64
  */
float SOLOMotorControllersKvaser::GetMagnetizingCurrentIdFeedback()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetMagnetizingCurrentIdFeedback(error);
}

/**
  * @brief  This command reads the number of Poles set for 3-phase motors 
				.The method refers to the Object Dictionary: 0x300F
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long between 1 to 254
  */
long SOLOMotorControllersKvaser::GetMotorPolesCounts(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_MotorPolesCounts, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}

/**
  * @brief  This command reads the number of Poles set for 3-phase motors  
				.The method refers to the Object Dictionary: 0x300F
  * @retval long between 1 to 254
  */
long SOLOMotorControllersKvaser::GetMotorPolesCounts()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetMotorPolesCounts(error);
}

/**
  * @brief  This command reads the number of physical Incremental encoder lines set on SOLO   
				.The method refers to the Object Dictionary: 0x3010
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long between 1 to 200000
  */
long SOLOMotorControllersKvaser::GetIncrementalEncoderLines(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_IncrementalEncoderLines, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}

/**
  * @brief  This command reads the number of physical Incremental encoder lines set on SOLO   
				.The method refers to the Object Dictionary: 0x3010
  * @retval long between 1 to 200000
  */
long SOLOMotorControllersKvaser::GetIncrementalEncoderLines()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetIncrementalEncoderLines(error);
}

/**
  * @brief  This command reads the amount of value set for Current controller
  *         Kp or proportional gain 
				.The method refers to the Object Dictionary: 0x3017
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 16000
  */
float SOLOMotorControllersKvaser::GetCurrentControllerKp(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_CurrentControllerKp, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the amount of value set for Current controller
  *         Kp or proportional gain  
				.The method refers to the Object Dictionary: 0x3017
  * @retval float between 0 to 16000
  */
float SOLOMotorControllersKvaser::GetCurrentControllerKp()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetCurrentControllerKp(error);
}

/**
  * @brief  This command reads the amount of value set for Current controller
  *         Ki or integrator gain  
				.The method refers to the Object Dictionary: 0x3018
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 16000
  */
float SOLOMotorControllersKvaser::GetCurrentControllerKi(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_CurrentControllerKi, 0x22, informationToSend, informationReceived, error)) {
		float readValue = soloUtils->ConvertToFloat(informationReceived);
		return readValue*0.00005;
	}
	return -1.0;
}

/**
  * @brief  This command reads the amount of value set for Current controller
  *         Ki or integrator gain   
				.The method refers to the Object Dictionary: 0x3018
  * @retval float between 0 to 16000
  */
float SOLOMotorControllersKvaser::GetCurrentControllerKi()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetCurrentControllerKi(error);
}

/**
  * @brief  This command reads the momentary temperature of the board in centigrade 
				.The method refers to the Object Dictionary: 0x3039
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between -30 to 150 Celsius
  */
float SOLOMotorControllersKvaser::GetBoardTemperature(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_BoardTemperature, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the momentary temperature of the board in centigrade  	
				.The method refers to the Object Dictionary: 0x3039
  * @retval float between -30 to 150 Celsius
  */
float SOLOMotorControllersKvaser::GetBoardTemperature()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetBoardTemperature(error);
}

/**
  * @brief  This command reads the Phase or Armature resistance of
  *         the 3-phase or DC brushed motor connected to SOLO respectively  
				.The method refers to the Object Dictionary: 0x300D
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 100
  */
float SOLOMotorControllersKvaser::GetMotorResistance(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_MotorResistance, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived)*8);
	}
	return -1.0;
}

/**
  * @brief  This command reads the Phase or Armature resistance of
  *         the 3-phase or DC brushed motor connected to SOLO respectively 
				.The method refers to the Object Dictionary: 0x300D
  * @retval float between 0 to 100
  */
float SOLOMotorControllersKvaser::GetMotorResistance()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetMotorResistance(error);
}

/**
  * @brief  This command reads the Phase or Armature Inductance of 
  *         the 3-phase or DC brushed motor connected to SOLO respectively
				.The method refers to the Object Dictionary: 0x300E
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 0.2
  */
float SOLOMotorControllersKvaser::GetMotorInductance(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_MotorInductance, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived)/8);
	}
	return -1.0;
}

/**
  * @brief  This command reads the Phase or Armature Inductance of 
  *         the 3-phase or DC brushed motor connected to SOLO respectively 
				.The method refers to the Object Dictionary: 0x300E
  * @retval float between 0 to 0.2
  */
float SOLOMotorControllersKvaser::GetMotorInductance()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetMotorInductance(error);
}

/**
  * @brief  this command reads the actual speed of the motor measured or estimated by SOLO in
            sensorless or sensor-based modes respectively  
				.The method refers to the Object Dictionary: 0x3036
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long between -30000 to 30000 RPM
  */
long SOLOMotorControllersKvaser::GetSpeedFeedback(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_SpeedFeedback, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}

/**
  * @brief  this command reads the actual speed of the motor measured or estimated by SOLO in
            sensorless or sensor-based modes respectively  
				.The method refers to the Object Dictionary: 0x3036
  * @retval long between -30000 to 30000 RPM
  */
long SOLOMotorControllersKvaser::GetSpeedFeedback()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetSpeedFeedback(error);
}

/**
  * @brief  This command reads the Motor type selected for Digital or Analogue mode operations 
				.The method refers to the Object Dictionary: 0x3015
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long between 0 to 3
  */
long SOLOMotorControllersKvaser::GetMotorType(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_MotorType, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}

/**
  * @brief  This command reads the Motor type selected for Digital or Analogue mode operations 
				.The method refers to the Object Dictionary: 0x3015
  * @retval long between 0 to 3
  */
long SOLOMotorControllersKvaser::GetMotorType()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetMotorType(error);
}

/**
  * @brief  This command reads the feedback control mode selected on SOLO both
  *         for Analogue and Digital operations  
				.The method refers to the Object Dictionary: 0x3013
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long between 0 to 2
  */
long SOLOMotorControllersKvaser::GetFeedbackControlMode(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_FeedbackControlMode, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}

/**
  * @brief  This command reads the feedback control mode selected on SOLO both
  *         for Analogue and Digital operations   
				.The method refers to the Object Dictionary: 0x3013
  * @retval long between 0 to 2
  */
long SOLOMotorControllersKvaser::GetFeedbackControlMode()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetFeedbackControlMode(error);
}

/**
  * @brief  This command reads the actual commanding mode that SOLO is operating 
				.The method refers to the Object Dictionary: 0x3002
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long between 0 or 1
  */
long SOLOMotorControllersKvaser::GetCommandMode(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_CommandMode, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}

/**
  * @brief  This command reads the actual commanding mode that SOLO is operating
				.The method refers to the Object Dictionary: 0x3002
  * @retval long between 0 or 1
  */
long SOLOMotorControllersKvaser::GetCommandMode()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetCommandMode(error);
}

/**
  * @brief  This command reads the Control Mode type in terms of Torque,
  *         Speed or Position in both Digital and Analogue modes 
				.The method refers to the Object Dictionary: 0x3013
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long between 0 to 2
  */
long SOLOMotorControllersKvaser::GetControlMode(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_ControlMode, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}

/**
  * @brief  This command reads the Control Mode type in terms of Torque,
  *         Speed or Position in both Digital and Analogue modes  
				.The method refers to the Object Dictionary: 0x3013
  * @retval long between 0 to 2
  */
long SOLOMotorControllersKvaser::GetControlMode()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetControlMode(error);
}

/**
  * @brief  This command reads the value of the speed limit set on SOLO 
				.The method refers to the Object Dictionary: 0x3011
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long between 0 to 30000
  */
long SOLOMotorControllersKvaser::GetSpeedLimit(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_SpeedLimit, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}

/**
  * @brief  This command reads the value of the speed limit set on SOLO 	
				.The method refers to the Object Dictionary: 0x3011
  * @retval long between 0 to 30000
  */
long SOLOMotorControllersKvaser::GetSpeedLimit()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetSpeedLimit(error);
}

/**
  * @brief  This command reads the amount of value set for Position
  *         controller Kp or proportional gain  
				.The method refers to the Object Dictionary: 0x301C
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 16000
  */
float SOLOMotorControllersKvaser::GetPositionControllerKp(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_PositionControllerKp, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the amount of value set for Position
  *         controller Kp or proportional gain 
				.The method refers to the Object Dictionary: 0x301C
  * @retval float between 0 to 16000
  */
float SOLOMotorControllersKvaser::GetPositionControllerKp()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetPositionControllerKp(error);
}

/**
  * @brief  This command reads the amount of value set for Position
  *         controller Ki or integrator gain  
				.The method refers to the Object Dictionary: 0x301D
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 16000
  */
float SOLOMotorControllersKvaser::GetPositionControllerKi(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_PositionControllerKi, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the amount of value set for Position
  *         controller Ki or integrator gain   
				.The method refers to the Object Dictionary: 0x301D
  * @retval float between 0 to 16000
  */
float SOLOMotorControllersKvaser::GetPositionControllerKi()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetPositionControllerKi(error);
}

/**
  * @brief  This command reads the number of counted pulses from the
  *         Incremental Encoder or Hall sensors 
				.The method refers to the Object Dictionary: 0x3037
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long between -2,147,483,647 to 2,147,483,647
  */
long SOLOMotorControllersKvaser::GetPositionCountsFeedback(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_PositionCountsFeedback, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}

/**
  * @brief  This command reads the number of counted pulses from the
  *         Incremental Encoder or Hall sensors   
				.The method refers to the Object Dictionary: 0x3037
  * @retval long between -2,147,483,647 to 2,147,483,647
  */
long SOLOMotorControllersKvaser::GetPositionCountsFeedback()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetPositionCountsFeedback(error);
}

/**
  * @brief  This command reads the error register which is a 32 bit register with
  *         each bit corresponding to specific errors  
				.The method refers to the Object Dictionary: 0x3020
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long 
  */
long SOLOMotorControllersKvaser::GetErrorRegister(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_OverwriteErrorRegister, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}

/**
  * @brief  This command reads the error register which is a 32 bit register with
  *         each bit corresponding to specific errors  
				.The method refers to the Object Dictionary: 0x3020
  * @retval long 
  */
long SOLOMotorControllersKvaser::GetErrorRegister()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetErrorRegister(error);
}

/**
  * @brief  This command reads the Firmware version existing currently on the SOLO unit  
				.The method refers to the Object Dictionary: 0x303A
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long
  */
long SOLOMotorControllersKvaser::GetDeviceFirmwareVersion(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_DeviceFirmwareVersion, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}

/**
  * @brief  This command reads the Firmware version existing currently on the SOLO unit  
				.The method refers to the Object Dictionary: 0x303A
  * @retval long
  */
long SOLOMotorControllersKvaser::GetDeviceFirmwareVersion()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetDeviceFirmwareVersion(error);
}

/**
  * @brief  This command reads the Hardware version of the SOLO unit connected  
				.The method refers to the Object Dictionary: 0x303B
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long
  */
long SOLOMotorControllersKvaser::GetDeviceHardwareVersion(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_DeviceHardwareVersion, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}

/**
  * @brief  This command reads the Hardware version of the SOLO unit connected   
				.The method refers to the Object Dictionary: 0x303B
  * @retval long
  */
long SOLOMotorControllersKvaser::GetDeviceHardwareVersion()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetDeviceHardwareVersion(error);
}

/**
  * @brief  This command reads the amount of desired Torque reference (Iq or IM)
  *         already set for the Motor to follow in Digital Closed-loop Torque control mode 	
				.The method refers to the Object Dictionary: 0x3004
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 32
  */
float SOLOMotorControllersKvaser::GetTorqueReferenceIq(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_TorqueReferenceIq, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the amount of desired Torque reference (Iq or IM)
  *         already set for the Motor to follow in Digital Closed-loop Torque control mode  
				.The method refers to the Object Dictionary: 0x3004
  * @retval float between 0 to 32
  */
float SOLOMotorControllersKvaser::GetTorqueReferenceIq()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetTorqueReferenceIq(error);
}

/**
  * @brief  This command reads the amount of desired Speed reference already set for
  *         the Motor to follow in Digital Closed-loop Speed control mode  
				.The method refers to the Object Dictionary: 0x3005
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long between 0 to 30000
  */
long SOLOMotorControllersKvaser::GetSpeedReference(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_SpeedReference, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}

/**
  * @brief  This command reads the amount of desired Speed reference already set for
  *         the Motor to follow in Digital Closed-loop Speed control mode  
				.The method refers to the Object Dictionary: 0x3005
  * @retval long between 0 to 30000
  */
long SOLOMotorControllersKvaser::GetSpeedReference()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetSpeedReference(error);
}

/**
  * @brief  This command the actual monetary value of Id that is the
  *         direct current acting in FOC  
  *         in Digital Closed-loop Speed control mode for ACIM motors 
				.The method refers to the Object Dictionary: 0x301A
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 32
  */
float SOLOMotorControllersKvaser::GetMagnetizingCurrentIdReference(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_MagnetizingCurrentIdReference, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the amount of desired Id (direct current) or
  *         Magnetizing current reference already set for the Motor to follow
  *         in Digital Closed-loop Speed control mode for ACIM motors 
				.The method refers to the Object Dictionary: 0x301A
  * @retval float between 0 to 32
  */
float SOLOMotorControllersKvaser::GetMagnetizingCurrentIdReference()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetMagnetizingCurrentIdReference(error);
}

/**
  * @brief  This command reads the desired position reference set for the Motor
  *         to follow in Digital Closed-loop Position mode in terms of quadrature pulses 
				.The method refers to the Object Dictionary: 0x301B
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long between -2,147,483,647 to 2,147,483,647 Quad-Pulses
  */
long SOLOMotorControllersKvaser::GetPositionReference(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_PositionReference, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}

/**
  * @brief  This command reads the desired position reference set for the Motor
  *         to follow in Digital Closed-loop Position mode in terms of quadrature pulses  
				.The method refers to the Object Dictionary: 0x301B
  * @retval long between -2,147,483,647 to 2,147,483,647 Quad-Pulses
  */
long SOLOMotorControllersKvaser::GetPositionReference()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetPositionReference(error);
}

/**
  * @brief  This command reads the desired Power reference for SOLO to apply in 
  *         Digital Open-loop speed control mode for 3-phase motors in terms of percentage
				.The method refers to the Object Dictionary: 0x3006
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 100 %
  */
float SOLOMotorControllersKvaser::GetPowerReference(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_PowerReference, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the desired Power reference for SOLO to apply in 
  *         Digital Open-loop speed control mode for 3-phase motors in terms of percentage 
				.The method refers to the Object Dictionary: 0x3006
  * @retval float between 0 to 100 %
  */
float SOLOMotorControllersKvaser::GetPowerReference()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetPowerReference(error);
}

/**
  * @brief  This commands reads the desired direction of rotation set for the Motor 
				.The method refers to the Object Dictionary: 0x300C
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long 0 Counter ClockWise / 1 ClockWise
  */
long SOLOMotorControllersKvaser::GetMotorDirection(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_MotorDirection, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}

/**
  * @brief  This commands reads the desired direction of rotation set for the Motor 
				.The method refers to the Object Dictionary: 0x300C
  * @retval long 0 Counter ClockWise / 1 ClockWise
  */
long SOLOMotorControllersKvaser::GetMotorDirection()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetMotorDirection(error);
}

/**
  * @brief  This command reads the value of Sensorless Observer Gain for Normal BLDC-PMSM Motors 
				.The method refers to the Object Dictionary: 0x3021
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0.01 to 1000
  */
float SOLOMotorControllersKvaser::GetObserverGainBldcPmsm(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_ObserverGainBldcPmsm, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the value of Sensorless Observer Gain for Normal BLDC-PMSM Motors 
				.The method refers to the Object Dictionary: 0x3021
  * @retval float between 0.01 to 1000
  */
float SOLOMotorControllersKvaser::GetObserverGainBldcPmsm()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetObserverGainBldcPmsm(error);
}

/**
  * @brief  This command reads the value of Sensorless Observer Gain for Normal BLDC-PMSM Motors 
				.The method refers to the Object Dictionary: 0x3022
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0.01 to 1000
  */
float SOLOMotorControllersKvaser::GetObserverGainBldcPmsmUltrafast(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_ObserverGainBldcPmsmUltrafast, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the value of Sensorless Observer Gain for Normal BLDC-PMSM Motors  
				.The method refers to the Object Dictionary: 0x3022
  * @retval float between 0.01 to 1000
  */
float SOLOMotorControllersKvaser::GetObserverGainBldcPmsmUltrafast()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetObserverGainBldcPmsmUltrafast(error);
}

/**
  * @brief  This command reads the value of Sensorless Observer Gain for DC Motor  
				.The method refers to the Object Dictionary: 0x3023
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0.01 to 1000
  */
float SOLOMotorControllersKvaser::GetObserverGainDc(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_ObserverGainDc, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the value of Sensorless Observer Gain for DC Motor   
				.The method refers to the Object Dictionary: 0x3023
  * @retval float between 0.01 to 1000
  */
float SOLOMotorControllersKvaser::GetObserverGainDc()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetObserverGainDc(error);
}

/**
  * @brief  This command reads the value of Sensorless Observer
  *         Filter Gain for Normal BLDC-PMSM Motors 
				.The method refers to the Object Dictionary: 0x3024
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0.01 to 16000
  */
float SOLOMotorControllersKvaser::GetFilterGainBldcPmsm(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_FilterGainBldcPmsm, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the value of Sensorless Observer
  *         Filter Gain for Normal BLDC-PMSM Motors
				.The method refers to the Object Dictionary: 0x3024
  * @retval float between 0.01 to 16000
  */
float SOLOMotorControllersKvaser::GetFilterGainBldcPmsm()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetFilterGainBldcPmsm(error);
}

/**
  * @brief  This command reads the value of Sensorless Observer
  *         Filter Gain for Ultra Fast BLDC-PMSM Motors 
				.The method refers to the Object Dictionary: 0x3025
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0.01 to 16000
  */
float SOLOMotorControllersKvaser::GetFilterGainBldcPmsmUltrafast(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_FilterGainBldcPmsmUltrafast, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the value of Sensorless Observer
  *         Filter Gain for Ultra Fast BLDC-PMSM Motors   
				.The method refers to the Object Dictionary: 0x3025
  * @retval float between 0.01 to 16000
  */
float SOLOMotorControllersKvaser::GetFilterGainBldcPmsmUltrafast()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetFilterGainBldcPmsmUltrafast(error);
}

/**
  * @brief  This command reads the measured or estimated per-unit angle of the 3-phase motors  
				.The method refers to the Object Dictionary: 0x3038
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between -2 to 2
  */
float SOLOMotorControllersKvaser::Get3PhaseMotorAngle(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_3PhaseMotorAngle, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the measured or estimated per-unit angle of the 3-phase motors  
				.The method refers to the Object Dictionary: 0x3038
  * @retval float between -2 to 2
  */
float SOLOMotorControllersKvaser::Get3PhaseMotorAngle()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::Get3PhaseMotorAngle(error);
}

/**
  * @brief  This command reads the per-unit Encoder or Hall sensor offset in C.C.W direction 
				.The method refers to the Object Dictionary: 0x3028
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 1.0
  */
float SOLOMotorControllersKvaser::GetEncoderHallCcwOffset(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_EncoderHallCcwOffset, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the per-unit Encoder or Hall sensor offset in C.C.W direction  
				.The method refers to the Object Dictionary: 0x3028
  * @retval float between 0 to 1.0
  */
float SOLOMotorControllersKvaser::GetEncoderHallCcwOffset()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetEncoderHallCcwOffset(error);
}

/**
  * @brief  This command reads the per-unit Encoder or Hall sensor offset in C.C.W direction  
				.The method refers to the Object Dictionary: 0x3029
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 1.0
  */
float SOLOMotorControllersKvaser::GetEncoderHallCwOffset(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_EncoderHallCwOffset, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the per-unit Encoder or Hall sensor offset in C.C.W direction 
				.The method refers to the Object Dictionary: 0x3029
  * @retval float between 0 to 1.0
  */
float SOLOMotorControllersKvaser::GetEncoderHallCwOffset()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetEncoderHallCwOffset(error);
}

/**
  * @brief  This command reads Baud Rate selected on SOLO unit to communicate through UART line  	
				.The method refers to the Object Dictionary: 0x3026
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long between 0 or 1
  */
long SOLOMotorControllersKvaser::GetUartBaudrate(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_UartBaudrate, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}

/**
  * @brief  This command reads Baud Rate selected on SOLO unit to communicate through UART line   
				.The method refers to the Object Dictionary: 0x3026
  * @retval long between 0 or 1
  */
long SOLOMotorControllersKvaser::GetUartBaudrate()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetUartBaudrate(error);
}

/**
  * @brief  This command reads the acceleration value of the Speed for
  *         speed controller both in Analogue and Digital modes
  *         in Revolution per square seconds  
				.The method refers to the Object Dictionary: 0x302A
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 1600 Rev/S^2
  */
float SOLOMotorControllersKvaser::GetSpeedAccelerationValue(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_SpeedAccelerationValue, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the acceleration value of the Speed for
  *         speed controller both in Analogue and Digital modes
  *         in Revolution per square seconds   
				.The method refers to the Object Dictionary: 0x302A
  * @retval float between 0 to 1600 Rev/S^2
  */
float SOLOMotorControllersKvaser::GetSpeedAccelerationValue()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetSpeedAccelerationValue(error);
}

/**
  * @brief  This command reads the deceleration value of the Speed for
  *         speed controller both in Analogue and Digital modes
  *         in Revolution per square seconds 
				.The method refers to the Object Dictionary: 0x302B
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 1600 Rev/S^2
  */
float SOLOMotorControllersKvaser::GetSpeedDecelerationValue(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_SpeedDecelerationValue, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}

/**
  * @brief  This command reads the deceleration value of the Speed for
  *         speed controller both in Analogue and Digital modes
  *         in Revolution per square seconds  
				.The method refers to the Object Dictionary: 0x302B
  * @retval float between 0 to 1600 Rev/S^2
  */
float SOLOMotorControllersKvaser::GetSpeedDecelerationValue()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetSpeedDecelerationValue(error);
}

/**
  * @brief  This command reads the Analogue Speed Resolution Division Coefficient (ASRDC)
				.The method refers to the Object Dictionary: 0x303E
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long
  */
long SOLOMotorControllersKvaser::GetAnalogueSpeedResolutionDivisionCoefficient(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
	if (comIf->CANOpenReceive(Address, Object_ASRDC, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
    return -1 ;
}

/**
  * @brief  This command reads the Analogue Speed Resolution Division Coefficient (ASRDC)
				.The method refers to the Object Dictionary: 0x303E
  * @retval long
  */
long SOLOMotorControllersKvaser::GetAnalogueSpeedResolutionDivisionCoefficient()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersKvaser::GetAnalogueSpeedResolutionDivisionCoefficient(error);
}

/**
  * @brief  This Command reads the number of counted index pulses 
  *         seen on the Incremental Encoder’s output  
				.The method refers to the Object Dictionary: 0x303D
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long between 0 to 2,147,483,647
  */
long SOLOMotorControllersKvaser::GetEncoderIndexCounts(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (comIf->CANOpenReceive(Address, Object_EncoderIndexCounts, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;

}

/**
  * @brief  This Command reads the number of counted index pulses 
  *         seen on the Incremental Encoder’s output   
				.The method refers to the Object Dictionary: 0x303D
  * @retval long between 0 to 2,147,483,647
  */
long SOLOMotorControllersKvaser::GetEncoderIndexCounts()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetEncoderIndexCounts(error);
}

/**
  * @brief  This command test if the communication is working   
  * @retval bool 0 not working / 1 for working
  */
bool SOLOMotorControllersKvaser::CommunicationIsWorking(int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	float temperature = SOLOMotorControllersKvaser::GetBoardTemperature(error);
	if (error == SOLOMotorControllers::Error::noErrorDetected) {
		return true;
	}
	return false;
}

/**
  * @brief  This command test if the communication is working   
  * @retval bool 0 not working / 1 for working
  */
bool SOLOMotorControllersKvaser::CommunicationIsWorking()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::CommunicationIsWorking(error);
}

/**
  * @brief  This command reads the type of the Embedded Motion profile active in the controller 
				.The method refers to the Object Dictionary: 0x303F
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long
  */
long SOLOMotorControllersKvaser::GetMotionProfileMode(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
	if (comIf->CANOpenReceive(Address, Object_MotionProfileMode, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
    return -1 ;
}

/**
  * @brief  This command reads the type of the Embedded Motion profile active in the controller 
				.The method refers to the Object Dictionary: 0x303F
  * @retval long
  */
long SOLOMotorControllersKvaser::GetMotionProfileMode()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersKvaser::GetMotionProfileMode(error);
}

/**
  * @brief  This command reads the value of the Motion Profile Variable1 set inside the controller 
				.The method refers to the Object Dictionary: 0x3040
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float
  */
float SOLOMotorControllersKvaser::GetMotionProfileVariable1(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
	if (comIf->CANOpenReceive(Address, Object_MotionProfileVariable1, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
    return -1 ;
}

/**
  * @brief  This command reads the value of the Motion Profile Variable1 set inside the controller  
				.The method refers to the Object Dictionary: 0x3040
  * @retval float
  */
float SOLOMotorControllersKvaser::GetMotionProfileVariable1()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersKvaser::GetMotionProfileVariable1(error);
}

/**
  * @brief  This command reads the value of the Motion Profile Variable2 set inside the controller 
				.The method refers to the Object Dictionary: 0x3041
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float
  */
float SOLOMotorControllersKvaser::GetMotionProfileVariable2(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
	if (comIf->CANOpenReceive(Address, Object_MotionProfileVariable2, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
    return -1 ;
}

/**
  * @brief  This command reads the value of the Motion Profile Variable2 set inside the controller 
				.The method refers to the Object Dictionary: 0x3041
  * @retval float
  */
float SOLOMotorControllersKvaser::GetMotionProfileVariable2()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersKvaser::GetMotionProfileVariable2(error);
}

/**
  * @brief  This command reads the value of the Motion Profile Variable3 set inside the controller 
				.The method refers to the Object Dictionary: 0x3042
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float
  */
float SOLOMotorControllersKvaser::GetMotionProfileVariable3(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
	if (comIf->CANOpenReceive(Address, Object_MotionProfileVariable3, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
    return -1 ;
}

/**
  * @brief  This command reads the value of the Motion Profile Variable3 set inside the controller
				.The method refers to the Object Dictionary: 0x3042
  * @retval float
  */
float SOLOMotorControllersKvaser::GetMotionProfileVariable3()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersKvaser::GetMotionProfileVariable3(error);
}

/**
  * @brief  This command reads the value of the Motion Profile Variable4 set inside the controller
				.The method refers to the Object Dictionary: 0x3043
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float
  */
float SOLOMotorControllersKvaser::GetMotionProfileVariable4(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
	if (comIf->CANOpenReceive(Address, Object_MotionProfileVariable4, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
    return -1 ;
}

/**
  * @brief  This command reads the value of the Motion Profile Variable4 set inside the controller 
				.The method refers to the Object Dictionary: 0x3043
  * @retval float
  */
float SOLOMotorControllersKvaser::GetMotionProfileVariable4()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersKvaser::GetMotionProfileVariable4(error);
}

/**
  * @brief  This command reads the value of the Motion Profile Variable5 set inside the controller 
				.The method refers to the Object Dictionary: 0x3044
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float
  */
float SOLOMotorControllersKvaser::GetMotionProfileVariable5(int &error)
{
    uint8_t  informationToSend  [4] = {0x00,0x00,0x00,0x00};
    uint8_t  informationReceived[4] = {0x00,0x00,0x00,0x00};
    error = SOLOMotorControllers::Error::noProcessedCommand;
	if (comIf->CANOpenReceive(Address, Object_MotionProfileVariable5, 0x22, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
    return -1 ; 
}

/**
  * @brief  This command reads the value of the Motion Profile Variable5 set inside the controller 
				.The method refers to the Object Dictionary: 0x3044
  * @retval float
  */
float SOLOMotorControllersKvaser::GetMotionProfileVariable5()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersKvaser::GetMotionProfileVariable5(error);
}

void SOLOMotorControllersKvaser::GeneralCanbusRead(uint16_t *ID , uint8_t *DLC, uint8_t *Data)
{
	comIf->CANOpenGenericReceive(ID , DLC, Data);
}

void SOLOMotorControllersKvaser::GeneralCanbusWrite(uint16_t ID, uint8_t *DLC, uint8_t *Data, int &error)
{
	comIf->CANOpenGenericTransmit(ID , DLC, Data, error);
}

/**
  * @brief  this PDO command give the first in the baffer position of the Motor
  *         to follow in Digital Closed-loop Position mode in terms of quadrature pulses 
  *				.The method refers to the Object Dictionary: 0x1814
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long between -2,147,483,647 to 2,147,483,647 Quad-Pulses
  */
long SOLOMotorControllersKvaser::GetPdoPositionCountsFeedback(int& error)
{
	return GetPdoParameterValueLong(PdoParameterName::positionCountsFeedback, error);
}

/**
  * @brief  Tthis PDO command give the first in the baffer position of the Motor
  *         to follow in Digital Closed-loop Position mode in terms of quadrature pulses  
  *				.The method refers to the Object Dictionary: 0x1814
  * @retval long between -2,147,483,647 to 2,147,483,647 Quad-Pulses
  */
long SOLOMotorControllersKvaser::GetPdoPositionCountsFeedback()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetPdoPositionCountsFeedback(error);
}

/**
  * @brief  this PDO command give the first in the baffer speed of the motor measured or estimated by SOLO in
  *		    sensorless or sensor-based modes respectively  
  *				.The method refers to the Object Dictionary: 0x1815
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long value that rappresent the speed feeback in RPM (positive and negative value) 
  */
long SOLOMotorControllersKvaser::GetPdoSpeedFeedback (int &error){
	return GetPdoParameterValueLong(PdoParameterName::speedFeedback, error);
}
/**
  * @brief  tthis PDO command give the first in the baffer speed of the motor measured or estimated by SOLO in
  *	       sensorless or sensor-based modes respectively  
  *				.The method refers to the Object Dictionary: 0x1815
  * @retval long value that rappresent the speed feeback in RPM (positive and negative value)
  */
long SOLOMotorControllersKvaser::GetPdoSpeedFeedback (){
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return GetPdoParameterValueLong(PdoParameterName::speedFeedback, error);
}

/**
  * @brief  This PDO command give the first in the baffer monetary value of “Iq” that is
  *         the current acts in torque generation in FOC mode for 3-phase motors
  *				.The method refers to the Object Dictionary: 0x1816
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 32
  */
float SOLOMotorControllersKvaser::GetPdoQuadratureCurrentIqFeedback(int& error)
{
	return GetPdoParameterValueFloat(PdoParameterName::quadratureCurrentIqFeedback, error);
}

/**
  * @brief  This PDO command give the first in the baffer monetary value of “Iq” that is
  *         the current acts in torque generation in FOC mode for 3-phase motors 
  *				.The method refers to the Object Dictionary: 0x1816
  * @retval float between 0 to 32
  */
float SOLOMotorControllersKvaser::GetPdoQuadratureCurrentIqFeedback()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetPdoQuadratureCurrentIqFeedback(error);
}

/**
  * @brief  This PDO command give the first in the baffer monetary value of Id that is the
  *         direct current acting in FOC  
				.The method refers to the Object Dictionary: 0x1817
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 32
  */
float SOLOMotorControllersKvaser::GetPdoMagnetizingCurrentIdFeedback(int& error)
{
	return GetPdoParameterValueFloat(PdoParameterName::magnetizingCurrentIdFeedback, error);
}

/**
  * @brief  This PDO command give the first in the baffer monetary value of Id that is the
  *         direct current acting in FOC 
  *				.The method refers to the Object Dictionary: 0x1817
  * @retval float between 0 to 32
  */
float SOLOMotorControllersKvaser::GetPdoMagnetizingCurrentIdFeedback()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetPdoMagnetizingCurrentIdFeedback(error);
}

/**
  * @brief  This PDO command reads the error register which is a 32 bit register with
  *         each bit corresponding to specific errors  
  *				.The method refers to the Object Dictionary: 0x1818
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long 
  */
long SOLOMotorControllersKvaser::GetPdoErrorRegister(int& error)
{
	return GetPdoParameterValueLong(PdoParameterName::errorRegister, error);
}

/**
  * @brief  This PDO command reads the error register which is a 32 bit register with
  *         each bit corresponding to specific errors  
  *				.The method refers to the Object Dictionary: 0x1818
  * @retval long 
  */
long SOLOMotorControllersKvaser::GetPdoErrorRegister()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetPdoErrorRegister(error);
}

/**
  * @brief  This PDO command reads the momentary temperature of the board in centigrade 
  *				.The method refers to the Object Dictionary: 0x1819
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between -30 to 150 Celsius
  */
float SOLOMotorControllersKvaser::GetPdoBoardTemperature(int& error)
{
	return GetPdoParameterValueFloat(PdoParameterName::boardTemperature, error);
}

/**
  * @brief  This PDO command reads the momentary temperature of the board in centigrade  	
  *				.The method refers to the Object Dictionary: 0x1819
  * @retval float between -30 to 150 Celsius
  */
float SOLOMotorControllersKvaser::GetPdoBoardTemperature()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetPdoBoardTemperature(error);
}
