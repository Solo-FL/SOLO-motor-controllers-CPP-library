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

#include "SOLOMotorControllersKvaser.h"

SOLOMotorControllersKvaser::SOLOMotorControllersKvaser(unsigned char deviceAddress, 
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
	case 1000:
		canBaudrate = canBITRATE_1M;
		break;
	case 500:
		canBaudrate = canBITRATE_500K;
		break;
	case 250:
		canBaudrate = canBITRATE_250K;
		break;
	case 125:
		canBaudrate = canBITRATE_125K;
		break;
	case 100:
		canBaudrate = canBITRATE_100K;
		break;
	default:
		canBaudrate = canBITRATE_1M;
		break;
	}
	soloUtils = new SOLOMotorControllersUtils();
	kvaser = new Kvaser();
	if(autoConnect)
	{
		SOLOMotorControllersKvaser::Connect();
	}	
}

SOLOMotorControllersKvaser::~SOLOMotorControllersKvaser()
{
	Disconnect();
}

bool SOLOMotorControllersKvaser::Connect(unsigned char deviceAddress, 
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

	return SOLOMotorControllersKvaser::Connect();
}

bool SOLOMotorControllersKvaser::Connect()
{
	canStatus stat;
	canInitializeLibrary();
	hnd = canOpenChannel(0, 0);
	if (hnd < 0)
	{
		return false;
	}
	stat = canSetBusParams(hnd, canBaudrate, 0, 0, 0, 0, 0);
	if(stat == canOK)
		return true;
	else
	{
		canClose(hnd);
		return false;
	}
}

void SOLOMotorControllersKvaser::Disconnect()
{
	canClose(hnd);
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
	return kvaser->CANOpenTransmit(hnd, Address, Object_GuardTime, informatrionToSend, error);
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
	return kvaser->CANOpenTransmit(hnd, Address, Object_LifeTimeFactor, informatrionToSend, error);
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
	return kvaser->CANOpenTransmit(hnd, Address, Object_ProducerHeartbeatTime, informatrionToSend, error);
}

bool SOLOMotorControllersKvaser::SetProducerHeartbeatTime(long producerHeartbeatTime)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersKvaser::SetProducerHeartbeatTime(producerHeartbeatTime,error);
}

bool SOLOMotorControllersKvaser::SetDeviceAddress(unsigned char deviceAddress, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetDeviceAddressInputValidation(deviceAddress, error))
	{
		return false;
	}
	soloUtils->ConvertToData((long)deviceAddress, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_SetDeviceAddress, informatrionToSend, error);

}
bool SOLOMotorControllersKvaser::SetDeviceAddress(unsigned char deviceAddress)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetDeviceAddress(deviceAddress, error);
}
bool SOLOMotorControllersKvaser::SetCommandMode(SOLOMotorControllers::CommandMode mode, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	soloUtils->ConvertToData((long)mode, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_CommandMode, informatrionToSend, error);

}
bool SOLOMotorControllersKvaser::SetCommandMode(SOLOMotorControllers::CommandMode mode)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetCommandMode(mode, error);
}
bool SOLOMotorControllersKvaser::SetCurrentLimit(float currentLimit, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetCurrentLimitInputValidation(currentLimit, error))
	{
		return false;
	}
	soloUtils->ConvertToData(currentLimit, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_CurrentLimit, informatrionToSend, error);

}
bool SOLOMotorControllersKvaser::SetCurrentLimit(float currentLimit)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetCurrentLimit(currentLimit, error);
}
bool SOLOMotorControllersKvaser::SetTorqueReferenceIq(float torqueReferenceIq, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetTorqueReferenceIqInputValidation(torqueReferenceIq, error))
	{
		return false;
	}
	soloUtils->ConvertToData(torqueReferenceIq, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_TorqueReferenceIq, informatrionToSend, error);

}
bool SOLOMotorControllersKvaser::SetTorqueReferenceIq(float torqueReferenceIq)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetTorqueReferenceIq(torqueReferenceIq, error);
}
bool SOLOMotorControllersKvaser::SetSpeedReference(long speedReference, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetSpeedReferenceInputValidation(speedReference, error))
	{
		return false;
	}
	soloUtils->ConvertToData(speedReference, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_SpeedReference, informatrionToSend, error);

}
bool SOLOMotorControllersKvaser::SetSpeedReference(long speedReference)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetSpeedReference(speedReference, error);
}
bool SOLOMotorControllersKvaser::SetPowerReference(float powerReference, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetPowerReferenceInputValidation(powerReference, error))
	{
		return false;
	}
	soloUtils->ConvertToData(powerReference, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_PowerReference, informatrionToSend, error);

}
bool SOLOMotorControllersKvaser::SetPowerReference(float powerReference)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetPowerReference(powerReference, error);
}
bool SOLOMotorControllersKvaser::MotorParametersIdentification(SOLOMotorControllers::Action identification, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	soloUtils->ConvertToData((long)identification, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_MotorParametersIdentification, informatrionToSend, error);

}
bool SOLOMotorControllersKvaser::MotorParametersIdentification(SOLOMotorControllers::Action identification)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::MotorParametersIdentification(identification, error);
}
bool SOLOMotorControllersKvaser::EmergencyStop(int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	return kvaser->CANOpenTransmit(hnd, Address, Object_EmergencyStop, informatrionToSend, error);

}
bool SOLOMotorControllersKvaser::EmergencyStop()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::EmergencyStop(error);
}
bool SOLOMotorControllersKvaser::SetOutputPwmFrequencyKhz(long outputPwmFrequencyKhz, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetOutputPwmFrequencyKhzInputValidation(outputPwmFrequencyKhz, error))
	{
		return false;
	}
	soloUtils->ConvertToData(outputPwmFrequencyKhz, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_OutputPwmFrequencyKhz, informatrionToSend, error);

}
bool SOLOMotorControllersKvaser::SetOutputPwmFrequencyKhz(long outputPwmFrequencyKhz)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetOutputPwmFrequencyKhz(outputPwmFrequencyKhz, error);
}
bool SOLOMotorControllersKvaser::SetSpeedControllerKp(float speedControllerKp, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetSpeedControllerKpInputValidation(speedControllerKp, error))
	{
		return false;
	}
	soloUtils->ConvertToData(speedControllerKp, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_SpeedControllerKp, informatrionToSend, error);

}
bool SOLOMotorControllersKvaser::SetSpeedControllerKp(float speedControllerKp)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetSpeedControllerKp(speedControllerKp, error);
}
bool SOLOMotorControllersKvaser::SetSpeedControllerKi(float speedControllerKi, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetSpeedControllerKiInputValidation(speedControllerKi, error))
	{
		return false;
	}
	soloUtils->ConvertToData(speedControllerKi, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_SpeedControllerKi, informatrionToSend, error);

}
bool SOLOMotorControllersKvaser::SetSpeedControllerKi(float speedControllerKi)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetSpeedControllerKi(speedControllerKi, error);
}
bool SOLOMotorControllersKvaser::SetMotorDirection(SOLOMotorControllers::Direction motorDirection, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	soloUtils->ConvertToData((long)motorDirection, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_MotorDirection, informatrionToSend, error);

}
bool SOLOMotorControllersKvaser::SetMotorDirection(SOLOMotorControllers::Direction motorDirection)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetMotorDirection(motorDirection, error);
}
bool SOLOMotorControllersKvaser::SetMotorResistance(float motorResistance, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetMotorResistanceInputValidation(motorResistance, error))
	{
		return false;
	}
	soloUtils->ConvertToData(motorResistance, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_MotorResistance, informatrionToSend, error);

}
bool SOLOMotorControllersKvaser::SetMotorResistance(float motorResistance)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetMotorResistance(motorResistance, error);
}
bool SOLOMotorControllersKvaser::SetMotorInductance(float motorInductance, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetMotorInductanceInputValidation(motorInductance, error))
	{
		return false;
	}
	soloUtils->ConvertToData(motorInductance, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_MotorInductance, informatrionToSend, error);

}
bool SOLOMotorControllersKvaser::SetMotorInductance(float motorInductance)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetMotorInductance(motorInductance, error);
}
bool SOLOMotorControllersKvaser::SetMotorPolesCounts(long motorPolesCounts, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetMotorPolesCountsInputValidation(motorPolesCounts, error))
	{
		return false;
	}

	soloUtils->ConvertToData(motorPolesCounts, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_MotorPolesCounts, informatrionToSend, error);

}
bool SOLOMotorControllersKvaser::SetMotorPolesCounts(long motorPolesCounts)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetMotorPolesCounts(motorPolesCounts, error);
}
bool SOLOMotorControllersKvaser::SetIncrementalEncoderLines(long incrementalEncoderLines, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetIncrementalEncoderLinesInputValidation(incrementalEncoderLines, error))
	{
		return false;
	}
	soloUtils->ConvertToData(incrementalEncoderLines, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_IncrementalEncoderLines, informatrionToSend, error);

}
bool SOLOMotorControllersKvaser::SetIncrementalEncoderLines(long incrementalEncoderLines)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetIncrementalEncoderLines(incrementalEncoderLines, error);
}
bool SOLOMotorControllersKvaser::SetSpeedLimit(long speedLimit, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetSpeedLimitInputValidation(speedLimit, error))
	{
		return false;
	}
	soloUtils->ConvertToData(speedLimit, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_SpeedLimit, informatrionToSend, error);

}
bool SOLOMotorControllersKvaser::SetSpeedLimit(long speedLimit)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetSpeedLimit(speedLimit, error);
}
bool SOLOMotorControllersKvaser::SetFeedbackControlMode(SOLOMotorControllers::FeedbackControlMode mode, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	soloUtils->ConvertToData((long)mode, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_FeedbackControlMode, informatrionToSend, error);

}
bool SOLOMotorControllersKvaser::SetFeedbackControlMode(SOLOMotorControllers::FeedbackControlMode mode)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetFeedbackControlMode(mode, error);
}
bool SOLOMotorControllersKvaser::ResetFactory(int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x01 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	return kvaser->CANOpenTransmit(hnd, Address, Object_ResetFactory, informatrionToSend, error);

}
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

bool SOLOMotorControllersKvaser::SetMotorType(SOLOMotorControllers::MotorType motorType, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	soloUtils->ConvertToData((long)motorType, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_MotorType, informatrionToSend, error);

}
bool SOLOMotorControllersKvaser::SetMotorType(SOLOMotorControllers::MotorType motorType)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetMotorType(motorType, error);
}
bool SOLOMotorControllersKvaser::SetControlMode(SOLOMotorControllers::ControlMode controlMode, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	soloUtils->ConvertToData((long)controlMode, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_ControlMode, informatrionToSend, error);

}
bool SOLOMotorControllersKvaser::SetControlMode(SOLOMotorControllers::ControlMode controlMode)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetControlMode(controlMode, error);
}
bool SOLOMotorControllersKvaser::SetCurrentControllerKp(float currentControllerKp, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetCurrentControllerKpInputValidation(currentControllerKp, error))
	{
		return false;
	}
	soloUtils->ConvertToData(currentControllerKp, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_CurrentControllerKp, informatrionToSend, error);

}
bool SOLOMotorControllersKvaser::SetCurrentControllerKp(float currentControllerKp)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetCurrentControllerKp(currentControllerKp, error);
}
bool SOLOMotorControllersKvaser::SetCurrentControllerKi(float currentControllerKi, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetCurrentControllerKiInputValidation(currentControllerKi, error))
	{
		return false;
	}
	soloUtils->ConvertToData(currentControllerKi, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_CurrentControllerKi, informatrionToSend, error);

}
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
bool SOLOMotorControllersKvaser::SetMagnetizingCurrentIdReference(float magnetizingCurrentIdReference, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetMagnetizingCurrentIdReferenceInputValidation(magnetizingCurrentIdReference, error))
	{
		return false;
	}
	soloUtils->ConvertToData(magnetizingCurrentIdReference, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_MagnetizingCurrentIdReference, informatrionToSend, error);

}
bool SOLOMotorControllersKvaser::SetMagnetizingCurrentIdReference(float magnetizingCurrentIdReference)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetMagnetizingCurrentIdReference(magnetizingCurrentIdReference, error);
}
bool SOLOMotorControllersKvaser::SetPositionReference(long positionReference, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetPositionReferenceInputValidation(positionReference, error))
	{
		return false;
	}
	soloUtils->ConvertToData(positionReference, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_PositionReference, informatrionToSend, error);

}
bool SOLOMotorControllersKvaser::SetPositionReference(long positionReference)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetPositionReference(positionReference, error);
}
bool SOLOMotorControllersKvaser::SetPositionControllerKp(float positionControllerKp, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetPositionControllerKpInputValidation(positionControllerKp, error))
	{
		return false;
	}
	soloUtils->ConvertToData(positionControllerKp, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_PositionControllerKp, informatrionToSend, error);

}
bool SOLOMotorControllersKvaser::SetPositionControllerKp(float positionControllerKp)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetPositionControllerKp(positionControllerKp, error);
}
bool SOLOMotorControllersKvaser::SetPositionControllerKi(float positionControllerKi, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetPositionControllerKiInputValidation(positionControllerKi, error))
	{
		return false;
	}
	soloUtils->ConvertToData(positionControllerKi, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_PositionControllerKi, informatrionToSend, error);

}
bool SOLOMotorControllersKvaser::SetPositionControllerKi(float positionControllerKi)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetPositionControllerKi(positionControllerKi, error);
}
bool SOLOMotorControllersKvaser::ResetPositionToZero(int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x01 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	return kvaser->CANOpenTransmit(hnd, Address, Object_ResetPositionToZero, informatrionToSend, error);

}
bool SOLOMotorControllersKvaser::ResetPositionToZero()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::ResetPositionToZero(error);
}
bool SOLOMotorControllersKvaser::OverwriteErrorRegister(int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	return kvaser->CANOpenTransmit(hnd, Address, Object_OverwriteErrorRegister, informatrionToSend, error);

}
bool SOLOMotorControllersKvaser::OverwriteErrorRegister()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::OverwriteErrorRegister(error);
}
bool SOLOMotorControllersKvaser::SetObserverGainBldcPmsm(float observerGain, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetObserverGainBldcPmsmInputValidation(observerGain, error))
	{
		return false;
	}
	soloUtils->ConvertToData(observerGain, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_ObserverGainBldcPmsm, informatrionToSend, error);

}
bool SOLOMotorControllersKvaser::SetObserverGainBldcPmsm(float observerGain)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetObserverGainBldcPmsm(observerGain, error);
}
bool SOLOMotorControllersKvaser::SetObserverGainBldcPmsmUltrafast(float observerGain, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetObserverGainBldcPmsmUltrafastInputValidation(observerGain, error))
	{
		return false;
	}
	soloUtils->ConvertToData(observerGain, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_ObserverGainBldcPmsmUltrafast, informatrionToSend, error);

}
bool SOLOMotorControllersKvaser::SetObserverGainBldcPmsmUltrafast(float observerGain)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetObserverGainBldcPmsmUltrafast(observerGain, error);
}
bool SOLOMotorControllersKvaser::SetObserverGainDc(float observerGain, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetObserverGainDcInputValidation(observerGain, error))
	{
		return false;
	}
	soloUtils->ConvertToData(observerGain, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_ObserverGainDc, informatrionToSend, error);

}
bool SOLOMotorControllersKvaser::SetObserverGainDc(float observerGain)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetObserverGainDc(observerGain, error);
}
bool SOLOMotorControllersKvaser::SetFilterGainBldcPmsm(float filterGain, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetFilterGainBldcPmsmInputValidation(filterGain, error))
	{
		return false;
	}
	soloUtils->ConvertToData(filterGain, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_FilterGainBldcPmsm, informatrionToSend, error);

}
bool SOLOMotorControllersKvaser::SetFilterGainBldcPmsm(float filterGain)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetFilterGainBldcPmsm(filterGain, error);
}
bool SOLOMotorControllersKvaser::SetFilterGainBldcPmsmUltrafast(float filterGain, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetFilterGainBldcPmsmUltrafastInputValidation(filterGain, error))
	{
		return false;
	}
	soloUtils->ConvertToData(filterGain, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_FilterGainBldcPmsmUltrafast, informatrionToSend, error);

}
bool SOLOMotorControllersKvaser::SetFilterGainBldcPmsmUltrafast(float filterGain)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetFilterGainBldcPmsmUltrafast(filterGain, error);
}
bool SOLOMotorControllersKvaser::SetUartBaudrate(SOLOMotorControllers::UartBaudrate baudrate, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	soloUtils->ConvertToData((long)baudrate, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_UartBaudrate, informatrionToSend, error);

}
bool SOLOMotorControllersKvaser::SetUartBaudrate(SOLOMotorControllers::UartBaudrate baudrate)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetUartBaudrate(baudrate, error);
}
bool SOLOMotorControllersKvaser::SensorCalibration(SOLOMotorControllers::PositionSensorCalibrationAction calibrationAction, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	soloUtils->ConvertToData((long)calibrationAction, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_SensorCalibration, informatrionToSend, error);

}
bool SOLOMotorControllersKvaser::SensorCalibration(SOLOMotorControllers::PositionSensorCalibrationAction calibrationAction)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SensorCalibration(calibrationAction, error);
}
bool SOLOMotorControllersKvaser::SetEncoderHallCcwOffset(float encoderHallOffset, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetEncoderHallCcwOffsetInputValidation(encoderHallOffset, error))
	{
		return false;
	}
	soloUtils->ConvertToData(encoderHallOffset, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_EncoderHallCcwOffset, informatrionToSend, error);

}
bool SOLOMotorControllersKvaser::SetEncoderHallCcwOffset(float encoderHallOffset)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetEncoderHallCcwOffset(encoderHallOffset, error);
}
bool SOLOMotorControllersKvaser::SetEncoderHallCwOffset(float encoderHallOffset, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetEncoderHallCwOffsetInputValidation(encoderHallOffset, error))
	{
		return false;
	}
	soloUtils->ConvertToData(encoderHallOffset, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_EncoderHallCwOffset, informatrionToSend, error);

}
bool SOLOMotorControllersKvaser::SetEncoderHallCwOffset(float encoderHallOffset)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetEncoderHallCwOffset(encoderHallOffset, error);
}
bool SOLOMotorControllersKvaser::SetSpeedAccelerationValue(float speedAccelerationValue, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetSpeedAccelerationValueInputValidation(speedAccelerationValue, error))
	{
		return false;
	}
	soloUtils->ConvertToData(speedAccelerationValue, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_SpeedAccelerationValue, informatrionToSend, error);

}
bool SOLOMotorControllersKvaser::SetSpeedAccelerationValue(float speedAccelerationValue)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetSpeedAccelerationValue(speedAccelerationValue, error);
}
bool SOLOMotorControllersKvaser::SetSpeedDecelerationValue(float speedDecelerationValue, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetSpeedDecelerationValueInputValidation(speedDecelerationValue, error))
	{
		return false;
	}
	soloUtils->ConvertToData(speedDecelerationValue, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_SpeedDecelerationValue, informatrionToSend, error);

}
bool SOLOMotorControllersKvaser::SetSpeedDecelerationValue(float speedDecelerationValue)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetSpeedDecelerationValue(speedDecelerationValue, error);
}
bool SOLOMotorControllersKvaser::SetCanbusBaudrate(CanbusBaudrate canbusBaudrate, int& error)
{
	uint8_t informatrionToSend[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	soloUtils->ConvertToData((long)canbusBaudrate, informatrionToSend);
	return kvaser->CANOpenTransmit(hnd, Address, Object_CanbusBaudrate, informatrionToSend, error);

}
bool SOLOMotorControllersKvaser::SetCanbusBaudrate(CanbusBaudrate canbusBaudrate)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::SetCanbusBaudrate(canbusBaudrate, error);
}
////---------------------Read---------------------
long SOLOMotorControllersKvaser::GetReadErrorRegister(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (kvaser->CANOpenReceive(hnd,Address, Object_ReadErrorRegister, informationToSend, informationReceived, error)) {
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

	if (kvaser->CANOpenReceive(hnd, Address, Object_GuardTime, informationToSend, informationReceived, error)) {
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

	if (kvaser->CANOpenReceive(hnd, Address, Object_LifeTimeFactor, informationToSend, informationReceived, error)) {
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

	if (kvaser->CANOpenReceive(hnd, Address, Object_ProducerHeartbeatTime, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}
long SOLOMotorControllersKvaser::GetProducerHeartbeatTime()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetProducerHeartbeatTime(error);
}
long SOLOMotorControllersKvaser::GetDeviceAddress(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_SetDeviceAddress, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}
long SOLOMotorControllersKvaser::GetDeviceAddress()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetDeviceAddress(error);
}
float SOLOMotorControllersKvaser::GetPhaseAVoltage(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_PhaseAVoltage, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}
float SOLOMotorControllersKvaser::GetPhaseAVoltage()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetPhaseAVoltage(error);
}
float SOLOMotorControllersKvaser::GetPhaseBVoltage(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_PhaseBVoltage, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}
float SOLOMotorControllersKvaser::GetPhaseBVoltage()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetPhaseBVoltage(error);
}
float SOLOMotorControllersKvaser::GetPhaseACurrent(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_PhaseACurrent, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}
float SOLOMotorControllersKvaser::GetPhaseACurrent()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetPhaseACurrent(error);
}
float SOLOMotorControllersKvaser::GetPhaseBCurrent(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_PhaseBCurrent, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}
float SOLOMotorControllersKvaser::GetPhaseBCurrent()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetPhaseBCurrent(error);
}
float SOLOMotorControllersKvaser::GetBusVoltage(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_BusVoltage, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}
float SOLOMotorControllersKvaser::GetBusVoltage()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetBusVoltage(error);
}
float SOLOMotorControllersKvaser::GetDcMotorCurrentIm(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_DcMotorCurrentIm, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}
float SOLOMotorControllersKvaser::GetDcMotorCurrentIm()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetDcMotorCurrentIm(error);
}
float SOLOMotorControllersKvaser::GetDcMotorVoltageVm(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_DcMotorVoltageVm, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}
float SOLOMotorControllersKvaser::GetDcMotorVoltageVm()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetDcMotorVoltageVm(error);
}
float SOLOMotorControllersKvaser::GetSpeedControllerKp(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_SpeedControllerKp, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}
float SOLOMotorControllersKvaser::GetSpeedControllerKp()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetSpeedControllerKp(error);
}
float SOLOMotorControllersKvaser::GetSpeedControllerKi(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_SpeedControllerKi, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}
float SOLOMotorControllersKvaser::GetSpeedControllerKi()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetSpeedControllerKi(error);
}
long SOLOMotorControllersKvaser::GetOutputPwmFrequencyKhz(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_OutputPwmFrequencyKhz, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}
long SOLOMotorControllersKvaser::GetOutputPwmFrequencyKhz()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetOutputPwmFrequencyKhz(error);
}
float SOLOMotorControllersKvaser::GetCurrentLimit(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_CurrentLimit, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}
float SOLOMotorControllersKvaser::GetCurrentLimit()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetCurrentLimit(error);
}
float SOLOMotorControllersKvaser::GetQuadratureCurrentIqFeedback(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_QuadratureCurrentIqFeedback, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}
float SOLOMotorControllersKvaser::GetQuadratureCurrentIqFeedback()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetQuadratureCurrentIqFeedback(error);
}
float SOLOMotorControllersKvaser::GetMagnetizingCurrentIdFeedback(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_MagnetizingCurrentIdFeedback, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}
float SOLOMotorControllersKvaser::GetMagnetizingCurrentIdFeedback()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetMagnetizingCurrentIdFeedback(error);
}
long SOLOMotorControllersKvaser::GetMotorPolesCounts(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_MotorPolesCounts, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}
long SOLOMotorControllersKvaser::GetMotorPolesCounts()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetMotorPolesCounts(error);
}
long SOLOMotorControllersKvaser::GetIncrementalEncoderLines(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_IncrementalEncoderLines, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}
long SOLOMotorControllersKvaser::GetIncrementalEncoderLines()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetIncrementalEncoderLines(error);
}
float SOLOMotorControllersKvaser::GetCurrentControllerKp(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_CurrentControllerKp, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}
float SOLOMotorControllersKvaser::GetCurrentControllerKp()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetCurrentControllerKp(error);
}
float SOLOMotorControllersKvaser::GetCurrentControllerKi(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_CurrentControllerKi, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}
float SOLOMotorControllersKvaser::GetCurrentControllerKi()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetCurrentControllerKi(error);
}
float SOLOMotorControllersKvaser::GetBoardTemperature(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_BoardTemperature, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}
float SOLOMotorControllersKvaser::GetBoardTemperature()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetBoardTemperature(error);
}
float SOLOMotorControllersKvaser::GetMotorResistance(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_MotorResistance, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}
float SOLOMotorControllersKvaser::GetMotorResistance()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetMotorResistance(error);
}
float SOLOMotorControllersKvaser::GetMotorInductance(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_MotorInductance, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}
float SOLOMotorControllersKvaser::GetMotorInductance()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetMotorInductance(error);
}
long SOLOMotorControllersKvaser::GetSpeedFeedback(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_SpeedFeedback, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}
long SOLOMotorControllersKvaser::GetSpeedFeedback()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetSpeedFeedback(error);
}
long SOLOMotorControllersKvaser::GetMotorType(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_MotorType, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}
long SOLOMotorControllersKvaser::GetMotorType()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetMotorType(error);
}
long SOLOMotorControllersKvaser::GetFeedbackControlMode(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_FeedbackControlMode, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}
long SOLOMotorControllersKvaser::GetFeedbackControlMode()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetFeedbackControlMode(error);
}
long SOLOMotorControllersKvaser::GetCommandMode(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_CommandMode, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}
long SOLOMotorControllersKvaser::GetCommandMode()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetCommandMode(error);
}
long SOLOMotorControllersKvaser::GetControlMode(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_ControlMode, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}
long SOLOMotorControllersKvaser::GetControlMode()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetControlMode(error);
}
long SOLOMotorControllersKvaser::GetSpeedLimit(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_SpeedLimit, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}
long SOLOMotorControllersKvaser::GetSpeedLimit()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetSpeedLimit(error);
}
float SOLOMotorControllersKvaser::GetPositionControllerKp(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_PositionControllerKp, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}
float SOLOMotorControllersKvaser::GetPositionControllerKp()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetPositionControllerKp(error);
}
float SOLOMotorControllersKvaser::GetPositionControllerKi(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_PositionControllerKi, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}
float SOLOMotorControllersKvaser::GetPositionControllerKi()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetPositionControllerKi(error);
}
long SOLOMotorControllersKvaser::GetPositionCountsFeedback(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_PositionCountsFeedback, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}
long SOLOMotorControllersKvaser::GetPositionCountsFeedback()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetPositionCountsFeedback(error);
}
long SOLOMotorControllersKvaser::GetErrorRegister(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_OverwriteErrorRegister, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}
long SOLOMotorControllersKvaser::GetErrorRegister()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetErrorRegister(error);
}
long SOLOMotorControllersKvaser::GetDeviceFirmwareVersion(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_DeviceFirmwareVersion, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}
long SOLOMotorControllersKvaser::GetDeviceFirmwareVersion()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetDeviceFirmwareVersion(error);
}
long SOLOMotorControllersKvaser::GetDeviceHardwareVersion(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_DeviceHardwareVersion, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}
long SOLOMotorControllersKvaser::GetDeviceHardwareVersion()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetDeviceHardwareVersion(error);
}
float SOLOMotorControllersKvaser::GetTorqueReferenceIq(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_TorqueReferenceIq, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}
float SOLOMotorControllersKvaser::GetTorqueReferenceIq()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetTorqueReferenceIq(error);
}
long SOLOMotorControllersKvaser::GetSpeedReference(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_SpeedReference, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}
long SOLOMotorControllersKvaser::GetSpeedReference()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetSpeedReference(error);
}
float SOLOMotorControllersKvaser::GetMagnetizingCurrentIdReference(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_MagnetizingCurrentIdFeedback, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}
float SOLOMotorControllersKvaser::GetMagnetizingCurrentIdReference()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetMagnetizingCurrentIdReference(error);
}
long SOLOMotorControllersKvaser::GetPositionReference(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_PositionReference, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}
long SOLOMotorControllersKvaser::GetPositionReference()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetPositionReference(error);
}
float SOLOMotorControllersKvaser::GetPowerReference(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_PowerReference, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}
float SOLOMotorControllersKvaser::GetPowerReference()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetPowerReference(error);
}
long SOLOMotorControllersKvaser::GetMotorDirection(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_MotorDirection, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}
long SOLOMotorControllersKvaser::GetMotorDirection()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetMotorDirection(error);
}
float SOLOMotorControllersKvaser::GetObserverGainBldcPmsm(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_ObserverGainBldcPmsm, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}
float SOLOMotorControllersKvaser::GetObserverGainBldcPmsm()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetObserverGainBldcPmsm(error);
}
float SOLOMotorControllersKvaser::GetObserverGainBldcPmsmUltrafast(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_ObserverGainBldcPmsmUltrafast, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}
float SOLOMotorControllersKvaser::GetObserverGainBldcPmsmUltrafast()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetObserverGainBldcPmsmUltrafast(error);
}
float SOLOMotorControllersKvaser::GetObserverGainDc(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_ObserverGainDc, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}
float SOLOMotorControllersKvaser::GetObserverGainDc()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetObserverGainDc(error);
}
float SOLOMotorControllersKvaser::GetFilterGainBldcPmsm(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_FilterGainBldcPmsm, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}
float SOLOMotorControllersKvaser::GetFilterGainBldcPmsm()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetFilterGainBldcPmsm(error);
}
float SOLOMotorControllersKvaser::GetFilterGainBldcPmsmUltrafast(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_FilterGainBldcPmsmUltrafast, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}
float SOLOMotorControllersKvaser::GetFilterGainBldcPmsmUltrafast()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetFilterGainBldcPmsmUltrafast(error);
}
float SOLOMotorControllersKvaser::Get3PhaseMotorAngle(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_3PhaseMotorAngle, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}
float SOLOMotorControllersKvaser::Get3PhaseMotorAngle()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::Get3PhaseMotorAngle(error);
}
float SOLOMotorControllersKvaser::GetEncoderHallCcwOffset(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_EncoderHallCcwOffset, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}
float SOLOMotorControllersKvaser::GetEncoderHallCcwOffset()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetEncoderHallCcwOffset(error);
}
float SOLOMotorControllersKvaser::GetEncoderHallCwOffset(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_EncoderHallCwOffset, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}
float SOLOMotorControllersKvaser::GetEncoderHallCwOffset()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetEncoderHallCwOffset(error);
}
long SOLOMotorControllersKvaser::GetUartBaudrate(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_UartBaudrate, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;
}
long SOLOMotorControllersKvaser::GetUartBaudrate()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetUartBaudrate(error);
}
float SOLOMotorControllersKvaser::GetSpeedAccelerationValue(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_SpeedAccelerationValue, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}
float SOLOMotorControllersKvaser::GetSpeedAccelerationValue()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetSpeedAccelerationValue(error);
}
float SOLOMotorControllersKvaser::GetSpeedDecelerationValue(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_SpeedDecelerationValue, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToFloat(informationReceived));
	}
	return -1.0;
}
float SOLOMotorControllersKvaser::GetSpeedDecelerationValue()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetSpeedDecelerationValue(error);
}
bool SOLOMotorControllersKvaser::CommunicationIsWorking(int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	float temperature = SOLOMotorControllersKvaser::GetBoardTemperature(error);
	if (error == SOLOMotorControllers::Error::noErrorDetected) {
		return true;
	}
	return false;
}
bool SOLOMotorControllersKvaser::CommunicationIsWorking()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersKvaser::CommunicationIsWorking(error);
}
long SOLOMotorControllersKvaser::GetEncoderIndexCounts(int& error)
{
	uint8_t  informationToSend[4] = { 0x00,0x00,0x00,0x00 };
	uint8_t  informationReceived[4] = { 0x00,0x00,0x00,0x00 };
	error = SOLOMotorControllers::Error::noProcessedCommand;

	if (kvaser->CANOpenReceive(hnd, Address, Object_EncoderIndexCounts, informationToSend, informationReceived, error)) {
		return (soloUtils->ConvertToLong(informationReceived));
	}
	return -1;

}
long SOLOMotorControllersKvaser::GetEncoderIndexCounts()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return  SOLOMotorControllersKvaser::GetEncoderIndexCounts(error);
}

void SOLOMotorControllersKvaser::GeneralCanbusRead(uint16_t *ID , uint8_t *DLC, uint8_t *Data)
{
	kvaser->CANOpenGenericReceive(hnd, ID , DLC, Data);
}
void SOLOMotorControllersKvaser::GeneralCanbusWrite(uint16_t ID, uint8_t *DLC, uint8_t *Data, int &error)
{
	kvaser->CANOpenGenericTransmit(hnd, ID , DLC, Data, error);
}