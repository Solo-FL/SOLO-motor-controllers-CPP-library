/**
 *******************************************************************************
 * @file    ESPTwai.cpp
 ******************************************************************************* 
 */

#include "ESPTwai.h"
#include "log.h"
#include "SOLOMotorControllers.h"

ESPTwai::ESPTwai(long canBaudrate, bool autoConnect)
{
    // TODO: do we need a baudrate and how to set it up?
    // switch (canBaudrate)
	// {
	// case CommunicationInterface::CanbusBaudrate::rate1000:
	// 	canBaudrate = canBITRATE_1M;
	// 	break;
	// case CommunicationInterface::CanbusBaudrate::rate500:
	// 	canBaudrate = canBITRATE_500K;
	// 	break;
	// case CommunicationInterface::CanbusBaudrate::rate250:
	// 	canBaudrate = canBITRATE_250K;
	// 	break;
	// case CommunicationInterface::CanbusBaudrate::rate125:
	// 	canBaudrate = canBITRATE_125K;
	// 	break;
	// case CommunicationInterface::CanbusBaudrate::rate100:
	// 	canBaudrate = canBITRATE_100K;
	// 	break;
	// default:
	// 	canBaudrate = canBITRATE_1M;
	// 	break;
	// }
    // mCanBaudrate = canBaudrate;
}

bool ESPTwai::PDOReceive(int _address,  uint8_t* _informationReceived, int& error){
	CANopenNodeStatusTypeDef_t stat;

	//Check message
	Sleep(50);
    CO_CANrxMsg_t_t canRxPacket;
    stat = xCANReceiveMessage(&canRxPacket);

	if (stat == CAN_OPEN_NODE_ERROR)
	{
		error = SOLOMotorControllers::Error::canEmptyBuffer;
		return false;
	}
	_informationReceived[0] = canRxPacket.data[3];
	_informationReceived[1] = canRxPacket.data[2];
	_informationReceived[2] = canRxPacket.data[1];
	_informationReceived[3] = canRxPacket.data[0];

	return true;
}
bool ESPTwai::PDOTransmit(int _address, uint8_t* _informatrionToSend, int& error)
{
    CO_CANtx_t canTxPacket;
    canTxPacket.ident = 0x600 + _address;
    canTxPacket.DLC   = 4;
    // FIXME: do we have to reverse the bytes (LSB / MSB)?
    canTxPacket.data  = {_informatrionToSend[3],       //Data 4 - LSB First Data 
						 _informatrionToSend[2],       //Data 5
                         _informatrionToSend[1],       //Data 6
                         _informatrionToSend[0]        //Data 7
	};
    CANopenNodeStatusTypeDef_t stat = xCANTransmitMessage(&canTxPacket);

	if (stat != CAN_OPEN_NODE_OK) //0x08 is Data Length(DLC)
	{
		//std::cout << "SendPdoSync - not canOK "<<stat<<"\n";
		error = SOLOMotorControllers::Error::generalError;
		return false;
	}
	//std::cout << "SendPdoSync - OK "<<stat<<"\n";
	error = SOLOMotorControllers::Error::noErrorDetected;
	return true;
}

bool ESPTwai::SendPdoSync(int& error)
{
    CO_CANtx_t canTxPacket;
    canTxPacket.ident = 128;
    canTxPacket.DLC   = 0;
    canTxPacket.data  = { 0,0,0,0,0,0,0,0 };

    CANopenNodeStatusTypeDef_t stat = xCANTransmitMessage(&canTxPacket);

	if (stat != CAN_OPEN_NODE_OK) //0x08 is Data Length(DLC)
	{
		error = SOLOMotorControllers::Error::generalError;
		return false;
	}
	//std::cout << "SendPdoSync - OK "<<stat<<"\n";
	error = SOLOMotorControllers::Error::noErrorDetected;
	return true;
}

bool ESPTwai::SendPdoRtr(int _address, int& error)
{
    CO_CANtx_t canTxPacket;
    canTxPacket.ident = _address;
    canTxPacket.DLC   = 0;
    canTxPacket.data  = { 0,0,0,0,0,0,0,0 };
    // FIXME: how to set RTR flag in Can message
    //        RTR is the lowest bit of the COB-ID
    //        4 bit function code, 7 bit node ID, 1 bit RTR, 4 bit Data length, 8 byte data

    CANopenNodeStatusTypeDef_t stat = xCANTransmitMessage(&canTxPacket);

	if (stat != CAN_OPEN_NODE_OK) //0x08 is Data Length(DLC)
	{
		//std::cout << "SendPdoSync - not canOK "<<stat<<"\n";
		error = SOLOMotorControllers::Error::generalError;
		return false;
	}
	//std::cout << "SendPdoSync - OK "<<stat<<"\n";
	error = SOLOMotorControllers::Error::noErrorDetected;
	return true;
}

CO_CANtx_t ESPTwai::getSDOPacket(SDOrequest rw, uint8_t _address, uint16_t _object,uint8_t _subIndex, uint8_t* _informatrionToSend) const {
    CO_CANtx_t canTxPacket;
    canTxPacket.ident = 0x600 + _address;
    canTxPacket.DLC   = 8;
    // FIXME: do we have to reverse the bytes (LSB / MSB)?
    canTxPacket.data  = {SDOrequest::write,           //Data 0 - SDO Write Request
						(uint8_t)(_object) ,          //Data 1 - Object Index(LSB)
						(uint8_t)(_object >> 8) ,     //Data 2 - Object Index(MSB)
						_subIndex,                    //Data 3 - SubIndex
						_informatrionToSend[3],       //Data 4 - LSB First Data 
						_informatrionToSend[2],       //Data 5
						_informatrionToSend[1],       //Data 6
						_informatrionToSend[0]        //Data 7
	};
    return canTxPacket;
}

bool ESPTwai::CANOpenTransmit(uint8_t _address, uint16_t _object,uint8_t _subIndex, uint8_t* _informatrionToSend, int& error)
{
    CO_CANtx_t canTxPacket = getSDOPacket(SDOrequest::write, _address, _object, _subIndex, _informationToSend);

    CANopenNodeStatusTypeDef_t stat = xCANTransmitMessage(&canTxPacket);
	if (stat != CAN_OPEN_NODE_OK) //0x08 is Data Length(DLC)
	{
		LOG_INFO("CANOpenTransmit failed\n");
		error = SOLOMotorControllers::Error::generalError;
		return false;
	}
	
    CO_CANrxMsg_t_t canRxPacket;

    // FIXME: in CANOpen you can setup filters and receiving should be interrupt
    //        handled! 
	//Check SDO Write Reply
	do{
		Sleep(50);
		stat = xCANReceiveMessage(&canRxPacket);
	}while(
		!((canRxPacket.data[1] == (uint8_t)(_object)) && (canRxPacket.data[2] == (uint8_t)(_object >> 8)))  //read the first correct object (lose the not related)
		&& stat == CAN_OPEN_NODE_OK  //exit if error happen
	);

	if(stat != canOK){
		error = SOLOMotorControllers::Error::generalError;
        LOG_INFO("Receiving response to CanTransmit failed\n");
		return false;
	}

	//Abort Checking
	if ((canRxPacket.ident == (uint16_t)(0x580 + _address))  // Check COB-ID
		&& (canRxPacket.data[0] == 0x80)                          // Check Byte1  
		&& (canRxPacket.data[1] == (uint8_t)(_object))            // Check Object Index(LSB)
		&& (canRxPacket.data[2] == (uint8_t)(_object >> 8)))        // Check Object Index(MSB)                 
	{
		if ((canRxPacket.data[4] == 0x06) && (canRxPacket.data[5] == 0x02)
            && (canRxPacket.data[6] == 0x00) && (canRxPacket.data[7] == 0x00))
		{
			error = SOLOMotorControllers::Error::Abort_Object;
			LOG_INFO("CANOpenTransmit - Abort_Object \n");
			return false;
		}
		else if ((canRxPacket.data[4] == 0x06) && (canRxPacket.data[5] == 0x09)
                 && (canRxPacket.data[6] == 0x00) && (canRxPacket.data[7] == 0x30))
		{
			error = SOLOMotorControllers::Error::Abort_Value;
			LOG_INFO("CANOpenTransmit - Abort_Value \n");
			return false;
		}
	}
    //End Abort Checking 
	//Check ACK
	if ((canRxPacket.ident == (uint16_t)(0x580 + _address))  // Check COB-ID
		&& (canRxPacket.data[0] == 0x60)                          // Check Byte1  
		&& (canRxPacket.data[1] == (uint8_t)(_object))            // Check Object Index(LSB)
		&& (canRxPacket.data[2] == (uint8_t)(_object >> 8)))       // Check Object Index(MSB)    
	{
		error = SOLOMotorControllers::Error::noErrorDetected;
		LOG_INFO("CANOpenTransmit - success \n)";
		return true;
	}
    LOG_INFO("CANOpenTransmit - end => error \n");	
	error = SOLOMotorControllers::Error::generalError;		
	return false;
}

bool ESPTwai::CANOpenGenericTransmit(uint16_t _ID , uint8_t *_DLC, uint8_t* _Data, int& error)
{
	uint8_t ID_High = (uint8_t) (_ID >> 3);
    uint8_t ID_Low  = (uint8_t) ( (_ID << 5) & 0x00E0 );

    CO_CANtx_t canTxPacket;
    canTxPacket.ident = (ID_High << 8 || ID_Low)
    canTxPacket.DLC   = *_DLC;
    // FIXME: do we have to reverse the bytes (LSB / MSB)?
    for (size_t i = 0; i < std::min(8, canTxPacket.DLC); ++i) {
        canTxPacket.data[i]  = _Data[i];
	};

    CANopenNodeStatusTypeDef_t stat = xCANTransmitMessage(&canTxPacket);
    if (stat != CAN_OPEN_NODE_OK) 
	{
		error = (int) stat;
		return false;
	}	
	error = (int) stat;
	return true;
}



bool ESPTwai::CANOpenReceive(uint8_t _address, uint16_t _object,uint8_t _subIndex, uint8_t* _informatrionToSend, uint8_t* _informationReceived, int& error)
{
    CO_CANtx_t canTxPacket = getSDOPacket(SDOrequest::read, _address, _object, _subIndex, _informationToSend);

    CANopenNodeStatusTypeDef_t stat = xCANTransmitMessage(&canTxPacket);
	if (stat != CAN_OPEN_NODE_OK) //0x08 is Data Length(DLC)
        {
            LOG_INFO("CANOpenReceive failed\n");
            error = SOLOMotorControllers::Error::generalError;
            return false;
        }

	//Check SDO Read Reply
    CO_CANrxMsg_t_t canRxPacket;

    // FIXME: in CANOpen you can setup filters and receiving should be interrupt
    //        handled! 
	do{
		Sleep(50);
		stat = xCANReceiveMessage(&canRxPacket);
	}while(
		!((canRxPacket.data[1] == (uint8_t)(_object)) && (canRxPacket.data[2] == (uint8_t)(_object >> 8)))  //read the first correct object (lose the not related)
		&& stat == CAN_OPEN_NODE_OK  //exit if error happen
	);

	//Check SDO Write Reply
	// stat = canReadWait(hnd, &ID_Read, rcvMsg, &dlc, &flags, &timestamp, 100);

	// stat = canBusOff(hnd);

	_informationReceived[0] = canRxPacket.data[7];
	_informationReceived[1] = canRxPacket.data[6];
	_informationReceived[2] = canRxPacket.data[5];
	_informationReceived[3] = canRxPacket.data[4];

	if(stat != CAN_OPEN_NODE_OK){
		error = SOLOMotorControllers::Error::generalError;
		return false;
	}

    if ((canRxPacket.ident == (uint16_t)(0x580 + _address))  // Check COB-ID
		&& (canRxPacket.data[0] == 0x80)                          // Check Byte1  
		&& (canRxPacket.data[1] == (uint8_t)(_object))            // Check Object Index(LSB)
		&& (canRxPacket.data[2] == (uint8_t)(_object >> 8)))       // Check Object Index(MSB)
        {
            if ((canRxPacket.data[4] == 0x06) && (canRxPacket.data[5] == 0x02)
                && (canRxPacket.data[6] == 0x00) && (canRxPacket.data[7] == 0x00))
                {
                    error = SOLOMotorControllers::Error::Abort_Object;
                    return false;
                }
            else if ((canRxPacket.data[4] == 0x06) && (canRxPacket.data[5] == 0x09)
                     && (canRxPacket.data[6] == 0x00) && (canRxPacket.data[7] == 0x30))
                {
                    error = SOLOMotorControllers::Error::Abort_Value;
                    return false;
                }
        } //End Abort Checking
	error = SOLOMotorControllers::Error::noErrorDetected;
	return true;
}

bool ESPTwai::CANOpenGenericReceive(uint16_t *_ID , uint8_t *_DLC, uint8_t* _Data)
{
	*_ID = 0;
	*_DLC = 0;

	stat = canReadWait(hnd, (long *)_ID, _Data, (unsigned int *)_DLC, &flags, &timestamp, 100);

    CO_CANrxMsg_t_t canRxPacket{0};
    CANopenNodeStatusTypeDef_t stat = xCANReceiveMessage(&canRxPacket);
    *_ID = canRxPackage.ident;
    *_DLC = canRxPackage.DLC;
    for (size_t i = 0; i < std::min(8, canRxPacket.DLC); ++i) {
        _Data[i] = canRxPacket.data[i];
	};
	if(stat != CAN_OPEN_NODE_OK){
	{
		return false;
	}
	return true;
}
