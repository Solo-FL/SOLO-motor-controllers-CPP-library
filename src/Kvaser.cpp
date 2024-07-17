/**
 *******************************************************************************
 * @file    Kvaser.cpp
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

#include "Kvaser.h"
#include "SOLOMotorControllers.h"

//TEST
// #include <iostream>
// #include <string>
// #include <stdio.h>
// using std::cout;
// using std::cin;
// using std::endl;


Kvaser::Kvaser()
{
	canInitializeLibrary();
}
bool Kvaser::PDOReceive(int hnd, int _address,  uint8_t* _informationReceived, int& error){
	canStatus stat;
	long ID_Read;
	uint8_t  DLC_Read;
	unsigned int dlc, flags;
	uint8_t  rcvMsg[4] = { 0,0,0,0};
	DWORD timestamp;
	uint16_t i = 0;

	//Check message
	
	Sleep(50);
	ID_Read = 0x580 + _address;
	stat = canReadSpecific(hnd, _address, rcvMsg, &dlc, &flags, &timestamp);
	//std::cout << "CANOpenTransmit - canReadSpecific loop. error: "<< stat << " | "<< static_cast<unsigned> (rcvMsg[0]) << "- "<< static_cast<unsigned> (rcvMsg[1]) << "- "<< static_cast<unsigned>(rcvMsg[2]) << "- "<< static_cast<unsigned>(rcvMsg[3]) << "- "<< static_cast<unsigned>(rcvMsg[4]) << "- "<< static_cast<unsigned>(rcvMsg[5]) << "- "<< static_cast<unsigned>(rcvMsg[6]) <<  "- "<< static_cast<unsigned>(rcvMsg[7]) << " \n";
	//std::cout<< "CANOPEN: write stat: "<< stat <<" ID: "<< 0x580 + _address<<" rcvMsg: "<< rcvMsg[0] <<rcvMsg[1]<<rcvMsg[2]<<" dlc: "<< dlc<<" flags: "<< flags <<" \n";
	if (stat == -2)
	{
		error = SOLOMotorControllers::Error::canEmptyBuffer;
		return false;
	}
	if (stat != canOK)
	{
		error = 1;
		return false;
	}

	_informationReceived[0] = rcvMsg[3];
	_informationReceived[1] = rcvMsg[2];
	_informationReceived[2] = rcvMsg[1];
	_informationReceived[3] = rcvMsg[0];

	return true;
}
bool Kvaser::PDOTransmit(int hnd, int _address, uint8_t* _informatrionToSend, int& error)
{
	//std::cout << "PDOTransmit - _address: "<< _address << " \n";
	canStatus stat;
	uint8_t msg[4] = { 	 _informatrionToSend[3],       //Data 4 - LSB First Data 
						 _informatrionToSend[2],       //Data 5
						 _informatrionToSend[1],       //Data 6
						 _informatrionToSend[0]        //Data 7
	};
	stat = canWriteWait(hnd, _address, msg , 4, 0, 100);
	if (stat != canOK) //0x08 is Data Length(DLC)
	{
		//std::cout << "SendPdoSync - not canOK "<<stat<<"\n";
		error = SOLOMotorControllers::Error::generalError;
		return false;
	}
	//std::cout << "SendPdoSync - OK "<<stat<<"\n";
	error = SOLOMotorControllers::Error::noErrorDetected;
	return true;
}
bool Kvaser::SendPdoSync(int hnd, int& error)
{
	canStatus stat;
	stat = canWriteWait(hnd, 128, NULL, 0, 0, 100);
	if (stat != canOK) //0x08 is Data Length(DLC)
	{
		//std::cout << "SendPdoSync - not canOK "<<stat<<"\n";
		error = SOLOMotorControllers::Error::generalError;
		return false;
	}
	//std::cout << "SendPdoSync - OK "<<stat<<"\n";
	error = SOLOMotorControllers::Error::noErrorDetected;
	return true;
}

bool Kvaser::SendPdoRtr(int hnd,int _address, int& error)
{
	canStatus stat;
	stat = canWriteWait(hnd, _address, NULL, 0, canMSG_RTR, 100);
	if (stat != canOK) //0x08 is Data Length(DLC)
	{
		//std::cout << "SendPdoSync - not canOK "<<stat<<"\n";
		error = SOLOMotorControllers::Error::generalError;
		return false;
	}
	//std::cout << "SendPdoSync - OK "<<stat<<"\n";
	error = SOLOMotorControllers::Error::noErrorDetected;
	return true;
}

bool Kvaser::CANOpenTransmit(int hnd, uint8_t _address, uint16_t _object,uint8_t _subIndex, uint8_t* _informatrionToSend, int& error)
{
	canStatus stat;
	long ID_Read;
	uint8_t  DLC_Read;
	unsigned int dlc, flags;
	uint8_t  rcvMsg[8] = { 0,0,0,0,0,0,0,0 };
	DWORD timestamp;
	uint16_t i = 0;
	uint8_t msg[8] = { 	 0x22 ,                        //Data 0 - SDO Write Request
						 (uint8_t)(_object) ,          //Data 1 - Object Index(LSB)
						 (uint8_t)(_object >> 8) ,     //Data 2 - Object Index(MSB)
						 _subIndex,                    //Data 3 - SubIndex
						 _informatrionToSend[3],       //Data 4 - LSB First Data 
						 _informatrionToSend[2],       //Data 5
						 _informatrionToSend[1],       //Data 6
						 _informatrionToSend[0]        //Data 7
	};

	stat = canWriteWait(hnd, 0x600 + _address, msg, 8, 0, 100);
	if (stat != canOK) //0x08 is Data Length(DLC)
	{
		//std::cout << "CANOpenTransmit - canWriteWait error: "<< stat << " \n";
		error = SOLOMotorControllers::Error::generalError;
		return false;
	}
	
	//Check SDO Write Reply
	do{
		Sleep(50);
		ID_Read = 0x580 + _address;
		stat = canReadSpecific(hnd, 0x580 + _address, rcvMsg, &dlc, &flags, &timestamp);
		//std::cout << "CANOpenTransmit - canReadSpecific loop. error: "<< stat << " | "<< static_cast<unsigned> (rcvMsg[0]) << "- "<< static_cast<unsigned> (rcvMsg[1]) << "- "<< static_cast<unsigned>(rcvMsg[2]) << "- "<< static_cast<unsigned>(rcvMsg[3]) << "- "<< static_cast<unsigned>(rcvMsg[4]) << "- "<< static_cast<unsigned>(rcvMsg[5]) << "- "<< static_cast<unsigned>(rcvMsg[6]) <<  "- "<< static_cast<unsigned>(rcvMsg[7]) << " \n";
		//std::cout<< "CANOPEN: write stat: "<< stat <<" ID: "<< 0x580 + _address<<" rcvMsg: "<< rcvMsg[0] <<rcvMsg[1]<<rcvMsg[2]<<" dlc: "<< dlc<<" flags: "<< flags <<" \n";
	}while(
		!((rcvMsg[1] == (uint8_t)(_object)) && (rcvMsg[2] == (uint8_t)(_object >> 8)))  //read the first correct object (lose the not related)
		&& stat == canOK  //exit if error happen
	);

	if(stat != canOK){
		error = SOLOMotorControllers::Error::generalError;
		//std::cout << "CANOpenTransmit - not canOK \n";
		return false;
	}

	//Abort Checking
	if ((ID_Read == (uint16_t)(0x580 + _address))  // Check COB-ID
		&& (rcvMsg[0] == 0x80)                          // Check Byte1  
		&& (rcvMsg[1] == (uint8_t)(_object))            // Check Object Index(LSB)
		&& (rcvMsg[2] == (uint8_t)(_object >> 8)))        // Check Object Index(MSB)                 
	{
		if ((rcvMsg[4] == 0x06) && (rcvMsg[5] == 0x02) && (rcvMsg[6] == 0x00) && (rcvMsg[7] == 0x00))
		{
			error = SOLOMotorControllers::Error::abortObject;
			//std::cout << "CANOpenTransmit - abortObject \n";
			return false;
		}
		else if ((rcvMsg[4] == 0x06) && (rcvMsg[5] == 0x09) && (rcvMsg[6] == 0x00) && (rcvMsg[7] == 0x30))
		{
			error = SOLOMotorControllers::Error::abortValue;
			//std::cout << "CANOpenTransmit - abortValue \n";
			return false;
		}
	} //End Abort Checking 
	//Check ACK
	if ((ID_Read == (uint16_t)(0x580 + _address))  // Check COB-ID
		&& (rcvMsg[0] == 0x60)                          // Check Byte1  
		&& (rcvMsg[1] == (uint8_t)(_object))            // Check Object Index(LSB)
		&& (rcvMsg[2] == (uint8_t)(_object >> 8)))       // Check Object Index(MSB)    
	{
		error = SOLOMotorControllers::Error::noErrorDetected;
		//std::cout << "CANOpenTransmit - success \n";
		return true;
	}
	//std::cout << "CANOpenTransmit - end => error \n";	
	error = SOLOMotorControllers::Error::generalError;		
	return false;
}

bool Kvaser::CANOpenGenericTransmit(int hnd, uint16_t _ID , uint8_t *_DLC, uint8_t* _Data, int& error)
{
	canStatus stat;
	uint8_t ID_High , ID_Low ;

	ID_High = (uint8_t) (_ID >> 3);
    ID_Low  = (uint8_t) ( (_ID << 5) & 0x00E0 );

	stat = canWriteWait(hnd, (ID_High << 8 || ID_Low), _Data, *_DLC, 0, 100);
	if (stat != canOK)
	{
		error = (int) stat;
		return false;
	}
	
	error = (int) stat;
	return true;
}



bool Kvaser::CANOpenReceive(int hnd, uint8_t _address, uint16_t _object,uint8_t _subIndex, uint8_t* _informatrionToSend, uint8_t* _informationReceived, int& error)
{
	canStatus stat;
	long ID_Read = 0;
	uint8_t  DLC_Read;
	unsigned int dlc, flags;
	uint8_t  rcvMsg[8] = { 0,0,0,0,0,0,0,0 };
	DWORD timestamp;
	uint16_t i = 0;

	uint8_t msg[8] = { 0x40 ,                        //Data 0 - SDO Write Request
						 (uint8_t)(_object) ,        //Data 1 - Object Index(LSB)
						 (uint8_t)(_object >> 8) ,   //Data 2 - Object Index(MSB)
						 _subIndex,                  //Data 3 - SubIndex
						 _informatrionToSend[3],     //Data 4 - LSB First Data
						 _informatrionToSend[2],     //Data 5
						 _informatrionToSend[1],     //Data 6
						 _informatrionToSend[0]      //Data 7
	};

	stat = canWriteWait(hnd, 0x600 + _address, msg, 8, 0, 100);
	if (stat != canOK) //0x08 is Data Length(DLC)
	{
		error = SOLOMotorControllers::Error::generalError;
		return false;
	}

	//Check SDO Write Reply
	do{
		Sleep(50);
		stat = canReadSpecific(hnd, 0x580 + _address, rcvMsg, &dlc, &flags, &timestamp);
		//std::cout<< "CANOPEN: read stat: "<< stat <<" ID: "<<  0x580 + _address<<" rcvMsg: "<< rcvMsg[0] <<rcvMsg[1]<<rcvMsg[2]<<" dlc: "<< dlc<<" flags: "<< flags <<" \n";
	}while(
		!((rcvMsg[1] == (uint8_t)(_object)) && (rcvMsg[2] == (uint8_t)(_object >> 8)))  //read the first correct object (lose the not related)
		&& stat == canOK  //exit if error happen
	);

	//Check SDO Write Reply
	// stat = canReadWait(hnd, &ID_Read, rcvMsg, &dlc, &flags, &timestamp, 100);

	// stat = canBusOff(hnd);

	_informationReceived[0] = rcvMsg[7];
	_informationReceived[1] = rcvMsg[6];
	_informationReceived[2] = rcvMsg[5];
	_informationReceived[3] = rcvMsg[4];

	if(stat != canOK){
		error = SOLOMotorControllers::Error::generalError;
		return false;
	}

	//Abort Checking
	if ((ID_Read == (uint16_t)(0x580 + _address))  // Check COB-ID
		&& (rcvMsg[0] == 0x80)                          // Check Byte1  
		&& (rcvMsg[1] == (uint8_t)(_object))            // Check Object Index(LSB)
		&& (rcvMsg[2] == (uint8_t)(_object >> 8)))        // Check Object Index(MSB)                 
	{
		if ((rcvMsg[4] == 0x06) && (rcvMsg[5] == 0x02) && (rcvMsg[6] == 0x00) && (rcvMsg[7] == 0x00))
		{
			error = SOLOMotorControllers::Error::abortObject;
			return false;
		}
		else if ((rcvMsg[4] == 0x06) && (rcvMsg[5] == 0x09) && (rcvMsg[6] == 0x00) && (rcvMsg[7] == 0x30))
		{
			error = SOLOMotorControllers::Error::abortValue;
			return false;
		}
	} //End Abort Checking

	error = SOLOMotorControllers::Error::noErrorDetected;
	return true;
}

bool Kvaser::CANOpenGenericReceive(int hnd, uint16_t *_ID , uint8_t *_DLC, uint8_t* _Data)
{
	canStatus stat;
	long ID_Read;
	uint8_t  DLC_Read;
	unsigned int flags;
	DWORD timestamp;
	uint16_t i = 0;

	*_ID = 0;
	*_DLC = 0;

	stat = canReadWait(hnd, (long *)_ID, _Data, (unsigned int *)_DLC, &flags, &timestamp, 100);

	if (stat != canOK)
	{
		return false;
	}

	if (stat != canOK)
	{
		return false;
	}
	return true;
}