/**
 *******************************************************************************
 * @file    SOLOMotorControllersUtils.h
 * @authors SOLO Motor Controllers
 * @brief   This file contains all the base functions prototypes for the Solo Drivers
 *          Availability: https://github.com/Solo-FL/SOLO-motor-controllers-ARDUINO-library
 * 
 * @date    Date: 2023
 * @version 1.1.0
 * *******************************************************************************    
 * @attention
 * Copyright: (c) 2021-2023, SOLO motor controllers project
 * GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)
 ******************************************************************************* 
 */

#include "Kvaser.h"
#include "SOLOMotorControllers.h"

Kvaser::Kvaser()
{
	canInitializeLibrary();
}

bool Kvaser::CANOpenTransmit(int hnd, uint8_t _address, uint16_t _object, uint8_t* _informatrionToSend, int& error)
{
	canStatus stat;
	long ID_Read;
	uint8_t  DLC_Read;
	unsigned int dlc, flags;
	uint8_t  rcvMsg[8] = { 0,0,0,0,0,0,0,0 };
	DWORD timestamp;
	uint16_t i = 0;
	uint8_t msg[8] = { 0x22 ,                        //Data 0 - SDO Write Request
						 (uint8_t)(_object) ,          //Data 1 - Object Index(LSB)
						 (uint8_t)(_object >> 8) ,     //Data 2 - Object Index(MSB)
						 0x00 ,                        //Data 3 - SubIndex
						 _informatrionToSend[3],       //Data 4 - LSB First Data 
						 _informatrionToSend[2],       //Data 5
						 _informatrionToSend[1],       //Data 6
						 _informatrionToSend[0]        //Data 7
	};

	canBusOn(hnd);

	stat = canWriteWait(hnd, 0x600 + _address, msg, 8, 0, 100);
	if (stat != canOK) //0x08 is Data Length(DLC)
	{
		return false;
	}
	
	//Check SDO Write Reply
	stat = canReadWait(hnd, &ID_Read, rcvMsg, &dlc, &flags, &timestamp, 100);

	stat = canBusOff(hnd);
	//stat = canClose(hnd);

	//Abort Checking
	if ((ID_Read == (uint16_t)(0x580 + _address))  // Check COB-ID
		&& (rcvMsg[0] == 0x80)                          // Check Byte1  
		&& (rcvMsg[1] == (uint8_t)(_object))            // Check Object Index(LSB)
		&& (rcvMsg[2] == (uint8_t)(_object >> 8)))        // Check Object Index(MSB)                 
	{
		if ((rcvMsg[4] == 0x06) && (rcvMsg[5] == 0x02) && (rcvMsg[6] == 0x00) && (rcvMsg[7] == 0x00))
		{
			error = SOLOMotorControllers::Error::Abort_Object;
			return false;
		}
		else if ((rcvMsg[4] == 0x06) && (rcvMsg[5] == 0x09) && (rcvMsg[6] == 0x00) && (rcvMsg[7] == 0x30))
		{
			error = SOLOMotorControllers::Error::Abort_Value;
			return false;
		}
	} //End Abort Checking 
	//Check ACK
	if ((ID_Read == (uint16_t)(0x580 + _address))  // Check COB-ID
		&& (rcvMsg[0] == 0x60)                          // Check Byte1  
		&& (rcvMsg[1] == (uint8_t)(_object))            // Check Object Index(LSB)
		&& (rcvMsg[2] == (uint8_t)(_object >> 8)))        // Check Object Index(MSB)                 
	{
		error = SOLOMotorControllers::Error::noErrorDetected;
		return true;
	}
	return false;
}

bool Kvaser::CANOpenGenericTransmit(int hnd, uint16_t _ID , uint8_t *_DLC, uint8_t* _Data, int& error)
{
	canStatus stat;
	uint8_t ID_High , ID_Low ;

	ID_High = (uint8_t) (_ID >> 3);
    ID_Low  = (uint8_t) ( (_ID << 5) & 0x00E0 );

	canBusOn(hnd);

	stat = canWriteWait(hnd, (ID_High << 8 || ID_Low), _Data, *_DLC, 0, 100);
	if (stat != canOK)
	{
		error = (int) stat;
		return false;
	}
	
	error = (int) stat;
	return true;
}

bool Kvaser::CANOpenReceive(int hnd, uint8_t _address, uint16_t _object, uint8_t* _informatrionToSend, uint8_t* _informationReceived, int& error)
{
	canStatus stat;
	long ID_Read;
	uint8_t  DLC_Read;
	unsigned int dlc, flags;
	uint8_t  rcvMsg[8] = { 0,0,0,0,0,0,0,0 };
	DWORD timestamp;
	uint16_t i = 0;

	canBusOn(hnd);

	uint8_t msg[8] = { 0x40 ,                      //Data 0 - SDO Write Request
						 (uint8_t)(_object) ,        //Data 1 - Object Index(LSB)
						 (uint8_t)(_object >> 8) ,   //Data 2 - Object Index(MSB)
						 0x00 ,                      //Data 3 - SubIndex
						 _informatrionToSend[3],     //Data 4 - LSB First Data
						 _informatrionToSend[2],     //Data 5
						 _informatrionToSend[1],     //Data 6
						 _informatrionToSend[0]      //Data 7
	};

	stat = canWriteWait(hnd, 0x600 + _address, msg, 8, 0, 100);
	if (stat != canOK) //0x08 is Data Length(DLC)
	{
		return false;
	}

	//Check SDO Write Reply
	stat = canReadWait(hnd, &ID_Read, rcvMsg, &dlc, &flags, &timestamp, 100);

	stat = canBusOff(hnd);

	_informationReceived[0] = rcvMsg[7];
	_informationReceived[1] = rcvMsg[6];
	_informationReceived[2] = rcvMsg[5];
	_informationReceived[3] = rcvMsg[4];

	//Abort Checking
	if ((ID_Read == (uint16_t)(0x580 + _address))  // Check COB-ID
		&& (rcvMsg[0] == 0x80)                          // Check Byte1  
		&& (rcvMsg[1] == (uint8_t)(_object))            // Check Object Index(LSB)
		&& (rcvMsg[2] == (uint8_t)(_object >> 8)))        // Check Object Index(MSB)                 
	{
		if ((rcvMsg[4] == 0x06) && (rcvMsg[5] == 0x02) && (rcvMsg[6] == 0x00) && (rcvMsg[7] == 0x00))
		{
			error = SOLOMotorControllers::Error::Abort_Object;
			return false;
		}
		else if ((rcvMsg[4] == 0x06) && (rcvMsg[5] == 0x09) && (rcvMsg[6] == 0x00) && (rcvMsg[7] == 0x30))
		{
			error = SOLOMotorControllers::Error::Abort_Value;
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

	canBusOn(hnd);

	stat = canReadWait(hnd, (long *)_ID, _Data, (unsigned int *)_DLC, &flags, &timestamp, 100);

	if (stat != canOK)
	{
		return false;
	}

	stat = canBusOff(hnd);

	if (stat != canOK)
	{
		return false;
	}
	return true;
}


