// Copyright: (c) 2021-2022, SOLO motor controllers project
// GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

/*
*    Title: SOLO Motor Controllers CPP Library
*    Author: SOLOMotorControllers
*    Date: 2022
*    Code version: 1.0.0
*    Availability: https://github.com/Solo-FL/SOLO-motor-controllers-CPP-library
This Library is made by SOLOMotorControllers.com
To learn more please visit:  https://www.SOLOMotorControllers.com/
*/

#include <iostream>
#include <string>
#include <stdio.h>


using std::cout;
using std::cin;
using std::endl;

#include <conio.h>
#include<windows.h>

//https://www.kvaser.com/canlib-webhelp/canlib_8h.html
#include "canlib.h"
#include "Kvaser.h"


Kvaser *kvaser;
canHandle hnd;

//Init writing variable
uint16_t ID_Write = 605;
uint8_t  DLC_Write = 8;
uint8_t  DataWrite[8]  = {0x23,0x01,0x02,0x03,0x04,0x05,0x06,0x07};
int ErrorWrite = false;


bool Connect()
{
	canStatus stat;
	canInitializeLibrary();
	hnd = canOpenChannel(0, 0);
	if (hnd < 0)
	{
		return false;
	}
	stat = canSetBusParams(hnd, canBITRATE_1M , 0, 0, 0, 0, 0);
	if(stat == canOK){
		canBusOn(hnd);
		return true;
	}else{
		canClose(hnd);
		return false;
	}
}


void Disconnect()
{
	canBusOff(hnd);
	canClose(hnd);
}

void canlib_canWriteWait(){
	//canBusOn(hnd);
	uint16_t ID_Write = 605;
	uint8_t DataWrite[8]  = {0x23,0x01,0x02,0x03,0x04,0x05,0x06,0x07};
	uint8_t ID_High = (uint8_t) (ID_Write >> 3);
    uint8_t ID_Low  = (uint8_t) ( (ID_Write << 5) & 0x00E0 );

	unsigned int flags = 0;
	flags ^= canMSG_RTR;
	canStatus stat = canWriteWait(hnd, (ID_High << 8 || ID_Low), &DataWrite, 8, flags, 100);
	//canBusOff(hnd);

	std::cout<< "CANOPEN: write stat: "<< stat <<" \n";
}

void canlib_canReadWait(){
	long ID_Read;
	uint8_t  rcvMsg[8] = { 0,0,0,0,0,0,0,0 };
	unsigned int dlc, flags;
	DWORD timestamp;

	canStatus stat = canReadWait(hnd, &ID_Read, rcvMsg, &dlc, &flags, &timestamp, 100);
	std::cout<< "CANOPEN: read stat: "<< stat <<" ID: "<< ID_Read<<" rcvMsg: "<< rcvMsg<<" dlc: "<< dlc<<" flags: "<< flags <<" \n";
}

void canlib_canReadSpecific(){
	long ID_Read = 5;
	uint8_t  rcvMsg[8] = { 0,0,0,0,0,0,0,0 };
	unsigned int dlc, flags;
	DWORD timestamp;

	canStatus stat = canReadSpecific(hnd, ID_Read, rcvMsg, &dlc, &flags, &timestamp);
	std::cout<< "CANOPEN: read stat: "<< stat <<" ID: "<< ID_Read<<" rcvMsg: "<< rcvMsg<<" dlc: "<< dlc<<" flags: "<< flags <<" \n";
}

int main(void){
	char again;

	std::cout<< "PROGRAM: start \n";
	std::cout<< "CANOPEN: Connect \n";
	Connect();

	std::cout<< "CANOPEN: write1 \n";
	canlib_canWriteWait();
	
	std::cout<< "CANOPEN: read loop \n";
	do
	{
		canlib_canReadSpecific();
		cout << "Would you like to read again? (y/n):";
		cin >> again;
	} while (again == 'y');



	std::cout<< "CANOPEN: Disconnect \n";
	Disconnect();

	std::cout<< "PROGRAM: end \n";
	return 0;
}