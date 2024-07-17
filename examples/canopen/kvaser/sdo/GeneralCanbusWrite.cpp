/**
 *******************************************************************************
 * @file    GeneralCanbusWrite.cpp
 * @authors SOLO Motor Controllers
 * @brief   General Canbus Write
 * 
 * @date    Date: 2024
 * @version 1.3.0
 * *******************************************************************************    
 * @attention
 * Copyright: (c) 2021-2024, SOLO motor controllers project
 * MIT License (see LICENSE file for more details)
 ******************************************************************************* 
 */

//#include "stdafx.h"
#include <iostream>

#include <conio.h>
#include<windows.h>
#include <string>
using namespace std;
#include "canlib.h"
#include "SOLOMotorControllersCanopenKvaser.h"

// instanciate a SOLO object as Canopen IMPORTANT
SOLOMotorControllersCanopenKvaser *solo; 

//Init writing variable
uint16_t ID_Write = 605;
uint8_t  DLC_Write = 8;
uint8_t  DataWrite[8]  = {0x23,0x01,0x02,0x03,0x04,0x05,0x06,0x07};
int ErrorWrite = false;
string SOLOMotorControllersErrors[10] = {"noErrorDetected","generalError","noProcessedCommand","outOfRengeSetting","packetFailureTrialAttemptsOverflow","recieveTimeOutError","abortObject","abortValue","mcp2515TransmitArbitrationLost","mcp2515TransmitError"};
bool status = FALSE;
int cnt = 0;

int main(void)
{
	//Initialize the SOLO object
	//Equivalent, avoiding the default parameter of SOLO Device Address:  solo = new SOLOMotorControllersCanopenKvaser(0);
	solo = new SOLOMotorControllersCanopenKvaser();

	//TRY CONNECT LOOP
	while(solo->CommunicationIsWorking() == false ){
		std::cout << "Solo connection failed. Retry" << std::endl;
		Sleep(500);
		solo->Connect();
	}
	std::cout << "Solo connected!" << std::endl;

    while(++cnt < 20)
    {
        solo->GeneralCanbusWrite(ID_Write, &DLC_Write, DataWrite, ErrorWrite);
        std::cout << SOLOMotorControllersErrors[ErrorWrite] << std::endl;
        Sleep(1000);
    }
}