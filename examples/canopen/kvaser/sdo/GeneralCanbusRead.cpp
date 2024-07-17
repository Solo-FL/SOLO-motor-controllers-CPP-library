/**
 *******************************************************************************
 * @file    GeneralCanbusRead.cpp
 * @authors SOLO Motor Controllers
 * @brief   General Canbus Read
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
uint16_t ID_Read;
uint8_t  DLC_Read;
uint8_t  DataRead[8]  = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
int ErrorWrite = false;
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
		// std::cout << std::showbase  << std::uppercase << std::hex << 255 << std::endl;
		std::cout << "NEW READING" << std::endl;
        solo->GeneralCanbusRead(&ID_Read, &DLC_Read, DataRead);

		if(ID_Read!=0)	//if ID_Read == 0 mean no data recived
		{ 
			//Printing ID
			std::cout << "ID: " << ID_Read << std::endl; 
			//Printing DLC
			std::cout << "DLC: " << std::hex << static_cast<unsigned int>(DLC_Read)  << std::endl;;
			//Printing Data Read
			for(int i = 0 ; i < static_cast<unsigned int>(DLC_Read); i++){
			std::cout << "Byte[" << std::dec << i << "]: " << std::hex <<static_cast<unsigned int>(DataRead[i])<< std::endl;
			}
		}
		Sleep(1000);
    }
}