/**
 *******************************************************************************
 * @file    PDOReadingSpeedFeedback4Sync.cpp
 * @authors SOLO Motor Controllers
 * @brief   PDO Reading SpeedFeedback with 4 Sync
 * 
 * @date    Date: 2024
 * @version 1.3.0
 * *******************************************************************************    
 * @attention
 * Copyright: (c) 2021-2024, SOLO motor controllers project
 * MIT License (see LICENSE file for more details)
 ******************************************************************************* 
 */

//know more reading the CAN manual or Library Documentation 
//Getting the Speed Feedback after 4 SYNC Message

#include <iostream>
using std::cout;
using std::endl;
#include <conio.h>

#include "SOLOMotorControllersCanopenKvaser.h"


SOLOMotorControllersCanopenKvaser *solo;
int error;

int main(void){
	solo = new SOLOMotorControllersCanopenKvaser(0, SOLOMotorControllers::CanbusBaudrate::rate1000);

	// 1 time needed CONFIGURATION:
	cout << "PdoParameterConfig:" << std::endl;
	PdoParameterConfig config;
	config.parameterName = PdoParameterName::speedFeedback;
	config.parameterCobId = 0x280;
	config.isPdoParameterEnable = true;
	config.isRrtParameterEnable = true;
	config.syncParameterCount = 4; 
	//send the configuration to SOLO
	solo->SetPdoParameterConfig(config, error);	
	Sleep(100);

	// if CONFIGURATION already done you can avoid and use the next commad:
	//solo->pdoParameterCobIdByPdoParameterName[PdoParameterName::speedFeedback] = 0x280;

	//ACTIVE section
    //send the sync messages
	solo->SendPdoSync(error);
	solo->SendPdoSync(error);
	solo->SendPdoSync(error);
	solo->SendPdoSync(error);
	Sleep(50);

    //read the older value in the PDO buffer
    long myRead = solo->GetPdoSpeedFeedback(error);
	std::cout << "READ VALUE: "<< myRead << " ERROR: "<< error <<"\n"; 

    //send the sync messages
	solo->SendPdoSync(error);
	solo->SendPdoSync(error);
	solo->SendPdoSync(error);
	solo->SendPdoSync(error);
	Sleep(50);

    //read the older value in the PDO buffer
    myRead = solo->GetPdoSpeedFeedback(error);
	std::cout << "READ VALUE: "<< myRead << " ERROR: "<< error <<"\n"; 

}