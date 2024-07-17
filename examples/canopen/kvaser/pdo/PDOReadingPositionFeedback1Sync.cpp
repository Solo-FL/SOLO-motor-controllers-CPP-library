/**
 *******************************************************************************
 * @file    PDOReadingPositionFeedback1Sync.cpp
 * @authors SOLO Motor Controllers
 * @brief   PDO Reading PositionFeedback using 1 Sync
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
//Getting the Position Feedback after 1 SYNC Message

#include <iostream>
using std::cout;
using std::endl;
#include <conio.h>

#include "SOLOMotorControllersCanopenKvaser.h"

SOLOMotorControllersCanopenKvaser *solo;
int error;

int main(void){
	std::cout << "PDOReadingPositionFeedback1Sync Kvaser Test" << std::endl;
	solo = new SOLOMotorControllersCanopenKvaser(0, SOLOMotorControllers::CanbusBaudrate::rate1000);

	// 1 time needed CONFIGURATION:
	cout << "PdoParameterConfig:" << std::endl;
	PdoParameterConfig config;
	config.parameterName = PdoParameterName::positionCountsFeedback;
	config.parameterCobId = 0x280;
	config.isPdoParameterEnable = true;
	config.isRrtParameterEnable = true;
	config.syncParameterCount = 0; 
	//send the configuration to SOLO
	solo->SetPdoParameterConfig(config, error);	
	Sleep(50);

	// if CONFIGURATION already done you can avoid and use the next commad:
	//solo->pdoParameterCobIdByPdoParameterName[PdoParameterName::positionFeedback] = 0x280;

	//ACTIVE section
    //send the sync message
	solo->SendPdoSync(error);
	Sleep(50);

    //read the older value in the PDO buffer
    long myRead = solo->GetPdoPositionCountsFeedback(error);
	std::cout << "READ VALUE: "<< myRead << " ERROR: "<< error <<"\n"; 

    //send the sync message
	solo->SendPdoSync(error);
	Sleep(50);

    //read the older value in the PDO buffer
    myRead = solo->GetPdoPositionCountsFeedback(error);
	std::cout << "READ VALUE: "<< myRead << " ERROR: "<< error <<"\n"; 

}