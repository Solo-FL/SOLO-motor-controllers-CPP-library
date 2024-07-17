/**
 *******************************************************************************
 * @file    PDOSetTorqueReference1Sync.cpp
 * @authors SOLO Motor Controllers
 * @brief   PDO send TorqueReference with 1 Sync
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
//Setting the Torque Reference to be applied after 1 sync on the Device

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
	cout << "PdoParameterConfig" << std::endl;
	PdoParameterConfig config;
	config.parameterName = PdoParameterName::torqueReferenceIq;
	config.parameterCobId = 0x201;
	config.isPdoParameterEnable = true;
	config.isRrtParameterEnable = true;
	config.syncParameterCount = 1; 
	//send the configuration to SOLO
	solo->SetPdoParameterConfig(config, error);	
	Sleep(100);

	// if CONFIGURATION already done you can avoid and use the next commad:
	//solo->pdoParameterCobIdByPdoParameterName[PdoParameterName::torqueReferenceIq] = 0x201;
	
	//ACTIVE section
    //send the RTR message
	solo->SetPdoTorqueReferenceIq(1, error);
	std::cout << "Set value: 1 ERROR: "<< error <<"\n"; 
	float getValue = solo->GetTorqueReferenceIq(error);
	std::cout << "Get value (before Pdo Sync): "<< getValue <<" ERROR: "<< error <<"\n"; 
	solo->SendPdoSync(error);
	getValue = solo->GetTorqueReferenceIq(error);
	std::cout << "Get value (after Pdo Sync): "<< getValue <<" ERROR: "<< error <<"\n"; 
}