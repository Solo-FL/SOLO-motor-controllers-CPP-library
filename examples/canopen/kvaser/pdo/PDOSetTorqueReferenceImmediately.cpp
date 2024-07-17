/**
 *******************************************************************************
 * @file    PDOSetTorqueReferenceImmediately.cpp
 * @authors SOLO Motor Controllers
 * @brief   PDO Set TorqueReference Immediately
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
//Setting the Torque Reference to be applied Immediately on the Device

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
	config.syncParameterCount = 0; 
	//send the configuration to SOLO
	solo->SetPdoParameterConfig(config, error);	
	Sleep(100);

	// if CONFIGURATION already done you can avoid and use the next commad:
	//solo->pdoParameterCobIdByPdoParameterName[PdoParameterName::torqueReferenceIq] = 0x201;
	
	//ACTIVE section
	solo->SetPdoTorqueReferenceIq(2, error);
	std::cout << "Set value: 2 ERROR: "<< error <<"\n"; 
	float getValue = solo->GetTorqueReferenceIq(error);
	std::cout << "Get value: "<< getValue <<" ERROR: "<< error <<"\n"; 
}