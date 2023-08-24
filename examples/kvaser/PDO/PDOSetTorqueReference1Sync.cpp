// Copyright: (c) 2021-2022, SOLO motor controllers project
// GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

/*
*    Title: SOLO Motor Controllers CPP Library
*    Author: SOLOMotorControllers
*    Date: 2023
*    Code version: 1.2.0
*    Availability: https://github.com/Solo-FL/SOLO-motor-controllers-CPP-library
This Library is made by SOLOMotorControllers.com
To learn more please visit:  https://www.SOLOMotorControllers.com/
*/

//know more reading the CAN manual or Library Documentation 
//Setting the Torque Reference to be applied after 1 sync on the Device

#include <iostream>
using std::cout;
using std::endl;
#include <conio.h>

#include "SOLOMotorControllersKvaser.h"

SOLOMotorControllersKvaser *solo;
int error;

int main(void){
	solo = new SOLOMotorControllersKvaser(0, SOLOMotorControllers::CanbusBaudrate::rate1000);

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