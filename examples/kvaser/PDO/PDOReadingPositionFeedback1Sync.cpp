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
//Getting the Position Feedback after 1 SYNC Message

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