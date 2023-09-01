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

//#include "stdafx.h"
#include <iostream>

#include <conio.h>
#include<windows.h>
#include <string>
using namespace std;
#include "canlib.h"
#include "Kvaser.h"
#include "SOLOMotorControllersKvaser.h"

// instanciate a SOLO object as Canopen IMPORTANT
SOLOMotorControllersKvaser *solo; 

//Init writing variable
uint16_t ID_Write = 605;
uint8_t  DLC_Write = 8;
uint8_t  DataWrite[8]  = {0x23,0x01,0x02,0x03,0x04,0x05,0x06,0x07};
int ErrorWrite = false;
string SOLOMotorControllersErrors[10] = {"noErrorDetected","generalError","noProcessedCommand","outOfRengeSetting","packetFailureTrialAttemptsOverflow","recieveTimeOutError","Abort_Object","Abort_Value","MCP2515_Transmit_ArbitrationLost","MCP2515_Transmit_Error"};
bool status = FALSE;
int cnt = 0;

int main(void)
{
	CommunicationInterface* ci = new Kvaser(SOLOMotorControllers::CanbusBaudrate::rate1000);

    //Initialize the SOLO object
    //Equivalent, avoiding the default parameter of SOLO Device Address:  solo = new SOLOMotorControllersKvaser(0);
    solo = new SOLOMotorControllersKvaser(ci);

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
