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
uint16_t ID_Read;
uint8_t  DLC_Read;
uint8_t  DataRead[8]  = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
int ErrorWrite = false;
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
