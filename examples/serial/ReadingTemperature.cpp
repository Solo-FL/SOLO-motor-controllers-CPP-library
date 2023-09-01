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

// EXAMPLE of how read the SOLO board temperature, 
// every second we print the value of the temperature
#include <iostream>

#include <conio.h>
#include "Kvaser.h"
#include "SOLOMotorControllersKvaser.h" 

// instanciate a SOLO object
SOLOMotorControllers *solo; 

float temperature=0; 
int error;

int main(void) {
	CommunicationInterface* ci = new Kvaser(SOLOMotorControllers::CanbusBaudrate::rate1000);

    //Initialize the SOLO object
    //Equivalent, avoiding the default parameter of SOLO Device Address:  solo = new SOLOMotorControllersKvaser(0);
    solo = new SOLOMotorControllersKvaser(ci);

  //Infinite Loop
  while (true)
  {
    //Reading
    temperature = solo->GetBoardTemperature(error);

    //Print
    std::cout << "Read from SOLO: "<< temperature << " (ERROR: "<<error<<")"<< std::endl;
    Sleep(1000);
  }
  return 0;
}
