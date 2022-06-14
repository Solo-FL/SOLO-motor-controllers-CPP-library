// Copyright: (c) 2021, SOLO motor controllers project
// GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

/*
*    Title: SOLO Motor Controllers DLL
*    Author: SOLOMotorControllers
*    Date: 2022
*    Code version: 0.0.0
*    Availability: https://github.com/Solo-FL/SOLO-motor-controllers-CPP-library
This Library is made by SOLOMotorControllers.com
To learn more please visit:  https://www.SOLOMotorControllers.com/
*/

#include <iostream>
using std::cout, std::endl;

#include <conio.h>
#include "SoloMotorControllers.h"

int main(void)
{
    boolean status;
    float readingValue;
    int error;
    SOLOMotorControllers solo;

    //TRY CONNECT LOOP
    while(status == false ){
        status = solo.serialSetup(0, (char *)"COM3", 115200);
        if(status)
        {
            std::cout << "Solo Succesfully Conneced! " << std::endl;
        }
        else
        {
            std::cout << "Solo connection failed. Retry" << std::endl;
            Sleep(500);
        }
    }
    
    //PRINT LOOP
    while (true)
    {
        readingValue = solo.GetBusVoltage(error);
        if(error == SOLOMotorControllers::SOLOMotorControllersError::noErrorDetected)
        {
            std::cout << "Succesfully read from solo, Value: " << readingValue << std::endl;
        }
        else
        {
            std::cout << "Failed to set solo, Error: " <<  error << std::endl;
        }
    }
    solo.Disconnect();

    cout << "Press Enter to Exit";
    getch();
    return 0;
}