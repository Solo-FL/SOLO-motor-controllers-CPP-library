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

//change val for new value set
float val = 19.43;

float readVal;

int main(void)
{
    bool status = FALSE;
    int error;
    SOLODLLCPP solo1, solo2;

    status = solo1.serialSetup(0, (char *)"COM3", 115200);

    if(status)
    {
        std::cout << "solo1 Succesfully Conneced" << std::endl;
    }

    else
    {
        std::cout << "solo1 Can't Connect" << std::endl;
        // return -1;
    }

    //first object read/write test
    //test write function 
    status = solo1.SetCurrentLimit(val, error);

    if(status)
    {
        std::cout << "Succesfully to Set solo1 Variable.Error is:" << error << std::endl;
    }
    else
    {
        std::cout << "Failed to set solo1 Variable.Error is:" <<  error << std::endl;
    }

    //test read function 
    readVal = solo1.GetCurrentLimit(error);

    if(readVal != -1)
    {
        std::cout << "Readed Value from solo1 is: " << readVal << std::endl;
    }
    else
    {
        std::cout << "Failed to Read solo1 Variable.Error is:" << error << std::endl;
    }
    ////////////////

    ////test conect second object
    status = solo2.serialSetup(0, (char *)"COM5", 115200);

    if(status)
    {
        std::cout << "solo2 Succesfully Conneced" << std::endl;
    }

    else
    {
        std::cout << "solo2 Can't Connect" << std::endl;
        // return -1;
    }

    // second object read/write test
    // test write function 
    status = solo2.SetCurrentLimit(val, error);

    if(status)
    {
        std::cout << "Succesfully Set solo2 Variable" << std::endl;
    }
    else
    {
        std::cout << "Failed to set solo2 Variable.Error is:" << error << std::endl;
    }

    // test read function 
    readVal = solo2.GetCurrentLimit(error);

    if(readVal != -1)
    {
        std::cout << "Readed Value from solo2 is: " << readVal << std::endl;
    }
    else
    {
        std::cout << "Failed to Read solo2 Variable.Error is:" << error << std::endl;
    }

    cout << "Press Enter to Exit";
    getch();
    return 0;
}