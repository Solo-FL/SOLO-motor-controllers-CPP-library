/**
 *******************************************************************************
 * @file    ReadingBusVoltage.cpp
 * @authors SOLO Motor Controllers
 * @brief   Reading BusVoltage
 * 
 * @date    Date: 2024
 * @version 1.3.0
 * *******************************************************************************    
 * @attention
 * Copyright: (c) 2021-2024, SOLO motor controllers project
 * MIT License (see LICENSE file for more details)
 ******************************************************************************* 
 */

#include <iostream>
using std::cout;
using std::endl;

#include <conio.h>
#include "SOLOMotorControllersCanopenKvaser.h"

int main(void)
{
    float readingValue;
    int error;
    SOLOMotorControllers *solo = new SOLOMotorControllersCanopenKvaser();

    //TRY CONNECT LOOP
    while(solo->CommunicationIsWorking() == false ){
        std::cout << "Solo connection failed. Retry" << std::endl;
        Sleep(500);
        solo->Connect();
    }
    std::cout << "Solo connected!" << std::endl;

    //PRINT LOOP
    while (true)
    {
        readingValue = solo->GetBusVoltage(error);
        if(error == SOLOMotorControllers::Error::noErrorDetected)
        {
            std::cout << "Succesfully read from solo, Value: " << readingValue << std::endl;
        }
        else
        {
            std::cout << "Failed to set solo, Error: " <<  error << std::endl;
        }
    }
}