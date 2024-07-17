/**
 *******************************************************************************
 * @file    ReadingTemperature.cpp
 * @authors SOLO Motor Controllers
 * @brief   Reading Temperature
 * 
 * @date    Date: 2024
 * @version 1.3.0
 * *******************************************************************************    
 * @attention
 * Copyright: (c) 2021-2024, SOLO motor controllers project
 * MIT License (see LICENSE file for more details)
 ******************************************************************************* 
 */

// EXAMPLE of how read the SOLO board temperature, 
// every second we print the value of the temperature
#include <iostream>
using std::cout;
using std::endl;

#include <conio.h>
#include "SOLOMotorControllersCanopenKvaser.h" 

// instanciate a SOLO object
SOLOMotorControllers *solo; 

float temperature=0; 
int error;

int main(void) {
  //Initialize the SOLO object
  //Equivalent, avoiding the default parameter of SOLO Device Address:  solo = new SOLOMotorControllersCanopenKvaser(0);
  solo = new SOLOMotorControllersCanopenKvaser();
  
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
