// Copyright: (c) 2021, SOLO motor controllers project
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

//In this example we want: 
//    STEP 1: to print the command mode of SOLO and the error status of the reading operation
//    STEP 2: if we read the command mode without error we want to change the command mode of SOLO.
#include <iostream>
using std::cout;
using std::endl;

#include <conio.h>
#include "SOLOMotorControllersSerial.h"

SOLOMotorControllers *solo; 
int error;
long commandMode;
bool setIsSuccesfull;

int main(void) {
  solo = new SOLOMotorControllersSerial((char*)"COM3");
  
  //Infinite Loop
  while(true){
    Sleep(500);
    
    //STEP 1
    //response : is the Command Mode reading from SOLO device 
    //error : after the execution of the fuction will have the error status of the execution
    commandMode = solo->GetCommandMode(error);
    
    //error is not mandatory, we can call the function without it, as other examples:
    //response = solo->GetCommandMode();
    
    //we print the info:
    std::cout << "COMMAND MODE: " << commandMode << " ERROR: "<< error << std::endl;
    
    //STEP 2
    //if we have no error we want to change the command mode of SOLO
    //we can compare error with SOLOMotorControllersError enum or int value. Equal code: 
    //    error == SOLOMotorControllers::SOLOMotorControllersError::noErrorDetected
    //    error == 0
    if (error == SOLOMotorControllers::Error::noErrorDetected)
    {
      //we check the commandMode readed value.
      //we can compare commandMode with CommandMode enum or int value. Equal code:
      //    commandMode == SOLOMotorControllers::CommandMode::analogue
      //    commandMode == 0
      if(commandMode == SOLOMotorControllers::CommandMode::analogue)
      {
        
        //setIsSuccesfull : set return if the set was succesfull
        //SOLOMotorControllers::CommandMode::digital : is the command mode i want to set to SOLO.
        //error : after the execution of the fuction will have the error status of the execution
        setIsSuccesfull = solo->SetCommandMode(SOLOMotorControllers::CommandMode::digital, error);
        
        //error is not mandatory, we can call the function without it, as for the setIsSuccesfull, other examples:
        //setIsSuccesfull = solo->SetCommandMode(SOLOMotorControllers::CommandMode::digital);
        //solo->SetCommandMode(SOLOMotorControllers::CommandMode::digital, error);
        //solo->SetCommandMode(SOLOMotorControllers::CommandMode::digital);
        
        //we print the info:
        std::cout << "SET COMMAND SUCCESS: " << setIsSuccesfull << " ERROR: "<< error << std::endl;
      }
      else
      {
        //in this situation we want to set analogue as command mode in SOLO
        //we choose the alternative code with less herror and status controlling:
        solo->SetCommandMode(SOLOMotorControllers::CommandMode::analogue);
        
        //Alternative are:
        //setIsSuccesfull = solo->SetCommandMode(SOLOMotorControllers::CommandMode::analogue, error);
        //setIsSuccesfull = solo->SetCommandMode(SOLOMotorControllers::CommandMode::analogue);
        //solo->SetCommandMode(SOLOMotorControllers::CommandMode::digital, error);
      }
    } 
  }
  return 0;
}