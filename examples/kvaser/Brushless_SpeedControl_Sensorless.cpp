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

#include <iostream>
using std::cout;
using std::endl;

#include <conio.h>
#include "SOLOMotorControllersKvaser.h" 

//instanciate a SOLO object:
SOLOMotorControllers *solo; 

//Desired Switching or PWM Frequency at Output
long pwmFrequency = 75; 

//Motor's Number of Poles
long numberOfPoles = 8; 

//Speed controller Kp
float speedControllerKp = 0.04; 

//Speed controller Ki
float speedControllerKi = 0.006; 

// Current Limit of the Motor
float currentLimit= 16.55; 

// Battery of Bus Voltage
float busVoltage = 0; 

// Desired Speed Reference 
long desiredMotorSpeed =0; 

// Motor speed feedback
long actualMotorSpeed = 0; 

void soloConfigInit() {
  //In this example, make sure you put SOLO into Closed-Loop by
  // pressing the Piano Switch NO# 5 DOWN. in SOLO UNO
  
  
  //Initialize the SOLO object
  //Equivalent, avoiding the default parameter of SOLO Device Address:  solo = new SOLOMotorControllersKvaser(0);
  solo = new SOLOMotorControllersKvaser();

  //TRY CONNECT LOOP
  while(solo->CommunicationIsWorking() == false ){
    std::cout << "Solo connection failed. Retry" << std::endl;
    Sleep(500);
    solo->Connect();
  }
  std::cout << "Solo connected!" << std::endl;
  
  // Initial Configurations
  solo->SetOutputPwmFrequencyKhz(pwmFrequency);
  solo->SetCurrentLimit(currentLimit);

  //select Digital Mode
  solo->SetCommandMode(SOLOMotorControllers::CommandMode::digital);

  solo->SetMotorType(SOLOMotorControllers::MotorType::bldcPmsm);

  //run the motor identification
  //run ID. always after selecting the Motor Type!
  solo->MotorParametersIdentification(SOLOMotorControllers::Action::start);
  std::cout << "Identifying the Motor" << std::endl;
  //wait at least for 2sec till ID. is done
  Sleep(2000); 
  
  //Operate in Sensor-less Mode
  solo->SetFeedbackControlMode(SOLOMotorControllers::FeedbackControlMode::sensorLess);

  //Control The Speed
  solo->SetControlMode(SOLOMotorControllers::ControlMode::speedMode);

  //Controller Tunings
  solo->SetSpeedControllerKp(speedControllerKp);
  solo->SetSpeedControllerKi(speedControllerKi);
}


int main(void) {
  soloConfigInit();

  //Infinite Loop
  while(true){
    //set the Direction on C.W. 
    solo->SetMotorDirection(SOLOMotorControllers::Direction::clockwise); 

    //set a new reference for speed
    desiredMotorSpeed = 5000;
    solo->SetSpeedReference(desiredMotorSpeed);

    // wait till motor reaches to the reference 
    Sleep(2000); 

    actualMotorSpeed = solo->GetSpeedFeedback();
    std::cout << "Measured Speed[RPM]: "<< actualMotorSpeed << std::endl;

    //set the Direction on C.C.W. 
    solo->SetMotorDirection(SOLOMotorControllers::Direction::clockwise); 

    //set a new reference for speed
    desiredMotorSpeed = 1500;
    solo->SetSpeedReference(desiredMotorSpeed);

    // wait till motor reaches to the reference 
    Sleep(2000);

    actualMotorSpeed = solo->GetSpeedFeedback();
    std::cout << "Measured Speed[RPM]: "<< actualMotorSpeed << std::endl;

    //stop the motor
    desiredMotorSpeed = 0;
    solo->SetSpeedReference(desiredMotorSpeed);
    Sleep(2000);
  }
  return 0;
}