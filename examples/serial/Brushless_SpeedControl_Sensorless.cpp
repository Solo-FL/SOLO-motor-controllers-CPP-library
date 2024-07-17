/**
 *******************************************************************************
 * @file    Brushless_SpeedControl_Sensorless.cpp
 * @authors SOLO Motor Controllers
 * @brief   Brushless motor Speed Control with Sensorless
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
#include "SOLOMotorControllersSerial.h"

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
  //Equivalent, avoiding the default parameter of SOLO Device Address:  solo = new SOLOMotorControllersSerial((char*)"COM3",0);
  solo = new SOLOMotorControllersSerial((char*)"COM3");

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
  solo->SetFeedbackControlMode(SOLOMotorControllers::FeedbackControlMode::sensorLessHso);

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