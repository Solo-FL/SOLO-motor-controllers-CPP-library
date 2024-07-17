/**
 *******************************************************************************
 * @file    BLDCDroneRcCar_Sensorless_300000RPM.cpp
 * @authors SOLO Motor Controllers
 * @brief   BLDC Drone-RcCar motor  with Sensorless at 300000RPM
 * 
 * @date    Date: 2024
 * @version 1.3.0
 * *******************************************************************************    
 * @attention
 * Copyright: (c) 2021-2024, SOLO motor controllers project
 * MIT License (see LICENSE file for more details)
 ******************************************************************************* 
 */

//Arduino Article about the same topic: https://www.solomotorcontrollers.com/drive-fast-drone-rc-car-brushless-motors-arduino-solo-sensorless
#include <iostream>
using std::cout;
using std::endl;

#include <conio.h>
#include "SOLOMotorControllersCanopenKvaser.h" 

//instanciate a SOLO object:
SOLOMotorControllers *solo;  

//Desired Switching or PWM Frequency at Output
long pwmFrequency = 79; 

//Motor's Number of Poles
long numberOfPoles = 2; 

//Speed controller Kp
float speedControllerKp = 0.03; 

//Speed controller Ki
float speedControllerKi = 0.001; 

// Current Limit of the Motor
float currentLimit = 32.0; 

// Battery of Bus Voltage
float busVoltage = 0; 

// Desired Speed Reference 
long desiredMotorSpeed = 0; 

// Motor speed feedback
long actualMotorSpeed = 0; 


void soloConfigInit() {
  //In this example, make sure you put SOLO into Closed-Loop by
  // pressing the Piano Switch NO# 5 DOWN. in SOLO UNO
  
  //Initialize the SOLO object
  //Equivalent, avoiding the default parameter of SOLO Device Address:  solo = new SOLOMotorControllersCanopenKvaser(0);
  solo = new SOLOMotorControllersCanopenKvaser();

  //TRY CONNECT LOOP
  while(solo->CommunicationIsWorking() == false ){
    std::cout << "Solo connection failed. Retry" << std::endl;
    Sleep(500);
    solo->Connect();
  }
  std::cout << "Solo connected!" << std::endl;
      
  // Initial Configuration of the device and the Motor
  solo->SetOutputPwmFrequencyKhz(pwmFrequency);
  solo->SetCurrentLimit(currentLimit);
  solo->SetMotorPolesCounts(numberOfPoles);
  solo->SetCommandMode(SOLOMotorControllers::CommandMode::digital);
  solo->SetMotorType(SOLOMotorControllers::MotorType::bldcPmsmUltrafast);
  solo->SetFeedbackControlMode(SOLOMotorControllers::FeedbackControlMode::sensorLessHso);
  solo->SetSpeedControllerKp(speedControllerKp);
  solo->SetSpeedControllerKi(speedControllerKi);
  solo->SetControlMode(SOLOMotorControllers::ControlMode::speedMode);
 
  //run the motor identification to Auto-tune the current controller gains Kp and Ki needed for Torque Loop
  //run ID. always after selecting the Motor Type!
  //ID. doesn't need to be called everytime, only one time after wiring up the Motor will be enough
  //the ID. values will be remembered by SOLO after power recycling
  solo->MotorParametersIdentification(SOLOMotorControllers::Action::start);
  std::cout << "Identifying the Motor" << std::endl;

  //wait at least for 2sec till ID. is done
  Sleep(2000); 
}


int main(void) {
  soloConfigInit();

  //Infinite Loop
  while(true){
    //set the Direction on C.W. 
    solo->SetMotorDirection(SOLOMotorControllers::Direction::clockwise); 

    //set a new reference for speed [RPM]
    desiredMotorSpeed = 10000;
    solo->SetSpeedReference(desiredMotorSpeed);

    // wait till motor reaches to the reference 
    Sleep(5000); 

    actualMotorSpeed = solo->GetSpeedFeedback();
    std::cout << "Motor Speed: "<< actualMotorSpeed << std::endl;
    
    //set the Direction on C.C.W. 
    solo->SetMotorDirection(SOLOMotorControllers::Direction::counterclockwise); 

    //set a new reference for speed [RPM]
    desiredMotorSpeed = 30000;
    solo->SetSpeedReference(desiredMotorSpeed);

    // wait till motor reaches to the reference 
    Sleep(5000);

    actualMotorSpeed = solo->GetSpeedFeedback();
    std::cout << "Motor Speed: "<< actualMotorSpeed << std::endl;
    
    //stop the motor
    desiredMotorSpeed = 0;
    solo->SetSpeedReference(desiredMotorSpeed);
    Sleep(2000);
  }
  return 0;
}