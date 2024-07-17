/**
 *******************************************************************************
 * @file    BLDC_TorqueControl_HALL.cpp
 * @authors SOLO Motor Controllers
 * @brief   BLDC motor Torque Control with HALL sensor
 * 
 * @date    Date: 2024
 * @version 1.3.0
 * *******************************************************************************    
 * @attention
 * Copyright: (c) 2021-2024, SOLO motor controllers project
 * MIT License (see LICENSE file for more details)
 ******************************************************************************* 
 */

// The Motor used for Testings: DB56C036030-A
#include <iostream>
using std::cout;
using std::endl;

#include <conio.h>
#include "SOLOMotorControllersCanopenKvaser.h" 

//For this Test, make sure you have calibrated your Motor and Hall sensors before
//to know more please read: https://www.solomotorcontrollers.com/hall-sensors-to-solo-for-controlling-speed-torque-brushless-motor/

//instanciate a SOLO object:
SOLOMotorControllers *solo;  

//Desired Switching or PWM Frequency at Output
long pwmFrequency = 20; 

//Motor's Number of Poles
long numberOfPoles = 8; 

// Current Limit of the Motor
float currentLimit = 10.0; 

// Battery or Bus Voltage
float busVoltage = 0; 

// Motor Torque feedback
float actualMotorTorque = 0; 

// Motor speed feedback
long actualMotorSpeed = 0; 

// Motor position feedback
long actualMotorPosition = 0; 

void soloConfigInit() {
  //In this example, make sure you put SOLO into Closed-Loop Mode
  
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
  solo->SetMotorType(SOLOMotorControllers::MotorType::bldcPmsm);
  solo->SetFeedbackControlMode(SOLOMotorControllers::FeedbackControlMode::hallSensors);
  solo->SetControlMode(SOLOMotorControllers::ControlMode::torqueMode);
  
  //run the motor identification to Auto-tune the current controller gains Kp and Ki needed for Torque Controlling
  //run ID. always after selecting the Motor Type!
  //ID. doesn't need to be called everytime, only one time after connection of a new motor it will be  enough
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
    //set the Direction on C.C.W. 
    solo->SetMotorDirection(SOLOMotorControllers::Direction::counterclockwise);
    //set an arbitrary Positive torque reference 
    solo->SetTorqueReferenceIq(1.2);
    // wait till motor reaches to the reference 
    Sleep(100);
    actualMotorTorque = solo->GetQuadratureCurrentIqFeedback();
    std::cout << "Measured Iq/Torque[A]: "<< actualMotorTorque << std::endl;

    // wait for the motor to speed up naturally  
    Sleep(1000);
    actualMotorSpeed = solo->GetSpeedFeedback();
    std::cout << "Measured Speed[RPM]: "<< actualMotorSpeed << std::endl;

    // wait for the motor to speed up naturally  
    Sleep(3000);
    //set the Direction on C.W. 
    solo->SetMotorDirection(SOLOMotorControllers::Direction::clockwise);
    //set an arbitrary Positive torque reference 
    solo->SetTorqueReferenceIq(0.8);
    // wait till motor reaches to the reference 
    
    Sleep(100);
    actualMotorTorque = solo->GetQuadratureCurrentIqFeedback();
    std::cout << "Measured Iq/Torque[A]: "<< actualMotorTorque << std::endl;

    // wait for the motor to speed up naturally  
    Sleep(1000);
    actualMotorSpeed = solo->GetSpeedFeedback();
    std::cout << "Measured Speed[RPM]: "<< actualMotorSpeed << std::endl;

    // wait for the motor to speed up naturally  
    Sleep(3000);
  }
  return 0;
}
