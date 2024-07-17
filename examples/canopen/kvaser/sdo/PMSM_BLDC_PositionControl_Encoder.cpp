/**
 *******************************************************************************
 * @file    PMSM_BLDC_PositionControl_Encoder.cpp
 * @authors SOLO Motor Controllers
 * @brief   PMSM-BLDC motor Position Control with Encoder
 * 
 * @date    Date: 2024
 * @version 1.3.0
 * *******************************************************************************    
 * @attention
 * Copyright: (c) 2021-2024, SOLO motor controllers project
 * MIT License (see LICENSE file for more details)
 ******************************************************************************* 
 */

//The Motor used for Testings: teknic m-2310P-LN-04K
//Similar article based on Arduino: https://www.solomotorcontrollers.com/position-control-brushless-arduino-and-solo/
#include <iostream>
using std::cout;
using std::endl;

#include <conio.h>
#include "SOLOMotorControllersCanopenKvaser.h" 

//For this Test, make sure you have calibrated your Encoder before
//to know more please read: https://www.solomotorcontrollers.com/how-to-connect-calibrate-incremental-encoder-with-solo/

//instanciate a SOLO object:
SOLOMotorControllers *solo; 

//Desired Switching or PWM Frequency at Output
long pwmFrequency = 70; 

//Motor's Number of Poles
long numberOfPoles = 8; 

//Motor's Number of Encoder Lines (PPR pre-quad)
long numberOfEncoderLines = 1000; 

//Speed controller Kp
float speedControllerKp = 0.15; 

//Speed controller Ki
float speedControllerKi = 0.03; 

//Position controller Kp
float positionControllerKp = 0.12;

//Position controller Ki
float positionControllerKi = 0.02; 

// Current Limit of the Motor
float currentLimit = 15.0; 

// Battery or Bus Voltage
float busVoltage = 0; 

// Desired Speed Limit[RPM]
long desiredSpeedLimit = 3000; 

// Desired Position Reference 
long desiredPositionReference = 0; 

// Motor speed feedback
long actualMotorSpeed = 0; 

// Motor position feedback
long actualMotorPosition = 0; 

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
  
  // Initial Configurations
  solo->SetOutputPwmFrequencyKhz(pwmFrequency);
  solo->SetCurrentLimit(currentLimit);
  solo->SetIncrementalEncoderLines(numberOfEncoderLines);

  //select Digital Mode
  solo->SetCommandMode(SOLOMotorControllers::CommandMode::digital);
  solo->SetMotorType(SOLOMotorControllers::MotorType::bldcPmsm);

  //run the motor identification
  //run ID. always after selecting the Motor Type!
  solo->MotorParametersIdentification(SOLOMotorControllers::Action::start);
  std::cout << "Identifying the Motor" << std::endl;

  //wait at least for 2sec till ID. is done
  Sleep(2000); 
  
  //Operate while using Quadrature Encoder
  solo->SetFeedbackControlMode(SOLOMotorControllers::FeedbackControlMode::encoders);

  //Control The Position
  solo->SetControlMode(SOLOMotorControllers::ControlMode::positionMode);

  //Speed Controller Tunings
  solo->SetSpeedControllerKp(speedControllerKp);
  solo->SetSpeedControllerKi(speedControllerKi);

  //Position Controller Tunings
  solo->SetPositionControllerKp(positionControllerKp);
  solo->SetPositionControllerKi(positionControllerKi);
}

int main(void) {
  soloConfigInit();

  //Infinite Loop
  while(true){
    //set a desired Speed Limit for trajectory in RPM
    desiredSpeedLimit = 5000;
    solo->SetSpeedLimit(desiredSpeedLimit);
    
    // set a positive desired Position Reference 
    desiredPositionReference =+500000;
    solo->SetPositionReference(desiredPositionReference);

    // wait till motor reaches to the reference 
    Sleep(3000); 

    actualMotorPosition = solo->GetPositionCountsFeedback();
    std::cout << "Number of Pulses passed: "<< actualMotorPosition << std::endl;


    //set a desired Speed Limit for trajectory in RPM
    desiredSpeedLimit = 1500;
    solo->SetSpeedLimit(desiredSpeedLimit);
    
    // set a negative desired Position Reference 
    desiredPositionReference =-32559;
    solo->SetPositionReference(desiredPositionReference);

    // wait till motor reaches to the reference 
    Sleep(6000); 

    actualMotorPosition = solo->GetPositionCountsFeedback();
    std::cout << "Number of Pulses passed: "<< actualMotorPosition << std::endl;
  }
  return 0;
}
