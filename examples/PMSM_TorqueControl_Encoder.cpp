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

// The Motor used for Testings: teknic m-2310P-LN-04K
#include <iostream>

#include <conio.h>
#ifdef ARDUINO
#include "MCP2515.hpp"
#else
#include "Kvaser.h"
#endif
#include "SOLOMotorControllersImpl.h" 

//For this Test, make sure you have calibrated your Motor and Hall sensors before
//to know more please read: https://www.solomotorcontrollers.com/how-to-connect-calibrate-incremental-encoder-with-solo/

//instanciate a SOLO object:
SOLOMotorControllers *solo; 

//Desired Switching or PWM Frequency at Output
long pwmFrequency = 20; 

//Motor's Number of Poles
long numberOfPoles = 8; 

// Current Limit of the Motor
float currentLimit = 7.0; 

//Motor's Number of Encoder Lines (PPR pre-quad)
long numberOfEncoderLines = 1000; 

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
  
#ifdef ARDUINO
	CommunicationInterface* ci = new MCP2515(CommunicationInterface::CanbusBaudrate::rate1000);
#else
	CommunicationInterface* ci = new Kvaser(CommunicationInterface::CanbusBaudrate::rate1000);
#endif

    //Initialize the SOLO object
    //Equivalent, avoiding the default parameter of SOLO Device Address:  solo = new SOLOMotorControllersImpl(0);
    solo = new SOLOMotorControllersImpl(ci);

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
  solo->SetIncrementalEncoderLines(numberOfEncoderLines);
  solo->SetCommandMode(SOLOMotorControllers::CommandMode::digital);
  solo->SetMotorType(SOLOMotorControllers::MotorType::bldcPmsm);
  solo->SetFeedbackControlMode(SOLOMotorControllers::FeedbackControlMode::encoders);
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
    solo->SetTorqueReferenceIq(1.3);
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
    solo->SetTorqueReferenceIq(0.85);
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
