// Copyright: (c) 2021-2022, SOLO motor controllers project
// GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

/*
*    Title: SOLO Motor Controllers DLL
*    Author: SOLOMotorControllers
*    Date: 2022
*    Code version: 1.0.0
*    Availability: https://github.com/Solo-FL/SOLO-motor-controllers-CPP-library
This Library is made by SOLOMotorControllers.com
To learn more please visit:  https://www.SOLOMotorControllers.com/
*/

#include <iostream>
#include <string>
#include <stdio.h>
#include <time.h>
#include <fstream>
using std::cout;
using std::endl;

#include "SOLOMotorControllersKvaser.h"

#include <conio.h>
#include<windows.h>
#include "canlib.h"

const std::string currentDateTime();

int TestCommunication();
int TestCommandMode(SOLOMotorControllers::CommandMode setValue);
int TestCurrentLimit(float setValue);
int TestTorqueReferenceIq(float setValue);
int TestSpeedReference(long setValue);
int TestPowerReference(float setValue);
int TestMotorParametersIdentification(SOLOMotorControllers::Action setValue);
int TestOutputPwmFrequencyKhz(long setValue);
int TestSpeedControllerKp(float setValue);
int TestSpeedControllerKi(float setValue);
int TestMotorDirection(SOLOMotorControllers::Direction setValue);
int TestMotorResistance(float setValue);
int TestMotorInductance(float setValue);
int TestMotorPolesCounts(long setValue);
int TestIncrementalEncoderLines(long setValue);
int TestSpeedLimit(long setValue);
int TestFeedbackControlMode(SOLOMotorControllers::FeedbackControlMode setValue);
int TestMotorType(SOLOMotorControllers::MotorType setValue);
int TestControlMode(SOLOMotorControllers::ControlMode setValue);
int TestCurrentControllerKp(float setValue);
int TestCurrentControllerKi(float setValue);
int TestMonitoringMode(bool setValue);
int TestMagnetizingCurrentIdReference(float setValue);
int TestPositionReference(long setValue);
int TestPositionControllerKp(float setValue);
int TestPositionControllerKi(float setValue);
int TestResetPositionToZero();
int TestOverwriteErrorRegister();
int TestObserverGainBldcPmsm(float setValue);
int TestObserverGainBldcPmsmUltrafast(float setValue);
int TestObserverGainDc(float setValue);
int TestFilterGainBldcPmsm(float setValue);
int TestFilterGainBldcPmsmUltrafast(float setValue);
int TestUartBaudrate(SOLOMotorControllers::UartBaudrate setValue);
int TestSensorCalibration(SOLOMotorControllers::PositionSensorCalibrationAction setValue);
int TestEncoderHallCcwOffset(float setValue);
int TestEncoderHallCwOffset(float setValue);
int TestSpeedAccelerationValue(float setValue);
int TestSpeedDecelerationValue(float setValue);
int TestCanbusBaudrate(SOLOMotorControllers::CanbusBaudrate setValue);
int TestGetPhaseAVoltage();
int TestGetPhaseBVoltage();
int TestGetPhaseACurrent();
int TestGetPhaseBCurrent();
int TestGetBusVoltage();
int TestGetDcMotorCurrentIm();
int TestGetDcMotorVoltageVm();
int TestGetQuadratureCurrentIqFeedback();
int TestGetMagnetizingCurrentIdFeedback();
int TestGetBoardTemperature();
int TestGetSpeedFeedback();
int TestGetPositionCountsFeedback();
int TestGetErrorRegister();
int TestGetDeviceFirmwareVersion();
int TestGetDeviceHardwareVersion();
int TestGet3PhaseMotorAngle();
int TestGetEncoderIndexCounts();

//////Kvaser Functions
int TestGuardTime(long setValue);
int TestLifeTimeFactor(long setValue);
int TestProducerHeartbeatTime(long setValue);


SOLOMotorControllersKvaser *solo;
std::ofstream file;
int successTest =0;
int totalTest =0;
float epsilon = 0.001;

int main(void){
	file.open ("RESULT_SOLOMotorControllersKvaser.txt");
	
	successTest+= TestCommunication();
	totalTest++;

	successTest+= TestCommandMode(SOLOMotorControllers::CommandMode::digital);
	totalTest++;
	
	successTest+= TestCurrentLimit(11.5);
	totalTest++;

	successTest+= TestTorqueReferenceIq(5.6);
	totalTest++;

	successTest+= TestSpeedReference(5000);
	totalTest++;

	successTest+= TestPowerReference(10);
	totalTest++;

	successTest+= TestMotorParametersIdentification(SOLOMotorControllers::Action::start);
	totalTest++;

	successTest+= TestOutputPwmFrequencyKhz(20);
	totalTest++;

	successTest+= TestSpeedControllerKp(2.4);
	totalTest++;

	successTest+= TestSpeedControllerKi(7.35);
	totalTest++;

	successTest+= TestMotorDirection(SOLOMotorControllers::Direction::clockwise);
	totalTest++;

	successTest+= TestMotorResistance(5);
	totalTest++;

	successTest+= TestMotorInductance(0.1);
	totalTest++;

	successTest+= TestMotorPolesCounts(10);
	totalTest++;

	successTest+= TestIncrementalEncoderLines(100);
	totalTest++;

	successTest+= TestSpeedLimit(1000);
	totalTest++;

	successTest+= TestFeedbackControlMode(SOLOMotorControllers::FeedbackControlMode::encoders);
	totalTest++;

	successTest+= TestMotorType(SOLOMotorControllers::MotorType::dc);
	totalTest++;

	successTest+= TestControlMode(SOLOMotorControllers::ControlMode::positionMode);
	totalTest++;

	successTest+= TestCurrentControllerKp(0.88);
	totalTest++;

	successTest+= TestCurrentControllerKi(.65);
	totalTest++;

	// successTest+= TestMonitoringMode(false);
	// totalTest++;

	successTest+= TestMagnetizingCurrentIdReference(12);
	totalTest++;

	successTest+= TestPositionReference(0.5);
	totalTest++;

	successTest+= TestPositionControllerKp(3000);
	totalTest++;

	successTest+= TestPositionControllerKi(2500);
	totalTest++;

	successTest+= TestResetPositionToZero();
	totalTest++;

	successTest+= TestOverwriteErrorRegister();
	totalTest++;

	successTest+= TestObserverGainBldcPmsm(3);
	totalTest++;

	successTest+= TestObserverGainBldcPmsmUltrafast(3.3);
	totalTest++;

	successTest+= TestObserverGainDc(100);
	totalTest++;

	successTest+= TestFilterGainBldcPmsm(4200);
	totalTest++;

	successTest+= TestFilterGainBldcPmsmUltrafast(4550);
	totalTest++;

	successTest+= TestUartBaudrate(SOLOMotorControllers::UartBaudrate::rate115200);
	totalTest++;

	successTest+= TestSensorCalibration(SOLOMotorControllers::PositionSensorCalibrationAction::incrementalEncoderStartCalibration);
	totalTest++;

	successTest+= TestEncoderHallCcwOffset(0.2);
	totalTest++;

	successTest+= TestEncoderHallCwOffset(0.3);
	totalTest++;

	successTest+= TestSpeedAccelerationValue(200);
	totalTest++;

	successTest+= TestSpeedDecelerationValue(220);
	totalTest++;

	// successTest+= TestCanbusBaudrate(SOLOMotorControllers::CanbusBaudrate::rate1000);
	// totalTest++;

	successTest+= TestGetPhaseAVoltage();
	totalTest++;

	successTest+= TestGetPhaseBVoltage();
	totalTest++;

	successTest+= TestGetPhaseACurrent();
	totalTest++;

	successTest+= TestGetPhaseBCurrent();
	totalTest++;

	successTest+= TestGetBusVoltage();
	totalTest++;

	successTest+= TestGetDcMotorCurrentIm();
	totalTest++;

	successTest+= TestGetDcMotorVoltageVm();
	totalTest++;

	successTest+= TestGetQuadratureCurrentIqFeedback();
	totalTest++;

	successTest+= TestGetMagnetizingCurrentIdFeedback();
	totalTest++;

	successTest+= TestGetBoardTemperature();
	totalTest++;

	successTest+= TestGetSpeedFeedback();
	totalTest++;

	successTest+= TestGetPositionCountsFeedback();
	totalTest++;

	successTest+= TestGetErrorRegister();
	totalTest++;

	successTest+= TestGetDeviceFirmwareVersion();
	totalTest++;

	successTest+= TestGetDeviceHardwareVersion();
	totalTest++;
	
	successTest+= TestGet3PhaseMotorAngle();
	totalTest++;
	
	successTest+= TestGetEncoderIndexCounts();
	totalTest++;

	//////Kvaser Functions
	successTest+= TestGuardTime(1000);
	totalTest++;

	successTest+= TestLifeTimeFactor(1000);
	totalTest++;

	successTest+= TestProducerHeartbeatTime(1000);
	totalTest++;

	std::cout<<currentDateTime()<< "SUCCESS TESTS: ["<<successTest << "/" <<totalTest<< "]" << std::endl;
	file << currentDateTime()<< "SUCCESS TESTS: ["<<successTest << "/" <<totalTest<< "]" << std::endl;
	file.close();
	getchar();
}

const std::string currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X - ", &tstruct);

    return buf;
}

int TestCommunication()
{
	int error;
	
	std::cout<<currentDateTime()<< "Testing Communication: ";
	file << currentDateTime()<< "Testing Communication: ";
	
	try {
		solo = new SOLOMotorControllersKvaser(0, SOLOMotorControllers::CanbusBaudrate::rate1000, 50,false);
		solo->Connect();
		Sleep(500);
		solo->Disconnect();
		Sleep(500);
		solo->Connect();
		Sleep(500);
		solo->CommunicationIsWorking(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestCurrentLimit(float setValue)
{
	float getValue;
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestCurrentLimit (Test value: "<< setValue << "): ";
	file << currentDateTime()<< "Testing TestCurrentLimit (Test value: "<< setValue << "): ";
	
	try {
		solo->SetCurrentLimit(setValue, error);
		getValue = solo->GetCurrentLimit(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(getValue==setValue && error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestCommandMode(SOLOMotorControllers::CommandMode setValue)
{
	long getValue;
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestCommandMode (Test value: "<< setValue << "): ";
	file << currentDateTime()<< "Testing TestCommandMode (Test value: "<< setValue << "): ";
	
	try {
		solo->SetCommandMode(setValue, error);
		getValue = solo->GetCommandMode(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(getValue==setValue && error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestTorqueReferenceIq(float setValue)
{
	float getValue;
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestTorqueReferenceIq (Test value: "<< setValue << "): ";
	file << currentDateTime()<< "Testing TestTorqueReferenceIq (Test value: "<< setValue << "): ";
	
	try {
		solo->SetTorqueReferenceIq(setValue, error);
		getValue = solo->GetTorqueReferenceIq(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(abs(getValue - setValue) < epsilon && error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestSpeedReference(long setValue)
{
	long getValue;
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestSpeedReference (Test value: "<< setValue << "): ";
	file << currentDateTime()<< "Testing TestSpeedReference (Test value: "<< setValue << "): ";
	
	try {
		solo->SetSpeedReference(setValue, error);
		getValue = solo->GetSpeedReference(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(getValue==setValue && error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestPowerReference(float setValue)
{
	float getValue;
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestPowerReference (Test value: "<< setValue << "): ";
	file << currentDateTime()<< "Testing TestPowerReference (Test value: "<< setValue << "): ";
	
	try {
		solo->SetPowerReference(setValue, error);
		getValue = solo->GetPowerReference(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(abs(getValue - setValue) < epsilon && error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestMotorParametersIdentification(SOLOMotorControllers::Action setValue)
{
	float getValue;
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestMotorParametersIdentification (Test value: "<< setValue << "): ";
	file << currentDateTime()<< "Testing TestMotorParametersIdentification (Test value: "<< setValue << "): ";
	
	try {
		solo->MotorParametersIdentification(setValue, error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestOutputPwmFrequencyKhz(long setValue)
{
	long getValue;
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestOutputPwmFrequencyKhz (Test value: "<< setValue << "): ";
	file << currentDateTime()<< "Testing TestOutputPwmFrequencyKhz (Test value: "<< setValue << "): ";
	
	try {
		solo->SetOutputPwmFrequencyKhz(setValue, error);
		getValue = solo->GetOutputPwmFrequencyKhz(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(getValue==setValue && error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestSpeedControllerKp(float setValue)
{
	float getValue;
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestSpeedControllerKp (Test value: "<< setValue << "): ";
	file << currentDateTime()<< "Testing TestSpeedControllerKp (Test value: "<< setValue << "): ";
	
	try {
		solo->SetSpeedControllerKp(setValue, error);
		getValue = solo->GetSpeedControllerKp(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(abs(getValue - setValue) < epsilon && error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestSpeedControllerKi(float setValue)
{
	float getValue;
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestSpeedControllerKi (Test value: "<< setValue << "): ";
	file << currentDateTime()<< "Testing TestSpeedControllerKi (Test value: "<< setValue << "): ";
	
	try {
		solo->SetSpeedControllerKi(setValue, error);
		getValue = solo->GetSpeedControllerKi(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(abs(getValue - setValue) < epsilon && error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestMotorDirection(SOLOMotorControllers::Direction setValue)
{
	long getValue;
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestMotorDirection (Test value: "<< setValue << "): ";
	file << currentDateTime()<< "Testing TestMotorDirection (Test value: "<< setValue << "): ";
	
	try {
		solo->SetMotorDirection(setValue, error);
		getValue = solo->GetMotorDirection(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(getValue==setValue && error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestMotorResistance(float setValue)
{
	float getValue;
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestMotorResistance (Test value: "<< setValue << "): ";
	file << currentDateTime()<< "Testing TestMotorResistance (Test value: "<< setValue << "): ";
	
	try {
		solo->SetMotorResistance(setValue, error);
		getValue = solo->GetMotorResistance(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(abs(getValue - setValue) < epsilon && error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestMotorInductance(float setValue)
{
	float getValue;
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestMotorInductance (Test value: "<< setValue << "): ";
	file << currentDateTime()<< "Testing TestMotorInductance (Test value: "<< setValue << "): ";
	
	try {
		solo->SetMotorInductance(setValue, error);
		getValue = solo->GetMotorInductance(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(abs(getValue - setValue) < epsilon && error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestMotorPolesCounts(long setValue)
{
	long getValue;
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestMotorPolesCounts (Test value: "<< setValue << "): ";
	file << currentDateTime()<< "Testing TestMotorPolesCounts (Test value: "<< setValue << "): ";
	
	try {
		solo->SetMotorPolesCounts(setValue, error);
		getValue = solo->GetMotorPolesCounts(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(getValue==setValue && error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestIncrementalEncoderLines(long setValue)
{
	long getValue;
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestIncrementalEncoderLines (Test value: "<< setValue << "): ";
	file << currentDateTime()<< "Testing TestIncrementalEncoderLines (Test value: "<< setValue << "): ";
	
	try {
		solo->SetIncrementalEncoderLines(setValue, error);
		getValue = solo->GetIncrementalEncoderLines(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(getValue==setValue && error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestSpeedLimit(long setValue)
{
	long getValue;
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestSpeedLimit (Test value: "<< setValue << "): ";
	file << currentDateTime()<< "Testing TestSpeedLimit (Test value: "<< setValue << "): ";
	
	try {
		solo->SetSpeedLimit(setValue, error);
		getValue = solo->GetSpeedLimit(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(getValue==setValue && error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestFeedbackControlMode(SOLOMotorControllers::FeedbackControlMode setValue)
{
	long getValue;
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestFeedbackControlMode (Test value: "<< setValue << "): ";
	file << currentDateTime()<< "Testing TestFeedbackControlMode (Test value: "<< setValue << "): ";
	
	try {
		solo->SetFeedbackControlMode(setValue, error);
		getValue = solo->GetFeedbackControlMode(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(getValue==setValue && error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestMotorType(SOLOMotorControllers::MotorType setValue)
{
	long getValue;
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestMotorType (Test value: "<< setValue << "): ";
	file << currentDateTime()<< "Testing TestMotorType (Test value: "<< setValue << "): ";
	
	try {
		solo->SetMotorType(setValue, error);
		getValue = solo->GetMotorType(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(getValue==setValue && error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestControlMode(SOLOMotorControllers::ControlMode setValue)
{
	long getValue;
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestControlMode (Test value: "<< setValue << "): ";
	file << currentDateTime()<< "Testing TestControlMode (Test value: "<< setValue << "): ";
	
	try {
		solo->SetControlMode(setValue, error);
		getValue = solo->GetControlMode(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(getValue==setValue && error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestCurrentControllerKp(float setValue)
{
	float getValue;
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestCurrentControllerKp (Test value: "<< setValue << "): ";
	file << currentDateTime()<< "Testing TestCurrentControllerKp (Test value: "<< setValue << "): ";
	
	try {
		solo->SetCurrentControllerKp(setValue, error);
		getValue = solo->GetCurrentControllerKp(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(abs(getValue - setValue) < epsilon && error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestCurrentControllerKi(float setValue)
{
	float getValue;
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestCurrentControllerKi (Test value: "<< setValue << "): ";
	file << currentDateTime()<< "Testing TestCurrentControllerKi (Test value: "<< setValue << "): ";
	
	try {
		solo->SetCurrentControllerKi(setValue, error);
		getValue = solo->GetCurrentControllerKi(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(abs(getValue - setValue) < epsilon && error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestMonitoringMode(bool setValue)
{
	bool getValue;
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestMonitoringMode (Test value: "<< setValue << "): ";
	file << currentDateTime()<< "Testing TestMonitoringMode (Test value: "<< setValue << "): ";
	
	try {
		solo->SetMonitoringMode(setValue, error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestMagnetizingCurrentIdReference(float setValue)
{
	float getValue;
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestMagnetizingCurrentIdReference (Test value: "<< setValue << "): ";
	file << currentDateTime()<< "Testing TestMagnetizingCurrentIdReference (Test value: "<< setValue << "): ";
	
	try {
		solo->SetMagnetizingCurrentIdReference(setValue, error);
		getValue = solo->GetMagnetizingCurrentIdReference(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(abs(getValue - setValue) < epsilon && error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestPositionReference(long setValue)
{
	long getValue;
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestPositionReference (Test value: "<< setValue << "): ";
	file << currentDateTime()<< "Testing TestPositionReference (Test value: "<< setValue << "): ";
	
	try {
		solo->SetPositionReference(setValue, error);
		getValue = solo->GetPositionReference(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(getValue==setValue && error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestPositionControllerKp(float setValue)
{
	float getValue;
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestPositionControllerKp (Test value: "<< setValue << "): ";
	file << currentDateTime()<< "Testing TestPositionControllerKp (Test value: "<< setValue << "): ";
	
	try {
		solo->SetPositionControllerKp(setValue, error);
		getValue = solo->GetPositionControllerKp(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(abs(getValue - setValue) < epsilon && error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestPositionControllerKi(float setValue)
{
	float getValue;
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestPositionControllerKi (Test value: "<< setValue << "): ";
	file << currentDateTime()<< "Testing TestPositionControllerKi (Test value: "<< setValue << "): ";
	
	try {
		solo->SetPositionControllerKi(setValue, error);
		getValue = solo->GetPositionControllerKi(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(abs(getValue - setValue) < epsilon && error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestResetPositionToZero()
{
	float getValue;
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestResetPositionToZero (Test value: ""): ";
	file << currentDateTime()<< "Testing TestResetPositionToZero (Test value: ""): ";
	
	try {
		solo->ResetPositionToZero(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestOverwriteErrorRegister()
{
	float getValue;
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestOverwriteErrorRegister (Test value: ""): ";
	file << currentDateTime()<< "Testing TestOverwriteErrorRegister (Test value: ""): ";
	
	try {
		solo->OverwriteErrorRegister(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestObserverGainBldcPmsm(float setValue)
{
	float getValue;
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestObserverGainBldcPmsm (Test value: "<< setValue << "): ";
	file << currentDateTime()<< "Testing TestObserverGainBldcPmsm (Test value: "<< setValue << "): ";
	
	try {
		solo->SetObserverGainBldcPmsm(setValue, error);
		getValue = solo->GetObserverGainBldcPmsm(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(abs(getValue - setValue) < epsilon && error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestObserverGainBldcPmsmUltrafast(float setValue)
{
	float getValue;
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestObserverGainBldcPmsmUltrafast (Test value: "<< setValue << "): ";
	file << currentDateTime()<< "Testing TestObserverGainBldcPmsmUltrafast (Test value: "<< setValue << "): ";
	
	try {
		solo->SetObserverGainBldcPmsmUltrafast(setValue, error);
		getValue = solo->GetObserverGainBldcPmsmUltrafast(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(abs(getValue - setValue) < epsilon && error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestObserverGainDc(float setValue)
{
	float getValue;
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestObserverGainDc (Test value: "<< setValue << "): ";
	file << currentDateTime()<< "Testing TestObserverGainDc (Test value: "<< setValue << "): ";
	
	try {
		solo->SetObserverGainDc(setValue, error);
		getValue = solo->GetObserverGainDc(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(abs(getValue - setValue) < epsilon && error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestFilterGainBldcPmsm(float setValue)
{
	float getValue;
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestFilterGainBldcPmsm (Test value: "<< setValue << "): ";
	file << currentDateTime()<< "Testing TestFilterGainBldcPmsm (Test value: "<< setValue << "): ";
	
	try {
		solo->SetFilterGainBldcPmsm(setValue, error);
		getValue = solo->GetFilterGainBldcPmsm(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(abs(getValue - setValue) < epsilon && error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestFilterGainBldcPmsmUltrafast(float setValue)
{
	float getValue;
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestFilterGainBldcPmsmUltrafast (Test value: "<< setValue << "): ";
	file << currentDateTime()<< "Testing TestFilterGainBldcPmsmUltrafast (Test value: "<< setValue << "): ";
	
	try {
		solo->SetFilterGainBldcPmsmUltrafast(setValue, error);
		getValue = solo->GetFilterGainBldcPmsmUltrafast(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(abs(getValue - setValue) < epsilon && error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestUartBaudrate(SOLOMotorControllers::UartBaudrate setValue)
{
	SOLOMotorControllers::UartBaudrate getValue;
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestUartBaudrate (Test value: "<< setValue << "): ";
	file << currentDateTime()<< "Testing TestUartBaudrate (Test value: "<< setValue << "): ";
	
	try {
		solo->SetUartBaudrate(setValue, error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestSensorCalibration(SOLOMotorControllers::PositionSensorCalibrationAction setValue)
{
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestUartBaudrate (Test value: "<< setValue << "): ";
	file << currentDateTime()<< "Testing TestUartBaudrate (Test value: "<< setValue << "): ";
	
	try {
		solo->SensorCalibration(setValue, error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestEncoderHallCcwOffset(float setValue)
{
	float getValue;
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestEncoderHallCcwOffset (Test value: "<< setValue << "): ";
	file << currentDateTime()<< "Testing TestEncoderHallCcwOffset (Test value: "<< setValue << "): ";
	
	try {
		solo->SetEncoderHallCcwOffset(setValue, error);
		getValue = solo->GetEncoderHallCcwOffset(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(abs(getValue - setValue) < epsilon && error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestEncoderHallCwOffset(float setValue)
{
	float getValue;
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestEncoderHallCwOffset (Test value: "<< setValue << "): ";
	file << currentDateTime()<< "Testing TestEncoderHallCwOffset (Test value: "<< setValue << "): ";
	
	try {
		solo->SetEncoderHallCwOffset(setValue, error);
		getValue = solo->GetEncoderHallCwOffset(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(abs(getValue - setValue) < epsilon && error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestSpeedAccelerationValue(float setValue)
{
	float getValue;
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestSpeedAccelerationValue (Test value: "<< setValue << "): ";
	file << currentDateTime()<< "Testing TestSpeedAccelerationValue (Test value: "<< setValue << "): ";
	
	try {
		solo->SetSpeedAccelerationValue(setValue, error);
		getValue = solo->GetSpeedAccelerationValue(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(abs(getValue - setValue) < epsilon && error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestSpeedDecelerationValue(float setValue)
{
	float getValue;
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestSpeedDecelerationValue (Test value: "<< setValue << "): ";
	file << currentDateTime()<< "Testing TestSpeedDecelerationValue (Test value: "<< setValue << "): ";
	
	try {
		solo->SetSpeedDecelerationValue(setValue, error);
		getValue = solo->GetSpeedDecelerationValue(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(abs(getValue - setValue) < epsilon && error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestCanbusBaudrate(SOLOMotorControllers::CanbusBaudrate setValue)
{
	SOLOMotorControllers::CanbusBaudrate getValue;
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestCanbusBaudrate (Test value: "<< setValue << "): ";
	file << currentDateTime()<< "Testing TestCanbusBaudrate (Test value: "<< setValue << "): ";
	
	try {
		solo->SetCanbusBaudrate(setValue, error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestGetPhaseAVoltage()
{
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestGetPhaseAVoltage (Test value: ""): ";
	file << currentDateTime()<< "Testing TestGetPhaseAVoltage (Test value: ""): ";
	
	try {
		solo->GetPhaseAVoltage(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestGetPhaseBVoltage()
{
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestGetPhaseBVoltage (Test value: ""): ";
	file << currentDateTime()<< "Testing TestGetPhaseBVoltage (Test value: ""): ";
	
	try {
		solo->GetPhaseAVoltage(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestGetPhaseACurrent()
{
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestGetPhaseACurrent (Test value: ""): ";
	file << currentDateTime()<< "Testing TestGetPhaseACurrent (Test value: ""): ";
	
	try {
		solo->GetPhaseAVoltage(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestGetPhaseBCurrent()
{
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestGetPhaseBCurrent (Test value: ""): ";
	file << currentDateTime()<< "Testing TestGetPhaseBCurrent (Test value: ""): ";
	
	try {
		solo->GetPhaseAVoltage(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestGetBusVoltage()
{
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestGetBusVoltage (Test value: ""): ";
	file << currentDateTime()<< "Testing TestGetBusVoltage (Test value: ""): ";
	
	try {
		solo->GetPhaseAVoltage(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestGetDcMotorCurrentIm()
{
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestGetDcMotorCurrentIm (Test value: ""): ";
	file << currentDateTime()<< "Testing TestGetDcMotorCurrentIm (Test value: ""): ";
	
	try {
		solo->GetPhaseAVoltage(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestGetDcMotorVoltageVm()
{
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestGetDcMotorVoltageVm (Test value: ""): ";
	file << currentDateTime()<< "Testing TestGetDcMotorVoltageVm (Test value: ""): ";
	
	try {
		solo->GetPhaseAVoltage(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestGetQuadratureCurrentIqFeedback()
{
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestGetQuadratureCurrentIqFeedback (Test value: ""): ";
	file << currentDateTime()<< "Testing TestGetQuadratureCurrentIqFeedback (Test value: ""): ";
	
	try {
		solo->GetPhaseAVoltage(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestGetMagnetizingCurrentIdFeedback()
{
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestGetMagnetizingCurrentIdFeedback (Test value: ""): ";
	file << currentDateTime()<< "Testing TestGetMagnetizingCurrentIdFeedback (Test value: ""): ";
	
	try {
		solo->GetPhaseAVoltage(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestGetBoardTemperature()
{
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestGetBoardTemperature (Test value: ""): ";
	file << currentDateTime()<< "Testing TestGetBoardTemperature (Test value: ""): ";
	
	try {
		solo->GetPhaseAVoltage(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestGetSpeedFeedback()
{
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestGetSpeedFeedback (Test value: ""): ";
	file << currentDateTime()<< "Testing TestGetSpeedFeedback (Test value: ""): ";
	
	try {
		solo->GetPhaseAVoltage(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestGetPositionCountsFeedback()
{
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestGetPositionCountsFeedback (Test value: ""): ";
	file << currentDateTime()<< "Testing TestGetPositionCountsFeedback (Test value: ""): ";
	
	try {
		solo->GetPhaseAVoltage(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestGetErrorRegister()
{
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestGetErrorRegister (Test value: ""): ";
	file << currentDateTime()<< "Testing TestGetErrorRegister (Test value: ""): ";
	
	try {
		solo->GetPhaseAVoltage(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestGetDeviceFirmwareVersion()
{
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestGetDeviceFirmwareVersion (Test value: ""): ";
	file << currentDateTime()<< "Testing TestGetDeviceFirmwareVersion (Test value: ""): ";
	
	try {
		solo->GetPhaseAVoltage(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestGetDeviceHardwareVersion()
{
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestGetDeviceHardwareVersion (Test value: ""): ";
	file << currentDateTime()<< "Testing TestGetDeviceHardwareVersion (Test value: ""): ";
	
	try {
		solo->GetPhaseAVoltage(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestGet3PhaseMotorAngle()
{
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestGet3PhaseMotorAngle (Test value: ""): ";
	file << currentDateTime()<< "Testing TestGet3PhaseMotorAngle (Test value: ""): ";
	
	try {
		solo->GetPhaseAVoltage(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestGetEncoderIndexCounts()
{
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestGetEncoderIndexCounts (Test value: ""): ";
	file << currentDateTime()<< "Testing TestGetEncoderIndexCounts (Test value: ""): ";
	
	try {
		solo->GetPhaseAVoltage(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

//////Kvaser Functions
int TestGuardTime(long setValue)
{
	float getValue;
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestGuardTime (Test value: "<< setValue << "): ";
	file << currentDateTime()<< "Testing TestGuardTime (Test value: "<< setValue << "): ";
	
	try {
		solo->SetGuardTime(setValue, error);
		getValue = solo->GetGuardTime(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(abs(getValue - setValue) < epsilon && error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestLifeTimeFactor(long setValue)
{
	float getValue;
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestLifeTimeFactor (Test value: "<< setValue << "): ";
	file << currentDateTime()<< "Testing TestLifeTimeFactor (Test value: "<< setValue << "): ";
	
	try {
		solo->SetGuardTime(setValue, error);
		getValue = solo->GetGuardTime(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(abs(getValue - setValue) < epsilon && error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}

int TestProducerHeartbeatTime(long setValue)
{
	float getValue;
	int error;
	
	std::cout<<currentDateTime()<< "Testing TestProducerHeartbeatTime (Test value: "<< setValue << "): ";
	file << currentDateTime()<< "Testing TestProducerHeartbeatTime (Test value: "<< setValue << "): ";
	
	try {
		solo->SetGuardTime(setValue, error);
		getValue = solo->GetGuardTime(error);
	}catch (...){
		std::cout << "[EXCEPTION]"<< std::endl;
		file<< "[EXCEPTION]"<< std::endl;
		return 0;
	}

	if(abs(getValue - setValue) < epsilon && error == 0){
		std::cout << "[OK]"<< std::endl;
		file<< "[OK]"<< std::endl;
		return 1;
	}else{
		std::cout << "[ERROR]"<< std::endl;
		file<< "[ERROR]"<< std::endl;
		return 0;
	}
}
