/**
 *******************************************************************************
 * @file    SOLOMotorControllersSerial.cpp
 * @authors SOLO Motor Controllers
 * @brief   This file contains all the base functions prototypes for the Solo Drivers
 *          Availability: https://github.com/Solo-FL/SOLO-motor-controllers-CPP-library
 * 
 * @date    Date: 2023
 * @version 1.2.0
 * *******************************************************************************    
 * @attention
 * Copyright: (c) 2021-2023, SOLO motor controllers project
 * GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)
 ******************************************************************************* 
 */

#include "SOLOMotorControllersSerial.h"

//DEBUG
// #include "stdio.h"
// #include <iostream>
// using std::cout;
// using std::endl;
// using std::hex; 

#define ReadData                            0x00 // 0x00000000
#define INITIATOR                           0xFF //0xFFFF
#define BroadcastAddress                    0xFF
#define ENDING                              0xFE
#define ERR		                          	  0xEE //0xEEEEEEEE
#define CRC                                 0x00
#define WriteDeviceAddres                   0x01
#define WriteCommandMode                    0x02
#define WriteCurrentLimit                   0x03
#define WriteTorqueReferenceIq              0x04
#define WriteSpeedReference                 0x05
#define WritePowerReference                 0x06
#define WriteMotorParametersIdentification  0x07
#define WriteEmergencyStop                  0x08
#define WriteOutputPwmFrequencyKhz          0x09
#define WriteSpeedControllerKp              0x0A
#define WriteSpeedControllerKi              0x0B
#define WriteMotorDirection                 0x0C
#define WriteMotorResistance                0x0D
#define WriteMotorInductance                0x0E
#define WriteMotorPolesCounts               0x0F
#define WriteIncrementalEncoderLines        0x10
#define WriteSpeedLimit                     0x11
#define WriteResetDeviceAddress             0x12
#define WriteFeedbackControlMode            0x13
#define WriteResetFactory                   0x14
#define WriteMotorType                      0x15
#define WriteControlMode                    0x16
#define WriteCurrentControllerKp            0x17
#define WriteCurrentControllerKi            0x18
#define WriteMonitoringMode                 0x19
#define WriteMagnetizingCurrentIdReference  0x1A
#define WritePositionReference              0x1B
#define WritePositionControllerKp           0x1C
#define WritePositionControllerKi           0x1D
#define WriteResetPositionToZero            0x1F //Home
#define WriteOverwriteErrorRegister         0x20
#define WriteObserverGainBldcPmsm           0x21 //Set Sensorless Observer Gain for Normal Brushless Motor
#define WriteObserverGainBldcPmsmUltrafast  0x22 //Set Sensorless Observer Gain for Ultra-Fast Brushless Motor
#define WriteObserverGainDc                 0x23 //Set Sensorless Observer Gain for DC Motor
#define WriteFilterGainBldcPmsm             0x24 //Set Sensorless Observer Filter Gain for Normal Brushless Motor
#define WriteFilterGainBldcPmsmUltrafast    0x25 //Set Sensorless Observer Filter Gain for ultra-fast Brushless Motor
#define WriteUartBaudrate                   0x26 //Set UART line baud-rate - 937500 / 115200 [ bits/s]
#define WriteSensorCalibration              0x27
#define WriteEncoderHallCcwOffset           0x28
#define WriteEncoderHallCwOffset            0x29
#define WriteSpeedAccelerationValue         0x2A
#define WriteSpeedDecelerationValue         0x2B
#define WriteCanbusBaudrate                 0x2C

#define ReadDeviceAddress                   0x81
#define ReadPhaseAVoltage                   0x82
#define ReadPhaseBVoltage                   0x83
#define ReadPhaseACurrent                   0x84
#define ReadPhaseBCurrent                   0x85
#define ReadBusVoltage                      0x86
#define ReadDcMotorCurrentIm                0x87
#define ReadDcMotorVoltageVm                0x88
#define ReadSpeedControllerKp               0x89
#define ReadSpeedControllerKi               0x8A
#define ReadOutputPwmFrequencyHz            0x8B
#define ReadCurrentLimit                    0x8C
#define ReadQuadratureCurrentIqFeedback     0x8D
#define ReadMagnetizingCurrentIdFeedback    0x8E //Magnetizing
#define ReadMotorPolesCounts                0x8F
#define ReadIncrementalEncoderLines         0x90
#define ReadCurrentControllerKp             0x91
#define ReadCurrentControllerKi             0x92
#define ReadBoardTemperature                0x93
#define ReadMotorResistance                 0x94
#define ReadMotorInductance                 0x95
#define ReadSpeedFeedback                   0x96
#define ReadMotorType                       0x97
#define ReadFeedbackControlMode             0x99
#define ReadCommandMode                     0x9A
#define ReadControlMode                     0x9B
#define ReadSpeedLimit                      0x9C
#define ReadPositionControllerKp            0x9D
#define ReadPositionControllerKi            0x9E
#define ReadPositionCountsFeedback          0xA0
#define ReadErrorRegister                   0xA1
#define ReadDeviceFirmwareVersion           0xA2
#define ReadDeviceHardwareVersion           0xA3
#define ReadTorqueReferenceIq               0xA4 // Read Torque Iq Reference
#define ReadSpeedReference                  0xA5 // Read Speed Reference
#define ReadMagnetizingCurrentIdReference   0xA6 // Read Magnetizing Current Id Reference
#define ReadPositionReference               0xA7
#define ReadPowerReference                  0xA8
#define ReadMotorDirection                  0xA9
#define ReadObserverGainBldcPmsm            0xAA // Read the Non-linear observer Gain for Normal Brushless motor in Sensorless mode
#define ReadObserverGainBldcPmsmUltrafast   0xAB // Read the Non-linear observer Gain for Ultra-fast Brushless motor in Sensorless mode
#define ReadObserverGainDc                  0xAC // Read the Non-linear observer Gain for DC motor in Sensorless mode
#define ReadFilterGainBldcPmsm              0xAD // Read the Non-linear observer Filter Gain for Normal Brushless motor in Sensorless mode
#define ReadFilterGainBldcPmsmUltrafast     0xAE // Read the Non-linear Filter Gain for Ultra-fast Brushless motor in Sensorless mode
#define Read3PhaseMotorAngle                0xB0 // Read Estimated or Measured Rotor Angle
#define ReadEncoderHallCcwOffset            0xB1
#define ReadEncoderHallCwOffset             0xB2
#define ReadUartBaudrate                    0xB3 // 0 / 1 ( 937500 / 115200 [bits/s] )
#define ReadSpeedAccelerationValue          0xB4
#define ReadSpeedDecelerationValue          0xB5
#define ReadEncoderIndexCounts              0xB8

SOLOMotorControllersSerial::SOLOMotorControllersSerial(char* COMPortName, UINT8 deviceAddress,
 		SOLOMotorControllers::UartBaudrate baudrate,
 		long millisecondsTimeout, int packetFailureTrialAttempts, bool autoConnect)
	:addr(deviceAddress)
	, portName(COMPortName)
	, uartBaudrate(baudrate)
	, timeout(millisecondsTimeout)
	, trialCount(packetFailureTrialAttempts)
{
	soloUtils = new SOLOMotorControllersUtils();
	if(autoConnect)
	{
		Connect();
	}	
}

SOLOMotorControllersSerial::~SOLOMotorControllersSerial()
{
	Disconnect();
}

bool SOLOMotorControllersSerial::Connect(char* COMPortName, UINT8 deviceAddress,
 		SOLOMotorControllers::UartBaudrate baudrate,
 		long millisecondsTimeout, int packetFailureTrialAttempts)
{
	addr = deviceAddress;
	portName = COMPortName;
	uartBaudrate = baudrate;
	timeout = millisecondsTimeout;
	trialCount = packetFailureTrialAttempts;
	return SOLOMotorControllersSerial::Connect();
}

bool SOLOMotorControllersSerial::Connect()
{
  if(isConnected){
    //std::cout << "Connect - Already connected " << std::endl;
    return true;
  }

	sprintf_s(ComPortName, "\\\\.\\%s", portName); // ComPortName = "\\\\.\\"+ portName
  //std::cout << "Connect - connection start ComPortName:" << ComPortName <<std::endl;
  
  //// Opening the serial port
  hSerial = CreateFile((LPCSTR)ComPortName,         // Name of the Port to be Opened
		GENERIC_READ | GENERIC_WRITE,                 // Read/Write Access
		0,                                            // No Sharing, ports cant be shared
		NULL,                                         // No Security
		OPEN_EXISTING,                                // Open existing port only
		0,                                            // Non Overlapped I/O
		NULL);                                        // Null for Comm Devices
  Sleep(100);

  DWORD lastError;
	if (hSerial == INVALID_HANDLE_VALUE)
	{
    lastError = GetLastError(); 
    if(lastError==ERROR_FILE_NOT_FOUND){
      //std::cout << "Connect -  hSerial: serial port does not exist - " << std::system_category().message(lastError)<< std::endl;
    }else{
      //std::cout << "Connect -  hSerial: general connection error - " << std::system_category().message(lastError) <<std::endl;
    }
		return false;
	} 

  Status = FlushFileBuffers(hSerial);
  if(!Status){
    //std::cout << "Connect -  FlushFileBuffers: error" << std::endl;
    return false;
  }

  ////Setting Parameters
	DCB dcbSerialParams = { 0 };                        // Initializing DCB structure
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

  if (!GetCommState(hSerial, &dcbSerialParams)) {
    //std::cout << "Connect -  dcbSerialParams: error getting state" << std::endl;
    return false;
  }

	switch (uartBaudrate)
	{
	case 0:
		dcbSerialParams.BaudRate = 937500;
		break;
	case 1:
		dcbSerialParams.BaudRate = CBR_115200;
		break;
	default:
		dcbSerialParams.BaudRate = CBR_115200;
		break;
	}

	dcbSerialParams.ByteSize = 8;             // Setting ByteSize = 8
	dcbSerialParams.StopBits = ONESTOPBIT;    // Setting StopBits = 1
	dcbSerialParams.Parity = NOPARITY;      // Setting Parity = None 
  dcbSerialParams.fRtsControl = RTS_CONTROL_DISABLE;
	dcbSerialParams.fDtrControl = DTR_CONTROL_DISABLE;

	Status = SetCommState(hSerial, &dcbSerialParams);  //Configuring the port according to settings in DCB 
  
	if (Status == FALSE)
	{
    //std::cout << "Connect -  SetCommState: error setting serial port state" << std::endl;
		return false;
	}

  ////Setting timeouts
	COMMTIMEOUTS timeouts = { 0 };

	timeouts.ReadIntervalTimeout = timeout;
	timeouts.ReadTotalTimeoutConstant = timeout;
	timeouts.ReadTotalTimeoutMultiplier = 10;
	timeouts.WriteTotalTimeoutConstant = timeout;
	timeouts.WriteTotalTimeoutMultiplier = 10;

	if (SetCommTimeouts(hSerial, &timeouts) == FALSE){
    //std::cout << "Connect -  SetCommTimeouts: error setting serial port timeouts" << std::endl;
    return false;
  }

  //std::cout << "Connect - connection success" << std::endl;
	isConnected = true;
  Sleep(100);
	return true;
}

void SOLOMotorControllersSerial::Disconnect()
{
	if(isConnected == true)
	{
		isConnected = false;
    Sleep(500);
		Status = CloseHandle(hSerial);
    Sleep(500);
    //std::cout << std::boolalpha << "Disconnect: "  << Status << std::endl;
	}
}

bool SOLOMotorControllersSerial::Test()
{

	unsigned char cmd[] = { 0xFF,0xFF,65,66,67,68,69,70,0,0xFE };
	int i = 0;
	if (addr == 0x01)
		return true;
	else return false;
}

bool SOLOMotorControllersSerial::ExeCMD(unsigned char* cmd, int& error)
{
  //std::cout << "ExeCMD - " << " isConnected: "<< isConnected << std::endl; 
	unsigned char _cmd[10] = { INITIATOR, INITIATOR, cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], CRC, ENDING };
	unsigned char _readPacket[500];
  int maxChars = 500;
	int idx = 0;

	bool isPacketFailureTrialAttemptsOverflow = true;
	//FailureTrialAttempts block
	for (int attempts = 0; attempts < trialCount; attempts++){ 
    if (!isConnected) {
      SOLOMotorControllersSerial::Connect();
    }

		_readPacket[0] = 0;
		_readPacket[1] = 0;
		_readPacket[2] = 0;
		_readPacket[3] = 0;
		_readPacket[4] = 0;
		_readPacket[5] = 0;
		_readPacket[6] = 0;
		_readPacket[7] = 0;
		_readPacket[8] = 0;
		_readPacket[9] = 0;

		dNoOFBytestoWrite = sizeof(_cmd);
    // //https://learn.microsoft.com/en-gb/windows/win32/api/winbase/nf-winbase-purgecomm
    // Status = PurgeComm(hSerial,PURGE_RXCLEAR); //flush the buffer (need empty for write)
    // std::cout << "PurgeComm Status: " << Status << std::endl; 

    // //https://learn.microsoft.com/en-us/windows/win32/fileio/cancelio
    // Status = CancelIo(hSerial);
    // std::cout << "CancelIo Status: " << Status << std::endl; 

    //https://learn.microsoft.com/en-us/windows/win32/api/fileapi/nf-fileapi-writefileex
		Status = WriteFile(hSerial,               // Handle to the Serialport
			_cmd,            // Data to be written to the port 
			dNoOFBytestoWrite,   // No of bytes to write into the port
			&dNoOfBytesWritten,  // No of bytes written to the port
			NULL);
    //std::cout << "ExeCMD - WriteFile Status: " << Status << std::endl; 

		//std::cout << "ExeCMD - WriteFile Status: " << Status <<" BytestoWrite: "<< dNoOFBytestoWrite <<" BytesWritten: "<< dNoOfBytesWritten << " isConnected: "<< isConnected <<" hSerial: "<<hSerial <<std::endl; 	
		if (Status == FALSE) {
			if (isConnected) {
			  //std::cout << "ExeCMD - Disconnect" << std::endl;
				SOLOMotorControllersSerial::Disconnect();
			}
			continue;
		}

		/*------------------------------------ Setting Receive Mask ----------------------------------------------*/

    Status = TRUE;
		//Status = SetCommMask(hSerial, EV_RXCHAR); //Configure Windows to Monitor the serial device for Character Reception
		//std::cout << "ExeCMD - SetCommMask Status: " << Status << std::endl; 

    idx = 0;
		// if (Status == FALSE) {
		// 	continue;
		// }
		// else //If  WaitCommEvent()==True Read the RXed data using ReadFile();
		// {
			do
			{
				Status = ReadFile(hSerial, &TempChar, 1, &NoBytesRecieved, NULL);
				_readPacket[idx] = TempChar;
				//Read messages
				//std::cout << (long) _readPacket[idx] << " ("<<(long)idx <<"-"<< NoBytesRecieved <<")  ";
				idx++;
			} while (NoBytesRecieved > 0 && idx < maxChars && Status == TRUE );
		// }
    //std::cout <<"\nExeCMD -  idx: "<< idx<<  " Status: "<< Status<<"\n";

    for(int i = 0 ; i+9<idx; i++){
      //std::cout << "ExeCMD - "<< std::hex << static_cast<unsigned>(_readPacket[i+0])<< "- " <<std::hex << static_cast<unsigned>(_readPacket[i+1])<<"- " << std::hex << static_cast<unsigned>(_readPacket[i+2])<< "- " << std::hex << static_cast<unsigned>(_readPacket[i+3])<< "- " << std::hex << static_cast<unsigned>(_readPacket[i+4])<< "- " << std::hex << static_cast<unsigned>(_readPacket[i+5])<< "- " << std::hex << static_cast<unsigned>(_readPacket[i+6]) << "- " << std::hex << static_cast<unsigned>(_readPacket[i+7])<< "- " << std::hex << static_cast<unsigned>(_readPacket[i+8])<< "- " << std::hex << static_cast<unsigned>(_readPacket[i+9])<<" \n";
      if (
        _readPacket[i+0] == _cmd[0] && _readPacket[i+1] == _cmd[1]
        && (_readPacket[i+2] == _cmd[2] || _cmd[2] == 0xFF)
        && _readPacket[i+8] == _cmd[8] && _readPacket[i+9] == _cmd[9]
        ){
          if(_readPacket[i+3] == _cmd[3]){
            cmd[0] = _readPacket[i+2];
            cmd[1] = _readPacket[i+3];
            cmd[2] = _readPacket[i+4];
            cmd[3] = _readPacket[i+5];
            cmd[4] = _readPacket[i+6];
            cmd[5] = _readPacket[i+7];

            error = Error::noErrorDetected;
            return true;
          }else{
            i+=9;
            continue;
          }
      }
    }
  }

  cmd[0] = ERR;
  cmd[1] = ERR;
  cmd[2] = ERR;
  cmd[3] = ERR;
  cmd[4] = ERR;
  cmd[5] = ERR;
  error = SOLOMotorControllers::packetFailureTrialAttemptsOverflow;
  return false;
}


void SOLOMotorControllersSerial::SplitData(unsigned char* data, unsigned char* cmd)
{
	data[0] = cmd[2];
	data[1] = cmd[3];
	data[2] = cmd[4];
	data[3] = cmd[5];
}

/**
  * @brief  This command sets the desired device address for a SOLO unit
  *           .The method refers to the Uart Write command: 0x01
  * @param[in]  deviceAddress  address want to set for board
  * @param[out]  error   pointer to an integer that specify result of function       
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetDeviceAddress(unsigned char deviceAddress, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	unsigned char cmd[] = { addr,WriteDeviceAddres,0x00,0x00,0x00,deviceAddress };

	if (!soloUtils->SetDeviceAddressInputValidation(deviceAddress, error))
	{
		return false;
	}

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}

/**
  * @brief  This command sets the desired device address for a SOLO unit
  *           .The method refers to the Uart Write command: 0x01
  * @param[in]  deviceAddress  address want to set for board    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetDeviceAddress(unsigned char deviceAddress)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetDeviceAddress(deviceAddress, error);
}

/**
  * @brief  This command sets the mode of the operation of SOLO
  *         in terms of operating in Analogue mode or Digital
  *           .The method refers to the Uart Write command: 0x02
  * @param[in]  mode  enum that specify mode of the operation of SOLO 
  * @param[out]  error   pointer to an integer that specify result of function       
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetCommandMode(CommandMode mode, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	unsigned char cmd[] = { addr,WriteCommandMode,0x00,0x00,0x00,mode };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}

/**
  * @brief  This command sets the mode of the operation of SOLO
  *         in terms of operating in Analogue mode or Digital
  *           .The method refers to the Uart Write command: 0x02
  * @param[in]  mode  enum that specify mode of the operation of SOLO      
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetCommandMode(CommandMode mode)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetCommandMode(mode, error);
}

/**
  * @brief  This command defines the maximum allowed current into the motor in terms of Amps
  *           .The method refers to the Uart Write command: 0x03
  * @param[in]  currentLimit  a float value [Amps]
  * @param[out]  error   pointer to an integer that specify result of function       
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetCurrentLimit(float currentLimit, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetCurrentLimitInputValidation(currentLimit, error))
	{
		return false;
	}

	unsigned char data[4];
	soloUtils->ConvertToData(currentLimit, data);
	unsigned char cmd[] = { addr,WriteCurrentLimit,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}

/**
  * @brief  This command defines the maximum allowed current into the motor in terms of Amps
  *           .The method refers to the Uart Write command: 0x03
  * @param[in]  currentLimit  a float value [Amps]     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetCurrentLimit(float currentLimit)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetCurrentLimit(currentLimit, error);
}

/**
  * @brief  This command sets the amount of desired current that acts in torque generation
  *           .The method refers to the Uart Write command: 0x04
  * @param[in]  torqueReferenceIq  a float [Amps]
  * @param[out]  error   pointer to an integer that specify result of function       
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetTorqueReferenceIq(float torqueReferenceIq, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetTorqueReferenceIqInputValidation(torqueReferenceIq, error))
	{
		return false;
	}

	unsigned char data[4];
	soloUtils->ConvertToData(torqueReferenceIq, data);
	unsigned char cmd[] = { addr,WriteTorqueReferenceIq,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}

/**
  * @brief  This command sets the amount of desired current that acts in torque generation
  *           .The method refers to the Uart Write command: 0x04
  * @param[in]  torqueReferenceIq  a float [Amps]      
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetTorqueReferenceIq(float torqueReferenceIq)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetTorqueReferenceIq(torqueReferenceIq, error);
}

/**
  * @brief  This command defines the speed reference for SOLO once it’s in Digital Speed Mode
  *           .The method refers to the Uart Write command: 0x05
  * @param[in]  speedReference  a long value [RPM]
  * @param[out]  error   pointer to an integer that specify result of function       
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetSpeedReference(long speedReference, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetSpeedReferenceInputValidation(speedReference, error))
	{
		return false;
	}

	unsigned char data[4];
	soloUtils->ConvertToData(speedReference, data);
	unsigned char cmd[] = { addr,WriteSpeedReference,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}

/**
  * @brief  This command defines the speed reference for SOLO once it’s in Digital Speed Mode
  *           .The method refers to the Uart Write command: 0x05
  * @param[in]  speedReference  a long value [RPM]      
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetSpeedReference(long speedReference)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetSpeedReference(speedReference, error);
}

/**
  * @brief  This command defines the amount of power percentage during only
  *         Open-loop mode for 3-phase motors
  *           .The method refers to the Uart Write command: 0x06
  * @param[in]  powerReference  a float value between 0 to 100
  * @param[out]  error   pointer to an integer that specify result of function       
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetPowerReference(float powerReference, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetPowerReferenceInputValidation(powerReference, error))
	{
		return false;
	}

	unsigned char data[4];
	soloUtils->ConvertToData(powerReference, data);
	unsigned char cmd[] = { addr,WritePowerReference,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}

/**
  * @brief  This command defines the amount of power percentage during only
  *         Open-loop mode for 3-phase motors
  *           .The method refers to the Uart Write command: 0x06
  * @param[in]  powerReference  a float value between 0 to 100       
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetPowerReference(float powerReference)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetPowerReference(powerReference, error);
}

/**
  * @brief  By putting 1 in the DATA section of a packet sent with this command, SOLO will start
            identifying the electrical parameters of the Motor connected
              .The method refers to the Uart Write command: 0x07
  * @param[in]  powerReference  enum that specify Start or Stop of something in SOLO 
  * @param[out]  error   pointer to an integer that specify result of function       
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::MotorParametersIdentification(Action identification, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	unsigned char cmd[] = { addr,WriteMotorParametersIdentification,0x00,0x00,0x00,identification };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}

/**
  * @brief  By putting 1 in the DATA section of a packet sent with this command, SOLO will start
            identifying the electrical parameters of the Motor connected
              .The method refers to the Uart Write command: 0x07
  * @param[in]  powerReference  enum that specify Start or Stop of something in SOLO      
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::MotorParametersIdentification(Action identification)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::MotorParametersIdentification(identification, error);
}

/**
  * @brief  This command if the DATA is set at zero will stop the whole power and switching system
            connected to the motor and it will cut the current floating into the Motor from SOLO
              .The method refers to the Uart Write command: 0x08
  * @param[out]  error   pointer to an integer that specify result of function       
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::EmergencyStop(int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	unsigned char cmd[] = { addr,WriteEmergencyStop,0x00,0x00,0x00,0x00 };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}

/**
  * @brief  This command if the DATA is set at zero will stop the whole power and switching system
            connected to the motor and it will cut the current floating into the Motor from SOLO 
              .The method refers to the Uart Write command: 0x08     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::EmergencyStop()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::EmergencyStop(error);
}

/**
  * @brief  This command sets the output switching frequency of the whole power unit on the Motor
  *           .The method refers to the Uart Write command: 0x09
  * @param[in]  outputPwmFrequencyKhz  switching frequencies [kHz]
  * @param[out]  error   pointer to an integer that specify result of function     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetOutputPwmFrequencyKhz(long outputPwmFrequencyKhz, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetOutputPwmFrequencyKhzInputValidation(outputPwmFrequencyKhz, error))
	{
		return false;
	}

	unsigned char data[4];
	soloUtils->ConvertToData(outputPwmFrequencyKhz, data);
	unsigned char cmd[] = { addr,WriteOutputPwmFrequencyKhz,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);

}

/**
  * @brief  This command sets the output switching frequency of the whole power unit on the Motor
  *           .The method refers to the Uart Write command: 0x09
  * @param[in]  outputPwmFrequencyKhz  switching frequencies [kHz]      
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetOutputPwmFrequencyKhz(long outputPwmFrequencyKhz)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetOutputPwmFrequencyKhz(outputPwmFrequencyKhz, error);
}

/**
  * @brief  This command sets the Speed controller Kp Gain, and it will
  *         be functional only in Digital Closed-loop mode  
  *           .The method refers to the Uart Write command: 0x0A
  * @param[in]  speedControllerKp  a float value between 0 to 300 
  * @param[out]  error   pointer to an integer that specify result of function     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetSpeedControllerKp(float speedControllerKp, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetSpeedControllerKpInputValidation(speedControllerKp, error))
	{
		return false;
	}

	unsigned char data[4];
	soloUtils->ConvertToData(speedControllerKp, data);
	unsigned char cmd[] = { addr,WriteSpeedControllerKp,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}

/**
  * @brief  This command sets the Speed controller Kp Gain, and it will
  *         be functional only in Digital Closed-loop mode
  *           .The method refers to the Uart Write command: 0x0A
  * @param[in]  speedControllerKp  a float value between 0 to 300     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetSpeedControllerKp(float speedControllerKp)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetSpeedControllerKp(speedControllerKp, error);
}

/**
  * @brief  This command sets the Speed controller Ki gain, and it will
  *         be functional only in Digital Closed-loop mode
  *           .The method refers to the Uart Write command: 0x0B
  * @param[in]  speedControllerKi  a float value between 0 to 300 
  * @param[out]  error   pointer to an integer that specify result of function     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetSpeedControllerKi(float speedControllerKi, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetSpeedControllerKiInputValidation(speedControllerKi, error))
	{
		return false;
	}

	unsigned char data[4];
	soloUtils->ConvertToData(speedControllerKi, data);
	unsigned char cmd[] = { addr,WriteSpeedControllerKi,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}

/**
  * @brief  This command sets the Speed controller Ki gain, and it will
  *         be functional only in Digital Closed-loop mode
  *           .The method refers to the Uart Write command: 0x0B
  * @param[in]  speedControllerKi  a float value between 0 to 300      
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetSpeedControllerKi(float speedControllerKi)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetSpeedControllerKi(speedControllerKi, error);
}

/**
  * @brief  This commands sets the direction of the rotation of the motor
  *         either to ClockWise rotation or to Counter Clockwise Rotation
  *           .The method refers to the Uart Write command: 0x0C
  * @param[in]  motorDirection  enum that specify the direction of the rotation of the motor
  * @param[out]  error   pointer to an integer that specify result of function     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetMotorDirection(Direction motorDirection, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	unsigned char cmd[] = { addr,WriteMotorDirection,0x00,0x00,0x00,motorDirection };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}

/**
  * @brief  This commands sets the direction of the rotation of the motor
  *         either to ClockWise rotation or to Counter Clockwise Rotation
  *           .The method refers to the Uart Write command: 0x0C
  * @param[in]  motorDirection  enum that specify the direction of the rotation of the motor    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetMotorDirection(Direction motorDirection)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetMotorDirection(motorDirection, error);
}

/**
  * @brief  This command sets the amount of the Phase or Armature resistance
  *         for 3-phase or DC Brushed motors respectively
  *           .The method refers to the Uart Write command: 0x0D
  * @param[in]  motorResistance  a float value [Ohm]
  * @param[out]  error   pointer to an integer that specify result of function     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetMotorResistance(float motorResistance, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetMotorResistanceInputValidation(motorResistance, error))
	{
		return false;
	}

	unsigned char data[4];
	soloUtils->ConvertToData(motorResistance, data);
	unsigned char cmd[] = { addr,WriteMotorResistance,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}

/**
  * @brief  This command sets the amount of the Phase or Armature resistance
  *         for 3-phase or DC Brushed motors respectively
  *           .The method refers to the Uart Write command: 0x0D
  * @param[in]  motorResistance  a float value [Ohm]    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetMotorResistance(float motorResistance)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetMotorResistance(motorResistance, error);
}

/**
  * @brief  This command sets the amount of the Phase or Armature Inductance
  *         for 3-phase or DC Brushed motors respectively
  *           .The method refers to the Uart Write command: 0x0E
  * @param[in]  motorInductance  a float value [Henry]
  * @param[out]  error   pointer to an integer that specify result of function     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetMotorInductance(float motorInductance, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetMotorInductanceInputValidation(motorInductance, error))
	{
		return false;
	}

	unsigned char data[4];
	soloUtils->ConvertToData(motorInductance, data);
	unsigned char cmd[] = { addr,WriteMotorInductance,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}

/**
  * @brief  This command sets the amount of the Phase or Armature Inductance
  *         for 3-phase or DC Brushed motors respectively
  *           .The method refers to the Uart Write command: 0x0E
  * @param[in]  motorInductance  a float value [Henry]   
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetMotorInductance(float motorInductance)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetMotorInductance(motorInductance, error);
}

/**
  * @brief  This command sets the number of the Poles of a 3-phase motor commissioned with SOLO
  *           .The method refers to the Uart Write command: 0x0F
  * @param[in]  motorPolesCounts  a long value between 1 to 254   
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetMotorPolesCounts(long motorPolesCounts, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetMotorPolesCountsInputValidation(motorPolesCounts, error))
	{
		return false;
	}

	unsigned char data[4];
	soloUtils->ConvertToData(motorPolesCounts, data);
	unsigned char cmd[] = { addr,WriteMotorPolesCounts,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}

/**
  * @brief  This command sets the number of the Poles of a 3-phase motor commissioned with SOLO
  *           .The method refers to the Uart Write command: 0x0F
  * @param[in]  motorPolesCounts  a long value between 1 to 254     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetMotorPolesCounts(long motorPolesCounts)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetMotorPolesCounts(motorPolesCounts, error);
}

/**
  * @brief  This command sets the pre-quad number of physical lines of an 
  *         incremental encoder engraved on its disk
  *           .The method refers to the Uart Write command: 0x10
  * @param[in]  incrementalEncoderLines  a long value [pre-quad]  
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetIncrementalEncoderLines(long incrementalEncoderLines, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetIncrementalEncoderLinesInputValidation(incrementalEncoderLines, error))
	{
		return false;
	}

	unsigned char data[4];
	soloUtils->ConvertToData(incrementalEncoderLines, data);
	unsigned char cmd[] = { addr,WriteIncrementalEncoderLines,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}

/**
  * @brief  This command sets the pre-quad number of physical lines of an 
  *         incremental encoder engraved on its disk
  *           .The method refers to the Uart Write command: 0x10
  * @param[in]  incrementalEncoderLines  a long value [pre-quad]    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetIncrementalEncoderLines(long incrementalEncoderLines)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetIncrementalEncoderLines(incrementalEncoderLines, error);
}

/**
  * @brief  This command sets the allowed speed during trajectory following
  *         in closed-loop position controlling mode
  *           .The method refers to the Uart Write command: 0x11
  * @param[in]  speedLimit  a long value [RPM]  
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetSpeedLimit(long speedLimit, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetSpeedLimitInputValidation(speedLimit, error))
	{
		return false;
	}

	unsigned char data[4];
	soloUtils->ConvertToData(speedLimit, data);
	unsigned char cmd[] = { addr,WriteSpeedLimit,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}

/**
  * @brief  This command sets the allowed speed during trajectory following
  *         in closed-loop position controlling mode
  *           .The method refers to the Uart Write command: 0x11
  * @param[in]  speedLimit  a long value [RPM]     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetSpeedLimit(long speedLimit)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetSpeedLimit(speedLimit, error);
}

/**
  * @brief  This command resets the device address of any connected SOLO to zero  
  *           .The method refers to the Uart Write command: 0x12 
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::ResetDeviceAddress(int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	unsigned char cmd[] = { 0xFF,WriteResetDeviceAddress,0x00,0x00,0x00,0xFF };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}

/**
  * @brief  This command resets the device address of any connected SOLO to zero  
  *           .The method refers to the Uart Write command: 0x12  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::ResetDeviceAddress()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::ResetDeviceAddress(error);
}

/**
  * @brief  This command sets the type of the feedback control SOLO has to operate
  *           .The method refers to the Uart Write command: 0x13
  * @param[in]  mode  enum that specify the type of the feedback control SOLO 
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetFeedbackControlMode(FeedbackControlMode mode, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	unsigned char data[4];
	soloUtils->ConvertToData((long)mode, data);
	unsigned char cmd[] = { addr,WriteFeedbackControlMode,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);

}

/**
  * @brief  This command sets the type of the feedback control SOLO has to operate
  *           .The method refers to the Uart Write command: 0x13
  * @param[in]  mode  enum that specify the type of the feedback control SOLO   
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetFeedbackControlMode(FeedbackControlMode mode)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetFeedbackControlMode(mode, error);
}

/**
  * @brief  This command resets SOLO to its factory setting to all the default parameters  
  *           .The method refers to the Uart Write command: 0x14 
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::ResetFactory(int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	unsigned char cmd[] = { addr,WriteResetFactory,0x00,0x00,0x00,0x01 };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}

/**
  * @brief  This command resets SOLO to its factory setting to all the default parameters 
  *           .The method refers to the Uart Write command: 0x14    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::ResetFactory()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::ResetFactory(error);
}

/**
  * @brief  This command sets the Motor type that is connected to SOLO in Digital Mode
  *           .The method refers to the Uart Write command: 0x15
  * @param[in]  motorType  enum that specify the Motor type that is connected to SOLO in Digital Mode
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetMotorType(MotorType motorType, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	unsigned char data[4];
	soloUtils->ConvertToData((long)motorType, data);
	unsigned char cmd[] = { addr,WriteMotorType,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}

/**
  * @brief  This command sets the Motor type that is connected to SOLO in Digital Mode
  *           .The method refers to the Uart Write command: 0x15
  * @param[in]  motorType  enum that specify the Motor type that is connected to SOLO in Digital Mode 
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetMotorType(MotorType motorType)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetMotorType(motorType, error);
}

/**
  * @brief  This command sets the Control Mode in terms of Torque,
  *         Speed or Position only in Digital Mode
  *           .The method refers to the Uart Write command: 0x16
  * @param[in]  controlMode  enum that specify the Control Mode in terms of Torque,
  *                       Speed or Position only in Digital Mode 
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetControlMode(ControlMode controlMode, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	unsigned char data[4];
	soloUtils->ConvertToData((long)controlMode, data);
	unsigned char cmd[] = { addr,WriteControlMode,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}

/**
  * @brief  This command sets the Control Mode in terms of Torque,
  *         Speed or Position only in Digital Mode
  *           .The method refers to the Uart Write command: 0x16
  * @param[in]  controlMode  enum that specify the Control Mode in terms of Torque,
  *                       Speed or Position only in Digital Mode  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetControlMode(ControlMode controlMode)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetControlMode(controlMode, error);
}

/**
  * @brief  This command sets the value for Current Controller Kp or proportional gain
  *           .The method refers to the Uart Write command: 0x17
  * @param[in]  currentControllerKp  a float value between 0 to 16000  
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetCurrentControllerKp(float currentControllerKp, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetCurrentControllerKpInputValidation(currentControllerKp, error))
	{
		return false;
	}

	unsigned char data[4];
	soloUtils->ConvertToData(currentControllerKp, data);
	unsigned char cmd[] = { addr,WriteCurrentControllerKp,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}

/**
  * @brief  This command sets the value for Current Controller Kp or proportional gain
  *           .The method refers to the Uart Write command: 0x17
  * @param[in]  currentControllerKp  a float value between 0 to 16000   
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetCurrentControllerKp(float currentControllerKp)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetCurrentControllerKp(currentControllerKp, error);
}

/**
  * @brief  This command sets the value for Current Controller Ki or integral gain
  *           .The method refers to the Uart Write command: 0x18
  * @param[in]  motorInductance  a float value between 0 to 16000  
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetCurrentControllerKi(float currentControllerKi, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetCurrentControllerKiInputValidation(currentControllerKi, error))
	{
		return false;
	}

	unsigned char data[4];
	soloUtils->ConvertToData(currentControllerKi, data);
	unsigned char cmd[] = { addr,WriteCurrentControllerKi,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}

/**
  * @brief  This command sets the value for Current Controller Ki or integral gain
  *           .The method refers to the Uart Write command: 0x18
  * @param[in]  motorInductance  a float value between 0 to 16000   
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetCurrentControllerKi(float currentControllerKi)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetCurrentControllerKi(currentControllerKi, error);
}
bool SOLOMotorControllersSerial::SetMonitoringMode(bool mode, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	unsigned char cmd[] = { addr,WriteMonitoringMode,0x00,0x00,0x00,mode };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}
bool SOLOMotorControllersSerial::SetMonitoringMode(bool mode)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetMonitoringMode(mode, error);
}

/**
  * @brief  depending on the Motor type: in case of BLDC or PMSM motors Sets the Field
  *         Weakening current reference to help the motor reaching speeds higher than
  *         nominal values and in case of AC Induction Motors Sets the desired magnetizing
  *         current (Id) required for controlling ACIM motors in FOC in Amps 
  *           .The method refers to the Uart Write command: 0x1A
  * @param[in]  magnetizingCurrentIdReference  a float value [Amps]   
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetMagnetizingCurrentIdReference(float magnetizingCurrentIdReference, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetMagnetizingCurrentIdReferenceInputValidation(magnetizingCurrentIdReference, error))
	{
		return false;
	}

	unsigned char data[4];
	soloUtils->ConvertToData(magnetizingCurrentIdReference, data);
	unsigned char cmd[] = { addr,WriteMagnetizingCurrentIdReference,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}

/**
  * @brief  depending on the Motor type: in case of BLDC or PMSM motors Sets the Field
  *         Weakening current reference to help the motor reaching speeds higher than
  *         nominal values and in case of AC Induction Motors Sets the desired magnetizing
  *         current (Id) required for controlling ACIM motors in FOC in Amps 
  *           .The method refers to the Uart Write command: 0x1A
  * @param[in]  magnetizingCurrentIdReference  a float value [Amps]    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetMagnetizingCurrentIdReference(float magnetizingCurrentIdReference)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetMagnetizingCurrentIdReference(magnetizingCurrentIdReference, error);
}

/**
  * @brief  This command sets the desired Position reference in terms of quadrature
  *         pulses while SOLO operates with the Incremental Encoders or in terms of
  *         pulses while while SOLO operates with Hall sensors
  *           .The method refers to the Uart Write command: 0x1B
  * @param[in]  positionReference  a long value [Quad-Pulse]   
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetPositionReference(long positionReference, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetPositionReferenceInputValidation(positionReference, error))
	{
		return false;
	}

	unsigned char data[4];
	soloUtils->ConvertToData(positionReference, data);
	unsigned char cmd[] = { addr,WritePositionReference,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}

/**
  * @brief  This command sets the desired Position reference in terms of quadrature
  *         pulses while SOLO operates with the Incremental Encoders or in terms of
  *         pulses while while SOLO operates with Hall sensors
  *           .The method refers to the Uart Write command: 0x1B
  * @param[in]  positionReference  a long value [Quad-Pulse]   
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetPositionReference(long positionReference)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetPositionReference(positionReference, error);
}

/**
  * @brief  This command sets the value for Position Controller Kp or proportional gain
  *           .The method refers to the Uart Write command: 0x1C
  * @param[in]  positionControllerKp  a float value between 0 to 16000  
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetPositionControllerKp(float positionControllerKp, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetPositionControllerKpInputValidation(positionControllerKp, error))
	{
		return false;
	}

	unsigned char data[4];
	soloUtils->ConvertToData(positionControllerKp, data);
	unsigned char cmd[] = { addr,WritePositionControllerKp,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}

/**
  * @brief  This command sets the value for Position Controller Kp or proportional gain 
  *           .The method refers to the Uart Write command: 0x1C
  * @param[in]  positionControllerKp  a float value between 0 to 16000   
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetPositionControllerKp(float positionControllerKp)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetPositionControllerKp(positionControllerKp, error);
}

/**
  * @brief  This command sets the value for Position Controller Ki or integrator gain
  *           .The method refers to the Uart Write command: 0x1D
  * @param[in]  positionControllerKi  a float value between 0 to 16000   
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetPositionControllerKi(float positionControllerKi, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetPositionControllerKiInputValidation(positionControllerKi, error))
	{
		return false;
	}

	unsigned char data[4];
	soloUtils->ConvertToData(positionControllerKi, data);
	unsigned char cmd[] = { addr,WritePositionControllerKi,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}

/**
  * @brief  This command sets the value for Position Controller Ki or integrator gain
  *           .The method refers to the Uart Write command: 0x1D
  * @param[in]  positionControllerKi  a float value between 0 to 16000     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetPositionControllerKi(float positionControllerKi)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetPositionControllerKi(positionControllerKi, error);
}

/**
  * @brief  This command resets the position counter back to zero    
  *           .The method refers to the Uart Write command: 0x1F
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::ResetPositionToZero(int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	unsigned char cmd[] = { addr,WriteResetPositionToZero,0x00,0x00,0x00,0x01 };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}

/**
  * @brief  This command resets the position counter back to zero   
  *           .The method refers to the Uart Write command: 0x1F  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::ResetPositionToZero()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::ResetPositionToZero(error);
}

/**
  * @brief  This command overwrites the reported errors in Error Register
  *         reported with command code of "0xA1"  
  *           .The method refers to the Uart Write command: 0x20
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::OverwriteErrorRegister(int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	unsigned char cmd[] = { addr,WriteOverwriteErrorRegister,0x00,0x00,0x00,0x00 };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}

/**
  * @brief  This command overwrites the reported errors in Error Register
  *         reported with command code of "0xA1"   
  *           .The method refers to the Uart Write command: 0x20
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::OverwriteErrorRegister()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::OverwriteErrorRegister(error);
}

/**
  * @brief  This command sets the observer gain for the Non-linear observer
  *         that estimates the speed and angle of a BLDC or PMSM once the 
  *         motor type is selected as normal BLDC-PMSM
  *           .The method refers to the Uart Write command: 0x21
  * @param[in]  observerGain  a float value between 0.01 to 1000  
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
// SOG => Sensorless Observer Gain 
bool SOLOMotorControllersSerial::SetObserverGainBldcPmsm(float observerGain, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetObserverGainBldcPmsmInputValidation(observerGain, error))
	{
		return false;
	}

	unsigned char data[4];
	soloUtils->ConvertToData(observerGain, data);
	unsigned char cmd[] = { addr,WriteObserverGainBldcPmsm,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}

/**
  * @brief  This command sets the observer gain for the Non-linear observer
  *         that estimates the speed and angle of a BLDC or PMSM once the 
  *         motor type is selected as normal BLDC-PMSM
  *           .The method refers to the Uart Write command: 0x21
  * @param[in]  observerGain  a float value between 0.01 to 1000    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetObserverGainBldcPmsm(float observerGain)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetObserverGainBldcPmsm(observerGain, error);
}

/**
  * @brief  This command sets the observer gain for the Non-linear observer that
  *         estimates the speed and angle of a BLDC or PMSM once the motor type
  *         is selected as ultra-fast BLDC-PMSM
  *           .The method refers to the Uart Write command: 0x22
  * @param[in]  observerGain  a float value between 0.01 to 1000   
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetObserverGainBldcPmsmUltrafast(float observerGain, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetObserverGainBldcPmsmUltrafastInputValidation(observerGain, error))
	{
		return false;
	}

	unsigned char data[4];
	soloUtils->ConvertToData(observerGain, data);
	unsigned char cmd[] = { addr,WriteObserverGainBldcPmsmUltrafast,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}

/**
  * @brief  This command sets the observer gain for the Non-linear observer that
  *         estimates the speed and angle of a BLDC or PMSM once the motor type
  *         is selected as ultra-fast BLDC-PMSM
  *           .The method refers to the Uart Write command: 0x22
  * @param[in]  observerGain  a float value between 0.01 to 1000    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetObserverGainBldcPmsmUltrafast(float observerGain)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetObserverGainBldcPmsmUltrafast(observerGain, error);
}

/**
  * @brief  This command sets the observer gain for the Non-linear observer
  *         that estimates the speed of a DC brushed once the motor type 
  *         is selected as DC brushed
  *           .The method refers to the Uart Write command: 0x23
  * @param[in]  observerGain  a float value between 0.01 to 1000   
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetObserverGainDc(float observerGain, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetObserverGainDcInputValidation(observerGain, error))
	{
		return false;
	}

	unsigned char data[4];
	soloUtils->ConvertToData(observerGain, data);
	unsigned char cmd[] = { addr,WriteObserverGainDc,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}

/**
  * @brief  This command sets the observer gain for the Non-linear observer
  *         that estimates the speed of a DC brushed once the motor type 
  *         is selected as DC brushed
  *           .The method refers to the Uart Write command: 0x23
  * @param[in]  observerGain  a float value between 0.01 to 1000    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetObserverGainDc(float observerGain)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetObserverGainDc(observerGain, error);
}

/**
  * @brief  This command sets how fast the observer should operate once
  *         SOLO is in sensorless mode with normal BLDC-PMSM selected as the Motor type
  *           .The method refers to the Uart Write command: 0x24
  * @param[in]  filterGain  a float value between 0.01 to 16000 
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
// SOFG => Sensorless Observer Filter Gain
bool SOLOMotorControllersSerial::SetFilterGainBldcPmsm(float filterGain, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetFilterGainBldcPmsmInputValidation(filterGain, error))
	{
		return false;
	}

	unsigned char data[4];
	soloUtils->ConvertToData(filterGain, data);
	unsigned char cmd[] = { addr,WriteFilterGainBldcPmsm,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}

/**
  * @brief  This command sets how fast the observer should operate once
  *         SOLO is in sensorless mode with normal BLDC-PMSM selected as the Motor type
  *           .The method refers to the Uart Write command: 0x24
  * @param[in]  filterGain  a float value between 0.01 to 16000   
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetFilterGainBldcPmsm(float filterGain)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetFilterGainBldcPmsm(filterGain, error);
}

/**
  * @brief  This command sets how fast the observer should operate once SOLO
  *         is in sensorless mode with ultra-fast BLDC-PMSM selected as the Motor type
  *           .The method refers to the Uart Write command: 0x25
  * @param[in]  filterGain  a float value between 0.01 to 16000  
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetFilterGainBldcPmsmUltrafast(float filterGain, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetFilterGainBldcPmsmUltrafastInputValidation(filterGain, error))
	{
		return false;
	}

	unsigned char data[4];
	soloUtils->ConvertToData(filterGain, data);
	unsigned char cmd[] = { addr,WriteFilterGainBldcPmsmUltrafast,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}

/**
  * @brief  This command sets how fast the observer should operate once SOLO
  *         is in sensorless mode with ultra-fast BLDC-PMSM selected as the Motor type
  *           .The method refers to the Uart Write command: 0x25
  * @param[in]  filterGain  a float value between 0.01 to 16000   
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetFilterGainBldcPmsmUltrafast(float filterGain)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetFilterGainBldcPmsmUltrafast(filterGain, error);
}

/**
  * @brief  This command sets the baud-rate of the UART line
  *           .The method refers to the Uart Write command: 0x26
  * @param[in]  baudrate  enum that specify the baud-rate of the UART line  
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetUartBaudrate(UartBaudrate baudrate, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	unsigned char data[4];
	soloUtils->ConvertToData((long)baudrate, data);
	unsigned char cmd[] = { addr,WriteUartBaudrate,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}

/**
  * @brief  This command sets the baud-rate of the UART line
  *           .The method refers to the Uart Write command: 0x26
  * @param[in]  baudrate  enum that specify the baud-rate of the UART line     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetUartBaudrate(UartBaudrate baudrate)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetUartBaudrate(baudrate, error);
}

/**
  * @brief  This command starts or stops the process of sensor calibration
  *           .The method refers to the Uart Write command: 0x27
  * @param[in]  calibrationAction  enum that specify the process of sensor calibration 
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SensorCalibration(PositionSensorCalibrationAction calibrationAction, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	unsigned char data[4];
	soloUtils->ConvertToData((long)calibrationAction, data);
	unsigned char cmd[] = { addr, WriteSensorCalibration, data[0], data[1], data[2], data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}

/**
  * @brief  This command starts or stops the process of sensor calibration
  *           .The method refers to the Uart Write command: 0x27
  * @param[in]  calibrationAction  enum that specify the process of sensor calibration   
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SensorCalibration(PositionSensorCalibrationAction calibrationAction)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SensorCalibration(calibrationAction, error);
}

/**
  * @brief  This command sets the per-unit offset identified after sensor calibration
  *         for Encoder or Hall sensors in C.C.W direction
  *           .The method refers to the Uart Write command: 0x28
  * @param[in]  encoderHallOffset  a float value between 0.0 to 1.0  
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetEncoderHallCcwOffset(float encoderHallOffset, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetEncoderHallCcwOffsetInputValidation(encoderHallOffset, error))
	{
		return false;
	}

	unsigned char data[4];
	soloUtils->ConvertToData(encoderHallOffset, data);
	unsigned char cmd[] = { addr, WriteEncoderHallCcwOffset, data[0], data[1], data[2], data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}

/**
  * @brief  This command sets the per-unit offset identified after sensor calibration
  *         for Encoder or Hall sensors in C.C.W direction
  *           .The method refers to the Uart Write command: 0x28
  * @param[in]  encoderHallOffset  a float value between 0.0 to 1.0   
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetEncoderHallCcwOffset(float encoderHallOffset)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetEncoderHallCcwOffset(encoderHallOffset, error);
}

/**
  * @brief  This command sets the per-unit offset identified after sensor calibration
  *         for Encoder or Hall sensors in C.W direction
  *           .The method refers to the Uart Write command: 0x29
  * @param[in]  encoderHallOffset  a float value between 0.0 to 1.0   
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetEncoderHallCwOffset(float encoderHallOffset, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetEncoderHallCwOffsetInputValidation(encoderHallOffset, error))
	{
		return false;
	}

	unsigned char data[4];
	soloUtils->ConvertToData(encoderHallOffset, data);
	unsigned char cmd[] = { addr, WriteEncoderHallCwOffset, data[0], data[1], data[2], data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}

/**
  * @brief  This command sets the per-unit offset identified after sensor calibration
  *         for Encoder or Hall sensors in C.W direction
  *           .The method refers to the Uart Write command: 0x29
  * @param[in]  encoderHallOffset  a float value between 0.0 to 1.0     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetEncoderHallCwOffset(float encoderHallOffset)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetEncoderHallCwOffset(encoderHallOffset, error);
}

/**
  * @brief  This command defines the acceleration value of the Speed for speed controller
  *         both in Analogue and Digital modes in Revolution per square seconds
  *           .The method refers to the Uart Write command: 0x2A
  * @param[in]  speedAccelerationValue  a float value [Rev/S^2]  
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetSpeedAccelerationValue(float speedAccelerationValue, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetSpeedAccelerationValueInputValidation(speedAccelerationValue, error))
	{
		return false;
	}

	unsigned char data[4];
	soloUtils->ConvertToData(speedAccelerationValue, data);
	unsigned char cmd[] = { addr, WriteSpeedAccelerationValue, data[0], data[1], data[2], data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}

/**
  * @brief  This command defines the acceleration value of the Speed for speed controller
  *         both in Analogue and Digital modes in Revolution per square seconds
  *           .The method refers to the Uart Write command: 0x2A
  * @param[in]  speedAccelerationValue  a float value [Rev/S^2]  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetSpeedAccelerationValue(float speedAccelerationValue)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetSpeedAccelerationValue(speedAccelerationValue, error);
}

/**
  * @brief  This command defines the deceleration value of the Speed for speed controller
  *         both in Analogue and Digital modes in Revolution per square seconds
  *           .The method refers to the Uart Write command: 0x2B
  * @param[in]  speedDecelerationValue  a float value [Rev/S^2]   
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetSpeedDecelerationValue(float speedDecelerationValue, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetSpeedDecelerationValueInputValidation(speedDecelerationValue, error))
	{
		return false;
	}

	unsigned char data[4];
	soloUtils->ConvertToData(speedDecelerationValue, data);
	unsigned char cmd[] = { addr, WriteSpeedDecelerationValue, data[0], data[1], data[2], data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}

/**
  * @brief  This command defines the deceleration value of the Speed for speed controller
  *         both in Analogue and Digital modes in Revolution per square seconds
  *           .The method refers to the Uart Write command: 0x2B
  * @param[in]  speedDecelerationValue  a float value [Rev/S^2]     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetSpeedDecelerationValue(float speedDecelerationValue)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetSpeedDecelerationValue(speedDecelerationValue, error);
}

/**
  * @brief  This command sets the baud rate of CAN bus in CANOpen network
  *           .The method refers to the Uart Write command: 0x2C
  * @param[in]  canbusBaudrate  enum that specify the baud rate of CAN bus in CANOpen network   
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetCanbusBaudrate(CanbusBaudrate canbusBoudrate, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	unsigned char data[4];
	soloUtils->ConvertToData((long)canbusBoudrate, data);
	unsigned char cmd[] = { addr,WriteUartBaudrate,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}

/**
  * @brief  This command sets the baud rate of CAN bus in CANOpen network
  *           .The method refers to the Uart Write command: 0x2C
  * @param[in]  canbusBaudrate  enum that specify the baud rate of CAN bus in CANOpen network    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetCanbusBaudrate(CanbusBaudrate canbusBoudrate)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetCanbusBaudrate(canbusBoudrate, error);
}

/**
  * @brief  This command defines the resolution of the speed at S/T input
  *           while SOLO operates in Analogue mode
  *           .The method refers to the Uart Write command: 0x2D
  * @param[in]  divisionCoefficient  a long value    
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetAnalogueSpeedResolutionDivisionCoefficient(long divisionCoefficient, int &error)
{	
    error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetAnalogueSpeedResolutionDivisionCoefficientInputValidation(divisionCoefficient, error))
	{
		return false;
	}
	
    unsigned char data[4];
    soloUtils->ConvertToData(divisionCoefficient, data);

    unsigned char cmd[] = {addr, WriteASRDC, data[0], data[1], data[2], data[3]};
    return SOLOMotorControllersSerial::ExeCMD(cmd,error);
}

/**
  * @brief  This command defines the resolution of the speed at S/T input
  *           while SOLO operates in Analogue mode
  *           .The method refers to the Uart Write command: 0x2D
  * @param[in]  divisionCoefficient  a long value     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetAnalogueSpeedResolutionDivisionCoefficient(long divisionCoefficient)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersSerial::SetAnalogueSpeedResolutionDivisionCoefficient(divisionCoefficient, error);
}

/**
  * @brief  This command defines the type of the Motion Profile that is 
  *           being used in Speed or Position Modes
  *           .The method refers to the Uart Write command: 0x30
  * @param[in]  motionProfileMode enum that specify the type of the Motion Profile   
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetMotionProfileMode( MotionProfileMode motionProfileMode, int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;

    unsigned char data[4];
    soloUtils->ConvertToData((long)motionProfileMode, data);

    unsigned char cmd[] = {addr, WriteMotionProfileMode, data[0], data[1], data[2], data[3]};
    return SOLOMotorControllersSerial::ExeCMD(cmd,error);
}

/**
  * @brief  This command defines the type of the Motion Profile that is 
  *           being used in Speed or Position Modes
  *           .The method refers to the Uart Write command: 0x30
  * @param[in]  motionProfileMode enum that specify the type of the Motion Profile    
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetMotionProfileMode( MotionProfileMode motionProfileMode)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersSerial::SetMotionProfileMode(motionProfileMode, error);
}

/**
  * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
  *           .The method refers to the Uart Write command: 0x31  
  * @param[in]  MotionProfileVariable1 a float value   
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetMotionProfileVariable1(float MotionProfileVariable1, int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetMotionProfileVariable1InputValidation(MotionProfileVariable1, error))
	{
		return false;
	}

    unsigned char data[4];
    soloUtils->ConvertToData(MotionProfileVariable1, data);

    unsigned char cmd[] = {addr, WriteMotionProfileVariable1, data[0], data[1], data[2], data[3]};
    return SOLOMotorControllersSerial::ExeCMD(cmd,error);
}

/**
  * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
  *           .The method refers to the Uart Write command: 0x31  
  * @param[in]  MotionProfileVariable1 a float value     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetMotionProfileVariable1(float MotionProfileVariable1)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersSerial::SetMotionProfileVariable1(MotionProfileVariable1, error);
}

/**
  * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
  *           .The method refers to the Uart Write command: 0x32  
  * @param[in]  MotionProfileVariable2 a float value   
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetMotionProfileVariable2(float MotionProfileVariable2, int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetMotionProfileVariable2InputValidation(MotionProfileVariable2, error))
	{
		return false;
	}

    unsigned char data[4];
    soloUtils->ConvertToData(MotionProfileVariable2, data);

    unsigned char cmd[] = {addr, WriteMotionProfileVariable2, data[0], data[1], data[2], data[3]};
    return SOLOMotorControllersSerial::ExeCMD(cmd,error);
}

/**
  * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
  *           .The method refers to the Uart Write command: 0x32  
  * @param[in]  MotionProfileVariable2 a float value     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetMotionProfileVariable2(float MotionProfileVariable2)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersSerial::SetMotionProfileVariable2(MotionProfileVariable2, error);
}

/**
  * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
  *           .The method refers to the Uart Write command: 0x33 
  * @param[in]  MotionProfileVariable3 a float value   
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetMotionProfileVariable3(float MotionProfileVariable3, int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetMotionProfileVariable3InputValidation(MotionProfileVariable3, error))
	{
		return false;
	}

    unsigned char data[4];
    soloUtils->ConvertToData(MotionProfileVariable3, data);

    unsigned char cmd[] = {addr, WriteMotionProfileVariable3, data[0], data[1], data[2], data[3]};
    return SOLOMotorControllersSerial::ExeCMD(cmd,error);
}

/**
  * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
  *           .The method refers to the Uart Write command: 0x31  
  * @param[in]  MotionProfileVariable3 a float value     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetMotionProfileVariable3(float MotionProfileVariable3)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersSerial::SetMotionProfileVariable3(MotionProfileVariable3, error);
}

/**
  * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
  *           .The method refers to the Uart Write command: 0x34 
  * @param[in]  MotionProfileVariable4 a float value   
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetMotionProfileVariable4(float MotionProfileVariable4, int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetMotionProfileVariable4InputValidation(MotionProfileVariable4, error))
	{
		return false;
	}

    unsigned char data[4];
    soloUtils->ConvertToData(MotionProfileVariable4, data);

    unsigned char cmd[] = {addr, WriteMotionProfileVariable4, data[0], data[1], data[2], data[3]};
    return SOLOMotorControllersSerial::ExeCMD(cmd,error);
}

/**
  * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
  *           .The method refers to the Uart Write command: 0x34  
  * @param[in]  MotionProfileVariable4 a float value     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetMotionProfileVariable4(float MotionProfileVariable4)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersSerial::SetMotionProfileVariable4(MotionProfileVariable4, error);
}

/**
  * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
  *           .The method refers to the Uart Write command: 0x35  
  * @param[in]  MotionProfileVariable5 a float value   
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetMotionProfileVariable5(float MotionProfileVariable5, int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
	if (!soloUtils->SetMotionProfileVariable5InputValidation(MotionProfileVariable5, error))
	{
		return false;
	}

    unsigned char data[4];
    soloUtils->ConvertToData(MotionProfileVariable5, data);

    unsigned char cmd[] = {addr, WriteMotionProfileVariable5, data[0], data[1], data[2], data[3]};
    return SOLOMotorControllersSerial::ExeCMD(cmd,error);
}

/**
  * @brief  This parameter depending on the Motion Profile Mode and the Control Type will have roles
  *           .The method refers to the Uart Write command: 0x35  
  * @param[in]  MotionProfileVariable5 a float value     
  * @retval bool 0 fail / 1 for success
  */
bool SOLOMotorControllersSerial::SetMotionProfileVariable5(float MotionProfileVariable5)
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersSerial::SetMotionProfileVariable5(MotionProfileVariable5, error);
}

////----------Read----------

/**
  * @brief  This command reads the device address connected on the line 
  *           .The method refers to the Uart Read command: 0x81 
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long device address connected on the line
  */
long SOLOMotorControllersSerial::GetDeviceAddress(int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	unsigned char cmd[] = { 0xFF,ReadDeviceAddress,0x00,0x00,0x00,0x00 };
	//return ReadAddress;
	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToLong(data);
	}
	return -1;
}

/**
  * @brief  This command reads the device address connected on the line 
  *           .The method refers to the Uart Read command: 0x81   
  * @retval long device address connected on the line
  */
long SOLOMotorControllersSerial::GetDeviceAddress()
{
	int error = Error::noProcessedCommand;
	return GetDeviceAddress(error);
}

/**
  * @brief  This command reads the phase-A voltage of the motor connected to the
  *         "A" pin output of SOLO for 3-phase Motors 
  *           .The method refers to the Uart Read command: 0x82
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float phase-A voltage of the motor [Volts]
  */
float SOLOMotorControllersSerial::GetPhaseAVoltage(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadPhaseAVoltage,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToFloat(data);
	}
	return -1;
}

/**
  * @brief  This command reads the phase-A voltage of the motor connected to the
  *         "A" pin output of SOLO for 3-phase Motors   
  *           .The method refers to the Uart Read command: 0x82
  * @retval float phase-A voltage of the motor [Volts]
  */
float SOLOMotorControllersSerial::GetPhaseAVoltage()
{
	int error = Error::noProcessedCommand;
	return GetPhaseAVoltage(error);
}

/**
  * @brief  This command reads the phase-B voltage of the motor connected to the
  *         "B" pin output of SOLO for 3-phase Motors  
  *           .The method refers to the Uart Read command: 0x83
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float 0 phase-A voltage of the motor [Volts]
  */
float SOLOMotorControllersSerial::GetPhaseBVoltage(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadPhaseBVoltage,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToFloat(data);
	}
	return -1;
}

/**
  * @brief  This command reads the phase-B voltage of the motor connected to the
  *         "B" pin output of SOLO for 3-phase Motors  
  *           .The method refers to the Uart Read command: 0x83
  * @retval float 0 phase-A voltage of the motor [Volts]
  */
float SOLOMotorControllersSerial::GetPhaseBVoltage()
{
	int error = Error::noProcessedCommand;
	return GetPhaseBVoltage(error);
}

/**
  * @brief  This command reads the phase-A current of the motor connected to the
  *         "A" pin output of SOLO for 3-phase Motors
  *           .The method refers to the Uart Read command: 0x84
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval phase-A current of the motor [Amps]
  */
float SOLOMotorControllersSerial::GetPhaseACurrent(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadPhaseACurrent,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToFloat(data);
	}
	return -1;
}

/**
  * @brief  This command reads the phase-A current of the motor connected to the
  *         "A" pin output of SOLO for 3-phase Motors 
  *           .The method refers to the Uart Read command: 0x84
  * @retval float phase-A current of the motor [Amps]
  */
float SOLOMotorControllersSerial::GetPhaseACurrent()
{
	int error = Error::noProcessedCommand;
	return GetPhaseACurrent(error);
}

/**
  * @brief  This command reads the phase-B current of the motor connected to the
  *         "B" pin output of SOLO for 3-phase Motors 
  *           .The method refers to the Uart Read command: 0x85
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float phase-B current of the motor [Amps]
  */
float SOLOMotorControllersSerial::GetPhaseBCurrent(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadPhaseBCurrent,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToFloat(data);
	}
	return -1;
}

/**
  * @brief  This command reads the phase-B current of the motor connected to the
  *         "B" pin output of SOLO for 3-phase Motors  
  *           .The method refers to the Uart Read command: 0x85
  * @retval float phase-B current of the motor [Amps]
  */
float SOLOMotorControllersSerial::GetPhaseBCurrent()
{
	int error = Error::noProcessedCommand;
	return GetPhaseBCurrent(error);
}

/**
  * @brief  This command reads the input BUS voltage  
  *           .The method refers to the Uart Read command: 0x86
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float  BUS voltage [Volts]
  */
//Battery Voltage
float SOLOMotorControllersSerial::GetBusVoltage(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadBusVoltage,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToFloat(data);
	}
	return -1;
}

/**
  * @brief  This command reads the input BUS voltage   
  *           .The method refers to the Uart Read command: 0x86 
  * @retval float  BUS voltage [Volts]
  */
float SOLOMotorControllersSerial::GetBusVoltage()
{
	int error = Error::noProcessedCommand;
	return GetBusVoltage(error);
}

/**
  * @brief  This command reads the current inside the DC brushed motor connected to
  *         "B" and "C" outputs of SOLO 
  *           .The method refers to the Uart Read command: 0x87
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between [Amps]
  */
float SOLOMotorControllersSerial::GetDcMotorCurrentIm(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadDcMotorCurrentIm,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToFloat(data);
	}
	return -1;
}

/**
  * @brief  This command reads the current inside the DC brushed motor connected to
  *         "B" and "C" outputs of SOLO  
  *           .The method refers to the Uart Read command: 0x87
  * @retval float between [Amps]
  */
float SOLOMotorControllersSerial::GetDcMotorCurrentIm()
{
	int error = Error::noProcessedCommand;
	return GetDcMotorCurrentIm(error);
}

/**
  * @brief  This command reads the voltage of the DC brushed motor connected to
  *         "B" and "C" outputs of SOLO 
  *           .The method refers to the Uart Read command: 0x88
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float [Volts]
  */
float SOLOMotorControllersSerial::GetDcMotorVoltageVm(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadDcMotorVoltageVm,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToFloat(data);
	}
	return -1;
}

/**
  * @brief  This command reads the voltage of the DC brushed motor connected to
  *         "B" and "C" outputs of SOLO  
  *           .The method refers to the Uart Read command: 0x88
  * @retval float [Volts]
  */
float SOLOMotorControllersSerial::GetDcMotorVoltageVm()
{
	int error = Error::noProcessedCommand;
	return GetDcMotorVoltageVm(error);
}

/**
  * @brief  This command reads the value of the Speed controller Kp gain, 
  *         set for Digital mode operations  
  *           .The method refers to the Uart Read command: 0x89
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 16000
  */
float SOLOMotorControllersSerial::GetSpeedControllerKp(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadSpeedControllerKp,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToFloat(data);
	}
	return -1;
}

/**
  * @brief  This command reads the value of the Speed controller Kp gain, 
  *         set for Digital mode operations   
  *           .The method refers to the Uart Read command: 0x89
  * @retval float between 0 to 16000
  */
float SOLOMotorControllersSerial::GetSpeedControllerKp()
{
	int error = Error::noProcessedCommand;
	return GetSpeedControllerKp(error);
}

/**
  * @brief  This command reads the value of the Speed controller Ki gain,
  *         set for Digital mode operations  
  *           .The method refers to the Uart Read command: 0x8A
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 16000
  */
float SOLOMotorControllersSerial::GetSpeedControllerKi(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadSpeedControllerKi,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToFloat(data);
	}
	return -1;
}

/**
  * @brief  This command reads the value of the Speed controller Ki gain,
  *         set for Digital mode operations   
  *           .The method refers to the Uart Read command: 0x8A
  * @retval float between 0 to 16000
  */
float SOLOMotorControllersSerial::GetSpeedControllerKi()
{
	int error = Error::noProcessedCommand;
	return GetSpeedControllerKi(error);
}

/**
  * @brief  This command reads the output switching frequency of SOLO in Hertz
  *           .The method refers to the Uart Read command: 0x8B  
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long [Hz]
  */
long SOLOMotorControllersSerial::GetOutputPwmFrequencyKhz(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadOutputPwmFrequencyHz,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return (soloUtils->ConvertToLong(data) / 1000L); //PWM reading is in Hz
	}
	return -1;
}

/**
  * @brief  This command reads the output switching frequency of SOLO in Hertz 
  *           .The method refers to the Uart Read command: 0x8B  
  * @retval long [Hz]
  */
long SOLOMotorControllersSerial::GetOutputPwmFrequencyKhz()
{
	int error = Error::noProcessedCommand;
	return GetOutputPwmFrequencyKhz(error);
}

/**
  * @brief  This command reads the value of the current limit set for SOLO in
  *         closed-loop digital operation mode   
  *           .The method refers to the Uart Read command: 0x8C
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float [Amps]
  */
float SOLOMotorControllersSerial::GetCurrentLimit(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadCurrentLimit,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToFloat(data);
	}
	return -1;
}

/**
  * @brief  This command reads the value of the current limit set for SOLO in
  *         closed-loop digital operation mode   
  *           .The method refers to the Uart Read command: 0x8C
  * @retval float [Amps]
  */
float SOLOMotorControllersSerial::GetCurrentLimit()
{
	int error = Error::noProcessedCommand;
	return GetCurrentLimit(error);
}

/**
  * @brief  This command reads the actual monetary value of “Iq” that is
  *         the current acts in torque generation in FOC mode for 3-phase motors
  *           .The method refers to the Uart Read command: 0x8D
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float [Amps]
  */
float SOLOMotorControllersSerial::GetQuadratureCurrentIqFeedback(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadQuadratureCurrentIqFeedback,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToFloat(data);
	}
	return -1;
}

/**
  * @brief  This command reads the actual monetary value of “Iq” that is
  *         the current acts in torque generation in FOC mode for 3-phase motors 
  *           .The method refers to the Uart Read command: 0x8D 
  * @retval float [Amps]
  */
float SOLOMotorControllersSerial::GetQuadratureCurrentIqFeedback()
{
	int error = Error::noProcessedCommand;
	return GetQuadratureCurrentIqFeedback(error);
}

/**
  * @brief  This command reads the actual monetary value of Id that is the
  *         direct current acting in FOC 
  *           .The method refers to the Uart Read command: 0x8E
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float [Amps]
  */
float SOLOMotorControllersSerial::GetMagnetizingCurrentIdFeedback(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadMagnetizingCurrentIdFeedback,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToFloat(data);
	}
	return -1;
}

/**
  * @brief  This command reads the actual monetary value of Id that is the
  *         direct current acting in FOC  
  *           .The method refers to the Uart Read command: 0x8E
  * @retval float [Amps]
  */
float SOLOMotorControllersSerial::GetMagnetizingCurrentIdFeedback()
{
	int error = Error::noProcessedCommand;
	return GetMagnetizingCurrentIdFeedback(error);
}

/**
  * @brief  This command reads the number of Poles set for 3-phase motors  
  *           .The method refers to the Uart Read command: 0x8F
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long between 1 to 254
  */
long SOLOMotorControllersSerial::GetMotorPolesCounts(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadMotorPolesCounts,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToLong(data);
	}
	return -1;
}

/**
  * @brief  This command reads the number of Poles set for 3-phase motors 
  *           .The method refers to the Uart Read command: 0x8F  
  * @retval long between 1 to 254
  */
long SOLOMotorControllersSerial::GetMotorPolesCounts()
{
	int error = Error::noProcessedCommand;
	return GetMotorPolesCounts(error);
}

/**
  * @brief  This command reads the number of physical Incremental encoder lines set on SOLO   
  *           .The method refers to the Uart Read command: 0x90
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long between 1 to 200000
  */
long SOLOMotorControllersSerial::GetIncrementalEncoderLines(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadIncrementalEncoderLines,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToLong(data);
	}
	return -1;
}

/**
  * @brief  This command reads the number of physical Incremental encoder lines set on SOLO   
  *           .The method refers to the Uart Read command: 0x90
  * @retval long between 1 to 200000
  */
long SOLOMotorControllersSerial::GetIncrementalEncoderLines()
{
	int error = Error::noProcessedCommand;
	return GetIncrementalEncoderLines(error);
}

/**
  * @brief  This command reads the amount of value set for Current controller
  *         Kp or proportional gain 
  *           .The method refers to the Uart Read command: 0x91
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 16000
  */
float SOLOMotorControllersSerial::GetCurrentControllerKp(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadCurrentControllerKp,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToFloat(data);
	}
	return -1;
}

/**
  * @brief  This command reads the amount of value set for Current controller
  *         Kp or proportional gain  
  *           .The method refers to the Uart Read command: 0x91
  * @retval float between 0 to 16000
  */
float SOLOMotorControllersSerial::GetCurrentControllerKp()
{
	int error = Error::noProcessedCommand;
	return GetCurrentControllerKp(error);
}

/**
  * @brief  This command reads the amount of value set for Current controller
  *         Ki or integrator gain  
  *           .The method refers to the Uart Read command: 0x92
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 16000
  */
float SOLOMotorControllersSerial::GetCurrentControllerKi(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadCurrentControllerKi,0x00,0x00,0x00,0x00 };
	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToFloat(data) * 0.00005;
	}
	return -1;
}

/**
  * @brief  This command reads the amount of value set for Current controller
  *         Ki or integrator gain    
  *           .The method refers to the Uart Read command: 0x92
  * @retval float between 0 to 16000
  */
float SOLOMotorControllersSerial::GetCurrentControllerKi()
{
	int error = Error::noProcessedCommand;
	return GetCurrentControllerKi(error);
}

/**
  * @brief  This command reads the momentary temperature of the board in centigrade
  *           .The method refers to the Uart Read command: 0x93  
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float [°C]
  */
float SOLOMotorControllersSerial::GetBoardTemperature(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadBoardTemperature,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToFloat(data);
	}
	return -1;
}

/**
  * @brief  This command reads the momentary temperature of the board in centigrade  
  *           .The method refers to the Uart Read command: 0x93
  * @retval float [°C]
  */
float SOLOMotorControllersSerial::GetBoardTemperature()
{
	int error = Error::noProcessedCommand;
	return GetBoardTemperature(error);
}

/**
  * @brief  This command reads the Phase or Armature resistance of
  *         the 3-phase or DC brushed motor connected to SOLO respectively  
  *           .The method refers to the Uart Read command: 0x94
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float [Ohms]
  */
float SOLOMotorControllersSerial::GetMotorResistance(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadMotorResistance,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToFloat(data)/8;
	}
	return -1;
}

/**
  * @brief  This command reads the Phase or Armature resistance of
  *         the 3-phase or DC brushed motor connected to SOLO respectively  
  *           .The method refers to the Uart Read command: 0x94 
  * @retval float [Ohms]
  */
float SOLOMotorControllersSerial::GetMotorResistance()
{
	int error = Error::noProcessedCommand;
	return GetMotorResistance(error);
}

/**
  * @brief  This command reads the Phase or Armature Inductance of 
  *         the 3-phase or DC brushed motor connected to SOLO respectively  
  *           .The method refers to the Uart Read command: 0x95
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float [Henry]
  */
float SOLOMotorControllersSerial::GetMotorInductance(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadMotorInductance,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToFloat(data)/8;
	}
	return -1;
}

/**
  * @brief  This command reads the Phase or Armature Inductance of 
  *         the 3-phase or DC brushed motor connected to SOLO respectively 
  *           .The method refers to the Uart Read command: 0x95  
  * @retval float [Henry]
  */
float SOLOMotorControllersSerial::GetMotorInductance()
{
	int error = Error::noProcessedCommand;
	return GetMotorInductance(error);
}

/**
  * @brief  his command reads the actual speed of the motor measured or estimated by SOLO in
            sensorless or sensor-based modes respectively  
              .The method refers to the Uart Read command: 0x96
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long [RPM]
  */
long SOLOMotorControllersSerial::GetSpeedFeedback(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadSpeedFeedback,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToLong(data);
	}
	return -1;
}

/**
  * @brief  his command reads the actual speed of the motor measured or estimated by SOLO in
            sensorless or sensor-based modes respectively   
              .The method refers to the Uart Read command: 0x96
  * @retval long [RPM]
  */
long SOLOMotorControllersSerial::GetSpeedFeedback()
{
	int error = Error::noProcessedCommand;
	return GetSpeedFeedback(error);
}

/**
  * @brief  This command reads the Motor type selected for Digital or Analogue mode operations
  *           .The method refers to the Uart Read command: 0x97 
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long between 0 to 3
  */
long SOLOMotorControllersSerial::GetMotorType(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadMotorType,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToLong(data);
	}
	return -1;
}

/**
  * @brief  This command reads the Motor type selected for Digital or Analogue mode operations 
  *           .The method refers to the Uart Read command: 0x97  
  * @retval long between 0 to 3
  */
long SOLOMotorControllersSerial::GetMotorType()
{
	int error = Error::noProcessedCommand;
	return GetMotorType(error);
}

/**
  * @brief  This command reads the feedback control mode selected on SOLO both
  *         for Analogue and Digital operations  
  *           .The method refers to the Uart Read command: 0x99
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long between 0 to 2
  */
long SOLOMotorControllersSerial::GetFeedbackControlMode(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadFeedbackControlMode,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToLong(data);
	}
	return -1;
}

/**
  * @brief  This command reads the feedback control mode selected on SOLO both
  *         for Analogue and Digital operations    
  *           .The method refers to the Uart Read command: 0x99
  * @retval long between 0 to 2
  */
long SOLOMotorControllersSerial::GetFeedbackControlMode()
{
	int error = Error::noProcessedCommand;
	return GetFeedbackControlMode(error);
}

/**
  * @brief  This command reads the actual commanding mode that SOLO is operating 
  *           .The method refers to the Uart Read command: 0x9A
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long between 0 or 1
  */
long SOLOMotorControllersSerial::GetCommandMode(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadCommandMode,0x00,0x00,0x00,0x00 };
	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return  soloUtils->ConvertToLong(data);

	}
	return -1;
}

/**
  * @brief  This command reads the actual commanding mode that SOLO is operating 
  *           .The method refers to the Uart Read command: 0x9A 
  * @retval long between 0 or 1
  */
long SOLOMotorControllersSerial::GetCommandMode()
{
	int error = Error::noProcessedCommand;
	return GetCommandMode(error);
}

/**
  * @brief  This command reads the Control Mode type in terms of Torque,
  *         Speed or Position in both Digital and Analogue modes 
  *           .The method refers to the Uart Read command: 0x9B
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long between 0 to 2
  */
long SOLOMotorControllersSerial::GetControlMode(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadControlMode,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToLong(data);
	}
	return -1;
}

/**
  * @brief  This command reads the Control Mode type in terms of Torque,
  *         Speed or Position in both Digital and Analogue modes  
  *           .The method refers to the Uart Read command: 0x9B
  * @retval long between 0 to 2
  */
long SOLOMotorControllersSerial::GetControlMode()
{
	int error = Error::noProcessedCommand;
	return GetControlMode(error);
}

/**
  * @brief  This command reads the value of the speed limit set on SOLO 
  *           .The method refers to the Uart Read command: 0x9C
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long [RPM]
  */
long SOLOMotorControllersSerial::GetSpeedLimit(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadSpeedLimit,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToLong(data);
	}
	return -1;
}

/**
  * @brief  This command reads the value of the speed limit set on SOLO  
  *           .The method refers to the Uart Read command: 0x9C
  * @retval long [RPM]
  */
long SOLOMotorControllersSerial::GetSpeedLimit()
{
	int error = Error::noProcessedCommand;
	return GetSpeedLimit(error);
}

/**
  * @brief  This command reads the amount of value set for Position
  *         controller Kp or proportional gain  
  *           .The method refers to the Uart Read command: 0x9D
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 16000
  */
float SOLOMotorControllersSerial::GetPositionControllerKp(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadPositionControllerKp,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToFloat(data);
	}
	return -1;
}

/**
  * @brief  This command reads the amount of value set for Position
  *         controller Kp or proportional gain   
  *           .The method refers to the Uart Read command: 0x9D
  * @retval float between 0 to 16000
  */
float SOLOMotorControllersSerial::GetPositionControllerKp()
{
	int error = Error::noProcessedCommand;
	return GetPositionControllerKp(error);
}

/**
  * @brief  This command reads the amount of value set for Position
  *         controller Ki or integrator gain  
  *           .The method refers to the Uart Read command: 0x9E
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0 to 16000
  */
float SOLOMotorControllersSerial::GetPositionControllerKi(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadPositionControllerKi,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToFloat(data);
	}
	return -1;
}

/**
  * @brief  This command reads the amount of value set for Position
  *         controller Ki or integrator gain   
  *           .The method refers to the Uart Read command: 0x9E
  * @retval float between 0 to 16000
  */
float SOLOMotorControllersSerial::GetPositionControllerKi()
{
	int error = Error::noProcessedCommand;
	return GetPositionControllerKi(error);
}

/**
  * @brief  This command reads the number of counted pulses from the
  *         Incremental Encoder or Hall sensors 
  *           .The method refers to the Uart Read command: 0xA0
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long [Quad-Pulses]
  */
long SOLOMotorControllersSerial::GetPositionCountsFeedback(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadPositionCountsFeedback,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToLong(data);
	}
	return -1;
}

/**
  * @brief  This command reads the number of counted pulses from the
  *         Incremental Encoder or Hall sensors  
  *           .The method refers to the Uart Read command: 0xA0
  * @retval long [Quad-Pulses]
  */
long SOLOMotorControllersSerial::GetPositionCountsFeedback()
{
	int error = Error::noProcessedCommand;
	return GetPositionCountsFeedback(error);
}

/**
  * @brief  This command reads the error register which is a 32 bit register with
  *         each bit corresponding to specific errors  
  *           .The method refers to the Uart Read command: 0xA1
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long 
  */
long SOLOMotorControllersSerial::GetErrorRegister(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadErrorRegister,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToLong(data);
	}
	return -1;
}

/**
  * @brief  This command reads the error register which is a 32 bit register with
  *         each bit corresponding to specific errors   
  *           .The method refers to the Uart Read command: 0xA1
  * @retval long 
  */
long SOLOMotorControllersSerial::GetErrorRegister()
{
	int error = Error::noProcessedCommand;
	return GetErrorRegister(error);
}

/**
  * @brief  This command reads the Firmware version existing currently on the SOLO unit   
  *           .The method refers to the Uart Read command: 0xA2  
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long
  */
long SOLOMotorControllersSerial::GetDeviceFirmwareVersion(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadDeviceFirmwareVersion,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToLong(data);
	}
	return -1;
}

/**
  * @brief  This command reads the Firmware version existing currently on the SOLO unit   
  *           .The method refers to the Uart Read command: 0xA2
  * @retval long
  */
long SOLOMotorControllersSerial::GetDeviceFirmwareVersion()
{
	int error = Error::noProcessedCommand;
	return GetDeviceFirmwareVersion(error);
}

/**
  * @brief  This command reads the Hardware version of the SOLO unit connected   
  *           .The method refers to the Uart Read command: 0xA3
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long
  */
long SOLOMotorControllersSerial::GetDeviceHardwareVersion(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadDeviceHardwareVersion,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToLong(data);
	}
	return -1;
}

/**
  * @brief  This command reads the Hardware version of the SOLO unit connected    
  *           .The method refers to the Uart Read command: 0xA3 
  * @retval long
  */
long SOLOMotorControllersSerial::GetDeviceHardwareVersion()
{
	int error = Error::noProcessedCommand;
	return GetDeviceHardwareVersion(error);
}

/**
  * @brief  This command reads the amount of desired Torque reference (Iq or IM)
  *         already set for the Motor to follow in Digital Closed-loop Torque control mode  
  *           .The method refers to the Uart Read command: 0xA4
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float [Amps]
  */
float SOLOMotorControllersSerial::GetTorqueReferenceIq(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadTorqueReferenceIq,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToFloat(data);
	}
	return -1;
}

/**
  * @brief  This command reads the amount of desired Torque reference (Iq or IM)
  *         already set for the Motor to follow in Digital Closed-loop Torque control mode  
  *           .The method refers to the Uart Read command: 0xA4 
  * @retval float [Amps]
  */
float SOLOMotorControllersSerial::GetTorqueReferenceIq()
{
	int error = Error::noProcessedCommand;
	return GetTorqueReferenceIq(error);
}

/**
  * @brief  This command reads the amount of desired Speed reference already set for
  *         the Motor to follow in Digital Closed-loop Speed control mode  
  *           .The method refers to the Uart Read command: 0xA5
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long [RPM]
  */
long SOLOMotorControllersSerial::GetSpeedReference(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadSpeedReference,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToLong(data);
	}
	return -1;
}

/**
  * @brief  This command reads the amount of desired Speed reference already set for
  *         the Motor to follow in Digital Closed-loop Speed control mode   
  *           .The method refers to the Uart Read command: 0xA5
  * @retval long [RPM]
  */
long SOLOMotorControllersSerial::GetSpeedReference()
{
	int error = Error::noProcessedCommand;
	return GetSpeedReference(error);
}

/**
  * @brief  This command reads the amount of desired Id (direct current) or
  *         Magnetizing current reference already set for the Motor to follow
  *         in Digital Closed-loop Speed control mode for ACIM motors 
  *           .The method refers to the Uart Read command: 0xA6
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float [Amps]
  */
float SOLOMotorControllersSerial::GetMagnetizingCurrentIdReference(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadMagnetizingCurrentIdReference,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToFloat(data);
	}
	return -1;
}

/**
  * @brief  This command reads the amount of desired Id (direct current) or
  *         Magnetizing current reference already set for the Motor to follow
  *         in Digital Closed-loop Speed control mode for ACIM motors  
  *           .The method refers to the Uart Read command: 0xA6
  * @retval float [Amps]
  */
float SOLOMotorControllersSerial::GetMagnetizingCurrentIdReference()
{
	int error = Error::noProcessedCommand;
	return GetMagnetizingCurrentIdReference(error);
}

/**
  * @brief  This command reads the desired position reference set for the Motor
  *         to follow in Digital Closed-loop Position mode in terms of quadrature pulses 
  *           .The method refers to the Uart Read command: 0xA7
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long [Quad-Pulses]
  */
long SOLOMotorControllersSerial::GetPositionReference(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadPositionReference,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToLong(data);
	}
	return -1;
}

/**
  * @brief  This command reads the desired position reference set for the Motor
  *         to follow in Digital Closed-loop Position mode in terms of quadrature pulses 
  *           .The method refers to the Uart Read command: 0xA7 
  * @retval long [Quad-Pulses]
  */
long SOLOMotorControllersSerial::GetPositionReference()
{
	int error = Error::noProcessedCommand;
	return GetPositionReference(error);
}

/**
  * @brief  This command reads the desired Power reference for SOLO to apply in 
  *         Digital Open-loop speed control mode for 3-phase motors in terms of percentage
  *           .The method refers to the Uart Read command: 0xA8
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float [%]
  */
float SOLOMotorControllersSerial::GetPowerReference(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadPowerReference,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToFloat(data);
	}
	return -1;
}

/**
  * @brief  This command reads the desired Power reference for SOLO to apply in 
  *         Digital Open-loop speed control mode for 3-phase motors in terms of percentage
  *           .The method refers to the Uart Read command: 0xA8
  * @retval float [%]
  */
float SOLOMotorControllersSerial::GetPowerReference()
{
	int error = Error::noProcessedCommand;
	return GetPowerReference(error);
}

/**
  * @brief  This commands reads the desired direction of rotation set for the Motor 
  *           .The method refers to the Uart Read command: 0xA9
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long 0 Counter ClockWise / 1 ClockWise
  */
long SOLOMotorControllersSerial::GetMotorDirection(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadMotorDirection,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToLong(data);
	}
	return -1;
}

/**
  * @brief  This commands reads the desired direction of rotation set for the Motor   
  *           .The method refers to the Uart Read command: 0xA9
  * @retval long 0 Counter ClockWise / 1 ClockWise
  */
long SOLOMotorControllersSerial::GetMotorDirection()
{
	int error = Error::noProcessedCommand;
	return GetMotorDirection(error);
}

/**
  * @brief  This command reads the value of Sensorless Observer Gain for Normal BLDC-PMSM Motors    
  *           .The method refers to the Uart Read command: 0xAA
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0.01 to 1000
  */
float SOLOMotorControllersSerial::GetObserverGainBldcPmsm(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadObserverGainBldcPmsm,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToFloat(data);
	}
	return -1;
}

/**
  * @brief  This command reads the value of Sensorless Observer Gain for Normal BLDC-PMSM Motors  
  *           .The method refers to the Uart Read command: 0xAA 
  * @retval float between 0.01 to 1000
  */
float SOLOMotorControllersSerial::GetObserverGainBldcPmsm()
{
	int error = Error::noProcessedCommand;
	return GetObserverGainBldcPmsm(error);
}

/**
  * @brief  This command reads the value of Sensorless Observer Gain for Normal BLDC-PMSM Motors 
  *           .The method refers to the Uart Read command: 0xAB
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0.01 to 1000
  */
float SOLOMotorControllersSerial::GetObserverGainBldcPmsmUltrafast(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadObserverGainBldcPmsmUltrafast,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToFloat(data);
	}
	return -1;
}

/**
  * @brief  This command reads the value of Sensorless Observer Gain for Normal BLDC-PMSM Motors  
  *           .The method refers to the Uart Read command: 0xAB
  * @retval float between 0.01 to 1000
  */
float SOLOMotorControllersSerial::GetObserverGainBldcPmsmUltrafast()
{
	int error = Error::noProcessedCommand;
	return GetObserverGainBldcPmsmUltrafast(error);
}

/**
  * @brief  This command reads the value of Sensorless Observer Gain for DC Motor  
  *           .The method refers to the Uart Read command: 0xAC
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0.01 to 1000
  */
float SOLOMotorControllersSerial::GetObserverGainDc(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadObserverGainDc,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToFloat(data);
	}
	return -1;
}

/**
  * @brief  This command reads the value of Sensorless Observer Gain for DC Motor  
  *           .The method refers to the Uart Read command: 0xAC 
  * @retval float between 0.01 to 1000
  */
float SOLOMotorControllersSerial::GetObserverGainDc()
{
	int error = Error::noProcessedCommand;
	return GetObserverGainDc(error);
}

/**
  * @brief  This command reads the value of Sensorless Observer
  *         Filter Gain for Normal BLDC-PMSM Motors  
  *           .The method refers to the Uart Read command: 0xAD
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0.01 to 16000
  */
float SOLOMotorControllersSerial::GetFilterGainBldcPmsm(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadFilterGainBldcPmsm,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToFloat(data);
	}
	return -1;
}

/**
  * @brief  This command reads the value of Sensorless Observer
  *         Filter Gain for Normal BLDC-PMSM Motors    
  *           .The method refers to the Uart Read command: 0xAD
  * @retval float between 0.01 to 16000
  */
float SOLOMotorControllersSerial::GetFilterGainBldcPmsm()
{
	int error = Error::noProcessedCommand;
	return GetFilterGainBldcPmsm(error);
}

/**
  * @brief  This command reads the value of Sensorless Observer
  *         Filter Gain for Ultra Fast BLDC-PMSM Motors 
  *           .The method refers to the Uart Read command: 0xAE
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float between 0.01 to 16000
  */
float SOLOMotorControllersSerial::GetFilterGainBldcPmsmUltrafast(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadFilterGainBldcPmsmUltrafast,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToFloat(data);
	}
	return -1;
}

/**
  * @brief  This command reads the value of Sensorless Observer
  *         Filter Gain for Ultra Fast BLDC-PMSM Motors  
  *           .The method refers to the Uart Read command: 0xAE 
  * @retval float between 0.01 to 16000
  */
float SOLOMotorControllersSerial::GetFilterGainBldcPmsmUltrafast()
{
	int error = Error::noProcessedCommand;
	return GetFilterGainBldcPmsmUltrafast(error);
}

/**
  * @brief  This command reads the measured or estimated per-unit angle of the 3-phase motors 
  *           .The method refers to the Uart Read command: 0xB0 
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float [Per Unit]
  */
float SOLOMotorControllersSerial::Get3PhaseMotorAngle(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr, Read3PhaseMotorAngle, 0x00, 0x00, 0x00, 0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToFloat(data);
	}
	return -1;
}

/**
  * @brief  This command reads the measured or estimated per-unit angle of the 3-phase motors   
  *           .The method refers to the Uart Read command: 0xB0
  * @retval float [Per Unit]
  */
float SOLOMotorControllersSerial::Get3PhaseMotorAngle()
{
	int error = Error::noProcessedCommand;
	return Get3PhaseMotorAngle(error);
}

/**
  * @brief  This command reads the per-unit Encoder or Hall sensor offset in C.C.W direction  
  *           .The method refers to the Uart Read command: 0xB1
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float [Per Unit]
  */
float SOLOMotorControllersSerial::GetEncoderHallCcwOffset(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr, ReadEncoderHallCcwOffset, 0x00, 0x00, 0x00, 0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToFloat(data);
	}
	return -1;
}

/**
  * @brief  This command reads the per-unit Encoder or Hall sensor offset in C.C.W direction    
  *           .The method refers to the Uart Read command: 0xB1
  * @retval float [Per Unit]
  */
float SOLOMotorControllersSerial::GetEncoderHallCcwOffset()
{
	int error = Error::noProcessedCommand;
	return GetEncoderHallCcwOffset(error);
}

/**
  * @brief  This command reads the per-unit Encoder or Hall sensor offset in C.C.W direction 
  *           .The method refers to the Uart Read command: 0xB2 
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float [Per Unit]
  */
float SOLOMotorControllersSerial::GetEncoderHallCwOffset(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr, ReadEncoderHallCwOffset, 0x00, 0x00, 0x00, 0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToFloat(data);
	}
	return -1;
}

/**
  * @brief  This command reads the per-unit Encoder or Hall sensor offset in C.C.W direction 
  *           .The method refers to the Uart Read command: 0xB2  
  * @retval float [Per Unit]
  */
float SOLOMotorControllersSerial::GetEncoderHallCwOffset()
{
	int error = Error::noProcessedCommand;
	return GetEncoderHallCwOffset(error);
}

/**
  * @brief  This command reads Baud Rate selected on SOLO unit to communicate through UART line  
  *           .The method refers to the Uart Read command: 0xB3
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long [Bits/s]
  */
long SOLOMotorControllersSerial::GetUartBaudrate(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadUartBaudrate,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToLong(data);
	}
	return -1;
}

/**
  * @brief  This command reads Baud Rate selected on SOLO unit to communicate through UART line   
  *           .The method refers to the Uart Read command: 0xB3
  * @retval long [Bits/s]
  */
long SOLOMotorControllersSerial::GetUartBaudrate()
{
	int error = Error::noProcessedCommand;
	return GetUartBaudrate(error);
}

/**
  * @brief  This command reads the acceleration value of the Speed for
  *         speed controller both in Analogue and Digital modes
  *         in Revolution per square seconds  
  *           .The method refers to the Uart Read command: 0xB4
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float [Rev/S^2]
  */
float SOLOMotorControllersSerial::GetSpeedAccelerationValue(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr, ReadSpeedAccelerationValue, 0x00, 0x00, 0x00, 0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToFloat(data);
	}
	return -1;
}

/**
  * @brief  This command reads the acceleration value of the Speed for
  *         speed controller both in Analogue and Digital modes
  *         in Revolution per square seconds  
  *           .The method refers to the Uart Read command: 0xB4
  * @retval float [Rev/S^2]
  */
float SOLOMotorControllersSerial::GetSpeedAccelerationValue()
{
	int error = Error::noProcessedCommand;
	return GetSpeedAccelerationValue(error);
}

/**
  * @brief  This command reads the deceleration value of the Speed for
  *         speed controller both in Analogue and Digital modes
  *         in Revolution per square seconds 
  *           .The method refers to the Uart Read command: 0xB5
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float [Rev/S^2]
  */
float SOLOMotorControllersSerial::GetSpeedDecelerationValue(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr, ReadSpeedDecelerationValue, 0x00, 0x00, 0x00, 0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToFloat(data);
	}
	return -1;
}

/**
  * @brief  This command reads the deceleration value of the Speed for
  *         speed controller both in Analogue and Digital modes
  *         in Revolution per square seconds  
  *           .The method refers to the Uart Read command: 0xB5
  * @retval float [Rev/S^2]
  */
float SOLOMotorControllersSerial::GetSpeedDecelerationValue()
{
	int error = Error::noProcessedCommand;
	return GetSpeedDecelerationValue(error);
}

/**
  * @brief  This Command reads the number of counted index pulses 
  *         seen on the Incremental Encoder’s output  
  *           .The method refers to the Uart Read command: 0xB8
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long [Pulses]
  */
long SOLOMotorControllersSerial::GetEncoderIndexCounts(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadEncoderIndexCounts,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToLong(data);
	}
	return -1;
}

/**
  * @brief  This Command reads the number of counted index pulses 
  *         seen on the Incremental Encoder’s output  
  *           .The method refers to the Uart Read command: 0xB8 
  * @retval long [Pulses]
  */
long SOLOMotorControllersSerial::GetEncoderIndexCounts()
{
	int error = Error::noProcessedCommand;
	return GetEncoderIndexCounts(error);
}

/**
 * 
  * @brief  This command test if the communication is working   
  * @retval bool 0 not working / 1 for working
  */
bool SOLOMotorControllersSerial::CommunicationIsWorking(int& error)
{
	error = Error::noProcessedCommand;
	float temperature = GetBoardTemperature(error);
	if (error == SOLOMotorControllers::Error::noErrorDetected) {
		return true;
	}
	return false;
}

/**
  * @brief  This command test if the communication is working   
  * @retval bool 0 not working / 1 for working
  */
bool SOLOMotorControllersSerial::CommunicationIsWorking()
{
	int error = Error::noProcessedCommand;
	return CommunicationIsWorking(error);
}

/**
  * @brief  This command reads the Analogue Speed Resolution Division Coefficient (ASRDC)
  *           .The method refers to the Uart Read command: 0xB7
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long
  */
long SOLOMotorControllersSerial::GetAnalogueSpeedResolutionDivisionCoefficient(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr, ReadASRDC, 0x00, 0x00, 0x00, 0x00};

    if (SOLOMotorControllersSerial::ExeCMD(cmd,error))
    {
        unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToLong(data);
    }
    return -1;
}

/**
  * @brief  This command reads the Analogue Speed Resolution Division Coefficient (ASRDC)
  *           .The method refers to the Uart Read command: 0xB7
  * @retval long
  */
long SOLOMotorControllersSerial::GetAnalogueSpeedResolutionDivisionCoefficient()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersSerial::GetAnalogueSpeedResolutionDivisionCoefficient(error);
}

/**
  * @brief  This command reads the type of the Embedded Motion profile active in the controller 
  *           .The method refers to the Uart Read command: 0xBB
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval long
  */
long SOLOMotorControllersSerial::GetMotionProfileMode(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr, ReadMotionProfileMode, 0x00, 0x00, 0x00, 0x00};

    if (SOLOMotorControllersSerial::ExeCMD(cmd,error))
    {
        unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToLong(data);
    }
    return -1;
}

/**
  * @brief  This command reads the type of the Embedded Motion profile active in the controller 
  *           .The method refers to the Uart Read command: 0xBB  
  * @retval long
  */
long SOLOMotorControllersSerial::GetMotionProfileMode()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersSerial::GetMotionProfileMode(error);
}

/**
  * @brief  This command reads the value of the Motion Profile Variable1 set inside the controller 
  *           .The method refers to the Uart Read command: 0xBC
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float
  */
float SOLOMotorControllersSerial::GetMotionProfileVariable1(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr, ReadMotionProfileVariable1, 0x00, 0x00, 0x00, 0x00};

    if (SOLOMotorControllersSerial::ExeCMD(cmd,error))
    {
        unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToFloat(data);
    }
    return -1;
}

/**
  * @brief  This command reads the value of the Motion Profile Variable1 set inside the controller 
  *           .The method refers to the Uart Read command: 0xBC  
  * @retval float
  */
float SOLOMotorControllersSerial::GetMotionProfileVariable1()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersSerial::GetMotionProfileVariable1(error);
}

/**
  * @brief  This command reads the value of the Motion Profile Variable2 set inside the controller 
  *           .The method refers to the Uart Read command: 0xBD
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float
  */
float SOLOMotorControllersSerial::GetMotionProfileVariable2(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr, ReadMotionProfileVariable2, 0x00, 0x00, 0x00, 0x00};

    if (SOLOMotorControllersSerial::ExeCMD(cmd,error))
    {
        unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToFloat(data);
    }
    return -1;
}

/**
  * @brief  This command reads the value of the Motion Profile Variable2 set inside the controller 
  *           .The method refers to the Uart Read command: 0xBD 
  * @retval float
  */
float SOLOMotorControllersSerial::GetMotionProfileVariable2()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersSerial::GetMotionProfileVariable2(error);
}

/**
  * @brief  This command reads the value of the Motion Profile Variable3 set inside the controller 
  *           .The method refers to the Uart Read command: 0xBE
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float
  */
float SOLOMotorControllersSerial::GetMotionProfileVariable3(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr, ReadMotionProfileVariable3, 0x00, 0x00, 0x00, 0x00};

    if (SOLOMotorControllersSerial::ExeCMD(cmd,error))
    {
        unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToFloat(data);
    }
    return -1;
}

/**
  * @brief  This command reads the value of the Motion Profile Variable3 set inside the controller 
  *           .The method refers to the Uart Read command: 0xBE 
  * @retval float
  */
float SOLOMotorControllersSerial::GetMotionProfileVariable3()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersSerial::GetMotionProfileVariable3(error);
}

/**
  * @brief  This command reads the value of the Motion Profile Variable4 set inside the controller
  *           .The method refers to the Uart Read command: 0xBF
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float
  */
float SOLOMotorControllersSerial::GetMotionProfileVariable4(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr, ReadMotionProfileVariable4, 0x00, 0x00, 0x00, 0x00};

    if (SOLOMotorControllersSerial::ExeCMD(cmd,error))
    {
        unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToFloat(data);
    }
    return -1;
}

/**
  * @brief  This command reads the value of the Motion Profile Variable4 set inside the controller
  *           .The method refers to the Uart Read command: 0xBF  
  * @retval float
  */
float SOLOMotorControllersSerial::GetMotionProfileVariable4()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersSerial::GetMotionProfileVariable4(error);
}

/**
  * @brief  This command reads the value of the Motion Profile Variable5 set inside the controller 
  *           .The method refers to the Uart Read command: 0xC0
  * @param[out]  error   pointer to an integer that specify result of function  
  * @retval float
  */
float SOLOMotorControllersSerial::GetMotionProfileVariable5(int &error)
{
    error = SOLOMotorControllers::Error::noProcessedCommand;
    unsigned char cmd[] = {addr, ReadMotionProfileVariable5, 0x00, 0x00, 0x00, 0x00};

    if (SOLOMotorControllersSerial::ExeCMD(cmd,error))
    {
        unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return soloUtils->ConvertToFloat(data);
    }
    return -1;  
}

/**
  * @brief  This command reads the value of the Motion Profile Variable5 set inside the controller 
  *           .The method refers to the Uart Read command: 0xC0 
  * @retval float
  */
float SOLOMotorControllersSerial::GetMotionProfileVariable5()
{
    int error = SOLOMotorControllers::Error::noProcessedCommand;
    return SOLOMotorControllersSerial::GetMotionProfileVariable5(error);
}