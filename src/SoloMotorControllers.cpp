// Copyright: (c) 2021, SOLO motor controllers project
// GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

/*
*    Title: SOLO Motor Controllers DLL
*    Author: SOLOMotorControllers
*    Date: 2022
*    Code version: 0.0.0
*    Availability: https://github.com/Solo-FL/SOLO-motor-controllers-CPP-library
This Library is made by SOLOMotorControllers.com
To learn more please visit:  https://www.SOLOMotorControllers.com/
*/

#include "SoloMotorControllers.h"

//DEBUG
//#include <iostream>
//using std::cout, std::endl;

#define ReadData                            0x00 // 0x00000000
#define INITIATOR                           0xFF //0xFFFF
#define BroadcastAddress                    0xFF
#define ENDING                              0xFE
#define ERR		                          	0xEE //0xEEEEEEEE
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
#define ReadTorqueReferenceIq               0xA4 // Read Torque /“Iq” Reference
#define ReadSpeedReference                  0xA5 // Read Speed Reference
#define ReadMagnetizingCurrentIdReference   0xA6 // Read Magnetizing Current / “Id” Reference
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

SOLOMotorControllers::SOLOMotorControllers(unsigned char _addr, long _baudrate, long _millisecondsTimeout, int _packetFailureTrialAttempts)
	:addr(_addr)
	, baudrate(_baudrate)
	, millisecondsTimeout(_millisecondsTimeout)
	, packetFailureTrialAttempts(_packetFailureTrialAttempts)
{

}

SOLOMotorControllers::~SOLOMotorControllers() 
{
	Disconnect();
	isConnected = false;
}

bool SOLOMotorControllers::serialSetup(unsigned char _addr, char* _portName, long _baudrate, long _millisecondsTimeout, int _packetFailureTrialAttempts)
{
	addr = _addr;
	portName = _portName;
	baudrate = _baudrate;
	millisecondsTimeout = _millisecondsTimeout;
	packetFailureTrialAttempts = _packetFailureTrialAttempts;
	return SOLOMotorControllers::Connect();

}

boolean SOLOMotorControllers::Connect()
{
	
	sprintf_s(ComPortName, "\\\\.\\%s", portName);
	//strcpy_s(ComPortName, _portName);
	
	CloseHandle(hComm);
	bool connectionError = true;

	hComm = CreateFile( (LPCSTR) ComPortName,        // Name of the Port to be Opened
		GENERIC_READ | GENERIC_WRITE,      // Read/Write Access
		0,                                 // No Sharing, ports cant be shared
		NULL,                              // No Security
		OPEN_EXISTING,                     // Open existing port only
		0,                                 // Non Overlapped I/O
		NULL);                             // Null for Comm Devices
	if (hComm == INVALID_HANDLE_VALUE)
	{
		return false;
	}

	DCB dcbSerialParams = { 0 };                        // Initializing DCB structure
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

	switch (baudrate)
	{
	case 9600:
		dcbSerialParams.BaudRate = CBR_9600;
		break;
	case 14400:
		dcbSerialParams.BaudRate = CBR_14400;
		break;
	case 19200:
		dcbSerialParams.BaudRate = CBR_19200;
		break;
	case 38400:
		dcbSerialParams.BaudRate = CBR_38400;
		break;
	case 56000:
		dcbSerialParams.BaudRate = CBR_56000;
		break;
	case 57600:
		dcbSerialParams.BaudRate = CBR_57600;
		break;
	case 115200:
		dcbSerialParams.BaudRate = CBR_115200;
		break;
	case 937500:
		dcbSerialParams.BaudRate = 937500;
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

	Status = SetCommState(hComm, &dcbSerialParams);  //Configuring the port according to settings in DCB 

	if (Status == FALSE)
	{
		return false;
	}

	COMMTIMEOUTS timeouts = { 0 };

	timeouts.ReadIntervalTimeout = 50;
	timeouts.ReadTotalTimeoutConstant = 50;
	timeouts.ReadTotalTimeoutMultiplier = 10;
	timeouts.WriteTotalTimeoutConstant = 50;
	timeouts.WriteTotalTimeoutMultiplier = 10;

	if (SetCommTimeouts(hComm, &timeouts) == FALSE)
		return false;

	isConnected = true;
	Sleep(50);

	return true;
}

void SOLOMotorControllers::Disconnect()
{
	isConnected = false;
	CloseHandle(hComm);
}

bool SOLOMotorControllers::Test()
{

	unsigned char cmd[] = { 0xFF,0xFF,65,66,67,68,69,70,0,0xFE };
	int i = 0;
	if (addr == 0x01)
		return true;
	else return false;
}

bool SOLOMotorControllers::ExeCMD(unsigned char* cmd, int& error)
{
	
	unsigned char _cmd[10] = { INITIATOR, INITIATOR, cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], CRC, ENDING };
	unsigned char _readPacket[20];
	unsigned char idx = 0;

	bool isPacketFailureTrialAttemptsOverflow = true;
	//FailureTrialAttempts block
	for (int attempts = 0; attempts < packetFailureTrialAttempts; attempts++)
	{
		idx = 0;
		
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

		Status = WriteFile(hComm,               // Handle to the Serialport
			_cmd,            // Data to be written to the port 
			dNoOFBytestoWrite,   // No of bytes to write into the port
			&dNoOfBytesWritten,  // No of bytes written to the port
			NULL);
		
		//std::cout << "EXECMD S: " << Status << std::endl; 	
		if (Status != TRUE){
			if(isConnected){
				//std::cout << "DISCONNECT" << std::endl;
				SOLOMotorControllers::Disconnect();
			}else{
				SOLOMotorControllers::Connect();
			}
			continue;
		}

		/*------------------------------------ Setting Receive Mask ----------------------------------------------*/

		Status = SetCommMask(hComm, EV_RXCHAR); //Configure Windows to Monitor the serial device for Character Reception
		//std::cout << "Status: " << Status << std::endl; 
		
		
		if (Status == FALSE){
			continue;
		}
		else //If  WaitCommEvent()==True Read the RXed data using ReadFile();
		{
			do
			{
				Status = ReadFile(hComm, &TempChar, sizeof(TempChar), &NoBytesRecieved, NULL);
				_readPacket[idx] = TempChar;
				//Read messages
				//std::cout << (long) _readPacket[idx] << " ";
				idx++;
			} while (NoBytesRecieved > 0);
			//std::cout << std::endl; 
		}

		if (Status)
		{
			if (_readPacket[0] != 0xFF || _readPacket[1] != 0xFF || _readPacket[9] != 0xFE)
			{
				continue;
			}
			else
			{
				isPacketFailureTrialAttemptsOverflow = false;
				break;
			}
		}
	}

	if (isPacketFailureTrialAttemptsOverflow)
	{
		cmd[0] = ERR;
		cmd[1] = ERR;
		cmd[2] = ERR;
		cmd[3] = ERR;
		cmd[4] = ERR;
		cmd[5] = ERR;
		error = SOLOMotorControllers::packetFailureTrialAttemptsOverflow;
		Sleep(500);
		return false;
	}
	else
	{
		if (_readPacket[0] == _cmd[0] && _readPacket[1] == _cmd[1]
			&& (_readPacket[2] == _cmd[2] || _cmd[2] == 0xFF) && _readPacket[3] == _cmd[3]
			&& _readPacket[8] == _cmd[8] && _readPacket[9] == _cmd[9])
		{
			cmd[0] = _readPacket[2];
			cmd[1] = _readPacket[3];
			cmd[2] = _readPacket[4];
			cmd[3] = _readPacket[5];
			cmd[4] = _readPacket[6];
			cmd[5] = _readPacket[7];

			error = SOLOMotorControllersError::noErrorDetected;
			return true;
		}

		else
		{
			cmd[0] = ERR;
			cmd[1] = ERR;
			cmd[2] = ERR;
			cmd[3] = ERR;
			cmd[4] = ERR;
			cmd[5] = ERR;
			error = SOLOMotorControllersError::generalError;
			return false;
		}
	}
}

float SOLOMotorControllers::ConvertToFloat(unsigned char* data)
{
	long dec = 0;
	dec = (long)data[0] << 24;
	dec += (long)data[1] << 16;
	dec += (long)data[2] << 8;
	dec += (long)data[3];

	if (dec <= 0x7FFE0000)
	{
		return (float)(dec / 131072.0);
	}
	else
	{
		dec = 0xFFFFFFFF - dec + 1;
		return ((float)(dec / 131072.0)) * -1;
	}
}

void SOLOMotorControllers::ConvertToData(float f, unsigned char* data)
{
	long dec = (long)(f * 131072);
	if (dec < 0)
	{
		dec *= -1;
		dec = 0xFFFFFFFF - dec;
	}
	data[0] = (UINT8)(dec >> 24);
	dec = dec % 16777216;
	data[1] = (UINT8)(dec >> 16);
	dec = dec % 65536;
	data[2] = (UINT8)(dec >> 8);
	data[3] = (UINT8)(dec % 256);
}

long SOLOMotorControllers::ConvertToLong(unsigned char* data)
{
	long dec = 0;
	dec = (long)data[0] << 24;
	dec += (long)data[1] << 16;
	dec += (long)data[2] << 8;
	dec += (long)data[3];

	if (dec <= 2147483647/*0x7FFFFFFF*/)
	{
		return dec;
	}
	else
	{
		dec = /*0xFFFFFFFF*/4294967295 - dec + 1;
		return dec * -1;
	}
}

void SOLOMotorControllers::ConvertToData(long l, unsigned char* data)
{
	long dec = l;
	if (dec < 0)
	{
		dec *= -1;
		dec = 0xFFFFFFFF - dec + 1;
	}
	data[0] = dec >> 24;
	dec = dec % 16777216;
	data[1] = dec >> 16;
	dec = dec % 65536;
	data[2] = dec >> 8;
	data[3] = dec % 256;
}

void SOLOMotorControllers::SplitData(unsigned char* data, unsigned char* cmd)
{
	data[0] = cmd[2];
	data[1] = cmd[3];
	data[2] = cmd[4];
	data[3] = cmd[5];
}

bool SOLOMotorControllers::SetDeviceAddress(unsigned char deviceAddress, int& error)
{
	error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,WriteDeviceAddres,0x00,0x00,0x00,deviceAddress };

	if (deviceAddress < 0 || deviceAddress > 254)
	{
		error = SOLOMotorControllers::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	return SOLOMotorControllers::ExeCMD(cmd, error);
}

bool SOLOMotorControllers::SetDeviceAddress(unsigned char deviceAddress)
{
	int error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	return SOLOMotorControllers::SetDeviceAddress(deviceAddress, error);
}

bool SOLOMotorControllers::SetCommandMode(CommandMode mode, int& error)
{
	error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,WriteCommandMode,0x00,0x00,0x00,mode };

	return SOLOMotorControllers::ExeCMD(cmd, error);
}
bool SOLOMotorControllers::SetCommandMode(CommandMode mode)
{
	int error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	return SOLOMotorControllers::SetCommandMode(mode, error);
}
bool SOLOMotorControllers::SetCurrentLimit(float currentLimit, int& error)
{
	error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	if (currentLimit < 0 || currentLimit > 32)
	{
		error = SOLOMotorControllers::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(currentLimit, data);
	unsigned char cmd[] = { addr,WriteCurrentLimit,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllers::ExeCMD(cmd, error);
}
bool SOLOMotorControllers::SetCurrentLimit(float currentLimit)
{
	int error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	return SOLOMotorControllers::SetCurrentLimit(currentLimit, error);
}
bool SOLOMotorControllers::SetTorqueReferenceIq(float torqueReferenceIq, int& error)
{
	error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	if (torqueReferenceIq < 0 || torqueReferenceIq > 32)
	{
		error = SOLOMotorControllers::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(torqueReferenceIq, data);
	unsigned char cmd[] = { addr,WriteTorqueReferenceIq,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllers::ExeCMD(cmd, error);
}
bool SOLOMotorControllers::SetTorqueReferenceIq(float torqueReferenceIq)
{
	int error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	return SOLOMotorControllers::SetTorqueReferenceIq(torqueReferenceIq, error);
}
bool SOLOMotorControllers::SetSpeedReference(long speedReference, int& error)
{
	error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	if (speedReference < 0 || speedReference > 30000)
	{
		error = SOLOMotorControllers::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(speedReference, data);
	unsigned char cmd[] = { addr,WriteSpeedReference,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllers::ExeCMD(cmd, error);
}
bool SOLOMotorControllers::SetSpeedReference(long speedReference)
{
	int error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	return SOLOMotorControllers::SetSpeedReference(speedReference, error);
}
bool SOLOMotorControllers::SetPowerReference(float powerReference, int& error)
{
	error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	if (powerReference < 0 || powerReference > 100)
	{
		error = SOLOMotorControllers::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(powerReference, data);
	unsigned char cmd[] = { addr,WritePowerReference,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllers::ExeCMD(cmd, error);
}
bool SOLOMotorControllers::SetPowerReference(float powerReference)
{
	int error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	return SOLOMotorControllers::SetPowerReference(powerReference, error);
}
bool SOLOMotorControllers::MotorParametersIdentification(Action identification, int& error)
{
	error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,WriteMotorParametersIdentification,0x00,0x00,0x00,identification };

	return SOLOMotorControllers::ExeCMD(cmd, error);
}
bool SOLOMotorControllers::MotorParametersIdentification(Action identification)
{
	int error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	return SOLOMotorControllers::MotorParametersIdentification(identification, error);
}
bool SOLOMotorControllers::EmergencyStop(int& error)
{
	error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,WriteEmergencyStop,0x00,0x00,0x00,0x00 };

	return SOLOMotorControllers::ExeCMD(cmd, error);
}
bool SOLOMotorControllers::EmergencyStop()
{
	int error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	return SOLOMotorControllers::EmergencyStop(error);
}
bool SOLOMotorControllers::SetOutputPwmFrequencyKhz(long outputPwmFrequencyKhz, int& error)
{
	error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	if (outputPwmFrequencyKhz < 8 || outputPwmFrequencyKhz > 80)
	{
		error = SOLOMotorControllers::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(outputPwmFrequencyKhz, data);
	unsigned char cmd[] = { addr,WriteOutputPwmFrequencyKhz,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllers::ExeCMD(cmd, error);

}
bool SOLOMotorControllers::SetOutputPwmFrequencyKhz(long outputPwmFrequencyKhz)
{
	int error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	return SOLOMotorControllers::SetOutputPwmFrequencyKhz(outputPwmFrequencyKhz, error);
}
bool SOLOMotorControllers::SetSpeedControllerKp(float speedControllerKp, int& error)
{
	error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	if (speedControllerKp < 0 || speedControllerKp > 300)
	{
		error = SOLOMotorControllers::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(speedControllerKp, data);
	unsigned char cmd[] = { addr,WriteSpeedControllerKp,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllers::ExeCMD(cmd, error);
}
bool SOLOMotorControllers::SetSpeedControllerKp(float speedControllerKp)
{
	int error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	return SOLOMotorControllers::SetSpeedControllerKp(speedControllerKp, error);
}
bool SOLOMotorControllers::SetSpeedControllerKi(float speedControllerKi, int& error)
{
	error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	if (speedControllerKi < 0 || speedControllerKi > 300)
	{
		error = SOLOMotorControllers::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(speedControllerKi, data);
	unsigned char cmd[] = { addr,WriteSpeedControllerKi,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllers::ExeCMD(cmd, error);
}
bool SOLOMotorControllers::SetSpeedControllerKi(float speedControllerKi)
{
	int error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	return SOLOMotorControllers::SetSpeedControllerKi(speedControllerKi, error);
}
bool SOLOMotorControllers::SetMotorDirection(Direction motorDirection, int& error)
{
	error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,WriteMotorDirection,0x00,0x00,0x00,motorDirection };

	return SOLOMotorControllers::ExeCMD(cmd, error);
}
bool SOLOMotorControllers::SetMotorDirection(Direction motorDirection)
{
	int error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	return SOLOMotorControllers::SetMotorDirection(motorDirection, error);
}
bool SOLOMotorControllers::SetMotorResistance(float motorResistance, int& error)
{
	error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	if (motorResistance < 0.001 || motorResistance > 50)
	{
		error = SOLOMotorControllers::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(motorResistance, data);
	unsigned char cmd[] = { addr,WriteMotorResistance,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllers::ExeCMD(cmd, error);
}
bool SOLOMotorControllers::SetMotorResistance(float motorResistance)
{
	int error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	return SOLOMotorControllers::SetMotorResistance(motorResistance, error);
}
bool SOLOMotorControllers::SetMotorInductance(float motorInductance, int& error)
{
	error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	if (motorInductance < 0.00001 || motorInductance > 0.2)
	{
		error = SOLOMotorControllers::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(motorInductance, data);
	unsigned char cmd[] = { addr,WriteMotorInductance,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllers::ExeCMD(cmd, error);
}
bool SOLOMotorControllers::SetMotorInductance(float motorInductance)
{
	int error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	return SOLOMotorControllers::SetMotorInductance(motorInductance, error);
}
bool SOLOMotorControllers::SetMotorPolesCounts(long motorPolesCounts, int& error)
{
	error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	if (motorPolesCounts < 1 || motorPolesCounts > 80)
	{
		error = SOLOMotorControllers::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(motorPolesCounts, data);
	unsigned char cmd[] = { addr,WriteMotorPolesCounts,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllers::ExeCMD(cmd, error);
}
bool SOLOMotorControllers::SetMotorPolesCounts(long motorPolesCounts)
{
	int error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	return SOLOMotorControllers::SetMotorPolesCounts(motorPolesCounts, error);
}
bool SOLOMotorControllers::SetIncrementalEncoderLines(long incrementalEncoderLines, int& error)
{
	error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	if (incrementalEncoderLines < 1 || incrementalEncoderLines > 40000)
	{
		error = SOLOMotorControllers::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(incrementalEncoderLines, data);
	unsigned char cmd[] = { addr,WriteIncrementalEncoderLines,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllers::ExeCMD(cmd, error);
}
bool SOLOMotorControllers::SetIncrementalEncoderLines(long incrementalEncoderLines)
{
	int error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	return SOLOMotorControllers::SetIncrementalEncoderLines(incrementalEncoderLines, error);
}
bool SOLOMotorControllers::SetSpeedLimit(long speedLimit, int& error)
{
	error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	if (speedLimit < 1 || speedLimit > 30000)
	{
		error = SOLOMotorControllers::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(speedLimit, data);
	unsigned char cmd[] = { addr,WriteSpeedLimit,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllers::ExeCMD(cmd, error);
}
bool SOLOMotorControllers::SetSpeedLimit(long speedLimit)
{
	int error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	return SOLOMotorControllers::SetSpeedLimit(speedLimit, error);
}
bool SOLOMotorControllers::ResetDeviceAddress(int& error)
{
	error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { 0xFF,WriteResetDeviceAddress,0x00,0x00,0x00,0xFF };

	return SOLOMotorControllers::ExeCMD(cmd, error);
}
bool SOLOMotorControllers::ResetDeviceAddress()
{
	int error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	return SOLOMotorControllers::ResetDeviceAddress(error);
}
bool SOLOMotorControllers::SetFeedbackControlMode(FeedbackControlMode mode, int& error)
{
	error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	unsigned char data[4];
	ConvertToData((long)mode, data);
	unsigned char cmd[] = { addr,WriteFeedbackControlMode,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllers::ExeCMD(cmd, error);

}
bool SOLOMotorControllers::SetFeedbackControlMode(FeedbackControlMode mode)
{
	int error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	return SOLOMotorControllers::SetFeedbackControlMode(mode, error);
}
bool SOLOMotorControllers::ResetFactory(int& error)
{
	error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,WriteResetFactory,0x00,0x00,0x00,0x01 };

	return SOLOMotorControllers::ExeCMD(cmd, error);
}
bool SOLOMotorControllers::ResetFactory()
{
	int error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	return SOLOMotorControllers::ResetFactory(error);
}
bool SOLOMotorControllers::SetMotorType(MotorType motorType, int& error)
{
	error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	unsigned char data[4];
	ConvertToData((long)motorType, data);
	unsigned char cmd[] = { addr,WriteMotorType,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllers::ExeCMD(cmd, error);
}
bool SOLOMotorControllers::SetMotorType(MotorType motorType)
{
	int error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	return SOLOMotorControllers::SetMotorType(motorType, error);
}
bool SOLOMotorControllers::SetControlMode(ControlMode controlMode, int& error)
{
	error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	unsigned char data[4];
	ConvertToData((long)controlMode, data);
	unsigned char cmd[] = { addr,WriteControlMode,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllers::ExeCMD(cmd, error);
}
bool SOLOMotorControllers::SetControlMode(ControlMode controlMode)
{
	int error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	return SOLOMotorControllers::SetControlMode(controlMode, error);
}
bool SOLOMotorControllers::SetCurrentControllerKp(float currentControllerKp, int& error)
{
	error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	if (currentControllerKp < 0 || currentControllerKp > 16000)
	{
		error = SOLOMotorControllers::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(currentControllerKp, data);
	unsigned char cmd[] = { addr,WriteCurrentControllerKp,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllers::ExeCMD(cmd, error);
}
bool SOLOMotorControllers::SetCurrentControllerKp(float currentControllerKp)
{
	int error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	return SOLOMotorControllers::SetCurrentControllerKp(currentControllerKp, error);
}
bool SOLOMotorControllers::SetCurrentControllerKi(float currentControllerKi, int& error)
{
	error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	if (currentControllerKi < 0 || currentControllerKi > 16000)
	{
		error = SOLOMotorControllers::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(currentControllerKi, data);
	unsigned char cmd[] = { addr,WriteCurrentControllerKi,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllers::ExeCMD(cmd, error);
}
bool SOLOMotorControllers::SetCurrentControllerKi(float currentControllerKi)
{
	int error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	return SOLOMotorControllers::SetCurrentControllerKi(currentControllerKi, error);
}
bool SOLOMotorControllers::SetMonitoringMode(bool mode, int& error)
{
	error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,WriteMonitoringMode,0x00,0x00,0x00,mode };

	return SOLOMotorControllers::ExeCMD(cmd, error);
}
bool SOLOMotorControllers::SetMonitoringMode(bool mode)
{
	int error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	return SOLOMotorControllers::SetMonitoringMode(mode, error);
}
bool SOLOMotorControllers::SetMagnetizingCurrentIdReference(float magnetizingCurrentIdReference, int& error)
{
	error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	if (magnetizingCurrentIdReference < 0 || magnetizingCurrentIdReference > 32)
	{
		error = SOLOMotorControllers::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(magnetizingCurrentIdReference, data);
	unsigned char cmd[] = { addr,WriteMagnetizingCurrentIdReference,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllers::ExeCMD(cmd, error);
}
bool SOLOMotorControllers::SetMagnetizingCurrentIdReference(float magnetizingCurrentIdReference)
{
	int error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	return SOLOMotorControllers::SetMagnetizingCurrentIdReference(magnetizingCurrentIdReference, error);
}
bool SOLOMotorControllers::SetPositionReference(long positionReference, int& error)
{
	error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	if (positionReference < -2147483647 || positionReference > 2147483647)
	{
		error = SOLOMotorControllers::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(positionReference, data);
	unsigned char cmd[] = { addr,WritePositionReference,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllers::ExeCMD(cmd, error);
}
bool SOLOMotorControllers::SetPositionReference(long positionReference)
{
	int error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	return SOLOMotorControllers::SetPositionReference(positionReference, error);
}
bool SOLOMotorControllers::SetPositionControllerKp(float positionControllerKp, int& error)
{
	error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	if (positionControllerKp < 0 || positionControllerKp > 16000)
	{
		error = SOLOMotorControllers::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(positionControllerKp, data);
	unsigned char cmd[] = { addr,WritePositionControllerKp,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllers::ExeCMD(cmd, error);
}
bool SOLOMotorControllers::SetPositionControllerKp(float positionControllerKp)
{
	int error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	return SOLOMotorControllers::SetPositionControllerKp(positionControllerKp, error);
}
bool SOLOMotorControllers::SetPositionControllerKi(float positionControllerKi, int& error)
{
	error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	if (positionControllerKi < 0 || positionControllerKi > 16000)
	{
		error = SOLOMotorControllers::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(positionControllerKi, data);
	unsigned char cmd[] = { addr,WritePositionControllerKi,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllers::ExeCMD(cmd, error);
}
bool SOLOMotorControllers::SetPositionControllerKi(float positionControllerKi)
{
	int error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	return SOLOMotorControllers::SetPositionControllerKi(positionControllerKi, error);
}
bool SOLOMotorControllers::ResetPositionToZero(int& error)
{
	error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,WriteResetPositionToZero,0x00,0x00,0x00,0x01 };

	return SOLOMotorControllers::ExeCMD(cmd, error);
}
bool SOLOMotorControllers::ResetPositionToZero()
{
	int error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	return SOLOMotorControllers::ResetPositionToZero(error);
}
bool SOLOMotorControllers::OverwriteErrorRegister(int& error)
{
	error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,WriteOverwriteErrorRegister,0x00,0x00,0x00,0x00 };

	return SOLOMotorControllers::ExeCMD(cmd, error);
}
bool SOLOMotorControllers::OverwriteErrorRegister()
{
	int error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	return SOLOMotorControllers::OverwriteErrorRegister(error);
}
// SOG => Sensorless Observer Gain 
bool SOLOMotorControllers::SetObserverGainBldcPmsm(float observerGain, int& error)
{
	error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	if (observerGain < 0.01 || observerGain > 1000)
	{
		error = SOLOMotorControllers::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(observerGain, data);
	unsigned char cmd[] = { addr,WriteObserverGainBldcPmsm,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllers::ExeCMD(cmd, error);
}
bool SOLOMotorControllers::SetObserverGainBldcPmsm(float observerGain)
{
	int error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	return SOLOMotorControllers::SetObserverGainBldcPmsm(observerGain, error);
}
bool SOLOMotorControllers::SetObserverGainBldcPmsmUltrafast(float observerGain, int& error)
{
	error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	if (observerGain < 0.01 || observerGain > 1000)
	{
		error = SOLOMotorControllers::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(observerGain, data);
	unsigned char cmd[] = { addr,WriteObserverGainBldcPmsmUltrafast,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllers::ExeCMD(cmd, error);
}
bool SOLOMotorControllers::SetObserverGainBldcPmsmUltrafast(float observerGain)
{
	int error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	return SOLOMotorControllers::SetObserverGainBldcPmsmUltrafast(observerGain, error);
}
bool SOLOMotorControllers::SetObserverGainDc(float observerGain, int& error)
{
	error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	if (observerGain < 0.01 || observerGain > 1000)
	{
		error = SOLOMotorControllers::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(observerGain, data);
	unsigned char cmd[] = { addr,WriteObserverGainDc,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllers::ExeCMD(cmd, error);
}
bool SOLOMotorControllers::SetObserverGainDc(float observerGain)
{
	int error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	return SOLOMotorControllers::SetObserverGainDc(observerGain, error);
}
// SOFG => Sensorless Observer Filter Gain
bool SOLOMotorControllers::SetFilterGainBldcPmsm(float filterGain, int& error)
{
	error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	if (filterGain < 0.01 || filterGain > 16000)
	{
		error = SOLOMotorControllers::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(filterGain, data);
	unsigned char cmd[] = { addr,WriteFilterGainBldcPmsm,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllers::ExeCMD(cmd, error);
}
bool SOLOMotorControllers::SetFilterGainBldcPmsm(float filterGain)
{
	int error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	return SOLOMotorControllers::SetFilterGainBldcPmsm(filterGain, error);
}
bool SOLOMotorControllers::SetFilterGainBldcPmsmUltrafast(float filterGain, int& error)
{
	error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	if (filterGain < 0.01 || filterGain > 16000)
	{
		error = SOLOMotorControllers::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(filterGain, data);
	unsigned char cmd[] = { addr,WriteFilterGainBldcPmsmUltrafast,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllers::ExeCMD(cmd, error);
}
bool SOLOMotorControllers::SetFilterGainBldcPmsmUltrafast(float filterGain)
{
	int error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	return SOLOMotorControllers::SetFilterGainBldcPmsmUltrafast(filterGain, error);
}
bool SOLOMotorControllers::SetUartBaudrate(UartBaudrate baudrate, int& error)
{
	error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	unsigned char data[4];
	ConvertToData((long)baudrate, data);
	unsigned char cmd[] = { addr,WriteUartBaudrate,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllers::ExeCMD(cmd, error);
}
bool SOLOMotorControllers::SetUartBaudrate(UartBaudrate baudrate)
{
	int error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	return SOLOMotorControllers::SetUartBaudrate(baudrate, error);
}
bool SOLOMotorControllers::SensorCalibration(PositionSensorCalibrationAction calibrationAction, int& error)
{
	error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	unsigned char data[4];
	ConvertToData((long)calibrationAction, data);
	unsigned char cmd[] = { addr, WriteSensorCalibration, data[0], data[1], data[2], data[3] };

	return SOLOMotorControllers::ExeCMD(cmd, error);
}
bool SOLOMotorControllers::SensorCalibration(PositionSensorCalibrationAction calibrationAction)
{
	int error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	return SOLOMotorControllers::SensorCalibration(calibrationAction, error);
}
bool SOLOMotorControllers::SetEncoderHallCcwOffset(float encoderHallOffset, int& error)
{
	error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	if (encoderHallOffset <= 0 || encoderHallOffset >= 1)
	{
		error = SOLOMotorControllers::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(encoderHallOffset, data);
	unsigned char cmd[] = { addr, WriteEncoderHallCcwOffset, data[0], data[1], data[2], data[3] };

	return SOLOMotorControllers::ExeCMD(cmd, error);
}
bool SOLOMotorControllers::SetEncoderHallCcwOffset(float encoderHallOffset)
{
	int error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	return SOLOMotorControllers::SetEncoderHallCcwOffset(encoderHallOffset, error);
}
bool SOLOMotorControllers::SetEncoderHallCwOffset(float encoderHallOffset, int& error)
{
	error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	if (encoderHallOffset <= 0 || encoderHallOffset >= 1)
	{
		error = SOLOMotorControllers::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(encoderHallOffset, data);
	unsigned char cmd[] = { addr, WriteEncoderHallCwOffset, data[0], data[1], data[2], data[3] };

	return SOLOMotorControllers::ExeCMD(cmd, error);
}
bool SOLOMotorControllers::SetEncoderHallCwOffset(float encoderHallOffset)
{
	int error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	return SOLOMotorControllers::SetEncoderHallCwOffset(encoderHallOffset, error);
}
bool SOLOMotorControllers::SetSpeedAccelerationValue(float speedAccelerationValue, int& error)
{
	error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	if (speedAccelerationValue < 0 || speedAccelerationValue > 1600)
	{
		error = SOLOMotorControllers::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(speedAccelerationValue, data);
	unsigned char cmd[] = { addr, WriteSpeedAccelerationValue, data[0], data[1], data[2], data[3] };

	return SOLOMotorControllers::ExeCMD(cmd, error);
}
bool SOLOMotorControllers::SetSpeedAccelerationValue(float speedAccelerationValue)
{
	int error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	return SOLOMotorControllers::SetSpeedAccelerationValue(speedAccelerationValue, error);
}
bool SOLOMotorControllers::SetSpeedDecelerationValue(float speedDecelerationValue, int& error)
{
	error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	if (speedDecelerationValue < 0 || speedDecelerationValue > 1600)
	{
		error = SOLOMotorControllers::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(speedDecelerationValue, data);
	unsigned char cmd[] = { addr, WriteSpeedDecelerationValue, data[0], data[1], data[2], data[3] };

	return SOLOMotorControllers::ExeCMD(cmd, error);
}
bool SOLOMotorControllers::SetSpeedDecelerationValue(float speedDecelerationValue)
{
	int error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	return SOLOMotorControllers::SetSpeedDecelerationValue(speedDecelerationValue, error);
}
bool SOLOMotorControllers::SetCanbusBoudrate(CanbusBaudrate canbusBoudrate, int& error)
{
	error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	unsigned char data[4];
	ConvertToData((long)canbusBoudrate, data);
	unsigned char cmd[] = { addr,WriteUartBaudrate,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllers::ExeCMD(cmd, error);
}
bool SOLOMotorControllers::SetCanbusBoudrate(CanbusBaudrate canbusBoudrate)
{
	int error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	return SOLOMotorControllers::SetCanbusBoudrate(canbusBoudrate, error);
}
//----------Read----------
long SOLOMotorControllers::GetDeviceAddress(int& error)
{
	error = SOLOMotorControllers::SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { 0xFF,ReadDeviceAddress,0x00,0x00,0x00,0x00 };
	//return ReadAddress;
	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToLong(data);
	}
	return -1;
}
long SOLOMotorControllers::GetDeviceAddress()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetDeviceAddress(error);
}
float SOLOMotorControllers::GetPhaseAVoltage(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadPhaseAVoltage,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllers::GetPhaseAVoltage()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetPhaseAVoltage(error);
}
float SOLOMotorControllers::GetPhaseBVoltage(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadPhaseBVoltage,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllers::GetPhaseBVoltage()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetPhaseBVoltage(error);
}
float SOLOMotorControllers::GetPhaseACurrent(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadPhaseACurrent,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllers::GetPhaseACurrent()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetPhaseACurrent(error);
}
float SOLOMotorControllers::GetPhaseBCurrent(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadPhaseBCurrent,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllers::GetPhaseBCurrent()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetPhaseBCurrent(error);
}
//Battery Voltage
float SOLOMotorControllers::GetBusVoltage(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadBusVoltage,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllers::GetBusVoltage()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetBusVoltage(error);
}
float SOLOMotorControllers::GetDcMotorCurrentIm(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadDcMotorCurrentIm,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllers::GetDcMotorCurrentIm()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetDcMotorCurrentIm(error);
}
float SOLOMotorControllers::GetDcMotorVoltageVm(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadDcMotorVoltageVm,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllers::GetDcMotorVoltageVm()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetDcMotorVoltageVm(error);
}
float SOLOMotorControllers::GetSpeedControllerKp(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadSpeedControllerKp,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllers::GetSpeedControllerKp()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetSpeedControllerKp(error);
}
float SOLOMotorControllers::GetSpeedControllerKi(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadSpeedControllerKi,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllers::GetSpeedControllerKi()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetSpeedControllerKi(error);
}
long SOLOMotorControllers::GetOutputPwmFrequencyKhz(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadOutputPwmFrequencyHz,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return (SOLOMotorControllers::ConvertToLong(data) / 1000L); //PWM reading is in Hz
	}
	return -1;
}
long SOLOMotorControllers::GetOutputPwmFrequencyKhz()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetOutputPwmFrequencyKhz(error);
}
float SOLOMotorControllers::GetCurrentLimit(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadCurrentLimit,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllers::GetCurrentLimit()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetCurrentLimit(error);
}
float SOLOMotorControllers::GetQuadratureCurrentIqFeedback(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadQuadratureCurrentIqFeedback,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllers::GetQuadratureCurrentIqFeedback()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetQuadratureCurrentIqFeedback(error);
}
float SOLOMotorControllers::GetMagnetizingCurrentIdFeedback(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadMagnetizingCurrentIdFeedback,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllers::GetMagnetizingCurrentIdFeedback()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetMagnetizingCurrentIdFeedback(error);
}
long SOLOMotorControllers::GetMotorPolesCounts(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadMotorPolesCounts,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToLong(data);
	}
	return -1;
}
long SOLOMotorControllers::GetMotorPolesCounts()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetMotorPolesCounts(error);
}
long SOLOMotorControllers::GetIncrementalEncoderLines(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadIncrementalEncoderLines,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToLong(data);
	}
	return -1;
}
long SOLOMotorControllers::GetIncrementalEncoderLines()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetIncrementalEncoderLines(error);
}
float SOLOMotorControllers::GetCurrentControllerKp(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadCurrentControllerKp,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllers::GetCurrentControllerKp()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetCurrentControllerKp(error);
}
float SOLOMotorControllers::GetCurrentControllerKi(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadCurrentControllerKi,0x00,0x00,0x00,0x00 };
	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToFloat(data) * 0.00005;
	}
	return -1;
}
float SOLOMotorControllers::GetCurrentControllerKi()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetCurrentControllerKi(error);
}
float SOLOMotorControllers::GetBoardTemperature(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadBoardTemperature,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllers::GetBoardTemperature()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetBoardTemperature(error);
}
float SOLOMotorControllers::GetMotorResistance(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadMotorResistance,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllers::GetMotorResistance()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetMotorResistance(error);
}
float SOLOMotorControllers::GetMotorInductance(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadMotorInductance,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllers::GetMotorInductance()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetMotorInductance(error);
}
long SOLOMotorControllers::GetSpeedFeedback(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadSpeedFeedback,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToLong(data);
	}
	return -1;
}
long SOLOMotorControllers::GetSpeedFeedback()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetSpeedFeedback(error);
}
long SOLOMotorControllers::GetMotorType(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadMotorType,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToLong(data);
	}
	return -1;
}
long SOLOMotorControllers::GetMotorType()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetMotorType(error);
}
long SOLOMotorControllers::GetFeedbackControlMode(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadFeedbackControlMode,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToLong(data);
	}
	return -1;
}
long SOLOMotorControllers::GetFeedbackControlMode()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetFeedbackControlMode(error);
}
long SOLOMotorControllers::GetCommandMode(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadCommandMode,0x00,0x00,0x00,0x00 };
	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return  SOLOMotorControllers::ConvertToLong(data);

	}
	return -1;
}
long SOLOMotorControllers::GetCommandMode()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetCommandMode(error);
}
long SOLOMotorControllers::GetControlMode(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadControlMode,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToLong(data);
	}
	return -1;
}
long SOLOMotorControllers::GetControlMode()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetControlMode(error);
}
long SOLOMotorControllers::GetSpeedLimit(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadSpeedLimit,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToLong(data);
	}
	return -1;
}
long SOLOMotorControllers::GetSpeedLimit()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetSpeedLimit(error);
}
float SOLOMotorControllers::GetPositionControllerKp(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadPositionControllerKp,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllers::GetPositionControllerKp()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetPositionControllerKp(error);
}
float SOLOMotorControllers::GetPositionControllerKi(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadPositionControllerKi,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllers::GetPositionControllerKi()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetPositionControllerKi(error);
}
long SOLOMotorControllers::GetPositionCountsFeedback(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadPositionCountsFeedback,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToLong(data);
	}
	return -1;
}
long SOLOMotorControllers::GetPositionCountsFeedback()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetPositionCountsFeedback(error);
}
long SOLOMotorControllers::GetErrorRegister(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadErrorRegister,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToLong(data);
	}
	return -1;
}
long SOLOMotorControllers::GetErrorRegister()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetErrorRegister(error);
}
long SOLOMotorControllers::GetDeviceFirmwareVersion(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadDeviceFirmwareVersion,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToLong(data);
	}
	return -1;
}
long SOLOMotorControllers::GetDeviceFirmwareVersion()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetDeviceFirmwareVersion(error);
}
long SOLOMotorControllers::GetDeviceHardwareVersion(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadDeviceHardwareVersion,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToLong(data);
	}
	return -1;
}
long SOLOMotorControllers::GetDeviceHardwareVersion()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetDeviceHardwareVersion(error);
}
float SOLOMotorControllers::GetTorqueReferenceIq(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadTorqueReferenceIq,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllers::GetTorqueReferenceIq()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetTorqueReferenceIq(error);
}
long SOLOMotorControllers::GetSpeedReference(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadSpeedReference,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToLong(data);
	}
	return -1;
}
long SOLOMotorControllers::GetSpeedReference()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetSpeedReference(error);
}
float SOLOMotorControllers::GetMagnetizingCurrentIdReference(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadMagnetizingCurrentIdReference,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllers::GetMagnetizingCurrentIdReference()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetMagnetizingCurrentIdReference(error);
}
long SOLOMotorControllers::GetPositionReference(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadPositionReference,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToLong(data);
	}
	return -1;
}
long SOLOMotorControllers::GetPositionReference()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetPositionReference(error);
}
float SOLOMotorControllers::GetPowerReference(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadPowerReference,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllers::GetPowerReference()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetPowerReference(error);
}
long SOLOMotorControllers::GetMotorDirection(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadMotorDirection,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToLong(data);
	}
	return -1;
}
long SOLOMotorControllers::GetMotorDirection()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetMotorDirection(error);
}
float SOLOMotorControllers::GetObserverGainBldcPmsm(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadObserverGainBldcPmsm,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllers::GetObserverGainBldcPmsm()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetObserverGainBldcPmsm(error);
}
float SOLOMotorControllers::GetObserverGainBldcPmsmUltrafast(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadObserverGainBldcPmsmUltrafast,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllers::GetObserverGainBldcPmsmUltrafast()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetObserverGainBldcPmsmUltrafast(error);
}
float SOLOMotorControllers::GetObserverGainDc(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadObserverGainDc,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllers::GetObserverGainDc()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetObserverGainDc(error);
}
float SOLOMotorControllers::GetFilterGainBldcPmsm(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadFilterGainBldcPmsm,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllers::GetFilterGainBldcPmsm()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetFilterGainBldcPmsm(error);
}
float SOLOMotorControllers::GetFilterGainBldcPmsmUltrafast(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadFilterGainBldcPmsmUltrafast,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllers::GetFilterGainBldcPmsmUltrafast()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetFilterGainBldcPmsmUltrafast(error);
}
float SOLOMotorControllers::Get3PhaseMotorAngle(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr, Read3PhaseMotorAngle, 0x00, 0x00, 0x00, 0x00 };

	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllers::Get3PhaseMotorAngle()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return Get3PhaseMotorAngle(error);
}
float SOLOMotorControllers::GetEncoderHallCcwOffset(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr, ReadEncoderHallCcwOffset, 0x00, 0x00, 0x00, 0x00 };

	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllers::GetEncoderHallCcwOffset()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetEncoderHallCcwOffset(error);
}
float SOLOMotorControllers::GetEncoderHallCwOffset(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr, ReadEncoderHallCwOffset, 0x00, 0x00, 0x00, 0x00 };

	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllers::GetEncoderHallCwOffset()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetEncoderHallCwOffset(error);
}
long SOLOMotorControllers::GetUartBaudrate(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadUartBaudrate,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToLong(data);
	}
	return -1;
}
long SOLOMotorControllers::GetUartBaudrate()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetUartBaudrate(error);
}
float SOLOMotorControllers::GetSpeedAccelerationValue(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr, ReadSpeedAccelerationValue, 0x00, 0x00, 0x00, 0x00 };

	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllers::GetSpeedAccelerationValue()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetSpeedAccelerationValue(error);
}
float SOLOMotorControllers::GetSpeedDecelerationValue(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr, ReadSpeedDecelerationValue, 0x00, 0x00, 0x00, 0x00 };

	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllers::GetSpeedDecelerationValue()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetSpeedDecelerationValue(error);
}
long SOLOMotorControllers::GetEncoderIndexCounts(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadEncoderIndexCounts,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllers::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllers::SplitData(data, cmd);
		return SOLOMotorControllers::ConvertToLong(data);
	}
	return -1;
}
long SOLOMotorControllers::GetEncoderIndexCounts()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetEncoderIndexCounts(error);
}
bool SOLOMotorControllers::serialIsWorking(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	float temperature = GetBoardTemperature(error);
	if (error == SOLOMotorControllers::SOLOMotorControllersError::noErrorDetected) {
		return true;
	}
	return false;
}
bool SOLOMotorControllers::serialIsWorking()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return serialIsWorking(error);
}