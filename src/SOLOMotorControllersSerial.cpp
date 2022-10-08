// Copyright: (c) 2021-2022, SOLO motor controllers project
// GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

/*
*    Title: SOLO Motor Controllers CPP Library
*    Author: SOLOMotorControllers
*    Date: 2022
*    Code version: 1.0.0
*    Availability: https://github.com/Solo-FL/SOLO-motor-controllers-CPP-library
This Library is made by SOLOMotorControllers.com
To learn more please visit:  https://www.SOLOMotorControllers.com/
*/

#include "SOLOMotorControllersSerial.h"

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
	sprintf_s(ComPortName, "\\\\.\\%s", portName);

	if(isConnected)
		CloseHandle(hComm);

	bool connectionError = true;

	hComm = CreateFile((LPCSTR)ComPortName,        // Name of the Port to be Opened
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

	switch (uartBaudrate)
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

	timeouts.ReadIntervalTimeout = timeout;
	timeouts.ReadTotalTimeoutConstant = timeout;
	timeouts.ReadTotalTimeoutMultiplier = 10;
	timeouts.WriteTotalTimeoutConstant = timeout;
	timeouts.WriteTotalTimeoutMultiplier = 10;

	if (SetCommTimeouts(hComm, &timeouts) == FALSE)
		return false;

	isConnected = true;
	Sleep(50);

	return true;
}

void SOLOMotorControllersSerial::Disconnect()
{
	if(isConnected == true)
	{
		isConnected = false;
		CloseHandle(hComm);
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

	unsigned char _cmd[10] = { INITIATOR, INITIATOR, cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], CRC, ENDING };
	unsigned char _readPacket[20];
	unsigned char idx = 0;

	bool isPacketFailureTrialAttemptsOverflow = true;
	//FailureTrialAttempts block
	for (int attempts = 0; attempts < trialCount; attempts++)
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
		if (Status != TRUE) {
			if (isConnected) {
				//std::cout << "DISCONNECT" << std::endl;
				SOLOMotorControllersSerial::Disconnect();
			}
			else {
				SOLOMotorControllersSerial::Connect();
			}
			continue;
		}

		/*------------------------------------ Setting Receive Mask ----------------------------------------------*/

		Status = SetCommMask(hComm, EV_RXCHAR); //Configure Windows to Monitor the serial device for Character Reception
		//std::cout << "Status: " << Status << std::endl; 


		if (Status == FALSE) {
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

			error = Error::noErrorDetected;
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
			error = Error::generalError;
			return false;
		}
	}
}

float SOLOMotorControllersSerial::ConvertToFloat(unsigned char* data)
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

void SOLOMotorControllersSerial::ConvertToData(float f, unsigned char* data)
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

long SOLOMotorControllersSerial::ConvertToLong(unsigned char* data)
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

void SOLOMotorControllersSerial::ConvertToData(long l, unsigned char* data)
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

void SOLOMotorControllersSerial::SplitData(unsigned char* data, unsigned char* cmd)
{
	data[0] = cmd[2];
	data[1] = cmd[3];
	data[2] = cmd[4];
	data[3] = cmd[5];
}

bool SOLOMotorControllersSerial::SetDeviceAddress(unsigned char deviceAddress, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	unsigned char cmd[] = { addr,WriteDeviceAddres,0x00,0x00,0x00,deviceAddress };

	if (deviceAddress < 0 || deviceAddress > 254)
	{
		error = SOLOMotorControllers::Error::outOfRengeSetting;
		return false;
	}

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}

bool SOLOMotorControllersSerial::SetDeviceAddress(unsigned char deviceAddress)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetDeviceAddress(deviceAddress, error);
}

bool SOLOMotorControllersSerial::SetCommandMode(CommandMode mode, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	unsigned char cmd[] = { addr,WriteCommandMode,0x00,0x00,0x00,mode };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}
bool SOLOMotorControllersSerial::SetCommandMode(CommandMode mode)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetCommandMode(mode, error);
}
bool SOLOMotorControllersSerial::SetCurrentLimit(float currentLimit, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (currentLimit < 0 || currentLimit > 32)
	{
		error = SOLOMotorControllers::Error::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(currentLimit, data);
	unsigned char cmd[] = { addr,WriteCurrentLimit,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}
bool SOLOMotorControllersSerial::SetCurrentLimit(float currentLimit)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetCurrentLimit(currentLimit, error);
}
bool SOLOMotorControllersSerial::SetTorqueReferenceIq(float torqueReferenceIq, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (torqueReferenceIq < 0 || torqueReferenceIq > 32)
	{
		error = SOLOMotorControllers::Error::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(torqueReferenceIq, data);
	unsigned char cmd[] = { addr,WriteTorqueReferenceIq,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}
bool SOLOMotorControllersSerial::SetTorqueReferenceIq(float torqueReferenceIq)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetTorqueReferenceIq(torqueReferenceIq, error);
}
bool SOLOMotorControllersSerial::SetSpeedReference(long speedReference, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (speedReference < 0 || speedReference > 30000)
	{
		error = SOLOMotorControllers::Error::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(speedReference, data);
	unsigned char cmd[] = { addr,WriteSpeedReference,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}
bool SOLOMotorControllersSerial::SetSpeedReference(long speedReference)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetSpeedReference(speedReference, error);
}
bool SOLOMotorControllersSerial::SetPowerReference(float powerReference, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (powerReference < 0 || powerReference > 100)
	{
		error = SOLOMotorControllers::Error::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(powerReference, data);
	unsigned char cmd[] = { addr,WritePowerReference,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}
bool SOLOMotorControllersSerial::SetPowerReference(float powerReference)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetPowerReference(powerReference, error);
}
bool SOLOMotorControllersSerial::MotorParametersIdentification(Action identification, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	unsigned char cmd[] = { addr,WriteMotorParametersIdentification,0x00,0x00,0x00,identification };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}
bool SOLOMotorControllersSerial::MotorParametersIdentification(Action identification)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::MotorParametersIdentification(identification, error);
}
bool SOLOMotorControllersSerial::EmergencyStop(int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	unsigned char cmd[] = { addr,WriteEmergencyStop,0x00,0x00,0x00,0x00 };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}
bool SOLOMotorControllersSerial::EmergencyStop()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::EmergencyStop(error);
}
bool SOLOMotorControllersSerial::SetOutputPwmFrequencyKhz(long outputPwmFrequencyKhz, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (outputPwmFrequencyKhz < 8 || outputPwmFrequencyKhz > 80)
	{
		error = SOLOMotorControllers::Error::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(outputPwmFrequencyKhz, data);
	unsigned char cmd[] = { addr,WriteOutputPwmFrequencyKhz,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);

}
bool SOLOMotorControllersSerial::SetOutputPwmFrequencyKhz(long outputPwmFrequencyKhz)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetOutputPwmFrequencyKhz(outputPwmFrequencyKhz, error);
}
bool SOLOMotorControllersSerial::SetSpeedControllerKp(float speedControllerKp, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (speedControllerKp < 0 || speedControllerKp > 300)
	{
		error = SOLOMotorControllers::Error::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(speedControllerKp, data);
	unsigned char cmd[] = { addr,WriteSpeedControllerKp,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}
bool SOLOMotorControllersSerial::SetSpeedControllerKp(float speedControllerKp)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetSpeedControllerKp(speedControllerKp, error);
}
bool SOLOMotorControllersSerial::SetSpeedControllerKi(float speedControllerKi, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (speedControllerKi < 0 || speedControllerKi > 300)
	{
		error = SOLOMotorControllers::Error::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(speedControllerKi, data);
	unsigned char cmd[] = { addr,WriteSpeedControllerKi,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}
bool SOLOMotorControllersSerial::SetSpeedControllerKi(float speedControllerKi)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetSpeedControllerKi(speedControllerKi, error);
}
bool SOLOMotorControllersSerial::SetMotorDirection(Direction motorDirection, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	unsigned char cmd[] = { addr,WriteMotorDirection,0x00,0x00,0x00,motorDirection };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}
bool SOLOMotorControllersSerial::SetMotorDirection(Direction motorDirection)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetMotorDirection(motorDirection, error);
}
bool SOLOMotorControllersSerial::SetMotorResistance(float motorResistance, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (motorResistance < 0.001 || motorResistance > 50)
	{
		error = SOLOMotorControllers::Error::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(motorResistance, data);
	unsigned char cmd[] = { addr,WriteMotorResistance,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}
bool SOLOMotorControllersSerial::SetMotorResistance(float motorResistance)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetMotorResistance(motorResistance, error);
}
bool SOLOMotorControllersSerial::SetMotorInductance(float motorInductance, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (motorInductance < 0.00001 || motorInductance > 0.2)
	{
		error = SOLOMotorControllers::Error::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(motorInductance, data);
	unsigned char cmd[] = { addr,WriteMotorInductance,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}
bool SOLOMotorControllersSerial::SetMotorInductance(float motorInductance)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetMotorInductance(motorInductance, error);
}
bool SOLOMotorControllersSerial::SetMotorPolesCounts(long motorPolesCounts, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (motorPolesCounts < 1 || motorPolesCounts > 80)
	{
		error = SOLOMotorControllers::Error::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(motorPolesCounts, data);
	unsigned char cmd[] = { addr,WriteMotorPolesCounts,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}
bool SOLOMotorControllersSerial::SetMotorPolesCounts(long motorPolesCounts)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetMotorPolesCounts(motorPolesCounts, error);
}
bool SOLOMotorControllersSerial::SetIncrementalEncoderLines(long incrementalEncoderLines, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (incrementalEncoderLines < 1 || incrementalEncoderLines > 40000)
	{
		error = SOLOMotorControllers::Error::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(incrementalEncoderLines, data);
	unsigned char cmd[] = { addr,WriteIncrementalEncoderLines,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}
bool SOLOMotorControllersSerial::SetIncrementalEncoderLines(long incrementalEncoderLines)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetIncrementalEncoderLines(incrementalEncoderLines, error);
}
bool SOLOMotorControllersSerial::SetSpeedLimit(long speedLimit, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (speedLimit < 1 || speedLimit > 30000)
	{
		error = SOLOMotorControllers::Error::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(speedLimit, data);
	unsigned char cmd[] = { addr,WriteSpeedLimit,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}
bool SOLOMotorControllersSerial::SetSpeedLimit(long speedLimit)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetSpeedLimit(speedLimit, error);
}
bool SOLOMotorControllersSerial::ResetDeviceAddress(int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	unsigned char cmd[] = { 0xFF,WriteResetDeviceAddress,0x00,0x00,0x00,0xFF };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}
bool SOLOMotorControllersSerial::ResetDeviceAddress()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::ResetDeviceAddress(error);
}
bool SOLOMotorControllersSerial::SetFeedbackControlMode(FeedbackControlMode mode, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	unsigned char data[4];
	ConvertToData((long)mode, data);
	unsigned char cmd[] = { addr,WriteFeedbackControlMode,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);

}
bool SOLOMotorControllersSerial::SetFeedbackControlMode(FeedbackControlMode mode)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetFeedbackControlMode(mode, error);
}
bool SOLOMotorControllersSerial::ResetFactory(int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	unsigned char cmd[] = { addr,WriteResetFactory,0x00,0x00,0x00,0x01 };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}
bool SOLOMotorControllersSerial::ResetFactory()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::ResetFactory(error);
}
bool SOLOMotorControllersSerial::SetMotorType(MotorType motorType, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	unsigned char data[4];
	ConvertToData((long)motorType, data);
	unsigned char cmd[] = { addr,WriteMotorType,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}
bool SOLOMotorControllersSerial::SetMotorType(MotorType motorType)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetMotorType(motorType, error);
}
bool SOLOMotorControllersSerial::SetControlMode(ControlMode controlMode, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	unsigned char data[4];
	ConvertToData((long)controlMode, data);
	unsigned char cmd[] = { addr,WriteControlMode,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}
bool SOLOMotorControllersSerial::SetControlMode(ControlMode controlMode)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetControlMode(controlMode, error);
}
bool SOLOMotorControllersSerial::SetCurrentControllerKp(float currentControllerKp, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (currentControllerKp < 0 || currentControllerKp > 16000)
	{
		error = SOLOMotorControllers::Error::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(currentControllerKp, data);
	unsigned char cmd[] = { addr,WriteCurrentControllerKp,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}
bool SOLOMotorControllersSerial::SetCurrentControllerKp(float currentControllerKp)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetCurrentControllerKp(currentControllerKp, error);
}
bool SOLOMotorControllersSerial::SetCurrentControllerKi(float currentControllerKi, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (currentControllerKi < 0 || currentControllerKi > 16000)
	{
		error = SOLOMotorControllers::Error::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(currentControllerKi, data);
	unsigned char cmd[] = { addr,WriteCurrentControllerKi,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}
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
bool SOLOMotorControllersSerial::SetMagnetizingCurrentIdReference(float magnetizingCurrentIdReference, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (magnetizingCurrentIdReference < 0 || magnetizingCurrentIdReference > 32)
	{
		error = SOLOMotorControllers::Error::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(magnetizingCurrentIdReference, data);
	unsigned char cmd[] = { addr,WriteMagnetizingCurrentIdReference,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}
bool SOLOMotorControllersSerial::SetMagnetizingCurrentIdReference(float magnetizingCurrentIdReference)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetMagnetizingCurrentIdReference(magnetizingCurrentIdReference, error);
}
bool SOLOMotorControllersSerial::SetPositionReference(long positionReference, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (positionReference < -2147483647 || positionReference > 2147483647)
	{
		error = SOLOMotorControllers::Error::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(positionReference, data);
	unsigned char cmd[] = { addr,WritePositionReference,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}
bool SOLOMotorControllersSerial::SetPositionReference(long positionReference)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetPositionReference(positionReference, error);
}
bool SOLOMotorControllersSerial::SetPositionControllerKp(float positionControllerKp, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (positionControllerKp < 0 || positionControllerKp > 16000)
	{
		error = SOLOMotorControllers::Error::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(positionControllerKp, data);
	unsigned char cmd[] = { addr,WritePositionControllerKp,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}
bool SOLOMotorControllersSerial::SetPositionControllerKp(float positionControllerKp)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetPositionControllerKp(positionControllerKp, error);
}
bool SOLOMotorControllersSerial::SetPositionControllerKi(float positionControllerKi, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (positionControllerKi < 0 || positionControllerKi > 16000)
	{
		error = SOLOMotorControllers::Error::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(positionControllerKi, data);
	unsigned char cmd[] = { addr,WritePositionControllerKi,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}
bool SOLOMotorControllersSerial::SetPositionControllerKi(float positionControllerKi)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetPositionControllerKi(positionControllerKi, error);
}
bool SOLOMotorControllersSerial::ResetPositionToZero(int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	unsigned char cmd[] = { addr,WriteResetPositionToZero,0x00,0x00,0x00,0x01 };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}
bool SOLOMotorControllersSerial::ResetPositionToZero()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::ResetPositionToZero(error);
}
bool SOLOMotorControllersSerial::OverwriteErrorRegister(int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	unsigned char cmd[] = { addr,WriteOverwriteErrorRegister,0x00,0x00,0x00,0x00 };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}
bool SOLOMotorControllersSerial::OverwriteErrorRegister()
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::OverwriteErrorRegister(error);
}
// SOG => Sensorless Observer Gain 
bool SOLOMotorControllersSerial::SetObserverGainBldcPmsm(float observerGain, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (observerGain < 0.01 || observerGain > 1000)
	{
		error = SOLOMotorControllers::Error::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(observerGain, data);
	unsigned char cmd[] = { addr,WriteObserverGainBldcPmsm,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}
bool SOLOMotorControllersSerial::SetObserverGainBldcPmsm(float observerGain)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetObserverGainBldcPmsm(observerGain, error);
}
bool SOLOMotorControllersSerial::SetObserverGainBldcPmsmUltrafast(float observerGain, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (observerGain < 0.01 || observerGain > 1000)
	{
		error = SOLOMotorControllers::Error::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(observerGain, data);
	unsigned char cmd[] = { addr,WriteObserverGainBldcPmsmUltrafast,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}
bool SOLOMotorControllersSerial::SetObserverGainBldcPmsmUltrafast(float observerGain)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetObserverGainBldcPmsmUltrafast(observerGain, error);
}
bool SOLOMotorControllersSerial::SetObserverGainDc(float observerGain, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (observerGain < 0.01 || observerGain > 1000)
	{
		error = SOLOMotorControllers::Error::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(observerGain, data);
	unsigned char cmd[] = { addr,WriteObserverGainDc,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}
bool SOLOMotorControllersSerial::SetObserverGainDc(float observerGain)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetObserverGainDc(observerGain, error);
}
// SOFG => Sensorless Observer Filter Gain
bool SOLOMotorControllersSerial::SetFilterGainBldcPmsm(float filterGain, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (filterGain < 0.01 || filterGain > 16000)
	{
		error = SOLOMotorControllers::Error::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(filterGain, data);
	unsigned char cmd[] = { addr,WriteFilterGainBldcPmsm,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}
bool SOLOMotorControllersSerial::SetFilterGainBldcPmsm(float filterGain)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetFilterGainBldcPmsm(filterGain, error);
}
bool SOLOMotorControllersSerial::SetFilterGainBldcPmsmUltrafast(float filterGain, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (filterGain < 0.01 || filterGain > 16000)
	{
		error = SOLOMotorControllers::Error::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(filterGain, data);
	unsigned char cmd[] = { addr,WriteFilterGainBldcPmsmUltrafast,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}
bool SOLOMotorControllersSerial::SetFilterGainBldcPmsmUltrafast(float filterGain)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetFilterGainBldcPmsmUltrafast(filterGain, error);
}
bool SOLOMotorControllersSerial::SetUartBaudrate(UartBaudrate baudrate, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	unsigned char data[4];
	ConvertToData((long)baudrate, data);
	unsigned char cmd[] = { addr,WriteUartBaudrate,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}
bool SOLOMotorControllersSerial::SetUartBaudrate(UartBaudrate baudrate)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetUartBaudrate(baudrate, error);
}
bool SOLOMotorControllersSerial::SensorCalibration(PositionSensorCalibrationAction calibrationAction, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	unsigned char data[4];
	ConvertToData((long)calibrationAction, data);
	unsigned char cmd[] = { addr, WriteSensorCalibration, data[0], data[1], data[2], data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}
bool SOLOMotorControllersSerial::SensorCalibration(PositionSensorCalibrationAction calibrationAction)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SensorCalibration(calibrationAction, error);
}
bool SOLOMotorControllersSerial::SetEncoderHallCcwOffset(float encoderHallOffset, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (encoderHallOffset <= 0 || encoderHallOffset >= 1)
	{
		error = SOLOMotorControllers::Error::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(encoderHallOffset, data);
	unsigned char cmd[] = { addr, WriteEncoderHallCcwOffset, data[0], data[1], data[2], data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}
bool SOLOMotorControllersSerial::SetEncoderHallCcwOffset(float encoderHallOffset)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetEncoderHallCcwOffset(encoderHallOffset, error);
}
bool SOLOMotorControllersSerial::SetEncoderHallCwOffset(float encoderHallOffset, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (encoderHallOffset <= 0 || encoderHallOffset >= 1)
	{
		error = SOLOMotorControllers::Error::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(encoderHallOffset, data);
	unsigned char cmd[] = { addr, WriteEncoderHallCwOffset, data[0], data[1], data[2], data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}
bool SOLOMotorControllersSerial::SetEncoderHallCwOffset(float encoderHallOffset)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetEncoderHallCwOffset(encoderHallOffset, error);
}
bool SOLOMotorControllersSerial::SetSpeedAccelerationValue(float speedAccelerationValue, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (speedAccelerationValue < 0 || speedAccelerationValue > 1600)
	{
		error = SOLOMotorControllers::Error::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(speedAccelerationValue, data);
	unsigned char cmd[] = { addr, WriteSpeedAccelerationValue, data[0], data[1], data[2], data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}
bool SOLOMotorControllersSerial::SetSpeedAccelerationValue(float speedAccelerationValue)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetSpeedAccelerationValue(speedAccelerationValue, error);
}
bool SOLOMotorControllersSerial::SetSpeedDecelerationValue(float speedDecelerationValue, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	if (speedDecelerationValue < 0 || speedDecelerationValue > 1600)
	{
		error = SOLOMotorControllers::Error::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(speedDecelerationValue, data);
	unsigned char cmd[] = { addr, WriteSpeedDecelerationValue, data[0], data[1], data[2], data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}
bool SOLOMotorControllersSerial::SetSpeedDecelerationValue(float speedDecelerationValue)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetSpeedDecelerationValue(speedDecelerationValue, error);
}
bool SOLOMotorControllersSerial::SetCanbusBaudrate(CanbusBaudrate canbusBoudrate, int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	unsigned char data[4];
	ConvertToData((long)canbusBoudrate, data);
	unsigned char cmd[] = { addr,WriteUartBaudrate,data[0],data[1],data[2],data[3] };

	return SOLOMotorControllersSerial::ExeCMD(cmd, error);
}
bool SOLOMotorControllersSerial::SetCanbusBaudrate(CanbusBaudrate canbusBoudrate)
{
	int error = SOLOMotorControllers::Error::noProcessedCommand;
	return SOLOMotorControllersSerial::SetCanbusBaudrate(canbusBoudrate, error);
}
////----------Read----------
long SOLOMotorControllersSerial::GetDeviceAddress(int& error)
{
	error = SOLOMotorControllers::Error::noProcessedCommand;
	unsigned char cmd[] = { 0xFF,ReadDeviceAddress,0x00,0x00,0x00,0x00 };
	//return ReadAddress;
	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		soloUtils->SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToLong(data);
	}
	return -1;
}
long SOLOMotorControllersSerial::GetDeviceAddress()
{
	int error = Error::noProcessedCommand;
	return GetDeviceAddress(error);
}
float SOLOMotorControllersSerial::GetPhaseAVoltage(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadPhaseAVoltage,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllersSerial::GetPhaseAVoltage()
{
	int error = Error::noProcessedCommand;
	return GetPhaseAVoltage(error);
}
float SOLOMotorControllersSerial::GetPhaseBVoltage(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadPhaseBVoltage,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllersSerial::GetPhaseBVoltage()
{
	int error = Error::noProcessedCommand;
	return GetPhaseBVoltage(error);
}
float SOLOMotorControllersSerial::GetPhaseACurrent(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadPhaseACurrent,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllersSerial::GetPhaseACurrent()
{
	int error = Error::noProcessedCommand;
	return GetPhaseACurrent(error);
}
float SOLOMotorControllersSerial::GetPhaseBCurrent(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadPhaseBCurrent,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllersSerial::GetPhaseBCurrent()
{
	int error = Error::noProcessedCommand;
	return GetPhaseBCurrent(error);
}
//Battery Voltage
float SOLOMotorControllersSerial::GetBusVoltage(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadBusVoltage,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllersSerial::GetBusVoltage()
{
	int error = Error::noProcessedCommand;
	return GetBusVoltage(error);
}
float SOLOMotorControllersSerial::GetDcMotorCurrentIm(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadDcMotorCurrentIm,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllersSerial::GetDcMotorCurrentIm()
{
	int error = Error::noProcessedCommand;
	return GetDcMotorCurrentIm(error);
}
float SOLOMotorControllersSerial::GetDcMotorVoltageVm(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadDcMotorVoltageVm,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllersSerial::GetDcMotorVoltageVm()
{
	int error = Error::noProcessedCommand;
	return GetDcMotorVoltageVm(error);
}
float SOLOMotorControllersSerial::GetSpeedControllerKp(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadSpeedControllerKp,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllersSerial::GetSpeedControllerKp()
{
	int error = Error::noProcessedCommand;
	return GetSpeedControllerKp(error);
}
float SOLOMotorControllersSerial::GetSpeedControllerKi(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadSpeedControllerKi,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllersSerial::GetSpeedControllerKi()
{
	int error = Error::noProcessedCommand;
	return GetSpeedControllerKi(error);
}
long SOLOMotorControllersSerial::GetOutputPwmFrequencyKhz(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadOutputPwmFrequencyHz,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return (SOLOMotorControllersSerial::ConvertToLong(data) / 1000L); //PWM reading is in Hz
	}
	return -1;
}
long SOLOMotorControllersSerial::GetOutputPwmFrequencyKhz()
{
	int error = Error::noProcessedCommand;
	return GetOutputPwmFrequencyKhz(error);
}
float SOLOMotorControllersSerial::GetCurrentLimit(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadCurrentLimit,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllersSerial::GetCurrentLimit()
{
	int error = Error::noProcessedCommand;
	return GetCurrentLimit(error);
}
float SOLOMotorControllersSerial::GetQuadratureCurrentIqFeedback(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadQuadratureCurrentIqFeedback,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllersSerial::GetQuadratureCurrentIqFeedback()
{
	int error = Error::noProcessedCommand;
	return GetQuadratureCurrentIqFeedback(error);
}
float SOLOMotorControllersSerial::GetMagnetizingCurrentIdFeedback(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadMagnetizingCurrentIdFeedback,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllersSerial::GetMagnetizingCurrentIdFeedback()
{
	int error = Error::noProcessedCommand;
	return GetMagnetizingCurrentIdFeedback(error);
}
long SOLOMotorControllersSerial::GetMotorPolesCounts(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadMotorPolesCounts,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToLong(data);
	}
	return -1;
}
long SOLOMotorControllersSerial::GetMotorPolesCounts()
{
	int error = Error::noProcessedCommand;
	return GetMotorPolesCounts(error);
}
long SOLOMotorControllersSerial::GetIncrementalEncoderLines(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadIncrementalEncoderLines,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToLong(data);
	}
	return -1;
}
long SOLOMotorControllersSerial::GetIncrementalEncoderLines()
{
	int error = Error::noProcessedCommand;
	return GetIncrementalEncoderLines(error);
}
float SOLOMotorControllersSerial::GetCurrentControllerKp(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadCurrentControllerKp,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllersSerial::GetCurrentControllerKp()
{
	int error = Error::noProcessedCommand;
	return GetCurrentControllerKp(error);
}
float SOLOMotorControllersSerial::GetCurrentControllerKi(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadCurrentControllerKi,0x00,0x00,0x00,0x00 };
	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToFloat(data) * 0.00005;
	}
	return -1;
}
float SOLOMotorControllersSerial::GetCurrentControllerKi()
{
	int error = Error::noProcessedCommand;
	return GetCurrentControllerKi(error);
}
float SOLOMotorControllersSerial::GetBoardTemperature(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadBoardTemperature,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllersSerial::GetBoardTemperature()
{
	int error = Error::noProcessedCommand;
	return GetBoardTemperature(error);
}
float SOLOMotorControllersSerial::GetMotorResistance(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadMotorResistance,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllersSerial::GetMotorResistance()
{
	int error = Error::noProcessedCommand;
	return GetMotorResistance(error);
}
float SOLOMotorControllersSerial::GetMotorInductance(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadMotorInductance,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllersSerial::GetMotorInductance()
{
	int error = Error::noProcessedCommand;
	return GetMotorInductance(error);
}
long SOLOMotorControllersSerial::GetSpeedFeedback(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadSpeedFeedback,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToLong(data);
	}
	return -1;
}
long SOLOMotorControllersSerial::GetSpeedFeedback()
{
	int error = Error::noProcessedCommand;
	return GetSpeedFeedback(error);
}
long SOLOMotorControllersSerial::GetMotorType(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadMotorType,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToLong(data);
	}
	return -1;
}
long SOLOMotorControllersSerial::GetMotorType()
{
	int error = Error::noProcessedCommand;
	return GetMotorType(error);
}
long SOLOMotorControllersSerial::GetFeedbackControlMode(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadFeedbackControlMode,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToLong(data);
	}
	return -1;
}
long SOLOMotorControllersSerial::GetFeedbackControlMode()
{
	int error = Error::noProcessedCommand;
	return GetFeedbackControlMode(error);
}
long SOLOMotorControllersSerial::GetCommandMode(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadCommandMode,0x00,0x00,0x00,0x00 };
	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return  SOLOMotorControllersSerial::ConvertToLong(data);

	}
	return -1;
}
long SOLOMotorControllersSerial::GetCommandMode()
{
	int error = Error::noProcessedCommand;
	return GetCommandMode(error);
}
long SOLOMotorControllersSerial::GetControlMode(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadControlMode,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToLong(data);
	}
	return -1;
}
long SOLOMotorControllersSerial::GetControlMode()
{
	int error = Error::noProcessedCommand;
	return GetControlMode(error);
}
long SOLOMotorControllersSerial::GetSpeedLimit(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadSpeedLimit,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToLong(data);
	}
	return -1;
}
long SOLOMotorControllersSerial::GetSpeedLimit()
{
	int error = Error::noProcessedCommand;
	return GetSpeedLimit(error);
}
float SOLOMotorControllersSerial::GetPositionControllerKp(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadPositionControllerKp,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllersSerial::GetPositionControllerKp()
{
	int error = Error::noProcessedCommand;
	return GetPositionControllerKp(error);
}
float SOLOMotorControllersSerial::GetPositionControllerKi(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadPositionControllerKi,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllersSerial::GetPositionControllerKi()
{
	int error = Error::noProcessedCommand;
	return GetPositionControllerKi(error);
}
long SOLOMotorControllersSerial::GetPositionCountsFeedback(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadPositionCountsFeedback,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToLong(data);
	}
	return -1;
}
long SOLOMotorControllersSerial::GetPositionCountsFeedback()
{
	int error = Error::noProcessedCommand;
	return GetPositionCountsFeedback(error);
}
long SOLOMotorControllersSerial::GetErrorRegister(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadErrorRegister,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToLong(data);
	}
	return -1;
}
long SOLOMotorControllersSerial::GetErrorRegister()
{
	int error = Error::noProcessedCommand;
	return GetErrorRegister(error);
}
long SOLOMotorControllersSerial::GetDeviceFirmwareVersion(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadDeviceFirmwareVersion,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToLong(data);
	}
	return -1;
}
long SOLOMotorControllersSerial::GetDeviceFirmwareVersion()
{
	int error = Error::noProcessedCommand;
	return GetDeviceFirmwareVersion(error);
}
long SOLOMotorControllersSerial::GetDeviceHardwareVersion(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadDeviceHardwareVersion,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToLong(data);
	}
	return -1;
}
long SOLOMotorControllersSerial::GetDeviceHardwareVersion()
{
	int error = Error::noProcessedCommand;
	return GetDeviceHardwareVersion(error);
}
float SOLOMotorControllersSerial::GetTorqueReferenceIq(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadTorqueReferenceIq,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllersSerial::GetTorqueReferenceIq()
{
	int error = Error::noProcessedCommand;
	return GetTorqueReferenceIq(error);
}
long SOLOMotorControllersSerial::GetSpeedReference(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadSpeedReference,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToLong(data);
	}
	return -1;
}
long SOLOMotorControllersSerial::GetSpeedReference()
{
	int error = Error::noProcessedCommand;
	return GetSpeedReference(error);
}
float SOLOMotorControllersSerial::GetMagnetizingCurrentIdReference(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadMagnetizingCurrentIdReference,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllersSerial::GetMagnetizingCurrentIdReference()
{
	int error = Error::noProcessedCommand;
	return GetMagnetizingCurrentIdReference(error);
}
long SOLOMotorControllersSerial::GetPositionReference(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadPositionReference,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToLong(data);
	}
	return -1;
}
long SOLOMotorControllersSerial::GetPositionReference()
{
	int error = Error::noProcessedCommand;
	return GetPositionReference(error);
}
float SOLOMotorControllersSerial::GetPowerReference(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadPowerReference,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllersSerial::GetPowerReference()
{
	int error = Error::noProcessedCommand;
	return GetPowerReference(error);
}
long SOLOMotorControllersSerial::GetMotorDirection(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadMotorDirection,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToLong(data);
	}
	return -1;
}
long SOLOMotorControllersSerial::GetMotorDirection()
{
	int error = Error::noProcessedCommand;
	return GetMotorDirection(error);
}
float SOLOMotorControllersSerial::GetObserverGainBldcPmsm(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadObserverGainBldcPmsm,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllersSerial::GetObserverGainBldcPmsm()
{
	int error = Error::noProcessedCommand;
	return GetObserverGainBldcPmsm(error);
}
float SOLOMotorControllersSerial::GetObserverGainBldcPmsmUltrafast(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadObserverGainBldcPmsmUltrafast,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllersSerial::GetObserverGainBldcPmsmUltrafast()
{
	int error = Error::noProcessedCommand;
	return GetObserverGainBldcPmsmUltrafast(error);
}
float SOLOMotorControllersSerial::GetObserverGainDc(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadObserverGainDc,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllersSerial::GetObserverGainDc()
{
	int error = Error::noProcessedCommand;
	return GetObserverGainDc(error);
}
float SOLOMotorControllersSerial::GetFilterGainBldcPmsm(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadFilterGainBldcPmsm,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllersSerial::GetFilterGainBldcPmsm()
{
	int error = Error::noProcessedCommand;
	return GetFilterGainBldcPmsm(error);
}
float SOLOMotorControllersSerial::GetFilterGainBldcPmsmUltrafast(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadFilterGainBldcPmsmUltrafast,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllersSerial::GetFilterGainBldcPmsmUltrafast()
{
	int error = Error::noProcessedCommand;
	return GetFilterGainBldcPmsmUltrafast(error);
}
float SOLOMotorControllersSerial::Get3PhaseMotorAngle(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr, Read3PhaseMotorAngle, 0x00, 0x00, 0x00, 0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllersSerial::Get3PhaseMotorAngle()
{
	int error = Error::noProcessedCommand;
	return Get3PhaseMotorAngle(error);
}
float SOLOMotorControllersSerial::GetEncoderHallCcwOffset(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr, ReadEncoderHallCcwOffset, 0x00, 0x00, 0x00, 0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllersSerial::GetEncoderHallCcwOffset()
{
	int error = Error::noProcessedCommand;
	return GetEncoderHallCcwOffset(error);
}
float SOLOMotorControllersSerial::GetEncoderHallCwOffset(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr, ReadEncoderHallCwOffset, 0x00, 0x00, 0x00, 0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllersSerial::GetEncoderHallCwOffset()
{
	int error = Error::noProcessedCommand;
	return GetEncoderHallCwOffset(error);
}
long SOLOMotorControllersSerial::GetUartBaudrate(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadUartBaudrate,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToLong(data);
	}
	return -1;
}
long SOLOMotorControllersSerial::GetUartBaudrate()
{
	int error = Error::noProcessedCommand;
	return GetUartBaudrate(error);
}
float SOLOMotorControllersSerial::GetSpeedAccelerationValue(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr, ReadSpeedAccelerationValue, 0x00, 0x00, 0x00, 0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllersSerial::GetSpeedAccelerationValue()
{
	int error = Error::noProcessedCommand;
	return GetSpeedAccelerationValue(error);
}
float SOLOMotorControllersSerial::GetSpeedDecelerationValue(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr, ReadSpeedDecelerationValue, 0x00, 0x00, 0x00, 0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToFloat(data);
	}
	return -1;
}
float SOLOMotorControllersSerial::GetSpeedDecelerationValue()
{
	int error = Error::noProcessedCommand;
	return GetSpeedDecelerationValue(error);
}
long SOLOMotorControllersSerial::GetEncoderIndexCounts(int& error)
{
	error = Error::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadEncoderIndexCounts,0x00,0x00,0x00,0x00 };

	if (SOLOMotorControllersSerial::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLOMotorControllersSerial::SplitData(data, cmd);
		return SOLOMotorControllersSerial::ConvertToLong(data);
	}
	return -1;
}
long SOLOMotorControllersSerial::GetEncoderIndexCounts()
{
	int error = Error::noProcessedCommand;
	return GetEncoderIndexCounts(error);
}
bool SOLOMotorControllersSerial::CommunicationIsWorking(int& error)
{
	error = Error::noProcessedCommand;
	float temperature = GetBoardTemperature(error);
	if (error == SOLOMotorControllers::Error::noErrorDetected) {
		return true;
	}
	return false;
}
bool SOLOMotorControllersSerial::CommunicationIsWorking()
{
	int error = Error::noProcessedCommand;
	return CommunicationIsWorking(error);
}