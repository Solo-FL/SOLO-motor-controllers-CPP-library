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

//SOLODLLCPP::SOLODLLCPP()
//{
//
//}

SOLODLLCPP::SOLODLLCPP(unsigned char _addr, long _baudrate, long _millisecondsTimeout, int _packetFailureTrialAttempts)
	:addr(_addr)
	, baudrate(_baudrate)
	, millisecondsTimeout(_millisecondsTimeout)
	, packetFailureTrialAttempts(_packetFailureTrialAttempts)
{

}

SOLODLLCPP::~SOLODLLCPP() 
{
	Disconnect();
	isConnected = false;
}

bool SOLODLLCPP::serialSetup(unsigned char _addr, char* _portName, long _baudrate, long _millisecondsTimeout, int _packetFailureTrialAttempts)
{
	addr = _addr;
	sprintf_s(ComPortName, "\\\\.\\%s", _portName);
	//strcpy_s(ComPortName, _portName);
	baudrate = _baudrate;
	millisecondsTimeout = _millisecondsTimeout;
	packetFailureTrialAttempts = _packetFailureTrialAttempts;

	CloseHandle(hComm);
	bool connectionError = true;

	hComm = CreateFile(ComPortName,                       // Name of the Port to be Opened
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

void SOLODLLCPP::Disconnect()
{
	CloseHandle(hComm);
}

bool SOLODLLCPP::Test()
{

	unsigned char cmd[] = { 0xFF,0xFF,65,66,67,68,69,70,0,0xFE };
	int i = 0;
	if (addr == 0x01)
		return true;
	else return false;
}

bool SOLODLLCPP::ExeCMD(unsigned char* cmd, int& error)
{
	//UINT8 _cmd[10] = { INITIATOR, INITIATOR, cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], CRC, ENDING };
	//UINT8 _readPacket[20];
	//UINT8 idx = 0;

	//dNoOFBytestoWrite = sizeof(_cmd);

	//Status = WriteFile(hComm,               // Handle to the Serialport
	//	_cmd,            // Data to be written to the port 
	//	dNoOFBytestoWrite,   // No of bytes to write into the port
	//	&dNoOfBytesWritten,  // No of bytes written to the port
	//	NULL);

	//if (Status != TRUE)
	//	return false;

	///*------------------------------------ Setting Receive Mask ----------------------------------------------*/

	//Status = SetCommMask(hComm, EV_RXCHAR); //Configure Windows to Monitor the serial device for Character Reception

	//if (Status == FALSE)
	//	return false;

	//else //If  WaitCommEvent()==True Read the RXed data using ReadFile();
	//{
	//	do
	//	{
	//		Status = ReadFile(hComm, &TempChar, sizeof(TempChar), &NoBytesRecieved, NULL);
	//		_readPacket[idx] = TempChar;
	//		idx++;
	//	} while (NoBytesRecieved > 0);
	//}

	//if (_readPacket[0] == _cmd[0] && _readPacket[1] == _cmd[1]
	//	&& _readPacket[2] == _cmd[2] && _readPacket[3] == _cmd[3]
	//	&& _readPacket[8] == _cmd[8] && _readPacket[9] == _cmd[9])
	//{
	//	cmd[0] = _readPacket[2];
	//	cmd[1] = _readPacket[3];
	//	cmd[2] = _readPacket[4];
	//	cmd[3] = _readPacket[5];
	//	cmd[4] = _readPacket[6];
	//	cmd[5] = _readPacket[7];
	//}

	//else
	//{
	//	cmd[0] = ERR;
	//	cmd[1] = ERR;
	//	cmd[2] = ERR;
	//	cmd[3] = ERR;
	//	cmd[4] = ERR;
	//	cmd[5] = ERR;
	//}

	//if (cmd[2] == ERR && cmd[3] == ERR && cmd[4] == ERR && cmd[5] == ERR)
	//	return false;
	//else
	//	return true;

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

		if (Status != TRUE)
			continue;

		/*------------------------------------ Setting Receive Mask ----------------------------------------------*/

		Status = SetCommMask(hComm, EV_RXCHAR); //Configure Windows to Monitor the serial device for Character Reception

		if (Status == FALSE)
			continue;

		else //If  WaitCommEvent()==True Read the RXed data using ReadFile();
		{
			do
			{
				Status = ReadFile(hComm, &TempChar, sizeof(TempChar), &NoBytesRecieved, NULL);
				_readPacket[idx] = TempChar;
				idx++;
			} while (NoBytesRecieved > 0);
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
		error = SOLODLLCPP::packetFailureTrialAttemptsOverflow;
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

float SOLODLLCPP::ConvertToFloat(unsigned char* data)
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

void SOLODLLCPP::ConvertToData(float f, unsigned char* data)
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

long SOLODLLCPP::ConvertToLong(unsigned char* data)
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

void SOLODLLCPP::ConvertToData(long l, unsigned char* data)
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

void SOLODLLCPP::SplitData(unsigned char* data, unsigned char* cmd)
{
	data[0] = cmd[2];
	data[1] = cmd[3];
	data[2] = cmd[4];
	data[3] = cmd[5];
}

bool SOLODLLCPP::SetDeviceAddress(unsigned char deviceAddress, int& error)
{
	error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,WriteDeviceAddres,0x00,0x00,0x00,deviceAddress };

	if (deviceAddress < 0 || deviceAddress > 254)
	{
		error = SOLODLLCPP::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	return SOLODLLCPP::ExeCMD(cmd, error);
}

bool SOLODLLCPP::SetDeviceAddress(unsigned char deviceAddress)
{
	int error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	return SOLODLLCPP::SetDeviceAddress(deviceAddress, error);
}

bool SOLODLLCPP::SetCommandMode(CommandMode mode, int& error)
{
	error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,WriteCommandMode,0x00,0x00,0x00,mode };

	return SOLODLLCPP::ExeCMD(cmd, error);
}
bool SOLODLLCPP::SetCommandMode(CommandMode mode)
{
	int error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	return SOLODLLCPP::SetCommandMode(mode, error);
}
bool SOLODLLCPP::SetCurrentLimit(float currentLimit, int& error)
{
	error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	if (currentLimit < 0 || currentLimit > 32)
	{
		error = SOLODLLCPP::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(currentLimit, data);
	unsigned char cmd[] = { addr,WriteCurrentLimit,data[0],data[1],data[2],data[3] };

	return SOLODLLCPP::ExeCMD(cmd, error);
}
bool SOLODLLCPP::SetCurrentLimit(float currentLimit)
{
	int error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	return SOLODLLCPP::SetCurrentLimit(currentLimit, error);
}
bool SOLODLLCPP::SetTorqueReferenceIq(float torqueReferenceIq, int& error)
{
	error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	if (torqueReferenceIq < 0 || torqueReferenceIq > 32)
	{
		error = SOLODLLCPP::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(torqueReferenceIq, data);
	unsigned char cmd[] = { addr,WriteTorqueReferenceIq,data[0],data[1],data[2],data[3] };

	return SOLODLLCPP::ExeCMD(cmd, error);
}
bool SOLODLLCPP::SetTorqueReferenceIq(float torqueReferenceIq)
{
	int error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	return SOLODLLCPP::SetTorqueReferenceIq(torqueReferenceIq, error);
}
bool SOLODLLCPP::SetSpeedReference(long speedReference, int& error)
{
	error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	if (speedReference < 0 || speedReference > 30000)
	{
		error = SOLODLLCPP::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(speedReference, data);
	unsigned char cmd[] = { addr,WriteSpeedReference,data[0],data[1],data[2],data[3] };

	return SOLODLLCPP::ExeCMD(cmd, error);
}
bool SOLODLLCPP::SetSpeedReference(long speedReference)
{
	int error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	return SOLODLLCPP::SetSpeedReference(speedReference, error);
}
bool SOLODLLCPP::SetPowerReference(float powerReference, int& error)
{
	error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	if (powerReference < 0 || powerReference > 100)
	{
		error = SOLODLLCPP::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(powerReference, data);
	unsigned char cmd[] = { addr,WritePowerReference,data[0],data[1],data[2],data[3] };

	return SOLODLLCPP::ExeCMD(cmd, error);
}
bool SOLODLLCPP::SetPowerReference(float powerReference)
{
	int error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	return SOLODLLCPP::SetPowerReference(powerReference, error);
}
bool SOLODLLCPP::MotorParametersIdentification(Action identification, int& error)
{
	error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,WriteMotorParametersIdentification,0x00,0x00,0x00,identification };

	return SOLODLLCPP::ExeCMD(cmd, error);
}
bool SOLODLLCPP::MotorParametersIdentification(Action identification)
{
	int error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	return SOLODLLCPP::MotorParametersIdentification(identification, error);
}
bool SOLODLLCPP::EmergencyStop(int& error)
{
	error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,WriteEmergencyStop,0x00,0x00,0x00,0x00 };

	return SOLODLLCPP::ExeCMD(cmd, error);
}
bool SOLODLLCPP::EmergencyStop()
{
	int error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	return SOLODLLCPP::EmergencyStop(error);
}
bool SOLODLLCPP::SetOutputPwmFrequencyKhz(long outputPwmFrequencyKhz, int& error)
{
	error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	if (outputPwmFrequencyKhz < 8 || outputPwmFrequencyKhz > 80)
	{
		error = SOLODLLCPP::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(outputPwmFrequencyKhz, data);
	unsigned char cmd[] = { addr,WriteOutputPwmFrequencyKhz,data[0],data[1],data[2],data[3] };

	return SOLODLLCPP::ExeCMD(cmd, error);

}
bool SOLODLLCPP::SetOutputPwmFrequencyKhz(long outputPwmFrequencyKhz)
{
	int error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	return SOLODLLCPP::SetOutputPwmFrequencyKhz(outputPwmFrequencyKhz, error);
}
bool SOLODLLCPP::SetSpeedControllerKp(float speedControllerKp, int& error)
{
	error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	if (speedControllerKp < 0 || speedControllerKp > 300)
	{
		error = SOLODLLCPP::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(speedControllerKp, data);
	unsigned char cmd[] = { addr,WriteSpeedControllerKp,data[0],data[1],data[2],data[3] };

	return SOLODLLCPP::ExeCMD(cmd, error);
}
bool SOLODLLCPP::SetSpeedControllerKp(float speedControllerKp)
{
	int error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	return SOLODLLCPP::SetSpeedControllerKp(speedControllerKp, error);
}
bool SOLODLLCPP::SetSpeedControllerKi(float speedControllerKi, int& error)
{
	error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	if (speedControllerKi < 0 || speedControllerKi > 300)
	{
		error = SOLODLLCPP::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(speedControllerKi, data);
	unsigned char cmd[] = { addr,WriteSpeedControllerKi,data[0],data[1],data[2],data[3] };

	return SOLODLLCPP::ExeCMD(cmd, error);
}
bool SOLODLLCPP::SetSpeedControllerKi(float speedControllerKi)
{
	int error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	return SOLODLLCPP::SetSpeedControllerKi(speedControllerKi, error);
}
bool SOLODLLCPP::SetMotorDirection(Direction motorDirection, int& error)
{
	error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,WriteMotorDirection,0x00,0x00,0x00,motorDirection };

	return SOLODLLCPP::ExeCMD(cmd, error);
}
bool SOLODLLCPP::SetMotorDirection(Direction motorDirection)
{
	int error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	return SOLODLLCPP::SetMotorDirection(motorDirection, error);
}
bool SOLODLLCPP::SetMotorResistance(float motorResistance, int& error)
{
	error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	if (motorResistance < 0.001 || motorResistance > 50)
	{
		error = SOLODLLCPP::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(motorResistance, data);
	unsigned char cmd[] = { addr,WriteMotorResistance,data[0],data[1],data[2],data[3] };

	return SOLODLLCPP::ExeCMD(cmd, error);
}
bool SOLODLLCPP::SetMotorResistance(float motorResistance)
{
	int error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	return SOLODLLCPP::SetMotorResistance(motorResistance, error);
}
bool SOLODLLCPP::SetMotorInductance(float motorInductance, int& error)
{
	error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	if (motorInductance < 0.00001 || motorInductance > 0.2)
	{
		error = SOLODLLCPP::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(motorInductance, data);
	unsigned char cmd[] = { addr,WriteMotorInductance,data[0],data[1],data[2],data[3] };

	return SOLODLLCPP::ExeCMD(cmd, error);
}
bool SOLODLLCPP::SetMotorInductance(float motorInductance)
{
	int error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	return SOLODLLCPP::SetMotorInductance(motorInductance, error);
}
bool SOLODLLCPP::SetMotorPolesCounts(long motorPolesCounts, int& error)
{
	error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	if (motorPolesCounts < 1 || motorPolesCounts > 80)
	{
		error = SOLODLLCPP::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(motorPolesCounts, data);
	unsigned char cmd[] = { addr,WriteMotorPolesCounts,data[0],data[1],data[2],data[3] };

	return SOLODLLCPP::ExeCMD(cmd, error);
}
bool SOLODLLCPP::SetMotorPolesCounts(long motorPolesCounts)
{
	int error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	return SOLODLLCPP::SetMotorPolesCounts(motorPolesCounts, error);
}
bool SOLODLLCPP::SetIncrementalEncoderLines(long incrementalEncoderLines, int& error)
{
	error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	if (incrementalEncoderLines < 1 || incrementalEncoderLines > 40000)
	{
		error = SOLODLLCPP::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(incrementalEncoderLines, data);
	unsigned char cmd[] = { addr,WriteIncrementalEncoderLines,data[0],data[1],data[2],data[3] };

	return SOLODLLCPP::ExeCMD(cmd, error);
}
bool SOLODLLCPP::SetIncrementalEncoderLines(long incrementalEncoderLines)
{
	int error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	return SOLODLLCPP::SetIncrementalEncoderLines(incrementalEncoderLines, error);
}
bool SOLODLLCPP::SetSpeedLimit(long speedLimit, int& error)
{
	error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	if (speedLimit < 1 || speedLimit > 30000)
	{
		error = SOLODLLCPP::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(speedLimit, data);
	unsigned char cmd[] = { addr,WriteSpeedLimit,data[0],data[1],data[2],data[3] };

	return SOLODLLCPP::ExeCMD(cmd, error);
}
bool SOLODLLCPP::SetSpeedLimit(long speedLimit)
{
	int error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	return SOLODLLCPP::SetSpeedLimit(speedLimit, error);
}
bool SOLODLLCPP::ResetDeviceAddress(int& error)
{
	error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { 0xFF,WriteResetDeviceAddress,0x00,0x00,0x00,0xFF };

	return SOLODLLCPP::ExeCMD(cmd, error);
}
bool SOLODLLCPP::ResetDeviceAddress()
{
	int error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	return SOLODLLCPP::ResetDeviceAddress(error);
}
bool SOLODLLCPP::SetFeedbackControlMode(FeedbackControlMode mode, int& error)
{
	error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	unsigned char data[4];
	ConvertToData((long)mode, data);
	unsigned char cmd[] = { addr,WriteFeedbackControlMode,data[0],data[1],data[2],data[3] };

	return SOLODLLCPP::ExeCMD(cmd, error);

}
bool SOLODLLCPP::SetFeedbackControlMode(FeedbackControlMode mode)
{
	int error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	return SOLODLLCPP::SetFeedbackControlMode(mode, error);
}
bool SOLODLLCPP::ResetFactory(int& error)
{
	error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,WriteResetFactory,0x00,0x00,0x00,0x01 };

	return SOLODLLCPP::ExeCMD(cmd, error);
}
bool SOLODLLCPP::ResetFactory()
{
	int error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	return SOLODLLCPP::ResetFactory(error);
}
bool SOLODLLCPP::SetMotorType(MotorType motorType, int& error)
{
	error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	unsigned char data[4];
	ConvertToData((long)motorType, data);
	unsigned char cmd[] = { addr,WriteMotorType,data[0],data[1],data[2],data[3] };

	return SOLODLLCPP::ExeCMD(cmd, error);
}
bool SOLODLLCPP::SetMotorType(MotorType motorType)
{
	int error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	return SOLODLLCPP::SetMotorType(motorType, error);
}
bool SOLODLLCPP::SetControlMode(ControlMode controlMode, int& error)
{
	error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	unsigned char data[4];
	ConvertToData((long)controlMode, data);
	unsigned char cmd[] = { addr,WriteControlMode,data[0],data[1],data[2],data[3] };

	return SOLODLLCPP::ExeCMD(cmd, error);
}
bool SOLODLLCPP::SetControlMode(ControlMode controlMode)
{
	int error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	return SOLODLLCPP::SetControlMode(controlMode, error);
}
bool SOLODLLCPP::SetCurrentControllerKp(float currentControllerKp, int& error)
{
	error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	if (currentControllerKp < 0 || currentControllerKp > 16000)
	{
		error = SOLODLLCPP::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(currentControllerKp, data);
	unsigned char cmd[] = { addr,WriteCurrentControllerKp,data[0],data[1],data[2],data[3] };

	return SOLODLLCPP::ExeCMD(cmd, error);
}
bool SOLODLLCPP::SetCurrentControllerKp(float currentControllerKp)
{
	int error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	return SOLODLLCPP::SetCurrentControllerKp(currentControllerKp, error);
}
bool SOLODLLCPP::SetCurrentControllerKi(float currentControllerKi, int& error)
{
	error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	if (currentControllerKi < 0 || currentControllerKi > 16000)
	{
		error = SOLODLLCPP::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(currentControllerKi, data);
	unsigned char cmd[] = { addr,WriteCurrentControllerKi,data[0],data[1],data[2],data[3] };

	return SOLODLLCPP::ExeCMD(cmd, error);
}
bool SOLODLLCPP::SetCurrentControllerKi(float currentControllerKi)
{
	int error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	return SOLODLLCPP::SetCurrentControllerKi(currentControllerKi, error);
}
bool SOLODLLCPP::SetMonitoringMode(bool mode, int& error)
{
	error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,WriteMonitoringMode,0x00,0x00,0x00,mode };

	return SOLODLLCPP::ExeCMD(cmd, error);
}
bool SOLODLLCPP::SetMonitoringMode(bool mode)
{
	int error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	return SOLODLLCPP::SetMonitoringMode(mode, error);
}
bool SOLODLLCPP::SetMagnetizingCurrentIdReference(float magnetizingCurrentIdReference, int& error)
{
	error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	if (magnetizingCurrentIdReference < 0 || magnetizingCurrentIdReference > 32)
	{
		error = SOLODLLCPP::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(magnetizingCurrentIdReference, data);
	unsigned char cmd[] = { addr,WriteMagnetizingCurrentIdReference,data[0],data[1],data[2],data[3] };

	return SOLODLLCPP::ExeCMD(cmd, error);
}
bool SOLODLLCPP::SetMagnetizingCurrentIdReference(float magnetizingCurrentIdReference)
{
	int error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	return SOLODLLCPP::SetMagnetizingCurrentIdReference(magnetizingCurrentIdReference, error);
}
bool SOLODLLCPP::SetPositionReference(long positionReference, int& error)
{
	error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	if (positionReference < -2147483647 || positionReference > 2147483647)
	{
		error = SOLODLLCPP::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(positionReference, data);
	unsigned char cmd[] = { addr,WritePositionReference,data[0],data[1],data[2],data[3] };

	return SOLODLLCPP::ExeCMD(cmd, error);
}
bool SOLODLLCPP::SetPositionReference(long positionReference)
{
	int error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	return SOLODLLCPP::SetPositionReference(positionReference, error);
}
bool SOLODLLCPP::SetPositionControllerKp(float positionControllerKp, int& error)
{
	error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	if (positionControllerKp < 0 || positionControllerKp > 16000)
	{
		error = SOLODLLCPP::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(positionControllerKp, data);
	unsigned char cmd[] = { addr,WritePositionControllerKp,data[0],data[1],data[2],data[3] };

	return SOLODLLCPP::ExeCMD(cmd, error);
}
bool SOLODLLCPP::SetPositionControllerKp(float positionControllerKp)
{
	int error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	return SOLODLLCPP::SetPositionControllerKp(positionControllerKp, error);
}
bool SOLODLLCPP::SetPositionControllerKi(float positionControllerKi, int& error)
{
	error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	if (positionControllerKi < 0 || positionControllerKi > 16000)
	{
		error = SOLODLLCPP::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(positionControllerKi, data);
	unsigned char cmd[] = { addr,WritePositionControllerKi,data[0],data[1],data[2],data[3] };

	return SOLODLLCPP::ExeCMD(cmd, error);
}
bool SOLODLLCPP::SetPositionControllerKi(float positionControllerKi)
{
	int error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	return SOLODLLCPP::SetPositionControllerKi(positionControllerKi, error);
}
bool SOLODLLCPP::ResetPositionToZero(int& error)
{
	error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,WriteResetPositionToZero,0x00,0x00,0x00,0x01 };

	return SOLODLLCPP::ExeCMD(cmd, error);
}
bool SOLODLLCPP::ResetPositionToZero()
{
	int error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	return SOLODLLCPP::ResetPositionToZero(error);
}
bool SOLODLLCPP::OverwriteErrorRegister(int& error)
{
	error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,WriteOverwriteErrorRegister,0x00,0x00,0x00,0x00 };

	return SOLODLLCPP::ExeCMD(cmd, error);
}
bool SOLODLLCPP::OverwriteErrorRegister()
{
	int error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	return SOLODLLCPP::OverwriteErrorRegister(error);
}
// SOG => Sensorless Observer Gain 
bool SOLODLLCPP::SetObserverGainBldcPmsm(float observerGain, int& error)
{
	error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	if (observerGain < 0.01 || observerGain > 1000)
	{
		error = SOLODLLCPP::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(observerGain, data);
	unsigned char cmd[] = { addr,WriteObserverGainBldcPmsm,data[0],data[1],data[2],data[3] };

	return SOLODLLCPP::ExeCMD(cmd, error);
}
bool SOLODLLCPP::SetObserverGainBldcPmsm(float observerGain)
{
	int error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	return SOLODLLCPP::SetObserverGainBldcPmsm(observerGain, error);
}
bool SOLODLLCPP::SetObserverGainBldcPmsmUltrafast(float observerGain, int& error)
{
	error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	if (observerGain < 0.01 || observerGain > 1000)
	{
		error = SOLODLLCPP::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(observerGain, data);
	unsigned char cmd[] = { addr,WriteObserverGainBldcPmsmUltrafast,data[0],data[1],data[2],data[3] };

	return SOLODLLCPP::ExeCMD(cmd, error);
}
bool SOLODLLCPP::SetObserverGainBldcPmsmUltrafast(float observerGain)
{
	int error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	return SOLODLLCPP::SetObserverGainBldcPmsmUltrafast(observerGain, error);
}
bool SOLODLLCPP::SetObserverGainDc(float observerGain, int& error)
{
	error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	if (observerGain < 0.01 || observerGain > 1000)
	{
		error = SOLODLLCPP::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(observerGain, data);
	unsigned char cmd[] = { addr,WriteObserverGainDc,data[0],data[1],data[2],data[3] };

	return SOLODLLCPP::ExeCMD(cmd, error);
}
bool SOLODLLCPP::SetObserverGainDc(float observerGain)
{
	int error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	return SOLODLLCPP::SetObserverGainDc(observerGain, error);
}
// SOFG => Sensorless Observer Filter Gain
bool SOLODLLCPP::SetFilterGainBldcPmsm(float filterGain, int& error)
{
	error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	if (filterGain < 0.01 || filterGain > 16000)
	{
		error = SOLODLLCPP::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(filterGain, data);
	unsigned char cmd[] = { addr,WriteFilterGainBldcPmsm,data[0],data[1],data[2],data[3] };

	return SOLODLLCPP::ExeCMD(cmd, error);
}
bool SOLODLLCPP::SetFilterGainBldcPmsm(float filterGain)
{
	int error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	return SOLODLLCPP::SetFilterGainBldcPmsm(filterGain, error);
}
bool SOLODLLCPP::SetFilterGainBldcPmsmUltrafast(float filterGain, int& error)
{
	error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	if (filterGain < 0.01 || filterGain > 16000)
	{
		error = SOLODLLCPP::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(filterGain, data);
	unsigned char cmd[] = { addr,WriteFilterGainBldcPmsmUltrafast,data[0],data[1],data[2],data[3] };

	return SOLODLLCPP::ExeCMD(cmd, error);
}
bool SOLODLLCPP::SetFilterGainBldcPmsmUltrafast(float filterGain)
{
	int error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	return SOLODLLCPP::SetFilterGainBldcPmsmUltrafast(filterGain, error);
}
bool SOLODLLCPP::SetUartBaudrate(UartBaudrate baudrate, int& error)
{
	error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	unsigned char data[4];
	ConvertToData((long)baudrate, data);
	unsigned char cmd[] = { addr,WriteUartBaudrate,data[0],data[1],data[2],data[3] };

	return SOLODLLCPP::ExeCMD(cmd, error);
}
bool SOLODLLCPP::SetUartBaudrate(UartBaudrate baudrate)
{
	int error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	return SOLODLLCPP::SetUartBaudrate(baudrate, error);
}
bool SOLODLLCPP::SensorCalibration(PositionSensorCalibrationAction calibrationAction, int& error)
{
	error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	unsigned char data[4];
	ConvertToData((long)calibrationAction, data);
	unsigned char cmd[] = { addr, WriteSensorCalibration, data[0], data[1], data[2], data[3] };

	return SOLODLLCPP::ExeCMD(cmd, error);
}
bool SOLODLLCPP::SensorCalibration(PositionSensorCalibrationAction calibrationAction)
{
	int error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	return SOLODLLCPP::SensorCalibration(calibrationAction, error);
}
bool SOLODLLCPP::SetEncoderHallCcwOffset(float encoderHallOffset, int& error)
{
	error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	if (encoderHallOffset <= 0 || encoderHallOffset >= 1)
	{
		error = SOLODLLCPP::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(encoderHallOffset, data);
	unsigned char cmd[] = { addr, WriteEncoderHallCcwOffset, data[0], data[1], data[2], data[3] };

	return SOLODLLCPP::ExeCMD(cmd, error);
}
bool SOLODLLCPP::SetEncoderHallCcwOffset(float encoderHallOffset)
{
	int error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	return SOLODLLCPP::SetEncoderHallCcwOffset(encoderHallOffset, error);
}
bool SOLODLLCPP::SetEncoderHallCwOffset(float encoderHallOffset, int& error)
{
	error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	if (encoderHallOffset <= 0 || encoderHallOffset >= 1)
	{
		error = SOLODLLCPP::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(encoderHallOffset, data);
	unsigned char cmd[] = { addr, WriteEncoderHallCwOffset, data[0], data[1], data[2], data[3] };

	return SOLODLLCPP::ExeCMD(cmd, error);
}
bool SOLODLLCPP::SetEncoderHallCwOffset(float encoderHallOffset)
{
	int error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	return SOLODLLCPP::SetEncoderHallCwOffset(encoderHallOffset, error);
}
bool SOLODLLCPP::SetSpeedAccelerationValue(float speedAccelerationValue, int& error)
{
	error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	if (speedAccelerationValue < 0 || speedAccelerationValue > 1600)
	{
		error = SOLODLLCPP::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(speedAccelerationValue, data);
	unsigned char cmd[] = { addr, WriteSpeedAccelerationValue, data[0], data[1], data[2], data[3] };

	return SOLODLLCPP::ExeCMD(cmd, error);
}
bool SOLODLLCPP::SetSpeedAccelerationValue(float speedAccelerationValue)
{
	int error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	return SOLODLLCPP::SetSpeedAccelerationValue(speedAccelerationValue, error);
}
bool SOLODLLCPP::SetSpeedDecelerationValue(float speedDecelerationValue, int& error)
{
	error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	if (speedDecelerationValue < 0 || speedDecelerationValue > 1600)
	{
		error = SOLODLLCPP::SOLOMotorControllersError::outOfRengeSetting;
		return false;
	}

	unsigned char data[4];
	ConvertToData(speedDecelerationValue, data);
	unsigned char cmd[] = { addr, WriteSpeedDecelerationValue, data[0], data[1], data[2], data[3] };

	return SOLODLLCPP::ExeCMD(cmd, error);
}
bool SOLODLLCPP::SetSpeedDecelerationValue(float speedDecelerationValue)
{
	int error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	return SOLODLLCPP::SetSpeedDecelerationValue(speedDecelerationValue, error);
}
bool SOLODLLCPP::SetCanbusBoudrate(CanbusBaudrate canbusBoudrate, int& error)
{
	error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	unsigned char data[4];
	ConvertToData((long)canbusBoudrate, data);
	unsigned char cmd[] = { addr,WriteUartBaudrate,data[0],data[1],data[2],data[3] };

	return SOLODLLCPP::ExeCMD(cmd, error);
}
bool SOLODLLCPP::SetCanbusBoudrate(CanbusBaudrate canbusBoudrate)
{
	int error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	return SOLODLLCPP::SetCanbusBoudrate(canbusBoudrate, error);
}
//----------Read----------
long SOLODLLCPP::GetDeviceAddress(int& error)
{
	error = SOLODLLCPP::SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { 0xFF,ReadDeviceAddress,0x00,0x00,0x00,0x00 };
	//return ReadAddress;
	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToLong(data);
	}
	return -1;
}
long SOLODLLCPP::GetDeviceAddress()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetDeviceAddress(error);
}
float SOLODLLCPP::GetPhaseAVoltage(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadPhaseAVoltage,0x00,0x00,0x00,0x00 };

	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToFloat(data);
	}
	return -1;
}
float SOLODLLCPP::GetPhaseAVoltage()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetPhaseAVoltage(error);
}
float SOLODLLCPP::GetPhaseBVoltage(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadPhaseBVoltage,0x00,0x00,0x00,0x00 };

	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToFloat(data);
	}
	return -1;
}
float SOLODLLCPP::GetPhaseBVoltage()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetPhaseBVoltage(error);
}
float SOLODLLCPP::GetPhaseACurrent(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadPhaseACurrent,0x00,0x00,0x00,0x00 };

	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToFloat(data);
	}
	return -1;
}
float SOLODLLCPP::GetPhaseACurrent()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetPhaseACurrent(error);
}
float SOLODLLCPP::GetPhaseBCurrent(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadPhaseBCurrent,0x00,0x00,0x00,0x00 };

	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToFloat(data);
	}
	return -1;
}
float SOLODLLCPP::GetPhaseBCurrent()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetPhaseBCurrent(error);
}
//Battery Voltage
float SOLODLLCPP::GetBusVoltage(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadBusVoltage,0x00,0x00,0x00,0x00 };

	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToFloat(data);
	}
	return -1;
}
float SOLODLLCPP::GetBusVoltage()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetBusVoltage(error);
}
float SOLODLLCPP::GetDcMotorCurrentIm(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadDcMotorCurrentIm,0x00,0x00,0x00,0x00 };

	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToFloat(data);
	}
	return -1;
}
float SOLODLLCPP::GetDcMotorCurrentIm()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetDcMotorCurrentIm(error);
}
float SOLODLLCPP::GetDcMotorVoltageVm(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadDcMotorVoltageVm,0x00,0x00,0x00,0x00 };

	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToFloat(data);
	}
	return -1;
}
float SOLODLLCPP::GetDcMotorVoltageVm()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetDcMotorVoltageVm(error);
}
float SOLODLLCPP::GetSpeedControllerKp(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadSpeedControllerKp,0x00,0x00,0x00,0x00 };

	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToFloat(data);
	}
	return -1;
}
float SOLODLLCPP::GetSpeedControllerKp()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetSpeedControllerKp(error);
}
float SOLODLLCPP::GetSpeedControllerKi(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadSpeedControllerKi,0x00,0x00,0x00,0x00 };

	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToFloat(data);
	}
	return -1;
}
float SOLODLLCPP::GetSpeedControllerKi()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetSpeedControllerKi(error);
}
long SOLODLLCPP::GetOutputPwmFrequencyKhz(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadOutputPwmFrequencyHz,0x00,0x00,0x00,0x00 };

	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return (SOLODLLCPP::ConvertToLong(data) / 1000L); //PWM reading is in Hz
	}
	return -1;
}
long SOLODLLCPP::GetOutputPwmFrequencyKhz()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetOutputPwmFrequencyKhz(error);
}
float SOLODLLCPP::GetCurrentLimit(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadCurrentLimit,0x00,0x00,0x00,0x00 };

	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToFloat(data);
	}
	return -1;
}
float SOLODLLCPP::GetCurrentLimit()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetCurrentLimit(error);
}
float SOLODLLCPP::GetQuadratureCurrentIqFeedback(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadQuadratureCurrentIqFeedback,0x00,0x00,0x00,0x00 };

	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToFloat(data);
	}
	return -1;
}
float SOLODLLCPP::GetQuadratureCurrentIqFeedback()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetQuadratureCurrentIqFeedback(error);
}
float SOLODLLCPP::GetMagnetizingCurrentIdFeedback(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadMagnetizingCurrentIdFeedback,0x00,0x00,0x00,0x00 };

	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToFloat(data);
	}
	return -1;
}
float SOLODLLCPP::GetMagnetizingCurrentIdFeedback()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetMagnetizingCurrentIdFeedback(error);
}
long SOLODLLCPP::GetMotorPolesCounts(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadMotorPolesCounts,0x00,0x00,0x00,0x00 };

	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToLong(data);
	}
	return -1;
}
long SOLODLLCPP::GetMotorPolesCounts()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetMotorPolesCounts(error);
}
long SOLODLLCPP::GetIncrementalEncoderLines(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadIncrementalEncoderLines,0x00,0x00,0x00,0x00 };

	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToLong(data);
	}
	return -1;
}
long SOLODLLCPP::GetIncrementalEncoderLines()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetIncrementalEncoderLines(error);
}
float SOLODLLCPP::GetCurrentControllerKp(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadCurrentControllerKp,0x00,0x00,0x00,0x00 };

	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToFloat(data);
	}
	return -1;
}
float SOLODLLCPP::GetCurrentControllerKp()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetCurrentControllerKp(error);
}
float SOLODLLCPP::GetCurrentControllerKi(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadCurrentControllerKi,0x00,0x00,0x00,0x00 };
	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToFloat(data) * 0.00005;
	}
	return -1;
}
float SOLODLLCPP::GetCurrentControllerKi()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetCurrentControllerKi(error);
}
float SOLODLLCPP::GetBoardTemperature(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadBoardTemperature,0x00,0x00,0x00,0x00 };

	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToFloat(data);
	}
	return -1;
}
float SOLODLLCPP::GetBoardTemperature()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetBoardTemperature(error);
}
float SOLODLLCPP::GetMotorResistance(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadMotorResistance,0x00,0x00,0x00,0x00 };

	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToFloat(data);
	}
	return -1;
}
float SOLODLLCPP::GetMotorResistance()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetMotorResistance(error);
}
float SOLODLLCPP::GetMotorInductance(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadMotorInductance,0x00,0x00,0x00,0x00 };

	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToFloat(data);
	}
	return -1;
}
float SOLODLLCPP::GetMotorInductance()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetMotorInductance(error);
}
long SOLODLLCPP::GetSpeedFeedback(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadSpeedFeedback,0x00,0x00,0x00,0x00 };

	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToLong(data);
	}
	return -1;
}
long SOLODLLCPP::GetSpeedFeedback()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetSpeedFeedback(error);
}
long SOLODLLCPP::GetMotorType(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadMotorType,0x00,0x00,0x00,0x00 };

	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToLong(data);
	}
	return -1;
}
long SOLODLLCPP::GetMotorType()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetMotorType(error);
}
long SOLODLLCPP::GetFeedbackControlMode(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadFeedbackControlMode,0x00,0x00,0x00,0x00 };

	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToLong(data);
	}
	return -1;
}
long SOLODLLCPP::GetFeedbackControlMode()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetFeedbackControlMode(error);
}
long SOLODLLCPP::GetCommandMode(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadCommandMode,0x00,0x00,0x00,0x00 };
	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return  SOLODLLCPP::ConvertToLong(data);

	}
	return -1;
}
long SOLODLLCPP::GetCommandMode()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetCommandMode(error);
}
long SOLODLLCPP::GetControlMode(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadControlMode,0x00,0x00,0x00,0x00 };

	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToLong(data);
	}
	return -1;
}
long SOLODLLCPP::GetControlMode()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetControlMode(error);
}
long SOLODLLCPP::GetSpeedLimit(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadSpeedLimit,0x00,0x00,0x00,0x00 };

	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToLong(data);
	}
	return -1;
}
long SOLODLLCPP::GetSpeedLimit()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetSpeedLimit(error);
}
float SOLODLLCPP::GetPositionControllerKp(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadPositionControllerKp,0x00,0x00,0x00,0x00 };

	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToFloat(data);
	}
	return -1;
}
float SOLODLLCPP::GetPositionControllerKp()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetPositionControllerKp(error);
}
float SOLODLLCPP::GetPositionControllerKi(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadPositionControllerKi,0x00,0x00,0x00,0x00 };

	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToFloat(data);
	}
	return -1;
}
float SOLODLLCPP::GetPositionControllerKi()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetPositionControllerKi(error);
}
long SOLODLLCPP::GetPositionCountsFeedback(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadPositionCountsFeedback,0x00,0x00,0x00,0x00 };

	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToLong(data);
	}
	return -1;
}
long SOLODLLCPP::GetPositionCountsFeedback()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetPositionCountsFeedback(error);
}
long SOLODLLCPP::GetErrorRegister(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadErrorRegister,0x00,0x00,0x00,0x00 };

	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToLong(data);
	}
	return -1;
}
long SOLODLLCPP::GetErrorRegister()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetErrorRegister(error);
}
long SOLODLLCPP::GetDeviceFirmwareVersion(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadDeviceFirmwareVersion,0x00,0x00,0x00,0x00 };

	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToLong(data);
	}
	return -1;
}
long SOLODLLCPP::GetDeviceFirmwareVersion()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetDeviceFirmwareVersion(error);
}
long SOLODLLCPP::GetDeviceHardwareVersion(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadDeviceHardwareVersion,0x00,0x00,0x00,0x00 };

	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToLong(data);
	}
	return -1;
}
long SOLODLLCPP::GetDeviceHardwareVersion()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetDeviceHardwareVersion(error);
}
float SOLODLLCPP::GetTorqueReferenceIq(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadTorqueReferenceIq,0x00,0x00,0x00,0x00 };

	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToFloat(data);
	}
	return -1;
}
float SOLODLLCPP::GetTorqueReferenceIq()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetTorqueReferenceIq(error);
}
long SOLODLLCPP::GetSpeedReference(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadSpeedReference,0x00,0x00,0x00,0x00 };

	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToLong(data);
	}
	return -1;
}
long SOLODLLCPP::GetSpeedReference()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetSpeedReference(error);
}
float SOLODLLCPP::GetMagnetizingCurrentIdReference(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadMagnetizingCurrentIdReference,0x00,0x00,0x00,0x00 };

	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToFloat(data);
	}
	return -1;
}
float SOLODLLCPP::GetMagnetizingCurrentIdReference()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetMagnetizingCurrentIdReference(error);
}
long SOLODLLCPP::GetPositionReference(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadPositionReference,0x00,0x00,0x00,0x00 };

	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToLong(data);
	}
	return -1;
}
long SOLODLLCPP::GetPositionReference()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetPositionReference(error);
}
float SOLODLLCPP::GetPowerReference(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadPowerReference,0x00,0x00,0x00,0x00 };

	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToFloat(data);
	}
	return -1;
}
float SOLODLLCPP::GetPowerReference()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetPowerReference(error);
}
long SOLODLLCPP::GetMotorDirection(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadMotorDirection,0x00,0x00,0x00,0x00 };

	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToLong(data);
	}
	return -1;
}
long SOLODLLCPP::GetMotorDirection()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetMotorDirection(error);
}
float SOLODLLCPP::GetObserverGainBldcPmsm(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadObserverGainBldcPmsm,0x00,0x00,0x00,0x00 };

	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToFloat(data);
	}
	return -1;
}
float SOLODLLCPP::GetObserverGainBldcPmsm()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetObserverGainBldcPmsm(error);
}
float SOLODLLCPP::GetObserverGainBldcPmsmUltrafast(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadObserverGainBldcPmsmUltrafast,0x00,0x00,0x00,0x00 };

	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToFloat(data);
	}
	return -1;
}
float SOLODLLCPP::GetObserverGainBldcPmsmUltrafast()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetObserverGainBldcPmsmUltrafast(error);
}
float SOLODLLCPP::GetObserverGainDc(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadObserverGainDc,0x00,0x00,0x00,0x00 };

	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToFloat(data);
	}
	return -1;
}
float SOLODLLCPP::GetObserverGainDc()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetObserverGainDc(error);
}
float SOLODLLCPP::GetFilterGainBldcPmsm(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadFilterGainBldcPmsm,0x00,0x00,0x00,0x00 };

	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToFloat(data);
	}
	return -1;
}
float SOLODLLCPP::GetFilterGainBldcPmsm()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetFilterGainBldcPmsm(error);
}
float SOLODLLCPP::GetFilterGainBldcPmsmUltrafast(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadFilterGainBldcPmsmUltrafast,0x00,0x00,0x00,0x00 };

	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToFloat(data);
	}
	return -1;
}
float SOLODLLCPP::GetFilterGainBldcPmsmUltrafast()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetFilterGainBldcPmsmUltrafast(error);
}
float SOLODLLCPP::Get3PhaseMotorAngle(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr, Read3PhaseMotorAngle, 0x00, 0x00, 0x00, 0x00 };

	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToFloat(data);
	}
	return -1;
}
float SOLODLLCPP::Get3PhaseMotorAngle()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return Get3PhaseMotorAngle(error);
}
float SOLODLLCPP::GetEncoderHallCcwOffset(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr, ReadEncoderHallCcwOffset, 0x00, 0x00, 0x00, 0x00 };

	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToFloat(data);
	}
	return -1;
}
float SOLODLLCPP::GetEncoderHallCcwOffset()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetEncoderHallCcwOffset(error);
}
float SOLODLLCPP::GetEncoderHallCwOffset(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr, ReadEncoderHallCwOffset, 0x00, 0x00, 0x00, 0x00 };

	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToFloat(data);
	}
	return -1;
}
float SOLODLLCPP::GetEncoderHallCwOffset()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetEncoderHallCwOffset(error);
}
long SOLODLLCPP::GetUartBaudrate(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadUartBaudrate,0x00,0x00,0x00,0x00 };

	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToLong(data);
	}
	return -1;
}
long SOLODLLCPP::GetUartBaudrate()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetUartBaudrate(error);
}
float SOLODLLCPP::GetSpeedAccelerationValue(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr, ReadSpeedAccelerationValue, 0x00, 0x00, 0x00, 0x00 };

	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToFloat(data);
	}
	return -1;
}
float SOLODLLCPP::GetSpeedAccelerationValue()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetSpeedAccelerationValue(error);
}
float SOLODLLCPP::GetSpeedDecelerationValue(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr, ReadSpeedDecelerationValue, 0x00, 0x00, 0x00, 0x00 };

	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToFloat(data);
	}
	return -1;
}
float SOLODLLCPP::GetSpeedDecelerationValue()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetSpeedDecelerationValue(error);
}
long SOLODLLCPP::GetEncoderIndexCounts(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	unsigned char cmd[] = { addr,ReadEncoderIndexCounts,0x00,0x00,0x00,0x00 };

	if (SOLODLLCPP::ExeCMD(cmd, error))
	{
		unsigned char data[4];
		SOLODLLCPP::SplitData(data, cmd);
		return SOLODLLCPP::ConvertToLong(data);
	}
	return -1;
}
long SOLODLLCPP::GetEncoderIndexCounts()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return GetEncoderIndexCounts(error);
}
bool SOLODLLCPP::serialIsWorking(int& error)
{
	error = SOLOMotorControllersError::noProcessedCommand;
	float temperature = GetBoardTemperature(error);
	if (error == SOLODLLCPP::SOLOMotorControllersError::noErrorDetected) {
		return true;
	}
	return false;
}
bool SOLODLLCPP::serialIsWorking()
{
	int error = SOLOMotorControllersError::noProcessedCommand;
	return serialIsWorking(error);
}