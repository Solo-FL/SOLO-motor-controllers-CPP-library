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

#pragma once

#include "canlib.h"
#include <stdint.h>

class Kvaser
{
public:
	Kvaser();
	bool CANOpenTransmit(int hnd, uint8_t _address, uint16_t _object, uint8_t* _informatrionToSend, int& error);
	bool CANOpenReceive(int hnd, uint8_t _address, uint16_t _object, uint8_t* _informatrionToSend, uint8_t* _informationReceived, int& error);
	bool CANOpenGenericTransmit(int hnd, uint16_t _ID , uint8_t *_DLC, uint8_t* _Data, int& error);
	bool CANOpenGenericReceive(int hnd, uint16_t *_ID , uint8_t *_DLC, uint8_t* _Data);
};
