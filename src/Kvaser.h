/**
 *******************************************************************************
 * @file    SOLOMotorControllersUtils.h
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

#pragma once

#include "canlib.h"
#include <stdint.h>

class Kvaser
{
public:
	Kvaser();
	bool CANOpenTransmit(int hnd, uint8_t _address, uint16_t _object,uint8_t _subIndex, uint8_t* _informatrionToSend, int& error);
	bool CANOpenReceive(int hnd, uint8_t _address, uint16_t _object,uint8_t _subIndex, uint8_t* _informatrionToSend, uint8_t* _informationReceived, int& error);
	bool CANOpenGenericTransmit(int hnd, uint16_t _ID , uint8_t *_DLC, uint8_t* _Data, int& error);
	bool CANOpenGenericReceive(int hnd, uint16_t *_ID , uint8_t *_DLC, uint8_t* _Data);
	bool SendPdoSync(int hnd, int& error);
	bool SendPdoRtr(int hnd, int _address, int& error);
	bool PDOTransmit(int hnd, int _address, uint8_t* _informatrionToSend, int& error);
	bool PDOReceive(int hnd, int _address,  uint8_t* _informationReceived, int& error);
};