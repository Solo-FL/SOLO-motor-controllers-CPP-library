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
#include "CommunicationInterface.h"
#include <stdint.h>

class Kvaser
    : public CommunicationInterface
{
public:
	Kvaser(long canBaudrate, bool autoConnect = true);
    ~Kvaser() = default;

	bool CANOpenTransmit(uint8_t _address, uint16_t _object,uint8_t _subIndex, uint8_t* informatrionToSend, int& error) override;
	bool CANOpenReceive(uint8_t _address, uint16_t _object,uint8_t _subIndex, uint8_t* informatrionToSend, uint8_t* _informationReceived, int& error) override;

	bool CANOpenGenericTransmit(uint16_t _ID , uint8_t *_DLC, uint8_t* _Data, int& error) override;
	bool CANOpenGenericReceive(uint16_t *_ID , uint8_t *_DLC, uint8_t* _Data) override;

	bool SendPdoSync(int& error) override;
	bool SendPdoRtr(int _address, int& error) override;
	bool PDOTransmit(int _address, uint8_t* _informatrionToSend, int& error) override;
	bool PDOReceive(int _address,  uint8_t* _informationReceived, int& error) override;

    bool Connect() override
    {
        canStatus stat;
        canInitializeLibrary();
        hnd = canOpenChannel(0, 0);
        if (hnd < 0)
            {
                return false;
            }
        stat = canSetBusParams(hnd, mCanBaudrate, 0, 0, 0, 0, 0);
        if(stat == canOK){
            canBusOn(hnd);
            return true;
        }
        else
            {
                canClose(hnd);
                return false;
            }
    }

    bool Connect(UINT8 deviceAddress, 
             CanbusBaudrate baudrate, long millisecondsTimeout) override
    {
        (void) deviceAddress;
        (void)  millisecondsTimeout;
        switch (baudrate)
            {
            case CanbusBaudrate::rate1000:
                mCanBaudrate = canBITRATE_1M;
                break;
            case CanbusBaudrate::rate500:
                mCanBaudrate = canBITRATE_500K;
                break;
            case CanbusBaudrate::rate250:
                mCanBaudrate = canBITRATE_250K;
                break;
            case CanbusBaudrate::rate125:
                mCanBaudrate = canBITRATE_125K;
                break;
            case CanbusBaudrate::rate100:
                mCanBaudrate = canBITRATE_100K;
                break;
            default:
                mCanBaudrate = canBITRATE_1M;
                break;
            }
        //timeout = millisecondsTimeout; // FIXME: not used!

        return Connect();
    }
    void Disconnect() override
    {
        canClose(hnd);
    }

    void getErrorMode(int& errorMode) override { errorMode = 0; }
    uint8_t getReceiveErrorCounter() override { return 0; };
    uint8_t getTransmitErrorCounter() override { return 0; };

private:
    canHandle hnd;		
	long mCanBaudrate;
};
