/**
 *******************************************************************************
 * @file    CommunicationInterface.h
 */

#pragma once

#include <stdint.h>

class CommunicationInterface {

public:
        /**
     * @brief  Canbus Baudrate enumeration definition
     */
    enum CanbusBaudrate
        {
            rate1000 = 0,							/*!< Baudrate 1000 kbits/s */
            rate500 = 1,							/*!< Baudrate 500 kbits/s */
            rate250 = 2,							/*!< Baudrate 250 kbits/s */
            rate125 = 3,							/*!< Baudrate 125 kbits/s */
            rate100 = 4								/*!< Baudrate 100 kbits/s */
        };

    virtual ~CommunicationInterface() = default;
    
    virtual bool CANOpenTransmit(uint8_t destination, uint16_t object, uint8_t subIndex, uint8_t * informatrionToSend, int &error) = 0;
    virtual bool CANOpenReceive (uint8_t source, uint16_t object, uint8_t subIndex, uint8_t* informatrionToSend, uint8_t *informationReceived, int &error) = 0;

    // FIXME: is subIndex needed?
    virtual bool CANOpenGenericTransmit(uint16_t destination, uint8_t *object, uint8_t *informationToSend, int& error) = 0;
    // FIXME: error needed?
    virtual bool CANOpenGenericReceive(uint16_t *source, uint8_t *object, uint8_t *informationReceived) = 0;

    virtual bool SendPdoSync(int& error) = 0;
	virtual bool SendPdoRtr(int destination, int& error) = 0;
	virtual bool PDOTransmit(int destination, uint8_t* informationToSend, int& error) = 0;
	virtual bool PDOReceive(int source,  uint8_t* informationReceived, int& error) = 0;

    virtual bool Connect() = 0;
    virtual bool Connect(UINT8 deviceAddress, 
                         CanbusBaudrate baudrate, long millisecondsTimeout) = 0;
    virtual void Disconnect() = 0;

    virtual void getErrorMode(int& errorMode) = 0;
    virtual uint8_t getReceiveErrorCounter() = 0;
    virtual uint8_t getTransmitErrorCounter() = 0;
};




