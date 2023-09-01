/**
 *******************************************************************************
 * @file    CommunicationInterface.h
 */

#pragma once

#include <stdint.h>

class CommunicationInterface {

public:
    virtual ~CommunicationInterface() = default;
    
    virtual bool CANOpenTransmit(uint8_t destination, uint16_t object, uint8_t subIndex, uint8_t * informatrionToSend, int &error) = 0;
    virtual bool CANOpenReceive (uint8_t source, uint16_t object, uint8_t subIndex, uint8_t* informatrionToSend, uint8_t *informationReceived, int &error) = 0;

    // FIXME: is subIndex needed?
    virtual bool CANOpenGenericTransmit(uint16_t destination, uint8_t *object, uint8_t *informationToSend, int& error) = 0;
    // FIXME: error needed?
    virtual bool CANOpenGenericReceive(uint16_t *source, uint8_t *object, uint8_t *informationReceived) = 0;

    virtual bool SendPdoSync(int& error) = 0;
	virtual bool SendPdoRtr(int destination, int& error) = 0;
	virtual bool PDOTransmit(int destination, uint8_t* informatrionToSend, int& error) = 0;
	virtual bool PDOReceive(int source,  uint8_t* informationReceived, int& error) = 0;

    virtual bool Connect() = 0;
    virtual void Disconnect() = 0;
};


class SerialCommunication {

public:
    virtual ~SerialCommunication() = default;
};


class MC2515Communication {

public:
    virtual ~MC2515Communication() = default;
};


