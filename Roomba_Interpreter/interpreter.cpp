#include "interpreter.h"

interpreter::interpreter()
{
    FailSave = new failsave;
    uart = new Uart;
}

interpreter::~interpreter()
{
    FailSave->stopFailsave();
    delete FailSave;
    delete uart;
}

void interpreter::startRoomba()
{
    uart->sendUart(roomba::Start);
    uart->sendUart(roomba::safeMode);
    uart->sendUart(roomba::brushes);
}

void interpreter::drives(speed s)
{
    uart->sendUart(roomba::drive);
    switch (s) {
    case SLOW:
        uart->sendUart(0x10); // Velocity high byte
        uart->sendUart(0x00); // Velocity low  byte
        uart->sendUart(0x80); // Radius high byte
        uart->sendUart(0x00); // Radius low  byte
        break;
    case CRUISE:
        uart->sendUart(0x08); // Velocity high byte
        uart->sendUart(0x00); // Velocity low  byte
        uart->sendUart(0x80); // Radius high byte
        uart->sendUart(0x00); // Radius low  byte
        break;
    case FAST: //2's complement is a bitch
        uart->sendUart(0x0F); // Velocity high byte
        uart->sendUart(0xFF); // Velocity low  byte
        uart->sendUart(0x80); // Radius high byte
        uart->sendUart(0x00); // Radius low  byte
        break;
    }
}

void interpreter::turnRoomba(int a)
{
    uart->sendUart(roomba::Stop);   // stop roomba
    void getAngle();                // reset angle
    uint8_t currentAngle = 0;
    uart->sendUart(roomba::drive);
    if(a < 0) // counter clockwise
    {
        uart->sendUart(0x00); // Velocity high byte
        uart->sendUart(0x00); // Velocity low  byte
        uart->sendUart(0x00); // Radius high byte
        uart->sendUart(0x01); // Radius low  byte

        while(1)
        {
            currentAngle += getAngle();
            if(currentAngle <= a)
            {
                uart->sendUart(roomba::Stop);
                break;
            }
        }
    }
    if(a > 0) // clockwise
    {
        uart->sendUart(0x00); // Velocity high byte
        uart->sendUart(0x00); // Velocity low  byte
        uart->sendUart(0xFF); // Radius high byte
        uart->sendUart(0xFF); // Radius low  byte

        while(1)
        {
            currentAngle += getAngle();
            if(currentAngle >= a)
            {
                uart->sendUart(roomba::Stop);
                break;
            }
        }
    }
}

bool interpreter::getBumpAndWheel()
{
    uart->sendUart(roomba::bumpAndWheel);
    return (uart->receiveUart() ? 1 : 0);
}

uint8_t interpreter::getWall()
{
    uart->sendUart(roomba::wall);
    return uart->receiveUart();
}

uint8_t interpreter::getCliffLeft()
{
    uart->sendUart(roomba::cliffLeft);
    return uart->receiveUart();
}

uint8_t interpreter::getCliffFrontLeft()
{
    uart->sendUart(roomba::cliffFrontLeft);
    return uart->receiveUart();
}

uint8_t interpreter::getCliffFrontRight()
{
    uart->sendUart(roomba::cliffFrontRight);
    return uart->receiveUart();
}

uint8_t interpreter::getCliffRight()
{
    uart->sendUart(roomba::cliffRight);
    return uart->receiveUart();
}

uint8_t interpreter::getVirtualWall()
{
    uart->sendUart(roomba::virtualWall);
    return uart->receiveUart();
}

bool interpreter::getWheelOvercurrents()
{
    uart->sendUart(roomba::wheelOvercurrents);
    return (uart->receiveUart() ? 1 : 0);
}

uint8_t interpreter::getDirtDetect()
{
    uart->sendUart(roomba::dirtDetect);
    return uart->receiveUart();
}

uint8_t interpreter::getIrReceiver()
{
    uart->sendUart(roomba::irReceiver);
    return uart->receiveUart();
}

int16_t interpreter::getDistance()
{
    uart->sendUart(roomba::distance);
    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    return halfWord;
}

int16_t interpreter::getAngle()
{
    uart->sendUart(roomba::angle);

    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    return halfWord;
}

uint8_t interpreter::getChargingState()
{
    uart->sendUart(roomba::chargingState);
    return uart->receiveUart();
}

uint16_t interpreter::getBatteryVoltage()
{
    uart->sendUart(roomba::batteryVoltage);

    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    return halfWord;
}

int16_t interpreter::getBatteryCurrent()
{
   uart->sendUart(roomba::batteryCurrent);

   uint16_t halfWord = 0;
   halfWord = (uart->receiveUart() << 8);
   halfWord |= uart->receiveUart();
   return halfWord;
}

int8_t interpreter::getBatteryTemperature()
{
    uart->sendUart(roomba::batteryTemperature);
    return uart->receiveUart();
}

uint16_t interpreter::getBatteryCharge()
{
    uart->sendUart(roomba::batteryCharge);

    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    return halfWord;
}

uint16_t interpreter::getBatteryCapacity()
{
    uart->sendUart(roomba::batteryCapacity);

    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    return halfWord;
}

uint16_t interpreter::getWallSignal()
{
    uart->sendUart(roomba::wallSignal);

    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    return halfWord;
}

uint16_t interpreter::getCliffLeftSignal()
{
    uart->sendUart(roomba::cliffLeftSignal);

    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    return halfWord;
}

uint16_t interpreter::getCliffFrontLeftSignal()
{
    uart->sendUart(roomba::cliffFrontLeftSignal);

    uint16_t halfWord;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    return halfWord;
}

uint16_t interpreter::getCliffFrontRightSignal()
{
    uart->sendUart(roomba::cliffFrontRightSignal);

    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    return halfWord;
}

uint16_t interpreter::getCliffRightSignal()
{
    uart->sendUart(roomba::cliffRightSignal);

    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    return halfWord;
}

uint8_t interpreter::getChargingSource()
{
    uart->sendUart(roomba::chargingSource);
    return uart->receiveUart();
}

uint8_t interpreter::getOiMode()
{
    uart->sendUart(roomba::oiMode);
    return uart->receiveUart();
}

uint8_t interpreter::getSongNumber()
{
    uart->sendUart(roomba::songNumber);
    return uart->receiveUart();
}

uint8_t interpreter::getSongPlaying()
{
    uart->sendUart(roomba::songPlaying);
    return uart->receiveUart();
}

int16_t interpreter::getRequestedVelocity()
{
    uart->sendUart(roomba::requestedVelocity);

    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    return halfWord;
}

int16_t interpreter::getRequestedRadius()
{
    uart->sendUart(roomba::requestedRadius);

    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    return halfWord;
}

int16_t interpreter::getRequestedRightVelocity()
{
    uart->sendUart(roomba::requestedRightVelocity);

    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    return halfWord;
}

int16_t interpreter::getRequestedLeftVelocity()
{
    uart->sendUart(roomba::requestedLeftVelocity);

    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    return halfWord;
}

uint16_t interpreter::getLeftEncoderCount()
{
    uart->sendUart(roomba::leftEncoderCount);

    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    return halfWord;
}

uint16_t interpreter::getRightEncoderCount()
{
    uart->sendUart(roomba::rightEncoderCount);

    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    return halfWord;
}

uint8_t interpreter::getLightBumper()
{
    uart->sendUart(roomba::lightBumper);
    return (uart->receiveUart() ? 1 : 0);
}

uint16_t interpreter::getLightBumpLeftSignal()
{
    uart->sendUart(roomba::lightBumpLeftSignal);

    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    return halfWord;
}

uint16_t interpreter::getLightBumpFrontLeftSignal()
{
    uart->sendUart(roomba::lightBumpFrontLeftSignal);

    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    return halfWord;
}

uint16_t interpreter::getLightBumpCenterLeftSignal()
{
    uart->sendUart(roomba::lightBumpCenterLeftSignal);

    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    return halfWord;
}

uint16_t interpreter::getLightBumpCenterRightSignal()
{
    uart->sendUart(roomba::lightBumpCenterRightSignal);

    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    return halfWord;
}

uint16_t interpreter::getLightBumpFrontRightSignal()
{
    uart->sendUart(roomba::lightBumpFrontRightSignal);

    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    return halfWord;
}

uint16_t interpreter::getLightBumpRightSignal()
{
    uart->sendUart(roomba::lightBumpRightSignal);

    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    return halfWord;
}

int16_t interpreter::getLeftMotorCurrent()
{
    uart->sendUart(roomba::leftMotorCurrent);

    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    return halfWord;
}

int16_t interpreter::getRightMotorCurrent()
{
    uart->sendUart(roomba::rightMotorCurrent);

    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    return halfWord;
}

int16_t interpreter::getMainBrushMotorCurrent()
{
    uart->sendUart(roomba::mainBrushMotorCurrent);

    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    return halfWord;
}

int16_t interpreter::getSideBrushMotorCurrent()
{
    uart->sendUart(roomba::sideBrushMotorCurrent);

    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    return halfWord;
}

uint8_t interpreter::getStatis()
{
    uart->sendUart(roomba::statis);
    return uart->receiveUart();
}


/***********************************************************/
bool interpreter::getBumpRight()
{
    uart->sendUart(roomba::bumpAndWheel);
    return (uart->receiveUart() & 0b00000001) == 0b00000001 ? 1 : 0;
}

bool interpreter::getBumpLeft()
{
    uart->sendUart(roomba::bumpAndWheel);
    return (uart->receiveUart() & 0b00000010) == 0b00000010 ? 1 : 0;
}

bool interpreter::getWheelDropRight()
{
    uart->sendUart(roomba::bumpAndWheel);
    return (uart->receiveUart() & 0b00000100) == 0b00000100 ? 1 : 0;
}

bool interpreter::getWheelDropLeft()
{
    uart->sendUart(roomba::bumpAndWheel);
    return (uart->receiveUart() & 0b00001000) == 0b00001000 ? 1 : 0;
}

bool interpreter::getSideBrushOvercurrent()
{
    uart->sendUart(roomba::wheelOvercurrents);
    return (uart->receiveUart() & 0b00000001) == 0b00000001 ? 1 : 0;
}

bool interpreter::getMainBrushOvercurrent()
{
    uart->sendUart(roomba::wheelOvercurrents);
    return (uart->receiveUart() & 0b00000100) == 0b00000100 ? 1 : 0;
}

bool interpreter::getRightWheelOvercurrent()
{
    uart->sendUart(roomba::wheelOvercurrents);
    return (uart->receiveUart() & 0b00001000) == 0b00001000 ? 1 : 0;
}

bool interpreter::getLeftWheelOvercurrent()
{
    uart->sendUart(roomba::wheelOvercurrents);
    return (uart->receiveUart() & 0b00010000) == 0b00010000 ? 1 : 0;
}

void interpreter::lockMutex()
{
    sendTex.lock();
}

void interpreter::unlockMutex()
{
    sendTex.unlock();
}
