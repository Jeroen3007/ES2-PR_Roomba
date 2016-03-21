#include "interpreter.h"

interpreter::interpreter()
{
    uart = new Uart;
}

interpreter::~interpreter()
{
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
    sendTex.lock();
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
        uart->sendUart(0xEF); // Velocity high byte
        uart->sendUart(0xFF); // Velocity low  byte
        uart->sendUart(0xFF); // Radius high byte
        uart->sendUart(0xFF); // Radius low  byte
        break;
    }
    sendTex.unlock();
}

void interpreter::turnRoomba(int a)
{
    sendTex.lock();

    uart->sendUart(roomba::Stop);   // stop roomba
    (void) getAngle();                // reset angle
    int16_t currentAngle = 0;
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
            if(currentAngle <= (uint16_t)a)
            {
                uart->sendUart(roomba::Stop);
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
            if(currentAngle >= (uint16_t)a)
            {
                uart->sendUart(roomba::Stop);
            }
        }
    }
    sendTex.unlock();
}

void interpreter::failSave()
{
    sendTex.lock();

    /* check critical sensors and intervene if neccesary*/
    using namespace roomba;

    /* omschrijven naar individuele pollings zonder functie aanroep ivm mutex) */
    if(Interpreter->getBumpAndWheel())uart->sendUart(Stop);
    if(Interpreter->getCliffLeftSignal() > 2048)uart->sendUart(Stop);
    if(Interpreter->getCliffFrontLeftSignal() > 2048)uart->sendUart(Stop);
    if(Interpreter->getCliffFrontRightSignal() > 2048)uart->sendUart(Stop);
    if(Interpreter->getCliffRightSignal() > 2048)uart->sendUart(Stop);
    if(Interpreter->getWheelOvercurrents())uart->sendUart(Stop);
    if((Interpreter->getBatteryCapacity()/Interpreter->getBatteryCharge()*100) < 10)uart->sendUart(Stop);
    sendTex.unlock();
    usleep(500);
}

bool interpreter::getBumpAndWheel()
{
    sendTex.lock();
    uart->sendUart(roomba::bumpAndWheel);
    sendTex.unlock();
    return (uart->receiveUart() ? 1 : 0);
}

uint8_t interpreter::getWall()
{
    sendTex.lock();
    uart->sendUart(roomba::wall);
    uint8_t tmp = uart->receiveUart();
    sendTex.unlock();
    return tmp;
}

uint8_t interpreter::getCliffLeft()
{
    sendTex.lock();
    uart->sendUart(roomba::cliffLeft);
    uint8_t tmp = uart->receiveUart();
    sendTex.unlock();
    return tmp;
}

uint8_t interpreter::getCliffFrontLeft()
{
    sendTex.lock();
    uart->sendUart(roomba::cliffFrontLeft);
    uint8_t tmp = uart->receiveUart();
    sendTex.unlock();
    return tmp;
}

uint8_t interpreter::getCliffFrontRight()
{
    sendTex.lock();
    uart->sendUart(roomba::cliffFrontRight);
    uint8_t tmp = uart->receiveUart();
    sendTex.unlock();
    return tmp;
}

uint8_t interpreter::getCliffRight()
{
    sendTex.lock();
    uart->sendUart(roomba::cliffRight);
    uint8_t tmp = uart->receiveUart();
    sendTex.unlock();
    return tmp;
}

uint8_t interpreter::getVirtualWall()
{
    sendTex.lock();
    uart->sendUart(roomba::virtualWall);
    uint8_t tmp = uart->receiveUart();
    sendTex.unlock();
    return tmp;
}

bool interpreter::getWheelOvercurrents()
{
    sendTex.lock();
    uart->sendUart(roomba::wheelOvercurrents);
    bool tmp = (uart->receiveUart() ? 1 : 0);
    sendTex.unlock();
    return tmp;
}

uint8_t interpreter::getDirtDetect()
{
    sendTex.lock();
    uart->sendUart(roomba::dirtDetect);
    uint8_t tmp = uart->receiveUart();
    sendTex.unlock();
    return tmp;
}

uint8_t interpreter::getIrReceiver()
{
    sendTex.lock();
    uart->sendUart(roomba::irReceiver);
    uint8_t tmp = uart->receiveUart();
    sendTex.unlock();
    return tmp;
}

int16_t interpreter::getDistance()
{
    sendTex.lock();
    uart->sendUart(roomba::distance);
    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    sendTex.unlock();
    return halfWord;
}

int16_t interpreter::getAngle()
{
    sendTex.lock();
    uart->sendUart(roomba::angle);

    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    sendTex.unlock();
    return halfWord;
}

uint8_t interpreter::getChargingState()
{
    sendTex.lock();
    uart->sendUart(roomba::chargingState);
    uint8_t tmp = uart->receiveUart();
    sendTex.unlock();
    return tmp;
}

uint16_t interpreter::getBatteryVoltage()
{
    sendTex.lock();
    uart->sendUart(roomba::batteryVoltage);

    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    sendTex.unlock();
    return halfWord;
}

int16_t interpreter::getBatteryCurrent()
{
    sendTex.lock();
   uart->sendUart(roomba::batteryCurrent);

   uint16_t halfWord = 0;
   halfWord = (uart->receiveUart() << 8);
   halfWord |= uart->receiveUart();
   sendTex.unlock();
   return halfWord;
}

int8_t interpreter::getBatteryTemperature()
{
    sendTex.lock();
    uart->sendUart(roomba::batteryTemperature);
    int8_t tmp = uart->receiveUart();
    sendTex.unlock();
    return tmp;
}

uint16_t interpreter::getBatteryCharge()
{
    sendTex.lock();
    uart->sendUart(roomba::batteryCharge);

    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    sendTex.unlock();
    return halfWord;
}

uint16_t interpreter::getBatteryCapacity()
{
    sendTex.lock();
    uart->sendUart(roomba::batteryCapacity);

    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    sendTex.unlock();
    return halfWord;
}

uint16_t interpreter::getWallSignal()
{
    sendTex.lock();
    uart->sendUart(roomba::wallSignal);

    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    sendTex.unlock();
    return halfWord;
}

uint16_t interpreter::getCliffLeftSignal()
{
    sendTex.lock();
    uart->sendUart(roomba::cliffLeftSignal);

    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    sendTex.unlock();
    return halfWord;
}

uint16_t interpreter::getCliffFrontLeftSignal()
{
    sendTex.lock();
    uart->sendUart(roomba::cliffFrontLeftSignal);

    uint16_t halfWord;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    sendTex.unlock();
    return halfWord;
}

uint16_t interpreter::getCliffFrontRightSignal()
{
    sendTex.lock();
    uart->sendUart(roomba::cliffFrontRightSignal);

    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    sendTex.unlock();
    return halfWord;
}

uint16_t interpreter::getCliffRightSignal()
{
    sendTex.lock();
    uart->sendUart(roomba::cliffRightSignal);

    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    sendTex.unlock();
    return halfWord;
}

uint8_t interpreter::getChargingSource()
{
    sendTex.lock();
    uart->sendUart(roomba::chargingSource);
    uint8_t tmp = uart->receiveUart();
    sendTex.unlock();
    return tmp;
}

uint8_t interpreter::getOiMode()
{
    sendTex.lock();
    uart->sendUart(roomba::oiMode);
    uint8_t tmp = uart->receiveUart();
    sendTex.unlock();
    return tmp;
}

uint8_t interpreter::getSongNumber()
{
    sendTex.lock();
    uart->sendUart(roomba::songNumber);
    uint8_t tmp = uart->receiveUart();
    sendTex.unlock();
    return tmp;
}

uint8_t interpreter::getSongPlaying()
{
    sendTex.lock();
    uart->sendUart(roomba::songPlaying);
    uint8_t tmp = uart->receiveUart();
    sendTex.unlock();
    return tmp;
}

int16_t interpreter::getRequestedVelocity()
{
    sendTex.lock();
    uart->sendUart(roomba::requestedVelocity);

    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    sendTex.unlock();
    return halfWord;
}

int16_t interpreter::getRequestedRadius()
{
    sendTex.lock();
    uart->sendUart(roomba::requestedRadius);

    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    sendTex.unlock();
    return halfWord;
}

int16_t interpreter::getRequestedRightVelocity()
{
    sendTex.lock();
    uart->sendUart(roomba::requestedRightVelocity);

    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    sendTex.unlock();
    return halfWord;
}

int16_t interpreter::getRequestedLeftVelocity()
{
    sendTex.lock();
    uart->sendUart(roomba::requestedLeftVelocity);

    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    sendTex.unlock();
    return halfWord;
}

uint16_t interpreter::getLeftEncoderCount()
{
    sendTex.lock();
    uart->sendUart(roomba::leftEncoderCount);

    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    sendTex.unlock();
    return halfWord;
}

uint16_t interpreter::getRightEncoderCount()
{
    sendTex.lock();
    uart->sendUart(roomba::rightEncoderCount);

    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    sendTex.unlock();
    return halfWord;
}

bool interpreter::getLightBumper()
{
    sendTex.lock();
    uart->sendUart(roomba::lightBumper);
    bool tmp = (uart->receiveUart() ? 1 : 0);
    sendTex.unlock();
    return tmp;
}

uint16_t interpreter::getLightBumpLeftSignal()
{
    sendTex.lock();
    uart->sendUart(roomba::lightBumpLeftSignal);

    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    sendTex.unlock();
    return halfWord;
}

uint16_t interpreter::getLightBumpFrontLeftSignal()
{
    sendTex.lock();
    uart->sendUart(roomba::lightBumpFrontLeftSignal);

    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    sendTex.unlock();
    return halfWord;
}

uint16_t interpreter::getLightBumpCenterLeftSignal()
{
    sendTex.lock();
    uart->sendUart(roomba::lightBumpCenterLeftSignal);

    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    sendTex.unlock();
    return halfWord;
}

uint16_t interpreter::getLightBumpCenterRightSignal()
{
    sendTex.lock();
    uart->sendUart(roomba::lightBumpCenterRightSignal);

    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    sendTex.unlock();
    return halfWord;
}

uint16_t interpreter::getLightBumpFrontRightSignal()
{
    sendTex.lock();
    uart->sendUart(roomba::lightBumpFrontRightSignal);

    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    sendTex.unlock();
    return halfWord;
}

uint16_t interpreter::getLightBumpRightSignal()
{
    sendTex.lock();
    uart->sendUart(roomba::lightBumpRightSignal);

    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    sendTex.unlock();
    return halfWord;
}

int16_t interpreter::getLeftMotorCurrent()
{
    sendTex.lock();
    uart->sendUart(roomba::leftMotorCurrent);

    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    sendTex.unlock();
    return halfWord;
}

int16_t interpreter::getRightMotorCurrent()
{
    sendTex.lock();
    uart->sendUart(roomba::rightMotorCurrent);

    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    sendTex.unlock();
    return halfWord;
}

int16_t interpreter::getMainBrushMotorCurrent()
{
    sendTex.lock();
    uart->sendUart(roomba::mainBrushMotorCurrent);

    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    sendTex.unlock();
    return halfWord;
}

int16_t interpreter::getSideBrushMotorCurrent()
{
    sendTex.lock();
    uart->sendUart(roomba::sideBrushMotorCurrent);

    uint16_t halfWord = 0;
    halfWord = (uart->receiveUart() << 8);
    halfWord |= uart->receiveUart();
    sendTex.unlock();
    return halfWord;
}

uint8_t interpreter::getStatis()
{
    sendTex.lock();
    uart->sendUart(roomba::statis);
    uint8_t tmp = uart->receiveUart();
    sendTex.unlock();
    return tmp;
}


/***********************************************************/
bool interpreter::getBumpRight()
{
    sendTex.lock();
    uart->sendUart(roomba::bumpAndWheel);
    bool tmp = (uart->receiveUart() & 0b00000001) == 0b00000001 ? 1 : 0;
    sendTex.unlock();
    return tmp;
}

bool interpreter::getBumpLeft()
{
    sendTex.lock();
    uart->sendUart(roomba::bumpAndWheel);
    bool tmp = (uart->receiveUart() & 0b00000010) == 0b00000010 ? 1 : 0;
    sendTex.unlock();
    return tmp;
}

bool interpreter::getWheelDropRight()
{
    sendTex.lock();
    uart->sendUart(roomba::bumpAndWheel);
    bool tmp = (uart->receiveUart() & 0b00000100) == 0b00000100 ? 1 : 0;
    sendTex.unlock();
    return tmp;
}

bool interpreter::getWheelDropLeft()
{
    sendTex.lock();
    uart->sendUart(roomba::bumpAndWheel);
    bool tmp = (uart->receiveUart() & 0b00001000) == 0b00001000 ? 1 : 0;
    sendTex.unlock();
    return tmp;
}

bool interpreter::getSideBrushOvercurrent()
{
    sendTex.lock();
    uart->sendUart(roomba::wheelOvercurrents);
    bool tmp = (uart->receiveUart() & 0b00000001) == 0b00000001 ? 1 : 0;
    sendTex.unlock();
    return tmp;
}

bool interpreter::getMainBrushOvercurrent()
{
    sendTex.lock();
    uart->sendUart(roomba::wheelOvercurrents);
    bool tmp = (uart->receiveUart() & 0b00000100) == 0b00000100 ? 1 : 0;
    sendTex.unlock();
    return tmp;
}

bool interpreter::getRightWheelOvercurrent()
{
    sendTex.lock();
    uart->sendUart(roomba::wheelOvercurrents);
    bool tmp = (uart->receiveUart() & 0b00001000) == 0b00001000 ? 1 : 0;
    sendTex.unlock();
    return tmp;
}

bool interpreter::getLeftWheelOvercurrent()
{
    sendTex.lock();
    uart->sendUart(roomba::wheelOvercurrents);
    bool tmp = (uart->receiveUart() & 0b00010000) == 0b00010000 ? 1 : 0;
    sendTex.unlock();
    return tmp;

}

