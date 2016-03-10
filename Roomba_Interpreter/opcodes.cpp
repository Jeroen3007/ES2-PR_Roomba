#include "opcodes.h"


opcodes::opcodes()
{
    for(unsigned int i = 0; i < 58 ; ++i)
    {
        sensorWaarden[i]=0;
    }
    sensorWaarden[wheelOvercurrents] = 12;
    FailSave = new FailSave(&tex);
}

void opcodes::print()
{
    while(1)
    {
        for(unsigned int i = 0; i < 58 ; ++i)
        {

            std::cout<<"Senson no"<<i<<": "<< sensorWaarden[i]<<std::endl;
        }
    }

}

void opcodes::startRoomba()
{
    sentUart(Start);
    sentUart(safeMode);
    sentUart(brushes);
}

void opcodes::drives(speed s)
{
    sentUart(drive);
    switch (s) {
    case SLOW:
        sentUart(0x10); // Velocity high byte
        sentUart(0x00); // Velocity low  byte
        sentUart(0x80); // Radius high byte
        sentUart(0x00); // Radius low  byte
        break;
    case CRUISE:
        sentUart(0x08); // Velocity high byte
        sentUart(0x00); // Velocity low  byte
        sentUart(0x80); // Radius high byte
        sentUart(0x00); // Radius low  byte
        break;
    case FAST: //2's complement is a bitch
        sentUart(0x0F); // Velocity high byte
        sentUart(0xFF); // Velocity low  byte
        sentUart(0x80); // Radius high byte
        sentUart(0x00); // Radius low  byte
        break;
    }
}

void opcodes::turnRoomba(angles a)
{
    uint8_t currentAngle = getAngle();
    sentUart(drive);
    switch (a) {
    case RIGHT:
        sentUart(0x00); // Velocity high byte
        sentUart(0x00); // Velocity low  byte
        sentUart(0xFF); // Radius high byte
        sentUart(0xFF); // Radius low  byte

        while(1)
        {
            currentAngle += getAngle();
            if(currentAngle == RIGHT)
            {
                sentUart(Stop);
                break;
            }
        }

        break;
    case LEFT:
        sentUart(0x00); // Velocity high byte
        sentUart(0x00); // Velocity low  byte
        sentUart(0x00); // Radius high byte
        sentUart(0x01); // Radius low  byte

        while(1)
        {
            currentAngle += getAngle();
            if(currentAngle == LEFT)
            {
                sentUart(Stop);
                break;
            }
        }

        break;
    }


}

void opcodes::receiveUart()
{
    while(blablaUart() != 19){}

    uint8_t nPackets = blablaUart();
    uint8_t checksum = 0;
    for(unsigned int i = nPackets ; i > 0 ; --i)
    {
        uint8_t code = blablaUart();
        checksum += code;
        std::vector<uint8_t> databytes;
        uint8_t data = 0;
        for(unsigned int i = howManyDatabytes(code) ; i > 0 ; --i)
        {
            uint8_t databyte = blablaUart();
            checksum += databyte;
            databytes.push_back(databyte);
        }
        checksum += blablaUart();
        if(checksum != 0)std::cout<<"error"<<std::endl;
        for(unsigned int i = 0; i < databytes.size(); i++)
        {
            data += databytes[i];
        }
        sensorWaarden[code] = data;
    }
}

bool opcodes::getBumpAndWheel()
{
    /*if(sensorWaarden[bumpAndWheel] != 0) return 1;
    else return 0;*/
    return (sensorWaarden[bumpAndWheel] ? 1 : 0);
}

uint8_t opcodes::getWall()
{
    return sensorWaarden[wall];
}

uint8_t opcodes::getCliffLeft()
{
    return sensorWaarden[cliffLeft];
}

uint8_t opcodes::getCliffFrontLeft()
{
    return sensorWaarden[cliffFrontLeft];
}

uint8_t opcodes::getCliffFrontRight()
{
    return sensorWaarden[cliffFrontRight];
}

uint8_t opcodes::getCliffRight()
{
    return sensorWaarden[cliffRight];
}

uint8_t opcodes::getVirtualWall()
{
    return sensorWaarden[virtualWall];
}

bool opcodes::getWheelOvercurrents()
{
    return (sensorWaarden[wheelOvercurrents] ? 1 : 0);
}

uint8_t opcodes::getDirtDetect()
{
    return sensorWaarden[dirtDetect];
}

uint8_t opcodes::getUnusedByte()
{
    return sensorWaarden[unusedByte];
}

uint8_t opcodes::getIrReceiver()
{
    return sensorWaarden[irReceiver];
}

int16_t opcodes::getDistance()
{
    return (int16_t)sensorWaarden[distance];
}

int16_t opcodes::getAngle()
{
    return (int16_t)sensorWaarden[angle];
}

uint8_t opcodes::getChargingState()
{
    return sensorWaarden[chargingState];
}

uint16_t opcodes::getBatteryVoltage()
{
    return sensorWaarden[batteryVoltage];
}

int16_t opcodes::getBatteryCurrent()
{
    return (int16_t)sensorWaarden[batteryCurrent];
}

int8_t opcodes::getBatteryTemperature()
{
    return (int8_t)sensorWaarden[batteryTemperature];
}

uint16_t opcodes::getBatteryCharge()
{
    return sensorWaarden[batteryCharge];
}

uint16_t opcodes::getBatteryCapacity()
{
    return sensorWaarden[batteryCapacity];
}

uint16_t opcodes::getWallSignal()
{
    return sensorWaarden[wallSignal];
}

uint16_t opcodes::getCliffLeftSignal()
{
    return sensorWaarden[cliffFrontLeftSignal];
}

uint16_t opcodes::getCliffFrontLeftSignal()
{
    return sensorWaarden[cliffFrontLeftSignal];
}

uint16_t opcodes::getCliffFrontRightSignal()
{
    return sensorWaarden[cliffFrontRightSignal];
}

uint16_t opcodes::getCliffRightSignal()
{
    return sensorWaarden[cliffRightSignal];
}

uint8_t opcodes::getChargingSource()
{
    return sensorWaarden[chargingSource];
}

uint8_t opcodes::getOiMode()
{
    return sensorWaarden[oiMode];
}

uint8_t opcodes::getSongNumber()
{
    return sensorWaarden[songNumber];
}

uint8_t opcodes::getSongPlaying()
{
    return sensorWaarden[songPlaying];
}

uint8_t opcodes::getNumberOfPackets()
{
    return sensorWaarden[numberOfPackets];
}

int16_t opcodes::getRequestedVelocity()
{
    return (int16_t)sensorWaarden[requestedVelocity];
}

int16_t opcodes::getRequestedRadius()
{
    return (int16_t)sensorWaarden[requestedRadius];
}

int16_t opcodes::getRequestedRightVelocity()
{
    return (int16_t)sensorWaarden[requestedRightVelocity];
}

int16_t opcodes::getRequestedLeftVelocity()
{
    return (int16_t)sensorWaarden[requestedLeftVelocity];
}

uint16_t opcodes::getLeftEncoderCount()
{
    return sensorWaarden[leftEncoderCount];
}

uint16_t opcodes::getRightEncoderCount()
{
    return sensorWaarden[rightEncoderCount];
}

uint8_t opcodes::getLightBumper()
{
    //return sensorWaarden[lightBumper] ? 1 : 0;
    return sensorWaarden[lightBumper];
}

uint16_t opcodes::getLightBumpLeftSignal()
{
    return sensorWaarden[lightBumpLeftSignal];
}

uint16_t opcodes::getLightBumpFrontLeftSignal()
{
    return sensorWaarden[lightBumpFrontLeftSignal];
}

uint16_t opcodes::getLightBumpCenterLeftSignal()
{
    return sensorWaarden[lightBumpCenterLeftSignal];
}

uint16_t opcodes::getLightBumpCenterRightSignal()
{
    return sensorWaarden[lightBumpCenterRightSignal];
}

uint16_t opcodes::getLightBumpFrontRightSignal()
{
    return  sensorWaarden[lightBumpFrontRightSignal];
}

uint16_t opcodes::getLightBumperRightSignal()
{
    return sensorWaarden[lightBumperRightSignal];
}

int16_t opcodes::getLeftMotorCurrent()
{
    return (int16_t)sensorWaarden[leftMotorCurrent];
}

int16_t opcodes::getRightMotorCurrent()
{
    return (int16_t)sensorWaarden[rightMotorCurrent];
}

int16_t opcodes::getMainBrushMotorCurrent()
{
    return (int16_t)sensorWaarden[mainBrushMotorCurrent];
}

int16_t opcodes::getSideBrushMotorCurrent()
{
    return (int16_t)sensorWaarden[sideBrushMotorCurrent];
}

uint8_t opcodes::getStatis()
{
    return sensorWaarden[statis];
}

bool opcodes::getBumpRight()
{
    return (sensorWaarden[bumpAndWheel] & 0b00000001) == 0b00000001 ? 1 : 0;
}

bool opcodes::getBumpLeft()
{
    return (sensorWaarden[bumpAndWheel] & 0b00000010) == 0b00000010 ? 1 : 0;
}

bool opcodes::getWheelDropRight()
{
    return (sensorWaarden[bumpAndWheel] & 0b00000100) == 0b00000100 ? 1 : 0;
}

bool opcodes::getWheelDropLeft()
{
    return (sensorWaarden[bumpAndWheel] & 0b00001000) == 0b00001000 ? 1 : 0;
}

bool opcodes::getSideBrushOvercurrent()
{
    return (sensorWaarden[wheelOvercurrents] & 0b00000001) == 0b00000001 ? 1 : 0;
}

bool opcodes::getMainBrushOvercurrent()
{
    return (sensorWaarden[wheelOvercurrents] & 0b00000100) == 0b00000100 ? 1 : 0;
}

bool opcodes::getRightWheelOvercurrent()
{
    return (sensorWaarden[wheelOvercurrents] & 0b00001000) == 0b00001000 ? 1 : 0;
}

bool opcodes::getLeftWheelOvercurrent()
{
    return (sensorWaarden[wheelOvercurrents] & 0b00010000) == 0b00010000 ? 1 : 0;
}

void opcodes::sentUart(uint8_t)
{
    void * point;
    point = &code;
    if (uart0_filestream != -1)
    {
            int count = write(uart0_filestream, point, ((size_t)sizeof(uint8_t)));          //Filestream, bytes $
            if (count < 0)
            {
                    printf("UART TX error\n");
            }
    }

}

void opcodes::startUart()
{
    //-------------------------
    //----- SETUP USART 0 -----
    //-------------------------
    //At bootup, pins 8 and 10 are already set to UART0_TXD, UART0_RXD (ie the alt0 function) respectively
    uart0_filestream = -1;

    //OPEN THE UART
    //The flags (defined in fcntl.h):
    //	Access modes (use 1 of these):
    //		O_RDONLY - Open for reading only.
    //		O_RDWR - Open for reading and writing.
    //		O_WRONLY - Open for writing only.
    //
    //	O_NDELAY / O_NONBLOCK (same function) - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
    //											if there is no input immediately available (instead of blocking). Likewise, write requests can also return
    //											immediately with a failure status if the output can't be written immediately.
    //
    //	O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.
    uart0_filestream = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode
    if (uart0_filestream == -1)
    {
        //ERROR - CAN'T OPEN SERIAL PORT
        printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
    }

    //CONFIGURE THE UART
    //The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
    //	Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
    //	CSIZE:- CS5, CS6, CS7, CS8
    //	CLOCAL - Ignore modem status lines
    //	CREAD - Enable receiver
    //	IGNPAR = Ignore characters with parity errors
    //	ICRNL - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
    //	PARENB - Parity enable
    //	PARODD - Odd parity (else even)
    struct termios options;
    tcgetattr(uart0_filestream, &options);
    options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;		//<Set baud rate
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(uart0_filestream, TCIFLUSH);
    tcsetattr(uart0_filestream, TCSANOW, &options);

}

uint8_t opcodes::howManyDatabytes(uint8_t code)
{
    switch(code)
    {
        case 7:
        case 8:
        case 9:
        case 10:
        case 11:
        case 12:
        case 13:
        case 14:
        case 15:
        case 16:
        case 17:
        case 18:
        case 21:
        case 24:
        case 32:
        case 34:
        case 35:
        case 36:
        case 37:
        case 38:
        case 45:
        case 52:
        case 53:
        case 58:
            return 1;
            break;
        default:
            return 2;
        break;
    }
}

uint8_t opcodes::blablaUart()
{
    /******************************************/
    /* dit bootst een stream van de roomba na */
    /******************************************/
    switch(c)
    {
        case 0:
            return 19;
            c++;
            break;
        case 1:
            return 7; // n-bytes
            c++;
            break;
        case 2:
            return 8; // wall sensor
            c++;
            break;
        case 3:
            return 0; // no wall
            c++;
            break;
        case 4:
            return 12; // cliff right
            c++;
            break;
        case 5:
            return 1; // cliff right detected
            c++;
            break;
        case 6:
            return 48; // light bump left
            c++;
            break;
        case 7:
            return 30; // lightbump lowbyte
            c++;
            break;
        case 8:
            return 160; // lightbump highbyte
            c++;
            break;
        case 9:
            return -266; // checksum
            c = 0;
        default:
            c = 0;
    }
}
