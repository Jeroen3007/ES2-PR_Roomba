#include "failsave.h"
#include "opcodes.h"

failsave::failsave()
{
    stop = false;
    Interpreter = new interpreter;
    uart = new Uart;
}

failsave::~failsave()
{
    stopFailsave();
    delete Interpreter;
}

void failsave::stopFailsave()
{
    stop = true;
}

void failsave::checkCodes()
{
    while(1)
    {
        if(checkRunning())break; // stop thread when requested
        usleep(100);

        Interpreter->lockMutex(); // lock mutex for send and receive

        /* check critical sensors and intervene if neccesary*/
        using namespace roomba;

        if(Interpreter->getBumpAndWheel())uart->sendUart(Stop);
        if(Interpreter->getCliffLeftSignal() > 2048)uart->sendUart(Stop);
        if(Interpreter->getCliffFrontLeftSignal() > 2048)uart->sendUart(Stop);
        if(Interpreter->getCliffFrontRightSignal() > 2048)uart->sendUart(Stop);
        if(Interpreter->getCliffRightSignal() > 2048)uart->sendUart(Stop);
        if(Interpreter->getLeftWheelOvercurrent())uart->sendUart(Stop);
        if((Interpreter->getBatteryCapacity()/Interpreter->getBatteryCharge()*100) < 10)uart->sendUart(Stop);

        Interpreter->unlockMutex(); // free mutex for standard procedure
    }
}

bool failsave::checkRunning()
{
    return stop;
}
