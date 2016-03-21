#ifndef FAILSAVE_H
#define FAILSAVE_H

#include <unistd.h>
#include "interpreter.h"
#include "UART/uart.h"

class interpreter;

class failsave
{
public:
    failsave();
    ~failsave();

    void stopFailsave();

private:
    void checkCodes();
    bool checkRunning();

    bool stop;

    interpreter* Interpreter;
    Uart *uart;
};

#endif // FAILSAVE_H
