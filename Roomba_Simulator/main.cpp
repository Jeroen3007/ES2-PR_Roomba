#include <iostream>
//#include <iomanip> // setw() setfill()
#include <string>

#include "file.h"

using namespace std;

int main()
{

    //roomobjectclass *roomba;
    //roomba = new roombaclass(null);
    //delete roomba;
    roomclass room;
    cout << room.roomba->iPosHor << " " << room.roomba->iPosVer << endl;
    room.roomba->setangle(40);
    for(int i=0;i<12;i++){
        room.roomba->setspeed(1);
        room.roomba->drive();
        cout << room.roomba->iPosHor << "   " << room.roomba->iPosVer << "      " << room.sensors->getbBumpLeft() << "      " << room.sensors->getbBumpRight() << endl;
        //cout << "---------------------" << endl;
    }
    room.roomba->setangle(-135);
    for(int i=0;i<20;i++){
        room.roomba->setspeed(2);
        room.roomba->drive();
        cout << room.roomba->iPosHor << "   " << room.roomba->iPosVer << "      " << room.sensors->getbBumpLeft() << "      " << room.sensors->getbBumpRight() << endl;
        //cout << "---------------------" << endl;
    }
    return 0;
}

