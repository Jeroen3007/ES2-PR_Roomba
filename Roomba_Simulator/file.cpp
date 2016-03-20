#include "file.h"

/**
 * @brief roomclass::roomclass
 * standard constructor for roomclass.
 * during testing it will create objects on its own for testing purposes.
 */
roomclass::roomclass(){
    sensors = new sensorclass(*this);
    roomba = new roombaclass(sensors);
#ifdef __WIN32
#ifdef __linux__
	timer = new timerclass(*roomba);
#endif
#endif

    roomobjectclass object(-5,-5,0,10);
    roomobjects.push_back(object);
    roomobjectclass objectb(5,-5,0,10);
    roomobjects.push_back(objectb);


    roomobjectclass objectc(-5,-5,10,0);
    roomobjects.push_back(objectc);

    roomobjectclass objectd(-5,5,10,0);
    roomobjects.push_back(objectd);

}
/**
 * @brief roomclass::~roomclass
 * default constructor
 */
roomclass::~roomclass(){
    delete sensors;
    delete roomba;
}

/**
 * @brief sensorclass::sensorclass
 * @param room
 * Default constructor for sensorclass and takes a reference param room in order to access the room
 */
sensorclass::sensorclass(roomclass& room) : room(room){
    bBumpLeft = false;
    bBumpRight = false;
}
/**
 * @brief sensorclass::calcmultiplication
 * @param iDiffHor
 * @param iDiffVer
 * @return float
 */
inline float sensorclass::calcmultiplication(float iDiffHor, float iDiffVer){
        return((float)(iDiffVer / iDiffHor));
}
/**
 * @brief sensorclass::checkbump
 * @param fHorMov
 * @param fVerMov
 * @return
 * This function implements at the moment (20-03-16) the feature of sensorclass
 * to detect objects and to stop the roomba.
 * In the future this will also implement the soft bump and cliff sensors
 */
bool sensorclass::checkbump(float fHorMov, float fVerMov){
    /** The following code makes sure that if a value is below 1 and
     *  above -1 and is not equal to 0 (+/- fFloatRange) that it wil
     *  be rounded up or down to the nearest full int value.
     */
    int iHorMov = fHorMov;
    int iVerMov = fVerMov;
    if(iVerMov == 0){
        if(floatcomp(fVerMov,0.0))
            iVerMov = 0;
        else{
            if(fVerMov > 0)
                iVerMov = 1;
            else
                iVerMov = -1;
        }
    }
    if(iHorMov == 0){
        if(floatcomp(fHorMov,0.0))
            iHorMov = 0;
        else{
            if(fHorMov > 0)
                iHorMov = 1;
            else
                iHorMov = -1;
        }
    }

    if(room.roomobjects.size() != 0){ /**< check if there are objects inside the room other than the roomba. if not do nothing an return false **/
        #warning "the following code should be optimized, it will now evaluate all if statements altough a if statement has already been true"
        /**< The following if statements will check the direction of the movement and trigger the right checkbumpXX() function **/
        if(/*checkbumpUD */floatcomp(iHorMov,0)){    /**< if the direction is vertical    **/
            if(iVerMov>0) /**< check if vertical movement is positive or negative **/
                for(int iPosVer=0;iPosVer<=iVerMov;iPosVer++){ /**< follow the complete move path through and check for obstacles **/
                    if(checkbumpU(room.roomba->iPosHor,(room.roomba->iPosVer+iPosVer)) == true){
                       --iPosVer;
                        room.roomba->move(0,iPosVer);
                        bBumpLeft = true;
                        bBumpRight = true;
                        return(true);
                    }
                }
            else
                for(int iPosVer=0;iPosVer>=iVerMov;iPosVer--){ /**< follow the complete move path through and check for obstacles **/
                    if(checkbumpD(room.roomba->iPosHor,(room.roomba->iPosVer+iPosVer)) == true){ /**< check the current position + movement for objects **/
                        room.roomba->move(0,iPosVer);
                        bBumpLeft = true;
                        bBumpRight = true;
                        return(true);
                    }
                }
        }
        if(/*checkbumpLR */floatcomp(iVerMov,0)){    /**< if the direction is horizontal  **/
            if(iHorMov>0) /**< check if horizontal movement is positive or negative **/
                for(int iPosHor=0;iPosHor<=iHorMov;iPosHor++){ /**< follow the complete move path through and check for obstacles **/
                    if(checkbumpR((room.roomba->iPosHor+iPosHor),room.roomba->iPosVer) == true){
                        iPosHor--;
                        room.roomba->move(iPosHor,0);
                        bBumpLeft = true;
                        bBumpRight = true;
                        return(true);
                    }
                }
            else
                for(int iPosHor=0;iPosHor>=iHorMov;iPosHor--){ /**< follow the complete move path through and check for obstacles **/
                    if(checkbumpL((room.roomba->iPosHor+iPosHor),room.roomba->iPosVer) == true){
                        room.roomba->move(iPosHor,0);
                        bBumpLeft = true;
                        bBumpRight = true;
                        return(true);
                    }
                }
        }
        if(/*checkbumpUL*/(iHorMov<0)&&(iVerMov>0)){ /**< if the direction is diagonal UL **/
            //checkbumpUL();
            int iResult=0;
            float fResult;
            vector<int> viIanswerVer;
            vector<int> viIanswerHor;
            bool bAnswerUsed = false;
            
            /** This code analyses the diagonal path the roomba will take an find the x,y integer positions where the roomba will move through.
              * Those x,y positions will be pushed inside the vectors and tested for bumps by checkbumpUL()
              */
            for(float fCounterHor=0;fCounterHor>((float)iHorMov-0.1);fCounterHor-=0.1){
            	fResult = calcmultiplication((float)iHorMov,(float)iVerMov);
            	iResult= (int)(fResult * fCounterHor);
            	bAnswerUsed = false;
            	for(unsigned int i=0;i<viIanswerVer.size();i++){
            		if(viIanswerVer[i] == iResult){
            			bAnswerUsed = true;
            			break;
            		}
            	}
            	if(bAnswerUsed == false){
            		viIanswerVer.push_back(iResult);
            		viIanswerHor.push_back((int)fCounterHor);
            	}
            }
            for(unsigned int i=0;i<viIanswerHor.size();i++){ /**< check the found x,y positions for obstacles **/
                if(checkbumpUL(room.roomba->iPosHor+viIanswerHor[i],room.roomba->iPosVer+viIanswerVer[i]) == true){
                    room.roomba->move(viIanswerHor[i],viIanswerVer[i]);
                    return(true);
            	}
            }
        }
        if(/*checkbumpUR*/(iHorMov>0)&&(iVerMov>0)){ /**< if the direction is diagonal UR **/
            //checkbumpUR();
            int iResult=0;
            float fResult;
            vector<int> viIanswerVer;
            vector<int> viIanswerHor;
            bool bAnswerUsed = false;

            /** This code analyses the diagonal path the roomba will take an find the x,y integer positions where the roomba will move through.
              * Those x,y positions will be pushed inside the vectors and tested for bumps by checkbumpUL()
              */
            for(float fCounterHor=0;fCounterHor<((float)iHorMov+0.1);fCounterHor+=0.1){
                fResult = calcmultiplication((float)iHorMov,(float)iVerMov);
                iResult= (int)(fResult * fCounterHor);
                bAnswerUsed = false;
                for(unsigned int i=0;i<viIanswerVer.size();i++){
                    if(viIanswerVer[i] == iResult){
                        bAnswerUsed = true;
                        break;
                    }
                }
                if(bAnswerUsed == false){
                    viIanswerVer.push_back(iResult);
                    viIanswerHor.push_back((int)fCounterHor);
                }

            }
            for(unsigned int i=0;i<viIanswerHor.size();i++){ /**< check the found x,y positions for obstacles **/
                if(checkbumpUR(room.roomba->iPosHor+viIanswerHor[i],room.roomba->iPosVer+viIanswerVer[i]) == true){
                    room.roomba->move(viIanswerHor[i],viIanswerVer[i]);
                    return(true);
                }
            }
        }
        if(/*checkbumpDR*/(iHorMov>0)&&(iVerMov<0)){ /**< if the direction is diagonal DL **/
            //checkbumpDR();
            int iResult=0;

            float fResult;
            vector<int> viIanswerVer;
            vector<int> viIanswerHor;
            bool bAnswerUsed = false;

            /** This code analyses the diagonal path the roomba will take an find the x,y integer positions where the roomba will move through.
              * Those x,y positions will be pushed inside the vectors and tested for bumps by checkbumpUL()
              */
            for(float fCounterHor=0;fCounterHor<((float)iHorMov+0.1);fCounterHor+=0.1){
                fResult = calcmultiplication((float)iHorMov,(float)iVerMov);
                iResult= (int)(fResult * fCounterHor);
                bAnswerUsed = false;
                for(unsigned int i=0;i<viIanswerVer.size();i++){
                    if(viIanswerVer[i] == iResult){
                        bAnswerUsed = true;
                        break;
                    }

                }
                if(bAnswerUsed == false){
                    viIanswerVer.push_back(iResult);
                    viIanswerHor.push_back((int)fCounterHor);
                }

            }
            for(unsigned int i=0;i<viIanswerHor.size();i++){ /**< check the found x,y positions for obstacles **/
                if(checkbumpDR(room.roomba->iPosHor+viIanswerHor[i],room.roomba->iPosVer+viIanswerVer[i]) == true){
                    //room.roomba->iPosHor += viIanswerHor[i];
                    //room.roomba->iPosVer += viIanswerVer[i];
                    room.roomba->move(viIanswerHor[i],viIanswerVer[i]);
                    return(true);
                }
            }
        }
        if(/*checkbumpDL*/(iHorMov<0)&&(iVerMov<0)){ /**< if the direction is diagonal DR **/
            //checkbumpDL();
            int iResult=0;

            float fResult;
            vector<int> viIanswerVer;
            vector<int> viIanswerHor;
            bool bAnswerUsed = false;

            /** This code analyses the diagonal path the roomba will take an find the x,y integer positions where the roomba will move through.
              * Those x,y positions will be pushed inside the vectors and tested for bumps by checkbumpUL()
              */
            for(float fCounterHor=0;fCounterHor>((float)iHorMov-0.1);fCounterHor-=0.1){
            	fResult = calcmultiplication((float)iHorMov,(float)iVerMov);
            	iResult= (int)(fResult * fCounterHor);
            	bAnswerUsed = false;
            	for(unsigned int i=0;i<viIanswerVer.size();i++){
            		if(viIanswerVer[i] == iResult){
            			bAnswerUsed = true;
            			break;
            		}
            	}
            	if(bAnswerUsed == false){
            		viIanswerVer.push_back(iResult);
            		viIanswerHor.push_back((int)fCounterHor);
            	}
            
            }
            for(unsigned int i=0;i<viIanswerHor.size();i++){ /**< check the found x,y positions for obstacles **/
                if(checkbumpDL(room.roomba->iPosHor+viIanswerHor[i],room.roomba->iPosVer+viIanswerVer[i]) == true){
                    //room.roomba->iPosHor += viIanswerHor[i];
                    //room.roomba->iPosVer += viIanswerVer[i];
                    room.roomba->move(viIanswerHor[i],viIanswerVer[i]);
                    return(true);
            	}
            }
        }
    }

    bBumpLeft = false;
    bBumpRight = false;
    return(false);
}
/**
 * @brief sensorclass::checkbumpL
 * @param iHorPos
 * @param iVerPos
 * @return
 * This function checks for a bump at the position where the roomba will move into.
 */
bool sensorclass::checkbumpL(int iHorPos,int iVerPos){
    for(unsigned int i=0;i<room.roomobjects.size();i++){ /**< loops through all objects inside the room**/
        for(int iHorI=room.roomobjects[i].iPosHor;iHorI<=(room.roomobjects[i].iPosHor+(signed int)room.roomobjects[i].iSizeHor);iHorI++){ /**< loops through all horizontal positions of that object **/
            for(int iVerI=room.roomobjects[i].iPosVer;iVerI<=(room.roomobjects[i].iPosVer+(signed int)room.roomobjects[i].iSizeVer);iVerI++){ /**< loops throught all vertical positions of that object **/
                if((iHorPos == iHorI)&&(iVerPos == iVerI)){
                    return(true);
                }
            }
        }
    }
    return(false);
}
/**
 * @brief sensorclass::checkbumpR
 * @param iHorPos
 * @param iVerPos
 * @return
 * This function checks for a bump at the position where the roomba will move into.
 */
bool sensorclass::checkbumpR(int iHorPos,int iVerPos){
    for(unsigned int i=0;i<room.roomobjects.size();i++){ /**< loops through all objects inside the room**/
        for(int iHorI=room.roomobjects[i].iPosHor;iHorI<=(room.roomobjects[i].iPosHor+(signed int)room.roomobjects[i].iSizeHor);iHorI++){ /**< loops through all horizontal positions of that object **/
            for(int iVerI=room.roomobjects[i].iPosVer;iVerI<=(room.roomobjects[i].iPosVer+(signed int)room.roomobjects[i].iSizeVer);iVerI++){ /**< loops throught all vertical positions of that object **/
                if((iHorPos == iHorI)&&(iVerPos == iVerI)){
                    return(true);
                }
            }
        }
    }
    return(false);
}
/**
 * @brief sensorclass::checkbumpU
 * @param iHorPos
 * @param iVerPos
 * @return
 * This function checks for a bump at the position where the roomba will move into.
 */
bool sensorclass::checkbumpU(int iHorPos,int iVerPos){
    for(unsigned int i=0;i<room.roomobjects.size();i++){ /**< loops through all objects inside the room**/
        for(int iHorI=room.roomobjects[i].iPosHor;iHorI<=(room.roomobjects[i].iPosHor+(signed int)room.roomobjects[i].iSizeHor);iHorI++){ /**< loops through all horizontal positions of that object **/
            for(int iVerI=room.roomobjects[i].iPosVer;iVerI<=(room.roomobjects[i].iPosVer+(signed int)room.roomobjects[i].iSizeVer);iVerI++){ /**< loops throught all vertical positions of that object **/
                if((iHorPos == iHorI)&&(iVerPos == iVerI)){
                    return(true);
                }
            }
        }
    }
    return(false);
}
/**
 * @brief sensorclass::checkbumpD
 * @param iHorPos
 * @param iVerPos
 * @return
 * This function checks for a bump at the position where the roomba will move into.
 */
bool sensorclass::checkbumpD(int iHorPos,int iVerPos){
    for(unsigned int i=0;i<room.roomobjects.size();i++){ /**< loops through all objects inside the room**/
        for(int iHorI=room.roomobjects[i].iPosHor;iHorI<=(room.roomobjects[i].iPosHor+(signed int)room.roomobjects[i].iSizeHor);iHorI++){ /**< loops through all horizontal positions of that object **/
            for(int iVerI=room.roomobjects[i].iPosVer;iVerI<=(room.roomobjects[i].iPosVer+(signed int)room.roomobjects[i].iSizeVer);iVerI++){ /**< loops throught all vertical positions of that object **/
                if((iHorPos == iHorI)&&(iVerPos == iVerI)){
                    return(true);
                }
            }
        }
    }
    return(false);
}
/**
 * @brief sensorclass::checkbumpUL
 * @param iHorPos
 * @param iVerPos
 * @return
 * This function checks for a bump at the position where the roomba will move into.
 */
bool sensorclass::checkbumpUL(int iHorPos,int iVerPos){
    bool bDoubleBump = false;
    bool bLeftBump = false;
    bool bRightBump = false;
    for(unsigned int i=0;i<room.roomobjects.size();i++){
        for(int iHorI=room.roomobjects[i].iPosHor;iHorI<=(room.roomobjects[i].iPosHor+(signed int)room.roomobjects[i].iSizeHor);iHorI++){
            for(int iVerI=room.roomobjects[i].iPosVer;iVerI<=(room.roomobjects[i].iPosVer+(signed int)room.roomobjects[i].iSizeVer);iVerI++){
                if(bDoubleBump == false)
                    bDoubleBump = (iHorPos-1 == iHorI)&&(iVerPos+1 == iVerI);
                if(bLeftBump == false)
                    bLeftBump = (((iHorPos-1) == iHorI)&&(iVerPos == iVerI));
                if(bRightBump == false)
                    bRightBump = ((iHorPos == iHorI)&&((iVerPos+1) == iVerI));
            }

        }
    }
    if(bDoubleBump||bLeftBump||bRightBump){ /**< if any bumps where detected set the senor bools **/
        if(bDoubleBump){
            bBumpLeft = true;
            bBumpRight = true;
            return(true);
        }
        else{
            if(bLeftBump)
                bBumpLeft = true;
            if(bRightBump)
                bBumpRight = true;
        }
        return(true);
    }
    return(false);
}
/**
 * @brief sensorclass::checkbumpUR
 * @param iHorPos
 * @param iVerPos
 * @return
 * This function checks for a bump at the position where the roomba will move into.
 */
bool sensorclass::checkbumpUR(int iHorPos,int iVerPos){
    bool bDoubleBump = false;
    bool bLeftBump = false;
    bool bRightBump = false;
    for(unsigned int i=0;i<room.roomobjects.size();i++){
        for(int iHorI=room.roomobjects[i].iPosHor;iHorI<=(room.roomobjects[i].iPosHor+(signed int)room.roomobjects[i].iSizeHor);iHorI++){
            for(int iVerI=room.roomobjects[i].iPosVer;iVerI<=(room.roomobjects[i].iPosVer+(signed int)room.roomobjects[i].iSizeVer);iVerI++){
                if(bDoubleBump == false)
                    bDoubleBump = (iHorPos+1 == iHorI)&&(iVerPos+1 == iVerI);
                if(bLeftBump == false)
                    bLeftBump = (((iHorPos+1) == iHorI)&&(iVerPos == iVerI));
                if(bRightBump == false)
                    bRightBump = ((iHorPos == iHorI)&&((iVerPos+1) == iVerI));
            }
        }
    }
    if(bDoubleBump||bLeftBump||bRightBump){ /**< if any bumps where detected set the senor bools **/
        if(bDoubleBump){
            bBumpLeft = true;
            bBumpRight = true;
            return(true);
        }
        else{
            if(bLeftBump)
                bBumpLeft = true;
            if(bRightBump)
                bBumpRight = true;
        }
        return(true);
    }
    return(false);
}
/**
 * @brief sensorclass::checkbumpDL
 * @param iHorPos
 * @param iVerPos
 * @return
 * This function checks for a bump at the position where the roomba will move into.
 */
bool sensorclass::checkbumpDL(int iHorPos,int iVerPos){
    bool bDoubleBump = false;
    bool bLeftBump = false;
    bool bRightBump = false;
    for(unsigned int i=0;i<room.roomobjects.size();i++){
        for(int iHorI=room.roomobjects[i].iPosHor;iHorI<=(room.roomobjects[i].iPosHor+(signed int)room.roomobjects[i].iSizeHor);iHorI++){
            for(int iVerI=room.roomobjects[i].iPosVer;iVerI<=(room.roomobjects[i].iPosVer+(signed int)room.roomobjects[i].iSizeVer);iVerI++){
                if(bDoubleBump == false)
                    bDoubleBump = ((iHorPos-1) == iHorI)&&((iVerPos-1)== iVerI);
                if(bLeftBump == false)
                    bLeftBump = ((iHorPos == iHorI)&&((iVerPos-1) == iVerI));
                if(bRightBump == false)
                    bRightBump = (((iHorPos-1) == iHorI)&&(iVerPos == iVerI));
            }
        }
    }
    if(bDoubleBump||bLeftBump||bRightBump){ /**< if any bumps where detected set the senor bools **/
        if(bDoubleBump){
            bBumpLeft = true;
            bBumpRight = true;
            return(true);
        }
        else{
            if(bLeftBump)
                bBumpLeft = true;
            if(bRightBump)
                bBumpRight = true;
        }
        return(true);
    }
    return(false);
}
/**
 * @brief sensorclass::checkbumpDR
 * @param iHorPos
 * @param iVerPos
 * @return
 * This function checks for a bump at the position where the roomba will move into.
 */
bool sensorclass::checkbumpDR(int iHorPos,int iVerPos){
    bool bDoubleBump = false;
    bool bLeftBump = false;
    bool bRightBump = false;
    for(unsigned int i=0;i<room.roomobjects.size();i++){
        for(int iHorI=room.roomobjects[i].iPosHor;iHorI<=(room.roomobjects[i].iPosHor+(signed int)room.roomobjects[i].iSizeHor);iHorI++){
            for(int iVerI=room.roomobjects[i].iPosVer;iVerI<=(room.roomobjects[i].iPosVer+(signed int)room.roomobjects[i].iSizeVer);iVerI++){
                if(bDoubleBump == false)
                    bDoubleBump = (iHorPos+1 == iHorI)&&(iVerPos-1 == iVerI);
                if(bLeftBump == false)
                    bLeftBump = (((iHorPos+1) == iHorI)&&(iVerPos == iVerI));
                if(bRightBump == false)
                    bRightBump = ((iHorPos == iHorI)&&((iVerPos-1) == iVerI));
            }
        }
    }
    if(bDoubleBump||bLeftBump||bRightBump){ /**< if any bumps where detected set the senor bools **/
        if(bDoubleBump){
            bBumpLeft = true;
            bBumpRight = true;
            return(true);
        }
        else{
            if(bLeftBump)
                bBumpLeft = true;
            if(bRightBump)
                bBumpRight = true;
        }
        return(true);
    }
    return(false);
}
/**
 * @brief sensorclass::floatcomp
 * @param fIn1
 * @param fIn2
 * @return float
 * checks if fIn1 ~= fIn2
 */
bool sensorclass::floatcomp(float fIn1,float fIn2){
    if(fabs(fIn1-fIn2)>fFloatRange)
        return(false);
    else
        return(true);
}

//#ifdef __WIN32
//#ifdef __linux__
//timerclass::timerclass(roombaclass& roomba):roomba(roomba){
//    timer.it_value.tv_sec = 1;
//	timer.it_value.tv_usec = 0;
//    timer.it_interval.tv_sec = 1;
//	timer.it_interval.tv_usec = 0;
//    signal(SIGALRM, &sigalrm_handler);
//    setitimer(ITIMER_REAL, &timer, NULL);

//}
//void timerclass::sigalrm_handler(int signum){
//	roomba->drive();
//}
//#endif
//#endif

roomobjectclass::roomobjectclass(signed int iPosHor,signed int iPosVer):iPosHor(iPosHor),iPosVer(iPosVer),fPosVer(iPosVer),fPosHor(iPosHor){
    iSizeHor = 0;
    iSizeVer = 0;
}
roomobjectclass::roomobjectclass(signed int iPosHor,signed int iPosVer,unsigned int iSizeHor,unsigned int iSizeVer):iPosHor(iPosHor),iPosVer(iPosVer),fPosVer(iPosVer),fPosHor(iPosHor),iSizeHor(iSizeHor),iSizeVer(iSizeVer){

}

roombaclass::roombaclass(sensorclass* sensors):roomobjectclass(0,0),sensors(*sensors){
    fAngle = 0;
    fSpeed = 0;
}
roombaclass::~roombaclass(){

}
void roombaclass::drive(void){
    // sin(angle) * speed = hor movement
    // cos(angle) * speed = ver movementy
    //float fHorMov = sin((fAngle*(pi/(float)180)) * fSpeed);

    float fHorMov = sin(fAngle*pi/180) * fSpeed;
    float fVerMov = cos(fAngle*pi/180) * fSpeed;


    if(sensors.checkbump(fHorMov,fVerMov) == false){
        move(fHorMov,fVerMov);
    }
}
void roombaclass::setspeed(float fInputSpeed){
    fSpeed = fInputSpeed;
}
void roombaclass::setangle(float fInputAngle){
    fAngle = fInputAngle;
}
void roombaclass::move(float fHorMov,float fVerMov){
    fPosHor += fHorMov;
    fPosVer += fVerMov;

    iPosHor += fPosHor-iPosHor;
    iPosVer += fPosVer-iPosVer;
}

string wallclass::readobjectname(void){
    return(sObjectName);
}
void wallclass::writeobjectname(string sInput){
   sObjectName = sInput;
}
