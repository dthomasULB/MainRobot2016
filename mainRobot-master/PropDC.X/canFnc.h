/* 
 * File:   canFnc.h
 * Author: Kevin
 *
 * Created on 13 juillet 2015, 14:19
 */

#ifndef CANFNC_H
#define	CANFNC_H


#include "globals.h"

void canPropInit(void);
absolutePosType canOrderGetPos(void);
#define canGetPatinageEnable()  (canReceivedData[0])
translationParamType canGetTranslation(void);
rotationParamType canGetRotation(void);
obstacleType canOrderGetObstacle(void);
void canSendState(propStateType curState);
void canSendPatinageFlag(void);
void canSendPos(absolutePosType curPos);
void canSendRelCsg(relativePosType curPos);
void canSendRelPos(relativePosType curPos);
void canSendIsObstacle(propIsObstacleType isObstacle);

#endif	/* CANFNC_H */

