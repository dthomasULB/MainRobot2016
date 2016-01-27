/* 
 * File:   basicActions.h
 * Author: Kevin
 *
 * Created on 21 septembre 2015, 10:16
 */

#ifndef BASICACTIONS_H
#define	BASICACTIONS_H


#include "dataTypes.h"
#include "../Can/CanTypes.h"

infoActionType noActionFct(int option);
actionStatutType trajectoryBasicAction(positionInteger destination);
actionStatutType translationBasicAction(int acc, int vit, int dist);
actionStatutType rotationBasicAction(int acc, int vit, int angle);
int compareXYAlpha(positionInteger pos1, positionInteger pos2);
int compareXY(positionInteger pos1, positionInteger pos2);


#endif	/* BASICACTIONS_H */

