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

infoActionType noActionFct(positionInteger destination,technique method);
infoActionType StopNowFct(positionInteger destination,technique method);
actionStatutType trajectoryBasicAction(positionInteger destination);
actionStatutType translationBasicAction(int acc, int vit, int dist);
actionStatutType rotationBasicAction(int acc, int vit, int angle);
int compareXYAlpha(positionInteger pos1, positionInteger pos2);
int compareXY(positionInteger pos1, positionInteger pos2);
infoActionType seDeplacer(positionInteger destination,technique method);
infoActionType pousser(positionInteger posObj,technique method);
infoObjet reconaitre(technique method);
infoActionType prendre(positionInteger posObj,technique method);
infoActionType gener(positionInteger destination,technique method);
int compareAlpha(positionInteger pos1, positionInteger pos2);

#endif	/* BASICACTIONS_H */

