/**
 * @file sharp.c
 * @author bULBot 2011-2012
 * @brief Implémentation des fonctions de contrôle des capteurs sharp.
 */


#include "../libdspic/adc.h"
#include "gestionPropulsion.h"
#include "sharp.h"
#include <math.h>
#include "globals.h"

#define DEG2RAD(x) (x*(PI/1800))


detectionSharpType detectionObstacleSharp() {
    int sharpMes;
	detectionSharpType detectionSharp;
    positionInteger curPos;
    
	sharpMes = conversionADC(INPUT_SHARP_AVANT);
	if (sharpMes > SEUIL_COLLISION_AVANT) {
		detectionSharp.isObstacleDetected = 1;
        curPos = propulsionGetPosition();
        detectionSharp.obstacleInfo.x = curPos.x + cos(curPos.alpha)*DISTANCE_OBSTACLE_AVANT;
		detectionSharp.obstacleInfo.y = curPos.y + sin(curPos.alpha) *DISTANCE_OBSTACLE_AVANT;
	} else if(sharpMes < SEUIL_COLLISION_AVANT-HYSTERESE_AVANT) {
		detectionSharp.isObstacleDetected = 0;
    }
    return (detectionSharp);
}
