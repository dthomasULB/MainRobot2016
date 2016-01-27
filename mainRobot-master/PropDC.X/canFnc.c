#include <xc.h>
#include "../libdspic/timers.h"
#include "../libdspic/CanDspic.h"
#include "../Can/CanNetwork.h"
#include "../can/CanProp.h"
#include "globals.h"
#include "generTraj.h"
#include "consigne.h"
#include "odolibre.h"


#define RAD_TO_DEG10	(1800.0/PI)
#define DEG10_TO_RAD	(PI/1800.0)


// type défini pour pouvoir manipuler efficacement les octets d'un int
union INTEG {
	int i;
	unsigned int ui;
	char b[2];
	unsigned char ub[2];
};

// variables CAN
positionInteger canAbsPos;                          //!< position actuelle du robot, en mm et deg/10
propStateType canState;                             //!< Etat de la propulsion
relativeCoordInteger canOdoRelPos, canCsgRelPos;    //!< position et consigne dans les coordonnées relatives (mm et deg/10)
char canPatinage = 1;
propIsObstacleType canIsObstacle;




void canPropInit(void) {
	CanInitialisation(CN_PROPULSION);	// Initialise le périphérique CAN
	ACTIVATE_CAN_INTERRUPTS = 1;	    // Enables CAN interrupts
	// Déclare les objets CAN produits par ce PIC
	CanDeclarationProduction(CO_PROP_STATUS, &canState, sizeof(canState));
	CanDeclarationProduction(CO_PROP_POS, &canAbsPos, sizeof(canAbsPos));
	CanDeclarationProduction(CO_PROP_REL_CSG, &canCsgRelPos, sizeof(canCsgRelPos));
	CanDeclarationProduction(CO_PROP_REL_ODO, &canOdoRelPos, sizeof(canOdoRelPos));
	CanDeclarationProduction(CO_PROP_PATINAGE, &canPatinage, sizeof(canPatinage));
    CanDeclarationProduction(CO_PROP_IS_OBSTACLE, &canIsObstacle, sizeof(canIsObstacle));
	CanEnvoiProduction(&canState);
}


// FONCTIONS D'ENVOI DES OBJETS PRODUITS
/////////////////////////////////////////////////////////////////////////////////////////
void canSendState(propStateType curState) {
    canState = curState;
    CanEnvoiProduction(&canState);
}

void canSendPatinageFlag(void) {
    CanEnvoiProduction(&canPatinage);
}

void canSendPos(absolutePosType curPos) {
    canAbsPos.x = (int)(1000*curPos.x);
    canAbsPos.y = (int)(1000*curPos.y);
    canAbsPos.alpha = (int)(RAD_TO_DEG10*curPos.alpha);
    CanEnvoiProduction(&canAbsPos);
}

void canSendRelCsg(relativePosType curPos) {
    canCsgRelPos.l = (int)(1000*curPos.l);
    canCsgRelPos.r = (int)(RAD_TO_DEG10*curPos.r);
    CanEnvoiProduction(&canCsgRelPos);
}

void canSendRelPos(relativePosType curPos) {
    canOdoRelPos.l = (int)(1000*curPos.l);
    canOdoRelPos.r = (int)(RAD_TO_DEG10*curPos.r);
    CanEnvoiProduction(&canOdoRelPos);
}

void canSendIsObstacle(propIsObstacleType isObstacle) {
    canIsObstacle = isObstacle;
    CanEnvoiProduction(&canIsObstacle);
}
/////////////////////////////////////////////////////////////////////////////////////////



absolutePosType canOrderGetPos(void) {
    absolutePosType newPos;
    union INTEG tempINTEG;

	tempINTEG.ub[0] = canReceivedData[0];		tempINTEG.ub[1] = canReceivedData[1];
	newPos.x = 1E-3*tempINTEG.i;
	tempINTEG.ub[0] = canReceivedData[2];		tempINTEG.ub[1] = canReceivedData[3];
	newPos.y = 1E-3*tempINTEG.i;
	tempINTEG.ub[0] = canReceivedData[4];		tempINTEG.ub[1] = canReceivedData[5];
	newPos.alpha = DEG10_TO_RAD*tempINTEG.i;
    
    return(newPos);
}

obstacleType canOrderGetObstacle(void) {
    obstacleType obstacle;
    union INTEG tempINTEG;

	tempINTEG.ub[0] = canReceivedData[0];		tempINTEG.ub[1] = canReceivedData[1];
	obstacle.x = 1E-3*tempINTEG.i;
	tempINTEG.ub[0] = canReceivedData[2];		tempINTEG.ub[1] = canReceivedData[3];
	obstacle.y = 1E-3*tempINTEG.i;
	tempINTEG.ub[0] = canReceivedData[4];		tempINTEG.ub[1] = canReceivedData[5];
	obstacle.size = tempINTEG.i;
    
    return(obstacle);
}


translationParamType canGetTranslation(void) {
    translationParamType param;
    union INTEG tempINTEG;

	tempINTEG.ub[0] = canReceivedData[0];		tempINTEG.ub[1] = canReceivedData[1];
	param.acc = 1E-3*tempINTEG.i;
	tempINTEG.ub[0] = canReceivedData[2];		tempINTEG.ub[1] = canReceivedData[3];
	param.vel = 1E-3*tempINTEG.i;
	tempINTEG.ub[0] = canReceivedData[4];		tempINTEG.ub[1] = canReceivedData[5];
	param.length = 1E-3*tempINTEG.i;

    return(param);
}


rotationParamType canGetRotation(void) {
    rotationParamType param;
    union INTEG tempINTEG;

	tempINTEG.ub[0] = canReceivedData[0];		tempINTEG.ub[1] = canReceivedData[1];
	param.acc = DEG10_TO_RAD*tempINTEG.i;
	tempINTEG.ub[0] = canReceivedData[2];		tempINTEG.ub[1] = canReceivedData[3];
	param.vel = DEG10_TO_RAD*tempINTEG.i;
	tempINTEG.ub[0] = canReceivedData[4];		tempINTEG.ub[1] = canReceivedData[5];
	param.angle = DEG10_TO_RAD*tempINTEG.i;

    return(param);
}
