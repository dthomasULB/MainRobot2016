#ifndef CONSIGNE_H
#define	CONSIGNE_H


#include "../can/CanProp.h"

	typedef enum {
		CSG_STANDING = 0,
		CSG_MOVING = 1
	} csgStatusType;


	// Initialise le générateur de consigne et renvoie son état initial (CSG_STANDING)
	// Les paramètres de la fonction sont les vitesses et accélérations relatives nominales
	csgStatusType csgInit(relativePosType nomVel, relativePosType nomAcc);
	// Calcule les nouvelles consignes. Elle doit être appellée UNE SEULE FOIS à chaque période d'échantillonnage
	csgStatusType csgCompute(void);
	// Modifie les accélérations relatives nominales
	void csgSetNomAcc(relativePosType nomAcc);
	// Modifie les vitesses relatives nominales
	void csgSetNomVel(relativePosType nomVel);
	// Définit une nouvelle position finale
	void csgSetFinalPos(relativePosType finalPos);
	// Renvoie la consigne de vitesses relatives actuelle
	relativePosType csgGetVel(void);
	// Renvoie la consigne de positions relatives actuelle
	relativePosType csgGetPos(void);
	// Renvoie l'état actuel du générateur de consigne
	csgStatusType csgGetState(void);
	// Renvoie les accélérations relatives nominales
	relativePosType csgGetNomAcc(void);
	// Renvoie les vitesses relatives nominales
	relativePosType csgGetNomVel(void);
	// Renvoie une nouvelle position finale
	relativePosType csgGetFinalPos(void);

#endif