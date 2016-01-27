#ifndef CONSIGNE_H
#define	CONSIGNE_H


#include "../can/CanProp.h"

	typedef enum {
		CSG_STANDING = 0,
		CSG_MOVING = 1
	} csgStatusType;


	// Initialise le g�n�rateur de consigne et renvoie son �tat initial (CSG_STANDING)
	// Les param�tres de la fonction sont les vitesses et acc�l�rations relatives nominales
	csgStatusType csgInit(relativePosType nomVel, relativePosType nomAcc);
	// Calcule les nouvelles consignes. Elle doit �tre appell�e UNE SEULE FOIS � chaque p�riode d'�chantillonnage
	csgStatusType csgCompute(void);
	// Modifie les acc�l�rations relatives nominales
	void csgSetNomAcc(relativePosType nomAcc);
	// Modifie les vitesses relatives nominales
	void csgSetNomVel(relativePosType nomVel);
	// D�finit une nouvelle position finale
	void csgSetFinalPos(relativePosType finalPos);
	// Renvoie la consigne de vitesses relatives actuelle
	relativePosType csgGetVel(void);
	// Renvoie la consigne de positions relatives actuelle
	relativePosType csgGetPos(void);
	// Renvoie l'�tat actuel du g�n�rateur de consigne
	csgStatusType csgGetState(void);
	// Renvoie les acc�l�rations relatives nominales
	relativePosType csgGetNomAcc(void);
	// Renvoie les vitesses relatives nominales
	relativePosType csgGetNomVel(void);
	// Renvoie une nouvelle position finale
	relativePosType csgGetFinalPos(void);

#endif