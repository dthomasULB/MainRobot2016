#include "../Can/CanProp.h"
#include "../libdspic/CanDspic.h"
#include "../Can/CanTypes.h"
#include "../Can/CanNetwork.h"

volatile positionInteger canCurrentPosition;
volatile char canOrdrePropulsion[7];
volatile propStateType canEtatPosition;
volatile char canPropPatinage;

//int patinage;
void propulsionOrdre(char cmd, int data0, int data1 , int data2);
/**
 * @brief Initialisation de la communication avec la propulsion.
 * @details D�claration de la consommation et de la production des
 * diff�rents messages CAN li�s au fonctionnement de la propulsion.
 * D�fini la r�ception de la position et de l'�tat de propulsion.
 */
void propulsionInitCan(void) {
    CanDeclarationProduction(CN_PROPULSION, (void*)&canOrdrePropulsion, sizeof(canOrdrePropulsion));
    CanDeclarationConsommation(CO_PROP_POS, (void*)&canCurrentPosition, sizeof(canCurrentPosition));
    CanDeclarationConsommation(CO_PROP_STATUS, (void*)&canEtatPosition, 1);
    CanDeclarationConsommation(CO_PROP_PATINAGE, &canPropPatinage, sizeof(canPropPatinage));
}

void propulsionEnable(void) {
    propulsionOrdre(PROP_ENABLE,0,0,0);
}


void propulsionDisable(void){
    propulsionOrdre(PROP_DISABLE,0,0,0);
}


/**
 * @brief Renvoie la position actuelle mesur�e par l'odom�trie 
 * @Details Le canCurrentPosition est mis � jour d�s que la propulsion le met sur le CAN
 */
positionInteger propulsionGetPosition(void) {
	positionInteger tmp;
	
	ACTIVATE_CAN_INTERRUPTS = 0;
    tmp = canCurrentPosition;
	ACTIVATE_CAN_INTERRUPTS = 1;
	return(tmp);
}

/**
 * @brief Renvoie l'�tat du module propulsion
 * @Details Le canEtatPosition est mis � jour d�s que la propulsion le met sur le CAN
 */
propStateType propulsionGetStatus(void) {
    propStateType tmp;
    
    ACTIVATE_CAN_INTERRUPTS = 0;
    tmp = canEtatPosition;
    ACTIVATE_CAN_INTERRUPTS = 1;
    return(tmp);
}

/**
 * @brief Ordre CAN g�n�rique � l'odom�trie (fonction interne)
 */
void propulsionOrdre(char cmd, int data0, int data1 , int data2) {
    canOrdrePropulsion[0] = cmd;
    canOrdrePropulsion[1] = (unsigned char)( data0 & 0x00FF );
    canOrdrePropulsion[2] = (unsigned char)( (data0 >> 8) & 0x00FF );
    canOrdrePropulsion[3] = (unsigned char)( data1 & 0x00FF );
    canOrdrePropulsion[4] = (unsigned char)( (data1>> 8) & 0x00FF );
    canOrdrePropulsion[5] = (unsigned char)( data2 & 0x00FF );
    canOrdrePropulsion[6] = (unsigned char)( (data2>> 8) & 0x00FF );
    CanEnvoiProduction(canOrdrePropulsion);
}


/**
 * @brief Fixe la position actuelle par un envoi CAN � la propulsion
 */
void propulsionSetPosition(positionInteger pos) {
//    int test = 0;
//    positionInteger tmp;

    propulsionOrdre(PROP_SET_POS, pos.x, pos.y , pos.alpha);
//    // on attend que l'ordre soit ex�cut�, en v�rifiant la position envoy�e par la prop
//    msTimerStart();
//    while( (test == 0) && (msTimerGet() < 100) ) {
//        tmp = propulsionGetPosition();
//        if( (pos.x == tmp.x) && (pos.y == tmp.y) && (pos.alpha == tmp.alpha) ) {
//            test = 1;
//        }
//    }
//    msTimerStop();
}


/**
 * @brief ordonne � la propulsion d'effectuer une translation � vitesse et
 * acceleration donn�es
 */
void propulsionTranslation(int acc, int vit, int dist) {
    propulsionOrdre(PROP_TRANSLATION, acc, vit , dist);	// on envoie l'ordre
	while(propulsionGetStatus() == STANDING);			// on attend que le mouvement ait commenc�
}

/**
 * @brief ordonne � la propulsion d'effectuer une rotation � vitesse et 
 * acceleration donn�es
 */
void propulsionRotation(int acc, int vit, int dist) {
    propulsionOrdre(PROP_ROTATION, acc, vit , dist);	// on envoie l'ordre
	while(propulsionGetStatus() == STANDING);			// on attend que le mouvement ait commenc�
}


void propulsionGotoxy(positionInteger pos) {
    propulsionOrdre(PROP_GOTO_XY, pos.x, pos.y, 0);
}


void propulsionGotoxyalpha(positionInteger pos) {
    propulsionOrdre(PROP_GOTO_XYALPHA, pos.x, pos.y, pos.alpha);
}


void propulsionStopNow(void){
    propulsionOrdre(PROP_STOP_NOW, 0,0,0);
}
