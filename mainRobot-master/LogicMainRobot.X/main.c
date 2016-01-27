/*!
 * @file main.c
 * @author Olivier Desenfans - Michel Os�e
 * @author bULBot 2015
 * @brief Gestion de la carte-maitre du robot.
 * @details Impl�mentation de l'intelligence haut niveau du robot
 * et coordination des diff�rents modules.
 */


#include "dataTypes.h"
#include "gestionActions.h"
#include "basicActions.h"
#include "gestionPropulsion.h"
#include "spio.h"
#include "timer.h"
#include "sharp.h"
#include "../Can/CanNetwork.h"
#include "../libdspic/CanDspic.h"
#include "../libdspic/clock.h"
#include "../libdspic/servo.h"
#include <math.h>


_FWDT(FWDTEN_OFF) // on d�sactive le Watchdog
_FOSCSEL(FNOSC_FRC);
// enables clock switching and configure the primary oscillator for a 10MHz crystal
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_XT);
_FGS(GSS_OFF & //
	GCP_OFF & //
	GWRP_OFF); //
_FICD(ICS_PGD1); // <-- � changer selon les cas


#define LOGI_SHARP_MESURE   (CN_LOGIQUE*0x10)

//////////////////////////////
//	Constantes				//
//////////////////////////////
const positionInteger positionInitialeJaune = {500, 1000, 0};        //! Position initiale pour l'�quipe jaune.
const positionInteger positionInitialeVert = {2500 , 1000, 1800};  //! Position initiale pour l'�quipe verte.


//////////////////////////////////
//  PROTOTYPES DES FONCTIONS    //
//////////////////////////////////
actionType choixAction(void);
actionType gestionErreur(actionType currentAction);


//////////////////////////////
//	Variables globales 		//
//////////////////////////////
unsigned int nbActions;             //<! d�finit le nombre total d'actions � ex�cuter
actionType *ordreActions;           //<! Tableau d'actions � effectuer dans l'ordre
match matchStatus = INIT;           //<! Etat du match.
int mesureSharp;                    //<! Mesure du Sharp


/**
 * @fn int main(void)
 * @brief Fonction main du programme.
 */
int main(void) {
	// D�claration des variables locales.
    match oldMatchStatus = OVER;            // Etat pr�c�dent duu match (pour les actions d'entr�e)
	actionType choix;                  //<! Pointeur vers l'action en cours.
	positionInteger positionInitiale;       //<! Position initiale du robot sur la table
	team equipe;                            //<! Equipe actuelle.
    infoActionType infoAction;
    int EnvoiSharpActif = 0, sharpLastTime = 0;

	pllConfig();					// Configuration de la PLL de l'horloge interne pour tourner � 40MIPS
	assignSPIO();					// Assignation des pins du dSPIC.
    servoInit(4, TIMER_2, 10);      // initialise 1 servo, utilisant le timer2 � 10ms
	// Initialisation du CAN
	CanInitialisation(CN_LOGIQUE);
	propulsionInitCan();
    CanDeclarationProduction(CN_LOGIQUE*0x10, &matchStatus, sizeof(matchStatus));
    CanDeclarationProduction(CN_LOGIQUE*0x10+1, &mesureSharp, sizeof(mesureSharp));
	ACTIVATE_CAN_INTERRUPTS = 1;
	msTimerInit();                  // Initialisation du timer des millisecondes, n'est utilis� que pour waitXms (TODO: enlever en m�m temps que waitXms)

	while(1) {
        LED = detectionObstacleSharp().isObstacleDetected;
		if (getMatchTimerFlag()) {
			matchStatus = OVER;
		}
		// Machine d'�tat de la logique
		switch (matchStatus) {
            case INIT:                  // Etat initial, phase pr�c�dant le match
                if (matchStatus != oldMatchStatus) {
                    oldMatchStatus = matchStatus;
                    CanEnvoiProduction(&matchStatus);
                    matchTimerInit();				// Initialisation du timer de match
                    initServoClap();
                }
                if (!GOUPILLE_OTEE) {
                    matchStatus = PRE_MATCH;
                }
                break;
			case PRE_MATCH:             // Le robot attend le d�but du match. On peut encore choisir la couleur
                if (matchStatus != oldMatchStatus) {
                    oldMatchStatus = matchStatus;
                    CanEnvoiProduction(&matchStatus);
                }
				if (BOUTON_EQUIPE == JAUNE) {
					equipe = JAUNE;
					positionInitiale = positionInitialeJaune;
					ordreActions = actionsJaune;
					nbActions = NB_ACTIONS_JAUNE;
				} else {
					equipe = VERT;
					positionInitiale = positionInitialeVert;
					ordreActions = actionsVert;
					nbActions = NB_ACTIONS_VERT;
				}
				// TRANSITIONS
				if (GOUPILLE_OTEE) {                                                        // si la goupille est retir�e, le match commence
					StartMatchTimer();                                                      // on lance le timer de match (90s))
                    propulsionEnable();                                                     // on active la propulsion
                    while(propulsionGetStatus() != STANDING);                               // on attend que l'ordre soit ex�cut�
					propulsionSetPosition(positionInitiale);                                // On initialise la position de la propulsion
					//waitXms(10);                                // on attend pour laisser le temps d'ex�cuter l'ordre TODO changer �a
                    while(!compareXYAlpha(propulsionGetPosition(), positionInitiale));     // on attend que l'ordre soit ex�cut�
                    choix = choixAction();                                                  // S�lection de la prochaine action => c'est dans cette fonction qu'il faut mettre de l'IA
					matchStatus = ONGOING;
				}
				break;
			case ONGOING: // Le match est lanc�
                if (matchStatus != oldMatchStatus) {
                    oldMatchStatus = matchStatus;
                    CanEnvoiProduction(&matchStatus);
                }
                infoAction = choix.ptr2Fct(choix.option);
                switch(infoAction.statut) {
                    case ACTION_PAS_COMMENCEE:
                    case ACTION_EN_COURS:
                        // on attend la fin de l'action
                        break;
                    case ACTION_FINIE:
                    case ACTION_REMISE:
                        choix = choixAction();                  // S�lection de la prochaine action => c'est dans cette fonction qu'il faut mettre de l'IA
                        break;
                    case ACTION_ERREUR:
                        matchStatus = ERROR;
                    default:
                        break;
                }
                break;
            case OVER:
                if (matchStatus != oldMatchStatus) {
                    oldMatchStatus = matchStatus;
                    CanEnvoiProduction(&matchStatus);
                    propulsionDisable();
                }
                if (!GOUPILLE_OTEE) {
                    matchStatus = INIT;
                }
                break;
            case ERROR:
                if (matchStatus != oldMatchStatus) {
                    oldMatchStatus = matchStatus;
                    CanEnvoiProduction(&matchStatus);
                }
                choix = gestionErreur(choix);
                matchStatus = ONGOING;
                break;
            default:
                matchStatus = ERROR;
                break;
        }
        
        if (canReceivedOrderFlag) {
            canReceivedOrderFlag = 0;
            if (canReceivedCommand == LOGI_SHARP_MESURE) {
                EnvoiSharpActif = canReceivedData[0];
            }
        }
        if (EnvoiSharpActif) {
            if (msTimerGet()-sharpLastTime >= 10) {
                sharpLastTime = msTimerGet();
                mesureSharp = sharpGetMesure();
                CanEnvoiProduction(&mesureSharp);
            }
        }
    }
    return 0;
}


actionType choixAction(void) {
    actionType actionChoisie;
	static unsigned int indiceAction = 0;					//<! d�finit l'action en cours
    
	if (indiceAction < nbActions) {
		indiceAction++;
	}
    actionChoisie = ordreActions[indiceAction-1];
	return(actionChoisie);
}



actionType gestionErreur(actionType currentAction) {
    actionType nextAction = {0, noActionFct};
    infoActionType infoAction;
    
    infoAction = currentAction.ptr2Fct(currentAction.option);
    switch(infoAction.typeEtapeEnCours) {
        case ACTION_TRAJECTOIRE:
            switch (propulsionGetStatus()) {
                case STANDING:                          // si on est � l'arr�t et en erreur, ce doit �tre un obstacle (ou un patinage)
                    if (!compareXYAlpha(propulsionGetPosition(), infoAction.position)) {   // on n'est pas au bon endroit a priori
                        nextAction = currentAction;                                         // on r�essaye TODO ajouter la gestion d'obstacle
                        nextAction.option = RESET_ACTION;
                    }
                    break;
                case TRAJ_NO_WAY:                       // si on n'a pas trouv� de chemin, c'est qu'il y a un obstacle dans le chemin
                    nextAction = choixAction();         // on passe la main 
                case TRAJ_START_OUT:            // pour les autres cas d'erreur trajectoire, pas encore de solution impl�ment�e
                case TRAJ_START_OBS:
                case TRAJ_END_OUT:
                case TRAJ_END_OBS:
                    break;
                default:
                    break;
            }
            break;
        case ACTION_PROPULSION:
            break;
        default:
            break;            
    }
    
    return(nextAction);
}
