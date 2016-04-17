/*!
 * @file main.c
 * @author Olivier Desenfans - Michel Osée
 * @author bULBot 2015
 * @brief Gestion de la carte-maitre du robot.
 * @details Implémentation de l'intelligence haut niveau du robot
 * et coordination des différents modules.
 */


#include "dataTypes.h"
#include "gestionActions.h"
#include "basicActions.h"
#include "gestionPropulsion.h"
#include "spio.h"
#include "adc.h"
#include "timer.h"
#include "Globals.h"
#include "sharp.h"
#include "Scheduler.h"
#include "../libdspic/uart.h"
#include "../Can/CanNetwork.h"
#include "../libdspic/CanDspic.h"
#include "../libdspic/clock.h"
#include <libpic30.h>
#include <math.h>
#include "BLE.h"
#include "stepper.h"


_FWDT(FWDTEN_OFF) // on désactive le Watchdog
_FOSCSEL(FNOSC_FRC);
// enables clock switching and configure the primary oscillator for a 10MHz crystal
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_XT);
_FGS(GSS_OFF & //
	GCP_OFF & //
	GWRP_OFF); //
_FICD(ICS_PGD1); // <-- à  changer selon les cas


#define LOGI_SHARP_MESURE   (CN_LOGIQUE*0x10)

//////////////////////////////
//	Constantes				//
//////////////////////////////
const positionInteger positionInitialeJaune = {500, 1000, 0};        //! Position initiale pour l'équipe jaune.
const positionInteger positionInitialeVert = {2800 , 1300, 1800};  //! Position initiale pour l'équipe verte.1800
obstacleType AllyObs = {0 , 0, 0};

//////////////////////////////////
//  PROTOTYPES DES FONCTIONS    //
//////////////////////////////////
actionType choixAction(void);
actionType gestionErreur(actionType currentAction);


//////////////////////////////
//	Variables globales 		//
//////////////////////////////
unsigned int nbActions;             //<! définit le nombre total d'actions à exécuter
actionType *ordreActions;           //<! Tableau d'actions à effectuer dans l'ordre
match matchStatus = INIT;           //<! Etat du match.
int mesureSharp;                    //<! Mesure du Sharp

//Denis
detectionSharpType detectionSharpMain;
int STOPED_OBS=0;
infoActionType infoAction;
int EnvoiSharpActif = 0;
propIsObstacleType obstacle;
positionInteger oldPos;


const positionInteger Zero = {0, 0, 0};
positionInteger RESET ={-1,-1,-1};
int Repeat;
int newTache = 1;




/**
 * @fn int main(void)
 * @brief Fonction main du programme.
 */
int main(void) {
	// Déclaration des variables locales.
    match oldMatchStatus = OVER;            // Etat précédent duu match (pour les actions d'entrée)
	actionType choix;                  //<! Pointeur vers l'action en cours.
	positionInteger positionInitiale;       //<! Position initiale du robot sur la table
	team equipe;                            //<! Equipe actuelle.

//    propIsObstacleType test;
	pllConfig();					// Configuration de la PLL de l'horloge interne pour tourner à 40MIPS
	assignSPIO();					// Assignation des pins du dSPIC.
    
    timerSetup(5, 50);
    TMR5 = 0;
	// Initialisation du CAN
	CanInitialisation(CN_LOGIQUE);
	propulsionInitCan();
    CanDeclarationProduction(CN_LOGIQUE*0x10, &matchStatus, sizeof(matchStatus));
    CanDeclarationProduction(CN_LOGIQUE*0x10, &mesureSharp, sizeof(mesureSharp));
	ACTIVATE_CAN_INTERRUPTS = 1;
	//msTimerInit();                  // Initialisation du timer des millisecondes, n'est utilisé que pour waitXms (TODO: enlever en mêm temps que waitXms)

	while(1) {
		if (getMatchTimerFlag()) {
			matchStatus = OVER;
		}
		// Machine d'état de la logique
	switch (matchStatus) {
            case INIT:                  // Etat initial, phase précédant le match
                if (matchStatus != oldMatchStatus) {
                    oldMatchStatus = matchStatus;
                    CanEnvoiProduction(&matchStatus);
                    matchTimerInit();				// Initialisation du timer de match
                    initADC();
                    //initBLE();
                    initStepper();
                    if (EnvoiSharpActif == 1){
                        timerStart(5);
                        IEC1bits.T5IE = 1;
                    }  
                    sendPosition(positionInitiale);
                }
                if (!GOUPILLE_OTEE) {
                    matchStatus = PRE_MATCH;
                }
                break;
            case PRE_MATCH:             // Le robot attend le début du match. On peut encore choisir la couleur

                if (matchStatus != oldMatchStatus) {
                    oldMatchStatus = matchStatus;
                    CanEnvoiProduction(&matchStatus);
                }

				//if (BOUTON_EQUIPE == JAUNE) {
				//	equipe = JAUNE;
				//	positionInitiale = positionInitialeJaune;
					//ordreActions = actionsJaune;
				//} else {
					equipe = VERT;
					positionInitiale = positionInitialeVert;
				//}
				// TRANSITIONS
                   // réparer la goupille ENCORE
			//	if (GOUPILLE_OTEE) {       
                    // si la goupille est retirée, le match commence
					__delay_ms(1000);
                    StartMatchTimer();                                                      // on lance le timer de match (90s))
                    propulsionEnable();  // on active la propulsion
                   // msTimerStart();
                    
                    while(propulsionGetStatus() != STANDING);                               // on attend que l'ordre soit exécuté
					propulsionSetPosition(positionInitiale);                                // On initialise la position de la propulsion
                    while(!compareXYAlpha(propulsionGetPosition(), positionInitiale));     // on attend que l'ordre soit exécuté
                    choix = choixAction();                                                  // Sélection de la prochaine action => c'est dans cette fonction qu'il faut mettre de l'IA
					matchStatus = ONGOING;

			//	}
				break;
            case ONGOING: // Le match est lancé

                if (matchStatus != oldMatchStatus) {
                    oldMatchStatus = matchStatus;
                    CanEnvoiProduction(&matchStatus);
                }
                
                    if (getTaskRequest()){
                        setTaskRequest(0);
                        Scheduler(getPetitRobot());
                }
                infoAction = choix.ptr2Fct(choix.dest,choix.methode);
                

                
                if (canReceivedOrderFlag) {
                    canReceivedOrderFlag = 0;
                    if (canReceivedCommand == LOGI_SHARP_MESURE) {
                        EnvoiSharpActif = canReceivedData[0];
                   }
                }

                switch(infoAction.statut) {
                    case ACTION_PAS_COMMENCEE:
                    case ACTION_EN_COURS:
                        
                        // on attend la fin de l'action
                        break;
                    case ACTION_FINIE:
                    case ACTION_REMISE:
                        sendPosition(propulsionGetPosition());
                        choix = choixAction();                  // Sélection de la prochaine action => c'est dans cette fonction qu'il faut mettre de l'IA
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
        
        
   }

    return 0;
}


actionType choixAction() {
      // on arrète l'intérruption quand on choisi une action
    actionType TacheChoisie;
      
      TacheChoisie= Scheduler(getGrandRobot());//ActionPossible[indiceTache].action[indiceAction];
      
	return(TacheChoisie);
}



actionType gestionErreur(actionType currentAction) {

    actionType nextAction={Zero,0, noActionFct};
    infoActionType infoAction;
    positionInteger curPos = propulsionGetPosition();
    infoAction = currentAction.ptr2Fct(currentAction.dest,currentAction.methode);
    trajectoryBasicAction(RESET); 
    switch(infoAction.typeEtapeEnCours) {
        case ACTION_TRAJECTOIRE:
            switch (propulsionGetStatus()) {
                case STANDING:
                case TRAJECTORY:
                case RELATIVE_MOVE:
              if (STOPED_OBS == 1){
                setGrandRobotObs(1);
                nextAction = choixAction();
            }
            else if (STOPED_OBS == 0){
                nextAction = choixAction();
            }    
            else if (STOPED_OBS == 2){
                nextAction = choixAction();
            }                  

                    break;
                case TRAJ_NO_WAY:                       // si on n'a pas trouvé de chemin, c'est qu'il y a un obstacle dans le chemin
                case TRAJ_START_OBS:
                case TRAJ_START_OUT:            // pour les autres cas d'erreur trajectoire, pas encore de solution implémentée               
                case TRAJ_END_OUT:
                case TRAJ_END_OBS:
                    propulsionEnable();
                    while(propulsionGetStatus() != STANDING);
                    curPos.x = (int) curPos.x-100*cos(DEG2RAD(curPos.alpha));             // On sort du carré de l'obstacle pour redémarrer
                    curPos.y = (int) curPos.y-100*sin(DEG2RAD(curPos.alpha));
                    nextAction.ptr2Fct = seDeplacer;
                    nextAction.methode = 0;
                    nextAction.dest = curPos ;
                    Repeat=1;
                    break;
                   
                default:
                    break;
            }
            
            break;
        case ACTION_PROPULSION:
            break;
        case ACTION_ACTIONNEURS:
            if (STOPED_OBS != 0){
                nextAction = currentAction;
            }

    default:
        nextAction = currentAction;
        break;
    }
    return(nextAction);
}


// INTERRUPT SHARP
void __attribute__((interrupt, auto_psv)) _T5Interrupt(void) {
	IFS1bits.T5IF = 0;		// Réinitialisation du flag
    //TMR5 = 0;
    actionType stop;
    
    if (STOPED_OBS == 0){ 
        detectionSharpMain = detectionObstacleSharp();
        if (detectionSharpMain.isObstacleDetected ){          
            
            propulsionIsObstacle(detectionSharpMain.obstacleInfo); 
            obstacle = propulsionGetObstacle();
            if (detectionSharpMain.obstacleInfo.x > 2950 || detectionSharpMain.obstacleInfo.y > 1950){ // ne doit pas rester c'est la propulsion qui devrait le faire
                obstacle = PROP_IS_NO_OBSTACLE;
            }
          if (obstacle == PROP_IS_NO_OBSTACLE && matchStatus != ERROR ){
                stop.ptr2Fct = StopNowFct;
                stop.dest = RESET;
                stop.methode = 0;
                infoAction = stop.ptr2Fct(stop.dest,stop.methode);
                matchStatus = ERROR;
                while (ABS(detectionSharpMain.obstacleInfo.x) < 1.0){
                detectionSharpMain = detectionObstacleSharp();
                }
                propulsionAddObstacle(detectionSharpMain.obstacleInfo);
                STOPED_OBS = 1;
                oldPos = propulsionGetPosition();
        }
    }
  }
    else if (STOPED_OBS == 2){ // on désactive la vision tant qu'il a pas fait 45°
        positionInteger pos = propulsionGetPosition();
        if (400 < (compareAlpha(pos,oldPos))){
            STOPED_OBS =0;
            detectionObstacleSharp(); // vide le buffer de l'ADC (sinon on aura les anciennes coordonées à la prochaine vision)
            detectionObstacleSharp();
        }
    }
}

