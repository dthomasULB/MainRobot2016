#include "Configware.h"				//<! Contient la configuration globale du PIC : clock, GPIO
#include "odoLibre.h"
#include "Motors.h"
#include "Regulator.h"
#include "consigne.h"
#include "mouvements.h"
#include "generTraj.h"
#include "canFnc.h"
#include "../libdspic/timers.h"
#include "../libdspic/CanDspic.h"
#include <xc.h>
#include <math.h>


#define DBG_PIN0	(LATCbits.LATC5)            // pattes de debug pour mesurer des temps d'exécution
#define DBG_PIN1	(LATBbits.LATB5)


// variables ISR
volatile int isrRegFlag;                        //!< flag indiuant si le régulateur (et le générateur de consigne) sont actifs
volatile regType patinageFlag = NO_PATINAGE;

void __attribute__((interrupt, auto_psv)) _T1Interrupt(void){
    IFS0bits.T1IF = 0;
    DBG_PIN1 = 1;
    // on envoie la position AVANT le calcul de l'odométrie pour être certain qu'un PROP_SET_POSITION soit suivi par l'envoi de
    canSendPos(odoGetAbsPos());                   // la nouvelle position, ce qui sert d'acquittement à l'ordre. (on est en retard de 10ms, mais c'est pas grave))
    calculeOdometrie();                         // on calcule la position absolue mesurée avec les encodeurs libres
    if (isrRegFlag) {
        csgCompute();                           // on calcule la nouvelle consigne
        patinageFlag = regCompute();            // on exécute le régulateur de position+vitesse
    }
    DBG_PIN1 = 0;
}



///////////////////////////////////////////////////////////////////////////////////////////////////
//	MAIN FUNCTION
///////////////////////////////////////////////////////////////////////////////////////////////////

int main(void) {
    propStateType state = DISABLED;         //!< Etat de la propulsion
    propStateType oldState = DISABLED;      //!< dernière valeur de l'état envoyée sur le CAN
    csgStatusType csgStatus;                //!< Statut du générateur de consigne : indique si le mouvement est en cours ou fini
    relativePosType nomVel;                 //!< vitesses nominales du robot
    relativePosType nomAcc;                 //!< Accélérations nominales du robot
    absolutePosType curPos;                 //!< Position actuelle du robot
    // Variables pour les commandes de mouevements relatifs
    translationParamType translationParam;  //!< contient les paramètres de la translation à effectuer
    rotationParamType rotationParam;        //!< contient les paramètres de la rotation à effectuer
    // Variables utilisées pour les commandes de trajectoires
    absolutePosType finalPos;               //!< position finale du mouvement
    relativePosType segment;                //!< contient les paramètres du prochain segment à parcourir
    absolutePosType path[15];               //!< Contient les points de passage de la trajectoire
    int patinageEnable = 1;
    int curSeg, nbSeg;
    enum {
        XY,
        XYALPHA
    } trajStyle;                             //!< Type de trajectoire à effctuer
    enum {
        ROTATION,
        TRANSLATION,
        LAST_ROTATION
    } trajectorySubState,                   //!< sous-état de l'état TRAJECTORY
      oldTrajectorySubState;


    pllConfig();                // configure l'horloge de PIC
    canPinAssign();             // assigne les pattes du CAN
    TRISCbits.TRISC5 = 0;       // configure les pins de debug en sortie
    TRISBbits.TRISB5 = 0;
    nomVel.l = 1;               // m/s
    nomVel.r = PI/2;            // rad/s
    nomAcc.l = 1;               // m/s^2
    nomAcc.r = PI/2;            // rad/s^2
    canPropInit();
    csgInit(nomVel, nomAcc);
    trajInit();
    regInit();                  // initialise les régulateurs de vitesses et positions
    odoInit();                  // initialise les périphériques QEI pour la mesure des encodeurs
    motorsInit();               // initialise les périphériques PWM pour le contrôle des moteurs
    isrRegFlag = 0;
    timerSetup(TIMER_1, 10);                    // Configuration du timer1 pour avoir une base de temps de 10ms
    _T1IE = 1;
    //timerInterrupt(TIMER_1, &propInterrupt);    // configuration de l'ISR
    timerStart(TIMER_1);
    DBG_PIN0 = 0;
    while (1) {
        switch (state) {
            case DISABLED:
                if (state != oldState) {            // Actions d'entrée
                    oldState = state;
                    canSendState(state);
                    motorsDisable();
                }
                if (canReceivedOrderFlag) {         // Traitement des évènements CAN
                    switch (canReceivedCommand) {
                        case PROP_ENABLE:
                            state = STANDING;
                            break;
                        case PROP_TEST:
                            state = TEST;
                            break;
                        case PROP_SET_POS:
                            curPos = canOrderGetPos();
                            disableIsrTimer1();
                            odoSetAbsPos(curPos);
                            csgSetFinalPos(odoGetRelPos());
                            enableIsrTimer1();
                            break;
                        case PROP_PATINAGE_ON_OFF:
                            patinageEnable = canGetPatinageEnable();
                            break;
                        case PROP_ADD_OBSTACLE:
                            trajAddObstacle(canOrderGetObstacle());
                            break;
                        case PROP_REMOVE_OBSTACLE:
                            trajRemoveObstacle(canOrderGetObstacle());
                            break;
                        case PROP_IS_OBSTACLE_IN_MAP:
                            canSendIsObstacle(trajIsObstacleInMap(canOrderGetPos()));
                            break;
                    }
                    canReceivedOrderFlag = 0;
                }
                break;
            case STANDING:
                if (state != oldState) {                  // Actions d'entrée
                    oldState = state;
                    canSendState(state);
                    disableIsrTimer1();
                    motorsEnable();
                    isrRegFlag = 1;
                    enableIsrTimer1();
                }
                if (canReceivedOrderFlag) {
                    switch (canReceivedCommand) {   // Traitement des évènements CAN
                        case PROP_DISABLE:
                            state = DISABLED;
                            break;
                        case PROP_TRANSLATION:
                            translationParam = canGetTranslation();
                            disableIsrTimer1();
                            addTranslation(translationParam);
                            enableIsrTimer1();
                            state = RELATIVE_MOVE;
                            break;
                        case PROP_ROTATION:
                            rotationParam = canGetRotation();
                            disableIsrTimer1();
                            addRotation(rotationParam);
                            enableIsrTimer1();
                            state = RELATIVE_MOVE;
                            break;
                        case PROP_GOTO_XY:
                        case PROP_GOTO_XYALPHA:
                            DBG_PIN0 = 1;
                            disableIsrTimer1();
                            curPos = odoGetAbsPos();
                            enableIsrTimer1();
                            finalPos = canOrderGetPos();
                            nbSeg = findTrajectoire(curPos, finalPos, path);
                            if (nbSeg < 0) {
                                state = nbSeg;
                            } else if (nbSeg == 0) {
                                state = STANDING;
                            } else {
                                if (canReceivedCommand == PROP_GOTO_XY) {
                                    trajStyle = XY;
                                } else {
                                    trajStyle = XYALPHA;
                                }
                                translationParam.acc = nomAcc.l;
                                translationParam.vel = nomVel.l;
                                rotationParam.acc = nomAcc.r;
                                rotationParam.vel = nomVel.r;
                                state = TRAJECTORY;
                            }
                            DBG_PIN0 = 0;
                            break;
                        case PROP_SET_POS:
                            curPos = canOrderGetPos();
                            disableIsrTimer1();
                            odoSetAbsPos(curPos);
                            csgSetFinalPos(odoGetRelPos());
                                enableIsrTimer1();
                            break;
                        case PROP_PATINAGE_ON_OFF:
                            patinageEnable = canGetPatinageEnable();
                            break;
                        case PROP_ADD_OBSTACLE:
                            trajAddObstacle(canOrderGetObstacle());
                            break;
                        case PROP_REMOVE_OBSTACLE:
                            trajRemoveObstacle(canOrderGetObstacle());
                            break;
                        case PROP_IS_OBSTACLE_IN_MAP:
                            canSendIsObstacle(trajIsObstacleInMap(canOrderGetPos()));
                            break;
                    }
                    canReceivedOrderFlag = 0;
                }
                break;
            case RELATIVE_MOVE:
                if (state != oldState) {                    // Actions d'entrée
                    oldState = state;
                    canSendState(state);
                }
                if ((patinageEnable == 1) && (patinageFlag == PATINAGE_DETECTED)) {
                    csgSetFinalPos(csgGetPos());
                    canSendPatinageFlag();
                    state = STANDING;
                }
                if (canReceivedOrderFlag) {
                    switch (canReceivedCommand) {
                        case PROP_DISABLE:
                            state = DISABLED;
                            break;
                        case PROP_STOP_NOW:
                            disableIsrTimer1();
                            stopNow();
                            enableIsrTimer1();
                            break;
                        default:
                            break;
                        case PROP_SET_POS:
                            curPos = canOrderGetPos();
                            disableIsrTimer1();
                            odoSetAbsPos(curPos);
                            csgSetFinalPos(odoGetRelPos());
                                enableIsrTimer1();
                            break;
                        case PROP_PATINAGE_ON_OFF:
                            patinageEnable = canGetPatinageEnable();
                            break;
                    }
                    canReceivedOrderFlag = 0;
                }
                disableIsrTimer1();
                csgStatus = csgGetState();
                enableIsrTimer1();
                if (csgStatus == CSG_STANDING) { // Si la consigne a atteint sa destination
                    state = STANDING; // on passe dans l'état STANDING
                }
                break;
            case TRAJECTORY:
                if (state != oldState) {                    // Actions d'entrée
                    oldState = state;
                    canSendState(state);
                    curSeg = 0;
                    trajectorySubState = ROTATION;
                    oldTrajectorySubState = LAST_ROTATION;
                }
                if ((patinageEnable == 1) && (patinageFlag == PATINAGE_DETECTED)) {
                    csgSetFinalPos(csgGetPos());
                    canSendPatinageFlag();
                    state = STANDING;
                }
                if (canReceivedOrderFlag) {
                    switch (canReceivedCommand) {
                        case PROP_DISABLE:
                            state = DISABLED;
                            break;
                        case PROP_STOP_NOW:
                            disableIsrTimer1();
                            stopNow();
                            enableIsrTimer1();
                            state = RELATIVE_MOVE;        // cela devient un mouvement relatif
                            break;
                        case PROP_PATINAGE_ON_OFF:
                            patinageEnable = canGetPatinageEnable();
                            break;
                    }
                    canReceivedOrderFlag = 0;
                }
                switch (trajectorySubState) {
                    case ROTATION:
                        if (trajectorySubState != oldTrajectorySubState) {
                            oldTrajectorySubState = trajectorySubState;
                            disableIsrTimer1();
                            curPos = odoGetAbsPos();
                            enableIsrTimer1();
                            segment = calcSegment(curPos, path[curSeg+1]);
                            if (segment.r != 0) {       // si il y a une rotation à faire
                                rotationParam.angle = segment.r;
                                disableIsrTimer1();
                                addRotation(rotationParam);
                                enableIsrTimer1();
                            } else {
                                trajectorySubState = TRANSLATION;
                            }
                        }
                        if (csgGetState() == CSG_STANDING) {
                            trajectorySubState = TRANSLATION;
                        }
                        break;
                    case TRANSLATION:
                        if (trajectorySubState != oldTrajectorySubState) {
                            oldTrajectorySubState = trajectorySubState;
                            translationParam.length = segment.l;
                            disableIsrTimer1();
                            addTranslation(translationParam);
                            enableIsrTimer1();
                        }
                        if (csgGetState() == CSG_STANDING ) {
                            curSeg++;
                            if (curSeg < nbSeg) { // si il reste des segments à parcourir
                                trajectorySubState = ROTATION;
                            } else if (trajStyle == XY) {
                                state = STANDING;
                            } else {
                                trajectorySubState = LAST_ROTATION;
                            }
                        }
                        break;
                    case LAST_ROTATION:
                        if (trajectorySubState != oldTrajectorySubState) {
                            oldTrajectorySubState = trajectorySubState;
                            disableIsrTimer1();
                            curPos = odoGetAbsPos();
                            enableIsrTimer1();
                            rotationParam.angle = satureAngle(finalPos.alpha - curPos.alpha);
                            if (rotationParam.angle != 0) {
                                disableIsrTimer1();
                                addRotation(rotationParam);
                                enableIsrTimer1();
                            }
                        }
                        if (csgGetState() == CSG_STANDING) {
                            state = STANDING;
                        }
                        break;
                    default:
                        disableIsrTimer1();
                        stopNow();
                        enableIsrTimer1();
                        state = RELATIVE_MOVE; // cela devient un mouvement relatif
                           break;
                }
                break;
            case TEST:
                if (state != oldState) {                    // Actions d'entrée
                    oldState = state;
                    canSendState(state);
                }
                if (canReceivedOrderFlag) {
                    switch (canReceivedCommand) {
                        case PROP_DISABLE:
                            state = DISABLED;
                            break;
                        case PROP_SET_DUTY_CYCLE:
                            disableIsrTimer1();
                            motorsSetSpeed(canReceivedData[0] / 100.0, canReceivedData[1] / 100.0);
                            enableIsrTimer1();
                            break;
                        case PROP_SET_POS:
                            curPos = canOrderGetPos();
                            disableIsrTimer1();
                            odoSetAbsPos(curPos);
                            csgSetFinalPos(odoGetRelPos());
                                enableIsrTimer1();
                            break;
                        case PROP_PATINAGE_ON_OFF:
                            patinageEnable = canGetPatinageEnable();
                            break;
                    }
                    canReceivedOrderFlag = 0;
                }
                break;
            case TRAJ_START_OUT:
            case TRAJ_END_OUT:
            case TRAJ_START_OBS:
            case TRAJ_END_OBS:
            case TRAJ_NO_WAY:
                if (state != oldState) {                    // Actions d'entrée
                    oldState = state;
                    canSendState(state);
                }
                if ( (canReceivedOrderFlag) && (canReceivedCommand == PROP_ENABLE)) {
                    state = STANDING;
                }
                break;

            default:
                break;
        }
    }
    return (1);
}
