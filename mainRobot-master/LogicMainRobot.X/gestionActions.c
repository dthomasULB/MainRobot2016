/**
 * @file gestionActions.c
 * @author bULBot 2011-2012
 * @author Olivier Desenfans
 * @brief Implémentation du module d'actions du robot.  
 */

#include "gestionActions.h"
#include "gestionPropulsion.h"
#include "basicActions.h"
#include "servoClap.h"
#include "../libdspic/servo.h"
#include "timer.h"

/*
const positionInteger clap1StartPos = {240, 150, -900};
const positionInteger clap1EndPos = {400, 150, 0};
const positionInteger clap2StartPos = {740, 150, -900};
const positionInteger clap2EndPos = {900, 150, 900};
*/

const positionInteger StartDuneV = {1800, 1800, 1800};
const positionInteger EndDuneV = {1200, 1800, -900};
const positionInteger ConstructZoneV = {1650, 1100, 1800};



infoActionType demarrageJauneFct(int option);
infoActionType demarrageVertFct(int option);
infoActionType retourVertFct(int option);
infoActionType TestArriere(int option);

actionType actionsJaune[NB_ACTIONS_JAUNE] = {{DEFAULT_ACTION, demarrageJauneFct}};
actionType actionsVert[NB_ACTIONS_VERT] = {{DEFAULT_ACTION, demarrageVertFct},{DEFAULT_ACTION, TestArriere},{DEFAULT_ACTION, retourVertFct}};


infoActionType demarrageJauneFct(int option) {
    static infoActionType infoAction = {ACTION_PAS_COMMENCEE, 0, 1, ACTION_TRAJECTOIRE, {1000, 1000, 0}};

    infoAction.statut = trajectoryBasicAction(infoAction.position);
    return (infoAction);
}

infoActionType TestArriere(int option) {
    static infoActionType infoAction = {ACTION_PAS_COMMENCEE, 0, 1, ACTION_TRAJECTOIRE, {0, 0, 0}};
    infoAction.statut = translationBasicAction(1000, 1000, -400);
    return (infoAction);
}

infoActionType demarrageVertFct(int option) {
    static infoActionType infoAction = {ACTION_PAS_COMMENCEE,   // statut de l'action
                                        0,                      // Etape actuelle
                                        2,                      // Nombre d'étapes
                                        ACTION_TRAJECTOIRE,     // Type de l'étape
                                        {0, 0, 0}};             // Destination de l'étape (si pertinent)
    actionStatutType trajStatut;
   enum {
        GOTO_DUNE = 0,
        GOTO_DUNE_END = 1,
        GOTO_CONSTR_ZONE=2,
        ACTION_RESET = 3,
        FINI = 255
    }etape = infoAction.etapeEnCours;
 
   switch (option) {
        case RESET_ACTION:
           etape = FINI;
           infoAction.statut = ACTION_FINIE;
            break;
 //      case DEFAULT_ACTION:
  //          etape = GOTO_DOOR1_START;
  //          infoAction.statut = ACTION_EN_COURS;
  //          option = ACTION_STARTED;
  //          break;
  //      ACTION_STATED:
  //          break;
        default:
            break;
    }
        switch (etape) {
            case GOTO_DUNE:
                trajStatut = trajectoryBasicAction(StartDuneV);
                 infoAction.statut = ACTION_EN_COURS;
                if (trajStatut == ACTION_FINIE) {
                    infoAction.etapeEnCours = GOTO_DUNE_END;
                }
                else  if (trajStatut == ACTION_ERREUR) {
                    infoAction.statut = ACTION_ERREUR;
                    etape = FINI;
                }
                break;
            case GOTO_DUNE_END:
                trajStatut = trajectoryBasicAction(EndDuneV);
                if (trajStatut == ACTION_FINIE) {
                    infoAction.etapeEnCours = GOTO_DUNE_END;
                }
                else  if (trajStatut == ACTION_ERREUR) {
                    infoAction.statut = ACTION_ERREUR;
                    etape = FINI;
                }
            case GOTO_CONSTR_ZONE:
                trajStatut = trajectoryBasicAction(ConstructZoneV);
                if (trajStatut == ACTION_FINIE) {
                    infoAction.statut = ACTION_FINIE;
                    etape = FINI;
                }
                if (trajStatut == ACTION_ERREUR) {
                    infoAction.statut = ACTION_REMISE;
                    etape = FINI;
                }
                break;
            case FINI:
                etape = 0;
                infoAction.statut = ACTION_FINIE;
            default:
                break;
}
    return (infoAction);
}

infoActionType retourVertFct(int option) {
  static infoActionType infoAction = {ACTION_PAS_COMMENCEE, 0, 1, ACTION_TRAJECTOIRE, {2500, 1700, 1800}};
   infoAction.statut = trajectoryBasicAction(infoAction.position);
    return (infoAction);
}