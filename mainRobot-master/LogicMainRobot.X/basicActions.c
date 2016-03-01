#include "Globals.h"
#include "../Can/CanTypes.h"
#include "dataTypes.h"
#include "gestionPropulsion.h"
#include <math.h>

#define NO_ACTION       (0)
#define INIT_ACTION     (1)
#define RESET_ACTION    (2)
#define EXECUTE         (3)

infoActionType noActionFct(int option) {
    infoActionType infoAction = {ACTION_PAS_COMMENCEE, 0, 0, ACTION_AUCUNE,
        {0, 0, 0}};

    return (infoAction);
}


int compareXYAlpha(positionInteger pos1, positionInteger pos2) {
    int tmp;
    tmp = pos1.alpha - pos2.alpha;
    while (tmp > 1800)  tmp -= 3600;
    while(tmp < -1800)  tmp += 3600;
    return ((ABS(pos1.x - pos2.x) < 20) && (ABS(pos1.y - pos2.y) < 20) && (ABS(tmp) < 30));  // 20E-3 = 1°
}

int compareXY(positionInteger pos1, positionInteger pos2) {
    return ((ABS(pos1.x - pos2.x) < 20) && (ABS(pos1.y - pos2.y) < 20));
}
infoActionType StopNowFct(){
    infoActionType infoAction = {ACTION_PAS_COMMENCEE, 0, 0, ACTION_TRAJECTOIRE,{0, 0, 0}};
        static enum {
        ENVOI_ORDRE = 0,
        ATTENTE_FIN_MOUVEMENT = 2,
        FINI = 3
    } etape = ENVOI_ORDRE;

    switch(etape){
        case ENVOI_ORDRE:
            propulsionStopNow();
            etape = ATTENTE_FIN_MOUVEMENT;
            break;
        case ATTENTE_FIN_MOUVEMENT:
        if (propulsionGetStatus() == STANDING) {    // on attend que le mouvement soit fini
                etape = FINI;
           }
            break;
        case FINI:
            infoAction.statut = ACTION_FINIE;
            break;
    }
    return(infoAction);
}

actionStatutType trajectoryBasicAction(positionInteger destination) {
    actionStatutType statutAction = ACTION_EN_COURS;
    positionInteger RESET = {-1,-1,-1};
    static enum {
        ENVOI_ORDRE = 0,
        ATTENTE_ACQUITTEMENT = 1,
        ATTENTE_FIN_MOUVEMENT = 2,
        FINI = 3
    } etape = ENVOI_ORDRE;

//    while (etape != FINI) {
    if (destination.x == RESET.x){   // RESET pour remettre la fonction à 0 si on a été coupé par un SHARP
        etape = ENVOI_ORDRE;
        statutAction = ACTION_FINIE;
    }
    else{
        switch (etape) {
            case ENVOI_ORDRE:       // 1: envoyer l'ordre à la propulsion
                propulsionGotoxyalpha(destination);     // on envoie l'ordre de trajectoire
                etape = ATTENTE_ACQUITTEMENT;
                break;
            case ATTENTE_ACQUITTEMENT:  // 2: attendre le changement de statut de la propulsion
                switch (propulsionGetStatus()) {
                    case STANDING: // le mouvement n'est pas encore commencé
                        break; // on attend
                    case TRAJECTORY: // le mouvement a commencé
                        etape = ATTENTE_FIN_MOUVEMENT; // on passe à l'état suivant
                        break;
                    case TRAJ_START_OUT: // la trajectoire n'a pas trouvé de chemin valable
                    case TRAJ_END_OUT:
                    case TRAJ_START_OBS:
                    case TRAJ_END_OBS:
                    case TRAJ_NO_WAY:
                        etape = FINI;                   // on termine l'action
                        statutAction = ACTION_ERREUR;   // avec un statut d'erreur
                        break;
                    case TEST: // ne doit jamais arriver en match, on repasse la prop en STANDING
                        propulsionDisable();
                        while (propulsionGetStatus() != DISABLED); // on attend que l'ordre soit exécuté
                        propulsionEnable();
                        while (propulsionGetStatus() != STANDING); // on attend que l'ordre soit exécuté
                        etape = ENVOI_ORDRE; // on recommence l'action
                        break;
                    default: // ne doit jamais arriver
                        etape = FINI;                   // on termine l'action
                        statutAction = ACTION_ERREUR;   // on termine l'action avec un statut d'erreur
                        break;
                }
                break;
            case ATTENTE_FIN_MOUVEMENT:
                if (propulsionGetStatus() == STANDING) {    // on attend que le mouvement soit fini
                    etape = FINI;
                    if (!compareXYAlpha(destination, propulsionGetPosition())) {    // on vérifie qu'on est au bon endroit
                        statutAction = ACTION_ERREUR;       // sinon, il y a eu un problème (patinage ou obstacle)
                    }
                }
                break;
            case FINI:
                if (!statutAction==ACTION_ERREUR){
                statutAction = ACTION_FINIE;
                }
                etape = ENVOI_ORDRE;
                break;
            default:
                etape = ATTENTE_FIN_MOUVEMENT;
                break;
        }
    }
//    }
    return (statutAction);
}

actionStatutType translationBasicAction(int acc, int vit, int dist) {
    actionStatutType statutAction = ACTION_FINIE;
    positionInteger destination;
    static enum {
        ENVOI_ORDRE = 0,
        ATTENTE_ACQUITTEMENT = 1,
        ATTENTE_FIN_MOUVEMENT = 2,
        FINI = 3
    } etape = ENVOI_ORDRE;

    while (etape != FINI) {
        switch (etape) {
            case ENVOI_ORDRE:       // 1: envoyer l'ordre à la propulsion
                destination = propulsionGetPosition();
                destination.x += dist*cos((PI*destination.alpha)/1800);
                destination.y += dist*sin((PI*destination.alpha)/1800);
                propulsionTranslation(acc, vit, dist);  // on envoie l'ordre de translation
                etape = ATTENTE_ACQUITTEMENT;
                break;
            case ATTENTE_ACQUITTEMENT:  // 2: attendre le changement de statut de la propulsion
                switch (propulsionGetStatus()) {
                    case STANDING: // le mouvement n'est pas encore commencé
                        break; // on attend
                    case RELATIVE_MOVE: // le mouvement a commencé
                        etape = ATTENTE_FIN_MOUVEMENT; // on passe à l'état suivant
                        break;
                    case TEST: // ne doit jamais arriver en match, on repasse la prop en STANDING
                        propulsionDisable();
                        while (propulsionGetStatus() != DISABLED); // on attend que l'ordre soit exécuté
                        propulsionEnable();
                        while (propulsionGetStatus() != STANDING); // on attend que l'ordre soit exécuté
                        etape = ENVOI_ORDRE; // on recommence l'action
                        break;
                    default: // ne doit jamais arriver
                        etape = FINI;                   // on termine l'action
                        statutAction = ACTION_ERREUR;   // on termine l'action avec un statut d'erreur
                        break;
                }
                break;
            case ATTENTE_FIN_MOUVEMENT:
                if (propulsionGetStatus() == STANDING) {    // on attend que le mouvement soit fini
                    etape = FINI;
                    if (!compareXYAlpha(destination, propulsionGetPosition())) {    // on vérifie qu'on est au bon endroit
                        statutAction = ACTION_ERREUR;       // sinon, il y a eu un problème (patinage ou obstacle)
                    }
                }
                break;
            case FINI:
                break;
            default:
                statutAction = ACTION_EN_COURS; // au cas où, on vérifie qu'on n'est pas en train de bouger
                etape = ATTENTE_FIN_MOUVEMENT;
                break;
        }
    }

    return (statutAction);
}


actionStatutType rotationBasicAction(int acc, int vit, int angle) {
    actionStatutType statutAction = ACTION_FINIE;
    positionInteger destination;
    static enum {
        ENVOI_ORDRE = 0,
        ATTENTE_ACQUITTEMENT = 1,
        ATTENTE_FIN_MOUVEMENT = 2,
        FINI = 3
    } etape = ENVOI_ORDRE;

    while (etape != FINI) {
        switch (etape) {
            case ENVOI_ORDRE:       // 1: envoyer l'ordre à la propulsion
                destination = propulsionGetPosition();
                destination.alpha += angle*PI/1800;
                propulsionRotation(acc, vit, angle);  // on envoie l'ordre de translation
                etape = ATTENTE_ACQUITTEMENT;
                break;
            case ATTENTE_ACQUITTEMENT:  // 2: attendre le changement de statut de la propulsion
                switch (propulsionGetStatus()) {
                    case STANDING: // le mouvement n'est pas encore commencé
                        break; // on attend
                    case RELATIVE_MOVE: // le mouvement a commencé
                        etape = ATTENTE_FIN_MOUVEMENT; // on passe à l'état suivant
                        break;
                    case TEST: // ne doit jamais arriver en match, on repasse la prop en STANDING
                        propulsionDisable();
                        while (propulsionGetStatus() != DISABLED); // on attend que l'ordre soit exécuté
                        propulsionEnable();
                        while (propulsionGetStatus() != STANDING); // on attend que l'ordre soit exécuté
                        etape = ENVOI_ORDRE; // on recommence l'action
                        break;
                    default: // ne doit jamais arriver
                        etape = FINI;                   // on termine l'action
                        statutAction = ACTION_ERREUR;   // on termine l'action avec un statut d'erreur
                        break;
                }
                break;
            case ATTENTE_FIN_MOUVEMENT:
                if (propulsionGetStatus() == STANDING) {    // on attend que le mouvement soit fini
                    etape = FINI;
                    if (!compareXYAlpha(destination, propulsionGetPosition())) {    // on vérifie qu'on est au bon endroit
                        statutAction = ACTION_ERREUR;       // sinon, il y a eu un problème (patinage ou obstacle)
                    }
                }
                break;
            case FINI:
                break;
            default:
                statutAction = ACTION_EN_COURS; // au cas où, on vérifie qu'on n'est pas en train de bouger
                etape = ATTENTE_FIN_MOUVEMENT;
                break;
        }
    }

    return (statutAction);
}