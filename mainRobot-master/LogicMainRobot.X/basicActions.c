#include "BLE.h"
#include "Globals.h"
#include "../Can/CanTypes.h"
#include "../libdspic/clock.h"
#include "dataTypes.h"
#include "gestionPropulsion.h"
#include <math.h>
#include "sharp.h"
#include "spio.h"
#include <libpic30.h>
#include "stepper.h"


#define NO_ACTION       (0)
#define INIT_ACTION     (1)
#define RESET_ACTION    (2)
#define EXECUTE         (3)


ObjetStock enStock[4] = {{0,CYLINDRE,AUTRE},{0,CONE,AUTRE},{0,CUBE,AUTRE},{0,POISSON,AUTRE}};
obstacleType OldAlly;

infoActionType noActionFct(positionInteger destination,technique method) {
    infoActionType infoAction = {ACTION_PAS_COMMENCEE, 0, 0, ACTION_AUCUNE,{0, 0, 0}};
    return (infoAction);
}

infoActionType StopNowFct(positionInteger destination,technique method){
    infoActionType infoAction = {ACTION_PAS_COMMENCEE, 0, 0, ACTION_TRAJECTOIRE,{0, 0, 0}};
        static enum {
        ENVOI_ORDRE = 0,
        ATTENTE_FIN_MOUVEMENT = 2,
        FINI = 3
    } etape = ENVOI_ORDRE;
    positionInteger RESET = {-1,-1,-1};
    
    if (destination.x == RESET.x){
        etape = ENVOI_ORDRE;
    }
    
    switch(etape){
        case ENVOI_ORDRE:
            propulsionStopNow();
            etape = ATTENTE_FIN_MOUVEMENT;
            infoAction.statut = ACTION_EN_COURS;
            break;
        case ATTENTE_FIN_MOUVEMENT:
        if (propulsionGetStatus() == STANDING) {    // on attend que le mouvement soit fini
                infoAction.statut = ACTION_ERREUR; //02/03/2016
                etape = ENVOI_ORDRE;
                
           }
            break;
        default:
            break;
    }
    return(infoAction);
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


actionStatutType trajectoryBasicAction(positionInteger destination) {
    // Si destion est négatif, le trajet est fait à l'envers (utile en cas d'asymétrie de mécanisme)
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
                statutAction = ACTION_EN_COURS;
                break;
            case ATTENTE_ACQUITTEMENT:  // 2: attendre le changement de statut de la propulsion
                switch (propulsionGetStatus()) {
                    case STANDING: // le mouvement n'est pas encore commencé
                        break; // on attend
                    case TRAJECTORY: // le mouvement a commencé
                    case RELATIVE_MOVE:
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
                        etape = FINI;        
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
    actionStatutType statutAction = ACTION_EN_COURS;
    positionInteger destination;
    static enum {
        ENVOI_ORDRE = 0,
        ATTENTE_ACQUITTEMENT = 1,
        ATTENTE_FIN_MOUVEMENT = 2,
        FINI = 3
    } etape = ENVOI_ORDRE;

   // while (etape != FINI) {
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
                etape = ENVOI_ORDRE;
                statutAction = ACTION_FINIE;
                break;
            default:
                statutAction = ACTION_EN_COURS; // au cas où, on vérifie qu'on n'est pas en train de bouger
                etape = ATTENTE_FIN_MOUVEMENT;
                break;
        }
    //}

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
/*
 Faut définir les actions mots/clés
 Actions possibles de l'année
 ("se déplacer","déplacer","gêner","stocker","prendre","reconaître","tirer","empiler","pousser")
 infoAction 
 infoAction actions2016 = {porte,poisson,sable,coquiallage}
 
*/

 infoActionType seDeplacer(positionInteger destination,technique method){
    static infoActionType infoAction = {ACTION_PAS_COMMENCEE, 0, 1, ACTION_TRAJECTOIRE, 0};
    positionInteger tmp;
    int distance;
    // On retire l'ancienne position de l'allié et on met la nouvelle a jours juste avant de détermienr une traj
    if (infoAction.statut != ACTION_EN_COURS){          // soit première fois qu'on rentre dedans soit on a cloturé la rpécédente par un FInI ou ERREUR
        propulsionRemoveObstacle(OldAlly);
        positionInteger tmp = getAllyPos();
        OldAlly.x =(float) tmp.x-175; // on prend le point en bas à gauche
        OldAlly.y =(float) tmp.y-175;
        __delay_ms(1);
       propulsionAddObstacle(OldAlly);
        __delay_ms(1);

    }
     switch (method){
        case (AUCUNE_ACTION):
            infoAction.position = destination;
            infoAction.statut = trajectoryBasicAction(infoAction.position);
            break;
         case (ACTIONNEUR): //à l'envers :D
             infoAction.position = destination;
            /*infoAction.position.x = -infoAction.position.x;
            infoAction.position.y = -infoAction.position.y;
            infoAction.position.alpha = -infoAction.position.alpha;
            infoAction.statut = trajectoryBasicAction(infoAction.position);*/
            tmp = propulsionGetPosition();
            distance = -ABS(destination.x-tmp.x);
            infoAction.statut = translationBasicAction(100,200,distance);
            break;
        case (ACTIONNEUR_ACTIF): // hard coded que pour en X
            tmp = propulsionGetPosition();
            distance = ABS(destination.x-tmp.x);
            infoAction.statut = translationBasicAction(100,200,distance);
             break;
        default:
            infoAction.statut = ACTION_ERREUR;
            break;     
    }
    return (infoAction);
  }
 
 infoActionType pousser(positionInteger posObj,technique method){
    static infoActionType infoAction = {ACTION_PAS_COMMENCEE, 0, 1, ACTION_ACTIONNEURS, 0};
    switch (method){
        case (AUCUNE_ACTION):
            infoAction.statut = ACTION_FINIE;
            break;
        
        case (ACTIONNEUR):
            infoAction.statut = ACTION_EN_COURS;
            if (compareXYAlpha(posObj, propulsionGetPosition())){
      //          sortirServoClap();
                infoAction.statut = ACTION_FINIE;
                break;
          
            }
        default:
            infoAction.statut = ACTION_FINIE;
            break;
    }
    return (infoAction);
  }
  infoActionType prendre(positionInteger posObj,technique method){
    static infoActionType infoAction = {ACTION_PAS_COMMENCEE, 0, 1, ACTION_ACTIONNEURS, 0};
    switch (method){
        case (AUCUNE_ACTION):               // = Déposer
            infoAction.statut = ACTION_FINIE;
            break;
        
        case (ACTIONNEUR):
            infoAction.statut = ACTION_EN_COURS;
           // if (compareXYAlpha(posObj, propulsionGetPosition())){
                ouvrirPorte();
                infoAction.statut = ACTION_FINIE;
                break;
          
           // }  
        case (PINCE):
            infoAction.statut = ACTION_EN_COURS;
         //   if (compareXYAlpha(posObj, propulsionGetPosition())){
                infoAction.statut = ACTION_FINIE;
                
                break;
          
          //  }  
        default:
            infoAction.statut = ACTION_FINIE;
            break;
    }
    return (infoAction);
  }
/*
 infoActionType empiler(positionInteger posObj,technique method){
    static infoActionType infoAction = {ACTION_PAS_COMMENCEE, 0, 1, ACTION_ACTIONNEURS, 0};
    switch (method){
        case (AUCUNE_ACTION):
            infoAction.statut = ACTION_FINIE;
            break;
        
        case (ACTIONNEUR):
            infoAction.statut = ACTION_EN_COURS;
            if (compareXY(posObj, propulsionGetPosition())){
                prendre (posObj.alpha,AUCUNE_ACTION) ;   
                infoAction.statut = ACTION_FINIE;
                break;
          
            }  
        case (PINCE):
            infoAction.statut = ACTION_EN_COURS;
            prendre(posObj.alpha,AUCUNE_ACTION); // ici c'est alpha = Z
            infoAction.statut = ACTION_FINIE;
                break;
          
          //  }  
    }
    return (infoAction);
  }


   infoActionType tirer(positionInteger posObj,technique method){
    static infoActionType infoAction = {ACTION_PAS_COMMENCEE, 0, 1, ACTION_ACTIONNEURS, 0};
    switch (method){
        case (AUCUNE_ACTION):
            infoAction.statut = ACTION_FINIE;
            break;
        
        case (ACTIONNEUR):
            infoAction.statut = ACTION_EN_COURS;
            if (compareXYAlpha(posObj, propulsionGetPosition())){
                infoAction.statut = ACTION_FINIE;
                break;
          
            }  
    }
    return (infoAction);
   }
  */ 
infoActionType gener(positionInteger destination,technique method){
     static infoActionType infoAction = {ACTION_PAS_COMMENCEE, 0, 1, ACTION_TRAJECTOIRE, 0};
    // On retire l'ancienne position de l'allié et on met la nouvelle a jours juste avant de détermienr une traj
    if (infoAction.statut == ACTION_PAS_COMMENCEE){
        propulsionRemoveObstacle(OldAlly);
        positionInteger tmp = getAllyPos();
        OldAlly.x =(float) tmp.x;
        OldAlly.y =(float) tmp.y;
        __delay_ms(1);
       propulsionAddObstacle(OldAlly);
        __delay_ms(1);

    }
    infoAction.position = destination;
    infoAction.statut = trajectoryBasicAction(infoAction.position);
    return (infoAction);

  }
   
   
/*
   infoActionType stocker(ObjetStock aStock,technique method){
    static infoActionType infoAction = {ACTION_PAS_COMMENCEE, 0, 1, ACTION_ACTIONNEURS, 0};
    
    switch (method){
        case (PINCE):
            enStock[aStock.forme].quantite += 1;
            infoAction.statut = ACTION_FINIE;           // on considère juste qu'on là dans la(es) pince(s)
 * // Test pour vérifier qu'on a bien l'objet
            break;
        
        case (ACTIONNEUR):
            infoAction.statut = ACTION_EN_COURS;
                // Test pour vérifier qu'on a bien l'objet
                enStock[aStock.forme].quantite += 1;       // on l'ajoute au stock
                infoAction.statut = ACTION_FINIE;
                break;
        default:
            infoAction.statut = ACTION_FINIE; 
            break;
            }  
    return (infoAction);
   }
 * 
 *  LES FONCTIONS ICI SONT FAITE POUR ETRE APPELEES AU SEIN D AUTRE FONCTION ET NON SEUL
 * 
  infoObjet reconaitre(technique method){
     infoObjet objet;
     detectionSharpType detectionSharpTemp;
     static infoActionType infoAction = {ACTION_PAS_COMMENCEE, 0, 1, ACTION_ACTIONNEURS, 0};
    switch (method){
        case (AUCUNE_ACTION):
            break;
        
        case (CAPTEUR_DISTANCE):
            detectionSharpTemp = detectionObstacleSharp();
            objet.dist= detectionSharpTemp.obstacleInfo.y;
            break;
          
         case (CAPTEUR_COULEUR):
             
           objet.couleur = 0;

            break;
         case (CAPTEUR_FORME):
           
            objet.forme = 0;

            break;
    }
    return (objet);
  } 
  
 */
  
  // DENIS_MATH
  
  int computeTime(positionInteger pos1, positionInteger pos2) {
    int tmp=0;
    const int temps_distance = 3; //ms
    const int temps_angle = 2; //ms
    int temps=0;
 
    if (pos1.y-pos2.y != 0){  // si on se déplace en Y
        tmp += ABS(cos(DEG2RAD(pos1.alpha)))*900 + ABS(cos(DEG2RAD(pos2.alpha)))*900;
    }
    if (pos1.x-pos2.x != 0){  // si on se déplace en X
        tmp += ABS(sin(DEG2RAD(pos1.alpha)))*900 + ABS(sin(DEG2RAD(pos1.alpha)))*900;
    }
    temps +=ABS(tmp*temps_angle);
    int x = pos1.x - pos2.x;
	int y = pos1.y - pos2.y;
	temps +=(ABS(x) + ABS(y))*temps_distance;
    
    return (temps);
}
int find_maximun(float a[],int n) {
  int c, index;
  float min = a[0];
  index = 0;
 
  for (c = 1; c < n; c++) {
    if (a[c] > min) {
       index = c;
       min = a[c];
    }
  }
 
  return index;
}

int compareAlpha(positionInteger pos1, positionInteger pos2) {
    int tmp;
    tmp = pos1.alpha - pos2.alpha;
    while (tmp > 1800)  tmp -= 3600;
    while(tmp < -1800)  tmp += 3600;
    tmp = ABS(tmp);
    return tmp;
}
