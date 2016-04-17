/**
 * @file dataTypes.h
 * @author bULBot 2011-2012
 * @brief Définition des structures et énumérations.
 */

#ifndef DEF_DATATYPES_H
#define DEF_DATATYPES_H


#include "../Can/CanTypes.h"


//! Description d'un point sur la carte.
typedef struct {
    signed int x; /** Abscisse. */
    signed int y; /** Ordonnée. */
} point;

//! Equipe actuelle.
typedef enum {
    VERT = 0,
    JAUNE = 1
} team;

//! Etat du match
typedef enum {
    INIT = 0, /*! Etat initial, en attente du placement de la goupille */
    PRE_MATCH = 1, /*! En attente du début du match. */
    ONGOING = 2, /*! Match en cours. */
    OVER = 3, /*! Match terminé. */
    ERROR = 4 /*! Un problème est survenu lors d'une action */
} match;


typedef enum {
    STRATEGIE = 1, /*! Détermination de l'action à effectuer. */
    INIT_ETAPE = 2, //! Démarrage de l'étape à effectuer
    EXEC_ETAPE = 3, /*! Etape en cours. */
    IDLE = 4, /*! Aucune action en cours, le robot n'a rien à faire. */
    ERREUR = 5 /*! Une erreur grave s'est produite. */
} generalStateMachine;


//! Actions de base (briques pour les actionType).
typedef enum {
    ACTION_AUCUNE = 0, /** Aucune action de base en cours. */
    ACTION_TRAJECTOIRE = 1, /** Action au moyen de la trajectoire. */
    ACTION_PROPULSION = 2, /** Action au moyen de la propulsion seule. */
    ACTION_ACTIONNEURS = 3 /** Action au moyen d'un actionneur. */
} basicActionType;

typedef enum {
    ACTION_ERREUR = 0,
    ACTION_PAS_COMMENCEE = 1,
    ACTION_EN_COURS = 2,
    ACTION_FINIE = 3,
    ACTION_REMISE = 4
} actionStatutType;

typedef struct {
    actionStatutType statut;
    int etapeEnCours;
    int nbEtapes;
    basicActionType typeEtapeEnCours;
    positionInteger position;
} infoActionType;

//! Structure contenant une action stratégique.
typedef enum {
    AUCUNE_ACTION = 0, /** Rien de spécial à faire **/
    ACTIONNEUR = 1, /** Besoin d'utiliser un actionneur */
    ACTIONNEUR_ACTIF = 2,
    ACTIONNEUR_RENTRER = 3,
    PINCE = 4, /** Besoin de la pince */
    CAPTEUR_DISTANCE = 5, /**Besoin d'un capteur  */ 
    CAPTEUR_COULEUR=6, /*Besoin d'un capteur de couleur*/
    CAPTEUR_FORME=7
} technique;






//! Ordre pour la trajectoire.

typedef struct {
    char cmd; /** Commande à envoyer. */
    positionInteger compo; /** Position complète à considérer dans l'ordre. */
} ordreTraj;

//! Ordre de mouvement pour la propulsion (translation ou rotation).

typedef struct {
    char cmd; /** Commande à envoyer. */
    int acc; /** Accélération [mm/s^2 ou °/(10*s^2)]. */
    int vit; /** Vitesse 	 [mm/s ou °/(10*s)]. */
    int dist; /** Distance	 [mm ou °/10]. */
} ordreMvtProp;


// DENIS 2016

typedef enum {
    CYLINDRE = 0, 
    CONE = 1, 
    CUBE = 2, 
    POISSON = 3,
} objetForme;

typedef enum {
    AUCUNE = 0,
    BLANC = 1, 
    NOIR = 2, 
    AUTRE = 3, 
} objetCouleur;

typedef struct {
    float dist; 
    objetForme forme;
    objetCouleur couleur;
} infoObjet;

typedef enum {
    A_FAIRE = 0, /** action à résoudre **/
    REPORTEE = 1, /** problème / obstacle */
    FAITE = 2, /** faite faut plus y toucher */
    NO_TRAJECT=3, /* trouve pas de trajet pour la faire*/
} StatusTache;

typedef struct {
    positionInteger dest;
    technique methode;
    infoActionType(*ptr2Fct)(positionInteger,technique); /** Pointeur vers la fonction d'exécution de l'action. */
    StatusTache done;
} actionType;

typedef struct {
    int numAction;
    float point;
    StatusTache done;
    actionType* action;//(*ptr2act)(positionInteger,technique,infoActionType);
}actionPossible ;

typedef struct {
    int quantite; 
    objetForme forme;
    objetCouleur couleur;
} ObjetStock;

typedef enum {
    UNINITIALISED = 0,
    MODE_COMMAND = 1,
    NOT_CONNECTED = 2,
    CONNECTED = 3,
    MLDP=4
} StatutsRadio;

typedef enum {
    AOK = 0,
    CMD = 1,
    REBOOT = 2,
    CONNECTED_BLE = 3,
    ERR = 4,
    END = 5,
    MLDP_BLE=6
}DectectedReponse;

typedef enum {
    TASK_ACTION = 0,
    POSITION_ALLY = 1,
    OTHER = 2,
}update;

typedef struct {
int Pince;
int Decteur_couleur;
int Aimant;					// mettre tout les actionneurs disponibles sur les robots
int Pelle;
}actionneurs;


typedef struct {
    int id;
    positionInteger position;			// position du robot
    actionneurs actionneur;				// Actionneur dispo sur le robot
    int Tache;							// Tache en cours
    int Action;							// Action en cours
    int OBST;                           // si obstacle vu ou pas
    int ERROR;                          // c'est la meeeeerde !
} infoRotbot;


#endif

