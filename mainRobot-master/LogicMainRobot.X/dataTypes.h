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
    ERR = 5 /*! Une erreur grave s'est produite. */
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

typedef struct {
    int option;
    infoActionType(*ptr2Fct)(int); /** Pointeur vers la fonction d'exécution de l'action. */
} actionType;


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


#endif
