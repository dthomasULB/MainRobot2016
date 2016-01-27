/**
 * @file dataTypes.h
 * @author bULBot 2011-2012
 * @brief D�finition des structures et �num�rations.
 */

#ifndef DEF_DATATYPES_H
#define DEF_DATATYPES_H


#include "../Can/CanTypes.h"


//! Description d'un point sur la carte.
typedef struct {
    signed int x; /** Abscisse. */
    signed int y; /** Ordonn�e. */
} point;

//! Equipe actuelle.
typedef enum {
    VERT = 0,
    JAUNE = 1
} team;

//! Etat du match
typedef enum {
    INIT = 0, /*! Etat initial, en attente du placement de la goupille */
    PRE_MATCH = 1, /*! En attente du d�but du match. */
    ONGOING = 2, /*! Match en cours. */
    OVER = 3, /*! Match termin�. */
    ERROR = 4 /*! Un probl�me est survenu lors d'une action */
} match;


typedef enum {
    STRATEGIE = 1, /*! D�termination de l'action � effectuer. */
    INIT_ETAPE = 2, //! D�marrage de l'�tape � effectuer
    EXEC_ETAPE = 3, /*! Etape en cours. */
    IDLE = 4, /*! Aucune action en cours, le robot n'a rien � faire. */
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

//! Structure contenant une action strat�gique.

typedef struct {
    int option;
    infoActionType(*ptr2Fct)(int); /** Pointeur vers la fonction d'ex�cution de l'action. */
} actionType;


//! Ordre pour la trajectoire.

typedef struct {
    char cmd; /** Commande � envoyer. */
    positionInteger compo; /** Position compl�te � consid�rer dans l'ordre. */
} ordreTraj;

//! Ordre de mouvement pour la propulsion (translation ou rotation).

typedef struct {
    char cmd; /** Commande � envoyer. */
    int acc; /** Acc�l�ration [mm/s^2 ou �/(10*s^2)]. */
    int vit; /** Vitesse 	 [mm/s ou �/(10*s)]. */
    int dist; /** Distance	 [mm ou �/10]. */
} ordreMvtProp;


#endif
