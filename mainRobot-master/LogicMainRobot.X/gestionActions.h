/**
 * @file gestionActions.h
 * @author bULBot 2011-2012
 * @author Olivier Desenfans
 * @brief Header pour les actions du robot.  
 */
 
 
#ifndef DEF_GESTIONACTIONS_H
#define DEF_GESTIONACTIONS_H

#include "dataTypes.h"


// rayon du robot, en mm
#define RAYON_ROBOT         110


#define NB_ACTIONS_JAUNE    (2)
extern actionType actionsJaune[NB_ACTIONS_JAUNE];

#define NB_ACTIONS_VERT    (1)
extern actionType actionsVert[NB_ACTIONS_VERT];


#define RESET_ACTION    (-1)
#define DEFAULT_ACTION  (0)
#define CLAP1_ONLY      (1)
#define CLAP2_ONLY      (2)
infoActionType clapChezNousJaune(int option);

 
#endif
