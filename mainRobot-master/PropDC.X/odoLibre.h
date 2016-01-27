#include "globals.h"

void odoInit(void);

//met à jour les positions et vitesses
void calculeOdometrie(void);
// Cette fonction renvoie la vitesse polaire actuelle
inline relativePosType odoGetRelVel(void);
// Cette fonction renvoie l'intégrale de la vitesse polaire
inline relativePosType odoGetRelPos(void);
// Cette fonction renvoie la position absolue actuelle
inline absolutePosType odoGetAbsPos(void);

// Cette fonction modifie la position absolue
inline void odoSetAbsPos(absolutePosType newPos);
