#include "globals.h"
#include "../Can/CanTypes.h"


// structure permettant de stocker les coordonn�es des points de passage,
// dans le syst�mes de coordonn�es de la carte de la table
typedef struct {
	char a;
	char b;
	char dir;
} mapPosType;

void trajInit(void);
//calculate trajectory, return the number of sub segment necessary to go to the selected point
//return the number of point in the vector used to go to the selected point
int findTrajectoire(absolutePosType rpStart, absolutePosType rpEnd, absolutePosType rpPath[]);

void trajAddObstacle(obstacleType obstacle);
void trajRemoveObstacle(obstacleType obstacle);
propIsObstacleType trajIsObstacleInMap(obstacleType obstacle);//absolutePosType rpPoint);
