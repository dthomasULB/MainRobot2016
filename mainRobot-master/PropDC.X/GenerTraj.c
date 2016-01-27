//////////////////////////////////////////////////////////////////////////////
//								HEADERS										//
//////////////////////////////////////////////////////////////////////////////
#include <math.h>
#include "GenerTraj.h"
#include "globals.h"
#include "../Can/CanTypes.h"


//////////////////////////////////////////////////////////////////////////////
//								DEFINE										//
//////////////////////////////////////////////////////////////////////////////
// Paramètres de conversion entre les axes (x,y) et les axes (a,b)
// Ici, on suppose que les axes (a,b) sont orthogonaux et parallèles aux
// axes (x,y).
// Il est possible d'utiliser des axes (a,b) non-orthogonaux (voir en 2010)
#define ORIG_X  0       // coordonnée x du point d'origine du repère (a,b)
#define ORIG_Y  0       // coordonnée y du point d'origine du repère (a,b)
#define DAX     0.1     // rapport entre  x et  a : l[x] = DAX*l[a]
#define DBY     0.1     // rapport entre  y et  b : l[y] = DBY*l[b]

#define MAX_PATHPOINTS  200
#define TABLE_SIZE_X    3.0
#define TABLE_SIZE_Y    2.0
#define MAP_SIZE_A      30
#define MAP_SIZE_B      20
#define MAP_SIZE        (MAP_SIZE_A*MAP_SIZE_B)

#define TOL_INTERSECT	2 // tolérance sur le calcul d'intersection pour simplifyPath2 (en mm)

#define DIR_UNDEF   127
#define DIR_E       0
#define DIR_NE      1
#define DIR_N       2
#define DIR_NO      3
#define DIR_O       4
#define DIR_SE     -1
#define DIR_S      -2
#define DIR_SO     -3


//////////////////////////////////////////////////////////////////////////////
//									TYPES									//
//////////////////////////////////////////////////////////////////////////////

typedef enum {
    UNPR = 0,
    DONE = 1,
    OBST = 2,
    MOVE_OBST = 3
} tileStatusType;

typedef struct {
    int d;                  // Distance du noeud au noeud d'origine (30000 si pas encore traité)
    char dir;               // Direction de la trajectoire en ce noeud
    tileStatusType  done;   // 0 si noeud pas traité, 1 sinon
    int prec;               // noeud précédent dans la trajectoire
} dijkstra_st;


dijkstra_st dkNodes[MAP_SIZE]; // Vecteur de recherche

//////////////////////////////////////////////////////////////////////////////
//								CONSTANTS									//
//////////////////////////////////////////////////////////////////////////////
#include "2015_2.h"     //définit initialMap

//////////////////////////////////////////////////////////////////////////////
//								FUNCTIONS									//
//////////////////////////////////////////////////////////////////////////////

void trajInit(void) {
    unsigned int i;
    
    for (i = 0; i < MAP_SIZE; i++) {
        dkNodes[i].d = 30000;
        dkNodes[i].dir = 0;
        dkNodes[i].done = initialMap[i];
        dkNodes[i].prec = -1;
    }
}

// convertit des coordonnées robotPosition en coordonées mapPosition
inline mapPosType RPtoMP(absolutePosType rpPos) {
    mapPosType mpPos;

    mpPos.a = (rpPos.x - ORIG_X) / DAX;
    mpPos.b = (rpPos.y - ORIG_Y) / DBY;
    if (rpPos.alpha >= 0) {
        mpPos.dir = (rpPos.alpha + PI / 8) / (PI / 4);
    } else {
        mpPos.dir = (rpPos.alpha - PI / 8) / (PI / 4);
    }
    return (mpPos);
}

// convertit des coordonnées mapPosition en coordonées robotPosition
inline absolutePosType MPtoRP(mapPosType mpPos) {
    absolutePosType rpPos;

    rpPos.x = ORIG_X + DAX * mpPos.a;
    rpPos.y = ORIG_Y + DBY * mpPos.b;
    rpPos.alpha = mpPos.dir * PI / 4;
    return (rpPos);
}

inline float computeSegAngle(absolutePosType rpP0, absolutePosType rpP1) {
    return (satureAngle(atan2(rpP1.y - rpP0.y, rpP1.x - rpP0.x)));
}

// initialise le vecteur des noeuds pour l'algorithme de Dijkstra
inline void resetDijkstraVect(void) {
    unsigned int i;
    
    for (i = 0; i < MAP_SIZE; i++) {
        dkNodes[i].d = 30000;
        dkNodes[i].dir = 0;
        if (dkNodes[i].done == DONE) {
            dkNodes[i].done = UNPR;
        }
        dkNodes[i].prec = -1;
    }
}

inline void deleteNode(int curNode) {
    dkNodes[curNode].done = DONE;
}

inline void trajAddObstacle(obstacleType obstacle) {
    int a, b;
    mapPosType mpObsPos, mpObsSize;
    absolutePosType tmp;

    tmp.x = obstacle.x;         // on calcule les coordonnées de l'obstacle dans la carte
    tmp.y = obstacle.y;
    mpObsPos = RPtoMP(tmp);
    tmp.x = obstacle.size;      // on calcule la taille de l'obstacle, en case
    tmp.y = obstacle.size;
    mpObsSize = RPtoMP(tmp);
    for (a = 0; a < mpObsSize.a+1; a++) {
        for (b = 0; b < mpObsSize.b+1; b++) {
            if (dkNodes[MAP_SIZE_B * (a + mpObsPos.a) + b + mpObsPos.b].done != OBST) {
                dkNodes[MAP_SIZE_B * (a + mpObsPos.a) + b + mpObsPos.b].done = MOVE_OBST;
            }
        }
    }
}

inline void trajRemoveObstacle(obstacleType obstacle) {
    int a, b;
    mapPosType mpObsPos, mpObsSize;
    absolutePosType tmp;

    tmp.x = obstacle.x;         // on calcule les coordonnées de l'obstacle dans la carte
    tmp.y = obstacle.y;
    mpObsPos = RPtoMP(tmp);
    tmp.x = obstacle.size;      // on calcule la taille de l'obstacle, en case
    tmp.y = obstacle.size;
    mpObsSize = RPtoMP(tmp);
    for (a = 0; a < mpObsSize.a+1; a++) {
        for (b = 0; b < mpObsSize.b+1; b++) {
            if (dkNodes[MAP_SIZE_B * (a + mpObsPos.a) + b + mpObsPos.b].done == MOVE_OBST) {
                dkNodes[MAP_SIZE_B * (a + mpObsPos.a) + b + mpObsPos.b].done = UNPR;
            }
        }
    }
}

inline int findNeigbours(int curNode, int *neigbours) {
    int a, b, i, minA, maxA, minB, maxB, dA, dB, nb;

    a = curNode / MAP_SIZE_B;
    b = curNode - MAP_SIZE_B*a;
    nb = 0;
    if (a == 0) minA = 0;                           // on commence par vérifier si le noeud courant est au bord de la table
    else minA = -1;                                 // si oui, il a moins de voisins
    if (a == MAP_SIZE_A - 1) maxA = 0;
    else maxA = 1;
    if (b == 0) minB = 0;
    else minB = -1;
    if (b == MAP_SIZE_B - 1) maxB = 0;
    else maxB = 1;
    for (dA = minA; dA <= maxA; dA++) {
        for (dB = minB; dB <= maxB; dB++) {
            i = curNode + MAP_SIZE_B * dA + dB;     // on calcule l'index du voisin potentiel
            if (dkNodes[i].done == UNPR) {          // si il n'a pas encore été traité
                neigbours[nb] = i;                  // c'est un voisin
                nb++;
            }
        }
    }
    return (nb);
}

inline int findNextNode(void) {
    int i, j;

    i = 0;
    while ((dkNodes[i].done != UNPR) && (i < MAP_SIZE)) {           // on commence par trouver le 1er noeud non traité,
        i++;                                                        // qui devient notre 1er candidat
    }
    if (i >= MAP_SIZE) {                                            // Si on a atteint la fin du vecteur,
        return (-1);                                                // c'est qu'il y a un problème
    }
    for (j = i; j < MAP_SIZE; j++) {                                        // Sinon, on vérifie si on n'a pas un meilleur candidat
        if ((dkNodes[j].done == UNPR) && (dkNodes[j].d < dkNodes[i].d)) {   // Si un noeud non encore traité est plus proche du noeud initial
            i = j;                                                          //  que le candidat actuel, il devient le nouveau candidat
        }
    }
    return (i);
}

inline int atanMP(int b, int a) {
    switch (a) {
        case -1:
            switch (b) {
                case -1: return (DIR_SO);
                    break;
                case 0: return (DIR_O);
                    break;
                case 1: return (DIR_NO);
                    break;
            }
            break;
        case 0:
            switch (b) {
                case -1: return (DIR_S);
                    break;
                case 0: return (DIR_UNDEF);
                    break;
                case 1: return (DIR_N);
                    break;
            }
            break;
        case 1:
            switch (b) {
                case -1: return (DIR_SE);
                    break;
                case 0: return (DIR_E);
                    break;
                case 1: return (DIR_NE);
                    break;
            }
            break;
    }
    return (DIR_UNDEF);
}

inline void updateDistance(int neigbour, int curNode, int endNode) {
    int a1, b1, a2, b2, dir, dist;

    a1 = curNode / MAP_SIZE_B;                  // on calcule les coordonnées des 2 noeuds
    b1 = curNode - MAP_SIZE_B*a1;
    a2 = neigbour / MAP_SIZE_B;
    b2 = neigbour - MAP_SIZE_B*a2;
    dir = atanMP(b2 - b1, a2 - a1);             // on calcule l'orientation du segment
    // on calcule la distance du noeud voisin au noeud initial, en passant par le noeud courant
    dist = 2 * (ABS(a1 - a2) + ABS(b1 - b2));   // on calcule d'abord la distance entre le noeud courant et son voisin                      3|2|3
    if (dist > 3) {                             // on utilise une pseudo-distance euclidienne, pour favoriser les diagonales, par rapport   2|0|2
        dist = 3;                               // à un chemin de type Manhattan; mais sans devoir calculer une racine carrée               3|2|3
    }
    dist += dkNodes[curNode].d;                 // on ajoute la distance du noeud courant au noeud initial
    if (dir != dkNodes[curNode].dir) {          // on ajoute encore une pseudo distance pour les virages, et privilégier les lignes droites
        dist++;                                 // on ajoute une distance constante, quelque soit l'amplitude du virage
    }
    if ((neigbour == endNode) && (dir != dkNodes[endNode].dir))     // si le voisin est le noeud final, on ajoute l'éventuel dernier virage
        dist++;                                                     // on ajoute une distance constante, quelque soit l'amplitude du virage
    // on vérifie maintenant si cette distance est plus petite que la distance actuelle du voisin
    // si elles sont égales, on privilégie la trajectoire qui est bien orientée
    if ((dist < dkNodes[neigbour].d) || ((dist == dkNodes[neigbour].d) && (dir == dkNodes[endNode].dir))) {
        dkNodes[neigbour].d = dist;             // on met à jour la distance
        dkNodes[neigbour].prec = curNode;       // on change le chemin pour arriver au voisin
        dkNodes[neigbour].dir = dir;            // on met à jour l'orientation
    }
}


/////////////////// S16FindPath ///////////////////
//find path from startpoint to endpoint using costMap
//return results in pathpoints
//result char trajStatus=1 si ok
//result char trajStatus=NO_LENGTH si origine=arrivee
//result char trajStatus=START_OUT si impossible: startpoint is outside map
//result char trajStatus=END_OUT si impossible: endpoint is outside map
//result char trajStatus=START_OBST si impossible: startpoint is obstacle
//result char trajStatus=END_OBST si impossible: endpoint is obstacle
//result char trajStatus=NO_WAY si impossible: cost generation failed (inconsistent map, no possible way)
inline int findPath(mapPosType mpStart, mapPosType mpEnd, mapPosType mpPath[]) {
    int curNode;        // Index du noeud courant
    int startNode;      // Index du noeud initial
    int endNode;        // Index du noeud qu'on veut atteindre
    int neigbours[8];   // liste des voisins du noeud courant
    int i, nb;          // variables de travail
    mapPosType mpTmp;   // variable de travail


    resetDijkstraVect();    // on initialise le vecteur de recherche, vierge d'obstacles mobiles
    // On vérifie les cas spéciaux détectables dans les axes (a,b)
    if ( (dkNodes[MAP_SIZE_B * mpStart.a + mpStart.b].done == OBST) ||
         (dkNodes[MAP_SIZE_B * mpStart.a + mpStart.b].done == MOVE_OBST) ) {         // On vérifie que les points de départ et d'arrivée ne sont pas dans un obstacle
        return START_OBST;
    } else if ( (dkNodes[MAP_SIZE_B * mpEnd.a + mpEnd.b].done == OBST) ||
                (dkNodes[MAP_SIZE_B * mpEnd.a + mpEnd.b].done == MOVE_OBST) ) {
        return END_OBST;
    }
    if ((mpStart.a == mpEnd.a) && (mpStart.b == mpEnd.b)) {         // On vérifie que les points de départ et d'arrivée ne sont pas dans la même case de la carte.
        mpPath[0].a = mpStart.a;                                    // Si c'est le cas, la trajectoire est triviale 
        mpPath[0].b = mpStart.b;
        mpPath[1].b = mpEnd.a;
        mpPath[1].b = mpEnd.b;
        return (2);                                                 // elle ne comprend que les points de départ et d'arrivée
    }
    // Sinon, on sort l'artillerie lourde
    startNode = MAP_SIZE_B * mpStart.a + mpStart.b;             // on initialise le noeud de départ
    endNode = MAP_SIZE_B * mpEnd.a + mpEnd.b;                   // et d'arrivée
    dkNodes[startNode].d = 0;                                   // le noeud dedépart a évidemment une distance nulle avec lui-même
    curNode = startNode;
    // On calcule les distances au noeud initial, jusqu'à arriver au noeud final
    while (curNode != endNode) {
        deleteNode(curNode);                                    // on élimine le noeud courant des noeuds à traiter
        nb = findNeigbours(curNode, neigbours);                 // on fait la liste de ses voisins
        for (i = 0; i < nb; i++) {                              // on met à jour la distance de ses voisins avec le noeud initial
            updateDistance(neigbours[i], curNode, endNode);
        }
        curNode = findNextNode();                               // on choisit un nouveau noeud courant
        if (curNode == -1) {
            return (NO_WAY);                                    // Si on n'a pas trouvé de nouveau noeud, il n'y a pas de chemin possible
        }
    }
    // une fois arrivé au point désiré, on retourne sur nos pas pour trouver le chemin
    mpPath[0].a = curNode / MAP_SIZE_B;                     // la trajectoire commence au point d'arrivée
    mpPath[0].b = curNode - MAP_SIZE_B*mpPath[0].a;
    mpPath[0].dir = dkNodes[curNode].dir;
    nb = 1;
    while (curNode != startNode) {                          // tant qu'on n'est pas arrivé
        curNode = dkNodes[curNode].prec;                    // on remonte la trajectoire en prenant le noed précédent
        if (curNode == -1) {                                // test qui ne doit jamais être vrai, mais on ne sait jamais
            return ( NO_WAY);
        }
        mpPath[nb].a = curNode / MAP_SIZE_B;;
        mpPath[nb].b = curNode - MAP_SIZE_B*mpPath[nb].a;
        mpPath[nb].dir = dkNodes[curNode].dir;
        nb++;
    }
    // on doit maintenant retourner le vecteur, puisqu'on a calculé le chemin, du point d'arrivée vers le point de départ
    for (i = 0; i < nb / 2; i++) {
        mpTmp.a = mpPath[i].a;                      // on copie le i-eme élément dans une variable temporaire
        mpTmp.b = mpPath[i].b;
        mpTmp.dir = mpPath[i].dir;
        mpPath[i].a = mpPath[nb - i - 1].a;         // on copie le (Nb-i)-eme élément dans le i-eme élément
        mpPath[i].b = mpPath[nb - i - 1].b;
        mpPath[i].dir = mpPath[nb - i - 1].dir;
        mpPath[nb - i - 1].a = mpTmp.a;             // on copie la variable temporaire dans le (Nb-i)-eme élément
        mpPath[nb - i - 1].b = mpTmp.b;
        mpPath[nb - i - 1].dir = mpTmp.dir;
    }
    return (nb);
}

inline int convertPath(absolutePosType rpStart, absolutePosType rpEnd, int pathSize, mapPosType mpPath[], absolutePosType rpPath[]) {
    unsigned int i;
    signed int length;
    signed int curDir, newDir;

    // 1) la trajectoire commence par le point de départ réel
    rpPath[0].x = rpStart.x;
    rpPath[0].y = rpStart.y;
    length = 1;
    // on calculera son angle plus tard. Il faut en effet connaitre le 2ème point de la trajectoire pour cela.
    // 2) on cherche maintenant le 1er point dans la trajectoire où la direction change
    // on détermine la direction initiale (la direction pour aller vers le 2ème point de la trajectoire, pas l'orientation initiale du robot)
    curDir = atanMP(mpPath[1].b - mpPath[0].b, mpPath[1].a - mpPath[0].a);
    for (i = 1; i < pathSize - 1; i++) {                                                // on parcourt tout le chemin fourni par le Dijkstra,
        newDir = atanMP(mpPath[i].b - mpPath[i - 1].b, mpPath[i].a - mpPath[i - 1].a);  // en ne retenant que les points où la direction change.
        if (mpPath[i].dir != curDir) {                                                  // quand on arrive en un point où on tourne,
            rpPath[length] = MPtoRP(mpPath[i - 1]);                                     // on l'ajoute à la trajectoire et on prend note de la nouvelle direction
            length++;
            curDir = newDir;
        }
    }
    // 3) on peut maintenant calculer l'angle du 1er segment
    rpPath[0].alpha = computeSegAngle(rpPath[0], rpPath[1]);
    // 4) on ajoute le point d'arrivée
    rpPath[length].x = rpEnd.x;
    rpPath[length].y = rpEnd.y;
    rpPath[length].alpha = rpEnd.alpha;
    // 5) on calcule l'angle de l'avant-dernier point de passage pour arriver au vrai point d'arrivée
    rpPath[length - 1].alpha = computeSegAngle(rpPath[length - 1], rpPath[length]);
    // 6) on retourne le nombre de points de la trajectoire
    return (length);
}

propIsObstacleType trajIsObstacleInMap(absolutePosType rpPoint) {
    mapPosType mpPoint;

    mpPoint = RPtoMP(rpPoint);
    if (initialMap[MAP_SIZE_B * mpPoint.a + mpPoint.b] == OBST) {
        return (PROP_IS_UNREMOVABLE_OBSTACLE);
    } else if (initialMap[MAP_SIZE_B * mpPoint.a + mpPoint.b] == MOVE_OBST) {
        return (PROP_IS_REMOVABLE_OBSTACLE);
    } else {
        return (PROP_IS_NO_OBSTACLE);
    }
}

// calcule la trajectoire ooptimale pour aller du point rpStart au point rpEnd,
// en tenant compte des obstacles sur la table. La carte de la table doit être
// initialisée et les obstacles éventuels ajoutés avant d'appeler cette fonction.
int findTrajectoire(absolutePosType rpStart, absolutePosType rpEnd, absolutePosType rpPath[]) {
    mapPosType mpStart, mpEnd;          // Coordonnées des points de départ et d'arrivée dans la carte
    mapPosType mpPath[MAX_PATHPOINTS];  // Coordonnées des points de passage de la trajectoire dans la carte
    int trajLength;                     // nombre de points de passage de la trajectoire, ou code d'erreur


    // On commence par vérifier les cas spéciaux détectables dans les coordonnées (x,y)
    if ((rpStart.x < 0) || (rpStart.x > TABLE_SIZE_X) || (rpStart.y < 0) || (rpStart.y > TABLE_SIZE_Y)) {   // On vérifie que les points de départ
        return START_OUT;                                                                                   // et d'arrivée ne sont pas en dehors de la table
    } else if ((rpEnd.x < 0) || (rpEnd.x > TABLE_SIZE_X) || (rpEnd.y < 0) || (rpEnd.y > TABLE_SIZE_Y)) {
        return END_OUT;
    }
    if ((rpStart.x == rpEnd.x) && (rpStart.y == rpEnd.y)) {         // On vérifie que les points de départ et d'arrivée ne sont pas confondus
        return (NO_LENGTH);
    }
    // Si on est dans le cas nominal, on calcule la trajectoire optimale dans les coordonnées (a,b)
    mpStart = RPtoMP(rpStart);                          // On convertit les points de départs et d'arrivée dans les coordonnées (a,b)
    mpEnd = RPtoMP(rpEnd);
    trajLength = findPath(mpStart, mpEnd, mpPath);      // On utilise l'algorithme de pathfinding
    if (trajLength > 1) {                                                       // Si une trajectoire a été trouvée (au moins 2 points),
        trajLength = convertPath(rpStart, rpEnd, trajLength, mpPath, rpPath);   // on convertit la trajectoire dans les coordonnées (x,y)
    }
    // Retourne le nombre de points de passage de la trajectoire,
    // ou le code d'erreur renvoyé par S16FindPath ou S16ConvertPath
    return trajLength;
}
