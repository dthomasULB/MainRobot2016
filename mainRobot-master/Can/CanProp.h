/**
 * @file CanProp.h
 * @brief Header définiccant les messages CAN concernant la carte propulsion.
 */
#ifndef CANPROP_H
#define CANPROP_H

//remarque : les données codées sur plusieurs octets sont envoyés
//en commencant l'octet de poids fort !!


//	COMMANDES POUR INTERFACE AVEC LE PIC PROPULSION :
//	-------------------------------------------

// Commande de mouvements relatifs
//////////////////////////////////////////////////////////////////////
// ordonne un mouvement de translation (mouvement relatif)
// octet 1-2 : accélération [mm/s²]
// octet 3-4 : vitesse de croisière [mm/s]
// octet 5-6 : longueur [mm]
#define PROP_TRANSLATION        0

// ordonne un mouvement de rotation (mouvement relatif)
// octet 1-2 : accélération [1/10 deg/s²]
// octet 3-4 : vitesse de croisière [1/10 deg/s]
// octet 5-6 : longueur [1/10 deg]
#define PROP_ROTATION           1

// ordonne d'arreter immediatement le robot en utilisant la pente de deceleration normale
#define PROP_STOP_NOW           2

// commande de changement de mode
//////////////////////////////////////////////////////////////////////
// passe dans le mode acttif
#define PROP_ENABLE				3
// passe dans le mode inactif
#define PROP_DISABLE			4
// passe en mode test
#define PROP_TEST				5

// commandes du mode test
//////////////////////////////////////////////////////////////////////
// impose le rapport cyclique sur les 2 moteurs
// octet 1 : rapport cyclique du moteur droit (en %)
// octet 2 : rapport cyclique du moteur gauche (en %)
#define PROP_SET_DUTY_CYCLE     6

// Commande de mouvements absolus
//////////////////////////////////////////////////////////////////////
// ordonne de calculer et exécuter une trajectoire vers un point (x,y) (mouvement absolu)
// l'orientation finale du robot n'est pas définie par l'ordre
// octet 1-2 : coordonnée x de la destination [mm]
// octet 3-4 : coordonnée y de la destination [mm]
#define PROP_GOTO_XY            7

// ordonne de calculer et exécuter une trajectoire vers un point (x,y,alpha) (mouvement absolu)
// l'orientation finale du robot est définie par le paramètre alpha
// octet 1-2 : coordonnée x de la destination [mm]
// octet 3-4 : coordonnée y de la destination [mm]
// octet 5-6 : coordonnée alpha de la destination [1/10 deg]
#define PROP_GOTO_XYALPHA       8



// Commandes de configuration
//////////////////////////////////////////////////////////////////////
// REMARQUE : Les 4 commandes suivantes ne sont pas prises en compte si le robot est en mouvement
//impose la position en x actuelle du robot, en mm, codé sur 2 octets
// octet 1-2 : nouvelle coordonnée x [mm]
#define PROP_SET_POS_X          9

//impose la position en y actuelle du robot, en mm, codé sur 2 octets
// octet 1-2 : nouvelle coordonnée y [mm]
#define PROP_SET_POS_Y          0x0A

//impose l'orientation actuelle du robot, en degre, codé sur 2 octets
// octet 1-2 : nouvelle coordonnée alpha [1/10 deg]
#define PROP_SET_POS_ALPHA          0x0B

// impose la position en x,y,alpha actuelle du robot, en mm, et en degré/10 codé sur 6 octets au total
// octet 1-2 : nouvelle coordonnée x [mm]
// octet 3-4 : nouvelle coordonnée y [mm]
// octet 5-6 : nouvelle coordonnée alpha [1/10 deg]
#define PROP_SET_POS                0x12


// ordonne à la propulsion d'activer ou de désactiver la détection de patinage.
// Par défaut, elle est activée
#define PROP_PATINAGE_ON_OFF        0x0D
#define PROP_PATINAGE_OFF	0
#define PROP_PATINAGE_ON	1

// Ordonne à la propulsion d'ajouter un obstacle amovible dans la carte
// L'obstacle est un carré, dont on envoie les coordonnées du coin inférieur gauche et la longueur du côté
// octet 1-2 : coordonnée x du coin inférieur gauche
// octet 3-4 : coordonnée y du coin inférieur droit
// octet 5-6 : longueur du côté
#define PROP_ADD_OBSTACLE           0x0E

// Ordonne à la propulsion de supprimer un obstacle amovible dans la carte
// L'obstacle est un carré, dont on envoie les coordonnées du coin inférieur gauche et la longueur du côté
// Si le carré contient des obstacles inamovibles (ceux de la carte initiale), ils ne sont pas supprimés
// octet 1-2 : coordonnée x du coin inférieur gauche
// octet 3-4 : coordonnée y du coin inférieur droit
// octet 5-6 : longueur du côté
#define PROP_REMOVE_OBSTACLE        0x0F

// Demande à la propulsion si un point se trouve dans un obstacle.
// La propulsion répond par un message CO_PROP_IS_OBSTACLE
// octet 1-2 : coordonnée x du point à examiner
// octet 3-4 : coordonnée y du point à examiner
#define PROP_IS_OBSTACLE_IN_MAP     0x10




//	OBJETS CAN PRODUITS PAR LE PIC PROPULSION :
//	-------------------------------------------

//////////////////////////////////////////////////////////////////////
// statut de la propulsion (cf. documentation pour plus de détails)
// Longueur : 2
// octet 0 : statut
// octet 1 : octet inutile (le dsPIC envoie un int)
#define CO_PROP_STATUS                  CN_PROPULSION*0x10+0

// flag indiquant qu'on a détecté un patinage
// le PIC propulsion coupe les moteurs de lui-même
// Longueur : 1
// l'octet vaut toujours 1
#define CO_PROP_PATINAGE                CN_PROPULSION*0x10+1

// Position du robot :
// Longueur : 6
// B1-B0 : x, en mm
// B3-B2 : y, en mm
// B5-B4 : alpha, en degré/10
#define CO_PROP_POS                     CN_PROPULSION*0x10+2

// Flag en réponse à l'ordre PROP_IS_OBSTACLE_IN_MAP
// Longueur : 2 de type propIsObstacleType
// B1-B0 : vaut PROP_IS_NO_OBSTACLE si le point défini dans l'ordre ne se trouve pas dans un obstacle
//              PROP_IS_UNREMOVABLE_OBSTACLE si le point défini dans l'ordre se trouve dans un obstacle fixe (obstacle de la carte initiale)
//              PROP_IS_REMOVABLE_OBSTACLE si le point défini dans l'ordre se trouve dans un obstacle amovible (ajouté par l'ordre PROP_ADD_OBSTACLE)
#define CO_PROP_IS_OBSTACLE             CN_PROPULSION*0x10+3

// consigne "polaire" du robot :
// Longueur : 4
// B1-B0 : l, en mm
// B3-B2 : alpha, en degré/10
#define CO_PROP_REL_CSG                 CN_PROPULSION*0x10+4

// Position "polaire" du robot :
// Longueur : 4
// B1-B0 : l, en mm
// B3-B2 : alpha, en degré/10
#define CO_PROP_REL_ODO                 CN_PROPULSION*0x10+5



#endif