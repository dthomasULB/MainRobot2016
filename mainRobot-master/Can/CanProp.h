/**
 * @file CanProp.h
 * @brief Header d�finiccant les messages CAN concernant la carte propulsion.
 */
#ifndef CANPROP_H
#define CANPROP_H

//remarque : les donn�es cod�es sur plusieurs octets sont envoy�s
//en commencant l'octet de poids fort !!


//	COMMANDES POUR INTERFACE AVEC LE PIC PROPULSION :
//	-------------------------------------------

// Commande de mouvements relatifs
//////////////////////////////////////////////////////////////////////
// ordonne un mouvement de translation (mouvement relatif)
// octet 1-2 : acc�l�ration [mm/s�]
// octet 3-4 : vitesse de croisi�re [mm/s]
// octet 5-6 : longueur [mm]
#define PROP_TRANSLATION        0

// ordonne un mouvement de rotation (mouvement relatif)
// octet 1-2 : acc�l�ration [1/10 deg/s�]
// octet 3-4 : vitesse de croisi�re [1/10 deg/s]
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
// ordonne de calculer et ex�cuter une trajectoire vers un point (x,y) (mouvement absolu)
// l'orientation finale du robot n'est pas d�finie par l'ordre
// octet 1-2 : coordonn�e x de la destination [mm]
// octet 3-4 : coordonn�e y de la destination [mm]
#define PROP_GOTO_XY            7

// ordonne de calculer et ex�cuter une trajectoire vers un point (x,y,alpha) (mouvement absolu)
// l'orientation finale du robot est d�finie par le param�tre alpha
// octet 1-2 : coordonn�e x de la destination [mm]
// octet 3-4 : coordonn�e y de la destination [mm]
// octet 5-6 : coordonn�e alpha de la destination [1/10 deg]
#define PROP_GOTO_XYALPHA       8



// Commandes de configuration
//////////////////////////////////////////////////////////////////////
// REMARQUE : Les 4 commandes suivantes ne sont pas prises en compte si le robot est en mouvement
//impose la position en x actuelle du robot, en mm, cod� sur 2 octets
// octet 1-2 : nouvelle coordonn�e x [mm]
#define PROP_SET_POS_X          9

//impose la position en y actuelle du robot, en mm, cod� sur 2 octets
// octet 1-2 : nouvelle coordonn�e y [mm]
#define PROP_SET_POS_Y          0x0A

//impose l'orientation actuelle du robot, en degre, cod� sur 2 octets
// octet 1-2 : nouvelle coordonn�e alpha [1/10 deg]
#define PROP_SET_POS_ALPHA          0x0B

// impose la position en x,y,alpha actuelle du robot, en mm, et en degr�/10 cod� sur 6 octets au total
// octet 1-2 : nouvelle coordonn�e x [mm]
// octet 3-4 : nouvelle coordonn�e y [mm]
// octet 5-6 : nouvelle coordonn�e alpha [1/10 deg]
#define PROP_SET_POS                0x12


// ordonne � la propulsion d'activer ou de d�sactiver la d�tection de patinage.
// Par d�faut, elle est activ�e
#define PROP_PATINAGE_ON_OFF        0x0D
#define PROP_PATINAGE_OFF	0
#define PROP_PATINAGE_ON	1

// Ordonne � la propulsion d'ajouter un obstacle amovible dans la carte
// L'obstacle est un carr�, dont on envoie les coordonn�es du coin inf�rieur gauche et la longueur du c�t�
// octet 1-2 : coordonn�e x du coin inf�rieur gauche
// octet 3-4 : coordonn�e y du coin inf�rieur droit
// octet 5-6 : longueur du c�t�
#define PROP_ADD_OBSTACLE           0x0E

// Ordonne � la propulsion de supprimer un obstacle amovible dans la carte
// L'obstacle est un carr�, dont on envoie les coordonn�es du coin inf�rieur gauche et la longueur du c�t�
// Si le carr� contient des obstacles inamovibles (ceux de la carte initiale), ils ne sont pas supprim�s
// octet 1-2 : coordonn�e x du coin inf�rieur gauche
// octet 3-4 : coordonn�e y du coin inf�rieur droit
// octet 5-6 : longueur du c�t�
#define PROP_REMOVE_OBSTACLE        0x0F

// Demande � la propulsion si un point se trouve dans un obstacle.
// La propulsion r�pond par un message CO_PROP_IS_OBSTACLE
// octet 1-2 : coordonn�e x du point � examiner
// octet 3-4 : coordonn�e y du point � examiner
#define PROP_IS_OBSTACLE_IN_MAP     0x10



//	OBJETS CAN PRODUITS PAR LE PIC PROPULSION :
//	-------------------------------------------

//////////////////////////////////////////////////////////////////////
// statut de la propulsion (cf. documentation pour plus de d�tails)
// Longueur : 2
// octet 0 : statut
// octet 1 : octet inutile (le dsPIC envoie un int)
#define CO_PROP_STATUS                  CN_PROPULSION*0x10+0

// flag indiquant qu'on a d�tect� un patinage
// le PIC propulsion coupe les moteurs de lui-m�me
// Longueur : 1
// l'octet vaut toujours 1
#define CO_PROP_PATINAGE                CN_PROPULSION*0x10+1

// Position du robot :
// Longueur : 6
// B1-B0 : x, en mm
// B3-B2 : y, en mm
// B5-B4 : alpha, en degr�/10
#define CO_PROP_POS                     CN_PROPULSION*0x10+2

// Flag en r�ponse � l'ordre PROP_IS_OBSTACLE_IN_MAP
// Longueur : 2 de type propIsObstacleType
// B1-B0 : vaut PROP_IS_NO_OBSTACLE si le point d�fini dans l'ordre ne se trouve pas dans un obstacle
//              PROP_IS_UNREMOVABLE_OBSTACLE si le point d�fini dans l'ordre se trouve dans un obstacle fixe (obstacle de la carte initiale)
//              PROP_IS_REMOVABLE_OBSTACLE si le point d�fini dans l'ordre se trouve dans un obstacle amovible (ajout� par l'ordre PROP_ADD_OBSTACLE)
#define CO_PROP_IS_OBSTACLE             CN_PROPULSION*0x10+3

// consigne "polaire" du robot :
// Longueur : 4
// B1-B0 : l, en mm
// B3-B2 : alpha, en degr�/10
#define CO_PROP_REL_CSG                 CN_PROPULSION*0x10+4

// Position "polaire" du robot :
// Longueur : 4
// B1-B0 : l, en mm
// B3-B2 : alpha, en degr�/10
#define CO_PROP_REL_ODO                 CN_PROPULSION*0x10+5



#endif