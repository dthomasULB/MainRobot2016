#include "../Can/CanTypes.h"

/**
 * @brief Initialisation de la communication avec la propulsion.
 * @details D�claration de la consommation et de la production des
 * diff�rents messages CAN li�s au fonctionnement de la propulsion.
 * D�fini la r�ception de la position et de l'�tat de propulsion.
 */
void propulsionInitCan(void);

void propulsionEnable(void);

void propulsionDisable(void);

/**
 * @brief Renvoie la position actuelle mesur�e par l'odom�trie 
 * @Details Le canCurrentPosition est mis � jour d�s que la propulsion le met sur le CAN
 */
positionInteger propulsionGetPosition(void);

/**
 * @brief Renvoie l'�tat du module propulsion
 * @Details Le canEtatPosition est mis � jour d�s que la propulsion le met sur le CAN
 */
propStateType propulsionGetStatus(void);

/**
 * @brief Fixe la position actuelle par un envoi CAN � la propulsion
 */
void propulsionSetPosition(positionInteger pos);

/**
 * @brief ordonne � la propulsion d'effectuer une translation � vitesse et
 * acceleration donn�es
 */
void propulsionTranslation(int acc, int vit, int dist);

/**
 * @brief ordonne � la propulsion d'effectuer une rotation � vitesse et 
 * acceleration donn�es
 */
void propulsionRotation(int acc, int vit, int dist);

void propulsionGotoxy(positionInteger pos);

void propulsionGotoxyalpha(positionInteger pos);

void propulsionStopNow(void);
