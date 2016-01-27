#include "../Can/CanTypes.h"

/**
 * @brief Initialisation de la communication avec la propulsion.
 * @details Déclaration de la consommation et de la production des
 * différents messages CAN liés au fonctionnement de la propulsion.
 * Défini la réception de la position et de l'état de propulsion.
 */
void propulsionInitCan(void);

void propulsionEnable(void);

void propulsionDisable(void);

/**
 * @brief Renvoie la position actuelle mesurée par l'odométrie 
 * @Details Le canCurrentPosition est mis à jour dès que la propulsion le met sur le CAN
 */
positionInteger propulsionGetPosition(void);

/**
 * @brief Renvoie l'état du module propulsion
 * @Details Le canEtatPosition est mis à jour dès que la propulsion le met sur le CAN
 */
propStateType propulsionGetStatus(void);

/**
 * @brief Fixe la position actuelle par un envoi CAN à la propulsion
 */
void propulsionSetPosition(positionInteger pos);

/**
 * @brief ordonne à la propulsion d'effectuer une translation à vitesse et
 * acceleration données
 */
void propulsionTranslation(int acc, int vit, int dist);

/**
 * @brief ordonne à la propulsion d'effectuer une rotation à vitesse et 
 * acceleration données
 */
void propulsionRotation(int acc, int vit, int dist);

void propulsionGotoxy(positionInteger pos);

void propulsionGotoxyalpha(positionInteger pos);

void propulsionStopNow(void);
