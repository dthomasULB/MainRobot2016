/**
 * @file sharp.h
 * @author bULBot 2011-2012
 * @brief Fichier header pour la gestion des capteurs sharps.
 */

#ifndef DEF_SHARP_H
#define DEF_SHARP_H

#include "../Can/CanTypes.h"


#define INPUT_SHARP_AVANT           0       //<! Entrée analogique pour le sharp avant
#define SEUIL_COLLISION_AVANT       279     //<! Seuil de détection (sur la valeur numérique fournie par l'ADC).
#define HYSTERESE_AVANT             60      //<! Hystérèse pour le sharp avant
#define DISTANCE_DETECTION          300     //<! Distance à laquelle on détecte un obstacle [mm]
#define CENTER_TO_SHARPS_AVANT      110     //<! Distance entre le centre odométrique du robot et l'axe du sharp [mm]
#define DISTANCE_OBSTACLE_AVANT     (DISTANCE_DETECTION + CENTER_TO_SHARPS_AVANT)


typedef struct {
    int isObstacleDetected;
    obstacleType obstacleInfo;
} detectionSharpType;

/**
 * @brief Détection d'un obstacle via les capteurs sharps.
 * @details Analyse de la mesures du capteur sharp placé à l'avant du robot.
 * L'algorithme détecte la présence d'un obstacle devant robot.
 * Le cas échéant, il calcule la position de ce dernier, puis renvoie les infos ainsi obtenues
 */
detectionSharpType detectionObstacleSharp(void);

/**
 * @brief Envoi des mesures brutes du sharps sur le CAN.
 */
#define sharpGetMesure()    conversionADC(INPUT_SHARP_AVANT)


#endif

