/**
 * @file sharp.h
 * @author bULBot 2011-2012
 * @brief Fichier header pour la gestion des capteurs sharps.
 */

#ifndef DEF_SHARP_H
#define DEF_SHARP_H

#include "../Can/CanTypes.h"


#define INPUT_SHARP_AVANT           0       //<! Entr�e analogique pour le sharp avant
#define SEUIL_COLLISION_AVANT       279     //<! Seuil de d�tection (sur la valeur num�rique fournie par l'ADC).
#define HYSTERESE_AVANT             60      //<! Hyst�r�se pour le sharp avant
#define DISTANCE_DETECTION          300     //<! Distance � laquelle on d�tecte un obstacle [mm]
#define CENTER_TO_SHARPS_AVANT      110     //<! Distance entre le centre odom�trique du robot et l'axe du sharp [mm]
#define DISTANCE_OBSTACLE_AVANT     (DISTANCE_DETECTION + CENTER_TO_SHARPS_AVANT)


typedef struct {
    int isObstacleDetected;
    obstacleType obstacleInfo;
} detectionSharpType;

/**
 * @brief D�tection d'un obstacle via les capteurs sharps.
 * @details Analyse de la mesures du capteur sharp plac� � l'avant du robot.
 * L'algorithme d�tecte la pr�sence d'un obstacle devant robot.
 * Le cas �ch�ant, il calcule la position de ce dernier, puis renvoie les infos ainsi obtenues
 */
detectionSharpType detectionObstacleSharp(void);

/**
 * @brief Envoi des mesures brutes du sharps sur le CAN.
 */
#define sharpGetMesure()    conversionADC(INPUT_SHARP_AVANT)


#endif

