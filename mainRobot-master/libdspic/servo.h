/**
 * @file pwm.h
 * @brief Header pour la libraire de fonctions des servo-moteurs
 * @author Yernaux
 * @author bULBot 2013-2014
 */

#ifndef SERVO_H
#define SERVO_H


#include "pwm.h"
#include "clock.h"  // pour la définition de FCY (pour __delay_ms())
#include <libpic30.h>

#define NB_SERVO_MAX    (4)


/**
 * @brief Initialise les sorties et les périphériques pour les servomoteurs
 * @param 
 * @param 
 */
void servoInit(int nbServo, int timerId, float periodMs);

void servoSetAngle(int servoId, int angle);

void servoSetAngleLimits(int servoId, int angleMin, int angleMax);

void servoSetPulseLimits(int servoId, float pulseMin, float pulseMax);

void servoDisable(int servoId);

// ATTENTION : fonction bloquante
void SoftServo(int servoId, int angleDeb, int angleFin, int delayMs);
#endif

