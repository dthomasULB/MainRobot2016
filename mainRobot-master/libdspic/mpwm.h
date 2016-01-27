/*!
 * @file pwm.h
 * @author Kevin De Cuyper
 * @brief Header files avec les fonctions de gestion des PWM du module Output Compare
 */

#ifndef MPWM_H
#define	MPWM_H

#include "../libdspic/timers.h"      // pour les macros TIMER_2 et TIMER_3

/**
 * \enum mpwmStatus
 * \brief spécifie si une erreur est apparue lors de l'exécution d'une fonction liée aux pwm
 */
typedef enum {
    MPWM_SUCCESS = 0, //!< tout s'est bien passé
    MPWM_PERIOD_ERROR = 2, //!< la p�riode est trop grande ou nulle
    MPWM_ID_ERROR = 3 //!< une mauvaise ID de PWM a �t� fournie
} mpwmStatus;


#define MPWM1L1     0
#define MPWM1H1     1
#define MPWM1L2     2
#define MPWM1H2     3
#define MPWM1L3     4
#define MPWM1H3     5
#define MPWM2L1     6
#define MPWM2H1     7

/**
 * @brief Configuration d'un canal pwm basé sur un timer 16bits, la PWM est directement activée avec un rapport cyclique nul
 * Attention à ne pas oublier de configurer la borne via le PPS
 * @param id  numero de la pwm (1 -> 4)
 * @param timer numéeo du timer associé (2 ou 3), le prescaler est calculé automatiquement
 * @param periodMs periode en ms (peut etre inférieure à 1ms)
 * @returns indication du succès ou de l'échec (et raison de l'échec)
 */
mpwmStatus mpwmSetup(int id, float periodMs);

/**
 * @brief Modification du rapport cyclique d'une PWM (utile pour les moteurs), avec saturation
 * @param id  numero de la pwm (1 -> 4)
 * @param dutyCycle rapport cyclique (entre 0 et 1)
 * @returns indication du succès ou de l'échec (et raison de l'échec)
 */
mpwmStatus mpwmSetDutyCycle(int id, float dutyCycle);

/**
 * @brief Modification de la durée de l'état haut d'une PWM (utile pour les servos), avec saturation
 * @param id  numero de la pwm (1 -> 4)
 * @param durationMs durée de l'état haut en ms
 * @returns indication du succès ou de l'échec (et raison de l'échec)
 */
mpwmStatus mpwmSetDuration(int id, float durationMs);

/**
 * @brief Assignation directe du registre de longueur de la PWM, avec saturation
 * @param id  numero de la pwm (1 -> 4)
 * @param duration durée de l'état haut en cycles machine
 * @returns indication du succès ou de l'échec (et raison de l'échec)
 */
mpwmStatus mpwmSetCycle(int id, float duration);

/**
 * @brief (Ré)activation de la PWM, déja fait lors du setup
 * @param id  numero de la pwm (1 -> 4)
 * @returns indication du succès ou de l'échec (et raison de l'échec)
 */
mpwmStatus mpwmEnable(int id);

/**
 * @brief Désactivation de la PWM
 * @param id  numero de la pwm (1 -> 4)
 * @returns indication du succès ou de l'échec (et raison de l'échec)
 */
mpwmStatus mpwmDisable(int id);

#endif	/* MPWM_H */

