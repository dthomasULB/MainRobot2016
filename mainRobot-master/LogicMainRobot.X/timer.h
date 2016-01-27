
#ifndef DEF_TIMER_H
#define DEF_TIMER_H

//#include "globales.h"
#include <xc.h>


void matchTimerInit(void);
int getMatchTimerFlag(void);
/**
 * @brief Lancement du timer de match (préchargé à 90 secondes).
 */
#define StartMatchTimer()		T2CONbits.TON = 1;

/**
 * @brief Arrêt du timer 2 et remise à zéro du compteur.
 */
#define StopMatchTimer()    T2CONbits.TON = 0;	\
                            TMR2 = 0x00;		\
                            TMR3 = 0x00;


void msTimerInit(void);
void msTimerStart(void);
void msTimerStop(void);
unsigned long msTimerGet(void);
void waitXms(int x);

#endif
