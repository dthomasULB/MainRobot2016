/*!
 * @file timer.c
 * @author Thomas Lemaire
 * @brief Initialisation des registres des timers utilisés.
 * Révision Michel Osée 2014
 */
 
#include "gestionPropulsion.h"
#include "timer.h"
#include <xc.h>


volatile int prMatchTimerFlag = 0;

/**
 * @brief Initialisation du deuxième timer 32 bits.
 * @details Configuration des timers 2 et 3 en timers
 * 32 bits pour calculer précisément la durée du match. 
¨*/
void matchTimerInit(void) {
	T2CONbits.TON = 0;		// On arrête les timers 2 et 3 pour les configurer
	T3CONbits.TON = 0;
	T2CONbits.T32 = 1;		// On les combine en un timer 32 bits
	T2CONbits.TCKPS = 1;	// avec un prescaler de 1:8
	T2CONbits.TCS = 0;		// avec Tcy comme horloge
	PR2 = 0x747F;			// pour déborder après 90s => PR = 90s/(8*25ns)-1 = (450E6)-1 = 0x1AD2747F
	PR3 = 0x1AD2; 
	TMR2 = 0;				// on initialise le compteur
	TMR3 = 0; 
	IFS0bits.T3IF = 0; 		// Réinitialisation du flag d'interruption
	IEC0bits.T3IE = 1; 		// Activation de l'interruption
}	


/**
 * @brief Gestion de l'interruption du timer 3.
 * @details Gestion du débordement du compteur de fin de match.
 */
void __attribute__((interrupt, auto_psv)) _T3Interrupt(void) {
	IFS0bits.T3IF = 0;		// Réinitialisation du flag
	T2CONbits.TON = 0;		// Arrêt du timer
	prMatchTimerFlag = 1;	// Activation du flag de fin de match
	propulsionStopNow();
}

int getMatchTimerFlag(void) {
	return(prMatchTimerFlag);
}




void msTimerInit(void) {
	T4CONbits.TON = 0;		// On arrête les timers 4 et 5 pour les configurer
	T5CONbits.TON = 0;
	T4CONbits.T32 = 1;		// On les combine en un timer 32 bits
	T4CONbits.TCKPS = 1;	// avec un prescaler de 1:8
	T4CONbits.TCS = 0;		// avec Tcy comme horloge   => compte par pas de 200ns
	PR4 = 0xFFFF;			// pour déborder le plus tard possible (+-860s)
	PR5 = 0xFFFF;
	TMR4 = 0;				// on initialise le compteur
	TMR5 = 0;
	IFS0bits.T3IF = 0; 		// Réinitialisation du flag d'interruption
	IEC0bits.T3IE = 1; 		// Activation de l'interruption
}


void msTimerStart(void) {
	TMR4 = 0;				// on initialise le compteur
	TMR5 = 0;
	IFS1bits.T5IF = 0;
	T4CONbits.TON = 1;
}


void msTimerStop(void) {
	T4CONbits.TON = 0;
}

unsigned long msTimerGet(void) {
    unsigned long time;

    time = TMR4;                        // on lit le LSW (le MSW est latché simultanément dans TMR5HLD)
    time |= (((long)TMR5HLD) << 16);    // on lit le MSW
    time /= 5000;                       // pour passer en ms (1ms/200ns = 5000)
    return (time);
}



void waitXms(int x) {
    msTimerStart();
    while(msTimerGet() < x);
    msTimerStop();
}
