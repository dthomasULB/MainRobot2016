/**
 * @file SPIO.c
 * @author bULBot 2011-2012
 * @author Olivier Desenfans
 * @brief Assigne les fonctions sp�ciales aux pattes d'I/O. 
 * @details Configuration des registres d'assignation des fonctions sp�ciales (UART, CAN) 
 * aux pattes d'entr�es sorties (Special Purpose I/O).
 */

#include "spio.h"
#include "../libdspic/pps.h"
#include <xc.h>

//! Fonction d'assignation des I/O. 
void assignSPIO(void) {
    // bouton choix d'�quipe
    TRISCbits.TRISC2 = 1;
    AD1PCFGLbits.PCFG8 = 1;
    // goupille de d�marrage
    TRISCbits.TRISC3 = 1; 
    // LED
    LED = 0;
    TRISCbits.TRISC1 = 0;
    AD1PCFGLbits.PCFG7 = 1;
    // CAN 
    ppsOutConfig(PPS_C1TX, 11);
    ppsInConfig(PPS_IN_C1RX, 10);
    // Sortie pour le servo    
    ppsOutConfig(PPS_OC1, 7);
}

