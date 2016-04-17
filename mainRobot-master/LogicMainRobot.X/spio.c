/**
 * @file SPIO.c
 * @author bULBot 2011-2012
 * @author Olivier Desenfans
 * @brief Assigne les fonctions spéciales aux pattes d'I/O. 
 * @details Configuration des registres d'assignation des fonctions spéciales (UART, CAN) 
 * aux pattes d'entrées sorties (Special Purpose I/O).
 */

#include "spio.h"
#include "../libdspic/pps.h"
#include <xc.h>

//! Fonction d'assignation des I/O. 
void assignSPIO(void) {
   /*
    EN COMMENTAIRE EN TESTANT LE STEPPER
    // bouton choix d'équipe
   
    TRISCbits.TRISC2 = 1;
    AD1PCFGLbits.PCFG8 = 1;
    */
    // goupille de démarrage
    AD1PCFGLbits.PCFG7 = 1;
    TRISCbits.TRISC1 = 1;
  /*
    // SW WAKE
    AD1PCFGLbits.PCFG6 = 1;
    TRISCbits.TRISC0=0;
*/

    // CAN 
    ppsOutConfig(PPS_C1TX, 11);
    ppsInConfig(PPS_IN_C1RX, 10);
    
  
    
    // Stepper
      AD1PCFGL = 0xFFFF;
    TRISCbits.TRISC0 = 0;
    TRISCbits.TRISC1 = 0;
    TRISCbits.TRISC2 = 0;
    TRISCbits.TRISC3 = 0;
}

