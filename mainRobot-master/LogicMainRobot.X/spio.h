/**
 * @file SPIO.h
 * @author bULBot 2011-2012
 * @author Olivier Desenfans
 * @brief Header d'assignation des SPIO. 
 *
 */
 
#ifndef DEF_SPIO_H
#define DEF_SPIO_H 

#include <xc.h>

//! Définition de la pin du bouton de choix d'équipe. 
#define BOUTON_EQUIPE   (PORTCbits.RC2)
//! Définition de la pin de la goupille de démarrage. 
#define GOUPILLE_OTEE   (PORTCbits.RC3)

#define LED             (LATCbits.LATC1)

void assignSPIO(void); 

#endif
