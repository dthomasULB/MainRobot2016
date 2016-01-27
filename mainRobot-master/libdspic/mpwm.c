/*!
 * @file pwm.c
 * @author Kevin De Cuyper
 * @brief Fonctions pour la gestion des pwm
 */
#include <xc.h>
#include "mpwm.h"
#include "clock.h"

volatile unsigned int* const PxTPER[8] = {&P1TPER, &P1TPER, &P1TPER, &P1TPER, &P1TPER, &P1TPER, &P2TPER, &P2TPER};
volatile unsigned int* const PxTCON[8] = {&P1TCON, &P1TCON, &P1TCON, &P1TCON, &P1TCON, &P1TCON, &P2TCON, &P2TCON};
volatile unsigned int* const PxDCy[8] = {&P1DC1, &P1DC1, &P1DC2, &P1DC2, &P1DC3, &P1DC3, &P2DC1, &P2DC1};

mpwmStatus mpwmSetup(int id, float periodMs) {
    float PRval;
    int prescale;

    //on vérifie si l'ID de la pwm est correcte
    if ((id < MPWM1L1) || (id > MPWM2H1)) {
        return MPWM_ID_ERROR;
    }
    // configuration de la patte correpondante
    // les paires de sorties MPWM (MPWMxLy, MPWMxHy) sont configurées en mode indépendant, pour que celle non utilisées reste une GPIO
    PWM1CON1 |= 0x0F00;
    PWM2CON1 |= 0x0100;
    switch (id) {
        case MPWM1L1:    PWM1CON1bits.PEN1L = 1;    break;
        case MPWM1H1:    PWM1CON1bits.PEN1H = 1;    break;
        case MPWM1L2:    PWM1CON1bits.PEN2L = 1;    break;
        case MPWM1H2:    PWM1CON1bits.PEN2H = 1;    break;
        case MPWM1L3:    PWM1CON1bits.PEN3L = 1;    break;
        case MPWM1H3:    PWM1CON1bits.PEN3H = 1;    break;
        case MPWM2L1:    PWM2CON1bits.PEN1L = 1;    break;
        case MPWM2H1:    PWM2CON1bits.PEN1H = 1;    break;
    }
    // Configuration de la période
    PRval = (float)(FCY/1000)*periodMs-1;
    if (PRval <= 0) {
        return MPWM_PERIOD_ERROR;
    }
    if (PRval <= 0x7FFF) {
        prescale = 0;
    } else if (PRval/4 <= 0x7FFF) {
        PRval /= 4;
        prescale = 1;
    } else if (PRval/16 <= 0x7FFF) {
        PRval /= 16;
        prescale = 2;
    } else if (PRval/64 <= 0x7FFF) {
        PRval /= 64;
        prescale = 3;
    } else {
        return MPWM_PERIOD_ERROR;
    }
    *PxTPER[id] = PRval;                    // définit la fréquence des PWM
    *PxTCON[id] = 0x8000 | (prescale<<2);  // activation de la PWM

    return MPWM_SUCCESS;
}


mpwmStatus mpwmSetDutyCycle(int id, float dutyCycle) {
    //on vérifie si l'ID de la pwm est correcte
    if ((id < MPWM1L1) || (id > MPWM2H1)) {
        return MPWM_ID_ERROR;
    }
    //saturation du rapport cyclique 
    if (dutyCycle > 1.0) {
        dutyCycle = 1.0;
    } else if (dutyCycle < 0) {
        dutyCycle = 0;
    }
    *PxDCy[id] = (float)(*PxTPER[id] + 1) * dutyCycle*2;    // les MPWM ont une résolution d'1/2 période pour le DC

    return MPWM_SUCCESS;
}


mpwmStatus mpwmSetDuration(int id, float durationMs) {
//
//    int PRx;
//    char prescaleShift[4] = {0,3,6,8}; //valeur de dÃ©calage Ã  droite correspondant au prescaler
//
//    //conversion de la durÃ©e en cycles machine
//    long OCx = (float) (FCY / 1000) * durationMs - 1;
//
//    //on vÃ©rifie si l'ID de la pwm est correcte
//    if (id != PWM_1 && id != PWM_2 && id != PWM_3 && id != PWM_4) {
//        return PWM_ID_ERROR;
//    }
//
//    //si la PWM est branchÃ©e sur le timer 2
//    if (0 == ((*OCxCON[id - 1])& 0b1000)) {
//        PRx = PR2;
//        OCx >>= prescaleShift[T2CONbits.TCKPS]; //on tient compte du prescaler original
//    } else {
//        PRx = PR3;
//        OCx >>= prescaleShift[T3CONbits.TCKPS];
//    }
//
//    //saturation du rapport cyclique
//    if (OCx > PRx) {
//        OCx = PRx;
//    } else if (OCx < 0) {
//        OCx = 0;
//    }
//
//    //modification du rapport cyclique
//    *OCxRS[id - 1] = OCx;

    return MPWM_SUCCESS;
}

mpwmStatus mpwmSetCycle(int id, float duration) {

//    int PRx;
//
//    //conversion de la durÃ©e en cycles machine
//    int OCx = duration;
//
//    //on vÃ©rifie si l'ID de la pwm est correcte
//    if (id != PWM_1 && id != PWM_2 && id != PWM_3 && id != PWM_4) {
//        return PWM_ID_ERROR;
//    }
//
//    //si la PWM est branchÃ©e sur le timer 2
//    if (0 == ((*OCxCON[id - 1])& 0b1000)) {
//        PRx = PR2;
//    } else {
//        PRx = PR3;
//    }
//
//    //saturation du rapport cyclique
//    if (OCx > PRx) {
//        OCx = PRx;
//    } else if (OCx < 0) {
//        OCx = 0;
//    }
//
//    //modification du rapport cyclique
//    *OCxRS[id - 1] = OCx;

    return MPWM_SUCCESS;
}

mpwmStatus mpwmEnable(int id){
//        //on vÃ©rifie si l'ID de la pwm est correcte
//    if (id != PWM_1 && id != PWM_2 && id != PWM_3 && id != PWM_4) {
//        return PWM_ID_ERROR;
//    }
//
//    *OCxCON[id - 1] |= 0b0110;

    return MPWM_SUCCESS;
}

mpwmStatus mpwmDisable(int id){
//        //on vÃ©rifie si l'ID de la pwm est correcte
//    if (id != PWM_1 && id != PWM_2 && id != PWM_3 && id != PWM_4) {
//        return PWM_ID_ERROR;
//    }
//
//    *OCxCON[id - 1] &= 0xFFF8;

    return MPWM_SUCCESS;
}