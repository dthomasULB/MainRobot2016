/**
 * @file adc.h
 * @author bULBot 2011-2012
 * @author Olivier Desenfans
 * @brief Fichier header pour la configuration du convertisseur AD.
 */

#ifndef DEF_ADC_H
#define DEF_ADC_H

#include <xc.h>


void initADC(void);

/**
 * @brief R»alise une conversion analogique sur le canal sp»cifi» et renvoie le r»sultat.
 * @returns R»sultat de la conversion analogique sur 10 bits.
 */
int conversionADC(unsigned char canal);


#endif
