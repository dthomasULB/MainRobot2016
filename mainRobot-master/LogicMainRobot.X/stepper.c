/*
 * File:   stepper.c
 * Author: DenisT
 *
 * Created on 15 avril 2016, 23:41
 */


#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include <math.h>
#include "timer.h"
#include "../libdspic/clock.h"
#include <libpic30.h>
#include "stepper.h"
#include "Globals.h"

char pattern[] = {0b10000011, 0b11100000, 0b00111000, 0b00001110};
//long position = 0;


#define IN1 LATCbits.LATC0
#define IN2 LATCbits.LATC1
#define IN3 LATCbits.LATC2
#define IN4 LATBbits.LATB9

// Move by 'd' deg at a speed of 'v' deg/s
void moveStepper(int d){
    // d deg / 360 deg * 400 steps/rev
    long position = 0;
    int steps = ((d*400.0)/360.0);
    long target = position + steps;
    long dt=625;
    
    if(d > 0){
        for(position; position < target; position++){
            int input = (ABS(position%8));
            char in = (1 << input);
            char a1 = pattern[0] & in;
            char a2 = pattern[1] & in;
            char a3 = pattern[2] & in;
            char a4 = pattern[3] & in;
            if(a1){
                IN1 = 1;
            }else{
                IN1 = 0;
            }
            if(a2){
                IN2 = 1;
            }else{
                IN2 = 0;
            }
            if(a3){
                IN3 = 1;
            }else{
                IN3 = 0;
            }
            if(a4){
                IN4 = 1;
            }else{
                IN4 = 0;
            }
            __delay_us(dt);
        }
    }else{
        for(position = -1; position > target; position--){
            int input = (ABS((position%8)+8));
            char in = (1 << input);
            char a1 = pattern[0] & in;
            char a2 = pattern[1] & in;
            char a3 = pattern[2] & in;
            char a4 = pattern[3] & in;
            if(a1){
                IN1 = 1;
            }else{
                IN1 = 0;
            }
            if(a2){
                IN2 = 1;
            }else{
                IN2 = 0;
            }
            if(a3){
                IN3 = 1;
            }else{
                IN3 = 0;
            }
            if(a4){
                IN4 = 1;
            }else{
                IN4 = 0;
            }
            __delay_us(dt);
        }
    }
    IN1 = 0;    // Chauffe et consomme bcp donc on arrête la blocage
    IN2 = 0;
    IN3 = 0;
    IN4 = 0;
}

void ioInit(){
    AD1PCFGL = 0xFFFF;
    TRISCbits.TRISC0 = 0;
    TRISCbits.TRISC1 = 0;
    TRISCbits.TRISC2 = 0;
    TRISBbits.TRISB9 = 0;
}

void initStepper(){
    ioInit();
}

void ouvrirPorte(){
moveStepper(1550);
}
void fermerPorte(){
moveStepper(-1550);
}
                    