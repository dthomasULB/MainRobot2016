#include "../libdspic/servo.h"

#define SERVO_CLAP  (0)     // ID du servo du bras pour faire tomber les claps
#define CLAP_POS_RENTRE     (0)
#define CLAP_POS_SORTI      (104)

void initServoClap(void) {
    servoInit(1, TIMER_2, 10);
    servoSetPulseLimits(SERVO_CLAP, 0.27, 2.45); // adapte les durées min et max des pulses pour utiliser toute la plage du servo
    servoSetAngle(SERVO_CLAP, CLAP_POS_RENTRE); // rentre le bras du clap
}

void rentrerServoClap(void) {
    servoSetAngle(SERVO_CLAP, CLAP_POS_RENTRE);
}

void sortirServoClap(void) {
    servoSetAngle(SERVO_CLAP, CLAP_POS_SORTI);
}
