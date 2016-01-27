/**
 * @file pwm.c
 * @brief Driver Servo PWM.
 * @author Yernaux
 * @author bULBot 2013-2014
 */

#include "../libdspic/servo.h"
#include "../libdspic/pwm.h"

#define ABS(a)		(((a) < 0) ? -(a) : (a))


typedef struct {
    //public:
    int angleMax;           // [deg]
    int angleMin;           // [deg]
    int angle;              // angle actuel
    float pulseMinMs;       // Durée minimum du pulse, en ms (correspondant à l'angle min)
    float pulseMaxMs;       // Durée maximum du pulse, en ms (correspondant à l'angle min)
    float angleToDuration;  // rapport pour transformer un angle en durée de pulse
} servoType;

servoType prServoList[NB_SERVO_MAX];
float prServoPeriod = 0;


void servoSetAngle(int servoId, int angle) {
    if ((servoId < 0) || (servoId >= NB_SERVO_MAX))                                             while(1);           // paramètres hors gamme
    if ((angle < prServoList[servoId].angleMin) || (angle > prServoList[servoId].angleMax))     while(1);           // paramètres hors gamme
    
    prServoList[servoId].angle = angle;                                                                             // on met à jour les infos du servo
    pwmSetDuration(servoId+1, prServoList[servoId].pulseMinMs+angle*prServoList[servoId].angleToDuration);          // on change le DC de la PWM
}

void servoDisable(int servoId) {
    if ((servoId < 0) || (servoId >= NB_SERVO_MAX))     while(1);           // paramètres hors gamme
    
    pwmSetDuration(servoId+1, 0);       // on met le DC à zéro -> plus de pulse -> servo en "roue libre"
}


void servoSetAngleLimits(int servoId, int angleMin, int angleMax) {
    if ((servoId < 0) || (servoId >= NB_SERVO_MAX))     while(1);           // paramètres hors gamme
    if ((angleMin <= 0) || (angleMin > 360))            while(1);
    if ((angleMax <= 0) || (angleMax > 360))            while(1);
    if ((angleMin >= angleMax))                         while(1);

    prServoList[servoId].angleMin = angleMin;
    prServoList[servoId].angleMax = angleMax;
    prServoList[servoId].angleToDuration = (prServoList[servoId].pulseMaxMs - prServoList[servoId].pulseMinMs) / (prServoList[servoId].angleMax - prServoList[servoId].angleMin);
}

void servoSetPulseLimits(int servoId, float pulseMin, float pulseMax) {
    if ((servoId < 0) || (servoId >= NB_SERVO_MAX))     while(1);           // paramètres hors gamme
    if ((pulseMin <= 0) || (pulseMin > prServoPeriod))  while(1);
    if ((pulseMax <= 0) || (pulseMax > prServoPeriod))  while(1);
    if ((pulseMin >= pulseMax))                         while(1);

    prServoList[servoId].pulseMinMs = pulseMin;
    prServoList[servoId].pulseMaxMs = pulseMax;
    prServoList[servoId].angleToDuration = (prServoList[servoId].pulseMaxMs - prServoList[servoId].pulseMinMs) / (prServoList[servoId].angleMax - prServoList[servoId].angleMin);
}


void servoInit(int nbServo, int timerId, float periodMs) {
    int servoId;
    
    if ((nbServo <= 0) || (nbServo > NB_SERVO_MAX))     while(1);           // paramètres hors gamme
    if ((timerId != TIMER_2) && (timerId != TIMER_3))   while(1);
    if ((periodMs < 0.33) || (periodMs > 20))           while(1);

    prServoPeriod = periodMs;
    for(servoId=0; servoId<nbServo; servoId++) {
        pwmSetup(servoId+1, timerId, periodMs);
        prServoList[servoId].angleMax = 180;                // [deg]
        prServoList[servoId].angleMin = 0;                  // [deg]
        prServoList[servoId].angle = 0;                     // [deg]
        prServoList[servoId].pulseMaxMs = 2;                // [ms]
        prServoList[servoId].pulseMinMs = 1;                // [ms]
        prServoList[servoId].angleToDuration = (prServoList[servoId].pulseMaxMs - prServoList[servoId].pulseMinMs) / (prServoList[servoId].angleMax - prServoList[servoId].angleMin);
    }
}

void SoftServo(int servoId, int angleDeb, int angleFin, int delayMs) {
    int i = 0;
    int pas;
    float tmp;

    tmp = delayMs/ABS(angleDeb-angleFin);
    pas = (int)(tmp + 0.5);


    for (i = 1; i < ABS(angleDeb - angleFin); i++) {
        if (angleDeb < angleFin) {
            servoSetAngle(servoId, angleDeb + i);
        } else {
            servoSetAngle(servoId, angleDeb - i);
        }
        __delay_ms(pas);
    }
}
