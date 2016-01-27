#ifndef GLOBALS_H
#define GLOBALS_H

#include "../can/CanTypes.h"

typedef struct {
 float x;			// [m]
 float y;			// [m]
 float alpha;		// [rad]
} absolutePosType;

typedef struct  {
    float l;		// [m]
    float r;		// [rad]
} relativePosType;

typedef struct {
    float acc;      // [m]
    float vel;      // [m/s]
    float length;   // [m/s²]
} translationParamType;

typedef struct {
    float acc;      // [rad]
    float vel;      // [rad/s]
    float angle;   // [rad/s²]
} rotationParamType;


positionInteger positionFloatToInteger(absolutePosType posFl);
relativeCoordInteger relativeCoordFloatToInteger(relativePosType posFl);
float satureAngle(float angle);		// ramène l'angle dans l'intervalle -pi -> +pi

#define PI (3.14159)

#define MAX(a, b)   (((a) < (b)) ? (b) : (a))
#define MIN(a, b)   (((a) > (b)) ? (a) : (b))
#define ABS(a)		(((a) < 0) ? -(a) : (a))
#define SIGN(a)		(((a) < 0) ? -1 : 1)


#endif