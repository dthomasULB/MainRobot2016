#include "globals.h"


inline float satureAngle(float angle) {
	while (angle > PI)		angle -= 2*PI;
	while (angle < -PI)		angle += 2*PI;
	return(angle);
}
