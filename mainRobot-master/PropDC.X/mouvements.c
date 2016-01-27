#include "Globals.h"
#include "consigne.h"
#include <stdlib.h>
#include <math.h>


void addTranslation(translationParamType param) {
	relativePosType tmp;

	tmp.r = 0;
	tmp.l = param.acc;
	csgSetNomAcc(tmp);
	tmp.l = param.vel;
	csgSetNomVel(tmp);
	tmp = csgGetPos();
	tmp.l += param.length;
	csgSetFinalPos(tmp);
}


void addRotation(rotationParamType param) {
	relativePosType tmp;

	tmp.l = 0;
	tmp.r = param.acc;
	csgSetNomAcc(tmp);
	tmp.r = param.vel;
	csgSetNomVel(tmp);
	tmp = csgGetPos();
	tmp.r += param.angle;
	csgSetFinalPos(tmp);
}


void stopNow(void) {
	relativePosType vel, acc, pos;

	vel = csgGetVel();
	acc = csgGetNomAcc();
	pos = csgGetPos();
	if (vel.l >0) {
		pos.l += vel.l*vel.l/(2*acc.l);
	} else{
		pos.l -= vel.l*vel.l/(2*acc.l);
	}
	if (vel.r >0) {
		pos.r += vel.r*vel.r/(2*acc.r);
	} else{
		pos.r -= vel.r*vel.r/(2*acc.r);
	}
	csgSetFinalPos(pos);
}


relativePosType calcSegment(absolutePosType curPos, absolutePosType newPos) {
	float x, y;
	 relativePosType seg;

	x = newPos.x - curPos.x;
	y = newPos.y - curPos.y;
	seg.l = sqrt(x*x + y*y);
	seg.r = satureAngle(atan2(y, x) - curPos.alpha);
	return(seg);
}
