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

        // Denis: exclusion du cas 0 qui est problématique 17/12/2015
    // *2 en plus sur l'acc pour freiner plus vite
	if (vel.l >0) {
		pos.l += vel.l*vel.l/(3*2*acc.l);
	} else if(vel.l < 0){
		pos.l -= vel.l*vel.l/(3*2*acc.l);
	}
	if (vel.r >0) {
		pos.r += vel.r*vel.r/(3*2*acc.r);
	} else if(vel.r < 0){
		pos.r -= vel.r*vel.r/(3*2*acc.r);
	}
	csgSetFinalPos(pos);
}


relativePosType calcSegment(absolutePosType curPos, absolutePosType newPos, int backward) {
	float x, y;
	 relativePosType seg;

	x = newPos.x - curPos.x;
	y = newPos.y - curPos.y;
	seg.l = sqrt(x*x + y*y);
	seg.r = satureAngle(atan2(y, x) - curPos.alpha);
    if (backward ==1){
        seg.l=-seg.l;
        seg.r = seg.r - PI;
        while (seg.r > PI){seg.r -= 2*PI;}
        while (seg.r < -PI){seg.r += 2*PI;}
    }
	return(seg);
}

