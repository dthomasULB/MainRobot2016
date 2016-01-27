#include "configware.h"
#include "../libdspic/pps.h"
#include <xc.h>


// disables the JTAG to prevent conflicts with TRISA
_FICD(JTAGEN_OFF);
// disables the watchdog
_FWDT(FWDTEN_OFF);
// disable the write protection on memory segments
_FGS(GSS_OFF & GCP_OFF & GWRP_OFF);
// The dsPIC will be clocked by the primary oscillator with a 10MHz crystal.
// We want to use the PLL to obtain Fosc = 80MHz ( <=> 40MIPS ).
// Problem : with a 10MHz crystal, PLL constraints are not met with the
// default parameter.
// Solution :
//	- boot using the internal FRC oscillator as clock source,
//	- set the right PLL parameters to obtain Fosc = 80MHz, without violating
//	  the PLL constraints,
//	- switch the clock source to the PLL
//	- wait for the end of the clock switch
//
// Select internal FRC oscillator as clock source
_FOSCSEL(FNOSC_FRC);
// enables clock switching and configure the primary oscillator for a 10MHz crystal
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_XT);



// assignation des I/Os du CAN
void canPinAssign(void) {
	ppsInConfig(PPS_IN_C1RX, 10);	// RP10 -> CANRX
	ppsOutConfig(PPS_C1TX, 11);		// CANTX -> RP11
}
