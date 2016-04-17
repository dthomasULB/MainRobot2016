#include <xc.h>
#include "../libdspic/clock.h"		// pour la définition de FCY
#include "../libdspic/uart.h"


#define UxMODE_UARTEN   (0x8000)
#define UxMODE_BRGH     (0x0008)
#define UxSTA_UTXEN     (0x0400)
#define UxSTA_UTXBF     (0x0200)
#define UxSTA_URXDA     (0x0001)
#define UxSTA_TMRT      (0x0100)
#define UxSTA_OERR      (0x0002)
#define UART1_RXIE      (IEC0bits.U1RXIE)
#define UART1_RXIF      (IFS0bits.U1RXIF)
#define UART1_TXIE      (IEC0bits.U1TXIE)
#define UART1_TXIF      (IFS0bits.U1TXIF)
#define UART2_RXIE      (IEC1bits.U2RXIE)
#define UART2_RXIF      (IFS1bits.U2RXIF)
#define UART2_TXIE      (IEC1bits.U2TXIE)
#define UART2_TXIF      (IFS1bits.U2TXIF)


//! Vecteur avec les pointeurs vers les registres de configuration
volatile unsigned int* const UxMODE[2] = {&U1MODE, &U2MODE};
//! Vecteur avec les pointeurs vers les registres de configuration
volatile unsigned int* const UxBRG[2] = {&U1BRG, &U2BRG};
//! Vecteur avec les pointeurs vers les registres de configuration
volatile unsigned int* const UxSTA[2] = {&U1STA, &U2STA};
//! Vecteur avec les pointeurs vers les registres de configuration
volatile unsigned int* const UxTXREG[2] = {&U1TXREG, &U2TXREG};
//! Vecteur avec les pointeurs vers les registres de configuration
volatile unsigned int* const UxRXREG[2] = {&U1RXREG, &U2RXREG};




void uartConfig(unsigned int channel, float baudrate, uartModeType uartMode) {
    float tmp;

    if ((channel > 2) ||(channel < 1)) {
        while(1);
    }
    *UxMODE[channel-1] = 0;                  // on désactive l'UART pour le configurer
    *UxMODE[channel-1] = uartMode;           // spécifie le mode de fonctionnement
    tmp = FCY / (16 * baudrate) - 1;        // on calcule la valeur exacte à placer dans le compteur pour obtenir le baudrate désiré
    if (tmp > 65535) {
        while(1);                           // baudrate trop lent, il faut diminuer FCY
    } else if (tmp < 50) {                  // si l'erreur max (0.5) sur tmp dépasse 1%, on passe en HiSpeed pour une meilleure résolution
        tmp = FCY / (4 * baudrate) - 1;
        *UxMODE[channel-1] |= UxMODE_BRGH;   // on configure le HiSpeed
    }
    *UxBRG[channel-1] = (unsigned int)tmp;   // on assigne la valeur calculée pour le baudrate, ce qui la tronque à l'entier inférieur
    if (tmp - *UxBRG[channel-1] > 0.5) {     // on vérifie si il n'aurait pas mieux valu arrondir à l'entier supérieur
        (*UxBRG[channel-1])++;                 // si oui, on le fait
    }
    *UxMODE[channel-1] |= UxMODE_UARTEN;      // on active l'UART
    *UxSTA[channel-1] |= UxSTA_UTXEN;        // on active l'émission (Attention, cela active aussi U1TXIF)
}


void uartSendChar(unsigned int channel, unsigned char data) {
    if ((channel > 2) ||(channel < 1)) {
        while(1);
    }
    while ((*(UxSTA[channel-1])) & UxSTA_UTXBF);
    *UxTXREG[channel-1] = data;
}


unsigned char uartGetChar(unsigned int channel) {
    if ((channel > 2) ||(channel < 1)) {
        while(1);
    }
    if (*UxSTA[channel-1] & UxSTA_OERR) {       // si il y a un overrun, on le clean (ATTENTION risque de perte de données))
        *UxSTA[channel-1] &= (~UxSTA_OERR);     // mais sinon, risique de blocage de l'uart 
    }
   while (!(*UxSTA[channel-1] & UxSTA_URXDA));
    return *UxRXREG[channel-1];
}

int uartRxDataAvailable(unsigned int channel) {
    if ((channel > 2) ||(channel < 1)) {
        while(1);
    }
    return (*UxSTA[channel-1] & UxSTA_URXDA);
}

int uartTxReady(unsigned int channel) {
    if ((channel > 2) ||(channel < 1)) {
        while(1);
    }
    return (!(*UxSTA[channel-1] & UxSTA_UTXBF));
}

int uartTxIdle(unsigned int channel) {
    if ((channel > 2) ||(channel < 1)) {
        while(1);
    }
    return ((*UxSTA[channel-1] & UxSTA_TMRT));
}

void uartRxIsrDisable(unsigned int channel) {
    if ((channel > 2) ||(channel < 1)) {
        while(1);
    }
    if (channel == 1) {
        UART1_RXIE = 0;
    } else {
        UART2_RXIE = 0;
    }
}

void uartTxIsrDisable(unsigned int channel) {
    if ((channel > 2) ||(channel < 1)) {
        while(1);
    }
    if (channel == 1) {
        UART1_TXIE = 0;
    } else {
        UART2_TXIE = 0;
    }
}

void uartRxIsrEnable(unsigned int channel) {
    if ((channel > 2) ||(channel < 1)) {
        while(1);
    }
    if (channel == 1) {
        UART1_RXIE = 1;
    } else {
        UART2_RXIE = 1;
    }
}

void uartTxIsrEnable(unsigned int channel) {
    if ((channel > 2) ||(channel < 1)) {
        while(1);
    }
    if (channel == 1) {
        UART1_TXIE = 1;
    } else {
        UART2_TXIE = 1;
    }
}
