
#include <xc.h>
#include "LCD.h"
#include "SX1276.h"
#include <stdio.h>
#include <string.h>

//MISO -> RD5
//MOSI -> RD4
//SCK -> RD6
//NSS -> RD7
//REST -> RD3
//DIO0 -> RB1
//VCC -> 3.3V
//GND -> GND

volatile int txCount = 0;

void onTxDone(void);

void main(void) {
    OSCTUNEbits.PLLEN = 1;
    LCDInit();
    lprintf(0, "SX1276");
    __delay_ms(1000);
    if (!SX1276_Init()) {
        lprintf(1, "SX1276 Not Found");
        while (1);
    };
    INTCON2bits.INTEDG1 = 1; //Rising edge
    INTCON3bits.INT1IE = 1;
    INTCON3bits.INT1IF = 0;
    INTCONbits.GIE = 1;
    SX1276_SetFrequency(915000000);
    SX1276_SetSignalBandwidth(BW125K);
    SX1276_SetSpreadingFactor(12);
    SX1276_SetCodingRate(5);
    SX1276_SetTransmitPower(14, PA_PABOOST_OUTPUT);  //must use PABOOST for these boards
    SX1276_SetTXDoneCallback(&onTxDone);
    lprintf(1, "Init done");
    while (1) {
        __delay_ms(1000);
        lprintf(0, "TX Count = %d ", txCount);
        char msg[17];
        sprintf(msg, "Hello World #%d", txCount);
        SX1276_SendPacket((uint8_t *) msg, (uint8_t) (strlen(msg) + 1), false);
    }
}

void __interrupt(high_priority) HighISR(void) {
    if (INTCON3bits.INT1IF) {
        INTCON3bits.INT1IF = 0;
        SX1276_HandleDIO0Int();
    }
}

void onTxDone(void) {
    ++txCount;
}

