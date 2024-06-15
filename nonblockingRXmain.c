
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

void onRxDone(uint8_t len);

volatile bool newPacket;
volatile uint8_t rxBuffer[SX1276_MAX_PACKET_LENGTH];
volatile uint8_t packetLength;

void main(void) {
    OSCTUNEbits.PLLEN = 1;
    LCDInit();
    lprintf(0, "SX1276");
    __delay_ms(1000);
    if (!SX1276_Init()) {
        lprintf(1, "SX1276 Not Found");
        while (1);
    }
    INTCON2bits.INTEDG1 = 1; //Rising edge
    INTCON3bits.INT1IE = 1;
    INTCON3bits.INT1IF = 0;
    INTCONbits.GIE = 1;
    SX1276_SetFrequency(915000000);
    SX1276_SetSignalBandwidth(BW125K);
    SX1276_SetSpreadingFactor(7);
    SX1276_SetRXDoneCallback(&onRxDone);
    lprintf(1, "Init done");
    newPacket = false;
    SX1276_ReceivePacket((uint8_t *)rxBuffer, SX1276_MAX_PACKET_LENGTH, false);
    while (1) {
        if (newPacket) {
            newPacket = false;
            lprintf(0, "%s", rxBuffer);
        }
    }
}

void __interrupt(high_priority) HighISR(void) {
    if (INTCON3bits.INT1IF) {
        INTCON3bits.INT1IF = 0;
        SX1276_HandleDIO0Int();
    }
}

void onRxDone(uint8_t len) {
    if (len) {
        newPacket = true;
    }
}

