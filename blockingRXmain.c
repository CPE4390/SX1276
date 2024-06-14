
#include <xc.h>
#include "LCD.h"
#include "SX1276.h"
#include <string.h>
#include <stdio.h>

//MISO -> RD5
//MOSI -> RD4
//SCK -> RD6
//NSS -> RD7
//REST -> RD3
//DIO0 -> RB1
//VCC -> 3.3V
//GND -> GND

void main(void) {
    OSCTUNEbits.PLLEN = 1;
    LCDInit();
    lprintf(0, "SX1276");
    __delay_ms(1000);
    if (!SX1276_Init()) {
        lprintf(1, "SX1276 Not Found");
        while (1);
    };
    SX1276_SetFrequency(915000000);
    SX1276_SetSignalBandwidth(BW125K);
    SX1276_SetSpreadingFactor(7);
    SX1276_SetCodingRate(5);
    
    lprintf(1, "Init done");
    int txCount = 0;
    while (1) {
        __delay_ms(1000);
        char msg[17];
        sprintf(msg, "Hello World #%d", txCount);
        bool success = SX1276_SendPacket((uint8_t *)msg, (uint8_t)(strlen(msg) + 1), true);
        if (success) {
            ++txCount;
            lprintf(0, "TX Count = %d ", txCount);
        } else {
            lprintf(0, "%d", success);
        }
    }
}

void __interrupt(high_priority) HighISR(void) {
    
}

 