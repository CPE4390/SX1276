
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
    }
    //SX1276_SetFrequency(915000000);
    SX1276_SetChannel(13);
    SX1276_SetSignalBandwidth(BW125K);
    SX1276_SetSpreadingFactor(7);
    SX1276_SetCodingRate(5);
    SX1276_SetLNAGain(0, true);
    SX1276_SetHeaderMode(EXPLICIT_HEADER);
    SX1276_EnableCRC(true);
    SX1276_OptimizeRxPerErrata();
    lprintf(1, "Init done");
    while (1) {
        uint8_t rxBuffer[SX1276_MAX_PACKET_LENGTH];
        if (SX1276_ReceivePacket(rxBuffer, SX1276_MAX_PACKET_LENGTH, true)) {
            lprintf(0, "%s", rxBuffer);
            lprintf(1, "%d, %.2f", SX1276_PacketRSSI(), SX1276_PacketSNR());
        }
    }
}

void __interrupt(high_priority) HighISR(void) {
    
}

 