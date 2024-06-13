
#include <xc.h>
#include "LCD.h"
#include "SX1276.h"

//MISO -> RD5
//MOSI -> RD4
//SCK -> RD6
//NSS -> RD7
//REST -> RD3
//DIO0 -> RB1
//VCC -> 3.3V
//GND -> GND

uint8_t readRegister(uint8_t regAddress);
void writeRegister(uint8_t regAddress, uint8_t value);

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
    SX1276_SetLNAGain(0, true);
    SX1276_SetTransmitPower(14, false);
    //SX1276_Standby();
    lprintf(1, "Init done");
    uint8_t rh = readRegister(0x0b);
    uint8_t rm = readRegister(0x26);
    uint8_t rl = readRegister(0x0c);
    lprintf(1, "%02x %02x %02x", rh, rm, rl);
    while (1) {
        
        __delay_ms(200);
    }
}


 