
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

uint8_t SX1276_readRegister(uint8_t regAddress);
void SX1276_writeRegister(uint8_t regAddress, uint8_t value);

void main(void) {
    OSCTUNEbits.PLLEN = 1;
    LCDInit();
    lprintf(0, "SX1276");
    __delay_ms(2000);
    SX1276_Init(5);
    uint8_t reg = SX1276_readRegister(0x07);
    lprintf(1, "%02x", reg);
    SX1276_writeRegister(0x07, 0x33);
    reg = SX1276_readRegister(0x07);
    __delay_ms(1000);
    lprintf(1, "*%02x", reg);
    while (1) {
        
        __delay_ms(200);
    }
}


 