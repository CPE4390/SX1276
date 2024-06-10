
#include <xc.h>
#include "LCD.h"
#include "SX1276.h"

//MISO ->
//MOSI ->
//SCK ->
//NSS ->
//REST ->
//DIO2 ->
//VCC -> 3.3V
//GND -> GND

void main(void) {
    OSCTUNEbits.PLLEN = 1;
    LCDInit();
    lprintf(0, "SX1276");
    __delay_ms(2000);
    while (1) {
        
        __delay_ms(200);
    }
}


 