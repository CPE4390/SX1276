
#include <xc.h>
#include "SX1276.h"

//pins
#define NSS     (LATDbits.LATD7)
#define REST    (LATDbits.LATD3)

// registers
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_OCP                  0x0b
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_RSSI_VALUE           0x1b
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_FREQ_ERROR_MSB       0x28
#define REG_FREQ_ERROR_MID       0x29
#define REG_FREQ_ERROR_LSB       0x2a
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_INVERTIQ             0x33
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_INVERTIQ2            0x3b
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42
#define REG_PA_DAC               0x4d

// modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06
#define MODE_CAD                 0x07

// PA config
#define PA_BOOST                 0x80

// IRQ masks
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40
#define IRQ_CAD_DONE_MASK          0x04
#define IRQ_CAD_DETECTED_MASK      0x01

#define RF_MID_BAND_THRESHOLD    525000000L
#define RSSI_OFFSET_HF_PORT      157
#define RSSI_OFFSET_LF_PORT      164

#define MAX_PKT_LENGTH           255

#define _XTAL_FREQ  32000000

static void initSPI(void);
static uint8_t spiTransfer(uint8_t byte);
uint8_t SX1276_readRegister(uint8_t regAddress);
void SX1276_writeRegister(uint8_t regAddress, uint8_t value);
uint8_t SX1276_readFIFO(uint8_t *buffer, uint8_t len);
uint8_t SX1276_writeFIFO(uint8_t *buffer, uint8_t len);

void SX1276_Init(uint8_t channel) {
    initSPI();
    LATDbits.LATD3 = 0; //Put SX1276 in reset
    TRISDbits.TRISD3 = 0; //REST pin
    TRISBbits.TRISB1 = 1; //DIO0 used for INT0 input
    __delay_ms(10);
    LATDbits.LATD3 = 1; //Release from reset
    __delay_ms(10);
    
}

char SX1276_SendPacket(uint8_t *data, int len) {
    
}

int SX1276_ReceivePacket(uint8_t *data, int maxLen) {
    
}

void SX1276_HandleRxInt(void) {
    
}

void SX1276_HandleTxInt(void) {
    
}


void initSPI(void) {
    NSS = 1;
    TRISDbits.TRISD7 = TRISDbits.TRISD6 = TRISDbits.TRISD4 = 0; //RD7, RD6, RD4 outputs
    TRISDbits.TRISD5 = 1; //RD5 is input (MISO)
    //Configure MSSP 2 for SPI at 8 MHz, mode 0,0
	SSP2STATbits.CKE = 1;
	SSP2CON1bits.CKP = 0;  //SPI mode 0,0
	SSP2CON1bits.SSPM = 0b0000;	//SPI Master - FOSC/4 = 8 MHz
    PIR3bits.SSP2IF = 0;
	SSP2CON1bits.SSPEN = 1;	//Enable MSSP
}

uint8_t spiTransfer(uint8_t byte) {
    uint8_t b;
	SSP2BUF = byte; //transmit byte
	while(!PIR3bits.SSP2IF); //Wait until completed
	PIR3bits.SSP2IF = 0; //Clear flag so it is ready for next transfer
    return SSP2BUF;  //return received byte
}

uint8_t SX1276_readRegister(uint8_t regAddress) {
    regAddress &= 0b01111111; //Clear r/w bit for read
    NSS = 0;
    spiTransfer(regAddress);
    uint8_t value = spiTransfer(0);
    NSS = 1;
    return value;
}

void SX1276_writeRegister(uint8_t regAddress, uint8_t value) {
    regAddress |= 0b10000000; //Set r/w bit for write
    NSS = 0;
    spiTransfer(regAddress);
    spiTransfer(value);
    NSS = 1;
}