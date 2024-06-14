
#include <xc.h>
#include "SX1276.h"

//Disable warning about functions used in ISR
#pragma warning disable 1510

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
#define MODE_LORA                0x80
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

#define MAX_PACKET_LENGTH           128

#define _XTAL_FREQ  32000000

#define TX_FIFO_BASE    0x00
#define RX_FIFO_BASE    0x80

enum {
    DIO0_TXDONE = 0x40, DIO0_RXDONE = 0x00, DIO0_CADDONE = 0x80
};

//private variables
static uint32_t nominalFrequency;
static bool crcEnabled;
static void (*onTxDone)(void);
static void (*onRxDone)(void);
static void (*onCadDone)(bool);

static enum _headerMode {
    IMPLICIT_HEADER = 1, EXPLICIT_HEADER = 0
} headerMode;

//private helper functions
static void initSPI(void);
static uint8_t spiTransfer(uint8_t byte);
static uint8_t readRegister(uint8_t regAddress);
static void writeRegister(uint8_t regAddress, uint8_t value);
static void readFIFO(uint8_t *buffer, uint8_t len);
static void writeFIFO(uint8_t *buffer, uint8_t len);
static void setOpMode(uint8_t mode);
static void setHeaderMode(enum _headerMode mode);
static bool setLDO(void);
static uint8_t getSpreadingFactor(void);
static uint32_t getBandwidthHz(void);

bool SX1276_Init(void) {
    initSPI();
    LATDbits.LATD3 = 0; //Put SX1276 in reset
    TRISDbits.TRISD3 = 0; //REST pin
    TRISBbits.TRISB1 = 1; //DIO0 used for INT0 input
    __delay_ms(10);
    LATDbits.LATD3 = 1; //Release from reset
    __delay_ms(10);
    SX1276_Sleep();
    if (readRegister(REG_VERSION) != 0x12) {
        return false;
    }
    writeRegister(REG_FIFO_TX_BASE_ADDR, TX_FIFO_BASE);
    writeRegister(REG_FIFO_RX_BASE_ADDR, RX_FIFO_BASE);
    SX1276_SetFrequency(915000000);
    headerMode = EXPLICIT_HEADER;
    crcEnabled = false;
    onTxDone = NULL;
    onRxDone = NULL;
    onCadDone = NULL;
    return true;
}

void SX1276_SetFrequency(uint32_t frequency) {
    if (frequency < 902300000 || frequency > 927500000) {
        return;
    }
    nominalFrequency = frequency;
    uint32_t bw = getBandwidthHz();
    if (bw <= 41670) {
        frequency += bw;
    }
    uint32_t frf = (uint32_t) ((((uint64_t) frequency) << 19) / 32000000);
    writeRegister(REG_FRF_MSB, (uint8_t) (frf >> 16));
    writeRegister(REG_FRF_MID, (uint8_t) (frf >> 8));
    writeRegister(REG_FRF_LSB, (uint8_t) frf);
}

void SX1276_SetChannel(uint8_t channel) {
    if (channel > 63) {
        return;
    }
    uint32_t frequency;
    frequency = 902300000 + channel * 200000;
    SX1276_SetFrequency(frequency);
}

void SX1276_SetSpreadingFactor(uint8_t sf) {
    if (sf < 6 || sf > 12) {
        return;
    }
    uint32_t bw = getBandwidthHz();
    if (sf == 6) {
        setHeaderMode(IMPLICIT_HEADER);
        writeRegister(REG_DETECTION_OPTIMIZE, bw == 500000 ? 0xc5 : 0x45);
        writeRegister(REG_DETECTION_THRESHOLD, 0x0c);
    } else {
        setHeaderMode(EXPLICIT_HEADER);
        writeRegister(REG_DETECTION_OPTIMIZE, bw == 500000 ? 0xc3 : 0x43);
        writeRegister(REG_DETECTION_THRESHOLD, 0x0a);
    }
    uint8_t reg = readRegister(REG_MODEM_CONFIG_2);
    reg &= 0x0f;
    reg |= sf << 4;
    writeRegister(REG_MODEM_CONFIG_2, reg);
    setLDO();
}

void SX1276_SetCodingRate(uint8_t cr) {
    if (cr < 5 || cr > 8) {
        return;
    }
    cr -= 4;
    cr &= 0x07;
    uint8_t reg = readRegister(REG_MODEM_CONFIG_1);
    reg &= 0xf1;
    reg |= cr << 1;
    writeRegister(REG_MODEM_CONFIG_1, reg);
}

void SX1276_SetSignalBandwidth(enum SX1276_BANDWIDTH bw) {
    bw &= 0x0f;
    uint8_t reg = readRegister(REG_MODEM_CONFIG_1);
    reg &= 0x0f;
    reg |= bw << 4;
    writeRegister(REG_MODEM_CONFIG_1, reg);
    setLDO();
    //Optimize detection - see errata
    uint32_t bwHz = getBandwidthHz();
    uint32_t frequency = nominalFrequency;
    if (bwHz <= 41670) {
        frequency += bwHz;
    }
    uint32_t frf = (uint32_t) ((((uint64_t) frequency) << 19) / 32000000);
    writeRegister(REG_FRF_MSB, (uint8_t) (frf >> 16));
    writeRegister(REG_FRF_MID, (uint8_t) (frf >> 8));
    writeRegister(REG_FRF_LSB, (uint8_t) frf);
    uint8_t IfFreq2;
    switch (bw) {
        case BW7_8K: IfFreq2 = 0x48;
            break;
        case BW10_4K:
        case BW15_6K:
        case BW20_8K:
        case BW31_2K:
        case BW41_7K: IfFreq2 = 0x44;
            break;
        case BW62_5K:
        case BW125K:
        case BW250K: IfFreq2 = 0x40;
            break;
        default: IfFreq2 = 0x20;
            break;
    }
    writeRegister(0x2f, IfFreq2);
    writeRegister(0x30, 0x00);
}

void SX1276_SetTransmitPower(uint8_t db, enum SX1276_PA_PIN pin) {
    if (pin == PA_RFO_OUTPUT) {
        if (db <= 14) {
            SX1276_SetOCP(100);
            writeRegister(REG_PA_DAC, 0x84);
            writeRegister(REG_PA_CONFIG, 0x70 | db);
        } else {
            return;
        }
    } else if (pin == PA_PABOOST_OUTPUT) {
        if (db < 2) {
            db = 2;
        }
        if (db <= 17) {
            SX1276_SetOCP(100); //TODO check the ocp settings
            writeRegister(REG_PA_DAC, 0x84);
            writeRegister(REG_PA_CONFIG, PA_BOOST | (db - 2));
        } else { //db > 17 set power at 20db
            SX1276_SetOCP(140); //TODO check ocp setting
            writeRegister(REG_PA_DAC, 0x87);
            writeRegister(REG_PA_CONFIG, PA_BOOST | 15);
        }
    }
}

void SX1276_SetOCP(uint8_t maMaxCurrent) {
    uint8_t ocpTrim;
    if (maMaxCurrent > 240) {
        maMaxCurrent = 240;
    }
    if (maMaxCurrent <= 120) {
        ocpTrim = (maMaxCurrent - 45) / 5;
    } else {
        ocpTrim = (maMaxCurrent + 30) / 10;
    }
    ocpTrim &= 0x1f;
    writeRegister(REG_OCP, 0x20 | ocpTrim);
}

void SX1276_SetLNAGain(uint8_t gain, bool boost) {
    if (gain > 6) {
        return;
    }
    uint8_t reg = readRegister(REG_MODEM_CONFIG_3);
    if (gain == 0) {
        //Use auto gain control
        reg |= 0x04;
    } else {
        reg &= ~0x04;
    }
    writeRegister(REG_MODEM_CONFIG_3, reg);
    if (gain == 0) {
        gain = 1; //default value for gain - won't be used since we are using agc
    }
    gain <<= 5;
    if (boost) {
        gain |= 0x03; //turn on HF LNA boost
    }
    writeRegister(REG_LNA, gain);
}

void SX1276_SetPreambleLength(uint8_t len) {
    //TODO code this
}

void SX1276_EnableCRC(bool enable) {
    //TODO test this
    uint8_t reg = readRegister(REG_MODEM_CONFIG_2);
    if (enable) {
        reg |= 0x04;
    } else {
        reg &= ~0x04;
    }
    writeRegister(REG_MODEM_CONFIG_2, reg);
}

void SX1276_Standby(void) {
    setOpMode(MODE_STDBY);
}

void SX1276_Sleep(void) {
    setOpMode(MODE_SLEEP);
}

bool SX1276_SendPacket(uint8_t *data, uint8_t len, bool block) {
    if (len > MAX_PACKET_LENGTH) {
        return false;
    }
    if (SX1276_TXBusy()) {
        return false;
    }
    SX1276_Standby();
    if (!block) {
        writeRegister(REG_DIO_MAPPING_1, DIO0_TXDONE);
    }
    writeFIFO(data, len);
    setOpMode(MODE_TX);
    if (block) {
        while (!(readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK));
        writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK); //Clear the IRQ
    }
    return true;
}

int SX1276_ReceivePacket(uint8_t *data, int maxLen) {
    return true;
}

bool SX1276_TXBusy(void) {
    if ((readRegister(REG_OP_MODE) & MODE_TX) == MODE_TX) {
        return true;
    } else {
        return false;
    }
}

void SX1276_HandleDIO0Int(void) {
    if (readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) {
        writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK); //Clear the IRQ
        if (onTxDone) {
            onTxDone();
        }
    }
    if (readRegister(REG_IRQ_FLAGS) & IRQ_RX_DONE_MASK) {
        writeRegister(REG_IRQ_FLAGS, IRQ_RX_DONE_MASK); //Clear the IRQ
        //TODO handle rx
    }
    if (readRegister(REG_IRQ_FLAGS) & IRQ_CAD_DONE_MASK) {
        writeRegister(REG_IRQ_FLAGS, IRQ_CAD_DONE_MASK); //Clear the IRQ
        //TODO handle CAD
    }
}

void SX1276_SetTXDoneCallback(void (*callback)(void)) {
    onTxDone = callback;
}

void initSPI(void) {
    NSS = 1;
    TRISDbits.TRISD7 = TRISDbits.TRISD6 = TRISDbits.TRISD4 = 0; //RD7, RD6, RD4 outputs
    TRISDbits.TRISD5 = 1; //RD5 is input (MISO)
    //Configure MSSP 2 for SPI at 8 MHz, mode 0,0
    SSP2STATbits.CKE = 1;
    SSP2CON1bits.CKP = 0; //SPI mode 0,0
    SSP2CON1bits.SSPM = 0b0000; //SPI Master - FOSC/4 = 8 MHz
    PIR3bits.SSP2IF = 0;
    SSP2CON1bits.SSPEN = 1; //Enable MSSP
}

uint8_t spiTransfer(uint8_t byte) {
    uint8_t b;
    SSP2BUF = byte; //transmit byte
    while (!PIR3bits.SSP2IF); //Wait until completed
    PIR3bits.SSP2IF = 0; //Clear flag so it is ready for next transfer
    return SSP2BUF; //return received byte
}

uint8_t readRegister(uint8_t regAddress) {
    regAddress &= 0b01111111; //Clear r/w bit for read
    NSS = 0;
    spiTransfer(regAddress);
    uint8_t value = spiTransfer(0);
    NSS = 1;
    return value;
}

void writeRegister(uint8_t regAddress, uint8_t value) {
    regAddress |= 0b10000000; //Set r/w bit for write
    NSS = 0;
    spiTransfer(regAddress);
    spiTransfer(value);
    NSS = 1;
}

void readFIFO(uint8_t *buffer, uint8_t len) {
    NSS = 0;
    spiTransfer(REG_FIFO);
    while (len > 0) {
        *buffer = spiTransfer(0);
        ++buffer;
        --len;
    }
    NSS = 1;
}

void writeFIFO(uint8_t *buffer, uint8_t len) {
    writeRegister(REG_FIFO_ADDR_PTR, TX_FIFO_BASE);
    NSS = 0;
    spiTransfer(REG_FIFO | 0b10000000);
    while (len > 0) {
        spiTransfer(*buffer);
        ++buffer;
        --len;
    }
    NSS = 1;
}

void setOpMode(uint8_t mode) {
    mode &= 0x07;
    writeRegister(REG_OP_MODE, MODE_LORA | mode);
}

void setHeaderMode(enum _headerMode mode) {
    headerMode = mode;
    uint8_t reg = readRegister(REG_MODEM_CONFIG_1);
    if (mode == IMPLICIT_HEADER) {
        reg |= 0x01;
    } else {
        reg &= ~0x01;
    }
    writeRegister(REG_MODEM_CONFIG_1, reg);
}

bool setLDO(void) {
    //calculate symbol duration in milliseconds
    uint32_t symbolDuration = (1000 * (1L << getSpreadingFactor())) / getBandwidthHz();
    uint8_t reg = readRegister(REG_MODEM_CONFIG_3);
    if (symbolDuration > 16) {
        reg |= 0x08;
    } else {
        reg &= ~0x08;
    }
    writeRegister(REG_MODEM_CONFIG_3, reg);
    return true;
}

uint8_t getSpreadingFactor(void) {
    uint8_t reg = readRegister(REG_MODEM_CONFIG_2);
    return reg >> 4;
}

uint32_t getBandwidthHz(void) {
    uint8_t reg = readRegister(REG_MODEM_CONFIG_1);
    reg >>= 4;
    switch (reg) {
        case BW7_8K: return 7810;
        case BW10_4K: return 10420;
        case BW15_6K: return 15620;
        case BW20_8K: return 20830;
        case BW31_2K: return 31250;
        case BW41_7K: return 41670;
        case BW62_5K: return 62500;
        case BW125K: return 125000;
        case BW250K: return 250000;
        case BW500K:
        default: return 500000;
    }
}