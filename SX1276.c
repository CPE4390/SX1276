
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

#define RSSI_OFFSET_HF_PORT      -157

//The packet length (in SX1276.h) and fifo bases can be adjusted based on need
#define TX_FIFO_BASE    0x00
#define RX_FIFO_BASE    0x80

#define _XTAL_FREQ  32000000

enum {
    DIO0_TXDONE = 0x40, DIO0_RXDONE = 0x00, DIO0_CADDONE = 0x80
};

//private variables
static uint32_t nominalFrequency;
static bool crcEnabled;
static enum _headerMode headerMode;
static uint8_t expectedLength;
static uint8_t *rxBuffer;
static void (*onTxDone)(void);
static void (*onRxDone)(uint8_t);
static void (*onCadDone)(bool);

//private helper functions
static void initSPI(void);
static uint8_t spiTransfer(uint8_t byte);
static uint8_t readRegister(uint8_t regAddress);
static void writeRegister(uint8_t regAddress, uint8_t value);
static void readFIFO(uint8_t *buffer, uint8_t len);
static void writeFIFO(uint8_t *buffer, uint8_t len);
static void setOpMode(uint8_t mode);
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
    //set 915 MHz as default
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
    uint8_t reg = readRegister(REG_DETECTION_OPTIMIZE);
    reg &= 0b11111000; //clear DetectionOptimize bits
    if (sf == 6) {
        SX1276_SetHeaderMode(IMPLICIT_HEADER);
        reg |= 0x05;
        writeRegister(REG_DETECTION_THRESHOLD, 0x0c);
    } else {
        reg |= 0x03;
        writeRegister(REG_DETECTION_THRESHOLD, 0x0a);
    }
    writeRegister(REG_DETECTION_OPTIMIZE, reg);
    reg = readRegister(REG_MODEM_CONFIG_2);
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
    reg = readRegister(REG_DETECTION_OPTIMIZE);
    reg &= 0b01111111; //clear AutomaticIFOn bit
    if (bw == BW500K) {
        reg |= 0x80;
        writeRegister(0x36, 0x02); //See errata 2.1
        writeRegister(0x3a, 0x64); //See errata 2.2
    } else {
        writeRegister(0x36, 0x03);
    }
    writeRegister(REG_DETECTION_OPTIMIZE, reg);
    setLDO();
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
            SX1276_SetOCP(100);
            writeRegister(REG_PA_DAC, 0x84);
            writeRegister(REG_PA_CONFIG, PA_BOOST | (db - 2));
        } else { //db > 17 set power at 20db
            SX1276_SetOCP(140);
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

void SX1276_SetPreambleLength(uint16_t len) {
    //Actual preamble length will be len + 4.25 symbols;
    if (len < 6) {
        len = 6; //minimum length = 6 + 4 = 10
    }
    writeRegister(REG_PREAMBLE_MSB, (uint8_t) (len >> 8));
    writeRegister(REG_PREAMBLE_LSB, (uint8_t) len);
}

void SX1276_EnableCRC(bool enable) {
    uint8_t reg = readRegister(REG_MODEM_CONFIG_2);
    if (enable) {
        reg |= 0x04;
    } else {
        reg &= ~0x04;
    }
    writeRegister(REG_MODEM_CONFIG_2, reg);
}

void SX1276_SetHeaderMode(enum _headerMode mode) {
    headerMode = mode;
    uint8_t reg = readRegister(REG_MODEM_CONFIG_1);
    if (mode == IMPLICIT_HEADER) {
        reg |= 0x01;
    } else {
        reg &= ~0x01;
    }
    writeRegister(REG_MODEM_CONFIG_1, reg);
}

void SX1276_Standby(void) {
    setOpMode(MODE_STDBY);
}

void SX1276_Sleep(void) {
    setOpMode(MODE_SLEEP);
}

bool SX1276_SendPacket(uint8_t *data, uint8_t len, bool block) {
    if (len > SX1276_MAX_PACKET_LENGTH) {
        return false;
    }
    if (SX1276_TXBusy()) {
        return false;
    }
    SX1276_Standby();
    if (!block) {
        writeRegister(REG_DIO_MAPPING_1, DIO0_TXDONE);
    }
    writeRegister(REG_PAYLOAD_LENGTH, len);
    writeFIFO(data, len);
    setOpMode(MODE_TX);
    if (block) {
        while (!(readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK));
        writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK); //Clear the IRQ
    }
    return true;
}

//In implicit header mode len = packet length
//In explicit header mode len = data buffer length = maximum packet that can be sent to buffer
//Return value is size of received packet or zero if packet is invalid

uint8_t SX1276_ReceivePacket(uint8_t *data, uint8_t len, bool block) {
    uint8_t rxLen = 0;
    SX1276_Standby();
    writeRegister(REG_FIFO_ADDR_PTR, RX_FIFO_BASE);
    if (headerMode == IMPLICIT_HEADER) {
        writeRegister(REG_PAYLOAD_LENGTH, len);
    }
    if (block) {
        setOpMode(MODE_RX_CONTINUOUS);
        uint8_t irqFlags;
        do {
            irqFlags = readRegister(REG_IRQ_FLAGS);
        } while (!(irqFlags & IRQ_RX_DONE_MASK));
        writeRegister(REG_IRQ_FLAGS, IRQ_RX_DONE_MASK); //Clear the IRQ
        if (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) {
            writeRegister(REG_IRQ_FLAGS, IRQ_PAYLOAD_CRC_ERROR_MASK); //Clear the IRQ
            return 0; //packet is invalid
        } else {
            if (headerMode == IMPLICIT_HEADER) {
                rxLen = len;
            } else {
                rxLen = readRegister(REG_RX_NB_BYTES);
                if (rxLen > len) {
                    rxLen = len;
                }
            }
            writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));
            readFIFO(data, rxLen);
        }
    } else {
        //non-blocking mode always returns zero. The callback will get the result
        rxBuffer = data;
        expectedLength = len;
        writeRegister(REG_DIO_MAPPING_1, DIO0_RXDONE);
        setOpMode(MODE_RX_CONTINUOUS);
    }
    return rxLen;
}

bool SX1276_ChannelActivityDetect(bool block) {
    SX1276_Standby();
    if (!block) {
        writeRegister(REG_DIO_MAPPING_1, DIO0_CADDONE);
    }
    setOpMode(MODE_CAD);
    if (block) {
        while (!(readRegister(REG_IRQ_FLAGS) & IRQ_CAD_DONE_MASK));
        writeRegister(REG_IRQ_FLAGS, IRQ_CAD_DONE_MASK); //Clear the IRQ
        if (readRegister(REG_IRQ_FLAGS) & IRQ_CAD_DETECTED_MASK) {
            writeRegister(REG_IRQ_FLAGS, IRQ_CAD_DETECTED_MASK); //Clear the IRQ
            return true;
        } else {
            return false;
        }
    } else {
        //non-blocking mode always returns false. The callback will get the result
        return false;
    }
}

bool SX1276_TXBusy(void) {
    if ((readRegister(REG_OP_MODE) & MODE_TX) == MODE_TX) {
        return true;
    } else {
        return false;
    }
}

int SX1276_RSSI(void) {
    int rssi = readRegister(REG_RSSI_VALUE);
    return RSSI_OFFSET_HF_PORT + rssi;
}

int SX1276_PacketRSSI(void) {
    int rssi = readRegister(REG_PKT_RSSI_VALUE);
    return RSSI_OFFSET_HF_PORT + rssi;
}

float SX1276_PacketSNR(void) {
    int snr = readRegister(REG_PKT_SNR_VALUE);
    return (float) (snr * 0.25);
}

void SX1276_HandleDIO0Int(void) {
    //TODO should we only read REG_IRQ_FLAGS once?
    if (readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) {
        writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK); //Clear the IRQ
        if (onTxDone) {
            onTxDone();
        }
    }
    if (readRegister(REG_IRQ_FLAGS) & IRQ_RX_DONE_MASK) {
        writeRegister(REG_IRQ_FLAGS, IRQ_RX_DONE_MASK); //Clear the IRQ
        uint8_t rxLen = 0;
        if (readRegister(REG_IRQ_FLAGS) & IRQ_PAYLOAD_CRC_ERROR_MASK) {
            writeRegister(REG_IRQ_FLAGS, IRQ_PAYLOAD_CRC_ERROR_MASK);
        } else {
            //valid packet
            if (headerMode == EXPLICIT_HEADER) {
                rxLen = readRegister(REG_RX_NB_BYTES);
                if (rxLen > expectedLength) {
                    rxLen = expectedLength;
                }
            } else {
                rxLen = expectedLength;
            }
            writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));
            readFIFO(rxBuffer, rxLen);
        }
        if (onRxDone) {
            onRxDone(rxLen);
        }
    }
    if (readRegister(REG_IRQ_FLAGS) & IRQ_CAD_DONE_MASK) {
        writeRegister(REG_IRQ_FLAGS, IRQ_CAD_DONE_MASK); //Clear the IRQ
        bool result = false;
        if (readRegister(REG_IRQ_FLAGS) & IRQ_CAD_DETECTED_MASK) {
            writeRegister(REG_IRQ_FLAGS, IRQ_CAD_DETECTED_MASK); //Clear the IRQ
            result = true;
        }
        if (onCadDone) {
            onCadDone(result);
        }
    }
}

void SX1276_SetTXDoneCallback(void (*callback)(void)) {
    onTxDone = callback;
}

void SX1276_SetRXDoneCallback(void (*callback)(uint8_t)) {
    onRxDone = callback;
}

void SX1276_SetCadDoneCallback(void (*callback)(bool)) {
    onCadDone = callback;
}

void SX1276_OptimizeRxPerErrata(void) {
    /* This function implements the settings to prevent spurious receptions
     as described in 2.3 of the errata document.
     It should be called after the frequency/channel or bandwidth 
     are set (or changed.)  It should only be used by a receiver node*/
    SX1276_Standby();
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
    uint8_t bw = readRegister(REG_MODEM_CONFIG_1);
    bw >>= 4;
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