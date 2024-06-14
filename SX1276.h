
#ifndef SX1276_H
#define	SX1276_H

#include <stdint.h>
#include <stdbool.h>

enum SX1276_BANDWIDTH {BW7_8K = 0, BW10_4K = 1, BW15_6K = 2, BW20_8K = 3,
BW31_2K = 4, BW41_7K = 5, BW62_5K = 6, BW125K = 7, BW250K = 8, BW500K = 9};

bool SX1276_Init(void);
void SX1276_SetFrequency(uint32_t frequency);
void SX1276_SetChannel(uint8_t channel);
void SX1276_SetSpreadingFactor(uint8_t sf);
void SX1276_SetCodingRate(uint8_t cr);
void SX1276_SetSignalBandwidth(enum SX1276_BANDWIDTH bw);
void SX1276_SetTransmitPower(uint8_t db);
void SX1276_SetOCP(uint8_t maxCurrent);
void SX1276_SetLNAGain(uint8_t gain, bool boost);
void SX1276_Standby(void);
void SX1276_Sleep(void);
bool SX1276_SendPacket(uint8_t *data, uint8_t len, bool block);
int SX1276_ReceivePacket(uint8_t *data, int maxLen);
bool SX1276_TXBusy(void);
void SX1276_HandleDIO0Int(void);
void SX1276_SetTXDoneCallback(void (*callback)(void));

#endif	/* SX1276_H */

