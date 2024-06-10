
#ifndef SX1276_H
#define	SX1276_H

#include <stdint.h>

void SX1276_Init(uint32_t frequency);
char SX1276_SendPacket(uint8_t *data, int len);
int SX1276_ReceivePacket(uint8_t *data, int maxLen);

void SX1276_HandleRxInt(void);
void SX1276_HandleTxInt(void);

#endif	/* SX1276_H */

