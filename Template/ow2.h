/* ow2.h - OneWire on PA12 (second DQ line) */
#ifndef OW2_H
#define OW2_H

#include "gd32e23x.h"

void OW2_Init(void);
int OW2_ResetPresence(void);
void OW2_WriteBit(uint8_t bit);
int OW2_ReadBit(void);
void OW2_WriteByte(uint8_t data);
uint8_t OW2_ReadByte(void);
int OW2_ConvertTemp(uint8_t x);
int OW2_ReadTempWaiting(uint16_t* iTemp, uint8_t x);
float GetTemp2(void);

#endif /* OW2_H */
