/* ow_common.h - shared OneWire driver for DS18B20/T117-like sensors */
#ifndef OW_COMMON_H
#define OW_COMMON_H

#include "gd32e23x.h"
#include <stdint.h>

typedef struct {
    uint32_t port;
    uint16_t pin;
} OW_Handle;

/* basics */
void OW_Common_Init(const OW_Handle *h);
int  OW_Common_ResetPresence(const OW_Handle *h);
void OW_Common_WriteBit(const OW_Handle *h, uint8_t bit);
int  OW_Common_ReadBit(const OW_Handle *h);
void OW_Common_WriteByte(const OW_Handle *h, uint8_t data);
uint8_t OW_Common_ReadByte(const OW_Handle *h);

/* temperature helpers */
int  OW_Common_ConvertTemp(const OW_Handle *h);
int  OW_Common_ReadTempWaiting(const OW_Handle *h, uint16_t *out);
float OW_Common_OutputToTemp(int16_t raw);

/* read full scratchpad (9 bytes), returns 1 on success (CRC ok) */
int OW_Common_ReadScratchpad(const OW_Handle *h, uint8_t *out9);

#endif /* OW_COMMON_H */
