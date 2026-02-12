/* Includes ------------------------------------------------------------------*/
#include "owmy.h"
#include "ow_common.h"

static const OW_Handle ow1 = { GPIOB, GPIO_PIN_11 };

void OW_Init(void)
{
    GPIOOW_DQ_GPIO_CLK_ENABLE();
    OW_Common_Init(&ow1);
}

int OW_ResetPresence(void)
{
    return OW_Common_ResetPresence(&ow1);
}

void OW_WriteBit(uint8_t bit)
{
    OW_Common_WriteBit(&ow1, bit);
}

int OW_ReadBit(void)
{
    return OW_Common_ReadBit(&ow1);
}

uint8_t OW_Read2Bits(void)
{
    uint8_t v = 0;
    v |= (OW_Common_ReadBit(&ow1) & 0x01);
    v |= ((OW_Common_ReadBit(&ow1) & 0x01) << 1);
    return v;
}

void OW_WriteByte(uint8_t data)
{
    OW_Common_WriteByte(&ow1, data);
}

uint8_t OW_ReadByte(void)
{
    return OW_Common_ReadByte(&ow1);
}
