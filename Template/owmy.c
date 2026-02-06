/* Includes ------------------------------------------------------------------*/
#include "owmy.h"

/* One-wire timing parameters (us) */
#define tSlot               60
#define tRecover            10
#define tInitSlot           3
#define tLow_Write_1        tInitSlot
#define tHigh_Write_1       tSlot
#define tLow_Write_0        53
#define tHigh_Write_0       tRecover
#define tLow_Read           tInitSlot
#define tSample_Read        10
#define tComplement_Read    55
#define tLow_Reset          480
#define tHigh_Reset         480
#define tSample_Presence    40
#define tComplement_Presence (tHigh_Reset - tSample_Presence)
#define tPdlow              ((240 - tSample_Presence) / 10)

/**
  * @brief  OW-GPIO初始化
  */
void OW_Init(void)
{
    /* enable port clock */
    GPIOOW_DQ_GPIO_CLK_ENABLE();

    /* configure PB11 as open-drain output and set high (released) using GD32 v2 API */
    gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_11);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_11);
    gpio_bit_set(GPIOB, GPIO_PIN_11);
}

void OW_Reset(void)
{
    ow_DQ_reset();
    ow_Delay_us(tLow_Reset);
    ow_DQ_set();
}

int OW_Presence(void)
{
    uint8_t dq = 1;
    int count = 0;

    ow_Delay_us(tSample_Presence);
    dq = ow_DQ_get();
    while (dq && (count < tPdlow)) {
        ow_Delay_us(tRecover);
        dq = ow_DQ_get();
        count++;
    }
    ow_Delay_us(tComplement_Presence);
    return (dq ? 0 : 1);
}

int OW_ResetPresence(void)
{
    uint8_t dq = 1;
    int count = 0;

    ow_DQ_reset();
    ow_Delay_us(tLow_Reset);
    ow_DQ_set();
    ow_Delay_us(tSample_Presence);
    dq = ow_DQ_get();
    while (dq && (count < tPdlow)) {
        ow_Delay_us(tRecover);
        dq = ow_DQ_get();
        count++;
    }
    ow_Delay_us(tComplement_Presence);
    return (dq ? 0 : 1);
}

void OW_WriteBit(uint8_t bit)
{
    if (bit) {
        ow_DQ_reset();
        ow_Delay_us(tLow_Write_1);
        ow_DQ_set();
        ow_Delay_us(tHigh_Write_1);
    } else {
        ow_DQ_reset();
        ow_Delay_us(tLow_Write_0);
        ow_DQ_set();
        ow_Delay_us(tHigh_Write_0);
    }
}

int OW_ReadBit(void)
{
    int bit;
    ow_DQ_reset();
    ow_Delay_us(tLow_Read);
    ow_DQ_set();
    ow_Delay_us(tSample_Read);
    bit = ow_DQ_get() ? 1 : 0;
    ow_Delay_us(tComplement_Read);
    return bit;
}

uint8_t OW_Read2Bits(void)
{
    uint8_t i, dq, data = 0;
    for (i = 0; i < 2; i++) {
        dq = OW_ReadBit();
        data |= (dq << i);
    }
    return data;
}

void OW_WriteByte(uint8_t data)
{
    for (int bit = 0; bit < 8; bit++) {
        OW_WriteBit(data & 0x01);
        data >>= 1;
    }
}

uint8_t OW_ReadByte(void)
{
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        if (OW_ReadBit()) {
            byte |= (1 << i);
        }
    }
    return byte;
}
