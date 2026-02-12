#include "ow_common.h"
#include "delay.h"
#include "gd32e23x_gpio.h"

/* Dallas/Maxim CRC8 (polynomial 0x31) */
static uint8_t crc8_compute(const uint8_t *data, int len)
{
    uint8_t crc = 0;
    for (int i = 0; i < len; i++) {
        uint8_t inbyte = data[i];
        for (int j = 0; j < 8; j++) {
            uint8_t mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;
            if (mix) crc ^= 0x8C; /* 0x8C is reverse-polynomial of 0x31 */
            inbyte >>= 1;
        }
    }
    return crc;
}

void OW_Common_Init(const OW_Handle *h)
{
    /* caller should enable GPIO clock before calling this */
    gpio_mode_set(h->port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, h->pin);
    gpio_output_options_set(h->port, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, h->pin);
    gpio_bit_set(h->port, h->pin);
}

int OW_Common_ResetPresence(const OW_Handle *h)
{
    int dq;
    gpio_bit_reset(h->port, h->pin);
    Delay_us(480);
    gpio_bit_set(h->port, h->pin);
    Delay_us(70);
    dq = gpio_input_bit_get(h->port, h->pin);
    Delay_us(410);
    return (dq ? 0 : 1);
}

void OW_Common_WriteBit(const OW_Handle *h, uint8_t bit)
{
    if (bit) {
        gpio_bit_reset(h->port, h->pin);
        Delay_us(6);
        gpio_bit_set(h->port, h->pin);
        Delay_us(64);
    } else {
        gpio_bit_reset(h->port, h->pin);
        Delay_us(60);
        gpio_bit_set(h->port, h->pin);
        Delay_us(10);
    }
}

int OW_Common_ReadBit(const OW_Handle *h)
{
    int bit;
    gpio_bit_reset(h->port, h->pin);
    Delay_us(6);
    gpio_bit_set(h->port, h->pin);
    Delay_us(9);
    bit = gpio_input_bit_get(h->port, h->pin) ? 1 : 0;
    Delay_us(55);
    return bit;
}

void OW_Common_WriteByte(const OW_Handle *h, uint8_t data)
{
    for (int i = 0; i < 8; i++) {
        OW_Common_WriteBit(h, data & 0x01);
        data >>= 1;
    }
}

uint8_t OW_Common_ReadByte(const OW_Handle *h)
{
    uint8_t val = 0;
    for (int i = 0; i < 8; i++) {
        if (OW_Common_ReadBit(h)) val |= (1 << i);
    }
    return val;
}

int OW_Common_ConvertTemp(const OW_Handle *h)
{
    if (!OW_Common_ResetPresence(h)) return 0;
    OW_Common_WriteByte(h, 0xCC); /* SKIP ROM */
    OW_Common_WriteByte(h, 0x44); /* CONVERT T */
    return 1;
}

int OW_Common_ReadScratchpad(const OW_Handle *h, uint8_t *out9)
{
    if (!OW_Common_ResetPresence(h)) return 0;
    OW_Common_WriteByte(h, 0xCC); /* SKIP ROM */
    OW_Common_WriteByte(h, 0xBE); /* READ SCRATCHPAD */
    for (int i = 0; i < 9; i++) out9[i] = OW_Common_ReadByte(h);
    uint8_t crc = crc8_compute(out9, 8);
    if (crc != out9[8]) return 0;
    return 1;
}

int OW_Common_ReadTempWaiting(const OW_Handle *h, uint16_t *out)
{
    uint8_t buf[9];
    if (!OW_Common_ReadScratchpad(h, buf)) return 0;
    uint16_t raw = (uint16_t)((buf[1] << 8) | buf[0]);
    *out = raw;
    return 1;
}

float OW_Common_OutputToTemp(int16_t raw)
{
    /* Use T117/MYSENTECH conversion: raw/256 + 25.0 (project's original formula) */
    return ((float)raw) / 256.0f + 25.0f;
}
