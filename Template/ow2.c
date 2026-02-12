/* Strict OneWire implementation for PA12 (copy of owmy style) */
#include "ow2.h"
#include "delay.h"
#include "gd32e23x_gpio.h"
#include "T117_MTS4_OW.h"

/* Match T117 usage: assume single device per bus */
#define SingleIC

/* timing constants (us) - match owmy.c */
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

/* PA12 as DQ2 */
#define OW2_GPIO_PORT GPIOA
#define OW2_GPIO_PIN  GPIO_PIN_12
#define OW2_GPIO_CLK_ENABLE() rcu_periph_clock_enable(RCU_GPIOA)

static inline void ow2_DQ_set(void) { gpio_bit_set(OW2_GPIO_PORT, OW2_GPIO_PIN); }
static inline void ow2_DQ_reset(void) { gpio_bit_reset(OW2_GPIO_PORT, OW2_GPIO_PIN); }
static inline int ow2_DQ_get(void) { return gpio_input_bit_get(OW2_GPIO_PORT, OW2_GPIO_PIN); }

void OW2_Init(void)
{
    OW2_GPIO_CLK_ENABLE();
    gpio_mode_set(OW2_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, OW2_GPIO_PIN);
    gpio_output_options_set(OW2_GPIO_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, OW2_GPIO_PIN);
    ow2_DQ_set();
}

int OW2_ResetPresence(void)
{
    int dq = 1; int count = 0;
    ow2_DQ_reset();
    Delay_us(tLow_Reset);
    ow2_DQ_set();
    Delay_us(tSample_Presence);
    dq = ow2_DQ_get();
    while (dq && (count < tPdlow)) {
        Delay_us(tRecover);
        dq = ow2_DQ_get();
        count++;
    }
    Delay_us(tComplement_Presence);
    return (dq ? 0 : 1);
}

void OW2_WriteBit(uint8_t bit)
{
    if (bit) {
        ow2_DQ_reset();
        Delay_us(tLow_Write_1);
        ow2_DQ_set();
        Delay_us(tHigh_Write_1);
    } else {
        ow2_DQ_reset();
        Delay_us(tLow_Write_0);
        ow2_DQ_set();
        Delay_us(tHigh_Write_0);
    }
}

int OW2_ReadBit(void)
{
    int bit;
    ow2_DQ_reset();
    Delay_us(tLow_Read);
    ow2_DQ_set();
    Delay_us(tSample_Read);
    bit = ow2_DQ_get() ? 1 : 0;
    Delay_us(tComplement_Read);
    return bit;
}

void OW2_WriteByte(uint8_t data)
{
    for (int i = 0; i < 8; i++) {
        OW2_WriteBit(data & 0x01);
        data >>= 1;
    }
}

uint8_t OW2_ReadByte(void)
{
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        if (OW2_ReadBit()) byte |= (1 << i);
    }
    return byte;
}

/* use commands from T117_MTS4_OW.h: SKIP_ROM, CONVERT_T, READ_TEMP, READ_SCR */

int OW2_ConvertTemp(uint8_t x)
{
    (void)x;
    if (OW2_ResetPresence() == 0) return 0;
    OW2_WriteByte(SKIP_ROM);
    OW2_WriteByte(CONVERT_T);
    return 1;
}

int OW2_ReadTemp(uint8_t* scr, uint8_t x)
{
    uint8_t i, j;
    if (OW2_ResetPresence() == 0) return 0;
#ifdef SingleIC
    OW2_WriteByte(SKIP_ROM);
#endif
#ifndef SingleIC
    OW2_WriteByte(MATCH_ROM);
    for (j = 0; j < 8; j++) {
        OW2_WriteByte(ID_Buff[x][j]);
    }
#endif
    OW2_WriteByte(READ_TEMP);
    for (i = 0; i < 3; i++) {
        *scr++ = OW2_ReadByte();
    }
    return 1;
}

int OW2_ReadScratchpad(uint8_t* scr, uint8_t x)
{
    uint8_t i, j;
    if (OW2_ResetPresence() == 0) return 0;
#ifdef SingleIC
    OW2_WriteByte(SKIP_ROM);
#endif
#ifndef SingleIC
    OW2_WriteByte(MATCH_ROM);
    for (j = 0; j < 8; j++) {
        OW2_WriteByte(ID_Buff[x][j]);
    }
#endif
    OW2_WriteByte(READ_SCR);
    for (i = 0; i < sizeof(SCRATCHPAD_READ); i++) {
        *scr++ = OW2_ReadByte();
    }
    return 1;
}

int OW2_ReadConfig(uint8_t* cfg, uint8_t x)
{
    uint8_t scrb[sizeof(SCRATCHPAD_READ)];
    SCRATCHPAD_READ* scr = (SCRATCHPAD_READ*)scrb;
    if (OW2_ReadScratchpad(scrb, x) == 0) return 0;
#ifdef SingleIC
    if (scrb[sizeof(scrb)-1] != MY_CRC8(scrb, sizeof(scrb)-1)) return 0;
#endif
#ifndef SingleIC
    uint8_t ID1[15];
    for (uint8_t j = 0; j < 7; j++) ID1[j] = ID_Buff[x][j];
    for (uint8_t j = 0; j < (sizeof(scrb)-1); j++) ID1[j+7] = scrb[j];
    if (scrb[sizeof(scrb)-1] != MY_CRC8(ID1, sizeof(scrb)+6)) return 0;
#endif
    *cfg = scr->Temp_Cfg;
    return 1;
}

int OW2_ReadTempWaiting(uint16_t* iTemp, uint8_t x)
{
    uint8_t scrb[sizeof(TEMP_READ)];
    TEMP_READ* scr = (TEMP_READ*)scrb;
    if (OW2_ReadTemp(scrb, x) == 0) return 0;
#ifdef SingleIC
    if (scrb[sizeof(scrb)-1] != MY_CRC8(scrb, sizeof(scrb)-1)) return 0;
#endif
    *iTemp = (uint16_t)scr->T_msb << 8 | scr->T_lsb;
    return 1;
}

/* conversion used in T117_MTS4_OW.c */
static float OW2_OutputtoTemp(int16_t out)
{
    return ((float)out / 256.0f + 25.0f);
}

float GetTemp2(void)
{
    uint16_t iTemp = 0;
    if (!OW2_ConvertTemp(0)) return 0.0f;
    /* wait for conversion */
    Delay_ms(tCon_A32);
    if (!OW2_ReadTempWaiting(&iTemp, 0)) return 0.0f;
#ifdef ENABLE_OW2_DEBUG
    printf("OW2 raw=0x%04X\r\n", (unsigned int)iTemp);
#endif
    return OW2_OutputtoTemp((int16_t)iTemp);
}
