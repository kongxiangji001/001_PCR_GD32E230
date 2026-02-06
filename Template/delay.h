/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DELAY_H
#define __DELAY_H

/* Includes ------------------------------------------------------------------*/
#include "gd32e23x.h"
#include <stdint.h>

/* delay in microseconds (blocking) */
void Delay_us(uint32_t n);
/* delay in milliseconds (blocking) - uses systick delay_1ms */
void Delay_ms(uint32_t n);

#endif /* __DELAY_H */
