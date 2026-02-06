#include "delay.h"
#include "systick.h"

/*
 * Delay_us: perform a blocking microsecond delay.
 * Implementation uses the SysTick timer temporarily; it saves and restores
 * SysTick registers so systick-based millisecond interrupts continue to work.
 */
void Delay_us(uint32_t us)
{
    if (us == 0) return;

    uint32_t ticks_per_us = SystemCoreClock / 1000000U;
    uint32_t max_reload = SysTick_LOAD_RELOAD_Msk;

    uint32_t ctrl = SysTick->CTRL;
    uint32_t load = SysTick->LOAD;
    uint32_t val  = SysTick->VAL;

    /* Disable SysTick while we reconfigure for microsecond delay */
    SysTick->CTRL = 0;

    while (us) {
        uint32_t this_us = us;
        /* ensure reload doesn't exceed 24-bit */
        uint32_t max_us = max_reload / (ticks_per_us ? ticks_per_us : 1);
        if (this_us > max_us) this_us = max_us;

        uint32_t ticks = this_us * ticks_per_us;
        if (ticks == 0) ticks = 1;

        SysTick->LOAD = (ticks - 1) & SysTick_LOAD_RELOAD_Msk;
        SysTick->VAL = 0;
        /* enable without interrupt */
        SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk;
        /* wait until count flag */
        while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)) {
        }

        us -= this_us;
    }

    /* restore SysTick registers */
    SysTick->CTRL = ctrl;
    SysTick->LOAD = load;
    SysTick->VAL  = val;
}

void Delay_ms(uint32_t n)
{
    while (n--) {
        delay_1ms(1);
    }
}
