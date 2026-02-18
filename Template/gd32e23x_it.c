/*!
    \file    gd32e23x_it.c
    \brief   interrupt service routines

    \version 2025-08-08, V2.4.0, firmware for GD32E23x
*/

/*
    Copyright (c) 2025, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "gd32e23x_it.h"
#include "systick.h"
#include "config.h"
#include "gd32e23x_gpio.h"

/* external LED/TIM5 control flags (defined in main.c) */
extern volatile uint8_t red_blink_enable;
extern volatile uint8_t green_blink_enable;
extern volatile uint8_t red_steady_on;
extern volatile uint8_t green_steady_on;
extern volatile uint8_t blink_state;

#define SRAM_PARITY_CHECK_ERROR_HANDLE(s)    do{}while(1)

/*!
    \brief      this function handles NMI exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void NMI_Handler(void)
{
    if(SET == syscfg_flag_get(SYSCFG_SRAM_PCEF)) {
        SRAM_PARITY_CHECK_ERROR_HANDLE("SRAM parity check error error\r\n");
    } else {
        /* if NMI exception occurs, go to infinite loop */
        /* HXTAL clock monitor NMI error or NMI pin error */
        while(1) {
        }
    }
}

/*!
    \brief      this function handles HardFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void HardFault_Handler(void)
{
    /* if Hard Fault exception occurs, go to infinite loop */
    while(1){
    }
}

/*!
    \brief      this function handles SVC exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SVC_Handler(void)
{
    /* if SVC exception occurs, go to infinite loop */
    while(1) {
    }
}

/*!
    \brief      this function handles PendSV exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void PendSV_Handler(void)
{
    /* if PendSV exception occurs, go to infinite loop */
    while(1) {
    }
}

/*!
    \brief      this function handles SysTick exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SysTick_Handler(void)
{
    delay_decrement();
}

/* TIMER2 update IRQ: set 100ms and 200ms flags (timer configured to update at 1kHz and ARR counted to 1000) */
void TIMER2_IRQHandler(void)
{
    static uint16_t ms_count_100ms = 0;
    if (RESET != timer_flag_get(TIMER2, TIMER_FLAG_UP)) {
        timer_flag_clear(TIMER2, TIMER_FLAG_UP);
        ms_count_100ms++;
        if (ms_count_100ms >= 200) { /* 200 updates @1ms -> 100ms */
            ms_count_100ms = 0;
            flag_100ms = 1;
        }
    }
}

/* TIMER5 IRQ: used for LED blink timing (toggle blink_state and update outputs in ISR-safe way) */
void TIMER5_IRQHandler(void)
{
    if (RESET != timer_flag_get(TIMER5, TIMER_FLAG_UP)) {
        timer_flag_clear(TIMER5, TIMER_FLAG_UP);
        /* toggle blink phase */
        blink_state = !blink_state;

        /* 红灯 PA8：常亮优先，其次闪烁 */
        if (red_steady_on) {
            gpio_bit_set(GPIOA, GPIO_PIN_8);
        } else if (red_blink_enable) {
            if (blink_state) gpio_bit_set(GPIOA, GPIO_PIN_8);
            else gpio_bit_reset(GPIOA, GPIO_PIN_8);
        } else {
            gpio_bit_reset(GPIOA, GPIO_PIN_8);
        }

        /* 绿灯 PA9：常亮优先，其次闪烁 */
        if (green_steady_on) {
            gpio_bit_set(GPIOA, GPIO_PIN_9);
        } else if (green_blink_enable) {
            if (blink_state) gpio_bit_set(GPIOA, GPIO_PIN_9);
            else gpio_bit_reset(GPIOA, GPIO_PIN_9);
        } else {
            gpio_bit_reset(GPIOA, GPIO_PIN_9);
        }
        
        /* 恒温区1档LED闪烁控制 */
        extern volatile uint8_t zone1_led_state;
        extern volatile uint8_t zone1_blink_count;
        extern volatile uint8_t zone1_blink_state;
        
        if (zone1_led_state == 3) {  /* LED_BLINK_RED_GREEN */
            /* 红绿交替闪烁 */
            zone1_blink_state = !zone1_blink_state;
            /* 每次切换红绿状态，计数一次 */
            if (zone1_blink_state) {
                zone1_blink_count++;
                /* 闪烁3次（6次切换）后，切换到绿色常亮 */
                if (zone1_blink_count >= 6) {
                    zone1_led_state = 2;  /* LED_GREEN */
                    zone1_blink_count = 0;
                    zone1_blink_state = 0;
                }
            }
        }
        
        /* 恒温区2档LED闪烁控制 */
        extern volatile uint8_t zone2_led_state;
        extern volatile uint8_t zone2_blink_count;
        extern volatile uint8_t zone2_blink_state;
        
        if (zone2_led_state == 3) {  /* LED_BLINK_RED_GREEN */
            /* 红绿交替闪烁 */
            zone2_blink_state = !zone2_blink_state;
            /* 每次切换红绿状态，计数一次 */
            if (zone2_blink_state) {
                zone2_blink_count++;
                /* 闪烁3次（6次切换）后，切换到绿色常亮 */
                if (zone2_blink_count >= 6) {
                    zone2_led_state = 2;  /* LED_GREEN */
                    zone2_blink_count = 0;
                    zone2_blink_state = 0;
                }
            }
        }
        
        /* 裂解区1档LED闪烁控制 */
        extern volatile uint8_t lysis1_led_state;
        extern volatile uint8_t lysis1_blink_count;
        extern volatile uint8_t lysis1_blink_state;
        
        if (lysis1_led_state == 3) {  /* LED_BLINK_RED_GREEN */
            /* 红绿交替闪烁 */
            lysis1_blink_state = !lysis1_blink_state;
            /* 每次切换红绿状态，计数一次 */
            if (lysis1_blink_state) {
                lysis1_blink_count++;
                /* 闪烁3次（6次切换）后，切换到绿色常亮 */
                if (lysis1_blink_count >= 6) {
                    lysis1_led_state = 2;  /* LED_GREEN */
                    lysis1_blink_count = 0;
                    lysis1_blink_state = 0;
                }
            }
        }
        
        /* 裂解区2档LED闪烁控制 */
        extern volatile uint8_t lysis2_led_state;
        extern volatile uint8_t lysis2_blink_count;
        extern volatile uint8_t lysis2_blink_state;
        
        if (lysis2_led_state == 3) {  /* LED_BLINK_RED_GREEN */
            /* 红绿交替闪烁 */
            lysis2_blink_state = !lysis2_blink_state;
            /* 每次切换红绿状态，计数一次 */
            if (lysis2_blink_state) {
                lysis2_blink_count++;
                /* 闪烁3次（6次切换）后，切换到绿色常亮 */
                if (lysis2_blink_count >= 6) {
                    lysis2_led_state = 2;  /* LED_GREEN */
                    lysis2_blink_count = 0;
                    lysis2_blink_state = 0;
                }
            }
        }
        
        /* 裂解区3档LED闪烁控制 */
        extern volatile uint8_t lysis3_led_state;
        extern volatile uint8_t lysis3_blink_count;
        extern volatile uint8_t lysis3_blink_state;
        
        if (lysis3_led_state == 3) {  /* LED_BLINK_RED_GREEN */
            /* 红绿交替闪烁 */
            lysis3_blink_state = !lysis3_blink_state;
            /* 每次切换红绿状态，计数一次 */
            if (lysis3_blink_state) {
                lysis3_blink_count++;
                /* 闪烁3次（6次切换）后，切换到绿色常亮 */
                if (lysis3_blink_count >= 6) {
                    lysis3_led_state = 2;  /* LED_GREEN */
                    lysis3_blink_count = 0;
                    lysis3_blink_state = 0;
                }
            }
        }
    }
}
