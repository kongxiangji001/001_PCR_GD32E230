/*!
    \file    main.c
    \brief   running LED

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

#include "gd32e23x.h"
#include "systick.h"
#include "owmy.h"
#include "delay.h"
#include "T117_MTS4_OW.h"
#include "gd32e23x_usart.h"
#include <stdio.h>

/* USART0 (PA9 TX, PA10 RX) basic init for printf debugging */
static void usart0_init(void)
{
    /* enable clocks */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_USART0);

    /* configure PA9 (TX) and PA10 (RX) as AF1 */
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_9 | GPIO_PIN_10);
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_9 | GPIO_PIN_10);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);

    /* configure USART0 */
    usart_deinit(USART0);
    usart_baudrate_set(USART0, 115200U);
    usart_word_length_set(USART0, USART_WL_8BIT);
    usart_stop_bit_set(USART0, USART_STB_1BIT);
    usart_parity_config(USART0, USART_PM_NONE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    usart_receive_config(USART0, USART_RECEIVE_DISABLE);
    usart_enable(USART0);
}

/* retarget printf to USART0 */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(USART0, (uint8_t)ch);
    while (RESET == usart_flag_get(USART0, USART_FLAG_TBE)) {
    }
    return ch;
}

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    systick_config();
    usart0_init();
    OW_Init();
    
    /* enable the LED GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOB);
    /* configure LED GPIO port */ 
    gpio_mode_set(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_13);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13);
    /* reset LED GPIO pin */
    gpio_bit_reset(GPIOC,GPIO_PIN_13);


    while(1){
        /* detect OneWire device presence and print */
        if (OW_ResetPresence()) {
            //printf("OneWire device detected\r\n");
            GetTemp(); /* read and print temperature via T117 functions */
        } else {
            printf("OneWire device not present\r\n");
        }

        /* blink LED */
        //gpio_bit_set(GPIOC,GPIO_PIN_13);
        //gpio_bit_reset(GPIOC,GPIO_PIN_13);
    }
}
