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
#include "config.h"
#include <stdio.h>
#include "pid.h"

/* retarget printf to USART1 */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(USART1, (uint8_t)ch);
    while (RESET == usart_flag_get(USART1, USART_FLAG_TBE)) {
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
    IO_Config(); /* initialize USART1, OW and LED IO */
    /* configure TIMER2 with desired PWM frequency (Hz) */
    Config_Timer2_Init(2000U); /* 2kHz PWM */

    const float setpoint = 95.0f;
    /* PID parameters (tune as needed) */
    const float Kp = 10.0f;
    const float Ki = 0.5f;
    const float Kd = 0.1f;
    uint8_t duty = 0;
    PIDController pid;
    pid_init(&pid, Kp, Ki, Kd, 0.0f, 100.0f, 0.1f); /* dt = 0.1s (100ms) */


    while (1) {
        if (flag_100ms) {
            flag_100ms = 0;
            /* read temperature (returns float) */
            float temp = GetTemp();
            /* PID compute */
            float out = pid_compute(&pid, setpoint, temp);
            if (out < 0.0f) out = 0.0f;
            if (out > 100.0f) out = 100.0f;
            duty = (uint8_t)out;
            SetHeaterDuty(2, duty); /* TIM2_CH2 -> heater */

            /* print status */
            printf("%.2f, %d\r\n", temp, duty);
        }
    }
}
