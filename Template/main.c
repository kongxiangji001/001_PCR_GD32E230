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
#include "ow2.h"

/* retarget printf to USART1 */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(USART1, (uint8_t)ch);
    while (RESET == usart_flag_get(USART1, USART_FLAG_TBE)) {
    }
    return ch;
}

/* Heater struct and helper (top-level, not nested) */
typedef struct {
    PIDController pid;
    float setpoint;
    uint8_t duty;
} Heater_t;

static uint8_t UpdateHeater(uint8_t channel, Heater_t *h, float (*getTempFunc)(void), float *outTemp)
{
    float temp = getTempFunc();
    float out = pid_compute(&h->pid, h->setpoint, temp);
    if (out < 0.0f) out = 0.0f;
    if (out > 100.0f) out = 100.0f;
    h->duty = (uint8_t)out;
    SetHeaterDuty(channel, h->duty);
    if (outTemp) *outTemp = temp;
    return h->duty;
}

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
/**
 * 主函数
 * 初始化系统定时器、IO接口、定时器2，并配置两个加热器的PID控制器
 * 在主循环中每100ms更新一次加热器状态并打印温度和占空比信息
 */
int main(void)
{
    systick_config();                    /* 初始化系统定时器 */
    IO_Config();                         /* initialize USART1, OW and LED IO */
    /* configure TIMER2 with desired PWM frequency (Hz) */
    Config_Timer2_Init(2000U);           /* 2kHz PWM */

    /* initialize two heaters with same default PID params */
    const float default_setpoint = 95.0f;
    const float Kp = 10.0f;
    const float Ki = 0.5f;
    const float Kd = 0.1f;
    Heater_t heater1, heater2;
    heater1.setpoint = default_setpoint;
    heater1.duty = 0;
    heater2.setpoint = default_setpoint;
    heater2.duty = 0;
    pid_init(&heater1.pid, Kp, Ki, Kd, 0.0f, 100.0f, 0.1f);
    pid_init(&heater2.pid, Kp, Ki, Kd, 0.0f, 100.0f, 0.1f);

    /* helper: update heater control is implemented at top-level */
    while (1) {
        float temp1 = 0.0f, temp2 = 0.0f;
        uint8_t d1 = 0, d2 = 0;
        if (flag_100ms) {
            flag_100ms = 0;
            heater1.setpoint = 42.0f; /* 更新加热片1的目标温度 */
            heater2.setpoint = 26.0f; /* 更新加热片2的目标温度 */
            d1 = UpdateHeater(2, &heater1, GetTemp, &temp1); /* TIM2_CH2 -> heater1, DQ1 PB11 */
            d2 = UpdateHeater(3, &heater2, GetTemp2, &temp2); /* TIM2_CH3 -> heater2, DQ2 PA12 */
            printf("%.2f, %d, %.2f, %d\r\n", temp1, d1, temp2, d2);
        }
    }
}
