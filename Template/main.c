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
#include "gd32e23x_gpio.h"
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

/* Power LED / TIM5 blink control globals and prototypes */
volatile uint8_t red_blink_enable = 0;
volatile uint8_t green_blink_enable = 0;
volatile uint8_t red_steady_on = 0;
volatile uint8_t blink_state = 0;

static void PowerLEDs_Init(void);
static void Timer5_Init_ms(uint32_t ms);

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
    PowerLEDs_Init();                    /* 初始化 PA8/PA9/PA10 */
    Timer5_Init_ms(250);                 /* TIM5 每 250ms 切换一次 blink_state */
    ADC_Config();                        /* configure ADC for battery voltage on PA1 */
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
        float temp1 = 0.0f, temp2 = 0.0f, battery_voltage = 0.0f;
        uint8_t d1 = 0, d2 = 0;
        //if (flag_100ms) {
            flag_100ms = 0;
            heater1.setpoint = 42.0f;                         /* 更新加热片1的目标温度 */
            heater2.setpoint = 95.0f;                         /* 更新加热片2的目标温度 */
            d1 = UpdateHeater(2, &heater1, GetTemp, &temp1);  /* TIM2_CH2 -> heater1, DQ1 PB11 */
            d2 = UpdateHeater(3, &heater2, GetTemp2, &temp2); /* TIM2_CH3 -> heater2, DQ2 PA12 */
            battery_voltage = Read_Battery_Voltage();         /* 读取电池电压 */

            /* 读取 PA10 (充电状态)：0=充电中，1=充满/无充电 */
            uint8_t pa10 = gpio_input_bit_get(GPIOA, GPIO_PIN_10);

            /* 更新 LED 行为（按需求）:
               红灯常亮 快没电了：PA10高且PA1电压<2.3V
               红灯闪烁 充电中：PA10低且PA1电压<3.1V
               绿灯闪烁 充电完了：
                 (PA10高 且 电压>3.1V) 或 (PA10低 且 电压>3.12V)
            */
            red_blink_enable = 0;
            red_steady_on = 0;
            green_blink_enable = 0;

            if (pa10) { /* PA10 高 */
                if (battery_voltage < 2.3f) {
                    red_steady_on = 1;
                } else if (battery_voltage > 3.1f) {
                    green_blink_enable = 1;
                }
            } else { /* PA10 低 -> 充电中 */
                if (battery_voltage < 3.1f) {
                    red_blink_enable = 1;
                }
                if (battery_voltage > 3.12f) {
                    green_blink_enable = 1;
                }
            }

            printf("%.2f, %d, %.2f, %d, %.2f\r\n", temp1, d1, temp2, d2, battery_voltage);
        //}
    }
}

/* 初始化 PA8/PA9 输出 (推挽), PA10 输入上拉 */
static void PowerLEDs_Init(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);

    /* PA8, PA9 输出推挽 (使用新的 gpio API) */
    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_8 | GPIO_PIN_9);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8 | GPIO_PIN_9);

    /* PA10 输入，上拉 */
    gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_10);

    /* 初始关闭 LED（高电平点亮） */
    gpio_bit_reset(GPIOA, GPIO_PIN_8 | GPIO_PIN_9);
}

/* 使用 TIMER5 产生 ms 级中断 (ms >= 1) */
static void Timer5_Init_ms(uint32_t ms)
{
    timer_parameter_struct timer_initpara;
    uint32_t pclk;

    rcu_periph_clock_enable(RCU_TIMER5);

    timer_deinit(TIMER5);
    timer_struct_para_init(&timer_initpara);

    pclk = rcu_clock_freq_get(CK_APB1);               /* APB1 时钟频率 */
    /* prescaler 使计数器时钟为 1kHz */
    timer_initpara.prescaler = (pclk / 1000U) - 1U;
    timer_initpara.alignedmode = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection = TIMER_COUNTER_UP;
    timer_initpara.period = ms - 1U;
    timer_initpara.clockdivision = TIMER_CKDIV_DIV1;

    timer_init(TIMER5, &timer_initpara);

    timer_interrupt_enable(TIMER5, TIMER_INT_UP);
    nvic_irq_enable(TIMER5_IRQn, 2U);

    timer_enable(TIMER5);
}
