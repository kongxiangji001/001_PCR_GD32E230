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

/* 恒温区1档状态机 */
typedef enum {
    ZONE1_IDLE = 0,          /* 空闲状态 */
    ZONE1_HEATING,           /* 加热中 */
    ZONE1_HOLDING,           /* 恒温计时中 */
    ZONE1_COMPLETE           /* 完成 */
} Zone1State_t;

/* 恒温区1档LED状态 */
typedef enum {
    LED_OFF = 0,
    LED_RED,
    LED_GREEN,
    LED_BLINK_RED_GREEN
} Zone1LEDState_t;

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
volatile uint8_t green_steady_on = 0;
volatile uint8_t blink_state = 0;

/* 恒温区1档控制变量 */
volatile Zone1State_t zone1_state = ZONE1_IDLE;
volatile uint8_t zone1_led_state = 0;  /* 0=LED_OFF, 1=LED_RED, 2=LED_GREEN, 3=LED_BLINK_RED_GREEN */
volatile uint32_t zone1_timer = 0;  /* 计时器，单位为100ms */
volatile uint8_t zone1_blink_count = 0;  /* 红绿交替闪烁计数 */
volatile uint8_t zone1_blink_state = 0;  /* 0=红灯, 1=绿灯 */

/* 恒温区2档控制变量 */
volatile Zone1State_t zone2_state = ZONE1_IDLE;
volatile uint8_t zone2_led_state = 0;  /* 0=LED_OFF, 1=LED_RED, 2=LED_GREEN, 3=LED_BLINK_RED_GREEN */
volatile uint32_t zone2_timer = 0;  /* 计时器，单位为100ms */
volatile uint8_t zone2_blink_count = 0;  /* 红绿交替闪烁计数 */
volatile uint8_t zone2_blink_state = 0;  /* 0=红灯, 1=绿灯 */

static void PowerLEDs_Init(void);
static void Timer5_Init_ms(uint32_t ms);
static void UpdatePowerLEDs(float battery_voltage, uint8_t pa10);
static void UpdateZoneLEDs(void);
static void UpdateZone1State(float temp1, Heater_t *heater1);
static void UpdateZone2State(float temp1, Heater_t *heater1);

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
    Timer5_Init_ms(1000);                 /* TIM5 每 250ms 切换一次 blink_state */
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
        if (flag_100ms) {
            flag_100ms = 0;
            /* 更新恒温区1档状态 - 已移动到UpdateHeater之后 */
            /* 更新恒温区2档状态 - 已移动到UpdateHeater之后 */
            /* 根据恒温区1档状态设置加热片1的目标温度 */
            if (zone1_state == 1 || zone1_state == 2) {  /* ZONE1_HEATING || ZONE1_HOLDING */
                heater1.setpoint = 45.0f;  /* 更新加热片1的目标温度为45摄氏度 */
            } else if (zone2_state == 1 || zone2_state == 2) {  /* ZONE2_HEATING || ZONE2_HOLDING */
                heater1.setpoint = 45.0f;  /* 更新加热片1的目标温度为45摄氏度 */
            } else {
                heater1.setpoint = 0.0f;   /* 其他状态停止加热 */
            }
            
            heater2.setpoint = 48.0f;                         /* 更新加热片2的目标温度 */
            d1 = UpdateHeater(2, &heater1, GetTemp, &temp1);  /* TIM2_CH2 -> heater1, DQ1 PB11 */
            d2 = UpdateHeater(3, &heater2, GetTemp2, &temp2); /* TIM2_CH3 -> heater2, DQ2 PA12 */
            battery_voltage = Read_Battery_Voltage();         /* 读取电池电压 */
            
            /* 更新恒温区1档状态 */
            UpdateZone1State(temp1, &heater1);
            /* 更新恒温区2档状态 */
            UpdateZone2State(temp1, &heater1);

            /* 读取 PA10 (充电状态)：0=充电中，1=充满/无充电 */
            uint8_t pa10 = gpio_input_bit_get(GPIOA, GPIO_PIN_10);

            /* 按新需求更新 LED 行为:
               充满电:  PA10 高 且 电压 > 3.12V -> PA9 持续绿灯
               充电中:  PA10 低 且 电压 < 3.12V -> PA9 绿灯闪烁
               快没电:  PA10 高 且 电压 < 2.3V   -> PA8 红灯闪烁
            */
            /* 更新 LED 显示电池状态 */
            UpdatePowerLEDs(battery_voltage, pa10);
            /* 更新 恒温区 / 裂解区 指示灯 */
            UpdateZoneLEDs();
            printf("%.2f, %d, %.2f, %d, %.2f\r\n", temp1, d1, temp2, d2, battery_voltage);
        }
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

/* 更新 LED 显示电池状态 */
static void UpdatePowerLEDs(float battery_voltage, uint8_t pa10)
{
    /* 按新需求更新 LED 行为:
       充满电:  PA10 高 且 电压 > 3.12V -> PA9 持续绿灯
       充电中:  PA10 低 且 电压 < 3.12V -> PA9 绿灯闪烁
       快没电:  PA10 高 且 电压 < 2.3V   -> PA8 红灯闪烁
    */
    red_blink_enable = 0;
    red_steady_on = 0;
    green_blink_enable = 0;
    green_steady_on = 0;

    if (pa10) { /* PA10 高 */
        if (battery_voltage > 3.12f) {
            /* 充满电：请求 PA9 持续绿灯（由 ISR 执行） */
            green_steady_on = 1;
        } else if (battery_voltage < 2.3f) {
            /* 快没电：PA8 红灯闪烁 */
            red_blink_enable = 1;
        }
    } else { /* PA10 低 -> 充电中 */
        if (battery_voltage < 3.12f) {
            /* 充电中且电压低于 3.12V：绿灯闪烁 */
            green_blink_enable = 1;
        } else if (battery_voltage >= 3.12f) {
            /* 充满电：请求 PA9 持续绿灯（由 ISR 执行） */
            green_steady_on = 1;
        }
    }
}

/* 更新 恒温区 与 裂解区 指示灯 状态
   恒温区输入: PA4(关), PA5(1档), PA7(2档) 低电平有效
   恒温区微动: PA6 低电平表示有试管
   恒温区输出:
     1档 绿 -> PA15 低, PB3 高
     1档 插管 红 -> PA15 高, PB3 低
     2档 绿 -> PB4 低, PB5 高
     2档 插管 红 -> PB4 高, PB5 低

   裂解区输入: PB7(1档), PB8(2档), PB9(3档) 低电平有效
   裂解区微动: PB6 低电平表示有试管
   裂解区输出:
     1档 绿 -> PB12 低, PB13 高
     1档 插管 红 -> PB12 高, PB13 低
     2档 绿 -> PB14 低, PB15 高
     2档 插管 红 -> PB14 高, PB15 低
     3档 绿 -> PB12/14 低, PB13/15 高
     3档 插管 红 -> PB12/14 高, PB13/15 低
*/
static void UpdateZoneLEDs(void)
{
    uint8_t pa4 = gpio_input_bit_get(GPIOA, GPIO_PIN_4);
    uint8_t pa5 = gpio_input_bit_get(GPIOA, GPIO_PIN_5);
    uint8_t pa7 = gpio_input_bit_get(GPIOA, GPIO_PIN_7);
    uint8_t pa6 = gpio_input_bit_get(GPIOA, GPIO_PIN_6); /* 恒温区微动 */
    uint8_t pb6 = gpio_input_bit_get(GPIOB, GPIO_PIN_6); /* 裂解区微动 */

    /* 先将恒温区相关输出置为默认 OFF (reset) */
    gpio_bit_reset(GPIOA, GPIO_PIN_15);
    gpio_bit_reset(GPIOB, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);

    /* 恒温区: 优先判断 1 档 -> 2 档 -> 关/无动作 */
    if (pa5 == 0) {
        /* 1 档 */
        if (pa6 == 0) {
            /* 插管: 根据恒温区1档状态控制LED */
            if (zone1_state == 3) {  /* ZONE1_COMPLETE */
                /* 完成状态: 根据LED状态控制 */
                if (zone1_led_state == 1) {  /* LED_RED */
                    /* 红灯: PA15 高, PB3 低 */
                    gpio_bit_set(GPIOA, GPIO_PIN_15);
                    gpio_bit_reset(GPIOB, GPIO_PIN_3);
                } else if (zone1_led_state == 2) {  /* LED_GREEN */
                    /* 绿灯: PA15 低, PB3 高 */
                    gpio_bit_reset(GPIOA, GPIO_PIN_15);
                    gpio_bit_set(GPIOB, GPIO_PIN_3);
                } else if (zone1_led_state == 3) {  /* LED_BLINK_RED_GREEN */
                    /* 红绿交替闪烁 */
                    if (zone1_blink_state == 0) {
                        /* 红灯: PA15 高, PB3 低 */
                        gpio_bit_set(GPIOA, GPIO_PIN_15);
                        gpio_bit_reset(GPIOB, GPIO_PIN_3);
                    } else {
                        /* 绿灯: PA15 低, PB3 高 */
                        gpio_bit_reset(GPIOA, GPIO_PIN_15);
                        gpio_bit_set(GPIOB, GPIO_PIN_3);
                    }
                } else {
                    /* 默认: 红灯: PA15 高, PB3 低 */
                    gpio_bit_set(GPIOA, GPIO_PIN_15);
                    gpio_bit_reset(GPIOB, GPIO_PIN_3);
                }
            } else {
                /* 其他状态: 红灯: PA15 高, PB3 低 */
                gpio_bit_set(GPIOA, GPIO_PIN_15);
                gpio_bit_reset(GPIOB, GPIO_PIN_3);
            }
        } else {
            /* 未插管 -> 绿灯: PA15 低, PB3 高 */
            gpio_bit_reset(GPIOA, GPIO_PIN_15);
            gpio_bit_set(GPIOB, GPIO_PIN_3);
        }
    } else if (pa7 == 0) {
        /* 2 档 */
        if (pa6 == 0) {
            /* 插管: 根据恒温区2档状态控制LED */
            if (zone2_state == 3) {  /* ZONE2_COMPLETE */
                /* 完成状态: 根据LED状态控制 */
                if (zone2_led_state == 1) {  /* LED_RED */
                    /* 红灯: PB4 高, PB5 低 */
                    gpio_bit_set(GPIOB, GPIO_PIN_4);
                    gpio_bit_reset(GPIOB, GPIO_PIN_5);
                } else if (zone2_led_state == 2) {  /* LED_GREEN */
                    /* 绿灯: PB4 低, PB5 高 */
                    gpio_bit_reset(GPIOB, GPIO_PIN_4);
                    gpio_bit_set(GPIOB, GPIO_PIN_5);
                } else if (zone2_led_state == 3) {  /* LED_BLINK_RED_GREEN */
                    /* 红绿交替闪烁 */
                    if (zone2_blink_state == 0) {
                        /* 红灯: PB4 高, PB5 低 */
                        gpio_bit_set(GPIOB, GPIO_PIN_4);
                        gpio_bit_reset(GPIOB, GPIO_PIN_5);
                    } else {
                        /* 绿灯: PB4 低, PB5 高 */
                        gpio_bit_reset(GPIOB, GPIO_PIN_4);
                        gpio_bit_set(GPIOB, GPIO_PIN_5);
                    }
                } else {
                    /* 默认: 红灯: PB4 高, PB5 低 */
                    gpio_bit_set(GPIOB, GPIO_PIN_4);
                    gpio_bit_reset(GPIOB, GPIO_PIN_5);
                }
            } else {
                /* 其他状态: 红灯: PB4 高, PB5 低 */
                gpio_bit_set(GPIOB, GPIO_PIN_4);
                gpio_bit_reset(GPIOB, GPIO_PIN_5);
            }
        } else {
            /* 未插管 -> 绿灯: PB4 低, PB5 高 */
            gpio_bit_reset(GPIOB, GPIO_PIN_4);
            gpio_bit_set(GPIOB, GPIO_PIN_5);
        }
    } else {
        /* OFF 或其他: 保持默认关闭 */
        (void)pa4; /* 未使用的变量占位，PA4 表示关 */
    }

    /* 裂解区: 先清除输出，再根据档位设置 */
    gpio_bit_reset(GPIOB, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);

    if (gpio_input_bit_get(GPIOB, GPIO_PIN_7) == 0) {
        /* 裂解区 1 档 */
        if (pb6 == 0) {
            gpio_bit_set(GPIOB, GPIO_PIN_12);
            gpio_bit_reset(GPIOB, GPIO_PIN_13);
        } else {
            gpio_bit_reset(GPIOB, GPIO_PIN_12);
            gpio_bit_set(GPIOB, GPIO_PIN_13);
        }
    } else if (gpio_input_bit_get(GPIOB, GPIO_PIN_8) == 0) {
        /* 裂解区 2 档 */
        if (pb6 == 0) {
            gpio_bit_set(GPIOB, GPIO_PIN_14);
            gpio_bit_reset(GPIOB, GPIO_PIN_15);
        } else {
            gpio_bit_reset(GPIOB, GPIO_PIN_14);
            gpio_bit_set(GPIOB, GPIO_PIN_15);
        }
    } else if (gpio_input_bit_get(GPIOB, GPIO_PIN_9) == 0) {
        /* 裂解区 3 档 (两组同时) */
        if (pb6 == 0) {
            gpio_bit_set(GPIOB, GPIO_PIN_12);
            gpio_bit_reset(GPIOB, GPIO_PIN_13);
            gpio_bit_set(GPIOB, GPIO_PIN_14);
            gpio_bit_reset(GPIOB, GPIO_PIN_15);
        } else {
            gpio_bit_reset(GPIOB, GPIO_PIN_12);
            gpio_bit_set(GPIOB, GPIO_PIN_13);
            gpio_bit_reset(GPIOB, GPIO_PIN_14);
            gpio_bit_set(GPIOB, GPIO_PIN_15);
        }
    }
}

/* 更新恒温区1档状态 */
static void UpdateZone1State(float temp1, Heater_t *heater1)
{
    uint8_t pa5 = gpio_input_bit_get(GPIOA, GPIO_PIN_5);
    uint8_t pa6 = gpio_input_bit_get(GPIOA, GPIO_PIN_6);
    
    /* 检查是否处于1档且有试管 */
    if (pa5 == 0 && pa6 == 0) {
        switch (zone1_state) {
            case 0:  /* ZONE1_IDLE */
                /* 空闲状态，开始加热 */
                zone1_state = 1;  /* ZONE1_HEATING */
                zone1_timer = 0;
                zone1_led_state = 1;  /* LED_RED */
                break;
                
            case 1:  /* ZONE1_HEATING */
                /* 加热中，检查是否到达目标温度 */
                if (temp1 >= 44.0f) {  /* 到达44度认为接近目标温度 */
                    zone1_state = 2;  /* ZONE1_HOLDING */
                    zone1_timer = 0;
                }
                break;
                
            case 2:  /* ZONE1_HOLDING */
                /* 恒温计时中，每100ms增加一次计数 */
                zone1_timer++;
                /* 10分钟 = 600秒 = 6000个100ms */
                if (zone1_timer >= 6000) {
                    /* 计时结束，停止加热 */
                    zone1_state = 3;  /* ZONE1_COMPLETE */
                    zone1_led_state = 3;  /* LED_BLINK_RED_GREEN */
                    zone1_blink_count = 0;
                    zone1_blink_state = 0;
                }
                break;
                
            case 3:  /* ZONE1_COMPLETE */
                /* 完成状态，保持LED状态 */
                break;
                
            default:
                zone1_state = 0;  /* ZONE1_IDLE */
                break;
        }
    } else {
        /* 不在1档或没有试管，重置状态 */
        zone1_state = 0;  /* ZONE1_IDLE */
        zone1_led_state = 0;  /* LED_OFF */
        zone1_timer = 0;
        zone1_blink_count = 0;
        zone1_blink_state = 0;
    }
}

/* 更新恒温区2档状态 */
static void UpdateZone2State(float temp1, Heater_t *heater1)
{
    uint8_t pa7 = gpio_input_bit_get(GPIOA, GPIO_PIN_7);
    uint8_t pa6 = gpio_input_bit_get(GPIOA, GPIO_PIN_6);
    
    /* 检查是否处于2档且有试管 */
    if (pa7 == 0 && pa6 == 0) {
        switch (zone2_state) {
            case 0:  /* ZONE2_IDLE */
                /* 空闲状态，开始加热 */
                zone2_state = 1;  /* ZONE2_HEATING */
                zone2_timer = 0;
                zone2_led_state = 1;  /* LED_RED */
                break;
                
            case 1:  /* ZONE2_HEATING */
                /* 加热中，检查是否到达目标温度 */
                if (temp1 >= 44.0f) {  /* 到达44度认为接近目标温度 */
                    zone2_state = 2;  /* ZONE2_HOLDING */
                    zone2_timer = 0;
                }
                break;
                
            case 2:  /* ZONE2_HOLDING */
                /* 恒温计时中，每100ms增加一次计数 */
                zone2_timer++;
                /* 15分钟 = 900秒 = 9000个100ms */
                if (zone2_timer >= 9000) {
                    /* 计时结束，停止加热 */
                    zone2_state = 3;  /* ZONE2_COMPLETE */
                    zone2_led_state = 3;  /* LED_BLINK_RED_GREEN */
                    zone2_blink_count = 0;
                    zone2_blink_state = 0;
                }
                break;
                
            case 3:  /* ZONE2_COMPLETE */
                /* 完成状态，保持LED状态 */
                break;
                
            default:
                zone2_state = 0;  /* ZONE2_IDLE */
                break;
        }
    } else {
        /* 不在2档或没有试管，重置状态 */
        zone2_state = 0;  /* ZONE2_IDLE */
        zone2_led_state = 0;  /* LED_OFF */
        zone2_timer = 0;
        zone2_blink_count = 0;
        zone2_blink_state = 0;
    }
}
