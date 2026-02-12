#include "config.h"
#include "gd32e23x_usart.h"
#include "gd32e23x_adc.h"
#include "owmy.h"
#include "ow2.h"
#include "delay.h"
#include "gd32e23x_timer.h"
#include "gd32e23x_rcu.h"
#include "gd32e23x_misc.h"

/* effective timer period (ARR+1) used for duty mapping */
uint32_t g_timer2_period = 0;

/* local helper: init USART1 on PA2 (TX) / PA3 (RX) */
static void usart1_init_local(void)
{
    /* enable clocks */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_USART1);

    /* configure PA2 (TX) and PA3 (RX) as AF1 */
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_2 | GPIO_PIN_3);
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_2 | GPIO_PIN_3);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);

    /* configure USART1 */
    usart_deinit(USART1);
    usart_baudrate_set(USART1, 115200U);
    usart_word_length_set(USART1, USART_WL_8BIT);
    usart_stop_bit_set(USART1, USART_STB_1BIT);
    usart_parity_config(USART1, USART_PM_NONE);
    usart_transmit_config(USART1, USART_TRANSMIT_ENABLE);
    usart_receive_config(USART1, USART_RECEIVE_DISABLE);
    usart_enable(USART1);
}

/* IO_Config: do all board-level IO initialization (LED, OW, USART) */
void IO_Config(void)
{
    /* USART1 init */
    usart1_init_local();

    /* OneWire init (OW_Init configures the selected DQ pin) */
    OW_Init();
    /* init second DQ on PA12 */
    OW2_Init();

    /* LED: PC13 */
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOB); /* keep PB clock available for OW if needed */
    gpio_mode_set(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_13);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13);
    gpio_bit_reset(GPIOC, GPIO_PIN_13);
}

/* 100ms flag (set in IRQ) */
volatile uint8_t flag_100ms = 0;

/* 200ms flag (set in IRQ) */
volatile uint8_t flag_200ms = 0;

/* configure TIMER2: PWM on CH2 (PB0) and CH3 (PB1), update interrupt every 100ms */
/**
 * @brief 配置TIMER2以生成指定频率的PWM信号
 * @param pwm_freq_hz 期望的PWM频率(Hz)
 */
void Config_Timer2_Init(uint32_t pwm_freq_hz)
{
    timer_parameter_struct initpara;    // 定时器基本参数结构体
    timer_oc_parameter_struct ocpara;   // 定时器输出比较参数结构体
    uint32_t pclk1;                    // APB1总线时钟频率
    uint32_t prescaler;                // 定时器预分频值
    uint32_t arr;                      // 自动重装载值

    /* enable clocks */
    rcu_periph_clock_enable(RCU_TIMER2);
    rcu_periph_clock_enable(RCU_GPIOB);

    /* configure PB0/PB1 as AF for TIMER2_CH2/CH3 */
    gpio_af_set(GPIOB, GPIO_AF_1, GPIO_PIN_0 | GPIO_PIN_1);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_0 | GPIO_PIN_1);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_0 | GPIO_PIN_1);
    /* choose ARR (period) from PWM_RESOLUTION and compute prescaler to meet pwm_freq_hz */
    pclk1 = rcu_clock_freq_get(CK_APB1);
    arr = PWM_RESOLUTION - 1U;
    /* try to find prescaler <= 0xFFFF; if not, reduce arr */
    while (1) {
        uint64_t denom = (uint64_t)pwm_freq_hz * (uint64_t)(arr + 1U);
        if (denom == 0) denom = 1;
        uint64_t pres64 = (uint64_t)pclk1 / denom;
        if (pres64 == 0) pres64 = 1;
        if (pres64 - 1U <= 0xFFFFU) {
            prescaler = (uint32_t)(pres64 - 1U);
            break;
        }
        if (arr > 16U) {
            arr = arr / 2U; /* reduce resolution to fit prescaler */
        } else {
            /* fall back to max prescaler */
            prescaler = 0xFFFFU;
            break;
        }
    }

    timer_struct_para_init(&initpara);
    initpara.prescaler = (uint16_t)prescaler;
    initpara.alignedmode = TIMER_COUNTER_EDGE;
    initpara.counterdirection = TIMER_COUNTER_UP;
    initpara.period = (uint32_t)arr; /* ARR */
    initpara.clockdivision = TIMER_CKDIV_DIV1;
    timer_init(TIMER2, &initpara);

    /* store effective period for duty mapping */
    g_timer2_period = (uint32_t)arr + 1U;

    /* configure OC for CH2 and CH3, PWM mode0 */
    timer_channel_output_struct_para_init(&ocpara);
    ocpara.outputstate = TIMER_CCX_ENABLE;
    ocpara.outputnstate = TIMER_CCXN_DISABLE;
    ocpara.ocpolarity = TIMER_OC_POLARITY_HIGH;
    ocpara.ocnpolarity = TIMER_OCN_POLARITY_HIGH;
    ocpara.ocidlestate = TIMER_OC_IDLE_STATE_LOW;
    ocpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

    timer_channel_output_config(TIMER2, TIMER_CH_2, &ocpara);
    timer_channel_output_mode_config(TIMER2, TIMER_CH_2, TIMER_OC_MODE_PWM0);
    timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_2, 0);
    timer_channel_output_shadow_config(TIMER2, TIMER_CH_2, TIMER_OC_SHADOW_DISABLE);

    timer_channel_output_config(TIMER2, TIMER_CH_3, &ocpara);
    timer_channel_output_mode_config(TIMER2, TIMER_CH_3, TIMER_OC_MODE_PWM0);
    timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_3, 0);
    timer_channel_output_shadow_config(TIMER2, TIMER_CH_3, TIMER_OC_SHADOW_DISABLE);

    /* enable update interrupt and NVIC */
    timer_interrupt_enable(TIMER2, TIMER_INT_UP);
    nvic_irq_enable(TIMER2_IRQn, 2U);

    /* enable timer */
    timer_enable(TIMER2);
}

/* set heater PWM duty on channel 2 or 3 (channel param: 2 or 3) */
void SetHeaterDuty(uint8_t channel, uint8_t duty_percent)
{
    if (duty_percent > 100) duty_percent = 100;
    /* map 0..100 -> 0..(period) */
    uint32_t period = (g_timer2_period == 0) ? PWM_RESOLUTION : g_timer2_period;
    uint32_t pulse = ((uint32_t)duty_percent * period) / 100U; /* map 0..100 -> 0..period */
    if (channel == 2) {
        timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_2, pulse);
    } else if (channel == 3) {
        timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_3, pulse);
    }
}

/* ADC configuration for battery voltage sampling on PA1 (ADC_IN1) */
void ADC_Config(void)
{
    /* enable GPIOA and ADC clock */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_ADC);
    
    /* configure ADC clock */
    rcu_adc_clock_config(RCU_ADCCK_APB2_DIV4);
    
    /* configure PA1 as analog input (ADC_IN1) */
    gpio_mode_set(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_PIN_1);
    
    /* ADC data alignment config */
    adc_data_alignment_config(ADC_DATAALIGN_RIGHT);
    
    /* ADC channel length config */
    adc_channel_length_config(ADC_REGULAR_CHANNEL, 1U);
    
    /* ADC regular channel config */
    adc_regular_channel_config(0U, ADC_CHANNEL_1, ADC_SAMPLETIME_55POINT5);
    
    /* ADC external trigger config */
    adc_external_trigger_source_config(ADC_REGULAR_CHANNEL, ADC_EXTTRIG_REGULAR_NONE);
    
    /* ADC external trigger enable */
    adc_external_trigger_config(ADC_REGULAR_CHANNEL, ENABLE);
    
    /* enable ADC */
    adc_enable();
    Delay_ms(1);
    
    /* ADC calibration and reset calibration */
    adc_calibration_enable();
}

/* read battery voltage (returns voltage in volts) */
float Read_Battery_Voltage(void)
{
    /* start ADC software trigger */
    adc_software_trigger_enable(ADC_REGULAR_CHANNEL);
    
    /* wait for end of conversion */
    while(RESET == adc_flag_get(ADC_FLAG_EOC)) {
    }
    
    /* read ADC conversion result */
    uint16_t adc_value = adc_regular_data_read();
    
    /* convert ADC value to voltage (assuming 3.3V reference) */
    float voltage = (float)adc_value * 3.3f / 4096.0f;
    
    return voltage;
}
