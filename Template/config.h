/* config.h - IO configuration header */
#ifndef __CONFIG_H
#define __CONFIG_H

#include "gd32e23x.h"

/* initialize all IO: USART1 (PA2/PA3), OneWire default pin, LED */
void IO_Config(void);

/* ADC initialization for battery voltage on PA1 */
void ADC_Config(void);

/* read battery voltage (returns voltage in volts) */
float Read_Battery_Voltage(void);

/* TIMER2 PWM and 100ms flag */
/* pwm_freq_hz: desired PWM frequency in Hz (e.g. 1000) */
void Config_Timer2_Init(uint32_t pwm_freq_hz);
/* set heater PWM duty 0..100 on channel (2 or 3) */
void SetHeaterDuty(uint8_t channel, uint8_t duty_percent);

/* PWM resolution (number of steps). Default 1000 -> 0..1000 mapping */
#define PWM_RESOLUTION 1000U

/* 100ms flag set by TIMER2 IRQ */
extern volatile uint8_t flag_100ms;

/* 200ms flag set by TIMER2 IRQ */
extern volatile uint8_t flag_200ms;

#endif /* __CONFIG_H */
