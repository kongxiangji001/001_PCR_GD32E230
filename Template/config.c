#include "config.h"
#include "gd32e23x_usart.h"
#include "owmy.h"
#include "delay.h"

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

    /* LED: PC13 */
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOB); /* keep PB clock available for OW if needed */
    gpio_mode_set(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_13);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13);
    gpio_bit_reset(GPIOC, GPIO_PIN_13);
}
