/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef OWMY_H
#define OWMY_H

/* Includes ------------------------------------------------------------------*/
#include "gd32e23x.h"
#include "delay.h"
#include <stdint.h>
#define MAXNUM         16     //120
/* 用户可自行配置us级延时 */
#define ow_Delay_us			Delay_us

/* 使用 PB11 作为 DQ */
#define GPIOOW_DQ_GPIO_PORT           		GPIOB
#define GPIOOW_DQ_PIN                   GPIO_PIN_11	//DQ
#define GPIOOW_DQ_GPIO_CLK_ENABLE()    		rcu_periph_clock_enable(RCU_GPIOB)

/* Macros for DQ manipulation (use GD32 gpio API) */
#define ow_DQ_set()    		gpio_bit_set(GPIOB, GPIO_PIN_11)
#define ow_DQ_reset()  		gpio_bit_reset(GPIOB, GPIO_PIN_11)
#define ow_DQ_get()    		gpio_input_bit_get(GPIOB, GPIO_PIN_11)

//#define ow_VDD_set()   	{ GPIOOW_VDD_GPIO_PORT->BSRR = GPIOOW_VDD_PIN; }
//#define ow_VDD_reset() 	{ GPIOOW_VDD_GPIO_PORT->BRR = GPIOOW_VDD_PIN; }


/* Exported_Functions----------------------------------------------------------*/

/**-----------------------------------------------------------------------
  * @brief  OW-GPIO初始化
  * @param  无
  * @retval None
-------------------------------------------------------------------------*/
void OW_Init(void);

/**-----------------------------------------------------------------------
  * @brief  主机向芯片发送复位脉冲并从芯片读取存在脉冲
  * @param  无
  * @retval 是否有存在脉冲
-------------------------------------------------------------------------*/
int OW_ResetPresence(void);

/**-----------------------------------------------------------------------
  * @brief  主机向芯片发送一个bit
  * @param  要发送的bit
  * @retval None
-------------------------------------------------------------------------*/
void OW_WriteBit(uint8_t bit);
	
/**-----------------------------------------------------------------------
  * @brief  主机从芯片读取一个bit
  * @param  无
  * @retval 读取到的一个bit数据
-------------------------------------------------------------------------*/
int OW_ReadBit(void);

/**-----------------------------------------------------------------------
  * @brief  主机从芯片读取两个bit（读取顺序：低位->高位）
  * @param  无
  * @retval 读取到的两个bit数据
-------------------------------------------------------------------------*/
uint8_t OW_Read2Bits(void);

/**-----------------------------------------------------------------------
  * @brief  主机向芯片发送一个字节（发送顺序：低位->高位）
  * @param  要发送的字节
  * @retval None
-------------------------------------------------------------------------*/
void OW_WriteByte(uint8_t data);

/**-----------------------------------------------------------------------
  * @brief  主机从芯片读取一个字节（读取顺序：低位->高位）
  * @param  无
  * @retval 读取到的一个字节数据
-------------------------------------------------------------------------*/
uint8_t OW_ReadByte(void);


#endif /* OWMY_H */
