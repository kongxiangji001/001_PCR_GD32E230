/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef T117_MTS4_OW_H
#define T117_MTS4_OW_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
/*打印开关*/
#define		print_en  1            //1为开启打印功能，0为关闭打印功能
#define		PR(format, ...)    if (print_en) printf(format,##__VA_ARGS__);
/*  Registers definition----------------------------------------------*/
/*Bit definition of config register*/
typedef enum 
{
//Temp_Cfg	
	//测温频率
	FRE_8TIMES          	= 0x00,   //每秒8次
	FRE_4TIMES          	= 0x20,   //每秒4次
	FRE_2TIMES          	= 0x40,   //每秒2次
	FRE_1TIMES          	= 0x60,   //每秒1次
	
	FRE_2S              	= 0x80,   //每2秒1次
	FRE_4S              	= 0xa0,   //每4秒1次
	FRE_8S              	= 0xc0,		//每8秒1次
	FRE_16S              	= 0xe0,	  //每16秒1次
	//平均次数
	AVG_1               	= 0xe7,   //位清0，转换时间2.1ms
	AVG_8	                = 0x08,   //转换时间5.2ms
	AVG_16                = 0x10,   //转换时间8.5ms
	AVG_32                = 0x18,   //转换时间15.3ms
	//低功耗模式	
	OFF_PD                = 0xfe,   //位清0，不进入低功耗模式
	ON_PD                 = 0x01,   //进入低功耗模式	
	
//Alert_Mode 
	//报警开关
	OFF_ALERT             = 0x00,   //清0，报警关
	ON_ALERT              = 0x80,   //报警开
	//Mode
	TL_CLEAR          	  = 0xbf,   //位清0，TL为报警清除门限阈值
	TL_ALERT          	  = 0x40,   //TL为报警门限下阈值
	//极性
	ALERT_LO          	  = 0xdf,   //位清0，低电平有效
	ALERT_HI          	  = 0x20,   //高电平有效	
	//报警端口模式选择
	ALERT_IO          	  = 0xef,   //位清0，用作温度报警
	CONVERT_FINI          = 0x10,   //用作测温完成标志	
} REG;


/*Definition of conversion time corresponding to repeatability setting*/
/*  AVG_1     //转换时间2.1ms
	AVG_8	  //转换时间5.2ms
	AVG_16    //转换时间8.5ms
	AVG_32    //转换时间15.3ms
考虑到不同用户的单片机延时偏差，预留2ms以保证转换时间充分*/
#define tCon_A1        4	/* ms. */
#define tCon_A8        7	/* ms. */
#define tCon_A16       10	/* ms. */
#define tCon_A32       17	/* ms. */

typedef enum
{
	READ_ROM          =   0x33,
	MATCH_ROM         =   0x55,
	SKIP_ROM          =   0xcc,
	SEARCH_ROM        =   0xf0, 
 	ALARM_SEARCH	  =   0xec,
	
	COPY_PAGE0		  =   0x48,	
	SOFT_RESET		  =   0x6A,	
	
	CONVERT_T         =   0x44,	
	READ_TEMP         =   0xbc,
	
	READ_SCR          =   0xbe,
	WRITE_SCR         =   0x4e,
	
	READ_SCR_EXT      =   0xdd,
	WRITE_SCR_EXT 	  =   0x77,
} OW_CMD;

/******************  Scratchpad/SRAM  ******************/

typedef struct
{
	uint8_t T_lsb;					/*The LSB of 温度结果, RO*/
	uint8_t T_msb;					/*The MSB of 温度结果, RO*/
	uint8_t Crc_temp;
}TEMP_READ;

typedef struct
{
	uint8_t Status;
	uint8_t Temp_Cmd;  //默认值0x40：停止测量，不加热
	uint8_t Temp_Cfg;  //默认值0x69：每秒1次，AVG_8,低功耗模式开启
	uint8_t Alert_Mode;//默认值0x00：报警关，报警模式为TL解除报警，报警低电平有效，标志位表示温度报警
	uint8_t Th_lsb;
	uint8_t Th_msb;
	uint8_t Tl_lsb;
	uint8_t Tl_msb;
	uint8_t Crc_scratch;
} SCRATCHPAD_READ;

typedef struct
{	
	uint8_t Temp_Cmd;
	uint8_t Temp_Cfg;
	uint8_t Alert_Mode;
	uint8_t Th_lsb;
	uint8_t Th_msb;
	uint8_t Tl_lsb;
	uint8_t Tl_msb;
} SCRATCHPAD_WRITE;

typedef struct
{
	uint8_t User_define_0;
	uint8_t User_define_1;
	uint8_t User_define_2;
	uint8_t User_define_3;
	uint8_t User_define_4;
	uint8_t User_define_5;
	uint8_t User_define_6;
	uint8_t User_define_7;
	uint8_t User_define_8;
	uint8_t User_define_9;
	uint8_t Crc_scratch_ext;

} SCRATCHPADEXT;

typedef struct
{
	uint8_t User_define_0;
	uint8_t User_define_1;
	uint8_t User_define_2;
	uint8_t User_define_3;
	uint8_t User_define_4;
	uint8_t User_define_5;
	uint8_t User_define_6;
	uint8_t User_define_7;
	uint8_t User_define_8;
	uint8_t User_define_9;
} SCRATCHPADEXT_WRITE;


typedef struct
{
	uint8_t Romcode1;
	uint8_t Romcode2;
	uint8_t Romcode3;
	uint8_t Romcode4;
	uint8_t Romcode5;
	uint8_t Romcode6;
	uint8_t Romcode7;
	uint8_t crc_romcode;
} ROMCODE;

/*Exported functions*/
float MY_OutputtoTemp(int16_t out);
int16_t MY_TemptoOutput(float Temp);
uint8_t MY_CRC8(uint8_t *serial, uint8_t length);
int OW_ReadTemp(uint8_t *scr,uint8_t x);
int OW_ReadScratchpad(uint8_t *scr,uint8_t x);
int OW_WriteScratchpad(uint8_t *scr,uint8_t x);
int OW_ReadScratchpadExt(uint8_t *scr,uint8_t x);
int OW_SavetoE2PROMPage0(void);
int OW_Softreset(void);
uint8_t OW_SearchROM(uint8_t (*pID)[8]); 
int OW_SetConfig(uint8_t mps,uint8_t avg, uint8_t sleep,uint8_t x);
int OW_ReadConfig(uint8_t *cfg,uint8_t x);
int OW_ConvertTemp(uint8_t x);
int OW_ReadTempWaiting(uint16_t *iTemp,uint8_t x);
float GetTemp(void);
#endif /*T117_MTS4_OW_H */
