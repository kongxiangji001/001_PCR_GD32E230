/****************************************************************************************/
/*
 *
 * Copyright (C) 2020. Mysentech Inc, unpublished work. This computer 
 * program includes Confidential, Proprietary Information and is a Trade Secret of 
 * Minyuan Sensing Technology Inc.(Mysentech)  
 * All Rights Reserved.
 *
 *  Please contact <sales@mysentech.com> or contributors for further questions.
*/
/****************************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "owmy.h"
#include "T117_MTS4_OW.h"

uint8_t ID_Buff[MAXNUM][8];

/*读取方式在此处定义*/
#define SingleIC    /*总线上只有一颗芯片*/
//#define AllIC    /*总线上有一颗以上数量芯片*/
//#define SingleConvert /*总线上所有芯片一颗一颗分别测温转换*/

/**
  * @brief  把16位二进制补码表示的温度输出转换为以摄氏度为单位的温度读数
  * @param  out：有符号的16位二进制温度输出
  * @retval 以摄氏度为单位的浮点温度
*/
float MY_OutputtoTemp(int16_t out)
{
	return ((float)out/256.0 + 25.0);	
}

/**
  * @brief  把以摄氏度为单位的浮点温度值转换为16位二进制补码表示的温度值
  * @param  以摄氏度为单位的浮点温度值
  * @retval 有符号的16位二进制温度值
*/
int16_t MY_TemptoOutput(float Temp)
{
	return (int16_t)((Temp-25.0)*256.0);	
}

/**
  * @brief  计算多个字节序列的校验和
  * @param  serial：字节数组指针
  * @param  length：字节数组的长度
  * @retval 校验和（CRC）
*/
#define POLYNOMIAL 	0x131 //100110001
uint8_t MY_CRC8(uint8_t *serial, uint8_t length) 
{
       uint8_t result = 0x00;
    uint8_t pDataBuf;
    uint8_t i;

    while(length--) {
        pDataBuf = *serial++;
        for(i=0; i<8; i++) {
            if((result^(pDataBuf))&0x01) {
                result ^= 0x18;
                result >>= 1;
                result |= 0x80;
            }
            else {
                result >>= 1;
            }
            pDataBuf >>= 1;
        }
    }
    return result;
}

/**
  * @brief  搜索总线上接入的所有芯片的ROM ID
  * @param  pID：存有ROM ID的二维数组
  * @retval 搜索到的T117的个数
*/
uint8_t OW_SearchROM(uint8_t (*pID)[8])  
{   
  unsigned char k,l = 0,ConflictBit,m,n;  
  unsigned char BUFFER[MAXNUM] = {0};  
  unsigned char ss[64];
	unsigned char s = 0;  
  uint8_t num = 0;
  do  
  {  
	  if(OW_ResetPresence() == 0)//FALSE
		{			
		  return 0;//FALSE
		}
	  OW_WriteByte(SEARCH_ROM);	   
			     
		for(m=0; m<8; m++)  
    {    
      for(n=0; n<8; n++)  
      {  
				k = OW_Read2Bits();		//read two bits						
				k = k&0x03;  
        s = s>>1;  
                
				if(k == 0x02)			//0000 0010, if get data of 0
				{             
					OW_WriteBit(0);  //write 0, let devices of 0 on bus response
					ss[(m*8+n)]=0;
					//PR("SEARCH_ROM: byte: %d bit %d, hit 0\r\n",m, n);
				}  
        else if(k == 0x01)				//0000 0001, if get data of 1
				{  
					s = s|0x80;  
					OW_WriteBit(1);  //write 1, let devices of 1 on bus response   
					ss[(m*8+n)] = 1;  
					//PR("SEARCH_ROM: byte: %d bit %d, hit 1\r\n",m, n);
				}  
        else if(k == 0x00)  //if get 00, then there is confliction, needs to check  
				{                
					ConflictBit = m*8+n+1;                                       
					if(ConflictBit > BUFFER[l])  //if conflict bit greater than top of stack, then write 0   
					{                         
						OW_WriteBit(0);  
						ss[(m*8+n)] = 0;                                                
						BUFFER[++l] = ConflictBit;                         
						//PR("SEARCH_ROM: byte: %d bit %d, conflist choose branch 0\r\n",m, n);
					}  
					else if(ConflictBit < BUFFER[l])  //if conflict bit less than top of stack, then write previous data
					{  
						s = s|((ss[(m*8+n)]&0x01)<<7);  
						OW_WriteBit(ss[(m*8+n)]);  
						//PR("SEARCH_ROM: byte: %d bit %d, conflist choose branch %d\r\n",m, n,ss[(m*8+n)]);
					}  
					else if(ConflictBit == BUFFER[l])  //if conflict bit equal to top of stack, then write 1
					{  
						s = s|0x80;  
						OW_WriteBit(1);  
						ss[(m*8+n)] = 1;  
						l = l-1;  
						//PR("SEARCH_ROM: byte: %d bit %d, conflist choose branch 1\r\n",m, n);
					}  
			}  
      else  //if get 0x03(0000 0011), then there's no device on bus
			{  
//				PR("\r\nSEARCH_ROM: byte: %d bit %d, got 0x03, device not exist\r\n",m,n);
        return num;  //search finish, return number of devices
			}
    }  
    pID[num][m] = s;
		
		s = 0;
  }  
  num = num+1;
	}while(BUFFER[l] != 0&&(num < MAXNUM));   
	Delay_us(800);		
	
  return num;     //return number of devices
}

/**
  * @brief  读芯片寄存器的暂存器组
  * @param  scr：字节数组指针， 长度为 @sizeof（SCRATCHPAD_READ）
  * @param  x：要匹配的芯片在Search到的总线上全部芯片中的序号（单点使用时,x可为0）
  * @retval 读状态
*/
int OW_ReadTemp(uint8_t* scr, uint8_t x)
{
	uint8_t i, j;
	/*size < sizeof(SCRATCHPAD_READ)*/
	if (OW_ResetPresence() == 0)	//FALSE				
		return 0;//FALSE
#ifdef SingleIC    /*总线上只有一颗芯片*/
	OW_WriteByte(SKIP_ROM);
#endif
#ifndef SingleIC    /*总线上有一颗以上数量芯片*/
	OW_WriteByte(MATCH_ROM);
	for (j = 0; j < 8; j++)
	{
		OW_WriteByte(ID_Buff[x][j]);
	}
#endif

	OW_WriteByte(READ_TEMP);

	for (i = 0; i < 3; i++)
	{
		*scr++ = OW_ReadByte();
	}

	return 1;//TRUE
}

/**
  * @brief  读芯片寄存器的暂存器组
  * @param  scr：字节数组指针， 长度为 @sizeof（SCRATCHPAD_READ）
  * @param  x：要匹配的芯片在Search到的总线上全部芯片中的序号（单点使用时,x可为0）
  * @retval 读状态
*/
int OW_ReadScratchpad(uint8_t* scr, uint8_t x)
{
	uint8_t i, j;
	/*size < sizeof(SCRATCHPAD_READ)*/
	if (OW_ResetPresence() == 0)	//FALSE				
		return 0;//FALSE
#ifdef SingleIC    /*总线上只有一颗芯片*/
	OW_WriteByte(SKIP_ROM);
#endif
#ifndef SingleIC    /*总线上有一颗以上数量芯片*/
	OW_WriteByte(MATCH_ROM);
	for (j = 0; j < 8; j++)
	{
		OW_WriteByte(ID_Buff[x][j]);
	}
#endif

	OW_WriteByte(READ_SCR);
	for (i = 0; i < sizeof(SCRATCHPAD_READ); i++)
	{
		*scr++ = OW_ReadByte();
	}
	return 1;//TRUE
}

/**
  * @brief  写芯片寄存器的暂存器组
  * @param  scr：字节数组指针， 长度为 @sizeof（SCRATCHPAD_WRITE）
  * @param  x：要匹配的芯片在Search到的总线上全部芯片中的序号（单点使用时,x可为0）
  * @retval 写状态
**/
int OW_WriteScratchpad(uint8_t* scr, uint8_t x)
{
	uint8_t i, j;

	if (OW_ResetPresence() == 0)	//FALSE					
		return 0;//FALSE
#ifdef SingleIC    /*总线上只有一颗芯片*/
	OW_WriteByte(SKIP_ROM);
#endif
#ifndef SingleIC    /*总线上有一颗以上数量芯片*/
	OW_WriteByte(MATCH_ROM);
	for (j = 0; j < 8; j++)
	{
		OW_WriteByte(ID_Buff[x][j]);
}
#endif
	OW_WriteByte(WRITE_SCR);
	for (i = 0; i < sizeof(SCRATCHPAD_WRITE); i++)
	{
		OW_WriteByte(*scr++);
	}
	return 1;//TRUE
}

/**
  * @brief  读芯片寄存器的扩展暂存器组
  * @param  scr：字节数组指针， 长度为 @sizeof（SCRATCHPADEXT）
  * @param  x：要匹配的芯片在Search到的总线上全部芯片中的序号（单点使用时,x可为0）
  * @retval 读状态
**/
int OW_ReadScratchpadExt(uint8_t* scr, uint8_t x)
{
	uint8_t i, j;

	if (OW_ResetPresence() == 0)	//FALSE					
		return 0;//FALSE
#ifdef SingleIC    /*总线上只有一颗芯片*/
	OW_WriteByte(SKIP_ROM);
#endif
#ifndef SingleIC    /*总线上有一颗以上数量芯片*/
	OW_WriteByte(MATCH_ROM);
	for (j = 0; j < 8; j++)
	{
		OW_WriteByte(ID_Buff[x][j]);
}
#endif
	OW_WriteByte(READ_SCR_EXT);
	for (i = 0; i < sizeof(SCRATCHPADEXT); i++)
	{
		*scr++ = OW_ReadByte();
	}
	return 1;	//TRUE
}

/**
  * @brief  保存暂存器和扩展暂存器的内容到EEPROM的Page0，并等待编程结束
  * @param  无
  * @retval 状态
**/
int OW_SavetoE2PROMPage0(void)
{
	if (OW_ResetPresence() == 0)	//FALSE				
		return 0;//FALSE

	OW_WriteByte(SKIP_ROM);
	OW_WriteByte(COPY_PAGE0);

	/*等待擦除和编程完成*/
	Delay_ms(45);

	return 1;//TRUE	
}

/**
  * @brief  软复位，重装载全部寄存器，并等待装载结束
  * @param  无
  * @retval 状态
**/
int OW_Softreset(void)
{
	if (OW_ResetPresence() == 0)
		return 0;

	OW_WriteByte(SKIP_ROM);
	OW_WriteByte(SOFT_RESET);
	/*等待装载完成*/
	Delay_ms(2);
	return 1;
}

/**
  * @brief  启动温度测量
  * @param  x：要匹配的芯片在Search到的总线上全部芯片中的序号（总线上芯片一起测温使用时,x可为0）
  * @retval 单总线发送状态
*/
int OW_ConvertTemp(uint8_t x)
{
	uint8_t j;
	if (OW_ResetPresence() == 0)//FALSE					
		return 0;//FALSE
#ifndef SingleConvert    /*总线上只有一颗芯片*/
	OW_WriteByte(SKIP_ROM);
#endif

#ifdef SingleConvert /*总线上所有芯片一颗一颗分别测温转换*/
	OW_WriteByte(MATCH_ROM);
	for (j = 0; j < 8; j++)
	{
		OW_WriteByte(ID_Buff[x][j]);
	}
#endif

	OW_WriteByte(CONVERT_T);

	return 1;//TRUE
}

/**
  * @brief  等待转换结束后读测量结果。和@ConvertTemp联合使用
  * @param  iTemp：返回的16位温度测量结果
  * @param  x：要匹配的芯片在Search到的总线上全部芯片中的序号（单点使用时,x可为0）
  * @retval 读状态
*/
int OW_ReadTempWaiting(uint16_t* iTemp, uint8_t x)
{
	uint8_t scrb[sizeof(TEMP_READ)], ID1[9];
	TEMP_READ* scr = (TEMP_READ*)scrb;
	if (OW_ReadTemp(scrb, x) == 0)//FALSE
	{
		return 0; //FALSE /*读寄存器失败*/
	}
#ifdef SingleIC    /*总线上只有一颗芯片*/
	if (scrb[sizeof(scrb) - 1] != MY_CRC8(scrb, sizeof(scrb) - 1))
	{
		return 0;//FALSE  /*CRC验证未通过*/
	}
#endif
#ifndef SingleIC    /*总线上有一颗以上数量芯片*/
	for (uint8_t j = 0; j < 7; j++)
	{
		ID1[j] = ID_Buff[x][j];
}
	for (uint8_t j = 0; j < (sizeof(scrb)-1); j++)
	{
		ID1[j + 7] = scrb[j];
	}
	if (scrb[sizeof(scrb) - 1] != MY_CRC8(ID1, sizeof(scrb) + 6))
	{
		return 0;
	}
#endif
	/*将温度测量结果的两个字节合成为16位字。*/
	* iTemp = (uint16_t)scr->T_msb << 8 | scr->T_lsb;
	return 1;//TRUE		
}

/**
  * @brief  多点设置周期测量频率、平均次数和低功耗模式
  * @param  mps：要设置的周期测量频率（每秒测量次数），可能为下列其一
	*				@arg FRE_8TIMES		：每执行ConvertTemp一次，启动每秒8次重复测量
	*				@arg FRE_4TIMES		：每执行ConvertTemp一次，启动每秒4次重复测量
	*				@arg FRE_2TIMES		：每执行ConvertTemp一次，启动每秒2次重复测量
	*				@arg FRE_1TIMES   ：每执行ConvertTemp一次，启动每秒1次温度测量
	*				@arg FRE_2S		   	：每执行ConvertTemp一次，启动每2秒1次重复测量
	*				@arg FRE_4S		   	：每执行ConvertTemp一次，启动每4秒1次重复测量
	*				@arg FRE_8S				：每执行ConvertTemp一次，启动每8秒1次重复测量
	*				@arg FRE_16S      ：每执行ConvertTemp一次，启动每16秒1次重复测量
  * @param  avg：要设置的平均次数，可能为下列其一
	*				@arg AVG_1				：平均次数1次
	*				@arg AVG_8		    ：平均次数8次
	*				@arg AVG_16			  ：平均次数16次
	*				@arg AVG_32			  ：平均次数32次
  * @param  sleep：要设置的低功耗模式，可能为下列其一
	*				@arg OFF_PD       ：位清0，不进入低功耗模式
	*				@arg ON_PD        ：进入低功耗模式
  * @param  x：要匹配的芯片在Search到的总线上全部芯片中的序号（单点使用时,x可为0）
  * @retval 无
*/
int OW_SetConfig(uint8_t mps, uint8_t avg, uint8_t sleep, uint8_t x)
{
	uint8_t scrb[sizeof(SCRATCHPAD_READ)], ID1[15];
	SCRATCHPAD_READ* scr = (SCRATCHPAD_READ*)scrb;
	/*读9个字节。第7字节是系统配置寄存器，第8字节是系统状态寄存器。最后字节是前8个的校验和--CRC。*/
	if (OW_ReadScratchpad(scrb, x) == 0)//FALSE
	{
		return 0;//FALSE  /*CRC验证未通过*/
	}
#ifdef SingleIC    /*总线上只有一颗芯片*/
	if (scrb[sizeof(scrb) - 1] != MY_CRC8(scrb, sizeof(scrb) - 1))
	{
		return 0;//FALSE  /*CRC验证未通过*/
	}
#endif
#ifndef SingleIC    /*总线上有一颗以上数量芯片*/
	for (uint8_t j = 0; j < 7; j++)
	{
		ID1[j] = ID_Buff[x][j];
}
	for (uint8_t j = 0; j < (sizeof(scrb)-1); j++)
	{
		ID1[j + 7] = scrb[j];
	}
	if (scrb[sizeof(scrb) - 1] != MY_CRC8(ID1, sizeof(scrb) + 6))
	{
		return 0;
	}
#endif	
	/*计算接收的前8个字节的校验和，并与接收的第9个CRC字节比较。*/
	scr->Temp_Cfg &= 0x00;
	scr->Temp_Cfg |= mps;
    if(avg==AVG_1)
    {
        scr->Temp_Cfg &= avg;
    }
    else
    {
		scr->Temp_Cfg |= avg;
    }
    if(sleep==OFF_PD)
    {
        scr->Temp_Cfg &= sleep;
    }
    else
    {
		scr->Temp_Cfg |= sleep;
    }
	OW_WriteScratchpad(scrb+1, x);
	return 1;//TRUE
}

/**
  * @brief  读取周期测量频率、平均次数和低功耗模式
  * @param  cfg：cfg的寄存器值
  * @param  x：要匹配的芯片在Search到的总线上全部芯片中的序号（单点使用时,x可为0）
  * @retval 无
*/
int OW_ReadConfig(uint8_t* cfg, uint8_t x)
{
	uint8_t scrb[sizeof(SCRATCHPAD_READ)], ID1[15];
	SCRATCHPAD_READ* scr = (SCRATCHPAD_READ*)scrb;
	/*读9个字节。第7字节是系统配置寄存器，第8字节是系统状态寄存器。最后字节是前8个的校验和--CRC。*/
	if (OW_ReadScratchpad(scrb, x) == 0)//FALSE
	{
		return 0;//FALSE  /*CRC验证未通过*/
	}
#ifdef SingleIC    /*总线上只有一颗芯片*/
	if (scrb[sizeof(scrb) - 1] != MY_CRC8(scrb, sizeof(scrb) - 1))
	{
		return 0;//FALSE  /*CRC验证未通过*/
	}
#endif
#ifndef SingleIC    /*总线上有一颗以上数量芯片*/
	for (uint8_t j = 0; j < 7; j++)
	{
		ID1[j] = ID_Buff[x][j];
	}
	for (uint8_t j = 0; j < (sizeof(scrb)-1); j++)
	{
		ID1[j + 7] = scrb[j];
	}
	if (scrb[sizeof(scrb) - 1] != MY_CRC8(ID1, sizeof(scrb) + 6))
	{
		return 0;
	}
#endif	
	* cfg = scr->Temp_Cfg;
	return 1;//TRUE
}
/**
  * @brief  测温读温
  * @param  无
  * @retval 状态
*/
int GetTemp(void)
{
	float fTemp;
	unsigned short iTemp;
	int RomNum;
	unsigned char  cfg; int i=0;
	memset(ID_Buff, 0, sizeof(ID_Buff));//清零
#ifndef SingleIC
	RomNum = OW_SearchROM(ID_Buff);					//搜索当前总线上的所有芯片，将ROM ID存入数组romid，并返回芯片个数
#endif
/*总线上所有芯片一起测温，使用SKIPROM指令*/
#ifndef SingleConvert
	if (OW_ConvertTemp(i) == 0)   
	{
		return 0;
	}
#endif

#ifndef SingleIC
	for (i = 0; i < RomNum; i++)      //遍历所有芯片
	{
#endif
	#ifdef SingleConvert
		/*匹配到ID的芯片测温，使用MATCHROM指令，根据需求选择测温方式*/
		if (OW_ConvertTemp(i) == 0)   
		{
			return 0;
		}
	#endif
	
		//等待测温结束/
		Delay_ms(tCon_A32);	//默认为15.3ms温度转换时间，如果AVG是其他配置，此处的延时请修改	
		/*读温*/	
		OW_ReadTempWaiting(&iTemp, i);
		fTemp = MY_OutputtoTemp((short)iTemp);		//温度输出的浮点数
		OW_ReadConfig(&cfg, i);
		/*打印*/
  #ifndef SingleIC
		PR("Num %d ",RomNum);
		PR("ID:");
		for (int j = 0; j < 8; j++)
		{
			PR(" %2x", ID_Buff[i][j]);
		}
  #endif
	PR("  Temp=%.2f\n", fTemp);

  #ifndef SingleIC
	}
  #endif
return 1;
}
