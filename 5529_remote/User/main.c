/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2016,CTGU-GB
 *     All rights reserved.
 *
 *
 * @file       main.c
* @brief      MSP430F5529 平台主程序

 * @author     CTGU-GB
 * @version    v2.7
 * @date       2016-11-26
 */
#include "include.h"

uint32_t upCount = 0;   //向上计数
uint16_t valueUp = 0;   //高电平计数
uint16_t valueDown = 0;   //低电平计数
uint8_t isUpCompare = 1;      //是否为上升沿捕获标记
uint32_t width;           //脉宽
uint16_t buffer[128] = {0};    //接收缓冲区
uint16_t bufferId = 0;        //接收缓冲区数组下标
uint8_t recFlag = 0;    //接收完成标志位
char IRCodes[4] = {0,0,0,0};



uint8_t Remote_Scan()
{
	if(recFlag)
	{
		recFlag = 0;    //防止if语句重复成立
		for(int i =0;i<4 ;i++)
		{
                  UART_Printf(UART1,"%d  ",IRCodes[i]);
		}
        }

	return IRCodes[2];
}


__interrupt void TIMER_A1_IRQ_Handler()
{
  
  
   if(TIMER_GetITStatus(TIMER_A1,TIMER_CCR1_IRQn) == TRUE)  //获取某一通道中断标志
   {
      //如果是上升沿捕获
       if( isUpCompare == 1)
       {
          valueUp = TIMER_Capture_GetValue  (TIMER_A1,TIMER_CH1);
          isUpCompare = 0;
    //        UART_Printf(UART1,"up:%d\n",valueUp);
          TIMER_Capture_SetMode   (TIMER_A1, TIMER_CH1, CAP_Falling);    //设置为下降沿捕获
          upCount = 0;
        }
       
    else
    {      
        valueDown = TIMER_Capture_GetValue(TIMER_A1,TIMER_CH1);
        isUpCompare = 1;

        TIMER_Capture_SetMode(TIMER_A1, TIMER_CH1, CAP_Rising);    //设置为上升沿捕获
        width = TIMER_Capture_CalTime_Us(valueUp,valueDown,upCount);
        if(width>4400 && width<4600)
        {
            bufferId = 0;
            buffer[bufferId++] = width;
        }
        else if(bufferId > 0)
        {
            buffer[bufferId++] = width;
            if(bufferId > 32)   //所有码一共32位已经全部收到
            {
              recFlag = 1;
              bufferId = 0;
            }      
        }
    }
    
       TIMER_ClearITPendingBit(TIMER_A1,TIMER_CCR1_IRQn);     //清除TIMER的某一个中断标志
   }
    
   if(TIMER_GetITStatus(TIMER_A1,TIMER_OverFlow_IRQn) == TRUE)//如果溢出
   {
     upCount++;          //溢出值加1
     TIMER_ClearITPendingBit(TIMER_A1,TIMER_OverFlow_IRQn);     //清除TIMER的某一个中断标志
   }
  
  for(int i = 0;i < 32; i++)
  {
      if(buffer[i+1]<1000)
      {
              IRCodes[i/8] = IRCodes[i/8]<<1;
      }
      else
      {
              IRCodes[i/8] = IRCodes[i/8]<<1;
              IRCodes[i/8] |= 0x01;
      }
  }
}


void main()
{
  DisableInterrupts();            //禁止总中断
  LED_Init(LED_ALL);              //LED灯初始化
  UART_Init        (UART1,115200);                      //初始化UART1模块,波特率115200，波特率高时最好把主时钟 设高些
  Set_Vector_Handler(VECTOR_TIMER_A1,TIMER_A1_IRQ_Handler);    //设置接收中断向量
  TIMER_Capture_Init  (TIMER_A1, TIMER_CH1, CAP_Rising);        //初始化一个通道为捕获模式， 
  TIMER_ITConfig   (TIMER_A1, TIMER_CCR1_IRQn,ENABLE);               //设置是否使能TIMER的某一个中断
  TIMER_ITConfig   (TIMER_A1, TIMER_OverFlow_IRQn,ENABLE);               //设置是否使能TIMER的某一个中断
  TIMER_Capture_Clear(TIMER_A1, TIMER_CH1);
  EnableInterrupts();
  
  while(1)
  {  
    DELAY_MS(100);
    Remote_Scan();
  }
}





