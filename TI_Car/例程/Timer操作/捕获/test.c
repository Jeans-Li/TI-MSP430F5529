#include "include.h"

uint32_t PulseValue=0;
uint32_t PulseValue_2=0;
int16_t overflowTime=0;
int16_t overflowTime_2=0;
__interrupt void GPIO_IRQ_Handler_1()
{
   if(GPIO_GetITStatus(P2,5)==TRUE)
   {
      if((GPIO_ReadBit(P2,0))==1u)
    {
        overflowTime++;
    }
  else
  {
    overflowTime--;
  }
  LED_On(LED2);
   GPIO_ClearITPendingBit(P2,5);                //清除位中断标志
//   overflowTime_2++;
//   LED_Turn(LED1);               //指示灯闪烁
  }
}
__interrupt void GPIO_IRQ_Handler_2()//溢出中断
{  
   if(GPIO_GetITStatus(P2,7)==TRUE)
   {
  if((GPIO_ReadBit(P2,4))==1u)
{
    overflowTime_2++;
}
  else
  {
    overflowTime_2--;
}
  GPIO_ClearITPendingBit(P2,7);                //清除位中断标志
}
}
__interrupt void TIMER_TA0_A0_IRQ_Handler()
{
//   LED_Turn(LED2);               //指示灯闪烁   
   //读取脉冲数
   PulseValue =overflowTime ;//TIMER_Pluse_GetValue    (TIMER_A1,overflowTime)
   overflowTime=0; 
   OLED_DispDecAt(FONT_ASCII_6X8,0,0,PulseValue,8);    //在指定位置显示一个整形数字
////    TIMER_Pluse_Clear   (TIMER_A1);//清除溢出次数
    PulseValue_2 = overflowTime_2; //TIMER_Pluse_GetValue    (TIMER_A2,overflowTime_2)
    overflowTime_2=0;              //清除溢出次数
//   TIMER_Pluse_Clear   (TIMER_A2);
}
void main()
{
  DisableInterrupts();          //禁止总中断
  
  LED_Init(LED_ALL);              //LED灯初始化
  OLED_Init();
  GPIO_MultiBits_Init(P2,0,GPI|PULL_UP);//编码器方向引脚1
  GPIO_MultiBits_Init(P2,4,GPI|PULL_UP);//编码器方向引脚2
  GPIO_MultiBits_Init(P2,5,GPI|PULL_UP|IRQ_FALLING);//编码器IO中断引脚1
  GPIO_MultiBits_Init(P2,7,GPI|PULL_UP|IRQ_FALLING);//编码器IO中断引脚2

  
  Set_Vector_Handler(VECTOR_TIMER0_A0,TIMER_TA0_A0_IRQ_Handler);    //设置中断向量，最好先设置中断向量，在开启中断
  TIMER_Interval_Ms(TIMER_A0,1000);                    //初始化一个1000ms的定时中断
  TIMER_ITConfig (TIMER_A0,TIMER_CCR0_IRQn,ENABLE);              //使能TIMER的某一个中断

  Set_Vector_Handler(VECTOR_PORT1,GPIO_IRQ_Handler_1);    //设置中断向量，最好先设置中断向量，在开启中断
  
  GPIO_ITConfig(P2,5,ENABLE);//使能IO中断
  GPIO_ClearITPendingBit(P2,5);                //清除位中断标志
  
  Set_Vector_Handler(VECTOR_PORT2,GPIO_IRQ_Handler_2);    //设置中断向量，最好先设置中断向量，在开启中断
  GPIO_ITConfig(P2,7,ENABLE);//使能IO中断
  GPIO_ClearITPendingBit(P2,7);                //清除位中断标志
  EnableInterrupts();
  while(1)
  {  

  }
}
