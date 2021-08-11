#include "encoder.h"
  
#define wheelDiam 0.08     //车轮直径
#define pulseNum  4680     //编码器一圈脉冲数

uint32_t PulseValue1=0;      //脉冲计数
uint32_t PulseValue2=0;      //脉冲计数
int16_t overflowTime=0;     //溢出次数
uint16_t encoderCnt_A = 30000;
uint16_t encoderCnt_B = 30000;
uint16_t encoderCnt_C = 30000;
uint16_t encoderCnt_D = 30000;
uint16_t cnt_Temp_A = 0;
uint16_t cnt_Temp_B = 0;
uint16_t cnt_Temp_C = 0;
uint16_t cnt_Temp_D = 0;
uint16_t RPM_A = 0;
uint16_t RPM_B = 0;
uint16_t RPM_C = 0;
uint16_t RPM_D = 0;
float speed = 0;
//100ms定时中断
__interrupt void TIMER_TA1_A0_IRQ_Handler()
{ 
   cnt_Temp_A = encoderCnt_A - 30000;
   cnt_Temp_B = encoderCnt_B - 30000;
   cnt_Temp_C = encoderCnt_C - 30000;
   cnt_Temp_D = encoderCnt_D - 30000;
   encoderCnt_A = 30000;
   encoderCnt_B = 30000;
   encoderCnt_C = 30000;
   encoderCnt_D = 30000;
}


__interrupt void GPIO_IRQ_Handler()
{
  //2.0中断
 if(GPIO_GetITStatus(P2,0)==TRUE)
 {
    encoderCnt_A++;
  GPIO_ClearITPendingBit(P2,0);                //清除位中断标志
 }
 //2.2中断
  if(GPIO_GetITStatus(P2,2)==TRUE)
 {
    encoderCnt_B++;
  GPIO_ClearITPendingBit(P2,2);                //清除位中断标志
 }
 //2.5中断
   if(GPIO_GetITStatus(P2,5)==TRUE)
 {
    encoderCnt_C++;
  GPIO_ClearITPendingBit(P2,5);                //清除位中断标志
 }
 //2.6中断
   if(GPIO_GetITStatus(P2,6)==TRUE)
 {
    encoderCnt_D++;
  GPIO_ClearITPendingBit(P2,6);                //清除位中断标志
 }
 
}


void Encoder_Init()
{  
  
  Set_Vector_Handler(VECTOR_TIMER1_A0,TIMER_TA1_A0_IRQ_Handler);    //设置中断向量，最好先设置中断向量，在开启中断
  TIMER_Interval_Ms(TIMER_A1,100);                    //初始化一个100ms的定时中断
  TIMER_ITConfig (TIMER_A1,TIMER_CCR0_IRQn,ENABLE);              //使能TIMER的某一个中断
  
  Set_Vector_Handler(VECTOR_PORT2,GPIO_IRQ_Handler);    //设置中断向量，最好先设置中断向量，在开启中断
  GPIO_Init     (P2,0 ,GPI|PULL_UP|IRQ_FALLING);                   //设为上拉且下降沿触发中断
  GPIO_ITConfig (P2,0 ,ENABLE);                           //使能中断
  GPIO_Init     (P2,2 ,GPI|PULL_UP|IRQ_FALLING);                   //设为上拉且下降沿触发中断
  GPIO_ITConfig (P2,2 ,ENABLE);                           //使能中断
  GPIO_Init     (P2,5 ,GPI|PULL_UP|IRQ_FALLING);                   //设为上拉且下降沿触发中断
  GPIO_ITConfig (P2,5 ,ENABLE);                           //使能中断
  GPIO_Init     (P2,6 ,GPI|PULL_UP|IRQ_FALLING);                   //设为上拉且下降沿触发中断
  GPIO_ITConfig (P2,6 ,ENABLE);                           //使能中断
  
}

void speed_Handle()
{
//  UART_Printf(UART1,"A:%d B:%d C:%d D:%d",cnt_Temp_A,cnt_Temp_B,cnt_Temp_C,cnt_Temp_D);
  speed = (cnt_Temp_A+cnt_Temp_B+cnt_Temp_C+cnt_Temp_D)/4/pulseNum*wheelDiam*3.14159*600;
  RPM_A = cnt_Temp_A *600 /pulseNum;
  RPM_B = cnt_Temp_B *600 /pulseNum;
  RPM_C = cnt_Temp_C *600 /pulseNum;
  RPM_D = cnt_Temp_D *600 /pulseNum;
}
