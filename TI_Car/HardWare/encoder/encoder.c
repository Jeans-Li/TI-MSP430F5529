#include "encoder.h"
  
#define wheelDiam 0.08     //����ֱ��
#define pulseNum  4680     //������һȦ������

uint32_t PulseValue1=0;      //�������
uint32_t PulseValue2=0;      //�������
int16_t overflowTime=0;     //�������
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
//100ms��ʱ�ж�
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
  //2.0�ж�
 if(GPIO_GetITStatus(P2,0)==TRUE)
 {
    encoderCnt_A++;
  GPIO_ClearITPendingBit(P2,0);                //���λ�жϱ�־
 }
 //2.2�ж�
  if(GPIO_GetITStatus(P2,2)==TRUE)
 {
    encoderCnt_B++;
  GPIO_ClearITPendingBit(P2,2);                //���λ�жϱ�־
 }
 //2.5�ж�
   if(GPIO_GetITStatus(P2,5)==TRUE)
 {
    encoderCnt_C++;
  GPIO_ClearITPendingBit(P2,5);                //���λ�жϱ�־
 }
 //2.6�ж�
   if(GPIO_GetITStatus(P2,6)==TRUE)
 {
    encoderCnt_D++;
  GPIO_ClearITPendingBit(P2,6);                //���λ�жϱ�־
 }
 
}


void Encoder_Init()
{  
  
  Set_Vector_Handler(VECTOR_TIMER1_A0,TIMER_TA1_A0_IRQ_Handler);    //�����ж�����������������ж��������ڿ����ж�
  TIMER_Interval_Ms(TIMER_A1,100);                    //��ʼ��һ��100ms�Ķ�ʱ�ж�
  TIMER_ITConfig (TIMER_A1,TIMER_CCR0_IRQn,ENABLE);              //ʹ��TIMER��ĳһ���ж�
  
  Set_Vector_Handler(VECTOR_PORT2,GPIO_IRQ_Handler);    //�����ж�����������������ж��������ڿ����ж�
  GPIO_Init     (P2,0 ,GPI|PULL_UP|IRQ_FALLING);                   //��Ϊ�������½��ش����ж�
  GPIO_ITConfig (P2,0 ,ENABLE);                           //ʹ���ж�
  GPIO_Init     (P2,2 ,GPI|PULL_UP|IRQ_FALLING);                   //��Ϊ�������½��ش����ж�
  GPIO_ITConfig (P2,2 ,ENABLE);                           //ʹ���ж�
  GPIO_Init     (P2,5 ,GPI|PULL_UP|IRQ_FALLING);                   //��Ϊ�������½��ش����ж�
  GPIO_ITConfig (P2,5 ,ENABLE);                           //ʹ���ж�
  GPIO_Init     (P2,6 ,GPI|PULL_UP|IRQ_FALLING);                   //��Ϊ�������½��ش����ж�
  GPIO_ITConfig (P2,6 ,ENABLE);                           //ʹ���ж�
  
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
