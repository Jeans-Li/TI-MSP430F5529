/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2016,CTGU-GB
 *     All rights reserved.
 *
 *
 * @file       main.c
* @brief      MSP430F5529 ƽ̨������

 * @author     CTGU-GB
 * @version    v2.7
 * @date       2016-11-26
 */
#include "include.h"

uint32_t upCount = 0;   //���ϼ���
uint16_t valueUp = 0;   //�ߵ�ƽ����
uint16_t valueDown = 0;   //�͵�ƽ����
uint8_t isUpCompare = 1;      //�Ƿ�Ϊ�����ز�����
uint32_t width;           //����
uint16_t buffer[128] = {0};    //���ջ�����
uint16_t bufferId = 0;        //���ջ����������±�
uint8_t recFlag = 0;    //������ɱ�־λ
char IRCodes[4] = {0,0,0,0};



uint8_t Remote_Scan()
{
	if(recFlag)
	{
		recFlag = 0;    //��ֹif����ظ�����
		for(int i =0;i<4 ;i++)
		{
                  UART_Printf(UART1,"%d  ",IRCodes[i]);
		}
        }

	return IRCodes[2];
}


__interrupt void TIMER_A1_IRQ_Handler()
{
  
  
   if(TIMER_GetITStatus(TIMER_A1,TIMER_CCR1_IRQn) == TRUE)  //��ȡĳһͨ���жϱ�־
   {
      //����������ز���
       if( isUpCompare == 1)
       {
          valueUp = TIMER_Capture_GetValue  (TIMER_A1,TIMER_CH1);
          isUpCompare = 0;
    //        UART_Printf(UART1,"up:%d\n",valueUp);
          TIMER_Capture_SetMode   (TIMER_A1, TIMER_CH1, CAP_Falling);    //����Ϊ�½��ز���
          upCount = 0;
        }
       
    else
    {      
        valueDown = TIMER_Capture_GetValue(TIMER_A1,TIMER_CH1);
        isUpCompare = 1;

        TIMER_Capture_SetMode(TIMER_A1, TIMER_CH1, CAP_Rising);    //����Ϊ�����ز���
        width = TIMER_Capture_CalTime_Us(valueUp,valueDown,upCount);
        if(width>4400 && width<4600)
        {
            bufferId = 0;
            buffer[bufferId++] = width;
        }
        else if(bufferId > 0)
        {
            buffer[bufferId++] = width;
            if(bufferId > 32)   //������һ��32λ�Ѿ�ȫ���յ�
            {
              recFlag = 1;
              bufferId = 0;
            }      
        }
    }
    
       TIMER_ClearITPendingBit(TIMER_A1,TIMER_CCR1_IRQn);     //���TIMER��ĳһ���жϱ�־
   }
    
   if(TIMER_GetITStatus(TIMER_A1,TIMER_OverFlow_IRQn) == TRUE)//������
   {
     upCount++;          //���ֵ��1
     TIMER_ClearITPendingBit(TIMER_A1,TIMER_OverFlow_IRQn);     //���TIMER��ĳһ���жϱ�־
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
  DisableInterrupts();            //��ֹ���ж�
  LED_Init(LED_ALL);              //LED�Ƴ�ʼ��
  UART_Init        (UART1,115200);                      //��ʼ��UART1ģ��,������115200�������ʸ�ʱ��ð���ʱ�� ���Щ
  Set_Vector_Handler(VECTOR_TIMER_A1,TIMER_A1_IRQ_Handler);    //���ý����ж�����
  TIMER_Capture_Init  (TIMER_A1, TIMER_CH1, CAP_Rising);        //��ʼ��һ��ͨ��Ϊ����ģʽ�� 
  TIMER_ITConfig   (TIMER_A1, TIMER_CCR1_IRQn,ENABLE);               //�����Ƿ�ʹ��TIMER��ĳһ���ж�
  TIMER_ITConfig   (TIMER_A1, TIMER_OverFlow_IRQn,ENABLE);               //�����Ƿ�ʹ��TIMER��ĳһ���ж�
  TIMER_Capture_Clear(TIMER_A1, TIMER_CH1);
  EnableInterrupts();
  
  while(1)
  {  
    DELAY_MS(100);
    Remote_Scan();
  }
}





