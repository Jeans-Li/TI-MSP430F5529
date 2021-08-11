//#include "remote.h"
//
//uint32_t upCount = 0;   //���ϼ���
//uint16_t valueUp = 0;   //�ߵ�ƽ����
//uint16_t valueDown = 0;   //�͵�ƽ����
//uint8_t isUpCompare = 1;      //�Ƿ�Ϊ�����ز�����
//uint16_t width;           //����
//uint16_t buffer[128] = {0};    //���ջ�����
//uint16_t bufferId = 0;        //���ջ����������±�
//uint8_t recFlag = 0;    //������ɱ�־λ
//char IRCodes[4] = {0,0,0,0};
//
//__interrupt void TIMER2_A0_IRQ_Handler()
//{
//  if(TIMER_GetITStatus(TIMER_A2,TIMER_OverFlow_IRQn) == TRUE)//������
//   {
//     upCount++;          //���ֵ��1
////     TIMER_ClearITPendingBit(TIMER_A2,TIMER_OverFlow_IRQn);     //���TIMER��ĳһ���жϱ�־
//   }
//  //����������ز���
//  if(isUpCompare)
//  {
//    valueUp = TIMER_Capture_GetValue  (TIMER_A2,TIMER_CH0);
//    isUpCompare = 0;
//    TIMER_Capture_SetMode   (TIMER_A2, TIMER_CH0, CAP_Falling);    //����Ϊ�½��ز���
//    upCount = 0;
//  }
//  else
//  {
//      valueDown = TIMER_Capture_GetValue  (TIMER_A2,TIMER_CH0);
//      isUpCompare = 1;
//      TIMER_Capture_SetMode   (TIMER_A2, TIMER_CH0, CAP_Rising);    //����Ϊ�����ز���
//      width = valueDown + upCount*65536 - valueUp;
//      if(width>4400 && width<4600)
//      {
//          bufferId = 0;
//          buffer[bufferId++] = width;
//      }
//      else if(bufferId > 0)
//      {
//          buffer[bufferId++] = width;
//          if(bufferId > 32)   //������һ��32λ�Ѿ�ȫ���յ�
//          {
//            recFlag = 1;
//            bufferId = 0;
//          }      
//      }
//  }
//  for(int i =0;i < 32; i++)
//  {
//      if(buffer[i+1]<1000)
//      {
//              IRCodes[i/8] = IRCodes[i/8]<<1;
//      }
//      else
//      {
//              IRCodes[i/8] = IRCodes[i/8]<<1;
//              IRCodes[i/8] |= 0x01;
//      }
//  }
//}
//
//void Remote_Init()
//{
//  Set_Vector_Handler(VECTOR_TIMER2_A0,TIMER2_A0_IRQ_Handler);    //���ý����ж�����
//  TIMER_Capture_Init  (TIMER_A2, TIMER_CH0, CAP_Rising );        //��ʼ��һ��ͨ��Ϊ����ģʽ�������ز���
//  TIMER_ITConfig   (TIMER_A2, TIMER_CCR0_IRQn,ENABLE);               //�����Ƿ�ʹ��TIMER��ĳһ���ж�
//  TIMER_ITConfig   (TIMER_A2, TIMER_OverFlow_IRQn,ENABLE);               //�����Ƿ�ʹ��TIMER������ж�
//}
//
//uint8_t Remote_Scan()
//{
//	if(recFlag)
//	{
//
//		recFlag = 0;    //��ֹif����ظ�����
//		for(int i =0;i<4 ;i++)
//		{
//                  UART_Printf(UART1,"%d  ",IRCodes[i]);
//		}
//    }
//
//	return IRCodes[2];
//}