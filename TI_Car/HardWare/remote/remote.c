//#include "remote.h"
//
//uint32_t upCount = 0;   //向上计数
//uint16_t valueUp = 0;   //高电平计数
//uint16_t valueDown = 0;   //低电平计数
//uint8_t isUpCompare = 1;      //是否为上升沿捕获标记
//uint16_t width;           //脉宽
//uint16_t buffer[128] = {0};    //接收缓冲区
//uint16_t bufferId = 0;        //接收缓冲区数组下标
//uint8_t recFlag = 0;    //接收完成标志位
//char IRCodes[4] = {0,0,0,0};
//
//__interrupt void TIMER2_A0_IRQ_Handler()
//{
//  if(TIMER_GetITStatus(TIMER_A2,TIMER_OverFlow_IRQn) == TRUE)//如果溢出
//   {
//     upCount++;          //溢出值加1
////     TIMER_ClearITPendingBit(TIMER_A2,TIMER_OverFlow_IRQn);     //清除TIMER的某一个中断标志
//   }
//  //如果是上升沿捕获
//  if(isUpCompare)
//  {
//    valueUp = TIMER_Capture_GetValue  (TIMER_A2,TIMER_CH0);
//    isUpCompare = 0;
//    TIMER_Capture_SetMode   (TIMER_A2, TIMER_CH0, CAP_Falling);    //设置为下降沿捕获
//    upCount = 0;
//  }
//  else
//  {
//      valueDown = TIMER_Capture_GetValue  (TIMER_A2,TIMER_CH0);
//      isUpCompare = 1;
//      TIMER_Capture_SetMode   (TIMER_A2, TIMER_CH0, CAP_Rising);    //设置为上升沿捕获
//      width = valueDown + upCount*65536 - valueUp;
//      if(width>4400 && width<4600)
//      {
//          bufferId = 0;
//          buffer[bufferId++] = width;
//      }
//      else if(bufferId > 0)
//      {
//          buffer[bufferId++] = width;
//          if(bufferId > 32)   //所有码一共32位已经全部收到
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
//  Set_Vector_Handler(VECTOR_TIMER2_A0,TIMER2_A0_IRQ_Handler);    //设置接收中断向量
//  TIMER_Capture_Init  (TIMER_A2, TIMER_CH0, CAP_Rising );        //初始化一个通道为捕获模式，上升沿捕获
//  TIMER_ITConfig   (TIMER_A2, TIMER_CCR0_IRQn,ENABLE);               //设置是否使能TIMER的某一个中断
//  TIMER_ITConfig   (TIMER_A2, TIMER_OverFlow_IRQn,ENABLE);               //设置是否使能TIMER的溢出中断
//}
//
//uint8_t Remote_Scan()
//{
//	if(recFlag)
//	{
//
//		recFlag = 0;    //防止if语句重复成立
//		for(int i =0;i<4 ;i++)
//		{
//                  UART_Printf(UART1,"%d  ",IRCodes[i]);
//		}
//    }
//
//	return IRCodes[2];
//}