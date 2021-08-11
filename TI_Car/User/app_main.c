#include "app_main.h"



char databuf[256];
static int num=0;
uint8_t flag = 0;


__interrupt void UART_RX_IRQ_Handler()
{ 
  if(UART_GetITStatus(UART1,UART_RX_IRQn) == TRUE)   //清除串口某一个中断标志
  {
    //传输速率高时，接收中断里不要做其他事件，否则会丢失数据，另外主时钟也要设高一点
    databuf[num]=UART_GetChar     (UART1);                 //读取一个字节1个字节
    num++;
    UART_ClearITPendingBit(UART1,UART_RX_IRQn);    //清除串口某一个中断标志
  }
}


void app_main_init()
{
  DisableInterrupts();          //禁止总中断
  DisableWatchDog();     //关闭开门狗
  LED_Init(LED_ALL);            //LED初始化
  MOTOR_Init();  //电机pwm初始化、电机控制io口初始化
  UART_Init        (UART1,115200);                      //初始化UART1模块,波特率115200，波特率高时最好把主时钟 设高些
  Set_Vector_Handler(VECTOR_UART1,UART_RX_IRQ_Handler);    //设置接收中断向量
  UART_ITConfig  (UART1,UART_RX_IRQn,TRUE);                //开串口接收中断
  Encoder_Init();   //编码器初始化
  OLED_Init();   //OLED初始化
  OLED_Clear();
  Car_Go();
  EnableInterrupts();     //开启总中断
}


void app_main()
{
  display();
  speed_Handle();    //速度处理函数
//  Remote_Scan();
//    LED_Turn(LED2);

    
//    UART_Printf(UART1," Hello-CTGU!\n UART-TEST!\n");         //发送字符串
//    UART_Printf(UART1,"编码器1：%d\r\n",encoderCnt_A);
//    UART_Printf(UART1,"编码器2：%d\r\n",encoderCnt_B);
//    UART_Printf(UART1,"编码器3：%d\r\n",encoderCnt_C);
//    UART_Printf(UART1,"编码器4：%d\r\n",encoderCnt_D);
}

void display()
{
        //显示提示字
        OLED_DispStringAt(FONT_ASCII_6X8,0,0,"Speed:");  //显示提示字：Speed
        OLED_DispStringAt(FONT_ASCII_6X8,0,90,"m/min");
        OLED_DispStringAt(FONT_ASCII_6X8,2,0,"A:         B:     ");
        OLED_DispStringAt(FONT_ASCII_6X8,4,0,"C:         D:     ");
        OLED_DispStringAt(FONT_ASCII_6X8,6,0,"P:   I:   D:");
        
        //显示车轮转速
        OLED_DispDecAt(FONT_ASCII_6X8,2,15,RPM_A,3);
        OLED_DispDecAt(FONT_ASCII_6X8,2,80,RPM_B,3);
        OLED_DispDecAt(FONT_ASCII_6X8,4,15,RPM_C,3);
        OLED_DispDecAt(FONT_ASCII_6X8,4,80,RPM_D,3);

        //显示速度
        OLED_DispFolatAt(FONT_ASCII_6X8,0,50,speed,2);

//	ssd1306_SetCursor(20, 15);
//	ssd1306_WriteString(showRPM(1), Font_7x10,White);     //A轮实际减速箱输出转速  单位RPM
//
//	ssd1306_SetCursor(80, 15);
//	ssd1306_WriteString(showRPM(2), Font_7x10,White);     //B轮实际减速箱输出转速  单位RPM
//
//	ssd1306_SetCursor(20, 30);
//	ssd1306_WriteString(showRPM(3), Font_7x10,White);     //C轮实际减速箱输出转速  单位RPM
//
//	ssd1306_SetCursor(80, 30);
//	ssd1306_WriteString(showRPM(4), Font_7x10,White);    //D轮实际减速箱输出转速  单位RPM
//
//	ssd1306_UpdateScreen();
//	ssd1306_Fill(Black);	//清屏
}