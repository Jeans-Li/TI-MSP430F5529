#include "app_main.h"



char databuf[256];
static int num=0;
uint8_t flag = 0;


__interrupt void UART_RX_IRQ_Handler()
{ 
  if(UART_GetITStatus(UART1,UART_RX_IRQn) == TRUE)   //�������ĳһ���жϱ�־
  {
    //�������ʸ�ʱ�������ж��ﲻҪ�������¼�������ᶪʧ���ݣ�������ʱ��ҲҪ���һ��
    databuf[num]=UART_GetChar     (UART1);                 //��ȡһ���ֽ�1���ֽ�
    num++;
    UART_ClearITPendingBit(UART1,UART_RX_IRQn);    //�������ĳһ���жϱ�־
  }
}


void app_main_init()
{
  DisableInterrupts();          //��ֹ���ж�
  DisableWatchDog();     //�رտ��Ź�
  LED_Init(LED_ALL);            //LED��ʼ��
  MOTOR_Init();  //���pwm��ʼ�����������io�ڳ�ʼ��
  UART_Init        (UART1,115200);                      //��ʼ��UART1ģ��,������115200�������ʸ�ʱ��ð���ʱ�� ���Щ
  Set_Vector_Handler(VECTOR_UART1,UART_RX_IRQ_Handler);    //���ý����ж�����
  UART_ITConfig  (UART1,UART_RX_IRQn,TRUE);                //�����ڽ����ж�
  Encoder_Init();   //��������ʼ��
  OLED_Init();   //OLED��ʼ��
  OLED_Clear();
  Car_Go();
  EnableInterrupts();     //�������ж�
}


void app_main()
{
  display();
  speed_Handle();    //�ٶȴ�����
//  Remote_Scan();
//    LED_Turn(LED2);

    
//    UART_Printf(UART1," Hello-CTGU!\n UART-TEST!\n");         //�����ַ���
//    UART_Printf(UART1,"������1��%d\r\n",encoderCnt_A);
//    UART_Printf(UART1,"������2��%d\r\n",encoderCnt_B);
//    UART_Printf(UART1,"������3��%d\r\n",encoderCnt_C);
//    UART_Printf(UART1,"������4��%d\r\n",encoderCnt_D);
}

void display()
{
        //��ʾ��ʾ��
        OLED_DispStringAt(FONT_ASCII_6X8,0,0,"Speed:");  //��ʾ��ʾ�֣�Speed
        OLED_DispStringAt(FONT_ASCII_6X8,0,90,"m/min");
        OLED_DispStringAt(FONT_ASCII_6X8,2,0,"A:         B:     ");
        OLED_DispStringAt(FONT_ASCII_6X8,4,0,"C:         D:     ");
        OLED_DispStringAt(FONT_ASCII_6X8,6,0,"P:   I:   D:");
        
        //��ʾ����ת��
        OLED_DispDecAt(FONT_ASCII_6X8,2,15,RPM_A,3);
        OLED_DispDecAt(FONT_ASCII_6X8,2,80,RPM_B,3);
        OLED_DispDecAt(FONT_ASCII_6X8,4,15,RPM_C,3);
        OLED_DispDecAt(FONT_ASCII_6X8,4,80,RPM_D,3);

        //��ʾ�ٶ�
        OLED_DispFolatAt(FONT_ASCII_6X8,0,50,speed,2);

//	ssd1306_SetCursor(20, 15);
//	ssd1306_WriteString(showRPM(1), Font_7x10,White);     //A��ʵ�ʼ��������ת��  ��λRPM
//
//	ssd1306_SetCursor(80, 15);
//	ssd1306_WriteString(showRPM(2), Font_7x10,White);     //B��ʵ�ʼ��������ת��  ��λRPM
//
//	ssd1306_SetCursor(20, 30);
//	ssd1306_WriteString(showRPM(3), Font_7x10,White);     //C��ʵ�ʼ��������ת��  ��λRPM
//
//	ssd1306_SetCursor(80, 30);
//	ssd1306_WriteString(showRPM(4), Font_7x10,White);    //D��ʵ�ʼ��������ת��  ��λRPM
//
//	ssd1306_UpdateScreen();
//	ssd1306_Fill(Black);	//����
}