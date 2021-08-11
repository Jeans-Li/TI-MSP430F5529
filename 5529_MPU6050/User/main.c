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
extern float Angle_X_Final_Kalman,Angle_Y_Final_Kalman;
void main()
{
  DisableInterrupts();            //��ֹ���ж�
  
  CLOCK_DCO_PLLConfig(FLLREF_REFO,FLLREF_DIV_1,25MHZ);   //DCO_FLL��Ƶ������ ������Ƶ������

  MPU6050_INIT();
  LED_Init(LED_ALL);              //LED�Ƴ�ʼ��
  OLED_Init();                       //OLED��ʼ��
   
  
  //OLED��ʾ�ַ����ַ�����������С����һ�������͹���
  OLED_PrintfAt(FONT_ASCII_6X8,0,0,"A");//��ʾһ���ַ�

  
  EnableInterrupts();
  while(1)
  {  
   MPU6050_Tim();

   OLED_DispFolatAt(FONT_ASCII_6X8,3,50,Angle_X_Final_Kalman,2);
   OLED_DispFolatAt(FONT_ASCII_6X8,5,50,Angle_Y_Final_Kalman,2);
  }
}