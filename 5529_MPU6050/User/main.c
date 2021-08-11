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
extern float Angle_X_Final_Kalman,Angle_Y_Final_Kalman;
void main()
{
  DisableInterrupts();            //禁止总中断
  
  CLOCK_DCO_PLLConfig(FLLREF_REFO,FLLREF_DIV_1,25MHZ);   //DCO_FLL倍频环设置 陀螺仪频率设置

  MPU6050_INIT();
  LED_Init(LED_ALL);              //LED灯初始化
  OLED_Init();                       //OLED初始化
   
  
  //OLED显示字符，字符串，整数，小数，一个函数就够了
  OLED_PrintfAt(FONT_ASCII_6X8,0,0,"A");//显示一个字符

  
  EnableInterrupts();
  while(1)
  {  
   MPU6050_Tim();

   OLED_DispFolatAt(FONT_ASCII_6X8,3,50,Angle_X_Final_Kalman,2);
   OLED_DispFolatAt(FONT_ASCII_6X8,5,50,Angle_Y_Final_Kalman,2);
  }
}