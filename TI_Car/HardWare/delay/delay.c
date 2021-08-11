#include "delay.h"
#include "msp430f5529_clock.h"
/*******************************************************************************
*  �������ƣ�
*  ����˵����US��λ��ʱ����ʹ�ö�ʱ���Ļ���ȽϿɿ�����delay.h�к궨�嶨ʱ��ģ�飬��û�к궨�壬��ʹ������ģ�⣬ע�������жϻ���ܴ�
*  ����˵����
*  �������أ�
*  ʹ��ʾ����
********************************************************************************/
void delay_us(uint32_t us)
{
  us = (uint32_t)(us*(CPU_FRE_MHZ/11.6));
  while(us--);//asm("nop");
}