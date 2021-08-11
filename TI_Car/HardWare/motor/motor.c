/*
 *
 *
            �����ķ�ְ�װ����

             TA0.1(P1.2)  A�� \\   ------   //  B��     TA0.2(P1.3)
                             \\   ------  //
                                  ------
                                  ------
                                  ------
                              //  ------  \\
              TA0.3(P1.4) D��  //   ------   \\  C��     TA0.4(P1.5)

                A�� //��ǰ
                B�� //��ǰ
                C�� //�Һ�
                D�� //���


            A�֣�����ǰ���˶�   P6.0 --- AIN1��P6.1 --- AIN2
            B�֣�����ǰ���˶�   P6.2 --- BIN1��P6.3 --- BIN2

            C�֣�����ǰ���˶�   P6.4 --- BIN1��P7.0 --- BIN1
            D�֣�����ǰ���˶�   P8.1 --- AIN1��P3.6 --- AIN2
*/


#include "motor.h"
void MOTOR_Init()
{
  //��ʱ��A0��Ƶ��1K,ռ�ձ�Ϊ0,��ʼ��4��ͨ�����ֱ��ǣ�TIMER_CH1(P1.2), TIMER_CH2(P1.3), TIMER_CH3(P1.4), TIMER_CH4(P1.5)
  TIMER_PWM_Init(TIMER_A0,10000, TIMER_CH1, TIMER_CH2, TIMER_CH3, TIMER_CH4);
  TIMER_PWM_SetChannelDuty  (TIMER_A0,TIMER_CH1,500);       //����ĳһ����ʱ��ĳһ��ͨ����ռ�ձ�
  TIMER_PWM_SetChannelDuty  (TIMER_A0,TIMER_CH2,500);     //����ĳһ����ʱ��ĳһ��ͨ����ռ�ձ�
  TIMER_PWM_SetChannelDuty  (TIMER_A0,TIMER_CH3,500);     //����ĳһ����ʱ��ĳһ��ͨ����ռ�ձ�
  TIMER_PWM_SetChannelDuty  (TIMER_A0,TIMER_CH4,500);    //����ĳһ����ʱ��ĳһ��ͨ����ռ�ձ�
  
  GPIO_MultiBits_Init(P6,BIT0,GPO|HDS); 
  GPIO_MultiBits_Init(P6,BIT1,GPO|HDS);
  GPIO_MultiBits_Init(P6,BIT2,GPO|HDS);
  GPIO_MultiBits_Init(P6,BIT3,GPO|HDS);
  GPIO_MultiBits_Init(P6,BIT4,GPO|HDS);
  GPIO_MultiBits_Init(P7,BIT0,GPO|HDS);
  GPIO_MultiBits_Init(P8,BIT1,GPO|HDS);
  GPIO_MultiBits_Init(P3,BIT6,GPO|HDS);
  
  
}

//�ı��ĸ����pwmռ�ձ�
void MOTOR_PWM_Out(int dutyCycle_A,int dutyCycle_B,int dutyCycle_C,int dutyCycle_D)
{
    TIMER_PWM_SetChannelDuty(TIMER_A0,TIMER_CH1,dutyCycle_A);
    TIMER_PWM_SetChannelDuty(TIMER_A0,TIMER_CH2,dutyCycle_B);
    TIMER_PWM_SetChannelDuty(TIMER_A0,TIMER_CH3,dutyCycle_C);
    TIMER_PWM_SetChannelDuty(TIMER_A0,TIMER_CH4,dutyCycle_D);
}

void MOTOR_A(char state)
{
    if(state == GO)
    {
        GPIO_SetBits(PORT6, GPIO_Pin_0);
        GPIO_ResetBits(PORT6, GPIO_Pin_1);
    }
    if(state == BACK)
    {
        GPIO_SetBits(PORT6, GPIO_Pin_1);
        GPIO_ResetBits(PORT6, GPIO_Pin_0);
    }
    if(state == STOP)
    {
        GPIO_SetBits(PORT6, GPIO_Pin_0);
        GPIO_SetBits(PORT6, GPIO_Pin_1);
    }
}

void MOTOR_B(char state)
{
    if(state == GO)
    {
        GPIO_SetBits(PORT6, GPIO_Pin_2);
        GPIO_ResetBits(PORT6, GPIO_Pin_3);
    }
    if(state == BACK)
    {
        GPIO_SetBits(PORT6, GPIO_Pin_3);
        GPIO_ResetBits(PORT6, GPIO_Pin_2);
    }
    if(state == STOP)
    {
        GPIO_SetBits(PORT6, GPIO_Pin_2);
        GPIO_SetBits(PORT6, GPIO_Pin_3);
    }
}

void MOTOR_C(char state)
{
    if(state == GO)
    {
        GPIO_SetBits(PORT6, GPIO_Pin_4);
        GPIO_ResetBits(PORT7, GPIO_Pin_0);
    }
    if(state == BACK)
    {
        GPIO_SetBits(PORT7, GPIO_Pin_0);
        GPIO_ResetBits(PORT6, GPIO_Pin_4);
    }
    if(state == STOP)
    {
        GPIO_SetBits(PORT6, GPIO_Pin_4);
        GPIO_SetBits(PORT7, GPIO_Pin_0);
    }
}

void MOTOR_D(char state)
{
    if(state == GO)
    {
        GPIO_SetBits(PORT8, GPIO_Pin_1);
        GPIO_ResetBits(PORT3, GPIO_Pin_6);
    }
    if(state == BACK)
    {
        GPIO_SetBits(PORT3, GPIO_Pin_6);
        GPIO_ResetBits(PORT8, GPIO_Pin_1);
    }
    if(state == STOP)
    {
        GPIO_SetBits(PORT8, GPIO_Pin_1);
        GPIO_SetBits(PORT3, GPIO_Pin_6);
    }
}

//***************************ǰ��***************************//
//ֻҪ����INx()��״̬�Ϳ��Ըı���ת������
void Car_Go(void)
{
    //��ǰ��� ǰ    //��ǰ��� ǰ
    MOTOR_A(GO);       MOTOR_B(GO);

    //����� ǰ   //�Һ��� ǰ
    MOTOR_D(GO);       MOTOR_C(GO);


}

void Car_Back(void)
{
        //��ǰ��� ��    //��ǰ��� ��
MOTOR_A(BACK);       MOTOR_B(BACK);

    //����� ��   //�Һ��� ��
MOTOR_D(BACK);       MOTOR_C(BACK);
}

//***************************ԭ����תȦ***************************//
void Car_Turn_Left_Around(void)
{

    //��ǰ��� ��    //��ǰ��� ǰ
MOTOR_A(BACK);       MOTOR_B(GO);

    //����� ��   //�Һ��� ǰ
MOTOR_D(BACK);       MOTOR_C(GO);

}
//***************************ԭ����תȦ***************************//
void Car_Turn_Right_Around(void)
{

    //��ǰ��� ǰ    //��ǰ��� ��
MOTOR_A(GO);       MOTOR_B(BACK);

    //����� ǰ   //�Һ��� ��
MOTOR_D(GO);       MOTOR_C(BACK);

}


//***************************�Һ��˶�***************************//
void Car_Right(void)
{
    //��ǰ���     //��ǰ���
MOTOR_A(GO);       MOTOR_B(BACK);

    //�����    //�Һ���
  MOTOR_D(BACK);       MOTOR_C(GO);

}
//***************************����˶�***************************//
void Car_Left(void)
{

    //��ǰ���     //��ǰ���
MOTOR_A(BACK);       MOTOR_B(GO);

    //�����    //�Һ���
MOTOR_D(GO);       MOTOR_C(BACK);
}
//***************************��ǰ45���˶�***************************//
void Car_Right_45Q(void)
{
    //��ǰ���     //��ǰ���
MOTOR_A(GO);       MOTOR_B(STOP);

    //�����    //�Һ���
  MOTOR_D(STOP);       MOTOR_C(GO);

}
//***************************�Һ�45���˶�***************************//
void Car_Left_45H(void)
{
    //��ǰ���     //��ǰ���
MOTOR_A(BACK);       MOTOR_B(STOP);

    //�����    //�Һ���
  MOTOR_D(STOP);       MOTOR_C(BACK);

}
//***************************��ǰ45�˶�***************************//
void Car_Left_45Q(void)
{

    //��ǰ���     //��ǰ���
MOTOR_A(STOP);       MOTOR_B(GO);

    //�����    //�Һ���
MOTOR_D(GO);       MOTOR_C(STOP);

}
//***************************���45�˶�***************************//
void Car_Right_45H(void)
{

    //��ǰ���     //��ǰ���
MOTOR_A(STOP);       MOTOR_B(BACK);

    //�����    //�Һ���
MOTOR_D(BACK);       MOTOR_C(STOP);

}
//***************************ͣ��***************************//
void Car_Stop(void)
{
    //��ǰ��� ͣ    //��ǰ��� ͣ
MOTOR_A(STOP);       MOTOR_B(STOP);

    //����� ͣ   //�Һ��� ͣ
  MOTOR_D(STOP);       MOTOR_C(STOP);
}