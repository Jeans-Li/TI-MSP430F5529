/*
 *
 *
            麦克纳姆轮安装方向

             TA0.1(P1.2)  A轮 \\   ------   //  B轮     TA0.2(P1.3)
                             \\   ------  //
                                  ------
                                  ------
                                  ------
                              //  ------  \\
              TA0.3(P1.4) D轮  //   ------   \\  C轮     TA0.4(P1.5)

                A轮 //左前
                B轮 //右前
                C轮 //右后
                D轮 //左后


            A轮：控制前后运动   P6.0 --- AIN1，P6.1 --- AIN2
            B轮：控制前后运动   P6.2 --- BIN1，P6.3 --- BIN2

            C轮：控制前后运动   P6.4 --- BIN1，P7.0 --- BIN1
            D轮：控制前后运动   P8.1 --- AIN1，P3.6 --- AIN2
*/


#include "motor.h"
void MOTOR_Init()
{
  //定时器A0，频率1K,占空比为0,初始化4个通道，分别是：TIMER_CH1(P1.2), TIMER_CH2(P1.3), TIMER_CH3(P1.4), TIMER_CH4(P1.5)
  TIMER_PWM_Init(TIMER_A0,10000, TIMER_CH1, TIMER_CH2, TIMER_CH3, TIMER_CH4);
  TIMER_PWM_SetChannelDuty  (TIMER_A0,TIMER_CH1,500);       //设置某一个定时器某一个通道的占空比
  TIMER_PWM_SetChannelDuty  (TIMER_A0,TIMER_CH2,500);     //设置某一个定时器某一个通道的占空比
  TIMER_PWM_SetChannelDuty  (TIMER_A0,TIMER_CH3,500);     //设置某一个定时器某一个通道的占空比
  TIMER_PWM_SetChannelDuty  (TIMER_A0,TIMER_CH4,500);    //设置某一个定时器某一个通道的占空比
  
  GPIO_MultiBits_Init(P6,BIT0,GPO|HDS); 
  GPIO_MultiBits_Init(P6,BIT1,GPO|HDS);
  GPIO_MultiBits_Init(P6,BIT2,GPO|HDS);
  GPIO_MultiBits_Init(P6,BIT3,GPO|HDS);
  GPIO_MultiBits_Init(P6,BIT4,GPO|HDS);
  GPIO_MultiBits_Init(P7,BIT0,GPO|HDS);
  GPIO_MultiBits_Init(P8,BIT1,GPO|HDS);
  GPIO_MultiBits_Init(P3,BIT6,GPO|HDS);
  
  
}

//改变四个电机pwm占空比
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

//***************************前进***************************//
//只要配置INx()的状态就可以改变电机转动方向
void Car_Go(void)
{
    //左前电机 前    //右前电机 前
    MOTOR_A(GO);       MOTOR_B(GO);

    //左后电机 前   //右后电机 前
    MOTOR_D(GO);       MOTOR_C(GO);


}

void Car_Back(void)
{
        //左前电机 后    //右前电机 后
MOTOR_A(BACK);       MOTOR_B(BACK);

    //左后电机 后   //右后电机 后
MOTOR_D(BACK);       MOTOR_C(BACK);
}

//***************************原地左转圈***************************//
void Car_Turn_Left_Around(void)
{

    //左前电机 后    //右前电机 前
MOTOR_A(BACK);       MOTOR_B(GO);

    //左后电机 后   //右后电机 前
MOTOR_D(BACK);       MOTOR_C(GO);

}
//***************************原地右转圈***************************//
void Car_Turn_Right_Around(void)
{

    //左前电机 前    //右前电机 后
MOTOR_A(GO);       MOTOR_B(BACK);

    //左后电机 前   //右后电机 后
MOTOR_D(GO);       MOTOR_C(BACK);

}


//***************************右横运动***************************//
void Car_Right(void)
{
    //左前电机     //右前电机
MOTOR_A(GO);       MOTOR_B(BACK);

    //左后电机    //右后电机
  MOTOR_D(BACK);       MOTOR_C(GO);

}
//***************************左横运动***************************//
void Car_Left(void)
{

    //左前电机     //右前电机
MOTOR_A(BACK);       MOTOR_B(GO);

    //左后电机    //右后电机
MOTOR_D(GO);       MOTOR_C(BACK);
}
//***************************右前45°运动***************************//
void Car_Right_45Q(void)
{
    //左前电机     //右前电机
MOTOR_A(GO);       MOTOR_B(STOP);

    //左后电机    //右后电机
  MOTOR_D(STOP);       MOTOR_C(GO);

}
//***************************右后45°运动***************************//
void Car_Left_45H(void)
{
    //左前电机     //右前电机
MOTOR_A(BACK);       MOTOR_B(STOP);

    //左后电机    //右后电机
  MOTOR_D(STOP);       MOTOR_C(BACK);

}
//***************************左前45运动***************************//
void Car_Left_45Q(void)
{

    //左前电机     //右前电机
MOTOR_A(STOP);       MOTOR_B(GO);

    //左后电机    //右后电机
MOTOR_D(GO);       MOTOR_C(STOP);

}
//***************************左后45运动***************************//
void Car_Right_45H(void)
{

    //左前电机     //右前电机
MOTOR_A(STOP);       MOTOR_B(BACK);

    //左后电机    //右后电机
MOTOR_D(BACK);       MOTOR_C(STOP);

}
//***************************停车***************************//
void Car_Stop(void)
{
    //左前电机 停    //右前电机 停
MOTOR_A(STOP);       MOTOR_B(STOP);

    //左后电机 停   //右后电机 停
  MOTOR_D(STOP);       MOTOR_C(STOP);
}