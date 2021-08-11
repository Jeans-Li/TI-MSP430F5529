
#include "include.h"
//定义电机状态
#define GO    0// 前进
#define BACK  1//后退
#define STOP  2//停车


void MOTOR_Init();

void MOTOR_PWM_Out(int dutyCycle_A,int dutyCycle_B,int dutyCycle_C,int dutyCycle_D);

void MOTOR_A(char state);
void MOTOR_B(char state);
void MOTOR_C(char state);
void MOTOR_D(char state);

void Car_Go(void);
void Car_Back(void);
void Car_Turn_Right_Around(void);
void Car_Turn_Left_Around(void);
void Car_Stop(void);
void Car_Left(void);
void Car_Right(void);

void Car_Right_45Q(void);
void Car_Right_45H(void);
void Car_Left_45Q(void);
void Car_Left_45H(void);