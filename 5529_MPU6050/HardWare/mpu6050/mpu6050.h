#ifndef __MPU6050_H_
#define __MPU6050_H_

#include <include.h>

#define SCL_H P3OUT |=BIT1
#define SCL_L P3OUT &=~BIT1
#define SCL_OUT P3DIR |= BIT1;
#define SDA_H P3OUT |=BIT0   //IIC��������
#define SDA_L P3OUT &=~BIT0
#define SDA_IN P3DIR &=~BIT0
#define SDA_OUT P3DIR |=BIT0
#define SDA_DATA (P3IN & BIT0)
//****************************************
// ����MPU6050�ڲ���ַ
//****************************************
#define SMPLRT_DIV  0x19 //�����ǲ����ʪ�����ֵ��0x07(125Hz)
#define CONFIG   0x1A //��ͨ�˲�Ƶ�ʪ�����ֵ��0x06(5Hz)
#define GYRO_CONFIG  0x1B //�������Լ켰������Χ������ֵ��0x18(���Լ쪢2000deg/s)
#define ACCEL_CONFIG 0x1C //���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʪ�����ֵ��0x01(���Լ쪢2G��5Hz)

#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

#define TEMP_OUT_H  0x41
#define TEMP_OUT_L  0x42

#define GYRO_XOUT_H  0x43
#define GYRO_XOUT_L  0x44
#define GYRO_YOUT_H  0x45
#define GYRO_YOUT_L  0x46
#define GYRO_ZOUT_H  0x47
#define GYRO_ZOUT_L  0x48

#define PWR_MGMT_1  0x6B //��Դ��������ֵ��0x00(��������)
#define WHO_AM_I   0x75 //IIC��ַ�Ĵ���(Ĭ����ֵ0x68��ֻ��)
#define SlaveAddress 0xD0 //IICд��ʱ�ĵ�ַ�ֽ����ݪ�+1Ϊ��ȡ

void IIC_Start();
void IIC_Stop();
void IIC_SendACK(uint8_t ack);
uint8_t IIC_RecvACK();
void IIC_SendByte(uint8_t dat);
uint8_t IIC_RecvByte();
void Single_WriteIIC(uint8_t REG_Address,uint8_t REG_data);
uint8_t  Single_ReadIIC(uint8_t REG_Address);
void MPU6050_Tim(void);
void MPU6050_INIT();
int MPU6050_GET(uint8_t REG_Address);
//void Kalman_Filter_Y(float Accel,float Gyro); //����������
//void Kalman_Filter_X(float Accel,float Gyro) //����������

#endif
