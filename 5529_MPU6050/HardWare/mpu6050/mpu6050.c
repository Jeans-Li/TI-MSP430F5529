#include <mpu6050.h>
#include <include.h>
//******************************************************************************
//! ��������                 IIC_Start
//! ���ܣ�                     IIC��ʼ�ź�
//! ���ߣ�                      ˧˧
//******************************************************************************
void IIC_Start(){
    SDA_OUT;
    SDA_H;                    //����������
    SCL_OUT;
    SCL_H;                    //����ʱ����
    DELAY_US(2);                 //��ʱ
    SDA_L;                    //�����½���
    DELAY_US(2);                  //��ʱ
    SCL_L;                    //����ʱ����
}
//******************************************************************************
//! ��������                 IIC_Stop
//! ���ܣ�                     IICֹͣ�ź�
//! ���ߣ�                      ˧˧
//******************************************************************************
void IIC_Stop(){
    SDA_OUT;
    SDA_L;                    //����������
    SCL_OUT;
    SCL_H;                    //����ʱ����
    DELAY_US(2);                   //��ʱ
    SDA_H;                    //����������
    DELAY_US(2);                  //��ʱ
}
//******************************************************************************
//! ��������                 IIC_SendACK
//! ���ܣ�                     IIC����Ӧ���ź�
//! ���ߣ�                      ˧˧
//******************************************************************************
void IIC_SendACK(uint8_t ack){
    SDA_OUT;
    SCL_OUT;
    if(ack) SDA_H;
    else SDA_L;
//    SDA = ack;                  //дӦ���ź�
    SCL_H;                    //����ʱ����
    DELAY_US(2);                  //��ʱ
    SCL_L;                    //����ʱ����
    DELAY_US(2);                  //��ʱ
}
//******************************************************************************
//! ��������                 IIC_RecvACK
//! ���ܣ�                     IIC����Ӧ���ź�
//! ���ߣ�                      ˧˧
//******************************************************************************
uint8_t IIC_RecvACK(){
    uint8_t cy;
    SCL_OUT;
    SCL_H;                    //����ʱ����
    SDA_IN;
    DELAY_US(2);                 //��ʱ
    if(SDA_DATA)    cy=1;
    else            cy=0;
//    cy = SDA;                   //��Ӧ���ź�
    SCL_L;                    //����ʱ����
    DELAY_US(2);                //��ʱ
    SDA_OUT;
    return cy;
}
//******************************************************************************
//! ��������                 IIC_SendByte
//! ���ܣ�                     ��IIC���߷���һ���ֽ�����
//! ���ߣ�                      ˧˧
//******************************************************************************
void IIC_SendByte(uint8_t dat){
    uint8_t i;
    SDA_OUT;
    for (i=0; i<8; i++){          //8λ������
        if((dat<<i)&0x80)   SDA_H;
        else                SDA_L;
       // SDA = cy;               //�����ݿ�
        SCL_OUT;
        SCL_H;                //����ʱ����
        DELAY_US(2);              //��ʱ
        SCL_L;                //����ʱ����
        DELAY_US(2);              //��ʱ
    }
    IIC_RecvACK();
}

//******************************************************************************
//! ��������                 IIC_RecvByte
//! ���ܣ�                     ��IIC���߽���һ���ֽ�����
//! ���ߣ�                      ˧˧
//******************************************************************************
uint8_t IIC_RecvByte(){
    uint8_t i;
    uint8_t dat = 0,cy;
    SDA_OUT;
    SCL_OUT;
    SDA_H;                    //ʹ���ڲ�����,׼����ȡ����,
    SDA_IN;
    for (i=0; i<8; i++){         //8λ������
        dat<<= 1;
        SCL_H;                //����ʱ����
        DELAY_US(2);             //��ʱ
        if(SDA_DATA)    cy=1;
        else            cy=0;
        dat |= cy;             //������
        SCL_L;                //����ʱ����
        DELAY_US(2);             //��ʱ
    }
    SDA_OUT;
    return dat;
}
//******************************************************************************
//! ��������                 Single_WriteIIC
//! ���ܣ�                     ��IIC�豸д��һ���ֽ�����
//! ���ߣ�                      ˧˧
//******************************************************************************
void Single_WriteIIC(uint8_t REG_Address,uint8_t REG_data)
{
    IIC_Start();                  //��ʼ�ź�
    IIC_SendByte(SlaveAddress);   //�����豸��ַ+д�ź�
    IIC_SendByte(REG_Address);    //�ڲ��Ĵ�����ַ��
    IIC_SendByte(REG_data);       //�ڲ��Ĵ������ݪ�
    IIC_Stop();                   //����ֹͣ�ź�
}

//******************************************************************************
//! ��������                 Single_ReadIIC
//! ���ܣ�                    ��IIC�豸��ȡһ���ֽ�����
//! ���ߣ�                      ˧˧
//******************************************************************************
uint8_t  Single_ReadIIC(uint8_t REG_Address)
{
 uint8_t REG_data;
 IIC_Start();                   //��ʼ�ź�
 IIC_SendByte(SlaveAddress);    //�����豸��ַ+д�ź�
 IIC_SendByte(REG_Address);     //���ʹ洢��Ԫ��ַ����0��ʼ
 IIC_Start();                   //��ʼ�ź�
 IIC_SendByte(SlaveAddress+1);  //�����豸��ַ+���ź�
 REG_data=IIC_RecvByte();       //�����Ĵ�������
 IIC_SendACK(1);                //����Ӧ���ź�
 IIC_Stop();                    //ֹͣ�ź�
 return REG_data;
}
//******************************************************************************
//! ��������                 MPU6050_INIT
//! ���ܣ�                     ��ʼ��MPU6050
//! ���ߣ�                      ˧˧
//******************************************************************************
void MPU6050_INIT(){
    Single_WriteIIC(PWR_MGMT_1, 0x00); //�������״̬
    Single_WriteIIC(SMPLRT_DIV, 0x07);
    Single_WriteIIC(CONFIG, 0x06);
    Single_WriteIIC(GYRO_CONFIG, 0x18);
    Single_WriteIIC(ACCEL_CONFIG, 0x01);
}
//******************************************************************************
//! ��������                 MPU6050_GET
//! ���ܣ�                     ��ȡMPU6050
//! ��ڣ�                     ѡ��
//! ���ߣ�                     ˧˧
//******************************************************************************
int MPU6050_GET(uint8_t REG_Address){
    char H,L;
    H = Single_ReadIIC(REG_Address);
    L = Single_ReadIIC(REG_Address+1);
    return ((H<<8)+L)/64;
}

//����������
float Q_angle = 0.05; //�ϴ�Ϊ0.05
float Q_gyro  = 0.05;
float R_angle = 0.06;
float dt      = 0.01;//dtΪkalman�˲�������ʱ��

char  C_0     = 1;
float Q_bias, Angle_err;
float PCt_0, PCt_1, E;
float K_0, K_1, t_0, t_1;
float Pdot[4] ={0,0,0,0};
float PP[2][2] = { { 1, 0 },{ 0, 1 } };
float ax,ay,az,Gyro_x,Gyro_y,Gyro_z,Angle_x_temp,Angle_y_temp;
float Angle_X_Final_Kalman,Angle_Y_Final_Kalman;
float Angle_Y_Final_Kalman_last;
//�������˲�����
void Kalman_Filter_X(float Accel,float Gyro) //����������
{

    Angle_X_Final_Kalman += (Gyro - Q_bias) * dt; //�������

    Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-����������Э�����΢��

    Pdot[1]= -PP[1][1];
    Pdot[2]= -PP[1][1];
    Pdot[3]= Q_gyro;


    PP[0][0] += Pdot[0] * dt;   // Pk-����������Э����΢�ֵĻ���
    PP[0][1] += Pdot[1] * dt;   // =����������Э����
    PP[1][0] += Pdot[2] * dt;
    PP[1][1] += Pdot[3] * dt;


    Angle_err = Accel - Angle_X_Final_Kalman;  //-�������

    PCt_0 = C_0 * PP[0][0];
    PCt_1 = C_0 * PP[1][0];

    E = R_angle + C_0 * PCt_0;

    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;

    t_0 = PCt_0;
    t_1 = C_0 * PP[0][1];

    PP[0][0] -= K_0 * t_0;       //����������Э����
    PP[0][1] -= K_0 * t_1;
    PP[1][0] -= K_1 * t_0;
    PP[1][1] -= K_1 * t_1;

    Angle_X_Final_Kalman += K_0 * Angle_err;    //�������
    Q_bias += K_1 * Angle_err;    //�������
}

void Kalman_Filter_Y(float Accel,float Gyro) //����������
{
    Angle_Y_Final_Kalman += (Gyro - Q_bias) * dt; //�������

    Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-����������Э�����΢��

    Pdot[1]=- PP[1][1];
    Pdot[2]=- PP[1][1];
    Pdot[3]=Q_gyro;

    PP[0][0] += Pdot[0] * dt;   // Pk-����������Э����΢�ֵĻ���
    PP[0][1] += Pdot[1] * dt;   // =����������Э����
    PP[1][0] += Pdot[2] * dt;
    PP[1][1] += Pdot[3] * dt;

    Angle_err=Accel-Angle_Y_Final_Kalman;  //-�������

    PCt_0 = C_0 * PP[0][0];
    PCt_1 = C_0 * PP[1][0];

    E = R_angle + C_0 * PCt_0;

    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;

    t_0 = PCt_0;
    t_1 = C_0 * PP[0][1];

    PP[0][0] -= K_0 * t_0;       //����������Э����
    PP[0][1] -= K_0 * t_1;
    PP[1][0] -= K_1 * t_0;
    PP[1][1] -= K_1 * t_1;

    Angle_Y_Final_Kalman   += K_0 * Angle_err;  //�������
    Q_bias  += K_1 * Angle_err;  //�������
}
///******************
// * @brief ���������ݲɼ�
// * @param �ޣ�
// * @see MPU6050_Tim();
// *
// * */
//
void MPU6050_Tim(void)
{
//    char txt[16];
    short aacx,aacy,aacz;    //���ٶȴ�����ԭʼ����
    short gyrox,gyroy,gyroz; //������ԭʼ����

//    MPU_Get_Raw_data(&aacx,&aacy,&aacz,&gyrox,&gyroy,&gyroz);   //�õ����ٶȴ���������
    aacx = MPU6050_GET(ACCEL_XOUT_H);
    aacy = MPU6050_GET(ACCEL_YOUT_H);
    aacz = MPU6050_GET(ACCEL_ZOUT_H);
    gyrox = MPU6050_GET(GYRO_XOUT_H);
    gyroy = MPU6050_GET(GYRO_YOUT_H);
    gyroz = MPU6050_GET(GYRO_ZOUT_H);

    ax = (9.8*aacx)/8192;//��̬����
    ay = (9.8*aacy)/8192;
    az = (9.8*aacz)/8192;

    Gyro_x = (gyrox)/16.4;
    Gyro_y = (gyroy)/16.4;
    Gyro_z = (gyroz)/16.4;


    //�ü��ٶȼ����������ˮƽ������ϵ֮��ļн�
//    Angle_x_temp=(atan((float)(ay)/(float)(az)))*180/3.14;   //����x��н�
    Angle_y_temp=(atan((float)(ax)/(float)(az)))*180/3.14;   //����y��н�

    Angle_x_temp = (atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * 180 / 3.14);
    Kalman_Filter_X(Angle_x_temp,Gyro_z);
    Kalman_Filter_Y(Angle_y_temp,Gyro_z);
//    sprintf(txt,"Angle_X:%f", Angle_X_Final_Kalman);
//    TFTSPI_P6X8Str(1,12,txt,u16YELLOW, u16BLUE);
//    OLED_DispDecAt(FONT_ASCII_6X8,3,50,Angle_X_Final_Kalman,3);          //��ʾ���յ������ݼ��ٶ�Y
//    OLED_DispDecAt(FONT_ASCII_6X8,5,50,Angle_Y_Final_Kalman,3);          //��ʾ���յ������ݼ��ٶ�Z
}
