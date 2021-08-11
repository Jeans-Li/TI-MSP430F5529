#include <mpu6050.h>
#include <include.h>
//******************************************************************************
//! 函数名：                 IIC_Start
//! 功能：                     IIC起始信号
//! 作者：                      帅帅
//******************************************************************************
void IIC_Start(){
    SDA_OUT;
    SDA_H;                    //拉高数据线
    SCL_OUT;
    SCL_H;                    //拉高时钟线
    DELAY_US(2);                 //延时
    SDA_L;                    //产生下降沿
    DELAY_US(2);                  //延时
    SCL_L;                    //拉低时钟线
}
//******************************************************************************
//! 函数名：                 IIC_Stop
//! 功能：                     IIC停止信号
//! 作者：                      帅帅
//******************************************************************************
void IIC_Stop(){
    SDA_OUT;
    SDA_L;                    //拉低数据线
    SCL_OUT;
    SCL_H;                    //拉高时钟线
    DELAY_US(2);                   //延时
    SDA_H;                    //产生上升沿
    DELAY_US(2);                  //延时
}
//******************************************************************************
//! 函数名：                 IIC_SendACK
//! 功能：                     IIC发送应答信号
//! 作者：                      帅帅
//******************************************************************************
void IIC_SendACK(uint8_t ack){
    SDA_OUT;
    SCL_OUT;
    if(ack) SDA_H;
    else SDA_L;
//    SDA = ack;                  //写应答信号
    SCL_H;                    //拉高时钟线
    DELAY_US(2);                  //延时
    SCL_L;                    //拉低时钟线
    DELAY_US(2);                  //延时
}
//******************************************************************************
//! 函数名：                 IIC_RecvACK
//! 功能：                     IIC接收应答信号
//! 作者：                      帅帅
//******************************************************************************
uint8_t IIC_RecvACK(){
    uint8_t cy;
    SCL_OUT;
    SCL_H;                    //拉高时钟线
    SDA_IN;
    DELAY_US(2);                 //延时
    if(SDA_DATA)    cy=1;
    else            cy=0;
//    cy = SDA;                   //读应答信号
    SCL_L;                    //拉低时钟线
    DELAY_US(2);                //延时
    SDA_OUT;
    return cy;
}
//******************************************************************************
//! 函数名：                 IIC_SendByte
//! 功能：                     向IIC总线发送一个字节数据
//! 作者：                      帅帅
//******************************************************************************
void IIC_SendByte(uint8_t dat){
    uint8_t i;
    SDA_OUT;
    for (i=0; i<8; i++){          //8位计数器
        if((dat<<i)&0x80)   SDA_H;
        else                SDA_L;
       // SDA = cy;               //送数据口
        SCL_OUT;
        SCL_H;                //拉高时钟线
        DELAY_US(2);              //延时
        SCL_L;                //拉低时钟线
        DELAY_US(2);              //延时
    }
    IIC_RecvACK();
}

//******************************************************************************
//! 函数名：                 IIC_RecvByte
//! 功能：                     从IIC总线接收一个字节数据
//! 作者：                      帅帅
//******************************************************************************
uint8_t IIC_RecvByte(){
    uint8_t i;
    uint8_t dat = 0,cy;
    SDA_OUT;
    SCL_OUT;
    SDA_H;                    //使能内部上拉,准备读取数据,
    SDA_IN;
    for (i=0; i<8; i++){         //8位计数器
        dat<<= 1;
        SCL_H;                //拉高时钟线
        DELAY_US(2);             //延时
        if(SDA_DATA)    cy=1;
        else            cy=0;
        dat |= cy;             //读数据
        SCL_L;                //拉低时钟线
        DELAY_US(2);             //延时
    }
    SDA_OUT;
    return dat;
}
//******************************************************************************
//! 函数名：                 Single_WriteIIC
//! 功能：                     向IIC设备写入一个字节数据
//! 作者：                      帅帅
//******************************************************************************
void Single_WriteIIC(uint8_t REG_Address,uint8_t REG_data)
{
    IIC_Start();                  //起始信号
    IIC_SendByte(SlaveAddress);   //发送设备地址+写信号
    IIC_SendByte(REG_Address);    //内部寄存器地址
    IIC_SendByte(REG_data);       //内部寄存器数据
    IIC_Stop();                   //发送停止信号
}

//******************************************************************************
//! 函数名：                 Single_ReadIIC
//! 功能：                    从IIC设备读取一个字节数据
//! 作者：                      帅帅
//******************************************************************************
uint8_t  Single_ReadIIC(uint8_t REG_Address)
{
 uint8_t REG_data;
 IIC_Start();                   //起始信号
 IIC_SendByte(SlaveAddress);    //发送设备地址+写信号
 IIC_SendByte(REG_Address);     //发送存储单元地址从0开始
 IIC_Start();                   //起始信号
 IIC_SendByte(SlaveAddress+1);  //发送设备地址+读信号
 REG_data=IIC_RecvByte();       //读出寄存器数据
 IIC_SendACK(1);                //接收应答信号
 IIC_Stop();                    //停止信号
 return REG_data;
}
//******************************************************************************
//! 函数名：                 MPU6050_INIT
//! 功能：                     初始化MPU6050
//! 作者：                      帅帅
//******************************************************************************
void MPU6050_INIT(){
    Single_WriteIIC(PWR_MGMT_1, 0x00); //解除休眠状态
    Single_WriteIIC(SMPLRT_DIV, 0x07);
    Single_WriteIIC(CONFIG, 0x06);
    Single_WriteIIC(GYRO_CONFIG, 0x18);
    Single_WriteIIC(ACCEL_CONFIG, 0x01);
}
//******************************************************************************
//! 函数名：                 MPU6050_GET
//! 功能：                     读取MPU6050
//! 入口：                     选轴
//! 作者：                     帅帅
//******************************************************************************
int MPU6050_GET(uint8_t REG_Address){
    char H,L;
    H = Single_ReadIIC(REG_Address);
    L = Single_ReadIIC(REG_Address+1);
    return ((H<<8)+L)/64;
}

//卡尔曼参数
float Q_angle = 0.05; //上次为0.05
float Q_gyro  = 0.05;
float R_angle = 0.06;
float dt      = 0.01;//dt为kalman滤波器采样时间

char  C_0     = 1;
float Q_bias, Angle_err;
float PCt_0, PCt_1, E;
float K_0, K_1, t_0, t_1;
float Pdot[4] ={0,0,0,0};
float PP[2][2] = { { 1, 0 },{ 0, 1 } };
float ax,ay,az,Gyro_x,Gyro_y,Gyro_z,Angle_x_temp,Angle_y_temp;
float Angle_X_Final_Kalman,Angle_Y_Final_Kalman;
float Angle_Y_Final_Kalman_last;
//卡尔曼滤波函数
void Kalman_Filter_X(float Accel,float Gyro) //卡尔曼函数
{

    Angle_X_Final_Kalman += (Gyro - Q_bias) * dt; //先验估计

    Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

    Pdot[1]= -PP[1][1];
    Pdot[2]= -PP[1][1];
    Pdot[3]= Q_gyro;


    PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
    PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
    PP[1][0] += Pdot[2] * dt;
    PP[1][1] += Pdot[3] * dt;


    Angle_err = Accel - Angle_X_Final_Kalman;  //-先验估计

    PCt_0 = C_0 * PP[0][0];
    PCt_1 = C_0 * PP[1][0];

    E = R_angle + C_0 * PCt_0;

    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;

    t_0 = PCt_0;
    t_1 = C_0 * PP[0][1];

    PP[0][0] -= K_0 * t_0;       //后验估计误差协方差
    PP[0][1] -= K_0 * t_1;
    PP[1][0] -= K_1 * t_0;
    PP[1][1] -= K_1 * t_1;

    Angle_X_Final_Kalman += K_0 * Angle_err;    //后验估计
    Q_bias += K_1 * Angle_err;    //后验估计
}

void Kalman_Filter_Y(float Accel,float Gyro) //卡尔曼函数
{
    Angle_Y_Final_Kalman += (Gyro - Q_bias) * dt; //先验估计

    Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

    Pdot[1]=- PP[1][1];
    Pdot[2]=- PP[1][1];
    Pdot[3]=Q_gyro;

    PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
    PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
    PP[1][0] += Pdot[2] * dt;
    PP[1][1] += Pdot[3] * dt;

    Angle_err=Accel-Angle_Y_Final_Kalman;  //-先验估计

    PCt_0 = C_0 * PP[0][0];
    PCt_1 = C_0 * PP[1][0];

    E = R_angle + C_0 * PCt_0;

    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;

    t_0 = PCt_0;
    t_1 = C_0 * PP[0][1];

    PP[0][0] -= K_0 * t_0;       //后验估计误差协方差
    PP[0][1] -= K_0 * t_1;
    PP[1][0] -= K_1 * t_0;
    PP[1][1] -= K_1 * t_1;

    Angle_Y_Final_Kalman   += K_0 * Angle_err;  //后验估计
    Q_bias  += K_1 * Angle_err;  //后验估计
}
///******************
// * @brief 陀螺仪数据采集
// * @param 无；
// * @see MPU6050_Tim();
// *
// * */
//
void MPU6050_Tim(void)
{
//    char txt[16];
    short aacx,aacy,aacz;    //加速度传感器原始数据
    short gyrox,gyroy,gyroz; //陀螺仪原始数据

//    MPU_Get_Raw_data(&aacx,&aacy,&aacz,&gyrox,&gyroy,&gyroz);   //得到加速度传感器数据
    aacx = MPU6050_GET(ACCEL_XOUT_H);
    aacy = MPU6050_GET(ACCEL_YOUT_H);
    aacz = MPU6050_GET(ACCEL_ZOUT_H);
    gyrox = MPU6050_GET(GYRO_XOUT_H);
    gyroy = MPU6050_GET(GYRO_YOUT_H);
    gyroz = MPU6050_GET(GYRO_ZOUT_H);

    ax = (9.8*aacx)/8192;//姿态解算
    ay = (9.8*aacy)/8192;
    az = (9.8*aacz)/8192;

    Gyro_x = (gyrox)/16.4;
    Gyro_y = (gyroy)/16.4;
    Gyro_z = (gyroz)/16.4;


    //用加速度计算三个轴和水平面坐标系之间的夹角
//    Angle_x_temp=(atan((float)(ay)/(float)(az)))*180/3.14;   //计算x轴夹角
    Angle_y_temp=(atan((float)(ax)/(float)(az)))*180/3.14;   //计算y轴夹角

    Angle_x_temp = (atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * 180 / 3.14);
    Kalman_Filter_X(Angle_x_temp,Gyro_z);
    Kalman_Filter_Y(Angle_y_temp,Gyro_z);
//    sprintf(txt,"Angle_X:%f", Angle_X_Final_Kalman);
//    TFTSPI_P6X8Str(1,12,txt,u16YELLOW, u16BLUE);
//    OLED_DispDecAt(FONT_ASCII_6X8,3,50,Angle_X_Final_Kalman,3);          //显示接收到的数据加速度Y
//    OLED_DispDecAt(FONT_ASCII_6X8,5,50,Angle_Y_Final_Kalman,3);          //显示接收到的数据加速度Z
}
