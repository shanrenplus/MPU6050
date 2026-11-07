//
// Created by Lenovo on 2026/11/5.
//
#include "MPU6050.h"
#include "math.h"

static float last_roll=0.0f;
static float last_pitch=0.0f;
static float last_yaw=0.0f;
static uint32_t lastTime=0;

//设备地址0x68<<1
#define MPU6050_ADDR    (0XD0)
//寄存器地址
#define PWR_MGMT_1   (0x6B)         //电源管理寄存器地址
#define ACCEL_XOUT_H   (0x3B)       //加速度计X轴高位寄存器地址
#define GYRO_XOUT_H   (0x43)        //陀螺仪X轴高位寄存器地址
#define MPU6050_WHO_AM_I_REG (0x75) //设备ID寄存器地址
#define TEMP_OUT_H   (0x41)         //温度高位
#define TEMP_OUT_L   (0x42)         //温度低位
void MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t date;
    //唤醒MPU6050
    date=0x00;
    HAL_I2C_Mem_Write(hi2c,MPU6050_ADDR,PWR_MGMT_1,1,&date,1,HAL_MAX_DELAY);
    //采样分辨率
    date=0x07;
    HAL_I2C_Mem_Write(hi2c,MPU6050_ADDR,0X19,1,&date,1,HAL_MAX_DELAY);
    //陀螺仪角速度，±500°/s
    date=0x08;
    HAL_I2C_Mem_Write(hi2c,MPU6050_ADDR,0X1B,1,&date,1,HAL_MAX_DELAY);
    //加速度计 ±2g
    date=0x00;
    HAL_I2C_Mem_Write(hi2c,MPU6050_ADDR,0X1c,1,&date,1,HAL_MAX_DELAY);
}
//读取MPU6050的16位数据
uint16_t MPU6050_ReadDate(I2C_HandleTypeDef *hi2c, uint8_t reg)
{
    uint8_t data[2];
    HAL_I2C_Mem_Read(hi2c,MPU6050_ADDR,reg,1,data,2,HAL_MAX_DELAY);
    return(int16_t)((data[0]<<8)|data[1]);
}
//读取MPU6050设备号
uint8_t MPU6050_ReadID(I2C_HandleTypeDef *hi2c)
{
    uint8_t id;
    HAL_I2C_Mem_Read(hi2c,MPU6050_ADDR,MPU6050_WHO_AM_I_REG,1,&id,1,HAL_MAX_DELAY);
    return id;
}
//读取加速度计数据
void MPU6050_ReadAccel(I2C_HandleTypeDef *hi2c,int16_t *ax, int16_t *ay, int16_t *az)
{
    *ax=(int16_t)MPU6050_ReadDate(hi2c,ACCEL_XOUT_H);
    *ay=(int16_t)MPU6050_ReadDate(hi2c,ACCEL_XOUT_H +2);
    *az=(int16_t)MPU6050_ReadDate(hi2c,ACCEL_XOUT_H +4);
}
//读取陀螺仪数据
void MPU6050_ReadGyro(I2C_HandleTypeDef *hi2c,int16_t *ax, int16_t *ay, int16_t *az)
{
    *ax=(int16_t)MPU6050_ReadDate(hi2c,GYRO_XOUT_H);
    *ay=(int16_t)MPU6050_ReadDate(hi2c,GYRO_XOUT_H +2);
    *az=(int16_t)MPU6050_ReadDate(hi2c,GYRO_XOUT_H +4);
}
int16_t MPU6050_ReadTemp(I2C_HandleTypeDef *hi2c)
{
    return (int16_t)MPU6050_ReadDate(hi2c,TEMP_OUT_H);
}
void MPU6050_Result(float *pAccelOut, float *pGyroOut, float *pTempOut, float *pEularAngleOut)
{
    int16_t rawAccel[3];
    int16_t rawGyro[3];

    MPU6050_ReadAccel(&hi2c1,&rawAccel[0],&rawAccel[1],&rawAccel[2]);
    MPU6050_ReadGyro(&hi2c1,&rawGyro[0],&rawGyro[1],&rawGyro[2]);
    int16_t rawTemp=MPU6050_ReadTemp(&hi2c1);

    pAccelOut[0]=(float)rawAccel[0]/163843.0f;
    pAccelOut[1]=(float)rawAccel[1]/163843.0f;
    pAccelOut[2]=(float)rawAccel[2]/163843.0f;

    pGyroOut[0]=(float)rawGyro[0]/65.5f;
    pGyroOut[1]=(float)rawGyro[1]/65.5f;
    pGyroOut[2]=(float)rawGyro[2]/65.5f;

    *pTempOut=((float)rawTemp/340.0f)+36.53f;

    uint32_t currentTime=HAL_GetTick();
    float dt=(float)(currentTime-lastTime)/1000.0f;
    lastTime=currentTime;

    if (dt<=0.0f||dt>=0.5f)
    {
        dt=0.01f;
    }

    float accX=pAccelOut[0];
    float accY=pAccelOut[1];
    float accZ=pAccelOut[2];
    float rad2deg=57.29578f;

    float acc_roll =atan2(accZ,accY)*rad2deg;
    float acc_pitch =atan2(-accX,sqrt(accX*accX+accY*accY))*rad2deg;

    float alpha=0.98f;

    float roll=alpha*(last_roll+pGyroOut[0]*dt)+(1.0f-alpha)*acc_roll;
    last_roll=roll;
    float pitch=alpha*(last_pitch+pGyroOut[1]*dt)+(1.0f-alpha)*acc_pitch;
    last_pitch=pitch;
    float yaw=last_yaw+pGyroOut[2]*dt;
    last_yaw=yaw;

    pEularAngleOut[0]=roll;
    pEularAngleOut[1]=pitch;
    pEularAngleOut[2]=yaw;

}
