//
// Created by Lenovo on 2026/11/5.
//

#ifndef MPU6050_MPU6050_H
#define MPU6050_MPU6050_H

#include "i2c.h"

void MPU6050_Init(I2C_HandleTypeDef *hic1);
uint16_t MPU6050_ReadDate(I2C_HandleTypeDef *hi2c, uint8_t reg);
uint8_t MPU6050_ReadID(I2C_HandleTypeDef *hi2c);
void MPU6050_ReadAccel(I2C_HandleTypeDef *hi2c,int16_t *ax, int16_t *ay, int16_t *az);
void MPU6050_ReadGyro(I2C_HandleTypeDef *hi2c,int16_t *ax, int16_t *ay, int16_t *az);
int16_t MPU6050_ReadTemp(I2C_HandleTypeDef *hi2c);
void MPU6050_Result(float *pAccelOut, float *pGyroOut, float *pTempOut, float *pEularAngleOut);
#endif //MPU6050_MPU6050_H
