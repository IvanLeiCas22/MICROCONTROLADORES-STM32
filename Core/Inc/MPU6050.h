#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/**
 * @file MPU6050.h
 * @brief Driver para MPU6050 usando I2C + DMA en STM32 (sin temperatura)
 */

#define MPU6050_I2C_ADDR         0x68 << 1  // Dirección I2C (shifted para HAL)

// Registros MPU6050
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_GYRO_XOUT_H  0x43
#define MPU6050_REG_WHO_AM_I     0x75
#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C

// Estructura para datos de acelerómetro y giroscopio
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} MPU6050_Data_t;

// Handler principal
typedef struct {
    I2C_HandleTypeDef *hi2c;
    DMA_HandleTypeDef *hdma;
    MPU6050_Data_t data;
    uint8_t raw[14]; // 6 accel + 2 temp + 6 gyro (temp se ignora)
    volatile uint8_t ready;
} MPU6050_Handle_t;

#ifdef __cplusplus
extern "C" {
#endif

// Inicializa el MPU6050 (configura registros básicos)
HAL_StatusTypeDef MPU6050_Init(MPU6050_Handle_t *hmpu, I2C_HandleTypeDef *hi2c);

// Inicia la lectura por DMA (no bloqueante)
HAL_StatusTypeDef MPU6050_Read_DMA(MPU6050_Handle_t *hmpu);

// Callback a llamar en HAL_I2C_MemRxCpltCallback
void MPU6050_DMA_Complete_Callback(MPU6050_Handle_t *hmpu);

#ifdef __cplusplus
}
#endif

#endif // MPU6050_H 