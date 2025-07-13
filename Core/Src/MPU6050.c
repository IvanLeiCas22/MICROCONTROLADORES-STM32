#include <stdint.h>
#include "MPU6050.h"

int8_t MPU6050_Init(MPU6050_HandleTypeDef *hmpu)
{
    if (!hmpu->i2c_read_blocking || !hmpu->i2c_write_blocking || !hmpu->delay_ms) {
        return -7; // Error: Funciones I2C no asignadas
    }

    uint8_t who_am_i, config_data;

    // 1. PRIMERO verificar si el dispositivo existe
    if(hmpu->i2c_read_blocking(hmpu->device_address, MPU6050_WHO_AM_I_ADDR, &who_am_i, 1, hmpu->i2c_context) != 1) {
        hmpu->is_connected = 0;
        return -9; // Error leyendo WHO_AM_I - dispositivo no encontrado
    }
    
    if(who_am_i == 0x68) {
        hmpu->is_connected = 1;
    } else {
        hmpu->is_connected = 0;
        return -2; // Dispositivo encontrado, pero no es un MPU6050
    }
    
    hmpu->delay_ms(100);

    // 2. Reset del dispositivo
    config_data = 0x80; // Escribir un 1 en el bit 7 (DEVICE_RESET)
    if(hmpu->i2c_write_blocking(hmpu->device_address, MPU6050_PWR_MGMT_1_ADDR, &config_data, 1, hmpu->i2c_context) != 1) {
        return -8; // Error de I2C durante el reset
    }
    hmpu->delay_ms(1000); // Esperar que el reset termine

    // 3. Despertar el sensor
    config_data = 0x00;  // Clear sleep bit
    if(hmpu->i2c_write_blocking(hmpu->device_address, MPU6050_PWR_MGMT_1_ADDR, &config_data, 1, hmpu->i2c_context) != 1) {
        return -3;
    }
    hmpu->delay_ms(100);

    // 4. Configurar acelerómetro (±2g)
    config_data = hmpu->accel_range;
    if(hmpu->i2c_write_blocking(hmpu->device_address, MPU6050_ACCEL_CONFIG_ADDR, &config_data, 1, hmpu->i2c_context) != 1) {
        return -4;
    }
    hmpu->delay_ms(100);

    // 5. Configurar giroscopio (±250°/s)
    config_data = hmpu->gyro_range;
    if(hmpu->i2c_write_blocking(hmpu->device_address, MPU6050_GYRO_CONFIG_ADDR, &config_data, 1, hmpu->i2c_context) != 1) {
        return -5;
    }
    hmpu->delay_ms(100);

    // 6. Configurar DLPF (44Hz)
    config_data = hmpu->dlpf_config;
    if(hmpu->i2c_write_blocking(hmpu->device_address, MPU6050_CONFIG_ADDR, &config_data, 1, hmpu->i2c_context) != 1) {
        return -6;
    }
    hmpu->delay_ms(100);

    hmpu->is_initialized = 1;
    return 1;
}

int8_t MPU6050_ReadRawDataDMA(MPU6050_HandleTypeDef *hmpu)
{
    if(hmpu->dma_busy) {
        return -1;  // DMA ya está en uso
    }
    
    hmpu->dma_busy = 1;
    
    // Iniciar lectura DMA
    if(hmpu->i2c_read_dma(hmpu->device_address, MPU6050_ACCEL_XOUT_H_ADDR, hmpu->dma_buffer, 14, hmpu->i2c_context) != 1) {
        hmpu->dma_busy = 0;
        return -1;
    }
    
    return 1;
}