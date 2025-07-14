#include <stdint.h>
#include "MPU6050.h"

int8_t MPU6050_Init(MPU6050_HandleTypeDef *hmpu)
{
    if (!hmpu->i2c_read_blocking || !hmpu->i2c_write_blocking || !hmpu->delay_ms)
    {
        return MPU6050_ERROR_I2C_FUNCS_NOT_ASSIGNED; // Error: Funciones I2C no asignadas
    }

    uint8_t who_am_i, config_data;

    // 1. PRIMERO verificar si el dispositivo existe
    if (hmpu->i2c_read_blocking(hmpu->device_address, MPU6050_WHO_AM_I_ADDR, &who_am_i, MPU6050_SINGLE_BYTE_LEN, hmpu->i2c_context) != MPU6050_OK)
    {
        hmpu->is_connected = 0;
        return MPU6050_ERROR_WHO_AM_I_READ_FAILED; // Error leyendo WHO_AM_I - dispositivo no encontrado
    }

    if (who_am_i == MPU6050_EXPECTED_WHO_AM_I_VALUE)
    {
        hmpu->is_connected = 1;
    }
    else
    {
        hmpu->is_connected = 0;
        return MPU6050_ERROR_WRONG_DEVICE_ID; // Dispositivo encontrado, pero no es un MPU6050
    }

    hmpu->delay_ms(MPU6050_INIT_DELAY_MS);

    // 2. Reset del dispositivo
    config_data = MPU6050_PWR_MGMT_1_RESET_BIT; // Escribir un 1 en el bit 7 (DEVICE_RESET)
    if (hmpu->i2c_write_blocking(hmpu->device_address, MPU6050_PWR_MGMT_1_ADDR, &config_data, MPU6050_SINGLE_BYTE_LEN, hmpu->i2c_context) != MPU6050_OK)
    {
        return MPU6050_ERROR_RESET_FAILED; // Error de I2C durante el reset
    }
    hmpu->delay_ms(MPU6050_RESET_DELAY_MS); // Esperar que el reset termine

    // 3. Despertar el sensor
    config_data = MPU6050_PWR_MGMT_1_WAKEUP_VALUE; // Clear sleep bit
    if (hmpu->i2c_write_blocking(hmpu->device_address, MPU6050_PWR_MGMT_1_ADDR, &config_data, MPU6050_SINGLE_BYTE_LEN, hmpu->i2c_context) != MPU6050_OK)
    {
        return MPU6050_ERROR_WAKEUP_FAILED;
    }
    hmpu->delay_ms(MPU6050_INIT_DELAY_MS);

    // 4. Configurar acelerómetro (±2g)
    config_data = hmpu->accel_range;
    if (hmpu->i2c_write_blocking(hmpu->device_address, MPU6050_ACCEL_CONFIG_ADDR, &config_data, MPU6050_SINGLE_BYTE_LEN, hmpu->i2c_context) != MPU6050_OK)
    {
        return MPU6050_ERROR_ACCEL_CONFIG_FAILED;
    }
    hmpu->delay_ms(MPU6050_INIT_DELAY_MS);

    // 5. Configurar giroscopio (±250°/s)
    config_data = hmpu->gyro_range;
    if (hmpu->i2c_write_blocking(hmpu->device_address, MPU6050_GYRO_CONFIG_ADDR, &config_data, MPU6050_SINGLE_BYTE_LEN, hmpu->i2c_context) != MPU6050_OK)
    {
        return MPU6050_ERROR_GYRO_CONFIG_FAILED;
    }
    hmpu->delay_ms(MPU6050_INIT_DELAY_MS);

    // 6. Configurar DLPF (44Hz)
    config_data = hmpu->dlpf_config;
    if (hmpu->i2c_write_blocking(hmpu->device_address, MPU6050_CONFIG_ADDR, &config_data, MPU6050_SINGLE_BYTE_LEN, hmpu->i2c_context) != MPU6050_OK)
    {
        return MPU6050_ERROR_DLPF_CONFIG_FAILED;
    }
    hmpu->delay_ms(MPU6050_INIT_DELAY_MS);

    hmpu->is_initialized = 1;
    return MPU6050_OK;
}

int8_t MPU6050_ReadRawDataDMA(MPU6050_HandleTypeDef *hmpu)
{
    if (hmpu->dma_busy)
    {
        return MPU6050_ERROR_DMA_BUSY; // DMA ya está en uso
    }

    hmpu->dma_busy = 1;

    // Iniciar lectura DMA
    if (hmpu->i2c_read_dma(hmpu->device_address, MPU6050_ACCEL_XOUT_H_ADDR, hmpu->dma_buffer, MPU6050_RAW_DATA_LEN, hmpu->i2c_context) != MPU6050_OK)
    {
        hmpu->dma_busy = 0;
        return MPU6050_ERROR_DMA_BUSY; // Reutilizo el mismo error para indicar fallo al iniciar DMA
    }

    return MPU6050_OK;
}