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
    // Iniciar lectura DMA
    if (hmpu->i2c_read_dma(hmpu->device_address, MPU6050_ACCEL_XOUT_H_ADDR, hmpu->dma_buffer, MPU6050_RAW_DATA_LEN, hmpu->i2c_context) != MPU6050_OK)
    {
        return MPU6050_I2C_ERROR;
    }

    return MPU6050_OK;
}

// Calibración: promedia N lecturas en reposo y guarda los offsets
void MPU6050_Calibrate(MPU6050_HandleTypeDef *hmpu, uint16_t samples)
{
    int32_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
    for (uint16_t i = 0; i < samples; i++)
    {
        // Leer datos crudos (blocking)
        uint8_t buf[14];
        if (hmpu->i2c_read_blocking(hmpu->device_address, MPU6050_ACCEL_XOUT_H_ADDR, buf, 14, hmpu->i2c_context) == MPU6050_OK)
        {
            int16_t accel_x = (int16_t)((buf[0] << 8) | buf[1]);
            int16_t accel_y = (int16_t)((buf[2] << 8) | buf[3]);
            int16_t accel_z = (int16_t)((buf[4] << 8) | buf[5]);
            int16_t gyro_x = (int16_t)((buf[8] << 8) | buf[9]);
            int16_t gyro_y = (int16_t)((buf[10] << 8) | buf[11]);
            int16_t gyro_z = (int16_t)((buf[12] << 8) | buf[13]);
            ax += accel_x;
            ay += accel_y;
            az += accel_z;
            gx += gyro_x;
            gy += gyro_y;
            gz += gyro_z;
        }
        if (hmpu->delay_ms)
            hmpu->delay_ms(2);
    }
    hmpu->accel_offset_x = (int16_t)(ax / samples);
    hmpu->accel_offset_y = (int16_t)(ay / samples);
    // Para Z, restar 1g (depende del rango, para 2g: 16384)
    hmpu->accel_offset_z = (int16_t)((az / samples) - 16384);
    hmpu->gyro_offset_x = (int16_t)(gx / samples);
    hmpu->gyro_offset_y = (int16_t)(gy / samples);
    hmpu->gyro_offset_z = (int16_t)(gz / samples);
}

// Devuelve los datos calibrados (resta los offsets)
void MPU6050_GetCalibratedData(MPU6050_HandleTypeDef *hmpu, int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
{
    if (ax)
        *ax = hmpu->raw_data.accel_x_raw - hmpu->accel_offset_x;
    if (ay)
        *ay = hmpu->raw_data.accel_y_raw - hmpu->accel_offset_y;
    if (az)
        *az = hmpu->raw_data.accel_z_raw - hmpu->accel_offset_z;
    if (gx)
        *gx = hmpu->raw_data.gyro_x_raw - hmpu->gyro_offset_x;
    if (gy)
        *gy = hmpu->raw_data.gyro_y_raw - hmpu->gyro_offset_y;
    if (gz)
        *gz = hmpu->raw_data.gyro_z_raw - hmpu->gyro_offset_z;
}