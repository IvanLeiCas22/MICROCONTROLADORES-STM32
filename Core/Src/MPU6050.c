#include "MPU6050.h"

/**
 * @brief Inicializa el MPU6050: saca del sleep y configura rangos por defecto.
 */
HAL_StatusTypeDef MPU6050_Init(MPU6050_Handle_t *hmpu, I2C_HandleTypeDef *hi2c) {
    hmpu->hi2c = hi2c;
    hmpu->ready = 0;
    // Wake up (PWR_MGMT_1 = 0)
    uint8_t data = 0;
    if (HAL_I2C_Mem_Write(hi2c, MPU6050_I2C_ADDR, MPU6050_REG_PWR_MGMT_1, 1, &data, 1, 100) != HAL_OK)
        return HAL_ERROR;
    // Configuración de sample rate, filtros y rangos (opcional, default)
    // Aquí puedes agregar configuraciones avanzadas si lo deseas
    return HAL_OK;
}

/**
 * @brief Inicia la lectura de 14 bytes (acelerómetro + temp + giroscopio) por DMA.
 * El callback debe ser llamado en HAL_I2C_MemRxCpltCallback.
 */
HAL_StatusTypeDef MPU6050_Read_DMA(MPU6050_Handle_t *hmpu) {
    hmpu->ready = 0;
    return HAL_I2C_Mem_Read_DMA(hmpu->hi2c, MPU6050_I2C_ADDR, MPU6050_REG_ACCEL_XOUT_H, 1, hmpu->raw, 14);
}

/**
 * @brief Procesa los datos RAW y los almacena en la estructura de datos.
 * Llamar en HAL_I2C_MemRxCpltCallback.
 */
void MPU6050_DMA_Complete_Callback(MPU6050_Handle_t *hmpu) {
    // Acelerómetro
    hmpu->data.accel_x = (int16_t)(hmpu->raw[0] << 8 | hmpu->raw[1]);
    hmpu->data.accel_y = (int16_t)(hmpu->raw[2] << 8 | hmpu->raw[3]);
    hmpu->data.accel_z = (int16_t)(hmpu->raw[4] << 8 | hmpu->raw[5]);
    // Giroscopio
    hmpu->data.gyro_x = (int16_t)(hmpu->raw[8] << 8 | hmpu->raw[9]);
    hmpu->data.gyro_y = (int16_t)(hmpu->raw[10] << 8 | hmpu->raw[11]);
    hmpu->data.gyro_z = (int16_t)(hmpu->raw[12] << 8 | hmpu->raw[13]);
    hmpu->ready = 1;
} 