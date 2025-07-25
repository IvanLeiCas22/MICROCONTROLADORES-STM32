/* STM32/Test2024-master/Core/Src/app_core.c */
#include "app_core.h"
#include "app_config.h"

#include <stdlib.h>
#include <stdio.h> // Para snprintf

#include "usbd_cdc_if.h"
#include "ESP01.h"
#include "UNERBUS.h"
#include "MPU6050.h"
#include "BUTTONS.h"
#include "SSD1306.h"
#include "pid_controller.h"

//==============================================================================
// DECLARACIONES EXTERN DE HANDLES DE PERIFÉRICOS (definidos en main.c)
//==============================================================================
extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_i2c2_rx;
extern DMA_HandleTypeDef hdma_i2c2_tx;

//==============================================================================
// VARIABLES GLOBALES DEL MÓDULO
//==============================================================================
SystemFlagTypeDef flags0;
uint16_t pwm_max_value = 1000; // Valor máximo del PWM

_sESP01Handle esp01_handle;
_sUNERBUSHandle unerbus_pc_handle;
_sUNERBUSHandle unerbus_esp01_handle;

char local_ip[IP_ADDRESS_STRING_LENGTH];
uint8_t buf_rx_pc[USB_CDC_RX_BUFFER_SIZE], buf_tx_pc[USB_CDC_TX_BUFFER_SIZE];
uint8_t buf_rx_esp01[WIFI_RX_BUFFER_SIZE], buf_tx_esp01[WIFI_TX_BUFFER_SIZE], data_rx_esp01;

uint32_t heartbeat_counter, heartbeat_mask;
uint8_t time_10ms, time_100ms, timeout_alive_udp;

uint16_t buf_adc[ADC_BUFFER_SIZE][ADC_CHANNELS];
uint8_t adc_buf_write_idx, adc_buf_read_idx;

static MPU6050_HandleTypeDef hmpu;
static SSD1306_HandleTypeDef hssd;
Button_HandleTypeDef h_user_button;
uint16_t motor_pwm_values[PWM_CHANNELS] = {0, 0, 0, 0};
static volatile I2C_BusStateTypeDef i2c_bus_state = I2C_BUS_IDLE;

PID_Controller_t wall_pid;
PID_Controller_t turn_pid;
uint16_t wall_follow_setpoint = 300;    // Valor ADC objetivo
uint16_t base_speed = 4000;             // Velocidad base de referencia
uint16_t right_motor_base_speed = 5000; // Velocidad base motor derecho
uint16_t left_motor_base_speed = 2500;  // Velocidad base motor izquierdo
uint16_t max_pwm_correction = 4000;     // Corrección máxima del PID

static volatile TurnStateTypeDef turn_state = TURN_STATE_IDLE;
static int32_t current_yaw_fixed = 0; // Yaw angle in Q16.16 fixed-point (degrees)
static int32_t target_yaw_fixed = 0;  // Target yaw for turning

//==============================================================================
// PROTOTIPOS DE FUNCIONES PRIVADAS
//==============================================================================
void ESP01_SetChipEnable(uint8_t value);
int ESP01_WriteUartByte(uint8_t value);
void ESP01_WriteByteToRxBuffer(uint8_t value);
void ESP01_ChangeState(_eESP01STATUS esp01State);
void DecodeCMD(struct UNERBUSHandle *aBus, uint8_t iStartData);
void Do10ms(void);
void Do100ms(void);
static void ManageTransmission(void);
static int8_t I2C_WriteBlocking(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, uint16_t data_len, void *context);
static int8_t I2C_WriteDMA(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, uint16_t data_len, void *context);
static int8_t I2C_ReadBlocking(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, uint16_t data_len, void *context);
static int8_t I2C_ReadDMA(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, uint16_t data_len, void *context);
uint8_t Read_User_Button(void *context);
static void ManageButtonEvents(void);
void IndicateError(uint8_t blinks, uint32_t delay_ms);
int8_t I2C_DevicesInit(void);
static void ManageI2CTransactions(void);
uint8_t UART_TransmitByte(uint8_t value);
void App_Core_Control_Loop(void);

static void Update_Yaw(void);
static void Manage_Turn(void);
void Turn_Start(int16_t angle_degrees);
static void Set_Motor_Speeds(int16_t right_speed, int16_t left_speed);

//==============================================================================
// IMPLEMENTACIÓN DE WRAPPERS DE CALLBACKS HAL
//==============================================================================
void App_Core_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        time_10ms--;
        if (!time_10ms)
        {
            ON10MS = true;
            time_10ms = TIME_10MS_PERIOD_COUNT;
        }
        HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&buf_adc[adc_buf_write_idx], ADC_CHANNELS);
    }
}

void App_Core_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    adc_buf_write_idx++;
    adc_buf_write_idx %= ADC_BUFFER_SIZE;
}

void App_Core_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if (UART_BYPASS)
        {
            UNERBUS_ReceiveByte(&unerbus_pc_handle, data_rx_esp01);
        }
        else
        {
            ESP01_WriteRX(data_rx_esp01);
        }
        HAL_UART_Receive_IT(&huart1, &data_rx_esp01, 1);
    }
}

void App_Core_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == &hi2c2)
    {
        i2c_bus_state = I2C_BUS_IDLE;
    }
}

void App_Core_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == &hi2c2)
    {
        hmpu.raw_data.accel_x_raw = (int16_t)((hmpu.dma_buffer[MPU_DMA_BUF_ACCEL_X_H] << 8) | hmpu.dma_buffer[MPU_DMA_BUF_ACCEL_X_L]);
        hmpu.raw_data.accel_y_raw = (int16_t)((hmpu.dma_buffer[MPU_DMA_BUF_ACCEL_Y_H] << 8) | hmpu.dma_buffer[MPU_DMA_BUF_ACCEL_Y_L]);
        hmpu.raw_data.accel_z_raw = (int16_t)((hmpu.dma_buffer[MPU_DMA_BUF_ACCEL_Z_H] << 8) | hmpu.dma_buffer[MPU_DMA_BUF_ACCEL_Z_L]);
        hmpu.raw_data.temp_raw = (int16_t)((hmpu.dma_buffer[MPU_DMA_BUF_TEMP_H] << 8) | hmpu.dma_buffer[MPU_DMA_BUF_TEMP_L]);
        hmpu.raw_data.gyro_x_raw = (int16_t)((hmpu.dma_buffer[MPU_DMA_BUF_GYRO_X_H] << 8) | hmpu.dma_buffer[MPU_DMA_BUF_GYRO_X_L]);
        hmpu.raw_data.gyro_y_raw = (int16_t)((hmpu.dma_buffer[MPU_DMA_BUF_GYRO_Y_H] << 8) | hmpu.dma_buffer[MPU_DMA_BUF_GYRO_Y_L]);
        hmpu.raw_data.gyro_z_raw = (int16_t)((hmpu.dma_buffer[MPU_DMA_BUF_GYRO_Z_H] << 8) | hmpu.dma_buffer[MPU_DMA_BUF_GYRO_Z_L]);
        i2c_bus_state = I2C_BUS_IDLE;
    }
}

void App_Core_USB_ReceiveData(uint8_t *buf, uint16_t len)
{
    UNERBUS_ReceiveBuf(&unerbus_pc_handle, buf, len);
}

//==============================================================================
// IMPLEMENTACIÓN DE FUNCIONES DE LA APLICACIÓN
//==============================================================================

void ESP01_SetChipEnable(uint8_t value)
{
    HAL_GPIO_WritePin(CH_EN_GPIO_Port, CH_EN_Pin, value);
}

int ESP01_WriteUartByte(uint8_t value)
{
    if (__HAL_UART_GET_FLAG(&huart1, USART_SR_TXE))
    {
        USART1->DR = value;
        return true;
    }
    return false;
}

void ESP01_WriteByteToRxBuffer(uint8_t value)
{
    UNERBUS_ReceiveByte(&unerbus_esp01_handle, value);
}

void ESP01_ChangeState(_eESP01STATUS esp01State)
{
    switch ((uint32_t)esp01State)
    {
    case ESP01_WIFI_CONNECTED:
        heartbeat_counter = HEARTBEAT_WIFI_READY;
        break;
    case ESP01_UDPTCP_CONNECTED:
        heartbeat_counter = HEARTBEAT_UDP_READY;
        break;
    case ESP01_UDPTCP_DISCONNECTED:
        heartbeat_counter = HEARTBEAT_WIFI_READY;
        break;
    case ESP01_WIFI_DISCONNECTED:
        heartbeat_counter = HEARTBEAT_IDLE;
        break;
    }
}

void DecodeCMD(struct UNERBUSHandle *aBus, uint8_t iStartData)
{
    uint8_t id;
    uint8_t length = 0;
    uint8_t idx = 0;

    uint16_t kp_int = 0;
    uint16_t ki_int = 0;
    uint16_t kd_int = 0;

    uint16_t turn_kp_int = 0;
    uint16_t turn_ki_int = 0;
    uint16_t turn_kd_int = 0;

    id = UNERBUS_GetUInt8(aBus);
    switch ((CommandIdTypeDef)id)
    {
    case CMD_GET_LOCAL_IP_ADDRESS: // GET LOCAL IP
        UNERBUS_Write(aBus, (uint8_t *)ESP01_GetLocalIP(), IP_ADDRESS_STRING_LENGTH);
        length = UNERBUS_CMD_ID_SIZE + IP_ADDRESS_STRING_LENGTH;
        break;
    case CMD_GET_ALIVE: // ALIVE
        UNERBUS_WriteByte(aBus, CMD_ACK);
        length = UNERBUS_CMD_ID_SIZE + UNERBUS_ACK_SIZE; //
        break;
    case CMD_GET_LAST_ADC_VALUES:           // LAST_ADC - Enviar datos del ADC - 55 4E 45 52 02 3A A0 94
        uint8_t adc_buffer[ADC_DATA_BYTES]; // Buffer temporal para los datos del ADC

        // Leer del último buffer de ADC completado y seguro
        uint8_t last_adc_idx = (adc_buf_write_idx == 0) ? (ADC_BUFFER_SIZE - 1) : (adc_buf_write_idx - 1);

        // Convertir los 8 canales uint16_t a bytes (Little Endian)
        for (uint8_t i = 0; i < ADC_CHANNELS; i++)
        {
            adc_buffer[idx++] = (uint8_t)(buf_adc[last_adc_idx][i] & 0xFF);        // Byte bajo
            adc_buffer[idx++] = (uint8_t)((buf_adc[last_adc_idx][i] >> 8) & 0xFF); // Byte alto
        }

        UNERBUS_Write(aBus, adc_buffer, ADC_DATA_BYTES);
        length = UNERBUS_CMD_ID_SIZE + ADC_DATA_BYTES; // 1 (CMD) + 16 (datos)
        break;
    case CMD_CALIBRATE_MPU:               // Calibrar el MPU6050
        MPU6050_Calibrate(&hmpu, 200);    // Calibrar con 200 muestras (ajustable)
        UNERBUS_WriteByte(aBus, CMD_ACK); // Confirmar calibración
        length = UNERBUS_CMD_ID_SIZE + UNERBUS_ACK_SIZE;
        break;
    case CMD_SET_UART_BYPASS_CONTROL: // UART_BYPASS_CONTROL - Activar/desactivar bypass
        UART_BYPASS = !UART_BYPASS;
        UNERBUS_WriteByte(aBus, UART_BYPASS);
        length = UNERBUS_CMD_ID_SIZE + UNERBUS_BYPASS_STATUS_SIZE;
        break;
    case CMD_GET_MPU_DATA: // Enviar datos del MPU6050 calibrados
    {
        uint8_t mpu_buffer[MPU_RAW_DATA_SIZE]; // Buffer para datos del MPU
        int16_t ax, ay, az, gx, gy, gz;
        MPU6050_GetCalibratedData(&hmpu, &ax, &ay, &az, &gx, &gy, &gz);

        mpu_buffer[idx++] = (uint8_t)(ax & 0xFF);
        mpu_buffer[idx++] = (uint8_t)((ax >> 8) & 0xFF);
        mpu_buffer[idx++] = (uint8_t)(ay & 0xFF);
        mpu_buffer[idx++] = (uint8_t)((ay >> 8) & 0xFF);
        mpu_buffer[idx++] = (uint8_t)(az & 0xFF);
        mpu_buffer[idx++] = (uint8_t)((az >> 8) & 0xFF);
        mpu_buffer[idx++] = (uint8_t)(hmpu.raw_data.temp_raw & 0xFF);
        mpu_buffer[idx++] = (uint8_t)((hmpu.raw_data.temp_raw >> 8) & 0xFF);
        mpu_buffer[idx++] = (uint8_t)(gx & 0xFF);
        mpu_buffer[idx++] = (uint8_t)((gx >> 8) & 0xFF);
        mpu_buffer[idx++] = (uint8_t)(gy & 0xFF);
        mpu_buffer[idx++] = (uint8_t)((gy >> 8) & 0xFF);
        mpu_buffer[idx++] = (uint8_t)(gz & 0xFF);
        mpu_buffer[idx++] = (uint8_t)((gz >> 8) & 0xFF);

        UNERBUS_Write(aBus, mpu_buffer, MPU_RAW_DATA_SIZE);
        length = UNERBUS_CMD_ID_SIZE + MPU_RAW_DATA_SIZE; // 1 (CMD) + 14 (datos)
    }
    break;
    case CMD_SET_MOTOR_PWM: // Control de PWM de motores
        // Recibir 4 valores uint16_t (8 bytes) para los 4 canales PWM
        uint8_t pwm_response[UNERBUS_PWM_RESPONSE_STATUS_SIZE + PWM_DATA_BYTES]; // Buffer para respuesta (1 byte status + 8 bytes valores actuales)

        // Extraer y validar valores PWM
        for (uint8_t i = 0; i < PWM_CHANNELS; i++)
        {
            uint16_t pwm_val = UNERBUS_GetUInt16(aBus);
            if (pwm_val > pwm_max_value)
                pwm_val = pwm_max_value; // Limitar a máximo
            motor_pwm_values[i] = pwm_val;
        }

        // Aplicar los valores PWM a los canales del TIM4
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, motor_pwm_values[MOTOR_REAR_RIGHT_IDX]);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, motor_pwm_values[MOTOR_FRONT_RIGHT_IDX]);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, motor_pwm_values[MOTOR_REAR_LEFT_IDX]);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, motor_pwm_values[MOTOR_FRONT_LEFT_IDX]);

        // Preparar respuesta con estado de éxito y valores actuales
        pwm_response[0] = CMD_ACK; // Status: OK
        for (uint8_t i = 0; i < PWM_CHANNELS; i++)
        {
            pwm_response[1 + i * 2] = (uint8_t)(motor_pwm_values[i] & 0xFF);        // Byte bajo
            pwm_response[2 + i * 2] = (uint8_t)((motor_pwm_values[i] >> 8) & 0xFF); // Byte alto
        }

        UNERBUS_Write(aBus, pwm_response, UNERBUS_PWM_RESPONSE_STATUS_SIZE + PWM_DATA_BYTES);
        length = UNERBUS_CMD_ID_SIZE + UNERBUS_PWM_RESPONSE_STATUS_SIZE + PWM_DATA_BYTES; // 1 (CMD) + 9 (status + datos)
        break;
    case CMD_GET_MOTOR_PWM:                         // Obtener valores PWM actuales
        uint8_t pwm_current_buffer[PWM_DATA_BYTES]; // Buffer para valores actuales
        uint16_t idx_pwm = 0;

        // Leer los valores directamente de los registros de comparación del temporizador
        // Este es el valor real que se está aplicando a los motores.
        uint16_t right_rev = __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_1);
        uint16_t right_fwd = __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_2);
        uint16_t left_rev = __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_3);
        uint16_t left_fwd = __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_4);

        // Escribir en el buffer en el orden esperado por la HMI (Little Endian)
        pwm_current_buffer[idx_pwm++] = (uint8_t)(right_rev & 0xFF);
        pwm_current_buffer[idx_pwm++] = (uint8_t)((right_rev >> 8) & 0xFF);
        pwm_current_buffer[idx_pwm++] = (uint8_t)(right_fwd & 0xFF);
        pwm_current_buffer[idx_pwm++] = (uint8_t)((right_fwd >> 8) & 0xFF);
        pwm_current_buffer[idx_pwm++] = (uint8_t)(left_rev & 0xFF);
        pwm_current_buffer[idx_pwm++] = (uint8_t)((left_rev >> 8) & 0xFF);
        pwm_current_buffer[idx_pwm++] = (uint8_t)(left_fwd & 0xFF);
        pwm_current_buffer[idx_pwm++] = (uint8_t)((left_fwd >> 8) & 0xFF);

        UNERBUS_Write(aBus, pwm_current_buffer, PWM_DATA_BYTES);
        length = UNERBUS_CMD_ID_SIZE + PWM_DATA_BYTES; // 1 (CMD) + 8 (datos)
        break;
    case CMD_SET_PWM_PERIOD:
        uint16_t new_period = UNERBUS_GetUInt16(aBus);
        // Validar para evitar valores que puedan dañar el hardware o bloquear el timer
        if (new_period > 100 && new_period <= 65535)
        {
            pwm_max_value = new_period;
            // Actualizar el registro de auto-recarga del temporizador
            __HAL_TIM_SET_AUTORELOAD(&htim4, pwm_max_value - 1);
        }
        UNERBUS_WriteByte(aBus, CMD_ACK);
        length = UNERBUS_CMD_ID_SIZE + UNERBUS_ACK_SIZE;
        break;
    case CMD_GET_PWM_PERIOD:
        uint8_t period_buffer[UNERBUS_PWM_PERIOD_SIZE];
        period_buffer[0] = (uint8_t)(pwm_max_value & 0xFF);
        period_buffer[1] = (uint8_t)((pwm_max_value >> 8) & 0xFF);
        UNERBUS_Write(aBus, period_buffer, UNERBUS_PWM_PERIOD_SIZE);
        length = UNERBUS_CMD_ID_SIZE + UNERBUS_PWM_PERIOD_SIZE;
        break;
    case CMD_SET_PID_GAINS: // Configurar Kp, Ki, Kd
        // Se esperan 3 valores uint16_t: Kp*1000, Ki*1000, Kd*1000
        kp_int = UNERBUS_GetUInt16(aBus);
        ki_int = UNERBUS_GetUInt16(aBus);
        kd_int = UNERBUS_GetUInt16(aBus);

        // Convertir de entero a punto fijo (dividiendo por 1000.0)
        // Para evitar floats, hacemos la división en punto fijo:
        // value_fixed = (value_int * 2^16) / 1000
        wall_pid.kp = (int32_t)(((int64_t)kp_int << FIXED_POINT_SHIFT) / 1000);
        wall_pid.ki = (int32_t)(((int64_t)ki_int << FIXED_POINT_SHIFT) / 1000);
        wall_pid.kd = (int32_t)(((int64_t)kd_int << FIXED_POINT_SHIFT) / 1000);

        // Enviar confirmación (ACK)
        UNERBUS_WriteByte(aBus, CMD_ACK);
        length = UNERBUS_CMD_ID_SIZE + UNERBUS_ACK_SIZE;
        break;
    case CMD_GET_PID_GAINS: // Leer Kp, Ki, Kd
        uint8_t response_buffer[UNERBUS_PID_GAINS_SIZE];

        // Convertir de punto fijo a entero para enviar
        kp_int = (uint16_t)(((int64_t)wall_pid.kp * 1000) >> FIXED_POINT_SHIFT);
        ki_int = (uint16_t)(((int64_t)wall_pid.ki * 1000) >> FIXED_POINT_SHIFT);
        kd_int = (uint16_t)(((int64_t)wall_pid.kd * 1000) >> FIXED_POINT_SHIFT);

        response_buffer[0] = (uint8_t)(kp_int & 0xFF);
        response_buffer[1] = (uint8_t)((kp_int >> 8) & 0xFF);
        response_buffer[2] = (uint8_t)(ki_int & 0xFF);
        response_buffer[3] = (uint8_t)((ki_int >> 8) & 0xFF);
        response_buffer[4] = (uint8_t)(kd_int & 0xFF);
        response_buffer[5] = (uint8_t)((kd_int >> 8) & 0xFF);

        UNERBUS_Write(aBus, response_buffer, UNERBUS_PID_GAINS_SIZE);
        length = UNERBUS_CMD_ID_SIZE + UNERBUS_PID_GAINS_SIZE;
        break;
    case CMD_SET_CONTROL_PARAMETERS: // Configurar Setpoint, Base Speed, Max Correction
        // Se esperan 3 valores uint16_t
        wall_follow_setpoint = UNERBUS_GetUInt16(aBus);
        base_speed = UNERBUS_GetUInt16(aBus);
        max_pwm_correction = UNERBUS_GetUInt16(aBus);

        // Actualizar la configuración del PID con los nuevos valores
        PID_Set_Setpoint(&wall_pid, wall_follow_setpoint);
        PID_Set_Output_Limits(&wall_pid, INT_TO_FIXED(-max_pwm_correction), INT_TO_FIXED(max_pwm_correction));

        // Enviar confirmación (ACK)
        UNERBUS_WriteByte(aBus, CMD_ACK);
        length = UNERBUS_CMD_ID_SIZE + UNERBUS_ACK_SIZE;
        break;
    case CMD_GET_CONTROL_PARAMETERS: // Leer Setpoint, Base Speed, Max Correction
        uint8_t response_buffer_2[UNERBUS_CONTROL_PARAMS_SIZE];

        response_buffer_2[0] = (uint8_t)(wall_follow_setpoint & 0xFF);
        response_buffer_2[1] = (uint8_t)((wall_follow_setpoint >> 8) & 0xFF);
        response_buffer_2[2] = (uint8_t)(base_speed & 0xFF);
        response_buffer_2[3] = (uint8_t)((base_speed >> 8) & 0xFF);
        response_buffer_2[4] = (uint8_t)(max_pwm_correction & 0xFF);
        response_buffer_2[5] = (uint8_t)((max_pwm_correction >> 8) & 0xFF);

        UNERBUS_Write(aBus, response_buffer_2, UNERBUS_CONTROL_PARAMS_SIZE);
        length = UNERBUS_CMD_ID_SIZE + UNERBUS_CONTROL_PARAMS_SIZE;
        break;
    case CMD_SET_MOTOR_BASE_SPEEDS: // Configurar velocidades base independientes
        // Se esperan 2 valores uint16_t: Right Motor Base Speed, Left Motor Base Speed
        right_motor_base_speed = UNERBUS_GetUInt16(aBus);
        left_motor_base_speed = UNERBUS_GetUInt16(aBus);

        // Actualizar también base_speed como promedio para compatibilidad
        base_speed = (right_motor_base_speed + left_motor_base_speed) / 2;

        // Enviar confirmación (ACK)
        UNERBUS_WriteByte(aBus, CMD_ACK);
        length = UNERBUS_CMD_ID_SIZE + UNERBUS_ACK_SIZE;
        break;
    case CMD_GET_MOTOR_BASE_SPEEDS: // Leer velocidades base independientes
        uint8_t motor_speeds_buffer[UNERBUS_MOTOR_BASE_SPEEDS_SIZE];

        motor_speeds_buffer[0] = (uint8_t)(right_motor_base_speed & 0xFF);
        motor_speeds_buffer[1] = (uint8_t)((right_motor_base_speed >> 8) & 0xFF);
        motor_speeds_buffer[2] = (uint8_t)(left_motor_base_speed & 0xFF);
        motor_speeds_buffer[3] = (uint8_t)((left_motor_base_speed >> 8) & 0xFF);

        UNERBUS_Write(aBus, motor_speeds_buffer, UNERBUS_MOTOR_BASE_SPEEDS_SIZE);
        length = UNERBUS_CMD_ID_SIZE + UNERBUS_MOTOR_BASE_SPEEDS_SIZE;
        break;
    case CMD_CALIBRATE_MOTORS: // Calibración automática de motores
        // Esta función realiza una calibración automática:
        // 1. Aplica la misma velocidad PWM a ambos motores
        // 2. Usa el giroscopio para detectar deriva
        // 3. Ajusta las velocidades base para compensar
        // NOTA: Requiere que el robot esté en una superficie lisa y sin obstáculos

        // Por ahora, enviar ACK indicando que la funcionalidad está pendiente
        UNERBUS_WriteByte(aBus, CMD_ACK);
        length = UNERBUS_CMD_ID_SIZE + UNERBUS_ACK_SIZE;
        break;
    case CMD_TURN_DEGREES:
        // Recibe un ángulo de 16 bits con signo
        int16_t angle = (int16_t)UNERBUS_GetUInt16(aBus);
        Turn_Start(angle);
        UNERBUS_WriteByte(aBus, CMD_ACK);
        length = UNERBUS_CMD_ID_SIZE + UNERBUS_ACK_SIZE + UNERBUS_TURN_DEGREES_SIZE;
        break;
    case CMD_SET_TURN_PID_GAINS: // Configurar Kp, Ki, Kd del PID de giro
        // Se esperan 3 valores uint16_t: Kp*100, Ki*100, Kd*100
        turn_kp_int = UNERBUS_GetUInt16(aBus);
        turn_ki_int = UNERBUS_GetUInt16(aBus);
        turn_kd_int = UNERBUS_GetUInt16(aBus);

        // Convertir de entero a punto fijo (dividiendo por 100.0)
        turn_pid.kp = (int32_t)(((int64_t)turn_kp_int << FIXED_POINT_SHIFT) / 100);
        turn_pid.ki = (int32_t)(((int64_t)turn_ki_int << FIXED_POINT_SHIFT) / 100);
        turn_pid.kd = (int32_t)(((int64_t)turn_kd_int << FIXED_POINT_SHIFT) / 100);

        // Enviar confirmación (ACK)
        UNERBUS_WriteByte(aBus, CMD_ACK);
        length = UNERBUS_CMD_ID_SIZE + UNERBUS_ACK_SIZE;
        break;
    case CMD_GET_TURN_PID_GAINS: // Leer Kp, Ki, Kd del PID de giro
        uint8_t turn_pid_buffer[UNERBUS_TURN_PID_GAINS_SIZE];

        // Convertir de punto fijo a entero para enviar (multiplicando por 100)
        turn_kp_int = (uint16_t)(((int64_t)turn_pid.kp * 100) >> FIXED_POINT_SHIFT);
        turn_ki_int = (uint16_t)(((int64_t)turn_pid.ki * 100) >> FIXED_POINT_SHIFT);
        turn_kd_int = (uint16_t)(((int64_t)turn_pid.kd * 100) >> FIXED_POINT_SHIFT);

        turn_pid_buffer[0] = (uint8_t)(turn_kp_int & 0xFF);
        turn_pid_buffer[1] = (uint8_t)((turn_kp_int >> 8) & 0xFF);
        turn_pid_buffer[2] = (uint8_t)(turn_ki_int & 0xFF);
        turn_pid_buffer[3] = (uint8_t)((turn_ki_int >> 8) & 0xFF);
        turn_pid_buffer[4] = (uint8_t)(turn_kd_int & 0xFF);
        turn_pid_buffer[5] = (uint8_t)((turn_kd_int >> 8) & 0xFF);

        UNERBUS_Write(aBus, turn_pid_buffer, UNERBUS_TURN_PID_GAINS_SIZE);
        length = UNERBUS_CMD_ID_SIZE + UNERBUS_TURN_PID_GAINS_SIZE;
        break;
    }

    if (length)
    {
        UNERBUS_Send(aBus, id, length);
    }
}

void Do10ms()
{
    ON10MS = false;

    Button_Tick(&h_user_button);

    if (time_100ms)
        time_100ms--;

    MPU_READ_REQUEST = true;
    Update_Yaw();

    ESP01_Timeout10ms();
    UNERBUS_Timeout(&unerbus_esp01_handle);
    UNERBUS_Timeout(&unerbus_pc_handle);
}

void Do100ms()
{
    time_100ms = TIME_100MS_PEDIOD_COUNT;

    if (heartbeat_mask & heartbeat_counter)
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    else
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

    heartbeat_mask >>= 1;
    if (!heartbeat_mask)
        heartbeat_mask = 0x80000000;

    if (timeout_alive_udp)
        timeout_alive_udp--;

    /* Prueba SSD1306: mostrar yaw y estado */
    char text_line1[20];
    char text_line2[20];

    // Limpiar pantalla SSD1306
    SSD1306_Clear(&hssd);

    // Mostrar Yaw actual
    int32_t yaw_degrees = FIXED_TO_INT(current_yaw_fixed);
    snprintf(text_line1, sizeof(text_line1), "Yaw: %ld deg", yaw_degrees);
    SSD1306_DrawText(&hssd, 0, 0, text_line1, SSD1306_TEXT_ALIGN_LEFT);

    // Mostrar estado del robot
    if (turn_state == TURN_STATE_TURNING)
    {
        int32_t target_deg = FIXED_TO_INT(target_yaw_fixed);
        snprintf(text_line2, sizeof(text_line2), "Turn -> %ld", target_deg);
    }
    else if (ACTIVATE_PID)
    {
        snprintf(text_line2, sizeof(text_line2), "PID Active");
    }
    else
    {
        snprintf(text_line2, sizeof(text_line2), "Idle");
    }
    SSD1306_DrawText(&hssd, 0, 10, text_line2, SSD1306_TEXT_ALIGN_LEFT);

    // Solicitar actualización de pantalla (no bloqueante)
    SSD_UPDATE_REQUEST = true;
}

uint8_t UART_TransmitByte(uint8_t value)
{
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE))
    {
        huart1.Instance->DR = value;
        return true;
    }
    return false;
}

/**
 * @brief  Gestiona la transmisión de datos para los diferentes canales de comunicación (USB, ESP01, UART-Bypass).
 * @retval None
 */
static void ManageTransmission(void)
{
    uint8_t len;
    // TRANSMISIÓN ESP01 (solo si NO está en bypass)
    if (!UART_BYPASS && (unerbus_esp01_handle.tx.iRead != unerbus_esp01_handle.tx.iWrite))
    {
        len = (unerbus_esp01_handle.tx.iWrite - unerbus_esp01_handle.tx.iRead) & unerbus_esp01_handle.tx.maxIndexRingBuf;
        if (ESP01_Send(unerbus_esp01_handle.tx.buf, unerbus_esp01_handle.tx.iRead, len, unerbus_esp01_handle.tx.maxIndexRingBuf + 1) == ESP01_SEND_READY)
            unerbus_esp01_handle.tx.iRead = unerbus_esp01_handle.tx.iWrite;
    }

    // TRANSMISIÓN UART DIRECTO (si está en bypass)
    if (UART_BYPASS && (unerbus_pc_handle.tx.iRead != unerbus_pc_handle.tx.iWrite))
    {
        if (unerbus_pc_handle.tx.iRead < unerbus_pc_handle.tx.iWrite)
            len = unerbus_pc_handle.tx.iWrite - unerbus_pc_handle.tx.iRead;
        else
            len = unerbus_pc_handle.tx.maxIndexRingBuf + 1 - unerbus_pc_handle.tx.iRead;

        // Enviar byte por byte por UART directo
        for (uint8_t i = 0; i < len; i++)
        {
            if (UART_TransmitByte(unerbus_pc_handle.tx.buf[unerbus_pc_handle.tx.iRead]))
            {
                unerbus_pc_handle.tx.iRead = (unerbus_pc_handle.tx.iRead + 1) & unerbus_pc_handle.tx.maxIndexRingBuf;
            }
            else
            {
                break; // Si no puede transmitir, salir y reintentar en siguiente ciclo
            }
        }
    }

    if (!UART_BYPASS && (unerbus_pc_handle.tx.iRead != unerbus_pc_handle.tx.iWrite))
    {
        if (unerbus_pc_handle.tx.iRead < unerbus_pc_handle.tx.iWrite)
            len = unerbus_pc_handle.tx.iWrite - unerbus_pc_handle.tx.iRead;
        else
            len = unerbus_pc_handle.tx.maxIndexRingBuf + 1 - unerbus_pc_handle.tx.iRead;

        if (CDC_Transmit_FS(&unerbus_pc_handle.tx.buf[unerbus_pc_handle.tx.iRead], len) == USBD_OK)
        {
            unerbus_pc_handle.tx.iRead = (unerbus_pc_handle.tx.iRead + len) & unerbus_pc_handle.tx.maxIndexRingBuf;
        }
    }
}

/* I2C */
int8_t I2C_DevicesInit(void)
{
    int8_t verificacion = 0;

    hmpu.i2c_write_blocking = I2C_WriteBlocking;
    hmpu.i2c_write_dma = I2C_WriteDMA;
    hmpu.i2c_read_blocking = I2C_ReadBlocking;
    hmpu.i2c_read_dma = I2C_ReadDMA;
    hmpu.delay_ms = HAL_Delay;
    hmpu.accel_range = MPU6050_ACCEL_RANGE_2G;
    hmpu.gyro_range = MPU6050_GYRO_RANGE_250DPS;
    hmpu.dlpf_config = MPU6050_DLPF_44HZ;
    hmpu.i2c_context = &hi2c2;
    hmpu.device_address = MPU6050_ADDR;
    hmpu.is_initialized = false;
    hmpu.is_connected = false;
    MPU_READ_REQUEST = false;

    verificacion = MPU6050_Init(&hmpu);
    if (verificacion != 1)
    {
        verificacion = verificacion * (-1);
        IndicateError(verificacion, I2C_INIT_ERROR_BLINK_DELAY_MS);
        Error_Handler();
    }

    // SSD1306: Set up function pointers and context
    hssd.i2c_write_blocking = I2C_WriteBlocking;
    hssd.i2c_write_dma = I2C_WriteDMA;
    hssd.delay_ms = HAL_Delay;
    hssd.i2c_context = &hi2c2;
    hssd.device_address = 0x3C << 1; // Typical SSD1306 I2C address
    hssd.is_initialized = false;
    SSD_UPDATE_REQUEST = false;

    // SSD1306: Initialize display
    if (SSD1306_Init(&hssd) != SSD1306_OK)
    {
        IndicateError(3, 500);
        Error_Handler();
    }

    return 1;
}

int8_t I2C_WriteBlocking(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, uint16_t data_len, void *context)
{
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)context;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(hi2c, device_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data, data_len, I2C_DEFAULT_TIMEOUT_MS);
    if (status == HAL_OK)
        return 1;
    return -1;
}

int8_t I2C_WriteDMA(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, uint16_t data_len, void *context)
{
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)context;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write_DMA(hi2c, device_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data, data_len);
    if (status == HAL_OK)
        return 1;
    return -1;
}

int8_t I2C_ReadBlocking(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, uint16_t data_len, void *context)
{
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)context;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, device_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data, data_len, I2C_DEFAULT_TIMEOUT_MS);
    if (status == HAL_OK)
        return 1;
    return -1;
}

int8_t I2C_ReadDMA(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, uint16_t data_len, void *context)
{
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)context;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(hi2c, device_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data, data_len);
    if (status == HAL_OK)
        return 1;
    return -1;
}

/* Fin I2C */

static void ManageI2CTransactions(void)
{
    // Solo iniciar una nueva transacción si el bus está libre
    if (i2c_bus_state != I2C_BUS_IDLE)
    {
        return;
    }

    // Prioridad 1: Lectura del MPU6050
    if (MPU_READ_REQUEST)
    {
        MPU_READ_REQUEST = false;         // Atender la solicitud
        i2c_bus_state = I2C_BUS_BUSY_MPU; // Marcar el bus como ocupado por el MPU
        if (MPU6050_ReadRawDataDMA(&hmpu) != MPU6050_OK)
        {
            // Si falla el inicio, liberar el bus y manejar el error
            i2c_bus_state = I2C_BUS_IDLE;
            IndicateError(MPU_READ_ERROR_BLINKS, MPU_READ_ERROR_BLINK_DELAY_MS);
            Error_Handler();
        }
    }
    // Prioridad 2: Actualización del SSD1306
    else if (SSD_UPDATE_REQUEST)
    {
        SSD_UPDATE_REQUEST = false;       // Atender la solicitud
        i2c_bus_state = I2C_BUS_BUSY_SSD; // Marcar el bus como ocupado por el SSD
        if (SSD1306_UpdateScreen_DMA(&hssd) != SSD1306_OK)
        {
            // Si falla el inicio, liberar el bus y manejar el error
            i2c_bus_state = I2C_BUS_IDLE;
            IndicateError(5, 400);
            Error_Handler();
        }
    }
}

uint8_t Read_User_Button(void *context)
{
    // We ignore context for this simple case, but it's good practice to have it.
    return (uint8_t)HAL_GPIO_ReadPin(SW0_GPIO_Port, SW0_Pin);
}

static void ManageButtonEvents(void)
{
    Button_EventsTypeDef button_event = Button_GetEvent(&h_user_button);
    if (button_event != EVENT_NONE)
    {

        // A button event has occurred, handle it here.
        // For example, send a message via UNERBUS
        UNERBUS_WriteByte(&unerbus_pc_handle, (uint8_t)button_event);
        UNERBUS_Send(&unerbus_pc_handle, CMD_GET_BUTTON_STATE, UNERBUS_CMD_ID_SIZE + UNERBUS_BUTTON_EVENT_SIZE);

        switch (button_event)
        {
        case EVENT_PRESSED:
            /* code */
            break;
        case EVENT_PRESS_RELEASED:
            ACTIVATE_PID = !ACTIVATE_PID; // Toggle PID activation

            if (!ACTIVATE_PID)
            {
                // si el PID no está activado, detener los motores
                Set_Motor_Speeds(0, 0);
            }
            else
            {
                // Si el PID está activado, reiniciar el controlador PID
                PID_Reset(&wall_pid);
            }
            /* code */
            break;
        case EVENT_LONG_PRESS:
            /* code */
            break;
        case EVENT_LONG_PRESS_RELEASED:
            // Iniciar un giro de 90 grados a la derecha
            Turn_Start(90);
            break;
        default:
            break;
        }
    }
}

void IndicateError(uint8_t blinks, uint32_t delay_ms)
{
    for (uint8_t i = 0; i < blinks; i++)
    {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
        HAL_Delay(delay_ms);
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        HAL_Delay(delay_ms);
    }
}

//==============================================================================
// PUNTO DE ENTRADA DEL MÓDULO
//==============================================================================
void App_Core_Init(void)
{
    // Aquí va todo el código de inicialización que estaba en main()
    // dentro de los bloques USER CODE 1 y USER CODE 2.

    /* Heartbeat */
    heartbeat_counter = HEARTBEAT_IDLE;
    heartbeat_mask = 0x80000000;

    /* Time */
    time_10ms = TIME_10MS_PERIOD_COUNT;
    time_100ms = TIME_100MS_PEDIOD_COUNT;
    timeout_alive_udp = ALIVE_UDP_PERIOD_COUNT;

    /* ADC */
    adc_buf_write_idx = 0;
    adc_buf_read_idx = 0;

    /* ESP01 */
    esp01_handle.DoCHPD = ESP01_SetChipEnable;
    esp01_handle.WriteByteToBufRX = ESP01_WriteByteToRxBuffer;
    esp01_handle.WriteUSARTByte = ESP01_WriteUartByte;

    /* UNERBUS ESP01 */
    unerbus_esp01_handle.MyDataReady = DecodeCMD;
    unerbus_esp01_handle.WriteUSARTByte = NULL;
    unerbus_esp01_handle.rx.buf = buf_rx_esp01;
    unerbus_esp01_handle.rx.maxIndexRingBuf = (WIFI_RX_BUFFER_SIZE - 1);
    unerbus_esp01_handle.tx.buf = buf_tx_esp01;
    unerbus_esp01_handle.tx.maxIndexRingBuf = (WIFI_TX_BUFFER_SIZE - 1);

    /*UNERBUS PC*/
    unerbus_pc_handle.MyDataReady = DecodeCMD;
    unerbus_pc_handle.WriteUSARTByte = NULL;
    unerbus_pc_handle.rx.buf = buf_rx_pc;
    unerbus_pc_handle.rx.maxIndexRingBuf = (USB_CDC_RX_BUFFER_SIZE - 1);
    unerbus_pc_handle.tx.buf = buf_tx_pc;
    unerbus_pc_handle.tx.maxIndexRingBuf = (USB_CDC_TX_BUFFER_SIZE - 1);

    /* Timers */
    HAL_TIM_Base_Start_IT(&htim1);
    __HAL_TIM_SET_AUTORELOAD(&htim4, pwm_max_value - 1);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
    HAL_TIM_Base_Start(&htim4);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

    /* USB */
    CDC_AttachRxData(App_Core_USB_ReceiveData);

    /* ESP01 */
    ESP01_Init(&esp01_handle);
    ESP01_AttachChangeState(ESP01_ChangeState);
    ESP01_SetWIFI(WIFI_SSID, WIFI_PASSWORD);
    ESP01_StartUDP(WIFI_UDP_REMOTE_IP, WIFI_UDP_REMOTE_PORT, WIFI_UDP_LOCAL_PORT);

    /* UNERBUS */
    UNERBUS_Init(&unerbus_esp01_handle);
    UNERBUS_Init(&unerbus_pc_handle);

    /* --- INICIALIZACIÓN DEL PID DE SEGUIMIENTO DE PARED --- */
    PID_Init(&wall_pid, FLOAT_TO_FIXED(1), FLOAT_TO_FIXED(0), FLOAT_TO_FIXED(0));
    PID_Set_Setpoint(&wall_pid, wall_follow_setpoint);
    PID_Set_Output_Limits(&wall_pid, INT_TO_FIXED(-max_pwm_correction), INT_TO_FIXED(max_pwm_correction));

    /* --- INICIALIZACIÓN DEL PID DE GIRO --- */
    PID_Init(&turn_pid,
             FLOAT_TO_FIXED(TURN_PID_KP_DEFAULT),
             FLOAT_TO_FIXED(TURN_PID_KI_DEFAULT),
             FLOAT_TO_FIXED(TURN_PID_KD_DEFAULT));
    PID_Set_Output_Limits(&turn_pid, INT_TO_FIXED(-TURN_PID_MAX_EFFORT), INT_TO_FIXED(TURN_PID_MAX_EFFORT));

    /* Buttons*/
    Button_Init(&h_user_button, Read_User_Button, NULL);

    /* I2C devices */
    HAL_Delay(DEVICE_INIT_DELAY_MS);
    I2C_DevicesInit();
    SSD1306_UpdateScreen_DMA(&hssd);
    HAL_Delay(DEVICE_INIT_DELAY_MS);

    /* UART */
    HAL_UART_Receive_IT(&huart1, &data_rx_esp01, 1);

    /* Flags */
    ON10MS = false;
    UART_BYPASS = false;
    ACTIVATE_PID = false;
}

/**
 * @brief Bucle principal de control del robot (movimiento, sensores).
 *        Esta función se debe llamar periódicamente.
 */
void App_Core_Control_Loop(void)
{
    // 1. Leer el valor del sensor de distancia derecho
    // Se lee del buffer del ADC. Se usa el último valor completo y seguro.
    uint8_t last_adc_idx = (adc_buf_write_idx == 0) ? (ADC_BUFFER_SIZE - 1) : (adc_buf_write_idx - 1);
    uint16_t right_distance_raw = buf_adc[last_adc_idx][0]; // Canal 0 es el sensor derecho

    // Lógica del sensor: ~4000 (cerca), ~0 (lejos).
    // Error = Setpoint - ValorActual.
    // - Si está muy cerca (Valor > Setpoint), Error < 0 -> Corrección negativa.
    // - Si está muy lejos (Valor < Setpoint), Error > 0 -> Corrección positiva.

    // 2. Calcular la salida del PID
    // El tiempo dt es 10ms, que es el intervalo de llamada de Do10ms.
    int32_t pid_output_fixed = PID_Update(&wall_pid, right_distance_raw, 10);

    // Convertir la salida de punto fijo a un entero para la corrección de velocidad.
    int16_t correction = (int16_t)FIXED_TO_INT(pid_output_fixed);

    // 3. Aplicar la corrección a los motores para girar
    // - Corrección positiva (lejos): Gira a la derecha (motor izq más rápido, der más lento).
    // - Corrección negativa (cerca): Gira a la izquierda (motor der más rápido, izq más lento).
    // IMPORTANTE: Usar velocidades base independientes para compensar diferencias entre motores
    int16_t right_motor_speed = right_motor_base_speed - correction;
    int16_t left_motor_speed = left_motor_base_speed + correction;

    // 4. Limitar (saturar) la velocidad de los motores al rango válido de PWM.
    if (right_motor_speed < 0)
        right_motor_speed = 0;
    if (right_motor_speed > pwm_max_value)
        right_motor_speed = pwm_max_value;
    if (left_motor_speed < 0)
        left_motor_speed = 0;
    if (left_motor_speed > pwm_max_value)
        left_motor_speed = pwm_max_value;

    // 5. Asignar a los canales de avance de los motores
    Set_Motor_Speeds(right_motor_speed, left_motor_speed);
}

void App_Core_Loop(void)
{
    ManageButtonEvents();

    if (!timeout_alive_udp && !UART_BYPASS)
    {
        timeout_alive_udp = ALIVE_UDP_PERIOD_COUNT;
        UNERBUS_WriteByte(&unerbus_esp01_handle, CMD_ACK);
        UNERBUS_Send(&unerbus_esp01_handle, CMD_GET_ALIVE, UNERBUS_CMD_ID_SIZE + UNERBUS_ACK_SIZE);
    }

    ManageI2CTransactions();

    if (!time_100ms)
        Do100ms();

    if (ON10MS)
    {
        Do10ms();
        // El control de giro tiene prioridad sobre el seguimiento de pared
        if (turn_state == TURN_STATE_TURNING)
        {
            Manage_Turn();
        }
        else if (ACTIVATE_PID)
        {
            App_Core_Control_Loop();
        }
    }

    ManageTransmission();

    ESP01_Task();

    UNERBUS_Task(&unerbus_esp01_handle);
    UNERBUS_Task(&unerbus_pc_handle);
}

/**
 * @brief Actualiza el ángulo de Yaw integrando la velocidad del giroscopio.
 *        Se llama cada 10ms.
 */
static void Update_Yaw(void)
{
    int16_t gz;
    // Obtener solo el dato calibrado del giroscopio en Z
    MPU6050_GetCalibratedData(&hmpu, NULL, NULL, NULL, NULL, NULL, &gz);

    // Integrar para obtener el ángulo en punto fijo (Q16.16)
    // El escalador convierte el valor raw del giroscopio a un cambio de ángulo en grados (formato Q16.16) para un dt de 10ms.
    current_yaw_fixed -= (int32_t)gz * GYRO_Z_SCALER;
}

/**
 * @brief Inicia un giro de un ángulo específico en grados.
 * @param angle_degrees Ángulo de giro. Positivo para la derecha, negativo para la izquierda.
 */
void Turn_Start(int16_t angle_degrees)
{
    if (turn_state == TURN_STATE_IDLE)
    {
        // Desactivar el PID de seguimiento de pared durante el giro
        ACTIVATE_PID = false;
        Set_Motor_Speeds(0, 0); // Detener motores antes de girar

        // Resetear el PID de giro y establecer el nuevo ángulo objetivo
        PID_Reset(&turn_pid);
        target_yaw_fixed = current_yaw_fixed + INT_TO_FIXED(angle_degrees);
        PID_Set_Setpoint(&turn_pid, FIXED_TO_INT(target_yaw_fixed)); // El setpoint del PID es un entero
        turn_state = TURN_STATE_TURNING;
    }
}

/**
 * @brief Gestiona el estado de giro del robot usando un controlador PID.
 *        Debe ser llamada periódicamente mientras se está girando.
 */
static void Manage_Turn(void)
{
    if (turn_state != TURN_STATE_TURNING)
    {
        return;
    }

    // El error se calcula en grados enteros para simplicidad
    int32_t current_yaw_degrees = FIXED_TO_INT(current_yaw_fixed);
    int32_t target_yaw_degrees = FIXED_TO_INT(target_yaw_fixed);
    int32_t error_degrees = target_yaw_degrees - current_yaw_degrees;

    // Comprobar si el giro ha terminado
    if (abs(error_degrees) <= TURN_COMPLETION_DEAD_ZONE)
    {
        Set_Motor_Speeds(0, 0);
        turn_state = TURN_STATE_IDLE;
        return;
    }

    // 1. Calcular la salida del PID. La función `PID_Update` espera un valor entero.
    //    El dt es 10ms, que es el período de llamada de esta función.
    int32_t pid_output_fixed = PID_Update(&turn_pid, current_yaw_degrees, 10);

    // 2. Convertir la salida del PID a un "esfuerzo de giro" entero.
    int16_t turn_effort = (int16_t)FIXED_TO_INT(pid_output_fixed);

    // 3. Escalar el esfuerzo a los PWM de cada motor usando sus velocidades base calibradas.
    //    Un esfuerzo máximo (TURN_PID_MAX_EFFORT) corresponde a girar usando las velocidades base.
    //    Se usa int32_t para evitar desbordes en la multiplicación intermedia.
    int16_t right_pwm = ((int32_t)turn_effort * right_motor_base_speed) / TURN_PID_MAX_EFFORT;
    int16_t left_pwm = ((int32_t)turn_effort * left_motor_base_speed) / TURN_PID_MAX_EFFORT;

    // 4. Aplicar velocidades a los motores para girar en el sitio.
    //    Si el esfuerzo es positivo (error > 0), se gira a la derecha.
    //    Giro a la derecha: motor izquierdo adelante (PWM positivo), motor derecho atrás (PWM negativo).
    //    Aquí, el esfuerzo ya tiene signo, así que lo aplicamos directamente.
    //    Para girar a la derecha (esfuerzo > 0), el motor derecho debe ir hacia atrás (-right_pwm) y el izquierdo hacia adelante (+left_pwm).
    Set_Motor_Speeds(-right_pwm, left_pwm);
}

/**
 * @brief Establece la velocidad de los motores derecho e izquierdo.
 * @param right_speed Velocidad del motor derecho. Positivo=adelante, Negativo=atrás.
 * @param left_speed Velocidad del motor izquierdo. Positivo=adelante, Negativo=atrás.
 */
static void Set_Motor_Speeds(int16_t right_speed, int16_t left_speed)
{
    uint16_t right_fwd = 0, right_rev = 0, left_fwd = 0, left_rev = 0;

    // Lógica para motor derecho
    if (right_speed > 0)
    {
        right_fwd = (right_speed > pwm_max_value) ? pwm_max_value : right_speed;
    }
    else
    {
        right_rev = (-right_speed > pwm_max_value) ? pwm_max_value : -right_speed;
    }

    // Lógica para motor izquierdo
    if (left_speed > 0)
    {
        left_fwd = (left_speed > pwm_max_value) ? pwm_max_value : left_speed;
    }
    else
    {
        left_rev = (-left_speed > pwm_max_value) ? pwm_max_value : -left_speed;
    }

    // Motor derecho: ch2 adelante (TIM4_CH2), ch1 atrás (TIM4_CH1)
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, right_fwd);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, right_rev);

    // Motor izquierdo: ch4 adelante (TIM4_CH4), ch3 atrás (TIM4_CH3)
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, left_fwd);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, left_rev);
}
