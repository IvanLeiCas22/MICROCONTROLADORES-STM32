/* STM32/Test2024-master/Core/Src/app_core.c */
#include "app_core.h"
#include "app_config.h"

#include <stdlib.h>
#include <stdio.h> // Para snprintf
#include <string.h> // Para memset

#include "usbd_cdc_if.h"
#include "ESP01.h"
#include "UNERBUS.h"
#include "MPU6050.h"
#include "BUTTONS.h"
#include "SSD1306.h"
#include "pid_controller.h"

/* --- Smooth Turn --- */
#define GYRO_SENSITIVITY FLOAT_TO_FIXED(65.5f)

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
// Typedefs y Estructuras
//==============================================================================

// Tabla de consulta (LUT) para linealización de sensores IR
typedef struct
{
    uint16_t adc;
    uint16_t dist_mm;
} SensorLutEntry;

static const SensorLutEntry sensor_lut[] = {
    {30, 150}, {65, 140}, {106, 130}, {135, 120}, {169, 110}, {208, 100}, {260, 90}, {337, 80}, {441, 70}, {511, 65}, {590, 60}, {711, 55}, {827, 50}, {1020, 45}, {1305, 40}, {1613, 35}, {2130, 30}, {2870, 25}, {3760, 20}};

enum
{
    ADC_LUT_SIZE = (int)(sizeof(sensor_lut) / sizeof(sensor_lut[0]))
};
enum
{
    ADC_SEG_COUNT = ADC_LUT_SIZE - 1
};
typedef struct
{
    uint16_t x0;       // Valor ADC de inicio del segmento
    uint16_t x1;       // Valor ADC de fin del segmento
    uint16_t y0;       // Distancia en mm correspondiente a x0
    int32_t slope_q16; // Pendiente (y1 - y0) / (x1 - x0) en formato Q16.16
} ADC_LutSeg_t;

// Coordenadas actuales del robot
typedef struct {
    uint8_t x;
    uint8_t y;
    HeadingTypeDef heading; // Dirección a la que mira: NORTH, SOUTH, EAST, WEST
} RobotPosition_t;

// Fila = Heading (N, E, S, W)
// Columna = Sensor (Frente, Derecha, Izquierda)
const uint8_t wall_lut[4][3] = {
    // FTE           DER           IZQ
    {WALL_NORTH, WALL_EAST,  WALL_WEST }, // HEADING_NORTH (0)
    {WALL_EAST,  WALL_SOUTH, WALL_NORTH}, // HEADING_EAST  (1)
    {WALL_SOUTH, WALL_WEST,  WALL_EAST }, // HEADING_SOUTH (2)
    {WALL_WEST,  WALL_NORTH, WALL_SOUTH}  // HEADING_WEST  (3)
};

//==============================================================================
// VARIABLES GLOBALES DEL MÓDULO
//==============================================================================

static ADC_LutSeg_t adc_lut_segs[ADC_SEG_COUNT];

SystemFlagTypeDef flags0;
uint16_t pwm_max_value = 10000; // Valor máximo del PWM

_sESP01Handle esp01_handle;
_sUNERBUSHandle unerbus_pc_handle;
_sUNERBUSHandle unerbus_esp01_handle;

char local_ip[IP_ADDRESS_STRING_LENGTH];
uint8_t buf_rx_pc[USB_CDC_RX_BUFFER_SIZE], buf_tx_pc[USB_CDC_TX_BUFFER_SIZE];
uint8_t buf_rx_esp01[WIFI_RX_BUFFER_SIZE], buf_tx_esp01[WIFI_TX_BUFFER_SIZE], data_rx_esp01;

uint32_t heartbeat_counter, heartbeat_mask;
uint8_t time_10ms, time_100ms, timeout_alive_udp;

uint16_t buf_adc[ADC_BUFFER_SIZE][ADC_CHANNELS];
volatile uint8_t adc_buf_write_idx = 0;
static uint32_t adc_running_sum[ADC_CHANNELS] = {0};
static volatile uint16_t adc_filtered_avg[ADC_CHANNELS] = {0};
static uint8_t adc_samples_accumulated = 0;
static volatile uint8_t adc_processed_idx = 0;

static MPU6050_HandleTypeDef hmpu;
static SSD1306_HandleTypeDef hssd;
Button_HandleTypeDef h_user_button;
uint16_t motor_pwm_values[PWM_CHANNELS] = {0, 0, 0, 0};
static volatile I2C_BusStateTypeDef i2c_bus_state = I2C_BUS_IDLE;

// --- Variables de Estado de la Aplicación ---
static AppStateTypeDef app_state = APP_STATE_MENU;
static MenuModeTypeDef menu_mode = MENU_MODE_IDLE;
static uint32_t temporary_heartbeat = 0;
static uint8_t temporary_heartbeat_ticks = 0;

// --- Variables de PID y Control del Robot ---
PID_Controller_t centering_pid;
PID_Controller_t turn_pid;
PID_Controller_t turn_velocity_pid;
PID_Controller_t braking_pid;
uint16_t right_motor_base_speed = 3575;         // Velocidad base motor derecho
uint16_t left_motor_base_speed = 4550;          // Velocidad base motor izquierdo
uint16_t faster_motor_smooth_turn_speed = 6000; // Velocidad del motor más rápido en giro suave
uint16_t slower_motor_smooth_turn_speed = 2500; // Velocidad del motor más lento en giro suave
uint16_t wall_threshold_mm_front = 70;          // Umbral en mm para detectar pared frontal
uint16_t tape_detection_threshold_adc = 1500;   // Umbral en ADC para detectar cinta
uint16_t wall_threshold_mm_braking_start = 40;  // Umbral en mm para iniciar el frenado
uint16_t wall_threshold_mm_diagonal = 130;      // Umbral en mm para detectar pared diagonal
uint16_t wall_threshold_mm_side = 100;          // Umbral en mm para detectar pared lateral
uint16_t after_turn_wall_threshold_mm = 80;     // Umbral en mm para pared después de un giro
uint16_t wall_target_mm = 55;                   // Distancia objetivo en mm para seguimiento de pared
uint16_t wall_braking_target_mm = 30;           // Distancia de parada objetivo
uint16_t braking_accel_stop_threshold = 2000;   // Umbral de aceleración para confirmar detención
uint16_t max_pwm_correction = 4000;             // Corrección máxima del PID
uint16_t turn_max_pwm = TURN_MAX_SPEED_DEFAULT;
uint16_t pivot_turn_target_dps = PIVOT_TURN_TARGET_DPS_DEFAULT;
uint16_t turn_target_dps = TURN_TARGET_DPS_DEFAULT;
uint16_t braking_max_pwm_offset = BRAKING_MAX_SPEED_DEFAULT; // PWM máximo de frenado
uint16_t braking_min_speed = BRAKING_MIN_SPEED_DEFAULT;
uint16_t braking_dead_zone = BRAKING_DEAD_ZONE_DEFAULT;

bool left_wall_detected = false, right_wall_detected = false, front_wall_detected = false,
     left_diagonal_wall_detected = false, right_diagonal_wall_detected = false, rear_tape_detected = false,
     front_tape_detected = false, was_rear_tape_detected = false;
uint16_t dist_diagonal_left_mm = 0;
uint16_t dist_diagonal_right_mm = 0;
uint16_t dist_front_left_mm = 0;
uint16_t dist_front_right_mm = 0;
uint16_t dist_left_lat_mm = 0;
uint16_t dist_right_lat_mm = 0;
uint16_t adc_rear_floor = 0;
uint16_t adc_front_floor = 0;
uint8_t wall_fade_counter = 0;
uint8_t wall_fade_ticks = WALL_FADE_TICKS_DEFAULT;

static int32_t current_yaw_fixed = 0; // Yaw angle in Q16.16 fixed-point (degrees)
static int32_t gyro_z_scaler;         // Factor de escala dinámico para el giroscopio

static volatile RobotStateTypeDef robot_state = STATE_IDLE;
uint16_t motor_kick_start_speed;
uint16_t accel_motion_threshold;
uint8_t accel_motion_confirm_ticks;
static bool kick_start_active = false;
static uint8_t motion_confirm_counter = 0;

/* --- Laberinto --- */
// El mapa completo. Todo el laberinto entra en 225 Bytes de RAM
uint8_t maze_map[MAZE_WIDTH][MAZE_HEIGHT];
RobotPosition_t current_pos;

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

static void Update_Gyro_Scaler(void);
static void Set_Motor_Speeds(int16_t right_speed, int16_t left_speed);

static void Handle_Idle(void);
static void Reset_Robot_Position(void);
static void Reset_Maze_State(void);
static void Current_Cell_Mapping(void);
static void Send_Maze_Cell_Update(void);
static void Update_Robot_Heading(int8_t turn_direction);
static void Handle_Navigating(void);
static void Handle_Braking(void);
static void Handle_Deciding();
static void Manage_Turn(void);
static void Update_Yaw(void);
void Turn_Start(int16_t angle_degrees);
static void ADC_Filter_Task(void);
static void ADC_LUT_Precompute(void);
static int32_t Get_Filtered_ADC_Value(uint8_t channel);
static void Set_Robot_State(RobotStateTypeDef new_state);
static void Update_Display_Content(void);
static int32_t ADC_To_Distance_mm(uint16_t adc_value);
static void Handle_Straight_Drive(bool have_to_decide);
static void Modes_State_Machine(void);

//==============================================================================
// IMPLEMENTACIÓN DE WRAPPERS DE CALLBACKS HAL
//==============================================================================
void App_Core_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static uint8_t MPU_READ_TICKER = MPU_READ_PERIOD_COUNT;
    if (htim->Instance == TIM1)
    {
        time_10ms--;
        if (!time_10ms)
        {
            ON10MS = true;
            time_10ms = TIME_10MS_PERIOD_COUNT;
        }
        HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&buf_adc[adc_buf_write_idx], ADC_CHANNELS);

        MPU_READ_TICKER--;
        if (!MPU_READ_TICKER)
        {
            MPU_READ_TICKER = MPU_READ_PERIOD_COUNT;
            MPU_READ_REQUEST = true; // Indicar que se debe leer el MPU
        }
    }
}

void App_Core_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    adc_buf_write_idx = (uint8_t)((adc_buf_write_idx + 1) & ADC_BUF_MASK);
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

    uint16_t vel_kp_int = 0;
    uint16_t vel_ki_int = 0;
    uint16_t vel_kd_int = 0;

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
    case CMD_GET_LAST_ADC_VALUES:           // LAST_ADC - Enviar datos del ADC
        uint8_t adc_buffer[ADC_DATA_BYTES]; // Buffer temporal para los datos del ADC

        // Obtener el valor filtrado (promedio móvil) para cada canal
        for (uint8_t i = 0; i < ADC_CHANNELS; i++)
        {
            // Llamar a la función de filtrado para el canal actual
            uint16_t filtered_value = (uint16_t)Get_Filtered_ADC_Value(i);
            uint16_t distance_value = ADC_To_Distance_mm(filtered_value);

            // Convertir el valor uint16_t a bytes (Little Endian)
            adc_buffer[idx++] = (uint8_t)(distance_value & 0xFF);        // Byte bajo
            adc_buffer[idx++] = (uint8_t)((distance_value >> 8) & 0xFF); // Byte alto
        }

        UNERBUS_Write(aBus, adc_buffer, ADC_DATA_BYTES);
        length = UNERBUS_CMD_ID_SIZE + ADC_DATA_BYTES; // 1 (CMD) + 16 (datos)
        break;
    case CMD_CALIBRATE_MPU:            // Calibrar el MPU6050
        MPU6050_Calibrate(&hmpu, 200); // Calibrar con 200 muestras (ajustable)
                                       /*         UNERBUS_WriteByte(aBus, CMD_ACK); // Confirmar calibración
                                               length = UNERBUS_CMD_ID_SIZE + UNERBUS_ACK_SIZE; */
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
        break;
    case CMD_GET_PWM_PERIOD:
        uint8_t period_buffer[UNERBUS_PWM_PERIOD_SIZE];
        period_buffer[0] = (uint8_t)(pwm_max_value & 0xFF);
        period_buffer[1] = (uint8_t)((pwm_max_value >> 8) & 0xFF);
        UNERBUS_Write(aBus, period_buffer, UNERBUS_PWM_PERIOD_SIZE);
        length = UNERBUS_CMD_ID_SIZE + UNERBUS_PWM_PERIOD_SIZE;
        break;
    case CMD_SET_MPU_CONFIG:
    {
        uint8_t new_accel_range = UNERBUS_GetUInt8(aBus);
        uint8_t new_gyro_range = UNERBUS_GetUInt8(aBus);
        uint8_t new_dlpf_config = UNERBUS_GetUInt8(aBus);

        // --- Validación de los datos recibidos (Sugerencia implementada) ---
        // Se comprueba que los valores estén dentro de los rangos permitidos por el MPU6050.h
        bool is_valid = (new_accel_range == MPU6050_ACCEL_RANGE_2G || new_accel_range == MPU6050_ACCEL_RANGE_4G || new_accel_range == MPU6050_ACCEL_RANGE_8G || new_accel_range == MPU6050_ACCEL_RANGE_16G) &&
                        (new_gyro_range == MPU6050_GYRO_RANGE_250DPS || new_gyro_range == MPU6050_GYRO_RANGE_500DPS || new_gyro_range == MPU6050_GYRO_RANGE_1000DPS || new_gyro_range == MPU6050_GYRO_RANGE_2000DPS) &&
                        (new_dlpf_config <= MPU6050_DLPF_5HZ);

        if (is_valid)
        {
            hmpu.accel_range = new_accel_range;
            hmpu.gyro_range = new_gyro_range;
            hmpu.dlpf_config = new_dlpf_config;

            // Re-inicializar el MPU para aplicar la nueva configuración
            MPU6050_Init(&hmpu);

            // Actualizar el escalador del giroscopio con la nueva configuración
            Update_Gyro_Scaler();
        }
    }
    break;
    case CMD_GET_MPU_CONFIG:
        uint8_t mpu_config_buffer[UNERBUS_MPU_CONFIG_SIZE];
        mpu_config_buffer[0] = hmpu.accel_range;
        mpu_config_buffer[1] = hmpu.gyro_range;
        mpu_config_buffer[2] = hmpu.dlpf_config;
        UNERBUS_Write(aBus, mpu_config_buffer, UNERBUS_MPU_CONFIG_SIZE);
        length = UNERBUS_CMD_ID_SIZE + UNERBUS_MPU_CONFIG_SIZE;
        break;
    case CMD_SET_PID_GAINS: // Configurar Kp, Ki, Kd
        // Se esperan 3 valores uint16_t: Kp*100, Ki*100, Kd*100
        kp_int = UNERBUS_GetUInt16(aBus);
        ki_int = UNERBUS_GetUInt16(aBus);
        kd_int = UNERBUS_GetUInt16(aBus);

        // Convertir de entero a punto fijo (dividiendo por 100.0)
        // Se usa 100 para ampliar el rango de Kp hasta ~655
        centering_pid.kp = (int32_t)(((int64_t)kp_int << FIXED_POINT_SHIFT) / 100);
        centering_pid.ki = (int32_t)(((int64_t)ki_int << FIXED_POINT_SHIFT) / 100);
        centering_pid.kd = (int32_t)(((int64_t)kd_int << FIXED_POINT_SHIFT) / 100);
        break;
    case CMD_GET_PID_GAINS: // Leer Kp, Ki, Kd
        uint8_t response_buffer[UNERBUS_PID_GAINS_SIZE];

        // Convertir de punto fijo a entero para enviar (multiplicando por 100)
        kp_int = (uint16_t)(((int64_t)centering_pid.kp * 100) >> FIXED_POINT_SHIFT);
        ki_int = (uint16_t)(((int64_t)centering_pid.ki * 100) >> FIXED_POINT_SHIFT);
        kd_int = (uint16_t)(((int64_t)centering_pid.kd * 100) >> FIXED_POINT_SHIFT);

        response_buffer[0] = (uint8_t)(kp_int & 0xFF);
        response_buffer[1] = (uint8_t)((kp_int >> 8) & 0xFF);
        response_buffer[2] = (uint8_t)(ki_int & 0xFF);
        response_buffer[3] = (uint8_t)((ki_int >> 8) & 0xFF);
        response_buffer[4] = (uint8_t)(kd_int & 0xFF);
        response_buffer[5] = (uint8_t)((kd_int >> 8) & 0xFF);

        UNERBUS_Write(aBus, response_buffer, UNERBUS_PID_GAINS_SIZE);
        length = UNERBUS_CMD_ID_SIZE + UNERBUS_PID_GAINS_SIZE;
        break;
    case CMD_SET_MAX_PWM_CORRECTION: // Configurar la corrección máxima del PWM
        // Se espera 1 valor uint16_t
        max_pwm_correction = UNERBUS_GetUInt16(aBus);

        // Actualizar la configuración del PID con los nuevos valores
        PID_Set_Output_Limits(&centering_pid, INT_TO_FIXED(-max_pwm_correction), INT_TO_FIXED(max_pwm_correction));
        break;
    case CMD_GET_MAX_PWM_CORRECTION: // Leer la corrección máxima del PWM
        uint8_t response_buffer_2[UNERBUS_CONTROL_PARAMS_SIZE];

        response_buffer_2[0] = (uint8_t)(max_pwm_correction & 0xFF);
        response_buffer_2[1] = (uint8_t)((max_pwm_correction >> 8) & 0xFF);

        UNERBUS_Write(aBus, response_buffer_2, UNERBUS_CONTROL_PARAMS_SIZE);
        length = UNERBUS_CMD_ID_SIZE + UNERBUS_CONTROL_PARAMS_SIZE;
        break;
    case CMD_SET_MOTOR_BASE_SPEEDS: // Configurar velocidades base independientes
        // Se esperan 2 valores uint16_t: Right Motor Base Speed, Left Motor Base Speed
        right_motor_base_speed = UNERBUS_GetUInt16(aBus);
        left_motor_base_speed = UNERBUS_GetUInt16(aBus);
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
    case CMD_SET_TURN_MAX_SPEED:
        turn_max_pwm = UNERBUS_GetUInt16(aBus);
        if (turn_max_pwm > pwm_max_value)
            turn_max_pwm = pwm_max_value; // Limitar al máximo global
        PID_Set_Output_Limits(&turn_pid, INT_TO_FIXED(-turn_max_pwm), INT_TO_FIXED(turn_max_pwm));
        break;
    case CMD_GET_TURN_MAX_SPEED:
        uint8_t speed_buffer[UNERBUS_TURN_MAX_SPEED_SIZE];
        speed_buffer[0] = (uint8_t)(turn_max_pwm & 0xFF);
        speed_buffer[1] = (uint8_t)((turn_max_pwm >> 8) & 0xFF);
        UNERBUS_Write(aBus, speed_buffer, UNERBUS_TURN_MAX_SPEED_SIZE);
        length = UNERBUS_CMD_ID_SIZE + UNERBUS_TURN_MAX_SPEED_SIZE;
        break;
    case CMD_SET_PIVOT_TURN_DPS:
        pivot_turn_target_dps = UNERBUS_GetUInt16(aBus);
        if (pivot_turn_target_dps > turn_max_pwm)
            pivot_turn_target_dps = turn_max_pwm; // No puede ser mayor que la máxima
        break;
    case CMD_GET_PIVOT_TURN_DPS:
        uint8_t min_speed_buffer[UNERBUS_TURN_MIN_SPEED_SIZE];
        min_speed_buffer[0] = (uint8_t)(pivot_turn_target_dps & 0xFF);
        min_speed_buffer[1] = (uint8_t)((pivot_turn_target_dps >> 8) & 0xFF);
        UNERBUS_Write(aBus, min_speed_buffer, UNERBUS_TURN_MIN_SPEED_SIZE);
        length = UNERBUS_CMD_ID_SIZE + UNERBUS_TURN_MIN_SPEED_SIZE;
        break;
    case CMD_SET_WALL_THRESHOLDS:
        wall_threshold_mm_front = UNERBUS_GetUInt16(aBus);
        wall_threshold_mm_side = UNERBUS_GetUInt16(aBus);
        wall_threshold_mm_diagonal = UNERBUS_GetUInt16(aBus);
        after_turn_wall_threshold_mm = UNERBUS_GetUInt16(aBus);
        break;
    case CMD_GET_WALL_THRESHOLDS:
        uint8_t thresholds_buffer[UNERBUS_WALL_THRESHOLDS_SIZE];
        thresholds_buffer[0] = (uint8_t)(wall_threshold_mm_front & 0xFF);
        thresholds_buffer[1] = (uint8_t)((wall_threshold_mm_front >> 8) & 0xFF);
        thresholds_buffer[2] = (uint8_t)(wall_threshold_mm_side & 0xFF);
        thresholds_buffer[3] = (uint8_t)((wall_threshold_mm_side >> 8) & 0xFF);
        thresholds_buffer[4] = (uint8_t)(wall_threshold_mm_diagonal & 0xFF);
        thresholds_buffer[5] = (uint8_t)((wall_threshold_mm_diagonal >> 8) & 0xFF);
        thresholds_buffer[6] = (uint8_t)(after_turn_wall_threshold_mm & 0xFF);
        thresholds_buffer[7] = (uint8_t)((after_turn_wall_threshold_mm >> 8) & 0xFF);
        UNERBUS_Write(aBus, thresholds_buffer, UNERBUS_WALL_THRESHOLDS_SIZE);
        length = UNERBUS_CMD_ID_SIZE + UNERBUS_WALL_THRESHOLDS_SIZE;
        break;
    case CMD_SET_WALL_TARGET_ADC:
        wall_target_mm = UNERBUS_GetUInt16(aBus);
        tape_detection_threshold_adc = UNERBUS_GetUInt16(aBus);
        break;
    case CMD_GET_WALL_TARGET_ADC:
        uint8_t target_buffer[UNERBUS_WALL_TARGET_ADC_SIZE];
        target_buffer[0] = (uint8_t)(wall_target_mm & 0xFF);
        target_buffer[1] = (uint8_t)((wall_target_mm >> 8) & 0xFF);
        target_buffer[2] = (uint8_t)(tape_detection_threshold_adc & 0xFF);
        target_buffer[3] = (uint8_t)((tape_detection_threshold_adc >> 8) & 0xFF);
        UNERBUS_Write(aBus, target_buffer, UNERBUS_WALL_TARGET_ADC_SIZE);
        length = UNERBUS_CMD_ID_SIZE + UNERBUS_WALL_TARGET_ADC_SIZE;
        break;
    case CMD_SET_APP_STATE:
        AppStateTypeDef new_state = (AppStateTypeDef)UNERBUS_GetUInt8(aBus);
        if (new_state == APP_STATE_RUNNING && app_state == APP_STATE_MENU)
        {
            // Transición de MENU a RUNNING
            app_state = APP_STATE_RUNNING;
            // Resetear PIDs y Yaw para un inicio limpio
            PID_Reset(&centering_pid);
            PID_Reset(&turn_pid);
            current_yaw_fixed = 0;
            // Iniciar la máquina de estados del robot si el modo es activo.
            // Esto replica el comportamiento del botón físico.
            if (menu_mode == MENU_MODE_FIND_CELLS || menu_mode == MENU_MODE_GO_TO_B)
            {
                Set_Robot_State(STATE_NAVIGATING); // Aquí se inicia el movimiento
                kick_start_active = true;
                motion_confirm_counter = 0;
                
                if (menu_mode == MENU_MODE_FIND_CELLS) {
                    Reset_Maze_State();
                } else if (menu_mode == MENU_MODE_GO_TO_B) {
                    Reset_Robot_Position();
                }
            }
            else
            {
                Set_Robot_State(STATE_IDLE); // Para modos que no inician movimiento
            }
        }
        else if (new_state == APP_STATE_MENU && app_state == APP_STATE_RUNNING)
        {
            // Transición de RUNNING a MENU
            app_state = APP_STATE_MENU;
            Set_Motor_Speeds(0, 0); // Detener motores por seguridad
            Set_Robot_State(STATE_IDLE);
        }
        Update_Display_Content();
        SSD_UPDATE_REQUEST = true;
        break;
    case CMD_GET_APP_STATE:
        UNERBUS_WriteByte(aBus, (uint8_t)app_state);
        length = UNERBUS_CMD_ID_SIZE + UNERBUS_APP_STATE_SIZE;
        break;
    case CMD_SET_MENU_MODE:
        menu_mode = (MenuModeTypeDef)UNERBUS_GetUInt8(aBus);
        Update_Display_Content();
        SSD_UPDATE_REQUEST = true;
        break;
    case CMD_GET_MENU_MODE:
        UNERBUS_WriteByte(aBus, (uint8_t)menu_mode);
        length = UNERBUS_CMD_ID_SIZE + UNERBUS_MENU_MODE_SIZE;
        break;
    case CMD_GET_ROBOT_STATUS:
        uint8_t status_buffer[UNERBUS_ROBOT_STATUS_SIZE];
        status_buffer[0] = (uint8_t)app_state;
        status_buffer[1] = (uint8_t)menu_mode;
        UNERBUS_Write(aBus, status_buffer, UNERBUS_ROBOT_STATUS_SIZE);
        length = UNERBUS_CMD_ID_SIZE + UNERBUS_ROBOT_STATUS_SIZE;
        break;
    case CMD_SET_CRUISE_PARAMS:
        motor_kick_start_speed = UNERBUS_GetUInt16(aBus);
        accel_motion_threshold = UNERBUS_GetUInt16(aBus);
        // Se recibe como u16 para alinear el paquete, pero se usa como u8.
        accel_motion_confirm_ticks = (uint8_t)UNERBUS_GetUInt16(aBus);
        break;
    case CMD_GET_CRUISE_PARAMS:
        uint8_t cruise_buffer[UNERBUS_CRUISE_PARAMS_SIZE];
        cruise_buffer[0] = (uint8_t)(motor_kick_start_speed & 0xFF);
        cruise_buffer[1] = (uint8_t)((motor_kick_start_speed >> 8) & 0xFF);
        cruise_buffer[2] = (uint8_t)(accel_motion_threshold & 0xFF);
        cruise_buffer[3] = (uint8_t)((accel_motion_threshold >> 8) & 0xFF);
        cruise_buffer[4] = (uint8_t)(accel_motion_confirm_ticks);
        cruise_buffer[5] = 0; // Padding para alinear a 16 bits
        UNERBUS_Write(aBus, cruise_buffer, UNERBUS_CRUISE_PARAMS_SIZE);
        length = UNERBUS_CMD_ID_SIZE + UNERBUS_CRUISE_PARAMS_SIZE;
        break;
    case CMD_SET_BRAKING_PID_GAINS:
        kp_int = UNERBUS_GetUInt16(aBus);
        ki_int = UNERBUS_GetUInt16(aBus);
        kd_int = UNERBUS_GetUInt16(aBus);
        // Se usa 100 para ampliar el rango de Kp hasta ~655
        braking_pid.kp = (int32_t)(((int64_t)kp_int << FIXED_POINT_SHIFT) / 100);
        braking_pid.ki = (int32_t)(((int64_t)ki_int << FIXED_POINT_SHIFT) / 100);
        braking_pid.kd = (int32_t)(((int64_t)kd_int << FIXED_POINT_SHIFT) / 100);
        break;
    case CMD_GET_BRAKING_PID_GAINS:
        uint8_t braking_pid_buffer[UNERBUS_BRAKING_PID_GAINS_SIZE];
        // Se multiplica por 100 para coincidir con el SET
        kp_int = (uint16_t)(((int64_t)braking_pid.kp * 100) >> FIXED_POINT_SHIFT);
        ki_int = (uint16_t)(((int64_t)braking_pid.ki * 100) >> FIXED_POINT_SHIFT);
        kd_int = (uint16_t)(((int64_t)braking_pid.kd * 100) >> FIXED_POINT_SHIFT);
        braking_pid_buffer[0] = (uint8_t)(kp_int & 0xFF);
        braking_pid_buffer[1] = (uint8_t)((kp_int >> 8) & 0xFF);
        braking_pid_buffer[2] = (uint8_t)(ki_int & 0xFF);
        braking_pid_buffer[3] = (uint8_t)((ki_int >> 8) & 0xFF);
        braking_pid_buffer[4] = (uint8_t)(kd_int & 0xFF);
        braking_pid_buffer[5] = (uint8_t)((kd_int >> 8) & 0xFF);
        UNERBUS_Write(aBus, braking_pid_buffer, UNERBUS_BRAKING_PID_GAINS_SIZE);
        length = UNERBUS_CMD_ID_SIZE + UNERBUS_BRAKING_PID_GAINS_SIZE;
        break;
    case CMD_SET_BRAKING_PARAMS:
        wall_braking_target_mm = UNERBUS_GetUInt16(aBus);
        braking_accel_stop_threshold = UNERBUS_GetUInt16(aBus);
        PID_Set_Setpoint(&braking_pid, wall_braking_target_mm);
        break;
    case CMD_GET_BRAKING_PARAMS:
        uint8_t braking_params_buffer[UNERBUS_BRAKING_PARAMS_SIZE];
        braking_params_buffer[0] = (uint8_t)(wall_braking_target_mm & 0xFF);
        braking_params_buffer[1] = (uint8_t)((wall_braking_target_mm >> 8) & 0xFF);
        braking_params_buffer[2] = (uint8_t)(braking_accel_stop_threshold & 0xFF);
        braking_params_buffer[3] = (uint8_t)((braking_accel_stop_threshold >> 8) & 0xFF);
        UNERBUS_Write(aBus, braking_params_buffer, UNERBUS_BRAKING_PARAMS_SIZE);
        length = UNERBUS_CMD_ID_SIZE + UNERBUS_BRAKING_PARAMS_SIZE;
        break;
    case CMD_SET_BRAKING_MAX_SPEED:
        braking_max_pwm_offset = UNERBUS_GetUInt16(aBus);
        PID_Set_Output_Limits(&braking_pid, INT_TO_FIXED(-braking_max_pwm_offset), INT_TO_FIXED(braking_max_pwm_offset));
        break;
    case CMD_GET_BRAKING_MAX_SPEED:
        uint8_t braking_speed_buffer[UNERBUS_BRAKING_MAX_SPEED_SIZE];
        braking_speed_buffer[0] = (uint8_t)(braking_max_pwm_offset & 0xFF);
        braking_speed_buffer[1] = (uint8_t)((braking_max_pwm_offset >> 8) & 0xFF);
        UNERBUS_Write(aBus, braking_speed_buffer, UNERBUS_BRAKING_MAX_SPEED_SIZE);
        length = UNERBUS_CMD_ID_SIZE + UNERBUS_BRAKING_MAX_SPEED_SIZE;
        break;
    case CMD_SET_BRAKING_MIN_SPEED:
        braking_min_speed = UNERBUS_GetUInt16(aBus);
        break;
    case CMD_GET_BRAKING_MIN_SPEED:
        uint8_t braking_min_speed_buffer[UNERBUS_BRAKING_MIN_SPEED_SIZE];
        braking_min_speed_buffer[0] = (uint8_t)(braking_min_speed & 0xFF);
        braking_min_speed_buffer[1] = (uint8_t)((braking_min_speed >> 8) & 0xFF);
        UNERBUS_Write(aBus, braking_min_speed_buffer, UNERBUS_BRAKING_MIN_SPEED_SIZE);
        length = UNERBUS_CMD_ID_SIZE + UNERBUS_BRAKING_MIN_SPEED_SIZE;
        break;
    case CMD_SET_BRAKING_DEAD_ZONE:
        braking_dead_zone = UNERBUS_GetUInt16(aBus);
        break;
    case CMD_GET_BRAKING_DEAD_ZONE:
        uint8_t braking_dead_zone_buffer[UNERBUS_BRAKING_DEAD_ZONE_SIZE];
        braking_dead_zone_buffer[0] = (uint8_t)(braking_dead_zone & 0xFF);
        braking_dead_zone_buffer[1] = (uint8_t)((braking_dead_zone >> 8) & 0xFF);
        UNERBUS_Write(aBus, braking_dead_zone_buffer, UNERBUS_BRAKING_DEAD_ZONE_SIZE);
        length = UNERBUS_CMD_ID_SIZE + UNERBUS_BRAKING_DEAD_ZONE_SIZE;
        break;
    case CMD_GET_YAW_ANGLE:
        uint8_t yaw_buffer[UNERBUS_YAW_ANGLE_SIZE];
        int32_t yaw_angle = FIXED_TO_INT(current_yaw_fixed);
        yaw_buffer[0] = (uint8_t)(yaw_angle & 0xFF);
        yaw_buffer[1] = (uint8_t)((yaw_angle >> 8) & 0xFF);
        yaw_buffer[2] = (uint8_t)((yaw_angle >> 16) & 0xFF);
        yaw_buffer[3] = (uint8_t)((yaw_angle >> 24) & 0xFF);
        UNERBUS_Write(aBus, yaw_buffer, UNERBUS_YAW_ANGLE_SIZE);
        length = UNERBUS_CMD_ID_SIZE + UNERBUS_YAW_ANGLE_SIZE;
        break;
    case CMD_GET_SMOOTH_TURN_CONFIG:
        uint8_t smooth_turn_config_buffer[UNERBUS_SMOOTH_TURN_CONFIG_SIZE];

        smooth_turn_config_buffer[0] = (uint8_t)(faster_motor_smooth_turn_speed & 0xFF);
        smooth_turn_config_buffer[1] = (uint8_t)((faster_motor_smooth_turn_speed >> 8) & 0xFF);
        smooth_turn_config_buffer[2] = (uint8_t)(slower_motor_smooth_turn_speed & 0xFF);
        smooth_turn_config_buffer[3] = (uint8_t)((slower_motor_smooth_turn_speed >> 8) & 0xFF);

        UNERBUS_Write(aBus, smooth_turn_config_buffer, UNERBUS_SMOOTH_TURN_CONFIG_SIZE);
        length = UNERBUS_CMD_ID_SIZE + UNERBUS_SMOOTH_TURN_CONFIG_SIZE;
        break;
    case CMD_SET_SMOOTH_TURN_CONFIG:
        faster_motor_smooth_turn_speed = UNERBUS_GetUInt16(aBus);
        slower_motor_smooth_turn_speed = UNERBUS_GetUInt16(aBus);

        if (faster_motor_smooth_turn_speed > (pwm_max_value - 1))
            faster_motor_smooth_turn_speed = (pwm_max_value - 1); // Limitar al máximo global

        if (slower_motor_smooth_turn_speed > (pwm_max_value - 1))
            slower_motor_smooth_turn_speed = (pwm_max_value - 1); // Limitar al máximo global
        break;
    case CMD_SET_TURN_VELOCITY_PID_GAINS:
        vel_kp_int = UNERBUS_GetUInt16(aBus);
        vel_ki_int = UNERBUS_GetUInt16(aBus);
        vel_kd_int = UNERBUS_GetUInt16(aBus);
        turn_velocity_pid.kp = (int32_t)(((int64_t)vel_kp_int << FIXED_POINT_SHIFT) / 100);
        turn_velocity_pid.ki = (int32_t)(((int64_t)vel_ki_int << FIXED_POINT_SHIFT) / 100);
        turn_velocity_pid.kd = (int32_t)(((int64_t)vel_kd_int << FIXED_POINT_SHIFT) / 100);
        break;
    case CMD_GET_TURN_VELOCITY_PID_GAINS:
        uint8_t vel_pid_buffer[UNERBUS_TURN_VELOCITY_PID_GAINS_SIZE];
        vel_kp_int = (uint16_t)(((int64_t)turn_velocity_pid.kp * 100) >> FIXED_POINT_SHIFT);
        vel_ki_int = (uint16_t)(((int64_t)turn_velocity_pid.ki * 100) >> FIXED_POINT_SHIFT);
        vel_kd_int = (uint16_t)(((int64_t)turn_velocity_pid.kd * 100) >> FIXED_POINT_SHIFT);
        vel_pid_buffer[0] = (uint8_t)(vel_kp_int & 0xFF);
        vel_pid_buffer[1] = (uint8_t)((vel_kp_int >> 8) & 0xFF);
        vel_pid_buffer[2] = (uint8_t)(vel_ki_int & 0xFF);
        vel_pid_buffer[3] = (uint8_t)((vel_ki_int >> 8) & 0xFF);
        vel_pid_buffer[4] = (uint8_t)(vel_kd_int & 0xFF);
        vel_pid_buffer[5] = (uint8_t)((vel_kd_int >> 8) & 0xFF);
        UNERBUS_Write(aBus, vel_pid_buffer, UNERBUS_TURN_VELOCITY_PID_GAINS_SIZE);
        length = UNERBUS_CMD_ID_SIZE + UNERBUS_TURN_VELOCITY_PID_GAINS_SIZE;
        break;
    case CMD_SET_TURN_TARGET_DPS:
        turn_target_dps = UNERBUS_GetUInt16(aBus);
        break;
    case CMD_GET_TURN_TARGET_DPS:
        uint8_t dps_buffer[UNERBUS_TURN_TARGET_DPS_SIZE];
        dps_buffer[0] = (uint8_t)(turn_target_dps & 0xFF);
        dps_buffer[1] = (uint8_t)((turn_target_dps >> 8) & 0xFF);
        UNERBUS_Write(aBus, dps_buffer, UNERBUS_TURN_TARGET_DPS_SIZE);
        length = UNERBUS_CMD_ID_SIZE + UNERBUS_TURN_TARGET_DPS_SIZE;
        break;
    case CMD_GET_DELAY_TICKS:
        uint8_t delays_buffer[UNERBUS_DELAY_TICKS_SIZE];

        delays_buffer[0] = wall_fade_ticks;
        UNERBUS_Write(aBus, delays_buffer, UNERBUS_DELAY_TICKS_SIZE);
        length = UNERBUS_CMD_ID_SIZE + UNERBUS_DELAY_TICKS_SIZE;
        break;
    case CMD_SET_DELAY_TICKS:
        wall_fade_ticks = UNERBUS_GetUInt8(aBus);
        break;
    default:
        // Comando desconocido, enviar ACK de error
        /*         UNERBUS_WriteByte(aBus, CMD_NACK);
                length = UNERBUS_CMD_ID_SIZE + UNERBUS_NACK_SIZE; */
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

    Update_Yaw();

    ESP01_Timeout10ms();
    UNERBUS_Timeout(&unerbus_esp01_handle);
    UNERBUS_Timeout(&unerbus_pc_handle);
}

void Do100ms()
{
    time_100ms = TIME_100MS_PEDIOD_COUNT;

    // --- Lógica de Heartbeat Dinámico ---
    if (temporary_heartbeat_ticks > 0)
    {
        temporary_heartbeat_ticks--;
        heartbeat_counter = temporary_heartbeat;
    }
    else
    {
        if (app_state == APP_STATE_MENU)
        {
            switch (menu_mode)
            {
            case MENU_MODE_IDLE:
                heartbeat_counter = HEARTBEAT_MENU_IDLE;
                break;
            case MENU_MODE_FIND_CELLS:
                heartbeat_counter = HEARTBEAT_MENU_FIND_CELLS;
                break;
            case MENU_MODE_GO_TO_B:
                heartbeat_counter = HEARTBEAT_MENU_GO_TO_B;
                break;
            case MENU_MODE_MANUAL_CONTROL:
                heartbeat_counter = HEARTBEAT_MENU_MANUAL_CONTROL;
                break;
            default:
                heartbeat_counter = HEARTBEAT_IDLE;
                break;
            }
        }
        else // APP_STATE_RUNNING
        {
            switch (menu_mode)
            {
            case MENU_MODE_IDLE:
                heartbeat_counter = HEARTBEAT_RUNNING_IDLE;
                break;
            case MENU_MODE_FIND_CELLS:
                heartbeat_counter = HEARTBEAT_RUNNING_FIND_CELLS;
                break;
            case MENU_MODE_GO_TO_B:
                heartbeat_counter = HEARTBEAT_RUNNING_GO_TO_B;
                break;
            case MENU_MODE_MANUAL_CONTROL:
                heartbeat_counter = HEARTBEAT_RUNNING_MANUAL_CONTROL;
                break;
            default:
                heartbeat_counter = HEARTBEAT_IDLE;
                break;
            }
        }
    }

    if (heartbeat_mask & heartbeat_counter)
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    else
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

    heartbeat_mask >>= 1;
    if (!heartbeat_mask)
        heartbeat_mask = 0x80000000;

    if (timeout_alive_udp)
        timeout_alive_udp--;
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
    hmpu.gyro_range = MPU6050_GYRO_RANGE_500DPS;
    hmpu.dlpf_config = MPU6050_DLPF_260HZ;
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
        // Enviar evento por UNERBUS para debug
        UNERBUS_WriteByte(&unerbus_pc_handle, (uint8_t)button_event);
        UNERBUS_Send(&unerbus_pc_handle, CMD_GET_BUTTON_STATE, UNERBUS_CMD_ID_SIZE + UNERBUS_BUTTON_EVENT_SIZE);

        if (app_state == APP_STATE_MENU)
        {
            switch (button_event)
            {
            case EVENT_PRESS_RELEASED: // Pulsación corta: ciclar menú
                menu_mode = (MenuModeTypeDef)((menu_mode + 1) % MENU_MODE_COUNT);
                temporary_heartbeat = HEARTBEAT_BTN_SHORT_PRESS;
                temporary_heartbeat_ticks = 5; // Duración del feedback (5 * 100ms = 0.5s)
                Update_Display_Content();
                SSD_UPDATE_REQUEST = true;
                break;
            case EVENT_LONG_PRESS_RELEASED: // Pulsación larga: seleccionar y correr
                app_state = APP_STATE_RUNNING;
                temporary_heartbeat = HEARTBEAT_BTN_LONG_PRESS;
                temporary_heartbeat_ticks = 10; // Duración del feedback (10 * 100ms = 1s)
                // Resetear PIDs y estados al iniciar un modo
                PID_Reset(&centering_pid);
                PID_Reset(&turn_pid);
                PID_Reset(&braking_pid);
                Set_Robot_State((menu_mode == MENU_MODE_FIND_CELLS) ? STATE_NAVIGATING : STATE_IDLE);
                if (robot_state == STATE_NAVIGATING)
                {
                    kick_start_active = true;
                    motion_confirm_counter = 0;
                    
                    if (menu_mode == MENU_MODE_FIND_CELLS) {
                        Reset_Maze_State();
                    } else if (menu_mode == MENU_MODE_GO_TO_B) {
                        Reset_Robot_Position();
                    }
                    
                    // Mapeo primera celda buscando paredes
                    Current_Cell_Mapping();
                    // Publicamos el estado de la celda mapeada para sincronizar la HMI
                    Send_Maze_Cell_Update();
                }
                if (menu_mode == MENU_MODE_DRIVE_STRAIGHT)
                {
                    Set_Robot_State(STATE_STRAIGHT_DRIVE);
                    PID_Reset(&centering_pid);
                    PID_Set_Setpoint(&centering_pid, FIXED_TO_INT(current_yaw_fixed));
                }
                break;
            default:
                break;
            }
        }
        else // APP_STATE_RUNNING
        {
            // En modo ejecución, una pulsación larga detiene y vuelve al menú
            if (button_event == EVENT_LONG_PRESS_RELEASED)
            {
                app_state = APP_STATE_MENU;
                Set_Robot_State(STATE_IDLE);
                Set_Motor_Speeds(0, 0); // Detener motores
                temporary_heartbeat = HEARTBEAT_BTN_LONG_PRESS;
                temporary_heartbeat_ticks = 10;
                // La llamada a Set_Robot_State ya activa la flag,
                // pero el cambio de app_state también requiere actualizar el display.
                Update_Display_Content();
            }
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

    /* --- INICIALIZACIÓN DE PARÁMETROS DE CRUCERO --- */
    motor_kick_start_speed = MOTOR_KICK_START_SPEED_DEFAULT;
    accel_motion_threshold = ACCEL_MOTION_THRESHOLD_DEFAULT;
    accel_motion_confirm_ticks = ACCEL_MOTION_CONFIRM_TICKS_DEFAULT;

    /* --- INICIALIZACIÓN DE PARÁMETROS DE NAVEGACIÓN --- */
    wall_threshold_mm_front = WALL_PRESENCE_THRESHOLD_MM_FRONT;
    wall_threshold_mm_diagonal = WALL_PRESENCE_THRESHOLD_MM_DIAGONAL;
    wall_threshold_mm_side = WALL_PRESENCE_THRESHOLD_MM_SIDE;
    wall_threshold_mm_braking_start = WALL_PRESENCE_THRESHOLD_MM_BRAKING_START;
    wall_target_mm = WALL_FOLLOW_TARGET_MM;
    wall_braking_target_mm = WALL_BRAKING_TARGET_MM;

    /* --- INICIALIZACIÓN DEL PID DE SEGUIMIENTO DE PARED --- */
    PID_Init(&centering_pid, FLOAT_TO_FIXED(0.8f), FLOAT_TO_FIXED(0.0f), FLOAT_TO_FIXED(0.2f)); // Kp, Ki, Kd
    PID_Set_Setpoint(&centering_pid, 0);                                                        // El setpoint se ajustará dinámicamente
    PID_Set_Output_Limits(&centering_pid, INT_TO_FIXED(-max_pwm_correction), INT_TO_FIXED(max_pwm_correction));

    /* --- INICIALIZACIÓN DEL PID DE FRENADO --- */
    braking_max_pwm_offset = BRAKING_MAX_SPEED_DEFAULT;
    PID_Init(&braking_pid,
             FLOAT_TO_FIXED(BRAKING_PID_KP_DEFAULT),
             FLOAT_TO_FIXED(BRAKING_PID_KI_DEFAULT),
             FLOAT_TO_FIXED(BRAKING_PID_KD_DEFAULT));
    PID_Set_Setpoint(&braking_pid, wall_braking_target_mm);
    // La salida es la velocidad, así que el límite es el PWM máximo.
    PID_Set_Output_Limits(&braking_pid, INT_TO_FIXED(-braking_max_pwm_offset), INT_TO_FIXED(braking_max_pwm_offset));

    /* --- INICIALIZACIÓN DEL PID DE GIRO --- */
    PID_Init(&turn_pid,
             FLOAT_TO_FIXED(TURN_PID_KP_DEFAULT),
             FLOAT_TO_FIXED(TURN_PID_KI_DEFAULT),
             FLOAT_TO_FIXED(TURN_PID_KD_DEFAULT));
    PID_Set_Output_Limits(&turn_pid, INT_TO_FIXED(-turn_max_pwm), INT_TO_FIXED(turn_max_pwm));

    /* --- NUEVO: INICIALIZACIÓN DEL PID DE VELOCIDAD DE GIRO --- */
    PID_Init(&turn_velocity_pid,
             FLOAT_TO_FIXED(TURN_VELOCITY_PID_KP_DEFAULT),
             FLOAT_TO_FIXED(TURN_VELOCITY_PID_KI_DEFAULT),
             FLOAT_TO_FIXED(TURN_VELOCITY_PID_KD_DEFAULT));
    // La salida de este PID ES la potencia del motor, así que sus límites son los límites de PWM.
    PID_Set_Output_Limits(&turn_velocity_pid, INT_TO_FIXED(-turn_max_pwm), INT_TO_FIXED(turn_max_pwm));
    PID_Set_Setpoint(&turn_velocity_pid, turn_target_dps); // Establecer el setpoint inicial

    srand(1); // Inicializa la semilla para rand()

    /* Buttons*/
    Button_Init(&h_user_button, Read_User_Button, NULL);

    /* I2C devices */
    HAL_Delay(DEVICE_INIT_DELAY_MS);
    I2C_DevicesInit();
    // Calcular el escalador inicial del giroscopio basado en la configuración por defecto
    Update_Gyro_Scaler();
    SSD1306_UpdateScreen_DMA(&hssd);
    HAL_Delay(DEVICE_INIT_DELAY_MS);
    ADC_LUT_Precompute();

    /* UART */
    HAL_UART_Receive_IT(&huart1, &data_rx_esp01, 1);

    /* Flags */
    ON10MS = false;
    UART_BYPASS = false;

    /* Estados de la aplicación */
    app_state = APP_STATE_MENU;
    menu_mode = MENU_MODE_IDLE;
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

    ADC_Filter_Task();

    if (ON10MS)
    {
        Do10ms();
        Modes_State_Machine();
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
    if (abs(gz) > 500)
    {
        current_yaw_fixed -= (int32_t)gz * gyro_z_scaler;
    }
}

/**
 * @brief Inicia un giro de un ángulo específico en grados.
 * @param angle_degrees Ángulo de giro. Positivo para la derecha, negativo para la izquierda.
 */
void Turn_Start(int16_t angle_degrees)
{
    if ((robot_state == STATE_NAVIGATING || robot_state == STATE_DECIDING) ||
        (robot_state == STATE_IDLE && menu_mode == MENU_MODE_MANUAL_CONTROL))
    {
        // Reseteamos el PID que usaremos para el control de velocidad y el ángulo acumulado.
        PID_Reset(&turn_pid);
        current_yaw_fixed = 0; // Reseteamos la medición de ángulo para un giro relativo.

        // Asignar el estado de giro correcto
        if (angle_degrees == 90)
        {
            Set_Robot_State(STATE_TURNING_RIGHT);
        }
        else if (angle_degrees == -90)
        {
            Set_Robot_State(STATE_TURNING_LEFT);
        }
        else if (angle_degrees == 180)
        {
            Set_Robot_State(STATE_TURN_AROUND_RIGHT);
        }
        else if (angle_degrees == -180)
        {
            Set_Robot_State(STATE_TURN_AROUND_LEFT);
        }
    }
}

/**
 * @brief Gestiona el estado de giro del robot usando un controlador PID.
 *        Utiliza una potencia de giro independiente y compensación mecánica.
 */
/* static void Manage_Turn(void)
{
    if (robot_state != STATE_TURNING_LEFT && robot_state != STATE_TURNING_RIGHT && robot_state != STATE_TURN_AROUND)
    {
        return;
    }

    int32_t current_yaw_degrees = FIXED_TO_INT(current_yaw_fixed);
    int32_t target_yaw_degrees = FIXED_TO_INT(turn_pid.setpoint);
    int32_t error_degrees = target_yaw_degrees - current_yaw_degrees;

    // Comprobar si el giro ha terminado
    if (abs(error_degrees) <= TURN_COMPLETION_DEAD_ZONE)
    {
        Set_Motor_Speeds(0, 0);
        if (menu_mode == MENU_MODE_MANUAL_CONTROL)
        {
            Set_Robot_State(STATE_IDLE);
        }
        else
        {
            Set_Robot_State(STATE_NAVIGATING);
            PID_Reset(&centering_pid);
            PID_Reset(&braking_pid);
            kick_start_active = true;
            motion_confirm_counter = 0;
        }
        return;
    }

    // 1. Calcular la salida del PID. Está limitada por `turn_max_pwm`.
    int32_t pid_output_fixed = PID_Update(&turn_pid, current_yaw_degrees, 10);
    int16_t correction_pwm = (int16_t)FIXED_TO_INT(pid_output_fixed);

    // 2. Calcular un ratio de giro de [-1.0, 1.0]
    int32_t turn_ratio_fixed = 0;
    if (turn_max_pwm != 0)
    {
        turn_ratio_fixed = FIXED_DIV(INT_TO_FIXED(correction_pwm), INT_TO_FIXED(turn_max_pwm));
    }

    // 3. Aplicar la potencia de giro MANTENIENDO LA COMPENSACIÓN MECÁNICA
    int16_t left_speed, right_speed;

    // Se asume que las velocidades base están calibradas para ir recto.
    // El ratio entre ellas nos da el factor de compensación.
    if (left_motor_base_speed > right_motor_base_speed)
    {
        // El motor izquierdo es el de referencia (el más rápido).
        int32_t compensation_ratio = FIXED_DIV(INT_TO_FIXED(right_motor_base_speed), INT_TO_FIXED(left_motor_base_speed));

        // La velocidad del motor de referencia se escala directamente con la potencia de giro.
        int32_t base_left_speed_fixed = FIXED_MUL(INT_TO_FIXED(turn_max_pwm), turn_ratio_fixed);
        // La velocidad del motor más lento se compensa.
        int32_t base_right_speed_fixed = FIXED_MUL(base_left_speed_fixed, compensation_ratio);

        left_speed = (int16_t)FIXED_TO_INT(base_left_speed_fixed);
        right_speed = -(int16_t)FIXED_TO_INT(base_right_speed_fixed);
    }
    else // El motor derecho es más rápido o son iguales
    {
        // El motor derecho es el de referencia.
        int32_t compensation_ratio = FIXED_DIV(INT_TO_FIXED(left_motor_base_speed), INT_TO_FIXED(right_motor_base_speed));

        int32_t base_right_speed_fixed = FIXED_MUL(INT_TO_FIXED(turn_max_pwm), turn_ratio_fixed);
        int32_t base_left_speed_fixed = FIXED_MUL(base_right_speed_fixed, compensation_ratio);

        left_speed = (int16_t)FIXED_TO_INT(base_left_speed_fixed);
        right_speed = -(int16_t)FIXED_TO_INT(base_right_speed_fixed);
    }

    // 4. Aplicar la velocidad mínima para vencer la inercia
    if (right_speed > 0 && right_speed < pivot_turn_target_dps)
        right_speed = pivot_turn_target_dps;
    else if (right_speed < 0 && right_speed > -pivot_turn_target_dps)
        right_speed = -pivot_turn_target_dps;

    if (left_speed > 0 && left_speed < pivot_turn_target_dps)
        left_speed = pivot_turn_target_dps;
    else if (left_speed < 0 && left_speed > -pivot_turn_target_dps)
        left_speed = -pivot_turn_target_dps;

    // 5. Aplicar las velocidades calculadas a los motores.
    Set_Motor_Speeds(right_speed, left_speed);
} */

// Actualiza la orientación luego de un giro. 
// turn_direction: 1 (Giro Derecha), -1 (Giro Izquierda), 2 (Media Vuelta)
static void Update_Robot_Heading(int8_t turn_direction) {
    // Sumamos 4 antes de aplicar módulo 4 para evitar problemas matemáticos con números negativos en C
    current_pos.heading = (HeadingTypeDef)((current_pos.heading + turn_direction + 4) % 4);
}

/**
 * @brief Gestiona el estado de giro del robot usando un controlador PID de velocidad angular.
 *        Este método utiliza 'turn_pid' para alcanzar y mantener una velocidad angular objetivo
 *        durante los giros pivote.
 */
static void Manage_Turn(void)
{
    int32_t current_yaw_degrees = FIXED_TO_INT(current_yaw_fixed);
    int16_t target_yaw_degrees = 0;
    int16_t target_dps = 0;

    // 1. Determinar el ángulo objetivo (para la condición de parada) y la velocidad angular
    //    objetivo (para el setpoint del PID) basándose en el estado de giro.
    switch (robot_state)
    {
    case STATE_TURNING_LEFT:
        target_yaw_degrees = -90;
        target_dps = (int16_t)pivot_turn_target_dps; // Un 'gz' positivo es giro a la izquierda.
        break;
    case STATE_TURNING_RIGHT:
        target_yaw_degrees = 90;
        target_dps = -((int16_t)pivot_turn_target_dps); // Un 'gz' negativo es giro a la derecha.
        break;
    case STATE_TURN_AROUND_LEFT:
        target_yaw_degrees = -180;
        target_dps = (int16_t)pivot_turn_target_dps;
        break;
    case STATE_TURN_AROUND_RIGHT:
        target_yaw_degrees = 180;
        target_dps = -((int16_t)pivot_turn_target_dps);
        break;
    default: // Estado inesperado
        Set_Motor_Speeds(0, 0);
        return;
    }

    target_dps = ((int32_t)target_dps * (int32_t)((((abs(target_yaw_degrees) - abs(current_yaw_degrees)) * (int16_t)100) / abs(target_yaw_degrees)) + (int16_t)40)) / 100;

    // 2. Comprobar si el giro ha terminado (condición de parada basada en el ángulo total girado).
    if (abs(current_yaw_degrees) >= (abs(target_yaw_degrees) - TURN_COMPLETION_DEAD_ZONE))
    {
        Set_Motor_Speeds(0, 0);

        // El pivote se usa para giros de 180 grados: actualizamos heading y
        // publicamos la pose para que la HMI refleje el giro sin esperar avance.
        Update_Robot_Heading(TURN_AROUND);
        Send_Maze_Cell_Update();

        if (menu_mode == MENU_MODE_MANUAL_CONTROL)
        {
            Set_Robot_State(STATE_IDLE);
        }
        else
        {
            // Después de un giro, volvemos a navegar.
            Set_Robot_State(STATE_NAVIGATING);
            PID_Reset(&centering_pid);
            PID_Reset(&braking_pid);
            kick_start_active = true;
            motion_confirm_counter = 0;
        }
        return;
    }

    // --- Lógica del PID de velocidad angular ---

    // 3. Obtener la velocidad angular actual del giroscopio.
    int16_t gz;
    MPU6050_GetCalibratedData(&hmpu, NULL, NULL, NULL, NULL, NULL, &gz);

    // Convertir el valor raw del giroscopio (gz) a grados por segundo (dps).
    int32_t angular_velocity_fixed = FIXED_DIV(INT_TO_FIXED(gz), GYRO_SENSITIVITY);
    int16_t angular_velocity_dps = (int16_t)FIXED_TO_INT(angular_velocity_fixed);

    // 4. Establecer el setpoint del PID de giro a la velocidad angular deseada.
    PID_Set_Setpoint(&turn_pid, target_dps);

    // 5. Calcular la salida del PID. La entrada es la velocidad angular actual.
    //    La salida es la "fuerza" de giro (un valor de PWM).
    int32_t pid_output_fixed = PID_Update(&turn_pid, angular_velocity_dps, 10);
    int16_t correction_pwm = (int16_t)FIXED_TO_INT(pid_output_fixed);

    // 7. Aplicar las velocidades calculadas a los motores.
    Set_Motor_Speeds(correction_pwm, -correction_pwm);
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
        right_fwd = (right_speed > (pwm_max_value - 1)) ? (pwm_max_value - 1) : right_speed;
    }
    else
    {
        right_rev = (-right_speed > (pwm_max_value - 1)) ? (pwm_max_value - 1) : -right_speed;
    }

    // Lógica para motor izquierdo
    if (left_speed > 0)
    {
        left_fwd = (left_speed > (pwm_max_value - 1)) ? (pwm_max_value - 1) : left_speed;
    }
    else
    {
        left_rev = (-left_speed > (pwm_max_value - 1)) ? (pwm_max_value - 1) : -left_speed;
    }

    // Motor derecho: ch2 adelante (TIM4_CH2), ch1 atrás (TIM4_CH1)
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, right_fwd);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, right_rev);

    // Motor izquierdo: ch4 adelante (TIM4_CH4), ch3 atrás (TIM4_CH3)
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, left_fwd);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, left_rev);
}

/**
 * @brief Actualiza el factor de escala del giroscopio según la configuración actual.
 *        Este factor se usa para convertir el valor raw del giroscopio a grados/s.
 */
static void Update_Gyro_Scaler(void)
{
    // La fórmula es: (dt_s * (1 << 16)) / LSB_per_dps
    // dt_s = 0.01s (10ms), (1 << 16) = 65536
    // El resultado es (655.36 / LSB_per_dps)
    switch (hmpu.gyro_range)
    {
    case MPU6050_GYRO_RANGE_250DPS:
        // LSB/dps = 131
        gyro_z_scaler = 5; // 655.36 / 131 = 4.995...
        break;
    case MPU6050_GYRO_RANGE_500DPS:
        // LSB/dps = 65.5
        gyro_z_scaler = 10; // 655.36 / 65.5 = 9.99...
        break;
    case MPU6050_GYRO_RANGE_1000DPS:
        // LSB/dps = 32.8
        gyro_z_scaler = 20; // 655.36 / 32.8 = 19.98...
        break;
    case MPU6050_GYRO_RANGE_2000DPS:
        // LSB/dps = 16.4
        gyro_z_scaler = 40; // 655.36 / 16.4 = 39.96...
        break;
    default:
        // Caso por defecto seguro
        gyro_z_scaler = 10;
        break;
    }
}

static void Handle_Idle(void)
{
    Set_Motor_Speeds(0, 0); // Asegurarse de que los motores estén parados
}

/**
 * @brief Procesa las muestras pendientes del ring buffer y actualiza el promedio móvil por canal.
 *        Ejecutar frecuentemente en el lazo principal.
 */
static void ADC_Filter_Task(void)
{
    // Procesar todas las muestras pendientes entre adc_processed_idx y adc_buf_write_idx
    while (adc_processed_idx != adc_buf_write_idx)
    {
        uint8_t idx_new = adc_processed_idx;
        bool window_full = (adc_samples_accumulated >= ADC_MOVING_AVERAGE_SAMPLES);

        if (!window_full)
        {
            // La ventana aún no está llena, solo agregamos y promediamos sobre el total actual
            adc_samples_accumulated++;
            for (uint8_t ch = 0; ch < ADC_CHANNELS; ch++)
            {
                adc_running_sum[ch] += buf_adc[idx_new][ch];
                adc_filtered_avg[ch] = (uint16_t)(adc_running_sum[ch] / adc_samples_accumulated);
            }
        }
        else // La ventana está llena, usamos el método eficiente de suma/resta
        {
            uint8_t idx_old = (uint8_t)((idx_new - ADC_MOVING_AVERAGE_SAMPLES) & ADC_BUF_MASK);
            for (uint8_t ch = 0; ch < ADC_CHANNELS; ch++)
            {
                uint16_t new_val = buf_adc[idx_new][ch];
                uint16_t old_val = buf_adc[idx_old][ch];

                // Se previene el underflow en la resta. Si el valor a restar es anómalamente
                // más grande que la suma acumulada (debido a un pico de ruido), la resta
                // de enteros sin signo causaría un 'wrap-around' a un número muy grande.
                // Para evitarlo, se resetea la suma. El filtro se recuperará en las siguientes muestras.
                if (old_val > adc_running_sum[ch])
                {
                    adc_running_sum[ch] = 0;
                }
                else
                {
                    adc_running_sum[ch] -= old_val;
                }

                adc_running_sum[ch] += new_val;

                adc_filtered_avg[ch] = (uint16_t)(adc_running_sum[ch] >> ADC_FILTER_SHIFT);
            }
        }

        adc_processed_idx = (uint8_t)((adc_processed_idx + 1) & ADC_BUF_MASK);
    }
}

/**
 * @brief Devuelve el promedio móvil precalculado del canal.
 */
static int32_t Get_Filtered_ADC_Value(uint8_t channel)
{
    if (channel >= ADC_CHANNELS)
        return 0;
    return (int32_t)adc_filtered_avg[channel];
}

// Actualiza la posición (x, y) asumiendo que el robot avanzó 1 celda hacia el frente
static void Update_Robot_Position(void) {
    switch (current_pos.heading) {
        case HEADING_NORTH:
            if (current_pos.y < MAZE_HEIGHT - 1) current_pos.y++;
            break;
        case HEADING_EAST:
            if (current_pos.x < MAZE_WIDTH - 1) current_pos.x++;
            break;
        case HEADING_SOUTH:
            if (current_pos.y > 0) current_pos.y--;
            break;
        case HEADING_WEST:
            if (current_pos.x > 0) current_pos.x--;
            break;
    }
}

static void Current_Cell_Mapping(void) {
    uint8_t cell_data = CELL_VISITED;
    
    // Indexamos directamente el array en O(1)
    if(front_wall_detected) cell_data |= wall_lut[current_pos.heading][0];
    if(right_wall_detected) cell_data |= wall_lut[current_pos.heading][1];
    if(left_wall_detected)  cell_data |= wall_lut[current_pos.heading][2];
    
    // Volcamos al mapa la celda actual
    maze_map[current_pos.x][current_pos.y] |= cell_data;
    
    // --- LÓGICA AUXILIAR PARA CELDAS VECINAS ---
    // IMPORTANTE: Asume que "Norte" es Y creciente (y+1) y "Este" es X creciente (x+1).
    // Las condicionales evitan desbordar el arreglo de memoria (ej. x < MAZE_WIDTH-1).
    
    if ((cell_data & WALL_NORTH) && (current_pos.y < MAZE_HEIGHT - 1)) {
        maze_map[current_pos.x][current_pos.y + 1] |= WALL_SOUTH;
    }
    if ((cell_data & WALL_SOUTH) && (current_pos.y > 0)) {
        maze_map[current_pos.x][current_pos.y - 1] |= WALL_NORTH;
    }
    if ((cell_data & WALL_EAST) && (current_pos.x < MAZE_WIDTH - 1)) {
        maze_map[current_pos.x + 1][current_pos.y] |= WALL_WEST;
    }
    if ((cell_data & WALL_WEST) && (current_pos.x > 0)) {
        maze_map[current_pos.x - 1][current_pos.y] |= WALL_EAST;
    }
}

static void Send_Maze_Cell_Update(void) {
    uint8_t cell_payload[UNERBUS_MAZE_CELL_UPDATE_SIZE] = {
        current_pos.x,
        current_pos.y,
        maze_map[current_pos.x][current_pos.y],
        (uint8_t)current_pos.heading
    };

    UNERBUS_Write(&unerbus_pc_handle, cell_payload, UNERBUS_MAZE_CELL_UPDATE_SIZE);
    UNERBUS_Send(&unerbus_pc_handle, CMD_UPDATE_MAZE_CELL,
                 (uint8_t)(UNERBUS_CMD_ID_SIZE + UNERBUS_MAZE_CELL_UPDATE_SIZE));

    UNERBUS_Write(&unerbus_esp01_handle, cell_payload, UNERBUS_MAZE_CELL_UPDATE_SIZE);
    UNERBUS_Send(&unerbus_esp01_handle, CMD_UPDATE_MAZE_CELL,
                 (uint8_t)(UNERBUS_CMD_ID_SIZE + UNERBUS_MAZE_CELL_UPDATE_SIZE));
}

static void Reset_Robot_Position(void) {
    current_pos.x = 7;
    current_pos.y = 7;
    current_pos.heading = HEADING_NORTH;
}

static void Reset_Maze_State(void) {
    memset(maze_map, 0, sizeof(maze_map));
    Reset_Robot_Position();
}

static void Handle_Navigating(void)
{
    // Lectura de sensores
    dist_diagonal_left_mm = (uint16_t)ADC_To_Distance_mm((uint16_t)Get_Filtered_ADC_Value(SENSOR_DIAGONAL_LEFT_CH));
    dist_diagonal_right_mm = (uint16_t)ADC_To_Distance_mm((uint16_t)Get_Filtered_ADC_Value(SENSOR_DIAGONAL_RIGHT_CH));
    dist_front_left_mm = (uint16_t)ADC_To_Distance_mm((uint16_t)Get_Filtered_ADC_Value(SENSOR_FRONT_LEFT_CH));
    dist_front_right_mm = (uint16_t)ADC_To_Distance_mm((uint16_t)Get_Filtered_ADC_Value(SENSOR_FRONT_RIGHT_CH));
    dist_left_lat_mm = (uint16_t)ADC_To_Distance_mm((uint16_t)Get_Filtered_ADC_Value(SENSOR_LEFT_LAT_CH));
    dist_right_lat_mm = (uint16_t)ADC_To_Distance_mm((uint16_t)Get_Filtered_ADC_Value(SENSOR_RIGHT_LAT_CH));
    adc_rear_floor = (uint16_t)Get_Filtered_ADC_Value(SENSOR_FLOOR_REAR_CH);
    bool current_rear_tape = (adc_rear_floor < tape_detection_threshold_adc);
    if (current_rear_tape && !was_rear_tape_detected) {
        rear_tape_detected = true;
    }
    was_rear_tape_detected = current_rear_tape;
    // adc_front_floor = (uint16_t)Get_Filtered_ADC_Value(SENSOR_FLOOR_FRONT_CH);
    uint16_t front_avg_mm = (uint16_t)((dist_front_left_mm + dist_front_right_mm) / 2);

    // Detección de paredes
    left_diagonal_wall_detected = dist_diagonal_left_mm < wall_threshold_mm_diagonal;
    right_diagonal_wall_detected = dist_diagonal_right_mm < wall_threshold_mm_diagonal;
    left_wall_detected = dist_left_lat_mm < wall_threshold_mm_side;
    right_wall_detected = dist_right_lat_mm < wall_threshold_mm_side;
    front_wall_detected = front_avg_mm < wall_threshold_mm_front;

    if (front_avg_mm < wall_threshold_mm_braking_start)
    {
        Set_Motor_Speeds(0, 0);
        PID_Reset(&braking_pid);
        Set_Robot_State(STATE_BRAKING);
        return;
    }

    static uint8_t wall_diagonal_faded = 0;

    if (!left_diagonal_wall_detected || !right_diagonal_wall_detected)
    {
        wall_fade_counter++;
        if (wall_fade_counter >= wall_fade_ticks)
        {
            if (!left_diagonal_wall_detected && !right_diagonal_wall_detected)
            {
                wall_fade_counter = 0;
                wall_diagonal_faded = NO_WALL_FADED;
                Set_Robot_State(STATE_STRAIGHT_DRIVE_DESIDING);
                PID_Reset(&centering_pid);
                PID_Set_Setpoint(&centering_pid, FIXED_TO_INT(current_yaw_fixed));
                return;
            }
            else if (!left_diagonal_wall_detected)
            {
                wall_diagonal_faded = LEFT_WALL_FADED;
            }
            else
            {
                wall_diagonal_faded = RIGHT_WALL_FADED;
            }
        }
    }
    else
    {
        wall_fade_counter = 0; // Resetear contador si hay paredes detectadas
        wall_diagonal_faded = 0; // Resetear estado de pared desvanecida
    }

    if (rear_tape_detected)
    {
        // 1. Llegué a una nueva celda, me muevo lógicamente
        Update_Robot_Position();
        // 2. Ahora que ya pisé mi nueva (x,y), la mapeo buscando paredes
        Current_Cell_Mapping();
        // 3. Publicamos el estado de la celda mapeada para sincronizar la HMI
        Send_Maze_Cell_Update();
        
        // 4. Evaluamos si veníamos esperando esta línea para tomar una decisión
        if (wall_diagonal_faded)
        {
            rear_tape_detected = false;
            wall_diagonal_faded = NO_WALL_FADED;
            wall_fade_counter = 0;
            Handle_Deciding();
            return;
        }
        else 
        {
            // Si íbamos por un pasillo recto y no hay decisiones que tomar, 
            // IGUALMENTE debemos apagar la bandera para no sumar +10 posiciones en 100ms.
            rear_tape_detected = false; 
        }
    }

    // (Lógica de kick-start)
    if (kick_start_active)
    {
        int16_t ax;
        MPU6050_GetCalibratedData(&hmpu, &ax, NULL, NULL, NULL, NULL, NULL);
        if (abs(ax) > accel_motion_threshold)
            motion_confirm_counter++;
        else
            motion_confirm_counter = 0;
        if (motion_confirm_counter >= accel_motion_confirm_ticks)
        {
            kick_start_active = false;
            motion_confirm_counter = 0;
        }
    }

    uint16_t current_left_base_speed = kick_start_active ? (left_motor_base_speed + motor_kick_start_speed) : left_motor_base_speed;
    uint16_t current_right_base_speed = kick_start_active ? (right_motor_base_speed + motor_kick_start_speed) : right_motor_base_speed;

    // Disminución de la velocidad al dejar de detectar pared en diagonal
    current_left_base_speed = wall_diagonal_faded ? ((uint32_t)current_left_base_speed * 90)/100 : current_left_base_speed;
    current_right_base_speed = wall_diagonal_faded ? ((uint32_t)current_right_base_speed * 90)/100 : current_right_base_speed;

    int32_t pid_output_fixed = 0;

    if (left_diagonal_wall_detected && right_diagonal_wall_detected && left_wall_detected && right_wall_detected)
    {
        int32_t measured_diff = dist_left_lat_mm - dist_right_lat_mm;
        PID_Set_Setpoint(&centering_pid, 0);
        pid_output_fixed = PID_Update(&centering_pid, measured_diff, 10);
    }
    else if (right_diagonal_wall_detected && right_wall_detected)
    {
/*         PID_Set_Setpoint(&centering_pid, wall_target_mm);
        pid_output_fixed = PID_Update(&centering_pid, dist_right_lat_mm, 10);
        pid_output_fixed = -pid_output_fixed; */

        int32_t measured_diff = (wall_target_mm - dist_right_lat_mm)*2; // Se multiplica por 2 para dar más peso a la corrección al perder la pared diagonal, ya que solo queda una referencia.
        PID_Set_Setpoint(&centering_pid, 0);
        pid_output_fixed = PID_Update(&centering_pid, measured_diff, 10);
    }
    else if (left_diagonal_wall_detected && left_wall_detected)
    {
/*         PID_Set_Setpoint(&centering_pid, wall_target_mm);
        pid_output_fixed = PID_Update(&centering_pid, dist_left_lat_mm, 10); */

        int32_t measured_diff = (dist_left_lat_mm - wall_target_mm)*2; // Se multiplica por 2 para dar más peso a la corrección al perder la pared diagonal, ya que solo queda una referencia.
        PID_Set_Setpoint(&centering_pid, 0);
        pid_output_fixed = PID_Update(&centering_pid, measured_diff, 10);
    }
    else
    {
        // CASO 4: Sin paredes.
        PID_Set_Setpoint(&centering_pid, 0);
        pid_output_fixed = PID_Update(&centering_pid, 0, 10);
        return;
    }

    int16_t correction = (int16_t)FIXED_TO_INT(pid_output_fixed);
    Set_Motor_Speeds(current_right_base_speed - correction, current_left_base_speed + correction);
}

/**
 * @brief Maneja el estado de avance recto usando el PID de yaw.
 *        Se detiene si detecta un obstáculo frontal.
 */
static void Handle_Straight_Drive(bool have_to_decide)
{
    if (menu_mode == MENU_MODE_DRIVE_STRAIGHT)
    {
        dist_front_left_mm = (uint16_t)ADC_To_Distance_mm((uint16_t)Get_Filtered_ADC_Value(SENSOR_FRONT_LEFT_CH));
        dist_front_right_mm = (uint16_t)ADC_To_Distance_mm((uint16_t)Get_Filtered_ADC_Value(SENSOR_FRONT_RIGHT_CH));
        front_wall_detected = (dist_front_left_mm < wall_threshold_mm_braking_start || dist_front_right_mm < wall_threshold_mm_braking_start);
        if (front_wall_detected)
        {
            Set_Motor_Speeds(0, 0);
            app_state = APP_STATE_MENU;  // Volver al menú
            Set_Robot_State(STATE_IDLE); // Volver al estado de espera
            return;
        }
    }
    else
    {
        adc_rear_floor = (uint16_t)Get_Filtered_ADC_Value(SENSOR_FLOOR_REAR_CH);
        bool current_rear_tape = (adc_rear_floor < tape_detection_threshold_adc);
        if (current_rear_tape && !was_rear_tape_detected) {
            rear_tape_detected = true;
        }
        was_rear_tape_detected = current_rear_tape;

        if (rear_tape_detected)
        {
            rear_tape_detected = false;
            if (have_to_decide)
            {
            	Handle_Deciding();
            }
            else
            {
            	Set_Robot_State(STATE_NAVIGATING);
				PID_Reset(&centering_pid);
				kick_start_active = true;
				motion_confirm_counter = 0;
            }
            return;
        }
    }

    // Calcular la corrección del PID de guiñada
    // El setpoint fue fijado al entrar en este estado
    // El dt es 10ms, que es la frecuencia con la que se llama esta lógica (vía Do10ms)
    int16_t pwm_correction = (int16_t)(FIXED_TO_INT(PID_Update(&centering_pid, FIXED_TO_INT(current_yaw_fixed), 10)));

    // Aplicar la corrección a la velocidad base de los motores
    Set_Motor_Speeds(right_motor_base_speed - pwm_correction, left_motor_base_speed + pwm_correction);
}

static void Handle_Deciding(void)
{
	uint8_t posibleOptions = 0;
	uint8_t validOptions[4] = {0,0,0,0};
	uint8_t validOptionsCounter = 0;
	enum Direcciones { ATRAS, ADELANTE, DERECHA, IZQUIERDA };

    // Lectura de sensores
    dist_front_left_mm = (uint16_t)ADC_To_Distance_mm((uint16_t)Get_Filtered_ADC_Value(SENSOR_FRONT_LEFT_CH));
    dist_front_right_mm = (uint16_t)ADC_To_Distance_mm((uint16_t)Get_Filtered_ADC_Value(SENSOR_FRONT_RIGHT_CH));
    dist_left_lat_mm = (uint16_t)ADC_To_Distance_mm((uint16_t)Get_Filtered_ADC_Value(SENSOR_LEFT_LAT_CH));
    dist_right_lat_mm = (uint16_t)ADC_To_Distance_mm((uint16_t)Get_Filtered_ADC_Value(SENSOR_RIGHT_LAT_CH));
    uint16_t front_avg_mm = (uint16_t)((dist_front_left_mm + dist_front_right_mm) / 2);

    // Detección de paredes
    left_wall_detected = dist_left_lat_mm < wall_threshold_mm_side;
    right_wall_detected = dist_right_lat_mm < wall_threshold_mm_side;
    front_wall_detected = front_avg_mm < wall_threshold_mm_front;

    // Analizar las opciones (asumiendo que 1 siempre es "atrás" y está libre)
    // Bit 0: Atrás (Siempre 1)
    // Bit 1: Adelante
    // Bit 2: Derecha
    // Bit 3: Izquierda
    posibleOptions = ((!left_wall_detected)  << 3) |
                     ((!right_wall_detected) << 2) |
                     ((!front_wall_detected) << 1) |
                     (1 << 0); // El camino de atrás siempre suele estar libre

    // Si solo está disponible el camino hacia atrás
    if (posibleOptions == 1)
    {
    	Turn_Start(180); // Callejón sin salida
    	return;
    }

    for (uint8_t i = 1; i < 4; i++)
    {
        // Verificamos si el bit 'i' está encendido
        if (posibleOptions & (1 << i))
        {
            validOptions[validOptionsCounter] = i;
            validOptionsCounter++;
        }
    }

    // Elegimos una opción al azar:
    uint8_t choice = validOptions[rand() % validOptionsCounter];

    if (choice == IZQUIERDA)
    {
        Set_Robot_State(STATE_SMOOTH_TURN_LEFT); // Prioridad a la izquierda
        current_yaw_fixed = 0;
        PID_Reset(&turn_velocity_pid);
        PID_Set_Setpoint(&turn_velocity_pid, turn_target_dps); // Valores positivos de gz para giro a la izquierda
    }
    else if (choice == DERECHA)
    {
        Set_Robot_State(STATE_SMOOTH_TURN_RIGHT);
        current_yaw_fixed = 0;
        PID_Reset(&turn_velocity_pid);
        PID_Set_Setpoint(&turn_velocity_pid, -turn_target_dps); // Valores negativos de gz para giro a la derecha
    }
    else if (choice == ADELANTE)
    {
        if (left_wall_detected || right_wall_detected)
        {
            Set_Robot_State(STATE_NAVIGATING);
            PID_Reset(&centering_pid);
/*             kick_start_active = true;
            motion_confirm_counter = 0; */
        }
        else
        {
            Set_Robot_State(STATE_STRAIGHT_DRIVE);
            PID_Reset(&centering_pid);
            PID_Set_Setpoint(&centering_pid, FIXED_TO_INT(current_yaw_fixed));
        }	
    }
}

static void Handle_Braking(void)
{
    // 1. Leer y convertir sensores frontales a mm
    uint16_t dist_front_avg_mm = ((uint16_t)ADC_To_Distance_mm((uint16_t)Get_Filtered_ADC_Value(SENSOR_FRONT_LEFT_CH)) +
                                  (uint16_t)ADC_To_Distance_mm((uint16_t)Get_Filtered_ADC_Value(SENSOR_FRONT_RIGHT_CH))) /
                                 2;

    // 2. Comprobar si el frenado ha terminado
    int16_t ax;
    MPU6050_GetCalibratedData(&hmpu, &ax, NULL, NULL, NULL, NULL, NULL);

    // La condición de parada usa el error absoluto en mm y la aceleración
    if (abs(dist_front_avg_mm - wall_braking_target_mm) < braking_dead_zone && abs(ax) < braking_accel_stop_threshold)
    {
        Set_Motor_Speeds(0, 0);
        Set_Robot_State(STATE_DECIDING);
        return;
    }

    // 3. Calcular la salida del PID. El setpoint ya está configurado.
    int32_t pid_output_fixed = PID_Update(&braking_pid, dist_front_avg_mm, 10);

    // 4. Invertir la salida del PID para obtener la velocidad.
    //    Si estamos lejos (dist > target), el PID da una salida negativa.
    //    Necesitamos una velocidad POSITIVA para avanzar.
    int16_t motor_speed = -(int16_t)FIXED_TO_INT(pid_output_fixed);

    // 5. Lógica de potencia mínima para vencer la inercia.
    if (motor_speed > 0 && motor_speed < braking_min_speed)
        motor_speed = braking_min_speed;
    else if (motor_speed < 0 && motor_speed > -braking_min_speed)
        motor_speed = -braking_min_speed;

    // 6. Aplicar la MISMA velocidad a ambos motores para un frenado recto.
    Set_Motor_Speeds(motor_speed, motor_speed);
}

/**
 * @brief  Establece un nuevo estado para el robot y solicita una actualización del display.
 * @param  new_state El nuevo estado del robot.
 * @retval None
 */
static void Set_Robot_State(RobotStateTypeDef new_state)
{
    if (robot_state != new_state)
    {
        robot_state = new_state;
        // SSD_UPDATE_REQUEST = true;
    }
}

/**
 * @brief Prepara el contenido del buffer del display OLED según el estado actual de la app.
 * @retval None
 */
static void Update_Display_Content(void)
{
    char text_line1[22];
    char text_line2[22];
    char text_line3[22];
    char text_line4[22];
    char text_line5[22];

    SSD1306_Clear(&hssd);

    if (app_state == APP_STATE_MENU)
    {
        snprintf(text_line1, sizeof(text_line1), "%s Idle", (menu_mode == MENU_MODE_IDLE) ? ">" : " ");
        snprintf(text_line2, sizeof(text_line2), "%s Find Cells", (menu_mode == MENU_MODE_FIND_CELLS) ? ">" : " ");
        snprintf(text_line3, sizeof(text_line3), "%s Go A->B", (menu_mode == MENU_MODE_GO_TO_B) ? ">" : " ");
        snprintf(text_line4, sizeof(text_line4), "%s Manual", (menu_mode == MENU_MODE_MANUAL_CONTROL) ? ">" : " ");
        snprintf(text_line5, sizeof(text_line5), "%s Drive Straight", (menu_mode == MENU_MODE_DRIVE_STRAIGHT) ? ">" : " ");

        SSD1306_DrawText(&hssd, 0, 0, "--- MENU ---", SSD1306_TEXT_ALIGN_LEFT);
        SSD1306_DrawText(&hssd, 0, 10, text_line1, SSD1306_TEXT_ALIGN_LEFT);
        SSD1306_DrawText(&hssd, 0, 20, text_line2, SSD1306_TEXT_ALIGN_LEFT);
        SSD1306_DrawText(&hssd, 0, 30, text_line3, SSD1306_TEXT_ALIGN_LEFT);
        SSD1306_DrawText(&hssd, 0, 40, text_line4, SSD1306_TEXT_ALIGN_LEFT);
        SSD1306_DrawText(&hssd, 0, 50, text_line5, SSD1306_TEXT_ALIGN_LEFT);
    }
    /*     else // APP_STATE_RUNNING
        {
            const char *current_mode_str = "Unknown";
            switch (menu_mode)
            {
            case MENU_MODE_IDLE:
                current_mode_str = "Idle";
                break;
            case MENU_MODE_FIND_CELLS:
                current_mode_str = "Finding Cells";
                break;
            case MENU_MODE_GO_TO_B:
                current_mode_str = "Going A->B";
                break;
            case MENU_MODE_MANUAL_CONTROL:
                current_mode_str = "Manual Control";
                break;
            } //
            snprintf(text_line1, sizeof(text_line1), "Mode: %s", current_mode_str);

            const char *robot_state_str = "Stopped";
            switch (robot_state)
            {
            case STATE_NAVIGATING:
                robot_state_str = "Centering...";
                break;
            case STATE_BRAKING:
                robot_state_str = "Braking...";
                break;
            case STATE_DECIDING:
                robot_state_str = "Deciding...";
                break;
            case STATE_TURNING_LEFT:
            case STATE_TURNING_RIGHT:
            case STATE_TURN_AROUND:
                robot_state_str = "Turning...";
                break;
            }
            snprintf(text_line2, sizeof(text_line2), "State: %s", robot_state_str);

            SSD1306_DrawText(&hssd, 0, 0, text_line1, SSD1306_TEXT_ALIGN_LEFT);
            SSD1306_DrawText(&hssd, 0, 10, text_line2, SSD1306_TEXT_ALIGN_LEFT);
        } */
}

/**
 * @brief Precalcula las pendientes de los segmentos de la LUT de ADC.
 * @note  Debe llamarse una sola vez durante la inicialización.
 *        Rellena el array global `adc_lut_segs`.
 * @retval None
 */
static void ADC_LUT_Precompute(void)
{
    for (uint8_t i = 0; i < ADC_SEG_COUNT; ++i)
    {
        const uint16_t x0 = sensor_lut[i].adc;
        const uint16_t x1 = sensor_lut[i + 1].adc;
        const int32_t y0 = (int32_t)sensor_lut[i].dist_mm;
        const int32_t y1 = (int32_t)sensor_lut[i + 1].dist_mm;

        adc_lut_segs[i].x0 = x0;
        adc_lut_segs[i].x1 = x1;
        adc_lut_segs[i].y0 = (uint16_t)y0;

        const int32_t dx = (int32_t)x1 - (int32_t)x0;
        const int32_t dy = y1 - y0;

        // Calcula la pendiente en formato Q16.16, manejando la división por cero.
        if (dx != 0)
        {
            adc_lut_segs[i].slope_q16 = ((dy << FIXED_POINT_SHIFT) / dx);
        }
        else
        {
            adc_lut_segs[i].slope_q16 = 0;
        }
    }
}

/**
 * @brief Convierte un valor de ADC a distancia en milímetros usando una LUT con pendientes precalculadas.
 * @param  adc_value: El valor de 12 bits del ADC a convertir.
 * @retval La distancia estimada en milímetros (int32_t).
 * @note   Utiliza búsqueda binaria y aritmética de punto fijo. No usa 'float'.
 */
static int32_t ADC_To_Distance_mm(uint16_t adc_value)
{
    // 1. Manejo de casos extremos (saturación)
    if (adc_value <= sensor_lut[0].adc)
    {
        return (int32_t)sensor_lut[0].dist_mm;
    }
    if (adc_value >= sensor_lut[ADC_LUT_SIZE - 1].adc)
    {
        return (int32_t)sensor_lut[ADC_LUT_SIZE - 1].dist_mm;
    }

    // 2. Búsqueda binaria para encontrar el segmento correcto [lo, lo+1]
    uint8_t lo = 0;
    uint8_t hi = (uint8_t)(ADC_LUT_SIZE - 1);
    while ((uint8_t)(hi - lo) > 1U)
    {
        uint8_t mid = (uint8_t)((lo + hi) >> 1);
        if (adc_value < sensor_lut[mid].adc)
        {
            hi = mid;
        }
        else
        {
            lo = mid;
        }
    }

    // 3. Interpolación lineal usando el segmento precalculado 'lo'
    const ADC_LutSeg_t *s = &adc_lut_segs[lo];

    // y = y0 + slope * (x - x0)
    int32_t dx = (int32_t)adc_value - (int32_t)s->x0;

    // Multiplicación en 64 bits para evitar overflow (Q16.16 * Q16.0 -> Q32.16)
    int64_t mul = (int64_t)s->slope_q16 * (int64_t)dx;

    // Redondeo al entero más cercano y conversión de vuelta a entero
    int32_t incr = (int32_t)((mul + (1LL << (FIXED_POINT_SHIFT - 1))) >> FIXED_POINT_SHIFT);

    int32_t y = (int32_t)s->y0 + incr;

    // 4. Seguridad final
    if (y < 0)
        return 0;

    return y;
}

/**
 * @brief Maneja el estado de giro suave en intersecciones.
 *
 */
static void Handle_Smooth_Turn(void)
{
    bool wall_detected = false;
    int16_t base_right = 0, base_left = 0;

    adc_rear_floor = (uint16_t)Get_Filtered_ADC_Value(SENSOR_FLOOR_REAR_CH);
    bool current_rear_tape = (adc_rear_floor < tape_detection_threshold_adc);
    if ((abs(FIXED_TO_INT(current_yaw_fixed)) > 60) && current_rear_tape && !was_rear_tape_detected) {
        rear_tape_detected = true;
    }
    was_rear_tape_detected = current_rear_tape;

    if (robot_state == STATE_SMOOTH_TURN_LEFT)
    {
        dist_diagonal_left_mm = (uint16_t)ADC_To_Distance_mm((uint16_t)Get_Filtered_ADC_Value(SENSOR_DIAGONAL_LEFT_CH));
        wall_detected = (dist_diagonal_left_mm < after_turn_wall_threshold_mm);
        base_right = (int16_t)faster_motor_smooth_turn_speed; // exterior
        base_left = (int16_t)slower_motor_smooth_turn_speed;  // interior
    }
    else if (robot_state == STATE_SMOOTH_TURN_RIGHT)
    {
        dist_diagonal_right_mm = (uint16_t)ADC_To_Distance_mm((uint16_t)Get_Filtered_ADC_Value(SENSOR_DIAGONAL_RIGHT_CH));
        wall_detected = (dist_diagonal_right_mm < after_turn_wall_threshold_mm);
        base_right = (int16_t)slower_motor_smooth_turn_speed; // interior
        base_left = (int16_t)faster_motor_smooth_turn_speed;  // exterior
    }

/*     if (!wall_detected)
    {
        dist_front_left_mm = (uint16_t)ADC_To_Distance_mm((uint16_t)Get_Filtered_ADC_Value(SENSOR_FRONT_LEFT_CH));
        dist_front_right_mm = (uint16_t)ADC_To_Distance_mm((uint16_t)Get_Filtered_ADC_Value(SENSOR_FRONT_RIGHT_CH));
        wall_detected = (dist_front_left_mm < (wall_threshold_mm_braking_start) || dist_front_right_mm < (wall_threshold_mm_braking_start));
    } */

    if (wall_detected || (abs(FIXED_TO_INT(current_yaw_fixed)) >= (90 - TURN_COMPLETION_DEAD_ZONE)))
    {
        RobotStateTypeDef completed_turn_state = robot_state;

        Set_Motor_Speeds(right_motor_base_speed, left_motor_base_speed);
        Set_Robot_State(STATE_NAVIGATING);
        PID_Reset(&centering_pid);
        kick_start_active = true;
        motion_confirm_counter = 0;

        if (completed_turn_state == STATE_SMOOTH_TURN_LEFT)
            Update_Robot_Heading(TURN_LEFT);
        else if (completed_turn_state == STATE_SMOOTH_TURN_RIGHT)
            Update_Robot_Heading(TURN_RIGHT);

        // Publicamos heading/pose al finalizar el giro para sincronizar Qt
        // aunque no haya cruce inmediato de cinta.
        Send_Maze_Cell_Update();

        return;
    }

    int16_t gz;
    MPU6050_GetCalibratedData(&hmpu, NULL, NULL, NULL, NULL, NULL, &gz);

    int32_t angular_velocity_fixed = FIXED_DIV(INT_TO_FIXED(gz), GYRO_SENSITIVITY);
    int16_t angular_velocity_dps = (int16_t)FIXED_TO_INT(angular_velocity_fixed);

    int32_t pid_output_fixed = PID_Update(&turn_velocity_pid, angular_velocity_dps, 10);
    int16_t correction = (int16_t)FIXED_TO_INT(pid_output_fixed);

    int16_t right_speed = base_right + correction;
    int16_t left_speed = base_left - correction;

    Set_Motor_Speeds(right_speed, left_speed);
}

static void Modes_State_Machine(void)
{
    // Maquina de estados del robot
    if (app_state == APP_STATE_RUNNING)
    {
        switch (menu_mode)
        {
        case MENU_MODE_FIND_CELLS:
            // Ejecutar la lógica de resolución de laberintos
            switch (robot_state)
            {
            case STATE_NAVIGATING:
                Handle_Navigating();
                break;
            case STATE_BRAKING:
                Handle_Braking();
                break;
            case STATE_DECIDING:
                Handle_Deciding();
                break;
            case STATE_LEFT_WALL_FADE:
            case STATE_RIGHT_WALL_FADE:
            case STATE_STRAIGHT_DRIVE:
                Handle_Straight_Drive(false);
                break;
            case STATE_STRAIGHT_DRIVE_DESIDING:
            	Handle_Straight_Drive(true);
                break;
            case STATE_TURNING_LEFT:
            case STATE_TURNING_RIGHT:
            case STATE_TURN_AROUND_LEFT:
            case STATE_TURN_AROUND_RIGHT:
                Manage_Turn();
                break;
            case STATE_SMOOTH_TURN_LEFT:
            case STATE_SMOOTH_TURN_RIGHT:
                Handle_Smooth_Turn();
                break;
            default:
                Handle_Idle();
                break;
            }
            break;
        case MENU_MODE_MANUAL_CONTROL:
            // En modo manual, solo gestionamos los giros.
            // El control de motores se hace directamente por comandos.
            switch (robot_state)
            {
            case STATE_TURNING_LEFT:
            case STATE_TURNING_RIGHT:
            case STATE_TURN_AROUND_LEFT:
            case STATE_TURN_AROUND_RIGHT:
                Manage_Turn();
                break;
            case STATE_IDLE:
            default:
                // No hacer nada, permite que los comandos externos
                // controlen los motores sin que Handle_Idle() los detenga.
                break;
            }
            break;
        case MENU_MODE_DRIVE_STRAIGHT:
            switch (robot_state)
            {
            case STATE_BRAKING:
                Handle_Braking();
                break;
            case STATE_STRAIGHT_DRIVE:
                Handle_Straight_Drive(false);
                break;
            default:
                // No hacer nada, permite que los comandos externos
                // controlen los motores sin que Handle_Idle() los detenga.
                break;
            }
            break;
        case MENU_MODE_IDLE:
        case MENU_MODE_GO_TO_B:
        default:
            // Para otros modos, por ahora, solo estar en reposo.
            Handle_Idle();
            break;
        }
    }
    else // APP_STATE_MENU
    {
        // En el menú, los motores siempre están parados.
        Handle_Idle();
    }
}
