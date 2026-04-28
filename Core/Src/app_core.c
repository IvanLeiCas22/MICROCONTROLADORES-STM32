/* STM32/Test2024-master/Core/Src/app_core.c */
#include "app_core.h"
#include "app_config.h"
#include "app_maze.h"
#include "app_sensors.h"
#include "app_timebase.h"
#include "app_timing.h"

#include <stdlib.h>
#include <stdio.h>  // Para snprintf

#include "usbd_cdc_if.h"
#include "ESP01.h"
#include "UNERBUS.h"
#include "MPU6050.h"
#include "BUTTONS.h"
#include "SSD1306.h"
#include "pid_controller.h"

/* --- Gyro scaling --- */
#define GYRO_SENSITIVITY_X10_250DPS 1310
#define GYRO_SENSITIVITY_X10_500DPS 655
#define GYRO_SENSITIVITY_X10_1000DPS 328
#define GYRO_SENSITIVITY_X10_2000DPS 164
#define GYRO_YAW_DEADBAND_DPS_X10 75

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
volatile bool app_uart_bypass = false;
volatile bool app_mpu_read_request = false;
volatile bool app_ssd_update_request = false;
uint16_t pwm_max_value = 10000; // Valor máximo del PWM

_sESP01Handle esp01_handle;
_sUNERBUSHandle unerbus_pc_handle;
_sUNERBUSHandle unerbus_esp01_handle;

char local_ip[IP_ADDRESS_STRING_LENGTH];
uint8_t buf_rx_pc[USB_CDC_RX_BUFFER_SIZE], buf_tx_pc[USB_CDC_TX_BUFFER_SIZE];
uint8_t buf_rx_esp01[WIFI_RX_BUFFER_SIZE], buf_tx_esp01[WIFI_TX_BUFFER_SIZE], data_rx_esp01;

uint32_t heartbeat_counter, heartbeat_mask;
uint8_t timeout_alive_udp;

static MPU6050_HandleTypeDef hmpu;
static SSD1306_HandleTypeDef hssd;
Button_HandleTypeDef h_user_button;
uint16_t motor_pwm_values[PWM_CHANNELS] = {0, 0, 0, 0};
static volatile I2C_BusStateTypeDef i2c_bus_state = I2C_BUS_IDLE;
static AppTimebase app_timebase;
static const AppTimebaseConfig app_timebase_config = {
    .period_ticks = {
        [APP_TIMEBASE_EVENT_1MS] = APP_TIMEBASE_1MS_TICKS,
        [APP_TIMEBASE_EVENT_10MS] = APP_TIMEBASE_10MS_TICKS,
        [APP_TIMEBASE_EVENT_100MS] = APP_TIMEBASE_100MS_TICKS,
        [APP_TIMEBASE_EVENT_IR_SAMPLE] = APP_TIMEBASE_IR_SAMPLE_TICKS,
        [APP_TIMEBASE_EVENT_MPU_SAMPLE] = APP_TIMEBASE_MPU_SAMPLE_TICKS,
        [APP_TIMEBASE_EVENT_CONTROL] = APP_TIMEBASE_CONTROL_TICKS,
    },
    .max_pending_events = APP_TIMEBASE_MAX_PENDING_EVENTS,
};
static uint32_t control_step_dt_ms = CONTROL_PERIOD_MS;

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
typedef enum
{
    PID_ROLE_CENTERING = 0,
    PID_ROLE_TURN,
    PID_ROLE_SMOOTH_TURN,
    PID_ROLE_BRAKING,
    PID_ROLE_COUNT
} PID_Role_t;

static PID_Config_t pid_configs[PID_ROLE_COUNT];
static PID_Controller_t *const pid_instances[PID_ROLE_COUNT] = {
    &centering_pid,
    &turn_pid,
    &turn_velocity_pid,
    &braking_pid};

typedef struct
{
    uint16_t adc_filtered[ADC_CHANNELS];
    uint16_t dist_diagonal_left_mm;
    uint16_t dist_diagonal_right_mm;
    uint16_t dist_front_left_mm;
    uint16_t dist_front_right_mm;
    uint16_t dist_left_lat_mm;
    uint16_t dist_right_lat_mm;
    uint8_t detection_flags;
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
    int32_t yaw_fixed;
} SensorSnapshotTypeDef;

typedef enum
{
    ACTION_NONE = 0,
    ACTION_GO_FRONT,
    ACTION_GO_LEFT_SMOOTH,
    ACTION_GO_RIGHT_SMOOTH,
    ACTION_GO_BACK
} MotionAction;

typedef enum
{
    MOTION_IDLE = 0,
    MOTION_STRAIGHT_ADVANCE,
    MOTION_SMOOTH_TURN,
    //MOTION_POST_SMOOTH_ADVANCE,
    MOTION_PIVOT_TURN,
    MOTION_BRAKING,
    MOTION_DONE,
    MOTION_ERROR
} MotionPhase;

typedef enum
{
    MOTION_RESULT_NONE = 0,
    MOTION_RESULT_NEW_CELL_REACHED,
    MOTION_RESULT_ERROR,
    MOTION_RESULT_TIMEOUT
} MotionResult;

typedef enum
{
    STRAIGHT_REF_NONE = 0,
    STRAIGHT_REF_BOTH_WALLS,
    STRAIGHT_REF_LEFT_WALL,
    STRAIGHT_REF_RIGHT_WALL,
    STRAIGHT_REF_YAW
} StraightReference;

typedef struct
{
    MotionAction active_action;
    uint8_t action_active;

    MotionResult result;
    uint8_t result_pending;

    MotionPhase phase;

    PID_Controller_t *active_pid;

    uint8_t line_detector_armed;
    uint8_t waiting_leave_current_line;

    StraightReference straight_ref;
} MotionContext;

static MotionContext navigation_motion;
static bool navigation_first_execution = true;

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
uint8_t wall_fade_ticks = WALL_FADE_TICKS_DEFAULT;
static SensorSnapshotTypeDef sensor_snapshot;

static volatile int32_t current_yaw_fixed = 0; // Yaw angle in Q16.16 fixed-point (degrees)
static volatile uint16_t gyro_sensitivity_x10 = GYRO_SENSITIVITY_X10_500DPS;
static AppTimingClock mpu_yaw_timing_clock;
static volatile bool mpu_yaw_timing_initialized = false;
static volatile uint32_t mpu_last_sample_cycle = 0;

static volatile RobotStateTypeDef robot_state = STATE_IDLE;
uint16_t motor_kick_start_speed;
uint16_t accel_motion_threshold;
uint8_t accel_motion_confirm_ticks;
static bool kick_start_active = false;
static uint8_t motion_confirm_counter = 0;

// LABERINTO
static bool pending_initial_cell_seed = false;

//==============================================================================
// PROTOTIPOS DE FUNCIONES PRIVADAS
//==============================================================================
void ESP01_SetChipEnable(uint8_t value);
int ESP01_WriteUartByte(uint8_t value);
void ESP01_WriteByteToRxBuffer(uint8_t value);
void ESP01_ChangeState(_eESP01STATUS esp01State);
void DecodeCMD(struct UNERBUSHandle *aBus, uint8_t iStartData);
void Do1ms(void);
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
static void Prepare_MPU_BlockingTransaction(void);
uint8_t UART_TransmitByte(uint8_t value);

static uint8_t Consume_Timebase_Event(AppTimebaseEvent event);
static void Run_Control_Step(uint32_t dt_ms);
static void Init_Cycle_Counter(void);
static uint32_t Read_Cycle_Counter(void);
static void Update_Gyro_Scaler(void);
static int32_t GyroRaw_To_DpsX10(int16_t gyro_raw);
static int16_t GyroRaw_To_Dps(int16_t gyro_raw);
static void Reset_Yaw_Tracking(void);
static void Integrate_Yaw_From_Gyro(int16_t gz_calibrated);
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
void Turn_Start(int16_t angle_degrees);
static void ADC_Filter_Task(void);
static void ADC_LUT_Precompute(void);
static int32_t Get_Filtered_ADC_Value(uint8_t channel);
static void Set_Robot_State(RobotStateTypeDef new_state);
static void Update_Display_Content(void);
static int32_t ADC_To_Distance_mm(uint16_t adc_value);
static bool Detect_Low_With_Hysteresis(uint16_t value, uint16_t threshold, uint16_t hysteresis, bool was_detected);
static bool Detect_Front_Wall_With_Hysteresis(uint16_t left_value, uint16_t right_value, uint16_t threshold, uint16_t hysteresis, bool was_detected);
static void Sync_Legacy_Perception_From_Snapshot(void);
static void Handle_Straight_Drive(bool have_to_decide);
static void Modes_State_Machine(void);
static void Update_Navigation_Perception(void);
static void Commit_Maze_State(int8_t heading_update, bool update_cell, bool send_update);
static void Init_Pid_Configs(void);
static void Apply_Pid_Config(PID_Role_t role, bool reset_state);
static void Set_Pid_Gains_From_U16(PID_Role_t role, uint16_t kp_x100, uint16_t ki_x100, uint16_t kd_x100, bool reset_state);
static void Write_Pid_Gains_To_Buffer(PID_Role_t role, uint8_t *buffer);
static int32_t Gain_Hundredths_To_Fixed(uint16_t gain_x100);
static uint16_t Fixed_To_Gain_Hundredths(int32_t gain_fixed);

// --- FUNCIONES DE REWORK DE NAVEGACIÓN ---
static void Navigation_DecideMovement(MotionContext *motion);
static MotionAction Navigation_SelectNextAction(void);
uint8_t Motion_StartAction(MotionContext *motion, MotionAction action);
static uint8_t Motion_SetPID(MotionContext *motion, PID_Role_t role, int32_t setpoint);
static void Motion_ExecuteAction(MotionContext *motion);
static void Motion_Complete(MotionContext *motion, MotionResult result);
static bool Motion_DetectedNewCellTape(MotionContext *motion);
static StraightReference Motion_SelectStraightReference(void);
static void Motion_ControlStraightAdvance(MotionContext *motion);

//==============================================================================
// IMPLEMENTACIÓN DE WRAPPERS DE CALLBACKS HAL
//==============================================================================
void App_Core_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        App_Timebase_OnTick(&app_timebase);

        if (App_Timebase_Consume(&app_timebase, APP_TIMEBASE_EVENT_IR_SAMPLE) > 0U)
        {
            HAL_ADC_Start_DMA(&hadc1, (uint32_t *)App_Sensors_GetAdcDmaWriteBuffer(), ADC_CHANNELS);
        }

        if (App_Timebase_Consume(&app_timebase, APP_TIMEBASE_EVENT_MPU_SAMPLE) > 0U)
        {
            MPU_READ_REQUEST = true; // Indicar que se debe leer el MPU
        }
    }
}

void App_Core_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    (void)hadc;
    App_Sensors_OnAdcDmaComplete();
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

        Integrate_Yaw_From_Gyro((int16_t)(hmpu.raw_data.gyro_z_raw - hmpu.gyro_offset_z));
        i2c_bus_state = I2C_BUS_IDLE;
    }
}

void App_Core_USB_ReceiveData(uint8_t *buf, uint16_t len)
{
    UNERBUS_ReceiveBuf(&unerbus_pc_handle, buf, len);
}

static uint8_t Consume_Timebase_Event(AppTimebaseEvent event)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t pending_events;

    __disable_irq();
    pending_events = App_Timebase_Consume(&app_timebase, event);
    if (primask == 0U)
    {
        __enable_irq();
    }

    return pending_events;
}

static void Init_Cycle_Counter(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    App_Timing_Init(&mpu_yaw_timing_clock, SystemCoreClock);
}

static uint32_t Read_Cycle_Counter(void)
{
    return DWT->CYCCNT;
}

//==============================================================================
// IMPLEMENTACIÓN DE FUNCIONES DE LA APLICACIÓN
//==============================================================================

static int32_t Gain_Hundredths_To_Fixed(uint16_t gain_x100)
{
    return HUNDREDTHS_TO_FIXED(gain_x100);
}

static uint16_t Fixed_To_Gain_Hundredths(int32_t gain_fixed)
{
    return (uint16_t)(((int64_t)gain_fixed * 100) >> FIXED_POINT_SHIFT);
}

static int32_t GyroRaw_To_DpsX10(int16_t gyro_raw)
{
    uint16_t sensitivity_x10 = gyro_sensitivity_x10;
    int32_t scaled_raw = (int32_t)gyro_raw * 100;

    if (sensitivity_x10 == 0U)
    {
        sensitivity_x10 = GYRO_SENSITIVITY_X10_500DPS;
    }

    if (scaled_raw >= 0)
    {
        return (scaled_raw + ((int32_t)sensitivity_x10 / 2)) / (int32_t)sensitivity_x10;
    }
    return (scaled_raw - ((int32_t)sensitivity_x10 / 2)) / (int32_t)sensitivity_x10;
}

static int16_t GyroRaw_To_Dps(int16_t gyro_raw)
{
    int32_t dps_x10 = GyroRaw_To_DpsX10(gyro_raw);
    if (dps_x10 >= 0)
    {
        return (int16_t)((dps_x10 + 5) / 10);
    }
    return (int16_t)((dps_x10 - 5) / 10);
}

static void Reset_Yaw_Tracking(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    current_yaw_fixed = 0;
    mpu_yaw_timing_initialized = false;
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static void Integrate_Yaw_From_Gyro(int16_t gz_calibrated)
{
    uint32_t now_cycles = Read_Cycle_Counter();

    if (!mpu_yaw_timing_initialized)
    {
        mpu_yaw_timing_initialized = true;
        mpu_last_sample_cycle = now_cycles;
        return;
    }

    uint32_t dt_us = App_Timing_ElapsedUs(&mpu_yaw_timing_clock, mpu_last_sample_cycle, now_cycles);
    mpu_last_sample_cycle = now_cycles;

    if ((dt_us == 0U) || (dt_us > 50000U))
    {
        dt_us = MPU_SAMPLE_PERIOD_US;
    }

    int32_t dps_x10 = GyroRaw_To_DpsX10(gz_calibrated);
    if ((dps_x10 <= GYRO_YAW_DEADBAND_DPS_X10) && (dps_x10 >= -GYRO_YAW_DEADBAND_DPS_X10))
    {
        return;
    }

    int32_t dt_q16 = (int32_t)(((uint32_t)dt_us << FIXED_POINT_SHIFT) / 1000000U);
    int32_t yaw_delta_q16 = (dps_x10 * dt_q16) / 10;
    current_yaw_fixed -= yaw_delta_q16;
}

static void Apply_Pid_Config(PID_Role_t role, bool reset_state)
{
    PID_ApplyConfig(pid_instances[role], &pid_configs[role], reset_state);
}

static void Set_Pid_Gains_From_U16(PID_Role_t role, uint16_t kp_x100, uint16_t ki_x100, uint16_t kd_x100, bool reset_state)
{
    pid_configs[role].kp = Gain_Hundredths_To_Fixed(kp_x100);
    pid_configs[role].ki = Gain_Hundredths_To_Fixed(ki_x100);
    pid_configs[role].kd = Gain_Hundredths_To_Fixed(kd_x100);
    Apply_Pid_Config(role, reset_state);
}

static void Write_Pid_Gains_To_Buffer(PID_Role_t role, uint8_t *buffer)
{
    uint16_t kp_x100 = Fixed_To_Gain_Hundredths(pid_configs[role].kp);
    uint16_t ki_x100 = Fixed_To_Gain_Hundredths(pid_configs[role].ki);
    uint16_t kd_x100 = Fixed_To_Gain_Hundredths(pid_configs[role].kd);

    buffer[0] = (uint8_t)(kp_x100 & 0xFF);
    buffer[1] = (uint8_t)((kp_x100 >> 8) & 0xFF);
    buffer[2] = (uint8_t)(ki_x100 & 0xFF);
    buffer[3] = (uint8_t)((ki_x100 >> 8) & 0xFF);
    buffer[4] = (uint8_t)(kd_x100 & 0xFF);
    buffer[5] = (uint8_t)((kd_x100 >> 8) & 0xFF);
}

static void Init_Pid_Configs(void)
{
    pid_configs[PID_ROLE_CENTERING] = (PID_Config_t){
        .kp = Gain_Hundredths_To_Fixed(80),
        .ki = Gain_Hundredths_To_Fixed(0),
        .kd = Gain_Hundredths_To_Fixed(20),
        .out_min = INT_TO_FIXED(-max_pwm_correction),
        .out_max = INT_TO_FIXED(max_pwm_correction)};

    pid_configs[PID_ROLE_BRAKING] = (PID_Config_t){
        .kp = Gain_Hundredths_To_Fixed(BRAKING_PID_KP_DEFAULT_X100),
        .ki = Gain_Hundredths_To_Fixed(BRAKING_PID_KI_DEFAULT_X100),
        .kd = Gain_Hundredths_To_Fixed(BRAKING_PID_KD_DEFAULT_X100),
        .out_min = INT_TO_FIXED(-braking_max_pwm_offset),
        .out_max = INT_TO_FIXED(braking_max_pwm_offset)};

    pid_configs[PID_ROLE_TURN] = (PID_Config_t){
        .kp = Gain_Hundredths_To_Fixed(TURN_PID_KP_DEFAULT_X100),
        .ki = Gain_Hundredths_To_Fixed(TURN_PID_KI_DEFAULT_X100),
        .kd = Gain_Hundredths_To_Fixed(TURN_PID_KD_DEFAULT_X100),
        .out_min = INT_TO_FIXED(-turn_max_pwm),
        .out_max = INT_TO_FIXED(turn_max_pwm)};

    pid_configs[PID_ROLE_SMOOTH_TURN] = (PID_Config_t){
        .kp = Gain_Hundredths_To_Fixed(TURN_VELOCITY_PID_KP_DEFAULT_X100),
        .ki = Gain_Hundredths_To_Fixed(TURN_VELOCITY_PID_KI_DEFAULT_X100),
        .kd = Gain_Hundredths_To_Fixed(TURN_VELOCITY_PID_KD_DEFAULT_X100),
        .out_min = INT_TO_FIXED(-turn_max_pwm),
        .out_max = INT_TO_FIXED(turn_max_pwm)};
}

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
    case CMD_GET_IR_SENSOR_SNAPSHOT:
    {
        uint8_t ir_buffer[IR_SENSOR_SNAPSHOT_BYTES];
        uint16_t ir_values[ADC_CHANNELS] = {
            sensor_snapshot.dist_right_lat_mm,
            sensor_snapshot.dist_diagonal_right_mm,
            sensor_snapshot.dist_front_right_mm,
            sensor_snapshot.adc_filtered[SENSOR_FLOOR_FRONT_CH],
            sensor_snapshot.dist_front_left_mm,
            sensor_snapshot.dist_diagonal_left_mm,
            sensor_snapshot.dist_left_lat_mm,
            sensor_snapshot.adc_filtered[SENSOR_FLOOR_REAR_CH]};

        for (uint8_t i = 0; i < ADC_CHANNELS; i++)
        {
            ir_buffer[idx++] = (uint8_t)(ir_values[i] & 0xFF);
            ir_buffer[idx++] = (uint8_t)((ir_values[i] >> 8) & 0xFF);
        }

        ir_buffer[idx++] = sensor_snapshot.detection_flags;

        UNERBUS_Write(aBus, ir_buffer, IR_SENSOR_SNAPSHOT_BYTES);
        length = UNERBUS_CMD_ID_SIZE + IR_SENSOR_SNAPSHOT_BYTES;
    }
        break;
    case CMD_CALIBRATE_MPU:            // Calibrar el MPU6050
        Prepare_MPU_BlockingTransaction();
        if (MPU6050_Calibrate(&hmpu, 200) != MPU6050_OK) // Calibrar con 200 muestras (ajustable)
        {
            Error_Handler();
        }
        Reset_Yaw_Tracking();
        MPU_READ_REQUEST = false;
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
            Prepare_MPU_BlockingTransaction();
            hmpu.accel_range = new_accel_range;
            hmpu.gyro_range = new_gyro_range;
            hmpu.dlpf_config = new_dlpf_config;

            // Re-inicializar el MPU para aplicar la nueva configuración
            if (MPU6050_Init(&hmpu) != MPU6050_OK)
            {
                Error_Handler();
            }

            // Actualizar el escalador del giroscopio con la nueva configuración
            Update_Gyro_Scaler();
            Reset_Yaw_Tracking();
            MPU_READ_REQUEST = false;
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

        // Convertir de entero x100 a punto fijo.
        // Se usa 100 para ampliar el rango de Kp hasta ~655
        Set_Pid_Gains_From_U16(PID_ROLE_CENTERING, kp_int, ki_int, kd_int, false);
        break;
    case CMD_GET_PID_GAINS: // Leer Kp, Ki, Kd
        uint8_t response_buffer[UNERBUS_PID_GAINS_SIZE];

        // Convertir de punto fijo a entero para enviar (multiplicando por 100)
        Write_Pid_Gains_To_Buffer(PID_ROLE_CENTERING, response_buffer);

        UNERBUS_Write(aBus, response_buffer, UNERBUS_PID_GAINS_SIZE);
        length = UNERBUS_CMD_ID_SIZE + UNERBUS_PID_GAINS_SIZE;
        break;
    case CMD_SET_MAX_PWM_CORRECTION: // Configurar la corrección máxima del PWM
        // Se espera 1 valor uint16_t
        max_pwm_correction = UNERBUS_GetUInt16(aBus);

        // Actualizar la configuración del PID con los nuevos valores
        pid_configs[PID_ROLE_CENTERING].out_min = INT_TO_FIXED(-max_pwm_correction);
        pid_configs[PID_ROLE_CENTERING].out_max = INT_TO_FIXED(max_pwm_correction);
        Apply_Pid_Config(PID_ROLE_CENTERING, false);
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

        // Convertir de entero x100 a punto fijo.
        Set_Pid_Gains_From_U16(PID_ROLE_TURN, turn_kp_int, turn_ki_int, turn_kd_int, false);
        break;
    case CMD_GET_TURN_PID_GAINS: // Leer Kp, Ki, Kd del PID de giro
        uint8_t turn_pid_buffer[UNERBUS_TURN_PID_GAINS_SIZE];

        // Convertir de punto fijo a entero para enviar (multiplicando por 100)
        Write_Pid_Gains_To_Buffer(PID_ROLE_TURN, turn_pid_buffer);

        UNERBUS_Write(aBus, turn_pid_buffer, UNERBUS_TURN_PID_GAINS_SIZE);
        length = UNERBUS_CMD_ID_SIZE + UNERBUS_TURN_PID_GAINS_SIZE;
        break;
    case CMD_SET_TURN_MAX_SPEED:
        turn_max_pwm = UNERBUS_GetUInt16(aBus);
        if (turn_max_pwm > pwm_max_value)
            turn_max_pwm = pwm_max_value;

        pid_configs[PID_ROLE_TURN].out_min = INT_TO_FIXED(-turn_max_pwm);
        pid_configs[PID_ROLE_TURN].out_max = INT_TO_FIXED(turn_max_pwm);
        Apply_Pid_Config(PID_ROLE_TURN, false);

        pid_configs[PID_ROLE_SMOOTH_TURN].out_min = INT_TO_FIXED(-turn_max_pwm);
        pid_configs[PID_ROLE_SMOOTH_TURN].out_max = INT_TO_FIXED(turn_max_pwm);
        Apply_Pid_Config(PID_ROLE_SMOOTH_TURN, false);
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
            Reset_Yaw_Tracking();
            // Iniciar la máquina de estados del robot si el modo es activo.
            // Esto replica el comportamiento del botón físico.
            if (menu_mode == MENU_MODE_FIND_CELLS || menu_mode == MENU_MODE_GO_TO_B)
            {
                Set_Robot_State(STATE_NAVIGATING); // Aquí se inicia el movimiento
                kick_start_active = true;
                motion_confirm_counter = 0;

                if (menu_mode == MENU_MODE_FIND_CELLS)
                {
                    Reset_Maze_State();
                }
                else if (menu_mode == MENU_MODE_GO_TO_B)
                {
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
        Set_Pid_Gains_From_U16(PID_ROLE_BRAKING, kp_int, ki_int, kd_int, false);
        break;
    case CMD_GET_BRAKING_PID_GAINS:
        uint8_t braking_pid_buffer[UNERBUS_BRAKING_PID_GAINS_SIZE];
        Write_Pid_Gains_To_Buffer(PID_ROLE_BRAKING, braking_pid_buffer);
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
        pid_configs[PID_ROLE_BRAKING].out_min = INT_TO_FIXED(-braking_max_pwm_offset);
        pid_configs[PID_ROLE_BRAKING].out_max = INT_TO_FIXED(braking_max_pwm_offset);
        Apply_Pid_Config(PID_ROLE_BRAKING, false);
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
        Set_Pid_Gains_From_U16(PID_ROLE_SMOOTH_TURN, vel_kp_int, vel_ki_int, vel_kd_int, false);
        break;
    case CMD_GET_TURN_VELOCITY_PID_GAINS:
        uint8_t vel_pid_buffer[UNERBUS_TURN_VELOCITY_PID_GAINS_SIZE];
        Write_Pid_Gains_To_Buffer(PID_ROLE_SMOOTH_TURN, vel_pid_buffer);
        UNERBUS_Write(aBus, vel_pid_buffer, UNERBUS_TURN_VELOCITY_PID_GAINS_SIZE);
        length = UNERBUS_CMD_ID_SIZE + UNERBUS_TURN_VELOCITY_PID_GAINS_SIZE;
        break;
    case CMD_SET_TURN_TARGET_DPS:
        turn_target_dps = UNERBUS_GetUInt16(aBus);

        if (robot_state == STATE_SMOOTH_TURN_LEFT)
        {
            PID_Set_Setpoint(&turn_velocity_pid, turn_target_dps);
        }
        else if (robot_state == STATE_SMOOTH_TURN_RIGHT)
        {
            PID_Set_Setpoint(&turn_velocity_pid, -turn_target_dps);
        }
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
    case CMD_SYNC_MAZE_COLUMN:
    {
        uint8_t requested_col = UNERBUS_GetUInt8(aBus);
        uint8_t col_buffer[APP_MAZE_COLUMN_SYNC_PAYLOAD_SIZE];
        uint8_t payload_len = App_Maze_WriteColumnSyncPayload(requested_col, col_buffer);

        if (payload_len != 0U)
        {
            UNERBUS_Write(aBus, col_buffer, payload_len);
            length = UNERBUS_CMD_ID_SIZE + payload_len;
        }
        break;
    }
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

    UNERBUS_MoveIndexRead(aBus, iStartData);
}

void Do1ms(void)
{
    ADC_Filter_Task();
}

void Do10ms(void)
{
    Button_Tick(&h_user_button);

    ESP01_Timeout10ms();
    UNERBUS_Timeout(&unerbus_esp01_handle);
    UNERBUS_Timeout(&unerbus_pc_handle);
}

void Do100ms(void)
{
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

static void Prepare_MPU_BlockingTransaction(void)
{
    MPU_READ_REQUEST = false;

    while (i2c_bus_state != I2C_BUS_IDLE)
    {
    }

    MPU_READ_REQUEST = false;
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
                temporary_heartbeat_ticks = 5; // Duracion del feedback: 5 ticks de 100 ms.
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
                Reset_Yaw_Tracking();
                Set_Robot_State((menu_mode == MENU_MODE_FIND_CELLS) ? STATE_NAVIGATING : STATE_IDLE);
                if (robot_state == STATE_NAVIGATING)
                {
                    kick_start_active = true;
                    motion_confirm_counter = 0;

                    if (menu_mode == MENU_MODE_FIND_CELLS)
                    {
                        Reset_Maze_State();
                    }
                    else if (menu_mode == MENU_MODE_GO_TO_B)
                    {
                        Reset_Robot_Position();
                    }

                    pending_initial_cell_seed = true;
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
    Init_Cycle_Counter();
    App_Timebase_Init(&app_timebase, &app_timebase_config);
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

    /* Sensores */
    ADC_LUT_Precompute();

    /* Timers */
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
    Init_Pid_Configs();
    Apply_Pid_Config(PID_ROLE_CENTERING, true);
    PID_Set_Setpoint(&centering_pid, 0);                                                        // El setpoint se ajustará dinámicamente

    /* --- INICIALIZACIÓN DEL PID DE FRENADO --- */
    braking_max_pwm_offset = BRAKING_MAX_SPEED_DEFAULT;
    Apply_Pid_Config(PID_ROLE_BRAKING, true);
    PID_Set_Setpoint(&braking_pid, wall_braking_target_mm);
    // La salida es la velocidad, así que el límite es el PWM máximo.

    /* --- INICIALIZACIÓN DEL PID DE GIRO --- */
    Apply_Pid_Config(PID_ROLE_TURN, true);

    /* --- NUEVO: INICIALIZACIÓN DEL PID DE VELOCIDAD DE GIRO --- */
    Apply_Pid_Config(PID_ROLE_SMOOTH_TURN, true);
    // La salida de este PID ES la potencia del motor, así que sus límites son los límites de PWM.
    PID_Set_Setpoint(&turn_velocity_pid, turn_target_dps);

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

    /* UART */
    HAL_UART_Receive_IT(&huart1, &data_rx_esp01, 1);

    /* Flags */
    App_Timebase_Reset(&app_timebase);
    MPU_READ_REQUEST = false;
    SSD_UPDATE_REQUEST = false;
    UART_BYPASS = false;

    /* Estados de la aplicación */
    app_state = APP_STATE_MENU;
    menu_mode = MENU_MODE_IDLE;

    HAL_TIM_Base_Start_IT(&htim1);
}

static void Run_Control_Step(uint32_t dt_ms)
{
    control_step_dt_ms = dt_ms;
    ADC_Filter_Task();
    Update_Navigation_Perception();

    Modes_State_Machine();
}

void App_Core_Loop(void)
{
    uint8_t pending_1ms;
    uint8_t pending_10ms;
    uint8_t pending_100ms;
    uint8_t pending_control;

    ManageButtonEvents();

    if (!timeout_alive_udp && !UART_BYPASS)
    {
        timeout_alive_udp = ALIVE_UDP_PERIOD_COUNT;
        UNERBUS_WriteByte(&unerbus_esp01_handle, CMD_ACK);
        UNERBUS_Send(&unerbus_esp01_handle, CMD_GET_ALIVE, UNERBUS_CMD_ID_SIZE + UNERBUS_ACK_SIZE);
    }

    pending_1ms = Consume_Timebase_Event(APP_TIMEBASE_EVENT_1MS);
    while (pending_1ms > 0U)
    {
        Do1ms();
        pending_1ms--;
    }

    pending_10ms = Consume_Timebase_Event(APP_TIMEBASE_EVENT_10MS);
    while (pending_10ms > 0U)
    {
        Do10ms();
        pending_10ms--;
    }

    pending_100ms = Consume_Timebase_Event(APP_TIMEBASE_EVENT_100MS);
    while (pending_100ms > 0U)
    {
        Do100ms();
        pending_100ms--;
    }

    pending_control = Consume_Timebase_Event(APP_TIMEBASE_EVENT_CONTROL);
    if (pending_control > 0U)
    {
        Run_Control_Step((uint32_t)pending_control * CONTROL_PERIOD_MS);
    }

    ManageI2CTransactions();

    ManageTransmission();

    ESP01_Task();

    UNERBUS_Task(&unerbus_esp01_handle);
    UNERBUS_Task(&unerbus_pc_handle);
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
        Reset_Yaw_Tracking(); // Reseteamos la medición de ángulo para un giro relativo.

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

// Actualiza la orientación luego de un giro.
// turn_direction: 1 (Giro Derecha), -1 (Giro Izquierda), 2 (Media Vuelta)
static void Update_Robot_Heading(int8_t turn_direction)
{
    App_Maze_UpdateRobotHeading((TurnTypeDef)turn_direction);
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
        Commit_Maze_State(TURN_AROUND, false, true);

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
    int16_t gz = sensor_snapshot.gz;

    // Convertir el valor raw del giroscopio (gz) a grados por segundo (dps).
    int16_t angular_velocity_dps = GyroRaw_To_Dps(gz);

    // 4. Establecer el setpoint del PID de giro a la velocidad angular deseada.
    PID_Set_Setpoint(&turn_pid, target_dps);

    // 5. Calcular la salida del PID. La entrada es la velocidad angular actual.
    //    La salida es la "fuerza" de giro (un valor de PWM).
    int32_t pid_output_fixed = PID_Update(&turn_pid, angular_velocity_dps, control_step_dt_ms);
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
    uint16_t new_sensitivity_x10;
    uint32_t primask;

    switch (hmpu.gyro_range)
    {
    case MPU6050_GYRO_RANGE_250DPS:
        new_sensitivity_x10 = GYRO_SENSITIVITY_X10_250DPS;
        break;
    case MPU6050_GYRO_RANGE_500DPS:
        new_sensitivity_x10 = GYRO_SENSITIVITY_X10_500DPS;
        break;
    case MPU6050_GYRO_RANGE_1000DPS:
        new_sensitivity_x10 = GYRO_SENSITIVITY_X10_1000DPS;
        break;
    case MPU6050_GYRO_RANGE_2000DPS:
        new_sensitivity_x10 = GYRO_SENSITIVITY_X10_2000DPS;
        break;
    default:
        // Caso por defecto seguro
        new_sensitivity_x10 = GYRO_SENSITIVITY_X10_500DPS;
        break;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    gyro_sensitivity_x10 = new_sensitivity_x10;
    mpu_yaw_timing_initialized = false;
    if (primask == 0U)
    {
        __enable_irq();
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
    App_Sensors_ProcessAdcSamples();
}

/**
 * @brief Devuelve el promedio móvil precalculado del canal.
 */
static int32_t Get_Filtered_ADC_Value(uint8_t channel)
{
    return (int32_t)App_Sensors_GetFilteredAdcValue(channel);
}

// Actualiza la posición (x, y) asumiendo que el robot avanzó 1 celda hacia el frente
static void Update_Robot_Position(void)
{
    App_Maze_AdvanceRobotPosition();
}

static void Current_Cell_Mapping(void)
{
    App_Maze_MapCurrentCell(front_wall_detected, right_wall_detected, left_wall_detected);
}

static void Send_Maze_Cell_Update(void)
{
    App_Maze_SendCurrentCellUpdate(&unerbus_esp01_handle);
}

static void Reset_Robot_Position(void)
{
    App_Maze_ResetRobotPosition();
}

static void Reset_Maze_State(void)
{
    App_Maze_ResetState();
}

static void Handle_Navigating(void)
{
    adc_rear_floor = sensor_snapshot.adc_filtered[SENSOR_FLOOR_REAR_CH];
    bool current_rear_tape = ((sensor_snapshot.detection_flags & SENSOR_DET_FLOOR_REAR) != 0U);

    if (current_rear_tape && !was_rear_tape_detected)
    {
        rear_tape_detected = true;
        was_rear_tape_detected = true;
    }
    else if (!current_rear_tape)
    {
        was_rear_tape_detected = false;
    }

    uint16_t front_avg_mm = (uint16_t)((dist_front_left_mm + dist_front_right_mm) / 2);

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
        if (!left_diagonal_wall_detected && !right_diagonal_wall_detected)
        {
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
    else
    {
        wall_diagonal_faded = NO_WALL_FADED;
    }

    if (rear_tape_detected)
    {
        Update_Robot_Position();
        Commit_Maze_State(false, true, true);

        // 4. Evaluamos si veníamos esperando esta línea para tomar una decisión
        if (wall_diagonal_faded)
        {
            rear_tape_detected = false;
            wall_diagonal_faded = NO_WALL_FADED;
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
        int16_t ax = sensor_snapshot.ax;
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
    current_left_base_speed = wall_diagonal_faded ? ((uint32_t)current_left_base_speed * 90) / 100 : current_left_base_speed;
    current_right_base_speed = wall_diagonal_faded ? ((uint32_t)current_right_base_speed * 90) / 100 : current_right_base_speed;

    int32_t pid_output_fixed = 0;

    if (left_diagonal_wall_detected && right_diagonal_wall_detected && left_wall_detected && right_wall_detected)
    {
        int32_t measured_diff = dist_left_lat_mm - dist_right_lat_mm;
        PID_Set_Setpoint(&centering_pid, 0);
        pid_output_fixed = PID_Update(&centering_pid, measured_diff, control_step_dt_ms);
    }
    else if (right_diagonal_wall_detected && right_wall_detected)
    {
        /*         PID_Set_Setpoint(&centering_pid, wall_target_mm);
                pid_output_fixed = PID_Update(&centering_pid, dist_right_lat_mm, control_step_dt_ms);
                pid_output_fixed = -pid_output_fixed; */

        int32_t measured_diff = (wall_target_mm - dist_right_lat_mm) * 2; // Se multiplica por 2 para dar más peso a la corrección al perder la pared diagonal, ya que solo queda una referencia.
        PID_Set_Setpoint(&centering_pid, 0);
        pid_output_fixed = PID_Update(&centering_pid, measured_diff, control_step_dt_ms);
    }
    else if (left_diagonal_wall_detected && left_wall_detected)
    {
        /*         PID_Set_Setpoint(&centering_pid, wall_target_mm);
                pid_output_fixed = PID_Update(&centering_pid, dist_left_lat_mm, control_step_dt_ms); */

        int32_t measured_diff = (dist_left_lat_mm - wall_target_mm) * 2; // Se multiplica por 2 para dar más peso a la corrección al perder la pared diagonal, ya que solo queda una referencia.
        PID_Set_Setpoint(&centering_pid, 0);
        pid_output_fixed = PID_Update(&centering_pid, measured_diff, control_step_dt_ms);
    }
    else
    {
        // CASO 4: Sin paredes.
        Set_Motor_Speeds(0, 0); 
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
        adc_rear_floor = sensor_snapshot.adc_filtered[SENSOR_FLOOR_REAR_CH];
        bool current_rear_tape = ((sensor_snapshot.detection_flags & SENSOR_DET_FLOOR_REAR) != 0U);
        if (current_rear_tape && !was_rear_tape_detected)
        {
            rear_tape_detected = true;
        }
        was_rear_tape_detected = current_rear_tape;

        if (rear_tape_detected)
        {
            if (have_to_decide)
            {
                rear_tape_detected = false;
                was_rear_tape_detected = true; // Bloqueamos para no volver a disparar en la misma cinta

                Update_Robot_Position();
                Commit_Maze_State(0, true, true);

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
    // El dt lo define el evento de control de la base de tiempo.
    int16_t pwm_correction = (int16_t)(FIXED_TO_INT(PID_Update(&centering_pid, FIXED_TO_INT(current_yaw_fixed), control_step_dt_ms)));

    // Aplicar la corrección a la velocidad base de los motores
    Set_Motor_Speeds(right_motor_base_speed - pwm_correction, left_motor_base_speed + pwm_correction);
}

static void Handle_Deciding(void)
{
    uint8_t available_Options = 0;
    uint8_t validOptions[4] = {0, 0, 0, 0};
    uint8_t validOptionsCounter = 0;
    enum Direcciones
    {
        ATRAS,
        ADELANTE,
        DERECHA,
        IZQUIERDA
    };

    // Las distancias y las banderas de pared ya vienen actualizadas por
    // Update_Navigation_Perception() antes de entrar en la máquina de estados.

    // Analizar las opciones (asumiendo que 1 siempre es "atrás" y está libre)
    // Bit 0: Atrás (Siempre 1)
    // Bit 1: Adelante
    // Bit 2: Derecha
    // Bit 3: Izquierda
    available_Options = ((!left_wall_detected) << 3) |
                     ((!right_wall_detected) << 2) |
                     ((!front_wall_detected) << 1) |
                     (1 << 0); // El camino de atrás siempre suele estar libre

    // Si solo está disponible el camino hacia atrás
    if (available_Options == 1)
    {
        Turn_Start(180); // Callejón sin salida
        return;
    }

    for (uint8_t i = 1; i < 4; i++)
    {
        // Verificamos si el bit 'i' está encendido
        if (available_Options & (1 << i))
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
        Reset_Yaw_Tracking();
        PID_Reset(&turn_velocity_pid);
        PID_Set_Setpoint(&turn_velocity_pid, turn_target_dps); // Valores positivos de gz para giro a la izquierda
    }
    else if (choice == DERECHA)
    {
        Set_Robot_State(STATE_SMOOTH_TURN_RIGHT);
        Reset_Yaw_Tracking();
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
    uint16_t dist_front_avg_mm = (uint16_t)((sensor_snapshot.dist_front_left_mm +
                                             sensor_snapshot.dist_front_right_mm) /
                                            2U);

    // 2. Comprobar si el frenado ha terminado
    int16_t ax = sensor_snapshot.ax;

    // La condición de parada usa el error absoluto en mm y la aceleración
    if (abs(dist_front_avg_mm - wall_braking_target_mm) < braking_dead_zone && abs(ax) < braking_accel_stop_threshold)
    {
        Set_Motor_Speeds(0, 0);
        Set_Robot_State(STATE_DECIDING);
        return;
    }

    // 3. Calcular la salida del PID. El setpoint ya está configurado.
    int32_t pid_output_fixed = PID_Update(&braking_pid, dist_front_avg_mm, control_step_dt_ms);

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
}

/**
 * @brief Precalcula las pendientes de los segmentos de la LUT de ADC.
 * @note  Debe llamarse una sola vez durante la inicialización.
 *        Rellena el array global `adc_lut_segs`.
 * @retval None
 */
static void ADC_LUT_Precompute(void)
{
    App_Sensors_Init();
}

/**
 * @brief Convierte un valor de ADC a distancia en milímetros usando una LUT con pendientes precalculadas.
 * @param  adc_value: El valor de 12 bits del ADC a convertir.
 * @retval La distancia estimada en milímetros (int32_t).
 * @note   Utiliza búsqueda binaria y aritmética de punto fijo. No usa 'float'.
 */
static int32_t ADC_To_Distance_mm(uint16_t adc_value)
{
    return (int32_t)App_Sensors_ConvertAdcToDistanceMm(adc_value);
}

static bool Detect_Low_With_Hysteresis(uint16_t value, uint16_t threshold, uint16_t hysteresis, bool was_detected)
{
    uint32_t release_threshold = (uint32_t)threshold + (uint32_t)hysteresis;

    if (was_detected)
    {
        return ((uint32_t)value < release_threshold);
    }

    return (value < threshold);
}

static bool Detect_Front_Wall_With_Hysteresis(uint16_t left_value, uint16_t right_value, uint16_t threshold, uint16_t hysteresis, bool was_detected)
{
    uint32_t release_threshold = (uint32_t)threshold + (uint32_t)hysteresis;

    if (was_detected)
    {
        return (((uint32_t)left_value < release_threshold) && ((uint32_t)right_value < release_threshold));
    }

    return ((left_value < threshold) && (right_value < threshold));
}

static void Sync_Legacy_Perception_From_Snapshot(void)
{
    uint8_t flags = sensor_snapshot.detection_flags;

    dist_diagonal_left_mm = sensor_snapshot.dist_diagonal_left_mm;
    dist_diagonal_right_mm = sensor_snapshot.dist_diagonal_right_mm;
    dist_front_left_mm = sensor_snapshot.dist_front_left_mm;
    dist_front_right_mm = sensor_snapshot.dist_front_right_mm;
    dist_left_lat_mm = sensor_snapshot.dist_left_lat_mm;
    dist_right_lat_mm = sensor_snapshot.dist_right_lat_mm;
    adc_front_floor = sensor_snapshot.adc_filtered[SENSOR_FLOOR_FRONT_CH];
    adc_rear_floor = sensor_snapshot.adc_filtered[SENSOR_FLOOR_REAR_CH];

    front_wall_detected = ((flags & SENSOR_DET_WALL_FRONT) != 0U);
    left_wall_detected = ((flags & SENSOR_DET_WALL_LEFT) != 0U);
    right_wall_detected = ((flags & SENSOR_DET_WALL_RIGHT) != 0U);
    left_diagonal_wall_detected = ((flags & SENSOR_DET_WALL_DIAG_LEFT) != 0U);
    right_diagonal_wall_detected = ((flags & SENSOR_DET_WALL_DIAG_RIGHT) != 0U);
    front_tape_detected = ((flags & SENSOR_DET_FLOOR_FRONT) != 0U);
}

/**
 * @brief Maneja el estado de giro suave en intersecciones.
 *
 */
static void Handle_Smooth_Turn(void)
{
    bool wall_detected = false;
    int16_t base_right = 0, base_left = 0;

    adc_rear_floor = sensor_snapshot.adc_filtered[SENSOR_FLOOR_REAR_CH];
    bool current_rear_tape = ((sensor_snapshot.detection_flags & SENSOR_DET_FLOOR_REAR) != 0U);

    // Si vemos blanco, el robot ha salido completamente de cualquier cinta previa
    if (!current_rear_tape) 
    {
        was_rear_tape_detected = false;
    }
    // Si vemos negro, no lo habíamos procesado, y ya giramos un umbral seguro (> 45)
    else if (!was_rear_tape_detected && (abs(FIXED_TO_INT(current_yaw_fixed)) > 45)) 
    {
        rear_tape_detected = true;
        was_rear_tape_detected = true; // Lo bloqueamos para que no se dispare repetidas veces
    }

    if (robot_state == STATE_SMOOTH_TURN_LEFT)
    {
        wall_detected = (dist_diagonal_left_mm < after_turn_wall_threshold_mm);
        base_right = (int16_t)faster_motor_smooth_turn_speed; // exterior
        base_left = (int16_t)slower_motor_smooth_turn_speed;  // interior
    }
    else if (robot_state == STATE_SMOOTH_TURN_RIGHT)
    {
        wall_detected = (dist_diagonal_right_mm < after_turn_wall_threshold_mm);
        base_right = (int16_t)slower_motor_smooth_turn_speed; // interior
        base_left = (int16_t)faster_motor_smooth_turn_speed;  // exterior
    }

    if (wall_detected || (abs(FIXED_TO_INT(current_yaw_fixed)) >= (90 - TURN_COMPLETION_DEAD_ZONE)) || rear_tape_detected)
    {
        RobotStateTypeDef completed_turn_state = robot_state;

        int8_t heading_update = 0;

        if (completed_turn_state == STATE_SMOOTH_TURN_LEFT)
            heading_update = TURN_LEFT;
        else if (completed_turn_state == STATE_SMOOTH_TURN_RIGHT)
            heading_update = TURN_RIGHT;

        if (rear_tape_detected)
        {
            Update_Robot_Position();
            Commit_Maze_State(heading_update, true, true);

            rear_tape_detected = false;
            was_rear_tape_detected = true;
        }
        else
        {
            Commit_Maze_State(heading_update, false, false);
        }


        Set_Motor_Speeds(right_motor_base_speed, left_motor_base_speed);
        Set_Robot_State(STATE_NAVIGATING);
        PID_Reset(&centering_pid);
        kick_start_active = true;
        motion_confirm_counter = 0;

        return;
    }

    int16_t gz = sensor_snapshot.gz;

    int16_t angular_velocity_dps = GyroRaw_To_Dps(gz);

    int32_t pid_output_fixed = PID_Update(&turn_velocity_pid, angular_velocity_dps, control_step_dt_ms);
    int16_t correction = (int16_t)FIXED_TO_INT(pid_output_fixed);

    int16_t right_speed = base_right + correction;
    int16_t left_speed = base_left - correction;

    Set_Motor_Speeds(right_speed, left_speed);
}

static void Update_Navigation_Perception(void)
{
    uint8_t previous_flags = sensor_snapshot.detection_flags;
    uint8_t new_flags = 0;

    for (uint8_t ch = 0; ch < ADC_CHANNELS; ch++)
    {
        sensor_snapshot.adc_filtered[ch] = (uint16_t)Get_Filtered_ADC_Value(ch);
    }

    sensor_snapshot.dist_diagonal_left_mm = (uint16_t)ADC_To_Distance_mm(sensor_snapshot.adc_filtered[SENSOR_DIAGONAL_LEFT_CH]);
    sensor_snapshot.dist_diagonal_right_mm = (uint16_t)ADC_To_Distance_mm(sensor_snapshot.adc_filtered[SENSOR_DIAGONAL_RIGHT_CH]);
    sensor_snapshot.dist_front_left_mm = (uint16_t)ADC_To_Distance_mm(sensor_snapshot.adc_filtered[SENSOR_FRONT_LEFT_CH]);
    sensor_snapshot.dist_front_right_mm = (uint16_t)ADC_To_Distance_mm(sensor_snapshot.adc_filtered[SENSOR_FRONT_RIGHT_CH]);
    sensor_snapshot.dist_left_lat_mm = (uint16_t)ADC_To_Distance_mm(sensor_snapshot.adc_filtered[SENSOR_LEFT_LAT_CH]);
    sensor_snapshot.dist_right_lat_mm = (uint16_t)ADC_To_Distance_mm(sensor_snapshot.adc_filtered[SENSOR_RIGHT_LAT_CH]);

    MPU6050_GetCalibratedData(&hmpu, &sensor_snapshot.ax, &sensor_snapshot.ay, &sensor_snapshot.az, &sensor_snapshot.gx, &sensor_snapshot.gy, &sensor_snapshot.gz);
    sensor_snapshot.yaw_fixed = current_yaw_fixed;

    if (Detect_Front_Wall_With_Hysteresis(sensor_snapshot.dist_front_left_mm,
                                          sensor_snapshot.dist_front_right_mm,
                                          wall_threshold_mm_front,
                                          WALL_HYSTERESIS_MM,
                                          ((previous_flags & SENSOR_DET_WALL_FRONT) != 0U)))
    {
        new_flags |= SENSOR_DET_WALL_FRONT;
    }

    if (Detect_Low_With_Hysteresis(sensor_snapshot.dist_left_lat_mm,
                                   wall_threshold_mm_side,
                                   WALL_HYSTERESIS_MM,
                                   ((previous_flags & SENSOR_DET_WALL_LEFT) != 0U)))
    {
        new_flags |= SENSOR_DET_WALL_LEFT;
    }

    if (Detect_Low_With_Hysteresis(sensor_snapshot.dist_right_lat_mm,
                                   wall_threshold_mm_side,
                                   WALL_HYSTERESIS_MM,
                                   ((previous_flags & SENSOR_DET_WALL_RIGHT) != 0U)))
    {
        new_flags |= SENSOR_DET_WALL_RIGHT;
    }

    if (Detect_Low_With_Hysteresis(sensor_snapshot.dist_diagonal_left_mm,
                                   wall_threshold_mm_diagonal,
                                   WALL_HYSTERESIS_MM,
                                   ((previous_flags & SENSOR_DET_WALL_DIAG_LEFT) != 0U)))
    {
        new_flags |= SENSOR_DET_WALL_DIAG_LEFT;
    }

    if (Detect_Low_With_Hysteresis(sensor_snapshot.dist_diagonal_right_mm,
                                   wall_threshold_mm_diagonal,
                                   WALL_HYSTERESIS_MM,
                                   ((previous_flags & SENSOR_DET_WALL_DIAG_RIGHT) != 0U)))
    {
        new_flags |= SENSOR_DET_WALL_DIAG_RIGHT;
    }

    if (Detect_Low_With_Hysteresis(sensor_snapshot.adc_filtered[SENSOR_FLOOR_FRONT_CH],
                                   tape_detection_threshold_adc,
                                   TAPE_HYSTERESIS_ADC,
                                   ((previous_flags & SENSOR_DET_FLOOR_FRONT) != 0U)))
    {
        new_flags |= SENSOR_DET_FLOOR_FRONT;
    }

    if (Detect_Low_With_Hysteresis(sensor_snapshot.adc_filtered[SENSOR_FLOOR_REAR_CH],
                                   tape_detection_threshold_adc,
                                   TAPE_HYSTERESIS_ADC,
                                   ((previous_flags & SENSOR_DET_FLOOR_REAR) != 0U)))
    {
        new_flags |= SENSOR_DET_FLOOR_REAR;
    }

    sensor_snapshot.detection_flags = new_flags;
    Sync_Legacy_Perception_From_Snapshot();
}

static void Commit_Maze_State(int8_t heading_update, bool update_cell, bool send_update)
{
    if (heading_update != 0)
    {
        Update_Robot_Heading(heading_update);
    }

    if (update_cell)
    {
        Current_Cell_Mapping();
    }

    if (send_update)
    {
        Send_Maze_Cell_Update();
    }
}


static void Modes_State_Machine(void)
{
    // Maquina de estados del robot
    if (app_state == APP_STATE_RUNNING)
    {
        switch (menu_mode)
        {
        case MENU_MODE_FIND_CELLS:
        	Motion_ExecuteAction(&navigation_motion);
        	Navigation_DecideMovement(&navigation_motion);
//            // Ejecutar la lógica de resolución de laberintos
//            switch (robot_state)
//            {
//            case STATE_NAVIGATING:
//                Handle_Navigating();
//                break;
//            case STATE_BRAKING:
//                Handle_Braking();
//                break;
//            case STATE_DECIDING:
//                Handle_Deciding();
//                break;
//            case STATE_LEFT_WALL_FADE:
//            case STATE_RIGHT_WALL_FADE:
//            case STATE_STRAIGHT_DRIVE:
//                Handle_Straight_Drive(false);
//                break;
//            case STATE_STRAIGHT_DRIVE_DESIDING:
//                Handle_Straight_Drive(true);
//                break;
//            case STATE_TURNING_LEFT:
//            case STATE_TURNING_RIGHT:
//            case STATE_TURN_AROUND_LEFT:
//            case STATE_TURN_AROUND_RIGHT:
//                Manage_Turn();
//                break;
//            case STATE_SMOOTH_TURN_LEFT:
//            case STATE_SMOOTH_TURN_RIGHT:
//                Handle_Smooth_Turn();
//                break;
//            default:
//                Handle_Idle();
//                break;
//            }
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

static void Navigation_DecideMovement(MotionContext *motion)
{
    if (motion->action_active)
    {
        return;
    }

    if (!navigation_first_execution)
    {
        if (!motion->result_pending)
        {
            return;
        }

        if (motion->result != MOTION_RESULT_NEW_CELL_REACHED)
        {
            Set_Motor_Speeds(0, 0);
            Set_Robot_State(STATE_IDLE);
            return;
        }

        motion->result_pending = false;
        motion->result = MOTION_RESULT_NONE;

        Update_Robot_Position();
        Commit_Maze_State(false, true, true);
    }
    else
    {
        navigation_first_execution = false;
        Commit_Maze_State(false, true, true);
    }

    MotionAction next_action = Navigation_SelectNextAction();
    Motion_StartAction(motion, next_action);
}

static MotionAction Navigation_SelectNextAction(void)
{
    uint8_t available_Options = 0;
	uint8_t validOptions[4] = {0, 0, 0, 0};
	uint8_t validOptionsCounter = 0;
	enum Direcciones
	{
		ATRAS,
		ADELANTE,
		DERECHA,
		IZQUIERDA
	};

	// Las distancias y las banderas de pared ya vienen actualizadas por
	// Update_Navigation_Perception() antes de entrar en la máquina de estados.

	// Analizar las opciones (asumiendo que 1 siempre es "atrás" y está libre)
	// Bit 0: Atrás (Siempre 1)
	// Bit 1: Adelante
	// Bit 2: Derecha
	// Bit 3: Izquierda
	available_Options = ((!left_wall_detected) << 3) |
					 ((!right_wall_detected) << 2) |
					 ((!front_wall_detected) << 1) |
					 (1 << 0); // El camino de atrás siempre suele estar libre

	// Si solo está disponible el camino hacia atrás
	if (available_Options == 1)
	{
		// Callejón sin salida, desarrollar que hacer
		return ACTION_GO_BACK;
	}

	for (uint8_t i = 1; i < 4; i++)
	{
		// Verificamos si el bit 'i' está encendido
		if (available_Options & (1 << i))
		{
			validOptions[validOptionsCounter] = i;
			validOptionsCounter++;
		}
	}

	// Elegimos una opción al azar:
	uint8_t choice = validOptions[rand() % validOptionsCounter];

	if (choice == IZQUIERDA)
	{
		return ACTION_GO_LEFT_SMOOTH;
	}
	else if (choice == DERECHA)
	{
		return ACTION_GO_RIGHT_SMOOTH;
	}
	else if (choice == ADELANTE)
	{
		return ACTION_GO_FRONT;
	}

	return ACTION_NONE;
}

uint8_t Motion_StartAction(MotionContext *motion, MotionAction action)
{
    if (motion == NULL)
    {
        return 0;
    }

    if (motion->action_active)
    {
        return 0;
    }

    motion->active_action = action;
    motion->action_active = true;

    motion->result = MOTION_RESULT_NONE;
    motion->result_pending = false;

    motion->line_detector_armed = false;
    motion->waiting_leave_current_line = true;

    motion->active_pid = NULL;

    switch (action)
    {
		case ACTION_GO_FRONT:
			motion->phase = MOTION_STRAIGHT_ADVANCE;
			motion->straight_ref = STRAIGHT_REF_NONE;
			Reset_Yaw_Tracking(); // Por las dudas

			// Si el PID corrige diferencia lateral, el setpoint es 0.
			return Motion_SetPID(motion, PID_ROLE_CENTERING, 0);

		case ACTION_GO_LEFT_SMOOTH:
			motion->phase = MOTION_SMOOTH_TURN;
			Reset_Yaw_Tracking();

			return Motion_SetPID(motion, PID_ROLE_SMOOTH_TURN, turn_target_dps);

		case ACTION_GO_RIGHT_SMOOTH:
			motion->phase = MOTION_SMOOTH_TURN;
			Reset_Yaw_Tracking();

			return Motion_SetPID(motion, PID_ROLE_SMOOTH_TURN, -((int32_t)turn_target_dps));

		case ACTION_GO_BACK:
			motion->phase = MOTION_PIVOT_TURN;
			Reset_Yaw_Tracking();

			return Motion_SetPID(motion, PID_ROLE_TURN, pivot_turn_target_dps);

		case ACTION_NONE:
		default:
			motion->active_action = ACTION_NONE;
			motion->action_active = false;
			motion->phase = MOTION_IDLE;
			motion->active_pid = NULL;
			return 0;
    }
}

static uint8_t Motion_SetPID(MotionContext *motion, PID_Role_t role, int32_t setpoint)
{
    if (motion == NULL || role >= PID_ROLE_COUNT)
    {
        return 0;
    }

    PID_Controller_t *pid = pid_instances[role];

    PID_Reset(pid);
    PID_Set_Setpoint(pid, setpoint);

    motion->active_pid = pid;

    return 1;
}

static void Motion_ExecuteAction(MotionContext *motion)
{
    if (!motion->action_active)
    {
        Set_Motor_Speeds(0, 0);
        return;
    }

    switch (motion->phase)
    {
    case MOTION_SMOOTH_TURN:
        // Controlar giro suave con gyro/sensores.
        // Si termina:
        //   Update_Robot_Heading(TURN_LEFT o TURN_RIGHT);
        //   motion->phase = MOTION_STRAIGHT_ADVANCE;
        break;

    case MOTION_PIVOT_TURN:
        // Controlar giro 180 con gyro.
        // Si termina:
        //   Update_Robot_Heading(TURN_AROUND);
        //   motion->phase = MOTION_STRAIGHT_ADVANCE;
        break;

    case MOTION_STRAIGHT_ADVANCE:
        // Controlar avance segun paredes/gyro.
    	Motion_ControlStraightAdvance(motion);
        break;

    case MOTION_BRAKING:
        // Reusar la logica de Handle_Braking().
        // Si el robot esta detenido:
        Motion_Complete(motion, MOTION_RESULT_NEW_CELL_REACHED);
        break;

    case MOTION_ERROR:
        Motion_Complete(motion, MOTION_RESULT_ERROR);
        break;

    default:
        Motion_Complete(motion, MOTION_RESULT_ERROR);
        break;
    }
}

static void Motion_Complete(MotionContext *motion, MotionResult result)
{
    Set_Motor_Speeds(0, 0);

    motion->result = result;
    motion->result_pending = true;
    motion->action_active = false;
    motion->phase = MOTION_DONE;
}

static bool Motion_DetectedNewCellTape(MotionContext *motion)
{
    bool rear_tape = ((sensor_snapshot.detection_flags & SENSOR_DET_FLOOR_REAR) != 0U);

    if (motion->waiting_leave_current_line)
    {
        if (!rear_tape)
        {
            motion->waiting_leave_current_line = false;
            motion->line_detector_armed = true;
        }

        return false;
    }

    return motion->line_detector_armed && rear_tape;
}

static void Motion_ControlStraightAdvance(MotionContext *motion)
{
    if (Motion_DetectedNewCellTape(motion))
    {
    	Motion_Complete(motion, MOTION_RESULT_NEW_CELL_REACHED);
        return;
    }

    if ((uint16_t)((sensor_snapshot.dist_front_left_mm + sensor_snapshot.dist_front_right_mm) / 2))
    {
        Motion_SetPID(motion, PID_ROLE_BRAKING, wall_braking_target_mm);
        motion->phase = MOTION_BRAKING;
        return;
    }

    StraightReference ref = Motion_SelectStraightReference();

    if (motion->straight_ref != ref)
    {
        motion->straight_ref = ref;
        Reset_Yaw_Tracking();
        PID_Reset(motion->active_pid);
    }

    int32_t measured_error = 0;

    switch (ref)
    {
    case STRAIGHT_REF_BOTH_WALLS:
        measured_error = (int32_t)sensor_snapshot.dist_left_lat_mm -
                         (int32_t)sensor_snapshot.dist_right_lat_mm;
        break;

    case STRAIGHT_REF_LEFT_WALL:
        measured_error = (int32_t)sensor_snapshot.dist_left_lat_mm -
                         (int32_t)wall_target_mm;
        break;

    case STRAIGHT_REF_RIGHT_WALL:
        measured_error = (int32_t)wall_target_mm -
                         (int32_t)sensor_snapshot.dist_right_lat_mm;
        break;

    case STRAIGHT_REF_YAW:
    default:
        measured_error = FIXED_TO_INT(current_yaw_fixed);
        break;
    }

    int32_t pid_output_fixed = PID_Update(motion->active_pid, measured_error, control_step_dt_ms);

    int16_t correction = (int16_t)FIXED_TO_INT(pid_output_fixed);

    Set_Motor_Speeds((int16_t)right_motor_base_speed - correction, (int16_t)left_motor_base_speed + correction);
}

static StraightReference Motion_SelectStraightReference(void)
{
    uint8_t flags = sensor_snapshot.detection_flags;

    bool left_wall = (flags & SENSOR_DET_WALL_LEFT) && (flags & SENSOR_DET_WALL_DIAG_LEFT);
    bool right_wall = (flags & SENSOR_DET_WALL_RIGHT) && (flags & SENSOR_DET_WALL_DIAG_RIGHT);

    if (left_wall && right_wall)
        return STRAIGHT_REF_BOTH_WALLS;

    if (left_wall)
        return STRAIGHT_REF_LEFT_WALL;

    if (right_wall)
        return STRAIGHT_REF_RIGHT_WALL;

    return STRAIGHT_REF_YAW;
}
