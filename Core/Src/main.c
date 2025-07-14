/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>

#include "usbd_cdc_if.h"
#include "ESP01.h"
#include "UNERBUS.h"
#include "MPU6050.h"
#include "BUTTONS.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef union
{
  struct
  {
    uint8_t b0 : 1;
    uint8_t b1 : 1;
    uint8_t b2 : 1;
    uint8_t b3 : 1;
    uint8_t b4 : 1;
    uint8_t b5 : 1;
    uint8_t b6 : 1;
    uint8_t b7 : 1;
  } bit;
  uint8_t byte;
} SystemFlagTypeDef;
SystemFlagTypeDef flags0;

typedef union
{
  uint8_t u8[4];
  int8_t i8[4];
  uint16_t u16[2];
  int16_t i16[2];
  uint32_t u32;
  int32_t i32;
} DataUnionTypeDef;
DataUnionTypeDef work_data;

typedef enum
{
  CMD_ACK = 0x0D,
  CMD_GET_ALIVE = 0xF0,
  CMD_START_CONFIG = 0xEE,
  CMD_FIRMWARE = 0xF1,
  CMD_BUTTON_STATE = 0x12,
  CMD_MPU = 0xA2,
  CMD_LAST_ADC = 0xA0,
  CMD_TEST_MOTORES = 0xA1,
  CMD_VELOCIDAD_MOTORES = 0xA4,
  CMD_SET_PWM_MOTORES = 0xA5,
  CMD_GET_PWM_MOTORES = 0xA6,
  CMD_TEST_MOTORES_50 = 0xA7,
  CMD_GET_LOCAL_IP = 0xE0,
  CMD_UART_BYPASS_CONTROL = 0xDD,
  CMD_OTHERS
} CommandIdTypeDef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Flags */
#define ON10MS flags0.bit.b0
#define UART_BYPASS flags0.bit.b1
#define MPU_READ_REQUEST flags0.bit.b2

/* ADC Buffer Sizes */
#define ADC_BUFFER_SIZE 48

/* USB CDC Buffer Sizes */
#define USB_CDC_RX_BUFFER_SIZE 128
#define USB_CDC_TX_BUFFER_SIZE 256

/* WiFi ESP01 Buffer Sizes */
#define WIFI_RX_BUFFER_SIZE 128
#define WIFI_TX_BUFFER_SIZE 128

/* Heartbeat Masks*/
#define HEARTBEAT_IDLE 0xF4000000
#define HEARTBEAT_WIFI_READY 0xF5000000
#define HEARTBEAT_UDP_READY 0xF5400000

/* Wifi Settings */
#define WIFI_SSID "InternetPlus_8e2fbb"
#define WIFI_PASSWORD "Akhantos2340"
#define WIFI_UDP_REMOTE_IP "192.168.1.8" // La IP de la PC
#define WIFI_UDP_REMOTE_PORT 30010       // El puerto UDP en la PC
#define WIFI_UDP_LOCAL_PORT 30000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c2_rx;
DMA_HandleTypeDef hdma_i2c2_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
_sESP01Handle esp01_handle;
_sUNERBUSHandle unerbus_pc_handle;
_sUNERBUSHandle unerbus_esp01_handle;

char local_ip[16];
uint8_t buf_rx_pc[USB_CDC_RX_BUFFER_SIZE], buf_tx_pc[USB_CDC_TX_BUFFER_SIZE];
uint8_t buf_rx_esp01[WIFI_RX_BUFFER_SIZE], buf_tx_esp01[WIFI_TX_BUFFER_SIZE], data_rx_esp01;

uint32_t heartbeat_counter, heartbeat_mask;
uint8_t time_10ms, time_100ms, timeout_alive_udp;

uint8_t rx_usb_data, new_data;

uint16_t buf_adc[ADC_BUFFER_SIZE][8];
uint8_t adc_buf_write_idx, adc_buf_read_idx;

/* MPU6050 */
static MPU6050_HandleTypeDef hmpu;

/* Motors */
uint16_t motor_pwm_values[4] = {0, 0, 0, 0}; // Valores PWM para los 4 canales

/* Buttons */
Button_HandleTypeDef h_user_button;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void ESP01_SetChipEnable(uint8_t value);
int ESP01_WriteUartByte(uint8_t value);
void ESP01_WriteByteToRxBuffer(uint8_t value);
void ESP01_ChangeState(_eESP01STATUS esp01State);

void DecodeCMD(struct UNERBUSHandle *aBus, uint8_t iStartData);

void Do10ms();

void USB_ReceiveData(uint8_t *buf, uint16_t len);

static void ManageTransmission(void);

/* I2C */
static int8_t I2C_WriteBlocking(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, uint16_t data_len, void *context);
static int8_t I2C_WriteDMA(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, uint16_t data_len, void *context);
static int8_t I2C_ReadBlocking(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, uint16_t data_len, void *context);
static int8_t I2C_ReadDMA(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, uint16_t data_len, void *context);

/* MPU */
static void ManageMpuReading(void);

/* Botones */
uint8_t Read_User_Button(void *context);
static void ManageButtonEvents(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* CALLBACKS */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    time_10ms--;
    if (!time_10ms)
    {
      ON10MS = 1;
      time_10ms = 40;
    }
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&buf_adc[adc_buf_write_idx], 8);
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  adc_buf_write_idx++;
  adc_buf_write_idx %= ADC_BUFFER_SIZE;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
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

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  // Escritura DMA completada
  if (hi2c == &hi2c2)
  {
    // Manejar finalización para I2C2
    // Por ejemplo: activar flag, procesar siguiente operación, etc.
  }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == &hi2c2)
  {
    // Convertir datos del buffer DMA a la estructura
    hmpu.raw_data.accel_x_raw = (int16_t)((hmpu.dma_buffer[0] << 8) | hmpu.dma_buffer[1]);
    hmpu.raw_data.accel_y_raw = (int16_t)((hmpu.dma_buffer[2] << 8) | hmpu.dma_buffer[3]);
    hmpu.raw_data.accel_z_raw = (int16_t)((hmpu.dma_buffer[4] << 8) | hmpu.dma_buffer[5]);
    hmpu.raw_data.temp_raw = (int16_t)((hmpu.dma_buffer[6] << 8) | hmpu.dma_buffer[7]);
    hmpu.raw_data.gyro_x_raw = (int16_t)((hmpu.dma_buffer[8] << 8) | hmpu.dma_buffer[9]);
    hmpu.raw_data.gyro_y_raw = (int16_t)((hmpu.dma_buffer[10] << 8) | hmpu.dma_buffer[11]);
    hmpu.raw_data.gyro_z_raw = (int16_t)((hmpu.dma_buffer[12] << 8) | hmpu.dma_buffer[13]);

    hmpu.dma_busy = 0; // Liberar DMA
  }
}

/* Fin CALLBACKS */

void ESP01_SetChipEnable(uint8_t value)
{
  HAL_GPIO_WritePin(CH_EN_GPIO_Port, CH_EN_Pin, value);
}

int ESP01_WriteUartByte(uint8_t value)
{
  if (__HAL_UART_GET_FLAG(&huart1, USART_SR_TXE))
  {
    USART1->DR = value;
    return 1;
  }
  return 0;
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

  id = UNERBUS_GetUInt8(aBus);
  switch ((CommandIdTypeDef)id)
  {
  case CMD_GET_LOCAL_IP: // GET LOCAL IP
    UNERBUS_Write(aBus, (uint8_t *)ESP01_GetLocalIP(), 16);
    length = 17;
    break;
  case CMD_GET_ALIVE: // ALIVE
    UNERBUS_WriteByte(aBus, CMD_ACK);
    length = 2; //
    break;
  case CMD_LAST_ADC:        // LAST_ADC - Enviar datos del ADC - 55 4E 45 52 02 3A A0 94
    uint8_t adc_buffer[16]; // Buffer temporal para los datos del ADC

    // Leer del último buffer de ADC completado y seguro
    uint8_t last_adc_idx = (adc_buf_write_idx == 0) ? (ADC_BUFFER_SIZE - 1) : (adc_buf_write_idx - 1);

    // Convertir los 8 canales uint16_t a bytes (Little Endian)
    for (uint8_t i = 0; i < 8; i++)
    {
      adc_buffer[idx++] = (uint8_t)(buf_adc[last_adc_idx][i] & 0xFF);        // Byte bajo
      adc_buffer[idx++] = (uint8_t)((buf_adc[last_adc_idx][i] >> 8) & 0xFF); // Byte alto
    }

    UNERBUS_Write(aBus, adc_buffer, 16);
    length = 17; // 1 (CMD) + 16 (datos)
    break;
  case CMD_UART_BYPASS_CONTROL:           // UART_BYPASS_CONTROL - Activar/desactivar bypass
    UART_BYPASS = UNERBUS_GetUInt8(aBus); // 0 o 1
    UNERBUS_WriteByte(aBus, UART_BYPASS); // Confirmar estado
    length = 2;
    break;
  case CMD_MPU:             // Enviar datos del MPU6050 55 4E 45 52 02 3A A2 96
    uint8_t mpu_buffer[14]; // Buffer para datos del MPU

    // Convertir datos raw a bytes (Little Endian)
    mpu_buffer[idx++] = (uint8_t)(hmpu.raw_data.accel_x_raw & 0xFF);
    mpu_buffer[idx++] = (uint8_t)((hmpu.raw_data.accel_x_raw >> 8) & 0xFF);
    mpu_buffer[idx++] = (uint8_t)(hmpu.raw_data.accel_y_raw & 0xFF);
    mpu_buffer[idx++] = (uint8_t)((hmpu.raw_data.accel_y_raw >> 8) & 0xFF);
    mpu_buffer[idx++] = (uint8_t)(hmpu.raw_data.accel_z_raw & 0xFF);
    mpu_buffer[idx++] = (uint8_t)((hmpu.raw_data.accel_z_raw >> 8) & 0xFF);
    mpu_buffer[idx++] = (uint8_t)(hmpu.raw_data.temp_raw & 0xFF);
    mpu_buffer[idx++] = (uint8_t)((hmpu.raw_data.temp_raw >> 8) & 0xFF);
    mpu_buffer[idx++] = (uint8_t)(hmpu.raw_data.gyro_x_raw & 0xFF);
    mpu_buffer[idx++] = (uint8_t)((hmpu.raw_data.gyro_x_raw >> 8) & 0xFF);
    mpu_buffer[idx++] = (uint8_t)(hmpu.raw_data.gyro_y_raw & 0xFF);
    mpu_buffer[idx++] = (uint8_t)((hmpu.raw_data.gyro_y_raw >> 8) & 0xFF);
    mpu_buffer[idx++] = (uint8_t)(hmpu.raw_data.gyro_z_raw & 0xFF);
    mpu_buffer[idx++] = (uint8_t)((hmpu.raw_data.gyro_z_raw >> 8) & 0xFF);

    UNERBUS_Write(aBus, mpu_buffer, 14);
    length = 15; // 1 (CMD) + 14 (datos)
    break;
  case CMD_SET_PWM_MOTORES: // Control de PWM de motores
    // Recibir 4 valores uint16_t (8 bytes) para los 4 canales PWM
    uint8_t pwm_response[9]; // Buffer para respuesta (1 byte status + 8 bytes valores actuales)

    // Extraer y validar valores PWM
    for (uint8_t i = 0; i < 4; i++)
    {
      uint16_t pwm_val = UNERBUS_GetUInt16(aBus);
      if (pwm_val > 9999)
        pwm_val = 9999; // Limitar a máximo
      motor_pwm_values[i] = pwm_val;
    }

    // Aplicar los valores PWM a los canales del TIM4
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, motor_pwm_values[0]);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, motor_pwm_values[1]);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, motor_pwm_values[2]);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, motor_pwm_values[3]);

    // Preparar respuesta con estado de éxito y valores actuales
    pwm_response[0] = CMD_ACK; // Status: OK
    for (uint8_t i = 0; i < 4; i++)
    {
      pwm_response[1 + i * 2] = (uint8_t)(motor_pwm_values[i] & 0xFF);        // Byte bajo
      pwm_response[2 + i * 2] = (uint8_t)((motor_pwm_values[i] >> 8) & 0xFF); // Byte alto
    }

    UNERBUS_Write(aBus, pwm_response, 9);
    length = 10; // 1 (CMD) + 9 (status + datos)
    break;
  case CMD_GET_PWM_MOTORES:        // Obtener valores PWM actuales
    uint8_t pwm_current_buffer[8]; // Buffer para valores actuales

    // Convertir valores actuales a bytes (Little Endian)
    for (uint8_t i = 0; i < 4; i++)
    {
      pwm_current_buffer[i * 2] = (uint8_t)(motor_pwm_values[i] & 0xFF);            // Byte bajo
      pwm_current_buffer[i * 2 + 1] = (uint8_t)((motor_pwm_values[i] >> 8) & 0xFF); // Byte alto
    }

    UNERBUS_Write(aBus, pwm_current_buffer, 8);
    length = 9; // 1 (CMD) + 8 (datos)
    break;
  case CMD_TEST_MOTORES_50: // Probar motores al 50% de velocidad
    // Establecer movimiento hacia adelante al 50% (Motor 2 y Motor 4)
    motor_pwm_values[0] = 0;    // Motor Der Atrás = 0
    motor_pwm_values[1] = 5000; // Motor Der Adelante = 50%
    motor_pwm_values[2] = 0;    // Motor Izq Atrás = 0
    motor_pwm_values[3] = 5000; // Motor Izq Adelante = 50%

    // Aplicar los valores PWM a los canales del TIM4
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, motor_pwm_values[0]);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, motor_pwm_values[1]);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, motor_pwm_values[2]);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, motor_pwm_values[3]);

    // Preparar respuesta con estado de éxito
    uint8_t test_response[1];
    test_response[0] = CMD_ACK; // Status: OK

    UNERBUS_Write(aBus, test_response, 1);
    length = 2; // 1 (CMD) + 1 (status)
    break;
  }

  if (length)
  {
    UNERBUS_Send(aBus, id, length);
  }
}

void Do10ms()
{
  ON10MS = 0;

  Button_Tick(&h_user_button);

  if (time_100ms)
    time_100ms--;

  MPU_READ_REQUEST = 1;

  ESP01_Timeout10ms();
  UNERBUS_Timeout(&unerbus_esp01_handle);
  UNERBUS_Timeout(&unerbus_pc_handle);
}

void Do100ms()
{
  time_100ms = 10;

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

void USB_ReceiveData(uint8_t *buf, uint16_t len)
{
  UNERBUS_ReceiveBuf(&unerbus_pc_handle, buf, len);
}

uint8_t UART_TransmitByte(uint8_t value)
{
  if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE))
  {
    huart1.Instance->DR = value;
    return 1;
  }
  return 0;
}

/**
 * @brief  Gestiona la transmisión de datos para los diferentes canales de comunicación (USB, ESP01, UART-Bypass).
 * @retval None
 */
static void ManageTransmission(void)
{
  // TRANSMISIÓN ESP01 (solo si NO está en bypass)
  if (!UART_BYPASS && (unerbus_esp01_handle.tx.iRead != unerbus_esp01_handle.tx.iWrite))
  {
    work_data.u8[0] = (unerbus_esp01_handle.tx.iWrite - unerbus_esp01_handle.tx.iRead) & unerbus_esp01_handle.tx.maxIndexRingBuf;
    if (ESP01_Send(unerbus_esp01_handle.tx.buf, unerbus_esp01_handle.tx.iRead, work_data.u8[0], unerbus_esp01_handle.tx.maxIndexRingBuf + 1) == ESP01_SEND_READY)
      unerbus_esp01_handle.tx.iRead = unerbus_esp01_handle.tx.iWrite;
  }

  // TRANSMISIÓN UART DIRECTO (si está en bypass)
  if (UART_BYPASS && (unerbus_pc_handle.tx.iRead != unerbus_pc_handle.tx.iWrite))
  {
    if (unerbus_pc_handle.tx.iRead < unerbus_pc_handle.tx.iWrite)
      work_data.u8[0] = unerbus_pc_handle.tx.iWrite - unerbus_pc_handle.tx.iRead;
    else
      work_data.u8[0] = unerbus_pc_handle.tx.maxIndexRingBuf + 1 - unerbus_pc_handle.tx.iRead;

    // Enviar byte por byte por UART directo
    for (uint8_t i = 0; i < work_data.u8[0]; i++)
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
      work_data.u8[0] = unerbus_pc_handle.tx.iWrite - unerbus_pc_handle.tx.iRead;
    else
      work_data.u8[0] = unerbus_pc_handle.tx.maxIndexRingBuf + 1 - unerbus_pc_handle.tx.iRead;

    if (CDC_Transmit_FS(&unerbus_pc_handle.tx.buf[unerbus_pc_handle.tx.iRead], work_data.u8[0]) == USBD_OK)
    {
      unerbus_pc_handle.tx.iRead = (unerbus_pc_handle.tx.iRead + work_data.u8[0]) & unerbus_pc_handle.tx.maxIndexRingBuf;
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
  hmpu.is_initialized = 0;
  hmpu.is_connected = 0;
  hmpu.dma_busy = 0;

  verificacion = MPU6050_Init(&hmpu);
  if (verificacion != 1)
  {
    verificacion = verificacion * (-1);
    for (uint8_t i = 0; i < verificacion; i++)
    {
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
      HAL_Delay(800);
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
      HAL_Delay(800);
    }
    Error_Handler();
  }

  return 1;
}

int8_t I2C_WriteBlocking(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, uint16_t data_len, void *context)
{
  I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)context;
  HAL_StatusTypeDef status = HAL_I2C_Mem_Write(hi2c, device_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data, data_len, 1000);
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
  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, device_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data, data_len, 1000);
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

static void ManageMpuReading(void)
{
  if (MPU_READ_REQUEST)
  {
    if (!hmpu.dma_busy && HAL_I2C_GetState(&hi2c2) == HAL_I2C_STATE_READY)
    {
      MPU_READ_REQUEST = 0; // Limpiar la solicitud
      if (MPU6050_ReadRawDataDMA(&hmpu) != 1)
      {
        // Error al iniciar la lectura del MPU. Indicar error y detener.
        // Nota: Esta sección es bloqueante y termina en Error_Handler().
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
        HAL_Delay(200);
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        HAL_Delay(200);
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
        HAL_Delay(200);
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        HAL_Delay(200);
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
        HAL_Delay(200);
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        HAL_Delay(200);
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
        HAL_Delay(200);
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        HAL_Delay(200);
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
        Error_Handler();
      }
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
    UNERBUS_Send(&unerbus_pc_handle, CMD_BUTTON_STATE, 2);

    switch (button_event)
    {
    case EVENT_PRESSED:
      /* code */
      break;
    case EVENT_PRESS_RELEASED:
      /* code */
      break;
    case EVENT_LONG_PRESS:
      /* code */
      break;
    case EVENT_LONG_PRESS_RELEASED:
      /* code */
      break;
    default:
      break;
    }
  }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* Heartbeat */
  heartbeat_counter = HEARTBEAT_IDLE;
  heartbeat_mask = 0x80000000;

  /* Time */
  time_10ms = 40;
  time_100ms = 10;
  timeout_alive_udp = 50;

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

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* Timers */
  HAL_TIM_Base_Start_IT(&htim1);

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
  CDC_AttachRxData(USB_ReceiveData);

  /* ESP01 */
  ESP01_Init(&esp01_handle);
  ESP01_AttachChangeState(ESP01_ChangeState);
  ESP01_SetWIFI(WIFI_SSID, WIFI_PASSWORD);
  ESP01_StartUDP(WIFI_UDP_REMOTE_IP, WIFI_UDP_REMOTE_PORT, WIFI_UDP_LOCAL_PORT);

  /* UNERBUS */
  UNERBUS_Init(&unerbus_esp01_handle);
  UNERBUS_Init(&unerbus_pc_handle);

  /* Buttons*/
  Button_Init(&h_user_button, Read_User_Button, NULL);

  /* I2C devices */
  HAL_Delay(1000);

  I2C_DevicesInit();

  HAL_Delay(1000);

  /* UART */
  HAL_UART_Receive_IT(&huart1, &data_rx_esp01, 1);

  /* Flags */
  ON10MS = 0;
  UART_BYPASS = 1;
  MPU_READ_REQUEST = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    ManageButtonEvents();

    if (!timeout_alive_udp && !UART_BYPASS)
    {
      timeout_alive_udp = 50;
      UNERBUS_WriteByte(&unerbus_esp01_handle, CMD_ACK);
      UNERBUS_Send(&unerbus_esp01_handle, CMD_GET_ALIVE, 2);

      /* 		  UNERBUS_WriteConstString(&unerbusPC, "UNER\x03:\xF0\x0D\xC8", 0);
            UNERBUS_WriteConstString(&unerbusPC, " El ALIVE", 1); */
    }

    ManageMpuReading();

    if (!time_100ms)
      Do100ms();

    if (ON10MS)
      Do10ms();

    ManageTransmission();

    ESP01_Task();

    UNERBUS_Task(&unerbus_esp01_handle);

    UNERBUS_Task(&unerbus_pc_handle);
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC | RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 8;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 249;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 9999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CH_EN_GPIO_Port, CH_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SW0_Pin */
  GPIO_InitStruct.Pin = SW0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SW0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CH_EN_Pin */
  GPIO_InitStruct.Pin = CH_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CH_EN_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
