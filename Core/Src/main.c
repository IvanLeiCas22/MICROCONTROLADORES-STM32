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
#include <stdio.h> // Para sprintf
#include "usbd_cdc_if.h"
#include "ESP01.h"
#include "UNERBUS.h"
#include "MPU6050.h"
#include "ssd1306.h"
#include "ssd1306_font_small.h"
#include "ssd1306_font_title.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef union{
	struct{
		uint8_t	ON10MS:	1;
		uint8_t	b1:	1;
		uint8_t	b2:	1;
		uint8_t	b3:	1;
		uint8_t	b4:	1;
		uint8_t	b5:	1;
		uint8_t	b6:	1;
		uint8_t	b7:	1;
	} bit;
	uint8_t byte;
} _uFlag;

typedef union{
	uint8_t		u8[4];
	int8_t		i8[4];
	uint16_t	u16[2];
	int16_t		i16[2];
	uint32_t	u32;
	int32_t		i32;
} _uWork;

/**
 * @brief Contexto asíncrono genérico para operaciones I2C no bloqueantes.
 *        Permite manejar múltiples dispositivos en el mismo bus.
 */
typedef struct {
  void *i2c_handle; // Handler específico de la plataforma (ej: HAL_I2C_HandleTypeDef*)
  void (*cb)(void *cb_ctx); // Callback a llamar al finalizar la transferencia
  void *cb_ctx; // Contexto del callback (ej: puntero al driver o datos)
} I2C_Async_Context_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SIZEBUFADC				48
#define SIZEBUFRXPC				128
#define SIZEBUFTXPC				256
#define SIZEBUFRXESP01			128
#define SIZEBUFTXESP01			128


#define	HEARTBEAT_IDLE			0xF4000000
#define	HEARTBEAT_WIFI_READY	0xF5000000
#define	HEARTBEAT_UDP_READY		0xF5400000

#define WIFI_SSID				"InternetPlus_8e2fbb"
#define WIFI_PASSWORD			"Akhantos2340"
#define WIFI_UDP_REMOTE_IP		"192.168.1.8"		//La IP de la PC
#define WIFI_UDP_REMOTE_PORT	30010				//El puerto UDP en la PC
#define WIFI_UDP_LOCAL_PORT		30000


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
_uFlag flag1;
_uWork w;

_sESP01Handle esp01;
_sUNERBUSHandle unerbusPC;
_sUNERBUSHandle unerbusESP01;

char localIP[16];
uint8_t bufRXPC[SIZEBUFRXPC], bufTXPC[SIZEBUFTXPC];
uint8_t bufRXESP01[SIZEBUFRXESP01], bufTXESP01[SIZEBUFTXESP01], dataRXESP01;

uint32_t heartbeat, heartbeatmask;
uint8_t time10ms, time100ms, timeOutAliveUDP, timeOutGetMpuData;

uint8_t rxUSBData, newData;

uint16_t bufADC[SIZEBUFADC][8];
uint8_t iwBufADC, irBufADC;

// ===== MPU6050 =====
MPU6050_t mpu;
volatile uint8_t mpu_data_ready = 0;
volatile uint8_t mpu_reconfig_request = 0; // Flag para solicitar reconfiguración segura

static MPU6050_Config_t mpu_config = {
    .accel_range = MPU6050_ACCEL_RANGE_4G,
    .gyro_range = MPU6050_GYRO_RANGE_500_DPS,
    .dlpf_bandwidth = MPU6050_DLPF_BW_20HZ,
    .sample_rate_divider = 9,
    .enable_calibration = 1,      // Habilitar calibración automática
    .calibration_samples = 1000   // 1000 muestras para calibración
};

// ===== SSD1306 =====
SSD1306_t ssd;

// ===== I2C =====
// --- Contextos independientes para cada dispositivo ---
I2C_Async_Context_t mpu_ctx = {0};
I2C_Async_Context_t ssd_ctx = {0};

// --- Uso en el loop principal ---
// Para iniciar una lectura asíncrona de MPU6050:
// MPU6050_ReadAll_Async(&mpu, mpu_user_callback);
// Para SSD1306, usa la función correspondiente de tu driver.

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
void ESP01DoCHPD(uint8_t value);
int ESP01WriteUSARTByte(uint8_t value);
void ESP01WriteByteToBufRX(uint8_t value);
void ESP01ChangeState(_eESP01STATUS esp01State);

void DecodeCMD(struct UNERBUSHandle *aBus, uint8_t iStartData);

void Do10ms();

void USBReceive(uint8_t *buf, uint16_t len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//CALLBACKS

// --- Funciones de bajo nivel I2C genéricas para drivers portables ---
int8_t i2cdev_write(uint8_t dev_addr, uint8_t reg, const uint8_t *data, uint16_t len, void *user_ctx) {
  I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)((I2C_Async_Context_t *)user_ctx)->i2c_handle;
  if (!hi2c || !data) return -1;
  HAL_StatusTypeDef status = HAL_I2C_Mem_Write(hi2c, dev_addr, reg, 1, (uint8_t *)data, len, 1000);
  return (status == HAL_OK) ? 0 : -1;
}

int8_t i2cdev_read(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint16_t len, void *user_ctx) {
  I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)((I2C_Async_Context_t *)user_ctx)->i2c_handle;
  if (!hi2c || !data) return -1;
  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, dev_addr, reg, 1, data, len, 1000);
  return (status == HAL_OK) ? 0 : -1;
}

int8_t i2cdev_read_async(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint16_t len, void *user_ctx, void (*cb)(void *cb_ctx), void *cb_ctx)
{
    I2C_Async_Context_t *ctx = (I2C_Async_Context_t *)user_ctx;
    if (!ctx || !data || !cb) return -1;
    
    ctx->cb = cb;
    ctx->cb_ctx = cb_ctx; // El driver pasa dev aquí automáticamente
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)ctx->i2c_handle;
    if (!hi2c) return -1;
    
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(hi2c, dev_addr, reg, 1, data, len);
    return (status == HAL_OK) ? 0 : -1;
}

int8_t i2cdev_write_async(uint8_t dev_addr, uint8_t reg, const uint8_t *data, uint16_t len, void *user_ctx, void (*cb)(void *cb_ctx), void *cb_ctx)
{
    I2C_Async_Context_t *ctx = (I2C_Async_Context_t *)user_ctx;
    if (!ctx || !data || !cb) return -1;
    
    ctx->cb = cb;
    ctx->cb_ctx = cb_ctx;
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)ctx->i2c_handle;
    if (!hi2c) return -1;
    
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write_DMA(hi2c, dev_addr, reg, 1, (uint8_t *)data, len);
    return (status == HAL_OK) ? 0 : -1;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM1){
		time10ms--;
		if(!time10ms){
			flag1.bit.ON10MS = 1;
			time10ms = 40;
		}

		HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&bufADC[iwBufADC], 8);
	}
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (mpu_ctx.i2c_handle == hi2c && mpu_ctx.cb){
      mpu_ctx.cb(mpu_ctx.cb_ctx);          // procesa datos
  }
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (ssd_ctx.i2c_handle == hi2c && ssd_ctx.cb) {
        ssd_ctx.cb(ssd_ctx.cb_ctx); // cb_ctx es el puntero a ssd
    }
}

// Callback de usuario para la lectura asíncrona del MPU6050
void mpu_user_callback(MPU6050_t *dev) {
  mpu_data_ready = 1;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	iwBufADC++;
	iwBufADC &= (SIZEBUFADC-1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART1){
		ESP01_WriteRX(dataRXESP01);
		HAL_UART_Receive_IT(&huart1, &dataRXESP01, 1);
	}
}

void setup_i2c_devices() {
  extern I2C_HandleTypeDef hi2c2; // Handler generado por CubeMX
  
  mpu_ctx.i2c_handle = &hi2c2;
  ssd_ctx.i2c_handle = &hi2c2;

  // --- MPU6050 ---
  mpu.iface.i2c_write = i2cdev_write;
  mpu.iface.i2c_read = i2cdev_read;
  mpu.iface.i2c_read_async = i2cdev_read_async;
  mpu.iface.user_ctx = &mpu_ctx;
  
  int8_t mpu_result = MPU6050_Init(&mpu, &mpu_config);
  if (mpu_result != 0) {
    // Indicar error MPU6050 con parpadeo rápido del LED
    for (int i = 0; i < 10; i++) {
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      HAL_Delay(100);
    }
    // Error_Handler(); // Comentar para continuar con SSD1306
  }

  // --- SSD1306 --- (usa el mismo bus y funciones de bajo nivel)
  ssd.iface.i2c_write = i2cdev_write;
  ssd.iface.i2c_read = i2cdev_read;
  ssd.iface.i2c_write_async = i2cdev_write_async;
  ssd.iface.user_ctx = &ssd_ctx;
  
  int8_t ssd_result = SSD1306_Init(&ssd);
  if (ssd_result != 0) {
    // Indicar error SSD1306 con parpadeo lento del LED
    for (int i = 0; i < 5; i++) {
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      HAL_Delay(300);
    }
    // Error_Handler(); // Comentar para no bloquear el sistema
  }
  
  // Si ambos fallan, llamar Error_Handler
  if (mpu_result != 0 && ssd_result != 0) {
    Error_Handler();
  }
}

void ESP01DoCHPD(uint8_t value){
	HAL_GPIO_WritePin(CH_EN_GPIO_Port, CH_EN_Pin, value);
}

int ESP01WriteUSARTByte(uint8_t value){
	if(__HAL_UART_GET_FLAG(&huart1, USART_SR_TXE)){
		USART1->DR = value;
		return 1;
	}
	return 0;
}

void ESP01WriteByteToBufRX(uint8_t value){
	UNERBUS_ReceiveByte(&unerbusESP01, value);
}

void ESP01ChangeState(_eESP01STATUS esp01State){
	switch((uint32_t)esp01State){
	case ESP01_WIFI_CONNECTED:
		heartbeat = HEARTBEAT_WIFI_READY;
		break;
	case ESP01_UDPTCP_CONNECTED:
		heartbeat = HEARTBEAT_UDP_READY;
		break;
	case ESP01_UDPTCP_DISCONNECTED:
		heartbeat = HEARTBEAT_WIFI_READY;
		break;
	case ESP01_WIFI_DISCONNECTED:
		heartbeat = HEARTBEAT_IDLE;
		break;
	}
}

void DecodeCMD(struct UNERBUSHandle *aBus, uint8_t iStartData){
	uint8_t id;
	uint8_t length = 0;

	id = UNERBUS_GetUInt8(aBus);
	switch(id){
    case 0xE0://GET LOCAL IP
      UNERBUS_Write(aBus, (uint8_t *)ESP01_GetLocalIP(), 16);
      length = 17;
      break;
    case 0xF0://ALIVE
      UNERBUS_WriteByte(aBus, 0x0D);
      length = 2;
      break;
    case 0xA2: // SOLICITUD DE DATOS MPU6050
    {
      uint8_t buf[12];
      buf[0] = mpu.accel_x & 0xFF;
      buf[1] = (mpu.accel_x >> 8) & 0xFF;
      buf[2] = mpu.accel_y & 0xFF;
      buf[3] = (mpu.accel_y >> 8) & 0xFF;
      buf[4] = mpu.accel_z & 0xFF;
      buf[5] = (mpu.accel_z >> 8) & 0xFF;
      buf[6] = mpu.gyro_x & 0xFF;
      buf[7] = (mpu.gyro_x >> 8) & 0xFF;
      buf[8] = mpu.gyro_y & 0xFF;
      buf[9] = (mpu.gyro_y >> 8) & 0xFF;
      buf[10] = mpu.gyro_z & 0xFF;
      buf[11] = (mpu.gyro_z >> 8) & 0xFF;
      UNERBUS_Write(aBus, buf, 12);
      length = 13; // 1 (CMD) + 12 (payload)
      break;
    }
    case 0xB0: // NUEVO COMANDO: RECONFIGURAR MPU6050
    {
      // Siempre intentar leer 4 bytes de configuración
      uint8_t received_config[4];

      // Leer la configuracion del buffer
      received_config[0] = UNERBUS_GetUInt8(aBus);
      received_config[1] = UNERBUS_GetUInt8(aBus);
      received_config[2] = UNERBUS_GetUInt8(aBus);
      received_config[3] = UNERBUS_GetUInt8(aBus);
      
      // Asignar la configuración leída
      mpu_config.accel_range = received_config[0];
      mpu_config.gyro_range = received_config[1];
      mpu_config.dlpf_bandwidth = received_config[2];
      mpu_config.sample_rate_divider = received_config[3];

      // Solicitar reconfiguración
      mpu_reconfig_request = 1;

      // Enviar ACK
      UNERBUS_WriteByte(aBus, 0x0D);
      length = 2;
      break;
    }
    case 0xB1: // CALIBRAR MPU6050 MANUALMENTE
    {
      // Leer número de muestras (2 bytes)
      uint16_t samples = UNERBUS_GetUInt8(aBus) | (UNERBUS_GetUInt8(aBus) << 8);
      if (samples == 0) samples = 1000; // Valor por defecto
      
      // Realizar calibración manual
      int8_t result = MPU6050_Calibrate(&mpu, samples);
      
      // Enviar resultado
      UNERBUS_WriteByte(aBus, result == 0 ? 0x0D : 0x0E); // 0x0D=OK, 0x0E=ERROR
      length = 2;
      break;
    }
    case 0xB2: // OBTENER DATOS DE CALIBRACIÓN
    {
      uint8_t cal_data[15];
      cal_data[0] = mpu.calibration.accel_offset_x & 0xFF;
      cal_data[1] = (mpu.calibration.accel_offset_x >> 8) & 0xFF;
      cal_data[2] = mpu.calibration.accel_offset_y & 0xFF;
      cal_data[3] = (mpu.calibration.accel_offset_y >> 8) & 0xFF;
      cal_data[4] = mpu.calibration.accel_offset_z & 0xFF;
      cal_data[5] = (mpu.calibration.accel_offset_z >> 8) & 0xFF;
      cal_data[6] = mpu.calibration.gyro_offset_x & 0xFF;
      cal_data[7] = (mpu.calibration.gyro_offset_x >> 8) & 0xFF;
      cal_data[8] = mpu.calibration.gyro_offset_y & 0xFF;
      cal_data[9] = (mpu.calibration.gyro_offset_y >> 8) & 0xFF;
      cal_data[10] = mpu.calibration.gyro_offset_z & 0xFF;
      cal_data[11] = (mpu.calibration.gyro_offset_z >> 8) & 0xFF;
      cal_data[12] = mpu.calibration.is_calibrated;
      cal_data[13] = 0; // Reservado
      cal_data[14] = 0; // Reservado
      
      UNERBUS_Write(aBus, cal_data, 15);
      length = 16; // 1 (CMD) + 15 (payload)
      break;
    }
    case 0xB3: // OBTENER DATOS RAW (SIN CALIBRAR)
    {
      int16_t accel_raw[3], gyro_raw[3];
      MPU6050_GetRawData(&mpu, accel_raw, gyro_raw);
      
      uint8_t raw_data[12];
      raw_data[0] = accel_raw[0] & 0xFF;
      raw_data[1] = (accel_raw[0] >> 8) & 0xFF;
      raw_data[2] = accel_raw[1] & 0xFF;
      raw_data[3] = (accel_raw[1] >> 8) & 0xFF;
      raw_data[4] = accel_raw[2] & 0xFF;
      raw_data[5] = (accel_raw[2] >> 8) & 0xFF;
      raw_data[6] = gyro_raw[0] & 0xFF;
      raw_data[7] = (gyro_raw[0] >> 8) & 0xFF;
      raw_data[8] = gyro_raw[1] & 0xFF;
      raw_data[9] = (gyro_raw[1] >> 8) & 0xFF;
      raw_data[10] = gyro_raw[2] & 0xFF;
      raw_data[11] = (gyro_raw[2] >> 8) & 0xFF;
      
      UNERBUS_Write(aBus, raw_data, 12);
      length = 13; // 1 (CMD) + 12 (payload)
      break;
    }
    case 0xB4: // HABILITAR/DESHABILITAR CALIBRACIÓN
    {
      uint8_t enable = UNERBUS_GetUInt8(aBus);
      MPU6050_SetCalibrationEnabled(&mpu, enable);
      
      // Enviar ACK
      UNERBUS_WriteByte(aBus, 0x0D);
      length = 2;
      break;
    }
	}

	if(length){
		UNERBUS_Send(aBus, id, length);
	}
}

void Do10ms(){
	flag1.bit.ON10MS = 0;

	if(time100ms)
		time100ms--;

  if(timeOutGetMpuData)
    timeOutGetMpuData--;

	ESP01_Timeout10ms();
	UNERBUS_Timeout(&unerbusESP01);
	UNERBUS_Timeout(&unerbusPC);
}

void Do100ms(){
  static uint8_t time_mpu_data = 1;
  time100ms = 10;

  if(heartbeatmask & heartbeat)
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  else
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  heartbeatmask >>= 1;//
  if(!heartbeatmask)
    heartbeatmask = 0x80000000;

  if(timeOutAliveUDP)
    timeOutAliveUDP--;

  // Enviar datos del MPU6050 cada 300ms si hay datos nuevos
  time_mpu_data--;
  if (mpu_data_ready && !time_mpu_data) {
    time_mpu_data = 1;
    mpu_data_ready = 0; // Limpia el flag

    char buf[64];
/*     snprintf(buf, sizeof(buf),
         "A:%d,%d,%d G:%d,%d,%d\r\n",
         mpu.accel_x, mpu.accel_y, mpu.accel_z,
         mpu.gyro_x, mpu.gyro_y, mpu.gyro_z);

    // Por USB (PC)
    UNERBUS_Write(&unerbusPC, (uint8_t*)buf, strlen(buf));
    UNERBUS_Send(&unerbusPC, 0xA2, strlen(buf) + 1); */

    // O por WiFi (ESP01)
    // UNERBUS_Write(&unerbusESP01, buf, 12);
    // UNERBUS_Send(&unerbusESP01, 0xA2, 13);

    // --- Mostrar en display SSD1306 ---
    SSD1306_Clear(&ssd);
    
    // Mostrar estado de calibración
    snprintf(buf, sizeof(buf), "MPU6050 %s", mpu.calibration.is_calibrated ? "CAL" : "RAW");
    SSD1306_DrawString(&ssd, 0, 0, (const uint8_t *)buf, &SSD1306_Font_Small, 1);
    
    // Mostrar datos calibrados
    snprintf(buf, sizeof(buf), "AX:%d AY:%d", mpu.accel_x, mpu.accel_y);
    SSD1306_DrawString(&ssd, 0, 10, (const uint8_t *)buf, &SSD1306_Font_Small, 1);
    snprintf(buf, sizeof(buf), "AZ:%d", mpu.accel_z);
    SSD1306_DrawString(&ssd, 0, 20, (const uint8_t *)buf, &SSD1306_Font_Small, 1);
    snprintf(buf, sizeof(buf), "GX:%d", mpu.gyro_x);
    SSD1306_DrawString(&ssd, 0, 30, (const uint8_t *)buf, &SSD1306_Font_Small, 1);
    snprintf(buf, sizeof(buf), "GY:%d", mpu.gyro_y);
    SSD1306_DrawString(&ssd, 0, 40, (const uint8_t *)buf, &SSD1306_Font_Small, 1);
    snprintf(buf, sizeof(buf), "GZ:%d", mpu.gyro_z);
    SSD1306_DrawString(&ssd, 0, 50, (const uint8_t *)buf, &SSD1306_Font_Small, 1);
  }
}


void USBReceive(uint8_t *buf, uint16_t len){
	UNERBUS_ReceiveBuf(&unerbusPC, buf, len);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	heartbeat = HEARTBEAT_IDLE;
	heartbeatmask = 0x80000000;

	time10ms = 40;
	time100ms = 10;
	timeOutAliveUDP = 50;
  timeOutGetMpuData = 2;

	iwBufADC = 0;
	irBufADC = 0;

	esp01.DoCHPD = ESP01DoCHPD;
	esp01.WriteByteToBufRX = ESP01WriteByteToBufRX;
	esp01.WriteUSARTByte = ESP01WriteUSARTByte;

	unerbusESP01.MyDataReady = DecodeCMD;
	unerbusESP01.WriteUSARTByte = NULL;
	unerbusESP01.rx.buf = bufRXESP01;
	unerbusESP01.rx.maxIndexRingBuf = (SIZEBUFRXESP01 - 1);
	unerbusESP01.tx.buf = bufTXESP01;
	unerbusESP01.tx.maxIndexRingBuf = (SIZEBUFTXESP01 - 1);

	unerbusPC.MyDataReady = DecodeCMD;
	unerbusPC.WriteUSARTByte = NULL;
	unerbusPC.rx.buf = bufRXPC;
	unerbusPC.rx.maxIndexRingBuf = (SIZEBUFRXPC - 1);
	unerbusPC.tx.buf = bufTXPC;
	unerbusPC.tx.maxIndexRingBuf = (SIZEBUFTXPC - 1);

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

  CDC_AttachRxData(USBReceive);

  HAL_TIM_Base_Start_IT(&htim1);

  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
  HAL_TIM_Base_Start(&htim4);

  ESP01_Init(&esp01);
  UNERBUS_Init(&unerbusESP01);
  UNERBUS_Init(&unerbusPC);

  ESP01_AttachChangeState(ESP01ChangeState);
  ESP01_SetWIFI(WIFI_SSID, WIFI_PASSWORD);
  ESP01_StartUDP(WIFI_UDP_REMOTE_IP, WIFI_UDP_REMOTE_PORT, WIFI_UDP_LOCAL_PORT);

  HAL_UART_Receive_IT(&huart1, &dataRXESP01, 1);

  HAL_Delay(3000);

  // Diagnostics de I2C antes de inicializar dispositivos
  if (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) {
    // Indicar problema I2C con parpadeo muy rápido
    for (int i = 0; i < 20; i++) {
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      HAL_Delay(50);
    }
  }

  setup_i2c_devices();

  // Debug: Mostrar estado básico en SSD1306 si está disponible
  if (ssd.ready) {
    SSD1306_Clear(&ssd);
    SSD1306_DrawString(&ssd, 0, 0, (const uint8_t *)"INIT COMPLETE", &SSD1306_Font_Small, 1);
    
    if (mpu.ready && mpu.calibration.is_calibrated) {
      SSD1306_DrawString(&ssd, 0, 10, (const uint8_t *)"MPU: OK + CAL", &SSD1306_Font_Small, 1);
    } else if (mpu.ready) {
      SSD1306_DrawString(&ssd, 0, 10, (const uint8_t *)"MPU: OK", &SSD1306_Font_Small, 1);
    } else {
      SSD1306_DrawString(&ssd, 0, 10, (const uint8_t *)"MPU: FAIL", &SSD1306_Font_Small, 1);
    }
    
    SSD1306_UpdateScreen(&ssd);
  }

  HAL_Delay(3000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if(!time100ms)
		  Do100ms();

	  if(flag1.bit.ON10MS)
		  Do10ms();

    /* –– RECONFIGURACIÓN SEGURA –– */
    if (mpu_reconfig_request && HAL_I2C_GetState(&hi2c2) == HAL_I2C_STATE_READY) {
      mpu_reconfig_request = 0;
      if (MPU6050_Init(&mpu, &mpu_config) == 0) {
          for (int i = 0; i < 6; ++i) { HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin); HAL_Delay(50); }
      } else {
          for (int i = 0; i < 4; ++i) { HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin); HAL_Delay(250); }
      }
    }

    // --- Actualización eficiente del display SSD1306 solo si hubo cambios ---
    if (!mpu_reconfig_request && ssd.dirty && ssd.ready && HAL_I2C_GetState(&hi2c2) == HAL_I2C_STATE_READY) {
      int8_t update_result = SSD1306_UpdateScreen_Async(&ssd, NULL); // NULL = no callback
    }

    if (!mpu_reconfig_request && mpu.ready && /*!mpu_data_ready &&*/ HAL_I2C_GetState(&hi2c2) == HAL_I2C_STATE_READY && !timeOutGetMpuData) {
      timeOutGetMpuData = 2;
      // mpu.ready = 0; se hace dentro de ReadAll_Async
      MPU6050_ReadAll_Async(&mpu, mpu_user_callback);
    }

	  if(unerbusESP01.tx.iRead != unerbusESP01.tx.iWrite){
		  w.u8[0] = unerbusESP01.tx.iWrite - unerbusESP01.tx.iRead;
		  w.u8[0] &= unerbusESP01.tx.maxIndexRingBuf;
		  if(ESP01_Send(unerbusESP01.tx.buf, unerbusESP01.tx.iRead, w.u8[0], unerbusESP01.tx.maxIndexRingBuf+1) == ESP01_SEND_READY)
			  unerbusESP01.tx.iRead = unerbusESP01.tx.iWrite;
	  }

	  if(unerbusPC.tx.iRead != unerbusPC.tx.iWrite){
		  if(unerbusPC.tx.iRead < unerbusPC.tx.iWrite)
			  w.u8[0] = unerbusPC.tx.iWrite - unerbusPC.tx.iRead;
		  else
			  w.u8[0] = unerbusPC.tx.maxIndexRingBuf+1 - unerbusPC.tx.iRead;

		  if(CDC_Transmit_FS(&unerbusPC.tx.buf[unerbusPC.tx.iRead], w.u8[0]) == USBD_OK){
			  unerbusPC.tx.iRead += w.u8[0];
			  unerbusPC.tx.iRead &= unerbusPC.tx.maxIndexRingBuf;
		  }
	  }

	  ESP01_Task();

	  UNERBUS_Task(&unerbusESP01);

	  UNERBUS_Task(&unerbusPC);

    if(!timeOutAliveUDP){
		  timeOutAliveUDP = 50;
      UNERBUS_WriteByte(&unerbusESP01, 0x0D);
      UNERBUS_Send(&unerbusESP01, 0xF0, 2);
/*       UNERBUS_WriteConstString(&unerbusPC, "UNER\x03:\xF0\x0D\xC8", 0);
      UNERBUS_WriteConstString(&unerbusPC, " El ALIVE", 1); */
    }

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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
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
  GPIO_InitStruct.Pull = GPIO_NOPULL;
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

#ifdef  USE_FULL_ASSERT
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
