/* SSD1306.h - OLED Display Library Header (BeastMode compliant)
 * Hardware-agnostic, robust, non-blocking, no float, explicit types
 * Compatible with STM32CubeIDE, only .h/.c, modular
 */
#ifndef __SSD1306_H__
#define __SSD1306_H__

#include <stdint.h>
#include <stdbool.h>

#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 64
#define SSD1306_BUFFER_SIZE (SSD1306_WIDTH * SSD1306_HEIGHT / 8)

// Error codes
#define SSD1306_OK 1
#define SSD1306_ERROR -1
#define SSD1306_BUSY -2

// Function pointer types for hardware abstraction
typedef int8_t (*SSD1306_I2C_WriteFunc)(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, uint16_t data_len, void *context);
typedef int8_t (*SSD1306_I2C_WriteDMAFunc)(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, uint16_t data_len, void *context);
typedef void (*SSD1306_DelayFunc)(uint32_t ms);

// SSD1306 handle struct
typedef struct SSD1306_HandleTypeDef
{
    SSD1306_I2C_WriteFunc i2c_write_blocking;
    SSD1306_I2C_WriteDMAFunc i2c_write_dma; // DMA-based I2C transfer
    SSD1306_DelayFunc delay_ms;
    void *i2c_context;
    uint8_t device_address;
    bool is_initialized;
    volatile bool dma_busy; // DMA transfer in progress
    uint8_t buffer[SSD1306_BUFFER_SIZE];
    uint8_t dma_buffer[SSD1306_BUFFER_SIZE + 1];
    // Add more fields as needed (cursor, font, etc.)
} SSD1306_HandleTypeDef;

typedef enum
{
    SSD1306_TEXT_ALIGN_LEFT,
    SSD1306_TEXT_ALIGN_CENTER,
    SSD1306_TEXT_ALIGN_RIGHT
} SSD1306_TextAlign;

// Nota importante:
// El flag hssd->dma_busy debe ser liberado manualmente en el callback de DMA/I2C tras finalizar la transferencia:
// Ejemplo: void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) { hssd->dma_busy = false; }

// Public API
int8_t SSD1306_Init(SSD1306_HandleTypeDef *hssd);
int8_t SSD1306_DrawPixel(SSD1306_HandleTypeDef *hssd, uint8_t x, uint8_t y, bool color);
int8_t SSD1306_DrawText(SSD1306_HandleTypeDef *hssd, uint8_t x, uint8_t y, const char *text, SSD1306_TextAlign align);
int8_t SSD1306_UpdateScreen_DMA(SSD1306_HandleTypeDef *hssd); // Non-blocking DMA
int8_t SSD1306_DrawMultilineText(SSD1306_HandleTypeDef *hssd, uint8_t x, uint8_t y, const char *text);

#endif /* __SSD1306_H__ */
