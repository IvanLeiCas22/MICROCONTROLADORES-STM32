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

// Forward declaration for callback type
struct SSD1306_HandleTypeDef;
typedef void (*SSD1306_UpdateDoneCallback)(struct SSD1306_HandleTypeDef *hssd);

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
    bool update;
    SSD1306_UpdateDoneCallback update_done_cb; // Callback for DMA completion
    uint8_t buffer[SSD1306_BUFFER_SIZE];
    // Add more fields as needed (cursor, font, etc.)
} SSD1306_HandleTypeDef;

// Public API
int8_t SSD1306_Init(SSD1306_HandleTypeDef *hssd);
int8_t SSD1306_DrawPixel(SSD1306_HandleTypeDef *hssd, uint8_t x, uint8_t y, bool color);
int8_t SSD1306_DrawText(SSD1306_HandleTypeDef *hssd, uint8_t x, uint8_t y, const char *text);
int8_t SSD1306_UpdateScreen_DMA(SSD1306_HandleTypeDef *hssd, SSD1306_UpdateDoneCallback cb); // Non-blocking DMA

#endif /* __SSD1306_H__ */
