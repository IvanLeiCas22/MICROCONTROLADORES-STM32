/**
 * @file SSD1306.h
 * @brief OLED SSD1306 Display Library Header (Robot Seguidor de Laberinto)
 * @author GitHub Copilot
 */

#ifndef __SSD1306_H__
#define __SSD1306_H__

#include <stdint.h>
#include <stdbool.h>

/* Display dimensions */
#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 64
#define SSD1306_BUFFER_SIZE (SSD1306_WIDTH * SSD1306_HEIGHT / 8)

/* Error codes */
#define SSD1306_OK 1
#define SSD1306_ERROR -1
#define SSD1306_BUSY -2

/* Hardware abstraction function pointer types */
typedef int8_t (*SSD1306_I2C_WriteFunc)(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, uint16_t data_len, void *context);
typedef int8_t (*SSD1306_I2C_WriteDMAFunc)(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, uint16_t data_len, void *context);
typedef void (*SSD1306_DelayFunc)(uint32_t ms);

/* Text alignment options */
typedef enum
{
    SSD1306_TEXT_ALIGN_LEFT,
    SSD1306_TEXT_ALIGN_CENTER,
    SSD1306_TEXT_ALIGN_RIGHT
} SSD1306_TextAlign;

/* SSD1306 handle structure */
typedef struct SSD1306_HandleTypeDef
{
    SSD1306_I2C_WriteFunc i2c_write_blocking;
    SSD1306_I2C_WriteDMAFunc i2c_write_dma;
    SSD1306_DelayFunc delay_ms;
    void *i2c_context;
    uint8_t device_address;
    bool is_initialized;
    uint8_t buffer[SSD1306_BUFFER_SIZE];
    uint8_t cursor_x;
    uint8_t cursor_y;
    const uint8_t *font;
    uint8_t font_width;
    uint8_t font_height;
} SSD1306_HandleTypeDef;

/* Public API */
int8_t SSD1306_Init(SSD1306_HandleTypeDef *hssd);
int8_t SSD1306_DrawPixel(SSD1306_HandleTypeDef *hssd, uint8_t x, uint8_t y, bool color);
int8_t SSD1306_DrawLine(SSD1306_HandleTypeDef *hssd, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, bool color);
int8_t SSD1306_DrawRect(SSD1306_HandleTypeDef *hssd, uint8_t x, uint8_t y, uint8_t w, uint8_t h, bool color);
int8_t SSD1306_DrawBitmap(SSD1306_HandleTypeDef *hssd, uint8_t x, uint8_t y, const uint8_t *bitmap, uint8_t w, uint8_t h);
int8_t SSD1306_DrawText(SSD1306_HandleTypeDef *hssd, uint8_t x, uint8_t y, const char *text, SSD1306_TextAlign align);
int8_t SSD1306_DrawMultilineText(SSD1306_HandleTypeDef *hssd, uint8_t x, uint8_t y, const char *text);
int8_t SSD1306_UpdateScreen_DMA(SSD1306_HandleTypeDef *hssd);
int8_t SSD1306_Clear(SSD1306_HandleTypeDef *hssd);
int8_t SSD1306_SetContrast(SSD1306_HandleTypeDef *hssd, uint8_t contrast);
int8_t SSD1306_DisplayOn(SSD1306_HandleTypeDef *hssd);
int8_t SSD1306_DisplayOff(SSD1306_HandleTypeDef *hssd);
void SSD1306_SetFont(SSD1306_HandleTypeDef *hssd, const uint8_t *font, uint8_t width, uint8_t height);

#endif /* __SSD1306_H__ */