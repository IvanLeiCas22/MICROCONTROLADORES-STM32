/*
 * SSD1306.c - OLED Display Library Implementation (BeastMode compliant)
 * -------------------------------------------------------------
 * - Hardware-agnostic, robust, non-blocking, no float, explicit types
 * - Compatible with STM32CubeIDE, solo .h/.c, modular
 * - Uso de DMA para transferencias no bloqueantes
 * - Manejo robusto de errores y flags
 * - Abstracción total de hardware mediante punteros a funciones
 * -------------------------------------------------------------
 */

#include "SSD1306.h"
#include <stddef.h>

// Internal SSD1306 commands
#define SSD1306_CMD_DISPLAY_OFF 0xAE
#define SSD1306_CMD_DISPLAY_ON 0xAF
#define SSD1306_CMD_SET_CONTRAST 0x81
#define SSD1306_CMD_SET_NORMAL_DISPLAY 0xA6
#define SSD1306_CMD_SET_INVERT_DISPLAY 0xA7
#define SSD1306_CMD_SET_DISPLAY_CLOCK_DIV 0xD5
#define SSD1306_CMD_SET_MULTIPLEX 0xA8
#define SSD1306_CMD_SET_DISPLAY_OFFSET 0xD3
#define SSD1306_CMD_SET_START_LINE 0x40
#define SSD1306_CMD_SET_CHARGE_PUMP 0x8D
#define SSD1306_CMD_SET_MEMORY_MODE 0x20
#define SSD1306_CMD_SET_SEG_REMAP 0xA1
#define SSD1306_CMD_SET_COM_SCAN_DEC 0xC8
#define SSD1306_CMD_SET_COM_PINS 0xDA
#define SSD1306_CMD_SET_PRECHARGE 0xD9
#define SSD1306_CMD_SET_VCOM_DETECT 0xDB
#define SSD1306_CMD_SET_DISPLAY_ALL_ON_RESUME 0xA4
#define SSD1306_CMD_SET_DISPLAY_ALL_ON 0xA5
#define SSD1306_CMD_SET_PAGE_ADDR 0xB0
#define SSD1306_CMD_SET_COL_LOW 0x00
#define SSD1306_CMD_SET_COL_HIGH 0x10

// 5x7 ASCII font (characters 32-127)
static const uint8_t font_5x7[96][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // ' '
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // '!'
    {0x00, 0x07, 0x00, 0x07, 0x00}, // '"'
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, // '#'
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, // '$'
    {0x23, 0x13, 0x08, 0x64, 0x62}, // '%'
    {0x36, 0x49, 0x55, 0x22, 0x50}, // '&'
    {0x00, 0x05, 0x03, 0x00, 0x00}, // '''
    {0x00, 0x1C, 0x22, 0x41, 0x00}, // '('
    {0x00, 0x41, 0x22, 0x1C, 0x00}, // ')'
    {0x14, 0x08, 0x3E, 0x08, 0x14}, // '*'
    {0x08, 0x08, 0x3E, 0x08, 0x08}, // '+'
    {0x00, 0x50, 0x30, 0x00, 0x00}, // ','
    {0x08, 0x08, 0x08, 0x08, 0x08}, // '-'
    {0x00, 0x60, 0x60, 0x00, 0x00}, // '.'
    {0x20, 0x10, 0x08, 0x04, 0x02}, // '/'
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // '0'
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // '1'
    {0x42, 0x61, 0x51, 0x49, 0x46}, // '2'
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // '3'
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // '4'
    {0x27, 0x45, 0x45, 0x45, 0x39}, // '5'
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // '6'
    {0x01, 0x71, 0x09, 0x05, 0x03}, // '7'
    {0x36, 0x49, 0x49, 0x49, 0x36}, // '8'
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // '9'
    {0x00, 0x36, 0x36, 0x00, 0x00}, // ':'
    {0x00, 0x56, 0x36, 0x00, 0x00}, // ';'
    {0x08, 0x14, 0x22, 0x41, 0x00}, // '<'
    {0x14, 0x14, 0x14, 0x14, 0x14}, // '='
    {0x00, 0x41, 0x22, 0x14, 0x08}, // '>'
    {0x02, 0x01, 0x51, 0x09, 0x06}, // '?'
    {0x32, 0x49, 0x79, 0x41, 0x3E}, // '@'
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // 'A'
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // 'B'
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // 'C'
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // 'D'
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // 'E'
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // 'F'
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, // 'G'
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // 'H'
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // 'I'
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // 'J'
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // 'K'
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // 'L'
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, // 'M'
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // 'N'
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // 'O'
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // 'P'
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // 'Q'
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // 'R'
    {0x46, 0x49, 0x49, 0x49, 0x31}, // 'S'
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // 'T'
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // 'U'
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // 'V'
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, // 'W'
    {0x63, 0x14, 0x08, 0x14, 0x63}, // 'X'
    {0x07, 0x08, 0x70, 0x08, 0x07}, // 'Y'
    {0x61, 0x51, 0x49, 0x45, 0x43}, // 'Z'
    {0x00, 0x7F, 0x41, 0x41, 0x00}, // '['
    {0x02, 0x04, 0x08, 0x10, 0x20}, // '\'
    {0x00, 0x41, 0x41, 0x7F, 0x00}, // ']'
    {0x04, 0x02, 0x01, 0x02, 0x04}, // '^'
    {0x40, 0x40, 0x40, 0x40, 0x40}, // '_'
    {0x00, 0x01, 0x02, 0x04, 0x00}, // '`'
    {0x20, 0x54, 0x54, 0x54, 0x78}, // 'a'
    {0x7F, 0x48, 0x44, 0x44, 0x38}, // 'b'
    {0x38, 0x44, 0x44, 0x44, 0x20}, // 'c'
    {0x38, 0x44, 0x44, 0x48, 0x7F}, // 'd'
    {0x38, 0x54, 0x54, 0x54, 0x18}, // 'e'
    {0x08, 0x7E, 0x09, 0x01, 0x02}, // 'f'
    {0x0C, 0x52, 0x52, 0x52, 0x3E}, // 'g'
    {0x7F, 0x08, 0x04, 0x04, 0x78}, // 'h'
    {0x00, 0x44, 0x7D, 0x40, 0x00}, // 'i'
    {0x20, 0x40, 0x44, 0x3D, 0x00}, // 'j'
    {0x7F, 0x10, 0x28, 0x44, 0x00}, // 'k'
    {0x00, 0x41, 0x7F, 0x40, 0x00}, // 'l'
    {0x7C, 0x04, 0x18, 0x04, 0x78}, // 'm'
    {0x7C, 0x08, 0x04, 0x04, 0x78}, // 'n'
    {0x38, 0x44, 0x44, 0x44, 0x38}, // 'o'
    {0x7C, 0x14, 0x14, 0x14, 0x08}, // 'p'
    {0x08, 0x14, 0x14, 0x18, 0x7C}, // 'q'
    {0x7C, 0x08, 0x04, 0x04, 0x08}, // 'r'
    {0x48, 0x54, 0x54, 0x54, 0x20}, // 's'
    {0x04, 0x3F, 0x44, 0x40, 0x20}, // 't'
    {0x3C, 0x40, 0x40, 0x20, 0x7C}, // 'u'
    {0x1C, 0x20, 0x40, 0x20, 0x1C}, // 'v'
    {0x3C, 0x40, 0x30, 0x40, 0x3C}, // 'w'
    {0x44, 0x28, 0x10, 0x28, 0x44}, // 'x'
    {0x0C, 0x50, 0x50, 0x50, 0x3C}, // 'y'
    {0x44, 0x64, 0x54, 0x4C, 0x44}, // 'z'
    {0x00, 0x08, 0x36, 0x41, 0x00}, // '{'
    {0x00, 0x00, 0x7F, 0x00, 0x00}, // '|'
    {0x00, 0x41, 0x36, 0x08, 0x00}, // '}'
    {0x08, 0x08, 0x2A, 0x1C, 0x08}, // '~'
};

/**
 * @brief  Envía un comando al display SSD1306 (bloqueante)
 * @param  hssd: puntero al handle SSD1306
 * @param  cmd: comando a enviar
 * @retval SSD1306_OK si éxito, SSD1306_ERROR si falla
 */
static int8_t SSD1306_WriteCommand(SSD1306_HandleTypeDef *hssd, uint8_t cmd)
{
    if (!hssd || !hssd->i2c_write_blocking)
        return SSD1306_ERROR;
    return hssd->i2c_write_blocking(hssd->device_address, 0x00, &cmd, 1, hssd->i2c_context);
}

/**
 * @brief  Inicializa el display SSD1306
 * @param  hssd: puntero al handle SSD1306
 * @retval SSD1306_OK si éxito, SSD1306_ERROR si falla
 * @note   Debe llamarse una sola vez tras configurar el hardware I2C y antes de cualquier operación de dibujo.
 *         El handle debe estar correctamente inicializado y los punteros a funciones asignados.
 *         Compatible con STM32CubeIDE: colocar llamada en bloque USER CODE BEGIN
 */

int8_t SSD1306_Init(SSD1306_HandleTypeDef *hssd)
{
    // Validate handle and required function pointers
    if (hssd == NULL)
        return SSD1306_ERROR;
    if (hssd->i2c_write_blocking == NULL || hssd->delay_ms == NULL)
        return SSD1306_ERROR;

    hssd->dma_busy = false;
    hssd->is_initialized = false;
    // Initialization sequence (blocking)
    if (SSD1306_WriteCommand(hssd, SSD1306_CMD_DISPLAY_OFF) != SSD1306_OK)
        return SSD1306_ERROR;
    if (SSD1306_WriteCommand(hssd, SSD1306_CMD_SET_DISPLAY_CLOCK_DIV) != SSD1306_OK)
        return SSD1306_ERROR;
    if (SSD1306_WriteCommand(hssd, 0x80) != SSD1306_OK)
        return SSD1306_ERROR;
    if (SSD1306_WriteCommand(hssd, SSD1306_CMD_SET_MULTIPLEX) != SSD1306_OK)
        return SSD1306_ERROR;
    if (SSD1306_WriteCommand(hssd, SSD1306_HEIGHT - 1) != SSD1306_OK)
        return SSD1306_ERROR;
    if (SSD1306_WriteCommand(hssd, SSD1306_CMD_SET_DISPLAY_OFFSET) != SSD1306_OK)
        return SSD1306_ERROR;
    if (SSD1306_WriteCommand(hssd, 0x00) != SSD1306_OK)
        return SSD1306_ERROR;
    if (SSD1306_WriteCommand(hssd, SSD1306_CMD_SET_START_LINE | 0x00) != SSD1306_OK)
        return SSD1306_ERROR;
    if (SSD1306_WriteCommand(hssd, SSD1306_CMD_SET_CHARGE_PUMP) != SSD1306_OK)
        return SSD1306_ERROR;
    if (SSD1306_WriteCommand(hssd, 0x14) != SSD1306_OK)
        return SSD1306_ERROR;
    if (SSD1306_WriteCommand(hssd, SSD1306_CMD_SET_MEMORY_MODE) != SSD1306_OK)
        return SSD1306_ERROR;
    if (SSD1306_WriteCommand(hssd, 0x00) != SSD1306_OK)
        return SSD1306_ERROR;
    if (SSD1306_WriteCommand(hssd, SSD1306_CMD_SET_SEG_REMAP | 0x01) != SSD1306_OK)
        return SSD1306_ERROR;
    if (SSD1306_WriteCommand(hssd, SSD1306_CMD_SET_COM_SCAN_DEC) != SSD1306_OK)
        return SSD1306_ERROR;
    if (SSD1306_WriteCommand(hssd, SSD1306_CMD_SET_COM_PINS) != SSD1306_OK)
        return SSD1306_ERROR;
    if (SSD1306_WriteCommand(hssd, 0x12) != SSD1306_OK)
        return SSD1306_ERROR;
    if (SSD1306_WriteCommand(hssd, SSD1306_CMD_SET_CONTRAST) != SSD1306_OK)
        return SSD1306_ERROR;
    if (SSD1306_WriteCommand(hssd, 0xCF) != SSD1306_OK)
        return SSD1306_ERROR;
    if (SSD1306_WriteCommand(hssd, SSD1306_CMD_SET_PRECHARGE) != SSD1306_OK)
        return SSD1306_ERROR;
    if (SSD1306_WriteCommand(hssd, 0xF1) != SSD1306_OK)
        return SSD1306_ERROR;
    if (SSD1306_WriteCommand(hssd, SSD1306_CMD_SET_VCOM_DETECT) != SSD1306_OK)
        return SSD1306_ERROR;
    if (SSD1306_WriteCommand(hssd, 0x40) != SSD1306_OK)
        return SSD1306_ERROR;
    if (SSD1306_WriteCommand(hssd, SSD1306_CMD_SET_DISPLAY_ALL_ON_RESUME) != SSD1306_OK)
        return SSD1306_ERROR;
    if (SSD1306_WriteCommand(hssd, SSD1306_CMD_SET_NORMAL_DISPLAY) != SSD1306_OK)
        return SSD1306_ERROR;
    if (SSD1306_WriteCommand(hssd, SSD1306_CMD_DISPLAY_ON) != SSD1306_OK)
        return SSD1306_ERROR;
    // Clear buffer
    for (uint16_t i = 0; i < SSD1306_BUFFER_SIZE; i++)
        hssd->buffer[i] = 0x00;
    hssd->is_initialized = true;
    return SSD1306_OK;
}

/**
 * @brief  Dibuja un pixel en el buffer del display
 * @param  hssd: puntero al handle SSD1306
 * @param  x, y: coordenadas
 * @param  color: true=encendido, false=apagado
 * @retval SSD1306_OK si éxito, SSD1306_ERROR si fuera de rango
 * @note   No actualiza el display; requiere llamada a SSD1306_UpdateScreen_DMA para reflejar cambios.
 *         Si las coordenadas están fuera de rango, no modifica el buffer y retorna error.
 */
int8_t SSD1306_DrawPixel(SSD1306_HandleTypeDef *hssd, uint8_t x, uint8_t y, bool color)
{
    // Validate handle
    if (hssd == NULL)
        return SSD1306_ERROR;
    // Validate coordinates
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT)
        return SSD1306_ERROR;
    uint16_t byte_idx = x + (y / 8) * SSD1306_WIDTH;
    uint8_t bit_mask = 1 << (y % 8);
    if (color)
        hssd->buffer[byte_idx] |= bit_mask;
    else
        hssd->buffer[byte_idx] &= ~bit_mask;
    return SSD1306_OK;
}

/**
 * @brief  Dibuja un texto ASCII en el buffer del display
 * @param  hssd: puntero al handle SSD1306
 * @param  x, y: coordenadas de inicio
 * @param  text: puntero a string ASCII
 * @retval SSD1306_OK si éxito, SSD1306_ERROR si puntero nulo
 * @note   Solo soporta caracteres ASCII 32-127. Caracteres fuera de rango se dibujan como '?'.
 *         El texto se dibuja horizontalmente y puede recortar si excede el ancho del display.
 */
int8_t SSD1306_DrawText(SSD1306_HandleTypeDef *hssd, uint8_t x, uint8_t y, const char *text)
{
    // Validate handle and text pointer
    if (hssd == NULL || text == NULL)
        return SSD1306_ERROR;
    uint8_t cursor_x = x;
    while (*text && cursor_x < SSD1306_WIDTH)
    {
        char c = *text;
        if (c < 32 || c > 127)
        {
            c = '?'; // Replace unsupported characters
        }
        const uint8_t *glyph = font_5x7[c - 32];
        for (uint8_t dx = 0; dx < 5 && (cursor_x + dx) < SSD1306_WIDTH; dx++)
        {
            uint8_t col = glyph[dx];
            for (uint8_t dy = 0; dy < 7 && (y + dy) < SSD1306_HEIGHT; dy++)
            {
                bool pixel_on = (col >> dy) & 0x01;
                SSD1306_DrawPixel(hssd, cursor_x + dx, y + dy, pixel_on);
            }
        }
        cursor_x += 6; // 5px char + 1px space
        text++;
    }
    return SSD1306_OK;
}

/**
 * @brief  Actualiza el display usando DMA (no bloqueante)
 * @param  hssd: puntero al handle SSD1306
 * @param  update_done_cb: callback a invocar al finalizar DMA
 * @retval SSD1306_OK si éxito, SSD1306_BUSY si ya hay transferencia, SSD1306_ERROR si falla
 * @note   No bloquea la ejecución; el callback se invoca al finalizar la transferencia DMA.
 *         Si DMA está ocupado, retorna SSD1306_BUSY y no inicia nueva transferencia.
 *         Compatible con STM32CubeIDE: configurar interrupciones I2C y DMA en el archivo .ioc.
 */
int8_t SSD1306_UpdateScreen_DMA(SSD1306_HandleTypeDef *hssd, SSD1306_UpdateDoneCallback update_done_cb)
{
    // Validate handle, buffer, and DMA function pointer
    if (hssd == NULL || hssd->i2c_write_dma == NULL)
        return SSD1306_ERROR;
    // Defensive: Check if DMA is busy
    if (hssd->dma_busy)
        return SSD1306_BUSY;
    // Defensive: Check if blocking write is available
    if (hssd->i2c_write_blocking == NULL)
        return SSD1306_ERROR;
    // Set column and page addresses for horizontal mode
    uint8_t cmd;
    cmd = 0x21; // Set column address
    if (hssd->i2c_write_blocking(hssd->device_address, 0x00, &cmd, 1, hssd->i2c_context) != SSD1306_OK)
        return SSD1306_ERROR;
    cmd = 0x00; // Start column
    if (hssd->i2c_write_blocking(hssd->device_address, 0x00, &cmd, 1, hssd->i2c_context) != SSD1306_OK)
        return SSD1306_ERROR;
    cmd = SSD1306_WIDTH - 1; // End column
    if (hssd->i2c_write_blocking(hssd->device_address, 0x00, &cmd, 1, hssd->i2c_context) != SSD1306_OK)
        return SSD1306_ERROR;
    cmd = 0x22; // Set page address
    if (hssd->i2c_write_blocking(hssd->device_address, 0x00, &cmd, 1, hssd->i2c_context) != SSD1306_OK)
        return SSD1306_ERROR;
    cmd = 0x00; // Start page
    if (hssd->i2c_write_blocking(hssd->device_address, 0x00, &cmd, 1, hssd->i2c_context) != SSD1306_OK)
        return SSD1306_ERROR;
    cmd = (SSD1306_HEIGHT / 8) - 1; // End page
    if (hssd->i2c_write_blocking(hssd->device_address, 0x00, &cmd, 1, hssd->i2c_context) != SSD1306_OK)
        return SSD1306_ERROR;
    // Prepare buffer for DMA (prefix 0x40)
    static uint8_t dma_buf[SSD1306_BUFFER_SIZE + 1];
    dma_buf[0] = 0x40;
    for (uint16_t i = 0; i < SSD1306_BUFFER_SIZE; i++)
        dma_buf[1 + i] = hssd->buffer[i];
    hssd->dma_busy = true;
    hssd->update_done_cb = update_done_cb;
    // Start DMA transfer (skip first byte for reg_addr)
    int8_t res = hssd->i2c_write_dma(hssd->device_address, dma_buf[0], &dma_buf[1], SSD1306_BUFFER_SIZE, hssd->i2c_context);
    if (res != SSD1306_OK)
    {
        hssd->dma_busy = false;
        hssd->update_done_cb = NULL;
        return SSD1306_ERROR;
    }
    return SSD1306_OK;
}
