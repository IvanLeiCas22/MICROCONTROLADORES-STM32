/**
 * @file SSD1306.c
 * @brief OLED SSD1306 Display Library Implementation (Robot Seguidor de Laberinto)
 * @author GitHub Copilot
 */

#include "SSD1306.h"
#include <string.h>
#include <stddef.h>

/* Internal SSD1306 commands */
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

#define SSD1306_BYTE_INDEX(x, y) (((y) / 8) * SSD1306_WIDTH + (x))
#define SSD1306_BIT_MASK(y) (1 << ((y) % 8))

/* 5x7 ASCII font (characters 32-127) */
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
    // ... font data ...
};

/* Public API implementation */

int8_t SSD1306_Init(SSD1306_HandleTypeDef *hssd)
{
    if (hssd == NULL || hssd->i2c_write_blocking == NULL)
        return SSD1306_ERROR;
    uint8_t cmds[] = {
        SSD1306_CMD_DISPLAY_OFF,
        SSD1306_CMD_SET_DISPLAY_CLOCK_DIV, 0x80,
        SSD1306_CMD_SET_MULTIPLEX, SSD1306_HEIGHT - 1,
        SSD1306_CMD_SET_DISPLAY_OFFSET, 0x00,
        SSD1306_CMD_SET_START_LINE | 0x00,
        SSD1306_CMD_SET_CHARGE_PUMP, 0x14,
        SSD1306_CMD_SET_MEMORY_MODE, 0x00,
        SSD1306_CMD_SET_SEG_REMAP | 0x01,
        SSD1306_CMD_SET_COM_SCAN_DEC,
        SSD1306_CMD_SET_COM_PINS, 0x12,
        SSD1306_CMD_SET_CONTRAST, 0xCF,
        SSD1306_CMD_SET_PRECHARGE, 0xF1,
        SSD1306_CMD_SET_VCOM_DETECT, 0x40,
        SSD1306_CMD_SET_DISPLAY_ALL_ON_RESUME,
        SSD1306_CMD_SET_NORMAL_DISPLAY,
        SSD1306_CMD_DISPLAY_ON};
    for (uint8_t i = 0; i < sizeof(cmds); i++)
    {
        if (hssd->i2c_write_blocking(hssd->device_address, 0x00, &cmds[i], 1, hssd->i2c_context) != SSD1306_OK)
            return SSD1306_ERROR;
    }
    memset(hssd->buffer, 0x00, SSD1306_BUFFER_SIZE);

    SSD1306_SetFont(hssd, &font_5x7[0][0], 5, 7);

    hssd->is_initialized = true;
    return SSD1306_OK;
}

int8_t SSD1306_DrawPixel(SSD1306_HandleTypeDef *hssd, uint8_t x, uint8_t y, bool color)
{
    if (!hssd || !hssd->is_initialized)
        return SSD1306_ERROR;
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT)
        return SSD1306_ERROR;
    uint16_t idx = SSD1306_BYTE_INDEX(x, y);
    if (color)
        hssd->buffer[idx] |= SSD1306_BIT_MASK(y);
    else
        hssd->buffer[idx] &= ~SSD1306_BIT_MASK(y);
    return SSD1306_OK;
}

int8_t SSD1306_DrawLine(SSD1306_HandleTypeDef *hssd, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, bool color)
{
    int16_t dx = (x1 > x0) ? (x1 - x0) : (x0 - x1);
    int16_t dy = (y1 > y0) ? (y1 - y0) : (y0 - y1);
    int16_t sx = (x0 < x1) ? 1 : -1;
    int16_t sy = (y0 < y1) ? 1 : -1;
    int16_t err = dx - dy;
    while (1)
    {
        SSD1306_DrawPixel(hssd, x0, y0, color);
        if (x0 == x1 && y0 == y1)
            break;
        int16_t e2 = 2 * err;
        if (e2 > -dy)
        {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx)
        {
            err += dx;
            y0 += sy;
        }
    }
    return SSD1306_OK;
}

int8_t SSD1306_DrawRect(SSD1306_HandleTypeDef *hssd, uint8_t x, uint8_t y, uint8_t w, uint8_t h, bool color)
{
    SSD1306_DrawLine(hssd, x, y, x + w - 1, y, color);
    SSD1306_DrawLine(hssd, x, y + h - 1, x + w - 1, y + h - 1, color);
    SSD1306_DrawLine(hssd, x, y, x, y + h - 1, color);
    SSD1306_DrawLine(hssd, x + w - 1, y, x + w - 1, y + h - 1, color);
    return SSD1306_OK;
}

int8_t SSD1306_DrawBitmap(SSD1306_HandleTypeDef *hssd, uint8_t x, uint8_t y, const uint8_t *bitmap, uint8_t w, uint8_t h)
{
    if (!hssd || !bitmap)
        return SSD1306_ERROR;
    for (uint8_t j = 0; j < h; j++)
    {
        for (uint8_t i = 0; i < w; i++)
        {
            uint8_t byte = bitmap[i + (j / 8) * w];
            if (byte & (1 << (j % 8)))
                SSD1306_DrawPixel(hssd, x + i, y + j, true);
            else
                SSD1306_DrawPixel(hssd, x + i, y + j, false);
        }
    }
    return SSD1306_OK;
}

int8_t SSD1306_DrawText(SSD1306_HandleTypeDef *hssd, uint8_t x, uint8_t y, const char *text, SSD1306_TextAlign align)
{
    if (!hssd || !text || !hssd->font)
        return SSD1306_ERROR;

    uint8_t char_width_spacing = hssd->font_width + 1; // Ancho del caracter + 1 píxel de espacio
    uint16_t text_len = strlen(text);
    uint16_t text_width = text_len * char_width_spacing;
    int16_t start_x = x;

    if (align == SSD1306_TEXT_ALIGN_CENTER)
    {
        start_x = (SSD1306_WIDTH - text_width) / 2;
    }
    else if (align == SSD1306_TEXT_ALIGN_RIGHT)
    {
        start_x = SSD1306_WIDTH - text_width;
    }

    for (uint16_t i = 0; i < text_len; i++)
    {
        char c = text[i];
        if (c < 32 || c > 127)
            c = '?';

        int16_t current_char_x = start_x + (i * char_width_spacing);

        // Omitir caracteres que están completamente fuera de la pantalla
        if (current_char_x >= SSD1306_WIDTH || (current_char_x + hssd->font_width) < 0)
        {
            continue;
        }

        const uint8_t *font_char = &hssd->font[(c - 32) * hssd->font_width];

        for (uint8_t col = 0; col < hssd->font_width; col++)
        {
            int16_t target_x = current_char_x + col;
            if (target_x >= SSD1306_WIDTH || target_x < 0)
                continue; // Omitir columnas fuera de la pantalla

            uint8_t font_byte = font_char[col];
            uint8_t y_offset = y % 8;

            // Máscara para limpiar el área del caracter (considerando su altura)
            uint16_t mask = (1 << hssd->font_height) - 1;

            // Desplazar datos y máscara a la posición vertical correcta
            uint16_t shifted_data = font_byte << y_offset;
            uint16_t shifted_mask = mask << y_offset;

            uint8_t page1_data = shifted_data & 0xFF;
            uint8_t page2_data = (shifted_data >> 8) & 0xFF;
            uint8_t page1_mask = shifted_mask & 0xFF;
            uint8_t page2_mask = (shifted_mask >> 8) & 0xFF;

            // Escribir en la primera página afectada
            uint16_t idx1 = (y / 8) * SSD1306_WIDTH + target_x;
            if (idx1 < SSD1306_BUFFER_SIZE)
            {
                hssd->buffer[idx1] = (hssd->buffer[idx1] & ~page1_mask) | page1_data;
            }

            // Escribir en la segunda página si el caracter se extiende a ella
            if (page2_data || page2_mask)
            {
                uint16_t idx2 = (y / 8 + 1) * SSD1306_WIDTH + target_x;
                if (idx2 < SSD1306_BUFFER_SIZE)
                {
                    hssd->buffer[idx2] = (hssd->buffer[idx2] & ~page2_mask) | page2_data;
                }
            }
        }
    }
    return SSD1306_OK;
}

int8_t SSD1306_DrawMultilineText(SSD1306_HandleTypeDef *hssd, uint8_t x, uint8_t y, const char *text)
{
    if (!hssd || !text)
        return SSD1306_ERROR;
    uint8_t line = 0, col = 0;
    for (uint16_t i = 0; text[i] != '\0'; i++)
    {
        if (text[i] == '\n' || col >= (SSD1306_WIDTH / 6))
        {
            line++;
            col = 0;
            continue;
        }
        SSD1306_DrawText(hssd, x + col * 6, y + line * 8, &text[i], SSD1306_TEXT_ALIGN_LEFT);
        col++;
    }
    return SSD1306_OK;
}

int8_t SSD1306_UpdateScreen_DMA(SSD1306_HandleTypeDef *hssd)
{
    if (!hssd || !hssd->i2c_write_blocking || !hssd->i2c_write_dma)
    {
        return SSD1306_BUSY;
    }

    // Comandos para resetear el cursor a la posición inicial (0,0)
    uint8_t cmds[] = {
        0x21, // Set Column Address
        0,    // Start
        127,  // End
        0x22, // Set Page Address
        0,    // Start
        7     // End
    };

    // Envía los comandos de posicionamiento en modo bloqueante
    for (uint8_t i = 0; i < sizeof(cmds); i++)
    {
        if (hssd->i2c_write_blocking(hssd->device_address, 0x00, &cmds[i], 1, hssd->i2c_context) != SSD1306_OK)
        {
            return SSD1306_ERROR;
        }
    }

    // Ahora, transfiere el framebuffer completo por DMA
    if (hssd->i2c_write_dma(hssd->device_address, 0x40, hssd->buffer, SSD1306_BUFFER_SIZE, hssd->i2c_context) != SSD1306_OK)
    {
        return SSD1306_ERROR;
    }

    return SSD1306_OK;
}

int8_t SSD1306_Clear(SSD1306_HandleTypeDef *hssd)
{
    if (!hssd)
        return SSD1306_ERROR;
    memset(hssd->buffer, 0x00, SSD1306_BUFFER_SIZE);
    return SSD1306_OK;
}

int8_t SSD1306_SetContrast(SSD1306_HandleTypeDef *hssd, uint8_t contrast)
{
    if (!hssd || !hssd->i2c_write_blocking)
        return SSD1306_ERROR;
    uint8_t cmd[2] = {SSD1306_CMD_SET_CONTRAST, contrast};
    return hssd->i2c_write_blocking(hssd->device_address, 0x00, cmd, 2, hssd->i2c_context);
}

int8_t SSD1306_DisplayOn(SSD1306_HandleTypeDef *hssd)
{
    if (!hssd || !hssd->i2c_write_blocking)
        return SSD1306_ERROR;
    uint8_t cmd = SSD1306_CMD_DISPLAY_ON;
    return hssd->i2c_write_blocking(hssd->device_address, 0x00, &cmd, 1, hssd->i2c_context);
}

int8_t SSD1306_DisplayOff(SSD1306_HandleTypeDef *hssd)
{
    if (!hssd || !hssd->i2c_write_blocking)
        return SSD1306_ERROR;
    uint8_t cmd = SSD1306_CMD_DISPLAY_OFF;
    return hssd->i2c_write_blocking(hssd->device_address, 0x00, &cmd, 1, hssd->i2c_context);
}

void SSD1306_SetFont(SSD1306_HandleTypeDef *hssd, const uint8_t *font, uint8_t width, uint8_t height)
{
    if (hssd)
    {
        hssd->font = font;
        hssd->font_width = width;
        hssd->font_height = height;
    }
}