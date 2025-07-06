#include "ssd1306.h"
#include <string.h> // Para memset

#define SSD1306_I2C_ADDR  (0x3C << 1) // Dirección típica para SSD1306 I2C
#define SSD1306_WIDTH     128
#define SSD1306_HEIGHT    64
#define SSD1306_BUF_SIZE  (SSD1306_WIDTH * SSD1306_HEIGHT / 8)

// Comandos SSD1306
#define SSD1306_CMD_DISPLAY_OFF          0xAE
#define SSD1306_CMD_DISPLAY_ON           0xAF
#define SSD1306_CMD_SET_DISPLAY_CLOCK    0xD5
#define SSD1306_CMD_SET_MULTIPLEX        0xA8
#define SSD1306_CMD_SET_DISPLAY_OFFSET   0xD3
#define SSD1306_CMD_SET_START_LINE       0x40
#define SSD1306_CMD_CHARGE_PUMP          0x8D
#define SSD1306_CMD_MEMORY_MODE          0x20
#define SSD1306_CMD_SEG_REMAP            0xA1
#define SSD1306_CMD_COM_SCAN_DEC         0xC8
#define SSD1306_CMD_SET_CONTRAST         0x81
#define SSD1306_CMD_SET_PRECHARGE        0xD9
#define SSD1306_CMD_SET_VCOM_DETECT      0xDB
#define SSD1306_CMD_DISPLAY_ALL_ON_RESUME 0xA4
#define SSD1306_CMD_NORMAL_DISPLAY       0xA6
#define SSD1306_CMD_DEACTIVATE_SCROLL    0x2E
#define SSD1306_CMD_SET_COM_PINS         0xDA

// --- Función interna para enviar un comando ---
static int8_t ssd1306_write_cmd(SSD1306_t *dev, uint8_t cmd) {
    uint8_t data[2] = {0x00, cmd}; // 0x00 = Co=0, D/C#=0 (comando)
    return dev->iface.i2c_write(SSD1306_I2C_ADDR, data[0], &data[1], 1, dev->iface.user_ctx);
}

// Callback interno para notificar al usuario (debe ser visible a nivel de archivo)
static void ssd1306_async_cb(void *ctx) {
    SSD1306_t *d = (SSD1306_t *)ctx;
    d->ready = 1;
    if (d->user_cb) d->user_cb(d);
}

// --- Implementación de funciones principales ---

int8_t SSD1306_Init(SSD1306_t *dev) {
    if (!dev || !dev->iface.i2c_write) return -1;
    dev->width = SSD1306_WIDTH;
    dev->height = SSD1306_HEIGHT;
    dev->ready = 0;
    SSD1306_Clear(dev);

    // Secuencia de inicialización recomendada (modo 128x64, addressing horizontal)
    int8_t res = 0;
    res |= ssd1306_write_cmd(dev, SSD1306_CMD_DISPLAY_OFF);
    res |= ssd1306_write_cmd(dev, SSD1306_CMD_SET_DISPLAY_CLOCK); res |= ssd1306_write_cmd(dev, 0x80);
    res |= ssd1306_write_cmd(dev, SSD1306_CMD_SET_MULTIPLEX);     res |= ssd1306_write_cmd(dev, 0x3F);
    res |= ssd1306_write_cmd(dev, SSD1306_CMD_SET_DISPLAY_OFFSET);res |= ssd1306_write_cmd(dev, 0x00);
    res |= ssd1306_write_cmd(dev, SSD1306_CMD_SET_START_LINE | 0x00);
    res |= ssd1306_write_cmd(dev, SSD1306_CMD_CHARGE_PUMP);       res |= ssd1306_write_cmd(dev, 0x14);
    res |= ssd1306_write_cmd(dev, SSD1306_CMD_MEMORY_MODE);       res |= ssd1306_write_cmd(dev, 0x00); // Horizontal addressing
    res |= ssd1306_write_cmd(dev, SSD1306_CMD_SEG_REMAP);
    res |= ssd1306_write_cmd(dev, SSD1306_CMD_COM_SCAN_DEC);
    res |= ssd1306_write_cmd(dev, SSD1306_CMD_SET_COM_PINS);      res |= ssd1306_write_cmd(dev, 0x12);
    res |= ssd1306_write_cmd(dev, SSD1306_CMD_SET_CONTRAST);      res |= ssd1306_write_cmd(dev, 0xCF);
    res |= ssd1306_write_cmd(dev, SSD1306_CMD_SET_PRECHARGE);     res |= ssd1306_write_cmd(dev, 0xF1);
    res |= ssd1306_write_cmd(dev, SSD1306_CMD_SET_VCOM_DETECT);   res |= ssd1306_write_cmd(dev, 0x40);
    res |= ssd1306_write_cmd(dev, SSD1306_CMD_DISPLAY_ALL_ON_RESUME);
    res |= ssd1306_write_cmd(dev, SSD1306_CMD_NORMAL_DISPLAY);
    res |= ssd1306_write_cmd(dev, SSD1306_CMD_DEACTIVATE_SCROLL);
    res |= SSD1306_DisplayOn(dev);

    dev->ready = (res == 0) ? 1 : 0;
    return res;
}

int8_t SSD1306_DisplayOn(SSD1306_t *dev) {
    if (!dev) return -1;
    return ssd1306_write_cmd(dev, SSD1306_CMD_DISPLAY_ON);
}

int8_t SSD1306_DisplayOff(SSD1306_t *dev) {
    if (!dev) return -1;
    return ssd1306_write_cmd(dev, SSD1306_CMD_DISPLAY_OFF);
}

int8_t SSD1306_UpdateScreen(SSD1306_t *dev) {
    if (!dev || !dev->iface.i2c_write) return -1;
    int8_t res = 0;
    // Set column and page addresses
    res |= ssd1306_write_cmd(dev, 0x21); // Set column address
    res |= ssd1306_write_cmd(dev, 0x00); // Start
    res |= ssd1306_write_cmd(dev, dev->width - 1); // End
    res |= ssd1306_write_cmd(dev, 0x22); // Set page address
    res |= ssd1306_write_cmd(dev, 0x00); // Start
    res |= ssd1306_write_cmd(dev, (dev->height / 8) - 1); // End
    // Enviar buffer (en bloques si es necesario)
    // El primer byte debe ser 0x40 (Co=0, D/C#=1)
    uint8_t data[17];
    data[0] = 0x40;
    for (uint16_t i = 0; i < SSD1306_BUF_SIZE; i += 16) {
        uint8_t chunk = (SSD1306_BUF_SIZE - i > 16) ? 16 : (SSD1306_BUF_SIZE - i);
        memcpy(&data[1], &dev->buffer[i], chunk);
        res |= dev->iface.i2c_write(SSD1306_I2C_ADDR, data[0], &data[1], chunk, dev->iface.user_ctx);
    }
    dev->dirty = 0; // Limpia el flag tras actualizar
    return res;
}

// --- Actualiza la pantalla (asíncrono) ---
int8_t SSD1306_UpdateScreen_Async(SSD1306_t *dev, void (*cb)(SSD1306_t *dev)) {
    if (!dev || !dev->iface.i2c_write_async) return -1;
    dev->user_cb = cb;
    // Solo soporta envío en un solo bloque (puede ampliarse a chunking si es necesario)
    // El primer byte debe ser 0x40 (Co=0, D/C#=1)
    static uint8_t data[SSD1306_BUF_SIZE + 1];
    data[0] = 0x40;
    memcpy(&data[1], dev->buffer, SSD1306_BUF_SIZE);
    // Set column and page addresses (sincrónico, previo al envío)
    int8_t res = 0;
    res |= ssd1306_write_cmd(dev, 0x21); // Set column address
    res |= ssd1306_write_cmd(dev, 0x00); // Start
    res |= ssd1306_write_cmd(dev, dev->width - 1); // End
    res |= ssd1306_write_cmd(dev, 0x22); // Set page address
    res |= ssd1306_write_cmd(dev, 0x00); // Start
    res |= ssd1306_write_cmd(dev, (dev->height / 8) - 1); // End
    if (res != 0) return -2;
    dev->ready = 0;
    dev->dirty = 0; // Limpia el flag tras actualizar
    // Enviar buffer asíncrono
    return dev->iface.i2c_write_async(SSD1306_I2C_ADDR, data[0], &data[1], SSD1306_BUF_SIZE, dev->iface.user_ctx, ssd1306_async_cb, dev);
}

void SSD1306_Clear(SSD1306_t *dev) {
    if (!dev) return;
    memset(dev->buffer, 0x00, SSD1306_BUF_SIZE);
    dev->dirty = 1;
}

void SSD1306_DrawPixel(SSD1306_t *dev, uint8_t x, uint8_t y, uint8_t color) {
    if (!dev) return;
    if (x >= dev->width || y >= dev->height) return;
    uint16_t byte_index = x + (y / 8) * dev->width;
    if (color)
        dev->buffer[byte_index] |= (1 << (y % 8));
    else
        dev->buffer[byte_index] &= ~(1 << (y % 8));
    dev->dirty = 1;
}

void SSD1306_DrawChar(SSD1306_t *dev, uint8_t x, uint8_t y, uint8_t c, const SSD1306_Font_t *font, uint8_t color) {
    if (!dev || !font) return;
    if (x >= dev->width || y >= dev->height) return;
    if (c < font->first_char || c > font->last_char) c = ' '; // Sustituir fuera de rango por espacio
    uint8_t char_index = c - font->first_char;
    uint16_t bytes_per_char = font->width * ((font->height + 7) / 8);
    const uint8_t *char_data = &font->data[char_index * bytes_per_char];
    for (uint8_t col = 0; col < font->width; col++) {
        for (uint8_t row = 0; row < font->height; row++) {
            uint8_t byte = char_data[col + (row / 8) * font->width];
            uint8_t pixel_on = (byte >> (row % 8)) & 0x1;
            if (pixel_on) {
                SSD1306_DrawPixel(dev, x + col, y + row, color);
            } else {
                SSD1306_DrawPixel(dev, x + col, y + row, !color); // Fondo
            }
        }
    }
    dev->dirty = 1;
}

void SSD1306_DrawString(SSD1306_t *dev, uint8_t x, uint8_t y, const uint8_t *str, const SSD1306_Font_t *font, uint8_t color) {
    if (!dev || !font || !str) return;
    uint8_t orig_x = x;
    while (*str) {
        if (x + font->width > dev->width) {
            x = orig_x;
            y += font->height;
        }
        if (y + font->height > dev->height) break;
        SSD1306_DrawChar(dev, x, y, *str, font, color);
        x += font->width;
        str++;
    }
    dev->dirty = 1;
} 