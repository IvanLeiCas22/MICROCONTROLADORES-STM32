#ifndef SSD1306_H
#define SSD1306_H

#include <stdint.h>

/**
 * @file ssd1306.h
 * @brief Driver portable y agnóstico para SSD1306 (solo C estándar, sin dependencias de hardware).
 *        Resolución fija 128x64, interfaz I2C, buffer local, soporte asíncrono.
 */

// Interfaz de bajo nivel para acceso I2C (el usuario debe implementarla)
typedef struct {
    int8_t (*i2c_write)(uint8_t dev_addr, uint8_t reg, const uint8_t *data, uint16_t len, void *user_ctx);
    int8_t (*i2c_read)(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint16_t len, void *user_ctx);
    int8_t (*i2c_write_async)(uint8_t dev_addr, uint8_t reg, const uint8_t *data, uint16_t len, void *user_ctx, void (*cb)(void *cb_ctx), void *cb_ctx);
    void *user_ctx; // Contexto de usuario (ej: handler, struct, etc.)
} SSD1306_I2C_Interface_t;

// Estructura de fuente
typedef struct {
    const uint8_t *data; // Puntero a los datos de la fuente
    uint8_t width;
    uint8_t height;
    uint8_t first_char;
    uint8_t last_char;
} SSD1306_Font_t;

// Estructura principal del driver
typedef struct SSD1306_t {
    SSD1306_I2C_Interface_t iface;
    uint8_t buffer[128 * 64 / 8]; // 1 bit por pixel
    uint8_t width, height;
    volatile uint8_t ready;
    volatile uint8_t dirty; // Flag: 1 si el buffer fue modificado y requiere actualización
    void (*user_cb)(struct SSD1306_t *dev); // Callback de usuario para async
} SSD1306_t;

#ifdef __cplusplus
extern "C" {
#endif

// --- API pública ---

// Inicializa el display SSD1306 (I2C, buffer, configuración básica)
int8_t SSD1306_Init(SSD1306_t *dev);

// Enciende/apaga el display
int8_t SSD1306_DisplayOn(SSD1306_t *dev);
int8_t SSD1306_DisplayOff(SSD1306_t *dev);

// Actualiza la pantalla (sincrónico)
int8_t SSD1306_UpdateScreen(SSD1306_t *dev);
// Actualiza la pantalla (asíncrono)
int8_t SSD1306_UpdateScreen_Async(SSD1306_t *dev, void (*cb)(SSD1306_t *dev));

// Limpia el buffer de pantalla
void SSD1306_Clear(SSD1306_t *dev);

// Dibuja un pixel en el buffer
void SSD1306_DrawPixel(SSD1306_t *dev, uint8_t x, uint8_t y, uint8_t color);

// Dibuja un caracter usando una fuente
void SSD1306_DrawChar(SSD1306_t *dev, uint8_t x, uint8_t y, uint8_t c, const SSD1306_Font_t *font, uint8_t color);

// Dibuja una cadena de texto
void SSD1306_DrawString(SSD1306_t *dev, uint8_t x, uint8_t y, const uint8_t *str, const SSD1306_Font_t *font, uint8_t color);

// --- Fuentes incluidas ---
extern const SSD1306_Font_t SSD1306_Font_Small;
extern const SSD1306_Font_t SSD1306_Font_Title;

#ifdef __cplusplus
}
#endif

#endif // SSD1306_H 