/* STM32/Test2024-master/Core/Inc/app_core.h */
#ifndef INC_APP_CORE_H_
#define INC_APP_CORE_H_

#include "main.h"

/*
 *  Declaraciones de las funciones principales del núcleo de la aplicación.
 *  Estas funciones son llamadas desde main.c para inicializar y ejecutar
 *  la lógica principal del programa.
 */

void App_Core_Init(void);
void App_Core_Loop(void);

/*
 *  Declaraciones de los "wrappers" de callbacks.
 *  Estas funciones son llamadas desde las verdaderas interrupciones HAL en main.c,
 *  permitiendo que toda la lógica de manejo de interrupciones resida en app_core.c
 *  y manteniendo main.c limpio.
 */
void App_Core_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void App_Core_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);
void App_Core_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void App_Core_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c);
void App_Core_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);
void App_Core_USB_ReceiveData(uint8_t *buf, uint16_t len);

#endif /* INC_APP_CORE_H_ */