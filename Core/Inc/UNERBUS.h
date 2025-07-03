/*
 * UNERBUS.h
 *
 * Created: 04/01/24 08:48:38
 *  Author: German
 */ 

/**
 * @file UNERBUS.h
 * @brief Protocolo de comunicación UNERBUS para sistemas embebidos.
 *
 * Estructura de paquete:
 * | HEADER (4B) | LENGTH (1B) | TOKEN (1B) | CMD (1B) | PAYLOAD (N) | CHECKSUM (1B) |
 * HEADER: "UNER" (0x55 0x4E 0x45 0x52)
 * LENGTH: Cantidad de bytes a enviar (CMD + PAYLOAD + CHECKSUM)
 * TOKEN: ':' (0x3A)
 * CMD: Código de comando
 * PAYLOAD: Datos útiles
 * CHECKSUM: XOR de todos los bytes anteriores, incluyendo HEADER
 *
 * El protocolo es robusto, eficiente y no bloqueante, soportando buffers circulares y callbacks para manejo de eventos.
 */

#ifndef UNERBUS_H_
#define UNERBUS_H_

#include <stdint.h>

//#define AVRGCC_ATMEGA

/**
 * @struct UNERBUSHandle
 * @brief Handler de UNERBUS para gestión de buffers y eventos.
 *
 * rx: Buffer de recepción y control de estado.
 * tx: Buffer de transmisión y control de estado.
 * MyDataReady: Callback invocado al recibir un paquete válido.
 * WriteUSARTByte: Callback para enviar un byte por USART (opcional).
 * iiTXw: Índice auxiliar para transmisión.
 */
typedef struct UNERBUSHandle
{
	struct{
		uint8_t		*buf;                /**< Buffer circular de recepción */
		uint8_t		maxIndexRingBuf;     /**< Tamaño máximo del buffer */
		uint8_t		iRead;               /**< Índice de lectura */
		uint8_t		iWrite;              /**< Índice de escritura */
		uint8_t		iData;               /**< Índice de datos */
		uint8_t		newData;             /**< Flag de nuevo dato */
		uint8_t		nBytes;              /**< Bytes restantes del paquete */
		uint8_t		header;              /**< Estado de la cabecera */
		uint8_t		timeout;             /**< Timeout de recepción */
		uint8_t		cks;                 /**< Checksum calculado */
	} rx;
	struct{
		uint8_t		*buf;                /**< Buffer circular de transmisión */
		uint8_t		maxIndexRingBuf;     /**< Tamaño máximo del buffer */
		uint8_t		iRead;               /**< Índice de lectura */
		uint8_t		iWrite;              /**< Índice de escritura */
		uint8_t		cks;                 /**< Checksum calculado */
	} tx;
	
	void (*MyDataReady)(struct UNERBUSHandle *aBus, uint8_t iStartData); /**< Callback al recibir paquete válido */
	uint8_t (*WriteUSARTByte)(uint8_t value); /**< Callback para enviar byte por USART */
	
	uint8_t			iiTXw; /**< Índice auxiliar de transmisión */
} _sUNERBUSHandle;

/**
 * @brief Inicializa el handler de UNERBUS.
 * @param aBus Puntero al handler.
 */
void UNERBUS_Init(_sUNERBUSHandle *aBus);

/**
 * @brief Escribe un buffer en el buffer de transmisión.
 * @param aBus Handler.
 * @param buf Buffer de datos.
 * @param lenBuf Longitud del buffer.
 */
void UNERBUS_Write(_sUNERBUSHandle *aBus, uint8_t *buf, uint8_t lenBuf);

/**
 * @brief Escribe un byte en el buffer de transmisión.
 * @param aBus Handler.
 * @param value Byte a escribir.
 */
void UNERBUS_WriteByte(_sUNERBUSHandle *aBus, uint8_t value);

/**
 * @brief Envía un paquete (arma cabecera, payload y checksum).
 * @param aBus Handler.
 * @param cmdID Comando.
 * @param lenCMD Longitud del payload + 1 (por el comando).
 */
void UNERBUS_Send(_sUNERBUSHandle *aBus, uint8_t cmdID, uint8_t lenCMD);

/**
 * @brief Envía un paquete a un buffer externo.
 */
void UNERBUS_SendToBuf(_sUNERBUSHandle *aBus, uint8_t cmdID, uint8_t lenCMD, uint8_t *bufForSend);

/**
 * @brief Procesa un byte recibido (recepción).
 */
void UNERBUS_ReceiveByte(_sUNERBUSHandle *aBus, uint8_t value);

/**
 * @brief Procesa un buffer recibido (recepción).
 */
void UNERBUS_ReceiveBuf(_sUNERBUSHandle *aBus, uint8_t *buf, uint8_t lenBuf);

/**
 * @brief Escribe una cadena constante en el buffer de transmisión.
 */
void UNERBUS_WriteConstString(_sUNERBUSHandle *aBus, const char *buf, uint8_t lastString);

/**
 * @brief Extrae un buffer del paquete recibido.
 */
void UNERBUS_GetBuf(_sUNERBUSHandle *aBus, uint8_t *buf, uint8_t lenBuf);

/**
 * @brief Extrae un uint8_t del paquete recibido.
 */
uint8_t UNERBUS_GetUInt8(_sUNERBUSHandle *aBus);

/**
 * @brief Extrae un int8_t del paquete recibido.
 */
int8_t UNERBUS_GetInt8(_sUNERBUSHandle *aBus);

/**
 * @brief Extrae un uint32_t del paquete recibido.
 */
uint32_t UNERBUS_GetUInt32(_sUNERBUSHandle *aBus);

/**
 * @brief Extrae un int32_t del paquete recibido.
 */
int32_t UNERBUS_GetInt32(_sUNERBUSHandle *aBus);

/**
 * @brief Extrae un uint16_t del paquete recibido.
 */
uint16_t UNERBUS_GetUInt16(_sUNERBUSHandle *aBus);

/**
 * @brief Extrae un int16_t del paquete recibido.
 */
int16_t UNERBUS_GetInt16(_sUNERBUSHandle *aBus);

/**
 * @brief Extrae un float del paquete recibido.
 */
float UNERBUS_GetFloat(_sUNERBUSHandle *aBus);

/**
 * @brief Obtiene el índice de lectura actual.
 */
uint8_t UNERBUS_GetIndexRead(_sUNERBUSHandle *aBus);

/**
 * @brief Mueve el índice de lectura.
 */
void UNERBUS_MoveIndexRead(_sUNERBUSHandle *aBus, int8_t newIndexRead);

/**
 * @brief Resetea el flag de nuevo dato.
 */
void UNERBUS_ResetNewData(_sUNERBUSHandle *aBus);

/**
 * @brief Procesa tareas pendientes (decodificación y transmisión).
 */
void UNERBUS_Task(_sUNERBUSHandle *aBus);

/**
 * @brief Maneja el timeout de recepción.
 */
void UNERBUS_Timeout(_sUNERBUSHandle *aBus);

/**
 * @brief Asocia un callback para evento de datos listos.
 */
void UNERBUS_AttachOnDataReady(void (*aOnDataReady)(_sUNERBUSHandle *aBus, uint8_t iStartData));

#endif /* UNERBUS_H_ */
