/*
 * UNERBUS.c
 *
 * Created: 04/01/24 08:49:00
 *  Author: German
 */ 

#include "UNERBUS.h"
#include <stdlib.h>

#ifdef AVRGCC_ATMEGA
#include <avr/pgmspace.h>
#endif

/**
 * @brief Unión de trabajo para conversión de tipos en la extracción de datos del payload.
 */
typedef union{
	uint8_t		u8[4];
	uint8_t		i8[4];
	uint16_t	u16[2];
	int16_t		i16[2];
	uint32_t	u32;
	int32_t		i32;
	float		f;	
}_uUNERBUSWork;

/**
 * @brief HEADER base del protocolo UNERBUS.
 * HEADER[0..3]: 'U','N','E','R'
 * HEADER[4]: LENGTH (CMD+PAYLOAD+CHECKSUM)
 * HEADER[5]: TOKEN ':'
 * HEADER[6]: CMD
 */
static uint8_t HEADER[7] = {'U', 'N', 'E', 'R', 0x00, ':', 0x00};

static _uUNERBUSWork w;

/**
 * @brief Decodifica la cabecera y el paquete recibido en el buffer circular.
 * Si el paquete es válido y el checksum coincide, invoca el callback MyDataReady.
 * Maneja timeouts y reinicio de estado ante errores.
 */
static void UNERBUS_DecodeHeader(_sUNERBUSHandle *aBus){
	uint8_t value;
	uint8_t index = aBus->rx.iWrite;

	while (aBus->rx.iRead != index)
	{
		value = aBus->rx.buf[aBus->rx.iRead];
		// Máquina de estados para decodificar la cabecera y el paquete
		switch(aBus->rx.header){
		case 0:
			// Espera el primer byte de la cabecera ('U')
			if(value == HEADER[aBus->rx.header]){
				aBus->rx.header = 1;
				aBus->rx.timeout = 5;
				aBus->rx.cks = value;
			}
			break;
		case 1:
		case 2:
		case 3:
		case 5:
			// Verifica los siguientes bytes de la cabecera ('N','E','R',':')
			if(value == HEADER[aBus->rx.header]){
				aBus->rx.cks ^= value;
				aBus->rx.header++;
			}
			else{
				// Si hay error, reinicia el estado y retrocede el índice
				aBus->rx.header = 0;
				aBus->rx.iRead--;
			}
			break;
		case 4:
			// Recibe LENGTH y lo almacena
			aBus->rx.cks ^= value;
			aBus->rx.nBytes = value;
			aBus->rx.iData = aBus->rx.iRead+2;
			aBus->rx.iData &= aBus->rx.maxIndexRingBuf;
			aBus->rx.newData = 0;
			aBus->rx.header = 5;
			break;
		case 6:
			// Recibe CMD, PAYLOAD y CHECKSUM
			aBus->rx.nBytes--;
			if(aBus->rx.nBytes)
				aBus->rx.cks ^= value;
			else{
				// Último byte: debe ser el checksum
				aBus->rx.header = 0;
				if(value == aBus->rx.cks){
					// Paquete válido: invoca el callback o setea flag
					if(aBus->MyDataReady != NULL)
						aBus->MyDataReady(aBus, aBus->rx.iData);
					else
						aBus->rx.newData = 1;
				}
			}
			break;
		default:
			// Estado inválido: reinicia
			aBus->rx.header = 0;
		}

		aBus->rx.iRead &= aBus->rx.maxIndexRingBuf;
		aBus->rx.iRead++;
		aBus->rx.iRead &= aBus->rx.maxIndexRingBuf;
	}
}

/**
 * @brief Inicializa el handler de UNERBUS, reseteando índices y flags.
 */
void UNERBUS_Init(_sUNERBUSHandle *aBus){
	aBus->rx.header = 0;
	aBus->rx.iRead = 0;
	aBus->rx.iWrite = 0;
	aBus->rx.newData = 0;
	aBus->tx.iRead = 0;
	aBus->tx.iWrite = 0;
	aBus->iiTXw = 6;
}

/**
 * @brief Escribe un buffer en el buffer de transmisión.
 * No envía aún, solo coloca los datos en el buffer.
 */
void UNERBUS_Write(_sUNERBUSHandle *aBus, uint8_t *buf, uint8_t lenBuf){
	for (uint8_t i=0; i<lenBuf; i++)
	{
		aBus->tx.buf[aBus->iiTXw++] = buf[i];
		aBus->iiTXw &= aBus->tx.maxIndexRingBuf;
	}
}

/**
 * @brief Escribe un byte en el buffer de transmisión.
 */
void UNERBUS_WriteByte(_sUNERBUSHandle *aBus, uint8_t value){
	aBus->tx.buf[aBus->iiTXw++] = value;
	aBus->iiTXw &= aBus->tx.maxIndexRingBuf;
}

/**
 * @brief Arma y envía un paquete completo (cabecera, payload, checksum).
 * lenCMD: longitud del payload + 1 (por el comando).
 * Calcula y agrega el checksum automáticamente.
 */
void UNERBUS_Send(_sUNERBUSHandle *aBus, uint8_t cmdID, uint8_t lenCMD){
	uint8_t i;

	i = aBus->tx.iWrite + 7;
	i &= aBus->tx.maxIndexRingBuf;
	
	if(aBus->iiTXw == i)
		return;
	
	HEADER[4] = lenCMD + 1;
	HEADER[6] = cmdID;


	aBus->tx.cks = 0;

	lenCMD += 6;
	
	for (i=0; i<lenCMD; i++)
	{
		if(i < 7)
			aBus->tx.buf[aBus->tx.iWrite] = HEADER[i];
		aBus->tx.cks ^= aBus->tx.buf[aBus->tx.iWrite];
		aBus->tx.iWrite++;
		aBus->tx.iWrite &= aBus->tx.maxIndexRingBuf;

	}

	aBus->tx.buf[aBus->tx.iWrite++] = aBus->tx.cks;	
	aBus->tx.iWrite &= aBus->tx.maxIndexRingBuf;
	aBus->iiTXw = (aBus->tx.iWrite + 7);
	aBus->iiTXw &= aBus->tx.maxIndexRingBuf;
}

/**
 * @brief Arma y envía un paquete a un buffer externo (útil para pruebas o logs).
 */
void UNERBUS_SendToBuf(_sUNERBUSHandle *aBus, uint8_t cmdID, uint8_t lenCMD, uint8_t *bufForSend){
	uint8_t i;

	i = aBus->tx.iWrite + 7;
	i &= aBus->tx.maxIndexRingBuf;
	
	if(aBus->iiTXw == i)
		return;
	
	HEADER[4] = lenCMD + 1;
	HEADER[6] = cmdID;

	aBus->tx.cks = 0;

	lenCMD += 6;
	
	for (i=0; i<lenCMD; i++)
	{
		if(i < 7)
			bufForSend[i] = HEADER[i];
		else{
			bufForSend[i] = aBus->tx.buf[aBus->tx.iWrite++];
			aBus->tx.iWrite &= aBus->tx.maxIndexRingBuf;
		}
		aBus->tx.cks ^= bufForSend[i];
	}

	bufForSend[i] = aBus->tx.cks;
	aBus->tx.iRead = aBus->tx.iWrite;
	aBus->iiTXw = (aBus->tx.iWrite + 7);
	aBus->iiTXw &= aBus->tx.maxIndexRingBuf;
	
}

/**
 * @brief Procesa un byte recibido y lo coloca en el buffer circular de recepción.
 */
void UNERBUS_ReceiveByte(_sUNERBUSHandle *aBus, uint8_t value){
	aBus->rx.buf[aBus->rx.iWrite++] = value;
	aBus->rx.iWrite &= aBus->rx.maxIndexRingBuf;
}

/**
 * @brief Procesa un buffer recibido y lo coloca en el buffer circular de recepción.
 */
void UNERBUS_ReceiveBuf(_sUNERBUSHandle *aBus, uint8_t *buf, uint8_t lenBuf){
	for (uint8_t i=0; i<lenBuf; i++)
	{
		aBus->rx.buf[aBus->rx.iWrite++] = buf[i];
		aBus->rx.iWrite &= aBus->rx.maxIndexRingBuf;
	}
}

/**
 * @brief Escribe una cadena constante en el buffer de transmisión.
 * lastString: si es 1, actualiza el índice auxiliar de transmisión.
 */
void UNERBUS_WriteConstString(_sUNERBUSHandle *aBus, const char *buf, uint8_t lastString){
	uint8_t i=0, value;
	
	#ifdef AVRGCC_ATMEGA
	value = pgm_read_byte(buf);
	#else
	value = buf[i];
	#endif 
	while(value){
		aBus->tx.buf[aBus->tx.iWrite++] = value;
		aBus->tx.iWrite &= aBus->tx.maxIndexRingBuf;
		i++;
		#ifdef AVRGCC_ATMEGA
		value = pgm_read_byte(buf+i);
		#else
		value = buf[i];
		#endif
	}
	if(lastString){
		aBus->iiTXw = (aBus->tx.iWrite + 7);
		aBus->iiTXw &= aBus->tx.maxIndexRingBuf;
	}
}

/**
 * @brief Extrae un buffer del payload recibido.
 */
void UNERBUS_GetBuf(_sUNERBUSHandle *aBus, uint8_t *buf, uint8_t lenBuf){
	for (uint8_t i=0; i<lenBuf; i++)
	{
		buf[i] =  aBus->rx.buf[aBus->rx.iData++];
		aBus->rx.iData &= aBus->rx.maxIndexRingBuf;
	}
}

/**
 * @brief Extrae un uint8_t del payload recibido.
 */
uint8_t UNERBUS_GetUInt8(_sUNERBUSHandle *aBus){
	w.u8[0] = aBus->rx.buf[aBus->rx.iData++];
	aBus->rx.iData &= aBus->rx.maxIndexRingBuf;
	
	return w.u8[0];
}

/**
 * @brief Extrae un int8_t del payload recibido.
 */
int8_t UNERBUS_GetInt8(_sUNERBUSHandle *aBus){
	w.i8[0] = aBus->rx.buf[aBus->rx.iData++];
	aBus->rx.iData &= aBus->rx.maxIndexRingBuf;

	return w.i8[0];
}

/**
 * @brief Extrae un uint32_t del payload recibido.
 */
uint32_t UNERBUS_GetUInt32(_sUNERBUSHandle *aBus){
	w.u8[0] = aBus->rx.buf[aBus->rx.iData++];
	aBus->rx.iData &= aBus->rx.maxIndexRingBuf;
	w.u8[1] = aBus->rx.buf[aBus->rx.iData++];
	aBus->rx.iData &= aBus->rx.maxIndexRingBuf;
	w.u8[2] = aBus->rx.buf[aBus->rx.iData++];
	aBus->rx.iData &= aBus->rx.maxIndexRingBuf;
	w.u8[3] = aBus->rx.buf[aBus->rx.iData++];
	aBus->rx.iData &= aBus->rx.maxIndexRingBuf;

	return w.u32;
}

/**
 * @brief Extrae un int32_t del payload recibido.
 */
int32_t UNERBUS_GetInt32(_sUNERBUSHandle *aBus){
	w.u8[0] = aBus->rx.buf[aBus->rx.iData++];
	aBus->rx.iData &= aBus->rx.maxIndexRingBuf;
	w.u8[1] = aBus->rx.buf[aBus->rx.iData++];
	aBus->rx.iData &= aBus->rx.maxIndexRingBuf;
	w.u8[2] = aBus->rx.buf[aBus->rx.iData++];
	aBus->rx.iData &= aBus->rx.maxIndexRingBuf;
	w.u8[3] = aBus->rx.buf[aBus->rx.iData++];
	aBus->rx.iData &= aBus->rx.maxIndexRingBuf;

	return w.i32;	
}

/**
 * @brief Extrae un uint16_t del payload recibido.
 */
uint16_t UNERBUS_GetUInt16(_sUNERBUSHandle *aBus){
	w.u8[0] = aBus->rx.buf[aBus->rx.iData++];
	aBus->rx.iData &= aBus->rx.maxIndexRingBuf;
	w.u8[1] = aBus->rx.buf[aBus->rx.iData++];
	aBus->rx.iData &= aBus->rx.maxIndexRingBuf;

	return w.u16[0];	
}

/**
 * @brief Extrae un int16_t del payload recibido.
 */
int16_t UNERBUS_GetInt16(_sUNERBUSHandle *aBus){
	w.u8[0] = aBus->rx.buf[aBus->rx.iData++];
	aBus->rx.iData &= aBus->rx.maxIndexRingBuf;
	w.u8[1] = aBus->rx.buf[aBus->rx.iData++];
	aBus->rx.iData &= aBus->rx.maxIndexRingBuf;

	return w.i16[0];
}

/**
 * @brief Extrae un float del payload recibido.
 */
float UNERBUS_GetFloat(_sUNERBUSHandle *aBus){
	w.u8[0] = aBus->rx.buf[aBus->rx.iData++];
	aBus->rx.iData &= aBus->rx.maxIndexRingBuf;
	w.u8[1] = aBus->rx.buf[aBus->rx.iData++];
	aBus->rx.iData &= aBus->rx.maxIndexRingBuf;
	w.u8[2] = aBus->rx.buf[aBus->rx.iData++];
	aBus->rx.iData &= aBus->rx.maxIndexRingBuf;
	w.u8[3] = aBus->rx.buf[aBus->rx.iData++];
	aBus->rx.iData &= aBus->rx.maxIndexRingBuf;

	return w.f;	
}

/**
 * @brief Obtiene el índice de lectura actual del payload.
 */
uint8_t UNERBUS_GetIndexRead(_sUNERBUSHandle *aBus){
	return aBus->rx.iData;
}

/**
 * @brief Mueve el índice de lectura del payload.
 */
void UNERBUS_MoveIndexRead(_sUNERBUSHandle *aBus, int8_t newIndexRead){
	aBus->rx.iData += newIndexRead;
	aBus->rx.iData &= aBus->rx.maxIndexRingBuf;
}

/**
 * @brief Resetea el flag de nuevo dato recibido.
 */
void UNERBUS_ResetNewData(_sUNERBUSHandle *aBus){
	aBus->rx.newData = 0;
}

/**
 * @brief Procesa tareas pendientes: decodificación de paquetes y transmisión por USART si corresponde.
 * Llama a UNERBUS_DecodeHeader si hay datos nuevos en el buffer de recepción.
 * Si hay datos pendientes en el buffer de transmisión y WriteUSARTByte está definido, los envía.
 */
void UNERBUS_Task(_sUNERBUSHandle *aBus){
	if(aBus->rx.iRead != aBus->rx.iWrite)
		UNERBUS_DecodeHeader(aBus);
		
	if(aBus->WriteUSARTByte != NULL){
		if(aBus->tx.iRead != aBus->tx.iWrite){
			if(aBus->WriteUSARTByte(aBus->tx.buf[aBus->tx.iRead])){
				aBus->tx.iRead++;
				aBus->tx.iRead &= aBus->tx.maxIndexRingBuf;
			}
		}
	}	
}

/**
 * @brief Maneja el timeout de recepción. Si expira, reinicia el estado de la cabecera.
 */
void UNERBUS_Timeout(_sUNERBUSHandle *aBus){
	if(aBus->rx.timeout){
		aBus->rx.timeout--;
		if(!aBus->rx.timeout)
			aBus->rx.header = 0;
	}
}

// Fin de archivo UNERBUS.c



