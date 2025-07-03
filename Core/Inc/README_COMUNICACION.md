# Arquitectura de Comunicación – Proyecto STM32

## 1. Descripción General
El sistema implementa una arquitectura de comunicación robusta y modular para un vehículo autónomo basado en STM32. Soporta dos canales principales:
- **USB (CDC, comunicación con PC)**
- **WiFi (ESP01, UDP)**
Ambos canales utilizan el protocolo UNERBUS para el intercambio de datos estructurados.

## 2. Protocolo UNERBUS
### Estructura de Paquete
| HEADER (4B) | LENGTH (1B) | TOKEN (1B) | CMD (1B) | PAYLOAD (N) | CHECKSUM (1B) |
|------------|-------------|------------|----------|-------------|---------------|
| 'U','N','E','R' | Cantidad de bytes a enviar (CMD+PAYLOAD+CHECKSUM) | ':' | Código de comando | Datos útiles | XOR de todos los bytes anteriores |

- **HEADER:** Identifica el inicio del paquete.
- **LENGTH:** CMD + PAYLOAD + CHECKSUM.
- **TOKEN:** Constante ':' (0x3A).
- **CMD:** Código de comando.
- **PAYLOAD:** Datos útiles.
- **CHECKSUM:** XOR de todos los bytes anteriores, incluyendo HEADER.

### Flujo de Datos
- Los datos recibidos se almacenan en buffers circulares.
- El parser de UNERBUS valida la cabecera, longitud y checksum.
- Si el paquete es válido, invoca un callback para procesar el comando.
- Las respuestas se arman y transmiten usando el mismo protocolo.

## 3. Canales de Comunicación
### USB (CDC)
- **Recepción:**
  - Los datos recibidos por USB se pasan a `UNERBUS_ReceiveBuf`.
  - El parser procesa el buffer y ejecuta el callback de comando.
- **Transmisión:**
  - Las respuestas se colocan en el buffer de transmisión y se envían por USB.

### WiFi (ESP01, UDP)
- **Recepción:**
  - El ESP01 recibe datos por UDP y los envía por UART al STM32.
  - Cada byte recibido se pasa a `UNERBUS_ReceiveByte`.
- **Transmisión:**
  - Los datos pendientes se envían al ESP01, que los transmite por UDP.

## 4. Principales Funciones y Callbacks
- `UNERBUS_Init`: Inicializa el handler y los buffers.
- `UNERBUS_ReceiveByte` / `UNERBUS_ReceiveBuf`: Procesan datos recibidos.
- `UNERBUS_Task`: Decodifica paquetes y gestiona transmisión.
- `UNERBUS_Send`: Arma y transmite un paquete.
- `MyDataReady`: Callback invocado al recibir un paquete válido.

## 5. Robustez y Manejo de Errores
- **Buffers circulares**: Evitan bloqueos y pérdida de datos.
- **Timeouts**: Si la cabecera no se completa en tiempo, se reinicia el estado.
- **Checksum**: Garantiza la integridad de los paquetes.
- **Callbacks**: Permiten desacoplar la lógica de recepción y procesamiento de comandos.

## 6. Ejemplo de Envío de Mensaje
Supongamos que se quiere enviar el mensaje "Juan hoy comió bien" con CMD=0x01:

```c
const char mensaje[] = "Juan hoy comió bien";
uint8_t cmd = 0x01;
UNERBUS_WriteByte(&unerbusPC, cmd);
UNERBUS_Write(&unerbusPC, (uint8_t*)mensaje, sizeof(mensaje) - 1);
UNERBUS_Send(&unerbusPC, cmd, sizeof(mensaje));
```

## 6b. Ejemplo de Recepción y Procesamiento de un Nuevo Comando

Supongamos que se define un nuevo comando CMD=0x10 para recibir un mensaje de texto y procesarlo:

```c
#define CMD_MENSAJE_TEXTO 0x10

// Callback de recepción de comandos UNERBUS
void DecodeCMD(struct UNERBUSHandle *aBus, uint8_t iStartData) {
    uint8_t id = UNERBUS_GetUInt8(aBus); // Extrae el CMD
    switch(id) {
        case CMD_MENSAJE_TEXTO: {
            char buffer[32];
            // Extrae el payload recibido (por ejemplo, 20 bytes)
            UNERBUS_GetBuf(aBus, (uint8_t*)buffer, 20);
            buffer[20] = '\0'; // Asegura fin de cadena
            // Procesa el mensaje recibido
            ProcesarMensaje(buffer);
            break;
        }
        // ... otros comandos ...
    }
}
```

- El callback `DecodeCMD` es invocado automáticamente al recibir un paquete válido.
- Se extrae el comando y el payload usando las funciones de UNERBUS.
- Se recomienda validar la longitud del payload según el comando esperado.

## 7. Extensibilidad y Recomendaciones
- Para agregar nuevos comandos, definir nuevos CMD y su lógica en el callback.
- Para nuevos canales, reutilizar la lógica de UNERBUS y adaptar el transporte físico.
- Para depuración, agregar logs en los callbacks y verificar el flujo de paquetes.

## 8. Referencias
- Ver archivos `UNERBUS.h` y `UNERBUS.c` para detalles de implementación.
- Ver integración con ESP01 en `ESP01.h` y `ESP01.c` para la capa WiFi.

# Configuración recomendada en .ioc para MPU6050 (I2C + DMA)

1. **Habilitar I2C2** (o el I2C que vayas a usar):
   - Modo: I2C
   - Velocidad sugerida: 400 kHz (Fast Mode)
   - Pines: Configura SCL y SDA como Alternate Function Open Drain

2. **Habilitar DMA para I2C RX**:
   - Asigna un canal DMA a I2C2_RX (por ejemplo, DMA1 Channel 5 para STM32F1xx)
   - Modo: Normal
   - Prioridad: Alta o Media

3. **No es necesario habilitar interrupciones de I2C para DMA**

4. **Conexión física**:
   - Asegúrate de tener resistencias pull-up externas (2.2kΩ–4.7kΩ) en SCL y SDA

5. **Guardar y regenerar código** antes de compilar

Con esto, el driver MPU6050 funcionará correctamente con lecturas no bloqueantes por DMA. 