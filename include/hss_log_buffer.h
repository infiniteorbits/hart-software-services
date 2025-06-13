#ifndef LOG_BUFFER_H
#define LOG_BUFFER_H

#include <stddef.h>  // para size_t
#include "mss_mmc.h"

#ifdef __cplusplus
extern "C" {
#endif

/// Tamaño total del buffer de logs (definido en .c)
#define LOG_BUFFER_SIZE 8192  // Tamaño total del log
#define LOG_LINE_MAX_LEN 256  // Máximo por línea
#define  ADDRESS_6GB  0x180000000
#define SIZE_512B      512

/// Agrega una cadena al buffer circular de logs.
/// No agrega newline al final. Sobrescribe si se llena.
void log_store(const char *msg);

/// Devuelve un puntero al buffer con los logs acumulados.
const char *log_get_buffer(void);

/// Devuelve el tamaño actual (en bytes) del contenido del log.
size_t log_get_size(void);

void log_save_to_emmc(void);
void log_append(const char *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif // LOG_BUFFER_H
