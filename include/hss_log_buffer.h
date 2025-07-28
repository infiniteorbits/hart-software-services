#ifndef LOG_BUFFER_H
#define LOG_BUFFER_H

#include <stddef.h> 
#include "mss_mmc.h"

#ifdef __cplusplus
extern "C" {
#endif

void log_store(const char *msg);
const char *log_get_buffer(void);
size_t log_get_size(void);
void log_save_to_emmc(void);
void log_append(const char *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif // LOG_BUFFER_H
