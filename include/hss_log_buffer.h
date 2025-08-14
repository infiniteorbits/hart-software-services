#ifndef LOG_BUFFER_H
#define LOG_BUFFER_H

#include <stddef.h> 

#ifdef __cplusplus
extern "C" {
#endif

void HSS_log_store(const char *msg);
const char *HSS_log_get_buffer(void);
size_t HSS_log_get_size(void);
void HSS_log_save_to_emmc(void);
void HSS_log_append(const char *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif // LOG_BUFFER_H
