
#ifndef HSS_CONFIG_H
#define HSS_CONFIG_H

#include "hss_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t linux_LastFailed;
    uint8_t linux_CurrentTry;
    uint8_t linux_current_sw;
    uint8_t freertos_LastFailed;
    uint8_t freertos_CurrentTry;
    uint8_t freertos_current_sw;
} BootSoftwareData;


#ifdef __cplusplus
}
#endif

#endif
