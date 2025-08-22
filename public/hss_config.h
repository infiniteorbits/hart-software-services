
#ifndef HSS_CONFIG_H
#define HSS_CONFIG_H

#include "hss_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t linux_boot_sequence;
    uint8_t linux_verify_payload;
    uint8_t freertos_boot_sequence;
    uint8_t freertos_verify_payload;
} BootSoftwareParams;


#ifdef __cplusplus
}
#endif

#endif
