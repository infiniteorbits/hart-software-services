#include "hss_log_buffer.h"
#include "hss_debug.h"
#include <string.h>
#include <stdbool.h>
#include "hss_slot_selection.h"

#include "config.h"
#include "hss_types.h"
#include "hss_init.h"
#include <string.h>
#include <assert.h>

#define LOG_SAVE_BASE_BLOCK (LOG_REGION / SIZE_512B)

static char log_buffer[LOG_BUFFER_SIZE];
static size_t log_buffer_index = 0;

void log_reset(void);


void log_append(const char *data, size_t len) {
    if (log_buffer_index + len >= LOG_BUFFER_SIZE) {
        // Opcional: truncar, sobrescribir o ignorar
        return;
    }

    memcpy(&log_buffer[log_buffer_index], data, len);
    log_buffer_index += len;
}

const char *log_get_buffer(void) {
    return log_buffer;
}

size_t log_get_size(void) {
    return log_buffer_index;
}

void log_reset(void) {
    log_buffer_index = 0;
}

void log_store(const char *msg) {
    size_t len = strlen(msg);
    if (len + log_buffer_index >= LOG_BUFFER_SIZE) {
        log_buffer_index = 0; // sobreescribimos desde el inicio (buffer circular simple)
    }
    memcpy(&log_buffer[log_buffer_index], msg, len);
    log_buffer_index += len;
}

void log_save_to_emmc(void)
{
    /*static bool log_saved = false;
    if (log_saved) {
        mHSS_DEBUG_PRINTF(LOG_FUNCTION, "log_save_to_emmc(): already saved, skipping\n");
        return;
    }
    log_saved = true;*/

    const char *log = log_get_buffer();
    size_t log_size = log_get_size();

    mHSS_DEBUG_PRINTF(LOG_FUNCTION, "%s(): log size = %u bytes\n", __func__, log_size);

    uint32_t start_block = LOG_SAVE_BASE_BLOCK;
    size_t remaining = log_size;
    size_t offset = 0;
    uint8_t tx_buffer[SIZE_512B] = {0};

    int block_count = 0;

    while (remaining > 0) {
        size_t chunk_size = (remaining > SIZE_512B) ? SIZE_512B : remaining;

        memset(tx_buffer, 0, SIZE_512B);
        memcpy(tx_buffer, log + offset, chunk_size);

        int result = MSS_MMC_single_block_write((uint32_t *)tx_buffer, start_block);
        if (result != MSS_MMC_TRANSFER_SUCCESS) {
            mHSS_DEBUG_PRINTF(LOG_ERROR, "EMMC write failed at block %u (offset %u)\n", start_block, offset);
            break;
        }

        //mHSS_DEBUG_PRINTF(LOG_NORMAL, "EMMC write ok: block %u, bytes %u\n", start_block, chunk_size);

        remaining -= chunk_size;
        offset += chunk_size;
        start_block++;
        block_count++;
    }

    mHSS_DEBUG_PRINTF(LOG_FUNCTION, "log_save_to_emmc(): wrote %d blocks\n", block_count);
}