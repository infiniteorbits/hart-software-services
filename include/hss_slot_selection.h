#ifndef HSS_SLOT_SELECTION_H
#define HSS_SLOT_SELECTION_H

#ifdef __cplusplus
extern "C" {
#endif

enum memory_devices_id {
    EMMC_PRIMARY = 10,
    EMMC_SECONDARY = 20,
    SPI_FLASH = 255,
};

// Enumeration for **physical** SPI addresses
typedef enum {
    GLD_PADDR = 0x00000400,                         // 1 KB
    UPD_PADDR = 0x00A00000,                         // 10 MB
    IAP_PADDR = 0x01400000,                         // 20 MB
    PARAM_PADDR = 0x02000000,                       // 32 MB
    SPI0_PADDR = 0x03000000,                        // 48 MB
    SPI1_PADDR = 0x04000000,                        // 64 MB
    SPI2_PADDR = 0x05000000,                        // 80 MB
    SPI3_PADDR = 0x06000000,                        // 96 MB
} vmem_spi_PADDR_t;

// Enumeration for **physical** EMMC addresses
typedef enum {
    EMMC0_PADDR = 0x00000000,
    EMMC1_PADDR = 0x06400000,
    EMMC2_PADDR = 0x0C800000,
    EMMC3_PADDR = 0x12C00000,
    EMMC4_PADDR = 0x100000000, // 4GB
    EMMC5_PADDR = 0x140000000, // 5GB
    EMMC6_PADDR = 0x180000000, // 6GB
    EMMC7_PADDR = 0x1C0000000, // 7GB
} vmem_emmc_PADDR_t;

typedef enum  {
    NO_ERROR = 0,
    INVALID_BOOT_SEQUENCE,
    FAIL_INIT,
    DECOMPRESSION,
    IMAGE_NULL,
    HEADER_READ,
    MAGIC_NUMBER,
    CRC_CALCULATION,
    COPY_TO_DDR
} boot_error_codes;

void HSS_slot_get_boot_params(void);
void HSS_slot_update_boot_params(int index, boot_error_codes code);
bool validateCrc_custom_emmc(struct HSS_BootImage *pImage, size_t offset);
bool validateCrc_custom_spi(struct HSS_BootImage *pImage);
void enable_emmc(uint8_t emmc_id);
bool spi_init(void);
bool spi_read(void *pDest, size_t srcOffset, size_t byteCount);
bool spi_write(size_t dstOffset, void *pSrc, size_t byteCount);
void spi_GetInfo(uint32_t *pBlockSize, uint32_t *pEraseSize, uint32_t *pBlockCount);
void erase_section(uint32_t address);
bool get_ignore_crc(void);
uint64_t get_offset(uint8_t slot);
uint8_t get_boot_sequence(uint8_t index);
void HSS_slot_restore_boot_sequence(void);

#ifdef __cplusplus
}
#endif

#endif
