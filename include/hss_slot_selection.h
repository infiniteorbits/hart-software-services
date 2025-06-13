#ifndef HSS_SLOT_SELECTION_H
#define HSS_SLOT_SELECTION_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    EMMC_PRIMARY = 10,
    EMMC_SECONDARY = 20,
    SPI_FLASH = 30,
    QSPI = 40,
} memory_type_t;

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

// Enumeration for eMMC **physical** addresses
typedef enum {
    LOG_REGION      = 0x180000000,        // 6 GiB
    PARAM_REGION    = 0x1A0000000,        // 6.5 GiB
    PAYLOAD_1       = 71303168,           //  71.3 MB
    PAYLOAD_2       = 0x1C0000000         // 7 GiB
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
bool validateMd5_custom(struct HSS_BootImage *pImage, size_t offset, memory_type_t mem_type);
void enable_emmc(uint8_t emmc_id);
bool spi_init(void);
bool spi_read(void *pDest, size_t srcOffset, size_t byteCount);
bool spi_write(size_t dstOffset, void *pSrc, size_t byteCount);
void spi_GetInfo(uint32_t *pBlockSize, uint32_t *pEraseSize, uint32_t *pBlockCount);
void spi_erase_section(uint32_t address);
bool get_verify_payload(void);
uint64_t get_offset(uint8_t slot);
uint8_t get_boot_sequence(uint8_t index);
void HSS_slot_restore_boot_sequence(void);

#ifdef __cplusplus
}
#endif

#endif
