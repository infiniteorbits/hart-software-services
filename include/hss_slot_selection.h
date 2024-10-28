#ifndef HSS_SLOT_SELECTION_H
#define HSS_SLOT_SELECTION_H

#ifdef __cplusplus
extern "C" {
#endif

enum memory_devices_id {
    EMMC_PRIMARY = 1,
    EMMC_SECONDARY = 2,
    SPI_FLASH = 3,
};

// Enumeration for **physical** SPI addresses
typedef enum {
    GLD_PADDR = 0x00000400,  // Address for GLD (0 MB)
    UPD_PADDR = 0x00A00000,  // Address for UPD (10 MB)
    IAP_PADDR = 0x01400000,  // Address for IAP (20 MB)
    BOOT_PARAMS_PADDR = 0x01E3CFE0,  // Boot Params address (IAP + 10 MB)
    SPI0_PADDR = 0x10000000, // Base address for SPI0 (256 MB)
    SPI1_PADDR = 0x14000000, // Base address for SPI1 (320 MB)
    SPI2_PADDR = 0x18000000, // Base address for SPI2 (384 MB)
    SPI3_PADDR = 0x1C000000  // Base address for SPI3 (448 MB)
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

void HSS_slot_get_boot_params(void);
void HSS_slot_update_boot_params(int index);
bool validateCrc_custom_emmc(struct HSS_BootImage *pImage);
bool validateCrc_custom_spi(struct HSS_BootImage *pImage);
void enable_emmc(uint8_t emmc_id);
bool spi_init(void);
bool spi_read(void *pDest, size_t srcOffset, size_t byteCount);
bool spi_write(size_t dstOffset, void *pSrc, size_t byteCount);
void spi_GetInfo(uint32_t *pBlockSize, uint32_t *pEraseSize, uint32_t *pBlockCount);
void erase_section(uint32_t address);
bool get_ignore_crc(void);
uint64_t get_offset(uint8_t slot);
uint8_t get_boot_sequence(void);

#ifdef __cplusplus
}
#endif

#endif
