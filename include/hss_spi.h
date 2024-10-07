/*
 * hss_spi.h
 *
 *  Created on: Sep 10, 2024
 *      Author: root
 */

#ifndef HSS_SPI_H_
#define HSS_SPI_H_

#include "hss_types.h"

// Enumeration for **physical** addresses
typedef enum {
    GLD_PADDR = 0x00000400,  // Address for GLD (0 MB)
    UPD_PADDR = 0x00A00000,  // Address for UPD (10 MB)
    IAP_PADDR = 0x0143CFE0,  // Address for IAP (20 MB)
    SPI0_PADDR = 0x10000000, // Base address for SPI0 (256 MB)
    SPI1_PADDR = 0x14000000, // Base address for SPI1 (320 MB)
    SPI2_PADDR = 0x18000000, // Base address for SPI2 (384 MB)
    SPI3_PADDR = 0x1C000000  // Base address for SPI3 (448 MB)
} vmem_spi_PADDR_t;


bool spi_init(void);
bool spi_read(void *pDest, size_t srcOffset, size_t byteCount);
bool spi_write(size_t dstOffset, void *pSrc, size_t byteCount);
void spi_GetInfo(uint32_t *pBlockSize, uint32_t *pEraseSize, uint32_t *pBlockCount);
void erase_section(uint32_t address);

#endif /* HSS_SPI_H_ */
