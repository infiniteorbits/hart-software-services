/*******************************************************************************
 * Copyright 2019-2022 Microchip FPGA Embedded Systems Solutions.
 *
 * SPDX-License-Identifier: MIT
 *
 * MPFS HSS Embedded Software
 *
 */

/**
 * \file HSS Boot Initalization
 * \brief Boot Initialization
 */

#include "config.h"
#include "hss_types.h"

#include "hss_init.h"
#include "hss_boot_service.h"
#include "hss_boot_init.h"
#include "hss_sys_setup.h"
#include "hss_progress.h"
#include "hss_trigger.h"
#include "u54_state.h"
#include "mss_sysreg.h"
#include "hss_slot_selection.h"
#include "micron1gflash.h"

#if IS_ENABLED(CONFIG_SERVICE_SPI)
#  include <mss_sys_services.h>
#  define SPI_FLASH_BOOT_ENABLED 1 //(CONFIG_SERVICE_BOOT_SPI_FLASH_OFFSET != 0xFFFFFFFF)
#else
#  define SPI_FLASH_BOOT_ENABLED 0
#endif

#if IS_ENABLED(CONFIG_SERVICE_OPENSBI)
#  include "opensbi_service.h"
#endif

#if IS_ENABLED(CONFIG_SERVICE_QSPI)
#  include "qspi_service.h"
#endif

#if IS_ENABLED(CONFIG_SERVICE_MMC)
#  include "mmc_service.h"
#  include "gpt.h"
#endif

#if (SPI_FLASH_BOOT_ENABLED)
#  include "mss_sys_services.h"
#endif

#include "hss_state_machine.h"
#include "hss_debug.h"
#include "hss_perfctr.h"

#include <string.h>
#include <assert.h>

#if IS_ENABLED(CONFIG_COMPRESSION)
#  include "hss_decompress.h"
#endif

#if IS_ENABLED(CONFIG_CRYPTO_SIGNING)
#  include "hss_boot_secure.h"
#endif

#include "hss_boot_pmp.h"
#include "hss_atomic.h"

#include "sbi_bitops.h"

//
// local module functions

#if IS_ENABLED(CONFIG_SERVICE_BOOT)
typedef bool (*HSS_BootImageCopyFnPtr_t)(void *pDest, size_t srcOffset, size_t byteCount);
static bool copyBootImageToDDR_(struct HSS_BootImage *pBootImage, char *pDest,
    size_t srcOffset, HSS_BootImageCopyFnPtr_t pCopyFunction);

static void printBootImageDetails_(struct HSS_BootImage const * const pBootImage);
static bool tryBootFunction_(struct HSS_Storage *pStorage, HSS_GetBootImageFnPtr_t getBootImageFunction);
#endif

static bool getBootImageFromQSPI_(struct HSS_Storage *pStorage, struct HSS_BootImage **ppBootImage);
static bool getBootImageFromMMC_(struct HSS_Storage *pStorage, struct HSS_BootImage **ppBootImage);
static bool getBootImageFromSpiFlash_(struct HSS_Storage *pStorage, struct HSS_BootImage **ppBootImage);
static bool getBootImageFromPayload_(struct HSS_Storage *pStorage, struct HSS_BootImage **ppBootImage);


static uint8_t index_boot_image = 0;

//
//

#if IS_ENABLED(CONFIG_SERVICE_QSPI)
static struct HSS_Storage qspiStorage_ = {
    .name = "QSPI",
    .getBootImage = getBootImageFromQSPI_,
    .init = HSS_CachedQSPIInit,
    .readBlock = HSS_CachedQSPI_ReadBlock,
    .writeBlock = HSS_CachedQSPI_WriteBlock,
    .getInfo = HSS_CachedQSPI_GetInfo,
    .flushWriteBuffer = HSS_CachedQSPI_FlushWriteBuffer
};
#endif
#if IS_ENABLED(CONFIG_SERVICE_MMC)
static struct HSS_Storage mmc1Storage_ = {
    .name = "MMC1",
    .getBootImage = getBootImageFromMMC_,
    .init = HSS_MMCInit,
    .readBlock = HSS_MMC_ReadBlock,
    .writeBlock = HSS_MMC_WriteBlockSDMA,
    .getInfo = HSS_MMC_GetInfo,
    .flushWriteBuffer = NULL
};
#endif
#if IS_ENABLED(CONFIG_SERVICE_MMC)
static struct HSS_Storage mmc2Storage_ = {
    .name = "MMC2",
    .getBootImage = getBootImageFromMMC_,
    .init = HSS_MMCInit,
    .readBlock = HSS_MMC_ReadBlock,
    .writeBlock = HSS_MMC_WriteBlockSDMA,
    .getInfo = HSS_MMC_GetInfo,
    .flushWriteBuffer = NULL
};
#endif
#if IS_ENABLED(CONFIG_SERVICE_SPI)
static struct HSS_Storage spiStorage_ = {
    .name = "SPI",
    .getBootImage = getBootImageFromSpiFlash_,
    .init = spi_init,
    .readBlock = spi_read,
    .writeBlock = spi_write,
    .getInfo = spi_GetInfo,
    .flushWriteBuffer = NULL
};
#endif
#if IS_ENABLED(CONFIG_SERVICE_BOOT_USE_PAYLOAD)
static struct HSS_Storage payloadStorage_ = {
    .name = "Payload",
    .getBootImage = getBootImageFromPayload_,
    .init = NULL,
    .readBlock = NULL,
    .writeBlock = NULL,
    .getInfo = NULL,
    .flushWriteBuffer = NULL
};
#endif

static struct HSS_Storage *pStorages[] =
{
#if IS_ENABLED(CONFIG_SERVICE_QSPI)
	&qspiStorage_,
#endif
#if IS_ENABLED(CONFIG_SERVICE_MMC)
	&mmc1Storage_,
#endif
#if IS_ENABLED(CONFIG_SERVICE_MMC)
	&mmc2Storage_,
#endif
#if IS_ENABLED(CONFIG_SERVICE_SPI)
	&spiStorage_,
#endif

#if IS_ENABLED(CONFIG_SERVICE_BOOT_USE_PAYLOAD)
	&payloadStorage_,
#endif
};

static struct HSS_Storage *pDefaultStorage = NULL;

#if IS_ENABLED(CONFIG_SERVICE_MMC) || IS_ENABLED(CONFIG_SERVICE_QSPI) || (IS_ENABLED(CONFIG_SERVICE_SPI) && (SPI_FLASH_BOOT_ENABLED))
struct HSS_BootImage bootImage __attribute__((aligned(8)));
#elif IS_ENABLED(CONFIG_SERVICE_BOOT_USE_PAYLOAD)
//
#else
#    error Unable to determine boot mechanism
#endif

struct HSS_Storage *HSS_BootGetActiveStorage(void);
struct HSS_Storage *HSS_BootGetActiveStorage(void)
{
    struct HSS_Storage *pResult = pDefaultStorage;

    if (!pResult) {
        pResult = pStorages[0];
    }

    return pResult;
}

void HSS_BootListStorageProviders(void)
{
    for (uint32_t i = 0; i < ARRAY_SIZE(pStorages); i++) {
        mHSS_DEBUG_PRINTF_EX(" - %s\n", pStorages[i]->name);
    }
}

void HSS_BootHarts(void)
{
#if IS_ENABLED(CONFIG_SERVICE_BOOT)
        union HSSHartBitmask restartHartBitmask = { .uint = 0u };

        for (int i = HSS_HART_U54_1; i < HSS_HART_NUM_PEERS; i++) {
            //mHSS_DEBUG_PRINTF(LOG_ERROR, "%s(): checking u54_%d\n", __func__, i);
            if (HSS_U54_GetState_Ex(i) == HSS_State_Idle) {
                //mHSS_DEBUG_PRINTF(LOG_ERROR, "%s(): => rebooting u54_%d\n", __func__, i);
                restartHartBitmask.uint |= BIT(i);
            }
        }

        if (restartHartBitmask.uint) {
            HSS_Boot_RestartCores_Using_Bitmask(restartHartBitmask);
        }
#endif
}

#include "../baremetal/polarfire-soc-bare-metal-library/src/platform/mpfs_hal/common/mss_peripherals.h"
#include "../baremetal/polarfire-soc-bare-metal-library/src/platform/drivers/mss/mss_gpio/mss_gpio.h"
int compare_strings(const char *str1, const char *str2);

int compare_strings(const char *str1, const char *str2) {
    while (*str1 && (*str1 == *str2)) {
        str1++;
        str2++;
    }
    return (*str1 == '\0' && *str2 == '\0');
}
bool tryBootFromStorage(int storageIndex, const char* message, int emmcType);
bool tryBootFromStorage(int storageIndex, const char* message, int emmcType) {
    mHSS_DEBUG_PRINTF(LOG_NORMAL, "Trying to get boot %s image via %s ...\n", message, pStorages[storageIndex]->name);
    enable_emmc(emmcType);

    bool result = false;
    if (pStorages[storageIndex]->init) {
        HSS_slot_update_boot_params(index_boot_image, NO_ERROR);
        result = pStorages[storageIndex]->init();
    } else {
        HSS_slot_update_boot_params(index_boot_image, FAIL_INIT);
    }

    if (result) {
        result = tryBootFunction_(pStorages[storageIndex], pStorages[storageIndex]->getBootImage);
    }
    else{
        HSS_slot_update_boot_params(index_boot_image, FAIL_INIT);
    }

    return result;
}

bool HSS_BootInit(void)
{
    (void)mss_config_clk_rst(MSS_PERIPH_GPIO1, (uint8_t) 1, PERIPHERAL_ON);
    MSS_GPIO_init(GPIO1_LO);
    MSS_GPIO_config(GPIO1_LO, MSS_GPIO_7, MSS_GPIO_OUTPUT_MODE);
    MSS_GPIO_set_output(GPIO1_LO, MSS_GPIO_7, 0u); //CAN0_EN
    MSS_GPIO_config(GPIO1_LO, MSS_GPIO_13, MSS_GPIO_OUTPUT_MODE);
    MSS_GPIO_set_output(GPIO1_LO, MSS_GPIO_13, 0u); //CAN1_EN
    mHSS_DEBUG_PRINTF(LOG_NORMAL,"GPIO init\n");

    //PLIC_init();
    //(void)mss_config_clk_rst(MSS_PERIPH_I2C1, (uint8_t) 1, PERIPHERAL_ON);
    //MSS_I2C_init(&g_mss_i2c1_lo, 0x21, MSS_I2C_PCLK_DIV_192);
    //MSS_I2C_register_transfer_completion_handler(I2C_MASTER, i2c1_completion_handler);
    //mHSS_DEBUG_PRINTF(LOG_NORMAL,"\n \r I2C init");
    /*mss_i2c_status_t instance;
    uint8_t g_master_tx_buf[32];
    g_master_tx_buf[0] = 0x0;
    instance = do_write_transaction(0x21, g_master_tx_buf, 1u);
    mHSS_DEBUG_PRINTF(LOG_NORMAL,"\n \r I2C status ) %d", instance);*/

    bool result = true;
    bool skip_boot_0 = false;
#if IS_ENABLED(CONFIG_SERVICE_BOOT)

    mHSS_DEBUG_PRINTF(LOG_NORMAL, "Initializing Boot Image ...\n");

    int perf_ctr_index = PERF_CTR_UNINITIALIZED;
    HSS_PerfCtr_Allocate(&perf_ctr_index, "Boot Image Init");

    HSS_slot_get_boot_params();

    if (pDefaultStorage) {
        if (pDefaultStorage->init) { result = pDefaultStorage->init(); }
        if (result) {
            result = tryBootFunction_(pDefaultStorage, pDefaultStorage->getBootImage);
        }
    } else {

        uint8_t bootSeq = get_boot_sequence(0);

        if (bootSeq == 0) {
            skip_boot_0 = true;
        } else if (bootSeq >= 10 && bootSeq <= 13) {
            result = tryBootFromStorage(0, "primary", EMMC_PRIMARY);
        } else if (bootSeq >= 20 && bootSeq <= 23) {
            result = tryBootFromStorage(1, "secondary", EMMC_SECONDARY);
        } else if (bootSeq >= 30u) {
            result = tryBootFromStorage(2, "spi", 0);
        } else {
            mHSS_DEBUG_PRINTF(LOG_ERROR, "Invalid boot sequence...\n");
            HSS_slot_update_boot_params(index_boot_image, INVALID_BOOT_SEQUENCE);
            skip_boot_0 = true;
        }
        if(!result || !skip_boot_0)
        {
            HSS_slot_update_boot_params(index_boot_image, INVALID_BOOT_SEQUENCE);
        }

        if(!result || skip_boot_0)
        {
            for (int i = 0; i < ARRAY_SIZE(pStorages); i++) {
                mHSS_DEBUG_PRINTF(LOG_NORMAL, "Trying to get boot image via %s ...\n", pStorages[i]->name);
                index_boot_image = i + 1;
                if (pStorages[i]->init) {
                    HSS_slot_update_boot_params(index_boot_image, NO_ERROR);
                    if (compare_strings(pStorages[i]->name, "MMC1")) {
                        enable_emmc(EMMC_PRIMARY);
                    }else if (compare_strings(pStorages[i]->name, "MMC2")) {
                        enable_emmc(EMMC_SECONDARY);
                    }
                    result = pStorages[i]->init();
                } else {
                    result = true;
                }

                if (result) {
                    
                    result = tryBootFunction_(pStorages[i], pStorages[i]->getBootImage);
                    if (result) { break; }
                } else{
                    mHSS_DEBUG_PRINTF(LOG_ERROR, "Fail init\n", pStorages[i]->name);
                    HSS_slot_update_boot_params(index_boot_image, FAIL_INIT);
                }
                HSS_SpinDelay_Secs(1);
            }
        }
    }

    HSS_PerfCtr_Lap(perf_ctr_index);
#endif

    return result;
}

#if IS_ENABLED(CONFIG_SERVICE_BOOT)
bool tryBootFunction_(struct HSS_Storage *pStorage, HSS_GetBootImageFnPtr_t const bootImageFunction)
{
    bool result = false;
    (void)pStorage;

    struct HSS_BootImage *pBootImage = NULL;
    bool decompressedFlag = false;

    (void)decompressedFlag;


    result = bootImageFunction(pStorage, &pBootImage);
    //
    // check if this image is compressed...
    // if so, decompress it to DDR
    //
    // for now, compression only works with a source already in DDR
#  if IS_ENABLED(CONFIG_COMPRESSION)
    if (result && pBootImage && (pBootImage->magic == mHSS_COMPRESSED_MAGIC)) {
        decompressedFlag = true;

        mHSS_DEBUG_PRINTF(LOG_NORMAL, "Preparing to decompress to DDR ...\n");
        void* const pInput = (void*)pBootImage;
        void * const pOutputInDDR = (void *)(CONFIG_SERVICE_BOOT_DDR_TARGET_ADDR);

        int outputSize = HSS_Decompress(pInput, pOutputInDDR);
        mHSS_DEBUG_PRINTF(LOG_NORMAL, "decompressed %d bytes ...\n", outputSize);

        if (outputSize) {
            pBootImage = (struct HSS_BootImage *)pOutputInDDR;
        } else {
            pBootImage = NULL;
        }
    } else if (!result) {
        mHSS_DEBUG_PRINTF(LOG_ERROR, "Failed to get boot image, cannot decompress\n");
        result = false;
    } else if (!pBootImage) {
        mHSS_DEBUG_PRINTF(LOG_ERROR, "Boot Image NULL, ignoring\n");
        HSS_slot_update_boot_params(index_boot_image, IMAGE_NULL);
        result = false;
    }
#  endif

    if (result) {
        HSS_Register_Boot_Image(pBootImage);
        mHSS_DEBUG_PRINTF(LOG_NORMAL, "%s: Boot Image registered ...\n", pStorage->name);
    } else {
        HSS_Register_Boot_Image(NULL);
    }
    HSS_slot_restore_boot_sequence();

    return result;
}

/////////////////////////////////////////////////////////////////////////////////////////

static void printBootImageDetails_(struct HSS_BootImage const * const pBootImage)
{
#  ifdef BOOT_DEBUG
    mHSS_DEBUG_PRINTF(LOG_NORMAL, " - set name is >>%s<<\n", pBootImage->set_name);
    mHSS_DEBUG_PRINTF(LOG_NORMAL, " - magic is    %08x\n", pBootImage->magic);
    mHSS_DEBUG_PRINTF(LOG_NORMAL, " - length is   %08x\n", pBootImage->bootImageLength);
#  endif
}
#endif

#if IS_ENABLED(CONFIG_SERVICE_BOOT)
static bool copyBootImageToDDR_(struct HSS_BootImage *pBootImage, char *pDest,
    size_t srcOffset, HSS_BootImageCopyFnPtr_t pCopyFunction)
{
    bool result = true;

    printBootImageDetails_(pBootImage);

    mHSS_DEBUG_PRINTF(LOG_NORMAL, "Copying %lu bytes to 0x%lx\n",
        pBootImage->bootImageLength, pDest);
    result = pCopyFunction(pDest, srcOffset, pBootImage->bootImageLength);

    return result;
}
#endif


static bool getBootImageFromMMC_(struct HSS_Storage *pStorage, struct HSS_BootImage **ppBootImage)
{
    bool result = false;

#if IS_ENABLED(CONFIG_SERVICE_BOOT) && IS_ENABLED(CONFIG_SERVICE_MMC)
    assert(ppBootImage);

    size_t srcLBAOffset = 0u;
    assert(pStorage);

    uint32_t blockSize, eraseSize, blockCount;
    pStorage->getInfo(&blockSize, &eraseSize, &blockCount);

    srcLBAOffset = get_offset(get_boot_sequence(index_boot_image));

    mHSS_DEBUG_PRINTF(LOG_NORMAL, "Attempting to copy from EMMC 0x%lx to DDR ...\n", srcLBAOffset);  
    result = HSS_MMC_ReadBlock(&bootImage, srcLBAOffset, sizeof(struct HSS_BootImage));

    if (!result) {
        mHSS_DEBUG_PRINTF(LOG_ERROR, "HSS_MMC_ReadBlock() failed\n");
        HSS_slot_update_boot_params(index_boot_image, HEADER_READ);
    } else {
       result = HSS_Boot_VerifyMagic(&bootImage);
        if (!result) {
           mHSS_DEBUG_PRINTF(LOG_ERROR, "Boot very magic failed \n");
           HSS_slot_update_boot_params(index_boot_image, MAGIC_NUMBER);
        }else {
           mHSS_DEBUG_PRINTF(LOG_NORMAL, "Boot very magic passed \n");

            if(get_ignore_crc()){
                result = true;
            }else{
                result = validateCrc_custom_emmc(&bootImage, srcLBAOffset);
            }

            if (result) {
                int perf_ctr_index = PERF_CTR_UNINITIALIZED;
                HSS_PerfCtr_Allocate(&perf_ctr_index, "Boot Image MMC Copy");

                result = copyBootImageToDDR_(&bootImage,
                    (char *)(CONFIG_SERVICE_BOOT_DDR_TARGET_ADDR), srcLBAOffset,
                    HSS_MMC_ReadBlock);
                *ppBootImage = (struct HSS_BootImage *)(CONFIG_SERVICE_BOOT_DDR_TARGET_ADDR);

                HSS_PerfCtr_Lap(perf_ctr_index);

                if (!result) {
                    mHSS_DEBUG_PRINTF(LOG_ERROR, "copyBootImageToDDR_() failed\n");
                    HSS_slot_update_boot_params(index_boot_image, COPY_TO_DDR);
                }
            }else{
                HSS_slot_update_boot_params(index_boot_image, CRC_CALCULATION);
            }
       }
    }
#endif
    return result;
}

void HSS_BootSelectSDCARD(void)
{
#if IS_ENABLED(CONFIG_SERVICE_MMC)
    mHSS_DEBUG_PRINTF(LOG_NORMAL, "Selecting SDCARD as boot source ...\n");
    pDefaultStorage = &mmc1Storage_;
    HSS_MMC_SelectSDCARD();
    HSS_Register_Boot_Image(NULL);
#else
    (void)getBootImageFromMMC_;
#endif
}

void HSS_BootSelectMMC(void)
{
#if IS_ENABLED(CONFIG_SERVICE_MMC)
    mHSS_DEBUG_PRINTF(LOG_NORMAL, "Selecting SDCARD/MMC (fallback) as boot source ...\n");
    pDefaultStorage = &mmc1Storage_;
    HSS_MMC_SelectMMC();
    HSS_Register_Boot_Image(NULL);
#else
    (void)getBootImageFromMMC_;
#endif
}

void HSS_BootSelectEMMC(void)
{
#if IS_ENABLED(CONFIG_SERVICE_MMC)
    mHSS_DEBUG_PRINTF(LOG_NORMAL, "Selecting EMMC as boot source ...\n");
    pDefaultStorage = &mmc1Storage_;
    HSS_MMC_SelectEMMC();
    HSS_Register_Boot_Image(NULL);
#else
    (void)getBootImageFromMMC_;
#endif
}

static bool getBootImageFromQSPI_(struct HSS_Storage *pStorage, struct HSS_BootImage **ppBootImage)
{
    bool result = false;

#if IS_ENABLED(CONFIG_SERVICE_BOOT) && IS_ENABLED(CONFIG_SERVICE_QSPI)
    assert(ppBootImage);

    // need to do an initial copy of the boot header into our structure, for subsequent use
    mHSS_DEBUG_PRINTF(LOG_NORMAL, "Preparing to copy from QSPI to DDR ...\n");

    size_t srcLBAOffset = 0u;
    assert(pStorage);

    uint32_t blockSize, eraseSize, blockCount;
    pStorage->getInfo(&blockSize, &eraseSize, &blockCount);

    mHSS_DEBUG_PRINTF(LOG_NORMAL, "Attempting to read image header (%d bytes) ...\n",
        sizeof(struct HSS_BootImage));
    result = HSS_QSPI_ReadBlock(&bootImage, srcLBAOffset * blockSize,
        sizeof(struct HSS_BootImage));
    if (!result) {
        mHSS_DEBUG_PRINTF(LOG_ERROR, "HSS_QSPI_ReadBlock() failed\n");
    } else {
        result = HSS_Boot_VerifyMagic(&bootImage);

        if (!result) {
            mHSS_DEBUG_PRINTF(LOG_ERROR, "HSS_Boot_VerifyMagic() failed\n");
        } else {
            int perf_ctr_index = PERF_CTR_UNINITIALIZED;
            HSS_PerfCtr_Allocate(&perf_ctr_index, "Boot Image QSPI Copy");

            result = copyBootImageToDDR_(&bootImage,
                (char *)(CONFIG_SERVICE_BOOT_DDR_TARGET_ADDR), srcLBAOffset * blockSize,
                HSS_QSPI_ReadBlock);
            *ppBootImage = (struct HSS_BootImage *)(CONFIG_SERVICE_BOOT_DDR_TARGET_ADDR);

            HSS_PerfCtr_Lap(perf_ctr_index);

            if (!result) {
                 mHSS_DEBUG_PRINTF(LOG_ERROR, "copyBootImageToDDR_() failed\n");
            }
        }
    }
#endif

    return result;
}

void HSS_BootSelectQSPI(void)
{
#if IS_ENABLED(CONFIG_SERVICE_QSPI)
    mHSS_DEBUG_PRINTF(LOG_NORMAL, "Selecting QSPI as boot source ...\n");
    pDefaultStorage = &qspiStorage_;
    HSS_Register_Boot_Image(NULL);
#else
    (void)getBootImageFromQSPI_;
#endif
}

static bool getBootImageFromPayload_(struct HSS_Storage *pStorage, struct HSS_BootImage **ppBootImage)
{
    bool result = false;
    (void)pStorage;

#if IS_ENABLED(CONFIG_SERVICE_BOOT) && IS_ENABLED(CONFIG_SERVICE_BOOT_USE_PAYLOAD)
    assert(ppBootImage);

#if IS_ENABLED(CONFIG_SERVICE_BOOT_USE_PAYLOAD_IN_FABRIC)
    *ppBootImage = (struct HSS_BootImage *)(CONFIG_SERVICE_BOOT_USE_PAYLOAD_IN_FABRIC_ADDRESS);
#else
    extern struct HSS_BootImage _payload_start;
    *ppBootImage = (struct HSS_BootImage *)&_payload_start;
#endif

    result = HSS_Boot_VerifyMagic(*ppBootImage);
    printBootImageDetails_(*ppBootImage);
#endif

    return result;
}

void HSS_BootSelectPayload(void)
{
#if IS_ENABLED(CONFIG_SERVICE_BOOT_USE_PAYLOAD)
    mHSS_DEBUG_PRINTF(LOG_NORMAL, "Selecting Payload as boot source ...\n");
    pDefaultStorage = &payloadStorage_;
    HSS_Register_Boot_Image(NULL);
#else
    (void)getBootImageFromPayload_;
#endif
}

#if 0
#if IS_ENABLED(CONFIG_SERVICE_BOOT) && IS_ENABLED(CONFIG_SERVICE_SPI)
static bool spiFlashReadBlock_(void *dst, size_t offs, size_t count) {
   int retval = MSS_SYS_spi_copy((uintptr_t)dst, offs, count, /* options */ 3, /* mb_offset */ 0);

   if (retval) {
        mHSS_DEBUG_PRINTF(LOG_ERROR, "Failed to read 0x%lx bytes from SPI flash @0x%lx (error code %d)!\n", count, offs, retval);
   }

   return (retval == 0);
}
#endif
#endif

static bool getBootImageFromSpiFlash_(struct HSS_Storage *pStorage, struct HSS_BootImage **ppBootImage) {
    bool result = false;
    (void)pStorage;

#if IS_ENABLED(CONFIG_SERVICE_BOOT) && IS_ENABLED(CONFIG_SERVICE_SPI)
    assert(ppBootImage);

   size_t srcOffset = SPI0_PADDR;
   mHSS_DEBUG_PRINTF(LOG_NORMAL, "Attempting to copy from SPI Flash 0x%lx to DDR ...\n", srcOffset);
   spi_read(&bootImage, srcOffset, sizeof(struct HSS_BootImage));
   result = HSS_Boot_VerifyMagic(&bootImage);
    if (!result) {
        mHSS_DEBUG_PRINTF(LOG_ERROR, "Boot very magic failed \n");
        HSS_slot_update_boot_params(index_boot_image, MAGIC_NUMBER);
    }else {
        mHSS_DEBUG_PRINTF(LOG_NORMAL, "Boot very magic passed \n");

        if(get_ignore_crc()){
            result = true;
        }else{
            result = validateCrc_custom_spi(&bootImage);
        }

        if (result) {
            mHSS_DEBUG_PRINTF(LOG_STATUS, "Boot image passed CRC 0x%X\n", bootImage.headerCrc);
            result = copyBootImageToDDR_(&bootImage, (char *)(CONFIG_SERVICE_BOOT_DDR_TARGET_ADDR),
                srcOffset, spi_read);
            *ppBootImage = (struct HSS_BootImage *)(CONFIG_SERVICE_BOOT_DDR_TARGET_ADDR);
    
            if (!result) {
                mHSS_DEBUG_PRINTF(LOG_ERROR, "copyBootImageToDDR_() failed\n");
                HSS_slot_update_boot_params(index_boot_image, COPY_TO_DDR);
            }
        }else{
            HSS_slot_update_boot_params(index_boot_image, CRC_CALCULATION);
        }
   }
#endif
    return result;
}

void HSS_BootSelectSPI(void)
{
#if IS_ENABLED(CONFIG_SERVICE_SPI)
    mHSS_DEBUG_PRINTF(LOG_NORMAL, "Selecting SPI Flash as boot source ...\n");
    pDefaultStorage = &spiStorage_;
    HSS_Register_Boot_Image(NULL);
#else
    (void)getBootImageFromSpiFlash_;
#endif
}

bool HSS_Storage_Init(void);
bool HSS_Storage_ReadBlock(void *pDest, size_t srcOffset, size_t byteCount);
bool HSS_Storage_WriteBlock(size_t dstOffset, void *pSrc, size_t byteCount);
void HSS_Storage_GetInfo(uint32_t *pBlockSize, uint32_t *pEraseSize, uint32_t *pBlockCount);
void HSS_Storage_FlushWriteBuffer(void);

bool HSS_Storage_Init(void)
{
    bool result = true;

    struct HSS_Storage *pStorage = pDefaultStorage ? pDefaultStorage : pStorages[0];
    assert(pStorage);

    if (pStorage->init) {
        mHSS_DEBUG_PRINTF(LOG_NORMAL, "initialize %s\n", pStorage->name);
        result = pStorage->init();
    }

    return result;
}

bool HSS_Storage_ReadBlock(void *pDest, size_t srcOffset, size_t byteCount)
{
    bool result = true;

    struct HSS_Storage *pStorage = pDefaultStorage ? pDefaultStorage : pStorages[0];
    assert(pStorage);

    if (pStorage->readBlock) {
        result = pStorage->readBlock(pDest, srcOffset, byteCount);
    }
    return result;
}

bool HSS_Storage_WriteBlock(size_t dstOffset, void *pSrc, size_t byteCount)
{
    struct HSS_Storage *pStorage = pDefaultStorage ? pDefaultStorage : pStorages[0];
    assert(pStorage);

    return pStorage->writeBlock(dstOffset, pSrc, byteCount);
}

void HSS_Storage_GetInfo(uint32_t *pBlockSize, uint32_t *pEraseSize, uint32_t *pBlockCount)
{
    struct HSS_Storage *pStorage = pDefaultStorage ? pDefaultStorage : pStorages[0];
    assert(pStorage);

    if (pStorage->getInfo) {
        pStorage->getInfo(pBlockSize, pEraseSize, pBlockCount);
    }
    mHSS_DEBUG_PRINTF(LOG_NORMAL, "%s - %u byte pages, %u byte blocks, %u pages\n", pStorage->name, *pBlockSize, *pEraseSize, *pBlockCount);
}

void HSS_Storage_FlushWriteBuffer(void)
{
    struct HSS_Storage *pStorage = pDefaultStorage ? pDefaultStorage : pStorages[0];
    assert(pStorage);

    if (pStorage->flushWriteBuffer) {
        pStorage->flushWriteBuffer();
    }
}
