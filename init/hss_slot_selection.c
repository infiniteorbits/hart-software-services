/******************************************************************************
 * Copyright (c) 2024 Infinite Orbits. All rights reserved.
 *
 * @file hss_slot_selection.c
 * @brief Implementation of slot selection.
 *
 * This file contains the implementation of the slot selection functionality,
 * which allows booting up a sepcific image in a specific slot region.
 *
 * @authors
 * - A. Tarragó (abel.tarrago@ixrev.com)
 *
 * @version
 * - 1.0: Initial version only emmc1
 * - 1.1: Add spi-flash
 * - 1.2: Add emmc2
 * 
 * @date
 * - 2024-09-10: Created
 *****************************************************************************/

/*---------------------------------Includes---------------------------------*/
#include "config.h"
#include "hss_types.h"
#include "hss_init.h"
#include "hss_slot_selection.h"
#include "mmc_service.h"
#include "mss_mmc.h"
#include "hss_crc32.h"
#include "hss_debug.h"
#include <string.h>
#include <assert.h>
#include "drivers/mss/CoreSPI/core_spi.h"
#include "drivers/mss/mss_sys_services/mss_sys_services.h"
#include "drivers/mss/mt25ql01gbbb/micron1gflash.h"

/*----------------------------Constant Definitions--------------------------*/
#define SPI_SLOT_SIZE           (10 * 1024 * 1024)  // 10 MB
#define BLOCK_SIZE_SPI          (64 * 1024)         // 64 KB
#define MAX_TRANSFER_SIZE       (32 * 1024)         // 32 KB
#define BLOCK_SIZE_EMMC         512
#define BLOCK_SIZE_CRC          512
#define STREAM_GEN_BASE_ADDR    0x4A000000u

typedef enum {
    DDR_DIS = 1,
    EMMPR_EN = 8,   // Bit 8
    EMMSC_EN = 9,   // Bit 9
    SW_EN = 10,     // Bit 10
    SW_DIS = 11,    // Bit 11
    SW_SEL0 = 12,   // Bit 12
    SW_SEL1 = 13,   // Bit 13
    P3V5_PG = 14,   // Bit 14
    P2V0_PG = 15,   // Bit 15
    P1V5_PG = 16,   // Bit 16
    DDR_PG = 17     // Bit 17
} RegisterBits;

typedef struct {
    uint8_t LastFailed;
    uint8_t CurrentTry;
    uint8_t LastSuccessful;
    uint8_t BootSequence[3];
    uint32_t CRC;
} ParamData;

/*------------------------------Local Variables-----------------------------*/
ParamData Params;
uint8_t buff[sizeof(Params)];
uint32_t* stream_gen_base_register = (void*)(STREAM_GEN_BASE_ADDR + 0x10);

/*-----------------------------Local Functions------------------------------*/
const char* getBootDeviceName(uint8_t id);
void delay1(volatile uint32_t n);
void set_register_bit(uint32_t* register_map_outputs, RegisterBits bit);
void clear_register_bit(uint32_t* register_map_outputs, RegisterBits bit);

const char* getBootDeviceName(uint8_t id)
{
    switch (id) {
        case 0x01:
            return "EMMC1";
        case 0x02:
            return "EMMC2";
        case 0x03:
            return "SPI_FLASH";
        default:
            return "None";
    }
}

/**
 * @brief Function to set a specific bit to 1
 */
void set_register_bit(uint32_t* register_map_outputs, RegisterBits bit)
{
    uint32_t reg_value = *register_map_outputs; // Read the current value of the register

    mHSS_DEBUG_PRINTF(LOG_NORMAL,"\n \r set bit %d of register_map_outputs = %08x", bit, *register_map_outputs);

    switch (bit)
    {
        case EMMPR_EN:
            reg_value |= (1 << EMMPR_EN);
            break;
        case EMMSC_EN:
            reg_value |= (1 << EMMSC_EN);
            break;
        case SW_EN:
            reg_value |= (1 << SW_EN);
            break;
        case SW_SEL0:
            reg_value |= (1 << SW_SEL0);
            break;
        case SW_SEL1:
            reg_value |= (1 << SW_SEL1);
            break;
        default:
            return; // If the bit is not one of the defined ones, do nothing
    }

    *register_map_outputs = reg_value; // Write the new value to the register*/
}

/**
 * @brief Function to clear a specific bit to 0
 */
void clear_register_bit(uint32_t* register_map_outputs, RegisterBits bit)
{
    uint32_t reg_value = *register_map_outputs; // Read the current value of the register

    mHSS_DEBUG_PRINTF(LOG_NORMAL,"\n \r clear bit %d of register_map_outputs = %08x", bit, *register_map_outputs);

    switch (bit)
    {
        case EMMPR_EN:
            reg_value &= ~(1 << EMMPR_EN);
            break;
        case EMMSC_EN:
            reg_value &= ~(1 << EMMSC_EN);
            break;
        case SW_EN:
            reg_value &= ~(1 << SW_EN);
            break;
        case SW_SEL0:
            reg_value &= ~(1 << SW_SEL0);
            break;
        case SW_SEL1:
            reg_value &= ~(1 << SW_SEL1);
            break;
        default:
            return; // If the bit is not one of the defined ones, do nothing
    }

    *register_map_outputs = reg_value; // Write the new value to the register
}

/**
 * @brief Function to enable specific emmc
 */
void enable_emmc(uint8_t emmc_id)
{
    mss_mmc_status_t ret_status;

    switch (emmc_id) {
        case EMMC_PRIMARY:
            clear_register_bit(stream_gen_base_register, SW_SEL0);
            clear_register_bit(stream_gen_base_register, SW_SEL1);
            set_register_bit(stream_gen_base_register, EMMPR_EN);
            clear_register_bit(stream_gen_base_register, EMMSC_EN);
            set_register_bit(stream_gen_base_register, SW_EN);

            ret_status = HSS_MMCInit();
            if (ret_status == 0) {
                 mHSS_DEBUG_PRINTF(LOG_NORMAL,"\n \r - Primary eMMC enabled");
            } else {
                 mHSS_DEBUG_PRINTF(LOG_ERROR,"\n \r - Primary eMMC init Failed");
            }
            break;

        case EMMC_SECONDARY:
            set_register_bit(stream_gen_base_register, SW_SEL0);
            set_register_bit(stream_gen_base_register, SW_SEL1);
            clear_register_bit(stream_gen_base_register, EMMPR_EN);
            set_register_bit(stream_gen_base_register, EMMSC_EN);
            set_register_bit(stream_gen_base_register, SW_EN);

            ret_status = HSS_MMCInit();
            if (ret_status == 0) {
                 mHSS_DEBUG_PRINTF(LOG_NORMAL,"\n \r - Secondary eMMC enabled");
            } else {
                 mHSS_DEBUG_PRINTF(LOG_ERROR,"\n \r - Secondary eMMC init Failed");
            }
            break;

        default:
             mHSS_DEBUG_PRINTF(LOG_ERROR,"\n\r - Invalid eMMC ID");
            break;
    }
}

void delay1(volatile uint32_t n)
{
    while (n)
        n--;
}

void erase_section(uint32_t address) 
{
    for (uint32_t index = 0; index < SPI_SLOT_SIZE; index += BLOCK_SIZE_SPI)
    {
        FLASH_erase_64k_block(address + index);
        delay1(500);
    }
}


void spi_GetInfo(uint32_t *pBlockSize, uint32_t *pEraseSize, uint32_t *pBlockCount)
{
    uint32_t sectorSize = 0x10000; //64KB
    *pEraseSize = *pBlockSize = sectorSize;
}

bool spi_init(void)
{
    static bool initialized = false; // Static flag to check if initialization has already been done

    if (initialized) {
        // If already initialized, exit without repeating the initialization
        return true;
    }

    uint8_t manufacturer_id = 0, device_id = 0;
    FLASH_init();
    FLASH_global_unprotect();
    FLASH_read_device_id(&manufacturer_id, &device_id);

    mHSS_DEBUG_PRINTF(LOG_NORMAL, "SPI Init: Device ID: %u, Manufacturer ID: %u\n", device_id, manufacturer_id);

    initialized = true;  // Mark as initialized

    return true;
}

bool spi_read(void *pDest, size_t srcOffset, size_t byteCount)
{
    uint8_t *pDestBytes = (uint8_t *)pDest;
    size_t totalRead = 0;

    while (totalRead < byteCount) {
        size_t bytesToRead = (byteCount - totalRead) < MAX_TRANSFER_SIZE ? (byteCount - totalRead) : MAX_TRANSFER_SIZE;
        FLASH_read(srcOffset + totalRead, pDestBytes + totalRead, bytesToRead);
        totalRead += bytesToRead;
    }
    return true;
}

bool spi_write(size_t dstOffset, void *pSrc, size_t byteCount)
{
    FLASH_program(dstOffset, pSrc, byteCount);
    return true;
}


void HSS_slot_update_boot_params(int index)
{
    Params.LastFailed = index;
    Params.CurrentTry = index+1;
    Params.LastSuccessful = 0;//Params.LastSuccessful;
    Params.BootSequence[0] = EMMC_PRIMARY;
    Params.BootSequence[1] = EMMC_SECONDARY;
    Params.BootSequence[2] = SPI_FLASH;

    Params.CRC = 0;
    uint32_t crc = CRC32_calculate((const uint8_t *)&buff, sizeof(Params));
    Params.CRC = crc;
    memcpy(buff, &Params, sizeof(Params));

    mHSS_DEBUG_PRINTF(LOG_NORMAL,"LastFailed: %s\n", getBootDeviceName(Params.LastFailed));
    mHSS_DEBUG_PRINTF(LOG_NORMAL,"CurrentTry: %s\n", getBootDeviceName(Params.CurrentTry));
    mHSS_DEBUG_PRINTF(LOG_NORMAL,"LastSuccessful: %s\n", getBootDeviceName(Params.LastSuccessful));
    mHSS_DEBUG_PRINTF(LOG_NORMAL,"BootSequence: %s, %s, %s\n ", 
        getBootDeviceName(Params.BootSequence[0]),
        getBootDeviceName(Params.BootSequence[1]), 
        getBootDeviceName(Params.BootSequence[2]));
    
    FLASH_erase_4k_block(BOOT_PARAMS_PADDR);
    delay1(500);
    spi_write(BOOT_PARAMS_PADDR, buff, sizeof(Params));
    mHSS_DEBUG_PRINTF(LOG_NORMAL,"Boot parameters update with crc %x\n", crc);
}

void HSS_slot_get_boot_params(void)
{
    spi_init();
    spi_read(&buff, BOOT_PARAMS_PADDR, sizeof(Params));

    Params.LastFailed = buff[0];
    Params.CurrentTry = buff[1];
    Params.LastSuccessful = buff[2];
    Params.BootSequence[0] = EMMC_PRIMARY;
    Params.BootSequence[1] = EMMC_SECONDARY;
    Params.BootSequence[2] = SPI_FLASH;
    Params.CRC = (buff[11] << 24) |
                 (buff[10] << 16) |
                 (buff[9] << 8)  |
                  buff[8];
    
    buff[11] = 0;
    buff[10] = 0;
    buff[9] = 0;
    buff[8] = 0;
    uint32_t crc = CRC32_calculate((const uint8_t *)&buff, sizeof(Params));

    if(crc != Params.CRC) {
        mHSS_DEBUG_PRINTF(LOG_ERROR,"Boot parameters CRC: calculated %x vs expected %x\n", crc, Params.CRC);
    }else{
        mHSS_DEBUG_PRINTF(LOG_NORMAL,"Boot parameters passed CRC: %x\n", crc);
    }
}

bool validateCrc_custom_emmc(struct HSS_BootImage *pImage)
{
    uint8_t temp_buffer[BLOCK_SIZE_EMMC] = {0};
    uint8_t header_buffer[BLOCK_SIZE_EMMC] = {0};
    uint32_t block_offset = 0;
    uint32_t CRC_read = 0;
    uint32_t CRC_calculated = 0;
    uint32_t bootImageLength = 0;
    int8_t status = 0;
    bool result = false;
    uint32_t start_addr = EMMC0_PADDR / BLOCK_SIZE_EMMC;

    mHSS_DEBUG_PRINTF(LOG_NORMAL, "reading CRC\n");
    status = MSS_MMC_single_block_read(start_addr, (uint32_t*)header_buffer);
    if (status == MSS_MMC_TRANSFER_SUCCESS)
    {
        CRC_read =  (header_buffer[19] << 24) |
                    (header_buffer[18] << 16) |
                    (header_buffer[17] << 8)  |
                    header_buffer[16];     

        status = MSS_MMC_single_block_read((start_addr + 2), (uint32_t*)header_buffer);

        if (status == MSS_MMC_TRANSFER_SUCCESS)
        {                
            bootImageLength =   (header_buffer[459] << 24) |
                                (header_buffer[458] << 16) |
                                (header_buffer[457] << 8)  |
                                header_buffer[456];      

            mHSS_DEBUG_PRINTF(LOG_NORMAL, "bootImageLength: 0x%0X (%d)\n", bootImageLength, bootImageLength);
            mHSS_DEBUG_PRINTF(LOG_NORMAL, "calculating CRC\n");
            for (uint32_t bytes_read = 0; bytes_read < bootImageLength ; bytes_read += BLOCK_SIZE_EMMC )
            {
                status = MSS_MMC_single_block_read(start_addr + block_offset, (uint32_t*)temp_buffer);
                if (status == MSS_MMC_TRANSFER_SUCCESS)
                { 
                    if(block_offset == 0)
                    {
                        temp_buffer[19] = 0;
                        temp_buffer[18] = 0;
                        temp_buffer[17] = 0;
                        temp_buffer[16] = 0;
                    }
                    CRC_calculated = CRC32_calculate_ex(CRC_calculated, (const uint8_t *)&temp_buffer, BLOCK_SIZE_EMMC);
                    block_offset++;
                }else {
                    mHSS_DEBUG_PRINTF(LOG_ERROR, "error reading eMMC blocks\n");
                }
            }

            mHSS_DEBUG_PRINTF(LOG_NORMAL, "CRC_calculated: 0x%0X\n", CRC_calculated);

        }else{
            mHSS_DEBUG_PRINTF(LOG_ERROR, "error reading bootImageLength\n");
        }               
    
    } else{
        mHSS_DEBUG_PRINTF(LOG_ERROR, "error reading header\n");
    }    

    if (CRC_read == CRC_calculated) {
        result = true;
    } else {
        mHSS_DEBUG_PRINTF(LOG_ERROR, "Boot image CRC: calculated %08x vs expected %08x\n",
            CRC_calculated, CRC_read);
    }

    pImage->headerCrc = CRC_calculated;

    return result;
}
void display_output(uint8_t* in_buffer);
void display_output(uint8_t* in_buffer)
{
    const uint8_t* data = (const uint8_t*)in_buffer;  // Cast to uint8_t pointer

        mHSS_DEBUG_PRINTF(LOG_NORMAL,
            "\n%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \n"
            "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \n"
            "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \n"
            "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \n"
            "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \n"
            "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \n"
            "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \n"
            "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \n"
            "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \n"
            "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \n"
            "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \n"
            "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \n"
            "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \n"
            "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \n"
            "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \n"
            "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \n"
            "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \n"
            "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \n"
            "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \n"
            "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \n"
            "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \n"
            "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \n"
            "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \n"
            "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \n"
            "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \n"
            "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \n"
            "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \n"
            "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \n"
            "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \n"
            "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \n"
            "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \n"
            "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \n"
            , data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7],
            data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15],
            data[16], data[17], data[18], data[19], data[20], data[21], data[22], data[23],
            data[24], data[25], data[26], data[27], data[28], data[29], data[30], data[31],
            data[32], data[33], data[34], data[35], data[36], data[37], data[38], data[39],
            data[40], data[41], data[42], data[43], data[44], data[45], data[46], data[47],
            data[48], data[49], data[50], data[51], data[52], data[53], data[54], data[55],
            data[56], data[57], data[58], data[59], data[60], data[61], data[62], data[63],
            data[64], data[65], data[66], data[67], data[68], data[69], data[70], data[71],
            data[72], data[73], data[74], data[75], data[76], data[77], data[78], data[79],
            data[80], data[81], data[82], data[83], data[84], data[85], data[86], data[87],
            data[88], data[89], data[90], data[91], data[92], data[93], data[94], data[95],
            data[96], data[97], data[98], data[99], data[100], data[101], data[102], data[103],
            data[104], data[105], data[106], data[107], data[108], data[109], data[110], data[111],
            data[112], data[113], data[114], data[115], data[116], data[117], data[118], data[119],
            data[120], data[121], data[122], data[123], data[124], data[125], data[126], data[127],
            data[128], data[129], data[130], data[131], data[132], data[133], data[134], data[135],
            data[136], data[137], data[138], data[139], data[140], data[141], data[142], data[143],
            data[144], data[145], data[146], data[147], data[148], data[149], data[150], data[151],
            data[152], data[153], data[154], data[155], data[156], data[157], data[158], data[159],
            data[160], data[161], data[162], data[163], data[164], data[165], data[166], data[167],
            data[168], data[169], data[170], data[171], data[172], data[173], data[174], data[175],
            data[176], data[177], data[178], data[179], data[180], data[181], data[182], data[183],
            data[184], data[185], data[186], data[187], data[188], data[189], data[190], data[191],
            data[192], data[193], data[194], data[195], data[196], data[197], data[198], data[199],
            data[200], data[201], data[202], data[203], data[204], data[205], data[206], data[207],
            data[208], data[209], data[210], data[211], data[212], data[213], data[214], data[215],
            data[216], data[217], data[218], data[219], data[220], data[221], data[222], data[223],
            data[224], data[225], data[226], data[227], data[228], data[229], data[230], data[231],
            data[232], data[233], data[234], data[235], data[236], data[237], data[238], data[239],
            data[240], data[241], data[242], data[243], data[244], data[245], data[246], data[247],
            data[248], data[249], data[250], data[251], data[252], data[253], data[254], data[255],
            data[256], data[257], data[258], data[259], data[260], data[261], data[262], data[263],
            data[264], data[265], data[266], data[267], data[268], data[269], data[270], data[271],
            data[272], data[273], data[274], data[275], data[276], data[277], data[278], data[279],
            data[280], data[281], data[282], data[283], data[284], data[285], data[286], data[287],
            data[288], data[289], data[290], data[291], data[292], data[293], data[294], data[295],
            data[296], data[297], data[298], data[299], data[300], data[301], data[302], data[303],
            data[304], data[305], data[306], data[307], data[308], data[309], data[310], data[311],
            data[312], data[313], data[314], data[315], data[316], data[317], data[318], data[319],
            data[320], data[321], data[322], data[323], data[324], data[325], data[326], data[327],
            data[328], data[329], data[330], data[331], data[332], data[333], data[334], data[335],
            data[336], data[337], data[338], data[339], data[340], data[341], data[342], data[343],
            data[344], data[345], data[346], data[347], data[348], data[349], data[350], data[351],
            data[352], data[353], data[354], data[355], data[356], data[357], data[358], data[359],
            data[360], data[361], data[362], data[363], data[364], data[365], data[366], data[367],
            data[368], data[369], data[370], data[371], data[372], data[373], data[374], data[375],
            data[376], data[377], data[378], data[379], data[380], data[381], data[382], data[383],
            data[384], data[385], data[386], data[387], data[388], data[389], data[390], data[391],
            data[392], data[393], data[394], data[395], data[396], data[397], data[398], data[399],
            data[400], data[401], data[402], data[403], data[404], data[405], data[406], data[407],
            data[408], data[409], data[410], data[411], data[412], data[413], data[414], data[415],
            data[416], data[417], data[418], data[419], data[420], data[421], data[422], data[423],
            data[424], data[425], data[426], data[427], data[428], data[429], data[430], data[431],
            data[432], data[433], data[434], data[435], data[436], data[437], data[438], data[439],
            data[440], data[441], data[442], data[443], data[444], data[445], data[446], data[447],
            data[448], data[449], data[450], data[451], data[452], data[453], data[454], data[455],
            data[456], data[457], data[458], data[459], data[460], data[461], data[462], data[463],
            data[464], data[465], data[466], data[467], data[468], data[469], data[470], data[471],
            data[472], data[473], data[474], data[475], data[476], data[477], data[478], data[479],
            data[480], data[481], data[482], data[483], data[484], data[485], data[486], data[487],
            data[488], data[489], data[490], data[491], data[492], data[493], data[494], data[495],
            data[496], data[497], data[498], data[499], data[500], data[501], data[502], data[503],
            data[504], data[505], data[506], data[507], data[508], data[509], data[510], data[511]);
}

static uint8_t spi_checkCrc(uint32_t headerCrc_read, uint32_t headerCrc_calculated);
static void spi_readCrc(uint32_t start_addr, uint32_t *headerCrc);
static void spi_CalculateCrc(uint32_t start_addr, uint32_t bootImageLength, uint32_t *headerCrc);
static void spi_readBootImageLength(uint32_t start_addr, uint32_t *bootImageLength);

bool validateCrc_custom_spi(struct HSS_BootImage *pImage)
{
   uint32_t headerCrc_read = 0;
   uint32_t headerCrc_calculated = 0;
   uint32_t bootImageLength = 0;
   uint32_t start_addr = SPI0_PADDR;
   uint8_t status = 0;

    spi_readCrc(start_addr, &headerCrc_read);
    spi_readBootImageLength(start_addr, &bootImageLength);
    spi_CalculateCrc(start_addr, bootImageLength, &headerCrc_calculated);
    status = spi_checkCrc(headerCrc_read, headerCrc_calculated);
    pImage->headerCrc = headerCrc_calculated;

    return status;
}

static uint8_t spi_checkCrc(uint32_t headerCrc_read, uint32_t headerCrc_calculated) {

    uint8_t status;

    if(headerCrc_read == headerCrc_calculated){
        status = true;
        //mHSS_DEBUG_PRINTF(LOG_NORMAL, "CRC_calculated: 0x%0X\n", headerCrc_calculated);
    }else {
        status = false;
        mHSS_DEBUG_PRINTF(LOG_ERROR, "Boot image CRC: calculated %08x vs expected %08x\n",
                headerCrc_calculated, headerCrc_read);
    }
    return status;
}

static void spi_readCrc(uint32_t start_addr, uint32_t *headerCrc) {
    uint8_t header_buffer[BLOCK_SIZE_CRC] = {0};
    FLASH_read(start_addr, header_buffer, (size_t)BLOCK_SIZE_CRC);

     *headerCrc = (header_buffer[19] << 24) |         // Save the CRC read from the header
                  (header_buffer[18] << 16) |
                  (header_buffer[17] << 8)  |
                   header_buffer[16];
}

static void spi_readBootImageLength(uint32_t start_addr, uint32_t *bootImageLength) {
    uint8_t header_buffer[BLOCK_SIZE_CRC] = {0};
    FLASH_read(start_addr + (BLOCK_SIZE_CRC * 2), header_buffer, (size_t)BLOCK_SIZE_CRC);
    *bootImageLength = (header_buffer[459] << 24) |
                       (header_buffer[458] << 16) |
                       (header_buffer[457] << 8)  |
                        header_buffer[456];
    mHSS_DEBUG_PRINTF(LOG_NORMAL, "BootImageLength: 0x%0X (%d)\n", *bootImageLength, *bootImageLength);
}

static void spi_CalculateCrc(uint32_t start_addr, uint32_t bootImageLength, uint32_t *headerCrc) {
    uint8_t temp_buffer[BLOCK_SIZE_CRC] = {0};
    uint32_t block_offset = 0;

    *headerCrc = 0;
    mHSS_DEBUG_PRINTF(LOG_NORMAL,"Calculating CRC...\n");
    for (uint32_t bytes_read = 0; bytes_read < bootImageLength; bytes_read += BLOCK_SIZE_CRC) {
        FLASH_read(start_addr + block_offset, temp_buffer, (size_t)BLOCK_SIZE_CRC);
        if(block_offset == 0)
        {
            for (int i = 16; i < 20; i++) {
                temp_buffer[i] = 0;         // Clear the CRC bytes in the header
            }
        }
        *headerCrc = CRC32_calculate_ex(*headerCrc, (const uint8_t *)&temp_buffer, BLOCK_SIZE_CRC);
        block_offset= block_offset + BLOCK_SIZE_CRC;
    }
}
