/** ----------------------------------------------------------------------------
 * @file        BSP_CoreMMC.c
 * @brief       HSS-side shim for the secondary eMMC access layer: forwards
 *              the BSP-shaped interface onto the HSS CoreMMC service, which
 *              owns the one mmc_instance_t in this tree.
 *
 *              See BSP_CoreMMC.h for why the shim exists. The equivalent
 *              application file (src/application/bsp/BSP_CoreMMC.c) holds the
 *              instance itself; keep the two interfaces in step, not the
 *              implementations.
 *
 * @author      Koksal Kurt | koksal@rfim.co.uk
 * @date        August 2026
 * @version     1.3.0       /// Initial version (HSS port)
 *
 * @copyright   RFIM Space 2026
 * -----------------------------------------------------------------------------
 */

#include "config.h"
#include "hss_types.h"     /* IS_ENABLED()                                  */

#include "BSP_CoreMMC.h"
#include "BSP_Config.h"

#include <stddef.h>
#include <stdint.h>

#if IS_ENABLED(CONFIG_SERVICE_COREMMC)
#  include "coremmc_service.h"
#endif

bsp_coremmc_status_t
BSP_coremmc_init(void)
{
#if IS_ENABLED(CONFIG_SERVICE_COREMMC)
    return HSS_CoreMMC_Init() ? BSP_COREMMC_OK : BSP_COREMMC_ERR_INIT;
#else
    return BSP_COREMMC_ERR_INIT;
#endif
}

bsp_coremmc_status_t
BSP_coremmc_read_block(uint32_t sector, void* p_dst)
{
#if IS_ENABLED(CONFIG_SERVICE_COREMMC)
    /* The service takes a byte offset, not a sector number - it is the
     * HSS_Storage read signature. The offset must stay inside a 32-bit
     * size_t on a 32-bit build, but this tree is 64-bit throughout.       */
    if (p_dst == NULL) {
        return BSP_COREMMC_ERR_INVALID_PARAM;
    }

    /* MISRA Rule 11.6 deviation: alignment can only be examined through the
     * pointer's integer representation. The CoreMMC driver moves 32-bit
     * words straight out of this buffer, so an unaligned one faults.      */
    if ((((uintptr_t)p_dst) & 0x3u) != 0u) {
        return BSP_COREMMC_ERR_INVALID_PARAM;
    }

    return HSS_CoreMMC_ReadBlock(p_dst,
        (size_t)sector * (size_t)BSP_COREMMC_BLOCK_SIZE,
        (size_t)BSP_COREMMC_BLOCK_SIZE)
            ? BSP_COREMMC_OK : BSP_COREMMC_ERR_TRANSFER;
#else
    (void)sector;
    (void)p_dst;
    return BSP_COREMMC_ERR_NOT_READY;
#endif
}
