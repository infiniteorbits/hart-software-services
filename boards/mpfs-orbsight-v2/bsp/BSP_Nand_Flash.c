/**-----------------------------------------------------------------------------
 * @file        BSP_Nand_Flash.c
 * @brief       PolarFire SoC implementation of the NAND BSP API
 *
 *              Hardware data path:
 *                Memory --> MM2Stream_DATA ProtoConv --> hamming_encoder --> nand_phy
 *                Memory --> MM2Stream_CMD  ProtoConv --> nand_phy command stream
 *                nand_phy --> hamming_decoder --> Stream2MM ProtoConv --> Memory
 * @author      Koksal Kurt | koksal.kurt@outlook.com
 * @date        July 2026 - October 2026
 *
 * @version     1.4.0       /// Stub replaced by the NAD-Flash bsp_nand.c
 *                          /// driver (branch nand-sdr-read-os2 @ 9ee2519).
 *                          /// Changes from that source: BSP_nand_read /
 *                          /// _write / _erase take no device id and 64-bit
 *                          /// byte addresses; the Bootloader log
 *                          /// (BOOT_log_*) was dropped, BSP_Boot_Loader0
 *                          /// owns it; prototypes already in
 *                          /// BSP_Nand_Flash.h are not repeated here
 *                          /// (-Wredundant-decls); added BSP_nand_program
 *                          /// (program without erase), BSP_nand_write is
 *                          /// erase + BSP_nand_program; in SDR fallback
 *                          /// mode the API no longer re-runs
 *                          /// BSP_nand_init() on every call.
 *                          /// NV-DDR: the block API uses the hardware-ECC
 *                          /// page format of ecc_write_page / ecc_read_page
 *                          /// (the baseline page ops did not match the os2
 *                          /// encoder/decoder), staged in cached buffers,
 *                          /// a 100 us encoder settle instead of the ~93 ms
 *                          /// delay loop, erased pages read as 0xFF;
 *                          /// device size = chip enable CE1 (8 GB).
 *                          /// BSP_nand_init: NV-DDR used only if a
 *                          /// synchronous READ ID works, else the part is
 *                          /// reset to SDR and the SDR path used
 *                          /// (g_fallback_reason); parameter-page buffer
 *                          /// overflow fixed (264 words read into w[256]);
 *                          /// bounded wait in do_init.
 *                          /// Single-file driver: BSP_Nand_Flash_Config.h
 *                          /// (address map, DMA buffers), _Onfi.h (command
 *                          /// stream) and _Bch8.c/.h (BCH-8 decoder) merged
 *                          /// in. Only the block-device API is exported;
 *                          /// the bring-up / diagnostic API (raw and
 *                          /// page-level access, PHY queries, probe
 *                          /// helpers, multi-page and 2-LUN BCH programs,
 *                          /// the BCH-8 software encoder) is gone, with
 *                          /// the code the block-device API does not reach.
 *                          /// All four chip enables (32 GB): the page ops
 *                          /// select the target of the block (reg
 *                          /// 0x0C[13:12]), init sets up and checks every
 *                          /// target in both interfaces, CE2..CE4 only
 *                          /// with an R/B# of their own (per-CE R/B# PHY
 *                          /// change).
 *                          /// BSP_NAND_FORCE_SYNC (BSP_Config.h): no SDR
 *                          /// fallback, the NV-DDR path is always used.
 *                          /// BSP_NAND_PROFILE (BSP_Config.h): step-by-step
 *                          /// timing of the last page read / program on
 *                          /// every path, read by the NAND unit test via
 *                          /// bsp_nand_dbg_profile() (not in the header;
 *                          /// replaces the g_sdr_rd_t, g_sdr_wr_t, g_wr_t
 *                          /// and g_ecc_rd_t debugger arrays)
 * @version     1.3.0
 *
 * @copyright   RFIM Space 2025-2026
 * -----------------------------------------------------------------------------
*/
#include "BSP_Nand_Flash.h"
#include "BSP_Config.h"
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

/* ==================================================================== *
 *        Configuration: geometry, fabric address map, DMA buffers       *
 * ==================================================================== */

/* NAND geometry - MT29F256G08 (public values: BSP_NAND_* in BSP_Nand_Flash.h). Per target (chip enable):
 * NAND_LUNS x NAND_LUN_BLOCKS blocks, checked against each target's ONFI parameter page at init. */
#define NAND_PAGE_DATA_SIZE   BSP_NAND_PAGE_SIZE
#define NAND_PAGES_PER_BLOCK  BSP_NAND_PAGES_PER_BLOCK
#define NAND_BLOCK_SIZE       BSP_NAND_BLOCK_SIZE
#define NAND_TARGET_BLOCKS    BSP_NAND_TARGET_BLOCKS
#define NAND_LUN_BLOCKS       4096u
#define NAND_LUNS             2u
#define NAND_DEVICE_SIZE      BSP_NAND_DEVICE_SIZE

/* calc_addr() puts the LUN select right above the block bits of a LUN (row address = page | block << 7) */
_Static_assert((NAND_LUN_BLOCKS * NAND_LUNS) == NAND_TARGET_BLOCKS, "a target holds NAND_LUNS x NAND_LUN_BLOCKS blocks");
_Static_assert((NAND_LUN_BLOCKS & (NAND_LUN_BLOCKS - 1u)) == 0u, "the LUN select bit must follow the block bits");

/* Fabric address map - OrbSight-2 NAND design (AXI4INTERCONNECT_6S on
 * MSS FIC-1, reached through the 64-bit FIC window 0x30_xxxx_xxxx):
 *   SLAVE2 0xF000_0000 NAND PHY regs   SLAVE4 0xF400_0000 MM2S CMD
 *   SLAVE3 0xF800_0000 MM2S DATA       SLAVE5 0xFC00_0000 S2MM      */
#define MM2STREAM_DATA_BASE   0x30F8000000UL
#define MM2STREAM_CMD_BASE    0x30F4000000UL
#define STREAM2MM_BASE        0x30FC000000UL
#define NAND_CTRL_REG_BASE    0x30F0000000UL
#define NAND_CLK_MODE         (NAND_CTRL_REG_BASE + 0x00)
#define NAND_CMD_PENDING      (NAND_CTRL_REG_BASE + 0x04)
#define NAND_CMD_NOF_BYTES    (NAND_CTRL_REG_BASE + 0x08)
#define NAND_DBG_REG          (NAND_CTRL_REG_BASE + 0x0C)

/* CoreAXI4ProtoConv register offsets - VERIFY against handbook */
/* CoreAXI4ProtoConv S2MM offsets */
#define S2MM_CTRL       0x10
#define S2MM_STS        0x14
#define S2MM_LEN        0x18
#define S2MM_ADDR0      0x1C
#define S2MM_ADDR1      0x20
#define S2MM_INTRENB    0x24
#define S2MM_INTRSRC    0x28

/* CoreAXI4ProtoConv MM2S offsets */
#define MM2S_CTRL       0x410
#define MM2S_STS        0x414
#define MM2S_LEN        0x418
#define MM2S_ADDR0      0x41C
#define MM2S_ADDR1      0x420
#define MM2S_INTRENB    0x424
#define MM2S_INTRSRC    0x428

#define PC_CTRL_START   0x3u   /* START (bit0) + BURTYP=01 INCR burst (bits 2:1); 0x1 = FIXED burst! */
#define PC_STATUS_DONE  0x1u
#define PC_STATUS_ERROR 0xCu   /* response bits [3:2], non-zero means error */

/* DMA buffers (DDR cached region): BSP_NAND_DMA_BUF_ADDR / _SIZE in BSP_Config.h */
#define DMA_BUF_BASE          ((uint64_t)BSP_NAND_DMA_BUF_ADDR)
#define DMA_CMD_BUF_ADDR      (DMA_BUF_BASE + 0x00000)
#define DMA_WRITE_BUF_ADDR    (DMA_BUF_BASE + 0x10000)
#define DMA_READ_BUF_ADDR     (DMA_BUF_BASE + 0x20000)
#define DMA_SCRATCH_ADDR      (DMA_BUF_BASE + 0x30000)   /* parity words of zero-copy SDR reads (v7+ PHY) */
#define DMA_BUF_END           (DMA_BUF_BASE + 0x40000)

/* Non-cached DDR alias of the read buffer (0x8xxx_xxxx -> 0xCxxx_xxxx) */
#define DMA_NC_ALIAS_OFFSET   0x40000000UL
#define NC_RD_BUF             (DMA_READ_BUF_ADDR + DMA_NC_ALIAS_OFFSET)

/* the buffers fit the reserved region, and the region and its non-cached alias lie inside the DDR windows */
_Static_assert((DMA_BUF_END - DMA_BUF_BASE) <= BSP_NAND_DMA_BUF_SIZE, "NAND DMA buffers exceed BSP_NAND_DMA_BUF_SIZE");
_Static_assert((DMA_BUF_BASE >= 0x80000000UL) && ((DMA_BUF_BASE + BSP_NAND_DMA_BUF_SIZE) <= 0x90000000UL),
               "BSP_NAND_DMA_BUF_ADDR must lie in 0x80000000 - 0x8FFFFFFF (non-cached alias window)");

/* ==================================================================== *
 *               ONFI command stream (nand_phy opcodes)                 *
 * ==================================================================== */

typedef uint32_t t_op_element;

enum {
    ONFI_OP_ASYNC_DATA       = 1u << 0,
    ONFI_OP_ASYNC_CMD        = 1u << 1,
    ONFI_OP_ASYNC_ADDRESS    = 1u << 2,
    ONFI_OP_SYNC_CMD         = 1u << 3,
    ONFI_OP_SYNC_ADDRESS     = 1u << 4,
    ONFI_OP_SYNC_WRITE_DATA  = 1u << 5,
    ONFI_OP_SYNC_READ_DATA   = 1u << 6,
    ONFI_OP_WAIT_RB_N        = 1u << 7,
    ONFI_OP_WAIT_TADL        = 1u << 8,
    ONFI_OP_WAIT_TRHW        = 1u << 9,
    ONFI_OP_WAIT_TWHR        = 1u << 10
};

#define OP_BYTE_PACK(op, b) ((uint32_t)(((uint32_t)(op) << 8) | ((uint32_t)(b) & 0xFFu)))

typedef struct {
    uint8_t C1;  /* Column CA[7:0]  */
    uint8_t C2;  /* Column CA[10:8] */
    uint8_t R1;  /* Row    RA[7:0]  */
    uint8_t R2;  /* Row    RA[15:8] */
    uint8_t R3;  /* Row    RA[17:16]*/
} nand_address_t;

static const t_op_element C_INIT_SEQUENCE[] = {
    OP_BYTE_PACK(ONFI_OP_WAIT_RB_N,     0x00),
    OP_BYTE_PACK(ONFI_OP_ASYNC_CMD,     0xFF),
    OP_BYTE_PACK(ONFI_OP_WAIT_RB_N,     0x00),
    OP_BYTE_PACK(ONFI_OP_ASYNC_CMD,     0xEF),
    OP_BYTE_PACK(ONFI_OP_ASYNC_ADDRESS, 0x01),
    OP_BYTE_PACK(ONFI_OP_WAIT_TADL,     0x00),
    OP_BYTE_PACK(ONFI_OP_ASYNC_DATA,    0x15),
    OP_BYTE_PACK(ONFI_OP_ASYNC_DATA,    0x00),
    OP_BYTE_PACK(ONFI_OP_ASYNC_DATA,    0x00),
    OP_BYTE_PACK(ONFI_OP_ASYNC_DATA,    0x00),
    OP_BYTE_PACK(ONFI_OP_WAIT_RB_N,     0x00)
};
#define C_INIT_SEQUENCE_LEN (sizeof(C_INIT_SEQUENCE)/sizeof(C_INIT_SEQUENCE[0]))

/* ==================================================================== *
 *                 PHY variants, SDR timing, page layouts               *
 * ==================================================================== */

/* ---- SDR v4 (nand_os2_sdrrd PHY): timing fields in the upper half of the nof_bytes register ----
 * [19:16] RE#/WE# low clocks, [23:20] RE#/WE# high clocks (0 = 2), [27:24] capture clocks after the RE# fall,
 * [28] falling-edge sample, [29] fast engine (0 = the v3 180 ns pulse engine). 10 ns per clock. */
#define PHY_SIG_SDR_V4   0x5D520006u                        /* v6 PHY: write-pop fix + packed reads */
#define PHY_SIG_SDR_V7   0x5D520007u                        /* v7 PHY: + pack32, reg 0x0C bit 11: two 16-bit words per 32-bit DMA beat */
#define PHY_SIG_SDR_V8   0x5D520008u                        /* v8 PHY: + unpack32, reg 0x0C bit 14: the encoder takes two words per beat */
#define PHY_SIG_SDR_V9   0x5D520009u                        /* v9 PHY: + BCH-8 page codec (reg 0x0C[16:15]) + fast SDR write engine (nof_bytes[31]) */
#define PHY_SIG_SDR_V10  0x5D52000Au                        /* v10 PHY: v9 + parity XOR the erased mask (erased pages read clean) */
#define DBG_ECC_BCH      (1u << 15)                         /* reg 0x0C[16:15] = 01: BCH-8 page (encoder appends, checker replaces parity by remainder) */
#define DBG_PACK32       (1u << 11)
#define DBG_UNPACK32     (1u << 14)
#define DBG_WIN_RB       9u                                 /* debug window 9: falling edges of the watched R/B# [30:0], level [31] */
#define SDR_PACK        (1u << 30)                         /* two flash bytes per 16-bit word */
#define SDR_T(lo, hi, cap, half, fast) ((((uint32_t)(fast)) << 29) | (((uint32_t)(half)) << 28) | \
    ((((uint32_t)(cap)) & 0xFu) << 24) | ((((uint32_t)(hi)) & 0xFu) << 20) | ((((uint32_t)(lo)) & 0xFu) << 16))
#define SDR_FW(lo, hi, L) ((1u << 31) | ((((uint32_t)(L)) & 3u) << 24) | ((((uint32_t)(hi)) & 15u) << 20) | ((((uint32_t)(lo)) & 15u) << 16))

/* BCH-8 page (v9+ PHY), from column 0 */
#define BCH_PAGE_WORDS   4202u                              /* 4096 data + bad-block marker + 104 parity + pad */
#define BCH_PAGE_BYTES   8404u
#define BCH_TRAILER      212u                               /* marker (2) + 16 x 13 parity or remainder (208) + pad (2) */

/* os2 design: ECC page write/read with the real sizes.
 *  - hamming: every 512 data words (16-bit) -> 512 + 10 parity words on NAND
 *    so one 8 KB page = 4096 data words -> 4176 NAND words
 *  - MM2S DATA / S2MM are 32-bit, encoder/decoder are 16-bit: each 16-bit
 *    word travels in its own 32-bit DMA slot (low half) */
#define ECC_BLK_WORDS    512u
#define ECC_PAR_WORDS    10u
#define PAGE_DATA_WORDS  (NAND_PAGE_DATA_SIZE / 2u)                                        /* 4096 */
#define PAGE_NAND_WORDS  (PAGE_DATA_WORDS + (PAGE_DATA_WORDS / ECC_BLK_WORDS) * ECC_PAR_WORDS) /* 4176 */
#define ECC_WR_SETTLE_CYCLES 60000u                     /* 100 us at the 600 MHz MSS core clock (rdcycle) */

/* why BSP_nand_init() selected the SDR path (g_fallback_reason) */
#define NAND_FALLBACK_NONE        0                     /* NV-DDR path in use (or a PHY without the SDR ops) */
#define NAND_FALLBACK_NO_NVDDR    1                     /* the part reports no NV-DDR interface */
#define NAND_FALLBACK_NVDDR_READ  3                     /* NV-DDR init done, but a synchronous READ ID failed (g_nvddr_check_*) */

/* BSP_NAND_FORCE_SYNC (BSP_Config.h): BSP_nand_init() stays on the NV-DDR path where it would otherwise fall back */
#ifdef BSP_NAND_FORCE_SYNC
#define NAND_FORCE_SYNC 1
#else
#define NAND_FORCE_SYNC 0
#endif

/* ==================================================================== *
 *                            Driver state                              *
 * ==================================================================== */

static bool     g_initialized = false;
static uint32_t g_sdr_rd_timing = 0u;                     /* chosen by sdr_calibrate(); 0 = slow v3 engine */
static uint32_t g_sdr_wr_timing = SDR_T(1, 2, 0, 0, 1);    /* WE# low 10 ns, setup/high 20 ns: 40 ns per byte */
static int      g_sdr_fallback  = 0;                       /* 1 = the part has no NV-DDR: all access through SDR */
static int      g_fallback_reason = NAND_FALLBACK_NONE;    /* NAND_FALLBACK_*: why the SDR path is in use */
static uint8_t  g_part_param[256];                         /* ONFI parameter page read at init (valid if g_part_valid) */
static int      g_part_valid = 0;
static int      g_phy_sdr = 0;                             /* PHY carries the nand_os2_sdrrd signature (debug window 12) */
static int      g_phy_pack32 = 0;                          /* v7+ PHY: SDR packed reads can land contiguously in memory */
static int      g_phy_unpack32 = 0;                        /* v8+ PHY: SDR page writes can be DMA'd from a contiguous buffer */
static int      g_phy_bch = 0;                             /* v9 PHY: BCH-8 page codec and fast write engine */
static uint32_t g_sync_rx_ok = 0u;                         /* bit t: a short arm-after synchronous read on target t delivered data since the last init / PHY abort */
static uint32_t g_dbg_ctrl = 0u;                           /* software mirror of the reg 0x0C control bits (reads return debug windows) */
static uint32_t g_ce = 0u;                                 /* selected target, reg 0x0C[13:12]: 0 = CE1 .. 3 = CE4 (nand_select) */
static uint32_t g_targets = 1u;                            /* bit t: target t passed the init check (CE1 always) */

/* timing instrumentation (BSP_NAND_PROFILE): g_prof_rd / g_prof_wr, see prof_begin; also readable in the debugger */

/* The SDR ops clear g_initialized: they leave the async clock mode, which the synchronous path cannot use. In SDR
 * fallback mode every access runs in that mode, so the driver stays initialized (re-running BSP_nand_init() here
 * would reset and recalibrate the part before every API call). */
static bsp_nand_status_t ensure_initialized(void) {
    if (g_initialized || g_sdr_fallback) return BSP_NAND_OK;
    return BSP_nand_init();
}

/* ==================================================================== *
 *                BCH-8 decoder (from NAD-Flash bch8.c)                 *
 * ==================================================================== *
 * Matches nand_os2_sdrrd/hdl/bch8_pkg.vhd, generated by scripts/gen_bch8.py. Binary BCH over GF(2^13), primitive
 * polynomial x^13 + x^4 + x^3 + x + 1, t = 8 bits per 512-byte sector, 13 parity bytes. Message bits = the sector's
 * bytes in order, MSB first; parity byte 0 = r103..r96 ... byte 12 = r7..r0. The FPGA appends the parity when writing
 * (BCH page mode) and, when reading, replaces each stored parity block by the remainder (stored XOR recomputed): all
 * zero = clean sector. bch8_correct() fixes a sector from that remainder. Table-free GF(2^13) arithmetic (fits the
 * 256 KB L2-LIM of the bare-metal build); a correction costs a few milliseconds and only runs for sectors the FPGA
 * flagged. */
#define BCH8_ECC_BYTES    13u
#define BCH8_T            8
#define GF_N    8191u
#define GF_POLY 0x201Bu
#define NBITS   (4096u + 104u)            /* shortened codeword: 4096 message + 104 parity bits */

static uint16_t gf_mul(uint16_t a, uint16_t b) {
    uint16_t r = 0;
    while (b) {
        if (b & 1u) r ^= a;
        b >>= 1;
        a <<= 1;
        if (a & 0x2000u) a ^= GF_POLY;
    }
    return r;
}
static uint16_t gf_pow(uint16_t a, uint32_t e) {
    uint16_t r = 1;
    e %= GF_N;
    while (e) { if (e & 1u) r = gf_mul(r, a); a = gf_mul(a, a); e >>= 1; }
    return r;
}
static uint16_t gf_inv(uint16_t a) { return gf_pow(a, GF_N - 1u); }

/* 1 if the 13-byte remainder is not all zero */
static int bch8_rem_nonzero(const uint8_t rem[BCH8_ECC_BYTES]) {
    uint8_t o = 0;
    for (uint32_t k = 0; k < BCH8_ECC_BYTES; k++) o |= rem[k];
    return o != 0;
}

/* correct one sector in place from its remainder: number of corrected bits (data and parity bits, 0..8),
 * or -1 if the error pattern is not correctable (more than 8 bit errors) */
static int bch8_correct(uint8_t *sector, const uint8_t rem[BCH8_ECC_BYTES]) {
    if (!bch8_rem_nonzero(rem)) return 0;
    /* syndromes S_j = R(alpha^j), j = 1..16, Horner from r103 down to r0 */
    uint16_t S[2 * BCH8_T + 1];
    uint16_t beta = 1;
    for (int j = 1; j <= 2 * BCH8_T; j++) {
        beta = gf_mul(beta, 2u);              /* alpha^j */
        uint16_t s = 0;
        for (int i = 103; i >= 0; i--) {
            uint32_t bit = (rem[(103 - i) / 8] >> (7 - ((103 - i) % 8))) & 1u;
            s = gf_mul(s, beta) ^ (uint16_t)bit;
        }
        S[j] = s;
    }
    /* Berlekamp-Massey */
    uint16_t C[2 * BCH8_T + 1] = {1}, B[2 * BCH8_T + 1] = {1}, Tm[2 * BCH8_T + 1];
    int L = 0, m = 1; uint16_t b = 1;
    for (int n = 0; n < 2 * BCH8_T; n++) {
        uint16_t d = S[n + 1];
        for (int i = 1; i <= L; i++) d ^= gf_mul(C[i], S[n + 1 - i]);
        if (d == 0) { m++; continue; }
        uint16_t coef = gf_mul(d, gf_inv(b));
        for (int i = 0; i <= 2 * BCH8_T; i++) Tm[i] = C[i];
        for (int i = 0; i + m <= 2 * BCH8_T; i++) if (B[i]) C[i + m] ^= gf_mul(coef, B[i]);
        if (2 * L <= n) {
            L = n + 1 - L;
            for (int i = 0; i <= 2 * BCH8_T; i++) B[i] = Tm[i];
            b = d; m = 1;
        } else {
            m++;
        }
    }
    if (L > BCH8_T) return -1;
    /* Chien search over the shortened code: error at x^i  <=>  Lambda(alpha^-i) = 0 */
    uint16_t term[BCH8_T + 1], step[BCH8_T + 1];
    for (int k = 0; k <= L; k++) { term[k] = C[k]; step[k] = gf_pow(2u, (GF_N - (uint32_t)k) % GF_N); }
    int pos[BCH8_T], nroots = 0;
    for (uint32_t i = 0; i < NBITS; i++) {
        uint16_t sum = 0;
        for (int k = 0; k <= L; k++) sum ^= term[k];
        if (sum == 0) { if (nroots == L) return -1; pos[nroots++] = (int)i; }
        for (int k = 1; k <= L; k++) term[k] = gf_mul(term[k], step[k]);
    }
    if (nroots != L) return -1;
    for (int r = 0; r < nroots; r++) {
        if (pos[r] >= 104) {                  /* message bit j = 4199 - i (0 = byte 0 bit 7) */
            uint32_t j = (NBITS - 1u) - (uint32_t)pos[r];
            sector[j / 8u] ^= (uint8_t)(0x80u >> (j % 8u));
        }                                     /* parity-bit errors leave the data unchanged */
    }
    return L;
}

/* ==================================================================== *
 *                      Low-level hardware access                       *
 * ==================================================================== */

/* ---- Register access ---- */
static inline void     reg_wr32(uint64_t a, uint32_t v) { *((volatile uint32_t *)a) = v; }
static inline uint32_t reg_rd32(uint64_t a)             { return *((volatile uint32_t *)a); }

static inline uint64_t cyc(void) { uint64_t c; __asm volatile("rdcycle %0" : "=r"(c)); return c; }

/* ---- Page-op profile (BSP_NAND_PROFILE, BSP_Config.h) ----
 * nand_read_page() / nand_program_page() open a profile; every prof_mark() inside the page op closes a step: the
 * cycles since the previous mark go to the step named by the mark. The steps therefore follow the path actually taken
 * and add up to the whole page op. Outside a page op g_prof is NULL and marks only move the timestamp. Without
 * BSP_NAND_PROFILE the marks compile to nothing. */
#ifdef BSP_NAND_PROFILE
#define NAND_PROF_STEPS 12u
typedef struct {
    uint64_t    cycles[NAND_PROF_STEPS];   /* CPU cycles (rdcycle) of each step */
    const char *name[NAND_PROF_STEPS];     /* what each step does */
    const char *path;                      /* interface path taken */
    uint32_t    steps;                     /* steps recorded */
    uint32_t    reserved;                  /* unused (no padding) */
} nand_prof_t;
static nand_prof_t  g_prof_rd = { .path = "none" };          /* last page read    */
static nand_prof_t  g_prof_wr = { .path = "none" };          /* last page program */
static nand_prof_t *g_prof    = NULL;                        /* profile being recorded */
static uint64_t     g_prof_t  = 0u;                          /* end of the previous step */

static void prof_begin(nand_prof_t *p) {
    p->steps = 0u;
    p->path  = "none";
    g_prof   = p;
    g_prof_t = cyc();
}
static void prof_mark(const char *step) {
    const uint64_t t = cyc();
    if ((g_prof != NULL) && (g_prof->steps < NAND_PROF_STEPS)) {
        g_prof->name[g_prof->steps]   = step;
        g_prof->cycles[g_prof->steps] = t - g_prof_t;
        g_prof->steps++;
    }
    g_prof_t = t;
}
static void prof_path(const char *path) { if (g_prof != NULL) g_prof->path = path; }
static void prof_end(void)              { g_prof = NULL; }

/* Diagnostics, not part of the BSP_Nand_Flash.h API: the step timing of the last page program (program != 0) or the
 * last page read. Copies up to max_steps step names and CPU cycle counts in the order the steps ran, sets *path to
 * the interface path, returns the number of steps copied. bsp_nand_unit_test.c declares it itself: keep the two
 * prototypes the same. */
uint32_t bsp_nand_dbg_profile(uint32_t program, const char **path, const char **names, uint64_t *cycles,
                              uint32_t max_steps);
uint32_t bsp_nand_dbg_profile(uint32_t program, const char **path, const char **names, uint64_t *cycles,
                              uint32_t max_steps) {
    const nand_prof_t *p = (program != 0u) ? &g_prof_wr : &g_prof_rd;
    uint32_t n = (p->steps < max_steps) ? p->steps : max_steps;
    if ((names == NULL) || (cycles == NULL)) n = 0u;
    if (path != NULL) *path = p->path;
    for (uint32_t i = 0; i < n; i++) { names[i] = p->name[i]; cycles[i] = p->cycles[i]; }
    return n;
}
#else
#define prof_begin(p) ((void)0)
#define prof_mark(s)  ((void)0)
#define prof_path(s)  ((void)0)
#define prof_end()    ((void)0)
#endif

/* The DMA buffers live in CACHED DDR (0x8100_0000) and the fabric ProtoConv
 * masters are NOT cache-coherent. SiFive L2 controller FLUSH64 (0x0201_0200):
 * writing a 64-byte line address performs write-back + invalidate of that
 * line - used to push CPU writes to DRAM before an MM2S read, and to drop
 * stale lines before the CPU reads back an S2MM destination buffer. */
#define L2_FLUSH64 (*(volatile uint64_t *)0x02010200ULL)
static inline void dcache_flush_range(uint64_t addr, uint32_t len) {
    __sync_synchronize();
    for (uint64_t p = (addr & ~63ULL); p < (addr + len); p += 64ULL) {
        L2_FLUSH64 = p;
    }
    __sync_synchronize();
}

/* ---- ProtoConv ---- */
static int pc_wait_done_mm2s(uint64_t base, uint32_t timeout) {
    for (uint32_t i = 0; i < timeout; i++) {
        uint32_t s = reg_rd32(base + MM2S_STS);

        if (s & PC_STATUS_ERROR)
            return -1;

        if (s & PC_STATUS_DONE)
            return 0;
    }
    return -2;
}

static int pc_wait_done_s2mm(uint64_t base, uint32_t timeout) {
    for (uint32_t i = 0; i < timeout; i++) {
        uint32_t s = reg_rd32(base + S2MM_STS);

        if (s & PC_STATUS_ERROR)
            return -1;

        if (s & PC_STATUS_DONE)
            return 0;
    }
    return -2;
}

static int pc_mm2s(uint64_t base, uint64_t src, uint32_t len) {
    reg_wr32(base + MM2S_ADDR0, (uint32_t)(src & 0xFFFFFFFFUL));
    reg_wr32(base + MM2S_ADDR1, (uint32_t)(src >> 32));
    reg_wr32(base + MM2S_LEN,   len);

    __sync_synchronize();

    reg_wr32(base + MM2S_CTRL, PC_CTRL_START);

    return pc_wait_done_mm2s(base, 10000000);
}

static void pc_mm2s_start(uint64_t base, uint64_t src, uint32_t len) {     /* like pc_mm2s() without the wait */
    reg_wr32(base + MM2S_ADDR0, (uint32_t)(src & 0xFFFFFFFFUL));
    reg_wr32(base + MM2S_ADDR1, (uint32_t)(src >> 32));
    reg_wr32(base + MM2S_LEN,   len);
    __sync_synchronize();
    reg_wr32(base + MM2S_CTRL, PC_CTRL_START);
}

static void pc_s2mm_arm(uint64_t base, uint64_t dst, uint32_t len) {
    reg_wr32(base + S2MM_ADDR0, (uint32_t)(dst & 0xFFFFFFFFUL));
    reg_wr32(base + S2MM_ADDR1, (uint32_t)(dst >> 32));
    reg_wr32(base + S2MM_LEN,   len);

    __sync_synchronize();

    reg_wr32(base + S2MM_CTRL, PC_CTRL_START);
}

/* ---- NAND PHY ---- */
static void wait_cmd(void)        { while (reg_rd32(NAND_CMD_PENDING)) {} }
static void set_clock_async(void) { reg_wr32(NAND_CLK_MODE, 0x1); }
static void set_clock_sync (void) { reg_wr32(NAND_CLK_MODE, 0x0); }

/* ---- Command sequence buffer ---- */
static uint32_t * const g_cmd_buf = (uint32_t *)DMA_CMD_BUF_ADDR;
static size_t           g_cmd_cnt = 0;
static void cmd_reset(void)         { g_cmd_cnt = 0; }
static void cmd_push (uint32_t w)   { g_cmd_buf[g_cmd_cnt++] = w; }
static int  cmd_flush(void) {
    if (!g_cmd_cnt) return 0;
    /* push the CPU-cached command words out to DRAM before the fabric reads them */
    dcache_flush_range(DMA_CMD_BUF_ADDR, (uint32_t)(g_cmd_cnt * sizeof(uint32_t)));
    int rc = pc_mm2s(MM2STREAM_CMD_BASE, (uint64_t)DMA_CMD_BUF_ADDR, g_cmd_cnt * sizeof(uint32_t));
    cmd_reset();
    return rc;
}

/* ---- Address calculation ---- */
static int calc_addr(uint32_t block, uint32_t page, uint32_t col, nand_address_t *a) {
    if (page  >= NAND_PAGES_PER_BLOCK) return -1;
    if (block >= NAND_TARGET_BLOCKS)   return -1;                 /* block of the selected target */
    a->C1 = (uint8_t)(col & 0xFFu);
    a->C2 = (uint8_t)((col >> 8) & 0xFFu);     /* was & 0x0F: columns >= 4096 (the spare area at 8192) wrapped to 0 */
    a->R1 = (uint8_t)(((block & 0x01u) << 7) | (page & 0x7Fu));
    a->R2 = (uint8_t)((block >> 1) & 0xFFu);
    a->R3 = (uint8_t)((block >> 9) & 0x7Fu);
    return 0;
}

/* ---- PHY control and debug windows ---- */

/* S2MM-LATE variant: the Stream2MM is armed only after the PHY has finished the read
 * (the captured words wait in the 4096 + 8192 word FIFOs), so a read that never gets DQS
 * edges leaves no starved S2MM behind (GS-21 bus-stall condition). Returns 0 = command done. */
static int wait_cmd_bounded(void) {
    for (uint32_t t = 0; t < 4000000u; t++) { if (reg_rd32(NAND_CMD_PENDING) == 0u) return 0; }
    return -1;
}

static void phy_fsm_reset(void) {           /* reg 0x0C[9] is a level: set it, then clear it; keep the control bits */
    reg_wr32(NAND_DBG_REG, g_dbg_ctrl | 0x200u);
    __asm volatile("fence" ::: "memory");
    reg_wr32(NAND_DBG_REG, g_dbg_ctrl);
    __asm volatile("fence" ::: "memory");
}

static uint32_t dbg_win(uint32_t sel) {        /* read a PHY debug window, keep the control bits */
    reg_wr32(NAND_DBG_REG, (g_dbg_ctrl & ~0xFu) | (sel & 0xFu));
    __asm volatile("fence" ::: "memory");
    uint32_t v = reg_rd32(NAND_DBG_REG);
    reg_wr32(NAND_DBG_REG, g_dbg_ctrl);
    __asm volatile("fence" ::: "memory");
    return v;
}

/* v3 PHY: a synchronous read that gets no DQS edges is aborted by the PHY after ~160 us and counted in debug
 * window 11. "command done" therefore no longer implies "data captured": check the counter before arming. */
static uint32_t g_rd_timeouts = 0u;
static void phy_abort(void) {                  /* the FSM reset also clears debug window 11: resync the mirror */
    phy_fsm_reset();
    g_rd_timeouts = g_phy_sdr ? dbg_win(11u) : 0u;      /* the baseline PHY has no window 11 */
    g_sync_rx_ok = 0u;
}
static int read_cmd_done_or_reset(void) {
    if (wait_cmd_bounded() != 0) { phy_abort(); return -1; }
    if (!g_phy_sdr) return 0;                                      /* baseline PHY: no time-out counter to check */
    uint32_t t = dbg_win(11u);
    if (t != g_rd_timeouts) { g_rd_timeouts = t; return -1; }     /* PHY time-out: no data, do not arm the S2MM */
    return 0;
}

/* PHY capabilities from the debug window 12 signature */
static int phy_has_sdr(void)      { uint32_t v = dbg_win(12u); return (v == PHY_SIG_SDR_V4 || v == PHY_SIG_SDR_V7 || v == PHY_SIG_SDR_V8 || v == PHY_SIG_SDR_V9 || v == PHY_SIG_SDR_V10); }
static int phy_has_pack32(void)   { uint32_t v = dbg_win(12u); return (v == PHY_SIG_SDR_V7 || v == PHY_SIG_SDR_V8 || v == PHY_SIG_SDR_V9 || v == PHY_SIG_SDR_V10); }
static int phy_has_unpack32(void) { uint32_t v = dbg_win(12u); return (v == PHY_SIG_SDR_V8 || v == PHY_SIG_SDR_V9 || v == PHY_SIG_SDR_V10); }
static int phy_has_bch(void)      { uint32_t v = dbg_win(12u); return (v == PHY_SIG_SDR_V9 || v == PHY_SIG_SDR_V10); }

/* ---- Targets (chip enables) ----
 * Reg 0x0C[13:12] selects the target of the following command streams in both interfaces: the PHY asserts its CE#
 * and, with the per-CE R/B# change, WAIT_RB_N and debug window 9 follow its R/B# (without it they watch RB1). The
 * select is changed only between command streams; the SDR ops keep it in g_dbg_ctrl. */
static void nand_select(uint32_t t) {
    g_ce = t & 3u;
    g_dbg_ctrl = (g_dbg_ctrl & ~(3u << 12)) | (g_ce << 12);
    reg_wr32(NAND_DBG_REG, g_dbg_ctrl);
    __asm volatile("fence" ::: "memory");
}
static inline int target_ok(uint32_t t) { return (int)((g_targets >> t) & 1u); }

/* ==================================================================== *
 *       SDR (asynchronous) access - needs the nand_os2_sdrrd PHY       *
 * ==================================================================== *
 * Async command/address cycles; with reg 0x0C[27:26] = "11" the PHY toggles RE# and pulses DQS itself, so
 * SYNC_READ_DATA executes as SDR data-out cycles. The controller is left in the async clock mode afterwards
 * (BSP_nand_init() runs again before the next synchronous access). For a flash without NV-DDR (MT29F64G08ADAAA3W found
 * on the failing boards). */

/* packed SDR reads (two flash bytes per word, nof_bytes[30]) exist only in the fast read engine (nof_bytes[29]): with
 * the slow engine the PHY would deliver one byte per word, twice the words the DMA takes, and the surplus would shift
 * every later read. The packed page reads therefore refuse to run without a calibrated fast timing (-11). */
static inline int sdr_pack_ok(void) { return (g_sdr_rd_timing & (1u << 29)) != 0u; }

static void sdr_cmd_begin(void) {
    g_initialized = false;                                      /* SDR ops leave the controller in the async clock mode */
    set_clock_async();
    g_dbg_ctrl = 0x400u | (3u << 26) | (g_ce << 12);
    reg_wr32(NAND_DBG_REG, g_dbg_ctrl);                /* decoder bypass + SDR read mode */
    __asm volatile("fence" ::: "memory");
    cmd_reset();
}

/* Short SDR read (READ ID / READ PARAMETER PAGE / READ STATUS). Each 16-bit word carries the same byte in both halves
 * (one DQS pulse per RE# cycle). */
static int sdr_read(uint8_t cmd, int use_addr, uint8_t addr, int wait_rb, uint16_t *dst, uint32_t words)
{
    if (words == 0u || words > 2048u) return -2;
    volatile uint32_t *buf = (volatile uint32_t *)NC_RD_BUF;
    for (uint32_t i = 0; i < words; i++) buf[i] = 0xDEAD0000u | i;
    __asm volatile("fence" ::: "memory");

    g_initialized = false;                              /* SDR ops leave the controller in the async clock mode */
    set_clock_async();                                  /* WE#/CLK pin idles high: SDR command/address cycles */
    g_dbg_ctrl = 0x400u | (3u << 26) | (g_ce << 12);
    reg_wr32(NAND_DBG_REG, g_dbg_ctrl);
    __asm volatile("fence" ::: "memory");
    {
        uint32_t nb = words | g_sdr_rd_timing;
        reg_wr32(NAND_CMD_NOF_BYTES, nb);
        while (reg_rd32(NAND_CMD_NOF_BYTES) != nb) {}
    }

    /* In SDR mode the PHY produces exactly `words` beats whatever the flash does, so the S2MM can be armed
     * first (no starvation risk) and the read path never stalls on a full downstream FIFO. */
    pc_s2mm_arm(STREAM2MM_BASE, NC_RD_BUF, words * 4u);
    cmd_reset();
    cmd_push(OP_BYTE_PACK(ONFI_OP_WAIT_TRHW,      0x00));
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_CMD,      cmd));
    if (use_addr) cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_ADDRESS, addr));
    if (wait_rb)  cmd_push(OP_BYTE_PACK(ONFI_OP_WAIT_RB_N, 0x00));
    else          cmd_push(OP_BYTE_PACK(ONFI_OP_WAIT_TWHR, 0x00));
    cmd_push(OP_BYTE_PACK(ONFI_OP_SYNC_READ_DATA, 0x00));
    int rc = -4;
    if (cmd_flush() == 0) {
        if (read_cmd_done_or_reset() != 0) rc = -6;
        else rc = (pc_wait_done_s2mm(STREAM2MM_BASE, 2000000u) == 0) ? 0 : -5;
    }
    __asm volatile("fence" ::: "memory");
    for (uint32_t i = 0; i < words; i++) dst[i] = (uint16_t)(buf[i] & 0xFFFFu);
    g_dbg_ctrl = g_ce << 12; reg_wr32(NAND_DBG_REG, g_dbg_ctrl);     /* leave SDR mode (the caller re-applies its own bits) */
    __asm volatile("fence" ::: "memory");
    return rc;
}

/* Read-path resync. A word that entered the read path outside a transfer is delivered at the start of the next
 * transfer and shifts every later read by one word (seen after a power-up and on a freshly programmed fabric; it
 * also turns READ STATUS into garbage). READ ID has a known start (2C 88), so its offset k is the number of stray
 * words; exactly k words of that read are then still pending and are drained with a k-beat transfer, which is safe
 * because they are known to be there (an armed Stream2MM that receives nothing would wedge the bus). */
static uint32_t g_sdr_resync_drained = 0u;                                    /* stray words drained so far */
static int sdr_resync(void) {
    static uint16_t idw[8];
    for (int iter = 0; iter < 4; iter++) {
        uint32_t saved = g_sdr_rd_timing; g_sdr_rd_timing = 0u;              /* slow engine: needs no calibration */
        int r = sdr_read(0x90u, 1, 0x00u, 0, idw, 8u);
        g_sdr_rd_timing = saved;
        if (r != 0) return -1;
        uint32_t k = 8u;
        for (uint32_t o = 0; o + 1u < 8u; o++) if ((idw[o] & 0xFFu) == 0x2Cu && (idw[o + 1u] & 0xFFu) == 0x88u) { k = o; break; }
        if (k == 0u) return 0;
        if (k >= 8u) return -2;                                               /* ID not found: offset unknown */
        g_dbg_ctrl = 0x400u | (3u << 26) | (g_ce << 12);               /* SDR + bypass: the pending words are visible */
        reg_wr32(NAND_DBG_REG, g_dbg_ctrl);
        __asm volatile("fence" ::: "memory");
        pc_s2mm_arm(STREAM2MM_BASE, NC_RD_BUF, k * 4u);
        int d = pc_wait_done_s2mm(STREAM2MM_BASE, 200000u);
        g_dbg_ctrl = g_ce << 12; reg_wr32(NAND_DBG_REG, g_dbg_ctrl);
        __asm volatile("fence" ::: "memory");
        if (d != 0) return -3;
        g_sdr_resync_drained += k;
    }
    return -4;
}

static int sdr_status(uint8_t *st) {                          /* READ STATUS 70h */
    uint16_t w[4];
    int rc = sdr_read(0x70u, 0, 0x00u, 0, w, 2u);
    *st = (uint8_t)(w[0] & 0xFFu);
    return rc;
}

static int sdr_reset(void) {                                  /* RESET FFh in the asynchronous interface */
    sdr_cmd_begin();
    cmd_push(OP_BYTE_PACK(ONFI_OP_WAIT_RB_N,  0x00));
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_CMD,  0xFF));
    cmd_push(OP_BYTE_PACK(ONFI_OP_WAIT_RB_N,  0x00));
    if (cmd_flush() != 0) return -4;
    if (wait_cmd_bounded() != 0) { phy_abort(); return -6; }
    return 0;
}

static int sdr_erase(uint32_t block) {
    nand_address_t a;
    if (calc_addr(block, 0, 0, &a) != 0) return -2;
    sdr_cmd_begin();
    cmd_push(OP_BYTE_PACK(ONFI_OP_WAIT_TRHW,      0x00));
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_CMD,      0x60));
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_ADDRESS,  a.R1));
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_ADDRESS,  a.R2));
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_ADDRESS,  a.R3));
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_CMD,      0xD0));
    cmd_push(OP_BYTE_PACK(ONFI_OP_WAIT_RB_N,      0x00));
    if (cmd_flush() != 0) return -4;
    if (wait_cmd_bounded() != 0) { phy_abort(); return -6; }
    uint8_t st = 0xFFu;
    if (sdr_status(&st) != 0) return -5;
    return (st & 0x01u) ? -8 : 0;                                /* status bit 0 = FAIL */
}

/* calibration of the fast read timing: every (RE# low, RE# high) pair is swept over the capture clock and the
 * sample edge; a setting passes when a 256-byte parameter-page read equals the slow-engine reference, on every target
 * in use (one timing serves the shared bus). Leaves the last target selected: the caller selects CE1 again. */
typedef struct { uint8_t lo, hi; uint32_t pass_mask; } sdr_cal_t;   /* bit (cap*2 + half), cap 0..9 */
static sdr_cal_t g_sdr_cal[6];
static int g_sdr_cal_n = 0;
static int sdr_calibrate(void) {                 /* picks the fast read timing; 0 = ok, <0 = slow engine kept */
    static uint16_t w[256]; static uint8_t ref[BSP_NAND_NUM_TARGETS][256];
    static const uint8_t cfg[6][2] = { {1,1}, {1,2}, {2,1}, {2,2}, {3,2}, {3,3} };
    g_sdr_rd_timing = 0u;
    for (uint32_t t = 0; t < BSP_NAND_NUM_TARGETS; t++) {             /* slow-engine reference of every target */
        if (!target_ok(t)) continue;
        nand_select(t);
        if (sdr_read(0xECu, 1, 0x00u, 1, w, 256u) != 0) return -1;
        for (uint32_t i = 0; i < 256u; i++) ref[t][i] = (uint8_t)(w[i] & 0xFFu);
        if (!(ref[t][0] == 'O' && ref[t][1] == 'N' && ref[t][2] == 'F' && ref[t][3] == 'I')) return -2;
    }
    int best = -1, best_center = 0, best_len = 0;
    g_sdr_cal_n = 0;
    for (int c = 0; c < 6; c++) {
        uint32_t mask = 0;
        for (uint32_t cap = 0; cap < 10u; cap++) {
            for (uint32_t half = 0; half < 2u; half++) {
                g_sdr_rd_timing = SDR_T(cfg[c][0], cfg[c][1], cap, half, 1);
                uint32_t ok = 1u;
                for (uint32_t t = 0; ok && t < BSP_NAND_NUM_TARGETS; t++) {
                    if (!target_ok(t)) continue;
                    nand_select(t);
                    ok = (sdr_read(0xECu, 1, 0x00u, 1, w, 256u) == 0);
                    for (uint32_t i = 0; ok && i < 256u; i++) if ((uint8_t)(w[i] & 0xFFu) != ref[t][i]) ok = 0;
                }
                if (ok) mask |= 1u << (cap * 2u + half);
            }
        }
        g_sdr_cal[c].lo = cfg[c][0]; g_sdr_cal[c].hi = cfg[c][1]; g_sdr_cal[c].pass_mask = mask; g_sdr_cal_n = c + 1;
        int run = 0, run_start = 0, len = 0, start = 0;          /* longest run of passing points (5 ns each) */
        for (int p = 0; p < 20; p++) {
            if (mask & (1u << p)) { if (run == 0) run_start = p; run++; if (run > len) { len = run; start = run_start; } }
            else run = 0;
        }
        int period = cfg[c][0] + cfg[c][1];
        if (len >= 3) {
            int bp = (best < 0) ? 99 : (g_sdr_cal[best].lo + g_sdr_cal[best].hi);
            if (period < bp || (period == bp && len > best_len)) { best = c; best_len = len; best_center = start + len / 2; }
        }
    }
    if (best < 0) { g_sdr_rd_timing = 0u; return -3; }     /* no robust fast setting: stay on the slow engine */
    g_sdr_rd_timing = SDR_T(g_sdr_cal[best].lo, g_sdr_cal[best].hi, best_center / 2, best_center & 1, 1);
    static uint16_t w3[768];                                /* confirm with the three parameter-page copies */
    for (uint32_t t = 0; t < BSP_NAND_NUM_TARGETS; t++) {
        if (!target_ok(t)) continue;
        nand_select(t);
        if (sdr_read(0xECu, 1, 0x00u, 1, w3, 768u) != 0) { g_sdr_rd_timing = 0u; return -4; }
        for (uint32_t i = 0; i < 256u; i++) if ((uint8_t)(w3[i] & 0xFFu) != ref[t][i]) { g_sdr_rd_timing = 0u; return -5; }
    }
    return 0;
}

/* Zero-copy page program (v8 PHY): the encoder takes the 8192 data bytes straight from a 64-byte aligned DDR buffer
 * (two 16-bit words per 32-bit beat, reg 0x0C bit 14); no CPU copy into the 32-bit-slot buffer. */
static int sdr_program_page_dma(uint32_t block, uint32_t page, uint64_t src_ddr) {
    nand_address_t a;
    if (!g_phy_unpack32) return -9;
    if ((src_ddr & 63u) != 0u || calc_addr(block, page, 0, &a) != 0) return -2;
    dcache_flush_range(src_ddr, NAND_PAGE_DATA_SIZE);                 /* CPU-written source data to DDR */
    prof_mark("L2 flush of the staging buffer");
    sdr_cmd_begin();
    g_dbg_ctrl |= DBG_UNPACK32; reg_wr32(NAND_DBG_REG, g_dbg_ctrl);      /* before the MM2S: the encoder consumes at once */
    __asm volatile("fence" ::: "memory");
    {
        uint32_t nb = PAGE_NAND_WORDS | g_sdr_wr_timing;
        reg_wr32(NAND_CMD_NOF_BYTES, nb);
        while (reg_rd32(NAND_CMD_NOF_BYTES) != nb) {}
    }
    pc_mm2s_start(MM2STREAM_DATA_BASE, src_ddr, NAND_PAGE_DATA_SIZE);     /* 2048 beats, two words each */
    prof_mark("PHY setup + DMA start");
    cmd_reset();
    cmd_push(OP_BYTE_PACK(ONFI_OP_WAIT_TRHW,        0x00));
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_CMD,        0x80));
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_ADDRESS,    a.C1));
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_ADDRESS,    a.C2));
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_ADDRESS,    a.R1));
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_ADDRESS,    a.R2));
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_ADDRESS,    a.R3));
    cmd_push(OP_BYTE_PACK(ONFI_OP_WAIT_TADL,        0x00));
    cmd_push(OP_BYTE_PACK(ONFI_OP_SYNC_WRITE_DATA,  0x00));
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_CMD,        0x10));
    cmd_push(OP_BYTE_PACK(ONFI_OP_WAIT_RB_N,        0x00));
    int rc = 0;
    if (cmd_flush() != 0) rc = -4;
    else if (wait_cmd_bounded() != 0) { phy_abort(); rc = -6; }
    prof_mark("command + data out + tPROG");
    if (rc == 0 && pc_wait_done_mm2s(MM2STREAM_DATA_BASE, 1000000u) != 0) rc = -3;
    g_dbg_ctrl = g_ce << 12; reg_wr32(NAND_DBG_REG, g_dbg_ctrl);
    __asm volatile("fence" ::: "memory");
    prof_mark("DMA completion check");
    uint8_t st = 0xFFu;
    if (rc == 0 && sdr_status(&st) != 0) rc = -5;
    prof_mark("status read (70h)");
    if (rc != 0) return rc;
    return (st & 0x01u) ? -8 : 0;
}

/* ======== v9 PHY: BCH-8 pages and the fast write engine ========
 * BCH page on the flash, from column 0: 8192 data bytes, the bad-block marker word 0xFFFF, 16 x 13 parity bytes
 * (sector 0 first) and a pad word = 8404 bytes. Writing: the encoder appends marker, parity and pad to the 8192 data
 * bytes. Reading: the checker passes data, marker and pad and replaces each stored parity block by the remainder
 * (stored XOR recomputed); all-zero remainder = clean sector, otherwise bch8_correct() fixes it. */
uint32_t g_bch_corrected = 0u;                                              /* totals since init (BSP_Nand_Flash.h) */
static uint32_t g_bch_uncorrectable = 0u, g_bch_sectors_fixed = 0u;

static int sdr_program_v9(uint32_t block, uint32_t page, uint64_t src) {
    nand_address_t a;
    if (calc_addr(block, page, 0, &a) != 0) return -2;
    sdr_cmd_begin();
    g_dbg_ctrl |= DBG_UNPACK32 | DBG_ECC_BCH; reg_wr32(NAND_DBG_REG, g_dbg_ctrl);    /* before the MM2S starts */
    __asm volatile("fence" ::: "memory");
    {
        uint32_t nb = BCH_PAGE_WORDS | g_sdr_wr_timing;
        reg_wr32(NAND_CMD_NOF_BYTES, nb);
        while (reg_rd32(NAND_CMD_NOF_BYTES) != nb) {}
    }
    pc_mm2s_start(MM2STREAM_DATA_BASE, src, NAND_PAGE_DATA_SIZE);
    prof_mark("PHY setup + DMA start");
    cmd_reset();
    cmd_push(OP_BYTE_PACK(ONFI_OP_WAIT_TRHW,        0x00));
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_CMD,        0x80));
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_ADDRESS,    a.C1));
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_ADDRESS,    a.C2));
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_ADDRESS,    a.R1));
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_ADDRESS,    a.R2));
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_ADDRESS,    a.R3));
    cmd_push(OP_BYTE_PACK(ONFI_OP_WAIT_TADL,        0x00));
    cmd_push(OP_BYTE_PACK(ONFI_OP_SYNC_WRITE_DATA,  0x00));
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_CMD,        0x10));
    cmd_push(OP_BYTE_PACK(ONFI_OP_WAIT_RB_N,        0x00));
    int rc = 0;
    if (cmd_flush() != 0) rc = -4;
    else if (wait_cmd_bounded() != 0) { phy_abort(); rc = -6; }
    prof_mark("command + data out + tPROG");
    if (rc == 0 && pc_wait_done_mm2s(MM2STREAM_DATA_BASE, 1000000u) != 0) rc = -3;
    g_dbg_ctrl = g_ce << 12; reg_wr32(NAND_DBG_REG, g_dbg_ctrl);
    __asm volatile("fence" ::: "memory");
    prof_mark("DMA completion check");
    return rc;
}

/* BCH page program from a 64-byte aligned DDR buffer of 8192 bytes; returns 0, -8 program FAIL, other < 0 transfer */
static int sdr_program_page_bch(uint32_t block, uint32_t page, uint64_t src_ddr) {
    if (!g_phy_bch) return -9;
    if ((src_ddr & 63u) != 0u) return -2;
    dcache_flush_range(src_ddr, NAND_PAGE_DATA_SIZE);
    prof_mark("L2 flush of the staging buffer");
    int rc = sdr_program_v9(block, page, src_ddr);
    if (rc != 0) return rc;
    uint8_t st = 0xFFu;
    rc = sdr_status(&st);
    prof_mark("status read (70h)");
    if (rc != 0) return -5;
    return (st & 0x01u) ? -8 : 0;
}

/* program one page through the data stream: 4096 x 16-bit words -> hardware Hamming encoder -> 4176 words
 * written as 8352 bytes (the same flash layout as the synchronous ECC page: 8 x (512 data + 10 parity words)).
 * A v9+ PHY writes a BCH-8 page instead, a v8 PHY takes the page zero-copy from the staging buffer. */
static int sdr_program_page_ecc(uint32_t block, uint32_t page, const uint16_t *src) {
    nand_address_t a;
    if (calc_addr(block, page, 0, &a) != 0) return -2;
    if (g_phy_bch) {                                                  /* v9: BCH-8 page from the DDR staging buffer */
        prof_path("SDR, BCH-8 (v9+ PHY)");
        volatile uint64_t *d64 = (volatile uint64_t *)DMA_WRITE_BUF_ADDR;
        const uint64_t *s64 = (const uint64_t *)src;
        if (((uintptr_t)src & 7u) == 0u) { for (uint32_t i = 0; i < NAND_PAGE_DATA_SIZE / 8u; i++) d64[i] = s64[i]; }
        else { volatile uint16_t *d16 = (volatile uint16_t *)DMA_WRITE_BUF_ADDR; for (uint32_t i = 0; i < PAGE_DATA_WORDS; i++) d16[i] = src[i]; }
        prof_mark("copy into the DMA staging buffer");
        return sdr_program_page_bch(block, page, (uint64_t)DMA_WRITE_BUF_ADDR);
    }
    if (g_phy_unpack32) {                                             /* v8: contiguous 64-bit copy, encoder unpacks */
        prof_path("SDR, Hamming, zero-copy (v8 PHY)");
        volatile uint64_t *d64 = (volatile uint64_t *)DMA_WRITE_BUF_ADDR;
        if (((uintptr_t)src & 7u) == 0u) { const uint64_t *s64 = (const uint64_t *)src; for (uint32_t i = 0; i < NAND_PAGE_DATA_SIZE / 8u; i++) d64[i] = s64[i]; }
        else { volatile uint16_t *d16 = (volatile uint16_t *)DMA_WRITE_BUF_ADDR; for (uint32_t i = 0; i < PAGE_DATA_WORDS; i++) d16[i] = src[i]; }
        prof_mark("copy into the DMA staging buffer");
        return sdr_program_page_dma(block, page, (uint64_t)DMA_WRITE_BUF_ADDR);
    }
    prof_path("SDR, Hamming");
    uint32_t *buf = (uint32_t *)DMA_WRITE_BUF_ADDR;              /* cached alias, written back before the DMA */
    for (uint32_t i = 0; i < PAGE_DATA_WORDS; i++) buf[i] = (uint32_t)src[i];
    prof_mark("widen to one word per 32-bit beat");
    dcache_flush_range(DMA_WRITE_BUF_ADDR, PAGE_DATA_WORDS * 4u);
    prof_mark("L2 flush of the staging buffer");
    sdr_cmd_begin();
    {
        uint32_t nb = PAGE_NAND_WORDS | g_sdr_wr_timing;        /* words leaving the encoder, SDR write timing */
        reg_wr32(NAND_CMD_NOF_BYTES, nb);
        while (reg_rd32(NAND_CMD_NOF_BYTES) != nb) {}
    }
    /* the v5+ write engine waits for data: MM2S started without waiting, the encoder fills the FIFO while the command runs */
    pc_mm2s_start(MM2STREAM_DATA_BASE, (uint64_t)DMA_WRITE_BUF_ADDR, PAGE_DATA_WORDS * 4u);
    prof_mark("PHY setup + DMA start");
    cmd_reset();
    cmd_push(OP_BYTE_PACK(ONFI_OP_WAIT_TRHW,        0x00));
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_CMD,        0x80));
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_ADDRESS,    a.C1));
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_ADDRESS,    a.C2));
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_ADDRESS,    a.R1));
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_ADDRESS,    a.R2));
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_ADDRESS,    a.R3));
    cmd_push(OP_BYTE_PACK(ONFI_OP_WAIT_TADL,        0x00));
    cmd_push(OP_BYTE_PACK(ONFI_OP_SYNC_WRITE_DATA,  0x00));    /* = SDR streamed write in SDR mode */
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_CMD,        0x10));
    cmd_push(OP_BYTE_PACK(ONFI_OP_WAIT_RB_N,        0x00));
    if (cmd_flush() != 0) return -4;
    if (wait_cmd_bounded() != 0) { phy_abort(); return -6; }
    prof_mark("command + data out + tPROG");
    if (pc_wait_done_mm2s(MM2STREAM_DATA_BASE, 1000000u) != 0) return -3;
    prof_mark("DMA completion check");
    uint8_t st = 0xFFu;
    if (sdr_status(&st) != 0) return -5;
    prof_mark("status read (70h)");
    return (st & 0x01u) ? -8 : 0;
}

/* v7 PHY: the decoder bypass packs two 16-bit words per 32-bit beat (reg 0x0C bit 11), so the Stream2MM writes the
 * page bytes contiguously. The 8 x 10 parity words of the hardware-Hamming layout go to a scratch area: one DMA
 * transfer per 1024-byte data chunk and one per 20-byte parity chunk (16 per page), the data straight into dst, no
 * CPU copy. The PHY's flow control holds the NAND while the next transfer is armed. */
static int sdr_read_page_pack32(uint32_t block, uint32_t page, uint64_t dst) {     /* dst: 64-byte aligned DDR buffer */
    nand_address_t a;
    if (!sdr_pack_ok()) return -11;
    if (calc_addr(block, page, 0, &a) != 0) return -2;
    const uint32_t nbytes = PAGE_NAND_WORDS * 2u, nblk = PAGE_DATA_WORDS / ECC_BLK_WORDS;
    const uint64_t par = (uint64_t)DMA_SCRATCH_ADDR;
    dcache_flush_range(dst, NAND_PAGE_DATA_SIZE);                     /* no stale lines over the DMA data */
    dcache_flush_range(par, 64u);
    prof_mark("L2 flush (invalidate) of the DMA buffer");
    sdr_cmd_begin();
    g_dbg_ctrl |= DBG_PACK32; reg_wr32(NAND_DBG_REG, g_dbg_ctrl);
    __asm volatile("fence" ::: "memory");
    {
        uint32_t nb = nbytes | g_sdr_rd_timing | SDR_PACK;
        reg_wr32(NAND_CMD_NOF_BYTES, nb);
        while (reg_rd32(NAND_CMD_NOF_BYTES) != nb) {}
    }
    pc_s2mm_arm(STREAM2MM_BASE, dst, ECC_BLK_WORDS * 2u);                 /* first data chunk */
    cmd_push(OP_BYTE_PACK(ONFI_OP_WAIT_TRHW,      0x00));
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_CMD,      0x00));
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_ADDRESS,  a.C1));
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_ADDRESS,  a.C2));
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_ADDRESS,  a.R1));
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_ADDRESS,  a.R2));
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_ADDRESS,  a.R3));
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_CMD,      0x30));
    cmd_push(OP_BYTE_PACK(ONFI_OP_WAIT_RB_N,      0x00));
    cmd_push(OP_BYTE_PACK(ONFI_OP_SYNC_READ_DATA, 0x00));
    int rc = -4;
    if (cmd_flush() == 0) {
        prof_mark("command + DMA arm");
        rc = 0;
        for (uint32_t blk = 0; blk < nblk; blk++) {
            if (pc_wait_done_s2mm(STREAM2MM_BASE, 4000000u) != 0) { rc = -5; break; }
            pc_s2mm_arm(STREAM2MM_BASE, par, ECC_PAR_WORDS * 2u);             /* 10 parity words -> scratch */
            if (pc_wait_done_s2mm(STREAM2MM_BASE, 4000000u) != 0) { rc = -5; break; }
            if (blk + 1u < nblk) pc_s2mm_arm(STREAM2MM_BASE, dst + (uint64_t)(blk + 1u) * (ECC_BLK_WORDS * 2u), ECC_BLK_WORDS * 2u);
        }
        prof_mark("tR + page transfer + DMA (16 chunks)");
        if (rc == 0 && read_cmd_done_or_reset() != 0) rc = -6;
        if (rc != 0) phy_abort();
    }
    __asm volatile("fence" ::: "memory");
    g_dbg_ctrl = g_ce << 12; reg_wr32(NAND_DBG_REG, g_dbg_ctrl);
    __asm volatile("fence" ::: "memory");
    prof_mark("command completion check");
    return rc;
}

/* BCH page read: 8192 data bytes to dst, the 212-byte trailer (marker, remainder, pad) to trailer */
static int sdr_read_v9(uint32_t block, uint32_t page, uint64_t dst, uint64_t trailer) {
    nand_address_t a;
    if (!sdr_pack_ok()) return -11;
    if (calc_addr(block, page, 0, &a) != 0) return -2;
    dcache_flush_range(dst, NAND_PAGE_DATA_SIZE);
    dcache_flush_range(trailer, BCH_TRAILER);
    prof_mark("L2 flush (invalidate) of the DMA buffer");
    sdr_cmd_begin();
    g_dbg_ctrl |= DBG_PACK32 | DBG_ECC_BCH; reg_wr32(NAND_DBG_REG, g_dbg_ctrl);
    __asm volatile("fence" ::: "memory");
    {
        uint32_t nb = BCH_PAGE_BYTES | g_sdr_rd_timing | SDR_PACK;
        reg_wr32(NAND_CMD_NOF_BYTES, nb);
        while (reg_rd32(NAND_CMD_NOF_BYTES) != nb) {}
    }
    pc_s2mm_arm(STREAM2MM_BASE, dst, NAND_PAGE_DATA_SIZE);
    cmd_push(OP_BYTE_PACK(ONFI_OP_WAIT_TRHW,      0x00));
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_CMD,      0x00));
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_ADDRESS,  a.C1));
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_ADDRESS,  a.C2));
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_ADDRESS,  a.R1));
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_ADDRESS,  a.R2));
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_ADDRESS,  a.R3));
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_CMD,      0x30));
    cmd_push(OP_BYTE_PACK(ONFI_OP_WAIT_RB_N,      0x00));
    cmd_push(OP_BYTE_PACK(ONFI_OP_SYNC_READ_DATA, 0x00));
    int rc = -4;
    if (cmd_flush() == 0) {
        prof_mark("command + DMA arm");
        rc = 0;
        if (pc_wait_done_s2mm(STREAM2MM_BASE, 4000000u) != 0) rc = -5;
        else {
            prof_mark("tR + data transfer + DMA");
            pc_s2mm_arm(STREAM2MM_BASE, trailer, BCH_TRAILER);
            if (pc_wait_done_s2mm(STREAM2MM_BASE, 4000000u) != 0) rc = -5;
            prof_mark("BCH trailer transfer + DMA");
        }
        if (rc == 0 && read_cmd_done_or_reset() != 0) rc = -6;
        if (rc != 0) phy_abort();
    }
    __asm volatile("fence" ::: "memory");
    g_dbg_ctrl = g_ce << 12; reg_wr32(NAND_DBG_REG, g_dbg_ctrl);
    __asm volatile("fence" ::: "memory");
    prof_mark("command completion check");
    return rc;
}

/* BCH page read into a 64-byte aligned DDR buffer, flagged sectors corrected in place. Returns the number of
 * corrected bits (>= 0), -10 if a sector is not correctable, other < 0 transfer errors. */
static int sdr_read_page_bch(uint32_t block, uint32_t page, uint64_t dst_ddr) {
    if (!g_phy_bch) return -9;
    if ((dst_ddr & 63u) != 0u) return -2;
    int rc = sdr_read_v9(block, page, dst_ddr, (uint64_t)DMA_SCRATCH_ADDR);
    if (rc != 0) return rc;
    const volatile uint8_t *tr = (const volatile uint8_t *)DMA_SCRATCH_ADDR;
    const volatile uint64_t *tr64 = (const volatile uint64_t *)DMA_SCRATCH_ADDR;
    uint64_t any = tr64[0] & ~0xFFFFull;                                  /* bytes 0..1 = marker */
    for (uint32_t i = 1; i < 26u; i++) any |= tr64[i];                    /* bytes 8..207 */
    any |= tr64[26] & 0xFFFFull;                                          /* bytes 208..209; 210..211 = pad */
    if (any == 0u) { prof_mark("BCH-8 check / correction"); return 0; }
    int total = 0, bad = 0;
    uint8_t *data = (uint8_t *)(uintptr_t)dst_ddr;
    for (uint32_t sct = 0; sct < 16u; sct++) {
        uint8_t rem[BCH8_ECC_BYTES];
        for (uint32_t k = 0; k < BCH8_ECC_BYTES; k++) rem[k] = tr[2u + 13u * sct + k];
        if (!bch8_rem_nonzero(rem)) continue;
        int n = bch8_correct(&data[512u * sct], rem);
        if (n < 0) { bad++; g_bch_uncorrectable++; } else { total += n; g_bch_corrected += (uint32_t)n; g_bch_sectors_fixed++; }
    }
    dcache_flush_range(dst_ddr, NAND_PAGE_DATA_SIZE);                      /* corrections out to DDR for later DMA */
    prof_mark("BCH-8 check / correction");
    return bad ? -10 : total;
}

/* read one ECC-layout page in SDR mode into dst (4096 data words). v9+ PHY: BCH-8 page; v7+ PHY: zero-copy into the
 * staging buffer. Otherwise 8352 bytes packed two per 16-bit word (4176 beats) into the cached DDR buffer, then the
 * 4096 data words are extracted (the 8 x 10 parity words are skipped). */
static int sdr_read_page_ecc(uint32_t block, uint32_t page, uint16_t *dst) {
    nand_address_t a;
    if (g_phy_bch || g_phy_pack32) {                                  /* contiguous in the staging buffer: 64-bit copy */
        prof_path(g_phy_bch ? "SDR, BCH-8 (v9+ PHY)" : "SDR, Hamming, packed (v7 PHY)");
        int r = g_phy_bch ? sdr_read_page_bch(block, page, (uint64_t)DMA_READ_BUF_ADDR)
                          : sdr_read_page_pack32(block, page, (uint64_t)DMA_READ_BUF_ADDR);
        if (r > 0) r = 0;                                             /* corrected bits: data is good */
        const volatile uint64_t *s64 = (const volatile uint64_t *)DMA_READ_BUF_ADDR;
        if (((uintptr_t)dst & 7u) == 0u) { uint64_t *d64 = (uint64_t *)dst; for (uint32_t i = 0; i < NAND_PAGE_DATA_SIZE / 8u; i++) d64[i] = s64[i]; }
        else { const volatile uint16_t *s16 = (const volatile uint16_t *)DMA_READ_BUF_ADDR; for (uint32_t i = 0; i < PAGE_DATA_WORDS; i++) dst[i] = s16[i]; }
        prof_mark("copy out of the DMA buffer");
        return r;
    }
    prof_path("SDR, Hamming");
    if (!sdr_pack_ok()) return -11;
    if (calc_addr(block, page, 0, &a) != 0) return -2;
    const uint32_t nbytes = PAGE_NAND_WORDS * 2u, beats = PAGE_NAND_WORDS;
    dcache_flush_range(DMA_READ_BUF_ADDR, beats * 4u);             /* no stale lines may be written back later */
    prof_mark("L2 flush (invalidate) of the DMA buffer");
    sdr_cmd_begin();
    {
        uint32_t nb = nbytes | g_sdr_rd_timing | SDR_PACK;
        reg_wr32(NAND_CMD_NOF_BYTES, nb);
        while (reg_rd32(NAND_CMD_NOF_BYTES) != nb) {}
    }
    pc_s2mm_arm(STREAM2MM_BASE, (uint64_t)DMA_READ_BUF_ADDR, beats * 4u);
    cmd_push(OP_BYTE_PACK(ONFI_OP_WAIT_TRHW,      0x00));
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_CMD,      0x00));
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_ADDRESS,  a.C1));
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_ADDRESS,  a.C2));
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_ADDRESS,  a.R1));
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_ADDRESS,  a.R2));
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_ADDRESS,  a.R3));
    cmd_push(OP_BYTE_PACK(ONFI_OP_ASYNC_CMD,      0x30));
    cmd_push(OP_BYTE_PACK(ONFI_OP_WAIT_RB_N,      0x00));
    cmd_push(OP_BYTE_PACK(ONFI_OP_SYNC_READ_DATA, 0x00));
    int rc = -4;
    if (cmd_flush() == 0) {
        prof_mark("command + DMA arm");
        if (read_cmd_done_or_reset() != 0) rc = -6;
        else {
            prof_mark("tR + page transfer (PHY)");
            rc = (pc_wait_done_s2mm(STREAM2MM_BASE, 4000000u) == 0) ? 0 : -5;
            prof_mark("DMA drain to DDR");
        }
    }
    /* The Stream2MM writes the cached alias through the L2, which is the coherence point for the harts; the
     * lines were invalidated before the DMA, so the CPU reads below fetch the DMA data (no second flush).
     * 64-bit loads halve the number of memory accesses; each beat carries one 16-bit word. */
    __asm volatile("fence" ::: "memory");
    const uint64_t *buf64 = (const uint64_t *)DMA_READ_BUF_ADDR;
    uint32_t *d32 = (uint32_t *)dst;
    for (uint32_t blk = 0; blk < PAGE_DATA_WORDS / ECC_BLK_WORDS; blk++) {
        const uint64_t *src = &buf64[(blk * (ECC_BLK_WORDS + ECC_PAR_WORDS)) / 2u];   /* 522 beats per block: even offset */
        uint32_t *d = &d32[(blk * ECC_BLK_WORDS) / 2u];
        for (uint32_t i = 0; i < ECC_BLK_WORDS / 2u; i++) {
            uint64_t v = src[i];                                   /* beat 2i in the low half, beat 2i+1 in the high half */
            d[i] = (uint32_t)(v & 0xFFFFu) | (uint32_t)((v >> 32) & 0xFFFFu) << 16;
        }
    }
    g_dbg_ctrl = g_ce << 12; reg_wr32(NAND_DBG_REG, g_dbg_ctrl);
    __asm volatile("fence" ::: "memory");
    prof_mark("unpack beats to 16-bit words");
    return rc;
}

/* ==================================================================== *
 *                    NV-DDR (synchronous) access                       *
 * ==================================================================== */

/* ---- RAW reads (decoder bypassed via reg 0x0C[10]): one 16-bit word
 *      {falling-edge byte, rising-edge byte} per 32-bit DMA slot ---- */
static int raw_read_id(uint16_t *dst, uint32_t words)
{
    if (ensure_initialized() != BSP_NAND_OK) return -1;
    if (words == 0u || words > 64u) return -2;

    volatile uint32_t *buf = (volatile uint32_t *)NC_RD_BUF;
    for (uint32_t i = 0; i < words; i++) buf[i] = 0xDEAD0000u | i;
    __asm volatile("fence" ::: "memory");

    reg_wr32(NAND_CMD_NOF_BYTES, words);
    while (reg_rd32(NAND_CMD_NOF_BYTES) != words) {}

    cmd_reset();
    cmd_push(OP_BYTE_PACK(ONFI_OP_WAIT_TRHW,      0x00));
    cmd_push(OP_BYTE_PACK(ONFI_OP_SYNC_CMD,       0x90));
    cmd_push(OP_BYTE_PACK(ONFI_OP_SYNC_ADDRESS,   0x00));
    cmd_push(OP_BYTE_PACK(ONFI_OP_WAIT_TWHR,      0x00));
    cmd_push(OP_BYTE_PACK(ONFI_OP_SYNC_READ_DATA, 0x00));
    if (cmd_flush() != 0) return -4;
    if (read_cmd_done_or_reset() != 0) return -6;          /* no data from the flash */
    pc_s2mm_arm(STREAM2MM_BASE, NC_RD_BUF, words * 4u);

    int rc = pc_wait_done_s2mm(STREAM2MM_BASE, 20000000u);
    if (rc == 0) g_sync_rx_ok |= 1u << g_ce;                /* short arm-after read delivered data */
    __asm volatile("fence" ::: "memory");
    for (uint32_t i = 0; i < words; i++) dst[i] = (uint16_t)(buf[i] & 0xFFFFu);
    return (rc == 0) ? 0 : -5;
}

/* Page-size synchronous reads (4176+ words) exceed the read-side buffering (PHY capture FIFO 4096 words, decoder FIFO
 * 512, none in bypass): the Stream2MM must drain while the PHY captures, so those reads arm it FIRST, as the baseline
 * did. A pre-armed Stream2MM that receives nothing wedges the SoC bus, so arm-first is allowed only once a short
 * arm-after read has proven that the selected target answers synchronous reads (its g_sync_rx_ok bit). */
static int sync_rx_verified(void) {
    if (g_sync_rx_ok & (1u << g_ce)) return 0;
    static uint16_t idw[8];
    uint32_t saved = g_dbg_ctrl;
    g_dbg_ctrl = saved | 0x400u; reg_wr32(NAND_DBG_REG, g_dbg_ctrl);   /* decoder bypass: the 8 words must reach the DMA */
    __asm volatile("fence" ::: "memory");
    int r = raw_read_id(idw, 8u);                                      /* arm-after, bounded, no data -> no DMA */
    g_dbg_ctrl = saved; reg_wr32(NAND_DBG_REG, g_dbg_ctrl);
    __asm volatile("fence" ::: "memory");
    return (r == 0 && (g_sync_rx_ok & (1u << g_ce))) ? 0 : -1;
}

/* raw page read from column 0; the caller sets the decoder bypass (reg 0x0C[10]) */
static int raw_read_page(uint32_t block, uint32_t page, uint16_t *dst, uint32_t words)
{
    nand_address_t a;
    if (ensure_initialized() != BSP_NAND_OK) return -1;
    if (calc_addr(block, page, 0, &a) != 0) return -2;
    if (words == 0u || words > 4320u) return -3;

    volatile uint32_t *buf = (volatile uint32_t *)NC_RD_BUF;
    for (uint32_t i = 0; i < words; i++) buf[i] = 0xDEAD0000u | i;
    __asm volatile("fence" ::: "memory");

    reg_wr32(NAND_CMD_NOF_BYTES, words);
    while (reg_rd32(NAND_CMD_NOF_BYTES) != words) {}

    int arm_first = (words > 1024u);                       /* beyond the read-side FIFOs: drain while capturing */
    if (arm_first) { if (sync_rx_verified() != 0) return -7; pc_s2mm_arm(STREAM2MM_BASE, NC_RD_BUF, words * 4u); }
    cmd_reset();
    cmd_push(OP_BYTE_PACK(ONFI_OP_WAIT_TRHW,      0x00));
    cmd_push(OP_BYTE_PACK(ONFI_OP_SYNC_CMD,       0x00));
    cmd_push(OP_BYTE_PACK(ONFI_OP_SYNC_ADDRESS,  a.C1));
    cmd_push(OP_BYTE_PACK(ONFI_OP_SYNC_ADDRESS,  a.C2));
    cmd_push(OP_BYTE_PACK(ONFI_OP_SYNC_ADDRESS,  a.R1));
    cmd_push(OP_BYTE_PACK(ONFI_OP_SYNC_ADDRESS,  a.R2));
    cmd_push(OP_BYTE_PACK(ONFI_OP_SYNC_ADDRESS,  a.R3));
    cmd_push(OP_BYTE_PACK(ONFI_OP_SYNC_CMD,       0x30));
    cmd_push(OP_BYTE_PACK(ONFI_OP_WAIT_RB_N,      0x00));
    cmd_push(OP_BYTE_PACK(ONFI_OP_SYNC_READ_DATA, 0x00));
    if (cmd_flush() != 0) return -4;
    if (arm_first) { (void)wait_cmd_bounded(); }
    else { if (read_cmd_done_or_reset() != 0) return -6; pc_s2mm_arm(STREAM2MM_BASE, NC_RD_BUF, words * 4u); }

    int rc = pc_wait_done_s2mm(STREAM2MM_BASE, 20000000u);
    if (rc == 0 && !arm_first) g_sync_rx_ok |= 1u << g_ce; else if (rc != 0) phy_abort();
    __asm volatile("fence" ::: "memory");
    for (uint32_t i = 0; i < words; i++) dst[i] = (uint16_t)(buf[i] & 0xFFFFu);
    return (rc == 0) ? 0 : -5;
}

/* ECC page program: 4096 x 16-bit words through the hardware Hamming encoder (SDR fallback: sdr_program_page_ecc) */
static int ecc_write_page(uint32_t block, uint32_t page, const uint16_t *src)
{
    nand_address_t a;
    if (ensure_initialized() != BSP_NAND_OK) return -1;
    if (g_sdr_fallback) return sdr_program_page_ecc(block, page, src);
    if (calc_addr(block, page, 0, &a) != 0) return -2;
    prof_path("NV-DDR, Hamming");

    /* Staged in the cached buffer and written back to DDR before the DMA: through the non-cached alias every 32-bit
     * store cost ~0.4 us (~1.6 ms per page on the NV-DDR board). Same pattern as the SDR page program. */
    uint32_t *buf = (uint32_t *)DMA_WRITE_BUF_ADDR;
    for (uint32_t i = 0; i < PAGE_DATA_WORDS; i++) buf[i] = (uint32_t)src[i];
    prof_mark("widen to one word per 32-bit beat");

    /* feed the encoder: 4096 x 16-bit words, one per 32-bit beat */
    dcache_flush_range(DMA_WRITE_BUF_ADDR, PAGE_DATA_WORDS * 4u);
    prof_mark("L2 flush of the staging buffer");
    if (pc_mm2s(MM2STREAM_DATA_BASE, (uint64_t)DMA_WRITE_BUF_ADDR, PAGE_DATA_WORDS * 4u) != 0) return -3;
    prof_mark("MM2S DMA into the encoder (16 KB)");
    uint64_t t2 = cyc();
    /* The synchronous write burst pops nof_bytes words on consecutive clocks without waiting for data
     * (nand_ctrl_phys_layer sync_write_s0_st), so the page must be in the PHY data FIFO before the command. The MM2S
     * reports done with up to its 1024-beat data FIFO (COREAXI4PROTOCONV_C0) still to stream; the encoder passes the
     * words straight through and appends each block's 10 parity words within a few clocks (axis_hamming_encoder):
     * ~1150 clocks, 12 us at 100 MHz. ECC_WR_SETTLE_CYCLES covers that with a wide margin; the former 1M-iteration
     * delay loop took ~93 ms per page. */
    while ((cyc() - t2) < ECC_WR_SETTLE_CYCLES) { }
    prof_mark("encoder settle (fixed 100 us wait)");

    /* data + parity + 1: the encoder delivers 4177 words per page; popping only 4176 leaves one word in the data
     * FIFO and the next page then starts with a word of the previous burst (seen on the NV-DDR board). Fresh pages
     * read with drop 0 on this PHY and on the baseline PHY (clean fabric); a page left by an earlier session needed
     * drop 1, so the self-test writes its read-training page fresh at every run instead of reusing one (a reused
     * page with another layout mis-trains the drop and fails every page). */
    uint32_t words = PAGE_NAND_WORDS + 1u;
    reg_wr32(NAND_CMD_NOF_BYTES, words);
    while (reg_rd32(NAND_CMD_NOF_BYTES) != words) {}

    cmd_reset();
    cmd_push(OP_BYTE_PACK(ONFI_OP_WAIT_TRHW,        0x00));
    cmd_push(OP_BYTE_PACK(ONFI_OP_SYNC_CMD,         0x80));
    cmd_push(OP_BYTE_PACK(ONFI_OP_SYNC_ADDRESS,    a.C1));
    cmd_push(OP_BYTE_PACK(ONFI_OP_SYNC_ADDRESS,    a.C2));
    cmd_push(OP_BYTE_PACK(ONFI_OP_SYNC_ADDRESS,    a.R1));
    cmd_push(OP_BYTE_PACK(ONFI_OP_SYNC_ADDRESS,    a.R2));
    cmd_push(OP_BYTE_PACK(ONFI_OP_SYNC_ADDRESS,    a.R3));
    cmd_push(OP_BYTE_PACK(ONFI_OP_WAIT_TADL,        0x00));
    cmd_push(OP_BYTE_PACK(ONFI_OP_SYNC_WRITE_DATA,  0x00));
    cmd_push(OP_BYTE_PACK(ONFI_OP_SYNC_CMD,         0x10));
    cmd_push(OP_BYTE_PACK(ONFI_OP_WAIT_RB_N,        0x00));
    if (cmd_flush() != 0) return -4;
    prof_mark("command stream out (MM2S, 11 ops)");
    int w = wait_cmd_bounded();
    prof_mark("data out + tPROG (until R/B# ready)");
    if (w != 0) { phy_abort(); return -6; }                 /* command (tPROG) never completed */
    return 0;
}

/* ECC page read: 4096 x 16-bit words from the hardware Hamming decoder (SDR fallback: sdr_read_page_ecc) */
static int ecc_read_page(uint32_t block, uint32_t page, uint16_t *dst)
{
    nand_address_t a;
    if (ensure_initialized() != BSP_NAND_OK) return -1;
    if (g_sdr_fallback) return sdr_read_page_ecc(block, page, dst);
    if (calc_addr(block, page, 0, &a) != 0) return -2;
    prof_path("NV-DDR, Hamming");

    /* Cached staging buffer, invalidated before the DMA: the Stream2MM writes through the L2 (the harts' coherence
     * point), so the CPU then reads the DMA data without a second flush - as in the SDR page read. Reading the
     * non-cached alias cost ~0.4 us per 32-bit word (2.4 MB/s on the NV-DDR board). */
    dcache_flush_range(DMA_READ_BUF_ADDR, PAGE_DATA_WORDS * 4u);
    prof_mark("L2 flush (invalidate) of the DMA buffer");

    uint32_t words = PAGE_NAND_WORDS;                       /* data + parity */
    reg_wr32(NAND_CMD_NOF_BYTES, words);
    while (reg_rd32(NAND_CMD_NOF_BYTES) != words) {}

    if (sync_rx_verified() != 0) return -7;                /* flash gives no synchronous data: do not arm */
    /* decoder outputs 4096 x 16-bit words, each arrives as one 32-bit beat; armed before the command (see sync_rx_verified) */
    pc_s2mm_arm(STREAM2MM_BASE, (uint64_t)DMA_READ_BUF_ADDR, PAGE_DATA_WORDS * 4u);
    prof_mark("PHY setup + DMA arm");
    cmd_reset();
    cmd_push(OP_BYTE_PACK(ONFI_OP_WAIT_TRHW,      0x00));
    cmd_push(OP_BYTE_PACK(ONFI_OP_SYNC_CMD,       0x00));
    cmd_push(OP_BYTE_PACK(ONFI_OP_SYNC_ADDRESS,  a.C1));
    cmd_push(OP_BYTE_PACK(ONFI_OP_SYNC_ADDRESS,  a.C2));
    cmd_push(OP_BYTE_PACK(ONFI_OP_SYNC_ADDRESS,  a.R1));
    cmd_push(OP_BYTE_PACK(ONFI_OP_SYNC_ADDRESS,  a.R2));
    cmd_push(OP_BYTE_PACK(ONFI_OP_SYNC_ADDRESS,  a.R3));
    cmd_push(OP_BYTE_PACK(ONFI_OP_SYNC_CMD,       0x30));
    cmd_push(OP_BYTE_PACK(ONFI_OP_WAIT_RB_N,      0x00));
    cmd_push(OP_BYTE_PACK(ONFI_OP_SYNC_READ_DATA, 0x00));
    if (cmd_flush() != 0) return -4;
    prof_mark("command stream out (MM2S, 10 ops)");
    (void)wait_cmd_bounded();
    prof_mark("tR + page transfer (until command done)");

    int rc = pc_wait_done_s2mm(STREAM2MM_BASE, 20000000u);
    if (rc != 0) phy_abort();
    prof_mark("S2MM DMA drain to DDR");
    __asm volatile("fence" ::: "memory");
    if (((uintptr_t)dst & 3u) == 0u) {                     /* 64-bit loads: two beats, one word in each low half */
        const uint64_t *b64 = (const uint64_t *)DMA_READ_BUF_ADDR;
        uint32_t *d32 = (uint32_t *)dst;
        for (uint32_t i = 0; i < PAGE_DATA_WORDS / 2u; i++) {
            uint64_t v = b64[i];
            d32[i] = (uint32_t)(v & 0xFFFFu) | ((uint32_t)((v >> 32) & 0xFFFFu) << 16);
        }
    } else {
        const uint32_t *b32 = (const uint32_t *)DMA_READ_BUF_ADDR;
        for (uint32_t i = 0; i < PAGE_DATA_WORDS; i++) dst[i] = (uint16_t)(b32[i] & 0xFFFFu);
    }
    prof_mark("unpack beats to 16-bit words");
    return (rc == 0) ? 0 : -5;
}

/* NV-DDR erased pages. Data and parity of an erased block are all 1s; the decoder's parity over 512 data bits of 1
 * is 0x200 (only position 512 has bit 9 set), so the syndrome is 0x200 ^ 0x3FF = 0x1FF in all 16 bit lanes and the
 * decoder inverts one word per 512-word block (hamming_decoder, axis_hamming_decoder correct_data_st): an erased
 * page reads 0xFFFF everywhere except one 0x0000 word per block. That pattern is confirmed on the raw page (decoder
 * bypassed), so programmed data can never be mistaken for an erased page. */
static int ecc_page_looks_erased(const uint16_t *w) {
    for (uint32_t blk = 0; blk < PAGE_DATA_WORDS / ECC_BLK_WORDS; blk++) {
        uint32_t zeros = 0u;
        for (uint32_t i = blk * ECC_BLK_WORDS; i < (blk + 1u) * ECC_BLK_WORDS; i++) {
            if (w[i] == 0xFFFFu) continue;
            if (w[i] != 0x0000u || ++zeros > 1u) return 0;
        }
    }
    return 1;
}
/* raw page with the decoder bypassed (reg 0x0C[10]) all 0xFFFF: the check of the NAD-Flash self-test (words 0..1 of a
 * raw read are not checked there either). Slow (non-cached buffer), but only reached for erased-looking pages. */
static int ecc_page_raw_erased(uint32_t block, uint32_t page) {
    static uint16_t raw[PAGE_NAND_WORDS];
    uint32_t saved = g_dbg_ctrl;
    g_dbg_ctrl = saved | 0x400u; reg_wr32(NAND_DBG_REG, g_dbg_ctrl);
    __asm volatile("fence" ::: "memory");
    int r = raw_read_page(block, page, raw, PAGE_NAND_WORDS);
    g_dbg_ctrl = saved; reg_wr32(NAND_DBG_REG, g_dbg_ctrl);
    __asm volatile("fence" ::: "memory");
    if (r != 0) return 0;
    for (uint32_t i = 2u; i < PAGE_NAND_WORDS; i++) if (raw[i] != 0xFFFFu) return 0;
    return 1;
}

/* NV-DDR read check (BSP_nand_init): READ ID 90h 00h through the synchronous read path, decoder bypassed. In NV-DDR
 * each ID byte arrives on both DQS edges, so a word carries it twice (0x2C2C 0x8888 ... for Micron, as in the NAD-Flash
 * self-test). Short read, Stream2MM armed only after the PHY reported the data captured: a failing check cannot stall
 * the bus. The DQS / clock debug counters are cleared first (reg 0x0C[8]); windows 0..11 are recorded afterwards. */
static int      g_nvddr_check_rc = 0;                  /* 0 ID read, 1 no ID in the data, -6 PHY time-out (no DQS edges), -5 DMA */
static uint16_t g_nvddr_check_id[16];                  /* its READ ID words (decoder bypassed, one byte on each DQS edge) */
static uint32_t g_nvddr_check_dbg[12];                 /* PHY debug windows 0..11 right after it (DQS edges, captures, time-outs ...) */
static int nvddr_read_check(void) {
    uint32_t saved = g_dbg_ctrl;
    reg_wr32(NAND_DBG_REG, saved | 0x100u);                         /* clear the debug counters */
    __asm volatile("fence" ::: "memory");
    reg_wr32(NAND_DBG_REG, saved);
    __asm volatile("fence" ::: "memory");
    g_rd_timeouts = dbg_win(11u);                                   /* time-out counter mirror in step */
    for (uint32_t i = 0; i < 16u; i++) g_nvddr_check_id[i] = 0u;
    g_dbg_ctrl = saved | 0x400u; reg_wr32(NAND_DBG_REG, g_dbg_ctrl);   /* decoder bypass */
    __asm volatile("fence" ::: "memory");
    int r = raw_read_id(g_nvddr_check_id, 16u);
    g_dbg_ctrl = saved; reg_wr32(NAND_DBG_REG, g_dbg_ctrl);
    __asm volatile("fence" ::: "memory");
    for (uint32_t sel = 0; sel < 12u; sel++) g_nvddr_check_dbg[sel] = dbg_win(sel);
    if (r != 0) { g_nvddr_check_rc = r; return -1; }               /* -6: PHY time-out (no DQS edges), -5: DMA */
    for (uint32_t o = 0; o + 1u < 16u; o++)
        if ((g_nvddr_check_id[o] & 0xFFu) == 0x2Cu && (g_nvddr_check_id[o + 1u] & 0xFFu) == 0x88u) { g_nvddr_check_rc = 0; return 0; }
    g_nvddr_check_rc = 1;                                           /* read completed, no ID in the data */
    return -1;
}

/* one synchronous command + R/B# wait (e.g. FF = RESET in the NV-DDR protocol) */
static int sync_cmd(uint8_t cmd)
{
    if (ensure_initialized() != BSP_NAND_OK) return -1;
    cmd_reset();
    cmd_push(OP_BYTE_PACK(ONFI_OP_WAIT_TRHW, 0x00));
    cmd_push(OP_BYTE_PACK(ONFI_OP_SYNC_CMD,  cmd));
    cmd_push(OP_BYTE_PACK(ONFI_OP_WAIT_RB_N, 0x00));
    int rc = cmd_flush();
    for (uint32_t t = 0; (t < 2000000u) && reg_rd32(NAND_CMD_PENDING); t++) { }
    return rc;
}

/* ==================================================================== *
 *                           Initialization                             *
 * ==================================================================== */

/* NV-DDR set-up of every target in use (RESET, SET FEATURES timing mode, each waiting on its own R/B#), then the
 * synchronous clock. A target other than CE1 whose stream does not complete is dropped. Ends with CE1 selected. */
static bsp_nand_status_t do_init(void) {
    set_clock_async();
    for (uint32_t t = 0; t < BSP_NAND_NUM_TARGETS; t++) {
        if (!target_ok(t)) continue;
        nand_select(t);
        cmd_reset();
        for (size_t i = 0; i < C_INIT_SEQUENCE_LEN; i++) cmd_push(C_INIT_SEQUENCE[i]);
        int rc = cmd_flush();
        if (rc == 0 && wait_cmd_bounded() != 0) { phy_abort(); rc = -1; }   /* was an unbounded wait */
        if (rc != 0) {
            if (t == 0u) return BSP_NAND_ERR_INTERNAL;
            g_targets &= ~(1u << t);
        }
    }
    nand_select(0u);
    set_clock_sync();
    g_initialized = true;
    return BSP_NAND_OK;
}

/* little-endian 32-bit field of the ONFI parameter page */
static uint32_t part_param_u32(uint32_t off) {
    return (uint32_t)g_part_param[off] | ((uint32_t)g_part_param[off + 1u] << 8) |
           ((uint32_t)g_part_param[off + 2u] << 16) | ((uint32_t)g_part_param[off + 3u] << 24);
}
/* the target has the geometry the address map assumes: ONFI bytes 80 data bytes per page, 92 pages per block,
 * 96 blocks per LUN, 100 LUNs */
static int part_geometry_ok(void) {
    return (part_param_u32(80u) == NAND_PAGE_DATA_SIZE) && (part_param_u32(92u) == NAND_PAGES_PER_BLOCK) &&
           (part_param_u32(96u) == NAND_LUN_BLOCKS) && (g_part_param[100] == NAND_LUNS);
}

/* RESET and ONFI parameter page of the selected target through the SDR ops (every ONFI target powers up in the
 * asynchronous interface). Returns its NV-DDR support (features byte 6 bit 5): 1 yes, 0 no; -1 page not read; -2 the
 * page reports another geometry than the address map (part_geometry_ok). CE2..CE4 must also show that WAIT_RB_N
 * watches their own R/B#: only a PHY with the per-CE R/B# change routes it (and only with NAND_RB2_N..RB4_N
 * connected), otherwise it watches RB1 and returns at once there. RESET drives the target's R/B# busy, so its busy
 * pulse must appear in debug window 9 (falling edges of the watched R/B#); without one the result is -1. */
static int sdr_probe_part(void) {
    static uint16_t w[264];                                              /* was w[256]: the read below takes 264 words */
    int nvddr = -1;
    g_part_valid = 0;
    g_sdr_rd_timing = 0u;
    for (int attempt = 0; attempt < 3 && !g_part_valid; attempt++) {
        const uint32_t busy = dbg_win(DBG_WIN_RB) & 0x7FFFFFFFu;       /* R/B# falling edges so far */
        if (sdr_reset() != 0) { phy_abort(); continue; }
        if (g_ce != 0u && (dbg_win(DBG_WIN_RB) & 0x7FFFFFFFu) == busy) return -1;   /* not this target's R/B# */
        (void)sdr_resync();                                              /* drain stray words before the parameter page */
        if (sdr_read(0xECu, 1, 0x00u, 1, w, 264u) != 0) { phy_abort(); continue; }
        for (uint32_t off = 0; off <= 8u && !g_part_valid; off++) {        /* tolerate stray words in front */
            if ((w[off] & 0xFFu) == 'O' && (w[off + 1u] & 0xFFu) == 'N' && (w[off + 2u] & 0xFFu) == 'F' && (w[off + 3u] & 0xFFu) == 'I') {
                for (uint32_t i = 0; i < 256u; i++) g_part_param[i] = (uint8_t)(w[off + i] & 0xFFu);
                g_part_valid = 1;
                nvddr = (g_part_param[6] & 0x20u) ? 1 : 0;
            }
        }
    }
    if (g_part_valid && !part_geometry_ok()) return -2;
    return nvddr;
}

/* SDR fallback: stay asynchronous, route all access through the SDR ops, calibrate the fast read timing. Called with
 * CE1 selected; ends with CE1 selected. */
static bsp_nand_status_t sdr_fallback_init(int reason) {
    g_sdr_fallback = 1;
    g_fallback_reason = reason;
    (void)sdr_resync();
    (void)sdr_calibrate();                 /* keeps the slow engine if no robust fast timing exists */
    nand_select(0u);
    g_initialized = true;
    return BSP_NAND_OK;
}

/* Initialisation with interface detection. Default = the synchronous (NV-DDR) path exactly as before: RESET,
 * SET FEATURES 0x15, synchronous clock. When the PHY has the SDR read op, the ONFI parameter page is read first
 * (every ONFI device starts in the asynchronous interface); if the part reports no NV-DDR data interface
 * (features byte 6 bit 5 clear) the driver stays asynchronous and routes all access through the SDR path.
 * The NV-DDR path is used only if a synchronous READ ID then returns the ID: on the NV-DDR board (MT29F256G08AUCABH3)
 * with a v9/v10 PHY, never validated there, every synchronous read timed out in the PHY (BSP 1.4.0 NAND unit test),
 * while the SDR path of the same PHY is validated. In that case the part is reset back to the SDR interface (RESET FFh
 * in the NV-DDR protocol) and the SDR path is used; g_fallback_reason records it, g_nvddr_check_* hold the evidence.
 * Targets: CE1 always; CE2..CE4 on an SDR-capable PHY once sdr_probe_part() passes on them (own R/B#, the driver's
 * geometry). Every target in use goes through the same steps, and all of them stay in one interface: NV-DDR only if
 * it is supported and works on each.
 * With BSP_NAND_FORCE_SYNC defined neither fallback is taken: the parameter page is still read (target check,
 * geometry) and the READ ID check still runs (g_nvddr_check_*), but the NV-DDR path is used whatever they report. */
bsp_nand_status_t BSP_nand_init(void) {
    g_initialized = false;   /* allow explicit re-init */
    g_sdr_fallback = 0;
    g_fallback_reason = NAND_FALLBACK_NONE;
    g_part_valid = 0;
    g_sync_rx_ok = 0u;
    g_targets = 1u;
    nand_select(0u);
    g_phy_sdr = phy_has_sdr();
    g_phy_pack32 = g_phy_sdr ? phy_has_pack32() : 0;
    g_phy_unpack32 = g_phy_sdr ? phy_has_unpack32() : 0;
    g_phy_bch = g_phy_sdr ? phy_has_bch() : 0;
    /* write timing: the v9+ fast engine at 30 ns per byte (clean at every launch on both 64G units); older PHYs read
     * nof_bytes[31] differently and keep the legacy engine (45 ns). The mode-5 cycle SDR_FW(1, 1, 0) is an option. */
    g_sdr_wr_timing = g_phy_bch ? SDR_FW(2, 1, 1) : SDR_T(1, 2, 0, 0, 1);
    if (g_phy_sdr) {
        int nvddr = 1;
        for (uint32_t t = 0; t < BSP_NAND_NUM_TARGETS; t++) {
            nand_select(t);
            int r = sdr_probe_part();
            if (t == 0u && r == -2) { nand_select(0u); return BSP_NAND_ERR_INTERNAL; }   /* the map does not fit CE1 */
            if (t != 0u && r >= 0) g_targets |= 1u << t;
            if (r == 0 && target_ok(t)) nvddr = 0;                      /* CE1: -1 (page not read) tries NV-DDR, as before */
        }
        nand_select(0u);
        if (nvddr == 0 && !NAND_FORCE_SYNC) return sdr_fallback_init(NAND_FALLBACK_NO_NVDDR);
    }
    bsp_nand_status_t rc = do_init();
    if (rc != BSP_NAND_OK || !g_phy_sdr) return rc;
    int nvddr_ok = 1;
    for (uint32_t t = 0; nvddr_ok && t < BSP_NAND_NUM_TARGETS; t++) {
        if (!target_ok(t)) continue;
        nand_select(t);
        nvddr_ok = (nvddr_read_check() == 0);
    }
    nand_select(0u);
    if (nvddr_ok || NAND_FORCE_SYNC) return BSP_NAND_OK;      /* forced: a failed check is left in g_nvddr_check_* */
    /* NV-DDR reads do not work: every target back to the SDR interface, timing mode 0, and the SDR path */
    g_fallback_reason = NAND_FALLBACK_NVDDR_READ;
    for (uint32_t t = 0; t < BSP_NAND_NUM_TARGETS; t++) {
        if (!target_ok(t)) continue;
        nand_select(t);
        (void)sync_cmd(0xFFu);
    }
    g_initialized = false;
    g_sync_rx_ok = 0u;
    for (uint32_t t = 0; t < BSP_NAND_NUM_TARGETS; t++) {
        if (!target_ok(t)) continue;
        nand_select(t);
        if (sdr_probe_part() >= 0) continue;
        if (t == 0u) { nand_select(0u); return BSP_NAND_ERR_INTERNAL; }    /* the part does not answer in SDR either */
        g_targets &= ~(1u << t);
    }
    nand_select(0u);
    return sdr_fallback_init(NAND_FALLBACK_NVDDR_READ);
}

/* ==================================================================== *
 *                        Page-level NAND ops                           *
 * ==================================================================== *
 * Both interfaces store the hardware-ECC page layout written by ecc_write_page() (NV-DDR: 8 x (512 data + 10 Hamming
 * parity words); SDR: the same, or BCH-8 on a v9+ PHY). The baseline synchronous page ops that used to sit here
 * moved 8192 bytes as 2048 32-bit beats, while the os2 encoder and decoder carry one 16-bit word per beat: the encoder
 * got half the page and the read came back interleaved (BSP 1.4.0 NAND unit test on the NV-DDR board). The page data
 * travels as 4096 little-endian 16-bit words: word i = byte 2i (low) and byte 2i+1 (high). The ops take a block of
 * the device and work on the block of its target (nand_target). */

/* Selects the target of device block `block` and returns its block on that target: block b is block b % 8192 of
 * chip enable b / 8192 + 1, so CE1 holds the first 8 GB. */
static bsp_nand_status_t nand_target(uint32_t block, uint32_t *tblock) {
    const uint32_t t = block / NAND_TARGET_BLOCKS;
    if (t >= BSP_NAND_NUM_TARGETS) return BSP_NAND_ERR_OUT_OF_RANGE;
    if (!target_ok(t))             return BSP_NAND_ERR_NO_TARGET;
    if (t != g_ce) nand_select(t);
    *tblock = block % NAND_TARGET_BLOCKS;
    return BSP_NAND_OK;
}

/* Both page ops record their steps in g_prof_wr / g_prof_rd (BSP_NAND_PROFILE). */
static bsp_nand_status_t nand_program_page(uint32_t block, uint32_t page, const uint8_t *src) {
    static uint16_t words16[NAND_PAGE_DATA_SIZE / 2u] __attribute__((aligned(8)));
    nand_address_t a;
    uint32_t tb;
    prof_begin(&g_prof_wr);
    bsp_nand_status_t rc = nand_target(block, &tb);
    prof_mark("select target");
    if (rc == BSP_NAND_OK && calc_addr(tb, page, 0, &a) != 0) rc = BSP_NAND_ERR_OUT_OF_RANGE;
    if (rc == BSP_NAND_OK) {
        memcpy(words16, src, NAND_PAGE_DATA_SIZE);
        prof_mark("copy in from the caller's buffer");
        if (ecc_write_page(tb, page, words16) != 0) rc = BSP_NAND_ERR_WRITE_FAILED;
    }
    prof_end();
    return rc;
}

static bsp_nand_status_t nand_read_page(uint32_t block, uint32_t page, uint8_t *dst) {
    static uint16_t words16[NAND_PAGE_DATA_SIZE / 2u] __attribute__((aligned(8)));
    nand_address_t a;
    uint32_t tb;
    prof_begin(&g_prof_rd);
    bsp_nand_status_t rc = nand_target(block, &tb);
    prof_mark("select target");
    if (rc == BSP_NAND_OK && calc_addr(tb, page, 0, &a) != 0) rc = BSP_NAND_ERR_OUT_OF_RANGE;
    if (rc == BSP_NAND_OK && ecc_read_page(tb, page, words16) != 0) rc = BSP_NAND_ERR_READ_FAILED;
    if (rc == BSP_NAND_OK) {
        /* NV-DDR: the Hamming decoder does not pass an erased page through unchanged (see ecc_page_raw_erased()) */
        if (!g_sdr_fallback && ecc_page_looks_erased(words16) && ecc_page_raw_erased(tb, page))
            memset(words16, 0xFF, sizeof(words16));
        prof_mark("erased-page check");
        memcpy(dst, words16, NAND_PAGE_DATA_SIZE);
        prof_mark("copy out to the caller's buffer");
    }
    prof_end();
    return rc;
}

static bsp_nand_status_t nand_erase_block(uint32_t block) {
    nand_address_t a;
    uint32_t tb;
    bsp_nand_status_t rc = nand_target(block, &tb);
    if (rc != BSP_NAND_OK) return rc;
    if (calc_addr(tb, 0, 0, &a) != 0) return BSP_NAND_ERR_OUT_OF_RANGE;
    if (g_sdr_fallback) return (sdr_erase(tb) == 0) ? BSP_NAND_OK : BSP_NAND_ERR_ERASE_FAILED;

    cmd_reset();
    cmd_push(OP_BYTE_PACK(ONFI_OP_SYNC_CMD,     0x60));
    cmd_push(OP_BYTE_PACK(ONFI_OP_SYNC_ADDRESS, a.R1));
    cmd_push(OP_BYTE_PACK(ONFI_OP_SYNC_ADDRESS, a.R2));
    cmd_push(OP_BYTE_PACK(ONFI_OP_SYNC_ADDRESS, a.R3));
    cmd_push(OP_BYTE_PACK(ONFI_OP_SYNC_CMD,     0xD0));
    cmd_push(OP_BYTE_PACK(ONFI_OP_WAIT_RB_N,    0x00));
    if (cmd_flush() != 0) return BSP_NAND_ERR_ERASE_FAILED;
    wait_cmd();
    return BSP_NAND_OK;
}

/* ---- Param validation ---- */
static bsp_nand_status_t validate(uint64_t addr, uint64_t length) {
    if (length == 0u)                      return BSP_NAND_ERR_OUT_OF_RANGE;
    if (addr > NAND_DEVICE_SIZE || length > NAND_DEVICE_SIZE - addr) return BSP_NAND_ERR_OUT_OF_RANGE;
    return BSP_NAND_OK;
}

/* ==================================================================== *
 *                          PUBLIC NAND API                             *
 * ==================================================================== */

bsp_nand_status_t BSP_nand_read(void *dst, uint64_t addr, uint64_t length) {
    if (!dst) return BSP_NAND_ERR_INTERNAL;
    bsp_nand_status_t rc = validate(addr, length);
    if (rc != BSP_NAND_OK) return rc;
    rc = ensure_initialized();
    if (rc != BSP_NAND_OK) return rc;

    static uint8_t page_buf[NAND_PAGE_DATA_SIZE];
    uint8_t *out = (uint8_t *)dst;
    uint64_t cur = addr, remaining = length;

    while (remaining > 0) {
        uint32_t block = (uint32_t)(cur / NAND_BLOCK_SIZE);
        uint32_t page  = (uint32_t)((cur % NAND_BLOCK_SIZE) / NAND_PAGE_DATA_SIZE);
        uint32_t off   = (uint32_t)(cur % NAND_PAGE_DATA_SIZE);

        if (off == 0u && remaining >= NAND_PAGE_DATA_SIZE) {      /* whole page: straight into the caller's buffer */
            rc = nand_read_page(block, page, out);
            if (rc != BSP_NAND_OK) return rc;
            out += NAND_PAGE_DATA_SIZE; cur += NAND_PAGE_DATA_SIZE; remaining -= NAND_PAGE_DATA_SIZE;
            continue;
        }

        rc = nand_read_page(block, page, page_buf);
        if (rc != BSP_NAND_OK) return rc;

        uint32_t chunk = NAND_PAGE_DATA_SIZE - off;
        if (chunk > remaining) chunk = (uint32_t)remaining;
        memcpy(out, &page_buf[off], chunk);

        out += chunk; cur += chunk; remaining -= chunk;
    }
    return BSP_NAND_OK;
}

bsp_nand_status_t BSP_nand_write(const void *src, uint64_t addr, uint64_t length) {
    if (!src) return BSP_NAND_ERR_INTERNAL;
    bsp_nand_status_t rc = validate(addr, length);
    if (rc != BSP_NAND_OK) return rc;
    if (addr   % NAND_PAGE_DATA_SIZE) return BSP_NAND_ERR_ALIGNMENT;
    if (length % NAND_PAGE_DATA_SIZE) return BSP_NAND_ERR_ALIGNMENT;
    rc = ensure_initialized();
    if (rc != BSP_NAND_OK) return rc;

    rc = BSP_nand_erase(addr, length);
    if (rc != BSP_NAND_OK) return rc;

    return BSP_nand_program(src, addr, length);
}

/* Program into erased pages, no erase: appends (log records, parameter slots) keep the rest of the block. A partial
 * last page is padded with the erased value 0xFF; every page is programmed once per erase (no partial programming). */
bsp_nand_status_t BSP_nand_program(const void *src, uint64_t addr, uint64_t length) {
    if (!src) return BSP_NAND_ERR_INTERNAL;
    bsp_nand_status_t rc = validate(addr, length);
    if (rc != BSP_NAND_OK) return rc;
    if (addr % NAND_PAGE_DATA_SIZE) return BSP_NAND_ERR_ALIGNMENT;
    rc = ensure_initialized();
    if (rc != BSP_NAND_OK) return rc;

    static uint8_t pad_buf[NAND_PAGE_DATA_SIZE];
    const uint8_t *in = (const uint8_t *)src;
    uint64_t cur = addr, remaining = length;

    while (remaining > 0) {
        uint32_t block = (uint32_t)(cur / NAND_BLOCK_SIZE);
        uint32_t page  = (uint32_t)((cur % NAND_BLOCK_SIZE) / NAND_PAGE_DATA_SIZE);
        uint32_t chunk = NAND_PAGE_DATA_SIZE;
        const uint8_t *pg = in;
        if (remaining < NAND_PAGE_DATA_SIZE) {
            chunk = (uint32_t)remaining;
            memcpy(pad_buf, in, chunk);
            memset(&pad_buf[chunk], 0xFF, NAND_PAGE_DATA_SIZE - chunk);
            pg = pad_buf;
        }
        rc = nand_program_page(block, page, pg);
        if (rc != BSP_NAND_OK) return rc;
        in += chunk; cur += chunk; remaining -= chunk;
    }
    return BSP_NAND_OK;
}

bsp_nand_status_t BSP_nand_erase(uint64_t address, uint64_t length) {
    bsp_nand_status_t rc = validate(address, length);
    if (rc != BSP_NAND_OK) return rc;
    rc = ensure_initialized();
    if (rc != BSP_NAND_OK) return rc;

    uint32_t first = (uint32_t)(address / NAND_BLOCK_SIZE);
    uint32_t last  = (uint32_t)((address + length - 1u) / NAND_BLOCK_SIZE);
    for (uint32_t b = first; b <= last; b++) {
        rc = nand_erase_block(b);
        if (rc == BSP_NAND_ERR_NO_TARGET) return rc;
        if (rc != BSP_NAND_OK) return BSP_NAND_ERR_ERASE_FAILED;
    }
    return BSP_NAND_OK;
}
