/*******************************************************************************
 * HSS Boot Logo (ASCII): User-provided orbsight-2/OrbSight block-art
 * SPDX-License-Identifier: MIT
 ******************************************************************************/

#include "config.h"
#include "hss_types.h"
#include "hss_init.h"
#include "hss_debug.h"

enum Token { CRLF_token = 0,
    LINE_0,
    LINE_1,
    LINE_2,
    LINE_3,
    LINE_4,
    LINE_5,
    LINE_6,
    LINE_7,
    LINE_8,
    LINE_9,
    LINE_10,
    LINE_11,
    LINE_12,
    LINE_13,
    LINE_14,
    LINE_15,
    LINE_16,
    LINE_17,
    LINE_18,
    LINE_19,
    LINE_20,
    LINE_21,
    LINE_22,
    LINE_23,
    LINE_24,
    LINE_25,
    LINE_26,
    LINE_27,
    LINE_28,
    LINE_29,
    LINE_30,
    LINE_31,
    LINE_32,
    LINE_33,
    LINE_34,
    LINE_35,
    LINE_36,
    LINE_37,
    LINE_38,
    LINE_39,
    LINE_40,
    LINE_41,
    LINE_42,
    LINE_43,
    LINE_44,
    LINE_45,
    LINE_46,
    LINE_47,
    LINE_48,
    LINE_49,
    LINE_50,
    LINE_51,
    LINE_52,
    LINE_53,
    LINE_54,
};

#if 0
static const char* tokenStringTable[] = {
    "\n",
    "                                                                                                    ",
    "                                                                                                    ",
    "                                                                                                    ",
    "                                                                                                    ",
    "                                                                                                    ",
    "                                                                                                    ",
    "                                                 %                                                  ",
    "                        -+               ++++++++++++++++++           -                             ",
    "                        -            +++++++*        ++++++++++                                     ",
    "                                  +++++                   %+++%                                     ",
    "                                +++%                                                                ",
    "                              +++                                 ----           :--:               ",
    "                            *++                                 --------                            ",
    "                           ++*                     %          -----------=                          ",
    "                          ++           :                      +----------                           ",
    "                         ++            -                        -------    **                       ",
    "                        *+          =------                 ===+  ---#    *****                     ",
    "                        *             :--                   ===-        ********%                   ",
    "                       +               -                                 *******                    ",
    "                       %                                                   ***                      ",
    "                              --%                                                                   ",
    "                                                     --                -                            ",
    "                                                                       +                            ",
    "                                                                                                    ",
    "                                                                                                    ",
    "                 @@@@@@@@@@@@       @@@@@@@@@@@@    @@@@@    @@@@@           @@@@@                  ",
    "                 @@@@@@@@@@@@@@     @@@@@@@@@@@@    :@@@@    @@@@@@         @@@@@@                  ",
    "                 @@@@      @@@@@    @@@@            @@@@@    @@@@@@@       @@@@@@@                  ",
    "                 @@@@      %@@@@    @@@@            @@@@@    @@@@@@@@     @@@@@@@@                  ",
    "                 @@@@      @@@@@    @@@@@@@@@@@     @@@@@    @@@@ @@@@   @@@@ @@@@                  ",
    "                 @@@@@@@@@@@@@@     @@@@@@@@@@@     @@@@@    @@@@  @@@@ @@@@  @@@@                  ",
    "                 @@@@@@@@@@@        @@@@            @@@@@    @@@@   @@@@@@@   @@@@                  ",
    "                 @@@@    @@@@       @@@@            @@@@@    @@@@    @@@@@    @@@@                  ",
    "                 @@@@     @@@@      @@@@            @@@@@    @@@@     @@@     @@@@                  ",
    "                 @@@@      @@@@     @@@@            @@@@@    @@@@             @@@@                  ",
    "                 @@@@       @@@@    @@@@            @@@@:    @@@@             @@@@                  ",
    "                                                                                                    ",
    "                                                                                                    ",
    "                          ---#     -----       --         ----      -----                           ",
    "                        *-         -   --     -*-       --          --                              ",
    "                         -===      -**--     -- --      -*          -----                           ",
    "                             #-    -        +---=--     --          --                              ",
    "                         ====      -       -=     -=      ----      -----                           ",
    "                                                                                                    ",
    "                                         orbsight-2 Space Ltd                                             ",
    "                                                                                                    ",
    "                                                                                                    ",
    "                                                                                                    ",
    "                                                                                                    ",
    "                                                                                                    ",
    "                                                                                                    ",
    "                                                                                                    ",
    "                                                                                                    ",
    "                                                                                                    ",
    "                                                                                                    "
};

#else

static const char* tokenStringTable[] = {
        "\n",
        "\033[38;5;80m _____  _____ ____    __ __  ____  __  __ _____    __ __ 2",
        "((   )) ||_// ||=)   ((  || (( ___ ||==||  ||      \\\\ //  ",
        " \\\\_//  || \\\\ ||_)) \\_)) ||  \\\\_|| ||  ||  ||       \\V/   \033[0m",
        "                                                          ",
        "                                                          "
};

#endif

struct __attribute__((packed)) RLEElement {
    unsigned char count;
    enum Token tokenIndex;
};

static const struct RLEElement rleLogoElements[] = {
    {1, LINE_0}, {1, CRLF_token},
    {1, LINE_1}, {1, CRLF_token},
    {1, LINE_2}, {1, CRLF_token},
    {1, LINE_3}, {1, CRLF_token},
    {1, LINE_4}, {1, CRLF_token},
#if 0
    {1, LINE_5}, {1, CRLF_token},
    {1, LINE_6}, {1, CRLF_token},
    {1, LINE_7}, {1, CRLF_token},
    {1, LINE_8}, {1, CRLF_token},
    {1, LINE_9}, {1, CRLF_token},
    {1, LINE_10}, {1, CRLF_token},
    {1, LINE_11}, {1, CRLF_token},
    {1, LINE_12}, {1, CRLF_token},
    {1, LINE_13}, {1, CRLF_token},
    {1, LINE_14}, {1, CRLF_token},
    {1, LINE_15}, {1, CRLF_token},
    {1, LINE_16}, {1, CRLF_token},
    {1, LINE_17}, {1, CRLF_token},
    {1, LINE_18}, {1, CRLF_token},
    {1, LINE_19}, {1, CRLF_token},
    {1, LINE_20}, {1, CRLF_token},
    {1, LINE_21}, {1, CRLF_token},
    {1, LINE_22}, {1, CRLF_token},
    {1, LINE_23}, {1, CRLF_token},
    {1, LINE_24}, {1, CRLF_token},
    {1, LINE_25}, {1, CRLF_token},
    {1, LINE_26}, {1, CRLF_token},
    {1, LINE_27}, {1, CRLF_token},
    {1, LINE_28}, {1, CRLF_token},
    {1, LINE_29}, {1, CRLF_token},
    {1, LINE_30}, {1, CRLF_token},
    {1, LINE_31}, {1, CRLF_token},
    {1, LINE_32}, {1, CRLF_token},
    {1, LINE_33}, {1, CRLF_token},
    {1, LINE_34}, {1, CRLF_token},
    {1, LINE_35}, {1, CRLF_token},
    {1, LINE_36}, {1, CRLF_token},
    {1, LINE_37}, {1, CRLF_token},
    {1, LINE_38}, {1, CRLF_token},
    {1, LINE_39}, {1, CRLF_token},
    {1, LINE_40}, {1, CRLF_token},
    {1, LINE_41}, {1, CRLF_token},
    {1, LINE_42}, {1, CRLF_token},
    {1, LINE_43}, {1, CRLF_token},
    {1, LINE_44}, {1, CRLF_token},
    {1, LINE_45}, {1, CRLF_token},
    {1, LINE_46}, {1, CRLF_token},
    {1, LINE_47}, {1, CRLF_token},
    {1, LINE_48}, {1, CRLF_token},
    {1, LINE_49}, {1, CRLF_token},
    {1, LINE_50}, {1, CRLF_token},
    {1, LINE_51}, {1, CRLF_token},
    {1, LINE_52}, {1, CRLF_token},
    {1, LINE_53}, {1, CRLF_token},
    {1, LINE_54}, {1, CRLF_token},
#endif
};

bool HSS_LogoInit(void) {
    for (unsigned int i = 0u; i < (sizeof(rleLogoElements)/sizeof(rleLogoElements[0])); i++) {
        for (unsigned char j = 0u; j < rleLogoElements[i].count; j++) {
            mHSS_PUTS(tokenStringTable[rleLogoElements[i].tokenIndex]);
        }
    }
    return true;
}
