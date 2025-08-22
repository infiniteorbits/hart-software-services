#
# MPFS HSS Embedded Software
#
# Copyright 2019-2022 Microchip FPGA Embedded Systems Solutions.
#
# SPDX-License-Identifier: MIT
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to
# deal in the Software without restriction, including without limitation the
# rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
# sell copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
# IN THE SOFTWARE.
#
#
# Defines target-specific build-rules variables, extra sources and include paths
#

$(info mpfs-icicle-kit-es selected)

TARGET-l2scratch=hss-l2scratch.elf
TARGET-envm-wrapper=hss-envm-wrapper.elf
RISCV_TARGET=$(TARGET-l2scratch) $(TARGET-envm-wrapper)
TARGET:=$(RISCV_TARGET)

LINKER_SCRIPT-l2scratch=boards/${BOARD}/hss-l2scratch.ld

BOARD_DIR=boards/mpfs-icicle-kit-es

INCLUDES += \
	$(CONAN_INCLUDE_DIRS_ICICLE_KIT_REF_DESIGN)
	# -I$(BINDIR)/$(BOARD_DIR)/ \
	# -I$(BOARD_DIR)/ \
	$(CONAN_INCLUDE_DIRS_ORBSIGHT2_PLATFORM) \

EXTRA_SRCS-y += \
	$(BOARD_DIR)/hss_uart_init.c \
        $(BOARD_DIR)/uart_device_list.c \
        $(BOARD_DIR)/hss_board_init.c \

EXTRA_SRCS-$(CONFIG_USE_LOGO) += \
        init/hss_logo_init.c

EXTRA_SRCS-$(CONFIG_SERVICE_HEALTHMON) += \
        $(BOARD_DIR)/healthmon_monitors.c \


$(BOARD_DIR)/hss_uart_init.o: CFLAGS=$(CFLAGS_GCCEXT)

ifndef CONFIG_SERVICE_BOOT_USE_PAYLOAD_IN_FABRIC
EXTRA_OBJS-$(CONFIG_SERVICE_BOOT_USE_PAYLOAD) += $(BOARD_DIR)/payload.o
$(BOARD_DIR)/payload.o: $(BOARD_DIR)/payload.bin
	$(LD) -r -b binary $< -o $@
else
endif


################################################################################################
#
# Linker Scripts
#

$(BOARD_DIR)/hss-l2scratch.ld: $(BOARD_DIR)/hss-l2scratch.lds $(CONFIG_H)

################################################################################################
