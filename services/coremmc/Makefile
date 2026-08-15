#
# MPFS HSS Embedded Software
#
# Copyright 2026 RFIM Space.
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
# CoreMMC Service - secondary eMMC on the fabric CoreMMC instance
# (NOTE: not a traditional super-loop service)
#
# The vendor CoreMMC driver is built here rather than from the board
# Makefile: it has no users outside this service, and its register access
# goes through the same HAL HW_get/set_*bit_reg helpers CoreQSPI uses
# (implemented in assembly in the bare-metal library HAL, added to ASM_SRCS
# by the board Makefile).

EXTRA_SRCS-$(CONFIG_SERVICE_COREMMC) += \
	services/coremmc/coremmc_api.c \
	baremetal/drivers/fpga_ip/CoreMMC/core_mmc.c \
	baremetal/drivers/fpga_ip/CoreMMC/core_mmc_if.c \

INCLUDES +=\
	-Iservices/coremmc \
	-Ibaremetal/drivers/fpga_ip/CoreMMC \

$(BINDIR)/services/coremmc/coremmc_api.o: CFLAGS=$(CFLAGS_GCCEXT)
$(BINDIR)/baremetal/drivers/fpga_ip/CoreMMC/core_mmc.o: CFLAGS=$(CFLAGS_GCCEXT)
$(BINDIR)/baremetal/drivers/fpga_ip/CoreMMC/core_mmc_if.o: CFLAGS=$(CFLAGS_GCCEXT)
