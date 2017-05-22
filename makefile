#
#  Copyright (c) 2017, Texas Instruments Incorporated
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#  *  Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#
#  *  Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#
#  *  Neither the name of Texas Instruments Incorporated nor the names of
#     its contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
#  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
#  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
#  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
#  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
#  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
#  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
#  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
#  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
#  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

#
#  ======== makefile ========
#

.PHONY:  host_bios host_bios/simple_buffer_example host_linux/simple_buffer_example

all: host_bios

host_bios: host_bios/simple_buffer_example

host_linux: host_linux/simple_buffer_example

host_bios/simple_buffer_example:
	@echo "#"
	@echo "# Making $@..."
	$(MAKE) -C $@ all

host_linux/simple_buffer_example:
	@echo "#"
	@echo "# Making $@..."
	$(MAKE) -C $@ all

help:
	@echo "make all        # build all examples"
	@echo "make host_bios  # build all host bios examples"
	@echo "make host_linux  # build all host linux examples"
	@echo "make clean      # clean all examples (does not delete them)"
	@echo "make       # clean all examples (does not delete them)"
	@echo ""

install_bin::
	@echo "#"
	@echo "# Installing host_bios examples..."
	$(MAKE) -C host_bios/simple_buffer_example install

install_linux_bin::
	@echo "#"
	@echo "# Installing host_linux examples..."
	$(MAKE) -C host_linux/simple_buffer_example install

install_rov::
	@echo "#"
	@echo "# Installing host_bios examples..."
	$(MAKE) -C host_bios/simple_buffer_example install_rov


clean:: clean_bios clean_linux

clean_bios::
	@echo "#"
	@echo "# Cleaning host_bios examples..."
	$(MAKE) -C host_bios/simple_buffer_example clean

clean_linux::
	@echo "#"
	@echo "# Cleaning host_linux examples..."
	$(MAKE) -C host_linux/simple_buffer_example clean
