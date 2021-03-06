#
#  Copyright (c) 2012-2017 Texas Instruments Incorporated - http://www.ti.com
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

# edit PROCLIST list to control how many executables to build
PLATFORM = DRA7XX

ifeq ($(PLATFORM),DRA7XX)
PROCLIST = host dsp
#PROCLIST = host ipu
PROC_HOST_NAME = "HOST"
PROC_DSP_NAME = "DSP1"
#PROC_IPU_NAME = "IPU1"
PROC_NAME_LIST = "\\\"$(PROC_HOST_NAME)\\\", \\\"IPU1\\\", \\\"IPU2\\\", \\\"DSP2\\\", \\\"DSP1\\\""
DSP_PLATFORM = ti.platforms.evmDRA7XX:dsp
IPU_PLATFORM = ti.platforms.evmDRA7XX:ipu
SERVER = "$(PROC_DSP_NAME)"
else ifeq ($(PLATFORM),AM574x)
PROCLIST = host dsp
#PROCLIST = host ipu
PROC_HOST_NAME = "HOST"
PROC_DSP_NAME = "DSP1"
#PROC_IPU_NAME = "IPU1"
PROC_NAME_LIST = "\\\"$(PROC_HOST_NAME)\\\", \\\"IPU1\\\", \\\"IPU2\\\", \\\"DSP2\\\", \\\"DSP1\\\""
DSP_PLATFORM = ti.platforms.evmDRA7XX:dsp
IPU_PLATFORM = ti.platforms.evmDRA7XX:ipu
SERVER = "$(PROC_DSP_NAME)"
else ifeq ($(PLATFORM),TCI6638)
PROCLIST = host dsp
PROC_HOST_NAME = "HOST"
PROC_DSP_NAME = "CORE0"
PROC_NAME_LIST = "\\\"$(PROC_HOST_NAME)\\\", \\\"CORE0\\\", \\\"CORE1\\\", \\\"CORE2\\\", \\\"CORE3\\\", \\\"CORE4\\\", \\\"CORE5\\\", \\\"CORE6\\\", \\\"CORE7\\\""
DSP_PLATFORM = ti.platforms.evmTCI6638K2K:dsp
SERVER = "$(PROC_DSP_NAME)"
else ifeq ($(PLATFORM),TCI6636)
PROCLIST = host dsp
PROC_HOST_NAME = "HOST"
PROC_DSP_NAME = "CORE0"
PROC_NAME_LIST = "\\\"$(PROC_HOST_NAME)\\\", \\\"CORE0\\\", \\\"CORE1\\\", \\\"CORE2\\\", \\\"CORE3\\\", \\\"CORE4\\\", \\\"CORE5\\\", \\\"CORE6\\\", \\\"CORE7\\\""
DSP_PLATFORM = ti.platforms.evmTCI6636K2H:dsp
SERVER = "$(PROC_DSP_NAME)"
else ifeq ($(PLATFORM),66AK2G)
PROCLIST = host dsp
PROC_HOST_NAME = "HOST"
PROC_DSP_NAME = "CORE0"
PROC_NAME_LIST = "\\\"$(PROC_HOST_NAME)\\\", \\\"CORE0\\\""
DSP_PLATFORM = ti.platforms.evmTCI66AK2G02:dsp
SERVER = "$(PROC_DSP_NAME)"
else ifeq ($(PLATFORM),66AK2E)
PROCLIST = host dsp
PROC_HOST_NAME = "HOST"
PROC_DSP_NAME = "CORE0"
PROC_NAME_LIST = "\\\"$(PROC_HOST_NAME)\\\", \\\"CORE0\\\""
DSP_PLATFORM = ti.platforms.evmC66AK2E:dsp
SERVER = "$(PROC_DSP_NAME)"
else ifeq ($(PLATFORM),TCI6630)
PROCLIST = host dsp
PROC_HOST_NAME = "HOST"
PROC_DSP_NAME = "CORE0"
PROC_NAME_LIST = "\\\"$(PROC_HOST_NAME)\\\", \\\"CORE0\\\", \\\"CORE1\\\", \\\"CORE2\\\", \\\"CORE3\\\""
DSP_PLATFORM = ti.platforms.evmTCI6630K2L:dsp
SERVER = "$(PROC_DSP_NAME)"
endif

EXBASE = .
include $(EXBASE)/products.mak
.PHONY: $(PROCLIST)

.PHONY: install

all: $(PROCLIST)

$(PROCLIST):
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
	$(MAKE) -C $@ PROCLIST="$(PROCLIST)" \
		DSP_PLATFORM=$(DSP_PLATFORM) \
		PROC_HOST_NAME="$(PROC_HOST_NAME)" \
		PROC_DSP_NAME="$(PROC_DSP_NAME)" \
		PROC_NAME_LIST=$(PROC_NAME_LIST) \
		SERVER="$(SERVER)"
help:
	@$(ECHO) "make                                  # build executables"
	@$(ECHO) "make clean                            # clean everything"
	@$(ECHO) "make install EXEC_DIR=/.../testbench  # install folder"

# setup install goal
ifeq ($(filter $(MAKECMDGOALS),install install_rov),$(MAKECMDGOALS))
ifeq (,$(EXEC_DIR))
EXEC_DIR=$(CURDIR)/install
endif
override EXEC_DIR:=$(EXEC_DIR)/simple_buffer_example
endif

install: $(PROCLIST) $(addsuffix _install,$(PROCLIST))
$(addsuffix _install,$(PROCLIST)):
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
#	@$(MKDIR) $(EXEC_DIR)/debug
	@$(MKDIR) $(EXEC_DIR)/release
	$(MAKE) -C $(subst _install,,$@) EXEC_DIR=$(EXEC_DIR) install

install_rov: $(PROCLIST) $(addsuffix _install_rov,$(PROCLIST))
$(addsuffix _install_rov,$(PROCLIST)):
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
#	@$(MKDIR) $(EXEC_DIR)/debug
	@$(MKDIR) $(EXEC_DIR)/release
	$(MAKE) -C $(subst _install_rov,,$@) EXEC_DIR=$(EXEC_DIR) install_rov

clean: $(addsuffix _clean,$(PROCLIST))
	$(RMDIR) install

$(addsuffix _clean,$(PROCLIST)):
	$(MAKE) -C $(subst _clean,,$@) clean

#  ======== standard macros ========
ifneq (,$(wildcard $(XDC_INSTALL_DIR)/xdc.exe))
    # use these on Windows
    CP      = $(XDC_INSTALL_DIR)/bin/cp
    ECHO    = $(XDC_INSTALL_DIR)/bin/echo
    INSTALL = $(XDC_INSTALL_DIR)/bin/cp
    MKDIR   = $(XDC_INSTALL_DIR)/bin/mkdir -p
    RM      = $(XDC_INSTALL_DIR)/bin/rm -f
    RMDIR   = $(XDC_INSTALL_DIR)/bin/rm -rf
else
    # use these on Linux
    CP      = cp
    ECHO    = echo
    INSTALL = install
    MKDIR   = mkdir -p
    RM      = rm -f
    RMDIR   = rm -rf
endif
