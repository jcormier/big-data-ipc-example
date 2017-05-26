#
#  Copyright (c) 2017 Texas Instruments Incorporated - http://www.ti.com
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

EXBASE = ..
include $(EXBASE)/products.mak

srcs = MainDsp.c Server.c
objs = $(addprefix bin/$(PLATFORM)/$(PROFILE)/obj/,$(patsubst %.c,%.oe66,$(srcs)))
CONFIG = bin/$(PLATFORM)/$(PROFILE)/configuro

locallibsrcs = shared/bigdataxlat/bios/bigdataxlat.c
locallibobjs = $(addprefix bin/shared/$(PLATFORM)/$(PROFILE)/obj/,$(patsubst %.c,%.oe66,$(locallibsrcs)))

PKGPATH := $(BIOS_INSTALL_DIR)/packages
PKGPATH := $(PKGPATH)+$(IPC_INSTALL_DIR)/packages
PKGPATH := $(PKGPATH)+$(XDC_INSTALL_DIR)/packages

-include $(addprefix bin/$(PLATFORM)/$(PROFILE)/obj/,$(patsubst %.c,%.oe66.dep,$(srcs)))
-include $(addprefix bin/shared/$(PLATFORM)/$(PROFILE)/obj/,$(patsubst %.c,%.oe66.dep,$(locallibsrcs)))

.PRECIOUS: %/compiler.opt %/linker.cmd
.PHONY: release debug install install_rov server_dsp.x

all: release

debug:
	echo Building debug
	$(MAKE) PROFILE=debug server_dsp.x

release:
	$(MAKE) PROFILE=release server_dsp.x

server_dsp.x: bin/$(PLATFORM)/$(PROFILE)/server_dsp.xe66

bin/$(PLATFORM)/$(PROFILE)/server_dsp.xe66: $(objs) $(locallibobjs) $(libs) $(CONFIG)/linker.cmd
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
	$(LD) $(LDFLAGS) -o $@ $^ $(LDLIBS)

bin/$(PLATFORM)/$(PROFILE)/obj/%.oe66: %.c $(CONFIG)/compiler.opt
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
	$(CC) $(CPPFLAGS) $(CFLAGS) --output_file=$@ -fc $<

bin/shared/$(PLATFORM)/$(PROFILE)/obj/%.oe66: ../%.c $(CONFIG)/compiler.opt
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
	$(CC) $(CPPFLAGS) $(CFLAGS) --output_file=$@ -fc $<

%/compiler.opt %/linker.cmd: Dsp.cfg ../shared/$(PLATFORM)/config.bld
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
	$(XDC_INSTALL_DIR)/xs --xdcpath="$(subst +,;,$(PKGPATH))" \
            xdc.tools.configuro -o $(CONFIG) \
            -t ti.targets.elf.C66 \
            -c $(ti.targets.elf.C66) \
            -p $(DSP_PLATFORM) \
            -b ../shared/$(PLATFORM)/config.bld -r $(PROFILE) \
            --cfgArgs "{ \
                procnamelist: [$(PROC_NAME_LIST)], \
                procname: \"$(PROC_DSP_NAME)\", \
                profile: \"$(PROFILE)\" \
            }" \
            Dsp.cfg

install:
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
#	@$(MKDIR) $(EXEC_DIR)/debug
#	$(CP) bin/$(PLATFORM)/debug/server_dsp.xe66 $(EXEC_DIR)/debug
	@$(MKDIR) $(EXEC_DIR)/release
	$(CP) bin/$(PLATFORM)/release/server_dsp.xe66 $(EXEC_DIR)/release

install_rov:
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
	@$(MKDIR) $(EXEC_DIR)/release
	$(CP) bin/$(PLATFORM)/release/configuro/package/cfg/Dsp_pe66.rov.xs $(EXEC_DIR)/release

help:
	@$(ECHO) "make                   # build executable"
	@$(ECHO) "make clean             # clean everything"

clean::
	$(RMDIR) bin

#  ======== install validation ========
ifeq (install,$(MAKECMDGOALS))
ifeq (,$(EXEC_DIR))
$(error must specify EXEC_DIR)
endif
endif

#  ======== toolchain macros ========
CGTOOLS = $(ti.targets.elf.C66)

CC = $(CGTOOLS)/bin/cl6x -c
LD = $(CGTOOLS)/bin/cl6x -z

CPPFLAGS =
CFLAGS = -qq -pdsw225 -ppd=$@.dep -ppa $(CCPROFILE_$(PROFILE)) -@$(CONFIG)/compiler.opt -I.
CFLAGS += -I../shared/bigdataxlat/bios

LDFLAGS = -w -q -c -m $(@D)/obj/$(@F).map
LDLIBS = -l $(CGTOOLS)/lib/rts6600_elf.lib

CCPROFILE_debug = -D_DEBUG_=1 --symdebug:dwarf
CCPROFILE_release = -O2

#  ======== standard macros ========
ifneq (,$(wildcard $(XDC_INSTALL_DIR)/xdc.exe))
    # use these on Windows
    CP      = $(XDC_INSTALL_DIR)/bin/cp
    ECHO    = $(XDC_INSTALL_DIR)/bin/echo
    MKDIR   = $(XDC_INSTALL_DIR)/bin/mkdir -p
    RM      = $(XDC_INSTALL_DIR)/bin/rm -f
    RMDIR   = $(XDC_INSTALL_DIR)/bin/rm -rf
else
    # use these on Linux
    CP      = cp
    ECHO    = echo
    MKDIR   = mkdir -p
    RM      = rm -f
    RMDIR   = rm -rf
endif

#  ======== create output directories ========
ifneq (clean,$(MAKECMDGOALS))
ifneq (,$(PROFILE))
ifeq (,$(wildcard bin/$(PLATFORM)/$(PROFILE)/obj))
    $(shell $(MKDIR) -p bin/$(PLATFORM)/$(PROFILE)/obj)
endif
ifeq (,$(wildcard bin/shared/$(PLATFORM)/$(PROFILE)/obj/shared/bigdataxlat/bios))
    $(shell $(MKDIR) -p bin/shared/$(PLATFORM)/$(PROFILE)/obj/shared/bigdataxlat/bios)
endif
endif
endif
