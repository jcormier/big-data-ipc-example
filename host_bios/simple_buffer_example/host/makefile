#
#  Copyright (c) 2012-2014 Texas Instruments Incorporated - http://www.ti.com
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

srcs = MainHost.c App.c
objs = $(addprefix bin/$(BOARD_NAME)/$(PROFILE)/obj/,$(patsubst %.c,%.oa15fg,$(srcs)))
CONFIG = bin/$(BOARD_NAME)/$(PROFILE)/configuro

HOSTOS = bios
locallibsrcs = shared/bigdataxlat/bios/bigdataxlat.c
locallibobjs = $(addprefix bin/shared/$(PLATFORM)/$(PROFILE)/obj/,$(patsubst %.c,%.oa15fg,$(locallibsrcs)))

-include $(addprefix bin/$(BOARD_NAME)/$(PROFILE)/obj/,$(patsubst %.c,%.oa15fg.dep,$(srcs)))
-include $(addprefix bin/shared/$(PLATFORM)/$(PROFILE)/obj/,$(patsubst %.c,%.oa15fg.dep,$(locallibsrcs)))

.PRECIOUS: %/compiler.opt %/linker.cmd

.PHONY: release debug install install_rov

all: release
#all: debug

release:
	$(MAKE) PROFILE=release app_host.x

debug:
	$(MAKE) PROFILE=debug app_host.x

app_host.x: bin/$(BOARD_NAME)/$(PROFILE)/app_host.xa15fg
bin/$(BOARD_NAME)/$(PROFILE)/app_host.xa15fg: $(objs) $(locallibobjs) $(libs) $(CONFIG)/linker.cmd
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
	$(LD) -o $@ $(LDFLAGS) $(objs) $(locallibobjs) $(libs) -Wl,-T,$(CONFIG)/linker.cmd \
		-lgcc -lc -lm -lnosys $(LDLIBS)

bin/$(BOARD_NAME)/$(PROFILE)/obj/%.oa15fg: %.c $(CONFIG)/compiler.opt
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $<

bin/shared/$(PLATFORM)/$(PROFILE)/obj/%.oa15fg: ../%.c $(CONFIG)/compiler.opt
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $<

%/compiler.opt: %/linker.cmd ;
%/linker.cmd: Host.cfg ../shared/$(PLATFORM)/config.bld
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
	$(XDC_INSTALL_DIR)/xs --xdcpath="$(subst +,;,$(PKGPATH))" \
            xdc.tools.configuro -o $(CONFIG) \
            -t gnu.targets.arm.A15F \
            -c $(gnu.targets.arm.A15F) \
            -p $(HOST_PLATFORM) \
            -b ../shared/$(PLATFORM)/config.bld -r release \
            --cfgArgs "{ \
                procnamelist: [$(PROC_NAME_LIST)], \
                procname: \"$(PROC_HOST_NAME)\", \
                boardname: \"$(BOARD_NAME)\", \
                profile: \"$(PROFILE)\" \
            }" \
	    Host.cfg

install:
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
#	@$(MKDIR) $(EXEC_DIR)/debug
#	$(CP) bin/$(PLATFORM)/debug/app_host.xa15fg $(EXEC_DIR)/debug
	@$(MKDIR) $(EXEC_DIR)/release
	$(CP) bin/$(BOARD_NAME)/release/app_host.xa15fg $(EXEC_DIR)/release

install_rov:
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
	@$(MKDIR) $(EXEC_DIR)/release
	$(CP) bin/$(BOARD_NAME)/release/configuro/package/cfg/Host_pa15fg.rov.xs $(EXEC_DIR)/release

help:
	@$(ECHO) "make                   # build executable"
	@$(ECHO) "make clean             # clean everything"

clean::
	$(RMDIR) bin

PKGPATH := $(BIOS_INSTALL_DIR)/packages
PKGPATH := $(PKGPATH)+$(IPC_INSTALL_DIR)/packages
PKGPATH := $(PKGPATH)+$(XDC_INSTALL_DIR)/packages
PKGPATH := $(PKGPATH)+$(PDK_INSTALL_DIR)


#  ======== install validation ========
ifeq (install,$(MAKECMDGOALS))
ifeq (,$(EXEC_DIR))
$(error must specify EXEC_DIR)
endif
endif

#  ======== toolchain macros ========
CC = $(gnu.targets.arm.A15F)/bin/arm-none-eabi-gcc -c -MD -MF $@.dep
LD = $(gnu.targets.arm.A15F)/bin/arm-none-eabi-gcc

CPPFLAGS = -Dfar= -D__DYNAMIC_REENT__
CFLAGS = -Wall -Wunused -Wunknown-pragmas -ffunction-sections -fdata-sections $(CCPROFILE_$(PROFILE)) @$(CONFIG)/compiler.opt -I.
CFLAGS += -I../shared/bigdataxlat/bios
CFLAGS += -DSERVER=\"$(SERVER)\" -DPLATFORM_$(PLATFORM)

LDFLAGS = $(LDPROFILE_$(PROFILE)) -mfloat-abi=hard -nostartfiles -Wl,-static -Wl,--gc-sections -Wl,-Map=$(@D)/obj/$(@F).map
LDLIBS = -L$(BIOS_INSTALL_DIR)/packages/gnu/targets/arm/libs/install-native/arm-none-eabi/lib/fpu

CCPROFILE_debug = -g -ggdb -D_DEBUG_=1
CCPROFILE_release = -O2

LDPROFILE_debug = -g -ggdb
LDPROFILE_release =

#  ======== standard macros ========
ifneq (,$(wildcard $(XDC_INSTALL_DIR)/bin/echo.exe))
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
ifeq (,$(wildcard bin/$(BOARD_NAME)/$(PROFILE)/obj))
    $(shell $(MKDIR) -p bin/$(BOARD_NAME)/$(PROFILE)/obj)
endif
ifeq (,$(wildcard bin/shared/$(PLATFORM)/$(PROFILE)/obj/shared/bigdataxlat/bios))
    $(shell $(MKDIR) -p bin/shared/$(PLATFORM)/$(PROFILE)/obj/shared/bigdataxlat/bios)
endif
endif
endif
