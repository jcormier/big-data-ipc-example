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
#  ======== Makefile ========
#
PLATFORM = DRA7XX

srcs = main_host.c App.c
locallibsrcs = shared/bigdataxlat/bigdataxlat.c \
  host/SharedRegion/src/SharedRegion.c \
  host/HeapMem/src/HeapMem.c \
  host/utils/List.c \
  host/utils/Memory.c \
  host/utils/Cache.c

EXBASE = ..
include $(EXBASE)/products.mak
-include $(addprefix bin/$(PROFILE)/obj/,$(patsubst %.c,%.ov7A.dep,$(srcs)))
-include $(addprefix bin/$(PLATFORM)/$(PROFILE)/obj/,$(patsubst %.c,%.ov7A.dep,$(locallibsrcs)))

objs = $(addprefix bin/$(PROFILE)/obj/,$(patsubst %.c,%.ov7A,$(srcs)))
locallibobjs = $(addprefix bin/$(PLATFORM)/$(PROFILE)/obj/,$(patsubst %.c,%.ov7A,$(locallibsrcs)))

ifdef LIBS_STATIC
libs = libtitransportrpmsg.a \
       libtiipc.a \
       libtiipcutils.a
else
# Use dynamic or shared libs
#libs = $(IPC_INSTALL_DIR)/linux/src/api/.libs/libtiipc.so \
#       $(IPC_INSTALL_DIR)/linux/src/utils/.libs/libtiipcutils.so \
#       $(IPC_INSTALL_DIR)/linux/src/transport/.libs/libtitransportrpmsg.so
libs =
endif

all:
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
#	$(MAKE) PROFILE=debug app_host
	$(MAKE) PROFILE=release app_host

help:
	@$(ECHO) "make                  # build executables"
	@$(ECHO) "make clean            # clean everything"

install:
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
#	@$(MKDIR) $(EXEC_DIR)/debug
#	$(CP) bin/debug/app_host $(EXEC_DIR)/debug
	@$(MKDIR) $(EXEC_DIR)/release
	$(CP) bin/$(PLATFORM)/release/app_host $(EXEC_DIR)/release

clean::
	$(RMDIR) bin

#
#  ======== rules ========
#
app_host: bin/$(PLATFORM)/$(PROFILE)/app_host
bin/$(PLATFORM)/$(PROFILE)/app_host: $(objs) $(locallibobjs) $(libs)
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
	$(LD) $(LDFLAGS) -o $@ $^ $(LDLIBS)

bin/$(PROFILE)/obj/%.ov7A: %.c
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $<

bin/$(PLATFORM)/$(PROFILE)/obj/%.ov7A: ../%.c
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $<

#  ======== install validation ========
ifeq (install,$(MAKECMDGOALS))
ifeq (,$(EXEC_DIR))
$(error must specify EXEC_DIR)
endif
endif

#  ======== toolchain macros ========
ifndef LINUX_SYSROOT_DIR
CC = $(TOOLCHAIN_PREFIX)gcc
AR = $(TOOLCHAIN_PREFIX)ar
LD = $(TOOLCHAIN_PREFIX)gcc
CFLAGS =
CPPFLAGS =
LDFLAGS = -L$(IPC_INSTALL_DIR)/linux/src/api/.libs/ \
    -L$(IPC_INSTALL_DIR)/linux/src/utils/.libs \
    -L$(IPC_INSTALL_DIR)/linux/src/transport/.libs \
    -L$(CMEM_INSTALL_DIR)/src/cmem/api/.libs
else
LD = $(CC)
endif

CFLAGS += -c -MD -MF $@.dep
ARFLAGS = cr

CPPFLAGS += -D_REENTRANT

CFLAGS += -Wall -ffloat-store -fPIC -Wunused -pthread -Dfar= $(CCPROFILE_$(PROFILE)) \
    -I. -I.. -I../shared/bigdataxlat -I../shared/bigdataxlat/linux -I../host/SharedRegion -I../host/SharedRegion/src/inc \
    -I../host/HeapMem -I../host/HeapMem/src/inc -I../host/utils

ifdef LINUX_SYSROOT_DIR
CFLAGS += -I$(LINUX_SYSROOT_DIR)
else
CFLAGS += -I$(IPC_INSTALL_DIR)/linux/include -I$(IPC_INSTALL_DIR)/packages -I$(CMEM_INSTALL_DIR)/include
endif

LDFLAGS += $(LDPROFILE_$(PROFILE)) -Wall -Wl,-Map=$@.map

LDLIBS = -lpthread -lc -lrt -lticmem
ifndef LIBS_STATIC
LDLIBS +=-ltiipc -ltiipcutils -ltitransportrpmsg
endif
CCPROFILE_debug = -ggdb -D DEBUG
CCPROFILE_release = -O3 -D NDEBUG

LDPROFILE_debug = -ggdb
LDPROFILE_release = -O3

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
ifeq (,$(wildcard bin/$(PROFILE)/obj))
    $(shell $(MKDIR) -p bin/$(PROFILE)/obj)
endif
ifeq (,$(wildcard bin/$(PLATFORM)/$(PROFILE)/obj/shared/bigdataxlat/linux))
    $(shell $(MKDIR) -p bin/$(PLATFORM)/$(PROFILE)/obj/shared/bigdataxlat/linux)
endif
ifeq (,$(wildcard bin/$(PLATFORM)/$(PROFILE)/obj/host/SharedRegion/src))
    $(shell $(MKDIR) -p bin/$(PLATFORM)/$(PROFILE)/obj/host/SharedRegion/src)
endif
ifeq (,$(wildcard bin/$(PLATFORM)/$(PROFILE)/obj/host/HeapMem/src))
    $(shell $(MKDIR) -p bin/$(PLATFORM)/$(PROFILE)/obj/host/HeapMem/src)
endif
ifeq (,$(wildcard bin/$(PLATFORM)/$(PROFILE)/obj/host/utils))
    $(shell $(MKDIR) -p bin/$(PLATFORM)/$(PROFILE)/obj/host/utils)
endif
endif
endif
