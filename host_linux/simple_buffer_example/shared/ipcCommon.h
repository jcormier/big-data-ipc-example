/*
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== ipcCommon.h ========
 *
 */

#ifndef ipcCommon__include
#define ipcCommon__include
#if defined (__cplusplus)
extern "C" {
#endif
#include "bigdataxlat.h"


/*
 *  ======== Application Configuration ========
 */

/* notify commands 00 - FF */
#define App_CMD_MASK                   0xFF000000
#define App_CMD_NOP                    0x00000000
#define App_CMD_SHARED_REGION_INIT     0x00000001
#define App_CMD_BIGDATA                0x00000002
#define App_CMD_SHUTDOWN               0x02000000

#define HIGH_SPEED_NUMBER_OF_BUFFERS    16                                  // # of buffers avail in shared mem
#define HIGH_SPEED_INTS_PER_BUFFER      (1024)                              // Size of each buffer in words
#define STREAMING_BUFFER_SIZE           (HIGH_SPEED_INTS_PER_BUFFER * 4)   // Size of each buffer in bytes

#define NUM_BUFFERS_TO_TEST             (HIGH_SPEED_NUMBER_OF_BUFFERS*20)

typedef struct {
    UInt64              base;
    UInt64              size;
} SharedRegionInitCfg;

typedef struct {
    MessageQ_MsgHeader  reserved;
    UInt32              cmd;
    Int32               id;
    UInt16              regionId;

    union {
        SharedRegionInitCfg sharedRegionInitCfg;
        bigDataSharedDesc_t bigDataSharedDesc;
    } u;
} App_Msg;

// Cache size is 128B (0x80), ensure each part is on a different cache line so ARM/DSP don't overwrite each other
typedef struct {
    Int32               dspBuffPtr; // 4B
    char                padding_dsp[124];

    Int32               armBuffPtr; // 4B @ 0x0080
    char                padding_arm[124];

    /* Each bufferFilled is 128B aligned to line up with cache size, since both ARM and DSP write to these addresses */
    UInt8               bufferFilled[HIGH_SPEED_NUMBER_OF_BUFFERS][128]; // 2048B @ 0x0100

    Int32               buffer[HIGH_SPEED_NUMBER_OF_BUFFERS][HIGH_SPEED_INTS_PER_BUFFER]; // 65,536B @ 0x0900
} Shared_Mem; // Total size: 67,840 (0x1_0900)

#define App_MsgHeapId           0
#define App_HostMsgQueName      "HOST:MsgQ:01"
#define App_SlaveMsgQueName     "%s:MsgQ:01"  /* %s is each slave's Proc Name */

#define BIGDATA_BUF_SIZE 16384

#define BIGDATA_BUF_ALIGN 1

#if defined (__cplusplus)
}
#endif /* defined (__cplusplus) */
#endif /* ipcCommon__include */
