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
 *  ======== Server.c ========
 *
 */

/* this define must precede inclusion of any xdc header file */
#define Registry_CURDESC Test__Desc
#define MODULE_NAME "Server"

/* xdctools header files */
#include <xdc/std.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Diags.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/Registry.h>

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

/* package header files */
#include <ti/ipc/MessageQ.h>
#include <ti/ipc/MultiProc.h>
#include <ti/ipc/SharedRegion.h>
#include <ti/ipc/HeapMemMP.h>
#include <ti/ipc/remoteproc/Resource.h>
#include <ti/sysbios/hal/Cache.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/Memory.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

// local defines
//   #define NEXT_GEN_STREAMING_SERVICE        // Un-comment to use next gen


/* local header files */
#include "../shared/ipcCommon.h"

/* module header file */
#include "../shared/dsp.h"

/* module structure */
typedef struct {
    UInt16              hostProcId;         // host processor id
    MessageQ_Handle     slaveQue;           // created locally
} Server_Module;

/* private data */
Registry_Desc               Registry_CURDESC;
static Server_Module        Module;


/*
 *  ======== Server_init ========
 */
Void Server_init(Void)
{
    Registry_Result result;

    /* register with xdc.runtime to get a diags mask */
    result = Registry_addModule(&Registry_CURDESC, MODULE_NAME);
    Assert_isTrue(result == Registry_SUCCESS, (Assert_Id)NULL);

    /* initialize module object state */
    Module.hostProcId = MultiProc_getId("HOST");
}

/*
 *  ======== Server_create ========
 */
Int Server_create()
{
    Int                 status = 0;
    MessageQ_Params     msgqParams;
    char                msgqName[32];

    /* enable some log events */
    Diags_setMask(MODULE_NAME"+EXF");

    /* create local message queue (inbound messages) */
    MessageQ_Params_init(&msgqParams);
    sprintf(msgqName, App_SlaveMsgQueName, MultiProc_getName(MultiProc_self()));
    Module.slaveQue = MessageQ_create(msgqName, &msgqParams);

    if (Module.slaveQue == NULL) {
        status = -1;
        goto leave;
    }

    Log_print0(Diags_INFO,"Server_create: server is ready");

leave:
    Log_print1(Diags_EXIT, "<-- Server_create: %d", (IArg)status);
    return (status);
}

/*
 *  ======== Server_exec ========
 */
Int Server_exec()
{
    Int                 dspCtr = 0;
    Int                 i, j, k;
    Int                 recPtr = 0;
    Int                 retVal;
    Int                 status;
    Int32               *streamingBuffer;

    float               dlyVal = 0.0;
    int                 dlyCtr = 0;

    Shared_Mem          *shmem;

    clock_t             startTime;

    Bool                running   = TRUE;
    Bool                firstPass = TRUE;

    App_Msg *           msg;
    MessageQ_QueueId    queId;
    UInt16              regionId1  = 1;
    UInt32              errorCount = 0;
    bigDataLocalDesc_t  bigDataLocalDesc;
    SharedRegion_Entry  srEntry;
    void                *sharedRegionAllocPtr = NULL;

//    struct timespec {        ts;

    Log_print0(Diags_ENTRY | Diags_INFO, "--> Server_exec-C:");

//    startTime = clock();
//    clock_gettime (CLOCK_REALTIME, &ts);
//    startTime = ts.tv_nsec;
//    startTime = Clock_getTicks();
//    startTime = -1;

    streamingBuffer = (Int32 *) malloc(HIGH_SPEED_INTS_PER_BUFFER*4);
    if (!streamingBuffer) {
        Log_print0(Diags_ERROR, "Failed to allocate streamingBuffer");
        status = -1;
        goto leave;
    }


    while (running) {


        /* wait for inbound message */
        status = MessageQ_get(Module.slaveQue, (MessageQ_Msg *)&msg,
            MessageQ_FOREVER);

        if (status < 0) {
            goto leave;
        }
        Log_print1(Diags_ENTRY | Diags_INFO, "Message received...%d", msg->id);
        switch (msg->cmd) {

        case App_CMD_SHARED_REGION_INIT:  // <=============================================================

            // Create Shared region with information from init message
            status = Resource_physToVirt((UInt32)msg->u.sharedRegionInitCfg.base, (UInt32 *)&sharedRegionAllocPtr);
            if(status != Resource_S_SUCCESS) {
                printf("Resource_physToVirt failed\n");
                goto leave;
            }

            srEntry.base          = sharedRegionAllocPtr;
            srEntry.len           = msg->u.sharedRegionInitCfg.size;
            srEntry.ownerProcId   = MultiProc_self();
            srEntry.isValid       = TRUE;
            srEntry.cacheEnable   = TRUE;
            srEntry.createHeap    = FALSE;
            srEntry.cacheLineSize = 128;
            srEntry.name          = "SR1";

            status = SharedRegion_setEntry (regionId1, &srEntry);
            Log_print0(Diags_ENTRY | Diags_INFO, "Shared region entry configured...");
            Log_print1(Diags_INFO, "Shared region base: 0x%x", (IArg)msg->u.sharedRegionInitCfg.base);
            Log_print1(Diags_INFO, "Shared region size: %d", (IArg)msg->u.sharedRegionInitCfg.size);
            Log_print1(Diags_INFO, "Local base: 0x%x", (IArg)srEntry.base);
            Log_print1(Diags_INFO, "Local size: %d", (IArg)srEntry.len);

            break;

        case App_CMD_BIGDATA:        // <=============================================================

            firstPass = TRUE;

            Log_print1(Diags_ENTRY | Diags_INFO, "msg->cmd=App_CMD_BIGDATA,msg->ptr=0x%x",
                (IArg)msg->u.bigDataSharedDesc.sharedPtr);

            // Translate to local descriptor
            retVal = bigDataXlatetoLocalAndSync(regionId1, &msg->u.bigDataSharedDesc, &bigDataLocalDesc);
            if (retVal) {
                status = -1;
                goto leave;
            }
            shmem = (Shared_Mem *) bigDataLocalDesc.localPtr;
            Log_print1(Diags_ENTRY | Diags_INFO, "shmem=0x%x",
                        (IArg)shmem);

            for (k=0; k<NUM_BUFFERS_TO_TEST; k++) {

                // Translate to local descriptor
                retVal = bigDataXlatetoLocalAndSync(regionId1, &msg->u.bigDataSharedDesc, &bigDataLocalDesc);
                if (retVal) {
                    status = -1;
                    goto leave;
                }

                i = shmem->dspBuffPtr;

                if ( shmem->bufferFilled[i] == 0) {                     // make sure buffer is ready to be filled

                    // test loop to get enough sweeps' data to fill a buffer
                    for (recPtr = 0; recPtr < HIGH_SPEED_INTS_PER_BUFFER; recPtr++) {

                        // Normal loop processing ===============================================================

                        streamingBuffer[recPtr] = dspCtr;

                        // // Do some calculations here to simulate normal DSP operation
                        // for (dlyCtr = 0; dlyCtr<100000; dlyCtr++); {
                        //     dlyVal = (float) (dlyCtr % 0x1fffffff) * 0.9;
                        // }

                        // =======================================================================================

                    }
                    dspCtr++;

                    memcpy ((void *) (shmem->buffer[i]), (void *) streamingBuffer, STREAMING_BUFFER_SIZE);

                    // Translate to Shared Descriptor and Sync
                    retVal = bigDataXlatetoGlobalAndSync(regionId1, &bigDataLocalDesc, &msg->u.bigDataSharedDesc);
                    if (retVal) {
                        status = -1;
                        goto leave;
                    }

                    shmem->bufferFilled[i] = 1;             // set buffer's bit to indicate it's full
                    shmem->dspBuffPtr = (i+1) % HIGH_SPEED_NUMBER_OF_BUFFERS;

                    // Translate to Shared Descriptor and Sync
                    retVal = bigDataXlatetoGlobalAndSync(regionId1, &bigDataLocalDesc, &msg->u.bigDataSharedDesc);
                    if (retVal) {
                        status = -1;
                        goto leave;
                    }
                } else {
                    // Log_print1(Diags_ENTRY | Diags_INFO, "Buffer %d skipped due to full buffer", k);
                    // Repeat last k until there is more room
                    k--;
                }

                // send message back (on first buffer fill only)
                if (firstPass) {
                    queId = MessageQ_getReplyQueue(msg);
                    MessageQ_put(queId, (MessageQ_Msg)msg);

                    firstPass = FALSE;
                }

                // Delay a short while to simulate normal dsp calcs
//              Task_sleep(1);                                          // delay in ms


                // Log_print1(Diags_ENTRY | Diags_INFO, "Checking buffer %d for invalid records", k);
                // for (i=0; i<HIGH_SPEED_INTS_PER_BUFFER; i++) {
                    // if (streamingBuffer[i*32+1] != 0xbad0dad ) {
                    //     Log_print1(Diags_ENTRY | Diags_INFO, "record %d is bad", i);
                    // }
                // }

            }

            break;

        case App_CMD_SHUTDOWN:        // <=============================================================
            running = FALSE;

            break;

        default:
            break;
        }

        /* process the message */
        Log_print2(Diags_INFO, "Server_exec: processed id %d, cmd=0x%x", msg->id, msg->cmd);

        /* send message back */
        if (msg->cmd != App_CMD_BIGDATA) {
            queId = MessageQ_getReplyQueue(msg);
            MessageQ_put(queId, (MessageQ_Msg)msg);
        }

    } /* while (running) */

leave:
    /* Print error count if non-zero */
    if (errorCount) {
        Log_print1(Diags_INFO, "Server_exec: Error Count %d", errorCount);
        status = -1;
    }
    else
        Log_print0(Diags_INFO, "Server_exec: Data check clean");

    Log_print1(Diags_EXIT, "<-- Server_exec: %d", (IArg)status);
    return(status);
}

/*
 *  ======== Server_delete ========
 */

Int Server_delete()
{
    Int         status;

    Log_print0(Diags_ENTRY, "--> Server_delete:");

    /* delete the video message queue */
    status = MessageQ_delete(&Module.slaveQue);

    if (status < 0) {
        goto leave;
    }

leave:
    if (status < 0) {
        Log_error1("Server_finish: error=0x%x", (IArg)status);
    }

    /* disable log events */
    Log_print1(Diags_EXIT, "<-- Server_delete: %d", (IArg)status);
    Diags_setMask(MODULE_NAME"-EXF");

    return(status);
}

/*
 *  ======== Server_exit ========
 */

Void Server_exit(Void)
{
    /*
     * Note that there isn't a Registry_removeModule() yet:
     *     https://bugs.eclipse.org/bugs/show_bug.cgi?id=315448
     *
     * ... but this is where we'd call it.
     */
}
