/*
 * Copyright (c) 2017 Texas Instruments Incorporated - http://www.ti.com
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
 *  ======== App.c ========
 */

/* package header files */
#include <xdc/std.h>
#include <xdc/runtime/Diags.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/System.h>

#include <ti/ipc/Ipc.h>
#include <ti/ipc/MessageQ.h>
#include <ti/ipc/MultiProc.h>
#include <ti/ipc/SharedRegion.h>

#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/ipc/HeapMemMP.h>
#include <ti/sysbios/hal/Cache.h>

/* local header files */
#include "../shared/AppCommon.h"
#include "App.h"

/* round up the value 'size' to the next 'align' boundary */
#define ROUNDUP(size, align) \
    (UInt32)(((UInt32)(size) + ((UInt32)(align) - 1)) & ~((UInt32)(align) - 1))

#define NUM_MSGS 4

/* module structure */
typedef struct {
    MessageQ_Handle     hostQue;    // created locally
    MessageQ_QueueId    slaveQue;   // opened remotely
    UInt16              heapId;     // MessageQ heapId
    HeapBuf_Handle      heap;       // message heap
    Int                 msgSize;    // aligned size of message
    Int                 poolSize;   // size of message pool
    Ptr                 store;      // memory store for message pool
} App_Module;

/* private data */
static App_Module Module;


/*
 *  ======== App_create ========
 */

Int App_create(UInt16 remoteProcId)
{
    Int                 status = 0;
    Int                 align;
    Error_Block         eb;
    IHeap_Handle        srHeap;
    HeapBuf_Params      heapParams;
    MessageQ_Params     msgqParams;
    char                msgqName[32];

    Log_print0(Diags_INFO, "App_create: -->");

    /* setting default values */
    Module.hostQue = NULL;
    Module.slaveQue = MessageQ_INVALIDMESSAGEQ;
    Module.heapId = App_MsgHeapId;
    Module.msgSize = 0;

    /* compute message size to fill entire cache lines */
    align = SharedRegion_getCacheLineSize(0);
    Module.msgSize = ROUNDUP(sizeof(App_Msg), align);

    /* compute message pool size */
    Module.poolSize = Module.msgSize * NUM_MSGS;

    /* acquire message pool memory */
    srHeap = (IHeap_Handle)SharedRegion_getHeap(0);
    Module.store = Memory_alloc(srHeap, Module.poolSize, align, NULL);

    /* create a heap in shared memory for message pool */
    HeapBuf_Params_init(&heapParams);
    heapParams.blockSize = Module.msgSize;
    heapParams.numBlocks = NUM_MSGS;
    heapParams.bufSize = Module.poolSize;
    heapParams.align = align;
    heapParams.buf = Module.store;
    Error_init(&eb);

    Module.heap = HeapBuf_create(&heapParams, &eb);

    if (Module.heap == NULL) {
        Log_error0("App_create: failed creating message pool");
        status = -1;
        goto leave;
    }

    /* bind message pool to heapId */
    MessageQ_registerHeap((Ptr)(Module.heap), App_MsgHeapId);

    /* create local message queue (inbound messages) */
    MessageQ_Params_init(&msgqParams);

    Module.hostQue = MessageQ_create(NULL, &msgqParams);

    if (Module.hostQue == NULL) {
        Log_error0("App_create: failed creating MessageQ");
        status = -1;
        goto leave;
    }

    /* open the remote message queue */
    System_sprintf(msgqName, App_SlaveMsgQueName,
            MultiProc_getName(remoteProcId));

    do {
        status = MessageQ_open(msgqName, &Module.slaveQue);
        Task_sleep(1);
    } while (status == MessageQ_E_NOTFOUND);

    if (status < 0) {
        Log_error0("App_create: failed opening MessageQ");
        goto leave;
    }

    Log_print0(Diags_INFO, "App_create: Host is ready");

leave:
    Log_print1(Diags_INFO, "App_create: <-- status=%d", (IArg)status);
    return(status);
}


/*
 *  ======== App_delete ========
 */
Int App_delete(Void)
{
    Int             status;
    IHeap_Handle    srHeap;

    Log_print0(Diags_INFO, "App_delete: -->");

    /* close remote resources */
    status = MessageQ_close(&Module.slaveQue);

    if (status < 0) {
        goto leave;
    }

    /* delete the host message queue */
    status = MessageQ_delete(&Module.hostQue);

    if (status < 0) {
        goto leave;
    }

    /* unregister message pool heap */
    MessageQ_unregisterHeap(App_MsgHeapId);

    /* delete the message pool heap */
    HeapBuf_delete(&Module.heap);

    /* release message pool memory */
    srHeap = (IHeap_Handle)SharedRegion_getHeap(0);
    Memory_free(srHeap, Module.store, Module.poolSize);

leave:
    Log_print1(Diags_INFO, "App_delete: <-- status=%d", (IArg)status);
    return(status);
}


/*
 *  ======== App_exec ========
 */
Int App_exec(Void)
{
    Int         status;
    App_Msg *   msg;
    Uint32 *bigDataPtr;
    SharedRegion_SRPtr bigDataSharedPtr;
    Int16 i,j;
    UInt16  regionId1;
    Memory_Stats stats;
    HeapMemMP_ExtendedStats extStats;
    SharedRegion_Entry *pSrEntry;
    HeapMemMP_Params heapMemMp_params;
    HeapMemMP_Handle sr1Heap;
    Uint16 regionId;
    HeapMemMP_Handle srHeap;
    UInt32 errorCount=0;
    
    Log_print0(Diags_INFO, "App_exec: -->");

    // obtain information about heap (if any) in SR_1
    regionId1 = SharedRegion_getIdByName("SR_1");
    Log_print1(Diags_INFO, "App_taskFxn: SR_1 region Id=%d",regionId1);
    sr1Heap = SharedRegion_getHeap(regionId1);
    if (sr1Heap != NULL)
    {
        /* SR has predefined heap */
        HeapMemMP_getStats((HeapMemMP_Handle)sr1Heap, &stats);
        Log_print3(Diags_INFO, "App_taskFxn: SR_1 heap, totalSize=%d,totalFreeSize=%d,largestFreeSize=%d", (IArg)stats.totalSize, (IArg)stats.totalFreeSize, (IArg)stats.largestFreeSize);
        HeapMemMP_getExtendedStats((HeapMemMP_Handle)sr1Heap, &extStats);
        Log_print2(Diags_INFO, "App_taskFxn: SR_1 heap, buf=0x%x,size=%d", (IArg)extStats.buf, (IArg)extStats.size);
        Log_error0("App_taskFxn: Error: Example assumes shared region without preconfigured heap");
        status = -1;
        goto leave;
    }
    else
    {
        Log_print0(Diags_INFO, "App_taskFxn: SR_1, sr1Heap=NULL");
        
        pSrEntry = SharedRegion_getEntryPtr(regionId1);
        Log_print1(Diags_INFO, "App_taskFxn: SR_1, len=%d", pSrEntry->len);

        // Create HeapMP at run-time
        HeapMemMP_Params_init(&heapMemMp_params);
        heapMemMp_params.name = "sr1HeapMemMp";
        heapMemMp_params.sharedAddr = pSrEntry->base;
        heapMemMp_params.sharedBufSize = ROUNDUP(pSrEntry->len, pSrEntry->cacheLineSize); 
        heapMemMp_params.gate = NULL;
        sr1Heap = HeapMemMP_create(&heapMemMp_params);
        if (!sr1Heap) {
            Log_error0("srHeap creation failed");
            status = -1;
            goto leave;
        }
        HeapMemMP_getStats((HeapMemMP_Handle)sr1Heap, &stats);
        Log_print3(Diags_INFO, "App_taskFxn: SR_1 heap, totalSize=%d,totalFreeSize=%d,largestFreeSize=%d", (IArg)stats.totalSize, (IArg)stats.totalFreeSize, (IArg)stats.largestFreeSize);
        HeapMemMP_getExtendedStats((HeapMemMP_Handle)sr1Heap, &extStats);
        Log_print2(Diags_INFO, "App_taskFxn: SR_1 heap, buf=0x%x,size=%d", (IArg)extStats.buf, (IArg)extStats.size);
    }

    srHeap = sr1Heap;
    regionId = regionId1;

    /* fill process pipeline */
    for (i = 1; i <= 3; i++) {
        Log_print1(Diags_INFO, "App_exec: sending message %d", (IArg)i);

        /* allocate message */
        msg = (App_Msg *)MessageQ_alloc(Module.heapId, Module.msgSize);

        if (msg == NULL) {
            status = -1;
            goto leave;
        }

        /* set the return address in the message header */
        MessageQ_setReplyQueue(Module.hostQue, (MessageQ_Msg)msg);

        if ( i == 1) {
            /* fill in message payload */
            msg->cmd = App_CMD_SHARED_REGION_INIT;
            msg->id = i;
            msg->regionId = regionId;
            /* Passing the local shared memory address to the remote */
            /* Actually this can be any allocated buffer for the used for the heap */
            msg->u.sharedRegionInitCfg.base = (intptr_t)(pSrEntry->base);
            msg->u.sharedRegionInitCfg.size = (UInt64)(pSrEntry->len);
        } else {
            /* fill in message payload */
            msg->cmd = App_CMD_NOP;
            msg->id = i;
	}

        /* send message */
        MessageQ_put(Module.slaveQue, (MessageQ_Msg)msg);
    }

    /* process steady state (keep pipeline full) */
    for (i = 4; i <= 18; i++) {

        /* wait for return message */
        status = MessageQ_get(Module.hostQue, (MessageQ_Msg *)&msg,
            MessageQ_FOREVER);

        if (status < 0) {
            goto leave;
        }

        /* extract message payload */

        if ((msg->cmd == App_CMD_BIGDATA) && (msg->u.bigDataBuffer.sharedPtr) ) {

            /* Translate to local address */
            bigDataPtr = (Uint32 *)SharedRegion_getPtr(msg->u.bigDataBuffer.sharedPtr);

            /* If shared region is configured for cache enabled do cache operation */
            if (SharedRegion_isCacheEnabled(msg->regionId)) {
                Cache_inv(bigDataPtr,
                    msg->u.bigDataBuffer.size, Cache_Type_ALL, TRUE);
            }
#ifdef DEBUG
            /* print data from big data buffer */
            Log_print1(Diags_INFO, " Received back buffer %d", msg->id);
            Log_print0(Diags_INFO, " First 8 bytes: ");
            for ( j = 0; j < 8 && j < msg->u.bigDataBuffer.size/sizeof(uint32_t); j+=4)
                Log_print4(Diags_INFO, "0x%x, 0x%x, 0x%x, 0x%x",
                    bigDataPtr[j], bigDataPtr[j+1], bigDataPtr[j+2], bigDataPtr[j+3]);
            Log_print0(Diags_INFO, " Last 8 bytes: ");
            for ( j = (msg->u.bigDataBuffer.size/sizeof(uint32_t))-8 ;
                 j < msg->u.bigDataBuffer.size/sizeof(uint32_t); j+=4)
                Log_print4(Diags_INFO, "0x%x, 0x%x, 0x%x, 0x%x",
                    bigDataPtr[j], bigDataPtr[j+1], bigDataPtr[j+2], bigDataPtr[j+3]);
#endif
            /* Check values to see expected results */
            for( j=0; j < msg->u.bigDataBuffer.size/sizeof(uint32_t); j++) {
                if ( bigDataPtr[j] != (msg->id+10+j) ) {
                    errorCount++;
                }
            }

            /* Free big data buffer */
            HeapMemMP_free(srHeap, bigDataPtr, msg->u.bigDataBuffer.size);
        }

        /* free the message */
        MessageQ_free((MessageQ_Msg)msg);

        Log_print1(Diags_INFO, "App_exec: message received, sending message %d",
                (IArg)i);

        /* allocate message */
        msg = (App_Msg *)MessageQ_alloc(Module.heapId, Module.msgSize);

        if (msg == NULL) {
            status = -1;
            goto leave;
        }

        /* set the return address in the message header */
        MessageQ_setReplyQueue(Module.hostQue, (MessageQ_Msg)msg);

        /* fill in message payload */
        if (i == 18) {
            /* Last message will tell the slave to shutdown */
            msg->cmd = App_CMD_SHUTDOWN;
            msg->id = i;
        } else if (i < 15) {
            msg->cmd = App_CMD_BIGDATA;
            msg->id = i;

            /* Allocate buffer from HeapMemMP */
            bigDataPtr = (Uint32 *)(HeapMemMP_alloc(srHeap, BIGDATA_SIZE, BIGDATA_ALIGN));

            if ( ! bigDataPtr ) {
                status = -1;
                goto leave;
            }

            /* Fill Big data buffer */
            for(j=0; j< BIGDATA_SIZE/sizeof(uint32_t); j++) {
               bigDataPtr[j] = j+i;
            }

            /* Cache write back data buffer */
            if (SharedRegion_isCacheEnabled(regionId)) {
                Cache_wb(bigDataPtr, BIGDATA_SIZE, Cache_Type_ALL, TRUE);
            }

            /* Translate Address to Shared address */
            bigDataSharedPtr = SharedRegion_getSRPtr(bigDataPtr, regionId);

            msg->u.bigDataBuffer.sharedPtr = bigDataSharedPtr;
            msg->u.bigDataBuffer.size = BIGDATA_SIZE;
            msg->regionId = regionId;
        }
        else {
            msg->cmd = App_CMD_NOP;
            msg->id = i;
        }

        /* send message */
        MessageQ_put(Module.slaveQue, (MessageQ_Msg)msg);
    }

    /* drain process pipeline */
    for (i = 1; i <= 3; i++) {
        Log_print0(Diags_INFO, "App_exec: message received");

        /* wait for return message */
        status = MessageQ_get(Module.hostQue, (MessageQ_Msg *)&msg,
            MessageQ_FOREVER);

        if (status < 0) {
            goto leave;
        }

        /* extract message payload */

        /* free the message */
        MessageQ_free((MessageQ_Msg)msg);
    }

leave:
    /* Print error count if non-zero */
    if (errorCount) {
        Log_print1(Diags_INFO, "App_exec: Error Count %d", errorCount);
        status = -1;
    }
    else
        Log_print0(Diags_INFO, "App_exec: Data check clean");

    Log_print1(Diags_INFO, "App_exec: <-- status=%d", (IArg)status);
    return(status);
}
