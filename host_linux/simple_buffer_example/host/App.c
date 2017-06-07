/*
 * Copyright (c) 2013-2017, Texas Instruments Incorporated
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
 *
 */

/* host header files */
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <stdlib.h>

#include <ti/cmem.h>

/* package header files */
#include <ti/ipc/Std.h>
#include "Std.h"
#include <ti/ipc/MessageQ.h>
#include "HeapMem.h"
#include "SharedRegion.h"
#include "MemoryDefs.h"

/* local header files */
#include "../shared/AppCommon.h"
#include "App.h"

/* Application specific defines */
#define BIG_DATA_POOL_SIZE 0x1000000

/* round up the value 'size' to the next 'align' boundary */
#define ROUNDUP(size, align) \
    (UInt32)(((UInt32)(size) + ((UInt32)(align) - 1)) & ~((UInt32)(align) - 1))

/* module structure */
typedef struct {
    MessageQ_Handle         hostQue;    // created locally
    MessageQ_QueueId        slaveQue;   // opened remotely
    UInt16                  heapId;     // MessageQ heapId
    UInt32                  msgSize;
} App_Module;

/* private data */
static App_Module Module;

/*
 *  ======== App_create ========
 */

Int App_create(UInt16 remoteProcId)
{
    Int                 status = 0;
    MessageQ_Params     msgqParams;
    char                msgqName[32];

    printf("--> App_create:\n");

    /* setting default values */
    Module.hostQue = NULL;
    Module.slaveQue = MessageQ_INVALIDMESSAGEQ;
    Module.heapId = App_MsgHeapId;
    Module.msgSize = sizeof(App_Msg);

    /* create local message queue (inbound messages) */
    MessageQ_Params_init(&msgqParams);

    Module.hostQue = MessageQ_create(App_HostMsgQueName, &msgqParams);

    if (Module.hostQue == NULL) {
        printf("App_create: Failed creating MessageQ\n");
        status = -1;
        goto leave;
    }

    /* open the remote message queue */
    sprintf(msgqName, App_SlaveMsgQueName, MultiProc_getName(remoteProcId));

    do {
        status = MessageQ_open(msgqName, &Module.slaveQue);
        sleep(1);
    } while (status == MessageQ_E_NOTFOUND);

    if (status < 0) {
        printf("App_create: Failed opening MessageQ\n");
        goto leave;
    }

    printf("App_create: Host is ready\n");

leave:
    printf("<-- App_create:\n");
    return(status);
}

/*
 *  ======== App_delete ========
 */
Int App_delete(Void)
{
    Int         status;

    printf("--> App_delete:\n");

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

leave:
    printf("<-- App_delete:\n");
    return(status);
}

/*
 *  ======== App_exec ========
 */
Int App_exec(Void)
{
    Int         status;
    App_Msg *   msg;
    UInt32 *bigDataLocalPtr;
    Int         i,j;
    UInt16  regionId1=1;
    SharedRegion_Entry *pSrEntry;
    void *sharedRegionAllocPtr=NULL;
    Int pool_id;
    CMEM_AllocParams cmemAttrs;
    HeapMem_Handle sr1Heap;
    UInt16 regionId;
    UInt32 errorCount=0;
    Int retVal;
    bigDataLocalDesc_t bigDataLocalDesc;
    SharedRegion_Config sharedRegionConfig;
    SharedRegion_Entry srEntry;
    HeapMem_Config HeapMemConfig;
    HeapMem_Params heapMem_params;
    Memory_Stats stats;
    HeapMem_ExtendedStats extStats;

    printf("--> App_exec:\n");

    /* CMEM: contiguous memory manager for HLOS */
    /* initialised here */
    status = CMEM_init();

    if (status < 0) {
        printf("CMEM_init failed\n");
        goto leave;
    }
    else {
        printf("CMEM_init success\n");
    }

    pool_id = CMEM_getPool(BIG_DATA_POOL_SIZE);
    if (pool_id < 0) {
        printf("CMEM_getPool failed\n");
        goto leave;
    }
    printf("CMEM_getPool success\n");

    cmemAttrs.type = CMEM_HEAP;
    cmemAttrs.flags =  CMEM_CACHED;
    cmemAttrs.alignment = 0;
    sharedRegionAllocPtr = CMEM_allocPool(pool_id, &cmemAttrs);
    if (sharedRegionAllocPtr == NULL) {
        printf("CMEM_allocPool failed\n");
        goto leave;
    }
    printf("CMEM_allocPool success: Allocated buffer %p\n", sharedRegionAllocPtr);

    /* Create shared region */
    SharedRegion_getConfig (&sharedRegionConfig);
    status = SharedRegion_setup (&sharedRegionConfig);
    if (status < 0) {
        printf("SharedRegion_setup failed\n");
        goto leave;
    }
    printf("SharedRegion_setup success\n");

    /* Configure srEntry */
    srEntry.base = sharedRegionAllocPtr;
    srEntry.len = BIG_DATA_POOL_SIZE;
    srEntry.ownerProcId = MultiProc_self();
    /* Make sure this is enabled if using Cached memory */
    srEntry.isValid = TRUE;
    srEntry.cacheEnable = TRUE;
    srEntry.createHeap = FALSE;
    srEntry.cacheLineSize = 128;
    srEntry.name = "SR1";

    status = SharedRegion_setEntry (regionId1, &srEntry);

    pSrEntry = SharedRegion_getEntryPtr(regionId1);
    printf("App_taskFxn: SR_1, base 0x%x, len=%x\n", (UInt32)pSrEntry->base, pSrEntry->len);

    regionId = regionId1;

    /* Setup HeapMem module */
    HeapMem_getConfig (&HeapMemConfig);
    status = HeapMem_setup (&HeapMemConfig);
    if (status < 0) {
        printf("HeapMem_setup failed\n");
        goto leave;
    }
    printf("HeapMem_setup success\n");

   /* Create HeapMP at run-time:
       This heap is intended to be used for big data ipc */
    HeapMem_Params_init(&heapMem_params);
    heapMem_params.name = "sr1HeapMem";
    heapMem_params.sharedAddr = pSrEntry->base;
    heapMem_params.sharedBufSize = ROUNDUP(pSrEntry->len, pSrEntry->cacheLineSize); 
    heapMem_params.gate = NULL;
    sr1Heap = HeapMem_create(&heapMem_params);
    if (!sr1Heap) {
        printf("sr1Heap creation failed\n");
        status = -1;
        goto leave;
    }
    printf("HeapMem_create success\n");
    HeapMem_getStats((HeapMem_Handle)sr1Heap, &stats);
    printf("App_taskFxn: SR_1 heap, totalSize=%d,totalFreeSize=%d,largestFreeSize=%d\n", stats.totalSize, stats.totalFreeSize, stats.largestFreeSize);
    HeapMem_getExtendedStats((HeapMem_Handle)sr1Heap, &extStats);
    printf("App_taskFxn: SR_1 heap, buf=0x%p,size=%d\n", extStats.buf, extStats.size);

    /* fill process pipeline */
    for (i = 1; i <= 3; i++) {
        printf("App_exec: sending message %d\n", i);

        /* allocate message */
        msg = (App_Msg *)MessageQ_alloc(Module.heapId, Module.msgSize);

        if (msg == NULL) {
            status = -1;
            goto leave;
        }

        /* set the return address in the message header */
        MessageQ_setReplyQueue(Module.hostQue, (MessageQ_Msg)msg);

        if ( i == 1) {
            /* fill in message payload for Shared region init*/
            msg->cmd = App_CMD_SHARED_REGION_INIT;
            msg->id = i;
            msg->regionId = regionId;
            /* Passing the local shared memory address to the remote */
            /* Actually this can be any allocated buffer for the used for the heap */
            msg->u.sharedRegionInitCfg.base = CMEM_getPhys(pSrEntry->base);
            printf("Shared memory phys Addr %llx\n", msg->u.sharedRegionInitCfg.base);
            if (!msg->u.sharedRegionInitCfg.base) {
                printf("CMEM_getPhys failed\n");
            }
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
    for (i = 4; i <= 16; i++) {

        /* Now this section of code starts receiving messages
           See the next section for the code for sending further messages */
        /* Receive messages: Start <======================================= */

        /* wait for return message */
        status = MessageQ_get(Module.hostQue, (MessageQ_Msg *)&msg,
            MessageQ_FOREVER);

        if (status < 0) {
            goto leave;
        }

        /* extract message payload */

        if (msg->cmd == App_CMD_BIGDATA) {

            retVal = bigDataXlatetoLocalAndSync(msg->regionId,
                &msg->u.bigDataSharedDesc, &bigDataLocalDesc);
            if (retVal) {
                status = -1;
                goto leave;
            }
            bigDataLocalPtr = (UInt32 *)bigDataLocalDesc.localPtr;
#ifdef DEBUG
            /* print data from big data buffer */
            printf(" Received back buffer %d\n", msg->id);
            printf(" First 8 bytes: \n");
            for ( j = 0; j < 8 && j < bigDataLocalDesc.size/sizeof(uint32_t); j+=4)
                printf("0x%x, 0x%x, 0x%x, 0x%x\n",
                    bigDataLocalPtr[j], bigDataLocalPtr[j+1], bigDataLocalPtr[j+2], bigDataLocalPtr[j+3]);
            printf(" Last 8 bytes: \n");
            for ( j = (bigDataLocalDesc.size/sizeof(uint32_t))-8 ;
                 j < bigDataLocalDesc.size/sizeof(uint32_t); j+=4)
                printf("0x%x, 0x%x, 0x%x, 0x%x\n",
                    bigDataLocalPtr[j], bigDataLocalPtr[j+1], bigDataLocalPtr[j+2], bigDataLocalPtr[j+3]);
#endif
            /* Check values to see expected results */
            for( j=0; j < bigDataLocalDesc.size/sizeof(uint32_t); j++) {
                if ( bigDataLocalPtr[j] != (msg->id+10+j) ) {
                    errorCount++;
                }
            }

            /* Free big data buffer */
            HeapMem_free(sr1Heap, bigDataLocalPtr, bigDataLocalDesc.size);
        }

        printf("App_exec: message received %d\n", msg->id);

        /* free the message */
        MessageQ_free((MessageQ_Msg)msg);

        printf("App_exec: Preparing message %d\n", i);

        /* Receive messages: End =======================================> */

        /* Send messages: Start  <======================================= */

        /* allocate message */
        msg = (App_Msg *)MessageQ_alloc(Module.heapId, Module.msgSize);

        if (msg == NULL) {
            status = -1;
            printf("MessageQ_alloc failed\n");
            goto leave;
        }

        /* set the return address in the message header */
        MessageQ_setReplyQueue(Module.hostQue, (MessageQ_Msg)msg);

        /* fill in message payload */
        if (i < 14) {

            /* Send Big data messages */

            msg->cmd = App_CMD_BIGDATA;
            msg->id = i;

            /* Allocate buffer from HeapMem */
            bigDataLocalPtr = (UInt32 *)(HeapMem_alloc(sr1Heap, BIGDATA_BUF_SIZE, BIGDATA_BUF_ALIGN));

            if (!bigDataLocalPtr) {
                status = -1;
                printf("HeapMem_alloc failed\n");
                goto leave;
            }

            /* Fill Big data buffer */
            for(j=0; j< BIGDATA_BUF_SIZE/sizeof(uint32_t); j++) {
               bigDataLocalPtr[j] = j+i;
            }

            /* Populate the Local descriptor */
            bigDataLocalDesc.localPtr = (Ptr)bigDataLocalPtr;
            bigDataLocalDesc.size = BIGDATA_BUF_SIZE;

            retVal = bigDataXlatetoGlobalAndSync(regionId,
                &bigDataLocalDesc, &msg->u.bigDataSharedDesc);
            if (retVal) {
                status = -1;
                printf("bigDataXlatetoGlobalAndSync failed\n");
                goto leave;
            }
            msg->regionId = regionId;
        } else {
            if (i == 16) {
                /* Last message will tell the slave to shutdown */
                msg->cmd = App_CMD_SHUTDOWN;
                msg->id = i;
            } else {
                /* Send dummy NOP messages before shutdown */
                msg->cmd = App_CMD_NOP;
                msg->id = i;
            }
        }
        printf("App_exec: Sending message %d\n", i);

        /* send message */
        MessageQ_put(Module.slaveQue, (MessageQ_Msg)msg);

        /* Send messages: End  =======================================> */

    }

    /* drain process pipeline */
    for (i = 1; i <= 3; i++) {

        /* wait for return message */
        status = MessageQ_get(Module.hostQue, (MessageQ_Msg *)&msg,
            MessageQ_FOREVER);

        if (status < 0) {
            printf("MessageQ_get failed\n");
            goto leave;
        }
        printf("App_exec: message received: %d\n", msg->id);

        /* extract message payload */

        /* free the message */
        MessageQ_free((MessageQ_Msg)msg);
    }

leave:
    if (sr1Heap) {
        HeapMem_delete(&sr1Heap);
    }
    if (sharedRegionAllocPtr) {
        /* free the message */
        CMEM_free(sharedRegionAllocPtr, &cmemAttrs);
    }
    /* Print error count if non-zero */
    if (errorCount) {
        printf("App_exec: Error Count %d\n", errorCount);
        status = -1;
    }
    else
        printf("App_exec: Data check clean\n");

    printf("<-- App_exec: %d\n", status);
    return(status);
}
