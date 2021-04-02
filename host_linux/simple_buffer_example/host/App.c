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
#include <unistd.h>
#include <string.h>

#include <ti/cmem.h>

/* package header files */
#include <ti/ipc/Std.h>
#include "Std.h"
#include <ti/ipc/MessageQ.h>
//#include "HeapMem.h"
#include "SharedRegion.h"
#include "MemoryDefs.h"
#include <ti/ipc/GateMP.h>
#include "Cache.h"

/* local header files */
#include "../shared/ipcCommon.h"
#include "App.h"

/* Application specific defines */
#define DEBUG                                   1
#define BIG_DATA_POOL_SIZE              0x1000000
//#define HIGH_SPEED_STREAMING_BUFFERS        128

//#define STREAMING_BUFFER_SIZE   HIGH_SPEED_FLAGS_PER_BUFFER * 4         // 4K 
// #define NEXT_GEN_STREAMING

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

    printf("     - remoteProcId, msgqName: %d, %s\n", remoteProcId, msgqName);

    do {
        status = MessageQ_open(msgqName, &Module.slaveQue);
        sleep(1);

    printf ("MessageQ_open status = %d\n", status);

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
    Int                     errorCtr = 0;
//  Int                     expectedDspCtr = 1;
    Int                     i,j, k;
    Int                     msgID = 0;
    Int                     pool_id;
    Int                     retVal;
//  Int                     sbPtr = 0;
    Int                     status;
    Int                     streamingBuffer[HIGH_SPEED_NUMBER_OF_BUFFERS][HIGH_SPEED_FLAGS_PER_BUFFER];
//  Int                     streamingBuffer2[STREAMING_BUFFER_SIZE];
    Int                     diagBuffer[500][6];
//  Int                     diagBuffCtr = 0;
    Int                     start, count;
    Int                     timer;

//  UInt32                  errorCount=0;

    UInt16                  regionId;
    UInt16                  regionId1=1;

    Bool                    getFlags         = TRUE;
    Bool                    streamingStarted = FALSE;

    Shared_Mem              *shmem;

//  UInt32         *bigDataLocalPtr;

    App_Msg *               msg;
    SharedRegion_Entry      *pSrEntry;
    void                    *sharedRegionAllocPtr=NULL;
    CMEM_AllocParams        cmemAttrs;
//  HeapMem_Handle          sr1Heap;
    bigDataLocalDesc_t      bigDataLocalDesc;
    SharedRegion_Config     sharedRegionConfig;
    SharedRegion_Entry      srEntry;
//  HeapMem_Config          HeapMemConfig;
//  HeapMem_Params          heapMem_params;
    Memory_Stats            stats;
//  HeapMem_ExtendedStats   extStats;

    GateMP_Params           gparams;
    GateMP_Handle           gateHandle;
    IArg                    gateKey;

    printf("--> App_exec-4:\n");

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


    /* fill process pipeline */
    for (i = 1; i <= 3; i++) {
        /* allocate message */
        msg = (App_Msg *)MessageQ_alloc(Module.heapId, Module.msgSize);

        if (msg == NULL) {
            status = -1;
            goto leave;
        }

        /* set the return address in the message header */
        MessageQ_setReplyQueue(Module.hostQue, (MessageQ_Msg)msg);

        if ( i == 1) {
            printf("App_exec: sending message %d (REGION_INIT)\n", i);

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
            printf("App_exec: sending message %d (NOP)\n", i);
            /* fill in message payload */
            msg->cmd = App_CMD_NOP;
            msg->id = i;
        }

        /* send message */
        MessageQ_put(Module.slaveQue, (MessageQ_Msg)msg);
    }


    msgID            = 4;
    streamingStarted = FALSE;

    // Retrieve & send 4 messages 
    //      read-1 SHARED_REGION_INIT / send-4 BIGDATA
    //      read-2 NOP                / send-5 NOP
    //      read-3 NOP                / send-6 NOP  
    //      read-4 BIGDATA            / send-7 SHUTDOWN
    while (streamingStarted == FALSE) {

        /* Now this section of code starts receiving messages
           See the next section for the code for sending further messages */
        /* Receive messages: Start <======================================= */

        /* wait for return message */
        status = MessageQ_get (Module.hostQue, (MessageQ_Msg *)&msg, MessageQ_FOREVER);
        if (status < 0) {
            goto leave;
        }

        printf("App_exec (v1.0): message received %d\n", msg->id);

        if (msg->cmd == App_CMD_BIGDATA) {
            streamingStarted = TRUE;
        }

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
        if (msgID == 4) {
            printf("App_exec: sending message %d (BIGDATA)\n", msgID);

            // Send Big data messages 
            msg->cmd = App_CMD_BIGDATA;
            msg->id  = msgID++;

            /* Allocate buffer from HeapMem */
            shmem = (Shared_Mem *) pSrEntry->base;
            if (!shmem) {
                status = -1;
                printf("HeapMem_alloc failed\n");
                goto leave;
            }

            // Initialized pointers for continuous operation
            shmem->dspBuffPtr   = 0;                                    // initialize the pointer to the first buffer
            shmem->armBuffPtr   = 0;                                    // initialize the pointer to the first buffer
    
            // Initialize data buffers to zero
            for (j=0; j<HIGH_SPEED_NUMBER_OF_BUFFERS; j++) {

                shmem->bufferFilled[j] = 0;                                    // all buffers available

                for (k=0; k<HIGH_SPEED_FLAGS_PER_BUFFER; k++) {
                    shmem->buffer[j][k]   = 0;
                    streamingBuffer[j][k] = 0;
                }
            }

            Int dspCtr = 1;
            for (k=0; k<HIGH_SPEED_NUMBER_OF_BUFFERS+1; k++) {

                i = shmem->dspBuffPtr;
                // Int *Buffer = shmem->buffer[i];
                Int Buffer[STREAMING_BUFFER_SIZE];

                if ( shmem->bufferFilled[i] == 0) {                     // make sure buffer is ready to be filled

                    // test loop to get enough sweeps' data to fill a buffer
                    for (Int recPtr = 0; recPtr<HIGH_SPEED_NUMBER_OF_RECORDS; recPtr++) {

                        // Normal loop processing ===============================================================

                        for (j=0; j<HIGH_SPEED_NUMBER_OF_FLAGS; j++) {
                            if (j==0) {
                                Buffer[recPtr*32+j] = dspCtr++;
                            } else if (j==1) {
                                Buffer[recPtr*32+j] = 0xbad0dad;
                            } else if (j==2) {
                                Buffer[recPtr*32+j] = k;
                            } else if (j== (HIGH_SPEED_NUMBER_OF_FLAGS-1) ) {
                                Buffer[recPtr*32+j] = 0; //(int) (dlyVal*100.);
                            } else {
                                Buffer[recPtr*32+j] = i * 0x1000000 + recPtr * 0x1000 + j;
                            }
                        }

                        // =======================================================================================

                    }
                }


                memcpy ((void *) (shmem->buffer[i]), (void *) Buffer, STREAMING_BUFFER_SIZE);

                shmem->bufferFilled[shmem->dspBuffPtr] = 1;             // set buffer's bit to indicate it's full
                shmem->dspBuffPtr    = (shmem->dspBuffPtr+1) % HIGH_SPEED_NUMBER_OF_BUFFERS;

            }

            // Populate the Local descriptor
            bigDataLocalDesc.localPtr = (void *)shmem;
            bigDataLocalDesc.size     = BIGDATA_BUF_SIZE;

            Cache_wb (bigDataLocalDesc.localPtr, bigDataLocalDesc.size, Cache_Type_ALL, TRUE);

            msg->u.bigDataSharedDesc.sharedPtr = SharedRegion_getSRPtr (bigDataLocalDesc.localPtr, regionId);
            msg->u.bigDataSharedDesc.size      = bigDataLocalDesc.size;

            msg->regionId                      = regionId;

        } else {
            if (msgID == 7) {
                printf("App_exec: sending message %d (SHUTDOWN)\n", msgID);
                
                msg->cmd = App_CMD_SHUTDOWN;                            // Last message will tell the slave to shutdown 
                msg->id  = msgID++;

            } else {
                printf("App_exec: sending message %d (NOP)\n", msgID);
                
                msg->cmd = App_CMD_NOP;                                    // Send dummy NOP messages before shutdown 
                msg->id  = msgID++;
            }
        }

        /* send message */
        MessageQ_put(Module.slaveQue, (MessageQ_Msg)msg);

        /* Send messages: End  =======================================> */

    }

    //===============================
    // Process streaming data & flags (goes on forever)
    //===============================
    if (streamingStarted) {

        Cache_inv(bigDataLocalDesc.localPtr, bigDataLocalDesc.size, Cache_Type_ALL, TRUE);

        // Process the first streaming packet
        diagBuffer[0][0] = shmem->bufferFilled[shmem->armBuffPtr];

        // Retrieve the first buffer
        if ( shmem->bufferFilled[shmem->armBuffPtr] == 1 ) {           // if the DSP has filled this buffer
            memcpy ((void *) (  streamingBuffer[shmem->armBuffPtr]), 
                                (void *) (shmem->buffer[shmem->armBuffPtr]), 
                                STREAMING_BUFFER_SIZE );

            shmem->bufferFilled[shmem->armBuffPtr] = 0;                 // clear flag to indicate buffer is free
            shmem->armBuffPtr    = (shmem->armBuffPtr+1)%HIGH_SPEED_NUMBER_OF_BUFFERS;
            errorCtr             = 0;
        }

                printf ("1...\n");

        Cache_wb (bigDataLocalDesc.localPtr, bigDataLocalDesc.size, Cache_Type_ALL, TRUE);


        // Next gen streaming data
        j = 1;
        while (j<HIGH_SPEED_NUMBER_OF_BUFFERS) {

            Cache_inv(bigDataLocalDesc.localPtr, bigDataLocalDesc.size, Cache_Type_ALL, TRUE);

            diagBuffer[j][0] = shmem->bufferFilled[shmem->armBuffPtr];


            if ( shmem->bufferFilled[shmem->armBuffPtr] == 1 ) {        // if the DSP has filled this buffer
                // Retrieve the next buffer
                memcpy ( (void *) (streamingBuffer[shmem->armBuffPtr]), 
                         (void *) (shmem->buffer[shmem->armBuffPtr]), 
                         STREAMING_BUFFER_SIZE );

                shmem->bufferFilled[shmem->armBuffPtr] = 0;             // clear this buffer's full bit => ready to fill
                shmem->armBuffPtr    = (shmem->armBuffPtr+1) % HIGH_SPEED_NUMBER_OF_BUFFERS;

                Cache_wb (bigDataLocalDesc.localPtr, bigDataLocalDesc.size, Cache_Type_ALL, TRUE);

                j++;                                                // increment buffer pointer

            } else {
                errorCtr++;
            }

            usleep (1000);                                          // delay in micro-seconds (10^-6)
        }    // while 

    }   // if streamingStarted

    /* drain process pipeline */
    for (i = 1; i <= 3; i++) {

        /* wait for return message */
        status = MessageQ_get(Module.hostQue, (MessageQ_Msg *)&msg,
            MessageQ_FOREVER);

        if (status < 0) {
            printf("MessageQ_get failed\n");
            goto leave;
        }
        printf("App_exec: message received-3: %d\n", msg->id);

        /* extract message payload */

        /* free the message */
        MessageQ_free((MessageQ_Msg)msg);
    }

leave:
    if (sharedRegionAllocPtr) {
        /* free the message */
        CMEM_free(sharedRegionAllocPtr, &cmemAttrs);
    }

    printf ("# of sweeps that a buffer wasn't ready from DSP: %d\nReceived buffer: \n", errorCtr);

    printf (" buffer\t\trecord\t\tdspCtr\t\t    [3]\t\t BuffReady \n");

    for (i=0; i<HIGH_SPEED_NUMBER_OF_BUFFERS; i++) {
        for (j=0; j<HIGH_SPEED_NUMBER_OF_RECORDS; j++) {

            printf (" %2.2d\t\t  %2.2d\t\t %2.2d \t\t%8.8x\t\t%8.8x \n", i, j, streamingBuffer[i][j*32], 
                                                                               streamingBuffer[i][j*32+3],
                                                                               diagBuffer[i][0]);
        }
    }

    status = 0;

    printf("<-- App_exec: %d\n", status);
    return(status);
}
