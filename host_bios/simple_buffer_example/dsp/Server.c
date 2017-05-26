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

/* package header files */
#include <ti/ipc/MessageQ.h>
#include <ti/ipc/MultiProc.h>
#include <ti/ipc/SharedRegion.h>
#include <ti/ipc/HeapMemMP.h>
#include <ti/sysbios/hal/Cache.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/Memory.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* local header files */
#include "../shared/AppCommon.h"

/* module header file */
#include "Server.h"

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
    Int                 status;
    Bool                running = TRUE;
    App_Msg *           msg;
    MessageQ_QueueId    queId;
    UInt16  regionId;
    IHeap_Handle srHeap;
    Memory_Stats stats;
    HeapMemMP_ExtendedStats extStats;
    Uint32 *bigDataLocalPtr;
    Int j;
    UInt32 errorCount=0;
    Int retVal;
    bigDataLocalDesc_t bigDataLocalDesc;

    Log_print0(Diags_ENTRY | Diags_INFO, "--> Server_exec:");

    while (running) {

        /* wait for inbound message */
        status = MessageQ_get(Module.slaveQue, (MessageQ_Msg *)&msg,
            MessageQ_FOREVER);

        if (status < 0) {
            goto leave;
        }
        Log_print1(Diags_ENTRY | Diags_INFO, "Message received...%d", msg->id);
        switch (msg->cmd) {
        case App_CMD_SHARED_REGION_INIT:
            regionId = msg->regionId;
            /* NOTE: Alternatively the Shared region can be created here by using id, base, length right here */
            Log_print1(Diags_INFO, "Server_taskFxn: SR_1 region Id=%d",msg->regionId);
            srHeap = (IHeap_Handle)SharedRegion_getHeap(regionId);
            if (srHeap != NULL)
            {
                HeapMemMP_getStats((HeapMemMP_Handle)srHeap, &stats);
                Log_print3(Diags_INFO, "Server_taskFxn: SR_1 heap, totalSize=%d,totalFreeSize=%d,largestFreeSize=%d", (IArg)stats.totalSize, (IArg)stats.totalFreeSize, (IArg)stats.largestFreeSize);
                HeapMemMP_getExtendedStats((HeapMemMP_Handle)srHeap, &extStats);
                Log_print2(Diags_INFO, "Server_taskFxn: SR_1 heap, buf=0x%x,size=%d", (IArg)extStats.buf, (IArg)extStats.size);
            }
            else
            {
                Log_print0(Diags_INFO, "Server_taskFxn: SR_1, srHeap=NULL");
            }    
        break;

        case App_CMD_BIGDATA:
#ifdef DEBUG
            Log_print1(Diags_ENTRY | Diags_INFO, "msg->cmd=App_CMD_BIGDATA,msg->ptr=0x%x",
                (IArg)msg->u.bigDataSharedDesc.sharedPtr);
#endif

            /* Translate to local descriptor */
            retVal = bigDataXlatetoLocalAndSync(msg->regionId,
                &msg->u.bigDataSharedDesc, &bigDataLocalDesc);
            if (retVal) {
                status = -1;
                goto leave;
            }
            bigDataLocalPtr = (Uint32 *)bigDataLocalDesc.localPtr;
#ifdef DEBUG
            /* print message from buffer */
            Log_print1(Diags_INFO, " Received message %d", msg->id);
            Log_print1(Diags_INFO, " Local Pointer 0x%x", (UInt32)bigDataLocalPtr);
            Log_print0(Diags_INFO, " First 8 bytes: ");
            for ( j = 0; j < 8 && j < bigDataLocalDesc.size/sizeof(uint32_t); j+=4)
                Log_print4(Diags_INFO, "0x%x, 0x%x, 0x%x, 0x%x",
                    bigDataLocalPtr[j], bigDataLocalPtr[j+1], bigDataLocalPtr[j+2], bigDataLocalPtr[j+3]);
            Log_print0(Diags_INFO, " Last 8 bytes: ");
            for ( j = (bigDataLocalDesc.size/sizeof(uint32_t))-8 ;
                 j < bigDataLocalDesc.size/sizeof(uint32_t); j+=4)
                Log_print4(Diags_INFO, "0x%x, 0x%x, 0x%x, 0x%x",
                    bigDataLocalPtr[j], bigDataLocalPtr[j+1], bigDataLocalPtr[j+2], bigDataLocalPtr[j+3]);
#endif
            /* Check values to see expected results */
            for( j=0; j < bigDataLocalDesc.size/sizeof(uint32_t); j++) {
                if ( bigDataLocalPtr[j] != (msg->id+j) ) {
                    errorCount++;
                }
            }

            /* Fill new data */
            for ( j=0; j < bigDataLocalDesc.size/sizeof(uint32_t); j++)
                bigDataLocalPtr[j] = msg->id + 10 +j;

            /* Translate to Shared Descriptor and Sync */
            retVal = bigDataXlatetoGlobalAndSync(msg->regionId,
                &bigDataLocalDesc, &msg->u.bigDataSharedDesc);
            if (retVal) {
                status = -1;
                goto leave;
            }    
        break;

        case App_CMD_SHUTDOWN:
            running = FALSE;
        break;

        default:
        break;
        }

        /* process the message */
        Log_print2(Diags_INFO, "Server_exec: processed id %d, cmd=0x%x", msg->id, msg->cmd);

        /* send message back */
        queId = MessageQ_getReplyQueue(msg);
        MessageQ_put(queId, (MessageQ_Msg)msg);
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
