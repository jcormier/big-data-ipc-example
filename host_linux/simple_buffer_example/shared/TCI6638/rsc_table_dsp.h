/*
 * Copyright (c) 2012-2017, Texas Instruments Incorporated
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
 *  ======== rsc_table_tci6638.h ========
 *
 *  Include this table in each base image, which is read from remoteproc on
 *  host side.
 *
 */

#ifndef _RSC_TABLE_DSP_H_
#define _RSC_TABLE_DSP_H_

#include <ti/ipc/remoteproc/rsc_types.h>

/* virtio ids: keep in sync with the linux "include/linux/virtio_ids.h" */
#define VIRTIO_ID_RPMSG         7 /* virtio remote processor messaging */

/* flip up bits whose indices represent features we support */
#define RPMSG_IPU_C0_FEATURES         1

#define RPMSG_VRING0_DA     0xA0000000
#define RPMSG_VRING1_DA     0xA0004000

/* NOTE: Make sure this matches what is configured in the linux device tree */
#define DSP_CMEM_IOBUFS 0xB0000000
#define PHYS_CMEM_IOBUFS 0xB0000000
#define DSP_CMEM_IOBUFS_SIZE (SZ_1M * 384)

/*
 * sizes of the virtqueues (expressed in number of buffers supported,
 * and must be power of 2)
 */
#define RPMSG_VQ0_SIZE          256
#define RPMSG_VQ1_SIZE          256

struct my_resource_table {
    struct resource_table base;

    UInt32 offset[3];

    /* rpmsg vdev entry */
    struct fw_rsc_vdev rpmsg_vdev;
    struct fw_rsc_vdev_vring rpmsg_vring0;
    struct fw_rsc_vdev_vring rpmsg_vring1;

    /* trace entry */
    struct fw_rsc_trace trace;
    /* devmem entry */
    struct fw_rsc_devmem devmem0;
};

/* Add trace buffer information to the resource table */
//#define TRACEBUFADDR (UInt32)&xdc_runtime_SysMin_Module_State_0_outbuf__A
extern char ti_trace_SysMin_Module_State_0_outbuf__A;
#define TRACEBUFADDR (UInt32)&ti_trace_SysMin_Module_State_0_outbuf__A
#define TRACEBUFSIZE 0x8000

#define CARVEOUTADDR TRACEBUFADDR
#define CARVEOUTSIZE TRACEBUFSIZE

#pragma DATA_SECTION(ti_ipc_remoteproc_ResourceTable, ".resource_table")
#pragma DATA_ALIGN(ti_ipc_remoteproc_ResourceTable, 4096)

struct my_resource_table ti_ipc_remoteproc_ResourceTable = {
    1, /* we're the first version that implements this */
    3, /* number of entries in the table */
    0, 0, /* reserved, must be zero */
    /* offsets to entries */
    {
        offsetof(struct my_resource_table, rpmsg_vdev),
        offsetof(struct my_resource_table, trace),
        offsetof(struct my_resource_table, devmem0),
    },

    /* rpmsg vdev entry */
    {
        TYPE_VDEV, VIRTIO_ID_RPMSG, 0,
        RPMSG_IPU_C0_FEATURES, 0, 0, 0, 2, { 0, 0 },
        /* no config data */
    },
    /* the two vrings */
    { RPMSG_VRING0_DA, 4096, RPMSG_VQ0_SIZE, 1, 0 },
    { RPMSG_VRING1_DA, 4096, RPMSG_VQ1_SIZE, 2, 0 },

    {
        TYPE_TRACE, TRACEBUFADDR, TRACEBUFSIZE, 0, "trace:dsp",
    },
    {
        TYPE_DEVMEM,
        DSP_CMEM_IOBUFS, PHYS_CMEM_IOBUFS,
        DSP_CMEM_IOBUFS_SIZE, 0, 0, "DSP_CMEM_IOBUFS",
    },
};

#endif /* _RSC_TABLE_DSP_H_ */
