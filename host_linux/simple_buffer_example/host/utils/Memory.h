/** 
 *  @file   Memory.h
 *
 *  @brief      Memory manager interface definitions.
 *
 *              This abstracts the Memory management interface to be used with
 *              SysLink code. Allocation, Freeing-up, copy and address translate
 *              are supported for the memory management.
 *
 *
 */
/* 
 *  ============================================================================
 *
 *  Copyright (c) 2008-2017, Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *  
 *  *  Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  
 *  *  Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  
 *  *  Neither the name of Texas Instruments Incorporated nor the names of
 *     its contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *  
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 *  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 *  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 *  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *  Contact information for paper mail:
 *  Texas Instruments
 *  Post Office Box 655303
 *  Dallas, Texas 75265
 *  Contact information: 
 *  http://www-k.ext.ti.com/sc/technical-support/product-information-centers.htm?
 *  DCMP=TIHomeTracking&HQS=Other+OT+home_d_contact
 *  ============================================================================
 *  
 */



#ifndef MEMORY_H_0xC97E
#define MEMORY_H_0xC97E

/* Module headers */
#include "IHeap.h"


#if defined (__cplusplus)
extern "C" {
#endif


/* =============================================================================
 * Macros
 * =============================================================================
 */
/*!
 *  @brief   Blocking quality
 *           Heaps with this "quality" may cause the calling thread to block;
 *           i.e., suspend execution until another thread leaves the gate.
 */
#define Memory_Q_BLOCKING 1

/* =============================================================================
 * Structures & Enums. See MemoryDefs.h
 * =============================================================================
 */

/* =============================================================================
 *  APIs
 * =============================================================================
 */
/*!
 *  @brief      Allocates the specified number of bytes.
 *
 *  @param      heap    Handle to the heap from which the memory is to be
 *                      allocated. Specify NULL to allocate from local memory.
 *  @param      size    Amount of memory to be allocated.
 *  @param      align   Alignment constraints (power of 2)
 *  @param      eb      Not used. Pass as NULL.
 *
 *  @retval     Pointer Success: Pointer to allocated buffer
 *  @retval     NULL    Failed to allocate memory
 *
 *  @sa         Memory_calloc
 */
Ptr Memory_alloc (IHeap_Handle heap, SizeT size, SizeT align, Ptr eb);

/*!
 *  @brief      Allocates the specified number of bytes and memory is set to
 *              zero.
 *
 *  @param      heap    Handle to the heap from which the memory is to be
 *                      allocated. Specify NULL to allocate from local memory.
 *  @param      size    Amount of memory to be allocated.
 *  @param      align   Alignment constraints (power of 2)
 *  @param      eb      Not used. Pass as NULL.
 *
 *  @retval     Pointer Success: Pointer to allocated buffer
 *  @retval     NULL    Failed to allocate memory
 *
 *  @sa         Memory_alloc, MemoryOS_calloc
 */
Ptr Memory_calloc (IHeap_Handle heap, SizeT size, SizeT align, Ptr eb);

/*!
 *  @brief      Frees up the specified chunk of memory.
 *
 *  @param      heap    Handle to the heap
 *  @param      block   Block of memory to be freed
 *  @param      size    Amount of memory to be freed
 *
 *  @sa         IHeap_free, MemoryOS_free
 */
Void Memory_free (IHeap_Handle heap, Ptr block, SizeT size);

/*!
 *  @brief      Function to obtain statistics from a heap.
 *
 *  @param      heap    Handle to the heap
 *  @param      stats   Pointer to the Memory stats structure to be filled.
 *
 *  @sa
 */
Void Memory_getStats (IHeap_Handle heap, Memory_Stats * stats);

/*!
 *  @brief      Function to test for a particular IHeap quality
 *
 *  @param      heap    Handle to the heap
 *  @param      qual    Quality to be queried.
 *
 *  @sa         Memory_Q_BLOCKING
 */
Bool Memory_query (IHeap_Handle heap, Int qual);

/*!
 *  @brief      Function to get memory alignment
 *
 *  @retval     Alignment Maximum default alignment
 */
SizeT Memory_getMaxDefaultTypeAlign (Void);

/*!
 *  @brief      Allocates the specified number of bytes and memory is set to
 *              the specified value.
 *
 *  @param      heap    Handle to the heap from which the memory is to be
 *                      allocated. Specify NULL to allocate from local memory.
 *  @param      size    Amount of memory to be allocated.
 *  @param      align   Alignment constraints (power of 2)
 *  @param      value   Value to be set for all bytes in the buffer
 *  @param      eb      Not used. Pass as NULL.
 *
 *  @retval     Pointer Success: Pointer to allocated buffer
 *  @retval     NULL    Failed to allocate memory
 *
 *  @sa         Memory_alloc, MemoryOS_calloc
 */
Ptr Memory_valloc (IHeap_Handle heap,
                   SizeT        size,
                   SizeT        align,
                   Char         value,
                   Ptr          eb);


/* =============================================================================
 *  APIs that are added for MemoryOS
 * =============================================================================
 */

/*!
 *  @brief      Copies the data between memory areas.
 */
#define Memory_copy          memcpy

/*!
 *  @brief      Set the specific value in the said memory area.
 */
#define Memory_set           memset


#if defined (__cplusplus)
}
#endif /* defined (__cplusplus) */

#endif /* ifndef MEMORY_H_0xC97E */
