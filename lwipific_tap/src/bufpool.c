/*
 *  Copyright (c) Texas Instruments Incorporated 2020
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*!
 * \file     bufpool.c
 *
 * \brief    Implements a simple buffer pool
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <assert.h>
#include <string.h>
#include <stdbool.h>
#include <tistdtypes.h>

#include <bufpool.h>

/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */

/*! \brief Magic vlaue used to check if a bufpool has been initialized. */
#define BUFPOOL_INIT_DONE               (0xABCDABCDU)

/*! \brief Assert wrapper */
#if defined(__KLOCWORK__) || defined(__cplusplus)
#define BufPool_assert(cond, ...)        do { if (!(cond)) abort(); } while (0)
#else
#define BufPool_assert(cond, ...)        assert(cond)
#endif

/* ========================================================================== */
/*                         Structures Declarations                            */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/*!
 * \brief System-wide inter-core bufpool handle
 * 
 *  Handle to the system-wide inter-core buffer pool table. All cores must
 *  refer to their respective buffer pools using this handle.
 *
 *  This data structure must be allocated in non-cacheable memory or kept
 *  cache-coherent
 */
 
BufPool_Pool BufPool_globalTable[BUFPOOL_MAX_POOLS]
__attribute__ ((section("intercore_eth_data_mem")))
__attribute__ ((aligned(128)));

BufPool_Handle BufPoolTable_Handle = (BufPool_Handle)&(BufPool_globalTable[0]);

/* ========================================================================== */
/*                          Private Functions                                 */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          API/Public Functions                              */
/* ========================================================================== */

/*!
 * \brief Initializes a buffer pool
 *
 */
int32_t BufPool_init(BufPool_Handle hBufPool,
                     uint32_t poolId,
                     uint32_t maxSize)
{
    int32_t bufIdx  = 0;
    int32_t retVal = BUFPOOL_OK;

    BufPool_assert(hBufPool && (maxSize<=BUFPOOL_BUF_MAX));

    hBufPool->poolId    = poolId;
    hBufPool->lastAlloc = 0;
    hBufPool->numBufGet = 0;
    hBufPool->numBufFree   = 0;
    hBufPool->numBufGetErr = 0;
    hBufPool->maxSize   = maxSize;
    hBufPool->magic     = BUFPOOL_INIT_DONE;

    /* Clear all the buffers */
    memset(hBufPool->buf_array, 0x00, maxSize * sizeof(BufPool_Buf));

    for (bufIdx = 0; bufIdx < maxSize; bufIdx++)
    {
        hBufPool->buf_array[bufIdx].poolId = poolId;
    }

    return retVal;
}

/*!
 * \brief Returns a pointer to a free buffer
 *
 */
BufPool_Buf* BufPool_getBuf(BufPool_Handle hBufPool)
{
    int32_t idx;
    bool found = FALSE;
    BufPool_Buf* hBuf = NULL;

    BufPool_assert(hBufPool->magic == BUFPOOL_INIT_DONE);

    /* Start search at the last allocated node */

    /* Search from last alloc to end of pool */
    for (idx = hBufPool->lastAlloc; idx < hBufPool->maxSize; idx++)
    {
        if (0 == hBufPool->buf_array[idx].isUsed)
        {
            /* Mark the buffer used and store it's index */
            hBufPool->buf_array[idx].isUsed = 1;
            hBufPool->buf_array[idx].refCount = 1;
            hBufPool->lastAlloc = idx;
            hBufPool->numBufGet++;
            hBuf = &(hBufPool->buf_array[idx]);
            break;
        }
    }

    /* Wrap around: search from 0 to last alloc */
    if (!found) /* Still looking */
    {
        if (idx == hBufPool->maxSize)
        {
            /* Search from 0 to last alloc */
            for (idx = 0; idx < hBufPool->lastAlloc; idx++)
            {
                if (0 == hBufPool->buf_array[idx].isUsed)
                {
                    /* Mark the buffer used and store it's index */
                    hBufPool->buf_array[idx].isUsed = 1;
                    hBufPool->buf_array[idx].refCount = 1;
                    hBufPool->lastAlloc = idx;
                    hBufPool->numBufGet++;
                    hBuf = &(hBufPool->buf_array[idx]);
                    //foundIndex = idx;
                    break;
                }
            }

            /* Search failed */
            hBufPool->numBufGetErr++;
        }
    }

    return hBuf;
}

/*!
 * \brief Decrements the ref count of a buffer object
 *
 * If the refCount reaches zero, free the pool buffer
 *
 */
int32_t BufPool_freeBuf(BufPool_Buf* hBuf)
{
    int32_t retVal = BUFPOOL_OK;

    BufPool_assert(hBuf && (hBuf->refCount >=0));

    if (hBuf->refCount > 0)
    {
        --(hBuf->refCount);

        if (0 == hBuf->refCount)
        {
            /* Mark the buffer free */
            hBuf->isUsed = 0;
        }
    }

    /*
     * Here we don't have a direct bufpool handle so
     * refer to the pool using the global table and
     * the poolId available in hBuf
     */
    BufPoolTable_Handle[hBuf->poolId].numBufFree++;
    return retVal;
}
