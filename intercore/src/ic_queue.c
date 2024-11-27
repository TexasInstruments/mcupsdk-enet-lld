/*
 *  Copyright (c) Texas Instruments Incorporated 2025
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
 * \file  ic_queue.c
 *
 * \brief This file contains the implementation of a circular queue.
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
#include <ic_queue.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/*! \brief Magic vlaue used to indicate when a queue has been initialized. */
#define IC_QUEUE_INIT_DONE               (0xABCDABCDU)

/*! \brief Assert wrapper */
#if defined(__KLOCWORK__) || defined(__cplusplus)
#define IcQueue_assert(cond, ...)        do { if (!(cond)) abort(); } while (0)
#else
#define IcQueue_assert(cond, ...)        assert(cond)
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

/* None */

/* ========================================================================== */
/*                          Private Functions                                 */
/* ========================================================================== */

/* Used when an element is enqueued */
static void IcQueue_moveHead(IcQ_Handle hIcQueue)
{
    IcQueue_assert(hIcQueue != NULL);
    IcQueue_assert(hIcQueue->magic == IC_QUEUE_INIT_DONE);

    if (IcQueue_isQFull(hIcQueue))
    {
        if (++(hIcQueue->tail) == hIcQueue->maxSize)
        {
            hIcQueue->tail = 0;
        }
    }

    if (++(hIcQueue->head) == hIcQueue->maxSize)
    {
        hIcQueue->head = 0;
    }
}

/* Used when an element is dequeued */
static void icQueue_moveTail(IcQ_Handle hIcQueue)
{
    IcQueue_assert(hIcQueue != NULL);
    IcQueue_assert(hIcQueue->magic == IC_QUEUE_INIT_DONE);

    if (++(hIcQueue->tail) == hIcQueue->maxSize)
    {
        hIcQueue->tail = 0;
    }
}

/* ========================================================================== */
/*                          API/Public Functions                              */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief Creates a circular queue and returns a handle to it
 *
 * \detail Creates a circular queue of \ref IcQ_Node objects.
 *  - The number of nodes on the circular queue is passed as a parameter.
 *
 *  @param[in]  hIcQueue        Handle to a pre-allocated icQueue object
 *  @param[in]  queueId         Unique queue identifier
 *  @param[in]  maxSize         Number of nodes on the circular queue
 *
 *  @return IcQueue Handle upon success. Error code otherwise.
 */
int32_t IcQueue_initQ(IcQ_Handle hIcQueue,
                      uint32_t queueId,
                      uint32_t maxSize)
{
    int32_t retVal = ICQ_RETURN_SUCCESS;

    IcQueue_assert(hIcQueue && maxSize);

    hIcQueue->queueId       = queueId;
    hIcQueue->head          = 0;
    hIcQueue->tail          = 0;
    hIcQueue->maxSize       = maxSize;
    hIcQueue->magic         = IC_QUEUE_INIT_DONE;

    /* Clear all the nodes */
    memset(hIcQueue->nodeArray, 0x00, maxSize * sizeof(IcQ_Node));

    return retVal;
}

/*!
 * \brief De-allocates a circular queue object
 *
 *  @param[in] hIcQueue     Handle to IcQ object
 *
 *  @return None.
 */
void IcQueue_freeQ(IcQ_Handle hIcQueue)
{
    IcQueue_assert(hIcQueue != NULL);
    IcQueue_assert(hIcQueue->magic == IC_QUEUE_INIT_DONE);

    /*@TODO: Replace free with osal function
     * or have the application own the IcQ_Handle and free it*/
    free(hIcQueue);
}

/*!
 * \brief Resets a circular queue
 *
 * \detail Reset the head and tail pointers of a circular queue.
 *
 * @param[in] hIcQueue      Handle to IcQ object 
 *
 * @return None
 */
void IcQueue_resetQ(IcQ_Handle hIcQueue)
{
    IcQueue_assert(hIcQueue != NULL);
    IcQueue_assert(hIcQueue->magic == IC_QUEUE_INIT_DONE);

    hIcQueue->head = 0;
    hIcQueue->tail = 0;
}

/*!
 * \brief Returns the current no. of objects on the given queue
 *
 * @param[in] hIcQueue      Handle to IcQ object
 *
 * @return Number of \ref IcQ_Node objects pushed onto the queue.
 */
uint32_t IcQueue_getQCount(IcQ_Handle hIcQueue)
{
    uint32_t count;

    IcQueue_assert(hIcQueue != NULL);
    IcQueue_assert(hIcQueue->magic == IC_QUEUE_INIT_DONE);

    count = hIcQueue->maxSize;

    if (!IcQueue_isQFull(hIcQueue))
    {
        if (hIcQueue->head >= hIcQueue->tail)
        {
            count = (hIcQueue->head - hIcQueue->tail);
        }
        else
        {
            count = (hIcQueue->maxSize + hIcQueue->head - hIcQueue->tail);
        }

    }

    return count;
}

/*!
 * \brief Returns the maximum capacity of the given queue
 *
 * @param[in] hIcQueue      Handle to IcQ object
 *
 * @return Maximum capacity i.e. the size of a queue.
 */
uint32_t IcQueue_getQMaxSize(IcQ_Handle hIcQueue)
{
    IcQueue_assert(hIcQueue != NULL);
    IcQueue_assert(hIcQueue->magic == IC_QUEUE_INIT_DONE);

    return hIcQueue->maxSize;
}

/*!
 * \brief Enqueues an \ref IcQ_Node object on the given queue
 *
 * @param[in] hIcQueue      Handle to IcQ object
 * @param[in] pNode         Handle to IcQ_Node object
 *
 * @return ICQ_RETURN_SUCCESS if enqueue is successful
 * @return ICQ_ERR_QFULL if queue is full
 */
int32_t IcQueue_enq(IcQ_Handle hIcQueue,
                    IcQ_Node *pNode)
{
    int32_t retVal = ICQ_RETURN_FAILURE;

    IcQueue_assert(hIcQueue && pNode);
    IcQueue_assert(hIcQueue->magic == IC_QUEUE_INIT_DONE);

    if (!IcQueue_isQFull(hIcQueue))
    {
        hIcQueue->nodeArray[hIcQueue->head].pDataBuffer   = pNode->pDataBuffer;
        hIcQueue->nodeArray[hIcQueue->head].dataBufferLen = pNode->dataBufferLen;
        hIcQueue->nodeArray[hIcQueue->head].packetId      = pNode->packetId;
        IcQueue_moveHead(hIcQueue);
        retVal = ICQ_RETURN_SUCCESS;
    }
    else
    {
        retVal = ICQ_ERR_QFULL;
    }

    return retVal;
}

/*!
 * \brief Dequeues a node from the given queue
 *
 * @param[in] hIcQueue      Handle to IcQ object
 *
 * @return The next \ref IcQ_Node object on the queue
 * @return NULL if the queue is empty
 */
IcQ_Node* IcQueue_deq(IcQ_Handle hIcQueue)
{
    IcQ_Node* pNode = NULL;

    IcQueue_assert(hIcQueue != NULL);
    IcQueue_assert(hIcQueue->magic == IC_QUEUE_INIT_DONE);

    if (!IcQueue_isQEmpty(hIcQueue))
    {
        pNode = &hIcQueue->nodeArray[hIcQueue->tail];
        icQueue_moveTail(hIcQueue);
    }

    return pNode;
}

/*!
 * \brief Returns BTRUE if the given queue is empty
 *
 * @param[in] hIcQueue      Handle to IcQ object
 *
 * @return True if a queue is empty.
 */
bool IcQueue_isQEmpty(IcQ_Handle hIcQueue)
{
    IcQueue_assert(hIcQueue != NULL);
    IcQueue_assert(hIcQueue->magic == IC_QUEUE_INIT_DONE);

    return (!IcQueue_isQFull(hIcQueue) && (hIcQueue->head == hIcQueue->tail));
}

/*!
 * \brief Returns BTURE if the given queue is full
 *
 * @param[in] hIcQueue      Handle to IcQ object
 *
 * @return True if a queue is full.
 */
bool IcQueue_isQFull(IcQ_Handle hIcQueue)
{
    uint32_t head;

    IcQueue_assert(hIcQueue != NULL);
    IcQueue_assert(hIcQueue->magic == IC_QUEUE_INIT_DONE);

    /* Check for wrap-around. This approach wastes one slot in the
     * queue but does not require producer-consumer synchronization
     */ 
    head = hIcQueue->head + 1;

    if (head == hIcQueue->maxSize)
    {
        head = 0;
    }

    return (head == hIcQueue->tail);
}

/*!
 * \brief Returns BTRUE if the given queue is initialized
 *
 * @param[in] hIcQueue      Handle to IcQ object
 *
 * @return True if the given queue is valid.
 */
bool IcQueue_isQValid(IcQ_Handle hIcQueue)
{
    bool retVal = BFALSE;

    IcQueue_assert(hIcQueue != NULL);

    if (IC_QUEUE_INIT_DONE == hIcQueue->magic)
    {
        retVal = BTRUE;
    }

    return retVal;
}
