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
 * \file  enet_queue.c
 *
 * \brief This file contains the implementation of the Enet Queue.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <enet_cfg.h>
#include <include/core/enet_osal.h>
#include <include/core/enet_utils.h>
#include <include/core/enet_queue.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/*! \brief Magic vlaue used to indicate when a queue has been initialized. */
#define ENET_Q_INIT_DONE                      (0xABCDABCDU)

/* ========================================================================== */
/*                         Structure Declarations                             */
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
/*                          Function Definitions                              */
/* ========================================================================== */

void EnetQueue_initQ(EnetQ *queue)
{
    queue->count = 0;
    queue->head  = NULL;
    queue->tail  = NULL;
    queue->magic = ENET_Q_INIT_DONE;
}

void EnetQueue_copyQ(EnetQ *dstQueue,
                     const EnetQ *srcQueue)
{
    uintptr_t key = EnetOsal_disableAllIntr();

    Enet_devAssert(srcQueue->magic == ENET_Q_INIT_DONE,
                   "Source queue is not initialized\n");

    dstQueue->magic = ENET_Q_INIT_DONE;
    dstQueue->head  = srcQueue->head;
    dstQueue->tail  = srcQueue->tail;
    dstQueue->count = srcQueue->count;

    EnetOsal_restoreAllIntr(key);
}

void EnetQueue_enq(EnetQ *queue,
                   EnetQ_Node *node)
{
    uintptr_t key = EnetOsal_disableAllIntr();

    Enet_devAssert(queue->magic == ENET_Q_INIT_DONE,
                   "Queue is not initializated\n");

    node->next = NULL;

    if (0U == queue->count)
    {
        /* Queue is empty - Initialize it with this one node */
        queue->head = node;
    }
    else
    {
        /* Queue is not empty - Push onto tail */
        queue->tail->next = node;
    }

    /* Make tail of queue point to new */
    queue->tail = node;
    queue->count++;

    EnetOsal_restoreAllIntr(key);
}

void EnetQueue_enqHead(EnetQ *queue,
                       EnetQ_Node *node)
{
    uintptr_t key = EnetOsal_disableAllIntr();

    Enet_devAssert(queue->magic == ENET_Q_INIT_DONE,
                   "Queue is not initialized\n");

    if (0U == queue->count)
    {
        queue->tail = node;
    }

    node->next = queue->head;
    queue->head = node;
    queue->count++;

    EnetOsal_restoreAllIntr(key);
}

EnetQ_Node *EnetQueue_deq(EnetQ *queue)
{
    EnetQ_Node *node;
    uintptr_t key = EnetOsal_disableAllIntr();

    Enet_devAssert(queue->magic == ENET_Q_INIT_DONE,
                   "Queue is not initialized\n");

    node = queue->head;

    if (NULL != node)
    {
        queue->head = node->next;
        if (NULL == queue->head)
        {
            queue->tail = NULL;
        }

        queue->count--;
        node->next = NULL;
    }

    EnetOsal_restoreAllIntr(key);

    return node;
}

void EnetQueue_append(EnetQ *dstQueue,
                      EnetQ *srcQueue)
{
    uintptr_t key = EnetOsal_disableAllIntr();

    Enet_devAssert(dstQueue->magic == ENET_Q_INIT_DONE,
                   "Destination queue is not initialized\n");
    Enet_devAssert(srcQueue->magic == ENET_Q_INIT_DONE,
                   "Source queue is not initialized\n");

    if (0U != srcQueue->count)
    {
        if (NULL == dstQueue->head)
        {
            /* queue is empty - Initialize it with the source queue */
            EnetQueue_copyQ(dstQueue, srcQueue);
        }
        else
        {
            /* queue is not empty - Add free queue contents onto end */
            dstQueue->tail->next = srcQueue->head;
            dstQueue->tail   = srcQueue->tail;
            dstQueue->count += srcQueue->count;
        }
    }

    EnetOsal_restoreAllIntr(key);
}

uint32_t EnetQueue_getQCount(EnetQ *queue)
{
    Enet_devAssert(queue->magic == ENET_Q_INIT_DONE,
                   "Queue is not initialized\n");

    return queue->count;
}

void EnetQueue_verifyQCount(EnetQ *queue)
{
#if ENET_CFG_IS_ON(DEV_ERROR)
    EnetQ_Node *node;
    uint32_t count = 0U;
    uintptr_t key = EnetOsal_disableAllIntr();

    Enet_devAssert(queue->magic == ENET_Q_INIT_DONE);
    node = queue->head;

    while (NULL != node)
    {
        count++;
        node = node->next;
    }

    Enet_devAssert(queue->count == count);
    EnetOsal_restoreAllIntr(key);
#endif
}
