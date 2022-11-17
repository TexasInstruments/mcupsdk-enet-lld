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
 * \file  enet_queue.h
 *
 * \brief This file contains the type definitions and helper macros for the
 *        Enet software queue.
 */

/*!
 * \ingroup  DRV_ENET_MODULE
 * \defgroup ENET_QUEUE_API Enet Queue API
 *
 * @{
 */

#ifndef ENET_QUEUE_H_
#define ENET_QUEUE_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief A generic node structure for a single link list.
 */
typedef struct EnetQ_Node_s
{
    /*! Pointer to the next element in the list */
    struct EnetQ_Node_s *next;
} EnetQ_Node;

/*!
 * \brief Generic queue.
 */
typedef struct EnetQ_s
{
    /*! The current number of entries in the queue */
    uint32_t count;

    /*! The top of the queue */
    EnetQ_Node *head;

    /*! The end of the queue */
    EnetQ_Node *tail;

    /*! Magic number used to check queue initialization */
    uint32_t magic;
} EnetQ;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief Init queue.
 *
 * Initializes a queue.
 *
 * \param queue   Queue pointer
 */
void EnetQueue_initQ(EnetQ *queue);

/*!
 * \brief Copy queues.
 *
 * Copies queues.  The copied queue will have the same head/tail pointers and
 * count as the source queue.
 *
 * \param dstQueue      Destination queue pointer
 * \param srcQueue      Source queue pointer
 */
void EnetQueue_copyQ(EnetQ *dstQueue,
                     const EnetQ *srcQueue);

/*!
 * \brief Enqueue a packet into the queue.
 *
 * Enqueues a packet into the queue.
 *
 * \param queue    Queue pointer
 * \param node     Queue node pointer
 */
void EnetQueue_enq(EnetQ *queue,
                   EnetQ_Node *node);

/*!
 * \brief Enqueue a packet at head into the queue.
 *
 * Enqueues a packet into head of queue.
 *
 * \param queue    Queue pointer
 * \param node     Queue node pointer
 */
void EnetQueue_enqHead(EnetQ *queue,
                       EnetQ_Node *node);

/*!
 * \brief Dequeue a packet from the queue
 *
 * Dequeues a packet from the queue.  A null pointer is
 * returned if the queue was already empty.
 *
 * \param queue    Queue pointer
 *
 * \return Queue node pointer or NULL if queue was empty
 */
EnetQ_Node *EnetQueue_deq(EnetQ *queue);

/*!
 * \brief Append queue.
 *
 * Appends a packet queue into another queue.  The packets in the source
 * queue are queued to the tail of the destination queue.
 *
 * \param dstQueue      Destination queue node pointer
 * \param srcQueue      Source queue pointer
 */
void EnetQueue_append(EnetQ *dstQueue,
                      EnetQ *srcQueue);

/*!
 * \brief Get queue count.
 *
 * Gets the number of packets in the queue.
 *
 * \param queue    Queue pointer
 *
 * \return Number of packets in the queue
 */
uint32_t EnetQueue_getQCount(EnetQ *queue);

/*!
 * \brief Checks queue corruption.
 *
 * Debug API which verifies the queue memory corruption by checking queue count
 * to actual elements in queue. Asserts if queue count doesn't match.
 *
 * \param queue    Queue pointer
 */
void EnetQueue_verifyQCount(EnetQ *queue);

/* ========================================================================== */
/*                        Deprecated Function Declarations                    */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* ENET_QUEUE_H_ */

/*! @} */
