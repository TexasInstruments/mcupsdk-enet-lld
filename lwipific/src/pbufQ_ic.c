/*
 *  Copyright (C) 2018-2024 Texas Instruments Incorporated
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

#include <assert.h>
#include <stdbool.h>
#include "lwip/pbuf.h"
#include "pbufQ_ic.h"
#include <kernel/dpl/HwiP.h>
/*
 * Free queue from which buffer pointers are allocated
 */
pbufIcQ pfreeQ_ic;
static bool gPbufQ_initialized = false;

static inline void pbufQ_ic_assert(uint8_t cond)
{
    while(!cond)
        {}
    ;
}

/*
 * Initializes the freeQ.
 * MUST BE CALLED BEFORE INITIALIZING THE FIRST BUFFER QUEUE
 * Needs to be called only once as early in the program as possible
 */
//TODO: Refractor using enQ/deQ
void pbufQ_ic_init_freeQ(pbufIcNode *pfree, uint32_t maxSize)
{
    int fQ_iter;
    pbufQ_ic_assert(NULL != pfree);

	if (!gPbufQ_initialized)
    {
        for(fQ_iter=0; fQ_iter<maxSize-1; fQ_iter++)
        {
            pfree[fQ_iter].next = &pfree[fQ_iter+1];
            pfree[fQ_iter].hPbufPkt = NULL;
        }

        pfree[fQ_iter].next = NULL;
        pfree[fQ_iter].hPbufPkt = NULL;

        pfreeQ_ic.head = &pfree[0];
        pfreeQ_ic.tail = &pfree[maxSize - 1];
        pfreeQ_ic.count = maxSize;

        gPbufQ_initialized = true;
    }
}

/*
 * Allocates memory from the freeQ to be used by other queues
 */
pbufIcNode* mempQ_ic_malloc()
{
    pbufIcNode* p = pfreeQ_ic.head;
    pfreeQ_ic.head = pfreeQ_ic.head->next;
    pfreeQ_ic.count--;
    return p;
}

/*
 * Returns buffer pointers used by other queues back to freeQ
 */
void mempQ_ic_free(pbufIcNode* p)
{
    p->next = NULL;
    p->hPbufPkt = NULL;
    pfreeQ_ic.tail->next = p;
    pfreeQ_ic.tail = p;
    if(pfreeQ_ic.count ==0)
    {
        pfreeQ_ic.head = pfreeQ_ic.tail;
    }
    pfreeQ_ic.count++;
}

/*
 * Initializes a queue. Must be called after declaring a queue
 */
void pbufQ_ic_init(pbufIcQ *pQ)
{
    uint32_t key = HwiP_disable();
    pQ->head = NULL;
    pQ->tail = NULL;
    pQ->count = 0;
    HwiP_restore(key);
}

/*
 * Enqueues a buffer to the tail of the queue
 */
void pbufQ_ic_enQ(pbufIcQ *pQ, struct pbuf *p)
{
    pbufIcNode* temp = NULL;

    uint32_t key = HwiP_disable();

    temp = mempQ_ic_malloc();
    pbufQ_ic_assert(NULL != temp);

    temp->hPbufPkt = p;
    temp->next = NULL;

    if(pQ->count == 0)
    {
        pQ->head = temp;
        pQ->tail = temp;
    }
    else if(pQ->count == 1)
    {
        pQ->head->next = temp;
        pQ->tail = temp;
    }
    else
    {
        pQ->tail->next = temp;
        pQ->tail = pQ->tail->next;
    }
    pQ->count++;

    HwiP_restore(key);

}

/*
 * Enqueues a packet to the head of the queue.
 * NOT USED ANYWHERE. Can be removed
 */
void pbufQ_ic_enQHead(pbufIcQ *pQ, struct pbuf *p)
{
    pbufIcNode* temp = NULL;
    pbufQ_ic_assert(p != NULL);

    temp = mempQ_ic_malloc();
    pbufQ_ic_assert(NULL != temp);

    uint32_t key = HwiP_disable();
    temp->hPbufPkt = p;
    temp->next = NULL;
    if(pQ->count == 0)
    {
        pQ->head = temp;
        pQ->tail = temp;
    }
    else
    {
        temp->next = pQ->head;
        pQ->head = temp;
    }
    HwiP_restore(key);

}

/*
 * Dequeues from the queue
 */
struct pbuf* pbufQ_ic_deQ(pbufIcQ *pQ)
{
    struct pbuf *rtnPbuf = NULL;
    pbufIcNode *temp = NULL;
    uint32_t key = HwiP_disable();

    if(pQ->count != 0)
    {
        rtnPbuf = pQ->head->hPbufPkt;
        temp = pQ->head;
        pQ->head = pQ->head->next;
        if(pQ->count == 1)
        {
            pQ->tail = NULL;
        }
        pQ->count--;

        pbufQ_ic_assert(rtnPbuf != NULL);
        mempQ_ic_free(temp);
    }

    HwiP_restore(key);

    return rtnPbuf;
}
