/**
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */

/**
 * Copyright (c) 2023 Texas Instruments Incorporated
 *
 * Functions to handle Rx Custom Pbuf: custom_pbuf_free is called from LwIP stack hook.
 *
 */

#include "custom_pbuf.h"
#include "lwip2enet.h"

static inline void custom_pbuf_init(Rx_CustomPbuf *cPbuf)
{
    cPbuf->next            = NULL;
    cPbuf->alivePbufCount  = 0U;
    cPbuf->orgBufLen       = 0U;
    cPbuf->orgBufPtr       = NULL;
}

#if (1U == ENET_CFG_DEV_ERROR)
static void custom_pbuf_validateChain(Rx_CustomPbuf *cPbuf)
{
    /* Loop through the cPbuf chain and make sure the alivePbufCount is same for every cPbuf */
    Rx_CustomPbuf *start = cPbuf;
    uint32_t alivePbufCount = cPbuf->alivePbufCount;
    cPbuf = cPbuf->next;
    while(start != cPbuf)
    {
        if(cPbuf->alivePbufCount != alivePbufCount)
        {
            Lwip2Enet_assert(false);
        }
        cPbuf = cPbuf->next;
    }
    Lwip2Enet_assert(start == cPbuf);
}
#endif

/*!
 *  @b custom_pbuf_free
 *  @n
 *  This function is called from pbuf_free for pbufs having custom flag set.
 *  If the pbuf p is the last pbuf (i.e pbuf_free is called on all the pbufs in the chain)
 *  in the chain, the whole pbuf chain (this corresponds to one pkt) is freed.
 *  To access the other pbufs in the chain from the last pbuf, custom pbuf structure
 *  stores the next cPbuf pointer in a circular fashion.
 *
 *  To know if the pbuf p is the last pbuf in the pkt, we can't depend on pbuf->next == NULL or
 *  pbuf->len == pbuf->tot_len, as in some cases the LwIP stack concatenates the pbufs of different
 *  pkts(for eg: in tcp ooseq handling, pbuf_cat is called on all the ooseq pbufs). For this purpose,
 *  we maintain alivePbufCount for the pbufs of a pkt, alivePbufCount == 0 indicates that pbuf_free
 *  is called on all the pbufs of the pkt and the pbufs can be freed and a pkt can be submitted back.
 *
 *  \param[in]  p
 *      the pbuf to be freed.
 *
 *  \retval
 *      NONE
 */
void custom_pbuf_free(struct pbuf *p)
{
    Rx_CustomPbuf *cPbuf = (Rx_CustomPbuf*)p;
    Rx_CustomPbuf *start = cPbuf;
    EnetDma_SGListEntry *list = NULL;
    uint32_t scatterSegmentIndex = 0;
    Lwip2Enet_RxObj *rx = (Lwip2Enet_RxObj *) cPbuf->customPbufArgs;
    Rx_CustomPbuf *cPbufNext = NULL;

#if (1U == ENET_CFG_DEV_ERROR)
    custom_pbuf_validateChain(cPbuf);
    Lwip2Enet_assert(cPbuf->alivePbufCount != 0);
    Lwip2Enet_assert(cPbuf->next != NULL);
#endif

    /* Decrement the alivePbufCount of the every cPbuf in the chain */
    start->alivePbufCount--;
    cPbuf = cPbuf->next;
    while(start != cPbuf)
    {
        cPbuf->alivePbufCount--;
        cPbuf = cPbuf->next;
    }
    Lwip2Enet_assert(start == cPbuf);
    if(cPbuf->alivePbufCount == 0)
    {
        /* This pbuf chain is no longer in use. */
        /* Loop through the cPbuf chain and enq in a dmapktinfo. */
        EnetDma_Pkt *pDmaPacket =  (EnetDma_Pkt *)EnetQueue_deq(&rx->freeRxPktInfoQ);
        LWIP2ENETSTATS_ADDONE(&rx->stats.freeAppPktDeq);
        Lwip2Enet_assert(pDmaPacket != NULL);
        EnetDma_checkPktState(&pDmaPacket->pktState,
                               ENET_PKTSTATE_MODULE_APP,
                               ENET_PKTSTATE_APP_WITH_FREEQ,
                               ENET_PKTSTATE_APP_WITH_READYQ);
        do {
            list = &pDmaPacket->sgList.list[scatterSegmentIndex];
            list->bufPtr = cPbuf->orgBufPtr;
            list->segmentFilledLen = 0;
            Lwip2Enet_assert(cPbuf->orgBufLen != 0);
            list->segmentAllocLen  = cPbuf->orgBufLen;
            cPbufNext = cPbuf->next;
            custom_pbuf_init(cPbuf);
            /* Enqueue the pbuf into freePbufInfoQ */
            pbufQ_enQ(&rx->freePbufInfoQ, (struct pbuf *)cPbuf);
            LWIP2ENETSTATS_ADDONE(&rx->stats.freePbufPktEnq);
            scatterSegmentIndex++;
            Lwip2Enet_assert(scatterSegmentIndex <= ENET_ARRAYSIZE(pDmaPacket->sgList.list));
            cPbuf = cPbufNext;
        } while(start != cPbuf);

        pDmaPacket->sgList.numScatterSegments = scatterSegmentIndex;
        EnetQueue_enq(&rx->readyRxPktQ, &pDmaPacket->node);
    }
}
