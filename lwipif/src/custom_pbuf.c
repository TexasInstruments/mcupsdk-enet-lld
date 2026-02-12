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
extern volatile uint32_t gLwipDebugTracker;
extern volatile uint32_t gLwipPktType;

/* Debug: Track pbuf free calls */
volatile uint32_t gPbufFreeCount = 0;
volatile uint32_t gPbufAliveAtEntry = 0;
volatile uint32_t gPbufAliveAtExit = 0;
volatile uint32_t gScatterSegCount = 0;
volatile uint32_t gCustomPbufPtr = 0;
volatile uint32_t gCustomPbufArgsPtr = 0;

void custom_pbuf_free(struct pbuf *p)
{
    /* Track which packet type is being freed */
    if (gLwipPktType == 1)
        gLwipDebugTracker = 0x3000; /* FREE: ARP */
    else if (gLwipPktType == 2)
        gLwipDebugTracker = 0x3010; /* FREE: ICMP */
    else
        gLwipDebugTracker = 0x3020; /* FREE: Other */

    gPbufFreeCount++;

    Rx_CustomPbuf *cPbuf = (Rx_CustomPbuf*)p;
    Rx_CustomPbuf *start = cPbuf;
    EnetDma_SGListEntry *list = NULL;
    uint32_t scatterSegmentIndex = 0;

    /* Capture pbuf pointer values for debugging */
    gCustomPbufPtr = (uint32_t)cPbuf;
    if (cPbuf != NULL)
    {
        gCustomPbufArgsPtr = (uint32_t)cPbuf->customPbufArgs;
        gPbufAliveAtEntry = cPbuf->alivePbufCount;
    }

    if (gLwipPktType == 1)
        gLwipDebugTracker = 0x3001; /* ARP: Before assert */
    else if (gLwipPktType == 2)
        gLwipDebugTracker = 0x3011; /* ICMP: Before assert */
    else
        gLwipDebugTracker = 0x3021; /* Other: Before assert */

    /* CRITICAL: Check customPbufArgs before using it */
    Lwip2Enet_assert(cPbuf != NULL);
    Lwip2Enet_assert(cPbuf->customPbufArgs != NULL);

    if (gLwipPktType == 1)
        gLwipDebugTracker = 0x3002; /* ARP: After assert */
    else if (gLwipPktType == 2)
        gLwipDebugTracker = 0x3012; /* ICMP: After assert */
    else
        gLwipDebugTracker = 0x3022; /* Other: After assert */
    Lwip2Enet_RxObj *rx = (Lwip2Enet_RxObj *) cPbuf->customPbufArgs;
    Rx_CustomPbuf *cPbufNext = NULL;

#if (1U == ENET_CFG_DEV_ERROR)
    custom_pbuf_validateChain(cPbuf);
    Lwip2Enet_assert(cPbuf->alivePbufCount != 0);
    Lwip2Enet_assert(cPbuf->next != NULL);
#endif

    if (gLwipPktType == 1)
        gLwipDebugTracker = 0x3003; /* ARP: Before alive decrement */
    else if (gLwipPktType == 2)
        gLwipDebugTracker = 0x3013; /* ICMP: Before alive decrement */
    else
        gLwipDebugTracker = 0x3023; /* Other: Before alive decrement */

    /* Decrement the alivePbufCount of the every cPbuf in the chain */
    start->alivePbufCount--;
    cPbuf = cPbuf->next;

    /* SAFETY: Limit loop iterations to prevent infinite loop on corruption */
    uint32_t loopCount = 0;
    const uint32_t MAX_PBUF_SEGMENTS = 16; /* Safety limit */

    while(start != cPbuf)
    {
        loopCount++;
        if (loopCount > MAX_PBUF_SEGMENTS)
        {
            /* Circular list corruption detected! */
            gLwipDebugTracker = 0x3FFD; /* ERROR: Circular list corruption */
            Lwip2Enet_assert(false);
        }
        cPbuf->alivePbufCount--;
        cPbuf = cPbuf->next;
    }
    Lwip2Enet_assert(start == cPbuf);

    if (gLwipPktType == 1)
        gLwipDebugTracker = 0x3004; /* ARP: After alive decrement */
    else if (gLwipPktType == 2)
        gLwipDebugTracker = 0x3014; /* ICMP: After alive decrement */
    else
        gLwipDebugTracker = 0x3024; /* Other: After alive decrement */

    if(cPbuf->alivePbufCount == 0)
    {
        if (gLwipPktType == 1)
            gLwipDebugTracker = 0x3005; /* ARP: Freeing buffers */
        else if (gLwipPktType == 2)
            gLwipDebugTracker = 0x3015; /* ICMP: Freeing buffers */
        else
            gLwipDebugTracker = 0x3025; /* Other: Freeing buffers */
        /* This pbuf chain is no longer in use. */
        /* Loop through the cPbuf chain and enq in a dmapktinfo. */
        EnetDma_Pkt *pDmaPacket =  (EnetDma_Pkt *)EnetQueue_deq(&rx->freeRxPktInfoQ);
        LWIP2ENETSTATS_ADDONE(&rx->stats.freeAppPktDeq);
        Lwip2Enet_assert(pDmaPacket != NULL);
        EnetDma_checkPktState(&pDmaPacket->pktState,
                               ENET_PKTSTATE_MODULE_APP,
                               ENET_PKTSTATE_APP_WITH_FREEQ,
                               ENET_PKTSTATE_APP_WITH_READYQ);

        /* SAFETY: Track loop iterations to prevent buffer overflow */
        uint32_t recycleLoopCount = 0;
        const uint32_t MAX_SG_SEGMENTS = 16; /* Must match driver limit */

        do {
            recycleLoopCount++;
            if (recycleLoopCount > MAX_SG_SEGMENTS)
            {
                /* Buffer recycling loop corruption! */
                gLwipDebugTracker = 0x3FFC; /* ERROR: Recycle loop overflow */
                Lwip2Enet_assert(false);
            }

            /* Check scatter segment index before array access */
            if (scatterSegmentIndex >= ENET_ARRAYSIZE(pDmaPacket->sgList.list))
            {
                gLwipDebugTracker = 0x3FFB; /* ERROR: SG array overflow */
                Lwip2Enet_assert(false);
            }

            list = &pDmaPacket->sgList.list[scatterSegmentIndex];
            list->bufPtr = cPbuf->orgBufPtr;
            list->origBufPtr = cPbuf->orgBufPtr;
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
        gScatterSegCount = scatterSegmentIndex; /* Save for debugging */

        /* CRITICAL: Check scatter segment count before enqueueing */
        if (scatterSegmentIndex == 0 || scatterSegmentIndex > 16)
        {
            /* Invalid scatter count - memory corruption! */
            gLwipDebugTracker = 0x3FFF; /* ERROR: Invalid scatter count */
            Lwip2Enet_assert(false);
        }

        EnetQueue_enq(&rx->readyRxPktQ, &pDmaPacket->node);

        if (gLwipPktType == 1)
            gLwipDebugTracker = 0x3006; /* ARP: Returned to ready queue */
        else if (gLwipPktType == 2)
            gLwipDebugTracker = 0x3016; /* ICMP: Returned to ready queue */
        else
            gLwipDebugTracker = 0x3026; /* Other: Returned to ready queue */
    }

    /* Save alive count at exit for debugging */
    gPbufAliveAtExit = cPbuf->alivePbufCount;

    if (gLwipPktType == 1)
        gLwipDebugTracker = 0x3007; /* ARP: Exiting custom_pbuf_free */
    else if (gLwipPktType == 2)
        gLwipDebugTracker = 0x3017; /* ICMP: Exiting custom_pbuf_free */
    else
        gLwipDebugTracker = 0x3027; /* Other: Exiting custom_pbuf_free */

    /* CRITICAL CHECK: Ensure stack not corrupted before return */
    {
        volatile uint32_t stackCheck = 0x12345678;
        if (stackCheck != 0x12345678)
        {
            gLwipDebugTracker = 0x3FFE; /* ERROR: Stack corruption detected! */
            while(1); /* Halt here */
        }
    }
}
