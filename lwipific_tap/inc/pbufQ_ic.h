/*
 * lwIP does not natively support buffer queues. This file implements a custom
 * queue. Each member (called a buffer pointer) of the queue is a pointer
 * pointing to a next member as well as to a packet buffer. Every time a packet
 * buffer is passed to the enQ, a buffer pointer is allocated and associated to
 * the packet passed. The buffer pointer is then enqueued. When dequeuing, the
 * buffer pointer is freed. In this way the lwipstack is not modified while still
 * implementing a queue.
 * The enQ routine for a buffer queued for transmit is slightly different. This
 * is because the stack may free the buffer any time after the buffer is passed
 * the queue is passed to the abstraction layer. To prevent the transmission of
 * empty buffers, additional steps must be made before queuing the packet.
 */

#ifndef LWIPIF_IC_PBUFQ_H_
#define LWIPIF_IC_PBUFQ_H_

#include "lwip/pbuf.h"
#include "lwipopts.h"

#define pbufQ_ic_count(pQ) ((pQ)->count)

/* Buffer pointer structure definition */
struct LWIPIC_pbufNode{
    struct pbuf* hPbufPkt;
    struct LWIPIC_pbufNode* next;
};
typedef struct LWIPIC_pbufNode pbufIcNode;

/* Queue structure definition */
struct LWIPIC_pbufQ
{
    pbufIcNode *head, *tail;
    uint32_t count;
};
typedef struct LWIPIC_pbufQ pbufIcQ;

void pbufQ_ic_init_freeQ(pbufIcNode *pfree, uint32_t maxSize);
void pbufQ_ic_init(pbufIcQ *pQ);
void pbufQ_ic_enQ(pbufIcQ *pQ, struct pbuf *p);
void pbufQ_ic_enQHead(pbufIcQ *pQ, struct pbuf *p);
struct pbuf* pbufQ_ic_deQ(pbufIcQ *pQ);

#endif //LWIPIF_IC_PBUFQ_H_
