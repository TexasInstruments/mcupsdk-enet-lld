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

#include "lwip/pbuf.h"
#include "lwipopts.h"

/* 
 * This number is the total number of free buffer pointers available (for all
 * queues. Make sure this number is sufficiently high to accommodate all the 
 * buffers that could potentially be made by the stack in worst scenario.
 */
#define MAXFREE (PBUF_POOL_SIZE * 2)

#define pbufQ_count(pQ) ((pQ)->count)

/* Buffer pointer structure definition */
struct LWIP2ENET_pbufNode{
    struct pbuf* hPbufPkt;
    struct LWIP2ENET_pbufNode* next;
};
typedef struct LWIP2ENET_pbufNode pbufNode;

/* Queue structure definition */
struct LWIP2ENET_pbufQ
{
    pbufNode *head, *tail;
    uint32_t count;
};
typedef struct LWIP2ENET_pbufQ pbufQ;

void pbufQ_init_freeQ();
void pbufQ_init(pbufQ *pQ);
void pbufQ_enQ(pbufQ *pQ, struct pbuf *p);
void pbufQ_enQHead(pbufQ *pQ, struct pbuf *p);
struct pbuf* pbufQ_deQ(pbufQ *pQ);
