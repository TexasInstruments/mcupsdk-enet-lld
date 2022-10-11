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

/**
 *  \file enet_udma_memcfg.h
 *
 *  \brief Enet UDMA MemCfg header file.
 */

#ifndef ENET_UDMA_MEMCFG_H_
#define ENET_UDMA_MEMCFG_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <include/core/enet_dma.h>
#include <drivers/udma/soc/udma_soc.h>

#include <core/src/dma/udma/enet_udma_priv.h>
/* hack to access gUdmaTxMappedChRingAttributes */
#include "drivers/udma/udma_priv.h"
#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  \brief
 */
typedef struct EnetUdma_TxChObjMem_s
{
    /*! The node element so this packet can be added to a queue
     * Note- Keep EnetQ_Node as first member always as driver uses generic Q functions
     *       and deferences to this member */
    EnetQ_Node node;

    /*! Tx channel object mem element */
    EnetUdma_TxChObj txChObj;
} EnetUdma_TxChObjMem;

/**
 *  \brief
 */
typedef struct EnetUdma_RxFlowObjMem_s
{
    /*! The node element so this packet can be added to a queue
     * Note- Keep EnetQ_Node as first member always as driver uses generic Q functions
     *       and deferences to this member */
    EnetQ_Node node;

    /*! Rx flow object mem element */
    EnetUdma_RxFlowObj rxFlowObj;
} EnetUdma_RxFlowObjMem;

/**
 *  \brief
 */
typedef struct EnetUdma_RingMonObjMem_s
{
    /*! The node element so this packet can be added to a queue
     * Note- Keep EnetQ_Node as first member always as driver uses generic Q functions
     *       and deferences to this member */
    EnetQ_Node node;
#if (UDMA_SOC_CFG_RING_MON_PRESENT == 1)
    /*! Ring monitor object mem element */
    struct Udma_RingMonObj ringMonObj;
#endif

} EnetUdma_RingMonObjMem;

/**
 *  \brief
 */
typedef struct EnetUdma_TdCqRingObjMem_s
{
    /*! The node element so this packet can be added to a queue
     * Note- Keep EnetQ_Node as first member always as driver uses generic Q functions
     *       and deferences to this member */
    EnetQ_Node node;

    /*! Teardown Cq Ring object mem element. Allocating cahce aligned start address and size,
     *  to avoid cache operations on other memory regions while cache invalidating after
     *  popping from the ring. Only allocating single element i.e. ENET_UDMA_TDCQ_RING_ELE_CNT of 1,
     *  as it stores the return status */
    uint64_t tdCqRingMemObj [ENET_UTILS_ALIGN(ENET_UDMA_TDCQ_RING_ELE_CNT, UDMA_CACHELINE_ALIGNMENT)]
    __attribute__ ((aligned(UDMA_CACHELINE_ALIGNMENT)));
} EnetUdma_TdCqRingObjMem;


/**
 *  \brief
 */
typedef struct EnetUdma_RxFlowObjMemCfg_s
{
    uint32_t numFlows;
    EnetUdma_RxFlowObjMem     *rxFlowObjMemContainerBase;
    uint32_t                  rxFlowObjMemContainerSize;
} EnetUdma_RxFlowObjMemCfg;

/**
 *  \brief
 */
typedef struct EnetUdma_TxChObjMemCfg_s
{
    uint32_t numCh;
    EnetUdma_TxChObjMem     *txChObjContainerBase;
    uint32_t                 txChObjContainerSize;
} EnetUdma_TxChObjMemCfg;

/**
 *  \brief
 */
typedef struct EnetUdma_RingMonObjMemCfg_s
{
    uint32_t numRings;
    EnetUdma_RingMonObjMem    *ringMonObjMemContainerBase;
    uint32_t                  ringMonObjMemContainerSize;
} EnetUdma_RingMonObjMemCfg;

/**
 *  \brief
 */
typedef struct EnetUdma_TdCqRingObjMemCfg_s
{
    uint32_t numTxCh;
    EnetUdma_TdCqRingObjMem    *tdCqRingObjMemContainerBase;
    uint32_t                  tdCqRingObjMemContainerSize;
} EnetUdma_TdCqRingObjMemCfg;

/**
 *  \brief
 */
typedef struct EnetUdma_MemCfg_s
{

    /*! RX flow driver object  */
    EnetUdma_RxFlowObjMemCfg rxFlowObjMemCfg;

    /*! TX channel driver object  */
    EnetUdma_TxChObjMemCfg txChObjMemCfg;

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
    /*! TX channel driver object  */
    EnetUdma_RingMonObjMemCfg ringMonObjMemCfg;

    /*! Tear Down Queue Ring driver object  */
    EnetUdma_TdCqRingObjMemCfg tdCqRingObjMemCfg;
#endif

} EnetUdma_MemCfg;
/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */\

/*! Application implemented function to get the Udma memory configuration */
const EnetUdma_MemCfg * EnetUdmaMem_getCfg(void);

#ifdef __cplusplus
}
#endif

#endif /* #ifndef ENET_UDMA_MEMCFG_H_ */
