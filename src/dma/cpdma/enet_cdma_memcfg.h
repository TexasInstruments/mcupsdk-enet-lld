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
 *  \file enet_cpdma_cfg.h
 *
 *  \brief Enet CPDMA MemCfg header file.
 */

#ifndef ENET_CPDMA_CFG_H_
#define ENET_CPDMA_CFG_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <include/core/enet_dma.h>
#include <core/src/dma/cpdma/enet_cpdma_priv.h>

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
typedef struct EnetCpdma_TxChObjMem_s
{
    /*! The node element so this packet can be added to a queue
     * Note- Keep EnetQ_Node as first member always as driver uses generic Q functions
     *       and deferences to this member */
    EnetQ_Node node;

    /*! Tx channel object mem element */
    EnetCpdma_TxChObj txChObj;
} EnetCpdma_TxChObjMem;

/**
 *  \brief
 */
typedef struct EnetCpdma_RxChObjMem_s
{
    /*! The node element so this packet can be added to a queue
     * Note- Keep EnetQ_Node as first member always as driver uses generic Q functions
     *       and deferences to this member */
    EnetQ_Node node;

    /*! Rx flow object mem element */
    EnetCpdma_RxChObj rxChObj;
} EnetCpdma_RxChObjMem;


/**
 *  \brief
 */
typedef struct EnetCpdma_RxChObjMemCfg_s
{
    uint32_t numRxCh;
    EnetCpdma_RxChObjMem     *rxChObjMemContainerBase;
    uint32_t                  rxChObjMemContainerSize;
} EnetCpdma_RxChObjMemCfg;

/**
 *  \brief
 */
typedef struct EnetCpdma_TxChObjMemCfg_s
{
    uint32_t numTxCh;
    EnetCpdma_TxChObjMem     *txChObjMemContainerBase;
    uint32_t                 txChObjMemContainerSize;
} EnetCpdma_TxChObjMemCfg;


/**
 *  \brief
 */
typedef struct EnetCpdma_MemCfg_s
{

    /*! RX channel driver object  */
    EnetCpdma_RxChObjMemCfg rxChObjMemCfg;

    /*! TX channel driver object  */
    EnetCpdma_TxChObjMemCfg txChObjMemCfg;

} EnetCpdma_MemCfg;
/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */\

/*! Application implemented function to get the Cpdma rx and tx obj memory configuration */
const EnetCpdma_MemCfg * EnetCpdmaMem_getCfg(void);


#ifdef __cplusplus
}
#endif

#endif /* #ifndef ENET_CPDMA_CFG_H_ */
