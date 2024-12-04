/*
 *  Copyright (c) Texas Instruments Incorporated 2020-23
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
 * \file  enet_cpdma.c
 *
 * \brief This file contains the implementation of the Enet data path with CPDMA.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <string.h>
#include <csl_cpswitch.h>

#include <enet_cfg.h>
#include <include/core/enet_base.h>
#include <include/core/enet_utils.h>
#include <include/core/enet_soc.h>
#include <include/core/enet_per.h>
#include <include/core/enet_queue.h>
#include <priv/core/enet_base_priv.h>
#include <priv/core/enet_trace_priv.h>
#include <include/common/enet_osal_dflt.h>
#include <include/common/enet_utils_dflt.h>
#include <utils/include/enet_cpdmautils.h>

#include <include/core/enet_dma.h>
#include <include/core/enet_dma_pktutils.h>
#include <src/dma/cpdma/enet_cpdma_priv.h>
#include <priv/mod/cpsw_clks.h>
#if defined (SOC_AM273X) || defined(SOC_AWR294X) || defined (SOC_AWR2544) || defined(SOC_AM263X) || defined(SOC_AM263PX) || defined(SOC_AM261X)
#include <priv/per/cpsw_cpdma_priv.h>
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define ENET_CPDMA_VIRT2SOCADDR(ptr)   ((typeof(ptr))((uintptr_t)EnetUtils_virtToPhys((const void *)(ptr), NULL)))
#define ENET_CPDMA_SOC2VIRTADDR(ptr)   ((typeof(ptr))((uintptr_t)EnetUtils_physToVirt((uint64_t)(ptr), NULL)))

#define ENET_CPDMA_PRESCALER_PERIOD_US        (4U)
#define ENET_CPDMA_SEC2PRESCALER_PERIOD_DIV_FACTOR ((1U * 1000U * 1000U) / ENET_CPDMA_PRESCALER_PERIOD_US)
#define ENET_CPDMA_CONV_SEC2PRESCALER(x)      ((x)/ENET_CPDMA_SEC2PRESCALER_PERIOD_DIV_FACTOR)

/* Tx packet requires Two Tx scatter gather segments + 1 csum offload descriptor */
#define ENET_CPDMA_NUM_DESC_PER_TXPKT         (3U)
/* Rx packet requires only one desc per packet till scatter gather is supported.
 * Csum info is at end of packet and no cpdma desc is used */
#define ENET_CPDMA_NUM_DESC_PER_RXPKT         (1U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static int32_t EnetCpdma_checkTxChParams(EnetCpdma_OpenTxChPrms *pTxChPrms);

static int32_t EnetCpdma_checkRxChParams(EnetCpdma_OpenRxChPrms *pRxChPrms);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/*!
 *  \brief EnetCpdma_mapApp2CpdmaCoreId returns the coreId expected by CPDMA module.
 *
 *  CPDMA supports simultaneous multiple core data path, but enabling this multi core support
 *  in CPDMA is an SoC integration parameter and is not enabled for all SoCs
 *  For SoCs where CPDMA supports multiple core (viz AM263x) the coreId used should
 *  be the coreIds as defined in source\drivers\hw_include\am263x\cslr_soc_defines.h
 *  These are 0-indexed coreId with specific number assigned to each core
 *  For SoCs where CPDMA does not support multiple cores (viz AM273x/AWR294x) the
 *  coreId should be set to 0 for all CPDMA configuration even if the application is on a
 *  different core other than RF5SS0_CORE0.
 *
 *  \param appCoreId	application CoreId
 */
static uint32_t EnetCpdma_mapApp2CpdmaCoreId(uint32_t appCoreId)
{
    uint32_t coreId;
#if ENET_CFG_IS_ON(CPDMA_MULTICORE_SUPPORT)
    coreId = EnetSoC_mapApp2CpdmaCoreId(appCoreId);
#endif
#if ENET_CFG_IS_OFF(CPDMA_MULTICORE_SUPPORT)
    coreId = ENET_CPDMA_DEFAULT_SINGLE_CORE_ID;
#endif
    return coreId;

}

/** ============================================================================
 *  @n@b EnetCpdma_enableChannel()
 *
 *  @b Description
 *  @n This function configures the appropriate registers to initialize a
 *  DMA channel.
 *
 *  @b Arguments
 *  @verbatim
 *      channel             Channel number
 *      direction           Channel Direction, i.e., CPSW_DMA_DIR_TX/CPSW_DMA_DIR_RX
    @endverbatim
 *
 *  <b> Return Value </b>
 *  @n  None
 *
 *  <b> Pre Condition </b>
 *  @n  None
 *
 *  <b> Post Condition </b>
 *  @n  Initializes the Tx/Rx HDP and enables interrupts on the specific channel.
 *
 *  @b Example
    @endverbatim
 * ============================================================================
 */
static void EnetCpdma_enableChannel(EnetDma_Handle hEnetDma, uint32_t channel, uint32_t direction, uint32_t cpdmaCoreId); /* for misra warning */
static void EnetCpdma_enableChannel(EnetDma_Handle hEnetDma, uint32_t channel, uint32_t direction, uint32_t cpdmaCoreId)
{

    if (direction == ENET_CPDMA_DIR_TX)
    {
        /* enable a TX Dma channel for transfers here */

        CSL_CPSW_setCpdmaTxHdrDescPtr(hEnetDma->cpdmaRegs, 0, channel);
        CSL_CPSW_enableCpdmaTxInt(hEnetDma->cpdmaRegs, channel);
        CSL_CPSW_enableWrTxInt(hEnetDma->cpswSsRegs, cpdmaCoreId, channel);

        /* mark the channel as open */
        hEnetDma->chIsOpen[ENET_CPDMA_DIR_TX][channel] = true;
    }
    else
    {
        /* enable RX Dma channel for transfer here */
        CSL_CPSW_enableCpdmaRxInt(hEnetDma->cpdmaRegs, channel);
        CSL_CPSW_enableWrRxInt(hEnetDma->cpswSsRegs, cpdmaCoreId, channel);
        /* mark the channel as open */
        hEnetDma->chIsOpen[ENET_CPDMA_DIR_RX][channel] = true;
    }
}

/** ============================================================================
 *  @n@b EnetCpdma_disableChannel()
 *
 *  @b Description
 *  @n This function configures the appropriate registers to de-initialize a
 *  DMA channel.
 *
 *  @b Arguments
 *  @verbatim
 *      channel             Channel number
 *      direction           Channel Direction, i.e., ENET_CPDMA_DIR_TX/ENET_CPDMA_DIR_RX
    @endverbatim
 *
 *  <b> Return Value </b>
 *  @n  Always returns ENET_SOK
 *
 *  <b> Pre Condition </b>
 *  @n  None
 *
 *  <b> Post Condition </b>
 *  @n  De-Initializes the channel by resetting Tx/Rx HDP and disabling interrupts on
 *  the specific channel.
 *
 *  @b Example
    @endverbatim
 * ============================================================================
 */
static void EnetCpdma_disableChannel(EnetDma_Handle hEnetDma, uint32_t channel, uint32_t direction, uint32_t cpdmaCoreId); /* for misra warning */
static void EnetCpdma_disableChannel(EnetDma_Handle hEnetDma, uint32_t channel, uint32_t direction, uint32_t cpdmaCoreId)
{
    uint32_t desc;
    hEnetDma->tdPending[direction][channel] = true;

    if (direction == ENET_CPDMA_DIR_TX)
    {
        /* disable the requested Tx DMA channel here */

        /* command teardown  */
        CSL_CPSW_cpdmaTxTeardown(hEnetDma->cpdmaRegs, channel);

        /* wait for ack ,examine CP for EMAC_TEARDOWN_DESC (0xfffffffc) */
        do
        {
            CSL_CPSW_getCpdmaTxCp(hEnetDma->cpdmaRegs, channel, &desc);
        } while(desc != ENET_CPDMA_TEARDOWN_DESC);


        /* disable Tx Interrupt on this channel */
        CSL_CPSW_disableCpdmaTxInt(hEnetDma->cpdmaRegs, channel);

        /* also disable in the wrapper */
        CSL_CPSW_disableWrTxInt(hEnetDma->cpswSsRegs, cpdmaCoreId, channel);
    }
    else
    {
        /* disable the requested Rx Dma channel here */

        /* command teardown  */
        CSL_CPSW_cpdmaRxTeardown(hEnetDma->cpdmaRegs, channel);

        /* wait for ack ,examine CP for 0xfffffffc */
        do
        {
            CSL_CPSW_getCpdmaRxCp(hEnetDma->cpdmaRegs, channel, &desc);
        } while(desc != ENET_CPDMA_TEARDOWN_DESC);

        /*disable Rx Interrupt on this channel */
        CSL_CPSW_disableCpdmaRxInt(hEnetDma->cpdmaRegs, channel);

        /* also disable in the wrapper */
        CSL_CPSW_disableWrRxInt(hEnetDma->cpswSsRegs, cpdmaCoreId, channel);
    }

    hEnetDma->tdPending[direction][channel] = false;
    hEnetDma->chIsOpen[direction][channel] = false;
}

/** ============================================================================
 *  @n@b EnetCpdma_initTxChannel()
 *
 *  @b Description
 *  @n This function sets up the Transmit Buffer descriptors.
 *
 *  @b Arguments
 *  @verbatim
 *      chInfo              Channel object
    @endverbatim
 *
 *  <b> Return Value </b>
 *  ENET_EBADARGS  -   Returned on BD allocation error
 *  ENET_SOK       -   On Success
 *
 *  <b> Pre Condition </b>
 *  @n  None
 *
 *  <b> Post Condition </b>
 *  @n  Sets up the Tx Buffer descriptors. Tx channel ready for send.
 *
 *  @b Example
    @endverbatim
 * ============================================================================
 */
static int32_t EnetCpdma_initTxChannel(EnetDma_Handle hEnetDma, EnetCpdma_ChInfo *chInfo); /* for misra warning */
static int32_t EnetCpdma_initTxChannel(EnetDma_Handle hEnetDma, EnetCpdma_ChInfo *chInfo)
{
    int32_t retVal = ENET_SOK ;
    EnetCpdma_DescCh* txChan = &hEnetDma->txCppi[chInfo->chNum];
    EnetCpdma_cppiDesc    *pDesc;

    /* zero init the book keeping structure */
    memset(txChan, 0, sizeof(EnetCpdma_DescCh));

    /* store pointer to channel info structure */
    txChan->chInfo = chInfo;

    /*
     * Setup Transmit Buffer Descriptors
     */
    /* Pointer to first descriptor to use on this channl */
    pDesc = (EnetCpdma_cppiDesc *)hEnetDma->cppiRamBase;
    pDesc += hEnetDma->numBdsAllocated; /* advance to next free BD */

    if ((hEnetDma->numBdsAllocated + chInfo->numBD) > hEnetDma->maxBds)
    {
        /* not enough room for the requested number of BDs, fail request */
        ENETTRACE_ERR("InitTx Channel : Unable to allocate %d BDs for channel %d. %d BDs already in use\n",
                      chInfo->numBD,chInfo->chNum,hEnetDma->numBdsAllocated);
        retVal = ENET_EBADARGS;
    }
    else
    {
        /* Setup Transmit Buffers */
        txChan->hEnetDma   = hEnetDma;
        txChan->descMax    = chInfo->numBD;
        /*Pointer for first TX desc = pointer to RX + num of RX desc.*/
        txChan->pDescFirst = pDesc;
        txChan->pDescLast  = pDesc + (chInfo->numBD - 1);
        txChan->pDescRead  = pDesc;
        txChan->pDescWrite = pDesc;
        txChan->descFreeCount = chInfo->numBD;
        txChan->descSubmittedCount = 0;
        txChan->pDescTail     = NULL;
        txChan->descHistory.descHistoryCount = 0U;
        txChan->unackedCpDescCount = 0U;
        /* clear the teardown pending flag */
        hEnetDma->tdPending[ENET_CPDMA_DIR_TX][chInfo->chNum] = false;
        /* update the Bd allocation count */
        hEnetDma->numBdsAllocated += chInfo->numBD;
        hEnetDma->chIsInit[ENET_CPDMA_DIR_TX][chInfo->chNum] = true;
    }
    return (retVal);
}

static int32_t EnetCpdma_restoreTxChannel(EnetDma_Handle hEnetDma, EnetCpdma_ChInfo *chInfo);
static int32_t EnetCpdma_restoreTxChannel(EnetDma_Handle hEnetDma, EnetCpdma_ChInfo *chInfo)
{
    int32_t retVal = ENET_SOK ;
    EnetCpdma_cppiDesc    *pDesc;

    Enet_assert(hEnetDma != NULL);
    EnetCpdma_DescCh* txChan = &hEnetDma->txCppi[chInfo->chNum];

    /* store pointer to channel info structure */
    txChan->chInfo = chInfo;

    /*
     * Setup Transmit Buffer Descriptors
     */
    /* Pointer to first descriptor to use on this channl */
    pDesc = (EnetCpdma_cppiDesc *)hEnetDma->cppiRamBase;
    pDesc += hEnetDma->numBdsAllocated; /* advance to next free BD */

    if ((hEnetDma->numBdsAllocated + chInfo->numBD) > hEnetDma->maxBds)
    {
        /* not enough room for the requested number of BDs, fail request */
        ENETTRACE_ERR("InitTx Channel : Unable to allocate %d BDs for channel %d. %d BDs already in use\n",
                      chInfo->numBD,chInfo->chNum,hEnetDma->numBdsAllocated);
        retVal = ENET_EBADARGS;
    }
    else
    {
        /* Setup Transmit Buffers */
        txChan->hEnetDma   = hEnetDma;
        txChan->descMax    = chInfo->numBD;
        /*Pointer for first TX desc = pointer to RX + num of RX desc.*/
        txChan->pDescFirst = pDesc;
        txChan->pDescLast  = pDesc + (chInfo->numBD - 1);
        txChan->pDescRead  = pDesc;
        txChan->pDescWrite = pDesc;
        txChan->descFreeCount = chInfo->numBD;
        txChan->pDescTail     = NULL;
        txChan->descHistory.descHistoryCount = 0U;
        txChan->unackedCpDescCount = 0U;
        /* clear the teardown pending flag */
        hEnetDma->tdPending[ENET_CPDMA_DIR_TX][chInfo->chNum] = false;
        /* update the Bd allocation count */
        hEnetDma->numBdsAllocated += chInfo->numBD;
        hEnetDma->chIsInit[ENET_CPDMA_DIR_TX][chInfo->chNum] = true;
    }
    return (retVal);
}

/** ============================================================================
 *  @n@b EnetCpdma_initRxChannel()
 *
 *  @b Description
 *  @n This function sets up the Receive Buffer descriptors.
 *
 *  @b Arguments
 *  @verbatim
 *      chInfo              Channel object
    @endverbatim
 *
 *  <b> Return Value </b>
 *  ENET_EBADARGS  -   Returned on BD allocation error
 *  ENET_SOK      -   On Success
 *
 *  <b> Pre Condition </b>
 *  @n  None
 *
 *  <b> Post Condition </b>
 *  @n  Sets up the Rx Buffer descriptors. Rx channel ready for receive.
 *
 *  @b Example
    @endverbatim
 * ============================================================================
 */
static int32_t EnetCpdma_initRxChannel(EnetDma_Handle hEnetDma, EnetCpdma_ChInfo *chInfo); /* for misra warning */
static int32_t EnetCpdma_initRxChannel(EnetDma_Handle hEnetDma, EnetCpdma_ChInfo *chInfo)
{
    int32_t retVal = ENET_SOK;
    EnetCpdma_DescCh* rxChan = &hEnetDma->rxCppi[chInfo->chNum];
    EnetCpdma_cppiDesc *pDesc;

    /* zero init the book keeping structure */
    memset(rxChan, 0, sizeof(EnetCpdma_DescCh));

    /* store pointer to channel info structure */
    rxChan->chInfo = chInfo;

    /*
     * Setup Receive Buffers
     */
    /* Pointer to first descriptor to use on RX */
    pDesc = (EnetCpdma_cppiDesc *)hEnetDma->cppiRamBase;

    pDesc += hEnetDma->numBdsAllocated; /* advance to next free BD */

    if ((hEnetDma->numBdsAllocated + chInfo->numBD) >  hEnetDma->maxBds)
    {
        /* not enough room for the requested number of BDs, fail request */
        ENETTRACE_ERR("InitRx Channel : Unable to allocate %d BDs for channel %d.%d BDs already in use\n",
                      chInfo->numBD,chInfo->chNum,hEnetDma->numBdsAllocated);
        retVal =  ENET_EBADARGS;
    }
    else
    {
        /* Init the Rx channel */
        rxChan->hEnetDma   = hEnetDma;
        rxChan->descMax    = chInfo->numBD;
        rxChan->pDescFirst = pDesc;
        rxChan->pDescLast  = pDesc + (chInfo->numBD - 1);
        rxChan->pDescRead  = pDesc;
        rxChan->pDescWrite = pDesc;
        rxChan->pDescTail     = NULL;
        rxChan->descHistory.descHistoryCount = 0U;
        rxChan->unackedCpDescCount = 0U;
        rxChan->descFreeCount = chInfo->numBD;
        rxChan->descSubmittedCount = 0;

        /* clear the teardown pending flag */
        hEnetDma->tdPending[ENET_CPDMA_DIR_RX][chInfo->chNum] = false;
        hEnetDma->numBdsAllocated += chInfo->numBD;
        hEnetDma->chIsInit[ENET_CPDMA_DIR_RX][chInfo->chNum] = true;
    }
    return (retVal);
}

static int32_t EnetCpdma_restoreRxChannel(EnetDma_Handle hEnetDma, EnetCpdma_ChInfo *chInfo);
static int32_t EnetCpdma_restoreRxChannel(EnetDma_Handle hEnetDma, EnetCpdma_ChInfo *chInfo)
{
    int32_t retVal = ENET_SOK;
    EnetCpdma_cppiDesc *pDesc;

    Enet_assert(hEnetDma != NULL);
    EnetCpdma_DescCh* rxChan = &hEnetDma->rxCppi[chInfo->chNum];

    /* store pointer to channel info structure */
    rxChan->chInfo = chInfo;

    /*
     * Setup Receive Buffers
     */
    /* Pointer to first descriptor to use on RX */
    pDesc = (EnetCpdma_cppiDesc *)hEnetDma->cppiRamBase;

    //pDesc += hEnetDma->numBdsAllocated; /* advance to next free BD */

    if ((hEnetDma->numBdsAllocated + chInfo->numBD) >  hEnetDma->maxBds)
    {
        /* not enough room for the requested number of BDs, fail request */
        ENETTRACE_ERR("InitRx Channel : Unable to allocate %d BDs for channel %d.%d BDs already in use\n",
                      chInfo->numBD,chInfo->chNum,hEnetDma->numBdsAllocated);
        retVal =  ENET_EBADARGS;
    }
    else
    {
        /* Init the Rx channel */
        rxChan->hEnetDma   = hEnetDma;
        rxChan->descMax    = chInfo->numBD;
        rxChan->pDescFirst = pDesc;
        rxChan->pDescLast  = pDesc + (chInfo->numBD - 1);
        rxChan->pDescRead  = pDesc;
        rxChan->pDescWrite = pDesc;
        rxChan->pDescTail     = NULL;
        rxChan->descHistory.descHistoryCount = 0U;
        rxChan->unackedCpDescCount = 0U;
        rxChan->descFreeCount = chInfo->numBD;

        /* clear the teardown pending flag */
        hEnetDma->tdPending[ENET_CPDMA_DIR_RX][chInfo->chNum] = false;
        hEnetDma->numBdsAllocated += chInfo->numBD;
        hEnetDma->chIsInit[ENET_CPDMA_DIR_RX][chInfo->chNum] = true;
    }

    return (retVal);
}

/** ============================================================================
 *  @n@b EnetCpdma_unInitTxChannel()
 *
 *  @b Description
 *  @n This function frees up the enqueued Transmit Buffer descriptors and the
 *  packets held in any of its queues.
 *
 *  @b Arguments
 *  @verbatim
 *      chInfo              Channel object
    @endverbatim
 *
 *  <b> Return Value </b>
 *  None
 *
 *  <b> Pre Condition </b>
 *  @n  None
 *
 *  <b> Post Condition </b> None
 *
 *  @b Example
    @endverbatim
 * ============================================================================
 */
static void EnetCpdma_unInitTxChannel(EnetDma_Handle hEnetDma, const EnetCpdma_ChInfo *chInfo); /* for misra warning */
static void EnetCpdma_unInitTxChannel(EnetDma_Handle hEnetDma, const EnetCpdma_ChInfo *chInfo)
{
    EnetCpdma_DescCh* txChan = &hEnetDma->txCppi[chInfo->chNum];

    EnetQueue_append(&txChan->waitQueue, &txChan->descQueue);
    EnetQueue_initQ(&txChan->descQueue);
    hEnetDma->numBdsAllocated -= txChan->chInfo->numBD;
    hEnetDma->chIsInit[ENET_CPDMA_DIR_TX][chInfo->chNum] = false;
}

/** ============================================================================
 *  @n@b EnetCpdma_unInitRxChannel()
 *
 *  @b Description
 *  @n This function frees up the enqueued Receive Buffer descriptors and any
 *  packets held in the Rx queue.
 *
 *  @b Arguments
 *  @verbatim
 *      chInfo              Channel object
    @endverbatim
 *
 *  <b> Return Value </b>
 *  None
 *
 *  <b> Pre Condition </b>
 *  @n  None
 *
 *  <b> Post Condition </b> None
 *
 *  @b Example
    @endverbatim
 * ============================================================================
 */
static void EnetCpdma_unInitRxChannel(EnetDma_Handle hEnetDma, const EnetCpdma_ChInfo *chInfo); /* for misra warning */
static void EnetCpdma_unInitRxChannel(EnetDma_Handle hEnetDma, const EnetCpdma_ChInfo *chInfo)
{
    EnetCpdma_DescCh* rxChan = &hEnetDma->rxCppi[chInfo->chNum];

    EnetQueue_append(&rxChan->freeQueue, &rxChan->descQueue);
    EnetQueue_initQ(&rxChan->descQueue);
    hEnetDma->numBdsAllocated -= rxChan->chInfo->numBD;
    hEnetDma->chIsInit[ENET_CPDMA_DIR_RX][chInfo->chNum] = false;
}

/** ============================================================================
 *  @n@b EnetCpdma_netChOpen()
 *
 *  @b Description
 *  @n This function opens a data channel on the CPPI DMA engine.
 *
 *  @b Arguments
 *  @verbatim
 *      chInfo              Channel object to setup.
    @endverbatim
 *
 *  <b> Return Value </b>
 *  ENET_SOK          - Channel setup successful
 *  CPSW3G_ERR_CH_ALREADY_INIT  - Channel already initialized
 *  Other error values if Channel init failed.
 *
 *  <b> Pre Condition </b>
 *  @n  None
 *
 *  <b> Post Condition </b> None
 *
 *  @b Example
    @endverbatim
 * ============================================================================
 */
static int32_t  EnetCpdma_netChOpen(EnetDma_Handle hEnetDma, EnetCpdma_ChInfo *chInfo, uint32_t cpdmaCoreId); /* for misra warning */
static int32_t  EnetCpdma_netChOpen(EnetDma_Handle hEnetDma, EnetCpdma_ChInfo *chInfo, uint32_t cpdmaCoreId)
{
    int32_t retVal = ENET_SOK;
    uint32_t numChans = (chInfo->chDir == ENET_CPDMA_DIR_TX)?hEnetDma->numTxChans:
                                                             hEnetDma->numRxChans;
    /* Perform sanity checks on input params */
    if( chInfo->chNum >= numChans)
    {
        ENETTRACE_ERR("NetChOpen: Channel number (%d) invalid \n", chInfo->chNum);
        retVal = ENET_EINVALIDPARAMS;
    }
    else
    {
        if(hEnetDma->chIsInit[chInfo->chDir][chInfo->chNum] == true)
        {
            ENETTRACE_ERR("NetChOpen: %s Channel %d already initialized\n",
                          ((chInfo->chDir == ENET_CPDMA_DIR_TX) ? "TX" : "RX"), chInfo->chNum);
            retVal = ENET_EALREADYOPEN;
        }
        else
        {
            /* Perform book keeping for indv channel */
            if (chInfo->chDir == ENET_CPDMA_DIR_TX)
            {
                retVal = EnetCpdma_initTxChannel(hEnetDma, chInfo);
            }
            else
            {
                retVal = EnetCpdma_initRxChannel(hEnetDma, chInfo);
            }

            if (retVal != ENET_SOK)
            {
                ENETTRACE_ERR("NetChOpen: Error in initializing %s channel %d",
                              ((chInfo->chDir == ENET_CPDMA_DIR_TX) ? "TX" : "RX"), chInfo->chNum);
            }
            else
            {
                /* Enable this channel for use */
                EnetCpdma_enableChannel(hEnetDma, chInfo->chNum, chInfo->chDir, cpdmaCoreId);
            }
        }
    }
    return (retVal);
}

/** ============================================================================
 *  @n@b EnetCpdma_netChClose()
 *
 *  @b Description
 *  @n This function closes a previously open data channel on the CPPI DMA engine
 *  and frees up any memory held associated with it.
 *
 *  @b Arguments
 *  @verbatim
 *      chInfo              Channel object to clean up.
    @endverbatim
 *
 *  <b> Return Value </b>
 *          None
 *
 *  <b> Pre Condition </b>
 *  @n  None
 *
 *  <b> Post Condition </b> None
 *
 *  @b Example
    @endverbatim
 * ============================================================================
 */
static int32_t EnetCpdma_netChClose(EnetDma_Handle hEnetDma, const EnetCpdma_ChInfo *chInfo, uint32_t cpdmaCoreId); /* for misra warning */
static int32_t EnetCpdma_netChClose(EnetDma_Handle hEnetDma, const EnetCpdma_ChInfo *chInfo, uint32_t cpdmaCoreId)
{

    int32_t retVal = ENET_SOK;

    if(hEnetDma->chIsOpen[chInfo->chDir][chInfo->chNum]  == true)
    {
        EnetCpdma_disableChannel(hEnetDma, chInfo->chNum, chInfo->chDir, cpdmaCoreId);
    }

    /* TODO: should we perform dequeue here? */

    if(hEnetDma->chIsInit[chInfo->chDir][chInfo->chNum] == true)
    {
        if(chInfo->chDir == ENET_CPDMA_DIR_TX)
        {
            EnetCpdma_unInitTxChannel(hEnetDma, chInfo);
        }
        else
        {
            EnetCpdma_unInitRxChannel(hEnetDma, chInfo);
        }
    }
    return (retVal);
}

static int32_t EnetCpdma_netChReset(EnetDma_Handle hEnetDma, const EnetCpdma_ChInfo *chInfo, uint32_t cpdmaCoreId)
{

    int32_t retVal = ENET_SOK;

    if(hEnetDma->chIsOpen[chInfo->chDir][chInfo->chNum]  == true)
    {
        EnetCpdma_disableChannel(hEnetDma, chInfo->chNum, chInfo->chDir, cpdmaCoreId);
    }

    return (retVal);
}

static void EnetCpdma_setSOPInfo(EnetCpdma_cppiDesc *pDescThis, EnetCpdma_PktInfo *pPkt, uint32_t totalPktLen);
static void EnetCpdma_setSOPInfo(EnetCpdma_cppiDesc *pDescThis, EnetCpdma_PktInfo *pPkt, uint32_t totalPktLen)
{
    if(pPkt->chkSumInfo != 0)
    {
#if ENET_CFG_IS_ON(DEV_ERROR)
       if( (ENET_CPDMA_GET_CSUM_RESULT_BYTE(pPkt->chkSumInfo) > totalPktLen) ||
           (ENET_CPDMA_GET_CSUM_START_BYTE(pPkt->chkSumInfo) + ENET_CPDMA_GET_CSUM_BYTE_COUNT(pPkt->chkSumInfo) - 1 > totalPktLen))
       {
           Enet_assert(false);
       }
#endif
        pDescThis->chkSumInfo = pPkt->chkSumInfo;
        pDescThis->pBuffer   = ENET_CPDMA_VIRT2SOCADDR((uint8_t *)(&(pDescThis->chkSumInfo)));
        pDescThis->bufOffLen = ENET_CPDMA_ENCAPINFO_CHECKSUM_INFO_LEN;
        pDescThis->pktFlgLen = (totalPktLen + ENET_CPDMA_ENCAPINFO_CHECKSUM_INFO_LEN)
                             | ENET_CPDMA_DESC_PKT_FLAG_SOP
                             | ENET_CPDMA_DESC_PKT_FLAG_OWNER
                             | ENET_CPDMA_DESC_PKT_FLAG_CHKSUM_ENCAP;
    }
    else
    {
        pDescThis->pBuffer = ENET_CPDMA_VIRT2SOCADDR(pPkt->sgList.list[0].bufPtr);
        pDescThis->bufOffLen = pPkt->sgList.list[0].segmentFilledLen;
        pDescThis->pktFlgLen  = totalPktLen
                              | ENET_CPDMA_DESC_PKT_FLAG_SOP
                              | ENET_CPDMA_DESC_PKT_FLAG_OWNER;
    }
    /* For directed packet case, port is specified, need to update TO_PORT_ENABLE and TO_PORT field in the descriptor */
    if (pPkt->txPortNum != ENET_MAC_PORT_INV)
    {
        uint8_t portNum = ENET_MACPORT_NORM(pPkt->txPortNum) + 1U;
        pDescThis->pktFlgLen |= (ENET_CPDMA_DESC_PSINFO_TO_PORT_ENABLE_MASK);
        pDescThis->pktFlgLen |= (portNum << ENET_CPDMA_DESC_PSINFO_TO_PORT_SHIFT);
    }
    return;
}
/** ============================================================================
 *  @n@b EnetCpdma_enqueueTx()
 *
 *  @b Description
 *  @n Enqueue a TX packet and restart transmitter as needed
 *
 *  @b Arguments
 *  @verbatim
        pq  pointer to Channel descriptor
    @endverbatim
 *
 *  <b> Return Value </b>  None
 *
 *  <b> Pre Condition </b>
 *  @n  None
 *
 *  <b> Post Condition </b>
 *  @n  Enqueue a TX packet and restart transmitter as needed
 *
 *  @b Example
 *  @verbatim
        EnetCpdma_DescCh *pDescCh;

        EnetCpdma_enqueueTx ( pDescCh );
    @endverbatim
 * ============================================================================
 */

static void EnetCpdma_enqueueTx( EnetCpdma_DescCh *pDescCh)
{
    EnetCpdma_cppiDesc    *pDescThis = NULL;
    EnetCpdma_PktInfo     *pPkt = NULL;
    volatile EnetCpdma_cppiDesc *pStartDesc = NULL;
    volatile EnetCpdma_cppiDesc *pEndDesc = NULL;
    uint32_t count;
    uint32_t totalPktLen = 0;
    uint32_t i = 0U;
    uint32_t loopCount = 0U;
    EnetCpdma_SGListEntry *list;
    uint32_t scatterSegmentIndex = 0U;
    bool isCsumIteration = false;

    pStartDesc = pDescCh->pDescWrite;
    /* Try to post any waiting packets if there is enough room */
    while (((count = EnetQueue_getQCount(&pDescCh->waitQueue)) > 0U) &&
            (pDescCh->descFreeCount > 0))
    {
        pPkt = (EnetCpdma_PktInfo *)EnetQueue_deq(&pDescCh->waitQueue);
        Enet_assert(pPkt != NULL);
        Enet_assert(pPkt->sgList.numScatterSegments > 0);
        Enet_assert(pPkt->sgList.numScatterSegments <= ENET_CPDMA_CPSW_MAX_SG_LIST);

        totalPktLen = 0;
        for (i = 0U; i < pPkt->sgList.numScatterSegments; i++)
        {
            totalPktLen += pPkt->sgList.list[i].segmentFilledLen;
        }
        loopCount = 0U;
        scatterSegmentIndex = 0U;
        while (scatterSegmentIndex < pPkt->sgList.numScatterSegments)
        {
            Enet_assert(pDescCh->descFreeCount > 0);
            pDescThis = pDescCh->pDescWrite;
            /* Move the write pointer and bump count */
            if (pDescCh->pDescWrite == pDescCh->pDescLast)
            {
                pDescCh->pDescWrite = pDescCh->pDescFirst;
            }
            else
            {
                pDescCh->pDescWrite++;
            }
            pDescCh->descFreeCount--;
            pDescThis->pktFlgLen = 0;
            isCsumIteration = false;
            if (loopCount == 0)
            {
                /* Fill the SOP pkt in the first loop iteration. This could be first sg segment or chksuminfo */
                EnetCpdma_setSOPInfo(pDescThis, pPkt, totalPktLen);
                if (pPkt->chkSumInfo != 0)
                {
                    /* Mark this loop iteration for chksuminfo. sgList.list[0] is handled in the next loop */
                    isCsumIteration = true;
                }
                else
                {
                    /* No chksuminfo, SOP is sgList.list[0], cacheWbInv the data */
                    list = &pPkt->sgList.list[0];
                    if (list->disableCacheOps != true)
                    {
                        EnetOsal_cacheWbInv((void*) list->bufPtr, list->segmentFilledLen);
                    }
                }
            }
            else
            {
                /* Handle the Non-SOP descriptors */
                list = &pPkt->sgList.list[scatterSegmentIndex];
                pDescThis->pBuffer = ENET_CPDMA_VIRT2SOCADDR(list->bufPtr);
                pDescThis->bufOffLen = list->segmentFilledLen;
                pDescThis->pktFlgLen = 0;
                if (list->disableCacheOps != true)
                {
                    EnetOsal_cacheWbInv((void*) list->bufPtr, list->segmentFilledLen);
                }
            }
            if (scatterSegmentIndex == (pPkt->sgList.numScatterSegments - 1) && (!isCsumIteration))
            {
                pDescThis->pktFlgLen |= ENET_CPDMA_DESC_PKT_FLAG_EOP;
                if ((count == 1U) || (pDescCh->descFreeCount == 0U))
                {
                    pDescThis->pNext = NULL;
                }
                else
                {
                    pDescThis->pNext = ENET_CPDMA_VIRT2SOCADDR(pDescCh->pDescWrite);
                }
            }
            else
            {
                Enet_assert(pDescCh->descFreeCount > 0);
                pDescThis->pNext = ENET_CPDMA_VIRT2SOCADDR(pDescCh->pDescWrite);
            }
            if (!isCsumIteration)
            {
                /* If this iteration is for chksuminfo, sgList.list[0] is not handled yet, do not increment the index */
                scatterSegmentIndex++;
            }
            loopCount++;
        }
        pEndDesc = pDescThis;
        EnetQueue_enq(&pDescCh->descQueue, &pPkt->node);
    }
    if (pPkt != NULL)
    {
        /* last descriptor in the chain should have pointer to next descriptor as NULL */
        if (pDescCh->pDescTail)
        {
            Enet_assert((pEndDesc != NULL) && (pStartDesc != NULL));
            Enet_assert(pDescCh->pDescTail->pNext == NULL);
            pDescCh->pDescTail->pNext = ENET_CPDMA_VIRT2SOCADDR(pStartDesc);
            pDescCh->pDescTail = pEndDesc;
        }
        else
        {
            CSL_CPSW_setCpdmaTxHdrDescPtr(pDescCh->hEnetDma->cpdmaRegs,
                                          (uint32_t)ENET_CPDMA_VIRT2SOCADDR(pStartDesc),
                                          pDescCh->chInfo->chNum);
            pDescCh->pDescTail = pEndDesc;
        }
    }
}

/** ============================================================================
 *  @n@b EnetCpdma_dequeueTx()
 *
 *  @b Description
 *  @n Dequeue all completed TX packets and return buffers to freeQueue
 *
 *  @b Arguments
 *  @verbatim
        pDescCh     pointer to channel descriptor
        pDescAck    pointer to Descriptor object (from ISR)
    @endverbatim
 *
 *  <b> Return Value </b>  None
 *
 *  <b> Pre Condition </b>
 *  @n  None
 *
 *  <b> Post Condition </b>
 *  @n  Dequeue all completed TX packets and return buffers to application
 *
 *  @b Example
 *  @verbatim
        EnetCpdma_DescCh *pDescCh;
        EnetCpdma_cppiDesc *pDescCp;

        EnetCpdma_dequeueTx ( pDescCh, pDescCp);
    @endverbatim
 * ============================================================================
 */
static bool EnetCpdma_dequeueTx(EnetCpdma_DescCh *pDescCh, EnetCpdma_cppiDesc *pDescCp)
{
    EnetCpdma_PktInfo *pPkt = NULL;
    volatile EnetCpdma_cppiDesc *pDesc;
    bool matchFound = false;
    volatile EnetCpdma_cppiDesc *pDescFirst = NULL;
    volatile EnetCpdma_cppiDesc *pDescLast = NULL;

    while (EnetQueue_getQCount(&pDescCh->descQueue) > 0U)
    {
        pDesc = pDescCh->pDescRead;
        Enet_assert(((pDesc->pktFlgLen) & ENET_CPDMA_DESC_PKT_FLAG_SOP) != 0);
        if ((pDesc->pktFlgLen) & ENET_CPDMA_DESC_PKT_FLAG_OWNER)
        {
            break;
        }
        else
        {
            if (pDescFirst == NULL)
            {
                pDescFirst = pDesc;
            }
            pDescLast = pDesc;
            if (pDescCh->unackedCpDescFirst == NULL)
            {
                pDescCh->unackedCpDescFirst = pDesc;
            }
            pDescCh->unackedCpDescCount++;
            if ((pDescCp != NULL) && (pDescCp == pDesc))
            {
                matchFound = true;
                pDescCh->unackedCpDescCount = 0;
                pDescCh->unackedCpDescFirst = NULL;
            }
            /* we now own the packet meaning its been transferred to the port */
            pPkt = (EnetCpdma_PktInfo *)EnetQueue_deq(&pDescCh->descQueue);
            Enet_assert(pPkt != NULL);

            if((pDesc->pktFlgLen) & ENET_CPDMA_DESC_PKT_FLAG_CHKSUM_ENCAP)
            {
                /* This is the chksum info desc, Move the read pointer */
                if (pDescCh->pDescRead == pDescCh->pDescLast)
                {
                    pDescCh->pDescRead = pDescCh->pDescFirst;
                }
                else
                {
                    pDescCh->pDescRead++;
                }
                pDescCh->descFreeCount++;
                pDesc = pDescCh->pDescRead;
                pDescLast = pDesc;
                if (pDescCh->unackedCpDescFirst == NULL)
                {
                    pDescCh->unackedCpDescFirst = pDesc;
                }
                pDescCh->unackedCpDescCount++;
                if ((pDescCp != NULL) && (pDescCp == pDesc))
                {
                    matchFound = true;
                    pDescCh->unackedCpDescCount = 0;
                    pDescCh->unackedCpDescFirst = NULL;
                }
            }

            if (pPkt->sgList.numScatterSegments == 1)
            {
                /* Move the read pointer */
                if (pDescCh->pDescRead == pDescCh->pDescLast)
                {
                    pDescCh->pDescRead = pDescCh->pDescFirst;
                }
                else
                {
                    pDescCh->pDescRead++;
                }
                pDescCh->descFreeCount++;

                if (pDescCh->pDescTail == pDesc)
                {
                    pDescCh->pDescTail = NULL;
                    Enet_assert(pDesc->pNext == NULL);
                }
                /* we have EOQ and we need to see if any packets chained to this one which will require re-start of transmitter */
                if( pDesc->pktFlgLen & ENET_CPDMA_DESC_PKT_FLAG_EOQ)
                {
                    if(pDesc->pNext)
                    {
                        CSL_CPSW_setCpdmaTxHdrDescPtr(pDescCh->hEnetDma->cpdmaRegs, (uint32_t)pDesc->pNext,
                                                      pDescCh->chInfo->chNum);
                    }
                }
                /* EOQ is not set, check to see if we chained any descriptors, if we did, continue with loop */
                else
                {
                    if(pDesc->pNext)
                    {
                        /* pNext should point to the next Read pointer */
                        Enet_assert (pDescCh->pDescRead == ENET_CPDMA_SOC2VIRTADDR(pDesc->pNext));
                    }
                }
            }
            else
            {
                uint32_t i;

                /* Move the read pointer */
                if (pDescCh->pDescRead == pDescCh->pDescLast)
                {
                    pDescCh->pDescRead = pDescCh->pDescFirst;
                }
                else
                {
                    pDescCh->pDescRead++;
                }
                pDescCh->descFreeCount++;

                Enet_assert (pDescCh->pDescTail != pDesc);
                /* we have EOQ and we need to see if any packets chained to this one which will require re-start of transmitter */
                Enet_assert (( pDesc->pktFlgLen & ENET_CPDMA_DESC_PKT_FLAG_EOQ) == 0);
                Enet_assert (pDesc->pNext != NULL);

                for (i = 1; i < pPkt->sgList.numScatterSegments;i++)
                {
                    pDesc = pDescCh->pDescRead;

                    pDescLast = pDesc;
                    if (pDescCh->unackedCpDescFirst == NULL)
                    {
                        pDescCh->unackedCpDescFirst = pDesc;
                    }
                    pDescCh->unackedCpDescCount++;
                    if ((pDescCp != NULL) && (pDescCp == pDesc))
                    {
                        matchFound = true;
                        pDescCh->unackedCpDescCount = 0;
                        pDescCh->unackedCpDescFirst = NULL;
                    }
                    Enet_assert(((pDesc->pktFlgLen) & ENET_CPDMA_DESC_PKT_FLAG_SOP) == 0);
                    Enet_assert(((pDesc->pktFlgLen) & ENET_CPDMA_DESC_PKT_FLAG_OWNER) == 0);
                    /* Move the read pointer */
                    if (pDescCh->pDescRead == pDescCh->pDescLast)
                    {
                        pDescCh->pDescRead = pDescCh->pDescFirst;
                    }
                    else
                    {
                        pDescCh->pDescRead++;
                    }
                    pDescCh->descFreeCount++;

                    if (pDescCh->pDescTail == pDesc)
                    {
                        pDescCh->pDescTail = NULL;
                        Enet_assert(pDesc->pNext == NULL);
                    }
                    if (i == (pPkt->sgList.numScatterSegments - 1))
                    {
                        Enet_assert(pDesc->pktFlgLen & ENET_CPDMA_DESC_PKT_FLAG_EOP);
                    }
                    /* we have EOQ and we need to see if any packets chained to this one which will require re-start of transmitter */
                    if( pDesc->pktFlgLen & ENET_CPDMA_DESC_PKT_FLAG_EOQ)
                    {
                        Enet_assert(i == (pPkt->sgList.numScatterSegments - 1));
                        if(pDesc->pNext)
                        {
                            CSL_CPSW_setCpdmaTxHdrDescPtr(pDescCh->hEnetDma->cpdmaRegs, (uint32_t)pDesc->pNext,
                                                          pDescCh->chInfo->chNum);
                        }
                    }
                    /* EOQ is not set, check to see if we chained any descriptors, if we did, continue with loop */
                    else
                    {
                        if(pDesc->pNext)
                        {
                            /* pNext should point to the next Read pointer */
                            Enet_assert (pDescCh->pDescRead == ENET_CPDMA_SOC2VIRTADDR(pDesc->pNext));
                        }
                    }
                }
            }
            EnetQueue_enq(&pDescCh->freeQueue, &pPkt->node);
        }
    }
    if ((!matchFound) && (pDescCp != NULL) && (pDescCh->unackedCpDescCount > 0))
    {
        uint32_t matchIndex;

        if ((pDescCp >= pDescCh->unackedCpDescFirst))
        {
            uint32_t unackedCpDescCount;

            Enet_assert(pDescCp <= pDescCh->pDescLast);
            if((pDescCh->unackedCpDescFirst + pDescCh->unackedCpDescCount) <= (pDescCh->pDescLast + 1))
            {
                unackedCpDescCount = pDescCh->unackedCpDescCount;
            }
            else
            {
                unackedCpDescCount = (pDescCh->pDescLast + 1) - pDescCh->unackedCpDescFirst;
            }
            if (pDescCp < (pDescCh->unackedCpDescFirst + unackedCpDescCount))
            {
                matchFound = true;
                matchIndex = (pDescCp - pDescCh->unackedCpDescFirst);
            }
        }
        else
        {
            volatile EnetCpdma_cppiDesc *pDescCpWrapped;

            Enet_assert(pDescCp <= pDescCh->pDescLast);
            Enet_assert(pDescCp >= pDescCh->pDescFirst);
            pDescCpWrapped = ((pDescCh->pDescLast + 1) + (pDescCp - pDescCh->pDescFirst));
            if (pDescCpWrapped < (pDescCh->unackedCpDescFirst + pDescCh->unackedCpDescCount))
            {
                matchFound = true;
                matchIndex = (pDescCpWrapped - pDescCh->unackedCpDescFirst);
            }
        }
        if (matchFound)
        {
            pDescCh->unackedCpDescCount -= (matchIndex + 1);
            if (pDescCh->unackedCpDescCount)
            {
                pDescCh->unackedCpDescFirst = (pDescCp + 1);
                if (pDescCh->unackedCpDescFirst == (pDescCh->pDescLast + 1))
                {
                    pDescCh->unackedCpDescFirst = pDescCh->pDescFirst;
                }
            }
            else
            {
                pDescCh->unackedCpDescFirst = NULL;
            }
        }
    }
    pDescCh->descHistory.descEntry[pDescCh->descHistory.descHistoryCount % ENET_ARRAYSIZE(pDescCh->descHistory.descEntry)].listBegin = pDescFirst;
    pDescCh->descHistory.descEntry[pDescCh->descHistory.descHistoryCount % ENET_ARRAYSIZE(pDescCh->descHistory.descEntry)].listEnd = pDescLast;
    pDescCh->descHistory.descEntry[pDescCh->descHistory.descHistoryCount % ENET_ARRAYSIZE(pDescCh->descHistory.descEntry)].cpDesc = pDescCp;
    pDescCh->descHistory.descEntry[pDescCh->descHistory.descHistoryCount % ENET_ARRAYSIZE(pDescCh->descHistory.descEntry)].unackedCpDescCount = pDescCh->unackedCpDescCount;
    pDescCh->descHistory.descEntry[pDescCh->descHistory.descHistoryCount % ENET_ARRAYSIZE(pDescCh->descHistory.descEntry)].unackedCpDescFirst = pDescCh->unackedCpDescFirst;
    pDescCh->descHistory.descHistoryCount++;

    return matchFound;
}

/** =========================================================================
 *  @n@b EnetCpdma_enqueueRx()
 *
 *  @b Description
 *  @n Fill any empty RX descriptors with new buffers from the free queue
 *
 *  @b Arguments
 *  @verbatim
        pDescCh     pointer to Descriptor object
        fRestart    re-fill packet
    @endverbatim
 *
 *  <b> Return Value </b>  None
 *
 *  <b> Pre Condition </b>
 *  @n  None
 *
 *  <b> Post Condition </b>
 *  @n  Fill any empty RX descriptors with new buffers from the freeQueue.
 *
 *  @b Example
 *  @verbatim
        EnetCpdma_DescCh  *pDescCh;
        uint32_t         fRestart;

        EnetCpdma_enqueueRx( pDescCh, fRestart );
    @endverbatim
 * ============================================================================
 */
void EnetCpdma_enqueueRx(EnetCpdma_DescCh *pDescCh)
{
    EnetCpdma_PktInfo     *pPkt = NULL;
    EnetCpdma_cppiDesc    *pDescThis, *pStartPtr;
    uintptr_t             key;
    uint32_t              scatterSegmentIndex;
    bool                  exit = false;
    EnetCpdma_PktInfo     *pPktNext = NULL;

    key = EnetOsal_disableAllIntr();

    pStartPtr = pDescCh->pDescWrite;

    /* First Packet */
    if (EnetQueue_getQCount(&pDescCh->freeQueue) > 0)
    {
        pPkt = (EnetCpdma_PktInfo*) EnetQueue_deq(&pDescCh->freeQueue);
        Enet_assert(pPkt != NULL);
        Enet_assert(pPkt->sgList.numScatterSegments > 0);
        Enet_assert(pPkt->sgList.numScatterSegments <= ENET_CPDMA_CPSW_MAX_SG_LIST);
        /* Check if the pkt can fit in the free descs */
        if (pDescCh->descFreeCount < pPkt->sgList.numScatterSegments)
        {
            /* Enq the pkt back into freeQueue */
            EnetQueue_enq(&pDescCh->freeQueue, &pPkt->node);
            pPkt = NULL;
        }
    }

    /* Fill RX Packets Until Full */
    while ((pDescCh->descFreeCount > 0) && (pPkt != NULL))
    {
        scatterSegmentIndex = 0;
        while (scatterSegmentIndex < pPkt->sgList.numScatterSegments)
        {
            /* Fill in the descriptor for this buffer */
            pDescThis = pDescCh->pDescWrite;

            /* Move the write pointer and bump count */
            if (pDescCh->pDescWrite == pDescCh->pDescLast)
            {
                pDescCh->pDescWrite = pDescCh->pDescFirst;
            }
            else
            {
                pDescCh->pDescWrite++;
            }
            pDescCh->descFreeCount--;
            pDescCh->descSubmittedCount++;

            pDescThis->pBuffer   = ENET_CPDMA_VIRT2SOCADDR(pPkt->sgList.list[scatterSegmentIndex].origBufPtr);
            pDescThis->bufOffLen = pPkt->sgList.list[scatterSegmentIndex].segmentAllocLen;
            pDescThis->orgBufLen = pPkt->sgList.list[scatterSegmentIndex].segmentAllocLen;
            pDescThis->pktFlgLen = ENET_CPDMA_DESC_PKT_FLAG_OWNER;

            EnetOsal_cacheInv((void*)pPkt->sgList.list[scatterSegmentIndex].origBufPtr, pPkt->sgList.list[scatterSegmentIndex].segmentAllocLen);

            if (scatterSegmentIndex == pPkt->sgList.numScatterSegments - 1)
            {
                /*
                 * If this is the last descriptor, the forward pointer is (void *)0
                 * Otherwise; this desc points to the next desc in the wait queue
                 */
                /* We exit the loop if we do not have any free dmapktinfos or
                 * free descs or the descs are not sufficient for the sglist */
                if ((EnetQueue_getQCount(&pDescCh->freeQueue) == 0U) || (pDescCh->descFreeCount == 0U))
                {
                    pDescThis->pNext = NULL;
                    exit = true;
                }
                else
                {
                    /* Get a new buffer from the free Queue */
                    pPktNext = (EnetCpdma_PktInfo*) EnetQueue_deq(&pDescCh->freeQueue);
                    Enet_assert(pPktNext != NULL);
                    Enet_assert(pPktNext->sgList.numScatterSegments > 0);
                    Enet_assert(pPktNext->sgList.numScatterSegments <= ENET_CPDMA_CPSW_MAX_SG_LIST);
                    /* Check if the pkt can fit in the free descs */
                    if (pDescCh->descFreeCount < pPktNext->sgList.numScatterSegments)
                    {
                        pDescThis->pNext = NULL;
                        /* Enq the pkt back into freeQueue */
                        EnetQueue_enq(&pDescCh->freeQueue, &pPktNext->node);
                        exit = true;
                    }
                    else
                    {
                        pDescThis->pNext = ENET_CPDMA_VIRT2SOCADDR(pDescCh->pDescWrite);
                    }
                }
            }
            else
            {
                pDescThis->pNext = ENET_CPDMA_VIRT2SOCADDR(pDescCh->pDescWrite);
            }

            scatterSegmentIndex++;
        }

        EnetQueue_enq(&pDescCh->descQueue, &pPkt->node);
        if (exit)
        {
            break;
        }
        else
        {
            Enet_assert(pPktNext != NULL);
            pPkt = pPktNext;
        }
    }
    if (pPkt != NULL)
    {
        Enet_assert(exit == true);
        /* last descriptor in the chain should have pointer to next descriptor as NULL */
        if (pDescCh->pDescTail)
        {
            Enet_assert((pDescThis != NULL) && (pStartPtr != NULL));
            Enet_assert(pDescCh->pDescTail->pNext == NULL);
            pDescCh->pDescTail->pNext = ENET_CPDMA_VIRT2SOCADDR(pStartPtr);
            pDescCh->pDescTail = pDescThis;
        }
        else
        {
            CSL_CPSW_setCpdmaRxHdrDescPtr(pDescCh->hEnetDma->cpdmaRegs,
                                          (uint32_t)ENET_CPDMA_VIRT2SOCADDR(pStartPtr),
                                          pDescCh->chInfo->chNum);
            pDescCh->pDescTail = pDescThis;
        }
    }

    EnetOsal_restoreAllIntr(key);
}

/** ============================================================================
 *  @n@b EnetCpdma_dequeueRx()
 *
 *  @b Description
 *  @n Dequeue all completed RX packets and start buffers to wait Queue
 *
 *  @b Arguments
 *  @verbatim
        pDescCh     pointer to descriptor channel object
        pDescAck    pointer to the acknowledge
    @endverbatim
 *
 *  <b> Return Value </b>  None
 *
 *  <b> Pre Condition </b>
 *  @n  None
 *
 *  <b> Post Condition </b>
 *  @n  Dequeue all completed RX packets and give buffers to application
 *
 *  @b Example
 *  @verbatim
        EnetCpdma_DescCh *pDescCh;
        EnetCpdma_cppiDesc   *pDescAck;

        EnetCpdma_dequeueRx( pDescCh, pDescAck );
    @endverbatim
 * ============================================================================
 */
bool EnetCpdma_dequeueRx(EnetCpdma_DescCh *pDescCh, EnetCpdma_cppiDesc *pDescCp)
{
    EnetCpdma_PktInfo  *pPkt = NULL;
    EnetCpdma_cppiDesc *pDesc;
    volatile EnetCpdma_cppiDesc *pDescFirst = NULL;
    volatile EnetCpdma_cppiDesc *pDescLast = NULL;
    uintptr_t key;
    EnetCpdma_SGListEntry *list;
    bool matchFound = false;
    uint32_t pktFlgLen, totalLenReceived, totalPktLen;
    uint32_t numScatterSegments = 0U;
    uint32_t firstWordTs = 0;
    uint32_t secondWordTs = 0;
    uint32_t hasTimeStamp = 0;

    key = EnetOsal_disableAllIntr();
    /*
     * Pop & Free Buffers 'till the last Descriptor
     * One thing we know for sure is that all the decriptors from
     * the read pointer to pDescAsk are linked to each other via
     * their pNext field.
    */
    /* TODO: error check only */
    while ((EnetQueue_getQCount(&pDescCh->descQueue) > 0U)
            && (pDescCh->descSubmittedCount > 0U))
    {
        pDesc = pDescCh->pDescRead;

        /* Get the status of this descriptor */
        /* Bit 16,17 and 18 indicate the port number(ingress)
         * Passcrc bit is always set in the received packets.Clear it before putting the \
         * packet in receive queue */
        /* This should be a SOP descriptor */
        const uint32_t sopPktFlgLen = (pDesc->pktFlgLen) & ((uint32_t)~ENET_CPDMA_DESC_PSINFO_PASSCRC_FLAG);
        void *appPriv = NULL;

        hasTimeStamp = sopPktFlgLen & ENET_CPDMA_DESC_PSINFO_RX_TS_ENCAP_MASK;

        /* Check the ownership of the packet. Ownership is only valid for SOP descs */
        if ( ((sopPktFlgLen & ENET_CPDMA_DESC_PKT_FLAG_OWNER) == 0U)
                && ((sopPktFlgLen & ENET_CPDMA_DESC_PKT_FLAG_SOP) != 0U))
        {
            pPkt = (EnetCpdma_PktInfo*) EnetQueue_deq(&pDescCh->descQueue);
            Enet_assert(pPkt != NULL);
            appPriv = pPkt->appPriv;
            EnetDma_initPktInfo(pPkt);
            pPkt->appPriv = appPriv;
            numScatterSegments = 0;
            totalLenReceived = 0;
            totalPktLen = (sopPktFlgLen & ENET_CPDMA_DESC_PSINFO_RX_PACKET_LEN_MASK);

            /* Loop through the descs until we get EOP */
            while (numScatterSegments < ENET_CPDMA_CPSW_MAX_SG_LIST)
            {
                pDesc = pDescCh->pDescRead;
                /* Move the read pointer and decrement count */
                if (pDescCh->pDescRead == pDescCh->pDescLast)
                {
                    pDescCh->pDescRead = pDescCh->pDescFirst;
                }
                else
                {
                    pDescCh->pDescRead++;
                }
                pDescCh->descFreeCount++;
                pDescCh->descSubmittedCount--;

                pktFlgLen = (pDesc->pktFlgLen) & ((uint32_t)~ENET_CPDMA_DESC_PSINFO_PASSCRC_FLAG);
                if (pDescFirst == NULL)
                {
                    pDescFirst = pDesc;
                }
                pDescLast = pDesc;
                if (pDescCh->unackedCpDescFirst == NULL)
                {
                    pDescCh->unackedCpDescFirst = pDesc;
                }
                pDescCh->unackedCpDescCount++;
                if ((pDescCp != NULL) && (pDescCp == pDesc))
                {
                    matchFound = true;
                    pDescCh->unackedCpDescCount = 0;
                    pDescCh->unackedCpDescFirst = NULL;
                }
                if (pDescCh->pDescTail == pDesc)
                {
                    pDescCh->pDescTail = NULL;
                    Enet_assert(pDesc->pNext == NULL);
                }

                /* Fill the sglist of dmapktinfo */
                list = &pPkt->sgList.list[numScatterSegments];

                if (numScatterSegments == 0)
                {
                    if (hasTimeStamp != 0)
                    {
                        /* Storing the original buffer pointer given by descriptor for submitting it back in enqueueRx */
                        list->origBufPtr = pDesc->pBuffer;
                        list->segmentFilledLen = (pDesc->bufOffLen & ENET_CPDMA_DESC_PSINFO_RX_PACKET_LEN_MASK)
                                                    - CPSW_CPDMA_RX_HOST_TIMESTAMP_SIZE;

                        /* Shifting the bufptr to point the packet header by incrementing the size of timeStamp */
                        list->bufPtr = (uint8_t*)list->origBufPtr + CPSW_CPDMA_RX_HOST_TIMESTAMP_SIZE;

                        /* Storing the timeStamp(first 8 bytes prepended to the packet) and updating the total packet length  */
                        firstWordTs = *((uint32_t*)list->origBufPtr);
                        secondWordTs = *((uint32_t*)(list->origBufPtr + (CPSW_CPDMA_RX_HOST_TIMESTAMP_SIZE >> 1)));
                        pPkt->tsInfo.rxPktTs = (uint64_t)(secondWordTs | ((uint64_t)firstWordTs << 32));

                        totalPktLen = totalPktLen - CPSW_CPDMA_RX_HOST_TIMESTAMP_SIZE;
                    }
                    else
                    {
                        /* Non-SOP descriptors in case of RX scatter segments handled here */
                        list->bufPtr = pDesc->pBuffer;
                        list->origBufPtr = pDesc->pBuffer;
                        list->segmentFilledLen = (pDesc->bufOffLen & ENET_CPDMA_DESC_PSINFO_RX_PACKET_LEN_MASK);
                        pPkt->tsInfo.rxPktTs = 0;
                    }
                }
                else
                {
                    list->bufPtr = pDesc->pBuffer;
                    list->origBufPtr = pDesc->pBuffer;
                    list->segmentFilledLen = (pDesc->bufOffLen & ENET_CPDMA_DESC_PSINFO_RX_PACKET_LEN_MASK);
                }

                list->segmentAllocLen = pDesc->orgBufLen;
                Enet_assert(list->segmentAllocLen >= list->segmentFilledLen);
                totalLenReceived += list->segmentFilledLen;
                numScatterSegments++;
                /* Acking Rx completion interrupt is done in Rx Isr. Dont do it here  */
                if (pktFlgLen & ENET_CPDMA_DESC_PKT_FLAG_EOQ)
                {
                    Enet_assert((pktFlgLen & ENET_CPDMA_DESC_PKT_FLAG_EOP) != 0);
                    if (pDesc->pNext)
                    {
                        CSL_CPSW_setCpdmaRxHdrDescPtr(pDescCh->hEnetDma->cpdmaRegs, (uint32_t) pDesc->pNext, pDescCh->chInfo->chNum);
                    }
                }
                /* EOQ is not set, check to see if we chained any descriptors, if we did, continue with loop */
                else
                {
                    if (pDesc->pNext)
                    {
                        /* pNext should point to the next Read pointer */
                        Enet_assert(pDescCh->pDescRead == ENET_CPDMA_SOC2VIRTADDR(pDesc->pNext));
                    }
                }
                if ((pktFlgLen & ENET_CPDMA_DESC_PKT_FLAG_EOP) != 0U)
                {
                    pPkt->sgList.numScatterSegments = numScatterSegments;
                    break;
                }
            }
            /* Assert if total pktlen is not equal to sum of buffer lengths */
            Enet_assert(totalLenReceived == totalPktLen);
            Enet_assert(pPkt->sgList.numScatterSegments > 0);

            if (sopPktFlgLen & ENET_CPDMA_DESC_PSINFO_RX_CHECKSUM_INFO_MASK)
            {
                Enet_assert(totalLenReceived >= ENET_CPDMA_ENCAPINFO_CHECKSUM_INFO_LEN);
                /* fall here if CHECKSUM OFFLOAD to CPSW is enabled */
                /* First,remove the THOST encapsulated word (suffixed) which is the last 4 bytes of the pkt */
                list = &pPkt->sgList.list[pPkt->sgList.numScatterSegments - 1];
                /* If the last segment has the complete 4 byte chksum encap info */
                if (list->segmentFilledLen >= ENET_CPDMA_ENCAPINFO_CHECKSUM_INFO_LEN)
                {
                    list->segmentFilledLen -= ENET_CPDMA_ENCAPINFO_CHECKSUM_INFO_LEN;
                    /* Then, Copy Checksum encapsulation word*/
                    memcpy(&pPkt->chkSumInfo, &(list->bufPtr[list->segmentFilledLen]), ENET_CPDMA_ENCAPINFO_CHECKSUM_INFO_LEN);
                }
                else
                {
                    /* The 4 bytes of chksum encap info are spread across the last two segments */
                    uint32_t csumInfoSuffixByteCnt = list->segmentFilledLen;
                    uint32_t csumInfoPrefixByteCnt = ENET_CPDMA_ENCAPINFO_CHECKSUM_INFO_LEN - csumInfoSuffixByteCnt;
                    uint8_t *pSuffix = (uint8_t *)&pPkt->chkSumInfo + csumInfoPrefixByteCnt;

                    /* Copy the Suffix of Checksum encapsulation word */
                    memcpy((void *) pSuffix, list->bufPtr, csumInfoSuffixByteCnt);
                    list->segmentFilledLen = 0;
                    Enet_assert(pPkt->sgList.numScatterSegments >= 2);
                    list = &pPkt->sgList.list[pPkt->sgList.numScatterSegments - 2];
                    list->segmentFilledLen -= csumInfoPrefixByteCnt;
                    /* Copy the Prefix of Checksum encapsulation word */
                    memcpy(&pPkt->chkSumInfo, &(list->bufPtr[list->segmentFilledLen]), csumInfoPrefixByteCnt);
                }
            }
            pPkt->rxPortNum = ENET_MACPORT_DENORM(((sopPktFlgLen & ENET_CPDMA_DESC_PSINFO_FROM_PORT_MASK)
                                             >> ENET_CPDMA_DESC_PSINFO_FROM_PORT_SHIFT) -1U );

            /* Store the packet to the wait Queue */
            EnetQueue_enq(&pDescCh->waitQueue, &pPkt->node);
        }
        else
        {
            break;
        }
    }
    if ((!matchFound) && (pDescCp != NULL) && (pDescCh->unackedCpDescCount > 0))
    {
        uint32_t matchIndex;

        if ((pDescCp >= pDescCh->unackedCpDescFirst))
        {
            uint32_t unackedCpDescCount;

            Enet_assert(pDescCp <= pDescCh->pDescLast);
            if((pDescCh->unackedCpDescFirst + pDescCh->unackedCpDescCount) <= (pDescCh->pDescLast + 1))
            {
                unackedCpDescCount = pDescCh->unackedCpDescCount;
            }
            else
            {
                unackedCpDescCount = (pDescCh->pDescLast + 1) - pDescCh->unackedCpDescFirst;
            }
            if (pDescCp < (pDescCh->unackedCpDescFirst + unackedCpDescCount))
            {
                matchFound = true;
                matchIndex = (pDescCp - pDescCh->unackedCpDescFirst);
            }
        }
        else
        {
            volatile EnetCpdma_cppiDesc *pDescCpWrapped;

            Enet_assert(pDescCp <= pDescCh->pDescLast);
            Enet_assert(pDescCp >= pDescCh->pDescFirst);
            pDescCpWrapped = ((pDescCh->pDescLast + 1) + (pDescCp - pDescCh->pDescFirst));
            if (pDescCpWrapped < (pDescCh->unackedCpDescFirst + pDescCh->unackedCpDescCount))
            {
                matchFound = true;
                matchIndex = (pDescCpWrapped - pDescCh->unackedCpDescFirst);
            }
        }
        if (matchFound)
        {
            pDescCh->unackedCpDescCount -= (matchIndex + 1);
            if (pDescCh->unackedCpDescCount)
            {
                pDescCh->unackedCpDescFirst = (pDescCp + 1);
                if (pDescCh->unackedCpDescFirst == (pDescCh->pDescLast + 1))
                {
                    pDescCh->unackedCpDescFirst = pDescCh->pDescFirst;
                }
            }
            else
            {
                pDescCh->unackedCpDescFirst = NULL;
            }
        }
    }
    pDescCh->descHistory.descEntry[pDescCh->descHistory.descHistoryCount % ENET_ARRAYSIZE(pDescCh->descHistory.descEntry)].listBegin = pDescFirst;
    pDescCh->descHistory.descEntry[pDescCh->descHistory.descHistoryCount % ENET_ARRAYSIZE(pDescCh->descHistory.descEntry)].listEnd = pDescLast;
    pDescCh->descHistory.descEntry[pDescCh->descHistory.descHistoryCount % ENET_ARRAYSIZE(pDescCh->descHistory.descEntry)].cpDesc = pDescCp;
    pDescCh->descHistory.descEntry[pDescCh->descHistory.descHistoryCount % ENET_ARRAYSIZE(pDescCh->descHistory.descEntry)].unackedCpDescCount = pDescCh->unackedCpDescCount;
    pDescCh->descHistory.descEntry[pDescCh->descHistory.descHistoryCount % ENET_ARRAYSIZE(pDescCh->descHistory.descEntry)].unackedCpDescFirst = pDescCh->unackedCpDescFirst;
    pDescCh->descHistory.descHistoryCount++;
    EnetOsal_restoreAllIntr(key);
    return matchFound;
}

/**
 *  @b EnetCpdma_rxIsrProc
 *  @n
 *     Common Processing function for both Rx and Rx Thresh interrupt
 *
 *  @param[in]  hEnetDma
 *
 *  @retval
 *      ENET_SOK
 */
int32_t EnetCpdma_rxIsrProc(EnetDma_Handle hEnetDma, uint32_t cpdmaCoreId)
{
    int32_t          retVal = ENET_SOK;
    uint32_t         desc;
    uint32_t         intFlags = 0U;
    uint32_t         chNum = 0U;

	/* Validate our handle */
    if (hEnetDma == NULL)
    {
        retVal = ENET_EBADARGS;
    }
	else
	{
		/* Read the Rx interrupt status from RX_STAT */
        intFlags = CSL_CPSW_getRxIntStatus(hEnetDma->cpswSsRegs, cpdmaCoreId);
		intFlags = (uint32_t)(intFlags & 0xFFU);

		/* Look for receive interrupts from across all CPDMA Rx Channels */
        while (0U != intFlags)
        {
            uint32_t chMask = (uint32_t)((uint32_t)0x00000001U << (uint32_t)chNum);

            if ( (uint32_t)0U != (uint32_t)(intFlags & chMask))
            {
                EnetCpdma_DescCh  *rxChan = &hEnetDma->rxCppi[chNum];

                CSL_CPSW_getCpdmaRxCp(hEnetDma->cpdmaRegs, chNum, &desc);

                if (desc == 0U)
                {
                    /* False alarm, do nothing */
                }
                else if (desc == ENET_CPDMA_TEARDOWN_DESC)
                {
                    /* Terdown is in progress */
                    /* ToDO: should we invoke EnetCpdma_dequeueRx */
                    CSL_CPSW_setCpdmaRxCp(hEnetDma->cpdmaRegs, chNum, ENET_CPDMA_TEARDOWN_DESC);
                }
                else
                {
                    EnetCpdma_cppiDesc * cpDesc;
                    bool descMatch;

                    if(hEnetDma->isResetOngoing == false)
                    {
                        cpDesc = ENET_CPDMA_SOC2VIRTADDR((EnetCpdma_cppiDesc *)desc);
                        descMatch = EnetCpdma_dequeueRx(rxChan, cpDesc);
                        if(descMatch == false)
                        {
                            EnetCpdma_enqueueRx(rxChan);
                            CSL_CPSW_setCpdmaRxCp(hEnetDma->cpdmaRegs, chNum, desc);
                        }
                        else
                        {
                            Enet_assert(descMatch == true);
                        }
                    }
                }

                /* Invoke callback function */
                if(NULL != rxChan->chInfo->notifyCb)
                {
                    rxChan->chInfo->notifyCb(rxChan->chInfo->hCbArg);
                }
			}

            /* Clear the channel flag for the channel just handled */
            intFlags &= ~chMask;
            chNum++;
		}
    }

    return (retVal);
}
/**
 *  @b EnetCpdma_rxThreshIsr
 *  @n
 *      CPSW CPDMA Receive Threshold ISR.
 *
 *  @param[in]  hEnetDma
 *
 *  @retval
 *      ENET_SOK
 */
int32_t EnetCpdma_rxThreshIsr(EnetDma_Handle hEnetDma)
{
    int32_t retVal = ENET_SOK;

    /* Validate our handle */
    if (hEnetDma == NULL)
    {
        retVal = ENET_EBADARGS;
    }
    else
    {
        retVal = EnetCpdma_rxIsrProc(hEnetDma, hEnetDma->cpdmaCoreId);
        CSL_CPSW_setCpdmaRxThresholdEndOfIntVector(hEnetDma->cpdmaRegs, hEnetDma->cpdmaCoreId);
    }

    return (retVal);
}

/**
 *  @b EnetCpdma_rxIsr
 *  @n
 *      CPSW CPDMA Receive ISR.
 *
 *  @param[in]  hEnetDma
 *
 *  @retval
 *      ENET_SOK
 */
int32_t EnetCpdma_rxIsr(EnetDma_Handle hEnetDma)
{
    int32_t retVal = ENET_SOK;

    /* Validate our handle */
    if (hEnetDma == NULL)
    {
        retVal = ENET_EBADARGS;
    }
    else
    {
        retVal = EnetCpdma_rxIsrProc(hEnetDma, hEnetDma->cpdmaCoreId);
        CSL_CPSW_setCpdmaRxEndOfIntVector(hEnetDma->cpdmaRegs, hEnetDma->cpdmaCoreId);
    }
    return (retVal);
}

/**
 *  @b EnetCpdma_txIsr
 *  @n
 *      CPSW CPDMA Transmit ISR.
 *
 *  @param[in]  hEnetDma
 *
 *  @retval
 *      ENET_SOK
 */
int32_t EnetCpdma_txIsr(EnetDma_Handle hEnetDma)
{
    int32_t          retVal = ENET_SOK;
    uint32_t         desc;
    uint32_t         intFlags = 0U;
    uint32_t         chNum = 0U;

    /* Validate our handle */
    if (hEnetDma == NULL)
    {
        retVal = ENET_EBADARGS;
    }
    else
    {
		/* Read the Tx interrupt status from TX_STAT, assume core 0 for now */
        intFlags = CSL_CPSW_getTxIntStatus(hEnetDma->cpswSsRegs, hEnetDma->cpdmaCoreId);
		intFlags = (uint32_t)(intFlags & 0xFFU);

		/* Look for receive interrupts from across all CPDMA Tx Channels */
        while (0U != intFlags)
        {
            uint32_t chMask = (uint32_t)((uint32_t)0x00000001U << (uint32_t)chNum);

            if ( (uint32_t)0U != (uint32_t)(intFlags & chMask))
            {
				EnetCpdma_DescCh  *txChan = &hEnetDma->txCppi[chNum];

                /* find out if interrupt happend */
                CSL_CPSW_getCpdmaTxCp(hEnetDma->cpdmaRegs, chNum, &desc);
                /* this should only happen while teardown is in process */
                if(desc == ENET_CPDMA_TEARDOWN_DESC)
                {
                    /* need to ack with acknowledge value */
                    CSL_CPSW_setCpdmaTxCp(hEnetDma->cpdmaRegs, chNum, ENET_CPDMA_TEARDOWN_DESC);
                }
                else
                {
                    EnetCpdma_cppiDesc * cpDesc;
                    bool descMatch;

                    if(hEnetDma->isResetOngoing == false)
                    {
                        cpDesc = ENET_CPDMA_SOC2VIRTADDR(((EnetCpdma_cppiDesc *)desc));
                        descMatch = EnetCpdma_dequeueTx(txChan, cpDesc);
                        Enet_assert(descMatch == true);
                        EnetCpdma_enqueueTx(txChan);
                        /* Ack Tx completion interrupt */
                        CSL_CPSW_setCpdmaTxCp(hEnetDma->cpdmaRegs, chNum, desc);
                    }
                }

                /* Invoke callback function */
                if(NULL != txChan->chInfo->notifyCb)
                {
                    txChan->chInfo->notifyCb(txChan->chInfo->hCbArg);
                }
            }

            /* Clear the channel flag for the channel just handled */
            intFlags &= ~chMask;
            chNum++;
        }

		/* write the EOI register */
		CSL_CPSW_setCpdmaTxEndOfIntVector(hEnetDma->cpdmaRegs, hEnetDma->cpdmaCoreId);
    }

    return (retVal);
}

/**
 *  @b EnetCpdma_miscIsr
 *  @n
 *      CPSW CPDMA Miscellaneous ISR.
 *
 *  @param[in]  hEnetDma
 *  @param[in]  pStatusMask
 *
 *  @retval
 *      ENET_SOK
 */
int32_t EnetCpdma_miscIsrGetStatus(EnetDma_Handle hEnetDma, uint32_t *pStatusMask)
{
    int32_t retVal = ENET_SOK;

    /* Validate our handle */
    if (hEnetDma == NULL)
    {
        retVal = ENET_EBADARGS;
    }
    else
    {
        *pStatusMask = CSL_CPSW_getWrMiscIntStatus(hEnetDma->cpswSsRegs, hEnetDma->cpdmaCoreId);
    }

    return (retVal);
}

/**
 *  @b EnetCpdma_ackMiscIsr
 *  @n
 *      CPSW CPDMA Miscellaneous ISR ACK.
 *
 *  @param[in]  hEnetDma
 *
 *  @retval
 *      ENET_SOK
 */
int32_t EnetCpdma_ackMiscIsr(EnetDma_Handle hEnetDma)
{
    int32_t retVal = ENET_SOK;

    /* Validate our handle */
    if (hEnetDma == NULL)
    {
        retVal = ENET_EBADARGS;
    }
    else
    {
        CSL_CPSW_setCpdmaMiscEndOfIntVector(hEnetDma->cpdmaRegs, hEnetDma->cpdmaCoreId);
    }

    return (retVal);
}

int32_t EnetCpdma_checkRxChSanity(EnetDma_RxChHandle hRxCh,
                                  uint32_t margin)
{
    int32_t retVal       = ENET_SOK;

    return retVal;
}

int32_t EnetCpdma_checkTxChSanity(EnetDma_TxChHandle hTxCh,
                                uint32_t margin)
{
    int32_t retVal       = ENET_SOK;

    return retVal;
}

static int32_t EnetCpdma_checkRxChParams(EnetCpdma_OpenRxChPrms *pRxChPrms)
{
    int32_t retVal = ENET_SOK;

    if (NULL == pRxChPrms)
    {
        ENETTRACE_ERR("[Enet CPDMA] Flow params is NULL!!\n");
        retVal = ENET_EBADARGS;
    }
    else
    {
        if (NULL == pRxChPrms->hEnet)
        {
            ENETTRACE_ERR("[Enet CPDMA] ENET not opened !!\n");
            retVal = ENET_EBADARGS;
        }
    }

    if (ENET_SOK == retVal)
    {
        if (pRxChPrms->chNum >= ENET_CPDMA_MAX_CHANNELS)
        {
            ENETTRACE_ERR("[Enet CPDMA] channel number for CPSW RX channel open !!\n", pRxChPrms->chNum);
            retVal = ENET_EBADARGS;
        }
    }
    return retVal;
}

static int32_t EnetCpdma_checkTxChParams(EnetCpdma_OpenTxChPrms *pTxChPrms)
{
    int32_t retVal = ENET_SOK;

    if (NULL == pTxChPrms)
    {
        ENETTRACE_ERR("[Enet CPDMA] TX channel params can't be NULL !!\n");
        retVal = ENET_EBADARGS;
    }
    else
    {
        if (NULL == pTxChPrms->hEnet)
        {
            /* DMA should be opened before opening Ch/Flow */
            ENETTRACE_ERR("[Enet CPDMA] ENET not opened !!\n");
            retVal = ENET_EBADARGS;
        }
    }

    if (ENET_SOK == retVal)
    {
        if (pTxChPrms->chNum >= ENET_CPDMA_MAX_CHANNELS)
        {
            ENETTRACE_ERR("[Enet CPDMA] Invalid channel number for CPDMA TX channel open !!: %d\n", pTxChPrms->chNum);
            retVal = ENET_EBADARGS;
        }
    }

    return retVal;
}


void EnetCpdma_initParams(Enet_Type enetType, EnetDma_Cfg *pDmaConfig)
{
    pDmaConfig->enChOverrideFlag            = false;
    pDmaConfig->enHostRxTsFlag              = false;
    pDmaConfig->rxInterruptPerMSec          = 0U;
    pDmaConfig->txInterruptPerMSec          = 0U;
    pDmaConfig->rxChInitPrms.rxBufferOffset = 0U;
    pDmaConfig->maxTxChannels               = ENET_CPDMA_CPSW_MAX_TX_CH;
    pDmaConfig->maxRxChannels               = ENET_CPDMA_CPSW_MAX_RX_CH;
}

static int32_t EnetCpdma_checkCpdmaParams(EnetCpdma_Cfg *pDmaCfg)
{
    int32_t status = ENET_SOK;

    if ((pDmaCfg->maxTxChannels > ENET_CPDMA_CPSW_MAX_TX_CH)
        ||
        (pDmaCfg->maxRxChannels > ENET_CPDMA_CPSW_MAX_RX_CH))
    {
        status = ENET_EINVALIDPARAMS;
    }
    return status;
}



EnetDma_Handle EnetCpdma_open(Enet_Type enetType,
                            uint32_t instId,
                            const void *dmaCfg,
                            uint32_t appCoreId)
{
    EnetCpdma_DrvObj *pEnetDmaObj = NULL;
    EnetCpdma_Cfg *pDmaCfg = (EnetCpdma_Cfg *)dmaCfg;
    uint32_t status;
    int32_t retVal = ENET_SOK;
    uint32_t pacingBitMask = 0;

    /* Error check */
    if (NULL == pDmaCfg)
    {
        ENETTRACE_ERR("[ENET DMA Error] pDmaCfg NULL !!\n");
        retVal = ENET_EBADARGS;
    }

    if (ENET_SOK == retVal)
    {
        retVal = EnetCpdma_checkCpdmaParams(pDmaCfg);
        ENETTRACE_ERR_IF(retVal, "[ENET DMA Error] pDmaCfg Invalid params !!\n");
    }

    if (ENET_SOK == retVal)
    {
        if (EnetAppUtils_isDescCached() == true)
        {
            ENETTRACE_ERR("[ENET DMA Error] CPDMA descriptors are not in un-cached memory region !!\n");
            retVal = ENET_EINVALIDPARAMS;
        }
    }

    if (ENET_SOK == retVal)
    {
        /* TODO: more error check */
        pEnetDmaObj = EnetSoc_getDmaHandle(enetType, 0U /* instId */);
        Enet_assert(pEnetDmaObj != NULL);


        /* Saving the Context for DMA cfg */
        pEnetDmaObj->rxBufOffset = pDmaCfg->rxChInitPrms.rxBufferOffset;
        pEnetDmaObj->rxInterruptPerMSec = pDmaCfg->rxInterruptPerMSec;
        pEnetDmaObj->txInterruptPerMSec = pDmaCfg->txInterruptPerMSec;
        pEnetDmaObj->enChOverrideFlag = pDmaCfg->enChOverrideFlag;
        pEnetDmaObj->enHostRxTsFlag = pDmaCfg->enHostRxTsFlag;
        pEnetDmaObj->isResetOngoing = false;

        pEnetDmaObj->cpdmaCoreId = EnetCpdma_mapApp2CpdmaCoreId(appCoreId);

        /* TODO: how to configure Rx and Tx channel count (from sOC) */
        pEnetDmaObj->numTxChans = pDmaCfg->maxTxChannels;
        pEnetDmaObj->numRxChans = pDmaCfg->maxRxChannels;


        /* Initialize memory manager module managing driver object memories */
        EnetCpdma_memMgrInit();

        CSL_CPSW_resetCpdma(pEnetDmaObj->cpdmaRegs);
        status = CSL_CPSW_isCpdmaResetDone(pEnetDmaObj->cpdmaRegs);
        while (status == ((uint32_t)false))
        {
            status = CSL_CPSW_isCpdmaResetDone(pEnetDmaObj->cpdmaRegs);
        }

        CSL_CPSW_setCpdmaRxBufOffset(pEnetDmaObj->cpdmaRegs,
                                     pDmaCfg->rxChInitPrms.rxBufferOffset);

        /* TODO: why channel 0 only? Assume one channel for now */
        CSL_CPSW_disableCpdmaTxInt(pEnetDmaObj->cpdmaRegs, 0);
        CSL_CPSW_disableCpdmaRxInt(pEnetDmaObj->cpdmaRegs, 0);

        /* Acknowledge receive and transmit interrupts for proper interrupt pulsing */
        CSL_CPSW_setCpdmaTxEndOfIntVector(pEnetDmaObj->cpdmaRegs, pEnetDmaObj->cpdmaCoreId);
        CSL_CPSW_setCpdmaRxEndOfIntVector(pEnetDmaObj->cpdmaRegs, pEnetDmaObj->cpdmaCoreId);

        CSL_CPSW_enableCpdmaTx(pEnetDmaObj->cpdmaRegs);
        CSL_CPSW_enableCpdmaRx(pEnetDmaObj->cpdmaRegs);

        /* Enable Miscellaneous interrupts - stats and host error interupt */
        CSL_CPSW_enableCpdmaDmaInt(pEnetDmaObj->cpdmaRegs, CSL_CPDMA_DMA_INTMASK_SET_STAT_INT_MASK_MASK |
                                                           CSL_CPDMA_DMA_INTMASK_SET_HOST_ERR_INT_MASK_MASK);
        /* enable host,stats interrupt in cpsw_ss_s wrapper */
        /* TODO: should we enable CPTS/MDIO interrupt here */
        CSL_CPSW_enableWrMiscInt(pEnetDmaObj->cpswSsRegs, pEnetDmaObj->cpdmaCoreId,
                                 CPSW_MISC_INT_MDIO_USERINT_MASK |
                                 CPSW_MISC_INT_MDIO_LINKINT_MASK |
                                 CPSW_MISC_INT_HOSTERR_MASK |
                                 CPSW_MISC_INT_STAT_OVERFLOW_MASK |
                                 CPSW_MISC_INT_CPTS_EVENT_MASK);

        if (pDmaCfg->rxInterruptPerMSec != 0U)
        {
            /* enable Interrupt Pacing Logic in the Wrapper */
            CSL_CPSW_setWrRxIntPerMSec(pEnetDmaObj->cpswSsRegs, pEnetDmaObj->cpdmaCoreId, pDmaCfg->rxInterruptPerMSec);
            pacingBitMask |= CPSW_INT_CONTROL_INT_PACE_EN_C0_RX;
        }
        if (pDmaCfg->txInterruptPerMSec != 0U)
        {
            /* enable Interrupt Pacing Logic in the Wrapper */
            CSL_CPSW_setWrTxIntPerMSec(pEnetDmaObj->cpswSsRegs, pEnetDmaObj->cpdmaCoreId, pDmaCfg->txInterruptPerMSec);
            pacingBitMask |= CPSW_INT_CONTROL_INT_PACE_EN_C0_TX;
        }
        if ((pDmaCfg->rxInterruptPerMSec != 0U) || (pDmaCfg->txInterruptPerMSec != 0U))
        {
            CSL_CPSW_setWrIntPacingControl(pEnetDmaObj->cpswSsRegs,  pacingBitMask);
            /* int_prescale	Interrupt Counter Prescaler –  The number of  VBUSP_CLK periods in 4us. */
            CSL_CPSW_setWrIntPrescaler(pEnetDmaObj->cpswSsRegs, ENET_CPDMA_CONV_SEC2PRESCALER(EnetSoc_getClkFreq(enetType,0U /* instId */,CPSW_CPPI_CLK)));
        }
        if (pDmaCfg->enHostRxTsFlag)
        {
            CSL_CPSW_enableCpdmaThostTsEncap(pEnetDmaObj->cpdmaRegs);
        }
#if ENET_CFG_IS_ON(CPDMA_CH_OVERRIDE)
        /* Set the thost_ch_override bit if set by application and if soc supports override feature */
        if((pDmaCfg->enChOverrideFlag == true) && (ENET_FEAT_IS_EN(pEnetDmaObj->features, ENET_CPDMA_CHANNEL_OVERRIDE)))
        {
            CSL_CPSW_enableCpdmaChOverride(pEnetDmaObj->cpdmaRegs);
        }
#endif
    }
    return pEnetDmaObj;
}

EnetDma_Handle EnetCpdma_restoreCtxt(Enet_Type enetType,
                            uint32_t instId,
                            uint32_t appCoreId)
{
    EnetCpdma_DrvObj *pEnetDmaObj = NULL;
    uint32_t status;
    uint32_t pacingBitMask = 0;

    /* TODO: error check */
    pEnetDmaObj = EnetSoc_getDmaHandle(enetType, instId);

    Enet_assert(pEnetDmaObj != NULL);

    CSL_CPSW_resetCpdma(pEnetDmaObj->cpdmaRegs);
    status = CSL_CPSW_isCpdmaResetDone(pEnetDmaObj->cpdmaRegs);
    while (status == ((uint32_t)false))
    {
        status = CSL_CPSW_isCpdmaResetDone(pEnetDmaObj->cpdmaRegs);
    }

    CSL_CPSW_setCpdmaRxBufOffset(pEnetDmaObj->cpdmaRegs,
            pEnetDmaObj->rxBufOffset);

    /* TODO: why channel 0 only? Assume one channel for now */
    CSL_CPSW_disableCpdmaTxInt(pEnetDmaObj->cpdmaRegs, 0);
    CSL_CPSW_disableCpdmaRxInt(pEnetDmaObj->cpdmaRegs, 0);

    /* Acknowledge receive and transmit interrupts for proper interrupt pulsing */
    CSL_CPSW_setCpdmaTxEndOfIntVector(pEnetDmaObj->cpdmaRegs, pEnetDmaObj->cpdmaCoreId);
    CSL_CPSW_setCpdmaRxEndOfIntVector(pEnetDmaObj->cpdmaRegs, pEnetDmaObj->cpdmaCoreId);

    CSL_CPSW_enableCpdmaTx(pEnetDmaObj->cpdmaRegs);
    CSL_CPSW_enableCpdmaRx(pEnetDmaObj->cpdmaRegs);

    /* Enable Miscellaneous interrupts - stats and host error interupt */
    CSL_CPSW_enableCpdmaDmaInt(pEnetDmaObj->cpdmaRegs, CSL_CPDMA_DMA_INTMASK_SET_STAT_INT_MASK_MASK |
                                                       CSL_CPDMA_DMA_INTMASK_SET_HOST_ERR_INT_MASK_MASK);
    /* enable host,stats interrupt in cpsw_ss_s wrapper */
    /* TODO: should we enable CPTS/MDIO interrupt here */
    CSL_CPSW_enableWrMiscInt(pEnetDmaObj->cpswSsRegs, pEnetDmaObj->cpdmaCoreId,
                             CPSW_MISC_INT_MDIO_USERINT_MASK |
                             CPSW_MISC_INT_MDIO_LINKINT_MASK |
                             CPSW_MISC_INT_HOSTERR_MASK |
                             CPSW_MISC_INT_STAT_OVERFLOW_MASK |
                             CPSW_MISC_INT_CPTS_EVENT_MASK);

    if (pEnetDmaObj->rxInterruptPerMSec != 0U)
    {
        /* enable Interrupt Pacing Logic in the Wrapper */
        CSL_CPSW_setWrRxIntPerMSec(pEnetDmaObj->cpswSsRegs, pEnetDmaObj->cpdmaCoreId, pEnetDmaObj->rxInterruptPerMSec);
        pacingBitMask |= CPSW_INT_CONTROL_INT_PACE_EN_C0_RX;
    }
    if (pEnetDmaObj->txInterruptPerMSec != 0U)
    {
        /* enable Interrupt Pacing Logic in the Wrapper */
        CSL_CPSW_setWrTxIntPerMSec(pEnetDmaObj->cpswSsRegs, pEnetDmaObj->cpdmaCoreId, pEnetDmaObj->txInterruptPerMSec);
        pacingBitMask |= CPSW_INT_CONTROL_INT_PACE_EN_C0_TX;
    }
    if ((pEnetDmaObj->rxInterruptPerMSec != 0U) || (pEnetDmaObj->txInterruptPerMSec != 0U))
    {
        CSL_CPSW_setWrIntPacingControl(pEnetDmaObj->cpswSsRegs,  pacingBitMask);
        /* int_prescale Interrupt Counter Prescaler –  The number of  VBUSP_CLK periods in 4us. */
        CSL_CPSW_setWrIntPrescaler(pEnetDmaObj->cpswSsRegs, ENET_CPDMA_CONV_SEC2PRESCALER(EnetSoc_getClkFreq(enetType,0U /* instId */,CPSW_CPPI_CLK)));
    }

#if ENET_CFG_IS_ON(CPDMA_CH_OVERRIDE)
    /* Set the thost_ch_override bit if set by application and if soc supports override feature */
    if((pEnetDmaObj->enChOverrideFlag == true) && (ENET_FEAT_IS_EN(pEnetDmaObj->features, ENET_CPDMA_CHANNEL_OVERRIDE)))
    {
        CSL_CPSW_enableCpdmaChOverride(pEnetDmaObj->cpdmaRegs);
    }
#endif

    EnetCpdma_restoreRxChannel(pEnetDmaObj, pEnetDmaObj->rxCppi[0].chInfo);
    EnetCpdma_enableChannel(pEnetDmaObj, pEnetDmaObj->rxCppi[0].chInfo->chNum, pEnetDmaObj->rxCppi[0].chInfo->chDir, pEnetDmaObj->cpdmaCoreId);
    EnetCpdma_enqueueRx(&pEnetDmaObj->rxCppi[0]);

    EnetCpdma_restoreTxChannel(pEnetDmaObj, pEnetDmaObj->txCppi[0].chInfo);
    EnetCpdma_enableChannel(pEnetDmaObj, pEnetDmaObj->txCppi[0].chInfo->chNum, pEnetDmaObj->txCppi[0].chInfo->chDir, pEnetDmaObj->cpdmaCoreId);
    EnetCpdma_enqueueTx(&pEnetDmaObj->txCppi[0]);

    pEnetDmaObj->initFlag = true;
    pEnetDmaObj->isResetOngoing = false;

    return pEnetDmaObj;
}

int32_t EnetCpdma_close(EnetDma_Handle hEnetDma)
{
    int32_t retVal = ENET_SOK;

    // TODO check whether all Rx & Tx channels are closed
    /* Error check */
    if (hEnetDma == NULL)
    {
        ENETTRACE_ERR("[Enet CPDMA] Enet CPDMA handle is NULL!! \n");
        retVal = ENET_EBADARGS;
    }

    if (ENET_SOK == retVal)
    {
        if (hEnetDma->initFlag == false)
        {
            ENETTRACE_ERR("[Enet CPDMA] DMA is not initialized before close !! \n");
            retVal = ENET_EFAIL;
        }
    }

    if (ENET_SOK == retVal)
    {
        //uint32_t channel;
        uint32_t status;

        #if 0 /* TODO: uninit/disable active channel */
              /* Error condition, the applicatio needs to close both rx/tx channel prior to close */
        /* Close TX Channels */
        for (channel = 0; channel < EnetCpdma_CPDMA_MAX_CHANNELS; channel++)
        {
            EnetCpdma_NetChClose(&hEnetDma->chInfo[ENET_CPDMA_DIR_TX][channel], hEnetDma->cpdmaCoreId);
        }

        /* Close RX Channels */
        for (channel = 0; channel < EnetCpdma_CPDMA_MAX_CHANNELS; channel++)
        {
            EnetCpdma_NetChClose(&hEnetDma->chInfo[ENET_CPDMA_DIR_RX][channel], hEnetDma->cpdmaCoreId);
        }
        #endif

        /* Disable Adapter check interrupts - Disable stats interupt */
        CSL_CPSW_disableCpdmaDmaInt(hEnetDma->cpdmaRegs, CSL_CPDMA_DMA_INTMASK_CLEAR_STAT_INT_MASK_MASK |
                                                         CSL_CPDMA_DMA_INTMASK_CLEAR_HOST_ERR_INT_MASK_MASK);

        /* Disable host,stats interrupt in cpsw_3gss_s wrapper */
        CSL_CPSW_disableWrMiscInt(hEnetDma->cpswSsRegs, hEnetDma->cpdmaCoreId,
                                  CPSW_MISC_INT_MDIO_USERINT_MASK |
                                   CPSW_MISC_INT_MDIO_LINKINT_MASK |
                                  CPSW_MISC_INT_HOSTERR_MASK |
                                  CPSW_MISC_INT_STAT_OVERFLOW_MASK |
                                  CPSW_MISC_INT_CPTS_EVENT_MASK);

        /* soft reset */
        CSL_CPSW_resetCpdma(hEnetDma->cpdmaRegs);
        status = CSL_CPSW_isCpdmaResetDone(hEnetDma->cpdmaRegs);
        while (status == ((uint32_t)false))
        {
            status = CSL_CPSW_isCpdmaResetDone(hEnetDma->cpdmaRegs);
        }
        /* De-initialize driver memory manager */
        EnetCpdma_memMgrDeInit();
        hEnetDma->initFlag = false;
    }

    return retVal;
}

int32_t EnetCpdma_saveCtxt(EnetDma_Handle hEnetDma)
{
    int32_t retVal = ENET_SOK;

    // TODO check whether all Rx & Tx channels are closed
    /* Error check */
    if (hEnetDma == NULL)
    {
        ENETTRACE_ERR("[Enet CPDMA] Enet CPDMA handle is NULL!! \n");
        retVal = ENET_EBADARGS;
    }

    if (ENET_SOK == retVal)
    {
        if (hEnetDma->initFlag == false)
        {
            ENETTRACE_ERR("[Enet CPDMA] DMA is not initialized before save !! \n");
            retVal = ENET_EFAIL;
        }
    }

    if (ENET_SOK == retVal)
    {
        hEnetDma->isResetOngoing = true;
        uint32_t channel;
        uint32_t status;

        /* TODO: uninit/disable active channel */
        /* Error condition, the application needs to close both rx/tx channel prior to close */
        /* Close TX Channels */
        for (channel = 0; channel < hEnetDma->numTxChans; channel++)
        {
            EnetCpdma_netChClose(hEnetDma, hEnetDma->txCppi[channel].chInfo, hEnetDma->cpdmaCoreId);
        }

        /* Close RX Channels */
        for (channel = 0; channel < hEnetDma->numRxChans; channel++)
        {
            EnetCpdma_netChClose(hEnetDma, hEnetDma->rxCppi[channel].chInfo, hEnetDma->cpdmaCoreId);
        }

        /* Disable Adapter check interrupts - Disable stats interupt */
        CSL_CPSW_disableCpdmaDmaInt(hEnetDma->cpdmaRegs, CSL_CPDMA_DMA_INTMASK_CLEAR_STAT_INT_MASK_MASK |
                                                         CSL_CPDMA_DMA_INTMASK_CLEAR_HOST_ERR_INT_MASK_MASK);

        /* Disable host,stats interrupt in cpsw_3gss_s wrapper */
        CSL_CPSW_disableWrMiscInt(hEnetDma->cpswSsRegs, hEnetDma->cpdmaCoreId,
                                  CPSW_MISC_INT_MDIO_USERINT_MASK |
                                  CPSW_MISC_INT_MDIO_LINKINT_MASK |
                                  CPSW_MISC_INT_HOSTERR_MASK |
                                  CPSW_MISC_INT_STAT_OVERFLOW_MASK |
                                  CPSW_MISC_INT_CPTS_EVENT_MASK);

        /* soft reset */
        CSL_CPSW_resetCpdma(hEnetDma->cpdmaRegs);
        status = CSL_CPSW_isCpdmaResetDone(hEnetDma->cpdmaRegs);
        while (status == ((uint32_t)false))
        {
            status = CSL_CPSW_isCpdmaResetDone(hEnetDma->cpdmaRegs);
        }

        hEnetDma->initFlag = false;
    }

    return retVal;
}

void EnetDma_initRxChParams(void *pRxChCfg)
{
    EnetCpdma_OpenRxChPrms *pRxChPrms = (EnetCpdma_OpenRxChPrms *)pRxChCfg;

    pRxChPrms->hEnet = NULL;
    pRxChPrms->chNum = 0U;
    pRxChPrms->numRxPkts = 0U;
    pRxChPrms->notifyCb = NULL;
    pRxChPrms->cbArg   = NULL;

    return;
}

EnetDma_RxChHandle EnetDma_openRxCh(EnetDma_Handle hDma,
                                    const void *pRxChCfg)
{
    int32_t retVal;
    EnetCpdma_RxChObj *pRxCh;
    bool allocChObj = false;
    uint32_t intrKey;
    EnetCpdma_OpenRxChPrms *pRxChPrms = (EnetCpdma_OpenRxChPrms *)pRxChCfg;

    intrKey = EnetOsal_disableAllIntr();

    /* Set to NULL so if error condition we return NULL */
    pRxCh = NULL;

    /* Error check */
    retVal = EnetCpdma_checkRxChParams(pRxChPrms);

    if (ENET_SOK == retVal)
    {
        pRxCh = EnetCpdma_memMgrAllocRxChObj();

        if (pRxCh == NULL)
        {
            ENETTRACE_ERR("[Enet CPDMA] Memory allocation for Rx flow object failed !!\n");
            retVal = ENET_EALLOC;
        }
        else
        {
            allocChObj = true;
            memset(pRxCh, 0U, sizeof(*pRxCh));
            /* Save Flow config */
            pRxCh->rxChPrms = *pRxChPrms;
            pRxCh->hEnetDma = Cpsw_getDmaHandle(pRxChPrms->hEnet);

            /* TODO range check and derive channel if any */
            pRxCh->chInfo.chNum = pRxChPrms->chNum;
            pRxCh->chInfo.numBD = (pRxChPrms->numRxPkts * ENET_CPDMA_NUM_DESC_PER_RXPKT);
            pRxCh->chInfo.chDir = ENET_CPDMA_DIR_RX;
            //pRxCh->chInfo.bufSize = pRxChPrms->rxChMtu;
            pRxCh->chInfo.notifyCb = pRxChPrms->notifyCb;
            pRxCh->chInfo.hCbArg = pRxChPrms->cbArg;

            retVal = EnetCpdma_netChOpen(pRxCh->hEnetDma, &pRxCh->chInfo, hDma->cpdmaCoreId);

            if( ENET_SOK != retVal)
            {
                EnetCpdma_memMgrFreeRxChObj (pRxCh);
                pRxCh = NULL;
                ENETTRACE_ERR("[Enet CPDMA] NetChOpen(Tx) failed !!: 0x%x\n", retVal);
            }
        }
    }

    if (ENET_SOK == retVal)
    {
        EnetCpdma_DescCh *pRxDescCh = &pRxCh->hEnetDma->rxCppi[pRxCh->chInfo.chNum];
        /* Initialize the Rx queues.
         * Must be done before enabling CQ events */
        EnetQueue_initQ(&pRxDescCh->waitQueue);
        EnetQueue_initQ(&pRxDescCh->descQueue);
        EnetQueue_initQ(&pRxDescCh->freeQueue);
    }

    if (ENET_SOK == retVal)
    {
        pRxCh->initFlag    = true;
    }
    else
    {
        /* Free the rxChObj last */
        if (allocChObj)
        {
            EnetCpdma_memMgrFreeRxChObj(pRxCh);
        }

        /* As flow open is failed return NULL */
        pRxCh = NULL;
    }

    EnetOsal_restoreAllIntr(intrKey);

    return pRxCh;
}

int32_t EnetDma_closeRxCh(EnetDma_RxChHandle hRxCh,
                          EnetDma_PktQ *fq,
                          EnetDma_PktQ *cq)
{
    int32_t retVal = ENET_SOK;
    uint32_t intrKey;

#if ENET_CFG_IS_ON(DEV_ERROR)
    if ((NULL == hRxCh) ||
        (NULL == fq) ||
        (NULL == cq))
    {
        ENETTRACE_ERR_IF((NULL == hRxCh), "[Enet CPDMA] hRxCh is NULL!!\n");
        ENETTRACE_ERR_IF((NULL == fq), "[Enet CPDMA] fq is NULL!!\n");
        ENETTRACE_ERR_IF((NULL == cq), "[Enet CPDMA] cq is NULL!!\n");
        Enet_assert(false);
        retVal = ENET_EBADARGS;
    }
    else
#endif
    {
        intrKey = EnetOsal_disableAllIntr();

        /* Error check */
        if (hRxCh == NULL)
        {
            retVal = ENET_EBADARGS;
        }

        if (ENET_SOK == retVal)
        {
            EnetCpdma_DescCh *pRxDescCh = &hRxCh->hEnetDma->rxCppi[hRxCh->chInfo.chNum];

            /* TODO: trigger the final rx Operation */

            Enet_assert(hRxCh->hEnetDma != NULL);
            retVal = EnetCpdma_netChClose(hRxCh->hEnetDma, &hRxCh->chInfo, hRxCh->hEnetDma->cpdmaCoreId);

            if( ENET_SOK != retVal)
            {
                ENETTRACE_ERR("[Enet CPDMA] NetChClose(Rx) failed !!: 0x%x\n", retVal);
            }
            else
            {
                hRxCh->initFlag = false;
                EnetQueue_append(fq, &pRxDescCh->freeQueue);
                EnetQueue_append(cq, &pRxDescCh->waitQueue);
                /* Free Rx channel driver object memory */
                EnetCpdma_memMgrFreeRxChObj(hRxCh);
            }
        }

        EnetOsal_restoreAllIntr(intrKey);
    }
    return retVal;
}

int32_t EnetDma_enableRxEvent(EnetDma_RxChHandle hRxCh)
{
    return ENET_SOK;
}

int32_t EnetDma_disableRxEvent(EnetDma_RxChHandle hRxCh)
{
    return ENET_SOK;
}

void EnetDma_initTxChParams(void *pTxChCfg)
{
    EnetCpdma_OpenTxChPrms *pTxChPrms = (EnetCpdma_OpenTxChPrms *)pTxChCfg;

    pTxChPrms->hEnet = NULL;
    pTxChPrms->chNum = 0U;
    pTxChPrms->numTxPkts = 0U;
    pTxChPrms->notifyCb = NULL;
    pTxChPrms->cbArg   = NULL;
    return;
}

EnetDma_TxChHandle EnetDma_openTxCh(EnetDma_Handle hDma,
                                    const void *pTxChCfg)
{
    int32_t retVal         = ENET_SOK;
    EnetCpdma_TxChObj *pTxCh = NULL;
    uint32_t intrKey;
    EnetCpdma_OpenTxChPrms *pTxChPrms = (EnetCpdma_OpenTxChPrms *)pTxChCfg;

    intrKey = EnetOsal_disableAllIntr();

    /* Set to NULL so if error condition we return NULL */
    pTxCh = NULL;

    /* Error check */
    retVal = EnetCpdma_checkTxChParams(pTxChPrms);

    if (ENET_SOK == retVal)
    {
        /* allocate Tx channel object */
        pTxCh = EnetCpdma_memMgrAllocTxChObj();
        if (pTxCh == NULL)
        {
            ENETTRACE_ERR("[Enet CPDMA] Memory allocation for TX channel object failed !!\n");
            retVal = ENET_EALLOC;
        }
        else
        {
            memset(pTxCh, 0U, sizeof(*pTxCh));
            /* Save channel config */
            pTxCh->txChPrms = *pTxChPrms;
            pTxCh->hEnetDma = Cpsw_getDmaHandle(pTxChPrms->hEnet);
            /* TODO range check and derive channel if any */
            pTxCh->chInfo.chNum = pTxChPrms->chNum;
            pTxCh->chInfo.numBD = (pTxChPrms->numTxPkts * ENET_CPDMA_NUM_DESC_PER_TXPKT); /* !TODO add check for chksum offload */
            pTxCh->chInfo.chDir = ENET_CPDMA_DIR_TX;
            //pTxCh->chInfo.bufSize = 0U;
            pTxCh->chInfo.notifyCb = pTxChPrms->notifyCb;
            pTxCh->chInfo.hCbArg = pTxChPrms->cbArg;
            retVal = EnetCpdma_netChOpen(pTxCh->hEnetDma, &pTxCh->chInfo, hDma->cpdmaCoreId);

            if( ENET_SOK != retVal)
            {
                EnetCpdma_memMgrFreeTxChObj (pTxCh);
                pTxCh = NULL;
                ENETTRACE_ERR("[Enet CPDMA] NetChOpen(Tx) failed !!: 0x%x\n", retVal);
            }
            else
            {
                EnetCpdma_DescCh *pTxDescCh = &pTxCh->hEnetDma->txCppi[pTxCh->chInfo.chNum];
                /* Initialize the Tx queues.
                * Must be done before enabling CQ events */
                EnetQueue_initQ(&pTxDescCh->waitQueue);
                EnetQueue_initQ(&pTxDescCh->descQueue);
                EnetQueue_initQ(&pTxDescCh->freeQueue);
                pTxCh->initFlag = true;
            }
        }
    }

    EnetOsal_restoreAllIntr(intrKey);

    return pTxCh;

}

int32_t EnetDma_closeTxCh(EnetDma_TxChHandle hTxCh,
                          EnetDma_PktQ *fq,
                          EnetDma_PktQ *cq)
{
    int32_t retVal = ENET_SOK;
    uint32_t intrKey;

#if ENET_CFG_IS_ON(DEV_ERROR)
    if ((NULL == hTxCh) ||
        (NULL == fq) ||
        (NULL == cq))
    {
        ENETTRACE_ERR_IF((NULL == hTxCh), "[Enet CPDMA] hTxCh is NULL!!\n");
        ENETTRACE_ERR_IF((NULL == fq), "[Enet CPDMA] fq is NULL!!\n");
        ENETTRACE_ERR_IF((NULL == cq), "[Enet CPDMA] cq is NULL!!\n");
        Enet_assert(false);
        retVal = ENET_EBADARGS;
    }
    else
#endif
    {
        /* TODO: intrrupt should be enabled for teradown  */
        intrKey = EnetOsal_disableAllIntr();

        /* Error check */
        if (hTxCh == NULL)
        {
            retVal = ENET_EBADARGS;
        }

        if (ENET_SOK == retVal)
        {
            if (!hTxCh->initFlag)
            {
                retVal = ENET_EFAIL;
            }
        }

        if (ENET_SOK == retVal)
        {
            Enet_assert(hTxCh->hEnetDma != NULL);
            retVal = EnetCpdma_netChClose(hTxCh->hEnetDma, &hTxCh->chInfo, hTxCh->hEnetDma->cpdmaCoreId);

            if( ENET_SOK != retVal)
            {
                ENETTRACE_ERR("[Enet CPDMA] NetChClose(Tx) failed !!: 0x%x\n", retVal);
            }
            else
            {
                /* TODO: range check: Assert */
                EnetCpdma_DescCh *pTxDescCh = &hTxCh->hEnetDma->txCppi[hTxCh->chInfo.chNum];
                hTxCh->initFlag = false;
                EnetQueue_append(fq, &pTxDescCh->waitQueue);
                EnetQueue_append(cq, &pTxDescCh->freeQueue);
                EnetCpdma_memMgrFreeTxChObj(hTxCh);
            }
        }

        EnetOsal_restoreAllIntr(intrKey);
    }

    return retVal;
}

int32_t EnetDma_enableTxEvent(EnetDma_TxChHandle hTxCh)
{
    return ENET_SOK;
}

int32_t EnetDma_disableTxEvent(EnetDma_TxChHandle hTxCh)
{
    return ENET_SOK;
}

int32_t EnetDma_retrieveRxPktQ(EnetDma_RxChHandle hRxCh,
                               EnetDma_PktQ *pRetrieveQ)
{
    int32_t retVal = ENET_SOK;
    EnetCpdma_DescCh *pDescCh;
    uint32_t key;

#if ENET_CFG_IS_ON(DEV_ERROR)
    if ((NULL == hRxCh) ||
        (NULL == pRetrieveQ))
    {
        ENETTRACE_ERR_IF((NULL == hRxCh), "[Enet CPDMA] hRxCh is NULL!!\n");
        ENETTRACE_ERR_IF((NULL == pRetrieveQ), "[Enet CPDMA] pRetrieveQ is NULL!!\n");
        Enet_assert(false);
        retVal = ENET_EBADARGS;
    }
    else
#endif
    {
#if defined(ENETDMA_INSTRUMENTATION_ENABLED)
        uint32_t startTime, diffTime;
        uint32_t pktCnt;
        startTime = EnetOsal_timerRead();
#endif

        pDescCh = &hRxCh->hEnetDma->rxCppi[hRxCh->chInfo.chNum];
        EnetQueue_initQ(pRetrieveQ);

        if (ENET_SOK == retVal)
        {
            key = EnetOsal_disableAllIntr();
            if(hRxCh->hEnetDma->isResetOngoing == false)
            {
                EnetCpdma_dequeueRx(pDescCh, NULL);
                EnetQueue_append(pRetrieveQ, &pDescCh->waitQueue);
                EnetQueue_initQ(&pDescCh->waitQueue);
            }
            EnetOsal_restoreAllIntr(key);
        }

#if defined(ENETDMA_INSTRUMENTATION_ENABLED)
        pktCnt = EnetQueue_getQCount(pRetrieveQ);
        EnetCpdmaStats_addCnt(&hRxCh->stats.rxRetrievePktDeq, pktCnt);
        diffTime = EnetOsal_timerGetDiff(startTime);
        EnetCpdmaStats_updateNotifyStats(&hRxCh->stats.retrievePktStats, pktCnt, diffTime);
#endif
    }

    return retVal;
}

int32_t EnetDma_retrieveRxPkt(EnetDma_RxChHandle hRxCh,
                              EnetDma_Pkt **ppPkt)
{
    int32_t retVal = ENET_SOK;
    EnetCpdma_DescCh *pDescCh;
    uint32_t key;

#if ENET_CFG_IS_ON(DEV_ERROR)
    if ((NULL == hRxCh) ||
        (NULL == ppPkt))
    {
        ENETTRACE_ERR_IF((NULL == hRxCh), "[Enet CPDMA] hRxCh is NULL!!\n");
        ENETTRACE_ERR_IF((NULL == ppPkt), "[Enet CPDMA] ppPkt is NULL!!\n");
        Enet_assert(false);
        retVal = ENET_EBADARGS;
    }
    else
#endif
    {
#if defined(ENETDMA_INSTRUMENTATION_ENABLED)
        uint32_t startTime, diffTime;
        startTime = EnetOsal_timerRead();
#endif

        pDescCh = &hRxCh->hEnetDma->rxCppi[hRxCh->chInfo.chNum];

        if (ENET_SOK == retVal)
        {
            key = EnetOsal_disableAllIntr();
            EnetCpdma_dequeueRx(pDescCh, NULL);
            *ppPkt = (EnetCpdma_PktInfo *)EnetQueue_deq(&pDescCh->waitQueue);
            EnetOsal_restoreAllIntr(key);
        }

#if defined(ENETDMA_INSTRUMENTATION_ENABLED)
        if(ppPkt != NULL)
        {
            EnetCpdmaStats_addCnt(&hRxCh->stats.rxRetrievePktDeq, 1U);
            diffTime = EnetOsal_timerGetDiff(startTime);
            EnetCpdmaStats_updateNotifyStats(&hRxCh->stats.retrievePktStats, 1U, diffTime);
        }
#endif
    }

    return retVal;
}

int32_t EnetDma_submitRxPktQ(EnetDma_RxChHandle hRxCh,
                             EnetDma_PktQ *pSubmitQ)
{
    int32_t retVal = ENET_SOK;
    EnetCpdma_DescCh *pDescCh;

#if ENET_CFG_IS_ON(DEV_ERROR)
    if ((NULL == hRxCh) ||
        (NULL == pSubmitQ))
    {
        ENETTRACE_ERR_IF((NULL == hRxCh), "[Enet CPDMA] hRxCh is NULL!!\n");
        ENETTRACE_ERR_IF((NULL == pSubmitQ), "[Enet CPDMA] pSubmitQ is NULL!!\n");
        Enet_assert(false);
        retVal = ENET_EBADARGS;
    }
    else
#endif
    {
#if defined(ENETDMA_INSTRUMENTATION_ENABLED)
        uint32_t startTime, diffTime;
        uint32_t pktCnt;
        startTime = EnetOsal_timerRead();
        pktCnt    = EnetQueue_getQCount(pSubmitQ);
#endif

        pDescCh = &hRxCh->hEnetDma->rxCppi[hRxCh->chInfo.chNum];

        /* Enqueue descs to fqRing regardless of caller's queue state */
        if (EnetQueue_getQCount(pSubmitQ) > 0U)
        {
            EnetQueue_append(&pDescCh->freeQueue, pSubmitQ);
            EnetQueue_initQ(pSubmitQ);
            if(hRxCh->hEnetDma->isResetOngoing == false)
            {
                EnetCpdma_enqueueRx(pDescCh);
            }
        }

#if defined(ENETDMA_INSTRUMENTATION_ENABLED)
        EnetCpdmaStats_addCnt(&hRxCh->stats.rxSubmitPktEnq, pktCnt);
        diffTime = EnetOsal_timerGetDiff(startTime);
        EnetCpdmaStats_updateNotifyStats(&hRxCh->stats.submitPktStats, pktCnt, diffTime);
#endif
    }

    return retVal;
}

int32_t EnetDma_submitRxPkt(EnetDma_RxChHandle hRxCh,
                            EnetDma_Pkt *pPkt)
{
    int32_t retVal = ENET_SOK;
    EnetCpdma_DescCh *pDescCh;

#if ENET_CFG_IS_ON(DEV_ERROR)
    if ((NULL == hRxCh) ||
        (NULL == pPkt))
    {
        ENETTRACE_ERR_IF((NULL == hRxCh), "[Enet CPDMA] hRxCh is NULL!!\n");
        ENETTRACE_ERR_IF((NULL == pPkt), "[Enet CPDMA] pPkt is NULL!!\n");
        Enet_assert(false);
        retVal = ENET_EBADARGS;
    }
    else
#endif
    {
#if defined(ENETDMA_INSTRUMENTATION_ENABLED)
        uint32_t startTime, diffTime;
        startTime = EnetOsal_timerRead();
#endif

        pDescCh = &hRxCh->hEnetDma->rxCppi[hRxCh->chInfo.chNum];

        /* Enqueue descs to fqRing regardless of caller's queue state */
        EnetQueue_enq(&pDescCh->freeQueue, &pPkt->node);
        EnetCpdma_enqueueRx(pDescCh);

#if defined(ENETDMA_INSTRUMENTATION_ENABLED)
        EnetCpdmaStats_addCnt(&hRxCh->stats.rxSubmitPktEnq, 1U);
        diffTime = EnetOsal_timerGetDiff(startTime);
        EnetCpdmaStats_updateNotifyStats(&hRxCh->stats.submitPktStats, 1U, diffTime);
#endif
    }

    return retVal;
}

int32_t EnetDma_retrieveTxPktQ(EnetDma_TxChHandle hTxCh,
                                   EnetDma_PktQ *pRetrieveQ)
{
    int32_t retVal = ENET_SOK;
    EnetCpdma_DescCh *pDescCh;
    uint32_t key;

#if ENET_CFG_IS_ON(DEV_ERROR)
    if ((NULL == hTxCh) ||
        (NULL == pRetrieveQ))
    {
        ENETTRACE_ERR_IF((NULL == hTxCh), "[Enet CPDMA] hTxCh is NULL!!\n");
        ENETTRACE_ERR_IF((NULL == pRetrieveQ), "[Enet CPDMA] pRetrieveQ is NULL!!\n");
        Enet_assert(false);
        retVal = ENET_EBADARGS;
    }
    else
#endif
    {
#if defined(ENETDMA_INSTRUMENTATION_ENABLED)
        uint32_t startTime, diffTime;
        uint32_t pktCnt;
        startTime = EnetOsal_timerRead();
#endif

        pDescCh = &hTxCh->hEnetDma->txCppi[hTxCh->chInfo.chNum];

        EnetQueue_initQ(pRetrieveQ);
        key = EnetOsal_disableAllIntr();
        if(hTxCh->hEnetDma->isResetOngoing == false)
        {
            EnetCpdma_dequeueTx(pDescCh, NULL);
            EnetQueue_append(pRetrieveQ, &pDescCh->freeQueue);
            EnetQueue_initQ(&pDescCh->freeQueue);
        }
        EnetOsal_restoreAllIntr(key);

#if defined(ENETDMA_INSTRUMENTATION_ENABLED)
        pktCnt = EnetQueue_getQCount(pRetrieveQ);
        EnetCpdmaStats_addCnt(&hTxCh->stats.txRetrievePktDeq, pktCnt);
        diffTime = EnetOsal_timerGetDiff(startTime);
        EnetCpdmaStats_updateNotifyStats(&hTxCh->stats.retrievePktStats, pktCnt, diffTime);
#endif
    }

    return retVal;
}

int32_t EnetDma_retrieveTxPkt(EnetDma_TxChHandle hTxCh,
                              EnetDma_Pkt **ppPkt)
{
    int32_t retVal = ENET_SOK;
    EnetCpdma_DescCh *pDescCh;
    uint32_t key;

#if ENET_CFG_IS_ON(DEV_ERROR)
    if ((NULL == hTxCh) ||
        (NULL == ppPkt))
    {
        ENETTRACE_ERR_IF((NULL == hTxCh), "[Enet CPDMA] hTxCh is NULL!!\n");
        ENETTRACE_ERR_IF((NULL == ppPkt), "[Enet CPDMA] ppPkt is NULL!!\n");
        Enet_assert(false);
        retVal = ENET_EBADARGS;
    }
    else
#endif
    {
#if defined(ENETDMA_INSTRUMENTATION_ENABLED)
        uint32_t startTime, diffTime;
        startTime = EnetOsal_timerRead();
#endif

        pDescCh = &hTxCh->hEnetDma->txCppi[hTxCh->chInfo.chNum];

        key = EnetOsal_disableAllIntr();
        EnetCpdma_dequeueTx(pDescCh, NULL);
        *ppPkt = (EnetDma_Pkt *)EnetQueue_deq(&pDescCh->freeQueue);
        EnetOsal_restoreAllIntr(key);

#if defined(ENETDMA_INSTRUMENTATION_ENABLED)
        if(NULL != *ppPkt)
        {
            EnetCpdmaStats_addCnt(&hTxCh->stats.txRetrievePktDeq, 1U);
            diffTime = EnetOsal_timerGetDiff(startTime);
            EnetCpdmaStats_updateNotifyStats(&hTxCh->stats.retrievePktStats, 1U, diffTime);
        }
#endif
    }

    return retVal;
}

int32_t EnetDma_submitTxPktQ(EnetDma_TxChHandle hTxCh,
                             EnetDma_PktQ *pSubmitQ)
{
    int32_t retVal = ENET_SOK;
    EnetCpdma_DescCh *pDescCh;
    uint32_t key;

#if ENET_CFG_IS_ON(DEV_ERROR)
    if ((NULL == hTxCh) ||
        (NULL == pSubmitQ))
    {
        ENETTRACE_ERR_IF((NULL == hTxCh), "[Enet CPDMA] hTxCh is NULL!!\n");
        ENETTRACE_ERR_IF((NULL == pSubmitQ), "[Enet CPDMA] pSubmitQ is NULL!!\n");
        Enet_assert(false);
        retVal = ENET_EBADARGS;
    }
    else
#endif
    {
#if defined(ENETDMA_INSTRUMENTATION_ENABLED)
        uint32_t startTime, diffTime;
        uint32_t pktCnt;
        startTime = EnetOsal_timerRead();
        pktCnt    = EnetQueue_getQCount(pSubmitQ);
#endif

        /* Enqueue descs to fqRing regardless of caller's queue state */
        if (EnetQueue_getQCount(pSubmitQ) > 0U)
        {
            pDescCh = &hTxCh->hEnetDma->txCppi[hTxCh->chInfo.chNum];
            key = EnetOsal_disableAllIntr();
            EnetQueue_append(&pDescCh->waitQueue, pSubmitQ);
            EnetQueue_initQ(pSubmitQ);
            if(hTxCh->hEnetDma->isResetOngoing == false)
            {
                EnetCpdma_enqueueTx(pDescCh);
            }
            EnetOsal_restoreAllIntr(key);
        }

#if defined(ENETDMA_INSTRUMENTATION_ENABLED)
        EnetCpdmaStats_addCnt(&hTxCh->stats.txSubmitPktEnq, pktCnt);
        diffTime = EnetOsal_timerGetDiff(startTime);
        EnetCpdmaStats_updateNotifyStats(&hTxCh->stats.submitPktStats, pktCnt, diffTime);
#endif
    }

    return retVal;
}

int32_t EnetDma_submitTxPkt(EnetDma_TxChHandle hTxCh,
                                 EnetDma_Pkt *pPkt)
{
    int32_t retVal = ENET_SOK;
    EnetCpdma_DescCh *pDescCh;
    uint32_t key;

#if ENET_CFG_IS_ON(DEV_ERROR)
    if ((NULL == hTxCh) ||
        (NULL == pPkt))
    {
        ENETTRACE_ERR_IF((NULL == hTxCh), "[Enet CPDMA] hTxCh is NULL!!\n");
        ENETTRACE_ERR_IF((NULL == pPkt), "[Enet CPDMA] pPkt is NULL!!\n");
        Enet_assert(false);
        retVal = ENET_EBADARGS;
    }
    else
#endif
    {
#if defined(ENETDMA_INSTRUMENTATION_ENABLED)
        uint32_t startTime, diffTime;
        startTime = EnetOsal_timerRead();
#endif

        /* Enqueue descs to fqRing regardless of caller's queue state */
        pDescCh = &hTxCh->hEnetDma->txCppi[hTxCh->chInfo.chNum];
        key = EnetOsal_disableAllIntr();
        EnetQueue_enq(&pDescCh->waitQueue, &pPkt->node);
        EnetCpdma_enqueueTx(pDescCh);
        EnetOsal_restoreAllIntr(key);

#if defined(ENETDMA_INSTRUMENTATION_ENABLED)
        EnetCpdmaStats_addCnt(&hTxCh->stats.txSubmitPktEnq, 1U);
        diffTime = EnetOsal_timerGetDiff(startTime);
        EnetCpdmaStats_updateNotifyStats(&hTxCh->stats.submitPktStats, 1U, diffTime);
#endif
    }

    return retVal;
}

void EnetDma_initPktInfo(EnetDma_Pkt *pktInfo)
{
    uint32_t i;

    memset(&pktInfo->node, 0U, sizeof(pktInfo->node));
    for (i = 0; i < ENET_CPDMA_CPSW_MAX_SG_LIST; i++)
    {
        pktInfo->sgList.list[i].bufPtr = NULL;
        pktInfo->sgList.list[i].origBufPtr = NULL;
        pktInfo->sgList.list[i].segmentFilledLen = 0U;
        pktInfo->sgList.list[i].segmentAllocLen = 0U;
        pktInfo->sgList.list[i].disableCacheOps = false;
    }
    pktInfo->chkSumInfo            = 0U;
    pktInfo->appPriv               = NULL;
    pktInfo->tsInfo.enableHostTxTs = false;
    pktInfo->txPortNum             = ENET_MAC_PORT_INV;
    pktInfo->sgList.numScatterSegments = 0U;
    ENET_UTILS_SET_PKT_DRIVER_STATE(&pktInfo->pktState,
                                    (uint32_t)ENET_PKTSTATE_DMA_NOT_WITH_HW);
    return;
}

int32_t EnetDma_getRxChStats(EnetDma_RxChHandle hRxCh,
                             EnetDma_RxChStats *pStats)
{
    int32_t retVal = ENET_ENOTSUPPORTED;

#if defined(ENETDMA_INSTRUMENTATION_ENABLED)
    *pStats = hRxCh->stats;
    retVal = ENET_SOK;
#endif

    return retVal;
}

int32_t EnetDma_getTxChStats(EnetDma_TxChHandle hTxCh,
                             EnetDma_TxChStats *pStats)
{
    int32_t retVal = ENET_ENOTSUPPORTED;

#if defined(ENETDMA_INSTRUMENTATION_ENABLED)
    *pStats = hTxCh->stats;
    retVal = ENET_SOK;
#endif

    return retVal;
}

int32_t EnetDma_resetRxChStats(EnetDma_RxChHandle hRxCh)
{
    int32_t retVal = ENET_ENOTSUPPORTED;

#if defined(ENETDMA_INSTRUMENTATION_ENABLED)
    memset(&hRxCh->stats, 0, sizeof(hRxCh->stats));
    retVal = ENET_SOK;
#endif

    return retVal;
}

int32_t EnetDma_resetTxChStats(EnetDma_TxChHandle hTxCh)
{
    int32_t retVal = ENET_ENOTSUPPORTED;

#if defined(ENETDMA_INSTRUMENTATION_ENABLED)
    memset(&hTxCh->stats, 0, sizeof(hTxCh->stats));
    retVal = ENET_SOK;
#endif

    return retVal;
}

int32_t EnetDma_registerRxEventCb(EnetDma_RxChHandle hRxCh, EnetDma_PktNotifyCb notifyCb, void *cbArg)
{
    uintptr_t             key;
    int32_t status        = ENET_SOK;

    key = EnetOsal_disableAllIntr();

    hRxCh->chInfo.hCbArg = cbArg;
    hRxCh->chInfo.notifyCb = notifyCb;

    EnetOsal_restoreAllIntr(key);
    return status;
}

int32_t EnetDma_registerTxEventCb(EnetDma_TxChHandle hTxCh, EnetDma_PktNotifyCb notifyCb, void *cbArg)
{
    uintptr_t             key;
    int32_t status        = ENET_SOK;

    key = EnetOsal_disableAllIntr();

    hTxCh->chInfo.hCbArg = cbArg;
    hTxCh->chInfo.notifyCb = notifyCb;

    EnetOsal_restoreAllIntr(key);
    return status;
}

#if 0 /* TODO */

void EnetCpdma_initDataPathParams(EnetDma_initCfg *pDmaConfig)
{
    return;
}

EnetDma_Handle EnetCpdma_initDataPath(Enet_Type enetType,
                                      uint32_t instId,
                                      const EnetDma_initCfg *pDmaConfig)
{
    EnetCpdma_Cfg cfg;
    EnetDma_Handle hDmaHandle;

    EnetDma_initCfg(enetType, &cfg);
    hDmaHandle   = EnetCpdma_open(enetType, instId, &cfg);
    return hDmaHandle;
}

int32_t EnetCpdma_deInitDataPath(EnetDma_Handle hEnetDma)
{
    int32_t status;

    status = EnetCpdma_close(hEnetDma);
    return status;
}

#endif

/* End of file */
