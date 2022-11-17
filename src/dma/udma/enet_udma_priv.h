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
 *  \file enet_udma_priv.h
 *
 *  \brief Enet UDMA private header file.
 */

#ifndef ENET_UDMA_PRIV_H_
#define ENET_UDMA_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <include/core/enet_dma.h>
#include <drivers/udma/soc/udma_soc.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief UDMAP descriptor type for CPSW channels */
#define ENET_UDMA_UDMAP_DESC_TYPE     (uint32_t)CSL_UDMAP_CPPI5_PD_DESCINFO_DTYPE_VAL_HOST

/** \brief UDMA max teardown time (in msecs) */
#define ENET_UDMA_TEARDOWN_TIME_MS   (100U)

/*! \brief Ethernet Jumbo frame size */
#define ENET_UDMA_JUMBO_PACKET_SIZE   (9000U)

/*! \brief UDMA teardown CQ ring memory size */
#define ENET_UDMA_TDCQ_RING_ELE_CNT   (1U)


/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */
/*!
 * \brief CPPI TX (host RX) Info words structure
 * Note - This maps to EPI (extended packet info) block of host descriptor.
 */
typedef struct
{
    /*! TX info word 0 */
    uint32_t word0;

    /*! TX info word 1 */
    uint32_t word1;

    /*! Reserved word */
    uint32_t rsvd;

    /*! TX info word 3 */
    uint32_t word3;
}EnetUdma_CppiTxInfo;

/*!
 * \brief CPPI TX (host RX) Status words structure
 * Note - This maps to PSI (protocol specific info) block of host descriptor.
 */
typedef struct
{
    /*! Lower word of 64-bit timestamp */
    uint32_t tsLow;

    /*! Higher word of 64-bit timestamp */
    uint32_t tsHigh;

    /*! pV4/IpV6 protocol information info status word */
    uint32_t chkSumInfo;

    /*! Reserved */
    uint32_t rsvd;
}EnetUdma_CppiTxStatus;

/*!
 * \brief CPPI RX (host TX) Info words structure
 * Note - This maps to EPI (extended packet info) block of host descriptor.
 */
typedef struct
{
    /*! RX info word 0 */
    uint32_t word0;

    /*! Reserved word */
    uint32_t rsvd1;

    /*! TX info word 2 */
    uint32_t word2;

    /*! Reserved word */
    uint32_t rsvd2;
}EnetUdma_CppiRxInfo;

/*!
 * \brief CPPI RX (host TX) Control words structure
 * Note - This maps to PSI (protocol specific info) block of host descriptor.
 */
typedef struct
{
    /*! Reserved */
    uint32_t rsvd1;

    /*! Timestamp info */
    uint32_t tsInfo;

    /*! IpV4/IpV6 protocol information  */
    uint32_t chkSumInfo;

    /*! Reserved */
    uint32_t rsvd2;
}EnetUdma_CppiRxControl;

/*!
 * \brief DMA transfer direction (rx/tx) enum
 */
typedef enum EnetUdma_Dir_s
{
    /*! DMA RX flow  */
    ENET_UDMA_DIR_RX = 0,

    /*! DMA Tx channel   */
    ENET_UDMA_DIR_TX = 1
} EnetUdma_Dir;

/*!
 * \brief DMA Desc Queue
 *
 * \details
 *      A queue of DMA Descs, which are use for managing the active queue of
 *      of DMA Descs given to the hardware driver.
 */
typedef struct EnetUdma_DmaDescQ_s
{
    /*! Number of DMA Descs in queue */
    uint32_t count;

    /*! Pointer to first DMA desc */
    EnetUdma_DmaDesc *head;

    /*! Pointer to last DMA Descs */
    EnetUdma_DmaDesc *tail;
}
EnetUdma_DmaDescQ;

/*!
 * \brief Structure holding UDMA handles for Enet UDMA Rx flow and TX channel.
 */
typedef struct
{
    /*! UDMA driver handle */
    Udma_DrvHandle hUdmaDrv;

    /*! UDMA Channel handle */
    Udma_ChHandle hUdmaCh;

    /*! UDMA Event handle */
    Udma_EventHandle hUdmaEvt;

#if (UDMA_SOC_CFG_RING_MON_PRESENT == 1)
    /*! UDMA Ring monitor handle (null if ringMon not used)*/
    Udma_RingMonHandle hUdmaRingMon;
#endif
    /*! Whether to use a global event or not */
    bool useGlobalEvt;
} EnetUdma_udmaInfo;

/*!
 * \brief Structure holding Ring allocation info.
 */
typedef struct
{
    /*! Enet Peripheral type */
    Enet_Type enetType;

    /*! Enet Peripheral instance id */
    uint32_t instId;

    /*! UDMA channel number */
    uint32_t mappedChNum;

    /*!  */
    EnetUdma_Dir transferDir;

    /*! Ring number to be used in ring alloc function. Use #UDMA_RING_ANY if use
     *   any ring */
    uint16_t ringNum;

    /*! Flag to allocate ring memory. If set, ringPrms.ringMem can not be NULL */
    bool allocRingMem;

    /*! Allocate ring memory callback function */
    EnetUdma_AllocRingMemFxn ringMemAllocFxn;

    /*! Free ring memory callback function */
    EnetUdma_FreeRingMemFxn ringMemFreeFxn;

    /*! Callback argument for the ring callback functions */
    void *cbArg;
} EnetUdma_ringAllocInfo;


/*!
 * \brief Structure for reserved flow params.
 */
typedef struct EnetUdma_OpenRsvdRxFlowPrms_s
{
    /*! UDMA driver handle*/
    Udma_DrvHandle hUdmaDrv;

    /*! Channel index*/
    uint32_t chIdx;

    /*! Flow start index*/
    uint32_t startIdx;

    /*! Flow index to be opened*/
    uint32_t flowIdx;
} EnetUdma_RsvdRxFlowPrms;

/*!
 * \brief Structure for RX flow memory objects
 */
typedef struct EnetUdma_RxFlowMemObj_s
{
    /*! UDMA Rx flow object */
    Udma_FlowObject flowUdmaObj;

    /*! UDMA event object */
    Udma_EventObject udmaEvtObj;

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
    /*! Free queue ring object */
    Udma_RingObject fqRingObj;
#endif

    /*! This FQ is used for when received packet length is larger than app
     *   allocated packet size (PDK-3723) */
    Udma_RingObject dropFqRingObj;

    /*! Complete queue ring handle */
    Udma_RingHandle cqRing;

    /*! Complete queue ring handle */
    Udma_RingObject cqRingObj;

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
    /*! Udma dedicated proxy object used if rxFlowPrms.useProxy is TRUE */
    Udma_ProxyObj proxyObj;
#endif

    /*! Ready DMA descriptor Q */
    EnetUdma_DmaDescQ rxReadyDmaDescQ;
} EnetUdma_RxFlowMemObj;

typedef struct EnetUdma_RxFlowObj_s
{
    /*! Enet DMA handle */
    EnetDma_Handle hDma;

    /*! Channel init params */
    EnetUdma_OpenRxFlowPrms rxFlowPrms;

    /*! Struct holding RX flow object memories */
    EnetUdma_RxFlowMemObj rxFlowMemObj;

    /*! UDMA driver handle */
    Udma_DrvHandle hUdmaDrv;

    /*! RX flow handle */
    Udma_FlowHandle hUdmaFlow;

    /*! UDMA Event object */
    Udma_EventHandle hUdmaEvt;

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
    /*! Free queue ring object */
    Udma_RingHandle fqRing;
#endif

    /*! Complete queue ring handle */
    Udma_RingHandle cqRing;

    /*! DMA descriptor free pool handle. This is used when auto-reclaim of TX FQ as
     *  RX CQ is enabled to use app passed free pool instead of rxReadyDmaDescQ pool */
    EnetUdma_DmaDescQ *hDmaDescPool;

    /*! Event initialization state */
    bool evtInitFlag;

    /*! Channel initialization state */
    bool initFlag;

    /*! App private */
    void *appPriv;

    /*! Packet queue into which CQ Ring ISR will drain packets into */
    EnetQ cqIsrQ;

#if (UDMA_SOC_CFG_PROXY_PRESENT == 1)
    /*! Udma proxy handle. Will be NULL or pointer to proxyObj based on
     *   useProxy is set or not.
     */
    Udma_ProxyHandle hUdmaProxy;
#endif

#if (UDMA_SOC_CFG_RING_MON_PRESENT == 1)
    /*! Udma ring monitor handle. Will be NULL based if useRingMon is not set.
     */
    Udma_RingMonHandle hUdmaRingMon;
#endif

    /*! Rx flow app packet and DMA desc stats */
    EnetDma_RxChStats stats;


    /*! Whether to use global event for this channel or not. If set to false, a dedicated
     *  event will be used */
    bool useGlobalEvt;
} EnetUdma_RxFlowObj;

/*!
 * \brief Structure for Tx channel memory objects
 */
typedef struct EnetUdma_TxChMemObj_s
{
    /*! UDMA channel object */
    Udma_ChObject udmaChObj;

    /*! UDMA event object */
    Udma_EventObject udmaEvtObj;

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
    /*! Free queue ring object */
    Udma_RingObject fqRingObj;
#endif

    /*! Complete queue ring handle */
    Udma_RingHandle cqRing;


#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
    /*! Complete queue ring handle */
    Udma_RingObject cqRingObj;

    /*! Udma dedicated proxy object used if rxFlowPrms.useProxy is TRUE */
    Udma_ProxyObj proxyObj;
#endif

    /*! Ready DMA descriptor Q */
    EnetUdma_DmaDescQ txFreeDmaDescQ;
} EnetUdma_TxChMemObj;

typedef struct EnetUdma_TxChObj_s
{
    /*! Enet DMA handle */
    EnetDma_Handle hDma;

    /*! Channel init params */
    EnetUdma_OpenTxChPrms txChPrms;

    /*! Struct holding TX Channel object memories */
    EnetUdma_TxChMemObj txChMemObj;

    /*! UDMA driver handle */
    Udma_DrvHandle hUdmaDrv;

    /*! TX channel handle */
    Udma_ChHandle hUdmaCh;

    /*! UDMA Event object */
    Udma_EventHandle hUdmaEvt;

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
    /*! Free queue ring handle */
    Udma_RingHandle fqRing;
#endif

    /*! Teardown complete queue ring handle */
    Udma_RingHandle tdCqRing;

    /*! Complete queue ring handle */
    Udma_RingHandle cqRing;

    /*! DMA descriptor free pool handle. This is used when auto-reclaim of RX FQ as
     *  TX CQ is enabled to use app passed free pool instead of rxReadyDmaDescQ pool */
    EnetUdma_DmaDescQ *hDmaDescPool;

    /*! Event initialization state */
    bool evtInitFlag;

    /*! Channel initialization state */
    bool initFlag;

    /*! App private */
    void *appPriv;

    /*! Packet queue into which CQ Ring ISR will drain packets into */
    EnetQ cqIsrQ;

#if (UDMA_SOC_CFG_PROXY_PRESENT == 1)
    /*! Udma proxy handle. Will be NULL or pointer to proxyObj based on
     *   useProxy is set or not.
     */
    Udma_ProxyHandle hUdmaProxy;
#endif

#if (UDMA_SOC_CFG_RING_MON_PRESENT == 1)
    /*! Udma ring monitor handle. Will be NULL based if useRingMon is not set.
     */
    Udma_RingMonHandle hUdmaRingMon;
#endif

    /*! Tx channel app packet and DMA desc stats */
    EnetDma_TxChStats stats;

    /*! Whether to use global event for this channel or not. If set to false, a dedicated
     *  event will be used */
    bool useGlobalEvt;
} EnetUdma_TxChObj;

typedef struct EnetUdma_RxChObj_s
{
    /*! Enet Peripheral type */
    Enet_Type enetType;

    /*! Enet Peripheral instance id */
    uint32_t instId;

    /*! Rx Channel Index (always 1 in dual MAC and CPSW case)*/
    uint32_t chIdx;

    /*! Channel init params */
    EnetUdma_RxChInitPrms rxChInitPrms;

    /*! UDMA channel object */
    Udma_ChObject udmaChObj;

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
    /*! UDMA Rx flow object - as we don't use default flow and allocate new using
     *  Udma_flowAlloc, we need to allocate flow object to our RX channel */
    Udma_FlowObject flowUdmaObj;

    /*! Teardown complete queue ring handle */
    Udma_RingHandle tdCqRing;
#endif

    /*! Channel initialization state */
    bool initFlag;
} EnetUdma_RxChObj;

/**
 *  \brief DMA module
 *
 *  This is an internal/private driver structure and should not be used
 *  or modified by the caller.
 */
typedef struct EnetUdma_DrvObj_s
{
    /*! Enet peripheral which this DMA module is used for */
    EnetPer_Handle hPer;

    /*! Enet peripheral type */
    Enet_Type enetType;

    /*! Enet peripheral instance id */
    uint32_t instId;

    /*! UDMA driver handle*/
    Udma_DrvHandle hUdmaDrv;

    /*! Module initialization state */
    bool initFlag;

    /*! Receive channel Obj */
    EnetUdma_RxChObj rxChObj[ENET_UDMA_NUM_RXCHAN_MAX];

    /*! Number of RX channels */
    uint32_t numRxCh;
} EnetUdma_DrvObj;

/**
 *  \brief DMA Object Memory Info
 *
 *  This structure is used to get Udma Driver handle from Application
 */
typedef struct EnetUdma_DrvObjMemInfo_s{

    /*! Number of Udma Instance */
    uint32_t numObjs;

    /*! Pointer to EnetUdma_DrvObj */
    EnetUdma_DrvObj *drvObjMem;

}EnetUdma_DrvObjMemInfo;

/* ========================================================================== */
/*                         Global Variables                                   */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void EnetUdma_initCfg(Enet_Type enetType, void *pDmaConfig);

EnetDma_Handle EnetUdma_open(Enet_Type enetType,
                             uint32_t instId,
                             const EnetUdma_Cfg *pEnetUdmaCfg);

int32_t EnetUdma_close(EnetDma_Handle hCpswDma);

uint32_t EnetUdma_getRxChFlowStartIdx(EnetDma_Handle hCpswDma,
                                      uint32_t chIdx);

uint32_t EnetUdma_getRxFlowCnt(EnetDma_Handle hCpswDma,
                               uint32_t chIdx);

/**
 *  \brief Enet UDMA Open function.
 *
 *  This API opens the Enet UDMA RX channel and sets data path module configuration.
 *  This function should be called only by Master core and should be called before
 *  calling any IOCTLs.
 *
 *  \param hCpswDma  [IN] Enet UDMA module handle.
 *  \param rxChInitPrms  [IN] CPSW RX channel init parameters.
 *  \param totalRxFlowCount [IN] Total number of Rx flows to be allocated to the
 *                               channel
 *
 *  \return \ref Enet_ErrorCodes
 */
int32_t EnetUdma_openRxCh(EnetDma_Handle hCpswDma,
                         const EnetUdma_RxChInitPrms *pRxChInitPrms,
                          uint32_t totalRxFlowCount,
                          uint32_t chIdx);

/**
 *  \brief Enet UDMA close function.
 *
 *  Stop and close the Rx channel and set deinit DMA module configuration.
 *
 *  \param hRxCh  [IN] Handle to Enet UDMA RX channel returned from call to
 *                   EnetUdma_open.
 *  \param chIdx  [IN] 0-relative channel index. Use 0 for single channel.
 *
 *  \return \ref Enet_ErrorCodes
 */
int32_t EnetUdma_closeRxCh(EnetDma_Handle hCpswDma,
                           uint32_t chIdx);

int32_t EnetUdma_registerEvent(EnetUdma_udmaInfo *pUdmaInfo,
                              Udma_RingHandle hUdmaRing,
                              Udma_EventCallback eventCb,
                              void *cbArg,
                              uint32_t eventType);

int32_t EnetUdma_unregisterEvent(Udma_EventHandle hUdmaEvt);

void EnetUdma_getRxFlowUdmaInfo(EnetDma_RxChHandle hRxFlow,
                               EnetUdma_udmaInfo *pUdmaInfo);

void EnetUdma_getTxChUdmaInfo(EnetDma_TxChHandle hTxCh,
                             EnetUdma_udmaInfo *pUdmaInfo);

int32_t EnetUdma_allocRing(Udma_DrvHandle hUdmaDrv,
                            Udma_RingHandle hUdmaRing,
                            Udma_RingPrms *pRingPrms,
                            EnetUdma_ringAllocInfo *pRingAllocInfo);

void EnetUdma_initRxFreeDescQ(EnetUdma_RxFlowObj *pRxFlow);

void EnetUdma_deInitRxFreeDescQ(EnetUdma_RxFlowObj *pRxFlow);

void EnetUdma_initTxFreeDescQ(EnetUdma_TxChObj *pTxChObj);

void EnetUdma_deInitTxFreeDescQ(EnetUdma_TxChObj *pTxChObj);

void EnetUdma_reconfigureDescQ(EnetUdma_TxChObj *pTxCh);

int32_t EnetUdma_ringDequeue(Udma_RingHandle hUdmaRing,
                            EnetUdma_DmaDesc **pDmaDesc,
                            bool disableCacheOpsFlag,
                            EnetUdma_Dir transferDir);

int32_t EnetUdma_ringEnqueue(Udma_RingHandle hUdmaRing,
                            EnetUdma_DmaDesc *pDmaDesc,
                            uint32_t packetSize,
                            bool disableCacheOpsFlag,
                            EnetUdma_Dir transferDir
#if (UDMA_SOC_CFG_PROXY_PRESENT == 1)
                            ,
                            Udma_ProxyHandle hUdmaProxy
#endif
                            );

int32_t EnetUdma_freeTxRing(Udma_RingHandle hUdmaRing,
                           EnetUdma_UdmaRingPrms *pRingPrms,
                           EnetUdma_OpenTxChPrms *pTxChPrms);

int32_t EnetUdma_flushRxFlowRing(EnetDma_RxChHandle hRxFlow,
                                Udma_RingHandle hUdmaRing,
                                EnetDma_PktQ *pFqPktInfoQ);

int32_t EnetUdma_flushTxChRing(EnetDma_TxChHandle hTxCh,
                              Udma_RingHandle hUdmaRing,
                              EnetDma_PktQ *pFqPktInfoQ);

int32_t EnetUdma_freeRing(Udma_RingHandle hUdmaRing,
                           uint32_t numPkts,
                           EnetUdma_FreeRingMemFxn ringMemFreeFxn,
                           void *cbArg);

int32_t EnetUdma_submitPkts(EnetPer_Handle hPer,
                            Udma_RingHandle hUdmaRing,
                           EnetDma_PktQ *pToHwQueue,
                           EnetUdma_DmaDescQ *pDmaDescQ,
                           bool disableCacheOpsFlag,
                           EnetUdma_Dir transferDir
#if (UDMA_SOC_CFG_PROXY_PRESENT == 1)
                           , Udma_ProxyHandle hUdmaProxy
#endif
                           );

int32_t EnetUdma_retrievePkts(EnetPer_Handle hPer,
                              Udma_RingHandle hUdmaRing,
                             EnetDma_PktQ *pFromHwQueue,
                             EnetUdma_DmaDescQ *pDmaDescQ,
                             bool disableCacheOpsFlag,
                             EnetUdma_Dir transferDir);

void EnetUdma_buffDescInit(EnetUdma_CpswHpdDesc *hpdDesc);

void EnetUdma_dmaDescInit(EnetUdma_DmaDesc *pCpswDmaPacket);

void EnetUdma_dmaDescQInit(EnetUdma_DmaDescQ *pDmaDescQ);

uint32_t EnetUdma_dmaDescQCount(EnetUdma_DmaDescQ *pDmaDescQ);

EnetUdma_DmaDesc *EnetUdma_dmaDescDeque(EnetUdma_DmaDescQ *pDmaDescQ);

void EnetUdma_dmaDescCopyQ(EnetUdma_DmaDescQ *pDstQueue,
                          const EnetUdma_DmaDescQ *pSrcQueue);

void EnetUdma_dmaDescEnque(EnetUdma_DmaDescQ *pDmaDescQ,
                          EnetUdma_DmaDesc *pDmaDesc);

int32_t EnetUdma_dmaDescCheck(EnetUdma_DmaDesc *dmaDesc,
                             EnetUdma_Dir transferDir);

EnetDma_RxChHandle EnetUdma_openRxRsvdFlow(EnetDma_Handle hDma,
                                           EnetUdma_RsvdRxFlowPrms *pRxFlowPrms);

int32_t EnetUdma_closeRxRsvdFlow(EnetDma_RxChHandle hRxFlow);

/*! Rx flow object allocation function  */
EnetUdma_RxFlowObj *EnetUdma_memMgrAllocRxFlowObj(void);

/*! Rx flow object free function  */
void EnetUdma_memMgrFreeRxFlowObj(EnetUdma_RxFlowObj *pRxFlowObj);

/*! TX channel object allocation function  */
EnetUdma_TxChObj *EnetUdma_memMgrAllocTxChObj(void);

/*! TX channel object free function  */
void EnetUdma_memMgrFreeTxChObj(EnetUdma_TxChObj *pTxChObj);

#if (UDMA_SOC_CFG_RING_MON_PRESENT == 1)
/*! Ring Monitor object allocation function  */
Udma_RingMonHandle EnetUdma_memMgrAllocRingMonObj(void);

/*! Ring Monitor object free function  */
void EnetUdma_memMgrFreeRingMonObj(Udma_RingMonHandle hUdmaRingMon);
#endif

/*! TD Cq Ring memory object allocation function  */
void *EnetUdma_memMgrAllocTdCqRingMemObj(void);

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
/*! TD Cq Ring memory object free function  */
void EnetUdma_memMgrFreeTdCqRingMemObj(void *pTdCqRingMem);
#endif

/*! Drv object memory manager init  */
void EnetUdma_memMgrInit(void);

/*! Drv object memory manager deinit function  */
void EnetUdma_memMgrDeInit(void);

void EnetUdma_txCqIsr(Udma_EventHandle hUdmaEvt,
                     uint32_t eventType,
                     void *appData);

void EnetUdma_rxCqIsr(Udma_EventHandle hUdmaEvt,
                     uint32_t eventType,
                     void *appData);

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
int32_t EnetUdma_allocProxy(Udma_DrvHandle hUdmaDrv,
                           Udma_ProxyHandle hUdmaProxy,
                           Udma_RingHandle hUdmaRing);

int32_t EnetUdma_freeProxy(Udma_ProxyHandle hUdmaProxy);
#endif

#if (UDMA_SOC_CFG_RING_MON_PRESENT == 1)
Udma_RingMonHandle EnetUdma_allocRingMon(Udma_DrvHandle hUdmaDrv,
                                        Udma_RingHandle hUdmaRing,
                                        uint32_t elemCnt,
                                        EnetUdma_RingMonCfg *pRingMonCfg);

int32_t EnetUdma_freeRingMon(Udma_RingMonHandle hRingMon);
#endif

void EnetUdmaStats_updateNotifyStats(EnetDma_CbStats *pktStats,
                                    uint32_t pktCnt,
                                    uint32_t timeDiff);

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

static inline void EnetUdmaStats_addCnt(uint64_t *statCnt,
                                       uint64_t addCnt)
{
#if defined(ENETUDMA_INSTRUMENTATION_ENABLED)
    *statCnt += addCnt;
#endif
}

#ifdef __cplusplus
}
#endif

#endif /* #ifndef ENET_UDMA_PRIV_H_ */
