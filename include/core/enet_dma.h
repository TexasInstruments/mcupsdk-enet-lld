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
 * \file  enet_dma.h
 *
 * \brief This file contains the type definitions and helper macros for the
 *        Enet Data Path (DMA) interface.
 */

/*!
 * \ingroup  DRV_ENET_MODULE
 * \defgroup ENET_DMA_API Enet Data Path (DMA)
 *
 * @{
 */

#ifndef ENET_DMA_H_
#define ENET_DMA_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <include/core/enet_types.h>
#include <include/core/enet_queue.h>
#include <include/core/enet_dma_pktutils.h>

#if defined(SOC_AM64X) || defined(SOC_AM243X) || defined(SOC_AM62AX)
#include <drivers/udma.h>
#include <include/dma/udma/enet_udma_types.h>
#include <include/dma/udma/enet_udma.h>
#include <include/dma/udma/enet_udma_psi.h>
#elif defined (SOC_AM273X) || defined(SOC_AWR294X) || defined(SOC_AWR2544) || defined(SOC_AM263X) || defined(SOC_AM263PX)
#include <include/dma/cpdma/enet_cpdma.h>
#else
#error "SOC not supported"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */



#if defined(SOC_AM273X) || defined(SOC_AWR294X) || defined(SOC_AWR2544) || defined(SOC_AM263X) || defined(SOC_AM263PX)
/*! * \brief Set to false as Cache is not coherent in AM273X, AWR294X SOC.*/
#define Enet_isCacheCoherent()                  (false)
#elif defined(SOC_AM64X) || defined(SOC_AM243X) || defined(SOC_AM62AX)
/*! * \brief Use cache coherent macro from UDMA driver */
#define Enet_isCacheCoherent()                  (Udma_isCacheCoherent())
#else
#error "SOC not supported"
#endif

/*! \brief THOST CSUM ENCAP Word- Checksum Add bit shift. */
#define ENETDMA_RXCSUMINFO_CHKSUM_ADD_SHIFT        (0U)

/*! \brief THOST CSUM ENCAP Word - Checksum Add bit mask. */
#define ENETDMA_RXCSUMINFO_CHKSUM_ADD_MASK         (((uint32_t) 0xFFFF) << ENETDMA_RXCSUMINFO_CHKSUM_ADD_SHIFT)

/*! \brief THOST CSUM ENCAP Word - Checksum Error bit shift. */
#define ENETDMA_RXCSUMINFO_CHKSUM_ERR_SHIFT        (16U)

/*! \brief THOST CSUM ENCAP Word - Checksum Error bit mask. */
#define ENETDMA_RXCSUMINFO_CHKSUM_ERR_MASK         (((uint32_t) 0x1U) << ENETDMA_RXCSUMINFO_CHKSUM_ERR_SHIFT)

/*! \brief THOST CSUM ENCAP Word - IP Fragment bit shift. */
#define ENETDMA_RXCSUMINFO_FRAGMENT_SHIFT          (17U)

/*! \brief THOST CSUM ENCAP Word - IP Fragment bit mask. */
#define ENETDMA_RXCSUMINFO_FRAGMENT_MASK           (((uint32_t) 0x1U) << ENETDMA_RXCSUMINFO_FRAGMENT_SHIFT)

/*! \brief THOST CSUM ENCAP Word - TCP or UDP bit shift. */
#define ENETDMA_RXCSUMINFO_TCP_UDP_N_SHIFT         (18U)

/*! \brief THOST CSUM ENCAP Word - TCP or UDP bit mask. */
#define ENETDMA_RXCSUMINFO_TCP_UDP_N_MASK          (((uint32_t) 0x1U) << ENETDMA_RXCSUMINFO_TCP_UDP_N_SHIFT)

/*! \brief THOST CSUM ENCAP Word - IPv6 Valid bit shift. */
#define ENETDMA_RXCSUMINFO_IPV6_VALID_SHIFT        (19U)

/*! \brief THOST CSUM ENCAP Word - IPv6 Valid bit mask. */
#define ENETDMA_RXCSUMINFO_IPV6_VALID_MASK         (((uint32_t) 0x1U) << ENETDMA_RXCSUMINFO_IPV6_VALID_SHIFT)

/*! \brief THOST CSUM ENCAP Word - IPv4 Valid bit shift. */
#define ENETDMA_RXCSUMINFO_IPV4_VALID_SHIFT        (20U)

/*! \brief THOST CSUM ENCAP Word - IPv4 Valid bit mask. */
#define ENETDMA_RXCSUMINFO_IPV4_VALID_MASK         (((uint32_t) 0x1U) << ENETDMA_RXCSUMINFO_IPV4_VALID_SHIFT)

/*! \brief FHOST CSUM ENCAP Word - Checksum Byte Count bit shift. */
#define ENETDMA_TXCSUMINFO_CHKSUM_BYTECNT_SHIFT      (0U)

/*! \brief FHOST CSUM ENCAP Word - Checksum Byte Count bit mask. */
#define ENETDMA_TXCSUMINFO_CHKSUM_BYTECNT_MASK       (((uint32_t) 0x3FFFU) << ENETDMA_TXCSUMINFO_CHKSUM_BYTECNT_SHIFT)

/*! \brief FHOST CSUM ENCAP Word - Inverted Checksum bit shift. */
#define ENETDMA_TXCSUMINFO_CHKSUM_INV_SHIFT          (15U)

/*! \brief FHOST CSUM ENCAP Word - Inverted Checksum bit mask. */
#define ENETDMA_TXCSUMINFO_CHKSUM_INV_MASK           (((uint32_t) 0x1U) << ENETDMA_TXCSUMINFO_CHKSUM_INV_SHIFT)

/*! \brief FHOST CSUM ENCAP Word - Checksum Start Byte bit shift. */
#define ENETDMA_TXCSUMINFO_CHKSUM_STARTBYTE_SHIFT    (16U)

/*! \brief FHOST CSUM ENCAP Word - Checksum Start Byte bit mask. */
#define ENETDMA_TXCSUMINFO_CHKSUM_STARTBYTE_MASK     (((uint32_t) 0xFFU) << ENETDMA_TXCSUMINFO_CHKSUM_STARTBYTE_SHIFT)

/*! \brief FHOST CSUM ENCAP Word - Checksum Result bit shift. */
#define ENETDMA_TXCSUMINFO_CHKSUM_RESULT_SHIFT       (24U)

/*! \brief FHOST CSUM ENCAP Word - Checksum Result bit mask. */
#define ENETDMA_TXCSUMINFO_CHKSUM_RESULT_MASK        (((uint32_t) 0xFFU) << ENETDMA_TXCSUMINFO_CHKSUM_RESULT_SHIFT)


/* --- Utilities macros to extract fields from THOST Checksum Encapsulation word --- */

/*! \brief Get IPv4 flag from THOST Checksum Encapsulation word */
#define ENETDMA_RXCSUMINFO_GET_IPV4_FLAG(chkSumInfo) \
                ENET_FEXT(chkSumInfo, ENETDMA_RXCSUMINFO_IPV4_VALID)

/*! \brief Get IPv6 flag from THOST Checksum Encapsulation word */
#define ENETDMA_RXCSUMINFO_GET_IPV6_FLAG(chkSumInfo) \
                ENET_FEXT(chkSumInfo, ENETDMA_RXCSUMINFO_IPV6_VALID)

/*! \brief Get TCP or UDP flag from THOST Checksum Encapsulation word */
#define ENETDMA_RXCSUMINFO_GET_TCPUDP_N_FLAG(chkSumInfo) \
                ENET_FEXT(chkSumInfo, ENETDMA_RXCSUMINFO_TCP_UDP_N)

/*! \brief Get IP Fragment flag from THOST Checksum Encapsulation word */
#define ENETDMA_RXCSUMINFO_GET_FRAGMENT_FLAG(chkSumInfo) \
                ENET_FEXT(chkSumInfo, ENETDMA_RXCSUMINFO_FRAGMENT)

/*! \brief Get Checksum Error flag from THOST Checksum Encapsulation word */
#define ENETDMA_RXCSUMINFO_GET_CHKSUM_ERR_FLAG(chkSumInfo) \
                ENET_FEXT(chkSumInfo, ENETDMA_RXCSUMINFO_CHKSUM_ERR)

/*! \brief Get Checksum Result from THOST Checksum Encapsulation word */
#define ENETDMA_RXCSUMINFO_GET_CHKSUM_RESULT(chkSumInfo) \
                ENET_FEXT(chkSumInfo, ENETDMA_RXCSUMINFO_CHKSUM_ADD)

/* --- Utilities macros to insert fields in FHOST Checksum Encapsulation word --- */

/*! \brief Set Checksum Result Byte into FHOST Checksum Encapsulation word */
#define ENETDMA_TXCSUMINFO_SET_CHKSUM_RESBYTE(chkSumInfo, val) \
                ENET_FINS(chkSumInfo, ENETDMA_TXCSUMINFO_CHKSUM_RESULT, val)

/*! \brief Set Checksum Start Byte into FHOST Checksum Encapsulation word */
#define ENETDMA_TXCSUMINFO_SET_CHKSUM_STARTBYTE(chkSumInfo, val) \
                ENET_FINS(chkSumInfo, ENETDMA_TXCSUMINFO_CHKSUM_STARTBYTE, val)

/*! \brief Set Invert Checksum flag into FHOST Checksum Encapsulation word */
#define ENETDMA_TXCSUMINFO_SET_CHKSUM_INV_FLAG(chkSumInfo, val) \
                ENET_FINS(chkSumInfo, ENETDMA_TXCSUMINFO_CHKSUM_INV, val)

/*! \brief Set Checksum Byte Count into FHOST Checksum Encapsulation word */
#define ENETDMA_TXCSUMINFO_SET_CHKSUM_BYTECNT(chkSumInfo, val) \
                ENET_FINS(chkSumInfo, ENETDMA_TXCSUMINFO_CHKSUM_BYTECNT, val)

/*!
 * \brief Enet DMA statistics configuration.
 *
 * Number of entries kept in the statistics for DMA module.
 */
#define ENET_DMA_STATS_HISTORY_CNT                  ((uint32_t)2U)

/*!
 * \name Enet DMA driver callback function types.
 *
 * Callback function typedefs so that the EnetDma layer can call into the app layer
 * and let it translate between the DMA packet descriptors and packets and the
 * stack/translation layer's buffers and packets.
 * @{
 */

/*! \brief Function pointer type for Ethernet packet allocation function. */
typedef EnetDma_Pkt *(*EnetDma_AllocEthPktFxn)(uint32_t pktSize,
                                               uint32_t alignSize,
                                               void *appPriv);

/*! \brief Function pointer type for Ethernet packet free function. */
typedef void (*EnetDma_FreeEthPktFxn)(EnetDma_Pkt *pPktInfo);

/*! @} */

/*!
 * \brief Packet queue.
 *
 * A queue of packets, which are used for managing the packets given to the DMA driver by
 * an application. It can also be used by the translation or application layer to manage
 * free packets pools.
 */
typedef EnetQ EnetDma_PktQ;


/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \defgroup ENET_DMA_PKT_STATS Packet statistics
 *
 *  This group contains details about Enet DMA RX channel and TX channel statistics
 *
 *  @{
 */

/*!
 * \brief Stats for packets submitted/retrieved in the DMA event callbacks functions.
 */
typedef struct EnetDma_CbStats_s
{
    /*! Total number of times the packets were submitted or retrieved from DMA */
    uint64_t dataNotifyCnt;

    /*! Counts when DMA retrievePkts is called but no packets was retrieved. This count
     *  can help in determining optimal pacing interval */
    uint64_t zeroNotifyCnt;
    /*! Total number of packets submitted or retrieved from DMA */
    uint64_t totalPktCnt;
    /*! Total cycles consumed to submit or retrieve totalPktCnt packets from DMA */
    uint64_t totalCycleCnt;

    /*! Maximum packets retrieved in single submit or retrieve call. This count
     *  can help in determining optimal number of DMA descriptors and packet counts to be
     *  allocated */
    uint64_t pktsPerNotifyMax;

    /*! Packets retrieved per call to single submit or retrieve call. Maintained for
     *   last #ENET_DMA_STATS_HISTORY_CNT packets */
    uint64_t pktsPerNotify[ENET_DMA_STATS_HISTORY_CNT];
    /*! Maximum of cycles consumed of all submit or retrieve call. */
    uint64_t cycleCntPerNotifyMax;

    /*! Cycles consumed per call to single submit or retrieve call. Maintained for
     *   last #ENET_DMA_STATS_HISTORY_CNT packets */
    uint64_t cycleCntPerNotify[ENET_DMA_STATS_HISTORY_CNT];
    /*! Maximum cycles for single packet in submit or retrieve call */
    uint64_t cycleCntPerPktMax;

    /*! Cycles per packet in submit or retrieve call. Maintained for
     *   last #ENET_DMA_STATS_HISTORY_CNT packets */
    uint64_t cycleCntPerPkt[ENET_DMA_STATS_HISTORY_CNT];

    /*! Number of free DMA descriptors in the RX channels or TX channels DMA descriptor
     * queue. Maintained for last #ENET_DMA_STATS_HISTORY_CNT packets. This can be
     * useful to detect and handle drops in packet bursts*/
    uint64_t readyDmaDescQCnt[ENET_DMA_STATS_HISTORY_CNT];
} EnetDma_CbStats;

/*!
 * \brief DMA descriptor stats for the RX & TX channels.
 */
typedef struct EnetDma_DmaDescStats_s
{
    /*! Total number of empty desc enqueued for RX channel and ready desc enqueued for TX channel. */
    uint64_t readyDmaDescEnq;
    /*! Total number of ready desc dequeued for RX channel and empty desc dequeued for TX channel. */
    uint64_t freeDmaDescDeq;
    /*! Total number of DMA desc queue underflows for RX channel or TX channel */
    uint64_t underFlowCnt;
} EnetDma_DmaDescStats;

/*!
 * \brief RX channel statistics.
 */
typedef struct EnetDma_RxChStats_s
{
    /*! Stats for RX submit packet calls  */
    EnetDma_CbStats submitPktStats;
    /*! Stats for RX retrieve packet calls  */
    EnetDma_CbStats retrievePktStats;
    /*! RX channels DMD desc queue stats  */
    EnetDma_DmaDescStats dmaDescStats;
    /*! Total number of free packets enqueued for RX channel */
    uint64_t rxSubmitPktEnq;
    /*! Total number of full packets deq for RX channel */
    uint64_t rxRetrievePktDeq;
    /*! Total number of packets underflows for RX channel */
    uint64_t rxSubmitPktUnderFlowCnt;
}EnetDma_RxChStats;

/*!
 * \brief TX channel statistics.
 */
typedef struct EnetDma_TxChStats_s
{
    /*! Stats for TX submit packet calls  */
    EnetDma_CbStats submitPktStats;
    /*! Stats for TX retrieve packet calls  */
    EnetDma_CbStats retrievePktStats;
    /*! TX Channels DMD desc queue stats  */
    EnetDma_DmaDescStats dmaDescStats;
    /*! Total number of free packets enqueued for TX Channel */
    uint64_t txSubmitPktEnq;
    /*! Total number of full packets deq for TX Channel */
    uint64_t txRetrievePktDeq;
    /*! Total number of packets underflows for TX Channel */
    uint64_t txSubmitPktOverFlowCnt;
} EnetDma_TxChStats;

/*! @} */

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief Initialize data path.
 *
 * Initialize DMA data path state on remote core.
 *
 * \param enetType [IN]  Enet Peripheral type
 * \param instId   [IN] Enet Peripheral instance id
 * \param pDmaCfg  [OUT] Data Path Init params
 *
 * \return Enet DMA opaque handle
 */
EnetDma_Handle EnetDma_initDmaCfg(Enet_Type enetType,
                                  uint32_t instId,
                                  const EnetDma_initCfg *pDmaCfg);

/*!
 * \brief De-initialize data path.
 *
 * De-initialize DMA data path state on remote core.
 *
 * \param hEnetUdma     [IN] Cpsw DMA Handle
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t EnetDma_deinitDmaCfg(EnetDma_Handle hEnetUdma);

/*!
 * \brief Initialize RX channel open parameters.
 *
 * Initializes RX channel parameters with default values.
 * Refer to SOC DMA specific RX channel config structure for specific config details.
 *
 * \param pRxChCfg RX channel configuration parameters.
 *
 */
void EnetDma_initRxChParams(void *pRxChCfg);

/*!
 * \brief Enet DMA open RX channel.
 *
 * Opens the Enet DMA RX channel based on the channel parameters. This function
 * configures the DMA channel. This also configures event if notifyCb is not null.
 * Refer to SOC DMA specific RX channel config structure for specific config details.
 *
 * Enet DMA is peripheral-aware as peripherals in a given SoC may need different handling,
 * i.e. DMA descriptor's extra fields having different meaning for two peripherals using
 * same DMA engine.  This peripheral-awareness is given to the RX channel/flow via
 * #EnetDma_Handle passed at open time.
 *
 *
 * \param hDma      Enet DMA handle
 * \param pRxChCfg  RX channel configuration parameters. This parameter can't be NULL.
 *
 * \return RX channel opaque handle if opened. Otherwise, NULL.
 */
EnetDma_RxChHandle EnetDma_openRxCh(EnetDma_Handle hDma,
                                    const void *pRxChCfg);

/*!
 * \brief Enet DMA close RX channel.
 *
 * Closes the Enet DMA RX channel and frees all associated resources. During close
 * operation, we flush FQ taking all DMA descriptors with packet submitted in
 * advance for reception and return to app. Also we retrieve all packets from
 * the CQ (packets received between last #EnetDma_retrieveRxPktQ() function call) and
 * return those to app. App doesn't need to call function #EnetDma_retrieveRxPktQ()
 * explicitly to retrieve these packets.
 *
 * \param hRxCh   [IN] Enet DMA channel handle.
 *                     This parameter can't be NULL.
 * \param fq      [OUT] Pointer to #EnetDma_PktQ structure where packets
 *                      from FQ (submitted for reception) are retrieved and returned
 *                      to application. This parameter can't be NULL.
 * \param cq      [OUT] Pointer to #EnetDma_PktQ structure where packets
 *                      from CQ (received packets) are retrieved and returned to application.
 *                      This parameter can't be NULL.
 *  \return \ref Enet_ErrorCodes
 */
int32_t EnetDma_closeRxCh(EnetDma_RxChHandle hRxCh,
                          EnetDma_PktQ *fq,
                          EnetDma_PktQ *cq);

/*!
 * \brief Enable RX channel packet reception event.
 *
 * Enables the packet arrival event for RX channel. This allows application to
 * control notify events runtime and retrieve packet only once to do burst packet
 * processing.
 *
 * \param hRxCh  [IN] Handle to Enet DMA RX channel
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t EnetDma_enableRxEvent(EnetDma_RxChHandle hRxCh);

/*!
 * \brief Register packet arrival event callback
 *
 * Enables the packet arrival event for RX channel. This allows application to
 * control notify events runtime and retrieve packet only once to do burst packet
 * processing.
 *
 * \param hRxCh    [IN] Handle to Enet DMA RX channel
 * \param notifyCb [IN] Packet rx event notify function callback
 * \param cbArg    [IN] Argument to be passed to notify function callback
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t EnetDma_registerRxEventCb(EnetDma_RxChHandle hRxCh, EnetDma_PktNotifyCb notifyCb, void *cbArg);

/*!
 * \brief Disable RX channel packet reception event.
 *
 * Disable the packet arrival event for RX channel.
 *
 * \param hRxCh [IN] Handle to Enet DMA RX channel
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t EnetDma_disableRxEvent(EnetDma_RxChHandle hRxCh);

/*!
 * \brief Initialize TX channel open parameters.
 *
 * Initializes TX channel open parameters with default values.
 * Refer to SOC DMA specific RX channel config structure for specific config details.
 *
 * \param pTxChCfg  TX channel configuration parameters.
 *
 */
void EnetDma_initTxChParams(void *pTxChCfg);

/*!
 * \brief Enet DMA open TX channel.
 *
 * Opens the DMA TX DMA channel based on the channel parameters. This function
 * open TX channel using chNum provided in EnetDma_OpenTxChPrms() and configures
 * TX channel. This also configures event if notifyCb is not null.
 * Refer to SOC DMA specific RX channel config structure for specific config details.
 *
 * Enet DMA is peripheral-aware as peripherals in a given SoC may need different handling,
 * i.e. DMA descriptor's extra fields having different meaning for two peripherals using
 * same DMA engine.  This peripheral-awareness is given to the TX channel via
 * #EnetDma_Handle passed at open time.
 *
 * \param hDma      Enet DMA handle
 * \param pTxChCfg  TX channel configuration parameters. This parameter can't be NULL.
 *
 * \return TX channel opaque handle if opened. Otherwise, NULL.
 */
EnetDma_TxChHandle EnetDma_openTxCh(EnetDma_Handle hDma,
                                    const void *pTxChCfg);

/*!
 * \brief Enet DMA close TX channel.
 *
 * Closes the Enet DMA TX channel and frees all associated resources. During
 * close operation, we flush FQ taking all DMA descriptors with packet submitted
 * but not yet transmitted and return to app. Also we retrieve all packets from
 * the CQ (transmission completed packets) and return those to app. App doesn't
 * need to call EnetDma_retrieveTxPktQ() explicitly to retrieve these
 * packets.
 *
 * \param hTxCh    [IN] Enet DMA TX Channel handle.
 *                      This parameter can't be NULL.
 * \param fq      [OUT] Pointer to #EnetDma_PktQ structure where packets from FQ
 *                   (TX ready - submitted for transmission) are retrieved and returned to application.
 *                   This parameter can't be NULL.
 * \param cq      [OUT] Pointer to #EnetDma_PktQ structure where packets from CQ
 *                   (TX free - transmitted packets) are retrieved and returned to application.
 *                   This parameter can't be NULL.
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t EnetDma_closeTxCh(EnetDma_TxChHandle hTxCh,
                          EnetDma_PktQ *fq,
                          EnetDma_PktQ *cq);

/*!
 * \brief Enable TX channel packet transmit completion event.
 *
 * Enables the packet transmit event for TX channel. This allows application to
 * control notify events runtime and retrieve empty packets only once to do burst packet
 * processing.
 *
 * \param hTxCh  [IN] Handle to Enet DMA TX channel
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t EnetDma_enableTxEvent(EnetDma_TxChHandle hTxCh);

/*!
 * \brief Register packet transmit completion event callback
 *
 *
 * \param hTxCh    [IN] Handle to Enet DMA TX channel
 * \param notifyCb [IN] Packet tx event notify function callback
 * \param cbArg    [IN] Argument to be passed to notify function callback
 * \return \ref Enet_ErrorCodes
 */
int32_t EnetDma_registerTxEventCb(EnetDma_TxChHandle hTxCh, EnetDma_PktNotifyCb notifyCb, void *cbArg);

/*!
 * \brief Disable TX channel packet transmit completion event.
 *
 * Disable the packet transmit event for TX channel.
 *
 * \param hTxCh  [IN] Handle to Enet DMA TX channel
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t EnetDma_disableTxEvent(EnetDma_TxChHandle hTxCh);

/*!
 * \brief Retrieve queue of RX ready (full) packets from RX channel.
 *
 * \param hRxCh       [IN] Enet DMA RX channel handle for the RX channel we
 *                         wish to retrieve packets from
 * \param pRetrieveQ  [OUT] Pointer to #EnetDma_PktQ structure where
 *                          packets from hardware are retrieved and returned to
 *                          application
 *
 *  \return \ref Enet_ErrorCodes
 */
int32_t EnetDma_retrieveRxPktQ(EnetDma_RxChHandle hRxCh,
                               EnetDma_PktQ *pRetrieveQ);

/*!
 * \brief Retrieve single RX ready (full) packet (single) from RX channel.
 *
 * \param hRxCh       [IN] Enet DMA RX channel handle for the RX channel we
 *                        wish to retrieve packet from
 * \param pPkt        [OUT] Pointer to #EnetDma_Pkt structure where
 *                          packet from hardware is retrieved and returned to
 *                          application
 *
 *  \return \ref Enet_ErrorCodes
 */
int32_t EnetDma_retrieveRxPkt(EnetDma_RxChHandle hRxCh,
                              EnetDma_Pkt **pPkt);

/*!
 * \brief Submit queue of RX free (empty) packets for reception to RX channel.
 *
 * \param hRxCh      [IN] Enet DMA RX channel handle for the RX channel we
 *                        wish to submit packets to
 * \param pSubmitQ   [IN] Pointer to #EnetDma_PktQ structure containing packets
 *                        to be submitted to channel
 *                   [OUT] Returned packets for which driver couldn't allocate the DMA
 *                         descriptors.
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t EnetDma_submitRxPktQ(EnetDma_RxChHandle hRxCh,
                             EnetDma_PktQ *pSubmitQ);

/*!
 * \brief Submit single RX free (empty) packet for reception to RX channel.
 *
 * \param hRxCh      [IN] Enet DMA RX channel handle for the RX channel we
 *                        wish to submit packet to
 * \param pPkt [IN] Pointer to #EnetDma_Pkt structure containing packet to be
 *                        submitted to channel
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t EnetDma_submitRxPkt(EnetDma_RxChHandle hRxCh,
                            EnetDma_Pkt *pPkt);

/*!
 * \brief Retrieve queue of TX free (empty) packets from TX channel.
 *
 * \param hTxCh           [IN] Enet DMA TX channel handle for the TX channel we
 *                        wish to retrieve packet from
 * \param pRetrieveQ  [OUT] Pointer to #EnetDma_PktQ structure where
 *                        packets from hardware are retrieved and returned to
 *                        application
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t EnetDma_retrieveTxPktQ(EnetDma_TxChHandle hTxCh,
                               EnetDma_PktQ *pRetrieveQ);

/*!
 * \brief Retrieve single TX free (empty) packet from TX channel.
 *
 * \param hTxCh           [IN] Enet DMA TX channel handle for the TX channel we
 *                        wish to retrieve packet from
 * \param pPkt            [OUT] Pointer to #EnetDma_Pkt structure where
 *                        packet from hardware is retrieved and returned to
 *                        application
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t EnetDma_retrieveTxPkt(EnetDma_TxChHandle hTxCh,
                              EnetDma_Pkt **pPkt);

/*!
 * \brief Submit a queue of ready (full) packets to TX channel.
 *
 * \param hTxCh      [IN] TX Channel handle for the TX channel we wish to
 *                        submit packets to
 * \param pSubmitQ   [IN] Pointer to #EnetDma_Pkt structure containing packets
 *                        to be submitted.
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t EnetDma_submitTxPktQ(EnetDma_TxChHandle hTxCh,
                             EnetDma_PktQ *pSubmitQ);

/*!
 * \brief Submit a single ready (full) packet to TX channel.
 *
 * \param hTxCh      [IN] TX Channel handle for the TX channel we wish to
 *                        submit packets to
 * \param pPkt   [IN] Pointer to #EnetDma_Pkt structure containing packet
 *                        to be submitted.
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t EnetDma_submitTxPkt(EnetDma_TxChHandle hTxCh,
                            EnetDma_Pkt *pPkt);

/*!
 * \brief Initialize packet information structure.
 *
 * Initialize packet information structure with null/zero values.
 *
 * \param pktInfo     [IN] Pointer to #EnetDma_Pkt structure
 */
void EnetDma_initPktInfo(EnetDma_Pkt *pktInfo);

/*!
 * \brief Get RX channel statistics.
 *
 * \param hRxCh      [IN] RX channel handle
 * \param pStats  [INOUT] Pointer to RX channel stats object where stats would
 *                     be returned
 *
 * \retval ENET_SOK            Retrieved stats successfully
 * \retval ENET_ENOTSUPPORTED  Stats is not supported (enabled) or error retrieving
 */
int32_t EnetDma_getRxChStats(EnetDma_RxChHandle hRxCh,
                             EnetDma_RxChStats *pStats);

/*!
 * \brief Get TX channel statistics.
 *
 * \param hTxCh       [IN] TX channel handle
 * \param pStats   [INOUT] Pointer to TX Channel stats object where stats
 *                    would be returned
 *
 * \retval ENET_SOK            Retrieved stats successfully
 * \retval ENET_ENOTSUPPORTED  Stats is not supported (enabled) or error retrieving
 */
int32_t EnetDma_getTxChStats(EnetDma_TxChHandle hTxCh,
                             EnetDma_TxChStats *pStats);

/*!
 * \brief Reset RX channel statistics.
 *
 * \param hRxCh     [IN] RX channel handle
 *
 * \retval ENET_SOK            Reseted stats successfully
 * \retval ENET_ENOTSUPPORTED  Stats is not supported (enabled) or error resetting
 */
int32_t EnetDma_resetRxChStats(EnetDma_RxChHandle hRxCh);

/*!
 * \brief Reset TX channel statistics.
 *
 * \param hTxCh     [IN] TX Channel handle
 *
 * \retval ENET_SOK            Reseted stats successfully
 * \retval ENET_ENOTSUPPORTED  Stats is not supported (enabled) or error resetting
 */
int32_t EnetDma_resetTxChStats(EnetDma_TxChHandle hTxCh);

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

#endif /* ENET_DMA_H_ */

/*! @} */
