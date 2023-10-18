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

/*!
 * \file  enet_cpdma.h
 *
 * \brief This file contains the type definitions and helper macros for the
 *        Enet CPDMA data path (DMA) interface.
 */

/*!
 * \ingroup  ENET_DMA_API
 * \defgroup ENET_CPDMA_API Enet CPDMA APIs
 *
 * @{
 */

#ifndef ENET_CPDMA_H_
#define ENET_CPDMA_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <include/core/enet_types.h>
#include <include/core/enet_base.h>
#include <include/core/enet_queue.h>
#include <include/dma/cpdma/enet_cpdma_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/*!
 * \name Enet CPDMA instance configuration
 *
 * Configuration macros for Enet CPDMA module.
 *
 * @{
 */

/*! \brief Maximum number of CPSW TX DMA channels. */
#define ENET_CPDMA_CPSW_MAX_TX_CH                                (8U)

/*! \brief Maximum number of CPSW RX DMA channels. */
#define ENET_CPDMA_CPSW_MAX_RX_CH                                (8U)

/*! @} */

/*! Maximum number of scatter gather segements supported in a packet */
#define ENET_CPDMA_CPSW_MAX_SG_LIST         (4U)

#if ((__ARM_ARCH == 7) && (__ARM_ARCH_PROFILE == 'R'))
/*! * \brief Set the cacheline alignment size */
#define ENETDMA_CACHELINE_ALIGNMENT            (32U)
#else
#error "Enet library compilation not supported on non cortex R cores. Update correct cache line size"
#endif

/*! Utilities to read the ChksumInfo fields */

#define ENET_CPDMA_CSUM_START_BYTE_SHIFT                         (16U)

#define ENET_CPDMA_CSUM_START_BYTE_MASK                          (((uint32_t) 0xFFU) << ENET_CPDMA_CSUM_START_BYTE_SHIFT)

#define ENET_CPDMA_CSUM_RESULT_BYTE_SHIFT                        (24U)

#define ENET_CPDMA_CSUM_RESULT_BYTE_MASK                         (((uint32_t) 0xFFU) << ENET_CPDMA_CSUM_RESULT_BYTE_SHIFT)

#define ENET_CPDMA_CSUM_BYTE_COUNT_SHIFT                         (0U)

#define ENET_CPDMA_CSUM_BYTE_COUNT_MASK                          (((uint32_t) 0x3FFU) << ENET_CPDMA_CSUM_BYTE_COUNT_SHIFT)

#define ENET_CPDMA_GET_CSUM_RESULT_BYTE(chkSumInfo) \
                ENET_FEXT(chkSumInfo, ENET_CPDMA_CSUM_RESULT_BYTE)

#define ENET_CPDMA_GET_CSUM_START_BYTE(chkSumInfo) \
                ENET_FEXT(chkSumInfo, ENET_CPDMA_CSUM_START_BYTE)

#define ENET_CPDMA_GET_CSUM_BYTE_COUNT(chkSumInfo) \
                ENET_FEXT(chkSumInfo, ENET_CPDMA_CSUM_BYTE_COUNT)

/*!
 * \brief Enet DMA configuration structure.
 *
 *  \name Enet DMA driver opaque handles
 *
 *  Opaque handle typedefs for Enet DMA driver objects.
 *  @{
 */

/*!
 * \brief Opaque handle that holds config Info for Enet DMA channel.
 */
typedef struct EnetCpdma_Cfg_s EnetDma_Cfg;

/* TODO: may be deleted */
/*!
 * \brief Function pointer type for packet notify call back.
 *
 * This is called by driver when packet is received on the RX channel or
 * transmission completed from TX channel.
 */
typedef void (*EnetDma_PktNotifyCb)(void *cbArg);

/*! @} */

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \defgroup ENET_CPDMA_PKT CPDMA Packet Definitions
 *
 * This group contains details about Enet CPDMA packet queue structures. These
 * packet queue's are used to exchange data between application and driver
 * module.
 *
 * @{
 */

/*
 * TODO: CPDMA does not support timestamp within packet
 *       Stub to share the application code?
 */

/*!
 * \brief CPPI buffer timestamp info.
 *
 * Buffer timestamp info structure.
 */
typedef struct EnetCpdma_PktTsInfo_s
{
    /*! Flag to enable/disable host TX packet timestamping */
    bool enableHostTxTs;

    /*! Host TX packet's sequence Id value */
    uint32_t txPktSeqId;

    /*! Host TX packet's message type value */
    uint8_t txPktMsgType;

    /*! Host TX packet's domain value */
    uint8_t txPktDomain;

    /*! Received packet's 64-bit ingress timestamp value */
    uint64_t rxPktTs;
} EnetCpdma_PktTsInfo;

/*!
 * \brief Transmit Packet scatter gather list entry.
 *
 * This structure contains info on scatter-gather fragments used for
 * trasmitting packets when packets are split into list of
 * discontinuous fragments called scatter-gather list. Each sg list
 * has a fragment of the packet
 */
typedef struct EnetCpdma_SGListEntry_s
{
    /*! Pointer to scatter fragment */
    uint8_t *bufPtr;

    /*! Length of valid data in the scatter fragment */
    uint32_t segmentFilledLen;

    /*! Length of allocated buffer for scatter fragment */
    uint32_t segmentAllocLen;

    bool disableCacheOps;
} EnetCpdma_SGListEntry;

/*!
 * \brief Transmit packet scatter list info
 *
 * This structure contains info on the scatter list used for
 * transmitting packets that are split into list of fragments.
 * Using a scatter list allows application to transmit packet
 * that are not a single continuous buffer but a list of
 * scatter fragments chained together. Support for scatter
 * list is needed for supporting zero copy in network
 * stacks protocols like UDP
 */
typedef struct EnetCpdma_SGList_s
{
    /*! Number of valid scatter segments in the packet to be transmitted
     *  If packet is made of a single continuous buffer (no scatter list
     *  case) then numScatterSegments == 1
     */
    uint32_t numScatterSegments;
    /*! Array of scatterList having info on each individual scatter segement */
    EnetCpdma_SGListEntry list[ENET_CPDMA_CPSW_MAX_SG_LIST];
} EnetCpdma_SGList;

typedef EnetCpdma_SGListEntry EnetDma_SGListEntry;

/*!
 * \brief Packet data structure.
 *
 * This structure contains packet info used by application for getting/passing
 * the packet data with DMA module.
 * The packet data structure supportes scatter list for transmit but not for
 * receive.
 * - Trasmit no scatter list case (single contiguous buffer trasmitted)
 *   If single buffer is used for transmission
 *     Packet Payload is bufPtr
 *     Packet length is tx.totalPktLen
 *     tx.bufPtrFilledLen is equal to totalPktLen
 *     sgList.numScatterSegments == 0
 * - Transmit scatter list case
 *   Packet is made of discontinuous chunks of data linked together
 *   If list of discontinuous fragments make up a single packet
 *     First segment is pointed to by bufPtr
 *     Packet length is tx.totalPktLen
 *     tx.bufPtrFilledLen is equal to valid data in bufPtr
 *     sgList.numScatterSegments == (number of fragments making up the packet - 1)
 *     The first segment of scatter list is pointed to by bufPtr.
 *     Length of the first segment is tx.bufPtrFilledLen
 *     The array sgList.list points to packets from 2 to the last fragments
 *     Each fragment from 2 to the last fragment has
 *       bufPtr : Pointer to start of fragment
 *       filledLen: Length of fragment
 *
 */
typedef struct EnetCpdma_PktInfo_s
{
    /*! Pointer to next buffer.
     *  Note: Keep EnetQ_Node as first member always as driver uses generic
     *  queue functions and dereferences to this member */
    EnetQ_Node node;

    /*! Pointer to application layer specific data blob */
    void *appPriv;

    /*! Packet state info. Refer to #EnetDma_PktStateModuleType,
     *  #EnetDma_PktStateMemMgr, #EnetDma_PktStateDma and
     *  #EnetDma_PktStateApp.
     *
     *  This is only for runtime check and debug, not to be used by application */
    uint32_t pktState;

    /*! Protocol information word. Currently used for checksum offload related
     *  information.
     *
     *  For RX it contains HW computed checksum. App should call appropriate
     *  CPPI helper macros to extract required fields.
     *
     *  For TX it contains information passed to HW for checksum computation.
     *  App should call appropriate CPPI helper macros to insert required
     *  fields */
    uint32_t chkSumInfo;

    /*! Packet time stamp information.
     *
     * For TX, if tsInfo.enableHostTxTs flag is set to true, packet will be timestampped
     * on egress and will trigger host transmit event in CPTS. The timestamp value is then
     * stored in CPTS FIFO with given sequence Id, message type and domain value. This
     * can be used to timestamp any packet (even non-PTP) sent from host.
     *
     * For RX, the received packet's ingress timestamp is captured and is stored in
     * tsInfo.rxPktTs. All ingress packets are timestampped. */
    EnetCpdma_PktTsInfo tsInfo;

    /*! Directed port number.
     *
     *  The port number to which the packet is to be directed bypassing ALE lookup.
     *  This value is written to Dst Tag – Low bits of packet descriptor.
     *  If ALE lookup based switching is to be performed, this value should
     *  be set to ENET_MAC_PORT_INV.
     *
     *  Note: Directed packets go to the directed port, but an ALE lookup is performed to
     *  determine untagged egress in VLAN aware mode. */
    Enet_MacPort txPortNum;

    /*! Packet's received port number
     *  This variable contains the port number from which the packet was received.
     *  This value is obtained from the Source Tag – Low bits of packet descriptor. */
    Enet_MacPort rxPortNum;

    /*! TX packet traffic class indicates which TX port queue. This is not used for CPDMA.
     *  Keeping this to maintain same pktInfo structure across CPDMA and UDMA. */
    uint32_t txPktTc;

    /*! Transmit timestamp id. Used to correlate request with response. This is not used for CPDMA.
     *  Keeping this to maintain same pktInfo structure across CPDMA and UDMA. */
    uint32_t txTsId;

    /*! Scatter Gather list information for packets to be transmitted.
     *  A single tx packet can be fragmented across multiple chunks,
     *  the bufPtrs and filled len of each segment are contained here.
     *  sgList.numScatterSegments = 1 by default.
     */
    EnetCpdma_SGList sgList;
} EnetCpdma_PktInfo;

/*! @} */


/*!
 * \defgroup ENET_CPDMA_CONFIG_DEFS CPDMA Configuration
 *
 * This group contains structure and type definitions needed to properly
 * construct the Enet CPDMA configuration structure which is part of the top-level
 * Enet CPDMA configuration structure.
 *
 * @{
 */

/*!
 * \brief Param struct for the TX channel open function.
 *
 * The configuration structure for the TX channel open function # EnetDma_openTxCh().
 */
typedef struct EnetCpdma_OpenTxChPrms_s
{
    /*! Enet handle*/
    Enet_Handle hEnet;

    /*! CPDMA channel number allocated for transmit. */
    uint32_t chNum;

    /*! Enet CPDMA event callback function - this function will be called when
     *  the registered packets are transmitted on TX channel */
    EnetDma_PktNotifyCb notifyCb;

    /*! Maximum number of transmit packets, used for allocating number of DMA descriptors
     */
    uint32_t numTxPkts;

    /*! Argument to be used for the callback routines (it should mean something
     *  to layer into which the callback calls) */
    void *cbArg;

} EnetCpdma_OpenTxChPrms;

/*!
 * \brief Param struct for the RX channel open function.
 *
 * The configuration structure for the RX channel open function #EnetDma_openTxCh().
 */
typedef struct EnetCpdma_OpenRxChPrms_s
{
    /*! Enet handle*/
    Enet_Handle hEnet;

    /*! CPDMA channel number allocated for receive. */
    uint32_t chNum;

    /*! Enet CPDMA event callback function - this function will be called when
     *  the registered packets are transmitted on TX channel */
    EnetDma_PktNotifyCb notifyCb;

    /*! Maximum number of transmit packets, used for allocating number of DMA descriptors
     */
    uint32_t numRxPkts;

    /*! Argument to be used for the callback routines (it should mean something
     *  to layer into which the callback calls) */
    void *cbArg;

} EnetCpdma_OpenRxChPrms;

/*!
 * \brief Param struct for the RX channel open function. We include this typedef as top level
 *        DMA APIs use EnetDma_OpenRxChPrms struct.
 */
typedef EnetCpdma_OpenRxChPrms EnetDma_OpenRxChPrms;

/*!
 * \brief Global Param struct for the Rx channel open
 *
 * The parameter structure for the Rx channel open, containing a global channel config
 * structure for all rx channels.
 */
typedef struct EnetCpdma_RxChInitPrms_s
{
    /*! Receive buffer offset */
    uint32_t rxBufferOffset;
} EnetCpdma_RxChInitPrms;

/*!
 * \brief Config structure for Enet CPDMA
 *
 * The parameter structure for Enet CPDMA configuration, containing a Rx channel
 * config and NAVSS instance id.
 */
typedef struct EnetCpdma_Cfg_s
{
    /*! Number of tx Interrupts per millisecond */
    uint32_t txInterruptPerMSec;

    /*! Number of rx Interrupts per millisecond */
    uint32_t rxInterruptPerMSec;

    /*! Flag to enable channel override feature to be used by classifier.
     *  For Rx channels, the packet is directed to a specific rx channel number based on switch priority by default.
	 *  The default priority mapping is identity based like packet with priority 1 will go to rx channel 1, and so on.
     *  To override this and direct packets to a specific rx channels based on ALE flow id classifier match enable this flag.
     *  This feature is supported only on AM263x SOC and not AM273x/AWR294x	*/
    bool     enChOverrideFlag;

    /*! RX channel configuration parameters */
    EnetCpdma_RxChInitPrms rxChInitPrms;

    /*! Maximum number of Tx channels. Must be <= ENET_CPDMA_CPSW_MAX_TX_CH */
    uint32_t maxTxChannels;

    /*! Maximum number of Rx channels. Must be <= ENET_CPDMA_CPSW_MAX_RX_CH */
    uint32_t maxRxChannels;

} EnetCpdma_Cfg;

/*!
 * \brief Config structure for Enet CPDMA Data Path initialization
 *
 * The parameter strcture for Enet CPDMA data path init configuration.
 */
typedef struct EnetDma_initCfg_s
{
}
EnetDma_initCfg;
/*! @} */

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief Set default data path parameters.
 *
 * \param enetType    [IN] Enet Peripheral type
 * \param instId      [IN] Enet Peripheral instance id
 * \param dmaCfg      [IN] DMA module (data path) configuration pointer.
 * \param appCoreId   [IN] Application core Id
 *
 * \retval Enet DMA handle if opened. Otherwise, NULL.
 *
 */
EnetDma_Handle EnetCpdma_open(Enet_Type enetType,
                              uint32_t instId,
                              const void *dmaCfg,
                              uint32_t appCoreId);

/*!
 * \brief Open DMA with default data path parameters.
 *
 * \param enetType    [IN] Enet Peripheral type
 * \param instId      [IN] Enet Peripheral instance id
 * \param appCoreId   [IN] Application core Id
 *
 * \retval Enet DMA handle if opened. Otherwise, NULL.
 *
 */
EnetDma_Handle EnetCpdma_restoreCtxt(Enet_Type enetType,
                            uint32_t instId,
                            uint32_t appCoreId);

/*!
 * \brief Close Enet DMA (data path).
 *
 * \param hEnetDma     [IN] DMA handle
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t EnetCpdma_close(EnetDma_Handle hEnetDma);

/*!
 * \brief Saves and closes Enet DMA (data path).
 *
 * \param hEnetDma     [IN] DMA handle
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t EnetCpdma_saveCtxt(EnetDma_Handle hEnetDma);

/*!
 * \brief ENET CPDMA Rx Threshold interrupt service routine
 *
 * Processes Rx Threshold interrupt. This function retrieves the received packets
 * from the linked CPPI descriptors and passes them to the receive queue when the
 * number of received descriptors exceed the threshold
 *
 * Requirement:
 *
 * \param hEnetDma     [IN] Enet DMA Handle
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t EnetCpdma_rxThreshIsr(EnetDma_Handle hEnetDma);

/*!
 * \brief ENET CPDMA Rx interrupt service routine
 *
 * Processes Rx interrupt. This function retrieves the received packets from
 * the linked CPPI descriptors and passes them to the receive queue when one
 * or more packets has been received
 *
 * Requirement:
 *
 * \param hEnetDma     [IN] Enet DMA Handle
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t EnetCpdma_rxIsr(EnetDma_Handle hEnetDma);

/*!
 * \brief ENET CPDMA Tx interrupt service routine
 *
 * Processes Tx interrupt. This function retrieves the completed Tx packets
 * from the linked CPPI descriptors and passes them to the free queue when
 * one or more packets have been transmitted.
 *
 * Requirement:
 *
 * \param hEnetDma     [IN] Enet DMA Handle
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t EnetCpdma_txIsr(EnetDma_Handle hEnetDma);

/*!
 * \brief ENET CPDMA Miscellaneous interrupt service routine
 *
 * Processes Miscellaneous interrupt. This function extracts the intrrupt
 * status bit masks from MMR and pass it to the caller to process all
 * masked interrupts.
 *
 * Requirement:
 *
 * \param hEnetDma     [IN] Enet DMA Handle
 * \param pStatusMask  [IN] pointer to the interrupt status bit mask
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t EnetCpdma_miscIsrGetStatus(EnetDma_Handle hEnetDma, uint32_t *pStatusMask);

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
int32_t EnetCpdma_ackMiscIsr(EnetDma_Handle hEnetDma);

/*!
 * \brief Initialize CPDMA config params
 *
 * Initialize CPDMA config params to default values
 *
 * Requirement:
 *
 * \param enetType     [IN] Enet Type
 * \param pDmaConfig  [IN] pointer to the config structure
 *
 */
void EnetCpdma_initParams(Enet_Type enetType, EnetDma_Cfg *pDmaConfig);

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

#endif /* ENET_CPDMA_H_ */

/*! @} */
