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
 * \file  enet_udma.h
 *
 * \brief This file contains the type definitions and helper macros for the
 *        Enet UDMA data path (DMA) interface.
 */

/*!
 * \ingroup  ENET_DMA_API
 * \defgroup ENET_UDMA_API Enet UDMA module APIs
 *
 * @{
 */

#ifndef ENET_UDMA_H_
#define ENET_UDMA_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <include/core/enet_types.h>
#include <include/core/enet_queue.h>
#include <include/dma/udma/enet_udma_types.h>

#include <drivers/udma.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/*!
 * \name Enet UDMA instance configuration
 *
 * Configuration macros for Enet UDMA module.
 *
 * @{
 */

/*! \brief Enet UDMA HPD packet size. */
#define ENET_UDMA_HPD_SIZE                                   (128U)

/*! \brief Extended Packet Info Block size. */
#define ENET_UDMA_EXTENDED_PKT_INFO_BLOCK_SIZE               (16U)

/*! \brief Extended Packet Info Block size. */
#define ENET_UDMA_PROTOCOL_SPECIFIC_INFO_BLOCK_SIZE          (16U)

/*! \brief Extended Packet Info Block size. */
#define ENET_UDMA_PKT_DESC_RESERVED_SIZE ( ENET_UDMA_HPD_SIZE - \
                                           sizeof(CSL_UdmapCppi5HMPD) - \
                                           (ENET_UDMA_EXTENDED_PKT_INFO_BLOCK_SIZE + \
                                           ENET_UDMA_PROTOCOL_SPECIFIC_INFO_BLOCK_SIZE))

/*! \brief UDMA ring single element size. */
#define ENET_UDMA_RING_MEM_SIZE                              (sizeof(uint64_t))

/*!
 * \brief Enet UDMA RX MTU alignment. The RxFlow MTU must be aligned to this value.
 *
 * As per UDMAP spec:
 * "RX Packet Size Threshold 0: This value is left shifted by 5 bits and compared
 * against the packet size to determine which free descriptor queue should be used
 * for the SOP buffer in the packet"
 * The value programmed is hence supposed to be 32 bit aligned. Programming
 * non aligned values will result in truncation and expected MTU size will not be
 * programmed.
 * For example setting MTU size of 1518 (required MTU size) results in actual value
 * of 1504 getting programmed resulting in packet drop for frames of size
 * 1504 - 1518 which the app expects to less than the programmed MTU based on the
 * param passed. Refer ksdma_udmap spec section 4.4.2.6 for more details
*/
#define ENET_UDMA_RXMTU_ALIGN                                (1U << 5U)

/*! Maximum number of scatter gather segments supported in a packet */
#define ENET_UDMA_CPSW_MAX_SG_LIST                           (4U)

/*! \brief Index of the Host Packet(first packet) descriptor */
#define ENET_UDMA_CPSW_HOSTPKTDESC_INDEX                     (0U)

/*! \brief Index of the first Host Buffer descriptor */
#define ENET_UDMA_CPSW_HOSTBUFDESC_INDEX                     (1U)

/*! Count of Host Buffer Descriptor */
#define ENET_UDMA_CPSW_MAX_HOSTBUFDESC_COUNT                 (ENET_UDMA_CPSW_MAX_SG_LIST - ENET_UDMA_CPSW_HOSTBUFDESC_INDEX)

/*! \brief Source tag low mask of descriptor (used to get packet's received port number) */
#define ENET_UDMA_HPD_SRC_TAG_LOW_MASK                       (0xFF)

#if ((__ARM_ARCH == 7) && (__ARM_ARCH_PROFILE == 'R'))
/*! * \brief Set the cacheline alignment size */
#define ENETDMA_CACHELINE_ALIGNMENT            (32U)
#else
#error "Enet library compilation not supported on non cortex R cores. Update correct cache line size"
#endif

/*! \brief UDMA descriptor address alignment requirement */
#define ENET_UDMA_DESC_ALIGNMENT               (64U)

/*!
 * \brief Opaque handle that holds config Info for Enet DMA channel.
 */
typedef struct EnetUdma_Cfg_s EnetDma_Cfg;

/*! @} */

/*!
 * \brief Opaque handle to Enet UDMA descriptor queue.
 */
typedef struct EnetUdma_DmaDescQ_s *EnetUdma_DmaDescQHandle;

/*!
 *  \name Enet DMA driver callback function types
 *
 *   Callback function typedefs so that the EnetDma layer can call into the app layer
 *   and let it translate between the hardware packet descriptors and packets and the
 *   stack/translation layer's buffers and packets.
 *  @{
 */

/*! \brief Function pointer type for packet allocation function. */
typedef uint8_t *(*EnetUdma_AllocRingMemFxn)(void *appPriv,
                                            uint32_t numRingEle,
                                            uint32_t alignSize);

/*! \brief Function pointer type for packet free function. */
typedef void (*EnetUdma_FreeRingMemFxn)(void *appPriv,
                                       void *pRingMem,
                                       uint32_t numRingEle);

/*! \brief Function pointer type for DMA descriptor allocation function. */
typedef struct EnetUdma_DmaDesc_s *(*EnetUdma_AllocDmaDescFxn)(void *appPriv,
                                                             uint32_t alignSize);

/*! \brief Function pointer type for DMA descriptor free function. */
typedef void (*EnetUdma_FreeDmaDescFxn)(void *appPriv,
                                       struct EnetUdma_DmaDesc_s *dmaDescPtr);

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
 * \defgroup ENET_UDMA_PKT UDMA Packet Definitions
 *
 * This group contains details about Enet UDMA packet queue structures. These
 * packet queue's are used to exchange data between application and driver
 * module.
 *
 * @{
 */

/*!
 * \brief CPPI buffer timestamp info.
 *
 * Buffer timestamp info structure.
 */
typedef struct EnetUdma_PktTsInfo_s
{
    /*! Flag to enable/disable host TX packet timestamping */
    bool enableHostTxTs;

    /*! Host TX packet's sequence Id value */
    uint32_t txPktSeqId;

    /*! Host TX packet's message type value */
    uint8_t txPktMsgType;

    /*! Host TX packet's domain value */
    uint8_t txPktDomain;

    /*! Received packet's 64-bit ingress timestamp value in nanoseconds */
    uint64_t rxPktTs;
} EnetUdma_PktTsInfo;

/*!
 * \brief Packet data structure.
 *
 * This structure contains packet info used by application for getting/passing
 * the packet data with DMA module.
 */

/*!
 * \brief Transmit Packet scatter gather list entry.
 *
 * This structure contains info on scatter-gather fragments used for
 * transmitting packets when packets are split into list of
 * discontinuous fragments called scatter-gather list. Each sg list
 * has a fragment of the packet
 */
typedef struct EnetUdma_SGListEntry_s
{
    /*! Pointer to scatter fragment */
    uint8_t *bufPtr;

    /*! Length of valid data in the scatter fragment */
    uint32_t segmentFilledLen;

    /*! Length of allocated buffer for scatter fragment */
    uint32_t segmentAllocLen;

    bool disableCacheOps;
} EnetUdma_SGListEntry;

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
typedef struct EnetUdma_SGList_s
{
    /*! Number of valid scatter segments in the packet to be transmitted
     *  If packet is made of a single continuous buffer then numScatterSegments = 1
     */
    uint32_t numScatterSegments;
    /*! Array of scatterList having info on each individual scatter segment */
    EnetUdma_SGListEntry list[ENET_UDMA_CPSW_MAX_SG_LIST];
} EnetUdma_SGList;

typedef EnetUdma_SGListEntry EnetDma_SGListEntry;
typedef struct EnetUdma_PktInfo_s
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
    EnetUdma_PktTsInfo tsInfo;

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

    /*! TX packet traffic class indicates which TX port queue the ICSSG FW should
      * use the transmit the packet to the PHY */
    uint32_t txPktTc;

    /*! Transmit timestamp id. Used to correleate request with response */
    uint32_t txTsId;

    /*! Scatter Gather list information for packets to be transmitted.
     * A single tx packet can be fragmented across multiple chunks,
     * the bufPtrs and filled len of each segment are contained here.
     * sgList.numScatterSegments = 1 by default.
     */
    EnetUdma_SGList sgList;

}EnetUdma_PktInfo;

/*!
 * \brief Enet UDMA descriptor format.
 *
 * Enet UDMA packet descriptor used to share meta-data information with DMA hardware.
 */
typedef struct EnetUdma_CpswHpdDesc_s
{
    /*! Host-mode packet descriptor (must be the first member of the structure */
    CSL_UdmapCppi5HMPD hostDesc  __attribute__ ((aligned(ENET_UDMA_DESC_ALIGNMENT)));

    /*! Extended Packet Info Block.
     *
     * Note:
     * - For RX Flow extendedPktInfo will be present only if flowPrms.einfoPresent
     *   is set to true (TISCI_MSG_VALUE_RM_UDMAP_RX_FLOW_EINFO_PRESENT)
     * - For TX channel extendedPktInfo will be present only if txPrms.filterEinfo
     *   is set to false(TISCI_MSG_VALUE_RM_UDMAP_TX_CH_FILT_EINFO_DISABLED)
     */
    uint8_t extendedPktInfo[ENET_UDMA_EXTENDED_PKT_INFO_BLOCK_SIZE];

    /*! Protocol Specific Info.
     *
     * Note:
     * - For RX Flow psInfo will be present only if flowPrms.psInfoPresent
     *   is set to true (TISCI_MSG_VALUE_RM_UDMAP_RX_FLOW_PSINFO_PRESENT)
     * - For TX channel psInfo will be present only if txPrms.filterPsWords
     *   is set to false(TISCI_MSG_VALUE_RM_UDMAP_TX_CH_FILT_PSWORDS_DISABLED)
     */
    uint8_t psInfo[ENET_UDMA_PROTOCOL_SPECIFIC_INFO_BLOCK_SIZE];

    /*! Reserved area. Desc size is 128 bytes (HMPD + reserved = 128) */
    uint8_t reserved[ENET_UDMA_PKT_DESC_RESERVED_SIZE];
} EnetUdma_CpswHpdDesc;

/*!
 * \brief CPPI DMA descriptor.
 *
 * The format of a single DMA descriptor to be transmitted via the Enet UDMA
 * interface.
 */
typedef struct EnetUdma_DmaDesc_s
{
    /*! CPPI HPD descriptor.
     * Note: This MUST be first member as it is typecasted to
     * #EnetUdma_CpswHpdDesc */
    EnetUdma_CpswHpdDesc hpdDesc;

    struct EnetUdma_HBDDesc_s
    {
        CSL_UdmapCppi5HMPD desc __attribute__ ((aligned(ENET_UDMA_DESC_ALIGNMENT)));
    } hostBufDesc[ENET_UDMA_CPSW_MAX_HOSTBUFDESC_COUNT];

    /*! Pointer to next DMA desc in the queue */
    struct EnetUdma_DmaDesc_s *pNextDesc;

    /*! Application packet details - this is used by application to store
     *  original packet info. This is to avoid asking buffer address from
     *  driver descriptor */
    EnetDma_Pkt *dmaPkt;
} EnetUdma_DmaDesc;

/*! @} */

/*!
 * \defgroup ENET_UDMA_CONFIG_DEFS UDMA Configuration
 *
 * This group contains structure and type definitions needed to properly
 * construct the Enet UDMA configuration structure which is part of the top-level
 * Enet UDMA configuration structure.
 *
 * @{
 */

/*!
 * \brief UDMA ring monitor config parameters.
 *
 * UDMA Ring Monitor configuration paramters. This structure is stripped-down
 * version of Udma_RingMonPrms.
 */
typedef struct EnetUdma_RingMonCfg_s
{
    /*! Ring monitor mode. Refer \ref tisci_msg_rm_ring_mon_cfg_req::mode */
    uint8_t mode;

    /*! When mode is TISCI_MSG_VALUE_RM_MON_MODE_PUSH_POP, this is read-only
     *  and represents number of pushes.
     *  When mode is TISCI_MSG_VALUE_RM_MON_MODE_THRESHOLD, this is read-write
     *  and represents the low threshold value which should be programmed
     *  to generate the RM event when the threshold is crossed (goes below).
     *  When mode is TISCI_MSG_VALUE_RM_MON_MODE_WATERMARK, this is read only
     *  and represents the low watermark.
     *  When mode is TISCI_MSG_VALUE_RM_MON_MODE_STARVATION, this is read only
     *  and represents the starvation count */
    uint32_t data0;

    /*! When mode is TISCI_MSG_VALUE_RM_MON_MODE_PUSH_POP, this is read-only
     *  and represents number of pops.
     *  When mode is TISCI_MSG_VALUE_RM_MON_MODE_THRESHOLD, this is read-write
     *  and represents the high threshold value which should be programmed
     *  to generate the RM event when the threshold is crossed (goes above).
     *  When mode is TISCI_MSG_VALUE_RM_MON_MODE_WATERMARK, this is read only
     *  and represents the high watermark.
     *  When mode is TISCI_MSG_VALUE_RM_MON_MODE_STARVATION, this is not
     *  applicable */
    uint32_t data1;
} EnetUdma_RingMonCfg;

/*!
 * \brief Enet UDMA channel ring parameters.
 *
 * Ring configuration structure for RX flow and TX channel FQ & CQ rings.
 */
typedef struct EnetUdma_UdmaRingPrms_s
{
    /*! Ring bus order ID value to be programmed into the orderid field of
     *  the ring's RING_ORDERID register. */
    uint8_t orderId;

    /*! Ring mode. Refer \ref tisci_msg_rm_ring_cfg_req::mode */
    uint8_t mode;

    /*! Flag to indicate if ring monitor should be allocated for this ring.
     *  This should be enabled only for those flows that require very high
     *  throughput as there are limited ring monitors available in the
     *  system */
    bool useRingMon;

    /*! Ring monitor config (used only if useRingMon is true) */
    EnetUdma_RingMonCfg ringMonCfg;
} EnetUdma_UdmaRingPrms;

/*!
 * \brief Enet UDMA RX flow/TX channel ring configuration parameters.
 *
 * The structure contains FQ and CQ ring configuration for RX flow/TX channel.
 * Teardown ring is allocated in the driver so we don't take it here.
 */
typedef struct EnetUdma_UdmaChPrms_s
{
#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
    /*! [IN] Free queue ring params where descriptors are queued */
    EnetUdma_UdmaRingPrms fqRingPrms;
#endif

    /*! [IN] Completion queue ring params where descriptors are dequeued */
    EnetUdma_UdmaRingPrms cqRingPrms;
} EnetUdma_UdmaChPrms;

/*!
 * \brief UDMA RX channel flow parameters.
 *
 * The structure is stripped down version of #Udma_FlowPrms.
 */
typedef struct EnetUdma_UdmaFlowPrms_s
{
    /*! [IN] Set to 1 if extended packet info is present in the descriptor */
    uint8_t einfoPresent;

    /*! [IN] Set to 1 if protocol-specific info is present in the
    *    descriptor */
    uint8_t psInfoPresent;

    /*! [IN] Start of rx packet data (byte offset from the start of the SOP
     *  buffer) */
    uint16_t sopOffset;

    /*! [IN] RX destination queue */
    uint16_t defaultRxCQ;

    /*! [IN] UDMAP receive flow source tag high byte constant configuration
     *  to be programmed into the rx_src_tag_hi field of the flow's RFLOW_RFB
     *  register */
    uint8_t srcTagHi;

    /*! [IN] UDMAP receive flow source tag low byte constant configuration
     *  to be programmed into the rx_src_tag_lo field of the flow's RFLOW_RFB
     *  register */
    uint8_t srcTagLo;

    /*! [IN] UDMAP receive flow source tag high byte selector configuration
     *  to be programmed into the rx_src_tag_hi_sel field of the RFLOW_RFC
     *  register. Refer #tisci_msg_rm_udmap_flow_cfg_req::rx_src_tag_hi_sel */
    uint8_t srcTagHiSel;

    /*! [IN] UDMAP receive flow source tag low byte selector configuration
     *  to be programmed into the rx_src_tag_low_sel field of the RFLOW_RFC
     *  register. Refer #tisci_msg_rm_udmap_flow_cfg_req::rx_src_tag_lo_sel */
    uint8_t srcTagLoSel;

    /*! [IN] UDMAP receive flow destination tag high byte constant configuration
     *  to be programmed into the rx_dest_tag_hi field of the flow's RFLOW_RFB
     *  register */
    uint8_t destTagHi;

    /*! [IN] UDMAP receive flow destination tag low byte constant configuration
     *  to be programmed into the rx_dest_tag_lo field of the flow's RFLOW_RFB
     *  register */
    uint8_t destTagLo;

    /*! [IN] UDMAP receive flow destination tag high byte selector configuration
     *  to be programmed into the rx_dest_tag_hi_sel field of the RFLOW_RFC
     *  register. Refer #tisci_msg_rm_udmap_flow_cfg_req::rx_dest_tag_hi_sel */
    uint8_t destTagHiSel;

    /*! [IN] UDMAP receive flow destination tag low byte selector configuration
     *  to be programmed into the rx_dest_tag_low_sel field of the RFLOW_RFC
     *  register. Refer #tisci_msg_rm_udmap_flow_cfg_req::rx_dest_tag_lo_sel */
    uint8_t destTagLoSel;

    /*! [IN] UDMAP receive flow packet size based free buffer queue enable
     * configuration */
    uint8_t sizeThreshEn;

} EnetUdma_UdmaFlowPrms;

/*!
 * \brief Enet UDMA TX channel parameters.
 *
 * The structure is stripped down version of #Udma_ChTxPrms
 */
typedef struct EnetUdma_UdmaChTxPrms_s
{
    /*! [IN] Bool: When set (TRUE), filter out extended info */
    uint8_t filterEinfo;

    /*! [IN] Bool: When set (TRUE), filter out protocl specific words */
    uint8_t filterPsWords;

    /*! [IN] Address type for this channel.
     *   Refer #tisci_msg_rm_udmap_tx_ch_cfg_req::tx_atype */
    uint8_t addrType;

    /*! [IN] Channel type. Refer #tisci_msg_rm_udmap_tx_ch_cfg_req::tx_chan_type */
    uint8_t chanType;

    /*! [IN] 3-bit priority value (0=highest, 7=lowest) */
    uint8_t busPriority;

    /*! [IN] 3-bit qos value (0=highest, 7=lowest) */
    uint8_t busQos;

    /*! [IN] 4-bit orderid value */
    uint8_t busOrderId;

    /*! [IN] This field selects which scheduling bin the channel will be placed
     *  in for bandwidth allocation of the TX DMA units.
     *  Refer #tisci_msg_rm_udmap_tx_ch_cfg_req::tx_sched_priority */
    uint8_t dmaPriority;

    /*! [IN] TX credit for external channels */
    uint8_t txCredit;

    /*! [IN] The fifo depth is used to specify how many FIFO data phases deep
     *  the TX per channel FIFO will be for the channel.
     *  While the maximum depth of the TX FIFO is set at design time, the FIFO
     *  depth can be artificially reduced in order to control the maximum latency
     *  which can be introduced due to buffering effects */
    uint16_t fifoDepth;
} EnetUdma_UdmaChTxPrms;

/*!
 * \brief Enet UDMA channel/flow auto-reclaim config struct.
 */
typedef struct EnetUdma_AutoReclaimPrms_s
{
    /*! Auto-reclaim enable flag
     *  Auto-reclaim or auto-recycling of buffer descriptors enables usage of same
     *  buffer descriptor between RX flow and TX channel.
     *  For example - by using auto-reclaim of RX FQ as TX CQ, we don't need to call
     *    EnetDma_retrieveTxPktQ and EnetUdma_submitRxReadyPackets
     *    in packet forwarding use-cases like SW interVLAN routing
     *  Note that current design offers/provides way to use TX FQ as RX CQ but not vice
     *  versa (RX CQ as TX FQ), similarly for RX FQ as TX CQ case. This is done due to
     *  UDMA HW design where FQ ring number is same as channel number. Basically any ring
     *  can't be attached as FQ ring to TX channel. Though this limitation doesn't exist
     *  for RX flow, we keep design consistent and enable auto-reclaim of RX FQ as TX CQ
     */
    bool enableFlag;

    /*! Handle to free DMA descriptor pool where a free descriptors retrieved in
     *  EnetDma_retrieveRxPktQ() function. This is as we bypass
     *  EnetDma_retrieveTxPktQ() in auto-reclaim */
    EnetUdma_DmaDescQHandle hDmaDescPool;

    /*! handle to ring used in auto-reclaim. For example when using
     *  TX CQ --> RX FQ, the RX FQ ring handle is passed by an app to be used as
     *  TX CQ so default ring should not be allocated/used */
    Udma_RingHandle hReclaimRing;
} EnetUdma_AutoReclaimPrms;

/*!
 * \brief Param struct for the get default flowId open parameters.
 *
 * The param struct for the RX flow open function
 * The returned channel handle is an opaque variable used to access the RX Flow in other
 * function commands.
 */
typedef struct EnetUdma_OpenRxFlowPrms_s
{
    /*! UDMA driver handle*/
    Udma_DrvHandle hUdmaDrv;

    /*! Channel index*/
    uint32_t chIdx;

    /*! Flow start index*/
    uint32_t startIdx;

    /*! Flow index to be opened*/
    uint32_t flowIdx;

    /*! UDMA specific channel params */
    EnetUdma_UdmaChPrms udmaChPrms;

    /*! Whether to use the shared global event or not. If set to false, a dedicated event
     *  will be used for this channel. */
    bool useGlobalEvt;

    /*! Enet UDMA event callback function - this function will be called when
     *  the registered packets are received on RX Flow*/
    EnetDma_PktNotifyCb notifyCb;

    /*! UDMA Flow params */
    EnetUdma_UdmaFlowPrms flowPrms;

    /*! Number of receive packets, used for allocating number of DMA descriptors
     *  Note - The HW ring element count field is 19-bit */
    uint32_t numRxPkts;

    /*! Flag to disable cache operations on the ring memory */
    bool disableCacheOpsFlag;

    /*! Maximum receive packet length for this flow. Make sure packets of at
     *  least this length are submitted in EnetDma_submitRxPktQ() */
    uint32_t rxFlowMtu;

    /*! Callback functions.
     *  Callback function typedefs so that the EnetUdma layer can call into the
     *  app layer and let it translate between the hardware buffer descriptors
     *  and packets and the stack/translation layer's buffers and packets  */

    /*! Ring memory allocation callback, this cannot be NULL */
    EnetUdma_AllocRingMemFxn ringMemAllocFxn;

    /*! Ring memory free function callback, used in close flow */
    EnetUdma_FreeRingMemFxn ringMemFreeFxn;

    /*! DMA HPD (host packet descriptor) memory allocation callback */
    EnetUdma_AllocDmaDescFxn dmaDescAllocFxn;

    /*! Transmit HPD (host packet descriptor) memory free callback */
    EnetUdma_FreeDmaDescFxn dmaDescFreeFxn;

    /*! Argument to be used for the callback routines (it should mean something
     *  to layer into which the callback calls) */
    void *cbArg;

    /*! Flag to indicate if a dedicated proxy should be allocated for the "FQ"
     *  for this flow.
     *  This should be enabled only for those flows that require very high
     *  throughput as there are limited proxy available in the system */
    bool useProxy;

    /*! Buffer auto-reclaim params*/
    EnetUdma_AutoReclaimPrms autoReclaimPrms;
} EnetUdma_OpenRxFlowPrms;

/*!
 * \brief Param struct for the RX channel open function. We include this typedef as top level
 *        DMA APIs use EnetDma_OpenRxChPrms struct.
 */
typedef EnetUdma_OpenRxFlowPrms EnetDma_OpenRxChPrms;

/*!
 * \brief Param struct for the TX channel open function.
 *
 * The configuration structure for the TX channel open function # EnetDma_openTxCh().
 */
typedef struct EnetUdma_OpenTxChPrms_s
{
    /*! UDMA driver handle*/
    Udma_DrvHandle hUdmaDrv;

    /*! UDMA channel number allocated for transmit.
     *  Set to #UDMA_DMA_CH_ANY if the channel to allocate and open could be
     *  any from the free pool */
    uint32_t chNum;

    /*! UDMA channel params */
    EnetUdma_UdmaChPrms udmaChPrms;

    /*! UDMA transmit channel params */
    EnetUdma_UdmaChTxPrms udmaTxChPrms;

    /*! Whether to use the shared global event or not. If set to false, a dedicated event
     *  will be used for this channel. */
    bool useGlobalEvt;

    /*! Enet UDMA event callback function - this function will be called when
     *  the registered packets are transmitted on TX channel */
    EnetDma_PktNotifyCb notifyCb;

    /*! Maximum number of transmit packets, used for allocating number of DMA descriptors
     *  Note - The HW ring element count field is 19-bit */
    uint32_t numTxPkts;

    /*! Flag to disable cache operations on the ring memory */
    bool disableCacheOpsFlag;

    /*! Callback functions.
     *  Callback function typedefs so that the EnetUdma layer can call into the
     *  app layer and let it translate between the hardware buffer descriptors
     *  and packets and the stack/translation layer's buffers and packets */

    /*! Ring memory allocation callback, this cannot be NULL */
    EnetUdma_AllocRingMemFxn ringMemAllocFxn;

    /*! Ring memory free function callback, used in close flow */
    EnetUdma_FreeRingMemFxn ringMemFreeFxn;

    /*! DMA HPD (host packet descriptor) memory allocation callback */
    EnetUdma_AllocDmaDescFxn dmaDescAllocFxn;

    /*! Transmit HPD (host packet descriptor) memory free callback */
    EnetUdma_FreeDmaDescFxn dmaDescFreeFxn;

    /*! Argument to be used for the callback routines (it should mean something
     *  to layer into which the callback calls) */
    void *cbArg;

    /*! Flag to indicate if a dedicated proxy should be allocated for the "FQ"
     *  for this flow.
     *  This should be enabled only for those flows that require very high
     *  throughput as there are limited proxy available in the system */
    bool useProxy;

    /*! Buffer auto-reclaim params*/
    EnetUdma_AutoReclaimPrms autoReclaimPrms;
} EnetUdma_OpenTxChPrms;

/*!
 * \brief Param struct for the RX channel open.
 *
 * The parameter structure for the RX channel open, containing a channel config
 * structure.
 */
typedef struct EnetUdma_RxChInitPrms_s
{
    /*! This field selects which scheduling bin the channel will be placed in
     *  for bandwidth allocation of the TX DMA units.
     *  Refer \ref tisci_msg_rm_udmap_tx_ch_cfg_req::tx_sched_priority */
    uint8_t dmaPriority;
} EnetUdma_RxChInitPrms;

/*!
 * \brief Config structure for Enet UDMA.
 *
 * The parameter structure for Enet UDMA configuration, containing a RX channel
 * config and NAVSS instance id.
 */
typedef struct EnetUdma_Cfg_s
{
    /*! UDMA driver handle*/
    Udma_DrvHandle hUdmaDrv;

    /*! RX channel configuration parameters */
    EnetUdma_RxChInitPrms rxChInitPrms;
} EnetUdma_Cfg;

/*!
 * \brief Config structure for Enet UDMA Data Path initialization.
 *
 * The parameter strcture for Enet UDMA data path init configuration.
 */
typedef struct EnetUdma_DmaCfg_s
{
    /*! UDMA driver handle*/
    Udma_DrvHandle hUdmaDrv;
} EnetDma_initCfg;

/*! @} */

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void EnetUdma_initDataPathParams(EnetDma_initCfg *pDmaConfig);
EnetDma_Handle EnetUdma_initDataPath(Enet_Type enetType,
                                     uint32_t instId,
                                     const EnetDma_initCfg *pDmaInitCfg);
int32_t EnetUdma_deInitDataPath(EnetDma_Handle hEnetUdma);

/*!
 * \brief Check if any packet loss in RX flow FQ and CQ rings.
 *
 * This development time error checks API checks DMA descriptor loss due to any
 * timing issues.  We check amount of descriptors with DMA and free matches the
 * number of allocated with some "margin" for packets in flight.
 *
 * Note: Margin can vary drastically based on Core (A72 vs R5), type of Udma
 * channel (NORMAL/HC etc.) and size of packet. This makes these API dependent
 * on application configuration.
 *
 * \param hRxFlow     [IN] flow handle for the flow we wish to check sanity
 * \param margin      [IN] Amount of packets which can be in flight in DMA
 *
 * \retval UDMA_SOK    Number of free descriptors and with DMA matches
 *                     (total allocated packets - margin) = (packets in CQ +
 *                     Packets in FQ)
 * \retval UDMA_EFAIL  Number offree descriptors and with DMA don't match
 */
int32_t EnetUdma_checkRxFlowSanity(EnetDma_RxChHandle hRxFlow,
                                   uint32_t margin);

/*!
 * \brief Check if any packet loss in TX Channel FQ and CQ rings.
 *
 * This development time error checks API checks dma desc loss due to any timing
 * issues. We check amount of descriptors with DMA and free matches the number
 * of allocated with some "margin" for packets in flight.
 *
 * Note: Margin can vary drastically based on Core (A72 vs R5), type of Udma
 * channel (NORMAL/HC etc.) and size of packet. This makes these API dependent
 * on application configuration.
 *
 * \param hTxCh       [IN] TX Channel handle for the TX channel we wish to check
 *                    sanity
 * \param margin      [IN] Amount of packets which can be in flight in DMA
 *
 * \retval UDMA_SOK    Number of free descriptors and with DMA matches (total
 *                     allocated packets - margin) = (packets in CQ + Packets
 *                     in FQ)
 * \retval UDMA_EFAIL  Number of free descriptors and with DMA don't match
 */
int32_t EnetUdma_checkTxChSanity(EnetDma_TxChHandle hTxCh,
                                uint32_t margin);

/*!
 * \brief Get TX channel FQ handle.
 *
 * Returns FQ handle for TX Channel. This function can be used when auto-recycling
 * of buffers is used.
 *
 * \param hTxCh     [IN] TX Channel handle
 *
 * \return Udma_RingHandle - Handle to FQ ring
 */
Udma_RingHandle EnetUdma_getTxChFqHandle(EnetDma_TxChHandle hTxCh);

/*!
 * \brief Get handle to DMA descriptor free pool for TX channel.
 *
 * Returns DMA descriptor free pool for TX channel. This function can be used when
 * auto-recycling of buffers is used.
 *
 * \param hTxCh     [IN] TX channel handle
 *
 * \return EnetUdma_DmaDescQHandle - Handle to DMA descriptor free pool
 */
EnetUdma_DmaDescQHandle EnetUdma_getTxChDescPoolHandle(EnetDma_TxChHandle hTxCh);

/*!
 * \brief Get RX flow FQ handle.
 *
 * Returns FQ handle for RX flow. This function can be used when auto-recycling
 * of buffers is used.
 *
 * \param hRxFlow     [IN] RX flow handle
 *
 * \return Udma_RingHandle - Handle to FQ ring
 */
Udma_RingHandle EnetUdma_getRxFlowFqHandle(EnetDma_RxChHandle hRxFlow);

/*!
 * \brief Get handle to DMA descriptor free pool for RX flow.
 *
 * Returns DMA descriptor free pool for RX flow. This function can be used when
 * auto-recycling of buffers is used.
 *
 * \param hRxFlow     [IN] RX flow handle
 *
 * \return EnetUdma_DmaDescQHandle - Handle to DMA descriptor free pool
 */
EnetUdma_DmaDescQHandle EnetUdma_getRxFlowDescPoolHandle(EnetDma_RxChHandle hRxFlow);

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

#endif /* ENET_UDMA_H_ */

/*! @} */
