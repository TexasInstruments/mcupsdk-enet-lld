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
 * \file  cpsw_macport.h
 *
 * \brief This file contains the type definitions and helper macros for the
 *        CPSW MAC port module interface.
 */

/*!
 * \ingroup  ENET_MOD_MACPORT
 * \defgroup CPSW_MACPORT_MOD CPSW MAC Port
 *
 * The CPSW MAC port module provides additional IOCTL commands than those
 * supported by the generic \ref ENET_MOD_MACPORT API set.
 *
 * CPSW MAC port clocks:
 * - CPSW_CPPI_CLK - CPSW main clock.
 *
 * Features:
 * - CPSW_MACPORT_FEATURE_SGMII - Q/SGMII feature.
 * - CPSW_MACPORT_FEATURE_INTERVLAN - InterVLAN feature.
 *
 * Compile-time configuration:
 * - #ENET_CFG_CPSW_HOSTPORT_TRAFFIC_SHAPING - Traffic shaping (rate limit) support.
 * - #ENET_CFG_CPSW_MACPORT_INTERVLAN - InterVLAN support.
 * - #ENET_CFG_CPSW_MACPORT_SGMII - Q/SGMII support.
 *
 * @{
 */

#ifndef CPSW_MACPORT_H_
#define CPSW_MACPORT_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <include/core/enet_mod_macport.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/*! \brief Helper macro to create IOCTL commands for CPSW MAC port module. */
#define CPSW_MACPORT_PUBLIC_IOCTL(x)          (ENET_IOCTL_TYPE_PUBLIC |  \
                                               ENET_IOCTL_MACPORT_BASE | \
                                               ENET_IOCTL_PER_CPSW |     \
                                               ENET_IOCTL_MIN(x))

/*! \brief EST non-zero minimum time interval, in wireside clocks. Needed to
 *  guarantee that next fetch value has time to be fetched before current
 *  fetch count is over. */
#define CPSW_MACPORT_EST_TIME_INTERVAL_MIN    (0x010U)

/*! \brief EST maximum time interval (14-bit), in wireside clocks. */
#define CPSW_MACPORT_EST_TIME_INTERVAL_MAX    (0x3FFFU)

/*! \brief Time interval step in nsecs for 1 Gbps link. */
#define CPSW_MACPORT_EST_TIME_STEP_1G         (8U)

/*! \brief Time interval step in nsecs for 100 Mbps link. */
#define CPSW_MACPORT_EST_TIME_STEP_100M       (40U)

/*! \brief Time interval step in nsecs for a 10 Mbps link. */
#define CPSW_MACPORT_EST_TIME_STEP_10M        (400U)

/*! \brief Min time interval in nsecs for given link speed. */
#define CPSW_MACPORT_EST_TIME_MIN(speed)      (CPSW_MACPORT_EST_TIME_STEP_##speed * CPSW_MACPORT_EST_TIME_INTERVAL_MIN)

/*! \brief Max time interval in nsecs for given link speed. */
#define CPSW_MACPORT_EST_TIME_MAX(speed)      (CPSW_MACPORT_EST_TIME_STEP_##speed * CPSW_MACPORT_EST_TIME_INTERVAL_MAX)

/*! \brief EST allow count scaling factor for 1 Gbps link. */
#define CPSW_MACPORT_EST_ALLOWCNT_FACTOR_1G   (1U)

/*! \brief EST allow count scaling factor for 100 Mbps link. */
#define CPSW_MACPORT_EST_ALLOWCNT_FACTOR_100M (2U)

/*! \brief EST allow count scaling factor for 10 Mbps link. */
#define CPSW_MACPORT_EST_ALLOWCNT_FACTOR_10M  (2U)

/*! \brief Guard band duration in nsecs for given link speed and
 *  maximum packet size in previous time interval. */
#define CPSW_MACPORT_EST_GUARD_BAND(maxPktSize, speed)  (((((maxPktSize) + 4U) * \
                                                           CPSW_MACPORT_EST_ALLOWCNT_FACTOR_##speed) + 292U) * \
                                                         CPSW_MACPORT_EST_TIME_STEP_##speed)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief CPSW MAC port IOCTL commands.
 */
typedef enum CpswMacPort_Ioctl_s
{
    /*!
     * \brief Get MAC port FIFO statistics.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetMacPort_GenericInArgs
     * - outArgs: #CpswMacPort_FifoStats
     */
    CPSW_MACPORT_IOCTL_GET_FIFO_STATS = CPSW_MACPORT_PUBLIC_IOCTL(0U),

    /*!
     * \brief Enable Ethernet port CPTS event.
     *
     * IOCTL parameters:
     * -  inArgs: #CpswMacPort_EnableTsEventInArgs
     * - outArgs: None
     */
    CPSW_MACPORT_IOCTL_ENABLE_CPTS_EVENT = CPSW_MACPORT_PUBLIC_IOCTL(1U),

    /*!
     * \brief Disable Ethernet port CPTS event.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetMacPort_GenericInArgs
     * - outArgs: None
     */
    CPSW_MACPORT_IOCTL_DISABLE_CPTS_EVENT = CPSW_MACPORT_PUBLIC_IOCTL(2U),

    /*!
     * \brief Enable EST packet timestamping functionality.
     *
     * Once enabled, timestamps can be retrieved using CPSW CPTS IOCTLs.
     *
     * IOCTL parameters:
     *   inArgs: #CpswMacPort_EstTimestampCfg
     *  outArgs: None
     */
    CPSW_MACPORT_IOCTL_EST_ENABLE_TIMESTAMP = CPSW_MACPORT_PUBLIC_IOCTL(3U),

    /*!
     * \brief Disable EST packet timestamping functionality.
     *
     * IOCTL parameters:
     *   inArgs: #EnetMacPort_GenericInArgs
     *  outArgs: None
     */
    CPSW_MACPORT_IOCTL_EST_DISABLE_TIMESTAMP = CPSW_MACPORT_PUBLIC_IOCTL(4U),
} CpswMacPort_Ioctl;

/*!
 * \brief Configuration of transmit short inter packet gap feature.
 */
typedef struct CpswMacPort_TxShortIpgCfg_s
{
    /*! Transmit short gap enable.
     *  When set this flag causes transmit with a short IPG when the TX FIFO
     *  length for the port exceeds the configured CPSW gap threshold. */
    bool txShortGapEn;

    /*! Transmit short gap limit enable.
     *  When set this flag limits the number of short gap packets transmitted to
     *  100ppm.  Each time a short gap packet is sent, a counter is loaded with
     *  10,000 and decremented on each wireside clock.  Another short gap packet
     *  will not be sent out until the counter decrements to zero.
     *  This mode is included to preclude the host from filling up the FIFO and
     *  sending every packet out with short gap which would violate the maximum
     *  number of packets per second allowed.
     *
     *  This flag is used only with GMII (not XGMII). */
    bool txShortGapLimitEn;
} CpswMacPort_TxShortIpgCfg;

/*!
 * \brief MAC port's short gap configuration.
 */
typedef struct CpswMacPort_PortTxShortIpgCfg_s
{
    /*! Port number */
    Enet_MacPort macPort;

    /*! Short inter-packet gap configuration */
    CpswMacPort_TxShortIpgCfg shortIpgCfg;
} CpswMacPort_PortTxShortIpgCfg;

/*!
 * \brief MAC port interVLAN route identifier.
 */
typedef enum CpswMacPort_InterVlanRouteId_e
{
    /*! First interVLAN route Id */
    CPSW_MACPORT_INTERVLAN_ROUTEID_FIRST,

    /*! InterVLAN route Id 1 */
    CPSW_MACPORT_INTERVLAN_ROUTEID_1    = CPSW_MACPORT_INTERVLAN_ROUTEID_FIRST,

    /*! InterVLAN route Id 2 */
    CPSW_MACPORT_INTERVLAN_ROUTEID_2,

    /*! InterVLAN route Id 3 */
    CPSW_MACPORT_INTERVLAN_ROUTEID_3,

    /*! InterVLAN route Id 4 */
    CPSW_MACPORT_INTERVLAN_ROUTEID_4,

    /*! Last interVLAN route Id */
    CPSW_MACPORT_INTERVLAN_ROUTEID_LAST = CPSW_MACPORT_INTERVLAN_ROUTEID_4
} CpswMacPort_InterVlanRouteId;

/*!
 * \brief Port interVLAN configuration parameters.
 */
typedef struct CpswMacPort_InterVlanRoutingCfg_s
{
    /*! Destination address */
    uint8_t dstAddr[ENET_MAC_ADDR_LEN];

    /*! Source address */
    uint8_t srcAddr[ENET_MAC_ADDR_LEN];

    /*! VLAN Id */
    uint32_t vlanId;

    /*! Whether routed packet's destination address will be replaced with
     *  dstAddr and packet's source address will be replaced with srcAddr */
    bool replaceDASA;

    /*! Destination VLAN Force Untagged Egress. When set, this bit indicates
     *  that the VLAN should be removed on egress for the routed packet. */
    bool forceUntaggedEgress;

    /*! Decrement Time To Live.
     *  When set, the Time To Live (TTL) field in the header is decremented:
     *  - IPV4 - Decrement the TTL byte and update the Header Checksum
     *  - IPV6 - Decrement the Hop Limit
     *
     *  Note: When this bit is set, the ALE should be configured to send any
     *  IPv4/6 packet with a zero or one TTL field to the host.
     *  When this bit is cleared the TTL/Hop Limit fields are not checked or
     *  modified. */
    bool decrementTTL;
} CpswMacPort_InterVlanRoutingCfg;

/*!
 * \brief FIFO related statistics of a MAC port.
 */
typedef struct CpswMacPort_FifoStats_s
{
    /*! Max throughput of the Ethernet ports to the crossbar SCR */
    uint32_t rxThroughputRate;

    /*! Number of 32-byte packet words in an Ethernet port transmit FIFO before
     *  packet egress will begin */
    uint32_t txStartWords;

    /*! Max number of blocks allowed on all TX FIFO priorities combined */
    uint32_t txMaxBlocks;

    /*! Max number of blocks allowed on the express and preempt RX FIFOs
     *  combined */
    uint32_t rxMaxBlocks;

    /*! Number of blocks allocated to the FIFO logical TX queues */
    uint32_t txBlockCount;

    /*! Number of blocks allocated to the FIFO express RX queue */
    uint32_t rxExpressBlockCount;

    /*! Number of blocks allocated to the FIFO preempt RX queue */
    uint32_t rxPreemptBlockCount;

    /*! Whether a FIFO priority has one or more queued packets or not */
    bool txActiveFifo[ENET_PRI_NUM];
} CpswMacPort_FifoStats;

/*!
 * \brief Port IP configuration for time synchronization events.
 *
 * Common configuration for Annex D (IPv4) and Annex E (IPv6) time sync events.
 */
typedef struct CpswMacPort_IpTsCfg_s
{
    /*! Time Sync Time to Live Non-zero enable */
    bool ttlNonzeroEn;

    /*! Time Sync Unicast enable */
    bool unicastEn;

    /*! Time Sync Destination IP Address 129 enable */
    bool tsIp129En;

    /*! Time Sync Destination IP Address 130 enable */
    bool tsIp130En;

    /*! Time Sync Destination IP Address 131 enable */
    bool tsIp131En;

    /*! Time Sync Destination IP Address 132 enable */
    bool tsIp132En;

    /*! Time Sync Destination IP Address 107 enable */
    bool tsIp107En;

    /*! Time Sync Destination port number 319 enable */
    bool tsPort319En;

    /*! Time Sync Destination port number 320 enable */
    bool tsPort320En;
} CpswMacPort_IpTsCfg;

/*!
 * \brief Port configuration for time synchronization.
 *
 * Configuration information for enabling Ethernet RX and TX time sync events
 * in a port.
 */
typedef struct CpswMacPort_TsEventCfg_s
{
    /*! Enable Annex D (IPv4) TX Time Synchronization */
    bool txAnnexDEn;

    /*! Enable Annex D (IPv4) RX Time Synchronization */
    bool rxAnnexDEn;

    /*! Enable Annex E (IPv6) TX Time Synchronization */
    bool txAnnexEEn;

   /*!  Enable Annex E (IPv6) RX Time Synchronization */
    bool rxAnnexEEn;

    /*! Enable Annex F (IEEE802.3) TX Time Synchronization */
    bool txAnnexFEn;

    /*! Enable Annex F (IEEE802.3) RX Time Synchronization */
    bool rxAnnexFEn;

    /*! Enable Time Sync transmit host timestamp */
    bool txHostTsEn;

    /*! TX VLAN Type */
    EnetMacPort_VlanType txVlanType;

    /*! RX VLAN Type */
    EnetMacPort_VlanType rxVlanType;

    /*! VLAN Ltype 1 and 2 are common for both TX and RX and are valid only if
     *  #txVlanType or #rxVlanType is not #ENET_MACPORT_VLAN_TYPE_NONE */
    uint32_t vlanLType1;

    /*! VLAN Ltype 1 and 2 are common for both TX and RX and are valid only if
     *  #txVlanType or #rxVlanType is not #ENET_MACPORT_VLAN_TYPE_NONE */
    uint32_t vlanLType2;

    /*! Common configuration for Annex D (IPv4) and Annex E (IPv6) */
    CpswMacPort_IpTsCfg commonPortIpCfg;

    /*! Multicast type value only for Annex E (IPv6) */
    uint32_t mcastType;

    /*! Message type value mask is only for Annex F (PTP) */
    uint32_t messageType;

    /*! Ltype 2 is only for Annex F (PTP) */
    bool ltype2En;

    /*! Time Sync sequence ID offset */
    uint32_t seqIdOffset;

    /*! Time Sync domain offset */
    uint32_t domainOffset;
} CpswMacPort_TsEventCfg;

/*!
 * \brief Input args for #CPSW_MACPORT_IOCTL_ENABLE_CPTS_EVENT.
 */
typedef struct CpswMacPort_EnableTsEventInArgs_s
{
    /*! Port number */
    Enet_MacPort macPort;

    /*! Time sync configuration */
    CpswMacPort_TsEventCfg tsEventCfg;
} CpswMacPort_EnableTsEventInArgs;

/*!
 * \brief EST timestamping modes.
 */
typedef enum CpswMacPort_EstTimestampMode_e
{
    /*! Timestamp all express packets on any priority. */
    CPSW_MACPORT_EST_TIMESTAMP_ALL,

    /*! Timestamp only express packets of a given priority. */
    CPSW_MACPORT_EST_TIMESTAMP_ONEPRI,

    /*! Timestamp the first express packet in each time interval. */
    CPSW_MACPORT_EST_TIMESTAMP_FIRST,

    /*! Timestamp the first express packet of a given priority in the time interval. */
    CPSW_MACPORT_EST_TIMESTAMP_FIRST_ONEPRI,

} CpswMacPort_EstTimestampMode;

/*!
 * \brief EST timestamping configuration parameters.
 */
typedef struct CpswMacPort_EstTimestampCfg_s
{
    /*! Port number. */
    Enet_MacPort macPort;

    /*! Timestamping mode. Determines what packets are timestamped. */
    CpswMacPort_EstTimestampMode mode;

    /*! Priority that timestamps will be generated on. Applicable for
     *  \ref CPSW_MACPORT_EST_TIMESTAMP_ONEPRI and
     *  \ref CPSW_MACPORT_EST_TIMESTAMP_FIRST_ONEPRI. */
    uint8_t priority;

    /*! CPTS EST timestamp domain.  It can be used to indicate and identify that an
     *  event came from EST. */
    uint8_t domain;
} CpswMacPort_EstTimestampCfg;

/*!
 * \brief MAC port module configuration parameters.
 */
typedef struct CpswMacPort_Cfg_s
{
    /*! Whether loopback mode is enabled or not */
    bool loopbackEn;

    /*! Type of CRC */
    Enet_CrcType crcType;

    /*! Max length of a received frame on ingress. This max length includes
     *  VLAN */
    uint32_t rxMtu;

    /*! Whether priority tagged packets should be passed unchanged (if set to
     *  true) or replaced with port's VID (if set to false) */
    bool passPriorityTaggedUnchanged;

    /*! Port VLAN configuration */
    EnetPort_VlanCfg vlanCfg;

    /*! Egress priority type */
    EnetPort_EgressPriorityType txPriorityType;

    /*! SGMII mode. Applicable only when port is used in Q/SGMII mode */
    EnetMac_SgmiiMode sgmiiMode;
} CpswMacPort_Cfg;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief Initialize CPSW MAC port configuration parameters.
 *
 * \param macPortCfg  Configuration parameters to be initialized
 */
void CpswMacPort_initCfg(CpswMacPort_Cfg *macPortCfg);

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

#endif /* CPSW_MACPORT_H_ */

/*! @} */
