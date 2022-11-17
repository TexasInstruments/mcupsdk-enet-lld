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
 * \file  cpsw.h
 *
 * \brief This file contains the type definitions and helper macros for the
 *        CPSW peripheral interface.
 *
 * This CPSW peripheral implementation supports CPSW_2G, CPSW_5G and CPSW_9G
 * found in SoCs of the Jacinto 7 family.
 */

/*!
 * \ingroup  DRV_ENET_PERS
 * \defgroup DRV_ENET_CPSW CPSW Peripheral
 *
 * CPSW peripheral supports CPSW_2G, CPSW_5G and CPSW_9G found in SoCs of the
 * Jacinto 7 family.
 *
 * Features:
 * - CPSW_FEATURE_INTERVLAN - InterVLAN feature.
 *
 * Compile-time configuration:
 * - #ENET_CFG_CPSW_SGMII - Q/SGMII support. It requires CPSW MAC port's
 *   #ENET_CFG_CPSW_MACPORT_SGMII compile-time flag.
 * - #ENET_CFG_CPSW_INTERVLAN - InterVLAN support. It requires CPSW MAC port's
 *   #ENET_CFG_CPSW_MACPORT_INTERVLAN compile-time flag.
 *
 * @{
 */

#ifndef CPSW_H_
#define CPSW_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <include/mod/cpsw_ale.h>
#include <include/mod/cpsw_cpts.h>
#include <include/mod/cpsw_hostport.h>
#include <include/mod/cpsw_macport.h>
#include <include/mod/mdio.h>
#include <include/mod/cpsw_stats.h>
#include <include/core/enet_mod_phy.h>
#include <include/core/enet_rm.h>
#include <include/core/enet_dma.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */
/*! \brief Helper macro to create CPSW IOCTL commands. */
#define CPSW_PER_PUBLIC_IOCTL(x)              (ENET_IOCTL_TYPE_PUBLIC | \
                                               ENET_IOCTL_PER_BASE | \
                                               ENET_IOCTL_PER_CPSW |    \
                                               ENET_IOCTL_MIN(x))

/*! Maximum number of MAC ports supported by this driver. */
#if defined(SOC_AM273X) || defined(SOC_AWR294X)
#define CPSW_MAC_PORT_NUM                     (1U)
#else
#define CPSW_MAC_PORT_NUM                     (8U)
#endif

/*!
 * \name CPSW InterVLAN ingress packet match types.
 *
 * Ingress packet match criteria for interVLAN routing.  Each packet match
 * criteria is represented by a bit.  To enable multiple match criteria create
 * bitmask ORing required ingress packet match criteria defines
 *  @{
 */

/*! \brief Enable classifier match with PORT number */
#define CPSW_INTERVLAN_INGRESSPKT_MATCH_PORT      (CPSW_ALE_POLICER_MATCH_PORT)

/*! \brief Enable classifier match with MAC source address entry */
#define CPSW_INTERVLAN_INGRESSPKT_MATCH_MACSRC    (CPSW_ALE_POLICER_MATCH_MACSRC)

/*! \brief Enable classifier match with MAC destination address entry */
#define CPSW_INTERVLAN_INGRESSPKT_MATCH_MACDST    (CPSW_ALE_POLICER_MATCH_MACDST)

/*! \brief Enable classifier match with MAC destination address entry */
#define CPSW_INTERVLAN_INGRESSPKT_MATCH_ETHERTYPE (CPSW_ALE_POLICER_MATCH_ETHERTYPE)

/*! \brief Enable classifier match with IPv4/IPv6 source address */
#define CPSW_INTERVLAN_INGRESSPKT_MATCH_IPSRC     (CPSW_ALE_POLICER_MATCH_IPSRC)

/*! \brief Enable classifier match with IPv4/IPv6 destination address */
#define CPSW_INTERVLAN_INGRESSPKT_MATCH_IPDST     (CPSW_ALE_POLICER_MATCH_IPDST)

/*!  @} */

/*! \brief CPSW statistics interrupt id. */
#define CPSW_INTR_STATS_PEND0                     (1U)

/*! \brief CPSW MDIO interrupt id. */
#define CPSW_INTR_MDIO_PEND                       (2U)

/*! \brief CPSW event pending interrupt (CPTS) id. */
#define CPSW_INTR_EVNT_PEND                       (3U)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief CPSW peripheral IOCTL commands.
 */
typedef enum Cpsw_Ioctl_e
{
    /*!
     * \brief Setup interVLAN route for a single egress port.
     *
     * IOCTL parameters:
     * -  inArgs: #Cpsw_SetInterVlanRouteUniEgressInArgs
     * - outArgs: #Cpsw_SetInterVlanRouteUniEgressOutArgs
     */
    CPSW_PER_IOCTL_SET_INTERVLAN_ROUTE_UNI_EGRESS = CPSW_PER_PUBLIC_IOCTL(0U),

    /*!
     * \brief Setup interVLAN route for a single egress port.
     *
     * IOCTL parameters:
     * -  inArgs: #Cpsw_ClearInterVlanRouteUniEgressInArgs
     * - outArgs: #CpswMacPort_InterVlanRouteId
     */
    CPSW_PER_IOCTL_CLEAR_INTERVLAN_ROUTE_UNI_EGRESS = CPSW_PER_PUBLIC_IOCTL(1U),

    /*!
     * \brief Setup interVLAN route for a multiple egress port.
     *
     * IOCTL parameters:
     * -  inArgs: #Cpsw_SetInterVlanRouteMultiEgressInArgs
     * - outArgs: #Cpsw_SetInterVlanRouteMultiEgressOutArgs
     */
    CPSW_PER_IOCTL_SET_INTERVLAN_ROUTE_MULTI_EGRESS = CPSW_PER_PUBLIC_IOCTL(2U),

    /*!
     * \brief Clear interVLAN route for a multiple egress port route.
     *
     * IOCTL parameters:
     * -  inArgs: #Cpsw_ClearInterVlanRouteMultiEgressInArgs
     * - outArgs: #CpswMacPort_InterVlanRouteId
     */
    CPSW_PER_IOCTL_CLEAR_INTERVLAN_ROUTE_MULTI_EGRESS = CPSW_PER_PUBLIC_IOCTL(3U),

    /*!
     * \brief Configure short inter-packet gap (IPG) for specific MAC ports.
     *
     * IOCTL parameters:
     * -  inArgs: #Cpsw_SetTxShortIpgCfgInArgs
     * - outArgs: None
     */
    CPSW_PER_IOCTL_SET_SHORT_IPG_CFG = CPSW_PER_PUBLIC_IOCTL(4U),

    /*!
     * \brief Get the current short inter-packet gap (IPG) configuration for
     *        all open MAC ports.
     *
     * IOCTL parameters:
     * -  inArgs: None
     * - outArgs: #Cpsw_TxShortIpgCfg
     */
    CPSW_PER_IOCTL_GET_SHORT_IPG_CFG = CPSW_PER_PUBLIC_IOCTL(5U),
} Cpsw_Ioctl;

/*!
 * \brief InterVLAN route ingress packet match criteria configuration.
 */
typedef struct Cpsw_InterVlanRouteIngressPktMatchCfg_s
{
    /*! Bitmask selecting classifier types to enable
     * @sa CPSW Intervlan ingress packet match types */
    uint32_t packetMatchEnMask;

    /*! Flag to enable TTL check.
     *  If enabled a packet whose TTL is zero will be routed to host port
     *  instead of being routed to egress port */
    bool ttlCheckEn;

    /*! Port number to match.
     *  Must be valid if bit #CPSW_INTERVLAN_INGRESSPKT_MATCH_PORT in
     *  packetMatchEnMask is set */
    Enet_MacPort ingressPort;

    /*! Source MAC  address to match.
     *  Must be valid if bit #CPSW_INTERVLAN_INGRESSPKT_MATCH_MACSRC in
     *  packetMatchEnMask is set */
    CpswAle_MacAddrClassifierInfo srcMacAddrInfo;

    /*! Destination MAC address to match.
     *  Must valid if bit #CPSW_INTERVLAN_INGRESSPKT_MATCH_MACDST in
     *  packetMatchEnMask is set */
    CpswAle_MacAddrClassifierInfo dstMacAddrInfo;

    /*! Ingress packet VLAN ID to match */
    uint32_t vlanId;

    /*! Source IP address to match.
     *  Must be valid if bit #CPSW_INTERVLAN_INGRESSPKT_MATCH_IPSRC in
     *  policerMatchEnMask is set */
    CpswAle_IpAddrClassifierInfo srcIpInfo;

    /*! Destination IP address to match.
     *  Must be valid if bit #CPSW_INTERVLAN_INGRESSPKT_MATCH_IPDST in
     *  policerMatchEnMask is set. */
    CpswAle_IpAddrClassifierInfo dstIpInfo;

    /*! Frame Ethertype to match.
     *  Must be valid if bit #CPSW_INTERVLAN_INGRESSPKT_MATCH_ETHERTYPE in
     *  policerMatchEnMask is set. */
    uint16_t etherType;
} Cpsw_InterVlanRouteIngressPktMatchCfg;

/*!
 * \brief InterVLAN egress port configuration.
 */
typedef struct Cpsw_InterVlanEgressPortCfg_s
{
    /*! Egress port for the interVLAN route */
    Enet_MacPort egressPort;

    /*! Egress packet modification for the interVLAN route.
     *  The packet that is routed to the egress port will be modified based on
     *  configuration specified in this parameter */
    CpswMacPort_InterVlanRoutingCfg outPktModCfg;
} Cpsw_InterVlanEgressPortCfg;

/*!
 * \brief Input args for #CPSW_PER_IOCTL_SET_INTERVLAN_ROUTE_UNI_EGRESS command.
 */
typedef struct Cpsw_SetInterVlanRouteUniEgressInArgs_s
{
    /*! Ingress packet match criteria for the interVLAN route.
     *  If the packet matches the given criteria and the destination port
     *  of the packet is host port then the interVLAN route will take effect */
    Cpsw_InterVlanRouteIngressPktMatchCfg inPktMatchCfg;

    /*! Egress packet modification configuration for the interVLAN route */
    Cpsw_InterVlanEgressPortCfg egressCfg;
} Cpsw_SetInterVlanRouteUniEgressInArgs;

/*!
 * \brief Output args for #CPSW_PER_IOCTL_SET_INTERVLAN_ROUTE_UNI_EGRESS command
 */
typedef struct Cpsw_SetInterVlanRouteUniEgressOutArgs_s
{
    /*! Egress port route index used for the interVLAN route */
    CpswMacPort_InterVlanRouteId egressPortRouteId;

    /*! Ingress packet match classifier info */
    CpswAle_PolicerEntryOutArgs ingressPacketClassifierInfo;
} Cpsw_SetInterVlanRouteUniEgressOutArgs;

/*!
 * \brief Input args for #CPSW_PER_IOCTL_CLEAR_INTERVLAN_ROUTE_UNI_EGRESS command
 */
typedef struct Cpsw_ClearInterVlanRouteUniEgressInArgs_s
{
    /*! Ingress packet match criteria for the interVLAN route.
     *  If the packet matches the given criteria and the destination port
     *  of the packet is host port then the interVLAN route will take effect */
    Cpsw_InterVlanRouteIngressPktMatchCfg inPktMatchCfg;

    /*! Egress packet modification configuration for the interVLAN route */
    Cpsw_InterVlanEgressPortCfg egressCfg;

    /*! Bitmask to indicate which ALE entries associated with the ingress
     *  packet match classifier needs to be deleted @sa InterVLAN ingress
     *  packet match aleEntry delete bitmask */
    uint32_t delAleEntryMask;
} Cpsw_ClearInterVlanRouteUniEgressInArgs;

/*!
 * \brief Input args for #CPSW_PER_IOCTL_SET_INTERVLAN_ROUTE_MULTI_EGRESS command.
 */
typedef struct Cpsw_SetInterVlanRouteMultiEgressInArgs_s
{
    /*! Number of egress ports the packet needs to be routed to */
    uint32_t numEgressPorts;

    /*! Egress packet modification configuration for the interVLAN route for
     *  each port the packet will be routed to.
     *  #numEgressPorts determine number of valid entries in this table */
    Cpsw_InterVlanEgressPortCfg egressCfg[CPSW_ALE_NUM_MAC_PORTS];

    /*! Ingress packet match criteria for the interVLAN route.
     *  If the packet matches the given criteria and the destination port
     *  of the packet is host port then the interVLAN route will take effect */
    Cpsw_InterVlanRouteIngressPktMatchCfg inPktMatchCfg;
} Cpsw_SetInterVlanRouteMultiEgressInArgs;

/*!
 * \brief Output args for #CPSW_PER_IOCTL_SET_INTERVLAN_ROUTE_MULTI_EGRESS command.
 */
typedef struct Cpsw_SetInterVlanRouteMultiEgressOutArgs_s
{
    /*! Egress port route index used for the interVLAN route */
    CpswMacPort_InterVlanRouteId egressPortRouteId;

    /*! Ingress packet match classifier info */
    CpswAle_PolicerEntryOutArgs ingressPacketClassifierInfo;
} Cpsw_SetInterVlanRouteMultiEgressOutArgs;

/*!
 * \brief Input args for #CPSW_PER_IOCTL_CLEAR_INTERVLAN_ROUTE_MULTI_EGRESS command.
 */
typedef struct Cpsw_ClearInterVlanRouteMultiEgressInArgs_s
{
    /*! Number of egress ports the packet needs to be routed to */
    uint32_t numEgressPorts;

    /*! Egress packet modification configuration for the interVLAN route for
     *  each port the packet will be routed to.
     *  #numEgressPorts determine number of valid entries in this table */
    Cpsw_InterVlanEgressPortCfg egressCfg[CPSW_ALE_NUM_MAC_PORTS];

    /*! Ingress packet match criteria for the interVLAN route.
     *  If the packet matches the given criteria and the destination port
     *  of the packet is host port then the interVLAN route will take effect */
    Cpsw_InterVlanRouteIngressPktMatchCfg inPktMatchCfg;

    /*! Bitmask to indicate which ALE entries associated with the
     *  ingress packet match classifier needs to be deleted
     *  @sa InterVLAN ingress packet match aleEntry delete bitmask */
    uint32_t delAleEntryMask;
} Cpsw_ClearInterVlanRouteMultiEgressInArgs;

/*!
 * \brief Input args for CPSW_PER_IOCTL_SET_SHORT_IPG_CFG.
 */
typedef struct Cpsw_SetTxShortIpgCfgInArgs_s
{
    /*! Flag to indicate if the IPG trigger threshold has to be configured
     *  or not. If set to FALSE, the parameter #ipgTriggerThreshBlkCnt
     *  will be ignored */
    bool configureGapThresh;

    /*! Ethernet port associated FIFO transmit block usage value for triggering
     *  transmit short gap when short gap is enabled */
    uint32_t ipgTriggerThreshBlkCnt;

    /*! Number of MAC ports to configure TX IPG */
    uint32_t numMacPorts;

    /*! Per Port short TX IPG configuration.
     *  Up to #numMacPorts entries should be populated */
    CpswMacPort_PortTxShortIpgCfg portShortIpgCfg[CPSW_MAC_PORT_NUM];
} Cpsw_SetTxShortIpgCfgInArgs;

/*!
 * \brief Short IPG configuration for CPSW IP.
 *
 * It's used as output args for CPSW_PER_IOCTL_GET_SHORT_IPG_CFG command.
 */
typedef struct Cpsw_TxShortIpgCfg_s
{
    /*! Ethernet port associated FIFO transmit block usage value for triggering
     *  transmit short gap when short gap is enabled */
    uint32_t ipgTriggerThreshBlkCnt;

    /*! Number of MAC ports to configure TX IPG */
    uint32_t numMacPorts;

    /*! Per Port short TX IPG configuration.
     *  Up to #numMacPorts entries should be populated */
    CpswMacPort_PortTxShortIpgCfg portShortIpgCfg[CPSW_MAC_PORT_NUM];
} Cpsw_TxShortIpgCfg;

/*!
 * \brief CPSW-level VLAN configuration.
 */
typedef struct Cpsw_VlanCfg_s
{
    /*! Whether in VLAN aware or unaware mode */
    bool vlanAware;

    /*! VLAN customer or service switch */
    Enet_VlanTagType vlanSwitch;

    /*! Outer VLAN ltype */
    uint16_t outerVlan;

    /*! Inner VLAN ltype */
    uint16_t innerVlan;
} Cpsw_VlanCfg;

/*!
 * \brief PHY link status change event information.
 */
typedef struct Cpsw_MdioLinkStateChangeInfo_s
{
    /*! PHY address */
    uint32_t phyAddr;

    /*! Whether PHY alive status changed.
     *  Applicable only for Status Change Mode */
    bool aliveChanged;

    /*! Whether PHY is alive or not.
     *  Applicable only for Status Change Mode */
    bool isAlive;

    /*! Whether PHY link status changed */
    bool linkChanged;

    /*! Whether PHY is linked or not */
    bool isLinked;

} Cpsw_MdioLinkStateChangeInfo;

/*!
 * \brief CPSW PHY link state change callback function.
 *
 * Callback for PHY link state change interrupt (MDIO_LINKINT).
 * This callback is invoked from interrupt context.
 */
typedef void (*Cpsw_MdioLinkStateChangeCb)(Cpsw_MdioLinkStateChangeInfo *info,
                                           void *appArg);

/*!
 * \brief CPSW port link status change callback function.
 *
 * Callback for port link state change event.  This callback is invoked when
 * port link is fully functional (PHY linked, ALE ports enabled, etc) or
 * when port link is fully shutdown (ALE ports disabled).
 */
typedef void (*Cpsw_PortLinkStatusChangeCb)(Enet_MacPort macPort,
                                            bool isLinkUp,
                                            void *appArg);

/*!
 * \brief CPSW configuration.
 *
 * Configuration information for the CPSW driver.
 */
typedef struct Cpsw_Cfg_s
{
    /*! Escalate priority load value */
    uint32_t escalatePriorityLoadVal;

    /*! Configuration of the CPSW DMA */
    const void *dmaCfg;

    /*! VLAN configuration (inner/outer VLAN ltype, customer/service switch) */
    Cpsw_VlanCfg vlanCfg;

    /*! Max packet length transmitted on egress. Packets that are larger than
     *  this length will be dropped. This length excludes VLAN addition or
     *  removal */
    uint32_t txMtu[ENET_PRI_NUM];

    // /*! Configuration of the CPSW DMA */
    // EnetDma_Cfg dmaCfg;

    /*! Configuration of the host (CPPI) port */
    CpswHostPort_Cfg hostPortCfg;

    /*! Configure of the ALE module */
    CpswAle_Cfg aleCfg;

    /*! Configure of the CPTS module */
    CpswCpts_Cfg cptsCfg;

    /*! Configuration of the MDIO module */
    Mdio_Cfg mdioCfg;

    /*! Configuration of CPSW Resource Partition */
    EnetRm_ResCfg resCfg;

    /*! Interrupt priority */
    uint32_t intrPriority;

    /*! MDIO Link state change callback function pointer */
    Cpsw_MdioLinkStateChangeCb mdioLinkStateChangeCb;

    /*! Application data to be passed to the MDIO link state change callback */
    void *mdioLinkStateChangeCbArg;

    /*! Port link status change callback function pointer.  This callback is
     *  called when the port link is either fully functional or fully shutdown */
    Cpsw_PortLinkStatusChangeCb portLinkStatusChangeCb;

    /*! Application data to be passed to the port link status change callback */
    void *portLinkStatusChangeCbArg;

    /*! QSGMII0 Running Disparity Check (RDCD) enable - disables the rx running
     *  disparity so that errors do not propagate across lanes.
     *  It can be used when debugging a multi-port link to disable the disparity
     *  to isolate the receive errors */
    bool enableQsgmii0RDC;

    /*! QSGMII1 Running Disparity Check (RDCD) enable */
    bool enableQsgmii1RDC;

    /*! Disable Enet LLD PHY driver - Disables use on PHY driver inside the
     *  Enet LLD. All PHY functionality including PHY state machine is bypassed
     *  Application will use this mode if ethernet PHY is managed outside the Enet LLD
     *  Application is responsible for PHY management. Application can register 
     *  with Enet LLD to get mdioLinkStateChangeCb callback.
     *  Application _must_ use Enet LLD IOCTLs to access MDIO as MDIO ownership 
     *  is still with Enet LLD and there should not be multiple masters for the
     *  MDIO peripheral
     */
    bool disablePhyDriver;

} Cpsw_Cfg;


/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

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

#endif /* CPSW_H_ */

/*! @} */
