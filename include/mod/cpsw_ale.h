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
 * \file  cpsw_ale.h
 *
 * \brief This file contains the type definitions and helper macros for the
 *        CPSW Address Lookup Engine (ALE) module interface.
 */

/*!
 * \ingroup  ENET_MOD_FDB
 * \defgroup CPSW_ALE_MOD CPSW Address Lookup Engine (ALE)
 *
 * The CPSW ALE module provides low level access to switch configuration.
 *
 * The following features are supported:
 * - VLAN aware or unaware switching including unknown VLAN and null VLAN controls.
 * - Auto-learn on a per VLAN and per port bases.
 * - Manual or fully automatic aging of agable addresses.
 * - STP-required forwarding state controls and status.
 * - MACSEC forwarding controls and status.
 * - Port and VLAN, ONU, or address traffic mirroring.
 * - Port trunking for up to four trunks across any port combination.
 * - Multicast ranging for up to 1024 addresses per ALE table entry.
 * - Overlapping multicast ranges.
 * - Packet classification based on port, priority, ONU, DA, SA, S-VLAN,
 *   C-VLAN, EtherType, IP DA and/or IP SA combination with full CIDR
 *   masking support for IPv4 and IPv6 addresses.
 * - Policing of ingress data flows based on any packet classification.
 * - Host thread mapping based on any packet classification.
 *
 * @{
 */

#ifndef CPSW_ALE_H_
#define CPSW_ALE_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <enet_cfg.h>
#include <include/core/enet_mod_fdb.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/*! \brief Helper macro to create IOCTL commands for ALE module. */
#define CPSW_ALE_PUBLIC_IOCTL(x)              (ENET_IOCTL_TYPE_PUBLIC | \
                                               ENET_IOCTL_FDB_BASE |    \
                                               ENET_IOCTL_PER_CPSW |    \
                                               ENET_IOCTL_MIN(x))

/*! \brief Helper macro to create IOCTL commands for ALE module. */
#define CPSW_ALE_PRIVATE_IOCTL(x)              (ENET_IOCTL_TYPE_PRIVATE | \
                                                ENET_IOCTL_FDB_BASE |    \
                                                ENET_IOCTL_PER_CPSW |    \
                                                ENET_IOCTL_MIN(x))

/*! \brief Number of external ports in the subsystem. */
#define CPSW_ALE_NUM_MAC_PORTS                (CPSW_ALE_NUM_PORTS - 1U)

/*! \brief Maximum IP next header whitelist. */
#define CPSW_ALE_MAX_WHITELIST_IP_NXT_HDR     (4U)

/*! \brief Mask value for the all the ports. */
#define CPSW_ALE_ALL_PORTS_MASK               ((uint32_t)((1U << CPSW_ALE_NUM_PORTS) - 1U))

/*! \brief Mask value for the all MAC ports. */
#define CPSW_ALE_ALL_MACPORTS_MASK            ((uint32_t)(((1U << CPSW_ALE_NUM_PORTS) - 1U) - 1U))

/*! \brief Mask value for the host port. */
#define CPSW_ALE_HOST_PORT_MASK               (ENET_BIT(0))

/*! \brief Port number for the host port. */
#define CPSW_ALE_HOST_PORT_NUM                (0U)

/*! \brief Base index for MAC ports in ALE context */
#define CPSW_ALE_MACPORT_BASE                 (1U)

/*! \brief Macro to convert MAC port (#Enet_MacPort) to ALE port number */
#define CPSW_ALE_MACPORT_TO_ALEPORT(macPortNum)      (ENET_MACPORT_NORM(macPortNum) + CPSW_ALE_MACPORT_BASE)

/*! \brief Macro to convert ALE port number to MAC port number(#Enet_MacPort) */
#define CPSW_ALE_ALEPORT_TO_MACPORT(alePortNum)      (ENET_MACPORT_DENORM(alePortNum - CPSW_ALE_MACPORT_BASE))

/*! \brief Macro to convert MAC port (#Enet_MacPort) to ALE port mask. */
#define CPSW_ALE_MACPORT_TO_PORTMASK(macPort)        (ENET_BIT(CPSW_ALE_MACPORT_TO_ALEPORT(macPort)))

/*! \brief Maximum number of ignore bits in multicast address. */
#define CPSW_ALE_MCAST_IGN_BITS_MAX           (10U)

/*! \brief Disable peak bit rate. */
#define CPSW_ALE_PEAKBITRATE_DISABLE          (0U)

/*! \brief Disable commit bit rate. */
#define CPSW_ALE_COMMITBITRATE_DISABLE        (0U)

/*! \brief ALE invalid thread id. */
#define CPSW_ALE_THREADID_INVALID             (~0U)

/*!
 * \name ALE configuration definitions.
 *
 * ALE mode of operation configuration options.  Each configuration option
 * is represented by a bit in a configuration bitmask.
 *
 * @{
 */

/*! \brief Enable ALE. */
#define CPSW_ALE_CFG_MODULE_EN                (ENET_BIT(0U))

/*! \brief Enable ALE bypass. */
#define CPSW_ALE_CFG_BYPASS_EN                (ENET_BIT(1U))

/*! \brief Enable Unknown unicast packet flooding to host port. */
#define CPSW_ALE_CFG_UNKNOWN_UCAST_FLOOD2HOST (ENET_BIT(2U))

/*! @} */

/*!
 * \name ALE classifier match types.
 *
 * Classifier match type supported by ALE. Each type is represented by a bit
 * in a bitmask. To enable multiple classifiers in a single classifier entry,
 * create bitmask ORing required classifier match type defines.
 *
 * @{
 */

/*! \brief Enable classifier match with port number. */
#define CPSW_ALE_POLICER_MATCH_PORT           (ENET_BIT(0U))

/*! \brief Enable classifier match with received packet priority. */
#define CPSW_ALE_POLICER_MATCH_PRIORITY       (ENET_BIT(1U))

/*! \brief Enable classifier match with OUI portion of source MAC address. */
#define CPSW_ALE_POLICER_MATCH_OUI            (ENET_BIT(2U))

/*! \brief Enable classifier match with MAC destination address entry with/without VLAN. */
#define CPSW_ALE_POLICER_MATCH_MACDST         (ENET_BIT(3U))

/*! \brief Enable classifier match with MAC source address entry with/without VLAN. */
#define CPSW_ALE_POLICER_MATCH_MACSRC         (ENET_BIT(4U))

/*! \brief Enable classifier match with outer VLAN entry. */
#define CPSW_ALE_POLICER_MATCH_OVLAN          (ENET_BIT(5U))

/*! \brief Enable classifier match with inner VLAN entry. */
#define CPSW_ALE_POLICER_MATCH_IVLAN          (ENET_BIT(6U))

/*! \brief Enable classifier match with EtherType. */
#define CPSW_ALE_POLICER_MATCH_ETHERTYPE      (ENET_BIT(7U))

/*! \brief Enable classifier match with IPv4/IPv6 source address. */
#define CPSW_ALE_POLICER_MATCH_IPSRC          (ENET_BIT(8U))

/*! \brief Enable classifier match with IPv4/IPv6 destination address. */
#define CPSW_ALE_POLICER_MATCH_IPDST          (ENET_BIT(9U))

/*! @} */

/*!
 * \name ALE classifier delete bitmask.
 *
 * Associated with some classifier types are ALE table entries. When deleting
 * the classifier entry application needs to specify by means of below bitmask
 * if the ALE entry associated with the classifier should also be deleted.
 * If not set the ALE entry will remain.
 *
 * @{
 */

/*! \brief Delete OUI ALE table entry. */
#define CPSW_ALE_POLICER_TABLEENTRY_DELETE_OUI       (CPSW_ALE_POLICER_MATCH_PRIORITY)

/*! \brief Delete MAC source address ALE table entry. */
#define CPSW_ALE_POLICER_TABLEENTRY_DELETE_MACSRC    (CPSW_ALE_POLICER_MATCH_MACSRC)

/*! \brief Delete MAC destination address ALE table entry. */
#define CPSW_ALE_POLICER_TABLEENTRY_DELETE_MACDST    (CPSW_ALE_POLICER_MATCH_MACDST)

/*! \brief Delete InnerVLAN classifier associated ALE table entry. */
#define CPSW_ALE_POLICER_TABLEENTRY_DELETE_IVLAN     (CPSW_ALE_POLICER_MATCH_IVLAN)

/*! \brief Delete OuterVLAN classifier associated ALE table entry. */
#define CPSW_ALE_POLICER_TABLEENTRY_DELETE_OVLAN     (CPSW_ALE_POLICER_MATCH_OVLAN)

/*! \brief Delete EtherType classifier associated ALE table entry. */
#define CPSW_ALE_POLICER_TABLEENTRY_DELETE_ETHERTYPE (CPSW_ALE_POLICER_MATCH_ETHERTYPE)

/*! \brief Delete IPv4/IPv6 source address classifier associated ALE table entry. */
#define CPSW_ALE_POLICER_TABLEENTRY_DELETE_IPSRC     (CPSW_ALE_POLICER_MATCH_IPSRC)

/*! \brief Delete IPv4/IPv6 destination address classifier associated ALE table entry. */
#define CPSW_ALE_POLICER_TABLEENTRY_DELETE_IPDST     (CPSW_ALE_POLICER_MATCH_IPDST)

/*! \brief Delete all ale entries associated with classifier entries. */
#define CPSW_ALE_POLICER_TABLEENTRY_DELETE_ALL       (CPSW_ALE_POLICER_TABLEENTRY_DELETE_OUI |       \
                                                      CPSW_ALE_POLICER_TABLEENTRY_DELETE_MACSRC |    \
                                                      CPSW_ALE_POLICER_TABLEENTRY_DELETE_MACDST |    \
                                                      CPSW_ALE_POLICER_TABLEENTRY_DELETE_IVLAN |     \
                                                      CPSW_ALE_POLICER_TABLEENTRY_DELETE_OVLAN |     \
                                                      CPSW_ALE_POLICER_TABLEENTRY_DELETE_ETHERTYPE | \
                                                      CPSW_ALE_POLICER_TABLEENTRY_DELETE_IPSRC |     \
                                                      CPSW_ALE_POLICER_TABLEENTRY_DELETE_IPDST)

/*! @} */

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief ALE IOCTL commands.
 */
typedef enum CpswAle_Ioctl_e
{
    /*!
     * \brief Dump ALE table entries.
     *
     * Print current entries in the ALE table.
     *
     * IOCTL params:
     * -  inArgs: None
     * - outArgs: None
     *
     * Calling context: Task
     */
    CPSW_ALE_IOCTL_DUMP_TABLE = CPSW_ALE_PUBLIC_IOCTL(0U),

    /*!
     * \brief Add unicast address entry.
     *
     * IOCTL params:
     * -  inArgs: #CpswAle_SetUcastEntryInArgs
     * - outArgs: uint32_t
     *
     * Calling context: Task
     */
    CPSW_ALE_IOCTL_ADD_UCAST = CPSW_ALE_PUBLIC_IOCTL(1U),

    /*!
     * \brief Add multicast address entry.
     *
     * IOCTL params:
     * -  inArgs: #CpswAle_SetMcastEntryInArgs
     * - outArgs: uint32_t
     *
     * Calling context: Task
     */
    CPSW_ALE_IOCTL_ADD_MCAST = CPSW_ALE_PUBLIC_IOCTL(2U),

    /*!
     * \brief Add inner/outer VLAN entry.
     *
     * IOCTL params:
     * -  inArgs: #CpswAle_VlanEntryInfo
     * - outArgs: uint32_t
     *
     * Calling context: Task
     */
    CPSW_ALE_IOCTL_ADD_VLAN = CPSW_ALE_PUBLIC_IOCTL(3U),

    /*!
     * \brief Add OUI address entry.
     * IOCTL params:
     * -  inArgs: #CpswAle_OuiEntryInfo
     * - outArgs: uint32_t
     *
     * Calling context: Task
     */
    CPSW_ALE_IOCTL_ADD_OUI = CPSW_ALE_PUBLIC_IOCTL(4U),

    /*!
     * \brief Add IPv4 address.
     *
     * IOCTL params:
     * -  inArgs: #CpswAle_IPv4EntryInfo
     * - outArgs: uint32_t
     *
     * Calling context: Task
     */
    CPSW_ALE_IOCTL_ADD_IPV4ADDR = CPSW_ALE_PUBLIC_IOCTL(5U),

    /*!
     * \brief Add IPv6 address.
     *
     * IOCTL params:
     * -  inArgs: #CpswAle_IPv6EntryInfo
     * - outArgs: uint32_t
     *
     * Calling context: Task
     */
    CPSW_ALE_IOCTL_ADD_IPV6ADDR = CPSW_ALE_PUBLIC_IOCTL(6U),

    /*!
     * \brief Add EtherType entry.
     *
     * IOCTL params:
     * -  inArgs: uint16_t
     * - outArgs: uint32_t
     *
     * Calling context: Task
     */
    CPSW_ALE_IOCTL_ADD_ETHERTYPE = CPSW_ALE_PUBLIC_IOCTL(7U),

    /*!
     * \brief Return entry info for given unicast address.
     *
     * IOCTL params:
     * -  inArgs: #CpswAle_MacAddrInfo
     * - outArgs: #CpswAle_GetUcastEntryOutArgs
     * Calling context: Task
     */
    CPSW_ALE_IOCTL_LOOKUP_UCAST = CPSW_ALE_PUBLIC_IOCTL(8U),

    /*!
     * \brief Return entry info for given multicast address.
     *
     * IOCTL params:
     * -  inArgs: #CpswAle_GetMcastEntryInArgs
     * - outArgs: #CpswAle_GetMcastEntryOutArgs
     *
     * Calling context: Task
     */
    CPSW_ALE_IOCTL_LOOKUP_MCAST = CPSW_ALE_PUBLIC_IOCTL(9U),

    /*!
     * \brief Return entry info for given VLAN id.
     *
     * IOCTL params:
     * -  inArgs: #CpswAle_VlanIdInfo
     * - outArgs: #CpswAle_GetVlanEntryOutArgs
     *
     * Calling context: Task
     */
    CPSW_ALE_IOCTL_LOOKUP_VLAN = CPSW_ALE_PUBLIC_IOCTL(10U),

    /*!
     * \brief Delete entry info for given unicast/multicast address.
     *
     * IOCTL params:
     * -  inArgs: #CpswAle_MacAddrInfo
     * - outArgs: None
     *
     * Calling context: Task
     */
    CPSW_ALE_IOCTL_REMOVE_ADDR = CPSW_ALE_PUBLIC_IOCTL(11U),

    /*!
     * \brief Delete entry info for given VLAN id.
     *
     * IOCTL params:
     * -  inArgs: #CpswAle_VlanIdInfo
     * - outArgs: None
     *
     * Calling context: Task
     */
    CPSW_ALE_IOCTL_REMOVE_VLAN = CPSW_ALE_PUBLIC_IOCTL(12U),

    /*!
     * \brief Delete entry info for given OUI address.
     *
     * IOCTL params:
     * -  inArgs: #CpswAle_OuiEntryInfo
     * - outArgs: None
     *
     * Calling context: Task
     */
    CPSW_ALE_IOCTL_REMOVE_OUI = CPSW_ALE_PUBLIC_IOCTL(13U),

    /*!
     * \brief Delete entry info for given IPv4 address.
     *
     * IOCTL params:
     * -  inArgs: #CpswAle_IPv4EntryInfo
     * - outArgs: None
     *
     * Calling context: Task
     */
    CPSW_ALE_IOCTL_REMOVE_IPV4ADDR = CPSW_ALE_PUBLIC_IOCTL(14U),

    /*!
     * \brief Delete entry info for given IPv6 address.
     *
     * IOCTL params:
     * -  inArgs: #CpswAle_IPv6EntryInfo
     * - outArgs: None
     *
     * Calling context: Task
     */
    CPSW_ALE_IOCTL_REMOVE_IPV6ADDR = CPSW_ALE_PUBLIC_IOCTL(15U),

    /*!
     * \brief Delete entry info for given EtherType.
     *
     * IOCTL params:
     * -  inArgs: uint16_t
     * - outArgs: None
     *
     * Calling context: Task
     */
    CPSW_ALE_IOCTL_REMOVE_ETHERTYPE = CPSW_ALE_PUBLIC_IOCTL(16U),

    /*!
     * \brief Delete all learned entries for a given port.
     *
     * Returns the number of learned entries that were successfully removed.
     *
     * IOCTL params:
     * -  inArgs: uint32_t
     * - outArgs: uint32_t
     *
     * Calling context: Task
     */
    CPSW_ALE_IOCTL_REMOVE_LEARNED_ENTRIES = CPSW_ALE_PUBLIC_IOCTL(17U),

    /*!
     * \brief Delete all entries.
     *
     * IOCTL params:
     * -  inArgs: None
     * - outArgs: None
     *
     * Calling context: Task
     */
    CPSW_ALE_IOCTL_REMOVE_ALL_ENTRIES = CPSW_ALE_PUBLIC_IOCTL(18U),

    /*!
     * \brief Age all entries now.
     *
     * IOCTL params:
     * -  inArgs: None
     * - outArgs: None
     *
     * Calling context:ISR/SWI/Task
     */
    CPSW_ALE_IOCTL_AGE_ALL_ENTRIES = CPSW_ALE_PUBLIC_IOCTL(19U),

    /*!
     * \brief Set host port RX filter.
     *
     * IOCTL params:
     * -  inArgs: CpswAle_RxFilter
     * - outArgs: None
     *
     * Calling context: Task
     */
    CPSW_ALE_IOCTL_SET_RX_FILTER = CPSW_ALE_PUBLIC_IOCTL(20U),

    /*!
     * \brief Get current host port Rx filter
     *
     * IOCTL params:
     * -  inArgs: None
     * - outArgs: #CpswAle_RxFilter
     *
     * Calling context: Task
     */
    CPSW_ALE_IOCTL_GET_RX_FILTER = CPSW_ALE_PUBLIC_IOCTL(21U),

    /*!
     * \brief Set port state.
     *
     * IOCTL params:
     * -  inArgs: #CpswAle_SetPortStateInArgs
     * - outArgs: None
     * Calling context: ISR/SWI/Task
     */
    CPSW_ALE_IOCTL_SET_PORT_STATE = CPSW_ALE_PUBLIC_IOCTL(22U),

    /*!
     * \brief Get port state.
     *
     * IOCTL params:
     * -  inArgs: uint32_t
     * - outArgs: CpswAle_PortState
     *
     * Calling context: ISR/SWI/Task
     */
    CPSW_ALE_IOCTL_GET_PORT_STATE = CPSW_ALE_PUBLIC_IOCTL(23U),

    /*!
     * \brief Get MAC addresses reachable on given port.
     *
     * IOCTL params:
     * -  inArgs: #CpswAle_GetPortMacAddrInArgs
     * - outArgs: #CpswAle_GetPortMacAddrOutArgs
     *
     * Calling context: Task
     */
    CPSW_ALE_IOCTL_GET_PORT_MACADDR = CPSW_ALE_PUBLIC_IOCTL(24U),

    /*!
     * \brief Set default thread configuration.
     *
     * IOCTL params:
     * -  inArgs: #CpswAle_DfltThreadCfg
     * - outArgs: None
     *
     * Calling context:Task
     */
    CPSW_ALE_IOCTL_SET_DEFAULT_THREADCFG = CPSW_ALE_PUBLIC_IOCTL(25U),

    /*!
     * \brief Get default thread configuration.
     *
     * IOCTL params:
     * -  inArgs: None
     * - outArgs: #CpswAle_DfltThreadCfg
     *
     * Calling context:Task
     */
    CPSW_ALE_IOCTL_GET_DEFAULT_THREADCFG = CPSW_ALE_PUBLIC_IOCTL(26U),

    /*!
     * \brief Set port mirroring configuration.
     *
     * IOCTL params:
     * -  inArgs: #CpswAle_PortMirroringCfg
     * - outArgs: None
     *
     * Calling context: Task
     */
    CPSW_ALE_IOCTL_SET_PORT_MIRROR_CFG = CPSW_ALE_PUBLIC_IOCTL(27U),

    /*!
     * \brief Disable port mirror (match mirror, destination port mirror
     *        and source port mirror).
     *
     * IOCTL params:
     * -  inArgs: None
     * - outArgs: None
     *
     * Calling context: Task
     */
    CPSW_ALE_IOCTL_DISABLE_PORT_MIRROR = CPSW_ALE_PUBLIC_IOCTL(28U),

    /*!
     * \brief Set port trunking configuration.
     *
     * IOCTL params:
     * -  inArgs: #CpswAle_TrunkCfg
     * - outArgs: None
     *
     * Calling context: Task
     */
    CPSW_ALE_IOCTL_SET_TRUNK_CFG = CPSW_ALE_PUBLIC_IOCTL(29U),

    /*!
     * \brief Enable OAM loopback for ports in the given port mask.
     *
     * IOCTL params:
     * -  inArgs: uint32_t
     * - outArgs: None
     *
     * Calling context: Task
     */
    CPSW_ALE_IOCTL_SET_OAMLPBK_CFG = CPSW_ALE_PUBLIC_IOCTL(30U),

    /*!
     * \brief Set broadcast/multicast rate limit configuration.
     *
     * IOCTL params:
     * -  inArgs: #CpswAle_SetBcastMcastRateLimitInArgs
     * - outArgs: None
     *
     * Calling context: Task
     */
    CPSW_ALE_IOCTL_SET_BCAST_MCAST_LIMIT = CPSW_ALE_PUBLIC_IOCTL(31U),

    /*!
     * \brief Get configured broadcast/multicast rate limit configuration.
     *
     * IOCTL params:
     * -  inArgs: None
     * - outArgs: #CpswAle_GetBcastMcastRateLimitOutArgs
     *
     * Calling context: Task
     */
    CPSW_ALE_IOCTL_GET_BCAST_MCAST_LIMIT = CPSW_ALE_PUBLIC_IOCTL(32U),

    /*!
     * \brief Disable broadcast/multicast rate limit.
     *
     * IOCTL params:
     * -  inArgs: None
     * - outArgs: None
     *
     * Calling context: Task
     */
    CPSW_ALE_IOCTL_DISABLE_BCAST_MCAST_LIMIT = CPSW_ALE_PUBLIC_IOCTL(33U),

    /*!
     * \brief Set policer/classifier entry.
     *
     * IOCTL params:
     * -  inArgs: #CpswAle_SetPolicerEntryInArgs
     * - outArgs: #CpswAle_SetPolicerEntryOutArgs
     *
     * Calling context: Task
     */
    CPSW_ALE_IOCTL_SET_POLICER = CPSW_ALE_PUBLIC_IOCTL(34U),

    /*!
     * \brief Get policer/classifier entry info.
     *
     * IOCTL params:
     * -  inArgs: #CpswAle_PolicerMatchParams
     * - outArgs: #CpswAle_PolicerEntryOutArgs
     *
     * Calling context: Task
     */
    CPSW_ALE_IOCTL_GET_POLICER = CPSW_ALE_PUBLIC_IOCTL(35U),

    /*!
     * \brief Delete policer/classifier entry.
     *
     * IOCTL params:
     * -  inArgs: #CpswAle_PolicerMatchParams
     * - outArgs: None
     *
     * Calling context: Task
     */
    CPSW_ALE_IOCTL_DEL_POLICER = CPSW_ALE_PUBLIC_IOCTL(36U),

    /*!
     * \brief Dump ALE policer entries.
     *
     * Print ALE policy entries.
     *
     * IOCTL params:
     * -  inArgs: None
     * - outArgs: None
     *
     * Calling context: Task
     */
    CPSW_ALE_IOCTL_DUMP_POLICER_ENTRIES = CPSW_ALE_PUBLIC_IOCTL(37U),

    /*!
     * \brief Get ALE policer statistics.
     *
     * IOCTL params:
     * -  inArgs: #CpswAle_GetPolicerStatsInArgs
     * - outArgs: #CpswAle_GetPolicerStatsOutArgs
     *
     * Calling context: Task
     */
    CPSW_ALE_IOCTL_GET_POLICER_STATS = CPSW_ALE_PUBLIC_IOCTL(38U),

    /*!
     * \brief Set thread id for given classifier/policer.
     *
     * IOCTL params:
     * -  inArgs: #CpswAle_SetPolicerThreadCfgInArgs
     * - outArgs: #CpswAle_PolicerEntryOutArgs
     *
     * Calling context: Task
     */
    CPSW_ALE_IOCTL_SET_POLICER_THREADCFG = CPSW_ALE_PUBLIC_IOCTL(39U),

    /*!
     * \brief Configure policer global settings.
     * IOCTL params:
     * -  inArgs: #CpswAle_PolicerGlobalCfg
     * - outArgs: None
     *
     * Calling context: Task
     */
    CPSW_ALE_IOCTL_SET_POLICER_GLOBAL_CFG = CPSW_ALE_PUBLIC_IOCTL(40U),

    /*!
     * \brief Configure policer global settings.
     *
     * IOCTL params:
     * -  inArgs: None
     * - outArgs: #CpswAle_PolicerGlobalCfg
     *
     * Calling context: Task
     */
    CPSW_ALE_IOCTL_GET_POLICER_GLOBAL_CFG = CPSW_ALE_PUBLIC_IOCTL(41U),

    /*!
     * \brief Delete all policer entries and associated ALE entry with the
     *        given thread id.
     *
     * IOCTL params:
     * -  inArgs: uint32_t
     * - outArgs: None
     *
     * Calling context: Task
     */
    CPSW_ALE_IOCTL_DEL_ALL_POLICER_THREADID = CPSW_ALE_PUBLIC_IOCTL(42U),

    /*!
     * \brief Blacklist classifier to host port.
     *
     * ALE supports feature to allow packets matching policer match criteria
     * that is destined for host port to be dropped.
     *
     * IOCTL params:
     * -  inArgs: #CpswAle_PolicerMatchParams
     * - outArgs: #CpswAle_PolicerEntryOutArgs
     *
     * Calling context: Task
     */
    CPSW_ALE_IOCTL_BLOCK_CLASSIFIER_HOSTPORT = CPSW_ALE_PUBLIC_IOCTL(43U),

    /*!
     * \brief Set InterVLAN routing configuration.
     *
     * IOCTL params:
     * -  inArgs: #CpswAle_SetInterVlanCfgInArgs
     * - outArgs: #CpswAle_PolicerEntryOutArgs
     *
     * Calling context: Task
     */
    CPSW_ALE_IOCTL_SET_INTERVLAN_CFG = CPSW_ALE_PUBLIC_IOCTL(44U),

    /*!
     * \brief Get InterVLAN routing configuration.
     *
     * IOCTL params:
     * -  inArgs: None
     * - outArgs: None
     *
     * Calling context: Task
     */
    CPSW_ALE_IOCTL_GET_INTERVLAN_CFG = CPSW_ALE_PUBLIC_IOCTL(45U),

} CpswAle_Ioctl;



/*!
 * \brief ALE port state.
 */
typedef enum CpswAle_PortState_e
{
    /*! Discard all packets */
    CPSW_ALE_PORTSTATE_DISABLED = 0U,

    /*! Discard all packets except supervisory packets (BPDUs) */
    CPSW_ALE_PORTSTATE_BLOCKED,

    /*! Discard all packets except supervisory packets (BPDUs).
     *  Also add packet MAC SA as learned entries in table */
    CPSW_ALE_PORTSTATE_LEARN,

    /*! Forward all packets. Learning also enabled */
    CPSW_ALE_PORTSTATE_FORWARD,
} CpswAle_PortState;

/*!
 * \brief Forward state level.
 *
 * Indicates the port state(s) required for the received port on a destination
 * address lookup in order for the multicast packet to be forwarded to the
 * transmit port(s).
 *
 * A transmit port must be in the Forwarding state in order to forward the packet.
 * If the transmit port mask has multiple set bits then each forward decision is
 * independent of the other transmit port(s) forward decision.
 */
typedef enum CpswAle_FwdStateLevel_e
{
    /*! Forwarding */
    CPSW_ALE_FWDSTLVL_FWD  = 0U,

    /*! Blocking/Forwarding/Learning */
    CPSW_ALE_FWDSTLVL_BLK_FWD_LRN,

    /*! Forwarding/Learning */
    CPSW_ALE_FWDSTLVL_FWD_LRN,
} CpswAle_FwdStateLevel;

/*!
 * \brief Packet filtering type (cumulative).
 */
typedef enum CpswAle_RxFilter_e
{
    /*! Receive filter set to Nothing */
    CPSW_ALE_RXFILTER_NOTHING = 0U,

    /*! Receive filter set to Direct */
    CPSW_ALE_RXFILTER_DIRECT,

    /*! Receive filter set to Broadcast */
    CPSW_ALE_RXFILTER_BCAST,

    /*! Receive filter set to Multicast */
    CPSW_ALE_RXFILTER_MCAST,

    /*! Receive filter set to All Multicast */
    CPSW_ALE_RXFILTER_ALLMCAST,

    /*! Receive filter set to All */
    CPSW_ALE_RXFILTER_ALL,
} CpswAle_RxFilter;

/*!
 * \brief MAC address and VLAN Id.
 */
typedef struct CpswAle_MacAddrInfo_s
{
    /*! 48-bit unicast/multicast MAC address */
    uint8_t addr[ENET_MAC_ADDR_LEN];

    /*! VLAN ID associated with the entry. vlanId of 0 indicates do not use VLAN */
    uint32_t vlanId;
} CpswAle_MacAddrInfo;

/*!
 * \brief Unicast entry type info apart from MAC address that can be set.
 */
typedef struct CpswAle_UcastEntryInfo_s
{
    /*! Port number that the packet with a unicast destination address is
     *  forwarded to */
    uint32_t portNum;

    /*! Indicates that packet with a matching source or destination address
     *  should be dropped */
    bool blocked;

    /*! Indicates that packet with a matching source address should be dropped,
     *  if the received port number is not equal to the port number in the
     *  table entry */
    bool secure;

    /*! Indicates that this address can be received in port state other than
     *  Forwarding state */
    bool super;

    /*! Indicates if the unicast entry is subject to aging */
    bool ageable;

    /*! Indicates if the portNum refers to trunk number of port number */
    bool trunk;
} CpswAle_UcastEntryInfo;

/*!
 * \brief Input args for #CPSW_ALE_IOCTL_ADD_UCAST command.
 */
typedef struct CpswAle_SetUcastEntryInArgs_s
{
    /*! Address info to be added */
    CpswAle_MacAddrInfo addr;

    /*! Unicast entry additional info to be added */
    CpswAle_UcastEntryInfo info;
} CpswAle_SetUcastEntryInArgs;

/*!
 * \brief Output args for #CPSW_ALE_IOCTL_LOOKUP_UCAST command.
 */
typedef struct CpswAle_GetUcastEntryOutArgs_s
{
    /*! Entry info associated with the given unicast address */
    CpswAle_UcastEntryInfo info;

    /*! Indicates if packet with MAC SA = entry address has been received
     *  since last aging interval */
    bool touched;

    /*! Entry index */
    uint32_t aleEntryIdx;
} CpswAle_GetUcastEntryOutArgs;

/*!
 * \brief Multicast entry type info apart from MAC address.
 */
typedef struct CpswAle_McastEntryInfo_s
{
    /*! Supervisory flag which indicates that this address can be received in port
     *  state other than Forwarding state */
    bool super;

    /*! Forward state level */
    CpswAle_FwdStateLevel fwdState;

    /*! Port bit mask that is returned with a found multicast destination address */
    uint32_t portMask;

    /*! Number of bits in the address to ignore. Up to 10 least significant bits
     *  can be ignored to form a multicast address range */
    uint32_t numIgnBits;
} CpswAle_McastEntryInfo;

/*!
 * \brief Input args for #CPSW_ALE_IOCTL_ADD_MCAST command.
 */
typedef struct CpswAle_SetMcastEntryInArgs_s
{
    /*! Multicast MAC address to be added */
    CpswAle_MacAddrInfo addr;

    /*! Additional info to be added for multicast entry */
    CpswAle_McastEntryInfo info;
} CpswAle_SetMcastEntryInArgs;

/*!
 * \brief Input args for #CPSW_ALE_IOCTL_LOOKUP_MCAST command.
 */
typedef struct CpswAle_GetMcastEntryInArgs_s
{
    /*! Multicast MAC address to be added */
    CpswAle_MacAddrInfo addr;

    /*! Number of bits in the address to ignore */
    uint32_t numIgnBits;
} CpswAle_GetMcastEntryInArgs;

/*!
 * \brief Output args for #CPSW_ALE_IOCTL_LOOKUP_MCAST command.
 */
typedef struct CpswAle_GetMcastEntryOutArgs_s
{
    /*! Entry info associated with the given multicast address */
    CpswAle_McastEntryInfo info;

    /*! Entry index */
    uint32_t aleEntryIdx;
} CpswAle_GetMcastEntryOutArgs;

/*!
 * \brief VLAN id info.
 */
typedef struct CpswAle_VlanIdInfo_s
{
    /*! VLAN Id */
    uint32_t vlanId;

    /*! VLAN type is either outer or inner VLAN */
    Enet_VlanTagType tagType;
} CpswAle_VlanIdInfo;

/*!
 * \brief VLAN entry type info.
 */
typedef struct CpswAle_VlanEntryInfo_s
{
    /*! VLAN id / VLAN type to be added */
    CpswAle_VlanIdInfo vlanIdInfo;

    /*! Port member mask for the VLAN entry being added */
    uint32_t vlanMemberList;

    /*! Unregistered multicast flood mask - Mask used for multicast when the
     *  multicast address is not found */
    uint32_t unregMcastFloodMask;

    /*! Registered multicast flood mask - Mask used for multicast when the
     *  multicast address is found */
    uint32_t regMcastFloodMask;

    /*! Force untagged egress bit flags - Causes the packet VLAN tag to be
     *  removed on egress */
    uint32_t forceUntaggedEgressMask;

    /*! VLAN No Learn Mask - When a bit is set in this mask, a packet with an
     *  unknown source address received on the associated port will not be
     *  learned (i.e. When a VLAN packet is received and the source address is
     *  not in the table, the source address will not be added to the table) */
    uint32_t noLearnMask;

    /*! VLAN Ingress Check - When set, if the receive port is not a member of
     *  this VLAN then the packet is dropped */
    bool vidIngressCheck;

    /*! Limit IP NXT hdr field - When set IP packets only with configured
     *  NXTHDR will be allowed @sa CpswAle_IPPktSecurityCfg */
    bool limitIPNxtHdr;

    /*! VLAN No IPv4 Fragmented frames Control - Causes IPv4 fragmented IP
     *  frames to be dropped */
    bool disallowIPFrag;
} CpswAle_VlanEntryInfo;

/*!
 * \brief Output args for #CPSW_ALE_IOCTL_LOOKUP_MCAST command.
 */
typedef struct CpswAle_GetVlanEntryOutArgs_s
{
    /*! Port member mask for the VLAN entry being queried */
    uint32_t vlanMemberList;

    /*! Unregistered multicast flood mask for queried VLAN */
    uint32_t unregMcastFloodMask;

    /*! Registered multicast flood mask for queried VLAN  */
    uint32_t regMcastFloodMask;

    /*! Force untagged egress port mask for queried VLAN */
    uint32_t forceUntaggedEgressMask;

    /*! VLAN No Learn Mask for queried VLAN */
    uint32_t noLearnMask;

    /*! VLAN Ingress Check enabled/disabled for queried VLAN */
    bool vidIngressCheck;

    /*! Limit IP NXT hdr field enabled/disabled for queried VLAN */
    bool limitIPNxtHdr;

    /*! VLAN No IPv4 Fragmented frames control enabled/disabled for queried VLAN */
    bool disallowIPFrag;

    /*! Entry index for queried VLAN */
    uint32_t aleEntryIdx;
} CpswAle_GetVlanEntryOutArgs;

/*!
 * \brief OUI entry info.
 */
typedef struct CpswAle_OuiEntryInfo_s
{
    /*! Ethernet frame Organization Unique Id (OUI) portion of MAC address */
    uint8_t ouiAddr[ENET_OUI_ADDR_LEN];
} CpswAle_OuiEntryInfo;

/*!
 * \brief IPv4 entry info.
 */
typedef struct CpswAle_IPv4EntryInfo_s
{
    /*! IPv4 address */
    uint8_t ipv4Addr[ENET_IPv4_ADDR_LEN];

    /*! CIDR mask in term of number of LSBs of IPaddr to ignore */
    uint32_t numLSBIgnoreBits;
} CpswAle_IPv4EntryInfo;

/*!
 * \brief IPv6 entry info.
 */
typedef struct CpswAle_IPv6EntryInfo_s
{
    /*! IPv6 address */
    uint8_t ipv6Addr[ENET_IPv6_ADDR_LEN];

    /*! CIDR mask in term of number of LSBs of IPaddr to ignore */
    uint32_t numLSBIgnoreBits;
} CpswAle_IPv6EntryInfo;

/*!
 * \brief Input args for #CPSW_ALE_IOCTL_SET_PORT_STATE command.
 */
typedef struct CpswAle_SetPortStateInArgs_s
{
    /*! Port number for which port state to be set */
    uint32_t portNum;

    /*! Port state to be set */
    CpswAle_PortState portState;
} CpswAle_SetPortStateInArgs;

/*!
 * \brief Input args for #CPSW_ALE_IOCTL_GET_PORT_MACADDR command.
 */
typedef struct CpswAle_GetPortMacAddrInArgs_s
{
    /*! Port number for which MAC addresses are being queried */
    uint32_t portNum;

    /*! Application provided buffer into which MAC addresses will be populated */
    CpswAle_MacAddrInfo *addrs;

    /*! Max number of MAC entries that can be stored in provided buffer */
    uint32_t addrCnt;
} CpswAle_GetPortMacAddrInArgs;

/*!
 * \brief Output args for #CPSW_ALE_IOCTL_GET_PORT_MACADDR command.
 */
typedef struct CpswAle_GetPortMacAddrOutArgs_s
{
    /*! Application provided buffer populated with MAC addresses reachable on
     *  the port number given in inArgs CpswAle_GetPortMacAddrInArgs::portNum */
    CpswAle_MacAddrInfo *addrs;

    /*! Number of MAC entries stored in provided buffer. Will not exceed
     *  CpswAle_GetPortMacAddrInArgs::macAddrMax */
    uint32_t addrCnt;

    /*! Total number of MAC addresses reachable on given portNum */
    uint32_t totalAddrCnt;
} CpswAle_GetPortMacAddrOutArgs;

/*!
 * \brief Input args for #CPSW_ALE_IOCTL_SET_DEFAULT_THREADCFG command
 *        and out args for #CPSW_ALE_IOCTL_GET_DEFAULT_THREADCFG command.
 */
typedef struct CpswAle_DefaultThreadCfg_s
{
    /*! Default thread enable.
     *  - If set to true, the switch will use the defthreadval for the host
     *    interface thread id if no classifier is matched
     *  - If set to false, the switch will generate its own thread Id based
     *    on port and priority if there is no classifier match */
    bool dfltThreadEn;

    /*! This field specifies the default thread Id value */
    uint32_t threadId;

    /*! OR priority of packet with default thread */
    bool priorityOrEn;

    /*! Do not use default thread for MAC only port */
    bool macPortDfltThreadDis;
} CpswAle_DfltThreadCfg;

/*!
 * \brief Defines ALE table entry type.
 */
typedef enum CpswAle_TableEntryType_s
{
    /*! Unicast/multicast address entry type */
    CPSW_ALE_TABLE_ENTRY_TYPE_ADDR,

   /*! Inner/outer VLAN entry type */
    CPSW_ALE_TABLE_ENTRY_TYPE_VLAN,

    /*! EtherType entry type */
    CPSW_ALE_TABLE_ENTRY_TYPE_ETHERTYPE,

    /*! OUI entry type */
    CPSW_ALE_TABLE_ENTRY_TYPE_OUI,

    /*! IPv4 entry type */
    CPSW_ALE_TABLE_ENTRY_TYPE_IPV4,

    /*! IPv6 entry type */
    CPSW_ALE_TABLE_ENTRY_TYPE_IPV6
} CpswAle_TableEntryType;

/*!
 * \brief ALE classifier/policer match parameters for source or destination
 *        MAC address.
 */
typedef struct CpswAle_MacAddrClassifierInfo_s
{
    /*! Source MAC address to match */
    CpswAle_MacAddrInfo addr;

    /*! Ingress or egress port number */
    uint32_t portNum;
} CpswAle_MacAddrClassifierInfo;

/*!
 * \brief ALE port mirroring configuration for mirroring based on packet match.
 */
typedef struct CpswAle_MirrorMatchParams_s
{
    /*! Type of ALE entry for matching and mirroring */
    CpswAle_TableEntryType entryType;

    /*! Unicast/multicast address.
     *  Must be valid if #entryType is #CPSW_ALE_TABLE_ENTRY_TYPE_ADDR */
    CpswAle_MacAddrClassifierInfo dstMacAddrInfo;

    /*! VLAN id.
     *  Must be valid if #entryType is #CPSW_ALE_TABLE_ENTRY_TYPE_VLAN */
    CpswAle_VlanIdInfo vlanIdInfo;

    /*! IPv4 address info.
     *  Must be valid if #entryType is #CPSW_ALE_TABLE_ENTRY_TYPE_IPV4 */
    CpswAle_IPv4EntryInfo ipv4Info;

    /*! IPv6 address info.
     *  Must be valid if #entryType is #CPSW_ALE_TABLE_ENTRY_TYPE_IPV6 */
    CpswAle_IPv6EntryInfo ipv6Info;

    /*! OUI address info.
     *  Must be valid if #entryType is #CPSW_ALE_TABLE_ENTRY_TYPE_OUI */
    CpswAle_OuiEntryInfo ouiInfo;

    /*! EtherType address info.
     *  Must be valid if #entryType is #CPSW_ALE_TABLE_ENTRY_TYPE_ETHERTYPE */
    uint16_t etherType;
} CpswAle_MirrorMatchParams;

/*!
 * \brief Input args for #CPSW_ALE_IOCTL_SET_PORT_MIRROR_CFG command.
 */
typedef struct CpswAle_PortMirroringCfg_s
{
    /*! Enables the source port mirror option. When this bit is set any port
     *  with the pX_mirror_sp set in the ALE Port Control registers set will
     *  have its received traffic also sent to the mirror_top port */
    bool srcEn;

    /*! Enables the destination port mirror option. When this bit is set any
     *  traffic destined for the mirror_dp port will have its transmit traffic
     *  also sent to the mirror_top port */
    bool dstEnEn;

    /*! Enables the match mirror option. When this bit is set any traffic whose
     *  destination, source, VLAN or OUI matches the mirror_midx entry index will
     *  have that traffic also sent to the mirror_top port */
    bool matchEn;

    /*! The port to which destination traffic destined will be duplicated */
    uint32_t dstPortNum;

    /*! The destination port for the mirror traffic */
    uint32_t toPortNum;

    /*! ALE lookup table entry info that when a match occurs will cause this
     *  traffic to be mirrored to the mirror_top port.
     *  Must be valid if #matchEn is TRUE */
    CpswAle_MirrorMatchParams matchParams;

    /*! Source port number mirroring enable bitmask */
    uint32_t srcPortNumMask;
} CpswAle_PortMirroringCfg;

/*!
 * \brief Input args for #CPSW_ALE_IOCTL_SET_TRUNK_CFG command.
 */
typedef struct CpswAle_TrunkCfg_s
{
    /*! Trunk group ID value */
    uint32_t trunkId;

    /*! Number of ports in the trunk group */
    uint32_t numPorts;

    /*! Trunk group index value */
    uint32_t trunkPortIdx[CPSW_ALE_NUM_PORTS];

    /*! Hash formula starting value */
    uint32_t trunkHashBase;

    /*! Enables the destination MAC address to be used with the hash function
     *  G(X) = 1 + X + X^3
     *  and affects the trunk port transmit link determination */
    bool dstIPEn;

    /*! Enables the source MAC address to be used with the hash function
     *  G(X) = 1 + X + X^3
     *  and affects the trunk port transmit link determination */
    bool srcIPEn;

    /*! Enables the inner VLAN ID value (C-VLANID) to be used with the hash
     *  function
     *  G(X) = 1 + X+ X^3
     *  and affects the trunk port transmit link determination */
    bool innerVlanEn;

    /*! Enables the VLAN Priority bits to be used with the hash function
     *  G(X) = 1 + X + X^3
     *  and affects the trunk port transmit link determination */
    bool enablePri;

    /*! Enables the source IP address to be used with the hash function
     *  G(X) = 1 + X + X^3
     * and affects the trunk port transmit link determination */
    bool srcEn;

    /*! Enables the destination IP address to be used with the hash function
     * G(X) = 1 + X + X^3
     * and affects the trunk port transmit link determination */
    bool dstEnEn;
} CpswAle_TrunkCfg;

/*!
 * \brief Broadcast/multicast bandwidth limit configuration parameters.
 */
typedef struct CpswAle_PortBcastMcastRateLimitParams_s
{
    /*! Port number for which broadcast/multicast rate limiting configuration
     *  is to be done */
    uint32_t portNum;

    /*! Enable Broadcast Rate Limit for port.
     *  If set to TRUE, #bcastLimitNumPktsPerSec should be non-zero */
    bool bcastRateLimitForPortEn;

    /*! Enable Multicast Rate Limit for port.
     *  If set to TRUE, #mcastLimitNumPktsPerSec should be non-zero */
    bool mcastRateLimitForPortEn;

    /*! Broadcast packet limit per second */
    uint32_t bcastLimitNumPktsPerSec;

    /*! Multicast packet limit per second */
    uint32_t mcastLimitNumPktsPerSec;
} CpswAle_PortBcastMcastRateLimitParams;

/*!
 * \brief Input args for #CPSW_ALE_IOCTL_SET_BCAST_MCAST_LIMIT command.
 */
typedef struct CpswAle_SetBcastMcastRateLimitInArgs_s
{
    /*! Rate limit to be done at transmit or receive port */
    bool rateLimitAtTxPort;

    /*! Number of port for which broadcast/multicast rate limiting config
     *  to be applied */
    uint32_t numPorts;

    /*! Per port broadcast/multicast rate limit params */
    CpswAle_PortBcastMcastRateLimitParams portPrms[CPSW_ALE_NUM_PORTS];
} CpswAle_SetBcastMcastRateLimitInArgs;

/*!
 * \brief Output args for #CPSW_ALE_IOCTL_GET_BCAST_MCAST_LIMIT IOCTL command
 */
typedef struct CpswAle_GetBcastMcastRateLimitOutArgs_s
{
    /*! Indicates if broadcast/multicast rate limit is enabled */
    bool rateLimitEn;

    /*! Rate limit to be done at transmit or receive port */
    bool rateLimitAtTxPort;

    /*! Number of port for which broadcast/multicast rate limiting config
     *  to be applied */
    uint32_t numPorts;

    /*! Per port broadcast/multicast rate limit params */
    CpswAle_PortBcastMcastRateLimitParams portPrms[CPSW_ALE_NUM_PORTS];
} CpswAle_GetBcastMcastRateLimitOutArgs;

/*!
 * \brief IP address classifier type.
 */
typedef enum CpswAle_IpAddrClassifierType_e
{
    /*! Classifier entry is IPv4 address */
    CPSW_ALE_IPADDR_CLASSIFIER_IPV4 = 0x00U,

    /*! Classifier entry is IPv6 address */
    CPSW_ALE_IPADDR_CLASSIFIER_IPV6 = 0x01U,
} CpswAle_IpAddrClassifierType;

/*!
 * \brief ALE IP address classifier info.
 */
typedef struct CpswAle_IpAddrClassifierInfo_s
{
    /*! IPv4/IPv6 address type identifier */
    CpswAle_IpAddrClassifierType ipAddrType;

    /*! IPv4 address for classifier */
    CpswAle_IPv4EntryInfo ipv4Info;

    /*! IPv6 address for classifier */
    CpswAle_IPv6EntryInfo ipv6Info;
} CpswAle_IpAddrClassifierInfo;

/*!
 * \brief ALE classifier/policer match parameters.
 */
typedef struct CpswAle_PolicerMatchParams_s
{
    /*! Bitmask selecting classifier types to enable.
     * @sa ALE classifier match types */
    uint32_t policerMatchEnMask;

    /*! Port number to match.
     *  Must be valid if bit CPSW_ALE_POLICER_MATCH_PORT in
     *  policerMatchEnMask is set */
    uint32_t portNum;

    /*! Flag indicating port number is a trunk group */
    bool portIsTrunk;

    /*! Received packet priority to match.
     *  Must be valid if bit CPSW_ALE_POLICER_MATCH_PRIORITY in
     *  policerMatchEnMask is set */
    uint32_t priority;

    /*! OUI address to match.
     *  Must be valid if bit CPSW_ALE_POLICER_MATCH_OUI in
     *  policerMatchEnMask is set */
    CpswAle_OuiEntryInfo ouiInfo;

    /*! Source MAC address to match.
     *  Must be valid if bit CPSW_ALE_POLICER_MATCH_MACSRC in
     *  policerMatchEnMask is set */
    CpswAle_MacAddrClassifierInfo srcMacAddrInfo;

    /*! Destination MAC address to match.
     *  Must valid if bit CPSW_ALE_POLICER_MATCH_MACDST in
     *  policerMatchEnMask is set */
    CpswAle_MacAddrClassifierInfo dstMacAddrInfo;

    /*! Inner VLAN ID to match.
     *  Must be valid if bit CPSW_ALE_POLICER_MATCH_IVLAN in
     *  policerMatchEnMask is set */
    uint32_t ivlanId;

    /*! Outer VLAN ID to match.
     *  Must be valid if bit CPSW_ALE_POLICER_MATCH_OVLAN in
     *  policerMatchEnMask is set */
    uint32_t ovlanId;

    /*! Frame EtherType to match.
     *  Must be valid if bit CPSW_ALE_POLICER_MATCH_ETHERTYPE in
     *  policerMatchEnMask is set */
    uint16_t etherType;

    /*! Source IP address to match.
     *  Must be valid if bit CPSW_ALE_POLICER_MATCH_IPSRC in
     *  policerMatchEnMask is set */
    CpswAle_IpAddrClassifierInfo srcIpInfo;

    /*! Destination IP address to match.
     *  Must be valid if bit CPSW_ALE_POLICER_MATCH_IPDST in
     *  policerMatchEnMask is set */
    CpswAle_IpAddrClassifierInfo dstIpInfo;
} CpswAle_PolicerMatchParams;

/*!
 * \brief Input args for #CPSW_ALE_IOCTL_SET_POLICER command.
 */
typedef struct CpswAle_SetPolicerEntryInArgs_s
{
    /*! Policer match config */
    CpswAle_PolicerMatchParams policerMatch;

    /*! Enable threadid setting for this policer entry */
    bool threadIdEn;

    /*! Thread Id which will be enabled for this policer match */
    uint32_t threadId;

    /*! Peak rate in bits per second. 0 indicates rate limit is disabled */
    uint32_t peakRateInBitsPerSec;

    /*! Commit rate in bits per second. 0 indicates rate limit is disabled */
    uint32_t commitRateInBitsPerSec;
} CpswAle_SetPolicerEntryInArgs;

/*!
 * \brief Input args for #CPSW_ALE_IOCTL_SET_POLICER command
 */
typedef struct CpswAle_SetPolicerEntryOutArgs_s
{
    /*! OUI ALE table entry index to match.
     *  Valid if bit CPSW_ALE_POLICER_MATCH_OUI in policerMatchEnMask
     *  is set */
    uint32_t ouiAleEntryIdx;

    /*! MAC address ALE table entry index to match source MAC address.
     *  Valid if bit CPSW_ALE_POLICER_MATCH_MACSRC in policerMatchEnMask
     *  is set */
    uint32_t srcMacAleEntryIdx;

    /*! MAC address ALE table entry index to match destination MAC address.
     *  Valid if bit CPSW_ALE_POLICER_MATCH_MACDST in policerMatchEnMask
     *  is set */
    uint32_t dstMacAleEntryIdx;

    /*! Inner VLAN ALE table entry index to match.
     *  Valid if bit CPSW_ALE_POLICER_MATCH_IVLAN in policerMatchEnMask
     *  is set */
    uint32_t ivlanAleEntryIdx;

    /*! Outer VLAN ALE table entry index to match.
     *  Valid if bit CPSW_ALE_POLICER_MATCH_OVLAN in policerMatchEnMask
     *  is set */
    uint32_t ovlanAleEntryIdx;

    /*! EtherType ALE table entry index to match.
     *  Valid if bit CPSW_ALE_POLICER_MATCH_ETHERTYPE in policerMatchEnMask
     *  is set */
    uint32_t etherTypeAleEntryIdx;

    /*! IP address ALE table entry index to match as source IP address.
     *  Valid if bit CPSW_ALE_POLICER_MATCH_IPSRC in policerMatchEnMask
     *  is set */
    uint32_t srcIpAleEntryIdx;

    /*! IP address ALE table entry index to match as destination IP address.
     *  Valid if bit CPSW_ALE_POLICER_MATCH_IPDST in policerMatchEnMask
     *  is set */
    uint32_t dstIpAleEntryIdx;

    /*! ALE policer table entry index */
    uint32_t policerEntryIdx;
} CpswAle_SetPolicerEntryOutArgs;

/*!
 * \brief Output args for #CPSW_ALE_IOCTL_GET_POLICER,
 *        #CPSW_ALE_IOCTL_SET_POLICER_THREADCFG,
 *        #CPSW_ALE_IOCTL_BLOCK_CLASSIFIER_HOSTPORT and
 *        #CPSW_ALE_IOCTL_SET_INTERVLAN_CFG commands.
 */
typedef struct CpswAle_PolicerEntryOutArgs_s
{
    /*! Bitmask selecting classifier types to be enabled for this policer.
     *  \sa ALE classifier match types */
    uint32_t policerMatchEnMask;

    /*! Port number to match.
     *  Valid if bit CPSW_ALE_POLICER_MATCH_PORT in policerMatchEnMask
     *  is set */
    uint32_t port;

    /*! Flag indicating port number is a trunk group.
     *  Valid if bit CPSW_ALE_POLICER_MATCH_PORT in policerMatchEnMask
     *  is set */
    bool portIsTrunk;

    /*! Received packet priority to match.
     *  Valid if bit CPSW_ALE_POLICER_MATCH_PRIORITY in policerMatchEnMask
     *  is set */
    uint32_t priority;

    /*! OUI ALE table entry index to match.
     *  Valid if bit CPSW_ALE_POLICER_MATCH_OUI in policerMatchEnMask
     *  is set */
    uint32_t ouiAleEntryIdx;

    /*! MAC address ALE table entry index to match source MAC address.
     *  Valid if bit CPSW_ALE_POLICER_MATCH_MACSRC in policerMatchEnMask
     *  is set */
    uint32_t srcMacAleEntryIdx;

    /*! MAC address ALE table entry index to match destination MAC address.
     *  Valid if bit CPSW_ALE_POLICER_MATCH_MACDST in policerMatchEnMask
     *  is set */
    uint32_t dstMacAleEntryIdx;

    /*! Inner VLAN ALE table entry index to match.
     *  Valid if bit CPSW_ALE_POLICER_MATCH_IVLAN in policerMatchEnMask
     *  is set */
    uint32_t ivlanAleEntryIdx;

    /*! Outer VLAN ALE table entry index to match.
     *  Valid if bit CPSW_ALE_POLICER_MATCH_OVLAN in policerMatchEnMask
     *  is set */
    uint32_t ovlanAleEntryIdx;

    /*! EtherType ALE table entry index to match.
     *  It's valid if bit CPSW_ALE_POLICER_MATCH_ETHERTYPE in
     *  policerMatchEnMask is set */
    uint32_t etherTypeAleEntryIdx;

    /*! IP address ALE table entry index to match as source IP address.
     *  Valid if bit CPSW_ALE_POLICER_MATCH_IPSRC in policerMatchEnMask
     *  is set */
    uint32_t srcIpAleEntryIdx;

    /*! IP address ALE table entry index to match as destination IP address.
     *  Valid if bit CPSW_ALE_POLICER_MATCH_IPDST in policerMatchEnMask
     *  is set */
    uint32_t dstIpAleEntryIdx;

    /*! Flag indicating if thread id setting is enabled for this policer entry */
    bool threadIdEn;

    /*! Thread Id which will be enabled for this policer match.
     *  Valid if #threadIdEn is true. */
    uint32_t threadId;

    /*! Peak rate in bits per second. 0 indicates rate limit is disabled */
    uint32_t peakRateInBitsPerSec;

    /*! Commit rate in bits per second. 0 indicates rate limit is disabled */
    uint32_t commitRateInBitsPerSec;

    /*! ALE policer table entry index */
    uint32_t policerEntryIdx;

    /*! Egress Op enabled */
    bool egressOpEn;

    /*! Egress Op code:
     *  - 0x00 - Egress Op disabled
     *  - 0xFF - Egress Op: OAM loopback enabled
     *  - 0x01 - #CPSW_MACPORT_INTERVLAN_ROUTEID_LAST - InterVLAN route enabled
     */
    uint32_t egressOpcode;

    /*! TTL check enabled for the interVLAN route.
     *  Valid only if egress op code has interVLan route enabled */
    bool ttlCheckEn;

    /*! Trunk index to be used if detPort for interVLAN route is a trunk.
     *  Valid only if egress op code has interVLAN route enabled */
    uint32_t egressTrunkIdx;

    /*! Port mask to which the packet is to be routed.
     *  Valid only if egress op code has interVLAN route enabled */
    uint32_t dstPortMask;
} CpswAle_PolicerEntryOutArgs;

/*!
 * \brief Input args for #CPSW_ALE_IOCTL_DEL_POLICER IOCTL command.
 */
typedef struct CpswAle_DelPolicerEntryInArgs_s
{
    /*! Policer match config */
    CpswAle_PolicerMatchParams policerMatch;

    /*! Bitmask to indicate which ALE entries associated with the policer
     *  have to be deleted.
     *  @sa ALE classifier aleEntry delete bitmask */
    uint32_t aleEntryMask;
} CpswAle_DelPolicerEntryInArgs;

/*!
 * \brief Input args for #CPSW_ALE_IOCTL_GET_POLICER_STATS command.
 */
typedef struct CpswAle_GetPolicerStatsInArgs_s
{
    /*! Policer info for which statistics is queried */
    CpswAle_PolicerMatchParams policerInfo;

    /*! Flag for clearing the policer statistics */
    bool clearStats;
} CpswAle_GetPolicerStatsInArgs;

/*!
 * \brief Output args for #CPSW_ALE_IOCTL_GET_POLICER_STATS command.
 */
typedef struct CpswAle_GetPolicerStatsOutArgs_s
{
    /*! Indicates that the selected policing/classifier via the pol_test_idx
     *  field has been hit by a packet seen on any port that matches the
     *  policing/classifier entry match */
    bool policerHit;

    /*! Indicates that the selected policing/classifier via the pol_test_idx
     *  field has been hit during a RED condition by a packet seen on any port
     * that matches the policing/classifier entry match */
    bool policerRedHit;

    /*! Indicates that the selected policing/classifier via the pol_test_idx
     *  field has been hit during a YELLOW condition by a packet seen on any port
     *  that matches the policing/classifier entry match */
    bool policerYellowHit;
} CpswAle_GetPolicerStatsOutArgs;

/*!
 * \brief Input args for #CPSW_ALE_IOCTL_SET_POLICER_THREADCFG command.
 *
 * Configuration to set the thread/flow id associated with a classifier.
 */
typedef struct CpswAle_SetPolicerThreadCfgInArgs_s
{
    /*! Policer match config */
    CpswAle_PolicerMatchParams policerMatch;

    /*! Enable/disable thread id setting for this policer entry */
    bool threadIdEn;

    /*! Thread Id which will be enabled for this policer match */
    uint32_t threadId;
} CpswAle_SetPolicerThreadCfgInArgs;

/*!
 * \brief Yellow threshold value.
 *
 * When set, enables a portion of the yellow packets to be dropped based on the
 * "yellow_drop_en" enable.
*/
typedef enum CpswAle_PolicerYellowThresh_e
{
    /*! Drop 100% of packets colored yellow by policer */
    CPSW_ALE_POLICER_YELLOWTHRESH_DROP_PERCENT_100,

    /*! Drop 50% of packets colored yellow by policer */
    CPSW_ALE_POLICER_YELLOWTHRESH_DROP_PERCENT_50,

    /*! Drop 33% of packets colored yellow by policer */
    CPSW_ALE_POLICER_YELLOWTHRESH_DROP_PERCENT_33,

    /*! Drop 25% of packets colored yellow by policer */
    CPSW_ALE_POLICER_YELLOWTHRESH_DROP_PERCENT_25,

    /*! Drop 20% of packets colored yellow by policer */
    CPSW_ALE_POLICER_YELLOWTHRESH_DROP_PERCENT_20,

    /*! Drop 17% of packets colored yellow by policer */
    CPSW_ALE_POLICER_YELLOWTHRESH_DROP_PERCENT_17,

    /*! Drop 14% of packets colored yellow by policer */
    CPSW_ALE_POLICER_YELLOWTHRESH_DROP_PERCENT_14,

    /*! Drop 13% of packets colored yellow by policer */
    CPSW_ALE_POLICER_YELLOWTHRESH_DROP_PERCENT_13,
} CpswAle_PolicerYellowThresh;

/*!
 * \brief Policing Match Mode
 *
 * This field determines what happens to packets that fail to hit any
 * policing/classifier entry.
 */
typedef enum CpswAle_PolicerNoMatchMode_e
{
    /*! Color packets not matching any classifier entry as GREEN */
    CPSW_ALE_POLICER_NOMATCH_MODE_GREEN,

    /*! Color packets not matching any classifier entry as YELLOW */
    CPSW_ALE_POLICER_NOMATCH_MODE_YELLOW,

    /*! Color packets not matching any classifier entry as RED */
    CPSW_ALE_POLICER_NOMATCH_MODE_RED,

    /*! No Hit packets are marked based on policing for unregulated traffic */
    CPSW_ALE_POLICER_NOMATCH_MODE_UNREGULATED_TRAFFIC_POLICER,
} CpswAle_PolicerNoMatchMode;

/*!
 * \brief Unregulated traffic (no classifier hit) policing params.
 */
typedef struct CpswAle_UnregulatedTrafficPolicer_s
{
    /*! Peak rate in bits per second. 0 indicates rate limit is disabled */
    uint32_t peakRateInBitsPerSec;

    /*! Commit rate in bits per second. 0 indicates rate limit is disabled */
    uint32_t commitRateInBitsPerSec;
} CpswAle_UnregulatedTrafficPolicer;

/*!
 * \brief Policer global configuration parameters.
 */
typedef struct CpswAle_PolicerGlobalCfg_s
{
    /*! Enables the policing to color the packets. It also enables red or yellow
     *  drop capabilities */
    bool policingEn;

    /*! Enables the ALE to drop yellow packets based on the yellow threshold
     *  value */
    bool yellowDropEn;

    /*! Enables the ALE to drop the red colored packets */
    bool redDropEn;

    /*! Yellow threshold value */
    CpswAle_PolicerYellowThresh yellowThresh;

    /*! Policing match mode */
    CpswAle_PolicerNoMatchMode policerNoMatchMode;

    /*! Policing params for no hit (unregulated traffic).
     *  Must be valid if policerNoMatchMode is set to
     *  #CPSW_ALE_POLICER_NOMATCH_MODE_UNREGULATED_TRAFFIC_POLICER */
    CpswAle_UnregulatedTrafficPolicer noMatchPolicer;
} CpswAle_PolicerGlobalCfg;

/*!
 * \brief Output args for #CPSW_ALE_IOCTL_SET_DEFAULT_THREADCFG command.
 *
 * InterVLAN routing configuration in ALE is associated with each
 * policer/classifier entry.
 */
typedef struct CpswAle_SetInterVlanCfgInArgs_s
{
    /*! Policer match configuration parameters */
    CpswAle_PolicerMatchParams policerMatch;

    /*! Enable TTL field check for non-zero value */
    bool ttlCheckEn;

    /*! Destination port mask to route the packet matching #policerMatch */
    uint32_t dstPortMask;

    /*! Destination port route index which has info on the MAC SA/DA to be
     *  replaced for the outgoing packet */
    uint32_t routeIdx;

    /*! The Egress Trunk Index is the calculated trunk index from the SA, DA or
     *  VLAN if modified to that InterVLAN routing will work on trunks as well.
     *  The DA, SA and VLAN are ignored for trunk generation on InterVLAN
     *  Routing so that this field is the index generated from the Egress Op
     *  replacements elclusive or'd together into a three bit index */
    uint32_t egressTrunkIdx;
} CpswAle_SetInterVlanCfgInArgs;

/*!
 * \brief ALE aging time configuration.
 */
typedef struct CpswAle_AgingCfg_s
{
    /*! Flag to enable auto aging */
    bool autoAgingEn;

    /*! Aging period in ms */
    uint32_t agingPeriodInMs;
} CpswAle_AgingCfg;

/*!
 * \brief Init time VLAN configuration.
 */
typedef struct CpswAle_InitVlanCfg_s
{
    /*! Configure ALE to operate in VLAN aware mode */
    bool aleVlanAwareMode;

    /*! Flag indicating if CPSW is in VLAN aware mode */
    bool cpswVlanAwareMode;

    /*! Learn VLAN with address when autolearning addresses */
    bool autoLearnWithVlan;

    /*! Do not learn addresses with unknown VLAN */
    bool unknownVlanNoLearn;

    /*! Unknown VLAN Force Untagged Egress */
    uint32_t unknownForceUntaggedEgressMask;

    /*! Unknown VLAN Registered Multicast Flood Mask */
    uint32_t unknownRegMcastFloodMask;

    /*! Unknown VLAN Multicast Flood Mask */
    uint32_t unknownUnregMcastFloodMask;

    /*! Unknown VLAN Member List */
    uint32_t unknownVlanMemberListMask;
} CpswAle_InitVlanCfg;

/*!
 * \brief Malformed packet handling configuration.
 */
typedef struct CpswAle_MalformedPktSecurityCfg_s
{
    /*! Disable dropping of packets with Multicast address as SA */
    bool srcMcastDropDis;

    /*! Enable dropping of packets where the 802.3 length field is larger than
     *  the packet. Ethertypes 0-1500 are 802.3 lengths, all others are
     *  EtherTypes */
    bool badLenPktDropEn;
} CpswAle_MalformedPktSecurityCfg;

/*!
 * \brief IP packet security configuration.
 */
typedef struct CpswAle_IPPktSecurityCfg_s
{
    /*! Enable dropping of packets if a VLAN entry is not found */
    bool dfltNoFragEn;

    /*! Enable dropping of packets if VLAN not found AND IP NXT HDR does not
     *  match one of #ipNxtHdrWhitelist */
    bool dfltNxtHdrWhitelistEn;

    /*! Number of ip Nxt Hdr to whitelist.
     * Must be <= CPSW_ALE_MAX_WHITELIST_IP_NXT_HDR */
    uint32_t ipNxtHdrWhitelistCnt;

    /*! IP NXT HDR whitelist */
    uint8_t ipNxtHdrWhitelist[CPSW_ALE_MAX_WHITELIST_IP_NXT_HDR];
} CpswAle_IPPktSecurityCfg;

/*!
 * \brief MAC authentication configuration.
 */
typedef struct CpswAle_MacAuthCfg_s
{
    /*! Enable MAC Authorization mode */
    bool authModeEn;

    /*! Disable Port authorization - When set will allow unknown addresses to
     *  arrive on a switch in authorization mode. It is intended for device to
     *  device network connection on ports which do not require MACSEC
     *  encryption */
    uint32_t macAuthDisMask;
} CpswAle_MacAuthCfg;

/*!
 * \brief High level structure with ALE network security configuration.
 */
typedef struct CpswAle_NetworkSecurityCfg_s
{
    /*! When set, any packet with a non-matching OUI source address will be
     *  dropped to the host unless the packet destination address matches a
     *  supervisory destination address table entry. When cleared, any packet
     *  source address matching an OUI address table entry will be dropped to
     *  the host unless the destination address matches with a supervisory
     *  destination address table entry */
    bool hostOuiNoMatchDeny;

    /*! Enable VLAN ID = 0 Mode, When cleared process the priority tagged packet
     *  with VID = PORT_VLAN[11:0]. When set process the priority tagged packet
     *  with VID = 0 */
    bool vid0ModeEn;

    /*! Malformed packet configuration */
    CpswAle_MalformedPktSecurityCfg malformedPktCfg;

    /*! IP packet security configuration */
    CpswAle_IPPktSecurityCfg ipPktCfg;

    /*! MAC authentication configuration */
    CpswAle_MacAuthCfg macAuthCfg;
} CpswAle_NetworkSecurityCfg;

/*!
 * \brief Port specific auto learning security configuration.
 */
typedef struct CpswAle_PortLearningSecurityCfg_s
{
    /*! No-learn mode */
    bool noLearn;

    /*! No Source Address Update - When set will not update the source addresses
     *  for this port */
    bool noSaUpdateEn;
} CpswAle_PortLearningSecurityCfg;

/*!
 * \brief Port specific VLAN security configuration.
 */
typedef struct CpswAle_PortVlanSecurityCfg_s
{
    /*! VLAN ID Ingress Check */
    bool vidIngressCheck;

    /*! Drop untagged packets */
    bool dropUntagged;

    /*! Drop Dual VLAN - When set will cause any received packet with dual VLAN
     *  stag followed by ctag to be dropped */
    bool dropDualVlan;

    /*! Drop Double VLAN - When set cause any received packet with double VLANs
     *  to be dropped. That is if there are two ctag or two stag fields in the
     * packet it will be dropped */
    bool dropDoubleVlan;
} CpswAle_PortVlanSecurityCfg;

/*!
 * \brief Port MAC mode configuration.
 */
typedef struct CpswAle_PortMacModeCfg_s
{
    /*! MAC-only Copy All Frames:
     *  - When set a Mac Only port will transfer all received good frames to the
     *    host
     *  - When clear a Mac Only port will transfer packets to the host based on
     *    ALE destination address lookup operation (which operates more like an
     *    Ethernet MAC) */
    bool macOnlyCafEn;

    /*! MAC-only.
     *  When set enables this port be treated like a MAC port for the host.
     *  All traffic received is only sent to the host. The host must direct
     *  traffic to this port as the lookup engine will not send traffic to the
     *  ports with the p0_maconly bit set and the p0_no_learn also set.
     *  If p0_maconly bit is set and the p0_no_learn is not set, the host can
     *  send non-directed packets that can be sent to the destination of a
     *  MacOnly port. It is also possible that The host can broadcast to
     *  ports including MacOnly ports in this mode */
    bool macOnlyEn;
} CpswAle_PortMacModeCfg;

/*!
 * \brief Port default VLAN configuration.
 */
typedef CpswAle_VlanEntryInfo CpswAle_PortVlanCfg;

/*!
 * \brief ALE init time port specific configuration params.
 */
typedef struct CpswAle_PortCfg_s
{
    /*! Port specific learning configuration */
    CpswAle_PortLearningSecurityCfg learningCfg;

    /*! Port specific learningvlan security configuration */
    CpswAle_PortVlanSecurityCfg vlanCfg;

    /*! Port specific mac only mode configuration  */
    CpswAle_PortMacModeCfg macModeCfg;

    /*! Default VLAN ID config for each port */
    CpswAle_PortVlanCfg pvidCfg;
} CpswAle_PortCfg;

/*!
 * \brief ALE configuration.
 *
 * This data structure contains configuration items related to the
 * ALE_CONTROL register fields.
 */
typedef struct CpswAle_Cfg_s
{
    /*! ALE Control fields. Bit mask based on CPSW_ALE_CFG_MASK */
    uint32_t modeFlags;

    /*! ALE Policer configuration */
    CpswAle_PolicerGlobalCfg policerGlobalCfg;

    /*! Aging config */
    CpswAle_AgingCfg agingCfg;

    /*! VLAN config */
    CpswAle_InitVlanCfg vlanCfg;

    /*! Network security related configuration of ALE */
    CpswAle_NetworkSecurityCfg nwSecCfg;

    /*! ALE Port configs */
    CpswAle_PortCfg portCfg[CPSW_ALE_NUM_PORTS];
} CpswAle_Cfg;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief Initialize CPSW ALE configuration parameters.
 *
 * \param aleCfg    Configuration parameters to be initialized
 */
void CpswAle_initCfg(CpswAle_Cfg *aleCfg);

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

#endif /* CPSW_ALE_H_ */

/*! @} */
