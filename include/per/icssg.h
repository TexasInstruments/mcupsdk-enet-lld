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
 * \file  icssg.h
 *
 * \brief This file contains the type definitions and helper macros for the
 *        ICSSG peripheral interface.
 */

/*!
 * \ingroup  DRV_ENET_PERS
 * \defgroup DRV_ENET_ICSSG ICSSG Peripheral
 *
 * This peripheral driver supports ICSSG Ethernet peripheral found in Jacinto 7
 * processor family.
 *
 * @{
 */

#ifndef ICSSG_H_
#define ICSSG_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <include/mod/icssg_timesync.h>
#include <include/mod/icssg_stats.h>
#include <include/mod/mdio.h>
#include <include/phy/enetphy.h>
#include <include/core/enet_mod_phy.h>
#include <priv/core/enet_rm_priv.h>
#include <drivers/pruicss/g_v0/pruicss.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/*! \brief Helper macro to create IOCTL commands for ICSSG per. */
#define ICSSG_PUBLIC_IOCTL(x)             (ENET_IOCTL_PER_BASE |  \
                                           ENET_IOCTL_PER_ICSSG | \
                                           ENET_IOCTL_MIN(x))

/*! \brief Number of supported ports (host port, physical ports 1 and 2). */
#define ICSSG_PORT_NUM                    (3U)

/*! \brief Maximum number of MAC ports per ICSSG instance. */
#define ICSSG_MAC_PORT_MAX                (ICSSG_PORT_NUM - 1U)

/*! \brief Maximum number of MAC ports in a ICSSG peripheral in Dual-MAC mode.
 *         Note that each port in Dual-MAC is handled as a separate Enet peripheral. */
#define ICSSG_PER_DUALMAC_PORT_MAX        (1U)

/*! \brief Maximum number of MAC ports in a ICSSG peripheral in Switch mode. */
#define ICSSG_PER_SWITCH_PORT_MAX         (ICSSG_MAC_PORT_MAX)

/*! \brief Number of supported icssg instances */
#ifdef SOC_AM65XX
#define ICSSG_INSTANCE_NUM                  (3U)
#else
#define ICSSG_INSTANCE_NUM                  (2U)
#endif

/*! \brief Max number of buffer pools */
#define ICSSG_MAX_NUM_BUFFER_POOLS          (24U)

/*! \brief Number of TX channels per slice in Dual-MAC mode. */
#define ICSSG_DUALMAC_TX_CH_NUM             (ENET_CFG_RM_TX_CH_MAX)

/*! \brief Number of RX flows per slice in Dual-MAC mode. */
#define ICSSG_DUALMAC_RX_FLOW_NUM           (ENET_CFG_RM_RX_CH_MAX / 2)

/*! \brief Number of TX channels in Switch mode. */
#define ICSSG_SWITCH_TX_CH_NUM              (ENET_CFG_RM_TX_CH_MAX)

/*! \brief Number of RX flows in Switch mode. */
#define ICSSG_SWITCH_RX_FLOW_NUM            (ENET_CFG_RM_RX_CH_MAX)

/*! \brief Number of port buffer pools required for Dual-MAC. */
#define ICSSG_DUALMAC_PORT_BUFFER_POOL_NUM  (0U)

/*! \brief Number of host egress queues required for Dual-MAC. */
#define ICSSG_DUALMAC_HOST_EGRESS_QUEUE_NUM (2U)

/*! \brief Number of port buffer pools required for Dual-MAC. */
#define ICSSG_SWITCH_PORT_BUFFER_POOL_NUM   (8U)

/*! \brief Number of host egress queues required for Dual-MAC. */
#define ICSSG_SWITCH_HOST_EGRESS_QUEUE_NUM  (2U)

/*! \brief Host egress queue padding size. */
#define ICSSG_HOST_EGRESS_BUFFER_PADDING    (2048U)

/*! \brief Size of the scratch buffer used for error frames and large frames. */
#define ICSSG_SCRATCH_BUFFER_SIZE           (ENET_UTILS_ALIGN((2048U), ICSSG_CACHELINE_ALIGNMENT))

/*!
 * \anchor IccsgFdb_EntryFields
 * \name   ICSSG FDB entry fields.
 *
 * @{
 */

/*!
 * \brief Host port membership.
 *
 * Indicates host port membership.
 */
#define ICSSG_FDB_ENTRY_P0_MEMBERSHIP         (ENET_BIT(0U))

/*!
 * \brief Physical port 1 membership.
 *
 * Indicates that MAC ID is connected to physical port 1.
 */
#define ICSSG_FDB_ENTRY_P1_MEMBERSHIP         (ENET_BIT(1U))

/*!
 * \brief Physical port 2 membership.
 *
 * Indicates that MAC ID is connected to physical port 2.
 */
#define ICSSG_FDB_ENTRY_P2_MEMBERSHIP         (ENET_BIT(2U))

/*!
 * \brief Ageable bit.
 *
 * Ageable bit is set for learned entries and cleared for static entries.
 */
#define ICSSG_FDB_ENTRY_AGEABLE               (ENET_BIT(3U))

/*!
 * \brief Block bit.
 *
 * If set for SA then packet is dropped (can be used to implement a blacklist).
 * If set for DA then packet is determined to be a special packet.
 */
#define ICSSG_FDB_ENTRY_BLOCK                 (ENET_BIT(4U))

/*!
 * \brief Secure bit.
 *
 * If set for DA then the SA from the packet is not learned.
 */
#define ICSSG_FDB_ENTRY_SECURE                (ENET_BIT(5U))

/*!
 * \brief Touched bit.
 *
 * If set, it means packet has been seen recently with source address + FID
 * matching MAC address/FID of entry.
 */
#define ICSSG_FDB_ENTRY_TOUCHED               (ENET_BIT(6U))

/*!
 * \brief Valid bit.
 *
 * Set if entry is valid.
 */
#define ICSSG_FDB_ENTRY_VALID                 (ENET_BIT(7U))

/*! @} */

/* RX Rate Source Selection */

#define ICSSG_RATE_SRC_SEL_FT1_MATCH0         (0U)
#define ICSSG_RATE_SRC_SEL_FT1_MATCH1         (1U)
#define ICSSG_RATE_SRC_SEL_FT1_MATCH2         (2U)
#define ICSSG_RATE_SRC_SEL_FT1_MATCH3         (3U)
#define ICSSG_RATE_SRC_SEL_FT1_MATCH4         (4U)
#define ICSSG_RATE_SRC_SEL_FT1_MATCH5         (5U)
#define ICSSG_RATE_SRC_SEL_FT1_MATCH6         (6U)
#define ICSSG_RATE_SRC_SEL_FT1_MATCH7         (7U)
#define ICSSG_RATE_SRC_SEL_FT3_MATCH0         (8U)
#define ICSSG_RATE_SRC_SEL_FT3_MATCH1         (9U)
#define ICSSG_RATE_SRC_SEL_FT3_MATCH2         (10U)
#define ICSSG_RATE_SRC_SEL_FT3_MATCH3         (11U)
#define ICSSG_RATE_SRC_SEL_FT3_MATCH4         (12U)
#define ICSSG_RATE_SRC_SEL_FT3_MATCH5         (13U)
#define ICSSG_RATE_SRC_SEL_FT3_MATCH6         (14U)
#define ICSSG_RATE_SRC_SEL_FT3_MATCH7         (15U)
#define ICSSG_RATE_SRC_SEL_FT3_MATCH8         (16U)
#define ICSSG_RATE_SRC_SEL_FT3_MATCH9         (17U)
#define ICSSG_RATE_SRC_SEL_FT3_MATCH10        (18U)
#define ICSSG_RATE_SRC_SEL_FT3_MATCH11        (19U)
#define ICSSG_RATE_SRC_SEL_FT3_MATCH12        (20U)
#define ICSSG_RATE_SRC_SEL_FT3_MATCH13        (21U)
#define ICSSG_RATE_SRC_SEL_FT3_MATCH14        (22U)
#define ICSSG_RATE_SRC_SEL_FT3_MATCH15        (23U)
#define ICSSG_RATE_SRC_SEL_FT_RX_UC           (24U)
#define ICSSG_RATE_SRC_SEL_FT_RX_MC           (25U)
#define ICSSG_RATE_SRC_SEL_FT_RX_BC           (26U)
#define ICSSG_RATE_SRC_SEL_FT_RX_SAV          (27U)
#define ICSSG_RATE_SRC_SEL_FT_RX_FWD          (28U)
#define ICSSG_RATE_SRC_SEL_FT_RX_RCV          (29U)
#define ICSSG_RATE_SRC_SEL_FT_RX_VLAN         (30U)
#define ICSSG_RATE_SRC_SEL_FT_RX_DA_P         (31U)
#define ICSSG_RATE_SRC_SEL_FT_RX_DA_I         (32U)
#define ICSSG_RATE_SRC_SEL_RX_CLASS_RAW0      (33U)
#define ICSSG_RATE_SRC_SEL_RX_CLASS_RAW1      (34U)
#define ICSSG_RATE_SRC_SEL_RX_CLASS_RAW2      (35U)
#define ICSSG_RATE_SRC_SEL_RX_CLASS_RAW3      (36U)
#define ICSSG_RATE_SRC_SEL_RX_CLASS_RAW4      (37U)
#define ICSSG_RATE_SRC_SEL_RX_CLASS_RAW5      (38U)
#define ICSSG_RATE_SRC_SEL_RX_CLASS_RAW6      (39U)
#define ICSSG_RATE_SRC_SEL_RX_CLASS_RAW7      (40U)
#define ICSSG_RATE_SRC_SEL_RX_CLASS_RAW8      (41U)
#define ICSSG_RATE_SRC_SEL_RX_CLASS_RAW9      (42U)
#define ICSSG_RATE_SRC_SEL_RX_CLASS_RAW10     (43U)
#define ICSSG_RATE_SRC_SEL_RX_CLASS_RAW11     (44U)
#define ICSSG_RATE_SRC_SEL_RX_CLASS_RAW12     (45U)
#define ICSSG_RATE_SRC_SEL_RX_CLASS_RAW13     (46U)
#define ICSSG_RATE_SRC_SEL_RX_CLASS_RAW14     (47U)
#define ICSSG_RATE_SRC_SEL_RX_CLASS_RAW15     (48U)

/* RX class data mapping */
#define ICCSG_RX_CLASS_DATA_FT3_MATCH0        (0U)
#define ICCSG_RX_CLASS_DATA_FT3_MATCH1        (1U)
#define ICCSG_RX_CLASS_DATA_FT3_MATCH2        (2U)
#define ICCSG_RX_CLASS_DATA_FT3_MATCH3        (3U)
#define ICCSG_RX_CLASS_DATA_FT3_MATCH4        (4U)
#define ICCSG_RX_CLASS_DATA_FT3_MATCH5        (5U)
#define ICCSG_RX_CLASS_DATA_FT3_MATCH6        (6U)
#define ICCSG_RX_CLASS_DATA_FT3_MATCH7        (7U)
#define ICCSG_RX_CLASS_DATA_FT3_MATCH8        (8U)
#define ICCSG_RX_CLASS_DATA_FT3_MATCH9        (9U)
#define ICCSG_RX_CLASS_DATA_FT3_MATCH10       (10U)
#define ICCSG_RX_CLASS_DATA_FT3_MATCH11       (11U)
#define ICCSG_RX_CLASS_DATA_FT3_MATCH12       (12U)
#define ICCSG_RX_CLASS_DATA_FT3_MATCH13       (13U)
#define ICCSG_RX_CLASS_DATA_FT3_MATCH14       (14U)
#define ICCSG_RX_CLASS_DATA_FT3_MATCH15       (15U)
#define ICCSG_RX_CLASS_DATA_FT1_MATCH0        (16U)
#define ICCSG_RX_CLASS_DATA_FT1_MATCH1        (17U)
#define ICCSG_RX_CLASS_DATA_FT1_MATCH2        (18U)
#define ICCSG_RX_CLASS_DATA_FT1_MATCH3        (19U)
#define ICCSG_RX_CLASS_DATA_FT1_MATCH4        (20U)
#define ICCSG_RX_CLASS_DATA_FT1_MATCH5        (21U)
#define ICCSG_RX_CLASS_DATA_FT1_MATCH6        (22U)
#define ICCSG_RX_CLASS_DATA_FT1_MATCH7        (23U)
#define ICCSG_RX_CLASS_DATA_FT_RX_DA_I        (24U)
#define ICCSG_RX_CLASS_DATA_FT_RX_DA_P        (25U)
#define ICCSG_RX_CLASS_DATA_FT_RX_VLAN        (26U)
#define ICCSG_RX_CLASS_DATA_FT_RX_RCV         (27U)
#define ICCSG_RX_CLASS_DATA_FT_RX_FWD         (28U)
#define ICCSG_RX_CLASS_DATA_FT_RX_BC          (29U)
#define ICCSG_RX_CLASS_DATA_FT_RX_MC          (30U)
#define ICCSG_RX_CLASS_DATA_FT_RX_SAV         (31U)

/*! \brief Minimum cycle time supported by implementation (in ns). */
#define ICSSG_TAS_MIN_CYCLE_TIME_NS           (1000000)

/*! \brief Minimum TAS window duration supported by implementation (in ns). */
#define ICSSG_TAS_MIN_WINDOW_DURATION_NS      (10000)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief ICSSG instance numbers.
 */
typedef enum Icssg_Instance_Num_e
{
    /*! Instance number 0 */
    ICSSG_INSTANCE_NUM_0,

    /*! Instance number 1 */
    ICSSG_INSTANCE_NUM_1,

    /*! Instance number 2 */
    ICSSG_INSTANCE_NUM_2,
} Icssg_Instance_Num;

/*!
 * \brief Icssg TX timestamp event callback info structure. This is passed to
 *        application when TX timestamp is retrieved by ICSSG driver.
 */
typedef struct Icssg_TxTsEvtCbInfo_s
{
    /*! Timestamp Id. This was passed by an application when enabling the timestamp
    *   for the packet during EnetDma_submitTxPktQ() */
    uint32_t txTsId;

    /*! Packet timestamp */
    uint64_t ts;
} Icssg_TxTsEvtCbInfo;

/*!
 * \brief ICSSG IOCTL commands.
 */
typedef enum Icssg_Ioctl_e
{
    /*!
     * \brief Enable promiscuous mode.
     *
     * IOCTL params:
     * -  inArgs: #Enet_MacPort
     * - outArgs: None
     *
     * Type: Synchronous.
     */
    ICSSG_MACPORT_IOCTL_ENABLE_PROMISC_MODE = ICSSG_PUBLIC_IOCTL(0U),

    /*!
     * \brief Disable promiscuous mode.
     *
     * IOCTL params:
     * -  inArgs: #Enet_MacPort
     * - outArgs: None
     *
     * Type: Synchronous.
     */
    ICSSG_MACPORT_IOCTL_DISABLE_PROMISC_MODE = ICSSG_PUBLIC_IOCTL(1U),

    /*!
     * \brief Populate VLAN table with default VLAN entry configuration.
     *
     * IOCTL params:
     * -  inArgs: #Icssg_VlanFidParams
     * - outArgs: None
     *
     * Type: Synchronous.
     */
    ICSSG_PER_IOCTL_VLAN_RESET_TABLE = ICSSG_PUBLIC_IOCTL(2U),

    /*!
     * \brief Update a VLAN table entry.
     *
     * IOCTL params:
     * -  inArgs: #Icssg_VlanFidEntry
     * - outArgs: None
     *
     * Type: Synchronous.
     */
    ICSSG_PER_IOCTL_VLAN_SET_ENTRY = ICSSG_PUBLIC_IOCTL(3U),

    /*!
     * \brief Get VLAN entry for VLAN table for requested VLAN id.
     *
     * IOCTL params:
     * -  inArgs: None
     * - outArgs: #Icssg_VlanFidEntry
     *
     * Type: Synchronous.
     */
    ICSSG_PER_IOCTL_VLAN_GET_ENTRY = ICSSG_PUBLIC_IOCTL(4U),

    /*!
     * \brief Set port state.
     *
     * Sets the port state in one of the following states (refer to
     * \ref Icssg_PortState):
     * - Disabled
     * - Blocking
     * - Forwarding
     * - Forwarding without learning
     * - TAS trigger
     * - TAS enable
     * - TAS reset
     * - TAS disable
     *
     * IOCTL params:
     * -  inArgs: #Icssg_PortState
     * - outArgs: None
     *
     * Type: Asynchronous.
     */
    ICSSG_PER_IOCTL_SET_PORT_STATE = ICSSG_PUBLIC_IOCTL(5U),

    /*!
     * \brief Add FDB entry.
     *
     * IOCTL params:
     * -  inArgs: #Icssg_FdbEntry
     * - outArgs: None
     *
     * Type: Asynchronous.
     */
    ICSSG_FDB_IOCTL_ADD_ENTRY = ICSSG_PUBLIC_IOCTL(6U),

    /*!
     * \brief Delete FDB entry.
     *
     * IOCTL params:
     * -  inArgs: #Icssg_FdbEntry
     * - outArgs: None
     *
     * Type: Asynchronous.
     */
    ICSSG_FDB_IOCTL_REMOVE_ENTRY = ICSSG_PUBLIC_IOCTL(7U),

    /*!
     * \brief Delete all FDB entries.
     *
     * IOCTL params:
     * -  inArgs: None
     * - outArgs: None
     *
     * Type: Asynchronous
     */
    ICSSG_FDB_IOCTL_REMOVE_ALL_ENTRIES = ICSSG_PUBLIC_IOCTL(8U),

    /*!
     * \brief Remote all ageable entries.
     *
     * IOCTL params:
     * -  inArgs: None
     * - outArgs: None
     *
     * Type: Asynchronous.
     */
    ICSSG_FDB_IOCTL_REMOVE_AGEABLE_ENTRIES = ICSSG_PUBLIC_IOCTL(9U),

    /*!
     * \brief Add MAC address of the interface.
     *
     * IOCTL params:
     * -  inArgs: #IcssgMacPort_SetMacAddressInArgs
     * - outArgs: None
     *
     * Type: Synchronous.
     */
    ICSSG_MACPORT_IOCTL_SET_MACADDR = ICSSG_PUBLIC_IOCTL(10U),

    /*!
     * \brief Add MAC address of the host port interface.
     *
     * IOCTL params:
     * -  inArgs: #Icssg_MacAddr
     * - outArgs: None
     *
     * Type: Synchronous.
     */
    ICSSG_HOSTPORT_IOCTL_SET_MACADDR = ICSSG_PUBLIC_IOCTL(11U),

    /*!
     * \internal
     * \brief Execute TAS 'trigger' command. Legacy implementation.
     */
    ICSSG_PER_IOCTL_TAS_TRIGGER = ICSSG_PUBLIC_IOCTL(12U),

    /*!
     * \internal
     * \brief Execute TAS 'enable' command. Legacy implementation.
     */
    ICSSG_PER_IOCTL_TAS_ENABLE = ICSSG_PUBLIC_IOCTL(13U),

    /*!
     * \internal
     * \brief Execute TAS 'disable' command. Legacy implementation.
     */
    ICSSG_PER_IOCTL_TAS_DISABLE = ICSSG_PUBLIC_IOCTL(14U),

    /*!
     * \internal
     * \brief Execute TAS 'reset' command. Legacy implementation.
     */
    ICSSG_PER_IOCTL_TAS_RESET = ICSSG_PUBLIC_IOCTL(15U),

    /*!
     * \brief Set the default VLAN ID and PCP bits for host port.
     *
     * Sets the default VLAN ID and PCP bits for host port. Only the VID
     * and PCP fields from the passed VLAN configuration are valid, the
     * CFI bit is ignored.
     *
     * IOCTL params:
     * -  inArgs: #EnetPort_VlanCfg
     * - outArgs: None
     *
     * Type: Synchronous.
     */
    ICSSG_PER_IOCTL_VLAN_SET_HOSTPORT_DFLT_VID = ICSSG_PUBLIC_IOCTL(16U),

    /*!
     * \brief Set the default VLAN ID and PCP bits for specified MAC port.
     *
     * IOCTL params:
     * -  inArgs: #Icssg_MacPortDfltVlanCfgInArgs
     * - outArgs: None
     *
     * Type: Synchronous.
     */
    ICSSG_PER_IOCTL_VLAN_SET_MACPORT_DFLT_VID = ICSSG_PUBLIC_IOCTL(17U),

    /*!
     * \brief Set the aging period of the FDB.
     *
     * Sets the FDB aging period in nanoseconds.
     *
     * IOCTL params:
     * -  inArgs: uint64_t
     * - outArgs: None
     *
     * Type: Synchronous.
     */
    ICSSG_FDB_IOCTL_SET_AGING_PERIOD = ICSSG_PUBLIC_IOCTL(18U),

    /*!
     * \brief Enable flooding of unicast packets to host port.
     *
     * IOCTL params:
     * -  inArgs: #Enet_MacPort
     * - outArgs: None
     *
     * Type: Asynchronous.
     */
    ICSSG_MACPORT_IOCTL_ENABLE_UCAST_FLOOD = ICSSG_PUBLIC_IOCTL(19U),

    /*!
     * \brief Disable flooding of unicast packets to host port.
     *
     * IOCTL params:
     * -  inArgs: #Enet_MacPort
     * - outArgs: None
     *
     * Type: Asynchronous.
     */
    ICSSG_MACPORT_IOCTL_DISABLE_UCAST_FLOOD = ICSSG_PUBLIC_IOCTL(20U),

    /*!
     * \brief Enable flooding of multicast packets to host port.
     *
     * IOCTL params:
     * -  inArgs: #Enet_MacPort
     * - outArgs: None
     *
     * Type: Asynchronous.
     */
    ICSSG_MACPORT_IOCTL_ENABLE_MCAST_FLOOD = ICSSG_PUBLIC_IOCTL(21U),

    /*!
     * \brief Disable flooding of multicast packets to host port.
     *
     * IOCTL params:
     * -  inArgs: #Enet_MacPort
     * - outArgs: None
     *
     * Type: Asynchronous.
     */
    ICSSG_MACPORT_IOCTL_DISABLE_MCAST_FLOOD = ICSSG_PUBLIC_IOCTL(22U),

    /*!
     * \brief Set the criteria for accepting VLAN tagged/untagged packets.
     *
     * Sets the accept criteria to:
     * - Accept only VLAN tagged frames
     * - Accept only untagged and priority tagged frames
     * - Accept all frames (default configuration)
     *
     * IOCTL params:
     * -  inArgs: #Icssg_SetAcceptFrameCheckInArgs
     * - outArgs: None
     *
     * Type: Synchronous.
     */
    ICSSG_MACPORT_IOCTL_SET_ACCEPT_FRAME_CHECK = ICSSG_PUBLIC_IOCTL(23U),

    /*!
     * \brief Configure ingress rate limiting.
     *
     * IOCTL params:
     * -  inArgs: #Icssg_IngressRateLim
     * - outArgs: None
     *
     * Type: Synchronous.
     */
    ICSSG_MACPORT_IOCTL_SET_INGRESS_RATE_LIM = ICSSG_PUBLIC_IOCTL(24U),

    /*!
     * \brief cut through or prempt select configuration.
     *
     * IOCTL params:
     * -  inArgs: #IcssgMacPort_SetQueueCtPremptModeInArgs
     * - outArgs: None
     *
     * Type: Synchronous.
     */
    ICSSG_MACPORT_IOCTL_SET_QUEUE_CUT_THROUGH_PREEMPT_SELECT = ICSSG_PUBLIC_IOCTL(25U),

    /*!
     * \brief special frame priority configuration.
     *
     * IOCTL params:
     * -  inArgs: #IcssgMacPort_ConfigSpecialFramePrioInArgs
     * - outArgs: None
     *
     * Type: Synchronous.
     */
    ICSSG_MACPORT_IOCTL_CONFIG_SPL_FRAME_PRIO = ICSSG_PUBLIC_IOCTL(26U),

    /*!
     * \brief Register Handler for the IOCTL CMD
     *
     * IOCTL params:
     * -  inArgs: IcssgInternalIoctlHandlerTableEntry_t
     * - outArgs: None
     *
     * Type: Synchronous.
     */
    ICSSG_INTERNAL_IOCTL_REGISTER_HANDLER = ICSSG_PUBLIC_IOCTL(27U),

} Icssg_Ioctl;

/*!
 * \brief Queue preemptive mode: express or preemptive.
 */
typedef enum Icssg_QueuePreemptMode_e
{
    /*! Express queue */
    ICSSG_QUEUE_PREEMPT_MODE_EXPRESS,

    /*! Preemptive queue */
    ICSSG_QUEUE_PREEMPT_MODE_PREEMPT,
} Icssg_QueuePreemptMode;

/*!
 * \brief Queue forward mode: cut-through or store-and-forward.
 */
typedef enum Icssg_QueueForwardMode_e
{
    /*! Store-and-forward queue */
    ICSSG_QUEUE_FORWARD_MODE_STOREANDFWD,

    /*! Cut-through queue */
    ICSSG_QUEUE_FORWARD_MODE_CUTTHROUGH,
} Icssg_QueueForwardMode;

/*!
 * \brief Input arguments for #ICSSG_MACPORT_IOCTL_SET_QUEUE_CUT_THROUGH_PREEMPT_SELECT command.
 */
typedef struct IcssgMacPort_SetQueueCtPremptModeInArgs_s
{
    /*! Port number */
    Enet_MacPort macPort;

    /*! Queue preemption mode */
    Icssg_QueuePreemptMode queuePreemptMode[ENET_PRI_NUM];

    /*! Queue forward mode */
    Icssg_QueueForwardMode queueForwardMode[ENET_PRI_NUM];
} IcssgMacPort_SetQueueCtPremptModeInArgs;

/*!
 * \brief Input arguments for #ICSSG_MACPORT_IOCTL_CONFIG_SPL_FRAME_PRIO command.
 */
typedef struct IcssgMacPort_ConfigSpecialFramePrioInArgs_s
{
    /*! Port number */
    Enet_MacPort macPort;

    /*! special frame priority */
    uint8_t specialFramePrio;
} IcssgMacPort_ConfigSpecialFramePrioInArgs;

/*!
 * \brief Port states.
 */
typedef enum Icssg_PortState_e
{
    /*! Disabled state.
     *  No frames can be sent or received, but there can be a link. */
    ICSSG_PORT_STATE_DISABLED,

    /*! Blocking state.
     *  At the port, only sync/management frames are sent or received
     *  (e.g. gPTP, LLDP, MRP) */
    ICSSG_PORT_STATE_BLOCKING,

    /*! Forwarding state.
     *  Forward frames as per MC/Stream/Static Registration FDB.
     *  Learning is enabled for frames received locally. */
    ICSSG_PORT_STATE_FORWARD,

    /*! Forwarding without learning state. */
    ICSSG_PORT_STATE_FORWARD_WO_LEARNING,

    /*! TAS trigger state */
    ICSSG_PORT_STATE_TAS_TRIGGER,

    /*! TAS enable state */
    ICSSG_PORT_STATE_TAS_ENABLE,

    /*! TAS reset state */
    ICSSG_PORT_STATE_TAS_RESET,

    /*! TAS disable state */
    ICSSG_PORT_STATE_TAS_DISABLE,
} Icssg_PortState;

/*!
 * \brief Input args for ICSSG_MACPORT_IOCTL_SET_PORT_STATE command.
 */
typedef struct IcssgMacPort_SetPortStateInArgs_s
{
    /*! Port number */
    Enet_MacPort macPort;

    /*! Icssg Port state */
    Icssg_PortState portState;
} IcssgMacPort_SetPortStateInArgs;

/*!
 * \brief Acceptable frame check criteria.
 */
typedef enum Icssg_AcceptFrameCheck_e
{
    /*! Accept only VLAN-tagged frames */
    ICSSG_ACCEPT_ONLY_VLAN_TAGGED,

    /*! Accept only untagged or priority-tagged frames */
    ICSSG_ACCEPT_ONLY_UNTAGGED_PRIO_TAGGED,

    /*! Accept all frames */
    ICSSG_ACCEPT_ALL,
} Icssg_AcceptFrameCheck;

/*!
 * \brief Input args for #ICSSG_MACPORT_IOCTL_SET_ACCEPT_FRAME_CHECK command.
 */
typedef struct Icssg_SetAcceptFrameCheckInArgs_s
{
    /*! Port number */
    Enet_MacPort macPort;

    /*! Accept frame check criteria */
    Icssg_AcceptFrameCheck acceptFrameCheck;
} Icssg_SetAcceptFrameCheckInArgs;

/*!
 * \brief MAC address.
 */
typedef struct Icssg_MacAddr_s
{
    /*! MAC address octets */
    uint8_t macAddr[ENET_MAC_ADDR_LEN];
} Icssg_MacAddr;

/*!
 * \brief Input args for ICSSG_MACPORT_IOCTL_SET_MACADDR command.
 */
typedef struct IcssgMacPort_SetMacAddressInArgs_s
{
    /*! Port number */
    Enet_MacPort macPort;

    /*! Icssg MAC address */
    uint8_t macAddr[ENET_MAC_ADDR_LEN];
} IcssgMacPort_SetMacAddressInArgs;


/*!
 * \brief VLAN FID entry parameters.
 */
typedef struct Icssg_VlanFidParams_s
{
    /*! Filtering identifier field.  This 8-bit field is included in SA and DA
     *  hashes as part of FDB function */
    uint8_t fid;

    /*! If set, forward to the host port */
    bool hostMember;

    /*! If set, forward to physical port 1 of the switch */
    bool p1Member;

    /*! If set, forward to physical port 2 of the switch */
    bool p2Member;

    /*! If set, VLAN tag should be added to host port packet being transmitted.
     *  If not set, then strip VLAN tag */
    bool hostTagged;

    /*! If set, VLAN tag should be added to phsyical port 1 packet being
     *  transmitted. If not set, then strip VLAN tag */
    bool p1Tagged;

    /*! If set, VLAN tag should be added to physical port 2 packet being
     *  transmitted, if not set, then strip VLAN tag */
    bool p2Tagged;

    /*! If set, VLAN is part of stream FDB, bypass unicast/multicast FDB */
    bool streamVid;

    /*! If set, send un-registered multicast packet to host port */
    bool floodToHost;
} Icssg_VlanFidParams;

/*!
 * \brief VLAN FID table entry.
 */
typedef struct Icssg_VlanFidEntry_s
{
    /*! Index into VLAN FID table, range from 0 to 4095 entries */
    uint16_t vlanId;

    /*! VLAN FID configuration parameters */
    Icssg_VlanFidParams vlanFidParams;
} Icssg_VlanFidEntry;

/*!
 * \brief FDB entry configuration.
 */
typedef struct Icssg_FdbEntry_s
{
    /*! MAC address */
    uint8_t macAddr[ENET_MAC_ADDR_LEN];

    /*! VLAN Id */
    int16_t vlanId;

    /*! FDB entry. It can be constructed with the FDB entry bit definitions
     *  from \ref IccsgFdb_EntryFields. Two entries for two ports */
    uint8_t fdbEntry[2];
} Icssg_FdbEntry;

/*!
 * \brief Default VLAN configuration.
 */
typedef struct Icssg_DfltVlanCfg_s
{
    /*! Port number */
    Enet_MacPort macPort;

    /*! Default VLAN Id, ranging from 0 to 4095 entries */
    uint16_t vlanId;

    /*! Priority Code Point (PCP) for default entry, ranging from 0 to 7 */
    uint8_t pcp;
} Icssg_DfltVlanCfg;

/*!
 * \brief Input arguments for #ICSSG_PER_IOCTL_VLAN_SET_MACPORT_DFLT_VID command.
 */
typedef struct Icssg_MacPortDfltVlanCfgInArgs_s
{
    /*! Port number */
    Enet_MacPort macPort;

    /*! Port's default VLAN configuration. Only VID and PCP fields are valid,
     *  CFI bit is ignored */
    EnetPort_VlanCfg vlanCfg;
} Icssg_MacPortDfltVlanCfgInArgs;

/*!
 * \brief Input arguments for #ICSSG_MACPORT_IOCTL_SET_INGRESS_RATE_LIM command.
 */
typedef struct Icssg_IngressRateLim_s
{
    /*! MAC port that this rate limiting configuration applies to */
    Enet_MacPort macPort;

    uint32_t rateLimit; //In Mbps
    uint32_t rateSrcSel;//0..47
    uint8_t  rateIndex; //0..7, 0xFF : skip
    uint8_t  classIndex; //0..15 - FW dependent
    uint8_t  notMask; //Bit 0 : !AND Bit1 : !OR
    uint8_t  classSel; //0: OR 1 : AND 2: OR AND AND 3 : OR OR AND
    uint32_t classDataAndTerm;
    uint32_t classDataOrTerm;
    struct FT_S {
        uint8_t index; //FT1 (0..7) FT3 (0..15)0XFF : Skip
        uint8_t type; // 0 : FT1 or 1 : FT3
        uint16_t ft1Start;
        uint16_t ft1Len;
        uint16_t ft1Cfg;
        union FT_U {
            struct FT1_CONFIG_S {
                uint32_t destAddrLow;
                uint32_t destAddrHigh;
                uint32_t destAddrMaskLow;
                uint32_t destAddrMaskHigh;
            } ft1;
            struct FT3_CONFIG_S {
                uint32_t start;
                uint32_t startAuto;
                uint32_t startLen;
                uint32_t jmpOffset;
                uint32_t len;
                uint32_t config;
                uint32_t type;
                uint32_t typeMask;
                uint32_t patternLow;
                uint32_t patternHigh;
                uint32_t patternMaskLow;
                uint32_t patternMaskHigh;
            } ft3;
        } u;
    } filter[2];
} Icssg_IngressRateLim;

/*!
 * \brief ICSSG custom firmware.
 *
 * Container structure for the ICSSG custom firmware blobs for PRU, RTU and TX PRU.
 */
typedef struct Icssg_custom_Fw_s
{
    /*! Pointer to PRU firmware header */
    const uint32_t *pru;

    /*! Size of PRU firmware header */
    uint32_t pruSize;

    /*! Pointer to RTU firmware header */
    const uint32_t *rtu;

    /*! Size of RTU firmware header */
    uint32_t rtuSize;

    /*! Pointer to TXPRU firmware header */
    const uint32_t *txpru;

    /*! Size of TX PRU firmware header */
    uint32_t txpruSize;
} Icssg_custom_Fw;

/*!
 * \brief ICSSG buffer pool memories.
 *
 * Memory used by ICSSG firmware for host and port buffer pools, as well as,
 * host egress queues.
 */
typedef struct Icssg_FwPoolMem_s
{
    /*! Address of the port buffer pool memory. The total size of this memory
     *  must be portBufferPoolSize * portBufferPoolNum */
    uint8_t *portBufferPoolMem;

    /*! Size of each port buffer pool */
    uint32_t portBufferPoolSize;

    /*! Number of port buffer pools */
    uint32_t portBufferPoolNum;

    /*! Address of the host buffer pool memory. The total size of this memory
     *  must be hostBufferPoolSize * hostBufferPoolNum */
    uint8_t *hostBufferPoolMem;

    /*! Size of each host buffer pool */
    uint32_t hostBufferPoolSize;

    /*! Number of host buffer pools */
    uint32_t hostBufferPoolNum;

    /*! Address of the host egress queue memory. The total size of this memory
     *  must be hostEgressQueueSize * hostEgressQueueNum */
    uint8_t *hostEgressQueueMem;

    /*! Address of the host egress queue memory. The total size of this memory
     *  must be hostEgressQueueSize * hostEgressQueueNum */
    uint8_t *hostEgressPreQueueMem;

    /*! Size of each egress queue.  This size value must include the actual egress
     *  queue size and also padding scratchBufferMem (2 kB) */
    uint32_t hostEgressQueueSize;

    /*! Size of each egress queue.  This size value must include the actual egress
     *  queue size and also padding buffer (2 kB) */
    uint32_t hostEgressPreQueueSize;

    /*! Number of egress queues */
    uint32_t hostEgressQueueNum;

    /*! Address of the scratch buffer used for error frames and large frames */
    uint8_t *scratchBufferMem;

    /*! Size of the scratch buffer */
    uint32_t scratchBufferSize;
} Icssg_FwPoolMem;

/*!
 * \brief PHY link status change event information.
 */
typedef struct Icssg_MdioLinkStateChangeInfo_s
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
} Icssg_MdioLinkStateChangeInfo;

/*!
 * \brief Icssg PHY link state change callback function.
 *
 * Callback for PHY link state change interrupt (MDIO_LINKINT).
 * This callback is invoked from interrupt context.
 */
typedef void (*Icssg_MdioLinkStateChangeCb)(Icssg_MdioLinkStateChangeInfo *info,
                                           void *appArg);


typedef struct Icssg_mdioLinkIntCfg_s
{
    /*! MDIO Link state change callback function pointer */
    Icssg_MdioLinkStateChangeCb mdioLinkStateChangeCb;

    /*! Application data to be passed to the MDIO link state change callback */
    void *mdioLinkStateChangeCbArg;
    /*! INTC Module mapping data passed by application for configuring PRU to R5F interrupts */
    const PRUICSS_IntcInitData  *prussIntcInitData;
    int32_t                      coreIntrNum;
    uint32_t                     pruEvtNum[ICSSG_MAC_PORT_MAX];
    uint32_t                     isPulseIntr;
    uint32_t                     intrPrio;
} Icssg_mdioLinkIntCfg;

/*!
 * \brief ICSSG peripheral configuration parameters.
 */
typedef struct Icssg_Cfg_s
{
    /*! FDB entry aging period */
    uint64_t agingPeriod;

    /*! Default VLAN Id and PCP for port */
    EnetPort_VlanCfg vlanCfg;

    /*! Configuration of the ICSSG DMA.
     *  Note - In Dual MAC mode, even though separate RX channel is opened for each port,
     *         same dmaCfg is used for both
     */
    const void *dmaCfg;

    /*! Configuration of the resource partition */
    EnetRm_ResCfg resCfg;

    /*! Configuration of the MDIO module */
    Mdio_Cfg mdioCfg;

    /*! TimeSync configuration parameters */
    IcssgTimeSync_Cfg timeSyncCfg;

    /*! MII mode for both MAC ports */
    EnetMacPort_Interface mii;

    /*! Cycle time in nanoseconds */
    uint32_t cycleTimeNs;

    /*! Mdio link interrupt config */
    Icssg_mdioLinkIntCfg mdioLinkIntCfg;

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

    /*!  Max number of QoS Level supported. Used to determine number of hostBufferPoolNum */
    uint32_t qosLevels;

    /*! Whether premption Queue is enabled or not  */
    uint32_t isPremQueEnable;

    /*! Clock type in firmware */
    IcssgTimeSync_ClkType clockTypeFw;

    /*! ICSSG custom firmware configuration: image addresses and sizes.
     *  - Switch peripheral (#ENET_ICSSG_SWITCH), application must populate all
     *    firmwares entries of this array.
     *  - Dual-MAC peripheral (#ENET_ICSSG_DUALMAC), application must populate
     *    only the first firmware entry. */
    Icssg_custom_Fw fw[ICSSG_MAC_PORT_MAX];
} Icssg_Cfg;

/*!
 * \brief ICSSG mac port configuration parameters.
 */
typedef struct IccsgMacPort_Cfg_s
{
    /*! Whether promiscuous mode is enabled at open time or not */
    bool promiscEn;

    /*! Whether unknown unicast flooding to host port is enabled or not */
    bool ucastFloodEn;

    /*! Whether multicast flooding to host port is enabled or not */
    bool mcastFloodEn;

    /*! Acceptable frame check criteria */
    Icssg_AcceptFrameCheck acceptFrameCheck;

    /*! Default VLAN Id and PCP  */
    EnetPort_VlanCfg vlanCfg;

    /*! Queue preemption mode */
    Icssg_QueuePreemptMode queuePreemptMode[ENET_PRI_NUM];

    /*! Queue forward mode */
    Icssg_QueueForwardMode queueForwardMode[ENET_PRI_NUM];

    uint8_t specialFramePrio;
} IcssgMacPort_Cfg;


/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/*!
 * \brief ICSSG memory pool callback function.
 *
 * Initializes the ICSSG memory pool callback function.
 *
 * \param enetType  Enet Peripheral type
 * \param instId    Instance Number
 */
const Icssg_FwPoolMem *EnetCb_GetFwPoolMem(Enet_Type enetType,
                                      uint32_t instId);

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief Initialises Mac Port COnfiguration
 *
 * Gets Application configuration related to ICSSG.
 *
 *  \param macPortCfg  Pointer to ICSSG mac port configuration structure
 */
void IcssgMacPort_initCfg(IcssgMacPort_Cfg *macPortCfg);

/*!
 * \brief ICSSG memory pool callback function.
 *
 * Initializes the ICSSG memory pool callback function.
 *
 * \param enetType  Enet Peripheral type
 * \param instId    Instance Number
 * \param macPort   Enet Mac Port
 */
uint32_t Icssg_getSliceNum(Enet_Type enetType,
                           uint32_t instId,
                           Enet_MacPort macPort);

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

#endif /* ICSSG_H_ */

/*! @} */
