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
 * \file  enet_per.h
 *
 * \brief This file contains the type definitions and helper macros for the
 *        Enet Peripheral interface.
 */

#ifndef ENET_PER_H_
#define ENET_PER_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <include/core/enet_types.h>
#include <include/core/enet_ioctl.h>
#include <include/core/enet_mod_macport.h>
#include <include/phy/enetphy.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/*! \brief Helper macro to create IOCTL commands for peripherals. */
#define ENET_PER_PUBLIC_IOCTL(x)              (ENET_IOCTL_TYPE_PUBLIC | \
                                               ENET_IOCTL_PER_BASE |    \
                                               ENET_IOCTL_MIN(x))

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief Ethernet peripheral IOCTL commands.
 */
enum EnetPer_Ioctl_e
{
    /*!
     * \brief Get the hardware version of the peripheral.
     *
     * IOCTL parameters:
     * -  inArgs: None
     * - outArgs: #Enet_Version
     */
    ENET_PER_IOCTL_GET_VERSION = ENET_PER_PUBLIC_IOCTL(0U),

    /*!
     * \brief Print registers of the peripheral and all its modules.
     *
     * IOCTL parameters:
     * -  inArgs: None
     * - outArgs: None
     */
    ENET_PER_IOCTL_PRINT_REGS = ENET_PER_PUBLIC_IOCTL(1U),

    /*!
     * \brief Open port link (MAC port and PHY).
     *
     * IOCTL parameters:
     * -  inArgs: #EnetPer_PortLinkCfg
     * - outArgs: None
     */
    ENET_PER_IOCTL_OPEN_PORT_LINK = ENET_PER_PUBLIC_IOCTL(2U),

    /*!
     * \brief Close port link (MAC port and PHY).
     *
     * IOCTL parameters:
     * -  inArgs: #Enet_MacPort
     * - outArgs: None
     */
    ENET_PER_IOCTL_CLOSE_PORT_LINK = ENET_PER_PUBLIC_IOCTL(3U),

    /*!
     * \brief Check if port link is up.
     *
     * IOCTL parameters:
     * -  inArgs: #Enet_MacPort
     * - outArgs: bool
     */
    ENET_PER_IOCTL_IS_PORT_LINK_UP = ENET_PER_PUBLIC_IOCTL(4U),

    /*!
     * \brief Get port link configuration (speed and duplexity).
     *
     * IOCTL parameters:
     * -  inArgs: #Enet_MacPort
     * - outArgs: #EnetMacPort_LinkCfg
     */
    ENET_PER_IOCTL_GET_PORT_LINK_CFG = ENET_PER_PUBLIC_IOCTL(5U),

    /*!
     * \brief Attach core to Ethernet peripheral.
     *
     * IOCTL parameters:
     * -  inArgs: uint32_t
     * - outArgs: #EnetPer_AttachCoreOutArgs
     */
    ENET_PER_IOCTL_ATTACH_CORE = ENET_PER_PUBLIC_IOCTL(6U),

    /*!
     * \brief Detach core from Ethernet peripheral using its core key.
     *
     * IOCTL parameters:
     * -  inArgs: uint32_t
     * - outArgs: None
     */
    ENET_PER_IOCTL_DETACH_CORE = ENET_PER_PUBLIC_IOCTL(7U),

    /*!
     * \brief Register default flow to a specific rx flow
     *
     * IOCTL parameters:
     * -  inArgs: #Enet_DfltFlowInfo
     * - outArgs: None
     */
    ENET_IOCTL_REGISTER_RX_DEFAULT_FLOW = ENET_PER_PUBLIC_IOCTL(8U),

    /*!
     * \brief Unregister default flow
     *
     * After unregistering default flow, default flow traffic will be directed
     * to CPSW internal reserved flow where they will be dropped.
     *
     * IOCTL parameters:
     * -  inArgs: #Enet_DfltFlowInfo
     * - outArgs: None
     */
    ENET_IOCTL_UNREGISTER_RX_DEFAULT_FLOW = ENET_PER_PUBLIC_IOCTL(9U),

    /*!
     * \brief Register destination MAC address to a specific rx flow
     *
     * Note that CPSW supports associating multiple L2/L3 header fields to a
     * specific rx flow. Refer ALE API #CPSW_ALE_IOCTL_SET_POLICER for details
     * on how to associate fields other than DST MAC to a specific flow.
     *
     * IOCTL parameters:
     * -  inArgs: #Enet_MacDstFlowInfo
     * - outArgs: None
     */
    ENET_IOCTL_REGISTER_DSTMAC_RX_FLOW = ENET_PER_PUBLIC_IOCTL(10U),

    /*!
     * \brief Unregister destination MAC address to a specific flow
     *
     * After unregistering dstmac, traffic with associated MAC address will be
     * directed to default flow.
     *
     * IOCTL parameters:
     * -  inArgs: #Enet_MacDstFlowInfo
     * - outArgs: None
     */
    ENET_IOCTL_UNREGISTER_DSTMAC_RX_FLOW = ENET_PER_PUBLIC_IOCTL(11U),

    /*!
     * \brief Enable VLAN aware mode.
     *
     * Enables VLAN aware mode at peripheral level.
     *
     * IOCTL parameters:
     * -  inArgs: None
     * - outArgs: None
     */
    ENET_PER_IOCTL_SET_VLAN_AWARE = ENET_PER_PUBLIC_IOCTL(12U),

    /*!
     * \brief Disable VLAN aware mode.
     *
     * Disables VLAN aware mode at peripheral level.
     *
     * IOCTL parameters:
     * -  inArgs: None
     * - outArgs: None
     */
    ENET_PER_IOCTL_SET_VLAN_UNAWARE = ENET_PER_PUBLIC_IOCTL(13U),

    /*!
     * \brief Handle link up event for an externally managed PHY
     *
     * IOCTL params:
     * -  inArgs: #Enet_ExtPhyLinkUpEventInfo
     * - outArgs: None
     *
     * Type: Synchronous.
     */
    ENET_PER_IOCTL_HANDLE_EXTPHY_LINKUP_EVENT = ENET_PER_PUBLIC_IOCTL(14U),

    /*!
     * \brief Handle link down event for an externally managed PHY.
     *
     * IOCTL params:
     * -  inArgs: #
     * - outArgs: None
     *
     * Type: Synchronous.
     */
    ENET_PER_IOCTL_HANDLE_EXTPHY_LINKDOWN_EVENT = ENET_PER_PUBLIC_IOCTL(15U),

    /*!
     * \brief Register IOCTL handler
     *
     * IOCTL params:
     * -  inArgs: #Enet_IoctlRegisterHandlerInArgs
     * - outArgs: None
     *
     * Type: Synchronous.
     */
    ENET_PER_IOCTL_REGISTER_IOCTL_HANDLER = ENET_PER_PUBLIC_IOCTL(16U),

    /*!
     * \brief Setting PHY ISOLATE mode
     *
     * Settting the PHY to ISOLATE mode
     *
     * IOCTL parameters:
     * -  inArgs: None
     * - outArgs: None
     */
    ENET_PER_IOCTL_SET_ISOLATE_STATE = ENET_PER_PUBLIC_IOCTL(17U),

    /*!
     * \brief Clear PHY ISOLATE mode
     *
     * Clear the PHY ISOLATE mode
     *
     * IOCTL parameters:
     * -  inArgs: None
     * - outArgs: None
     */
    ENET_PER_IOCTL_CLEAR_ISOLATE_STATE = ENET_PER_PUBLIC_IOCTL(17U),

};

/*!
 * \brief Rx Default Flow Info used for default flow registration/unregistration.
 */
typedef struct Enet_DfltFlowInfo_s
{
    /*! Core key */
    uint32_t coreKey;

    /*! 0-relative channel index */
    uint32_t chIdx;

    /*! RX flow base or start index */
    uint32_t startIdx;

    /*! Allocated flow's index (offset from #startIdx) */
    uint32_t flowIdx;
} Enet_DfltFlowInfo;

/*!
 * \brief Output args for #ENET_PER_IOCTL_ATTACH_CORE command
 */
typedef struct Enet_MacDstFlowInfo_s
{
    /*! Core Key */
    uint32_t coreKey;

    /*! Rx Flow Base or Start index */
    uint32_t startIdx;

    /*! Allocated flow's index (offset from #startIdx) */
    uint32_t flowIdx;

    /*! Destination mac address associated with the flow */
    uint8_t macAddress[ENET_MAC_ADDR_LEN];
} Enet_MacDstFlowInfo;

/*!
 * \brief Input args for #ENET_PER_IOCTL_OPEN_PORT_LINK command.
 */
typedef struct EnetPer_PortLinkCfg_s
{
    /*! Port number */
    Enet_MacPort macPort;

    /*! MAC port configuration */
    void *macCfg;

    /*! PHY configuration */
    EnetPhy_Cfg phyCfg;

    /*! MAC port MII interface */
    EnetMacPort_Interface mii;

    /*! Link configuration (speed and duplexity) */
    EnetMacPort_LinkCfg linkCfg;
} EnetPer_PortLinkCfg;

/*!
 * \brief Output args for #ENET_PER_IOCTL_ATTACH_CORE command
 */
typedef struct EnetPer_AttachCoreOutArgs_s
{
    /*! Core key */
    uint32_t coreKey;

    /*! Host Port RX MTU */
    uint32_t rxMtu;

    /*! TX MTU per priority */
    uint32_t txMtu[ENET_PRI_NUM];
} EnetPer_AttachCoreOutArgs;


/*!
 * \brief Input args for #ENET_PER_IOCTL_HANDLE_EXTPHY_LINKUP_EVENT command
 */
typedef struct Enet_ExtPhyLinkUpEventInfo_s
{
    /*! Mac port for which link up event occured */
    Enet_MacPort macPort;

    /*! PHY link info as determined by the externally managed PHY */
    EnetPhy_LinkCfg phyLinkCfg;
} Enet_ExtPhyLinkUpEventInfo;

/*!
 * \brief #ENET_PER_IOCTL_REGISTER_IOCTL_HANDLER inArgs
 */
typedef struct Enet_IoctlRegisterHandlerInArgs_s
{
    /*! IOCTL cmd */
    uint32_t cmd;

    /*! Address of IOCTL handler function */
    uintptr_t fxn;
} Enet_IoctlRegisterHandlerInArgs;

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

#endif /* ENET_PER_H_ */
