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
 * \file  enet_mod_macport.h
 *
 * \brief This file contains the type definitions and helper macros for the
 *        Enet MAC port module interface.
 */

/*!
 * \ingroup  DRV_ENET_MODULE
 * \defgroup ENET_MOD_MACPORT Enet MAC Port
 *
 * @{
 */

#ifndef ENET_MOD_MACPORT_H_
#define ENET_MOD_MACPORT_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <include/core/enet_mod.h>
#include <include/core/enet_mod_port.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/*! \brief Helper macro to create IOCTL commands for MAC port module */
#define ENET_MACPORT_PUBLIC_IOCTL(x)          (ENET_IOCTL_TYPE_PUBLIC |  \
                                               ENET_IOCTL_MACPORT_BASE | \
                                               ENET_IOCTL_MIN(x))

/*! \brief Helper macro to create private IOCTL commands for MAC Port module. */
#define ENET_MACPORT_PRIVATE_IOCTL(x)         (ENET_IOCTL_TYPE_PRIVATE | \
                                              ENET_IOCTL_MACPORT_BASE |    \
                                              ENET_IOCTL_MIN(x))
/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief MAC port IOCTL commands.
 */
typedef enum EnetMacPort_Ioctl_e
{
    /*!
     * \brief Get the hardware version of the MAC port module.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetMacPort_GenericInArgs
     * - outArgs: #Enet_Version
     */
    ENET_MACPORT_IOCTL_GET_VERSION = ENET_MACPORT_PUBLIC_IOCTL(0U),

    /*!
     * \brief Print MAC port registers.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetMacPort_GenericInArgs
     * - outArgs: None
     */
    ENET_MACPORT_IOCTL_PRINT_REGS = ENET_MACPORT_PUBLIC_IOCTL(1U),

    /*!
     * \brief Set ingress DSCP priority (TOS) map.
     *
     * NOTE:
     * - For ICSSG upto 7 DSCP values can have traffic class mapping,
     *   All others will be treated as best effort traffic.
     * - For ICSSG only IPv4 based priority mapping is supported and
     *   Ipv6 based is not supported.
     * - For ICSSG incase of local generated traffic for switch mode
     *   expecting both ports has same mapping and if it is undirected
     *   traffic priorities are taken from Port1 mapping
     *
     * IOCTL parameters:
     * -  inArgs: #EnetMacPort_SetIngressDscpPriorityMapInArgs
     * - outArgs: None
     */
    ENET_MACPORT_IOCTL_SET_INGRESS_DSCP_PRI_MAP = ENET_MACPORT_PUBLIC_IOCTL(2U),

    /*!
     * \brief Get ingress DSCP priority (TOS) map.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetMacPort_GenericInArgs
     * - outArgs: #EnetPort_DscpPriorityMap
     */
    ENET_MACPORT_IOCTL_GET_INGRESS_DSCP_PRI_MAP = ENET_MACPORT_PUBLIC_IOCTL(3U),

    /*!
     * \brief Set VLAN priority regeneration map.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetMacPort_SetPriorityRegenMapInArgs
     * - outArgs: None
     */
    ENET_MACPORT_IOCTL_SET_PRI_REGEN_MAP = ENET_MACPORT_PUBLIC_IOCTL(4U),

    /*!
     * \brief Get VLAN priority regeneration map.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetMacPort_GenericInArgs
     * - outArgs: #EnetPort_PriorityMap
     */
    ENET_MACPORT_IOCTL_GET_PRI_REGEN_MAP = ENET_MACPORT_PUBLIC_IOCTL(5U),

    /*!
     * \brief Set QoS egress priority map.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetMacPort_SetEgressPriorityMapInArgs
     * - outArgs: None
     */
    ENET_MACPORT_IOCTL_SET_EGRESS_QOS_PRI_MAP = ENET_MACPORT_PUBLIC_IOCTL(6U),

    /*!
     * \brief Get QoS egress priority map.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetMacPort_GenericInArgs
     * - outArgs: #EnetPort_PriorityMap
     */
    ENET_MACPORT_IOCTL_GET_EGRESS_QOS_PRI_MAP = ENET_MACPORT_PUBLIC_IOCTL(7U),

    /*!
     * \brief Enable egress traffic shaping configuration.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetMacPort_EnableEgressTrafficShapingInArgs
     * - outArgs: None
     */
    ENET_MACPORT_IOCTL_ENABLE_EGRESS_TRAFFIC_SHAPING = ENET_MACPORT_PUBLIC_IOCTL(8U),

    /*!
     * \brief Disable egress traffic shaping configuration.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetMacPort_GenericInArgs
     * - outArgs: None
     */
    ENET_MACPORT_IOCTL_DISABLE_EGRESS_TRAFFIC_SHAPING = ENET_MACPORT_PUBLIC_IOCTL(9U),

    /*!
     * \brief Get egress traffic shaping configuration.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetMacPort_GenericInArgs
     * - outArgs: #EnetPort_TrafficShapingCfg
     */
    ENET_MACPORT_IOCTL_GET_EGRESS_TRAFFIC_SHAPING = ENET_MACPORT_PUBLIC_IOCTL(10U),

    /*!
     * \brief Get MRU and MTU.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetMacPort_GenericInArgs
     * - outArgs: #EnetPort_MaxLen
     */
    ENET_MACPORT_IOCTL_GET_MAXLEN = ENET_MACPORT_PUBLIC_IOCTL(11U),

    /*!
     * \brief Get MAC port's speed and duplexity.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetMacPort_GenericInArgs
     * - outArgs: #EnetMacPort_LinkCfg
     */
    ENET_MACPORT_IOCTL_GET_LINK_CFG = ENET_MACPORT_PUBLIC_IOCTL(12U),

} EnetMacPort_Ioctl;

/*!
 * \brief MAC interface layer type.
 */
typedef enum EnetMac_LayerType_e
{
    /*! Media-Independent Interface (MII) layer */
    ENET_MAC_LAYER_MII   = 1U,

    /*! Gigabit Media-Independent Interface (GMII) layer */
    ENET_MAC_LAYER_GMII  = 0U,

    /*! 10-Gigabit Media-Independent Interface (XGMII) layer */
    ENET_MAC_LAYER_XGMII = 2U,
} EnetMac_LayerType;

/*!
 * \brief MAC interface sublayer type.
 */
typedef enum EnetMac_SublayerType_e
{
    /*! Standard interface sublayer */
    ENET_MAC_SUBLAYER_STANDARD = 0U,

    /*! Reduced interface sublayer */
    ENET_MAC_SUBLAYER_REDUCED,

    /*! Serial interface sublayer */
    ENET_MAC_SUBLAYER_SERIAL,

    /*! Quad-serial interface sublayer (main port) */
    ENET_MAC_SUBLAYER_QUAD_SERIAL_MAIN,

    /*! Quad-serial interface sublayer (sub port) */
    ENET_MAC_SUBLAYER_QUAD_SERIAL_SUB,
} EnetMac_SublayerType;

/*!
 * \brief MAC interface variant (only for specific interface types).
 */
typedef enum EnetMac_VariantType_e
{
    /*! No variant */
    ENET_MAC_VARIANT_NONE = 0U,

    /*! Forced mode (out of band) */
    ENET_MAC_VARIANT_FORCED,
} EnetMac_VariantType;

/*!
 * \brief SGMII mode.
 */
typedef enum EnetMac_SgmiiMode_e
{
    /*! Invalid mode. Set to when port is used in non Q/SGMII mode */
    ENET_MAC_SGMIIMODE_INVALID = 0U,

    /*! SGMII in fiber mode with PHY connection */
    ENET_MAC_SGMIIMODE_FIBER_WITH_PHY,

    /*! SGMII in SGMII mode with PHY connection */
    ENET_MAC_SGMIIMODE_SGMII_WITH_PHY,

    /*! SGMII in MAC to MAC with auto-neg master mode */
    ENET_MAC_SGMIIMODE_SGMII_AUTONEG_MASTER,

    /*! SGMII in MAC to MAC with auto-neg slave mode */
    ENET_MAC_SGMIIMODE_SGMII_AUTONEG_SLAVE,

    /*! SGMII in MAC to MAC with forced master mode */
    ENET_MAC_SGMIIMODE_SGMII_FORCEDLINK,
} EnetMac_SgmiiMode;

/*!
 * \brief MAC port interface.
 */
typedef struct EnetMacPort_Interface_s
{
    /*! Interface layer type */
    EnetMac_LayerType layerType;

    /*! Interface sublayer type */
    EnetMac_SublayerType sublayerType;

    /*! Variant type */
    EnetMac_VariantType variantType;
} EnetMacPort_Interface;

/*!
 * \brief Link speed and duplexity configuration.
 */
typedef struct EnetMacPort_LinkCfg_s
{
    /*! Link speed */
    Enet_Speed speed;

    /*! Duplexity */
    Enet_Duplexity duplexity;
} EnetMacPort_LinkCfg;

/*!
 * \brief VLAN usage type.
 */
typedef enum EnetMacPort_VlanType_e
{
    /*! No VLAN tag */
    ENET_MACPORT_VLAN_TYPE_NONE = 0x00U,

    /*! Single VLAN tag */
    ENET_MACPORT_VLAN_TYPE_SINGLE_TAG = 0x01U,

    /*! Stacked VLAN tag */
    ENET_MACPORT_VLAN_TYPE_STACKED_TAGS = 0x02U,
} EnetMacPort_VlanType;

/*!
 * \brief Generic MAC port input args.
 */
typedef struct EnetMacPort_GenericInArgs_s
{
    /*! Port number */
    Enet_MacPort macPort;
} EnetMacPort_GenericInArgs;

/*!
 * \brief Input args for #ENET_MACPORT_IOCTL_SET_INGRESS_DSCP_PRI_MAP command.
 */
typedef struct EnetMacPort_SetIngressDscpPriorityMapInArgs_s
{
    /*! Port number */
    Enet_MacPort macPort;

    /*! Ingress DSCP priority (TOS) map */
    EnetPort_DscpPriorityMap dscpPriorityMap;
} EnetMacPort_SetIngressDscpPriorityMapInArgs;

/*!
 * \brief Input args for #ENET_MACPORT_IOCTL_SET_PRI_REGEN_MAP command.
 */
typedef struct EnetMacPort_SetPriorityRegenMapInArgs_s
{
    /*! Port number */
    Enet_MacPort macPort;

    /*! Priority regeneration map */
    EnetPort_PriorityMap priorityRegenMap;
} EnetMacPort_SetPriorityRegenMapInArgs;

/*!
 * \brief Input args for #ENET_MACPORT_IOCTL_SET_EGRESS_QOS_PRI_MAP command.
 */
typedef struct EnetMacPort_SetEgressPriorityMapInArgs_s
{
    /*! Port number */
    Enet_MacPort macPort;

    /*! Egress priority map */
    EnetPort_PriorityMap priorityMap;
} EnetMacPort_SetEgressPriorityMapInArgs;

/*!
 * \brief Input args for #ENET_MACPORT_IOCTL_ENABLE_EGRESS_TRAFFIC_SHAPING command.
 */
typedef struct EnetMacPort_EnableEgressTrafficShapingInArgs_s
{
    /*! Port number */
    Enet_MacPort macPort;

    /*! Traffic shaping configuration for all priorities */
    EnetPort_TrafficShapingCfg trafficShapingCfg;
} EnetMacPort_EnableEgressTrafficShapingInArgs;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief Check if interface is MII.
 *
 * \param mii       MAC port interface
 *
 * \return true if MAC port interface is MII, otherwise false.
 */
static inline bool EnetMacPort_isMii(const EnetMacPort_Interface *mii);

/*!
 * \brief Check if interface is RMII.
 *
 * \param mii       MAC port interface
 *
 * \return true if MAC port interface is RMII, otherwise false.
 */
static inline bool EnetMacPort_isRmii(const EnetMacPort_Interface *mii);

/*!
 * \brief Check if interface is GMII.
 *
 * \param mii       MAC port interface
 *
 * \return true if MAC port interface is GMII, otherwise false.
 */
static inline bool EnetMacPort_isGmii(const EnetMacPort_Interface *mii);

/*!
 * \brief Check if interface is RGMII.
 *
 * \param mii       MAC port interface
 *
 * \return true if MAC port interface is RGMII, otherwise false.
 */
static inline bool EnetMacPort_isRgmii(const EnetMacPort_Interface *mii);

/*!
 * \brief Check if interface is SGMII.
 *
 * \param mii       MAC port interface
 *
 * \return true if MAC port interface is SGMII, otherwise false.
 */
static inline bool EnetMacPort_isSgmii(const EnetMacPort_Interface *mii);

/*!
 * \brief Check if interface is QSGMII.
 *
 * \param mii       MAC port interface
 *
 * \return true if MAC port interface is QSGMII, otherwise false.
 */
static inline bool EnetMacPort_isQsgmii(const EnetMacPort_Interface *mii);

/*!
 * \brief Check if interface is XFI.
 *
 * \param mii       MAC port interface
 *
 * \return true if MAC port interface is XFI, otherwise false.
 */
static inline bool EnetMacPort_isXfi(const EnetMacPort_Interface *mii);

/* ========================================================================== */
/*                        Deprecated Function Declarations                    */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

static inline bool EnetMacPort_isMii(const EnetMacPort_Interface *mii)
{
    return (mii->layerType == ENET_MAC_LAYER_MII) &&
           (mii->sublayerType == ENET_MAC_SUBLAYER_STANDARD);
}

static inline bool EnetMacPort_isRmii(const EnetMacPort_Interface *mii)
{
    return (mii->layerType == ENET_MAC_LAYER_MII) &&
           (mii->sublayerType == ENET_MAC_SUBLAYER_REDUCED);
}

static inline bool EnetMacPort_isGmii(const EnetMacPort_Interface *mii)
{
    return (mii->layerType == ENET_MAC_LAYER_GMII) &&
           (mii->sublayerType == ENET_MAC_SUBLAYER_STANDARD);
}

static inline bool EnetMacPort_isRgmii(const EnetMacPort_Interface *mii)
{
    return (mii->layerType == ENET_MAC_LAYER_GMII) &&
           (mii->sublayerType == ENET_MAC_SUBLAYER_REDUCED);
}

static inline bool EnetMacPort_isSgmii(const EnetMacPort_Interface *mii)
{
    return (mii->layerType == ENET_MAC_LAYER_GMII) &&
           (mii->sublayerType == ENET_MAC_SUBLAYER_SERIAL);
}

static inline bool EnetMacPort_isQsgmii(const EnetMacPort_Interface *mii)
{
    return (mii->layerType == ENET_MAC_LAYER_GMII) &&
           ((mii->sublayerType == ENET_MAC_SUBLAYER_QUAD_SERIAL_MAIN) ||
            (mii->sublayerType == ENET_MAC_SUBLAYER_QUAD_SERIAL_SUB));
}

static inline bool EnetMacPort_isXfi(const EnetMacPort_Interface *mii)
{
    return (mii->layerType == ENET_MAC_LAYER_XGMII) &&
           (mii->sublayerType == ENET_MAC_SUBLAYER_STANDARD);
}

#ifdef __cplusplus
}
#endif

#endif /* ENET_MOD_MACPORT_H_ */

/*! @} */
