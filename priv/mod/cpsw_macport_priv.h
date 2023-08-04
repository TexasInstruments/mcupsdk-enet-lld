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
 * \file  cpsw_macport_priv.h
 *
 * \brief This file contains private type definitions and helper macros for the
 *        CPSW MAC port module which are meant for internal use in Enet Per
 *        drivers.
 */

#ifndef CPSW_MACPORT_PRIV_H_
#define CPSW_MACPORT_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#if ENET_CFG_IS_ON(CPSW_MACPORT_EST)
#include <include/core/enet_mod_tas.h>
#endif
#include <include/core/enet_mod_macport.h>
#include <include/mod/cpsw_macport.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/*! \brief Helper macro to create private IOCTL commands for CPSW MAC port module. */
#define CPSW_MACPORT_PRIVATE_IOCTL(x)         (ENET_IOCTL_TYPE_PRIVATE | \
                                               ENET_IOCTL_MACPORT_BASE | \
                                               ENET_IOCTL_PER_CPSW |     \
                                               ENET_IOCTL_MIN(x))

/*! \brief SGMII feature mask. */
#define CPSW_MACPORT_FEATURE_SGMII            (ENET_BIT(0U))

/*! \brief InterVLAN feature mask. */
#define CPSW_MACPORT_FEATURE_INTERVLAN        (ENET_BIT(1U))

/*! \brief EST feature mask. */
#define CPSW_MACPORT_FEATURE_EST              (ENET_BIT(2U))

/*! \brief Base InterVLAN Route Id. */
#define CPSW_MACPORT_INTERVLAN_ROUTEID_BASE   (1U)

/*! \brief Maximum number of interVLAN routes available per egress port. */
#define CPSW_MACPORT_INTERVLAN_ROUTE_MAX      (CPSW_MACPORT_INTERVLAN_ROUTEID_LAST - \
                                               CPSW_MACPORT_INTERVLAN_ROUTEID_FIRST + 1U)

/*! \brief Convert route id to pointer. */
#define CPSW_MACPORT_INTERVLAN_ROUTE2PTR(routeId) (CPSW_MACPORT_INTERVLAN_ROUTEID_BASE + \
                                                   (routeId) - CPSW_MACPORT_INTERVLAN_ROUTEID_FIRST)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief MAC port private IOCTL commands.
 */
typedef enum CpswMacPort_PrivIoctls_e
{
    /*!
     * \brief Enable MAC port.
     *
     * IOCTL parameters:
     *   inArgs: #EnetMacPort_LinkCfg
     *  outArgs: None
     */
    CPSW_MACPORT_IOCTL_ENABLE = CPSW_MACPORT_PRIVATE_IOCTL(0U),

    /*!
     * \brief Disable MAC port.
     *
     * IOCTL parameters:
     *   inArgs: None
     *  outArgs: None
     */
    CPSW_MACPORT_IOCTL_DISABLE = CPSW_MACPORT_PRIVATE_IOCTL(1U),

    /*!
     * \brief Alloc free route and set interVLAN config for egress MAC port.
     *
     * IOCTL parameters:
     *   inArgs: #CpswMacPort_InterVlanRoutingCfg
     *  outArgs: #CpswMacPort_InterVlanRouteId
     */
    CPSW_MACPORT_IOCTL_SET_INTERVLAN_ROUTE = CPSW_MACPORT_PRIVATE_IOCTL(2U),

    /*!
     * \brief Set interVLAN config for egress MAC port at given route index.
     *
     * IOCTL parameters:
     *   inArgs: #CpswMacPort_SetSpecificInterVlanRouteInArgs
     *  outArgs: None
     */
    CPSW_MACPORT_IOCTL_SET_SPECIFIC_INTERVLAN_ROUTE = CPSW_MACPORT_PRIVATE_IOCTL(3U),

    /*!
     * \brief Get interVLAN config for egress MAC port.
     *
     * IOCTL parameters:
     *   inArgs: #CpswMacPort_InterVlanRouteId
     *  outArgs: #CpswMacPort_InterVlanRoutingCfg
     */
    CPSW_MACPORT_IOCTL_GET_INTERVLAN_ROUTE = CPSW_MACPORT_PRIVATE_IOCTL(4U),

    /*!
     * \brief Delete the specified interVlan route.
     *
     * IOCTL parameters:
     *   inArgs: CpswMacPort_InterVlanRoutingCfg
     *  outArgs: #CpswMacPort_InterVlanRouteId
     */
    CPSW_MACPORT_IOCTL_DELETE_INTERVLAN_ROUTE = CPSW_MACPORT_PRIVATE_IOCTL(5U),

    /*!
     * \brief Get interVLAN free routes available for the given egress port.
     *
     * IOCTL parameters:
     *   inArgs: None
     *  outArgs: #CpswMacPort_InterVlanFreeRouteInfo
     */
    CPSW_MACPORT_IOCTL_GET_INTERVLAN_FREEROUTES = CPSW_MACPORT_PRIVATE_IOCTL(6U),

    /*!
     * \brief Find interVLAN route id matching the given route params.
     *
     * If unable to find the route returns CPSW_MACPORT_EROUTENOTFOUND
     * IOCTL parameters:
     *   inArgs: #CpswMacPort_InterVlanRoutingCfg
     *  outArgs: #CpswMacPort_InterVlanRouteId
     */
    CPSW_MACPORT_IOCTL_FIND_INTERVLAN_ROUTE = CPSW_MACPORT_PRIVATE_IOCTL(7U),

    /*!
     * \brief Check if given interVLAN route is free.
     *
     * IOCTL parameters:
     *   inArgs: #CpswMacPort_InterVlanRouteId
     *  outArgs: bool
     */
    CPSW_MACPORT_IOCTL_IS_INTERVLAN_ROUTE_FREE = CPSW_MACPORT_PRIVATE_IOCTL(8U),

    /*!
     * \brief Set transmit short gap configuration for MAC port.
     *
     * IOCTL parameters:
     *   inArgs: #CpswMacPort_PortTxShortIpgCfg
     *  outArgs: None
     */
    CPSW_MACPORT_IOCTL_SET_SHORT_IPG = CPSW_MACPORT_PRIVATE_IOCTL(9U),

    /*!
     * \brief Get transmit short gap configuration for MAC port.
     *
     * IOCTL parameters:
     *   inArgs: #EnetMacPort_GenericInArgs
     *  outArgs: #CpswMacPort_TxShortIpgCfg
     */
    CPSW_MACPORT_IOCTL_GET_SHORT_IPG = CPSW_MACPORT_PRIVATE_IOCTL(10U),

    /*!
     * \brief Get SGMII auto-negotiate link status from the SGMII status register.
     *
     * IOCTL parameters:
     *   inArgs: #EnetMacPort_GenericInArgs
     *  outArgs: bool
     */
    CPSW_MACPORT_IOCTL_GET_SGMII_AUTONEG_LINK_STATUS = CPSW_MACPORT_PRIVATE_IOCTL(11U),

    /*!
     * \brief Get SGMII link status from the SGMII status register.
     *
     * IOCTL parameters:
     *   inArgs: #EnetMacPort_GenericInArgs
     *  outArgs: bool
     */
    CPSW_MACPORT_IOCTL_GET_SGMII_LINK_STATUS = CPSW_MACPORT_PRIVATE_IOCTL(12U),

    /*!
     * \brief Register IOCTL handler
     *
     * IOCTL params:
     * -  inArgs: #Enet_IoctlRegisterHandlerInArgs
     * - outArgs: None
     *
     * Type: Synchronous.
     */
    CPSW_MACPORT_IOCTL_REGISTER_HANDLER = CPSW_MACPORT_PRIVATE_IOCTL(13U),

} CpswMacPort_PrivIoctls;

/*!
 * \brief Input args for #CPSW_MACPORT_IOCTL_SET_SPECIFIC_INTERVLAN_ROUTE command.
 */
typedef struct CpswMacPort_SetSpecificInterVlanRouteInArgs_s
{
    /*! InterVLAN egress port route index. It can be value from 1 - 4 */
    CpswMacPort_InterVlanRouteId routeId;

    /*! Egress route interVLAN configuration to be set for the route */
    CpswMacPort_InterVlanRoutingCfg routeCfg;
} CpswMacPort_SetSpecificInterVlanRouteInArgs;

/*!
 * \brief Information on free interVLAN routes available for an egress port.
 */
typedef struct CpswMacPort_InterVlanFreeRouteInfo_s
{
    /*! Number of free interVLAN routes available */
    uint32_t numFreeRoutes;

    /*! Array of free route indexes */
    CpswMacPort_InterVlanRouteId freeRouteId[CPSW_MACPORT_INTERVLAN_ROUTE_MAX];
} CpswMacPort_InterVlanFreeRouteInfo;

/*!
 * \brief MAC port module configuration parameters.
 *
 * This is the MAC port configuration type expected to be passed to the
 * EnetMod_open() function, as it includes no only the MAC port specific
 * configuration but also the configuration parameters shared with PHY,
 * such as MII interface type and link configuration.
 */
typedef struct CpswMacPort_ModCfg_s
{
    /*! MAC port specific configuration */
    CpswMacPort_Cfg macCfg;

    /*! MAC port MII interface */
    EnetMacPort_Interface mii;

    /*! Link configuration (speed and duplexity) */
    EnetMacPort_LinkCfg linkCfg;
} CpswMacPort_ModCfg;

/*!
 * \brief CPSW MAC port object.
 */
typedef struct CpswMacPort_Obj_s
{
    /*! EnetMod must be the first member */
    EnetMod_Obj enetMod;

    /*! MAC port number */
    Enet_MacPort macPort;

    /*! Ethernet peripheral type. Required to query SoC parameters (clock freq) */
    Enet_Type enetType;

    /*! Peripheral instance number. Required to query SoC parameters (clock freq) */
    uint32_t instId;

    /*! Whether MAC port is enabled (i.e. PHY is linked) */
    bool enabled;

    /*! Current speed and duplexity. Valid when 'enabled' field is true */
    EnetMacPort_LinkCfg linkCfg;

#if ENET_CFG_IS_ON(CPSW_MACPORT_EST)
    /*! Current EST module state (enabled or disabled) */
    EnetTas_TasState state;

    /*! Last administrative list successfully set */
    EnetTas_ControlList adminList;

    /*! Current operational list */
    EnetTas_ControlList operList;

    /*! Admin EST buffer bank: lower or upper */
    bool estBufUpper;

    /*! Config state machine status */
    EnetTas_ConfigStatus configStatus;
#endif

    /*! Saving Macport config before reset */
    CpswMacPort_ModCfg macModCfgCtxt;
} CpswMacPort_Obj;

/*!
 * \brief MAC port module handle.
 */
typedef CpswMacPort_Obj *CpswMacPort_Handle;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief Open and initialize CPSW MAC port.
 *
 * \param hMod      Enet Module handle
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 * \param cfg       Configuration parameters
 * \param cfgSize   Size of the configuration parameters
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t CpswMacPort_open(EnetMod_Handle hMod,
                         Enet_Type enetType,
                         uint32_t instId,
                         const void *cfg,
                         uint32_t cfgSize);

/*!
 * \brief Rejoin a running CPSW MAC port.
 *
 * \param hMod      Enet Module handle
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t CpswMacPort_rejoin(EnetMod_Handle hMod,
                           Enet_Type enetType,
                           uint32_t instId);

/*!
 * \brief Run an IOCTL operation on CPSW MAC port.
 *
 * \param hMod         Enet Module handle
 * \param cmd          IOCTL command Id
 * \param prms         IOCTL parameters
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t CpswMacPort_ioctl(EnetMod_Handle hMod,
                          uint32_t cmd,
                          Enet_IoctlPrms *prms);

/*!
 * \brief Close CPSW MAC port.
 *
 * \param hMod         Enet Module handle
 */
void CpswMacPort_close(EnetMod_Handle hMod);

/*!
 * \brief Saves and Close CPSW MAC port.
 *
 * \param hMod         Enet Module handle
 */
void CpswMacPort_saveCtxt(EnetMod_Handle hMod);

/*!
 * \brief Restores and Open CPSW MAC port.
 *
 * \param hMod      Enet Module handle
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 * \param cfg       Configuration parameters
 * \param cfgSize   Size of the configuration parameters
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t CpswMacPort_restoreCtxt(EnetMod_Handle hMod,
                                Enet_Type enetType,
                                uint32_t instId,
                                const void *cfg,
                                uint32_t cfgSize);

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

#endif /* CPSW_MACPORT_PRIV_H_ */
