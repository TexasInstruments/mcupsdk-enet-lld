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

/*! \brief Convert peripheral specific handle to EnetMod generic handle. */
#define ENET_TO_PER(per)                      ((EnetPer_Handle)(per))

/*! \brief Get peripheral name. */
#define ENET_PER_NAME(per)                    (ENET_TO_PER(per)->name)

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

/*!
 * \brief Ethernet Peripheral handle.
 *
 * Ethernet Peripheral handle used to call any EnetPer related APIs.
 */
typedef struct EnetPer_Obj_s *EnetPer_Handle;

/*!
 * \brief Initialize peripheral's configuration parameters.
 *
 * Initializes the configuration parameter of the Enet Peripheral.
 *
 * \param hPer      Enet Peripheral handle
 * \param enetType  Enet Peripheral type
 * \param cfg       Configuration parameters to be initialized
 * \param cfgSize   Size of the configuration parameters
 */
typedef void (* const EnetPer_InitCfg)(EnetPer_Handle hPer,
                                Enet_Type enetType,
                                void *cfg,
                                uint32_t cfgSize);

/*!
 * \brief Open and initialize the Enet Peripheral.
 *
 * Opens and initializes the Enet Peripheral with the configuration parameters
 * provided by the caller.
 *
 * \param hPer      Enet Peripheral handle
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 * \param cfg       Configuration parameters
 * \param cfgSize   Size of the configuration parameters
 *
 * \return \ref Enet_ErrorCodes
 */
typedef int32_t (* const EnetPer_Open)(EnetPer_Handle hPer,
                                Enet_Type enetType,
                                uint32_t instId,
                                const void *cfg,
                                uint32_t cfgSize);

/*!
 * \brief Rejoin a running Enet Peripheral.
 *
 * Reopens the Enet Peripheral, but doesn't perform any hardware initialization.
 * This function is expected to be called to attach to a running peripheral.
 *
 * This is an optional function and could be set to NULL if the peripheral
 * doesn't implement it.
 *
 * \param hPer      Enet Peripheral handle
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 *
 * \return \ref Enet_ErrorCodes
 */
typedef int32_t (* const EnetPer_Rejoin)(EnetPer_Handle hPer,
                                  Enet_Type enetType,
                                  uint32_t instId);

/*!
 * \brief Issue an operation on the Enet Peripheral.
 *
 * Issues a control operation on the Enet Peripheral.
 *
 * \param hPer         Enet Peripheral handle
 * \param cmd          IOCTL command Id
 * \param prms         IOCTL parameters
 *
 * \return \ref Enet_ErrorCodes
 */
typedef int32_t (* const EnetPer_Ioctl)(EnetPer_Handle hPer,
                                 uint32_t cmd,
                                 Enet_IoctlPrms *prms);

/*!
 * \brief Poll for Ethernet events.
 *
 * Unblocking poll for the events specified in \p evt.
 *
 * This is an optional function and could be set to NULL if the peripheral
 * doesn't implement it.
 *
 * \param hPer         Enet Peripheral handle
 * \param evt          Event type
 * \param arg          Pointer to the poll argument. This is specific to the
 *                     poll event type
 * \param argSize      Size of \p arg
 */
typedef void (* const EnetPer_Poll)(EnetPer_Handle hPer,
                             Enet_Event evt,
                             const void *arg,
                             uint32_t argSize);

/*!
 * \brief Convert a timestamp to nanoseconds.
 *
 * Converts a peripheral-specific timestamp value to nanoseconds value.
 * Timestamp values could be defined as bitfields or absolute values, so the
 * peripheral has to perform the conversion to a nanosecond value, so it's
 * presented in an uniform unit to the application.
 *
 * This is an optional function and could be set to NULL if the peripheral
 * doesn't implement it.
 *
 * \param hPer         Enet Peripheral handle
 * \param ts           Timestamp value, definition is peripheral specific
 *
 * \return Nanoseconds value.
 */
typedef uint64_t (* const EnetPer_ConvertTs)(EnetPer_Handle hPer,
                                      uint64_t ts);

/*!
 * \brief Run periodic tick on the Ethernet peripheral.
 *
 * Run PHY periodic tick on the Ethernet peripheral.
 *
 * \param hPer        Enet Peripheral handle
 */
typedef void (* const EnetPer_PeriodicTick)(EnetPer_Handle hPer);

/*!
 * \brief Register a callback for an event.
 *
 * Registers a callback for an event.  The callback will be called either via
 * Enet_poll() when interrupts are disabled or internally by the driver when
 * servicing an interrupt in ISR context.
 *
 * \param hPer         Enet Peripheral handle
 * \param evt          Event being registered for
 * \param evtNum       Event number. Use 0 for single event.
 * \param evtCb        Callback function
 * \param evtCbArgs    Callback function arguments
 */
typedef void (* const EnetPer_RegisterEventCb)(EnetPer_Handle hPer,
                                        Enet_Event evt,
                                        uint32_t evtNum,
                                        Enet_EventCallback evtCb,
                                        void *evtCbArgs);

/*!
 * \brief Unregister callback for an event.
 *
 * Unregisters a callback for an event.
 *
 * \param hPer         Enet Peripheral handle
 * \param evt          Event being registered for
 * \param evtNum       Event number. Use 0 for single event.
 */
typedef void (* const EnetPer_UnregisterEventCb)(EnetPer_Handle hPer,
                                          Enet_Event evt,
                                          uint32_t evtNum);

/*!
 * \brief Close the Enet Peripheral.
 *
 * Closes the Enet Peripheral.
 *
 * \param hPer        Enet Peripheral handle
 */
typedef void (* const EnetPer_Close)(EnetPer_Handle hPer);

/*!
 * \brief Save and close the Enet Peripheral.
 *
 * Saves and Closes the Enet Peripheral.
 *
 * \param hPer        Enet Peripheral handle
 */
typedef void (* const EnetPer_SaveCtxt)(EnetPer_Handle hPer);

/*!
 * \brief Restores and Opens the Enet Peripheral.
 *
 * Restores and opens the Enet Peripheral with the configuration parameters
 * provided by the caller.
 *
 * \param hPer      Enet Peripheral handle
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 *
 * \return \ref Enet_ErrorCodes
 */
typedef int32_t (* const EnetPer_RestoreCtxt)(EnetPer_Handle hPer,
                                              Enet_Type enetType,
                                              uint32_t instId);

/*!
 * \brief Ethernet Peripheral object.
 */
typedef struct EnetPer_Obj_s
{
    /*! Peripheral name */
    const char *name;

    /*! Peripheral type */
    Enet_Type enetType;

    /*! Peripheral instance id */
    uint32_t instId;

    /*! Peripheral initialization magic */
    Enet_Magic magic;

    /*! Peripheral's physical address. Used for peripherals that have registers
     *  that are not part of any module (i.e. peripherals that have a wrapper
     *  subsystem).
     *  It can be set to 0 for peripherals that  don't have have additional
     *  registers other than those of their modules. */
    uint64_t physAddr;

    /*! Peripheral's virtual address */
    void *virtAddr;

    /*! Peripheral's second physical address, if needed */
    uint64_t physAddr2;

    /*! Peripheral's second virtual address, if needed */
    void *virtAddr2;

    /*! Peripheral features */
    uint32_t features;

    /*! Peripheral applicable errata */
    uint32_t errata;

    /*! Pointer to the EnetPer config initialization function */
    EnetPer_InitCfg initCfg;

    /*! Pointer to the EnetPer open function */
    EnetPer_Open open;

    /*! Pointer to the EnetPer rejoin function */
    EnetPer_Rejoin rejoin;

    /*! Pointer to the EnetPer ioctl function */
    EnetPer_Ioctl ioctl;

    /*! Pointer to the EnetPer poll function */
    EnetPer_Poll poll;

    /*! Pointer to the EnetPer timestamp conversion function */
    EnetPer_ConvertTs convertTs;

    /*! Pointer to the EnetPer periodic tick function */
    EnetPer_PeriodicTick periodicTick;

    /*! Pointer to the EnetPer register event callback function */
    EnetPer_RegisterEventCb registerEventCb;

    /*! Pointer to the EnetPer unregister event callback function */
    EnetPer_UnregisterEventCb unregisterEventCb;

    /*! Pointer to the EnetPer close function */
    EnetPer_Close close;

    /*! Pointer to the EnetPer saveCtxt function */
    EnetPer_SaveCtxt saveCtxt;

    /*! Pointer to the EnetPer restoreCtxt function */
    EnetPer_RestoreCtxt restoreCtxt;
} EnetPer_Obj;

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
