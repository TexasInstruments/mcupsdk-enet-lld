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
 * \file  Icssg.c
 *
 * \brief This file contains the implementation of the ICSSG Ethernet
 *        Peripheral.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <drivers/hw_include/cslr_soc.h>
#include <hw_include/cslr_icss.h>

#include <include/core/enet_base.h>
#include <include/core/enet_trace.h>
#include <include/core/enet_soc.h>
#include <include/core/enet_utils.h>
#include <include/core/enet_osal.h>
#include <include/core/enet_mod_hostport.h>
#include <include/core/enet_mod_stats.h>
#include <priv/core/enet_trace_priv.h>
#include <include/per/icssg.h>
#include <src/per/firmware/icssg/fw_mem_map.h>
#include <drivers/pruicss.h>
#include <priv/per/icssg_priv.h>
#include <priv/per/icssg_ioctl_priv.h>
#include <priv/per/enet_hostport_udma.h>
#include <priv/core/enet_rm_priv.h>
#include <include/common/enet_phymdio_dflt.h>
#include <include/phy/enetphy.h>

#include "icssg_utils.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define ICSSG_NUM_FDB_BUCKET_ENTRIES        (4U)
#define ICSSG_SIZE_OF_FDB                   (2048U)
#define ICSSG_VLAN_TBL_MAX_ENTRIES          (4096U)
#define ICSSG_VLAN_UNTAGGED                 ((int16_t)-1)
#define ICSSG_DRAM1_OFFSET_FROM_DRAM0       (0x2000U)
#define ICSSG_CFG_DEFAULT_AGING_PERIOD_MS   (0x6FC23AC00)
#define ICSSG_CFG_TX_IPG_960_NS             (0x17U)
#define ICSSG_CFG_TX_IPG_104_NS             (0x0BU)

/*! IEP default cycle time of 1 ms. */
#define ICSSG_IEP_DFLT_CYCLE_TIME_NSECS     (1000000U)

/* Implement promiscuous mode using ucast/mcast flooding as filter-based approach
 * is not fully functional for both Switch and Dual-MAC */
#define ICSSG_PROMISC_MODE_WORKAROUND

#define ICSSG_IOCTL_HANDLER_ENTRY_INIT(x)            \
          {.cmd = x,                                \
           .fxn = &Icssg_ioctl_handler_##x}

#define ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(x)    \
          {.cmd = x,                                \
           .fxn = &Icssg_ioctl_handler_default}

#define ICSSG_MACPORT_IOCTL_HANDLER_ENTRY_INIT(x)            \
          {.cmd = x,                                \
           .fxn = &IcssgMacPort_ioctl_handler_##x}

#define ICSSG_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(x)    \
          {.cmd = x,                                \
           .fxn = &IcssgMacPort_ioctl_handler_default}
/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */
/*!
 * \brief Internal Ioctl Handler Function.
 *
 * \param hPer      Enet Peripheral handle
 * \param cmd       IOCTL command Id
 * \param prms      IOCTL parameters
 */
typedef int32_t IcssgInternalIoctlHandlerFxn_t(EnetPer_Handle hPer,
                                       uint32_t cmd,
                                       Enet_IoctlPrms *prms);

/*!
 * \brief Internal IOCTL register handler structure.
 */
typedef struct IcssgInternalIoctlHandlerTableEntry_s
{
    uint32_t cmd;
    IcssgInternalIoctlHandlerFxn_t *fxn;
} IcssgInternalIoctlHandlerTableEntry_t;


/*!
 * \brief Mac Port Ioctl Handler Function.
 *
 * \param hPer      Enet Peripheral handle
 * \param cmd       IOCTL command Id
 * \param prms      IOCTL parameters
 */
typedef int32_t IcssgMacPortIoctlHandlerFxn_t(EnetPer_Handle hPer,
                                       uint32_t cmd,
                                       Enet_IoctlPrms *prms);

/*!
 * \brief Mac Port IOCTL register Handler structure.
 */
typedef struct IcssgMacPortIoctlHandlerTableEntry_s
{
    uint32_t cmd;
    IcssgMacPortIoctlHandlerFxn_t *fxn;
} IcssgMacPortIoctlHandlerTableEntry_t;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static IcssgInternalIoctlHandlerFxn_t * Icssg_getInternalIoctlHandler(EnetPer_Handle hPer, uint32_t cmd, IcssgInternalIoctlHandlerTableEntry_t ioctlTbl[], uint32_t numEntries);

static IcssgMacPortIoctlHandlerFxn_t * Icssg_getMacPortIoctlHandler(EnetPer_Handle hPer, uint32_t cmd, IcssgMacPortIoctlHandlerTableEntry_t ioctlTbl[], uint32_t numEntries);

static void Icssg_setPromiscMode(Icssg_Handle hIcssg,
                                          Enet_MacPort macPort,
                                          bool enable);

static void Icssg_ioctlCfgAgeingPeriod(Icssg_Handle hIcssg,
                                       uint64_t ageingPeriod);

static void Icssg_ioctlVlanFidResetTable(Icssg_Handle hIcssg,
                                         Icssg_VlanFidParams *vlanFidParams);

static int32_t Icssg_ioctlVlanFidSetEntry(Icssg_Handle hIcssg,
                                          Icssg_VlanFidEntry *vlanFidEntry);

static int32_t Icssg_ioctlVlanFidGetEntry(Icssg_Handle hIcssg,
                                          Icssg_VlanFidEntry *vlanFidEntry);

static int32_t Icssg_ioctlSetPortState(Icssg_Handle hIcssg,
                                       IcssgMacPort_SetPortStateInArgs *portStateCfg);

static int32_t Icssg_ioctlFdbAddEntry(Icssg_Handle hIcssg,
                                      Enet_MacPort macPort,
                                      Icssg_FdbEntry *fdbEntry);

static int32_t Icssg_ioctlFdbRemoveEntry(Icssg_Handle hIcssg,
                                         Enet_MacPort macPort,
                                         Icssg_FdbEntry *fdbEntry);

static int32_t Icssg_ioctlFdbRemoveAllEntries(Icssg_Handle hIcssg,
                                              Enet_MacPort macPort);

static int32_t Icssg_ioctlFdbRemoveAllAgeableEntries(Icssg_Handle hIcssg,
                                                     Enet_MacPort macPort);

static void Icssg_ioctlSetMacAddress(Icssg_Handle hIcssg,
                                     IcssgMacPort_SetMacAddressInArgs *macAddressCfg);

static void Icssg_ioctlSetMacAddressHostPort(Icssg_Handle hIcssg,
                                             Icssg_MacAddr *macAddressCfg);

static void Icssg_setSpecialFramePrioCfg(Icssg_Handle hIcssg,
                                         Enet_MacPort macPort,
                                         uint8_t specialFramePrio);

static void Icssg_configCutThroughOrPreempt(Icssg_Handle hIcssg,
                                            Enet_MacPort macPort,
                                            Icssg_QueuePreemptMode *queuePreemptMode,
                                            Icssg_QueueForwardMode *queueForwardMode);

static int32_t Icssg_ioctlPreemptTxEnable(Icssg_Handle hIcssg,
                                          Enet_MacPort macPort);

static int32_t Icssg_ioctlPreemptTxDisable(Icssg_Handle hIcssg,
                                           Enet_MacPort macPort);

static int32_t Icssg_ioctlPreemptGetTxEnableStatus(Icssg_Handle hIcssg,
                                                   Enet_MacPort macPort,
                                                   bool *status);

static int32_t Icssg_ioctlPreemptGetTxActiveStatus(Icssg_Handle hIcssg,
                                                   Enet_MacPort macPort,
                                                   bool *status);

static int32_t Icssg_ioctlPreemptVerifyEnable(Icssg_Handle hIcssg,
                                              Enet_MacPort macPort);

static int32_t Icssg_ioctlPreemptVerifyDisable(Icssg_Handle hIcssg,
                                               Enet_MacPort macPort);

static int32_t Icssg_ioctlPreemptGetVerifyState(Icssg_Handle hIcssg,
                                                Enet_MacPort macPort,
                                                EnetMacPort_PreemptVerifyStatus *status);

static int32_t Icssg_ioctlPreemptGetMinFragSizeLocal(Icssg_Handle hIcssg,
                                                     Enet_MacPort macPort,
                                                     uint8_t *minFragSizeLocal);

static int32_t Icssg_ioctlPreemptSetMinFragSizeRemote(Icssg_Handle hIcssg,
                                                      EnetMacPort_SetPreemptMinFragSizeInArgs *minFragSizeRemoteArgs);

static int32_t Icssg_ioctlPortLinkCfg(Icssg_Handle hIcssg,
                                      const EnetPer_PortLinkCfg *portLinkCfg);

static int32_t Icssg_handleLinkUp(Icssg_Handle hIcssg,
                                  Enet_MacPort macPort);

static int32_t Icssg_handleLinkDown(Icssg_Handle hIcssg,
                                    Enet_MacPort macPort);

static int32_t Icssg_configHostPortDfltVlanId(Icssg_Handle hIcssg,
                                              const EnetPort_VlanCfg *vlanCfg);

static int32_t Icssg_configMacPortDfltVlanId(Icssg_Handle hIcssg,
                                             Enet_MacPort macPort,
                                             const EnetPort_VlanCfg *vlanCfg);

static int32_t Icssg_validateFlowId(Icssg_Handle hIcssg,
                                    Enet_MacPort macPort,
                                    uint32_t coreKey,
                                    uint32_t startIdx,
                                    uint32_t flowIdx);

static int32_t Icssg_ioctlInternal(EnetPer_Handle hPer,
                                   uint32_t cmd,
                                   Enet_IoctlPrms *prms);

static int32_t Icssg_ioctlMacPort(Icssg_Handle hIcssg,
                                  uint32_t cmd,
                                  Enet_IoctlPrms *prms);

static int32_t Icssg_openEnetPhy(Icssg_Handle hIcssg,
                                 Enet_MacPort macPort,
                                 const EnetPhy_Cfg *phyCfg,
                                 EnetPhy_Mii phyMii,
                                 const EnetPhy_LinkCfg *phyLinkCfg);

static void Icssg_closeEnetPhy(Icssg_Handle hIcssg,
                               Enet_MacPort macPort);


static void Icssg_UCastFloodingCtrl(Icssg_Handle hIcssg,
                                    Enet_MacPort macPort,
                                    bool enable);

static void Icssg_MCastFloodingCtrl(Icssg_Handle hIcssg,
                                    Enet_MacPort macPort,
                                    bool enable);

static bool Icssg_isVlanAware(Icssg_Handle hIcssg);

static int32_t Icssg_setVlanAwareMode(Icssg_Handle hIcssg);

static int32_t Icssg_setVlanUnawareMode(Icssg_Handle hIcssg);

static int32_t Icssg_setDscpEnable(Icssg_Handle hIcssg,
                                   Enet_MacPort macPort);

static int32_t Icssg_setDscpDisable(Icssg_Handle hIcssg,
                                   Enet_MacPort macPort);

static int32_t Icssg_setAcceptableFrameCheck(Icssg_Handle hIcssg,
                                             Enet_MacPort macPort,
                                             Icssg_AcceptFrameCheck acceptFrameCheck);

static int32_t Icssg_setAcceptableFrameCheckSync(Icssg_Handle hIcssg,
                                                 Enet_MacPort macPort,
                                                 Icssg_AcceptFrameCheck acceptFrameCheck);

static int32_t Icssg_setPriorityMapping(Icssg_Handle hIcssg,
                                        Enet_MacPort macPort,
                                        EnetPort_PriorityMap *priMap);

static int32_t Icssg_setPriorityRegen(Icssg_Handle hIcssg,
                                      Enet_MacPort macPort,
                                      EnetPort_PriorityMap *priMap);

static int32_t Icssg_setDscpPriority(Icssg_Handle hIcssg,
                                      Enet_MacPort macPort,
                                      EnetPort_DscpPriorityMap *priMap);

static int32_t Icssg_setIngressRateLim(Icssg_Handle hIcssg,
                                       Enet_MacPort macPort,
                                       Icssg_IngressRateLim *rateLimCfg);

static uintptr_t Icssg_getTxIpgCfgAddr(Icssg_Handle hIcssg,
                                       Enet_MacPort macPort,
                                       bool crossSlice);

static void Icssg_updateRgmiiCfg10HD(Icssg_Handle hIcssg,
                                     Enet_MacPort macPort);

static void Icssg_updateRgmiiCfg10FD(Icssg_Handle hIcssg,
                                     Enet_MacPort macPort);

static void Icssg_updateRgmiiCfg100HD(Icssg_Handle hIcssg,
                                      Enet_MacPort macPort);

static void Icssg_updateRgmiiCfg100FD(Icssg_Handle hIcssg,
                                      Enet_MacPort macPort);

static void Icssg_updateRgmiiCfg1G(Icssg_Handle hIcssg,
                                   Enet_MacPort macPort);

static void Icssg_updateLinkSpeed10MB(Icssg_Handle hIcssg,
                                      Enet_MacPort macPort,
                                      EnetPhy_Duplexity duplex);

static void Icssg_updateLinkSpeed100MB(Icssg_Handle hIcssg,
                                       Enet_MacPort macPort,
                                       EnetPhy_Duplexity duplex);

static void Icssg_updateLinkSpeed1G(Icssg_Handle hIcssg,
                                    Enet_MacPort macPort);

static void Icssg_updateLinkDown(Icssg_Handle hIcssg,
                                 Enet_MacPort macPort);

static int32_t Icssg_cfgMdioLinkInt(Icssg_Handle hIcssg,
                                 Enet_Type enetType,
                                 uint32_t instId,
                                 const Icssg_Cfg *cfg);

static int32_t Icssg_registerMdioLinkIntr(Icssg_Handle hIcssg,
                                 const Icssg_mdioLinkIntCfg *mdioLinkIntCfg);

static void Icssg_unregisterMdioLinkIntr(Icssg_Handle hIcssg);

static int32_t Icssg_enablePruIcssInt(Icssg_Handle hIcssg);

static int32_t Icssg_handleExternalPhyLinkUp(Icssg_Handle hIcssg,
                                             Enet_MacPort macPort,
                                             const EnetPhy_LinkCfg *phyLinkCfg);

static int32_t Icssg_ioctl_handler_ENET_PER_IOCTL_REGISTER_IOCTL_HANDLER(EnetPer_Handle hPer,
                                                                    uint32_t cmd,
                                                                    Enet_IoctlPrms *prms);

static int32_t Icssg_ioctl_handler_default(EnetPer_Handle hPer,
                                    uint32_t cmd,
                                    Enet_IoctlPrms *prms);

static int32_t IcssgMacPort_ioctl_handler_default(EnetPer_Handle hPer,
                                            uint32_t cmd,
                                            Enet_IoctlPrms *prms);

static int32_t IcssgMacPort_ioctl_handler_ICSSG_MACPORT_IOCTL_REGISTER_HANDLER(EnetPer_Handle hPer,
                                                                                uint32_t cmd,
                                                                                Enet_IoctlPrms *prms);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

#if ENET_CFG_IS_ON(DEV_ERROR)
/* Public ICSSG peripheral IOCTL validation data. */
static Enet_IoctlValidate gIcssg_ioctlValidate[] =
{
    ENET_IOCTL_VALID_PRMS(ICSSG_MACPORT_IOCTL_ENABLE_PROMISC_MODE,
                          sizeof(Enet_MacPort),
                          0U),

    ENET_IOCTL_VALID_PRMS(ICSSG_MACPORT_IOCTL_DISABLE_PROMISC_MODE,
                          sizeof(Enet_MacPort),
                          0U),

    ENET_IOCTL_VALID_PRMS(ICSSG_PER_IOCTL_VLAN_RESET_TABLE,
                          sizeof(Icssg_VlanFidParams),
                          0U),

    ENET_IOCTL_VALID_PRMS(ICSSG_PER_IOCTL_VLAN_SET_ENTRY,
                          sizeof(Icssg_VlanFidEntry),
                          0U),

    ENET_IOCTL_VALID_PRMS(ICSSG_PER_IOCTL_VLAN_GET_ENTRY,
                          0U,
                          sizeof(Icssg_VlanFidEntry)),

    ENET_IOCTL_VALID_PRMS(ICSSG_PER_IOCTL_SET_PORT_STATE,
                          sizeof(Icssg_PortState),
                          0U),

    ENET_IOCTL_VALID_PRMS(ICSSG_FDB_IOCTL_ADD_ENTRY,
                          sizeof(Icssg_FdbEntry),
                          0U),

    ENET_IOCTL_VALID_PRMS(ICSSG_FDB_IOCTL_REMOVE_ENTRY,
                          sizeof(Icssg_FdbEntry),
                          0U),

    ENET_IOCTL_VALID_PRMS(ICSSG_FDB_IOCTL_REMOVE_ALL_ENTRIES,
                          0U,
                          0U),

    ENET_IOCTL_VALID_PRMS(ICSSG_FDB_IOCTL_REMOVE_AGEABLE_ENTRIES,
                          0U,
                          0U),

    ENET_IOCTL_VALID_PRMS(ICSSG_MACPORT_IOCTL_SET_MACADDR,
                          sizeof(IcssgMacPort_SetMacAddressInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_MACPORT_IOCTL_ENABLE_PREEMPTION,
                          sizeof(Enet_MacPort),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_MACPORT_IOCTL_DISABLE_PREEMPTION,
                          sizeof(Enet_MacPort),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_MACPORT_IOCTL_GET_PREEMPTION_ENABLE_STATUS,
                          sizeof(Enet_MacPort),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_MACPORT_IOCTL_GET_PREEMPTION_ACTIVE_STATUS,
                          sizeof(Enet_MacPort),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_MACPORT_IOCTL_ENABLE_PREEMPT_VERIFICATION,
                          sizeof(Enet_MacPort),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_MACPORT_IOCTL_DISABLE_PREEMPT_VERIFICATION,
                          sizeof(Enet_MacPort),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_MACPORT_IOCTL_GET_PREEMPT_VERIFY_STATUS,
                          sizeof(Enet_MacPort),
                          sizeof(EnetMacPort_PreemptVerifyStatus)),

    ENET_IOCTL_VALID_PRMS(ENET_MACPORT_IOCTL_GET_PREEMPT_MIN_FRAG_SIZE,
                          sizeof(Enet_MacPort),
                          sizeof(uint8_t)),

    ENET_IOCTL_VALID_PRMS(ENET_MACPORT_IOCTL_SET_PREEMPT_MIN_FRAG_SIZE,
                          sizeof(EnetMacPort_SetPreemptMinFragSizeInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_MACPORT_IOCTL_SET_PREEMPT_QUEUE,
                          sizeof(EnetMacPort_SetPreemptQueueInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_MACPORT_IOCTL_GET_PREEMPT_MIN_FRAG_SIZE,
                          sizeof(EnetMacPort_GenericInArgs),
                          sizeof(uint8_t)),

    ENET_IOCTL_VALID_PRMS(ICSSG_HOSTPORT_IOCTL_SET_MACADDR,
                          sizeof(Icssg_MacAddr),
                          0U),

    ENET_IOCTL_VALID_PRMS(ICSSG_PER_IOCTL_VLAN_SET_HOSTPORT_DFLT_VID,
                          sizeof(EnetPort_VlanCfg),
                          0U),

    ENET_IOCTL_VALID_PRMS(ICSSG_PER_IOCTL_VLAN_SET_MACPORT_DFLT_VID,
                          sizeof(Icssg_MacPortDfltVlanCfgInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ICSSG_FDB_IOCTL_SET_AGING_PERIOD,
                          sizeof(uint64_t),
                          0U),

    ENET_IOCTL_VALID_PRMS(ICSSG_MACPORT_IOCTL_ENABLE_UCAST_FLOOD,
                          sizeof(Enet_MacPort),
                          0U),

    ENET_IOCTL_VALID_PRMS(ICSSG_MACPORT_IOCTL_DISABLE_UCAST_FLOOD,
                          sizeof(Enet_MacPort),
                          0U),

    ENET_IOCTL_VALID_PRMS(ICSSG_MACPORT_IOCTL_ENABLE_MCAST_FLOOD,
                          sizeof(Enet_MacPort),
                          0U),

    ENET_IOCTL_VALID_PRMS(ICSSG_MACPORT_IOCTL_DISABLE_MCAST_FLOOD,
                          sizeof(Enet_MacPort),
                          0U),

    ENET_IOCTL_VALID_PRMS(ICSSG_MACPORT_IOCTL_SET_ACCEPT_FRAME_CHECK,
                          sizeof(Icssg_SetAcceptFrameCheckInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ICSSG_MACPORT_IOCTL_SET_INGRESS_RATE_LIM,
                          sizeof(Icssg_IngressRateLim),
                          0U),

    ENET_IOCTL_VALID_PRMS(ICSSG_MACPORT_IOCTL_SET_QUEUE_CUT_THROUGH_PREEMPT_SELECT,
                          sizeof(IcssgMacPort_SetQueueCtPremptModeInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ICSSG_MACPORT_IOCTL_CONFIG_SPL_FRAME_PRIO,
                          sizeof(IcssgMacPort_ConfigSpecialFramePrioInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_PER_IOCTL_REGISTER_IOCTL_HANDLER,
                          sizeof(Enet_IoctlRegisterHandlerInArgs),
                          0U),
};
#endif

static IcssgInternalIoctlHandlerTableEntry_t IcssgInternalIoctlHandlerTable[] =
{
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_PER_IOCTL_OPEN_PORT_LINK),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_PER_IOCTL_CLOSE_PORT_LINK),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_PER_IOCTL_IS_PORT_LINK_UP),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_PER_IOCTL_GET_PORT_LINK_CFG),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ICSSG_MACPORT_IOCTL_ENABLE_PROMISC_MODE),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ICSSG_MACPORT_IOCTL_DISABLE_PROMISC_MODE),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ICSSG_MACPORT_IOCTL_ENABLE_UCAST_FLOOD),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ICSSG_MACPORT_IOCTL_DISABLE_UCAST_FLOOD),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ICSSG_MACPORT_IOCTL_ENABLE_MCAST_FLOOD),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ICSSG_MACPORT_IOCTL_DISABLE_MCAST_FLOOD),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ICSSG_PER_IOCTL_VLAN_RESET_TABLE),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ICSSG_PER_IOCTL_VLAN_SET_ENTRY),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ICSSG_PER_IOCTL_VLAN_GET_ENTRY),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ICSSG_PER_IOCTL_VLAN_SET_HOSTPORT_DFLT_VID),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ICSSG_PER_IOCTL_VLAN_SET_MACPORT_DFLT_VID),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ICSSG_PER_IOCTL_SET_PORT_STATE),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ICSSG_FDB_IOCTL_ADD_ENTRY),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ICSSG_FDB_IOCTL_REMOVE_ENTRY),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ICSSG_FDB_IOCTL_REMOVE_ALL_ENTRIES),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ICSSG_FDB_IOCTL_REMOVE_AGEABLE_ENTRIES),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ICSSG_MACPORT_IOCTL_SET_MACADDR),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ICSSG_HOSTPORT_IOCTL_SET_MACADDR),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_IOCTL_REGISTER_RX_DEFAULT_FLOW),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_IOCTL_UNREGISTER_RX_DEFAULT_FLOW),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_PER_IOCTL_ATTACH_CORE),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_PER_IOCTL_DETACH_CORE),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ICSSG_PER_IOCTL_TAS_TRIGGER),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ICSSG_PER_IOCTL_TAS_ENABLE),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ICSSG_PER_IOCTL_TAS_DISABLE),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ICSSG_PER_IOCTL_TAS_RESET),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ICSSG_FDB_IOCTL_SET_AGING_PERIOD),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ICSSG_MACPORT_IOCTL_SET_ACCEPT_FRAME_CHECK),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ICSSG_MACPORT_IOCTL_SET_INGRESS_RATE_LIM),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_PER_IOCTL_SET_VLAN_AWARE),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_PER_IOCTL_SET_VLAN_UNAWARE),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ICSSG_MACPORT_IOCTL_SET_QUEUE_CUT_THROUGH_PREEMPT_SELECT),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ICSSG_MACPORT_IOCTL_CONFIG_SPL_FRAME_PRIO),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_PER_IOCTL_HANDLE_EXTPHY_LINKUP_EVENT),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_PER_IOCTL_HANDLE_EXTPHY_LINKDOWN_EVENT),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT(ENET_PER_IOCTL_REGISTER_IOCTL_HANDLER),
};

static IcssgMacPortIoctlHandlerTableEntry_t IcssgMacPortIoctlHandlerTable[] =
{
    ICSSG_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MACPORT_IOCTL_SET_EGRESS_QOS_PRI_MAP),
    ICSSG_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MACPORT_IOCTL_SET_PRI_REGEN_MAP),
    ICSSG_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MACPORT_IOCTL_SET_INGRESS_DSCP_PRI_MAP),
    ICSSG_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MACPORT_IOCTL_GET_EGRESS_QOS_PRI_MAP), /*IOCTL Not supported*/
    ICSSG_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MACPORT_IOCTL_GET_PRI_REGEN_MAP), /*IOCTL Not supported*/
    ICSSG_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MACPORT_IOCTL_GET_INGRESS_DSCP_PRI_MAP), /*IOCTL Not supported*/
    ICSSG_MACPORT_IOCTL_HANDLER_ENTRY_INIT(ICSSG_MACPORT_IOCTL_REGISTER_HANDLER),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MACPORT_IOCTL_ENABLE_PREEMPTION),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MACPORT_IOCTL_DISABLE_PREEMPTION),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MACPORT_IOCTL_GET_PREEMPTION_ENABLE_STATUS),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MACPORT_IOCTL_GET_PREEMPTION_ACTIVE_STATUS),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MACPORT_IOCTL_ENABLE_PREEMPT_VERIFICATION),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MACPORT_IOCTL_DISABLE_PREEMPT_VERIFICATION),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MACPORT_IOCTL_GET_PREEMPT_VERIFY_STATUS),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MACPORT_IOCTL_GET_PREEMPT_MIN_FRAG_SIZE),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MACPORT_IOCTL_SET_PREEMPT_MIN_FRAG_SIZE),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MACPORT_IOCTL_SET_PREEMPT_QUEUE),
    ICSSG_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MACPORT_IOCTL_GET_QUEUE_PREEMPT_STATUS)
};

#if ((ENET_CFG_TRACE_LEVEL >= ENET_CFG_TRACE_LEVEL_ERROR) && ENET_CFG_IS_OFF(TRACE_DISABLE_INFOSTRING))
static const char *Icssg_gSpeedNames[] =
{
    [ENET_SPEED_10MBIT]  = "10-Mbps",
    [ENET_SPEED_100MBIT] = "100-Mbps",
    [ENET_SPEED_1GBIT]   = "1-Gbps",
    [ENET_SPEED_AUTO]    = "auto",
};

static const char *Icssg_gDuplexNames[] =
{
    [ENET_DUPLEX_HALF] = "Half-Duplex",
    [ENET_DUPLEX_FULL] = "Full-Duplex",
    [ENET_DUPLEX_AUTO] = "auto",
};
#endif

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void Icssg_initCfg(EnetPer_Handle hPer,
                   Enet_Type enetType,
                   void *cfg,
                   uint32_t cfgSize)
{
    Icssg_Cfg *icssgInitCfg = (Icssg_Cfg *)cfg;

    ENETTRACE_DBG("Initialize ICSSG peripheral config\r\n");

    Enet_devAssert(cfgSize == sizeof(Icssg_Cfg),
                   "Invalid ICSSG peripheral config params size %u (expected %u)\r\n",
                   cfgSize, sizeof(Icssg_Cfg));

    memset(icssgInitCfg, 0, sizeof(Icssg_Cfg));

    icssgInitCfg->agingPeriod = (uint64_t)ICSSG_CFG_DEFAULT_AGING_PERIOD_MS;

    icssgInitCfg->vlanCfg.portPri  = 0U;
    icssgInitCfg->vlanCfg.portCfi  = 0U;
    icssgInitCfg->vlanCfg.portVID  = 0U;

    /* Initialize MDIO config params */
    Mdio_initCfg(&icssgInitCfg->mdioCfg);

    icssgInitCfg->cycleTimeNs = ICSSG_IEP_DFLT_CYCLE_TIME_NSECS;

    icssgInitCfg->clockTypeFw = ICSSG_TIMESYNC_CLKTYPE_WORKING_CLOCK;

    /* Initialize TimeSync config params */
    IcssgTimeSync_initCfg(&icssgInitCfg->timeSyncCfg);

    /* MII mode is common for both ports. Initialize to RGMII */
    icssgInitCfg->mii.layerType    = ENET_MAC_LAYER_GMII;
    icssgInitCfg->mii.sublayerType = ENET_MAC_SUBLAYER_REDUCED;
    icssgInitCfg->mii.variantType  = ENET_MAC_VARIANT_NONE;
    icssgInitCfg->mdioLinkIntCfg.mdioLinkStateChangeCb     = NULL;
    icssgInitCfg->mdioLinkIntCfg.mdioLinkStateChangeCbArg  = NULL;
    icssgInitCfg->disablePhyDriver = false;
    icssgInitCfg->isPremQueEnable  = true;
    icssgInitCfg->qosLevels = ICSSG_QOS_MAX;
}

void IcssgMacPort_initCfg(IcssgMacPort_Cfg *macPortCfg)
{
    uint32_t i;

    macPortCfg->promiscEn        = false;
    macPortCfg->ucastFloodEn     = false;
    macPortCfg->mcastFloodEn     = false;
    macPortCfg->acceptFrameCheck = ICSSG_ACCEPT_ALL;
    macPortCfg->vlanCfg.portPri  = 0U;
    macPortCfg->vlanCfg.portCfi  = 0U;
    macPortCfg->vlanCfg.portVID  = 0U;
    macPortCfg->specialFramePrio = 0U;

    for (i = 0U; i < ENET_PRI_NUM; i++)
    {
        macPortCfg->queuePreemptMode[i] = ICSSG_QUEUE_PREEMPT_MODE_EXPRESS;
        macPortCfg->queueForwardMode[i] = ICSSG_QUEUE_FORWARD_MODE_STOREANDFWD;
    }
}

static inline uint32_t Icssg_getChIdx(Icssg_Handle hIcssg,
                                      Enet_MacPort macPort)
{
    EnetPer_Handle hPer = (EnetPer_Handle)hIcssg;
    uint32_t chIdx;

    if (hPer->enetType == ENET_ICSSG_SWITCH)
    {
        chIdx = (macPort == ENET_MAC_PORT_1) ? 0U : 1U;
    }
    else
    {
        chIdx = 0U;
    }

    return chIdx;
}

static int32_t Icssg_openDma(Icssg_Handle hIcssg,
                             const Icssg_Cfg *icssgCfg,
                             Enet_Type enetType,
                             uint32_t instId)
{
    EnetPer_Handle hPer = (EnetPer_Handle)hIcssg;
    int32_t status = ENET_SOK;

    /* Open DMA */
    if (NULL != icssgCfg->dmaCfg)
    {
        /* Open UDMA for ICSSG NAVSS instance type */
        hIcssg->hDma = EnetHostPortDma_open(hPer, icssgCfg->dmaCfg, &icssgCfg->resCfg);
        if (NULL == hIcssg->hDma)
        {
            ENETTRACE_ERR("%s: failed to open ICSSG Host Port RX\r\n", ENET_PER_NAME(hIcssg));
            status = ENET_EFAIL;
        }
    }
    else
    {
        ENETTRACE_ERR("%s: DMA open config is NULL\r\n", ENET_PER_NAME(hIcssg));
        status = ENET_EINVALIDPARAMS;
    }

    return status;
}

static int32_t Icssg_enablePruss(Icssg_Handle hIcssg)
{
    EnetPer_Handle hPer = (EnetPer_Handle)hIcssg;
    int32_t status;

    status = IcssgUtils_enablePruss(hIcssg, ENET_MAC_PORT_1);
    ENETTRACE_ERR_IF((status != ENET_SOK),
                     "%s: Port %u: Failed to enable PRUSS: %d\r\n",
                     ENET_PER_NAME(hIcssg), ENET_MACPORT_ID(ENET_MAC_PORT_1), status);

    if ((status == ENET_SOK) &&
        (hPer->enetType == ENET_ICSSG_SWITCH))
    {
        status = IcssgUtils_enablePruss(hIcssg, ENET_MAC_PORT_2);
        ENETTRACE_ERR_IF((status != ENET_SOK),
                         "%s: Port %u: Failed to enable PRUSS: %d\r\n",
                         ENET_PER_NAME(hIcssg), ENET_MACPORT_ID(ENET_MAC_PORT_2), status);
    }

    return status;
}

static int32_t Icssg_disablePruss(Icssg_Handle hIcssg)
{
    EnetPer_Handle hPer = (EnetPer_Handle)hIcssg;
    int32_t status;

    status = IcssgUtils_disablePruss(hIcssg, ENET_MAC_PORT_1);
    ENETTRACE_ERR_IF((status != ENET_SOK),
                     "%s: Port %u: Failed to disable PRUSS: %d\r\n",
                     ENET_PER_NAME(hIcssg), ENET_MACPORT_ID(ENET_MAC_PORT_1), status);

    if ((status == ENET_SOK) &&
        (hPer->enetType == ENET_ICSSG_SWITCH))
    {
        status = IcssgUtils_disablePruss(hIcssg, ENET_MAC_PORT_2);
        ENETTRACE_ERR_IF((status != ENET_SOK),
                         "%s: Port %u: Failed to enable PRUSS: %d\r\n",
                         ENET_PER_NAME(hIcssg), ENET_MACPORT_ID(ENET_MAC_PORT_2), status);
    }

    return status;
}

static uint32_t Icssg_getFwIdx(uint32_t instId)
{
    uint32_t fwIdx = 0;

    switch(instId)
    {
        case 0:
            fwIdx = 0;
            break;

        case 1:
            fwIdx = 1;
            break;

        case 2:
            fwIdx = 0;
            break;

        case 3:
            fwIdx = 1;
            break;
    }
    return fwIdx;
}

static Enet_MacPort Icssg_getMacFromInstId(uint32_t instId)
{
    Enet_MacPort macPort = ENET_MAC_PORT_FIRST;

        switch(instId) { case 0: macPort = ENET_MAC_PORT_1; break; case 2: macPort = ENET_MAC_PORT_1; break;

        case 1: macPort = ENET_MAC_PORT_2; break;

        case 3: macPort = ENET_MAC_PORT_2; break; }

        return macPort; }

static int32_t Icssg_configAndDownloadFw(Icssg_Handle hIcssg,
                                         const Icssg_Cfg *cfg)
{
    EnetPer_Handle hPer = (EnetPer_Handle)hIcssg;
    int32_t status = ENET_SOK;
    const Icssg_FwPoolMem *fwPoolMem;
    fwPoolMem = EnetCb_GetFwPoolMem(hPer->enetType, hPer->instId);

    /* Config and download firmware for MAC port 1. This is applicable for Dual-MAC */
    if (hPer->enetType == ENET_ICSSG_DUALMAC)
    {
        Enet_MacPort macPort;

        macPort = Icssg_getMacFromInstId(hPer->instId);

        status = IcssgUtils_checkFwPoolMem(hIcssg, &fwPoolMem[0U]);
        ENETTRACE_ERR_IF((status != ENET_SOK),
                         "%s: invalid firmware memory\r\n",
                         ENET_PER_NAME(hIcssg), status);
        if (status == ENET_SOK)
        {
            status = IcssgUtils_checkPortMode(hIcssg, &cfg->mii);
            ENETTRACE_ERR_IF((status != ENET_SOK),
                             "%s: MII mode mismatch: %d\r\n",
                             ENET_PER_NAME(hIcssg), status);
        }

        if (status == ENET_SOK)
        {
            IcssgUtils_fwConfig(hIcssg, macPort,
                                cfg,
                                &fwPoolMem[0U],
                                hIcssg->dmaResInfo[0U].rxStartIdx);

            if ((cfg->fw[0U].pru != NULL) && (cfg->fw[0U].rtu != NULL) && (cfg->fw[0U].txpru != NULL))
            {
                status = IcssgUtils_downloadFirmware(hIcssg, macPort, (Icssg_Fw *)&cfg->fw[0U]);
            }
            else
            {
                status = IcssgUtils_downloadFirmware(hIcssg, macPort, &hIcssg->fw[0U]);
            }
            ENETTRACE_ERR_IF((status != ENET_SOK),
                             "%s: firmware download failure: %d\r\n",
                             ENET_PER_NAME(hIcssg), status);
        }
    }
    else
    {
        status = IcssgUtils_checkFwPoolMem(hIcssg, &fwPoolMem[0U]);
        ENETTRACE_ERR_IF((status != ENET_SOK),
                         "%s: Port 1: invalid slice 0 firmware memory\r\n",
                         ENET_PER_NAME(hIcssg), status);

        status = IcssgUtils_checkFwPoolMem(hIcssg, &fwPoolMem[1U]);
        ENETTRACE_ERR_IF((status != ENET_SOK),
                         "%s: Port 2: invalid slice 1 firmware memory\r\n",
                         ENET_PER_NAME(hIcssg), status);

        if (status == ENET_SOK)
        {
            IcssgUtils_configSwtFw(hIcssg,
                                   cfg,
                                   &fwPoolMem[0U],
                                   &fwPoolMem[1U],
                                   hIcssg->dmaResInfo[0U].rxStartIdx,
                                   hIcssg->dmaResInfo[1U].rxStartIdx);
        }

        if (status == ENET_SOK)
        {
            if ((cfg->fw[0U].pru != NULL) && (cfg->fw[0U].rtu != NULL) && (cfg->fw[0U].txpru != NULL))
            {
                status = IcssgUtils_downloadFirmware(hIcssg, ENET_MAC_PORT_1, (Icssg_Fw *)&cfg->fw[0U]);
            }
            else
            {
                status = IcssgUtils_downloadFirmware(hIcssg, ENET_MAC_PORT_1, &hIcssg->fw[0U]);
            }
            ENETTRACE_ERR_IF((status != ENET_SOK),
                             "%s: Port 1: firmware download failure: %d\r\n",
                             ENET_PER_NAME(hIcssg), status);
        }

        if (status == ENET_SOK)
        {
            if ((cfg->fw[1U].pru != NULL) && (cfg->fw[1U].rtu != NULL) && (cfg->fw[1U].txpru != NULL))
            {
                status = IcssgUtils_downloadFirmware(hIcssg, ENET_MAC_PORT_2, (Icssg_Fw *)&cfg->fw[1U]);
            }
            else
            {
                status = IcssgUtils_downloadFirmware(hIcssg, ENET_MAC_PORT_2, &hIcssg->fw[1U]);
            }
            ENETTRACE_ERR_IF((status != ENET_SOK),
                             "%s: Port 2: firmware download failure: %d\r\n",
                             ENET_PER_NAME(hIcssg), status);
        }
    }

    return status;
}

static void Icssg_disableClassifiers(Icssg_Handle hIcssg)
{
    EnetPer_Handle hPer = (EnetPer_Handle)hIcssg;

    IcssgUtils_classiDisable(hIcssg, ENET_MAC_PORT_1);

    if (hPer->enetType == ENET_ICSSG_SWITCH)
    {
        IcssgUtils_classiDisable(hIcssg, ENET_MAC_PORT_2);
    }
}

static void Icssg_initR30Cmd(Icssg_Handle hIcssg)
{
    EnetPer_Handle hPer = (EnetPer_Handle)hIcssg;

    IcssgUtils_R30CmdInit(hIcssg, ENET_MAC_PORT_1);

    if (hPer->enetType == ENET_ICSSG_SWITCH)
    {
        IcssgUtils_R30CmdInit(hIcssg, ENET_MAC_PORT_2);
    }
}

static int32_t Icssg_initPriorityRegen(Icssg_Handle hIcssg)
{
    EnetPer_Handle hPer = (EnetPer_Handle)hIcssg;
    EnetPort_PriorityMap priMap = {
        .priorityMap = { 0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U }
    };
    int32_t status;

    status = Icssg_setPriorityRegen(hIcssg, ENET_MAC_PORT_1, &priMap);
    ENETTRACE_ERR_IF((status != ENET_SOK),
                     "%s: Port 1: failed to initialize priority regeneration: %d\r\n",
                     ENET_PER_NAME(hIcssg), status);

    if ((status == ENET_SOK) &&
        (hPer->enetType == ENET_ICSSG_SWITCH))
    {
        status = Icssg_setPriorityRegen(hIcssg, ENET_MAC_PORT_2, &priMap);
        ENETTRACE_ERR_IF((status != ENET_SOK),
                         "%s: Port 2: failed to initialize priority regeneration: %d\r\n",
                         ENET_PER_NAME(hIcssg), status);
    }

    return status;
}

static void Icssg_configFt3PriorityTag(Icssg_Handle hIcssg)
{
    EnetPer_Handle hPer = (EnetPer_Handle)hIcssg;
    /* Program FT3[8] to detect priority tagged frames */
    Icssg_Filter3Cfg ft3CfgPrioTagFrames = {
        .ft3Start           = 0xCU,
        .ft3StartAuto       = 0x0U,
        .ft3StartOffset     = 0x0U,
        .ft3JmpOffset       = 0x0U,
        .ft3Len             = 0x0U,
        .ft3Config          = 0x5U,
        .ft3Type            = 0x00000081U,
        .ft3TypeMask        = 0x00F00000U,
        .ft3PatternLow      = 0x0U,
        .ft3PatternHigh     = 0x0U,
        .ft3PatternMaskLow  = 0xFFFFFFFFU,
        .ft3PatternMaskHigh = 0xFFFFFFFFU,
    };

    IcssgUtils_configFilter3(hIcssg, ENET_MAC_PORT_1, 8, &ft3CfgPrioTagFrames);

    if (hPer->enetType == ENET_ICSSG_SWITCH)
    {
        IcssgUtils_configFilter3(hIcssg, ENET_MAC_PORT_2, 8, &ft3CfgPrioTagFrames);
    }
}

int32_t Icssg_open(EnetPer_Handle hPer,
                   Enet_Type enetType,
                   uint32_t instId,
                   const void *cfg,
                   uint32_t cfgSize)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    const Icssg_Cfg *icssgCfg = (const Icssg_Cfg *)cfg;
    uint32_t i;
    int32_t status = ENET_SOK;

    ENETTRACE_DBG("%s: open peripheral\r\n", ENET_PER_NAME(hIcssg));

    Enet_devAssert(cfgSize == sizeof(Icssg_Cfg),
                   "%s: Invalid ICSSG peripheral config params size %u (expected %u)\r\n",
                   ENET_PER_NAME(hIcssg), cfgSize, sizeof(Icssg_Cfg));

    /* Save EnetMod and EnetPhy handles for easy access */
    hIcssg->hTimeSync = ENET_MOD(&hIcssg->timeSyncObj);
    hIcssg->hStats = ENET_MOD(&hIcssg->statsObj);
    hIcssg->hMdio  = ENET_MOD(&hIcssg->mdioObj);
    hIcssg->hRm    = ENET_MOD(&hIcssg->rmObj);
    hIcssg->hTas[0] = ENET_MOD(&hIcssg->tasObj[0]);
    if (enetType == ENET_ICSSG_SWITCH)
    {
        hIcssg->hTas[1] = ENET_MOD(&hIcssg->tasObj[1]);
    }

    /* save hIcssg handle in hTas */
    hIcssg->tasObj[0].hIcssg = (void*)hIcssg;
    hIcssg->tasObj[1].hIcssg = (void*)hIcssg;

    /* save hIcssg handle in hTimeSync */
    hIcssg->timeSyncObj.hIcssg = (void*)hIcssg;

    hIcssg->enetPer.enetType  = enetType;
    hIcssg->enetPer.instId    = instId;

    /* Copy config info from config structure to instance structure */
    hIcssg->disablePhyDriver = icssgCfg->disablePhyDriver;
    hIcssg->qosLevels        = icssgCfg->qosLevels;
    hIcssg->isPremQueEnable  = icssgCfg->isPremQueEnable;

    /* Create handle to the corresponding PRU instance to use with PRUICSS driver */
    if (status == ENET_SOK)
    {
        status = IcssgUtils_createPruss(hIcssg);
        ENETTRACE_ERR_IF((status != ENET_SOK),
                         "%s: failed to create PRUSS driver: %d\r\n",
                         ENET_PER_NAME(hIcssg), status);
    }

    /* Config ageing period for FDB entries */
    if (status == ENET_SOK)
    {
        Icssg_ioctlCfgAgeingPeriod(hIcssg, icssgCfg->agingPeriod);
    }

    /* Open DMA */
    if (status == ENET_SOK)
    {
        hIcssg->numRxCh = (enetType == ENET_ICSSG_SWITCH) ? 2U : 1U;

        status = Icssg_openDma(hIcssg, icssgCfg, enetType, instId);
        ENETTRACE_ERR_IF((status != ENET_SOK),
                         "%s: failed to open DMA: %d\r\n",
                         ENET_PER_NAME(hIcssg), status);
    }

    /* Initialize R30 cmd mechanism, disable classifier */
    if (status == ENET_SOK)
    {
        Icssg_disableClassifiers(hIcssg);
        Icssg_initR30Cmd(hIcssg);
        Icssg_configFt3PriorityTag(hIcssg);
    }

    /* Open Resource Manager if Rx Channel open succeeded */
    if (status == ENET_SOK)
    {
        EnetRm_Cfg rmCfg;
        Enet_dmaResInfo *dmaResInfo;

        rmCfg.enetType              = enetType;
        rmCfg.instId                = instId;
        rmCfg.ioctlPermissionInfo   = icssgCfg->resCfg.ioctlPermissionInfo;
        rmCfg.macList               = icssgCfg->resCfg.macList;
        rmCfg.resPartInfo           = icssgCfg->resCfg.resPartInfo;
        rmCfg.numRxCh               = hIcssg->numRxCh;

        for (i = 0U; i < hIcssg->numRxCh; i++)
        {
            dmaResInfo = &hIcssg->dmaResInfo[i];

            EnetHostPortDma_getDmaResInfo(hIcssg->hDma, dmaResInfo, i);
            rmCfg.rxStartFlowIdx[i] = dmaResInfo->rxStartIdx;
            rmCfg.rxFlowIdxCnt[i]   = dmaResInfo->rxIdxCnt;
        }

        if (rmCfg.resPartInfo.numCores > 1U)
        {
            ENETTRACE_ERR("%s: number of RM cores not supported\r\n", ENET_PER_NAME(hIcssg));
            status = ENET_ENOTSUPPORTED;
        }

        if (status == ENET_SOK)
        {
            status = EnetMod_open(hIcssg->hRm, enetType, instId, &rmCfg, sizeof(rmCfg));
            ENETTRACE_ERR_IF((status != ENET_SOK),
                             "%s: failed to open RM: %d\r\n",
                             ENET_PER_NAME(hIcssg), status);
        }
    }

    /* Disable PRUSS in preparation for firmware download */
    if (status == ENET_SOK)
    {
        status = Icssg_disablePruss(hIcssg);
        ENETTRACE_ERR_IF((status != ENET_SOK),
                         "%s: failed to disable PRUSS: %d\r\n",
                         ENET_PER_NAME(hIcssg), status);
    }

    /* Download ICSSG firmware */
    if (status == ENET_SOK)
    {
        status = Icssg_configAndDownloadFw(hIcssg, icssgCfg);
        ENETTRACE_ERR_IF((status != ENET_SOK),
                         "%s: failed to download firmware: %d\r\n",
                         ENET_PER_NAME(hIcssg), status);
    }

    /* Enable PRUSS */
    if (status == ENET_SOK)
    {
        status = Icssg_enablePruss(hIcssg);
        ENETTRACE_ERR_IF((status != ENET_SOK),
                         "%s: failed to enable PRUSS: %d\r\n",
                         ENET_PER_NAME(hIcssg), status);
    }

    /* Initialize switch's VLAN and configure default VLAN for host port */
    if ((status == ENET_SOK) &&
        (enetType == ENET_ICSSG_SWITCH) &&
        (icssgCfg->vlanCfg.portVID != 0U))
    {
        status = Icssg_configHostPortDfltVlanId(hIcssg, &icssgCfg->vlanCfg);
        ENETTRACE_ERR_IF((status != ENET_SOK),
                         "%s: failed to set host port default VLAN: %d\r\n",
                         ENET_PER_NAME(hIcssg), status);
    }

    /* Set one-to-one priority regeneration mapping */
    if (status == ENET_SOK)
    {
        status = Icssg_initPriorityRegen(hIcssg);
    }

    /* Open Stats */
    if (status == ENET_SOK)
    {
        status = EnetMod_open(hIcssg->hStats,
                              enetType,
                              instId,
                              NULL,
                              0U);
        ENETTRACE_ERR_IF((status != ENET_SOK),
                         "%s: failed to initialize stats: %d\r\n",
                         ENET_PER_NAME(hIcssg), status);
    }

    /* Open TimeSync */
    if (status == ENET_SOK)
    {
        if (icssgCfg->timeSyncCfg.enable)
        {
            if (!hIcssg->pruss->iep0InUse)
            {
                status = EnetMod_open(hIcssg->hTimeSync,
                                      enetType,
                                      instId,
                                      &icssgCfg->timeSyncCfg,
                                      sizeof(icssgCfg->timeSyncCfg));
                ENETTRACE_ERR_IF((status != ENET_SOK),
                                 "%s: failed to initialize time sync: %d\r\n",
                                 ENET_PER_NAME(hIcssg), status);

                /* TimeSync uses IEP0, so it can be enabled only in one peripheral
                 * per ICSSG instance.  This is not a problem for Switch mode, but
                 * it's for Dual-MAC where it can be enabled only for one port at
                 * a time. */
                if (status == ENET_SOK)
                {
                    hIcssg->pruss->iep0InUse = true;
                }
            }
            else
            {
                ENETTRACE_WARN("%s: TimeSync cannot be enabled, IEP0 already in use\r\n",
                               ENET_PER_NAME(hIcssg));
                hIcssg->hTimeSync = NULL;
            }
        }
        else
        {
            hIcssg->hTimeSync = NULL;
        }
    }

    /* Open MDIO */
    if (status == ENET_SOK)
    {
        status = EnetMod_open(hIcssg->hMdio,
                              enetType,
                              instId,
                              &icssgCfg->mdioCfg,
                              sizeof(icssgCfg->mdioCfg));
        ENETTRACE_ERR_IF((status != ENET_SOK),
                         "%s: failed to open MDIO: %d\r\n",
                         ENET_PER_NAME(hIcssg), status);
    }

    /* Open Tas */
    if (status == ENET_SOK)
    {
        status = EnetMod_open(hIcssg->hTas[0],
                              enetType,
                              instId,
                              NULL,
                              0);
        ENETTRACE_ERR_IF((status != ENET_SOK),
                         "%s: failed to initialize tas[0]: %d\n",
                         ENET_PER_NAME(hIcssg), status);

        if ((status == ENET_SOK) &&
            (enetType == ENET_ICSSG_SWITCH))
        {
            status = EnetMod_open(hIcssg->hTas[1],
                                  enetType,
                                  instId,
                                  NULL,
                                  0);
            ENETTRACE_ERR_IF((status != ENET_SOK),
                             "%s: failed to initialize tas[1]: %d\n",
                             ENET_PER_NAME(hIcssg), status);

        }

    }
    if (status == ENET_SOK)
    {
        status = Icssg_cfgMdioLinkInt(hIcssg, enetType, instId, icssgCfg);
    }

    /* Initialize PHY handles to NULL */
    for (uint32_t phyHandleNum = 0; phyHandleNum < ICSSG_MAC_PORT_MAX; phyHandleNum++)
    {
        hIcssg->hPhy[phyHandleNum] = NULL;
    }

    /* All initialization is complete */
    if (status == ENET_SOK)
    {
        hIcssg->selfCoreId = icssgCfg->resCfg.selfCoreId;
    }

    return status;
}

int32_t Icssg_rejoin(EnetPer_Handle hPer,
                     Enet_Type enetType,
                     uint32_t instId)
{
    ENETTRACE_ERR("%s: rejoin not supported\r\n", hPer->name);

    return ENET_ENOTSUPPORTED;
}

int32_t Icssg_ioctl(EnetPer_Handle hPer,
                    uint32_t cmd,
                    Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    uint32_t major;
    int32_t status = ENET_SOK;

#if ENET_CFG_IS_ON(DEV_ERROR)
    /* Validate ICSSG peripheral IOCTL parameters */
    if ((ENET_IOCTL_GET_PER(cmd) == ENET_IOCTL_PER_ICSSG) &&
        (ENET_IOCTL_GET_TYPE(cmd) == ENET_IOCTL_TYPE_PUBLIC))
    {
        status = Enet_validateIoctl(cmd, prms,
                                    gIcssg_ioctlValidate,
                                    ENET_ARRAYSIZE(gIcssg_ioctlValidate));
        ENETTRACE_ERR_IF((status != ENET_SOK),
                         "%s: IOCTL 0x%08x params are not valid\r\n",
                         ENET_PER_NAME(hIcssg), cmd);
    }
#endif

    if (status == ENET_SOK)
    {
        major = ENET_IOCTL_GET_MAJ(cmd);
        switch (major)
        {
            case ENET_IOCTL_PER_BASE:
            {
                status = Icssg_ioctlInternal(hPer, cmd, prms);
                ENETTRACE_ERR_IF((status < ENET_SOK),
                                 "%s: failed to run ICSSG Per IOCTL 0x%08x: %d\r\n",
                                 ENET_PER_NAME(hIcssg), cmd, status);
            }
            break;

            case ENET_IOCTL_MDIO_BASE:
            {
                status = EnetMod_ioctl(hIcssg->hMdio, cmd, prms);
                ENETTRACE_ERR_IF((status != ENET_SOK),
                                 "%s: failed to run MDIO IOCTL 0x%08x: %d\r\n",
                                 ENET_PER_NAME(hIcssg), cmd, status);
            }
            break;

            case ENET_IOCTL_RM_BASE:
            {
                status = EnetMod_ioctl(hIcssg->hRm, cmd, prms);
                ENETTRACE_ERR_IF((status != ENET_SOK),
                                 "%s: failed to run RM IOCTL 0x%08x: %d\r\n",
                                 ENET_PER_NAME(hIcssg), cmd, status);
            }
            break;

            case ENET_IOCTL_TIMESYNC_BASE:
            {
                if (hIcssg->hTimeSync != NULL)
                {
                    status = EnetMod_ioctl(hIcssg->hTimeSync, cmd, prms);
                }
                else
                {
                    ENETTRACE_ERR("%s: TimeSync is not enabled\r\n", ENET_PER_NAME(hIcssg));
                    status = ENET_ENOTSUPPORTED;
                }
            }
            break;

            case ENET_IOCTL_TAS_BASE:
            {
                /* Note: Typecast to GenericInArgs is possible because all public
                 * MAC port IOCTL input args have macPort as their first member */
                EnetTas_GenericInArgs *inArgs = (EnetTas_GenericInArgs *)prms->inArgs;
                uint32_t portNum = ENET_MACPORT_NORM(inArgs->macPort);
                EnetMod_Handle hTas;

                if (portNum < ENET_ARRAYSIZE(hIcssg->hTas))
                {
                    hTas = hIcssg->hTas[portNum];
                    if (hTas != NULL)
                    {
                        status = EnetMod_ioctl(hTas, cmd, prms);
                    }
                    else
                    {
                        ENETTRACE_ERR("%s: Tas is not enabled\n", ENET_PER_NAME(hIcssg));
                        status = ENET_ENOTSUPPORTED;
                    }
                }
                else
                {
                    ENETTRACE_ERR("%s: invalid MAC port %u\n",
                                  ENET_PER_NAME(hIcssg), ENET_MACPORT_ID(inArgs->macPort));
                    status = ENET_EINVALIDPARAMS;
                }
            }
            break;

            case ENET_IOCTL_FDB_BASE:
            case ENET_IOCTL_HOSTPORT_BASE:
            {
                ENETTRACE_ERR("%s: IOCTL 0x%08x not implemented for ICSSG\r\n",
                              ENET_PER_NAME(hIcssg), cmd);
                status = ENET_ENOTSUPPORTED;
            }
            break;

            case ENET_IOCTL_MACPORT_BASE:
            {
                status = Icssg_ioctlMacPort(hIcssg, cmd, prms);
                ENETTRACE_ERR_IF((status != ENET_SOK),
                                 "%s: failed to run IOCTL 0x%08x: %d\r\n",
                                 ENET_PER_NAME(hIcssg), cmd, status);
            }
            break;

            case ENET_IOCTL_STATS_BASE:
            {
                status = EnetMod_ioctl(hIcssg->hStats, cmd, prms);
            }
            break;

            case ENET_IOCTL_PHY_BASE:
            {
                /* Note: Typecast to GenericInArgs is possible because all public
                 * MAC port IOCTL input args have macPort as their first member */
                EnetPhy_GenericInArgs *inArgs = (EnetPhy_GenericInArgs *)prms->inArgs;
                uint32_t portNum = ENET_MACPORT_NORM(inArgs->macPort);
                EnetPhy_Handle hPhy;

                if (portNum < ENET_ARRAYSIZE(hIcssg->hPhy))
                {
                    hPhy = hIcssg->hPhy[portNum];
                    if (hPhy != NULL)
                    {
                        status = EnetPhyMdioDflt_ioctl(hPhy, cmd, prms);
                    }
                    else
                    {
                        ENETTRACE_ERR("%s: port %u is not open\r\n",
                                      ENET_PER_NAME(hIcssg), ENET_MACPORT_ID(inArgs->macPort));
                        status = ENET_EFAIL;
                    }
                }
                else
                {
                    ENETTRACE_ERR("%s: invalid MAC port %u\r\n",
                                  ENET_PER_NAME(hIcssg), ENET_MACPORT_ID(inArgs->macPort));
                    status = ENET_EINVALIDPARAMS;
                }
            }
            break;

            default:
                status = ENET_EUNKNOWNIOCTL;
                break;
        }
    }

    return status;
}

static int32_t Icssg_ioctlInternal(EnetPer_Handle hPer,
                                   uint32_t cmd,
                                   Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;
    IcssgInternalIoctlHandlerFxn_t * ioctlHandler;

    ioctlHandler = Icssg_getInternalIoctlHandler(hPer, cmd, IcssgInternalIoctlHandlerTable , ENET_ARRAYSIZE(IcssgInternalIoctlHandlerTable));
    Enet_assert(ioctlHandler != NULL);
    status = ioctlHandler(hPer, cmd, prms);
    return status;
}

static int32_t Icssg_ioctlMacPort(Icssg_Handle hIcssg,
                                  uint32_t cmd,
                                  Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;
    IcssgMacPortIoctlHandlerFxn_t * ioctlHandler;

    ioctlHandler = Icssg_getMacPortIoctlHandler(&hIcssg->enetPer, cmd, IcssgMacPortIoctlHandlerTable , ENET_ARRAYSIZE(IcssgInternalIoctlHandlerTable));
    Enet_assert(ioctlHandler != NULL);
    status = ioctlHandler(&hIcssg->enetPer, cmd, prms);
    return status;
}

static int32_t Icssg_pollTxTs(EnetPer_Handle hPer,
                              const void *arg,
                              uint32_t argSize)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    Icssg_TxTsEvtCbInfo tsEvtCbInfo;
    Enet_MacPort macPort;
    uint64_t ts;
    uint32_t *pMgmtPkt = NULL;
    uint32_t slice;
    int32_t hwQLevel;
    int32_t status = ENET_SOK;

    if (argSize != sizeof(Enet_MacPort))
    {
        ENETTRACE_ERR("%s: Incorrect arg size for ENET_EVT_TIMESTAMP_TX (exp %u, got %u)\r\n",
                      ENET_PER_NAME(hIcssg), sizeof(Enet_MacPort), argSize);
        status = ENET_EINVALIDPARAMS;
    }

    if (arg == NULL)
    {
        ENETTRACE_ERR("%s: Invalid arg for ENET_EVT_TIMESTAMP_TX\r\n", ENET_PER_NAME(hIcssg));
        status = ENET_EINVALIDPARAMS;
    }

    if (status == ENET_SOK)
    {
        macPort = *(Enet_MacPort *)arg;

        hwQLevel = IcssgUtils_hwqLevel(hIcssg, macPort, ICSSG_TXTS_RX_HWQA);
        while (hwQLevel != 0)
        {
            pMgmtPkt = (uint32_t*)IcssgUtils_hwqPop(hIcssg, macPort, ICSSG_TXTS_RX_HWQA);
            if (pMgmtPkt != NULL)
            {
                slice = ENET_GET_BIT(pMgmtPkt[0], 23);
                tsEvtCbInfo.txTsId = pMgmtPkt[2];
                ts = pMgmtPkt[4];
                ts = ts << 32U | pMgmtPkt[3];
                tsEvtCbInfo.ts = Icssg_convertTs(hPer, ts);

                if (hIcssg->txTsCbEvtInfo.evtCb != NULL)
                {
                    hIcssg->txTsCbEvtInfo.evtCb(ENET_EVT_TIMESTAMP_TX,
                                                0U,
                                                hIcssg->txTsCbEvtInfo.evtCbArgs,
                                                (void *)&tsEvtCbInfo,
                                                (void *)&macPort);
                }

                /* Pop from port-dependent HwQ, but push into specific HwQ as indicated
                 * by bit 23 of word 0, irrespective of port number */
                IcssgUtils_hwqPushForSlice(hIcssg, slice, ICSSG_TXTS_FREE_HWQA, pMgmtPkt);

                hwQLevel = IcssgUtils_hwqLevel(hIcssg, macPort, ICSSG_TXTS_RX_HWQA);
            }
        }
    }

    return status;
}

static void Icssg_pollHwQAsyncResp(EnetPer_Handle hPer)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    Enet_MacPort macPort;
    Icssg_IoctlCmdResp resp;
    Icssg_IoctlCmd *cmd;
    uint32_t *pMgmtPkt = NULL;
    int32_t hwQLevel;
    uint32_t maxPorts;
    uint32_t i;

    /* Get the max number of ports */
    maxPorts = EnetSoc_getMacPortMax(hPer->enetType, hPer->instId);

    for (i = 0U; i < maxPorts; i++)
    {
        macPort = ENET_MACPORT_DENORM(i);

        hwQLevel = IcssgUtils_hwqLevel(hIcssg, macPort, ICSSG_MGMT_RX_HWQA);
        while (hwQLevel != 0)
        {
            pMgmtPkt = (uint32_t*)IcssgUtils_hwqPop(hIcssg, macPort, ICSSG_MGMT_RX_HWQA);
            if (pMgmtPkt != NULL)
            {
                cmd = (Icssg_IoctlCmd *)&pMgmtPkt[1];
                resp.status = cmd->param;
                resp.seqNum = hIcssg->asyncIoctlSeqNum;
                resp.paramsLen = 0;

                /* for command response with status 0x3, its cmd response for ADD FDB entry.
                   if adding entry results in removing aged-out FDB entry, that is returned to caller
                   in response Params*/
                if (resp.status == 0x3)
                {
                    resp.paramsLen = 2;
                    memcpy(&resp.params, cmd->spare, 2);
                }

                if (hIcssg->asyncCmdRespCbEvtInfo.evtCb != NULL)
                {
                    hIcssg->asyncCmdRespCbEvtInfo.evtCb(ENET_EVT_ASYNC_CMD_RESP,
                                                        0U,
                                                        hIcssg->asyncCmdRespCbEvtInfo.evtCbArgs,
                                                        (void *)&resp,
                                                        NULL);
                }

                IcssgUtils_hwqPush(hIcssg, macPort, ICSSG_MGMT_FREE_HWQA, pMgmtPkt);

                hwQLevel = IcssgUtils_hwqLevel(hIcssg, macPort, ICSSG_MGMT_RX_HWQA);
            }
        }
    }
}

static void Icssg_pollR30CmdAsyncResp(EnetPer_Handle hPer)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    Enet_MacPort macPort;
    Icssg_IoctlCmdResp resp;
    uint32_t maxPorts;
    uint32_t i;

    /* Get the max number of ports */
    maxPorts = EnetSoc_getMacPortMax(hPer->enetType, hPer->instId);

    for (i = 0U; i < maxPorts; i++)
    {
        macPort = ENET_MACPORT_DENORM(i);

        if (IcssgUtils_isR30CmdDone(hIcssg, macPort))
        {
            hIcssg->asyncIoctlType = 0;
            if (hIcssg->asyncCmdRespCbEvtInfo.evtCb != NULL)
            {
                resp.seqNum = hIcssg->asyncIoctlSeqNum;
                resp.status = 1;
                resp.paramsLen = 0;
                hIcssg->asyncCmdRespCbEvtInfo.evtCb(ENET_EVT_ASYNC_CMD_RESP,
                                                    0U,
                                                    hIcssg->asyncCmdRespCbEvtInfo.evtCbArgs,
                                                    (void *)&resp,
                                                    NULL);
            }
        }
    }
}

void Icssg_poll(EnetPer_Handle hPer,
                Enet_Event evt,
                const void *arg,
                uint32_t argSize)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;

    ENETTRACE_DBG("%s: event %d\r\n", ENET_PER_NAME(hIcssg), evt);

    if (evt == ENET_EVT_TIMESTAMP_TX)
    {
        Icssg_pollTxTs(hPer, arg, argSize);
    }
    else if (evt == ENET_EVT_ASYNC_CMD_RESP)
    {
        if (hIcssg->asyncIoctlType == ICSSG_UTILS_IOCTL_TYPE_R30_OVER_DMEM)
        {
            Icssg_pollR30CmdAsyncResp(hPer);
        }
        else
        {
            Icssg_pollHwQAsyncResp(hPer);
        }
    }
    else
    {
        ENETTRACE_ERR("%s: Invalid event %d\r\n", ENET_PER_NAME(hIcssg), evt);
    }
}

void Icssg_registerEventCb(EnetPer_Handle hPer,
                           Enet_Event evt,
                           uint32_t evtNum,
                           Enet_EventCallback evtCb,
                           void *evtCbArgs)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;

    if ((ENET_EVT_ASYNC_CMD_RESP == evt) && (0U == evtNum))
    {
        ENETTRACE_WARN_IF((hIcssg->asyncCmdRespCbEvtInfo.evtCb != NULL),
                         "%s: event %u already registered, overwriting config\r\n",
                         ENET_PER_NAME(hIcssg), evt);

        hIcssg->asyncCmdRespCbEvtInfo.evtCb = evtCb;
        hIcssg->asyncCmdRespCbEvtInfo.evtCbArgs = evtCbArgs;
    }
    else if ((ENET_EVT_TIMESTAMP_TX == evt) && (0U == evtNum))
    {
        ENETTRACE_WARN_IF((hIcssg->txTsCbEvtInfo.evtCb != NULL),
                         "%s: event %u already registered, overwriting config\r\n",
                         ENET_PER_NAME(hIcssg), evt);
        hIcssg->txTsCbEvtInfo.evtCb = evtCb;
        hIcssg->txTsCbEvtInfo.evtCbArgs = evtCbArgs;
    }
    else
    {
        ENETTRACE_ERR("%s: invalid event %u\r\n", ENET_PER_NAME(hIcssg), evt);
    }
}

void Icssg_unregisterEventCb(EnetPer_Handle hPer,
                             Enet_Event evt,
                             uint32_t evtNum)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;

    if ((ENET_EVT_ASYNC_CMD_RESP == evt) && (0U == evtNum) &&
        (hIcssg->asyncCmdRespCbEvtInfo.evtCb != NULL))
    {
        hIcssg->asyncCmdRespCbEvtInfo.evtCb = NULL;
        hIcssg->asyncCmdRespCbEvtInfo.evtCbArgs = NULL;
    }
    else if ((ENET_EVT_TIMESTAMP_TX == evt) && (0U == evtNum) &&
             (hIcssg->txTsCbEvtInfo.evtCb != NULL))
    {
        hIcssg->txTsCbEvtInfo.evtCb = NULL;
        hIcssg->txTsCbEvtInfo.evtCbArgs = NULL;
    }
    else
    {
        ENETTRACE_ERR("%s: event not registered %u\r\n", ENET_PER_NAME(hIcssg), evt);
    }
}

void Icssg_periodicTick(EnetPer_Handle hPer)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    EnetPhy_Handle hPhy;
    Enet_MacPort macPort;
    EnetPhy_LinkStatus linkStatus;
    bool linked;
    uint32_t portId;
    uint32_t i;
    int32_t status = ENET_EFAIL;

    /* Run PHY tick */
    for (i = 0U; i < ICSSG_MAC_PORT_MAX; i++)
    {
        hPhy = hIcssg->hPhy[i];
        /* Check if the corresponding PHY is enabled */
        if (hPhy != NULL)
        {
            macPort = ENET_MACPORT_DENORM(i);
            portId = ENET_MACPORT_ID(macPort);

            ENETTRACE_VAR(portId);
            ENETTRACE_VAR(status);
            /* TODO: Need to make lock more granular */
            //EnetOsal_lockMutex(hIcssg->lock);

            /* Run PHY tick */
            linkStatus = EnetPhy_tick(hPhy);

            /* Handle link up/down events */
            if ((linkStatus == ENETPHY_GOT_LINK) ||
                (linkStatus == ENETPHY_LOST_LINK))
            {
                linked = (linkStatus == ENETPHY_GOT_LINK);
                status = linked ? Icssg_handleLinkUp(hIcssg, macPort) :
                                  Icssg_handleLinkDown(hIcssg, macPort);
                ENETTRACE_ERR_IF((status != ENET_SOK),
                                 "%s: Port %u: Failed to handle link change: %d\r\n",
                                 ENET_PER_NAME(hIcssg), portId, status);
            }

            //EnetOsal_unlockMutex(hIcssg->lock);
        }
    }
}

void Icssg_closeDma(Icssg_Handle hIcssg)
{
    Enet_assert (hIcssg->hDma != NULL);
    EnetHostPortDma_close(hIcssg->hDma);
    hIcssg->hDma = NULL;
}

void Icssg_close(EnetPer_Handle hPer)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    uintptr_t key;

    ENETTRACE_DBG("%s: close peripheral\r\n", ENET_PER_NAME(hIcssg));

    key = EnetOsal_disableAllIntr();

    /* Close MDIO module */
    EnetMod_close(hIcssg->hMdio);

    Icssg_unregisterMdioLinkIntr(hIcssg);

    /* Set PHY handles to NULL */
    for (uint32_t phyHandleNum = 0; phyHandleNum < ICSSG_MAC_PORT_MAX; phyHandleNum++)
    {
        hIcssg->hPhy[phyHandleNum] = NULL;
    }

    /* Close TimeSync module, if opened */
    if (hIcssg->hTimeSync != NULL)
    {
        EnetMod_close(hIcssg->hTimeSync);
        hIcssg->pruss->iep0InUse = false;
    }

    /* Close Tas module, if opened */
    if (hIcssg->hTas[0] != NULL)
    {
        EnetMod_close(hIcssg->hTas[0]);
    }

    /* Close Tas module, if opened */
    if (hIcssg->hTas[1] != NULL)
    {
        EnetMod_close(hIcssg->hTas[1]);
    }

    /* Close DMA */
    Icssg_closeDma(hIcssg);

    /* Close RM */
    EnetMod_close(hIcssg->hRm);

    /* Close statistics module */
    EnetMod_close(hIcssg->hStats);

    EnetOsal_restoreAllIntr(key);
}

static void Icssg_setPromiscMode(Icssg_Handle hIcssg,
                                 Enet_MacPort macPort,
                                 bool enable)
{
#ifdef ICSSG_PROMISC_MODE_WORKAROUND
    /* Implement promiscuous mode using ucast/mcast flooding as a workaround for filter-based
     * promiscuous mode not being fully functional */
    Icssg_UCastFloodingCtrl(hIcssg, macPort, enable);
    Icssg_MCastFloodingCtrl(hIcssg, macPort, enable);
#else
    uintptr_t cfgRegs = Icssg_getCfgAddr(hIcssg);
    uint32_t slice = IcssgUtils_getSliceNum(hIcssg, macPort);

    if (enable == true)
    {
        if (ICSSG_IS_SLICE_0(slice))
        {
            Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT1_START_LEN_PRU0, 0x60000U);
            Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT1_CFG_PRU0, 0x5555U);
            Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT1_0_DA0_PRU0, 0x0U);
            Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT1_0_DA1_PRU0, 0x0U);
            Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT1_0_DA_MASK0_PRU0, 0xFFFFFFFEU);
            Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT1_0_DA_MASK1_PRU0, 0xFFFFU);
            Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS4_AND_EN_PRU0, 0x0U);
            Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS4_OR_EN_PRU0, 0x60010000U);
            Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS_CFG1_PRU0, 0x300U);
            Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS_CFG2_PRU0, 0x0U);
        }
        else
        {
            Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT1_START_LEN_PRU1, 0x60000U);
            Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT1_CFG_PRU1, 0x5555U);
            Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT1_0_DA0_PRU1, 0x0U);
            Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT1_0_DA1_PRU1, 0x0U);
            Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT1_0_DA_MASK0_PRU1, 0xFFFFFFFEU);
            Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT1_0_DA_MASK1_PRU1, 0xFFFFU);
            Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS4_AND_EN_PRU1, 0x0U);
            Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS4_OR_EN_PRU1, 0x60010000U);
            Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS_CFG1_PRU1, 0x300U);
            Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS_CFG2_PRU1, 0x0U);
        }
    }
    else
    {
        if (ICSSG_IS_SLICE_0(slice))
        {
            Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT1_START_LEN_PRU0, 0x60000U);
            Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT1_CFG_PRU0, 0x5555U);
            Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT1_0_DA0_PRU0, 0x0U);
            Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT1_0_DA1_PRU0, 0x0U);
            Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT1_0_DA_MASK0_PRU0, 0x0U);
            Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT1_0_DA_MASK1_PRU0, 0x0U);
            Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS4_AND_EN_PRU0, 0x0U);
            Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS4_OR_EN_PRU0, 0x22000000U);
            Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS_CFG1_PRU0, 0x300U);
            Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS_CFG2_PRU0, 0x0U);
        }
        else
        {
            Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT1_START_LEN_PRU1, 0x60000U);
            Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT1_CFG_PRU1, 0x5555U);
            Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT1_0_DA0_PRU1, 0x0U);
            Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT1_0_DA1_PRU1, 0x0U);
            Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT1_0_DA_MASK0_PRU1, 0x0U);
            Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT1_0_DA_MASK1_PRU1, 0x0U);
            Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS4_AND_EN_PRU1, 0x0U);
            Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS4_OR_EN_PRU1, 0x22000000U);
            Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS_CFG1_PRU1, 0x300U);
            Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS_CFG2_PRU1, 0x0U);
        }
    }
#endif
}

static void Icssg_ioctlCfgAgeingPeriod(Icssg_Handle hIcssg,
                                       uint64_t ageingPeriod)
{
    uintptr_t sharedRam = Icssg_getSharedRamAddr(hIcssg);
    uint32_t numFDBBuckets = ICSSG_SIZE_OF_FDB / ICSSG_NUM_FDB_BUCKET_ENTRIES;

    /* The actual value written to memory is aging timeout divided by
     * number of buckets because firmware iterates per bucket not for
     * entire FDB. See NRT design doc for more details. */
    Icssg_wr64(hIcssg, sharedRam + FDB_AGEING_TIMEOUT_OFFSET, 2*((uint64_t)ageingPeriod / (uint64_t)numFDBBuckets));
}

static void Icssg_ioctlVlanFidResetTable(Icssg_Handle hIcssg,
                                         Icssg_VlanFidParams *vlanFidParams)
{
    uintptr_t vlanTableAddr = Icssg_getVlanTableAddr(hIcssg);
    uint32_t entry;
    uint16_t vlanEntry;

    vlanEntry = (vlanFidParams->hostMember  << 0)   |
                (vlanFidParams->p1Member    << 1)   |
                (vlanFidParams->p2Member    << 2)   |
                (vlanFidParams->hostTagged  << 3)   |
                (vlanFidParams->p1Tagged    << 4)   |
                (vlanFidParams->p2Tagged    << 5)   |
                (vlanFidParams->streamVid   << 6)   |
                (vlanFidParams->floodToHost << 7);

    vlanEntry  |=  (vlanFidParams->fid << 8U);

    for (entry = 0U; entry < (ICSSG_VLAN_TBL_MAX_ENTRIES - 1U); entry++)
    {
        Icssg_wr16(hIcssg, vlanTableAddr + (entry * 2U), vlanEntry);
    }
}

static int32_t Icssg_ioctlVlanFidSetEntry(Icssg_Handle hIcssg,
                                          Icssg_VlanFidEntry *vlanFidEntry)
{
    uintptr_t vlanTableAddr = Icssg_getVlanTableAddr(hIcssg);
    int32_t retVal = ENET_SOK;
    uint16_t vlanEntry = 0U;

    if (vlanFidEntry->vlanId > (ICSSG_VLAN_TBL_MAX_ENTRIES - 1U))
    {
        retVal = ENET_EINVALIDPARAMS;
    }
    else
    {
        vlanEntry |= vlanFidEntry->vlanFidParams.hostMember  ? ENET_BIT(0) : 0U;
        vlanEntry |= vlanFidEntry->vlanFidParams.p1Member    ? ENET_BIT(1) : 0U;
        vlanEntry |= vlanFidEntry->vlanFidParams.p2Member    ? ENET_BIT(2) : 0U;
        vlanEntry |= vlanFidEntry->vlanFidParams.hostTagged  ? ENET_BIT(3) : 0U;
        vlanEntry |= vlanFidEntry->vlanFidParams.p1Tagged    ? ENET_BIT(4) : 0U;
        vlanEntry |= vlanFidEntry->vlanFidParams.p2Tagged    ? ENET_BIT(5) : 0U;
        vlanEntry |= vlanFidEntry->vlanFidParams.streamVid   ? ENET_BIT(6) : 0U;
        vlanEntry |= vlanFidEntry->vlanFidParams.floodToHost ? ENET_BIT(7) : 0U;
        vlanEntry |= vlanFidEntry->vlanFidParams.fid << 8U;

        Icssg_wr16(hIcssg, vlanTableAddr + (vlanFidEntry->vlanId * 2U), vlanEntry);
    }

    return retVal;
}

static int32_t Icssg_ioctlVlanFidGetEntry(Icssg_Handle hIcssg,
                                          Icssg_VlanFidEntry *vlanFidEntry)
{
    uintptr_t vlanTableAddr = Icssg_getVlanTableAddr(hIcssg);
    int32_t retVal = ENET_SOK;
    uint16_t val;

    if (vlanFidEntry->vlanId > (ICSSG_VLAN_TBL_MAX_ENTRIES - 1))
    {
        retVal = ENET_EINVALIDPARAMS;
    }
    else
    {
        val = Icssg_rd16(hIcssg, vlanTableAddr + (vlanFidEntry->vlanId * 2U));
        vlanFidEntry->vlanFidParams.hostMember  = ENET_IS_BIT_SET(val, 0U);
        vlanFidEntry->vlanFidParams.p1Member    = ENET_IS_BIT_SET(val, 1U);
        vlanFidEntry->vlanFidParams.p2Member    = ENET_IS_BIT_SET(val, 2U);
        vlanFidEntry->vlanFidParams.hostTagged  = ENET_IS_BIT_SET(val, 3U);
        vlanFidEntry->vlanFidParams.p1Tagged    = ENET_IS_BIT_SET(val, 4U);
        vlanFidEntry->vlanFidParams.p2Tagged    = ENET_IS_BIT_SET(val, 5U);
        vlanFidEntry->vlanFidParams.streamVid   = ENET_IS_BIT_SET(val, 6U);
        vlanFidEntry->vlanFidParams.floodToHost = ENET_IS_BIT_SET(val, 7U);
        vlanFidEntry->vlanFidParams.fid         = (val & 0xFF00U) >> 8U;
    }

    return retVal;
}

static int32_t Icssg_ioctlSetPortState(Icssg_Handle hIcssg,
                                       IcssgMacPort_SetPortStateInArgs *portStateCfg)
{
    IcssgUtils_ioctlR30Cmd cmd;
    int32_t status = ENET_SOK;

    switch (portStateCfg->portState)
    {
        case ICSSG_PORT_STATE_DISABLED:
            cmd = ICSSG_UTILS_R30_CMD_DISABLE;
            break;

        case ICSSG_PORT_STATE_BLOCKING:
            cmd = ICSSG_UTILS_R30_CMD_BLOCK;
            break;

        case ICSSG_PORT_STATE_FORWARD:
            cmd = ICSSG_UTILS_R30_CMD_FORWARD;
            break;

        case ICSSG_PORT_STATE_FORWARD_WO_LEARNING:
            cmd = ICSSG_UTILS_R30_CMD_FORWARD_WO_LEARNING;
            break;

        /* TODO: Legacy TAS implementation to be replaced with proper IOCTLs */
        case ICSSG_PORT_STATE_TAS_TRIGGER:
            cmd = ICSSG_UTILS_R30_CMD_TAS_TRIGGER;
            break;

        case ICSSG_PORT_STATE_TAS_ENABLE:
            cmd = ICSSG_UTILS_R30_CMD_TAS_ENABLE;
            break;

        case ICSSG_PORT_STATE_TAS_RESET:
            cmd = ICSSG_UTILS_R30_CMD_TAS_RESET;
            break;

        case ICSSG_PORT_STATE_TAS_DISABLE:
            cmd = ICSSG_UTILS_R30_CMD_DISABLE;
            break;

        default:
            status = ENET_EINVALIDPARAMS;
            break;
    }

    if (status == ENET_SOK)
    {
        status = Icssg_R30SendAsyncIoctl(hIcssg,
                                         portStateCfg->macPort,
                                         cmd,
                                         &hIcssg->asyncIoctlSeqNum,
                                         &hIcssg->asyncIoctlType);
    }

    return status;
}

static void Icssg_getVlanId(Icssg_Handle hIcssg,
                            Enet_MacPort macPort,
                            Icssg_FdbEntry *fdbEntry)
{
    uintptr_t cfgRegs = Icssg_getCfgAddr(hIcssg);
    uint32_t slice = IcssgUtils_getSliceNum(hIcssg, macPort);

    /*
     * vlanId is -1 in two scenarios:
     *  o VLAN aware mode - port's default VLAN id is taken for VLAN untagged
     *  o VLAN unaware mode - original id (-1) is not modified and will result in
     *    a FID value of 0 when creating broadside slot.
     */
    if ((fdbEntry->vlanId == ICSSG_VLAN_UNTAGGED) &&
        Icssg_isVlanAware(hIcssg))
    {
        /* 'slice' here is used to differentiate MAC port 1 and MAC port 2 */
        fdbEntry->vlanId = Icssg_rd16(hIcssg,
                                      cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FDB_DF_VLAN +
                                      (2 * slice));
    }
}

static int32_t Icssg_ioctlFdbAddEntry(Icssg_Handle hIcssg,
                                      Enet_MacPort macPort,
                                      Icssg_FdbEntry *fdbEntry)
{
    uintptr_t vlanTableAddr = Icssg_getVlanTableAddr(hIcssg);
    uint8_t fid;
    uint16_t broadSideSlot;
    int32_t status = ENET_EINVALIDPARAMS;

    if ((fdbEntry->vlanId < (int16_t)(ICSSG_VLAN_TBL_MAX_ENTRIES)) ||
        (fdbEntry->vlanId == ICSSG_VLAN_UNTAGGED))
    {
        Icssg_getVlanId(hIcssg, macPort, fdbEntry);

        broadSideSlot = IcssgUtils_FdbHelper(vlanTableAddr, fdbEntry->vlanId, fdbEntry->macAddr, &fid);

        status = IcssgUtils_sendFdbCmd(hIcssg,
                                       macPort,
                                       ICSSG_IOCTL_SUBCMD_FDB_ENTRY_ADD,
                                       fdbEntry,
                                       broadSideSlot,
                                       fid);
    }

    return status;
}

static int32_t Icssg_ioctlFdbRemoveEntry(Icssg_Handle hIcssg,
                                         Enet_MacPort macPort,
                                         Icssg_FdbEntry *fdbEntry)
{
    uintptr_t vlanTableAddr = Icssg_getVlanTableAddr(hIcssg);
    uint8_t fid;
    uint16_t broadSideSlot;
    int32_t status = ENET_EINVALIDPARAMS;

    if ((fdbEntry->vlanId < (int16_t)(ICSSG_VLAN_TBL_MAX_ENTRIES)) ||
        (fdbEntry->vlanId == ICSSG_VLAN_UNTAGGED))
    {
        Icssg_getVlanId(hIcssg, macPort, fdbEntry);

        broadSideSlot = IcssgUtils_FdbHelper(vlanTableAddr, fdbEntry->vlanId, fdbEntry->macAddr, &fid);

        status = IcssgUtils_sendFdbCmd(hIcssg,
                                       macPort,
                                       ICSSG_IOCTL_SUBCMD_FDB_ENTRY_REMOVE,
                                       fdbEntry,
                                       broadSideSlot,
                                       fid);
    }

    return status;
}

static int32_t Icssg_ioctlFdbRemoveAllEntries(Icssg_Handle hIcssg,
                                              Enet_MacPort macPort)
{
    int32_t status;

    status = IcssgUtils_sendFdbCmd(hIcssg,
                                   macPort,
                                   ICSSG_IOCTL_SUBCMD_FDB_ENTRY_REMOVE_ALL,
                                   NULL,
                                   0,
                                   0);

    return status;
}

static int32_t Icssg_ioctlFdbRemoveAllAgeableEntries(Icssg_Handle hIcssg,
                                                     Enet_MacPort macPort)
{
    int32_t status;

    status = IcssgUtils_sendFdbCmd(hIcssg,
                                   macPort,
                                   ICSSG_IOCTL_SUBCMD_FDB_ENTRY_REMOVE_ALL_AGEABLE,
                                   NULL,
                                   0,
                                   0);

    return status;
}

static void Icssg_ioctlSetMacAddress(Icssg_Handle hIcssg,
                                     IcssgMacPort_SetMacAddressInArgs *macAddressCfg)
{
    int32_t macLo;
    int32_t macHi;
    int16_t temp;
    uint32_t slice;
    uintptr_t cfgRegs = Icssg_getCfgAddr(hIcssg);
    uint8_t *macAddr = &(macAddressCfg->macAddr[0]);

    macLo = *((int32_t *)&macAddr[0]);
    temp  = *((int16_t *)&macAddr[4]);
    macHi = (int32_t)temp;

    slice = IcssgUtils_getSliceNum(hIcssg, macAddressCfg->macPort);

    if (ICSSG_IS_SLICE_0(slice))
    {
        Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_MAC_PRU0_0, macLo);
        Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_MAC_PRU0_1, macHi);
    }
    else
    {
        Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_MAC_PRU1_0, macLo);
        Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_MAC_PRU1_1, macHi);
    }
}

static void Icssg_ioctlSetMacAddressHostPort(Icssg_Handle hIcssg,
                                             Icssg_MacAddr *macAddress)
{
    int32_t macLo;
    int32_t macHi;
    int16_t temp;
    uintptr_t cfgRegs = Icssg_getCfgAddr(hIcssg);
    uint8_t *macAddr = &(macAddress->macAddr[0]);

    macLo =  *((int32_t *)&macAddr[0]);
    temp  =  *((int16_t *)&macAddr[4]);
    macHi = (int32_t)temp;

    Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_MAC_INTERFACE_0, macLo);
    Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_MAC_INTERFACE_1, macHi);
}

static void Icssg_setSpecialFramePrioCfg(Icssg_Handle hIcssg,
                                         Enet_MacPort macPort,
                                         uint8_t specialFramePrio)
{
    uintptr_t dram = Icssg_getDramAddr(hIcssg, macPort);

    Icssg_wr8(hIcssg, dram + SPL_PKT_DEFAULT_PRIORITY, specialFramePrio);
}

static void Icssg_configCutThroughOrPreempt(Icssg_Handle hIcssg,
                                            Enet_MacPort macPort,
                                            Icssg_QueuePreemptMode *queuePreemptMode,
                                            Icssg_QueueForwardMode *queueForwardMode)
{
    uintptr_t dram = Icssg_getDramAddr(hIcssg, macPort);
    uint8_t queueMask = 0U;
    uint8_t queueNum;
    uint8_t val;

    for (queueNum = 0U; queueNum < ENET_PRI_NUM; queueNum++)
    {
        /* bit 4 if set will indicate packet from queue will be pre-emptible.
         * bit 7 if set will indicate that queue data should be cut-through */
        val = (queuePreemptMode[queueNum] << 4U) |
              (queueForwardMode[queueNum] << 7U);

        Icssg_wr8(hIcssg, dram + EXPRESS_PRE_EMPTIVE_Q_MAP + queueNum, val);
        queueMask = queueMask | ((queuePreemptMode[queueNum]) << queueNum);
    }

    Icssg_wr8(hIcssg, dram + EXPRESS_PRE_EMPTIVE_Q_MASK, ~queueMask);
}

static int32_t Icssg_ioctlPreemptTxEnable(Icssg_Handle hIcssg,
                                          Enet_MacPort macPort)
{
    uintptr_t dram = Icssg_getDramAddr(hIcssg, macPort);
    int32_t retVal = ENET_EINVALIDPARAMS;

    if ((macPort == ENET_MAC_PORT_1) ||
        (macPort == ENET_MAC_PORT_2))
    {
        retVal = Icssg_R30SendAsyncIoctl(hIcssg,
                                         macPort,
                                         ICSSG_UTILS_R30_CMD_PREMPT_TX_ENABLE,
                                         &hIcssg->asyncIoctlSeqNum,
                                         &hIcssg->asyncIoctlType);

        if (retVal == ENET_SINPROGRESS)
        {
            Icssg_wr8(hIcssg, dram + PRE_EMPTION_ENABLE_TX, 0x01);
        }
    }

    return retVal;
}

static int32_t Icssg_ioctlPreemptTxDisable(Icssg_Handle hIcssg,
                                           Enet_MacPort macPort)
{
    uintptr_t dram = Icssg_getDramAddr(hIcssg, macPort);
    int32_t retVal = ENET_EINVALIDPARAMS;

    if ((macPort == ENET_MAC_PORT_1) ||
        (macPort == ENET_MAC_PORT_2))
    {
        retVal = Icssg_R30SendAsyncIoctl(hIcssg,
                                         macPort,
                                         ICSSG_UTILS_R30_CMD_PREMPT_TX_DISABLE,
                                         &hIcssg->asyncIoctlSeqNum,
                                         &hIcssg->asyncIoctlType);

        if (retVal == ENET_SINPROGRESS)
        {
            Icssg_wr8(hIcssg, dram + PRE_EMPTION_ENABLE_TX, 0x00);
        }
    }

    return retVal;
}

static int32_t Icssg_ioctlPreemptGetTxEnableStatus(Icssg_Handle hIcssg,
                                                   Enet_MacPort macPort,
                                                   bool *status)
{
    uintptr_t dram = Icssg_getDramAddr(hIcssg, macPort);
    int32_t retVal = ENET_EINVALIDPARAMS;

    if ((macPort == ENET_MAC_PORT_1) ||
        (macPort == ENET_MAC_PORT_2))
    {
        *status = (bool)Icssg_rd8(hIcssg, dram + PRE_EMPTION_ENABLE_TX);
        retVal = ENET_SOK;
    }

    return retVal;
}

static int32_t Icssg_ioctlPreemptGetTxActiveStatus(Icssg_Handle hIcssg,
                                                   Enet_MacPort macPort,
                                                   bool *status)
{
    uintptr_t dram = Icssg_getDramAddr(hIcssg, macPort);
    int32_t retVal = ENET_EINVALIDPARAMS;

    if ((macPort == ENET_MAC_PORT_1) ||
        (macPort == ENET_MAC_PORT_2))
    {
        *status = (bool)Icssg_rd8(hIcssg, dram + PRE_EMPTION_ACTIVE_TX);
        retVal = ENET_SOK;
    }

    return retVal;
}

static int32_t Icssg_ioctlPreemptVerifyEnable(Icssg_Handle hIcssg,
                                              Enet_MacPort macPort)
{
    uintptr_t dram = Icssg_getDramAddr(hIcssg, macPort);
    int32_t retVal = ENET_EINVALIDPARAMS;

    if ((macPort == ENET_MAC_PORT_1) ||
        (macPort == ENET_MAC_PORT_2))
    {
        Icssg_wr8(hIcssg, dram + PRE_EMPTION_ENABLE_VERIFY, 0x01);
        retVal = ENET_SOK;
    }

    return retVal;
}

static int32_t Icssg_ioctlPreemptVerifyDisable(Icssg_Handle hIcssg,
                                               Enet_MacPort macPort)
{
    uintptr_t dram = Icssg_getDramAddr(hIcssg, macPort);
    int32_t retVal = ENET_EINVALIDPARAMS;

    if ((macPort == ENET_MAC_PORT_1) ||
        (macPort == ENET_MAC_PORT_2))
    {
        Icssg_wr8(hIcssg, dram + PRE_EMPTION_ENABLE_VERIFY, 0x00);
        retVal = ENET_SOK;
    }

    return retVal;
}

static int32_t Icssg_ioctlPreemptGetVerifyState(Icssg_Handle hIcssg,
                                                Enet_MacPort macPort,
                                                EnetMacPort_PreemptVerifyStatus *status)
{
    uintptr_t dram = Icssg_getDramAddr(hIcssg, macPort);
    int32_t retVal = ENET_EINVALIDPARAMS;

    if ((macPort == ENET_MAC_PORT_1) ||
        (macPort == ENET_MAC_PORT_2))
    {
        *status = (EnetMacPort_PreemptVerifyStatus)(Icssg_rd8(hIcssg, dram + PRE_EMPTION_VERIFY_STATUS));
        retVal = ENET_SOK;
    }

    return retVal;
}

static int32_t Icssg_ioctlPreemptGetMinFragSizeLocal(Icssg_Handle hIcssg,
                                                     Enet_MacPort macPort,
                                                     uint8_t *minFragSizeLocal)
{
    uintptr_t dram = Icssg_getDramAddr(hIcssg, macPort);
    uint16_t val;
    int32_t retVal = ENET_EINVALIDPARAMS;

    if ((macPort == ENET_MAC_PORT_1) ||
        (macPort == ENET_MAC_PORT_2))
    {
        val = Icssg_rd16(hIcssg, dram + PRE_EMPTION_ADD_FRAG_SIZE_LOCAL) / 64 - 1;
        *minFragSizeLocal = (uint8_t)val;
        retVal = ENET_SOK;
    }

    return retVal;
}

static int32_t Icssg_ioctlPreemptSetMinFragSizeRemote(Icssg_Handle hIcssg,
                                                     EnetMacPort_SetPreemptMinFragSizeInArgs *minFragSizeRemoteArgs)
{
    uintptr_t dram = Icssg_getDramAddr(hIcssg, minFragSizeRemoteArgs->macPort);
    int32_t retVal = ENET_EINVALIDPARAMS;

    if ((minFragSizeRemoteArgs->macPort == ENET_MAC_PORT_1) ||
        (minFragSizeRemoteArgs->macPort == ENET_MAC_PORT_2))
    {
        Icssg_wr16(hIcssg, dram + PRE_EMPTION_ADD_FRAG_SIZE_REMOTE,
                   (minFragSizeRemoteArgs->preemptMinFragSize + 1) * 64);
        retVal = ENET_SOK;
    }

    return retVal;
}

static int32_t Icssg_openEnetPhy(Icssg_Handle hIcssg,
                                 Enet_MacPort macPort,
                                 const EnetPhy_Cfg *phyCfg,
                                 EnetPhy_Mii phyMii,
                                 const EnetPhy_LinkCfg *phyLinkCfg)
{
    EnetPer_Handle hPer = (EnetPer_Handle)hIcssg;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    uint32_t portId = ENET_MACPORT_ID(macPort);
    EnetPhy_MdioHandle hPhyMdio = EnetPhyMdioDflt_getPhyMdio();
    uint32_t macPortCaps = EnetSoc_getMacPortCaps(hPer->enetType, hPer->instId, macPort);
    int32_t status = ENET_SOK;

    ENETTRACE_VAR(hPer);
    ENETTRACE_VAR(portId);
    ENETTRACE_DBG("%s: Port %u: open port link\r\n", ENET_PER_NAME(hIcssg), portId);

    if (portNum < ENET_ARRAYSIZE(hIcssg->hPhy))
    {
        if (hIcssg->disablePhyDriver != true)
        {
            if (hIcssg->hPhy[portNum] == NULL)
            {
                hIcssg->hPhy[portNum] = EnetPhy_open(phyCfg, phyMii, phyLinkCfg, macPortCaps, hPhyMdio, hIcssg->hMdio);
                if (hIcssg->hPhy[portNum] == NULL)
                {
                    ENETTRACE_ERR("%s: Port %u: failed to open PHY\r\n", ENET_PER_NAME(hIcssg), portId);
                    status = ENET_EFAIL;
                }
            }
            else
            {
                ENETTRACE_ERR("%s: Port %u: PHY handle in Icssg handle should be NULL before EnetPhy_open() call \r\n", ENET_PER_NAME(hIcssg), portId);
                status = ENET_EFAIL;
            }
        }
        else
        {
            hIcssg->hPhy[portNum] = NULL;
        }
    }
    else
    {
        ENETTRACE_ERR("%s: invalid MAC port %u\r\n", ENET_PER_NAME(hIcssg), portId);
        status = ENET_EINVALIDPARAMS;
    }

    return status;
}

static void Icssg_closeEnetPhy(Icssg_Handle hIcssg,
                               Enet_MacPort macPort)
{
    EnetPer_Handle hPer = (EnetPer_Handle)hIcssg;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    uint32_t portId = ENET_MACPORT_ID(macPort);
    EnetPhy_Handle hPhy;

    ENETTRACE_VAR(hPer);
    ENETTRACE_VAR(portId);

    if (portNum < ENET_ARRAYSIZE(hIcssg->hPhy))
    {
        hPhy = hIcssg->hPhy[portNum];
        if (hPhy != NULL)
        {
            ENETTRACE_DBG("%s: Port %u: close port link\r\n", ENET_PER_NAME(hIcssg), portId);

            if (EnetPhy_isLinked(hPhy))
            {
                Icssg_handleLinkDown(hIcssg, macPort);
            }

            EnetPhy_close(hPhy);
            hIcssg->hPhy[portNum] = NULL;
        }
    }
    else
    {
        ENETTRACE_ERR("%s: invalid MAC port %u\r\n", ENET_PER_NAME(hIcssg), portId);
    }
}

static int32_t Icssg_ioctlPortLinkCfg(Icssg_Handle hIcssg,
                                      const EnetPer_PortLinkCfg *portLinkCfg)
{
    EnetPer_Handle hPer = (EnetPer_Handle)hIcssg;
    IcssgMacPort_Cfg *macPortCfg = (IcssgMacPort_Cfg *)portLinkCfg->macCfg;
    Enet_MacPort macPort = portLinkCfg->macPort;
    const EnetPhy_Cfg *phyCfg = &portLinkCfg->phyCfg;
    EnetPhy_Mii phyMii;
    EnetPhy_LinkCfg phyLinkCfg;
    int32_t status = ENET_EFAIL;

    ENETTRACE_VAR(status);
    ENETTRACE_VAR(hPer);

    /* Opening a MAC port in switch use case, this does not apply for single MAC use case */
    if (hIcssg->enetPer.enetType == ENET_ICSSG_SWITCH)
    {
        Icssg_configCutThroughOrPreempt(hIcssg, macPort,
                                        &macPortCfg->queuePreemptMode[0U],
                                        &macPortCfg->queueForwardMode[0U]);

        /* Configure acceptable frame check setting */
        status = Icssg_setAcceptableFrameCheckSync(hIcssg, macPort, macPortCfg->acceptFrameCheck);
        ENETTRACE_ERR_IF((status != ENET_SOK),
                         "%s: Port %u: failed to set acceptable frame check: %d\r\n",
                         ENET_PER_NAME(hIcssg), ENET_MACPORT_ID(portLinkCfg->macPort), status);

        /* Set MAC port default VLAN id */
        if ((status == ENET_SOK) &&
            Icssg_isVlanAware(hIcssg))
        {
            status = Icssg_configMacPortDfltVlanId(hIcssg, macPort, &macPortCfg->vlanCfg);
            ENETTRACE_ERR_IF((status != ENET_SOK),
                             "%s: Port %u: failed to set MAC port default VLAN: %d\r\n",
                             ENET_PER_NAME(hIcssg), ENET_MACPORT_ID(portLinkCfg->macPort), status);
        }
    }
    else
    {   /* only applies for single mac use case */
        /* Configure promiscous mode setting */
        Icssg_setPromiscMode(hIcssg, macPort, macPortCfg->promiscEn);
        status = ENET_SOK;
    }

    /* Configure unicast Flooding enable/disable */
    Icssg_UCastFloodingCtrl(hIcssg, macPort, macPortCfg->ucastFloodEn);

    /* Configure multicast Flooding enable/disable */
    Icssg_MCastFloodingCtrl(hIcssg, macPort, macPortCfg->mcastFloodEn);

    /* Config special frame priority */
    Icssg_setSpecialFramePrioCfg(hIcssg, macPort, macPortCfg->specialFramePrio);

    /* Check port mode: MII or RGMII. It must match what has already been set during
     * firmware config by either this peripheral or another peripheral sharing the
     * same ICCSG (i.e. in Dual-MAC mode) */
    if (status == ENET_SOK)
    {
        status = IcssgUtils_checkPortMode(hIcssg, &portLinkCfg->mii);
        ENETTRACE_ERR_IF((status != ENET_SOK),
                         "%s: failed to check MAC port mode: %d\r\n",
                         ENET_PER_NAME(hIcssg), status);
    }

    if (status == ENET_SOK)
    {
        /* Convert MII and link configuration from Enet to ENETPHY types */
        phyMii = EnetUtils_macToPhyMii(&portLinkCfg->mii);
        phyLinkCfg.speed = (EnetPhy_Speed)portLinkCfg->linkCfg.speed;
        phyLinkCfg.duplexity = (EnetPhy_Duplexity)portLinkCfg->linkCfg.duplexity;

        status = Icssg_openEnetPhy(hIcssg, macPort, phyCfg, phyMii, &phyLinkCfg);
        ENETTRACE_ERR_IF((status != ENET_SOK),
                         "%s: Port %u: failed to open port link with PHY: %d\r\n",
                         ENET_PER_NAME(hIcssg), ENET_MACPORT_ID(portLinkCfg->macPort), status);
    }
    return status;
}

static int32_t Icssg_handleLinkUp(Icssg_Handle hIcssg,
                                  Enet_MacPort macPort)
{
    EnetPer_Handle hPer = (EnetPer_Handle)hIcssg;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    uint32_t portId = ENET_MACPORT_ID(macPort);
    EnetPhy_Handle hPhy;
    EnetPhy_LinkCfg phyLinkCfg;
    int32_t status;

    ENETTRACE_VAR(hPer);
    ENETTRACE_VAR(portId);

    if (portNum < ENET_ARRAYSIZE(hIcssg->hPhy))
    {
        hPhy = hIcssg->hPhy[portNum];

        /* Get link parameters (speed/duplexity) from PHY state machine */
        status = EnetPhy_getLinkCfg(hPhy, &phyLinkCfg);
        ENETTRACE_ERR_IF((status != ENET_SOK),
                         "%s: Port %u: failed to get PHY link config: %d\r\n",
                         ENET_PER_NAME(hIcssg), portId, status);

        if (status == ENET_SOK)
        {
            switch (phyLinkCfg.speed)
            {
                case ENETPHY_SPEED_10MBIT:
                    Icssg_updateLinkSpeed10MB(hIcssg, macPort, phyLinkCfg.duplexity);
                    break;

                case ENETPHY_SPEED_100MBIT:
                    Icssg_updateLinkSpeed100MB(hIcssg, macPort, phyLinkCfg.duplexity);
                    break;

                case ENETPHY_SPEED_1GBIT:
                    Icssg_updateLinkSpeed1G(hIcssg, macPort);
                    break;

                default:
                    Enet_assert(false, "%s: Port %u: invalid link speed %u\r\n",
                                ENET_PER_NAME(hIcssg), portId, phyLinkCfg.speed);
                    break;
            }

            ENETTRACE_INFO("%s: Port %u: Link up: %s %s\r\n",
                           ENET_PER_NAME(hIcssg), portId,
                           Icssg_gSpeedNames[phyLinkCfg.speed],
                           Icssg_gDuplexNames[phyLinkCfg.duplexity]);
        }
    }
    else
    {
        ENETTRACE_ERR("%s: invalid MAC port %u\r\n", ENET_PER_NAME(hIcssg), portId);
        status = ENET_EINVALIDPARAMS;
    }

    return status;
}

static int32_t Icssg_handleLinkDown(Icssg_Handle hIcssg,
                                    Enet_MacPort macPort)
{
    uint32_t portId = ENET_MACPORT_ID(macPort);
    int32_t status = ENET_SOK;

    ENETTRACE_VAR(portId);
    ENETTRACE_INFO("%s: Port %d: Link down\r\n", ENET_PER_NAME(hIcssg), portId);
    Icssg_updateLinkDown(hIcssg, macPort);

    return status;
}

static int32_t Icssg_configHostPortDfltVlanId(Icssg_Handle hIcssg,
                                              const EnetPort_VlanCfg *vlanCfg)
{
    uintptr_t sharedRam = Icssg_getSharedRamAddr(hIcssg);
    uint32_t regVal=0;
    uint8_t tempByte1 = 0;
    uint8_t tempByte2 = 0;
    int32_t status = ENET_SOK;

    if ((vlanCfg->portVID >= (ICSSG_VLAN_TBL_MAX_ENTRIES-1)) ||
        (vlanCfg->portVID == 0))
    {
        ENETTRACE_ERR("%s: invalid host port default VLAN id %u\r\n",
                      ENET_PER_NAME(hIcssg), vlanCfg->portVID);
        status = ENET_EINVALIDPARAMS;
    }
    else
    {
        /*Populate first byte which contains 3 bits of PCP + 1 bit of DEI and 4 bits of VLAN*/
        tempByte1 = (uint8_t)(vlanCfg->portPri << 5) | (uint8_t)((uint16_t)vlanCfg->portVID >> 8);
        /*Populate second byte which contains 8 bits of VLAN*/
        tempByte2 = (uint8_t)((uint16_t)vlanCfg->portVID & 0xFF);  /*Lower 8 bits*/

        /* Write  default VLAN to memory. This is used directly to update MMR for inserting VLAN tag . DEI is 0*/
        /*OR the values*/
        regVal = (uint32_t)(0x0081) | ((uint32_t)tempByte1 << 16) | ((uint32_t)tempByte2 << 24);

        Icssg_wr32(hIcssg, sharedRam + EMAC_ICSSG_SWITCH_PORT0_DEFAULT_VLAN_OFFSET, regVal);
    }
    return status;
}

static int32_t Icssg_configMacPortDfltVlanId(Icssg_Handle hIcssg,
                                             Enet_MacPort macPort,
                                             const EnetPort_VlanCfg *vlanCfg)
{
    uintptr_t dram = Icssg_getDramAddr(hIcssg, macPort);
    uintptr_t cfgRegs = Icssg_getCfgAddr(hIcssg);
    uintptr_t dfltVlanAddr = Icssg_getDfltVlanAddr(hIcssg, macPort);
    uint32_t regVal=0;
    uint8_t tempByte1 = 0;
    uint8_t tempByte2 = 0;
    uint32_t slice = IcssgUtils_getSliceNum(hIcssg, macPort);
    int32_t status = ENET_SOK;

    if ((vlanCfg->portVID >= (ICSSG_VLAN_TBL_MAX_ENTRIES-1)) ||
        (vlanCfg->portVID == 0))
    {
        ENETTRACE_ERR("%s: invalid MAC port default vlan id %u\r\n",
                      ENET_PER_NAME(hIcssg), vlanCfg->portVID);
        status = ENET_EINVALIDPARAMS;
    }
    else
    {
        /* Program Port VLAN in HW MMR.
         * 'slice' here is used to differentiate MAC port 1 and MAC port 2 */
        Icssg_wr16(hIcssg, cfgRegs +
                   CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FDB_DF_VLAN + (2 * slice),
                   vlanCfg->portVID);

        /*Populate first byte which contains 3 bits of PCP + 1 bit of DEI and 4 bits of VLAN*/
        tempByte1 = (uint8_t)(vlanCfg->portPri << 5) | (uint8_t)((uint16_t)vlanCfg->portVID >> 8);
        /*Populate second byte which contains 8 bits of VLAN*/
        tempByte2 = (uint8_t)((uint16_t)vlanCfg->portVID & 0xFF);  /*Lower 8 bits*/

        /* Write  default VLAN to memory. This is used directly to update MMR for inserting VLAN tag . DEI is 0*/
        /*OR the values*/
        regVal = (uint32_t)(0x0081) | ((uint32_t)tempByte1 << 16) | ((uint32_t)tempByte2 << 24);

        Icssg_wr32(hIcssg, dfltVlanAddr, regVal);

        /* Update Default Queue number for untagged packet*/
        tempByte1 = Icssg_rd8(hIcssg, dram + PORT_Q_PRIORITY_MAPPING_OFFSET + vlanCfg->portPri);
        Icssg_wr8(hIcssg, dram + QUEUE_NUM_UNTAGGED, tempByte1);
    }

    return status;
}

static int32_t Icssg_validateFlowId(Icssg_Handle hIcssg,
                                    Enet_MacPort macPort,
                                    uint32_t coreKey,
                                    uint32_t startIdx,
                                    uint32_t flowIdx)
{
    uintptr_t dram = Icssg_getDramAddr(hIcssg, macPort);
    int32_t status = ENET_SOK;
    uint32_t flowIdOffset;

    flowIdOffset = Icssg_rd32(hIcssg, dram + PSI_L_REGULAR_FLOW_ID_BASE_OFFSET);
    if (startIdx != flowIdOffset)
    {
        status = ENET_EINVALIDPARAMS;
    }

    if (status == ENET_SOK)
    {
        Enet_IoctlPrms prms;
        EnetRm_ValidateRxFlowInArgs rmInArgs;

        rmInArgs.chIdx   = Icssg_getChIdx(hIcssg, macPort);
        rmInArgs.coreKey = coreKey;
        rmInArgs.flowIdx = flowIdx;

        ENET_IOCTL_SET_IN_ARGS(&prms, &rmInArgs);
        ENET_RM_PRIV_IOCTL(hIcssg->hRm, ENET_RM_IOCTL_VALIDATE_RX_FLOW, &prms, status);
    }

    return status;
}

static void Icssg_UCastFloodingCtrl(Icssg_Handle hIcssg,
                                    Enet_MacPort macPort,
                                    bool enable)
{
    IcssgUtils_ioctlR30Cmd cmd;
    bool cmdDone;
    int32_t status;

    cmd = enable ? ICSSG_UTILS_R30_CMD_UC_FLOODING_ENABLE :
                   ICSSG_UTILS_R30_CMD_UC_FLOODING_DISABLE;

    /* Run the R30 async command */
    status = Icssg_R30SendAsyncIoctl(hIcssg,
                                     macPort,
                                     cmd,
                                     &hIcssg->asyncIoctlSeqNum,
                                     &hIcssg->asyncIoctlType);

    /* Now wait for the response from FW, FW should respond immediately */
    if (status == ENET_SINPROGRESS)
    {
        cmdDone = IcssgUtils_isR30CmdDone(hIcssg, macPort);
        while (!cmdDone)
        {
            EnetUtils_delayTicks(100);
            cmdDone = IcssgUtils_isR30CmdDone(hIcssg, macPort);
        }
    }
}

static void Icssg_MCastFloodingCtrl(Icssg_Handle hIcssg,
                                    Enet_MacPort macPort,
                                    bool enable)
{
    IcssgUtils_ioctlR30Cmd cmd;
    bool cmdDone;
    int32_t status;

    cmd = enable ? ICSSG_UTILS_R30_CMD_MC_FLOODING_ENABLE :
                   ICSSG_UTILS_R30_CMD_MC_FLOODING_DISABLE;

    /* Run the R30 async command */
    status = Icssg_R30SendAsyncIoctl(hIcssg,
                                     macPort,
                                     cmd,
                                     &hIcssg->asyncIoctlSeqNum,
                                     &hIcssg->asyncIoctlType);

    /* Now wait for the response from FW, FW should respond immediately */
    if (status == ENET_SINPROGRESS)
    {
        cmdDone = IcssgUtils_isR30CmdDone(hIcssg, macPort);
        while (!cmdDone)
        {
            EnetUtils_delayTicks(100);
            cmdDone = IcssgUtils_isR30CmdDone(hIcssg, macPort);
        }
    }
}

static bool Icssg_isVlanAware(Icssg_Handle hIcssg)
{
    uintptr_t cfgRegs = Icssg_getCfgAddr(hIcssg);
    uint32_t val;

    val = CSL_REG32_FEXT(cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FDB_GEN_CFG2,
                         ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FDB_GEN_CFG2_FDB_VLAN_EN);

    return (val != 0U);
}

static int32_t Icssg_setVlanAwareMode(Icssg_Handle hIcssg)
{
    EnetPer_Handle hPer = (EnetPer_Handle)hIcssg;
    uintptr_t cfgRegs = Icssg_getCfgAddr(hIcssg);
    uintptr_t vlanTableAddr;
    uint32_t val;
    uint32_t i;
    int32_t status = ENET_SOK;

    vlanTableAddr = Icssg_getVlanTableAddr(hIcssg);

    /* Program VLAN TABLE address in MMR */
    CSL_REG32_FINS(cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FDB_GEN_CFG1,
                   ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FDB_GEN_CFG1_SMEM_VLAN_OFFSET,
                   vlanTableAddr);

    /* Enable VLAN FDB in MMR */
    val = CSL_REG32_RD(cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FDB_GEN_CFG2);

    val |= (1U << CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FDB_GEN_CFG2_FDB_PRU0_EN_SHIFT) |
           (1U << CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FDB_GEN_CFG2_FDB_PRU1_EN_SHIFT) |
           (1U << CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FDB_GEN_CFG2_FDB_HOST_EN_SHIFT) |
           (1U << CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FDB_GEN_CFG2_FDB_VLAN_EN_SHIFT);

    CSL_REG32_WR(cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FDB_GEN_CFG2, val);

    /* Initialize VLAN-FID table to 0, do two entries at a time (4 byte) */
    for (i = 0U; i < 2048U; i++)
    {
        CSL_REG32_WR(vlanTableAddr + (i * 4U), 0U);
    }

    /* Send VLAN_AWARE_ENABLE cmd for MAC port 1 */
    status = Icssg_R30SendSyncIoctl(hIcssg,
                                    ENET_MAC_PORT_1,
                                    ICSSG_UTILS_R30_CMD_VLAN_AWARE_ENABLE);
    ENETTRACE_ERR_IF((status != ENET_SOK),
                     "%s: Port 1: failed to send VLAN_AWARE_ENABLE R30 cmd: %d\r\n",
                     ENET_PER_NAME(hIcssg), status);

    /* Send VLAN_AWARE_ENABLE cmd for MAC port 2 only in Switch mode */
    if ((status == ENET_SOK) &&
        (hPer->enetType == ENET_ICSSG_SWITCH))
    {
        status = Icssg_R30SendSyncIoctl(hIcssg,
                                        ENET_MAC_PORT_2,
                                        ICSSG_UTILS_R30_CMD_VLAN_AWARE_ENABLE);
        ENETTRACE_ERR_IF((status != ENET_SOK),
                         "%s: Port 2: failed to send VLAN_AWARE_ENABLE R30 cmd: %d\r\n",
                         ENET_PER_NAME(hIcssg), status);
    }

    return status;
}

static int32_t Icssg_setVlanUnawareMode(Icssg_Handle hIcssg)
{
    EnetPer_Handle hPer = (EnetPer_Handle)hIcssg;
    uintptr_t cfgRegs = Icssg_getCfgAddr(hIcssg);
    int32_t status;

    /* Send VLAN_AWARE_DISABLE cmd for MAC port 1 */
    status = Icssg_R30SendSyncIoctl(hIcssg,
                                    ENET_MAC_PORT_1,
                                    ICSSG_UTILS_R30_CMD_VLAN_AWARE_DISABLE);
    ENETTRACE_ERR_IF((status != ENET_SOK),
                     "%s: Port 1: failed to send VLAN_AWARE_DISABLE R30 cmd: %d\r\n",
                     ENET_PER_NAME(hIcssg), status);

    /* Send VLAN_AWARE_DISABLE cmd for MAC port 2 only in Switch mode */
    if ((status == ENET_SOK) &&
        (hPer->enetType == ENET_ICSSG_SWITCH))
    {
        status = Icssg_R30SendSyncIoctl(hIcssg,
                                        ENET_MAC_PORT_2,
                                        ICSSG_UTILS_R30_CMD_VLAN_AWARE_DISABLE);
        ENETTRACE_ERR_IF((status != ENET_SOK),
                         "%s: Port 2: failed to send VLAN_AWARE_DISABLE R30 cmd: %d\r\n",
                         ENET_PER_NAME(hIcssg), status);
    }

    /* Disable VLAN FDB in MMR */
    if (status == ENET_SOK)
    {
        CSL_REG32_FINS(cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FDB_GEN_CFG2,
                       ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FDB_GEN_CFG2_FDB_VLAN_EN,
                       0U);
    }

    return status;
}

static int32_t Icssg_setDscpEnable(Icssg_Handle hIcssg, Enet_MacPort macPort)
{
    int32_t status = ENET_SOK;

    /* Send DSCP Enable cmd for MAC port */
    status = Icssg_R30SendSyncIoctl(hIcssg,
                                    macPort,
                                    ICSSG_UTILS_R30_CMD_DSCP_ENABLE);
    ENETTRACE_ERR_IF((status != ENET_SOK),
                     "%s: Port %d: failed to send DSCP Enable R30 cmd: %d\r\n",
                     ENET_PER_NAME(hIcssg), macPort, status);
    return status;
}

static int32_t Icssg_setDscpDisable(Icssg_Handle hIcssg, Enet_MacPort macPort)
{
    int32_t status;

    /* Send VLAN_AWARE_DISABLE cmd for MAC port */
    status = Icssg_R30SendSyncIoctl(hIcssg,
                                    macPort,
                                    ICSSG_UTILS_R30_CMD_DSCP_DISABLE);
    ENETTRACE_ERR_IF((status != ENET_SOK),
                     "%s: Port %d: failed to send DSCP Disable R30 cmd: %d\r\n",
                     ENET_PER_NAME(hIcssg), macPort, status);
    return status;
}

static int32_t Icssg_setAcceptableFrameCheck(Icssg_Handle hIcssg,
                                             Enet_MacPort macPort,
                                             Icssg_AcceptFrameCheck acceptFrameCheck)
{
    IcssgUtils_ioctlR30Cmd cmd;
    int32_t status = ENET_SOK;

    switch (acceptFrameCheck)
    {
        case ICSSG_ACCEPT_ONLY_VLAN_TAGGED:
            cmd = ICSSG_UTILS_R30_CMD_ACCEPT_TAGGED;
            break;

        case ICSSG_ACCEPT_ONLY_UNTAGGED_PRIO_TAGGED:
            cmd = ICSSG_UTILS_R30_CMD_ACCEPT_UNTAGGED_N_PRIO;
            break;

        case ICSSG_ACCEPT_ALL:
            cmd = ICSSG_UTILS_R30_CMD_ACCEPT_ALL;
            break;

        default:
            status = ENET_EINVALIDPARAMS;
            break;
    }

    /* Run the R30 async command */
    if (status == ENET_SOK)
    {
        status = Icssg_R30SendAsyncIoctl(hIcssg,
                                         macPort,
                                         cmd,
                                         &hIcssg->asyncIoctlSeqNum,
                                         &hIcssg->asyncIoctlType);
    }

    return status;
}

static int32_t Icssg_setAcceptableFrameCheckSync(Icssg_Handle hIcssg,
                                                 Enet_MacPort macPort,
                                                 Icssg_AcceptFrameCheck acceptFrameCheck)
{
    bool cmdDone;
    int32_t status;

    /* Issue acceptable frame check command */
    status = Icssg_setAcceptableFrameCheck(hIcssg, macPort, acceptFrameCheck);

    /* Now wait for the response from FW, FW should respond immediately */
    if (status == ENET_SINPROGRESS)
    {
        cmdDone = IcssgUtils_isR30CmdDone(hIcssg, macPort);
        while (!cmdDone)
        {
            EnetUtils_delayTicks(100);
            cmdDone = IcssgUtils_isR30CmdDone(hIcssg, macPort);
        }

        status = ENET_SOK;
    }

    return status;
}

static int32_t Icssg_setPriorityMapping(Icssg_Handle hIcssg,
                                        Enet_MacPort macPort,
                                        EnetPort_PriorityMap *priMap)
{
    uintptr_t dram = Icssg_getDramAddr(hIcssg, macPort);
    uintptr_t dfltVlanAddr = Icssg_getDfltVlanAddr(hIcssg, macPort);
    uint8_t untaggedQueueNum;
    uint8_t tempVal;
    uint32_t i;

    for (i = 0U; i < ENET_PRI_NUM; i++)
    {
        tempVal = (uint8_t)priMap->priorityMap[i];
        Icssg_wr8(hIcssg, dram + PORT_Q_PRIORITY_MAPPING_OFFSET + i, tempVal);
    }

    /* Update default queue number for untagged packet */
    tempVal = Icssg_rd8(hIcssg, dfltVlanAddr + 2); /* Read the configured PCP value */
    tempVal = tempVal >> 5;                        /* Shift to get the value in correct format */

    untaggedQueueNum = priMap->priorityMap[tempVal];
    Icssg_wr8(hIcssg, dram + QUEUE_NUM_UNTAGGED, untaggedQueueNum);

    return ENET_SOK;
}

static int32_t Icssg_setDscpPriority(Icssg_Handle hIcssg,
                                      Enet_MacPort macPort,
                                      EnetPort_DscpPriorityMap *dscpPriority)
{
    /* One-to-one mapping from PCP -> Traffic Class.
     * Managed using FT3[0:7] and Classifier[0:7]. */
    uintptr_t dram = Icssg_getDramAddr(hIcssg, macPort);
    uintptr_t cfgRegs = Icssg_getCfgAddr(hIcssg);
    uint32_t ft3Type = 0U;
    uint32_t classSelect = 0U;
    uint32_t orEnable[ENET_PRI_NUM] = { 0U, };
    uint32_t andEnable[ENET_PRI_NUM] = { 0U, };
    uint16_t orNvEnable[ENET_PRI_NUM] = { 0U, };
    uint16_t andNvEnable[ENET_PRI_NUM] = { 0U, };
    uint32_t gateConfig = 0x50;
    uint32_t tempReg = 0U;
    Icssg_Filter3Cfg ft3CfgPcp = {0xc, 0, 0, 0, 0, 1, 0, 0x03ff0000, 0, 0, 0xffffffff, 0xffffffff};
    uint32_t slice = IcssgUtils_getSliceNum(hIcssg, macPort);
    uint32_t tempVal, loopCnt = 1U;
    uint8_t pcp;
    uint32_t i;
    int32_t status = ENET_SOK;

    for (i = 0U; i < ENET_TOS_PRI_NUM; i++)
    {
        if (dscpPriority->tosMap[i] > ENET_PRI_MAX)
        {
            ENETTRACE_ERR("Invalid priority map %u -> %u\n", i, dscpPriority->tosMap[i]);
            status = ENET_EINVALIDPARAMS;
            break;
        }
    }

    if (status == ENET_SOK)
    {
        /* Set up filter type 3's to match pcp bits */
        /* First 8 non zero indexes are mapped to 8 pcp values*/
        for (pcp = 0U; pcp < ENET_PRI_NUM; pcp++)
        {
            if(pcp)
            {
                do
                {
                    tempVal =  dscpPriority->tosMap[loopCnt];
                    loopCnt++;
                }while(tempVal == 0);
                /* Setup FT3[1:7] to detect PCP1 - PCP7 */
                ft3Type = (uint32_t)((((uint32_t)(loopCnt - 1)) << 26U) | 0x00000008U);
                ft3CfgPcp.ft3Type = ft3Type;
                IcssgUtils_configFilter3(hIcssg, macPort, tempVal, &ft3CfgPcp);
                Icssg_wr8(hIcssg, dram + DSCP_BASED_PRI_MAP_INDEX_OFFSET + tempVal, (loopCnt - 1));
            }
            else
            {
                /* pcp = 0 means best effort and last priority*/
                ft3Type = (uint32_t)((((uint32_t)(0)) << 26U) | 0x00000008U);
                ft3CfgPcp.ft3Type = ft3Type;
                IcssgUtils_configFilter3(hIcssg, macPort, pcp, &ft3CfgPcp);
                Icssg_wr8(hIcssg, dram + DSCP_BASED_PRI_MAP_INDEX_OFFSET, pcp);
            }
        }

        /* Build up the or lists */
        for (pcp = 0U; pcp < ENET_PRI_NUM; pcp++)
        {
            classSelect = pcp;
            orEnable[classSelect] |= (1U << pcp);
        }

        if (ICSSG_IS_SLICE_1(slice))
        {
            cfgRegs += (CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS0_OR_EN_PRU1
                        - CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS0_OR_EN_PRU0);
        }

        /* Now program classifier c */
        for (pcp = 0U; pcp < ENET_PRI_NUM; pcp++)
        {
            /* Configure OR Enable*/
            Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS0_OR_EN_PRU0 + (8U * pcp), orEnable[pcp]);

            /* Configure AND Enable */
            Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS0_AND_EN_PRU0 + (8U * pcp), andEnable[pcp]);
            tempReg = Icssg_rd32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS_CFG1_PRU0);
            tempReg &= ~(0x3U << (pcp * 2U));
            Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS_CFG1_PRU0, tempReg);

            /* Configure NV Enable bits (1 bit in upper16, 1bit in lower16 */
            tempReg = Icssg_rd32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS_CFG2_PRU0);
            if (orNvEnable[pcp])
            {
                tempReg |= 1U << (pcp + 16U);
            }
            else
            {
                tempReg &= ~(1U << (pcp + 16U));
            }

            if (andNvEnable[pcp])
            {
                tempReg |= 1U << (pcp);
            }
            else
            {
                tempReg &= ~(1U << (pcp));
            }

            Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS_CFG2_PRU0, tempReg);
            /* Configure class gate */
            Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS_GATES0_PRU0 + (4U * pcp), gateConfig);
        }

        for (i = 0; i < ENET_PRI_NUM; i++)
        {
            tempVal = (uint32_t)i;

            /* Shift PCP value by 5 so HW can save a cycle */
            tempVal = tempVal << 5;
            Icssg_wr8(hIcssg, dram + PORT_Q_PRIORITY_REGEN_OFFSET + i, tempVal);
        }
    }
    return status;
}

static int32_t Icssg_setPriorityRegen(Icssg_Handle hIcssg,
                                      Enet_MacPort macPort,
                                      EnetPort_PriorityMap *priMap)
{
    /* One-to-one mapping from PCP -> Traffic Class.
     * Managed using FT3[0:7] and Classifier[0:7]. */
    uintptr_t dram = Icssg_getDramAddr(hIcssg, macPort);
    uintptr_t cfgRegs = Icssg_getCfgAddr(hIcssg);
    uint32_t ft3Type = 0U;
    uint32_t classSelect = 0U;
    uint32_t orEnable[ENET_PRI_NUM] = { 0U, };
    uint32_t andEnable[ENET_PRI_NUM] = { 0U, };
    uint16_t orNvEnable[ENET_PRI_NUM] = { 0U, };
    uint16_t andNvEnable[ENET_PRI_NUM] = { 0U, };
    uint32_t gateConfig = 0x50;
    uint32_t tempReg = 0U;
    Icssg_Filter3Cfg ft3CfgPcp = {0xc, 0, 0, 0, 0, 5, 0, 0xff1f0000, 0, 0, 0xffffffff, 0xffffffff};
    uint32_t slice = IcssgUtils_getSliceNum(hIcssg, macPort);
    uint32_t tempVal;
    uint8_t pcp;
    uint8_t i;

    /* Set up filter type 3's to match pcp bits */
    for (pcp = 0U; pcp < ENET_PRI_NUM; pcp++)
    {
        /* Setup FT3[0:7] to detect PCP0 - PCP7 */
        ft3Type = (uint32_t)((((uint32_t)pcp) << 21U) | 0x00000081U);
        ft3CfgPcp.ft3Type = ft3Type;
        IcssgUtils_configFilter3(hIcssg, macPort, pcp, &ft3CfgPcp);
    }

    /* Build up the or lists */
    for (pcp = 0U; pcp < ENET_PRI_NUM; pcp++)
    {
        classSelect = priMap->priorityMap[pcp];
        orEnable[classSelect] |= (1U << pcp);
    }

    if (ICSSG_IS_SLICE_1(slice))
    {
        cfgRegs += (CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS0_OR_EN_PRU1
                     - CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS0_OR_EN_PRU0);
    }

    /* Now program classifier c */
    for (pcp = 0U; pcp < ENET_PRI_NUM; pcp++)
    {
        /* Configure OR Enable*/
        Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS0_OR_EN_PRU0 + (8U * pcp), orEnable[pcp]);

        /* Configure AND Enable */
        Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS0_AND_EN_PRU0 + (8U * pcp), andEnable[pcp]);
        tempReg = Icssg_rd32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS_CFG1_PRU0);
        tempReg &= ~(0x3U << (pcp * 2U));
        Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS_CFG1_PRU0, tempReg);

        /* Configure NV Enable bits (1 bit in upper16, 1bit in lower16 */
        tempReg = Icssg_rd32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS_CFG2_PRU0);
        if (orNvEnable[pcp])
        {
            tempReg |= 1U << (pcp + 16U);
        }
        else
        {
            tempReg &= ~(1U << (pcp + 16U));
        }

        if (andNvEnable[pcp])
        {
            tempReg |= 1U << (pcp);
        }
        else
        {
            tempReg &= ~(1U << (pcp));
        }

        Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS_CFG2_PRU0, tempReg);
        /* Configure class gate */
        Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS_GATES0_PRU0 + (4U * pcp), gateConfig);
    }

    for (i = 0; i < ENET_PRI_NUM; i++)
    {
        tempVal = (uint32_t)priMap->priorityMap[i];

        /* Shift PCP value by 5 so HW can save a cycle */
        tempVal = tempVal << 5;
        Icssg_wr8(hIcssg, dram + PORT_Q_PRIORITY_REGEN_OFFSET + i, tempVal);
    }

    return ENET_SOK;
}

static int32_t Icssg_setIngressRateLim(Icssg_Handle hIcssg,
                                       Enet_MacPort macPort,
                                       Icssg_IngressRateLim *rateLimCfg)
{
    uintptr_t cfgRegs = Icssg_getCfgAddr(hIcssg);
    uintptr_t offs;
    uint32_t slice = IcssgUtils_getSliceNum(hIcssg, macPort);
    uint32_t val;
    uint32_t mask = ~0xFF;
    uint32_t shift;
    uint32_t i;
    int32_t status = ENET_SOK;

    offs = ICSSG_IS_SLICE_0(slice) ? CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_RATE_CFG0_PRU0 :
                                     CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_RATE_CFG0_PRU1;
    if (rateLimCfg->rateIndex < 8U)
    {
        offs += (rateLimCfg->rateIndex << 2U);
    }

    /* Program rate in Mbps */
    Icssg_wr32(hIcssg, cfgRegs + offs, (rateLimCfg->rateLimit * 32768U) / 250U);
    offs = ICSSG_IS_SLICE_0(slice) ? CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_RATE_SRC_SEL0_PRU0 :
                                     CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_RATE_SRC_SEL0_PRU1;
    if (rateLimCfg->rateIndex < 8U)
    {
        shift = rateLimCfg->rateIndex << 3U;
        if (rateLimCfg->rateIndex < 4U)
        {
            shift = rateLimCfg->rateIndex << 3U;
            mask = ~(0xFF << shift);
        }
        else if ((rateLimCfg->rateIndex >= 4U) && (rateLimCfg->rateIndex <= 7U))
        {
            shift = (rateLimCfg->rateIndex - 4U) << 3U;
            offs += 4U; //CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_RATE_SRC_SEL1_PRU0/1
            mask = ~(0xFF << shift);
        }
        val = Icssg_rd32(hIcssg, cfgRegs + offs);
        val &= mask;
        val |= (rateLimCfg->rateSrcSel << shift);
        Icssg_wr32(hIcssg, cfgRegs + offs, val);
    }

    if (rateLimCfg->classIndex < 16U)
    {
        offs = ICSSG_IS_SLICE_0(slice) ? CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS_GATES0_PRU0 :
                                         CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS_GATES0_PRU1;
        offs += (rateLimCfg->classIndex << 2U);
        Icssg_wr32(hIcssg, cfgRegs + offs, (0x10 | rateLimCfg->rateIndex));

        offs = ICSSG_IS_SLICE_0(slice) ? CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS0_AND_EN_PRU0 :
                                         CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS0_AND_EN_PRU1;
        offs += (rateLimCfg->classIndex << 3U);
        Icssg_wr32(hIcssg, cfgRegs + offs, rateLimCfg->classDataAndTerm);

        offs = ICSSG_IS_SLICE_0(slice) ? CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS0_OR_EN_PRU0 :
                                         CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS0_OR_EN_PRU1;
        offs += (rateLimCfg->classIndex << 3U);
        Icssg_wr32(hIcssg, cfgRegs + offs, rateLimCfg->classDataOrTerm);

        shift = rateLimCfg->classIndex << 1U;
        mask = ~(0x3 << shift);

        offs = ICSSG_IS_SLICE_0(slice) ? CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS_CFG1_PRU0 :
                                         CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS_CFG1_PRU1;

        val = Icssg_rd32(hIcssg, cfgRegs + offs);
        val &= mask;

        val |= (rateLimCfg->classSel << shift);
        Icssg_wr32(hIcssg, cfgRegs + offs, val);

        offs = ICSSG_IS_SLICE_0(slice) ? CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS_CFG2_PRU0 :
                                         CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS_CFG2_PRU1;

        val = Icssg_rd32(hIcssg, cfgRegs + offs);
        if (rateLimCfg->notMask & 1U) //AND nv enable
        {
            val |= (1U << rateLimCfg->classIndex);
        }
        if (rateLimCfg->notMask & 2U) //OR nv enable
        {
            val |= (1U << (rateLimCfg->classIndex+16));
        }
        Icssg_wr32(hIcssg, cfgRegs + offs, val);
    }

    for (i = 0U; i < 2U; i++)
    {
        if (((rateLimCfg->filter[i].type == 0U) && (rateLimCfg->filter[i].index >= 8U)) ||
            ((rateLimCfg->filter[i].type == 1U) && (rateLimCfg->filter[i].index >= 16U)))
        {
            break;
        }

        if ((rateLimCfg->filter[i].type == 0U) && (rateLimCfg->filter[i].index < 8U))
        {
            offs = ICSSG_IS_SLICE_0(slice) ? CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT1_START_LEN_PRU0 :
                                             CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT1_START_LEN_PRU1;
            val = Icssg_rd32(hIcssg, cfgRegs + offs);
            if (val != (rateLimCfg->filter[i].ft1Start | (rateLimCfg->filter[i].ft1Len << 16U)))
            {
                status = ENET_EINVALIDPARAMS;
            }
            offs = ICSSG_IS_SLICE_0(slice) ? CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT1_CFG_PRU0 :
                                             CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT1_CFG_PRU1;
            val = Icssg_rd16(hIcssg, cfgRegs + offs);
            if (val != rateLimCfg->filter[i].ft1Cfg)
            {
                status = ENET_EINVALIDPARAMS;
            }
            offs = ICSSG_IS_SLICE_0(slice) ? CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT1_0_DA0_PRU0 :
                                             CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT1_0_DA0_PRU1;
            offs += (rateLimCfg->filter[i].index << 4U);
            Icssg_wr32(hIcssg, cfgRegs + offs, rateLimCfg->filter[i].u.ft1.destAddrLow);
            offs = ICSSG_IS_SLICE_0(slice) ? CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT1_0_DA1_PRU0 :
                                             CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT1_0_DA1_PRU1;
            offs += (rateLimCfg->filter[i].index << 4U);
            Icssg_wr32(hIcssg, cfgRegs + offs, rateLimCfg->filter[i].u.ft1.destAddrHigh);
            offs = ICSSG_IS_SLICE_0(slice) ? CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT1_0_DA_MASK0_PRU0 :
                                             CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT1_0_DA_MASK0_PRU1;
            offs += (rateLimCfg->filter[i].index << 4U);
            Icssg_wr32(hIcssg, cfgRegs + offs, rateLimCfg->filter[i].u.ft1.destAddrMaskLow);
            offs = ICSSG_IS_SLICE_0(slice) ? CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT1_0_DA_MASK1_PRU0 :
                                             CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT1_0_DA_MASK1_PRU1;
            offs += (rateLimCfg->filter[i].index << 4U);
            Icssg_wr32(hIcssg, cfgRegs + offs, rateLimCfg->filter[i].u.ft1.destAddrMaskHigh);

        }
        else if ((rateLimCfg->filter[i].type == 1U) && (rateLimCfg->filter[i].index < 16U))
        {
            offs = ICSSG_IS_SLICE_0(slice) ? CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT3_0_START_PRU0 :
                                             CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT3_0_START_PRU1;
            offs += (rateLimCfg->filter[i].index << 5U);
            Icssg_wr32(hIcssg, cfgRegs + offs, rateLimCfg->filter[i].u.ft3.start);
            offs = ICSSG_IS_SLICE_0(slice) ? CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT3_0_START_AUTO_PRU0 :
                                             CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT3_0_START_AUTO_PRU1;
            offs += (rateLimCfg->filter[i].index << 5U);
            Icssg_wr32(hIcssg, cfgRegs + offs, rateLimCfg->filter[i].u.ft3.startAuto);
            offs = ICSSG_IS_SLICE_0(slice) ? CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT3_0_START_LEN_PRU0 :
                                             CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT3_0_START_LEN_PRU1;
            offs += (rateLimCfg->filter[i].index << 5U);
            Icssg_wr32(hIcssg, cfgRegs + offs, rateLimCfg->filter[i].u.ft3.startLen);
            offs = ICSSG_IS_SLICE_0(slice) ? CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT3_0_JMP_OFFSET_PRU0 :
                                             CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT3_0_JMP_OFFSET_PRU1;
            offs += (rateLimCfg->filter[i].index << 5U);
            Icssg_wr32(hIcssg, cfgRegs + offs, rateLimCfg->filter[i].u.ft3.jmpOffset);
            offs = ICSSG_IS_SLICE_0(slice) ? CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT3_0_LEN_PRU0 :
                                             CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT3_0_LEN_PRU1;
            offs += (rateLimCfg->filter[i].index << 5U);
            Icssg_wr32(hIcssg, cfgRegs + offs, rateLimCfg->filter[i].u.ft3.len);
            offs = ICSSG_IS_SLICE_0(slice) ? CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT3_0_CFG_PRU0 :
                                             CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT3_0_CFG_PRU1;
            offs += (rateLimCfg->filter[i].index << 5U);
            Icssg_wr32(hIcssg, cfgRegs + offs, rateLimCfg->filter[i].u.ft3.config);
            offs = ICSSG_IS_SLICE_0(slice) ? CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT3_0_T_PRU0 :
                                             CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT3_0_T_PRU1;
            offs += (rateLimCfg->filter[i].index << 5U);
            Icssg_wr32(hIcssg, cfgRegs + offs, rateLimCfg->filter[i].u.ft3.type);
            offs = ICSSG_IS_SLICE_0(slice) ? CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT3_0_T_MASK_PRU0 :
                                             CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT3_0_T_MASK_PRU1;
            offs += (rateLimCfg->filter[i].index << 5U);
            Icssg_wr32(hIcssg, cfgRegs + offs, rateLimCfg->filter[i].u.ft3.typeMask);

            offs = ICSSG_IS_SLICE_0(slice) ? CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT3_0_P0_PRU0 :
                                             CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT3_0_P0_PRU1;
            offs += (rateLimCfg->filter[i].index << 5U);
            Icssg_wr32(hIcssg, cfgRegs + offs, rateLimCfg->filter[i].u.ft3.patternLow);
            offs = ICSSG_IS_SLICE_0(slice) ? CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT3_0_P1_PRU0 :
                                             CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT3_0_P1_PRU1;
            offs += (rateLimCfg->filter[i].index << 5U);
            Icssg_wr32(hIcssg, cfgRegs + offs, rateLimCfg->filter[i].u.ft3.patternHigh);
            offs = ICSSG_IS_SLICE_0(slice) ? CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT3_0_P_MASK0_PRU0 :
                                             CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT3_0_P_MASK0_PRU1;
            offs += (rateLimCfg->filter[i].index << 5U);
            Icssg_wr32(hIcssg, cfgRegs + offs, rateLimCfg->filter[i].u.ft3.patternMaskLow);
            offs = ICSSG_IS_SLICE_0(slice) ? CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT3_0_P_MASK1_PRU0 :
                                             CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT3_0_P_MASK1_PRU1;
            offs += (rateLimCfg->filter[i].index << 5U);
            Icssg_wr32(hIcssg, cfgRegs + offs, rateLimCfg->filter[i].u.ft3.patternMaskHigh);
        }
    }

    return status;
}

static uintptr_t Icssg_getTxIpgCfgAddr(Icssg_Handle hIcssg,
                                       Enet_MacPort macPort,
                                       bool crossSlice)
{
    uintptr_t baseAddr = (uintptr_t)hIcssg->enetPer.virtAddr;
    uint32_t slice = IcssgUtils_getSliceNum(hIcssg, macPort);

    if (crossSlice)
    {
        slice = ICSSG_IS_SLICE_0(slice) ? 1U : 0U;
    }

    if (ICSSG_IS_SLICE_0(slice))
    {
        baseAddr += CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_REGS_BASE +
                    CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_TX_IPG0;
    }
    else
    {
        baseAddr += CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_REGS_BASE +
                    CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_TX_IPG1;
    }

    return baseAddr;
}

static void Icssg_updateRgmiiCfg10HD(Icssg_Handle hIcssg,
                                     Enet_MacPort macPort)
{
    uintptr_t rgmiiCfgAddr = Icssg_getRgmiiCfgAddr(hIcssg);
    uint32_t slice = IcssgUtils_getSliceNum(hIcssg, macPort);
    uint32_t val;

    val = Icssg_rd32(hIcssg, rgmiiCfgAddr);

    if (ICSSG_IS_SLICE_0(slice))
    {
        val &= ~CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG_RGMII0_GIG_IN_MASK;
        val |= CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG_RGMII0_INBAND_MASK;
        val &= ~CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG_RGMII0_FULLDUPLEX_IN_MASK;
    }
    else
    {
        val &= ~CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG_RGMII1_GIG_IN_MASK;
        val |= CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG_RGMII1_INBAND_MASK;
        val &= ~CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG_RGMII1_FULLDUPLEX_IN_MASK;
    }

    Icssg_wr32(hIcssg, rgmiiCfgAddr, val);
}

static void Icssg_updateRgmiiCfg10FD(Icssg_Handle hIcssg,
                                     Enet_MacPort macPort)
{
    uintptr_t rgmiiCfgAddr = Icssg_getRgmiiCfgAddr(hIcssg);
    uint32_t slice = IcssgUtils_getSliceNum(hIcssg, macPort);
    uint32_t val;

    val = Icssg_rd32(hIcssg, rgmiiCfgAddr);

    if (ICSSG_IS_SLICE_0(slice))
    {
        val &= ~CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG_RGMII0_GIG_IN_MASK;
        val |= CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG_RGMII0_INBAND_MASK;
        val |= CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG_RGMII0_FULLDUPLEX_IN_MASK;
    }
    else
    {
        val &= ~CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG_RGMII1_GIG_IN_MASK;
        val |= CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG_RGMII1_INBAND_MASK;
        val |= CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG_RGMII1_FULLDUPLEX_IN_MASK;
    }

    Icssg_wr32(hIcssg, rgmiiCfgAddr, val);
}

static void Icssg_updateRgmiiCfg100HD(Icssg_Handle hIcssg,
                                      Enet_MacPort macPort)
{
    uintptr_t rgmiiCfgAddr = Icssg_getRgmiiCfgAddr(hIcssg);
    uint32_t slice = IcssgUtils_getSliceNum(hIcssg, macPort);
    uint32_t val;

    val = Icssg_rd32(hIcssg, rgmiiCfgAddr);

    /* Need to clear bit 17 for slice 0 or bit 21 for slice 1 of ICSSG_RGMII_CFG to configure 100 mpbs */
    /* Need to clear bit 18 for slice 0 or bit 22 for slice 1 of ICSSG_RGMII_CFG to configure half duplex */
    if (ICSSG_IS_SLICE_0(slice))
    {
        val &= ~CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG_RGMII0_GIG_IN_MASK;
        val &= ~CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG_RGMII0_FULLDUPLEX_IN_MASK;
    }
    else
    {
        val &= ~CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG_RGMII1_GIG_IN_MASK;
        val &= ~CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG_RGMII1_FULLDUPLEX_IN_MASK;
    }

    Icssg_wr32(hIcssg, rgmiiCfgAddr, val);
}

static void Icssg_updateRgmiiCfg100FD(Icssg_Handle hIcssg,
                                      Enet_MacPort macPort)
{
    uintptr_t rgmiiCfgAddr = Icssg_getRgmiiCfgAddr(hIcssg);
    uint32_t slice = IcssgUtils_getSliceNum(hIcssg, macPort);
    uint32_t val;

    val = Icssg_rd32(hIcssg, rgmiiCfgAddr);

    /* Need to clear bit 17 for slice 0 or bit 21 for slice 1 of ICSSG_RGMII_CFG to configure 100 mpbs */
    /* Need to set bit 18 for slice 0 or bit 22 for slice 1 of ICSSG_RGMII_CFG to configure full duplex */
    if (ICSSG_IS_SLICE_0(slice))
    {
        val &= ~CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG_RGMII0_GIG_IN_MASK;
        val |= CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG_RGMII0_FULLDUPLEX_IN_MASK;
    }
    else
    {
        val &= ~CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG_RGMII1_GIG_IN_MASK;
        val |= CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG_RGMII1_FULLDUPLEX_IN_MASK;

    }

    Icssg_wr32(hIcssg, rgmiiCfgAddr, val);
}

static void Icssg_updateRgmiiCfg1G(Icssg_Handle hIcssg,
                                   Enet_MacPort macPort)
{
    uintptr_t rgmiiCfgAddr = Icssg_getRgmiiCfgAddr(hIcssg);
    uint32_t slice = IcssgUtils_getSliceNum(hIcssg, macPort);
    uint32_t val;

    val = Icssg_rd32(hIcssg, rgmiiCfgAddr);

    if (ICSSG_IS_SLICE_0(slice))
    {
        val |= CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG_RGMII0_GIG_IN_MASK;
        val |= CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG_RGMII0_FULLDUPLEX_IN_MASK;
    }
    else
    {
        val |= CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG_RGMII1_GIG_IN_MASK;
        val |= CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG_RGMII1_FULLDUPLEX_IN_MASK;
    }

    Icssg_wr32(hIcssg, rgmiiCfgAddr, val);
}

static void Icssg_updateLinkSpeed10MB(Icssg_Handle hIcssg,
                                      Enet_MacPort macPort,
                                      EnetPhy_Duplexity duplex)
{
    uintptr_t dram = Icssg_getDramAddr(hIcssg, macPort);

    if (duplex == ENETPHY_DUPLEX_HALF)
    {
        Icssg_updateRgmiiCfg10HD(hIcssg, macPort);
        /* Notify the port link speed to firmware */
        Icssg_wr8(hIcssg, dram + PORT_LINK_SPEED_OFFSET, FW_LINK_SPEED_10M_HD);
    }
    else
    {
        Icssg_updateRgmiiCfg10FD(hIcssg, macPort);
        /* Notify the port link speed to firmware */
        Icssg_wr8(hIcssg, dram + PORT_LINK_SPEED_OFFSET, FW_LINK_SPEED_10M);
    }
}

static void Icssg_updateLinkSpeed100MB(Icssg_Handle hIcssg,
                                       Enet_MacPort macPort,
                                       EnetPhy_Duplexity duplex)
{
    uintptr_t dram = Icssg_getDramAddr(hIcssg, macPort);
    uintptr_t addr;
    uint32_t slice = IcssgUtils_getSliceNum(hIcssg, macPort);
    uint32_t val;

    if (duplex == ENETPHY_DUPLEX_HALF)
    {
        Icssg_updateRgmiiCfg100HD(hIcssg, macPort);
        /* Notify the port link speed to firmware */
        Icssg_wr8(hIcssg, dram + PORT_LINK_SPEED_OFFSET, FW_LINK_SPEED_100M_HD);
    }
    else
    {
        Icssg_updateRgmiiCfg100FD(hIcssg, macPort);
        /* Notify the port link speed to firmware */
        Icssg_wr8(hIcssg, dram + PORT_LINK_SPEED_OFFSET, FW_LINK_SPEED_100M);
    }

    /* Configuring 960ns TX IPG in ICSSG HW MMR */
    addr = Icssg_getTxIpgCfgAddr(hIcssg, macPort, false);
    Icssg_wr32(hIcssg, addr, ICSSG_CFG_TX_IPG_960_NS);

    /* HW Errata: work-around to do RMW to TXIPG0 so that TXIPG1 will be latched by HW */
    if (ICSSG_IS_SLICE_1(slice))
    {
        addr = Icssg_getTxIpgCfgAddr(hIcssg, macPort, true);
        val = Icssg_rd32(hIcssg, addr);
        Icssg_wr32(hIcssg, addr, val);
    }
}

static void Icssg_updateLinkSpeed1G(Icssg_Handle hIcssg,
                                    Enet_MacPort macPort)
{
    uintptr_t dram = Icssg_getDramAddr(hIcssg, macPort);
    uintptr_t addr;
    uint32_t slice = IcssgUtils_getSliceNum(hIcssg, macPort);
    uint32_t val;

    Icssg_updateRgmiiCfg1G(hIcssg, macPort);

    /* Configuring 104ns TX IPG ICSSG HW MMR */
    addr = Icssg_getTxIpgCfgAddr(hIcssg, macPort, false);
    Icssg_wr32(hIcssg, addr, ICSSG_CFG_TX_IPG_104_NS);

    /* HW Errata: work-around to do RMW to TXIPG0 so that TXIPG1 will be latched by HW */
    if (ICSSG_IS_SLICE_1(slice))
    {
        addr = Icssg_getTxIpgCfgAddr(hIcssg, macPort, true);
        val = Icssg_rd32(hIcssg, addr);
        Icssg_wr32(hIcssg, addr, val);
    }

    /* Notify the port link speed to firmware */
    Icssg_wr8(hIcssg, dram + PORT_LINK_SPEED_OFFSET, FW_LINK_SPEED_1G);
}

static void Icssg_updateLinkDown(Icssg_Handle hIcssg,
                                 Enet_MacPort macPort)
{
    uintptr_t rgmiiCfgAddr = Icssg_getRgmiiCfgAddr(hIcssg);
    uint32_t slice = IcssgUtils_getSliceNum(hIcssg, macPort);
    uint32_t val;

    val = Icssg_rd32(hIcssg, rgmiiCfgAddr);

    if (ICSSG_IS_SLICE_0(slice))
    {
        val |= CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG_RGMII0_GIG_IN_MASK;
        val |= CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG_RGMII0_FULLDUPLEX_IN_MASK;
        val &= ~CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG_RGMII0_INBAND_MASK;
    }
    else
    {
        val |= CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG_RGMII1_GIG_IN_MASK;
        val |= CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG_RGMII1_FULLDUPLEX_IN_MASK;
        val &= ~CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG_RGMII1_INBAND_MASK;
    }

    Icssg_wr32(hIcssg, rgmiiCfgAddr, val);
}

uint64_t Icssg_convertTs(EnetPer_Handle hPer,
                         uint64_t ts)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    uintptr_t sharedRam = Icssg_getSharedRamAddr(hIcssg);
    uint32_t cycleTimeNs = hIcssg->cycleTimeNs;
    uint32_t swHi;
    uint32_t iepCntLo;
    uint32_t iepCntHi;
    uint32_t rolloverCntHi;
    uint64_t ns;

    if(hIcssg->clockTypeFw == ICSSG_TIMESYNC_CLKTYPE_SYSTEM_TIME)
    {
        /* Working clock conversion not required in case FW doing in system time*/
        ns = ts;
    }
    else
    {
        swHi = Icssg_rd32(hIcssg, sharedRam + TIMESYNC_FW_WC_COUNT_HI_SW_OFFSET_OFFSET);

        iepCntLo = (uint32_t)(ts & 0xFFFFFU);
        iepCntHi = (uint32_t)((ts >> 20U) & 0x7FFFFFU);
        rolloverCntHi = (uint32_t)((ts >> 43U) & 0x1FFFFFU);

        ns = ((uint64_t)rolloverCntHi) << 23U | (iepCntHi + swHi);
        ns = ns * cycleTimeNs + iepCntLo;
    }
    return ns;
}

static int32_t Icssg_cfgMdioLinkInt(Icssg_Handle hIcssg,
                                 Enet_Type enetType,
                                 uint32_t instId,
                                 const Icssg_Cfg *cfg)
{
    Icssg_MdioLinkIntCtx *linkIntCtx = &hIcssg->mdioLinkIntCtx;
    uintptr_t key;
    uint32_t status;

    linkIntCtx->aliveMask            = ENET_MDIO_PHY_ADDR_MASK_NONE;
    linkIntCtx->linkedMask           = ENET_MDIO_PHY_ADDR_MASK_NONE;
    linkIntCtx->pollEnableMask       = cfg->mdioCfg.pollEnMask;
    linkIntCtx->linkStateChangeCb    = cfg->mdioLinkIntCfg.mdioLinkStateChangeCb;
    linkIntCtx->linkStateChangeCbArg = cfg->mdioLinkIntCfg.mdioLinkStateChangeCbArg;
    linkIntCtx->pruEvtNum[0]         = cfg->mdioLinkIntCfg.pruEvtNum[0];
    linkIntCtx->pruEvtNum[1]         = cfg->mdioLinkIntCfg.pruEvtNum[1];
    key = EnetOsal_disableAllIntr();

    /* Register interrupts */
    status = Icssg_registerMdioLinkIntr(hIcssg, &(cfg->mdioLinkIntCfg));
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to register interrupts: %d\r\n", status);

    EnetOsal_restoreAllIntr(key);

    return status;
}

static void Icssg_handleMdioLinkStateChange(EnetMdio_Group group,
                                            Mdio_PhyStatus *phyStatus,
                                            void *cbArgs)
{
    Icssg_Handle hIcssg = (Icssg_Handle)cbArgs;
    Icssg_MdioLinkIntCtx *linkIntCtx = &hIcssg->mdioLinkIntCtx;
    Icssg_MdioLinkStateChangeInfo info;
    uint32_t aliveMaskChange;
    uint32_t linkedMaskChange;
    uint32_t i;

    aliveMaskChange  = phyStatus->aliveMask ^ linkIntCtx->aliveMask;
    linkedMaskChange = phyStatus->linkedMask ^ linkIntCtx->linkedMask;

    for (i = 0U; i <= MDIO_MAX_PHY_CNT; i++)
    {
        info.aliveChanged = ENET_IS_BIT_SET(aliveMaskChange, i);
        info.linkChanged  = ENET_IS_BIT_SET(linkedMaskChange, i);

        if (info.aliveChanged || info.linkChanged)
        {
            info.phyAddr  = i;

            info.isAlive  = ENET_IS_BIT_SET(phyStatus->aliveMask, i);
            info.isLinked = ENET_IS_BIT_SET(phyStatus->linkedMask, i);

            if (linkIntCtx->linkStateChangeCb != NULL)
            {
                linkIntCtx->linkStateChangeCb(&info, linkIntCtx->linkStateChangeCbArg);
            }
        }
    }

    linkIntCtx->aliveMask  = phyStatus->aliveMask;
    linkIntCtx->linkedMask = phyStatus->linkedMask;
}

static void Icssg_mdioIsr(uintptr_t arg)
{
    Icssg_Handle hIcssg = (Icssg_Handle )arg;
    PRUICSS_Handle hPruIcss = hIcssg->pruss->hPruss;
    EnetMod_Handle hMdio = hIcssg->hMdio;
    Enet_IoctlPrms prms;
    Mdio_Callbacks callbacks =
    {
        .linkStateCb  = Icssg_handleMdioLinkStateChange,
        .userAccessCb = NULL,
        .cbArgs       = hIcssg,
    };
    uintptr_t baseaddr;
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t status, pruEvtNum = 0;

    ENET_IOCTL_SET_IN_ARGS(&prms, &callbacks);
    status = EnetMod_ioctlFromIsr(hMdio, MDIO_IOCTL_HANDLE_INTR, &prms);

    /* TODO: Add ISR safe error:
     * ("Failed to handle MDIO intr: %d\r\n", status); */
    ENET_UNUSED(status);

    if (hPruIcss != NULL)
    {
        hwAttrs = (PRUICSS_HwAttrs const *)hPruIcss->hwAttrs;
        baseaddr = hwAttrs->intcRegBase;
        pruEvtNum = Icssg_rd32(hIcssg, (baseaddr + CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_GLB_PRI_INTR_REG));
    }

    if(pruEvtNum == hIcssg->mdioLinkIntCtx.pruEvtNum[0])
    {
        PRUICSS_clearEvent(hPruIcss, hIcssg->mdioLinkIntCtx.pruEvtNum[0]);
    }
    else if(pruEvtNum == hIcssg->mdioLinkIntCtx.pruEvtNum[1])
    {
        PRUICSS_clearEvent(hPruIcss, hIcssg->mdioLinkIntCtx.pruEvtNum[1]);
    }

}


static int32_t Icssg_registerMdioLinkIntr(Icssg_Handle hIcssg,
                                          const Icssg_mdioLinkIntCfg *mdioLinkIntCfg)
{
    int32_t status = ENET_SOK;
    uint32_t trigType;

    if (mdioLinkIntCfg->mdioLinkStateChangeCb != NULL)
    {
        Enet_assert((NULL != hIcssg->pruss) && (hIcssg->pruss->initialized == true));

        hIcssg->mdioLinkIntCtx.prussIntcInitData = mdioLinkIntCfg->prussIntcInitData;
        trigType = (mdioLinkIntCfg->isPulseIntr == true) ? ENETOSAL_ARM_GIC_TRIG_TYPE_EDGE : ENETOSAL_ARM_GIC_TRIG_TYPE_LEVEL;

        hIcssg->mdioLinkIntCtx.hMdioIntr = EnetOsal_registerIntr(Icssg_mdioIsr,
                                                 mdioLinkIntCfg->coreIntrNum,
                                                 mdioLinkIntCfg->intrPrio,
                                                 trigType,
                                                 hIcssg);
        if (hIcssg->mdioLinkIntCtx.hMdioIntr == NULL)
        {
            ENETTRACE_ERR("Failed to register MDIO interrupt\r\n");
            status = ENET_EFAIL;
        }
        if (ENET_SOK == status)
        {
            status = Icssg_enablePruIcssInt(hIcssg);
            ENETTRACE_ERR_IF((status != ENET_SOK),
                            "%s: failed to enable pruicss interrupt: %d\r\n",
                            ENET_PER_NAME(hIcssg), status);
        }
    }
    return status;
}

static int32_t Icssg_enablePruIcssInt(Icssg_Handle hIcssg)
{
    uint32_t  status;

    if (hIcssg->mdioLinkIntCtx.prussIntcInitData != NULL)
    {
        status = PRUICSS_intcInit(hIcssg->pruss->hPruss, hIcssg->mdioLinkIntCtx.prussIntcInitData);
    }
    else
    {
        status = ENET_EFAIL;
    }
    return status;
}

static void Icssg_unregisterMdioLinkIntr(Icssg_Handle hIcssg)
{
    /* Unregister MDIO interrupt */
    if (hIcssg->mdioLinkIntCtx.hMdioIntr != NULL)
    {
        EnetOsal_unregisterIntr(hIcssg->mdioLinkIntCtx.hMdioIntr);
        hIcssg->mdioLinkIntCtx.hMdioIntr = NULL;
    }

}


static int32_t Icssg_handleExternalPhyLinkUp(Icssg_Handle hIcssg,
                                             Enet_MacPort macPort,
                                             const EnetPhy_LinkCfg *phyLinkCfg)
{
    uint32_t portId = ENET_MACPORT_ID(macPort);
    int32_t status = ENET_SOK;

    ENETTRACE_VAR(portId);

    switch (phyLinkCfg->speed)
    {
        case ENETPHY_SPEED_10MBIT:
            Icssg_updateLinkSpeed10MB(hIcssg, macPort, phyLinkCfg->duplexity);
            break;

        case ENETPHY_SPEED_100MBIT:
            Icssg_updateLinkSpeed100MB(hIcssg, macPort, phyLinkCfg->duplexity);
            break;

        case ENETPHY_SPEED_1GBIT:
            Icssg_updateLinkSpeed1G(hIcssg, macPort);
            break;

        default:
            Enet_assert(false, "%s: Port %u: invalid link speed %u\r\n",
                        ENET_PER_NAME(hIcssg), portId, phyLinkCfg->speed);
            break;
    }

    ENETTRACE_INFO("%s: Port %u: Link up: %s %s\r\n",
                   ENET_PER_NAME(hIcssg), portId,
                   Icssg_gSpeedNames[phyLinkCfg->speed],
                   Icssg_gDuplexNames[phyLinkCfg->duplexity]);
    return status;
}


int32_t Icssg_ioctl_handler_ENET_PER_IOCTL_OPEN_PORT_LINK(EnetPer_Handle hPer,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    int32_t status = ENET_SOK;
    const EnetPer_PortLinkCfg *inArgs = (const EnetPer_PortLinkCfg *)prms->inArgs;
    Enet_MacPort macPort = inArgs->macPort;

    Enet_assert(cmd == ENET_PER_IOCTL_OPEN_PORT_LINK);

    ENETTRACE_VAR(macPort);
    status = Icssg_ioctlPortLinkCfg(hIcssg, inArgs);
    ENETTRACE_ERR_IF((status != ENET_SOK),
                        "%s: Port %u: failed to open port link: %d\r\n",
                        ENET_PER_NAME(hIcssg), ENET_MACPORT_ID(macPort), status);
    return status;
}

int32_t Icssg_ioctl_handler_ENET_PER_IOCTL_CLOSE_PORT_LINK(EnetPer_Handle hPer,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    int32_t status = ENET_SOK;
    Enet_MacPort macPort = *(Enet_MacPort *)prms->inArgs;

    Enet_assert(cmd == ENET_PER_IOCTL_CLOSE_PORT_LINK);

    Icssg_closeEnetPhy(hIcssg, macPort);
    return status;
}

int32_t Icssg_ioctl_handler_ENET_PER_IOCTL_IS_PORT_LINK_UP(EnetPer_Handle hPer,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    int32_t status = ENET_SOK;
    Enet_MacPort macPort = *(Enet_MacPort *)prms->inArgs;
    bool *linkUp = (bool *)prms->outArgs;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    EnetPhy_Handle hPhy;

    Enet_assert(cmd == ENET_PER_IOCTL_IS_PORT_LINK_UP);

    if (portNum < ENET_ARRAYSIZE(hIcssg->hPhy))
    {
        hPhy = hIcssg->hPhy[portNum];
        if (hPhy != NULL)
        {
            *linkUp = EnetPhy_isLinked(hPhy);
        }
        else
        {
            *linkUp = false;
        }
    }
    else
    {
        ENETTRACE_ERR("%s: invalid MAC port %u\r\n",
                        ENET_PER_NAME(hIcssg), ENET_MACPORT_ID(macPort));
        *linkUp = false;
        status = ENET_EINVALIDPARAMS;
    }
    return status;
}

int32_t Icssg_ioctl_handler_ENET_PER_IOCTL_GET_PORT_LINK_CFG(EnetPer_Handle hPer,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    int32_t status = ENET_SOK;
    Enet_MacPort macPort = *(Enet_MacPort *)prms->inArgs;
    EnetMacPort_LinkCfg *linkCfg = (EnetMacPort_LinkCfg *)prms->outArgs;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    EnetPhy_Handle hPhy;
    EnetPhy_LinkCfg phyLinkCfg;

    Enet_assert(cmd == ENET_PER_IOCTL_GET_PORT_LINK_CFG);

    if (portNum < ENET_ARRAYSIZE(hIcssg->hPhy))
    {
        hPhy = hIcssg->hPhy[portNum];
        if (hPhy != NULL)
        {
            status = EnetPhy_getLinkCfg(hPhy, &phyLinkCfg);
            if (status == ENET_SOK)
            {
                /* Link config enums are compatible, typecast is enough */
                linkCfg->speed = (Enet_Speed)phyLinkCfg.speed;
                linkCfg->duplexity = (Enet_Duplexity)phyLinkCfg.duplexity;
            }
            else
            {
                ENETTRACE_ERR("%s: Port %u: failed to get PHY link config: %d\r\n",
                                ENET_PER_NAME(hIcssg), ENET_MACPORT_ID(macPort), status);
            }
        }
        else
        {
            ENETTRACE_WARN("%s: Port %u: can't get link config of a closed port\r\n",
                            ENET_PER_NAME(hIcssg), ENET_MACPORT_ID(macPort));
            status = ENET_EPERM;
        }
    }
    else
    {
        ENETTRACE_ERR("%s: invalid MAC port %u\r\n",
                        ENET_PER_NAME(hIcssg), ENET_MACPORT_ID(macPort));
    }
    return status;
}


int32_t Icssg_ioctl_handler_ICSSG_MACPORT_IOCTL_ENABLE_PROMISC_MODE(EnetPer_Handle hPer,
                                                                    uint32_t cmd,
                                                                    Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    Enet_MacPort *inArgs = (Enet_MacPort *)prms->inArgs;
    int32_t status = ENET_SOK;
    Enet_assert(cmd == ICSSG_MACPORT_IOCTL_ENABLE_PROMISC_MODE);

    Icssg_setPromiscMode(hIcssg, *inArgs, true);
    return status;
}


int32_t Icssg_ioctl_handler_ICSSG_MACPORT_IOCTL_DISABLE_PROMISC_MODE(EnetPer_Handle hPer,
                                                                    uint32_t cmd,
                                                                    Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    int32_t status = ENET_SOK;
    Enet_MacPort *inArgs = (Enet_MacPort *)prms->inArgs;

    Enet_assert(cmd == ICSSG_MACPORT_IOCTL_DISABLE_PROMISC_MODE);

    Icssg_setPromiscMode(hIcssg, *inArgs, false);
    return status;
}


int32_t Icssg_ioctl_handler_ICSSG_MACPORT_IOCTL_ENABLE_UCAST_FLOOD(EnetPer_Handle hPer,
                                                                    uint32_t cmd,
                                                                    Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    int32_t status = ENET_SOK;
    Enet_MacPort macPort = *(Enet_MacPort *)prms->inArgs;

    Enet_assert(cmd == ICSSG_MACPORT_IOCTL_ENABLE_UCAST_FLOOD);

    Icssg_UCastFloodingCtrl(hIcssg, macPort, true);
    return status;
}


int32_t Icssg_ioctl_handler_ICSSG_MACPORT_IOCTL_DISABLE_UCAST_FLOOD(EnetPer_Handle hPer,
                                                                    uint32_t cmd,
                                                                    Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    int32_t status = ENET_SOK;
    Enet_MacPort macPort = *(Enet_MacPort *)prms->inArgs;

    Enet_assert(cmd == ICSSG_MACPORT_IOCTL_DISABLE_UCAST_FLOOD);

    Icssg_UCastFloodingCtrl(hIcssg, macPort, false);
    return status;
}


int32_t Icssg_ioctl_handler_ICSSG_MACPORT_IOCTL_ENABLE_MCAST_FLOOD(EnetPer_Handle hPer,
                                                                    uint32_t cmd,
                                                                    Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    int32_t status = ENET_SOK;
    Enet_MacPort macPort = *(Enet_MacPort *)prms->inArgs;

    Enet_assert(cmd == ICSSG_MACPORT_IOCTL_ENABLE_MCAST_FLOOD);

    Icssg_MCastFloodingCtrl(hIcssg, macPort, true);
    return status;
}


int32_t Icssg_ioctl_handler_ICSSG_MACPORT_IOCTL_DISABLE_MCAST_FLOOD(EnetPer_Handle hPer,
                                                                    uint32_t cmd,
                                                                    Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    int32_t status = ENET_SOK;
    Enet_MacPort macPort = *(Enet_MacPort *)prms->inArgs;

    Enet_assert(cmd == ICSSG_MACPORT_IOCTL_DISABLE_MCAST_FLOOD);

    Icssg_MCastFloodingCtrl(hIcssg, macPort, false);
    return status;
}


int32_t Icssg_ioctl_handler_ICSSG_PER_IOCTL_VLAN_RESET_TABLE(EnetPer_Handle hPer,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    Icssg_VlanFidParams *inArgs = (Icssg_VlanFidParams *)prms->inArgs;
    int32_t status = ENET_SOK;

    Enet_assert(cmd == ICSSG_PER_IOCTL_VLAN_RESET_TABLE);

    Icssg_ioctlVlanFidResetTable(hIcssg, inArgs);

    return status;
}


int32_t Icssg_ioctl_handler_ICSSG_PER_IOCTL_VLAN_SET_ENTRY(EnetPer_Handle hPer,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    int32_t status = ENET_SOK;
    Icssg_VlanFidEntry *inArgs = (Icssg_VlanFidEntry *)prms->inArgs;

    Enet_assert(cmd == ICSSG_PER_IOCTL_VLAN_SET_ENTRY);

    status = Icssg_ioctlVlanFidSetEntry(hIcssg, inArgs);
    ENETTRACE_ERR_IF((status != ENET_SOK),
                        "%s: failed to set vlan table entry: %d\r\n",
                        ENET_PER_NAME(hIcssg), status);

    return status;
}


int32_t Icssg_ioctl_handler_ICSSG_PER_IOCTL_VLAN_GET_ENTRY(EnetPer_Handle hPer,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    int32_t status = ENET_SOK;
    Icssg_VlanFidEntry *outArgs = (Icssg_VlanFidEntry *)prms->outArgs;

    Enet_assert(cmd == ICSSG_PER_IOCTL_VLAN_GET_ENTRY);

    status = Icssg_ioctlVlanFidGetEntry(hIcssg, outArgs);
    ENETTRACE_ERR_IF((status != ENET_SOK),
                        "%s: failed to get vlan table entry: %d\r\n",
                        ENET_PER_NAME(hIcssg), status);
    return status;
}


int32_t Icssg_ioctl_handler_ICSSG_PER_IOCTL_VLAN_SET_HOSTPORT_DFLT_VID(EnetPer_Handle hPer,
                                                                        uint32_t cmd,
                                                                        Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    int32_t status = ENET_SOK;
    EnetPort_VlanCfg *vlanCfg = (EnetPort_VlanCfg *)prms->inArgs;

    Enet_assert(cmd == ICSSG_PER_IOCTL_VLAN_SET_HOSTPORT_DFLT_VID);

    status = Icssg_configHostPortDfltVlanId(hIcssg, vlanCfg);
    ENETTRACE_ERR_IF((status != ENET_SOK),
                        "%s: failed to config host port default VLAN Id: %d\r\n",
                        ENET_PER_NAME(hIcssg), status);
    return status;
}


int32_t Icssg_ioctl_handler_ICSSG_PER_IOCTL_VLAN_SET_MACPORT_DFLT_VID(EnetPer_Handle hPer,
                                                                        uint32_t cmd,
                                                                        Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    int32_t status = ENET_SOK;
    Icssg_MacPortDfltVlanCfgInArgs *inArgs = (Icssg_MacPortDfltVlanCfgInArgs *)prms->inArgs;
    Enet_MacPort macPort = inArgs->macPort;

    Enet_assert(cmd == ICSSG_PER_IOCTL_VLAN_SET_MACPORT_DFLT_VID);

    status = Icssg_configMacPortDfltVlanId(hIcssg, macPort, &inArgs->vlanCfg);
    ENETTRACE_ERR_IF((status != ENET_SOK),
                        "%s: Port %u: failed to config default VLAN Id: %d\r\n",
                        ENET_PER_NAME(hIcssg), ENET_MACPORT_ID(macPort), status);
    return status;
}


int32_t Icssg_ioctl_handler_ICSSG_PER_IOCTL_SET_PORT_STATE(EnetPer_Handle hPer,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    int32_t status = ENET_SOK;
    IcssgMacPort_SetPortStateInArgs *inArgs = (IcssgMacPort_SetPortStateInArgs *)prms->inArgs;
    Enet_MacPort macPort = inArgs->macPort;

    Enet_assert(cmd == ICSSG_PER_IOCTL_SET_PORT_STATE);

    ENETTRACE_VAR(macPort);

    status = Icssg_ioctlSetPortState(hIcssg, inArgs);
    ENETTRACE_ERR_IF((status != ENET_SINPROGRESS),
                        "%s: port %u: failed to set port state: %d\r\n",
                        ENET_PER_NAME(hIcssg), ENET_MACPORT_ID(macPort), status);
    return status;
}


int32_t Icssg_ioctl_handler_ICSSG_FDB_IOCTL_ADD_ENTRY(EnetPer_Handle hPer,
                                                        uint32_t cmd,
                                                        Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    int32_t status = ENET_SOK;
    Icssg_FdbEntry *inArgs = (Icssg_FdbEntry *)prms->inArgs;

    Enet_assert(cmd == ICSSG_FDB_IOCTL_ADD_ENTRY);

    /* It's a peripheral level IOCTL but command is issued as if it were for port 1 */
    status = Icssg_ioctlFdbAddEntry(hIcssg, ENET_MAC_PORT_1, inArgs);
    ENETTRACE_ERR_IF((status != ENET_SINPROGRESS),
                        "%s: failed to add fdb entry: %d\r\n",
                        ENET_PER_NAME(hIcssg), status);
    return status;
}


int32_t Icssg_ioctl_handler_ICSSG_FDB_IOCTL_REMOVE_ENTRY(EnetPer_Handle hPer,
                                                        uint32_t cmd,
                                                        Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    int32_t status = ENET_SOK;
    Icssg_FdbEntry *inArgs = (Icssg_FdbEntry *)prms->inArgs;

    Enet_assert(cmd == ICSSG_FDB_IOCTL_REMOVE_ENTRY);

    /* It's a peripheral level IOCTL but command is issued as if it were for port 1 */
    status = Icssg_ioctlFdbRemoveEntry(hIcssg, ENET_MAC_PORT_1, inArgs);
    ENETTRACE_ERR_IF((status != ENET_SINPROGRESS),
                        "%s: failed to remove fdb entry: %d\r\n",
                        ENET_PER_NAME(hIcssg), status);
    return status;
}


int32_t Icssg_ioctl_handler_ICSSG_FDB_IOCTL_REMOVE_ALL_ENTRIES(EnetPer_Handle hPer,
                                                                uint32_t cmd,
                                                                Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    int32_t status = ENET_SOK;

    Enet_assert(cmd == ICSSG_FDB_IOCTL_REMOVE_ALL_ENTRIES);

    /* REVISIT - In Switch mode, should this be MAC port dependent? */
    status = Icssg_ioctlFdbRemoveAllEntries(hIcssg, ENET_MAC_PORT_1);
    ENETTRACE_ERR_IF((status != ENET_SINPROGRESS),
                        "%s: failed to remove all entries: %d\r\n",
                        ENET_PER_NAME(hIcssg), status);
    return status;
}


int32_t Icssg_ioctl_handler_ICSSG_FDB_IOCTL_REMOVE_AGEABLE_ENTRIES(EnetPer_Handle hPer,
                                                                    uint32_t cmd,
                                                                    Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    int32_t status = ENET_SOK;

    Enet_assert(cmd == ICSSG_FDB_IOCTL_REMOVE_AGEABLE_ENTRIES);

    /* REVISIT - In Switch mode, should this be MAC port dependent? */
    status = Icssg_ioctlFdbRemoveAllAgeableEntries(hIcssg, ENET_MAC_PORT_1);
    ENETTRACE_ERR_IF((status != ENET_SINPROGRESS),
                        "%s: failed to remove all ageable entries: %d\r\n",
                        ENET_PER_NAME(hIcssg), status);
    return status;
}


int32_t Icssg_ioctl_handler_ICSSG_MACPORT_IOCTL_SET_MACADDR(EnetPer_Handle hPer,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    int32_t status = ENET_SOK;
    IcssgMacPort_SetMacAddressInArgs *inArgs = (IcssgMacPort_SetMacAddressInArgs *)prms->inArgs;
    Enet_MacPort macPort = inArgs->macPort;

    Enet_assert(cmd == ICSSG_MACPORT_IOCTL_SET_MACADDR);

    ENETTRACE_VAR(macPort);

    Icssg_ioctlSetMacAddress(hIcssg, inArgs);
    return status;
}


int32_t Icssg_ioctl_handler_ICSSG_HOSTPORT_IOCTL_SET_MACADDR(EnetPer_Handle hPer,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    int32_t status = ENET_SOK;
    Icssg_MacAddr *inArgs = (Icssg_MacAddr *)prms->inArgs;

    Enet_assert(cmd == ICSSG_HOSTPORT_IOCTL_SET_MACADDR);

    Icssg_ioctlSetMacAddressHostPort(hIcssg, inArgs);
    return status;
}


int32_t Icssg_ioctl_handler_ENET_MACPORT_IOCTL_ENABLE_PREEMPTION(EnetPer_Handle hPer,
                                                                    uint32_t cmd,
                                                                    Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    int32_t status = ENET_SOK;
    Enet_MacPort macPort = *(Enet_MacPort *)prms->inArgs;

    Enet_assert(cmd == ENET_MACPORT_IOCTL_ENABLE_PREEMPTION);

    if (!(hIcssg->isPremQueEnable))
    {
        status = ENET_ENOTSUPPORTED;
    }
    if (status == ENET_SOK)
    {
        status = Icssg_ioctlPreemptTxEnable(hIcssg, macPort);
        ENETTRACE_ERR_IF((status != ENET_SINPROGRESS),
                        "%s: Port %u: failed to configure preempt TX enable: %d\r\n",
                        ENET_PER_NAME(hIcssg), ENET_MACPORT_ID(macPort), status);
    }

    return status;
}

int32_t Icssg_ioctl_handler_ENET_MACPORT_IOCTL_DISABLE_PREEMPTION(EnetPer_Handle hPer,
                                                                    uint32_t cmd,
                                                                    Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    int32_t status = ENET_SOK;
    Enet_MacPort macPort = *(Enet_MacPort *)prms->inArgs;

    Enet_assert(cmd == ENET_MACPORT_IOCTL_DISABLE_PREEMPTION);

    status = Icssg_ioctlPreemptTxDisable(hIcssg, macPort);
    ENETTRACE_ERR_IF((status != ENET_SINPROGRESS),
                        "%s: Port %u: failed to configure preempt TX disable: %d\r\n",
                        ENET_PER_NAME(hIcssg), ENET_MACPORT_ID(macPort), status);

    return status;
}


int32_t Icssg_ioctl_handler_ENET_MACPORT_IOCTL_GET_PREEMPTION_ENABLE_STATUS(EnetPer_Handle hPer,
                                                                            uint32_t cmd,
                                                                            Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    int32_t status = ENET_SOK;
    Enet_MacPort macPort = *(Enet_MacPort *)prms->inArgs;
    bool *enabled = (bool *)prms->outArgs;

    Enet_assert(cmd == ENET_MACPORT_IOCTL_GET_PREEMPTION_ENABLE_STATUS);

    status = Icssg_ioctlPreemptGetTxEnableStatus(hIcssg, macPort, enabled);
    ENETTRACE_ERR_IF((status != ENET_SOK),
                        "%s: Port %u: failed to get preempt TX enable status: %d\r\n",
                        ENET_PER_NAME(hIcssg), ENET_MACPORT_ID(macPort), status);

    return status;
}


int32_t Icssg_ioctl_handler_ENET_MACPORT_IOCTL_GET_PREEMPTION_ACTIVE_STATUS(EnetPer_Handle hPer,
                                                                            uint32_t cmd,
                                                                            Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    int32_t status = ENET_SOK;
    Enet_MacPort macPort = *(Enet_MacPort *)prms->inArgs;
    bool *active = (bool *)prms->outArgs;

    Enet_assert(cmd == ENET_MACPORT_IOCTL_GET_PREEMPTION_ACTIVE_STATUS);

    status = Icssg_ioctlPreemptGetTxActiveStatus(hIcssg, macPort, active);
    ENETTRACE_ERR_IF((status != ENET_SOK),
                            "%s: Port %u: failed to get preempt TX enable status: %d\r\n",
                            ENET_PER_NAME(hIcssg), ENET_MACPORT_ID(macPort), status);

    return status;
}


int32_t Icssg_ioctl_handler_ENET_MACPORT_IOCTL_ENABLE_PREEMPT_VERIFICATION(EnetPer_Handle hPer,
                                                                        uint32_t cmd,
                                                                        Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    int32_t status = ENET_SOK;
    Enet_MacPort macPort = *(Enet_MacPort *)prms->inArgs;

    Enet_assert(cmd == ENET_MACPORT_IOCTL_ENABLE_PREEMPT_VERIFICATION);

    status = Icssg_ioctlPreemptVerifyEnable(hIcssg, macPort);
    ENETTRACE_ERR_IF((status != ENET_SINPROGRESS),
                        "%s: Port %u: failed to verify preempt enable: %d\r\n",
                        ENET_PER_NAME(hIcssg), ENET_MACPORT_ID(macPort), status);

    return status;
}


int32_t Icssg_ioctl_handler_ENET_MACPORT_IOCTL_DISABLE_PREEMPT_VERIFICATION(EnetPer_Handle hPer,
                                                                        uint32_t cmd,
                                                                        Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    int32_t status = ENET_SOK;
    Enet_MacPort macPort = *(Enet_MacPort *)prms->inArgs;

    Enet_assert(cmd == ENET_MACPORT_IOCTL_DISABLE_PREEMPT_VERIFICATION);

    status = Icssg_ioctlPreemptVerifyDisable(hIcssg, macPort);
    ENETTRACE_ERR_IF((status != ENET_SINPROGRESS),
                        "%s: Port %u: failed to verify preempt disable: %d\r\n",
                        ENET_PER_NAME(hIcssg), ENET_MACPORT_ID(macPort), status);

    return status;
}

int32_t Icssg_ioctl_handler_ENET_MACPORT_IOCTL_GET_PREEMPT_VERIFY_STATUS(EnetPer_Handle hPer,
                                                                        uint32_t cmd,
                                                                        Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    int32_t status = ENET_SOK;
    Enet_MacPort macPort = *(Enet_MacPort *)prms->inArgs;
    EnetMacPort_PreemptVerifyStatus *outArgs = (EnetMacPort_PreemptVerifyStatus *)prms->outArgs;

    Enet_assert(cmd == ENET_MACPORT_IOCTL_GET_PREEMPT_VERIFY_STATUS);

    status = Icssg_ioctlPreemptGetVerifyState(hIcssg, macPort, outArgs);
    ENETTRACE_ERR_IF((status != ENET_SOK),
                        "%s: Port %u: failed to get preempt verify state: %d\r\n",
                        ENET_PER_NAME(hIcssg), ENET_MACPORT_ID(macPort), status);
    return status;
}

int32_t Icssg_ioctl_handler_ENET_MACPORT_IOCTL_GET_PREEMPT_MIN_FRAG_SIZE(EnetPer_Handle hPer,
                                                                                uint32_t cmd,
                                                                                Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    int32_t status = ENET_SOK;
    Enet_MacPort macPort = *(Enet_MacPort *)prms->inArgs;
    uint8_t *minFragSize = (uint8_t *)prms->outArgs;

    Enet_assert(cmd == ENET_MACPORT_IOCTL_GET_PREEMPT_MIN_FRAG_SIZE);

    status = Icssg_ioctlPreemptGetMinFragSizeLocal(hIcssg, macPort, minFragSize);
    ENETTRACE_ERR_IF((status != ENET_SOK),
                        "%s: Port %u: failed to get preempt min frag size local: %d\r\n",
                        ENET_PER_NAME(hIcssg), ENET_MACPORT_ID(macPort), status);

    return status;
}

int32_t Icssg_ioctl_handler_ENET_MACPORT_IOCTL_SET_PREEMPT_MIN_FRAG_SIZE(EnetPer_Handle hPer,
                                                                                uint32_t cmd,
                                                                                Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    int32_t status = ENET_SOK;
    EnetMacPort_SetPreemptMinFragSizeInArgs *inArgs =
                (EnetMacPort_SetPreemptMinFragSizeInArgs *)prms->inArgs;

    Enet_assert(cmd == ENET_MACPORT_IOCTL_SET_PREEMPT_MIN_FRAG_SIZE);

    status = Icssg_ioctlPreemptSetMinFragSizeRemote(hIcssg, inArgs);
    ENETTRACE_ERR_IF((status != ENET_SOK),
                        "%s: Port %u: failed to verify preempt disable: %d\r\n",
                        ENET_PER_NAME(hIcssg), ENET_MACPORT_ID(inArgs->macPort), status);
    return status;
}

int32_t Icssg_ioctl_handler_ENET_MACPORT_IOCTL_SET_PREEMPT_QUEUE(EnetPer_Handle hPer,
                                                                 uint32_t cmd,
                                                                 Enet_IoctlPrms *prms)
{

    int32_t status = ENET_SOK;

    Enet_assert(cmd == ENET_MACPORT_IOCTL_GET_QUEUE_PREEMPT_STATUS);

    status = ENET_EINVALIDPARAMS;
    Enet_assert(status != ENET_SOK);
    return status;
}

int32_t Icssg_ioctl_handler_ENET_MACPORT_IOCTL_GET_QUEUE_PREEMPT_STATUS(EnetPer_Handle hPer,
                                                                        uint32_t cmd,
                                                                        Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

    Enet_assert(cmd == ENET_MACPORT_IOCTL_GET_QUEUE_PREEMPT_STATUS);

    status = ENET_EINVALIDPARAMS;
    Enet_assert(status != ENET_SOK);
    return status;
}

int32_t Icssg_ioctl_handler_ENET_IOCTL_REGISTER_RX_DEFAULT_FLOW(EnetPer_Handle hPer,
                                                                uint32_t cmd,
                                                                Enet_IoctlPrms *prms)
{
    Enet_assert(cmd == ENET_IOCTL_REGISTER_RX_DEFAULT_FLOW);
    return ENET_ENOTSUPPORTED;
}

int32_t Icssg_ioctl_handler_ENET_IOCTL_UNREGISTER_RX_DEFAULT_FLOW(EnetPer_Handle hPer,
                                                                    uint32_t cmd,
                                                                    Enet_IoctlPrms *prms)
{
    Enet_assert(cmd == ENET_IOCTL_UNREGISTER_RX_DEFAULT_FLOW);
    return ENET_ENOTSUPPORTED;
}

int32_t Icssg_ioctl_handler_ENET_PER_IOCTL_ATTACH_CORE(EnetPer_Handle hPer,
                                                        uint32_t cmd,
                                                        Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    int32_t status = ENET_SOK;
    uint32_t i;
    uint32_t coreId = *(uint32_t *)prms->inArgs;
    EnetPer_AttachCoreOutArgs *outArgs = (EnetPer_AttachCoreOutArgs *)prms->outArgs;
    Enet_IoctlPrms rmPrms;

    Enet_assert(cmd == ENET_PER_IOCTL_ATTACH_CORE);

    ENET_IOCTL_SET_INOUT_ARGS(&rmPrms, &coreId, &outArgs->coreKey);
    ENET_RM_PRIV_IOCTL(hIcssg->hRm, ENET_RM_IOCTL_ATTACH, &rmPrms, status);
    if (status == ENET_SOK)
    {
        /* Get MTU values */
        for (i = 0U; i < ENET_PRI_NUM; i++)
        {
            outArgs->txMtu[i] = 2008U; //FIXME - Does ICSSG supports per priority MTU?
        }

        //outArgs->rxMtu = 1536U;
        outArgs->rxMtu = ENET_RX_MTU_MAX;
#if 0
        outArgs->rxMtu = CSL_REG32_FEXT(hIcssg->dram0 +
                                        CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_REGS_BASE +
                                        CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_RX_FRMS0,
                                        ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_RX_FRMS0_RX_MAX_FRM0);
        /* MTU is one more than value programmed */
        outArgs->rxMtu += 1U;
#endif
    }
    else
    {
        ENETTRACE_ERR("%s: failed to attach core %u: %d\r\n",
                        ENET_PER_NAME(hIcssg), coreId, status);
    }
    return status;
}

int32_t Icssg_ioctl_handler_ENET_PER_IOCTL_DETACH_CORE(EnetPer_Handle hPer,
                                                        uint32_t cmd,
                                                        Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    int32_t status = ENET_SOK;
    uint32_t coreKey = *((uint32_t *)prms->inArgs);
    Enet_IoctlPrms rmPrms;

    Enet_assert(cmd == ENET_PER_IOCTL_DETACH_CORE);

    ENET_IOCTL_SET_IN_ARGS(&rmPrms, &coreKey);
    ENET_RM_PRIV_IOCTL(hIcssg->hRm, ENET_RM_IOCTL_DETACH, &rmPrms, status);
    ENETTRACE_ERR_IF((status != ENET_SOK),
                        "%s: failed to detach core: %d\r\n",
                        ENET_PER_NAME(hIcssg), status);
    return status;
}

int32_t Icssg_ioctl_handler_ICSSG_PER_IOCTL_TAS_TRIGGER(EnetPer_Handle hPer,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    int32_t status = ENET_SOK;
    Enet_MacPort macPort = *(Enet_MacPort *)prms->inArgs;

    Enet_assert(cmd == ICSSG_PER_IOCTL_TAS_TRIGGER);

    status = Icssg_R30SendAsyncIoctl(hIcssg,
                                        macPort,
                                        ICSSG_UTILS_R30_CMD_TAS_TRIGGER,
                                        &hIcssg->asyncIoctlSeqNum,
                                        &hIcssg->asyncIoctlType);
    ENETTRACE_ERR_IF((status != ENET_SOK),
                        "%s: Port %u: failed to do TAS trigger: %d\r\n",
                        ENET_PER_NAME(hIcssg), ENET_MACPORT_ID(macPort), status);
    return status;
}

int32_t Icssg_ioctl_handler_ICSSG_PER_IOCTL_TAS_ENABLE(EnetPer_Handle hPer,
                                                        uint32_t cmd,
                                                        Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    int32_t status = ENET_SOK;
    Enet_MacPort macPort = *(Enet_MacPort *)prms->inArgs;

    Enet_assert(cmd == ICSSG_PER_IOCTL_TAS_ENABLE);

    status = Icssg_R30SendAsyncIoctl(hIcssg,
                                        macPort,
                                        ICSSG_UTILS_R30_CMD_TAS_ENABLE,
                                        &hIcssg->asyncIoctlSeqNum,
                                        &hIcssg->asyncIoctlType);
    ENETTRACE_ERR_IF((status != ENET_SOK),
                        "%s: Port %u: failed to do TAS enable: %d\r\n",
                        ENET_PER_NAME(hIcssg), ENET_MACPORT_ID(macPort), status);
    return status;
}

int32_t Icssg_ioctl_handler_ICSSG_PER_IOCTL_TAS_DISABLE(EnetPer_Handle hPer,
                                                        uint32_t cmd,
                                                        Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    int32_t status = ENET_SOK;
    Enet_MacPort macPort = *(Enet_MacPort *)prms->inArgs;

    Enet_assert(cmd == ICSSG_PER_IOCTL_TAS_DISABLE);

    status = Icssg_R30SendAsyncIoctl(hIcssg,
                                        macPort,
                                        ICSSG_UTILS_R30_CMD_DISABLE,
                                        &hIcssg->asyncIoctlSeqNum,
                                        &hIcssg->asyncIoctlType);
    ENETTRACE_ERR_IF((status != ENET_SOK),
                        "%s: Port %u: failed to do TAS disable %d\r\n",
                        ENET_PER_NAME(hIcssg), ENET_MACPORT_ID(macPort), status);
    return status;
}

int32_t Icssg_ioctl_handler_ICSSG_PER_IOCTL_TAS_RESET(EnetPer_Handle hPer,
                                                        uint32_t cmd,
                                                        Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    int32_t status = ENET_SOK;
    Enet_MacPort macPort = *(Enet_MacPort *)prms->inArgs;

    Enet_assert(cmd == ICSSG_PER_IOCTL_TAS_RESET);

    status = Icssg_R30SendAsyncIoctl(hIcssg,
                                        macPort,
                                        ICSSG_UTILS_R30_CMD_TAS_RESET,
                                        &hIcssg->asyncIoctlSeqNum,
                                        &hIcssg->asyncIoctlType);
    ENETTRACE_ERR_IF((status != ENET_SOK),
                        "%s: Port %u: failed to do TAS reset: %d\r\n",
                        ENET_PER_NAME(hIcssg), ENET_MACPORT_ID(macPort), status);

    return status;
}

int32_t Icssg_ioctl_handler_ICSSG_FDB_IOCTL_SET_AGING_PERIOD(EnetPer_Handle hPer,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    int32_t status = ENET_SOK;
    uint64_t period = *(uint64_t *)prms->inArgs;

    Enet_assert(cmd == ICSSG_FDB_IOCTL_SET_AGING_PERIOD);

    Icssg_ioctlCfgAgeingPeriod(hIcssg, period);
    return status;
}

int32_t Icssg_ioctl_handler_ICSSG_MACPORT_IOCTL_SET_ACCEPT_FRAME_CHECK(EnetPer_Handle hPer,
                                                                        uint32_t cmd,
                                                                        Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    int32_t status = ENET_SOK;
    Icssg_SetAcceptFrameCheckInArgs *inArgs = (Icssg_SetAcceptFrameCheckInArgs *)prms->inArgs;
    Enet_MacPort macPort = inArgs->macPort;

    Enet_assert(cmd == ICSSG_MACPORT_IOCTL_SET_ACCEPT_FRAME_CHECK);

    status = Icssg_setAcceptableFrameCheck(hIcssg,
                                            macPort,
                                            inArgs->acceptFrameCheck);
    ENETTRACE_ERR_IF((status != ENET_SINPROGRESS),
                        "%s: Port %u: failed to set acceptable frame check: %d\r\n",
                        ENET_PER_NAME(hIcssg), ENET_MACPORT_ID(macPort), status);
    return status;
}

int32_t Icssg_ioctl_handler_ICSSG_MACPORT_IOCTL_SET_INGRESS_RATE_LIM(EnetPer_Handle hPer,
                                                                    uint32_t cmd,
                                                                    Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    int32_t status = ENET_SOK;
    Icssg_IngressRateLim *rateLimCfg = (Icssg_IngressRateLim *)prms->inArgs;
    Enet_MacPort macPort = rateLimCfg->macPort;

    Enet_assert(cmd == ICSSG_MACPORT_IOCTL_SET_INGRESS_RATE_LIM);

    status = Icssg_setIngressRateLim(hIcssg, macPort, rateLimCfg);
    ENETTRACE_ERR_IF((status != ENET_SOK),
                        "Failed to set ingress rate limiting for MAC port %u: %d\r\n",
                        ENET_MACPORT_ID(macPort), status);
    return status;
}

int32_t Icssg_ioctl_handler_ENET_PER_IOCTL_SET_VLAN_AWARE(EnetPer_Handle hPer,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    int32_t status = ENET_SOK;

    Enet_assert(cmd == ENET_PER_IOCTL_SET_VLAN_AWARE);

    status = Icssg_setVlanAwareMode(hIcssg);
    ENETTRACE_ERR_IF((status != ENET_SOK),
                        "%s: failed to set VLAN aware mode: %d\r\n",
                        ENET_PER_NAME(hIcssg), status);
    return status;
}

int32_t Icssg_ioctl_handler_ENET_PER_IOCTL_SET_VLAN_UNAWARE(EnetPer_Handle hPer,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    int32_t status = ENET_SOK;

    Enet_assert(cmd == ENET_PER_IOCTL_SET_VLAN_UNAWARE);

    status = Icssg_setVlanUnawareMode(hIcssg);
    ENETTRACE_ERR_IF((status != ENET_SOK),
                        "%s: failed to set VLAN unaware mode: %d\r\n",
                        ENET_PER_NAME(hIcssg), status);
    return status;
}

int32_t Icssg_ioctl_handler_ICSSG_MACPORT_IOCTL_SET_QUEUE_CUT_THROUGH_PREEMPT_SELECT(EnetPer_Handle hPer,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    int32_t status = ENET_SOK;
    IcssgMacPort_SetQueueCtPremptModeInArgs *inArgs =
        (IcssgMacPort_SetQueueCtPremptModeInArgs *)prms->inArgs;

    Enet_assert(cmd == ICSSG_MACPORT_IOCTL_SET_QUEUE_CUT_THROUGH_PREEMPT_SELECT);

    Enet_MacPort macPort = inArgs->macPort;

    Icssg_configCutThroughOrPreempt(hIcssg, macPort,
                                &inArgs->queuePreemptMode[0U],
                                &inArgs->queueForwardMode[0U]);
    return status;
}

int32_t Icssg_ioctl_handler_ICSSG_MACPORT_IOCTL_CONFIG_SPL_FRAME_PRIO(EnetPer_Handle hPer,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    int32_t status = ENET_SOK;
    IcssgMacPort_ConfigSpecialFramePrioInArgs *inArgs =
        (IcssgMacPort_ConfigSpecialFramePrioInArgs *)prms->inArgs;
    Enet_MacPort macPort = inArgs->macPort;
    uint8_t specialFramePrio = inArgs->specialFramePrio;

    Enet_assert(cmd == ICSSG_MACPORT_IOCTL_CONFIG_SPL_FRAME_PRIO);

    Icssg_setSpecialFramePrioCfg(hIcssg, macPort, specialFramePrio);

    return status;
}

int32_t Icssg_ioctl_handler_ENET_PER_IOCTL_HANDLE_EXTPHY_LINKUP_EVENT(EnetPer_Handle hPer,
                                                                    uint32_t cmd,
                                                                    Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    int32_t status = ENET_SOK;
    Enet_ExtPhyLinkUpEventInfo *linkInfo = (Enet_ExtPhyLinkUpEventInfo *)prms->inArgs;

    Enet_assert(cmd == ENET_PER_IOCTL_HANDLE_EXTPHY_LINKUP_EVENT);

    status = Icssg_handleExternalPhyLinkUp(hIcssg, linkInfo->macPort, &linkInfo->phyLinkCfg);
    ENETTRACE_ERR_IF((status != ENET_SOK),
                        "%s: Link Up Failed: %d\r\n",
                        ENET_PER_NAME(hIcssg), status);
    return status;
}

int32_t Icssg_ioctl_handler_ENET_PER_IOCTL_HANDLE_EXTPHY_LINKDOWN_EVENT(EnetPer_Handle hPer,
                                                                        uint32_t cmd,
                                                                        Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    int32_t status = ENET_SOK;
    Enet_MacPort macPort = *((Enet_MacPort *)prms->inArgs);

    Enet_assert(cmd == ENET_PER_IOCTL_HANDLE_EXTPHY_LINKDOWN_EVENT);

    status = Icssg_handleLinkDown(hIcssg, macPort);
    ENETTRACE_ERR_IF((status != ENET_SOK),
                        "%s: Link Down failed: %d\r\n",
                        ENET_PER_NAME(hIcssg), status);
    return status;
}

int32_t IcssgMacPort_ioctl_handler_ENET_MACPORT_IOCTL_SET_EGRESS_QOS_PRI_MAP(EnetPer_Handle hPer,
                                                                            uint32_t cmd,
                                                                            Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    int32_t status = ENET_SOK;
    EnetMacPort_SetEgressPriorityMapInArgs *inArgs = (EnetMacPort_SetEgressPriorityMapInArgs *)prms->inArgs;
    Enet_MacPort macPort = inArgs->macPort;

    Enet_assert(cmd == ENET_MACPORT_IOCTL_SET_EGRESS_QOS_PRI_MAP);

    status = Icssg_setPriorityMapping(hIcssg, macPort, &inArgs->priorityMap);
    ENETTRACE_ERR_IF((status != ENET_SOK),
                        "%s: Port %u: failed to set QoS priority: %d\r\n",
                        ENET_PER_NAME(hIcssg), ENET_MACPORT_ID(macPort), status);
    return status;
}

int32_t IcssgMacPort_ioctl_handler_ENET_MACPORT_IOCTL_SET_PRI_REGEN_MAP(EnetPer_Handle hPer,
                                                                        uint32_t cmd,
                                                                        Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    int32_t status = ENET_SOK;

    EnetMacPort_SetPriorityRegenMapInArgs *inArgs = (EnetMacPort_SetPriorityRegenMapInArgs *)prms->inArgs;
    Enet_MacPort macPort = inArgs->macPort;

    Enet_assert(cmd == ENET_MACPORT_IOCTL_SET_PRI_REGEN_MAP);

    status = Icssg_setPriorityRegen(hIcssg, macPort, &inArgs->priorityRegenMap);
    ENETTRACE_ERR_IF((status != ENET_SOK),
                        "%s: Port %u: failed to set priority regeneration: %d\r\n",
                        ENET_PER_NAME(hIcssg), ENET_MACPORT_ID(macPort), status);
    return status;

}

int32_t IcssgMacPort_ioctl_handler_ENET_MACPORT_IOCTL_SET_INGRESS_DSCP_PRI_MAP(EnetPer_Handle hPer,
                                                                                uint32_t cmd,
                                                                                Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    int32_t status = ENET_SOK;
    EnetMacPort_SetIngressDscpPriorityMapInArgs *inArgs = (EnetMacPort_SetIngressDscpPriorityMapInArgs *)prms->inArgs;
    Enet_MacPort macPort = inArgs->macPort;
    uintptr_t dram = Icssg_getDramAddr(hIcssg, macPort);

    Enet_assert(cmd == ENET_MACPORT_IOCTL_SET_INGRESS_DSCP_PRI_MAP);

    Icssg_wr8(hIcssg, dram + DSCP_ENABLE_DISABLE_STATUS, inArgs->dscpPriorityMap.dscpIPv4En);

    if(inArgs->dscpPriorityMap.dscpIPv4En)
    {
        status = Icssg_setDscpPriority(hIcssg, macPort, &inArgs->dscpPriorityMap);
        ENETTRACE_ERR_IF((status != ENET_SOK),
                        "%s: Port %u: failed to set dscp priority map: %d\r\n",
                        ENET_PER_NAME(hIcssg), ENET_MACPORT_ID(macPort), status);
        if(status == ENET_SOK)
        {
            status = Icssg_setDscpEnable(hIcssg, macPort);
            ENETTRACE_ERR_IF((status != ENET_SOK),
                            "%s: failed to set DSCP enable: %d\r\n",
                            ENET_PER_NAME(hIcssg), status);
        }
    }
    else
    {
        status = Icssg_setDscpDisable(hIcssg, macPort);
        ENETTRACE_ERR_IF((status != ENET_SOK),
                        "%s: failed to set DSCP disable: %d\r\n",
                        ENET_PER_NAME(hIcssg), status);
    }

    return status;
}

static int32_t Icssg_getInternalIoctlHandlerEntry(EnetPer_Handle hPer, uint32_t cmd, IcssgInternalIoctlHandlerTableEntry_t ioctlTbl[], uint32_t numEntries, uint32_t *entryIdx)
{
#if ((ENET_CFG_TRACE_LEVEL >= ENET_CFG_TRACE_LEVEL_ERROR) && ENET_CFG_IS_OFF(TRACE_DISABLE_INFOSTRING))
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
#endif
    uint32_t i;
    int32_t status;

    for (i = 0; i < numEntries; i++)
    {
        if (ioctlTbl[i].cmd == cmd)
        {
            break;
        }
    }
    if (i < numEntries)
    {
        *entryIdx = i;
        status = ENET_SOK;
    }
    else
    {
        ENETTRACE_ERR("%s: failed to get ioctl cmd entry for ICSSG IOCTL cmd %x\r\n", ENET_PER_NAME(hIcssg), cmd);
        status = ENET_EINVALIDPARAMS;
    }
    return status;
}

static IcssgInternalIoctlHandlerFxn_t * Icssg_getInternalIoctlHandler(EnetPer_Handle hPer, uint32_t cmd, IcssgInternalIoctlHandlerTableEntry_t ioctlTbl[], uint32_t numEntries)
{
#if ((ENET_CFG_TRACE_LEVEL >= ENET_CFG_TRACE_LEVEL_ERROR) && ENET_CFG_IS_OFF(TRACE_DISABLE_INFOSTRING))
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
#endif
    uint32_t entryIndex;
    int32_t status;
    IcssgInternalIoctlHandlerFxn_t *ioctlHandler = NULL;

    status = Icssg_getInternalIoctlHandlerEntry(hPer, cmd, ioctlTbl, numEntries, &entryIndex);
    if (status == ENET_SOK)
    {
        Enet_assert(entryIndex < numEntries);
        ioctlHandler = ioctlTbl[entryIndex].fxn;
    }
    else
    {
        ENETTRACE_ERR("%s: failed to get ioctl handler for ICSSG IOCTL cmd %x\r\n", ENET_PER_NAME(hIcssg), cmd);
        ioctlHandler = &Icssg_ioctl_handler_default;
    }
    return ioctlHandler;
}


int32_t Icssg_ioctl_handler_default(EnetPer_Handle hPer,
                                    uint32_t cmd,
                                    Enet_IoctlPrms *prms)
{
    return ENET_ENOTSUPPORTED;
}

int32_t Icssg_internalIoctlHandler_registerHandler(EnetPer_Handle hPer,
                                                    uint32_t cmd,
                                                    Enet_IoctlPrms *prms)
{
#if ((ENET_CFG_TRACE_LEVEL >= ENET_CFG_TRACE_LEVEL_ERROR) && ENET_CFG_IS_OFF(TRACE_DISABLE_INFOSTRING))
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
#endif
    int32_t status = ENET_SOK;
    IcssgInternalIoctlHandlerTableEntry_t *ioctlHandlerToRegister  = (IcssgInternalIoctlHandlerTableEntry_t *)prms->inArgs;
    IcssgInternalIoctlHandlerTableEntry_t *currentIoctlTblEntry;
    uint32_t entryIndex;

    status = Icssg_getInternalIoctlHandlerEntry(hPer, ioctlHandlerToRegister->cmd,
                                        IcssgInternalIoctlHandlerTable ,
                                        ENET_ARRAYSIZE(IcssgInternalIoctlHandlerTable),
                                        &entryIndex);
    if (status == ENET_SOK)
    {
        Enet_assert(entryIndex < ENET_ARRAYSIZE(IcssgInternalIoctlHandlerTable));
        currentIoctlTblEntry = &IcssgInternalIoctlHandlerTable[entryIndex];
        Enet_assert(ioctlHandlerToRegister->cmd == currentIoctlTblEntry->cmd);
        currentIoctlTblEntry->fxn = (IcssgInternalIoctlHandlerFxn_t *)ioctlHandlerToRegister->fxn;
    }
    else
    {
        ENETTRACE_ERR("%s: failed to register ioctl cmd entry for ICSSG IOCTL cmd %x\r\n", ENET_PER_NAME(hIcssg), cmd);
        status = ENET_EINVALIDPARAMS;
    }
    return status;
}


int32_t Icssg_registerIoctlHandler(EnetPer_Handle hPer,
                                    Enet_IoctlPrms *prms)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
    uint32_t major;
    int32_t status = ENET_SOK;
    Enet_IoctlRegisterHandlerInArgs *ioctlHandlerRegister = (Enet_IoctlRegisterHandlerInArgs *)prms->inArgs;


#if ENET_CFG_IS_ON(DEV_ERROR)
    /* Validate ICSSG peripheral IOCTL parameters */
    if ((ENET_IOCTL_GET_PER(ioctlHandlerRegister->cmd) == ENET_IOCTL_PER_ICSSG) &&
        (ENET_IOCTL_GET_TYPE(ioctlHandlerRegister->cmd) == ENET_IOCTL_TYPE_PUBLIC))
    {
        status = Enet_validateIoctl(ioctlHandlerRegister->cmd, prms,
                                    gIcssg_ioctlValidate,
                                    ENET_ARRAYSIZE(gIcssg_ioctlValidate));
        ENETTRACE_ERR_IF((status != ENET_SOK),
                         "%s: IOCTL 0x%08x params are not valid\r\n",
                         ENET_PER_NAME(hIcssg), ioctlHandlerRegister->cmd);
    }
#endif

    if (status == ENET_SOK)
    {
        major = ENET_IOCTL_GET_MAJ(ioctlHandlerRegister->cmd);
        switch (major)
        {
            case ENET_IOCTL_PER_BASE:
            {
                status = Icssg_internalIoctlHandler_registerHandler(hPer, ioctlHandlerRegister->cmd, prms);
            }
            break;

            case ENET_IOCTL_MDIO_BASE:
            {
                status = EnetMod_ioctl(hIcssg->hMdio, MDIO_IOCTL_REGISTER_HANDLER, prms);
            }
            break;

            case ENET_IOCTL_RM_BASE:
            {
                status = EnetMod_ioctl(hIcssg->hRm, ENET_RM_IOCTL_REGISTER_HANDLER, prms);
            }
            break;

            case ENET_IOCTL_TIMESYNC_BASE:
            {
                if (hIcssg->hTimeSync != NULL)
                {
                    status = EnetMod_ioctl(hIcssg->hTimeSync, ICSSG_TIMESYNC_IOCTL_REGISTER_HANDLER, prms);
                }
                else
                {
                    ENETTRACE_ERR("%s: TimeSync is not enabled\r\n", ENET_PER_NAME(hIcssg));
                    status = ENET_ENOTSUPPORTED;
                }
            }
            break;

            case ENET_IOCTL_TAS_BASE:
            {
                /* Note: Typecast to GenericInArgs is possible because all public
                 * MAC port IOCTL input args have macPort as their first member */
                EnetTas_RegisterIoctlInArgs tasInArgs;
                Enet_IoctlPrms tasPrms;
                /* Register Handler is not instance based, pass first macport as tas
                 * instance does not matter. Register Handler updates a table which is
                 * per module not per instance
                */
                Enet_MacPort macPort = ENET_MAC_PORT_FIRST;
                uint32_t portNum = ENET_MACPORT_NORM(macPort);
                EnetMod_Handle hTas;

                Enet_assert(portNum < ENET_ARRAYSIZE(hIcssg->hTas));

                hTas = hIcssg->hTas[portNum];
                if (hTas != NULL)
                {
                    tasInArgs.commonInArgs.macPort = macPort;
                    tasInArgs.registerHandler = *ioctlHandlerRegister;
                    ENET_IOCTL_SET_IN_ARGS(&tasPrms, &tasInArgs);
                    status = EnetMod_ioctl(hTas, ICSSG_TAS_IOCTL_REGISTER_HANDLER, &tasPrms);
                }
                else
                {
                    ENETTRACE_ERR("%s: Tas is not enabled\n", ENET_PER_NAME(hIcssg));
                    status = ENET_ENOTSUPPORTED;
                }
            }
            break;

            case ENET_IOCTL_FDB_BASE:
            case ENET_IOCTL_HOSTPORT_BASE:
            {
                status = ENET_ENOTSUPPORTED;
            }
            break;

            case ENET_IOCTL_MACPORT_BASE:
            {
                status = Icssg_ioctlMacPort(hIcssg, ICSSG_MACPORT_IOCTL_REGISTER_HANDLER, prms);
                ENETTRACE_ERR_IF((status != ENET_SOK),
                                 "%s: failed to run IOCTL 0x%08x: %d\r\n",
                                 ENET_PER_NAME(hIcssg), ioctlHandlerRegister->cmd, status);
            }
            break;

            case ENET_IOCTL_STATS_BASE:
            {
                status = EnetMod_ioctl(hIcssg->hStats, ICSSG_STATS_IOCTL_REGISTER_HANDLER, prms);
            }
            break;

            case ENET_IOCTL_PHY_BASE:
            {
                /* Register Handler is not instance based, pass first macport as mac port
                 * instance does not matter. Register Handler updates a table which is
                 * per module not per instance
                */
                Enet_MacPort macPort = ENET_MAC_PORT_FIRST;
                uint32_t portNum = ENET_MACPORT_NORM(macPort);
                EnetPhy_Handle hPhy;

                if (portNum < ENET_ARRAYSIZE(hIcssg->hPhy))
                {
                    hPhy = hIcssg->hPhy[portNum];
                    if (hPhy != NULL)
                    {
                        status = EnetPhyMdioDflt_ioctl(hPhy, ENET_PHY_IOCTL_REGISTER_HANDLER, prms);
                    }
                    else
                    {
                        ENETTRACE_ERR("%s: port %u is not open\r\n",
                                      ENET_PER_NAME(hIcssg), ENET_MACPORT_ID(macPort));
                        status = ENET_EFAIL;
                    }
                }
                else
                {
                    ENETTRACE_ERR("%s: invalid MAC port %u\r\n",
                                  ENET_PER_NAME(hIcssg), ENET_MACPORT_ID(macPort));
                    status = ENET_EINVALIDPARAMS;
                }
            }
            break;

            default:
            {
                status = ENET_EUNKNOWNIOCTL;
                break;
            }
        }
    }
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to register IOCTL handler: %d, %x, %p\r\n",
                     status,
                     ioctlHandlerRegister->cmd,
                     ioctlHandlerRegister->fxn);
    return status;
}



int32_t Icssg_ioctl_handler_ENET_PER_IOCTL_REGISTER_IOCTL_HANDLER(EnetPer_Handle hPer,
                                                                    uint32_t cmd,
                                                                    Enet_IoctlPrms *prms)
{
    int32_t status;

    status = Icssg_registerIoctlHandler(hPer, prms);
    return status;
}

int32_t IcssgMacPort_ioctl_handler_default(EnetPer_Handle hPer,
                                    uint32_t cmd,
                                    Enet_IoctlPrms *prms)
{
    return ENET_ENOTSUPPORTED;
}

static int32_t Icssg_getMacPortIoctlHandlerEntry(EnetPer_Handle hPer, uint32_t cmd, IcssgMacPortIoctlHandlerTableEntry_t ioctlTbl[], uint32_t numEntries, uint32_t *entryIdx)
{
#if ((ENET_CFG_TRACE_LEVEL >= ENET_CFG_TRACE_LEVEL_ERROR) && ENET_CFG_IS_OFF(TRACE_DISABLE_INFOSTRING))
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
#endif
    uint32_t i;
    int32_t status;

    for (i = 0; i < numEntries; i++)
    {
        if (ioctlTbl[i].cmd == cmd)
        {
            break;
        }
    }
    if (i < numEntries)
    {
        *entryIdx = i;
        status = ENET_SOK;
    }
    else
    {
        ENETTRACE_ERR("%s: failed to get ioctl cmd entry for ICSSG IOCTL cmd %x\r\n", ENET_PER_NAME(hIcssg), cmd);
        status = ENET_EINVALIDPARAMS;
    }
    return status;
}

static IcssgMacPortIoctlHandlerFxn_t * Icssg_getMacPortIoctlHandler(EnetPer_Handle hPer, uint32_t cmd, IcssgMacPortIoctlHandlerTableEntry_t ioctlTbl[], uint32_t numEntries)
{
    uint32_t entryIndex;
    int32_t status;
    IcssgMacPortIoctlHandlerFxn_t *ioctlHandler = NULL;

    status = Icssg_getMacPortIoctlHandlerEntry(hPer, cmd, ioctlTbl, numEntries, &entryIndex);
    if (status == ENET_SOK)
    {
        Enet_assert(entryIndex < numEntries);
        ioctlHandler = ioctlTbl[entryIndex].fxn;
    }
    else
    {
        ioctlHandler = &IcssgMacPort_ioctl_handler_default;
    }
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to get ioctl handler for ioctl cmd: %x. Using default ioctl handler \r\n", cmd);
    return ioctlHandler;
}

int32_t IcssgMacPort_ioctl_handler_registerHandler(EnetPer_Handle hPer,
                                                    uint32_t cmd,
                                                    Enet_IoctlPrms *prms)
{
#if ((ENET_CFG_TRACE_LEVEL >= ENET_CFG_TRACE_LEVEL_ERROR) && ENET_CFG_IS_OFF(TRACE_DISABLE_INFOSTRING))
    Icssg_Handle hIcssg = (Icssg_Handle)hPer;
#endif
    int32_t status = ENET_SOK;
    IcssgMacPortIoctlHandlerTableEntry_t *ioctlHandlerToRegister  = (IcssgMacPortIoctlHandlerTableEntry_t *)prms->inArgs;
    IcssgMacPortIoctlHandlerTableEntry_t *currentIoctlTblEntry;
    uint32_t entryIndex;

    Enet_assert(cmd == ICSSG_MACPORT_IOCTL_REGISTER_HANDLER);
    status = Icssg_getMacPortIoctlHandlerEntry(hPer, ioctlHandlerToRegister->cmd,
                                        IcssgMacPortIoctlHandlerTable ,
                                        ENET_ARRAYSIZE(IcssgMacPortIoctlHandlerTable),
                                        &entryIndex);
    if (status == ENET_SOK)
    {
        Enet_assert(entryIndex < ENET_ARRAYSIZE(IcssgMacPortIoctlHandlerTable));
        currentIoctlTblEntry = &IcssgMacPortIoctlHandlerTable[entryIndex];
        Enet_assert(ioctlHandlerToRegister->cmd == currentIoctlTblEntry->cmd);
        currentIoctlTblEntry->fxn = ioctlHandlerToRegister->fxn;
    }
    else
    {
        ENETTRACE_ERR("%s: failed to register ioctl cmd entry for ICSSG IOCTL cmd %x\r\n", ENET_PER_NAME(hIcssg), cmd);
        status = ENET_EINVALIDPARAMS;
    }
    return status;
}

int32_t IcssgMacPort_ioctl_handler_ICSSG_MACPORT_IOCTL_REGISTER_HANDLER(EnetPer_Handle hPer,
                                                                        uint32_t cmd,
                                                                        Enet_IoctlPrms *prms)
{
    int32_t status;

    status = IcssgMacPort_ioctl_handler_registerHandler(hPer, cmd, prms);
    return status;
}
