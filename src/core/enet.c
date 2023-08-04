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
 * \file  enet.c
 *
 * \brief This file contains the implementation of the main interface
 *        layer of the Enet driver.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <string.h>
#include <enet.h>
#include <enet_cfg.h>
#include <include/core/enet_base.h>
#include <include/core/enet_utils.h>
#include <include/core/enet_soc.h>
#include <include/core/enet_per.h>
#include <include/core/enet_mod_fdb.h>
#include <include/core/enet_mod_hostport.h>
#include <include/core/enet_mod_macport.h>
#include <include/core/enet_mod_mdio.h>
#include <include/core/enet_mod_port.h>
#include <include/core/enet_mod_stats.h>
#include <include/core/enet_mod_timesync.h>
#include <include/core/enet_mod_tas.h>
#include <priv/core/enet_base_priv.h>
#include <priv/core/enet_trace_priv.h>
#include <include/common/enet_osal_dflt.h>
#include <include/common/enet_utils_dflt.h>
#include <drivers/soc.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static int32_t Enet_getHandles(Enet_Type enetType,
                               uint32_t instId,
                               Enet_Handle *hEnet,
                               EnetPer_Handle *hPer);

static void EnetPer_initCfg(EnetPer_Handle hPer,
                            Enet_Type enetType,
                            void *cfg,
                            uint32_t cfgSize);

static int32_t EnetPer_open(EnetPer_Handle hPer,
                            Enet_Type enetType,
                            uint32_t instId,
                            const void *cfg,
                            uint32_t cfgSize);

static int32_t EnetPer_rejoin(EnetPer_Handle hPer,
                              Enet_Type enetType,
                              uint32_t instId);

static void  EnetPer_saveCtxt(EnetPer_Handle hPer);

static int32_t  EnetPer_restoreCtxt(EnetPer_Handle hPer,
                                    Enet_Type enetType,
                                    uint32_t instId);

static int32_t EnetPer_ioctl(EnetPer_Handle hPer,
                             uint32_t cmd,
                             Enet_IoctlPrms *prms);

static void EnetPer_periodicTick(EnetPer_Handle hPer);

static void EnetPer_poll(EnetPer_Handle hPer,
                         Enet_Event evt,
                         const void *arg,
                         uint32_t argSize);

static void EnetPer_registerEventCb(EnetPer_Handle hPer,
                                    Enet_Event evt,
                                    uint32_t evtNum,
                                    Enet_EventCallback evtCb,
                                    void *evtCbArgs);

static void EnetPer_unregisterEventCb(EnetPer_Handle hPer,
                                      Enet_Event evt,
                                      uint32_t evtNum);

static void EnetPer_close(EnetPer_Handle hPer);

#if ENET_CFG_IS_ON(DEV_ERROR)
int32_t Enet_validateGenericIoctl(uint32_t cmd,
                                  const Enet_IoctlPrms *prms);
#endif

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

#if ENET_CFG_IS_ON(DEV_ERROR)
/* Public Peripheral IOCTL validation data. */
static Enet_IoctlValidate gEnetPer_ioctlValidate[] =
{
    ENET_IOCTL_VALID_PRMS(ENET_PER_IOCTL_GET_VERSION,
                          0U,
                          sizeof(Enet_Version)),

    ENET_IOCTL_VALID_PRMS(ENET_PER_IOCTL_PRINT_REGS,
                          0U,
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_PER_IOCTL_OPEN_PORT_LINK,
                          sizeof(EnetPer_PortLinkCfg),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_PER_IOCTL_CLOSE_PORT_LINK,
                          sizeof(Enet_MacPort),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_PER_IOCTL_IS_PORT_LINK_UP,
                          sizeof(Enet_MacPort),
                          sizeof(bool)),

    ENET_IOCTL_VALID_PRMS(ENET_PER_IOCTL_SET_ISOLATE_STATE,
                          sizeof(Enet_MacPort),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_PER_IOCTL_CLEAR_ISOLATE_STATE,
                          sizeof(Enet_MacPort),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_PER_IOCTL_GET_PORT_LINK_CFG,
                          sizeof(Enet_MacPort),
                          sizeof(EnetMacPort_LinkCfg)),

    ENET_IOCTL_VALID_PRMS(ENET_PER_IOCTL_ATTACH_CORE,
                          sizeof(uint32_t),
                          sizeof(EnetPer_AttachCoreOutArgs)),

    ENET_IOCTL_VALID_PRMS(ENET_PER_IOCTL_DETACH_CORE,
                          sizeof(uint32_t),
                          0U),
    ENET_IOCTL_VALID_PRMS(ENET_IOCTL_REGISTER_RX_DEFAULT_FLOW,
                          sizeof(Enet_DfltFlowInfo),
                          0U),
    ENET_IOCTL_VALID_PRMS(ENET_IOCTL_UNREGISTER_RX_DEFAULT_FLOW,
                          sizeof(Enet_DfltFlowInfo),
                          0U),
    ENET_IOCTL_VALID_PRMS(ENET_IOCTL_REGISTER_DSTMAC_RX_FLOW,
                          sizeof(Enet_MacDstFlowInfo),
                          0U),
    ENET_IOCTL_VALID_PRMS(ENET_IOCTL_UNREGISTER_DSTMAC_RX_FLOW,
                          sizeof(Enet_MacDstFlowInfo),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_PER_IOCTL_SET_VLAN_AWARE,
                          0U,
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_PER_IOCTL_SET_VLAN_UNAWARE,
                          0U,
                          0U),
    ENET_IOCTL_VALID_PRMS(ENET_PER_IOCTL_HANDLE_EXTPHY_LINKUP_EVENT,
                          sizeof(Enet_ExtPhyLinkUpEventInfo),
                          0U),
    ENET_IOCTL_VALID_PRMS(ENET_PER_IOCTL_HANDLE_EXTPHY_LINKDOWN_EVENT,
                          sizeof(Enet_MacPort),
                          0U),
    ENET_IOCTL_VALID_PRMS(ENET_PER_IOCTL_REGISTER_IOCTL_HANDLER,
                          sizeof(Enet_IoctlRegisterHandlerInArgs),
                          0U),

};

/* Public FDB IOCTL validation data. */
static Enet_IoctlValidate gEnetFdb_ioctlValidate[] =
{
    ENET_IOCTL_VALID_PRMS(ENET_FDB_IOCTL_GET_VERSION,
                          0U,
                          sizeof(Enet_Version)),

    ENET_IOCTL_VALID_PRMS(ENET_FDB_IOCTL_PRINT_REGS,
                          0U,
                          0U),
};

/* Public host port IOCTL validation data. */
static Enet_IoctlValidate gEnetHostPort_ioctlValidate[] =
{
    ENET_IOCTL_VALID_PRMS(ENET_FDB_IOCTL_PRINT_REGS,
                          0U,
                          sizeof(Enet_Version)),

    ENET_IOCTL_VALID_PRMS(ENET_HOSTPORT_IOCTL_PRINT_REGS,
                          0U,
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_HOSTPORT_IOCTL_ENABLE,
                          0U,
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_HOSTPORT_IOCTL_DISABLE,
                          0U,
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_HOSTPORT_IOCTL_SET_INGRESS_DSCP_PRI_MAP,
                          sizeof(EnetPort_DscpPriorityMap),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_HOSTPORT_IOCTL_GET_INGRESS_DSCP_PRI_MAP,
                          0U,
                          sizeof(EnetPort_DscpPriorityMap)),

    ENET_IOCTL_VALID_PRMS(ENET_HOSTPORT_IOCTL_SET_PRI_REGEN_MAP,
                          sizeof(EnetPort_PriorityMap),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_HOSTPORT_IOCTL_GET_PRI_REGEN_MAP,
                          0U,
                          sizeof(EnetPort_PriorityMap)),

    ENET_IOCTL_VALID_PRMS(ENET_HOSTPORT_IOCTL_SET_EGRESS_QOS_PRI_MAP,
                          sizeof(EnetPort_PriorityMap),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_HOSTPORT_IOCTL_GET_EGRESS_QOS_PRI_MAP,
                          0U,
                          sizeof(EnetPort_PriorityMap)),

    ENET_IOCTL_VALID_PRMS(ENET_HOSTPORT_IOCTL_ENABLE_INGRESS_TRAFFIC_SHAPING,
                          sizeof(EnetPort_TrafficShapingCfg),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_HOSTPORT_IOCTL_DISABLE_INGRESS_TRAFFIC_SHAPING,
                          0U,
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_HOSTPORT_IOCTL_GET_INGRESS_TRAFFIC_SHAPING,
                          0U,
                          sizeof(EnetPort_TrafficShapingCfg)),

    ENET_IOCTL_VALID_PRMS(ENET_HOSTPORT_IOCTL_GET_MAXLEN,
                          0U,
                          sizeof(EnetPort_MaxLen)),

    ENET_IOCTL_VALID_PRMS(ENET_HOSTPORT_IS_CSUM_OFFLOAD_ENABLED,
                          0U,
                          sizeof(bool)),

    ENET_IOCTL_VALID_PRMS(ENET_HOSTPORT_IOCTL_SET_CREDIT_BASED_SHAPING,
                          sizeof(EnetPort_CreditBasedShapingCfg),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_HOSTPORT_IOCTL_GET_CREDIT_BASED_SHAPING,
                          0U,
                          sizeof(EnetPort_CreditBasedShapingCfg)),
};

/* Public MAC port IOCTL validation data. */
static Enet_IoctlValidate gEnetMacPort_ioctlValidate[] =
{
    ENET_IOCTL_VALID_PRMS(ENET_MACPORT_IOCTL_GET_VERSION,
                          sizeof(EnetMacPort_GenericInArgs),
                          sizeof(Enet_Version)),

    ENET_IOCTL_VALID_PRMS(ENET_MACPORT_IOCTL_PRINT_REGS,
                          sizeof(EnetMacPort_GenericInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_MACPORT_IOCTL_SET_INGRESS_DSCP_PRI_MAP,
                          sizeof(EnetMacPort_SetIngressDscpPriorityMapInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_MACPORT_IOCTL_GET_INGRESS_DSCP_PRI_MAP,
                          sizeof(EnetMacPort_GenericInArgs),
                          sizeof(EnetPort_DscpPriorityMap)),

    ENET_IOCTL_VALID_PRMS(ENET_MACPORT_IOCTL_SET_PRI_REGEN_MAP,
                          sizeof(EnetMacPort_SetPriorityRegenMapInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_MACPORT_IOCTL_GET_PRI_REGEN_MAP,
                          sizeof(EnetMacPort_GenericInArgs),
                          sizeof(EnetPort_PriorityMap)),

    ENET_IOCTL_VALID_PRMS(ENET_MACPORT_IOCTL_SET_EGRESS_QOS_PRI_MAP,
                          sizeof(EnetMacPort_SetEgressPriorityMapInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_MACPORT_IOCTL_GET_EGRESS_QOS_PRI_MAP,
                          sizeof(EnetMacPort_GenericInArgs),
                          sizeof(EnetPort_PriorityMap)),

    ENET_IOCTL_VALID_PRMS(ENET_MACPORT_IOCTL_ENABLE_EGRESS_TRAFFIC_SHAPING,
                          sizeof(EnetMacPort_EnableEgressTrafficShapingInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_MACPORT_IOCTL_DISABLE_EGRESS_TRAFFIC_SHAPING,
                          sizeof(EnetMacPort_GenericInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_MACPORT_IOCTL_GET_EGRESS_TRAFFIC_SHAPING,
                          sizeof(EnetMacPort_GenericInArgs),
                          sizeof(EnetPort_TrafficShapingCfg)),

    ENET_IOCTL_VALID_PRMS(ENET_MACPORT_IOCTL_GET_MAXLEN,
                          sizeof(EnetMacPort_GenericInArgs),
                          sizeof(EnetPort_MaxLen)),

    ENET_IOCTL_VALID_PRMS(ENET_MACPORT_IOCTL_GET_LINK_CFG,
                          sizeof(EnetMacPort_GenericInArgs),
                          sizeof(EnetMacPort_LinkCfg)),

    ENET_IOCTL_VALID_PRMS(ENET_MACPORT_IOCTL_SET_CREDIT_BASED_SHAPING,
                          sizeof(EnetMacPort_CreditBasedShaperInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_MACPORT_IOCTL_GET_CREDIT_BASED_SHAPING,
                          sizeof(EnetMacPort_GenericInArgs),
                          sizeof(EnetPort_CreditBasedShapingCfg)),

    ENET_IOCTL_VALID_PRMS(ENET_MACPORT_IOCTL_IET_RELEASE_PREEMPT_TRAFFIC,
                          sizeof(EnetMacPort_GenericInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_MACPORT_IOCTL_IET_HOLD_PREEMPT_TRAFFIC,
                          sizeof(EnetMacPort_GenericInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_MACPORT_IOCTL_GET_QUEUE_PREEMPT_STATUS,
                          sizeof(EnetMacPort_GenericInArgs),
                          sizeof(EnetMacPort_QueuePreemptCfg)),

    ENET_IOCTL_VALID_PRMS(ENET_MACPORT_IOCTL_SET_PREEMPT_QUEUE,
                          sizeof(EnetMacPort_SetPreemptQueueInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_MACPORT_IOCTL_GET_PREEMPT_MIN_FRAG_SIZE,
                          sizeof(EnetMacPort_GenericInArgs),
                          sizeof(uint8_t)),

    ENET_IOCTL_VALID_PRMS(ENET_MACPORT_IOCTL_SET_PREEMPT_MIN_FRAG_SIZE,
                          sizeof(EnetMacPort_SetPreemptMinFragSizeInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_MACPORT_IOCTL_GET_PREEMPT_VERIFY_STATUS,
                          sizeof(EnetMacPort_GenericInArgs),
                          sizeof(EnetMacPort_PreemptVerifyStatus)),

    ENET_IOCTL_VALID_PRMS(ENET_MACPORT_IOCTL_DISABLE_PREEMPT_VERIFICATION,
                          sizeof(EnetMacPort_GenericInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_MACPORT_IOCTL_ENABLE_PREEMPT_VERIFICATION,
                          sizeof(EnetMacPort_GenericInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_MACPORT_IOCTL_GET_PREEMPTION_ACTIVE_STATUS,
                          sizeof(EnetMacPort_GenericInArgs),
                          sizeof(bool)),

    ENET_IOCTL_VALID_PRMS(ENET_MACPORT_IOCTL_GET_PREEMPTION_ENABLE_STATUS,
                          sizeof(EnetMacPort_GenericInArgs),
                          sizeof(bool)),

    ENET_IOCTL_VALID_PRMS(ENET_MACPORT_IOCTL_DISABLE_PREEMPTION,
                          sizeof(EnetMacPort_GenericInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_MACPORT_IOCTL_ENABLE_PREEMPTION,
                          sizeof(EnetMacPort_GenericInArgs),
                          0U),
#if ENET_CFG_IS_ON(CPSW_CUTTHRU)
    ENET_IOCTL_VALID_PRMS(ENET_MACPORT_IOCTL_SET_CUT_THRU_PARAMS,
                          sizeof(EnetMacPort_CutThruParams),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_MACPORT_IOCTL_GET_CUT_THRU_PARAMS,
                          0U,
                          sizeof(EnetMacPort_CutThruParams)),
#endif
};

/* Public MDIO IOCTL validation data. */
static Enet_IoctlValidate gEnetMdio_ioctlValidate[] =
{
    ENET_IOCTL_VALID_PRMS(ENET_MDIO_IOCTL_GET_VERSION,
                          0U,
                          sizeof(Enet_Version)),

    ENET_IOCTL_VALID_PRMS(ENET_MDIO_IOCTL_PRINT_REGS,
                          0U,
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_MDIO_IOCTL_IS_ALIVE,
                          sizeof(uint32_t),
                          sizeof(bool)),

    ENET_IOCTL_VALID_PRMS(ENET_MDIO_IOCTL_IS_LINKED,
                          sizeof(uint32_t),
                          sizeof(bool)),

    ENET_IOCTL_VALID_PRMS(ENET_MDIO_IOCTL_IS_POLL_ENABLED,
                          sizeof(uint32_t),
                          sizeof(bool)),

    ENET_IOCTL_VALID_PRMS(ENET_MDIO_IOCTL_C22_READ,
                          sizeof(EnetMdio_C22ReadInArgs),
                          sizeof(uint16_t)),

    ENET_IOCTL_VALID_PRMS(ENET_MDIO_IOCTL_C22_WRITE,
                          sizeof(EnetMdio_C22WriteInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_MDIO_IOCTL_C45_READ,
                          sizeof(EnetMdio_C45ReadInArgs),
                          sizeof(uint16_t)),

    ENET_IOCTL_VALID_PRMS(ENET_MDIO_IOCTL_C45_WRITE,
                          sizeof(EnetMdio_C45WriteInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_MDIO_IOCTL_C22_ASYNC_READ_TRIGGER,
                          sizeof(EnetMdio_C22ReadInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_MDIO_IOCTL_C22_ASYNC_READ_COMPLETE,
                          sizeof(EnetMdio_C22ReadInArgs),
                          sizeof(uint16_t)),

    ENET_IOCTL_VALID_PRMS(ENET_MDIO_IOCTL_C22_ASYNC_WRITE_TRIGGER,
                          sizeof(EnetMdio_C22WriteInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_MDIO_IOCTL_C22_ASYNC_WRITE_COMPLETE,
                          sizeof(EnetMdio_C22WriteInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_MDIO_IOCTL_C45_ASYNC_READ_TRIGGER,
                          sizeof(EnetMdio_C45ReadInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_MDIO_IOCTL_C45_ASYNC_READ_COMPLETE,
                          sizeof(EnetMdio_C45ReadInArgs),
                          sizeof(uint16_t)),

    ENET_IOCTL_VALID_PRMS(ENET_MDIO_IOCTL_C45_ASYNC_WRITE_TRIGGER,
                          sizeof(EnetMdio_C45WriteInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_MDIO_IOCTL_C45_ASYNC_WRITE_COMPLETE,
                          sizeof(EnetMdio_C45WriteInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_MDIO_IOCTL_ENABLE_STATE_MACHINE,
                          0U,
                          0U),

};

/* Public Statistics IOCTL validation data. */
static Enet_IoctlValidate gEnetStats_ioctlValidate[] =
{
    ENET_IOCTL_VALID_PRMS(ENET_STATS_IOCTL_GET_VERSION,
                          0U,
                          sizeof(Enet_Version)),

    ENET_IOCTL_VALID_PRMS(ENET_STATS_IOCTL_PRINT_REGS,
                          0U,
                          0U),
};

/* Public Time Sync IOCTL validation data. */
static Enet_IoctlValidate gEnetTimeSync_ioctlValidate[] =
{
    ENET_IOCTL_VALID_PRMS(ENET_TIMESYNC_IOCTL_GET_VERSION,
                          0U,
                          sizeof(Enet_Version)),

    ENET_IOCTL_VALID_PRMS(ENET_TIMESYNC_IOCTL_PRINT_REGS,
                          0U,
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_TIMESYNC_IOCTL_PRINT_STATS,
                          0U,
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_TIMESYNC_IOCTL_GET_CURRENT_TIMESTAMP,
                          0U,
                          sizeof(uint64_t)),

    ENET_IOCTL_VALID_PRMS(ENET_TIMESYNC_IOCTL_SET_TIMESTAMP,
                          sizeof(EnetTimeSync_setTimestamp),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_TIMESYNC_IOCTL_ADJUST_TIMESTAMP,
                          sizeof(EnetTimeSync_TimestampAdj),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_TIMESYNC_IOCTL_GET_ETH_RX_TIMESTAMP,
                          sizeof(EnetTimeSync_GetEthTimestampInArgs),
                          sizeof(uint64_t)),

    ENET_IOCTL_VALID_PRMS(ENET_TIMESYNC_IOCTL_GET_ETH_TX_TIMESTAMP,
                          sizeof(EnetTimeSync_GetEthTimestampInArgs),
                          sizeof(uint64_t)),

    ENET_IOCTL_VALID_PRMS(ENET_TIMESYNC_IOCTL_RESET,
                          0U,
                          0U),
    ENET_IOCTL_VALID_PRMS(ENET_TIMESYNC_IOCTL_SET_TIMESTAMP_COMPLETE,
                          0U,
                          0U),
};

/* Public Tas IOCTL validation data. */
static Enet_IoctlValidate gEnetTas_ioctlValidate[] =
{
    ENET_IOCTL_VALID_PRMS(ENET_TAS_IOCTL_GET_VERSION,
                          0U,
                          sizeof(Enet_Version)),

    ENET_IOCTL_VALID_PRMS(ENET_TAS_IOCTL_SET_ADMIN_LIST,
                          sizeof(EnetTas_SetAdminListInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_TAS_IOCTL_GET_OPER_LIST_STATUS,
                          sizeof(Enet_MacPort),
                          sizeof(EnetTas_OperStatus)),

    ENET_IOCTL_VALID_PRMS(ENET_TAS_IOCTL_SET_STATE,
                          sizeof(EnetTas_SetStateInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_TAS_IOCTL_GET_STATE,
                          sizeof(Enet_MacPort),
                          sizeof(EnetTas_TasState)),

    ENET_IOCTL_VALID_PRMS(ENET_TAS_IOCTL_GET_ADMIN_LIST,
                          sizeof(Enet_MacPort),
                          sizeof(EnetTas_ControlList)),

    ENET_IOCTL_VALID_PRMS(ENET_TAS_IOCTL_GET_OPER_LIST,
                          sizeof(Enet_MacPort),
                          sizeof(EnetTas_ControlList)),

    ENET_IOCTL_VALID_PRMS(ENET_TAS_IOCTL_CONFIG_CHANGE_STATUS_PARAMS,
                          sizeof(Enet_MacPort),
                          sizeof(EnetTas_ConfigStatus)),
};

/* Public PHY IOCTL validation data. */
static Enet_IoctlValidate gEnetPhy_ioctlValidate[] =
{
    ENET_IOCTL_VALID_PRMS(ENET_PHY_IOCTL_GET_ID,
                          sizeof(EnetPhy_GenericInArgs),
                          sizeof(EnetPhy_Version)),

    ENET_IOCTL_VALID_PRMS(ENET_PHY_IOCTL_GET_SUPPORTED_MODES,
                          sizeof(EnetPhy_GenericInArgs),
                          sizeof(uint32_t)),

    ENET_IOCTL_VALID_PRMS(ENET_PHY_IOCTL_GET_LOOPBACK_STATE,
                          sizeof(EnetPhy_GenericInArgs),
                          sizeof(bool)),

    ENET_IOCTL_VALID_PRMS(ENET_PHY_IOCTL_IS_ALIVE,
                          sizeof(EnetPhy_GenericInArgs),
                          sizeof(bool)),

    ENET_IOCTL_VALID_PRMS(ENET_PHY_IOCTL_IS_LINKED,
                          sizeof(EnetPhy_GenericInArgs),
                          sizeof(bool)),

    ENET_IOCTL_VALID_PRMS(ENET_PHY_IOCTL_GET_LINK_MODE,
                          sizeof(EnetPhy_GenericInArgs),
                          sizeof(EnetMacPort_LinkCfg)),

    ENET_IOCTL_VALID_PRMS(ENET_PHY_IOCTL_RESET,
                          sizeof(EnetPhy_GenericInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_PHY_IOCTL_READ_REG,
                          sizeof(EnetPhy_ReadRegInArgs),
                          sizeof(uint16_t)),

    ENET_IOCTL_VALID_PRMS(ENET_PHY_IOCTL_WRITE_REG,
                          sizeof(EnetPhy_WriteRegInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_PHY_IOCTL_READ_EXT_REG,
                          sizeof(EnetPhy_ReadRegInArgs),
                          sizeof(uint16_t)),

    ENET_IOCTL_VALID_PRMS(ENET_PHY_IOCTL_WRITE_EXT_REG,
                          sizeof(EnetPhy_WriteRegInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_PHY_IOCTL_C45_READ_REG,
                          sizeof(EnetPhy_C45ReadRegInArgs),
                          sizeof(uint16_t)),

    ENET_IOCTL_VALID_PRMS(ENET_PHY_IOCTL_C45_WRITE_REG,
                          sizeof(EnetPhy_C45WriteRegInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_PHY_IOCTL_PRINT_REGS,
                          sizeof(EnetPhy_GenericInArgs),
                          0U),
};
#endif

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

uint32_t Enet_getCoreId(void)
{
    return EnetSoc_getCoreId();
}

EnetTrace_TraceLevel Enet_setTraceLevel(EnetTrace_TraceLevel level)
{
    return EnetTrace_setLevel(level);
}

EnetTrace_TraceLevel Enet_getTraceLevel(void)
{
    return EnetTrace_getLevel();
}

void Enet_initOsalCfg(EnetOsal_Cfg *cfg)
{
#if ENET_CFG_IS_ON(HAS_DEFAULT_OSAL)
    EnetOsalDflt_initCfg(cfg);
#else
    memset(cfg, 0, sizeof(*cfg));
#endif
}

void Enet_initUtilsCfg(EnetUtils_Cfg *cfg)
{
#if ENET_CFG_IS_ON(HAS_DEFAULT_UTILS)
    EnetUtilsDflt_initCfg(cfg);
#else
    memset(cfg, 0, sizeof(*cfg));
#endif
}

static void  EnetPer_saveCtxt(EnetPer_Handle hPer)
{
    Enet_devAssert(hPer->saveCtxt != NULL, "%s: Invalid saveCtxt function\n", hPer->name);

    ENETTRACE_VERBOSE("%s: Save and close peripheral\n", hPer->name);

    if (hPer->magic == ENET_MAGIC)
    {
        hPer->saveCtxt(hPer);
        hPer->magic = ENET_NO_MAGIC;
        ENETTRACE_VERBOSE("%s: Peripheral is now closed\n", hPer->name);
    }
}

static int32_t  EnetPer_restoreCtxt(EnetPer_Handle hPer,
                                    Enet_Type enetType,
                                    uint32_t instId)
{
    int32_t status;

    Enet_devAssert(hPer->open != NULL, "%s: Invalid restoreCtxt function\n", hPer->name);

    ENETTRACE_VERBOSE("%s: Restore and Open peripheral\n", hPer->name);

    if (hPer->magic == ENET_NO_MAGIC)
    {
        hPer->virtAddr  = (void *)EnetUtils_physToVirt(hPer->physAddr, NULL);
        hPer->virtAddr2 = (void *)EnetUtils_physToVirt(hPer->physAddr2, NULL);

        status = hPer->restoreCtxt(hPer, enetType, instId);
        if (status == ENET_SOK)
        {
            hPer->magic = ENET_MAGIC;
            ENETTRACE_VERBOSE("%s: Peripheral is now open\n", hPer->name);
        }
        else
        {
            ENETTRACE_ERR("%s: Failed to open: %d\n", hPer->name, status);
            hPer->magic = ENET_NO_MAGIC;
        }
    }
    else
    {
        ENETTRACE_ERR("%s: Peripheral is already open\n", hPer->name);
        status = ENET_EALREADYOPEN;
    }

    return status;
}

void Enet_initCfg(Enet_Type enetType,
                  uint32_t instId,
                  void *cfg,
                  uint32_t cfgSize)
{
    Enet_Handle hEnet = NULL;
    EnetPer_Handle hPer = NULL;
    int32_t status;

    status = Enet_getHandles(enetType, instId, &hEnet, &hPer);
    ENETTRACE_ERR_IF(status != ENET_SOK,
                     "Failed to initialize config for %u.%u: %d\n", enetType, instId, status);

    if (status == ENET_SOK)
    {
        EnetPer_initCfg(hPer, enetType, cfg, cfgSize);
    }
}

void Enet_init(const EnetOsal_Cfg *osalCfg,
               const EnetUtils_Cfg *utilsCfg)
{
    Enet_Handle hEnet;
    uint32_t count;
    uint32_t i;
#if ENET_CFG_IS_ON(HAS_DEFAULT_OSAL)
    EnetOsal_Cfg dfltOsalCfg;
#endif
#if ENET_CFG_IS_ON(HAS_DEFAULT_UTILS)
    EnetUtils_Cfg dfltUtilsCfg;
#endif

    /* If defaut OSAL and/or utils is enabled, use them in case
     * the application hasn't provided any */
#if ENET_CFG_IS_ON(HAS_DEFAULT_OSAL)
    if (osalCfg == NULL)
    {
        EnetOsalDflt_initCfg(&dfltOsalCfg);
        osalCfg = &dfltOsalCfg;
    }
#endif
#if ENET_CFG_IS_ON(HAS_DEFAULT_UTILS)
    if (utilsCfg == NULL)
    {
        EnetUtilsDflt_initCfg(&dfltUtilsCfg);
        utilsCfg = &dfltUtilsCfg;
    }
#endif

    EnetOsal_init(osalCfg);
    EnetUtils_init(utilsCfg);
    EnetSoc_init();

    /* Create top-level Enet locks for all peripherals in the SoC */
    count = EnetSoc_getEnetNum();
    for (i = 0U; i < count; i++)
    {
        hEnet = EnetSoc_getEnetHandleByIdx(i);
        if (hEnet != NULL)
        {
            hEnet->lock = EnetOsal_createMutex();
            ENETTRACE_ERR_IF(hEnet->lock == NULL,
                             "%s: Failed to create mutex\n", hEnet->enetPer->name);
        }
    }
}

void Enet_deinit(void)
{
    Enet_Handle hEnet;
    uint32_t count;
    uint32_t i;

    /* Destroy all top-level Enet locks */
    count = EnetSoc_getEnetNum();
    for (i = 0U; i < count; i++)
    {
        hEnet = EnetSoc_getEnetHandleByIdx(i);
        if (hEnet != NULL)
        {
            EnetOsal_deleteMutex(hEnet->lock);
            hEnet->lock = NULL;
        }
    }

    EnetSoc_deinit();
    EnetUtils_deinit();
    EnetOsal_deinit();
}

Enet_Handle Enet_getHandle(Enet_Type enetType,
                           uint32_t instId)
{
    Enet_Handle hEnet = NULL;
    bool isOpen = false;

    /* Get handle for the requested peripheral, but return NULL
     * if it hasn't been opened */
    hEnet = EnetSoc_getEnetHandle(enetType, instId);
    if (hEnet)
    {
        EnetOsal_lockMutex(hEnet->lock);
        isOpen = (hEnet->magic == ENET_MAGIC);
        EnetOsal_unlockMutex(hEnet->lock);
    }

    return isOpen ? hEnet : NULL;
}

uint32_t Enet_getMacPortMax(Enet_Type enetType,
                            uint32_t instId)
{
    return EnetSoc_getMacPortMax(enetType, instId);
}

Enet_Handle Enet_open(Enet_Type enetType,
                      uint32_t instId,
                      const void *cfg,
                      uint32_t cfgSize)
{
    Enet_Handle hEnet = NULL;
    EnetPer_Handle hPer = NULL;
    int32_t status = ENET_SOK;

    /* Get Enet and EnetPer handles */
    status = Enet_getHandles(enetType, instId, &hEnet, &hPer);
    ENETTRACE_ERR_IF(status != ENET_SOK,
                     "Failed get handles for %u.%u: %d\n", enetType, instId, status);

    if (status == ENET_SOK)
    {
        EnetOsal_lockMutex(hEnet->lock);

#if ENET_CFG_IS_ON(SANITY_CHECKS)
        /* Print enabled configurable features and applicable erratas */
        ENETTRACE_DBG("%s: features: 0x%08x\n", hPer->name, hPer->features);
        ENETTRACE_DBG("%s: errata  : 0x%08x\n", hPer->name, hPer->errata);
#endif

        /* Open the Enet peripheral */
        status = EnetPer_open(hPer, enetType, instId, cfg, cfgSize);
        ENETTRACE_ERR_IF(status != ENET_SOK,
                         "%s: Failed to open: %d\n", hPer->name, status);

        /* Set driver open state */
        hEnet->magic = (status == ENET_SOK) ? ENET_MAGIC : ENET_NO_MAGIC;

        EnetOsal_unlockMutex(hEnet->lock);
    }

    return (status == ENET_SOK) ? hEnet : NULL;
}

Enet_Handle Enet_rejoin(Enet_Type enetType,
                        uint32_t instId)
{
    Enet_Handle hEnet = NULL;
    EnetPer_Handle hPer = NULL;
    int32_t status = ENET_SOK;

    /* Get Enet and EnetPer handles */
    status = Enet_getHandles(enetType, instId, &hEnet, &hPer);
    ENETTRACE_ERR_IF(status != ENET_SOK,
                     "Failed get handles for %u.%u: %d\n", enetType, instId, status);

    if (status == ENET_SOK)
    {
        EnetOsal_lockMutex(hEnet->lock);

#if ENET_CFG_IS_ON(SANITY_CHECKS)
        /* Print enabled configurable features and applicable erratas */
        ENETTRACE_DBG("%s: features: 0x%08x\n", hPer->name, hPer->features);
        ENETTRACE_DBG("%s: errata  : 0x%08x\n", hPer->name, hPer->errata);
#endif

        /* Rejoin the Enet peripheral */
        status = EnetPer_rejoin(hPer, enetType, instId);
        ENETTRACE_ERR_IF(status != ENET_SOK,
                         "%s: Failed to join: %d\n", hPer->name, status);

        /* Set driver open state */
        hEnet->magic = (status == ENET_SOK) ? ENET_MAGIC : ENET_NO_MAGIC;

        EnetOsal_unlockMutex(hEnet->lock);
    }

    return (status == ENET_SOK) ? hEnet : NULL;
}

void Enet_close(Enet_Handle hEnet)
{
    EnetPer_Handle hPer = NULL;

    /* Close the Enet peripheral */
    if (hEnet != NULL)
    {
        EnetOsal_lockMutex(hEnet->lock);

        hPer = Enet_getPerHandle(hEnet);
        Enet_devAssert(hPer != NULL, "Invalid EnetPer handle\n");

        EnetPer_close(hPer);

        /* Set driver open state */
        hEnet->magic = ENET_NO_MAGIC;

        EnetOsal_unlockMutex(hEnet->lock);
    }
    else
    {
        ENETTRACE_ERR("Trying to close an invalid Enet handle, ignoring...\n");
    }
}

EnetDma_Handle Enet_getDmaHandle(Enet_Handle hEnet)
{
    EnetPer_Handle hPer = NULL;
    EnetDma_Handle hDma = NULL;

    if (hEnet != NULL)
    {
        EnetOsal_lockMutex(hEnet->lock);

        hPer = Enet_getPerHandle(hEnet);
        Enet_devAssert(hPer != NULL, "Invalid EnetPer handle\n");

        if (hPer->magic == ENET_MAGIC)
        {
            hDma = EnetSoc_getDmaHandle(hPer->enetType, hPer->instId);
        }
        else
        {
            ENETTRACE_ERR("%s: Peripheral is not open\n", hPer->name);
        }

        EnetOsal_unlockMutex(hEnet->lock);
    }

    return hDma;
}

int32_t Enet_ioctl(Enet_Handle hEnet,
                   uint32_t coreId,
                   uint32_t cmd,
                   Enet_IoctlPrms *prms)
{
    EnetPer_Handle hPer = NULL;
    int32_t status = ENET_SOK;

    /* Call IOCTL on the Enet peripheral */
    if (hEnet != NULL)
    {
        EnetOsal_lockMutex(hEnet->lock);

        hPer = Enet_getPerHandle(hEnet);
        Enet_devAssert(hPer != NULL, "Invalid EnetPer handle\n");

#if ENET_CFG_IS_ON(DEV_ERROR)
        status = Enet_validateGenericIoctl(cmd, prms);
        ENETTRACE_ERR_IF(status != ENET_SOK, "IOCTL params are not valid\n");
#endif

        if (status == ENET_SOK)
        {
            status = EnetPer_ioctl(hPer, cmd, prms);
            ENETTRACE_ERR_IF(status < ENET_SOK,
                             "%s: IOCTL 0x%08x failed: %d\n", hPer->name, cmd, status);
        }

        EnetOsal_unlockMutex(hEnet->lock);
    }
    else
    {
        ENETTRACE_ERR("Invalid Enet handle\n");
        status = ENET_EBADARGS;
    }

    return status;
}


void Enet_periodicTick(Enet_Handle hEnet)
{
    EnetPer_Handle hPer = NULL;

    /* Run the periodic tick */
    if (hEnet != NULL)
    {
        /* TODO: Need to make lock more granular */
        EnetOsal_lockMutex(hEnet->lock);

        hPer = Enet_getPerHandle(hEnet);
        Enet_devAssert(hPer != NULL, "Invalid EnetPer handle\n");

        EnetPer_periodicTick(hPer);

        /* TODO: Need to make lock more granular */
        EnetOsal_unlockMutex(hEnet->lock);
    }
    else
    {
        ENETTRACE_ERR("Periodic tick called on an invalid Enet\n");
    }
}

void Enet_poll(Enet_Handle hEnet,
               Enet_Event evt,
               const void *arg,
               uint32_t argSize)
{
    EnetPer_Handle hPer = NULL;

    /* Poll the Enet peripheral for requested events */
    if (hEnet != NULL)
    {
        EnetOsal_lockMutex(hEnet->lock);

        hPer = Enet_getPerHandle(hEnet);
        Enet_devAssert(hPer != NULL, "Invalid EnetPer handle\n");

        EnetPer_poll(hPer, evt, arg, argSize);

        EnetOsal_unlockMutex(hEnet->lock);
    }
    else
    {
        ENETTRACE_ERR("Invalid Enet handle\n");
    }
}

void Enet_registerEventCb(Enet_Handle hEnet,
                          Enet_Event evt,
                          uint32_t evtNum,
                          Enet_EventCallback evtCb,
                          void *evtCbArgs)
{
    EnetPer_Handle hPer = NULL;

    if (hEnet != NULL)
    {
        EnetOsal_lockMutex(hEnet->lock);

        hPer = Enet_getPerHandle(hEnet);
        Enet_devAssert(hPer != NULL, "Invalid EnetPer handle\n");

        EnetPer_registerEventCb(hPer, evt, evtNum, evtCb, evtCbArgs);

        EnetOsal_unlockMutex(hEnet->lock);
    }
    else
    {
        ENETTRACE_ERR("Invalid Enet handle\n");
    }
}

void Enet_unregisterEventCb(Enet_Handle hEnet,
                            Enet_Event evt,
                            uint32_t evtNum)
{
    EnetPer_Handle hPer = NULL;

    if (hEnet != NULL)
    {
        EnetOsal_lockMutex(hEnet->lock);

        hPer = Enet_getPerHandle(hEnet);
        Enet_devAssert(hPer != NULL, "Invalid EnetPer handle\n");

        EnetPer_unregisterEventCb(hPer, evt, evtNum);

        EnetOsal_unlockMutex(hEnet->lock);
    }
    else
    {
        ENETTRACE_ERR("Invalid Enet handle\n");
    }
}

static inline int32_t Enet_getHandles(Enet_Type enetType,
                                      uint32_t instId,
                                      Enet_Handle *hEnet,
                                      EnetPer_Handle *hPer)
{
    int32_t status = ENET_SOK;

    /* Get handle for the requested Enet peripheral */
    *hEnet = EnetSoc_getEnetHandle(enetType, instId);
    if (*hEnet != NULL)
    {
        *hPer = Enet_getPerHandle(*hEnet);
        Enet_devAssert(*hPer != NULL,
                       "Invalid EnetPer handle for %u.%u\n", enetType, instId);
    }
    else
    {
        ENETTRACE_ERR("No EnetPer %u:%u has been found\n", enetType, instId);
        status = ENET_ENOTFOUND;
        *hPer = NULL;
    }

    return status;
}

static void EnetPer_initCfg(EnetPer_Handle hPer,
                            Enet_Type enetType,
                            void *cfg,
                            uint32_t cfgSize)
{
    /* intCfg() function is mandatory */
    Enet_devAssert(hPer->initCfg != NULL, "%s: Invalid initCfg function\n", hPer->name);

    hPer->initCfg(hPer, enetType, cfg, cfgSize);
}

static int32_t EnetPer_open(EnetPer_Handle hPer,
                            Enet_Type enetType,
                            uint32_t instId,
                            const void *cfg,
                            uint32_t cfgSize)
{
    int32_t status;

    /* open() function is mandatory */
    Enet_devAssert(hPer->open != NULL, "%s: Invalid open function\n", hPer->name);

    ENETTRACE_VERBOSE("%s: Open peripheral\n", hPer->name);

    if (hPer->magic == ENET_NO_MAGIC)
    {
        hPer->virtAddr  = (void *)EnetUtils_physToVirt(hPer->physAddr, NULL);
        hPer->virtAddr2 = (void *)EnetUtils_physToVirt(hPer->physAddr2, NULL);

        status = hPer->open(hPer, enetType, instId, cfg, cfgSize);
        if (status == ENET_SOK)
        {
            hPer->magic = ENET_MAGIC;
            ENETTRACE_VERBOSE("%s: Peripheral is now open\n", hPer->name);
        }
        else
        {
            ENETTRACE_ERR("%s: Failed to open: %d\n", hPer->name, status);
            hPer->magic = ENET_NO_MAGIC;
        }
    }
    else
    {
        ENETTRACE_ERR("%s: Peripheral is already open\n", hPer->name);
        status = ENET_EALREADYOPEN;
    }

    return status;
}

static int32_t EnetPer_rejoin(EnetPer_Handle hPer,
                              Enet_Type enetType,
                              uint32_t instId)
{
    int32_t status = ENET_ENOTSUPPORTED;

    /* rejoin() function is optional */
    if (hPer->rejoin != NULL)
    {
        ENETTRACE_VERBOSE("%s: Rejoin peripheral\n", hPer->name);

        if (hPer->magic == ENET_NO_MAGIC)
        {
            hPer->virtAddr  = (void *)EnetUtils_physToVirt(hPer->physAddr, NULL);
            hPer->virtAddr2 = (void *)EnetUtils_physToVirt(hPer->physAddr2, NULL);

            status = hPer->rejoin(hPer, enetType, instId);
            if (status == ENET_SOK)
            {
                hPer->magic = ENET_MAGIC;
                ENETTRACE_VERBOSE("%s: Peripheral has now joined\n", hPer->name);
            }
            else
            {
                ENETTRACE_ERR("%s: Failed to open: %d\n", hPer->name, status);
                hPer->magic = ENET_NO_MAGIC;
            }
        }
        else
        {
            ENETTRACE_ERR("%s: Peripheral is already open\n", hPer->name);
            status = ENET_EALREADYOPEN;
        }
    }

    return status;
}

int32_t Enet_saveCtxt(Enet_Handle hEnet)
{
    EnetPer_Handle hPer = NULL;
    int32_t status = ENET_SOK;

    /* Close the Enet peripheral */
    if (hEnet != NULL)
    {
        EnetOsal_lockMutex(hEnet->lock);

        hPer = Enet_getPerHandle(hEnet);
        Enet_devAssert(hPer != NULL, "Invalid EnetPer handle\n");

        EnetPer_saveCtxt(hPer);

        /* Set driver open state */
        hEnet->magic = ENET_NO_MAGIC;

        EnetOsal_unlockMutex(hEnet->lock);
    }
    else
    {
        status = ENET_EFAIL;
        ENETTRACE_ERR("Trying to close an invalid Enet handle, ignoring...\n");
    }

    return status;
}

int32_t Enet_restoreCtxt(Enet_Type enetType,
                             uint32_t instId)
{
    Enet_Handle hEnet = NULL;
    EnetPer_Handle hPer = NULL;
    int32_t status = ENET_SOK;

    /* Get Enet and EnetPer handles */
    status = Enet_getHandles(enetType, instId, &hEnet, &hPer);
    ENETTRACE_ERR_IF(status != ENET_SOK,
                     "Failed get handles for %u.%u: %d\n", enetType, instId, status);

    if (status == ENET_SOK)
    {
        EnetOsal_lockMutex(hEnet->lock);

#if ENET_CFG_IS_ON(SANITY_CHECKS)
        /* Print enabled configurable features and applicable erratas */
        ENETTRACE_DBG("%s: features: 0x%08x\n", hPer->name, hPer->features);
        ENETTRACE_DBG("%s: errata  : 0x%08x\n", hPer->name, hPer->errata);
#endif

        /* Open the Enet peripheral */
        status = EnetPer_restoreCtxt(hPer, enetType, instId);
        ENETTRACE_ERR_IF(status != ENET_SOK,
                         "%s: Failed to open: %d\n", hPer->name, status);

        /* Set driver open state */
        hEnet->magic = (status == ENET_SOK) ? ENET_MAGIC : ENET_NO_MAGIC;

        EnetOsal_unlockMutex(hEnet->lock);
    }

    if(hEnet == NULL)
    {
        status = ENET_EFAIL;
    }

    return status;
}

int32_t Enet_hardResetCpsw(Enet_Handle hEnet, Enet_Type enetType, uint32_t instId, Enet_notify_t *pCpswTriggerResetCb)
{
    int32_t status = ENET_SOK;

    /* Close the Enet peripheral */
    if (hEnet != NULL)
    {
        status = Enet_saveCtxt(hEnet);

        if(status == ENET_SOK)
        {
            if(pCpswTriggerResetCb->cbFxn != NULL)
            {
                /* CPSW hard reset*/
                pCpswTriggerResetCb->cbFxn(pCpswTriggerResetCb->cbArg);
            }
        }

        /* Restoring Enet handle and opening it*/
        status = Enet_restoreCtxt(enetType, instId);
        ENETTRACE_ERR_IF(status != ENET_SOK,
                         "Failed to Reset CPSW peripheral:%d\r\n", status);
    }

    return status;
}
static int32_t EnetPer_ioctl(EnetPer_Handle hPer,
                             uint32_t cmd,
                             Enet_IoctlPrms *prms)
{
    int32_t status = ENET_EFAIL;

    /* ioctl() function is mandatory */
    Enet_devAssert(hPer->ioctl != NULL, "%s: Invalid ioctl function\n", hPer->name);

    ENETTRACE_VERBOSE("%s: Do IOCTL 0x%08x prms %p\n", hPer->name, cmd, prms);

    if (hPer->magic == ENET_MAGIC)
    {
        status = hPer->ioctl(hPer, cmd, prms);
        if (status < ENET_SOK)
        {
            ENETTRACE_ERR("%s: Failed to do IOCTL cmd 0x%08x: %d\n", hPer->name, cmd, status);
        }
    }
    else
    {
        ENETTRACE_ERR("%s: Peripheral is not open\n", hPer->name);
    }

    return status;
}

static void EnetPer_periodicTick(EnetPer_Handle hPer)
{
    /* periodicTick() function is mandatory */
    Enet_devAssert(hPer->periodicTick != NULL, "%s: Invalid periodic tick function\n", hPer->name);

    ENETTRACE_VERBOSE("%s: Do periodic tick\n", hPer->name);

    if (hPer->magic == ENET_MAGIC)
    {
        hPer->periodicTick(hPer);
    }
    else
    {
        ENETTRACE_ERR("%s: Peripheral is not open\n", hPer->name);
    }
}

static void EnetPer_registerEventCb(EnetPer_Handle hPer,
                                    Enet_Event evt,
                                    uint32_t evtNum,
                                    Enet_EventCallback evtCb,
                                    void *evtCbArgs)
{
    if (hPer->registerEventCb != NULL)
    {
        ENETTRACE_VERBOSE("%s: Register callback for event %u.%u\n", hPer->name, evt, evtNum);

        if (hPer->magic == ENET_MAGIC)
        {
            hPer->registerEventCb(hPer, evt, evtNum, evtCb, evtCbArgs);
        }
        else
        {
            ENETTRACE_ERR("%s: Peripheral is not open\n", hPer->name);
        }
    }
    else
    {
        ENETTRACE_WARN("%s: Enet events not supported by peripheral\n", hPer->name);
    }
}

static void EnetPer_unregisterEventCb(EnetPer_Handle hPer,
                                      Enet_Event evt,
                                      uint32_t evtNum)
{
    if (hPer->unregisterEventCb != NULL)
    {
        ENETTRACE_VERBOSE("%s: Unregister callback for event %u.%u\n", hPer->name, evt, evtNum);

        if (hPer->magic == ENET_MAGIC)
        {
            hPer->unregisterEventCb(hPer, evt, evtNum);
        }
        else
        {
            ENETTRACE_ERR("%s: Peripheral is not open\n", hPer->name);
        }
    }
    else
    {
        ENETTRACE_WARN("%s: Enet events not supported by peripheral\n", hPer->name);
    }
}

static void EnetPer_poll(EnetPer_Handle hPer,
                         Enet_Event evt,
                         const void *arg,
                         uint32_t argSize)
{
    /* poll() function is optional */
    if (hPer->poll != NULL)
    {
        ENETTRACE_VERBOSE("%s: Poll peripheral for event %u\n", hPer->name, evt);

        if (hPer->magic == ENET_MAGIC)
        {
            hPer->poll(hPer, evt, arg, argSize);
        }
        else
        {
            ENETTRACE_ERR("%s: Peripheral is not open\n", hPer->name);
        }
    }
    else
    {
        ENETTRACE_WARN("%s: Poll function is not supported\n", hPer->name);
    }
}

static void EnetPer_close(EnetPer_Handle hPer)
{
    /* close() function is mandatory */
    Enet_devAssert(hPer->close != NULL, "%s: Invalid close function\n", hPer->name);

    ENETTRACE_VERBOSE("%s: Close peripheral\n", hPer->name);

    if (hPer->magic == ENET_MAGIC)
    {
        hPer->close(hPer);
        hPer->magic = ENET_NO_MAGIC;
        ENETTRACE_VERBOSE("%s: Peripheral is now closed\n", hPer->name);
    }
}

int32_t EnetMod_open(EnetMod_Handle hMod,
                     Enet_Type enetType,
                     uint32_t instId,
                     const void *cfg,
                     uint32_t cfgSize)
{
    int32_t status;

    /* open() function is mandatory */
    Enet_devAssert(hMod->open != NULL, "%s: Invalid open function\n", hMod->name);

    ENETTRACE_VERBOSE("%s: Open module\n", hMod->name);

    if (hMod->magic == ENET_NO_MAGIC)
    {
        hMod->virtAddr  = (void *)EnetUtils_physToVirt(hMod->physAddr, NULL);
        hMod->virtAddr2 = (void *)EnetUtils_physToVirt(hMod->physAddr2, NULL);

        status = hMod->open(hMod, enetType, instId, cfg, cfgSize);
        if (status == ENET_SOK)
        {
            hMod->magic = ENET_MAGIC;
            ENETTRACE_VERBOSE("%s: Module is now open\n", hMod->name);
        }
        else
        {
            ENETTRACE_ERR("%s: Failed to open: %d\n", hMod->name, status);
            hMod->magic = ENET_NO_MAGIC;
        }
    }
    else
    {
        ENETTRACE_ERR("%s: Module is already open\n", hMod->name);
        status = ENET_EALREADYOPEN;
    }

    return status;
}

int32_t EnetMod_rejoin(EnetMod_Handle hMod,
                       Enet_Type enetType,
                       uint32_t instId)
{
    int32_t status = ENET_ENOTSUPPORTED;

    /* rejoin() function is optional */
    if (hMod->rejoin != NULL)
    {
        ENETTRACE_VERBOSE("%s: Rejoin module\n", hMod->name);

        if (hMod->magic == ENET_NO_MAGIC)
        {
            hMod->virtAddr  = (void *)EnetUtils_physToVirt(hMod->physAddr, NULL);
            hMod->virtAddr2 = (void *)EnetUtils_physToVirt(hMod->physAddr2, NULL);

            status = hMod->rejoin(hMod, enetType, instId);
            if (status == ENET_SOK)
            {
                hMod->magic = ENET_MAGIC;
                ENETTRACE_VERBOSE("%s: Module has now joined\n", hMod->name);
            }
            else
            {
                ENETTRACE_ERR("%s: Failed to open: %d\n", hMod->name, status);
                hMod->magic = ENET_NO_MAGIC;
            }
        }
        else
        {
            ENETTRACE_ERR("%s: Module is already open\n", hMod->name);
            status = ENET_EALREADYOPEN;
        }
    }

    return status;
}

int32_t EnetMod_ioctl(EnetMod_Handle hMod,
                      uint32_t cmd,
                      Enet_IoctlPrms *prms)
{
    int32_t status = ENET_EFAIL;

    /* ioctl() function is mandatory */
    Enet_devAssert(hMod->ioctl != NULL, "%s: Invalid ioctl function\n", hMod->name);

    ENETTRACE_VERBOSE("%s: Do IOCTL 0x%08x prms %p\n", hMod->name, cmd, prms);

    if (hMod->magic == ENET_MAGIC)
    {
        status = hMod->ioctl(hMod, cmd, prms);
        if (status != ENET_SOK)
        {
            ENETTRACE_ERR("%s: Failed to do IOCTL cmd 0x%08x: %d\n", hMod->name, cmd, status);
        }
    }
    else
    {
        ENETTRACE_ERR("%s: Module is not open\n", hMod->name);
    }

    return status;
}

int32_t EnetMod_registerMacportIoctlHandler(EnetMod_Handle hMod,
                                            uint32_t cmdBase,
                                            uint32_t cmd,
                                            Enet_IoctlPrms *prms)
{
    int32_t status = ENET_EFAIL;

    /* ioctl() function is mandatory */
    Enet_devAssert(hMod->ioctl != NULL, "%s: Invalid ioctl function\n", hMod->name);

    ENETTRACE_VERBOSE("%s: Do IOCTL 0x%08x prms %p\n", hMod->name, cmd, prms);

    if(cmdBase == ENET_IOCTL_MACPORT_BASE)
    {
        status = hMod->ioctl(hMod, cmd, prms);
        if (status != ENET_SOK)
        {
            ENETTRACE_ERR("%s: Failed to do IOCTL cmd 0x%08x: %d\n", hMod->name, cmd, status);
        }
    }

    return status;
}

int32_t EnetMod_ioctlFromIsr(EnetMod_Handle hMod,
                             uint32_t cmd,
                             Enet_IoctlPrms *prms)
{
    int32_t status = ENET_EFAIL;

    if (hMod->magic == ENET_MAGIC)
    {
        status = hMod->ioctl(hMod, cmd, prms);
    }

    return status;
}

void EnetMod_close(EnetMod_Handle hMod)
{
    /* close() function is mandatory */
    Enet_devAssert(hMod->close != NULL, "%s: Invalid close function\n", hMod->name);

    ENETTRACE_VERBOSE("%s: Close module\n", hMod->name);

    if (hMod->magic == ENET_MAGIC)
    {
        hMod->close(hMod);
        hMod->magic = ENET_NO_MAGIC;
        ENETTRACE_VERBOSE("%s: Module is now closed\n", hMod->name);
    }
    else
    {
        ENETTRACE_ERR("%s: Module is not open\n", hMod->name);
    }
}

void EnetMod_saveCtxt(EnetMod_Handle hMod)
{
    /* close() function is mandatory */
    Enet_devAssert(hMod->close != NULL, "%s: Invalid close function\n", hMod->name);

    ENETTRACE_VERBOSE("%s: Close module\n", hMod->name);

    if (hMod->magic == ENET_MAGIC)
    {
        hMod->close(hMod);
        hMod->magic = ENET_NO_MAGIC;
        ENETTRACE_VERBOSE("%s: Module is now closed\n", hMod->name);
    }
    else
    {
        ENETTRACE_ERR("%s: Module is not open\n", hMod->name);
    }
}

int32_t EnetMod_restoreCtxt(EnetMod_Handle hMod,
                            Enet_Type enetType,
                            uint32_t instId,
                            const void *cfg,
                           uint32_t cfgSize)
{
    int32_t status;

    /* open() function is mandatory */
    Enet_devAssert(hMod->open != NULL, "%s: Invalid open function\n", hMod->name);

    ENETTRACE_VERBOSE("%s: Open module\n", hMod->name);

    if (hMod->magic == ENET_NO_MAGIC)
    {
        hMod->virtAddr  = (void *)EnetUtils_physToVirt(hMod->physAddr, NULL);
        hMod->virtAddr2 = (void *)EnetUtils_physToVirt(hMod->physAddr2, NULL);

        status = hMod->restoreCtxt(hMod, enetType, instId, cfg, cfgSize);
        if (status == ENET_SOK)
        {
            hMod->magic = ENET_MAGIC;
            ENETTRACE_VERBOSE("%s: Module is now open\n", hMod->name);
        }
        else
        {
            ENETTRACE_ERR("%s: Failed to open: %d\n", hMod->name, status);
            hMod->magic = ENET_NO_MAGIC;
        }
    }
    else
    {
        ENETTRACE_ERR("%s: Module is already open\n", hMod->name);
        status = ENET_EALREADYOPEN;
    }

    return status;
}

#if ENET_CFG_IS_ON(DEV_ERROR)
int32_t Enet_validateIoctl(uint32_t cmd,
                           const Enet_IoctlPrms *prms,
                           const Enet_IoctlValidate *validIoctls,
                           uint32_t numValidIoctls)
{
    const Enet_IoctlValidate *ioctlInfo;
    uint32_t idx = ENET_IOCTL_GET_MIN(cmd);
    int32_t status = ENET_SOK;

    if (idx >= numValidIoctls)
    {
        status = ENET_EINVALIDPARAMS;
    }
    else if (prms == NULL)
    {
        status = ENET_EMALFORMEDIOCTL;
    }
    else
    {
        ioctlInfo = &validIoctls[idx];
        if (ioctlInfo->cmd != cmd)
        {
            status = ENET_EMALFORMEDIOCTL;
        }

        if (status == ENET_SOK)
        {
            if ((prms->inArgsSize != ioctlInfo->inArgsSize) ||
                ((prms->inArgsSize != 0U) && (prms->inArgs == NULL)) ||
                ((prms->inArgsSize == 0U) && (prms->inArgs != NULL)))
            {
                status = ENET_EMALFORMEDIOCTL;
            }

            if ((prms->outArgsSize != ioctlInfo->outArgsSize) ||
                ((prms->outArgsSize != 0U) && (prms->outArgs == NULL)) ||
                ((prms->outArgsSize == 0U) && (prms->outArgs != NULL)))
            {
                status = ENET_EMALFORMEDIOCTL;
            }
        }
    }

    ENETTRACE_ERR_IF(status != ENET_SOK,
                     "Malformed IOCTL cmd 0x%08x prms %p inArgs %p / %u outArg %p / %u\n",
                     cmd, prms,
                     (prms != NULL) ? prms->inArgs : NULL,
                     (prms != NULL) ? prms->inArgsSize : 0U,
                     (prms != NULL) ? prms->outArgs : NULL,
                     (prms != NULL) ? prms->outArgsSize : 0U);

    Enet_devAssert(status == ENET_SOK, "Malformed IOCTL\n");

    return status;
}

int32_t Enet_validateGenericIoctl(uint32_t cmd,
                                  const Enet_IoctlPrms *prms)
{
    const Enet_IoctlValidate *validIoctls;
    uint32_t numValidIoctls;
    uint32_t major = ENET_IOCTL_GET_MAJ(cmd);
    bool deferValidation = false;
    int32_t status = ENET_SOK;

    if ((ENET_IOCTL_GET_PER(cmd) == ENET_IOCTL_PER_GENERIC) &&
        (ENET_IOCTL_GET_TYPE(cmd) == ENET_IOCTL_TYPE_PUBLIC))
    {
        switch (major)
        {
            case ENET_IOCTL_PER_BASE:
                validIoctls    = gEnetPer_ioctlValidate;
                numValidIoctls = ENET_ARRAYSIZE(gEnetPer_ioctlValidate);
                break;

            case ENET_IOCTL_FDB_BASE:
                validIoctls    = gEnetFdb_ioctlValidate;
                numValidIoctls = ENET_ARRAYSIZE(gEnetFdb_ioctlValidate);
                break;

            case ENET_IOCTL_TIMESYNC_BASE:
                validIoctls    = gEnetTimeSync_ioctlValidate;
                numValidIoctls = ENET_ARRAYSIZE(gEnetTimeSync_ioctlValidate);
                break;

            case ENET_IOCTL_TAS_BASE:
                validIoctls    = gEnetTas_ioctlValidate;
                numValidIoctls = ENET_ARRAYSIZE(gEnetTas_ioctlValidate);
                break;

            case ENET_IOCTL_HOSTPORT_BASE:
                validIoctls    = gEnetHostPort_ioctlValidate;
                numValidIoctls = ENET_ARRAYSIZE(gEnetHostPort_ioctlValidate);
                break;

            case ENET_IOCTL_MACPORT_BASE:
                validIoctls    = gEnetMacPort_ioctlValidate;
                numValidIoctls = ENET_ARRAYSIZE(gEnetMacPort_ioctlValidate);
                break;

            case ENET_IOCTL_MDIO_BASE:
                validIoctls    = gEnetMdio_ioctlValidate;
                numValidIoctls = ENET_ARRAYSIZE(gEnetMdio_ioctlValidate);
                break;

            case ENET_IOCTL_STATS_BASE:
                validIoctls    = gEnetStats_ioctlValidate;
                numValidIoctls = ENET_ARRAYSIZE(gEnetStats_ioctlValidate);
                break;

            case ENET_IOCTL_PHY_BASE:
                validIoctls    = gEnetPhy_ioctlValidate;
                numValidIoctls = ENET_ARRAYSIZE(gEnetPhy_ioctlValidate);
                break;

            case ENET_IOCTL_RM_BASE:
                /* RM is a generic module implemented by Enet core layer. It has
                 * public and private IOCTLs.
                 * This validation function checks public IOCTLs, not private ones.
                 * Hence for RM the best place to check params is in the Enet RM
                 * module itself, so deferring the validation */
                deferValidation = true;
                break;

            default:
                Enet_devAssert(false, "Invalid IOCTL major number %u\n", major);
                status = ENET_EMALFORMEDIOCTL;
                break;
        }

        if ((status == ENET_SOK) && !deferValidation)
        {
            switch (cmd)
            {
                /*
                 * Note: The get/reset host and MAC port statistics listed below
                 * must be checked by each module as the size of the statistics
                 * block is hardware dependent.
                 */
                case ENET_STATS_IOCTL_GET_HOSTPORT_STATS:
                case ENET_STATS_IOCTL_GET_MACPORT_STATS:
                case ENET_STATS_IOCTL_RESET_HOSTPORT_STATS:
                case ENET_STATS_IOCTL_RESET_MACPORT_STATS:
                    status = ENET_SOK;
                    break;

                default:
#if ENET_CFG_IS_ON(DEV_ERROR)
                    status = Enet_validateIoctl(cmd, prms, validIoctls, numValidIoctls);
#endif
                    break;
            }
        }
    }

    return status;
}
#endif

int32_t Enet_getHandleInfo(Enet_Handle hEnet,
                           Enet_Type *enetType,
                           uint32_t *instId)
{
    bool isOpen = false;
    int32_t status = ENET_EBADARGS;

    if (hEnet)
    {
        EnetOsal_lockMutex(hEnet->lock);
        isOpen = (hEnet->magic == ENET_MAGIC);
        if (isOpen)
        {
            *enetType = hEnet->enetPer->enetType;
            *instId = hEnet->enetPer->instId;
            status = ENET_SOK;
        }
        EnetOsal_unlockMutex(hEnet->lock);
    }

    return status;
}
