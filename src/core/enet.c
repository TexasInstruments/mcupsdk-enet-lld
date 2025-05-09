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
                          sizeof(uint32_t),
                          sizeof(uint64_t)),
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
                          sizeof(EnetMacPort_SetCreditBasedShaperInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_MACPORT_IOCTL_GET_CREDIT_BASED_SHAPING,
                          sizeof(EnetMacPort_GetCreditBasedShaperInArgs),
                          sizeof(uint64_t)),

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

    ENET_IOCTL_VALID_PRMS(ENET_PHY_IOCTL_ADJ_PTP_FREQ,
                          sizeof(EnetPhy_AdjPtpFreqInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_PHY_IOCTL_ADJ_PTP_PHASE,
                          sizeof(EnetPhy_AdjPtpPhaseInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_PHY_IOCTL_GET_PTP_TIME,
                          sizeof(EnetPhy_GenericInArgs),
                          sizeof(uint64_t)),

    ENET_IOCTL_VALID_PRMS(ENET_PHY_IOCTL_SET_PTP_TIME,
                          sizeof(EnetPhy_SetPtpTimeInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_PHY_IOCTL_GET_PTP_TXTS,
                          sizeof(EnetPhy_PtpPktTimestampInArgs),
                          sizeof(uint64_t)),

    ENET_IOCTL_VALID_PRMS(ENET_PHY_IOCTL_GET_PTP_RXTS,
                          sizeof(EnetPhy_PtpPktTimestampInArgs),
                          sizeof(uint64_t)),

    ENET_IOCTL_VALID_PRMS(ENET_PHY_IOCTL_WAIT_PTP_TXTS,
                          sizeof(EnetPhy_PtpPktTimestampInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_PHY_IOCTL_PROC_STATUS_FRAME,
                          sizeof(EnetPhy_ProcStatusFrameInArgs),
                          sizeof(EnetPhy_ProcStatusFrameOutArgs)),

    ENET_IOCTL_VALID_PRMS(ENET_PHY_IOCTL_GET_STATUS_FRAME_ETHDR,
                          sizeof(EnetPhy_GenericInArgs),
                          sizeof(EnetPhy_GetStatusFrameEthdrOutArgs)),

    ENET_IOCTL_VALID_PRMS(ENET_PHY_IOCTL_ENABLE_PTP,
                          sizeof(EnetPhy_EnablePtpInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_PHY_IOCTL_ENABLE_EVENT_CAPTURE,
                          sizeof(EnetPhy_EnableEventCaptureInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_PHY_IOCTL_ENABLE_TRIGGER_OUTPUT,
                          sizeof(EnetPhy_EnableTriggerOutputInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_PHY_IOCTL_GET_EVENT_TIMESTAMP,
                          sizeof(EnetPhy_GenericInArgs),
                          sizeof(EnetPhy_GetEventTimestampOutArgs)),
};
#endif

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

inline Enet_Type Enet_getEnetType(uint32_t hEnet)
{
    return ((hEnet & 0xFFFF0000U) >> 16);
}

inline uint32_t Enet_getInstId(uint32_t hEnet)
{
    return (hEnet & 0x0000FFFFU);
}

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

void Enet_initUtilsCfg(EnetUtils_Cfg *cfg)
{
#if ENET_CFG_IS_ON(HAS_DEFAULT_UTILS)
    EnetUtilsDflt_initCfg(cfg);
#else
    memset(cfg, 0, sizeof(*cfg));
#endif
}

uint32_t Enet_getMacPortMax(Enet_Type enetType,
                            uint32_t instId)
{
    return EnetSoc_getMacPortMax(enetType, instId);
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
