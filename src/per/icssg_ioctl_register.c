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
#define ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(x)                                        \
int32_t Enet_ioctl_register_##x(Enet_Handle hEnet, uint32_t coreId)                   \
                                                                                      \
{                                                                                     \
    int32_t  status;                                                                  \
    Enet_IoctlPrms prms;                                                              \
    Enet_IoctlRegisterHandlerInArgs inArgs;                                           \
                                                                                      \
    inArgs.cmd = x;                                                                   \
    inArgs.fxn = (uintptr_t)&Icssg_ioctl_handler_##x;                                  \
                                                                                      \
    ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);                                           \
    status = Enet_ioctl(hEnet, coreId, ENET_PER_IOCTL_REGISTER_IOCTL_HANDLER, &prms); \
    return  status;                                                                   \
                                                                                      \
}
#define ICSSG_MACPORT_GEN_REGISTER_IOCTL_HANDLER_FXN(x)                                        \
int32_t Enet_ioctl_register_##x(Enet_Handle hEnet, uint32_t coreId)                   \
                                                                                      \
{                                                                                     \
    int32_t  status;                                                                  \
    Enet_IoctlPrms prms;                                                              \
    Enet_IoctlRegisterHandlerInArgs inArgs;                                           \
                                                                                      \
    inArgs.cmd = x;                                                                   \
    inArgs.fxn = (uintptr_t)&IcssgMacPort_ioctl_handler_##x;                                  \
                                                                                      \
    ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);                                           \
    status = Enet_ioctl(hEnet, coreId, ENET_PER_IOCTL_REGISTER_IOCTL_HANDLER, &prms); \
    return  status;                                                                   \
                                                                                      \
}
/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */



/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_PER_IOCTL_OPEN_PORT_LINK)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_PER_IOCTL_CLOSE_PORT_LINK)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_PER_IOCTL_IS_PORT_LINK_UP)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_PER_IOCTL_GET_PORT_LINK_CFG)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ICSSG_MACPORT_IOCTL_ENABLE_PROMISC_MODE)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ICSSG_MACPORT_IOCTL_DISABLE_PROMISC_MODE)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ICSSG_MACPORT_IOCTL_ENABLE_UCAST_FLOOD)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ICSSG_MACPORT_IOCTL_DISABLE_UCAST_FLOOD)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ICSSG_MACPORT_IOCTL_ENABLE_MCAST_FLOOD)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ICSSG_MACPORT_IOCTL_DISABLE_MCAST_FLOOD)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ICSSG_PER_IOCTL_VLAN_RESET_TABLE)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ICSSG_PER_IOCTL_VLAN_SET_ENTRY)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ICSSG_PER_IOCTL_VLAN_GET_ENTRY)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ICSSG_PER_IOCTL_VLAN_SET_HOSTPORT_DFLT_VID)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ICSSG_PER_IOCTL_VLAN_SET_MACPORT_DFLT_VID)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ICSSG_PER_IOCTL_SET_PORT_STATE)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ICSSG_FDB_IOCTL_ADD_ENTRY)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ICSSG_FDB_IOCTL_REMOVE_ENTRY)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ICSSG_FDB_IOCTL_REMOVE_ALL_ENTRIES)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ICSSG_FDB_IOCTL_REMOVE_AGEABLE_ENTRIES)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ICSSG_MACPORT_IOCTL_SET_MACADDR)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ICSSG_HOSTPORT_IOCTL_SET_MACADDR)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_MACPORT_IOCTL_ENABLE_PREEMPTION)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_MACPORT_IOCTL_DISABLE_PREEMPTION)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_MACPORT_IOCTL_GET_PREEMPTION_ENABLE_STATUS)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_MACPORT_IOCTL_GET_PREEMPTION_ACTIVE_STATUS)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_MACPORT_IOCTL_ENABLE_PREEMPT_VERIFICATION)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_MACPORT_IOCTL_DISABLE_PREEMPT_VERIFICATION)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_MACPORT_IOCTL_GET_PREEMPT_VERIFY_STATUS)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_MACPORT_IOCTL_GET_PREEMPT_MIN_FRAG_SIZE)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_MACPORT_IOCTL_SET_PREEMPT_MIN_FRAG_SIZE)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_MACPORT_IOCTL_SET_PREEMPT_QUEUE)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_MACPORT_IOCTL_GET_QUEUE_PREEMPT_STATUS)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_IOCTL_REGISTER_RX_DEFAULT_FLOW)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_IOCTL_UNREGISTER_RX_DEFAULT_FLOW)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_PER_IOCTL_ATTACH_CORE)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_PER_IOCTL_DETACH_CORE)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ICSSG_PER_IOCTL_TAS_TRIGGER)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ICSSG_PER_IOCTL_TAS_ENABLE)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ICSSG_PER_IOCTL_TAS_DISABLE)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ICSSG_PER_IOCTL_TAS_RESET)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ICSSG_FDB_IOCTL_SET_AGING_PERIOD)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ICSSG_MACPORT_IOCTL_SET_ACCEPT_FRAME_CHECK)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ICSSG_MACPORT_IOCTL_SET_INGRESS_RATE_LIM)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_PER_IOCTL_SET_VLAN_AWARE)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_PER_IOCTL_SET_VLAN_UNAWARE)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ICSSG_MACPORT_IOCTL_SET_QUEUE_CUT_THROUGH_PREEMPT_SELECT)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ICSSG_MACPORT_IOCTL_CONFIG_SPL_FRAME_PRIO)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_PER_IOCTL_HANDLE_EXTPHY_LINKUP_EVENT)
ICSSG_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_PER_IOCTL_HANDLE_EXTPHY_LINKDOWN_EVENT)
ICSSG_MACPORT_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_MACPORT_IOCTL_SET_EGRESS_QOS_PRI_MAP)
ICSSG_MACPORT_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_MACPORT_IOCTL_SET_PRI_REGEN_MAP)
ICSSG_MACPORT_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_MACPORT_IOCTL_SET_INGRESS_DSCP_PRI_MAP)
ICSSG_MACPORT_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_MACPORT_IOCTL_GET_EGRESS_QOS_PRI_MAP)
ICSSG_MACPORT_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_MACPORT_IOCTL_GET_PRI_REGEN_MAP)
ICSSG_MACPORT_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_MACPORT_IOCTL_GET_INGRESS_DSCP_PRI_MAP)
