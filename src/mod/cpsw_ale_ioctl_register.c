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
 * \file  cpsw_ale_ioctl.c
 *
 * \brief This file contains the implementation of the CPSW ALE IOCTLs
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdarg.h>
#include <csl_cpswitch.h>
#include <enet.h>
#include <enet_cfg.h>
#include <include/core/enet_utils.h>
#include <include/core/enet_soc.h>
#include <include/mod/cpsw_ale.h>
#include <priv/core/enet_base_priv.h>
#include <priv/core/enet_trace_priv.h>
#include <priv/mod/cpsw_ale_priv.h>
#include <priv/mod/cpsw_ale_ioctl_priv.h>
#include <priv/mod/cpsw_clks.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */


/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */


/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
#define CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(x)                                    \
int32_t Enet_ioctl_register_##x(Enet_Handle hEnet, uint32_t coreId)                   \
                                                                                      \
{                                                                                     \
    int32_t  status;                                                                  \
    Enet_IoctlPrms prms;                                                              \
    Enet_IoctlRegisterHandlerInArgs inArgs;                                           \
                                                                                      \
    inArgs.cmd = x;                                                                   \
    inArgs.fxn = (uintptr_t)&CpswAle_ioctl_handler_##x;                               \
                                                                                      \
    ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);                                           \
    status = Enet_ioctl(hEnet, coreId, ENET_PER_IOCTL_REGISTER_IOCTL_HANDLER, &prms); \
    return  status;                                                                   \
                                                                                      \
}


CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_FDB_IOCTL_GET_VERSION)
#if ENET_CFG_IS_ON(DEV_ERROR)
CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_FDB_IOCTL_PRINT_REGS)
CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_ALE_IOCTL_DUMP_TABLE)
#endif
CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_ALE_IOCTL_ADD_UCAST)
CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_ALE_IOCTL_ADD_MCAST)
CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_ALE_IOCTL_ADD_VLAN)
CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_ALE_IOCTL_ADD_OUI)
CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_ALE_IOCTL_ADD_IPV4ADDR)
CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_ALE_IOCTL_ADD_IPV6ADDR)
CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_ALE_IOCTL_ADD_ETHERTYPE)
CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_ALE_IOCTL_LOOKUP_UCAST)
CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_ALE_IOCTL_LOOKUP_MCAST)
CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_ALE_IOCTL_LOOKUP_VLAN)
CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_ALE_IOCTL_REMOVE_ADDR)
CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_ALE_IOCTL_REMOVE_VLAN)
CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_ALE_IOCTL_REMOVE_OUI)
CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_ALE_IOCTL_REMOVE_IPV4ADDR)
CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_ALE_IOCTL_REMOVE_IPV6ADDR)
CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_ALE_IOCTL_REMOVE_ETHERTYPE)
CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_ALE_IOCTL_REMOVE_LEARNED_ENTRIES)
CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_ALE_IOCTL_REMOVE_ALL_ENTRIES)
CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_ALE_IOCTL_AGE_ALL_ENTRIES)
CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_ALE_IOCTL_SET_RX_FILTER)
CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_ALE_IOCTL_GET_RX_FILTER)
CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_ALE_IOCTL_SET_PORT_STATE)
CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_ALE_IOCTL_GET_PORT_STATE)
CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_ALE_IOCTL_GET_PORT_MACADDR)
CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_ALE_IOCTL_SET_DEFAULT_THREADCFG)
CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_ALE_IOCTL_GET_DEFAULT_THREADCFG)
CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_ALE_IOCTL_SET_PORT_MIRROR_CFG)
CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_ALE_IOCTL_DISABLE_PORT_MIRROR)
CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_ALE_IOCTL_SET_TRUNK_CFG)
CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_ALE_IOCTL_SET_OAMLPBK_CFG)
CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_ALE_IOCTL_SET_BCAST_MCAST_LIMIT)
CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_ALE_IOCTL_GET_BCAST_MCAST_LIMIT)
CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_ALE_IOCTL_DISABLE_BCAST_MCAST_LIMIT)
CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_ALE_IOCTL_SET_POLICER)
CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_ALE_IOCTL_GET_POLICER)
CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_ALE_IOCTL_DEL_POLICER)
CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_ALE_IOCTL_DUMP_POLICER_ENTRIES)
CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_ALE_IOCTL_GET_POLICER_STATS)
CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_ALE_IOCTL_SET_POLICER_THREADCFG)
CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_ALE_IOCTL_SET_POLICER_GLOBAL_CFG)
CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_ALE_IOCTL_GET_POLICER_GLOBAL_CFG)
CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_ALE_IOCTL_DEL_ALL_POLICER_THREADID)
#if ENET_CFG_IS_ON(CPSW_INTERVLAN)
CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_ALE_IOCTL_BLOCK_CLASSIFIER_HOSTPORT)
CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_ALE_IOCTL_SET_INTERVLAN_CFG)
CPSW_ALE_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_ALE_IOCTL_GET_INTERVLAN_CFG)
#endif /* ENET_CFG_IS_ON(CPSW_INTERVLAN) */


