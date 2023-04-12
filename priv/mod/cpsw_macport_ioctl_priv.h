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
 * \file  cpsw_macport_ioctl_priv.h
 *
 * \brief This file contains private type definitions and helper macros for the
 *        CPSW MAC port module which are meant for internal use in Enet Per
 *        drivers.
 */

#ifndef CPSW_MACPORT_IOCTL_PRIV_H_
#define CPSW_MACPORT_IOCTL_PRIV_H_

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
/*!
 * \brief Helper macro used to first register private IOCTL handler and then invoke the
 *        IOCTL
 */
#define CPSW_MACPORT_PRIV_IOCTL(hMacport, ioctlCmd,prms,status)                                \
    do {                                                                                       \
        Enet_IoctlPrms regIoctlPrms;                                                           \
        Enet_IoctlRegisterHandlerInArgs regIoctlInArgs;                                        \
                                                                                               \
        regIoctlInArgs.cmd = ioctlCmd;                                                         \
        regIoctlInArgs.fxn = (uintptr_t)&CpswMacPort_ioctl_handler_##ioctlCmd;                 \
                                                                                               \
        ENET_IOCTL_SET_IN_ARGS(&regIoctlPrms, &regIoctlInArgs);                                \
        status = EnetMod_ioctl(hMacport, CPSW_MACPORT_IOCTL_REGISTER_HANDLER, &regIoctlPrms);  \
        if (ENET_SOK == status)                                                                \
        {                                                                                      \
            status = EnetMod_ioctl(hMacport, ioctlCmd,prms);                                   \
        }                                                                                      \
    } while (0)


/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */


/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_GET_VERSION(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_PRINT_REGS(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_SET_INGRESS_DSCP_PRI_MAP(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_GET_INGRESS_DSCP_PRI_MAP(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_SET_PRI_REGEN_MAP(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_GET_PRI_REGEN_MAP(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_SET_EGRESS_QOS_PRI_MAP(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_GET_EGRESS_QOS_PRI_MAP(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_ENABLE_EGRESS_TRAFFIC_SHAPING(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_DISABLE_EGRESS_TRAFFIC_SHAPING(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_GET_EGRESS_TRAFFIC_SHAPING(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_GET_MAXLEN(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_GET_LINK_CFG(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_CPSW_MACPORT_IOCTL_GET_FIFO_STATS(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_CPSW_MACPORT_IOCTL_ENABLE_CPTS_EVENT(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_CPSW_MACPORT_IOCTL_DISABLE_CPTS_EVENT(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_CPSW_MACPORT_IOCTL_ENABLE(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_CPSW_MACPORT_IOCTL_DISABLE(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_CPSW_MACPORT_IOCTL_SET_INTERVLAN_ROUTE(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_CPSW_MACPORT_IOCTL_SET_SPECIFIC_INTERVLAN_ROUTE(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_CPSW_MACPORT_IOCTL_GET_INTERVLAN_ROUTE(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_CPSW_MACPORT_IOCTL_DELETE_INTERVLAN_ROUTE(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_CPSW_MACPORT_IOCTL_GET_INTERVLAN_FREEROUTES(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_CPSW_MACPORT_IOCTL_FIND_INTERVLAN_ROUTE(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_CPSW_MACPORT_IOCTL_IS_INTERVLAN_ROUTE_FREE(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_CPSW_MACPORT_IOCTL_SET_SHORT_IPG(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_CPSW_MACPORT_IOCTL_GET_SHORT_IPG(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_CPSW_MACPORT_IOCTL_GET_SGMII_AUTONEG_LINK_STATUS(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_CPSW_MACPORT_IOCTL_GET_SGMII_LINK_STATUS(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_ENET_TAS_IOCTL_SET_ADMIN_LIST(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_ENET_TAS_IOCTL_GET_OPER_LIST_STATUS(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_ENET_TAS_IOCTL_SET_STATE(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_ENET_TAS_IOCTL_GET_STATE(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_ENET_TAS_IOCTL_GET_ADMIN_LIST(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_ENET_TAS_IOCTL_GET_OPER_LIST(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_ENET_TAS_IOCTL_CONFIG_CHANGE_STATUS_PARAMS(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_CPSW_MACPORT_IOCTL_EST_ENABLE_TIMESTAMP(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_CPSW_MACPORT_IOCTL_EST_DISABLE_TIMESTAMP(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_SET_CREDIT_BASED_SHAPING(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_GET_CREDIT_BASED_SHAPING(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_IET_RELEASE_PREEMPT_TRAFFIC(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_IET_HOLD_PREEMPT_TRAFFIC(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_GET_QUEUE_PREEMPT_STATUS(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_SET_PREEMPT_QUEUE(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_GET_PREEMPT_MIN_FRAG_SIZE(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_SET_PREEMPT_MIN_FRAG_SIZE(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_GET_PREEMPT_VERIFY_STATUS(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_DISABLE_PREEMPT_VERIFICATION(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_ENABLE_PREEMPT_VERIFICATION(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_GET_PREEMPTION_ACTIVE_STATUS(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_GET_PREEMPTION_ENABLE_STATUS(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_DISABLE_PREEMPTION(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_ENABLE_PREEMPTION(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
#if ENET_CFG_IS_ON(CPSW_CUTTHRU)
int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_SET_CUT_THRU_PARAMS(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_GET_CUT_THRU_PARAMS(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
#endif
void CpswMacPort_disablePort(CSL_Xge_cpswRegs *regs, Enet_MacPort macPort);

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
