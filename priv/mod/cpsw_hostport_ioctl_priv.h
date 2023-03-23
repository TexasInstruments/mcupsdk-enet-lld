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
 * \file  cpsw_hostport_ioctl_priv.h
 *
 * \brief This file contains private type definitions and helper macros for the
 *        CPSW host port module which are meant for internal use in Enet Per
 *        drivers.
 */

#ifndef CPSW_HOSTPORT_IOCTL_PRIV_H_
#define CPSW_HOSTPORT_IOCTL_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <include/core/enet_mod_hostport.h>

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
#define CPSW_HOSTPORT_PRIV_IOCTL(hHostPort, ioctlCmd,prms,status)                                \
    do {                                                                                         \
        Enet_IoctlPrms regIoctlPrms;                                                             \
        Enet_IoctlRegisterHandlerInArgs regIoctlInArgs;                                          \
                                                                                                 \
        regIoctlInArgs.cmd = ioctlCmd;                                                           \
        regIoctlInArgs.fxn = (uintptr_t)&CpswHostPort_ioctl_handler_##ioctlCmd;                  \
                                                                                                 \
        ENET_IOCTL_SET_IN_ARGS(&regIoctlPrms, &regIoctlInArgs);                                  \
        status = EnetMod_ioctl(hHostPort, CPSW_HOSTPORT_IOCTL_REGISTER_HANDLER, &regIoctlPrms);  \
        if (ENET_SOK == status)                                                                  \
        {                                                                                        \
            status = EnetMod_ioctl(hHostPort, ioctlCmd,prms);                                    \
        }                                                                                        \
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
int32_t CpswHostPort_ioctl_handler_ENET_HOSTPORT_IOCTL_GET_VERSION(CpswHostPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswHostPort_ioctl_handler_ENET_HOSTPORT_IOCTL_PRINT_REGS(CpswHostPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswHostPort_ioctl_handler_ENET_HOSTPORT_IOCTL_ENABLE(CpswHostPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswHostPort_ioctl_handler_ENET_HOSTPORT_IOCTL_DISABLE(CpswHostPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswHostPort_ioctl_handler_ENET_HOSTPORT_IOCTL_SET_INGRESS_DSCP_PRI_MAP(CpswHostPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswHostPort_ioctl_handler_ENET_HOSTPORT_IOCTL_GET_INGRESS_DSCP_PRI_MAP(CpswHostPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswHostPort_ioctl_handler_ENET_HOSTPORT_IOCTL_SET_PRI_REGEN_MAP(CpswHostPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswHostPort_ioctl_handler_ENET_HOSTPORT_IOCTL_GET_PRI_REGEN_MAP(CpswHostPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswHostPort_ioctl_handler_ENET_HOSTPORT_IOCTL_SET_EGRESS_QOS_PRI_MAP(CpswHostPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswHostPort_ioctl_handler_ENET_HOSTPORT_IOCTL_GET_EGRESS_QOS_PRI_MAP(CpswHostPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswHostPort_ioctl_handler_ENET_HOSTPORT_IOCTL_ENABLE_INGRESS_TRAFFIC_SHAPING(CpswHostPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswHostPort_ioctl_handler_ENET_HOSTPORT_IOCTL_DISABLE_INGRESS_TRAFFIC_SHAPING(CpswHostPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswHostPort_ioctl_handler_ENET_HOSTPORT_IOCTL_GET_INGRESS_TRAFFIC_SHAPING(CpswHostPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswHostPort_ioctl_handler_ENET_HOSTPORT_IOCTL_GET_MAXLEN(CpswHostPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswHostPort_ioctl_handler_ENET_HOSTPORT_IS_CSUM_OFFLOAD_ENABLED(CpswHostPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswHostPort_ioctl_handler_CPSW_HOSTPORT_IOCTL_GET_FIFO_STATS(CpswHostPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswHostPort_ioctl_handler_CPSW_HOSTPORT_GET_FLOW_ID_OFFSET(CpswHostPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswHostPort_ioctl_handler_CPSW_HOSTPORT_SET_FLOW_ID_OFFSET(CpswHostPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswHostPort_ioctl_handler_ENET_HOSTPORT_IOCTL_SET_CREDIT_BASED_SHAPING(CpswHostPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswHostPort_ioctl_handler_ENET_HOSTPORT_IOCTL_GET_CREDIT_BASED_SHAPING(CpswHostPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);

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

#endif /* CPSW_HOSTPORT_IOCTL_PRIV_H_ */
