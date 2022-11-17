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
 * \file  cpsw_cpts_ioctl_priv.h
 *
 * \brief This file contains the private type definitions and helper macros for the
 *        CPSW CPTS module.
 */

#ifndef CPSW_CPTS_IOCTL_PRIV_H_
#define CPSW_CPTS_IOCTL_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <enet_cfg.h>
#include <include/core/enet_mod_timesync.h>
#include <include/mod/cpsw_cpts.h>

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
#define CPSW_CPTS_PRIV_IOCTL(hCpts, ioctlCmd,prms,status)                                        \
    do {                                                                                         \
        Enet_IoctlPrms regIoctlPrms;                                                             \
        Enet_IoctlRegisterHandlerInArgs regIoctlInArgs;                                          \
                                                                                                 \
        regIoctlInArgs.cmd = ioctlCmd;                                                           \
        regIoctlInArgs.fxn = (uintptr_t)&CpswCpts_ioctl_handler_##ioctlCmd;                      \
                                                                                                 \
        ENET_IOCTL_SET_IN_ARGS(&regIoctlPrms, &regIoctlInArgs);                                  \
        status = EnetMod_ioctl(hCpts, CPSW_CPTS_IOCTL_REGISTER_HANDLER, &regIoctlPrms);          \
        if (ENET_SOK == status)                                                                  \
        {                                                                                        \
            status = EnetMod_ioctl(hCpts, ioctlCmd,prms);                                        \
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

int32_t CpswCpts_ioctl_handler_ENET_TIMESYNC_IOCTL_GET_VERSION(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswCpts_ioctl_handler_ENET_TIMESYNC_IOCTL_PRINT_REGS(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswCpts_ioctl_handler_ENET_TIMESYNC_IOCTL_PRINT_STATS(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswCpts_ioctl_handler_ENET_TIMESYNC_IOCTL_GET_CURRENT_TIMESTAMP(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswCpts_ioctl_handler_ENET_TIMESYNC_IOCTL_SET_TIMESTAMP(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswCpts_ioctl_handler_ENET_TIMESYNC_IOCTL_ADJUST_TIMESTAMP(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswCpts_ioctl_handler_ENET_TIMESYNC_IOCTL_GET_ETH_RX_TIMESTAMP(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswCpts_ioctl_handler_ENET_TIMESYNC_IOCTL_GET_ETH_TX_TIMESTAMP(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswCpts_ioctl_handler_ENET_TIMESYNC_IOCTL_RESET(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswCpts_ioctl_handler_CPSW_CPTS_IOCTL_REGISTER_STACK(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswCpts_ioctl_handler_CPSW_CPTS_IOCTL_UNREGISTER_STACK(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswCpts_ioctl_handler_CPSW_CPTS_IOCTL_REGISTER_HWPUSH_CALLBACK(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswCpts_ioctl_handler_CPSW_CPTS_IOCTL_UNREGISTER_HWPUSH_CALLBACK(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswCpts_ioctl_handler_CPSW_CPTS_IOCTL_SET_TS_NUDGE(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswCpts_ioctl_handler_CPSW_CPTS_IOCTL_SET_COMP(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswCpts_ioctl_handler_CPSW_CPTS_IOCTL_SET_COMP_NUDGE(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswCpts_ioctl_handler_CPSW_CPTS_IOCTL_SET_GENF(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswCpts_ioctl_handler_CPSW_CPTS_IOCTL_SET_GENF_NUDGE(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswCpts_ioctl_handler_CPSW_CPTS_IOCTL_SET_ESTF(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswCpts_ioctl_handler_CPSW_CPTS_IOCTL_SET_ESTF_NUDGE(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswCpts_ioctl_handler_CPSW_CPTS_IOCTL_SELECT_TS_OUTPUT_BIT(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswCpts_ioctl_handler_CPSW_CPTS_IOCTL_LOOKUP_EVENT(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswCpts_ioctl_handler_CPSW_CPTS_IOCTL_LOOKUP_EST_EVENT(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswCpts_ioctl_handler_CPSW_CPTS_IOCTL_HANDLE_INTR(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswCpts_ioctl_handler_CPSW_CPTS_IOCTL_ENABLE_INTR(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswCpts_ioctl_handler_CPSW_CPTS_IOCTL_DISABLE_INTR(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prm);

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

#endif /* CPSW_CPTS_IOCTL_PRIV_H_ */
