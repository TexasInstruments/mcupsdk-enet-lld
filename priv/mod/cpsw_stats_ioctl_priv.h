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
 * \file  cpsw_stats_priv.h
 *
 * \brief This file contains private type definitions and helper macros for the
 *        CPSW statistics module which are meant for internal use in Enet Per
 *        drivers.
 */

#ifndef CPSW_STATS_IOCTL_PRIV_H_
#define CPSW_STATS_IOCTL_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <include/core/enet_mod_stats.h>
#include <include/mod/cpsw_stats.h>
#include <priv/mod/cpsw_stats_priv.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */


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
int32_t CpswStats_ioctl_handler_ENET_STATS_IOCTL_GET_VERSION(CpswStats_Handle hStats, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswStats_ioctl_handler_ENET_STATS_IOCTL_PRINT_REGS(CpswStats_Handle hStats, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswStats_ioctl_handler_ENET_STATS_IOCTL_GET_HOSTPORT_STATS(CpswStats_Handle hStats, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswStats_ioctl_handler_ENET_STATS_IOCTL_GET_MACPORT_STATS(CpswStats_Handle hStats, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswStats_ioctl_handler_ENET_STATS_IOCTL_RESET_HOSTPORT_STATS(CpswStats_Handle hStats, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswStats_ioctl_handler_ENET_STATS_IOCTL_RESET_MACPORT_STATS(CpswStats_Handle hStats, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswStats_ioctl_handler_CPSW_STATS_IOCTL_SYNC(CpswStats_Handle hStats, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);

void CpswStats_resetHostStats(CpswStats_Handle hStats);
void CpswStats_resetMacStats(CpswStats_Handle hStats,
                             Enet_MacPort macPort);

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

#endif /* CPSW_STATS_IOCTL_PRIV_H_ */
