/*
 *  Copyright (c) Texas Instruments Incorporated 2022
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
 * \file  cpsw_est_priv.h
 *
 * \brief This file contains the private interface for CPSW peripheral
 *        EST feature.
 */

#ifndef CPSW_EST_IOCTL_PRIV_H_
#define CPSW_EST_IOCTL_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <include/core/enet_per.h>
#include <include/core/enet_mod_tas.h>

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
#define CPSW_EST_PRIV_IOCTL(hPer, ioctlCmd,prms,status)                                          \
    do {                                                                                         \
        Enet_IoctlPrms regIoctlPrms;                                                             \
        Enet_IoctlRegisterHandlerInArgs regIoctlInArgs;                                          \
                                                                                                 \
        regIoctlInArgs.cmd = ioctlCmd;                                                           \
        regIoctlInArgs.fxn = (uintptr_t)&CpswEst_ioctl_handler_##ioctlCmd;                       \
                                                                                                 \
        ENET_IOCTL_SET_IN_ARGS(&regIoctlPrms, &regIoctlInArgs);                                  \
        status = Cpsw_ioctlEst(hPer, CPSW_EST_IOCTL_REGISTER_HANDLER, &regIoctlPrms);            \
        if (ENET_SOK == status)                                                                  \
        {                                                                                        \
            status = Cpsw_ioctlEst(hPer, ioctlCmd,prms);                                         \
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

int32_t CpswEst_ioctl_handler_ENET_TAS_IOCTL_SET_ADMIN_LIST(Cpsw_Handle hCpsw, uint32_t cmd, Enet_IoctlPrms *prms);
int32_t CpswEst_ioctl_handler_ENET_TAS_IOCTL_SET_STATE(Cpsw_Handle hCpsw, uint32_t cmd, Enet_IoctlPrms *prms);
int32_t CpswEst_ioctl_handler_ENET_TAS_IOCTL_GET_ADMIN_LIST(Cpsw_Handle hCpsw, uint32_t cmd, Enet_IoctlPrms *prms);
int32_t CpswEst_ioctl_handler_ENET_TAS_IOCTL_GET_OPER_LIST(Cpsw_Handle hCpsw, uint32_t cmd, Enet_IoctlPrms *prms);
int32_t CpswEst_ioctl_handler_ENET_TAS_IOCTL_GET_OPER_LIST_STATUS(Cpsw_Handle hCpsw, uint32_t cmd, Enet_IoctlPrms *prms);
int32_t CpswEst_ioctl_handler_ENET_TAS_IOCTL_GET_STATE(Cpsw_Handle hCpsw, uint32_t cmd, Enet_IoctlPrms *prms);
int32_t CpswEst_ioctl_handler_ENET_TAS_IOCTL_CONFIG_CHANGE_STATUS_PARAMS(Cpsw_Handle hCpsw, uint32_t cmd, Enet_IoctlPrms *prms);

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

#endif /* CPSW_EST_IOCTL_PRIV_H_ */
