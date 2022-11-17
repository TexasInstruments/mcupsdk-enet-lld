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
 * \file  cpsw_cpts.c
 *
 * \brief This file contains the implementation of the CPSW CPTS module.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdarg.h>
#include <csl_cpswitch.h>
#include <enet_cfg.h>
#include <enet.h>
#include <include/core/enet_utils.h>
#include <include/core/enet_osal.h>
#include <include/core/enet_soc.h>
#include <include/mod/cpsw_cpts.h>
#include <priv/core/enet_base_priv.h>
#include <priv/core/enet_trace_priv.h>
#include <priv/mod/cpsw_cpts_priv.h>
#include <priv/mod/cpsw_cpts_ioctl_priv.h>
#include <priv/mod/cpsw_clks.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define CPSW_CPTS_GEN_REGISTER_IOCTL_HANDLER_FXN(x)                                   \
int32_t Enet_ioctl_register_##x(Enet_Handle hEnet, uint32_t coreId)                   \
                                                                                      \
{                                                                                     \
    int32_t  status;                                                                  \
    Enet_IoctlPrms prms;                                                              \
    Enet_IoctlRegisterHandlerInArgs inArgs;                                           \
                                                                                      \
    inArgs.cmd = x;                                                                   \
    inArgs.fxn = (uintptr_t)&CpswCpts_ioctl_handler_##x;                                  \
                                                                                      \
    ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);                                           \
    status = Enet_ioctl(hEnet, coreId, ENET_PER_IOCTL_REGISTER_IOCTL_HANDLER, &prms); \
    return  status;                                                                   \
                                                                                      \
}

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */


/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */


/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
CPSW_CPTS_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_TIMESYNC_IOCTL_GET_VERSION)
CPSW_CPTS_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_TIMESYNC_IOCTL_PRINT_REGS)
CPSW_CPTS_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_TIMESYNC_IOCTL_PRINT_STATS)
CPSW_CPTS_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_TIMESYNC_IOCTL_GET_CURRENT_TIMESTAMP)
CPSW_CPTS_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_TIMESYNC_IOCTL_SET_TIMESTAMP)
CPSW_CPTS_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_TIMESYNC_IOCTL_ADJUST_TIMESTAMP)
CPSW_CPTS_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_TIMESYNC_IOCTL_GET_ETH_RX_TIMESTAMP)
CPSW_CPTS_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_TIMESYNC_IOCTL_GET_ETH_TX_TIMESTAMP)
CPSW_CPTS_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_TIMESYNC_IOCTL_RESET)
CPSW_CPTS_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_CPTS_IOCTL_REGISTER_STACK)
CPSW_CPTS_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_CPTS_IOCTL_UNREGISTER_STACK)
CPSW_CPTS_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_CPTS_IOCTL_REGISTER_HWPUSH_CALLBACK)
CPSW_CPTS_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_CPTS_IOCTL_UNREGISTER_HWPUSH_CALLBACK)
CPSW_CPTS_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_CPTS_IOCTL_SET_TS_NUDGE)
CPSW_CPTS_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_CPTS_IOCTL_SET_COMP)
CPSW_CPTS_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_CPTS_IOCTL_SET_COMP_NUDGE)
CPSW_CPTS_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_CPTS_IOCTL_SET_GENF)
CPSW_CPTS_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_CPTS_IOCTL_SET_GENF_NUDGE)
CPSW_CPTS_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_CPTS_IOCTL_SET_ESTF)
CPSW_CPTS_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_CPTS_IOCTL_SET_ESTF_NUDGE)
CPSW_CPTS_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_CPTS_IOCTL_SELECT_TS_OUTPUT_BIT)
CPSW_CPTS_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_CPTS_IOCTL_LOOKUP_EVENT)
CPSW_CPTS_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_CPTS_IOCTL_LOOKUP_EST_EVENT)
CPSW_CPTS_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_CPTS_IOCTL_HANDLE_INTR)
CPSW_CPTS_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_CPTS_IOCTL_ENABLE_INTR)
CPSW_CPTS_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_CPTS_IOCTL_DISABLE_INTR)
