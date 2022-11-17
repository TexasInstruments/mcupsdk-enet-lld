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
 * \file  cpsw_macport_intervlan.c
 *
 * \brief This file contains the implementation of the InterVLAN feature of
 *        the CPSW MAC port module.
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
#include <include/mod/cpsw_macport.h>
#include <priv/core/enet_trace_priv.h>
#include <priv/core/enet_base_priv.h>
#include <priv/mod/cpsw_macport_priv.h>
#include "cpsw_macport_intervlan_ioctl_priv.h"


#if ENET_CFG_IS_ON(CPSW_MACPORT_INTERVLAN)

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define CPSW_MACPORT_INTERVLAN_GEN_REGISTER_IOCTL_HANDLER_FXN(x)                      \
int32_t Enet_ioctl_register_##x(Enet_Handle hEnet, uint32_t coreId)                   \
                                                                                      \
{                                                                                     \
    int32_t  status;                                                                  \
    Enet_IoctlPrms prms;                                                              \
    Enet_IoctlRegisterHandlerInArgs inArgs;                                           \
                                                                                      \
    inArgs.cmd = x;                                                                   \
    inArgs.fxn = (uintptr_t)&CpswMacPort_interVlan_ioctl_handler_##x;                 \
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

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
CPSW_MACPORT_INTERVLAN_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_MACPORT_IOCTL_SET_INTERVLAN_ROUTE)
CPSW_MACPORT_INTERVLAN_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_MACPORT_IOCTL_SET_SPECIFIC_INTERVLAN_ROUTE)
CPSW_MACPORT_INTERVLAN_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_MACPORT_IOCTL_GET_INTERVLAN_ROUTE)
CPSW_MACPORT_INTERVLAN_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_MACPORT_IOCTL_DELETE_INTERVLAN_ROUTE)
CPSW_MACPORT_INTERVLAN_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_MACPORT_IOCTL_GET_INTERVLAN_FREEROUTES)
CPSW_MACPORT_INTERVLAN_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_MACPORT_IOCTL_FIND_INTERVLAN_ROUTE)
CPSW_MACPORT_INTERVLAN_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_MACPORT_IOCTL_IS_INTERVLAN_ROUTE_FREE)


#endif
