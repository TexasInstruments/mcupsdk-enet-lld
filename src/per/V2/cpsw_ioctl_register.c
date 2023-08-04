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
 * \file  cpsw_ioctl.c
 *
 * \brief This file contains the implementation of the CPSW_2G/CPSW_3G peripheral
 *        variant which is present in the AM273X and AM263x device.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdarg.h>
#include <csl_cpswitch.h>
#include <enet.h>
#include <enet_cfg.h>
#include <priv/mod/cpsw_ale_priv.h>
#include <priv/mod/cpsw_cpts_priv.h>
#include <priv/mod/cpsw_hostport_priv.h>
#include <priv/mod/cpsw_macport_priv.h>
#include <priv/mod/mdio_priv.h>
#include <priv/mod/cpsw_stats_priv.h>
#include <include/core/enet_utils.h>
#include <include/core/enet_osal.h>
#include <include/core/enet_soc.h>
#include <include/core/enet_per.h>
#include <include/per/cpsw.h>
#include <include/dma/cpdma/enet_cpdma.h>
#include <priv/per/cpsw_cpdma_priv.h>
#include <priv/per/cpsw_cpdma_ioctl_priv.h>
#include <priv/per/enet_hostport_cpdma.h>
#include <include/core/enet_utils.h>
#include <include/common/enet_phymdio_dflt.h>
#include <include/phy/enetphy.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define CPSW_GEN_REGISTER_IOCTL_HANDLER_FXN(x)                                        \
int32_t Enet_ioctl_register_##x(Enet_Handle hEnet, uint32_t coreId)                   \
                                                                                      \
{                                                                                     \
    int32_t  status;                                                                  \
    Enet_IoctlPrms prms;                                                              \
    Enet_IoctlRegisterHandlerInArgs inArgs;                                           \
                                                                                      \
    inArgs.cmd = x;                                                                   \
    inArgs.fxn = (uintptr_t)&Cpsw_internalIoctl_handler_##x;                          \
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
CPSW_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_PER_IOCTL_GET_VERSION)
CPSW_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_PER_IOCTL_PRINT_REGS)
CPSW_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_PER_IOCTL_OPEN_PORT_LINK)
CPSW_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_PER_IOCTL_CLOSE_PORT_LINK)
CPSW_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_PER_IOCTL_IS_PORT_LINK_UP)
CPSW_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_PER_IOCTL_SET_ISOLATE_STATE)
CPSW_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_PER_IOCTL_CLEAR_ISOLATE_STATE)
CPSW_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_PER_IOCTL_GET_PORT_LINK_CFG)
CPSW_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_PER_IOCTL_ATTACH_CORE)
CPSW_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_PER_IOCTL_DETACH_CORE)
CPSW_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_PER_IOCTL_SET_INTERVLAN_ROUTE_UNI_EGRESS)
CPSW_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_PER_IOCTL_CLEAR_INTERVLAN_ROUTE_UNI_EGRESS)
CPSW_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_PER_IOCTL_SET_INTERVLAN_ROUTE_MULTI_EGRESS)
CPSW_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_PER_IOCTL_CLEAR_INTERVLAN_ROUTE_MULTI_EGRESS)
CPSW_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_PER_IOCTL_SET_SHORT_IPG_CFG)
CPSW_GEN_REGISTER_IOCTL_HANDLER_FXN(CPSW_PER_IOCTL_GET_SHORT_IPG_CFG)
CPSW_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_IOCTL_REGISTER_RX_DEFAULT_FLOW)
CPSW_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_IOCTL_UNREGISTER_RX_DEFAULT_FLOW)
CPSW_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_IOCTL_REGISTER_DSTMAC_RX_FLOW)
CPSW_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_IOCTL_UNREGISTER_DSTMAC_RX_FLOW)
CPSW_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_PER_IOCTL_HANDLE_EXTPHY_LINKUP_EVENT)
CPSW_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_PER_IOCTL_HANDLE_EXTPHY_LINKDOWN_EVENT)







