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
 * \file  cpsw_ioctl_priv.h
 *
 * \brief This file contains the private type definitions and helper macros for
 *        the CPSW peripheral.
 */

#ifndef CPSW_IOCTL_PRIV_H_
#define CPSW_IOCTL_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <priv/core/enet_base_priv.h>
#include <priv/mod/cpsw_ale_priv.h>
#include <priv/mod/cpsw_cpts_priv.h>
#include <priv/mod/cpsw_hostport_priv.h>
#include <priv/mod/cpsw_macport_priv.h>
#include <priv/mod/mdio_priv.h>
#include <priv/mod/cpsw_stats_priv.h>
#include <priv/core/enet_rm_priv.h>
#include <include/core/enet_per.h>
#include <include/core/enet_dma.h>
#include <include/per/cpsw.h>
#include <include/phy/enetphy.h>
#if ENET_CFG_IS_ON(CPSW_EST)
#include <priv/per/cpsw_est_priv.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */
/*!
 * \brief RM RX channel index for CPSW with UDMA.
 *
 * Single UDMA RX channel is used for CPSW, RM channel index is 0-relative.
 */
#define CPSW_RM_RX_CH_IDX                     (0U)


/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */


/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

#if ((ENET_CFG_TRACE_LEVEL >= ENET_CFG_TRACE_LEVEL_ERROR) && ENET_CFG_IS_OFF(TRACE_DISABLE_INFOSTRING))
extern const char *Cpsw_gSpeedNames[];

extern const char *Cpsw_gDuplexNames[];
#endif

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
int32_t Cpsw_internalIoctl_handler_ENET_PER_IOCTL_GET_VERSION(Cpsw_Handle hCpsw, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t Cpsw_internalIoctl_handler_ENET_PER_IOCTL_PRINT_REGS(Cpsw_Handle hCpsw, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t Cpsw_internalIoctl_handler_ENET_PER_IOCTL_OPEN_PORT_LINK(Cpsw_Handle hCpsw, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t Cpsw_internalIoctl_handler_ENET_PER_IOCTL_CLOSE_PORT_LINK(Cpsw_Handle hCpsw, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t Cpsw_internalIoctl_handler_ENET_PER_IOCTL_IS_PORT_LINK_UP(Cpsw_Handle hCpsw, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t Cpsw_internalIoctl_handler_ENET_PER_IOCTL_SET_ISOLATE_STATE(Cpsw_Handle hCpsw, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t Cpsw_internalIoctl_handler_ENET_PER_IOCTL_CLEAR_ISOLATE_STATE(Cpsw_Handle hCpsw, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t Cpsw_internalIoctl_handler_ENET_PER_IOCTL_GET_PORT_LINK_CFG(Cpsw_Handle hCpsw, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t Cpsw_internalIoctl_handler_ENET_PER_IOCTL_ATTACH_CORE(Cpsw_Handle hCpsw, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t Cpsw_internalIoctl_handler_ENET_PER_IOCTL_DETACH_CORE(Cpsw_Handle hCpsw, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t Cpsw_internalIoctl_handler_CPSW_PER_IOCTL_SET_INTERVLAN_ROUTE_UNI_EGRESS(Cpsw_Handle hCpsw, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t Cpsw_internalIoctl_handler_CPSW_PER_IOCTL_CLEAR_INTERVLAN_ROUTE_UNI_EGRESS(Cpsw_Handle hCpsw, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t Cpsw_internalIoctl_handler_CPSW_PER_IOCTL_SET_INTERVLAN_ROUTE_MULTI_EGRESS(Cpsw_Handle hCpsw, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t Cpsw_internalIoctl_handler_CPSW_PER_IOCTL_CLEAR_INTERVLAN_ROUTE_MULTI_EGRESS(Cpsw_Handle hCpsw, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t Cpsw_internalIoctl_handler_CPSW_PER_IOCTL_SET_SHORT_IPG_CFG(Cpsw_Handle hCpsw, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t Cpsw_internalIoctl_handler_CPSW_PER_IOCTL_GET_SHORT_IPG_CFG(Cpsw_Handle hCpsw, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t Cpsw_internalIoctl_handler_ENET_IOCTL_REGISTER_RX_DEFAULT_FLOW(Cpsw_Handle hCpsw, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t Cpsw_internalIoctl_handler_ENET_IOCTL_UNREGISTER_RX_DEFAULT_FLOW(Cpsw_Handle hCpsw, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t Cpsw_internalIoctl_handler_ENET_IOCTL_REGISTER_DSTMAC_RX_FLOW(Cpsw_Handle hCpsw, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t Cpsw_internalIoctl_handler_ENET_IOCTL_UNREGISTER_DSTMAC_RX_FLOW(Cpsw_Handle hCpsw, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t Cpsw_internalIoctl_handler_ENET_PER_IOCTL_HANDLE_EXTPHY_LINKUP_EVENT(Cpsw_Handle hCpsw, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
int32_t Cpsw_internalIoctl_handler_ENET_PER_IOCTL_HANDLE_EXTPHY_LINKDOWN_EVENT(Cpsw_Handle hCpsw, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);

int32_t Cpsw_setDfltThreadCfg(Cpsw_Handle hCpsw,
                              uint32_t flowId);

int32_t Cpsw_handleLinkDown(Cpsw_Handle hCpsw,
                            Enet_MacPort macPort);

int32_t Cpsw_isPortLinkUp(Cpsw_Handle hCpsw,
                          Enet_MacPort macPort,
                          bool *linked);

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

#endif /* CPSW_IOCTL_PRIV_H_ */
