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
 * \file  icssg_ioctl_priv.h
 *
 * \brief This file contains the private type definitions and helper macros for
 *        the ICSSG peripheral for handling IOCTLs.
 */

#ifndef ICSSG_IOCTL_PRIV_H_
#define ICSSG_IOCTL_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <drivers/pruicss.h>
#include <priv/core/enet_base_priv.h>
#include <priv/mod/mdio_priv.h>
#include <priv/per/icssg_priv.h>
#include <priv/mod/icssg_timesync_priv.h>
#include <priv/mod/icssg_stats_priv.h>
#include <priv/mod/icssg_tas_priv.h>
#include <enet.h>
#include <include/core/enet_per.h>
#include <include/core/enet_types.h>
#include <include/mod/mdio.h>
#include <include/phy/enetphy.h>
#include <include/core/enet_mod_phy.h>
#include <priv/core/enet_rm_priv.h>
#include <priv/core/enet_rm_ioctl_priv.h>
#include <priv/mod/phy_priv.h>


#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                       Externs & Macros                                     */
/* ========================================================================== */

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */


/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
int32_t Icssg_ioctl_handler_ENET_PER_IOCTL_OPEN_PORT_LINK(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);

int32_t Icssg_ioctl_handler_ENET_PER_IOCTL_CLOSE_PORT_LINK(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);

int32_t Icssg_ioctl_handler_ENET_PER_IOCTL_IS_PORT_LINK_UP(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);

int32_t Icssg_ioctl_handler_ENET_PER_IOCTL_GET_PORT_LINK_CFG(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);


int32_t Icssg_ioctl_handler_ICSSG_MACPORT_IOCTL_ENABLE_PROMISC_MODE(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);


int32_t Icssg_ioctl_handler_ICSSG_MACPORT_IOCTL_DISABLE_PROMISC_MODE(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);

int32_t Icssg_ioctl_handler_ICSSG_MACPORT_IOCTL_ENABLE_UCAST_FLOOD(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);

int32_t Icssg_ioctl_handler_ICSSG_MACPORT_IOCTL_DISABLE_UCAST_FLOOD(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);

int32_t Icssg_ioctl_handler_ICSSG_MACPORT_IOCTL_ENABLE_MCAST_FLOOD(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);

int32_t Icssg_ioctl_handler_ICSSG_MACPORT_IOCTL_DISABLE_MCAST_FLOOD(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);

int32_t Icssg_ioctl_handler_ICSSG_PER_IOCTL_VLAN_RESET_TABLE(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);

int32_t Icssg_ioctl_handler_ICSSG_PER_IOCTL_VLAN_SET_ENTRY(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);

int32_t Icssg_ioctl_handler_ICSSG_PER_IOCTL_VLAN_GET_ENTRY(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);

int32_t Icssg_ioctl_handler_ICSSG_PER_IOCTL_VLAN_SET_HOSTPORT_DFLT_VID(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);

int32_t Icssg_ioctl_handler_ICSSG_PER_IOCTL_VLAN_SET_MACPORT_DFLT_VID(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);

int32_t Icssg_ioctl_handler_ICSSG_PER_IOCTL_SET_PORT_STATE(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);


int32_t Icssg_ioctl_handler_ICSSG_FDB_IOCTL_ADD_ENTRY(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);


int32_t Icssg_ioctl_handler_ICSSG_FDB_IOCTL_REMOVE_ENTRY(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);


int32_t Icssg_ioctl_handler_ICSSG_FDB_IOCTL_REMOVE_ALL_ENTRIES(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);

int32_t Icssg_ioctl_handler_ICSSG_FDB_IOCTL_REMOVE_AGEABLE_ENTRIES(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);

int32_t Icssg_ioctl_handler_ICSSG_FDB_IOCTL_READ_SLOT_ENTRIES(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);

int32_t Icssg_ioctl_handler_ICSSG_FDB_IOCTL_GET_SLOT_ENTRIES(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);

int32_t Icssg_ioctl_handler_ICSSG_MACPORT_IOCTL_SET_MACADDR(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);

int32_t Icssg_ioctl_handler_ICSSG_HOSTPORT_IOCTL_SET_MACADDR(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);

int32_t Icssg_ioctl_handler_ENET_MACPORT_IOCTL_ENABLE_PREEMPTION(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);

int32_t Icssg_ioctl_handler_ENET_MACPORT_IOCTL_DISABLE_PREEMPTION(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);

int32_t Icssg_ioctl_handler_ENET_MACPORT_IOCTL_GET_PREEMPTION_ENABLE_STATUS(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);

int32_t Icssg_ioctl_handler_ENET_MACPORT_IOCTL_GET_PREEMPTION_ACTIVE_STATUS(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);

int32_t Icssg_ioctl_handler_ENET_MACPORT_IOCTL_ENABLE_PREEMPT_VERIFICATION(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);

int32_t Icssg_ioctl_handler_ENET_MACPORT_IOCTL_DISABLE_PREEMPT_VERIFICATION(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);
int32_t Icssg_ioctl_handler_ENET_MACPORT_IOCTL_GET_PREEMPT_VERIFY_STATUS(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);

int32_t Icssg_ioctl_handler_ENET_MACPORT_IOCTL_GET_PREEMPT_MIN_FRAG_SIZE(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);

int32_t Icssg_ioctl_handler_ENET_MACPORT_IOCTL_SET_PREEMPT_MIN_FRAG_SIZE(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);

int32_t Icssg_ioctl_handler_ENET_MACPORT_IOCTL_SET_PREEMPT_QUEUE(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);

int32_t Icssg_ioctl_handler_ENET_MACPORT_IOCTL_GET_QUEUE_PREEMPT_STATUS(Icssg_Handle hIcssg,
                                                                        uint32_t cmd,
                                                                        Enet_IoctlPrms *prms);

int32_t Icssg_ioctl_handler_ENET_IOCTL_REGISTER_RX_DEFAULT_FLOW(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);

int32_t Icssg_ioctl_handler_ENET_IOCTL_UNREGISTER_RX_DEFAULT_FLOW(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);

int32_t Icssg_ioctl_handler_ENET_PER_IOCTL_ATTACH_CORE(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);
int32_t Icssg_ioctl_handler_ENET_PER_IOCTL_DETACH_CORE(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);

int32_t Icssg_ioctl_handler_ICSSG_PER_IOCTL_TAS_TRIGGER(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);

int32_t Icssg_ioctl_handler_ICSSG_PER_IOCTL_TAS_ENABLE(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);

int32_t Icssg_ioctl_handler_ICSSG_PER_IOCTL_TAS_DISABLE(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);

int32_t Icssg_ioctl_handler_ICSSG_PER_IOCTL_TAS_RESET(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);
int32_t Icssg_ioctl_handler_ICSSG_FDB_IOCTL_SET_AGING_PERIOD(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);

int32_t Icssg_ioctl_handler_ICSSG_MACPORT_IOCTL_SET_ACCEPT_FRAME_CHECK(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);
int32_t Icssg_ioctl_handler_ICSSG_MACPORT_IOCTL_SET_INGRESS_RATE_LIM(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);

int32_t Icssg_ioctl_handler_ENET_PER_IOCTL_SET_VLAN_AWARE(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);

int32_t Icssg_ioctl_handler_ENET_PER_IOCTL_SET_VLAN_UNAWARE(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);

int32_t Icssg_ioctl_handler_ICSSG_MACPORT_IOCTL_SET_QUEUE_CUT_THROUGH_PREEMPT_SELECT(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);

int32_t Icssg_ioctl_handler_ICSSG_MACPORT_IOCTL_CONFIG_SPL_FRAME_PRIO(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);

int32_t Icssg_ioctl_handler_ENET_PER_IOCTL_HANDLE_EXTPHY_LINKUP_EVENT(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);

int32_t Icssg_ioctl_handler_ENET_PER_IOCTL_HANDLE_EXTPHY_LINKDOWN_EVENT(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);

int32_t Icssg_ioctl_handler_ICSSG_ENABLE_PROTOCOL_SPECIFIC_TAG_IOCTL(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);

int32_t Icssg_ioctl_handler_ICSSG_DISABLE_PROTOCOL_SPECIFIC_TAG_IOCTL(Icssg_Handle hIcssg,
                                                            uint32_t cmd,
                                                            Enet_IoctlPrms *prms);

/*ICSSG MAC PORT IOCTLS HANDLER*/

int32_t IcssgMacPort_ioctl_handler_ENET_MACPORT_IOCTL_SET_EGRESS_QOS_PRI_MAP(Icssg_Handle hIcssg,
                                                                                uint32_t cmd,
                                                                                Enet_IoctlPrms *prms);

int32_t IcssgMacPort_ioctl_handler_ENET_MACPORT_IOCTL_SET_PRI_REGEN_MAP(Icssg_Handle hIcssg,
                                                                        uint32_t cmd,
                                                                        Enet_IoctlPrms *prms);

int32_t IcssgMacPort_ioctl_handler_ENET_MACPORT_IOCTL_SET_INGRESS_DSCP_PRI_MAP(Icssg_Handle hIcssg,
                                                                                uint32_t cmd,
                                                                                Enet_IoctlPrms *prms);

int32_t IcssgMacPort_ioctl_handler_ENET_MACPORT_IOCTL_GET_EGRESS_QOS_PRI_MAP(Icssg_Handle hIcssg,
                                                                                uint32_t cmd,
                                                                                Enet_IoctlPrms *prms);

int32_t IcssgMacPort_ioctl_handler_ENET_MACPORT_IOCTL_GET_PRI_REGEN_MAP(Icssg_Handle hIcssg,
                                                                                uint32_t cmd,
                                                                                Enet_IoctlPrms *prms);

int32_t IcssgMacPort_ioctl_handler_ENET_MACPORT_IOCTL_GET_INGRESS_DSCP_PRI_MAP(Icssg_Handle hIcssg,
                                                                                uint32_t cmd,
                                                                                Enet_IoctlPrms *prms);

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

#endif /* ICSSG_IOCTL_PRIV_H_ */
