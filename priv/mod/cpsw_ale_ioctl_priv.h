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
 * \file  cpsw_ale_ioctl_priv.h
 *
 * \brief This file contains private type definitions and helper macros for the
 *        CPSW ALE module which are meant for internal use in Enet Per drivers.
 */

#ifndef CPSW_ALE_IOCTL_PRIV_H_
#define CPSW_ALE_IOCTL_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <csl_cpswitch.h>
#include <include/core/enet_mod_fdb.h>
#include <include/mod/cpsw_ale.h>

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
#define CPSW_ALE_PRIV_IOCTL(hAle, ioctlCmd,prms,status)                                        \
    do {                                                                                       \
        Enet_IoctlPrms regIoctlPrms;                                                           \
        Enet_IoctlRegisterHandlerInArgs regIoctlInArgs;                                        \
                                                                                               \
        regIoctlInArgs.cmd = ioctlCmd;                                                         \
        regIoctlInArgs.fxn = (uintptr_t)&CpswAle_ioctl_handler_##ioctlCmd;                     \
                                                                                               \
        ENET_IOCTL_SET_IN_ARGS(&regIoctlPrms, &regIoctlInArgs);                                \
        status = EnetMod_ioctl(hAle, CPSW_ALE_IOCTL_REGISTER_HANDLER, &regIoctlPrms);          \
        if (ENET_SOK == status)                                                                \
        {                                                                                      \
            status = EnetMod_ioctl(hAle, ioctlCmd,prms);                                       \
        }                                                                                      \
    } while (0)


/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */


/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */


/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
uint32_t CpswAle_getMaxAleEntries(CSL_AleRegs *regs);
void CpswAle_getVlanMcastPortMask(CSL_AleRegs *regs,
                                  const CSL_CPSW_ALE_VLAN_ENTRY *vlanEntry,
                                  uint32_t *regMcastFloodMask,
                                  uint32_t *unregMcastFloodMask);
int32_t CpswAle_setVlanMcastPortMask(CpswAle_Handle hAle,
                                     CSL_AleRegs *regs,
                                     CSL_CPSW_ALE_VLAN_ENTRY *vlanEntry,
                                     uint32_t entryIdx,
                                     bool isOuterVlan,
                                     uint32_t regMcastFloodMask,
                                     uint32_t unregMcastFloodMask);
int32_t CpswAle_findVlan(CpswAle_Handle hAle,
                         CSL_AleRegs *regs,
                         uint32_t vlanId,
                         bool isOuterVlan,
                         uint32_t *vlanMemberList,
                         uint32_t *unregMcastFloodMask,
                         uint32_t *regMcastFloodMask,
                         uint32_t *forceUntaggedEgress,
                         uint32_t *noLearnMask,
                         bool *vidIngressCheck,
                         bool *limitIpNxtHdr,
                         bool *disallowFrag,
                         uint32_t *entryIdx);
int32_t CpswAle_clearTableEntry(CSL_AleRegs *regs,
                                uint32_t entryIdx);
int32_t CpswAle_getFreeEntry(CpswAle_Handle hAle,
                             CSL_AleRegs *regs,
                             uint32_t *entryIdx);
uint32_t CpswAle_getMaxPolicers(CSL_AleRegs *regs);
uint32_t CpswAle_mapBw2IdlIncVal(uint32_t aleFreqHz,
                                 uint32_t rateInBps);
void CpswAle_clearSelectedPolicerStats(CSL_AleRegs *regs,
                                       uint32_t policerIdx);
int32_t CpswAle_delPolicerEntry(CpswAle_Handle hAle,
                                CSL_AleRegs *regs,
                                uint32_t policerIdx,
                                uint32_t delAleEntryMask);
int32_t CpswAle_addVlan(CpswAle_Handle hAle,
                        CSL_AleRegs *regs,
                        uint32_t vlanId,
                        bool isOuterVlan,
                        uint32_t vlanMemberList,
                        uint32_t unregMcastFloodMask,
                        uint32_t regMcastFloodMask,
                        uint32_t forceUntaggedEgress,
                        uint32_t noLearnMask,
                        bool vidIngressCheck,
                        bool limitIPNxtHdr,
                        bool disallowIPFrag,
                        uint32_t *entryIdx);

int32_t CpswAle_setAlePortState(CSL_AleRegs *regs,
                                uint32_t port_num,
                                CSL_CPSW_ALE_PORTSTATE portState);
int32_t CpswAle_delAllEntries(CpswAle_Handle hAle,
                              CSL_AleRegs *regs);
int32_t CpswAle_setPolicerDefaultThreadCfg(CpswAle_Handle hAle,
                                           CSL_AleRegs *regs,
                                           bool threadEn,
                                           uint32_t threadVal,
                                           bool priorityOREn,
                                           bool macPortDefaultThreadDis);
int32_t CpswAle_setPolicerControl(CSL_AleRegs *regs,
                                  uint32_t aleFreqHz,
                                  bool policingEn,
                                  bool yellowDropEn,
                                  bool redDropEn,
                                  CpswAle_PolicerYellowThresh yellowThresh,
                                  CpswAle_PolicerNoMatchMode policerMatchMode,
                                  const CpswAle_UnregulatedTrafficPolicer *noMatchPolicer);


int32_t CpswAle_ioctl_handler_ENET_FDB_IOCTL_GET_VERSION(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswAle_ioctl_handler_ENET_FDB_IOCTL_PRINT_REGS(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_DUMP_TABLE(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_ADD_UCAST(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_ADD_MCAST(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_ADD_VLAN(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_ADD_OUI(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_ADD_IPV4ADDR(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_ADD_IPV6ADDR(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_ADD_ETHERTYPE(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_LOOKUP_UCAST(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_LOOKUP_MCAST(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_LOOKUP_VLAN(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_REMOVE_ADDR(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_REMOVE_VLAN(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_REMOVE_OUI(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_REMOVE_IPV4ADDR(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_REMOVE_IPV6ADDR(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_REMOVE_ETHERTYPE(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_REMOVE_LEARNED_ENTRIES(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_REMOVE_ALL_ENTRIES(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_AGE_ALL_ENTRIES(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_SET_RX_FILTER(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_GET_RX_FILTER(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_SET_PORT_STATE(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_GET_PORT_STATE(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_GET_PORT_MACADDR(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_SET_DEFAULT_THREADCFG(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_GET_DEFAULT_THREADCFG(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_SET_PORT_MIRROR_CFG(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_DISABLE_PORT_MIRROR(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_SET_TRUNK_CFG(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_SET_OAMLPBK_CFG(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_SET_BCAST_MCAST_LIMIT(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_GET_BCAST_MCAST_LIMIT(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_DISABLE_BCAST_MCAST_LIMIT(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_SET_POLICER(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_GET_POLICER(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_DEL_POLICER(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_DUMP_POLICER_ENTRIES(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_GET_POLICER_STATS(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_SET_POLICER_THREADCFG(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_SET_POLICER_GLOBAL_CFG(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_GET_POLICER_GLOBAL_CFG(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_DEL_ALL_POLICER_THREADID(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_BLOCK_CLASSIFIER_HOSTPORT(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_SET_INTERVLAN_CFG(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_GET_INTERVLAN_CFG(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);


#ifdef __cplusplus
}
#endif

#endif /* CPSW_ALE_IOCTL_PRIV_H_ */
