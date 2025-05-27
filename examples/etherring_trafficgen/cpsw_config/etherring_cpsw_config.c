/*
 *  Copyright (c) Texas Instruments Incorporated 2025
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
 * \file  app_cpsw_config_handler.c
 *
 * \brief This file contains the configuration of CPSW
 */
/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "etherring_cpsw_config.h"
#include "../etherring_trafficgen_config.h"
#include <enet_appmemutils.h>
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

static const uint8_t BROADCAST_MAC_ADDRESS[ENET_MAC_ADDR_LEN] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};
/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

extern EnetApp_Cfg gEnetAppCfg;

static void EnetApp_mdioLinkStatusChange(Cpsw_MdioLinkStateChangeInfo *info,
                                         void *appArg);

static void EnetApp_portLinkStatusChangeCb(Enet_MacPort macPort,
                                           bool isLinkUp,
                                           void *appArg);
static void EnetApp_updateCfg(EnetApp_Cfg *enet_cfg);

static int32_t EnetApp_openDma();
static void EnetApp_updateEtherRingCfg(EtherRing_Cfg *etherRingCfg);
static void EnetApp_initTxFreePktQ(void);
static void EnetApp_initTxFreePktQ(void);
static void EnetApp_initRxReadyPktQ(EnetDma_RxChHandle hRxCh);
static int32_t EnetApp_addEtherringMcastEntry(void);
int32_t EnetApp_etherRingInit();
static int32_t EnetApp_applyClassifier(uint32_t rxChNum);
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
extern EtherRing_Cfg gEtherRingCfg;
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static void EnetApp_mdioLinkStatusChange(Cpsw_MdioLinkStateChangeInfo *info,
                                         void *appArg)
{
    if (info->linkChanged)
    {
        EnetAppUtils_print("Link Status Changed. PHY: 0x%x, state: %s\r\n",
                info->phyAddr,
                info->isLinked? "up" : "down");
    }
}


static void EnetApp_portLinkStatusChangeCb(Enet_MacPort macPort,
                                           bool isLinkUp, void *appArg)
{
    EnetAppUtils_print("MAC Port %u: link %s\r\n",
                       ENET_MACPORT_ID(macPort), isLinkUp ? "up" : "down");
}

static void EnetApp_initAleConfig(CpswAle_Cfg *aleCfg)
{
    aleCfg->modeFlags = CPSW_ALE_CFG_MODULE_EN;
    aleCfg->agingCfg.autoAgingEn = true;
    aleCfg->agingCfg.agingPeriodInMs = 1000;

    aleCfg->nwSecCfg.vid0ModeEn = true;
    aleCfg->vlanCfg.unknownUnregMcastFloodMask = CPSW_ALE_ALL_PORTS_MASK;
    aleCfg->vlanCfg.unknownRegMcastFloodMask = CPSW_ALE_ALL_PORTS_MASK;
    aleCfg->vlanCfg.unknownVlanMemberListMask = CPSW_ALE_ALL_PORTS_MASK;
    aleCfg->policerGlobalCfg.policingEn = true;
    aleCfg->policerGlobalCfg.yellowDropEn = false;
    /* Enables the ALE to drop the red colored packets. */
    aleCfg->policerGlobalCfg.redDropEn = true;
    /* Policing match mode */
    aleCfg->policerGlobalCfg.policerNoMatchMode = CPSW_ALE_POLICER_NOMATCH_MODE_GREEN;
}

static void EnetApp_initEnetLinkCbPrms(Cpsw_Cfg *cpswCfg)
{
#if (ENET_SYSCFG_ENABLE_MDIO_MANUALMODE == 1U)
    cpswCfg->mdioLinkStateChangeCb = NULL;
    cpswCfg->mdioLinkStateChangeCbArg = NULL;
#else
    cpswCfg->mdioLinkStateChangeCb = EnetApp_mdioLinkStatusChange;
    cpswCfg->mdioLinkStateChangeCbArg = NULL;
#endif

    cpswCfg->portLinkStatusChangeCb    = &EnetApp_portLinkStatusChangeCb;
    cpswCfg->portLinkStatusChangeCbArg = NULL;
}

void EnetApp_updateCpswInitCfg(Enet_Type enetType, uint32_t instId, Cpsw_Cfg *cpswCfg)
{
    CpswHostPort_Cfg *hostPortCfg = &cpswCfg->hostPortCfg;
    /* Prepare init configuration for all peripherals */
    EnetAppUtils_print("\nInit all configs\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");
    EnetAppUtils_print("%s: init config\r\n");

    cpswCfg->hostPortCfg.removeCrc = true;
    cpswCfg->hostPortCfg.padShortPacket = true;
    cpswCfg->hostPortCfg.passCrcErrors = true;
    EnetApp_initEnetLinkCbPrms(cpswCfg);
    EnetApp_initAleConfig(&cpswCfg->aleCfg);

    /* Hardware switch priority is taken from packet's PCP or DSCP */
    hostPortCfg->rxVlanRemapEn     = true;
    hostPortCfg->rxDscpIPv4RemapEn = true;
    hostPortCfg->rxDscpIPv6RemapEn = true;

#if defined SOC_AM263X || defined SOC_AM263PX || defined SOC_AM261X
    EnetDma_Cfg *dmaCfg;
    dmaCfg=(EnetDma_Cfg *)cpswCfg->dmaCfg;
    /* Set the enChOverrideFlag to enable the channel override feature of CPDMA */
    dmaCfg->enChOverrideFlag = true;
#endif

    CpswAle_Cfg *aleCfg = &cpswCfg->aleCfg;

    /* Set Enet global runtime log level */
    Enet_setTraceLevel(ENET_TRACE_DEBUG);

    /* Peripheral config */
    cpswCfg->vlanCfg.vlanAware = true;

    /* Host port config */
    hostPortCfg->rxVlanRemapEn  = true;

    /* ALE config */
    aleCfg->policerGlobalCfg.policingEn = true;
    aleCfg->modeFlags                          |= CPSW_ALE_CFG_MODULE_EN;
    aleCfg->nwSecCfg.vid0ModeEn                = true;
    aleCfg->vlanCfg.aleVlanAwareMode           = true;
    aleCfg->vlanCfg.cpswVlanAwareMode          = true;
    aleCfg->vlanCfg.unknownUnregMcastFloodMask = CPSW_ALE_ALL_MACPORTS_MASK;
    aleCfg->vlanCfg.unknownRegMcastFloodMask   = CPSW_ALE_ALL_MACPORTS_MASK;
    aleCfg->vlanCfg.unknownVlanMemberListMask  = CPSW_ALE_ALL_MACPORTS_MASK;
}


int32_t EnetApp_addVlanEntries(Enet_Handle hEnet, uint32_t coreId, uint32_t vlan)
{
    /* Add the original un-modified vlanId to ALE table */
        Enet_IoctlPrms prms;
        int32_t status = ENET_SOK;
        CpswAle_VlanEntryInfo vlanInArgs;
        uint32_t vlanOutArgs;

        /* Adding ALE vlan entry for redundancy traffic */
        memset(&vlanInArgs, 0, sizeof(vlanInArgs));
        vlanInArgs.vlanIdInfo.vlanId        = vlan;
        vlanInArgs.vlanIdInfo.tagType       = ENET_VLAN_TAG_TYPE_INNER;
        vlanInArgs.vlanMemberList           = 0x7; /* allow for all ports */
        vlanInArgs.unregMcastFloodMask      = 0x6; /* except host port */
        vlanInArgs.regMcastFloodMask        = 0x7;
        vlanInArgs.forceUntaggedEgressMask  = 0U;
        vlanInArgs.noLearnMask              = 0U;
        vlanInArgs.vidIngressCheck          = false;
        vlanInArgs.limitIPNxtHdr            = false;
        vlanInArgs.disallowIPFrag  = false;

        ENET_IOCTL_SET_INOUT_ARGS(&prms, &vlanInArgs, &vlanOutArgs);
        ENET_IOCTL(hEnet, gEnetAppCfg.coreId, CPSW_ALE_IOCTL_ADD_VLAN, &prms, status);
        EnetAppUtils_assert(status == ENET_SOK);

        return status;
}

int32_t EnetApp_updateDefaultPortVlan(Enet_Handle hEnet, uint32_t coreId)
{
    /* Add the original un-modified vlanId to ALE table */
        Enet_IoctlPrms prms;
        int32_t status = ENET_SOK;
        CpswAle_VlanEntryInfo vlanInArgs;
        uint32_t vlanOutArgs;

        /* Adding vlan entry for default port vlan to enable forceUntaggedEgressMask */
        memset(&vlanInArgs, 0, sizeof(vlanInArgs));
        vlanInArgs.vlanIdInfo.vlanId        = 0x0;
        vlanInArgs.vlanIdInfo.tagType       = ENET_VLAN_TAG_TYPE_INNER;
        vlanInArgs.vlanMemberList           = 0x7; /* allow for all ports */
        vlanInArgs.unregMcastFloodMask      = 0x7;
        vlanInArgs.regMcastFloodMask        = 0x7;
        vlanInArgs.forceUntaggedEgressMask  = 7U;
        vlanInArgs.noLearnMask              = 0U;
        vlanInArgs.vidIngressCheck          = false;
        vlanInArgs.limitIPNxtHdr            = false;
        vlanInArgs.disallowIPFrag  = false;

        ENET_IOCTL_SET_INOUT_ARGS(&prms, &vlanInArgs, &vlanOutArgs);
        ENET_IOCTL(hEnet, gEnetAppCfg.coreId, CPSW_ALE_IOCTL_ADD_VLAN, &prms, status);
        EnetAppUtils_assert(status == ENET_SOK);

        return status;
}

int32_t EnetApp_addMCastEntry(Enet_Handle hEnet,
                           uint32_t coreId,
                           const uint8_t *testMCastAddr,
                           uint32_t portMask,
                           uint32_t vlanId)
{
    /* Adding multicast entry for Redundancy Traffic */
    int32_t status = ENET_SOK;
    Enet_IoctlPrms prms;
    uint32_t setMcastoutArgs;

    CpswAle_SetMcastEntryInArgs setMcastInArgs = {
            .addr =
            {
                .vlanId = vlanId,
            },
            .info =
            {
                .portMask = portMask, /* allow only for host port */
                .super = false,
                .fwdState = CPSW_ALE_FWDSTLVL_FWD,
                .numIgnBits =0U,
            },
    };
    memcpy(&setMcastInArgs.addr.addr[0U], testMCastAddr,
           sizeof(setMcastInArgs.addr.addr));
    ENET_IOCTL_SET_INOUT_ARGS(&prms, &setMcastInArgs, &setMcastoutArgs);
    ENET_IOCTL(hEnet,
               coreId,
               CPSW_ALE_IOCTL_ADD_MCAST,
               &prms,
               status);
    EnetAppUtils_assert(status == ENET_SOK);

    return status;
}

int32_t EnetApp_addBroadcastEntry(void)
{
    int32_t status = ENET_SOK;
    status = EnetApp_addMCastEntry(gEnetAppCfg.hEnet,
                          EnetSoc_getCoreId(),
                          BROADCAST_MAC_ADDRESS,
                          CPSW_ALE_ALL_PORTS_MASK,
                          0);
    return status;
}
void EnetApp_initAppCfg(EnetPer_AttachCoreOutArgs *attachArgs, EnetApp_HandleInfo *handleInfo)
{
    /* To support gptp switch mode, we must configure from syscfg file:
     * enet_cpsw1.DisableMacPort2 = false; */
    EnetApp_updateCfg(&gEnetAppCfg);

    gEnetAppCfg.coreId = EnetSoc_getCoreId();
    EnetQueue_initQ(&gEnetAppCfg.txFreePktInfoQ);
    EnetAppUtils_enableClocks(gEnetAppCfg.enetType, gEnetAppCfg.instId);
    DebugP_log("start to open driver.\r\n");
    EnetApp_driverInit();
    EnetApp_driverOpen(gEnetAppCfg.enetType, gEnetAppCfg.instId);
    EnetApp_acquireHandleInfo(gEnetAppCfg.enetType, gEnetAppCfg.instId, handleInfo);
    gEnetAppCfg.hEnet = handleInfo->hEnet;
    EnetApp_coreAttach(gEnetAppCfg.enetType, gEnetAppCfg.instId, gEnetAppCfg.coreId, attachArgs);
    gEnetAppCfg.coreKey = attachArgs->coreKey;
}

static void EnetApp_updateCfg(EnetApp_Cfg *enet_cfg)
{
    EnetApp_getEnetInstInfo(CONFIG_ENET_CPSW0, &enet_cfg->enetType, &enet_cfg->instId);
    EnetApp_getEnetInstMacInfo(enet_cfg->enetType, enet_cfg->instId,
                               enet_cfg->macPorts, &enet_cfg->numMacPorts);
}

/* Rx Pkt DMA Callback for Ether-Ring Redundancy traffic */
static void EnetApp_rxPktCb(void *appData)
{
    SemaphoreP_post(&gEnetAppCfg.rxSemObj);
}

int32_t EnetApp_open()
{
    EnetPer_AttachCoreOutArgs attachCoreOutArgs;
    EnetApp_HandleInfo handleInfo;

    EnetApp_initAppCfg(&attachCoreOutArgs, &handleInfo);
    int32_t status = ENET_SOK;
    status = EnetApp_openDma();
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Open DMA Failed\r\n");
    }
    EnetAppUtils_assert(status == ENET_SOK);

    /* Initializing Ether-ring Config and Attaching Tx,Rx DMA Channel */
    status = EnetApp_etherRingInit();
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Ether-Ring Init Failed\r\n");
    }
    EnetAppUtils_assert(status == ENET_SOK);

    /* Add vlan Id for Redundancy Packet */
    status = EnetApp_addVlanEntries(gEnetAppCfg.hEnet, gEnetAppCfg.coreId, 255);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Add Vlan Entry Failed\r\n");
    }

    /* Update DefaultPortVlan to remove vlanTag on Egress packet */
    status = EnetApp_updateDefaultPortVlan(gEnetAppCfg.hEnet, gEnetAppCfg.coreId);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Add Vlan Entry Failed\r\n");
    }

    EnetAppUtils_assert(status == ENET_SOK);

    /* Add Mcast ALE Entry for Redundancy Packet */
    status = EnetApp_addEtherringMcastEntry();
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Add EtherRing Mcast Entry Failed\r\n");
    }
    EnetAppUtils_assert(status == ENET_SOK);

    /* Add Bcast Entry */
    status = EnetApp_addBroadcastEntry();
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Add BroadCast Entry Failed\r\n");
    }

    EnetAppUtils_assert(status == ENET_SOK);

    gEnetAppCfg.totalRxCnt = 0;

    return status;
}

static int32_t EnetApp_openDma()
{
    EnetApp_GetDmaHandleInArgs     txInArgs;
    EnetApp_GetTxDmaHandleOutArgs  txChInfo;
    int32_t status = ENET_SOK;

    /* Open the TX channel */
    txInArgs.cbArg   = NULL;
    txInArgs.notifyCb = NULL;

    /* ENET_DMA_TX_CH1 used for Ether-Ring Redundancy Traffic */
    EnetApp_getTxDmaHandle((ENET_DMA_TX_CH1),
                           &txInArgs,
                           &txChInfo);

    gEnetAppCfg.txChNum = txChInfo.txChNum;
    gEnetAppCfg.hTxCh   = txChInfo.hTxCh;

    if (gEnetAppCfg.hTxCh == NULL)
    {
#if FIX_RM
        /* Free the channel number if open Tx channel failed */
        EnetAppUtils_freeTxCh(gEnetAppCfg.hEnet,
                              gEnetAppCfg.coreKey,
                              gEnetAppCfg.coreId,
                              gEnetAppCfg.txChNum);
#endif
        EnetAppUtils_print("EnetApp_openDma() failed to open TX channel\r\n");
        status = ENET_EFAIL;
        EnetAppUtils_assert(gEnetAppCfg.hTxCh != NULL);
    }

    /* Allocate TX packets and keep them locally enqueued */
    if (status == ENET_SOK)
    {
        EnetApp_initTxFreePktQ();
    }

    /* Open the RX flow for Regular frames */
    if (status == ENET_SOK)
    {
        EnetApp_GetDmaHandleInArgs     rxInArgs;
        EnetApp_GetRxDmaHandleOutArgs  rxChInfo;

        /* RxISR used for Ether-Ring Redundancy Traffic */
        rxInArgs.notifyCb = EnetApp_rxPktCb;
        rxInArgs.cbArg   = NULL;

        /* ENET_DMA_RX_CH1 used for Ether-Ring Redundancy Traffic */
        EnetApp_getRxDmaHandle(ENET_DMA_RX_CH1,
                               &rxInArgs,
                               &rxChInfo);

        gEnetAppCfg.rxChNum = rxChInfo.rxChNum;

        gEnetAppCfg.hRxCh  = rxChInfo.hRxCh;
        EnetAppUtils_assert(rxChInfo.numValidMacAddress == 1);
        EnetUtils_copyMacAddr(gEnetAppCfg.macAddr, rxChInfo.macAddr[rxChInfo.numValidMacAddress - 1]);
        EnetAppUtils_print("MAC port addr: ");
        EnetAppUtils_printMacAddr(gEnetAppCfg.macAddr);

        if (gEnetAppCfg.hRxCh == NULL)
        {
            EnetAppUtils_print("EnetApp_openRxCh() failed to open RX flow\r\n");
            status = ENET_EFAIL;
            EnetAppUtils_assert(gEnetAppCfg.hRxCh != NULL);
        }
    }

    /* Submit all ready RX buffers to DMA */
    if (status == ENET_SOK)
    {
        EnetApp_initRxReadyPktQ(gEnetAppCfg.hRxCh);
    }

    EnetApp_applyClassifier(gEnetAppCfg.rxChNum);
     return status;
}

void EnetApp_startHwTimer()
{
    TimerP_start(gTimerBaseAddr[CONFIG_TIMER0]);
}

int32_t EnetApp_etherRingInit()
{
    int32_t status = ENET_SOK;
    EtherRing_Cfg *pEtherRingCfg = &gEtherRingCfg;

    EnetApp_updateEtherRingCfg(pEtherRingCfg);
    gEnetAppCfg.hEtherRing = EtherRing_open(gEnetAppCfg.hEnet, gEnetAppCfg.coreId, pEtherRingCfg);
    if (gEnetAppCfg.hEtherRing == NULL)
    {
        EnetAppUtils_print("EtherRing Handle is NULL!!\n");
        Enet_assert(gEnetAppCfg.hEtherRing != NULL);
        status = ENET_EFAIL;
    }

    EtherRing_attachTxDmaHandle((void *)gEnetAppCfg.hEtherRing, gEnetAppCfg.hTxCh, gEnetAppCfg.txChNum);
    EtherRing_attachRxDmaHandle((void *)gEnetAppCfg.hEtherRing, gEnetAppCfg.hRxCh, gEnetAppCfg.rxChNum);

    return status;
}

static void EnetApp_updateEtherRingCfg(EtherRing_Cfg *etherRingCfg)
{
    /* Last byte of Host Mac address is stored to add it in CB Header */
    etherRingCfg->hostMacAddLastByte = gEnetAppCfg.macAddr[ENET_MAC_ADDR_LEN - 1];
    etherRingCfg->isCfg = true;
}

static int32_t EnetApp_applyClassifier(uint32_t rxChNum)
{
    Enet_IoctlPrms prms;
    CpswAle_SetPolicerEntryOutArgs setPolicerEntryOutArgs;
    CpswAle_SetPolicerEntryInArgs setPolicerEntryInArgs;
    int32_t status;

    memset(&setPolicerEntryInArgs, 0, sizeof(setPolicerEntryInArgs));

    if (1)
    {
        setPolicerEntryInArgs.policerMatch.policerMatchEnMask |=
                              CPSW_ALE_POLICER_MATCH_IVLAN;
        setPolicerEntryInArgs.policerMatch.ivlanId = 255U;
    }

    setPolicerEntryInArgs.threadIdEn = true;
    setPolicerEntryInArgs.threadId = rxChNum;

    ENET_IOCTL_SET_INOUT_ARGS(&prms, &setPolicerEntryInArgs,
            &setPolicerEntryOutArgs);
    ENET_IOCTL(gEnetAppCfg.hEnet, gEnetAppCfg.coreId, CPSW_ALE_IOCTL_SET_POLICER, &prms, status);

    if (status != ENET_SOK)
    {
        EnetAppUtils_print("%s():CPSW_ALE_IOCTL_SET_POLICER failed: %d\r\n",
                __func__, status);
    }

    return status;
}

static void EnetApp_initTxFreePktQ(void)
{
    EnetDma_Pkt *pPktInfo;
    uint32_t i;
    uint32_t scatterSegments[] = { ENET_MEM_LARGE_POOL_PKT_SIZE };

    /* Initialize TX EthPkts and queue them to txFreePktInfoQ */
    for (i = 0U; i < 48; i++)
    {
        pPktInfo = EnetMem_allocEthPkt(&gEnetAppCfg,
                                       ENETDMA_CACHELINE_ALIGNMENT,
                                       ENET_ARRAYSIZE(scatterSegments),
                                       scatterSegments);
        EnetAppUtils_assert(pPktInfo != NULL);
        ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState, ENET_PKTSTATE_APP_WITH_FREEQ);

        EnetQueue_enq(&gEnetAppCfg.txFreePktInfoQ, &pPktInfo->node);
    }

    EnetAppUtils_print("initQs() txFreePktInfoQ initialized with %d pkts\r\n",
                       EnetQueue_getQCount(&gEnetAppCfg.txFreePktInfoQ));
}

static void EnetApp_initRxReadyPktQ(EnetDma_RxChHandle hRxCh)
{
    EnetDma_PktQ rxReadyQ;
    EnetDma_PktQ rxFreeQ;
    EnetDma_Pkt *pPktInfo;
    uint32_t i;
    int32_t status;
    uint32_t scatterSegments[] = { ENET_MEM_LARGE_POOL_PKT_SIZE };

    EnetQueue_initQ(&rxFreeQ);

    for (i = 0U; i < 40U; i++)
    {
        pPktInfo = EnetMem_allocEthPkt(&gEnetAppCfg,
                                       ENETDMA_CACHELINE_ALIGNMENT,
                                       ENET_ARRAYSIZE(scatterSegments),
                                       scatterSegments);
        EnetAppUtils_assert(pPktInfo != NULL);

        ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState, ENET_PKTSTATE_APP_WITH_FREEQ);

        EnetQueue_enq(&rxFreeQ, &pPktInfo->node);
    }

    /* Retrieve any packets which are ready */
    EnetQueue_initQ(&rxReadyQ);
    status = EnetDma_retrieveRxPktQ(hRxCh, &rxReadyQ);
    EnetAppUtils_assert(status == ENET_SOK);

    /* There should not be any packet with DMA during init */
    EnetAppUtils_assert(EnetQueue_getQCount(&rxReadyQ) == 0U);

    EnetAppUtils_validatePacketState(&rxFreeQ,
                                     ENET_PKTSTATE_APP_WITH_FREEQ,
                                     ENET_PKTSTATE_APP_WITH_DRIVER);

    EnetDma_submitRxPktQ(hRxCh, &rxFreeQ);

    /* Assert here, as during init, the number of DMA descriptors should be equal to
     * the number of free Ethernet buffers available with app */
    EnetAppUtils_assert(EnetQueue_getQCount(&rxFreeQ) == 0U);
}


int32_t EnetApp_addEtherringMcastEntry(void)
{
    int32_t status = ENET_SOK;
    uint8_t etherringMcastAddress[ENET_MAC_ADDR_LEN] = {
            0x01, 0x00, 0x5E, 0x7F, 0xFF, gEnetAppCfg.nodeId
    };
    status = EnetApp_addMCastEntry(gEnetAppCfg.hEnet,
                          EnetSoc_getCoreId(),
                          etherringMcastAddress,
                          CPSW_ALE_HOST_PORT_MASK,
                          255);
    return status;
}
