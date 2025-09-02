/*
 *  Copyright (C) Texas Instruments Incorporated 2025
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
 * \file  cpsw_config.c
 *
 * \brief This file contains the Cpsw configuration APIs used by Applciation
 */
/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */
#include "ethernet_config.h"
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static void EnetApp_updateCfg(EnetApp_Cfg *enetAppCfg)
{
    /* Get Ethernet instance type and ID information */
    EnetApp_getEnetInstInfo(CONFIG_ENET_CPSW0, &enetAppCfg->enetType, &enetAppCfg->instId);
    /* Get MAC port information for this instance */
    EnetApp_getEnetInstMacInfo(enetAppCfg->enetType, enetAppCfg->instId,
                              enetAppCfg->macPorts, &enetAppCfg->numMacPorts);
}

void EnetApp_initAppCfg(EnetPer_AttachCoreOutArgs *attachArgs, EnetApp_HandleInfo *handleInfo,
                         EnetApp_Cfg *enetAppCfg)
{
    /* Update configuration with instance specific information */
    EnetApp_updateCfg(enetAppCfg);
    /* Get core ID */
    enetAppCfg->coreId = EnetSoc_getCoreId();
    /* Initialize the TX free packet queue */
    EnetQueue_initQ(&enetAppCfg->txFreePktInfoQ);
    /* Enable peripheral clocks */
    EnetAppUtils_enableClocks(enetAppCfg->enetType, enetAppCfg->instId);

    DebugP_log("start to open driver.\r\n");
    /* Initialize and open the Ethernet driver */
    EnetApp_driverInit();
    EnetApp_driverOpen(enetAppCfg->enetType, enetAppCfg->instId);
    /* Get handle info for the opened instance */
    EnetApp_acquireHandleInfo(enetAppCfg->enetType, enetAppCfg->instId, handleInfo);
    enetAppCfg->hEnet = handleInfo->hEnet;
    /* Attach this core to the Ethernet peripheral */
    EnetApp_coreAttach(enetAppCfg->enetType, enetAppCfg->instId, enetAppCfg->coreId, attachArgs);
    enetAppCfg->coreKey = attachArgs->coreKey;
}

static void EnetApp_initTxFreePktQ(EnetApp_Cfg *enetAppCfg)
{
    EnetDma_Pkt *pPktInfo;
    uint32_t i;
    uint32_t scatterSegments[] = { ENET_MEM_LARGE_POOL_PKT_SIZE };

    /* Initialize TX EthPkts and queue them to txFreePktInfoQ */
    for (i = 0U; i < 16; i++)
    {
        /* Allocate packet for transmission */
        pPktInfo = EnetMem_allocEthPkt(enetAppCfg,
                                      ENETDMA_CACHELINE_ALIGNMENT,
                                      ENET_ARRAYSIZE(scatterSegments),
                                      scatterSegments);
        EnetAppUtils_assert(pPktInfo != NULL);
        /* Mark packet as being with the free queue */
        ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState, ENET_PKTSTATE_APP_WITH_FREEQ);
        /* Add to free queue */
        EnetQueue_enq(&enetAppCfg->txFreePktInfoQ, &pPktInfo->node);
    }
    EnetAppUtils_print("initQs() txFreePktInfoQ initialized with %d pkts\r\n",
                      EnetQueue_getQCount(&enetAppCfg->txFreePktInfoQ));
}

static void EnetApp_initRxReadyPktQ(EnetDma_RxChHandle hRxCh, EnetApp_Cfg *enetAppCfg)
{
    EnetDma_Pkt *pPktInfo;
    uint32_t i;
    uint32_t scatterSegments[] = { ENET_MEM_LARGE_POOL_PKT_SIZE };

    EnetQueue_initQ(&enetAppCfg->rxFreePktInfoQ);

    /* Allocate and initialize packets for reception */
    for (i = 0U; i < ENET_DMA_RX_CH0_NUM_PKTS; i++)
    {
        pPktInfo = EnetMem_allocEthPkt(enetAppCfg,
                                      ENETDMA_CACHELINE_ALIGNMENT,
                                      ENET_ARRAYSIZE(scatterSegments),
                                      scatterSegments);
        EnetAppUtils_assert(pPktInfo != NULL);
        /* Mark packet as being with the free queue */
        ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState, ENET_PKTSTATE_APP_WITH_FREEQ);
        /* Add to free queue */
        EnetQueue_enq(&enetAppCfg->rxFreePktInfoQ, &pPktInfo->node);
    }
}

static void EnetApp_rxIsrFxn(void* rxSemObj)
{
    /* Post to semaphore when RX interrupt occurs */
    SemaphoreP_post((SemaphoreP_Object*)rxSemObj);
}

int32_t EnetApp_openDma(EnetApp_Cfg *enetAppCfg)
{
    EnetApp_GetDmaHandleInArgs     txInArgs;
    EnetApp_GetTxDmaHandleOutArgs  txChInfo;
    int32_t status = ENET_SOK;

    /* Open the TX channel */
    txInArgs.cbArg   = NULL;
    txInArgs.notifyCb = NULL;
    EnetApp_getTxDmaHandle((ENET_DMA_TX_CH0),
                          &txInArgs,
                          &txChInfo);
    enetAppCfg->txChNum = txChInfo.txChNum;
    enetAppCfg->hTxCh   = txChInfo.hTxCh;

    if (enetAppCfg->hTxCh == NULL)
    {
#if FIX_RM
        /* Free the channel number if open Tx channel failed */
        EnetAppUtils_freeTxCh(enetAppCfg->hEnet,
                             enetAppCfg->coreKey,
                             enetAppCfg->coreId,
                             enetAppCfg->txChNum);
#endif
        EnetAppUtils_print("EnetApp_openDma() failed to open TX channel\r\n");
        status = ENET_EFAIL;
        EnetAppUtils_assert(enetAppCfg->hTxCh != NULL);
    }

    /* Allocate TX packets and keep them locally enqueued */
    if (status == ENET_SOK)
    {
        EnetApp_initTxFreePktQ(enetAppCfg);
    }

    /* Open the RX flow for Regular frames */
    if (status == ENET_SOK)
    {
        EnetApp_GetDmaHandleInArgs     rxInArgs;
        EnetApp_GetRxDmaHandleOutArgs  rxChInfo;

        /* Create semaphore for RX notification */
        status = SemaphoreP_constructBinary(&enetAppCfg->rxSemObj, 0);

        /* Setup RX channel */
        rxInArgs.notifyCb = EnetApp_rxIsrFxn;
        rxInArgs.cbArg   = (void*)&enetAppCfg->rxSemObj;

        EnetApp_getRxDmaHandle(ENET_DMA_RX_CH0,
                              &rxInArgs,
                              &rxChInfo);

        enetAppCfg->rxChNum = rxChInfo.rxChNum;
        enetAppCfg->hRxCh  = rxChInfo.hRxCh;

        EnetAppUtils_assert(rxChInfo.numValidMacAddress == 1);

        /* Store MAC address */
        EnetUtils_copyMacAddr(enetAppCfg->macAddr, rxChInfo.macAddr[rxChInfo.numValidMacAddress - 1]);
        EnetAppUtils_print("MAC port addr: ");
        EnetAppUtils_printMacAddr(enetAppCfg->macAddr);

        if (enetAppCfg->hRxCh == NULL)
        {
            EnetAppUtils_print("EnetApp_openRxCh() failed to open RX flow\r\n");
            status = ENET_EFAIL;
            EnetAppUtils_assert(enetAppCfg->hRxCh != NULL);
        }
    }

    /* Submit all ready RX buffers to DMA */
    if (status == ENET_SOK)
    {
        EnetApp_initRxReadyPktQ(enetAppCfg->hRxCh, enetAppCfg);
    }

    return status;
}

int32_t EnetApp_etherRingInit(EtherRing_Cfg* etherRingCfg, EnetApp_Cfg *enetAppCfg)
{
    int32_t status = ENET_SOK;
    EnetDma_PktQ rxReadyQ;
    EtherRing_Cfg *pEtherRingCfg = etherRingCfg;

    /* Open EtherRing module */
    enetAppCfg->hEtherRing = EtherRing_open(enetAppCfg->hEnet, enetAppCfg->coreId, pEtherRingCfg);
    if (enetAppCfg->hEtherRing == NULL)
    {
        EnetAppUtils_print("EtherRing Handle is NULL!!\n");
        Enet_assert(enetAppCfg->hEtherRing != NULL);
        status = ENET_EFAIL;
    }

    /* Attach TX and RX DMA channels to EtherRing */
    EtherRing_attachTxDmaHandle((void *)enetAppCfg->hEtherRing, enetAppCfg->hTxCh, enetAppCfg->txChNum);
    EtherRing_attachRxDmaHandle((void *)enetAppCfg->hEtherRing, enetAppCfg->hRxCh, enetAppCfg->rxChNum);

    /* Retrieve any packets which are ready */
    EnetQueue_initQ(&rxReadyQ);
    status = EtherRing_retrieveRxPktQ(enetAppCfg->hEtherRing, &rxReadyQ);
    EnetAppUtils_assert(status == ENET_SOK);

    /* There should not be any packet with DMA during init */
    EnetAppUtils_assert(EnetQueue_getQCount(&rxReadyQ) == 0U);

    /* Validate and submit packets to RX DMA */
    EnetAppUtils_validatePacketState(&enetAppCfg->rxFreePktInfoQ,
                                    ENET_PKTSTATE_APP_WITH_FREEQ,
                                    ENET_PKTSTATE_APP_WITH_DRIVER);

    EtherRing_submitRxPktQ(enetAppCfg->hEtherRing, &enetAppCfg->rxFreePktInfoQ);

    /* Assert here, as during init, the number of DMA descriptors should be equal to
     * the number of free Ethernet buffers available with app */
    EnetAppUtils_assert(EnetQueue_getQCount(&enetAppCfg->rxFreePktInfoQ) == 0U);

    return status;
}

int32_t EnetApp_updateDefaultPortVlan(Enet_Handle hEnet, uint32_t coreId)
{
    /* Add the original un-modified vlanId to ALE table */
    Enet_IoctlPrms prms;
    int32_t status = ENET_SOK;
    CpswAle_VlanEntryInfo vlanInArgs;
    uint32_t vlanOutArgs;

    /* Adding vlan entry for default port vlan to enable forceUntaggedEgressMask for ptp */
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
    ENET_IOCTL(hEnet, coreId, CPSW_ALE_IOCTL_ADD_VLAN, &prms, status);
    EnetAppUtils_assert(status == ENET_SOK);

    return status;
}

int32_t EnetApp_addVlanEntries(Enet_Handle hEnet, uint32_t coreId, uint32_t vlan)
{
    /* Add the original un-modified vlanId to ALE table */
    Enet_IoctlPrms prms;
    int32_t status = ENET_SOK;
    CpswAle_VlanEntryInfo vlanInArgs;
    uint32_t vlanOutArgs;

    /* Adding vlan entry for traffic generation */
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
    ENET_IOCTL(hEnet, coreId, CPSW_ALE_IOCTL_ADD_VLAN, &prms, status);
    EnetAppUtils_assert(status == ENET_SOK);

    return status;
}

int32_t EnetApp_configureNodeMcastAddress(Enet_Handle hEnet, uint32_t coreId, uint8_t nodeId)
{
    /* Adding multicast entry for traffic generation */
    int32_t status = ENET_SOK;
    Enet_IoctlPrms prms;
    uint32_t setMcastoutArgs;
    CpswAle_SetMcastEntryInArgs setMcastInArgs = {
        .addr =
        {
            .addr = {0x01,0x00,0x5E,0x7F,0xFF,nodeId},
            .vlanId = 0x255,
        },
        .info =
        {
            .portMask = 0x01, /* allow only for host port */
            .super = false,
            .fwdState = CPSW_ALE_FWDSTLVL_FWD,
            .numIgnBits =0U,
        },
    };

    /* Configure multicast address in ALE table */
    ENET_IOCTL_SET_INOUT_ARGS(&prms, &setMcastInArgs, &setMcastoutArgs);
    ENET_IOCTL(hEnet,
              coreId,
              CPSW_ALE_IOCTL_ADD_MCAST,
              &prms,
              status);
    EnetAppUtils_assert(status == ENET_SOK);

    return status;
}

void EnetApp_configureNodeId(uint32_t* nodeId)
{
    /* Interactive menu to select node type */
    while (true)
    {
        EnetAppUtils_print("0 - Central Compute Node\r\n");
        EnetAppUtils_print("1 - Zone Left Node\r\n");
        EnetAppUtils_print("2 - Zone Right Node\r\n");
        EnetAppUtils_print("3 - Zone Tail Node\r\n");
        EnetAppUtils_print("Enter the nodeId : \r\n");
        DebugP_scanf("%u", nodeId);

        /* Validate input */
        if ((*nodeId < 0) || (*nodeId > 3))
        {
            EnetAppUtils_print("Enter a valid number for nodeId\r\n");
            continue;
        }
        break;
    }
}

static void EnetApp_portLinkStatusChangeCb(Enet_MacPort macPort,
                                         bool isLinkUp, void *appArg)
{
    /* Print link status change message */
    EnetAppUtils_print("MAC Port %u: link %s\r\n",
                      ENET_MACPORT_ID(macPort), isLinkUp ? "up" : "down");
}

static void EnetApp_mdioLinkStatusChange(Cpsw_MdioLinkStateChangeInfo *info,
                                       void *appArg)
{
    /* MDIO link status change callback */
}

static void EnetApp_initEnetLinkCbPrms(Cpsw_Cfg *cpswCfg)
{
    /* Initialize link callback parameters */
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

static void EnetApp_initAleConfig(CpswAle_Cfg *aleCfg)
{
    /* Initialize ALE configuration */
    aleCfg->modeFlags = CPSW_ALE_CFG_MODULE_EN;
    aleCfg->agingCfg.autoAgingEn = true;
    aleCfg->agingCfg.agingPeriodInMs = 1000;
    aleCfg->nwSecCfg.vid0ModeEn = true;
    aleCfg->vlanCfg.unknownUnregMcastFloodMask = CPSW_ALE_ALL_PORTS_MASK;
    aleCfg->vlanCfg.unknownRegMcastFloodMask = CPSW_ALE_ALL_PORTS_MASK;
    aleCfg->vlanCfg.unknownVlanMemberListMask = CPSW_ALE_ALL_PORTS_MASK;

    /* Configure policer settings */
    aleCfg->policerGlobalCfg.policingEn = true;
    aleCfg->policerGlobalCfg.yellowDropEn = false;
    /* Enables the ALE to drop the red colored packets. */
    aleCfg->policerGlobalCfg.redDropEn = true;
    /* Policing match mode */
    aleCfg->policerGlobalCfg.policerNoMatchMode = CPSW_ALE_POLICER_NOMATCH_MODE_GREEN;
}

void EnetApp_updateCpswInitCfg(Enet_Type enetType, uint32_t instId, Cpsw_Cfg *cpswCfg)
{
    CpswHostPort_Cfg *hostPortCfg = &cpswCfg->hostPortCfg;

    /* Prepare init configuration for all peripherals */
    EnetAppUtils_print("\nInit all configs\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");
    EnetAppUtils_print("init config\r\n");

    /* Configure host port settings */
    cpswCfg->hostPortCfg.removeCrc = true;
    cpswCfg->hostPortCfg.padShortPacket = true;
    cpswCfg->hostPortCfg.passCrcErrors = true;

    /* Initialize link callback parameters */
    EnetApp_initEnetLinkCbPrms(cpswCfg);

    /* Initialize ALE configuration */
    EnetApp_initAleConfig(&cpswCfg->aleCfg);

    /* Hardware switch priority is taken from packet's PCP or DSCP */
    hostPortCfg->rxVlanRemapEn     = true;
    hostPortCfg->rxDscpIPv4RemapEn = true;
    hostPortCfg->rxDscpIPv6RemapEn = true;

    EnetDma_Cfg *dmaCfg;
    dmaCfg=(EnetDma_Cfg *)cpswCfg->dmaCfg;

#if defined SOC_AM263X || defined SOC_AM263PX || defined SOC_AM261X
    /* Set the enChOverrideFlag to enable the channel override feature of CPDMA */
    dmaCfg->enChOverrideFlag = true;
#endif

    CpswAle_Cfg *aleCfg = &cpswCfg->aleCfg;

    /* Set Enet global runtime log level */
    Enet_setTraceLevel(ENET_TRACE_DEBUG);

    /* Configure VLAN awareness */
    cpswCfg->vlanCfg.vlanAware = false;

    /* Configure host port */
    hostPortCfg->rxVlanRemapEn  = true;

    /* Configure advanced ALE settings */
    aleCfg->policerGlobalCfg.policingEn = true;
    aleCfg->modeFlags                          |= CPSW_ALE_CFG_MODULE_EN;
    aleCfg->nwSecCfg.vid0ModeEn                = true;
    aleCfg->vlanCfg.aleVlanAwareMode           = false;
    aleCfg->vlanCfg.cpswVlanAwareMode          = false;
    aleCfg->vlanCfg.unknownUnregMcastFloodMask = CPSW_ALE_ALL_MACPORTS_MASK;
    aleCfg->vlanCfg.unknownRegMcastFloodMask   = CPSW_ALE_ALL_MACPORTS_MASK;
    aleCfg->vlanCfg.unknownVlanMemberListMask  = CPSW_ALE_ALL_MACPORTS_MASK;
}

void EnetApp_addMCastEntry(Enet_Type enetType,
                         uint32_t instId,
                         uint32_t coreId,
                         const uint8_t *testMCastAddr,
                         uint32_t portMask)
{
    Enet_IoctlPrms prms;
    int32_t status;
    CpswAle_SetMcastEntryInArgs setMcastInArgs;
    uint32_t setMcastOutArgs;

    if (Enet_isCpswFamily(enetType))
    {
        /* Get Ethernet handle */
        Enet_Handle hEnet = Enet_getHandle(enetType, instId);
        EnetAppUtils_assert(hEnet != NULL);

        /* Setup multicast entry parameters */
        memset(&setMcastInArgs, 0, sizeof(setMcastInArgs));
        memcpy(&setMcastInArgs.addr.addr[0U], testMCastAddr,
              sizeof(setMcastInArgs.addr.addr));
        setMcastInArgs.addr.vlanId  = 0;
        setMcastInArgs.info.super = false;
        setMcastInArgs.info.numIgnBits = 0;
        setMcastInArgs.info.fwdState = CPSW_ALE_FWDSTLVL_FWD;
        setMcastInArgs.info.portMask = portMask;

        /* Add multicast entry to ALE table */
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &setMcastInArgs, &setMcastOutArgs);
        ENET_IOCTL(hEnet, coreId, CPSW_ALE_IOCTL_ADD_MCAST, &prms, status);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("EnetTestBcastMcastLimit_AddAleEntry() failed CPSW_ALE_IOCTL_ADD_MCAST: %d\n",
                              status);
        }
    }
}

void EnetApp_addBCastEntry(Enet_Type enetType,
                         uint32_t instId,
                         uint32_t coreId)
{
    /* Define broadcast MAC address */
    uint8_t BROADCAST_MAC_ADDRESS[ENET_MAC_ADDR_LEN] = {
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
    };

    /* Add broadcast entry to ALE table */
    EnetApp_addMCastEntry(enetType, instId, coreId, BROADCAST_MAC_ADDRESS, CPSW_ALE_ALL_PORTS_MASK);
}

int32_t EnetApp_waitForLinkUp(uint8_t numMacPorts, Enet_MacPort macPorts[], Enet_Handle hEnet, uint32_t coreId)
{
    Enet_IoctlPrms prms;
    Enet_MacPort macPort;
    bool linked;
    uint32_t i;
    int32_t status = ENET_SOK;

    EnetAppUtils_print("Waiting for link up...\r\n");

    /* Check link status for each MAC port */
    for (i = 0U; i < numMacPorts; i++)
    {
        macPort = macPorts[i];
        linked = false;

        /* Poll until link is up */
        while (!linked)
        {
            ENET_IOCTL_SET_INOUT_ARGS(&prms, &macPort, &linked);
            ENET_IOCTL(hEnet, coreId, ENET_PER_IOCTL_IS_PORT_LINK_UP, &prms, status);

            if (status != ENET_SOK)
            {
                EnetAppUtils_print("Failed to get port %u link status: %d\r\n",
                                  ENET_MACPORT_ID(macPort), status);
                linked = false;
                break;
            }

            if (!linked)
            {
                /* Sleep before checking again */
                ClockP_sleep(1);
            }
        }
    }

    return status;
}
