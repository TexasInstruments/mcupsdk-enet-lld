/*
 *  Copyright (c) Texas Instruments Incorporated 2024
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
 * \file  enetapp.c
 *
 * \brief This file contains the implementation of the Enet TSN example.
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include <stdint.h>
#include <tsn_combase/combase.h>
#include <tsn_combase/combase_link.h>
#include <tsn_combase/tilld/cb_lld_ethernet.h>
#include "dataflow.h"
#include "debug_log.h"
#include "tsninit.h"
#include "enetapp_cpsw.h"
#include "src/per/cpsw_intervlan.h"
/* ========================================================================== */
/*                              Global Variables                              */
/* ========================================================================== */
#define TIMER_ARRAY_SIZE 2000U

#define CONFIG_TIMER0_CLOCK_SRC_MUX_ADDR_ER (0x53208118u)
#define CONFIG_TIMER0_CLOCK_SRC_CTPS_GENF0_ER (0x777u)
#define CONFIG_TIMER0_BASE_ADDR_ER        (0x52181000u)
#define INT_NUM 91u
#define TIMER_NUM_INSTANCES_ER 1

static HwiP_Object gTimerHwiObjER[TIMER_NUM_INSTANCES_ER];
static uint32_t gTimerBaseAddrER[TIMER_NUM_INSTANCES_ER];
static volatile bool isTimerInitDone = false;

EnetApp_Cfg gEnetAppCfg =
{
    .name = ENETAPP_DEFAULT_CFG_NAME,
};

static const uint8_t BROADCAST_MAC_ADDRESS[ENET_MAC_ADDR_LEN] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

/* these vars are shared with gptp task to configure gptp, put it in the global mem */
static char g_netdevices[MAX_NUM_MAC_PORTS][CB_MAX_NETDEVNAME] = {0};
/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */
static void EnetApp_unregisterHwPushEvent0Cb();
static void EnetApp_initRti0();

static void EnetApp_updateCfg(EnetApp_Cfg *enet_cfg)
{
    EnetApp_getEnetInstInfo(CONFIG_ENET_CPSW0, &enet_cfg->enetType, &enet_cfg->instId);
    EnetApp_getEnetInstMacInfo(enet_cfg->enetType, enet_cfg->instId,
                               enet_cfg->macPorts, &enet_cfg->numMacPorts);
}

#define LOG_BUFFER_SIZE (1024)
void ConsolePrint(const char *pcString, ...)
{
    /* Use DebugP_log() because EnetAppUtils_print() has limit bufsize */
    va_list args;
    char buffer[LOG_BUFFER_SIZE];

    va_start(args, pcString);
    vsnprintf(buffer, sizeof(buffer), pcString, args);
    va_end(args);

    DebugP_log("%s", buffer);
}

int EnetApp_lldCfgUpdateCb(cb_socket_lldcfg_update_t *update_cfg)
{
    if (update_cfg->proto == ETH_P_1588)
    {
        update_cfg->numRxChannels = 1;
        update_cfg->dmaTxChId = ENET_DMA_TX_CH_PTP;
        update_cfg->dmaRxChId[0] = ENET_DMA_RX_CH_PTP;
        update_cfg->nTxPkts = ENET_DMA_TX_CH_PTP_NUM_PKTS;
        update_cfg->nRxPkts[0] = ENET_DMA_RX_CH_PTP_NUM_PKTS;
        update_cfg->pktSize = ENET_MEM_LARGE_POOL_PKT_SIZE;
    }
    return 0;
}

int EnetApp_initTsn(void)
{
    lld_ethdev_t ethdevs[MAX_NUMBER_ENET_DEVS] = {0};
    int i;
    int res = 0;
    AppTsnCfg_t appCfg =
    {
        .consoleOutCb = ConsolePrint,
    };

    for (i = 0; i < gEnetAppCfg.numMacPorts; i++)
    {
        snprintf(&g_netdevices[i][0], CB_MAX_NETDEVNAME, "tilld%d", i);
        appCfg.netdevs[i] = &g_netdevices[i][0];
        ethdevs[i].netdev = g_netdevices[i];
        ethdevs[i].macport = gEnetAppCfg.macPorts[i];
        if (i == 0)
        {
            /* tilld0 reuses the allocated source mac, other interfaces will allocate
             * the mac by themself */
            memcpy(ethdevs[i].srcmac, gEnetAppCfg.macAddr, ENET_MAC_ADDR_LEN);
        }
    }

    appCfg.netdevs[i] = NULL;
    if (EnetApp_initTsnByCfg(&appCfg) < 0)
    {
        EnetAppAbort("Failed to int tsn!\r\n");
    }
    if (cb_lld_init_devs_table(ethdevs, i, gEnetAppCfg.enetType,
                               gEnetAppCfg.instId, ENET_SYSCFG_TIMESTAMP_SOURCE) < 0)
    {
        EnetAppAbort("Failed to int devs table!\r\n");
    }
    cb_socket_set_lldcfg_update_cb(EnetApp_lldCfgUpdateCb);

    if (EnetApp_startTsn() < 0)
    {
        EnetAppAbort("Failed to start TSN App!\r\n");
    }
    EnetAppUtils_print("%s:TSN app start done!\r\n", __func__);

    return res;
}

void EnetApp_printCpuLoad(void)
{
    static uint32_t startTime_ms = 0;
    const  uint32_t currTime_ms  = ClockP_getTimeUsec()/1000;
    const  uint32_t printInterval_ms = 5000;

    if (startTime_ms == 0)
    {
        startTime_ms = currTime_ms;
    }
    else if ((currTime_ms - startTime_ms) > printInterval_ms)
    {
        const uint32_t cpuLoad = TaskP_loadGetTotalCpuLoad();

        DebugP_log(" %6d.%3ds : CPU load = %3d.%02d %%\r\n",
                  currTime_ms/1000, currTime_ms%1000,
                  cpuLoad/100, cpuLoad%100 );

        startTime_ms = currTime_ms;
        TaskP_loadResetAll();
    }
    return;
}

static bool IsMacAddrSet(uint8_t *mac)
{
    return ((mac[0]|mac[1]|mac[2]|mac[3]|mac[4]|mac[5]) != 0);
}

static int AddVlan(Enet_Handle hEnet, uint32_t coreId, uint32_t vlanId)
{
    CpswAle_VlanEntryInfo inArgs;
    uint32_t outArgs;
    Enet_IoctlPrms prms;
    int32_t status = ENET_SOK;

    inArgs.vlanIdInfo.vlanId        = vlanId;
    inArgs.vlanIdInfo.tagType       = ENET_VLAN_TAG_TYPE_INNER;
    inArgs.vlanMemberList           = CPSW_ALE_ALL_PORTS_MASK;
    inArgs.unregMcastFloodMask      = CPSW_ALE_ALL_PORTS_MASK;
    inArgs.regMcastFloodMask        = CPSW_ALE_ALL_PORTS_MASK;
    inArgs.forceUntaggedEgressMask  = 0U;
    inArgs.noLearnMask              = 0U;
    inArgs.vidIngressCheck          = BFALSE;
    inArgs.limitIPNxtHdr            = BFALSE;
    inArgs.disallowIPFrag           = BFALSE;

    ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs, &outArgs);
    ENET_IOCTL(hEnet, coreId, CPSW_ALE_IOCTL_ADD_VLAN, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("%s():CPSW_ALE_IOCTL_ADD_VLAN failed: %d\r\n",
                           __func__, status);
    }
    else
    {
        EnetAppUtils_print("CPSW_ALE_IOCTL_ADD_VLAN: %d\r\n", vlanId);
    }

    return status;
}

int32_t EnetApp_addHostPortMcastMembership(Enet_Handle hEnet, uint8_t *mcastMacAddr)
{
    int32_t status = ENET_SOK;
    Enet_IoctlPrms prms;
    CpswAle_SetMcastEntryInArgs setMcastInArgs;
    uint32_t setMcastOutArgs;
    uint32_t coreId = EnetSoc_getCoreId();

    memset(&setMcastInArgs, 0, sizeof(setMcastInArgs));
    memcpy(&(setMcastInArgs.addr.addr[0U]), &(mcastMacAddr[0U]), sizeof(setMcastInArgs.addr.addr));
    setMcastInArgs.info.super = false;
    setMcastInArgs.info.numIgnBits = 0U;
    setMcastInArgs.info.fwdState = CPSW_ALE_FWDSTLVL_FWD;
    setMcastInArgs.info.portMask = CPSW_ALE_HOST_PORT_MASK;
    ENET_IOCTL_SET_INOUT_ARGS(&prms, &setMcastInArgs, &setMcastOutArgs);
    ENET_IOCTL(hEnet, coreId, CPSW_ALE_IOCTL_ADD_MCAST, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("failed to add a new mcast entry to ALE table: %d\n", status);
    }
    return status;
}

static void EnetApp_enableTsSync()
{
    Enet_IoctlPrms prms;
    CpswCpts_OutputBitSel bitSelect;
    int32_t status;

    bitSelect = CPSW_CPTS_TS_OUTPUT_BIT_17;
    ENET_IOCTL_SET_IN_ARGS(&prms, &bitSelect);
    ENET_IOCTL(gEnetAppCfg.hEnet, gEnetAppCfg.coreId, CPSW_CPTS_IOCTL_SELECT_TS_OUTPUT_BIT, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("%s: Failed to set TS SYNC OUT BIT : %d\r\n", gEnetAppCfg.name, status);
    }
    return;
}

void EnetApp_initAppCfg(EnetPer_AttachCoreOutArgs *attachArgs, EnetApp_HandleInfo *handleInfo)
{
    /* To support gptp switch mode, we must configure from syscfg file:
     * enet_cpsw1.DisableMacPort2 = BFALSE; */
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

    EnetApp_enableTsSync();
}

static void EnetApp_macMode2MacMii(emac_mode macMode, EnetMacPort_Interface *mii)
{
    switch (macMode)
    {
    case RMII:
        mii->layerType = ENET_MAC_LAYER_MII;
        mii->sublayerType = ENET_MAC_SUBLAYER_REDUCED;
        mii->variantType = ENET_MAC_VARIANT_NONE;
        break;

    case RGMII:
        mii->layerType = ENET_MAC_LAYER_GMII;
        mii->sublayerType = ENET_MAC_SUBLAYER_REDUCED;
        mii->variantType = ENET_MAC_VARIANT_FORCED;
        break;
    default:
        EnetAppUtils_print("Invalid MAC mode: %u\r\n", macMode);
        EnetAppUtils_assert(BFALSE);
        break;
    }
}

void EnetApp_initLinkArgs(Enet_Type enetType, uint32_t instId,
                          EnetPer_PortLinkCfg *linkArgs, Enet_MacPort macPort)
{
    EnetBoard_EthPort ethPort;
    const EnetBoard_PhyCfg *boardPhyCfg;
    EnetMacPort_LinkCfg *linkCfg = &linkArgs->linkCfg;
    EnetMacPort_Interface *mii = &linkArgs->mii;
    EnetPhy_Cfg *phyCfg = &linkArgs->phyCfg;

    int32_t status = ENET_SOK;

    EnetAppUtils_print("%s: Open port %u\r\n",
                       gEnetAppCfg.name, ENET_MACPORT_ID(macPort));

    /* Setup board for requested Ethernet port */
    ethPort.enetType = gEnetAppCfg.enetType;
    ethPort.instId = gEnetAppCfg.instId;
    ethPort.macPort = macPort;
    ethPort.boardId = EnetBoard_getId();
    EnetApp_macMode2MacMii(RGMII, &ethPort.mii);

    status = EnetBoard_setupPorts(&ethPort, 1U);
    if (status != ENET_SOK) {
        EnetAppUtils_print("%s: Failed to setup MAC port %u\r\n",
                           gEnetAppCfg.name, ENET_MACPORT_ID(macPort));
        EnetAppUtils_assert(BFALSE);
    }

    /* Set port link params */
    linkArgs->macPort = macPort;

    mii->layerType = ethPort.mii.layerType;
    mii->sublayerType = ethPort.mii.sublayerType;
    mii->variantType = ENET_MAC_VARIANT_FORCED;

    linkCfg->speed = ENET_SPEED_AUTO;
    linkCfg->duplexity = ENET_DUPLEX_AUTO;

    boardPhyCfg = EnetBoard_getPhyCfg(&ethPort);
    if (boardPhyCfg != NULL) {
        EnetPhy_initCfg(phyCfg);
        phyCfg->phyAddr = boardPhyCfg->phyAddr;
        phyCfg->isStrapped = boardPhyCfg->isStrapped;
        phyCfg->loopbackEn = BFALSE;
        phyCfg->skipExtendedCfg = boardPhyCfg->skipExtendedCfg;
        phyCfg->extendedCfgSize = boardPhyCfg->extendedCfgSize;
        /* Setting Tx and Rx skew delay values */
        memcpy(phyCfg->extendedCfg, boardPhyCfg->extendedCfg, phyCfg->extendedCfgSize);

    } else {
        EnetAppUtils_print("%s: No PHY configuration found\r\n", gEnetAppCfg.name);
        EnetAppUtils_assert(BFALSE);
    }
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
        Enet_Handle hEnet = Enet_getHandle(enetType, instId);

        EnetAppUtils_assert(hEnet != NULL);
        memset(&setMcastInArgs, 0, sizeof(setMcastInArgs));
        memcpy(&setMcastInArgs.addr.addr[0U], testMCastAddr,
               sizeof(setMcastInArgs.addr.addr));
        setMcastInArgs.addr.vlanId  = 0;
        setMcastInArgs.info.super = BFALSE;
        setMcastInArgs.info.numIgnBits = 0;
        setMcastInArgs.info.fwdState = CPSW_ALE_FWDSTLVL_FWD;
        setMcastInArgs.info.portMask = portMask;
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &setMcastInArgs, &setMcastOutArgs);
        ENET_IOCTL(hEnet, coreId, CPSW_ALE_IOCTL_ADD_MCAST, &prms, status);
        if (status != ENET_SOK)
        {
           EnetAppUtils_print("EnetTestBcastMcastLimit_AddAleEntry() failed CPSW_ALE_IOCTL_ADD_MCAST: %d\n",
                               status);
        }
    }
}

void EnetApp_addBroadcastEntry(void)
{
    EnetApp_addMCastEntry(gEnetAppCfg.enetType,
                          gEnetAppCfg.instId,
                          EnetSoc_getCoreId(),
                          BROADCAST_MAC_ADDRESS,
                          CPSW_ALE_ALL_PORTS_MASK);
}

void EnetApp_setMacAddr(uint8_t hwaddr[])
{
    memcpy(gEnetAppCfg.macAddr, hwaddr, ENET_MAC_ADDR_LEN);
    EnetAppUtils_print("Host MAC address Set: ");
    EnetAppUtils_printMacAddr(hwaddr);
}

static void EnetApp_portLinkStatusChangeCb(Enet_MacPort macPort,
                                           bool isLinkUp, void *appArg)
{
    EnetAppUtils_print("MAC Port %u: link %s\r\n",
                       ENET_MACPORT_ID(macPort), isLinkUp ? "up" : "down");
    cb_lld_notify_linkchange();
}

static void EnetApp_mdioLinkStatusChange(Cpsw_MdioLinkStateChangeInfo *info,
                                             void *appArg)
{
}

static void EnetApp_initAleConfig(CpswAle_Cfg *aleCfg)
{
    aleCfg->modeFlags = CPSW_ALE_CFG_MODULE_EN;
    aleCfg->agingCfg.autoAgingEn = BTRUE;
    aleCfg->agingCfg.agingPeriodInMs = 1000;

    aleCfg->nwSecCfg.vid0ModeEn = BTRUE;
    aleCfg->vlanCfg.unknownUnregMcastFloodMask = CPSW_ALE_ALL_PORTS_MASK;
    aleCfg->vlanCfg.unknownRegMcastFloodMask = CPSW_ALE_ALL_PORTS_MASK;
    aleCfg->vlanCfg.unknownVlanMemberListMask = CPSW_ALE_ALL_PORTS_MASK;
    aleCfg->policerGlobalCfg.policingEn = BTRUE;
    aleCfg->policerGlobalCfg.yellowDropEn = BFALSE;
    /* Enables the ALE to drop the red colored packets. */
    aleCfg->policerGlobalCfg.redDropEn = BTRUE;
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

static void EnetApp_cptsHwPushCb(void *hwPushNotifyCbArg, CpswCpts_HwPush hwPushNum)
{
    (void)hwPushNum;
    if (isTimerInitDone == false)
    {
            isTimerInitDone = true;
            EnetApp_initRti0();
            TimerP_start(gTimerBaseAddrER[0u]);
            // EnetApp_unregisterHwPushEvent0Cb();
    }
}
static void EnetApp_rti0Isr(void *args)
{
    void timerIsrClassA(void);

    timerIsrClassA();
    TimerP_clearOverflowInt(CONFIG_TIMER0_BASE_ADDR_ER);
    HwiP_clearInt(INT_NUM);
}
static void EnetApp_initRti0()
{
    TimerP_Params timerParams;
    HwiP_Params timerHwiParams;
    int32_t status;

    /* set timer clock source */
    SOC_controlModuleUnlockMMR(0u, 4);
    *(volatile uint32_t*)AddrTranslateP_getLocalAddr(CONFIG_TIMER0_CLOCK_SRC_MUX_ADDR_ER) = CONFIG_TIMER0_CLOCK_SRC_CTPS_GENF0_ER; //CPU clock as input
    SOC_controlModuleLockMMR(0u, 4);

    gTimerBaseAddrER[0u] = (uint32_t)AddrTranslateP_getLocalAddr(CONFIG_TIMER0_BASE_ADDR_ER);

    TimerP_Params_init(&timerParams);
    timerParams.inputPreScaler = 1u;
    timerParams.inputClkHz     = 50000000u;
    timerParams.periodInNsec   = 125000u;
    timerParams.oneshotMode    = 0;
    timerParams.enableOverflowInt = 1;
    timerParams.enableDmaTrigger  = 0;
    TimerP_setup(CONFIG_TIMER0_BASE_ADDR_ER, &timerParams);

    HwiP_Params_init(&timerHwiParams);
    timerHwiParams.intNum = INT_NUM;
    timerHwiParams.callback = EnetApp_rti0Isr;
    timerHwiParams.isPulse = 0;
    timerHwiParams.priority = 5;
    status = HwiP_construct(&gTimerHwiObjER[0u], &timerHwiParams);
    DebugP_assertNoLog(status==SystemP_SUCCESS);

    TimerP_start(gTimerBaseAddrER[0u]);
}

static void EnetApp_deInitRti0()
{
    TimerP_stop(gTimerBaseAddrER[0u]);
    HwiP_destruct(&gTimerHwiObjER[0u]);

}

void EnetApp_registerHwPushEvent0Cb()
{
    CpswCpts_RegisterHwPushCbInArgs cptsInArgs;
    cptsInArgs.hwPushNum = CPSW_CPTS_HWPUSH_FIRST;
    cptsInArgs.hwPushNotifyCb = EnetApp_cptsHwPushCb;
    cptsInArgs.hwPushNotifyCbArg = NULL;
    Enet_IoctlPrms prms;
    int32_t status;
    ENET_IOCTL_SET_IN_ARGS(&prms, &cptsInArgs);
    ENET_IOCTL(gEnetAppCfg.hEnet, gEnetAppCfg.coreId, CPSW_CPTS_IOCTL_REGISTER_HWPUSH_CALLBACK, &prms, status);
}

static void EnetApp_unregisterHwPushEvent0Cb()
{
    CpswCpts_RegisterHwPushCbInArgs cptsInArgs;
    cptsInArgs.hwPushNum = CPSW_CPTS_HWPUSH_FIRST;
    Enet_IoctlPrms prms;
    int32_t status;
    ENET_IOCTL_SET_IN_ARGS(&prms, &cptsInArgs);
    ENET_IOCTL(gEnetAppCfg.hEnet, gEnetAppCfg.coreId, CPSW_CPTS_IOCTL_UNREGISTER_HWPUSH_CALLBACK, &prms, status);
}

extern uint64_t gbaseTime;
void EnetApp_configGenf0(Enet_Handle hEnet, uint32_t coreId)
{
    if (hEnet == NULL)
    {
        EnetAppUtils_assert(BFALSE);
    }
    CpswCpts_SetFxnGenInArgs setGenFInArgs;
    Enet_IoctlPrms prms;
    int32_t status;

    /* Configure GENF0 to generate pulse signal */
    /* Length should be same for both master and slave */
    setGenFInArgs.index  = 0U;
    setGenFInArgs.length = 4; //50MHz

    /* Add compare value 20 secs in the future to ensure GENF compare happens
     * and PPS gets generated after the master and slave gets synchronized and reaches
     * stable state */
    setGenFInArgs.compare = 0;
    setGenFInArgs.polarityInv = BTRUE;
    setGenFInArgs.ppmVal  = 0U;
    setGenFInArgs.ppmDir  = CPSW_CPTS_GENF_PPM_ADJDIR_INCREASE;
    setGenFInArgs.ppmMode = ENET_TIMESYNC_ADJMODE_DISABLE;

    ENET_IOCTL_SET_IN_ARGS(&prms, &setGenFInArgs);
    ENET_IOCTL(hEnet,
                coreId,
                CPSW_CPTS_IOCTL_SET_GENF,
                &prms, status);

    if (status != ENET_SOK)
    {
        EnetAppUtils_print("GENF0 Config failed");
    }
}

void EnetApp_configGenf1(Enet_Handle hEnet, uint32_t coreId)
{
    if (hEnet == NULL)
    {
        EnetAppUtils_assert(BFALSE);
    }
    CpswCpts_SetFxnGenInArgs setGenFInArgs;
    Enet_IoctlPrms prms;
    int32_t status;

    /* Configure GENF0 to generate pulse signal */
    /* Length should be same for both master and slave */
    setGenFInArgs.index  = 1U;
    setGenFInArgs.length = 25000*2; //25000is for 125us


    /* A fixed base time of 30 sec is kept.
       20us is subtracted to align the interrupt at start of Clas A Band.
    */
    setGenFInArgs.compare = gbaseTime - 20000;
    setGenFInArgs.polarityInv = BFALSE;
    setGenFInArgs.ppmVal  = 0U;
    setGenFInArgs.ppmDir  = CPSW_CPTS_GENF_PPM_ADJDIR_INCREASE;
    setGenFInArgs.ppmMode = ENET_TIMESYNC_ADJMODE_DISABLE;

    ENET_IOCTL_SET_IN_ARGS(&prms, &setGenFInArgs);
    ENET_IOCTL(hEnet,
                coreId,
                CPSW_CPTS_IOCTL_SET_GENF,
                &prms, status);

    if (status != ENET_SOK)
    {
        EnetAppUtils_print("GENF1 Config failed");
    }
}

void EnetApp_updateCpswInitCfg(Enet_Type enetType, uint32_t instId, Cpsw_Cfg *cpswCfg)
{
    CpswHostPort_Cfg *hostPortCfg = &cpswCfg->hostPortCfg;
    /* Prepare init configuration for all peripherals */
    EnetAppUtils_print("\nInit all configs\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");
    EnetAppUtils_print("%s: init config\r\n", gEnetAppCfg.name);

    cpswCfg->hostPortCfg.removeCrc = BTRUE;
    cpswCfg->hostPortCfg.padShortPacket = BTRUE;
    cpswCfg->hostPortCfg.passCrcErrors = BTRUE;
    EnetApp_initEnetLinkCbPrms(cpswCfg);
    EnetApp_initAleConfig(&cpswCfg->aleCfg);

    /* Hardware switch priority is taken from packet's PCP or DSCP */
    hostPortCfg->rxVlanRemapEn     = BTRUE;
    hostPortCfg->rxDscpIPv4RemapEn = BTRUE;
    hostPortCfg->rxDscpIPv6RemapEn = BTRUE;


    EnetDma_Cfg *dmaCfg;
    dmaCfg=(EnetDma_Cfg *)cpswCfg->dmaCfg;
#if defined SOC_AM263X || defined SOC_AM263PX || defined SOC_AM261X
    /* Set the enChOverrideFlag to enable the channel override feature of CPDMA */

    dmaCfg->enChOverrideFlag = BTRUE;
#endif

//    dmaCfg->txInterruptPerMSec = 1;
//    dmaCfg->rxInterruptPerMSec = 8;

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

static void EnetApp_closePort()
{
    Enet_IoctlPrms prms;
    Enet_MacPort macPort;
    uint32_t i;
    int32_t status;

    for (i = 0U; i < gEnetAppCfg.numMacPorts; i++)
    {
        macPort = gEnetAppCfg.macPorts[i];

        EnetAppUtils_print("%s: Close port %u\r\n", gEnetAppCfg.name, ENET_MACPORT_ID(macPort));

        /* Close port link */
        ENET_IOCTL_SET_IN_ARGS(&prms, &macPort);

        EnetAppUtils_print("%s: Close port %u link\r\n", gEnetAppCfg.name, ENET_MACPORT_ID(macPort));
        ENET_IOCTL(gEnetAppCfg.hEnet, gEnetAppCfg.coreId, ENET_PER_IOCTL_CLOSE_PORT_LINK, &prms, status);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("%s: Failed to close port link: %d\r\n", gEnetAppCfg.name, status);
        }
    }
}

void EnetApp_close()
{
    EnetAppUtils_print("\nClose Ports for all peripherals\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");
    EnetApp_closePort();

    /* Delete RX tasks created for all peripherals */
    EnetAppUtils_print("\nDelete RX tasks\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");
    EnetApp_destroyRxTask();

    /* Detach core */
    EnetAppUtils_print("\nDetach core from all peripherals\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");
    EnetApp_coreDetach(gEnetAppCfg.enetType,gEnetAppCfg.instId,
                       gEnetAppCfg.coreId,
                       gEnetAppCfg.coreKey);

    /* Close opened Enet drivers if any peripheral failed */
    EnetAppUtils_print("\nClose all peripherals\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");
    EnetApp_releaseHandleInfo(gEnetAppCfg.enetType, gEnetAppCfg.instId);
    gEnetAppCfg.hEnet = NULL;

    /* Do peripheral dependent initalization */
    EnetAppUtils_print("\nDeinit all peripheral clocks\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");
    EnetAppUtils_disableClocks(gEnetAppCfg.enetType, gEnetAppCfg.instId);
}
