/*
 * Copyright (C) 2025 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*!
 * \file app_ethcfg.c
 *
 * \brief this file contains APIs related to ethernet driver(CPSW) configuration
 */
/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "app_ethcfg.h"
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void EthCfg_mdioLinkStatusChange(Cpsw_MdioLinkStateChangeInfo *info,
                                         void *appArg);

static void EthCfg_portLinkStatusChangeCb(Enet_MacPort macPort,
                                           bool isLinkUp,
                                           void *appArg);

void EthCfg_addMCastEntry(Enet_Type enetType,
                          uint32_t instId,
                          uint32_t coreId,
                          const uint8_t *testMCastAddr,
                          uint32_t portMask);

void EthCfg_getEnetInstMacInfo(Enet_Type enetType,
                             uint32_t instId,
                             Enet_MacPort macPortList[],
                             uint8_t numMacPorts);

int32_t EthCfg_setCutThruParams(Enet_Handle hEnet,
                                Enet_MacPort macPort);

int32_t EthCfg_waitForLinkUp(EnetApp_Obj* pEnetApp);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t EthCfg_waitForLinkUp(EnetApp_Obj* pEnetApp)
{
    Enet_IoctlPrms prms;
    Enet_MacPort macPort;
    bool linked;
    uint32_t i;
    int32_t status = ENET_SOK;

    EnetAppUtils_print("Waiting for link up...\r\n");

    for (i = 0U; i < pEnetApp->numMacPort; i++)
    {
        macPort = pEnetApp->macPortList[i];
        linked = false;

        EnetAppUtils_print("Waiting for port %u link up...\r\n", ENET_MACPORT_ID(macPort));
        while (!linked)
        {
            ENET_IOCTL_SET_INOUT_ARGS(&prms, &macPort, &linked);

            ENET_IOCTL(pEnetApp->hEnet, pEnetApp->coreId, ENET_PER_IOCTL_IS_PORT_LINK_UP, &prms, status);
            if (status != ENET_SOK)
            {
                EnetAppUtils_print("Failed to get port %u link status: %d\r\n",
                                    ENET_MACPORT_ID(macPort), status);
                linked = false;
                break;
            }

            ClockP_sleep(1);

        }
    }
    EnetAppUtils_print("\nAll links up \r\n");
    return status;
}

#if ENET_CFG_IS_ON(CPSW_CUTTHRU) 
int32_t EthCfg_setCutThruParams(Enet_Handle hEnet, Enet_MacPort macPort)
{
    EnetMacPort_CutThruParams cutThruInArgs;
    Enet_IoctlPrms prms;
    int32_t status = ENET_SOK;

    cutThruInArgs.cutThruCfg.rxPriCutThruEn = 0xFF;    /* Enabling RX Cut-thru for packet priority 0-7*/
    cutThruInArgs.cutThruCfg.txPriCutThruEn = 0xFF;    /* Enabling TX Cut-thru for packet priority 0-7*/
    cutThruInArgs.cutThruCfg.portSpeedAutoEn = 1U;
    cutThruInArgs.macPort = macPort;

    ENET_IOCTL_SET_IN_ARGS(&prms, &cutThruInArgs);
    ENET_IOCTL(hEnet, EnetSoc_getCoreId(), ENET_MACPORT_IOCTL_SET_CUT_THRU_PARAMS, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to set Cut-thru params: %d\r\n",
                                status);
    }

    return status;
}

#endif 

int32_t EthCfg_init(EnetApp_Obj* pEnetApp)
{
    int32_t status = ENET_SOK;
#if ENET_CFG_IS_ON(CPSW_CUTTHRU)
    uint32_t macPortIndex = 0;
#endif

    /* Read MAC Port details and enable clock for the ENET instance */
    EnetApp_getEnetInstInfo(CONFIG_ENET_CPSW0, &pEnetApp->enetType, &pEnetApp->instId);
    EthCfg_getEnetInstMacInfo(pEnetApp->enetType,
                                pEnetApp->instId,
                                &pEnetApp->macPortList[0],
                                pEnetApp->numMacPort);
    EnetAppUtils_enableClocks(pEnetApp->enetType, pEnetApp->instId);

    EnetAppUtils_print("Enabled clocks for all ENET instances\r\n");
    /* Open ENET driver for each ENET instance */
    EnetApp_driverInit();

    pEnetApp->coreId = EnetSoc_getCoreId();
    
    status = EnetApp_driverOpen(pEnetApp->enetType, pEnetApp->instId);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to open ENET instance: %d\r\n", status);
        EnetAppUtils_assert(status == ENET_SOK);
    }
    else {
        EnetAppUtils_print("Successfully opened ENET instance: %d\r\n", status);
    }

    EthCfg_addMCastEntry(pEnetApp->enetType,
                            pEnetApp->instId,
                            pEnetApp->coreId,
                            BROADCAST_MAC_ADDRESS,
                            CPSW_ALE_ALL_PORTS_MASK);

    /* Attach core */
    if (status == ENET_SOK)
    {
        EnetAppUtils_print("\nAttach core id %u on all peripherals\r\n", pEnetApp->coreId);
        EnetAppUtils_print("----------------------------------------------\r\n");
        EnetPer_AttachCoreOutArgs attachCoreOutArgs;

        EnetApp_coreAttach(pEnetApp->enetType,pEnetApp->instId, pEnetApp->coreId, &attachCoreOutArgs);
        pEnetApp->coreKey = attachCoreOutArgs.coreKey;

        /* Get the ENET handle */
        EnetApp_HandleInfo handleInfo;
        EnetApp_acquireHandleInfo(pEnetApp->enetType, pEnetApp->instId, &handleInfo);
        pEnetApp->hEnet = handleInfo.hEnet;
    }

#if ENET_CFG_IS_ON(CPSW_CUTTHRU) 
    if(status == ENET_SOK)
    {
        for(macPortIndex = 0; macPortIndex < pEnetApp->numMacPort; macPortIndex++)
        {
            status = EthCfg_setCutThruParams(pEnetApp->hEnet, pEnetApp->macPortList[macPortIndex]);
            if (status != ENET_SOK)
            {
                EnetAppUtils_print("failed to enable Cut-thru %d\r\n", status);
            }
        }
    }
#endif 

    if (status == ENET_SOK)
    {
       status = EthCfg_waitForLinkUp(pEnetApp);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to wait for link up: %d\r\n", status);
        }
    }

    /*Print host port MAC address*/
    EnetApp_GetMacAddrOutArgs outArgs;
    EnetApp_getMacAddress(0, &outArgs);

    EnetAppUtils_print("Host MAC Address-%u : ", 0);
    EnetAppUtils_printMacAddr(outArgs.macAddr[0]);
    EnetAppUtils_print("\r\n");

    return status;
}

void EthCfg_getEnetInstMacInfo(Enet_Type enetType,
                             uint32_t instId,
                             Enet_MacPort macPortList[],
                             uint8_t numMacPorts)
{
    macPortList[0] = ENET_MAC_PORT_1;
    if(numMacPorts == 2){
        macPortList[1] = ENET_MAC_PORT_2;
    }
};

void EthCfg_addMCastEntry(Enet_Type enetType,
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
        setMcastInArgs.info.super = false;
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

/*Couldn't rename this because it is declared and used in syscfg generated files*/
void EnetApp_updateCpswInitCfg(Enet_Type enetType, uint32_t instId, Cpsw_Cfg *cpswCfg)
{
#if defined (ENET_SOC_HOSTPORT_DMA_TYPE_CPDMA)
    EnetDma_Cfg * dmaCfg = (EnetDma_Cfg *)cpswCfg->dmaCfg;

    EnetAppUtils_assert(dmaCfg != NULL);
    EnetAppUtils_assert(EnetAppUtils_isDescCached() == false);

    /* Set the enChOverrideFlag to enable the channel override feature of CPDMA */
    dmaCfg->enChOverrideFlag = true;
#endif

#if (ENET_SYSCFG_ENABLE_MDIO_MANUALMODE == 1U)
    cpswCfg->mdioLinkStateChangeCb    = NULL;
    cpswCfg->mdioLinkStateChangeCbArg = NULL;
#else
    cpswCfg->mdioLinkStateChangeCb    = &EthCfg_mdioLinkStatusChange;
    cpswCfg->mdioLinkStateChangeCbArg = NULL;
#endif

    Enet_Handle hEnet = EnetSoc_getEnetHandle(enetType, instId);
    cpswCfg->portLinkStatusChangeCb = &EthCfg_portLinkStatusChangeCb;
    cpswCfg->portLinkStatusChangeCbArg = &hEnet;
}

static void EthCfg_mdioLinkStatusChange(Cpsw_MdioLinkStateChangeInfo *info,
                                         void *appArg)
{
    if (info->linkChanged)
    {
        EnetAppUtils_print("Link Status Changed. PHY: 0x%x, state: %s\r\n",
                info->phyAddr,
                info->isLinked? "up" : "down");
    }
}

static void EthCfg_portLinkStatusChangeCb(Enet_MacPort macPort,
                                           bool isLinkUp,
                                           void *appArg)
{
    EnetAppUtils_print("MAC Port %u: link %s\r\n",
                       ENET_MACPORT_ID(macPort), isLinkUp ? "up" : "down");
}


