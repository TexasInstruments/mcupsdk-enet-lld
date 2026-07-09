/*
 *  Copyright (C) Texas Instruments Incorporated 2022-2024
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
 * \file  l2_cpsw_cfg.c
 *
 * \brief This file contains the implementation of the APIs for peripheral configuration for l2 cpsw example.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "l2_cpsw_common.h"
#include "l2_cpsw_cfg.h"
#include "l2_cpsw_dataflow.h"
#include "ti_enet_open_close.h"
#include "ti_enet_config.h"
#include <enet_apputils.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static EnetApp_PerCtxt *EnetApp_getPerCtxt(Enet_Type enetType,
                                           uint32_t instId);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t EnetApp_init(void)
{
    int32_t status = ENET_SOK;

    gEnetApp.coreId = EnetSoc_getCoreId();

    /* Initialize all queues */
    EnetQueue_initQ(&gEnetApp.txFreePktInfoQ);

    return status;
}

void EnetApp_deinit(void)
{
    EnetAppUtils_print("Deinit complete\r\n");
}

void EnetApp_showMenu(void)
{
    EnetAppUtils_print("\nEnet L2 cpsw Menu:\r\n");
    EnetAppUtils_print(" 's'  -  Print statistics\r\n");
    EnetAppUtils_print(" 'r'  -  Reset statistics\r\n");
    EnetAppUtils_print(" 'm'  -  Show allocated MAC addresses\r\n");
    EnetAppUtils_print(" 'x'  -  Stop the test\r\n\n");
}

void EnetApp_portLinkStatusChangeCb(Enet_MacPort macPort,
                                    bool isLinkUp,
                                    void *appArg)
{
    EnetAppUtils_print("MAC Port %u: link %s\r\n",
                       ENET_MACPORT_ID(macPort), isLinkUp ? "up" : "down");
}

void EnetApp_mdioLinkStatusChange(Cpsw_MdioLinkStateChangeInfo *info,
                                  void *appArg)
{
    if (info->linkChanged)
    {
        EnetAppUtils_print("Link Status Changed. PHY: 0x%x, state: %s\r\n",
                           info->phyAddr,
                           info->isLinked ? "up" : "down");
    }
}

void EnetApp_updateCpswInitCfg(Enet_Type enetType, uint32_t instId, Cpsw_Cfg *cpswCfg)
{
    EnetApp_PerCtxt *perCtxt = EnetApp_getPerCtxt(enetType, instId);

    EnetAppUtils_assert(perCtxt != NULL);
    /* Prepare init configuration for all peripherals */
    EnetAppUtils_print("\nInit all configs\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");
    EnetAppUtils_print("init config\r\n");

#if (ENET_SYSCFG_ENABLE_MDIO_MANUALMODE == 1U)
    cpswCfg->mdioLinkStateChangeCb = NULL;
    cpswCfg->mdioLinkStateChangeCbArg = NULL;
#else
    cpswCfg->mdioLinkStateChangeCb = EnetApp_mdioLinkStatusChange;
    cpswCfg->mdioLinkStateChangeCbArg = &gEnetApp;
#endif

    cpswCfg->portLinkStatusChangeCb = &EnetApp_portLinkStatusChangeCb;
    cpswCfg->portLinkStatusChangeCbArg = &gEnetApp;
}

int32_t EnetApp_open(EnetApp_PerCtxt *perCtxt)
{
    int32_t status = ENET_SOK;

    /* Do peripheral dependent initalization */
    EnetAppUtils_print("\nInit all peripheral clocks\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");

    EnetAppUtils_enableClocks(perCtxt->enetType, perCtxt->instId);

    /* Create RX tasks for each peripheral */
    if (status == ENET_SOK)
    {
        EnetAppUtils_print("\nCreate RX tasks\r\n");
        EnetAppUtils_print("----------------------------------------------\r\n");
        EnetAppUtils_print("Create RX task\r\n");

        EnetApp_createRxTask(perCtxt);
    }

    /* Open Enet driver for all peripherals */
    EnetAppUtils_print("\nOpen all peripherals\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");

    EnetApp_driverInit();

    EnetApp_HandleInfo handleInfo;

    EnetAppUtils_print("Open enet\r\n");
    status = EnetApp_driverOpen(perCtxt->enetType, perCtxt->instId);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("failed to open enet\r\n");
    }
    EnetApp_acquireHandleInfo(perCtxt->enetType, perCtxt->instId, &handleInfo);
    perCtxt->hEnet = handleInfo.hEnet;
    perCtxt->hMainUdmaDrv = handleInfo.hUdmaDrv;

    /* Attach the core with RM */
    if (status == ENET_SOK)
    {
        EnetAppUtils_print("\nAttach core id %u on all peripherals\r\n", gEnetApp.coreId);
        EnetAppUtils_print("----------------------------------------------\r\n");
        EnetPer_AttachCoreOutArgs attachCoreOutArgs;

        EnetAppUtils_print("Attach core\r\n");

        EnetApp_coreAttach(perCtxt->enetType, perCtxt->instId, gEnetApp.coreId, &attachCoreOutArgs);
        perCtxt->coreKey = attachCoreOutArgs.coreKey;
    }

    /* Open DMA for peripheral/port */
    if (status == ENET_SOK)
    {
        EnetAppUtils_print("Open DMA\r\n");

        status = EnetApp_openDma(perCtxt);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("failed to open DMA: %d\r\n", status);
        }
    }

    if (status == ENET_SOK)
    {
        status = EnetApp_waitForLinkUp(perCtxt);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to wait for link up: %d\r\n", status);
        }
    }

    EnetAppUtils_print(" MAC port addr: ");

    EnetAppUtils_printMacAddr(&perCtxt->macAddr[0U]);

    return status;
}

static EnetApp_PerCtxt *EnetApp_getPerCtxt(Enet_Type enetType,
                                           uint32_t instId)
{
    return (&gEnetApp.perCtxt);
}

void EnetApp_close(EnetApp_PerCtxt *perCtxt)
{
    EnetAppUtils_print("\nClose Ports for all peripherals\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");


    EnetAppUtils_print(" Close Port\r\n");

    EnetApp_closePort(perCtxt);

    EnetAppUtils_print("\nClose DMA for all peripherals\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");

    EnetAppUtils_print("Close DMA\r\n");

    EnetApp_closeDma(perCtxt);

    /* Delete RX tasks created for all peripherals */
    EnetAppUtils_print("\nDelete RX tasks\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");

    EnetApp_destroyRxTask(perCtxt);

    /* Detach core */
    EnetAppUtils_print("\nDetach core from all peripherals\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");

    EnetAppUtils_print("Detach core\r\n");

    EnetApp_coreDetach(perCtxt->enetType, perCtxt->instId,
                       gEnetApp.coreId,
                       perCtxt->coreKey);

    /* Close opened Enet drivers if any peripheral failed */
    EnetAppUtils_print("\nClose all peripherals\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");

    EnetAppUtils_print("Close enet\r\n");
    EnetApp_releaseHandleInfo(perCtxt->enetType, perCtxt->instId);
    perCtxt->hEnet = NULL;

    EnetApp_driverDeInit();

    /* Do peripheral dependent initalization */
    EnetAppUtils_print("\nDeinit all peripheral clocks\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");

    EnetAppUtils_disableClocks(perCtxt->enetType, perCtxt->instId);
}

void EnetApp_printStats(EnetApp_PerCtxt *perCtxt)
{
    Enet_IoctlPrms prms;
    Enet_MacPort macPort;
    const CpswStats_PortStats *pCpswStats;
    int32_t status;

    EnetAppUtils_print("\nPrint statistics\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");

    ENET_IOCTL_SET_OUT_ARGS(&prms, &pCpswStats);

    ENET_IOCTL(perCtxt->hEnet, gEnetApp.coreId, ENET_STATS_IOCTL_GET_HOSTPORT_STATS, &prms, status);
    if (status == ENET_SOK)
    {
        EnetAppUtils_printHostPortStats9G((const CpswStats_HostPort_Ng *)pCpswStats);
    }
    else
    {
        EnetAppUtils_print("Failed to get port stats\r\n");
    }

    macPort = perCtxt->macPort;

    EnetAppUtils_print("\n - Port %u statistics\r\n", ENET_MACPORT_ID(macPort));
    EnetAppUtils_print("--------------------------------\r\n");

    ENET_IOCTL_SET_INOUT_ARGS(&prms, &macPort, &pCpswStats);

    ENET_IOCTL(perCtxt->hEnet, gEnetApp.coreId, ENET_STATS_IOCTL_GET_MACPORT_STATS, &prms, status);
    if (status == ENET_SOK)
    {
        EnetAppUtils_printMacPortStats9G((const CpswStats_MacPort_Ng *)pCpswStats);
    }
    else
    {
        EnetAppUtils_print("Failed to get port %u stats\r\n", ENET_MACPORT_ID(macPort));
    }

    EnetAppUtils_print("\n");
}

void EnetApp_resetStats(EnetApp_PerCtxt *perCtxts)
{
    Enet_IoctlPrms prms;
    Enet_MacPort macPort;
    int32_t status;

    EnetAppUtils_print("\nReset statistics\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");
    EnetApp_PerCtxt *perCtxt = &gEnetApp.perCtxt;

    EnetAppUtils_print("Reset statistics\r\n");

    ENET_IOCTL_SET_NO_ARGS(&prms);
    ENET_IOCTL(perCtxt->hEnet, gEnetApp.coreId, ENET_STATS_IOCTL_RESET_HOSTPORT_STATS, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to reset  host port stats\r\n");
    }

    macPort = perCtxt->macPort;

    ENET_IOCTL_SET_IN_ARGS(&prms, &macPort);
    ENET_IOCTL(perCtxt->hEnet, gEnetApp.coreId, ENET_STATS_IOCTL_RESET_MACPORT_STATS, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to reset port %u stats\r\n", ENET_MACPORT_ID(macPort));
    }
}

void EnetApp_showMacAddrs(EnetApp_PerCtxt *perCtxt)
{

    EnetAppUtils_print("\nAllocated MAC addresses\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");

    EnetAppUtils_print("MAC Address: \t");
    EnetAppUtils_printMacAddr(&perCtxt->macAddr[0U]);
}

void EnetApp_closePort(EnetApp_PerCtxt *perCtxt)
{
    Enet_IoctlPrms prms;
    Enet_MacPort macPort;
    int32_t status;

    macPort = perCtxt->macPort;

    EnetAppUtils_print("Close port %u\r\n", ENET_MACPORT_ID(macPort));

    /* Close port link */
    ENET_IOCTL_SET_IN_ARGS(&prms, &macPort);

    EnetAppUtils_print("Close port %u link\r\n", ENET_MACPORT_ID(macPort));
    ENET_IOCTL(perCtxt->hEnet, gEnetApp.coreId, ENET_PER_IOCTL_CLOSE_PORT_LINK, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to close port link: %d\r\n", status);
    }
}

int32_t EnetApp_waitForLinkUp(EnetApp_PerCtxt *perCtxt)
{
    Enet_IoctlPrms prms;
    Enet_MacPort macPort;
    bool linked;
    int32_t status = ENET_SOK;

    EnetAppUtils_print("Waiting for link up...\r\n");

    macPort = perCtxt->macPort;

    linked = false;

    while (gEnetApp.run && !linked)
    {
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &macPort, &linked);

        ENET_IOCTL(perCtxt->hEnet, gEnetApp.coreId, ENET_PER_IOCTL_IS_PORT_LINK_UP, &prms, status);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to get port %u link status: %d\r\n", ENET_MACPORT_ID(macPort), status);
            linked = false;
            break;
        }

        if (!linked)
        {
            ClockP_sleep(1);
        }
    }

    return status;
}
