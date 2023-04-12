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
 * \file  cpsw_stats.c
 *
 * \brief This file contains the implementation of the CPSW statistics module.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdarg.h>
#include <csl_cpswitch.h>
#include <enet_cfg.h>
#include <include/core/enet_utils.h>
#include <include/core/enet_soc.h>
#include <include/mod/cpsw_stats.h>
#include <priv/core/enet_trace_priv.h>
#include <priv/mod/cpsw_stats_priv.h>
#include <priv/mod/cpsw_stats_ioctl_priv.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
static void CpswStats_readHostStats(CpswStats_Handle hStats);
static void CpswStats_readMacStats(CpswStats_Handle hStats,
                                   Enet_MacPort macPort);


/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */



/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
int32_t CpswStats_ioctl_handler_ENET_STATS_IOCTL_GET_VERSION(CpswStats_Handle hStats, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    Enet_Version *version = (Enet_Version *)prms->outArgs;
    CSL_CPSW_VERSION ver = {0};
    int32_t status = ENET_SOK;

    /* Report CPSW Control version as ours */
    CSL_CPSW_getCpswVersionInfo(regs, &ver);
    version->maj = ver.majorVer;
    version->min = ver.minorVer;
    version->rtl = ver.rtlVer;
    version->id  = ver.id;
    version->other1 = ENET_VERSION_NONE;
    version->other2 = ENET_VERSION_NONE;
    return status;
}

int32_t CpswStats_ioctl_handler_ENET_STATS_IOCTL_PRINT_REGS(CpswStats_Handle hStats, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

    ENETTRACE_VAR(regs);
    ENETTRACE_INFO("STATS: 0x%08x\n", regs->STAT_PORT_EN_REG);
    return status;
}

int32_t CpswStats_ioctl_handler_ENET_STATS_IOCTL_GET_HOSTPORT_STATS(CpswStats_Handle hStats, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

    status = Enet_checkOutArgs(prms, sizeof(CpswStats_PortStats));
    if (status == ENET_SOK)
    {
        CpswStats_PortStats *portStats = (CpswStats_PortStats *)prms->outArgs;

        CpswStats_readHostStats(hStats);
        memcpy(portStats, hStats->hostPortStats, sizeof(CpswStats_PortStats));
    }
    else
    {
        ENETTRACE_ERR("Invalid GET_HOSTPORT_STATS params\n");
    }
    return status;
}

int32_t CpswStats_ioctl_handler_ENET_STATS_IOCTL_GET_MACPORT_STATS(CpswStats_Handle hStats, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

    status = Enet_checkInOutArgs(prms, sizeof(Enet_MacPort), sizeof(CpswStats_PortStats));
    if (status == ENET_SOK)
    {
        Enet_MacPort *macPort = (Enet_MacPort *)prms->inArgs;
        CpswStats_PortStats *portStats = (CpswStats_PortStats *)prms->outArgs;
        uint32_t portNum = ENET_MACPORT_NORM(*macPort);

        CpswStats_readMacStats(hStats, *macPort);
        memcpy(portStats, &hStats->macPortStats[portNum], sizeof(CpswStats_PortStats));
    }
    else
    {
        ENETTRACE_ERR("Invalid GET_MACPORT_STATS params\n");
    }
    return status;
}

int32_t CpswStats_ioctl_handler_ENET_STATS_IOCTL_RESET_HOSTPORT_STATS(CpswStats_Handle hStats, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

    CpswStats_resetHostStats(hStats);
    return status;
}

int32_t CpswStats_ioctl_handler_ENET_STATS_IOCTL_RESET_MACPORT_STATS(CpswStats_Handle hStats, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

    status = Enet_checkInArgs(prms, sizeof(Enet_MacPort));
    if (status == ENET_SOK)
    {
        Enet_MacPort *macPort = (Enet_MacPort *)prms->inArgs;

        CpswStats_resetMacStats(hStats, *macPort);
    }
    else
    {
        ENETTRACE_ERR("Invalid RESET_MACPORT_STATS params\n");
    }
    return status;
}

int32_t CpswStats_ioctl_handler_CPSW_STATS_IOCTL_SYNC(CpswStats_Handle hStats, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    uint32_t i;
    int32_t status = ENET_SOK;

    CpswStats_readHostStats(hStats);
    for (i = 0U; i < hStats->macPortNum; i++)
    {
        CpswStats_readMacStats(hStats, ENET_MACPORT_DENORM(i));
    }
    return status;
}

static void CpswStats_readHostStats(CpswStats_Handle hStats)
{
    CSL_Xge_cpswRegs *regs = (CSL_Xge_cpswRegs *)hStats->enetMod.virtAddr;
    union CSL_CPSW_STATS portStats;
    uint64_t *stats64;
    uint32_t *stats32 = (uint32_t *)&portStats;
    uint32_t i;

    Enet_devAssert(hStats->hostPortStats != NULL, "Invalid host port stats memory address\n");

    /* CSL blindly reads all registers in the statistics block regardless
     * of whether they are applicable or not to a CPSW instance type */
    memset(&portStats.p0_stats, 0, sizeof(portStats.p0_stats));
    CSL_CPSW_getPortStats(regs, 0U, &portStats);

    stats64 = &hStats->hostPortStats->val[0U];
    for (i = 0U; i < CPSW_STATS_BLOCK_ELEM_NUM; i++)
    {
        stats64[i] += stats32[i];
    }

    /* Clear reserved fields */
    switch (hStats->enetType)
    {
        case ENET_CPSW_2G:
        {
            CpswStats_HostPort_2g *hostStats2g = NULL;
            hostStats2g = (CpswStats_HostPort_2g *)stats64;

            hostStats2g->reserved4 = 0U;
            hostStats2g->reserved6 = 0U;
            hostStats2g->reserved8 = 0U;
            memset(hostStats2g->reserved17to19, 0, sizeof(hostStats2g->reserved17to19));
            memset(hostStats2g->reserved22to25, 0, sizeof(hostStats2g->reserved22to25));
            memset(hostStats2g->reserved57to95, 0, sizeof(hostStats2g->reserved57to95));
        }
        break;

        case ENET_CPSW_3G:
        case ENET_CPSW_5G:
        case ENET_CPSW_9G:
        {
            CpswStats_HostPort_Ng *hostStatsNg = NULL;
            hostStatsNg = (CpswStats_HostPort_Ng *)stats64;

            hostStatsNg->reserved4 = 0U;
            hostStatsNg->reserved6 = 0U;
            hostStatsNg->reserved8 = 0U;
            hostStatsNg->reserved10 = 0U;
            memset(hostStatsNg->reserved17to19, 0, sizeof(hostStatsNg->reserved17to19));
            memset(hostStatsNg->reserved22to25, 0, sizeof(hostStatsNg->reserved22to25));
            memset(hostStatsNg->reserved57to95, 0, sizeof(hostStatsNg->reserved57to95));
        }
        break;

        default:
        {
            Enet_devAssert(false, "Unsupported peripheral type %u\n", hStats->enetType);
        }
        break;
    }
}

static void CpswStats_readMacStats(CpswStats_Handle hStats,
                                   Enet_MacPort macPort)
{
    CSL_Xge_cpswRegs *regs = (CSL_Xge_cpswRegs *)hStats->enetMod.virtAddr;
    union CSL_CPSW_STATS portStats;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    uint32_t portId = ENET_MACPORT_ID(macPort);
    uint64_t *stats64;
    uint32_t *stats32 = (uint32_t *)&portStats;
    uint32_t i;

    ENETTRACE_VAR(portId);
    Enet_devAssert(portNum < hStats->macPortNum, "Invalid MAC port %u\n", portId);
    Enet_devAssert(hStats->macPortStats != NULL, "Invalid MAC port stats memory address\n");

    /* CSL blindly reads all registers in the statistics block regardless
     * of whether they are applicable or not to a CPSW instance type */
    memset(&portStats.pn_stats, 0, sizeof(portStats.pn_stats));
    CSL_CPSW_getPortStats(regs, portNum + 1, &portStats);

    stats64 = &hStats->macPortStats[portNum].val[0U];
    for (i = 0U; i < CPSW_STATS_BLOCK_ELEM_NUM; i++)
    {
        stats64[i] += stats32[i];
    }

    /* Clear reserved fields */
    switch (hStats->enetType)
    {
        case ENET_CPSW_2G:
        {
            CpswStats_MacPort_2g *macStats2g = (CpswStats_MacPort_2g *)stats64;

            memset(macStats2g->reserved57to80, 0, sizeof(macStats2g->reserved57to80));
            memset(macStats2g->reserved87to95, 0, sizeof(macStats2g->reserved87to95));
        }
        break;

        case ENET_CPSW_3G:
        case ENET_CPSW_5G:
        case ENET_CPSW_9G:
        {
            CpswStats_MacPort_Ng *macStatsNg = (CpswStats_MacPort_Ng *)stats64;

            memset(macStatsNg->reserved57to80, 0, sizeof(macStatsNg->reserved57to80));
            memset(macStatsNg->reserved87to95, 0, sizeof(macStatsNg->reserved87to95));
        }
        break;

        default:
        {
            Enet_devAssert(false, "Unsupported peripheral type %u\n", hStats->enetType);
        }
        break;
    }
}

