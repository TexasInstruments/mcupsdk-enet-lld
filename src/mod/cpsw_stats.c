/*
 *  Copyright (c) Texas Instruments Incorporated 2020-23
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

/* Supported AM64x version */
#define CPSW_STATS_VER_REVMAJ_AM64X          (0x00000001U)
#define CPSW_STATS_VER_REVMIN_AM64X          (0x00000003U)
#define CPSW_STATS_VER_REVRTL_AM64X          (0x00000001U)
#define CPSW_STATS_VER_ID_AM64X              (0x00006BA8U)

/* Supported AM273X version */
#define CPSW_STATS_VER_REVMAJ_AM273X           (0x00000001U)
#define CPSW_STATS_VER_REVMIN_AM273X           (0x00000002U)
#define CPSW_STATS_VER_REVRTL_AM273X           (0x00000000U)
#define CPSW_STATS_VER_ID_AM273X               (0x00006B90U)

/* Supported AM263X version */
#define CPSW_STATS_VER_REVMAJ_AM263X           (0x00000001U)
#define CPSW_STATS_VER_REVMIN_AM263X           (0x00000003U)
#define CPSW_STATS_VER_REVRTL_AM263X           (0x00000002U)
#define CPSW_STATS_VER_ID_AM263X               (0x00006B90U)

/* Supported AWR294X version */
#define CPSW_STATS_VER_REVMAJ_AWR294X           (0x00000001U)
#define CPSW_STATS_VER_REVMIN_AWR294X           (0x00000002U)
#define CPSW_STATS_VER_REVRTL_AWR294X           (0x00000000U)
#define CPSW_STATS_VER_ID_AWR294X               (0x00006B90U)

/* Supported AWR2544 version */
#define CPSW_STATS_VER_REVMAJ_AWR2544           (0x00000001U)
#define CPSW_STATS_VER_REVMIN_AWR2544           (0x00000003U)
#define CPSW_STATS_VER_REVRTL_AWR2544           (0x00000003U)
#define CPSW_STATS_VER_ID_AWR2544               (0x00006B90U)

/* Supported AM62AX version */
#define CPSW_STATS_VER_REVMAJ_AM62AX           (0x00000001U)
#define CPSW_STATS_VER_REVMIN_AM62AX           (0x00000003U)
#define CPSW_STATS_VER_REVRTL_AM62AX           (0x00000002U)
#define CPSW_STATS_VER_ID_AM62AX               (0x00006BA8U)

#define CPSW_STATS_IOCTL_HANDLER_ENTRY_INIT(x)    \
          {.cmd = x,                            \
           .fxn = &CpswStats_ioctl_handler_##x}

#define CPSW_STATS_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(x)    \
          {.cmd = x,                            \
           .fxn = &CpswStats_ioctl_handler_default}

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef int32_t (CpswStatsIoctlHandler)(CpswStats_Handle hStats, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);

typedef struct CpswStatsIoctlHandlerRegistry_s
{
    uint32_t cmd;
    CpswStatsIoctlHandler *fxn;
} CpswStatsIoctlHandlerRegistry_t;


/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

#if ENET_CFG_IS_ON(DEV_ERROR)
static int32_t CpswStats_isSupported(CSL_Xge_cpswRegs *regs);
#endif
static int32_t CpswStats_ioctl_handler_default(CpswStats_Handle hStats, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
static int32_t CpswStats_ioctl_handler_CPSW_STATS_IOCTL_REGISTER_HANDLER(CpswStats_Handle hStats, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
static int32_t CpswStats_setIoctlHandlerFxn(uint32_t ioctlCmd,
                                            CpswStatsIoctlHandler *ioctlHandlerFxn,
                                            CpswStatsIoctlHandlerRegistry_t *ioctlRegistryTbl,
                                            uint32_t tableSize);
static CpswStatsIoctlHandler * CpswStats_getIoctlHandlerFxn(uint32_t ioctlCmd, CpswStatsIoctlHandlerRegistry_t *ioctlRegistryTbl, uint32_t tableSize);
static int32_t CpswStats_getIoctlHandlerIdx(uint32_t ioctlCmd, CpswStatsIoctlHandlerRegistry_t *ioctlRegistryTbl, uint32_t tableSize, uint32_t *tblIdx);


/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

#if ENET_CFG_IS_ON(DEV_ERROR)
/*! \brief CPSW statistics versions supported by this driver. */
static CSL_CPSW_VERSION CpswStats_gSupportedVer[] =
{
    {   /* AM64x */
        .majorVer = CPSW_STATS_VER_REVMAJ_AM64X,
        .minorVer = CPSW_STATS_VER_REVMIN_AM64X,
        .rtlVer   = CPSW_STATS_VER_REVRTL_AM64X,
        .id       = CPSW_STATS_VER_ID_AM64X,
    },
    {   /* AM273X */
        .majorVer = CPSW_STATS_VER_REVMAJ_AM273X,
        .minorVer = CPSW_STATS_VER_REVMIN_AM273X,
        .rtlVer   = CPSW_STATS_VER_REVRTL_AM273X,
        .id       = CPSW_STATS_VER_ID_AM273X,
    },
	{   /* AM263X */
        .majorVer = CPSW_STATS_VER_REVMAJ_AM263X,
        .minorVer = CPSW_STATS_VER_REVMIN_AM263X,
        .rtlVer   = CPSW_STATS_VER_REVRTL_AM263X,
        .id       = CPSW_STATS_VER_ID_AM263X,
    },
    {   /* AWR294X */
        .majorVer = CPSW_STATS_VER_REVMAJ_AWR294X,
        .minorVer = CPSW_STATS_VER_REVMIN_AWR294X,
        .rtlVer   = CPSW_STATS_VER_REVRTL_AWR294X,
        .id       = CPSW_STATS_VER_ID_AWR294X,
    },
    {   /* AWR2544 */
        .majorVer = CPSW_STATS_VER_REVMAJ_AWR2544,
        .minorVer = CPSW_STATS_VER_REVMIN_AWR2544,
        .rtlVer   = CPSW_STATS_VER_REVRTL_AWR2544,
        .id       = CPSW_STATS_VER_ID_AWR2544,
    },
    {   /* AM62AX */
        .majorVer = CPSW_STATS_VER_REVMAJ_AM62AX,
        .minorVer = CPSW_STATS_VER_REVMIN_AM62AX,
        .rtlVer   = CPSW_STATS_VER_REVRTL_AM62AX,
        .id       = CPSW_STATS_VER_ID_AM62AX,
    },
};

/* Public statistics IOCTL validation data. */
static Enet_IoctlValidate gCpswStats_ioctlValidate[] =
{
    ENET_IOCTL_VALID_PRMS(ENET_STATS_IOCTL_GET_HOSTPORT_STATS,
                          0U,
                          sizeof(CpswStats_PortStats)),

    ENET_IOCTL_VALID_PRMS(ENET_STATS_IOCTL_GET_MACPORT_STATS,
                          sizeof(Enet_MacPort),
                          sizeof(CpswStats_PortStats)),

    ENET_IOCTL_VALID_PRMS(ENET_STATS_IOCTL_RESET_HOSTPORT_STATS,
                          0U,
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_STATS_IOCTL_RESET_MACPORT_STATS,
                          sizeof(Enet_MacPort),
                          0U),
};

/* Private statistics IOCTL validation data. */
static Enet_IoctlValidate gCpswStats_privIoctlValidate[] =
{
    ENET_IOCTL_VALID_PRMS(CPSW_STATS_IOCTL_SYNC,
                          0U,
                          0U),

    ENET_IOCTL_VALID_PRMS(CPSW_STATS_IOCTL_REGISTER_HANDLER,
                          sizeof(Enet_IoctlRegisterHandlerInArgs),
                          0U),
};
#endif

static CpswStatsIoctlHandlerRegistry_t CpswStatsIoctlHandlerRegistry[] =
{
    CPSW_STATS_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_STATS_IOCTL_GET_VERSION),
    CPSW_STATS_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_STATS_IOCTL_PRINT_REGS),
    CPSW_STATS_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_STATS_IOCTL_GET_HOSTPORT_STATS),
    CPSW_STATS_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_STATS_IOCTL_GET_MACPORT_STATS),
    CPSW_STATS_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_STATS_IOCTL_RESET_HOSTPORT_STATS),
    CPSW_STATS_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_STATS_IOCTL_RESET_MACPORT_STATS),
    CPSW_STATS_IOCTL_HANDLER_ENTRY_INIT(CPSW_STATS_IOCTL_SYNC), /* Register ISR callback IOCTLS by default */
    CPSW_STATS_IOCTL_HANDLER_ENTRY_INIT(CPSW_STATS_IOCTL_REGISTER_HANDLER),
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t CpswStats_open(EnetMod_Handle hMod,
                       Enet_Type enetType,
                       uint32_t instId,
                       const void *cfg,
                       uint32_t cfgSize)
{
    CpswStats_Handle hStats = (CpswStats_Handle)hMod;
    CSL_Xge_cpswRegs *regs = (CSL_Xge_cpswRegs *)hMod->virtAddr;
    CSL_CPSW_PORTSTAT portStat;
    uint32_t i;
#if ENET_CFG_IS_ON(DEV_ERROR)
    int32_t status = ENET_SOK;
#endif

    Enet_devAssert(cfgSize == 0U,
                   "Invalid stats config params size %u (expected %u)\n",
                   cfgSize, 0U);

    Enet_devAssert(regs != NULL, "CPSW stats regs address is not valid\n");

    /* Check supported stats module versions */
#if ENET_CFG_IS_ON(DEV_ERROR)
    status = CpswStats_isSupported(regs);
    Enet_devAssert(status == ENET_SOK, "Stats version is not supported\n");
#endif

    memset(&portStat, 0, sizeof(CSL_CPSW_PORTSTAT));

    /* For now, use statistics block memories from stats module object */
    hStats->hostPortStats = &hStats->hostPortStatsMem;
    hStats->macPortStats = &hStats->macPortStatsMem[0U];
    hStats->macPortNum = ENET_ARRAYSIZE(hStats->macPortStatsMem);

    hStats->enetType = enetType;

    /* Enable statistics on all applicable ports */
    portStat.p0StatEnable = true;
    if (enetType == ENET_CPSW_9G)
    {
        portStat.p1StatEnable = true;
        portStat.p2StatEnable = true;
        portStat.p3StatEnable = true;
        portStat.p4StatEnable = true;
        portStat.p5StatEnable = true;
        portStat.p6StatEnable = true;
        portStat.p7StatEnable = true;
        portStat.p8StatEnable = true;
    }
    else if (enetType == ENET_CPSW_5G)
    {
        portStat.p1StatEnable = true;
        portStat.p2StatEnable = true;
        portStat.p3StatEnable = true;
        portStat.p4StatEnable = true;
    }
    else if (enetType == ENET_CPSW_3G)
    {
        portStat.p1StatEnable = true;
        portStat.p2StatEnable = true;
    }
    else
    {
        portStat.p1StatEnable = true;
    }

    CSL_CPSW_setPortStatsEnableReg(regs, &portStat);

    /* Clear all statistics counters */
    CpswStats_resetHostStats(hStats);
    for (i = 0U; i < hStats->macPortNum; i++)
    {
        CpswStats_resetMacStats(hStats, ENET_MACPORT_DENORM(i));
    }

    return ENET_SOK;
}

int32_t CpswStats_rejoin(EnetMod_Handle hMod,
                         Enet_Type enetType,
                         uint32_t instId)
{
    CpswStats_Handle hStats = (CpswStats_Handle)hMod;

    /* For now, use statistics block memories from stats module object */
    hStats->hostPortStats = &hStats->hostPortStatsMem;
    hStats->macPortStats = &hStats->macPortStatsMem[0U];
    hStats->macPortNum = ENET_ARRAYSIZE(hStats->macPortStatsMem);

    hStats->enetType = enetType;

    /* Clear only host statistics counters */
    CpswStats_resetHostStats(hStats);

    return ENET_SOK;
}

void CpswStats_close(EnetMod_Handle hMod)
{
    /* Nothing to do */
}

void CpswStats_saveCtxt(EnetMod_Handle hMod)
{
    CpswStats_close(hMod);
}

int32_t CpswStats_restoreCtxt(EnetMod_Handle hMod,
                              Enet_Type enetType,
                              uint32_t instId,
                              const void *cfg,
                              uint32_t cfgSize)
{
    int32_t status = ENET_SOK;
    status = CpswStats_open(hMod, enetType, instId, cfg, cfgSize);
    return status;
}
int32_t CpswStats_ioctl(EnetMod_Handle hMod,
                        uint32_t cmd,
                        Enet_IoctlPrms *prms)
{
    CpswStats_Handle hStats = (CpswStats_Handle)hMod;
    CSL_Xge_cpswRegs *regs = (CSL_Xge_cpswRegs *)hMod->virtAddr;
    int32_t status = ENET_SOK;

#if ENET_CFG_IS_ON(DEV_ERROR)
    /* Validate CPSW statistics IOCTL parameters */
    if (ENET_IOCTL_GET_PER(cmd) == ENET_IOCTL_PER_CPSW)
    {
        if (ENET_IOCTL_GET_TYPE(cmd) == ENET_IOCTL_TYPE_PUBLIC)
        {
            status = Enet_validateIoctl(cmd, prms,
                                        gCpswStats_ioctlValidate,
                                        ENET_ARRAYSIZE(gCpswStats_ioctlValidate));
        }
        else
        {
            status = Enet_validateIoctl(cmd, prms,
                                        gCpswStats_privIoctlValidate,
                                        ENET_ARRAYSIZE(gCpswStats_privIoctlValidate));
        }

        ENETTRACE_ERR_IF(status != ENET_SOK, "IOCTL 0x%08x params are not valid\n", cmd);
    }
#endif

    if (status == ENET_SOK)
    {
        CpswStatsIoctlHandler * ioctlHandlerFxn;

        Enet_devAssert(regs != NULL, "CPSW stats regs address is not valid\n");

        ioctlHandlerFxn = CpswStats_getIoctlHandlerFxn(cmd, CpswStatsIoctlHandlerRegistry, ENET_ARRAYSIZE(CpswStatsIoctlHandlerRegistry));
        Enet_devAssert(ioctlHandlerFxn != NULL);
        status = ioctlHandlerFxn(hStats, regs, prms);
    }

    return status;
}

#if ENET_CFG_IS_ON(DEV_ERROR)
static int32_t CpswStats_isSupported(CSL_Xge_cpswRegs *regs)
{
    CSL_CPSW_VERSION version;
    uint32_t i;
    int32_t status = ENET_ENOTSUPPORTED;

    CSL_CPSW_getCpswVersionInfo(regs, &version);

    for (i = 0U; i < ENET_ARRAYSIZE(CpswStats_gSupportedVer); i++)
    {
        if ((version.majorVer == CpswStats_gSupportedVer[i].majorVer) &&
            (version.minorVer == CpswStats_gSupportedVer[i].minorVer) &&
            (version.rtlVer == CpswStats_gSupportedVer[i].rtlVer) &&
            (version.id == CpswStats_gSupportedVer[i].id))
        {
            status = ENET_SOK;
            break;
        }
    }

    return status;
}
#endif

void CpswStats_resetHostStats(CpswStats_Handle hStats)
{
    CSL_Xge_cpswRegs *regs = (CSL_Xge_cpswRegs *)hStats->enetMod.virtAddr;
    union CSL_CPSW_STATS portStats;

    Enet_devAssert(hStats->hostPortStats != NULL, "Invalid host port stats memory address\n");

    /* Clear local stats */
    memset(hStats->hostPortStats, 0, sizeof(CpswStats_PortStats));

    /* Clear hardware stats through a dummy read */
    memset(&portStats.p0_stats, 0, sizeof(portStats.p0_stats));
    CSL_CPSW_getPortStats(regs, 0U, &portStats);
}

void CpswStats_resetMacStats(CpswStats_Handle hStats,
                                    Enet_MacPort macPort)
{
    CSL_Xge_cpswRegs *regs = (CSL_Xge_cpswRegs *)hStats->enetMod.virtAddr;
    union CSL_CPSW_STATS portStats;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);

    Enet_devAssert(portNum < hStats->macPortNum, "Invalid port number %u\n", portNum);
    Enet_devAssert(hStats->macPortStats != NULL, "Invalid MAC port stats memory address\n");

    /* Clear local stats */
    memset(&hStats->macPortStats[portNum], 0, sizeof(CpswStats_PortStats));

    /* Clear hardware stats through a dummy read */
    memset(&portStats.pn_stats, 0, sizeof(portStats.pn_stats));
    CSL_CPSW_getPortStats(regs, portNum + 1, &portStats);
}


static int32_t CpswStats_getIoctlHandlerIdx(uint32_t ioctlCmd, CpswStatsIoctlHandlerRegistry_t *ioctlRegistryTbl, uint32_t tableSize, uint32_t *tblIdx)
{
    int32_t status;
    uint32_t i;

    for (i = 0; i < tableSize; i++)
    {
        if (ioctlRegistryTbl[i].cmd == ioctlCmd)
        {
            break;
        }
    }
    if (i < tableSize)
    {
        *tblIdx = i;
        status = ENET_SOK;
    }
    else
    {
        status = ENET_EINVALIDPARAMS;
    }
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to get ioctl registry index for ioctl cmd: %x\n", ioctlCmd);
    return status;
}

static CpswStatsIoctlHandler * CpswStats_getIoctlHandlerFxn(uint32_t ioctlCmd, CpswStatsIoctlHandlerRegistry_t *ioctlRegistryTbl, uint32_t tableSize)
{
    int32_t status;
    uint32_t tblIdx;
    CpswStatsIoctlHandler *handlerFxn = NULL;

    status = CpswStats_getIoctlHandlerIdx(ioctlCmd, ioctlRegistryTbl, tableSize, &tblIdx);
    if (ENET_SOK == status)
    {
        Enet_devAssert(tblIdx < tableSize, "Invalid IOCTL registry table index for IOCTL cmd:%x", ioctlCmd);
        handlerFxn = ioctlRegistryTbl[tblIdx].fxn;
    }
    else
    {
        handlerFxn = &CpswStats_ioctl_handler_default;
    }
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to get ioctl handler for ioctl cmd: %x. Using default ioctl handler \r\n", ioctlCmd);
    return handlerFxn;
}

static int32_t CpswStats_setIoctlHandlerFxn(uint32_t ioctlCmd,
                                            CpswStatsIoctlHandler *ioctlHandlerFxn,
                                            CpswStatsIoctlHandlerRegistry_t *ioctlRegistryTbl,
                                            uint32_t tableSize)
{
    int32_t status;
    uint32_t tblIdx;

    status = CpswStats_getIoctlHandlerIdx(ioctlCmd, ioctlRegistryTbl, tableSize, &tblIdx);
    if (ENET_SOK == status)
    {
        Enet_devAssert((tblIdx < tableSize), "Invalid IOCTL registry table index for IOCTL cmd:%x", ioctlCmd);
        Enet_devAssert((ioctlCmd == ioctlRegistryTbl[tblIdx].cmd), "Ioctl table corrupted");
        ioctlRegistryTbl[tblIdx].fxn = ioctlHandlerFxn;
    }
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to set ioctl handler for ioctl cmd: %x \r\n", ioctlCmd);
    return status;
}

static int32_t CpswStats_ioctl_handler_CPSW_STATS_IOCTL_REGISTER_HANDLER(CpswStats_Handle hStats, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    const Enet_IoctlRegisterHandlerInArgs *inArgs = (const Enet_IoctlRegisterHandlerInArgs *)prms->inArgs;
    int32_t status;

    status = CpswStats_setIoctlHandlerFxn(inArgs->cmd,
                                        (CpswStatsIoctlHandler *)inArgs->fxn,
                                        CpswStatsIoctlHandlerRegistry,
                                        ENET_ARRAYSIZE(CpswStatsIoctlHandlerRegistry));
    return status;
}


static int32_t CpswStats_ioctl_handler_default(CpswStats_Handle hStats, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    return ENET_ENOTSUPPORTED;
}



