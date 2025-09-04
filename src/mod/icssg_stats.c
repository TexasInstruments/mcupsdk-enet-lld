/*
 *  Copyright (c) Texas Instruments Incorporated 2021-2025
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
 * \file  icssg_stats.c
 *
 * \brief This file contains the implementation of ICSSG statistics.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <hw_include/cslr_icss.h>
#include <include/core/enet_base.h>
#include <include/core/enet_utils.h>
#include <include/core/enet_types.h>
#include <include/core/enet_ioctl.h>
#include <priv/mod/icssg_stats_priv.h>
#include <priv/mod/icssg_stats_ioctl_priv.h>
#include <src/per/firmware/icssg/fw_mem_map.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */



/*!
 * \brief Statistics Ioctl Handler Function.
 *
 * \param hPer      Enet Peripheral handle
 * \param cmd       IOCTL command Id
 * \param prms      IOCTL parameters
 */
typedef int32_t IcssgStatsIoctlHandlerFxn_t(IcssgStats_Handle hStats,
                                       uint32_t cmd,
                                       Enet_IoctlPrms *prms);

/*!
 * \brief Statistics IOCTL register Handler structure.
 */
typedef struct IcssgStatsIoctlHandlerTableEntry_s
{
    uint32_t cmd;
    IcssgStatsIoctlHandlerFxn_t *fxn;
} IcssgStatsIoctlHandlerTableEntry_t;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void IcssgStats_getPaStats(IcssgStats_Handle hStats,
                                  IcssgStats_Pa *paStats);

static int32_t IcssgStats_getMacPortStats(IcssgStats_Handle hStats,
                                       Enet_MacPort macPort,
                                       IcssgStats_MacPort *stats);

static int32_t IcssgStats_resetMacPortStats(IcssgStats_Handle hStats,
                                         Enet_MacPort macPort);

static IcssgStatsIoctlHandlerFxn_t * Icssg_getStatsIoctlHandler(IcssgStats_Handle hStats,
                                                                uint32_t cmd,
                                                                IcssgStatsIoctlHandlerTableEntry_t ioctlTbl[],
                                                                uint32_t numEntries);

int32_t IcssgStats_ioctl_handler_default(IcssgStats_Handle hStats,
                                            uint32_t cmd,
                                            Enet_IoctlPrms *prms);

int32_t IcssgStats_ioctl_handler_ICSSG_STATS_IOCTL_REGISTER_HANDLER(IcssgStats_Handle hStats,
                                                                        uint32_t cmd,
                                                                        Enet_IoctlPrms *prms);
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Register offsets for all 64-bit PA statistics */
static uint32_t IcssgStats_64bitpaStatsOffset[] =
{
    NRT_HOST_RX_BYTE_COUNT_PASTATID,
    NRT_HOST_TX_BYTE_COUNT_PASTATID,
    NRT_HOST_RX_BYTE_COUNT_MAC_SLICE0_PASTATID,
    NRT_HOST_RX_BYTE_COUNT_MAC_SLICE1_PASTATID,
    NRT_HOST_TX_BYTE_COUNT_MAC_SLICE0_PASTATID,
    NRT_HOST_TX_BYTE_COUNT_MAC_SLICE1_PASTATID,
};

/* Register offsets for all 32-bit PA statistics */
static uint32_t IcssgStats_paStatsOffset[] =
{
    NRT_HOST_RX_PKT_COUNT_PASTATID,
    NRT_HOST_TX_PKT_COUNT_PASTATID,
    NRT_HOST_RX_PKT_COUNT_MAC_SLICE0_PASTATID,
    NRT_HOST_RX_PKT_COUNT_MAC_SLICE1_PASTATID,
    NRT_HOST_TX_PKT_COUNT_MAC_SLICE0_PASTATID,
    NRT_HOST_TX_PKT_COUNT_MAC_SLICE1_PASTATID,
    NRT_RTU0_PACKET_DROPPED_SLICE0_PASTATID,
    NRT_RTU0_PACKET_DROPPED_SLICE1_PASTATID,
    NRT_PORT1_Q0_OVERFLOW_PASTATID,
    NRT_PORT1_Q1_OVERFLOW_PASTATID,
    NRT_PORT1_Q2_OVERFLOW_PASTATID,
    NRT_PORT1_Q3_OVERFLOW_PASTATID,
    NRT_PORT1_Q4_OVERFLOW_PASTATID,
    NRT_PORT1_Q5_OVERFLOW_PASTATID,
    NRT_PORT1_Q6_OVERFLOW_PASTATID,
    NRT_PORT1_Q7_OVERFLOW_PASTATID,
    NRT_PORT2_Q0_OVERFLOW_PASTATID,
    NRT_PORT2_Q1_OVERFLOW_PASTATID,
    NRT_PORT2_Q2_OVERFLOW_PASTATID,
    NRT_PORT2_Q3_OVERFLOW_PASTATID,
    NRT_PORT2_Q4_OVERFLOW_PASTATID,
    NRT_PORT2_Q5_OVERFLOW_PASTATID,
    NRT_PORT2_Q6_OVERFLOW_PASTATID,
    NRT_PORT2_Q7_OVERFLOW_PASTATID,
    NRT_HOST_Q0_OVERFLOW_PASTATID,
    NRT_HOST_Q1_OVERFLOW_PASTATID,
    NRT_HOST_Q2_OVERFLOW_PASTATID,
    NRT_HOST_Q3_OVERFLOW_PASTATID,
    NRT_HOST_Q4_OVERFLOW_PASTATID,
    NRT_HOST_Q5_OVERFLOW_PASTATID,
    NRT_HOST_Q6_OVERFLOW_PASTATID,
    NRT_HOST_Q7_OVERFLOW_PASTATID,
    NRT_HOST_EGRESS_Q_PRE_OVERFLOW_PASTATID,
    NRT_DROPPED_PKT_SLICE0_PASTATID,
    NRT_DROPPED_PKT_SLICE1_PASTATID,
    NRT_RX_ERROR_SLICE0_PASTATID,
    NRT_RX_ERROR_SLICE1_PASTATID,
    RX_EOF_RTU_DS_INVALID_SLICE0_PASTATID,
    RX_EOF_RTU_DS_INVALID_SLICE1_PASTATID,
    NRT_TX_PORT1_DROPPED_PACKET_PASTATID,
    NRT_TX_PORT2_DROPPED_PACKET_PASTATID,
    NRT_TX_PORT1_TS_DROPPED_PACKET_PASTATID,
    NRT_TX_PORT2_TS_DROPPED_PACKET_PASTATID,
    NRT_INF_PORT_DISABLED_SLICE0_PASTATID,
    NRT_INF_PORT_DISABLED_SLICE1_PASTATID,
    NRT_INF_SAV_SLICE0_PASTATID,
    NRT_INF_SAV_SLICE1_PASTATID,
    NRT_INF_SA_BL_SLICE0_PASTATID,
    NRT_INF_SA_BL_SLICE1_PASTATID,
    NRT_INF_PORT_BLOCKED_SLICE0_PASTATID,
    NRT_INF_PORT_BLOCKED_SLICE1_PASTATID,
    NRT_INF_AFT_DROP_TAGGED_SLICE0_PASTATID,
    NRT_INF_AFT_DROP_TAGGED_SLICE1_PASTATID,
    NRT_INF_AFT_DROP_PRIOTAGGED_SLICE0_PASTATID,
    NRT_INF_AFT_DROP_PRIOTAGGED_SLICE1_PASTATID,
    NRT_INF_AFT_DROP_NOTAG_SLICE0_PASTATID,
    NRT_INF_AFT_DROP_NOTAG_SLICE1_PASTATID,
    NRT_INF_AFT_DROP_NOTMEMBER_SLICE0_PASTATID,
    NRT_INF_AFT_DROP_NOTMEMBER_SLICE1_PASTATID,
    NRT_FDB_NO_SPACE_TO_LEARN,
    NRT_PREEMPT_BAD_FRAG_SLICE0_PASTATID,
    NRT_PREEMPT_BAD_FRAG_SLICE1_PASTATID,
    NRT_PREEMPT_ASSEMBLY_ERROR_SLICE0_PASTATID,
    NRT_PREEMPT_ASSEMBLY_ERROR_SLICE1_PASTATID,
    NRT_PREEMPT_FRAG_COUNT_TX_SLICE0_PASTATID,
    NRT_PREEMPT_FRAG_COUNT_TX_SLICE1_PASTATID,
    NRT_PREEMPT_ASSEMBLY_OK_SLICE0_PASTATID,
    NRT_PREEMPT_ASSEMBLY_OK_SLICE1_PASTATID,
    NRT_PREEMPT_FRAG_COUNT_RX_SLICE0_PASTATID,
    NRT_PREEMPT_FRAG_COUNT_RX_SLICE1_PASTATID,
    RX_EOF_SHORT_FRAMEERR_SLICE0_PASTATID,
    RX_EOF_SHORT_FRAMEERR_SLICE1_PASTATID,
    RX_B0_DROP_EARLY_EOF_SLICE0_PASTATID,
    RX_B0_DROP_EARLY_EOF_SLICE1_PASTATID,
    TX_JUMBO_FRAME_CUTOFF_SLICE0_PASTATID,
    TX_JUMBO_FRAME_CUTOFF_SLICE1_PASTATID,
    RX_EXPRESS_FRAG_Q_DROP_SLICE0_PASTATID,
    RX_EXPRESS_FRAG_Q_DROP_SLICE1_PASTATID,
    RX_FIFO_OVERRUN_SLICE0_PASTATID,
    RX_FIFO_OVERRUN_SLICE1_PASTATID,
    NRT_HOST_EGRESS_Q_PRE_OVERFLOW_MAC_SLICE0_PASTATID,
    NRT_HOST_EGRESS_Q_PRE_OVERFLOW_MAC_SLICE1_PASTATID,
    NRT_HOST_EGRESS_Q_EXP_OVERFLOW_MAC_SLICE0_PASTATID,
    NRT_HOST_EGRESS_Q_EXP_OVERFLOW_MAC_SLICE1_PASTATID,
    NRT_CUT_THR_PKT_SLICE0_PASTATID,
    NRT_CUT_THR_PKT_SLICE1_PASTATID
};

#define ICSSG_STATS_IOCTL_HANDLER_ENTRY_INIT(x)    \
          {.cmd = x,                            \
           .fxn = &IcssgStats_ioctl_handler_##x}

#define ICSSG_STATS_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(x)    \
          {.cmd = x,                            \
           .fxn = &IcssgStats_ioctl_handler_default}

static IcssgStatsIoctlHandlerTableEntry_t IcssgStatsIoctlHandlerTable[] =
{
    ICSSG_STATS_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_STATS_IOCTL_GET_HOSTPORT_STATS),
    ICSSG_STATS_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_STATS_IOCTL_GET_MACPORT_STATS),
    ICSSG_STATS_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_STATS_IOCTL_RESET_MACPORT_STATS),
    ICSSG_STATS_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_STATS_IOCTL_RESET_HOSTPORT_STATS),
    ICSSG_STATS_IOCTL_HANDLER_ENTRY_INIT(ICSSG_STATS_IOCTL_REGISTER_HANDLER)

};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t IcssgStats_open(IcssgStats_Handle hStats,
                        Enet_Type enetType,
                        uint32_t instId)
{
    int32_t status = ENET_SOK;
    ENETTRACE_VERBOSE("%s: Open module\n", hStats->name);

    if (hStats->magic == ENET_NO_MAGIC)
    {
        hStats->virtAddr  = (void *)EnetUtils_physToVirt(hStats->physAddr, NULL);
        hStats->virtAddr2 = (void *)EnetUtils_physToVirt(hStats->physAddr2, NULL);

        uintptr_t baseAddr = (uintptr_t)hStats->virtAddr;
        uintptr_t cfgRegs = baseAddr + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_REGS_BASE;

        /* Statistics addresses need to be set according to ICSSG Enet types:
        *  - Dual-MAC peripheral has single port, so it maps to port 1 or port 2 based on
        *    the peripheral instance id.
        *  - Switch peripheral is one-to-one mapping */
        if (enetType == ENET_ICSSG_SWITCH)
        {
            hStats->port1Addr = cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_STAT_GOOD_PRU0;
            hStats->port2Addr = cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_STAT_GOOD_PRU1;
        }
        else
        {
            if ((instId % 2U) == 0U)
            {
                hStats->port1Addr = cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_STAT_GOOD_PRU0;
            }
            else
            {
                hStats->port1Addr = cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_STAT_GOOD_PRU1;
            }

            hStats->port2Addr = 0U;
        }

        /* Set PA stats address */
        hStats->paStatsAddr = baseAddr + CSL_ICSS_G_PA_STAT_WRAP_PA_SLV_QSTAT_REGS_BASE;

        /* Clear MAC port stats */
        IcssgStats_resetMacPortStats(hStats, ENET_MAC_PORT_1);
        if (enetType == ENET_ICSSG_SWITCH)
        {
            IcssgStats_resetMacPortStats(hStats, ENET_MAC_PORT_2);
        }

        /* Enable PA_STAT block for diagnostic counters, 2 vi */
        CSL_REG32_WR((uint32_t *)(baseAddr + CSL_ICSS_G_PA_STAT_WRAP_PA_SLV_REGS_BASE + 8U),
                    (1U << 31U) | 2U);
        if (status == ENET_SOK)
        {
            hStats->magic = ENET_MAGIC;
            ENETTRACE_VERBOSE("%s: Module is now open\n", hStats->name);
        }
        else
        {
            ENETTRACE_ERR("%s: Failed to open: %d\n", hStats->name, status);
            hStats->magic = ENET_NO_MAGIC;
        }
    }
    else
    {
        ENETTRACE_ERR("%s: Module is already open\n", hStats->name);
        status = ENET_EALREADYOPEN;
    }
    return status;
}

void IcssgStats_close(IcssgStats_Handle hStats)
{
    ENETTRACE_VERBOSE("%s: Close module\n", hStats->name);

    if (hStats->magic == ENET_MAGIC)
    {
        hStats->magic = ENET_NO_MAGIC;
        ENETTRACE_VERBOSE("%s: Module is now closed\n", hStats->name);
    }
    else
    {
        ENETTRACE_ERR("%s: Module is not open\n", hStats->name);
    }
}

int32_t IcssgStats_rejoin(IcssgStats_Handle hStats,
                          Enet_Type enetType,
                          uint32_t instId)
{
    return ENET_ENOTSUPPORTED;
}

int32_t IcssgStats_ioctl(IcssgStats_Handle hStats,
                         uint32_t cmd,
                         Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;
    bool isStatsOpen = true;

    ENETTRACE_VERBOSE("%s: Do IOCTL 0x%08x prms %p\n", hStats->name, cmd, prms);

    isStatsOpen = (hStats->magic == ENET_MAGIC) ? true : false;
    if (isStatsOpen == true)
    {
        status = ENET_SOK;
        IcssgStatsIoctlHandlerFxn_t * ioctlHandler;

        ioctlHandler = Icssg_getStatsIoctlHandler(hStats, cmd, IcssgStatsIoctlHandlerTable , ENET_ARRAYSIZE(IcssgStatsIoctlHandlerTable));
        Enet_assert(ioctlHandler != NULL);
        status = ioctlHandler(hStats, cmd, prms);

        if(status != ENET_SOK)
        {
            ENETTRACE_ERR("%s: Failed to do IOCTL cmd 0x%08x: %d\n", hStats->name, cmd, status);
        }
    }
    else
    {
        ENETTRACE_ERR("%s: Module is not open\n", hStats->name);
    }

    return status;
}

static void IcssgStats_getPaStats(IcssgStats_Handle hStats,
                                  IcssgStats_Pa *paStats)
{
    uintptr_t baseAddr = (uintptr_t)hStats->paStatsAddr;
    uint32_t *stats64 = (uint32_t *)&paStats->hostRxByteCnt;
    uint32_t *stats32 = (uint32_t *)&paStats->hostRxPktCnt;
    uint32_t i;

    /* First two PA statistics counters are 64-bit values */
    for(i = 0U; i < ENET_ARRAYSIZE(IcssgStats_64bitpaStatsOffset); i++)
    {
        *stats64 = CSL_REG64_RD(baseAddr + IcssgStats_64bitpaStatsOffset[i]);
        stats64 += 2U;
    }

    /* Rest of PA statistics counters are 32-bit values */
    for (i = 0U; i < ENET_ARRAYSIZE(IcssgStats_paStatsOffset); i++)
    {
        *stats32 = CSL_REG32_RD(baseAddr + IcssgStats_paStatsOffset[i]);
        stats32++;
    }
}

static int32_t IcssgStats_getMacPortStats(IcssgStats_Handle hStats,
                                       Enet_MacPort macPort,
                                       IcssgStats_MacPort *stats)
{
    uint32_t *statsRegs;
    uint32_t *stats32 = (uint32_t *)stats;
    uint32_t i;
    int32_t status = ENET_SOK;

    if (macPort == ENET_MAC_PORT_1)
    {
        statsRegs = (uint32_t *)hStats->port1Addr;
    }
    else
    {
        // valid only for SWITCH mode
        statsRegs = (uint32_t *)hStats->port2Addr;
    }

    if (statsRegs == NULL)
    {
        status = ENET_EINVALIDPARAMS;
    }

    if (status == ENET_SOK)
    {
        for (i = 0U; i < sizeof(IcssgStats_MacPort) / sizeof(uint32_t); i++)
        {
            stats32[i] = CSL_REG32_RD(&statsRegs[i]);
        }
    }

    return status;
}

static int32_t IcssgStats_resetMacPortStats(IcssgStats_Handle hStats,
                                         Enet_MacPort macPort)
{
    uint32_t *statsRegs;
    uint32_t stats32;
    uint32_t i;
    int32_t status = ENET_SOK;

    if (macPort == ENET_MAC_PORT_1)
    {
        statsRegs = (uint32_t *)hStats->port1Addr;
    }
    else
    {
        statsRegs = (uint32_t *)hStats->port2Addr;
    }

    if (statsRegs == NULL)
    {
        status = ENET_EINVALIDPARAMS;
    }

    if (status == ENET_SOK)
    {
        for (i = 0U; i < sizeof(IcssgStats_MacPort) / sizeof(uint32_t); i++)
        {
            stats32 = CSL_REG32_RD(&statsRegs[i]);
            CSL_REG32_WR(&statsRegs[i], stats32);
        }
    }
    return status;
}

int32_t  IcssgStats_ioctl_handler_ENET_STATS_IOCTL_GET_HOSTPORT_STATS(IcssgStats_Handle hStats,
                                             uint32_t cmd,
                                             Enet_IoctlPrms *prms)
{
    Enet_assert(cmd == ENET_STATS_IOCTL_GET_HOSTPORT_STATS);

    IcssgStats_Pa *stats = (IcssgStats_Pa *)prms->outArgs;

    IcssgStats_getPaStats(hStats, stats);

    return ENET_SOK;

}

int32_t IcssgStats_ioctl_handler_ENET_STATS_IOCTL_GET_MACPORT_STATS(IcssgStats_Handle hStats,
                                             uint32_t cmd,
                                             Enet_IoctlPrms *prms)
{
    Enet_assert(cmd == ENET_STATS_IOCTL_GET_MACPORT_STATS);

    Enet_MacPort macPort = *(Enet_MacPort *)prms->inArgs;
    IcssgStats_MacPort *stats = (IcssgStats_MacPort *)prms->outArgs;

    return IcssgStats_getMacPortStats(hStats, macPort, stats);
}

int32_t IcssgStats_ioctl_handler_ENET_STATS_IOCTL_RESET_MACPORT_STATS(IcssgStats_Handle hStats,
                                             uint32_t cmd,
                                             Enet_IoctlPrms *prms)
{
    Enet_assert(cmd == ENET_STATS_IOCTL_RESET_MACPORT_STATS);

    Enet_MacPort macPort = *(Enet_MacPort *)prms->inArgs;

    return IcssgStats_resetMacPortStats(hStats, macPort);
}

int32_t IcssgStats_ioctl_handler_ENET_STATS_IOCTL_RESET_HOSTPORT_STATS(IcssgStats_Handle hStats,
                                                                        uint32_t cmd,
                                                                        Enet_IoctlPrms *prms)
{
    return ENET_ENOTSUPPORTED;
}

static int32_t Icssg_getStatsIoctlHandlerEntry(IcssgStats_Handle hStats, uint32_t cmd, IcssgStatsIoctlHandlerTableEntry_t ioctlTbl[], uint32_t numEntries, uint32_t *entryIdx)
{
    uint32_t i;
    int32_t status;

    for (i = 0; i < numEntries; i++)
    {
        if (ioctlTbl[i].cmd == cmd)
        {
            break;
        }
    }
    if (i < numEntries)
    {
        *entryIdx = i;
        status = ENET_SOK;
    }
    else
    {
        ENETTRACE_ERR("%s: failed to get ioctl handler for ICSSG STATS IOCTL cmd %x\r\n", cmd);
        status = ENET_EINVALIDPARAMS;
    }
    return status;
}

static IcssgStatsIoctlHandlerFxn_t * Icssg_getStatsIoctlHandler(IcssgStats_Handle hStats, uint32_t cmd, IcssgStatsIoctlHandlerTableEntry_t ioctlTbl[], uint32_t numEntries)
{
    uint32_t entryIndex;
    int32_t status;
    IcssgStatsIoctlHandlerFxn_t *ioctlHandler = NULL;

    status = Icssg_getStatsIoctlHandlerEntry(hStats, cmd, ioctlTbl, numEntries, &entryIndex);
    if (status == ENET_SOK)
    {
        Enet_assert(entryIndex < numEntries);
        ioctlHandler = ioctlTbl[entryIndex].fxn;
    }
    else
    {
        ENETTRACE_ERR("%s: failed to get ioctl handler for ICSSG STATS IOCTL cmd %x\r\n", cmd);
        ioctlHandler = &IcssgStats_ioctl_handler_default;
    }
    return ioctlHandler;
}


int32_t IcssgStats_ioctl_handler_default(IcssgStats_Handle hStats,
                                    uint32_t cmd,
                                    Enet_IoctlPrms *prms)
{
    return ENET_ENOTSUPPORTED;
}

int32_t IcssgStats_ioctl_handler_registerHandler(IcssgStats_Handle hStats,
                                                    uint32_t cmd,
                                                    Enet_IoctlPrms *prms)
{

    int32_t status = ENET_SOK;
    IcssgStatsIoctlHandlerTableEntry_t *ioctlHandlerToRegister  = (IcssgStatsIoctlHandlerTableEntry_t *)prms->inArgs;
    IcssgStatsIoctlHandlerTableEntry_t *currentIoctlTblEntry;
    uint32_t entryIndex;

    Enet_assert(cmd == ICSSG_STATS_IOCTL_REGISTER_HANDLER);
    status = Icssg_getStatsIoctlHandlerEntry(hStats, ioctlHandlerToRegister->cmd,
                                        IcssgStatsIoctlHandlerTable ,
                                        ENET_ARRAYSIZE(IcssgStatsIoctlHandlerTable),
                                        &entryIndex);
    if (status == ENET_SOK)
    {
        Enet_assert(entryIndex < ENET_ARRAYSIZE(IcssgStatsIoctlHandlerTable));
        currentIoctlTblEntry = &IcssgStatsIoctlHandlerTable[entryIndex];
        Enet_assert(ioctlHandlerToRegister->cmd == currentIoctlTblEntry->cmd);
        currentIoctlTblEntry->fxn = (IcssgStatsIoctlHandlerFxn_t *)ioctlHandlerToRegister->fxn;
    }
    else
    {
        ENETTRACE_ERR("%s: failed to get ioctl handler for ICSSG STATS IOCTL cmd %x\r\n", cmd);
        status = ENET_EINVALIDPARAMS;
    }
    return status;
}

int32_t IcssgStats_ioctl_handler_ICSSG_STATS_IOCTL_REGISTER_HANDLER(IcssgStats_Handle hStats,
                                                                        uint32_t cmd,
                                                                        Enet_IoctlPrms *prms)
{
    int32_t status;

    status = IcssgStats_ioctl_handler_registerHandler(hStats, cmd, prms);
    return status;
}
