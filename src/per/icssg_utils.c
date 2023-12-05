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
 * \file  icssg_utils.c
 *
 * \brief This file contains the implementation of the ICSSG EnetPer utilities.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <drivers/hw_include/cslr_soc.h>
#include <hw_include/cslr_icss.h>
#include <enet_cfg.h>
#include <include/core/enet_utils.h>
#include <include/core/enet_soc.h>
#include <priv/core/enet_trace_priv.h>
#include <include/per/icssg.h>
#include <priv/per/icssg_priv.h>
#include <src/per/firmware/icssg/fw_mem_map.h>
#include <drivers/pruicss.h>
#include <kernel/dpl/ClockP.h>

#include "icssg_utils.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Retry count for completion of asynchronous IOCTLs */
#define ICSSG_ASYNC_RETRY_CNT                  (10U)

/* Specifies no R30 over DMEM command */
#define ICSSG_UTILS_CMD_NONE                   (0xffff0000U)

/* Length in words of R30 command */
#define ICSSG_UTILS_R30_CMD_LEN                (4U)

/* 64 registers, 1 per queue */
#define ICSSG_UTILS_HWQ_MAX                    ((int32_t)64)
/* 16 peekable hw queues, currently not used */

/* 16 peekable hw queues, currently not used */
#define ICSSG_UTILS_HWQ_PEEK_MAX               ((int32_t)16)

/* Macros for FW configuraton  initializtion */
#define ICSSG_FW_CFG_MAX_HWQ                        ((uint32_t)8U)
#define ICSSG_FW_CFG_RECYCLE_Q_SLICE0               ((uint32_t)16U)
#define ICSSG_FW_CFG_RECYCLE_Q_SLICE1               ((uint32_t)17U)
#define ICSSG_FW_CFG_PORT_HF_Q_SLICE0               ((uint32_t)33U)
#define ICSSG_FW_CFG_PORT_HF_Q_SLICE1               ((uint32_t)36U)
#define ICSSG_FW_CFG_PORT_LF_Q_SLICE0               ((uint32_t)33U)
#define ICSSG_FW_CFG_PORT_LF_Q_SLICE1               ((uint32_t)37U)
#define ICSSG_FW_CFG_HOST_HF_Q_SLICE0               ((uint32_t)34U)
#define ICSSG_FW_CFG_HOST_HF_Q_SLICE1               ((uint32_t)38U)
#define ICSSG_FW_CFG_HOST_LF_Q_SLICE0               ((uint32_t)35U)
#define ICSSG_FW_CFG_HOST_LF_Q_SLICE1               ((uint32_t)39U)
#define ICSSG_FW_CFG_HOST_SF_Q_SLICE0               ((uint32_t)40U)
#define ICSSG_FW_CFG_HOST_SF_Q_SLICE1               ((uint32_t)41U)
#define ICSSG_FW_CFG_RECYCLE_Q_SLICE0_OFFSET        ((uint32_t)4U * 16U)
#define ICSSG_FW_CFG_RECYCLE_Q_SLICE1_OFFSET        ((uint32_t)4U * 17U)
#define ICSSG_FW_CFG_PORT_HF_Q_SLICE0_OFFSET        ((uint32_t)4U * 32U)
#define ICSSG_FW_CFG_PORT_HF_Q_SLICE1_OFFSET        ((uint32_t)4U * 36U)
#define ICSSG_FW_CFG_PORT_LF_Q_SLICE0_OFFSET        ((uint32_t)4U * 33U)
#define ICSSG_FW_CFG_PORT_LF_Q_SLICE1_OFFSET        ((uint32_t)4U * 37U)
#define ICSSG_FW_CFG_HOST_HF_Q_SLICE0_OFFSET        ((uint32_t)4U * 34U)
#define ICSSG_FW_CFG_HOST_HF_Q_SLICE1_OFFSET        ((uint32_t)4U * 38U)
#define ICSSG_FW_CFG_HOST_LF_Q_SLICE0_OFFSET        ((uint32_t)4U * 35U)
#define ICSSG_FW_CFG_HOST_LF_Q_SLICE1_OFFSET        ((uint32_t)4U * 39U)
#define ICSSG_FW_CFG_HOST_SF_Q_SLICE0_OFFSET        ((uint32_t)4U * 40U)
#define ICSSG_FW_CFG_HOST_SF_Q_SLICE1_OFFSET        ((uint32_t)4U * 41U)
#define ICSSG_FW_CFG_NORMAL_PD_SIZE                 ((uint32_t)8U)
#define ICSSG_FW_CFG_SPECIAL_PD_SIZE                ((uint32_t)20U)
#define ICSSG_FW_CFG_NORMAL_PD_SIZE_BYTE_COUNT      ((uint32_t)2U)
#define ICSSG_FW_CFG_SPECIAL_PD_SIZE_BYTE_COUNT     ((uint32_t)5U)
#define ICSSG_FW_CFG_DUAL_MAC_START_BUFFER_POOL_NUM ((uint32_t)8U)

/* Macros for classifier programming */
#define ICSSG_UTILS_NUM_CLASSIFIERS         ((uint32_t)16U)
#define ICSSG_UTILS_CLASSI_CFG1_OFF         (CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS_CFG1_PRU0 - \
                                             CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS0_AND_EN_PRU0)
#define ICSSG_UTILS_CLASSI_CFG2_OFF         ((uint32_t)4U)
#define ICSSG_UTILS_CLASSI_GATE_OFF         ((uint32_t)8U)
#define ICSSG_UTILS_CLASSI_AND_EN_OFF       ((uint32_t)0U)
#define ICSSG_UTILS_CLASSI_OR_EN_OFF        ((uint32_t)4U)

#define ICSSG_SWITCH_NORMAL_PD_SIZE                 (8U)
#define ICSSG_SWITCH_SPECIAL_PD_SIZE                (20U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* Command definition to program ICSSG R30 register */
typedef struct IcssgUtils_R30Cmd_s
{
    volatile uint32_t cmd[ICSSG_UTILS_R30_CMD_LEN];
} IcssgUtils_R30Cmd;

typedef struct IcssgUtils_HwqMap_s
{
    volatile uint32_t hwq[ICSSG_UTILS_HWQ_MAX];
    volatile uint32_t hwq_peek[ICSSG_UTILS_HWQ_PEEK_MAX];
    volatile uint32_t hwq_len[ICSSG_UTILS_HWQ_MAX];
    volatile uint32_t reset;
} IcssgUtils_HwqIpMmap;

struct IcssgUtils_BufPoolCfg_s
{
    uint32_t poolBase;
    uint32_t poolLen;
} IcssgUtils_BufPoolCfg;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void IcssgUtils_pdInitAndPush(Icssg_Handle hIcssg,
                                     uint32_t pool_addr,
                                     uint32_t ofs,
                                     uint32_t w0,
                                     uint32_t bdSize,
                                     uint32_t bdNum);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static IcssgUtils_R30Cmd IcssgUtils_R30Bitmask[ICSSG_UTILS_MAX_COMMANDS] =
{
    [ICSSG_UTILS_R30_CMD_DISABLE] =
    {
        { 0xffff0004, 0xffff0100, 0xffff0104, ICSSG_UTILS_CMD_NONE }
    },

    [ICSSG_UTILS_R30_CMD_BLOCK] =
    {
        { 0xfffb0040, 0xfeff0200, 0xfefb0208, ICSSG_UTILS_CMD_NONE }
    },

    [ICSSG_UTILS_R30_CMD_FORWARD] =
    {
        { 0xffbb0000, 0xfcff0000, 0xdcf30000, ICSSG_UTILS_CMD_NONE }
    },

    [ICSSG_UTILS_R30_CMD_FORWARD_WO_LEARNING] =
    {
        { 0xffbb0000, 0xfcff0000, 0xfcf32000, ICSSG_UTILS_CMD_NONE }
    },

    [ICSSG_UTILS_R30_CMD_ACCEPT_ALL] =
    {
        { 0xffff0001, ICSSG_UTILS_CMD_NONE, 0xffff0001, ICSSG_UTILS_CMD_NONE }
    },

    [ICSSG_UTILS_R30_CMD_ACCEPT_TAGGED] =
    {
        { 0xfffe0002, ICSSG_UTILS_CMD_NONE, 0xfffe0002, ICSSG_UTILS_CMD_NONE }
    },

    [ICSSG_UTILS_R30_CMD_ACCEPT_UNTAGGED_N_PRIO] =
    {
        { 0xfffc0000, ICSSG_UTILS_CMD_NONE, 0xfffc0000, ICSSG_UTILS_CMD_NONE }
    },

    [ICSSG_UTILS_R30_CMD_TAS_TRIGGER] =
    {
        { ICSSG_UTILS_CMD_NONE, 0xffff0020, ICSSG_UTILS_CMD_NONE, ICSSG_UTILS_CMD_NONE }
    },

    [ICSSG_UTILS_R30_CMD_TAS_ENABLE] =
    {
        { ICSSG_UTILS_CMD_NONE, 0xdfff1000, ICSSG_UTILS_CMD_NONE, ICSSG_UTILS_CMD_NONE }
    },

    [ICSSG_UTILS_R30_CMD_TAS_RESET] =
    {
        { ICSSG_UTILS_CMD_NONE, 0xefff2000, ICSSG_UTILS_CMD_NONE, ICSSG_UTILS_CMD_NONE }
    },

    [ICSSG_UTILS_R30_CMD_TAS_DISABLE] =
    {
        { ICSSG_UTILS_CMD_NONE, 0xcfff0000, ICSSG_UTILS_CMD_NONE, ICSSG_UTILS_CMD_NONE }
    },

    [ICSSG_UTILS_R30_CMD_UC_FLOODING_ENABLE] =
    {
        { ICSSG_UTILS_CMD_NONE, ICSSG_UTILS_CMD_NONE, 0xffff0400, ICSSG_UTILS_CMD_NONE }
    },

    [ICSSG_UTILS_R30_CMD_UC_FLOODING_DISABLE] =
    {
        { ICSSG_UTILS_CMD_NONE, ICSSG_UTILS_CMD_NONE, 0xfbff0000, ICSSG_UTILS_CMD_NONE }
    },

    [ICSSG_UTILS_R30_CMD_MC_FLOODING_ENABLE] =
    {
        { ICSSG_UTILS_CMD_NONE, ICSSG_UTILS_CMD_NONE, 0xffff0800, ICSSG_UTILS_CMD_NONE }
    },

    [ICSSG_UTILS_R30_CMD_MC_FLOODING_DISABLE] =
    {
        { ICSSG_UTILS_CMD_NONE, ICSSG_UTILS_CMD_NONE, 0xf7ff0000, ICSSG_UTILS_CMD_NONE }
    },

    [ICSSG_UTILS_R30_CMD_PREMPT_TX_ENABLE] =
    {
        { ICSSG_UTILS_CMD_NONE, 0xffff4000, ICSSG_UTILS_CMD_NONE, ICSSG_UTILS_CMD_NONE }
    },

    [ICSSG_UTILS_R30_CMD_PREMPT_TX_DISABLE] =
    {
        { ICSSG_UTILS_CMD_NONE, 0xbfff0000, ICSSG_UTILS_CMD_NONE, ICSSG_UTILS_CMD_NONE }
    },

    [ICSSG_UTILS_R30_CMD_VLAN_AWARE_ENABLE] =
    {
    { 0xffff0010, ICSSG_UTILS_CMD_NONE, 0xffff0010, ICSSG_UTILS_CMD_NONE },
    },

    [ICSSG_UTILS_R30_CMD_VLAN_AWARE_DISABLE] =
    {
        { 0xffef0000, ICSSG_UTILS_CMD_NONE, 0xffef0000, ICSSG_UTILS_CMD_NONE},
    },

    [ICSSG_UTILS_R30_CMD_DSCP_ENABLE] =
    {
        { 0xffff0040, ICSSG_UTILS_CMD_NONE, 0xffff0040, ICSSG_UTILS_CMD_NONE },
    },

    [ICSSG_UTILS_R30_CMD_DSCP_DISABLE] =
    {
        { 0xffbf0000, ICSSG_UTILS_CMD_NONE, 0xffbf0000, ICSSG_UTILS_CMD_NONE},
    },
};

IcssgUtils_R30Cmd *IcssgUtils_R30DmemAddr[ICSSG_INSTANCE_NUM][ICSSG_MAC_PORT_MAX] =
{
    {
        (IcssgUtils_R30Cmd *)(CSL_PRU_ICSSG0_DRAM0_SLV_RAM_BASE + MGR_R30_CMD_OFFSET),
        (IcssgUtils_R30Cmd *)(CSL_PRU_ICSSG0_DRAM1_SLV_RAM_BASE + MGR_R30_CMD_OFFSET)
    },
    {
        (IcssgUtils_R30Cmd *)(CSL_PRU_ICSSG1_DRAM0_SLV_RAM_BASE + MGR_R30_CMD_OFFSET),
        (IcssgUtils_R30Cmd *)(CSL_PRU_ICSSG1_DRAM1_SLV_RAM_BASE + MGR_R30_CMD_OFFSET)
    },
#ifdef SOC_AM65XX
    {
        (IcssgUtils_R30Cmd *)(CSL_PRU_ICSSG2_DRAM0_SLV_RAM_BASE + MGR_R30_CMD_OFFSET),
        (IcssgUtils_R30Cmd *)(CSL_PRU_ICSSG2_DRAM1_SLV_RAM_BASE + MGR_R30_CMD_OFFSET)
    },
#endif
};

/* Desricptors are in smem, there are 8 per hw q in our implemenation */
#if defined(SOC_AM65XX)
static uint32_t IcssgUtils_hwq_smem_start[ICSSG_INSTANCE_NUM] = { CSL_PRU_ICSSG0_RAM_SLV_RAM_BASE, CSL_PRU_ICSSG1_RAM_SLV_RAM_BASE, CSL_PRU_ICSSG2_RAM_SLV_RAM_BASE };
static IcssgUtils_HwqIpMmap *IcssgUtils_hwq_a[ICSSG_INSTANCE_NUM] =
{
    (IcssgUtils_HwqIpMmap *)(CSL_PRU_ICSSG0_DRAM0_SLV_RAM_BASE + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_REGS_BASE + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_QUEUE0),
    (IcssgUtils_HwqIpMmap *)(CSL_PRU_ICSSG1_DRAM0_SLV_RAM_BASE + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_REGS_BASE + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_QUEUE0),
    (IcssgUtils_HwqIpMmap *)(CSL_PRU_ICSSG2_DRAM0_SLV_RAM_BASE + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_REGS_BASE + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_QUEUE0)
};
#else
static uint32_t IcssgUtils_hwq_smem_start[ICSSG_INSTANCE_NUM] = { CSL_PRU_ICSSG0_RAM_SLV_RAM_BASE, CSL_PRU_ICSSG1_RAM_SLV_RAM_BASE };
static IcssgUtils_HwqIpMmap *IcssgUtils_hwq_a[ICSSG_INSTANCE_NUM] =
{
    (IcssgUtils_HwqIpMmap *)(CSL_PRU_ICSSG0_DRAM0_SLV_RAM_BASE + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_REGS_BASE + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_QUEUE0),
    (IcssgUtils_HwqIpMmap *)(CSL_PRU_ICSSG1_DRAM0_SLV_RAM_BASE + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_REGS_BASE + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_QUEUE0)
};
#endif

#if (ENET_CFG_TRACE_LEVEL >= ENET_CFG_TRACE_LEVEL_DEBUG)
/*! \brief R30 command names. */
static const char *gIcssg_r30CmdNames[] =
{
    [ICSSG_UTILS_R30_CMD_DISABLE] = "DISABLE",
    [ICSSG_UTILS_R30_CMD_BLOCK] = "BLOCK",
    [ICSSG_UTILS_R30_CMD_FORWARD] = "FORWARD",
    [ICSSG_UTILS_R30_CMD_FORWARD_WO_LEARNING] = "FORWARD_WO_LEARNING",
    [ICSSG_UTILS_R30_CMD_ACCEPT_ALL] = "ACCEPT_ALL",
    [ICSSG_UTILS_R30_CMD_ACCEPT_TAGGED] = "ACCEPT_TAGGED",
    [ICSSG_UTILS_R30_CMD_ACCEPT_UNTAGGED_N_PRIO] = "UNTAGGED_N_PRIO",
    [ICSSG_UTILS_R30_CMD_TAS_TRIGGER] = "TAS_TRIGGER",
    [ICSSG_UTILS_R30_CMD_TAS_ENABLE] = "TAS_ENABLE",
    [ICSSG_UTILS_R30_CMD_TAS_RESET] = "TAS_RESET",
    [ICSSG_UTILS_R30_CMD_TAS_DISABLE] = "TAS_DISABLE",
    [ICSSG_UTILS_R30_CMD_UC_FLOODING_ENABLE] = "UC_FLOODING_ENABLE",
    [ICSSG_UTILS_R30_CMD_UC_FLOODING_DISABLE] = "UC_FLOODING_DISABLE",
    [ICSSG_UTILS_R30_CMD_MC_FLOODING_ENABLE] = "MC_FLOODING_ENABLE",
    [ICSSG_UTILS_R30_CMD_MC_FLOODING_DISABLE] = "MC_FLOODING_DISABLE",
    [ICSSG_UTILS_R30_CMD_PREMPT_TX_ENABLE] = "PREMPT_TX_ENABLE",
    [ICSSG_UTILS_R30_CMD_PREMPT_TX_DISABLE] = "PREMPT_TX_DISABLE",
    [ICSSG_UTILS_R30_CMD_VLAN_AWARE_ENABLE] = "VLAN_AWARE_ENABLE",
    [ICSSG_UTILS_R30_CMD_VLAN_AWARE_DISABLE] = "VLAN_AWARE_DISABLE",
    [ICSSG_UTILS_R30_CMD_DSCP_ENABLE] = "IPV4_DSCP_ENABLE",
    [ICSSG_UTILS_R30_CMD_DSCP_DISABLE] = "IPV4_DSCP_DISABLE",
};
#endif

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

uint64_t Icssg_rd64(Icssg_Handle hIcssg,
                    uintptr_t addr)
{
    uint64_t val;

    val = CSL_REG64_RD(addr);
    ENETTRACE_VERBOSE("%s: RD: 0x%08x = 0x%016llx\n", hIcssg->enetPer.name, addr, val);

    return val;
}

uint32_t Icssg_rd32(Icssg_Handle hIcssg,
                    uintptr_t addr)
{
    uint32_t val;

    val = CSL_REG32_RD(addr);
    ENETTRACE_VERBOSE("%s: RD: 0x%08x = 0x%08x\n", hIcssg->enetPer.name, addr, val);

    return val;
}

uint16_t Icssg_rd16(Icssg_Handle hIcssg,
                    uintptr_t addr)
{
    uint16_t val;

    val = CSL_REG16_RD(addr);
    ENETTRACE_VERBOSE("%s: RD: 0x%08x = 0x%04x\n", hIcssg->enetPer.name, addr, val);

    return val;
}

uint8_t Icssg_rd8(Icssg_Handle hIcssg,
                  uintptr_t addr)
{
    uint8_t val;

    val = CSL_REG8_RD(addr);
    ENETTRACE_VERBOSE("%s: RD: 0x%08x = 0x%02x\n", hIcssg->enetPer.name, addr, val);

    return val;
}

void Icssg_wr64(Icssg_Handle hIcssg,
                uintptr_t addr,
                uint64_t val)
{
    ENETTRACE_VERBOSE("%s: WR: 0x%08x = 0x%016llx\n", hIcssg->enetPer.name, addr, val);
    CSL_REG64_WR(addr, val);
}

void Icssg_wr32(Icssg_Handle hIcssg,
                uintptr_t addr,
                uint32_t val)
{
    ENETTRACE_VERBOSE("%s: WR: 0x%08x = 0x%08x\n", hIcssg->enetPer.name, addr, val);
    CSL_REG32_WR(addr, val);
}

void Icssg_wr16(Icssg_Handle hIcssg,
                uintptr_t addr,
                uint16_t val)
{
    ENETTRACE_VERBOSE("%s: WR: 0x%08x = 0x%04x\n", hIcssg->enetPer.name, addr, val);
    CSL_REG16_WR(addr, val);
}

void Icssg_wr8(Icssg_Handle hIcssg,
               uintptr_t addr,
               uint8_t val)
{
    ENETTRACE_VERBOSE("%s: WR: 0x%08x = 0x%02x\n", hIcssg->enetPer.name, addr, val);
    CSL_REG8_WR(addr, val);
}

uint32_t IcssgUtils_getSliceNum(Icssg_Handle hIcssg,
                                Enet_MacPort macPort)
{
    EnetPer_Handle hPer = (EnetPer_Handle)hIcssg;
    uint32_t slice = 0U;

    if (hPer->enetType == ENET_ICSSG_DUALMAC)
    {
        switch (hPer->instId)
        {
            case 0:
            case 2:
                slice = 0U;
                break;

            case 1:
            case 3:
                slice = 1U;
                break;

            default:
                Enet_assert(false, "%s: invalid instance number %u\n",
                            ENET_PER_NAME(hIcssg), hPer->instId);
                break;
        }
    }
    else
    {
        Enet_assert(ENET_MACPORT_NORM(macPort) < 2U,
                    "%s: invalid MAC port %u\n",
                    ENET_PER_NAME(hIcssg), ENET_MACPORT_ID(macPort));
        slice = ENET_MACPORT_NORM(macPort) % 2U;
    }

    return slice;
}

uintptr_t Icssg_getDramAddr(Icssg_Handle hIcssg,
                            Enet_MacPort macPort)
{
    uintptr_t addr = (uintptr_t)hIcssg->enetPer.virtAddr;
    uint32_t slice = IcssgUtils_getSliceNum(hIcssg, macPort);

    if (ICSSG_IS_SLICE_0(slice))
    {
        addr += CSL_ICSS_G_DRAM0_SLV_RAM_REGS_BASE;
    }
    else
    {
        addr += CSL_ICSS_G_DRAM1_SLV_RAM_REGS_BASE;
    }

    return addr;
}

uintptr_t Icssg_getCfgAddr(Icssg_Handle hIcssg)
{
    return (uintptr_t)hIcssg->enetPer.virtAddr +
           CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_REGS_BASE;
}

uintptr_t Icssg_getRgmiiCfgAddr(Icssg_Handle hIcssg)
{
    return Icssg_getCfgAddr(hIcssg) +
           CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG;
}

uintptr_t Icssg_getSharedRamAddr(Icssg_Handle hIcssg)
{
    return (uintptr_t)hIcssg->enetPer.virtAddr +
           CSL_ICSS_G_RAM_SLV_RAM_REGS_BASE;
}

uintptr_t Icssg_getDfltVlanAddr(Icssg_Handle hIcssg,
                                Enet_MacPort macPort)
{
    uintptr_t addr = Icssg_getSharedRamAddr(hIcssg);
    uint32_t slice = IcssgUtils_getSliceNum(hIcssg, macPort);

    if (ICSSG_IS_SLICE_0(slice))
    {
        addr += EMAC_ICSSG_SWITCH_PORT1_DEFAULT_VLAN_OFFSET;
    }
    else
    {
        addr += EMAC_ICSSG_SWITCH_PORT2_DEFAULT_VLAN_OFFSET;
    }

    return addr;
}

uintptr_t Icssg_getVlanTableAddr(Icssg_Handle hIcssg)
{
    return Icssg_getSharedRamAddr(hIcssg) +
           EMAC_ICSSG_SWITCH_DEFAULT_VLAN_TABLE_OFFSET;
}

void IcssgUtils_WriteMem(uintptr_t addr,
                         const void *ptr,
                         uint32_t element_count)
{
    uint32_t i;
    volatile uint32_t *dst = (volatile uint32_t *)addr;
    uint32_t *src = (uint32_t *)ptr;

    for (i = 0; i < element_count; i++)
    {
        *dst++ = *src++;
    }
}

void IcssgUtils_R30CmdInit(Icssg_Handle hIcssg,
                           Enet_MacPort macPort)
{
    uint32_t inst = hIcssg->pruss->instance;
    uint32_t slice = IcssgUtils_getSliceNum(hIcssg, macPort);
    int32_t i;

    for (i = 0U; i < ICSSG_UTILS_R30_CMD_LEN; i++)
    {
        IcssgUtils_R30DmemAddr[inst][slice]->cmd[i] = ICSSG_UTILS_CMD_NONE;
    }
}

bool IcssgUtils_isR30CmdDone(Icssg_Handle hIcssg,
                             Enet_MacPort macPort)
{
    uint32_t inst = hIcssg->pruss->instance;
    uint32_t slice = IcssgUtils_getSliceNum(hIcssg, macPort);
    uint32_t i;
    bool done = true;

    for (i = 0U; i < ICSSG_UTILS_R30_CMD_LEN; i++)
    {
        if (IcssgUtils_R30DmemAddr[inst][slice]->cmd[i] == ICSSG_UTILS_CMD_NONE)
        {
            continue;
        }

        done = false;
        break;
    }

    return done;
}

int32_t Icssg_R30SendAsyncIoctl(Icssg_Handle hIcssg,
                                Enet_MacPort macPort,
                                IcssgUtils_ioctlR30Cmd cmd,
                                uint8_t *seqNum,
                                uint32_t *asyncIoctlType)
{
    uint32_t inst = hIcssg->pruss->instance;
    uint32_t slice = IcssgUtils_getSliceNum(hIcssg, macPort);
    uint32_t i;

    ++(*seqNum);
    *asyncIoctlType = ICSSG_UTILS_IOCTL_TYPE_R30_OVER_DMEM;

    ENETTRACE_DBG("%s: MAC port %u: Send R30 '%s' cmd seq %u\n",
                  hIcssg->enetPer.name, ENET_MACPORT_ID(macPort), gIcssg_r30CmdNames[cmd], *seqNum);

    for (i = 0U; i < ICSSG_UTILS_R30_CMD_LEN; i++)
    {
        IcssgUtils_R30DmemAddr[inst][slice]->cmd[i] = IcssgUtils_R30Bitmask[cmd].cmd[i];
    }

    return ENET_SINPROGRESS;
}

int32_t Icssg_R30SendSyncIoctl(Icssg_Handle hIcssg,
                               Enet_MacPort macPort,
                               IcssgUtils_ioctlR30Cmd cmd)
{
    uint32_t retry = ICSSG_ASYNC_RETRY_CNT;
    int32_t status;
    bool cmdDone;

    status = Icssg_R30SendAsyncIoctl(hIcssg,
                                     macPort,
                                     cmd,
                                     &hIcssg->asyncIoctlSeqNum,
                                     &hIcssg->asyncIoctlType);
    if (status == ENET_SINPROGRESS)
    {
        cmdDone = IcssgUtils_isR30CmdDone(hIcssg, macPort);
        while (!cmdDone && (retry != 0U))
        {
            EnetUtils_delayTicks(100);
            cmdDone = IcssgUtils_isR30CmdDone(hIcssg, macPort);
            if (!cmdDone)
            {
                retry--;
            }
        }

        status = cmdDone ? ENET_SOK : ENET_ETIMEOUT;
    }

    return status;
}

/*!
 * \brief utility to return lower 16 bits of 32 bit crc on array of 8 bytes
 */
uint16_t IcssgUtils_CRC64(uint8_t input[8])
{
    int32_t i;
    uint16_t res = 0x0000;
    int32_t j;
    int32_t k = 0;
    uint8_t d[64];
    uint8_t utilsNextC[64];

    for (i = 0; i < 8; i++)
    {
        for (j = 0; j < 8; j++)
        {
            d[k] = (input[i] & (1 << j)) ?  1 : 0;
            k += 1;
        }
    }

    utilsNextC[0] = d[63] ^ d[57] ^ d[54] ^ d[53] ^ d[51] ^ d[47] ^ d[39] ^ d[38] ^ d[37] ^
                    d[35] ^ d[34] ^ d[33] ^ d[32] ^ d[31] ^ d[29] ^ d[26] ^ d[19] ^ d[18] ^
                    d[16] ^ d[15] ^ d[13] ^ d[10] ^ d[9]  ^ d[8]  ^ d[5]  ^ d[3]  ^ d[2]  ^
                    d[0];
    utilsNextC[1] = d[63] ^ d[62] ^ d[57] ^ d[56] ^ d[54] ^ d[52] ^ d[51] ^ d[50] ^ d[47] ^
                    d[46] ^ d[39] ^ d[36] ^ d[35] ^ d[30] ^ d[29] ^ d[28] ^ d[26] ^ d[25] ^
                    d[19] ^ d[17] ^ d[16] ^ d[14] ^ d[13] ^ d[12] ^ d[10] ^ d[7]  ^ d[5 ] ^
                    d[4]  ^ d[3]  ^ d[1]  ^ d[0];
    utilsNextC[2] = d[63] ^ d[62] ^ d[61] ^ d[57] ^ d[56] ^ d[55] ^ d[54] ^ d[50] ^ d[49] ^
                    d[47] ^ d[46] ^ d[45] ^ d[39] ^ d[37] ^ d[33] ^ d[32] ^ d[31] ^ d[28] ^
                    d[27] ^ d[26] ^ d[25] ^ d[24] ^ d[19] ^ d[12] ^ d[11] ^ d[10] ^ d[8]  ^
                    d[6]  ^ d[5]  ^ d[4];
    utilsNextC[3] = d[62] ^ d[61] ^ d[60] ^ d[56] ^ d[55] ^ d[54] ^ d[53] ^ d[49] ^ d[48] ^
                    d[46] ^ d[45] ^ d[44] ^ d[38] ^ d[36] ^ d[32] ^ d[31] ^ d[30] ^ d[27] ^
                    d[26] ^ d[25] ^ d[24] ^ d[23] ^ d[18] ^ d[11] ^ d[10] ^ d[9]  ^ d[7]  ^
                    d[5]  ^ d[4]  ^ d[3];
    utilsNextC[4] = d[63] ^ d[61] ^ d[60] ^ d[59] ^ d[57] ^ d[55] ^ d[52] ^ d[51] ^ d[48] ^
                    d[45] ^ d[44] ^ d[43] ^ d[39] ^ d[38] ^ d[34] ^ d[33] ^ d[32] ^ d[30] ^
                    d[25] ^ d[24] ^ d[23] ^ d[22] ^ d[19] ^ d[18] ^ d[17] ^ d[16] ^ d[15] ^
                    d[13] ^ d[6]  ^ d[5]  ^ d[4]  ^ d[0];
    utilsNextC[5] = d[62] ^ d[60] ^ d[59] ^ d[58] ^ d[57] ^ d[56] ^ d[53] ^ d[50] ^ d[44] ^
                    d[43] ^ d[42] ^ d[39] ^ d[35] ^ d[34] ^ d[26] ^ d[24] ^ d[23] ^ d[22] ^
                    d[21] ^ d[19] ^ d[17] ^ d[14] ^ d[13] ^ d[12] ^ d[10] ^ d[9]  ^ d[8]  ^
                    d[4]  ^ d[2]  ^ d[0];
    utilsNextC[6] = d[62] ^ d[61] ^ d[59] ^ d[58] ^ d[57] ^ d[56] ^ d[55] ^ d[52] ^ d[49] ^
                    d[43] ^ d[42] ^ d[41] ^ d[38] ^ d[34] ^ d[33] ^ d[25] ^ d[23] ^ d[22] ^
                    d[21] ^ d[20] ^ d[18] ^ d[16] ^ d[13] ^ d[12] ^ d[11] ^ d[9]  ^ d[8]  ^
                    d[7]  ^ d[3]  ^ d[1];
    utilsNextC[7] = d[63] ^ d[61] ^ d[60] ^ d[58] ^ d[56] ^ d[55] ^ d[53] ^ d[48] ^ d[47] ^
                    d[42] ^ d[41] ^ d[40] ^ d[39] ^ d[38] ^ d[35] ^ d[34] ^ d[31] ^ d[29] ^
                    d[26] ^ d[24] ^ d[22] ^ d[21] ^ d[20] ^ d[18] ^ d[17] ^ d[16] ^ d[13] ^
                    d[12] ^ d[11] ^ d[9]  ^ d[7]  ^ d[6]  ^ d[5]  ^ d[3];
    utilsNextC[8] = d[63] ^ d[62] ^ d[60] ^ d[59] ^ d[55] ^ d[53] ^ d[52] ^ d[51] ^ d[46] ^
                    d[41] ^ d[40] ^ d[35] ^ d[32] ^ d[31] ^ d[30] ^ d[29] ^ d[28] ^ d[26] ^
                    d[25] ^ d[23] ^ d[21] ^ d[20] ^ d[18] ^ d[17] ^ d[13] ^ d[12] ^ d[11] ^
                    d[9]  ^ d[6]  ^ d[4]  ^ d[3]  ^ d[0];
    utilsNextC[9] = d[62] ^ d[61] ^ d[59] ^ d[58] ^ d[54] ^ d[52] ^ d[51] ^ d[50] ^ d[45] ^
                    d[40] ^ d[39] ^ d[34] ^ d[31] ^ d[30] ^ d[29] ^ d[28] ^ d[27] ^ d[25] ^
                    d[24] ^ d[22] ^ d[20] ^ d[19] ^ d[17] ^ d[16] ^ d[12] ^ d[11] ^ d[10] ^
                    d[8]  ^ d[5]  ^ d[3]  ^ d[2];
    utilsNextC[10] = d[63] ^ d[61] ^ d[60] ^ d[58] ^ d[54] ^ d[50] ^ d[49] ^ d[47] ^ d[44] ^
                     d[37] ^ d[35] ^ d[34] ^ d[32] ^ d[31] ^ d[30] ^ d[28] ^ d[27] ^ d[24] ^
                     d[23] ^ d[21] ^ d[13] ^ d[11] ^ d[8]  ^ d[7]  ^ d[5]  ^ d[4]  ^ d[3]  ^
                     d[1]  ^ d[0];
    utilsNextC[11] = d[63] ^ d[62] ^ d[60] ^ d[59] ^ d[54] ^ d[51] ^ d[49] ^ d[48] ^ d[47] ^
                     d[46] ^ d[43] ^ d[39] ^ d[38] ^ d[37] ^ d[36] ^ d[35] ^ d[32] ^ d[30] ^
                     d[27] ^ d[23] ^ d[22] ^ d[20] ^ d[19] ^ d[18] ^ d[16] ^ d[15] ^ d[13] ^
                     d[12] ^ d[9]  ^ d[8]  ^ d[7]  ^ d[6]  ^ d[5]  ^ d[4];
    utilsNextC[12] = d[63] ^ d[62] ^ d[61] ^ d[59] ^ d[58] ^ d[57] ^ d[54] ^ d[51] ^ d[50] ^
                     d[48] ^ d[46] ^ d[45] ^ d[42] ^ d[39] ^ d[36] ^ d[33] ^ d[32] ^ d[22] ^
                     d[21] ^ d[17] ^ d[16] ^ d[14] ^ d[13] ^ d[12] ^ d[11] ^ d[10] ^ d[9]  ^
                     d[7]  ^ d[6]  ^ d[4]  ^ d[2]  ^ d[0];
    utilsNextC[13] = d[61] ^ d[60] ^ d[58] ^ d[57] ^ d[56] ^ d[53] ^ d[50] ^ d[49] ^ d[47] ^
                     d[45] ^ d[44] ^ d[41] ^ d[38] ^ d[35] ^ d[32] ^ d[31] ^ d[21] ^ d[20] ^
                     d[16] ^ d[15] ^ d[13] ^ d[12] ^ d[11] ^ d[10] ^ d[9]  ^ d[8]  ^ d[6]  ^
                     d[5]  ^ d[3]  ^ d[1];
    utilsNextC[14] = d[61] ^ d[60] ^ d[59] ^ d[57] ^ d[56] ^ d[55] ^ d[52] ^ d[49] ^ d[48] ^
                     d[46] ^ d[44] ^ d[43] ^ d[40] ^ d[37] ^ d[34] ^ d[31] ^ d[30] ^ d[20] ^
                     d[19] ^ d[15] ^ d[14] ^ d[12] ^ d[11] ^ d[10] ^ d[9]  ^ d[8]  ^ d[7]  ^
                     d[5]  ^ d[4]  ^ d[2]  ^ d[0];
    utilsNextC[15] = d[60] ^ d[59] ^ d[58] ^ d[56] ^ d[55] ^ d[54] ^ d[51] ^ d[48] ^ d[47] ^
                     d[45] ^ d[43] ^ d[42] ^ d[39] ^ d[36] ^ d[33] ^ d[30] ^ d[29] ^ d[19] ^
                     d[18] ^ d[14] ^ d[13] ^ d[11] ^ d[10] ^ d[9]  ^ d[8]  ^ d[7]  ^ d[6]  ^
                     d[4]  ^ d[3]  ^ d[1];

    /* assign crc_result = utilsNextC[15:0]; */
    /* so we walk the lower 16 bytes and pack into 16 bits */
    for (i = 0; i < 16; i++)
    {
        res |= (utilsNextC[i] ? (1 << i) : 0);
    }

    return res;
}

uint16_t IcssgUtils_FdbHelper(uintptr_t vlanTable,
                              int16_t vlanId,
                              uint8_t mac[],
                              uint8_t *fid)
{
    uint16_t *table;
    uint8_t input64[8];  /*64 bit input */

    if (vlanId == -1)
    {
        *fid = 0U;
    }
    else
    {
        table = (uint16_t *)vlanTable;
        *fid = (table[vlanId] & 0xff00) >> 8;
    }

    /* juggle input be->le */
    input64[0] = mac[0];
    input64[1] = mac[1];
    input64[2] = mac[2];
    input64[3] = mac[3];
    input64[4] = mac[4];
    input64[5] = mac[5];

    input64[6] = *fid;
    input64[7] = 0x00;

    return (0x1ff & IcssgUtils_CRC64(input64));
}

int32_t IcssgUtils_sendFdbCmd(Icssg_Handle hIcssg,
                              Enet_MacPort macPort,
                              uint32_t subCmd,
                              const Icssg_FdbEntry *entry,
                              uint16_t broadSideSlot,
                              uint8_t fid)
{
    Icssg_IoctlCmd *cmd = &hIcssg->cmd;
    uint32_t slice = IcssgUtils_getSliceNum(hIcssg, macPort);
    int32_t status;

    memset(cmd, 0, sizeof(Icssg_IoctlCmd));

    cmd->header = ICSSG_UTILS_FW_MGMT_CMD_HEADER;
    cmd->type   = ICSSG_UTILS_FW_MGMT_FDB_CMD_TYPE;
    cmd->seqNum = ++(hIcssg->asyncIoctlSeqNum);
    cmd->param  = subCmd;

    if (ICSSG_IS_SLICE_1(slice))
    {
        cmd->param |= (1 << 4);
    }

    switch (subCmd)
    {
        case ICSSG_IOCTL_SUBCMD_FDB_ENTRY_ADD:
        case ICSSG_IOCTL_SUBCMD_FDB_ENTRY_REMOVE:
            memcpy((void *)(&cmd->spare[0]), (void *)(&entry->macAddr[0]), 4);
            memcpy((void *)(&cmd->spare[1]), (void *)(&entry->macAddr[4]), 2);
            cmd->spare[2] = broadSideSlot;
            cmd->spare[1] = cmd->spare[1] | (fid << 16);
            cmd->spare[1] = cmd->spare[1] | (entry->fdbEntry[slice] << 24);
            break;

        case ICSSG_IOCTL_SUBCMD_FDB_ENTRY_REMOVE_ALL:
            cmd->spare[0] = 0x0;
            break;

        case ICSSG_IOCTL_SUBCMD_FDB_ENTRY_REMOVE_ALL_AGEABLE:
            cmd->spare[0] = 0x000000008;
            break;

        default:
            break;
    }

    status = IcssgUtils_sendHwqMgmtMsg(hIcssg, macPort);

    return status;
}

static void IcssgUtils_pdInitAndPush(Icssg_Handle hIcssg,
                                     uint32_t pool_addr,
                                     uint32_t ofs,
                                     uint32_t w0,
                                     uint32_t bdSize,
                                     uint32_t bdNum)
{
    uintptr_t cfgRegs = Icssg_getCfgAddr(hIcssg);
    uintptr_t sharedRam = Icssg_getSharedRamAddr(hIcssg);
    uint32_t j;
    uint32_t *pool_ptr = (uint32_t *)(sharedRam + pool_addr);

    for (j = 0; j < bdNum; j++)
    {
        memset ((void *)pool_ptr, 0, bdSize * sizeof(pool_ptr));

        *pool_ptr = w0;
        Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_QUEUE0 + ofs,
                   (uint32_t)(uintptr_t)pool_ptr);
        pool_ptr += bdSize;
    }
}

static int32_t IcssgUtils_getHwqNum(uint32_t slice,
                                    Icssg_MgmtHwQ hwQ)
{
    uint32_t queue = 0U;

    switch (hwQ)
    {
        case ICSSG_MGMT_FREE_HWQA:
            queue = ICSSG_IS_SLICE_0(slice) ? ICSSG_UTILS_MGMT_FREE_HWQA_PORT0 :
                                              ICSSG_UTILS_MGMT_FREE_HWQA_PORT1;
            break;

        case ICSSG_MGMT_RX_HWQA:
            queue = ICSSG_IS_SLICE_0(slice) ? ICSSG_UTILS_MGMT_RX_HWQA_PORT0 :
                                              ICSSG_UTILS_MGMT_RX_HWQA_PORT1;
            break;

        case ICSSG_MGMT_TX_HWQA:
            queue = ICSSG_IS_SLICE_0(slice) ? ICSSG_UTILS_MGMT_TX_HWQA_PORT0 :
                                              ICSSG_UTILS_MGMT_TX_HWQA_PORT1;
            break;

        case ICSSG_TXTS_RX_HWQA:
            queue = ICSSG_IS_SLICE_0(slice) ? ICSSG_UTILS_TXTS_RX_HWQA_PORT0 :
                                              ICSSG_UTILS_TXTS_RX_HWQA_PORT1;
            break;

        case ICSSG_TXTS_FREE_HWQA:
            queue = ICSSG_IS_SLICE_0(slice) ? ICSSG_UTILS_TXTS_FREE_HWQA_PORT0 :
                                              ICSSG_UTILS_TXTS_FREE_HWQA_PORT1;
            break;

        default:
            Enet_assert(false, "Invalid hardware queue type %u\n", hwQ);
            break;
    }

    return queue;
}


void *IcssgUtils_hwqPop(Icssg_Handle hIcssg,
                        Enet_MacPort macPort,
                        Icssg_MgmtHwQ hwQ)
{
    uint32_t inst = hIcssg->pruss->instance;
    uint32_t slice = IcssgUtils_getSliceNum(hIcssg, macPort);
    uint32_t queueNum = IcssgUtils_getHwqNum(slice, hwQ);

    Enet_devAssert((queueNum < ICSSG_UTILS_HWQ_MAX),
                   "Invalid queue number %u\n", queueNum);

    /* IcssgUtils_hwq_a[icssg]->hwq[queueNum] will get you a 16 bit address
     * from the mmr of the descriptor, this is actually a pop */
    return (void *)((uintptr_t)(IcssgUtils_hwq_a[inst]->hwq[queueNum] |
                                IcssgUtils_hwq_smem_start[inst]));
}

void IcssgUtils_hwqPushForSlice(Icssg_Handle hIcssg,
                                uint32_t slice,
                                Icssg_MgmtHwQ hwQ,
                                void *p)
{
    uint32_t inst = hIcssg->pruss->instance;
    uint32_t queueNum = IcssgUtils_getHwqNum(slice, hwQ);

    Enet_devAssert((queueNum < ICSSG_UTILS_HWQ_MAX),
                   "Invalid queue number %u\n", queueNum);

    IcssgUtils_hwq_a[inst]->hwq[queueNum] = (uint32_t)(uintptr_t)p;
}

void IcssgUtils_hwqPush(Icssg_Handle hIcssg,
                        Enet_MacPort macPort,
                        Icssg_MgmtHwQ hwQ,
                        void *p)
{
    uint32_t inst = hIcssg->pruss->instance;
    uint32_t slice = IcssgUtils_getSliceNum(hIcssg, macPort);
    uint32_t queueNum = IcssgUtils_getHwqNum(slice, hwQ);

    Enet_devAssert((queueNum < ICSSG_UTILS_HWQ_MAX),
                   "Invalid queue number %u\n", queueNum);

    IcssgUtils_hwq_a[inst]->hwq[queueNum] = (uint32_t)(uintptr_t)p;
}

void *IcssgUtils_hwqPeek(Icssg_Handle hIcssg,
                         Enet_MacPort macPort,
                         Icssg_MgmtHwQ hwQ)
{
    uint32_t inst = hIcssg->pruss->instance;
    uint32_t slice = IcssgUtils_getSliceNum(hIcssg, macPort);
    uint32_t queueNum = IcssgUtils_getHwqNum(slice, hwQ);

    Enet_devAssert((queueNum < ICSSG_UTILS_HWQ_PEEK_MAX),
                   "Invalid peekable queue number %u\n", queueNum);

    return (void *)(uintptr_t)((IcssgUtils_hwq_a[inst]->hwq_peek[queueNum] |
                                IcssgUtils_hwq_smem_start[inst]));
}

int32_t IcssgUtils_hwqLevel(Icssg_Handle hIcssg,
                            Enet_MacPort macPort,
                            Icssg_MgmtHwQ hwQ)
{
    uint32_t inst = hIcssg->pruss->instance;
    uint32_t slice = IcssgUtils_getSliceNum(hIcssg, macPort);
    uint32_t queueNum = IcssgUtils_getHwqNum(slice, hwQ);

    Enet_devAssert((queueNum < ICSSG_UTILS_HWQ_MAX),
                   "Invalid queue number %u\n", queueNum);

    return IcssgUtils_hwq_a[inst]->hwq_len[queueNum];
}

void IcssgUtils_hwqReset(Icssg_Handle hIcssg,
                         uint32_t queueNum)
{
    uintptr_t cfgRegs = Icssg_getCfgAddr(hIcssg);

    Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_QUEUE_RESET, queueNum);
}

int32_t IcssgUtils_sendHwqMgmtMsg(Icssg_Handle hIcssg,
                                  Enet_MacPort macPort)
{
    Icssg_IoctlCmd *cmd = &hIcssg->cmd;
    IcssgUtils_MgmtPkt *pMgmtPkt;
    int32_t retVal = ENET_SINPROGRESS;

    pMgmtPkt = (IcssgUtils_MgmtPkt *)IcssgUtils_hwqPop(hIcssg, macPort, ICSSG_MGMT_FREE_HWQA);
    if (pMgmtPkt == NULL)
    {
        retVal = ENET_EALLOC;
    }
    else
    {
        IcssgUtils_WriteMem((uintptr_t)&(pMgmtPkt->payload), cmd, 4U);
        IcssgUtils_hwqPush(hIcssg, macPort, ICSSG_MGMT_TX_HWQA, pMgmtPkt);
        hIcssg->asyncIoctlType = ICSSG_UTILS_IOCTL_TYPE_HWQ;
    }

    return retVal;
}

int32_t IcssgUtils_checkPortMode(Icssg_Handle hIcssg,
                                 const EnetMacPort_Interface *mii)
{
    uintptr_t cfgRegs = Icssg_getCfgAddr(hIcssg);
    uint32_t mode;
    uint32_t mii0Mode;
    uint32_t mii1Mode;
    uint32_t val;
    int32_t status = ENET_SOK;

    val = Icssg_rd32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_ICSS_G_CFG);

    if (ENET_IS_BIT_SET(val, CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_ICSS_G_CFG_TX_PRU_EN_SHIFT))
    {
        mii0Mode = CSL_FEXT(val, ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_ICSS_G_CFG_MII0_MODE);
        mii1Mode = CSL_FEXT(val, ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_ICSS_G_CFG_MII1_MODE);

        if (EnetMacPort_isMii(mii))
        {
            mode = 0U;
        }
        else if (EnetMacPort_isRgmii(mii))
        {
            mode = 1U;
        }
        else
        {
            ENETTRACE_ERR("%s: MII mode not supported, must be MII or RGMII\n", ENET_PER_NAME(hIcssg));
            status = ENET_ENOTSUPPORTED;
        }

        if (status == ENET_SOK)
        {
            /* As per hardware restriction, both MAC ports must be configured to the
             * same MII mode.
             * First Dual-MAC port to run configures both MII modes, second Dual-MAC port
             * to run checks that requested MII mode matches existing configuration */
            if ((mii0Mode != mode) || (mii1Mode != mode))
            {
                ENETTRACE_ERR("%s: MII mode mismatch (expected %u, requested %u)\n",
                              ENET_PER_NAME(hIcssg), mii0Mode, mode);
                status = ENET_EINVALIDPARAMS;
            }
        }
    }

    return status;
}

void IcssgUtils_fwConfig(Icssg_Handle hIcssg,
                         Enet_MacPort macPort,
                         const Icssg_Cfg *cfg,
                         const Icssg_FwPoolMem *fwPoolMem,
                         uint32_t rxPktFlowStart)
{
    uintptr_t baseAddr = (uintptr_t)hIcssg->enetPer.virtAddr;
    uintptr_t dram = Icssg_getDramAddr(hIcssg, macPort);
    uintptr_t cfgRegs = Icssg_getCfgAddr(hIcssg);
    uint32_t startAddr;
    uint32_t endAddr;
    uint32_t regVal, hwQueueNum, i;
    uint32_t MaxNumNormalPDs = 64, MaxNumSpecialPDs = 16;
    uint32_t pdWord[5];
    uint32_t slice = IcssgUtils_getSliceNum(hIcssg, macPort);

    memset((void *)&pdWord, 0, sizeof(pdWord));

    // For reducing IEP latency. Enable OCP clock.
    Icssg_wr32(hIcssg, baseAddr + CSL_ICSSCFG_REGS_BASE + CSL_ICSSCFG_IEPCLK, 1U);

    //Delay after IEP Sync Config. Requires minimum 10 ICSS clock cycles before IEP register access
    ClockP_usleep(100 * 1000);

    // Core sync will make ICSSG access to MSMC optimal
    Icssg_wr32(hIcssg, baseAddr + CSL_ICSSCFG_REGS_BASE + CSL_ICSSCFG_CORE_SYNC_REG, 1U); /* Enable coresync */

    /* Enable IEP0 counter and set default increment as 4 */
    regVal = (0x1U << CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG_CNT_ENABLE_SHIFT)  |
             (0x4U << CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG_DEFAULT_INC_SHIFT) |
             (0x4U << CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG_CMP_INC_SHIFT);
    Icssg_wr32(hIcssg, baseAddr + CSL_ICSS_G_PR1_IEP0_SLV_REGS_BASE + CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG, regVal);

    /*Enable IEP1 counter and set default increment as 4 - Required for RX and TX time stamping*/
    regVal = (0x1U << CSL_ICSS_G_PR1_IEP1_SLV_GLOBAL_CFG_REG_CNT_ENABLE_SHIFT)  |
             (0x4U << CSL_ICSS_G_PR1_IEP1_SLV_GLOBAL_CFG_REG_DEFAULT_INC_SHIFT) |
             (0x4U << CSL_ICSS_G_PR1_IEP1_SLV_GLOBAL_CFG_REG_CMP_INC_SHIFT);
    Icssg_wr32(hIcssg, baseAddr + CSL_ICSS_G_PR1_IEP1_SLV_REGS_BASE + CSL_ICSS_G_PR1_IEP1_SLV_GLOBAL_CFG_REG, regVal);

    /* Set IEP0 CMP regiser to cfg->cycleTimeNs - 4 */
    Icssg_wr32(hIcssg, baseAddr + CSL_ICSS_G_PR1_IEP0_SLV_REGS_BASE + CSL_ICSS_G_PR1_IEP0_SLV_CMP0_REG0, (cfg->cycleTimeNs - 0x4U));
    Icssg_wr32(hIcssg, baseAddr + CSL_ICSS_G_PR1_IEP0_SLV_REGS_BASE + CSL_ICSS_G_PR1_IEP0_SLV_CMP0_REG1, (cfg->cycleTimeNs - 0x4U));
    hIcssg->cycleTimeNs = cfg->cycleTimeNs;
    hIcssg->clockTypeFw = cfg->clockTypeFw;

    /* Clear IEP0 COUNT register */
    Icssg_wr32(hIcssg, baseAddr + CSL_ICSS_G_PR1_IEP0_SLV_REGS_BASE + CSL_ICSS_G_PR1_IEP0_SLV_COUNT_REG0, 0x0U);
    Icssg_wr32(hIcssg, baseAddr + CSL_ICSS_G_PR1_IEP0_SLV_REGS_BASE + CSL_ICSS_G_PR1_IEP0_SLV_COUNT_REG1, 0x0U);

    /* Enable CMP0, CMP5, CMP8 and CMP9, shadow mode */
    regVal = Icssg_rd32(hIcssg, baseAddr + CSL_ICSS_G_PR1_IEP0_SLV_REGS_BASE + CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG);
    regVal |= 0x20673;
    Icssg_wr32(hIcssg, baseAddr + CSL_ICSS_G_PR1_IEP0_SLV_REGS_BASE + CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG, regVal);

    Icssg_wr32(hIcssg, baseAddr + CSL_ICSSCFG_REGS_BASE + CSL_ICSSCFG_GPCFG0, 0x08000003U); /* GPCFG0 mux sel MII_RT */
    Icssg_wr32(hIcssg, baseAddr + CSL_ICSSCFG_REGS_BASE + CSL_ICSSCFG_GPCFG1, 0x08000003U); /* GPCFG1 mux sel MII_RT */

    // Need to be slice specific
    if (ICSSG_IS_SLICE_0(slice))
    {
        // Program constant table pointer for C28 on all 3 PRU cores
        Icssg_wr32(hIcssg, baseAddr + CSL_ICSS_G_PR1_PDSP0_IRAM_REGS_BASE + CSL_ICSS_G_PR1_PDSP0_IRAM_CONSTANT_TABLE_PROG_PTR_0, 0x100U);
        Icssg_wr32(hIcssg, baseAddr + CSL_ICSS_G_PR1_RTU0_PR1_RTU0_IRAM_REGS_BASE + CSL_ICSS_G_PR1_PDSP0_IRAM_CONSTANT_TABLE_PROG_PTR_0, 0x100U);
        Icssg_wr32(hIcssg, baseAddr + CSL_ICSS_G_PR1_PDSP_TX0_IRAM_REGS_BASE + CSL_ICSS_G_PR1_PDSP0_IRAM_CONSTANT_TABLE_PROG_PTR_0, 0x100U);

        for (hwQueueNum = 0; hwQueueNum < ICSSG_FW_CFG_MAX_HWQ; hwQueueNum++)
        {
            IcssgUtils_hwqReset(hIcssg, hwQueueNum);
        }

        IcssgUtils_hwqReset(hIcssg, ICSSG_FW_CFG_RECYCLE_Q_SLICE0);
        IcssgUtils_hwqReset(hIcssg, ICSSG_FW_CFG_PORT_HF_Q_SLICE0);
        IcssgUtils_hwqReset(hIcssg, ICSSG_FW_CFG_PORT_LF_Q_SLICE0);
        IcssgUtils_hwqReset(hIcssg, ICSSG_FW_CFG_HOST_HF_Q_SLICE0);
        IcssgUtils_hwqReset(hIcssg, ICSSG_FW_CFG_HOST_LF_Q_SLICE0);
        IcssgUtils_hwqReset(hIcssg, ICSSG_FW_CFG_HOST_SF_Q_SLICE0);

        IcssgUtils_pdInitAndPush(hIcssg, PORT_DESC0_LO, ICSSG_FW_CFG_PORT_LF_Q_SLICE0_OFFSET, 0x000000U, 2, MaxNumNormalPDs);
        IcssgUtils_pdInitAndPush(hIcssg, PORT_DESC0_HI, ICSSG_FW_CFG_PORT_HF_Q_SLICE0_OFFSET, 0x200000U, 2, MaxNumNormalPDs);
        IcssgUtils_pdInitAndPush(hIcssg, HOST_DESC0_LO, ICSSG_FW_CFG_HOST_LF_Q_SLICE0_OFFSET, 0x000000U, 2, MaxNumNormalPDs);
        IcssgUtils_pdInitAndPush(hIcssg, HOST_DESC0_HI, ICSSG_FW_CFG_HOST_HF_Q_SLICE0_OFFSET, 0x200000U, 2, MaxNumNormalPDs);
        IcssgUtils_pdInitAndPush(hIcssg, HOST_SPPD0,    ICSSG_FW_CFG_HOST_SF_Q_SLICE0_OFFSET, 0x400000U, 5, MaxNumSpecialPDs);

        /* Scratch buffer for error and large packets */
        Icssg_wr32(hIcssg, dram + DEFAULT_MSMC_Q_OFFSET, (uint32_t)fwPoolMem->scratchBufferMem);

        /* Port buffer pool memory */
        IcssgUtils_BufPoolCfg.poolLen  = fwPoolMem->portBufferPoolSize;
        IcssgUtils_BufPoolCfg.poolBase = (uint32_t)fwPoolMem->portBufferPoolMem;
        for (i = 0U; i < fwPoolMem->portBufferPoolNum; i++)
        {
            IcssgUtils_WriteMem(dram + BUFFER_POOL_0_ADDR_OFFSET + (i * 8),
                                &IcssgUtils_BufPoolCfg,
                                ICSSG_FW_CFG_NORMAL_PD_SIZE_BYTE_COUNT);

            IcssgUtils_BufPoolCfg.poolBase += IcssgUtils_BufPoolCfg.poolLen;
        }

        /* Host buffer pool memory */
        IcssgUtils_BufPoolCfg.poolLen  = fwPoolMem->hostBufferPoolSize;
        IcssgUtils_BufPoolCfg.poolBase = (uint32_t)fwPoolMem->hostBufferPoolMem;
        for (i = ICSSG_SWITCH_PORT_BUFFER_POOL_NUM; i < (fwPoolMem->hostBufferPoolNum + ICSSG_SWITCH_PORT_BUFFER_POOL_NUM); i++)
        {
            IcssgUtils_WriteMem(dram + BUFFER_POOL_0_ADDR_OFFSET + (i * 8),
                                &IcssgUtils_BufPoolCfg,
                                ICSSG_FW_CFG_NORMAL_PD_SIZE_BYTE_COUNT);

            IcssgUtils_BufPoolCfg.poolBase += IcssgUtils_BufPoolCfg.poolLen;
        }

        /* Host egress queue memory */
        startAddr = (uint32_t)fwPoolMem->hostEgressQueueMem;
        endAddr   = startAddr + (fwPoolMem->hostEgressQueueSize * fwPoolMem->hostEgressQueueNum);
        Icssg_wr32(hIcssg, dram + HOST_RX_Q_EXP_CONTEXT_OFFSET,       startAddr);
        Icssg_wr32(hIcssg, dram + HOST_RX_Q_EXP_CONTEXT_OFFSET + 4U,  startAddr);
        Icssg_wr32(hIcssg, dram + HOST_RX_Q_EXP_CONTEXT_OFFSET + 8U,  startAddr);
        Icssg_wr32(hIcssg, dram + HOST_RX_Q_EXP_CONTEXT_OFFSET + 12U, endAddr);

        startAddr = (uint32_t)fwPoolMem->hostEgressPreQueueMem;
        endAddr   = startAddr + (fwPoolMem->hostEgressPreQueueSize * fwPoolMem->hostEgressQueueNum);
        Icssg_wr32(hIcssg, dram + HOST_RX_Q_PRE_CONTEXT_OFFSET,       startAddr);
        Icssg_wr32(hIcssg, dram + HOST_RX_Q_PRE_CONTEXT_OFFSET + 4U,  startAddr);
        Icssg_wr32(hIcssg, dram + HOST_RX_Q_PRE_CONTEXT_OFFSET + 8U,  startAddr);
        Icssg_wr32(hIcssg, dram + HOST_RX_Q_PRE_CONTEXT_OFFSET + 12U, endAddr);

        // Configure PSI flow info for firmware
        // SMEM
        // #define PSI_L_REGULAR_FLOW_ID_BASE_OFFSET
        // #define PSI_L_MGMT_FLOW_ID_OFFSET
        // 10 flows per slice
        Icssg_wr32(hIcssg, dram + PSI_L_REGULAR_FLOW_ID_BASE_OFFSET, rxPktFlowStart);
        Icssg_wr8(hIcssg, dram + SPL_PKT_DEFAULT_PRIORITY, 0);
        Icssg_wr8(hIcssg, dram + QUEUE_NUM_UNTAGGED, 0x0);

        /* Enable VLAN FDB in MMR */
        regVal = CSL_REG32_RD(cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FDB_GEN_CFG2);
        regVal |= (1U << CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FDB_GEN_CFG2_FDB_PRU0_EN_SHIFT) |
                  (1U << CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FDB_GEN_CFG2_FDB_HOST_EN_SHIFT);
        CSL_REG32_WR(cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FDB_GEN_CFG2, regVal);
    }
    else
    {
        // Program constant table pointer for C28 on all 3 PRU cores
        Icssg_wr32(hIcssg, baseAddr + CSL_ICSS_G_PR1_PDSP1_IRAM_REGS_BASE + CSL_ICSS_G_PR1_PDSP1_IRAM_CONSTANT_TABLE_PROG_PTR_0, 0x100U);
        Icssg_wr32(hIcssg, baseAddr + CSL_ICSS_G_PR1_RTU1_PR1_RTU1_IRAM_REGS_BASE + CSL_ICSS_G_PR1_PDSP1_IRAM_CONSTANT_TABLE_PROG_PTR_0, 0x100U);
        Icssg_wr32(hIcssg, baseAddr + CSL_ICSS_G_PR1_PDSP_TX1_IRAM_REGS_BASE + CSL_ICSS_G_PR1_PDSP1_IRAM_CONSTANT_TABLE_PROG_PTR_0, 0x100U);

        for (hwQueueNum = ICSSG_FW_CFG_MAX_HWQ; hwQueueNum < ICSSG_FW_CFG_MAX_HWQ * 2; hwQueueNum++)
        {
            IcssgUtils_hwqReset(hIcssg, hwQueueNum);
        }

        IcssgUtils_hwqReset(hIcssg, ICSSG_FW_CFG_RECYCLE_Q_SLICE1);
        IcssgUtils_hwqReset(hIcssg, ICSSG_FW_CFG_PORT_HF_Q_SLICE1);
        IcssgUtils_hwqReset(hIcssg, ICSSG_FW_CFG_PORT_LF_Q_SLICE1);
        IcssgUtils_hwqReset(hIcssg, ICSSG_FW_CFG_HOST_HF_Q_SLICE1);
        IcssgUtils_hwqReset(hIcssg, ICSSG_FW_CFG_HOST_LF_Q_SLICE1);
        IcssgUtils_hwqReset(hIcssg, ICSSG_FW_CFG_HOST_SF_Q_SLICE1);

        IcssgUtils_pdInitAndPush(hIcssg, PORT_DESC1_LO, ICSSG_FW_CFG_PORT_LF_Q_SLICE1_OFFSET, 0x800000U, 2, MaxNumNormalPDs);
        IcssgUtils_pdInitAndPush(hIcssg, PORT_DESC1_HI, ICSSG_FW_CFG_PORT_HF_Q_SLICE1_OFFSET, 0xA00000U, 2, MaxNumNormalPDs);
        IcssgUtils_pdInitAndPush(hIcssg, HOST_DESC1_LO, ICSSG_FW_CFG_HOST_LF_Q_SLICE1_OFFSET, 0x800000U, 2, MaxNumNormalPDs);
        IcssgUtils_pdInitAndPush(hIcssg, HOST_DESC1_HI, ICSSG_FW_CFG_HOST_HF_Q_SLICE1_OFFSET, 0xA00000U, 2, MaxNumNormalPDs);
        IcssgUtils_pdInitAndPush(hIcssg, HOST_SPPD1, ICSSG_FW_CFG_HOST_SF_Q_SLICE1_OFFSET,    0xC00000U, 5, MaxNumSpecialPDs);

        /* Port buffer pool memory */
        IcssgUtils_BufPoolCfg.poolLen  = fwPoolMem->portBufferPoolSize;
        IcssgUtils_BufPoolCfg.poolBase = (uint32_t)fwPoolMem->portBufferPoolMem;
        for (i = 0U; i < fwPoolMem->portBufferPoolNum; i++)
        {
            IcssgUtils_WriteMem(dram + BUFFER_POOL_0_ADDR_OFFSET + (i * 8),
                                &IcssgUtils_BufPoolCfg,
                                ICSSG_FW_CFG_NORMAL_PD_SIZE_BYTE_COUNT);

            IcssgUtils_BufPoolCfg.poolBase += IcssgUtils_BufPoolCfg.poolLen;
        }

        /* Host buffer pool memory */
        IcssgUtils_BufPoolCfg.poolLen  = fwPoolMem->hostBufferPoolSize;
        IcssgUtils_BufPoolCfg.poolBase = (uint32_t)fwPoolMem->hostBufferPoolMem;
        for (i = ICSSG_SWITCH_PORT_BUFFER_POOL_NUM; i < (fwPoolMem->hostBufferPoolNum + ICSSG_SWITCH_PORT_BUFFER_POOL_NUM); i++)
        {
            IcssgUtils_WriteMem(dram + BUFFER_POOL_0_ADDR_OFFSET + (i * 8),
                                &IcssgUtils_BufPoolCfg,
                                ICSSG_FW_CFG_NORMAL_PD_SIZE_BYTE_COUNT);

            IcssgUtils_BufPoolCfg.poolBase += IcssgUtils_BufPoolCfg.poolLen;
        }

        /* Host egress queue memory */
        startAddr = (uint32_t)fwPoolMem->hostEgressQueueMem;
        endAddr   = startAddr + (fwPoolMem->hostEgressQueueSize * fwPoolMem->hostEgressQueueNum);
        Icssg_wr32(hIcssg, dram + HOST_RX_Q_EXP_CONTEXT_OFFSET,       startAddr);
        Icssg_wr32(hIcssg, dram + HOST_RX_Q_EXP_CONTEXT_OFFSET + 4U,  startAddr);
        Icssg_wr32(hIcssg, dram + HOST_RX_Q_EXP_CONTEXT_OFFSET + 8U,  startAddr);
        Icssg_wr32(hIcssg, dram + HOST_RX_Q_EXP_CONTEXT_OFFSET + 12U, endAddr);

        startAddr = (uint32_t)fwPoolMem->hostEgressPreQueueMem;
        endAddr   = startAddr + (fwPoolMem->hostEgressPreQueueSize * fwPoolMem->hostEgressQueueNum);
        Icssg_wr32(hIcssg, dram + HOST_RX_Q_PRE_CONTEXT_OFFSET,       startAddr);
        Icssg_wr32(hIcssg, dram + HOST_RX_Q_PRE_CONTEXT_OFFSET + 4U,  startAddr);
        Icssg_wr32(hIcssg, dram + HOST_RX_Q_PRE_CONTEXT_OFFSET + 8U,  startAddr);
        Icssg_wr32(hIcssg, dram + HOST_RX_Q_PRE_CONTEXT_OFFSET + 12U, endAddr);

        // Configure PSI flow info for firmware
        // SMEM
        // #define PSI_L_REGULAR_FLOW_ID_BASE_OFFSET
        // #define PSI_L_MGMT_FLOW_ID_OFFSET
        // 10 flows per slice
        Icssg_wr32(hIcssg, dram + PSI_L_REGULAR_FLOW_ID_BASE_OFFSET, rxPktFlowStart);
        Icssg_wr8(hIcssg, dram + SPL_PKT_DEFAULT_PRIORITY, 0);
        Icssg_wr8(hIcssg, dram + QUEUE_NUM_UNTAGGED, 0x0U);

        /* Enable VLAN FDB in MMR */
        regVal = CSL_REG32_RD(cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FDB_GEN_CFG2);
        regVal |= (1U << CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FDB_GEN_CFG2_FDB_PRU1_EN_SHIFT) |
                  (1U << CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FDB_GEN_CFG2_FDB_HOST_EN_SHIFT);
        CSL_REG32_WR(cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FDB_GEN_CFG2, regVal);
    }

    // TODO - Below calls are port-related, so should be done conditionally.

    /* Random number for Backoff timer calculation */
    Icssg_wr32(hIcssg, dram + HD_RAND_SEED_OFFSET, (uint32_t)rand());

/*  Moved MII RXCFG configuration to firmware (done by RX_PRU)
    // Program RXCFG0/1 and TXCFG0/1
    Icssg_wr32(hIcssg, baseAddr + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_REGS_BASE + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_RXCFG0, 0x213U);
    Icssg_wr32(hIcssg, baseAddr + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_REGS_BASE + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_RXCFG1, 0x21BU);
*/

    if (EnetMacPort_isRgmii(&cfg->mii))
    {
        Icssg_wr32(hIcssg, baseAddr + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_REGS_BASE + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG0, 0x1803U);
        Icssg_wr32(hIcssg, baseAddr + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_REGS_BASE + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG1, 0x1903U);
    }
    else
    {
        Icssg_wr32(hIcssg, baseAddr + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_REGS_BASE + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG0, 0x1903U);
        Icssg_wr32(hIcssg, baseAddr + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_REGS_BASE + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG1, 0x1803U);
    }

    // Program TX_IPG0/IPG1
    /* Workaround: Due to hardware bug, the following sequence has to be followed:
     * - For port 0, write as is.
     * - For port 1, write value, and then do a dummy read-then-write operation on port 0 */
    Icssg_wr32(hIcssg, baseAddr + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_REGS_BASE + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_TX_IPG1, 0xBU);
    Icssg_wr32(hIcssg, baseAddr + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_REGS_BASE + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_TX_IPG0, 0xBU);
    // Reset Max preamble count
    Icssg_wr32(hIcssg, baseAddr + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_REGS_BASE + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_RX_PCNT0, 0x1U);
    Icssg_wr32(hIcssg, baseAddr + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_REGS_BASE + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_RX_PCNT1, 0x1U);
    //Init RGMII config for ICSSG : TXL2, TXPRU enable etc
    Icssg_wr32(hIcssg, baseAddr+CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_REGS_BASE+CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_RX_FRMS0, 0x7CF003F);
    Icssg_wr32(hIcssg, baseAddr+CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_REGS_BASE+CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_RX_FRMS1, 0x7CF003F);

    //Update Tx Max Frame Length to 2000 -> value to be programmed (2000+8)
    Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_TX_STAT_MAX_SIZE_PORT0, 0x7D8);
    Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_TX_STAT_MAX_SIZE_PORT1, 0x7D8);

    /*Configure shift enable for Scratch pad*/
    Icssg_wr32(hIcssg, baseAddr + CSL_ICSSCFG_REGS_BASE + CSL_ICSSCFG_SPP, 0xA);

    /* Init RGMII config for ICSSG: TXL2, TXPRU enable, etc */
    if (EnetMacPort_isRgmii(&cfg->mii))
    {
        Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_ICSS_G_CFG, 0x1082FU);
    }
    else
    {
        Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_ICSS_G_CFG, 0x10807U);
    }
}

void IcssgUtils_configSwtFw(Icssg_Handle hIcssg,
                            const Icssg_Cfg *cfg,
                            const Icssg_FwPoolMem *fwPoolMem0,
                            const Icssg_FwPoolMem *fwPoolMem1,
                            uint32_t rxPktFlowStart0,
                            uint32_t rxPktFlowStart1)
{
    uintptr_t baseAddr = (uintptr_t)hIcssg->enetPer.virtAddr;
    uintptr_t dram;
    uintptr_t cfgRegs = Icssg_getCfgAddr(hIcssg);
    uintptr_t miiRg;
    uint32_t startAddr;
    uint32_t endAddr;
    uint32_t regVal, hwQueueNum, i;
    uint32_t maxNumSpecialPDs = 16;

    /* ICSS G CFG */
    regVal = Icssg_rd32(hIcssg,
                        cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_ICSS_G_CFG);

    regVal &= ~(CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_ICSS_G_CFG_MII0_MODE_MASK |
                CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_ICSS_G_CFG_MII1_MODE_MASK);

    regVal |= ((0x1U << CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_ICSS_G_CFG_TX_L1_EN_SHIFT) |
               (0x1U << CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_ICSS_G_CFG_TX_L2_EN_SHIFT) |
               (0x1U << CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_ICSS_G_CFG_RX_L2_G_EN_SHIFT) |
               (0x1U << CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_ICSS_G_CFG_TX_PRU_EN_SHIFT));

    if (EnetMacPort_isRgmii(&cfg->mii))
    {
        regVal |= ((0x1U << CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_ICSS_G_CFG_MII0_MODE_SHIFT) |
                   (0x1U << CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_ICSS_G_CFG_MII1_MODE_SHIFT));
    }
    else
    {
        regVal |= ((0x0U << CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_ICSS_G_CFG_MII0_MODE_SHIFT) |
                   (0x0U << CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_ICSS_G_CFG_MII1_MODE_SHIFT));
    }

    Icssg_wr32(hIcssg,
               cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_ICSS_G_CFG,
               regVal);

    /* GPCFG0 mux sel MII_RT */
    Icssg_wr32(hIcssg, baseAddr + CSL_ICSSCFG_REGS_BASE + CSL_ICSSCFG_GPCFG0, 0x08000003U);

    /* GPCFG1 mux sel MII_RT */
    Icssg_wr32(hIcssg, baseAddr + CSL_ICSSCFG_REGS_BASE + CSL_ICSSCFG_GPCFG1, 0x08000003U);

    /* For reducing IEP latency. Enable OCP clock. */
    Icssg_wr32(hIcssg, baseAddr + CSL_ICSSCFG_REGS_BASE + CSL_ICSSCFG_IEPCLK, 1U);

    /* Delay after IEP Sync Config. Requires minimum 10 ICSS clock cycles before IEP register access */
    ClockP_usleep(100 * 1000);

    /* Core sync will make ICSSG access to MSMC optimal */
    Icssg_wr32(hIcssg, baseAddr + CSL_ICSSCFG_REGS_BASE + CSL_ICSSCFG_CORE_SYNC_REG, 1U);

    /* Enable IEP0 counter and set default increment as 4 */
    regVal = ((0x1U << CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG_CNT_ENABLE_SHIFT) |
              (0x4U << CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG_DEFAULT_INC_SHIFT) |
              (0x4U << CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG_CMP_INC_SHIFT));

    Icssg_wr32(hIcssg,
               baseAddr +
               CSL_ICSS_G_PR1_IEP0_SLV_REGS_BASE +
               CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG, regVal);

    /* Enable IEP1 counter and set default increment as 4 - Required for RX and TX time stamping */
    regVal = (0x1U << CSL_ICSS_G_PR1_IEP1_SLV_GLOBAL_CFG_REG_CNT_ENABLE_SHIFT)  |
             (0x4U << CSL_ICSS_G_PR1_IEP1_SLV_GLOBAL_CFG_REG_DEFAULT_INC_SHIFT) |
             (0x4U << CSL_ICSS_G_PR1_IEP1_SLV_GLOBAL_CFG_REG_CMP_INC_SHIFT);

    Icssg_wr32(hIcssg,
               baseAddr +
               CSL_ICSS_G_PR1_IEP1_SLV_REGS_BASE +
               CSL_ICSS_G_PR1_IEP1_SLV_GLOBAL_CFG_REG,
               regVal);

    /* Set IEP0 CMP regiser to 1 msec */
    Icssg_wr32(hIcssg, baseAddr + CSL_ICSS_G_PR1_IEP0_SLV_REGS_BASE + CSL_ICSS_G_PR1_IEP0_SLV_CMP0_REG0, 0x000F423C);
    Icssg_wr32(hIcssg, baseAddr + CSL_ICSS_G_PR1_IEP0_SLV_REGS_BASE + CSL_ICSS_G_PR1_IEP0_SLV_CMP0_REG1, 0x000F423C);
    hIcssg->cycleTimeNs = cfg->cycleTimeNs;
    hIcssg->clockTypeFw = cfg->clockTypeFw;

    /* Clear IEP0 COUNT register */
    Icssg_wr32(hIcssg, baseAddr + CSL_ICSS_G_PR1_IEP0_SLV_REGS_BASE + CSL_ICSS_G_PR1_IEP0_SLV_COUNT_REG0, 0x0U);
    Icssg_wr32(hIcssg, baseAddr + CSL_ICSS_G_PR1_IEP0_SLV_REGS_BASE + CSL_ICSS_G_PR1_IEP0_SLV_COUNT_REG1, 0x0U);

    /* Enable CMP0, CMP5, CMP8 and CMP9, shadow mode */
    regVal = Icssg_rd32(hIcssg, baseAddr + CSL_ICSS_G_PR1_IEP0_SLV_REGS_BASE + CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG);
    regVal |= 0x20673;
    Icssg_wr32(hIcssg, baseAddr + CSL_ICSS_G_PR1_IEP0_SLV_REGS_BASE + CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG, regVal);

    /* CMP5 and CMP6 used for learning */
    regVal = Icssg_rd32(hIcssg,
                        baseAddr +
                        CSL_ICSS_G_PR1_IEP1_SLV_REGS_BASE +
                        CSL_ICSS_G_PR1_IEP1_SLV_CMP_CFG_REG);
    regVal |= ENET_BIT(6) | ENET_BIT(7);
    Icssg_wr32(hIcssg,
               baseAddr +
               CSL_ICSS_G_PR1_IEP1_SLV_REGS_BASE +
               CSL_ICSS_G_PR1_IEP1_SLV_CMP_CFG_REG, regVal);

    /* Program RXCFG0/1 and TXCFG0/1 */
    Icssg_wr32(hIcssg,
               baseAddr +
               CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_REGS_BASE +
               CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_RXCFG0,
               0x211);
    Icssg_wr32(hIcssg,
               baseAddr +
               CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_REGS_BASE +
               CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_RXCFG1,
               0x219);

    if (EnetMacPort_isRgmii(&cfg->mii))
    {
        Icssg_wr32(hIcssg,
                   baseAddr +
                   CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_REGS_BASE +
                   CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG0,
                   0x00000103 | CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG0_TX_IPG_WIRE_CLK_EN0_MASK);
        Icssg_wr32(hIcssg,
                   baseAddr +
                   CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_REGS_BASE +
                   CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG1,
                   0x00000003 | CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG1_TX_IPG_WIRE_CLK_EN1_MASK);
    }
    else
    {
        Icssg_wr32(hIcssg,
                   baseAddr +
                   CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_REGS_BASE +
                   CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG0,
                   0x00000003 | CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG0_TX_IPG_WIRE_CLK_EN0_MASK);
        Icssg_wr32(hIcssg,
                   baseAddr +
                   CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_REGS_BASE +
                   CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG1,
                   0x00000103 | CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG1_TX_IPG_WIRE_CLK_EN1_MASK);
    }

    /* Note that it is important to update TX_IPG1 before TX_IPG0 as in WIRE_CLK mode TX_IPG0 write
     * is required to load the IPG value to hardware */
    Icssg_wr32(hIcssg,
               baseAddr +
               CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_REGS_BASE +
               CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_TX_IPG1,
               0xBU); /* Wire CLK @ 125 MHz : 8ns */
    Icssg_wr32(hIcssg,
               baseAddr +
               CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_REGS_BASE +
               CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_TX_IPG0,
               0xBU);  /* Wire CLK @ 125 MHz : 8ns */


    /* Reset Max preamble count */
    Icssg_wr32(hIcssg,
               baseAddr +
               CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_REGS_BASE +
               CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_RX_PCNT0,
               0x1U);
    Icssg_wr32(hIcssg,
               baseAddr +
               CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_REGS_BASE +
               CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_RX_PCNT1,
               0x1U);

    /* Update Max Frame Length to 2000 */
    Icssg_wr32(hIcssg, baseAddr+CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_REGS_BASE+CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_RX_FRMS0, 0x7CF003F);
    Icssg_wr32(hIcssg, baseAddr+CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_REGS_BASE+CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_RX_FRMS1, 0x7CF003F);

    /* Update Tx Max Frame Length to 2000 -> value to be programmed (2000+8) */
    Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_TX_STAT_MAX_SIZE_PORT0, 0x7D8);
    Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_TX_STAT_MAX_SIZE_PORT1, 0x7D8);

    if (EnetMacPort_isRgmii(&cfg->mii))
    {
        /* ICSS_G_CFG make it RGMII */
        regVal = Icssg_rd32(hIcssg,
                            baseAddr +
                            CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_REGS_BASE + 0x1000);
        regVal |= 0x2F;
        Icssg_wr32(hIcssg,
                   baseAddr +
                   CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_REGS_BASE + 0x1000,
                   regVal);
    }

    /* Configure shift enable for Scratch pad */
    Icssg_wr32(hIcssg, baseAddr + CSL_ICSSCFG_REGS_BASE + CSL_ICSSCFG_SPP, 0xA);

    /* Slice 0 settings */
    dram = Icssg_getDramAddr(hIcssg, ENET_MAC_PORT_1);
    Icssg_wr8(hIcssg, dram + SPL_PKT_DEFAULT_PRIORITY, 0);
    Icssg_wr8(hIcssg, dram + QUEUE_NUM_UNTAGGED, 0x0U);

    /* Configure PSI flow info for firmware */
    Icssg_wr32(hIcssg, dram + PSI_L_REGULAR_FLOW_ID_BASE_OFFSET, rxPktFlowStart0);

    /* Scratch buffer for error and large packets */
    Icssg_wr32(hIcssg, dram + DEFAULT_MSMC_Q_OFFSET, (uint32_t)fwPoolMem0->scratchBufferMem);

    /* Random number for Backoff timer calculation */
    Icssg_wr32(hIcssg, dram + HD_RAND_SEED_OFFSET, (uint32_t)rand());

    for (hwQueueNum = 0; hwQueueNum < ICSSG_FW_CFG_MAX_HWQ; hwQueueNum++)
    {
        IcssgUtils_hwqReset(hIcssg, hwQueueNum);
    }

    IcssgUtils_hwqReset(hIcssg, ICSSG_FW_CFG_RECYCLE_Q_SLICE0);
    IcssgUtils_hwqReset(hIcssg, ICSSG_FW_CFG_PORT_HF_Q_SLICE0);
    IcssgUtils_hwqReset(hIcssg, ICSSG_FW_CFG_PORT_LF_Q_SLICE0);
    IcssgUtils_hwqReset(hIcssg, ICSSG_FW_CFG_HOST_HF_Q_SLICE0);
    IcssgUtils_hwqReset(hIcssg, ICSSG_FW_CFG_HOST_LF_Q_SLICE0);
    IcssgUtils_hwqReset(hIcssg, ICSSG_FW_CFG_HOST_SF_Q_SLICE0);

    IcssgUtils_pdInitAndPush(hIcssg, PORT_DESC0_LO, ICSSG_FW_CFG_PORT_LF_Q_SLICE0_OFFSET, 0x000000U, 2, NRT_PORT_DESC_SMEM_SIZE / ICSSG_SWITCH_NORMAL_PD_SIZE);
    IcssgUtils_pdInitAndPush(hIcssg, PORT_DESC0_HI, ICSSG_FW_CFG_PORT_HF_Q_SLICE0_OFFSET, 0x200000U, 2, NRT_PORT_DESC_SMEM_SIZE / ICSSG_SWITCH_NORMAL_PD_SIZE);
    IcssgUtils_pdInitAndPush(hIcssg, HOST_DESC0_LO, ICSSG_FW_CFG_HOST_LF_Q_SLICE0_OFFSET, 0x000000U, 2, NRT_HOST_DESC_SMEM_SIZE / ICSSG_SWITCH_NORMAL_PD_SIZE);
    IcssgUtils_pdInitAndPush(hIcssg, HOST_DESC0_HI, ICSSG_FW_CFG_HOST_HF_Q_SLICE0_OFFSET, 0x200000U, 2, NRT_HOST_DESC_SMEM_SIZE / ICSSG_SWITCH_NORMAL_PD_SIZE);
    IcssgUtils_pdInitAndPush(hIcssg, HOST_SPPD0,    ICSSG_FW_CFG_HOST_SF_Q_SLICE0_OFFSET, 0x400000U, 5, maxNumSpecialPDs);

    /* Port buffer pool memory */
    IcssgUtils_BufPoolCfg.poolLen  = fwPoolMem0->portBufferPoolSize;
    IcssgUtils_BufPoolCfg.poolBase = (uint32_t)fwPoolMem0->portBufferPoolMem;
    for (i = 0U; i < fwPoolMem0->portBufferPoolNum; i++)
    {
        IcssgUtils_WriteMem(dram + BUFFER_POOL_0_ADDR_OFFSET + (i * 8),
                            &IcssgUtils_BufPoolCfg,
                            ICSSG_FW_CFG_NORMAL_PD_SIZE_BYTE_COUNT);

        IcssgUtils_BufPoolCfg.poolBase += IcssgUtils_BufPoolCfg.poolLen;
    }

    /* Host buffer pool memory */
    IcssgUtils_BufPoolCfg.poolLen  = fwPoolMem0->hostBufferPoolSize;
    IcssgUtils_BufPoolCfg.poolBase = (uint32_t)fwPoolMem0->hostBufferPoolMem;
    for (i = ICSSG_SWITCH_PORT_BUFFER_POOL_NUM; i < (fwPoolMem0->hostBufferPoolNum + ICSSG_SWITCH_PORT_BUFFER_POOL_NUM); i++)
    {
        IcssgUtils_WriteMem(dram + BUFFER_POOL_0_ADDR_OFFSET + (i * 8),
                            &IcssgUtils_BufPoolCfg,
                            ICSSG_FW_CFG_NORMAL_PD_SIZE_BYTE_COUNT);

        IcssgUtils_BufPoolCfg.poolBase += IcssgUtils_BufPoolCfg.poolLen;
    }

    /* Host egress queue memory */
    startAddr = (uint32_t)fwPoolMem0->hostEgressQueueMem;
    endAddr   = startAddr +
                (fwPoolMem0->hostEgressQueueSize * fwPoolMem0->hostEgressQueueNum);

    Icssg_wr32(hIcssg, dram + HOST_RX_Q_EXP_CONTEXT_OFFSET,       startAddr);
    Icssg_wr32(hIcssg, dram + HOST_RX_Q_EXP_CONTEXT_OFFSET + 4U,  startAddr);
    Icssg_wr32(hIcssg, dram + HOST_RX_Q_EXP_CONTEXT_OFFSET + 8U,  startAddr);
    Icssg_wr32(hIcssg, dram + HOST_RX_Q_EXP_CONTEXT_OFFSET + 12U, endAddr);

    /* Host egress queue memory */
    startAddr = (uint32_t)fwPoolMem0->hostEgressPreQueueMem;
    endAddr   = startAddr +
                (fwPoolMem0->hostEgressPreQueueSize * fwPoolMem0->hostEgressQueueNum);

    Icssg_wr32(hIcssg, dram + HOST_RX_Q_PRE_CONTEXT_OFFSET,       startAddr);
    Icssg_wr32(hIcssg, dram + HOST_RX_Q_PRE_CONTEXT_OFFSET + 4U,  startAddr);
    Icssg_wr32(hIcssg, dram + HOST_RX_Q_PRE_CONTEXT_OFFSET + 8U,  startAddr);
    Icssg_wr32(hIcssg, dram + HOST_RX_Q_PRE_CONTEXT_OFFSET + 12U, endAddr);

    /* Program constant table pointer for C28 on all 3 PRU cores */
    Icssg_wr32(hIcssg, baseAddr + CSL_ICSS_G_PR1_PDSP0_IRAM_REGS_BASE + CSL_ICSS_G_PR1_PDSP0_IRAM_CONSTANT_TABLE_PROG_PTR_0, 0x100U);
    Icssg_wr32(hIcssg, baseAddr + CSL_ICSS_G_PR1_RTU0_PR1_RTU0_IRAM_REGS_BASE + CSL_ICSS_G_PR1_PDSP0_IRAM_CONSTANT_TABLE_PROG_PTR_0, 0x100U);
    Icssg_wr32(hIcssg, baseAddr + CSL_ICSS_G_PR1_PDSP_TX0_IRAM_REGS_BASE + CSL_ICSS_G_PR1_PDSP0_IRAM_CONSTANT_TABLE_PROG_PTR_0, 0x100U);

    /* Ingress rate limiter in FW uses Class8 and Class9 for rate control,
     * make them hit always by default */
    Icssg_wr32(hIcssg,
               baseAddr +
               CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_REGS_BASE +
               CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS_GATES8_PRU0,
               0x70);
    Icssg_wr32(hIcssg,
               baseAddr +
               CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_REGS_BASE +
               CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS_GATES9_PRU0,
               0x70);

    miiRg = baseAddr + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_REGS_BASE;
    Icssg_wr32(hIcssg, miiRg + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT1_START_LEN_PRU0,  0x60000U);
    Icssg_wr32(hIcssg, miiRg + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT1_CFG_PRU0,        0x5555U);
    Icssg_wr32(hIcssg, miiRg + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT1_1_DA0_PRU0,      0x00C28001U); /* 01:80:C2:00 */
    Icssg_wr32(hIcssg, miiRg + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT1_1_DA1_PRU0,      0x0E00U);     /* 00:0X */
    Icssg_wr32(hIcssg, miiRg + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT1_1_DA_MASK0_PRU0, 0x0U);
    Icssg_wr32(hIcssg, miiRg + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT1_1_DA_MASK1_PRU0, 0x0F00U);

    /* Slice 1 settings */
    dram = Icssg_getDramAddr(hIcssg, ENET_MAC_PORT_2);
    Icssg_wr8(hIcssg, dram + SPL_PKT_DEFAULT_PRIORITY, 0);
    Icssg_wr8(hIcssg, dram + QUEUE_NUM_UNTAGGED, 0x0U);

    /* Configure PSI flow info for firmware */
    Icssg_wr32(hIcssg, dram + PSI_L_REGULAR_FLOW_ID_BASE_OFFSET, rxPktFlowStart1);

    /* Scratch buffer for error and large packets */
    Icssg_wr32(hIcssg, dram + DEFAULT_MSMC_Q_OFFSET, (uint32_t)fwPoolMem1->scratchBufferMem);

    /* Random number for Backoff timer calculation */
    Icssg_wr32(hIcssg, dram + HD_RAND_SEED_OFFSET, (uint32_t)rand());

    for (hwQueueNum = ICSSG_FW_CFG_MAX_HWQ; hwQueueNum < ICSSG_FW_CFG_MAX_HWQ * 2; hwQueueNum++)
    {
        IcssgUtils_hwqReset(hIcssg, hwQueueNum);
    }

    IcssgUtils_hwqReset(hIcssg, ICSSG_FW_CFG_RECYCLE_Q_SLICE1);
    IcssgUtils_hwqReset(hIcssg, ICSSG_FW_CFG_PORT_HF_Q_SLICE1);
    IcssgUtils_hwqReset(hIcssg, ICSSG_FW_CFG_PORT_LF_Q_SLICE1);
    IcssgUtils_hwqReset(hIcssg, ICSSG_FW_CFG_HOST_HF_Q_SLICE1);
    IcssgUtils_hwqReset(hIcssg, ICSSG_FW_CFG_HOST_LF_Q_SLICE1);
    IcssgUtils_hwqReset(hIcssg, ICSSG_FW_CFG_HOST_SF_Q_SLICE1);

    IcssgUtils_pdInitAndPush(hIcssg, PORT_DESC1_LO, ICSSG_FW_CFG_PORT_LF_Q_SLICE1_OFFSET, 0x800000U, 2, NRT_PORT_DESC_SMEM_SIZE / ICSSG_SWITCH_NORMAL_PD_SIZE);
    IcssgUtils_pdInitAndPush(hIcssg, PORT_DESC1_HI, ICSSG_FW_CFG_PORT_HF_Q_SLICE1_OFFSET, 0xA00000U, 2, NRT_PORT_DESC_SMEM_SIZE / ICSSG_SWITCH_NORMAL_PD_SIZE);
    IcssgUtils_pdInitAndPush(hIcssg, HOST_DESC1_LO, ICSSG_FW_CFG_HOST_LF_Q_SLICE1_OFFSET, 0x800000U, 2, NRT_HOST_DESC_SMEM_SIZE / ICSSG_SWITCH_NORMAL_PD_SIZE);
    IcssgUtils_pdInitAndPush(hIcssg, HOST_DESC1_HI, ICSSG_FW_CFG_HOST_HF_Q_SLICE1_OFFSET, 0xA00000U, 2, NRT_HOST_DESC_SMEM_SIZE / ICSSG_SWITCH_NORMAL_PD_SIZE);
    IcssgUtils_pdInitAndPush(hIcssg, HOST_SPPD1,    ICSSG_FW_CFG_HOST_SF_Q_SLICE1_OFFSET, 0xC00000U, 5, maxNumSpecialPDs);

    /* Port buffer pool memory */
    IcssgUtils_BufPoolCfg.poolLen  = fwPoolMem1->portBufferPoolSize;
    IcssgUtils_BufPoolCfg.poolBase = (uint32_t)fwPoolMem1->portBufferPoolMem;
    for (i = 0U; i < fwPoolMem1->portBufferPoolNum; i++)
    {
        IcssgUtils_WriteMem(dram + BUFFER_POOL_0_ADDR_OFFSET + (i * 8),
                            &IcssgUtils_BufPoolCfg,
                            ICSSG_FW_CFG_NORMAL_PD_SIZE_BYTE_COUNT);

        IcssgUtils_BufPoolCfg.poolBase += IcssgUtils_BufPoolCfg.poolLen;
    }

    /* Host buffer pool memory */
    IcssgUtils_BufPoolCfg.poolLen  = fwPoolMem1->hostBufferPoolSize;
    IcssgUtils_BufPoolCfg.poolBase = (uint32_t)fwPoolMem1->hostBufferPoolMem;
    for (i = ICSSG_SWITCH_PORT_BUFFER_POOL_NUM; i < (fwPoolMem1->hostBufferPoolNum + ICSSG_SWITCH_PORT_BUFFER_POOL_NUM); i++)
    {
        IcssgUtils_WriteMem(dram + BUFFER_POOL_0_ADDR_OFFSET + (i * 8),
                            &IcssgUtils_BufPoolCfg,
                            ICSSG_FW_CFG_NORMAL_PD_SIZE_BYTE_COUNT);

        IcssgUtils_BufPoolCfg.poolBase += IcssgUtils_BufPoolCfg.poolLen;
    }

    /* Host egress queue memory */
    startAddr = (uint32_t)fwPoolMem1->hostEgressQueueMem;
    endAddr   = startAddr +
                (fwPoolMem1->hostEgressQueueSize * fwPoolMem1->hostEgressQueueNum);

    Icssg_wr32(hIcssg, dram + HOST_RX_Q_EXP_CONTEXT_OFFSET,       startAddr);
    Icssg_wr32(hIcssg, dram + HOST_RX_Q_EXP_CONTEXT_OFFSET + 4U,  startAddr);
    Icssg_wr32(hIcssg, dram + HOST_RX_Q_EXP_CONTEXT_OFFSET + 8U,  startAddr);
    Icssg_wr32(hIcssg, dram + HOST_RX_Q_EXP_CONTEXT_OFFSET + 12U, endAddr);

    /* Host egress queue memory */
    startAddr = (uint32_t)fwPoolMem1->hostEgressPreQueueMem;
    endAddr   = startAddr +
                (fwPoolMem1->hostEgressPreQueueSize * fwPoolMem1->hostEgressQueueNum);

    Icssg_wr32(hIcssg, dram + HOST_RX_Q_PRE_CONTEXT_OFFSET,       startAddr);
    Icssg_wr32(hIcssg, dram + HOST_RX_Q_PRE_CONTEXT_OFFSET + 4U,  startAddr);
    Icssg_wr32(hIcssg, dram + HOST_RX_Q_PRE_CONTEXT_OFFSET + 8U,  startAddr);
    Icssg_wr32(hIcssg, dram + HOST_RX_Q_PRE_CONTEXT_OFFSET + 12U, endAddr);

    /* Program constant table pointer for C28 on all 3 PRU cores */
    Icssg_wr32(hIcssg, baseAddr + CSL_ICSS_G_PR1_PDSP1_IRAM_REGS_BASE + CSL_ICSS_G_PR1_PDSP1_IRAM_CONSTANT_TABLE_PROG_PTR_0, 0x100U);
    Icssg_wr32(hIcssg, baseAddr + CSL_ICSS_G_PR1_RTU1_PR1_RTU1_IRAM_REGS_BASE + CSL_ICSS_G_PR1_PDSP1_IRAM_CONSTANT_TABLE_PROG_PTR_0, 0x100U);
    Icssg_wr32(hIcssg, baseAddr + CSL_ICSS_G_PR1_PDSP_TX1_IRAM_REGS_BASE + CSL_ICSS_G_PR1_PDSP1_IRAM_CONSTANT_TABLE_PROG_PTR_0, 0x100U);

    /* Ingress rate limiter in FW uses Class8 and Class9 for rate control,
     * make them hit always by default */
    Icssg_wr32(hIcssg,
               baseAddr +
               CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_REGS_BASE +
               CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS_GATES8_PRU1,
               0x70);
    Icssg_wr32(hIcssg,
               baseAddr +
               CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_REGS_BASE +
               CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS_GATES9_PRU1,
               0x70);

    miiRg = baseAddr + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_REGS_BASE;
    Icssg_wr32(hIcssg, miiRg + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT1_START_LEN_PRU1,  0x60000U);
    Icssg_wr32(hIcssg, miiRg + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT1_CFG_PRU1,        0x5555U);
    Icssg_wr32(hIcssg, miiRg + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT1_1_DA0_PRU1,      0x00C28001U); /* 01:80:C2:00 */
    Icssg_wr32(hIcssg, miiRg + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT1_1_DA1_PRU1,      0x0E00U);     /* 00:0X */
    Icssg_wr32(hIcssg, miiRg + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT1_1_DA_MASK0_PRU1, 0x0U);
    Icssg_wr32(hIcssg, miiRg + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT1_1_DA_MASK1_PRU1, 0x0F00U);

    regVal = Icssg_rd32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FDB_GEN_CFG2);
    regVal |= (1U << CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FDB_GEN_CFG2_FDB_PRU0_EN_SHIFT) |
              (1U << CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FDB_GEN_CFG2_FDB_PRU1_EN_SHIFT) |
              (1U << CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FDB_GEN_CFG2_FDB_HOST_EN_SHIFT);
    Icssg_wr32(hIcssg, cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FDB_GEN_CFG2, regVal);
}

int32_t IcssgUtils_createPruss(Icssg_Handle hIcssg)
{
    PRUICSS_Handle hPruss;
    uint32_t index = hIcssg->pruss->name;
    int32_t status = ENET_SOK;

    EnetOsal_lockMutex(hIcssg->pruss->lock);

    /* Create PRUICSS instance object and initialize memory */
    if (!hIcssg->pruss->initialized)
    {
        /* PRU-ICSS instances are 1-relative */
        /* Index of required PRU instance in PRU module of syscfg */
        hPruss = PRUICSS_open(index);

        if (hPruss != NULL)
        {
            PRUICSS_initMemory(hPruss, PRUICSS_SHARED_RAM);
            hIcssg->pruss->hPruss = hPruss;
            hIcssg->pruss->initialized = true;
            Enet_assert((hPruss->hwAttrs->instance == hIcssg->pruss->instance),
                         "%s: Mismatched PRUSS instance %u\n",
                         ENET_PER_NAME(hIcssg), index);
        }
        else
        {
            ENETTRACE_ERR("%s: failed to create PRUICSS instance %u\n",
                          ENET_PER_NAME(hIcssg), index);
            status = ENET_EFAIL;
        }
    }

    EnetOsal_unlockMutex(hIcssg->pruss->lock);

    return status;
}

int32_t IcssgUtils_enablePruss(Icssg_Handle hIcssg,
                               Enet_MacPort macPort)
{
    PRUICSS_Handle hPruss = hIcssg->pruss->hPruss;
    uint8_t pruNum, rtuNum, txpruNum;
    uint32_t slice = IcssgUtils_getSliceNum(hIcssg, macPort);
    uint32_t portId = ENET_MACPORT_ID(macPort);
    int32_t status = ENET_SOK;

    ENETTRACE_VAR(portId);

    pruNum = ICSSG_IS_SLICE_0(slice) ? PRUICSS_PRU0 : PRUICSS_PRU1;
    rtuNum = ICSSG_IS_SLICE_0(slice) ? PRUICSS_RTU_PRU0 : PRUICSS_RTU_PRU1;
    txpruNum = ICSSG_IS_SLICE_0(slice) ? PRUICSS_TX_PRU0 : PRUICSS_TX_PRU1;

    EnetOsal_lockMutex(hIcssg->pruss->lock);

    if (PRUICSS_enableCore(hPruss, pruNum) != SystemP_SUCCESS)
    {
        ENETTRACE_ERR("%s: failed to enable PRU core for port %u\n",
                      ENET_PER_NAME(hIcssg), portId);
        status = ENET_EFAIL;
    }

    if (status == ENET_SOK)
    {
        if (PRUICSS_enableCore(hPruss, rtuNum) != SystemP_SUCCESS)
        {
            ENETTRACE_ERR("%s: failed to enable RTU core for port %u\n",
                          ENET_PER_NAME(hIcssg), portId);
            status = ENET_EFAIL;
        }
    }

    if (status == ENET_SOK)
    {
        if (PRUICSS_enableCore(hPruss, txpruNum) != SystemP_SUCCESS)
        {
            ENETTRACE_ERR("%s: failed to enable TX PRU core for port %u\n",
                          ENET_PER_NAME(hIcssg), portId);
            status = ENET_EFAIL;
        }
    }

    EnetOsal_unlockMutex(hIcssg->pruss->lock);

    return ENET_SOK;
}

int32_t IcssgUtils_disablePruss(Icssg_Handle hIcssg,
                                Enet_MacPort macPort)
{
    PRUICSS_Handle hPruss = hIcssg->pruss->hPruss;
    uint8_t pruNum, rtuNum, txpruNum;
    uint32_t slice = IcssgUtils_getSliceNum(hIcssg, macPort);
    uint32_t portId = ENET_MACPORT_ID(macPort);
    int32_t status = ENET_SOK;

    ENETTRACE_VAR(portId);

    pruNum = ICSSG_IS_SLICE_0(slice) ? PRUICSS_PRU0 : PRUICSS_PRU1;
    rtuNum = ICSSG_IS_SLICE_0(slice) ? PRUICSS_RTU_PRU0 : PRUICSS_RTU_PRU1;
    txpruNum = ICSSG_IS_SLICE_0(slice) ? PRUICSS_TX_PRU0 : PRUICSS_TX_PRU1;

    EnetOsal_lockMutex(hIcssg->pruss->lock);

    if (PRUICSS_disableCore(hPruss, pruNum) != SystemP_SUCCESS)
    {
        ENETTRACE_ERR("%s: failed to disable PRU core for port %u\n",
                      ENET_PER_NAME(hIcssg), portId);
        status = ENET_EFAIL;
    }

    if (status == ENET_SOK)
    {
        if (PRUICSS_disableCore(hPruss, rtuNum) != SystemP_SUCCESS)
        {
            ENETTRACE_ERR("%s: failed to disable RTU core for port %u\n",
                          ENET_PER_NAME(hIcssg), portId);
            status = ENET_EFAIL;
        }
    }

    if (status == ENET_SOK)
    {
        if (PRUICSS_disableCore(hPruss, txpruNum) != SystemP_SUCCESS)
        {
            ENETTRACE_ERR("%s: failed to disable TX PRU core for port %u\n",
                          ENET_PER_NAME(hIcssg), portId);
            status = ENET_EFAIL;
        }
    }

    EnetOsal_unlockMutex(hIcssg->pruss->lock);

    return status;
}

int32_t IcssgUtils_downloadFirmware(Icssg_Handle hIcssg,
                                    Enet_MacPort macPort,
                                    const Icssg_Fw *fw)
{
    PRUICSS_Handle hPruss = hIcssg->pruss->hPruss;
    uint32_t portId = ENET_MACPORT_ID(macPort);
    uint32_t slice = IcssgUtils_getSliceNum(hIcssg, macPort);
    int32_t status = ENET_SOK;
    int32_t retVal;

    ENETTRACE_VAR(portId);

    /* PRU core */
    retVal = PRUICSS_writeMemory(hPruss,
                                    PRUICSS_IRAM_PRU(slice),
                                    0,
                                    fw->pru,
                                    fw->pruSize);
    if (retVal == 0)
    {
        ENETTRACE_ERR("%s: Port %u: firmware download failure for PRU core: %d\n",
                      ENET_PER_NAME(hIcssg), portId, retVal);
        status = ENET_EFAIL;
    }

    /* RTU core */
    if (status == ENET_SOK)
    {
        retVal = PRUICSS_writeMemory(hPruss,
                                        PRUICSS_IRAM_RTU_PRU(slice),
                                        0,
                                        fw->rtu,
                                        fw->rtuSize);
        if (retVal == 0)
        {
            ENETTRACE_ERR("%s: Port %u: firmware download failure for RTU core: %d\n",
                          ENET_PER_NAME(hIcssg), portId, retVal);
            status = ENET_EFAIL;
        }
    }

    /* TX PRU core */
    if (status == ENET_SOK)
    {
        retVal = PRUICSS_writeMemory(hPruss,
                                        PRUICSS_IRAM_TX_PRU(slice),
                                        0,
                                        fw->txpru,
                                        fw->txpruSize);
        if (retVal == 0)
        {
            ENETTRACE_ERR("%s: Port %u: firmware download failure for TX PRU core: %d\n",
                          ENET_PER_NAME(hIcssg), portId, retVal);
            status = ENET_EFAIL;
        }
    }

    return status;
}

void IcssgUtils_classiDisable(Icssg_Handle hIcssg,
                              Enet_MacPort macPort)
{
    uintptr_t cfgRegs = Icssg_getCfgAddr(hIcssg);
    uint32_t slice = IcssgUtils_getSliceNum(hIcssg, macPort);
    uint32_t classi;
    uintptr_t addr;
    uint32_t temp;

    for (classi = 0U; classi<ICSSG_UTILS_NUM_CLASSIFIERS; classi++)
    {
        addr = cfgRegs + (classi * 8U);
        addr += ICSSG_IS_SLICE_0(slice) ?
                CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS0_AND_EN_PRU0 :
                CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS0_AND_EN_PRU1;

        Icssg_wr32(hIcssg, addr + ICSSG_UTILS_CLASSI_AND_EN_OFF, 0x0);
        Icssg_wr32(hIcssg, addr + ICSSG_UTILS_CLASSI_OR_EN_OFF, 0x0);


        addr = cfgRegs + ICSSG_UTILS_CLASSI_CFG1_OFF;
        addr += ICSSG_IS_SLICE_0(slice) ?
                CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS0_AND_EN_PRU0 :
                CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RX_CLASS0_AND_EN_PRU1;

        temp = Icssg_rd32(hIcssg, addr + ICSSG_UTILS_CLASSI_CFG2_OFF);
        temp &= ~(1U<< (classi+16U));
        temp &= ~(1U<< (classi));
        Icssg_wr32(hIcssg, addr + ICSSG_UTILS_CLASSI_CFG2_OFF, temp);
        temp = Icssg_rd32(hIcssg, addr);
        /* clear the bits */
        temp &= ~(0x3U << (classi*2U));
        temp |=    (0U << (classi *2U));
        Icssg_wr32(hIcssg, addr, temp);

        temp = Icssg_rd32(hIcssg, addr + ICSSG_UTILS_CLASSI_GATE_OFF);
        /*clear the classi & allow masks */
        temp &= ~(0x60U );
        /*set rate & phase masks */
        temp |= (0x50U);
        Icssg_wr32(hIcssg, addr + ICSSG_UTILS_CLASSI_GATE_OFF+classi*4, temp);
    }
}

void IcssgUtils_configFilter3(Icssg_Handle hIcssg,
                              Enet_MacPort macPort,
                              uint32_t filterNum,
                              Icssg_Filter3Cfg *ftCfg)
{
    uintptr_t cfgRegs = Icssg_getCfgAddr(hIcssg);
    uint32_t slice = IcssgUtils_getSliceNum(hIcssg, macPort);
    uintptr_t icssg_filter_base;
    uintptr_t offsetToFt3Pattern = (CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT3_0_P0_PRU0 -
                                    CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT3_0_START_PRU0);

    if (ICSSG_IS_SLICE_0(slice))
    {
        icssg_filter_base = cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT3_0_START_PRU0;
    }
    else
    {
        icssg_filter_base = cfgRegs + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_FT3_0_START_PRU1;
    }

    Icssg_wr32(hIcssg, icssg_filter_base + filterNum*32,    ftCfg->ft3Start);
    Icssg_wr32(hIcssg, icssg_filter_base + filterNum*32+4,  ftCfg->ft3StartAuto);
    Icssg_wr32(hIcssg, icssg_filter_base + filterNum*32+8,  ftCfg->ft3StartOffset);
    Icssg_wr32(hIcssg, icssg_filter_base + filterNum*32+12, ftCfg->ft3JmpOffset);
    Icssg_wr32(hIcssg, icssg_filter_base + filterNum*32+16, ftCfg->ft3Len);
    Icssg_wr32(hIcssg, icssg_filter_base + filterNum*32+20, ftCfg->ft3Config);
    Icssg_wr32(hIcssg, icssg_filter_base + filterNum*32+24, ftCfg->ft3Type);
    Icssg_wr32(hIcssg, icssg_filter_base + filterNum*32+28, ftCfg->ft3TypeMask);
    Icssg_wr32(hIcssg, icssg_filter_base + filterNum*16 + offsetToFt3Pattern,    ftCfg->ft3PatternLow);
    Icssg_wr32(hIcssg, icssg_filter_base + filterNum*16 + offsetToFt3Pattern+4,  ftCfg->ft3PatternHigh);
    Icssg_wr32(hIcssg, icssg_filter_base + filterNum*16 + offsetToFt3Pattern+8,  ftCfg->ft3PatternMaskLow);
    Icssg_wr32(hIcssg, icssg_filter_base + filterNum*16 + offsetToFt3Pattern+12, ftCfg->ft3PatternMaskHigh);
}

int32_t IcssgUtils_checkFwPoolMem(Icssg_Handle hIcssg,
                                  const Icssg_FwPoolMem *fwPoolMem)
{
    EnetPer_Handle hPer = (EnetPer_Handle)hIcssg;
    uint32_t portBufferPoolNum;
    uint32_t hostBufferPoolNum;
    uint32_t hostEgressQueueNum;
    int32_t status = ENET_SOK;

    if (hPer->enetType == ENET_ICSSG_DUALMAC)
    {
        portBufferPoolNum  = ICSSG_DUALMAC_PORT_BUFFER_POOL_NUM;
        hostBufferPoolNum  = ICSSG_DUALMAC_GET_HOST_BUFFER_POOL_NUM(hIcssg->qosLevels);
        hostEgressQueueNum = ICSSG_DUALMAC_HOST_EGRESS_QUEUE_NUM;
    }
    else
    {
        portBufferPoolNum  = ICSSG_SWITCH_PORT_BUFFER_POOL_NUM;
        hostBufferPoolNum  = ICSSG_SWITCH_GET_HOST_BUFFER_POOL_NUM(hIcssg->qosLevels);
        hostEgressQueueNum = ICSSG_SWITCH_HOST_EGRESS_QUEUE_NUM;
    }

    /* Port buffer pool - check number of pools, valid address/size and cache alignment */
    if (portBufferPoolNum != fwPoolMem->portBufferPoolNum)
    {
        ENETTRACE_ERR("%s: Invalid number of port buffer pools (expected %u, got %u)\n",
                      ENET_PER_NAME(hIcssg), portBufferPoolNum, fwPoolMem->portBufferPoolNum);
        status = ENET_EINVALIDPARAMS;
    }
    else
    {
        if (portBufferPoolNum != 0U)
        {
            if ((fwPoolMem->portBufferPoolMem == NULL) ||
                (fwPoolMem->portBufferPoolSize == 0U))
            {
                ENETTRACE_ERR("%s: Invalid port buffer pool memory (addr=0x%08x size=%u bytes)\n",
                              ENET_PER_NAME(hIcssg), fwPoolMem->portBufferPoolMem, fwPoolMem->portBufferPoolSize);
                status = ENET_EINVALIDPARAMS;
            }
            else
            {
                if (!ENET_UTILS_IS_ALIGNED(fwPoolMem->portBufferPoolMem, ICSSG_CACHELINE_ALIGNMENT))
                {
                    ENETTRACE_ERR("%s: Port buffer pool memory is not aligned (addr 0x%08x)\n",
                                  ENET_PER_NAME(hIcssg), fwPoolMem->portBufferPoolMem);
                    status = ENET_EINVALIDPARAMS;
                }
            }
        }
        else
        {
            if ((fwPoolMem->portBufferPoolMem != NULL) ||
                (fwPoolMem->portBufferPoolSize != 0U))
            {
                ENETTRACE_ERR("%s: Valid port buffer pool memory passed, but none required\n", ENET_PER_NAME(hIcssg));
                status = ENET_EINVALIDPARAMS;
            }
        }
    }

    /* Host buffer pool - check number of pools, valid address/size and cache alignment */
    if (hostBufferPoolNum != fwPoolMem->hostBufferPoolNum)
    {
        ENETTRACE_ERR("%s: Invalid number of host buffer pools (expected %u, got %u)\n",
                      ENET_PER_NAME(hIcssg), hostBufferPoolNum, fwPoolMem->hostBufferPoolNum);
        status = ENET_EINVALIDPARAMS;
    }
    else
    {
        if (hostBufferPoolNum != 0U)
        {
            if ((fwPoolMem->hostBufferPoolMem == NULL) ||
                (fwPoolMem->hostBufferPoolSize == 0U))
            {
                ENETTRACE_ERR("%s: Invalid host buffer pool memory (addr=0x%08x size=%u bytes)\n",
                              ENET_PER_NAME(hIcssg), fwPoolMem->hostBufferPoolMem, fwPoolMem->hostBufferPoolSize);
                status = ENET_EINVALIDPARAMS;
            }
            else
            {
                if (!ENET_UTILS_IS_ALIGNED(fwPoolMem->hostBufferPoolMem, ICSSG_CACHELINE_ALIGNMENT))
                {
                    ENETTRACE_ERR("%s: host buffer pool memory is not aligned (addr 0x%08x)\n",
                                  ENET_PER_NAME(hIcssg), fwPoolMem->hostBufferPoolMem);
                    status = ENET_EINVALIDPARAMS;
                }
            }
        }
        else
        {
            if ((fwPoolMem->hostBufferPoolMem != NULL) ||
                (fwPoolMem->hostBufferPoolSize != 0U))
            {
                ENETTRACE_ERR("%s: Valid host buffer pool memory passed, but none required\n", ENET_PER_NAME(hIcssg));
                status = ENET_EINVALIDPARAMS;
            }
        }
    }

    /* Host egress queues - check number of pools, valid address/size and cache alignment */
    if (hostEgressQueueNum != fwPoolMem->hostEgressQueueNum)
    {
        ENETTRACE_ERR("%s: Invalid number of host egress queues (expected %u, got %u)\n",
                      ENET_PER_NAME(hIcssg), hostEgressQueueNum, fwPoolMem->hostEgressQueueNum);
        status = ENET_EINVALIDPARAMS;
    }
    else
    {
        if (hostEgressQueueNum != 0U)
        {
            if ((fwPoolMem->hostEgressQueueMem == NULL) ||
                (fwPoolMem->hostEgressQueueSize == 0U))
            {
                ENETTRACE_ERR("%s: Invalid host egress queue memory (addr=0x%08x size=%u bytes)\n",
                              ENET_PER_NAME(hIcssg), fwPoolMem->hostEgressQueueMem, fwPoolMem->hostEgressQueueSize);
                status = ENET_EINVALIDPARAMS;
            }
            else
            {
                if (!ENET_UTILS_IS_ALIGNED(fwPoolMem->hostEgressQueueMem, ICSSG_CACHELINE_ALIGNMENT))
                {
                    ENETTRACE_ERR("%s: Host egress queue memory is not aligned (addr 0x%08x)\n",
                                  ENET_PER_NAME(hIcssg), fwPoolMem->hostEgressQueueMem);
                    status = ENET_EINVALIDPARAMS;
                }
            }
        }
        else
        {
            if ((fwPoolMem->hostEgressQueueMem != NULL) ||
                (fwPoolMem->hostEgressQueueSize != 0U))
            {
                ENETTRACE_ERR("%s: Valid host egress queue memory passed, but none required\n", ENET_PER_NAME(hIcssg));
                status = ENET_EINVALIDPARAMS;
            }
        }
    }

    /* Host egress queues - check number of pools, valid address/size and cache alignment */
    if (hostEgressQueueNum != fwPoolMem->hostEgressQueueNum)
    {
        ENETTRACE_ERR("%s: Invalid number of host egress queues (expected %u, got %u)\n",
                      ENET_PER_NAME(hIcssg), hostEgressQueueNum, fwPoolMem->hostEgressQueueNum);
        status = ENET_EINVALIDPARAMS;
    }
    else
    {

        if(hIcssg->isPremQueEnable)
        {
            if (hostEgressQueueNum != 0U)
            {
                if ((fwPoolMem->hostEgressPreQueueMem == NULL) ||
                    (fwPoolMem->hostEgressPreQueueSize == 0U))
                {
                    ENETTRACE_ERR("%s: Invalid host egress queue memory (addr=0x%08x size=%u bytes)\n",
                                  ENET_PER_NAME(hIcssg), fwPoolMem->hostEgressPreQueueMem, fwPoolMem->hostEgressPreQueueSize);
                    status = ENET_EINVALIDPARAMS;
                }
                else
                {
                    if (!ENET_UTILS_IS_ALIGNED(fwPoolMem->hostEgressPreQueueMem, ICSSG_CACHELINE_ALIGNMENT))
                    {
                        ENETTRACE_ERR("%s: Host egress queue memory is not aligned (addr 0x%08x)\n",
                                      ENET_PER_NAME(hIcssg), fwPoolMem->hostEgressPreQueueMem);
                        status = ENET_EINVALIDPARAMS;
                    }
                }
            }
            else
            {
                if ((fwPoolMem->hostEgressPreQueueMem != NULL) ||
                    (fwPoolMem->hostEgressPreQueueSize != 0U))
                {
                    ENETTRACE_ERR("%s: Valid host egress queue memory passed, but none required\n", ENET_PER_NAME(hIcssg));
                    status = ENET_EINVALIDPARAMS;
                }
            }
        }
    }

    /* Scratch buffer - used for error frames and large frames */
    if ((fwPoolMem->scratchBufferMem == NULL) ||
        (fwPoolMem->scratchBufferSize != ICSSG_SCRATCH_BUFFER_SIZE))
    {
        ENETTRACE_ERR("%s: Invalid scratch buffer (addr=0x%08x size=%u bytes)\n",
                      ENET_PER_NAME(hIcssg), fwPoolMem->scratchBufferMem, fwPoolMem->scratchBufferSize);
        status = ENET_EINVALIDPARAMS;
    }

    return status;
}

