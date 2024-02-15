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
 * \file  cpsw_hostport.c
 *
 * \brief This file contains the implementation of the CPSW host port module.
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
#include <include/mod/cpsw_hostport.h>
#include <priv/core/enet_trace_priv.h>
#include <priv/mod/cpsw_hostport_priv.h>
#include <priv/mod/cpsw_hostport_ioctl_priv.h>
#include <priv/mod/cpsw_clks.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Supported AM64x version */
#define CPSW_HOSTPORT_VER_REVMAJ_AM64X       (0x00000001U)
#define CPSW_HOSTPORT_VER_REVMIN_AM64X       (0x00000003U)
#define CPSW_HOSTPORT_VER_REVRTL_AM64X       (0x00000001U)
#define CPSW_HOSTPORT_VER_ID_AM64X           (0x00006BA8U)

/* Supported AM273X version */
#define CPSW_HOSTPORT_VER_REVMAJ_AM273X        (0x00000001U)
#define CPSW_HOSTPORT_VER_REVMIN_AM273X        (0x00000002U)
#define CPSW_HOSTPORT_VER_REVRTL_AM273X        (0x00000000U)
#define CPSW_HOSTPORT_VER_ID_AM273X            (0x00006B90U)

/* Supported AM263X version */
#define CPSW_HOSTPORT_VER_REVMAJ_AM263X        (0x00000001U)
#define CPSW_HOSTPORT_VER_REVMIN_AM263X        (0x00000003U)
#define CPSW_HOSTPORT_VER_REVRTL_AM263X        (0x00000002U)
#define CPSW_HOSTPORT_VER_ID_AM263X            (0x00006B90U)

/* Supported AWR294X version */
#define CPSW_HOSTPORT_VER_REVMAJ_AWR294X        (0x00000001U)
#define CPSW_HOSTPORT_VER_REVMIN_AWR294X        (0x00000002U)
#define CPSW_HOSTPORT_VER_REVRTL_AWR294X        (0x00000000U)
#define CPSW_HOSTPORT_VER_ID_AWR294X            (0x00006B90U)

/* Supported AWR2544 version */
#define CPSW_HOSTPORT_VER_REVMAJ_AWR2544        (0x00000001U)
#define CPSW_HOSTPORT_VER_REVMIN_AWR2544        (0x00000003U)
#define CPSW_HOSTPORT_VER_REVRTL_AWR2544        (0x00000003U)
#define CPSW_HOSTPORT_VER_ID_AWR2544            (0x00006B90U)

/* Supported AM62AX version */
#define CPSW_HOSTPORT_VER_REVMAJ_AM62AX        (0x00000001U)
#define CPSW_HOSTPORT_VER_REVMIN_AM62AX        (0x00000003U)
#define CPSW_HOSTPORT_VER_REVRTL_AM62AX        (0x00000002U)
#define CPSW_HOSTPORT_VER_ID_AM62AX            (0x00006BA8U)

/*! \brief Default value used for host port RX MTU. */
#define CPSW_HOSTPORT_RX_MTU_DEFAULT          (1518U)

#define CPSW_HOSTPORT_IOCTL_HANDLER_ENTRY_INIT(x)    \
          {.cmd = x,                            \
           .fxn = &CpswHostPort_ioctl_handler_##x}

#define CPSW_HOSTPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(x)    \
          {.cmd = x,                            \
           .fxn = &CpswHostPort_ioctl_handler_default}

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef int32_t (CpswHostPortIoctlHandler)(CpswHostPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);

typedef struct CpswHostPortIoctlHandlerRegistry_s
{
    uint32_t cmd;
    CpswHostPortIoctlHandler *fxn;
} CpswHostPortIoctlHandlerRegistry_t;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

#if ENET_CFG_IS_ON(DEV_ERROR)
static int32_t CpswHostPort_isSupported(CSL_Xge_cpswRegs *regs);
#endif


static int32_t CpswHostPort_ioctl_handler_default(CpswHostPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
static int32_t CpswHostPort_ioctl_handler_CPSW_HOSTPORT_IOCTL_REGISTER_HANDLER(CpswHostPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
static int32_t CpswHostPort_setIoctlHandlerFxn(uint32_t ioctlCmd,
                                          CpswHostPortIoctlHandler *ioctlHandlerFxn,
                                          CpswHostPortIoctlHandlerRegistry_t *ioctlRegistryTbl,
                                          uint32_t tableSize);
static CpswHostPortIoctlHandler * CpswHostPort_getIoctlHandlerFxn(uint32_t ioctlCmd, CpswHostPortIoctlHandlerRegistry_t *ioctlRegistryTbl, uint32_t tableSize);
static int32_t CpswHostPort_getIoctlHandlerIdx(uint32_t ioctlCmd, CpswHostPortIoctlHandlerRegistry_t *ioctlRegistryTbl, uint32_t tableSize, uint32_t *tblIdx);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

#if ENET_CFG_IS_ON(DEV_ERROR)
/*! \brief CPSW hostport versions supported by this driver. */
static CSL_CPSW_VERSION CpswHostPort_gSupportedVer[] =
{
    {   /* AM64x */
        .majorVer = CPSW_HOSTPORT_VER_REVMAJ_AM64X,
        .minorVer = CPSW_HOSTPORT_VER_REVMIN_AM64X,
        .rtlVer   = CPSW_HOSTPORT_VER_REVRTL_AM64X,
        .id       = CPSW_HOSTPORT_VER_ID_AM64X,
    },
    {   /* AM273X */
        .majorVer = CPSW_HOSTPORT_VER_REVMAJ_AM273X,
        .minorVer = CPSW_HOSTPORT_VER_REVMIN_AM273X,
        .rtlVer   = CPSW_HOSTPORT_VER_REVRTL_AM273X,
        .id       = CPSW_HOSTPORT_VER_ID_AM273X,
    },
	{   /* AM263X */
        .majorVer = CPSW_HOSTPORT_VER_REVMAJ_AM263X,
        .minorVer = CPSW_HOSTPORT_VER_REVMIN_AM263X,
        .rtlVer   = CPSW_HOSTPORT_VER_REVRTL_AM263X,
        .id       = CPSW_HOSTPORT_VER_ID_AM263X,
    },
    {   /* AWR294X */
        .majorVer = CPSW_HOSTPORT_VER_REVMAJ_AWR294X,
        .minorVer = CPSW_HOSTPORT_VER_REVMIN_AWR294X,
        .rtlVer   = CPSW_HOSTPORT_VER_REVRTL_AWR294X,
        .id       = CPSW_HOSTPORT_VER_ID_AWR294X,
    },
    {   /* AWR2544 */
        .majorVer = CPSW_HOSTPORT_VER_REVMAJ_AWR2544,
        .minorVer = CPSW_HOSTPORT_VER_REVMIN_AWR2544,
        .rtlVer   = CPSW_HOSTPORT_VER_REVRTL_AWR2544,
        .id       = CPSW_HOSTPORT_VER_ID_AWR2544,
    },
    {   /* AM62AX */
        .majorVer = CPSW_HOSTPORT_VER_REVMAJ_AM62AX,
        .minorVer = CPSW_HOSTPORT_VER_REVMIN_AM62AX,
        .rtlVer   = CPSW_HOSTPORT_VER_REVRTL_AM62AX,
        .id       = CPSW_HOSTPORT_VER_ID_AM62AX,
    },
};

/* Public host port IOCTL validation data. */
static Enet_IoctlValidate gCpswHostPort_ioctlValidate[] =
{
    ENET_IOCTL_VALID_PRMS(CPSW_HOSTPORT_IOCTL_GET_FIFO_STATS,
                          0U,
                          sizeof(CpswHostPort_FifoStats)),

    ENET_IOCTL_VALID_PRMS(CPSW_HOSTPORT_GET_FLOW_ID_OFFSET,
                          0U,
                          sizeof(uint32_t)),
};

/* Private host port IOCTL validation data. */
static Enet_IoctlValidate gCpswHostPort_privIoctlValidate[] =
{
    ENET_IOCTL_VALID_PRMS(CPSW_HOSTPORT_SET_FLOW_ID_OFFSET,
                          sizeof(uint32_t),
                          0U),
    ENET_IOCTL_VALID_PRMS(CPSW_HOSTPORT_IOCTL_REGISTER_HANDLER,
                          sizeof(Enet_IoctlRegisterHandlerInArgs),
                          0U),

};
#endif

static CpswHostPortIoctlHandlerRegistry_t CpswHostPortIoctlHandlerRegistry[] =
{
    CPSW_HOSTPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_HOSTPORT_IOCTL_GET_VERSION),
    CPSW_HOSTPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_HOSTPORT_IOCTL_PRINT_REGS),
    CPSW_HOSTPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_HOSTPORT_IOCTL_ENABLE),
    CPSW_HOSTPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_HOSTPORT_IOCTL_DISABLE),
    CPSW_HOSTPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_HOSTPORT_IOCTL_SET_INGRESS_DSCP_PRI_MAP),
    CPSW_HOSTPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_HOSTPORT_IOCTL_GET_INGRESS_DSCP_PRI_MAP),
    CPSW_HOSTPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_HOSTPORT_IOCTL_SET_PRI_REGEN_MAP),
    CPSW_HOSTPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_HOSTPORT_IOCTL_GET_PRI_REGEN_MAP),
    CPSW_HOSTPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_HOSTPORT_IOCTL_SET_EGRESS_QOS_PRI_MAP),
    CPSW_HOSTPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_HOSTPORT_IOCTL_GET_EGRESS_QOS_PRI_MAP),
    CPSW_HOSTPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_HOSTPORT_IOCTL_ENABLE_INGRESS_TRAFFIC_SHAPING),
    CPSW_HOSTPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_HOSTPORT_IOCTL_DISABLE_INGRESS_TRAFFIC_SHAPING),
    CPSW_HOSTPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_HOSTPORT_IOCTL_GET_INGRESS_TRAFFIC_SHAPING),
    CPSW_HOSTPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_HOSTPORT_IOCTL_SET_CREDIT_BASED_SHAPING),
    CPSW_HOSTPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_HOSTPORT_IOCTL_GET_CREDIT_BASED_SHAPING),
    CPSW_HOSTPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_HOSTPORT_IOCTL_GET_MAXLEN),
    CPSW_HOSTPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_HOSTPORT_IS_CSUM_OFFLOAD_ENABLED),
    CPSW_HOSTPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_HOSTPORT_IOCTL_GET_FIFO_STATS),
    CPSW_HOSTPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_HOSTPORT_GET_FLOW_ID_OFFSET),
    CPSW_HOSTPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_HOSTPORT_SET_FLOW_ID_OFFSET),
    CPSW_HOSTPORT_IOCTL_HANDLER_ENTRY_INIT(CPSW_HOSTPORT_IOCTL_REGISTER_HANDLER),
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void CpswHostPort_initCfg(CpswHostPort_Cfg *hostPortCfg)
{
    hostPortCfg->crcType           = ENET_CRC_ETHERNET;
    hostPortCfg->removeCrc         = false;
    hostPortCfg->padShortPacket    = true; /* Short packets were dropped by default */
    hostPortCfg->passCrcErrors     = false;
    hostPortCfg->passPriorityTaggedUnchanged = false;
    hostPortCfg->rxVlanRemapEn     = false;
    hostPortCfg->rxDscpIPv4RemapEn = false;
    hostPortCfg->rxDscpIPv6RemapEn = false;
    hostPortCfg->rxMtu             = CPSW_HOSTPORT_RX_MTU_DEFAULT;
    hostPortCfg->vlanCfg.portPri   = 0U;
    hostPortCfg->vlanCfg.portCfi   = 0U;
    hostPortCfg->vlanCfg.portVID   = 0U;
    hostPortCfg->rxPriorityType    = ENET_INGRESS_PRI_TYPE_FIXED;
    hostPortCfg->txPriorityType    = ENET_EGRESS_PRI_TYPE_FIXED;
    hostPortCfg->rxCsumOffloadEn   = true;
    hostPortCfg->txCsumOffloadEn   = true;
}

int32_t CpswHostPort_open(EnetMod_Handle hMod,
                          Enet_Type enetType,
                          uint32_t instId,
                          const void *cfg,
                          uint32_t cfgSize)
{
    CpswHostPort_Handle hPort = (CpswHostPort_Handle)hMod;
    const CpswHostPort_Cfg *hostPortCfg = (const CpswHostPort_Cfg *)cfg;
    CSL_Xge_cpswRegs *regs = (CSL_Xge_cpswRegs *)hMod->virtAddr;
    CSL_CPSW_CONTROL control;
    CSL_CPSW_PTYPE pType;
    CSL_CPSW_CPPI_P0_CONTROL cppiP0ControlCfg;
    uint32_t status = ENET_SOK;

    Enet_devAssert(cfgSize == sizeof(CpswHostPort_Cfg),
                   "Invalid host port config params size %u (expected %u)\n",
                   cfgSize, sizeof(CpswHostPort_Cfg));

    Enet_devAssert(regs != NULL, "CPSW hostport regs address is not valid\n");

    /* Check supported host port module versions */
#if ENET_CFG_IS_ON(DEV_ERROR)
    status = CpswHostPort_isSupported(regs);
    Enet_devAssert(status == ENET_SOK, "Host port version is not supported\n");
#endif

    /* Save peripheral info to use it later to query SoC parameters */
    hPort->enetType = enetType;
    hPort->instId = instId;

#if ENET_CFG_IS_ON(CPSW_CPPI_CAST)
    if (hostPortCfg->crcType == ENET_CRC_ETHERNET)
    {
        CSL_CPSW_disableP0TxCastagnoliCRC(regs);
    }
    else
    {
        CSL_CPSW_enableP0TxCastagnoliCRC(regs);
    }
#endif

    CSL_CPSW_getCpswControlReg(regs, &control);
    control.p0Enable       = false;
    control.p0PassPriTag   = hostPortCfg->passPriorityTaggedUnchanged;
#if ENET_CFG_IS_ON(DISABLE_CRC_STRIP)
    ENETTRACE_WARN_IF(hostPortCfg->removeCrc,
        "ETHFW-1705 - CRC removal is not allowed, CRC will be passed in packet buffer\n");
    control.p0TxCrcRemove  = false;
#else
    control.p0TxCrcRemove  = hostPortCfg->removeCrc;
#endif
    control.p0RxPad        = hostPortCfg->padShortPacket;
    control.p0RxPassCrcErr = hostPortCfg->passCrcErrors;
    CSL_CPSW_setCpswControlReg(regs, &control);

    CSL_CPSW_setPort0VlanReg(regs,
                             hostPortCfg->vlanCfg.portVID,
                             hostPortCfg->vlanCfg.portCfi,
                             hostPortCfg->vlanCfg.portPri);

    CSL_CPSW_setPort0RxMaxLen(regs, hostPortCfg->rxMtu);

    CSL_CPSW_getPTypeReg(regs, &pType);
    if (hostPortCfg->txPriorityType == ENET_EGRESS_PRI_TYPE_FIXED)
    {
        pType.port0PriorityTypeEscalateEnable = FALSE;
    }
    else
    {
        pType.port0PriorityTypeEscalateEnable = TRUE;
    }

    CSL_CPSW_setPTypeReg(regs, &pType);

    CSL_CPSW_getCppiP0Control(regs, &cppiP0ControlCfg);

#if (ENET_CFG_IS_ON(CPSW_CSUM_OFFLOAD_SUPPORT) == 1)
    cppiP0ControlCfg.p0RxChksumEn = hostPortCfg->txCsumOffloadEn; // FHOST
    cppiP0ControlCfg.p0TxChksumEn = hostPortCfg->rxCsumOffloadEn; // THOST
#else
    cppiP0ControlCfg.p0RxChksumEn = FALSE;
    cppiP0ControlCfg.p0TxChksumEn = FALSE;
#endif

    cppiP0ControlCfg.p0RxRemapVlan     = hostPortCfg->rxVlanRemapEn ? TRUE : FALSE;
    cppiP0ControlCfg.p0RxRemapDscpIpv4 = hostPortCfg->rxDscpIPv4RemapEn ? TRUE : FALSE;
    cppiP0ControlCfg.p0RxRemapDscpIpv6 = hostPortCfg->rxDscpIPv6RemapEn ? TRUE : FALSE;

    CSL_CPSW_setCppiP0Control(regs, &cppiP0ControlCfg);

    return status;
}

int32_t CpswHostPort_rejoin(EnetMod_Handle hMod,
                            Enet_Type enetType,
                            uint32_t instId)
{
    CpswHostPort_Handle hPort = (CpswHostPort_Handle)hMod;

    /* Save peripheral info to use it later to query SoC parameters */
    hPort->enetType = enetType;
    hPort->instId = instId;

    return ENET_SOK;
}

void CpswHostPort_close(EnetMod_Handle hMod)
{
    CSL_Xge_cpswRegs *regs = (CSL_Xge_cpswRegs *)hMod->virtAddr;

    Enet_devAssert(regs != NULL, "CPSW hostport regs address is not valid\n");

    /* Disable host port */
    CSL_CPSW_disablePort0(regs);
}

void CpswHostPort_saveCtxt(EnetMod_Handle hMod)
{
    CpswHostPort_close(hMod);
}

int32_t CpswHostPort_restoreCtxt(EnetMod_Handle hMod,
                             Enet_Type enetType,
                             uint32_t instId,
                             const void *cfg,
                             uint32_t cfgSize)
{
    CSL_Xge_cpswRegs *regs = (CSL_Xge_cpswRegs *)hMod->virtAddr;
    int32_t status = ENET_SOK;

    /* Open the host port*/
    status = CpswHostPort_open(hMod, enetType, instId, cfg, cfgSize);

    if (status == ENET_SOK)
    {
        /* Enabling host port event */
        CSL_CPSW_enablePort0(regs);
    }

    return status;
}

int32_t CpswHostPort_ioctl(EnetMod_Handle hMod,
                           uint32_t cmd,
                           Enet_IoctlPrms *prms)
{
    CpswHostPort_Handle hPort = (CpswHostPort_Handle)hMod;
    CSL_Xge_cpswRegs *regs = (CSL_Xge_cpswRegs *)hMod->virtAddr;
    int32_t status = ENET_SOK;

#if ENET_CFG_IS_ON(DEV_ERROR)
    /* Validate CPSW host port IOCTL parameters */
    if (ENET_IOCTL_GET_PER(cmd) == ENET_IOCTL_PER_CPSW)
    {
        if (ENET_IOCTL_GET_TYPE(cmd) == ENET_IOCTL_TYPE_PUBLIC)
        {
            status = Enet_validateIoctl(cmd, prms,
                                        gCpswHostPort_ioctlValidate,
                                        ENET_ARRAYSIZE(gCpswHostPort_ioctlValidate));
        }
        else
        {
            status = Enet_validateIoctl(cmd, prms,
                                        gCpswHostPort_privIoctlValidate,
                                        ENET_ARRAYSIZE(gCpswHostPort_privIoctlValidate));
        }

        ENETTRACE_ERR_IF(status != ENET_SOK, "IOCTL 0x%08x params are not valid\n", cmd);
    }
#endif

    if (status == ENET_SOK)
    {
        CpswHostPortIoctlHandler * ioctlHandlerFxn;

        Enet_devAssert(regs != NULL, "CPSW hostport regs address is not valid\n");

        ioctlHandlerFxn = CpswHostPort_getIoctlHandlerFxn(cmd, CpswHostPortIoctlHandlerRegistry, ENET_ARRAYSIZE(CpswHostPortIoctlHandlerRegistry));
        Enet_devAssert(ioctlHandlerFxn != NULL);
        status = ioctlHandlerFxn(hPort, regs, prms);
    }

    return status;
}

#if ENET_CFG_IS_ON(DEV_ERROR)
static int32_t CpswHostPort_isSupported(CSL_Xge_cpswRegs *regs)
{
    CSL_CPSW_VERSION version;
    uint32_t i;
    int32_t status = ENET_ENOTSUPPORTED;

    CSL_CPSW_getCpswVersionInfo(regs, &version);

    for (i = 0U; i < ENET_ARRAYSIZE(CpswHostPort_gSupportedVer); i++)
    {
        if ((version.majorVer == CpswHostPort_gSupportedVer[i].majorVer) &&
            (version.minorVer == CpswHostPort_gSupportedVer[i].minorVer) &&
            (version.rtlVer == CpswHostPort_gSupportedVer[i].rtlVer) &&
            (version.id == CpswHostPort_gSupportedVer[i].id))
        {
            status = ENET_SOK;
            break;
        }
    }

    return status;
}
#endif






static int32_t CpswHostPort_getIoctlHandlerIdx(uint32_t ioctlCmd, CpswHostPortIoctlHandlerRegistry_t *ioctlRegistryTbl, uint32_t tableSize, uint32_t *tblIdx)
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

static CpswHostPortIoctlHandler * CpswHostPort_getIoctlHandlerFxn(uint32_t ioctlCmd, CpswHostPortIoctlHandlerRegistry_t *ioctlRegistryTbl, uint32_t tableSize)
{
    int32_t status;
    uint32_t tblIdx;
    CpswHostPortIoctlHandler *handlerFxn = NULL;

    status = CpswHostPort_getIoctlHandlerIdx(ioctlCmd, ioctlRegistryTbl, tableSize, &tblIdx);
    if (ENET_SOK == status)
    {
        Enet_devAssert(tblIdx < tableSize, "Invalid IOCTL registry table index for IOCTL cmd:%x", ioctlCmd);
        handlerFxn = ioctlRegistryTbl[tblIdx].fxn;
    }
    else
    {
        handlerFxn = &CpswHostPort_ioctl_handler_default;
    }
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to get ioctl handler for ioctl cmd: %x. Using default ioctl handler \r\n", ioctlCmd);
    return handlerFxn;
}

static int32_t CpswHostPort_setIoctlHandlerFxn(uint32_t ioctlCmd,
                                          CpswHostPortIoctlHandler *ioctlHandlerFxn,
                                          CpswHostPortIoctlHandlerRegistry_t *ioctlRegistryTbl,
                                          uint32_t tableSize)
{
    int32_t status;
    uint32_t tblIdx;

    status = CpswHostPort_getIoctlHandlerIdx(ioctlCmd, ioctlRegistryTbl, tableSize, &tblIdx);
    if (ENET_SOK == status)
    {
        Enet_devAssert((tblIdx < tableSize), "Invalid IOCTL registry table index for IOCTL cmd:%x", ioctlCmd);
        Enet_devAssert((ioctlCmd == ioctlRegistryTbl[tblIdx].cmd), "Ioctl table corrupted");
        ioctlRegistryTbl[tblIdx].fxn = ioctlHandlerFxn;
    }
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to set ioctl handler for ioctl cmd: %x \r\n", ioctlCmd);
    return status;
}

static int32_t CpswHostPort_ioctl_handler_CPSW_HOSTPORT_IOCTL_REGISTER_HANDLER(CpswHostPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    const Enet_IoctlRegisterHandlerInArgs *inArgs = (const Enet_IoctlRegisterHandlerInArgs *)prms->inArgs;
    int32_t status;

    status = CpswHostPort_setIoctlHandlerFxn(inArgs->cmd,
                                        (CpswHostPortIoctlHandler *)inArgs->fxn,
                                        CpswHostPortIoctlHandlerRegistry,
                                        ENET_ARRAYSIZE(CpswHostPortIoctlHandlerRegistry));
    return status;
}


static int32_t CpswHostPort_ioctl_handler_default(CpswHostPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    return ENET_ENOTSUPPORTED;
}
