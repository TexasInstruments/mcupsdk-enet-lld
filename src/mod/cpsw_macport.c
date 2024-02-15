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
 * \file  cpsw_macport.c
 *
 * \brief This file contains the implementation of the CPSW MAC port module.
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
#include <include/core/enet_mod_tas.h>
#include <include/mod/cpsw_macport.h>
#include <priv/core/enet_trace_priv.h>
#include <priv/mod/cpsw_macport_priv.h>
#include <priv/mod/cpsw_macport_ioctl_priv.h>
#include <priv/mod/cpsw_clks.h>
#include "cpsw_macport_intervlan.h"
#if ENET_CFG_IS_ON(CPSW_MACPORT_EST)
#include "cpsw_macport_est.h"
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Supported AM64x version */
#define CPSW_MACPORT_VER_REVMAJ_AM64X        (0x00000001U)
#define CPSW_MACPORT_VER_REVMIN_AM64X        (0x00000003U)
#define CPSW_MACPORT_VER_REVRTL_AM64X        (0x00000001U)
#define CPSW_MACPORT_VER_ID_AM64X            (0x00006BA8U)

/* Supported AM273x version */
#define CPSW_MACPORT_VER_REVMAJ_AM273X         (0x00000001U)
#define CPSW_MACPORT_VER_REVMIN_AM273X         (0x00000002U)
#define CPSW_MACPORT_VER_REVRTL_AM273X         (0x00000000U)
#define CPSW_MACPORT_VER_ID_AM273X             (0x00006B90U)

/* Supported AM263x version */
#define CPSW_MACPORT_VER_REVMAJ_AM263X         (0x00000001U)
#define CPSW_MACPORT_VER_REVMIN_AM263X         (0x00000003U)
#define CPSW_MACPORT_VER_REVRTL_AM263X         (0x00000002U)
#define CPSW_MACPORT_VER_ID_AM263X             (0x00006B90U)

/* Supported AWR294x version */
#define CPSW_MACPORT_VER_REVMAJ_AWR294X         (0x00000001U)
#define CPSW_MACPORT_VER_REVMIN_AWR294X         (0x00000002U)
#define CPSW_MACPORT_VER_REVRTL_AWR294X         (0x00000000U)
#define CPSW_MACPORT_VER_ID_AWR294X             (0x00006B90U)

/* Supported AWR2544 version */
#define CPSW_MACPORT_VER_REVMAJ_AWR2544         (0x00000001U)
#define CPSW_MACPORT_VER_REVMIN_AWR2544         (0x00000003U)
#define CPSW_MACPORT_VER_REVRTL_AWR2544         (0x00000003U)
#define CPSW_MACPORT_VER_ID_AWR2544             (0x00006B90U)

/* Supported AM62AX version */
#define CPSW_MACPORT_VER_REVMAJ_AM62AX         (0x00000001U)
#define CPSW_MACPORT_VER_REVMIN_AM62AX         (0x00000003U)
#define CPSW_MACPORT_VER_REVRTL_AM62AX         (0x00000002U)
#define CPSW_MACPORT_VER_ID_AM62AX             (0x00006BA8U)

/*! \brief Default value used for MAC port RX MTU (MRU). */
#define CPSW_MACPORT_RX_MTU_DEFAULT           (1518U)

/*!
 * \brief Priority Escalation value for switch scheduler
 *
 * When a port is in escalate priority, this is the number of higher priority
 * packets sent before the next lower priority is allowed to send a packet.
 * Escalate priority allows lower priority packets to be sent at a fixed rate
 * relative to the next higher priority.  The min value of esc_pri_ld_val = 2.
 */
#define CPSW_MACPORT_ESC_PRI_LD_VAL           (2U)

#define CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT(x)    \
          {.cmd = x,                            \
           .fxn = &CpswMacPort_ioctl_handler_##x}

#define CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(x)    \
          {.cmd = x,                                    \
           .fxn = &CpswMacPort_ioctl_handler_default}

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */
typedef int32_t (CpswMacPortIoctlHandler)(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);

typedef struct CpswMacPortIoctlHandlerRegistry_s
{
    uint32_t cmd;
    CpswMacPortIoctlHandler *fxn;
} CpswMacPortIoctlHandlerRegistry_t;


/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

#if ENET_CFG_IS_ON(DEV_ERROR)
static int32_t CpswMacPort_isSupported(CSL_Xge_cpswRegs *regs);
#endif
static int32_t CpswMacPort_isMiiSupported(EnetMod_Handle hMod,
                                          const EnetMacPort_Interface *mii);


static int32_t CpswMacPort_checkSocCfg(Enet_Type enetType,
                                       uint32_t instId,
                                       Enet_MacPort macPort,
                                       const EnetMacPort_Interface *mii);

static void CpswMacPort_reset(CSL_Xge_cpswRegs *regs,
                              Enet_MacPort macPort);

static void CpswMacPort_setSwitchTxSched(CSL_Xge_cpswRegs *regs,
                                         Enet_MacPort macPort,
                                         EnetPort_EgressPriorityType priority);

static int32_t CpswMacPort_enableLoopback(CSL_Xge_cpswRegs *regs,
                                          Enet_MacPort macPort,
                                          const EnetMacPort_Interface *mii);

static int32_t CpswMacPort_setInterface(CSL_Xge_cpswRegs *regs,
                                        Enet_MacPort macPort,
                                        const EnetMacPort_Interface *mii);

static bool CpswMacPort_isPortEnabled(CSL_Xge_cpswRegs *regs,
                                      Enet_MacPort macPort);

#if ENET_CFG_IS_ON(CPSW_MACPORT_SGMII)
#if ENET_CFG_IS_ON(DEV_ERROR)
static int32_t CpswMacPort_isSgmiiSupported(CSL_CpsgmiiRegs *sgmiiRegs,
                                            Enet_MacPort macPort);
#endif

static void CpswMacPort_resetSgmiiPort(CSL_CpsgmiiRegs *sgmiiRegs,
                                       Enet_MacPort macPort);

static int32_t CpswMacPort_enableSgmiiLoopback(CSL_Xge_cpswRegs *regs,
                                               CSL_CpsgmiiRegs *sgmiiRegs,
                                               Enet_MacPort macPort);

static int32_t CpswMacPort_setSgmiiInterface(CSL_Xge_cpswRegs *regs,
                                             CSL_CpsgmiiRegs *sgmiiRegs,
                                             Enet_MacPort macPort,
                                             EnetMac_SgmiiMode sgmiiMode,
                                             const EnetMacPort_LinkCfg *linkCfg);

static int32_t CpswMacPort_configSgmii(CSL_CpsgmiiRegs *sgmiiRegs,
                                       EnetMac_SgmiiMode sgmiiMode,
                                       Enet_MacPort macPort,
                                       const EnetMacPort_LinkCfg *linkCfg);

static void CpswMacPort_mapSgmiiLinkCfg(CSL_SGMII_ADVABILITY *sgmiiAdvAbility,
                                        const EnetMacPort_LinkCfg *linkCfg);

static bool CpswMacPort_getSgmiiStatus(CSL_CpsgmiiRegs *sgmiiRegs,
                                       Enet_MacPort macPort);

static int32_t CpswMacPort_checkSgmiiAutoNegStatus(CSL_CpsgmiiRegs *sgmiiRegs,
                                                   Enet_MacPort macPort);

static int32_t CpswMacPort_enableSgmiiPort(CSL_Xge_cpswRegs *regs,
                                           CSL_CpsgmiiRegs *sgmiiRegs,
                                           Enet_MacPort macPort,
                                           const EnetMacPort_Interface *mii,
                                           const EnetMacPort_LinkCfg *linkCfg);
#endif

static int32_t CpswMacPort_ioctl_handler_default(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
static int32_t CpswMacPort_ioctl_handler_CPSW_MACPORT_IOCTL_REGISTER_HANDLER(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
static int32_t CpswMacPort_setIoctlHandlerFxn(uint32_t ioctlCmd,
                                          CpswMacPortIoctlHandler *ioctlHandlerFxn,
                                          CpswMacPortIoctlHandlerRegistry_t *ioctlRegistryTbl,
                                          uint32_t tableSize);
static CpswMacPortIoctlHandler * CpswMacPort_getIoctlHandlerFxn(uint32_t ioctlCmd, CpswMacPortIoctlHandlerRegistry_t *ioctlRegistryTbl, uint32_t tableSize);
static int32_t CpswMacPort_getIoctlHandlerIdx(uint32_t ioctlCmd, CpswMacPortIoctlHandlerRegistry_t *ioctlRegistryTbl, uint32_t tableSize, uint32_t *tblIdx);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

#if ENET_CFG_IS_ON(DEV_ERROR)
/*! \brief CPSW MAC port versions supported by this driver. */
static CSL_CPSW_VERSION CpswMacPort_gSupportedVer[] =
{
    {   /* AM64x */
        .majorVer = CPSW_MACPORT_VER_REVMAJ_AM64X,
        .minorVer = CPSW_MACPORT_VER_REVMIN_AM64X,
        .rtlVer   = CPSW_MACPORT_VER_REVRTL_AM64X,
        .id       = CPSW_MACPORT_VER_ID_AM64X,
    },
    {   /* AM273X */
        .majorVer = CPSW_MACPORT_VER_REVMAJ_AM273X,
        .minorVer = CPSW_MACPORT_VER_REVMIN_AM273X,
        .rtlVer   = CPSW_MACPORT_VER_REVRTL_AM273X,
        .id       = CPSW_MACPORT_VER_ID_AM273X,
    },
	{   /* AM263X */
        .majorVer = CPSW_MACPORT_VER_REVMAJ_AM263X,
        .minorVer = CPSW_MACPORT_VER_REVMIN_AM263X,
        .rtlVer   = CPSW_MACPORT_VER_REVRTL_AM263X,
        .id       = CPSW_MACPORT_VER_ID_AM263X,
    },
    {   /* AWR294X */
        .majorVer = CPSW_MACPORT_VER_REVMAJ_AWR294X,
        .minorVer = CPSW_MACPORT_VER_REVMIN_AWR294X,
        .rtlVer   = CPSW_MACPORT_VER_REVRTL_AWR294X,
        .id       = CPSW_MACPORT_VER_ID_AWR294X,
    },
    {   /* AWR2544 */
        .majorVer = CPSW_MACPORT_VER_REVMAJ_AWR2544,
        .minorVer = CPSW_MACPORT_VER_REVMIN_AWR2544,
        .rtlVer   = CPSW_MACPORT_VER_REVRTL_AWR2544,
        .id       = CPSW_MACPORT_VER_ID_AWR2544,
    },
    {   /* AM62AX */
        .majorVer = CPSW_MACPORT_VER_REVMAJ_AM62AX,
        .minorVer = CPSW_MACPORT_VER_REVMIN_AM62AX,
        .rtlVer   = CPSW_MACPORT_VER_REVRTL_AM62AX,
        .id       = CPSW_MACPORT_VER_ID_AM62AX,
    },
};

/* Public MAC port IOCTL validation data. */
static Enet_IoctlValidate gCpswMacPort_ioctlValidate[] =
{
    ENET_IOCTL_VALID_PRMS(CPSW_MACPORT_IOCTL_GET_FIFO_STATS,
                          sizeof(EnetMacPort_GenericInArgs),
                          sizeof(CpswMacPort_FifoStats)),

    ENET_IOCTL_VALID_PRMS(CPSW_MACPORT_IOCTL_ENABLE_CPTS_EVENT,
                          sizeof(CpswMacPort_EnableTsEventInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(CPSW_MACPORT_IOCTL_DISABLE_CPTS_EVENT,
                          sizeof(EnetMacPort_GenericInArgs),
                          0U),
    ENET_IOCTL_VALID_PRMS(CPSW_MACPORT_IOCTL_EST_ENABLE_TIMESTAMP,
                          sizeof(CpswMacPort_EstTimestampCfg),
                           0U),

    ENET_IOCTL_VALID_PRMS(CPSW_MACPORT_IOCTL_EST_DISABLE_TIMESTAMP,
                          sizeof(EnetMacPort_GenericInArgs),
                          0U),
};

/* Private MAC port IOCTL validation data. */
static Enet_IoctlValidate gCpswMacPort_privIoctlValidate[] =
{
    ENET_IOCTL_VALID_PRMS(CPSW_MACPORT_IOCTL_ENABLE,
                          sizeof(EnetMacPort_LinkCfg),
                          0U),

    ENET_IOCTL_VALID_PRMS(CPSW_MACPORT_IOCTL_DISABLE,
                          0U,
                          0U),

    ENET_IOCTL_VALID_PRMS(CPSW_MACPORT_IOCTL_SET_INTERVLAN_ROUTE,
                          sizeof(CpswMacPort_InterVlanRoutingCfg),
                          sizeof(CpswMacPort_InterVlanRouteId)),

    ENET_IOCTL_VALID_PRMS(CPSW_MACPORT_IOCTL_SET_SPECIFIC_INTERVLAN_ROUTE,
                          sizeof(CpswMacPort_SetSpecificInterVlanRouteInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(CPSW_MACPORT_IOCTL_GET_INTERVLAN_ROUTE,
                          sizeof(CpswMacPort_InterVlanRouteId),
                          sizeof(CpswMacPort_InterVlanRoutingCfg)),

    ENET_IOCTL_VALID_PRMS(CPSW_MACPORT_IOCTL_DELETE_INTERVLAN_ROUTE,
                          sizeof(CpswMacPort_InterVlanRoutingCfg),
                          sizeof(CpswMacPort_InterVlanRouteId)),

    ENET_IOCTL_VALID_PRMS(CPSW_MACPORT_IOCTL_GET_INTERVLAN_FREEROUTES,
                          0U,
                          sizeof(CpswMacPort_InterVlanFreeRouteInfo)),

    ENET_IOCTL_VALID_PRMS(CPSW_MACPORT_IOCTL_FIND_INTERVLAN_ROUTE,
                          sizeof(CpswMacPort_InterVlanRoutingCfg),
                          sizeof(CpswMacPort_InterVlanRouteId)),

    ENET_IOCTL_VALID_PRMS(CPSW_MACPORT_IOCTL_IS_INTERVLAN_ROUTE_FREE,
                          sizeof(CpswMacPort_InterVlanRouteId),
                          sizeof(bool)),

    ENET_IOCTL_VALID_PRMS(CPSW_MACPORT_IOCTL_SET_SHORT_IPG,
                          sizeof(CpswMacPort_PortTxShortIpgCfg),
                          0U),

    ENET_IOCTL_VALID_PRMS(CPSW_MACPORT_IOCTL_GET_SHORT_IPG,
                          sizeof(EnetMacPort_GenericInArgs),
                          sizeof(CpswMacPort_TxShortIpgCfg)),

    ENET_IOCTL_VALID_PRMS(CPSW_MACPORT_IOCTL_GET_SGMII_AUTONEG_LINK_STATUS,
                          sizeof(EnetMacPort_GenericInArgs),
                          sizeof(bool)),

    ENET_IOCTL_VALID_PRMS(CPSW_MACPORT_IOCTL_GET_SGMII_LINK_STATUS,
                          sizeof(EnetMacPort_GenericInArgs),
                          sizeof(bool)),

    ENET_IOCTL_VALID_PRMS(CPSW_MACPORT_IOCTL_REGISTER_HANDLER,
                          sizeof(Enet_IoctlRegisterHandlerInArgs),
                          0U),
};
#endif

static CpswMacPortIoctlHandlerRegistry_t CpswMacPortIoctlHandlerRegistry[] =
{
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MACPORT_IOCTL_GET_VERSION),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MACPORT_IOCTL_PRINT_REGS),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MACPORT_IOCTL_SET_INGRESS_DSCP_PRI_MAP),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MACPORT_IOCTL_GET_INGRESS_DSCP_PRI_MAP),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MACPORT_IOCTL_SET_PRI_REGEN_MAP),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MACPORT_IOCTL_GET_PRI_REGEN_MAP),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MACPORT_IOCTL_SET_EGRESS_QOS_PRI_MAP),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MACPORT_IOCTL_GET_EGRESS_QOS_PRI_MAP),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MACPORT_IOCTL_ENABLE_EGRESS_TRAFFIC_SHAPING),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MACPORT_IOCTL_DISABLE_EGRESS_TRAFFIC_SHAPING),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MACPORT_IOCTL_GET_EGRESS_TRAFFIC_SHAPING),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MACPORT_IOCTL_SET_CREDIT_BASED_SHAPING),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MACPORT_IOCTL_GET_CREDIT_BASED_SHAPING),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MACPORT_IOCTL_GET_MAXLEN),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MACPORT_IOCTL_GET_LINK_CFG),
#if ENET_CFG_IS_ON(CPSW_CUTTHRU)
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MACPORT_IOCTL_SET_CUT_THRU_PARAMS),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MACPORT_IOCTL_GET_CUT_THRU_PARAMS),
#endif
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_MACPORT_IOCTL_GET_FIFO_STATS),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_MACPORT_IOCTL_ENABLE_CPTS_EVENT),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_MACPORT_IOCTL_DISABLE_CPTS_EVENT),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_MACPORT_IOCTL_ENABLE),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_MACPORT_IOCTL_DISABLE),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_MACPORT_IOCTL_SET_INTERVLAN_ROUTE),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_MACPORT_IOCTL_SET_SPECIFIC_INTERVLAN_ROUTE),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_MACPORT_IOCTL_GET_INTERVLAN_ROUTE),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_MACPORT_IOCTL_DELETE_INTERVLAN_ROUTE),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_MACPORT_IOCTL_GET_INTERVLAN_FREEROUTES),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_MACPORT_IOCTL_FIND_INTERVLAN_ROUTE),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_MACPORT_IOCTL_IS_INTERVLAN_ROUTE_FREE),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_MACPORT_IOCTL_SET_SHORT_IPG),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_MACPORT_IOCTL_GET_SHORT_IPG),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_MACPORT_IOCTL_GET_SGMII_AUTONEG_LINK_STATUS),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_MACPORT_IOCTL_GET_SGMII_LINK_STATUS),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_TAS_IOCTL_SET_ADMIN_LIST),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_TAS_IOCTL_GET_OPER_LIST_STATUS),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_TAS_IOCTL_SET_STATE),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_TAS_IOCTL_GET_STATE),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_TAS_IOCTL_GET_ADMIN_LIST),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_TAS_IOCTL_GET_OPER_LIST),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_TAS_IOCTL_CONFIG_CHANGE_STATUS_PARAMS),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_MACPORT_IOCTL_EST_ENABLE_TIMESTAMP),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_MACPORT_IOCTL_EST_DISABLE_TIMESTAMP),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MACPORT_IOCTL_IET_RELEASE_PREEMPT_TRAFFIC),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MACPORT_IOCTL_IET_HOLD_PREEMPT_TRAFFIC),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MACPORT_IOCTL_GET_QUEUE_PREEMPT_STATUS),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MACPORT_IOCTL_SET_PREEMPT_QUEUE),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MACPORT_IOCTL_GET_PREEMPT_MIN_FRAG_SIZE),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MACPORT_IOCTL_SET_PREEMPT_MIN_FRAG_SIZE),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MACPORT_IOCTL_GET_PREEMPT_VERIFY_STATUS),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MACPORT_IOCTL_DISABLE_PREEMPT_VERIFICATION),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MACPORT_IOCTL_ENABLE_PREEMPT_VERIFICATION),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MACPORT_IOCTL_GET_PREEMPTION_ACTIVE_STATUS),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MACPORT_IOCTL_GET_PREEMPTION_ENABLE_STATUS),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MACPORT_IOCTL_DISABLE_PREEMPTION),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MACPORT_IOCTL_ENABLE_PREEMPTION),
    CPSW_MACPORT_IOCTL_HANDLER_ENTRY_INIT(CPSW_MACPORT_IOCTL_REGISTER_HANDLER),
};

#if ENET_CFG_IS_ON(CPSW_MACPORT_SGMII)
#if (ENET_CFG_TRACE_LEVEL >= ENET_CFG_TRACE_LEVEL_INFO)
static const char *CpswMacPort_gSgmiiSpeedNames[] =
{
    [CSL_SGMII_10_MBPS]   = "10-Mbps",
    [CSL_SGMII_100_MBPS]  = "100-Mbps",
    [CSL_SGMII_1000_MBPS] = "1-Gbps",
};

static const char *CpswMacPort_gSgmiiDuplexNames[] =
{
    [CSL_SGMII_HALF_DUPLEX] = "Half-Duplex",
    [CSL_SGMII_FULL_DUPLEX] = "Full-Duplex",
};
#endif
#endif

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void CpswMacPort_initCfg(CpswMacPort_Cfg *macPortCfg)
{
    macPortCfg->loopbackEn        = false;
    macPortCfg->crcType           = ENET_CRC_ETHERNET;
    macPortCfg->rxMtu             = CPSW_MACPORT_RX_MTU_DEFAULT;
    macPortCfg->passPriorityTaggedUnchanged = false;
    macPortCfg->vlanCfg.portPri   = 0U;
    macPortCfg->vlanCfg.portCfi   = 0U;
    macPortCfg->vlanCfg.portVID   = 0U;
    macPortCfg->txPriorityType    = ENET_EGRESS_PRI_TYPE_FIXED;
    macPortCfg->sgmiiMode         = ENET_MAC_SGMIIMODE_INVALID;
}

int32_t CpswMacPort_open(EnetMod_Handle hMod,
                         Enet_Type enetType,
                         uint32_t instId,
                         const void *cfg,
                         uint32_t cfgSize)
{
    CpswMacPort_Handle hPort = (CpswMacPort_Handle)hMod;
    const CpswMacPort_ModCfg *macModCfg = (const CpswMacPort_ModCfg *)cfg;
    const CpswMacPort_Cfg *macCfg = &macModCfg->macCfg;
    const EnetMacPort_Interface *mii = &macModCfg->mii;
    CSL_Xge_cpswRegs *regs = (CSL_Xge_cpswRegs *)hMod->virtAddr;
#if ENET_CFG_IS_ON(CPSW_MACPORT_SGMII)
    CSL_CpsgmiiRegs *sgmiiRegs = (CSL_CpsgmiiRegs *)hMod->virtAddr2;
#endif
    Enet_MacPort macPort = hPort->macPort;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    uint32_t portId = ENET_MACPORT_ID(macPort);
    int32_t status = ENET_SOK;

    /* Saving macMode Cfg */
    hPort->macModCfgCtxt = *((CpswMacPort_ModCfg *)(macModCfg));
    ENETTRACE_VAR(portId);
    Enet_devAssert(cfgSize == sizeof(CpswMacPort_ModCfg),
                   "Invalid MAC port config params size %u (expected %u)\n",
                   cfgSize, sizeof(CpswMacPort_ModCfg));

    Enet_devAssert(regs != NULL, "MAC %u: regs address is not valid\n", portId);

#if ENET_CFG_IS_ON(CPSW_MACPORT_SGMII)
    Enet_devAssert(sgmiiRegs != NULL, "MAC %u: SGMII regs address is not valid\n", portId);
#endif

    /* Check supported MAC port module versions */
#if ENET_CFG_IS_ON(DEV_ERROR)
    status = CpswMacPort_isSupported(regs);
    Enet_devAssert(status == ENET_SOK, "MAC %u: version is not supported\n", portId);

#if ENET_CFG_IS_ON(CPSW_MACPORT_SGMII)
    if ((status == ENET_SOK) && (ENET_FEAT_IS_EN(hMod->features, CPSW_MACPORT_FEATURE_SGMII)))
    {
        status = CpswMacPort_isSgmiiSupported(sgmiiRegs, macPort);
        Enet_devAssert(status == ENET_SOK, "MAC %u: SGMII version is not supported\n", portId);
    }
#endif /*#if ENET_CFG_IS_ON(CPSW_MACPORT_SGMII) */

#endif /*#if ENET_CFG_IS_ON(DEV_ERROR) */

    /* Check if MII is supported */
    status = CpswMacPort_isMiiSupported(hMod, mii);
    ENETTRACE_ERR_IF(status != ENET_SOK,  "MAC %u: MII not supported\n", portId);

    /* Save peripheral info to use it later to query SoC parameters */
    hPort->enetType = enetType;
    hPort->instId = instId;
    hPort->enabled = true;

    if (status == ENET_SOK)
    {
        /* Check if SoC settings (if any) matches the requested MII config */
        status = CpswMacPort_checkSocCfg(enetType, instId, macPort, mii);
        ENETTRACE_ERR_IF(status != ENET_SOK, "MAC %u: MII mismatch with SoC settings\n", portId);
    }

    /* Soft-reset the Ethernet MAC logic and SGMII port */
    if (status == ENET_SOK)
    {
        CpswMacPort_reset(regs, macPort);

#if ENET_CFG_IS_ON(CPSW_MACPORT_SGMII)
        if (ENET_FEAT_IS_EN(hMod->features, CPSW_MACPORT_FEATURE_SGMII))
        {
            if (EnetMacPort_isSgmii(mii) ||
                EnetMacPort_isQsgmii(mii))
            {
                CpswMacPort_resetSgmiiPort(sgmiiRegs, macPort);
            }
        }
#endif
    }

    /* Set CRC, MRU and port VLAN config */
    if (status == ENET_SOK)
    {
        if (macCfg->crcType == ENET_CRC_ETHERNET)
        {
            CSL_CPGMAC_SL_disableCastagnoliCRC(regs, portNum);
        }
        else
        {
            CSL_CPGMAC_SL_enableCastagnoliCRC(regs, portNum);
        }

        CSL_CPGMAC_SL_setRxMaxLen(regs, portNum, macCfg->rxMtu);

        if (macCfg->passPriorityTaggedUnchanged)
        {
            CSL_CPSW_enablePortPassPriTag(regs, portNum + 1U);
        }
        else
        {
            CSL_CPSW_disablePortPassPriTag(regs, portNum + 1U);
        }

        CSL_CPSW_setPortVlanReg(regs, portNum + 1U,
                                macCfg->vlanCfg.portVID,
                                macCfg->vlanCfg.portCfi,
                                macCfg->vlanCfg.portPri);

        CpswMacPort_setSwitchTxSched(regs, macPort, macCfg->txPriorityType);
    }

    /* Set normal mode or loopback mode */
    if (status == ENET_SOK)
    {
        if (macCfg->loopbackEn)
        {
            if (EnetMacPort_isSgmii(mii))
            {
#if ENET_CFG_IS_ON(CPSW_MACPORT_SGMII)
                status = CpswMacPort_enableSgmiiLoopback(regs, sgmiiRegs, macPort);
#else
                status = ENET_ENOTSUPPORTED;
#endif
                ENETTRACE_ERR_IF(status != ENET_SOK,
                                 "MAC %u: failed to set SGMII loopback mode: %d\n", portId, status);
            }
            else
            {
                status = CpswMacPort_enableLoopback(regs, macPort, mii);
                ENETTRACE_ERR_IF(status != ENET_SOK,
                                 "MAC %u: failed to set loopback mode: %d\n", portId, status);
            }
        }
        else
        {
            CSL_CPGMAC_SL_disableLoopback(regs, portNum);
        }
    }

    /* Configure MII interface (except for SGMII loopback mode) */
    if (status == ENET_SOK)
    {
        if (EnetMacPort_isSgmii(mii) ||
            EnetMacPort_isQsgmii(mii))
        {
#if ENET_CFG_IS_ON(CPSW_MACPORT_SGMII)
            /* SGMII loopback is digital loopback (before the SERDES) from the CPSGMII transmit
             * to the CPSGMII receive. The SGMII modes are complementary to loopback so
             * configuring MAC interface in loopback mode will cause conflicting configuration */
            if (!macCfg->loopbackEn)
            {
                status = CpswMacPort_setSgmiiInterface(regs, sgmiiRegs,
                                                       macPort,
                                                       macCfg->sgmiiMode,
                                                       &macModCfg->linkCfg);
            }
#else
            status = ENET_ENOTSUPPORTED;
#endif
            ENETTRACE_ERR_IF(status != ENET_SOK,
                             "MAC %u: failed to set Q/SGMII interface: %d\n", portId, status);
        }
        else
        {
            status = CpswMacPort_setInterface(regs, macPort, mii);
            ENETTRACE_ERR_IF(status != ENET_SOK,
                             "MAC %u: failed to set interface: %d\n", portId, status);
        }
    }

#if ENET_CFG_IS_ON(CPSW_MACPORT_INTERVLAN)
    /* Open InterVLAN (clear VLAN routes) */
    if (status == ENET_SOK)
    {
        if (ENET_FEAT_IS_EN(hMod->features, CPSW_MACPORT_FEATURE_INTERVLAN))
        {
            CpswMacPort_openInterVlan(hMod);
        }
    }
#endif

#if ENET_CFG_IS_ON(CPSW_MACPORT_EST)
    if (status == ENET_SOK)
    {
        if (ENET_FEAT_IS_EN(hMod->features, CPSW_MACPORT_FEATURE_EST))
        {
            CpswMacPort_openEst(hMod);
        }
    }
#endif

    return status;
}

int32_t CpswMacPort_rejoin(EnetMod_Handle hMod,
                           Enet_Type enetType,
                           uint32_t instId)
{
    CpswMacPort_Handle hPort = (CpswMacPort_Handle)hMod;

    /* Save peripheral info to use it later to query SoC parameters */
    hPort->enetType = enetType;
    hPort->instId = instId;

    return ENET_SOK;
}

void CpswMacPort_close(EnetMod_Handle hMod)
{
    CpswMacPort_Handle hPort = (CpswMacPort_Handle)hMod;
    CSL_Xge_cpswRegs *regs = (CSL_Xge_cpswRegs *)hMod->virtAddr;
    bool enabled;

    enabled = CpswMacPort_isPortEnabled(regs, hPort->macPort);
    if (enabled)
    {
        CpswMacPort_disablePort(regs, hPort->macPort);
    }
}

void CpswMacPort_saveCtxt(EnetMod_Handle hMod)
{
    CpswMacPort_close(hMod);
}

int32_t CpswMacPort_restoreCtxt(EnetMod_Handle hMod, Enet_Type enetType,
                             uint32_t instId, const void *cfg, uint32_t cfgSize)
{
    int32_t status = ENET_SOK;
    CpswMacPort_Handle hPort = (CpswMacPort_Handle)hMod;

    status = CpswMacPort_open(hMod, enetType, instId, &hPort->macModCfgCtxt, sizeof(hPort->macModCfgCtxt));
    return status;
}

int32_t CpswMacPort_ioctl(EnetMod_Handle hMod,
                          uint32_t cmd,
                          Enet_IoctlPrms *prms)
{
    CpswMacPort_Handle hPort = (CpswMacPort_Handle)hMod;
    CSL_Xge_cpswRegs *regs = (CSL_Xge_cpswRegs *)hMod->virtAddr;
#if ENET_CFG_IS_ON(CPSW_MACPORT_SGMII)
    CSL_CpsgmiiRegs *sgmiiRegs = (CSL_CpsgmiiRegs *)hMod->virtAddr2;
#endif
    Enet_MacPort macPort = hPort->macPort;
    uint32_t portId = ENET_MACPORT_ID(macPort);
    int32_t status = ENET_SOK;

    ENETTRACE_VAR(portId);
#if ENET_CFG_IS_ON(DEV_ERROR)
    /* Validate CPSW MAC port IOCTL parameters */
    if (ENET_IOCTL_GET_PER(cmd) == ENET_IOCTL_PER_CPSW)
    {
        if (ENET_IOCTL_GET_TYPE(cmd) == ENET_IOCTL_TYPE_PUBLIC)
        {
            status = Enet_validateIoctl(cmd, prms,
                                        gCpswMacPort_ioctlValidate,
                                        ENET_ARRAYSIZE(gCpswMacPort_ioctlValidate));
        }
        else
        {
            status = Enet_validateIoctl(cmd, prms,
                                        gCpswMacPort_privIoctlValidate,
                                        ENET_ARRAYSIZE(gCpswMacPort_privIoctlValidate));
        }

        ENETTRACE_ERR_IF(status != ENET_SOK, "MAC %u: IOCTL 0x%08x params are not valid\n", portId, cmd);
    }
#endif

    if (status == ENET_SOK)
    {
        CpswMacPortIoctlHandler * ioctlHandlerFxn;

        ioctlHandlerFxn = CpswMacPort_getIoctlHandlerFxn(cmd, CpswMacPortIoctlHandlerRegistry, ENET_ARRAYSIZE(CpswMacPortIoctlHandlerRegistry));
        Enet_devAssert(ioctlHandlerFxn != NULL);
        status = ioctlHandlerFxn(hPort, regs,prms);
    }

    return status;
}

#if ENET_CFG_IS_ON(DEV_ERROR)
static int32_t CpswMacPort_isSupported(CSL_Xge_cpswRegs *regs)
{
    CSL_CPSW_VERSION version;
    uint32_t i;
    int32_t status = ENET_ENOTSUPPORTED;

    CSL_CPSW_getCpswVersionInfo(regs, &version);

    for (i = 0U; i < ENET_ARRAYSIZE(CpswMacPort_gSupportedVer); i++)
    {
        if ((version.majorVer == CpswMacPort_gSupportedVer[i].majorVer) &&
            (version.minorVer == CpswMacPort_gSupportedVer[i].minorVer) &&
            (version.rtlVer == CpswMacPort_gSupportedVer[i].rtlVer) &&
            (version.id == CpswMacPort_gSupportedVer[i].id))
        {
            status = ENET_SOK;
            break;
        }
    }

    return status;
}
#endif
static int32_t CpswMacPort_isMiiSupported(EnetMod_Handle hMod,
                                          const EnetMacPort_Interface *mii)
{
    int32_t status = ENET_ENOTSUPPORTED;

    if (EnetMacPort_isMii(mii))
   {
#if ENET_CFG_IS_ON(CPSW_MACPORT_MII)
        status = ENET_SOK;
#endif
   }
    else if (EnetMacPort_isRmii(mii))
    {
        status = ENET_SOK;
    }
    else if (EnetMacPort_isRgmii(mii))
    {
        status = ENET_SOK;
    }
    else if (EnetMacPort_isSgmii(mii) ||
             EnetMacPort_isQsgmii(mii))
    {
#if ENET_CFG_IS_ON(CPSW_MACPORT_SGMII)
        if (ENET_FEAT_IS_EN(hMod->features, CPSW_MACPORT_FEATURE_SGMII))
        {
            status = ENET_SOK;
        }
#endif
    }
    else
    {
        status = ENET_ENOTSUPPORTED;
    }

    return status;
}


static int32_t CpswMacPort_checkSocCfg(Enet_Type enetType,
                                       uint32_t instId,
                                       Enet_MacPort macPort,
                                       const EnetMacPort_Interface *mii)
{
    EnetMacPort_Interface miiSoc;
    int32_t status;

    status = EnetSoc_getMacPortMii(enetType, instId, macPort, &miiSoc);
    if (status == ENET_SOK)
    {
        if ((miiSoc.layerType != mii->layerType) ||
            (miiSoc.sublayerType != mii->sublayerType))
        {
            status = ENET_EINVALIDPARAMS;
        }
    }
    else
    {
        /* SoC layer reports ENET_ENOTSUPPORTED when there isn't any SoC layer
         * settings related to the MAC port configuration.  So we don't treat
         * that as an error */
        if (status == ENET_ENOTSUPPORTED)
        {
            status = ENET_SOK;
        }
    }

    return status;
}

static void CpswMacPort_reset(CSL_Xge_cpswRegs *regs,
                              Enet_MacPort macPort)
{
    CSL_CPGMAC_SL_MACSTATUS macStatus = {0};
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    uint32_t gmiiEn = 0;
    uint32_t done;

   gmiiEn = CSL_CPGMAC_SL_isGMIIEnabled(regs, portNum);

   /* Idle MAC port */
    CSL_CPGMAC_SL_enableIdleMode(regs, portNum);
    do
    {
        CSL_CPGMAC_SL_getMacStatusReg(regs, portNum, &macStatus);
        done = (macStatus.idle == 1U);

        /* TX idle is set only if GMII clock is enabled */
        if (gmiiEn == 1U)
        {
            done = done && (macStatus.macTxIdle == 1U);
        }
    }
    while (done == FALSE);

    /* Soft-reset the Ethernet MAC logic */
    CSL_CPGMAC_SL_resetMac(regs, portNum);
    do
    {
        done = CSL_CPGMAC_SL_isMACResetDone(regs, portNum);
    }
    while (done == FALSE);
}

static void CpswMacPort_setSwitchTxSched(CSL_Xge_cpswRegs *regs,
                                         Enet_MacPort macPort,
                                         EnetPort_EgressPriorityType priority)
{
    CSL_CPSW_PTYPE pType;
    uint32_t escEn;
    uint32_t escPriLoadVal;

    if (priority == ENET_EGRESS_PRI_TYPE_FIXED)
    {
        escEn = FALSE;
        escPriLoadVal = CPSW_MACPORT_ESC_PRI_LD_VAL;
    }
    else
    {
        escEn = TRUE;
        escPriLoadVal = 0U;
    }

    CSL_CPSW_getPTypeReg(regs, &pType);

    switch (macPort)
    {
        case ENET_MAC_PORT_1:
            pType.port1PriorityTypeEscalateEnable = escEn;
            pType.escPriLoadVal = escPriLoadVal;
            break;

        case ENET_MAC_PORT_2:
            pType.port2PriorityTypeEscalateEnable = escEn;
            pType.escPriLoadVal = escPriLoadVal;
            break;

        case ENET_MAC_PORT_3:
            pType.port3PriorityTypeEscalateEnable = escEn;
            pType.escPriLoadVal = escPriLoadVal;
            break;

        case ENET_MAC_PORT_4:
            pType.port4PriorityTypeEscalateEnable = escEn;
            pType.escPriLoadVal = escPriLoadVal;
            break;

        case ENET_MAC_PORT_5:
            pType.port5PriorityTypeEscalateEnable = escEn;
            pType.escPriLoadVal = escPriLoadVal;
            break;

        case ENET_MAC_PORT_6:
            pType.port6PriorityTypeEscalateEnable = escEn;
            pType.escPriLoadVal = escPriLoadVal;
            break;

        case ENET_MAC_PORT_7:
            pType.port7PriorityTypeEscalateEnable = escEn;
            pType.escPriLoadVal = escPriLoadVal;
            break;

        case ENET_MAC_PORT_8:
            pType.port8PriorityTypeEscalateEnable = escEn;
            pType.escPriLoadVal = escPriLoadVal;
            break;

        default:
            Enet_devAssert(false, "MAC %u: Invalid MAC port\n", ENET_MACPORT_ID(macPort));
            break;
    }

    CSL_CPSW_setPTypeReg(regs, &pType);
}

static int32_t CpswMacPort_enableLoopback(CSL_Xge_cpswRegs *regs,
                                          Enet_MacPort macPort,
                                          const EnetMacPort_Interface *mii)
{
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    uint32_t portId = ENET_MACPORT_ID(macPort);
    int32_t status = ENET_SOK;

    ENETTRACE_VAR(portId);
    if (EnetMacPort_isRmii(mii) ||
        EnetMacPort_isRgmii(mii))
    {
        CSL_CPGMAC_SL_enableLoopback(regs, portNum);
    }
    else
    {
        ENETTRACE_ERR("MAC %u: Loopback is not supported in MII mode %u-%u\n",
                      portId, mii->layerType, mii->sublayerType);
        status = ENET_ENOTSUPPORTED;
    }

    return status;
}

static int32_t CpswMacPort_setInterface(CSL_Xge_cpswRegs *regs,
                                        Enet_MacPort macPort,
                                        const EnetMacPort_Interface *mii)
{
    uint32_t macControl;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    uint32_t portId = ENET_MACPORT_ID(macPort);
    int32_t status = ENET_EINVALIDPARAMS;

    ENETTRACE_VAR(portId);
    macControl = CSL_CPGMAC_SL_getMacControlReg(regs, portNum);

    /* Clear fields not supported by hardware */
    CSL_FINS(macControl, XGE_CPSW_PN_MAC_CONTROL_REG_EXT_RX_FLOW_EN, 0U);
    CSL_FINS(macControl, XGE_CPSW_PN_MAC_CONTROL_REG_EXT_TX_FLOW_EN, 0U);

    /* Set MAC_CONTROL register settings according to the MII type */
    if (EnetMacPort_isRmii(mii))
    {
        status = ENET_SOK;
    }
    else if (EnetMacPort_isMii(mii))
    {
#if ENET_CFG_IS_ON(CPSW_MACPORT_MII)
        status = ENET_SOK;
#else
        status = ENET_ENOTSUPPORTED;
#endif
    }
    else if (EnetMacPort_isRgmii(mii))
    {
        CSL_FINS(macControl, XGE_CPSW_PN_MAC_CONTROL_REG_EXT_EN, 0U);
        CSL_FINS(macControl, XGE_CPSW_PN_MAC_CONTROL_REG_GIG_FORCE, 1U);
        status = ENET_SOK;
    }
#if ENET_CFG_IS_ON(CPSW_XGMII)
    else if (EnetMacPort_isXfi(mii))
    {
        CSL_FINS(macControl, XGE_CPSW_PN_MAC_CONTROL_REG_EXT_EN_XGIG, 0U);
        status = ENET_SOK;
    }
#endif
    else
    {
        status = ENET_EINVALIDPARAMS;
    }

    /* Set interface layer and sublayer */
    if (status == ENET_SOK)
    {
        CSL_CPGMAC_SL_setMacControlReg(regs, portNum, macControl);
    }

    ENETTRACE_ERR_IF(status != ENET_SOK,
                     "MAC %u: failed to set interface layer %u sublayer %u: %d\n",
                     portId, mii->layerType, mii->sublayerType, status);

    return status;
}

static bool CpswMacPort_isPortEnabled(CSL_Xge_cpswRegs *regs,
                                      Enet_MacPort macPort)
{
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    uint32_t macControl;
    uint32_t gmiiEn;
    uint32_t xgmiiEn = 0U;

    macControl = CSL_CPGMAC_SL_getMacControlReg(regs, portNum);
    gmiiEn = CSL_FEXT(macControl, XGE_CPSW_PN_MAC_CONTROL_REG_GMII_EN);
#if ENET_CFG_IS_ON(CPSW_XGMII)
    xgmiiEn = CSL_FEXT(macControl, XGE_CPSW_PN_MAC_CONTROL_REG_XGMII_EN);
#endif

    return((gmiiEn == 1U) || (xgmiiEn == 1U));
}


#if ENET_CFG_IS_ON(CPSW_MACPORT_SGMII)
#if ENET_CFG_IS_ON(DEV_ERROR)
static int32_t CpswMacPort_isSgmiiSupported(CSL_CpsgmiiRegs *sgmiiRegs,
                                            Enet_MacPort macPort)
{
    CSL_SGMII_VERSION version;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    uint32_t portId = ENET_MACPORT_ID(macPort);
    int32_t status = ENET_ENOTSUPPORTED;

    ENETTRACE_VAR(portId);
    CSL_SGMII_getVersionInfo(sgmiiRegs, portNum, &version);

    if ((version.ident_val == CPSW_MACPORT_SGMII_VER_TX_ID_J7X) &&
        (version.major_version == CPSW_MACPORT_SGMII_VER_REVMAJ_J7X) &&
        (version.minor_version == CPSW_MACPORT_SGMII_VER_REVMIN_J7X) &&
        (version.rtl_version == CPSW_MACPORT_SGMII_VER_REVRTL))
    {
        status = ENET_SOK;
    }

    /* The SGMII registers will be reset to zero if the SERDES clock is not initialized
     * i.e. the SERDES must be configured for the SGMII module to be configured */
    if (version.major_version == 0U)
    {
        ENETTRACE_ERR("MAC %u: SGMII port not ready, SERDES PLL not locked\n", portId);
        Enet_devAssert(false);
        status = ENET_EUNEXPECTED;
    }

    return status;
}
#endif

static void CpswMacPort_resetSgmiiPort(CSL_CpsgmiiRegs *sgmiiRegs,
                                       Enet_MacPort macPort)
{
    uint32_t portNum = ENET_MACPORT_NORM(macPort);

    CSL_SGMII_startRxTxSoftReset(sgmiiRegs, portNum);
    CSL_SGMII_endRxTxSoftReset(sgmiiRegs, portNum);

    /* Wait till software reset is complete. SGMII reset is expected
     * to happen immediately */
    while (CSL_SGMII_getRxTxSoftResetStatus(sgmiiRegs, portNum) != 0U)
    {
         EnetUtils_delayTicks(0U);
    }
}

static int32_t CpswMacPort_enableSgmiiLoopback(CSL_Xge_cpswRegs *regs,
                                               CSL_CpsgmiiRegs *sgmiiRegs,
                                               Enet_MacPort macPort)
{
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    int32_t status = ENET_SOK;

    CSL_SGMII_disableAutoNegotiation(sgmiiRegs, portNum);
    CSL_SGMII_startRxTxSoftReset(sgmiiRegs, portNum);
    CSL_SGMII_enableLoopback(sgmiiRegs, portNum);
    CSL_SGMII_endRxTxSoftReset(sgmiiRegs, portNum);

    /* Software reset should happen immediately, hence don't wait for
     * completion, just check if it's done */
    if (CSL_SGMII_getRxTxSoftResetStatus(sgmiiRegs, portNum) != 0U)
    {
        status = ENET_EFAIL;
    }

    if (status == ENET_SOK)
    {
        CSL_CPGMAC_SL_enableLoopback(regs, portNum);
    }

    return status;
}

static int32_t CpswMacPort_setSgmiiInterface(CSL_Xge_cpswRegs *regs,
                                             CSL_CpsgmiiRegs *sgmiiRegs,
                                             Enet_MacPort macPort,
                                             EnetMac_SgmiiMode sgmiiMode,
                                             const EnetMacPort_LinkCfg *linkCfg)
{
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    uint32_t portId = ENET_MACPORT_ID(macPort);
    uint32_t macControl;
    int32_t status = ENET_SOK;

    ENETTRACE_VAR(portId);
    macControl = CSL_CPGMAC_SL_getMacControlReg(regs, portNum);

    /* Clear fields not supported by hardware */
    CSL_FINS(macControl, XGE_CPSW_PN_MAC_CONTROL_REG_EXT_RX_FLOW_EN, 0U);
    CSL_FINS(macControl, XGE_CPSW_PN_MAC_CONTROL_REG_EXT_TX_FLOW_EN, 0U);

    /* In all SGMII modes EXT_EN bit in the CONTROL register must be set to
     * allow the speed and duplexity to be set by the signals from the CPSGMII */
    CSL_FINS(macControl, XGE_CPSW_PN_MAC_CONTROL_REG_EXT_EN, 1U);

    CSL_SGMII_DisableTestPattern(sgmiiRegs, portNum);
    CSL_SGMII_disableMasterMode(sgmiiRegs, portNum);
    CSL_SGMII_disableLoopback(sgmiiRegs, portNum);
    CSL_SGMII_disableAutoNegotiation(sgmiiRegs, portNum);

    /* In case of SGMII, the MDIO controls the auto-negotiation of the PHY to the
     * other side of the fiber or wire (ie. to the remote system). The CPSGMII is
     * the link to the PHY.  The PHY auto-negotiates with the other side and
     * then the CPSGMII auto-negotiates with the PHY. */

    /* Confirm SERDES PLL is locked before configuring the port */
    if (CSL_SGMII_getSerdesPLLLockStatus(sgmiiRegs, portNum) == 0U)
    {
        ENETTRACE_ERR("MAC %u: SERDES PLL is not locked\n", portId);
        status = ENET_EUNEXPECTED;
    }

    if (status == ENET_SOK)
    {
        status = CpswMacPort_configSgmii(sgmiiRegs, sgmiiMode, macPort, linkCfg);
        ENETTRACE_ERR_IF(status != ENET_SOK,
                         "MAC %u: Failed to config SGMII interface: %d\n", portId, status);
    }

    if (status == ENET_SOK)
    {
        CSL_CPGMAC_SL_setMacControlReg(regs, portNum, macControl);
    }

    ENETTRACE_ERR_IF(status != ENET_SOK,
                     "MAC %u: Failed to set SGMII interface: %d\n", portId, status);

    return status;
}

static int32_t CpswMacPort_configSgmii(CSL_CpsgmiiRegs *sgmiiRegs,
                                       EnetMac_SgmiiMode sgmiiMode,
                                       Enet_MacPort macPort,
                                       const EnetMacPort_LinkCfg *linkCfg)
{
    CSL_SGMII_ADVABILITY sgmiiAdvAbility;
    CSL_SGMII_STATUS sgmiiStatus;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    uint32_t portId = ENET_MACPORT_ID(macPort);
    int32_t status = ENET_SOK;

    ENETTRACE_VAR(portId);
    memset(&sgmiiAdvAbility, 0, sizeof(sgmiiAdvAbility));

    if (sgmiiMode == ENET_MAC_SGMIIMODE_FIBER_WITH_PHY)
    {
        ENETTRACE_DBG("MAC %u: Configuring SGMII in FIBER_WITH_PHY mode\n", portId);

        /* In fiber mode Advertise full-duplex only */
        sgmiiAdvAbility.duplexMode = CSL_SGMII_FULL_DUPLEX;
        sgmiiAdvAbility.sgmiiMode  = CSL_SGMII_MODE_FIBER;

        CSL_SGMII_setAdvAbility(sgmiiRegs, portNum, &sgmiiAdvAbility);
        CSL_SGMII_enableAutoNegotiation(sgmiiRegs, portNum);
    }
    else if (sgmiiMode == ENET_MAC_SGMIIMODE_SGMII_WITH_PHY)
    {
        ENETTRACE_DBG("MAC %u: Configuring SGMII in SGMII_WITH_PHY mode\n", portId);

        /* Set highest speed when auto-negotiating with PHY. We don't need to configure
         * user provided speed/duplexity here as it is set in PHY and we auto-negotiate
         * with PHY.  For example, app wants to set 100Mbps full-duplex, it will be set
         * in PHY and PHY would auto-negotiate with with remote PHY with 100Mbps or lower
         * which will be fine with SGMII as we are at highest config. */
        sgmiiAdvAbility.duplexMode = CSL_SGMII_FULL_DUPLEX;
        sgmiiAdvAbility.linkSpeed  = CSL_SGMII_1000_MBPS;
        sgmiiAdvAbility.bLinkUp    = 1U;
        sgmiiAdvAbility.sgmiiMode  = CSL_SGMII_MODE_SGMII;

        CSL_SGMII_setAdvAbility(sgmiiRegs, portNum, &sgmiiAdvAbility);

        /* Note - For SGMII connection with PHY, auto negotiate should always be a '1'
         * (master/slave should be selected) */
        CSL_SGMII_enableAutoNegotiation(sgmiiRegs, portNum);
    }
    else if (sgmiiMode == ENET_MAC_SGMIIMODE_SGMII_AUTONEG_MASTER)
    {
        ENETTRACE_DBG("MAC %u: Configure SGMII in SGMII_AUTONEG_MASTER mode\n", portId);

        /* For SGMII master, advertise full-duplex gigabit */
        sgmiiAdvAbility.linkSpeed  = CSL_SGMII_1000_MBPS;
        sgmiiAdvAbility.duplexMode = CSL_SGMII_FULL_DUPLEX;
        sgmiiAdvAbility.bLinkUp    = 1U;
        sgmiiAdvAbility.sgmiiMode  = CSL_SGMII_MODE_SGMII;

        CSL_SGMII_setAdvAbility(sgmiiRegs, portNum, &sgmiiAdvAbility);
        CSL_SGMII_enableMasterMode(sgmiiRegs, portNum);
        CSL_SGMII_enableAutoNegotiation(sgmiiRegs, portNum);
    }
    else if (sgmiiMode == ENET_MAC_SGMIIMODE_SGMII_AUTONEG_SLAVE)
    {
        ENETTRACE_DBG("MAC %u: Configure SGMII in SGMII_AUTONEG_SLAVE mode\n", portId);

        /* To write 1 to tx_config_reg[0] bit, we pass empty ability structure */
        sgmiiAdvAbility.sgmiiMode  = CSL_SGMII_MODE_SGMII;

        CSL_SGMII_setAdvAbility(sgmiiRegs, portNum, &sgmiiAdvAbility);
        CSL_SGMII_disableMasterMode(sgmiiRegs, portNum);
        CSL_SGMII_enableAutoNegotiation(sgmiiRegs, portNum);
    }
    else if (sgmiiMode == ENET_MAC_SGMIIMODE_SGMII_FORCEDLINK)
    {
        ENETTRACE_DBG("MAC %u: Configure SGMII in SGMII_FORCEDLINK mode\n", portId);

        CpswMacPort_mapSgmiiLinkCfg(&sgmiiAdvAbility, linkCfg);
        sgmiiAdvAbility.bLinkUp   = 1U;
        sgmiiAdvAbility.sgmiiMode = CSL_SGMII_MODE_SGMII;

        CSL_SGMII_setAdvAbility(sgmiiRegs, portNum, &sgmiiAdvAbility);
        CSL_SGMII_enableMasterMode(sgmiiRegs, portNum);
        CSL_SGMII_disableAutoNegotiation(sgmiiRegs, portNum);

        /* Wait for SGMII link */
        do
        {
            CSL_SGMII_getStatus(sgmiiRegs, portNum, &sgmiiStatus);
        }
        while (sgmiiStatus.bIsLinkUp != 1U);

        status = ENET_SOK;
    }
    else
    {
        ENETTRACE_ERR("MAC %u: Invalid SGMII mode config\n", portId);
        status = ENET_EINVALIDPARAMS;
    }

    return status;
}

static void CpswMacPort_mapSgmiiLinkCfg(CSL_SGMII_ADVABILITY *sgmiiAdvAbility,
                                        const EnetMacPort_LinkCfg *linkCfg)
{
    /* Set SGMII duplexity config */
    if (linkCfg->duplexity == ENET_DUPLEX_HALF)
    {
        sgmiiAdvAbility->duplexMode = CSL_SGMII_HALF_DUPLEX;
    }
    else
    {
        sgmiiAdvAbility->duplexMode = CSL_SGMII_FULL_DUPLEX;
    }

    /* Set SGMII speed config */
    if (linkCfg->speed == ENET_SPEED_10MBIT)
    {
        sgmiiAdvAbility->linkSpeed  = CSL_SGMII_10_MBPS;
    }
    else if (linkCfg->speed == ENET_SPEED_100MBIT)
    {
        sgmiiAdvAbility->linkSpeed  = CSL_SGMII_100_MBPS;
    }
    else
    {
        sgmiiAdvAbility->linkSpeed  = CSL_SGMII_1000_MBPS;
    }
}

static bool CpswMacPort_getSgmiiStatus(CSL_CpsgmiiRegs *sgmiiRegs,
                                       Enet_MacPort macPort)
{
    CSL_SGMII_STATUS sgmiiStatus;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);

    CSL_SGMII_getStatus(sgmiiRegs, portNum, &sgmiiStatus);

    return (sgmiiStatus.bIsLinkUp != 0U);
}

static int32_t CpswMacPort_checkSgmiiAutoNegStatus(CSL_CpsgmiiRegs *sgmiiRegs,
                                                   Enet_MacPort macPort)
{
    CSL_SGMII_STATUS sgmiiStatus;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    uint32_t portId = ENET_MACPORT_ID(macPort);
    int32_t status = ENET_SOK;

    ENETTRACE_VAR(portId);
    /* Wait for SGMII Autonegotiation to complete without error */
    do
    {
        CSL_SGMII_getStatus(sgmiiRegs, portNum, &sgmiiStatus);
        if (sgmiiStatus.bIsAutoNegError != 0U)
        {
            /* Auto-negotiation error */
            ENETTRACE_ERR("MAC %u: SGMII auto-negotiation failed: %d\n", portId, status);
            status = ENET_EFAIL;
            break;
        }
    }
    while (sgmiiStatus.bIsAutoNegComplete != 1U);

    /* Check SGMII link status */
    if (status == ENET_SOK)
    {
        /* Link indicator is not valid until the lock status bit is asserted, so
         * check for lock first */
        if (CSL_SGMII_getSerdesPLLLockStatus(sgmiiRegs, portNum) != 1U)
        {
            ENETTRACE_ERR("MAC %u: SGMII SERDES PLL not locked: %d\n", portId, status);
            status = ENET_EUNEXPECTED;
        }
        else
        {
            /* Wait for SGMII link */
            do
            {
                CSL_SGMII_getStatus(sgmiiRegs, portNum, &sgmiiStatus);
            }
            while (sgmiiStatus.bIsLinkUp != 1U);
        }
    }

    return status;
}

static int32_t CpswMacPort_checkSgmiiStatus(CSL_CpsgmiiRegs *sgmiiRegs,
                                            Enet_MacPort macPort)
{
    CSL_SGMII_ADVABILITY sgmiiAdvAbility;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    uint32_t portId = ENET_MACPORT_ID(macPort);
    int32_t status;

    ENETTRACE_VAR(portId);
    if (CSL_SGMII_isAutoNegotiationEnabled(sgmiiRegs, portNum) == 1U)
    {
        status = CpswMacPort_checkSgmiiAutoNegStatus(sgmiiRegs, macPort);
        if (status == ENET_SOK)
        {
            if (CSL_SGMII_isMasterModeEnabled(sgmiiRegs, portNum) == 1U)
            {
                /* In SGMII AUTONEG with MASTER SLAVE config, the master tells the slave
                 * the rate so the slave would auto negotiate to master config. Hence we
                 * report master configured speed/duplexity */
                CSL_SGMII_getAdvAbility(sgmiiRegs, portNum, &sgmiiAdvAbility);
            }
            else
            {
                /* Get link partner speed/duplexity */
                CSL_SGMII_getLinkPartnerAdvAbility(sgmiiRegs, portNum, &sgmiiAdvAbility);
            }

            ENETTRACE_INFO("MAC %u: SGMII link parter config port: link %s: %s %s\n",
                           portId, sgmiiAdvAbility.bLinkUp ? "up" : "down",
                           CpswMacPort_gSgmiiSpeedNames[sgmiiAdvAbility.linkSpeed],
                           CpswMacPort_gSgmiiDuplexNames[sgmiiAdvAbility.duplexMode]);
        }
        else
        {
            ENETTRACE_ERR("MAC %u: SGMII auto-neggotiation failed: %d\n", portId, status);
        }
    }
    else
    {
        /* Forced or loopback mode */
        status = ENET_SOK;
    }

    return status;
}

static int32_t CpswMacPort_enableSgmiiPort(CSL_Xge_cpswRegs *regs,
                                           CSL_CpsgmiiRegs *sgmiiRegs,
                                           Enet_MacPort macPort,
                                           const EnetMacPort_Interface *mii,
                                           const EnetMacPort_LinkCfg *linkCfg)
{
    uint32_t portId = ENET_MACPORT_ID(macPort);
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    uint32_t extCtrl;
    uint32_t macControl;
    int32_t status = ENET_SOK;
    bool forced = true;

    ENETTRACE_VAR(portId);
    macControl = CSL_CPGMAC_SL_getMacControlReg(regs, portNum);

    /* Disable all fields, enable as needed based on link config */
    CSL_FINS(macControl, XGE_CPSW_PN_MAC_CONTROL_REG_FULLDUPLEX, 0U);
    CSL_FINS(macControl, XGE_CPSW_PN_MAC_CONTROL_REG_GMII_EN, 0U);
    CSL_FINS(macControl, XGE_CPSW_PN_MAC_CONTROL_REG_GIG, 0U);
#if ENET_CFG_IS_ON(CPSW_XGMII)
    CSL_FINS(macControl, XGE_CPSW_PN_MAC_CONTROL_REG_XGMII_EN, 0U);
#endif
    CSL_FINS(macControl, XGE_CPSW_PN_MAC_CONTROL_REG_IFCTL_A, 0U);
    CSL_FINS(macControl, XGE_CPSW_PN_MAC_CONTROL_REG_IFCTL_B, 0U);

    if (!EnetMacPort_isSgmii(mii) &&
        !EnetMacPort_isQsgmii(mii))
    {
        status = ENET_EINVALIDPARAMS;
    }

    /* Set speed mode */
    if (status == ENET_SOK)
    {
        extCtrl = CSL_FEXT(macControl, XGE_CPSW_PN_MAC_CONTROL_REG_EXT_EN);
        forced = (extCtrl == TRUE) ? false : true;

        if (linkCfg->speed == ENET_SPEED_1GBIT)
        {
            CSL_FINS(macControl, XGE_CPSW_PN_MAC_CONTROL_REG_GIG, 1U);
        }

        CSL_FINS(macControl, XGE_CPSW_PN_MAC_CONTROL_REG_GMII_EN, 1U);

        status = CpswMacPort_checkSgmiiStatus(sgmiiRegs, macPort);
    }

    /* Half-duplex mode is supported only at 10/100 Mbps */
    if (status == ENET_SOK)
    {
        if ((linkCfg->duplexity == ENET_DUPLEX_HALF) &&
            (linkCfg->speed != ENET_SPEED_10MBIT) &&
            (linkCfg->speed != ENET_SPEED_100MBIT))
        {
            ENETTRACE_ERR("MAC %u: 1Gbps half-duplex is not supported\n", portId);
            status = ENET_EINVALIDPARAMS;
        }
    }

    /* Set duplexity, speed related fields in MAC control */
    if (status == ENET_SOK)
    {
        if (forced && (linkCfg->duplexity == ENET_DUPLEX_FULL))
        {
            CSL_FINS(macControl, XGE_CPSW_PN_MAC_CONTROL_REG_FULLDUPLEX, 1U);
        }

        /* Set interface layer, sublayer, speed */
        CSL_CPGMAC_SL_setMacControlReg(regs, portNum, macControl);
    }

    ENETTRACE_ERR_IF(status != ENET_SOK,
                     "MAC %u: Failed to enable MAC port: %d\n", portId, status);

    return status;
}
#endif

void CpswMacPort_disablePort(CSL_Xge_cpswRegs *regs,
                             Enet_MacPort macPort)
{
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    uint32_t macControl;

    /* Save MAC_CONTROL register context before soft-reset */
    macControl = CSL_CPGMAC_SL_getMacControlReg(regs, portNum);

    CSL_FINS(macControl, XGE_CPSW_PN_MAC_CONTROL_REG_FULLDUPLEX, 0U);
    CSL_FINS(macControl, XGE_CPSW_PN_MAC_CONTROL_REG_GMII_EN, 0U);
    CSL_FINS(macControl, XGE_CPSW_PN_MAC_CONTROL_REG_GIG, 0U);
#if ENET_CFG_IS_ON(CPSW_XGMII)
    CSL_FINS(macControl, XGE_CPSW_PN_MAC_CONTROL_REG_XGMII_EN, 0U);
#endif
    CSL_FINS(macControl, XGE_CPSW_PN_MAC_CONTROL_REG_IFCTL_A, 0U);
    CSL_FINS(macControl, XGE_CPSW_PN_MAC_CONTROL_REG_IFCTL_B, 0U);

    /* Soft-reset the Ethernet MAC logic */
    CpswMacPort_reset(regs, macPort);

    /* Restore context of fields in MAC_CONTROL register that are configured at
     * open time via CpswMacPort_Cfg and that are not related to link configuration */
    CSL_CPGMAC_SL_setMacControlReg(regs, portNum, macControl);
}

static int32_t CpswMacPort_getIoctlHandlerIdx(uint32_t ioctlCmd, CpswMacPortIoctlHandlerRegistry_t *ioctlRegistryTbl, uint32_t tableSize, uint32_t *tblIdx)
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

static CpswMacPortIoctlHandler * CpswMacPort_getIoctlHandlerFxn(uint32_t ioctlCmd, CpswMacPortIoctlHandlerRegistry_t *ioctlRegistryTbl, uint32_t tableSize)
{
    int32_t status;
    uint32_t tblIdx;
    CpswMacPortIoctlHandler *handlerFxn = NULL;

    status = CpswMacPort_getIoctlHandlerIdx(ioctlCmd, ioctlRegistryTbl, tableSize, &tblIdx);
    if (ENET_SOK == status)
    {
        Enet_devAssert(tblIdx < tableSize, "Invalid IOCTL registry table index for IOCTL cmd:%x", ioctlCmd);
        handlerFxn = ioctlRegistryTbl[tblIdx].fxn;
    }
    else
    {
        handlerFxn = &CpswMacPort_ioctl_handler_default;
    }
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to get ioctl handler for ioctl cmd: %x. Using default ioctl handler \r\n", ioctlCmd);
    return handlerFxn;
}

static int32_t CpswMacPort_setIoctlHandlerFxn(uint32_t ioctlCmd,
                                          CpswMacPortIoctlHandler *ioctlHandlerFxn,
                                          CpswMacPortIoctlHandlerRegistry_t *ioctlRegistryTbl,
                                          uint32_t tableSize)
{
    int32_t status;
    uint32_t tblIdx;

    status = CpswMacPort_getIoctlHandlerIdx(ioctlCmd, ioctlRegistryTbl, tableSize, &tblIdx);
    if (ENET_SOK == status)
    {
        Enet_devAssert((tblIdx < tableSize), "Invalid IOCTL registry table index for IOCTL cmd:%x", ioctlCmd);
        Enet_devAssert((ioctlCmd == ioctlRegistryTbl[tblIdx].cmd), "Ioctl table corrupted");
        ioctlRegistryTbl[tblIdx].fxn = ioctlHandlerFxn;
    }
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to set ioctl handler for ioctl cmd: %x \r\n", ioctlCmd);
    return status;
}

static int32_t CpswMacPort_ioctl_handler_CPSW_MACPORT_IOCTL_REGISTER_HANDLER(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    const Enet_IoctlRegisterHandlerInArgs *inArgs = (const Enet_IoctlRegisterHandlerInArgs *)prms->inArgs;
    int32_t status = ENET_ENOTSUPPORTED;

    status = CpswMacPort_setIoctlHandlerFxn(inArgs->cmd,
                                         (CpswMacPortIoctlHandler *)inArgs->fxn,
                                         CpswMacPortIoctlHandlerRegistry,
                                         ENET_ARRAYSIZE(CpswMacPortIoctlHandlerRegistry));
    return status;
}


static int32_t CpswMacPort_ioctl_handler_default(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    return ENET_ENOTSUPPORTED;
}
