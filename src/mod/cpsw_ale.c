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
 * \file  cpsw_ale.c
 *
 * \brief This file contains the implementation of the CPSW ALE.
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
#include <include/mod/cpsw_ale.h>
#include <priv/core/enet_trace_priv.h>
#include <priv/mod/cpsw_ale_priv.h>
#include <priv/mod/cpsw_ale_ioctl_priv.h>
#include <priv/mod/cpsw_clks.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Supported AM64x version */
#define CPSW_ALE_VER_REVMAJ_AM64X              (0x00000001U)
#define CPSW_ALE_VER_REVMIN_AM64X              (0x00000004U)
#define CPSW_ALE_VER_RTL_AM64X                 (0x00000007U)
#define CPSW_ALE_VER_ID_AM64X                  (0x00000029U)

/* Supported AM273X version */
#define CPSW_ALE_VER_REVMAJ_AM273X              (0x00000001U)
#define CPSW_ALE_VER_REVMIN_AM273X              (0x00000004U)
#define CPSW_ALE_VER_RTL_AM273X                 (0x00000008U)
#define CPSW_ALE_VER_ID_AM273X                  (0x00000029U)

/* Supported AM263X version */
#define CPSW_ALE_VER_REVMAJ_AM263X              (0x00000001U)
#define CPSW_ALE_VER_REVMIN_AM263X              (0x00000005U)
#define CPSW_ALE_VER_RTL_AM263X                 (0x00000000U)
#define CPSW_ALE_VER_ID_AM263X                  (0x00000029U)

/* Number of ALE table entries and policers per CPSW variant */
#if defined(SOC_AM273X) || defined(SOC_AWR294X) || defined(SOC_AWR2544)
#define CPSW_ALE_2G_ALE_ENTRIES_MAX             (32U)
#else
#define CPSW_ALE_2G_ALE_ENTRIES_MAX             (64U)
#endif
#if defined(SOC_AM273X) || defined(SOC_AWR294X) || defined(SOC_AWR2544)
#define CPSW_ALE_2G_POLICERS_MAX                (4U)
#else
#define CPSW_ALE_2G_POLICERS_MAX                (8U)
#endif
#define CPSW_ALE_3G_ALE_ENTRIES_MAX             (512U)
#define CPSW_ALE_3G_POLICERS_MAX                (32U)
#define CPSW_ALE_5G_ALE_ENTRIES_MAX             (512U)
#define CPSW_ALE_5G_POLICERS_MAX                (64U)
#define CPSW_ALE_9G_ALE_ENTRIES_MAX             (1024U)
#define CPSW_ALE_9G_POLICERS_MAX                (96U)

/*! \brief CIR/PIR policing rate divider. */
#define CPSW_ALE_IDLINCVAL_DIV_FACTOR           (32768U)

/*! \brief Aging timer divider. */
#define CPSW_ALE_AGING_TIMER_DIV_FACTOR         (1000000U)

/*! \brief VLAN ID field mask. */
#define CPSW_ALE_VLAN_ID_MASK                   ((uint32_t)ENET_VLAN_ID_MAX)

/* TODO: Make them part of CpswAle_Cfg */
#define CPSW_ALE_CFG_BCAST_MCAST_PRESCALE       (250000U)
#define CPSW_ALE_CFG_DEFAULT_AGING_PERIOD_MS    (5000U)

/*!
 * \brief Set value only if pointer is not NULL.
 *
 * Useful to handle cases where a NULL pointer means value is don't care and
 * need not be set.
 */
#define CPSW_ALE_SAFE_ASSIGN(ptr, val)        \
    do {                                      \
        if ((ptr) != NULL)                    \
        {                                     \
            *(ptr) = (val);                   \
        }                                     \
} while (0)

#define NOT_ZERO(x)                             ((uint32_t)0U != (uint32_t)(x))

#define CPSW_ALE_MAC_ADDR_INIT_LIST(macAddr)    { macAddr[0], macAddr[1], \
                                                  macAddr[2], macAddr[3], \
                                                  macAddr[4], macAddr[5] }

#define CPSW_ALE_OUI_ADDR_INIT_LIST(ouiAddr)    { ouiAddr[0], ouiAddr[1], \
                                                  ouiAddr[2] }

#define CPSW_ALE_IPV4_ADDR_INIT_LIST(ipv4Addr)  { ipv4Addr[0], ipv4Addr[1], \
                                                  ipv4Addr[2], ipv4Addr[3]}

#define CPSW_ALE_IPV6_ADDR_INIT_LIST(ipv6Addr)  { ipv6Addr[0], ipv6Addr[1],   \
                                                  ipv6Addr[2], ipv6Addr[3],   \
                                                  ipv6Addr[4], ipv6Addr[5],   \
                                                  ipv6Addr[6], ipv6Addr[7],   \
                                                  ipv6Addr[8], ipv6Addr[9],   \
                                                  ipv6Addr[10], ipv6Addr[11], \
                                                  ipv6Addr[12], ipv6Addr[13], \
                                                  ipv6Addr[14], ipv6Addr[15]}

/* The first ALE Mask Mux is a readonly register set to ALL_PORTS mask */
#define CPSW_ALE_VLAN_MASK_MUX_COUNT(regs)      (ENET_ARRAYSIZE(regs->I1_ALE_MSK_MUX1) + 1)
#define CPSW_ALE_VLANMASKMUX2REG(index)         ((index) - 1U)

#define CPSW_ALE_NOMATCH_ENTRY0_POLICER_INDEX   (0)
#define CPSW_ALE_BCAST_ADDR_INIT_LIST           { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }
#define CPSW_ALE_BLOCKCLASSIFIER_PORTMASK       (0)

/* Default field values for unicast entry */
#define CPSW_ALE_UCAST_ENTRY_DFLT_BLOCKEDFLAG   (0U)
#define CPSW_ALE_UCAST_ENTRY_DFLT_SECUREFLAG    (0U)
#define CPSW_ALE_UCAST_ENTRY_DFLT_AGEABLEFLAG   (0U)
#define CPSW_ALE_UCAST_ENTRY_DFLT_TRUNKFLAG     (0U)

/* Default field values for multicast entry */
#define CPSW_ALE_MCAST_ENTRY_DFLT_SUPERFLAG     (0U)
#define CPSW_ALE_MCAST_ENTRY_DFLT_FWDSTATE      (CPSW_ALE_FWDSTLVL_FWD)
#define CPSW_ALE_MCAST_ENTRY_DFLT_IGNBITS       (0U)

/* Default field values for VLAN entry */
#define CPSW_ALE_VLAN_ENTRY_DFLT_NOLEARNMASK    (0U)
#define CPSW_ALE_VLAN_ENTRY_DFLT_VIDINGRESSCHK  (0U)
#define CPSW_ALE_VLAN_ENTRY_DFLT_LIMITIPNXTHDR  (0U)
#define CPSW_ALE_VLAN_ENTRY_DFLT_DISALLOWFRAG   (0U)

/* Route index maps to egressOp and should be non-zero */
#define CPSW_ALE_BLOCKCLASSIFIER_ROUTEINDEX     (1)

#define CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT(x)    \
          {.cmd = x,                            \
           .fxn = &CpswAle_ioctl_handler_##x}

#define CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(x)    \
          {.cmd = x,                            \
           .fxn = &CpswAle_ioctl_handler_default}


/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef const uint8_t (*const CpswAle_MacAddrType)[ENET_MAC_ADDR_LEN];
typedef const uint8_t (*const CpswAle_OuiAddrType)[ENET_OUI_ADDR_LEN];
typedef const uint8_t (*const CpswAle_IPv4AddrType)[ENET_IPv4_ADDR_LEN];
typedef const uint8_t (*const CpswAle_IPv6AddrType)[ENET_IPv6_ADDR_LEN];

typedef int32_t (CpswAleIoctlHandler)(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);

typedef struct CpswAleIoctlHandlerRegistry_s
{
    uint32_t cmd;
    CpswAleIoctlHandler *fxn;
} CpswAleIoctlHandlerRegistry_t;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
#if ENET_CFG_IS_ON(DEV_ERROR)
static int32_t CpswAle_isSupported(CSL_AleRegs *regs);
#endif
static uint32_t CpswAle_mapAleAgingIntervalMsToAgingTimer(uint32_t aleFreqHz,
                                                          uint32_t aleAgingIntervalInMs);
static CSL_CPSW_ALETABLE_TYPE CpswAle_getTableType(Enet_Type enetType);
static bool CpswAle_isVlanMaskMuxRegSupported(CSL_AleRegs *regs);
static void CpswAle_setAleCfg(CpswAle_Handle hAle,
                              CSL_AleRegs *regs,
                              const CpswAle_Cfg *aleCfg);
static int32_t CpswAle_addDefaultEntries(CpswAle_Handle hAle);
static void CpswAle_initAleRegs(CSL_AleRegs *regs);
static void CpswAle_setAleModeFlags(CSL_AleRegs *regs,
                                    uint32_t modeFlags);
static void CpswAle_setAleAging(CSL_AleRegs *regs,
                                const CpswAle_AgingCfg *agingCfg,
                                uint32_t aleFreqHz,
                                bool *softTimerActive,
                                uint32_t *tickTimeoutCount,
                                uint32_t *softTickCount);
static void CpswAle_setUnknownVlanCfg(CSL_AleRegs *regs,
                                      uint32_t memberList,
                                      uint32_t forceUntaggedEgress,
                                      uint32_t regMcastFloodMask,
                                      uint32_t unregMcastFloodMask);
static int32_t CpswAle_setNoMatchEntry0Policer(CSL_AleRegs *regs,
                                               uint32_t aleFreqHz,
                                               uint32_t peakRateInBitsPs,
                                               uint32_t commitRateInBitsPs);
static void CpswAle_clearGlobalPolicerStats(CSL_AleRegs *regs,
                                     bool clearAllHit,
                                     bool clearRedHit,
                                     bool clearYellowHit);

static void CpswAle_setAlePortCfg(CSL_AleRegs *regs,
                                  uint32_t portNum,
                                  const CpswAle_PortCfg *portCfg,
                                  bool macAuthForPortDis);

static void CpswAle_setNetSecCfg(CSL_AleRegs *regs,
                                 const CpswAle_NetworkSecurityCfg *netSecCfg);

static void CpswAle_setIPFilterCfg(CSL_AleRegs *regs,
                                   const CpswAle_IPPktSecurityCfg *ipSecCfg);

static void CpswAle_initVlanCfg(CSL_AleRegs *regs,
                                const CpswAle_InitVlanCfg *vlanCfg);

static bool CpswAle_delIPv6HighEntry(CpswAle_Handle hAle,
                                     CSL_AleRegs *regs,
                                     uint32_t entryIdx);

static int32_t CpswAle_delAllPolicerEntries(CpswAle_Handle hAle,
                                            CSL_AleRegs *regs);
#if ENET_CFG_IS_ON(ALE_VLAN_MASK_MUX)
static int32_t CpswAle_findVlanMaskMuxEntry(CpswAle_Handle hAle,
                                            CSL_AleRegs *regs,
                                            uint32_t matchMask,
                                            uint32_t *entryIdx);
static int32_t CpswAle_getVlanMaskMuxFreeEntry(CpswAle_Handle hAle,
                                               CSL_AleRegs *regs,
                                               uint32_t *entryIdx);
static int32_t CpswAle_addVlanMask(CpswAle_Handle hAle,
                                   CSL_AleRegs *regs,
                                   uint32_t mask,
                                   uint32_t *entryIdx);
#endif

static int32_t CpswAle_getIoctlHandlerIdx(uint32_t ioctlCmd, 
                                          CpswAleIoctlHandlerRegistry_t *ioctlRegistryTbl, 
                                          uint32_t tableSize, 
                                          uint32_t *tblIdx);
static CpswAleIoctlHandler * CpswAle_getIoctlHandlerFxn(uint32_t ioctlCmd, 
                                                        CpswAleIoctlHandlerRegistry_t *ioctlRegistryTbl,
                                                        uint32_t tableSize);
static int32_t CpswAle_setIoctlHandlerFxn(uint32_t ioctlCmd,
                                          CpswAleIoctlHandler *ioctlHandlerFxn,
                                          CpswAleIoctlHandlerRegistry_t *ioctlRegistryTbl,
                                          uint32_t tableSize);
static int32_t CpswAle_ioctl_handler_default(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms);
static int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_REGISTER_HANDLER(CpswAle_Handle hAle, 
                                                                     CSL_AleRegs *regs, 
                                                                     Enet_IoctlPrms *prms);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

#if ENET_CFG_IS_ON(DEV_ERROR)
/*! \brief CPSW ALE versions supported by this driver. */
static CSL_CPSW_VERSION CpswAle_gSupportedVer[] =
{
    {   /* AM64x */
        .majorVer = CPSW_ALE_VER_REVMAJ_AM64X,
        .minorVer = CPSW_ALE_VER_REVMIN_AM64X,
        .rtlVer   = CPSW_ALE_VER_RTL_AM64X,
        .id       = CPSW_ALE_VER_ID_AM64X,
    },
    {   /* AM273x */
        .majorVer = CPSW_ALE_VER_REVMAJ_AM273X,
        .minorVer = CPSW_ALE_VER_REVMIN_AM273X,
        .rtlVer   = CPSW_ALE_VER_RTL_AM273X,
        .id       = CPSW_ALE_VER_ID_AM273X,
    },
	{   /* AM263x */
        .majorVer = CPSW_ALE_VER_REVMAJ_AM263X,
        .minorVer = CPSW_ALE_VER_REVMIN_AM263X,
        .rtlVer   = CPSW_ALE_VER_RTL_AM263X,
        .id       = CPSW_ALE_VER_ID_AM263X,
    },
};

/*! \brief CPSW ALE IOCTL validation data. */
static Enet_IoctlValidate gCpswAle_ioctlValidate[] =
{
    ENET_IOCTL_VALID_PRMS(CPSW_ALE_IOCTL_DUMP_TABLE,
                          0U,
                          0U),

    ENET_IOCTL_VALID_PRMS(CPSW_ALE_IOCTL_ADD_UCAST,
                          sizeof(CpswAle_SetUcastEntryInArgs),
                          sizeof(uint32_t)),

    ENET_IOCTL_VALID_PRMS(CPSW_ALE_IOCTL_ADD_MCAST,
                          sizeof(CpswAle_SetMcastEntryInArgs),
                          sizeof(uint32_t)),

    ENET_IOCTL_VALID_PRMS(CPSW_ALE_IOCTL_ADD_VLAN,
                          sizeof(CpswAle_VlanEntryInfo),
                          sizeof(uint32_t)),

    ENET_IOCTL_VALID_PRMS(CPSW_ALE_IOCTL_ADD_OUI,
                          sizeof(CpswAle_OuiEntryInfo),
                          sizeof(uint32_t)),

    ENET_IOCTL_VALID_PRMS(CPSW_ALE_IOCTL_ADD_IPV4ADDR,
                          sizeof(CpswAle_IPv4EntryInfo),
                          sizeof(uint32_t)),

    ENET_IOCTL_VALID_PRMS(CPSW_ALE_IOCTL_ADD_IPV6ADDR,
                          sizeof(CpswAle_IPv6EntryInfo),
                          sizeof(uint32_t)),

    ENET_IOCTL_VALID_PRMS(CPSW_ALE_IOCTL_ADD_ETHERTYPE,
                          sizeof(uint16_t),
                          sizeof(uint32_t)),

    ENET_IOCTL_VALID_PRMS(CPSW_ALE_IOCTL_LOOKUP_UCAST,
                          sizeof(CpswAle_MacAddrInfo),
                          sizeof(CpswAle_GetUcastEntryOutArgs)),

    ENET_IOCTL_VALID_PRMS(CPSW_ALE_IOCTL_LOOKUP_MCAST,
                          sizeof(CpswAle_GetMcastEntryInArgs),
                          sizeof(CpswAle_GetMcastEntryOutArgs)),

    ENET_IOCTL_VALID_PRMS(CPSW_ALE_IOCTL_LOOKUP_VLAN,
                          sizeof(CpswAle_VlanIdInfo),
                          sizeof(CpswAle_GetVlanEntryOutArgs)),

    ENET_IOCTL_VALID_PRMS(CPSW_ALE_IOCTL_REMOVE_ADDR,
                          sizeof(CpswAle_MacAddrInfo),
                          0U),

    ENET_IOCTL_VALID_PRMS(CPSW_ALE_IOCTL_REMOVE_VLAN,
                          sizeof(CpswAle_VlanIdInfo),
                          0U),

    ENET_IOCTL_VALID_PRMS(CPSW_ALE_IOCTL_REMOVE_OUI,
                          sizeof(CpswAle_OuiEntryInfo),
                          0U),

    ENET_IOCTL_VALID_PRMS(CPSW_ALE_IOCTL_REMOVE_IPV4ADDR,
                          sizeof(CpswAle_IPv4EntryInfo),
                          0U),

    ENET_IOCTL_VALID_PRMS(CPSW_ALE_IOCTL_REMOVE_IPV6ADDR,
                          sizeof(CpswAle_IPv6EntryInfo),
                          0U),

    ENET_IOCTL_VALID_PRMS(CPSW_ALE_IOCTL_REMOVE_ETHERTYPE,
                          sizeof(uint16_t),
                          0U),

    ENET_IOCTL_VALID_PRMS(CPSW_ALE_IOCTL_REMOVE_LEARNED_ENTRIES,
                          sizeof(uint32_t),
                          sizeof(uint32_t)),

    ENET_IOCTL_VALID_PRMS(CPSW_ALE_IOCTL_REMOVE_ALL_ENTRIES,
                          0U,
                          0U),

    ENET_IOCTL_VALID_PRMS(CPSW_ALE_IOCTL_AGE_ALL_ENTRIES,
                          0U,
                          0U),

    ENET_IOCTL_VALID_PRMS(CPSW_ALE_IOCTL_SET_RX_FILTER,
                          sizeof(CpswAle_RxFilter),
                          0U),

    ENET_IOCTL_VALID_PRMS(CPSW_ALE_IOCTL_GET_RX_FILTER,
                          0U,
                          sizeof(CpswAle_RxFilter)),

    ENET_IOCTL_VALID_PRMS(CPSW_ALE_IOCTL_SET_PORT_STATE,
                          sizeof(CpswAle_SetPortStateInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(CPSW_ALE_IOCTL_GET_PORT_STATE,
                          sizeof(uint32_t),
                          sizeof(CpswAle_PortState)),

    ENET_IOCTL_VALID_PRMS(CPSW_ALE_IOCTL_GET_PORT_MACADDR,
                          sizeof(CpswAle_GetPortMacAddrInArgs),
                          sizeof(CpswAle_GetPortMacAddrOutArgs)),

    ENET_IOCTL_VALID_PRMS(CPSW_ALE_IOCTL_SET_DEFAULT_THREADCFG,
                          sizeof(CpswAle_DfltThreadCfg),
                          0U),

    ENET_IOCTL_VALID_PRMS(CPSW_ALE_IOCTL_GET_DEFAULT_THREADCFG,
                          0U,
                          sizeof(CpswAle_DfltThreadCfg)),

    ENET_IOCTL_VALID_PRMS(CPSW_ALE_IOCTL_SET_PORT_MIRROR_CFG,
                          sizeof(CpswAle_PortMirroringCfg),
                          0U),

    ENET_IOCTL_VALID_PRMS(CPSW_ALE_IOCTL_DISABLE_PORT_MIRROR,
                          0U,
                          0U),

    ENET_IOCTL_VALID_PRMS(CPSW_ALE_IOCTL_SET_TRUNK_CFG,
                          sizeof(CpswAle_TrunkCfg),
                          0U),

    ENET_IOCTL_VALID_PRMS(CPSW_ALE_IOCTL_SET_OAMLPBK_CFG,
                          sizeof(uint32_t),
                          0U),

    ENET_IOCTL_VALID_PRMS(CPSW_ALE_IOCTL_SET_BCAST_MCAST_LIMIT,
                          sizeof(CpswAle_SetBcastMcastRateLimitInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(CPSW_ALE_IOCTL_GET_BCAST_MCAST_LIMIT,
                          0U,
                          sizeof(CpswAle_GetBcastMcastRateLimitOutArgs)),

    ENET_IOCTL_VALID_PRMS(CPSW_ALE_IOCTL_DISABLE_BCAST_MCAST_LIMIT,
                          0U,
                          0U),

    ENET_IOCTL_VALID_PRMS(CPSW_ALE_IOCTL_SET_POLICER,
                          sizeof(CpswAle_SetPolicerEntryInArgs),
                          sizeof(CpswAle_SetPolicerEntryOutArgs)),

    ENET_IOCTL_VALID_PRMS(CPSW_ALE_IOCTL_GET_POLICER,
                          sizeof(CpswAle_PolicerMatchParams),
                          sizeof(CpswAle_PolicerEntryOutArgs)),

    ENET_IOCTL_VALID_PRMS(CPSW_ALE_IOCTL_DEL_POLICER,
                          sizeof(CpswAle_DelPolicerEntryInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(CPSW_ALE_IOCTL_DUMP_POLICER_ENTRIES,
                          0U,
                          0U),

    ENET_IOCTL_VALID_PRMS(CPSW_ALE_IOCTL_GET_POLICER_STATS,
                          sizeof(CpswAle_GetPolicerStatsInArgs),
                          sizeof(CpswAle_GetPolicerStatsOutArgs)),

    ENET_IOCTL_VALID_PRMS(CPSW_ALE_IOCTL_SET_POLICER_THREADCFG,
                          sizeof(CpswAle_SetPolicerThreadCfgInArgs),
                          sizeof(CpswAle_PolicerEntryOutArgs)),

    ENET_IOCTL_VALID_PRMS(CPSW_ALE_IOCTL_SET_POLICER_GLOBAL_CFG,
                          sizeof(CpswAle_PolicerGlobalCfg),
                          0U),

    ENET_IOCTL_VALID_PRMS(CPSW_ALE_IOCTL_GET_POLICER_GLOBAL_CFG,
                          0U,
                          sizeof(CpswAle_PolicerGlobalCfg)),

    ENET_IOCTL_VALID_PRMS(CPSW_ALE_IOCTL_DEL_ALL_POLICER_THREADID,
                          sizeof(uint32_t),
                          0U),

    ENET_IOCTL_VALID_PRMS(CPSW_ALE_IOCTL_BLOCK_CLASSIFIER_HOSTPORT,
                          sizeof(CpswAle_PolicerMatchParams),
                          sizeof(CpswAle_PolicerEntryOutArgs)),

#if ENET_CFG_IS_ON(CPSW_INTERVLAN)
    ENET_IOCTL_VALID_PRMS(CPSW_ALE_IOCTL_SET_INTERVLAN_CFG,
                          sizeof(CpswAle_SetInterVlanCfgInArgs),
                          sizeof(CpswAle_PolicerEntryOutArgs)),

    ENET_IOCTL_VALID_PRMS(CPSW_ALE_IOCTL_GET_INTERVLAN_CFG,
                          0U,
                          0U),
#endif

};

/*! \brief CPSW ALE IOCTL validation data. */
static Enet_IoctlValidate gCpswAle_privIoctlValidate[] =
{
    ENET_IOCTL_VALID_PRMS(CPSW_ALE_IOCTL_REGISTER_HANDLER,
                          sizeof(Enet_IoctlRegisterHandlerInArgs),
                          0U),
};

#endif



static CpswAleIoctlHandlerRegistry_t CpswAleIoctlHandlerRegistry[] = 
{
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_ALE_IOCTL_SET_PORT_STATE),
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_FDB_IOCTL_GET_VERSION),
#if ENET_CFG_IS_ON(DEV_ERROR)
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_FDB_IOCTL_PRINT_REGS),
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_ALE_IOCTL_DUMP_TABLE),
#endif    
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_ALE_IOCTL_ADD_UCAST),
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_ALE_IOCTL_ADD_MCAST),
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_ALE_IOCTL_ADD_VLAN),
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_ALE_IOCTL_ADD_OUI),
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_ALE_IOCTL_ADD_IPV4ADDR),
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_ALE_IOCTL_ADD_IPV6ADDR),
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_ALE_IOCTL_ADD_ETHERTYPE),
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_ALE_IOCTL_LOOKUP_UCAST),
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_ALE_IOCTL_LOOKUP_MCAST),
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_ALE_IOCTL_LOOKUP_VLAN),
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_ALE_IOCTL_REMOVE_ADDR),
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_ALE_IOCTL_REMOVE_VLAN),
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_ALE_IOCTL_REMOVE_OUI),
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_ALE_IOCTL_REMOVE_IPV4ADDR),
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_ALE_IOCTL_REMOVE_IPV6ADDR),
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_ALE_IOCTL_REMOVE_ETHERTYPE),
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_ALE_IOCTL_REMOVE_LEARNED_ENTRIES),
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_ALE_IOCTL_REMOVE_ALL_ENTRIES),
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_ALE_IOCTL_AGE_ALL_ENTRIES),
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_ALE_IOCTL_SET_RX_FILTER),
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_ALE_IOCTL_GET_RX_FILTER),
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_ALE_IOCTL_GET_PORT_STATE),
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_ALE_IOCTL_GET_PORT_MACADDR),
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_ALE_IOCTL_SET_DEFAULT_THREADCFG),
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_ALE_IOCTL_GET_DEFAULT_THREADCFG),
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_ALE_IOCTL_SET_PORT_MIRROR_CFG),
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_ALE_IOCTL_DISABLE_PORT_MIRROR),
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_ALE_IOCTL_SET_TRUNK_CFG),
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_ALE_IOCTL_SET_OAMLPBK_CFG),
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_ALE_IOCTL_SET_BCAST_MCAST_LIMIT),
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_ALE_IOCTL_GET_BCAST_MCAST_LIMIT),
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_ALE_IOCTL_DISABLE_BCAST_MCAST_LIMIT),
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_ALE_IOCTL_SET_POLICER),
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_ALE_IOCTL_GET_POLICER),
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_ALE_IOCTL_DEL_POLICER),
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_ALE_IOCTL_DUMP_POLICER_ENTRIES),
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_ALE_IOCTL_GET_POLICER_STATS),
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_ALE_IOCTL_SET_POLICER_THREADCFG),
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_ALE_IOCTL_SET_POLICER_GLOBAL_CFG),
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_ALE_IOCTL_GET_POLICER_GLOBAL_CFG),
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_ALE_IOCTL_DEL_ALL_POLICER_THREADID),
#if ENET_CFG_IS_ON(CPSW_INTERVLAN)
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_ALE_IOCTL_BLOCK_CLASSIFIER_HOSTPORT),
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_ALE_IOCTL_SET_INTERVLAN_CFG),
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_ALE_IOCTL_GET_INTERVLAN_CFG),
#endif
    CPSW_ALE_IOCTL_HANDLER_ENTRY_INIT(CPSW_ALE_IOCTL_REGISTER_HANDLER),
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void CpswAle_initCfg(CpswAle_Cfg *aleCfg)
{
    uint32_t i;

    memset(aleCfg, 0, sizeof(*aleCfg));
    aleCfg->modeFlags = CPSW_ALE_CFG_MODULE_EN;
    aleCfg->policerGlobalCfg.policingEn   = TRUE;
    aleCfg->policerGlobalCfg.redDropEn    = TRUE;
    aleCfg->policerGlobalCfg.yellowDropEn = FALSE;

    aleCfg->agingCfg.autoAgingEn     = TRUE;
    aleCfg->agingCfg.agingPeriodInMs = CPSW_ALE_CFG_DEFAULT_AGING_PERIOD_MS;
    aleCfg->vlanCfg.aleVlanAwareMode  = TRUE;
    aleCfg->vlanCfg.cpswVlanAwareMode = FALSE;
    aleCfg->vlanCfg.unknownForceUntaggedEgressMask = 0;
    aleCfg->vlanCfg.unknownRegMcastFloodMask   = CPSW_ALE_ALL_PORTS_MASK;
    aleCfg->vlanCfg.unknownUnregMcastFloodMask = CPSW_ALE_ALL_PORTS_MASK;
    aleCfg->vlanCfg.unknownVlanMemberListMask  = CPSW_ALE_ALL_PORTS_MASK;
    aleCfg->nwSecCfg.vid0ModeEn                      = TRUE;
    aleCfg->nwSecCfg.hostOuiNoMatchDeny              = FALSE;
    aleCfg->nwSecCfg.ipPktCfg.dfltNoFragEn           = FALSE;
    aleCfg->nwSecCfg.ipPktCfg.dfltNxtHdrWhitelistEn  = FALSE;
    aleCfg->nwSecCfg.macAuthCfg.authModeEn           = FALSE;
    aleCfg->nwSecCfg.malformedPktCfg.badLenPktDropEn = FALSE;
    aleCfg->nwSecCfg.malformedPktCfg.srcMcastDropDis = FALSE;

    for (i = 0U; i < ENET_ARRAYSIZE(aleCfg->portCfg); i++)
    {
        aleCfg->portCfg[i].learningCfg.noLearn         = FALSE;
        aleCfg->portCfg[i].learningCfg.noSaUpdateEn    = FALSE;
        aleCfg->portCfg[i].macModeCfg.macOnlyEn        = FALSE;
        aleCfg->portCfg[i].pvidCfg.disallowIPFrag      = FALSE;
        aleCfg->portCfg[i].pvidCfg.forceUntaggedEgressMask = 0;
        aleCfg->portCfg[i].pvidCfg.limitIPNxtHdr       = FALSE;
        aleCfg->portCfg[i].pvidCfg.noLearnMask         = 0;
        aleCfg->portCfg[i].pvidCfg.regMcastFloodMask   = CPSW_ALE_ALL_PORTS_MASK;
        aleCfg->portCfg[i].pvidCfg.unregMcastFloodMask = CPSW_ALE_ALL_PORTS_MASK;
        aleCfg->portCfg[i].pvidCfg.vidIngressCheck     = 0;
        aleCfg->portCfg[i].pvidCfg.vlanIdInfo.tagType  = ENET_VLAN_TAG_TYPE_INNER;
        aleCfg->portCfg[i].pvidCfg.vlanIdInfo.vlanId   = 0;
        aleCfg->portCfg[i].pvidCfg.vlanMemberList      = CPSW_ALE_ALL_PORTS_MASK;
    }
}

int32_t CpswAle_open(EnetMod_Handle hMod,
                     Enet_Type enetType,
                     uint32_t instId,
                     const void *cfg,
                     uint32_t cfgSize)
{
    CpswAle_Handle hAle = (CpswAle_Handle)hMod;
    const CpswAle_Cfg *aleCfg = (const CpswAle_Cfg *)cfg;
    CSL_AleRegs *regs = (CSL_AleRegs *)hMod->virtAddr;
#if ENET_CFG_IS_ON(DEV_ERROR)
    uint32_t numEntries;
    uint32_t numPolicers;
#endif
    int32_t status = ENET_SOK;

    Enet_devAssert(cfgSize == sizeof(CpswAle_Cfg),
                   "Invalid ALE config params size %u (expected %u)\n",
                   cfgSize, sizeof(CpswAle_Cfg));

    /* Check supported ALE module versions */
#if ENET_CFG_IS_ON(DEV_ERROR)
    status = CpswAle_isSupported(regs);
    Enet_devAssert(status == ENET_SOK, "ALE version is not supported\n");

    /* Clear ALE address table after POR in order to prevent ECC errors when
     * reading the ALE table in CpswAle_getMaxAleEntries() */
    CSL_CPSW_clearAleTable(regs);

    /* Check expected number of policers and ALE table entries */
    numEntries = CpswAle_getMaxAleEntries(regs);
    numPolicers = CpswAle_getMaxPolicers(regs);

    switch (enetType)
    {
        case ENET_CPSW_9G:
            Enet_devAssert(numEntries == CPSW_ALE_9G_ALE_ENTRIES_MAX,
                           "Unexpected number of ALE entries (got %u, expected %u)\n",
                           numEntries, CPSW_ALE_9G_ALE_ENTRIES_MAX);
            Enet_devAssert(numPolicers == CPSW_ALE_9G_POLICERS_MAX,
                           "Unexpected number of policers (got %u, expected %u)\n",
                           numPolicers, CPSW_ALE_9G_POLICERS_MAX);
            break;

        case ENET_CPSW_5G:
            Enet_devAssert(numEntries == CPSW_ALE_5G_ALE_ENTRIES_MAX,
                           "Unexpected number of ALE entries (got %u, expected %u)\n",
                           numEntries, CPSW_ALE_5G_ALE_ENTRIES_MAX);
            Enet_devAssert(numPolicers == CPSW_ALE_5G_POLICERS_MAX,
                           "Unexpected number of policers (got %u, expected %u)\n",
                           numPolicers, CPSW_ALE_5G_POLICERS_MAX);
            break;

        case ENET_CPSW_3G:
            Enet_devAssert(numEntries == CPSW_ALE_3G_ALE_ENTRIES_MAX,
                           "Unexpected number of ALE entries (got %u, expected %u)\n",
                           numEntries, CPSW_ALE_3G_ALE_ENTRIES_MAX);
            Enet_devAssert(numPolicers == CPSW_ALE_3G_POLICERS_MAX,
                           "Unexpected number of policers (got %u, expected %u)\n",
                           numPolicers, CPSW_ALE_3G_POLICERS_MAX);
            break;

        case ENET_CPSW_2G:
            Enet_devAssert(numEntries == CPSW_ALE_2G_ALE_ENTRIES_MAX,
                           "Unexpected number of ALE entries (got %u, expected %u)\n",
                           numEntries, CPSW_ALE_2G_ALE_ENTRIES_MAX);
#if !(defined(SOC_AM273X) || defined(SOC_AWR294X) || defined(SOC_AWR2544))
            /*
             * There are 4 entries  for AM273X, AWR294X which can be represented as div8 value,
             * Skip the test for AM273X, AWR294X
             */
            Enet_devAssert(numPolicers == CPSW_ALE_2G_POLICERS_MAX,
                           "Unexpected number of policers (got %u, expected %u)\n",
                           numPolicers, CPSW_ALE_2G_POLICERS_MAX);
#endif
            break;

        default:
            Enet_devAssert(false, "Invalid Ethernet type %u\n", enetType);
            break;
    }
#endif

#if ENET_CFG_IS_OFF(ALE_VLAN_MASK_MUX)
    Enet_devAssert(CpswAle_isVlanMaskMuxRegSupported(regs) == false);
#endif

#if ENET_CFG_IS_ON(ALE_VLAN_MASK_MUX)
    Enet_devAssert(CpswAle_isVlanMaskMuxRegSupported(regs) == true);
#endif

    /* Save peripheral info to use it later to query SoC parameters */
    hAle->enetType = enetType;
    hAle->instId = instId;

    /* Get ALE table type: CPSW_2G has TYPE_4PORT, CPSW_5G/CPSW_9G have TYPE_9PORT */
    hAle->tableType = CpswAle_getTableType(enetType);

    /* Number of ALE ports = MAC ports + host port */
    hAle->numPorts = EnetSoc_getMacPortMax(enetType, instId) + 1U;

    /* Set ALE open time configuration */
    CpswAle_setAleCfg(hAle, regs, aleCfg);

    /* Add port default VLAN entries */
    status = CpswAle_addDefaultEntries(hAle);
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to add default VLAN entries: %d\n", status);

    return status;
}

int32_t CpswAle_rejoin(EnetMod_Handle hMod,
                       Enet_Type enetType,
                       uint32_t instId)
{
    return ENET_SOK;
}

void CpswAle_close(EnetMod_Handle hMod)
{
    CpswAle_Handle hAle = (CpswAle_Handle)hMod;
    CSL_AleRegs *regs = (CSL_AleRegs *)hMod->virtAddr;
    uint32_t i;

    hAle->softTimerActive = false;

    CpswAle_delAllPolicerEntries(hAle, regs);
    CpswAle_delAllEntries(hAle, regs);

    for (i = 0U; i < hAle->numPorts; i++)
    {
        CpswAle_setAlePortState(regs, i, CSL_ALE_PORTSTATE_DISABLED);
    }

}

void CpswAle_saveCtxt(EnetMod_Handle hMod)
{
    CpswAle_close(hMod);
}

int32_t CpswAle_restoreCtxt(EnetMod_Handle hMod,
                            Enet_Type enetType,
                            uint32_t instId,
                            const void *cfg,
                            uint32_t cfgSize)
{
    int32_t status = ENET_SOK;

    CSL_AleRegs *regs = (CSL_AleRegs *)hMod->virtAddr;

    status = CpswAle_open(hMod, enetType, instId, cfg, cfgSize);
    if(status == ENET_SOK)
    {
        /* Setting ALE host port to Forward state */
        status = CpswAle_setAlePortState(regs, CPSW_ALE_HOST_PORT_NUM, CSL_ALE_PORTSTATE_FORWARD);
    }

    return status;
}

int32_t CpswAle_ioctl(EnetMod_Handle hMod,
                      uint32_t cmd,
                      Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

#if ENET_CFG_IS_ON(DEV_ERROR)
    /* Validate CPSW ALE IOCTL parameters */
    if (ENET_IOCTL_GET_PER(cmd) == ENET_IOCTL_PER_CPSW)
    {
        if (ENET_IOCTL_GET_TYPE(cmd) == ENET_IOCTL_TYPE_PUBLIC)
        {
            status = Enet_validateIoctl(cmd, prms,
                                        gCpswAle_ioctlValidate,
                                        ENET_ARRAYSIZE(gCpswAle_ioctlValidate));
        }
        else
        {
            status = Enet_validateIoctl(cmd, prms,
                                        gCpswAle_privIoctlValidate,
                                        ENET_ARRAYSIZE(gCpswAle_privIoctlValidate));
        }

        ENETTRACE_ERR_IF(status != ENET_SOK, "IOCTL 0x%08x params are not valid\n", cmd);
    }
#endif

    if (status == ENET_SOK)
    {
        CpswAleIoctlHandler * ioctlHandlerFxn;
        CpswAle_Handle hAle = (CpswAle_Handle)hMod;
        CSL_AleRegs *regs = (CSL_AleRegs *)hMod->virtAddr;

        ioctlHandlerFxn = CpswAle_getIoctlHandlerFxn(cmd, CpswAleIoctlHandlerRegistry, ENET_ARRAYSIZE(CpswAleIoctlHandlerRegistry));
        Enet_devAssert(ioctlHandlerFxn != NULL);
        status = ioctlHandlerFxn(hAle, regs, prms);
    }

    return status;
}


#if ENET_CFG_IS_ON(DEV_ERROR)
static int32_t CpswAle_isSupported(CSL_AleRegs *regs)
{
    CSL_CPSW_ALE_VERSION version;
    uint32_t i;
    int32_t status = ENET_ENOTSUPPORTED;

    CSL_CPSW_getAleVersionInfo(regs, &version);

    for (i = 0U; i < ENET_ARRAYSIZE(CpswAle_gSupportedVer); i++)
    {
        if ((version.majorVer == CpswAle_gSupportedVer[i].majorVer) &&
            (version.minorVer == CpswAle_gSupportedVer[i].minorVer) &&
            (version.rtlVer == CpswAle_gSupportedVer[i].rtlVer) &&
            (version.id == CpswAle_gSupportedVer[i].id))
        {
            status = ENET_SOK;
            break;
        }
    }

    return status;
}
#endif

uint32_t CpswAle_mapBw2IdlIncVal(uint32_t aleFreqHz,
                                 uint32_t rateInBps)
{
    uint64_t idlIncVal;

    /*
     * CIR or PIR policing inc value:
     *
     *              Policing rate (Mbps) * 32768
     * Inc value = -----------------------------
     *                  CPPI_ICLK freq (MHz)
     */
    idlIncVal = (uint64_t)rateInBps * CPSW_ALE_IDLINCVAL_DIV_FACTOR;
    idlIncVal /= aleFreqHz;

    return (uint32_t)idlIncVal;
}

static uint32_t CpswAle_mapAleAgingIntervalMsToAgingTimer(uint32_t aleFreqHz,
                                                          uint32_t aleAgingIntervalInMs)
{
    uint32_t agingTimer;

    agingTimer = (aleFreqHz / CPSW_ALE_AGING_TIMER_DIV_FACTOR) * aleAgingIntervalInMs;

    agingTimer /= 1000;

    /* Round up the aging timer register value if above division operation
     * truncated the timer value */
    if ((agingTimer % 1000) != 0U)
    {
        agingTimer++;
    }

    return agingTimer;
}

uint32_t CpswAle_getMaxPolicers(CSL_AleRegs *regs)
{
    uint32_t numPolicers;

    CSL_CPSW_getAleStatusNumPolicers(regs, &numPolicers);

    return numPolicers;
}

uint32_t CpswAle_getMaxAleEntries(CSL_AleRegs *regs)
{
    uint32_t numAleEntries;

    CSL_CPSW_getAleStatusNumAleEntries(regs, &numAleEntries);

    return numAleEntries;
}

static CSL_CPSW_ALETABLE_TYPE CpswAle_getTableType(Enet_Type enetType)
{
    CSL_CPSW_ALETABLE_TYPE tableType = CSL_CPSW_ALETABLE_TYPE_4PORT;

    switch (enetType)
    {
        case ENET_CPSW_2G:
            tableType = CSL_CPSW_ALETABLE_TYPE_4PORT;
            break;
        case ENET_CPSW_3G:
            tableType = CSL_CPSW_ALETABLE_TYPE_9PORT;
            break;
        default:
            Enet_assert(false, "Invalid Ethernet type %u\n", enetType);
            break;
    }

    return tableType;
}

static bool CpswAle_isVlanMaskMuxRegSupported(CSL_AleRegs *regs)
{
    bool vlanMsk08, vlanMsk12;

    CSL_CPSW_getAleStatusVlanMask(regs, &vlanMsk08, &vlanMsk12);

    return (!vlanMsk08 && !vlanMsk12);
}

static void CpswAle_setAleCfg(CpswAle_Handle hAle,
                              CSL_AleRegs *regs,
                              const CpswAle_Cfg *aleCfg)
{
    const CpswAle_PolicerGlobalCfg *polCfg = &aleCfg->policerGlobalCfg;
    const CpswAle_NetworkSecurityCfg *secCfg = &aleCfg->nwSecCfg;
    const CpswAle_InitVlanCfg *vlanCfg = &aleCfg->vlanCfg;
    bool macAuthDisable;
    uint32_t i = 0;

    hAle->aleFreqHz = EnetSoc_getClkFreq(hAle->enetType, hAle->instId, CPSW_CPPI_CLK);
    Enet_assert(hAle->aleFreqHz != 0,"Invalid ALE Frequency");

    CpswAle_initAleRegs(regs);

    CpswAle_setAleModeFlags(regs, aleCfg->modeFlags);

    CpswAle_setAleAging(regs,
                        &aleCfg->agingCfg,
                        hAle->aleFreqHz,
                        &hAle->softTimerActive,
                        &hAle->tickTimeoutCnt,
                        &hAle->softTickCnt);

    CpswAle_setUnknownVlanCfg(regs,
                              vlanCfg->unknownVlanMemberListMask,
                              vlanCfg->unknownForceUntaggedEgressMask,
                              vlanCfg->unknownRegMcastFloodMask,
                              vlanCfg->unknownUnregMcastFloodMask);

    CpswAle_setPolicerControl(regs,
                              hAle->aleFreqHz,
                              polCfg->policingEn,
                              polCfg->yellowDropEn,
                              polCfg->redDropEn,
                              polCfg->yellowThresh,
                              polCfg->policerNoMatchMode,
                              &polCfg->noMatchPolicer);

    /* Clear all policers on init */
    CpswAle_clearGlobalPolicerStats(regs, true, true, true);

    /* By default disable default thread enable */
    CpswAle_setPolicerDefaultThreadCfg(hAle, regs, false, 0U, false, false);

    /* Configure ALE port control register */
    for (i = 0U; i < hAle->numPorts; i++)
    {
        macAuthDisable = ((secCfg->macAuthCfg.macAuthDisMask & (1U << i)) == 1U);
        CpswAle_setAlePortCfg(regs, i, &aleCfg->portCfg[i], macAuthDisable);
        hAle->pvid[i] = aleCfg->portCfg[i].pvidCfg;
    }

    CpswAle_setNetSecCfg(regs, secCfg);
    CpswAle_initVlanCfg(regs, vlanCfg);
    hAle->pvidEn = vlanCfg->cpswVlanAwareMode;
    hAle->rxFilter = CPSW_ALE_RXFILTER_DIRECT;
}

static int32_t CpswAle_addDefaultEntries(CpswAle_Handle hAle)
{
    EnetMod_Handle hMod = (EnetMod_Handle)hAle;
    CSL_AleRegs *regs = (CSL_AleRegs *)hMod->virtAddr;
    CpswAle_PortVlanCfg *pvidCfg;
    uint32_t entryIdx;
    uint32_t i;
    int32_t status = ENET_SOK;

    if (hAle->pvidEn &&
        CSL_CPSW_isAleVlanAwareEnabled(regs))
    {
        /* Add default VLAN entry for each port */
        for (i = 0U; i < hAle->numPorts; i++)
        {
            pvidCfg = &hAle->pvid[i];

            status = CpswAle_addVlan(hAle,
                                     regs,
                                     pvidCfg->vlanIdInfo.vlanId,
                                     pvidCfg->vlanIdInfo.tagType == ENET_VLAN_TAG_TYPE_OUTER,
                                     pvidCfg->vlanMemberList,
                                     pvidCfg->unregMcastFloodMask,
                                     pvidCfg->regMcastFloodMask,
                                     pvidCfg->forceUntaggedEgressMask,
                                     pvidCfg->noLearnMask,
                                     pvidCfg->vidIngressCheck,
                                     pvidCfg->limitIPNxtHdr,
                                     pvidCfg->disallowIPFrag,
                                     &entryIdx);
            if (status != ENET_SOK)
            {
                break;
            }
        }
    }

    return status;
}


static void CpswAle_initAleRegs(CSL_AleRegs *regs)
{
    uint32_t aleCtlVal = 0;

    /* On ALE open, init ALE control register to known state */
    aleCtlVal |= CSL_FMK(ALE_ALE_CONTROL_CLEAR_TABLE, 1U);
    aleCtlVal |= CSL_FMK(ALE_ALE_CONTROL_AGE_OUT_NOW, 1U);
    aleCtlVal |= CSL_FMK(ALE_ALE_CONTROL_BCAST_MCAST_CTL, 0U);
    aleCtlVal |= CSL_FMK(ALE_ALE_CONTROL_ENABLE_RATE_LIMIT, 0U);
    aleCtlVal |= CSL_FMK(ALE_ALE_CONTROL_UPD_STATIC, 0U);
    aleCtlVal |= CSL_FMK(ALE_ALE_CONTROL_UPD_BW_CTRL, 0U);
    aleCtlVal |= CSL_FMK(ALE_ALE_CONTROL_UPD_STATIC, 0U);

    /* Set ALE control register */
    CSL_CPSW_setAleControlReg(regs, aleCtlVal);
    CSL_CPSW_setAlePrescaleReg(regs, CPSW_ALE_CFG_BCAST_MCAST_PRESCALE);
}

static void CpswAle_setAleModeFlags(CSL_AleRegs *regs,
                                    uint32_t modeFlags)
{
    uint32_t aleCtlVal;

    aleCtlVal = CSL_CPSW_getAleControlReg(regs);
    if ((modeFlags & CPSW_ALE_CFG_MODULE_EN) != 0U)
    {
        aleCtlVal |= CSL_FMK(ALE_ALE_CONTROL_ENABLE_ALE, 1U);
    }
    else
    {
        aleCtlVal |= CSL_FMK(ALE_ALE_CONTROL_ENABLE_ALE, 0U);
    }

    if ((modeFlags & CPSW_ALE_CFG_BYPASS_EN) != 0U)
    {
        aleCtlVal |= CSL_FMK(ALE_ALE_CONTROL_ENABLE_BYPASS, 1U);
    }
    else
    {
        aleCtlVal |= CSL_FMK(ALE_ALE_CONTROL_ENABLE_BYPASS, 0U);
    }

    if ((modeFlags & CPSW_ALE_CFG_UNKNOWN_UCAST_FLOOD2HOST) != 0U)
    {
        aleCtlVal |= CSL_FMK(ALE_ALE_CONTROL_EN_HOST_UNI_FLOOD, 1U);
    }
    else
    {
        aleCtlVal |= CSL_FMK(ALE_ALE_CONTROL_EN_HOST_UNI_FLOOD, 0U);
    }

    /* Update ALE control register */
    CSL_CPSW_setAleControlReg(regs, aleCtlVal);
}

static void CpswAle_setAleAging(CSL_AleRegs *regs,
                                const CpswAle_AgingCfg *agingCfg,
                                uint32_t aleFreqHz,
                                bool *softTimerActive,
                                uint32_t *tickTimeoutCnt,
                                uint32_t *softTickCnt)
{
    uint32_t agingPeriod;

    if (agingCfg->autoAgingEn)
    {
        agingPeriod = CpswAle_mapAleAgingIntervalMsToAgingTimer(aleFreqHz, agingCfg->agingPeriodInMs);

        CSL_CPSW_setAleAgingTimerReg(regs, CSL_ALE_AGT_PRESACLE_1M, agingPeriod);
        *softTimerActive = false;
    }
    else
    {
        *softTimerActive = true;
        *tickTimeoutCnt = agingCfg->agingPeriodInMs;
        *softTickCnt = 0U;
    }
}

static void CpswAle_setUnknownVlanCfg(CSL_AleRegs *regs,
                                      uint32_t memberList,
                                      uint32_t forceUntaggedEgress,
                                      uint32_t regMcastFloodMask,
                                      uint32_t unregMcastFloodMask)
{
    uint32_t val;

    val = CSL_FMK(ALE_ALE_UVLAN_MEMBER_UVLAN_MEMBER_LIST, memberList);
    CSL_CPSW_setAleUnknwnVlanMemberReg(regs, val);

    val = CSL_FMK(ALE_ALE_UVLAN_UNTAG_UVLAN_FORCE_UNTAGGED_EGRESS, forceUntaggedEgress);
    CSL_CPSW_setAleUnknwnVlanUntagReg(regs, val);

    val = CSL_FMK(ALE_ALE_UVLAN_RMCAST_UVLAN_REG_MCAST_FLOOD_MASK, regMcastFloodMask);
    CSL_CPSW_setAleUnknwnVlanRegMcastReg(regs, val);

    val = CSL_FMK(ALE_ALE_UVLAN_URCAST_UVLAN_UNREG_MCAST_FLOOD_MASK, unregMcastFloodMask);
    CSL_CPSW_setAleUnknwnVlanUnregMcastReg(regs, val);
}

int32_t CpswAle_setPolicerControl(CSL_AleRegs *regs,
                                  uint32_t aleFreqHz,
                                  bool policingEn,
                                  bool yellowDropEn,
                                  bool redDropEn,
                                  CpswAle_PolicerYellowThresh yellowThresh,
                                  CpswAle_PolicerNoMatchMode policerMatchMode,
                                  const CpswAle_UnregulatedTrafficPolicer *noMatchPolicer)
{
    CSL_CPSW_ALE_POLICER_CONTROL policerControl;
    int32_t status = ENET_SOK;

    if (CPSW_ALE_POLICER_NOMATCH_MODE_UNREGULATED_TRAFFIC_POLICER == policerMatchMode)
    {
        status = CpswAle_setNoMatchEntry0Policer(regs, aleFreqHz,
                                                 noMatchPolicer->peakRateInBitsPerSec,
                                                 noMatchPolicer->commitRateInBitsPerSec);
    }

    if (status == ENET_SOK)
    {
        CSL_CPSW_getAlePolicerControlReg(regs, &policerControl);

        policerControl.policingEnable   = policingEn;
        policerControl.redDropEnable    = redDropEn;
        policerControl.yellowDropEnable = yellowDropEn;
        policerControl.yellowDropThresh = (CSL_CPSW_ALE_POLICER_CONTROL_YELLOWTHRESH)yellowThresh;
        policerControl.policeMatchMode  = (CSL_CPSW_ALE_POLICER_CONTROL_POLICING_MATCH_MODE)policerMatchMode;

        CSL_CPSW_setAlePolicerControlReg(regs, &policerControl);
    }

    return status;
}

static int32_t CpswAle_setNoMatchEntry0Policer(CSL_AleRegs *regs,
                                               uint32_t aleFreqHz,
                                               uint32_t peakRateInBitsPs,
                                               uint32_t commitRateInBitsPs)
{
    CSL_CPSW_ALE_POLICER_ENTRY polCfg;
    uint32_t pir_idl_inc_val = CpswAle_mapBw2IdlIncVal(aleFreqHz, peakRateInBitsPs);
    uint32_t cir_idl_inc_val = CpswAle_mapBw2IdlIncVal(aleFreqHz, commitRateInBitsPs);
    int32_t status = ENET_SOK;

    CSL_CPSW_getAlePolicerEntry(regs, CPSW_ALE_NOMATCH_ENTRY0_POLICER_INDEX, &polCfg);

    /* Confirm entry 0 is free. If entry 0 is not free then, ALE open sequence must be
     * modified to ensure entry 0 is free to allow configuration of NO_MATCH policer */
    if (polCfg.validBitmap == 0U)
    {
        polCfg.validBitmap   = (CSL_CPSW_ALE_POLICER_PIR_VALID | CSL_CPSW_ALE_POLICER_CIR_VALID);
        polCfg.pirIdleIncVal = pir_idl_inc_val;
        polCfg.cirIdleIncVal = cir_idl_inc_val;

        CSL_CPSW_setAlePolicerEntry(regs, CPSW_ALE_NOMATCH_ENTRY0_POLICER_INDEX, &polCfg);
        CpswAle_clearSelectedPolicerStats(regs, CPSW_ALE_NOMATCH_ENTRY0_POLICER_INDEX);
    }
    else
    {
        ENETTRACE_ERR("Entry 0 is not free, NO_MATCH policer can't be configured\n");
        status = ENET_EFAIL;
    }

    return status;
}

void CpswAle_clearSelectedPolicerStats(CSL_AleRegs *regs,
                                              uint32_t policerIdx)
{
    CSL_CPSW_ALE_POLICER_TEST_CONTROL policerTstCtrl;

    policerTstCtrl.polClrallHit       = 0U;
    policerTstCtrl.polClrallRedhit    = 0U;
    policerTstCtrl.polClrallYellowhit = 0U;
    policerTstCtrl.polClrselAll       = 1U;
    policerTstCtrl.polTestIdx         = policerIdx; /* Don't care param */
    CSL_CPSW_setAlePolicerTestControlReg(regs, &policerTstCtrl);
}

static void CpswAle_clearGlobalPolicerStats(CSL_AleRegs *regs,
                                     bool clearAllHit,
                                     bool clearRedHit,
                                     bool clearYellowHit)
{
    CSL_CPSW_ALE_POLICER_TEST_CONTROL policerTstCtrl;

    policerTstCtrl.polClrallHit       = clearAllHit;
    policerTstCtrl.polClrallRedhit    = clearRedHit;
    policerTstCtrl.polClrallYellowhit = clearYellowHit;
    policerTstCtrl.polClrselAll       = 0U;
    policerTstCtrl.polTestIdx         = 0U; /* Don't care param */

    CSL_CPSW_setAlePolicerTestControlReg(regs, &policerTstCtrl);
}

int32_t CpswAle_setPolicerDefaultThreadCfg(CpswAle_Handle hAle,
                                           CSL_AleRegs *regs,
                                           bool threadEn,
                                           uint32_t threadVal,
                                           bool priorityOREn,
                                           bool macPortDefaultThreadDis)
{
    EnetMod_Handle hMod = (EnetMod_Handle)hAle;
    CSL_CPSW_ALE_POLICER_GLOB_CONFIG defThreadCfg;
    CSL_CPSW_ALE_POLICER_CONTROL policerControl;

    defThreadCfg.defThreadEnable = threadEn;
    defThreadCfg.defThread = threadVal;

    CSL_CPSW_setAlePolicerGlobConfig(regs, &defThreadCfg);

    if (ENET_FEAT_IS_EN(hMod->features, CPSW_ALE_FEATURE_FLOW_PRIORITY))
    {
        /* disableMacPort and priorityOrEn are part of policer control register */
        CSL_CPSW_getAlePolicerControlReg(regs, &policerControl);

        policerControl.disableMacPortDefaultThread = macPortDefaultThreadDis;
        policerControl.enablePriorityOR = priorityOREn;

        CSL_CPSW_setAlePolicerControlReg(regs, &policerControl);
    }

    return ENET_SOK;
}

static void CpswAle_setAlePortCfg(CSL_AleRegs *regs,
                                  uint32_t portNum,
                                  const CpswAle_PortCfg *portCfg,
                                  bool macAuthForPortDis)
{
    CSL_CPSW_ALE_PORTCONTROL portControl;

    CSL_CPSW_getAlePortControlReg(regs, portNum, &portControl);

    portControl.macAuthDisable        = macAuthForPortDis;
    portControl.dropDualVlan          = portCfg->vlanCfg.dropDualVlan;
    portControl.dropDoubleVlan        = portCfg->vlanCfg.dropDoubleVlan;
    portControl.vidIngressCheckEnable = portCfg->vlanCfg.vidIngressCheck;
    portControl.dropUntaggedEnable    = portCfg->vlanCfg.dropUntagged;

    portControl.noSaUpdateEnable      = portCfg->learningCfg.noSaUpdateEn;
    portControl.noLearnModeEnable     = portCfg->learningCfg.noLearn;

    portControl.macOnlyEnable         = portCfg->macModeCfg.macOnlyEn;
    portControl.macOnlyCafEnable      = portCfg->macModeCfg.macOnlyCafEn;

    /* At time of open, set ALE port state in disabled mode.
     * ALE port must be enabled only after MAC config and PHY link up */
    portControl.portState = CSL_ALE_PORTSTATE_DISABLED;

    CSL_CPSW_setAlePortControlReg(regs, portNum, &portControl);
}

static void CpswAle_setNetSecCfg(CSL_AleRegs *regs,
                                 const CpswAle_NetworkSecurityCfg *netSecCfg)
{
    CSL_CPSW_ALE_CTRL2_MALFORMEDFRAME_CONFIG badPkt;
    uint32_t val;

    val = CSL_CPSW_getAleControlReg(regs);
    CSL_FINS(val, ALE_ALE_CONTROL_ENABLE_VID0_MODE, netSecCfg->vid0ModeEn);
    CSL_FINS(val, ALE_ALE_CONTROL_ENABLE_OUI_DENY,  netSecCfg->hostOuiNoMatchDeny);
    CSL_FINS(val, ALE_ALE_CONTROL_ENABLE_AUTH_MODE, netSecCfg->macAuthCfg.authModeEn);
    CSL_CPSW_setAleControlReg(regs, val);

    badPkt.dropBadLen = netSecCfg->malformedPktCfg.badLenPktDropEn;
    badPkt.noDropSrcMcast = netSecCfg->malformedPktCfg.srcMcastDropDis;
    CSL_CPSW_setAleCtrl2MalformedFrameConfig(regs, &badPkt);

    CpswAle_setIPFilterCfg(regs, &netSecCfg->ipPktCfg);
}

static void CpswAle_setIPFilterCfg(CSL_AleRegs *regs,
                                   const CpswAle_IPPktSecurityCfg *ipSecCfg)
{
    CSL_CPSW_ALE_CTRL2_IPPKTFLT_CONFIG ipPkt;
    uint8_t ipNxtHdr[CPSW_ALE_MAX_WHITELIST_IP_NXT_HDR];
    uint32_t i;

    ipPkt.ipPktFltEnableDefNoFrag = ipSecCfg->dfltNoFragEn;
    ipPkt.ipPktFltEnableDefNxtHdrLimit = ipSecCfg->dfltNxtHdrWhitelistEn;

    CSL_CPSW_setAleCtrl2IPPktFilterConfig(regs, &ipPkt);

    if (ipSecCfg->ipNxtHdrWhitelistCnt > 0U)
    {
        /* If configuring less than 4 IP NXT HDRs, the NXT hdrs should be replicated
         * to ensure valid values present in NXT_HDR_0/1/2/3.
         * So first replicate application passed NXT_HDR_0 and then set the app passed
         * NXT hdr values */
        for (i = 0U; i < CPSW_ALE_MAX_WHITELIST_IP_NXT_HDR; i++)
        {
            ipNxtHdr[i] = ipSecCfg->ipNxtHdrWhitelist[0U];
        }

        for (i = 0U; (i < ipSecCfg->ipNxtHdrWhitelistCnt) && (i < CPSW_ALE_MAX_WHITELIST_IP_NXT_HDR); i++)
        {
            ipNxtHdr[i] = ipSecCfg->ipNxtHdrWhitelist[i];
        }

        CSL_CPSW_setAleIPNxtHdrWhitelist(regs, ipNxtHdr[0U], ipNxtHdr[1U], ipNxtHdr[2U], ipNxtHdr[3U]);
    }
}

static void CpswAle_initVlanCfg(CSL_AleRegs *regs,
                                const CpswAle_InitVlanCfg *vlanCfg)
{
    uint32_t val;

    if (vlanCfg->aleVlanAwareMode)
    {
        CSL_CPSW_enableAleVlanAware(regs);
    }
    else
    {
        CSL_CPSW_disableAleVlanAware(regs);
    }

    val = CSL_CPSW_getAleControlReg(regs);

    CSL_FINS(val, ALE_ALE_CONTROL_UVLAN_NO_LEARN, vlanCfg->unknownVlanNoLearn);
    if (vlanCfg->autoLearnWithVlan)
    {
        CSL_FINS(val, ALE_ALE_CONTROL_LEARN_NO_VLANID, 0U);
    }
    else
    {
        CSL_FINS(val, ALE_ALE_CONTROL_LEARN_NO_VLANID, 1U);
    }

    CSL_CPSW_setAleControlReg(regs, val);

    CpswAle_setUnknownVlanCfg(regs,
                              vlanCfg->unknownVlanMemberListMask,
                              vlanCfg->unknownForceUntaggedEgressMask,
                              vlanCfg->unknownRegMcastFloodMask,
                              vlanCfg->unknownUnregMcastFloodMask);
}

void CpswAle_getVlanMcastPortMask(CSL_AleRegs *regs,
                                         const CSL_CPSW_ALE_VLAN_ENTRY *vlanEntry,
                                         uint32_t *regMcastFloodMask,
                                         uint32_t *unregMcastFloodMask)
{
#if ENET_CFG_IS_ON(ALE_VLAN_MASK_MUX)
    if (CpswAle_isVlanMaskMuxRegSupported(regs))
    {
        CSL_CPSW_getAleVlanMaskMuxEntryReg(regs, vlanEntry->unRegMcastFloodIndex, unregMcastFloodMask);
        CSL_CPSW_getAleVlanMaskMuxEntryReg(regs, vlanEntry->regMcastFloodIndex, regMcastFloodMask);
    }
#else
    {
        *unregMcastFloodMask = vlanEntry->unRegMcastFloodMask;
        *regMcastFloodMask = vlanEntry->regMcastFloodMask;
    }
#endif	
}

int32_t CpswAle_setVlanMcastPortMask(CpswAle_Handle hAle,
                                     CSL_AleRegs *regs,
                                     CSL_CPSW_ALE_VLAN_ENTRY *vlanEntry,
                                     uint32_t entryIdx,
                                     bool isOuterVlan,
                                     uint32_t regMcastFloodMask,
                                     uint32_t unregMcastFloodMask)
{
    CSL_CPSW_ALETABLE_TYPE tableType = hAle->tableType;
    int32_t status = ENET_SOK;

#if ENET_CFG_IS_ON(ALE_VLAN_MASK_MUX)
    if (CpswAle_isVlanMaskMuxRegSupported(regs))
    {
        status = CpswAle_addVlanMask(hAle, regs, regMcastFloodMask, &vlanEntry->regMcastFloodIndex);
        if (status == ENET_SOK)
        {
            status = CpswAle_addVlanMask(hAle, regs, unregMcastFloodMask, &vlanEntry->unRegMcastFloodIndex);
        }
    }
#else
    {
        vlanEntry->unRegMcastFloodMask = unregMcastFloodMask;
        vlanEntry->regMcastFloodMask = regMcastFloodMask;
    }
#endif

    if (status == ENET_SOK)
    {
        if (isOuterVlan)
        {
            CSL_CPSW_setAleOutVlanEntry(regs, entryIdx, vlanEntry, tableType);
        }
        else
        {
            CSL_CPSW_setAleVlanEntry(regs, entryIdx, vlanEntry, tableType);
        }
    }

    return status;
}

int32_t CpswAle_addVlan(CpswAle_Handle hAle,
                        CSL_AleRegs *regs,
                        uint32_t vlanId,
                        bool isOuterVlan,
                        uint32_t vlanMemberList,
                        uint32_t unregMcastFloodMask,
                        uint32_t regMcastFloodMask,
                        uint32_t forceUntaggedEgress,
                        uint32_t noLearnMask,
                        bool vidIngressCheck,
                        bool limitIPNxtHdr,
                        bool disallowIPFrag,
                        uint32_t *entryIdx)
{
    uint32_t tableDepth = CpswAle_getMaxAleEntries(regs);
    bool vlanAware;
    uint32_t freeEntry = 0;
    uint32_t vlanEntry = 0;
    int32_t status = ENET_SOK;

    /* Check the current setting for vlanAware */
    vlanAware = (CSL_CPSW_isAleVlanAwareEnabled(regs) == TRUE);
    if (!vlanAware)
    {
        ENETTRACE_ERR("Not allowed to add VLAN entry in VLAN unaware mode\n");
        status = ENET_EPERM;
    }

    if (status == ENET_SOK)
    {
        vlanId              &= (uint32_t)CPSW_ALE_VLAN_ID_MASK;
        vlanMemberList      &= (uint32_t)CPSW_ALE_ALL_PORTS_MASK;
        unregMcastFloodMask &= (uint32_t)CPSW_ALE_ALL_PORTS_MASK;
        regMcastFloodMask   &= (uint32_t)CPSW_ALE_ALL_PORTS_MASK;
        forceUntaggedEgress &= (uint32_t)CPSW_ALE_ALL_PORTS_MASK;
        noLearnMask         &= (uint32_t)CPSW_ALE_ALL_PORTS_MASK;

        status = CpswAle_findVlan(hAle,
                                  regs,
                                  vlanId,
                                  isOuterVlan,
                                  NULL,
                                  NULL,
                                  NULL,
                                  NULL,
                                  NULL,
                                  NULL,
                                  NULL,
                                  NULL,
                                  &vlanEntry);
        if ((status != ENET_ENOTFOUND) &&
            (vlanEntry < tableDepth))
        {
            /* Update existing entry */
            freeEntry = vlanEntry;
        }
        else
        {
            /* Update VLAN Info */
            status = CpswAle_getFreeEntry(hAle, regs, &freeEntry);
        }

        ENETTRACE_ERR_IF(status == ENET_EALLOC, "ALE table is full\n");

        /* If ALE table is not full */
        if (status != ENET_EALLOC)
        {
            if (status == ENET_SOK)
            {
                if (!isOuterVlan)
                {
                    CSL_CPSW_ALE_VLAN_ENTRY vlanCfg;

                    vlanCfg.forceUntaggedEgress     = forceUntaggedEgress;
                    vlanCfg.vlanMemList             = vlanMemberList;
                    vlanCfg.vlanId                  = vlanId;
                    vlanCfg.noLearnMask             = noLearnMask;
                    vlanCfg.ingressCheckFlag        = vidIngressCheck;
                    vlanCfg.limitIPNxtHdr           = limitIPNxtHdr;
                    vlanCfg.disallowIPFragmentation = disallowIPFrag;

                    status = CpswAle_setVlanMcastPortMask(hAle,
                                                          regs,
                                                          &vlanCfg,
                                                          freeEntry,
                                                          false,
                                                          regMcastFloodMask,
                                                          unregMcastFloodMask);
                    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to add VLAN entry: %d\n", status);
                }
                else
                {
                    CSL_CPSW_ALE_OUTER_VLAN_ENTRY oVlanCfg;

                    oVlanCfg.forceUntaggedEgress     = forceUntaggedEgress;
                    oVlanCfg.vlanMemList             = vlanMemberList;
                    oVlanCfg.vlanId                  = vlanId;
                    oVlanCfg.noLearnMask             = noLearnMask;
                    oVlanCfg.ingressCheckFlag        = vidIngressCheck;
                    oVlanCfg.limitIPNxtHdr           = limitIPNxtHdr;
                    oVlanCfg.disallowIPFragmentation = disallowIPFrag;

                    status = CpswAle_setVlanMcastPortMask(hAle,
                                                          regs,
                                                          &oVlanCfg,
                                                          freeEntry,
                                                          true,
                                                          regMcastFloodMask,
                                                          unregMcastFloodMask);
                    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to add outer VLAN entry: %d\n", status);
                }

                CPSW_ALE_SAFE_ASSIGN(entryIdx, freeEntry);
            }
        }
    }

    return status;
}

int32_t CpswAle_findVlan(CpswAle_Handle hAle,
                         CSL_AleRegs *regs,
                         uint32_t vlanId,
                         bool isOuterVlan,
                         uint32_t *vlanMemberList,
                         uint32_t *unregMcastFloodMask,
                         uint32_t *regMcastFloodMask,
                         uint32_t *forceUntaggedEgress,
                         uint32_t *noLearnMask,
                         bool *vidIngressCheck,
                         bool *limitIpNxtHdr,
                         bool *disallowFrag,
                         uint32_t *entryIdx)
{
    EnetMod_Handle hMod = (EnetMod_Handle)hAle;
    CSL_CPSW_ALETABLE_TYPE tableType = hAle->tableType;
    CSL_CPSW_ALE_POLICER_ENTRYTYPE polEntryType;
    uint32_t tableDepth = CpswAle_getMaxAleEntries(regs);
    bool matchIngressCheck = false;
    bool matchLimitIpNxtHdr = false;
    bool matchDisallowFrag = false;
    bool matchFound;
    uint32_t matchVlanMemberList = 0U;
    uint32_t matchUnregMcastFloodMask = 0U;
    uint32_t matchRegMcastFloodMask = 0U;
    uint32_t matchForceUntaggedEgress = 0U;
    uint32_t matchNoLearnMask = 0U;
    uint32_t val;
    uint32_t i;
    int32_t status = ENET_ENOTFOUND;

    matchFound = false;

    for (i = 0U; i < tableDepth; i++)
    {
        val = CSL_CPSW_getALEEntryType(regs, i, tableType);
        if (val == CSL_ALE_ENTRYTYPE_VLAN)
        {
            polEntryType = CSL_CPSW_getALEPolicerEntryType(regs, i, tableType);
            if (isOuterVlan)
            {
                if (polEntryType == CSL_ALE_POLICER_ENTRYTYPE_OVLAN)
                {
                    CSL_CPSW_ALE_OUTER_VLAN_ENTRY oVlanEntry = {0};

                    CSL_CPSW_getAleOutVlanEntry(regs, i, &oVlanEntry, tableType);
                    if (oVlanEntry.vlanId == vlanId)
                    {
                        matchFound                 = true;
                        matchVlanMemberList        = oVlanEntry.vlanMemList;
                        matchForceUntaggedEgress   = oVlanEntry.forceUntaggedEgress;
                        matchNoLearnMask           = oVlanEntry.noLearnMask;
                        matchIngressCheck          = oVlanEntry.ingressCheckFlag;
                        matchLimitIpNxtHdr         = oVlanEntry.limitIPNxtHdr;
                        matchDisallowFrag          = oVlanEntry.disallowIPFragmentation;
                        CpswAle_getVlanMcastPortMask(regs, &oVlanEntry,
                                                     &matchRegMcastFloodMask,
                                                     &matchUnregMcastFloodMask);
                        break;
                    }
                }
            }
            else
            {
                if (polEntryType == CSL_ALE_POLICER_ENTRYTYPE_VLAN)
                {
                    CSL_CPSW_ALE_VLAN_ENTRY vlanEntry;

                    CSL_CPSW_getAleVlanEntry(regs, i, &vlanEntry, tableType);
                    if (vlanEntry.vlanId == vlanId)
                    {
                        matchFound               = true;
                        matchVlanMemberList      = vlanEntry.vlanMemList;
                        matchForceUntaggedEgress = vlanEntry.forceUntaggedEgress;
                        matchNoLearnMask         = vlanEntry.noLearnMask;
                        matchIngressCheck        = vlanEntry.ingressCheckFlag;
                        if (ENET_FEAT_IS_EN(hMod->features, CPSW_ALE_FEATURE_IP_HDR_WHITELIST))
                        {
                            matchLimitIpNxtHdr = vlanEntry.limitIPNxtHdr;
                            matchDisallowFrag  = vlanEntry.disallowIPFragmentation;
                        }
                        else
                        {
                            matchLimitIpNxtHdr = false;
                            matchDisallowFrag = false;
                        }

                        CpswAle_getVlanMcastPortMask(regs,
                                                     &vlanEntry,
                                                     &matchRegMcastFloodMask,
                                                     &matchUnregMcastFloodMask);

                        break;
                    }
                }
            }
        }
    }

    if (matchFound)
    {
        CPSW_ALE_SAFE_ASSIGN(vlanMemberList, matchVlanMemberList);
        CPSW_ALE_SAFE_ASSIGN(unregMcastFloodMask, matchUnregMcastFloodMask);
        CPSW_ALE_SAFE_ASSIGN(regMcastFloodMask, matchRegMcastFloodMask);
        CPSW_ALE_SAFE_ASSIGN(forceUntaggedEgress, matchForceUntaggedEgress);
        CPSW_ALE_SAFE_ASSIGN(noLearnMask, matchNoLearnMask);
        CPSW_ALE_SAFE_ASSIGN(vidIngressCheck, matchIngressCheck);
        CPSW_ALE_SAFE_ASSIGN(limitIpNxtHdr, matchLimitIpNxtHdr);
        CPSW_ALE_SAFE_ASSIGN(disallowFrag, matchDisallowFrag);
        CPSW_ALE_SAFE_ASSIGN(entryIdx, i);
        status = ENET_SOK;
    }

    return status;
}

int32_t CpswAle_getFreeEntry(CpswAle_Handle hAle,
                             CSL_AleRegs *regs,
                             uint32_t *entryIdx)
{
    CSL_CPSW_ALETABLE_TYPE tableType = hAle->tableType;
    CSL_CPSW_ALE_ENTRYTYPE entryType;
    uint32_t tableDepth = CpswAle_getMaxAleEntries(regs);
    uint32_t i;
    int32_t status = ENET_SOK;

    for (i = 0U; i < tableDepth; i++)
    {
        entryType = CSL_CPSW_getALEEntryType(regs, i, tableType);
        if (entryType == CSL_ALE_ENTRYTYPE_FREE)
        {
            break;
        }
    }

    if (i == tableDepth)
    {
        status = ENET_EALLOC;
    }
    else
    {
        *entryIdx = i;
    }

    return status;
}

int32_t CpswAle_clearTableEntry(CSL_AleRegs *regs,
                                       uint32_t entryIdx)
{
    int32_t status = ENET_EINVALIDPARAMS;

    if (entryIdx <= CpswAle_getMaxAleEntries(regs))
    {
        CSL_CPSW_clearAleEntry(regs, entryIdx);
        status = ENET_SOK;
    }

    return status;
}

int32_t CpswAle_setAlePortState(CSL_AleRegs *regs,
                                uint32_t port_num,
                                CSL_CPSW_ALE_PORTSTATE portState)
{
    CSL_CPSW_setAlePortState(regs, port_num, portState);
    return ENET_SOK;
}



static bool CpswAle_delIPv6HighEntry(CpswAle_Handle hAle,
                                     CSL_AleRegs *regs,
                                     uint32_t entryIdx)
{
    CSL_CPSW_ALETABLE_TYPE tableType = hAle->tableType;
    CSL_CPSW_ALE_ENTRYTYPE entryType;
    bool ipv6EntryDeleted = false;

    entryType = CSL_CPSW_getALEEntryType(regs, entryIdx, tableType);
    if (entryType == CSL_ALE_ENTRYTYPE_POLICER)
    {
        if (CSL_CPSW_getALEPolicerEntryType(regs, entryIdx, tableType) == CSL_ALE_POLICER_ENTRYTYPE_IPV6)
        {
            CpswAle_clearTableEntry(regs, CSL_CPSW_getAleIPv6HighEntryIndex(regs, entryIdx));
            CpswAle_clearTableEntry(regs, CSL_CPSW_getAleIPv6LowEntryIndex(regs, entryIdx));
            ipv6EntryDeleted = true;
        }
    }

    return ipv6EntryDeleted;
}

int32_t CpswAle_delPolicerEntry(CpswAle_Handle hAle,
                                       CSL_AleRegs *regs,
                                       uint32_t policerIdx,
                                       uint32_t delAleEntryMask)
{
    CSL_CPSW_ALE_POLICER_ENTRY policerEntry;
    uint32_t numPolicerEntries;
    uint32_t aleDelEntryIndex;
    int32_t status = ENET_SOK;

    CSL_CPSW_getAleStatusNumPolicers(regs, &numPolicerEntries);

    if (policerIdx >= numPolicerEntries)
    {
        status = ENET_EINVALIDPARAMS;
    }

    if (status == ENET_SOK)
    {
        CpswAle_clearSelectedPolicerStats(regs, policerIdx);

        CSL_CPSW_getAlePolicerEntry(regs, policerIdx, &policerEntry);

        if ((policerEntry.validBitmap & CSL_CPSW_ALE_POLICER_OUI_VALID) &&
            (delAleEntryMask & CPSW_ALE_POLICER_TABLEENTRY_DELETE_OUI))
        {
            aleDelEntryIndex = policerEntry.ouiIdx;
            CpswAle_clearTableEntry(regs, aleDelEntryIndex);
        }

        if ((policerEntry.validBitmap & CSL_CPSW_ALE_POLICER_DST_MAC_VALID) &&
            (delAleEntryMask & CPSW_ALE_POLICER_TABLEENTRY_DELETE_MACDST))
        {
            aleDelEntryIndex = policerEntry.dstMacIdx;
            CpswAle_clearTableEntry(regs, aleDelEntryIndex);
        }

        if ((policerEntry.validBitmap & CSL_CPSW_ALE_POLICER_SRC_MAC_VALID) &&
            (delAleEntryMask & CPSW_ALE_POLICER_TABLEENTRY_DELETE_MACSRC))
        {
            aleDelEntryIndex = policerEntry.srcMacIdx;
            CpswAle_clearTableEntry(regs, aleDelEntryIndex);
        }

        if ((policerEntry.validBitmap & CSL_CPSW_ALE_POLICER_OVLAN_VALID) &&
            (delAleEntryMask & CPSW_ALE_POLICER_TABLEENTRY_DELETE_OVLAN))
        {
            aleDelEntryIndex = policerEntry.ovlanIdx;
            CpswAle_clearTableEntry(regs, aleDelEntryIndex);
        }

        if ((policerEntry.validBitmap & CSL_CPSW_ALE_POLICER_VLAN_VALID) &&
            (delAleEntryMask & CPSW_ALE_POLICER_TABLEENTRY_DELETE_IVLAN))
        {
            aleDelEntryIndex = policerEntry.vlanIdx;
            CpswAle_clearTableEntry(regs, aleDelEntryIndex);
        }

        if ((policerEntry.validBitmap & CSL_CPSW_ALE_POLICER_ETHERTYPE_VALID) &&
            (delAleEntryMask & CPSW_ALE_POLICER_TABLEENTRY_DELETE_ETHERTYPE))
        {
            aleDelEntryIndex = policerEntry.ethertypeIdx;
            CpswAle_clearTableEntry(regs, aleDelEntryIndex);
        }

        if ((policerEntry.validBitmap & CSL_CPSW_ALE_POLICER_SRC_IP_VALID) &&
            (delAleEntryMask & CPSW_ALE_POLICER_TABLEENTRY_DELETE_IPSRC))
        {
            aleDelEntryIndex = policerEntry.srcIpIdx;

            if (CpswAle_delIPv6HighEntry(hAle, regs, aleDelEntryIndex) == FALSE)
            {
                CpswAle_clearTableEntry(regs, aleDelEntryIndex);
            }
        }

        if ((policerEntry.validBitmap & CSL_CPSW_ALE_POLICER_DST_IP_VALID) &&
            (delAleEntryMask & CPSW_ALE_POLICER_TABLEENTRY_DELETE_IPDST))
        {
            aleDelEntryIndex = policerEntry.dstIpIdx;

            if (CpswAle_delIPv6HighEntry(hAle, regs, aleDelEntryIndex) == FALSE)
            {
                CpswAle_clearTableEntry(regs, aleDelEntryIndex);
            }
        }

        policerEntry.validBitmap = 0;
        CSL_CPSW_setAlePolicerEntry(regs, policerIdx, &policerEntry);
        CSL_CPSW_disableAlePolicerThread(regs, policerIdx);
    }

    return status;
}

static int32_t CpswAle_delAllPolicerEntries(CpswAle_Handle hAle,
                                            CSL_AleRegs *regs)
{
    uint32_t numPolicerEntries;
    uint32_t i;
    int32_t status = ENET_SOK;

    CSL_CPSW_getAleStatusNumPolicers(regs, &numPolicerEntries);

    for (i = 0U; i < numPolicerEntries; i++)
    {
        status = CpswAle_delPolicerEntry(hAle, regs, i, CPSW_ALE_POLICER_TABLEENTRY_DELETE_ALL);
        if (status != ENET_SOK)
        {
            break;
        }
    }

    return status;
}

int32_t CpswAle_delAllEntries(CpswAle_Handle hAle,
                              CSL_AleRegs *regs)
{
    CSL_CPSW_ALETABLE_TYPE tableType = hAle->tableType;
    uint32_t tableDepth = CpswAle_getMaxAleEntries(regs);
    uint32_t i;
    int32_t status = ENET_SOK;

    for (i = 0U; i < tableDepth; i++)
    {
        if (CSL_CPSW_getALEEntryType(regs, i, tableType) != CSL_ALE_ENTRYTYPE_FREE)
        {
            CpswAle_clearTableEntry(regs, i);
        }
    }

    return status;
}

#if ENET_CFG_IS_ON(ALE_VLAN_MASK_MUX)
static int32_t CpswAle_findVlanMaskMuxEntry(CpswAle_Handle hAle,
                                            CSL_AleRegs *regs,
                                            uint32_t matchMask,
                                            uint32_t *entryIdx)
{
    uint32_t i;
    uint32_t entryMask;
    int32_t status;

    /* The first ALE Mask Mux is a readonly register set to ALL_PORTS mask */
    for (i = 0U; i < CPSW_ALE_VLAN_MASK_MUX_COUNT(regs); i++)
    {
        CSL_CPSW_getAleVlanMaskMuxEntryReg(regs, i, &entryMask);
        if (matchMask == entryMask)
        {
            break;
        }
    }

    if (i < CPSW_ALE_VLAN_MASK_MUX_COUNT(regs))
    {
        *entryIdx = i;
        status = ENET_SOK;
    }
    else
    {
        status = ENET_ENOTFOUND;
    }

    return status;
}

static int32_t CpswAle_getVlanMaskMuxFreeEntry(CpswAle_Handle hAle,
                                               CSL_AleRegs *regs,
                                               uint32_t *entryIdx)
{
    CSL_CPSW_ALETABLE_TYPE tableType = hAle->tableType;
    CSL_CPSW_ALE_ENTRYTYPE entryType;
    uint32_t tableDepth = CpswAle_getMaxAleEntries(regs);
    uint32_t vlanMaskUsedList[7];
    uint32_t i;
    int32_t status;

    ENET_UTILS_COMPILETIME_ASSERT(ENET_ARRAYSIZE(vlanMaskUsedList) ==
                                  ENET_ARRAYSIZE(regs->I1_ALE_MSK_MUX1));

    memset(vlanMaskUsedList, 0, sizeof(vlanMaskUsedList));

    for (i = 0U; i < tableDepth; i++)
    {
        entryType = CSL_CPSW_getALEEntryType(regs, i, tableType);
        if (entryType == CSL_ALE_ENTRYTYPE_POLICER)
        {
            switch (CSL_CPSW_getALEPolicerEntryType(regs, i, tableType))
            {
                case CSL_ALE_POLICER_ENTRYTYPE_VLAN:
                {
                    CSL_CPSW_ALE_VLAN_ENTRY vlanEntry;

                    memset(&vlanEntry, 0, sizeof(vlanEntry));

                    CSL_CPSW_getAleVlanEntry(regs, i, &vlanEntry, tableType);
                    if (vlanEntry.unRegMcastFloodIndex > 0)
                    {
                        vlanMaskUsedList[CPSW_ALE_VLANMASKMUX2REG(vlanEntry.unRegMcastFloodIndex)] = 1;
                    }

                    if (vlanEntry.regMcastFloodIndex > 0)
                    {
                        vlanMaskUsedList[CPSW_ALE_VLANMASKMUX2REG(vlanEntry.regMcastFloodIndex)] = 1;
                    }

                    break;
                }

                case CSL_ALE_POLICER_ENTRYTYPE_OVLAN:
                {
                    CSL_CPSW_ALE_OUTER_VLAN_ENTRY vlanEntry;

                    CSL_CPSW_getAleOutVlanEntry(regs, i, &vlanEntry, tableType);
                    vlanMaskUsedList[CPSW_ALE_VLANMASKMUX2REG(vlanEntry.unRegMcastFloodIndex)] = 1;
                    vlanMaskUsedList[CPSW_ALE_VLANMASKMUX2REG(vlanEntry.regMcastFloodIndex)] = 1;
                    break;
                }

                default:
                    /* Do nothing for other policer types */
                    break;
            }
        }
    }

    /* Entry 0 is readonly and cannot be used as free entry */
    for (i = 0U; i < ENET_ARRAYSIZE(vlanMaskUsedList); i++)
    {
        if (vlanMaskUsedList[i] != 1)
        {
            break;
        }
    }

    if (i < ENET_ARRAYSIZE(vlanMaskUsedList))
    {
        *entryIdx = i;
        status = ENET_SOK;
    }
    else
    {
        status = ENET_EALLOC;
    }

    return status;
}

static int32_t CpswAle_addVlanMask(CpswAle_Handle hAle,
                                   CSL_AleRegs *regs,
                                   uint32_t mask,
                                   uint32_t *entryIdx)
{
    int32_t status;
    uint32_t maskEntryIdx;

    status = CpswAle_findVlanMaskMuxEntry(hAle, regs, mask, &maskEntryIdx);

    if (status != ENET_SOK)
    {
        status = CpswAle_getVlanMaskMuxFreeEntry(hAle, regs, &maskEntryIdx);
        if (status == ENET_SOK)
        {
            CSL_CPSW_setAleVlanMaskMuxEntryReg(regs, maskEntryIdx, mask);
        }
    }

    if (status == ENET_SOK)
    {
        *entryIdx = maskEntryIdx;
    }

    return status;
}
#endif

static int32_t CpswAle_getIoctlHandlerIdx(uint32_t ioctlCmd, CpswAleIoctlHandlerRegistry_t *ioctlRegistryTbl, uint32_t tableSize, uint32_t *tblIdx)
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

static CpswAleIoctlHandler * CpswAle_getIoctlHandlerFxn(uint32_t ioctlCmd, CpswAleIoctlHandlerRegistry_t *ioctlRegistryTbl, uint32_t tableSize)
{
    int32_t status;
    uint32_t tblIdx;
    CpswAleIoctlHandler *handlerFxn = NULL;

    status = CpswAle_getIoctlHandlerIdx(ioctlCmd, ioctlRegistryTbl, tableSize, &tblIdx);
    if (ENET_SOK == status)
    {
        Enet_devAssert(tblIdx < tableSize, "Invalid IOCTL registry table index for IOCTL cmd:%x", ioctlCmd);
        handlerFxn = ioctlRegistryTbl[tblIdx].fxn;
    }
    else
    {
        handlerFxn = &CpswAle_ioctl_handler_default;
    }
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to get ioctl handler for ioctl cmd: %x. Using default ioctl handler \r\n", ioctlCmd);
    return handlerFxn;
}

static int32_t CpswAle_setIoctlHandlerFxn(uint32_t ioctlCmd,
                                          CpswAleIoctlHandler *ioctlHandlerFxn,
                                          CpswAleIoctlHandlerRegistry_t *ioctlRegistryTbl,
                                          uint32_t tableSize)
{
    int32_t status;
    uint32_t tblIdx;

    status = CpswAle_getIoctlHandlerIdx(ioctlCmd, ioctlRegistryTbl, tableSize, &tblIdx);
    if (ENET_SOK == status)
    {
        Enet_devAssert((tblIdx < tableSize), "Invalid IOCTL registry table index for IOCTL cmd:%x", ioctlCmd);
        Enet_devAssert((ioctlCmd == ioctlRegistryTbl[tblIdx].cmd), "Ioctl table corrupted");
        ioctlRegistryTbl[tblIdx].fxn = ioctlHandlerFxn;
    }
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to set ioctl handler for ioctl cmd: %x \r\n", ioctlCmd);
    return status;
}

static int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_REGISTER_HANDLER(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    const Enet_IoctlRegisterHandlerInArgs *inArgs = (const Enet_IoctlRegisterHandlerInArgs *)prms->inArgs;
    int32_t status;

    status = CpswAle_setIoctlHandlerFxn(inArgs->cmd, 
                                        (CpswAleIoctlHandler *)inArgs->fxn, 
                                        CpswAleIoctlHandlerRegistry, 
                                        ENET_ARRAYSIZE(CpswAleIoctlHandlerRegistry));
    return status;
}

static int32_t CpswAle_ioctl_handler_default(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    return ENET_ENOTSUPPORTED;
}
