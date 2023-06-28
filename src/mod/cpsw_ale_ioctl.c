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
 * \file  cpsw_ale_ioctl.c
 *
 * \brief This file contains the implementation of the CPSW ALE IOCTLs
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
#define CPSW_ALE_VER_REVMIN_AM263X              (0x00000004U)
#define CPSW_ALE_VER_RTL_AM263X                 (0x00000008U)
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

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef const uint8_t (*const CpswAle_MacAddrType)[ENET_MAC_ADDR_LEN];
typedef const uint8_t (*const CpswAle_OuiAddrType)[ENET_OUI_ADDR_LEN];
typedef const uint8_t (*const CpswAle_IPv4AddrType)[ENET_IPv4_ADDR_LEN];
typedef const uint8_t (*const CpswAle_IPv6AddrType)[ENET_IPv6_ADDR_LEN];

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
static uint32_t CpswAle_mapIdlIncVal2Bw(uint32_t aleFreqHz,
                                        uint32_t idlIncVal);

static int32_t CpswAle_delVlan(CpswAle_Handle hAle,
                               CSL_AleRegs *regs,
                               uint32_t vlanId,
                               bool isOuterVlan);

static void CpswAle_ageOut(CSL_AleRegs *regs,
                           bool aleTimerActive,
                           uint32_t tickTimeoutCnt,
                           uint32_t *tickCnt);

static int32_t CpswAle_findAddr(CpswAle_Handle hAle,
                                CSL_AleRegs *regs,
                                CpswAle_MacAddrType addr,
                                uint32_t vlanId,
                                uint32_t *entry);

static void CpswAle_mapMcastMaskBits2RegCfg(uint8_t *addr,
                                            uint32_t numIgnBits,
                                            uint8_t *maskedAddr,
                                            uint32_t *ignMBits,
                                            CSL_CPSW_ALETABLE_TYPE tableType);

static int32_t CpswAle_lookupUcastAddr(CpswAle_Handle hAle,
                                       CSL_AleRegs *regs,
                                       CpswAle_MacAddrType addr,
                                       uint32_t vlanId,
                                       uint32_t *portNum,
                                       bool *blocked,
                                       bool *secure,
                                       bool *super,
                                       bool *ageable,
                                       bool *trunk,
                                       bool *touched,
                                       uint32_t *entryIdx);

static int32_t CpswAle_lookupMcastAddr(CpswAle_Handle hAle,
                                       CSL_AleRegs *regs,
                                       CpswAle_MacAddrType addr,
                                       uint32_t vlanId,
                                       uint32_t numIgnBits,
                                       uint32_t *portMask,
                                       CpswAle_FwdStateLevel *fwdState,
                                       bool *super,
                                       uint32_t *pNumIgnBits,
                                       uint32_t *entryIdx);


static int32_t CpswAle_findOuiAddr(CpswAle_Handle hAle,
                                   CSL_AleRegs *regs,
                                   CpswAle_OuiAddrType addr,
                                   uint32_t *entryIdx);



static int32_t CpswAle_findIPv4Addr(CpswAle_Handle hAle,
                                    CSL_AleRegs *regs,
                                    CpswAle_IPv4AddrType ipv4Addr,
                                    uint32_t numLSBIgnore,
                                    uint32_t *entryIdx);

static int32_t CpswAle_addIPv4Addr(CpswAle_Handle hAle,
                                   CSL_AleRegs *regs,
                                   CpswAle_IPv4AddrType ipv4Addr,
                                   uint32_t numLSBIgnore,
                                   uint32_t *entryIdx,
                                   bool *alloced);

static int32_t CpswAle_findIPv6Addr(CpswAle_Handle hAle,
                                    CSL_AleRegs *regs,
                                    const CpswAle_IPv6AddrType ipv6Addr,
                                    const uint32_t numLSBIgnore,
                                    uint32_t *entry);

static int32_t CpswAle_getFreeIPv6Entry(CpswAle_Handle hAle,
                                        CSL_AleRegs *regs,
                                        uint32_t *entryIdx);

static int32_t CpswAle_addIPv6Addr(CpswAle_Handle hAle,
                                   CSL_AleRegs *regs,
                                   CpswAle_IPv6AddrType ipv6Addr,
                                   uint32_t numLSBIgnore,
                                   uint32_t *entryIdx,
                                   bool *alloced);

static int32_t CpswAle_setReceiveFilter(CpswAle_Handle hAle,
                                        CpswAle_RxFilter rxFilter);

static int32_t CpswAle_getReceiveFilter(const CpswAle_Handle hAle,
                                        CpswAle_RxFilter *rxFilter);

static int32_t CpswAle_delAddr(CpswAle_Handle hAle,
                               CSL_AleRegs *regs,
                               CpswAle_MacAddrType addr,
                               uint32_t vlanId);

static bool CpswAle_isIpv6HighEntry(CpswAle_Handle hAle,
                                    CSL_AleRegs *regs,
                                    uint32_t entryIdx,
                                    uint32_t numPending);


static int32_t CpswAle_validatePolicerIndex(CSL_AleRegs *regs,
                                            uint32_t policerEntryIdx);

static int32_t CpswAle_validateAleEntryIndex(CSL_AleRegs *regs,
                                             uint32_t entryIdx);

static int32_t CpswAle_validateSetPolicerCommon(CSL_AleRegs *regs,
                                                uint32_t policerEntryIdx,
                                                uint32_t entryIdx);

static int32_t CpswAle_validateOuiPolicer(CpswAle_Handle hAle,
                                          CSL_AleRegs *regs,
                                          uint32_t policerEntryIdx,
                                          uint32_t entryIdx,
                                          CpswAle_OuiAddrType ouiAddr);

static int32_t CpswAle_setOuiPolicerEntry(CpswAle_Handle hAle,
                                          CSL_AleRegs *regs,
                                          uint32_t policerEntryIdx,
                                          CpswAle_OuiAddrType ouiAddr,
                                          uint32_t peakRateInBps,
                                          uint32_t commitRateInBps,
                                          uint32_t aleFreq,
                                          uint32_t *entryIdx,
                                          bool *alloced);

static int32_t CpswAle_validateIPv4Policer(CpswAle_Handle hAle,
                                           CSL_AleRegs *regs,
                                           uint32_t policerEntryIdx,
                                           uint32_t entryIdx,
                                           CpswAle_IPv4AddrType ipv4Addr,
                                           uint32_t numLSBIgnore);

static int32_t CpswAle_setIPPolicerEntry(CSL_AleRegs *regs,
                                         uint32_t policerEntryIdx,
                                         uint32_t entryIdx,
                                         CpswAle_AddrType addrType);

static uint32_t CpswAle_mapIPv4AddrOctet2RegWord(CpswAle_IPv4AddrType ipv4Addr);

static uint32_t CpswAle_maskIPv4AddrRegWord(const uint32_t ipv4AddrRegWord,
                                            const uint32_t numLSBIgnore);

static int32_t CpswAle_setIPv4PolicerEntry(CpswAle_Handle hAle,
                                           CSL_AleRegs *regs,
                                           uint32_t policerEntryIdx,
                                           CpswAle_IPv4AddrType ipv4Addr,
                                           CpswAle_AddrType addrType,
                                           uint32_t numLSBIgnore,
                                           uint32_t peakRateInBps,
                                           uint32_t commitRateInBps,
                                           uint32_t aleFreq,
                                           uint32_t *entryIdx,
                                           bool *pEntryAlloced);

static int32_t CpswAle_validateIPv6Policer(CpswAle_Handle hAle,
                                           CSL_AleRegs *regs,
                                           uint32_t policerEntryIdx,
                                           uint32_t entryIdx,
                                           CpswAle_IPv6AddrType ipv6Addr,
                                           uint32_t numLSBIgnore);


static void CpswAle_mapIPv6AddrOctet2RegWord(CpswAle_IPv6AddrType ipv6Addr,
                                             uint32_t(*ipv6RegWord)[4]);

static void CpswAle_maskIPv6AddrRegWord(uint32_t ipv6RegWord[4],
                                        const uint32_t numLSBIgnore);

static bool CpswAle_matchIPv6AddrRegWord(const uint32_t ipv6RegWord1[4],
                                         const uint32_t ipv6RegWord2[4]);

static int32_t CpswAle_setIPv6PolicerEntry(CpswAle_Handle hAle,
                                           CSL_AleRegs *regs,
                                           uint32_t policerEntryIdx,
                                           CpswAle_IPv6AddrType ipv6Addr,
                                           CpswAle_AddrType addrtype,
                                           uint32_t numLSBIgnore,
                                           uint32_t peakRateInBps,
                                           uint32_t commitRateInBps,
                                           uint32_t aleFreq,
                                           uint32_t *entryIdx,
                                           bool *pEntryAlloced);

static int32_t CpswAle_findEtherType(CpswAle_Handle hAle,
                                     CSL_AleRegs *regs,
                                     uint32_t etherType,
                                     uint32_t *entryIdx);

static int32_t CpswAle_addEtherType(CpswAle_Handle hAle,
                                    CSL_AleRegs *regs,
                                    uint32_t etherType,
                                    uint32_t *entryIdx,
                                    bool *alloced);

static int32_t CpswAle_validateEtherTypePolicer(CpswAle_Handle hAle,
                                                CSL_AleRegs *regs,
                                                uint32_t etherType,
                                                uint32_t policerEntryIdx,
                                                uint32_t entryIdx);

static int32_t CpswAle_setEtherTypePolicerEntry(CpswAle_Handle hAle,
                                                CSL_AleRegs *regs,
                                                uint32_t policerEntryIdx,
                                                uint32_t etherType,
                                                uint32_t peakRateInBps,
                                                uint32_t commitRateInBps,
                                                uint32_t aleFreq,
                                                uint32_t *entryIdx,
                                                bool *alloced);

static int32_t CpswAle_validateMacAddrPolicer(CpswAle_Handle hAle,
                                              CSL_AleRegs *regs,
                                              CpswAle_MacAddrType addr,
                                              uint32_t policerEntryIdx,
                                              uint32_t entryIdx,
                                              CpswAle_AddrType addrType);

static int32_t CpswAle_setMacAddrPolicerEntry(CpswAle_Handle hAle,
                                              CSL_AleRegs *regs,
                                              uint32_t policerEntryIdx,
                                              CpswAle_MacAddrType addr,
                                              uint32_t vlanId,
                                              uint32_t portNum,
                                              CpswAle_AddrType addrType,
                                              uint32_t peakRateInBps,
                                              uint32_t commitRateInBps,
                                              uint32_t aleFreq,
                                              uint32_t *entryIdx,
                                              bool *alloced);

static int32_t CpswAle_validateThreadPolicer(CSL_AleRegs *regs,
                                             uint32_t policerEntryIdx,
                                             uint32_t threadId);

static int32_t CpswAle_setThreadPolicerConfig(CSL_AleRegs *regs,
                                              uint32_t policerEntryIdx,
                                              uint32_t threadId);

static int32_t CpswAle_disableThreadPolicerConfig(CSL_AleRegs *regs,
                                                  uint32_t policerEntryIdx);

static int32_t CpswAle_validateVlanPolicer(CSL_AleRegs *regs,
                                           uint32_t policerEntryIdx,
                                           uint32_t entryIdx);

static int32_t CpswAle_setVlanPolicerEntry(CpswAle_Handle hAle,
                                           CSL_AleRegs *regs,
                                           uint32_t policerEntryIdx,
                                           uint32_t vlanId,
                                           bool outVlanFlag,
                                           uint32_t peakRateInBps,
                                           uint32_t commitRateInBps,
                                           uint32_t aleFreq,
                                           uint32_t *entryIdx,
                                           bool *alloced);

static int32_t CpswAle_validatePortPolicer(CpswAle_Handle hAle,
                                           CSL_AleRegs *regs,
                                           uint32_t policerEntryIdx,
                                           uint32_t port_num);

static int32_t CpswAle_setPortPolicerEntry(CpswAle_Handle hAle,
                                           CSL_AleRegs *regs,
                                           uint32_t policerEntryIdx,
                                           uint32_t port_num,
                                           bool isTrunkPort,
                                           uint32_t peakRateInBps,
                                           uint32_t commitRateInBps,
                                           uint32_t aleFreq);

static int32_t CpswAle_validatePriorityPolicer(CSL_AleRegs *regs,
                                               uint32_t policerEntryIdx,
                                               uint32_t priority);

static int32_t CpswAle_setPriorityPolicerEntry(CpswAle_Handle hAle,
                                               CSL_AleRegs *regs,
                                               uint32_t policerEntryIdx,
                                               uint32_t priority,
                                               uint32_t peakRateInBps,
                                               uint32_t commitRateInBps,
                                               uint32_t aleFreq);

static uint32_t CpswAle_mapNumPktsPerSec2RateLimit(CSL_AleRegs *regs,
                                                   uint32_t aleFreq,
                                                   uint32_t numPktsPerSec);

static uint32_t CpswAle_mapRateLimit2NumPktsPerSec(CSL_AleRegs *regs,
                                                   uint32_t aleFreq,
                                                   uint32_t rateLimit);

static int32_t CpswAle_getPolicerDefaultThreadConfig(CpswAle_Handle hAle,
                                                     CSL_AleRegs *regs,
                                                     CpswAle_DfltThreadCfg *defaultThreadCfg);

static int32_t CpswAle_setPolicerRateLimit(CpswAle_Handle hAle,
                                           CSL_AleRegs *regs,
                                           uint32_t policerEntryIdx,
                                           uint32_t peakRateInBitsPs,
                                           uint32_t commitRateInBitsPs,
                                           uint32_t aleFreq);

static void CpswAle_getPolicerStats(CSL_AleRegs *regs,
                                    uint32_t policerIdx,
                                    bool *pPolHit,
                                    bool *pPolRedHit,
                                    bool *pPolYellowHit,
                                    bool clearStats);

static int32_t CpswAle_getMirrorMatchIndex(CpswAle_Handle hAle,
                                           CSL_AleRegs *regs,
                                           const CpswAle_MirrorMatchParams *mirrorMatchParams,
                                           uint32_t *entryIdx);

static int32_t CpswAle_setPortMirrorConfig(CpswAle_Handle hAle,
                                           CSL_AleRegs *regs,
                                           uint32_t maxNumPorts,
                                           const CpswAle_PortMirroringCfg *portMirrorCfg);

static int32_t CpswAle_setPortTrunkConfig(CpswAle_Handle hAle,
                                          CSL_AleRegs *regs,
                                          const CpswAle_TrunkCfg *trunkCfg);

static int32_t CpswAle_allocPolicerEntry(CSL_AleRegs *regs,
                                         uint32_t *freePolicerEntry);

static int32_t CpswAle_setPortBcastMcastLimit(CSL_AleRegs *regs,
                                              uint32_t aleFreqHz,
                                              uint32_t portNum,
                                              bool bcastRateLimitForPortEn,
                                              uint32_t bcastLimitNumPktsPerSec,
                                              bool mcastRateLimitForPortEn,
                                              uint32_t mcastLimitNumPktsPerSec);

static int32_t CpswAle_getPortBcastMcastLimit(CSL_AleRegs *regs,
                                              uint32_t aleFreqHz,
                                              uint32_t portNum,
                                              bool *pBRateLimitEnableForPort,
                                              uint32_t *pBcastLimitNumPktsPerSec,
                                              bool *pMRateLimitEnableForPort,
                                              uint32_t *pMcastLimitNumPktsPerSec);

static void CpswAle_dumpPolicerEntries(CpswAle_Handle hAle,
                                       CSL_AleRegs *regs);

static int32_t CpswAle_delOuiAddr(CpswAle_Handle hAle,
                                  CSL_AleRegs *regs,
                                  const CpswAle_OuiAddrType ouiAddr);

static int32_t CpswAle_delIPv4Addr(CpswAle_Handle hAle,
                                   CSL_AleRegs *regs,
                                   CpswAle_IPv4AddrType ipv4Addr,
                                   uint32_t numLSBIgnore);

static int32_t CpswAle_delIPv6Addr(CpswAle_Handle hAle,
                                   CSL_AleRegs *regs,
                                   CpswAle_IPv6AddrType ipv6Addr,
                                   uint32_t numLSBIgnore);

static int32_t CpswAle_delEtherType(CpswAle_Handle hAle,
                                    CSL_AleRegs *regs,
                                    uint32_t etherType);

static int32_t CpswAle_setPolicer(CpswAle_Handle hAle,
                                  CSL_AleRegs *regs,
                                  const CpswAle_PolicerMatchParams *policerMatchParams,
                                  uint32_t peakRateInBitsPerSec,
                                  uint32_t commitRateInBitsPerSec,
                                  uint32_t aleFreqHz,
                                  bool threadIdEn,
                                  uint32_t threadId,
                                  CpswAle_SetPolicerEntryOutArgs *outArgs);

static int32_t CpswAle_setIPSrcDstPolicerEntry(CpswAle_Handle hAle,
                                               CSL_AleRegs *regs,
                                               uint32_t policerEntryIdx,
                                               const CpswAle_IpAddrClassifierInfo *ipClassifier,
                                               CpswAle_AddrType addrType,
                                               uint32_t peakRateInBitsPerSec,
                                               uint32_t commitRateInBitsPerSec,
                                               uint32_t aleFreqHz,
                                               uint32_t *entryIdx,
                                               bool *alloced);

static int32_t CpswAle_addMacAddrEntry(CpswAle_Handle hAle,
                                       CSL_AleRegs *regs,
                                       CpswAle_MacAddrType addr,
                                       uint32_t vlanId,
                                       uint32_t portNum,
                                       CpswAle_AddrType addrType,
                                       uint32_t *entryIdx);

static int32_t CpswAle_addVlanEntry(CpswAle_Handle hAle,
                                    CSL_AleRegs *regs,
                                    uint32_t vlanId,
                                    bool outVlanFlag,
                                    uint32_t *entryIdx);

static bool CpswAle_checkPolicerMatch(CpswAle_Handle hAle,
                                      CSL_AleRegs *regs,
                                      CSL_CPSW_ALE_POLICER_ENTRY *policerEntry,
                                      const CpswAle_PolicerMatchParams *policerMatchParams);

static uint32_t CpswAle_mapPolicer2CslMask(uint32_t policerMatchEnMask);

static int32_t CpswAle_getPolicer(CpswAle_Handle hAle,
                                  CSL_AleRegs *regs,
                                  const CpswAle_PolicerMatchParams *policerMatchParams,
                                  CpswAle_PolicerEntryOutArgs *outArgs,
                                  uint32_t aleFreqHz);

static int32_t CpswAle_getPolicerIndex(CpswAle_Handle hAle,
                                       CSL_AleRegs *regs,
                                       const CpswAle_PolicerMatchParams *policerMatchParams,
                                       uint32_t *policerIdx);

static int32_t CpswAle_delPolicer(CpswAle_Handle hAle,
                                  CSL_AleRegs *regs,
                                  const CpswAle_PolicerMatchParams *policerMatchParams,
                                  uint32_t delAleEntryMask);

static int32_t CpswAle_delAllPolicerThreadId(CpswAle_Handle hAle,
                                             CSL_AleRegs *regs,
                                             uint32_t threadId);

static int32_t CpswAle_getPolicerStatsHandler(CpswAle_Handle hAle,
                                              CSL_AleRegs *regs,
                                              const CpswAle_PolicerMatchParams *policerMatchParams,
                                              bool *pPolHit,
                                              bool *pPolRedHit,
                                              bool *pPolYellowHit,
                                              bool clearStats);

static int32_t CpswAle_setOAMLpbk(CSL_AleRegs *regs,
                                  uint32_t enableLpbkPortMask);

static void CpswAle_updateMcastHostPortMask(CpswAle_Handle hAle,
                                            CSL_AleRegs *regs,
                                            bool addHostPort);

static uint32_t CpswAle_delPortUcastEntries(CpswAle_Handle hAle,
                                            CSL_AleRegs *regs,
                                            uint32_t portNum);

static void CpswAle_updateUnknownVlanMcastHostPortMask(CpswAle_Handle hAle,
                                                       CSL_AleRegs *regs,
                                                       bool updateRegMcast,
                                                       bool updateUnregMcast,
                                                       bool addHostPort);

static int32_t CpswAle_setRxFilterAll(CpswAle_Handle hAle,
                                      CSL_AleRegs *regs);

static int32_t CpswAle_setRxFilterAllMcast(CpswAle_Handle hAle,
                                           CSL_AleRegs *regs);

static int32_t CpswAle_setRxFilterMcast(CpswAle_Handle hAle,
                                        CSL_AleRegs *regs);

static int32_t CpswAle_setRxFilterBcast(CpswAle_Handle hAle,
                                        CSL_AleRegs *regs);

static int32_t CpswAle_setRxFilterDirect(CpswAle_Handle hAle,
                                         CSL_AleRegs *regs);

static int32_t CpswAle_setRxFilterNone(CpswAle_Handle hAle,
                                       CSL_AleRegs *regs);

static int32_t CpswAle_setBcastMcastLimit(CpswAle_Handle hAle,
                                          CSL_AleRegs *regs,
                                          uint32_t aleFreqHz,
                                          const CpswAle_SetBcastMcastRateLimitInArgs *inArgs);

static int32_t CpswAle_getBcastMcastLimit(CpswAle_Handle hAle,
                                          CSL_AleRegs *regs,
                                          uint32_t aleFreqHz,
                                          CpswAle_GetBcastMcastRateLimitOutArgs *outArgs);

static int32_t CpswAle_setInterVlanConfig(CpswAle_Handle hAle,
                                          CSL_AleRegs *regs,
                                          uint32_t aleFreqHz,
                                          const CpswAle_SetInterVlanCfgInArgs *inArgs,
                                          CpswAle_PolicerEntryOutArgs *outArgs);

static int32_t CpswAle_configThreadPolicer(CpswAle_Handle hAle,
                                           CSL_AleRegs *regs,
                                           uint32_t aleFreqHz,
                                           const CpswAle_PolicerMatchParams *policerMatch,
                                           bool threadIdEn,
                                           uint32_t threadId,
                                           CpswAle_PolicerEntryOutArgs *outArgs);

static int32_t CpswAle_getPortMacAddr(CpswAle_Handle hAle,
                                      CSL_AleRegs *regs,
                                      uint32_t portNum,
                                      CpswAle_MacAddrInfo *macAddrBuffer,
                                      uint32_t maxNumAddr,
                                      uint32_t *portTotalMacAddr);

static int32_t CpswAle_disablePortMirror(CSL_AleRegs *regs);

static int32_t CpswAle_disableBcastMcastLimit(CSL_AleRegs *regs);

static int32_t CpswAle_getPortState(CSL_AleRegs *regs,
                                    uint32_t portNum,
                                    CpswAle_PortState *portState);


static int32_t CpswAle_getPolicerControl(CpswAle_Handle hAle,
                                         CSL_AleRegs *regs,
                                         uint32_t aleFreqHz,
                                         CpswAle_PolicerGlobalCfg *policerCfg);

static int32_t CpswAle_getNoMatchEntry0Policer(CSL_AleRegs *regs,
                                               uint32_t aleFreq,
                                               uint32_t *pPeakRateInBitsPs,
                                               uint32_t *pCommitRateInBitsPs);

static int32_t CpswAle_blockClassifierToHostPort(CpswAle_Handle hAle,
                                                 CSL_AleRegs *regs,
                                                 uint32_t aleFreqHz,
                                                 const CpswAle_PolicerMatchParams *policerMatch,
                                                 CpswAle_PolicerEntryOutArgs *outArgs);

static void CpswAle_printRegs(CpswAle_Handle hAle,
                              CSL_AleRegs *regs);

static void CpswAle_dumpTable(CpswAle_Handle hAle,
                              CSL_AleRegs *regs);

static int32_t CpswAle_addUcastAddr(CpswAle_Handle hAle,
                                    CSL_AleRegs *regs,
                                    uint32_t portNum,
                                    CpswAle_MacAddrType addr,
                                    uint32_t vlanId,
                                    uint32_t blocked,
                                    uint32_t secure,
                                    uint32_t ageable,
                                    uint32_t trunk,
                                    uint32_t *entryIdx);

static int32_t CpswAle_addMcastAddr(CpswAle_Handle hAle,
                                    CSL_AleRegs *regs,
                                    CpswAle_MacAddrType addr,
                                    uint32_t vlanId,
                                    uint32_t portMask,
                                    uint32_t super,
                                    uint32_t fwdState,
                                    uint32_t numIgnBits,
                                    uint32_t *entryIdx);

static int32_t CpswAle_addOuiAddr(CpswAle_Handle hAle,
                                  CSL_AleRegs *regs,
                                  CpswAle_OuiAddrType ouiAddr,
                                  uint32_t *entryIdx,
                                  bool *alloced);


static uint32_t CpswAle_mapIdlIncVal2Bw(uint32_t aleFreqHz,
                                        uint32_t idlIncVal)
{
    uint64_t rateInBps;

    /*
     * CIR or PIR policing rate equation:
     *
     *                         inc value * CPPI_ICLK freq (MHz)
     * Policing rate (Mbps) = ----------------------------------
     *                                   32768
     */
    rateInBps = (uint64_t)idlIncVal * aleFreqHz;
    rateInBps /= CPSW_ALE_IDLINCVAL_DIV_FACTOR;

    return (uint32_t)rateInBps;
}

static int32_t CpswAle_updateVlanMcastHostPortMask(CpswAle_Handle hAle,
                                                   CSL_AleRegs *regs,
                                                   bool updateRegMcast,
                                                   bool updateUnregMcast,
                                                   bool addHostPort)
{
    CSL_CPSW_ALETABLE_TYPE tableType = hAle->tableType;
    CSL_CPSW_ALE_ENTRYTYPE entryType;
    uint32_t tableDepth = CpswAle_getMaxAleEntries(regs);
    uint32_t regMcastFloodMask;
    uint32_t unregMcastFloodMask;
    uint32_t i;
    int32_t status = ENET_SOK;

    for (i = 0U; i < tableDepth; i++)
    {
        entryType = CSL_CPSW_getALEEntryType(regs, i, tableType);
        if (entryType == CSL_ALE_ENTRYTYPE_VLAN)
        {
            if (CSL_CPSW_getALEPolicerEntryType(regs, i, tableType) == CSL_ALE_POLICER_ENTRYTYPE_OVLAN)
            {
                CSL_CPSW_ALE_OUTER_VLAN_ENTRY oVlanEntry;

                CSL_CPSW_getAleOutVlanEntry(regs, i, &oVlanEntry, tableType);
                if ((oVlanEntry.vlanMemList & CPSW_ALE_HOST_PORT_MASK) != 0U)
                {
                    CpswAle_getVlanMcastPortMask(regs, &oVlanEntry, &regMcastFloodMask, &unregMcastFloodMask);
                    if (updateUnregMcast)
                    {
                        if (addHostPort)
                        {
                            unregMcastFloodMask |= CPSW_ALE_HOST_PORT_MASK;
                        }
                        else
                        {
                            unregMcastFloodMask &= CPSW_ALE_ALL_MACPORTS_MASK;
                        }
                    }

                    if (updateRegMcast)
                    {
                        if (addHostPort)
                        {
                            regMcastFloodMask |= CPSW_ALE_HOST_PORT_MASK;
                        }
                        else
                        {
                            regMcastFloodMask &= CPSW_ALE_ALL_MACPORTS_MASK;
                        }
                    }

                    status = CpswAle_setVlanMcastPortMask(hAle,
                                                          regs,
                                                          &oVlanEntry,
                                                          i,
                                                          true,
                                                          regMcastFloodMask,
                                                          unregMcastFloodMask);
                    if (status != ENET_SOK)
                    {
                        break;
                    }
                }
            }

            if (CSL_CPSW_getALEPolicerEntryType(regs, i, tableType) == CSL_ALE_POLICER_ENTRYTYPE_VLAN)
            {
                CSL_CPSW_ALE_VLAN_ENTRY vlanEntry;

                CSL_CPSW_getAleVlanEntry(regs, i, &vlanEntry, tableType);
                if ((vlanEntry.vlanMemList & CPSW_ALE_HOST_PORT_MASK) != 0U)
                {
                    CpswAle_getVlanMcastPortMask(regs, &vlanEntry, &regMcastFloodMask, &unregMcastFloodMask);
                    if (updateUnregMcast)
                    {
                        if (addHostPort)
                        {
                            unregMcastFloodMask |= CPSW_ALE_HOST_PORT_MASK;
                        }
                        else
                        {
                            unregMcastFloodMask &= CPSW_ALE_ALL_MACPORTS_MASK;
                        }
                    }

                    if (updateRegMcast)
                    {
                        if (addHostPort)
                        {
                            regMcastFloodMask |= CPSW_ALE_HOST_PORT_MASK;
                        }
                        else
                        {
                            regMcastFloodMask &= CPSW_ALE_ALL_MACPORTS_MASK;
                        }
                    }

                    status = CpswAle_setVlanMcastPortMask(hAle,
                                                          regs,
                                                          &vlanEntry,
                                                          i,
                                                          false,
                                                          regMcastFloodMask,
                                                          unregMcastFloodMask);
                    if (status != ENET_SOK)
                    {
                        break;
                    }
                }
            }
        }
    }

    return status;
}


static int32_t CpswAle_delVlan(CpswAle_Handle hAle,
                               CSL_AleRegs *regs,
                               uint32_t vlanId,
                               bool isOuterVlan)
{
    uint32_t tableDepth = CpswAle_getMaxAleEntries(regs);
    uint32_t entryIdx;
    int32_t status;

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
                              &entryIdx);
    if ((status != ENET_ENOTFOUND) &&
        (entryIdx < tableDepth))
    {
        status = CpswAle_clearTableEntry(regs, entryIdx);
    }

    return status;
}


static void CpswAle_ageOut(CSL_AleRegs *regs,
                           bool aleTimerActive,
                           uint32_t tickTimeoutCnt,
                           uint32_t *tickCnt)
{
    if (aleTimerActive && (0U != tickTimeoutCnt))
    {
        /* tickCount can become greater if timeout period is changed in between */
        *tickCnt += 1U;
        if (*tickCnt >= tickTimeoutCnt)
        {
            CSL_CPSW_startAleAgeOutNow(regs);

            while (CSL_CPSW_isAleAgeOutDone(regs) != TRUE)
            {
                /* Do nothing */
            };

            *tickCnt = 0U;
        }
    }
}

static int32_t CpswAle_findAddr(CpswAle_Handle hAle,
                                CSL_AleRegs *regs,
                                CpswAle_MacAddrType addr,
                                uint32_t vlanId,
                                uint32_t *entry)
{
    CSL_CPSW_ALETABLE_TYPE tableType = hAle->tableType;
    CSL_CPSW_ALE_ENTRYTYPE entryType;
    uint32_t tableDepth = CpswAle_getMaxAleEntries(regs);
    uint8_t entryAddr[ENET_MAC_ADDR_LEN];
    uint32_t i;
    uint32_t reg0, reg1, reg2;
    uint32_t vid;
    int32_t status = ENET_ENOTFOUND;

    for (i = 0U; i < tableDepth; i++)
    {

        /* read from the ALE table */
        CSL_CPSW_getAleTableEntry(regs, i, &reg0, &reg1, &reg2);

        entryType = CSL_CPSW_getALEEntryType(regs, i, tableType);

        /* We only need to check the entry type and address fields */

        if ((entryType == CSL_ALE_ENTRYTYPE_ADDRESS) ||
            (entryType == CSL_ALE_ENTRYTYPE_VLANADDRESS))
        {
            CSL_CPSW_mapTableWord2MacAddr(reg0, reg1, &entryAddr[0U], tableType);
            if (EnetUtils_cmpMacAddr(entryAddr, *addr))
            {
                vid = CSL_CPSW_extractVid(reg1, tableType);
                if ((vlanId > ENET_VLAN_ID_MAX) ||
                    (vlanId == vid) ||
                    (vlanId == 0U))
                {
                    break;
                }
            }
        }
    }

    if (i != tableDepth)
    {
        *entry = i;
        status = ENET_SOK;
    }

    return status;
}

static int32_t CpswAle_lookupUcastAddr(CpswAle_Handle hAle,
                                       CSL_AleRegs *regs,
                                       CpswAle_MacAddrType addr,
                                       uint32_t vlanId,
                                       uint32_t *portNum,
                                       bool *blocked,
                                       bool *secure,
                                       bool *super,
                                       bool *ageable,
                                       bool *trunk,
                                       bool *touched,
                                       uint32_t *entryIdx)
{
    CSL_CPSW_ALETABLE_TYPE tableType = hAle->tableType;
    CSL_CPSW_ALE_ENTRYTYPE entryType;
    int32_t status;

    status = CpswAle_findAddr(hAle, regs, addr, vlanId, entryIdx);
    if (status == ENET_SOK)
    {
        entryType = CSL_CPSW_getALEEntryType(regs, *entryIdx, tableType);

        /* We only need to check the entry type and address fields */

        if (entryType == CSL_ALE_ENTRYTYPE_ADDRESS)
        {
            CSL_CPSW_ALE_UNICASTADDR_ENTRY ucastEntry;

            CSL_CPSW_getAleUnicastAddrEntry(regs, *entryIdx, &ucastEntry, tableType);

            *portNum = ucastEntry.portNumber;
            *blocked = ucastEntry.blockEnable;
            *secure  = ucastEntry.secureEnable;
            *ageable = ucastEntry.ageable;
            *trunk   = ucastEntry.trunkFlag;
            *touched = ucastEntry.touched;

            if (ucastEntry.blockEnable &&
                ucastEntry.secureEnable)
            {
                *super = TRUE;
                *blocked = FALSE;
                *secure = FALSE;
            }
        }
        else
        {
            CSL_CPSW_ALE_VLANUNICASTADDR_ENTRY vlanUcastEntry;

            Enet_devAssert(entryType == CSL_ALE_ENTRYTYPE_VLANADDRESS,
                           "Entry type %u is a VLANADDRESS type\r\n", entryType);

            CSL_CPSW_getAleVlanUnicastAddrEntry(regs, *entryIdx, &vlanUcastEntry, tableType);

            *portNum = vlanUcastEntry.portNumber;
            *blocked = vlanUcastEntry.blockEnable;
            *secure  = vlanUcastEntry.secureEnable;
            *trunk   = vlanUcastEntry.trunkFlag;
            *ageable = vlanUcastEntry.ageable;
            *touched = vlanUcastEntry.touched;

            if (vlanUcastEntry.blockEnable &&
                vlanUcastEntry.secureEnable)
            {
                *super = TRUE;
                *blocked = FALSE;
                *secure = FALSE;
            }
        }
    }

    return status;
}

static void CpswAle_mapMcastMaskBits2RegCfg(uint8_t *addr,
                                            uint32_t numIgnBits,
                                            uint8_t *maskedAddr,
                                            uint32_t *ignMBits,
                                            CSL_CPSW_ALETABLE_TYPE tableType)
{
    uint32_t ignClrMsk, ignSetMsk;
    uint32_t word0, word1;

    if (numIgnBits != 0U)
    {
        ignClrMsk = ENET_MK_ONES((numIgnBits - 1), 0);
        ignSetMsk = ignClrMsk >> 1U;

        CSL_CPSW_mapMacAddr2TableWord(&word0, &word1, addr, tableType);
        word0 &= ~ignClrMsk;
        word0 |= ignSetMsk;
        CSL_CPSW_mapTableWord2MacAddr(word0, word1, maskedAddr, tableType);

        *ignMBits = 1U;
    }
    else
    {
        EnetUtils_copyMacAddr(maskedAddr, addr);
        *ignMBits = 0U;
    }
}

static int32_t CpswAle_lookupMcastAddr(CpswAle_Handle hAle,
                                       CSL_AleRegs *regs,
                                       CpswAle_MacAddrType addr,
                                       uint32_t vlanId,
                                       uint32_t numIgnBits,
                                       uint32_t *portMask,
                                       CpswAle_FwdStateLevel *fwdState,
                                       bool *super,
                                       uint32_t *pNumIgnBits,
                                       uint32_t *entryIdx)
{
    CSL_CPSW_ALETABLE_TYPE tableType = hAle->tableType;
    CSL_CPSW_ALE_ENTRYTYPE entryType;
    uint8_t mcastMaskedAddr[ENET_MAC_ADDR_LEN];
    uint32_t ignMBitsFlag;
    int32_t status;

    CpswAle_mapMcastMaskBits2RegCfg((uint8_t *)addr,
                                    numIgnBits,
                                    mcastMaskedAddr,
                                    &ignMBitsFlag,
                                    tableType);

    status = CpswAle_findAddr(hAle, regs, (CpswAle_MacAddrType)&mcastMaskedAddr, vlanId, entryIdx);
    if (status == ENET_SOK)
    {
        entryType = CSL_CPSW_getALEEntryType(regs, *entryIdx, tableType);

        /* We only need to check the entry type and address fields */

        if (entryType == CSL_ALE_ENTRYTYPE_ADDRESS)
        {
            CSL_CPSW_ALE_MCASTADDR_ENTRY mcastEntry;

            CSL_CPSW_getAleMcastAddrEntry(regs, *entryIdx, &mcastEntry, tableType);

            *portMask    = mcastEntry.portMask;
            *fwdState    = (CpswAle_FwdStateLevel)mcastEntry.mcastFwdState;
            *super       = mcastEntry.superEnable;
            *pNumIgnBits = mcastEntry.ignMBits;
            EnetUtils_copyMacAddr(mcastMaskedAddr, mcastEntry.macAddress);
        }
        else
        {
            CSL_CPSW_ALE_VLANMCASTADDR_ENTRY vlanMcastEntry;

            Enet_devAssert(entryType == CSL_ALE_ENTRYTYPE_VLANADDRESS,
                           "Entry type %u is a VLANADDRESS type\r\n", entryType);

            CSL_CPSW_getAleVlanMcastAddrEntry(regs, *entryIdx, &vlanMcastEntry, tableType);
            *portMask    = vlanMcastEntry.portMask;
            *fwdState    = (CpswAle_FwdStateLevel)vlanMcastEntry.mcastFwdState;
            *super       = vlanMcastEntry.superEnable;
            *pNumIgnBits = vlanMcastEntry.ignMBits;
            EnetUtils_copyMacAddr(mcastMaskedAddr, vlanMcastEntry.macAddress);
        }

        if (*pNumIgnBits != 0U)
        {
            uint32_t word0, word1;
            uint32_t bitPos = 0U;

            CSL_CPSW_mapMacAddr2TableWord(&word0, &word1, mcastMaskedAddr, tableType);

            /* We will check only word0. Ensure max ign bits cannot be greater than 32 */
            ENET_UTILS_COMPILETIME_ASSERT(CPSW_ALE_MCAST_IGN_BITS_MAX < 32U);

            while (bitPos < CPSW_ALE_MCAST_IGN_BITS_MAX)
            {
                if ((word0 & ENET_BIT(bitPos)) == 0U)
                {
                    break;
                }

                bitPos++;
            }

            *pNumIgnBits += bitPos;
        }
    }

    return status;
}


static int32_t CpswAle_findOuiAddr(CpswAle_Handle hAle,
                                   CSL_AleRegs *regs,
                                   CpswAle_OuiAddrType addr,
                                   uint32_t *entryIdx)
{
    CSL_CPSW_ALETABLE_TYPE tableType = hAle->tableType;
    CSL_CPSW_ALE_ENTRYTYPE entryType;
    uint32_t tableDepth = CpswAle_getMaxAleEntries(regs);
    uint32_t i;
    int32_t status = ENET_ENOTFOUND;

    for (i = 0U; i < tableDepth; i++)
    {
        entryType = CSL_CPSW_getALEEntryType(regs, i, tableType);

        /* We only need to check the entry type and address fields */
        if (entryType == CSL_ALE_ENTRYTYPE_ADDRESS)
        {
            if (CSL_CPSW_getALEAddressType(regs, i, tableType) == CSL_ALE_ADDRTYPE_OUI)
            {
                CSL_CPSW_ALE_OUIADDR_ENTRY ouiEntry;

                CSL_CPSW_getAleOUIAddrEntry(regs, i, &ouiEntry, tableType);

                if (memcmp(ouiEntry.ouiAddress, *addr, ENET_OUI_ADDR_LEN) == 0U)
                {
                    break;
                }
            }
        }
    }

    if (i != tableDepth)
    {
        *entryIdx = i;
        status = ENET_SOK;
    }

    return status;
}



static int32_t CpswAle_findIPv4Addr(CpswAle_Handle hAle,
                                    CSL_AleRegs *regs,
                                    CpswAle_IPv4AddrType ipv4Addr,
                                    uint32_t numLSBIgnore,
                                    uint32_t *entryIdx)
{
    CSL_CPSW_ALETABLE_TYPE tableType = hAle->tableType;
    CSL_CPSW_ALE_ENTRYTYPE entryType;
    uint32_t tableDepth = CpswAle_getMaxAleEntries(regs);
    uint32_t ipv4MatchAddr = CpswAle_mapIPv4AddrOctet2RegWord(ipv4Addr);
    uint32_t ipv4MatchAddrMasked = CpswAle_maskIPv4AddrRegWord(ipv4MatchAddr, numLSBIgnore);
    uint32_t i;
    int32_t status = ENET_ENOTFOUND;

    for (i = 0U; i < tableDepth; i++)
    {
        entryType = CSL_CPSW_getALEEntryType(regs, i, tableType);
        if (entryType == CSL_ALE_ENTRYTYPE_POLICER)
        {
            if (CSL_CPSW_getALEPolicerEntryType(regs, i, tableType) == CSL_ALE_POLICER_ENTRYTYPE_IPV4)
            {
                CSL_CPSW_ALE_IPv4_ENTRY ipv4Entry;
                uint32_t entryIpv4AddrReg;
                uint32_t entryIpv4AddrMaskedReg;

                CSL_CPSW_getAleIPv4Entry(regs, i, &ipv4Entry, tableType);
                {
                    const uint8_t ipv4Addr[ENET_IPv4_ADDR_LEN] =
                        CPSW_ALE_IPV4_ADDR_INIT_LIST(ipv4Entry.address);

                    entryIpv4AddrReg = CpswAle_mapIPv4AddrOctet2RegWord(&ipv4Addr);
                }

                entryIpv4AddrMaskedReg = CpswAle_maskIPv4AddrRegWord(entryIpv4AddrReg, numLSBIgnore);
                if (entryIpv4AddrMaskedReg == ipv4MatchAddrMasked)
                {
                    break;
                }
            }
        }
    }

    if (i != tableDepth)
    {
        *entryIdx = i;
        status = ENET_SOK;
    }

    return status;
}

static int32_t CpswAle_addIPv4Addr(CpswAle_Handle hAle,
                                   CSL_AleRegs *regs,
                                   CpswAle_IPv4AddrType ipv4Addr,
                                   uint32_t numLSBIgnore,
                                   uint32_t *entryIdx,
                                   bool *alloced)
{
    CSL_CPSW_ALETABLE_TYPE tableType = hAle->tableType;
    uint32_t freeEntry;
    uint32_t addrEntry;
    int32_t status = ENET_SOK;

    /* Check for space in the ALE table */
    status = CpswAle_findIPv4Addr(hAle, regs, ipv4Addr, numLSBIgnore, &addrEntry);
    if (status == ENET_ENOTFOUND)
    {
        status = CpswAle_getFreeEntry(hAle, regs, &freeEntry);
        if (status != ENET_EALLOC)
        {
            CPSW_ALE_SAFE_ASSIGN(alloced, true);
        }
    }
    else
    {
        freeEntry = addrEntry;
        CPSW_ALE_SAFE_ASSIGN(alloced, true);
    }

    if (status != ENET_EALLOC)
    {
        CSL_CPSW_ALE_IPv4_ENTRY ipv4Entry;

        memcpy(ipv4Entry.address, *ipv4Addr, ENET_IPv4_ADDR_LEN);
        ipv4Entry.numLSBIgnore = numLSBIgnore;
        CSL_CPSW_setAleIPv4Entry(regs, freeEntry, &ipv4Entry, tableType);

        CPSW_ALE_SAFE_ASSIGN(entryIdx, freeEntry);
        status = ENET_SOK;
    }

    return status;
}

static int32_t CpswAle_findIPv6Addr(CpswAle_Handle hAle,
                                    CSL_AleRegs *regs,
                                    const CpswAle_IPv6AddrType ipv6Addr,
                                    const uint32_t numLSBIgnore,
                                    uint32_t *entry)
{
    CSL_CPSW_ALETABLE_TYPE tableType = hAle->tableType;
    CSL_CPSW_ALE_ENTRYTYPE entryType;
    uint32_t tableDepth = CpswAle_getMaxAleEntries(regs);
    uint32_t ipv6MatchAddr[4U];
    uint32_t i;
    int32_t status = ENET_ENOTFOUND;

    CpswAle_mapIPv6AddrOctet2RegWord(ipv6Addr, &ipv6MatchAddr);
    CpswAle_maskIPv6AddrRegWord(ipv6MatchAddr, numLSBIgnore);

    for (i = 0U; i < tableDepth; i++)
    {
        entryType = CSL_CPSW_getALEEntryType(regs, i, tableType);
        if (entryType == CSL_ALE_ENTRYTYPE_POLICER)
        {
            if (CSL_CPSW_getALEPolicerEntryType(regs, i, tableType) == CSL_ALE_POLICER_ENTRYTYPE_IPV6)
            {
                CSL_CPSW_ALE_IPv6_ENTRY ipv6Entry;
                uint32_t entryIpv6AddrReg[4U];

                CSL_CPSW_getAleIPv6Entry(regs, i, &ipv6Entry, tableType);
                {
                    const uint8_t ipv6Addr[ENET_IPv6_ADDR_LEN] =
                        CPSW_ALE_IPV6_ADDR_INIT_LIST(ipv6Entry.address);

                    CpswAle_mapIPv6AddrOctet2RegWord(&ipv6Addr, &entryIpv6AddrReg);
                }

                CpswAle_maskIPv6AddrRegWord(entryIpv6AddrReg, numLSBIgnore);

                if (CpswAle_matchIPv6AddrRegWord(entryIpv6AddrReg, ipv6MatchAddr))
                {
                    break;
                }
            }
        }
    }

    if (i != tableDepth)
    {
        *entry = i;
        status = ENET_SOK;
    }

    return status;
}

static int32_t CpswAle_getFreeIPv6Entry(CpswAle_Handle hAle,
                                        CSL_AleRegs *regs,
                                        uint32_t *entryIdx)
{
    CSL_CPSW_ALETABLE_TYPE tableType = hAle->tableType;
    CSL_CPSW_ALE_ENTRYTYPE entryTypeLow;
    CSL_CPSW_ALE_ENTRYTYPE entryTypeHigh;
    uint32_t tableDepth = CpswAle_getMaxAleEntries(regs);
    uint32_t ramDepth = CSL_CPSW_getAleStatusRamDepth(regs);
    uint32_t i;
    int32_t status = ENET_SOK;

    switch (ramDepth)
    {
        case CSL_ALE_RAMDEPTH_32:
            tableDepth -= 32;
            break;

        case CSL_ALE_RAMDEPTH_64:
            tableDepth -= 64;
            break;

        case CSL_ALE_RAMDEPTH_128:
            tableDepth -= 128;
            break;
    }

    for (i = 0U; i < tableDepth; i++)
    {
        entryTypeLow = CSL_CPSW_getALEEntryType(regs, CSL_CPSW_getAleIPv6LowEntryIndex(regs, i), tableType);
        entryTypeHigh = CSL_CPSW_getALEEntryType(regs, CSL_CPSW_getAleIPv6HighEntryIndex(regs, i), tableType);
        if ((entryTypeLow == CSL_ALE_ENTRYTYPE_FREE) &&
            (entryTypeHigh == CSL_ALE_ENTRYTYPE_FREE))
        {
            break;
        }
    }

    if (tableDepth == i)
    {
        status = ENET_EALLOC;
    }
    else
    {
        *entryIdx = i;
    }

    return status;
}

static int32_t CpswAle_addIPv6Addr(CpswAle_Handle hAle,
                                   CSL_AleRegs *regs,
                                   CpswAle_IPv6AddrType ipv6Addr,
                                   uint32_t numLSBIgnore,
                                   uint32_t *entryIdx,
                                   bool *alloced)
{
    CSL_CPSW_ALETABLE_TYPE tableType = hAle->tableType;
    uint32_t freeEntry;
    uint32_t addrEntry;
    int32_t status = ENET_SOK;

    /* Check for space in the ALE table */
    status = CpswAle_findIPv6Addr(hAle, regs, ipv6Addr, numLSBIgnore, &addrEntry);
    if (status == ENET_ENOTFOUND)
    {
        status = CpswAle_getFreeIPv6Entry(hAle, regs, &freeEntry);
        if (status != ENET_EALLOC)
        {
            CPSW_ALE_SAFE_ASSIGN(alloced, true);
        }
    }
    else
    {
        freeEntry = addrEntry;
        CPSW_ALE_SAFE_ASSIGN(alloced, false);
    }

    if (status != ENET_EALLOC)
    {
        CSL_CPSW_ALE_IPv6_ENTRY ipv6Entry;

        memcpy(ipv6Entry.address, *ipv6Addr, ENET_IPv6_ADDR_LEN);
        ipv6Entry.numLSBIgnore = numLSBIgnore;
        CSL_CPSW_setAleIPv6Entry(regs, freeEntry, &ipv6Entry, tableType);

        CPSW_ALE_SAFE_ASSIGN(entryIdx, freeEntry);
        status = ENET_SOK;
    }

    return status;
}

/* --------------------------------------------------------------------------- */

static int32_t CpswAle_setReceiveFilter(CpswAle_Handle hAle,
                                        CpswAle_RxFilter rxFilter)
{
    EnetMod_Handle hMod = (EnetMod_Handle)hAle;
    CSL_AleRegs *regs = (CSL_AleRegs *)hMod->virtAddr;
    int32_t status;

    switch (rxFilter)
    {
        case CPSW_ALE_RXFILTER_ALL:
            status = CpswAle_setRxFilterAll(hAle, regs);
            break;

        case CPSW_ALE_RXFILTER_ALLMCAST:
            status = CpswAle_setRxFilterAllMcast(hAle, regs);
            break;

        case CPSW_ALE_RXFILTER_MCAST:
            status = CpswAle_setRxFilterMcast(hAle, regs);
            break;

        case CPSW_ALE_RXFILTER_BCAST:
            status = CpswAle_setRxFilterBcast(hAle, regs);
            break;

        case CPSW_ALE_RXFILTER_DIRECT:
            status = CpswAle_setRxFilterDirect(hAle, regs);
            break;

        case CPSW_ALE_RXFILTER_NOTHING:
            status = CpswAle_setRxFilterNone(hAle, regs);
            break;

        default:
            status = ENET_EINVALIDPARAMS;
            break;
    }

    if (status == ENET_SOK)
    {
        hAle->rxFilter = rxFilter;
    }

    return status;
}

static int32_t CpswAle_getReceiveFilter(const CpswAle_Handle hAle,
                                        CpswAle_RxFilter *rxFilter)
{
    int32_t status;

    CPSW_ALE_SAFE_ASSIGN(rxFilter, hAle->rxFilter);

    /* Validate our handle */
    if (rxFilter == NULL)
    {
        status = ENET_EINVALIDPARAMS;
    }
    else
    {
        status = ENET_SOK;
    }

    return status;
}

static int32_t CpswAle_delAddr(CpswAle_Handle hAle,
                               CSL_AleRegs *regs,
                               CpswAle_MacAddrType addr,
                               uint32_t vlanId)
{
    uint32_t entryIdx;
    int32_t status = ENET_ENOTFOUND;

    status = CpswAle_findAddr(hAle, regs, addr, vlanId, &entryIdx);
    if (status == ENET_SOK)
    {
        CpswAle_clearTableEntry(regs, entryIdx);
    }

    return status;
}

static bool CpswAle_isIpv6HighEntry(CpswAle_Handle hAle,
                                    CSL_AleRegs *regs,
                                    uint32_t entryIdx,
                                    uint32_t numPending)
{
    CSL_CPSW_ALETABLE_TYPE tableType = hAle->tableType;
    bool status = false;
    uint32_t highOffset;
    uint32_t tableIdx;

    if (numPending > 0U)
    {
        highOffset = CSL_CPSW_getAleIPv6HighEntryOffset(regs);

        tableIdx = entryIdx - highOffset;
        if ((CSL_CPSW_getALEEntryType(regs, tableIdx, tableType) == CSL_ALE_ENTRYTYPE_POLICER)  &&
            (CSL_CPSW_getALEPolicerEntryType(regs, tableIdx, tableType) == CSL_ALE_POLICER_ENTRYTYPE_IPV6))
        {
            status = true;
        }
    }

    return status;
}


static int32_t CpswAle_validatePolicerIndex(CSL_AleRegs *regs,
                                            uint32_t policerEntryIdx)
{
    int32_t status = ENET_SOK;

    if (policerEntryIdx >= CpswAle_getMaxPolicers(regs))
    {
        status = ENET_EINVALIDPARAMS;
    }

    return status;
}

static int32_t CpswAle_validateAleEntryIndex(CSL_AleRegs *regs,
                                             uint32_t entryIdx)
{
    int32_t status = ENET_SOK;

    if (entryIdx >= CpswAle_getMaxAleEntries(regs))
    {
        status = ENET_EINVALIDPARAMS;
    }

    return status;
}

static int32_t CpswAle_validateSetPolicerCommon(CSL_AleRegs *regs,
                                                uint32_t policerEntryIdx,
                                                uint32_t entryIdx)
{
    int32_t status = ENET_SOK;

    status = CpswAle_validatePolicerIndex(regs, policerEntryIdx);

    if (status == ENET_SOK)
    {
        status = CpswAle_validateAleEntryIndex(regs, entryIdx);
    }

    return status;
}

static int32_t CpswAle_validateOuiPolicer(CpswAle_Handle hAle,
                                          CSL_AleRegs *regs,
                                          uint32_t policerEntryIdx,
                                          uint32_t entryIdx,
                                          CpswAle_OuiAddrType ouiAddr)
{
    int32_t status;

    status = CpswAle_validateSetPolicerCommon(regs, policerEntryIdx, entryIdx);
    if (status == ENET_SOK)
    {
        uint32_t matchEntry;

        status = CpswAle_findOuiAddr(hAle, regs, ouiAddr, &matchEntry);
        if (status == ENET_SOK)
        {
            if (matchEntry != entryIdx)
            {
                status = ENET_EINVALIDPARAMS;
            }
        }
    }

    return status;
}

static int32_t CpswAle_setOuiPolicerEntry(CpswAle_Handle hAle,
                                          CSL_AleRegs *regs,
                                          uint32_t policerEntryIdx,
                                          CpswAle_OuiAddrType ouiAddr,
                                          uint32_t peakRateInBps,
                                          uint32_t commitRateInBps,
                                          uint32_t aleFreq,
                                          uint32_t *entryIdx,
                                          bool *alloced)
{
    int32_t status;

    status = CpswAle_addOuiAddr(hAle, regs, ouiAddr, entryIdx, alloced);
    if (status == ENET_SOK)
    {
        status = CpswAle_validateOuiPolicer(hAle, regs, policerEntryIdx, *entryIdx, ouiAddr);
    }

    if (status == ENET_SOK)
    {
        CSL_CPSW_ALE_POLICER_ENTRY polEntry;

        CSL_CPSW_getAlePolicerEntry(regs, policerEntryIdx, &polEntry);
        polEntry.validBitmap |= CSL_CPSW_ALE_POLICER_OUI_VALID;
        polEntry.ouiIdx = *entryIdx;
        CSL_CPSW_setAlePolicerEntry(regs, policerEntryIdx, &polEntry);
    }

    if (status == ENET_SOK)
    {
        status = CpswAle_setPolicerRateLimit(hAle,
                                             regs,
                                             policerEntryIdx,
                                             peakRateInBps,
                                             commitRateInBps,
                                             aleFreq);
    }

    return status;
}

static int32_t CpswAle_validateIPv4Policer(CpswAle_Handle hAle,
                                           CSL_AleRegs *regs,
                                           uint32_t policerEntryIdx,
                                           uint32_t entryIdx,
                                           CpswAle_IPv4AddrType ipv4Addr,
                                           uint32_t numLSBIgnore)
{
    CSL_CPSW_ALETABLE_TYPE tableType = hAle->tableType;
    int32_t status;

    status = CpswAle_validateSetPolicerCommon(regs, policerEntryIdx, entryIdx);
    if (status == ENET_SOK)
    {
        if (numLSBIgnore > CSL_CPSW_getIpv4IgnBitsMax(tableType))
        {
            status = ENET_EINVALIDPARAMS;
        }
    }

    if (status == ENET_SOK)
    {
        uint32_t matchEntry;

        status = CpswAle_findIPv4Addr(hAle, regs, ipv4Addr, numLSBIgnore, &matchEntry);
        if (status == ENET_SOK)
        {
            if (matchEntry != entryIdx)
            {
                status = ENET_EINVALIDPARAMS;
            }
        }
    }

    return status;
}

static int32_t CpswAle_setIPPolicerEntry(CSL_AleRegs *regs,
                                         uint32_t policerEntryIdx,
                                         uint32_t entryIdx,
                                         CpswAle_AddrType addrType)
{
    CSL_CPSW_ALE_POLICER_ENTRY polEntry;

    CSL_CPSW_getAlePolicerEntry(regs, policerEntryIdx, &polEntry);

    if (addrType == CPSW_ALE_ADDR_TYPE_SRC)
    {
        polEntry.validBitmap |= CSL_CPSW_ALE_POLICER_SRC_IP_VALID;
        polEntry.srcIpIdx = entryIdx;
    }
    else
    {
        polEntry.validBitmap |= CSL_CPSW_ALE_POLICER_DST_IP_VALID;
        polEntry.dstIpIdx = entryIdx;
    }

    CSL_CPSW_setAlePolicerEntry(regs, policerEntryIdx, &polEntry);

    return ENET_SOK;
}

static uint32_t CpswAle_mapIPv4AddrOctet2RegWord(CpswAle_IPv4AddrType ipv4Addr)
{
    uint32_t regWord;

    regWord = ((*ipv4Addr)[0] << 24) |
              ((*ipv4Addr)[1] << 16) |
              ((*ipv4Addr)[2] << 8) |
              ((*ipv4Addr)[3] << 0);
    return regWord;
}

static uint32_t CpswAle_maskIPv4AddrRegWord(const uint32_t ipv4AddrRegWord,
                                            const uint32_t numLSBIgnore)
{
    uint32_t maskedAddr;

    maskedAddr = ipv4AddrRegWord & (uint32_t) ~((1U << numLSBIgnore) - 1U);
    return maskedAddr;
}

static int32_t CpswAle_setIPv4PolicerEntry(CpswAle_Handle hAle,
                                           CSL_AleRegs *regs,
                                           uint32_t policerEntryIdx,
                                           CpswAle_IPv4AddrType ipv4Addr,
                                           CpswAle_AddrType addrType,
                                           uint32_t numLSBIgnore,
                                           uint32_t peakRateInBps,
                                           uint32_t commitRateInBps,
                                           uint32_t aleFreq,
                                           uint32_t *entryIdx,
                                           bool *pEntryAlloced)
{
    int32_t status = ENET_SOK;

    if (status == ENET_SOK)
    {
        status = CpswAle_addIPv4Addr(hAle,
                                     regs,
                                     ipv4Addr,
                                     numLSBIgnore,
                                     entryIdx,
                                     pEntryAlloced);
    }

    if (status == ENET_SOK)
    {
        status = CpswAle_validateIPv4Policer(hAle,
                                             regs,
                                             policerEntryIdx,
                                             *entryIdx,
                                             ipv4Addr,
                                             numLSBIgnore);
    }

    if (status == ENET_SOK)
    {
        CpswAle_setIPPolicerEntry(regs,
                                  policerEntryIdx,
                                  *entryIdx,
                                  addrType);
    }

    if (status == ENET_SOK)
    {
        status = CpswAle_setPolicerRateLimit(hAle,
                                             regs,
                                             policerEntryIdx,
                                             peakRateInBps,
                                             commitRateInBps,
                                             aleFreq);
    }

    return status;
}

static int32_t CpswAle_validateIPv6Policer(CpswAle_Handle hAle,
                                           CSL_AleRegs *regs,
                                           uint32_t policerEntryIdx,
                                           uint32_t entryIdx,
                                           CpswAle_IPv6AddrType ipv6Addr,
                                           uint32_t numLSBIgnore)
{
    CSL_CPSW_ALETABLE_TYPE tableType = hAle->tableType;
    int32_t status;

    status = CpswAle_validateSetPolicerCommon(regs, policerEntryIdx, entryIdx);
    if (status == ENET_SOK)
    {
        if (numLSBIgnore > CSL_CPSW_getIpv6IgnBitsMax(tableType))
        {
            status = ENET_EINVALIDPARAMS;
        }
    }

    if (status == ENET_SOK)
    {
        uint32_t matchEntry;

        status = CpswAle_findIPv6Addr(hAle, regs, ipv6Addr, numLSBIgnore, &matchEntry);
        if (status == ENET_SOK)
        {
            if (matchEntry != entryIdx)
            {
                status = ENET_EINVALIDPARAMS;
            }
        }
    }

    return status;
}

static void CpswAle_mapIPv6AddrOctet2RegWord(CpswAle_IPv6AddrType ipv6Addr,
                                             uint32_t(*ipv6RegWord)[4])
{
    uint32_t i;

    /* Setup the IPv6 configuration */
    for (i = 0U; i < 4; i++)
    {
        (*ipv6RegWord)[i] = ((*ipv6Addr)[4 * i] << 24) |
                            ((*ipv6Addr)[4 * i + 1] << 16) |
                            ((*ipv6Addr)[4 * i + 2] << 8) |
                            ((*ipv6Addr)[4 * i + 3] << 0);
    }
}

static void CpswAle_maskIPv6AddrRegWord(uint32_t ipv6RegWord[4],
                                        const uint32_t numLSBIgnore)
{
    Uint32 addrMask[4];
    UInt32 addrMaskBitPos;
    UInt32 addrMaskBitRemain;
    uint32_t i;

    /* Setup the IPv6 address mask configuration */
    addrMaskBitRemain = numLSBIgnore;
    addrMaskBitPos = 96;
    for (i = 0U; i < 4; i++)
    {
        if (addrMaskBitRemain > addrMaskBitPos)
        {
            addrMask[i] = ~((1U << (addrMaskBitRemain - addrMaskBitPos)) - 1U);
            addrMaskBitRemain -= addrMaskBitPos;
        }
        else
        {
            addrMask[i] = ~0U;
        }

        addrMaskBitPos -= 32;
    }

    /* Setup the IPv6 configuration */
    for (i = 0U; i < 4; i++)
    {
        ipv6RegWord[i] &= addrMask[i];
    }
}

static bool CpswAle_matchIPv6AddrRegWord(const uint32_t ipv6RegWord1[4],
                                         const uint32_t ipv6RegWord2[4])
{
    bool match;
    uint32_t i;

    /* Setup the IPv6 configuration */
    for (i = 0U; i < 4; i++)
    {
        if (ipv6RegWord1[i] != ipv6RegWord2[i])
        {
            break;
        }
    }

    if (i < 4)
    {
        match = FALSE;
    }
    else
    {
        match = TRUE;
    }

    return match;
}

static int32_t CpswAle_setIPv6PolicerEntry(CpswAle_Handle hAle,
                                           CSL_AleRegs *regs,
                                           uint32_t policerEntryIdx,
                                           CpswAle_IPv6AddrType ipv6Addr,
                                           CpswAle_AddrType addrtype,
                                           uint32_t numLSBIgnore,
                                           uint32_t peakRateInBps,
                                           uint32_t commitRateInBps,
                                           uint32_t aleFreq,
                                           uint32_t *entryIdx,
                                           bool *pEntryAlloced)
{
    int32_t status = ENET_SOK;

    if (status == ENET_SOK)
    {
        status = CpswAle_addIPv6Addr(hAle,
                                     regs,
                                     ipv6Addr,
                                     numLSBIgnore,
                                     entryIdx,
                                     pEntryAlloced);
    }

    if (status == ENET_SOK)
    {
        status = CpswAle_validateIPv6Policer(hAle,
                                             regs,
                                             policerEntryIdx,
                                             *entryIdx,
                                             ipv6Addr,
                                             numLSBIgnore);
    }

    if (status == ENET_SOK)
    {
        status = CpswAle_setIPPolicerEntry(regs,
                                           policerEntryIdx,
                                           *entryIdx,
                                           addrtype);
    }

    if (status == ENET_SOK)
    {
        status = CpswAle_setPolicerRateLimit(hAle,
                                             regs,
                                             policerEntryIdx,
                                             peakRateInBps,
                                             commitRateInBps,
                                             aleFreq);
    }

    return status;
}

static int32_t CpswAle_findEtherType(CpswAle_Handle hAle,
                                     CSL_AleRegs *regs,
                                     uint32_t etherType,
                                     uint32_t *entryIdx)
{
    CSL_CPSW_ALETABLE_TYPE tableType = hAle->tableType;
    CSL_CPSW_ALE_ENTRYTYPE entryType;
    uint32_t tableDepth = CpswAle_getMaxAleEntries(regs);
    uint32_t i;
    int32_t status = ENET_ENOTFOUND;

    for (i = 0U; i < tableDepth; i++)
    {
        entryType = CSL_CPSW_getALEEntryType(regs, i, tableType);
        if (entryType == CSL_ALE_ENTRYTYPE_POLICER)
        {
            if (CSL_CPSW_getALEPolicerEntryType(regs, i, tableType) == CSL_ALE_POLICER_ENTRYTYPE_ETHERTYPE)
            {
                CSL_CPSW_ALE_ETHERTYPE_ENTRY etherEntry;

                CSL_CPSW_getAleEthertypeEntry(regs, i, &etherEntry, tableType);
                if (etherEntry.ethertype == etherType)
                {
                    break;
                }
            }
        }
    }

    if (i != tableDepth)
    {
        *entryIdx = i;
        status = ENET_SOK;
    }

    return status;
}

static int32_t CpswAle_addEtherType(CpswAle_Handle hAle,
                                    CSL_AleRegs *regs,
                                    uint32_t etherType,
                                    uint32_t *entryIdx,
                                    bool *alloced)
{
    CSL_CPSW_ALETABLE_TYPE tableType = hAle->tableType;
    uint32_t freeEntry;
    uint32_t addrEntry;
    int32_t status = ENET_SOK;

    /* Check for space in the ALE table */
    status = CpswAle_findEtherType(hAle, regs, etherType, &addrEntry);
    if (ENET_ENOTFOUND == status)
    {
        status = CpswAle_getFreeEntry(hAle, regs, &freeEntry);
        if (status != ENET_EALLOC)
        {
            CPSW_ALE_SAFE_ASSIGN(alloced, TRUE);
        }
    }
    else
    {
        freeEntry = addrEntry;
        CPSW_ALE_SAFE_ASSIGN(alloced, FALSE);
    }

    if (status != ENET_EALLOC)
    {
        CSL_CPSW_ALE_ETHERTYPE_ENTRY etherTypeEntry;

        etherTypeEntry.ethertype = etherType;
        CSL_CPSW_setAleEthertypeEntry(regs, freeEntry, &etherTypeEntry, tableType);
        CPSW_ALE_SAFE_ASSIGN(entryIdx, freeEntry);
        status = ENET_SOK;
    }

    return status;
}

static int32_t CpswAle_validateEtherTypePolicer(CpswAle_Handle hAle,
                                                CSL_AleRegs *regs,
                                                uint32_t etherType,
                                                uint32_t policerEntryIdx,
                                                uint32_t entryIdx)
{
    CSL_CPSW_ALETABLE_TYPE tableType = hAle->tableType;
    int32_t status;

    status = CpswAle_validateSetPolicerCommon(regs,
                                              policerEntryIdx,
                                              entryIdx);

    if (status == ENET_SOK)
    {
        if (etherType > CSL_CPSW_getEthertypeMax(tableType))
        {
            status = ENET_EINVALIDPARAMS;
        }
    }

    if (status == ENET_SOK)
    {
        uint32_t matchEntry;

        status = CpswAle_findEtherType(hAle, regs, etherType, &matchEntry);
        if (status == ENET_SOK)
        {
            if (matchEntry != entryIdx)
            {
                status = ENET_EINVALIDPARAMS;
            }
        }
    }

    return status;
}

static int32_t CpswAle_setEtherTypePolicerEntry(CpswAle_Handle hAle,
                                                CSL_AleRegs *regs,
                                                uint32_t policerEntryIdx,
                                                uint32_t etherType,
                                                uint32_t peakRateInBps,
                                                uint32_t commitRateInBps,
                                                uint32_t aleFreq,
                                                uint32_t *entryIdx,
                                                bool *alloced)
{
    int32_t status = ENET_SOK;

    if (status == ENET_SOK)
    {
        status = CpswAle_addEtherType(hAle, regs, etherType, entryIdx, alloced);
    }

    if (status == ENET_SOK)
    {
        status = CpswAle_validateEtherTypePolicer(hAle, regs, etherType, policerEntryIdx, *entryIdx);
    }

    if (status == ENET_SOK)
    {
        CSL_CPSW_ALE_POLICER_ENTRY polEntry;

        CSL_CPSW_getAlePolicerEntry(regs, policerEntryIdx, &polEntry);

        polEntry.validBitmap |= CSL_CPSW_ALE_POLICER_ETHERTYPE_VALID;
        polEntry.ethertypeIdx = *entryIdx;

        CSL_CPSW_setAlePolicerEntry(regs, policerEntryIdx, &polEntry);
    }

    if (status == ENET_SOK)
    {
        status = CpswAle_setPolicerRateLimit(hAle,
                                             regs,
                                             policerEntryIdx,
                                             peakRateInBps,
                                             commitRateInBps,
                                             aleFreq);
    }

    return status;
}

static int32_t CpswAle_validateMacAddrPolicer(CpswAle_Handle hAle,
                                              CSL_AleRegs *regs,
                                              CpswAle_MacAddrType addr,
                                              uint32_t policerEntryIdx,
                                              uint32_t entryIdx,
                                              CpswAle_AddrType addrType)
{
    CSL_CPSW_ALETABLE_TYPE tableType = hAle->tableType;
    bool mcastAddr;
    uint32_t word0, word1;
    int32_t status;

    status = CpswAle_validateSetPolicerCommon(regs, policerEntryIdx, entryIdx);
    if (status == ENET_SOK)
    {
        CSL_CPSW_mapMacAddr2TableWord(&word0, &word1, (uint8_t *)addr, tableType);
        mcastAddr = EnetUtils_isMcastAddr((const uint8_t *)addr);
        if ((mcastAddr) && (addrType == CPSW_ALE_ADDR_TYPE_SRC))
        {
            status = ENET_EINVALIDPARAMS;
        }
    }

    return status;
}

static int32_t CpswAle_setMacAddrPolicerEntry(CpswAle_Handle hAle,
                                              CSL_AleRegs *regs,
                                              uint32_t policerEntryIdx,
                                              CpswAle_MacAddrType addr,
                                              uint32_t vlanId,
                                              uint32_t portNum,
                                              CpswAle_AddrType addrType,
                                              uint32_t peakRateInBps,
                                              uint32_t commitRateInBps,
                                              uint32_t aleFreq,
                                              uint32_t *entryIdx,
                                              bool *alloced)
{
    int32_t status;

    status = CpswAle_findAddr(hAle, regs, addr, vlanId, entryIdx);
    if (ENET_ENOTFOUND == status)
    {
        /* If entry is not found add the entry with default settings */
        status = CpswAle_addMacAddrEntry(hAle,
                                         regs,
                                         addr,
                                         vlanId,
                                         portNum,
                                         addrType,
                                         entryIdx);
        if (status == ENET_SOK)
        {
            CPSW_ALE_SAFE_ASSIGN(alloced, TRUE);
        }
    }
    else
    {
        CPSW_ALE_SAFE_ASSIGN(alloced, FALSE);
    }

    if (status == ENET_SOK)
    {
        status = CpswAle_validateMacAddrPolicer(hAle,
                                                regs,
                                                addr,
                                                policerEntryIdx,
                                                *entryIdx,
                                                addrType);
    }

    if (status == ENET_SOK)
    {
        CSL_CPSW_ALE_POLICER_ENTRY polEntry;

        CSL_CPSW_getAlePolicerEntry(regs, policerEntryIdx, &polEntry);
        if (addrType == CPSW_ALE_ADDR_TYPE_SRC)
        {
            polEntry.validBitmap |= CSL_CPSW_ALE_POLICER_SRC_MAC_VALID;
            polEntry.srcMacIdx = *entryIdx;
        }
        else
        {
            polEntry.validBitmap |= CSL_CPSW_ALE_POLICER_DST_MAC_VALID;
            polEntry.dstMacIdx = *entryIdx;
        }

        CSL_CPSW_setAlePolicerEntry(regs, policerEntryIdx, &polEntry);
    }

    if (status == ENET_SOK)
    {
        status = CpswAle_setPolicerRateLimit(hAle, regs,
                                             policerEntryIdx,
                                             peakRateInBps,
                                             commitRateInBps,
                                             aleFreq);
    }

    return status;
}

static int32_t CpswAle_validateThreadPolicer(CSL_AleRegs *regs,
                                             uint32_t policerEntryIdx,
                                             uint32_t threadId)
{
    int32_t status;

    status = CpswAle_validatePolicerIndex(regs, policerEntryIdx);
    if (status == ENET_SOK)
    {
        if (threadId > CSL_ALE_THREADMAPVAL_THREADVAL_MAX)
        {
            status = ENET_EINVALIDPARAMS;
        }
    }

    if (status == ENET_SOK)
    {
        CSL_CPSW_ALE_POLICER_ENTRY policerEntry;

        CSL_CPSW_getAlePolicerEntry(regs, policerEntryIdx, &policerEntry);
        if (policerEntry.validBitmap == 0)
        {
            status = ENET_EINVALIDPARAMS;
        }
    }

    return status;
}

static int32_t CpswAle_setThreadPolicerConfig(CSL_AleRegs *regs,
                                              uint32_t policerEntryIdx,
                                              uint32_t threadId)
{
    int32_t status = ENET_SOK;

    if (status == ENET_SOK)
    {
        status = CpswAle_validateThreadPolicer(regs, policerEntryIdx, threadId);
    }

    if (status == ENET_SOK)
    {
        CSL_CPSW_ALE_POLICER_ENTRY polEntry;

        CSL_CPSW_getAlePolicerEntry(regs, policerEntryIdx, &polEntry);
        polEntry.validBitmap |= CSL_CPSW_ALE_POLICER_THREAD_VALID;
        polEntry.thread = threadId;
        CSL_CPSW_setAlePolicerEntry(regs, policerEntryIdx, &polEntry);
    }

    return status;
}

static int32_t CpswAle_disableThreadPolicerConfig(CSL_AleRegs *regs,
                                                  uint32_t policerEntryIdx)
{
    CSL_CPSW_disableAlePolicerThread(regs, policerEntryIdx);
    return ENET_SOK;
}

static int32_t CpswAle_validateVlanPolicer(CSL_AleRegs *regs,
                                           uint32_t policerEntryIdx,
                                           uint32_t entryIdx)
{
    int32_t status;

    status = CpswAle_validateSetPolicerCommon(regs, policerEntryIdx, entryIdx);

    return status;
}

static int32_t CpswAle_setVlanPolicerEntry(CpswAle_Handle hAle,
                                           CSL_AleRegs *regs,
                                           uint32_t policerEntryIdx,
                                           uint32_t vlanId,
                                           bool outVlanFlag,
                                           uint32_t peakRateInBps,
                                           uint32_t commitRateInBps,
                                           uint32_t aleFreq,
                                           uint32_t *entryIdx,
                                           bool *alloced)
{
    int32_t status = ENET_SOK;

    if (status == ENET_SOK)
    {
        status = CpswAle_findVlan(hAle, regs, vlanId, outVlanFlag,
                                  NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
                                  entryIdx);
        if (status == ENET_ENOTFOUND)
        {
            /* If entry is not found add the entry with default settings */
            status = CpswAle_addVlanEntry(hAle, regs, vlanId, outVlanFlag, entryIdx);
            if (status == ENET_SOK)
            {
                CPSW_ALE_SAFE_ASSIGN(alloced, TRUE);
            }
        }
        else
        {
            CPSW_ALE_SAFE_ASSIGN(alloced, FALSE);
        }
    }

    if (status == ENET_SOK)
    {
        status = CpswAle_validateVlanPolicer(regs, policerEntryIdx, *entryIdx);
    }

    if (status == ENET_SOK)
    {
        CSL_CPSW_ALE_POLICER_ENTRY polEntry;

        CSL_CPSW_getAlePolicerEntry(regs, policerEntryIdx, &polEntry);
        if (outVlanFlag)
        {
            polEntry.validBitmap |= CSL_CPSW_ALE_POLICER_OVLAN_VALID;
            polEntry.ovlanIdx = *entryIdx;
        }
        else
        {
            polEntry.validBitmap |= CSL_CPSW_ALE_POLICER_VLAN_VALID;
            polEntry.vlanIdx = *entryIdx;
        }

        CSL_CPSW_setAlePolicerEntry(regs, policerEntryIdx, &polEntry);
    }

    if (status == ENET_SOK)
    {
        status = CpswAle_setPolicerRateLimit(hAle,
                                             regs,
                                             policerEntryIdx,
                                             peakRateInBps,
                                             commitRateInBps,
                                             aleFreq);
    }

    return status;
}

static int32_t CpswAle_validatePortPolicer(CpswAle_Handle hAle,
                                           CSL_AleRegs *regs,
                                           uint32_t policerEntryIdx,
                                           uint32_t port_num)
{
    int32_t status = ENET_SOK;

    status = CpswAle_validatePolicerIndex(regs, policerEntryIdx);

    if (status == ENET_SOK)
    {
        if (port_num >= hAle->numPorts)
        {
            status = ENET_EINVALIDPARAMS;
        }
    }

    return status;
}

static int32_t CpswAle_setPortPolicerEntry(CpswAle_Handle hAle,
                                           CSL_AleRegs *regs,
                                           uint32_t policerEntryIdx,
                                           uint32_t port_num,
                                           bool isTrunkPort,
                                           uint32_t peakRateInBps,
                                           uint32_t commitRateInBps,
                                           uint32_t aleFreq)
{
    int32_t status = ENET_SOK;

    status = CpswAle_validatePortPolicer(hAle, regs, policerEntryIdx, port_num);
    if (status == ENET_SOK)
    {
        CSL_CPSW_ALE_POLICER_ENTRY polEntry;

        CSL_CPSW_getAlePolicerEntry(regs, policerEntryIdx, &polEntry);
        polEntry.validBitmap |= CSL_CPSW_ALE_POLICER_PORT_VALID;
        polEntry.port = port_num;
        if (isTrunkPort)
        {
            polEntry.validBitmap |= CSL_CPSW_ALE_POLICER_PORT_TRUNK_VALID;
        }

        CSL_CPSW_setAlePolicerEntry(regs, policerEntryIdx, &polEntry);
    }

    if (status == ENET_SOK)
    {
        status = CpswAle_setPolicerRateLimit(hAle,
                                             regs,
                                             policerEntryIdx,
                                             peakRateInBps,
                                             commitRateInBps,
                                             aleFreq);
    }

    return status;
}

static int32_t CpswAle_validatePriorityPolicer(CSL_AleRegs *regs,
                                               uint32_t policerEntryIdx,
                                               uint32_t priority)
{
    int32_t status = ENET_SOK;

    status = CpswAle_validatePolicerIndex(regs, policerEntryIdx);
    if (status == ENET_SOK)
    {
        if (priority > CSL_ALE_POLICECFG0_PRI_VAL_MAX)
        {
            status = ENET_EINVALIDPARAMS;
        }
    }

    return status;
}

static int32_t CpswAle_setPriorityPolicerEntry(CpswAle_Handle hAle,
                                               CSL_AleRegs *regs,
                                               uint32_t policerEntryIdx,
                                               uint32_t priority,
                                               uint32_t peakRateInBps,
                                               uint32_t commitRateInBps,
                                               uint32_t aleFreq)
{
    int32_t status = ENET_SOK;

    status = CpswAle_validatePriorityPolicer(regs, policerEntryIdx, priority);
    if (status == ENET_SOK)
    {
        CSL_CPSW_ALE_POLICER_ENTRY polEntry;

        CSL_CPSW_getAlePolicerEntry(regs, policerEntryIdx, &polEntry);
        polEntry.validBitmap |= CSL_CPSW_ALE_POLICER_PRI_VALID;
        polEntry.pri = priority;
        CSL_CPSW_setAlePolicerEntry(regs, policerEntryIdx, &polEntry);
    }

    if (status == ENET_SOK)
    {
        status = CpswAle_setPolicerRateLimit(hAle,
                                             regs,
                                             policerEntryIdx,
                                             peakRateInBps,
                                             commitRateInBps,
                                             aleFreq);
    }

    return status;
}

static uint32_t CpswAle_mapNumPktsPerSec2RateLimit(CSL_AleRegs *regs,
                                                   uint32_t aleFreq,
                                                   uint32_t numPktsPerSec)
{
    uint32_t alePrescale = CSL_CPSW_getAlePrescaleReg(regs);
    uint32_t preScalerPulseFreq;

    if (alePrescale == 0)
    {
        alePrescale = 1;
    }

    preScalerPulseFreq = aleFreq / alePrescale;

    return(numPktsPerSec / preScalerPulseFreq);
}

static uint32_t CpswAle_mapRateLimit2NumPktsPerSec(CSL_AleRegs *regs,
                                                   uint32_t aleFreq,
                                                   uint32_t rateLimit)
{
    uint32_t alePrescale = CSL_CPSW_getAlePrescaleReg(regs);
    uint32_t preScalerPulseFreq;

    if (alePrescale == 0)
    {
        alePrescale = 1;
    }

    preScalerPulseFreq = aleFreq / alePrescale;

    return(rateLimit * preScalerPulseFreq);
}

static int32_t CpswAle_getPolicerDefaultThreadConfig(CpswAle_Handle hAle,
                                                     CSL_AleRegs *regs,
                                                     CpswAle_DfltThreadCfg *defaultThreadCfg)
{
    EnetMod_Handle hMod = (EnetMod_Handle)hAle;
    CSL_CPSW_ALE_POLICER_GLOB_CONFIG defThreadConfig = {0};
    CSL_CPSW_ALE_POLICER_CONTROL policerControl;

    CSL_CPSW_getAlePolicerGlobConfig(regs, &defThreadConfig);

    if (ENET_FEAT_IS_EN(hMod->features, CPSW_ALE_FEATURE_FLOW_PRIORITY))
    {
        /* disableMacPort and priorityOrEn are part of policer control register */
        CSL_CPSW_getAlePolicerControlReg(regs, &policerControl);

        defaultThreadCfg->macPortDfltThreadDis = policerControl.disableMacPortDefaultThread;
        defaultThreadCfg->priorityOrEn = policerControl.enablePriorityOR;
    }
    else
    {
        defaultThreadCfg->macPortDfltThreadDis = FALSE;
        defaultThreadCfg->priorityOrEn = FALSE;
    }

    defaultThreadCfg->dfltThreadEn = defThreadConfig.defThreadEnable;
    defaultThreadCfg->threadId = defThreadConfig.defThread;

    return ENET_SOK;
}

static int32_t CpswAle_setPolicerRateLimit(CpswAle_Handle hAle,
                                           CSL_AleRegs *regs,
                                           uint32_t policerEntryIdx,
                                           uint32_t peakRateInBitsPs,
                                           uint32_t commitRateInBitsPs,
                                           uint32_t aleFreq)
{
    CSL_CPSW_ALE_POLICER_ENTRY polEntry;
    uint32_t pir_idl_inc_val = CpswAle_mapBw2IdlIncVal(aleFreq, peakRateInBitsPs);
    uint32_t cir_idl_inc_val = CpswAle_mapBw2IdlIncVal(aleFreq, commitRateInBitsPs);

    CSL_CPSW_getAlePolicerEntry(regs, policerEntryIdx, &polEntry);

    polEntry.validBitmap |= (CSL_CPSW_ALE_POLICER_PIR_VALID |
                             CSL_CPSW_ALE_POLICER_CIR_VALID);
    polEntry.pirIdleIncVal = pir_idl_inc_val;
    polEntry.cirIdleIncVal = cir_idl_inc_val;

    CSL_CPSW_setAlePolicerEntry(regs, policerEntryIdx, &polEntry);
    CpswAle_clearSelectedPolicerStats(regs, policerEntryIdx);

    return ENET_SOK;
}

static void CpswAle_getPolicerStats(CSL_AleRegs *regs,
                                    uint32_t policerIdx,
                                    bool *pPolHit,
                                    bool *pPolRedHit,
                                    bool *pPolYellowHit,
                                    bool clearStats)
{
    CSL_CPSW_ALE_POLICER_TEST_CONTROL policerTstCtrl;
    CSL_CPSW_ALE_POLICER_HSTAT policerHStatCfg = {0};

    policerTstCtrl.polClrallHit       = 0;
    policerTstCtrl.polClrallRedhit    = 0;
    policerTstCtrl.polClrallYellowhit = 0;
    policerTstCtrl.polClrselAll       = 0;
    policerTstCtrl.polTestIdx         = policerIdx; /* Don't care param */
    CSL_CPSW_setAlePolicerTestControlReg(regs, &policerTstCtrl);

    CSL_CPSW_getAlePolicerHstatReg(regs, &policerHStatCfg);
    *pPolHit = policerHStatCfg.polHit;
    *pPolRedHit = policerHStatCfg.polRedhit;
    *pPolYellowHit = policerHStatCfg.polYellowhit;

    if (clearStats)
    {
        policerTstCtrl.polClrselAll = 1;
        policerTstCtrl.polTestIdx = policerIdx; /* Don't care param */
        CSL_CPSW_setAlePolicerTestControlReg(regs, &policerTstCtrl);
    }
}

static int32_t CpswAle_getMirrorMatchIndex(CpswAle_Handle hAle,
                                           CSL_AleRegs *regs,
                                           const CpswAle_MirrorMatchParams *mirrorMatchParams,
                                           uint32_t *entryIdx)
{
    int32_t status = ENET_SOK;

    switch (mirrorMatchParams->entryType)
    {
        case CPSW_ALE_TABLE_ENTRY_TYPE_ADDR:
        {
            const uint8_t addr[ENET_MAC_ADDR_LEN] =
                CPSW_ALE_MAC_ADDR_INIT_LIST(mirrorMatchParams->dstMacAddrInfo.addr.addr);

            status = CpswAle_findAddr(hAle, regs, &addr, mirrorMatchParams->dstMacAddrInfo.addr.vlanId, entryIdx);
            if (status == ENET_ENOTFOUND)
            {
                /* As per spec enabling Mirror Match Entry enables the match
                 * mirror option. When this bit is set any traffic whose
                 * destination, source, VLAN or OUI matches the
                 * mirror_midx entry index will have that traffic also sent to
                 * the mirror_top port.
                 * So address type to dest to allow MCast address to be
                 * mirrored
                 */
                status = CpswAle_addMacAddrEntry(hAle,
                                                 regs,
                                                 &addr,
                                                 mirrorMatchParams->dstMacAddrInfo.addr.vlanId,
                                                 mirrorMatchParams->dstMacAddrInfo.portNum,
                                                 CPSW_ALE_ADDR_TYPE_DST,
                                                 entryIdx);
            }

            break;
        }

        case CPSW_ALE_TABLE_ENTRY_TYPE_VLAN:
            status = CpswAle_findVlan(hAle,
                                      regs,
                                      mirrorMatchParams->vlanIdInfo.vlanId,
                                      mirrorMatchParams->vlanIdInfo.tagType == ENET_VLAN_TAG_TYPE_OUTER,
                                      NULL,
                                      NULL,
                                      NULL,
                                      NULL,
                                      NULL,
                                      NULL,
                                      NULL,
                                      NULL,
                                      entryIdx);
            if (ENET_ENOTFOUND == status)
            {
                status = CpswAle_addVlanEntry(hAle,
                                              regs,
                                              mirrorMatchParams->vlanIdInfo.vlanId,
                                              mirrorMatchParams->vlanIdInfo.tagType == ENET_VLAN_TAG_TYPE_OUTER,
                                              entryIdx);
            }

            break;

        case CPSW_ALE_TABLE_ENTRY_TYPE_ETHERTYPE:
            status = CpswAle_addEtherType(hAle,
                                          regs,
                                          mirrorMatchParams->etherType,
                                          entryIdx,
                                          NULL);
            break;

        case CPSW_ALE_TABLE_ENTRY_TYPE_OUI:
        {
            const uint8_t ouiAddr[ENET_OUI_ADDR_LEN] =
                CPSW_ALE_OUI_ADDR_INIT_LIST(mirrorMatchParams->ouiInfo.ouiAddr);

            status = CpswAle_addOuiAddr(hAle,
                                        regs,
                                        &ouiAddr,
                                        entryIdx,
                                        NULL);
            break;
        }

        case CPSW_ALE_TABLE_ENTRY_TYPE_IPV4:
        {
            const uint8_t ipv4Addr[ENET_IPv4_ADDR_LEN] =
                CPSW_ALE_IPV4_ADDR_INIT_LIST(mirrorMatchParams->ipv4Info.ipv4Addr);

            status = CpswAle_addIPv4Addr(hAle,
                                         regs,
                                         &ipv4Addr,
                                         mirrorMatchParams->ipv4Info.numLSBIgnoreBits,
                                         entryIdx,
                                         NULL);
            break;
        }

        case CPSW_ALE_TABLE_ENTRY_TYPE_IPV6:
        {
            const uint8_t ipv6Addr[ENET_IPv6_ADDR_LEN] =
                CPSW_ALE_IPV6_ADDR_INIT_LIST(mirrorMatchParams->ipv6Info.ipv6Addr);

            status = CpswAle_addIPv6Addr(hAle,
                                         regs,
                                         &ipv6Addr,
                                         mirrorMatchParams->ipv6Info.numLSBIgnoreBits,
                                         entryIdx,
                                         NULL);
            break;
        }

        default:
            status = ENET_EINVALIDPARAMS;
            break;
    }

    return status;
}

static int32_t CpswAle_setPortMirrorConfig(CpswAle_Handle hAle,
                                           CSL_AleRegs *regs,
                                           uint32_t maxNumPorts,
                                           const CpswAle_PortMirroringCfg *portMirrorCfg)
{
    int32_t status = ENET_SOK;
    uint32_t regVal;
    uint32_t i;

    regVal = CSL_CPSW_getAleControlReg(regs);
    if (portMirrorCfg->srcEn)
    {
        bool srcEnPortMirror;
        for (i = 0U; i < maxNumPorts; i++)
        {
            srcEnPortMirror = NOT_ZERO((portMirrorCfg->srcPortNumMask >> i) & 0x1);
            CSL_CPSW_setAlePortMirrorSouce(regs, i, srcEnPortMirror);
        }

        CSL_FINS(regVal, ALE_ALE_CONTROL_MIRROR_SEN, TRUE);
    }
    else
    {
        CSL_FINS(regVal, ALE_ALE_CONTROL_MIRROR_SEN, FALSE);
    }

    if (portMirrorCfg->dstEnEn)
    {
        CSL_FINS(regVal, ALE_ALE_CONTROL_MIRROR_DP, portMirrorCfg->dstPortNum);
        CSL_FINS(regVal, ALE_ALE_CONTROL_MIRROR_DEN, TRUE);
    }
    else
    {
        CSL_FINS(regVal, ALE_ALE_CONTROL_MIRROR_DEN, FALSE);
    }

    if (portMirrorCfg->matchEn)
    {
        uint32_t mirrorMatchIndex;

        status = CpswAle_getMirrorMatchIndex(hAle,
                                             regs,
                                             &portMirrorCfg->matchParams,
                                             &mirrorMatchIndex);
        if (status == ENET_SOK)
        {
            CSL_CPSW_setAleCtrl2MirrorMatchIndex(regs,
                                                 mirrorMatchIndex);
            CSL_FINS(regVal, ALE_ALE_CONTROL_MIRROR_MEN, TRUE);
        }
    }
    else
    {
        CSL_FINS(regVal, ALE_ALE_CONTROL_MIRROR_MEN, FALSE);
    }

    CSL_FINS(regVal, ALE_ALE_CONTROL_MIRROR_TOP, portMirrorCfg->toPortNum);
    CSL_CPSW_setAleControlReg(regs, regVal);

    return status;
}

static int32_t CpswAle_setPortTrunkConfig(CpswAle_Handle hAle,
                                          CSL_AleRegs *regs,
                                          const CpswAle_TrunkCfg *trunkCfg)
{
    CSL_CPSW_ALE_CTRL2_TRUNK_CONFIG cslTrunkCfg;
    uint32_t i;

    for (i = 0U; i < trunkCfg->numPorts; i++)
    {
        CSL_CPSW_setAlePortControlTrunk(regs, trunkCfg->trunkPortIdx[i], TRUE, trunkCfg->trunkId);
    }

    cslTrunkCfg.trunkBase            = trunkCfg->trunkHashBase;
    cslTrunkCfg.trunkEnableDestIP    = trunkCfg->dstIPEn;
    cslTrunkCfg.trunkEnableSrcIP     = trunkCfg->srcIPEn;
    cslTrunkCfg.trunkEnableInnerVLAN = trunkCfg->innerVlanEn;
    cslTrunkCfg.trunkEnablePri       = trunkCfg->enablePri;
    cslTrunkCfg.trunkEnableSrc       = trunkCfg->srcEn;
    cslTrunkCfg.trunkEnableDst       = trunkCfg->dstEnEn;
    CSL_CPSW_setAleCtrl2TrunkParams(regs, &cslTrunkCfg);

    return ENET_SOK;
}

static int32_t CpswAle_allocPolicerEntry(CSL_AleRegs *regs,
                                         uint32_t *freePolicerEntry)
{
    CSL_CPSW_ALE_POLICER_ENTRY policerEntry;
    uint32_t numPolicerEntries;
    int32_t status;
    uint32_t i;

    CSL_CPSW_getAleStatusNumPolicers(regs, &numPolicerEntries);
    for (i = 0U; i < numPolicerEntries; i++)
    {
        CSL_CPSW_getAlePolicerEntry(regs, i, &policerEntry);
        if (policerEntry.validBitmap == 0)
        {
            break;
        }
    }

    if (i < numPolicerEntries)
    {
        *freePolicerEntry = i;
        status = ENET_SOK;
    }
    else
    {
        status = ENET_EALLOC;
    }

    return status;
}

static int32_t CpswAle_setPortBcastMcastLimit(CSL_AleRegs *regs,
                                              uint32_t aleFreqHz,
                                              uint32_t portNum,
                                              bool bcastRateLimitForPortEn,
                                              uint32_t bcastLimitNumPktsPerSec,
                                              bool mcastRateLimitForPortEn,
                                              uint32_t mcastLimitNumPktsPerSec)
{
    CSL_CPSW_ALE_PORTCONTROL alePortControl;
    int32_t status = ENET_SOK;

    CSL_CPSW_getAlePortControlReg(regs, portNum, &alePortControl);
    if (bcastRateLimitForPortEn)
    {
        alePortControl.bcastLimit = CpswAle_mapNumPktsPerSec2RateLimit(regs,
                                                                       aleFreqHz,
                                                                       bcastLimitNumPktsPerSec);
    }
    else
    {
        alePortControl.bcastLimit = 0;
    }

    if (mcastRateLimitForPortEn)
    {
        alePortControl.mcastLimit = CpswAle_mapNumPktsPerSec2RateLimit(regs,
                                                                       aleFreqHz,
                                                                       mcastLimitNumPktsPerSec);
    }
    else
    {
        alePortControl.mcastLimit = 0;
    }

    CSL_CPSW_setAlePortControlReg(regs, portNum, &alePortControl);

    return status;
}

static int32_t CpswAle_getPortBcastMcastLimit(CSL_AleRegs *regs,
                                              uint32_t aleFreqHz,
                                              uint32_t portNum,
                                              bool *pBRateLimitEnableForPort,
                                              uint32_t *pBcastLimitNumPktsPerSec,
                                              bool *pMRateLimitEnableForPort,
                                              uint32_t *pMcastLimitNumPktsPerSec)
{
    CSL_CPSW_ALE_PORTCONTROL alePortControl;
    int32_t status = ENET_SOK;

    CSL_CPSW_getAlePortControlReg(regs, portNum, &alePortControl);
    if (alePortControl.bcastLimit != 0)
    {
        *pBcastLimitNumPktsPerSec = CpswAle_mapRateLimit2NumPktsPerSec(regs,
                                                                       aleFreqHz,
                                                                       alePortControl.bcastLimit);
        *pBRateLimitEnableForPort = TRUE;
    }
    else
    {
        *pBcastLimitNumPktsPerSec = 0;
        *pBRateLimitEnableForPort = FALSE;
    }

    if (alePortControl.mcastLimit != 0)
    {
        *pMcastLimitNumPktsPerSec = CpswAle_mapRateLimit2NumPktsPerSec(regs,
                                                                       aleFreqHz,
                                                                       alePortControl.mcastLimit);
        *pMRateLimitEnableForPort = TRUE;
    }
    else
    {
        *pMcastLimitNumPktsPerSec = 0;
        *pMRateLimitEnableForPort = FALSE;
    }

    return status;
}

static void CpswAle_dumpPolicerEntries(CpswAle_Handle hAle,
                                       CSL_AleRegs *regs)
{
    uint32_t i, freeEntries;
    uint32_t numPolicerEntries;
    bool policerHit, policerRedHit, policerYellowHit;
    CSL_CPSW_ALE_POLICER_ENTRY policerEntry;
    bool trunkIndicator;

    EnetUtils_printf("\r\n");

    CSL_CPSW_getAleStatusNumPolicers(regs, &numPolicerEntries);

    freeEntries = 0;
    for (i = 0U; i < numPolicerEntries; i++)
    {
        CSL_CPSW_getAlePolicerEntry(regs, i, &policerEntry);
        if (policerEntry.validBitmap & CSL_CPSW_ALE_POLICER_PORT_VALID)
        {
            if (policerEntry.validBitmap &
                CSL_CPSW_ALE_POLICER_PORT_TRUNK_VALID)
            {
                trunkIndicator = TRUE;
            }
            else
            {
                trunkIndicator = FALSE;
            }

            EnetUtils_printf(" %4d: POLICER_PORT, PORT_NUM: %d ISTRUNK:%d\r\n",
                             i, policerEntry.port, trunkIndicator);
        }

        if (policerEntry.validBitmap & CSL_CPSW_ALE_POLICER_PRI_VALID)
        {
            EnetUtils_printf(" %4d: POLICER_PRI, PRI:%d",
                             i, policerEntry.pri);
        }

        if (policerEntry.validBitmap & CSL_CPSW_ALE_POLICER_OUI_VALID)
        {
            EnetUtils_printf(" %4d: POLICER_OUI, ALE Index: %d\r\n",
                             i, policerEntry.ouiIdx);
        }

        if (policerEntry.validBitmap & CSL_CPSW_ALE_POLICER_DST_MAC_VALID)
        {
            EnetUtils_printf(" %4d: POLICER_DST_MAC,ALE Index: %d\r\n",
                             i, policerEntry.dstMacIdx);
        }

        if (policerEntry.validBitmap & CSL_CPSW_ALE_POLICER_SRC_MAC_VALID)
        {
            EnetUtils_printf(" %4d: POLICER_SRC_MAC,ALE Index: %d\r\n",
                             i, policerEntry.srcMacIdx);
        }

        if (policerEntry.validBitmap & CSL_CPSW_ALE_POLICER_OVLAN_VALID)
        {
            EnetUtils_printf(" %4d: POLICER_OVLAN,ALE Index: %d\r\n",
                             i, policerEntry.ovlanIdx);
        }

        if (policerEntry.validBitmap & CSL_CPSW_ALE_POLICER_VLAN_VALID)
        {
            EnetUtils_printf(" %4d: POLICER_VLAN,ALE Index: %d\r\n",
                             i, policerEntry.vlanIdx);
        }

        if (policerEntry.validBitmap & CSL_CPSW_ALE_POLICER_ETHERTYPE_VALID)
        {
            EnetUtils_printf(" %4d: POLICER_ETHERTYPE,ALE Index: %d\r\n",
                             i, policerEntry.ethertypeIdx);
        }

        if (policerEntry.validBitmap & CSL_CPSW_ALE_POLICER_SRC_IP_VALID)
        {
            EnetUtils_printf(" %4d: POLICER_SRC_IP,ALE Index: %d\r\n",
                             i, policerEntry.srcIpIdx);
        }

        if (policerEntry.validBitmap & CSL_CPSW_ALE_POLICER_DST_IP_VALID)
        {
            EnetUtils_printf(" %4d: POLICER_DST_IP,ALE Index: %d\r\n",
                             i, policerEntry.dstIpIdx);
        }

        if (policerEntry.validBitmap & CSL_CPSW_ALE_POLICER_THREAD_VALID)
        {
            EnetUtils_printf(" %4d: POLICER_THREAD,THREAD ID:%d\r\n",
                             i, policerEntry.thread);
        }

        if (policerEntry.validBitmap & CSL_CPSW_ALE_POLICER_PIR_VALID)
        {
            if (policerEntry.pirIdleIncVal)
            {
                EnetUtils_printf(" %4d: POLICER_PIR_IDL_INC_VAL: %d\r\n",
                                 i, policerEntry.pirIdleIncVal);
            }
        }

        if (policerEntry.validBitmap & CSL_CPSW_ALE_POLICER_CIR_VALID)
        {
            if (policerEntry.cirIdleIncVal)
            {
                EnetUtils_printf(" %4d: POLICER_CIR_IDL_INC_VAL: %d\r\n",
                                 i, policerEntry.cirIdleIncVal);
            }
        }

        if (policerEntry.validBitmap & CSL_CPSW_ALE_POLICER_EGRESSOP_VALID)
        {
            EnetUtils_printf(" %4d: "
                             "EGRESS_OP: %x \t"
                             "TRUNK_IDX: %d \t"
                             "TTL_CHECK: %d \t"
                             "DEST_PORT_MASK: %x\r\n",
                             i,
                             policerEntry.egressOp,
                             policerEntry.egressTrunkIndex,
                             policerEntry.enableTTLCheck,
                             policerEntry.destPortMask);
        }

        if (policerEntry.validBitmap == 0)
        {
            freeEntries++;
        }
        else
        {
            CpswAle_getPolicerStats(regs, i, &policerHit,
                                    &policerRedHit, &policerYellowHit,
                                    FALSE);
            EnetUtils_printf(" %4d: POLICER_STATS: Hit: %d, RedHit: %d, YellowHit: %d\r\n",
                             i, policerHit, policerRedHit, policerYellowHit);
        }
    }

    EnetUtils_printf("\n\r");
    EnetUtils_printf("%d Free Entries \r\n", freeEntries);
}

static int32_t CpswAle_delOuiAddr(CpswAle_Handle hAle,
                                  CSL_AleRegs *regs,
                                  const CpswAle_OuiAddrType ouiAddr)
{
    uint32_t entryIdx;
    int32_t status = ENET_ENOTFOUND;

    status = CpswAle_findOuiAddr(hAle, regs, ouiAddr, &entryIdx);
    if (status == ENET_SOK)
    {
        CpswAle_clearTableEntry(regs, entryIdx);
    }

    return status;
}

static int32_t CpswAle_delIPv4Addr(CpswAle_Handle hAle,
                                   CSL_AleRegs *regs,
                                   CpswAle_IPv4AddrType ipv4Addr,
                                   uint32_t numLSBIgnore)
{
    uint32_t entryIdx;
    int32_t status = ENET_ENOTFOUND;

    status = CpswAle_findIPv4Addr(hAle, regs, ipv4Addr, numLSBIgnore, &entryIdx);
    if (status == ENET_SOK)
    {
        CpswAle_clearTableEntry(regs, entryIdx);
    }

    return status;
}

static int32_t CpswAle_delIPv6Addr(CpswAle_Handle hAle,
                                   CSL_AleRegs *regs,
                                   CpswAle_IPv6AddrType ipv6Addr,
                                   uint32_t numLSBIgnore)
{
    uint32_t entryIdx;
    int32_t status = ENET_ENOTFOUND;

    status = CpswAle_findIPv6Addr(hAle, regs, ipv6Addr, numLSBIgnore, &entryIdx);
    if (status == ENET_SOK)
    {
        CpswAle_clearTableEntry(regs, entryIdx);
    }

    return status;
}

static int32_t CpswAle_delEtherType(CpswAle_Handle hAle,
                                    CSL_AleRegs *regs,
                                    uint32_t etherType)
{
    uint32_t entryIdx;
    int32_t status = ENET_ENOTFOUND;

    status = CpswAle_findEtherType(hAle, regs, etherType, &entryIdx);
    if (status == ENET_SOK)
    {
        CpswAle_clearTableEntry(regs, entryIdx);
    }

    return status;
}

static int32_t CpswAle_setPolicer(CpswAle_Handle hAle,
                                  CSL_AleRegs *regs,
                                  const CpswAle_PolicerMatchParams *policerMatchParams,
                                  uint32_t peakRateInBitsPerSec,
                                  uint32_t commitRateInBitsPerSec,
                                  uint32_t aleFreqHz,
                                  bool threadIdEn,
                                  uint32_t threadId,
                                  CpswAle_SetPolicerEntryOutArgs *outArgs)
{
    uint32_t policerEntryIdx = (uint32_t)ENET_EALLOC;
    uint32_t aleFreeEntryMask;
    bool policerAlloced;
    int32_t status = ENET_SOK;

    policerAlloced = FALSE;
    aleFreeEntryMask = 0;
    if (NOT_ZERO(policerMatchParams->policerMatchEnMask))
    {
        status = CpswAle_allocPolicerEntry(regs, &policerEntryIdx);
        if (status == ENET_SOK)
        {
            outArgs->policerEntryIdx = policerEntryIdx;
            policerAlloced = TRUE;
        }
    }

    if (status == ENET_SOK)
    {
        if (NOT_ZERO(policerMatchParams->policerMatchEnMask & CPSW_ALE_POLICER_MATCH_OUI))
        {
            bool newEntry;
            const CpswAle_OuiEntryInfo *ouiEntryInfo = &policerMatchParams->ouiInfo;
            const uint8_t ouiAddr[ENET_OUI_ADDR_LEN] =
                CPSW_ALE_OUI_ADDR_INIT_LIST(ouiEntryInfo->ouiAddr);

            status = CpswAle_setOuiPolicerEntry(hAle,
                                                regs,
                                                policerEntryIdx,
                                                &ouiAddr,
                                                peakRateInBitsPerSec,
                                                commitRateInBitsPerSec,
                                                aleFreqHz,
                                                &outArgs->ouiAleEntryIdx,
                                                &newEntry);
            if (newEntry)
            {
                aleFreeEntryMask |= CPSW_ALE_POLICER_TABLEENTRY_DELETE_OUI;
            }
        }
    }

    if (status == ENET_SOK)
    {
        if (NOT_ZERO(policerMatchParams->policerMatchEnMask & CPSW_ALE_POLICER_MATCH_IPSRC))
        {
            bool newEntry;

            status = CpswAle_setIPSrcDstPolicerEntry(hAle,
                                                     regs,
                                                     policerEntryIdx,
                                                     &policerMatchParams->srcIpInfo,
                                                     CPSW_ALE_ADDR_TYPE_SRC,
                                                     peakRateInBitsPerSec,
                                                     commitRateInBitsPerSec,
                                                     aleFreqHz,
                                                     &outArgs->srcIpAleEntryIdx,
                                                     &newEntry);
            if (newEntry)
            {
                aleFreeEntryMask |= CPSW_ALE_POLICER_TABLEENTRY_DELETE_IPSRC;
            }
        }
    }

    if (status == ENET_SOK)
    {
        if (NOT_ZERO(policerMatchParams->policerMatchEnMask & CPSW_ALE_POLICER_MATCH_IPDST))
        {
            bool newEntry;

            status = CpswAle_setIPSrcDstPolicerEntry(hAle,
                                                     regs,
                                                     policerEntryIdx,
                                                     &policerMatchParams->dstIpInfo,
                                                     CPSW_ALE_ADDR_TYPE_DST,
                                                     peakRateInBitsPerSec,
                                                     commitRateInBitsPerSec,
                                                     aleFreqHz,
                                                     &outArgs->dstIpAleEntryIdx,
                                                     &newEntry);
            if (newEntry)
            {
                aleFreeEntryMask |= CPSW_ALE_POLICER_TABLEENTRY_DELETE_IPDST;
            }
        }
    }

    if (status == ENET_SOK)
    {
        if (NOT_ZERO(policerMatchParams->policerMatchEnMask & CPSW_ALE_POLICER_MATCH_ETHERTYPE))
        {
            bool newEntry;

            status = CpswAle_setEtherTypePolicerEntry(hAle,
                                                      regs,
                                                      policerEntryIdx,
                                                      policerMatchParams->etherType,
                                                      peakRateInBitsPerSec,
                                                      commitRateInBitsPerSec,
                                                      aleFreqHz,
                                                      &outArgs->etherTypeAleEntryIdx,
                                                      &newEntry);
            if (newEntry)
            {
                aleFreeEntryMask |= CPSW_ALE_POLICER_TABLEENTRY_DELETE_ETHERTYPE;
            }
        }
    }

    if (status == ENET_SOK)
    {
        if (NOT_ZERO(policerMatchParams->policerMatchEnMask & CPSW_ALE_POLICER_MATCH_MACSRC))
        {
            bool newEntry;
            const uint8_t macAddr[ENET_MAC_ADDR_LEN] =
                CPSW_ALE_MAC_ADDR_INIT_LIST(policerMatchParams->srcMacAddrInfo.addr.addr);

            status = CpswAle_setMacAddrPolicerEntry(hAle,
                                                    regs,
                                                    policerEntryIdx,
                                                    &macAddr,
                                                    policerMatchParams->srcMacAddrInfo.addr.vlanId,
                                                    policerMatchParams->srcMacAddrInfo.portNum,
                                                    CPSW_ALE_ADDR_TYPE_SRC,
                                                    peakRateInBitsPerSec,
                                                    commitRateInBitsPerSec,
                                                    aleFreqHz,
                                                    &outArgs->srcMacAleEntryIdx,
                                                    &newEntry);
            if (newEntry)
            {
                aleFreeEntryMask |= CPSW_ALE_POLICER_TABLEENTRY_DELETE_MACSRC;
            }
        }
    }

    if (status == ENET_SOK)
    {
        if (NOT_ZERO(policerMatchParams->policerMatchEnMask & CPSW_ALE_POLICER_MATCH_MACDST))
        {
            bool newEntry;
            const uint8_t macAddr[ENET_MAC_ADDR_LEN] =
                CPSW_ALE_MAC_ADDR_INIT_LIST(policerMatchParams->dstMacAddrInfo.addr.addr);

            status = CpswAle_setMacAddrPolicerEntry(hAle,
                                                    regs,
                                                    policerEntryIdx,
                                                    &macAddr,
                                                    policerMatchParams->dstMacAddrInfo.addr.vlanId,
                                                    policerMatchParams->dstMacAddrInfo.portNum,
                                                    CPSW_ALE_ADDR_TYPE_DST,
                                                    peakRateInBitsPerSec,
                                                    commitRateInBitsPerSec,
                                                    aleFreqHz,
                                                    &outArgs->dstMacAleEntryIdx,
                                                    &newEntry);
            if (newEntry)
            {
                aleFreeEntryMask |= CPSW_ALE_POLICER_TABLEENTRY_DELETE_MACDST;
            }
        }
    }

    if (status == ENET_SOK)
    {
        if (NOT_ZERO(policerMatchParams->policerMatchEnMask & CPSW_ALE_POLICER_MATCH_IVLAN))
        {
            bool newEntry;

            status = CpswAle_setVlanPolicerEntry(hAle,
                                                 regs,
                                                 policerEntryIdx,
                                                 policerMatchParams->ivlanId,
                                                 FALSE,
                                                 peakRateInBitsPerSec,
                                                 commitRateInBitsPerSec,
                                                 aleFreqHz,
                                                 &outArgs->ivlanAleEntryIdx,
                                                 &newEntry);
            if (newEntry)
            {
                aleFreeEntryMask |= CPSW_ALE_POLICER_TABLEENTRY_DELETE_IVLAN;
            }
        }
    }

    if (status == ENET_SOK)
    {
        if (NOT_ZERO(policerMatchParams->policerMatchEnMask & CPSW_ALE_POLICER_MATCH_OVLAN))
        {
            bool newEntry;

            status = CpswAle_setVlanPolicerEntry(hAle,
                                                 regs,
                                                 policerEntryIdx,
                                                 policerMatchParams->ovlanId,
                                                 TRUE,
                                                 peakRateInBitsPerSec,
                                                 commitRateInBitsPerSec,
                                                 aleFreqHz,
                                                 &outArgs->ovlanAleEntryIdx,
                                                 &newEntry);
            if (newEntry)
            {
                aleFreeEntryMask |= CPSW_ALE_POLICER_TABLEENTRY_DELETE_OVLAN;
            }
        }
    }

    if (status == ENET_SOK)
    {
        if (NOT_ZERO(policerMatchParams->policerMatchEnMask & CPSW_ALE_POLICER_MATCH_PORT))
        {
            status = CpswAle_setPortPolicerEntry(hAle,
                                                 regs,
                                                 policerEntryIdx,
                                                 policerMatchParams->portNum,
                                                 policerMatchParams->portIsTrunk,
                                                 peakRateInBitsPerSec,
                                                 commitRateInBitsPerSec,
                                                 aleFreqHz);
        }
    }

    if (status == ENET_SOK)
    {
        if (NOT_ZERO(policerMatchParams->policerMatchEnMask & CPSW_ALE_POLICER_MATCH_PRIORITY))
        {
            status = CpswAle_setPriorityPolicerEntry(hAle,
                                                     regs,
                                                     policerEntryIdx,
                                                     policerMatchParams->priority,
                                                     peakRateInBitsPerSec,
                                                     commitRateInBitsPerSec,
                                                     aleFreqHz);
        }
    }

    if (status == ENET_SOK)
    {
        if (threadIdEn)
        {
            status = CpswAle_setThreadPolicerConfig(regs,
                                                    policerEntryIdx,
                                                    threadId);
        }
    }

    if ((status != ENET_SOK) && policerAlloced)
    {
        CpswAle_delPolicerEntry(hAle, regs, policerEntryIdx, aleFreeEntryMask);
    }

    return status;
}

static int32_t CpswAle_setIPSrcDstPolicerEntry(CpswAle_Handle hAle,
                                               CSL_AleRegs *regs,
                                               uint32_t policerEntryIdx,
                                               const CpswAle_IpAddrClassifierInfo *ipClassifier,
                                               CpswAle_AddrType addrType,
                                               uint32_t peakRateInBitsPerSec,
                                               uint32_t commitRateInBitsPerSec,
                                               uint32_t aleFreqHz,
                                               uint32_t *entryIdx,
                                               bool *alloced)
{
    int32_t status = ENET_SOK;

    if (CPSW_ALE_IPADDR_CLASSIFIER_IPV4 == ipClassifier->ipAddrType)
    {
        const uint8_t ipv4Addr[ENET_IPv4_ADDR_LEN] =
            CPSW_ALE_IPV4_ADDR_INIT_LIST(ipClassifier->ipv4Info.ipv4Addr);

        status = CpswAle_setIPv4PolicerEntry(hAle,
                                             regs,
                                             policerEntryIdx,
                                             &ipv4Addr,
                                             addrType,
                                             ipClassifier->ipv4Info.numLSBIgnoreBits,
                                             peakRateInBitsPerSec,
                                             commitRateInBitsPerSec,
                                             aleFreqHz,
                                             entryIdx,
                                             alloced);
    }
    else
    {
        const uint8_t ipv6Addr[ENET_IPv6_ADDR_LEN] =
            CPSW_ALE_IPV6_ADDR_INIT_LIST(ipClassifier->ipv6Info.ipv6Addr);

        status = CpswAle_setIPv6PolicerEntry(hAle,
                                             regs,
                                             policerEntryIdx,
                                             &ipv6Addr,
                                             addrType,
                                             ipClassifier->ipv6Info.numLSBIgnoreBits,
                                             peakRateInBitsPerSec,
                                             commitRateInBitsPerSec,
                                             aleFreqHz,
                                             entryIdx,
                                             alloced);
    }

    return status;
}

static int32_t CpswAle_addMacAddrEntry(CpswAle_Handle hAle,
                                       CSL_AleRegs *regs,
                                       CpswAle_MacAddrType addr,
                                       uint32_t vlanId,
                                       uint32_t portNum,
                                       CpswAle_AddrType addrType,
                                       uint32_t *entryIdx)
{
    int32_t status;

    if (EnetUtils_isMcastAddr((const uint8_t *)addr) != true)
    {
        status = CpswAle_addUcastAddr(hAle,
                                      regs,
                                      portNum,
                                      addr,
                                      vlanId,
                                      CPSW_ALE_UCAST_ENTRY_DFLT_BLOCKEDFLAG,
                                      CPSW_ALE_UCAST_ENTRY_DFLT_SECUREFLAG,
                                      CPSW_ALE_UCAST_ENTRY_DFLT_AGEABLEFLAG,
                                      CPSW_ALE_UCAST_ENTRY_DFLT_TRUNKFLAG,
                                      entryIdx);
    }
    else
    {
        if (CPSW_ALE_ADDR_TYPE_SRC == addrType)
        {
            /* Multicast cannot be source address */
            status = ENET_EINVALIDPARAMS;
        }
        else
        {
            status = CpswAle_addMcastAddr(hAle,
                                          regs,
                                          addr,
                                          vlanId,
                                          ENET_BIT(portNum),
                                          CPSW_ALE_MCAST_ENTRY_DFLT_SUPERFLAG,
                                          CPSW_ALE_MCAST_ENTRY_DFLT_FWDSTATE,
                                          CPSW_ALE_MCAST_ENTRY_DFLT_IGNBITS,
                                          entryIdx);
        }
    }

    return status;
}

static int32_t CpswAle_addVlanEntry(CpswAle_Handle hAle,
                                    CSL_AleRegs *regs,
                                    uint32_t vlanId,
                                    bool outVlanFlag,
                                    uint32_t *entryIdx)
{
    int32_t status;
    uint32_t unknownVlanMemberList, unknownForceUntaggedEgress;
    uint32_t unknownRegMcastFloodMask, unknownMcastFloodMask;

    /* For adding vlan entry without info from application regarding
     * vlan port membership, use unknwon vlan membership mask values */
    CSL_CPSW_getAleUnkownVlanReg(regs,
                                 &unknownVlanMemberList,
                                 &unknownMcastFloodMask,
                                 &unknownRegMcastFloodMask,
                                 &unknownForceUntaggedEgress);

    status = CpswAle_addVlan(hAle,
                             regs,
                             vlanId,
                             outVlanFlag,
                             unknownVlanMemberList,
                             unknownMcastFloodMask,
                             unknownRegMcastFloodMask,
                             unknownForceUntaggedEgress,
                             CPSW_ALE_VLAN_ENTRY_DFLT_NOLEARNMASK,
                             CPSW_ALE_VLAN_ENTRY_DFLT_VIDINGRESSCHK,
                             CPSW_ALE_VLAN_ENTRY_DFLT_LIMITIPNXTHDR,
                             CPSW_ALE_VLAN_ENTRY_DFLT_DISALLOWFRAG,
                             entryIdx);
    return status;
}

static bool CpswAle_checkPolicerMatch(CpswAle_Handle hAle,
                                      CSL_AleRegs *regs,
                                      CSL_CPSW_ALE_POLICER_ENTRY *policerEntry,
                                      const CpswAle_PolicerMatchParams *policerMatchParams)
{
    bool matchFound = TRUE;
    uint32_t entryIdx;
    uint32_t status;

    if (policerMatchParams->policerMatchEnMask & CPSW_ALE_POLICER_MATCH_PORT)
    {
        if (policerEntry->port != policerMatchParams->portNum)
        {
            matchFound = FALSE;
        }
    }

    if (policerMatchParams->policerMatchEnMask & CPSW_ALE_POLICER_MATCH_PRIORITY)
    {
        if (policerEntry->pri != policerMatchParams->priority)
        {
            matchFound = FALSE;
        }
    }

    if (policerMatchParams->policerMatchEnMask & CPSW_ALE_POLICER_MATCH_OUI)
    {
        const uint8_t ouiAddr[ENET_OUI_ADDR_LEN] =
            CPSW_ALE_OUI_ADDR_INIT_LIST(policerMatchParams->ouiInfo.ouiAddr);

        status = CpswAle_findOuiAddr(hAle, regs, &ouiAddr, &entryIdx);
        if ((status != ENET_SOK) ||
            (policerEntry->ouiIdx != entryIdx))
        {
            matchFound = FALSE;
        }
    }

    if (policerMatchParams->policerMatchEnMask & CPSW_ALE_POLICER_MATCH_MACSRC)
    {
        const uint8_t addr[ENET_MAC_ADDR_LEN] =
            CPSW_ALE_MAC_ADDR_INIT_LIST(policerMatchParams->srcMacAddrInfo.addr.addr);

        status = CpswAle_findAddr(hAle,
                                  regs,
                                  &addr,
                                  policerMatchParams->srcMacAddrInfo.addr.vlanId,
                                  &entryIdx);
        if ((status != ENET_SOK) ||
            (policerEntry->srcMacIdx != entryIdx))
        {
            matchFound = FALSE;
        }
    }

    if (policerMatchParams->policerMatchEnMask & CPSW_ALE_POLICER_MATCH_MACDST)
    {
        const uint8_t addr[ENET_MAC_ADDR_LEN] =
            CPSW_ALE_MAC_ADDR_INIT_LIST(policerMatchParams->dstMacAddrInfo.addr.addr);

        status = CpswAle_findAddr(hAle,
                                  regs,
                                  &addr,
                                  policerMatchParams->dstMacAddrInfo.addr.vlanId,
                                  &entryIdx);
        if ((status != ENET_SOK) ||
            (policerEntry->dstMacIdx != entryIdx))
        {
            matchFound = FALSE;
        }
    }

    if (policerMatchParams->policerMatchEnMask & CPSW_ALE_POLICER_MATCH_IVLAN)
    {
        status = CpswAle_findVlan(hAle,
                                  regs,
                                  policerMatchParams->ivlanId,
                                  FALSE,
                                  NULL,
                                  NULL,
                                  NULL,
                                  NULL,
                                  NULL,
                                  NULL,
                                  NULL,
                                  NULL,
                                  &entryIdx);
        if ((status != ENET_SOK) ||
            (policerEntry->vlanIdx != entryIdx))
        {
            matchFound = FALSE;
        }
    }

    if (policerMatchParams->policerMatchEnMask & CPSW_ALE_POLICER_MATCH_OVLAN)
    {
        status = CpswAle_findVlan(hAle,
                                  regs,
                                  policerMatchParams->ovlanId,
                                  TRUE,
                                  NULL,
                                  NULL,
                                  NULL,
                                  NULL,
                                  NULL,
                                  NULL,
                                  NULL,
                                  NULL,
                                  &entryIdx);
        if ((status != ENET_SOK) ||
            (policerEntry->ovlanIdx != entryIdx))
        {
            matchFound = FALSE;
        }
    }

    if (policerMatchParams->policerMatchEnMask & CPSW_ALE_POLICER_MATCH_ETHERTYPE)
    {
        status = CpswAle_findEtherType(hAle, regs, policerMatchParams->etherType, &entryIdx);
        if ((status != ENET_SOK) ||
            (policerEntry->ethertypeIdx != entryIdx))
        {
            matchFound = FALSE;
        }
    }

    if (policerMatchParams->policerMatchEnMask & CPSW_ALE_POLICER_MATCH_IPSRC)
    {
        if (policerMatchParams->srcIpInfo.ipAddrType == CPSW_ALE_IPADDR_CLASSIFIER_IPV4)
        {
            const uint8_t ipv4Addr[ENET_IPv4_ADDR_LEN] =
                CPSW_ALE_IPV4_ADDR_INIT_LIST(policerMatchParams->srcIpInfo.ipv4Info.ipv4Addr);

            status = CpswAle_findIPv4Addr(hAle,
                                          regs,
                                          &ipv4Addr,
                                          policerMatchParams->srcIpInfo.ipv4Info.numLSBIgnoreBits,
                                          &entryIdx);
            if ((status != ENET_SOK) ||
                (policerEntry->srcIpIdx != entryIdx))
            {
                matchFound = FALSE;
            }
        }
        else
        {
            const uint8_t ipv6Addr[ENET_IPv6_ADDR_LEN] =
                CPSW_ALE_IPV6_ADDR_INIT_LIST(policerMatchParams->srcIpInfo.ipv6Info.ipv6Addr);

            status = CpswAle_findIPv6Addr(hAle,
                                          regs,
                                          &ipv6Addr,
                                          policerMatchParams->srcIpInfo.ipv6Info.numLSBIgnoreBits,
                                          &entryIdx);
            if ((status != ENET_SOK) ||
                (policerEntry->srcIpIdx != entryIdx))
            {
                matchFound = FALSE;
            }
        }
    }

    if (policerMatchParams->policerMatchEnMask & CPSW_ALE_POLICER_MATCH_IPDST)
    {
        if (policerMatchParams->dstIpInfo.ipAddrType == CPSW_ALE_IPADDR_CLASSIFIER_IPV4)
        {
            const uint8_t ipv4Addr[ENET_IPv4_ADDR_LEN] =
                CPSW_ALE_IPV4_ADDR_INIT_LIST(policerMatchParams->dstIpInfo.ipv4Info.ipv4Addr);

            status = CpswAle_findIPv4Addr(hAle,
                                          regs,
                                          &ipv4Addr,
                                          policerMatchParams->dstIpInfo.ipv4Info.numLSBIgnoreBits,
                                          &entryIdx);
            if ((status != ENET_SOK) ||
                (policerEntry->dstIpIdx != entryIdx))
            {
                matchFound = FALSE;
            }
        }
        else
        {
            const uint8_t ipv6Addr[ENET_IPv6_ADDR_LEN] =
                CPSW_ALE_IPV6_ADDR_INIT_LIST(policerMatchParams->dstIpInfo.ipv6Info.ipv6Addr);

            status = CpswAle_findIPv6Addr(hAle,
                                          regs,
                                          &ipv6Addr,
                                          policerMatchParams->dstIpInfo.ipv6Info.numLSBIgnoreBits,
                                          &entryIdx);
            if ((status != ENET_SOK) ||
                (policerEntry->dstIpIdx != entryIdx))
            {
                matchFound = FALSE;
            }
        }
    }

    return matchFound;
}

static uint32_t CpswAle_mapPolicer2CslMask(uint32_t policerMatchEnMask)
{
    uint32_t matchBitmap;

    ENET_UTILS_COMPILETIME_ASSERT(CPSW_ALE_POLICER_MATCH_PORT == CSL_CPSW_ALE_POLICER_PORT_VALID);
    ENET_UTILS_COMPILETIME_ASSERT(CPSW_ALE_POLICER_MATCH_PRIORITY == CSL_CPSW_ALE_POLICER_PRI_VALID);
    ENET_UTILS_COMPILETIME_ASSERT(CPSW_ALE_POLICER_MATCH_OUI == CSL_CPSW_ALE_POLICER_OUI_VALID);
    ENET_UTILS_COMPILETIME_ASSERT(CPSW_ALE_POLICER_MATCH_MACSRC == CSL_CPSW_ALE_POLICER_SRC_MAC_VALID);
    ENET_UTILS_COMPILETIME_ASSERT(CPSW_ALE_POLICER_MATCH_MACDST == CSL_CPSW_ALE_POLICER_DST_MAC_VALID);
    ENET_UTILS_COMPILETIME_ASSERT(CPSW_ALE_POLICER_MATCH_IVLAN == CSL_CPSW_ALE_POLICER_VLAN_VALID);
    ENET_UTILS_COMPILETIME_ASSERT(CPSW_ALE_POLICER_MATCH_OVLAN == CSL_CPSW_ALE_POLICER_OVLAN_VALID);
    ENET_UTILS_COMPILETIME_ASSERT(CPSW_ALE_POLICER_MATCH_ETHERTYPE == CSL_CPSW_ALE_POLICER_ETHERTYPE_VALID);
    ENET_UTILS_COMPILETIME_ASSERT(CPSW_ALE_POLICER_MATCH_IPSRC == CSL_CPSW_ALE_POLICER_SRC_IP_VALID);
    ENET_UTILS_COMPILETIME_ASSERT(CPSW_ALE_POLICER_MATCH_IPDST == CSL_CPSW_ALE_POLICER_DST_IP_VALID);

    matchBitmap = 0;

    if (NOT_ZERO(policerMatchEnMask & CPSW_ALE_POLICER_MATCH_PORT))
    {
        matchBitmap |= CSL_CPSW_ALE_POLICER_PORT_VALID;
    }

    if (NOT_ZERO(policerMatchEnMask & CPSW_ALE_POLICER_MATCH_PRIORITY))
    {
        matchBitmap |= CSL_CPSW_ALE_POLICER_PRI_VALID;
    }

    if (NOT_ZERO(policerMatchEnMask & CPSW_ALE_POLICER_MATCH_OUI))
    {
        matchBitmap |= CSL_CPSW_ALE_POLICER_OUI_VALID;
    }

    if (NOT_ZERO(policerMatchEnMask & CPSW_ALE_POLICER_MATCH_MACSRC))
    {
        matchBitmap |= CSL_CPSW_ALE_POLICER_SRC_MAC_VALID;
    }

    if (NOT_ZERO(policerMatchEnMask & CPSW_ALE_POLICER_MATCH_MACDST))
    {
        matchBitmap |= CSL_CPSW_ALE_POLICER_DST_MAC_VALID;
    }

    if (NOT_ZERO(policerMatchEnMask & CPSW_ALE_POLICER_MATCH_IVLAN))
    {
        matchBitmap |= CSL_CPSW_ALE_POLICER_VLAN_VALID;
    }

    if (NOT_ZERO(policerMatchEnMask & CPSW_ALE_POLICER_MATCH_OVLAN))
    {
        matchBitmap |= CSL_CPSW_ALE_POLICER_OVLAN_VALID;
    }

    if (NOT_ZERO(policerMatchEnMask & CPSW_ALE_POLICER_MATCH_ETHERTYPE))
    {
        matchBitmap |= CSL_CPSW_ALE_POLICER_ETHERTYPE_VALID;
    }

    if (NOT_ZERO(policerMatchEnMask & CPSW_ALE_POLICER_MATCH_IPSRC))
    {
        matchBitmap |= CSL_CPSW_ALE_POLICER_SRC_IP_VALID;
    }

    if (NOT_ZERO(policerMatchEnMask & CPSW_ALE_POLICER_MATCH_IPDST))
    {
        matchBitmap |= CSL_CPSW_ALE_POLICER_DST_IP_VALID;
    }

    return matchBitmap;
}

static int32_t CpswAle_getPolicer(CpswAle_Handle hAle,
                                  CSL_AleRegs *regs,
                                  const CpswAle_PolicerMatchParams *policerMatchParams,
                                  CpswAle_PolicerEntryOutArgs *outArgs,
                                  uint32_t aleFreqHz)
{
    CSL_CPSW_ALE_POLICER_ENTRY policerEntry;
    int32_t status;
    uint32_t policerIdx;

    status = CpswAle_getPolicerIndex(hAle, regs, policerMatchParams, &policerIdx);
    if (status == ENET_SOK)
    {
        CSL_CPSW_getAlePolicerEntry(regs, policerIdx, &policerEntry);

        /* Do not set valid bitmask to CSL valid bitmask as it is not understood
         * by application */
        outArgs->policerMatchEnMask     = policerMatchParams->policerMatchEnMask;
        outArgs->policerEntryIdx        = policerIdx;
        outArgs->port                   = policerEntry.port;
        outArgs->portIsTrunk            = NOT_ZERO(policerEntry.validBitmap & CSL_CPSW_ALE_POLICER_PORT_TRUNK_VALID);
        outArgs->priority               = policerEntry.pri;
        outArgs->ouiAleEntryIdx         = policerEntry.ouiIdx;
        outArgs->etherTypeAleEntryIdx   = policerEntry.ethertypeIdx;
        outArgs->ivlanAleEntryIdx       = policerEntry.vlanIdx;
        outArgs->ovlanAleEntryIdx       = policerEntry.ovlanIdx;
        outArgs->srcMacAleEntryIdx      = policerEntry.srcMacIdx;
        outArgs->dstMacAleEntryIdx      = policerEntry.dstMacIdx;
        outArgs->srcIpAleEntryIdx       = policerEntry.srcIpIdx;
        outArgs->dstIpAleEntryIdx       = policerEntry.dstIpIdx;
        outArgs->threadId               = policerEntry.thread;
        outArgs->threadIdEn             = NOT_ZERO(policerEntry.validBitmap & CSL_CPSW_ALE_POLICER_THREAD_VALID);
        outArgs->commitRateInBitsPerSec = CpswAle_mapIdlIncVal2Bw(aleFreqHz, policerEntry.cirIdleIncVal);
        outArgs->peakRateInBitsPerSec   = CpswAle_mapIdlIncVal2Bw(aleFreqHz, policerEntry.pirIdleIncVal);
        outArgs->egressOpEn             = (policerEntry.egressOp != 0);
        outArgs->egressOpcode           = policerEntry.egressOp;
        outArgs->egressTrunkIdx         = policerEntry.egressTrunkIndex;
        outArgs->ttlCheckEn             = policerEntry.enableTTLCheck;
        outArgs->dstPortMask            = policerEntry.destPortMask;
    }

    return status;
}

static int32_t CpswAle_getPolicerIndex(CpswAle_Handle hAle,
                                       CSL_AleRegs *regs,
                                       const CpswAle_PolicerMatchParams *policerMatchParams,
                                       uint32_t *policerIdx)
{
    uint32_t numPolicerEntries;
    int32_t status;
    uint32_t i;
    uint32_t matchBitmap;
    bool matchFound;
    CSL_CPSW_ALE_POLICER_ENTRY policerEntry;

    matchBitmap = CpswAle_mapPolicer2CslMask(policerMatchParams->policerMatchEnMask);
    CSL_CPSW_getAleStatusNumPolicers(regs, &numPolicerEntries);
    for (i = 0U; i < numPolicerEntries; i++)
    {
        CSL_CPSW_getAlePolicerEntry(regs, i, &policerEntry);
        if ((policerEntry.validBitmap & matchBitmap) == matchBitmap)
        {
            matchFound = CpswAle_checkPolicerMatch(hAle, regs, &policerEntry, policerMatchParams);
            if (matchFound)
            {
                break;
            }
        }
    }

    if (i < numPolicerEntries)
    {
        *policerIdx = i;
        status = ENET_SOK;
    }
    else
    {
        status = ENET_EFAIL;
    }

    return status;
}

static int32_t CpswAle_delPolicer(CpswAle_Handle hAle,
                                  CSL_AleRegs *regs,
                                  const CpswAle_PolicerMatchParams *policerMatchParams,
                                  uint32_t delAleEntryMask)
{
    uint32_t policerEntryIdx;
    int32_t status;

    status = CpswAle_getPolicerIndex(hAle, regs, policerMatchParams, &policerEntryIdx);

    if (status == ENET_SOK)
    {
        CpswAle_delPolicerEntry(hAle, regs, policerEntryIdx, delAleEntryMask);
    }

    return status;
}

static int32_t CpswAle_delAllPolicerThreadId(CpswAle_Handle hAle,
                                             CSL_AleRegs *regs,
                                             uint32_t threadId)
{
    uint32_t numPolicerEntries;
    uint32_t i;
    CSL_CPSW_ALE_POLICER_ENTRY policerEntry;
    int32_t status = ENET_SOK;

    if (threadId > CSL_ALE_THREADMAPVAL_THREADVAL_MAX)
    {
        status = ENET_EINVALIDPARAMS;
    }

    if (status == ENET_SOK)
    {
        CSL_CPSW_getAleStatusNumPolicers(regs, &numPolicerEntries);
        for (i = 0U; i < numPolicerEntries; i++)
        {
            CSL_CPSW_getAlePolicerEntry(regs, i, &policerEntry);
            if ((policerEntry.validBitmap & CSL_CPSW_ALE_POLICER_THREAD_VALID)
                &&
                (policerEntry.thread == threadId))
            {
                CpswAle_delPolicerEntry(hAle, regs, i, CPSW_ALE_POLICER_TABLEENTRY_DELETE_ALL);
            }
        }
    }

    return status;
}

static int32_t CpswAle_getPolicerStatsHandler(CpswAle_Handle hAle,
                                              CSL_AleRegs *regs,
                                              const CpswAle_PolicerMatchParams *policerMatchParams,
                                              bool *pPolHit,
                                              bool *pPolRedHit,
                                              bool *pPolYellowHit,
                                              bool clearStats)
{
    uint32_t policerIdx;
    int32_t status;

    status = CpswAle_getPolicerIndex(hAle, regs, policerMatchParams, &policerIdx);
    if (status == ENET_SOK)
    {
        CpswAle_getPolicerStats(regs,
                                policerIdx,
                                pPolHit,
                                pPolRedHit,
                                pPolYellowHit,
                                clearStats);
    }

    return status;
}

static int32_t CpswAle_setOAMLpbk(CSL_AleRegs *regs,
                                  uint32_t enableLpbkPortMask)
{
    CSL_CPSW_setAleOAMLpbkControl(regs, enableLpbkPortMask);
    return ENET_SOK;
}

static void CpswAle_updateMcastHostPortMask(CpswAle_Handle hAle,
                                            CSL_AleRegs *regs,
                                            bool addHostPort)
{
    CSL_CPSW_ALETABLE_TYPE tableType = hAle->tableType;
    uint32_t tableDepth = CpswAle_getMaxAleEntries(regs);
    uint8_t aleEntryMacAddr[ENET_MAC_ADDR_LEN];
    uint32_t i;

    for (i = 0U; i < tableDepth; i++)
    {
        uint32_t value;
        uint32_t reg0, reg1, reg2;

        /* read from the ALE table */
        CSL_CPSW_getAleTableEntry(regs, i, &reg0, &reg1, &reg2);

        value = CSL_CPSW_getALEEntryType(regs, i, tableType);
        /* We only need to check the entry type and address fields */

        if ((value == CSL_ALE_ENTRYTYPE_ADDRESS) ||
            (value == CSL_ALE_ENTRYTYPE_VLANADDRESS))
        {
            CSL_CPSW_mapTableWord2MacAddr(reg0, reg1, aleEntryMacAddr, tableType);
            if (EnetUtils_isMcastAddr(aleEntryMacAddr))
            {
                if (value == CSL_ALE_ENTRYTYPE_ADDRESS)
                {
                    CSL_CPSW_ALE_MCASTADDR_ENTRY entry;

                    CSL_CPSW_getAleMcastAddrEntry(regs, i, &entry, tableType);
                    if (addHostPort)
                    {
                        entry.portMask |= CPSW_ALE_HOST_PORT_MASK;
                    }
                    else
                    {
                        entry.portMask &= CPSW_ALE_ALL_MACPORTS_MASK;
                    }

                    CSL_CPSW_setAleMcastAddrEntry(regs, i, &entry, tableType);
                }
                else
                {
                    CSL_CPSW_ALE_VLANMCASTADDR_ENTRY entry;

                    CSL_CPSW_getAleVlanMcastAddrEntry(regs, i, &entry, tableType);
                    if (addHostPort)
                    {
                        entry.portMask |= CPSW_ALE_HOST_PORT_MASK;
                    }
                    else
                    {
                        entry.portMask &= CPSW_ALE_ALL_MACPORTS_MASK;
                    }

                    CSL_CPSW_setAleVlanMcastAddrEntry(regs, i, &entry, tableType);
                }
            }
        }
    }
}

static uint32_t CpswAle_delPortUcastEntries(CpswAle_Handle hAle,
                                            CSL_AleRegs *regs,
                                            uint32_t portNum)
{
    CSL_CPSW_ALETABLE_TYPE tableType = hAle->tableType;
    uint32_t tableDepth = CpswAle_getMaxAleEntries(regs);
    uint32_t i;
    uint8_t aleEntryMacAddr[ENET_MAC_ADDR_LEN];
    uint32_t numEntries = 0U;

    for (i = 0U; i < tableDepth; i++)
    {
        uint32_t value;
        uint32_t reg0, reg1, reg2;

        /* read from the ALE table */
        CSL_CPSW_getAleTableEntry(regs, i, &reg0, &reg1, &reg2);

        value = CSL_CPSW_getALEEntryType(regs, i, tableType);
        /* We only need to check the entry type and address fields */

        if ((value == CSL_ALE_ENTRYTYPE_ADDRESS) ||
            (value == CSL_ALE_ENTRYTYPE_VLANADDRESS))
        {
            CSL_CPSW_mapTableWord2MacAddr(reg0, reg1, aleEntryMacAddr, tableType);
            if (!EnetUtils_isMcastAddr(aleEntryMacAddr))
            {
                if (value == CSL_ALE_ENTRYTYPE_ADDRESS)
                {
                    CSL_CPSW_ALE_UNICASTADDR_ENTRY entry;

                    CSL_CPSW_getAleUnicastAddrEntry(regs, i, &entry, tableType);
                    if (portNum == entry.portNumber)
                    {
                        /* Clear ALE table unicast entry for the given port
                         * only if following conditions are met
                         * (ageable == TRUE)  : Dont delete static entries
                         * (secure  == FALSE) : Dont delete secured address
                         * (block  == FALSE) : Dont delete blocked address
                         * (trunk == FALSE)  : Entry is not part of trunk
                         */
                        if ((entry.ageable == TRUE) &&
                            (entry.secureEnable == FALSE) &&
                            (entry.blockEnable == FALSE) &&
                            (entry.trunkFlag == FALSE))
                        {
                            CpswAle_clearTableEntry(regs, i);
                            numEntries++;
                        }
                    }

                    CSL_CPSW_setAleUnicastAddrEntry(regs, i, &entry, tableType);
                }
                else
                {
                    CSL_CPSW_ALE_VLANUNICASTADDR_ENTRY entry;

                    CSL_CPSW_getAleVlanUnicastAddrEntry(regs, i, &entry, tableType);
                    if (portNum == entry.portNumber)
                    {
                        if ((entry.ageable == TRUE) &&
                            (entry.secureEnable == FALSE) &&
                            (entry.blockEnable == FALSE) &&
                            (entry.trunkFlag == FALSE))
                        {
                            CpswAle_clearTableEntry(regs, i);
                            numEntries++;
                        }
                    }

                    CSL_CPSW_setAleVlanUnicastAddrEntry(regs, i, &entry, tableType);
                }
            }
        }
    }

    return numEntries;
}

static void CpswAle_updateUnknownVlanMcastHostPortMask(CpswAle_Handle hAle,
                                                       CSL_AleRegs *regs,
                                                       bool updateRegMcast,
                                                       bool updateUnregMcast,
                                                       bool addHostPort)
{
    uint32_t unknownVlanMemList, unknownForceUntaggedEgress;
    uint32_t unknownVlanUnRegMcastFloodMask, unknownVlanRegMcastFloodMask;

    CSL_CPSW_getAleUnkownVlanReg(regs,
                                 &unknownVlanMemList,
                                 &unknownVlanUnRegMcastFloodMask,
                                 &unknownVlanRegMcastFloodMask,
                                 &unknownForceUntaggedEgress);
    if (updateRegMcast)
    {
        if (addHostPort)
        {
            unknownVlanRegMcastFloodMask |= CPSW_ALE_HOST_PORT_MASK;
            CSL_CPSW_setAleUnknwnVlanRegMcastReg(regs,
                                                 unknownVlanRegMcastFloodMask);
        }
        else
        {
            unknownVlanRegMcastFloodMask &= CPSW_ALE_ALL_MACPORTS_MASK;
            CSL_CPSW_setAleUnknwnVlanRegMcastReg(regs,
                                                 unknownVlanRegMcastFloodMask);
        }
    }

    if (updateUnregMcast)
    {
        if (addHostPort)
        {
            unknownVlanUnRegMcastFloodMask |= CPSW_ALE_HOST_PORT_MASK;
            CSL_CPSW_setAleUnknwnVlanUnregMcastReg(regs,
                                                   unknownVlanUnRegMcastFloodMask);
        }
        else
        {
            unknownVlanUnRegMcastFloodMask &= CPSW_ALE_ALL_MACPORTS_MASK;
            CSL_CPSW_setAleUnknwnVlanUnregMcastReg(regs,
                                                   unknownVlanUnRegMcastFloodMask);
        }
    }
}

static int32_t CpswAle_setRxFilterAll(CpswAle_Handle hAle,
                                      CSL_AleRegs *regs)
{
    if (NOT_ZERO(CSL_CPSW_isAleBypassEnabled(regs)) == FALSE)
    {
        /* Set ALE in bypass */
        CSL_CPSW_enableAleBypass(regs);
    }

    /* Set all ports in MAC only mode */

    /* Set all ports in MAC CAF enable */

    /* Disable learning on all ports */

    /* Clear all ALE table entries */

    /* Set ALE port state to forward for host port */
    CpswAle_setAlePortState(regs, CPSW_ALE_HOST_PORT_NUM, CSL_ALE_PORTSTATE_FORWARD);

    return ENET_SOK;
}

static int32_t CpswAle_setRxFilterAllMcast(CpswAle_Handle hAle,
                                           CSL_AleRegs *regs)
{
    if (CSL_CPSW_isAleBypassEnabled(regs))
    {
        /* Clear ALE bypass */
        CSL_CPSW_disableAleBypass(regs);
    }

    if (CSL_CPSW_isAleVlanAwareEnabled(regs))
    {
        /* Enable Host port for Unknown VLAN,reg and unreg mcast */
        CpswAle_updateUnknownVlanMcastHostPortMask(hAle, regs, TRUE, TRUE, TRUE);

        /* Enable Host port for Known VLAN,reg and unreg mcast */
        CpswAle_updateVlanMcastHostPortMask(hAle, regs, TRUE, TRUE, TRUE);
    }

    CpswAle_updateMcastHostPortMask(hAle, regs, TRUE);

    CpswAle_setAlePortState(regs, CPSW_ALE_HOST_PORT_NUM, CSL_ALE_PORTSTATE_FORWARD);

    return ENET_SOK;
}

static int32_t CpswAle_setRxFilterMcast(CpswAle_Handle hAle,
                                        CSL_AleRegs *regs)
{
    if (CSL_CPSW_isAleBypassEnabled(regs))
    {
        /* Clear ALE bypass */
        CSL_CPSW_disableAleBypass(regs);
    }

    if (CSL_CPSW_isAleVlanAwareEnabled(regs))
    {
        /* Enable Host port for Unknown VLAN reg mcast */
        CpswAle_updateUnknownVlanMcastHostPortMask(hAle, regs, TRUE, FALSE, TRUE);

        /* Disable Host port for Unknown VLAN unreg mcast */
        CpswAle_updateUnknownVlanMcastHostPortMask(hAle, regs, FALSE, TRUE, FALSE);

        /* Enable Host port for Known VLAN,reg mcast */
        CpswAle_updateVlanMcastHostPortMask(hAle, regs, TRUE, FALSE, TRUE);

        /* Disable Host port for Known VLAN,unreg mcast */
        CpswAle_updateVlanMcastHostPortMask(hAle, regs, FALSE, TRUE, FALSE);
    }

    CpswAle_updateMcastHostPortMask(hAle, regs, TRUE);
    CpswAle_setAlePortState(regs, CPSW_ALE_HOST_PORT_NUM, CSL_ALE_PORTSTATE_FORWARD);

    return ENET_SOK;
}

static int32_t CpswAle_setRxFilterBcast(CpswAle_Handle hAle,
                                        CSL_AleRegs *regs)
{
    CSL_CPSW_ALETABLE_TYPE tableType = hAle->tableType;
    CSL_CPSW_ALE_ENTRYTYPE entryType;
    const uint8_t addr[ENET_MAC_ADDR_LEN] = CPSW_ALE_BCAST_ADDR_INIT_LIST;
    uint32_t entryIdx;
    int32_t status = ENET_SOK;

    if (CSL_CPSW_isAleBypassEnabled(regs))
    {
        /* Clear ALE bypass */
        CSL_CPSW_disableAleBypass(regs);
    }

    if (CSL_CPSW_isAleVlanAwareEnabled(regs))
    {
        /* Enable Host port for Unknown VLAN reg mcast */
        CpswAle_updateUnknownVlanMcastHostPortMask(hAle, regs, TRUE, FALSE, TRUE);

        /* Disable Host port for Unknown VLAN unreg mcast */
        CpswAle_updateUnknownVlanMcastHostPortMask(hAle, regs, FALSE, TRUE, FALSE);

        /* Enable Host port for Known VLAN,reg mcast */
        CpswAle_updateVlanMcastHostPortMask(hAle, regs, TRUE, FALSE, TRUE);

        /* Disable Host port for Known VLAN,unreg mcast */
        CpswAle_updateVlanMcastHostPortMask(hAle, regs, FALSE, TRUE, FALSE);
    }

    /* Disable host port mask as we want to receive only broadcast */
    CpswAle_updateMcastHostPortMask(hAle, regs, FALSE);

    status = CpswAle_findAddr(hAle, regs, &addr, 0, &entryIdx);
    if (ENET_ENOTFOUND == status)
    {
        status = CpswAle_addMcastAddr(hAle,
                                      regs,
                                      &addr,
                                      0,
                                      CPSW_ALE_HOST_PORT_MASK,
                                      0,
                                      CPSW_ALE_FWDSTLVL_FWD,
                                      0,
                                      &entryIdx);
    }

    if (status == ENET_SOK)
    {
        entryType = CSL_CPSW_getALEEntryType(regs, entryIdx, tableType);
        if (entryType == CSL_ALE_ENTRYTYPE_ADDRESS)
        {
            CSL_CPSW_ALE_MCASTADDR_ENTRY entry;

            CSL_CPSW_getAleMcastAddrEntry(regs, entryIdx, &entry, tableType);
            entry.portMask |= CPSW_ALE_HOST_PORT_MASK;
            CSL_CPSW_setAleMcastAddrEntry(regs, entryIdx, &entry, tableType);
        }
        else
        {
            CSL_CPSW_ALE_VLANMCASTADDR_ENTRY entry;

            CSL_CPSW_getAleVlanMcastAddrEntry(regs, entryIdx, &entry, tableType);
            entry.portMask |= CPSW_ALE_HOST_PORT_MASK;
            CSL_CPSW_setAleVlanMcastAddrEntry(regs, entryIdx, &entry, tableType);
        }
    }

    if (status == ENET_SOK)
    {
        CpswAle_setAlePortState(regs, CPSW_ALE_HOST_PORT_NUM, CSL_ALE_PORTSTATE_FORWARD);
    }

    return status;
}

static int32_t CpswAle_setRxFilterDirect(CpswAle_Handle hAle,
                                         CSL_AleRegs *regs)
{
    if (CSL_CPSW_isAleBypassEnabled(regs))
    {
        /* Clear ALE bypass */
        CSL_CPSW_disableAleBypass(regs);
    }

    if (CSL_CPSW_isAleVlanAwareEnabled(regs))
    {
        /* Disable Host port for Unknown VLAN unreg/reg mcast */
        CpswAle_updateUnknownVlanMcastHostPortMask(hAle, regs, TRUE, TRUE, FALSE);

        /* Disable Host port for Known VLAN,reg,unreg mcast */
        CpswAle_updateVlanMcastHostPortMask(hAle, regs, TRUE, TRUE, FALSE);
    }

    /* Disable host port mask for all multicast addresses */
    CpswAle_updateMcastHostPortMask(hAle, regs, FALSE);

    CpswAle_setAlePortState(regs,
                            CPSW_ALE_HOST_PORT_NUM,
                            CSL_ALE_PORTSTATE_FORWARD);

    return ENET_SOK;
}

static int32_t CpswAle_setRxFilterNone(CpswAle_Handle hAle,
                                       CSL_AleRegs *regs)
{
    if (CSL_CPSW_isAleBypassEnabled(regs))
    {
        /* Clear ALE bypass */
        CSL_CPSW_disableAleBypass(regs);
    }

    if (CSL_CPSW_isAleVlanAwareEnabled(regs))
    {
        /* Disable Host port for Unknown VLAN unreg/reg mcast */
        CpswAle_updateUnknownVlanMcastHostPortMask(hAle, regs, TRUE, TRUE, FALSE);

        /* Disable Host port for Known VLAN,reg,unreg mcast */
        CpswAle_updateVlanMcastHostPortMask(hAle, regs, TRUE, TRUE, FALSE);
    }

    /* Disable host port mask for all multicast addresses */
    CpswAle_updateMcastHostPortMask(hAle, regs, FALSE);

    /* Delete all unicast addresses with host port as port Number */
    CpswAle_delPortUcastEntries(hAle, regs, CPSW_ALE_HOST_PORT_NUM);

    /* When Rx filter is none , set ALE host port state to disabled
     * to block all traffic to host port */
    CpswAle_setAlePortState(regs, CPSW_ALE_HOST_PORT_NUM, CSL_ALE_PORTSTATE_DISABLED);

    return ENET_SOK;
}

static int32_t CpswAle_setBcastMcastLimit(CpswAle_Handle hAle,
                                          CSL_AleRegs *regs,
                                          uint32_t aleFreqHz,
                                          const CpswAle_SetBcastMcastRateLimitInArgs *inArgs)
{
    uint32_t i;
    int32_t status = ENET_SOK;

    if (inArgs->rateLimitAtTxPort)
    {
        CSL_CPSW_enableAleTxRateLimit(regs);
    }
    else
    {
        CSL_CPSW_disableAleTxRateLimit(regs);
    }

    for (i = 0U; i < inArgs->numPorts; i++)
    {
        status = CpswAle_setPortBcastMcastLimit(regs,
                                                aleFreqHz,
                                                inArgs->portPrms[i].portNum,
                                                inArgs->portPrms[i].bcastRateLimitForPortEn,
                                                inArgs->portPrms[i].bcastLimitNumPktsPerSec,
                                                inArgs->portPrms[i].mcastRateLimitForPortEn,
                                                inArgs->portPrms[i].mcastLimitNumPktsPerSec);
        if (status != ENET_SOK)
        {
            break;
        }
    }

    CSL_CPSW_enableAleRateLimit(regs);

    return status;
}

static int32_t CpswAle_getBcastMcastLimit(CpswAle_Handle hAle,
                                          CSL_AleRegs *regs,
                                          uint32_t aleFreqHz,
                                          CpswAle_GetBcastMcastRateLimitOutArgs *outArgs)
{
    uint32_t numPorts = hAle->numPorts;
    uint32_t i;
    int32_t status = ENET_SOK;

    outArgs->rateLimitEn = CSL_CPSW_isAleRateLimitEnabled(regs);
    outArgs->rateLimitAtTxPort = CSL_CPSW_isAleTxRateLimitEnabled(regs);
    outArgs->numPorts = 0U;

    if (outArgs->rateLimitEn)
    {
        for (i = 0U; i < numPorts; i++)
        {
            bool bcastRateLimitForPortEn, mcastRateLimitForPortEn;
            uint32_t bcastLimitNumPktsPerSec, mcastLimitNumPktsPerSec;

            status = CpswAle_getPortBcastMcastLimit(regs,
                                                    aleFreqHz,
                                                    i,
                                                    &bcastRateLimitForPortEn,
                                                    &bcastLimitNumPktsPerSec,
                                                    &mcastRateLimitForPortEn,
                                                    &mcastLimitNumPktsPerSec);
            if (status != ENET_SOK)
            {
                break;
            }
            else
            {
                if ((bcastRateLimitForPortEn == true) || (mcastRateLimitForPortEn == true))
                {
                    if  (outArgs->numPorts < ENET_ARRAYSIZE(outArgs->portPrms))
                    {
                        outArgs->portPrms[outArgs->numPorts].portNum = i;
                        outArgs->portPrms[outArgs->numPorts].bcastRateLimitForPortEn = bcastRateLimitForPortEn;
                        outArgs->portPrms[outArgs->numPorts].mcastRateLimitForPortEn = mcastRateLimitForPortEn;
                        outArgs->portPrms[outArgs->numPorts].bcastLimitNumPktsPerSec = bcastLimitNumPktsPerSec;
                        outArgs->portPrms[outArgs->numPorts].mcastLimitNumPktsPerSec = mcastLimitNumPktsPerSec;
                        outArgs->numPorts++;
                    }
                    else
                    {
                        status = ENET_EINVALIDPARAMS;
                    }
                }
            }
        }
    }

    return status;
}

static int32_t CpswAle_setInterVlanConfig(CpswAle_Handle hAle,
                                          CSL_AleRegs *regs,
                                          uint32_t aleFreqHz,
                                          const CpswAle_SetInterVlanCfgInArgs *inArgs,
                                          CpswAle_PolicerEntryOutArgs *outArgs)
{
    uint32_t policerIdx;
    int32_t status;

    status = CpswAle_getPolicerIndex(hAle, regs, &inArgs->policerMatch, &policerIdx);
    if (status != ENET_SOK)
    {
        CpswAle_SetPolicerEntryOutArgs policerSetOutArgs;

        status = CpswAle_setPolicer(hAle,
                                    regs,
                                    &inArgs->policerMatch,
                                    CPSW_ALE_PEAKBITRATE_DISABLE,
                                    CPSW_ALE_COMMITBITRATE_DISABLE,
                                    aleFreqHz,
                                    FALSE,
                                    CPSW_ALE_THREADID_INVALID,
                                    &policerSetOutArgs);
        if (status == ENET_SOK)
        {
            policerIdx = policerSetOutArgs.policerEntryIdx;
        }
    }

    if (status == ENET_SOK)
    {
        CSL_CPSW_ALE_POLICER_ENTRY polEntry;

        CSL_CPSW_getAlePolicerEntry(regs, policerIdx, &polEntry);
        polEntry.validBitmap     |= CSL_CPSW_ALE_POLICER_EGRESSOP_VALID;
        polEntry.egressOp         = inArgs->routeIdx;
        polEntry.egressTrunkIndex = inArgs->egressTrunkIdx;
        polEntry.enableTTLCheck   = inArgs->ttlCheckEn;
        polEntry.destPortMask     = inArgs->dstPortMask;
        CSL_CPSW_setAlePolicerEntry(regs, policerIdx, &polEntry);
    }

    if (status == ENET_SOK)
    {
        status = CpswAle_getPolicer(hAle, regs, &inArgs->policerMatch, outArgs, aleFreqHz);
    }

    return status;
}

static int32_t CpswAle_configThreadPolicer(CpswAle_Handle hAle,
                                           CSL_AleRegs *regs,
                                           uint32_t aleFreqHz,
                                           const CpswAle_PolicerMatchParams *policerMatch,
                                           bool threadIdEn,
                                           uint32_t threadId,
                                           CpswAle_PolicerEntryOutArgs *outArgs)
{
    uint32_t policerIdx;
    int32_t status;

    status = CpswAle_getPolicerIndex(hAle, regs, policerMatch, &policerIdx);
    if (status != ENET_SOK)
    {
        CpswAle_SetPolicerEntryOutArgs policerSetOutArgs;

        status = CpswAle_setPolicer(hAle,
                                    regs,
                                    policerMatch,
                                    CPSW_ALE_PEAKBITRATE_DISABLE,
                                    CPSW_ALE_COMMITBITRATE_DISABLE,
                                    aleFreqHz,
                                    FALSE,
                                    CPSW_ALE_THREADID_INVALID,
                                    &policerSetOutArgs);
        if (status == ENET_SOK)
        {
            policerIdx = policerSetOutArgs.policerEntryIdx;
        }
    }

    if (status == ENET_SOK)
    {
        if (threadIdEn)
        {
            status = CpswAle_setThreadPolicerConfig(regs, policerIdx, threadId);
        }
        else
        {
            status = CpswAle_disableThreadPolicerConfig(regs, policerIdx);
        }
    }

    if (status == ENET_SOK)
    {
        status = CpswAle_getPolicer(hAle, regs, policerMatch, outArgs, aleFreqHz);
    }

    return status;
}

static int32_t CpswAle_getPortMacAddr(CpswAle_Handle hAle,
                                      CSL_AleRegs *regs,
                                      uint32_t portNum,
                                      CpswAle_MacAddrInfo *macAddrBuffer,
                                      uint32_t maxNumAddr,
                                      uint32_t *portTotalMacAddr)
{
    CSL_CPSW_ALETABLE_TYPE tableType = hAle->tableType;
    uint32_t tableDepth = CpswAle_getMaxAleEntries(regs);
    uint32_t macAddrBufferIndex;
    uint32_t i;
    uint8_t aleEntryMacAddr[ENET_MAC_ADDR_LEN];
    int32_t status = ENET_ENOTFOUND;

    *portTotalMacAddr = 0;
    macAddrBufferIndex = 0;
    for (i = 0U; i < tableDepth; i++)
    {
        uint32_t value;
        uint32_t reg0, reg1, reg2;

        /* read from the ALE table */
        CSL_CPSW_getAleTableEntry(regs, i, &reg0, &reg1, &reg2);

        value = CSL_CPSW_getALEEntryType(regs, i, tableType);
        /* We only need to check the entry type and address fields */

        if ((value == CSL_ALE_ENTRYTYPE_ADDRESS) ||
            (value == CSL_ALE_ENTRYTYPE_VLANADDRESS))
        {
            CSL_CPSW_mapTableWord2MacAddr(reg0, reg1, aleEntryMacAddr, tableType);
            if (!EnetUtils_isMcastAddr(aleEntryMacAddr))
            {
                if (value == CSL_ALE_ENTRYTYPE_ADDRESS)
                {
                    CSL_CPSW_ALE_UNICASTADDR_ENTRY entry;

                    CSL_CPSW_getAleUnicastAddrEntry(regs, i, &entry, tableType);
                    if (portNum == entry.portNumber)
                    {
                        *portTotalMacAddr = *portTotalMacAddr + 1;
                        if (macAddrBufferIndex < maxNumAddr)
                        {
                            CpswAle_MacAddrInfo *destEntry = &macAddrBuffer[macAddrBufferIndex];

                            EnetUtils_copyMacAddr(destEntry->addr, entry.macAddress);
                            destEntry->vlanId = 0;
                            macAddrBufferIndex++;
                        }
                    }
                }
                else
                {
                    CSL_CPSW_ALE_VLANUNICASTADDR_ENTRY entry;

                    CSL_CPSW_getAleVlanUnicastAddrEntry(regs, i, &entry, tableType);
                    if (portNum == entry.portNumber)
                    {
                        *portTotalMacAddr = *portTotalMacAddr + 1;
                        if (macAddrBufferIndex < maxNumAddr)
                        {
                            CpswAle_MacAddrInfo *destEntry = &macAddrBuffer[macAddrBufferIndex];

                            EnetUtils_copyMacAddr(destEntry->addr, entry.macAddress);
                            destEntry->vlanId = entry.vlanId;
                            macAddrBufferIndex++;
                        }
                    }
                }
            }
        }
    }

    return status;
}

static int32_t CpswAle_disablePortMirror(CSL_AleRegs *regs)
{
    uint32_t regVal;

    regVal = CSL_CPSW_getAleControlReg(regs);

    CSL_FINS(regVal, ALE_ALE_CONTROL_MIRROR_SEN, FALSE);
    CSL_FINS(regVal, ALE_ALE_CONTROL_MIRROR_DEN, FALSE);
    CSL_FINS(regVal, ALE_ALE_CONTROL_MIRROR_MEN, FALSE);

    CSL_CPSW_setAleControlReg(regs, regVal);

    return ENET_SOK;
}

static int32_t CpswAle_disableBcastMcastLimit(CSL_AleRegs *regs)
{
    CSL_CPSW_disableAleRateLimit(regs);
    return ENET_SOK;
}

static int32_t CpswAle_getPortState(CSL_AleRegs *regs,
                                    uint32_t portNum,
                                    CpswAle_PortState *portState)
{
    CSL_CPSW_getAlePortState(regs, portNum, (CSL_CPSW_ALE_PORTSTATE *)portState);
    return ENET_SOK;
}


static int32_t CpswAle_getPolicerControl(CpswAle_Handle hAle,
                                         CSL_AleRegs *regs,
                                         uint32_t aleFreqHz,
                                         CpswAle_PolicerGlobalCfg *policerCfg)
{
    CSL_CPSW_ALE_POLICER_CONTROL policerControl;
    uint32_t status = ENET_SOK;

    CSL_CPSW_getAlePolicerControlReg(regs, &policerControl);
    policerCfg->policingEn = policerControl.policingEnable;
    policerCfg->redDropEn = policerControl.redDropEnable;
    policerCfg->yellowDropEn = policerControl.yellowDropEnable;

#ifndef __aarch64__
    /* Direct assignment of CSL enums to CPSW enums. Ensure by compile time check that the enum values are equivalent */
    /* This check is disabled for GCC compiler due to warning -Werror=enum-compare so disabled for GCC compiler.
     * However the check must be there so that any change in CPSW or CSL enum values are flagged as compile time error */
    ENET_UTILS_COMPILETIME_ASSERT(CPSW_ALE_POLICER_YELLOWTHRESH_DROP_PERCENT_100 == CSL_ALE_POLICER_CONTROL_YELLOWTHRESH_DROP_PERCENT_100);
    ENET_UTILS_COMPILETIME_ASSERT(CPSW_ALE_POLICER_YELLOWTHRESH_DROP_PERCENT_50 == CSL_ALE_POLICER_CONTROL_YELLOWTHRESH_DROP_PERCENT_50);
    ENET_UTILS_COMPILETIME_ASSERT(CPSW_ALE_POLICER_YELLOWTHRESH_DROP_PERCENT_33 == CSL_ALE_POLICER_CONTROL_YELLOWTHRESH_DROP_PERCENT_33);
    ENET_UTILS_COMPILETIME_ASSERT(CPSW_ALE_POLICER_YELLOWTHRESH_DROP_PERCENT_25 == CSL_ALE_POLICER_CONTROL_YELLOWTHRESH_DROP_PERCENT_25);
    ENET_UTILS_COMPILETIME_ASSERT(CPSW_ALE_POLICER_YELLOWTHRESH_DROP_PERCENT_20 == CSL_ALE_POLICER_CONTROL_YELLOWTHRESH_DROP_PERCENT_20);
    ENET_UTILS_COMPILETIME_ASSERT(CPSW_ALE_POLICER_YELLOWTHRESH_DROP_PERCENT_17 == CSL_ALE_POLICER_CONTROL_YELLOWTHRESH_DROP_PERCENT_17);
    ENET_UTILS_COMPILETIME_ASSERT(CPSW_ALE_POLICER_YELLOWTHRESH_DROP_PERCENT_14 == CSL_ALE_POLICER_CONTROL_YELLOWTHRESH_DROP_PERCENT_14);
    ENET_UTILS_COMPILETIME_ASSERT(CPSW_ALE_POLICER_YELLOWTHRESH_DROP_PERCENT_13 == CSL_ALE_POLICER_CONTROL_YELLOWTHRESH_DROP_PERCENT_13);
#endif

    policerCfg->yellowThresh = (CpswAle_PolicerYellowThresh)policerControl.yellowDropThresh;
    /* Direct assignment of CSL enums to CPSW enums. Ensure by compile time check that the enum values are equivalent */
#ifndef __aarch64__
    ENET_UTILS_COMPILETIME_ASSERT(CPSW_ALE_POLICER_NOMATCH_MODE_GREEN == CSL_ALE_POLICER_CONTROL_POLICING_MATCH_MODE_NOMATCH_GREEN);
    ENET_UTILS_COMPILETIME_ASSERT(CPSW_ALE_POLICER_NOMATCH_MODE_YELLOW == CSL_ALE_POLICER_CONTROL_POLICING_MATCH_MODE_NOMATCH_YELLOW);
    ENET_UTILS_COMPILETIME_ASSERT(CPSW_ALE_POLICER_NOMATCH_MODE_RED == CSL_ALE_POLICER_CONTROL_POLICING_MATCH_MODE_NOMATCH_RED);
    ENET_UTILS_COMPILETIME_ASSERT(CPSW_ALE_POLICER_NOMATCH_MODE_UNREGULATED_TRAFFIC_POLICER == CSL_ALE_POLICER_CONTROL_POLICING_MATCH_MODE_NOMATCH_ENTRY0STATE);
#endif
    policerCfg->policerNoMatchMode = (CpswAle_PolicerNoMatchMode)policerControl.policeMatchMode;

    if (CPSW_ALE_POLICER_NOMATCH_MODE_UNREGULATED_TRAFFIC_POLICER == policerCfg->policerNoMatchMode)
    {

        status = CpswAle_getNoMatchEntry0Policer(regs,
                                                 aleFreqHz,
                                                 &policerCfg->noMatchPolicer.peakRateInBitsPerSec,
                                                 &policerCfg->noMatchPolicer.commitRateInBitsPerSec);
    }

    return status;
}

static int32_t CpswAle_getNoMatchEntry0Policer(CSL_AleRegs *regs,
                                               uint32_t aleFreq,
                                               uint32_t *pPeakRateInBitsPs,
                                               uint32_t *pCommitRateInBitsPs)
{
    CSL_CPSW_ALE_POLICER_ENTRY polEntry;

    CSL_CPSW_getAlePolicerEntry(regs, CPSW_ALE_NOMATCH_ENTRY0_POLICER_INDEX, &polEntry);
    if (polEntry.validBitmap & CSL_CPSW_ALE_POLICER_PIR_VALID)
    {
        *pPeakRateInBitsPs = CpswAle_mapIdlIncVal2Bw(aleFreq, polEntry.pirIdleIncVal);
    }
    else
    {
        *pPeakRateInBitsPs = 0U;
    }

    if (polEntry.validBitmap & CSL_CPSW_ALE_POLICER_CIR_VALID)
    {
        *pCommitRateInBitsPs = CpswAle_mapIdlIncVal2Bw(aleFreq, polEntry.cirIdleIncVal);
    }
    else
    {
        *pCommitRateInBitsPs = 0U;
    }

    return ENET_SOK;
}

static int32_t CpswAle_blockClassifierToHostPort(CpswAle_Handle hAle,
                                                 CSL_AleRegs *regs,
                                                 uint32_t aleFreqHz,
                                                 const CpswAle_PolicerMatchParams *policerMatch,
                                                 CpswAle_PolicerEntryOutArgs *outArgs)
{
    CpswAle_SetInterVlanCfgInArgs interVlanCfg;
    int32_t status;

    memset(&interVlanCfg, 0, sizeof(interVlanCfg));

    interVlanCfg.policerMatch   = *policerMatch;
    interVlanCfg.dstPortMask    = CPSW_ALE_BLOCKCLASSIFIER_PORTMASK;
    interVlanCfg.egressTrunkIdx = 0U;
    interVlanCfg.ttlCheckEn = false;
    interVlanCfg.routeIdx       = CPSW_ALE_BLOCKCLASSIFIER_ROUTEINDEX;
    status = CpswAle_setInterVlanConfig(hAle, regs, aleFreqHz, &interVlanCfg, outArgs);

    return status;
}

static int32_t CpswAle_addOuiAddr(CpswAle_Handle hAle,
                                  CSL_AleRegs *regs,
                                  CpswAle_OuiAddrType ouiAddr,
                                  uint32_t *entryIdx,
                                  bool *alloced)
{
    CSL_CPSW_ALETABLE_TYPE tableType = hAle->tableType;
    uint32_t freeEntry;
    uint32_t addrEntry;
    int32_t status;

    /* Check for space in the ALE table */
    status = CpswAle_findOuiAddr(hAle, regs, ouiAddr, &addrEntry);
    if (status == ENET_ENOTFOUND)
    {
        status = CpswAle_getFreeEntry(hAle, regs, &freeEntry);
        if (status != ENET_EALLOC)
        {
            CPSW_ALE_SAFE_ASSIGN(alloced, true);
        }
    }
    else
    {
        freeEntry = addrEntry;
        CPSW_ALE_SAFE_ASSIGN(alloced, false);
    }

    if (status != ENET_EALLOC)
    {
        CSL_CPSW_ALE_OUIADDR_ENTRY ouiEntry;

        memcpy(ouiEntry.ouiAddress, *ouiAddr, ENET_OUI_ADDR_LEN);
        CSL_CPSW_setAleOUIAddrEntry(regs, freeEntry, &ouiEntry, tableType);
        CPSW_ALE_SAFE_ASSIGN(entryIdx, freeEntry);
        status = ENET_SOK;
    }

    return status;
}


static int32_t CpswAle_addMcastAddr(CpswAle_Handle hAle,
                                    CSL_AleRegs *regs,
                                    CpswAle_MacAddrType addr,
                                    uint32_t vlanId,
                                    uint32_t portMask,
                                    uint32_t super,
                                    uint32_t fwdState,
                                    uint32_t numIgnBits,
                                    uint32_t *entryIdx)
{
    CSL_CPSW_ALETABLE_TYPE tableType = hAle->tableType;
    uint8_t mcastMaskedAddr[ENET_MAC_ADDR_LEN];
    bool vlanAware;
    uint32_t freeEntry;
    uint32_t addrEntry = 0U;
    uint32_t ignMBitsFlag;
    int32_t status = ENET_SOK;

    if ((numIgnBits > CPSW_ALE_MCAST_IGN_BITS_MAX) ||
        !EnetUtils_isMcastAddr(*addr))
    {
        status = ENET_EINVALIDPARAMS;
    }

    ENETTRACE_ERR_IF(status != ENET_SOK,
                     "Bad multicast address %02x:%02x:%02x:%02x:%02x:%02x/%u\r\n",
                     addr[0U], addr[1U], addr[2U], addr[3U], addr[4U], addr[5U], numIgnBits);

    if (status == ENET_SOK)
    {
        /* Check the current setting for vlanAware */
        vlanAware = (CSL_CPSW_isAleVlanAwareEnabled(regs) == TRUE);

        /* Ignore input vlanId if not in VLAN aware mode */
        if (!vlanAware)
        {
            vlanId = 0U;
        }

        CpswAle_mapMcastMaskBits2RegCfg((uint8_t *)addr, numIgnBits, mcastMaskedAddr, &ignMBitsFlag, tableType);

        /* Check for space in the ALE table */
        status = CpswAle_findAddr(hAle, regs, (CpswAle_MacAddrType)&mcastMaskedAddr, vlanId, &addrEntry);
        if (status == ENET_ENOTFOUND)
        {
            status = CpswAle_getFreeEntry(hAle, regs, &freeEntry);
        }
        else
        {
            /* Update the existing entry */
            freeEntry = addrEntry;
        }

        if (status != ENET_EALLOC)
        {
            if (vlanId != 0U)
            {
                CSL_CPSW_ALE_VLANMCASTADDR_ENTRY vlanMcastEntry;

                vlanMcastEntry.mcastFwdState = fwdState;
                vlanMcastEntry.portMask = portMask;
                vlanMcastEntry.superEnable = super;
                vlanMcastEntry.vlanId = vlanId;
                vlanMcastEntry.ignMBits = ignMBitsFlag;
                EnetUtils_copyMacAddr(vlanMcastEntry.macAddress, mcastMaskedAddr);

                CSL_CPSW_setAleVlanMcastAddrEntry(regs, freeEntry, &vlanMcastEntry, tableType);
            }
            else
            {
                CSL_CPSW_ALE_MCASTADDR_ENTRY mcastEntry;

                mcastEntry.mcastFwdState = fwdState;
                mcastEntry.portMask = portMask;
                mcastEntry.superEnable = super;
                mcastEntry.ignMBits = ignMBitsFlag;
                EnetUtils_copyMacAddr(mcastEntry.macAddress, mcastMaskedAddr);

                CSL_CPSW_setAleMcastAddrEntry(regs, freeEntry, &mcastEntry, tableType);
            }

            CPSW_ALE_SAFE_ASSIGN(entryIdx, freeEntry);
            status = ENET_SOK;
        }
    }

    return status;
}

static int32_t CpswAle_addUcastAddr(CpswAle_Handle hAle,
                                    CSL_AleRegs *regs,
                                    uint32_t portNum,
                                    CpswAle_MacAddrType addr,
                                    uint32_t vlanId,
                                    uint32_t blocked,
                                    uint32_t secure,
                                    uint32_t ageable,
                                    uint32_t trunk,
                                    uint32_t *entryIdx)
{
    CSL_CPSW_ALETABLE_TYPE tableType = hAle->tableType;
    bool vlanAware;
    uint32_t freeEntry;
    uint32_t addrEntry = 0;
    int32_t status;

    /* Check the current setting for vlanAware */
    vlanAware = (CSL_CPSW_isAleVlanAwareEnabled(regs) == TRUE);

    /* Ignore input vlanId if not in VLAN aware mode */
    if (!vlanAware)
    {
        vlanId = 0U;
    }

    /* Check for space in the ALE table */
    status = CpswAle_findAddr(hAle, regs, addr, vlanId, &addrEntry);
    if (status == ENET_ENOTFOUND)
    {
        status = CpswAle_getFreeEntry(hAle, regs, &freeEntry);
    }
    else
    {
        /* Update the existing entry itself */
        freeEntry = addrEntry;
    }

    if (status != ENET_EALLOC)
    {
        if ((vlanId != 0U) &&
            (vlanId <= ENET_VLAN_ID_MAX))
        {
            CSL_CPSW_ALE_VLANUNICASTADDR_ENTRY vlanUcastEntry;

            vlanUcastEntry.blockEnable  = blocked;
            vlanUcastEntry.portNumber   = portNum;
            vlanUcastEntry.secureEnable = secure;
            vlanUcastEntry.vlanId       = vlanId;
            vlanUcastEntry.ageable      = ageable;
            vlanUcastEntry.trunkFlag    = trunk;
            EnetUtils_copyMacAddr(vlanUcastEntry.macAddress, *addr);

            CSL_CPSW_setAleVlanUnicastAddrEntry(regs, freeEntry, &vlanUcastEntry, tableType);
        }
        else
        {
            CSL_CPSW_ALE_UNICASTADDR_ENTRY ucastEntry;

            /* Add new entry to ALE table */
            ucastEntry.blockEnable  = blocked;
            ucastEntry.portNumber   = portNum;
            ucastEntry.secureEnable = secure;
            ucastEntry.ageable      = ageable;
            ucastEntry.trunkFlag    = trunk;
            EnetUtils_copyMacAddr(ucastEntry.macAddress, *addr);

            CSL_CPSW_setAleUnicastAddrEntry(regs, freeEntry, &ucastEntry, tableType);
        }

        CPSW_ALE_SAFE_ASSIGN(entryIdx, freeEntry);
        status = ENET_SOK;
    }

    return status;
}

static void CpswAle_dumpTable(CpswAle_Handle hAle,
                              CSL_AleRegs *regs)
{
    CSL_CPSW_ALETABLE_TYPE tableType = hAle->tableType;
    uint32_t tableDepth = CpswAle_getMaxAleEntries(regs);
    uint32_t numPending = 0U;
    uint32_t i, freeEntries = 0U;

    EnetUtils_printf("\r\n");

    for (i = 0U; i < tableDepth; i++)
    {
        uint32_t info0, info1, info2;
        bool skipPrintEntryRaw = false;

        CSL_CPSW_getAleTableEntry(regs, i, &info0, &info1, &info2);

        if ((info0 == 0xFFFFFFFEU) && ((info1 & 0x0FFFFU) == 0x0FFFFU))
        {
        }
        else
        {
            skipPrintEntryRaw = false;
            switch (CSL_CPSW_getALEEntryType(regs, i, tableType))
            {
                case CSL_ALE_ENTRYTYPE_FREE:
                {
                    freeEntries++;
                    skipPrintEntryRaw = true;
                    break;
                }

                case CSL_ALE_ENTRYTYPE_ADDRESS: /* MAC address */
                {
                    CSL_CPSW_ALE_ADDRTYPE addrType = CSL_CPSW_getALEAddressType(regs, i, tableType);

                    if (CSL_ALE_ADDRTYPE_MCAST == addrType)
                    {
                        CSL_CPSW_ALE_MCASTADDR_ENTRY mcastEntry;

                        CSL_CPSW_getAleMcastAddrEntry(regs, i, &mcastEntry, tableType);
                        EnetUtils_printf("  %4d: Address: %02x%02x%02x%02x%02x%02x, Member:%03x Su=%x FWDSTLVL=%d IGNMBITS=%d",
                                         i,
                                         mcastEntry.macAddress[0],
                                         mcastEntry.macAddress[1],
                                         mcastEntry.macAddress[2],
                                         mcastEntry.macAddress[3],
                                         mcastEntry.macAddress[4],
                                         mcastEntry.macAddress[5],
                                         mcastEntry.portMask,
                                         mcastEntry.superEnable,
                                         mcastEntry.mcastFwdState,
                                         mcastEntry.ignMBits);
                    }
                    else
                    {
                        CSL_CPSW_ALE_UNICASTADDR_ENTRY ucastEntry;

                        CSL_CPSW_getAleUnicastAddrEntry(regs, i, &ucastEntry, tableType);

                        EnetUtils_printf("  %4d: Address: %02x%02x%02x%02x%02x%02x, Port: %03x Se=%x Bl=%d TOUCH=%d AGE=%d TRUNK=%d",
                                         i,
                                         ucastEntry.macAddress[0],
                                         ucastEntry.macAddress[1],
                                         ucastEntry.macAddress[2],
                                         ucastEntry.macAddress[3],
                                         ucastEntry.macAddress[4],
                                         ucastEntry.macAddress[5],
                                         ucastEntry.portNumber,
                                         ucastEntry.secureEnable,
                                         ucastEntry.blockEnable,
                                         ucastEntry.touched,
                                         ucastEntry.ageable,
                                         ucastEntry.trunkFlag);
                    }

                    break;
                }

                case CSL_ALE_ENTRYTYPE_POLICER: /* VLAN */
                {
                    CSL_CPSW_ALE_POLICER_ENTRYTYPE policerType;

                    policerType = CSL_CPSW_getALEPolicerEntryType(regs, i, tableType);
                    switch (policerType)
                    {
                        case CSL_ALE_POLICER_ENTRYTYPE_OVLAN:
                        {
                            CSL_CPSW_ALE_OUTER_VLAN_ENTRY ovlan;
                            uint32_t regMcastFloodMask;
                            uint32_t unRegMcastFloodMask;

                            CSL_CPSW_getAleOutVlanEntry(regs, i, &ovlan, tableType);
                            CpswAle_getVlanMcastPortMask(regs, &ovlan, &regMcastFloodMask, &unRegMcastFloodMask);
                            EnetUtils_printf("  %4d: Outer Vlanid: %04x, UTagged: %x, Mult: %x, UMult: %x, Member: %x",
                                             i,
                                             ovlan.vlanId,
                                             ovlan.forceUntaggedEgress,
                                             regMcastFloodMask,
                                             unRegMcastFloodMask,
                                             ovlan.vlanMemList);
                            break;
                        }

                        case CSL_ALE_POLICER_ENTRYTYPE_VLAN:
                        {
                            CSL_CPSW_ALE_VLAN_ENTRY vlanEntry;
                            uint32_t regMcastFloodMask;
                            uint32_t unRegMcastFloodMask;

                            CSL_CPSW_getAleVlanEntry(regs, i, &vlanEntry, tableType);
                            CpswAle_getVlanMcastPortMask(regs, &vlanEntry, &regMcastFloodMask, &unRegMcastFloodMask);
                            EnetUtils_printf("  %4d: Vlanid: %04x, UTagged: %x, Mult: %x, UMult: %x, Member: %x",
                                             i,
                                             vlanEntry.vlanId,
                                             vlanEntry.forceUntaggedEgress,
                                             regMcastFloodMask,
                                             unRegMcastFloodMask,
                                             vlanEntry.vlanMemList);
                            break;
                        }

                        case CSL_ALE_POLICER_ENTRYTYPE_ETHERTYPE:
                        {
                            CSL_CPSW_ALE_ETHERTYPE_ENTRY etherEntry;

                            CSL_CPSW_getAleEthertypeEntry(regs, i, &etherEntry, tableType);
                            EnetUtils_printf("  %4d: EtherType: %04x", i, etherEntry.ethertype);
                            break;
                        }

                        case CSL_ALE_POLICER_ENTRYTYPE_IPV4:
                        {
                            CSL_CPSW_ALE_IPv4_ENTRY ipv4Type;

                            CSL_CPSW_getAleIPv4Entry(regs, i, &ipv4Type, tableType);
                            EnetUtils_printf("  %4d: IPv4: Address: %02d:%02d:%02d:%02d, IGNBITS:%d",
                                             i,
                                             ipv4Type.address[0],
                                             ipv4Type.address[1],
                                             ipv4Type.address[2],
                                             ipv4Type.address[3],
                                             ipv4Type.numLSBIgnore);
                            break;
                        }

                        case CSL_ALE_POLICER_ENTRYTYPE_IPV6:
                        {
                            CSL_CPSW_ALE_IPv6_ENTRY ipv6Type;
                            bool isHighEntry;

                            isHighEntry = CpswAle_isIpv6HighEntry(hAle, regs, i, numPending);
                            if (isHighEntry)
                            {
                                skipPrintEntryRaw = TRUE;
                                numPending--;
                            }
                            else
                            {
                                numPending++;
                                CSL_CPSW_getAleIPv6Entry(regs, i, &ipv6Type, tableType);
                                EnetUtils_printf("  %4d: IPv6:     ,"
                                                 " Address: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:"
                                                 "%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x,"
                                                 " IGNBITS:%d",
                                                 i,
                                                 ipv6Type.address[0],
                                                 ipv6Type.address[1],
                                                 ipv6Type.address[2],
                                                 ipv6Type.address[3],
                                                 ipv6Type.address[4],
                                                 ipv6Type.address[5],
                                                 ipv6Type.address[6],
                                                 ipv6Type.address[7],
                                                 ipv6Type.address[8],
                                                 ipv6Type.address[9],
                                                 ipv6Type.address[10],
                                                 ipv6Type.address[11],
                                                 ipv6Type.address[12],
                                                 ipv6Type.address[13],
                                                 ipv6Type.address[14],
                                                 ipv6Type.address[15],
                                                 ipv6Type.numLSBIgnore);
                            }

                            break;
                        }
                    }

                    break;
                }

                case CSL_ALE_ENTRYTYPE_VLANADDRESS: /* VLAN MAC address */
                {
                    CSL_CPSW_ALE_ADDRTYPE addrType = CSL_CPSW_getALEAddressType(regs, i, tableType);

                    if (CSL_ALE_ADDRTYPE_MCAST == addrType)
                    {
                        CSL_CPSW_ALE_VLANMCASTADDR_ENTRY mcastEntry;

                        CSL_CPSW_getAleVlanMcastAddrEntry(regs, i, &mcastEntry, tableType);
                        EnetUtils_printf("  %4d: VlanId:%03x, Address: %02x%02x%02x%02x%02x%02x, Member:%03x Su=%x FWDSTLVL=%d IGNMBITS=%d",
                                         i,
                                         mcastEntry.vlanId,
                                         mcastEntry.macAddress[0],
                                         mcastEntry.macAddress[1],
                                         mcastEntry.macAddress[2],
                                         mcastEntry.macAddress[3],
                                         mcastEntry.macAddress[4],
                                         mcastEntry.macAddress[5],
                                         mcastEntry.portMask,
                                         mcastEntry.superEnable,
                                         mcastEntry.mcastFwdState,
                                         mcastEntry.ignMBits);
                    }
                    else
                    {
                        CSL_CPSW_ALE_VLANUNICASTADDR_ENTRY ucastEntry;

                        CSL_CPSW_getAleVlanUnicastAddrEntry(regs, i, &ucastEntry, tableType);
                        EnetUtils_printf("  %4d: VlanId:%03x, Address: %02x%02x%02x%02x%02x%02x, Port: %03x Se=%x Bl=%d TOUCH=%d AGE=%d TRUNK=%d",
                                         i,
                                         ucastEntry.vlanId,
                                         ucastEntry.macAddress[0],
                                         ucastEntry.macAddress[1],
                                         ucastEntry.macAddress[2],
                                         ucastEntry.macAddress[3],
                                         ucastEntry.macAddress[4],
                                         ucastEntry.macAddress[5],
                                         ucastEntry.portNumber,
                                         ucastEntry.secureEnable,
                                         ucastEntry.blockEnable,
                                         ucastEntry.touched,
                                         ucastEntry.ageable,
                                         ucastEntry.trunkFlag);
                    }

                    break;
                }

                default:
                {
                    break;
                }
            }

            /* Raw dump, followed by parsed output */
            if (!skipPrintEntryRaw)
            {
                /* print Info for entried that are not free */
                EnetUtils_printf(" RAW:[%01x %02x%02x%02x",
                                 info2 & 0xFU,
                                 (info1 >> 24U) & 0xFFU,
                                 (info1 >> 16U) & 0xFFU,
                                 (info1 >> 8U) & 0xFFU);
                EnetUtils_printf("%02x %02x%02x%02x%02x] \r\n",
                                 info1 & 0xFFU,
                                 (info0 >> 24U) & 0xFFU,
                                 (info0 >> 16U) & 0xFFU,
                                 (info0 >> 8U) & 0xFFU, info0 & 0xFFU);
            }
        }
    }

    EnetUtils_printf("\n\r");
    EnetUtils_printf("%d Free Entries \r\n", freeEntries);
}

static void CpswAle_printRegs(CpswAle_Handle hAle,
                              CSL_AleRegs *regs)
{
    uint32_t *regAddr = (uint32_t *)regs;
    uint32_t regIdx = 0;

    while (((uintptr_t)regAddr) < ((uintptr_t)regs + sizeof(*regs)))
    {
        if (*regAddr)
        {
            EnetUtils_printf("ALE: %u: 0x%08x\r\n", regIdx, *regAddr);
        }

        regAddr++;
        regIdx++;
    }
}

int32_t CpswAle_ioctl_handler_ENET_FDB_IOCTL_GET_VERSION(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    Enet_Version *version = (Enet_Version *)prms->outArgs;
    CSL_CPSW_ALE_VERSION ver = {0};
    int32_t status = ENET_SOK;

    CSL_CPSW_getAleVersionInfo(regs, &ver);
    version->maj = ver.majorVer;
    version->min = ver.minorVer;
    version->rtl = ver.rtlVer;
    version->id  = ver.id;
    version->other1 = ENET_VERSION_NONE;
    version->other2 = ENET_VERSION_NONE;
    return status;
}

#if ENET_CFG_IS_ON(DEV_ERROR)
int32_t CpswAle_ioctl_handler_ENET_FDB_IOCTL_PRINT_REGS(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

    CpswAle_printRegs(hAle, regs);
    return status;
}

int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_DUMP_TABLE(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

    CpswAle_dumpTable(hAle, regs);
    return status;
}
#endif
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_ADD_UCAST(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    const CpswAle_SetUcastEntryInArgs *inArgs = (const CpswAle_SetUcastEntryInArgs *)prms->inArgs;
    const uint8_t addr[ENET_MAC_ADDR_LEN] = CPSW_ALE_MAC_ADDR_INIT_LIST(inArgs->addr.addr);
    bool blocked = inArgs->info.blocked;
    bool secure = inArgs->info.secure;
    uint32_t *entryIdx = (uint32_t *)prms->outArgs;
    int32_t status = ENET_SOK;


    if (inArgs->info.super == TRUE)
    {
        blocked = TRUE;
        secure = TRUE;
    }

    status = CpswAle_addUcastAddr(hAle,
                                  regs,
                                  inArgs->info.portNum,
                                  &addr,
                                  inArgs->addr.vlanId,
                                  blocked,
                                  secure,
                                  inArgs->info.ageable,
                                  inArgs->info.trunk,
                                  entryIdx);
    return status;
}

int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_ADD_MCAST(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    const CpswAle_SetMcastEntryInArgs *inArgs = (const CpswAle_SetMcastEntryInArgs *)prms->inArgs;
    const uint8_t addr[ENET_MAC_ADDR_LEN] = CPSW_ALE_MAC_ADDR_INIT_LIST(inArgs->addr.addr);
    uint32_t *entryIdx = (uint32_t *)prms->outArgs;
    int32_t status = ENET_SOK;


    status = CpswAle_addMcastAddr(hAle,
                                  regs,
                                  &addr,
                                  inArgs->addr.vlanId,
                                  inArgs->info.portMask,
                                  inArgs->info.super,
                                  inArgs->info.fwdState,
                                  inArgs->info.numIgnBits,
                                  entryIdx);
    return status;
}

int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_ADD_VLAN(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    const CpswAle_VlanEntryInfo *inArgs = (const CpswAle_VlanEntryInfo *)prms->inArgs;
    uint32_t *entryIdx = (uint32_t *)prms->outArgs;
    int32_t status = ENET_SOK;


    status = CpswAle_addVlan(hAle,
                             regs,
                             inArgs->vlanIdInfo.vlanId,
                             inArgs->vlanIdInfo.tagType == ENET_VLAN_TAG_TYPE_OUTER,
                             inArgs->vlanMemberList,
                             inArgs->unregMcastFloodMask,
                             inArgs->regMcastFloodMask,
                             inArgs->forceUntaggedEgressMask,
                             inArgs->noLearnMask,
                             inArgs->vidIngressCheck,
                             inArgs->limitIPNxtHdr,
                             inArgs->disallowIPFrag,
                             entryIdx);

    return status;
}

int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_ADD_OUI(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    const CpswAle_OuiEntryInfo *ouiInfo = (const CpswAle_OuiEntryInfo *)prms->inArgs;
    uint32_t *entryIdx = (uint32_t *)prms->outArgs;
    const uint8_t ouiAddr[ENET_OUI_ADDR_LEN] = CPSW_ALE_OUI_ADDR_INIT_LIST(ouiInfo->ouiAddr);
    int32_t status = ENET_SOK;


    status = CpswAle_addOuiAddr(hAle, regs, &ouiAddr, entryIdx, NULL);
    return status;
}

int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_ADD_IPV4ADDR(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    const CpswAle_IPv4EntryInfo *ipv4Info = (const CpswAle_IPv4EntryInfo *)prms->inArgs;
    uint32_t *entryIdx = (uint32_t *)prms->outArgs;
    const uint8_t ipv4Addr[ENET_IPv4_ADDR_LEN] = CPSW_ALE_IPV4_ADDR_INIT_LIST(ipv4Info->ipv4Addr);
    int32_t status = ENET_SOK;

    status = CpswAle_addIPv4Addr(hAle, regs, &ipv4Addr, ipv4Info->numLSBIgnoreBits, entryIdx, NULL);
    return status;
}

int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_ADD_IPV6ADDR(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    const CpswAle_IPv6EntryInfo *ipv6Info = (const CpswAle_IPv6EntryInfo *)prms->inArgs;
    uint32_t *entryIdx = (uint32_t *)prms->outArgs;
    const uint8_t ipv6Addr[ENET_IPv6_ADDR_LEN] = CPSW_ALE_IPV6_ADDR_INIT_LIST(ipv6Info->ipv6Addr);
    int32_t status = ENET_SOK;

    status = CpswAle_addIPv6Addr(hAle, regs, &ipv6Addr, ipv6Info->numLSBIgnoreBits, entryIdx, NULL);
    return status;
}

int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_ADD_ETHERTYPE(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    uint16_t etherType = *(uint16_t *)prms->inArgs;
    uint32_t *entryIdx = (uint32_t *)prms->outArgs;
    int32_t status = ENET_SOK;

    status = CpswAle_addEtherType(hAle, regs, etherType, entryIdx, NULL);
    return status;
}

int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_LOOKUP_UCAST(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    const CpswAle_MacAddrInfo *macInfo = (const CpswAle_MacAddrInfo *)prms->inArgs;
    CpswAle_GetUcastEntryOutArgs *outArgs = (CpswAle_GetUcastEntryOutArgs *)prms->outArgs;
    const uint8_t addr[ENET_MAC_ADDR_LEN] = CPSW_ALE_MAC_ADDR_INIT_LIST(macInfo->addr);
    int32_t status = ENET_SOK;

    status = CpswAle_lookupUcastAddr(hAle,
                                     regs,
                                     &addr,
                                     macInfo->vlanId,
                                     &outArgs->info.portNum,
                                     &outArgs->info.blocked,
                                     &outArgs->info.secure,
                                     &outArgs->info.super,
                                     &outArgs->info.ageable,
                                     &outArgs->info.trunk,
                                     &outArgs->touched,
                                     &outArgs->aleEntryIdx);
    return status;
}

int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_LOOKUP_MCAST(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    const CpswAle_GetMcastEntryInArgs *mcastInfo = (const CpswAle_GetMcastEntryInArgs *)prms->inArgs;
    CpswAle_GetMcastEntryOutArgs *outArgs = (CpswAle_GetMcastEntryOutArgs *)prms->outArgs;
    const uint8_t addr[ENET_MAC_ADDR_LEN] = CPSW_ALE_MAC_ADDR_INIT_LIST(mcastInfo->addr.addr);
    int32_t status = ENET_SOK;

    status = CpswAle_lookupMcastAddr(hAle,
                                     regs,
                                     &addr,
                                     mcastInfo->addr.vlanId,
                                     mcastInfo->numIgnBits,
                                     &outArgs->info.portMask,
                                     &outArgs->info.fwdState,
                                     &outArgs->info.super,
                                     &outArgs->info.numIgnBits,
                                     &outArgs->aleEntryIdx);
    return status;
}

int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_LOOKUP_VLAN(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    const CpswAle_VlanIdInfo *vlanInfo = (const CpswAle_VlanIdInfo *)prms->inArgs;
    CpswAle_GetVlanEntryOutArgs *outArgs = (CpswAle_GetVlanEntryOutArgs *)prms->outArgs;
    int32_t status = ENET_SOK;

    status = CpswAle_findVlan(hAle,
                              regs,
                              vlanInfo->vlanId,
                              vlanInfo->tagType == ENET_VLAN_TAG_TYPE_OUTER,
                              &outArgs->vlanMemberList,
                              &outArgs->unregMcastFloodMask,
                              &outArgs->regMcastFloodMask,
                              &outArgs->forceUntaggedEgressMask,
                              &outArgs->noLearnMask,
                              &outArgs->vidIngressCheck,
                              &outArgs->limitIPNxtHdr,
                              &outArgs->disallowIPFrag,
                              &outArgs->aleEntryIdx);
    return status;
}

int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_REMOVE_ADDR(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    const CpswAle_MacAddrInfo *macInfo = (const CpswAle_MacAddrInfo *)prms->inArgs;
    const uint8_t addr[ENET_MAC_ADDR_LEN] = CPSW_ALE_MAC_ADDR_INIT_LIST(macInfo->addr);
    int32_t status = ENET_SOK;

    status = CpswAle_delAddr(hAle, regs, &addr, macInfo->vlanId);
    return status;
}

int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_REMOVE_VLAN(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    const CpswAle_VlanIdInfo *vlanInfo = (CpswAle_VlanIdInfo *)prms->inArgs;
    int32_t status = ENET_SOK;

    status = CpswAle_delVlan(hAle, regs, vlanInfo->vlanId, vlanInfo->tagType == ENET_VLAN_TAG_TYPE_OUTER);
    return status;
}

int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_REMOVE_OUI(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    const CpswAle_OuiEntryInfo *ouiInfo = (const CpswAle_OuiEntryInfo *)prms->inArgs;
    const uint8_t ouiAddr[ENET_OUI_ADDR_LEN] = CPSW_ALE_OUI_ADDR_INIT_LIST(ouiInfo->ouiAddr);
    int32_t status = ENET_SOK;

    status = CpswAle_delOuiAddr(hAle, regs, &ouiAddr);
    return status;
}

int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_REMOVE_IPV4ADDR(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    const CpswAle_IPv4EntryInfo *ipv4Info = (const CpswAle_IPv4EntryInfo *)prms->inArgs;
    const uint8_t ipv4Addr[ENET_IPv4_ADDR_LEN] = CPSW_ALE_IPV4_ADDR_INIT_LIST(ipv4Info->ipv4Addr);
    int32_t status = ENET_SOK;

    status = CpswAle_delIPv4Addr(hAle, regs, &ipv4Addr, ipv4Info->numLSBIgnoreBits);
    return status;
}

int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_REMOVE_IPV6ADDR(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    const CpswAle_IPv6EntryInfo *ipv6Info = (const CpswAle_IPv6EntryInfo *)prms->inArgs;
    const uint8_t ipv6Addr[ENET_IPv6_ADDR_LEN] = CPSW_ALE_IPV6_ADDR_INIT_LIST(ipv6Info->ipv6Addr);
    int32_t status = ENET_SOK;

    status = CpswAle_delIPv6Addr(hAle, regs, &ipv6Addr, ipv6Info->numLSBIgnoreBits);
    return status;
}

int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_REMOVE_ETHERTYPE(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    uint16_t etherType = *(uint16_t *)prms->inArgs;
    int32_t status = ENET_SOK;

    status = CpswAle_delEtherType(hAle, regs, etherType);
    return status;
}

int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_REMOVE_LEARNED_ENTRIES(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    uint32_t portNum = *(uint32_t *)prms->inArgs;
    uint32_t *numEntries = (uint32_t *)prms->outArgs;
    int32_t status = ENET_SOK;

    *numEntries = CpswAle_delPortUcastEntries(hAle, regs, portNum);
    return status;
}

int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_REMOVE_ALL_ENTRIES(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

    status = CpswAle_delAllEntries(hAle, regs);
    return status;
}

int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_AGE_ALL_ENTRIES(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

    CpswAle_ageOut(regs,
                   hAle->softTimerActive,
                   hAle->tickTimeoutCnt,
                   &hAle->softTickCnt);
    return status;
}

int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_SET_RX_FILTER(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    CpswAle_RxFilter rxFilter = *(CpswAle_RxFilter *)prms->inArgs;
    int32_t status = ENET_SOK;

    status = CpswAle_setReceiveFilter(hAle, rxFilter);
    return status;
}

int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_GET_RX_FILTER(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    CpswAle_RxFilter *rxFilter = (CpswAle_RxFilter *)prms->outArgs;
    int32_t status = ENET_SOK;

    status = CpswAle_getReceiveFilter(hAle, rxFilter);
    return status;
}

int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_SET_PORT_STATE(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    const CpswAle_SetPortStateInArgs *inArgs = (const CpswAle_SetPortStateInArgs *)prms->inArgs;
    int32_t status = ENET_SOK;

    status = CpswAle_setAlePortState(regs,
                                     inArgs->portNum,
                                     (CSL_CPSW_ALE_PORTSTATE)inArgs->portState);
    return status;
}

int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_GET_PORT_STATE(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    uint32_t portNum = *(uint32_t *)prms->inArgs;
    CpswAle_PortState *portState = (CpswAle_PortState *)prms->outArgs;
    int32_t status = ENET_SOK;

    status = CpswAle_getPortState(regs, portNum, portState);
    return status;
}

int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_GET_PORT_MACADDR(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    const CpswAle_GetPortMacAddrInArgs *inArgs = (const CpswAle_GetPortMacAddrInArgs *)prms->inArgs;
    CpswAle_GetPortMacAddrOutArgs *outArgs = (CpswAle_GetPortMacAddrOutArgs *)prms->outArgs;
    int32_t status = ENET_SOK;

    status = CpswAle_getPortMacAddr(hAle,
                                    regs,
                                    inArgs->portNum,
                                    inArgs->addrs,
                                    inArgs->addrCnt,
                                    &outArgs->totalAddrCnt);
    if (status == ENET_SOK)
    {
        outArgs->addrs = inArgs->addrs;
        if (outArgs->totalAddrCnt < inArgs->addrCnt)
        {
            outArgs->addrCnt = outArgs->totalAddrCnt;
        }
        else
        {
            outArgs->addrCnt = inArgs->addrCnt;
        }
    }
    return status;
}

int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_SET_DEFAULT_THREADCFG(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    const CpswAle_DfltThreadCfg *inArgs = (const CpswAle_DfltThreadCfg *)prms->inArgs;
    int32_t status = ENET_SOK;

    status = CpswAle_setPolicerDefaultThreadCfg(hAle,
                                                regs,
                                                inArgs->dfltThreadEn,
                                                inArgs->threadId,
                                                inArgs->priorityOrEn,
                                                inArgs->macPortDfltThreadDis);
    return status;
}

int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_GET_DEFAULT_THREADCFG(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    CpswAle_DfltThreadCfg *outArgs = (CpswAle_DfltThreadCfg *)prms->outArgs;
    int32_t status = ENET_SOK;

    status = CpswAle_getPolicerDefaultThreadConfig(hAle, regs, outArgs);
    return status;
}

int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_SET_PORT_MIRROR_CFG(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    const CpswAle_PortMirroringCfg *mirroringCfg = (const CpswAle_PortMirroringCfg *)prms->inArgs;
    int32_t status = ENET_SOK;

    status = CpswAle_setPortMirrorConfig(hAle, regs, hAle->numPorts, mirroringCfg);
    return status;
}

int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_DISABLE_PORT_MIRROR(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

    status = CpswAle_disablePortMirror(regs);
    return status;
}

int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_SET_TRUNK_CFG(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    const CpswAle_TrunkCfg *trunkCfg = (const CpswAle_TrunkCfg *)prms->inArgs;
    int32_t status = ENET_SOK;

    status = CpswAle_setPortTrunkConfig(hAle, regs, trunkCfg);
    return status;
}

int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_SET_OAMLPBK_CFG(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    uint32_t enableLpbkPortMask = *(uint32_t *)prms->inArgs;
    int32_t status = ENET_SOK;

    status = CpswAle_setOAMLpbk(regs, enableLpbkPortMask);
    return status;
}

int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_SET_BCAST_MCAST_LIMIT(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    const CpswAle_SetBcastMcastRateLimitInArgs *inArgs =
        (const CpswAle_SetBcastMcastRateLimitInArgs *)prms->inArgs;
    int32_t status = ENET_SOK;

    status = CpswAle_setBcastMcastLimit(hAle, regs, hAle->aleFreqHz, inArgs);
    return status;
}

int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_GET_BCAST_MCAST_LIMIT(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    CpswAle_GetBcastMcastRateLimitOutArgs *outArgs = (CpswAle_GetBcastMcastRateLimitOutArgs *)prms->outArgs;
    int32_t status = ENET_SOK;

    status = CpswAle_getBcastMcastLimit(hAle, regs, hAle->aleFreqHz, outArgs);
    return status;
}

int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_DISABLE_BCAST_MCAST_LIMIT(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

    status = CpswAle_disableBcastMcastLimit(regs);
    return status;
}

int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_SET_POLICER(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    const CpswAle_SetPolicerEntryInArgs *inArgs = (const CpswAle_SetPolicerEntryInArgs *)prms->inArgs;
    CpswAle_SetPolicerEntryOutArgs *outArgs = (CpswAle_SetPolicerEntryOutArgs *)prms->outArgs;
    int32_t status = ENET_SOK;

    status = CpswAle_setPolicer(hAle,
                                regs,
                                &inArgs->policerMatch,
                                inArgs->peakRateInBitsPerSec,
                                inArgs->commitRateInBitsPerSec,
                                hAle->aleFreqHz,
                                inArgs->threadIdEn,
                                inArgs->threadId,
                                outArgs);
    return status;
}


int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_GET_POLICER(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    const CpswAle_PolicerMatchParams *inArgs = (const CpswAle_PolicerMatchParams *)prms->inArgs;
    CpswAle_PolicerEntryOutArgs *outArgs = (CpswAle_PolicerEntryOutArgs *)prms->outArgs;
    int32_t status = ENET_SOK;

    status = CpswAle_getPolicer(hAle, regs, inArgs, outArgs, hAle->aleFreqHz);
    return status;
}

int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_DEL_POLICER(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    const CpswAle_DelPolicerEntryInArgs *inArgs = (const CpswAle_DelPolicerEntryInArgs *)prms->inArgs;
    int32_t status = ENET_SOK;

    status = CpswAle_delPolicer(hAle, regs, &inArgs->policerMatch, inArgs->aleEntryMask);
    return status;
}

int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_DUMP_POLICER_ENTRIES(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

    CpswAle_dumpPolicerEntries(hAle, regs);
    return status;
}

int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_GET_POLICER_STATS(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    const CpswAle_GetPolicerStatsInArgs *inArgs = (const CpswAle_GetPolicerStatsInArgs *)prms->inArgs;
    CpswAle_GetPolicerStatsOutArgs *outArgs = (CpswAle_GetPolicerStatsOutArgs *)prms->outArgs;
    int32_t status = ENET_SOK;

    status = CpswAle_getPolicerStatsHandler(hAle,
                                            regs,
                                            &inArgs->policerInfo,
                                            &outArgs->policerHit,
                                            &outArgs->policerRedHit,
                                            &outArgs->policerYellowHit,
                                            inArgs->clearStats);
    return status;
}

int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_SET_POLICER_THREADCFG(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    const CpswAle_SetPolicerThreadCfgInArgs *inArgs =
        (const CpswAle_SetPolicerThreadCfgInArgs *)prms->inArgs;
    CpswAle_PolicerEntryOutArgs *outArgs = (CpswAle_PolicerEntryOutArgs *)prms->outArgs;
    int32_t status = ENET_SOK;

    status = CpswAle_configThreadPolicer(hAle,
                                         regs,
                                         hAle->aleFreqHz,
                                         &inArgs->policerMatch,
                                         inArgs->threadIdEn,
                                         inArgs->threadId,
                                         outArgs);
    return status;
}

int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_SET_POLICER_GLOBAL_CFG(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    const CpswAle_PolicerGlobalCfg *inArgs = (const CpswAle_PolicerGlobalCfg *)prms->inArgs;
    int32_t status = ENET_SOK;

    status = CpswAle_setPolicerControl(regs,
                                       hAle->aleFreqHz,
                                       inArgs->policingEn,
                                       inArgs->yellowDropEn,
                                       inArgs->redDropEn,
                                       inArgs->yellowThresh,
                                       inArgs->policerNoMatchMode,
                                       &inArgs->noMatchPolicer);

    return status;
}

int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_GET_POLICER_GLOBAL_CFG(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    CpswAle_PolicerGlobalCfg *outArgs = (CpswAle_PolicerGlobalCfg *)prms->outArgs;
    int32_t status = ENET_SOK;

    status = CpswAle_getPolicerControl(hAle, regs, hAle->aleFreqHz, outArgs);
    return status;
}

int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_DEL_ALL_POLICER_THREADID(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    uint32_t threadId = *(uint32_t *)prms->inArgs;
    int32_t status = ENET_SOK;

    status = CpswAle_delAllPolicerThreadId(hAle, regs, threadId);
    return status;
}

#if ENET_CFG_IS_ON(CPSW_INTERVLAN)
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_BLOCK_CLASSIFIER_HOSTPORT(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    const CpswAle_PolicerMatchParams *inArgs = (const CpswAle_PolicerMatchParams *)prms->inArgs;
    CpswAle_PolicerEntryOutArgs *outArgs = (CpswAle_PolicerEntryOutArgs *)prms->outArgs;
    int32_t status = ENET_SOK;

    status = CpswAle_blockClassifierToHostPort(hAle, regs, hAle->aleFreqHz, inArgs, outArgs);
    return status;
}
#endif

#if ENET_CFG_IS_ON(CPSW_INTERVLAN)
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_SET_INTERVLAN_CFG(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    const CpswAle_SetInterVlanCfgInArgs *inArgs = (const CpswAle_SetInterVlanCfgInArgs *)prms->inArgs;
    CpswAle_PolicerEntryOutArgs *outArgs = (CpswAle_PolicerEntryOutArgs *)prms->outArgs;
    int32_t status = ENET_SOK;

    status = CpswAle_setInterVlanConfig(hAle, regs, hAle->aleFreqHz, inArgs, outArgs);
    return status;
}
#endif

#if ENET_CFG_IS_ON(CPSW_INTERVLAN)
int32_t CpswAle_ioctl_handler_CPSW_ALE_IOCTL_GET_INTERVLAN_CFG(CpswAle_Handle hAle, CSL_AleRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

    /* TODO */
    status = ENET_ENOTSUPPORTED;
    return status;
}
#endif



