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
 * \file  icssg_utils.h
 *
 * \brief This file contains the type definitions and helper macros for the
 *        ICSSG utils.
 */

/*!
 * \ingroup  DRV_ENET_ICSSG
 * \defgroup DRV_ENET_ICSSG_UTILS ICSSG Peripheral Utils
 *
 * @{
 */

#ifndef ICSSG_UTILS_H_
#define ICSSG_UTILS_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <include/core/enet_types.h>
#include <priv/per/icssg_priv.h>

#include <drivers/pruicss.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

#define ICSSG_UTILS_FW_MGMT_CMD_HEADER                      (0x81U)
#define ICSSG_UTILS_FW_MGMT_FDB_CMD_TYPE                    (0x03U)
#define ICSSG_UTILS_FW_MGMT_CMD_TYPE                        (0x04U)
#define ICSSG_IOCTL_SUBCMD_FDB_ENTRY_ADD                    (1U)
#define ICSSG_IOCTL_SUBCMD_FDB_ENTRY_REMOVE                 (2U)
#define ICSSG_IOCTL_SUBCMD_FDB_ENTRY_REMOVE_ALL             (3U)
#define ICSSG_IOCTL_SUBCMD_FDB_ENTRY_ADD_MULTIPLE           (4U)
#define ICSSG_IOCTL_SUBCMD_FDB_ENTRY_REMOVE_ALL_AGEABLE     (5U)
#define EMAC_FW_MGMT_SPEED_DUPLEXITY_CMD_TYPE               (0x02U)

#define ICSSG_UTILS_MGMT_FREE_HWQA_PORT0     ((int32_t)56)   /* mgmt free queue port 0*/
#define ICSSG_UTILS_MGMT_RX_HWQA_PORT0       ((int32_t)58)   /* mgmt response queue port 0*/
#define ICSSG_UTILS_MGMT_TX_HWQA_PORT0       ((int32_t)57)   /* tx submit queue port 0*/
#define ICSSG_UTILS_TXTS_FREE_HWQA_PORT0     ((int32_t)40)   /* tx ts free queue port 0*/
#define ICSSG_UTILS_TXTS_RX_HWQA_PORT0       ((int32_t)59)   /* txts response queue port 0 */

#define ICSSG_UTILS_MGMT_FREE_HWQA_PORT1     ((int32_t)60)   /* mgmt free queue port 1 */
#define ICSSG_UTILS_MGMT_RX_HWQA_PORT1       ((int32_t)62)   /* mgmt response queue port 1*/
#define ICSSG_UTILS_MGMT_TX_HWQA_PORT1       ((int32_t)61)   /* tx submit queue port 1 */
#define ICSSG_UTILS_TXTS_FREE_HWQA_PORT1     ((int32_t)41)   /* tx ts free queue port 1 */
#define ICSSG_UTILS_TXTS_RX_HWQA_PORT1       ((int32_t)63)   /* txts response queue port 1 */

/* Specifies NO IOCTL type in progress */
#define ICSSG_UTILS_IOCTL_TYPE_NON_ACTIVE      (0)

/* Specifies asynchronous ioctl is over DMEM in progress*/
#define ICSSG_UTILS_IOCTL_TYPE_R30_OVER_DMEM      (1U)

/* Specifies asynchronous ioctl is over HWQ */
#define ICSSG_UTILS_IOCTL_TYPE_HWQ                (2U)

/*! \brief Whether it is slice 0 or not */
#define ICSSG_IS_SLICE_0(slice)                   ((slice) == 0U)

/*! \brief Whether it is slice 1 or not */
#define ICSSG_IS_SLICE_1(slice)                   ((slice) == 1U)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* Command types to program ICSSG R30 register, internal to enet driver */
typedef enum IcssgUtils_ioctlR30Cmd_e
{
    ICSSG_UTILS_R30_CMD_DISABLE = 0,
    ICSSG_UTILS_R30_CMD_BLOCK,
    ICSSG_UTILS_R30_CMD_FORWARD,
    ICSSG_UTILS_R30_CMD_FORWARD_WO_LEARNING,
    ICSSG_UTILS_R30_CMD_ACCEPT_ALL,
    ICSSG_UTILS_R30_CMD_ACCEPT_TAGGED,
    ICSSG_UTILS_R30_CMD_ACCEPT_UNTAGGED_N_PRIO,
    ICSSG_UTILS_R30_CMD_TAS_TRIGGER,
    ICSSG_UTILS_R30_CMD_TAS_ENABLE,
    ICSSG_UTILS_R30_CMD_TAS_RESET,
    ICSSG_UTILS_R30_CMD_TAS_DISABLE,
    ICSSG_UTILS_R30_CMD_UC_FLOODING_ENABLE,
    ICSSG_UTILS_R30_CMD_UC_FLOODING_DISABLE,
    ICSSG_UTILS_R30_CMD_MC_FLOODING_ENABLE,
    ICSSG_UTILS_R30_CMD_MC_FLOODING_DISABLE,
    ICSSG_UTILS_R30_CMD_PREMPT_TX_ENABLE,
    ICSSG_UTILS_R30_CMD_PREMPT_TX_DISABLE,
    ICSSG_UTILS_R30_CMD_VLAN_AWARE_ENABLE,
    ICSSG_UTILS_R30_CMD_VLAN_AWARE_DISABLE,
    ICSSG_UTILS_R30_CMD_DSCP_ENABLE,
    ICSSG_UTILS_R30_CMD_DSCP_DISABLE,
    ICSSG_UTILS_MAX_COMMANDS
} IcssgUtils_ioctlR30Cmd;

typedef enum Icssg_MgmtHwQ_e
{
    ICSSG_MGMT_FREE_HWQA,
    ICSSG_MGMT_RX_HWQA,
    ICSSG_MGMT_TX_HWQA,
    ICSSG_TXTS_FREE_HWQA,
    ICSSG_TXTS_RX_HWQA,
} Icssg_MgmtHwQ;

typedef struct IcssgUtils_MgmtPkt_s
{
    uint32_t link;
    Icssg_IoctlCmd payload;
} IcssgUtils_MgmtPkt;

typedef struct Icssg_Filter3Cfg_S
{
    uint32_t ft3Start;
    uint32_t ft3StartAuto;
    uint32_t ft3StartOffset;
    uint32_t ft3JmpOffset;
    uint32_t ft3Len;
    uint32_t ft3Config;
    uint32_t ft3Type;
    uint32_t ft3TypeMask;
    uint32_t ft3PatternLow;
    uint32_t ft3PatternHigh;
    uint32_t ft3PatternMaskLow;
    uint32_t ft3PatternMaskHigh;
} Icssg_Filter3Cfg;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

uint64_t Icssg_rd64(Icssg_Handle hIcssg,
                    uintptr_t addr);

uint32_t Icssg_rd32(Icssg_Handle hIcssg,
                    uintptr_t addr);

uint16_t Icssg_rd16(Icssg_Handle hIcssg,
                    uintptr_t addr);

uint8_t Icssg_rd8(Icssg_Handle hIcssg,
                  uintptr_t addr);

void Icssg_wr64(Icssg_Handle hIcssg,
                uintptr_t addr,
                uint64_t val);

void Icssg_wr32(Icssg_Handle hIcssg,
                uintptr_t addr,
                uint32_t val);

void Icssg_wr16(Icssg_Handle hIcssg,
                uintptr_t addr,
                uint16_t val);

void Icssg_wr8(Icssg_Handle hIcssg,
               uintptr_t addr,
               uint8_t val);

uint32_t IcssgUtils_getSliceNum(Icssg_Handle hIcssg,
                                Enet_MacPort macPort);

uintptr_t Icssg_getDramAddr(Icssg_Handle hIcssg,
                            Enet_MacPort macPort);

uintptr_t Icssg_getCfgAddr(Icssg_Handle hIcssg);

uintptr_t Icssg_getRgmiiCfgAddr(Icssg_Handle hIcssg);

uintptr_t Icssg_getSharedRamAddr(Icssg_Handle hIcssg);

uintptr_t Icssg_getDfltVlanAddr(Icssg_Handle hIcssg,
                                Enet_MacPort macPort);

uintptr_t Icssg_getVlanTableAddr(Icssg_Handle hIcssg);

void IcssgUtils_R30CmdInit(Icssg_Handle hIcssg,
                           Enet_MacPort macPort);

bool IcssgUtils_isR30CmdDone(Icssg_Handle hIcssg,
                             Enet_MacPort macPort);

int32_t Icssg_R30SendAsyncIoctl(Icssg_Handle hIcssg,
                                Enet_MacPort macPort,
                                IcssgUtils_ioctlR30Cmd cmd,
                                uint8_t *seqNum,
                                uint32_t *asyncIoctlType);

int32_t Icssg_R30SendSyncIoctl(Icssg_Handle hIcssg,
                               Enet_MacPort macPort,
                               IcssgUtils_ioctlR30Cmd cmd);

int32_t IcssgUtils_sendFdbCmd(Icssg_Handle hIcssg,
                              Enet_MacPort macPort,
                              uint32_t subCmd,
                              const Icssg_FdbEntry *entry,
                              uint16_t broadSideSlot,
                              uint8_t fid);

int32_t IcssgUtils_createPruss(Icssg_Handle hIcssg);

int32_t IcssgUtils_enablePruss(Icssg_Handle hIcssg,
                               Enet_MacPort macPort);

int32_t IcssgUtils_disablePruss(Icssg_Handle hIcssg,
                                Enet_MacPort macPort);

int32_t IcssgUtils_downloadFirmware(Icssg_Handle hIcssg,
                                    Enet_MacPort macPort,
                                    const Icssg_Fw *fw);

void IcssgUtils_WriteMem(uintptr_t addr,
                         const void *ptr,
                         uint32_t element_count);

void *IcssgUtils_hwqPop(Icssg_Handle hIcssg,
                        Enet_MacPort macPort,
                        Icssg_MgmtHwQ hwQ);

void IcssgUtils_hwqPush(Icssg_Handle hIcssg,
                        Enet_MacPort macPort,
                        Icssg_MgmtHwQ hwQ,
                        void *p);

void IcssgUtils_hwqPushForSlice(Icssg_Handle hIcssg,
                                uint32_t slice,
                                Icssg_MgmtHwQ hwQ,
                                void *p);

void *IcssgUtils_hwqPeek(Icssg_Handle hIcssg,
                         Enet_MacPort macPort,
                         Icssg_MgmtHwQ hwQ);

int32_t IcssgUtils_hwqLevel(Icssg_Handle hIcssg,
                            Enet_MacPort macPort,
                            Icssg_MgmtHwQ hwQ);

void IcssgUtils_hwqReset(Icssg_Handle hIcssg,
                         uint32_t queueNum);

int32_t IcssgUtils_sendHwqMgmtMsg(Icssg_Handle hIcssg,
                                  Enet_MacPort macPort);

int32_t IcssgUtils_checkPortMode(Icssg_Handle hIcssg,
                                 const EnetMacPort_Interface *mii);

void IcssgUtils_fwConfig(Icssg_Handle hIcssg,
                         Enet_MacPort macPort,
                         const Icssg_Cfg *cfg,
                         const Icssg_FwPoolMem *fwPoolMem,
                         uint32_t rxPktFlowStart);

void IcssgUtils_configSwtFw(Icssg_Handle hIcssg,
                            const Icssg_Cfg *cfg,
                            const Icssg_FwPoolMem *fwPoolMem0,
                            const Icssg_FwPoolMem *fwPoolMem1,
                            uint32_t rxPktFlowStart0,
                            uint32_t rxPktFlowStart1);

/*!
 * \brief utility to return lower 16 bits of 32 bit crc on array of 8 bytes
 */
uint16_t IcssgUtils_CRC64(uint8_t input[8]);

/*!
 * \brief return broad-side slot to use for fdb entry
 *        also return the 8 bit fid to populate as part of the entry
 */
uint16_t IcssgUtils_FdbHelper(uintptr_t vlanTable,
                              int16_t vlanId,
                              uint8_t mac[],
                              uint8_t *fid);

void IcssgUtils_classiDisable(Icssg_Handle hIcssg,
                              Enet_MacPort macPort);

void IcssgUtils_configFilter3(Icssg_Handle hIcssg,
                              Enet_MacPort macPort,
                              uint32_t filterNum,
                              Icssg_Filter3Cfg *ftCfg);

int32_t IcssgUtils_checkFwPoolMem(Icssg_Handle hIcssg,
                                  const Icssg_FwPoolMem *fwPoolMem);

/* ========================================================================== */
/*                        Deprecated Function Declarations                    */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* ICSSG_H_ */

/*! @} */
