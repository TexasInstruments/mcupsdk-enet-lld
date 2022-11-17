/*
 *  Copyright (c) Texas Instruments Incorporated 2021
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
 * \file  icssg_tas.c
 *
 * \brief This file contains the implementation of the ICSSG Tas driver.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdlib.h>
#include <hw_include/cslr_icss.h>
#include <drivers/hw_include/hw_types.h>
#include <include/core/enet_base.h>
#include <include/core/enet_utils.h>
#include <include/core/enet_mod.h>
#include <include/per/icssg.h>
#include <src/per/icssg_utils.h>
#include <priv/mod/icssg_tas_priv.h>
#include <priv/mod/icssg_tas_ioctl_priv.h>
#include <src/per/firmware/icssg/fw_mem_map.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define HWREG(x)         (*((volatile uint32_t *)(x)))
#define HWREGH(x)        (*((volatile uint16_t *)(x)))
#define HWREGB(x)        (*((volatile uint8_t *)(x)))

/*!
 * \brief List number 0 or 1. Also the value at memory location TAS_ACTIVE_LIST_INDEX
 */
typedef enum IcssgTas_ListNum_s
{
    ICSSG_TAS_LIST1 = 0U,
    ICSSG_TAS_LIST2 = 1U
} IcssgTas_ListNum;

#define ICSSG_TAS_IOCTL_HANDLER_ENTRY_INIT(x)    \
          {.cmd = x,                            \
           .fxn = &IcssgTas_ioctl_handler_##x}

#define ICSSG_TAS_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(x)    \
          {.cmd = x,                            \
           .fxn = &IcssgTas_ioctl_handler_default}


/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */
/*!
 * \brief Tas Ioctl Handler Function.
 *
 * \param hPer      Enet Peripheral handle
 * \param cmd       IOCTL command Id
 * \param prms      IOCTL parameters
 */
typedef int32_t IcssgTasIoctlHandlerFxn_t(EnetMod_Handle hMod,
                                       uint32_t cmd,
                                       Enet_IoctlPrms *prms);

/*!
 * \brief Tas IOCTL register Handler structure.
 */
typedef struct IcssgTasIoctlHandlerTableEntry_s
{
    uint32_t cmd;
    IcssgTasIoctlHandlerFxn_t *fxn;

} IcssgTasIoctlHandlerTableEntry_t;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void IcssgTas_getFirmwareListPointers(IcssgTas_Handle hTas);

static int32_t IcssgTas_setState(IcssgTas_Handle hTas,
                                 Enet_MacPort macPort,
                                 EnetTas_TasState state);

static int32_t IcssgTas_updateOperList(IcssgTas_Handle hTas,
                                       Enet_MacPort macPort,
                                       EnetTas_ControlList* adminList);

static int32_t IcssgTas_setTriggerForListChange(IcssgTas_Handle hTas,
                                                Enet_MacPort macPort);

static EnetTas_OperStatus IccsgTas_updateOperListStatus(IcssgTas_Handle hTas);

static IcssgTasIoctlHandlerFxn_t * Icssg_getTasIoctlHandler(EnetMod_Handle hMod,
                                                                uint32_t cmd,
                                                                IcssgTasIoctlHandlerTableEntry_t ioctlTbl[],
                                                                uint32_t numEntries);

int32_t IcssgTas_ioctl_handler_default(EnetMod_Handle hMod,
                                            uint32_t cmd,
                                            Enet_IoctlPrms *prms);

int32_t IcssgTas_ioctl_handler_ICSSG_TAS_IOCTL_REGISTER_HANDLER(EnetMod_Handle hMod,
                                                                        uint32_t cmd,
                                                                        Enet_IoctlPrms *prms);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static IcssgTasIoctlHandlerTableEntry_t IcssgTasIoctlHandlerTable[] =
{
    ICSSG_TAS_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_TAS_IOCTL_SET_ADMIN_LIST),
    ICSSG_TAS_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_TAS_IOCTL_GET_OPER_LIST_STATUS),
    ICSSG_TAS_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_TAS_IOCTL_SET_STATE),
    ICSSG_TAS_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_TAS_IOCTL_GET_STATE),
    ICSSG_TAS_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_TAS_IOCTL_GET_ADMIN_LIST),
    ICSSG_TAS_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_TAS_IOCTL_GET_OPER_LIST),
    ICSSG_TAS_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_TAS_IOCTL_CONFIG_CHANGE_STATUS_PARAMS),
    ICSSG_TAS_IOCTL_HANDLER_ENTRY_INIT(ICSSG_TAS_IOCTL_REGISTER_HANDLER),
};

#if ENET_CFG_IS_ON(DEV_ERROR)
/* Public ICSSG peripheral IOCTL validation data. */
static Enet_IoctlValidate gIcssgTas_ioctlValidate[] =
{
    ENET_IOCTL_VALID_PRMS(ENET_TAS_IOCTL_SET_ADMIN_LIST,
                          sizeof(EnetTas_SetAdminListInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_TAS_IOCTL_GET_OPER_LIST_STATUS,
                          sizeof(Enet_MacPort),
                          sizeof(EnetTas_OperStatus)),

    ENET_IOCTL_VALID_PRMS(ENET_TAS_IOCTL_SET_STATE,
                          sizeof(EnetTas_SetStateInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_TAS_IOCTL_GET_STATE,
                          0U,
                          sizeof(EnetTas_TasState)),

    ENET_IOCTL_VALID_PRMS(ENET_TAS_IOCTL_GET_ADMIN_LIST,
                          sizeof(EnetTas_RegisterIoctlInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_TAS_IOCTL_GET_OPER_LIST,
                          0U,
                          sizeof(EnetTas_ControlList)),

    ENET_IOCTL_VALID_PRMS(ICSSG_TAS_IOCTL_REGISTER_HANDLER,
                          sizeof(EnetTas_RegisterIoctlInArgs),
                          0U),

};
#endif

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t IcssgTas_open(EnetMod_Handle hMod,
                      Enet_Type enetType,
                      uint32_t instId,
                      const void *cfg,
                      uint32_t cfgSize)
{
    IcssgTas_Handle hTas = (IcssgTas_Handle)hMod;
    uintptr_t dmemOffset = (uintptr_t)hMod->virtAddr;
    int32_t status = ENET_SOK;

    Enet_devAssert(cfgSize == 0U,
                   "Invalid ICSSG Tas config params size %u (expected %u)\r\n",
                   cfgSize, 0U);

    hTas->configStatus = (volatile EnetTas_ConfigStatus *)(dmemOffset + TAS_CONFIG_CHANGE_TIME);

    IcssgTas_getFirmwareListPointers(hTas);

    memset((void*)hTas->fwActiveList, 0, sizeof(IcssgTas_FwList));
    memset((void*)hTas->fwShadowList, 0, sizeof(IcssgTas_FwList));

    return status;
}

void IcssgTas_close(EnetMod_Handle hMod)
{
    return;
}

int32_t IcssgTas_rejoin(EnetMod_Handle hMod,
                        Enet_Type enetType,
                        uint32_t instId)
{
    return ENET_ENOTSUPPORTED;
}

int32_t IcssgTas_ioctl(EnetMod_Handle hMod,
                       uint32_t cmd,
                       Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;
    IcssgTasIoctlHandlerFxn_t * ioctlHandler;

#if ENET_CFG_IS_ON(DEV_ERROR)

    if(ENET_IOCTL_GET_MAJ(cmd) == ENET_IOCTL_TAS_BASE)
    {
        /* Validate ICSSG TAS Module IOCTL parameters */
        status = Enet_validateIoctl(cmd, prms,
                                    gIcssgTas_ioctlValidate,
                                    ENET_ARRAYSIZE(gIcssgTas_ioctlValidate));
        ENETTRACE_ERR_IF((status != ENET_SOK),
                         "ICSSG_TAS: IOCTL 0x%08x params are not valid\r\n", cmd);
    }
    else
    {
        status = ENET_EINVALIDPARAMS;
    }
#endif
    if(ENET_SOK == status)
    {
        ioctlHandler = Icssg_getTasIoctlHandler(hMod, cmd, IcssgTasIoctlHandlerTable , ENET_ARRAYSIZE(IcssgTasIoctlHandlerTable));
        Enet_assert(ioctlHandler != NULL);
        status = ioctlHandler(hMod, cmd, prms);
    }
    return status;

}

void IcssgTas_getFirmwareListPointers(IcssgTas_Handle hTas)
{
    EnetMod_Handle hMod = ENET_MOD(hTas);
    uintptr_t dmemOffset = (uintptr_t)hMod->virtAddr;
    IcssgTas_ListNum activeList = (IcssgTas_ListNum)*(volatile uint8_t *)(dmemOffset + TAS_ACTIVE_LIST_INDEX);

    if (activeList == ICSSG_TAS_LIST1)
    {
        hTas->fwActiveList = (IcssgTas_FwList *)(dmemOffset + TAS_GATE_MASK_LIST0);
        hTas->fwShadowList = (IcssgTas_FwList *)(dmemOffset + TAS_GATE_MASK_LIST1);
    }
    else if (activeList == ICSSG_TAS_LIST2)
    {
        hTas->fwActiveList = (IcssgTas_FwList *)(dmemOffset + TAS_GATE_MASK_LIST1);
        hTas->fwShadowList = (IcssgTas_FwList *)(dmemOffset + TAS_GATE_MASK_LIST0);
    }
    else
    {
        Enet_devAssert(false, "Invalid TAS active list index %u\r\n", activeList);
    }
}

void IcssgTas_reset(IcssgTas_Handle hTas)
{
    /* Initialize the structures */
    memset(&(hTas->operList), 0, sizeof(EnetTas_ControlList));
    memset(&(hTas->adminList), 0, sizeof(EnetTas_ControlList));
}

int32_t IcssgTas_setState(IcssgTas_Handle hTas,
                          Enet_MacPort macPort,
                          EnetTas_TasState state)
{
    Icssg_Handle hIcssg = (Icssg_Handle)hTas->hIcssg;
    IcssgUtils_ioctlR30Cmd cmd;
    int32_t status = ENET_SOK;

    if (state == ENET_TAS_ENABLE)
    {
        cmd = ICSSG_UTILS_R30_CMD_TAS_ENABLE;
        hTas->state = ENET_TAS_ENABLE;
    }

    if (state == ENET_TAS_DISABLE)
    {
        cmd = ICSSG_UTILS_R30_CMD_TAS_DISABLE;
        hTas->state = ENET_TAS_DISABLE;
    }

    if (state == ENET_TAS_RESET)
    {
        IcssgTas_reset(hTas);
        cmd = ICSSG_UTILS_R30_CMD_TAS_RESET;
        hTas->state = ENET_TAS_RESET;
    }

    status = Icssg_R30SendAsyncIoctl(hIcssg,
                                     macPort,
                                     cmd,
                                     &(hIcssg->asyncIoctlSeqNum),
                                     &(hIcssg->asyncIoctlType));

    return status;
}

int32_t IcssgTas_updateOperList(IcssgTas_Handle hTas,
                                Enet_MacPort macPort,
                                EnetTas_ControlList* adminList)
{
    EnetMod_Handle hMod = ENET_MOD(hTas);
    uintptr_t dmemOffset = (uintptr_t)hMod->virtAddr;
    uint8_t windowIdx, gateIdx, i;
    uint32_t tasAccGateCloseTime = 0U;
    uint8_t *maxSduTablePointer;
    int32_t status = ENET_SOK;

    /* List legth of zero is not allowed */
    if (adminList->listLength == 0U)
    {
        status = ENET_EINVALIDPARAMS;
    }

    if (status == ENET_SOK)
    {
        /* If any timeInterval is 0 in between the list, then exit with error */
        for (windowIdx = 0U; windowIdx < adminList->listLength - 1; windowIdx++)
        {
            if (adminList->gateCmdList[windowIdx].timeInterval == 0U)
            {
                status = ENET_EINVALIDPARAMS;
            }
        }
    }

    if (status == ENET_SOK)
    {
        IcssgTas_getFirmwareListPointers(hTas);

        /* Copy the adminList into the Tas Handle */
        memcpy(&hTas->adminList, adminList, sizeof(EnetTas_ControlList));

        for (windowIdx = 0U; windowIdx < hTas->adminList.listLength; windowIdx++)
        {
            /* Update the gate mask in the shadow list */
            hTas->fwShadowList->gateMaskList[windowIdx] = hTas->adminList.gateCmdList[windowIdx].gateStateMask;
            /* Update the window close time in the shadow list */
            tasAccGateCloseTime += hTas->adminList.gateCmdList[windowIdx].timeInterval;
            if (windowIdx == hTas->adminList.listLength - 1)
            {
                /* Extend the last entry in the list till end of cycle */
                hTas->fwShadowList->windowEndTimeList[windowIdx] = hTas->adminList.cycleTime;
            }
            else
            {
                hTas->fwShadowList->windowEndTimeList[windowIdx] = tasAccGateCloseTime;
            }
        }

        for (windowIdx = hTas->adminList.listLength; windowIdx < ENET_TAS_MAX_CMD_LISTS; windowIdx++)
        {
            /* Update the gate mask in the shadow list */
            hTas->fwShadowList->gateMaskList[windowIdx] = 0x00U;
            hTas->fwShadowList->windowEndTimeList[windowIdx] = 0U;
        }

        /* Update the Array of gate close time for each queue in each window */
        for (windowIdx = 0U; windowIdx < hTas->adminList.listLength; windowIdx++)
        {
            for (gateIdx = 0U; gateIdx < ENET_TAS_MAX_NUM_QUEUES; gateIdx++)
            {
                uint32_t gateCloseTime = 0U;

                if (!ENET_IS_BIT_SET(hTas->fwShadowList->gateMaskList[windowIdx], gateIdx))
                {
                    /* gate closed */
                    gateCloseTime = 0U;
                }
                else
                {
                    /* gate open */
                    for (i = windowIdx; i < hTas->adminList.listLength; i++)
                    {
                        if (!ENET_IS_BIT_SET(hTas->fwShadowList->gateMaskList[i], gateIdx))
                        {
                            break;
                        }
                        else
                        {
                            gateCloseTime = hTas->fwShadowList->windowEndTimeList[i];
                        }
                    }
                }
                hTas->fwShadowList->gateCloseTimeList[windowIdx][gateIdx] = gateCloseTime;
            }
        }

        /* Update the maxsdu table */
        maxSduTablePointer = (uint8_t *)(dmemOffset + TAS_QUEUE_MAX_SDU_LIST);

        for (gateIdx = 0U; gateIdx < ENET_TAS_MAX_NUM_QUEUES; gateIdx++)
        {
            HWREGH(maxSduTablePointer + gateIdx*2U) = hTas->adminList.sduTable.maxSDU[gateIdx];
        }

        status = IcssgTas_setTriggerForListChange(hTas, macPort);
    }

    return status;
}

int32_t IcssgTas_setTriggerForListChange(IcssgTas_Handle hTas,
                                         Enet_MacPort macPort)
{
    EnetMod_Handle hMod = ENET_MOD(hTas);
    Icssg_Handle hIcssg = (Icssg_Handle)hTas->hIcssg;
    uintptr_t dmemOffset = (uintptr_t)hMod->virtAddr;
    uintptr_t smemOffset = (uintptr_t)hMod->virtAddr2;
    uint32_t tsCycleCounter;
    uint64_t cycleTime;
    uint64_t baseTime;
    uint32_t cycleCount;
    int64_t temp = 0;
    int32_t status = ENET_SOK;

    tsCycleCounter = HWREG(smemOffset + TIMESYNC_FW_WC_CYCLECOUNT_OFFSET);

    cycleTime = hTas->adminList.cycleTime - 4U; /* Subtracting 4ns to compensate for IEP wrap around time */
    baseTime = hTas->adminList.baseTime;

    if (cycleTime > 0U)
    {
        temp = ((baseTime / cycleTime) - tsCycleCounter);
    }

    if (temp <= 0) /* Time at which user wants to load has already past, so we will load at start of next cycle */
    {
        cycleCount = 1U;
    }
    else if ((baseTime % cycleTime) != 0U) /* Implementing ceiling function */
    {
        cycleCount = (uint32_t)(temp + 1);
    }
    else
    {
        cycleCount = (uint32_t)(temp);
    }

    HWREG(dmemOffset + TAS_ADMIN_CYCLE_TIME) = cycleTime;
    HWREG(dmemOffset + TAS_CONFIG_CHANGE_CYCLE_COUNT) = cycleCount;
    HWREGB(dmemOffset + TAS_ADMIN_LIST_LENGTH) = hTas->adminList.listLength;

    hTas->configStatus->configChange = 1U;
    /* indicates firmware can now copy data list from DMEM and put in BSRAM */
    hTas->configStatus->configPending = 1U;

    /* Also set r.30.config.change bit for firmware */
    status = Icssg_R30SendAsyncIoctl(hIcssg,
                                     macPort,
                                     ICSSG_UTILS_R30_CMD_TAS_TRIGGER,
                                     &(hIcssg->asyncIoctlSeqNum),
                                     &(hIcssg->asyncIoctlType));

    return status;
}

EnetTas_OperStatus IccsgTas_updateOperListStatus(IcssgTas_Handle hTas)
{
    EnetTas_OperStatus operStatus = ENET_TAS_OPER_LIST_NOT_YET_UPDATED;

    if (hTas->configStatus->configChange == 0U)
    {
        /* Update the firmware list pointers */
        IcssgTas_getFirmwareListPointers(hTas);

        /* Copy the admin list to active list */
        memcpy(&(hTas->operList), &(hTas->adminList), sizeof(EnetTas_ControlList));

        Icssg_Handle hIcssg = (Icssg_Handle)hTas->hIcssg;
        hIcssg->cycleTimeNs = hTas->operList.cycleTime;

        operStatus = ENET_TAS_OPER_LIST_UPDATED;
    }

    return operStatus;
}

int32_t  IcssgTas_ioctl_handler_ENET_TAS_IOCTL_SET_ADMIN_LIST(EnetMod_Handle hMod,
                                             uint32_t cmd,
                                             Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;
    IcssgTas_Handle hTas = (IcssgTas_Handle)hMod;
    Enet_assert(cmd == ENET_TAS_IOCTL_SET_ADMIN_LIST);

    EnetTas_SetAdminListInArgs *inArgs = (EnetTas_SetAdminListInArgs *)prms->inArgs;

    status = IcssgTas_updateOperList(hTas, inArgs->macPort, &inArgs->adminList);
    ENETTRACE_ERR_IF((status != ENET_SINPROGRESS),
                        "%s: Failed to set admin list: %d\r\n", hMod->name, status);

    return status;
}

int32_t  IcssgTas_ioctl_handler_ENET_TAS_IOCTL_GET_OPER_LIST_STATUS(EnetMod_Handle hMod,
                                             uint32_t cmd,
                                             Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;
    IcssgTas_Handle hTas = (IcssgTas_Handle)hMod;
    Enet_assert(cmd == ENET_TAS_IOCTL_GET_OPER_LIST_STATUS);

    EnetTas_OperStatus *operStatus = (EnetTas_OperStatus *)prms->outArgs;

    *operStatus = IccsgTas_updateOperListStatus(hTas);
    ENETTRACE_ERR_IF((status != ENET_SOK),
                        "%s: Failed to set admin list: %d\r\n", hMod->name, status);

    return status;
}

int32_t  IcssgTas_ioctl_handler_ENET_TAS_IOCTL_SET_STATE(EnetMod_Handle hMod,
                                             uint32_t cmd,
                                             Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;
    IcssgTas_Handle hTas = (IcssgTas_Handle)hMod;
    Enet_assert(cmd == ENET_TAS_IOCTL_SET_STATE);

    EnetTas_SetStateInArgs *inArgs = (EnetTas_SetStateInArgs *)prms->inArgs;

    status = IcssgTas_setState(hTas, inArgs->macPort, inArgs->state);
    ENETTRACE_ERR_IF((status != ENET_SINPROGRESS),
                        "%s: Failed to set TAS state: %d\r\n", hMod->name, status);

    return status;
}

int32_t  IcssgTas_ioctl_handler_ENET_TAS_IOCTL_GET_STATE(EnetMod_Handle hMod,
                                             uint32_t cmd,
                                             Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;
    IcssgTas_Handle hTas = (IcssgTas_Handle)hMod;
    Enet_assert(cmd == ENET_TAS_IOCTL_GET_STATE);

    EnetTas_TasState *state = (EnetTas_TasState *)prms->outArgs;
    *state = hTas->state;

    return status;
}

int32_t  IcssgTas_ioctl_handler_ENET_TAS_IOCTL_GET_ADMIN_LIST(EnetMod_Handle hMod,
                                             uint32_t cmd,
                                             Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;
    IcssgTas_Handle hTas = (IcssgTas_Handle)hMod;
    Enet_assert(cmd == ENET_TAS_IOCTL_GET_ADMIN_LIST);

    EnetTas_ControlList *adminList = (EnetTas_ControlList *)prms->outArgs;
    memcpy(adminList, &(hTas->adminList), sizeof(EnetTas_ControlList));

    return status;
}

int32_t  IcssgTas_ioctl_handler_ENET_TAS_IOCTL_GET_OPER_LIST(EnetMod_Handle hMod,
                                             uint32_t cmd,
                                             Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;
    IcssgTas_Handle hTas = (IcssgTas_Handle)hMod;
    Enet_assert(cmd == ENET_TAS_IOCTL_GET_OPER_LIST);

    EnetTas_ControlList *operList = (EnetTas_ControlList *)prms->outArgs;
    memcpy(operList, &(hTas->operList), sizeof(EnetTas_ControlList));

    return status;
}

int32_t  IcssgTas_ioctl_handler_ENET_TAS_IOCTL_CONFIG_CHANGE_STATUS_PARAMS(EnetMod_Handle hMod,
                                             uint32_t cmd,
                                             Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;
    IcssgTas_Handle hTas = (IcssgTas_Handle)hMod;
    Enet_assert(cmd == ENET_TAS_IOCTL_CONFIG_CHANGE_STATUS_PARAMS);

    EnetTas_ConfigStatus *configChangeStatus = (EnetTas_ConfigStatus *)prms->outArgs;
    memcpy(configChangeStatus, (void*)hTas->configStatus, sizeof(EnetTas_ConfigStatus));

    return status;
}


static int32_t Icssg_getTasIoctlHandlerEntry(EnetMod_Handle hMod, uint32_t cmd, IcssgTasIoctlHandlerTableEntry_t ioctlTbl[], uint32_t numEntries, uint32_t *entryIdx)
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
        ENETTRACE_ERR("%s: failed to get ioctl handler for ICSSG TAS IOCTL cmd %x\r\n", cmd);
        status = ENET_EINVALIDPARAMS;
    }
    return status;
}

static IcssgTasIoctlHandlerFxn_t * Icssg_getTasIoctlHandler(EnetMod_Handle hMod, uint32_t cmd, IcssgTasIoctlHandlerTableEntry_t ioctlTbl[], uint32_t numEntries)
{
    uint32_t entryIndex;
    int32_t status;
    IcssgTasIoctlHandlerFxn_t *ioctlHandler = NULL;

    status = Icssg_getTasIoctlHandlerEntry(hMod, cmd, ioctlTbl, numEntries, &entryIndex);
    if (status == ENET_SOK)
    {
        Enet_assert(entryIndex < numEntries);
        ioctlHandler = ioctlTbl[entryIndex].fxn;
    }
    else
    {
        ENETTRACE_ERR("%s: failed to get ioctl handler for ICSSG TAS IOCTL cmd %x\r\n", cmd);
        ioctlHandler = &IcssgTas_ioctl_handler_default;
    }
    return ioctlHandler;
}


int32_t IcssgTas_ioctl_handler_default(EnetMod_Handle hMod,
                                    uint32_t cmd,
                                    Enet_IoctlPrms *prms)
{
    return ENET_ENOTSUPPORTED;
}

int32_t IcssgTas_ioctl_handler_registerHandler(EnetMod_Handle hMod,
                                                    uint32_t cmd,
                                                    Enet_IoctlPrms *prms)
{

    int32_t status = ENET_SOK;
    EnetTas_RegisterIoctlInArgs *ioctlHandlerToRegister  = (EnetTas_RegisterIoctlInArgs *)prms->inArgs;
    IcssgTasIoctlHandlerTableEntry_t *currentIoctlTblEntry;
    uint32_t entryIndex;

    Enet_assert(cmd == ICSSG_TAS_IOCTL_REGISTER_HANDLER);
    status = Icssg_getTasIoctlHandlerEntry(hMod, ioctlHandlerToRegister->registerHandler.cmd,
                                        IcssgTasIoctlHandlerTable ,
                                        ENET_ARRAYSIZE(IcssgTasIoctlHandlerTable),
                                        &entryIndex);
    if (status == ENET_SOK)
    {
        Enet_assert(entryIndex < ENET_ARRAYSIZE(IcssgTasIoctlHandlerTable));
        currentIoctlTblEntry = &IcssgTasIoctlHandlerTable[entryIndex];
        Enet_assert(ioctlHandlerToRegister->registerHandler.cmd == currentIoctlTblEntry->cmd);
        currentIoctlTblEntry->fxn = (IcssgTasIoctlHandlerFxn_t *)ioctlHandlerToRegister->registerHandler.fxn;
    }
    else
    {
        ENETTRACE_ERR("%s: failed to get ioctl handler for ICSSG TAS IOCTL cmd %x\r\n", cmd);
        status = ENET_EINVALIDPARAMS;
    }
    return status;
}

int32_t IcssgTas_ioctl_handler_ICSSG_TAS_IOCTL_REGISTER_HANDLER(EnetMod_Handle hMod,
                                                                        uint32_t cmd,
                                                                        Enet_IoctlPrms *prms)
{
    int32_t status;

    status = IcssgTas_ioctl_handler_registerHandler(hMod, cmd, prms);
    return status;
}