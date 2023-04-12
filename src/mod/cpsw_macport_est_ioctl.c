/*
 *  Copyright (c) Texas Instruments Incorporated 2022
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
 * \file  cpsw_macport_est.c
 *
 * \brief This file contains the implementation of the CPSW EST functionality.
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
#include <include/mod/cpsw_macport.h>
#include <priv/core/enet_trace_priv.h>
#include <priv/mod/cpsw_macport_priv.h>
#include <priv/mod/cpsw_clks.h>
#include "cpsw_macport_est_ioctl_priv.h"

#if ENET_CFG_IS_ON(CPSW_MACPORT_EST)


/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/*! EST upper buffer location offset */
#define CPSW_MACPORT_EST_BUF_UPPER_LOC        (64U)


/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
static int32_t CpswEst_writeGateControlList(CpswMacPort_Handle hPort,
                                            const EnetTas_ControlList *controlList);
static EnetTas_OperStatus CpswEst_getOperListStatus(CpswMacPort_Handle hPort);
static EnetTas_TasState CpswEst_getState(CpswMacPort_Handle hPort);
static int32_t CpswEst_enableTimestamp(CpswMacPort_Handle hPort,
                                       const CpswMacPort_EstTimestampCfg *cfg);
static void CpswEst_disableTimestamp(CpswMacPort_Handle hPort);
static int32_t CpswEst_checkGateCmdList(CpswMacPort_Handle hPort,
                                        const EnetTas_ControlList *controlList);
static int32_t CpswEst_readGateControlList(CpswMacPort_Handle hPort,
                                           bool estBufUpper,
                                           EnetTas_ControlList *controlList);

int32_t CpswEst_setState(CpswMacPort_Handle hPort, EnetTas_TasState state);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */


int32_t CpswMacPortEst_ioctl_handler_ENET_TAS_IOCTL_SET_ADMIN_LIST(CpswMacPort_Handle hPort, Enet_IoctlPrms *prms)
{
    EnetTas_SetAdminListInArgs *inArgs = (EnetTas_SetAdminListInArgs *)prms->inArgs;
    EnetTas_ControlList *adminList = &inArgs->adminList;
    int32_t status = ENET_SOK;

    Enet_devAssert((hPort->macPort == inArgs->macPort),
                   "MAC %u: Port mismatch %u\n", ENET_MACPORT_ID(hPort->macPort), inArgs->macPort);

    status = CpswEst_writeGateControlList(hPort, adminList);
    return status;
}

int32_t CpswMacPortEst_ioctl_handler_ENET_TAS_IOCTL_GET_OPER_LIST_STATUS(CpswMacPort_Handle hPort, Enet_IoctlPrms *prms)
{
    EnetTas_GenericInArgs *inArgs = (EnetTas_GenericInArgs *)prms->inArgs;
    EnetTas_OperStatus *operStatus = (EnetTas_OperStatus *)prms->outArgs;
    int32_t status = ENET_SOK;

    Enet_devAssert((hPort->macPort == inArgs->macPort),
                   "MAC %u: Port mismatch %u\n", ENET_MACPORT_ID(hPort->macPort), inArgs->macPort);

    *operStatus = CpswEst_getOperListStatus(hPort);
    return status;
}

int32_t CpswMacPortEst_ioctl_handler_ENET_TAS_IOCTL_SET_STATE(CpswMacPort_Handle hPort, Enet_IoctlPrms *prms)
{
    EnetTas_SetStateInArgs *inArgs = (EnetTas_SetStateInArgs *)prms->inArgs;
    int32_t status = ENET_SOK;

    Enet_devAssert((hPort->macPort == inArgs->macPort),
                   "MAC %u: Port mismatch %u\n", ENET_MACPORT_ID(hPort->macPort), inArgs->macPort);

    status = CpswEst_setState(hPort, inArgs->state);
    return status;
}

int32_t CpswMacPortEst_ioctl_handler_ENET_TAS_IOCTL_GET_STATE(CpswMacPort_Handle hPort, Enet_IoctlPrms *prms)
{
    EnetTas_GenericInArgs *inArgs = (EnetTas_GenericInArgs *)prms->inArgs;
    EnetTas_TasState *state = (EnetTas_TasState *)prms->outArgs;
    int32_t status = ENET_SOK;

    Enet_devAssert((hPort->macPort == inArgs->macPort),
                   "MAC %u: Port mismatch %u\n", ENET_MACPORT_ID(hPort->macPort), inArgs->macPort);

    *state = CpswEst_getState(hPort);
    return status;
}

int32_t CpswMacPortEst_ioctl_handler_ENET_TAS_IOCTL_GET_ADMIN_LIST(CpswMacPort_Handle hPort, Enet_IoctlPrms *prms)
{
    EnetTas_GenericInArgs *inArgs = (EnetTas_GenericInArgs *)prms->inArgs;
    EnetTas_ControlList *adminList = (EnetTas_ControlList *)prms->outArgs;
    int32_t status = ENET_SOK;

    Enet_devAssert((hPort->macPort == inArgs->macPort),
                   "MAC %u: Port mismatch %u\n", ENET_MACPORT_ID(hPort->macPort), inArgs->macPort);

    memcpy(adminList, &hPort->adminList, sizeof(*adminList));
    return status;
}

int32_t CpswMacPortEst_ioctl_handler_ENET_TAS_IOCTL_GET_OPER_LIST(CpswMacPort_Handle hPort, Enet_IoctlPrms *prms)
{
    EnetTas_GenericInArgs *inArgs = (EnetTas_GenericInArgs *)prms->inArgs;
    EnetTas_ControlList *operList = (EnetTas_ControlList *)prms->outArgs;
    int32_t status = ENET_SOK;

    Enet_devAssert((hPort->macPort == inArgs->macPort),
                   "MAC %u: Port mismatch %u\n", ENET_MACPORT_ID(hPort->macPort), inArgs->macPort);

    memcpy(operList, &hPort->operList, sizeof(*operList));
    return status;
}

int32_t CpswMacPortEst_ioctl_handler_ENET_TAS_IOCTL_CONFIG_CHANGE_STATUS_PARAMS(CpswMacPort_Handle hPort, Enet_IoctlPrms *prms)
{
    EnetTas_GenericInArgs *inArgs = (EnetTas_GenericInArgs *)prms->inArgs;
    EnetTas_ConfigStatus *configChangeStatus = (EnetTas_ConfigStatus *)prms->outArgs;
    int32_t status = ENET_SOK;

    Enet_devAssert((hPort->macPort == inArgs->macPort),
                   "MAC %u: Port mismatch %u\n", ENET_MACPORT_ID(hPort->macPort), inArgs->macPort);

    memcpy(configChangeStatus, &hPort->configStatus, sizeof(*configChangeStatus));
    return status;
}

int32_t CpswMacPortEst_ioctl_handler_CPSW_MACPORT_IOCTL_EST_ENABLE_TIMESTAMP(CpswMacPort_Handle hPort, Enet_IoctlPrms *prms)
{
    CpswMacPort_EstTimestampCfg *inArgs = (CpswMacPort_EstTimestampCfg *)prms->inArgs;
    int32_t status = ENET_SOK;

    Enet_devAssert((hPort->macPort == inArgs->macPort),
                   "MAC %u: Port mismatch %u\n", ENET_MACPORT_ID(hPort->macPort), inArgs->macPort);

    status = CpswEst_enableTimestamp(hPort, inArgs);
    return status;
}

int32_t CpswMacPortEst_ioctl_handler_CPSW_MACPORT_IOCTL_EST_DISABLE_TIMESTAMP(CpswMacPort_Handle hPort, Enet_IoctlPrms *prms)
{
    EnetMacPort_GenericInArgs *inArgs = (EnetMacPort_GenericInArgs *)prms->inArgs;
    int32_t status = ENET_SOK;

    Enet_devAssert((hPort->macPort == inArgs->macPort),
                   "MAC %u: Port mismatch %u\n", ENET_MACPORT_ID(hPort->macPort), inArgs->macPort);

    CpswEst_disableTimestamp(hPort);
    return status;
}

static int32_t CpswEst_writeGateControlList(CpswMacPort_Handle hPort,
                                            const EnetTas_ControlList *controlList)
{
    EnetMod_Handle hMod = ENET_MOD(hPort);
    Enet_MacPort macPort = hPort->macPort;
    CSL_Xge_cpswRegs *regs = (CSL_Xge_cpswRegs *)hMod->virtAddr;
    CSL_CPSW_EST_CONFIG estCfg;
    const EnetTas_GateCmdEntry *cmd;
    uint32_t portNum = ENET_MACPORT_NORM(macPort) + 1U;
    uint32_t estBufLoc;
    uint32_t fetchCnt;
    uint32_t timeIntvlStep;
    uint64_t timeIntvlAcc = 0LLU;
    uint8_t lastGateStateMask = 0U;
    uint32_t i;
    int32_t status = ENET_SOK;

    /* Link speed is needed to compute fetch count as it is in Ethernet wireside clocks
     * (bytes in 1000, nibbles in 10/100) */
    if (hPort->enabled)
    {
        switch (hPort->linkCfg.speed)
        {
            case ENET_SPEED_10MBIT:
                timeIntvlStep = CPSW_MACPORT_EST_TIME_STEP_10M;
                break;
            case ENET_SPEED_100MBIT:
                timeIntvlStep = CPSW_MACPORT_EST_TIME_STEP_100M;
                break;
            case ENET_SPEED_1GBIT:
            default:
                timeIntvlStep = CPSW_MACPORT_EST_TIME_STEP_1G;
                break;
        }
    }
    else
    {
        ENETTRACE_ERR("MAC %u: Admin list can be set only when link is up\n",
                      ENET_MACPORT_ID(macPort));
        status = ENET_EPERM;
    }

    /* Check gate command list parameters passed from application */
    if (status == ENET_SOK)
    {
        status = CpswEst_checkGateCmdList(hPort, controlList);
        ENETTRACE_ERR_IF((status != ENET_SOK),
                         "MAC %u: Invalid gate control list params\n",
                         ENET_MACPORT_ID(macPort));
    }

    if (status == ENET_SOK)
    {
        /* Start offset depends on the admin EST bank */
        estBufLoc = hPort->estBufUpper ? CPSW_MACPORT_EST_BUF_UPPER_LOC : 0U;

        /* Write control list to EST fetch buffer */
        for (i = 0U; i < controlList->listLength; i++)
        {
            cmd = &controlList->gateCmdList[i];

            /* Stop processing the control list when a zero time interval is found */
            if (cmd->timeInterval == 0U)
            {
                break;
            }

            fetchCnt = ENET_DIV_ROUNDUP(cmd->timeInterval, timeIntvlStep);
            lastGateStateMask = cmd->gateStateMask;

            ENETTRACE_DBG("MAC %u: idx %u: loc %u: timeInterval=%u ns (%u) gateStateMask=0x%02x\n",
                          ENET_MACPORT_ID(macPort),
                          i, estBufLoc,
                          cmd->timeInterval,
                          fetchCnt,
                          cmd->gateStateMask);

            CSL_CPSW_writeEstFetchCmd(regs, portNum, estBufLoc,
                                      fetchCnt,
                                      cmd->gateStateMask);

            timeIntvlAcc += cmd->timeInterval;
            estBufLoc++;
        }

        /* Stretch last entry if cycle time is longer than total time intervals */
        if (timeIntvlAcc < controlList->cycleTime)
        {
            if (i < CPSW_MACPORT_EST_BUF_SIZE)
            {
                ENETTRACE_DBG("MAC %u: idx %u: loc %u: timeInterval=stretch gateStateMask=0x%02x\n",
                              ENET_MACPORT_ID(macPort),
                              i, estBufLoc,
                              lastGateStateMask);
                CSL_CPSW_writeEstFetchCmd(regs, portNum, estBufLoc, 0, lastGateStateMask);
                estBufLoc++;
                i++;
            }
        }

        /* Zero-out the remaining EST fetch buffer entries */
        for (; i < CPSW_MACPORT_EST_BUF_SIZE; i++)
        {
            CSL_CPSW_writeEstFetchCmd(regs, portNum, estBufLoc, 0U, 0U);
            estBufLoc++;
        }

        /* Select EST bank just populated as the active one */
        CSL_CPSW_getPortEstConfig(regs, portNum, &estCfg);
        estCfg.estBufSel = hPort->estBufUpper ? 1U : 0U;
        CSL_CPSW_setPortEstConfig(regs, portNum, &estCfg);

        /* Update local config status */
        hPort->configStatus.configChangeTime = controlList->baseTime;
        hPort->configStatus.configPending = 1U;
        hPort->configStatus.configChange  = 1U;
        if (hPort->state == ENET_TAS_ENABLE)
        {
            hPort->configStatus.configChangeErrorCounter++;
        }
        else if (hPort->state == ENET_TAS_RESET)
        {
            hPort->state = ENET_TAS_DISABLE;
        }

        /* Use other bank for next admin list */
        hPort->estBufUpper = !hPort->estBufUpper;
    }

    /* Save last successful admin list */
    if (status == ENET_SOK)
    {
        memcpy(&hPort->adminList, controlList, sizeof(hPort->adminList));
    }

    return status;
}

static EnetTas_OperStatus CpswEst_getOperListStatus(CpswMacPort_Handle hPort)
{
    EnetMod_Handle hMod = ENET_MOD(hPort);
    Enet_MacPort macPort = hPort->macPort;
    CSL_Xge_cpswRegs *regs = (CSL_Xge_cpswRegs *)hMod->virtAddr;
    EnetTas_OperStatus operStatus = ENET_TAS_OPER_LIST_NOT_YET_UPDATED;
    CSL_CPSW_PORT_CONTROL portControl;
    CSL_CPGMAC_SL_FIFOSTATUS fifoStatus;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    bool operEstBufUpper;
    bool curEstBufUpper;
    int32_t status;

    if (hPort->configStatus.configChange == 1U)
    {
        /* Oper EST bank is the opposite bank of next admin bank (estBufUpper) */
        operEstBufUpper = !hPort->estBufUpper;

        CSL_CPSW_getPortControlReg(regs, portNum + 1U, &portControl);
        if (portControl.estPortEnable == 0U)
        {
            /* Active EST buffer can't be checked when EST is disabled */
            curEstBufUpper = operEstBufUpper;
            ENETTRACE_DBG("MAC %u: Ignoring active buffer bank check as EST is disabled\n",
                          ENET_MACPORT_ID(macPort));
        }
        else
        {
            /* Get active EST buffer for the MAC port */
            CSL_CPGMAC_SL_getFifoStatus(regs, portNum, &fifoStatus);
            curEstBufUpper = (fifoStatus.estBufAct == 1U);

            ENETTRACE_DBG("MAC %u: Active buffer bank: %s, target oper bank: %s\n",
                          ENET_MACPORT_ID(macPort),
                          curEstBufUpper ? "LOWER" : "UPPER",
                          operEstBufUpper ? "LOWER" : "UPPER");
        }

        /* Check if active EST buffer is the 'oper' EST bank */
        if (curEstBufUpper == operEstBufUpper)
        {
            operStatus = ENET_TAS_OPER_LIST_UPDATED;

            hPort->operList.cycleTime = hPort->adminList.cycleTime;
            hPort->configStatus.configPending = 0U;
            hPort->configStatus.configChange  = 0U;

            status = CpswEst_readGateControlList(hPort, operEstBufUpper, &hPort->operList);
            ENETTRACE_WARN_IF((status != ENET_SOK),
                              "MAC %u: Failed to save oper list: %d\n",
                              ENET_MACPORT_ID(macPort), status);
        }
        else
        {
            operStatus = ENET_TAS_OPER_LIST_NOT_YET_UPDATED;
        }

        /* TODO - Check error status in FIFO_STATUS_REG */
    }

    return operStatus;
}

static EnetTas_TasState CpswEst_getState(CpswMacPort_Handle hPort)
{
    EnetMod_Handle hMod = ENET_MOD(hPort);
    Enet_MacPort macPort = hPort->macPort;
    CSL_Xge_cpswRegs *regs = (CSL_Xge_cpswRegs *)hMod->virtAddr;
    CSL_CPSW_PORT_CONTROL portControl = {0};
    uint32_t portNum = ENET_MACPORT_NORM(macPort) + 1U;
    EnetTas_TasState state;

    CSL_CPSW_getPortControlReg(regs, portNum, &portControl);
    state = (portControl.estPortEnable != 0U) ? ENET_TAS_ENABLE : ENET_TAS_DISABLE;

    return state;
}

static int32_t CpswEst_enableTimestamp(CpswMacPort_Handle hPort,
                                       const CpswMacPort_EstTimestampCfg *cfg)
{
    EnetMod_Handle hMod = ENET_MOD(hPort);
    Enet_MacPort macPort = hPort->macPort;
    CSL_Xge_cpswRegs *regs = (CSL_Xge_cpswRegs *)hMod->virtAddr;
    CSL_CPSW_EST_CONFIG estCfg;
    uint32_t portNum = ENET_MACPORT_NORM(macPort) + 1U;
    int32_t status = ENET_SOK;

    CSL_CPSW_getPortEstConfig(regs, portNum, &estCfg);

    switch (cfg->mode)
    {
        case CPSW_MACPORT_EST_TIMESTAMP_ALL:
        {
            ENETTRACE_DBG("MAC %u: Enable timestamping on all packets\n",
                          ENET_MACPORT_ID(macPort));

            /* All express packets on any priority */
            estCfg.estTsFirst  = 0U;
            estCfg.estTsOnePri = 0U;
            estCfg.estTsPri    = 0U;
            break;
        }

        case CPSW_MACPORT_EST_TIMESTAMP_ONEPRI:
        {
            ENETTRACE_DBG("MAC %u: Enable timestamping on all packets of priority %u\n",
                          ENET_MACPORT_ID(macPort), cfg->priority);

            /* All packet of the given priority */
            if (cfg->priority <= ENET_PRI_MAX)
            {
                estCfg.estTsFirst  = 0U;
                estCfg.estTsOnePri = 1U;
                estCfg.estTsPri    = cfg->priority;
            }
            else
            {
                ENETTRACE_ERR("MAC %u: Invalid timestamping priority %u\n",
                              ENET_MACPORT_ID(macPort), cfg->priority);
            }
            break;
        }

        case CPSW_MACPORT_EST_TIMESTAMP_FIRST:
        {
            ENETTRACE_DBG("MAC %u: Enable timestamping on first packet of each time interval\n",
                          ENET_MACPORT_ID(macPort));

            /* First packet of each time interval */
            estCfg.estTsFirst  = 1U;
            estCfg.estTsOnePri = 0U;
            estCfg.estTsPri    = 0U;
            break;
        }

        case CPSW_MACPORT_EST_TIMESTAMP_FIRST_ONEPRI:
        {
            ENETTRACE_DBG("MAC %u: Enable timestamping on first packet of priority %u\n",
                          ENET_MACPORT_ID(macPort), cfg->priority);

            /* First packet of the given priority */
            if (cfg->priority <= ENET_PRI_MAX)
            {
                estCfg.estTsFirst  = 1U;
                estCfg.estTsOnePri = 1U;
                estCfg.estTsPri    = cfg->priority;
            }
            else
            {
                ENETTRACE_ERR("MAC %u: Invalid timestamping priority %u\n",
                              ENET_MACPORT_ID(macPort), cfg->priority);
            }
            break;
        }

        default:
        {
            ENETTRACE_ERR("MAC %u: Invalid EST timestamping mode %u\n",
                          ENET_MACPORT_ID(macPort), cfg->mode);
            status = ENET_EINVALIDPARAMS;
            break;
        }
    }

    if (status == ENET_SOK)
    {
        CSL_CPSW_setEstTsDomain(regs, cfg->domain);

        estCfg.estTsEnable = 1U;
        CSL_CPSW_setPortEstConfig(regs, portNum, &estCfg);
    }

    return status;
}

static void CpswEst_disableTimestamp(CpswMacPort_Handle hPort)
{
    EnetMod_Handle hMod = ENET_MOD(hPort);
    Enet_MacPort macPort = hPort->macPort;
    CSL_Xge_cpswRegs *regs = (CSL_Xge_cpswRegs *)hMod->virtAddr;
    CSL_CPSW_EST_CONFIG estCfg;
    uint32_t portNum = ENET_MACPORT_NORM(macPort) + 1U;

    ENETTRACE_DBG("MAC %u: Enable timestamping on first packet of each time interval\n",
                  ENET_MACPORT_ID(macPort));

    CSL_CPSW_getPortEstConfig(regs, portNum, &estCfg);

    estCfg.estTsFirst  = 0U;
    estCfg.estTsOnePri = 0U;
    estCfg.estTsPri    = 0U;
    estCfg.estTsEnable = 1U;

    CSL_CPSW_setPortEstConfig(regs, portNum, &estCfg);
}

static int32_t CpswEst_checkGateCmdList(CpswMacPort_Handle hPort,
                                        const EnetTas_ControlList *controlList)
{
#if ((ENET_CFG_TRACE_LEVEL >= ENET_CFG_TRACE_LEVEL_ERROR) && ENET_CFG_IS_OFF(TRACE_DISABLE_INFOSTRING))
    Enet_MacPort macPort = hPort->macPort;
#endif
    const EnetTas_GateCmdEntry *cmd;
    uint32_t listLength = controlList->listLength;
    uint32_t timeIntvlMin;
    uint32_t timeIntvlMax;
    bool zeroIntvl = false;
    uint32_t i;
    int32_t status = ENET_SOK;

    switch (hPort->linkCfg.speed)
    {
        case ENET_SPEED_10MBIT:
            timeIntvlMin = CPSW_MACPORT_EST_TIME_MIN(10M);
            timeIntvlMax = CPSW_MACPORT_EST_TIME_MAX(10M);
            break;
        case ENET_SPEED_100MBIT:
            timeIntvlMin = CPSW_MACPORT_EST_TIME_MIN(100M);
            timeIntvlMax = CPSW_MACPORT_EST_TIME_MAX(100M);
            break;
        case ENET_SPEED_1GBIT:
        default:
            timeIntvlMin = CPSW_MACPORT_EST_TIME_MIN(1G);
            timeIntvlMax = CPSW_MACPORT_EST_TIME_MAX(1G);
            break;
    }

    /* Check that EST bank size is non-zero and is sufficient for admin list length */
    if (listLength == 0U)
    {
        ENETTRACE_ERR("MAC %u: Invalid gate control list length (%u), can't be 0\n",
                      ENET_MACPORT_ID(macPort), listLength);
        status = ENET_EINVALIDPARAMS;
    }
    else if (listLength > ENET_TAS_MAX_CMD_LISTS)
    {
        ENETTRACE_ERR("MAC %u: Invalid gate control list length (%u), max is %u\n",
                      ENET_MACPORT_ID(macPort), listLength, ENET_TAS_MAX_CMD_LISTS);
        status = ENET_EINVALIDPARAMS;
    }

    /* Check that all time intervals in admin list are valid */
    if (status == ENET_SOK)
    {
        for (i = 0U; i < listLength; i++)
        {
            cmd = &controlList->gateCmdList[i];

            if (cmd->timeInterval == 0U)
            {
                zeroIntvl = true;
            }
            else if (cmd->timeInterval < timeIntvlMin)
            {
                ENETTRACE_ERR("MAC %u: idx %u: Invalid time interval (%u ns), min is %u ns\n",
                              ENET_MACPORT_ID(macPort), i,
                              cmd->timeInterval,
                              timeIntvlMin);
                status = ENET_EINVALIDPARAMS;
            }
            else if (cmd->timeInterval > timeIntvlMax)
            {
                ENETTRACE_ERR("MAC %u: idx %u: Invalid time interval (%u ns), max is %u ns\n",
                              ENET_MACPORT_ID(macPort), i,
                              cmd->timeInterval,
                              timeIntvlMax);
                status = ENET_EINVALIDPARAMS;
            }
            else
            {
                if (zeroIntvl)
                {
                    ENETTRACE_ERR("MAC %u: idx %u: Non-zero interval can't follow a zero interval\n",
                                  ENET_MACPORT_ID(macPort), i);
                    status = ENET_EINVALIDPARAMS;
                }
            }
        }
    }

    return status;
}

static int32_t CpswEst_readGateControlList(CpswMacPort_Handle hPort,
                                           bool estBufUpper,
                                           EnetTas_ControlList *controlList)
{
    EnetMod_Handle hMod = ENET_MOD(hPort);
    Enet_MacPort macPort = hPort->macPort;
    CSL_Xge_cpswRegs *regs = (CSL_Xge_cpswRegs *)hMod->virtAddr;
    EnetTas_GateCmdEntry *cmd;
    uint32_t portNum = ENET_MACPORT_NORM(macPort) + 1U;
    uint32_t estBufLoc;
    uint32_t timeIntvlStep;
    uint32_t fetchCnt;
    uint8_t fetchAllow;
    uint32_t i;
    int32_t status = ENET_SOK;

    /* Link speed is needed to compute fetch count as it is in Ethernet wireside clocks
     * (bytes in 1000, nibbles in 10/100) */
    if (hPort->enabled)
    {
        switch (hPort->linkCfg.speed)
        {
            case ENET_SPEED_10MBIT:
                timeIntvlStep = CPSW_MACPORT_EST_TIME_STEP_10M;
                break;
            case ENET_SPEED_100MBIT:
                timeIntvlStep = CPSW_MACPORT_EST_TIME_STEP_100M;
                break;
            case ENET_SPEED_1GBIT:
            default:
                timeIntvlStep = CPSW_MACPORT_EST_TIME_STEP_1G;
                break;
        }
    }
    else
    {
        ENETTRACE_ERR("MAC %u: Oper list can be read only when link is up\n",
                      ENET_MACPORT_ID(macPort));
        status = ENET_EPERM;
    }

    if (status == ENET_SOK)
    {
        if (ENET_TAS_MAX_CMD_LISTS > CPSW_MACPORT_EST_BUF_SIZE)
        {
            ENETTRACE_ERR("MAC %u: Invalid gate control list length (%u), max is %u\n",
                          ENET_MACPORT_ID(macPort), ENET_TAS_MAX_CMD_LISTS, CPSW_MACPORT_EST_BUF_SIZE);
            status = ENET_EINVALIDPARAMS;
        }
    }

    /* Read control list from EST fetch buffer */
    if (status == ENET_SOK)
    {
        estBufLoc = estBufUpper ? CPSW_MACPORT_EST_BUF_UPPER_LOC : 0U;

        for (i = 0U; i < CPSW_MACPORT_EST_BUF_SIZE; i++)
        {
            CSL_CPSW_readEstFetchCmd(regs, portNum, estBufLoc,
                                     &fetchCnt,
                                     &fetchAllow);

            if ((fetchCnt == 0U) &&
                (fetchAllow == 0U))
            {
                controlList->listLength = i;
                break;
            }

            if (i == ENET_TAS_MAX_CMD_LISTS)
            {
                ENETTRACE_WARN("MAC %u: Partial list, length %u not enough\n",
                               ENET_MACPORT_ID(macPort), ENET_TAS_MAX_CMD_LISTS);
                status = ENET_EINVALIDPARAMS;
                break;
            }

            cmd = &controlList->gateCmdList[i];
            cmd->timeInterval  = fetchCnt * timeIntvlStep;
            cmd->gateStateMask = fetchAllow;

            ENETTRACE_DBG("MAC %u: idx %u: loc %u: timeInterval=%u ns (%u) gateStateMask=0x%02x\n",
                          ENET_MACPORT_ID(macPort),
                          i, estBufLoc,
                          cmd->timeInterval,
                          fetchCnt,
                          cmd->gateStateMask);

            estBufLoc++;
        }
    }

    return status;
}

#endif
