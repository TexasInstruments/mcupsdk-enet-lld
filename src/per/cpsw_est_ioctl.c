/*
 *  Copyright (c) Texas Instruments Incorporated 2022-23
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
 * \file  cpsw_est.c
 *
 * \brief This file contains the implementation of the global configuration
 *        needed for EST feature.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdarg.h>
#include <csl_cpswitch.h>
#include <enet_cfg.h>
#include <include/core/enet_base.h>
#if defined(SOC_AM273X) || defined(SOC_AWR294X) || defined (SOC_AWR2544) || defined(SOC_AM263X) || defined(SOC_AM263PX)
#include <priv/per/cpsw_cpdma_priv.h>
#else
#include <priv/per/cpsw_priv.h>
#endif
#include <include/per/cpsw.h>
#include <priv/per/cpsw_est_priv.h>
#include <priv/per/cpsw_est_ioctl_priv.h>
#include <priv/mod/cpsw_macport_ioctl_priv.h>
#include <priv/mod/cpsw_cpts_ioctl_priv.h>


#if ENET_CFG_IS_ON(CPSW_EST)

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
static int32_t Cpsw_setupEstf(Cpsw_Handle hCpsw,
                              Enet_MacPort macPort,
                              uint64_t cycleTimeNs,
                              uint64_t baseTimeNs);


/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
int32_t CpswEst_ioctl_handler_ENET_TAS_IOCTL_SET_ADMIN_LIST(Cpsw_Handle hCpsw, uint32_t cmd, Enet_IoctlPrms *prms)
{
    EnetTas_SetAdminListInArgs *inArgs = (EnetTas_SetAdminListInArgs *)prms->inArgs;
    /* Note: Typecast to GenericInArgs is possible because all public
     *       TAS IOCTL input args have macPort as their first member */
    EnetTas_GenericInArgs *genInArgs = (EnetTas_GenericInArgs *)prms->inArgs;
    Enet_MacPort macPort = genInArgs->macPort;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    int32_t status = ENET_SOK;

    Enet_devAssert(portNum < CPSW_MAC_PORT_NUM);
    if (hCpsw->estState[portNum].state == ENET_TAS_ENABLE)
    {
        if (inArgs->adminList.cycleTime != hCpsw->estState[portNum].cycleTime)
        {
            ENETTRACE_ERR("Port %u: Admin cycle time (%llu) and oper (%llu) must be same\n",
                          ENET_MACPORT_ID(macPort),
                          inArgs->adminList.cycleTime,
                          hCpsw->estState[portNum].cycleTime);
            status = ENET_ENOTSUPPORTED;
        }
        else if (inArgs->adminList.baseTime != 0ULL)
        {
            ENETTRACE_ERR("Port %u: AdminBasetime in the future is not supported, must pass 0\n",
                          ENET_MACPORT_ID(macPort));
            status = ENET_ENOTSUPPORTED;
        }
    }

    /* CPSW MAC port will take care of writing new admin list to EST RAM */
    if (status == ENET_SOK)
    {
        CPSW_MACPORT_PRIV_IOCTL(hCpsw->hMacPort[portNum], ENET_TAS_IOCTL_SET_ADMIN_LIST, prms, status);
    }

    /* Save cycle time so CPTS ESTF can be setup accordingly */
    if (status == ENET_SOK)
    {
        hCpsw->estState[portNum].cycleTime     = inArgs->adminList.cycleTime;
        hCpsw->estState[portNum].adminBaseTime = inArgs->adminList.baseTime;
    }
    return status;
}

int32_t CpswEst_ioctl_handler_ENET_TAS_IOCTL_SET_STATE(Cpsw_Handle hCpsw, uint32_t cmd, Enet_IoctlPrms *prms)
{
    EnetTas_SetStateInArgs *inArgs = (EnetTas_SetStateInArgs *)prms->inArgs;
    /* Note: Typecast to GenericInArgs is possible because all public
     *       TAS IOCTL input args have macPort as their first member */
    EnetTas_GenericInArgs *genInArgs = (EnetTas_GenericInArgs *)prms->inArgs;
    Enet_MacPort macPort = genInArgs->macPort;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    int32_t status = ENET_SOK;

    Enet_devAssert(portNum < CPSW_MAC_PORT_NUM);
    if (hCpsw->estState[portNum].state != inArgs->state)
    {
        if (inArgs->state == ENET_TAS_ENABLE)
        {
            /* Enable port-specific EST functionality */
            CPSW_MACPORT_PRIV_IOCTL(hCpsw->hMacPort[portNum], ENET_TAS_IOCTL_SET_STATE, prms, status);

            /* Enable ESTF with saved cycle time */
            if (status == ENET_SOK)
            {
                status = Cpsw_setupEstf(hCpsw, macPort,
                                        hCpsw->estState[portNum].cycleTime,
                                        hCpsw->estState[portNum].adminBaseTime);
                ENETTRACE_ERR_IF((status != ENET_SOK),
                                 "Port %u: Failed to enable ESTF: %d\n",
                                 ENET_MACPORT_ID(macPort), status);
            }
        }
        else
        {
            /* Disable ESTF for that port */
            status = Cpsw_setupEstf(hCpsw, macPort, 0U, 0U);
            ENETTRACE_ERR_IF((status != ENET_SOK),
                             "Port %u: Failed to disable ESTF: %d\n",
                             ENET_MACPORT_ID(macPort), status);

            /* Disable (or reset) port-specific EST */
            if (status == ENET_SOK)
            {
                CPSW_MACPORT_PRIV_IOCTL(hCpsw->hMacPort[portNum], ENET_TAS_IOCTL_SET_STATE, prms, status);
            }
        }

        if (status == ENET_SOK)
        {
            hCpsw->estState[portNum].state = inArgs->state;
        }
    }
    return status;
}

int32_t CpswEst_ioctl_handler_ENET_TAS_IOCTL_GET_ADMIN_LIST(Cpsw_Handle hCpsw, uint32_t cmd, Enet_IoctlPrms *prms)
{
    EnetTas_ControlList *controlList = (EnetTas_ControlList *)prms->outArgs;
    /* Note: Typecast to GenericInArgs is possible because all public
     *       TAS IOCTL input args have macPort as their first member */
    EnetTas_GenericInArgs *genInArgs = (EnetTas_GenericInArgs *)prms->inArgs;
    Enet_MacPort macPort = genInArgs->macPort;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    int32_t status = ENET_SOK;

    Enet_devAssert(portNum < CPSW_MAC_PORT_NUM);
    CPSW_MACPORT_PRIV_IOCTL(hCpsw->hMacPort[portNum], ENET_TAS_IOCTL_GET_ADMIN_LIST, prms, status);

    controlList->cycleTime = hCpsw->estState[portNum].cycleTime;
    controlList->baseTime  = hCpsw->estState[portNum].adminBaseTime;
    return status;
}

int32_t CpswEst_ioctl_handler_ENET_TAS_IOCTL_GET_OPER_LIST(Cpsw_Handle hCpsw, uint32_t cmd, Enet_IoctlPrms *prms)
{
    EnetTas_ControlList *controlList = (EnetTas_ControlList *)prms->outArgs;
    /* Note: Typecast to GenericInArgs is possible because all public
     *       TAS IOCTL input args have macPort as their first member */
    EnetTas_GenericInArgs *genInArgs = (EnetTas_GenericInArgs *)prms->inArgs;
    Enet_MacPort macPort = genInArgs->macPort;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    int32_t status = ENET_SOK;

    Enet_devAssert(portNum < CPSW_MAC_PORT_NUM);
    CPSW_MACPORT_PRIV_IOCTL(hCpsw->hMacPort[portNum], ENET_TAS_IOCTL_GET_OPER_LIST, prms, status);

    controlList->cycleTime = hCpsw->estState[portNum].cycleTime;
    controlList->baseTime  = hCpsw->estState[portNum].operBaseTime;
    return status;
}

int32_t CpswEst_ioctl_handler_ENET_TAS_IOCTL_GET_OPER_LIST_STATUS(Cpsw_Handle hCpsw, uint32_t cmd, Enet_IoctlPrms *prms)
{
    EnetTas_OperStatus *operStatus = (EnetTas_OperStatus *)prms->outArgs;
    /* Note: Typecast to GenericInArgs is possible because all public
     *       TAS IOCTL input args have macPort as their first member */
    EnetTas_GenericInArgs *genInArgs = (EnetTas_GenericInArgs *)prms->inArgs;
    Enet_MacPort macPort = genInArgs->macPort;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    int32_t status = ENET_SOK;

    Enet_devAssert(portNum < CPSW_MAC_PORT_NUM);
    /* Update operBaseTime when oper list has been updated */
    CPSW_MACPORT_PRIV_IOCTL(hCpsw->hMacPort[portNum], ENET_TAS_IOCTL_GET_OPER_LIST_STATUS, prms, status);
    if ((status == ENET_SOK) &&
        (*operStatus == ENET_TAS_OPER_LIST_UPDATED))
    {
        hCpsw->estState[portNum].operBaseTime = hCpsw->estState[portNum].adminBaseTime;
    }
    return status;
}

int32_t CpswEst_ioctl_handler_ENET_TAS_IOCTL_GET_STATE(Cpsw_Handle hCpsw, uint32_t cmd, Enet_IoctlPrms *prms)
{
    /* Note: Typecast to GenericInArgs is possible because all public
     *       TAS IOCTL input args have macPort as their first member */
    EnetTas_GenericInArgs *genInArgs = (EnetTas_GenericInArgs *)prms->inArgs;
    Enet_MacPort macPort = genInArgs->macPort;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    int32_t status = ENET_SOK;

    Enet_devAssert(portNum < CPSW_MAC_PORT_NUM);
    /* Pass through */
    CPSW_MACPORT_PRIV_IOCTL(hCpsw->hMacPort[portNum], ENET_TAS_IOCTL_GET_STATE, prms, status);
    return status;
}

int32_t CpswEst_ioctl_handler_ENET_TAS_IOCTL_CONFIG_CHANGE_STATUS_PARAMS(Cpsw_Handle hCpsw, uint32_t cmd, Enet_IoctlPrms *prms)
{
    /* Note: Typecast to GenericInArgs is possible because all public
     *       TAS IOCTL input args have macPort as their first member */
    EnetTas_GenericInArgs *genInArgs = (EnetTas_GenericInArgs *)prms->inArgs;
    Enet_MacPort macPort = genInArgs->macPort;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    int32_t status = ENET_SOK;

    /* Pass through */
    CPSW_MACPORT_PRIV_IOCTL(hCpsw->hMacPort[portNum], ENET_TAS_IOCTL_CONFIG_CHANGE_STATUS_PARAMS, prms, status);
    return status;
}

static int32_t Cpsw_setupEstf(Cpsw_Handle hCpsw,
                              Enet_MacPort macPort,
                              uint64_t cycleTimeNs,
                              uint64_t baseTimeNs)
{
    Enet_IoctlPrms prms;
    CpswCpts_SetFxnGenInArgs setGenFInArgs;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    uint32_t cptsClkPeriod;
    int32_t status;

    /* CPTS RFT clock frequency is an enumeration whose numeric value
     * represents the CPTS clock period - 1 */
    cptsClkPeriod = (uint32_t)hCpsw->cptsRftClkFreq + 1U;

    /* Enable ESTF for the given MAC port */
    setGenFInArgs.index   = portNum;
    setGenFInArgs.length  = (uint32_t)cycleTimeNs / cptsClkPeriod;
    setGenFInArgs.compare = baseTimeNs;
    setGenFInArgs.polarityInv = false;
    setGenFInArgs.ppmVal  = 0U;
    setGenFInArgs.ppmDir  = CPSW_CPTS_GENF_PPM_ADJDIR_DECREASE;
    setGenFInArgs.ppmMode = ENET_TIMESYNC_ADJMODE_DISABLE;

    ENET_IOCTL_SET_IN_ARGS(&prms, &setGenFInArgs);
    CPSW_CPTS_PRIV_IOCTL(hCpsw->hCpts, CPSW_CPTS_IOCTL_SET_ESTF, &prms, status);

    return status;
}

#endif
