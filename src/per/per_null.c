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
 * \file  per_null.c
 *
 * \brief This file contains the implementation of the "null" Ethernet
 *        Peripheral.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <include/core/enet_base.h>
#include <include/core/enet_trace.h>
#include <include/mod/mod_null.h>
#include <include/per/per_null.h>
#include <priv/core/enet_trace_priv.h>

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

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void NullPer_initCfg(EnetPer_Handle hPer,
                     Enet_Type enetType,
                     void *cfg,
                     uint32_t cfgSize)
{
    NullPer_Cfg *nullPerCfg = (NullPer_Cfg *)cfg;

    ENETTRACE_INFO("Initialize null peripheral config\n");

    /* Initialize mod1 config params */
    NullMod_initCfg(&nullPerCfg->mod1Cfg);

    /* Initialize mod2 config params */
    NullMod_initCfg(&nullPerCfg->mod2Cfg);
}

int32_t NullPer_open(EnetPer_Handle hPer,
                     Enet_Type enetType,
                     uint32_t instId,
                     const void *cfg,
                     uint32_t cfgSize)
{
    NullPer_Handle hNullPer = (NullPer_Handle)hPer;
    const NullPer_Cfg *nullPerCfg = (const NullPer_Cfg *)cfg;
    EnetMod_Handle hMod1 = ENET_MOD(&hNullPer->mod1);
    EnetMod_Handle hMod2 = ENET_MOD(&hNullPer->mod2);
    int32_t status = ENET_SOK;

    ENETTRACE_INFO("%s: Open null peripheral\n", hPer->name);

    /* Check if dummy feature 1 is enabled */
    if (ENET_FEAT_IS_EN(hPer->features, ENET_NULLPER_FEAT1))
    {
        ENETTRACE_INFO("%s: Feature 1 is enabled\n", hPer->name);
    }

    /* Check if dummy feature 2 is enabled */
    if (ENET_FEAT_IS_EN(hPer->features, ENET_NULLPER_FEAT2))
    {
        ENETTRACE_INFO("%s: Feature 2 is enabled\n", hPer->name);
    }

    /* Check if dummy errata 1 is applicable */
    if (ENET_ERRATA_IS_EN(hPer->errata, ENET_NULLPER_ERRATA1))
    {
        ENETTRACE_INFO("%s: Errata 1 is applicable\n", hPer->name);
    }

    /* Open module 1 */
    ENETTRACE_INFO("%s: Opening mod1\n", hPer->name);
    status = EnetMod_open(hMod1, enetType, instId, &nullPerCfg->mod1Cfg, sizeof(nullPerCfg->mod1Cfg));
    ENETTRACE_ERR_IF(status != ENET_SOK, "%s: Failed to open mod1: %d\n", hPer->name, status);

    /* Open module 2 */
    if (status == ENET_SOK)
    {
        ENETTRACE_INFO("%s: Opening mod2\n", hPer->name);
        status = EnetMod_open(hMod2, enetType, instId, &nullPerCfg->mod2Cfg, sizeof(nullPerCfg->mod2Cfg));
        ENETTRACE_ERR_IF(status != ENET_SOK, "%s: Failed to open mod2: %d\n", hPer->name, status);
    }

    return status;
}

int32_t NullPer_rejoin(EnetPer_Handle hPer,
                       Enet_Type enetType,
                       uint32_t instId)
{
    NullPer_Handle hNullPer = (NullPer_Handle)hPer;
    EnetMod_Handle hMod1 = ENET_MOD(&hNullPer->mod1);
    EnetMod_Handle hMod2 = ENET_MOD(&hNullPer->mod2);
    int32_t status = ENET_SOK;

    ENETTRACE_INFO("%s: Rejoin null peripheral\n", hPer->name);

    /* Rejoin module 1 */
    ENETTRACE_INFO("%s: Rejoining mod1\n", hPer->name);
    status = EnetMod_rejoin(hMod1, enetType, instId);
    ENETTRACE_ERR_IF(status != ENET_SOK, "%s: Failed to rejoin mod1: %d\n", hPer->name, status);

    /* Rejoin module 2 */
    if (status == ENET_SOK)
    {
        ENETTRACE_INFO("%s: Rejoining mod2\n", hPer->name);
        status = EnetMod_rejoin(hMod2, enetType, instId);
        ENETTRACE_ERR_IF(status != ENET_SOK, "%s: Failed to rejoin mod2: %d\n", hPer->name, status);
    }

    return status;
}

int32_t NullPer_ioctl(EnetPer_Handle hPer,
                      uint32_t cmd,
                      Enet_IoctlPrms *prms)
{
    NullPer_Handle hNullPer = (NullPer_Handle)hPer;
    EnetMod_Handle hMod1 = ENET_MOD(&hNullPer->mod1);
    EnetMod_Handle hMod2 = ENET_MOD(&hNullPer->mod2);
    int32_t status = ENET_SOK;

    ENETTRACE_INFO("%s: IOCTL %u on null peripheral\n", hPer->name, cmd);

    /* Do IOCTL on module 1 */
    ENETTRACE_INFO("%s: IOCTL %u on mod1\n", hPer->name, cmd);
    status = EnetMod_ioctl(hMod1, cmd, prms);
    ENETTRACE_ERR_IF(status != ENET_SOK, "%s: Failed to do IOCTL %u on mod1: %d\n", hPer->name, cmd, status);

    /* Do IOCTL on module 2 */
    if (status == ENET_SOK)
    {
        ENETTRACE_INFO("%s: IOCTL %u on mod2\n", hPer->name, cmd);
        status = EnetMod_ioctl(hMod2, cmd, prms);
        ENETTRACE_ERR_IF(status != ENET_SOK, "%s: Failed to do IOCTL %u on mod2: %d\n", hPer->name, cmd, status);
    }

    return status;
}

void NullPer_poll(EnetPer_Handle hPer,
                  uint32_t evtMask)
{
    ENETTRACE_INFO("%s: events 0x%08x\n", hPer->name, evtMask);
}

void NullPer_periodicTick(EnetPer_Handle hPer)
{
    ENETTRACE_INFO("%s: tick!\n", hPer->name);
}

void NullPer_close(EnetPer_Handle hPer)
{
    NullPer_Handle hNullPer = (NullPer_Handle)hPer;
    EnetMod_Handle hMod1 = ENET_MOD(&hNullPer->mod1);
    EnetMod_Handle hMod2 = ENET_MOD(&hNullPer->mod2);

    ENETTRACE_INFO("%s: Close null peripheral\n", hPer->name);

    /* Close module 1 */
    ENETTRACE_INFO("%s: Closing mod1\n", hPer->name);
    EnetMod_close(hMod1);

    /* Close module 2 */
    ENETTRACE_INFO("%s: Closing mod2\n", hPer->name);
    EnetMod_close(hMod2);
}
