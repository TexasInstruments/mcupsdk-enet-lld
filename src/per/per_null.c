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
#include <include/core/enet_osal.h>
#include <include/core/enet_utils.h>
#include <include/common/enet_utils_dflt.h>
#include <include/core/enet_soc.h>
#include <include/mod/mod_null.h>
#include <include/per/per_null.h>
#include <priv/core/enet_trace_priv.h>
#include <include/core/enet_osal.h>

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

extern int32_t Enet_validateGenericIoctl(uint32_t cmd, const Enet_IoctlPrms *prms);

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

NullPer_Obj * NullPer_getHandle(uint32_t hEnet)
{
    return NULL;
}

void NullPer_init(const EnetUtils_Cfg *utilsCfg)
{
    uint32_t count;
    uint32_t i;
#if ENET_CFG_IS_ON(HAS_DEFAULT_UTILS)
    EnetUtils_Cfg dfltUtilsCfg;
#endif

    /* If defaut OSAL and/or utils is enabled, use them in case
    * the application hasn't provided any */
#if ENET_CFG_IS_ON(HAS_DEFAULT_UTILS)
    if (utilsCfg == NULL)
    {
        EnetUtilsDflt_initCfg(&dfltUtilsCfg);
        utilsCfg = &dfltUtilsCfg;
    }
#endif

    EnetUtils_init(utilsCfg);
    EnetSoc_init();

    /* Create top-level Enet locks for all peripherals in the SoC */
    count = EnetSoc_getEnetNum();
    NullPer_Handle hNullPer = NULL;
    uint32_t hEnet = -1U;
    for (i = 0U; i < count; i++)
    {
        hEnet = EnetSoc_getEnetHandleByIdx(i);
        hNullPer = NullPer_getHandle(hEnet);
        if (hNullPer != NULL)
        {
            hNullPer->lock = EnetOsal_createMutex();
            ENETTRACE_ERR_IF(hNullPer->lock == NULL,
                    "%s: Failed to create mutex\n", hNullPer->name);
        }
    }
}

void NullPer_deinit(void)
{
    uint32_t count;
    uint32_t i;

    /* Destroy all top-level Enet locks */
    count = EnetSoc_getEnetNum();
    NullPer_Handle hNullPer =NULL;
    for (i = 0U; i < count; i++)
    {
        hNullPer = NullPer_getHandle(i);
        if (hNullPer != NULL)
        {
            EnetOsal_deleteMutex(hNullPer->lock);
            hNullPer->lock = NULL;
        }
    }

    EnetSoc_deinit();
    EnetUtils_deinit();
}

void NullPer_initCfg(NullPer_Cfg *nullPerCfg)
{
    ENETTRACE_INFO("Initialize null peripheral config\n");

    /* Initialize mod1 config params */
    NullMod_initCfg(&nullPerCfg->mod1Cfg);

    /* Initialize mod2 config params */
    NullMod_initCfg(&nullPerCfg->mod2Cfg);
}

int32_t NullPer_open(uint32_t hEnet,
                     Enet_Type enetType,
                     uint32_t instId,
                     NullPer_Cfg *nullPerCfg)
{
    NullPer_Handle hNullPer = NULL;
    int32_t status = ENET_SOK;
    bool isAlreadyOpen = false;

    if (hEnet != -1)
    {
        hNullPer = NullPer_getHandle(hEnet);
        Enet_devAssert(hNullPer != NULL,
                       "Invalid EnetPer handle for %u.%u\n", enetType, instId);
    }
    else
    {
        ENETTRACE_ERR("No EnetPer %u:%u has been found\n", enetType, instId);
        status = ENET_ENOTFOUND;
        hNullPer = NULL;
    }
    
    ENETTRACE_ERR_IF(status != ENET_SOK,
    "Failed get handles for %u.%u: %d\n", enetType, instId, status);

    isAlreadyOpen = (hNullPer->magic == ENET_MAGIC) ? true : false;

    if (isAlreadyOpen == false)
    {
        EnetOsal_lockMutex(hNullPer->lock);

#if ENET_CFG_IS_ON(SANITY_CHECKS)
        /* Print enabled configurable features and applicable erratas */
        ENETTRACE_DBG("%s: features: 0x%08x\n", hNullPer->name, hNullPer->features);
        ENETTRACE_DBG("%s: errata  : 0x%08x\n", hNullPer->name, hNullPer->errata);
#endif

        ENETTRACE_VERBOSE("%s: Open peripheral\n", hNullPer->name);

        hNullPer->virtAddr  = EnetUtils_physToVirt(hNullPer->physAddr, NULL);
        hNullPer->virtAddr2 = EnetUtils_physToVirt(hNullPer->physAddr2, NULL);

        status = ENET_SOK;

        ENETTRACE_INFO("%s: Open null peripheral\n", hNullPer->name);

        /* Check if dummy feature 1 is enabled */
        if (ENET_FEAT_IS_EN(hNullPer->features, ENET_NULLPER_FEAT1))
        {
            ENETTRACE_INFO("%s: Feature 1 is enabled\n", hNullPer->name);
        }

        /* Check if dummy feature 2 is enabled */
        if (ENET_FEAT_IS_EN(hNullPer->features, ENET_NULLPER_FEAT2))
        {
            ENETTRACE_INFO("%s: Feature 2 is enabled\n", hNullPer->name);
        }

        /* Check if dummy errata 1 is applicable */
        if (ENET_ERRATA_IS_EN(hNullPer->errata, ENET_NULLPER_ERRATA1))
        {
            ENETTRACE_INFO("%s: Errata 1 is applicable\n", hNullPer->name);
        }

        /* Open module 1 */
        ENETTRACE_INFO("%s: Opening mod1\n", hNullPer->mod1.name);

        bool isMod1Open = (hNullPer->mod1.magic == ENET_MAGIC) ? true : false;

        if (isMod1Open == false)
        {
            hNullPer->mod1.virtAddr  = EnetUtils_physToVirt(hNullPer->mod1.physAddr, NULL);
            hNullPer->mod1.virtAddr2 = EnetUtils_physToVirt(hNullPer->mod1.physAddr2, NULL);

            status = NullMod_open(&hNullPer->mod1, enetType, instId, &nullPerCfg->mod1Cfg);
            if (status == ENET_SOK)
            {
                hNullPer->mod1.magic = ENET_MAGIC;
                ENETTRACE_VERBOSE("%s: Module is now open\n", hNullPer->mod1.name);
            }
            else
            {
                ENETTRACE_ERR("%s: Failed to open: %d\n", hNullPer->mod1.name, status);
                hNullPer->mod1.magic = ENET_NO_MAGIC;
            }
        }
        else
        {
            ENETTRACE_ERR("%s: Module is already open\n", hNullPer->mod1.name);
            status = ENET_EALREADYOPEN;
        }
        ENETTRACE_ERR_IF(status != ENET_SOK, "%s: Failed to open mod1: %d\n", hNullPer->mod1.name, status);

        /* Open module 2 */
        if (status == ENET_SOK)
        {
            ENETTRACE_INFO("%s: Opening mod1\n", hNullPer->mod2.name);

            bool isMod2Open = (hNullPer->mod2.magic == ENET_MAGIC) ? true : false;

            if (isMod2Open == false)
            {
                hNullPer->mod2.virtAddr  = (void *)EnetUtils_physToVirt(hNullPer->mod2.physAddr, NULL);
                hNullPer->mod2.virtAddr2 = (void *)EnetUtils_physToVirt(hNullPer->mod2.physAddr2, NULL);

                status = NullMod_open(&hNullPer->mod2, enetType, instId, &nullPerCfg->mod2Cfg);
                if (status == ENET_SOK)
                {
                    hNullPer->mod2.magic = ENET_MAGIC;
                    ENETTRACE_VERBOSE("%s: Module is now open\n", hNullPer->mod2.name);
                }
                else
                {
                    ENETTRACE_ERR("%s: Failed to open: %d\n", hNullPer->mod2.name, status);
                    hNullPer->mod2.magic = ENET_NO_MAGIC;
                }
            }
            else
            {
                ENETTRACE_ERR("%s: Module is already open\n", hNullPer->mod2.name);
                status = ENET_EALREADYOPEN;
            }
            ENETTRACE_ERR_IF(status != ENET_SOK, "%s: Failed to open mod1: %d\n", hNullPer->mod2.name, status);
        }
        if (status == ENET_SOK)
        {
            hNullPer->magic = ENET_MAGIC;
            ENETTRACE_VERBOSE("%s: Peripheral is now open\n", hNullPer->name);
        }
        else
        {
            ENETTRACE_ERR("%s: Failed to open: %d\n", hNullPer->name, status);
            hNullPer->magic = ENET_NO_MAGIC;
        }

        /* Set driver open state */
        hNullPer->magic = (status == ENET_SOK) ? ENET_MAGIC : ENET_NO_MAGIC;

        EnetOsal_unlockMutex(hNullPer->lock);
    }
    else
    {
        ENETTRACE_ERR("%s: Peripheral is already open\n", hNullPer->name);
        status = ENET_EALREADYOPEN;
    }

    return status;
}

int32_t NullPer_rejoin(uint32_t hEnet,
                       Enet_Type enetType,
                       uint32_t instId)
{
    int32_t status = ENET_SOK;

    NullPer_Handle hNullPer = NULL;

    if (hEnet != -1)
    {
        hNullPer = NullPer_getHandle(hEnet);
        Enet_devAssert(hNullPer != NULL,
                       "Invalid EnetPer handle for %u.%u\n", enetType, instId);
    }
    else
    {
        ENETTRACE_ERR("No EnetPer %u:%u has been found\n", enetType, instId);
        status = ENET_ENOTFOUND;
        hNullPer = NULL;
    }

    if (status == ENET_SOK)
    {
        EnetOsal_lockMutex(hNullPer->lock);

#if ENET_CFG_IS_ON(SANITY_CHECKS)
        /* Print enabled configurable features and applicable erratas */
        ENETTRACE_DBG("%s: features: 0x%08x\n", hNullPer->name, hNullPer->features);
        ENETTRACE_DBG("%s: errata  : 0x%08x\n", hNullPer->name, hNullPer->errata);
#endif
        status = ENET_ENOTSUPPORTED;

        ENETTRACE_VERBOSE("%s: Rejoin peripheral\n", hNullPer->name);

        if (hNullPer->magic == ENET_NO_MAGIC)
        {
            hNullPer->virtAddr  = EnetUtils_physToVirt(hNullPer->physAddr, NULL);
            hNullPer->virtAddr2 = EnetUtils_physToVirt(hNullPer->physAddr2, NULL);
            status = ENET_SOK;

            ENETTRACE_INFO("%s: Rejoin null peripheral\n", hNullPer->name);

            /* Rejoin module 1 */
            ENETTRACE_INFO("%s: Rejoining mod1\n", hNullPer->name);
            status = NullMod_rejoin(&hNullPer->mod1, enetType, instId);
            ENETTRACE_ERR_IF(status != ENET_SOK, "%s: Failed to rejoin mod1: %d\n", hNullPer->name, status);

            /* Rejoin module 2 */
            if (status == ENET_SOK)
            {
                ENETTRACE_INFO("%s: Rejoining mod2\n", hNullPer->name);
                status = NullMod_rejoin(&hNullPer->mod2, enetType, instId);
                ENETTRACE_ERR_IF(status != ENET_SOK, "%s: Failed to rejoin mod2: %d\n", hNullPer->name, status);
            }
            if (status == ENET_SOK)
            {
                hNullPer->magic = ENET_MAGIC;
                ENETTRACE_VERBOSE("%s: Peripheral has now joined\n", hNullPer->name);
            }
            else
            {
                ENETTRACE_ERR("%s: Failed to open: %d\n", hNullPer->name, status);
                hNullPer->magic = ENET_NO_MAGIC;
            }
        }
        else
        {
            ENETTRACE_ERR("%s: Peripheral is already open\n", hNullPer->name);
            status = ENET_EALREADYOPEN;
        }
        ENETTRACE_ERR_IF(status != ENET_SOK,
            "%s: Failed to join: %d\n", hNullPer->name, status);

        /* Set driver open state */
        hNullPer->magic = (status == ENET_SOK) ? ENET_MAGIC : ENET_NO_MAGIC;

        EnetOsal_unlockMutex(hNullPer->lock);
    }
    return status;
}

int32_t NullPer_ioctl(uint32_t hEnet,
                      uint32_t cmd,
                      Enet_IoctlPrms *prms)
{
    NullPer_Handle hNullPer = NULL;
    int32_t status = ENET_SOK;

    /* Call IOCTL on the Enet peripheral */
    if (hEnet != -1)
    {
        hNullPer = NullPer_getHandle(hEnet);
        Enet_devAssert(hNullPer != NULL, "Invalid EnetPer handle\n");

        EnetOsal_lockMutex(hNullPer->lock);

#if ENET_CFG_IS_ON(DEV_ERROR)
        status = Enet_validateGenericIoctl(cmd, prms);
        ENETTRACE_ERR_IF(status != ENET_SOK, "IOCTL params are not valid\n");
#endif

        if (status == ENET_SOK)
        {
            status = ENET_EFAIL;

            ENETTRACE_VERBOSE("%s: Do IOCTL 0x%08x prms %p\n", hNullPer->name, cmd, prms);

            if (hNullPer->magic == ENET_MAGIC)
            {
                status = ENET_SOK;

                ENETTRACE_INFO("%s: IOCTL %u on null peripheral\n", hNullPer->name, cmd);

                /* Do IOCTL on module 1 */
                ENETTRACE_INFO("%s: IOCTL %u on mod1\n", hNullPer->name, cmd);
                status = NullMod_ioctl(&hNullPer->mod1, cmd, prms);
                ENETTRACE_ERR_IF(status != ENET_SOK, "%s: Failed to do IOCTL %u on mod1: %d\n", hNullPer->name, cmd, status);

                /* Do IOCTL on module 2 */
                if (status == ENET_SOK)
                {
                    ENETTRACE_INFO("%s: IOCTL %u on mod2\n", hNullPer->name, cmd);
                    status = NullMod_ioctl(&hNullPer->mod2, cmd, prms);
                    ENETTRACE_ERR_IF(status != ENET_SOK, "%s: Failed to do IOCTL %u on mod2: %d\n", hNullPer->name, cmd, status);
                }
                if (status < ENET_SOK)
                {
                    ENETTRACE_ERR("%s: Failed to do IOCTL cmd 0x%08x: %d\n", hNullPer->name, cmd, status);
                }
            }
            else
            {
                ENETTRACE_ERR("%s: Peripheral is not open\n", hNullPer->name);
            }
            ENETTRACE_ERR_IF(status < ENET_SOK,
                "%s: IOCTL 0x%08x failed: %d\n", hNullPer->name, cmd, status);
        }
        EnetOsal_unlockMutex(hNullPer->lock);
    }   
    else
    {
        ENETTRACE_ERR("Invalid Enet handle\n");
        status = ENET_EBADARGS;
    }
    return status;
}

void NullPer_poll(uint32_t hEnet,
                  uint32_t evtMask)
{
    NullPer_Handle hNullPer = NULL;

    /* Poll the Enet peripheral for requested events */
    if (hEnet != -1)
    {
        hNullPer = NullPer_getHandle(hEnet);
        Enet_devAssert(hNullPer != NULL, "Invalid EnetPer handle\n");

        EnetOsal_lockMutex(hNullPer->lock);

        ENETTRACE_VERBOSE("%s: Poll peripheral for event %u\n", hNullPer->name, evt);

        if (hNullPer->magic == ENET_MAGIC)
        {
            ENETTRACE_INFO("%s: events 0x%08x\n", hNullPer->name, evtMask);
        }
        else
        {
            ENETTRACE_ERR("%s: Peripheral is not open\n", hNullPer->name);
        }

        EnetOsal_unlockMutex(hNullPer->lock);
    }
    else
    {
        ENETTRACE_ERR("Invalid Enet handle\n");
    }
    
}

void NullPer_periodicTick(uint32_t hEnet)
{
    NullPer_Handle hNullPer = NULL;

    /* Run the periodic tick */
    if (hEnet != -1)
    {
        hNullPer = NullPer_getHandle(hEnet);
        Enet_devAssert(hNullPer != NULL, "Invalid EnetPer handle\n");

        /* TODO: Need to make lock more granular */
        EnetOsal_lockMutex(hNullPer->lock);

        ENETTRACE_VERBOSE("%s: Do periodic tick\n", hNullPer->name);

        if (hNullPer->magic == ENET_MAGIC)
        {
            ENETTRACE_INFO("%s: tick!\n", hNullPer->name);
        }
        else
        {
            ENETTRACE_ERR("%s: Peripheral is not open\n", hNullPer->name);
        }

        /* TODO: Need to make lock more granular */
        EnetOsal_unlockMutex(hNullPer->lock);
    }
    else
    {
        ENETTRACE_ERR("Periodic tick called on an invalid Enet\n");
    }
    
}

void NullPer_close(uint32_t hEnet)
{
    NullPer_Handle hNullPer = NULL;

    /* Close the Enet peripheral */
    if (hEnet != -1)
    {
        hNullPer = NullPer_getHandle(hEnet);
        Enet_devAssert(hNullPer != NULL, "Invalid EnetPer handle\n");

        EnetOsal_lockMutex(hNullPer->lock);

        ENETTRACE_VERBOSE("%s: Close peripheral\n", hNullPer->name);

        if (hNullPer->magic == ENET_MAGIC)
        {
            ENETTRACE_INFO("%s: Close null peripheral\n", hNullPer->name);

            /* Close module 1 */
            ENETTRACE_INFO("%s: Closing mod1\n", hNullPer->name);
            NullMod_close(&hNullPer->mod1);

            /* Close module 2 */
            ENETTRACE_INFO("%s: Closing mod2\n", hNullPer->name);
            NullMod_close(&hNullPer->mod2);

            hNullPer->magic = ENET_NO_MAGIC;
            ENETTRACE_VERBOSE("%s: Peripheral is now closed\n", hNullPer->name);
        }

        /* Set driver open state */
        hNullPer->magic = ENET_NO_MAGIC;

        EnetOsal_unlockMutex(hNullPer->lock);
    }
    else
    {
        ENETTRACE_ERR("Trying to close an invalid Enet handle, ignoring...\n");
    }
}
