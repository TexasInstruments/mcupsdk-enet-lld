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
 * \file  mdio.c
 *
 * \brief This file contains the implementation of the MDIO module.
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
#include <include/mod/mdio.h>
#include <priv/core/enet_trace_priv.h>
#include <priv/mod/mdio_priv.h>
#include <priv/mod/mdio_ioctl_priv.h>
#include <priv/mod/cpsw_clks.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
static void Mdio_printRegs(CSL_mdioHandle mdioRegs);
#if ENET_CFG_IS_ON(MDIO_CLAUSE45)
static int32_t Mdio_readRegC45(CSL_mdioHandle mdioRegs,
                               uint32_t userCh,
                               uint32_t mmd,
                               uint32_t phyAddr,
                               uint16_t reg,
                               uint16_t *val);

static int32_t Mdio_writeRegC45(CSL_mdioHandle mdioRegs,
                                uint32_t userCh,
                                uint32_t mmd,
                                uint32_t phyAddr,
                                uint16_t reg,
                                uint16_t val);

static int32_t Mdio_readRegTriggerC45(CSL_mdioHandle mdioRegs,
                               uint32_t userCh,
                               uint32_t mmd,
                               uint32_t phyAddr,
                               uint16_t reg);

static int32_t Mdio_readRegCompleteC45(CSL_mdioHandle mdioRegs,
                               uint32_t userCh,
                               uint32_t mmd,
                               uint32_t phyAddr,
                               uint16_t reg,
                               uint16_t *val);

static int32_t Mdio_writeRegTriggerC45(CSL_mdioHandle mdioRegs,
                                uint32_t userCh,
                                uint32_t mmd,
                                uint32_t phyAddr,
                                uint16_t reg,
                                uint16_t val);

static int32_t Mdio_writeRegCompleteC45(CSL_mdioHandle mdioRegs,
                                uint32_t userCh,
                                uint32_t mmd,
                                uint32_t phyAddr,
                                uint16_t reg,
                                uint16_t val);

#endif

static void Mdio_handleIntr(CSL_mdioHandle mdioRegs,
                            Mdio_Callbacks *callbacks);




/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */


int32_t Mdio_ioctl_handler_ENET_MDIO_IOCTL_GET_VERSION(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    Enet_Version *version = (Enet_Version *)prms->outArgs;
    CSL_MDIO_VERSION ver = {0};
    int32_t status = ENET_SOK;

    CSL_MDIO_getVersionInfo(mdioRegs, &ver);
    version->maj    = ver.revMaj;
    version->min    = ver.revMin;
    version->rtl    = ver.revRtl;
    version->id     = ver.modId;
    version->other1 = ver.scheme;
    version->other2 = ver.bu;
    return status;
}

int32_t Mdio_ioctl_handler_ENET_MDIO_IOCTL_PRINT_REGS(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

    Mdio_printRegs(mdioRegs);
    return status;
}

int32_t Mdio_normal_ioctl_handler_ENET_MDIO_IOCTL_IS_ALIVE(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    uint32_t *phyAddr = (uint32_t *)prms->inArgs;
    bool *alive = (bool *)prms->outArgs;
    int32_t status = ENET_SOK;

    *alive = (CSL_MDIO_isPhyAlive(mdioRegs, *phyAddr) == 0U) ? false : true;
    return status;
}

int32_t Mdio_normal_ioctl_handler_ENET_MDIO_IOCTL_IS_LINKED(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    uint32_t *phyAddr = (uint32_t *)prms->inArgs;
    bool *linked = (bool *)prms->outArgs;
    int32_t status = ENET_SOK;

    *linked = (CSL_MDIO_isPhyLinked(mdioRegs, *phyAddr) == 0) ? false : true;
    return status;
}

int32_t Mdio_normal_ioctl_handler_ENET_MDIO_IOCTL_IS_POLL_ENABLED(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    uint32_t *phyAddr = (uint32_t *)prms->inArgs;
    bool *enabled = (bool *)prms->outArgs;
    uint8_t monPhyAddr;
    uint32_t isStatusChangeMode;
    uint32_t pollMask = 0U;
    uint32_t i;
    int32_t status = ENET_SOK;

    isStatusChangeMode = CSL_MDIO_isStateChangeModeEnabled(mdioRegs);
    if (isStatusChangeMode == 1U)
    {
        pollMask = CSL_MDIO_getPollEnableMask(mdioRegs);
    }
    else
    {
        for (i = 0U; i < MDIO_PHY_MONITOR_MAX; i++)
        {
            monPhyAddr = CSL_MDIO_getLinkStatusChangePhyAddr(mdioRegs, i);
            pollMask |= ENET_MDIO_PHY_ADDR_MASK(monPhyAddr);
        }
    }

    *enabled = ENET_IS_BIT_SET(pollMask, *phyAddr);
    return status;
}

int32_t Mdio_normal_ioctl_handler_ENET_MDIO_IOCTL_C22_READ(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    EnetMdio_C22ReadInArgs *inArgs = (EnetMdio_C22ReadInArgs *)prms->inArgs;
    uint16_t *val = (uint16_t *)prms->outArgs;
    uint32_t ack;
    int32_t status = ENET_SOK;
#if ENET_CFG_IS_ON(MDIO_CLAUSE45)
    uint32_t c45EnMask;

    if (ENET_FEAT_IS_EN(hMod->features, MDIO_FEATURE_CLAUSE45))
    {
        c45EnMask = CSL_MDIO_getClause45EnableMask(mdioRegs);
        if (ENET_IS_BIT_SET(c45EnMask, inArgs->phyAddr))
        {
            ENETTRACE_ERR("PHY %u is not configured for C22 access\n", inArgs->phyAddr);
            status = ENET_EPERM;
        }
    }
#endif

    if (status == ENET_SOK)
    {
        ack = CSL_MDIO_phyRegRead2(mdioRegs,
                                   inArgs->group,
                                   inArgs->phyAddr,
                                   inArgs->reg,
                                   val);
        status = (ack == TRUE) ? ENET_SOK : ENET_EFAIL;
        ENETTRACE_ERR_IF(status != ENET_SOK,
                         "failed to read PHY %u C22 reg %u: %d\n",
                         inArgs->phyAddr, inArgs->reg, status);
    }
    return status;
}

int32_t Mdio_normal_ioctl_handler_ENET_MDIO_IOCTL_C22_WRITE(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    EnetMdio_C22WriteInArgs *inArgs = (EnetMdio_C22WriteInArgs *)prms->inArgs;
    int32_t status = ENET_SOK;
#if ENET_CFG_IS_ON(MDIO_CLAUSE45)
    uint32_t c45EnMask;

    if (ENET_FEAT_IS_EN(hMod->features, MDIO_FEATURE_CLAUSE45))
    {
        c45EnMask = CSL_MDIO_getClause45EnableMask(mdioRegs);
        if (ENET_IS_BIT_SET(c45EnMask, inArgs->phyAddr))
        {
            ENETTRACE_ERR("PHY %u is not configured for C22 access\n", inArgs->phyAddr);
            status = ENET_EPERM;
        }
    }
#endif

    if (status == ENET_SOK)
    {
        CSL_MDIO_phyRegWrite2(mdioRegs,
                              inArgs->group,
                              inArgs->phyAddr,
                              inArgs->reg,
                              inArgs->val);
    }
    return status;
}

int32_t Mdio_normal_ioctl_handler_ENET_MDIO_IOCTL_C45_READ(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;
#if ENET_CFG_IS_ON(MDIO_CLAUSE45)
    EnetMdio_C45ReadInArgs *inArgs = (EnetMdio_C45ReadInArgs *)prms->inArgs;
    uint16_t *val = (uint16_t *)prms->outArgs;

    if (ENET_FEAT_IS_EN(hMod->features, MDIO_FEATURE_CLAUSE45))
    {
        status = Mdio_readRegC45(mdioRegs,
                                 (uint32_t)inArgs->group,
                                 (uint32_t)inArgs->mmd,
                                 (uint32_t)inArgs->phyAddr,
                                 inArgs->reg,
                                 val);
        ENETTRACE_ERR_IF(status != ENET_SOK,
                         "failed to read PHY %u C45 MMD %u reg %u: %d\n",
                         inArgs->phyAddr, inArgs->mmd, inArgs->reg, status);
    }
    else
    {
        ENETTRACE_ERR("C45 support is not supported\n");
        status = ENET_ENOTSUPPORTED;
    }
#else
    ENETTRACE_ERR("C45 support is not enabled\n");
    status = ENET_ENOTSUPPORTED;
#endif
    return status;
}

int32_t Mdio_normal_ioctl_handler_ENET_MDIO_IOCTL_C45_WRITE(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;
#if ENET_CFG_IS_ON(MDIO_CLAUSE45)
    EnetMdio_C45WriteInArgs *inArgs = (EnetMdio_C45WriteInArgs *)prms->inArgs;

    if (ENET_FEAT_IS_EN(hMod->features, MDIO_FEATURE_CLAUSE45))
    {
        status = Mdio_writeRegC45(mdioRegs,
                                  (uint32_t)inArgs->group,
                                  (uint32_t)inArgs->mmd,
                                  (uint32_t)inArgs->phyAddr,
                                  inArgs->reg,
                                  inArgs->val);
        ENETTRACE_ERR_IF(status != ENET_SOK,
                         "failed to write PHY %u C45 MMD %u reg %u: %d\n",
                         inArgs->phyAddr, inArgs->mmd, inArgs->reg, status);
    }
    else
    {
        ENETTRACE_ERR("C45 support is not supported\n");
        status = ENET_ENOTSUPPORTED;
    }
#else
    ENETTRACE_ERR("C45 support is not enabled\n");
    status = ENET_ENOTSUPPORTED;
#endif
    return status;
}

int32_t Mdio_normal_ioctl_handler_ENET_MDIO_IOCTL_C22_ASYNC_READ_TRIGGER(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    EnetMdio_C22ReadInArgs *inArgs = (EnetMdio_C22ReadInArgs *)prms->inArgs;
    uint32_t ack;
    int32_t status = ENET_SOK;
#if ENET_CFG_IS_ON(MDIO_CLAUSE45)
    uint32_t c45EnMask;

    if (ENET_FEAT_IS_EN(hMod->features, MDIO_FEATURE_CLAUSE45))
    {
        c45EnMask = CSL_MDIO_getClause45EnableMask(mdioRegs);
        if (ENET_IS_BIT_SET(c45EnMask, inArgs->phyAddr))
        {
            ENETTRACE_ERR("PHY %u is not configured for C22 access\n", inArgs->phyAddr);
            status = ENET_EPERM;
        }
    }
#endif

    if (status == ENET_SOK)
    {
        ack = CSL_MDIO_phyRegReadAsyncTrigger(mdioRegs,
                                   inArgs->group,
                                   inArgs->phyAddr,
                                   inArgs->reg);
        status = (ack == CSL_PASS) ? ENET_SOK : ENET_EFAIL;
        ENETTRACE_ERR_IF(status != ENET_SOK,
                         "failed to read PHY %u C22 reg %u: %d\n",
                         inArgs->phyAddr, inArgs->reg, status);
    }
    return status;
}

int32_t Mdio_normal_ioctl_handler_ENET_MDIO_IOCTL_C22_ASYNC_READ_COMPLETE(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    EnetMdio_C22ReadInArgs *inArgs = (EnetMdio_C22ReadInArgs *)prms->inArgs;
    uint16_t *val = (uint16_t *)prms->outArgs;
    uint32_t ack;
    int32_t status = ENET_SOK;
#if ENET_CFG_IS_ON(MDIO_CLAUSE45)
    uint32_t c45EnMask;

    if (ENET_FEAT_IS_EN(hMod->features, MDIO_FEATURE_CLAUSE45))
    {
        c45EnMask = CSL_MDIO_getClause45EnableMask(mdioRegs);
        if (ENET_IS_BIT_SET(c45EnMask, inArgs->phyAddr))
        {
            ENETTRACE_ERR("PHY %u is not configured for C22 access\n", inArgs->phyAddr);
            status = ENET_EPERM;
        }
    }
#endif

    if (status == ENET_SOK)
    {

        ack = CSL_MDIO_phyRegReadAsyncComplete(mdioRegs,
                                   inArgs->group,
                                   inArgs->phyAddr,
                                   inArgs->reg,
                                   val);
        if  (ack == CSL_ETIMEOUT)
        {
            status = ENET_SINPROGRESS;
        }
        else
        {
            status = (ack == CSL_PASS) ? ENET_SOK : ENET_EFAIL;
        }
        ENETTRACE_ERR_IF(status != ENET_SOK,
                         "failed to read PHY %u C22 reg %u: %d\n",
                         inArgs->phyAddr, inArgs->reg, status);
    }
    return status;
}

int32_t Mdio_normal_ioctl_handler_ENET_MDIO_IOCTL_C22_ASYNC_WRITE_TRIGGER(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    EnetMdio_C22WriteInArgs *inArgs = (EnetMdio_C22WriteInArgs *)prms->inArgs;
    uint32_t ack;
    int32_t status = ENET_SOK;
#if ENET_CFG_IS_ON(MDIO_CLAUSE45)
    uint32_t c45EnMask;

    if (ENET_FEAT_IS_EN(hMod->features, MDIO_FEATURE_CLAUSE45))
    {
        c45EnMask = CSL_MDIO_getClause45EnableMask(mdioRegs);
        if (ENET_IS_BIT_SET(c45EnMask, inArgs->phyAddr))
        {
            ENETTRACE_ERR("PHY %u is not configured for C22 access\n", inArgs->phyAddr);
            status = ENET_EPERM;
        }
    }
#endif

    if (status == ENET_SOK)
    {
        ack = CSL_MDIO_phyRegWriteAsyncTrigger(mdioRegs,
                                   inArgs->group,
                                   inArgs->phyAddr,
                                   inArgs->reg,
                                   inArgs->val);
        status = (ack == CSL_PASS) ? ENET_SOK : ENET_EFAIL;
        ENETTRACE_ERR_IF(status != ENET_SOK,
                         "failed to read PHY %u C22 reg %u: %d\n",
                         inArgs->phyAddr, inArgs->reg, status);
    }
    return status;
}

int32_t Mdio_normal_ioctl_handler_ENET_MDIO_IOCTL_C22_ASYNC_WRITE_COMPLETE(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    EnetMdio_C22WriteInArgs *inArgs = (EnetMdio_C22WriteInArgs *)prms->inArgs;
    uint32_t ack;
    int32_t status = ENET_SOK;
#if ENET_CFG_IS_ON(MDIO_CLAUSE45)
    uint32_t c45EnMask;

    if (ENET_FEAT_IS_EN(hMod->features, MDIO_FEATURE_CLAUSE45))
    {
        c45EnMask = CSL_MDIO_getClause45EnableMask(mdioRegs);
        if (ENET_IS_BIT_SET(c45EnMask, inArgs->phyAddr))
        {
            ENETTRACE_ERR("PHY %u is not configured for C22 access\n", inArgs->phyAddr);
            status = ENET_EPERM;
        }
    }
#endif

    if (status == ENET_SOK)
    {
        ack = CSL_MDIO_phyRegWriteAsyncComplete(mdioRegs,
                                   inArgs->group,
                                   inArgs->phyAddr,
                                   inArgs->reg,
                                   inArgs->val);
        if  (ack == CSL_ETIMEOUT)
        {
            status = ENET_SINPROGRESS;
        }
        else
        {
            status = (ack == CSL_PASS) ? ENET_SOK : ENET_EFAIL;
        }
        ENETTRACE_ERR_IF(status != ENET_SOK,
                         "failed to read PHY %u C22 reg %u: %d\n",
                         inArgs->phyAddr, inArgs->reg, status);
    }
    return status;
}

int32_t Mdio_normal_ioctl_handler_ENET_MDIO_IOCTL_C45_ASYNC_READ_TRIGGER(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;
#if ENET_CFG_IS_ON(MDIO_CLAUSE45)
    EnetMdio_C45ReadInArgs *inArgs = (EnetMdio_C45ReadInArgs *)prms->inArgs;
    uint32_t ack;

    if (ENET_FEAT_IS_EN(hMod->features, MDIO_FEATURE_CLAUSE45))
    {
        if (status == ENET_SOK)
        {
            ack = Mdio_readRegTriggerC45(mdioRegs,
                                    (uint32_t)inArgs->group,
                                    (uint32_t)inArgs->mmd,
                                    inArgs->phyAddr,
                                    inArgs->reg);
            status = (ack == CSL_PASS) ? ENET_SOK : ENET_EFAIL;
            ENETTRACE_ERR_IF(status != ENET_SOK,
                            "failed to read PHY %u C45 MMD %u reg %u: %d\n",
                            inArgs->phyAddr, inArgs->mmd, inArgs->reg, status);
        }
    }
    else
    {
        ENETTRACE_ERR("C45 support is not supported\n");
        status = ENET_ENOTSUPPORTED;
    }
#else
    ENETTRACE_ERR("C45 support is not enabled\n");
    status = ENET_ENOTSUPPORTED;
#endif
    return status;
}

int32_t Mdio_normal_ioctl_handler_ENET_MDIO_IOCTL_C45_ASYNC_READ_COMPLETE(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;
#if ENET_CFG_IS_ON(MDIO_CLAUSE45)
    EnetMdio_C45ReadInArgs *inArgs = (EnetMdio_C45ReadInArgs *)prms->inArgs;
    uint16_t *val = (uint16_t *)prms->outArgs;
    uint32_t ack;

    if (ENET_FEAT_IS_EN(hMod->features, MDIO_FEATURE_CLAUSE45))
    {
        ack = Mdio_readRegCompleteC45(mdioRegs,
                                 (uint32_t)inArgs->group,
                                 (uint32_t)inArgs->mmd,
                                 inArgs->phyAddr,
                                 inArgs->reg,
                                 val);
        if  (ack == CSL_ETIMEOUT)
        {
            status = ENET_SINPROGRESS;
        }
        else
        {
            status = (ack == CSL_PASS) ? ENET_SOK : ENET_EFAIL;
        }
        ENETTRACE_ERR_IF(status != ENET_SOK,
                         "failed to read PHY %u C45 MMD %u reg %u: %d\n",
                         inArgs->phyAddr, inArgs->mmd, inArgs->reg, status);
    }
    else
    {
        ENETTRACE_ERR("C45 support is not supported\n");
        status = ENET_ENOTSUPPORTED;
    }
#else
    ENETTRACE_ERR("C45 support is not enabled\n");
    status = ENET_ENOTSUPPORTED;
#endif
    return status;
}

int32_t Mdio_normal_ioctl_handler_ENET_MDIO_IOCTL_C45_ASYNC_WRITE_TRIGGER(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;
#if ENET_CFG_IS_ON(MDIO_CLAUSE45)
    EnetMdio_C45WriteInArgs *inArgs = (EnetMdio_C45WriteInArgs *)prms->inArgs;
    uint32_t ack;

    if (ENET_FEAT_IS_EN(hMod->features, MDIO_FEATURE_CLAUSE45))
    {
        ack = Mdio_writeRegTriggerC45(mdioRegs,
                                  (uint32_t)inArgs->group,
                                  (uint32_t)inArgs->mmd,
                                  inArgs->phyAddr,
                                  inArgs->reg,
                                  inArgs->val);
        status = (ack == CSL_PASS) ? ENET_SOK : ENET_EFAIL;
        ENETTRACE_ERR_IF(status != ENET_SOK,
                         "failed to write PHY %u C45 MMD %u reg %u: %d\n",
                         inArgs->phyAddr, inArgs->mmd, inArgs->reg, status);
    }
    else
    {
        ENETTRACE_ERR("C45 support is not supported\n");
        status = ENET_ENOTSUPPORTED;
    }
#else
    ENETTRACE_ERR("C45 support is not enabled\n");
    status = ENET_ENOTSUPPORTED;
#endif
    return status;
}

int32_t Mdio_normal_ioctl_handler_ENET_MDIO_IOCTL_C45_ASYNC_WRITE_COMPLETE(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;
#if ENET_CFG_IS_ON(MDIO_CLAUSE45)
    EnetMdio_C45WriteInArgs *inArgs = (EnetMdio_C45WriteInArgs *)prms->inArgs;
    uint32_t ack;

    if (ENET_FEAT_IS_EN(hMod->features, MDIO_FEATURE_CLAUSE45))
    {
        ack = Mdio_writeRegCompleteC45(mdioRegs,
                                  (uint32_t)inArgs->group,
                                  (uint32_t)inArgs->mmd,
                                  inArgs->phyAddr,
                                  inArgs->reg,
                                  inArgs->val);
        if  (ack == CSL_ETIMEOUT)
        {
            status = ENET_SINPROGRESS;
        }
        else
        {
            status = (ack == CSL_PASS) ? ENET_SOK : ENET_EFAIL;
        }
        ENETTRACE_ERR_IF(status != ENET_SOK,
                         "failed to write PHY %u C45 MMD %u reg %u: %d\n",
                         inArgs->phyAddr, inArgs->mmd, inArgs->reg, status);
    }
    else
    {
        ENETTRACE_ERR("C45 support is not supported\n");
        status = ENET_ENOTSUPPORTED;
    }
#else
    ENETTRACE_ERR("C45 support is not enabled\n");
    status = ENET_ENOTSUPPORTED;
#endif
    return status;
}

int32_t Mdio_ioctl_handler_MDIO_IOCTL_HANDLE_INTR(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    Mdio_Callbacks *callbacks = (Mdio_Callbacks *)prms->inArgs;
    int32_t status = ENET_SOK;

    Mdio_handleIntr(mdioRegs, callbacks);
    return status;
}

int32_t Mdio_ioctl_handler_ENET_MDIO_IOCTL_ENABLE_STATE_MACHINE(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

    CSL_MDIO_enableStateMachine(mdioRegs);
    return status;
}

static void Mdio_printRegs(CSL_mdioHandle mdioRegs)
{
    uint32_t *regAddr = (uint32_t *)mdioRegs;
    uint32_t regIdx = 0U;

    while ((uintptr_t)regAddr < ((uintptr_t)mdioRegs + sizeof(*mdioRegs)))
    {
        if (*regAddr != 0U)
        {
            ENETTRACE_INFO("MDIO: %u: 0x%08x\n", regIdx, *regAddr);
        }

        regAddr++;
        regIdx++;
    }
}

#if ENET_CFG_IS_ON(MDIO_CLAUSE45)
static int32_t Mdio_readRegC45(CSL_mdioHandle mdioRegs,
                               uint32_t userCh,
                               uint32_t mmd,
                               uint32_t phyAddr,
                               uint16_t reg,
                               uint16_t *val)
{
    bool isComplete;
    uint32_t c45EnMask;
    int32_t status;

    c45EnMask = CSL_MDIO_getClause45EnableMask(mdioRegs);
    if (ENET_IS_BIT_SET(c45EnMask, phyAddr))
    {

        /* Wait for any ongoing transaction to complete */
        do
        {
            isComplete = CSL_MDIO_isPhyRegAccessComplete(mdioRegs, userCh);
        }
        while (isComplete == FALSE);

        /* Initiate register read */
        status = CSL_MDIO_phyInitiateRegReadC45(mdioRegs, userCh, phyAddr, mmd, reg);
        ENETTRACE_ERR_IF(status != CSL_PASS,
                         "failed to initiate PHY %u C45 MMD %u register %u read: %d\n",
                         phyAddr, mmd, reg, status);

        /* Wait for register read transaction to complete */
        if (status == CSL_PASS)
        {
            do
            {
                isComplete = CSL_MDIO_isPhyRegAccessComplete(mdioRegs, userCh);
            }
            while (isComplete == FALSE);
        }

        /* Get the value read from PHY register once transaction is complete */
        if (status == CSL_PASS)
        {
            status = CSL_MDIO_phyGetRegReadVal(mdioRegs, userCh, val);
            ENETTRACE_ERR_IF(status == CSL_ETIMEOUT,
                             "C45 register read %u was not acknowledged by PHY %u: %d\n",
                             reg, phyAddr, status);
            ENETTRACE_ERR_IF(status == CSL_EFAIL,
                             "failed to read PHY %u C45 MMD %u register %u: %d\n",
                             phyAddr, mmd, reg, status);
        }
    }
    else
    {
        ENETTRACE_ERR("PHY %u is not configured for C45 access\n", phyAddr);
        status = ENET_EPERM;
    }

    return status;
}

static int32_t Mdio_readRegTriggerC45(CSL_mdioHandle mdioRegs,
                               uint32_t userCh,
                               uint32_t mmd,
                               uint32_t phyAddr,
                               uint16_t reg)
{
    uint32_t c45EnMask;
    int32_t status = CSL_EFAIL;
    bool accessRegBusy = true;

    c45EnMask = CSL_MDIO_getClause45EnableMask(mdioRegs);
    if (ENET_IS_BIT_SET(c45EnMask, phyAddr))
    {

        /* Wait for any ongoing transaction to complete */
        if(CSL_MDIO_isPhyRegAccessComplete(mdioRegs, userCh))
        {
            accessRegBusy = false;
        }

        if (accessRegBusy == false)
        {
            /* Initiate register read */
            status = CSL_MDIO_phyInitiateRegReadC45(mdioRegs, userCh, phyAddr, mmd, reg);
        }
    }
    else
    {
        ENETTRACE_ERR("PHY %u is not configured for C45 access\n", phyAddr);
        status = ENET_EPERM;
    }
    return status;
}

static int32_t Mdio_readRegCompleteC45(CSL_mdioHandle mdioRegs,
                               uint32_t userCh,
                               uint32_t mmd,
                               uint32_t phyAddr,
                               uint16_t reg,
                               uint16_t *val)
{
    uint32_t c45EnMask;
    int32_t status = CSL_EFAIL;
    bool accessRegBusy = true;

    c45EnMask = CSL_MDIO_getClause45EnableMask(mdioRegs);
    if (ENET_IS_BIT_SET(c45EnMask, phyAddr))
    {
    /* Wait for register read transaction to complete */
        if (CSL_MDIO_isPhyRegAccessComplete(mdioRegs, userCh))
        {
            accessRegBusy = false;
        }

        if(accessRegBusy == false)
        {
            uint32_t regPhyAddr, regPhyRegNum;
            regPhyAddr = CSL_FEXT(mdioRegs->USER_GROUP[userCh].USER_ACCESS_REG, MDIO_USER_GROUP_USER_ACCESS_REG_PHYADR);
            regPhyRegNum = CSL_FEXT(mdioRegs->USER_GROUP[userCh].USER_ACCESS_REG, MDIO_USER_GROUP_USER_ACCESS_REG_REGADR);
            if ((regPhyAddr == phyAddr)
                &&
                (regPhyRegNum == reg))
            {
                status = CSL_MDIO_phyGetRegReadVal(mdioRegs, userCh, val);
            }
            if(status != CSL_PASS)
            {
                ENETTRACE_ERR_IF(status == CSL_ETIMEOUT,
                                    "C45 register read %u was not acknowledged by PHY %u: %d\n",
                                    reg, phyAddr, status);
                ENETTRACE_ERR_IF(status == CSL_EFAIL,
                                    "failed to read PHY %u C45 MMD %u register %u: %d\n",
                                    phyAddr, mmd, reg, status);
            }
        }
    }
    else
    {
        ENETTRACE_ERR("PHY %u is not configured for C45 access\n", phyAddr);
        status = ENET_EPERM;
    }

    return status;
}

static int32_t Mdio_writeRegC45(CSL_mdioHandle mdioRegs,
                                uint32_t userCh,
                                uint32_t mmd,
                                uint32_t phyAddr,
                                uint16_t reg,
                                uint16_t val)
{
    bool isComplete;
    uint32_t c45EnMask;
    int32_t status;

    c45EnMask = CSL_MDIO_getClause45EnableMask(mdioRegs);
    if (ENET_IS_BIT_SET(c45EnMask, phyAddr))
    {
        /* Wait for any ongoing transaction to complete */
        do
        {
            isComplete = CSL_MDIO_isPhyRegAccessComplete(mdioRegs, userCh);
        }
        while (isComplete == FALSE);

        /* Initiate register write */
        status = CSL_MDIO_phyInitiateRegWriteC45(mdioRegs, userCh, phyAddr, mmd, reg, val);
        ENETTRACE_ERR_IF(status != CSL_PASS,
                         "failed to initiate PHY %u C45 MMD %u register %u write: %d\n",
                         phyAddr, mmd, reg, status);

        /* Wait for register write transaction to complete */
        if (status == CSL_PASS)
        {
            do
            {
                isComplete = CSL_MDIO_isPhyRegAccessComplete(mdioRegs, userCh);
            }
            while (isComplete == FALSE);
        }
    }
    else
    {
        ENETTRACE_ERR("PHY %u is not configured for C45 access\n", phyAddr);
        status = ENET_EPERM;
    }

    return status;
}

static int32_t Mdio_writeRegTriggerC45(CSL_mdioHandle mdioRegs,
                                uint32_t userCh,
                                uint32_t mmd,
                                uint32_t phyAddr,
                                uint16_t reg,
                                uint16_t val)
{
    uint32_t c45EnMask;
    int32_t status = CSL_EFAIL;
    bool accessRegBusy = true;

    c45EnMask = CSL_MDIO_getClause45EnableMask(mdioRegs);
    if (ENET_IS_BIT_SET(c45EnMask, phyAddr))
    {
        if(CSL_MDIO_isPhyRegAccessComplete(mdioRegs, userCh))
        {
            accessRegBusy = false;
        }

        if(accessRegBusy == false)
        {
            /* Initiate register write */
            status = CSL_MDIO_phyInitiateRegWriteC45(mdioRegs, userCh, phyAddr, mmd, reg, val);
            ENETTRACE_ERR_IF(status != CSL_PASS,
                            "failed to initiate PHY %u C45 MMD %u register %u write: %d\n",
                            phyAddr, mmd, reg, status);
        }
    }
    else
    {
        ENETTRACE_ERR("PHY %u is not configured for C45 access\n", phyAddr);
        status = ENET_EPERM;
    }

    return status;
}

static int32_t Mdio_writeRegCompleteC45(CSL_mdioHandle mdioRegs,
                                uint32_t userCh,
                                uint32_t mmd,
                                uint32_t phyAddr,
                                uint16_t reg,
                                uint16_t val)
{
    uint32_t c45EnMask;
    int32_t status = CSL_EFAIL;

    c45EnMask = CSL_MDIO_getClause45EnableMask(mdioRegs);
    if (ENET_IS_BIT_SET(c45EnMask, phyAddr))
    {
        if(CSL_MDIO_isPhyRegAccessComplete(mdioRegs, userCh))
        {
            uint32_t regPhyAddr, regPhyRegNum, regWrVal;

            regPhyAddr = CSL_FEXT(mdioRegs->USER_GROUP[userCh].USER_ACCESS_REG, MDIO_USER_GROUP_USER_ACCESS_REG_PHYADR);
            regPhyRegNum = CSL_FEXT(mdioRegs->USER_GROUP[userCh].USER_ACCESS_REG, MDIO_USER_GROUP_USER_ACCESS_REG_REGADR);
            regWrVal = CSL_FEXT(mdioRegs->USER_GROUP[userCh].USER_ACCESS_REG, MDIO_USER_GROUP_USER_ACCESS_REG_DATA);
            if ((regPhyAddr == phyAddr)
                &&
                (regPhyRegNum == reg)
                &&
                (regWrVal == val))
            {
                status = ENET_SOK;
            }
            else
            {
                status = ENET_EFAIL;
            }
        }
        else
        {
            status = ENET_SINPROGRESS;
        }
    }
    else
    {
        ENETTRACE_ERR("PHY %u is not configured for C45 access\n", phyAddr);
        status = ENET_EPERM;
    }

    return status;

}

#endif /* MDIO_CLAUSE45 */

static void Mdio_handleIntr(CSL_mdioHandle mdioRegs,
                            Mdio_Callbacks *callbacks)
{
    Mdio_PhyStatus phyStatus;
    uint32_t phyAddr;
    uint32_t isStatusChangeMode;
    uint32_t pollEnMask = ENET_MDIO_PHY_ADDR_MASK_NONE;
    uint32_t linkChanged;
    uint32_t i;
    bool linkInt0Changed = false;
    bool linkInt1Changed = false;

    linkChanged = CSL_MDIO_isUnmaskedLinkStatusChangeIntSet(mdioRegs, 0U);
    if (linkChanged == 1U)
    {
        CSL_MDIO_clearUnmaskedLinkStatusChangeInt(mdioRegs, 0U);
        linkInt0Changed = true;
    }

    linkChanged = CSL_MDIO_isUnmaskedLinkStatusChangeIntSet(mdioRegs, 1U);
    if (linkChanged == 1U)
    {
        CSL_MDIO_clearUnmaskedLinkStatusChangeInt(mdioRegs, 1U);
        linkInt1Changed = true;
    }

    /* Only report state change of PHYs being polled */
    isStatusChangeMode = CSL_MDIO_isStateChangeModeEnabled(mdioRegs);
    if (isStatusChangeMode == 0)
    {
        for (i = 0U; i < MDIO_PHY_MONITOR_MAX; i++)
        {
            phyAddr = CSL_MDIO_getLinkStatusChangePhyAddr(mdioRegs, i);

            pollEnMask |= ENET_MDIO_PHY_ADDR_MASK(phyAddr);
        }

        phyStatus.aliveMask  = ENET_MDIO_PHY_ADDR_MASK_NONE;
        phyStatus.linkedMask = mdioRegs->LINK_REG & pollEnMask;
    }
    else
    {
        pollEnMask = CSL_MDIO_getPollEnableMask(mdioRegs);

        phyStatus.aliveMask  = mdioRegs->ALIVE_REG & pollEnMask;
        phyStatus.linkedMask = mdioRegs->LINK_REG & pollEnMask;
    }

    /* MDIO_LINKINT[0] event handling */
    if (linkInt0Changed)
    {
        if (callbacks->linkStateCb != NULL)
        {
            callbacks->linkStateCb(ENET_MDIO_GROUP_0, &phyStatus, callbacks->cbArgs);
        }
    }

    /* MDIO_LINKINT[1] event handling */
    if (linkInt1Changed)
    {
        if (isStatusChangeMode == 0U)
        {
            if (callbacks->linkStateCb != NULL)
            {
                callbacks->linkStateCb(ENET_MDIO_GROUP_1, &phyStatus, callbacks->cbArgs);
            }
        }
        else
        {
            /* MDIO_LINKINT[1] is unexpected in State Change Mode */
            /* TODO: Report an error message */
        }
    }
}

#define MDIO_INVOKE_IOCTL_HANDLER(hMod, mdioRegs, prms, cmd, status)    \
do {                                                                    \
    if (hMdio->mode == MDIO_MODE_MANUAL)                                \
    {                                                                   \
        status = Mdio_manual_ioctl_handler_##cmd(hMod, mdioRegs, prms); \
    }                                                                   \
    else                                                                \
    {                                                                   \
        status = Mdio_normal_ioctl_handler_##cmd(hMod, mdioRegs, prms); \
    }                                                                   \
} while(0)

int32_t Mdio_ioctl_handler_ENET_MDIO_IOCTL_IS_ALIVE(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    Mdio_Handle hMdio = (Mdio_Handle)hMod;
    int32_t status;

    MDIO_INVOKE_IOCTL_HANDLER(hMod, mdioRegs, prms, ENET_MDIO_IOCTL_IS_ALIVE, status);
    return status;
}

int32_t Mdio_ioctl_handler_ENET_MDIO_IOCTL_IS_LINKED(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    Mdio_Handle hMdio = (Mdio_Handle)hMod;
    int32_t status;

    MDIO_INVOKE_IOCTL_HANDLER(hMod, mdioRegs, prms, ENET_MDIO_IOCTL_IS_LINKED, status);
    return status;
}

int32_t Mdio_ioctl_handler_ENET_MDIO_IOCTL_IS_POLL_ENABLED(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    Mdio_Handle hMdio = (Mdio_Handle)hMod;
    int32_t status;

    MDIO_INVOKE_IOCTL_HANDLER(hMod, mdioRegs, prms, ENET_MDIO_IOCTL_IS_POLL_ENABLED, status);
    return status;
}

int32_t Mdio_ioctl_handler_ENET_MDIO_IOCTL_C22_READ(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    Mdio_Handle hMdio = (Mdio_Handle)hMod;
    int32_t status;

    MDIO_INVOKE_IOCTL_HANDLER(hMod, mdioRegs, prms, ENET_MDIO_IOCTL_C22_READ, status);
    return status;
}

int32_t Mdio_ioctl_handler_ENET_MDIO_IOCTL_C22_WRITE(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    Mdio_Handle hMdio = (Mdio_Handle)hMod;
    int32_t status;

    MDIO_INVOKE_IOCTL_HANDLER(hMod, mdioRegs, prms, ENET_MDIO_IOCTL_C22_WRITE, status);
    return status;
}

int32_t Mdio_ioctl_handler_ENET_MDIO_IOCTL_C45_READ(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    Mdio_Handle hMdio = (Mdio_Handle)hMod;
    int32_t status;

    MDIO_INVOKE_IOCTL_HANDLER(hMod, mdioRegs, prms, ENET_MDIO_IOCTL_C45_READ, status);
    return status;
}
int32_t Mdio_ioctl_handler_ENET_MDIO_IOCTL_C45_WRITE(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    Mdio_Handle hMdio = (Mdio_Handle)hMod;
    int32_t status;

    MDIO_INVOKE_IOCTL_HANDLER(hMod, mdioRegs, prms, ENET_MDIO_IOCTL_C45_WRITE, status);
    return status;
}

int32_t Mdio_ioctl_handler_ENET_MDIO_IOCTL_C22_ASYNC_READ_TRIGGER(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    Mdio_Handle hMdio = (Mdio_Handle)hMod;
    int32_t status;

    MDIO_INVOKE_IOCTL_HANDLER(hMod, mdioRegs, prms, ENET_MDIO_IOCTL_C22_ASYNC_READ_TRIGGER, status);
    return status;
}

int32_t Mdio_ioctl_handler_ENET_MDIO_IOCTL_C22_ASYNC_READ_COMPLETE(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    Mdio_Handle hMdio = (Mdio_Handle)hMod;
    int32_t status;

    MDIO_INVOKE_IOCTL_HANDLER(hMod, mdioRegs, prms, ENET_MDIO_IOCTL_C22_ASYNC_READ_COMPLETE, status);
    return status;
}

int32_t Mdio_ioctl_handler_ENET_MDIO_IOCTL_C22_ASYNC_WRITE_TRIGGER(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    Mdio_Handle hMdio = (Mdio_Handle)hMod;
    int32_t status;

    MDIO_INVOKE_IOCTL_HANDLER(hMod, mdioRegs, prms, ENET_MDIO_IOCTL_C22_ASYNC_WRITE_TRIGGER, status);
    return status;
}

int32_t Mdio_ioctl_handler_ENET_MDIO_IOCTL_C22_ASYNC_WRITE_COMPLETE(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    Mdio_Handle hMdio = (Mdio_Handle)hMod;
    int32_t status;

    MDIO_INVOKE_IOCTL_HANDLER(hMod, mdioRegs, prms, ENET_MDIO_IOCTL_C22_ASYNC_WRITE_COMPLETE, status);
    return status;
}

int32_t Mdio_ioctl_handler_ENET_MDIO_IOCTL_C45_ASYNC_READ_TRIGGER(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    Mdio_Handle hMdio = (Mdio_Handle)hMod;
    int32_t status;

    MDIO_INVOKE_IOCTL_HANDLER(hMod, mdioRegs, prms, ENET_MDIO_IOCTL_C45_ASYNC_READ_TRIGGER, status);
    return status;
}

int32_t Mdio_ioctl_handler_ENET_MDIO_IOCTL_C45_ASYNC_READ_COMPLETE(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    Mdio_Handle hMdio = (Mdio_Handle)hMod;
    int32_t status;

    MDIO_INVOKE_IOCTL_HANDLER(hMod, mdioRegs, prms, ENET_MDIO_IOCTL_C45_ASYNC_READ_COMPLETE, status);
    return status;
}

int32_t Mdio_ioctl_handler_ENET_MDIO_IOCTL_C45_ASYNC_WRITE_TRIGGER(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    Mdio_Handle hMdio = (Mdio_Handle)hMod;
    int32_t status;

    MDIO_INVOKE_IOCTL_HANDLER(hMod, mdioRegs, prms, ENET_MDIO_IOCTL_C45_ASYNC_WRITE_TRIGGER, status);
    return status;
}

int32_t Mdio_ioctl_handler_ENET_MDIO_IOCTL_C45_ASYNC_WRITE_COMPLETE(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    Mdio_Handle hMdio = (Mdio_Handle)hMod;
    int32_t status;

    MDIO_INVOKE_IOCTL_HANDLER(hMod, mdioRegs, prms, ENET_MDIO_IOCTL_C45_ASYNC_WRITE_COMPLETE, status);
    return status;
}

