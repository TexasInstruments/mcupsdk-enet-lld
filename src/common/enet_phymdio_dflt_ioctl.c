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
 * \file  enet_phymdio_dflt.c
 *
 * \brief This file contains the default implementation of the MDIO interface
 *        of the Ethernet PHY (ENETPHY) driver with Enet LLD APIs.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <include/phy/enetphy.h>
#include <include/core/enet_mod_mdio.h>
#include <include/core/enet_mod_phy.h>
#include <include/core/enet_mod_macport.h>
#include <include/common/enet_phymdio_dflt.h>
#include <priv/core/enet_trace_priv.h>
#include <priv/mod/phy_priv.h>
#include <priv/mod/phy_ioctl_priv.h>



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

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */



/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_GET_ID(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms)
{
#if ((ENET_CFG_TRACE_LEVEL >= ENET_CFG_TRACE_LEVEL_ERROR) && ENET_CFG_IS_OFF(TRACE_DISABLE_INFOSTRING))
    const EnetPhy_GenericInArgs *inArgs = (const EnetPhy_GenericInArgs *)prms->inArgs;
#endif
    EnetPhy_Version *version = (EnetPhy_Version *)prms->outArgs;
    int32_t status = ENET_SOK;

    status = EnetPhy_getId(hPhy, version);
    ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                     "Port %u: Failed to get PHY id: %d\n",
                     ENET_MACPORT_ID(inArgs->macPort), status);
    return status;
}

int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_GET_SUPPORTED_MODES(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

    /* Currently not supported */
    status = ENET_ENOTSUPPORTED;
    return status;
}

int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_GET_LOOPBACK_STATE(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

    /* Currently not supported */
    status = ENET_ENOTSUPPORTED;
    return status;
}

int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_IS_ALIVE(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms)
{
    bool *isAlive = (bool *)prms->outArgs;
    int32_t status = ENET_SOK;

    *isAlive = EnetPhy_isAlive(hPhy);
    return status;
}

int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_IS_LINKED(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms)
{
    bool *isLinked = (bool *)prms->outArgs;
    int32_t status = ENET_SOK;

    *isLinked = EnetPhy_isLinked(hPhy);
    return status;
}

int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_GET_LINK_MODE(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms)
{
#if ((ENET_CFG_TRACE_LEVEL >= ENET_CFG_TRACE_LEVEL_ERROR) && ENET_CFG_IS_OFF(TRACE_DISABLE_INFOSTRING))
    const EnetPhy_GenericInArgs *inArgs = (const EnetPhy_GenericInArgs *)prms->inArgs;
#endif
    EnetMacPort_LinkCfg *linkCfg = (EnetMacPort_LinkCfg *)prms->outArgs;
    EnetPhy_LinkCfg phyLinkCfg;
    int32_t status = ENET_SOK;

    status = EnetPhy_getLinkCfg(hPhy, &phyLinkCfg);
    if (status == ENETPHY_SOK)
    {
        linkCfg->speed = (Enet_Speed)phyLinkCfg.speed;
        linkCfg->duplexity = (Enet_Duplexity)phyLinkCfg.duplexity;
    }
    else
    {
        ENETTRACE_ERR("Port %u: Failed to get link config: %d\n",
                      ENET_MACPORT_ID(inArgs->macPort), status);
    }
    return status;
}

int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_RESET(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

    /* Currently not supported */
    status = ENET_ENOTSUPPORTED;
    return status;
}

int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_READ_REG(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms)
{
    const EnetPhy_ReadRegInArgs *inArgs = (const EnetPhy_ReadRegInArgs *)prms->inArgs;
    uint16_t *val = (uint16_t *)prms->outArgs;
    int32_t status = ENET_SOK;

    status = EnetPhy_readReg(hPhy, inArgs->reg, val);
    ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                     "Port %u: Failed to read reg %u: %d\n",
                     ENET_MACPORT_ID(inArgs->macPort), inArgs->reg, status);
    return status;
}

int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_WRITE_REG(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms)
{
    const EnetPhy_WriteRegInArgs *inArgs = (const EnetPhy_WriteRegInArgs *)prms->inArgs;
    int32_t status = ENET_SOK;

    status = EnetPhy_writeReg(hPhy, inArgs->reg, inArgs->val);
    ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                     "Port %u: Failed to write reg %u: %d\n",
                     ENET_MACPORT_ID(inArgs->macPort), inArgs->reg, status);
    return status;
}

int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_READ_EXT_REG(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms)
{
    const EnetPhy_ReadRegInArgs *inArgs = (const EnetPhy_ReadRegInArgs *)prms->inArgs;
    uint16_t *val = (uint16_t *)prms->outArgs;
    int32_t status = ENET_SOK;

    status = EnetPhy_readExtReg(hPhy, inArgs->reg, val);
    ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                     "Port %u: Failed to read ext reg %u: %d\n",
                     ENET_MACPORT_ID(inArgs->macPort), inArgs->reg, status);
    return status;
}

int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_WRITE_EXT_REG(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms)
{
    const EnetPhy_WriteRegInArgs *inArgs = (const EnetPhy_WriteRegInArgs *)prms->inArgs;
    int32_t status = ENET_SOK;

    status = EnetPhy_writeExtReg(hPhy, inArgs->reg, inArgs->val);
    ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                     "Port %u: Failed to write ext reg %u: %d\n",
                     ENET_MACPORT_ID(inArgs->macPort), inArgs->reg, status);
    return status;
}

int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_C45_READ_REG(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms)
{
    const EnetPhy_C45ReadRegInArgs *inArgs = (const EnetPhy_C45ReadRegInArgs *)prms->inArgs;
    uint16_t *val = (uint16_t *)prms->outArgs;
    int32_t status = ENET_SOK;

    status = EnetPhy_readC45Reg(hPhy, inArgs->mmd, inArgs->reg, val);
    ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                     "Port %u: Failed to read C45 mmd %u reg %u: %d\n",
                     ENET_MACPORT_ID(inArgs->macPort), inArgs->mmd, inArgs->reg, status);
    return status;
}

int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_C45_WRITE_REG(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms)
{
    const EnetPhy_C45WriteRegInArgs *inArgs = (const EnetPhy_C45WriteRegInArgs *)prms->inArgs;
    int32_t status = ENET_SOK;

    status = EnetPhy_writeC45Reg(hPhy, inArgs->mmd, inArgs->reg, inArgs->val);
    ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                     "Port %u: Failed to write C45 mmd %u reg %u: %d\n",
                     ENET_MACPORT_ID(inArgs->macPort), inArgs->mmd, inArgs->reg, status);
    return status;
}

int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_PRINT_REGS(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

    EnetPhy_printRegs(hPhy);
    return status;
}

