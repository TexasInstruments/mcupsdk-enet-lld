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
 * \file  generic_phy.c
 *
 * \brief This file contains the implementation of the generic Ethernet PHY.
 *        It provides the basic functionality allowed with IEEE standard
 *        registers.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdarg.h>
#include <include/core/enet_utils.h>
#include <priv/core/enet_trace_priv.h>
#include <include/phy/enetphy.h>
#include "enetphy_priv.h"
#include "generic_phy.h"

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

static bool GenericPhy_isPhyDevSupported(EnetPhy_Handle hPhy,
                                         const EnetPhy_Version *version);

static bool GenericPhy_isMacModeSupported(EnetPhy_Handle hPhy,
                                          EnetPhy_Mii mii);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

EnetPhy_Drv gEnetPhyDrvGeneric =
{
    .name               = "generic",
    .isPhyDevSupported  = GenericPhy_isPhyDevSupported,
    .isMacModeSupported = GenericPhy_isMacModeSupported,
    .config             = NULL,
    .reset              = GenericPhy_reset,
    .isResetComplete    = GenericPhy_isResetComplete,
    .readExtReg         = GenericPhy_readExtReg,
    .writeExtReg        = GenericPhy_writeExtReg,
    .printRegs          = GenericPhy_printRegs,
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static bool GenericPhy_isPhyDevSupported(EnetPhy_Handle hPhy,
                                         const EnetPhy_Version *version)
{
    /* All IEEE-standard PHY models are supported */
    return true;
}

static bool GenericPhy_isMacModeSupported(EnetPhy_Handle hPhy,
                                          EnetPhy_Mii mii)
{
    /* All MAC modes are supported */
    return true;
}

void GenericPhy_reset(EnetPhy_Handle hPhy)
{
    ENETTRACE_DBG("PHY %u: reset\n", hPhy->addr);

    /* Reset the PHY */
    EnetPhy_rmwReg(hPhy, PHY_BMCR, BMCR_RESET, BMCR_RESET);
}

bool GenericPhy_isResetComplete(EnetPhy_Handle hPhy)
{
    int32_t status;
    uint16_t val;
    bool complete = false;

    /* Reset is complete when RESET bit has self-cleared */
    status = EnetPhy_readReg(hPhy, PHY_BMCR, &val);
    if (status == ENETPHY_SOK)
    {
        complete = ((val & BMCR_RESET) == 0U);
    }

    ENETTRACE_DBG("PHY %u: reset is %scomplete\n", hPhy->addr, complete ? "" : "not");

    return complete;
}

int32_t GenericPhy_readExtReg(EnetPhy_Handle hPhy,
                              uint32_t reg,
                              uint16_t *val)
{
    uint16_t devad = MMD_CR_DEVADDR;
    int32_t status;

    status = EnetPhy_writeReg(hPhy, PHY_MMD_CR, devad | MMD_CR_ADDR);

    if (status == ENETPHY_SOK)
    {
        status = EnetPhy_writeReg(hPhy, PHY_MMD_DR, reg);
    }

    if (status == ENETPHY_SOK)
    {
        EnetPhy_writeReg(hPhy, PHY_MMD_CR, devad | MMD_CR_DATA_NOPOSTINC);
    }

    if (status == ENETPHY_SOK)
    {
        status = EnetPhy_readReg(hPhy, PHY_MMD_DR, val);
    }

    ENETTRACE_VERBOSE_IF(status == ENETPHY_SOK,
                         "PHY %u: failed to read reg %u\n", hPhy->addr, reg);
    ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                     "PHY %u: read reg %u val 0x%04x\n", hPhy->addr, reg, *val);

    return status;
}

int32_t GenericPhy_writeExtReg(EnetPhy_Handle hPhy,
                               uint32_t reg,
                               uint16_t val)
{
    uint16_t devad = MMD_CR_DEVADDR;
    int32_t status;

    ENETTRACE_VERBOSE("PHY %u: write %u val 0x%04x\n", hPhy->addr, reg, val);

    status = EnetPhy_writeReg(hPhy, PHY_MMD_CR, devad | MMD_CR_ADDR);
    if (status == ENETPHY_SOK)
    {
        EnetPhy_writeReg(hPhy, PHY_MMD_DR, reg);
    }

    if (status == ENETPHY_SOK)
    {
        EnetPhy_writeReg(hPhy, PHY_MMD_CR, devad | MMD_CR_DATA_NOPOSTINC);
    }

    if (status == ENETPHY_SOK)
    {
        EnetPhy_writeReg(hPhy, PHY_MMD_DR, val);
    }

    ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                     "PHY %u: failed to write reg %u val 0x%04x\n", hPhy->addr, reg, val);

    return status;
}

void GenericPhy_printRegs(EnetPhy_Handle hPhy)
{
    uint32_t i;
    uint16_t val;

    for (i = PHY_BMCR; i <= PHY_GIGESR; i++)
    {
        EnetPhy_readReg(hPhy, i, &val);
        EnetUtils_printf("PHY %u: reg 0x%02x = 0x%04x\n", hPhy->addr, i, val);
    }
}
