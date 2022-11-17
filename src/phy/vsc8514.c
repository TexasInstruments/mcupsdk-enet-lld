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
 * \file  vsc8514.c
 *
 * \brief This file contains the implementation of the VSC8514 PHY.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <drivers/hw_include/cslr.h>
#include <include/core/enet_utils.h>
#include <priv/core/enet_trace_priv.h>
#include <include/phy/enetphy.h>
#include <include/phy/vsc8514.h>
#include "enetphy_priv.h"
#include "generic_phy.h"
#include "vsc8514_priv.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define VSC8514_OUI                           (0x000001C1U)
#define VSC8514_MODEL                         (0x27U)
#define VSC8514_REV                           (0x00U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static bool Vsc8514_isPhyDevSupported(EnetPhy_Handle hPhy,
                                      const EnetPhy_Version *version);

static bool Vsc8514_isMacModeSupported(EnetPhy_Handle hPhy,
                                       EnetPhy_Mii mii);

static int32_t Vsc8514_writeMicroCmd(EnetPhy_Handle hPhy);

static int32_t Vsc8514_config(EnetPhy_Handle hPhy,
                              const EnetPhy_Cfg *cfg,
                              EnetPhy_Mii mii);

static void Vsc8514_reset(EnetPhy_Handle hPhy);

static bool Vsc8514_isResetComplete(EnetPhy_Handle hPhy);

static void Vsc8514_rmwExtReg(EnetPhy_Handle hPhy,
                              Vsc8514_ExtRegSetType extRegSet,
                              uint32_t reg,
                              uint16_t mask,
                              uint16_t shift,
                              uint16_t val);

static void Vsc8514_readExtReg(EnetPhy_Handle hPhy,
                               Vsc8514_ExtRegSetType extRegSet,
                               uint32_t reg,
                               uint16_t *val);

static void Vsc8514_writeExtReg(EnetPhy_Handle hPhy,
                                Vsc8514_ExtRegSetType extRegSet,
                                uint32_t reg,
                                uint16_t val);

static void Vsc8514_printRegs(EnetPhy_Handle hPhy);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

EnetPhy_Drv gEnetPhyDrvVsc8514 =
{
    .name               = "vsc8514",
    .isPhyDevSupported  = Vsc8514_isPhyDevSupported,
    .isMacModeSupported = Vsc8514_isMacModeSupported,
    .config             = Vsc8514_config,
    .reset              = Vsc8514_reset,
    .isResetComplete    = Vsc8514_isResetComplete,
    .readExtReg         = GenericPhy_readExtReg,
    .writeExtReg        = GenericPhy_writeExtReg,
    .printRegs          = Vsc8514_printRegs,
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void Vsc8514_initCfg(Vsc8514_Cfg *cfg)
{
    /* No extended config parameters at the moment */
}

static bool Vsc8514_isPhyDevSupported(EnetPhy_Handle hPhy,
                                      const EnetPhy_Version *version)
{
    bool supported = false;

    if ((version->oui == VSC8514_OUI) &&
        (version->model == VSC8514_MODEL) &&
        (version->revision == VSC8514_REV))
    {
        supported = true;
    }

    return supported;
}

static bool Vsc8514_isMacModeSupported(EnetPhy_Handle hPhy,
                                       EnetPhy_Mii mii)
{
    bool supported;

    switch (mii)
    {
        case ENETPHY_MAC_MII_QSGMII:
            supported = true;
            break;

        default:
            supported = false;
            break;
    }

    return supported;
}

static int32_t Vsc8514_config(EnetPhy_Handle hPhy,
                              const EnetPhy_Cfg *cfg,
                              EnetPhy_Mii mii)
{
    const Vsc8514_Cfg *extendedCfg = (const Vsc8514_Cfg *)cfg->extendedCfg;
    uint32_t extendedCfgSize = cfg->extendedCfgSize;
    int32_t status = ENETPHY_SOK;

    if ((extendedCfg == NULL) ||
        (extendedCfgSize != sizeof(*extendedCfg)))
    {
        ENETTRACE_ERR("PHY %u: invalid config params (cfg=%p, size=%u)\n",
                      hPhy->addr, extendedCfg, extendedCfgSize);
        status = ENETPHY_EINVALIDPARAMS;
    }

    /* Configure MAC configuration register to set QSGMII mac mode */
    Vsc8514_rmwExtReg(hPhy,
                      VSC8514_EXTREGSET_GPIO,
                      VSC8514_GPIO_MACMODE,
                      VSC8514_GPIO_MACMODE_MACMODE_MASK,
                      VSC8514_GPIO_MACMODE_MACMODE_SHIFT,
                      VSC8514_GPIO_MACMODE_MACMODE_QSGMII);

    /* Enable 4 ports MAC QSGMII using micro command register */
    status = Vsc8514_writeMicroCmd(hPhy);
    ENETTRACE_ERR_IF(ENETPHY_SOK != status,
                     "PHY %u: Micro command write failed \n", hPhy->addr);
    Enet_assert(ENETPHY_SOK == status);

    /* Configure media operating mode in PHY extended control register */
    EnetPhy_writeReg(hPhy, VSC8514_EXTENDED_PHYCTRL1, 0x0U);

    /* Software reset PHY to apply initial SGMII configure */
    Vsc8514_reset(hPhy);
    while(false == Vsc8514_isResetComplete(hPhy))
    {
        /* Do nothing until reset is complete */
    }

    /* Enable MAC interface restart on media link change */
    Vsc8514_rmwExtReg(hPhy,
                      VSC8514_EXTREGSET_PAGE3,
                      VSC8514_EXT3_MACSERDES_PCS_CTRL,
                      VSC8514_EXT3_MACSERDES_PCS_CTRL_MACIF_RESTART_MASK,
                      VSC8514_EXT3_MACSERDES_PCS_CTRL_MACIF_RESTART_SHIFT,
                      1U);

    /* Enable auto-negotiation */
    Vsc8514_rmwExtReg(hPhy,
                      VSC8514_EXTREGSET_PAGE3,
                      VSC8514_EXT3_MACSERDES_PCS_CTRL,
                      VSC8514_EXT3_MACSERDES_PCS_CTRL_AUTONEG_EN_MASK,
                      VSC8514_EXT3_MACSERDES_PCS_CTRL_AUTONEG_EN_SHIFT,
                      1U);

    /* Restart auto-negotiation */
    Vsc8514_rmwExtReg(hPhy,
                      VSC8514_EXTREGSET_PAGE3,
                      VSC8514_EXT3_MACSERDES_PCS_CTRL,
                      VSC8514_EXT3_MACSERDES_PCS_CTRL_AUTONEG_RESTART_MASK,
                      VSC8514_EXT3_MACSERDES_PCS_CTRL_AUTONEG_RESTART_SHIFT,
                      1U);

    return status;
}

static void Vsc8514_reset(EnetPhy_Handle hPhy)
{
    GenericPhy_reset(hPhy);
}

static bool Vsc8514_isResetComplete(EnetPhy_Handle hPhy)
{
    return GenericPhy_isResetComplete(hPhy);
}

static void Vsc8514_readExtReg(EnetPhy_Handle hPhy,
                               Vsc8514_ExtRegSetType extRegSet,
                               uint32_t reg,
                               uint16_t *val)
{
    ENETTRACE_VERBOSE("PHY %u: Read reg %u val 0x%04x\n", hPhy->addr, reg, *val);

    /* Set page select */
    EnetPhy_writeReg(hPhy, VSC8514_PAGE_ACCESS, extRegSet);

    EnetPhy_readReg(hPhy, reg, val);

    /* Set page select register back to 0 to access main register space */
    EnetPhy_writeReg(hPhy, VSC8514_PAGE_ACCESS, VSC8514_EXTREGSET_MAIN);
}

static void Vsc8514_writeExtReg(EnetPhy_Handle hPhy,
                                Vsc8514_ExtRegSetType extRegSet,
                                uint32_t reg,
                                uint16_t val)
{
    ENETTRACE_VERBOSE("PHY %u: write reg %u val 0x%04x\n", hPhy->addr, reg, val);

    /* Set page select */
    EnetPhy_writeReg(hPhy, VSC8514_PAGE_ACCESS, extRegSet);

    EnetPhy_writeReg(hPhy, reg, val);

    /* Set page select register back to 0 to access main register space */
    EnetPhy_writeReg(hPhy, VSC8514_PAGE_ACCESS, VSC8514_EXTREGSET_MAIN);
}

static void Vsc8514_rmwExtReg(EnetPhy_Handle hPhy,
                              Vsc8514_ExtRegSetType extRegSet,
                              uint32_t reg,
                              uint16_t mask,
                              uint16_t shift,
                              uint16_t val)
{
    uint16_t data;

    ENETTRACE_VERBOSE("PHY %u: write extended reg %u mask 0x%04x val 0x%04x\n", hPhy->addr, reg, mask, val);
    Vsc8514_readExtReg(hPhy, extRegSet, reg, &data);
    CSL_REG16_FINS_RAW(&data, mask, shift, val);
    Vsc8514_writeExtReg(hPhy, extRegSet, reg, data);
}

static int32_t Vsc8514_writeMicroCmd(EnetPhy_Handle hPhy)
{
    uint16_t regVal;
    int32_t status = ENETPHY_SOK;

    /* Set page select */
    status = EnetPhy_writeReg(hPhy, VSC8514_PAGE_ACCESS, VSC8514_EXTREGSET_GPIO);

    if (ENETPHY_SOK == status)
    {
        while (true)
        {
            status = EnetPhy_readReg(hPhy, VSC8514_GPIO_COMMAND, &regVal);

            /* Wait until not busy */
            if (0U == (regVal & VSC8514_GPIO_COMMAND_GO_MASK))
            {
                break;
            }
            EnetUtils_delay(100U);
        }
    }

    if (ENETPHY_SOK == status)
    {
        /* Enable 4 ports MAC QSGMII */
        regVal = VSC8514_GPIO_COMMAND_GO_SET | VSC8514_GPIO_COMMAND_QSGMII;
        status = EnetPhy_writeReg(hPhy, VSC8514_GPIO_COMMAND, regVal);
    }

    if (ENETPHY_SOK == status)
    {
        while (true)
        {
            /* Commands may take up to 25 ms to complete before command complete bit
             * (bit 15) changes to 0 */
            EnetUtils_delay(2500U);
            status = EnetPhy_readReg(hPhy, VSC8514_GPIO_COMMAND, &regVal);

            /* Wait until not busy */
            if (VSC8514_GPIO_COMMAND_QSGMII == regVal)
            {
                break;
            }
        }
    }

    if (ENETPHY_SOK == status)
    {
        /* Set page select register back to 0x0 to access main register space  */
        status = EnetPhy_writeReg(hPhy, VSC8514_PAGE_ACCESS, VSC8514_EXTREGSET_MAIN);
    }

    return status;
}

static void Vsc8514_printRegs(EnetPhy_Handle hPhy)
{
    uint32_t phyAddr = hPhy->addr;
    uint16_t val;

    EnetPhy_readReg(hPhy, PHY_BMCR, &val);
    EnetUtils_printf("PHY %u: BMCR    = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, PHY_BMSR, &val);
    EnetUtils_printf("PHY %u: BMSR    = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, PHY_PHYIDR1, &val);
    EnetUtils_printf("PHY %u: PHYIDR1 = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, PHY_PHYIDR2, &val);
    EnetUtils_printf("PHY %u: PHYIDR2 = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, PHY_ANAR, &val);
    EnetUtils_printf("PHY %u: ANAR    = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, PHY_ANLPAR, &val);
    EnetUtils_printf("PHY %u: ANLPAR  = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, PHY_ANER, &val);
    EnetUtils_printf("PHY %u: ANER    = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, PHY_ANNPTR, &val);
    EnetUtils_printf("PHY %u: ANNPTR  = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, PHY_ANNPRR, &val);
    EnetUtils_printf("PHY %u: ANNPRR  = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, PHY_GIGCR, &val);
    EnetUtils_printf("PHY %u: CFG1    = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, PHY_GIGSR, &val);
    EnetUtils_printf("PHY %u: STS1    = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, PHY_GIGESR, &val);
    EnetUtils_printf("PHY %u: 1KSCR   = 0x%04x\n", phyAddr, val);
}
