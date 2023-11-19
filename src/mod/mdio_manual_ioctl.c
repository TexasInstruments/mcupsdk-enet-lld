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
 * \file  mdio_manual_ioctl.c
 *
 * \brief This file contains the implementation of the MDIO Manual mode IOCTL functionality
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
#include <priv/mod/cpsw_clks.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/*! \brief PHY Basic Mode Status Register (BMSR) */
#define PHY_BMSR                              (0x01U)

/*! \brief Link status bit mask in PHY Basic Mode Status Register (BMSR) */
#define LINK_STATUS_BITMASK                   (0x04U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void Mdio_manualModeFieldSend(EnetMod_Handle hMod,
                                    uint32_t iMsb,
                                    uint32_t iVal);

static uint32_t  Mdio_manualModePhyRegRead22(EnetMod_Handle hMod,
                                        uint32_t phyAddr,
                                        uint32_t regNum,
                                        uint16_t *pData);

static void Mdio_manualModePhyRegWrite22(EnetMod_Handle hMod,
                                  uint32_t phyAddr,
                                  uint32_t regNum,
                                  uint16_t wrVal);

static void Mdio_manualModeToggleMdclk(CSL_mdioHandle hMdioRegs, const uint32_t halfCycleDelay_ns);

static uint32_t Mdio_manualModeScanPhyAddress(EnetMod_Handle hMod);

#if ENET_CFG_IS_ON(MDIO_CLAUSE45)

static void Mdio_manualModePhyRegAddWriteC45(EnetMod_Handle hMod,
                                              uint32_t phyAddr,
                                              uint32_t mmd,
                                              uint16_t wrVal);

static uint32_t  Mdio_manualModePhyRegValReadC45(EnetMod_Handle hMod,
                                          uint32_t phyAddr,
                                          uint32_t mmd,
                                          uint16_t *pData);

static int32_t Mdio_manualModePhyRegReadC45(EnetMod_Handle hMod,
                                            uint32_t mmd,
                                            uint8_t phyAddr,
                                            uint16_t reg,
                                            uint16_t *val);

static void Mdio_manualModeSendPreamble(EnetMod_Handle hMod);

static void Mdio_manualModePhyRegValWriteC45(EnetMod_Handle hMod,
                                              uint32_t phyAddr,
                                              uint32_t mmd,
                                              uint16_t wrVal);

static int32_t Mdio_manualModePhyRegWriteC45(EnetMod_Handle hMod,
                                            uint32_t mmd,
                                            uint8_t phyAddr,
                                            uint16_t reg,
                                            uint16_t val);

#endif

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static uint32_t  Mdio_manualModePhyRegRead22(EnetMod_Handle hMod,
                                        uint32_t phyAddr,
                                        uint32_t regNum,
                                        uint16_t *pData)
{
    uint32_t i, sts;
    uint16_t tmp;
    char ack;
    Mdio_Handle hMdio = (Mdio_Handle)hMod;
    CSL_mdioHandle hMdioRegs = (CSL_mdioHandle)hMod->virtAddr;

    CSL_MDIO_setMdclkLow(hMdioRegs); /* Disable Phy Interrupt driver */

    CSL_MDIO_setMdoOutputEnable(hMdioRegs); /* Enable our drive capability */

    Mdio_manualModeFieldSend(hMod, 0x80000000, 0xFFFFFFFF); /* Send MDIO preamble */
    Mdio_manualModeFieldSend(hMod, 0x8, 0x6); /* Issue clause 22 MII read function {0,1,1,0}*/
    Mdio_manualModeFieldSend(hMod, 0x10, phyAddr); /* Send the device number MSB first */
    Mdio_manualModeFieldSend(hMod, 0x10, regNum); /* Send the register number MSB first */

    CSL_MDIO_setMdoInputEnable(hMdioRegs); /* send Turn-arround cycles */
    Mdio_manualModeToggleMdclk(hMdioRegs, hMdio->mdcHalfCycleNs);

    ack = CSL_MDIO_readMdi(hMdioRegs); /* Get PHY Ack */
    Mdio_manualModeToggleMdclk(hMdioRegs, hMdio->mdcHalfCycleNs);

    if (ack == 0) /* If Acked read the data */
    {
        for (tmp = 0, i = 0x8000; i; i >>= 1)
        {
            if (CSL_MDIO_readMdi(hMdioRegs))
            {
                tmp |= i;
            }
            Mdio_manualModeToggleMdclk(hMdioRegs, hMdio->mdcHalfCycleNs);
        }
        sts = CSL_PASS;
    }
    else
    {
        for (tmp = 0, i = 0x8000; i; i >>= 1)
        {
            Mdio_manualModeToggleMdclk(hMdioRegs, hMdio->mdcHalfCycleNs);
        }
        tmp = 0xffff;
        sts = CSL_ETIMEOUT;
    }

    CSL_MDIO_setMdclkLow(hMdioRegs); /* Give time for pull-up to work */
    CSL_MDIO_setMdclkLow(hMdioRegs);
    CSL_MDIO_setMdclkLow(hMdioRegs);
    Mdio_manualModeToggleMdclk(hMdioRegs, hMdio->mdcHalfCycleNs); /* re-enable PHY Interrupt function */
    *pData = tmp;
    return (sts);
}

static void Mdio_manualModePhyRegWrite22(EnetMod_Handle hMod,
                                  uint32_t phyAddr,
                                  uint32_t regNum,
                                  uint16_t wrVal)
{
    Mdio_Handle hMdio = (Mdio_Handle)hMod;
    CSL_mdioHandle hMdioRegs = (CSL_mdioHandle)hMod->virtAddr;

    CSL_MDIO_setMdclkLow(hMdioRegs);          /* Disable Phy Interrupt driver */
    CSL_MDIO_setMdoOutputEnable(hMdioRegs);          /* Enable our drive capability */

    Mdio_manualModeFieldSend(hMod, 0x80000000, 0xFFFFFFFF); /* Send 32-bit MDIO preamble */
    Mdio_manualModeFieldSend(hMod, 0x8,0x5);     /* Issue clause 22 MII write function {0,1,0,1}*/
    Mdio_manualModeFieldSend(hMod, 0x10,phyAddr);    /* Send the device number MSB first */
    Mdio_manualModeFieldSend(hMod, 0x10,regNum);    /* Send the register number MSB first */

    CSL_MDIO_setMdoHigh(hMdioRegs);          /* send Turn-arround cycles */
    Mdio_manualModeToggleMdclk(hMdioRegs, hMdio->mdcHalfCycleNs);
    CSL_MDIO_setMdoLow(hMdioRegs);
    Mdio_manualModeToggleMdclk(hMdioRegs, hMdio->mdcHalfCycleNs);
    Mdio_manualModeFieldSend(hMod, 0x08000, wrVal); /* Send Register data MSB first */
    CSL_MDIO_setMdoInputEnable(hMdioRegs);

    CSL_MDIO_setMdclkLow(hMdioRegs);          /* Give time for pull-up to work */
    CSL_MDIO_setMdclkLow(hMdioRegs);
    CSL_MDIO_setMdclkLow(hMdioRegs);
    Mdio_manualModeToggleMdclk(hMdioRegs, hMdio->mdcHalfCycleNs);        /* re-enable PHY Interrupt function */
}

static void Mdio_manualModeToggleMdclk(CSL_mdioHandle hMdioRegs, const uint32_t halfCycleDelay_ns)
{
    CSL_MDIO_setMdclkLow(hMdioRegs);
    EnetUtils_delayNs(halfCycleDelay_ns);
    CSL_MDIO_setMdclkHigh(hMdioRegs);
    EnetUtils_delayNs(halfCycleDelay_ns);
}

static void Mdio_manualModeFieldSend(EnetMod_Handle hMod,
                                    uint32_t iMsb,
                                    uint32_t iVal)
{
    Mdio_Handle hMdio = (Mdio_Handle)hMod;
    CSL_mdioHandle hMdioRegs = (CSL_mdioHandle)hMod->virtAddr;

    for (uint32_t i = iMsb; i; i >>= 1)
    {
        if (i & iVal)
        {
            CSL_MDIO_setMdoHigh(hMdioRegs);
        }
        else
        {
            CSL_MDIO_setMdoLow(hMdioRegs);
        }
        Mdio_manualModeToggleMdclk(hMdioRegs, hMdio->mdcHalfCycleNs);
    }

}

#if ENET_CFG_IS_ON(MDIO_CLAUSE45)
static void Mdio_manualModePhyRegAddWriteC45(EnetMod_Handle hMod,
                                              uint32_t phyAddr,
                                              uint32_t mmd,
                                              uint16_t wrVal)
{
    Mdio_Handle hMdio = (Mdio_Handle)hMod;
    CSL_mdioHandle hMdioRegs = (CSL_mdioHandle)hMod->virtAddr;

    Mdio_manualModeSendPreamble(hMod);
    CSL_MDIO_setMdclkLow(hMdioRegs);          /* Disable Phy Interrupt driver */
    CSL_MDIO_setMdoOutputEnable(hMdioRegs);          /* Enable our drive capability */

    Mdio_manualModeFieldSend(hMod, 0x08, 0x0);     /* Issue clause 45 MII write function {0,0,0,1}*/
    Mdio_manualModeFieldSend(hMod, 0x10, phyAddr);    /* Send the device number MSB first */
    Mdio_manualModeFieldSend(hMod, 0x10, mmd);    /* Send the register number MSB first */

    CSL_MDIO_setMdoHigh(hMdioRegs);          /* send Turn-arround cycles */
    Mdio_manualModeToggleMdclk(hMdioRegs, hMdio->mdcHalfCycleNs);
    CSL_MDIO_setMdoLow(hMdioRegs);
    Mdio_manualModeToggleMdclk(hMdioRegs, hMdio->mdcHalfCycleNs);

    Mdio_manualModeFieldSend(hMod, 0x08000, wrVal); /* Send Register data MSB first */
    CSL_MDIO_setMdoInputEnable(hMdioRegs);

    CSL_MDIO_setMdclkLow(hMdioRegs);          /* Give time for pull-up to work */
    CSL_MDIO_setMdclkLow(hMdioRegs);
    CSL_MDIO_setMdclkLow(hMdioRegs);
    Mdio_manualModeToggleMdclk(hMdioRegs, hMdio->mdcHalfCycleNs);        /* re-enable PHY Interrupt function */

}

static uint32_t  Mdio_manualModePhyRegValReadC45(EnetMod_Handle hMod,
                                          uint32_t phyAddr,
                                          uint32_t mmd,
                                          uint16_t *pData)
{
    uint32_t i, sts;
    uint8_t ack;
    uint16_t tmp;
    Mdio_Handle hMdio = (Mdio_Handle)hMod;
    CSL_mdioHandle hMdioRegs = (CSL_mdioHandle)hMod->virtAddr;

    Mdio_manualModeSendPreamble(hMod);
    CSL_MDIO_setMdclkLow(hMdioRegs); /* Disable Phy Interrupt driver */
    CSL_MDIO_setMdoOutputEnable(hMdioRegs); /* Enable our drive capability */

    Mdio_manualModeFieldSend(hMod, 0x08, 0x3); /* Issue clause 45 MII read function {0,0,1,1}*/
    Mdio_manualModeFieldSend(hMod, 0x10, phyAddr); /* Send the device number MSB first */
    Mdio_manualModeFieldSend(hMod, 0x10, mmd); /* Send the register number MSB first */

    CSL_MDIO_setMdclkLow(hMdioRegs); /* send Turn-arround cycles */
    CSL_MDIO_setMdoInputEnable(hMdioRegs);
    Mdio_manualModeToggleMdclk(hMdioRegs, hMdio->mdcHalfCycleNs);

    ack = CSL_MDIO_readMdi(hMdioRegs); /* Get PHY Ack */
    Mdio_manualModeToggleMdclk(hMdioRegs, hMdio->mdcHalfCycleNs);

    if (ack == 0) /* If Acked read the data */
    {
        for (tmp = 0, i = 0x8000; i; i >>= 1)
        {
            if (CSL_MDIO_readMdi(hMdioRegs))
            {
                tmp |= i;
            }
            Mdio_manualModeToggleMdclk(hMdioRegs, hMdio->mdcHalfCycleNs);
        }
        sts = CSL_PASS;
    }
    else
    {
        for (tmp = 0, i = 0x8000; i; i >>= 1)
        {
            Mdio_manualModeToggleMdclk(hMdioRegs, hMdio->mdcHalfCycleNs);
        }
        tmp = 0xffff;
        sts = CSL_ETIMEOUT;
    }

    CSL_MDIO_setMdclkLow(hMdioRegs); /* Give time for pull-up to work */
    CSL_MDIO_setMdclkLow(hMdioRegs);
    CSL_MDIO_setMdclkLow(hMdioRegs);
    Mdio_manualModeToggleMdclk(hMdioRegs, hMdio->mdcHalfCycleNs); /* re-enable PHY Interrupt function */
    *pData = tmp;
    return (sts);
}

static int32_t Mdio_manualModePhyRegReadC45(EnetMod_Handle hMod,
                                            uint32_t mmd,
                                            uint8_t phyAddr,
                                            uint16_t reg,
                                            uint16_t *val)
{
    int32_t status = CSL_EFAIL;

    /* Initiate register read */
    Mdio_manualModePhyRegAddWriteC45(hMod, phyAddr, mmd, reg);

    status = Mdio_manualModePhyRegValReadC45(hMod, phyAddr, mmd, val);
    /* Get the value read from PHY register once transaction is complete */

    ENETTRACE_ERR_IF(status == CSL_ETIMEOUT,
                     "C45 register read %u was not acknowledged by PHY %u: %d\n",
                     reg, phyAddr, status);
    ENETTRACE_ERR_IF(status == CSL_EFAIL,
                     "failed to read PHY %u C45 MMD %u register %u: %d\n",
                     phyAddr, mmd, reg, status);

    return status;
}

static void Mdio_manualModeSendPreamble(EnetMod_Handle hMod)
{
    Mdio_Handle hMdio = (Mdio_Handle)hMod;
    CSL_mdioHandle hMdioRegs = (CSL_mdioHandle)hMod->virtAddr;

    CSL_MDIO_setMdoOutputEnable(hMdioRegs); /* Enable our drive capability */

    CSL_MDIO_setMdclkLow(hMdioRegs); /* Give time for pull-up to work */
    CSL_MDIO_setMdclkLow(hMdioRegs);
    CSL_MDIO_setMdclkLow(hMdioRegs);
    CSL_MDIO_setMdclkHigh(hMdioRegs);
    for (uint32_t bit = 0; bit < 32; bit++)
    {
        CSL_MDIO_setMdclkLow(hMdioRegs); /* Give time for pull-up to work */
        CSL_MDIO_setMdclkLow(hMdioRegs);
        CSL_MDIO_setMdclkLow(hMdioRegs);
        Mdio_manualModeToggleMdclk(hMdioRegs, hMdio->mdcHalfCycleNs);
    }
}

static void Mdio_manualModePhyRegValWriteC45(EnetMod_Handle hMod,
                                              uint32_t phyAddr,
                                              uint32_t mmd,
                                              uint16_t wrVal)
{
    Mdio_Handle hMdio = (Mdio_Handle)hMod;
    CSL_mdioHandle hMdioRegs = (CSL_mdioHandle)hMod->virtAddr;

    Mdio_manualModeSendPreamble(hMod);
    CSL_MDIO_setMdclkLow(hMdioRegs);          /* Disable Phy Interrupt driver */
    CSL_MDIO_setMdoOutputEnable(hMdioRegs);          /* Enable our drive capability */

    Mdio_manualModeFieldSend(hMod, 0x08, 0x1);     /* Issue clause 45 MII write function {0,0,0,1}*/
    Mdio_manualModeFieldSend(hMod, 0x10, phyAddr);    /* Send the device number MSB first */
    Mdio_manualModeFieldSend(hMod, 0x10, mmd);    /* Send the register number MSB first */

    CSL_MDIO_setMdoHigh(hMdioRegs);          /* send Turn-arround cycles */
    Mdio_manualModeToggleMdclk(hMdioRegs, hMdio->mdcHalfCycleNs);
    CSL_MDIO_setMdoLow(hMdioRegs);
    Mdio_manualModeToggleMdclk(hMdioRegs, hMdio->mdcHalfCycleNs);

    Mdio_manualModeFieldSend(hMod, 0x08000, wrVal); /* Send Register data MSB first */
    CSL_MDIO_setMdoInputEnable(hMdioRegs);

    CSL_MDIO_setMdclkLow(hMdioRegs);          /* Give time for pull-up to work */
    CSL_MDIO_setMdclkLow(hMdioRegs);
    CSL_MDIO_setMdclkLow(hMdioRegs);
    Mdio_manualModeToggleMdclk(hMdioRegs, hMdio->mdcHalfCycleNs);        /* re-enable PHY Interrupt function */
}

static int32_t Mdio_manualModePhyRegWriteC45(EnetMod_Handle hMod,
                                            uint32_t mmd,
                                            uint8_t phyAddr,
                                            uint16_t reg,
                                            uint16_t val)
{
    Mdio_manualModePhyRegAddWriteC45(hMod, phyAddr, mmd, reg);
    Mdio_manualModePhyRegValWriteC45(hMod, phyAddr, mmd, val);

    return CSL_PASS;
}
#endif /* MDIO_CLAUSE45 */


static uint32_t Mdio_manualModeScanPhyAddress(EnetMod_Handle hMod)
{
    volatile uint32_t phyActiveBm = 0;

    for (uint32_t phyAdd = 0x00; phyAdd < 32; phyAdd++)
    {
        // Phy status register
        uint16_t val = 0;
        if (Mdio_manualModePhyRegRead22(hMod, phyAdd, PHY_BMSR, &val) == CSL_PASS)
        {
            phyActiveBm |= (1 << phyAdd);
        }
    }

    return phyActiveBm;
}


int32_t Mdio_manual_ioctl_handler_ENET_MDIO_IOCTL_IS_ALIVE(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    uint8_t *phyAddr = (uint8_t *)prms->inArgs;
    bool *alive = (bool *)prms->outArgs;
    uint16_t bmsrVal;
    int32_t status = ENET_SOK;

    *alive = (Mdio_manualModePhyRegRead22(hMod, *phyAddr, PHY_BMSR, &bmsrVal) == CSL_PASS);
    return status;
}

int32_t Mdio_manual_ioctl_handler_ENET_MDIO_IOCTL_IS_LINKED(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    uint8_t *phyAddr = (uint8_t *)prms->inArgs;
    bool *linked = (bool *)prms->outArgs;
    uint16_t bmsrVal;
    int32_t status = ENET_SOK;

    *linked = false;
    if (Mdio_manualModePhyRegRead22(hMod, *phyAddr, PHY_BMSR, &bmsrVal) == CSL_PASS)
    {
        if (bmsrVal & LINK_STATUS_BITMASK)
        {
            *linked = true;
        }
    }

    return status;
}

int32_t Mdio_manual_ioctl_handler_ENET_MDIO_IOCTL_IS_POLL_ENABLED(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    bool *pIsEnabled = (bool *)prms->outArgs;
    int32_t status = ENET_SOK;

    *pIsEnabled = false;
    return status;
}

int32_t Mdio_manual_ioctl_handler_ENET_MDIO_IOCTL_C22_READ(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    EnetMdio_C22ReadInArgs *inArgs = (EnetMdio_C22ReadInArgs *)prms->inArgs;
    uint16_t *val = (uint16_t *)prms->outArgs;
    int32_t status = ENET_SOK;

    uint32_t ack = Mdio_manualModePhyRegRead22(hMod,
                                              inArgs->phyAddr,
                                              inArgs->reg,
                                              val);

    status = (ack == CSL_PASS) ? ENET_SOK : ENET_EFAIL;
    ENETTRACE_ERR_IF(status != ENET_SOK,
                     "failed to read PHY %u C22 reg %u: %d\n",
                     inArgs->phyAddr, inArgs->reg, status);
    return status;
}

int32_t Mdio_manual_ioctl_handler_ENET_MDIO_IOCTL_C22_WRITE(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

    EnetMdio_C22WriteInArgs *inArgs = (EnetMdio_C22WriteInArgs *)prms->inArgs;
    Mdio_manualModePhyRegWrite22(hMod,
                                 inArgs->phyAddr,
                                 inArgs->reg,
                                 inArgs->val);
    return status;
}

int32_t Mdio_manual_ioctl_handler_ENET_MDIO_IOCTL_C45_READ(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

#if ENET_CFG_IS_ON(MDIO_CLAUSE45)
    EnetMdio_C45ReadInArgs *inArgs = (EnetMdio_C45ReadInArgs *)prms->inArgs;
    uint16_t *val = (uint16_t *)prms->outArgs;

    if (ENET_FEAT_IS_EN(hMod->features, MDIO_FEATURE_CLAUSE45))
    {
        status = Mdio_manualModePhyRegReadC45(hMod,
                                 (uint32_t)inArgs->mmd,
                                 inArgs->phyAddr,
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

int32_t Mdio_manual_ioctl_handler_ENET_MDIO_IOCTL_C45_WRITE(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

#if ENET_CFG_IS_ON(MDIO_CLAUSE45)
    EnetMdio_C45WriteInArgs *inArgs = (EnetMdio_C45WriteInArgs *)prms->inArgs;

    if (ENET_FEAT_IS_EN(hMod->features, MDIO_FEATURE_CLAUSE45))
    {
        status = Mdio_manualModePhyRegWriteC45(hMod,
                                  (uint32_t)inArgs->mmd,
                                  inArgs->phyAddr,
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

int32_t Mdio_manual_ioctl_handler_ENET_MDIO_IOCTL_C22_ASYNC_READ_TRIGGER(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_ENOTSUPPORTED;

    // not supported in MDIO manual mode.
    ENETTRACE_ERR("MDIO IOCTL cmd %d operation manual mode\n", ENET_MDIO_IOCTL_C22_ASYNC_READ_TRIGGER);
    return status;
}

int32_t Mdio_manual_ioctl_handler_ENET_MDIO_IOCTL_C22_ASYNC_WRITE_TRIGGER(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_ENOTSUPPORTED;

    // not supported in MDIO manual mode.
    ENETTRACE_ERR("MDIO IOCTL cmd %d operation manual mode\n", ENET_MDIO_IOCTL_C22_ASYNC_WRITE_TRIGGER);
    return status;
}

int32_t Mdio_manual_ioctl_handler_ENET_MDIO_IOCTL_C22_ASYNC_READ_COMPLETE(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_ENOTSUPPORTED;

    // not supported in MDIO manual mode.
    ENETTRACE_ERR("MDIO IOCTL cmd %d operation manual mode\n", ENET_MDIO_IOCTL_C22_ASYNC_READ_COMPLETE);
    return status;
}

int32_t Mdio_manual_ioctl_handler_ENET_MDIO_IOCTL_C22_ASYNC_WRITE_COMPLETE(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_ENOTSUPPORTED;

    // not supported in MDIO manual mode.
    ENETTRACE_ERR("MDIO IOCTL cmd %d operation manual mode\n", ENET_MDIO_IOCTL_C22_ASYNC_WRITE_COMPLETE);
    return status;
}

int32_t Mdio_manual_ioctl_handler_ENET_MDIO_IOCTL_C45_ASYNC_READ_TRIGGER(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_ENOTSUPPORTED;

    // not supported in MDIO manual mode.
    ENETTRACE_ERR("MDIO IOCTL cmd %d operation manual mode\n", ENET_MDIO_IOCTL_C45_ASYNC_READ_TRIGGER);
    return status;
}

int32_t Mdio_manual_ioctl_handler_ENET_MDIO_IOCTL_C45_ASYNC_WRITE_TRIGGER(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_ENOTSUPPORTED;

    // not supported in MDIO manual mode.
    ENETTRACE_ERR("MDIO IOCTL cmd %d operation manual mode\n", ENET_MDIO_IOCTL_C45_ASYNC_WRITE_TRIGGER);
    return status;
}

int32_t Mdio_manual_ioctl_handler_ENET_MDIO_IOCTL_C45_ASYNC_READ_COMPLETE(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_ENOTSUPPORTED;

    // not supported in MDIO manual mode.
    ENETTRACE_ERR("MDIO IOCTL cmd %d operation manual mode\n", ENET_MDIO_IOCTL_C45_ASYNC_READ_COMPLETE);
    return status;
}

int32_t Mdio_manual_ioctl_handler_ENET_MDIO_IOCTL_C45_ASYNC_WRITE_COMPLETE(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_ENOTSUPPORTED;

    // not supported in MDIO manual mode.
    ENETTRACE_ERR("MDIO IOCTL cmd %d operation manual mode\n", ENET_MDIO_IOCTL_C45_ASYNC_WRITE_COMPLETE);
    return status;
}

