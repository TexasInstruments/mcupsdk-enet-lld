/*
 *  Copyright (c) Texas Instruments Incorporated 2024
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
 * \file  myethphy.c
 *
 * \brief This file contains the implementation of the MYETHPHY PHY.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>

#include "myethphy.h"
#include "myethphy_priv.h"

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

void Myethphy_initCfg(Myethphy_Cfg *cfg);

void Myethphy_bind(EthPhyDrv_Handle* hPhy, 
                    uint8_t phyAddr, 
                    Phy_RegAccessCb_t* pRegAccessCb);

bool Myethphy_isPhyDevSupported(EthPhyDrv_Handle hPhy, 
                                const void *pVersion);

bool Myethphy_isMacModeSupported(EthPhyDrv_Handle hPhy, 
                                Phy_Mii mii);

int32_t Myethphy_config(EthPhyDrv_Handle hPhy,
                        const void *pExtCfg,
                        const uint32_t extCfgSize,
                        Phy_Mii mii, 
                        bool loopbackEn);

void Myethphy_reset(EthPhyDrv_Handle hPhy);

bool Myethphy_isResetComplete(EthPhyDrv_Handle hPhy);

void Myethphy_printRegs(EthPhyDrv_Handle hPhy);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

Phy_DrvObj_t gEnetPhyDrvMyethphy =
{
    .fxn =
    {
        .name               = "Myethphy",                 /*PHY driver name*/
        .bind               = Myethphy_bind,              /*bind driver handle and APIs to specific PHY device*/
        .isPhyDevSupported  = Myethphy_isPhyDevSupported, /*check if PHY driver supports a PHY model*/
        .isMacModeSupported = Myethphy_isMacModeSupported,/*check if a MAC mode is supported by the PHY driver*/
        .config             = Myethphy_config,            /*perform device-specific extended configuration*/
        .reset              = Myethphy_reset,             /*device specific reset*/
        .isResetComplete    = Myethphy_isResetComplete,   /*check if reset triggered by `reset()` is complete*/
        .readExtReg         = GenericPhy_readExtReg,      /*read extended registers*/
        .writeExtReg        = GenericPhy_writeExtReg,     /*write extended register*/
        .printRegs          = Myethphy_printRegs,         /*print PHY registers*/
        /*Below functions can only be supported when the PHY has a built-in PTP clock.*/
        .adjPtpFreq              = NULL,    /*adjust PTP clock frequency*/
        .adjPtpPhase             = NULL,    /*adjust PTP clock phase*/
        .getPtpTime              = NULL,    /*get current PHY PTP clock time*/
        .setPtpTime              = NULL,    /*set PHY PTP clock time*/
        .getPtpTxTime            = NULL,    /*get PHY PTP TX packet timestamp*/
        .getPtpRxTime            = NULL,    /*get PHY PTP RX packet timestamp*/
        .waitPtpTxTime           = NULL,    /*add PHY PTP TX packet info to a waiting TX timestamp list*/
        .procStatusFrame         = NULL,    /*process PHY status frame*/
        .getStatusFrameEthHeader = NULL,    /*get the Ethernet header of the PHY status frame*/
        .enablePtp               = NULL,    /*enable/disable the PHY PTP module*/
        .tickDriver              = NULL,    /*provide timer tick to the driver*/
        .enableEventCapture      = NULL,    /*enable/disable an event capture on a PHY GPIO pin*/
        .enableTriggerOutput     = NULL,    /*enable/disable trigger output on a GPIO pin*/
        .getEventTs              = NULL,    /*get event timestamp*/
    }
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void Myethphy_initCfg(Myethphy_Cfg *cfg)
{
    /*
     * Initialize the MYETHPHY specific extended config parameters here.
     */
}

void Myethphy_bind(EthPhyDrv_Handle* hPhy, uint8_t phyAddr, Phy_RegAccessCb_t* pRegAccessCb)
{
    Phy_Obj_t* pObj = (Phy_Obj_t*) hPhy;
    pObj->phyAddr = phyAddr;
    pObj->regAccessApi = *pRegAccessCb;
}

bool Myethphy_isPhyDevSupported(EthPhyDrv_Handle hPhy,
                                const void *pVersion)
{
	const Phy_Version *version = (Phy_Version *)pVersion;

    bool supported = false;

    /* 
     * Add condition to check if this device specific driver supports 
     * a given PHY hardware identified by its version (OUI).
     */

    return supported;
}

bool Myethphy_isMacModeSupported(EthPhyDrv_Handle hPhy,
                                Phy_Mii mii)
{
    bool supported = false;

    /* 
     * Add condition to check if the this device specific driver supports 
     * the requested MAC mode (MII, RMII, RGMII, SGMII, etc).
     */

    return supported;
}

int32_t Myethphy_config(EthPhyDrv_Handle hPhy,
                        const void *pExtCfg,
                        const uint32_t extCfgSize,
                        Phy_Mii mii, 
                        bool loopbackEn)
{
    uint8_t phyAddr = PhyPriv_getPhyAddr(hPhy);

    const Myethphy_Cfg *extendedCfg = (const Myethphy_Cfg *)pExtCfg;
    uint32_t extendedCfgSize = extCfgSize;
    int32_t status = PHY_SOK;

    if ((extendedCfg == NULL) ||
        (extendedCfgSize != sizeof(*extendedCfg)))
    {
        PHYTRACE_ERR("PHY %u: invalid config params (cfg=%p, size=%u)\r\n",
                     phyAddr, extendedCfg, extendedCfgSize);
        status = PHY_EINVALIDPARAMS;
    }

    /* 
     * Add code to perform any device-specific extended configuration.
     *
     * This function is optional and will not be called if 
     * the config function pointer in gEnetPhyDrvMyethphy is set to `NULL`.
     */

    return status;
}

void Myethphy_reset(EthPhyDrv_Handle hPhy)
{
    Phy_RegAccessCb_t* pRegAccessApi = PhyPriv_getRegAccessApi(hPhy);
    /* Global software reset: all PHY internal circuits including IEEE-defined
     * registers and all extended registers are reset */
    PHYTRACE_DBG("PHY %u: global soft-reset\n", PhyPriv_getPhyAddr(hPhy));

    /* 
     * Add code to start a soft-reset operation.
     *
     * This function is optional and will not be called if 
     * the reset function pointer in gEnetPhyDrvMyethphy is set to `NULL`.
     */
}

bool Myethphy_isResetComplete(EthPhyDrv_Handle hPhy)
{
    int32_t status;
    uint16_t val;
    bool complete = false;
    Phy_RegAccessCb_t* pRegAccessApi = PhyPriv_getRegAccessApi(hPhy);

    /* 
     * Add code to check if soft-reset operation is complete.
     *
     * If `reset()` is provided, `isResetComplete()` must be provided as well.
     */

    PHYTRACE_DBG("PHY %u: global soft-reset is %scomplete\n", PhyPriv_getPhyAddr(hPhy), complete ? "" : "not");

    return complete;
}

void Myethphy_printRegs(EthPhyDrv_Handle hPhy)
{
    uint16_t val;
    const uint8_t phyAddr = PhyPriv_getPhyAddr(hPhy);
    Phy_RegAccessCb_t* pRegAccessApi = PhyPriv_getRegAccessApi(hPhy);

    /* 
     * Add code to print PHY registers
     *
     * This function is optional and will not be called if 
     * the printRegs function pointer in gEnetPhyDrvMyethphy is set to `NULL`.
     */

    /*
     * Sample print statements to print BMCR register value:
     *
     * pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, PHY_BMCR, &val);
     * printf("PHY %u: BMCR        = 0x%04x\r\n",phyAddr, val);
     */
}
