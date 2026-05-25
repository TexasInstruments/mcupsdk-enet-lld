/*
 *  Copyright (C) 2026 Texas Instruments Incorporated
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

#include "ti_board_config.h"

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <enet.h>
#include "dp83869.h"
#include <enet_apputils.h>
#include <enet_appboardutils.h>
#include <drivers/hw_include/cslr_soc.h>
#include <networking/enet/core/src/phy/enetphy_priv.h>
#include <generic_phy.h>
#include "ti_board_open_close.h"
#include <kernel/dpl/AddrTranslateP.h>


/* PHY drivers */
extern Phy_DrvObj_t gEnetPhyDrvDp83869;

/*! \brief All the registered PHY specific drivers. */
static const EthPhyDrv_If gEnetPhyDrvs[] =
{
    &gEnetPhyDrvDp83869,    /* DP83869 */
};

const EnetPhy_DrvInfoTbl gEnetPhyDrvTbl =
{
    .numHandles = ENET_ARRAYSIZE(gEnetPhyDrvs),
    .hPhyDrvList = gEnetPhyDrvs,
};

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
    MSS_CTRL:CPSW_CONTROL

    Instance          MSS_CTRL
    CPSW_CONTROL_RGMII1_ID_MODE     16  Writing 1'b1 would disable the internal clock delays. And those delays need to be handled on board.
    CPSW_CONTROL_RMII_REF_CLK_OE_N  8   To select the rmii_ref_clk from PAD or from MSS_RCM. 0: clock will be from mss_rcm through IO internal loopback 1: will be from
    CPSW_CONTROL_PORT1_MODE_SEL     2:0 Port 1 Interface
                                            00 = GMII/MII
                                            01 = RMII
                                            10 = RGMII
                                            11 = Not Supported
*/

#define MSS_CPSW_CONTROL_PORT_MODE_MII                                   (0x0U)
#define MSS_CPSW_CONTROL_PORT_MODE_RMII                                  (0x1U)
#define MSS_CPSW_CONTROL_PORT_MODE_RGMII                                 (0x2U)
/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static const EnetBoard_PortCfg *EnetBoard_getPortCfg(const EnetBoard_EthPort *ethPort);

static const EnetBoard_PortCfg *EnetBoard_findPortCfg(const EnetBoard_EthPort *ethPort,
                                                      const EnetBoard_PortCfg *ethPortCfgs,
                                                      uint32_t numEthPorts);

static void EnetBoard_enableExternalMux();

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/*!
 * \brief Common Processor Board (CPB) board's DP83869 PHY configuration.
 */
static const Dp83869_Cfg gEnetCpbBoard_ConfigEnetEthphy0PhyCfg =
{
    .txClkShiftEn         = true,
    .rxClkShiftEn         = true,
    .txDelayInPs          = 500U,  /* Value in pecosec. Refer to DLL_RX_DELAY_CTRL_SL field in ANA_RGMII_DLL_CTRL register of DP83869 PHY datasheet */
    .rxDelayInPs          = 500U,  /* Value in pecosec. Refer to DLL_TX_DELAY_CTRL_SL field in ANA_RGMII_DLL_CTRL register of DP83869 PHY datasheet */
    .txFifoDepth          = 4U,
    .impedanceInMilliOhms = 35000,  /* 35 ohms */
    .idleCntThresh        = 4U,     /* Improves short cable performance */
    .gpio0Mode            = DP83869_GPIO0_LED_2,
    .gpio1Mode            = DP83869_GPIO1_COL, /* Unused */
    .ledMode              =
    {
        DP83869_LED_RXTXACT,
        DP83869_LED_LINKED_100BTX,
        DP83869_LED_LINKED,
        DP83869_LED_LINKED_1000BT,
    },
};

/*
 * am263x-cc board configuration.
 *
 * RMII/RGMII/MII PHY connected to am263x-cc CPSW_3G MAC port.
 */
static const EnetBoard_PortCfg gEnetCpbBoard_am263x_cc_EthPort[] =
{
    {
        /* "CPSW3G" */
        .enetType = ENET_CPSW_3G,
        .instId   = 0U,
        .macPort  = ENET_MAC_PORT_1,
        .mii      = {ENET_MAC_LAYER_GMII, ENET_MAC_SUBLAYER_REDUCED},
        .phyCfg   =
        {
            .phyAddr         = 3,
            .isStrapped      = false,
            .skipExtendedCfg = false,
            .extendedCfg     = &gEnetCpbBoard_ConfigEnetEthphy0PhyCfg,
            .extendedCfgSize = sizeof(gEnetCpbBoard_ConfigEnetEthphy0PhyCfg)
        },
        .flags    = 0U,
    },
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

const EnetBoard_PhyCfg *EnetBoard_getPhyCfg(const EnetBoard_EthPort *ethPort)
{
    const EnetBoard_PortCfg *portCfg;

    portCfg = EnetBoard_getPortCfg(ethPort);

    return (portCfg != NULL) ? &portCfg->phyCfg : NULL;
}

static void EnetBoard_enableExternalMux()
{
    /* Enable the  External MUX selection. Applicable for AM263x LP E2 ver board.
     * Has no effect on AM263x LP E1 ver board.
     *
     * For AM263x-LP E2 board:
     *  - GPIO0_1 - RGMII1 MUX Enable PIN:
     *        HIGH to enable
     *        LOW  to disable
     *  - GPIO_58 - MII MUX Enable PIN:
     *        HIGH to enable
     *        LOW  to disable
     */

    uint32_t rgmii1MuxEnPinAddr = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_GPIO1_BASE_ADDR);
    uint32_t miiMuxEnPinAddr    = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_GPIO58_BASE_ADDR);

    GPIO_setDirMode(rgmii1MuxEnPinAddr, CONFIG_GPIO1_PIN, CONFIG_GPIO1_DIR);
    GPIO_setDirMode(miiMuxEnPinAddr, CONFIG_GPIO58_PIN, CONFIG_GPIO58_DIR);
    GPIO_pinWriteHigh(rgmii1MuxEnPinAddr, CONFIG_GPIO1_PIN);
    GPIO_pinWriteHigh(miiMuxEnPinAddr, CONFIG_GPIO58_PIN);
}

static const EnetBoard_PortCfg *EnetBoard_getPortCfg(const EnetBoard_EthPort *ethPort)
{
    const EnetBoard_PortCfg *portCfg = NULL;

    if (ENET_NOT_ZERO(ethPort->boardId & ENETBOARD_CPB_ID) ||
        ((portCfg == NULL) && ENET_NOT_ZERO(ethPort->boardId & ENETBOARD_LOOPBACK_ID)))
    {
        portCfg = EnetBoard_findPortCfg(ethPort,
                                        gEnetCpbBoard_am263x_cc_EthPort,
                                        ENETPHY_ARRAYSIZE(gEnetCpbBoard_am263x_cc_EthPort));
    }

    return portCfg;
}

static const EnetBoard_PortCfg *EnetBoard_findPortCfg(const EnetBoard_EthPort *ethPort,
                                                      const EnetBoard_PortCfg *ethPortCfgs,
                                                      uint32_t numEthPorts)
{
    const EnetBoard_PortCfg *ethPortCfg = NULL;
    bool found = false;
    uint32_t idx;

    for (idx = 0U; idx < numEthPorts; idx++)
    {
        ethPortCfg = &ethPortCfgs[idx];

        if ((ethPortCfg->enetType == ethPort->enetType) &&
            (ethPortCfg->instId == ethPort->instId) &&
            (ethPortCfg->macPort == ethPort->macPort) &&
            (ethPortCfg->mii.layerType == ethPort->mii.layerType) &&
            (ethPortCfg->mii.sublayerType == ethPort->mii.sublayerType))
        {
            found = true;
            break;
        }
    }

    return found ? ethPortCfg : NULL;
}

void EnetBoard_getMiiConfig(EnetMacPort_Interface *mii, const Enet_MacPort macPort)
{
    switch(macPort){
        case ENET_MAC_PORT_1:
            mii->layerType      = ENET_MAC_LAYER_GMII;
            mii->variantType    = ENET_MAC_VARIANT_FORCED;
            mii->sublayerType   = ENET_MAC_SUBLAYER_REDUCED;
            break;
        default:
            break;
    }
}

int32_t EnetBoard_setupPorts(EnetBoard_EthPort *ethPorts,
                             uint32_t numEthPorts)
{
    CSL_mss_ctrlRegs *mssCtrlRegs = (CSL_mss_ctrlRegs *)CSL_MSS_CTRL_U_BASE;

    DebugP_assert(numEthPorts == 1);

    EnetBoard_enableExternalMux();

    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, MSS_CTRL_PARTITION0);
    switch(ethPorts->macPort)
    {
        case ENET_MAC_PORT_1:
            CSL_FINS( mssCtrlRegs->CPSW_CONTROL,MSS_CTRL_CPSW_CONTROL_RGMII1_ID_MODE, 0U);
            CSL_FINS( mssCtrlRegs->CPSW_CONTROL,MSS_CTRL_CPSW_CONTROL_PORT1_MODE_SEL, MSS_CPSW_CONTROL_PORT_MODE_RGMII);
            break;
        case ENET_MAC_PORT_2:
            CSL_FINS( mssCtrlRegs->CPSW_CONTROL,MSS_CTRL_CPSW_CONTROL_RGMII2_ID_MODE, 0U);
            CSL_FINS( mssCtrlRegs->CPSW_CONTROL,MSS_CTRL_CPSW_CONTROL_PORT2_MODE_SEL, MSS_CPSW_CONTROL_PORT_MODE_RGMII);
            break;
        default:
            DebugP_assert(false);
    }
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, MSS_CTRL_PARTITION0);

    /* Nothing else to do */
    return ENET_SOK;
}

/*
 * Get ethernet board id
 */
uint32_t EnetBoard_getId(void)
{
    return ENETBOARD_AM263X_EVM;
}
