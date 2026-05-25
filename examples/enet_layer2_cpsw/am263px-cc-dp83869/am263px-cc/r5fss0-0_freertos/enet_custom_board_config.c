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
#include <generic_phy.h>
#include <networking/enet/core/src/phy/enetphy_priv.h>
#include "ti_board_open_close.h"
#include <kernel/dpl/AddrTranslateP.h>

#include <ti_drivers_config.h>
#include <drivers/i2c.h>
#include <board/ioexp/ioexp_tca6424.h>
#include <board/ioexp/ioexp_tca6416.h>

static void EnetBoard_setMacPort2IOExpanderCfg(void);

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

#define MSS_CPSW_CONTROL_PORT_MODE_MII                                    (0x0U)
#define MSS_CPSW_CONTROL_PORT_MODE_RMII                                   (0x1U)
#define MSS_CPSW_CONTROL_PORT_MODE_RGMII                                  (0x2U)

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

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/*!
 * \brief Common Processor Board (CPB) board's DP83869 PHY configuration.
 */
static const Dp83869_Cfg gEnetCpbBoard_ConfigEnetEthphy1PhyCfg =
{
	.txClkShiftEn         = true,
    .rxClkShiftEn         = true,
    .txDelayInPs          = 2000U,   /* Value in pecosec. Refer to DLL_RX_DELAY_CTRL_SL field in ANA_RGMII_DLL_CTRL register of DP83869 PHY datasheet */
    .rxDelayInPs          = 2000U,   /* Value in pecosec. Refer to DLL_TX_DELAY_CTRL_SL field in ANA_RGMII_DLL_CTRL register of DP83869 PHY datasheet */
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
 * am263px-cc board configuration.
 *
 * RMII/RGMII PHY connected to am263px-cc CPSW_3G MAC port.
 */
static const EnetBoard_PortCfg gEnetCpbBoard_am263px_cc_EthPort[] =
{
    {
        /* "CPSW3G" */
        .enetType = ENET_CPSW_3G,
        .instId   = 0U,
        .macPort  = ENET_MAC_PORT_2,
        .mii      = {ENET_MAC_LAYER_GMII, ENET_MAC_SUBLAYER_REDUCED},
        .phyCfg   =
        {
            .phyAddr         = 0,
            .isStrapped      = false,
            .skipExtendedCfg = false,
            .extendedCfg     = &gEnetCpbBoard_ConfigEnetEthphy1PhyCfg,
            .extendedCfgSize = sizeof(gEnetCpbBoard_ConfigEnetEthphy1PhyCfg)
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

static const EnetBoard_PortCfg *EnetBoard_getPortCfg(const EnetBoard_EthPort *ethPort)
{
    const EnetBoard_PortCfg *portCfg = NULL;

    if (ENET_NOT_ZERO(ethPort->boardId & ENETBOARD_CPB_ID) ||
        ((portCfg == NULL) && ENET_NOT_ZERO(ethPort->boardId & ENETBOARD_LOOPBACK_ID)))
    {
        portCfg = EnetBoard_findPortCfg(ethPort,
                                        gEnetCpbBoard_am263px_cc_EthPort,
                                        ENETPHY_ARRAYSIZE(gEnetCpbBoard_am263px_cc_EthPort));
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
        case ENET_MAC_PORT_2:
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

    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, MSS_CTRL_PARTITION0);

    switch(ethPorts->macPort)
    {
        case ENET_MAC_PORT_2:
            CSL_FINS( mssCtrlRegs->CPSW_CONTROL,MSS_CTRL_CPSW_CONTROL_RGMII2_ID_MODE, 0U);
            CSL_FINS( mssCtrlRegs->CPSW_CONTROL,MSS_CTRL_CPSW_CONTROL_PORT2_MODE_SEL, MSS_CPSW_CONTROL_PORT_MODE_RGMII);
            EnetBoard_setMacPort2IOExpanderCfg();
            break;
        default:
            DebugP_assert(false);
    }
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, MSS_CTRL_PARTITION0);

    /* Nothing else to do */
    return ENET_SOK;
}

static void EnetBoard_setMacPort2IOExpanderCfg(void)
{
    int32_t status    = SystemP_SUCCESS;

    const uint32_t mdioMuxSel1Line  = 8*2 + 2; // PORT 2, PIN 2
    const uint32_t mdioMuxSel2Line  = 8*2 + 3; // PORT 2, PIN 3
    const uint32_t rgmii0MuxSelLine = 8*2 + 4; // PORT 2, PIN 4
    const uint32_t rgmii1MuxSelLine = 8*0 + 2; // PORT 0, PIN 2
    const uint32_t rgmii2MuxSelLine = 8*0 + 3; // PORT 0, PIN 3
    static TCA6424_Config  gTCA6424_Config;

    TCA6424_Params      TCA6424Params;
    TCA6424_Params_init(&TCA6424Params);

    status = TCA6424_open(&gTCA6424_Config, &TCA6424Params);

    if (status == SystemP_SUCCESS)
    {
        /* Configure as output  */
        status = TCA6424_config(&gTCA6424_Config,
                    mdioMuxSel1Line,
                    TCA6424_MODE_OUTPUT);
    }
    if (status == SystemP_SUCCESS)
    {
        status =  TCA6424_setOutput(&gTCA6424_Config, mdioMuxSel1Line, TCA6424_OUT_STATE_HIGH);
    }

    if (status == SystemP_SUCCESS)
    {
        /* Configure as output  */
        status = TCA6424_config(&gTCA6424_Config,
                    mdioMuxSel2Line,
                    TCA6424_MODE_OUTPUT);
    }
    if (status == SystemP_SUCCESS)
    {
        status =  TCA6424_setOutput(&gTCA6424_Config, mdioMuxSel2Line, TCA6424_OUT_STATE_HIGH);
    }

    if (status == SystemP_SUCCESS)
    {
        /* Configure as output  */
        status = TCA6424_config(&gTCA6424_Config,
                    rgmii0MuxSelLine,
                    TCA6424_MODE_OUTPUT);
    }
    if (status == SystemP_SUCCESS)
    {
        status =  TCA6424_setOutput(&gTCA6424_Config, rgmii0MuxSelLine, TCA6424_OUT_STATE_LOW);
    }

    if (status == SystemP_SUCCESS)
    {
        /* Configure as output  */
        status = TCA6424_config(&gTCA6424_Config,
                    rgmii1MuxSelLine,
                    TCA6424_MODE_OUTPUT);
    }
    if (status == SystemP_SUCCESS)
    {
        status =  TCA6424_setOutput(&gTCA6424_Config, rgmii1MuxSelLine, TCA6424_OUT_STATE_HIGH);
    }

    if (status == SystemP_SUCCESS)
    {
        /* Configure as output  */
        status = TCA6424_config(&gTCA6424_Config,
                    rgmii2MuxSelLine,
                    TCA6424_MODE_OUTPUT);
    }
    if (status == SystemP_SUCCESS)
    {
        status =  TCA6424_setOutput(&gTCA6424_Config, rgmii2MuxSelLine, TCA6424_OUT_STATE_LOW);
    }
}

/*
 * Get ethernet board id
 */
uint32_t EnetBoard_getId(void)
{
    return ENETBOARD_AM263PX_EVM;
}

