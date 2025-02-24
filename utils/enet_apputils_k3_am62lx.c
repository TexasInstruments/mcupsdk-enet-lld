/*
 *  Copyright (C) Texas Instruments Incorporated 2020-2024
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
 * \file     enet_apputils_k3.c
 *
 * \brief    Common Enet application utility functions for K3 SOCs.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <drivers/hw_include/cslr_soc.h>
#include <csl_cpswitch.h>

#include <enet.h>
#include <enet_cfg.h>
#include <include/per/cpsw.h>

#include <drivers/uart.h>


#include "include/enet_apputils.h"
#include "include/enet_appboardutils.h"

#include "include/enet_appsoc.h"
#include "include/enet_apprm.h"

//TODO - private dependency
#include <priv/mod/cpsw_clks.h>
#include <kernel/dpl/SystemP.h>

#include <drivers/soc.h>
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



EnetAppUtils_MmrLockState EnetAppUtils_mcuMmrCtrl(EnetAppUtils_CtrlMmrType mmrNum,
                                                  EnetAppUtils_MmrLockState lock)
{
    EnetAppUtils_MmrLockState prevLockState = ENETAPPUTILS_LOCK_MMR;
    return prevLockState;

}

EnetAppUtils_MmrLockState EnetAppUtils_mainMmrCtrl(EnetAppUtils_CtrlMmrType mmrNum,
                                                   EnetAppUtils_MmrLockState lock)
{
 
    EnetAppUtils_MmrLockState prevLockState = ENETAPPUTILS_LOCK_MMR;
    return prevLockState;

}


#if (ENET_ENABLE_PER_CPSW == 1)
static void EnetAppUtils_selectCptsClock(Enet_Type enetType,
                                         EnetAppUtils_CptsClkSelMux clkSelMux)
{
    uint32_t muxVal;
    muxVal = (uint32_t) clkSelMux;
#if defined(SOC_AM62LX)

    switch (enetType)
    {
        case ENET_CPSW_3G:
        {
            CSL_cptsRegs *regs = NULL;

            regs = (CSL_cptsRegs *)(uintptr_t)(CSL_CPSW0_CPSW_NUSS_VBUSP_BASE +
                                               CPSW_CPTS_OFFSET);
            CSL_CPTS_setRFTCLKSelectReg(regs, muxVal);
        }
        break;

        default:
            EnetAppUtils_assert(false);
            break;
    }
#else
#error "Unsupported platform"
#endif
}
#endif

#define CPSW_SOC_RGMII_MHZ_250_CLK_VAL        (250000000U)
#define CPSW_SOC_RGMII_MHZ_50_CLK_VAL         (50000000U)
#define CPSW_SOC_RGMII_MHZ_5_CLK_VAL          (5000000U)

void EnetAppUtils_enableClocks(Enet_Type enetType, uint32_t instId)
{
    uint32_t moduleId = AM62LX_DEV_CPSW0;;
    uint32_t enableClock = 1;
    SOC_moduleClockEnable(moduleId, enableClock);

#if (ENET_ENABLE_PER_CPSW == 1)
    if (Enet_isCpswFamily(enetType))
    {
        EnetAppUtils_CptsClkSelMux clkSelMux;
        clkSelMux = ENETAPPUTILS_CPTS_CLKSEL_CPSWHSDIV_CLKOUT2;
        EnetAppUtils_selectCptsClock(enetType, clkSelMux);
    }
#endif
}

void EnetAppUtils_disableClocks(Enet_Type enetType, uint32_t instId)
{
    uint32_t moduleId = AM62LX_DEV_CPSW0;
    uint32_t enableClock = 0;
    SOC_moduleClockEnable(moduleId, enableClock);
}


int32_t EnetAppUtils_setTimeSyncRouter(Enet_Type enetType, uint32_t instId, uint32_t input, uint32_t output)
{
    int32_t  status = ENET_SOK;

#if defined(SOC_AM64X) || defined(SOC_AM243X) || defined(SOC_AM62AX) || defined(SOC_AM62PX) || defined(SOC_AM62DX) || defined(SOC_AM62X) || defined(SOC_AM62LX) || defined(SOC_AM275X)
    EnetAppUtils_assert(enetType == ENET_CPSW_3G);
#endif

    return status;
}

void EnetAppUtils_setupSciServer(void)
{
    return;
}

/* end of file */
