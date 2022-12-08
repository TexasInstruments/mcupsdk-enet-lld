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
 * \file  enet_utils.c
 *
 * \brief This file contains the implementation of the Enet Utils functionality.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include <enet_cfg.h>
#include <include/core/enet_utils.h>
#include <include/core/enet_osal.h>
#include <kernel/nortos/dpl/common/printf.h>
#include <kernel/dpl/CycleCounterP.h>
#ifdef __ARM_ACLE
#include <arm_acle.h>
#endif
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define __NOP  __nop()

#define NOP5   do { __NOP; __NOP; __NOP; __NOP; __NOP; } while (0)

#define NOP10  NOP5; \
               NOP5

#define NOP50  NOP10; \
               NOP10; \
               NOP10; \
               NOP10; \
               NOP10

/* Max value reported by CycleCounterP_getTimeStamp() */
#define ENET_UTILS_TIMESTAMP_MAX (0xFFFFFFFFULL)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/*!
 * \brief Enet Utils object.
 */
typedef struct EnetUtils_Obj_s
{
    /*! Print function pointer */
    Enet_Print print;

    /*! Virtual to physical translation function pointer */
    Enet_VirtToPhys virtToPhys;

    /*! Physical to virtual translation function pointer */
    Enet_PhysToVirt physToVirt;

    /*! Lock used in print functions */
    void *printLock;

    /*! Print buffer */
    char printBuf[ENET_CFG_PRINT_BUF_LEN];

} EnetUtils_Obj;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

EnetUtils_Obj gEnetUtilsObj;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void EnetUtils_init(const EnetUtils_Cfg *cfg)
{
    memset(&gEnetUtilsObj, 0, sizeof(gEnetUtilsObj));
    gEnetUtilsObj.printLock = EnetOsal_createMutex();

    gEnetUtilsObj.print = cfg->print;

    if (cfg->virtToPhys == NULL || cfg->physToVirt == NULL)
    {
        Enet_assert(false, "EnetUtils_init failed : EnetUtils_Cfg's virtToPhys or physToVirt is not initialized");

    }
    else
    {
        gEnetUtilsObj.virtToPhys = cfg->virtToPhys;
        gEnetUtilsObj.physToVirt = cfg->physToVirt;
    }
}

void EnetUtils_deinit(void)
{
    EnetOsal_deleteMutex(gEnetUtilsObj.printLock);
    memset(&gEnetUtilsObj, 0, sizeof(gEnetUtilsObj));
}

void EnetUtils_printf(const char *fmt,
                      ...)
{
#if defined(ENET_CFG_PRINT_ENABLE)
    va_list args;
    char *buf;

    if (gEnetUtilsObj.print != NULL)
    {
        EnetOsal_lockMutex(gEnetUtilsObj.printLock);

        buf = &gEnetUtilsObj.printBuf[0];
        va_start(args, fmt);
        vsnprintf(buf, ENET_CFG_PRINT_BUF_LEN, fmt, args);
        va_end(args);

        gEnetUtilsObj.print(buf);

        EnetOsal_unlockMutex(gEnetUtilsObj.printLock);
    }
#endif
}

void EnetUtils_vprintf(const char *fmt,
                       va_list args)
{
#if defined(ENET_CFG_PRINT_ENABLE)
    char *buf;

    if (gEnetUtilsObj.print != NULL)
    {
        EnetOsal_lockMutex(gEnetUtilsObj.printLock);

        buf = &gEnetUtilsObj.printBuf[0];
        vsnprintf(buf, ENET_CFG_PRINT_BUF_LEN, fmt, args);

        gEnetUtilsObj.print(buf);

        EnetOsal_unlockMutex(gEnetUtilsObj.printLock);
    }
#endif
}

uint32_t EnetUtils_min(uint32_t num1,
                       uint32_t num2)
{
    return ((num1 < num2) ? num1 : num2);
}

uint32_t EnetUtils_max(uint32_t num1,
                       uint32_t num2)
{
    return ((num1 > num2) ? num1 : num2);
}

void EnetUtils_delayTicks(const uint32_t delayTicks)
{
    uint64_t currentTick   = (uint64_t)CycleCounterP_getCount32();
    const uint64_t endTick = currentTick + (uint64_t)delayTicks;

    while (currentTick < endTick)
    {
        NOP50;
        currentTick = (uint64_t)CycleCounterP_getCount32();
        if ((currentTick < endTick) &&
            ((endTick - currentTick) > delayTicks))
        {
            currentTick += ENET_UTILS_TIMESTAMP_MAX + 1ULL;
        }
    }
    return;
}

void EnetUtils_delayNs(const uint32_t delayNs)
{
    uint64_t delayTicks = (uint64_t)CycleCounterP_nsToTicks(delayNs);
    uint64_t currentTick   = (uint64_t)CycleCounterP_getCount32();
    const uint64_t endTick = currentTick + (uint64_t)CycleCounterP_nsToTicks(delayNs);

    while (currentTick < endTick)
    {
        NOP50;
        currentTick = (uint64_t)CycleCounterP_getCount32();
        if ((currentTick < endTick) &&
            ((endTick - currentTick) > delayTicks))
        {
            currentTick += ENET_UTILS_TIMESTAMP_MAX + 1ULL;
        }
    }
    return;
}

uint64_t EnetUtils_virtToPhys(const void *virtAddr,
                              void *appData)
{
    return gEnetUtilsObj.virtToPhys(virtAddr, appData);
}

void *EnetUtils_physToVirt(uint64_t physAddr,
                           void *appData)
{
    return gEnetUtilsObj.physToVirt(physAddr, appData);
}

EnetPhy_Mii EnetUtils_macToPhyMii(const EnetMacPort_Interface *macMii)
{
    EnetPhy_Mii phyMii;

    if ((macMii->layerType == ENET_MAC_LAYER_MII) &&
        (macMii->sublayerType == ENET_MAC_SUBLAYER_STANDARD))
    {
        phyMii = ENETPHY_MAC_MII_MII;
    }
    else if ((macMii->layerType == ENET_MAC_LAYER_MII) &&
             (macMii->sublayerType == ENET_MAC_SUBLAYER_REDUCED))
    {
        phyMii = ENETPHY_MAC_MII_RMII;
    }
    else if ((macMii->layerType == ENET_MAC_LAYER_GMII) &&
             (macMii->sublayerType == ENET_MAC_SUBLAYER_STANDARD))
    {
        phyMii = ENETPHY_MAC_MII_GMII;
    }
    else if ((macMii->layerType == ENET_MAC_LAYER_GMII) &&
             (macMii->sublayerType == ENET_MAC_SUBLAYER_REDUCED))
    {
        phyMii = ENETPHY_MAC_MII_RGMII;
    }
    else if ((macMii->layerType == ENET_MAC_LAYER_GMII) &&
             (macMii->sublayerType == ENET_MAC_SUBLAYER_SERIAL))
    {
        phyMii = ENETPHY_MAC_MII_SGMII;
    }
    else if ((macMii->layerType == ENET_MAC_LAYER_GMII) &&
             ((macMii->sublayerType == ENET_MAC_SUBLAYER_QUAD_SERIAL_MAIN) ||
              (macMii->sublayerType == ENET_MAC_SUBLAYER_QUAD_SERIAL_SUB)))
    {
        phyMii = ENETPHY_MAC_MII_QSGMII;
    }
    else
    {
        Enet_assert(false,
                    "ENETPHY doesn't support MAC interface layer %u sublayer %u\n",
                    macMii->layerType, macMii->sublayerType);
        phyMii = ENETPHY_MAC_MII_RGMII;
    }

    return phyMii;
}

