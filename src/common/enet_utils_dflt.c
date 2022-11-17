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
 * \file  enet_utils_dflt.c
 *
 * \brief This file contains a default implementation of the Enet Utils.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <assert.h>
#include <kernel/dpl/ClockP.h>
#include <include/common/enet_utils_dflt.h>
#include <drivers/soc.h>
#include <kernel/nortos/dpl/common/printf.h>

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

static void EnetUtilsDflt_print(const char *fmt, ...);

static uint64_t EnetUtilsDflt_virtToPhysDflt(const void *virtAddr,
                                             void *appData);

static void *EnetUtilsDflt_physToVirtDflt(uint64_t phyAddr,
                                          void *appData);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void EnetUtilsDflt_initCfg(EnetUtils_Cfg *cfg)
{
    cfg->print = EnetUtilsDflt_print;
    cfg->virtToPhys = &EnetUtilsDflt_virtToPhysDflt;
    cfg->physToVirt = &EnetUtilsDflt_physToVirtDflt;
}

static void EnetUtilsDflt_print(const char *fmt, ...)
{
    char buf[ENET_CFG_PRINT_BUF_LEN];
    va_list args;

#if ENET_CFG_IS_ON(DEV_ERROR)
    if (ENET_CFG_PRINT_BUF_LEN < strlen(fmt))
    {
        assert(false);
    }
#endif

    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    DebugP_log("%s",buf);
    va_end(args);
}

static uint64_t EnetUtilsDflt_virtToPhysDflt(const void *virtAddr,
                                             void *appData)
{
    return ((uint64_t)SOC_virtToPhy((void *)virtAddr));
}

static void *EnetUtilsDflt_physToVirtDflt(uint64_t phyAddr,
                                          void *appData)
{

    return ((void*) SOC_phyToVirt(phyAddr));

}
