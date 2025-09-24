/*
 *  Copyright (c) Texas Instruments Incorporated 2025
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
 * \file  app_main.c
 *
 * \brief This file contains the main task of Raw (without LwIP) UDP application.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <enet_apputils.h>
#include <kernel/dpl/ClockP.h>

#include "app_enethandler.h"
#include "udp_test.h"

#include "ti_enet_config.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define NUM_ITERATIONS    (100U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */
       
/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void EnetApp_mainTask(void *args)
{
    uint32_t i;
    int32_t status=ENET_SOK;
    Enet_MacPort macPortList[ENET_MAC_PORT_NUM];
    uint8_t numMacPorts;

    /* Initialize loopback test config */
    EnetApp_cfg *pEnetApp = EnetApp_getAppCfg();

    memset(pEnetApp, 0, sizeof(*pEnetApp));

    EnetApp_getEnetInstInfo(CONFIG_ENET_CPSW0, &pEnetApp->enetType,
                                &pEnetApp->instId);

    EnetApp_getEnetInstMacInfo(pEnetApp->enetType,
                                pEnetApp->instId,
                                macPortList,
                                &numMacPorts);

    EnetAppUtils_assert(numMacPorts == 1);
    pEnetApp->macPort = macPortList[0];

    for (i = 0U; i < NUM_ITERATIONS; i++)
    {
        EnetAppUtils_print("=============================\r\n");
        EnetAppUtils_print(" Enet Raw UDP : Iteration %u \r\n", i + 1);
        EnetAppUtils_print("=============================\r\n");

        status |= EnetApp_rawUdpTest();

        ClockP_usleep(100000);
    }

    if (status == ENET_SOK)
    {
        EnetAppUtils_print("Raw UDP application completed\r\n");
        EnetAppUtils_print("All tests have passed!!\r\n");
    }
    else
    {
        EnetAppUtils_print("Raw UDP  application failed to complete\r\n");
    }

    return;
}



