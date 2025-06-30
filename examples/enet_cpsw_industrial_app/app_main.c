/*
 * Copyright (C) 2025 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*!
 * \file  app_main.c
 *
 * \brief This file contains the implementation of the entry point to Enet industrial application
 */
/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

/* Including application header files */
#include "app_cfg.h"
#include "app_ethcfg/app_ethcfg.h"
#include "app_rawudp_handler.h"
#include "app_tcpserver.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

int EnetApp_main();

void EnetApp_configureNodeId(uint8_t *nodeId,uint8_t* numMacPort);

void EnetApp_setStaticAddr(EnetApp_TcpObj *hTcp,uint8_t *nodeId);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

EnetApp_Obj gEnetApp;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */


int EnetApp_main()
{

    DebugP_log("=======================================================\r\n");
    DebugP_log("            CPSW Industrial Application                \r\n");
    DebugP_log("=======================================================\r\n");


    /* Set ID of current node */
    EnetApp_configureNodeId(&gEnetApp.nodeId, &gEnetApp.numMacPort);

    /* Configure peripherals and open driver*/
    EthCfg_init(&gEnetApp);

    /* Setup real-time channel */
    RawUdpApp_setup(&gEnetApp);

    /* Set static IP address for TCP server */
    EnetApp_setStaticAddr(&gEnetApp.hTcp, &gEnetApp.nodeId);

    /* Setup TCP server */
    TcpApp_setup(&gEnetApp.hTcp);

    while(true){
        ClockP_usleep(5000);
        TaskP_yield();
    }

    return 0;
}

void EnetApp_configureNodeId(uint8_t *nodeId,uint8_t* numMacPort) {
    while (true) {
        EnetAppUtils_print("Enter the nodeId between 1 to %d: \r\n", ENETAPP_NODE_COUNT);

        DebugP_scanf("%d", (int*)nodeId);
        if ((*nodeId < 1) || (*nodeId > ENETAPP_NODE_COUNT)) {
            EnetAppUtils_print("Enter a valid number for nodeId\r\n");
            continue;
        }
        EnetAppUtils_print("NodeId: %d\r\n", *nodeId);
        if(*nodeId == ENETAPP_NODE_COUNT){
            *numMacPort = 1;
        }
        else{
            *numMacPort = 2;
        }
        break;
    }
}

void EnetApp_setStaticAddr(EnetApp_TcpObj* hTcp, uint8_t *nodeId){
    gEnetApp.hTcp.staticIP[0] = ENETAPP_STATIC_IP_ADDR(*nodeId);
    gEnetApp.hTcp.staticIPGateway[0] = ENETAPP_STATIC_IP_GATEWAY;
    gEnetApp.hTcp.staticIPNetmask[0] = ENETAPP_STATIC_IP_NETMASK;
}