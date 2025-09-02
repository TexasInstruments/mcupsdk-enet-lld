/*
 *  Copyright (C) Texas Instruments Incorporated 2025
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
 * \file  etherring_can_app.c
 *
 * \brief This file contains the ethernet tx,rx Application tasks
 */
/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */
#include "etherring_can_app.h"
#include "etherring_mcan.h"
/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */
#define ENETAPP_AVTP_STREAMID                                               (0U)
#define ENETAPP_CAN_BUSID                                                   (0U)
#define ENETAPP_TX_TASK_PRIORITY                                            (4U)
#define ENETAPP_RX_TASK_PRIORITY                                            (3U)

/* ========================================================================== */
/*                              Global Variables                              */
/* ========================================================================== */
CanTrafficGen_Object gTgConfig;
static SemaphoreP_Object gTxSemObj;
static EtherRing_Cfg gEtherRingCfg;
static uint8_t gEnetAppTaskStackTxApp[ENETAPP_TASK_STACK_SZ] __attribute__((aligned(32)));
static uint8_t gEnetAppTaskStackRxApp[ENETAPP_TASK_STACK_SZ] __attribute__((aligned(32)));
static uint32_t rxCanPktCounter = 0;
static EtherringMcanObj gEtherringMcanObj;
/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */
static void EnetApp_txTask(void* args);
static void EnetApp_rxTask(void *args);
static int32_t EnetApp_setupConfig(CanTrafficGen_Object *tgCfg, EnetApp_Cfg *enetAppCfg);
static int32_t EnetApp_createTxTask(EnetApp_Cfg *enetAppCfg);
static int32_t EnetApp_createRxTask(EnetApp_Cfg *enetAppCfg);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
static void EnetApp_initEtherRingCfg(EtherRing_Cfg *etherRingCfg, uint8_t hostMacAddLastByte)
{
    /* Last byte of Host Mac address is stored to add it in CB Header*/
    etherRingCfg->hostMacAddLastByte = hostMacAddLastByte;
    etherRingCfg->isCfg = true;
}

int32_t EnetApp_initApp(EnetApp_Cfg *enetAppCfg)
{
    int32_t status = ENET_SOK;
#ifdef ETHERRING_MCAN_ENABLE_PROFILING
    uint32_t memPoolIndex = 0U;
#endif

    /* Open DMA channels for TX and RX */
    status = EnetApp_openDma(enetAppCfg);

    /* Initialize EtherRing configuration with host MAC address */
    EnetApp_initEtherRingCfg(&gEtherRingCfg, enetAppCfg->macAddr[ENET_MAC_ADDR_LEN - 1]);

    /* Initialize EtherRing module */
    status = EnetApp_etherRingInit(&gEtherRingCfg, enetAppCfg);
    EnetAppUtils_assert(status == ENET_SOK);

    /* Adding vlan entry for Ether-Ring stream traffic */

    /* Configure node-specific multicast address */
    EnetApp_configureNodeMcastAddress(enetAppCfg->hEnet, enetAppCfg->coreId, enetAppCfg->nodeId);

    /* Add broadcast entry to ALE table */
    EnetApp_addBCastEntry(enetAppCfg->enetType,
                         enetAppCfg->instId,
                         EnetSoc_getCoreId());

#ifdef ETHERRING_MCAN_ENABLE_PROFILING
    /* Initialize the queue for managing CAN packet memory */
    EnetQueue_initQ(&enetAppCfg->timestampFreeQ);
    EnetQueue_initQ(&enetAppCfg->timestampReadyQ);

    /* Enqueue all available memory blocks to the free queue */
    for (memPoolIndex = 0; memPoolIndex < ETHERRING_CAN_TIMESTAMP_POOL_COUNT; memPoolIndex++)
    {
        if (&enetAppCfg->timestampPool[memPoolIndex] != NULL)
        {
            EnetQueue_enq(&enetAppCfg->timestampFreeQ, &enetAppCfg->timestampPool[memPoolIndex].node);
        }
    }

    /* Verify all memory blocks were successfully enqueued */
    if (EnetQueue_getQCount(&enetAppCfg->timestampFreeQ) != ETHERRING_CAN_TIMESTAMP_POOL_COUNT)
    {
        status = ENET_EFAIL;
    }
#endif

    /* Setup CAN traffic generator configuration */
    status = EnetApp_setupConfig(&gTgConfig, enetAppCfg);
    EnetAppUtils_assert(status == ENET_SOK);
    return status;
}

static int32_t EnetApp_setupConfig(CanTrafficGen_Object *tgCfg, EnetApp_Cfg *enetAppCfg)
{
    int32_t status = ENET_SOK;

    /* Timer tick is dependent on Application configuration. This needs to be
    *  updated if timer tick periodicity is changed */
    tgCfg->timerTickPeriodicityInus = 500;

    /* Configure a single traffic pattern */
    tgCfg->validPktParamsCount = 1;
    tgCfg->pktParams[0].CAN_msgId = 0x2;
    tgCfg->pktParams[0].packetCountPerTick = 1;
    tgCfg->pktParams[0].payloadSize = 64;
    tgCfg->pktParams[0].periodicity = 500;
    tgCfg->pktParams[0].packetCounter = 0;
    tgCfg->pktParams[0].maxPacketSend = ETHERRING_MCAN_TEST_PKT_COUNT;
    tgCfg->pktParams[0].hasAllCanPktsSent = false;

    /* Initialize traffic generator */
    status = CanTrafficGen_setup(tgCfg);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("CAN Traffic Gen setup failed\r\n");
        status = ENET_EFAIL;
    }
    else
    {
#ifdef ETHERRING_MCAN_ENABLE_PROFILING
        tgCfg->timestampFreeQPtr = &enetAppCfg->timestampFreeQ;
        tgCfg->timestampReadyQPtr = &enetAppCfg->timestampReadyQ;
        tgCfg->hEnet = enetAppCfg->hEnet;
        tgCfg->coreId = enetAppCfg->coreId;
#endif
    }

    /* Initialize CAN-Ethernet gateway */
    if (status == ENET_SOK)
    {
        status = Gateway_setup();
    }

    /* Configure MCAN Driver */
    if (status == ENET_SOK)
    {
        EtherringCAN_mcanConfig(&gEtherringMcanObj);
#ifdef ETHERRING_MCAN_ENABLE_PROFILING
        gEtherringMcanObj.timestampFreeQPtr = &enetAppCfg->timestampFreeQ;
        gEtherringMcanObj.timestampReadyQPtr = &enetAppCfg->timestampReadyQ;
        gEtherringMcanObj.hEnet = enetAppCfg->hEnet;
        gEtherringMcanObj.coreId = enetAppCfg->coreId;
#endif
    }

    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Gateway setup failed\r\n");
    }

    return status;
}

void EnetApp_startTraffic(EnetApp_Cfg *enetAppCfg)
{
    int32_t status = ENET_SOK;

    /* Wait for link up before starting traffic */
    if (status == ENET_SOK)
    {
        status = EnetApp_waitForLinkUp(2, enetAppCfg->macPorts, enetAppCfg->hEnet, enetAppCfg->coreId);
    }

    /* Create RX task to receive packets */
    status = EnetApp_createRxTask(enetAppCfg);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Ethernet RX Task creation failed\r\n");
        status = ENET_EFAIL;
    }
    EnetAppUtils_print("Ethernet RX Task created\r\n");

    if (status == ENET_SOK)
    {
        status = EtherringCAN_createRxCANTask(&gEtherringMcanObj);
    }

    if (status != ENET_SOK)
    {
        EnetAppUtils_print("MCAN RX Task create failed\r\n");
    }

    EnetAppUtils_assert(status == ENET_SOK);
    /* Generate CAN traffic only from Node0 */
    if (enetAppCfg->nodeId == 0U)
    {
        /* Create TX task for packet transmission */
        status = EnetApp_createTxTask(enetAppCfg);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("TX Task creation failed\r\n");
            status = ENET_EFAIL;
        }
        else
        {
            EnetAppUtils_print("Ethernet TX Task created\r\n");
        }

        EnetAppUtils_assert(status == ENET_SOK);
        EnetAppUtils_print("Generating CAN Traffic\r\n");

        /* Start periodic timer for traffic generation */
        TimerP_start(gTimerBaseAddr[CONFIG_TIMER0]);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Timer start failed\r\n");
        }
    }

}

static void EnetApp_retreiveErTxPktQ(EtherRing_Handle hEtherRing, EnetDma_PktQ* freePktInfoQ)
{
    EnetDma_PktQ txFreeQ;
    int32_t retVal;

    /* Initialize queue for completed TX packets */
    EnetQueue_initQ(&txFreeQ);

    /* Retrieve packets that have completed transmission */
    retVal = EtherRing_retrieveTxPktQ(hEtherRing, &txFreeQ);
    if (retVal == ENET_SOK)
    {
        EnetDma_Pkt *ethPktInfo;

        /* Process each completed packet */
        ethPktInfo = (EnetDma_Pkt *)EnetQueue_deq(&txFreeQ);
        while (NULL != ethPktInfo)
        {
            /* Validate packet state transition */
            EnetDma_checkPktState(&ethPktInfo->pktState,
                                ENET_PKTSTATE_MODULE_APP,
                                ENET_PKTSTATE_APP_WITH_DRIVER,
                                ENET_PKTSTATE_APP_WITH_FREEQ);

            /* Return packet to free queue */
            EnetQueue_enq(freePktInfoQ, &ethPktInfo->node);
            ethPktInfo = (EnetDma_Pkt *)EnetQueue_deq(&txFreeQ);
        }
    }
}

static void EnetApp_txTask(void* args)
{
    EnetApp_Cfg* enetAppCfg = (EnetApp_Cfg*)args;

    while(true)
    {
        /* Wait for timer trigger */
        SemaphoreP_pend(&gTxSemObj, SystemP_WAIT_FOREVER);

        while(true)
        {
            if (enetAppCfg->isTestPassPrinted == false &&
                gTgConfig.canGeneratedPktCount == ETHERRING_MCAN_TEST_PKT_COUNT &&
                gEtherringMcanObj.mcanStats.rxCanPacketCount == ETHERRING_MCAN_TEST_PKT_COUNT)
            {
                EnetAppUtils_print("EtherRing CAN Test Passed\r\n");
                enetAppCfg->isTestPassPrinted = true;
            }
            /* Check if we have CAN packets and free TX buffers */
            if(EnetQueue_getQCount(&gTgConfig.rxReadyElementQ) > 0 &&
               EnetQueue_getQCount(&enetAppCfg->txFreePktInfoQ) > 0)
            {
                /* Get next CAN packet to transmit */
                CAN_pkt *rxCanPkt = (CAN_pkt *)EnetQueue_deq(&gTgConfig.rxReadyElementQ);
                if(rxCanPkt == NULL)
                {
                    continue;
                }

                MCAN_RxBufElement* rxElement = &rxCanPkt->canElement;
                if (rxElement != NULL)
                {
                    /* Get a free Ethernet packet buffer */
                    EnetDma_Pkt* ethPktInfo = (EnetDma_Pkt*)EnetQueue_deq(&enetAppCfg->txFreePktInfoQ);
                    if (ethPktInfo == NULL)
                    {
                        continue;
                    }

                    /* Convert CAN frame to AVTP packet */
                    if(Gateway_convertCanToAvtpPacket(rxElement, ethPktInfo,
                                                    ENETAPP_AVTP_STREAMID, enetAppCfg->macAddr, ENETAPP_CAN_BUSID) == ENET_SOK)
                    {
                        /* Free the rxCanPkt after encapsulation */
                        CanTrafficGen_memFree(rxCanPkt);

                        EnetDma_PktQ txSubmitQ;
                        EnetQueue_initQ(&txSubmitQ);

                        /* Enqueue the packet for transmission */
                        EnetQueue_enq(&txSubmitQ, &ethPktInfo->node);

                        /* Submit packet for transmission */
                        int32_t status = EtherRing_submitTxPktQ(enetAppCfg->hEtherRing, &txSubmitQ);
                        if (status == ENET_SOK)
                        {
                            /* Retrieve completed TX packets */
                            EnetApp_retreiveErTxPktQ(enetAppCfg->hEtherRing, &enetAppCfg->txFreePktInfoQ);
                        }
                        else
                        {
                            EnetAppUtils_print("etherring submit failed\r\n");
                        }
                    }
                    else
                    {
                        EnetAppUtils_print("CAN to ETH encap failed\r\n");
                    }
                }
                else
                {
                    EnetAppUtils_print("invalid rxElement\r\n");
                }
            }
            else
            {
                /* No CAN packets or TX buffers available, check for completed packets */
                EnetApp_retreiveErTxPktQ(enetAppCfg->hEtherRing, &enetAppCfg->txFreePktInfoQ);
            }
        }
    }

    TaskP_exit();
}

static void EnetApp_rxTask(void *args)
{
    EnetDma_PktQ rxReadyQ;
    EnetDma_PktQ rxSubmitQ;
    EnetDma_Pkt *ethPktInfo = NULL;
    int32_t status = ENET_SOK;
    EnetApp_Cfg* enetAppCfg = (EnetApp_Cfg*)args;

    while ((ENET_SOK == status))
    {
        /* Wait for packet reception */
        SemaphoreP_pend(&enetAppCfg->rxSemObj, SystemP_WAIT_FOREVER);

        /* Initialize packet queues */
        EnetQueue_initQ(&rxReadyQ);
        EnetQueue_initQ(&rxSubmitQ);

        /* Get the packets received so far */
        status = EtherRing_retrieveRxPktQ(enetAppCfg->hEtherRing, &rxReadyQ);

        /* Process each received packet */
        ethPktInfo = (EnetDma_Pkt*) EnetQueue_deq(&rxReadyQ);
        while(ethPktInfo)
        {
            /* Add packet to submission queue to be reused */
            EnetQueue_enq(&rxSubmitQ, &ethPktInfo->node);

            /* Convert AVTP packet to CAN frame */
            MCAN_TxBufElement txCanElement;
            Gateway_convertAvtpToCanPacket(ethPktInfo, &txCanElement);

            /* Fetching the CAN msgId in CAN element  */
            txCanElement.id = ((txCanElement.id  & 0x7FFU) << 18U);
            rxCanPktCounter++;

            if (rxCanPktCounter % ETHERRING_MCAN_TEST_PKT_COUNT == 0U)
            {
#ifndef ETHERRING_MCAN_ENABLE_PROFILING
                EnetAppUtils_print("[Ethernet Rx App] Received %u EtherRing CAN packets to"
                                   " Rx Eth task\r\n", rxCanPktCounter);
#endif
            }
            /* Sending CAN frame using MCAN driver*/
            EtherringCAN_sendCANPacket(&txCanElement, &gEtherringMcanObj);

            /* Dequeue next packet */
            ethPktInfo = (EnetDma_Pkt*) EnetQueue_deq(&rxReadyQ);
        }

        /* Submit processed buffers back to hardware for reuse */
        EtherRing_submitRxPktQ(enetAppCfg->hEtherRing, &rxSubmitQ);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to submit RX pkt queue: %d\r\n", status);
            Enet_assert(false);
        }
    }

    TaskP_exit();
}

void EnetApp_periodicTimerCb(void *args)
{
    (void)args;
    /* Release sem for traffic generator and TX task */
    SemaphoreP_post(&gTgConfig.tgSemObj);
    SemaphoreP_post(&gTxSemObj);
}

static int32_t EnetApp_createTxTask(EnetApp_Cfg *enetAppCfg)
{
    TaskP_Params taskParams;
    int32_t status;

    /* Create semaphore for signaling TX task */
    status = SemaphoreP_constructBinary(&gTxSemObj, 0);
    EnetAppUtils_assert(status == ENET_SOK);

    /* Initialize task parameters */
    TaskP_Params_init(&taskParams);
    taskParams.priority       = ENETAPP_TX_TASK_PRIORITY;
    taskParams.stack         = gEnetAppTaskStackTxApp;
    taskParams.stackSize     = sizeof(gEnetAppTaskStackTxApp);
    taskParams.args          = (void*)enetAppCfg;
    taskParams.name          = "TX Task";
    taskParams.taskMain      = &EnetApp_txTask;

    /* Create the TX task */
    status = TaskP_construct(&enetAppCfg->txTaskObj, &taskParams);
    return status;
}

static int32_t EnetApp_createRxTask(EnetApp_Cfg *enetAppCfg)
{
    TaskP_Params taskParams;
    int32_t status = ENET_SOK;

    DebugP_assert(ENET_SOK == status);

    /* Initialize task parameters */
    TaskP_Params_init(&taskParams);
    taskParams.priority       = ENETAPP_RX_TASK_PRIORITY;
    taskParams.stack          = gEnetAppTaskStackRxApp;
    taskParams.stackSize      = sizeof(gEnetAppTaskStackRxApp);
    taskParams.args           = (void*)enetAppCfg;
    taskParams.name           = "Rx Task";
    taskParams.taskMain       = &EnetApp_rxTask;

    /* Create the RX task */
    status = TaskP_construct(&enetAppCfg->rxTaskObj, &taskParams);
    DebugP_assert(ENET_SOK == status);

    return status;
}
