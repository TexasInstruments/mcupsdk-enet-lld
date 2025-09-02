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
 * \file  can_trafficgen.c
 *
 * \brief This file contains the CAN traffic generator
 */
/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */
#include "can_trafficgen.h"

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */
#define ENETAPP_TG_TASK_PRIORITY                                            5U

/* ========================================================================== */
/*                              Global Variables                              */
/* ========================================================================== */
/* Task stack allocation */
uint8_t gEnetAppTaskStackTg[ENETAPP_TASK_STACK_SZ] __attribute__((aligned(32)));

/* Traffic-Gen Task Object */
TaskP_Object gTgTaskObj;
/* ========================================================================== */
/*                                 Structures                                 */
/* ========================================================================== */
/* Gateway memory pool structure */
typedef struct
{
    /* Empty Rx CAN Element Queue */
    EnetQ freeQueue;
    /* Flag to check If mempool is initialised */
    bool isMemPoolInitialised;
} GatewayPool;

/* Global gateway pool instance */
static GatewayPool gGatewayPool =
{
    .isMemPoolInitialised = false,
};

/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */
/* Function prototypes */

/**
 * @brief Converts payload size in bytes to DLC value
 *
 * @param payload_size CAN payload size in bytes
 * @return uint8_t Corresponding DLC value, or 0xFF for invalid size
 */
static uint8_t CanTrafficGen_convertCanPayloadtoDlc(uint8_t payload_size, uint32_t* dlc);

/**
 * @brief Traffic generator task function that processes CAN packet generation
 *
 * @param args Task parameters pointer (CanTrafficGen_Object*)
 */
static void CanTrafficGen_tgTask(void* args);

/**
 * @brief Creates the traffic generator task
 *
 * @param configTg Pointer to TG configuration
 * @return int32_t SystemP_SUCCESS on success, error code otherwise
 */
static int32_t CanTrafficGen_createTgTask(CanTrafficGen_Object* configTg);

/**
 * @brief Initializes the memory pool for CAN packets
 *
 * @param memPool Pointer to the GatewayPool structure
 * @param configTg Pointer to TG configuration
 * @return uint32_t ENET_SOK on success, ENET_EFAIL on failure
 */
static uint32_t CanTrafficGen_initPool(GatewayPool *memPool, CanTrafficGen_Object* configTg);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
static uint8_t CanTrafficGen_convertCanPayloadtoDlc(uint8_t payload_size, uint32_t* dlc)
{
    int32_t status = ENET_SOK;
    bool isDlcSet = false;

    /* Use lookup table for cleaner implementation */
    static const struct {
        uint8_t size;
        uint8_t dlc;
    } dlcMap[] = {
        {0, 0}, {1, 1}, {2, 2}, {3, 3}, {4, 4}, {5, 5}, {6, 6}, {7, 7}, {8, 8},
        {12, 9}, {16, 10}, {20, 11}, {24, 12}, {32, 13}, {48, 14}, {64, 15}
    };

    /* Iterate through table to find matching payload size */
    for (int i = 0; i < sizeof(dlcMap)/sizeof(dlcMap[0]); i++)
    {
        if (dlcMap[i].size == payload_size)
        {
            *dlc =  dlcMap[i].dlc;
            isDlcSet = true;
        }
    }

    if (!isDlcSet)
    {
        *dlc = 0xFF;
        status = ENET_EFAIL;
    }

    return status;
}

static uint32_t CanTrafficGen_initPool(GatewayPool *memPool, CanTrafficGen_Object* configTg)
{
    int32_t status = ENET_SOK;
    uint32_t memPoolIndex = 0;

    /* Validate input parameters */
    if (memPool == NULL)
    {
        status = ENET_EFAIL;
    }

    /* Initialize the memory pool only if not already initialized */
    if (status == ENET_SOK && memPool->isMemPoolInitialised == false)
    {
        /* Initialize the queue for managing CAN packet memory */
        EnetQueue_initQ(&memPool->freeQueue);

        /* Enqueue all available memory blocks to the free queue */
        for (memPoolIndex = 0; memPoolIndex < ENETAPP_MAX_CANPKT_MEMBLOCKS; memPoolIndex++)
        {
            if (&configTg->rxCANPktList[memPoolIndex] != NULL)
            {
                EnetQueue_enq(&memPool->freeQueue, &configTg->rxCANPktList[memPoolIndex].node);
            }
        }

        /* Verify all memory blocks were successfully enqueued */
        if (EnetQueue_getQCount(&memPool->freeQueue) != ENETAPP_MAX_CANPKT_MEMBLOCKS)
        {
            status = ENET_EFAIL;
        }

        /* Mark pool as initialized on success */
        memPool->isMemPoolInitialised = true;
    }

    return status;
}

void CanTrafficGen_memFree(CAN_pkt *canPkt)
{
    /* Return the packet to the free queue if valid */
    if (canPkt != NULL)
    {
        EnetQueue_enq(&gGatewayPool.freeQueue, &canPkt->node);
    }
}

int32_t CanTrafficGen_setup(CanTrafficGen_Object *cfg)
{
    int32_t status = ENET_SOK;

    /* Validate input parameters */
    if (cfg == NULL)
    {
        status = ENET_EFAIL;
    }

    if (status == ENET_SOK)
    {
        /* Initialize queue for ready CAN elements */
        EnetQueue_initQ(&cfg->rxReadyElementQ);

        /* Initialize memory pool for CAN packet management */
        status = CanTrafficGen_initPool(&gGatewayPool, cfg);

        /* Create traffic generator task if memory pool initialization succeeded */
        if (status != ENET_SOK || CanTrafficGen_createTgTask(cfg) != ENET_SOK)
        {
            status = ENET_EFAIL;
        }
    }

    return status;
}

static int32_t CanTrafficGen_createTgTask(CanTrafficGen_Object* configTg)
{
    TaskP_Params taskParams;

    int32_t status = SystemP_SUCCESS;

    /* Create binary semaphore for task synchronization with initial count 0 */
    status = SemaphoreP_constructBinary(&configTg->tgSemObj, 0);
    if (status != SystemP_SUCCESS)
    {
        return status;
    }

    /* Initialize task parameters with default values */
    TaskP_Params_init(&taskParams);

    /* Configure task parameters */
    taskParams.priority       = ENETAPP_TG_TASK_PRIORITY;/* Set task priority */
    taskParams.stack          = gEnetAppTaskStackTg;   /* Assign stack memory */
    taskParams.stackSize      = sizeof(gEnetAppTaskStackTg); /* Set stack size */
    taskParams.args           = configTg;              /* Pass config as argument */
    taskParams.name           = "TG Task";             /* Name the task */
    taskParams.taskMain       = &CanTrafficGen_tgTask; /* Set entry point function */

    /* Create the task with the specified parameters */
    status = TaskP_construct(&gTgTaskObj, &taskParams);

    /* Clean up on failure */
    if (status != SystemP_SUCCESS)
    {
        SemaphoreP_destruct(&configTg->tgSemObj);
    }

    return status;
}

static void CanTrafficGen_tgTask(void* args)
{
    CanTrafficGen_Object* configTg = (CanTrafficGen_Object*)args;
    static uint64_t tgTickCount = 0;
    uint32_t dlc = 0U;
#ifdef ETHERRING_MCAN_ENABLE_PROFILING
    int32_t status = 0;
    EnetApp_timestampQ* timestampElement = NULL;
    uint64_t* timeStamp = NULL;
    Enet_IoctlPrms prms;
#endif

    /* Main task loop */
    while (true)
    {
        SemaphoreP_pend(&configTg->tgSemObj, SystemP_WAIT_FOREVER);
        /* Increment tick counter used for timing operations */
        tgTickCount++;

        /* Process each packet stream configuration */
        for (uint32_t index = 0; index < configTg->validPktParamsCount; index++)
        {
            /* This condition helps to send CAN streams based on independent periodicity.
            * If the timer tick periodicity is 250us and CAN stream periodcity is 500us,
            * then one CAN packet is generated for every 2 timer ticks*/
            if ((configTg->timerTickPeriodicityInus* tgTickCount) %
                (configTg->pktParams[index].periodicity) != 0 )
            {
                continue;
            }

            if (configTg->pktParams[0].hasAllCanPktsSent)
            {
                continue;
            }

            /* Generate the configured number of packets per tick for this stream */
            for (uint32_t itr = 0; itr < configTg->pktParams[index].packetCountPerTick; itr++)
            {
                /* maxPacketSend "0" treated as periodic CAN traffic */
                if (configTg->pktParams[index].maxPacketSend != 0U)
                {
                    /* This condition limits the max packet count per CAN packet stream */
                    if (configTg->pktParams[index].packetCounter >= configTg->pktParams[index].maxPacketSend - 1)
                    {
                        configTg->pktParams[0].hasAllCanPktsSent = true;
                    }
                }

                /* Increment total packet count */
                configTg->canGeneratedPktCount++;
                /* Increment packet counter for this stream */
                configTg->pktParams[index].packetCounter++;
                static uint32_t pktCount = 0;

                /* Check if free packets are available */
                if (EnetQueue_getQCount(&gGatewayPool.freeQueue) == 0)
                {
                    /* Queue empty, skip this iteration */
                    EnetAppUtils_print("free PktInfo not available\r\n");
                    continue;
                }

                /* Get a free packet from the pool */
                CAN_pkt* rxPkt = (CAN_pkt*)EnetQueue_deq(&gGatewayPool.freeQueue);
                MCAN_RxBufElement* rxElement = &rxPkt->canElement;

                /* Validate the element */
                if (rxElement == NULL)
                {
                    continue;
                }

                /* Calculate safe payload size for memory operations */
                size_t maxSize = sizeof(rxElement->data);
                size_t copySize = (configTg->pktParams[index].payloadSize < maxSize) ?
                    configTg->pktParams[index].payloadSize : maxSize;

                CanTrafficGen_convertCanPayloadtoDlc(configTg->pktParams[index].payloadSize , &dlc);

                /* Configure CAN message fields */
                rxElement->id = ((configTg->pktParams[index].CAN_msgId & 0x7FFU) << 18U);
                rxElement->dlc = dlc;
                rxElement->rtr = 0;                /* Not a remote transmission request */
                rxElement->xtd = 0;                /* Standard identifier */
                rxElement->esi = 0;                /* Error state indicator */
                rxElement->rxts = 0;               /* Rx timestamp */
                rxElement->brs = 1;                /* Bit rate switch */
                rxElement->fdf = 1;                /* FD format indicator */
                rxElement->fidx = 0;               /* Filter index */
                rxElement->anmf = 0;               /* Accepted non-matching frame */

                /* Fill data with incrementing pattern */
                memset(rxElement->data, (uint8_t)pktCount++, copySize);

#ifdef ETHERRING_MCAN_ENABLE_PROFILING
                /* Check if free packets are available */
                if (EnetQueue_getQCount(configTg->timestampFreeQPtr) == 0)
                {
                    /* Queue empty, skip this iteration */
                    EnetAppUtils_print("free timestamp not available: %d\r\n", configTg->canGeneratedPktCount);
                }
                /* Get a free packet from the pool */
                timestampElement = (EnetApp_timestampQ*)EnetQueue_deq(configTg->timestampFreeQPtr);
                timeStamp = &timestampElement->timeStamp;
                /* Software Time stamp Push event */
                ENET_IOCTL_SET_OUT_ARGS(&prms, timeStamp);
                ENET_IOCTL(configTg->hEnet, configTg->coreId,
                       ENET_TIMESYNC_IOCTL_GET_CURRENT_TIMESTAMP, &prms, status);
                EnetQueue_enq(configTg->timestampReadyQPtr, &timestampElement->node);
                EnetAppUtils_assert(status == ENET_SOK);
#endif
                /* Add packet to ready queue for processing */
                EnetQueue_enq(&configTg->rxReadyElementQ, &rxPkt->node);
                if (configTg->pktParams[0].hasAllCanPktsSent == true)
                {
#ifndef ETHERRING_MCAN_ENABLE_PROFILING
                    EnetAppUtils_print("[Traffic Gen] Generated %u CAN packet of CAN Message ID: %u\r\n",
                                       configTg->pktParams[index].maxPacketSend, configTg->pktParams[index].CAN_msgId);
#endif
                }
            }
        }
    }

    TaskP_exit();
}
