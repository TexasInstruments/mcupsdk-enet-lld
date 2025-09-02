/*
 *  Copyright (C) 2025 Texas Instruments Incorporated
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
 * \file  etherring_mcan.c
 *
 * \brief This file contains the MCAN Driver config APIs and configuration params
 */
/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */
#include <etherring_mcan.h>

/* ========================================================================== */
/*                                  Macros                                    */
/* ========================================================================== */
/* MCAN0 base address */
#define ETHERRING_MCAN_BASE_ADDR                       (CONFIG_MCAN0_BASE_ADDR)
/* MCAN0 interrupt number */
#define ETHERRING_MCAN_INTR_NUM                        (CONFIG_MCAN0_INTR)
/* Allocate Message RAM memory section to filter elements, buffers, FIFO */
/* Maximum STD Filter Element can be configured is 128 */
#define ETHERRING_MCAN_STD_ID_FILTER_CNT               (4U)
/* Maximum EXT Filter Element can be configured is 64 */
#define ETHERRING_MCAN_EXT_ID_FILTER_CNT               (0U)
/* Maximum TX Buffer + TX FIFO, combined can be configured is 32 */
#define ETHERRING_MCAN_TX_BUFF_CNT                     (1U)
#define ETHERRING_MCAN_TX_FIFO_CNT                     (0U)
/* Maximum TX Event FIFO can be configured is 32 */
#define ETHERRING_MCAN_TX_EVENT_FIFO_CNT               (0U)
/* Maximum RX FIFO 0 can be configured is 64 */
#define ETHERRING_MCAN_FIFO_0_CNT                      (0U)
/* Maximum RX FIFO 1 can be configured is 64 and
 * rest of the memory is allocated to RX buffer which is again of max size 64 */
#define ETHERRING_MCAN_FIFO_1_CNT                      (0U)
/* Extended CAN Msg Identifier Mask */
#define ETHERRING_MCAN_EXT_ID_MASK                     (0x1FFFFFFFU)
/* Task stack size in bytes */
#define ETHERRING_MCAN_TASK_STACK_SZ                   (4096U)
/* ========================================================================== */
/*                              Global Variables                              */
/* ========================================================================== */
static uint8_t gEnetAppTaskStackRxApp[ETHERRING_MCAN_TASK_STACK_SZ] __attribute__((aligned(32)));

/* ========================================================================== */
/*                          Function Declarations                              */
/* ========================================================================== */
static void    EtherringCAN_mcanIntrISR(void *arg);
static void    EtherringCAN_mcanInitMsgRamConfigParams(MCAN_MsgRAMConfigParams *msgRAMConfigParams);
static void    EtherringCAN_mcanEnableIntr(uint32_t mcanBaseAddr);
static void    EtherringCAN_mcanInitStdFilterElemParams(MCAN_StdMsgIDFilterElement *stdFiltElem,
                                                        uint32_t msgId,
                                                        uint32_t bufNum);
static int32_t EtherringCAN_calculateBufferNumber(int32_t newDataStatus);
void           EtherringCAN_mcanConfig(EtherringMcanObj *etherringMcanObj);
void           EtherringCAN_mcanEnableTransceiver(void);

/* ========================================================================== */
/*                          Function Definations                              */
/* ========================================================================== */
void EtherringCAN_sendCANPacket(MCAN_TxBufElement * txMsg, EtherringMcanObj *etherringMcanObj)
{
    MCAN_ProtocolStatus     protStatus;
    int32_t status;

    /* Select buffer number, 32 buffers available */
    uint32_t bufNum = 0U;
    /* Enable Transmission interrupt for the selected buf num,
     * If FIFO is used, then need to send FIFO start index until FIFO count */
    status = MCAN_txBufTransIntrEnable(etherringMcanObj->mcanBaseAddr, bufNum, (uint32_t)true);
    EnetAppUtils_assert(status == CSL_PASS);

    /* Write message to Msg RAM */
    MCAN_writeMsgRam(etherringMcanObj->mcanBaseAddr, MCAN_MEM_TYPE_BUF, bufNum, txMsg);

    /* Add request for transmission, This function will trigger transmission */
    status = MCAN_txBufAddReq(etherringMcanObj->mcanBaseAddr, bufNum);
    EnetAppUtils_assert(status == CSL_PASS);

    /* Wait for CAN Tx completion */
    SemaphoreP_pend(&etherringMcanObj->mcanTxDoneSem, SystemP_WAIT_FOREVER);

    MCAN_getProtocolStatus(etherringMcanObj->mcanBaseAddr, &protStatus);
    /* Checking for Tx Errors */
    if (((MCAN_ERR_CODE_NO_ERROR != protStatus.lastErrCode) ||
         (MCAN_ERR_CODE_NO_CHANGE != protStatus.lastErrCode)) &&
        ((MCAN_ERR_CODE_NO_ERROR != protStatus.dlec) ||
         (MCAN_ERR_CODE_NO_CHANGE != protStatus.dlec)) &&
        (0U != protStatus.pxe))
    {
        etherringMcanObj->mcanStats.txErrorCount++;
    }
    else
    {
        etherringMcanObj->mcanStats.txCanPacketCount++;
        if (etherringMcanObj->mcanStats.txCanPacketCount % ETHERRING_MCAN_TEST_PKT_COUNT == 0)
        {
#ifndef ETHERRING_MCAN_ENABLE_PROFILING
            EnetAppUtils_print("[MCAN Tx App] Sent %u CAN packets to CAN bus\r\n",
                               etherringMcanObj->mcanStats.txCanPacketCount);
#endif
        }
    }
}

void EtherringCAN_rxCANTask(void *args)
{
    EtherringMcanObj *etherringMcanObj = (EtherringMcanObj *)args;
    MCAN_RxBufElement       rxMsg;
    MCAN_RxNewDataStatus    newDataStatus;
    MCAN_ErrCntStatus       errCounter;
    uint32_t                bufNum, fifoNum;
#ifdef ETHERRING_MCAN_ENABLE_PROFILING
    uint64_t currentTimeStamp = 0U;
    int32_t retVal = 0U;
    Enet_IoctlPrms prms;
    uint64_t packetGenTimeStamp = 0U;
#endif

    while(true)
    {
        /* Wait for Rx completion */
        SemaphoreP_pend(&etherringMcanObj->mcanRxDoneSem, SystemP_WAIT_FOREVER);

        /* Checking for Rx Errors */
        MCAN_getErrCounters(etherringMcanObj->mcanBaseAddr, &errCounter);
        if ((0U != errCounter.recErrCnt)  || (0U != errCounter.canErrLogCnt))
        {
            etherringMcanObj->mcanStats.rxErrorCount++;
        }

        /* Get the new data staus, indicates buffer num which received message */
        MCAN_getNewDataStatus(etherringMcanObj->mcanBaseAddr, &newDataStatus);
        MCAN_clearNewDataStatus(etherringMcanObj->mcanBaseAddr, &newDataStatus);

        /* Select buffer and fifo number, Buffer is used in this app */
        fifoNum = MCAN_RX_FIFO_NUM_0;
        bufNum= EtherringCAN_calculateBufferNumber(newDataStatus.statusLow);

        if(bufNum >= 0)
        {
            MCAN_readMsgRam(etherringMcanObj->mcanBaseAddr, MCAN_MEM_TYPE_BUF, bufNum, fifoNum, &rxMsg);
            etherringMcanObj->mcanStats.rxCanPacketCount++;

#ifdef ETHERRING_MCAN_ENABLE_PROFILING
            EnetApp_timestampQ *tsElement = (EnetApp_timestampQ *)EnetQueue_deq(etherringMcanObj->timestampReadyQPtr);
            if(tsElement == NULL)
            {
                  EnetAppUtils_print("[MCAN Rx App] tsElement null\r\n");
            }
            packetGenTimeStamp = tsElement->timeStamp;
            currentTimeStamp = 0U;
            /* Software Time stamp Push event */
            ENET_IOCTL_SET_OUT_ARGS(&prms, &currentTimeStamp);
            ENET_IOCTL(etherringMcanObj->hEnet, etherringMcanObj->coreId,
                   ENET_TIMESYNC_IOCTL_GET_CURRENT_TIMESTAMP, &prms, retVal);
            EnetAppUtils_assert(retVal == ENET_SOK);

            etherringMcanObj->avgTimeStamp = (((currentTimeStamp - packetGenTimeStamp)/1000) + etherringMcanObj->avgTimeStamp *
                                             (etherringMcanObj->profileIndex))/(etherringMcanObj->profileIndex+1);
            if(((currentTimeStamp - packetGenTimeStamp)/1000) > etherringMcanObj->maxTimeStamp)
            {
                etherringMcanObj->maxTimeStamp = ((currentTimeStamp - packetGenTimeStamp)/1000);
            }
            etherringMcanObj->profileIndex++;

            if (etherringMcanObj->mcanStats.rxCanPacketCount == ETHERRING_MCAN_TEST_PKT_COUNT)
            {
                EnetAppUtils_print("[MCAN Rx App] Avg Round Trip Time:%llu us\r\n",etherringMcanObj->avgTimeStamp);
                EnetAppUtils_print("[MCAN Rx App] Max Round Trip Time:%llu us\r\n",etherringMcanObj->maxTimeStamp);
                etherringMcanObj->mcanStats.rxCanPacketCount++;
            }
            EnetQueue_enq(etherringMcanObj->timestampFreeQPtr, &tsElement->node);
#endif
            if (etherringMcanObj->mcanStats.rxCanPacketCount % ETHERRING_MCAN_TEST_PKT_COUNT == 0)
            {
#ifndef ETHERRING_MCAN_ENABLE_PROFILING
                EnetAppUtils_print("[MCAN Rx App] Received %u CAN packets from CAN bus\r\n",
                                   etherringMcanObj->mcanStats.rxCanPacketCount);
#endif
            }
        }
        else
        {
            etherringMcanObj->mcanStats.rxErrorCount++;
        }
    }
}

static int32_t EtherringCAN_calculateBufferNumber(int32_t newDataStatus) {
    int32_t count = 0;

    if(newDataStatus == 1)
    {
        count = 0;
    }
    else if (newDataStatus <= 0)
    {
        count = -1;
    }
    else
    {
        /* Find position of the least significant set bit */
        while ((newDataStatus & 0x1) == 0)
        {
            newDataStatus >>= 1;
            count++;
        }

        /* Verify only one bit was set */
        if (newDataStatus != 1)
        {
            count = -1;
        }
    }

    return count;
}

void EtherringCAN_mcanConfig(EtherringMcanObj *etherringMcanObj)
{
    MCAN_StdMsgIDFilterElement stdFiltElem[ETHERRING_MCAN_STD_ID_FILTER_CNT] = {0U};
    MCAN_InitParams            initParams = {0U};
    MCAN_ConfigParams          configParams = {0U};
    MCAN_MsgRAMConfigParams    msgRAMConfigParams = {0U};
    MCAN_BitTimingParams       bitTimes = {0U};
    uint32_t                   filterIndex = {0U};
    HwiP_Params             hwiPrms;
    int32_t status = ENET_SOK;

    EtherringCAN_mcanEnableTransceiver();

    /* Construct Tx/Rx Semaphore objects */
    status = SemaphoreP_constructBinary(&etherringMcanObj->mcanTxDoneSem, 0);
    EnetAppUtils_assert(ENET_SOK == status);
    status = SemaphoreP_constructBinary(&etherringMcanObj->mcanRxDoneSem, 0);
    EnetAppUtils_assert(ENET_SOK == status);

    /* Register interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = ETHERRING_MCAN_INTR_NUM;
    hwiPrms.callback    = &EtherringCAN_mcanIntrISR;
    hwiPrms.args = (void*)etherringMcanObj;
    status              = HwiP_construct(&etherringMcanObj->mcanHwiObject, &hwiPrms);
    EnetAppUtils_assert(status == ENET_SOK);

    /* Assign MCAN instance address */
    etherringMcanObj->mcanBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(ETHERRING_MCAN_BASE_ADDR);

    /* Initialize MCAN module initParams */
    MCAN_initOperModeParams(&initParams);
    /* CAN FD Mode and Bit Rate Switch Enabled */
    initParams.fdMode          = true;
    initParams.brsEnable       = true;

    /* Initialize MCAN module Global Filter Params */
    MCAN_initGlobalFilterConfigParams(&configParams);

    /* Initialize MCAN module Bit Time Params */
    /* Configuring default 1Mbps and 5Mbps as nominal and data bit-rate resp */
    MCAN_initSetBitTimeParams(&bitTimes);

    /* Initialize MCAN module Message Ram Params */
    EtherringCAN_mcanInitMsgRamConfigParams(&msgRAMConfigParams);

    /* Initialize Filter element to receive msg. Added filter for 0x1,0x2,0x3,0xFF CAN msgIds*/
    EtherringCAN_mcanInitStdFilterElemParams(&stdFiltElem[0], 0x2, 0x0);
    EtherringCAN_mcanInitStdFilterElemParams(&stdFiltElem[1], 0x1, 0x1);
    EtherringCAN_mcanInitStdFilterElemParams(&stdFiltElem[2], 0x3, 0x2);
    EtherringCAN_mcanInitStdFilterElemParams(&stdFiltElem[3], 0xFF, 0x3);

    /* wait for memory initialization to happen */
    while (FALSE == MCAN_isMemInitDone(etherringMcanObj->mcanBaseAddr))
    {}

    /* Put MCAN in SW initialization mode */
    MCAN_setOpMode(etherringMcanObj->mcanBaseAddr, MCAN_OPERATION_MODE_SW_INIT);
    while (MCAN_OPERATION_MODE_SW_INIT != MCAN_getOpMode(etherringMcanObj->mcanBaseAddr))
    {}

    /* Initialize MCAN module */
    MCAN_init(etherringMcanObj->mcanBaseAddr, &initParams);
    /* Configure MCAN module Gloabal Filter */
    MCAN_config(etherringMcanObj->mcanBaseAddr, &configParams);
    /* Configure Bit timings */
    MCAN_setBitTime(etherringMcanObj->mcanBaseAddr, &bitTimes);
    /* Configure Message RAM Sections */
    MCAN_msgRAMConfig(etherringMcanObj->mcanBaseAddr, &msgRAMConfigParams);
    /* Set Extended ID Mask */
    MCAN_setExtIDAndMask(etherringMcanObj->mcanBaseAddr, ETHERRING_MCAN_EXT_ID_MASK);

    /* Configure Standard ID filter element */
    for (filterIndex = 0U; filterIndex < ETHERRING_MCAN_STD_ID_FILTER_CNT; filterIndex++)
    {
        MCAN_addStdMsgIDFilter(etherringMcanObj->mcanBaseAddr, filterIndex, &stdFiltElem[filterIndex]);
    }

    /* Take MCAN out of the SW initialization mode */
    MCAN_setOpMode(etherringMcanObj->mcanBaseAddr, MCAN_OPERATION_MODE_NORMAL);
    while (MCAN_OPERATION_MODE_NORMAL != MCAN_getOpMode(etherringMcanObj->mcanBaseAddr))
    {}

    /* Enable Interrupts */
    EtherringCAN_mcanEnableIntr(etherringMcanObj->mcanBaseAddr);

    EnetAppUtils_print("[MCAN App] Mcan Driver Configured\r\n");
}

static void EtherringCAN_mcanInitStdFilterElemParams(MCAN_StdMsgIDFilterElement *stdFiltElem,
                                                     uint32_t msgId,
                                                     uint32_t bufNum)
{
    /* sfid1 defines the ID of the standard message to be stored. */
    stdFiltElem->sfid1 = msgId;
    /* As buffer mode is selected, sfid2 should be bufNum[0 - 63] */
    stdFiltElem->sfid2 = bufNum;
    /* Store message in buffer */
    stdFiltElem->sfec  = MCAN_STD_FILT_ELEM_BUFFER;
    /* Below configuration is ignored if message is stored in buffer */
    stdFiltElem->sft   = MCAN_STD_FILT_TYPE_RANGE;
}

static void EtherringCAN_mcanEnableIntr(uint32_t          mcanBaseAddr)
{
    MCAN_enableIntr(mcanBaseAddr, MCAN_INTR_MASK_ALL, (uint32_t)true);
    MCAN_enableIntr(mcanBaseAddr, MCAN_INTR_SRC_RES_ADDR_ACCESS, (uint32_t)FALSE);
    /* Select Interrupt Line 0 */
    MCAN_selectIntrLine(mcanBaseAddr, MCAN_INTR_MASK_ALL, MCAN_INTR_LINE_NUM_0);
    /* Enable Interrupt Line */
    MCAN_enableIntrLine(mcanBaseAddr, MCAN_INTR_LINE_NUM_0, (uint32_t)true);
}

static void EtherringCAN_mcanInitMsgRamConfigParams(MCAN_MsgRAMConfigParams
                                           *msgRAMConfigParams)
{
    int32_t status;

    MCAN_initMsgRamConfigParams(msgRAMConfigParams);

    /* Configure the user required msg ram params */
    msgRAMConfigParams->lss = ETHERRING_MCAN_STD_ID_FILTER_CNT;
    msgRAMConfigParams->lse = ETHERRING_MCAN_EXT_ID_FILTER_CNT;
    msgRAMConfigParams->txBufCnt = ETHERRING_MCAN_TX_BUFF_CNT;
    msgRAMConfigParams->txFIFOCnt = ETHERRING_MCAN_TX_FIFO_CNT;
    /* Buffer/FIFO mode is selected */
    msgRAMConfigParams->txBufMode = MCAN_TX_MEM_TYPE_BUF;
    msgRAMConfigParams->txEventFIFOCnt = ETHERRING_MCAN_TX_EVENT_FIFO_CNT;
    msgRAMConfigParams->rxFIFO0Cnt = ETHERRING_MCAN_FIFO_0_CNT;
    msgRAMConfigParams->rxFIFO1Cnt = ETHERRING_MCAN_FIFO_1_CNT;
    /* FIFO blocking mode is selected */
    msgRAMConfigParams->rxFIFO0OpMode = MCAN_RX_FIFO_OPERATION_MODE_BLOCKING;
    msgRAMConfigParams->rxFIFO1OpMode = MCAN_RX_FIFO_OPERATION_MODE_BLOCKING;

    status = MCAN_calcMsgRamParamsStartAddr(msgRAMConfigParams);
    EnetAppUtils_assert(status == ENET_SOK);
}

static void EtherringCAN_mcanIntrISR(void *arg)
{
    uint32_t intrStatus = 0U;
    EtherringMcanObj *etherringMcanObj = (EtherringMcanObj *)arg;

    intrStatus = MCAN_getIntrStatus(etherringMcanObj->mcanBaseAddr);
    MCAN_clearIntrStatus(etherringMcanObj->mcanBaseAddr, intrStatus);

    if (MCAN_INTR_SRC_TRANS_COMPLETE ==
       (intrStatus & MCAN_INTR_SRC_TRANS_COMPLETE))
    {
        SemaphoreP_post(&etherringMcanObj->mcanTxDoneSem);
    }

    /* If FIFO0/FIFO1 is used, then MCAN_INTR_SRC_DEDICATED_RX_BUFF_MSG macro
     * needs to be replaced by MCAN_INTR_SRC_RX_FIFO0_NEW_MSG/
     * MCAN_INTR_SRC_RX_FIFO1_NEW_MSG respectively */
    if (MCAN_INTR_SRC_DEDICATED_RX_BUFF_MSG ==
       (intrStatus & MCAN_INTR_SRC_DEDICATED_RX_BUFF_MSG))
    {
        etherringMcanObj->mcanStats.rxIsrCount++;
        SemaphoreP_post(&etherringMcanObj->mcanRxDoneSem);
    }
}

int32_t EtherringCAN_createRxCANTask(EtherringMcanObj *etherringMcanObj)
{
    TaskP_Params taskParams;
    int32_t status = ENET_SOK;

    EnetAppUtils_assert(ENET_SOK == status);

    /* Initialize task parameters */
    TaskP_Params_init(&taskParams);
    taskParams.priority       = 4U;
    taskParams.stack          = gEnetAppTaskStackRxApp;
    taskParams.stackSize      = sizeof(gEnetAppTaskStackRxApp);
    taskParams.args           = (void*)etherringMcanObj;;
    taskParams.name           = "Rx Mcan Task";
    taskParams.taskMain       = &EtherringCAN_rxCANTask;

    /* Create the RX  MCAN task */
    status = TaskP_construct(&etherringMcanObj->rxTaskObj, &taskParams);
    EnetAppUtils_assert(ENET_SOK == status);

    return status;
}
