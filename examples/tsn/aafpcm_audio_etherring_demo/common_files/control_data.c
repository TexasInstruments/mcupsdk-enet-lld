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

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include "kernel/dpl/TaskP.h"
#include "../../tsninit.h"
#include "../../enetapp_cpsw.h"
#include "../../../../ether_ring/inc/ether_ring.h"
#include "ti_enet_config.h"
#include "enet_apputils.h"
#include "enet_apputils_k3.h"
#include "enet_appmemutils.h"
#include <stdint.h>
#include "avtpcf.h"
#include "Enetapp_common.h"
#include <ti_drivers_config.h>
#include <kernel/dpl/TimerP.h>
#include <drivers/gpio.h>
#include <ti_drivers_config.h>
#include <drivers/pinmux.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define ENETAPP_STREAM_VLANID                         (255U)
#define ENETAPP_TEST_SIGNAL_FREQ_HZ                     (10)
#define ENETAPP_CONTROL_DATA_PERIOD_NS              (250000)

extern EnetApp_Cfg gEnetAppCfg;

avtpcf_data_t avtpData;

typedef enum
{
    ROLE_LISTENER = -1,
    ROLE_NONE = 0,
    ROLE_TALKER = 1,
}streamRole;

typedef int (*ctrlData_txHook)(uint8_t *payload, int *size, node_index src, node_index dest);

typedef struct
{
    int prevSeqNumber;
    int totalMissing;
    int totalReceived;
} stats_t;
typedef struct
{
    const node_index source_device;
    const node_index dest_device;
    const char* streamID;
    const char* destMcastAddr;
    const bool isEnabled;
    const uint16_t vlanID;
    const uint8_t priority;
    const ctrlData_txHook txHook;
    const avtpcf_recv_cb rxCb;

    streamRole role;
    union
    {
        avtpcf_talker_cfg talkerCfg;
        avtpcf_listener_cfg listenerCfg;
    };
    union
    {
        avtpcf_talker_Obj talkerObj;
        avtpcf_listener_Obj listenerObj;
    };

    stats_t stats;
} streamTable_t;

typedef struct
{
    uint32_t gpioBase;
    uint32_t gpioPin;
    uint32_t gpioPinDir;
} gpio_cfg;

typedef struct __attribute__ ((packed))
{
    uint32_t acf_msg_type;
    uint32_t acf_msg_length;
    uint32_t acf_gpioState;
} gpio_acf_msg;

static int32_t talkerListener_TxHook(uint8_t *payload, int *size, node_index src, node_index dest);

static int32_t avtpcf_listener1_cb(avtpcf_info *streamInfo, \
                                                    uint8_t *payload, int size, void *cbArgs);

static void controlData_printStats(streamTable_t *streamCfg);

streamTable_t gStreamTable[] = {
    {.isEnabled = true, .source_device = NODE_CENTRAL, .dest_device = NODE_RIGHT  ,
        .streamID = "00:01:02:03:04:05:01:01", .destMcastAddr = "91:E0:F0:00:FE:01",
        .txHook=talkerListener_TxHook, .rxCb=avtpcf_listener1_cb,
        .priority=5, .vlanID=ENETAPP_STREAM_VLANID},

    {.isEnabled = true, .source_device = NODE_CENTRAL, .dest_device = NODE_LEFT   ,
        .streamID = "00:01:02:03:04:05:01:02", .destMcastAddr = "91:E0:F0:00:FE:02",
        .txHook=talkerListener_TxHook, .rxCb=avtpcf_listener1_cb,
        .priority=5, .vlanID=ENETAPP_STREAM_VLANID},

    {.isEnabled = true, .source_device = NODE_CENTRAL, .dest_device = NODE_TAIL   ,
        .streamID = "00:01:02:03:04:05:01:03", .destMcastAddr = "91:E0:F0:00:FE:03",
        .txHook=talkerListener_TxHook, .rxCb=avtpcf_listener1_cb,
        .priority=5, .vlanID=ENETAPP_STREAM_VLANID},

    {.isEnabled = true, .source_device = NODE_RIGHT  , .dest_device = NODE_CENTRAL,
        .streamID = "00:01:02:03:04:05:01:21", .destMcastAddr = "91:E0:F0:00:FE:21",
        .txHook=talkerListener_TxHook, .rxCb=avtpcf_listener1_cb,
        .priority=5, .vlanID=ENETAPP_STREAM_VLANID},

    {.isEnabled = true, .source_device = NODE_LEFT   , .dest_device = NODE_CENTRAL,
        .streamID = "00:01:02:03:04:05:01:22", .destMcastAddr = "91:E0:F0:00:FE:22",
        .txHook=talkerListener_TxHook, .rxCb=avtpcf_listener1_cb,
        .priority=5, .vlanID=ENETAPP_STREAM_VLANID},

    {.isEnabled = true, .source_device = NODE_TAIL   , .dest_device = NODE_CENTRAL,
        .streamID = "00:01:02:03:04:05:01:23", .destMcastAddr = "91:E0:F0:00:FE:23",
        .txHook=talkerListener_TxHook, .rxCb=avtpcf_listener1_cb,
        .priority=5, .vlanID=ENETAPP_STREAM_VLANID},
};

SemaphoreP_Object gControlDataSem;
#if DEF_NODE_CENTRAL

#define CONFIG_CENT_IN_1_BASE_ADDR (CSL_GPIO1_BASE)
#define CONFIG_CENT_IN_1_PIN (74)
#define CONFIG_CENT_IN_1_DIR (GPIO_DIRECTION_INPUT)

#define CONFIG_CENT_IN_2_BASE_ADDR (CSL_GPIO1_BASE)
#define CONFIG_CENT_IN_2_PIN (75)
#define CONFIG_CENT_IN_2_DIR (GPIO_DIRECTION_INPUT)

#define CONFIG_CENT_IN_3_BASE_ADDR (CSL_GPIO1_BASE)
#define CONFIG_CENT_IN_3_PIN (76)
#define CONFIG_CENT_IN_3_DIR (GPIO_DIRECTION_INPUT)

/* Read Input LUT. Access using "gpioInLut[dest_node]" */
gpio_cfg gpioInLut[NODE_TOTAL_NODES] = {
    [NODE_CENTRAL] = {.gpioBase = 0,                          .gpioPin = 0,                    .gpioPinDir = 0},
    [NODE_RIGHT]   = {.gpioBase = CONFIG_CENT_IN_1_BASE_ADDR, .gpioPin = CONFIG_CENT_IN_1_PIN, .gpioPinDir = CONFIG_CENT_IN_1_DIR},
    [NODE_LEFT]    = {.gpioBase = CONFIG_CENT_IN_2_BASE_ADDR, .gpioPin = CONFIG_CENT_IN_2_PIN, .gpioPinDir = CONFIG_CENT_IN_2_DIR},
    [NODE_TAIL]    = {.gpioBase = CONFIG_CENT_IN_3_BASE_ADDR, .gpioPin = CONFIG_CENT_IN_3_PIN, .gpioPinDir = CONFIG_CENT_IN_3_DIR}
};

/* Write Output LUT. Access using "gpioOutLut[src_node]" */
gpio_cfg gpioLocalOutLut[NODE_TOTAL_NODES] = {
    [NODE_CENTRAL] = {.gpioBase = 0,    .gpioPin = 0,    .gpioPinDir = 0},
    [NODE_RIGHT]   = {.gpioBase = CONFIG_CENTRAL_OUT_P1_BASE_ADDR,    .gpioPin = CONFIG_CENTRAL_OUT_P1_PIN,    .gpioPinDir = CONFIG_CENTRAL_OUT_P1_DIR},
    [NODE_LEFT]    = {.gpioBase = CONFIG_CENTRAL_OUT_P2_BASE_ADDR,    .gpioPin = CONFIG_CENTRAL_OUT_P2_PIN,    .gpioPinDir = CONFIG_CENTRAL_OUT_P2_DIR},
    [NODE_TAIL]    = {.gpioBase = CONFIG_CENTRAL_OUT_P3_BASE_ADDR,    .gpioPin = CONFIG_CENTRAL_OUT_P3_PIN,    .gpioPinDir = CONFIG_CENTRAL_OUT_P3_DIR}
};

/* Write Output LUT. Access using "gpioOutLut[src_node]" */
gpio_cfg gpioRemoteOutLut[NODE_TOTAL_NODES] = {
    [NODE_CENTRAL] = {.gpioBase = 0,    .gpioPin = 0,    .gpioPinDir = 0},
    [NODE_RIGHT]   = {.gpioBase = CONFIG_CENTRAL_OUT_Q1_BASE_ADDR,    .gpioPin = CONFIG_CENTRAL_OUT_Q1_PIN,    .gpioPinDir = CONFIG_CENTRAL_OUT_Q1_DIR},
    [NODE_LEFT]    = {.gpioBase = CONFIG_CENTRAL_OUT_Q2_BASE_ADDR,    .gpioPin = CONFIG_CENTRAL_OUT_Q2_PIN,    .gpioPinDir = CONFIG_CENTRAL_OUT_Q2_DIR},
    [NODE_TAIL]    = {.gpioBase = CONFIG_CENTRAL_OUT_Q3_BASE_ADDR,    .gpioPin = CONFIG_CENTRAL_OUT_Q3_PIN,    .gpioPinDir = CONFIG_CENTRAL_OUT_Q3_DIR}
};

#else /* For All the other Nodes.  */

/* Defining AIN Pins here. */
#define CONFIG_AMP_IN_1_BASE_ADDR (CSL_GPIO1_BASE)
#define CONFIG_AMP_IN_1_PIN       (74)
#define CONFIG_AMP_IN_1_DIR       (GPIO_DIRECTION_INPUT)

/* Read Input LUT. Access using "gpioInLut[dest_node]" */
gpio_cfg gpioInLut[NODE_TOTAL_NODES] = {
    [NODE_CENTRAL] = {.gpioBase = CONFIG_AMP_IN_1_BASE_ADDR, .gpioPin = CONFIG_AMP_IN_1_PIN, .gpioPinDir = CONFIG_AMP_IN_1_DIR},
    [NODE_LEFT]    = {.gpioBase = 0,    .gpioPin = 0,    .gpioPinDir = 0},
    [NODE_RIGHT]   = {.gpioBase = 0,    .gpioPin = 0,    .gpioPinDir = 0},
    [NODE_TAIL]    = {.gpioBase = 0,    .gpioPin = 0,    .gpioPinDir = 0}
};

/* Write Output LUT. Access using "gpioOutLut[src_node]" */
gpio_cfg gpioLocalOutLut[NODE_TOTAL_NODES] = {
    [NODE_CENTRAL] = {.gpioBase = CONFIG_AMP_OUT_P1_BASE_ADDR, .gpioPin = CONFIG_AMP_OUT_P1_PIN, .gpioPinDir = CONFIG_AMP_OUT_P1_DIR},
    [NODE_LEFT]    = {.gpioBase = 0,    .gpioPin = 0,    .gpioPinDir = 0},
    [NODE_RIGHT]   = {.gpioBase = 0,    .gpioPin = 0,    .gpioPinDir = 0},
    [NODE_TAIL]    = {.gpioBase = 0,    .gpioPin = 0,    .gpioPinDir = 0}
};

/* Write Output LUT. Access using "gpioOutLut[src_node]" */
gpio_cfg gpioRemoteOutLut[NODE_TOTAL_NODES] = {
    [NODE_CENTRAL] = {.gpioBase = CONFIG_AMP_OUT_Q1_BASE_ADDR, .gpioPin = CONFIG_AMP_OUT_Q1_PIN, .gpioPinDir = CONFIG_AMP_OUT_Q1_DIR},
    [NODE_LEFT]    = {.gpioBase = 0,    .gpioPin = 0,    .gpioPinDir = 0},
    [NODE_RIGHT]   = {.gpioBase = 0,    .gpioPin = 0,    .gpioPinDir = 0},
    [NODE_TAIL]    = {.gpioBase = 0,    .gpioPin = 0,    .gpioPinDir = 0}
};
#endif

TaskP_Object statsTask;

uint8_t statsTaskStack[TSN_TSK_STACK_SIZE]
                    __attribute__ ((aligned(TSN_TSK_STACK_ALIGN)));

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static int32_t EnetApp_addVlanEntries(Enet_Handle hEnet, uint32_t coreId, uint32_t vlan);\

static int32_t EnetApp_configMcastAddr(Enet_Handle hEnet, \
                          uint32_t coreId, uint8_t *mcast, uint32_t vlanID, uint8_t portmask);

static int32_t EnetApp_applyClassifierMatch(Enet_Handle hEnet, \
                                        uint32_t coreId, uint32_t vlanID, uint32_t rxFlowIdx);

static int32_t CtrlData_streamIdAtoI(const char *txt, uint8_t *addr);

static int32_t controlData_configTalker(avtpcf_data_t* avtpcfData, const streamTable_t* streamInfo, \
                                    avtpcf_talker_cfg* cfg, avtpcf_talker_Obj* obj);

static int32_t controlData_configListener(avtpcf_data_t* avtpcfData, streamTable_t* streamInfo, \
                                    avtpcf_listener_cfg* cfg, avtpcf_listener_Obj* obj);

static int32_t controlData_configureStreams(avtpcf_data_t* avtpcfData, node_index thisNode);

static int32_t controlData_talkerSpin(avtpcf_data_t* avtpcfData, node_index thisNode);

static void controlData_configGpio(void);

static void controlData_statsTask(void* args);

void timerPwm_configTestSignal(uint32_t freq);

static void controlData_setPinMux(void);

static void EnetApp_setupControlTimer(uint32_t tickPeriodNs);

void EnetApp_configControlData(void* args)
{
    controlData_setPinMux();

    timerPwm_configTestSignal(ENETAPP_TEST_SIGNAL_FREQ_HZ);

    EnetApp_setupControlTimer(ENETAPP_CONTROL_DATA_PERIOD_NS);

    TaskP_Params params = {
        .name = "stats_task",
        .stack = statsTaskStack,
        .stackSize = sizeof(statsTaskStack),
        .priority  = TaskP_PRIORITY_LOWEST,
        .taskMain  = controlData_statsTask,
    };

    TaskP_construct(&statsTask, &params);

    int32_t status = ENET_SOK;
    avtpData.enetInfo.hEnet   = gEnetAppCfg.hEnet;
    avtpData.enetInfo.coreId  = gEnetAppCfg.coreId;
    avtpData.enetInfo.coreKey = gEnetAppCfg.coreKey;
    memcpy(avtpData.enetInfo.macAddr, gEnetAppCfg.macAddr, sizeof(avtpData.enetInfo.macAddr));

    /* Configure DMA TX and RX Channels and Packet Info. */
    avtpData.enetInfo.cfg.rxChCfgIdx = ENET_DMA_RX_ETHERRING;
    avtpData.enetInfo.cfg.txChCfgIdx = ENET_DMA_TX_ETHERRING;
    avtpData.enetInfo.cfg.numRxPkts  = ENET_DMA_RX_ETHERRING_NUM_PKTS;
    avtpData.enetInfo.cfg.numTxPkts  = ENET_DMA_TX_ETHERRING_NUM_PKTS;
    avtpData.enetInfo.cfg.packetSize = ENET_MEM_LARGE_POOL_PKT_SIZE;

    avtpcf_init(&avtpData);

    if (status == ENET_SOK)
    {
        EnetApp_addVlanEntries(avtpData.enetInfo.hEnet,
                    avtpData.enetInfo.coreId, ENETAPP_STREAM_VLANID);
        /*
            Apply classifier to filter based on the VLAN ID.
            This Configuration has to be done after
        */
        status = EnetApp_applyClassifierMatch(avtpData.enetInfo.hEnet,
                            avtpData.enetInfo.coreId, ENETAPP_STREAM_VLANID,
                            avtpData.enetInfo.rxFlowIdx);
        DebugP_assert(status == ENET_SOK);
    }

    controlData_configureStreams(&avtpData, gNodeIndex);

    SemaphoreP_constructBinary(&gControlDataSem, 0);

    controlData_configGpio();

    ClockP_sleep(3);

    TimerP_start(gTimerBaseAddr[CONFIG_CTRL_TICK]);

    while (1)
    {
        SemaphoreP_pend(&gControlDataSem, SystemP_WAIT_FOREVER);
        controlData_talkerSpin(&avtpData, gNodeIndex);
    }
}

static void EnetApp_setupControlTimer(uint32_t tickPeriodNs)
{
    TimerP_Params timerParams;
    TimerP_Params_init(&timerParams);
    timerParams.inputPreScaler = CONFIG_CTRL_TICK_INPUT_PRE_SCALER;
    timerParams.inputClkHz     = CONFIG_CTRL_TICK_INPUT_CLK_HZ;
    timerParams.periodInNsec   = tickPeriodNs;
    timerParams.oneshotMode    = 0;
    timerParams.enableOverflowInt = 1;
    timerParams.enableDmaTrigger  = 0;
    TimerP_setup(gTimerBaseAddr[CONFIG_CTRL_TICK], &timerParams);
}

static void controlData_configGpio(void)
{
    for (int i = 0; i < NODE_TOTAL_NODES; i++)
    {
        /* GPIO IN LUT */
        gpio_cfg *cfg = &gpioInLut[i];
        if (cfg->gpioBase != 0)
        {
            cfg->gpioBase = (uint32_t) AddrTranslateP_getLocalAddr(cfg->gpioBase);
            GPIO_setDirMode(cfg->gpioBase, cfg->gpioPin, cfg->gpioPinDir);
        }

        /* GPIO OUT LUT */
        cfg = &gpioLocalOutLut[i];
        if (cfg->gpioBase != 0)
        {
            cfg->gpioBase = (uint32_t) AddrTranslateP_getLocalAddr(cfg->gpioBase);
            GPIO_setDirMode(cfg->gpioBase, cfg->gpioPin, cfg->gpioPinDir);
        }

        /* GPIO OUT LUT */
        cfg = &gpioRemoteOutLut[i];
        if (cfg->gpioBase != 0)
        {
            cfg->gpioBase = (uint32_t) AddrTranslateP_getLocalAddr(cfg->gpioBase);
            GPIO_setDirMode(cfg->gpioBase, cfg->gpioPin, cfg->gpioPinDir);
        }
    }
}

static void controlData_statsTask(void* args)
{
    while (1)
    {
        ClockP_sleep(1);
        for (int i = 0; i < ENET_ARRAYSIZE(gStreamTable); i++)
        {
            if (gStreamTable[i].role == ROLE_LISTENER)
            {
                // gStreamTable[i].stats
                // controlData_printStats(&gStreamTable[i]);
            }
        }
    }
}

static int32_t talkerListener_TxHook(uint8_t *payload, int *size, node_index src, node_index dest)
{
    /* Read Respective GPIO and Fill the buffer. */
    DebugP_assert(src == gNodeIndex);

    uint32_t pinState = 0;

    /* Lookup the destination and toggle accordingly. */
    gpio_cfg *cfg = &gpioInLut[dest];
    DebugP_assert(cfg->gpioBase > 0);

    /* Get GPIO Status from Payload. */
    pinState = GPIO_pinRead(cfg->gpioBase, cfg->gpioPin);

    gpio_acf_msg *msg   = (gpio_acf_msg*)payload;
    msg->acf_msg_type   = 1;
    msg->acf_msg_length = 4;
    msg->acf_gpioState  = pinState;
    *size = sizeof(*msg);

    gpio_cfg *outCfg = &gpioLocalOutLut[dest];
    if (pinState)
    {
        GPIO_pinWriteHigh(outCfg->gpioBase, outCfg->gpioPin);
    }
    else
    {
        GPIO_pinWriteLow(outCfg->gpioBase, outCfg->gpioPin);
    }

    return ENET_SOK;
}

static int32_t controlData_talkerSpin(avtpcf_data_t* avtpcfData, node_index thisNode)
{
    for (int i = 0; i < ENET_ARRAYSIZE(gStreamTable); i++)
    {
        streamTable_t* streamInfo = &gStreamTable[i];

        if (streamInfo->role == ROLE_TALKER)
        {
            /* Talker Enqueue is not Zero copy hence this is safe. */
            uint8_t payload[100];
            int size = sizeof(payload);
            /* Call the tx Hook Function functions. */
            streamInfo->txHook(payload, &size, \
                                streamInfo->source_device, streamInfo->dest_device);
            avtpcf_talker_enqueue(&streamInfo->talkerObj, payload, size);
        }
    }
    avtpcf_submit_packets(avtpcfData);
    avtpcf_recycle_packets(avtpcfData);
    return ENET_SOK;
}

static int32_t controlData_configureStreams(avtpcf_data_t* avtpcfData, node_index thisNode)
{
    int32_t status = ENET_SOK;
    for (int i = 0; i < ENET_ARRAYSIZE(gStreamTable); i++)
    {
        streamTable_t* streamInfo = &gStreamTable[i];

        /* Configure as No Role. */
        streamInfo->role = ROLE_NONE;

        if (thisNode == streamInfo->source_device)
        {
            /* Configure as Talker. */
            streamInfo->role = ROLE_TALKER;

            /* Configure Talker. */
            controlData_configTalker(avtpcfData, streamInfo, \
                                                        &streamInfo->talkerCfg, &streamInfo->talkerObj);
            DebugP_log("Configuring stream Index %d as Talker\r\n", i);
        }

        if (thisNode == streamInfo->dest_device)
        {
            /* Check if its configured as talker already
                and configure as listener. */
            if (streamInfo->role == ROLE_TALKER)
            {
                DebugP_log("ERROR in Stream Configuration\r\n");
                DebugP_assert(false);
            }
            else
            {
                streamInfo->role = ROLE_LISTENER;

                /* Configure Listener. */
                controlData_configListener(avtpcfData, streamInfo, \
                                                   &streamInfo->listenerCfg, &streamInfo->listenerObj);

                /* Register Mcast to Host Port. */
                EnetApp_configMcastAddr(avtpcfData->enetInfo.hEnet, avtpcfData->enetInfo.coreId, \
                                            streamInfo->listenerCfg.mcastMac, streamInfo->vlanID, 0x01);

                DebugP_log("Configuring stream Index %d as Listener\r\n", i);
                DebugP_log("Registering Mac Address ");
                EnetAppUtils_printMacAddr(streamInfo->listenerCfg.mcastMac);
            }
        }

        if (streamInfo->role == ROLE_NONE)
        {
            /* What to do when no Role? */
        }
    }
    return status;
}

static int32_t controlData_configTalker(avtpcf_data_t* avtpcfData, const streamTable_t* streamInfo, \
                                    avtpcf_talker_cfg* cfg, avtpcf_talker_Obj* obj)
{
    int32_t status = ENET_SOK;
    cfg->priority = streamInfo->priority;
    cfg->vlanID   = streamInfo->vlanID;
    status = CtrlData_streamIdAtoI(streamInfo->streamID, cfg->streamID);
    if (status == ENET_SOK)
    {
        status = EnetAppUtils_macAddrAtoI(streamInfo->destMcastAddr, cfg->mcastMac);
        memcpy(cfg->srcMac, gEnetAppCfg.macAddr, sizeof(cfg->srcMac));
    }
    if (status == ENET_SOK)
    {
        status = avtpcf_talker_init(avtpcfData, obj, cfg);
    }
    return status;
}

static int32_t controlData_configListener(avtpcf_data_t* avtpcfData, streamTable_t* streamInfo, \
                                    avtpcf_listener_cfg* cfg, avtpcf_listener_Obj* obj)
{
    int32_t status = ENET_SOK;
    status = CtrlData_streamIdAtoI(streamInfo->streamID, cfg->streamID);
    if (status == ENET_SOK)
    {
        status = EnetAppUtils_macAddrAtoI(streamInfo->destMcastAddr, cfg->mcastMac);
    }
    DebugP_log("Subscribing to streamID %s\r\n", streamInfo->streamID);
    cfg->callback = streamInfo->rxCb;
    cfg->cbArgs   = (void*)streamInfo;

    if (status == ENET_SOK)
    {
        status = avtpcf_listener_init(avtpcfData, obj, cfg);
    }

    streamInfo->stats.prevSeqNumber = -1;
    streamInfo->stats.totalMissing  = 0;
    streamInfo->stats.totalReceived = 0;
    return status;
}

static int32_t avtpcf_listener1_cb(avtpcf_info *streamInfo, uint8_t *payload, int size, void *cbArgs)
{
    /* Get Stream ID. */
    streamTable_t *streamCfg = (streamTable_t*)cbArgs;

    uint32_t src = streamCfg->source_device;

    /* Lookup the Source and toggle accordingly. */
    gpio_cfg *outCfg = &gpioRemoteOutLut[src];
    DebugP_assert(outCfg->gpioBase > 0);

    gpio_acf_msg *msg   = (gpio_acf_msg*)payload;
    uint32_t pinState   = msg->acf_gpioState;

    if (pinState)
    {
        GPIO_pinWriteHigh(outCfg->gpioBase, outCfg->gpioPin);
    }
    else
    {
        GPIO_pinWriteLow(outCfg->gpioBase, outCfg->gpioPin);
    }

    if (streamCfg->stats.prevSeqNumber == -1)
    {
        streamCfg->stats.prevSeqNumber = 0;
    }
    else
    {
        /* Check the sequence Number. */
        int currNum = streamInfo->seqID;
        int prevNum = streamCfg->stats.prevSeqNumber;
        int diff = 0;
        if (currNum > prevNum)
        {
            diff = currNum-prevNum-1;
        }
        else if (prevNum > currNum)
        {
            diff = currNum+256-prevNum-1;
        }
        else
        {
            diff = 0;
        }

        if (diff != 0)
        {
            // DebugP_log("prev,curr,diff %d, %d, %d\r\n", prevNum, currNum, diff);
        }

        /* Add the Missed counter. */
        streamCfg->stats.totalMissing += diff;
        streamCfg->stats.totalReceived += 1;
        streamCfg->stats.prevSeqNumber = currNum;
    }
    return 0;
}

static void controlData_printStats(streamTable_t *streamCfg)
{
    DebugP_log("Stat: T,M, %d, %d\r\n", streamCfg->stats.totalReceived, streamCfg->stats.totalMissing);
}

static int32_t EnetApp_addVlanEntries(Enet_Handle hEnet, uint32_t coreId, uint32_t vlan)
{
        /* Add the original un-modified vlanId to ALE table */
        Enet_IoctlPrms prms;
        int32_t status = ENET_SOK;
        CpswAle_VlanEntryInfo vlanInArgs;
        uint32_t vlanOutArgs;

        /* Adding vlan entry for traffic generation */
        memset(&vlanInArgs, 0, sizeof(vlanInArgs));
        vlanInArgs.vlanIdInfo.vlanId        = vlan;
        vlanInArgs.vlanIdInfo.tagType       = ENET_VLAN_TAG_TYPE_INNER;
        vlanInArgs.vlanMemberList           = 0x7; /* allow for all ports */
        vlanInArgs.unregMcastFloodMask      = 0x6; /* except host port */
        vlanInArgs.regMcastFloodMask        = 0x7;
        vlanInArgs.forceUntaggedEgressMask  = 0U;
        vlanInArgs.noLearnMask              = 0U;
        vlanInArgs.vidIngressCheck          = false;
        vlanInArgs.limitIPNxtHdr            = false;
        vlanInArgs.disallowIPFrag  = false;

        ENET_IOCTL_SET_INOUT_ARGS(&prms, &vlanInArgs, &vlanOutArgs);
        EnetAppUtils_assert(status == ENET_SOK);
        ENET_IOCTL(hEnet, coreId, CPSW_ALE_IOCTL_ADD_VLAN, &prms, status);

        return status;
}

static int32_t EnetApp_configMcastAddr(Enet_Handle hEnet, uint32_t coreId, uint8_t *mcast, uint32_t vlanID, uint8_t portmask)
{
    /* Adding multicast entry for traffic generation */
    int32_t status = ENET_SOK;
    Enet_IoctlPrms prms;
    uint32_t setMcastoutArgs;

    CpswAle_SetMcastEntryInArgs setMcastInArgs = {
            .addr =
            {
                .vlanId = vlanID,
            },
            .info =
            {
                .portMask = portmask, /* allow only for host port */
                .super = false,
                .fwdState = CPSW_ALE_FWDSTLVL_FWD,
                .numIgnBits =0U,
            },
    };
    memcpy(&setMcastInArgs.addr.addr, mcast, sizeof(setMcastInArgs.addr.addr));
    ENET_IOCTL_SET_INOUT_ARGS(&prms, &setMcastInArgs, &setMcastoutArgs);
    ENET_IOCTL(hEnet,
               coreId,
               CPSW_ALE_IOCTL_ADD_MCAST,
               &prms,
               status);
    EnetAppUtils_assert(status == ENET_SOK);

    return status;
}

static int32_t EnetApp_applyClassifierMatch(Enet_Handle hEnet, uint32_t coreId, uint32_t vlanID, uint32_t rxFlowIdx)
{
    CpswAle_SetPolicerEntryInArgs inArgs;
    CpswAle_SetPolicerEntryOutArgs outArgs;
    Enet_IoctlPrms prms;
    int32_t status;

    memset(&inArgs, 0, sizeof(inArgs));

    inArgs.policerMatch.portIsTrunk = false;
    inArgs.threadIdEn = true;
    inArgs.threadId = rxFlowIdx;

    if (vlanID > 0)
    {
        inArgs.policerMatch.ivlanId = vlanID;
        inArgs.policerMatch.etherType = 0x22F0;
        inArgs.policerMatch.policerMatchEnMask = CPSW_ALE_POLICER_MATCH_IVLAN | CPSW_ALE_POLICER_MATCH_ETHERTYPE;
    }

    ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs, &outArgs);
    ENET_IOCTL(hEnet, coreId, CPSW_ALE_IOCTL_SET_POLICER, &prms, status);

    if (status != ENET_SOK)
    {
        EnetAppUtils_print("%s():CPSW_ALE_IOCTL_SET_POLICER %d\r\n", __func__, status);
    }

    return status;
}

static void controlData_setPinMux(void)
{
#if DEF_NODE_CENTRAL
    static Pinmux_PerCfg_t gPinMuxMainDomainCfg[] = {
        /* GPIO1_74 -> ADC0_AIN0 (F1) */
        {
            PIN_ADC0_AIN0,
            (PIN_MODE(7) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE)
        },
        /* GPIO1_75 -> ADC0_AIN1 (H1) */
        {
            PIN_ADC0_AIN1,
            (PIN_MODE(7) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE)
        },
        /* GPIO1_76 -> ADC0_AIN2 (K2) */
        {
            PIN_ADC0_AIN2,
            (PIN_MODE(7) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE)
        },
        {PINMUX_END, 0U}
    };
#else /* DEF_NODE_CENTRAL */
    static Pinmux_PerCfg_t gPinMuxMainDomainCfg[] = {
        /* GPIO1_74 -> ADC0_AIN0 (F1) */
        {
            PIN_ADC0_AIN0,
            (PIN_MODE(7) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE)
        },
        {PINMUX_END, 0U}
    };
#endif /* DEF_NODE_CENTRAL */

    Pinmux_config(gPinMuxMainDomainCfg, PINMUX_DOMAIN_ID_MAIN);

    /* Enable ADC input to be used as GPI pins. */
    SOC_setAdcGPIEnable();
}

static int32_t CtrlData_streamIdAtoI(const char *txt, uint8_t *addr)
{
    int32_t status = ENET_SOK;
    int8_t a, b, i;

    for (i = 0; i < 8; i++)
    {
        a = EnetAppUtils_hex2Num(*txt++);
        if (a < 0)
        {
            status = ENET_EFAIL;
        }

        b = EnetAppUtils_hex2Num(*txt++);
        if (b < 0)
        {
            status = ENET_EFAIL;
        }

        *addr++ = (a << 4) | b;

        if ((i < 7) && (*txt++ != ':'))
        {
            status = ENET_EFAIL;
            break;
        }
    }
    return status;
}

void controlData_tick(void* args)
{
    SemaphoreP_post(&gControlDataSem);
}
