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
 * \file  tsnapp_cpsw_main.c
 *
 * \brief This file contains the implementation of the Enet TSN example entry
 *        point
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include <kernel/dpl/DebugP.h>
#include "nrt_flow/dataflow.h"
#include "debug_log.h"
#include "tsninit.h"
#include "enetapp_cpsw.h"
#include "Enetapp_common.h"
#include <drivers/pinmux.h>
#include "crf_app.h"
#include <enet_apputils.h>
#include <drivers/ipc_notify.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define ENETAPP_CRF_TASK_PRIORITY       (5)
#define ENETAPP_AVB_TASK_PRIORITY      (16)
#define ENETAPP_CONTROL_TASK_PRIORITY  (20)

/* ========================================================================== */
/*                           Global Variables                                 */
/* ========================================================================== */

extern EnetApp_Cfg gEnetAppCfg;

static node_info gAllNodes[NODE_TOTAL_NODES] = {
    [NODE_CENTRAL] = {.nodeName = "Central Compute Node" },
    [NODE_LEFT]    = {.nodeName = "Zone Left Node"       },
    [NODE_RIGHT]   = {.nodeName = "Zone Right Node"      },
    [NODE_TAIL]    = {.nodeName = "Zone Tail Node"       },
};

node_index gNodeIndex = NODE_INVALID;

static uint8_t ctrlData_taskStack[TSN_TSK_STACK_SIZE]
                    __attribute__ ((aligned(TSN_TSK_STACK_ALIGN)));

static uint8_t audioData_taskStack[TSN_TSK_STACK_SIZE]
                    __attribute__ ((aligned(TSN_TSK_STACK_ALIGN)));

static uint8_t crf_taskStack[TSN_TSK_STACK_SIZE]
                    __attribute__ ((aligned(TSN_TSK_STACK_ALIGN)));

static TaskP_Object ctrlData_TaskObj;

static TaskP_Object audioData_TaskObj;

static TaskP_Object crf_TaskObj;

/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */

static void EnetApp_setPinmux(void);

static void EnetApp_createControlDataTask(void);

static void EnetApp_configureAmplifierRole(void);

static void EnetApp_createAudioDataTask(void);

static void EnetApp_createCrfTask(void);

int32_t EnetApp_addMcastEntry(uint8_t *mcastAddr, uint8_t portMask, uint16_t vlanId);

#ifndef DEF_NODE_CENTRAL
int32_t AutoAmp_playbackMcaspConfig(void);
#endif

#ifdef GPTP_QUICKSYNC
#ifdef GPTP_SLAVE
int EnetApp_adjustTimeInterval();
#endif
#endif

/* ========================================================================== */
/*                           Function Definitions                             */
/* ========================================================================== */

void EnetApp_mainTask(void *args)
{
    EnetPer_AttachCoreOutArgs attachCoreOutArgs;
    EnetApp_HandleInfo handleInfo;
    int status = SystemP_SUCCESS;
#ifdef GPTP_QUICKSYNC
#ifdef GPTP_SLAVE
    bool adjustedTimeInterval = false;
#endif
#endif
    DebugP_log("=====================================\r\n");
    DebugP_log("       AVB Audio Playback Demo       \r\n");
    DebugP_log("=====================================\r\n");

    /* Take input from the user. */
    EnetApp_configureAmplifierRole();

    /* Init Enet Stack Configuration. */
    EnetApp_initAppCfg(&attachCoreOutArgs, &handleInfo);

    /* Create Background Traffic Task. */
    EnetApp_createRxTask();

    /* Set pinmux for this Demo. */
    EnetApp_setPinmux();

    if (status == SystemP_SUCCESS)
    {
        status = Board_codecConfig();
    }

    if (status == SystemP_SUCCESS)
    {
        status = Board_CdceConfig();
    }

    if (status == SystemP_SUCCESS)
    {
        /* Setup McASP4 for Timestamping. */
        status = Board_MuxSelMcASP4();
    }

    if (status == SystemP_SUCCESS)
    {
        /* Init and Start TSN Tasks. (gPTP) */
        status = EnetApp_initTsn();
    }

    /* Wait for gPTP to be Initialized. */
    while(gptpmasterclock_init(NULL))
    {
        DebugP_log("Waiting for tsn_gptpd to be initialzied...\n");
        ClockP_usleep(100000);
    }

    /* Update PTP Multicast Address. */
    uint8_t ptpMcastAddr[] = {0x01, 0x80, 0xC2, 0x00, 0x00, 0x0E};
    EnetApp_addMcastEntry(ptpMcastAddr, 0x01, 0);

    /* Configure AVB Stack. */
    EnetApp_createAudioDataTask();

    /*  Configure Timestamping McASP.
     *  Use different index for different apps */
    if (gNodeIndex == NODE_CENTRAL)
    {
        status |= TsMcasp_tsMcaspConfig(0);

        /* Wait for C7x Core to Start. */
        DebugP_log("Waiting for C7x Core......\r\n");
        IpcNotify_syncAll(SystemP_WAIT_FOREVER);
        DebugP_log("C7x Core Started......\r\n");
    }
    else
    {
        status |= TsMcasp_tsMcaspConfig(1);

#ifndef DEF_NODE_CENTRAL
        status |= AutoAmp_playbackMcaspConfig();
#endif
    }

    bool linkStatus = false;
    do
    {
        linkStatus = (EnetAppUtils_isPortLinkUp(gEnetAppCfg.hEnet, \
                            gEnetAppCfg.coreId, ENET_MAC_PORT_1) && \
                      EnetAppUtils_isPortLinkUp(gEnetAppCfg.hEnet, \
                            gEnetAppCfg.coreId, ENET_MAC_PORT_2));
        DebugP_log("Wait for Linkup......\r\n");
        ClockP_usleep(500000);
    } while (!linkStatus);

    ClockP_sleep(3);

    crfApp_initTask();

    for (int i = 0; i < (GPTP_SYNC_WAIT_TIME_SEC/3); i++)
    {
        /* Wait for gPTP Sync */
        DebugP_log("Waiting for gPTP sync....\r\n");
        ClockP_sleep(3);
    }

    EnetApp_createCrfTask();

    /* Create Control Data Task. */
    EnetApp_createControlDataTask();

    uint64_t prevPrintTime = ClockP_getTimeUsec();
    while (true)
    {
        /* Print CPU load */
        ClockP_usleep(1000);
#ifdef GPTP_QUICKSYNC
#ifdef GPTP_SLAVE
        if (!adjustedTimeInterval)
        {
            if (EnetApp_adjustTimeInterval() == 0)
            {
                /* no need to adjust again */
                adjustedTimeInterval = true;
            }
        }
#endif
#endif
        if (ClockP_getTimeUsec() - prevPrintTime > 1000*1000)
        {
            prevPrintTime = ClockP_getTimeUsec();
            // EnetApp_printStats(prevPrintTime);
            (void)prevPrintTime;
        }
        EnetApp_printCpuLoad();
        TaskP_yield();
    }

    EnetApp_stopTsn();
    EnetApp_deInitTsn();
}

static void EnetApp_configureAmplifierRole(void)
{
    int nodeId = -1;
    do {
        EnetAppUtils_print("\r\n");
        for (int i = 0; i < NODE_TOTAL_NODES; i++)
        {
            EnetAppUtils_print("%d - %s\r\n", i, gAllNodes[i].nodeName);
        }
        EnetAppUtils_print("Enter the nodeId : \r\n");
        DebugP_scanf("%d", &nodeId);
        DebugP_log("Got Node Id: %d\r\n", nodeId);

        if ((nodeId < 0) || (nodeId > 4))
        {
            DebugP_log("Invalid Node Id: %d, Please enter between [0-4]\r\n", nodeId);
            nodeId = -1;
        }
    } while (nodeId == -1);

    gNodeIndex = nodeId;

    EnetAppUtils_print("Acting as %s\r\n", gAllNodes[gNodeIndex].nodeName);
}

static void EnetApp_createControlDataTask(void)
{
    TaskP_Params params = {
        .name      = "Control_Task",
        .stack     = ctrlData_taskStack,
        .stackSize = sizeof(ctrlData_taskStack),
        .priority  = ENETAPP_CONTROL_TASK_PRIORITY,
        .taskMain  = EnetApp_configControlData,
        .args      = NULL,
    };

    TaskP_construct(&ctrlData_TaskObj, &params);
}

static void EnetApp_createAudioDataTask(void)
{
    TaskP_Params params = {
        .name      =  "Audio_Task",
        .stack     = audioData_taskStack,
        .stackSize = sizeof(audioData_taskStack),
        .priority  = ENETAPP_AVB_TASK_PRIORITY,
        .taskMain  = aaf_audio_task,
        .args      = NULL,
    };

    TaskP_construct(&audioData_TaskObj, &params);
}

static void EnetApp_createCrfTask(void)
{
    TaskP_Params params = {
        .name      =  "Crf_Task",
        .stack     = crf_taskStack,
        .stackSize = sizeof(crf_taskStack),
        .priority  = ENETAPP_CRF_TASK_PRIORITY,
        .taskMain  = crfApp_runTask,
        .args      = NULL,
    };

    TaskP_construct(&crf_TaskObj, &params);
}

static void EnetApp_setPinmux(void)
{
    /* Fix I2C Pinmux Config. */
    #if defined (SOC_AM275X)
    Pinmux_PerCfg_t gPinMuxMainDomainCfg[] =
    {
        {
            PIN_GPIO1_72,
            ( PIN_MODE(1) | PIN_INPUT_ENABLE | PIN_PULL_DIRECTION  )
        },
        {
            PIN_EXT_REFCLK1,
            ( PIN_MODE(1) )
        },
        {PINMUX_END, 0U}
    };

    Pinmux_config(gPinMuxMainDomainCfg, PINMUX_DOMAIN_ID_MAIN);
    #endif
}

int32_t EnetApp_addMcastEntry(uint8_t *mcastAddr, uint8_t portMask, uint16_t vlanId)
{
    /* Adding multicast entry for ptp to update the mask only to hostPort */
    int32_t status = ENET_SOK;
    Enet_IoctlPrms prms;
    uint32_t setMcastoutArgs;
    CpswAle_SetMcastEntryInArgs setMcastPtpInArgs = {
        .addr = {.vlanId  = vlanId},
        .info =
        {
            .portMask = portMask,
            .super = false,
            .fwdState = CPSW_ALE_FWDSTLVL_FWD,
            .numIgnBits =0U,
        },
    };

    if (mcastAddr != NULL)
    {
        memcpy(setMcastPtpInArgs.addr.addr, mcastAddr, sizeof(setMcastPtpInArgs.addr.addr));
    }

    ENET_IOCTL_SET_INOUT_ARGS(&prms, &setMcastPtpInArgs, &setMcastoutArgs);
    ENET_IOCTL(gEnetAppCfg.hEnet, gEnetAppCfg.coreId,
            CPSW_ALE_IOCTL_ADD_MCAST, &prms, status);

    return status;
}

