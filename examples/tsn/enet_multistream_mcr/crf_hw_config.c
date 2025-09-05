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
 * \file   crf_hw_config.c
 *
 * \brief This file contains CRF Hw Configuration Functions.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "tsninit.h"
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include <board/ioexp/ioexp_tca6424.h>
#include <drivers/i2c.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/TaskP.h>
#include "enetapp_cpsw.h"
#include <drivers/soc.h>
#include <drivers/sciclient.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <drivers/pinmux.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#ifdef SOC_AM275X
#define TIMESYNC_ROUTER_DEVID   (TISCI_DEV_TIMESYNC_EVENT_INTROUTER0)
#endif

#define TS_ROUTER_IN_ATL_BWS_SEL_0          (20)
#define TS_ROUTER_IN_ATL_BWS_SEL_1          (21)
#define TS_ROUTER_IN_ATL_BWS_SEL_2          (22)
#define TS_ROUTER_IN_ATL_BWS_SEL_3          (23)

#define TS_ROUTER_OUT_CPSW_CPTS_HW_PUSH_1   (8)
#define TS_ROUTER_OUT_CPSW_CPTS_HW_PUSH_2   (9)
#define TS_ROUTER_OUT_CPSW_CPTS_HW_PUSH_3   (10)
#define TS_ROUTER_OUT_CPSW_CPTS_HW_PUSH_4   (11)
#define TS_ROUTER_OUT_CPSW_CPTS_HW_PUSH_5   (12)
#define TS_ROUTER_OUT_CPSW_CPTS_HW_PUSH_6   (13)
#define TS_ROUTER_OUT_CPSW_CPTS_HW_PUSH_7   (14)
#define TS_ROUTER_OUT_CPSW_CPTS_HW_PUSH_8   (15)

#define TS_ROUTER_OUT_SYNC0_OUT             (16)
#define TS_ROUTER_OUT_SYNC1_OUT             (17)
#define TS_ROUTER_OUT_SYNC2_OUT             (18)
#define TS_ROUTER_OUT_SYNC3_OUT             (19)

#define CTRL_MMR0_CFG0_ATL_BWS0_SEL         (0x001082C0)
#define CTRL_MMR0_CFG0_ATL_BWS1_SEL         (0x001082C4)
#define CTRL_MMR0_CFG0_ATL_BWS2_SEL         (0x001082C8)
#define CTRL_MMR0_CFG0_ATL_BWS3_SEL         (0x001082CC)

#define ATL_BWSX_MCASP0_AFSR_IN             (0b0000)
#define ATL_BWSX_MCASP1_AFSR_IN             (0b0001)
#define ATL_BWSX_MCASP2_AFSR_IN             (0b0010)
#define ATL_BWSX_MCASP3_AFSR_IN             (0b0011)
#define ATL_BWSX_MCASP4_AFSR_IN             (0b0100)
#define ATL_BWSX_MCASP0_AFSX_IN             (0b0101)
#define ATL_BWSX_MCASP1_AFSX_IN             (0b0110)
#define ATL_BWSX_MCASP2_AFSX_IN             (0b0111)
#define ATL_BWSX_MCASP3_AFSX_IN             (0b1000)
#define ATL_BWSX_MCASP4_AFSX_IN             (0b1001)
/* ========================================================================== */
/*                              Struct & Enums                                */
/* ========================================================================== */
typedef enum
{
    TS_ACQUIRE_STATE_RECORD_LF = 0,
    TS_ACQUIRE_STATE_RECORD_HF,
    TS_ACQUIRE_STATE_END
}tsAcquireState_e;

typedef struct
{
    tsAcquireState_e state; /* State of the statemachine. */
    SemaphoreP_Object StartSem;
    SemaphoreP_Object EndSem;
    int lfCbCount;
    int hfCbCount;
} tsAcquireStateVar;
/* ========================================================================== */
/*                              Global Variables                              */
/* ========================================================================== */

static tsAcquireStateVar gAcquireSm = {0};

/* This is used to access hEnet and Core ID.
   The variable name may vary from application to application.*/
extern EnetApp_Cfg gEnetAppCfg;

/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */

static int32_t crfHwConfig_SetPinMux(void)
{
    static Pinmux_PerCfg_t gPinMuxMainDomainCfg[] = {

        /*  pin config */
        /* GPIO1_30 -> EXT_REFCLK1 (B16) */
        {
            PIN_EXT_REFCLK1,
            ( PIN_MODE(1) )
        },

        {PINMUX_END, 0U}
    };

    Pinmux_config(gPinMuxMainDomainCfg, PINMUX_DOMAIN_ID_MAIN);
    return SystemP_SUCCESS;
}

static int32_t crfHwConfig_setTimeSyncRouter(const uint32_t src, const uint32_t dest, uint32_t set)
{
    int32_t                             retVal;

    if (set > 0)
    {
        struct tisci_msg_rm_irq_set_req     rmIrqReq;
        struct tisci_msg_rm_irq_set_resp    rmIrqResp;
        rmIrqReq.valid_params           = 0U;
        rmIrqReq.valid_params          |= TISCI_MSG_VALUE_RM_DST_ID_VALID;
        rmIrqReq.valid_params          |= TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID;
        rmIrqReq.global_event           = 0U;
        rmIrqReq.src_id                 = TIMESYNC_ROUTER_DEVID;
        rmIrqReq.src_index              = src;
        rmIrqReq.dst_id                 = TIMESYNC_ROUTER_DEVID;
        rmIrqReq.dst_host_irq           = dest;
        rmIrqReq.ia_id                  = 0U;
        rmIrqReq.vint                   = 0U;
        rmIrqReq.vint_status_bit_index  = 0U;
        rmIrqReq.secondary_host         = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;
        retVal = Sciclient_rmIrqSetRaw(&rmIrqReq, &rmIrqResp, SystemP_WAIT_FOREVER);
        if(0 != retVal)
        {
            DebugP_log("[Error] Sciclient event config failed!!!\r\n");
        }
    }
    else
    {
        struct tisci_msg_rm_irq_release_req rmIrqReleaseReq;
        rmIrqReleaseReq.valid_params           = 0;
        rmIrqReleaseReq.valid_params           |= TISCI_MSG_VALUE_RM_DST_ID_VALID;
        rmIrqReleaseReq.valid_params           |= TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID;
        rmIrqReleaseReq.src_id                 = TIMESYNC_ROUTER_DEVID;
        rmIrqReleaseReq.src_index              = src;
        rmIrqReleaseReq.dst_id                 = TIMESYNC_ROUTER_DEVID;
        rmIrqReleaseReq.dst_host_irq           = dest;
        rmIrqReleaseReq.ia_id                  = 0;
        rmIrqReleaseReq.vint                   = 0;
        rmIrqReleaseReq.global_event           = 0;
        rmIrqReleaseReq.vint_status_bit_index  = 0;
        rmIrqReleaseReq.secondary_host         = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;
        retVal = Sciclient_rmIrqReleaseRaw(&rmIrqReleaseReq, SystemP_WAIT_FOREVER);
        if(0 != retVal)
        {
            DebugP_log("[Error] Sciclient event config failed!!!\r\n");
        }
    }
    return retVal;
}

static void crfHwConfig_setAtlMux(const uint32_t atlBwsReg, uint32_t muxVal)
{
    volatile uint32_t *ctrl_mmr_addr = \
        (volatile uint32_t*)AddrTranslateP_getLocalAddr(atlBwsReg);

    /* Connect McASP1_AFSX to TS Router. */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, 2);
    *ctrl_mmr_addr = muxVal;
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, 2);
}

static int32_t crfHwConfig_ConfigExtClkMux(void)
{
    /*
        Configure the clock mux using IO Expander
        Ref: PROC190E1 (AM275-EVM Schematics)

        To connect RGMII1_BCLK to AUDIO_EXT_REFCLK2
        IO Expander Addr: 0x22
        AUDIO_EXT_REFCLK2_SO -> P24 -> 0(LOW)
        AUDIO_EXT_REFCLK2_S1 -> P25 -> 1(HIGH)
    */
   const uint32_t I2C_ADDRESS = 0x22;
   const uint32_t REFCLK2_S0  = (8 * 2) + 4; // P24
   const uint32_t REFCLK2_S1  = (8 * 2) + 5; // P25

    int32_t status;
    /* IO Expander Config Object */
    static TCA6424_Config IOExp_Config;

    TCA6424_Params  IOExp_Params = {
        .i2cInstance = CONFIG_I2C0,
        .i2cAddress  = I2C_ADDRESS,
    };
    /* open the IO Expander object */
    status = TCA6424_open(&IOExp_Config, &IOExp_Params);

    if (status == SystemP_SUCCESS)
    {
        status = TCA6424_config(&IOExp_Config,
            REFCLK2_S0, TCA6424_MODE_OUTPUT);
        status += TCA6424_config(&IOExp_Config,
            REFCLK2_S1, TCA6424_MODE_OUTPUT);
    }

    if (status == SystemP_SUCCESS)
    {
        status = TCA6424_setOutput(&IOExp_Config,
            REFCLK2_S0, TCA6424_OUT_STATE_LOW);
        status += TCA6424_setOutput(&IOExp_Config,
            REFCLK2_S1, TCA6424_OUT_STATE_HIGH);
    }

    TCA6424_close(&IOExp_Config);

    return status;
}

static int32_t crfHwConfig_registerCptsPushEvents(CpswCpts_HwPush pushNum, \
                                CpswCpts_HwPushNotifyCb hwPushNotifyCb, void* args)
{
    CpswCpts_RegisterHwPushCbInArgs inArgs;
    inArgs.hwPushNum = pushNum;
    inArgs.hwPushNotifyCb = hwPushNotifyCb;
    inArgs.hwPushNotifyCbArg = args;

    Enet_IoctlPrms prms;
    int32_t status = ENET_SOK;
    ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);
    ENET_IOCTL(gEnetAppCfg.hEnet, gEnetAppCfg.coreId, CPSW_CPTS_IOCTL_REGISTER_HWPUSH_CALLBACK, &prms, status);
    if(status != ENET_SOK)
    {
        DebugP_log("[Error] Failed to configure HWPUSH CALLBACK !!!\r\n");
        DebugP_assert(false);
    }
    return status;
}

static int32_t crfHwConfig_unRegisterCptsPushEvents(CpswCpts_HwPush pushNum)
{
    Enet_IoctlPrms prms;
    int32_t status = ENET_SOK;
    ENET_IOCTL_SET_IN_ARGS(&prms, &pushNum);
    ENET_IOCTL(gEnetAppCfg.hEnet, gEnetAppCfg.coreId, CPSW_CPTS_IOCTL_UNREGISTER_HWPUSH_CALLBACK, &prms, status);
    if(status != ENET_SOK)
    {
        DebugP_log("[Error] Failed to configure HWPUSH CALLBACK !!!\r\n");
        DebugP_assert(false);
    }

    return status;
}

static inline uint64_t crfHwConfig_absDiff(uint64_t num1, uint64_t num2)
{
    return (num1 > num2) ? (num1 - num2) : (num2 - num1);
}

static int64_t crfHwConfig_getEdgeDiff(uint64_t* topSigArr, uint64_t* bottomSigArr, \
                uint32_t topSigFreq, uint32_t bottomSigFreq, uint32_t maxJitter_ns)
{
    uint64_t topPeriod_ns = 1e9/(float)topSigFreq;
    uint64_t bottomPeriod_ns = 1e9/(float)bottomSigFreq;

    uint64_t goodTopSample = 0;
    uint64_t goodBottomSample = 0;

    /* Select good samples */
    for (int i = 0; i < 2; i++)
    {
        /* Calc diff */
        if (crfHwConfig_absDiff(topSigArr[i+1]-topSigArr[i], topPeriod_ns) < maxJitter_ns)
        {
            /* Good Samples!!*/
            goodTopSample = topSigArr[i];
        }
        /* Calc diff */
        if (crfHwConfig_absDiff(bottomSigArr[i+1]-bottomSigArr[i], bottomPeriod_ns) < maxJitter_ns)
        {
            /* Good Samples!!*/
            goodBottomSample = bottomSigArr[i];
        }
    }

    DebugP_assert(goodTopSample != 0);
    DebugP_assert(goodBottomSample != 0);
    DebugP_assert(goodTopSample > goodBottomSample);

    int64_t diff = goodTopSample - goodBottomSample;
    double cycleDiff = diff/(double)topPeriod_ns;
    int64_t floor = (int64_t)(cycleDiff);

    diff = (cycleDiff-floor) * topPeriod_ns;

    return diff;
}

static uint64_t crfHwConfig_getCptsHwPushEventTs(uint32_t CptsHwPushNum)
{
    CpswCpts_Event inArgs = {
        .eventType = CPSW_CPTS_EVENTTYPE_HW_TS_PUSH,
        .hwPushNum = CptsHwPushNum,
    };
    CpswCpts_Event outArgs;
    Enet_IoctlPrms prms;
    uint64_t timestamp = 0;
    int32_t status = ENET_SOK;
    ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs, &outArgs);
    ENET_IOCTL(gEnetAppCfg.hEnet, gEnetAppCfg.coreId, CPSW_CPTS_IOCTL_LOOKUP_EVENT, &prms, status);
    if(status != ENET_SOK)
    {
        DebugP_log("No Timstamp found\r\n");
    }
    else
    {
        timestamp = outArgs.tsVal;
    }
    return timestamp;
}

static void crfHwConfig_CptspushNotifyCb(void *hwPushNotifyCbArg, CpswCpts_HwPush hwPushNum)
{
    tsAcquireStateVar* sm = (tsAcquireStateVar*)&gAcquireSm;
    switch (sm->state)
    {
        case TS_ACQUIRE_STATE_RECORD_LF:
            if (hwPushNum == CPSW_CPTS_HWPUSH_1)
            {
                sm->lfCbCount += 1;
                if (sm->lfCbCount >= 3)
                {
                    sm->state = TS_ACQUIRE_STATE_RECORD_HF;
                    sm->hfCbCount = 0;
                    /* Post Start Semaphore. */
                    SemaphoreP_post(&sm->StartSem);
                }
            }
            break;
        case TS_ACQUIRE_STATE_RECORD_HF:
            if (hwPushNum == CPSW_CPTS_HWPUSH_2)
            {
                sm->hfCbCount += 1;
                if (sm->hfCbCount >= 3)
                {
                    sm->state = TS_ACQUIRE_STATE_END;
                    /* Post End Semaphore. */
                    SemaphoreP_post(&sm->EndSem);
                }
            }
            break;
        case TS_ACQUIRE_STATE_END:
        break;
        default:
        break;
    }
}

int32_t crfHwConfig_setup(void)
{
    int status;

    /* Select PHY Clock as EXT_REFCLK1 */
    status = crfHwConfig_ConfigExtClkMux();

    if (status == SystemP_SUCCESS)
    {
        /* Configure Pinmux to connect
         * EXT_REFCLK1 to GPIO1_30 */
        status = crfHwConfig_SetPinMux();
    }

    if (status == SystemP_SUCCESS)
    {
        /* Configure McASP.
           Todo: Move this to main task. */
        extern int32_t configure_mcasp();
        status = configure_mcasp();
    }

    if (status == SystemP_SUCCESS)
    {
        /* Configure ATL BWS0 and BWS1 to tap Timestamp Signal
           and Media Clock from McASP Respectively
           Todo: Make this configurable, take Mcasp input from arguments.
        */
        crfHwConfig_setAtlMux(CTRL_MMR0_CFG0_ATL_BWS0_SEL, ATL_BWSX_MCASP4_AFSX_IN);
        crfHwConfig_setAtlMux(CTRL_MMR0_CFG0_ATL_BWS1_SEL, ATL_BWSX_MCASP4_AFSR_IN);
    }

    if (status == SystemP_SUCCESS)
    {
        status = (SemaphoreP_constructBinary(&gAcquireSm.StartSem, 0) || \
                  SemaphoreP_constructBinary(&gAcquireSm.EndSem,   0));
        ENETTRACE_ERR_IF(status != SystemP_SUCCESS, "Semphore Construction Failed\r\n");
    }

    return status;
}

int64_t crfHwConfig_estimateEdgeDiff(double mediaClockFreq, double tsSignalFreq)
{
    /* Config Parameters. */
    uint32_t periodJitter   = 50;

    /* Clear up the pends to the Semaphores. */
    SemaphoreP_pend(&gAcquireSm.StartSem, SystemP_NO_WAIT);
    SemaphoreP_pend(&gAcquireSm.EndSem,   SystemP_NO_WAIT);

    crfHwConfig_registerCptsPushEvents(CPSW_CPTS_HWPUSH_1, crfHwConfig_CptspushNotifyCb, (void*)&gAcquireSm);
    crfHwConfig_registerCptsPushEvents(CPSW_CPTS_HWPUSH_2, crfHwConfig_CptspushNotifyCb, (void*)&gAcquireSm);

    /* Start Timestamping Signal Sampling. */
    crfHwConfig_setTimeSyncRouter(TS_ROUTER_IN_ATL_BWS_SEL_0, TS_ROUTER_OUT_CPSW_CPTS_HW_PUSH_1, 1);
    SemaphoreP_pend(&gAcquireSm.StartSem, SystemP_WAIT_FOREVER);
    /* Stop Timestamping Signal Sampling. */
    crfHwConfig_setTimeSyncRouter(TS_ROUTER_IN_ATL_BWS_SEL_0, TS_ROUTER_OUT_CPSW_CPTS_HW_PUSH_1, 0);

    /* Start the Media Clock Sampling. */
    crfHwConfig_setTimeSyncRouter(TS_ROUTER_IN_ATL_BWS_SEL_1, TS_ROUTER_OUT_CPSW_CPTS_HW_PUSH_2, 1);
    SemaphoreP_pend(&gAcquireSm.EndSem, SystemP_WAIT_FOREVER);
    /* Stop the Media Clock Sampling. */
    crfHwConfig_setTimeSyncRouter(TS_ROUTER_IN_ATL_BWS_SEL_1, TS_ROUTER_OUT_CPSW_CPTS_HW_PUSH_2, 0);

    /* Array to store Timestamping signal Edge Timestamps. */
    uint64_t tsSignalTimestamps[3] = {0, 0, 0};

    /* Array to store Media Clock Edge Timestamps. */
    uint64_t mediaClockTimestamps[3] = {0, 0, 0};

    for (int i = 0; i < 3; i++)
    {
        tsSignalTimestamps[i]   = crfHwConfig_getCptsHwPushEventTs(CPSW_CPTS_HWPUSH_1);
        mediaClockTimestamps[i] = crfHwConfig_getCptsHwPushEventTs(CPSW_CPTS_HWPUSH_2);
    }

    DebugP_log("LF: %llu, %llu, %llu\r\n", tsSignalTimestamps[0], tsSignalTimestamps[1], tsSignalTimestamps[2]);
    DebugP_log("HF: %llu, %llu, %llu\r\n", mediaClockTimestamps[0], mediaClockTimestamps[1], mediaClockTimestamps[2]);

    int64_t edgeDifference = crfHwConfig_getEdgeDiff(mediaClockTimestamps, tsSignalTimestamps, \
                                                    mediaClockFreq, tsSignalFreq, periodJitter);

    DebugP_log("Edge Difference = %lld, or %lld\r\n", edgeDifference, edgeDifference-20833);

    /* Unregister push event callbacks. */
    crfHwConfig_unRegisterCptsPushEvents(CPSW_CPTS_HWPUSH_1);
    crfHwConfig_unRegisterCptsPushEvents(CPSW_CPTS_HWPUSH_2);

    return edgeDifference;
}

int32_t crfHwConfig_routeTsSignalToPhy(void)
{
   return crfHwConfig_setTimeSyncRouter(TS_ROUTER_IN_ATL_BWS_SEL_0, TS_ROUTER_OUT_SYNC1_OUT, 1);
}