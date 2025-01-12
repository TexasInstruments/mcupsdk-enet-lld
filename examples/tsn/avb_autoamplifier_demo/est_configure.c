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

#include <stdint.h>
#include <stdbool.h>
#include "est_configure.h"
#include <tsn_unibase/unibase.h>
#include <tsn_combase/combase.h>
#include <tsn_combase/cb_tmevent.h>
#include <tsn_gptp/tilld/lld_gptp_private.h>
#include <tsn_unibase/unibase_binding.h>
#include <tsn_uniconf/yangs/yang_db_runtime.h>
#include <tsn_uniconf/yangs/yang_modules.h>
#include <tsn_uniconf/ucman.h>
#include <tsn_uniconf/uc_dbal.h>

#ifdef GPTP_ENABLED
#include <tsn_gptp/gptpmasterclock.h>
#endif
#include "debug_log.h"
#include "tsninit.h"
#include "common.h"
#include "qosapp_misc.h"
#include "aaf_pcm_app.h"

#define CLASSA_INTERVAL_OPEN_TIME_NS 2000
#define CLASSD1_INTERVAL_OPEN_TIME_NS 20000
#define ENETEST_TX_CLASSA_16CHANNELS_PAYLOAD_LEN (192)
#define MAX_BASE_TIME_US   20000000
/* 18: length of layer 2 header */
#define CALC_BITRATE_KBPS(pl_bytes, interval_us)  \
    (uint32_t)( (((uint64_t)(pl_bytes)+18)*8*UB_SEC_US)/ ((interval_us)*1000ULL) )

#define ADMIN_DELAY_OFFSET_FACTOR  (100000)

/*! Base path of admin list parameters in yang file of Qbv */
#define GATE_PARAM_TABLE_NODE "/ietf-interfaces/interfaces/interface|name:%s|" \
    "/bridge-port/gate-parameter-table"
#define GATE_CONTROL_ENTRY_NODE GATE_PARAM_TABLE_NODE   \
    "/admin-control-list/gate-control-entry"

/*! Base path of clock-state node in  yang file for checking PTP synchronized */
#define IEEE1588_PTP_TT_CLOCKSTATE_NODE  "/ieee1588-ptp-tt/ptp/instances" \
    "/instance|instance-index:0,0|/clock-state"

/*! Base path of port-state node in  yang file for checking PTP synchronized */
#define IEE1588_PTP_PORT_STATE_NODE  "/ieee1588-ptp-tt/ptp/instances" \
    "/instance|instance-index:0,0|/ports/port|port-index:%d|/port-ds"

#define TC_CLASS_NODE       "/ietf-interfaces/interfaces/interface|name:%s|/bridge-port/traffic-class/"
#define TC_CBS_ENABLED_STR  TC_CLASS_NODE"cbs-enabled"

typedef struct TimeSlot
{
    uint64_t start; /*! Expected start time for receiving packet */
    uint64_t end;   /*! Expected end time for receiving packet */
} TimeSlot_t;

typedef struct PerPriorityTimeSlot
{
    int32_t nLength; /*! Num of timeslots for each priority */
    TimeSlot_t timeSlots[ENET_TAS_MAX_CMD_LISTS]; /*! Timeslot for each priority */
} PerPriorityTimeSlot_t;

typedef struct EstStatsInfo
{
    uint64_t nGoodPkt;         /*! Num of packets received inside timeslot */
    uint64_t nBadPkt;          /*! Num of packets received outside timeslot */
} EstStatsInfo_t;

typedef struct EnetEstAppCtx
{
    EnetQoSApp_AppCtx_t appCtx;/*! Common context param is general for all QoS applications. */
    int schedIdx;              /*! Index of EST schedule applied for talker and listener. */
    EstStatsInfo_t estStatsInfo[QOSAPP_PRIORITY_MAX];
    /*! Expected timeslot for all priority traffic */
    PerPriorityTimeSlot_t exptTimeSlots[QOSAPP_PRIORITY_MAX];
} EnetEstAppCtx_t;

typedef struct EnetEstAppTestParam
{
    EnetTas_ControlList list;          /*! List of Admin param for EST */
    QoSAppStreamConfigParam_t stParam; /*! Streams parameters */
} EnetEstAppTestParam_t;

UB_SD_GETMEM_DEF_EXTERN(YANGINIT_GEN_SMEM);

static EnetEstAppCtx_t gEnetEstAppCtx;
SemaphoreP_Object gEstFinishedSem;

static void EnetEstApp_printAdminControlList(EnetTas_ControlList *list)
{
    uint8_t gateMask = 0U;
    uint32_t start = 0U;
    uint32_t end;
    uint32_t dur;
    uint32_t i;

    DPRINT("The following AdminList param will be configured for EST:");
    for (i = 0U; i < list->listLength; i++)
    {
        gateMask = list->gateCmdList[i].gateStateMask;
        dur = list->gateCmdList[i].timeInterval;
        end = start + dur - 1U;
        /* o = Gate open, C = Gate closed */
        DPRINT("GateMask[7..0]=%s%s%s%s%s%s%s%s (0x%02x), start=%u ns, end=%u ns, dur=%u ns",
               ENET_IS_BIT_SET(gateMask, 7U) ? "o" : "C",
               ENET_IS_BIT_SET(gateMask, 6U) ? "o" : "C",
               ENET_IS_BIT_SET(gateMask, 5U) ? "o" : "C",
               ENET_IS_BIT_SET(gateMask, 4U) ? "o" : "C",
               ENET_IS_BIT_SET(gateMask, 3U) ? "o" : "C",
               ENET_IS_BIT_SET(gateMask, 2U) ? "o" : "C",
               ENET_IS_BIT_SET(gateMask, 1U) ? "o" : "C",
               ENET_IS_BIT_SET(gateMask, 0U) ? "o" : "C",
               gateMask, start, end, dur);
        start += dur;
    }
    char buffer[MAX_LOG_LEN];
    snprintf(buffer, sizeof(buffer), "Base time=%lluns,Cycle time=%lluns",
             list->baseTime,list->cycleTime);
    DPRINT("%s", buffer);
}

static EnetEstAppTestParam_t gEnetEstAppTestLists[] =
{
    {
        .list =
        {
            .baseTime    = 0ULL,
            // Class A cycle 125us, open 10us
            // Class D cycle 1000us, open 125us
            .cycleTime   = 1000*UB_USEC_NS,
            .gateCmdList =
            {
                // Gate: (gptp gate7)                 7           3  2       0
                // 125us
                { .gateStateMask = ENET_TAS_GATE_MASK(1, 0, 0, 0, 1, 0, 0, 0),
                  .timeInterval =  CLASSA_INTERVAL_OPEN_TIME_NS
                },
                { .gateStateMask = ENET_TAS_GATE_MASK(1, 0, 0, 0, 0, 1, 0, 0),
                  .timeInterval =  (125*UB_USEC_NS - CLASSA_INTERVAL_OPEN_TIME_NS)
                },
                // 125us
                { .gateStateMask = ENET_TAS_GATE_MASK(1, 0, 0, 0, 1, 0, 0, 0),
                  .timeInterval =  CLASSA_INTERVAL_OPEN_TIME_NS
                },
                { .gateStateMask = ENET_TAS_GATE_MASK(1, 0, 0, 0, 0, 0, 0, 0),
                  .timeInterval =  (125*UB_USEC_NS - CLASSA_INTERVAL_OPEN_TIME_NS)
                },
                                // 125us
                { .gateStateMask = ENET_TAS_GATE_MASK(1, 0, 0, 0, 1, 0, 0, 0),
                  .timeInterval =  CLASSA_INTERVAL_OPEN_TIME_NS
                },
                { .gateStateMask = ENET_TAS_GATE_MASK(1, 0, 0, 0, 0, 0, 0, 0),
                  .timeInterval =  (125*UB_USEC_NS - CLASSA_INTERVAL_OPEN_TIME_NS)
                },
                                // 125us
                { .gateStateMask = ENET_TAS_GATE_MASK(1, 0, 0, 0, 1, 0, 0, 0),
                  .timeInterval =  CLASSA_INTERVAL_OPEN_TIME_NS
                },
                { .gateStateMask = ENET_TAS_GATE_MASK(1, 0, 0, 0, 0, 0, 0, 0),
                  .timeInterval =  (125*UB_USEC_NS - CLASSA_INTERVAL_OPEN_TIME_NS)
                },
                                // 125us
                { .gateStateMask = ENET_TAS_GATE_MASK(1, 0, 0, 0, 1, 0, 0, 0),
                  .timeInterval =  CLASSA_INTERVAL_OPEN_TIME_NS
                },
                { .gateStateMask = ENET_TAS_GATE_MASK(1, 0, 0, 0, 0, 0, 0, 0),
                  .timeInterval =  (125*UB_USEC_NS - CLASSA_INTERVAL_OPEN_TIME_NS)
                },
                                // 125us
                { .gateStateMask = ENET_TAS_GATE_MASK(1, 0, 0, 0, 1, 0, 0, 0),
                  .timeInterval =  CLASSA_INTERVAL_OPEN_TIME_NS
                },
                { .gateStateMask = ENET_TAS_GATE_MASK(1, 0, 0, 0, 0, 0, 0, 0),
                  .timeInterval =  (125*UB_USEC_NS - CLASSA_INTERVAL_OPEN_TIME_NS)
                },
                                // 125us
                { .gateStateMask = ENET_TAS_GATE_MASK(1, 0, 0, 0, 1, 0, 0, 0),
                  .timeInterval =  CLASSA_INTERVAL_OPEN_TIME_NS
                },
                { .gateStateMask = ENET_TAS_GATE_MASK(1, 0, 0, 0, 0, 0, 0, 0),
                  .timeInterval =  (125*UB_USEC_NS - CLASSA_INTERVAL_OPEN_TIME_NS)
                },
                                // 125us
                { .gateStateMask = ENET_TAS_GATE_MASK(1, 0, 0, 0, 1, 0, 0, 0),
                  .timeInterval =  CLASSA_INTERVAL_OPEN_TIME_NS
                },
                { .gateStateMask = ENET_TAS_GATE_MASK(1, 0, 0, 0, 0, 0, 0, 0),
                  .timeInterval =  (125*UB_USEC_NS - CLASSA_INTERVAL_OPEN_TIME_NS)
                },

            },
            .listLength = 16U,
        },
        .stParam =
        {
            .streamParams =
            {
                /* test appliction sends packet with interval 1000us */
                {.bitRateKbps = CALC_BITRATE_KBPS(ENETEST_TX_CLASSA_16CHANNELS_PAYLOAD_LEN, 125),
                 .payloadLen = ENETEST_TX_CLASSA_16CHANNELS_PAYLOAD_LEN, // AAF 16channels classA
                 .tc = 2,
                 .priority = 2,
                },
                /* test appliction sends packet with interval 125us */
                {.bitRateKbps = CALC_BITRATE_KBPS(ENETEST_TX_CLASSA_16CHANNELS_PAYLOAD_LEN, 125),
                 .payloadLen = ENETEST_TX_CLASSA_16CHANNELS_PAYLOAD_LEN, // AAF 16channels classA
                 .tc = 3,
                 .priority = 3,
                },
            },
            .nStreams = 2,
        }
    },
};

static int EnetEstApp_setAdminControlList(EnetTas_ControlList *list, char *ifname,
                                          yang_db_runtime_dataq_t *ydrd,
                                          uc_notice_data_t *ucntd)
{
    int i, err = 0;
    char buffer[MAX_KEY_SIZE];
    char val[MAX_VAL_SIZE];

    EnetEstApp_printAdminControlList(list);

    if (list->cycleTime > 0)
    {
        /* Expected unit is mircosecond. */
        uint32_t cycletime_numerator = list->cycleTime/1000U;
        uint32_t cycletime_denominator = 1000000UL;
        snprintf(buffer, sizeof(buffer),
                 GATE_PARAM_TABLE_NODE"/admin-cycle-time/numerator",
                 ifname);
        snprintf(val, sizeof(val), "%d", cycletime_numerator);
        YANGDB_RUNTIME_WRITE(buffer, val);

        snprintf(buffer, sizeof(buffer),
                 GATE_PARAM_TABLE_NODE"/admin-cycle-time/denominator",
                 ifname);
        snprintf(val, sizeof(val), "%d", cycletime_denominator);
        YANGDB_RUNTIME_WRITE(buffer, val);
    }

    uint32_t second = list->baseTime/1000000000ULL;
    uint32_t nanosecond = list->baseTime%1000000000ULL;
    snprintf(buffer, sizeof(buffer),
             GATE_PARAM_TABLE_NODE"/admin-base-time/seconds",
             ifname);
    snprintf(val, sizeof(val), "%d", second);
    YANGDB_RUNTIME_WRITE(buffer, val);

    snprintf(buffer, sizeof(buffer),
             GATE_PARAM_TABLE_NODE"/admin-base-time/nanoseconds",
             ifname);
    snprintf(val, sizeof(val), "%d", nanosecond);
    YANGDB_RUNTIME_WRITE(buffer, val);

    for (i = 0; i < list->listLength; i++)
    {
        snprintf(buffer, sizeof(buffer),
                 GATE_CONTROL_ENTRY_NODE"|index:%d|/operation-name",
                 ifname, i);
        strcpy(val, "set-gate-states");
        YANGDB_RUNTIME_WRITE(buffer, val);

        snprintf(buffer, sizeof(buffer),
                 GATE_CONTROL_ENTRY_NODE"|index:%d|/time-interval-value",
                 ifname, i);
        snprintf(val, sizeof(val), "%d",
                 list->gateCmdList[i].timeInterval);
        YANGDB_RUNTIME_WRITE(buffer, val);

        snprintf(buffer, sizeof(buffer),
                 GATE_CONTROL_ENTRY_NODE"|index:%d|/gate-states-value",
                 ifname, i);
        snprintf(val, sizeof(val), "%d",
                 list->gateCmdList[i].gateStateMask);
        YANGDB_RUNTIME_WRITE(buffer, val);
    }

    snprintf(buffer, sizeof(buffer), GATE_PARAM_TABLE_NODE"/gate-enabled", ifname);
    strcpy(val, "true");
    YANGDB_RUNTIME_WRITE(buffer, val);

    /* Trigger the uniconf to write parameters from DB to HW */
    err = yang_db_runtime_askaction(ydrd, ucntd);
    if (err != 0)
    {
        DPRINT("%s, Failed to trigger uniconf to enable EST", __func__);
    }

    return err;
}

static bool EnetEstApp_isPTPClockStateSync(EnetQoSApp_AppCtx_t *ctx,
                                           char *netdev)
{
    int err = -1;
    bool syncFlag = BFALSE;
    EnetApp_dbArgs dbarg;
    EnetApp_Ctx_t *ectx = ctx->ectx;

    err = EnetQoSApp_openDB(&dbarg, ectx->dbName, "w");
    if (err)
    {
        DPRINT("Failed to open DB!");
    }
    else
    {
        do
        {
            char buffer[MAX_KEY_SIZE];
            void *val = NULL;
            uint32_t vsize;
            uint8_t portState = 0;
            int8_t portIdx = EnetQoSApp_getPortIdx(ctx, netdev);
            DPRINT("portIdx=%d netdev %s\n", portIdx, netdev);
            DebugP_assert(portIdx >= 0 && portIdx < ctx->netdevSize);

            snprintf(buffer, sizeof(buffer),
                     IEEE1588_PTP_TT_CLOCKSTATE_NODE"/gmstate");
            err = yang_db_runtime_get_oneline(dbarg.ydrd, buffer, &val, &vsize);
            if (err == -1)
            {
                DPRINT("Failed to read %s from the DB!", buffer);
                break;
            }

            syncFlag = *(uint8_t *)val == 2? BTRUE: BFALSE;
            UB_SD_RELMEM(YANGINIT_GEN_SMEM, val);
            val  = NULL;

            if (!syncFlag)
            {
                break;
            }
            syncFlag = BFALSE;
            /* gPTP port index in the DB started from 1 */
            snprintf(buffer, sizeof(buffer),
                     IEE1588_PTP_PORT_STATE_NODE"/port-state", portIdx+1);
            err = yang_db_runtime_get_oneline(dbarg.ydrd, buffer, &val, &vsize);
            if (err == -1)
            {
                DPRINT("Failed to read %s ", buffer);
                break;
            }
            portState =  *(uint8_t *)val;
            UB_SD_RELMEM(YANGINIT_GEN_SMEM, val);
            val  = NULL;

            /* check ieee1588-ptp-tt.yang for description of portState */
            if (portState != 6 && portState != 9)
            {
                DPRINT("Current port-state: %d ", portState);
                break;
            }

            snprintf(buffer, sizeof(buffer), IEE1588_PTP_PORT_STATE_NODE"/as-capable", portIdx+1);
            err = yang_db_runtime_get_oneline(dbarg.ydrd, buffer, &val, &vsize);
            if (err == -1)
            {
                DPRINT("Failed to read %s ", buffer);
                break;
            }
            bool asCapable = *(uint8_t *)val? BTRUE: BFALSE;
            UB_SD_RELMEM(YANGINIT_GEN_SMEM, val);
            if ((portState == 6 || portState == 9) && asCapable)
            {
                syncFlag = BTRUE;
            }
            else if (portState == 9 && !asCapable)
            {
                syncFlag = BTRUE;
            }
        } while (0);

        EnetQoSApp_closeDB(&dbarg);
    }

    return syncFlag;
}

static int EnetEstApp_getAdminBaseTime(uint64_t *time)
{
    int res = 0;
    int64_t ts;
    // Since we are disabling gptpmasterclock_getts64 due to CPU load
    // temporarily using this API
    // TODO: replace by int64_t ts = gptpmasterclock_getts64();
    gptpmasterclock_get_domain_ts64(&ts, 0);
    if (ts < 0)
    {
        res = -1;
    }
    else
    {
        *time = ts;
    }
    return res;
}

static int EnetApp_enableCBS(yang_db_runtime_dataq_t *ydrd,
                            uc_notice_data_t *ucntd,
                            char *netdev)
{
    int err;
    char buffer[MAX_KEY_SIZE];
    char val[MAX_VAL_SIZE];
    DPRINT("%s", __func__);
    snprintf(buffer, sizeof(buffer),
             TRAFFIC_CLASS_NODE"/cbs-enabled",
             netdev);
    strcpy(val, "1");
    YANGDB_RUNTIME_WRITE(buffer, val);
    err = yang_db_runtime_askaction(ydrd, ucntd);
    if (err != 0)
    {
        DPRINT("%s, Failed to trigger uniconf to write idleSlope",
               __func__);
    }

    return err;
}

static int EnetEstApp_runSchedule(EnetQoSApp_AppCtx_t *ctx,
                                  EnetTas_ControlList *adminList,
                                  char *netdev)
{
    bool openDBSuccess = BFALSE;
    EnetApp_dbArgs dbarg;
    EnetApp_Ctx_t *ectx = (EnetApp_Ctx_t *)ctx->ectx;
    int err, i;


    do
    {
        err = EnetQoSApp_openDB(&dbarg, ectx->dbName, "w");
        if (err)
        {
            DPRINT("Failed to open DB!");
            break;
        }
        openDBSuccess = BTRUE;

        err = EnetApp_enableCBS(dbarg.ydrd, dbarg.ucntd, netdev);
        if (err)
        {
            DPRINT("Failed to set CBS enable");
            break;
        }

        CB_USLEEP(1000000);


        QoSAppCommonParam_t prm =
        {
            .netdev = netdev,
            /* initialize with invalid TC value */
            {-1, -1, -1, -1, -1, -1, -1, -1},
            .nTCs = QOSAPP_PRIORITY_MAX,
            .nQueues = QOSAPP_PRIORITY_MAX,
        };
        for (i = 0; i < prm.nTCs; i++)
        {
            /* Use one-to-one mapping between TC and priority */
            prm.priority2TcMapping[i] = i;
        }

        err = EnetQoSApp_setCommonParam(&prm, &dbarg);
        if (err)
        {
            DPRINT("Failed to set EST common param!");
            break;
        }

        if (EnetEstApp_getAdminBaseTime(&adminList->baseTime) == 0)
        {
            /* Add a delay time to allow the admin list scheduled in the future
            * the offset should be large enough to have  both EST schedules from
            * talker and listener started at the same time
            */
            int64_t offset = ADMIN_DELAY_OFFSET_FACTOR*adminList->cycleTime;
            adminList->baseTime = ((adminList->baseTime+offset)/offset)*offset;
            adminList->baseTime = (adminList->baseTime > (uint64_t)((uint64_t)MAX_BASE_TIME_US*1000)) ? (uint64_t)((uint64_t)MAX_BASE_TIME_US*1000) : adminList->baseTime;
            ctx->adminDelayOffset = offset/1000; /* Convert to microsecond */
            ctx->adminDelayOffset = (ctx->adminDelayOffset > MAX_BASE_TIME_US) ? MAX_BASE_TIME_US : ctx->adminDelayOffset;
        }
        err = EnetEstApp_setAdminControlList(adminList,
                                             netdev,
                                             dbarg.ydrd, dbarg.ucntd);
        if (err)
        {
            DPRINT("Failed to set admin control list for %s",
                   netdev);
            break;
        }
        else
        {
            DPRINT("Set admin control list succesfully");
        }
    } while (0);

    if (openDBSuccess)
    {
        EnetQoSApp_closeDB(&dbarg);
    }

    return err;
}

void init_est()
{
    SemaphoreP_constructBinary(&gEstFinishedSem, 0);
}

void est_schedule(EnetApp_ModuleCtx_t *modCtx)
{
    int err;
    EnetQoSApp_AppCtx_t *ctx =  (EnetQoSApp_AppCtx_t *)&gEnetEstAppCtx;
    ctx->ectx = (EnetApp_Ctx_t *)modCtx->appCtx;
    ctx->talker.vid = 110;
    ctx->talker.nStreams = QOSAPP_NUM_OF_STREAMS;
    ctx->talker.nTCs = QOSAPP_NUM_OF_STREAMS;
    for (int i = 0; i < ((EnetApp_Ctx_t *)ctx->ectx)->netdevSize; i++)
    {
        ctx->netdev[i] = ((EnetApp_Ctx_t *)ctx->ectx)->netdev[i];
    }
    ctx->netdevSize = ((EnetApp_Ctx_t *)ctx->ectx)->netdevSize;

    while (!EnetEstApp_isPTPClockStateSync(ctx, ctx->netdev[0]))
    {
        DPRINT("Waiting for PTP clock to be synchronized!");
        CB_USLEEP(1000000ULL);
    }

    int schedIdx = 0;
    err = EnetEstApp_runSchedule(ctx,
                                 &gEnetEstAppTestLists[schedIdx].list,
                                 ctx->netdev[0]);
    if (err == 0)
    {
        // CB_USLEEP(ctx->adminDelayOffset); // 20s?
        CB_USLEEP(20000000); // 20s?
        DPRINT("Waiting EST Setting done");

        start_hw_timer();

        // trigger for talker/listener start
        SemaphoreP_post(&gEstFinishedSem);
    }
}

void wait_est_configured()
{
    int32_t count = 0;
    while(true)
    {
        count = SemaphoreP_getCount(&gEstFinishedSem);
        if (count > 0)
        {
            break;
        }
        CB_USLEEP(10000);
    }
    DPRINT("%s: done", __func__);
}