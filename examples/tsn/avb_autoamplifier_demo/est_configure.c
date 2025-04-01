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
#include <tsn_uniconf/yangs/yang_modules.h>
#include <tsn_uniconf/ucman.h>
#include <tsn_uniconf/uc_dbal.h>
#include <tsn_uniconf/yangs/cores/ieee1588-ptp-tt_access.h>

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

extern uint8_t IETF_INTERFACES_func(uc_dbald *dbald);
#define IETF_INTERFACES_RW IETF_INTERFACES_func(dbald)

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
                                          uc_notice_data_t *ucntd)
{
    int i, err;

    EnetEstApp_printAdminControlList(list);

    uint8_t kn_traffic_sched[5] = {
		[0] = IETF_INTERFACES_BRIDGE_PORT,
		[1] = IETF_INTERFACES_GATE_PARAMETER_TABLE,
	};
    uint8_t kn_traffic_sched_size = 0;

    //ietf-interfaces/interfaces/interface/bridge-port/gate-parameter-table/admin-cycle-time
    if (list->cycleTime > 0)
    {
        /* Expected unit is mircosecond. */
        uint32_t cycletime_numerator = list->cycleTime/1000U;
        uint32_t cycletime_denominator = 1000000UL;

        kn_traffic_sched[2]=IETF_INTERFACES_ADMIN_CYCLE_TIME;
	    kn_traffic_sched[3]=IETF_INTERFACES_NUMERATOR;
        kn_traffic_sched_size = 4;
        err=YDBI_SET_ITEM(ifknvk0, ifname,
			    kn_traffic_sched, kn_traffic_sched_size,
			    YDBI_CONFIG,
                (void *)&cycletime_numerator, sizeof(cycletime_numerator), 
                YDBI_NO_NOTICE);
        DebugP_assert(err == 0);

        kn_traffic_sched[3]=IETF_INTERFACES_DENOMINATOR;
        err=YDBI_SET_ITEM(ifknvk0, ifname,
			    kn_traffic_sched, kn_traffic_sched_size,
			    YDBI_CONFIG,
                (void *)&cycletime_denominator, sizeof(cycletime_denominator), 
                YDBI_NO_NOTICE);
        DebugP_assert(err == 0);
    }

    uint32_t second = list->baseTime/1000000000ULL;
    uint32_t nanosecond = list->baseTime%1000000000ULL;

    kn_traffic_sched[2]=IETF_INTERFACES_ADMIN_BASE_TIME;
	kn_traffic_sched[3]=IETF_INTERFACES_SECONDS;
    err=YDBI_SET_ITEM(ifknvk0, ifname,
			    kn_traffic_sched, kn_traffic_sched_size,
			    YDBI_CONFIG,
                (void *)&second, sizeof(second), 
                YDBI_NO_NOTICE);
    DebugP_assert(err == 0);

    kn_traffic_sched[3]=IETF_INTERFACES_NANOSECONDS;
    err=YDBI_SET_ITEM(ifknvk0, ifname,
			    kn_traffic_sched, kn_traffic_sched_size,
			    YDBI_CONFIG,
                (void *)&nanosecond, sizeof(nanosecond), 
                YDBI_NO_NOTICE);
    DebugP_assert(err == 0);

    //ietf-interfaces/interfaces/interface/bridge-port/gate-parameter-table/admin-control-list/gate-control-entry
    for (i = 0; i < list->listLength; i++)
    {
        kn_traffic_sched[2]=IETF_INTERFACES_ADMIN_CONTROL_LIST;
        kn_traffic_sched[3]=IETF_INTERFACES_GATE_CONTROL_ENTRY;
        kn_traffic_sched[4]=IETF_INTERFACES_OPERATION_NAME;
        kn_traffic_sched_size=5u;
        uint32_t gate_operation=0x0; // {"dot1q-types", "set-gate-states"       , 0x0}
        err=YDBI_SET_ITEM(ifknvk1, ifname, i, 4u,
			    kn_traffic_sched, kn_traffic_sched_size,
			    YDBI_CONFIG, (void*)&gate_operation, sizeof(gate_operation), YDBI_NO_NOTICE);
        DebugP_assert(err == 0);

        kn_traffic_sched[4]=IETF_INTERFACES_TIME_INTERVAL_VALUE;
        err=YDBI_SET_ITEM(ifknvk1, ifname, i, 4u,
			    kn_traffic_sched, kn_traffic_sched_size,
			    YDBI_CONFIG, 
                (void*)&list->gateCmdList[i].timeInterval, sizeof(list->gateCmdList[i].timeInterval),
                YDBI_NO_NOTICE);
        DebugP_assert(err == 0);

        kn_traffic_sched[4]=IETF_INTERFACES_GATE_STATES_VALUE;
        err=YDBI_SET_ITEM(ifknvk1, ifname, i, 4u,
			    kn_traffic_sched, kn_traffic_sched_size,
			    YDBI_CONFIG, 
                (void*)&list->gateCmdList[i].gateStateMask, sizeof(list->gateCmdList[i].gateStateMask),
                YDBI_NO_NOTICE);
        DebugP_assert(err == 0);
    }

    // /ietf-interfaces/interfaces/interface|name:%s|/bridge-port/gate-parameter-table/gate-enabled
    kn_traffic_sched[2]=IETF_INTERFACES_GATE_ENABLED;
    kn_traffic_sched_size=3u;
    bool enable=1;
    err=YDBI_SET_ITEM(ifk3vk0, ifname,
			    IETF_INTERFACES_BRIDGE_PORT,
                IETF_INTERFACES_GATE_PARAMETER_TABLE,
                IETF_INTERFACES_GATE_ENABLED,
			    YDBI_CONFIG,
                (void *)&enable, sizeof(enable), 
                YDBI_NO_NOTICE);
    DebugP_assert(err == 0);

    /* Trigger the uniconf to write parameters from DB to HW */
    uc_dbald * dbald = ydbi_access_handle()->dbald;
    void *kvs[]={(void*)ifname, NULL, NULL};
	uint8_t kss[]={strlen(ifname)+1, 0};
    uint8_t aps[]={IETF_INTERFACES_RW,
		IETF_INTERFACES_INTERFACES,
		IETF_INTERFACES_INTERFACE,
		IETF_INTERFACES_BRIDGE_PORT,
		IETF_INTERFACES_GATE_PARAMETER_TABLE,
        IETF_INTERFACES_GATE_ENABLED,
        255u,
	};
    err=uc_nc_askaction_push(ucntd, dbald, aps, kvs, kss);
    if (err!=0)
    {
        DPRINT("uc_nc_askaction_push failed. err=%d\n", err);
    } 
    else 
    {
        DPRINT("%s: succeeded \n", __func__);
    }

    return 0;
}

static bool EnetEstApp_isPTPClockStateSync(EnetQoSApp_AppCtx_t *ctx,
                                           char *netdev)
{
    int err = -1;
    bool syncFlag = false;
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
            void *val = NULL;
            uint8_t portState = 0; // 6: master port, 9: slave port
            uint32_t gmState=0; //0: no sync, 1: sync, 2: sync stable
            bool asCapable=false;
            uint16_t portIdx = EnetQoSApp_getPortIdx(ctx, netdev);
            DebugP_assert(portIdx >= 0 && portIdx < ctx->netdevSize);

            int gdi=ydbi_gptpinstdomain2dbinst_pt(ydbi_access_handle(), 0, 0);
            YDBI_GET_ITEM_INTSUBST(ptk3vk0, gmState, val, gdi,
			       IEEE1588_PTP_TT_CLOCK_STATE, IEEE1588_PTP_TT_GMSTATE, 255,
			       YDBI_STATUS);

            syncFlag = (gmState == 1 || gmState==2) ? BTRUE: BFALSE;
            if (!syncFlag)
            {
                break;
            }
            syncFlag = false;
            /* gPTP port index in the DB started from 1 */
            portIdx+=1;
            YDBI_GET_ITEM_INTSUBST(ptk4vk1, portState, val, gdi,
                    IEEE1588_PTP_TT_PORTS, IEEE1588_PTP_TT_PORT,
                    IEEE1588_PTP_TT_PORT_DS, IEEE1588_PTP_TT_PORT_STATE,
                    &portIdx, sizeof(uint16_t), YDBI_STATUS);

            /* check ieee1588-ptp-tt.yang for description of portState */
            if (portState != 6 && portState != 9)
            {
                DPRINT("Current port-state: %d ", portState);
                break;
            }

            asCapable=ydbi_get_asCapable(ydbi_access_handle(), 0, 0, portIdx);
            if ((portState == 6 || portState == 9) && asCapable)
            {
                DPRINT("ptpSync-ed: %d ", portState);
                syncFlag = true;
            }
            else if (portState == 9 && !asCapable)
            {
                DPRINT("ptpSync-ed: %d ", portState);
                syncFlag = true;
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

static int EnetApp_enableCBS(uc_dbald *dbald,
                            uc_notice_data_t *ucntd,
                            char *ifname)
{
    int err;
    uint8_t kn_traffic_sched[5] = {
		[0] = IETF_INTERFACES_BRIDGE_PORT,
		[1] = IETF_INTERFACES_TRAFFIC_CLASS,
        [2] = IETF_INTERFACES_CBS_ENABLED,
	};
    uint8_t kn_traffic_sched_size = 3;
    // "/ietf-interfaces/interfaces/interface|name:%s|/bridge-port/traffic-class/cbs-enabled"
    bool cbs_enabled=true;
    err=YDBI_SET_ITEM(ifknvk0, ifname,
			    kn_traffic_sched, kn_traffic_sched_size,
			    YDBI_CONFIG,
                (void *)&cbs_enabled, sizeof(cbs_enabled), 
                YDBI_NO_NOTICE);
    DebugP_assert(err == 0);

    void *kvs[]={(void*)ifname, NULL, NULL};
	uint8_t kss[]={strlen(ifname)+1, 0};
    uint8_t aps[]={IETF_INTERFACES_RW,
		IETF_INTERFACES_INTERFACES,
		IETF_INTERFACES_INTERFACE,
		IETF_INTERFACES_BRIDGE_PORT,
		IETF_INTERFACES_TRAFFIC_CLASS,
        IETF_INTERFACES_CBS_ENABLED,
        255u,
	};
    err=uc_nc_askaction_push(ucntd, dbald, aps, kvs, kss);
    if (err!=0)
    {
        DPRINT("uc_nc_askaction_push failed. err=%d\n", err);
    } 
    else 
    {
        DPRINT("%s: succeeded \n", __func__);
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

        err = EnetApp_enableCBS(dbarg.dbald, dbarg.ucntd, netdev);
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
                                             dbarg.ucntd);
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
