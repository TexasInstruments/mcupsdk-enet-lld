/*
 *  Copyright (c) Texas Instruments Incorporated 2023
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
#include <tsn_uniconf/yangs/yang_modules.h>
#include <tsn_uniconf/yangs/yang_db_access.h>
#include <tsn_uniconf/yangs/ietf-interfaces_access.h>
#include <tsn_uniconf/yangs/cores/ieee802-dot1q-bridge_access.h>
#include <tsn_uniconf/yangs/cores/ieee1588-ptp-tt_access.h>
#include <tsn_uniconf/ucman.h>
#include <tsn_uniconf/uc_dbal.h>
#include <xmrpd/xmrpdconf/mrpgcfg.h>
#include <xmrpd/mrpman.h>
#include <xmrpd/mrp_map.h>
#include "debug_log.h"
#include "tsninit.h"
#include "common.h"

#include "avtp_xmrpd.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define AVTPD_TASK_PRIORITY     (2)
#define TALKER_TASK_PRIORITY    (2)
#define LISTENER_TASK_PRIORITY  (2)
#define XMRPD_TASK_PRIORITY     (2)
#define XMRPC_TASK_PRIORITY     (2)

// Currently MRP is working with 1 talker/1 listener
#define AVTPD_TASK_NAME         "avtpd_task"
#define TALKER_TASK_NAME        "talker_task"
#define LISTENER_TASK_NAME      "listener_task"
#define XMRPD_TASK_NAME         "xmrpd_task"
#define XMRP_CLIENT_TASK_NAME   "xmrpd_client_task"

#define MVRP_APPS_NO            1
#define MSRP_DOMAINS_NO         1
#define MSRP_TA_NO              1

#define BASE_VID 100
#define XMRPD_LINKSEMNAME "/xmrplinksem"

#define TESTING_PORT_INDEX 0 // Corresponding to tilld0

extern uint8_t IETF_INTERFACES_func(uc_dbald *dbald);
#define IETF_INTERFACES_RW IETF_INTERFACES_func(dbald)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */
typedef struct
{
    uint8_t tcNum;                  // traffic-class/traffic-class-table/number-of-traffic-classes
    int8_t priorityToTcMap[8];          // traffic-class/traffic-class-table/priority0
    int8_t tcToLqMap[8];            // traffic-class/tc-data|tc:0|/lqueue
    uint8_t pQueueNum;              // traffic-class/number-of-pqueues
    int8_t pqToLqMap[8];           // traffic-class/pqueue-map|pqueue:0|/lqueue
    uint32_t tcMaxFrameSize[8];      // traffic-class/tc-data|tc:1|/max-frame-size
    bool cbsEnable;              // traffic-class/cbs-enabled
    int64_t tcToAdminIdleSlopeMap[8];    // traffic-class/tc-data|tc:2|/admin-idleslope
} EnetApp_TrafficClassCfg_t;
/* ========================================================================== */
/*                            Local Variables                                */
/* ========================================================================== */
extern EnetApp_Ctx_t gAppCtx;
extern int uc_dbal_setproc(uc_dbald *dbald, const char *name, int64_t pvalue);

static uint8_t gAvtpdStackBuf[TSN_TSK_STACK_SIZE] __attribute__ ((aligned(TSN_TSK_STACK_ALIGN)));
static uint8_t gMrpdStackBuf[TSN_TSK_STACK_SIZE] __attribute__ ((aligned(TSN_TSK_STACK_ALIGN)));
static uint8_t gMrpCliStackBuf[TSN_TSK_STACK_SIZE] __attribute__ ((aligned(TSN_TSK_STACK_ALIGN)));

EnetApp_TrafficClassCfg_t gTcCfg =
{
    .tcNum = 8,
    // Priority-TC-Map      0         1          2         3         4      5       6       7 <- priority
    // For ex: priority 2 (class B) is mapping with TC 1
    .priorityToTcMap = {    0,        3,       1,        2,        4,     5,      6,      7}, // <- tc
    .tcToLqMap =       {    0,        1,       2,        3,        4,     5,      6,      7}, // <- Logical queue
    .pQueueNum = 8,
    .pqToLqMap =       {    0,        1,       2,        3,        4,     5,      6,      7}, // <- Logical Queue
    .tcMaxFrameSize =  {   1500,      1500,    1500,    1500,     1500,  1500,   1500,   1500},
    .cbsEnable = true,
    .tcToAdminIdleSlopeMap = {1024,   1024,    1024,    1024,     1024,  1024,   1024,  4096000} // last item is for ptp
};
// This example, is to send data with priority 2 (class B) -> it should map with TC "1"
// This data should be map to LQ#1 <-> PQ#1
// And the Admin Idle Slope is 1, for initialization, we just need to set to small value.
// While MRP declare TA, it will calculate correct idleSlope, and register to HW.

UB_SD_GETMEM_DEF_EXTERN(YANGINIT_GEN_SMEM);
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
extern int AVTPD_MAIN(int argc, char *argv[]);
extern int XMRPD_MAIN(int argc, char *argv[]);
static void *EnetApp_avtpdTask(void *arg);
static void *EnetApp_xmrpdTask(void *arg);
static void *EnetApp_xmrpcTask(void *arg);

#define AVTPD_TASK_ENTRY \
    [ENETAPP_AVTPD_TASK_IDX]={ \
        .enable = false, \
        .stopFlag = true, \
        .taskPriority = AVTPD_TASK_PRIORITY, \
        .taskName = AVTPD_TASK_NAME, \
        .stackBuffer = gAvtpdStackBuf, \
        .stackSize = sizeof(gAvtpdStackBuf), \
        .onModuleDBInit = NULL, \
        .onModuleRunner = EnetApp_avtpdTask, \
        .appCtx = &gAppCtx \
    }

#define XMRPD_TASK_ENTRY \
	[ENETAPP_XMRPD_TASK_IDX]={ \
        .enable = true, \
        .stopFlag = true, \
        .taskPriority = XMRPD_TASK_PRIORITY, \
        .taskName = XMRPD_TASK_NAME, \
        .stackBuffer = gMrpdStackBuf, \
        .stackSize = sizeof(gMrpdStackBuf), \
        .onModuleDBInit = NULL, \
        .onModuleRunner = EnetApp_xmrpdTask, \
        .appCtx = &gAppCtx \
    }

#define XMRPC_TASK_ENTRY \
	[ENETAPP_XMRPC_TASK_IDX]={ \
        .enable = true, \
        .stopFlag = true, \
        .taskPriority = XMRPC_TASK_PRIORITY, \
        .taskName = XMRP_CLIENT_TASK_NAME, \
        .stackBuffer = gMrpCliStackBuf, \
        .stackSize = sizeof(gMrpCliStackBuf), \
        .onModuleDBInit = NULL, \
        .onModuleRunner = EnetApp_xmrpcTask, \
        .appCtx = &gAppCtx \
    }

/// @brief AVTPD task
/// @param arg 
/// @return 
static void *EnetApp_avtpdTask(void *arg)
{
    char *argv[]={"avtpd", "-n", NULL};
    int timeout_ms = 3000;
    int res;

    res = uniconf_ready(NULL, UC_CALLMODE_THREAD, timeout_ms);
    if (res)
    {
        DPRINT("The uniconf must be run first !");
    }
    else
    {
        AVTPD_MAIN(2, argv);
    }
    return NULL;
}

int EnetApp_addMrpconfModCtx(EnetApp_ModuleCtx_t *modCtxTbl)
{
    int i;

    EnetApp_ModuleCtx_t mrpModCtx[ENETAPP_MAX_TASK_IDX] =
    {
        AVTPD_TASK_ENTRY,
        XMRPD_TASK_ENTRY,
        XMRPC_TASK_ENTRY
    };

    for (i = 0; i < ENETAPP_MAX_TASK_IDX; i++)
    {
        if (mrpModCtx[i].enable == true)
        {
            memcpy(&modCtxTbl[i], &mrpModCtx[i], sizeof(EnetApp_ModuleCtx_t));
        }
    }
    return 0;
}

static int EnetCbsApp_registerAdminIdleSlope(uc_dbald *dbald, uc_notice_data_t *ucntd,
                                                char *ifname, int8_t tc,
                                                int64_t idleSlope)
{
    char sem_name[64];
    int err;
    uint32_t ksize;
    char key[UC_MAX_KEYSIZE];
    UC_NOTICE_SIG_T *sem = NULL;
    uint8_t aps[]={IETF_INTERFACES_RW, IETF_INTERFACES_INTERFACES,
                IETF_INTERFACES_INTERFACE, IETF_INTERFACES_BRIDGE_PORT,
                IETF_INTERFACES_TRAFFIC_CLASS, IETF_INTERFACES_TC_DATA,
                IETF_INTERFACES_ADMIN_IDLESLOPE,
                255u};
    /*
    * We want to register a semaphore for a notification
    * on completing of admin-idleslope seeting at HW side
    * before going to the next TC's idle slope to make sure
    * the idleSlope of highest priority queue must be configured
    * first before going for the lower priority queue.
    * This restriction is only required at the initial time.
    * In the run time configuration, since all priority queues
    * have been configured, setting idle slope of any queue will
    * work fine.
    */
    snprintf(sem_name, sizeof(sem_name), "/cbs_wait_sem_%d", tc);
    void *kvs[]={(void*)ifname, (void*)&tc, sem_name, NULL};
    uint8_t kss[]={strlen(ifname)+1, sizeof(tc), strlen(sem_name)+1};
    if(uc_nc_notice_register(ucntd, dbald, aps, kvs, kss, UC_NOTICE_DBVAL_ADD, &sem))
    {
        DPRINT("%s: uc_nc_notice_register failure. tc=%d\n", __func__, tc);
        return -1;
    }
    
    err=YDBI_SET_ITEM(ifk4vk1, ifname, 
                IETF_INTERFACES_TRAFFIC_CLASS,
                IETF_INTERFACES_TC_DATA,
                IETF_INTERFACES_ADMIN_IDLESLOPE,
                255,
                &tc, sizeof(tc), 
                YDBI_CONFIG, 
                (void*)&idleSlope,
                sizeof(idleSlope),
                YDBI_NO_NOTICE,
                YANG_DB_ONHW_NOACTION
                );
    DebugP_assert(err == 0);

    // Ask uniconf to write adminIdleSlop to HW
    kvs[2]=NULL;
    kss[2]=0;
    err=uc_nc_askaction_push(ucntd, dbald, aps, kvs, kss);
    if (err!=0)
    {
        DPRINT("uc_nc_askaction_push failed. err=%d\n", err);
    } 
    else 
    {
        DPRINT("ask uniconf to write adminIdleSlop succeeded \n", __func__);
    }
    // Now wait for uniconf to finish writing adminIdleSlop to HW
    if (sem)
    {
        /* Waiting for setting completed at the HW with the timeout */
        if (uc_notice_sig_check(BTRUE, sem, 200, __func__))
        {
            DPRINT("%s, Failed to get a notice from the uniconf",
                    __func__);
        } else
        {
            err = uc_nc_get_notice_act(ucntd, dbald,
                                        sem_name, key, &ksize);
            if (err)
            {
                DPRINT("There is no notice from the uniconf");
            }
            else
            {
                DPRINT("Registered adminIdleSlope finished. tc=%d", tc);
            }
        }
        /* Release the semaphore */
        err = uc_nc_notice_deregister_all(ucntd, dbald, sem_name);
        if (err != 0)
        {
            DPRINT("Failed to unregister sempahore");
        }
        sem = NULL;
    }
    return err;

}

/// Register idle initial idelSlope 
void EnetApp_registerIdleSlope(uc_dbald *dbald, uc_notice_data_t* ucntd, char* ndev)
{
    int err;
    for (int i=7; i>0; i--)
    {
        err=EnetCbsApp_registerAdminIdleSlope(dbald, ucntd,
                                                ndev, i,
                                                gTcCfg.tcToAdminIdleSlopeMap[i]);
        if (err==0)
        {
            DPRINT("%s, Register idleSlope for tc=%d", __func__, i);
        } else {
            DPRINT("%s, Failed to Register idleSlope for tc=%d, err=%d", __func__, i, err);
        }
    }

}

static int EnetCbsApp_registerCbsEnableToUniconf(uc_dbald *dbald, uc_notice_data_t *ucntd,
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

/// Set common configuration parameter
int EnetApp_setMrpExtControlConfig(uc_notice_data_t* ucntd, char* dev)
{
    // char buffer[MAX_KEY_SIZE];
    int err;
    int8_t i;
    uc_dbald *dbald;

    ydbi_mrp_set_external_control(0, dev, MRP_MVRP, 1);
    ydbi_mrp_set_external_control(0, dev, MRP_MSRP, 1);

    dbald = ydbi_access_handle()->dbald;

    err=YDBI_SET_ITEM(ifk4vk1, dev, 
                IETF_INTERFACES_TRAFFIC_CLASS,
                IETF_INTERFACES_TRAFFIC_CLASS_TABLE,
                IETF_INTERFACES_NUMBER_OF_TRAFFIC_CLASSES,
                255,
                NULL, 0, YDBI_CONFIG, (void*)&gTcCfg.tcNum, sizeof(gTcCfg.tcNum),
                YDBI_NO_NOTICE,
                YANG_DB_ONHW_NOACTION
                );
    DebugP_assert(err == 0);

    for (i=0; i<gTcCfg.tcNum; i++)
    {
        err=YDBI_SET_ITEM(ifk4vk1, dev, 
                IETF_INTERFACES_TRAFFIC_CLASS,
                IETF_INTERFACES_TRAFFIC_CLASS_TABLE,
                IETF_INTERFACES_PRIORITY0+i,
                255,
                NULL, 0, YDBI_CONFIG, (void*)&gTcCfg.priorityToTcMap[i],
                sizeof(gTcCfg.priorityToTcMap[i]),
                YDBI_NO_NOTICE,
                YANG_DB_ONHW_NOACTION
                );
        DebugP_assert(err == 0);

        err=YDBI_SET_ITEM(ifk4vk1, dev, 
                IETF_INTERFACES_TRAFFIC_CLASS,
                IETF_INTERFACES_TC_DATA,
                IETF_INTERFACES_LQUEUE,
                255,
                &i, sizeof(i), 
                YDBI_STATUS, 
                (void*)&gTcCfg.tcToLqMap[i],
                sizeof(gTcCfg.tcToLqMap[i]),
                YDBI_NO_NOTICE,
                YANG_DB_ONHW_NOACTION
                );
        DebugP_assert(err == 0);
    }

    err=YDBI_SET_ITEM(ifk4vk1, dev, 
            IETF_INTERFACES_TRAFFIC_CLASS,
            IETF_INTERFACES_NUMBER_OF_PQUEUES,
            255,
            255,
            NULL, 0, YDBI_STATUS, (void*)&gTcCfg.pQueueNum, sizeof(gTcCfg.pQueueNum),
            YDBI_NO_NOTICE,
            YANG_DB_ONHW_NOACTION
            );
    DebugP_assert(err == 0);

    for (i=0; i<gTcCfg.pQueueNum; i++)
    {
        err=YDBI_SET_ITEM(ifk4vk1, dev, 
            IETF_INTERFACES_TRAFFIC_CLASS,
            IETF_INTERFACES_PQUEUE_MAP,
            IETF_INTERFACES_LQUEUE,
            255,
            &i, sizeof(i), YDBI_STATUS,
            (void*)&gTcCfg.pqToLqMap[i], sizeof(gTcCfg.pqToLqMap[i]),
            YDBI_NO_NOTICE,
            YANG_DB_ONHW_NOACTION
            );
        DebugP_assert(err == 0);
    }

    err=ydbi_set_item_qbk1vk0(ydbi_access_handle(), 
                            "br0", 
                            0, 
                            IEEE802_DOT1Q_BRIDGE_BRIDGE_PORT, 
                            YDBI_STATUS,
                            dev, strlen(dev)+1,
                            YDBI_NO_NOTICE);
    DebugP_assert(err == 0);
    
    uint16_t dot1q_ports=1;
    err=ydbi_set_item_qbk1vk0(ydbi_access_handle(), 
                            "br0", 
                            0, 
                            IEEE802_DOT1Q_BRIDGE_PORTS, 
                            YDBI_STATUS,
                            (void*)&dot1q_ports, 2,
                            YDBI_NO_NOTICE);
    DebugP_assert(err == 0);

    for (i=0; i<8; i++)
    {
        err=YDBI_SET_ITEM(ifk4vk1, dev, 
                IETF_INTERFACES_TRAFFIC_CLASS,
                IETF_INTERFACES_TC_DATA,
                IETF_INTERFACES_MAX_FRAME_SIZE,
                255,
                &i, sizeof(i), 
                YDBI_CONFIG, 
                (void*)&gTcCfg.tcMaxFrameSize[i],
                sizeof(gTcCfg.tcMaxFrameSize[i]),
                YDBI_NO_NOTICE,
                YANG_DB_ONHW_NOACTION
                );
        DebugP_assert(err == 0);
    }
    
    err=EnetCbsApp_registerCbsEnableToUniconf(dbald, ucntd, dev);
    if (err != 0)
    {
        DPRINT("%s, Failed to trigger uniconf to write idleSlope",
               __func__);
    }
    DPRINT("%s", __func__);

    return 0;
}

/// Check if gptp is sync before starting mrp app
bool EnetApp_isGptpSync()
{
    bool syncFlag = BFALSE;
    void *val = NULL;
    uint8_t portState = 0;
    uint8_t gmState=0;//0: no sync, 1: sync, 2: sync stable
    bool asCapable;

    int portIdx=TESTING_PORT_INDEX+1; /* gPTP port index in the DB started from 1 */
    int gdi=ydbi_gptpinstdomain2dbinst_pt(ydbi_access_handle(), 0, 0);
    YDBI_GET_ITEM_INTSUBST(ptk3vk0, gmState, val, gdi,
			       IEEE1588_PTP_TT_CLOCK_STATE, IEEE1588_PTP_TT_GMSTATE, 255,
			       YDBI_STATUS);

    syncFlag = (gmState == 2 || gmState==1) ? BTRUE: BFALSE;
    if (!syncFlag) {return BFALSE;}

    syncFlag = BFALSE;
    YDBI_GET_ITEM_INTSUBST(ptk4vk1, portState, val, gdi,
                    IEEE1588_PTP_TT_PORTS, IEEE1588_PTP_TT_PORT,
                    IEEE1588_PTP_TT_PORT_DS, IEEE1588_PTP_TT_PORT_STATE,
                    &portIdx, sizeof(uint16_t), YDBI_STATUS);

    /* check ieee1588-ptp-tt.yang for description of portState */
    if (portState != 6 && portState != 9) {DPRINT("Current port-state: %d ", portState);}

    asCapable=ydbi_get_asCapable(ydbi_access_handle(), 0, 0, portIdx);

    if ((portState == 6 || portState == 9) && asCapable) {syncFlag = BTRUE;}
    else if (portState == 9 && !asCapable) {syncFlag = BTRUE;}

    return syncFlag;
}

static void *EnetApp_xmrpdTask(void *arg)
{
    EnetApp_ModuleCtx_t *modCtx = (EnetApp_ModuleCtx_t *)arg;
    // EnetApp_Ctx_t *appCtx = modCtx->appCtx;
    int res = -1;
    int timeout_ms = 3000;
    mrpmand_t *mrpmand;

    res = uniconf_ready(NULL, UC_CALLMODE_THREAD, timeout_ms);
    if (res)
    {
        DPRINT("The uniconf must be run first !");
    }
    else
    {
        mrpmand=mrpman_init(0, modCtx->appCtx->dbName, NULL, true);
        res=mrpman_open(mrpmand);
        if (res==0)
        {
            int64_t tid=(int64_t)&modCtx->hTaskHandle;
            uc_dbal_setproc(ydbi_access_handle()->dbald, "xmrpd", tid);

            uint8_t ready=1;
            res=YDBI_SET_ITEM(nymrk1vk0, 0, XL4_EXTMOD_XL4MRP_XMRPD_READY,
			                            YDBI_STATUS, &ready, 1, YDBI_NO_NOTICE);
            if(res!=0)
            {
                DPRINT("failed to set XMRPD_READY");
            }
            else
            {
                DPRINT("xmrpd ready");
                res=mrpman_eloop(mrpmand, &modCtx->stopFlag); // blocking
            }
        }
        else
        {
            DPRINT("[%s] Cannot init mrpman. Exiting!!!", __func__);
        }

        mrpman_close(mrpmand);
    }
    return NULL;
}

static uint8_t to_priority_rank_reserved(uint8_t pcp, uint8_t rank)
{
    uint8_t priority_rank_reserved = 0;
	priority_rank_reserved|=(pcp<<5);
	
    priority_rank_reserved&=0xef;
	priority_rank_reserved|=(rank<<4)&0x10;

    // debug
    DPRINT("%s: Input pcp/rank[%d/%d] Extracted pcp/rank[%d/%d]", __func__, pcp, rank, (priority_rank_reserved>>5), (priority_rank_reserved>>4)&1);

    return priority_rank_reserved;
}

#if XMRPD_LISTENER_ENABLE==1
mode_type mrp_modes[MAX_AVB_APP]={LISTENER_MODE, LISTENER_MODE, LISTENER_MODE, LISTENER_MODE, LISTENER_MODE, LISTENER_MODE, LISTENER_MODE};
#else
mode_type mrp_modes[MAX_AVB_APP]={TALKER_MODE, TALKER_MODE, TALKER_MODE, TALKER_MODE, TALKER_MODE, TALKER_MODE, TALKER_MODE};
#endif
ub_bytearray8_t base_streamid={0x01, 0x02, 0x03, 0x04, 0x05,0x06, 0x00, 0x00};
ub_bytearray6_t dst_mac={0x91, 0xE0, 0xF0, 0x00, 0xFE,0x00};

static void EnetApp_initMrpCfg(xmrpd_app_data_t* xmrpd_app_info, uint8_t appno)
{
    mrp_data_t *mrp_data=&xmrpd_app_info->mrp_data[appno];
    
    mrp_data->vid=BASE_VID;
    mrp_data->mbps=1.0;
    mrp_data->mode=mrp_modes[appno];
    mrp_data->rec_tagged=false;

    mrp_data->domain_info.srclass_id = MSRP_SRClassID_B;
    mrp_data->domain_info.srclass_priority = MSRP_Priority_ClassB;
    mrp_data->domain_info.srclass_vid = BASE_VID;

    memcpy(mrp_data->stream_info.stream_id, base_streamid, sizeof(mrp_data->stream_info.stream_id));
    mrp_data->stream_info.stream_id[7] = appno;

    memset(mrp_data->sstream_id, 0, sizeof(mrp_data->sstream_id));
    bsid2ssid(mrp_data->stream_info.stream_id, mrp_data->sstream_id);

    memcpy(mrp_data->stream_info.destmac, dst_mac, sizeof(mrp_data->stream_info.destmac));
    mrp_data->stream_info.destmac[5] = appno;
    mrp_data->stream_info.vlan_id = BASE_VID;
    mrp_data->stream_info.max_frame_size = 1500;
    mrp_data->stream_info.max_intv_frames = 1;
    mrp_data->stream_info.priority_rank_reserved = to_priority_rank_reserved(MSRP_Priority_ClassB, MSRP_Rank_NonEmergency);
    mrp_data->stream_info.accum_latency = 100000;
    mrp_data->stream_info.failure_code = 0;

    mrp_data->stream_available = false;
}

static int EnetApp_registerUpDownNotice(EnetApp_Ctx_t *ctx, xmrpd_app_data_t* xmrpd_app_info)
{
    UC_NOTICE_SIG_T *sem=NULL;
    int res;

    (void)sprintf(xmrpd_app_info->semname, "%s%16"PRIx64, XMRPD_LINKSEMNAME, ub_rt_gettime64());
    for (int ndev = 0; ndev < ctx->netdevSize; ndev++)
    {
        if(sem==NULL){
            res=ydbi_set_ifupdown_ucnotice(ydbi_access_handle(), ctx->netdev[ndev], &sem, xmrpd_app_info->semname);
            if(!res && !xmrpd_app_info->linksem){xmrpd_app_info->linksem=sem;}
        }else{
            res=ydbi_set_ifupdown_ucnotice(ydbi_access_handle(), ctx->netdev[ndev], NULL, xmrpd_app_info->semname);
        }

        if(res!=0){
            DPRINT("%s:error in uc_nc_notice_register, %s\n", __func__, ctx->netdev[ndev]);
        }else{
            DPRINT("%s:uc_nc_notice_register, %s\n", __func__, ctx->netdev[ndev]);
        }
    }

    if(!xmrpd_app_info->linksem){
        DPRINT("%s:no semaphore is registered\n", __func__);
        return -1;
    }

    return 0;
}

static void *EnetApp_xmrpcTask(void *arg)
{
    EnetApp_ModuleCtx_t *modCtx = (EnetApp_ModuleCtx_t *)arg;
    EnetApp_Ctx_t *ctx = modCtx->appCtx;

    xmrpd_app_data_t xmrpd_app_info;
    xmrpd_app_info.dbname = modCtx->appCtx->dbName;
    xmrpd_app_info.avb_app_num = MRP_APP_NO;
    xmrpd_app_info.linksem = NULL;
    xmrpd_app_info.netdev = &ctx->netdev[0][0]; // tilld0

    if (EnetApp_registerUpDownNotice(ctx, &xmrpd_app_info) == -1)
    {
        return NULL;
    }

    // Init DB in here
    yang_db_item_access_t *ydbi = ydbi_access_handle();
    uc_notice_data_t* ucntd;
    ucntd = uc_notice_init(UC_CALLMODE_THREAD, xmrpd_app_info.dbname);
    //

    xmrpd_app_info.dbald = ydbi->dbald;
    xmrpd_app_info.ucntd = ucntd;

    for (uint8_t i=0; i<xmrpd_app_info.avb_app_num; i++)
    {
        EnetApp_initMrpCfg(&xmrpd_app_info, i);
        xmrpd_app_info.mrp_data[i].netdev=&ctx->netdev[TESTING_PORT_INDEX][0];
    }

    int64_t tid=(int64_t)&modCtx->hTaskHandle;
    uc_dbal_setproc(ydbi_access_handle()->dbald, "xmrpd", tid);

    /* Doesn't return */
	run_xmrpd_app(&xmrpd_app_info);

    return NULL;
}