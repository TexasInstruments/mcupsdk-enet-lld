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
 * \file   crf_app.c
 *
 * \brief This file contains CRF Task and its helper functions
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "avtpc_crf.h"
#include "tsn_gptp/gptpmasterclock.h"
#include <tsn_unibase/unibase_binding.h>
#include "enet_media_clock_ctrl/media_clock_ctrl.h"
#include "kernel/dpl/TimerP.h"
#include "ti_dpl_config.h"
#include <kernel/dpl/QueueP.h>
#include <math.h>
#include "crf_hw_config.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define DEFAULT_MCAST_ADDR   "91:E0:F0:00:FE:00"
#define NUM_CRF_EDGE         (16)
#define TS_PER_AVTPDU        (6)
/* ========================================================================== */
/*                              Struct & Enums                                */
/* ========================================================================== */
typedef struct
{
    const avtpc_crf_config_t* crfConfig;
    avtpc_crf_data_t *avtpc_crf;
    uint64_t array_ts[TS_PER_AVTPDU];
    int currentTsIndex;
    int64_t measurementOffset;
    uint64_t readyCRFTimestamp;
    double mediaClockFrequency;
    double mediaClockPeriod;
    double timestampingFrequency;
    double timestampingPeriod;
    MCC_handle hMCC;
    bool isLocked;
    uint32_t lockSlipCounter;
    uint32_t courseCorrCounter;
    uint32_t fineCorrCounter;
    QueueP_Object CRF_freeQueue;
    QueueP_Object CRF_readyQueue;
    QueueP_Handle CRF_freeHandle;
    QueueP_Handle CRF_readyHandle;
}crfData_t;

typedef struct
{
    QueueP_Elem elem;
    uint64_t timestamp;
}tsQueueElem;
/* ========================================================================== */
/*                              Global Variables                              */
/* ========================================================================== */
static tsQueueElem gCrfEdgeMemory[NUM_CRF_EDGE];
static CB_SEM_T gCRF_Tick;
/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */

static int crfApp_deqCrfSample(crfData_t* crfdata, uint64_t *timestamp);
static int crfApp_enqCrfSample(crfData_t* crfdata, uint64_t timestamp);
static void crfApp_runCrfTalker(crfData_t* pCrfData);
static void crfApp_runCrfListener(crfData_t* pCrfData);
static int crfApp_rxCallback(uint8_t *payload, int payload_size,
                      avbtp_rcv_cb_info_t *cbinfo, void *cbdata);
static inline uint64_t crfApp_absDiff(int64_t val1, int64_t val2);
void crfApp_startCrfTask(bool isListener);
/* ========================================================================== */
/*                           Function Definitions                             */
/* ========================================================================== */

int crf_task(int argc, char *argv[])
{
#ifdef AVTP_CRF_TALKER_ENABLED
    crfApp_startCrfTask(false);
#elif AVTP_CRF_LISTENER_ENABLED
    crfApp_startCrfTask(true);
#else
    #error "Error either Talker or Listener has to be enabled"
#endif
    return 0;
}

void crfApp_startCrfTask(bool isListener)
{
    int32_t status = SystemP_SUCCESS;

    avtpc_crf_config_t crfcfg = {
        .listener                = isListener,
        .netdev                  = "tilld0",
        .dest_mcast              = DEFAULT_MCAST_ADDR,
        .join_mcast              = DEFAULT_MCAST_ADDR,
        .vlan_id                 = 110,
        .vlan_pcp                = IEEE1722_DEFAULT_PCP,
        .stream_id               = "00:01:02:03:04:05:00:04",
        .type                    = AVBTP_CRF_TYPE_AUDIO_SAMPLE,
        .mr_pdus_toggle          = AVTPC_MR_PDUS_TOGGLE_MIN,
        .base_frequency          = 48000,
        .pull                    = AVBTP_CRF_PULL_MULT_1,
        .timestamp_interval      = 160,
    };

    client_connect_request_t ccr;

    crfData_t crfData = {
        .crfConfig = &crfcfg,
    };

    while((status = gptpmasterclock_init(NULL)))
    {
        UB_LOG(UBL_INFO,"Waiting for tsn_gptpd to be ready...\n");
        CB_USLEEP(100000);
    }

    if (status == 0)
    {
        crfData.avtpc_crf = avtpc_crf_init(NULL);
        status = (crfData.avtpc_crf == NULL) ? SystemP_FAILURE : SystemP_SUCCESS;
    }

    if (status == SystemP_SUCCESS)
    {
        if (crfcfg.listener)
        {
            avtpc_crf_set_rcv_cb(crfData.avtpc_crf, crfApp_rxCallback, (void*)&crfData);
            if (avtpc_crf_set_rxdirect(crfData.avtpc_crf) != 0)
            {
                status = SystemP_FAILURE;
            }
        }
        else
        {
            if (avtpc_crf_set_txdirect(crfData.avtpc_crf) != 0)
            {
                status = SystemP_FAILURE;
            }
        }
    }

    if (status == SystemP_SUCCESS)
    {
        status = avtpc_crf_set_ccr(crfData.avtpc_crf, &crfcfg, &ccr);
    }

    if (status == 0)
    {
        ccr.echoback = false;
        if(avtpc_crf_connection_request(crfData.avtpc_crf, &ccr))
        {
            status = SystemP_FAILURE;
        }
        else
        {
            status = SystemP_SUCCESS;
        }
    }

    if (status == SystemP_SUCCESS)
    {
        crfData.CRF_freeHandle  = QueueP_create(&crfData.CRF_freeQueue);
        crfData.CRF_readyHandle = QueueP_create(&crfData.CRF_readyQueue);

        for (int i = 0; i < NUM_CRF_EDGE; i++)
        {
            (void)QueueP_put(crfData.CRF_freeHandle, &gCrfEdgeMemory[i]);
        }
    }

    if (status == SystemP_SUCCESS)
    {
        crfData.mediaClockFrequency = avbtp_crf_nominal_frequency(crfData.crfConfig->pull, \
                                                                  crfData.crfConfig->base_frequency);
        crfData.mediaClockPeriod = 1e9/crfData.mediaClockFrequency;

        crfData.timestampingFrequency = crfData.mediaClockFrequency/crfData.crfConfig->timestamp_interval;
        crfData.timestampingPeriod = 1e9/crfData.timestampingFrequency;
    }

    crfData.hMCC = MCC_init(crfData.mediaClockFrequency,crfData.timestampingFrequency);

    CB_SLEEP(5);

    if (status == SystemP_SUCCESS)
    {
        status = crfHwConfig_setup();
    }

    CB_SLEEP(12);

    if (status == SystemP_SUCCESS)
    {
        crfData.measurementOffset = crfHwConfig_estimateEdgeDiff(crfData.mediaClockFrequency,
                                                                 crfData.timestampingFrequency);
    }

    if (status == SystemP_SUCCESS)
    {
        status = crfHwConfig_routeTsSignalToPhy();
    }

    MCC_enableEventCapture(crfData.hMCC, 0x01);

    if (status == SystemP_SUCCESS)
    {
        CB_SEM_INIT(&gCRF_Tick, 0, 0);

        /* Start the Timer. */
        TimerP_start(gTimerBaseAddr[CRFTICK_TIMER]);


        if (crfcfg.listener)
        {
            crfApp_runCrfListener(&crfData);
        }
        else
        {
            crfApp_runCrfTalker(&crfData);
        }
    }
    return;
}

static int crfApp_rxCallback(uint8_t *payload, int payload_size,
                      avbtp_rcv_cb_info_t *cbinfo, void *cbdata)
{
    crfData_t *pCrfData = (crfData_t*)cbdata;
    avbtp_crf_info_t *rcrfinfo = &cbinfo->u.rcrfinfo;
    int32_t status = 0;
    uint64_t crfTsArray[TS_PER_AVTPDU];
    int num_ts = 0;

    if(rcrfinfo->subtype != AVBTP_SUBTYPE_CRF){
        UB_LOG(UBL_INFO,"%s:Unsupported subtype: %d\n",__func__, rcrfinfo->subtype);
        status = -1;
    }

    if (status == 0)
    {
        num_ts = avbtp_deserialize_stdtype_crf_data(crfTsArray, TS_PER_AVTPDU, \
                                                                payload, payload_size);

        if (num_ts < 0)
        {
            UB_LOG(UBL_ERROR, "%s: deserialization CRF failed.\n", __func__);
            status = -1;
        }
        else if (num_ts != TS_PER_AVTPDU)
        {
            UB_LOG(UBL_ERROR, "%s: Received Less CRF Timestamps\n", __func__);
            status = -1;
        }
        else
        {}
    }

    if (status == 0)
    {
        for (int i = 0; i < TS_PER_AVTPDU; i++)
        {
            crfApp_enqCrfSample(pCrfData, crfTsArray[i]);
        }
    }

    return status;
}

/* Callback from the Timer.
 * Must be programmed to match MediaClockFreq/TimestampingInterval */
void crfApp_tick(void *args)
{
    CB_SEM_POST(&gCRF_Tick);
}

static uint64_t crfApp_getWorthyCrfTs(crfData_t* pCrfData, uint64_t currentMediaClockTs)
{
    uint64_t matchCrfTimestamp = 0;

    if (crfApp_absDiff(pCrfData->readyCRFTimestamp, currentMediaClockTs) \
                                                < pCrfData->timestampingPeriod)
    {
        matchCrfTimestamp = pCrfData->readyCRFTimestamp;
    }

    return matchCrfTimestamp;
}

static void crfApp_latchNextCrfTS(crfData_t* pCrfData, uint64_t currentMediaClockTs)
{
    /* Store Next Ready Crf Timestamp*/
    for (int i = 0; i < NUM_CRF_EDGE; i++)
    {
        if (pCrfData->readyCRFTimestamp > currentMediaClockTs)
        {
            /* Found Future CRF Timestamp, break from here. */
            break;
        }
        else
        {
            if (crfApp_deqCrfSample(pCrfData, &pCrfData->readyCRFTimestamp) == SystemP_FAILURE)
            {
                pCrfData->readyCRFTimestamp = 0;
                break;
            }
        }
    }
}

static void crfApp_mediaClock_adjPhase(crfData_t *pCrfdata, int64_t phaseDelta)
{
    double mediaClockPeriod = pCrfdata->mediaClockPeriod;

    if (phaseDelta > (int64_t)(mediaClockPeriod/2))
    {
        phaseDelta -= mediaClockPeriod;
    }

    /*
     * Select course or Fine Correction.
     *
     *      Course Correction - 5 Cycles/Adjustment
     * 7%  ---------------
     *      Fine Correction - 1 Cycle/Adjustment
     * 1%   ---------------
     *      No Correction
     * 0%   ---------------
     *      No Correction
     * -1%  ---------------
     *      Fine Correction - 1 Cycle/Adjustment
     * -7% ---------------
     *      Course Correction - 5 Cycles/Adjustment
    */

    double percent = ((double)phaseDelta*100/(double)mediaClockPeriod);

    if (crfApp_absDiff(0, percent) < 1)
    {
        /* No Correction Needed. */
        pCrfdata->isLocked = true;
    }
    else if (crfApp_absDiff(0, percent) < 5)
    {
        /* Fine Correction. */
        MCC_NudgeClock(pCrfdata->hMCC, (percent>0)?-1:1);
        pCrfdata->fineCorrCounter += 1;
    }
    else
    {
        /* Course Correction. */
        MCC_NudgeClock(pCrfdata->hMCC, (percent>0)?-5:5);
        pCrfdata->courseCorrCounter += 1;
    }

    if (crfApp_absDiff(0, percent) > 5)
    {
        if (pCrfdata->isLocked == true)
        {
            pCrfdata->lockSlipCounter += 1;
        }
        pCrfdata->isLocked = false;
    }
}

static int crfApp_enqCrfSample(crfData_t* crfdata, uint64_t timestamp)
{
    if (QueueP_isEmpty(crfdata->CRF_freeHandle) == true)
    {
        return SystemP_FAILURE;
    }
    /* Dequeue a buffer from free queue. */
    tsQueueElem* ptr = (tsQueueElem*)QueueP_get(crfdata->CRF_freeHandle);

    if (ptr == NULL)
    {
        return SystemP_FAILURE;
    }

    ptr->timestamp = timestamp;

    return QueueP_put(crfdata->CRF_readyHandle, ptr);
}

static int crfApp_deqCrfSample(crfData_t* crfdata, uint64_t *timestamp)
{
    if (QueueP_isEmpty(crfdata->CRF_readyHandle) == true)
    {
        return SystemP_FAILURE;
    }

    tsQueueElem* ptr = (tsQueueElem*)QueueP_get(crfdata->CRF_readyHandle);

    if (ptr == NULL)
    {
        return SystemP_FAILURE;
    }

    *timestamp = ptr->timestamp;

    return QueueP_put(crfdata->CRF_freeHandle, ptr);
}

static inline uint64_t crfApp_absDiff(int64_t val1, int64_t val2)
{
    if (val1 > val2) {
        return val1 - val2;
    } else {
        return val2 - val1;
    }
}

static void crfApp_printStats(crfData_t *pCrfData)
{

    static uint64_t previousPrintTime;

    uint64_t currentTime = ClockP_getTimeUsec();

    if (currentTime - previousPrintTime > UB_SEC_US)
    {
        UB_LOG(UBL_INFO, "Lock = %d, SlipCounter = %u, \
            Course Correction = %u, Fine Correction = %u\n", (int)pCrfData->isLocked,
               pCrfData->lockSlipCounter,
               pCrfData->courseCorrCounter,
               pCrfData->fineCorrCounter);
        previousPrintTime = currentTime;
    }
}

static bool crfApp_local2CrfTs(const crfData_t* pCrfData, uint64_t *timestamp)
{
    bool retval = false;
    double mediaClockPeriod = pCrfData->mediaClockPeriod;
    double timestampingPeriod = pCrfData->timestampingPeriod;
    uint64_t maxTransitTime = 2*UB_MSEC_NS;
    uint64_t prevTimestamp = (pCrfData->currentTsIndex == 0) ?
                                  0:
                                  pCrfData->array_ts[pCrfData->currentTsIndex-1];
    uint64_t totalAccumTime = (TS_PER_AVTPDU - 1)*timestampingPeriod;

    *timestamp += ceil(maxTransitTime/mediaClockPeriod)*mediaClockPeriod + totalAccumTime;

    if (prevTimestamp == 0 ||
       (crfApp_absDiff(prevTimestamp+timestampingPeriod, *timestamp) < 100))
    {
        /* Modify the timestmap*/
        retval = true;
    }
    return retval;
}

static int32_t crfApp_sendCrfFrame(crfData_t *pCrfData)
{
    int res = 0;
    uint8_t ts_buffer[TS_PER_AVTPDU * 8];

    res = avbtp_serialize_stdtype_crf_data(pCrfData->array_ts,
                        TS_PER_AVTPDU, ts_buffer, sizeof(ts_buffer));

    if (res > 0)
    {
        res = avtpc_crf_write_packet(pCrfData->avtpc_crf,
                                        NULL, ts_buffer, res);
    }

    return res;
}

static void crfApp_runCrfListener(crfData_t* pCrfData)
{
    while (1)
    {
        uint64_t mediaClockEdgeTs = 0;
        int32_t status = SystemP_SUCCESS;
        CB_SEM_WAIT(&gCRF_Tick);

        crfApp_printStats(pCrfData);

        status = MCC_getTimestamp(pCrfData->hMCC, 0x01, &mediaClockEdgeTs);
        if (status == SystemP_SUCCESS)
        {
            /* Correct the Timestamp offset
               to get approximate Media Clock Edge.*/
            mediaClockEdgeTs += pCrfData->measurementOffset;

            /* ready Timestamp should be a worthy to compare
            To be worthy. Difference shall not exceed one Period of TimestampInterval*/
            uint64_t crfTs = crfApp_getWorthyCrfTs(pCrfData, mediaClockEdgeTs);

            if (crfTs != 0)
            {
                /* Found a worthy Crf Timestamp, take the Synchronization for a spin. */
                int64_t timeDifference = mediaClockEdgeTs - crfTs;
                double cycles = timeDifference/pCrfData->mediaClockPeriod;

                int64_t phaseDelta_ns = (cycles-floor(cycles))*pCrfData->mediaClockPeriod;

                crfApp_mediaClock_adjPhase(pCrfData, phaseDelta_ns);
            }

            /* Latch the future Timestamp to compare. */
            crfApp_latchNextCrfTS(pCrfData, mediaClockEdgeTs);
        }
    }
}

static void crfApp_runCrfTalker(crfData_t* pCrfData)
{
    while (1)
    {
        uint64_t mediaClockEdgeTs = 0;
        int32_t status = SystemP_SUCCESS;
        CB_SEM_WAIT(&gCRF_Tick);

        status = MCC_getTimestamp(pCrfData->hMCC, 0x01, &mediaClockEdgeTs);

        if (status == SystemP_SUCCESS)
        {
            uint64_t modifiedMediaClockTimestamp = mediaClockEdgeTs + pCrfData->measurementOffset;

            if (crfApp_local2CrfTs(pCrfData, &modifiedMediaClockTimestamp) == true)
            {
                pCrfData->array_ts[pCrfData->currentTsIndex] = modifiedMediaClockTimestamp;
                pCrfData->currentTsIndex++;
            }
            else
            {
                /* Discard the whole array, Some missing Timestamp */
                UB_LOG(UBL_ERROR, "%s: Non Consecutive Media Clock Timestamp\n", __func__);
                pCrfData->currentTsIndex = 0;
            }

            if (pCrfData->currentTsIndex == TS_PER_AVTPDU)
            {
                /* Send CRF Frame. */
                if (crfApp_sendCrfFrame(pCrfData) < 0)
                {
                    UB_LOG(UBL_INFO, "Sending CRF Frame Failed\n");
                }

                memset(pCrfData->array_ts, 0, sizeof(pCrfData->array_ts));
                pCrfData->currentTsIndex = 0;
            }
        }
    }
}


