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
#include <stdint.h>
#include "tsn_gptp/gptpmasterclock.h"
#include <kernel/dpl/QueueP.h>
#include <math.h>
#include "crf_app.h"
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
    double currentMediaClockperiod;
    uint64_t prevLatLocalEdge;
    uint64_t prevLocalEdge;
    uint64_t edgeCounter;
    uint64_t tsCounter;
}calcLocalMCP_t;

typedef struct
{
    bool isLocked;
    uint32_t lockSlipCounter;
    uint32_t courseCorrCounter;
    uint32_t fineCorrCounter;
    bool freqSyncStatus;
    bool freqStableStatus;
    uint64_t freqSyncStatusCounter;
}syncMetrics_t;

typedef struct
{
    double measCrfFreq;
    uint64_t readyCRFTs;
    syncMetrics_t syMet;
    int32_t currPhaseErrNs;
    double currFreqErrPPM;
    calcLocalMCP_t clmcpState;
}listenerData_t;

typedef struct
{
    uint64_t array_ts[TS_PER_AVTPDU];
    uint32_t currTsIndex;
    uint32_t measuredTssPeriod;
    uint64_t previousMediaClockEdge;
}talkerData_t;

typedef struct
{
    crfHwCfg_info* hwInfo;
    avtpc_crf_config_t crfConfig;
    avtpc_crf_data_t *avtpc_crf;

    uint32_t correctionFlag;

    double nMediaClockFreq;
    double nMediaClockPeriod;
    double nTsFreq;
    double nTsPeriod;

    /* Todo: Cleanup queue handling. */
    QueueP_Object CRF_freeQueue;
    QueueP_Object CRF_readyQueue;
    QueueP_Handle CRF_freeHandle;
    QueueP_Handle CRF_readyHandle;

    talkerData_t td;
    listenerData_t ld;
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

crfData_t gCrfData;

/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */

static int crfApp_deqCrfSample(crfData_t* crfdata, uint64_t *timestamp);
static int crfApp_enqCrfSample(crfData_t* crfdata, uint64_t timestamp);
static inline uint64_t crfApp_absDiff(int64_t val1, int64_t val2);
static void crfApp_spinListener(crfData_t* pCrfData, uint64_t mediaClockEdgeTs);
static void crfApp_spinTalker(crfData_t* pCrfData, uint64_t mediaClockEdgeTs);

/* ========================================================================== */
/*                           Function Definitions                             */
/* ========================================================================== */

int32_t crfApp_init(crfApp_crfConfig* config)
{
    int32_t status = SystemP_SUCCESS;

    avtpc_crf_config_t crfcfg = {
        .listener                = config->isListener,
        .netdev                  = "tilld0",
        .dest_mcast              = DEFAULT_MCAST_ADDR,
        .join_mcast              = DEFAULT_MCAST_ADDR,
        .vlan_id                 = config->vlanID,
        .vlan_pcp                = config->vlanPCP,
        .type                    = AVBTP_CRF_TYPE_AUDIO_SAMPLE,
        .mr_pdus_toggle          = AVTPC_MR_PDUS_TOGGLE_MIN,
        .base_frequency          = config->baseFrequency,
        .pull                    = AVBTP_CRF_PULL_MULT_1,
        .timestamp_interval      = config->timestampingInterval,
    };

    char mstring[30];
    ub_streamid_to_string(config->streamID, mstring);
    crfcfg.stream_id = mstring;

    memcpy(&gCrfData.crfConfig, &crfcfg, sizeof(avtpc_crf_config_t));

    while((status = gptpmasterclock_init(NULL)))
    {
        UB_LOG(UBL_INFO,"Waiting for tsn_gptpd to be ready...\n");
        CB_USLEEP(100000);
    }

    if (status == 0)
    {
        gCrfData.avtpc_crf = avtpc_crf_init(NULL);
        status = (gCrfData.avtpc_crf == NULL) ? SystemP_FAILURE : SystemP_SUCCESS;
    }

    if (status == SystemP_SUCCESS)
    {
        gCrfData.hwInfo = config->hwInfo;
        gCrfData.correctionFlag = config->correctionFlag;
        if (gCrfData.crfConfig.listener)
        {
            if (avtpc_crf_set_rxdirect(gCrfData.avtpc_crf) != 0)
            {
                status = SystemP_FAILURE;
            }
        }
        else
        {
            if (avtpc_crf_set_txdirect(gCrfData.avtpc_crf) != 0)
            {
                status = SystemP_FAILURE;
            }
        }
    }

    client_connect_request_t ccr;

    if (gCrfData.crfConfig.listener == false)
    {
        if (status == SystemP_SUCCESS)
        {
            status = avtpc_crf_set_ccr(gCrfData.avtpc_crf, &gCrfData.crfConfig, &ccr);
        }

        if (status == 0)
        {
            ccr.echoback = false;
            if(avtpc_crf_connection_request(gCrfData.avtpc_crf, &ccr))
            {
                status = SystemP_FAILURE;
            }
            else
            {
                status = SystemP_SUCCESS;
            }
        }
    }

    if (status == SystemP_SUCCESS)
    {
        gCrfData.CRF_freeHandle  = QueueP_create(&gCrfData.CRF_freeQueue);
        gCrfData.CRF_readyHandle = QueueP_create(&gCrfData.CRF_readyQueue);

        for (int i = 0; i < NUM_CRF_EDGE; i++)
        {
            (void)QueueP_put(gCrfData.CRF_freeHandle, &gCrfEdgeMemory[i]);
        }
    }

    if (status == SystemP_SUCCESS)
    {
        gCrfData.nMediaClockFreq = avbtp_crf_nominal_frequency(gCrfData.crfConfig.pull, \
                                                                  gCrfData.crfConfig.base_frequency);
        gCrfData.nMediaClockPeriod = 1e9/gCrfData.nMediaClockFreq;

        gCrfData.nTsFreq   = gCrfData.nMediaClockFreq/gCrfData.crfConfig.timestamp_interval;
        gCrfData.nTsPeriod = 1e9/gCrfData.nTsFreq;
    }

    return status;
}

bool crfApp_getFreqStableStatus(void)
{
    crfData_t* pCrfData = &gCrfData;
    return pCrfData->ld.syMet.freqStableStatus;
}

void crfApp_tick(uint64_t mediaClockEdge)
{
    crfData_t* pCrfData = &gCrfData;

    if (pCrfData->crfConfig.listener == true)
    {
        crfApp_spinListener(pCrfData, mediaClockEdge);
    }
    else
    {
        crfApp_spinTalker(pCrfData, mediaClockEdge);
    }
}

int crfApp_rxCallback(uint8_t *payload, int payload_size,
                      avbtp_rcv_cb_info_t *cbinfo, void *cbdata)
{
    crfData_t *pCrfData = (crfData_t*)&gCrfData;
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
        pCrfData->ld.measCrfFreq= (1e9/((crfTsArray[TS_PER_AVTPDU-1]-crfTsArray[0]))*\
                                         pCrfData->crfConfig.timestamp_interval*(TS_PER_AVTPDU-1));
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

static uint64_t crfApp_getWorthyCrfTs(crfData_t* pCrfData, uint64_t currentMediaClockTs)
{
    uint64_t matchCrfTimestamp = 0;

    if (crfApp_absDiff(pCrfData->ld.readyCRFTs, currentMediaClockTs) \
                                                < pCrfData->nTsPeriod)
    {
        matchCrfTimestamp = pCrfData->ld.readyCRFTs;
    }

    return matchCrfTimestamp;
}

static void crfApp_latchNextCrfTS(crfData_t* pCrfData, uint64_t currentMediaClockTs)
{
    /* Store Next Ready Crf Timestamp*/
    for (int i = 0; i < NUM_CRF_EDGE; i++)
    {
        if (pCrfData->ld.readyCRFTs > currentMediaClockTs)
        {
            /* Found Future CRF Timestamp, break from here. */
            break;
        }
        else
        {
            if (crfApp_deqCrfSample(pCrfData, &pCrfData->ld.readyCRFTs) == SystemP_FAILURE)
            {
                pCrfData->ld.readyCRFTs = 0;
                break;
            }
        }
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
    #if 0
    static uint64_t previousPrintLimitTime;
    uint64_t currentTime = ClockP_getTimeUsec();
    if (currentTime - previousPrintLimitTime > 500*UB_MSEC_US)
    {
        // UB_LOG(UBL_INFO, "Crf Sync = %d, Freq Err PPM = %.3lf\r\n", pCrfData->ld.syMet.freqStableStatus, pCrfData->ld.currFreqErrPPM);
        UB_LOG(UBL_INFO, "CRF Lock Status = %d\r\n", pCrfData->ld.syMet.isLocked);
        previousPrintLimitTime = currentTime;
    }
    #endif
}

static void crfApp_printLimiter(char* string)
{

    static uint64_t previousPrintLimitTime;

    uint64_t currentTime = ClockP_getTimeUsec();

    if (currentTime - previousPrintLimitTime > 100*UB_MSEC_US)
    {
        UB_LOG(UBL_INFO, "%s", string);
        previousPrintLimitTime = currentTime;
    }
}

static bool crfApp_local2CrfTs(const crfData_t* pCrfData, uint64_t *timestamp)
{
    bool retval = false;
    double mediaClockPeriod = pCrfData->crfConfig.timestamp_interval;
    double timestampingPeriod = pCrfData->td.measuredTssPeriod;
    uint64_t maxTransitTime = 2*UB_MSEC_NS;
    uint64_t prevTimestamp = (pCrfData->td.currTsIndex == 0) ?
                                  0:
                                  pCrfData->td.array_ts[pCrfData->td.currTsIndex-1];
    uint64_t totalAccumTime = (TS_PER_AVTPDU - 1)*timestampingPeriod;

    uint64_t timestampOffset = ceil(maxTransitTime/mediaClockPeriod)*mediaClockPeriod + totalAccumTime;
    *timestamp += timestampOffset;

    if (prevTimestamp == 0 ||
       (crfApp_absDiff(prevTimestamp+timestampingPeriod, *timestamp) < 0.1*timestampingPeriod))
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

    res = avbtp_serialize_stdtype_crf_data(pCrfData->td.array_ts,
                        TS_PER_AVTPDU, ts_buffer, sizeof(ts_buffer));

    if (res > 0)
    {
        res = avtpc_crf_write_packet(pCrfData->avtpc_crf,
                                        NULL, ts_buffer, res);
    }
    return res;
}

static double crfApp_getMediaClockPeriod(crfData_t* pCrfData, uint64_t mediaClockEdgeTs)
{
    calcLocalMCP_t *pClmcp = &pCrfData->ld.clmcpState;
    double mediaClockPeriod = -1;
    if (pClmcp->tsCounter == 0)
    {
        pClmcp->prevLatLocalEdge = mediaClockEdgeTs;
        pClmcp->prevLocalEdge    = mediaClockEdgeTs;
        pClmcp->edgeCounter = 0;

    }
    else
    {
        /* Check the Previous Timestamp and increment the edge Counter accordingly. */
        if (crfApp_absDiff(mediaClockEdgeTs-pClmcp->prevLocalEdge, \
            pCrfData->nTsPeriod) < 0.1*pCrfData->nTsPeriod)
        {
            /* Seems a consecutive Timestamp. Increment tsCounter and edge Counter. */
            pClmcp->edgeCounter += pCrfData->crfConfig.timestamp_interval;
        }
        else
        {
            /* Calculate appropriate Increment value. */
            int64_t diff = mediaClockEdgeTs-pClmcp->prevLocalEdge;
            int approxCycles = round(diff/(double)pCrfData->nTsPeriod);

            pClmcp->edgeCounter += approxCycles*pCrfData->crfConfig.timestamp_interval;
        }
        pClmcp->prevLocalEdge    = mediaClockEdgeTs;
    }
    /* Increment the media clock Edge Ts counter */
    pClmcp->tsCounter += 1;

    if (pClmcp->tsCounter == TS_PER_AVTPDU)
    {
        /* Received TS_PER_AVTPDU Timestamps, calculate period. */
        mediaClockPeriod = ((mediaClockEdgeTs - pClmcp->prevLatLocalEdge)/\
                                   (double)pClmcp->edgeCounter);
        pClmcp->currentMediaClockperiod = mediaClockPeriod;

        /* Latch New Media Clock Edge. */
        pClmcp->prevLatLocalEdge = mediaClockEdgeTs;
        pClmcp->prevLocalEdge    = mediaClockEdgeTs;
        pClmcp->edgeCounter = 0;
        pClmcp->tsCounter = 1;
    }

    return mediaClockPeriod;
}

static int32_t crfApp_CalculateError(crfData_t* pCrfData, uint64_t crfTs, uint64_t mediaClockEdgeTs)
{
    int32_t status = SystemP_FAILURE;
    /*
        Calculate Frequency Difference and Phase Difference.
        1. Use mediaClockEdgeTs and its history to calculate local Media Clock Frequency.
        2. Use pCrfData->measuredCrfFrequency to calculate Frequency Difference.
        3. Use Both crfTs and mediaClockEdgeTs to calculate Phase Difference and update in pCrfData.
    */
    double currentMediaClockPeriod = crfApp_getMediaClockPeriod(pCrfData, mediaClockEdgeTs);
    /* Check if the current Media Clock Period is comparble. */
    if ((currentMediaClockPeriod > 0) && (fabs(currentMediaClockPeriod-pCrfData->nMediaClockPeriod) < 0.1*pCrfData->nMediaClockPeriod))
    {
        /* Looks like the correct Media Clock Period. */
        double localMcFreq  = 1e9/currentMediaClockPeriod;
        double crfFreq      = pCrfData->ld.measCrfFreq;
        double currentFrequencyErrorPPM = (localMcFreq-crfFreq)*1e6/crfFreq;

        /* IIR filter: y[n] = alpha * x[n] + (1 - alpha) * y[n-1] */
        double alpha = 0.05;
        pCrfData->ld.currFreqErrPPM = alpha * currentFrequencyErrorPPM +
                                                 ((1.0 - alpha) * pCrfData->ld.currFreqErrPPM);
        /* Calculate Phase Difference.  */
        int64_t timeDifference = mediaClockEdgeTs - crfTs;
        double cycles = timeDifference/currentMediaClockPeriod;
        pCrfData->ld.currPhaseErrNs = (cycles-floor(cycles))*currentMediaClockPeriod;
        status = SystemP_SUCCESS;
        if ((currentFrequencyErrorPPM > 1000) || (currentFrequencyErrorPPM < -1000))
        {
            UB_LOG(UBL_INFO, "%.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf\r\n", crfFreq,currentFrequencyErrorPPM, pCrfData->ld.currFreqErrPPM,
                                                                        localMcFreq, currentMediaClockPeriod, pCrfData->nMediaClockPeriod);
        }
    }

    return status;
}

static int32_t crfApp_correctFrequency(crfData_t* pCrfData)
{
    double errorPPM = pCrfData->ld.currFreqErrPPM;
    int corr = 0;
    if ((errorPPM > -0.5) && (errorPPM < 0.5))
    {
        /* Do Nothing, Synced. Already. */
        pCrfData->ld.syMet.freqSyncStatus = true;
        corr = 0;
    }
    else if ((errorPPM > -1) && (errorPPM < 1))
    {
        /* Fine Correction. */
        int polarity = (errorPPM>0)?1:-1;
        crfHwConfig_fineTuneFreq(pCrfData->hwInfo, -polarity*0.05);
        corr = 1;
    }
    else if ((errorPPM > -50) && (errorPPM < 50))
    {
        pCrfData->ld.syMet.freqSyncStatus= false;
        /* Course Correction. */
        int polarity = (errorPPM>0)?1:-1;
        crfHwConfig_fineTuneFreq(pCrfData->hwInfo, -polarity*1);
        corr = 2;
    }
    else
    {
        /* Set the PPM Error.*/
        corr = 3;
    }

    if (pCrfData->ld.syMet.freqSyncStatus)
    {
        pCrfData->ld.syMet.freqSyncStatusCounter += 1;
    }
    else
    {
        pCrfData->ld.syMet.freqSyncStatusCounter = 0;
    }

    if (pCrfData->ld.syMet.freqSyncStatusCounter > 200)
    {
        pCrfData->ld.syMet.freqStableStatus = 1;
    }
    else
    {
        pCrfData->ld.syMet.freqStableStatus = 0;
    }
    (void)corr;

    // UB_LOG(UBL_INFO, "Freq  PPM, SyncStatus = %lf,%d\r\n", errorPPM, pCrfData->ld.syMet.freqStableStatus);

    return SystemP_SUCCESS;
}

static int32_t crfApp_correctPhase(crfData_t* pCrfData, bool isErrorValid)
{
    if (isErrorValid)
    {
        double mediaClockPeriod = pCrfData->nMediaClockPeriod;

        int64_t phaseDelta = pCrfData->ld.currPhaseErrNs;

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
            pCrfData->ld.syMet.isLocked = true;
        }
        else if (crfApp_absDiff(0, percent) < 5)
        {
            /* Fine Correction. */
            crfHwConfig_adjPhase(pCrfData->hwInfo, (percent>0)?-1:1);
            pCrfData->ld.syMet.fineCorrCounter += 1;
        }
        else
        {
            /* Course Correction. */
            crfHwConfig_adjPhase(pCrfData->hwInfo, (percent>0)?-5:5);
            pCrfData->ld.syMet.courseCorrCounter += 1;
        }

        if (crfApp_absDiff(0, percent) > 5)
        {
            if (pCrfData->ld.syMet.isLocked == true)
            {
                pCrfData->ld.syMet.lockSlipCounter += 1;
            }
            pCrfData->ld.syMet.isLocked = false;
        }
    }
    return SystemP_SUCCESS;
}

static void crfApp_spinListener(crfData_t* pCrfData, uint64_t mediaClockEdgeTs)
{
    crfApp_printStats(pCrfData);

    /* ready Timestamp should be a worthy to compare
    To be worthy. Difference shall not exceed one Period of TimestampInterval*/
    uint64_t crfTs = crfApp_getWorthyCrfTs(pCrfData, mediaClockEdgeTs);

    if (crfTs != 0)
    {
        int32_t status = crfApp_CalculateError(pCrfData, crfTs, mediaClockEdgeTs);

        if (status == SystemP_SUCCESS && (pCrfData->correctionFlag & CRFAPP_CORRECT_FREQUENCY))
        {
            crfApp_correctFrequency(pCrfData);
        }

        if (pCrfData->correctionFlag & CRFAPP_CORRECT_PHASE)
        {
            crfApp_correctPhase(pCrfData, status == SystemP_SUCCESS);
        }
    }

    /* Latch the future Timestamp to compare. */
    crfApp_latchNextCrfTS(pCrfData, mediaClockEdgeTs);
}

static void crfApp_spinTalker(crfData_t* pCrfData, uint64_t mediaClockEdgeTs)
{
    /* Calculate Frequency */
    uint64_t currentTSSPeriod = mediaClockEdgeTs - pCrfData->td.previousMediaClockEdge;
    if (crfApp_absDiff((int64_t)currentTSSPeriod, (int64_t)pCrfData->nTsPeriod) \
                                            < (0.1*pCrfData->nTsPeriod))
    {
        /* Valid candidate to calculate Frequency Difference. */
        pCrfData->td.measuredTssPeriod = currentTSSPeriod;
    }

    pCrfData->td.previousMediaClockEdge = mediaClockEdgeTs;

    if (crfApp_local2CrfTs(pCrfData, &mediaClockEdgeTs) == true)
    {
        pCrfData->td.array_ts[pCrfData->td.currTsIndex] = mediaClockEdgeTs;
        pCrfData->td.currTsIndex++;
    }
    else
    {
        /* Discard the whole array, Some missing Timestamp */
        // UB_LOG(UBL_ERROR, "%s: Non Consecutive Media Clock Timestamp\n", __func__);
        pCrfData->td.currTsIndex = 0;
    }

    if (pCrfData->td.currTsIndex == TS_PER_AVTPDU)
    {
        /* Send CRF Frame. */
        if (crfApp_sendCrfFrame(pCrfData) < 0)
        {
            UB_LOG(UBL_INFO, "Sending CRF Frame Failed\n");
        }
        else
        {
            /* Send the Timestamps to the local Listener. */
            for (int i = 0; i < 6; i++)
            {
                crfApp_enqCrfSample(pCrfData, pCrfData->td.array_ts[i]);
            }
        }

        memset(pCrfData->td.array_ts, 0, sizeof(pCrfData->td.array_ts));
        pCrfData->td.currTsIndex = 0;
    }
}
