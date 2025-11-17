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
#include <kernel/dpl/TimerP.h>
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/SemaphoreP.h>
#include "tsn_conl2/aaf_avtpc_listener.h"
#include "tsn_conl2/aaf_avtpc_talker.h"
#include "ti_dpl_config.h"
#include "../debug_log.h"
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

#define MAX_TX_STATS_SIZE (20000)
#define MAX_RX_STATS_SIZE (20000)

#define GPTP_WAIT_TIME_SEC (8)
const ub_streamid_t CONST_PART_SID = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x00, 0x00};

typedef struct
{
    avbtp_rcv_cb_t avtp_stream_cb;
    void *arg;
}rxStream_cb;

typedef struct
{
    char netdev[CB_MAX_NETDEVNAME];
    int priority;
    int vlanID;
    ub_streamid_t streamID;      /* Stream ID Ref: IEEE802.1Q. */
    ub_macaddr_t  DestmacAddr;   /* Destination mac address(Usually a multicast Mac Address)*/
    uint32_t timeInterval_us;    /* Time interval of the TX stream (Must be a multiple of 125us)*/

    /* PCM Parameters */
    aaf_avtpc_pcminfo_t pcminfo;
}avtp_pcm_conf;

/*
 *   Note: According to the IEEE1722 and IEEE802.1Q,
 *         the first 6 bytes of stream ID must have the source MAC Address.
 *         This rule is not being followed in this Demo to simplify the application
 *         implemenation and the performance is not affected because of this change.
*/
typedef struct
{
    uint64_t sentPackets;
    uint64_t errorPackets;
}tx_stream_stats;

typedef struct
{
    uint32_t prevSeqId;
    uint64_t receivedPackets;
    uint64_t missedPackets;
    int32_t minLatencyUs;
    int32_t maxLatencyUs;
    int32_t avgLatencyUs;
}rx_stream_stats;

typedef void* aafcrf_data;

/*
 * To add a stream:
 * 1. Add an entry in the enum txStreams/rxStreams like TX_CLASS_D1.
 * 2. Add an entry in the gTxStreamConf/gRxStreamConf with the index
 *    defined in the above step.
 * 3. Modify the reqired parameters in the gTxStreamConf/gRxStreamConf entry.
 * 4. Repeat the same steps in the counterpart application.
 * 5. Keep the TX and RX parameters in both source and sink application
 *    consistent.
*/
/* Semaphore that gets posted by the
   AVB Tick Timer Handler at 125us period*/
static SemaphoreP_Object gAvbTickSem;

#if AVB_AUTOAMP_SOURCE_DEMO
typedef enum
{
    TX_FIRST_STREAM,
    TX_CLASS_A1 = TX_FIRST_STREAM,
    TX_CLASS_D1,
    TX_CLASS_D2,
    TX_CLASS_D3,
    MAX_TX_STREAMS,
}txStreams;

typedef enum
{
    RX_FIRST_STREAM,
    RX_CLASS_A1 = RX_FIRST_STREAM,
    RX_CLASS_D1,
    MAX_RX_STREAMS,
}rxStreams;

/* TX Stream Configuration, Add or Remove the Stream configuration if required. */
static const avtp_pcm_conf gTxStreamConf[MAX_TX_STREAMS] = {
    [TX_CLASS_A1] = {
     .netdev          = "tilld0",
     .streamID        = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x00, 0x01},
     .DestmacAddr     = {0x91, 0xE0, 0xF0, 0x00, 0xFE, 0x01},
     .vlanID          = 110,
     .priority        = 3,
     .timeInterval_us = 125,
     .pcminfo = {.format = AVBTP_AAF_FORMAT_INT_16BIT,
                 .channels = 8, .bit_depth = 16, .srate  = 48000},
    },
    [TX_CLASS_D1] = {
     .netdev          = "tilld0",
     .streamID        = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x00, 0x02},
     .DestmacAddr     = {0x91, 0xE0, 0xF0, 0x00, 0xFE, 0x01},
     .vlanID          = 110,
     .priority        = 3,
     .timeInterval_us = 1000,
     .pcminfo = {.format = AVBTP_AAF_FORMAT_INT_16BIT,
                 .channels = 8, .bit_depth = 16, .srate  = 48000},
    },
    [TX_CLASS_D2] = {
     .netdev          = "tilld0",
     .streamID        = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x00, 0x03},
     .DestmacAddr     = {0x91, 0xE0, 0xF0, 0x00, 0xFE, 0x01},
     .vlanID          = 110,
     .priority        = 3,
     .timeInterval_us = 1000,
     .pcminfo = {.format = AVBTP_AAF_FORMAT_INT_16BIT,
                 .channels = 8, .bit_depth = 16, .srate  = 48000},
    },
    [TX_CLASS_D3] = {
     .netdev          = "tilld0",
     .streamID        = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x00, 0x04},
     .DestmacAddr     = {0x91, 0xE0, 0xF0, 0x00, 0xFE, 0x01},
     .vlanID          = 110,
     .priority        = 3,
     .timeInterval_us = 1000,
     .pcminfo = {.format = AVBTP_AAF_FORMAT_INT_16BIT,
                 .channels = 8, .bit_depth = 16, .srate  = 48000},
    },
};

/* RX Stream Configuration, Add or Remove the Stream configuration if required. */
static const avtp_pcm_conf gRxStreamConf[MAX_RX_STREAMS] = {
    [RX_CLASS_A1] = {
     .netdev          = "tilld0",
     .streamID        = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x00, 0x05},
     .DestmacAddr     = {0x91, 0xE0, 0xF0, 0x00, 0xFE, 0x01},
     .vlanID          = 110,
     .priority        = 3,
     .timeInterval_us = 125,
     .pcminfo = {.format = AVBTP_AAF_FORMAT_INT_16BIT,
                 .channels = 8, .bit_depth = 16, .srate  = 48000},
    },
    [RX_CLASS_D1] = {
     .netdev          = "tilld0",
     .streamID        = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x00, 0x06},
     .DestmacAddr     = {0x91, 0xE0, 0xF0, 0x00, 0xFE, 0x01},
     .vlanID          = 110,
     .priority        = 3,
     .timeInterval_us = 1000,
     .pcminfo = {.format = AVBTP_AAF_FORMAT_INT_16BIT,
                 .channels = 8, .bit_depth = 16, .srate  = 48000},
     },
};
#endif

#if AVB_AUTOAMP_SINK_DEMO

typedef enum
{
    TX_FIRST_STREAM,
    TX_CLASS_A1 = TX_FIRST_STREAM,
    TX_CLASS_D1,
    MAX_TX_STREAMS,
}txStreams;

typedef enum
{
    RX_FIRST_STREAM,
    RX_CLASS_A1 = RX_FIRST_STREAM,
    RX_CLASS_D1,
    RX_CLASS_D2,
    RX_CLASS_D3,
    MAX_RX_STREAMS,
}rxStreams;

/* TX Stream Configuration, Add or Remove the Stream configuration if required. */
static const avtp_pcm_conf gTxStreamConf[MAX_TX_STREAMS] = {
    [TX_CLASS_A1] = {
     .netdev          = "tilld0",
     .streamID        = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x00, 0x05},
     .DestmacAddr     = {0x91, 0xE0, 0xF0, 0x00, 0xFE, 0x01},
     .vlanID          = 110,
     .priority        = 3,
     .timeInterval_us = 125,
     .pcminfo = {.format = AVBTP_AAF_FORMAT_INT_16BIT,
                 .channels = 8, .bit_depth = 16, .srate  = 48000},
    },
    [TX_CLASS_D1] = {
     .netdev          = "tilld0",
     .streamID        = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x00, 0x06},
     .DestmacAddr     = {0x91, 0xE0, 0xF0, 0x00, 0xFE, 0x01},
     .vlanID          = 110,
     .priority        = 3,
     .timeInterval_us = 1000,
     .pcminfo = {.format = AVBTP_AAF_FORMAT_INT_16BIT,
                 .channels = 8, .bit_depth = 16, .srate  = 48000},
     },
};

/* RX Stream Configuration, Add or Remove the Stream configuration if required. */
static const avtp_pcm_conf gRxStreamConf[MAX_RX_STREAMS] = {
    [RX_CLASS_A1] = {
     .netdev          = "tilld0",
     .streamID        = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x00, 0x01},
     .DestmacAddr     = {0x91, 0xE0, 0xF0, 0x00, 0xFE, 0x01},
     .vlanID          = 110,
     .priority        = 3,
     .timeInterval_us = 125,
     .pcminfo = {.format = AVBTP_AAF_FORMAT_INT_16BIT,
                 .channels = 8, .bit_depth = 16, .srate  = 48000},
    },
    [RX_CLASS_D1] = {
     .netdev          = "tilld0",
     .streamID        = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x00, 0x02},
     .DestmacAddr     = {0x91, 0xE0, 0xF0, 0x00, 0xFE, 0x01},
     .vlanID          = 110,
     .priority        = 3,
     .timeInterval_us = 1000,
     .pcminfo = {.format = AVBTP_AAF_FORMAT_INT_16BIT,
                 .channels = 8, .bit_depth = 16, .srate  = 48000},
    },
    [RX_CLASS_D2] = {
     .netdev          = "tilld0",
     .streamID        = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x00, 0x03},
     .DestmacAddr     = {0x91, 0xE0, 0xF0, 0x00, 0xFE, 0x01},
     .vlanID          = 110,
     .priority        = 3,
     .timeInterval_us = 1000,
     .pcminfo = {.format = AVBTP_AAF_FORMAT_INT_16BIT,
                 .channels = 8, .bit_depth = 16, .srate  = 48000},
    },
    [RX_CLASS_D3] = {
     .netdev          = "tilld0",
     .streamID        = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x00, 0x04},
     .DestmacAddr     = {0x91, 0xE0, 0xF0, 0x00, 0xFE, 0x01},
     .vlanID          = 110,
     .priority        = 3,
     .timeInterval_us = 1000,
     .pcminfo = {.format = AVBTP_AAF_FORMAT_INT_16BIT,
                 .channels = 8, .bit_depth = 16, .srate  = 48000},
    },
};
#endif

/* Static global Variable to store Talker stats. */
static tx_stream_stats talkerStats[MAX_TX_STREAMS] = {};

/* Stores the callback functions indexed to streamID[7] */
static rxStream_cb gStreamCbArr[256] = {0};

/* Static global Variable to store Listener stats.
   Also contains Latency Stats*/
static rx_stream_stats listenerStats[MAX_RX_STREAMS];

static uint8_t gDummyPayload[1500];

static aafcrf_data gCrfData;
/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static int autoamp_rxStreamCallback(uint8_t *payload, int plsize,
                                avbtp_rcv_cb_info_t *cbinfo, void *cbdata);
static void inline initStats(rx_stream_stats *stats);
static void inline updateStats(const int64_t currentVal, rx_stream_stats *stats);
static void inline printStats(const rx_stream_stats *stats, char* printHeader);

int crfrx_callback(uint8_t *payload, int payload_size,
                                avbtp_rcv_cb_info_t *cbinfo, void *cbdata);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void set_crf_cbdata(aafcrf_data cbdata)
{
    gCrfData = cbdata;
}

void init_conl2_params(const avtp_pcm_conf* conf, conl2_basic_conparas_t *bcp)
{
    bcp->vid       = conf->vlanID;
    bcp->time_intv = conf->timeInterval_us;
    bcp->pcp       = conf->priority;

    memcpy(&bcp->mcastmac, conf->DestmacAddr, sizeof(ub_macaddr_t));
    memcpy(&bcp->streamid, conf->streamID,    sizeof(ub_streamid_t));
    strcpy(bcp->netdev, conf->netdev);

    /* Hardcoded parameters goes here. */
    bcp->max_frm_sz        = 1500;
    bcp->max_intv_frames   = 1;
    bcp->avtpd_bufftime_us = 10000;
    bcp->is_direct         = true;
    bcp->send_ahead_ts     = 20000;
}

void init_pcm_params(const avtp_pcm_conf* conf, aaf_avtpc_pcminfo_t* pcm_info)
{
    memcpy(pcm_info, &conf->pcminfo, sizeof(aaf_avtpc_pcminfo_t));
}

aaf_avtpc_talker_data_t *autoAmpDemo_initPcmTalker(const avtp_pcm_conf* conf)
{
    conl2_basic_conparas_t conl2Params;
    aaf_avtpc_pcminfo_t    pcmInfo;
    init_conl2_params(conf, &conl2Params);
    init_pcm_params(conf, &pcmInfo);
    aaf_avtpc_talker_data_t *talker = aaf_avtpc_talker_init(&conl2Params);
    if (talker != NULL)
    {
        aaf_avtpc_set_pcm_info(talker, &pcmInfo);
    }
    return talker;
}

void start_all_talkers(void)
{
    if (MAX_TX_STREAMS == 0)
    {
        /* No Talkers are configured. Skip this task  */
        return;
    }
    /* Initialize the AVB Tick Semaphore. */
    SemaphoreP_constructBinary(&gAvbTickSem, 0);

    aaf_avtpc_talker_data_t* talker[MAX_TX_STREAMS];
    uint32_t dataSize[MAX_TX_STREAMS];
    for (int i = TX_FIRST_STREAM; i < MAX_TX_STREAMS; i++)
    {
        talker[i] = autoAmpDemo_initPcmTalker(&gTxStreamConf[i]);

        const aaf_avtpc_pcminfo_t *pcmInfo = &gTxStreamConf[i].pcminfo;

        /*
         *  Data size of a TX Stream is computed from this relation
         *  Size = SampleSize*channels*sampleRate*DataInterval_in_seconds;
        */
        dataSize[i] = (pcmInfo->bit_depth/8)*pcmInfo->channels* \
                       (pcmInfo->srate)/(UB_SEC_US/gTxStreamConf[i].timeInterval_us);
    }

    /*
     *  Wait for GPTP to sync,
     * Typical 4 Sec to Linkup + < 1Sec to Sync within 100ns.
    */
    CB_SLEEP(GPTP_WAIT_TIME_SEC);

    DebugP_log("Starting Transmission of Tx packets ......\r\n");

    TimerP_start(AVB_TICK_TIMER_BASE_ADDR);

    uint64_t talkerRunTime_us = 0;

    bool stopTalker = false;

    while (!stopTalker)
    {
        SemaphoreP_pend(&gAvbTickSem, SystemP_WAIT_FOREVER);

#if MEASURE_AVB_LATENCY
        const uint64_t gptpTime = gptpmasterclock_getts64()/UB_USEC_NS;
#else
        const uint64_t gptpTime = 0;
#endif

        for (int i = TX_FIRST_STREAM; i < MAX_TX_STREAMS; i++)
        {
            if (talkerRunTime_us%gTxStreamConf[i].timeInterval_us == 0)
            {
                /* Fill the Presentation Time. */
                const uint64_t pts = gptpTime;

                /*
                 * Send the AVTP packet. Latency Measurement starts here....
                 * To get more accurate latency numbers, Toggle a GPIO here...
                */
                if (aaf_avtpc_talker_write(talker[i], pts, gDummyPayload, dataSize[i]) == 0)
                {
                    talkerStats[i].sentPackets += 1;
                }
                else
                {
                    talkerStats[i].errorPackets += 1;
                }
            }
        }
        talkerRunTime_us += 125;
    }

    for (int i = TX_FIRST_STREAM; i < MAX_TX_STREAMS; i++)
    {
        aaf_avtpc_talker_close(talker[i]);
    }

    DebugP_log("Closed all the AAF PCM Talkers\n");
}

static int autoamp_avtpRxPacketCallback(uint8_t *payload, int plsize,
                                avbtp_rcv_cb_info_t *cbinfo, void *cbdata)
{
#if (AVTP_CRF_TALKER_ENABLED == 1) || (AVTP_CRF_LISTENER_ENABLED == 1)
    if(cbinfo->u.rcrfinfo.subtype==AVBTP_SUBTYPE_CRF){
        return crfrx_callback(payload, plsize, cbinfo, gCrfData);
    }
#endif
    int status = 0;
    /*
     * Look up callback fn from Stream ID.
     * To simplify this application demo, Simple stream ID to
     * Function callback mapping is implemented. First 7 bytes
     * of the stream ID must be "0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x00"
     */
    const ub_streamid_t *sid = &cbinfo->u.rsdinfo.stream_id;

    /* Check the first 7 Bytes, it should match the
    constrained stream ID else, the callback map will not work. */
    DebugP_assert(memcmp(CONST_PART_SID, sid, 7) == 0);

    /* First 7 Bytes of stream ID is same,
    Just call callback function from the lookup. */
    const rxStream_cb *streamCbInfo = &gStreamCbArr[((uint8_t*)sid)[7]];

    if (streamCbInfo != NULL)
    {
        status = streamCbInfo->avtp_stream_cb(payload, plsize, cbinfo, streamCbInfo->arg);
    }

    return status;
}

void start_aaf_pcm_listener(char *netdev)
{
    int status = 0;
    if (MAX_RX_STREAMS > 0)
    {
        /* Configure the RX Stream. */
        const avtp_pcm_conf *conf = &gRxStreamConf[RX_FIRST_STREAM];

        conl2_basic_conparas_t conl2Params;
        init_conl2_params(conf, &conl2Params);

        aaf_avtpc_listener_data_t* listener = \
                 aaf_avtpc_listener_init(&conl2Params, autoamp_avtpRxPacketCallback, NULL);
        if (listener != NULL)
        {
            status = 0;
        }
        else
        {
            status = -1;
        }

        if (status == 0)
        {
            for (int i = RX_FIRST_STREAM; i < MAX_RX_STREAMS; i++)
            {
                /* Register for the stream id specific callback function. */
                const avtp_pcm_conf* conf = &gRxStreamConf[i];
                DebugP_assert(memcmp(CONST_PART_SID, conf->streamID, 7) == 0);

                /* Check if callback is registered already. */
                uint8_t index = ((uint8_t*)conf->streamID)[7];

                rxStream_cb *streamCbInfo = &gStreamCbArr[index];

                char streamID_string[100]; const uint8_t *sid = conf->streamID;
                sprintf(streamID_string, "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
                        sid[0],sid[1],sid[2],sid[3],sid[4],sid[5],sid[6],sid[7]);

                if (streamCbInfo->avtp_stream_cb != NULL)
                {
                    UB_LOG(UBL_INFO, "Stream Registered already %s\n", streamID_string);
                }
                else
                {
                    streamCbInfo->avtp_stream_cb = autoamp_rxStreamCallback;
                    streamCbInfo->arg            = (void*)&listenerStats[i];
                    /* Initialize stats. */
                    initStats(&listenerStats[i]);
                    UB_LOG(UBL_INFO, "Registered Callback for streamID:%s\n", streamID_string);
                }

            }
        }
    }
    CB_SLEEP(GPTP_WAIT_TIME_SEC);
    while (status == 0)
    {
        CB_SLEEP(1);
        for (int i = RX_FIRST_STREAM; i < MAX_RX_STREAMS; i++)
        {
            char stream_name[100];
            snprintf(stream_name, sizeof(stream_name), "Stream idx[%d]:", i);
            printStats(&listenerStats[i], stream_name);
            // initStats(&listenerStats[i]);
            listenerStats[i].minLatencyUs = INT32_MAX;
            listenerStats[i].maxLatencyUs = 0;
        }
    }
}
static void inline initStats(rx_stream_stats *stats)
{
    stats->minLatencyUs = INT32_MAX;
    stats->maxLatencyUs = 0;
    stats->avgLatencyUs = 0;
}

static void inline updateStats(const int64_t currentVal, rx_stream_stats *stats)
{
    if (currentVal < stats->minLatencyUs) stats->minLatencyUs = currentVal;
    if (currentVal > stats->maxLatencyUs) stats->maxLatencyUs = currentVal;
    /* Assumes that the receivedPackets is incremented before calling this function. */
    stats->avgLatencyUs = ((stats->avgLatencyUs)*(double)(stats->receivedPackets-1) \
                                     + currentVal)/(stats->receivedPackets);
}

static void inline printStats(const rx_stream_stats *stats, char* printHeader)
{
#if MEASURE_AVB_LATENCY
    DebugP_log("%s Latency min: %dus, max: %dus, avg: %uus, total: %llu, missed: %llu\r\n",
                printHeader,
                stats->minLatencyUs,
                stats->maxLatencyUs,
                stats->avgLatencyUs,
                stats->receivedPackets,
                stats->missedPackets);
#else
    DebugP_log("%s Packets Received: %llu, missed: %llu\r\n", printHeader,
                                                stats->receivedPackets, stats->missedPackets);
#endif
}

static int autoamp_rxStreamCallback(uint8_t *payload, int plsize,
                                avbtp_rcv_cb_info_t *cbinfo, void *cbdata)
{
    rx_stream_stats* stats = (rx_stream_stats*)cbdata;
    stats->receivedPackets += 1;

    const avbtp_sd_info_t* sdInfo = &cbinfo->u.rsdinfo;
    /*
     * Latency Measurement ends here.....
     * Measure Latency. It is assumed that the packets's presentation time is
     * filled with the presentation time in us.
     *
     * To get more accurate latency numbers, Toggle a GPIO here...
     */
#if MEASURE_AVB_LATENCY
    const uint64_t gptpTimeUs = sdInfo->rxts/1000;
#else
    const uint64_t gptpTimeUs = 0;
#endif
    const int64_t latency = gptpTimeUs-sdInfo->timestamp;

    if (stats->receivedPackets == 1)
    {
        /* First RX packet. */
    }
    else
    {
        /* Count the missing packets. */
        stats->missedPackets += (cbinfo->u.rsdinfo.seqn_diff - 1);
    }

    updateStats(latency, stats);
    return 0;
}

/* Timer callback registed in syscfg. */
void avbTickTimer_callback(void *args)
{
    SemaphoreP_post(&gAvbTickSem);
}