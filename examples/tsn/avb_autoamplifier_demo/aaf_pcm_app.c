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

#include "aaf_pcm_app.h"
#include "tsn_conl2/aaf_avtpc_listener.h"
#include "tsn_conl2/aaf_avtpc_talker.h"
#include "sample_audio.h"
#include "tsn_gptp/gptpmasterclock.h"
#include "debug_log.h"
#include "kernel/dpl/HwiP.h"
#include "kernel/dpl/TimerP.h"
#include "ti_dpl_config.h"
#include "est_configure.h"

#define CONF_TSOFFSET_USEC 200000

// t1 - at Timer ISR
// t2 - at EnetDma_SubmitTx
// t3 - after SubmitTx call is done
typedef struct talker_tx_analysis
{
    uint32_t t1;
    uint32_t t2;
    uint32_t t3;
} talker_tx_analysis_t;

typedef struct {
	void (*avtpc_close)(void *avtpc_talker);
	int (*avtpc_send)(void *avtpc_talker, uint64_t pts,
			  uint8_t *audio_data, uint32_t data_size);
	void *avtpc_talker;
	int read_size;
	uint32_t sent_bytes;
	uint32_t sent_packets;
    talker_tx_analysis_t txanalysis[1]; // using sent_packets as counter. Stop talker once sent_packets = 10000
    uint32_t delay_packets; // count if processing time >= 250us
    uint32_t ok_packets;
    uint32_t max_sending_time; // count if processing time >= 250us
} audio_talker_t;

uint32_t current_sending_time_t2 = 0;
uint32_t current_sending_time_t1 = 0;
int current_active_app = 0; // Current app is set before call to avtpc_send

// expect max loss is 100
#define MAX_LOST 100
typedef struct
{
    uint64_t lost_at_rxts;
    int64_t interval;
    uint8_t seq_diff;
    uint32_t pkt_count;
}loss_pkt_info_t;

#define MAX_ARRAY 20000
typedef struct {
    bool is_first_pkt_rx;
    uint64_t max_rxinterval;
    uint64_t min_rxinterval;
    float total_rxinterval; // classA should be around 125us, classB should be 1000us
    uint32_t rx_count;      // avg = total rx interval / rx_count
    bool is_rx_enabled;
    uint64_t prv_rxts;
    uint64_t rxts;
    uint8_t prv_seq;
    float diffarr[MAX_ARRAY]; // in ms
    loss_pkt_info_t lostpktinfo[MAX_LOST];
    int totallostpkt;
    bool rx_done;
} audio_stream_info_t;

typedef struct {
	void (*avtpc_close)(void *avtpc_listener);
	void *avtpc_listener; /* aaf or iec61883_6 */
    audio_stream_info_t rxstreams[6];
} audio_listener_t;

uint8_t txbuf[1500];
// appno can run from 0 --> 5
audio_talker_t gaudioTalker[6] = {0};
audio_listener_t gaudioListener;
SemaphoreP_Object gTimerSem[6];


uint32_t EnetApp_getClockCount(void)
{
    return TimerP_getCount(gTimerBaseAddr[CONFIG_TIMER0]);
}

uint32_t EnetApp_getDurationDiff(uint32_t startClock)
{
    uint32_t elapsedClock = 0;
    uint32_t endClock = EnetApp_getClockCount();
    uint32_t diff_count;

    if (endClock >= startClock)
    {
        elapsedClock = endClock - startClock;
    }
    else /* overflow. timerP count from start to MAX_TIMER_COUNT_VALUE then from reload to end */
    {
        elapsedClock = MAX_TIMER_COUNT_VALUE - startClock;
        elapsedClock += endClock - TimerP_getReloadCount(gTimerBaseAddr[CONFIG_TIMER0]);
    }

    diff_count = elapsedClock *1000000U;
    diff_count /= CONFIG_TIMER0_INPUT_CLK_HZ;

    return diff_count;
}

/// @brief Use for lldenet to know which app to update stat
/// @todo: consider conflict?
/// @param appno
void update_delay_info_t2()
{
    // gaudioTalker[current_active_app].txanalysis[gaudioTalker[current_active_app].sent_packets].t2 = CycleCounterP_getCount32();
}
void update_delay_info_t3()
{
    // gaudioTalker[current_active_app].txanalysis[gaudioTalker[current_active_app].sent_packets].t3 = CycleCounterP_getCount32();
}

static int audio_aaf_avtp_push_packet(uint8_t *payload, int plsize, avbtp_rcv_cb_info_t *cbinfo, void *cbdata);

static void audio_aaf_avtpc_talker_close(void *avtpc_talker)
{
	if (avtpc_talker) aaf_avtpc_talker_close(avtpc_talker);
}

static void audio_aaf_avtpc_listener_close(void *audio_listener)
{
	if (audio_listener) aaf_avtpc_listener_close(audio_listener);
}

static int audio_aaf_avtpc_talker_send(void *avtpc_talker, uint64_t pts,
					uint8_t *audio_data, uint32_t data_size)
{
	return aaf_avtpc_talker_write(avtpc_talker, pts, audio_data, data_size);
}

static int audio_aaf_talker_init(int appno, conl2_basic_conparas_t* basic_param, aaf_avtpc_pcminfo_t* pcm_info)
{
    gaudioTalker[appno].avtpc_talker = aaf_avtpc_talker_init(basic_param);
	if (!gaudioTalker[appno].avtpc_talker) {
		DPRINT("%s:failed to open aaf talker\n",__func__);
		return -1;
	}
    aaf_avtpc_set_pcm_info(gaudioTalker[appno].avtpc_talker, pcm_info);

    gaudioTalker[appno].avtpc_close = audio_aaf_avtpc_talker_close;
	gaudioTalker[appno].avtpc_send = audio_aaf_avtpc_talker_send;

	return 0;
}

static int audio_aaf_listener_init(conl2_basic_conparas_t* basic_param, aaf_avtpc_pcminfo_t* pcm_info)
{
    gaudioListener.avtpc_listener = aaf_avtpc_listener_init(basic_param, audio_aaf_avtp_push_packet, &gaudioListener);
    if (!gaudioListener.avtpc_listener) {
		DPRINT("%s:failed to open aaf listener\n",__func__);
		return -1;
	}

    gaudioListener.avtpc_close=audio_aaf_avtpc_listener_close;
    return 0;
}

/// In case of listener, set to -1
static void init_default_params(conl2_basic_conparas_t* basic_param, aaf_avtpc_pcminfo_t* pcm_info, uint8_t appno, int txintervalus)
{
    basic_param->vid = 110;
    basic_param->send_ahead_ts = 20000; // ahead 20ms

    /// App1: 2 tx streams, appno = 00, 05
    ///       4 rx streams, appno = 01, 02, 03, 04
    /// App2: 4 tx streams, appno = 01, 02, 03, 04
    ///       2 rx streams, appno = 00, 05
    ub_streamid_t streamid = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x00, appno};
    memcpy(basic_param->streamid, streamid, sizeof(streamid));

    ub_macaddr_t mcastmac = {0x91, 0xE0, 0xF0, 0x00, 0xFE, appno};
    memcpy(basic_param->mcastmac, mcastmac, sizeof(mcastmac));

    basic_param->time_intv = txintervalus; // app0: class A, app1 classD
    basic_param->max_frm_sz = 1500;
    basic_param->max_intv_frames = 1;
    basic_param->pcp = (txintervalus == 125)? 3:2;
    basic_param->avtpd_bufftime_us = 10000;
    /// In case of direct mode enabled, application needs to self filter stream
    basic_param->is_direct = true;


    pcm_info->format = AVBTP_AAF_FORMAT_INT_16BIT;
    pcm_info->bit_depth = 16;
    pcm_info->channels = 8;
    pcm_info->srate = 48000;

    // we won't care about data
    memset(txbuf, 0x01, sizeof(txbuf));
}

int start_aaf_pcm_talker(char* netdev, int appno, int txintervalus, int channels)
{
    uint64_t pts;
    uint64_t sts, ets, tsdiff; // start and end ts, use to print report
    conl2_basic_conparas_t basic_param;
    aaf_avtpc_pcminfo_t pcm_info;

    memset(&basic_param, 0, sizeof(basic_param));
    memset(&pcm_info, 0, sizeof(pcm_info));
    memcpy(basic_param.netdev, netdev, strlen(netdev));
    init_default_params(&basic_param, &pcm_info, appno, txintervalus);

    while(gptpmasterclock_init(NULL)){
		UB_LOG(UBL_INFO,"Waiting for tsn_gptpd to be ready...\n");
		CB_USLEEP(100000);
	}

    if (audio_aaf_talker_init(appno, &basic_param, &pcm_info) == -1)
    {
        UB_LOG(UBL_INFO,"Initial appno=%d failed\n", appno);
        return -1;
    }
    else
    {
        UB_LOG(UBL_INFO,"appno=%d waiting for est config before streaming\n", appno);
    }

#ifdef WITH_EST_CONFIG
    wait_est_configured();
#endif

    // 125us, 16 channels => buffer size = 48 (frames per 1ms) * 125(us) / 1000 (us) * 2 (16 bit-depth) * 16 (channels) = 192 bytes
    // 125us, 8 channels => buffer size = 48 (frames per 1ms) * 125(us) / 1000 (us) * 2 (16 bit-depth) * 8 (channels) = 96 bytes
    // 1000us, 8 channels => buffer size = 48 (frames per 1ms) * 1000(us) / 1000 (us) * 2 (16 bit-depth) * 8 (channels) = 768 bytes
    // 1000us, 12 channels => buffer size = 48 (frames per 1ms) * 1000(us) / 1000 (us) * 2 (16 bit-depth) * 12 (channels) = 1152 bytes
    int pcmbuf_size = 48 * txintervalus / UB_MSEC_US * channels * 2;
    sts = ub_mt_gettime64();
    // repeat: start from 0 and read until last sync frame
    while(true)
    {
        SemaphoreP_pend(&gTimerSem[appno], SystemP_WAIT_FOREVER);

        // pts = gptpmasterclock_getts64();
        pts = 0;

        current_active_app = appno;
        if (gaudioTalker[appno].avtpc_send( gaudioTalker[appno].avtpc_talker,
                                pts + CONF_TSOFFSET_USEC,
                                &txbuf[0],
                                pcmbuf_size) < 0)
        {
            return -1;
        }
        // gaudioTalker[appno].sent_packets++;
        // gaudioTalker[appno].sent_bytes += pcmbuf_size;

        // if (gaudioTalker[appno].sent_packets == 10000) break;

#if 0
        ets = ub_mt_gettime64();
        tsdiff = ets - sts;
        if (tsdiff >= 5*UB_SEC_NS) {
            // uint64_t tmp_send_pkt = (uint64_t) gaudioTalker[appno].sent_packets * (uint64_t)UB_SEC_NS;
            // uint64_t tmp_send_bytes = (uint64_t) gaudioTalker[appno].sent_bytes * 8000;
            UB_LOG(UBL_INFO,"["UB_PRIhexB8"] ok=%d delay=%d|t1+t2[0]=%d|t1[0]=%d|t2[0]=%d|max sendingtime=%d\n",
                UB_ARRAY_B8(basic_param.streamid),
                 gaudioTalker[appno].ok_packets,
                gaudioTalker[appno].delay_packets,
                (gaudioTalker[appno].delay_packets > 0) ? gaudioTalker[appno].delay_info[0].sending_time:0, // currently print only the first one
                (gaudioTalker[appno].delay_packets > 0) ? gaudioTalker[appno].delay_info[0].sending_time_t1:0, // currently print only the first one
                (gaudioTalker[appno].delay_packets > 0) ? gaudioTalker[appno].delay_info[0].sending_time_t2:0, // currently print only the first one
                gaudioTalker[appno].max_sending_time);
            gaudioTalker[appno].sent_packets = 0;
            gaudioTalker[appno].sent_bytes = 0;
            gaudioTalker[appno].delay_packets = 0;
            gaudioTalker[appno].ok_packets = 0;
            gaudioTalker[appno].max_sending_time = 0;

            // (void)tmp_send_pkt;
            // (void)tmp_send_bytes;

            sts = ets;
        }
#else
        (void) sts;
        (void) ets;
        (void) tsdiff;
#endif
    }

    for (int i=0; i<gaudioTalker[appno].sent_packets; i++)
    {
        UB_LOG(UBL_INFO,"["UB_PRIhexB8"] t1|t2|t3=%u|%u|%u\n",
                UB_ARRAY_B8(basic_param.streamid),
                gaudioTalker[current_active_app].txanalysis[i].t1,
                gaudioTalker[current_active_app].txanalysis[i].t2,
                gaudioTalker[current_active_app].txanalysis[i].t3);

    }

    return 0;
}

static int audio_aaf_avtp_push_packet(uint8_t *payload, int plsize,
				avbtp_rcv_cb_info_t *cbinfo, void *cbdata)
{
#ifdef RX_REPORT
    int64_t interval;

    avbtp_sd_info_t *rsdinfo=&cbinfo->u.rsdinfo;
    audio_listener_t *audio_listener = &gaudioListener;
    audio_stream_info_t *streaminfo = &audio_listener->rxstreams[rsdinfo->stream_id[7]];
    if (!audio_listener || !streaminfo->is_rx_enabled) {return -1;}

    if (streaminfo->is_first_pkt_rx == false)
    {
        streaminfo->prv_rxts = rsdinfo->current_timestamp;
        interval = 0;
        streaminfo->is_first_pkt_rx = true;
        UB_LOG(UBL_INFO,"%s: appno=%u Rx first packet at ts %" PRIu64 "\n", __func__, rsdinfo->stream_id[7], streaminfo->prv_rxts);
    }
    else
    {
        if(rsdinfo->seqn_diff > 1)
        {
            // UB_LOG(UBL_INFO,"%s: appno=%u lost pkg. seqdiff=%d. seq=%d. rxcount=%d\n", __func__, rsdinfo->stream_id[7], seq_diff, rsdinfo->seqn_diff, streaminfo->rx_count);
            if (streaminfo->totallostpkt<MAX_LOST)
            {
                streaminfo->lostpktinfo[streaminfo->totallostpkt].lost_at_rxts = rsdinfo->current_timestamp;
                streaminfo->lostpktinfo[streaminfo->totallostpkt].interval = rsdinfo->current_timestamp - streaminfo->prv_rxts;
                streaminfo->lostpktinfo[streaminfo->totallostpkt].seq_diff = rsdinfo->seqn_diff;
                streaminfo->lostpktinfo[streaminfo->totallostpkt].pkt_count = streaminfo->rx_count +1;
                streaminfo->totallostpkt++;
            }
        }

        streaminfo->rxts=rsdinfo->current_timestamp;
        interval = streaminfo->rxts - streaminfo->prv_rxts;
        streaminfo->prv_rxts = streaminfo->rxts;

        if (streaminfo->rx_count < MAX_ARRAY)
        {
            streaminfo->diffarr[streaminfo->rx_count] = (float)interval/(float)UB_USEC_NS;
            streaminfo->rx_count ++;
        }
        else
        {
            streaminfo->rx_done=true;
        }
    }
#else
    // Simply do nothing
#endif
    return 0;
}
uint8_t mon_streams[4]; // max is 4 rx
static void enable_monitor_stream()
{
    memset(&gaudioListener, 0, sizeof(gaudioListener));
    memset (&mon_streams[0], 0xFF, sizeof(mon_streams));
    int mon_index=0;

#ifdef AAF_RX_1_APPNO
    gaudioListener.rxstreams[AAF_RX_1_APPNO].is_rx_enabled = true;
    mon_streams[mon_index] = AAF_RX_1_APPNO;
    mon_index++;
#endif
#ifdef AAF_RX_2_APPNO
    gaudioListener.rxstreams[AAF_RX_2_APPNO].is_rx_enabled = true;
    mon_streams[mon_index] = AAF_RX_2_APPNO;
    mon_index++;
#endif
#ifdef AAF_RX_3_APPNO
    gaudioListener.rxstreams[AAF_RX_3_APPNO].is_rx_enabled = true;
    mon_streams[mon_index] = AAF_RX_3_APPNO;
    mon_index++;
#endif
#ifdef AAF_RX_4_APPNO
    gaudioListener.rxstreams[AAF_RX_4_APPNO].is_rx_enabled = true;
    mon_streams[mon_index] = AAF_RX_4_APPNO;
#endif
    for (int i=0; i<4; i++)
    {
        UB_LOG(UBL_INFO, "Monitor rxstream[7]=%u\n", mon_streams[i]);
    }

    (void) mon_index;
}

static bool is_rxalldone()
{
    bool isdone = true;
    for (int i=0; i<4; i++)
    {
        if (mon_streams[i] == 0xFF) break;
        if (!gaudioListener.rxstreams[mon_streams[i]].rx_done)
        {
            isdone = false;
            break;
        }
    }

    if (isdone) {
        UB_LOG(UBL_INFO, "All stream rx enough data to analyzed\n");
    }
    return isdone;
}

int start_aaf_pcm_listener(char* netdev)
{
    conl2_basic_conparas_t basic_param;
    aaf_avtpc_pcminfo_t pcm_info;

    memset(&basic_param, 0, sizeof(basic_param));
    memset(&pcm_info, 0, sizeof(pcm_info));
    memcpy(basic_param.netdev, netdev, strlen(netdev));
    init_default_params(&basic_param, &pcm_info, 0xFF, -1);

    enable_monitor_stream();

	while(gptpmasterclock_init(NULL)){
		UB_LOG(UBL_INFO,"Waiting for tsn_gptpd to be ready...\n");
		CB_USLEEP(100000);
	}

    if (audio_aaf_listener_init(&basic_param, &pcm_info) == -1)
    {
        UB_LOG(UBL_INFO,"Cannot init listener\n");
        return -1;
    }

    while(!is_rxalldone())
    {
        CB_USLEEP(200000);
        ub_log_flush();
    }

    // gaudioListener.avtpc_close(gaudioListener.avtpc_listener);

#ifdef RX_REPORT
    for (int i=0; i<4; i++)
    {
        if (mon_streams[i] == 0xFF) break;
        UB_LOG(UBL_INFO, "[RX=%d] total lost: %d\n", mon_streams[i], gaudioListener.rxstreams[mon_streams[i]].totallostpkt);
        if (gaudioListener.rxstreams[mon_streams[i]].totallostpkt > 0)
        {
            for (int j=0; j<gaudioListener.rxstreams[mon_streams[i]].totallostpkt; j++)
            {
                UB_LOG(UBL_INFO, "[%d] pkglost rxts=%" PRIu64 "|tsdiff=%" PRId64 "|seqdiff=%d|pktcounter=%d \n",
                                 j,
                                 gaudioListener.rxstreams[mon_streams[i]].lostpktinfo[j].lost_at_rxts,
                                 gaudioListener.rxstreams[mon_streams[i]].lostpktinfo[j].interval,
                                 gaudioListener.rxstreams[mon_streams[i]].lostpktinfo[j].seq_diff,
                                 gaudioListener.rxstreams[mon_streams[i]].lostpktinfo[j].pkt_count);
            }
        }
        for (int j=0; j<MAX_ARRAY; j++)
        {
            UB_LOG(UBL_INFO, "[RX=%d] %f \n", mon_streams[i], gaudioListener.rxstreams[mon_streams[i]].diffarr[j]);
        }
    }
#endif
    return 0;
}

void init_hw_timer(int appno)
{
    SemaphoreP_constructBinary(&gTimerSem[appno], 0);
}

void start_hw_timer()
{
    TimerP_start(gTimerBaseAddr[CONFIG_TIMER0]);
}

void avbTimerIsrClassA(void)
{
    static int counter = 0;
#ifdef AAF_TX_CLASS_A_APPNO
    // gaudioTalker[AAF_TX_CLASS_A_APPNO].txanalysis[gaudioTalker[AAF_TX_CLASS_A_APPNO].sent_packets].t1 = CycleCounterP_getCount32();
    SemaphoreP_post(&gTimerSem[AAF_TX_CLASS_A_APPNO]);
#endif

    counter++;
    if (counter % 8 == 0)
    {
#ifdef AAF_TX_CLASS_D1_1_APPNO
        SemaphoreP_post(&gTimerSem[AAF_TX_CLASS_D1_1_APPNO]);
#endif
#ifdef AAF_TX_CLASS_D1_2_APPNO
        SemaphoreP_post(&gTimerSem[AAF_TX_CLASS_D1_2_APPNO]);
#endif
#ifdef AAF_TX_CLASS_D1_3_APPNO
        SemaphoreP_post(&gTimerSem[AAF_TX_CLASS_D1_3_APPNO]);
#endif
        counter=0;
    }
}