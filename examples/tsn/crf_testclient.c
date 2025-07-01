/*
 * crf_testappli.c
 * test program to use l2 layer and crf format.
 * Only AVBTP_CRF_TYPE_AUDIO_SAMPLE data type is supported in this test program.
 *
 * Copyright (C) 2021 Excelfore Corporation
 * Author: Nam Nguyen Van (nam.nguyen@excelfore.com)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <getopt.h>
#include "avtpc_crf.h"
#include "tsn_gptp/gptpmasterclock.h"
#include <tsn_unibase/unibase_binding.h>
#include "enet_media_clock_ctrl/media_clock_ctrl.h"
#include "kernel/dpl/TimerP.h"
#include "ti_dpl_config.h"

#include "avb_autoamplifier_demo/aaf_pcm_app.h"

#ifdef AVTP_PLATFORM_INCLUDE
#include AVTP_PLATFORM_INCLUDE
#endif
#ifndef WAIT_AVTPD_READY
#define WAIT_AVTPD_READY
#endif

#define DEFAULT_MCAST_ADDR "91:E0:F0:00:FE:00"

#define TS_PER_AVTPDU 6 //Recommended timestamps per CRF AVTPDU
#define AVTPDU_PER_SEC 50 //Recommended CRF AVTPDU per second

static int s_running = 1;
CB_SEM_T gCRF_Tick;
typedef struct crfdata {
    avtpc_crf_data_t *avtpc_crf;
    char mode;
    char *netdev;
    uint16_t vid;
    char *mcast;
    char *sid;
    uint8_t vlan_pcp;
    char *shsuf;
    char *gshmem;
    uint64_t tscomp; //ts compensate in nanosec
    bool echoback;
    uint32_t tsintv;
    uint32_t base_freq;
    uint8_t ts_buffer[TS_PER_AVTPDU * 8];
    uint64_t array_ts[TS_PER_AVTPDU];
    uint64_t npdus;
    uint64_t sts, ets, run_time;
    int64_t tsdiff_accum, tsdiff_min, tsdiff_max;
    /* Last crfts of previous packet. It is used to check crfts
     * gap between 2 CRF packets. */
    uint64_t prev_lastcrfts;
    /* if this value greater than 0, then generate random frequency
     * b/w [freq*(1-ppm/10^6), freq*(1+ppm/10^6)] */
    uint32_t ppm;
    bool direct;
    uint16_t tsport; //UDP test source port in direct mode
    uint16_t tdport; //UDP test dest port in direct mode
} crfdata_t;

static int print_usage(char *pname, crfdata_t *crfdata)
{
    char *s;
    if((s=strchr(pname,'/'))==NULL){s=pname;}
    UB_CONSOLE_PRINT("%s [options]\n", s);
    UB_CONSOLE_PRINT("-h|--help: this help\n");
    UB_CONSOLE_PRINT("-m|--mode: t (talker), l (listener): default=%c\n", crfdata->mode);
    UB_CONSOLE_PRINT("-d|--netdev: network_device: defaut=%s\n", crfdata->netdev);
    UB_CONSOLE_PRINT("-v|--vid: vlan id: defaut=%d\n", crfdata->vid);
    UB_CONSOLE_PRINT("-a|--mcast: multicast address: default=%s\n", DEFAULT_MCAST_ADDR);
    UB_CONSOLE_PRINT("-s|--sid: stream id: default=%s\n", crfdata->sid);
    UB_CONSOLE_PRINT("-p|--pcp: pcp value in vlan tag: default=%d\n", crfdata->vlan_pcp);
    UB_CONSOLE_PRINT("-e|--echoback: receive CRF packets back after sending\n");
    UB_CONSOLE_PRINT("-c|--tscomp: ts compensate nsec: default=%"PRIu64"\n", crfdata->tscomp);
    UB_CONSOLE_PRINT("-t|--tsintv: timestamp interval: default=%d\n", crfdata->tsintv);
    UB_CONSOLE_PRINT("-f|--freq: base frequency: default=%d\n", crfdata->base_freq);
    UB_CONSOLE_PRINT("-S|--shsuf: suffix: shared memory suffix\n");
    UB_CONSOLE_PRINT("-g|--gshmem: node : gptp shared memory node\n");
    UB_CONSOLE_PRINT("-r|--ppm: generate random frequency b/w "
                     "[freq*(1-ppm/10^6), freq*(1+ppm/10^6)]\n");
    UB_CONSOLE_PRINT("-i|--direct: enable avtpc_crf direct mode\n");
    UB_CONSOLE_PRINT("-k|--tsport: src_port: UDP test source port in direct mode\n");
    UB_CONSOLE_PRINT("-K|--tdport: dst_port: UDP test dest port in direct mode\n");
    return -1;
}

static int set_options(crfdata_t *crfdata, int argc, char *argv[])
{
    int oc;
    int res=0;
    struct option long_options[] = {
        {"help", no_argument, 0, 'h'},
        {"mode", required_argument, 0, 'm'},
        {"netdev", required_argument, 0, 'd'},
        {"vid", required_argument, 0, 'v'},
        {"mcast", required_argument, 0, 'a'},
        {"sid", required_argument, 0, 's'},
        {"pcp", required_argument, 0, 'p'},
        {"echoback", no_argument, 0, 'e'},
        {"tscomp", required_argument, 0, 'c'},
        {"tsintv", required_argument, 0, 't'},
        {"freq", required_argument, 0, 'f'},
        {"shsuf", required_argument, 0, 'S'},
        {"gshmem", required_argument, 0, 'g'},
        {"ppm", required_argument, 0, 'r'},
        {"direct", required_argument, 0, 'i'},
        {"tsport", required_argument, 0, 'k'},
        {"tdport", required_argument, 0, 'K'},
        {"noubinit", no_argument, 0, 'u'},
        {0, 0, 0, 0}
    };

    while((oc=getopt_long(argc, argv, "hm:d:v:a:s:p:ec:t:f:S:g:r:ik:K:u",
                          long_options, NULL))!=-1){
        switch(oc){
        case 'm':
            crfdata->mode=optarg[0];
            if(crfdata->mode!='t' && crfdata->mode!='l'){return -1;}
            break;
        case 'd':
            crfdata->netdev=optarg;
            break;
        case 'v':
            crfdata->vid=strtol(optarg, NULL, 0);
            break;
        case 'a':
            crfdata->mcast=optarg;
            break;
        case 's':
            crfdata->sid=optarg;
            break;
        case 'p':
            crfdata->vlan_pcp=strtol(optarg, NULL, 0);
            break;
        case 'e':
            crfdata->echoback=true;
            break;
        case 'c':
            crfdata->tscomp=strtoll(optarg, NULL, 0);
            break;
        case 't':
            crfdata->tsintv=strtol(optarg, NULL, 0);
            break;
        case 'f':
            crfdata->base_freq=strtol(optarg, NULL, 0);
            break;
        case 'S':
            crfdata->shsuf=optarg;
            break;
        case 'g':
            crfdata->gshmem=optarg;
            break;
        case 'r':
            crfdata->ppm=strtol(optarg, NULL, 0);
            break;
        case 'i':
            crfdata->direct=true;
            break;
        case 'k':
            crfdata->tsport=(uint16_t)strtol(optarg, NULL, 0);
            break;
        case 'K':
            crfdata->tdport=(uint16_t)strtol(optarg, NULL, 0);
            break;
        case 'u':
            //noubinit must be the last one
            break;
        case 'h':
            print_usage(argv[0], crfdata);
            res=1;
            break;
        default:
            return -1;
        }
    }
    return res;
}

int crfrx_callback(uint8_t *payload, int payload_size,
                      avbtp_rcv_cb_info_t *cbinfo, void *cbdata)
{
    crfdata_t *crfdata = cbdata;
    avbtp_crf_info_t *rcrfinfo = &cbinfo->u.rcrfinfo;
    int num_ts;
    float freq;
    int pdups;
    int64_t tsdiff;

    if(rcrfinfo->subtype!=AVBTP_SUBTYPE_CRF){
        UB_LOG(UBL_INFO,"%s:Unsupported subtype: %d\n",__func__, rcrfinfo->subtype);
        return -1;
    }

    num_ts = avbtp_deserialize_stdtype_crf_data(crfdata->array_ts,
            TS_PER_AVTPDU, payload, payload_size);
    if (num_ts < 0) {
        UB_LOG(UBL_ERROR,"%s: deserialized CRF failed.\n",__func__);
        return -1;
    }

    //Checking only the first ts is enough
    tsdiff = crfdata->array_ts[0] - gptpmasterclock_getts64();
    if (!crfdata->tsdiff_min || crfdata->tsdiff_min > tsdiff) {
        crfdata->tsdiff_min = tsdiff;
    }
    if (!crfdata->tsdiff_max || crfdata->tsdiff_max < tsdiff) {
        crfdata->tsdiff_max = tsdiff;
    }
    crfdata->tsdiff_accum += tsdiff;

    if (rcrfinfo->seqn_diff != 1) {
        UB_LOG(UBL_INFO, "CRF packet: seqn_diff=%d\n", rcrfinfo->seqn_diff);
        //reset counter
        crfdata->npdus = 0;
        crfdata->sts = 0;
        crfdata->run_time = 0;
        crfdata->tsdiff_min = 0;
        crfdata->tsdiff_max = 0;
        crfdata->tsdiff_accum = 0;
    }

    freq = avbtp_crf_nominal_frequency(rcrfinfo->pull, rcrfinfo->base_frequency);

    crfdata->npdus++;
    if (!crfdata->sts){crfdata->sts = ub_mt_gettime64();}
    crfdata->ets = ub_mt_gettime64();
    if (crfdata->ets - crfdata->sts > crfdata->run_time + UB_SEC_NS ) {
        crfdata->run_time = crfdata->ets - crfdata->sts;
        pdups = (int)((double)crfdata->npdus/
                      ((double)crfdata->run_time/(double)UB_SEC_NS));

        UB_LOG(UBL_INFO,"CRF listener "UB_PRIhexB8":pdus/sec=%d "
               "freq_Hz=%d ts_interval=%d tsdiff_ns={min=%"PRIi64
               " avrg=%"PRIi64" max=%"PRIi64"} crftsgap_ns=%"PRIu64"\n",
               UB_ARRAY_B8(rcrfinfo->stream_id), pdups,
               (int)freq, rcrfinfo->timestamp_interval,
               crfdata->tsdiff_min,
               crfdata->tsdiff_accum/(int64_t)crfdata->npdus,
               crfdata->tsdiff_max,
               crfdata->array_ts[0]-crfdata->prev_lastcrfts
            );
    }
    crfdata->prev_lastcrfts = crfdata->array_ts[num_ts-1];

    return 0;
}

/* This example supports only AVBTP_CRF_TYPE_AUDIO_SAMPLE */
static int start_crf(crfdata_t *crfdata)
{
    client_connect_request_t ccr;
    avtpc_crf_config_t crfcfg;
    double period;
    float freq;
    int res, pdups;
    uint32_t crfts_period;

    memset(&crfcfg, 0, sizeof(crfcfg));

    crfcfg.netdev = crfdata->netdev;
    crfcfg.vlan_id = crfdata->vid;
    crfcfg.vlan_pcp = crfdata->vlan_pcp;
    crfcfg.listener = (crfdata->mode == 'l');
    crfcfg.stream_id = crfdata->sid;
    crfcfg.dest_mcast = crfdata->mcast;
    crfcfg.join_mcast = crfdata->mcast;
    crfcfg.pull = AVBTP_CRF_PULL_MULT_1;
    crfcfg.type = AVBTP_CRF_TYPE_AUDIO_SAMPLE;
    crfcfg.timestamp_interval = crfdata->tsintv;
    crfcfg.base_frequency = crfdata->base_freq;
    crfcfg.mr_pdus_toggle = AVTPC_MR_PDUS_TOGGLE_MIN;

    crfdata->avtpc_crf=avtpc_crf_init(crfdata->shsuf);
    if (!crfdata->avtpc_crf){return -1;}

#ifndef AUTOAMP_APP_ENABLED
    if (crfcfg.listener) {
        avtpc_crf_set_rcv_cb(crfdata->avtpc_crf, crfrx_callback, crfdata);
    }
#else
    set_crf_cbdata(crfdata);
#endif
    if(crfdata->direct){
        if (crfcfg.listener) {
            avtpc_crf_set_rxdirect(crfdata->avtpc_crf);
        }else{
            avtpc_crf_set_txdirect(crfdata->avtpc_crf);
        }
    }
    if(crfdata->tsport || crfdata->tdport) {
        avtpc_crf_set_testport(crfdata->avtpc_crf, crfdata->tsport, crfdata->tdport);
    }

#if (AUTOAMP_APP_ENABLED && AVTP_CRF_LISTENER_ENABLED)
    (void)ccr;
#else
    if (avtpc_crf_set_ccr(crfdata->avtpc_crf, &crfcfg, &ccr) < 0) {
        goto end;
    }
    ccr.echoback = crfdata->echoback;

    if(avtpc_crf_connection_request(crfdata->avtpc_crf, &ccr)){
        goto end;
    }
#endif
    freq = avbtp_crf_nominal_frequency(crfcfg.pull, crfcfg.base_frequency);
    period = (double)1000000000/(double)freq;
    crfts_period = (uint32_t)(period * (double)crfcfg.timestamp_interval);

    //Assume ts is the timestamp of the first audio sample
    crfdata->sts = ub_mt_gettime64();

    MCC_handle hMCC = MCC_init(freq, crfts_period);
    #if AVTP_CRF_TALKER_ENABLED
    CB_SEM_INIT(&gCRF_Tick, 0, 0);
    TimerP_start(gTimerBaseAddr[CONFIG_TIMER0]);
    #endif

    while(s_running) {
        if (crfcfg.listener == false) {
            uint64_t crfTimestamps[2] = {0};
            for (int i = 0; i < 2;)
            {
                CB_SEM_WAIT(&gCRF_Tick);
                if (MCC_getTimestamp(hMCC, &crfTimestamps[i]) == 0)
                {
                    /* Extrapolate the timestamps. */
                    for (int j = 0; j < TS_PER_AVTPDU/2; j++)
                    {
                        crfdata->array_ts[3*i+j] = crfTimestamps[i] + (j * crfts_period) + 100*UB_MSEC_NS;
                    }
                    i++;
                }
            }

            res = avbtp_serialize_stdtype_crf_data(crfdata->array_ts,
                        TS_PER_AVTPDU, crfdata->ts_buffer, sizeof(crfdata->ts_buffer));
            if (res < 0) {
                UB_LOG(UBL_ERROR,"%s: serialized CRF failed.\n",__func__);
                break;
            }

            res = avtpc_crf_write_packet(crfdata->avtpc_crf,
                        NULL, crfdata->ts_buffer, res);
            if (res < 0) {
                UB_LOG(UBL_ERROR,"%s: send CRF failed.\n",__func__);
                break;
            }
            crfdata->npdus++;

            crfdata->ets = ub_mt_gettime64();
            if (crfdata->ets - crfdata->sts > crfdata->run_time + UB_SEC_NS ) {
                crfdata->run_time = crfdata->ets - crfdata->sts;
                pdups = (int)((double)crfdata->npdus/
                              ((double)crfdata->run_time/(double)UB_SEC_NS));

                UB_LOG(UBL_INFO,"CRF talker %s:pdus/sec=%d freq_Hz=%d ts_interval=%d\n",
                       crfcfg.stream_id, pdups, (int)freq, crfdata->tsintv);
            }

        } else {
            CB_SLEEP(1);
        }
    }
#if (AUTOAMP_APP_ENABLED && AVTP_CRF_LISTENER_ENABLED)
#else
    end:
    avtpc_crf_close(crfdata->avtpc_crf);
#endif
    return 0;
}


int crf_task(int argc, char *argv[])
{
    int res = -1;
    bool noubinit = false;
    static crfdata_t crfdata_td = {
        .avtpc_crf = NULL,
        .mode = 't',
        .netdev = "enp2s0",
        .vid = 110,
        .mcast = DEFAULT_MCAST_ADDR,
        .sid = "00:01:02:03:04:05:00:00",
        .vlan_pcp = IEEE1722_DEFAULT_PCP,
        .shsuf = NULL,
        .gshmem = NULL,
        .tscomp = 200000000,
        .echoback = false,
        .tsintv = 160,
        .base_freq = 48000,
        .direct = false,
    };
    if(argc>0 && (!strcmp(argv[argc-1],"--noubinit") || !strcmp(argv[argc-1], "-u"))){
        noubinit=true;
        UB_CONSOLE_PRINT("crf_testclient: noubinit=true\r\n");
    }
    if(!noubinit){
        unibase_init_para_t init_para;
        ubb_default_initpara(&init_para);
        init_para.ub_log_initstr =
            UBL_OVERRIDE_ISTR("4,ubase:45,cbase:45,uconf:46,gptp:46,l2:46", "UBL_L2");
        unibase_init(&init_para);
    }

    if(set_options(&crfdata_td, argc, argv))
        goto end;

    if(!crfdata_td.direct){
        UB_LOG(UBL_INFO,"Waiting for avtpd to be ready...\n");
        WAIT_AVTPD_READY;
    }
    while(s_running && gptpmasterclock_init(crfdata_td.gshmem)){
        UB_LOG(UBL_INFO,"Waiting for tsn_gptpd to be ready...\n");
        CB_USLEEP(100000);
    }

    if(gptpmasterclock_init(crfdata_td.gshmem))
        goto end;

    res = start_crf(&crfdata_td);

end:
    gptpmasterclock_close();
    ubb_memory_out_close();
    unibase_close();

    return res;
}
void crf_tick(void *args)
{
    CB_SEM_POST(&gCRF_Tick);
}