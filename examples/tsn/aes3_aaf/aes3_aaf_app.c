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

#include "aes3_aaf_app.h"
#include "tsn_conl2/aaf_avtpc_listener.h"
#include "tsn_conl2/aaf_avtpc_talker.h"
#include "tsn_conl2/aes3_helper.h"
#include "sample_audio.h"
#include "tsn_gptp/gptpmasterclock.h"
#include "debug_log.h"
#include "shm_cirbuf.h"
#include <drivers/ipc_notify.h>

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */
#define SHM_AVB_DATA_RX_SIZE            (0x80000)
#define AAF_DOLBY_SYNC_FRAME_SIZE       (768)
#define SMPTE_HEADER_SIZE               (12)
#define SMPTE_FRAME_SIZE                (SMPTE_HEADER_SIZE + AAF_DOLBY_SYNC_FRAME_SIZE)

#if defined(SOC_AM62DX)
#define SHARED_MEMORY_START_ADDRESS     (0xA3000000)
#elif defined(SOC_AM275X)
/*
 * The SRAM address 0x72400000 is aliased to 0x80400000 in C7X Core,
 * Same reflects in the linker.cmd as well.
 */
#define SHARED_MEMORY_START_ADDRESS     (0x72400000)
#endif

typedef struct {
    void (*avtpc_close)(void *avtpc_talker);
    int (*avtpc_send)(void *avtpc_talker, uint64_t pts,
              uint8_t *audio_data, uint32_t data_size);
    void *avtpc_talker;
    int read_size;
    int sent_bytes;
    uint32_t sent_packets;
    aes3helper_talkerprocessor_t aes3talker_helper;
} audio_talker_t;

typedef struct {
    void (*avtpc_close)(void *avtpc_listener);
    void *avtpc_listener; /* aaf or iec61883_6 */
    shm_handle shmHandle;
    aes3helper_listenerprocessor_t aes3listener_helper;
} audio_listener_t;

audio_talker_t audio_talker;
audio_listener_t audio_listener;

typedef struct {
    audio_talker_t audio_talker;
    audio_listener_t audio_listener;
    conl2_basic_conparas_t bcp;
    int audio_samples; //number of audio samples per AVTP packet
    char* netdev;
} conl2_data_t;

/* ========================================================================== */
/*                              Global Variables                              */
/* ========================================================================== */

/*
 * AES3 dummy header
 * Application layer should self-prepare 12 bytes smpte337 preamble
 * Only Pd is input from file size
 */
static uint8_t smpte_337_preamble[12] =
{
    0x96, 0xF8, 0x72, // Pa. (bit from 27 -> 4)
    0xA5, 0x4E, 0x1F, // Pb. (bit from 27 -> 4)

    /*
     * Pc. First 0x00 data stream = 0, data_type_dependent = 0
     * Pc. Second byte 0x50 = 01010000b (mode = 2, 24 bit) and 10000b is 16 for Dolby EC3
     * Pc. Third 0x00 unused
     */
    0x00, 0x50, 0x00, // Pc. (bit from 27 -> 4)
    // Pd length code = EC3_SYNC_FRMSIZE * 2 * 8 = 0x001800 (will be set from outside)
    0x00, 0x00, 0x00, // Pd. (bit from 27 -> 4)
};

/*  Application layer should self-prepare 192 bits channel status This is just dummy data */
static uint8_t aes3_channel_status[24] =
{
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02
};

static int audio_aaf_avtp_push_packet(uint8_t *payload, int plsize, \
                                     avbtp_rcv_cb_info_t *cbinfo, void *cbdata);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static void set_smpte_preamble_word_length(uint32_t frmsiz)
{
    uint32_t w_length = frmsiz * 2 * 8;
    smpte_337_preamble[9] = (uint8_t)(w_length >> 16);
    smpte_337_preamble[10] = (uint8_t)(w_length >> 8);
    smpte_337_preamble[11] = (uint8_t)(w_length >> 0);
}

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

static int audio_aaf_talker_init(conl2_basic_conparas_t* basic_param, aaf_avtpc_aesinfo_t* aes3_org_info)
{
    audio_talker.avtpc_talker = aaf_avtpc_talker_init(basic_param);
    if (!audio_talker.avtpc_talker) {
        DPRINT("%s:failed to open aaf talker\n",__func__);
        return -1;
    }
    aaf_avtpc_set_aes_info(audio_talker.avtpc_talker, aes3_org_info);

    audio_talker.avtpc_close = audio_aaf_avtpc_talker_close;
    audio_talker.avtpc_send = audio_aaf_avtpc_talker_send;

    init_aes3talkerprocessor(&audio_talker.aes3talker_helper);
    set_smpte_preamble_word_length(EC3_SYNC_FRMSIZE);
    set_tx_smpte_preamle(&audio_talker.aes3talker_helper, &smpte_337_preamble[0]);
    set_tx_channelstatus(&audio_talker.aes3talker_helper, &aes3_channel_status[0]);

    return 0;
}

static int audio_aaf_listener_init(conl2_basic_conparas_t* basic_param, aaf_avtpc_aesinfo_t* aes3_org_info)
{
    audio_listener.avtpc_listener = aaf_avtpc_listener_init(basic_param, audio_aaf_avtp_push_packet, &audio_listener);
    if (!audio_listener.avtpc_listener) {
        DPRINT("%s:failed to open aaf listener\n",__func__);
        return -1;
    }
    audio_listener.avtpc_close=audio_aaf_avtpc_listener_close;
    init_aes3listenerprocessor(&audio_listener.aes3listener_helper);
    return 0;
}

static void init_default_params(conl2_basic_conparas_t* basic_param, aaf_avtpc_aesinfo_t* aes3_org_info)
{
    basic_param->vid = 110;
    basic_param->send_ahead_ts = 20000; // ahead 20ms
    ub_streamid_t streamid = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x00, 0x03};;
    memcpy(basic_param->streamid, streamid, sizeof(streamid));

    ub_macaddr_t mcastmac = {0x91, 0xE0, 0xF0, 0x00, 0xFE, 0x00};
    memcpy(basic_param->mcastmac, mcastmac, sizeof(mcastmac));

    basic_param->time_intv = 250;
    basic_param->max_frm_sz = 1500;
    basic_param->max_intv_frames = 1;
    basic_param->pcp = 2;
    basic_param->avtpd_bufftime_us = 10000;
    basic_param->is_direct = true;

    aes3_org_info->format = AVBTP_AAF_FORMAT_AES3_32BIT;
    aes3_org_info->aes3_dt_ref = DT_SMPTE338;
    aes3_org_info->aes3_data_type_h = 0x00;
    aes3_org_info->aes3_data_type_l = 0x10; /* 0x10 For Dolby EC3 Frame */
    aes3_org_info->frate = 48000;
    aes3_org_info->streams_per_frame = 1;
}

int start_aes3_aaf_talker(char* netdev)
{
    uint64_t pts;
    uint64_t sts, ets, tsdiff; // start and end ts, use to print report
    conl2_basic_conparas_t basic_param;
    aaf_avtpc_aesinfo_t aes3_org_info;
    uint64_t period_packet;

    memset(&basic_param, 0, sizeof(basic_param));
    memset(&aes3_org_info, 0, sizeof(aes3_org_info));
    memcpy(basic_param.netdev, netdev, strlen(netdev));
    init_default_params(&basic_param, &aes3_org_info);

    while(gptpmasterclock_init(NULL)){
        UB_LOG(UBL_INFO,"Waiting for tsn_gptpd to be ready...\n");
        CB_USLEEP(100000);
    }

    // https://datatracker.ietf.org/doc/html/rfc4598
    // Table 1.  Time duration of E-AC-3 frame (number of blocks vs. sampling rate)
    // +------------------+--------+-----------------+-----------------+
    // | blocks per frame | 32 kHz |        44.1 kHz |          48 kHz |
    // +------------------+--------+-----------------+-----------------+
    // |                1 |   8 ms |  approx. 5.8 ms |  approx. 5.3 ms |
    // |                2 |  16 ms | approx. 11.6 ms | approx. 10.7 ms |
    // |                3 |  24 ms | approx. 17.4 ms |           16 ms |
    // |                6 |  48 ms | approx. 34.8 ms |           32 ms |
    // +------------------+--------+-----------------+-----------------+
    period_packet = 32 * UB_MSEC_NS; // ns

    if (audio_aaf_talker_init(&basic_param, &aes3_org_info) == -1) return -1;

    int dolby_sync_frame_size_byte = EC3_SYNC_FRMSIZE * 2;
    int total_dolby_sync_frame = get_payload_sample_size() / dolby_sync_frame_size_byte;
    sts = ub_mt_gettime64();
    // repeat: start from 0 and read until last sync frame
    while(true)
    {
        for (int sync_frame=0; sync_frame<total_dolby_sync_frame; sync_frame++)
        {
            pts = gptpmasterclock_getts64();
            uint8_t* audio_buf = &payload_sample[sync_frame * dolby_sync_frame_size_byte];
            uint8_t* send_data = dolbysyncframe_to_aes3aaf(&audio_talker.aes3talker_helper, audio_buf, dolby_sync_frame_size_byte);

            int datasize = AES3_CARRY_SMPTE_DATA_SIZE;
            if (datasize < 1500) // no need fragments
            {
                if (audio_talker.avtpc_send( audio_talker.avtpc_talker,
                                        pts + basic_param.send_ahead_ts,
                                        &send_data[0],
                                        datasize) < 0)
                {
                    return -1;
                }
                audio_talker.sent_packets++;
                audio_talker.sent_bytes += datasize;
            }
            else // fragmented needed
            {
                int send_data_index = 0;
                int fragment_no =0;
                // currently fragment size is 1000/8=125 frames
                // In case of 6 channels aes3, data size = 1792+12=1804bytes audio+preamble
                // Convert to AES3 actual data size = ( 1804 / 6 ) * 8 = 2405.3333 bytes
                // => below loop will run 2405.3333/1000=3 times
                static const int aes3_fragment_data_size = 1000; // bytes
                while(send_data_index<datasize)
                {
                    if (audio_talker.avtpc_send(
                                        audio_talker.avtpc_talker,
                                        pts + basic_param.send_ahead_ts,
                                        &send_data[send_data_index],
                                        aes3_fragment_data_size) < 0) {
                        return -1;
                    }
                    send_data_index+=aes3_fragment_data_size;
                    fragment_no++;
                }
                (void)fragment_no;
                audio_talker.sent_packets+=fragment_no;
                audio_talker.sent_bytes += datasize;
            }

            ets = ub_mt_gettime64();
            tsdiff = ets - sts;
            if (tsdiff >= UB_SEC_NS) {
                UB_LOG(UBL_INFO,"audio talker sent: %f(packets/sec), %f(mbps)\n",
                    (float)audio_talker.sent_packets/((float)tsdiff/(float)UB_SEC_NS),
                    (float)(audio_talker.sent_bytes * 8000.0f)/(float)tsdiff);
                audio_talker.sent_packets = 0;
                audio_talker.sent_bytes = 0;

                sts = ets;
            }

            pts += period_packet;
            gptpmasterclock_wait_until_ts64(pts, 0, period_packet);

        }
    }

    return 0;
}

static int audio_aaf_avtp_push_packet(uint8_t *payload, int plsize,
                avbtp_rcv_cb_info_t *cbinfo, void *cbdata)
{
    audio_listener_t *audio_listener = (audio_listener_t *)cbdata;
    if (!audio_listener) {return -1;}
    // int64_t pts = -1;

    avbtp_sd_info_t *rsdinfo=&cbinfo->u.rsdinfo;
    aaf_avtpc_aesinfo_t aesinfo;
    if (rsdinfo->u.aaf.format != AVBTP_AAF_FORMAT_AES3_32BIT)
    {
        UB_LOG(UBL_INFO,"%s unexpected rx format %d\n", __func__, rsdinfo->u.aaf.format);
        return -1;
    }

    aaf_avtpc_get_aes3_info(&cbinfo->u.rsdinfo, &aesinfo);

    // if (!aaf_avtpc_listener_checkts(rsdinfo, NULL)){pts = rsdinfo->timestamp;}

    AES3_RX_RET depayRet = fill_aes3aaf_rx(&audio_listener->aes3listener_helper, payload, plsize);
    if (depayRet==RX_DONE)
    {
        smpte337_frame_t* fram337 = get_frame337rx(&audio_listener->aes3listener_helper);
        shm_write(audio_listener->shmHandle, fram337->buffer, SMPTE_FRAME_SIZE);
        // UB_LOG(UBL_INFO,"%s: pts:%" PRId64 ", frate: %d\n", __func__, pts, aesinfo.frate);
        UB_LOG(UBL_INFO,"[RX] frame337 wlen=%d bits\n", fram337->word_length);
        // ub_hexdump(true, false, &fram337->buffer[0], 12, 0);
        fram337->word_length = 0; // reset for next recv
    }

    return 0;
}

shm_handle aes3_aaf_init_shm(void* const address, const int blockSize, const int totalSize)
{
    const uint32_t shmOvrHd     = shm_metadata_overhead();

    /* floor the totalSize to be a multiple of blocksize, exclude the overhead size. */
    const uint32_t rxShmBufSize = ((totalSize-shmOvrHd)/blockSize)*blockSize + shmOvrHd;

    memset(address, 0, rxShmBufSize);
    shm_handle handle = shm_create((uint32_t)address, rxShmBufSize);

    ub_assert_fatal(handle != NULL, __func__, "SHM Creation failed!!");
    return handle;
}

int start_aes3_aaf_listener(char* netdev)
{
    conl2_basic_conparas_t basic_param;
    aaf_avtpc_aesinfo_t aes3_org_info;

    memset(&basic_param, 0, sizeof(basic_param));
    memset(&aes3_org_info, 0, sizeof(aes3_org_info));
    memcpy(basic_param.netdev, netdev, strlen(netdev));
    init_default_params(&basic_param, &aes3_org_info);

    while(gptpmasterclock_init(NULL)){
        UB_LOG(UBL_INFO,"Waiting for tsn_gptpd to be ready...\n");
        CB_USLEEP(100000);
    }

    audio_listener.shmHandle = aes3_aaf_init_shm((void*)SHARED_MEMORY_START_ADDRESS,
                    SMPTE_FRAME_SIZE, SHM_AVB_DATA_RX_SIZE);

    if (audio_listener.shmHandle == NULL)
    {
        UB_LOG(UBL_ERROR, "Shared Memory Creation Failed!!!\n");
        return -1;
    }

    if (audio_aaf_listener_init(&basic_param, &aes3_org_info) == -1) return -1;

    while(true)
    {
        CB_USLEEP(100000);
        ub_log_flush();
    }

    return 0;
}
