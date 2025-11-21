/*
 *  Copyright (C) 2025 Texas Instruments Incorporated
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
/*                             Include Files                                  */
/* ========================================================================== */
#include <kernel/dpl/SemaphoreP.h>
#include "ti_dpl_config.h"
#include "tsn_conl2/aaf_avtpc_listener.h"
#include "tsn_conl2/aaf_avtpc_talker.h"
#include "../common_files/shm_cirbuf.h"
#include "ti_drivers_open_close.h"
#include "../common_files/gpio_sm.h"
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define R5F_AVB_SHM_START_ADDRESS    (0x72400000U)

#define AVB_C7X_TO_R5_SHM_ADDR       (R5F_AVB_SHM_START_ADDRESS+0U)
#define AVB_R5_TO_C7X_SHM_ADDR       (R5F_AVB_SHM_START_ADDRESS+100*1024U)

/* This is shared Memory Block size,
   should be an LCM Of McASP buffer size and AVTPDU Size. */
#define APP_MCASP_SHM_BLOCK_SIZE     (1536U)
#define APP_MCASP_SHM_NUM_BLOCKS     (1U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

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

typedef struct
{
    avbtp_rcv_cb_t avtp_stream_cb;
    void *arg;
}rxStream_cb;

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

extern SemaphoreP_Object gStartAVBTxSem;

/*
*   Note: According to the IEEE1722 and IEEE802.1Q,
*         the first 6 bytes of stream ID must have the source MAC Address.
*         This rule is not being followed in this Demo to simplify the application
*         implemenation and the performance is not affected because of this change.
*/
const ub_streamid_t CONST_PART_SID = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x00, 0x00};

static uint32_t gTickCounter = 0;

static const avtp_pcm_conf gTxStreamConf = {
     .netdev          = "tilld0",
     .streamID        = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x00, 0x01},
     .DestmacAddr     = {0x91, 0xE0, 0xF0, 0x00, 0xFE, 0x01},
     .vlanID          = 110,
     .priority        = 3,
     .timeInterval_us = 125,
     .pcminfo = {.format = AVBTP_AAF_FORMAT_INT_32BIT,
                 .channels = 8, .bit_depth = 32, .srate  = 48000},
};

static const avtp_pcm_conf gRxStreamConf = {
     .netdev          = "tilld0",
     .streamID        = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x00, 0x02},
     .DestmacAddr     = {0x91, 0xE0, 0xF0, 0x00, 0xFE, 0x01},
     .vlanID          = 110,
     .priority        = 3,
     .timeInterval_us = 125,
     .pcminfo = {.format = AVBTP_AAF_FORMAT_INT_32BIT,
                 .channels = 8, .bit_depth = 32, .srate  = 48000},
};

static SemaphoreP_Object gAvbTickSem;

static gpioStateMachine gListenerGpioSm = {
    .isEnabled = true,
    .isRxMode = false,
    .Periodicity = 3000,
    .pulseWidth = 30,
    .counter    = 0,
};

static gpioStateMachine gTalkerGpioSm = {
    .isEnabled = true,
    .isRxMode = false,
    .Periodicity = 3000,
    .pulseWidth = 30,
    .counter    = 0,
};

static bool gSemInit = false;

static uint8_t gTxCopyBuffer[APP_MCASP_SHM_BLOCK_SIZE];

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

int crfApp_rxCallback(uint8_t *payload, int payload_size, avbtp_rcv_cb_info_t *cbinfo, void *cbdata);
static aaf_avtpc_listener_data_t* init_aaf_pcm_listener(void* cbArgs);
static int autoamp_avtpRxPacketCallback(uint8_t *payload, int plsize, avbtp_rcv_cb_info_t *cbinfo, void *cbdata);
static shm_handle central_createSharedMemory(void* const address, const int blockSize, const int totalSize);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
static void init_conl2_params(const avtp_pcm_conf* conf, conl2_basic_conparas_t *bcp)
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

static void init_pcm_params(const avtp_pcm_conf* conf, aaf_avtpc_pcminfo_t* pcm_info)
{
    memcpy(pcm_info, &conf->pcminfo, sizeof(aaf_avtpc_pcminfo_t));
}

static aaf_avtpc_talker_data_t *autoAmpDemo_initPcmTalker(const avtp_pcm_conf* conf)
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

void EnetApp_aafAudioTask(void *args)
{
    /* Initialize the AVB Tick Semaphore. */
    SemaphoreP_constructBinary(&gAvbTickSem, 0);

    gSemInit = true;

    shm_handle shmRxHandle = central_createSharedMemory((void*)AVB_C7X_TO_R5_SHM_ADDR, APP_MCASP_SHM_BLOCK_SIZE, \
                             APP_MCASP_SHM_BLOCK_SIZE*(APP_MCASP_SHM_NUM_BLOCKS+1));
    shm_handle shmTxHandle = central_createSharedMemory((void*)AVB_R5_TO_C7X_SHM_ADDR, APP_MCASP_SHM_BLOCK_SIZE, \
                             APP_MCASP_SHM_BLOCK_SIZE*(APP_MCASP_SHM_NUM_BLOCKS+1));

    gpio_sm_init(&gListenerGpioSm, CONFIG_GPIO_LISTENER_BASE_ADDR, CONFIG_GPIO_LISTENER_PIN, CONFIG_GPIO_LISTENER_DIR);

    gpio_sm_init(&gTalkerGpioSm, CONFIG_GPIO_TALKER_BASE_ADDR, CONFIG_GPIO_TALKER_PIN, CONFIG_GPIO_TALKER_DIR);

    aaf_avtpc_talker_data_t* talker = autoAmpDemo_initPcmTalker(&gTxStreamConf);

    (void)init_aaf_pcm_listener((void*)shmTxHandle);

    const uint32_t pduSize = (gTxStreamConf.pcminfo.bit_depth/8)*(gTxStreamConf.pcminfo.channels)* \
                             ((gTxStreamConf.pcminfo.srate)*gTxStreamConf.timeInterval_us)/UB_SEC_US;

    const uint32_t samplesPerPdu = pduSize/((gTxStreamConf.pcminfo.bit_depth/8)*(gTxStreamConf.pcminfo.channels));

    int64_t gptpTime = 0;

    SemaphoreP_pend(&gStartAVBTxSem, SystemP_WAIT_FOREVER);

    while (1)
    {
        SemaphoreP_pend(&gAvbTickSem, SystemP_WAIT_FOREVER);
        if (gTickCounter == 0)
        {
            TimerP_stop(gTimerBaseAddr[CONFIG_TIMER0]);
        }
        else
        {
            gTickCounter--;
        }
        uint16_t reqSize = pduSize;

        /* Read from Shared Memory. */
        shm_read(shm_core_r5f, shmRxHandle, gTxCopyBuffer, &reqSize);

        if (reqSize != 0)
        {
            gpio_sm_spin(&gTalkerGpioSm, (uint32_t*)gTxCopyBuffer, samplesPerPdu);
            aaf_avtpc_talker_write(talker, gptpTime , gTxCopyBuffer, (int)reqSize);
        }
    }
}

static int autoamp_avtpRxPacketCallback(uint8_t *payload, int plsize,
                                avbtp_rcv_cb_info_t *cbinfo, void *cbdata)
{
    int status = SystemP_SUCCESS;

    /* If the Packet type is CRF, Call the CRF Callback function. */
    if(cbinfo->u.rcrfinfo.subtype == AVBTP_SUBTYPE_CRF)
    {
        crfApp_rxCallback(payload, plsize, cbinfo, NULL);
    }
    else
    {
        shm_handle shmTxHandle = (shm_handle)cbdata;

        const uint32_t expectedPduSize = (gRxStreamConf.pcminfo.bit_depth/8)*gRxStreamConf.pcminfo.channels*\
                                (gRxStreamConf.pcminfo.srate*gRxStreamConf.timeInterval_us)/UB_SEC_US;
        if ((uint32_t)plsize != expectedPduSize)
        {
            UB_LOG(UBL_INFO, "Not Matching the expected Payload Size, %d\r\n", plsize);
        }
        else
        {
            gpio_sm_spin(&gListenerGpioSm, (uint32_t*)payload, 6);
            /* Process the Payload, i.e. Send to C7x/McASP. */
            shm_write(shm_core_r5f, shmTxHandle, payload, plsize);
        }
    }
    return status;
}

static aaf_avtpc_listener_data_t* init_aaf_pcm_listener(void* cbArgs)
{
    /* Configure the RX Stream. */
    const avtp_pcm_conf *conf = &gRxStreamConf;

    conl2_basic_conparas_t conl2Params;
    init_conl2_params(conf, &conl2Params);

    aaf_avtpc_listener_data_t* listener = \
        aaf_avtpc_listener_init(&conl2Params, autoamp_avtpRxPacketCallback, cbArgs);

    return listener;
}

static shm_handle central_createSharedMemory(void* const address, const int blockSize, const int totalSize)
{
    shm_handle handle;
    const uint32_t shmOvrHd = shm_metadata_overhead();
    /* floor the totalSize to be a multiple of blocksize, exclude the overhead size. */
    const uint32_t rxShmBufSize = ((totalSize-shmOvrHd)/blockSize)*blockSize + shmOvrHd;
    handle = shm_create(shm_core_r5f, (uint32_t)address, rxShmBufSize);
    return handle;
}

/* Timer callback registed in syscfg. */
void avbTickTimer_callback(void *args)
{
    if (gSemInit)
    SemaphoreP_post(&gAvbTickSem);
}

void EnetApp_ipcNotifyCallback(uint16_t remoteCoreId, uint16_t localClientId, uint32_t msgValue, void *args)
{
    /* Start Timer with counter value = 7 */
    TimerP_start(gTimerBaseAddr[CONFIG_TIMER0]);
    gTickCounter = 7;
    avbTickTimer_callback(NULL);
}

