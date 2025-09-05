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
 * \file  enetapp_mcasp.c
 *
 * \brief This file contains the implementation of the Enet TSN example.
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */
#include <stdint.h>
#include <tsn_combase/combase.h>
#include <tsn_combase/combase_link.h>
#include <tsn_combase/tilld/cb_lld_ethernet.h>
#include "nrt_flow/dataflow.h"
#include "debug_log.h"
#include "tsninit.h"
#include "enetapp_cpsw.h"
#include <drivers/pinmux.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"


/* Audio buffer settings */
#define APP_MCASP_AUDIO_BUFF_COUNT  (4U)
#define APP_MCASP_AUDIO_BUFF_SIZE   (2048U)

/* ========================================================================== */
/*                           Global Variables                                 */
/* ========================================================================== */

/* Create buffers for transmit and Receive */
uint8_t gMcaspAudioBufferTx[APP_MCASP_AUDIO_BUFF_COUNT][APP_MCASP_AUDIO_BUFF_SIZE] __attribute__((aligned(256)));
uint8_t gMcaspAudioBufferRx[APP_MCASP_AUDIO_BUFF_COUNT][APP_MCASP_AUDIO_BUFF_SIZE] __attribute__((aligned(256)));

/* Create transaction objects for transmit and Receive */
MCASP_Transaction   gMcaspAudioTxnTx[APP_MCASP_AUDIO_BUFF_COUNT] = {0};
MCASP_Transaction   gMcaspAudioTxnRx[APP_MCASP_AUDIO_BUFF_COUNT] = {0};

extern MCASP_Config gMcaspConfig[];

#include "drivers/hw_include/cslr.h"
#include "board/ioexp/ioexp_tca6416.h"

static TCA6416_Config  gTCA6416_Config1;

int32_t configure_mcasp()
{
    int32_t     status = SystemP_SUCCESS;
    uint32_t    i;
    MCASP_Handle    mcaspHandle;

#if defined (SOC_AM275X)
    Pinmux_PerCfg_t i2cPinmuxConfig[] =
    {
        {
            PIN_GPIO1_72, /* AUDIO_EXT_REFCLK2 */
            ( PIN_MODE(1) | PIN_INPUT_ENABLE | PIN_PULL_DIRECTION  )
        },
        {PINMUX_END, 0U}
    };

    Pinmux_config(i2cPinmuxConfig, PINMUX_DOMAIN_ID_MAIN);
#endif

    uint32_t pinNum = 3;

    TCA6416_Params  tca6416Params;
    TCA6416_Params_init(&tca6416Params);
    tca6416Params.i2cInstance = CONFIG_I2C0;
    tca6416Params.i2cAddress = 0x20;
    TCA6416_open(&gTCA6416_Config1, &tca6416Params);

    status = TCA6416_setOutput(
                    &gTCA6416_Config1,
                    pinNum,
                    TCA6416_OUT_STATE_LOW);

    /* Configure as output  */
    status += TCA6416_config(
                    &gTCA6416_Config1,
                    pinNum,
                    TCA6416_MODE_OUTPUT);

    status = TCA6416_setOutput(
                    &gTCA6416_Config1,
                    pinNum,
                    TCA6416_OUT_STATE_HIGH);

    /* Open MCASP driver after enabling the HCLK */
    gMcaspHandle[0] = MCASP_open(0, &gMcaspOpenParams[0]);
    if(NULL == gMcaspHandle[0])
    {
        DebugP_logError("MCASP open failed for instance 0 !!!\r\n");
        DebugP_assert(false);
    }

    /* Open MCASP driver after enabling the HCLK */
    gMcaspHandle[1] = MCASP_open(1, &gMcaspOpenParams[1]);
    if(NULL == gMcaspHandle[1])
    {
        DebugP_logError("MCASP open failed for instance 1 !!!\r\n");
        DebugP_assert(false);
    }

    mcaspHandle = MCASP_getHandle(CONFIG_MCASP0);

    /* Prepare and submit audio transaction transmit objects */
    for (i = 0U; i < APP_MCASP_AUDIO_BUFF_COUNT; i++)
    {
        gMcaspAudioTxnTx[i].buf = (void*) &gMcaspAudioBufferTx[i][0];
        gMcaspAudioTxnTx[i].count = APP_MCASP_AUDIO_BUFF_SIZE/4;
        gMcaspAudioTxnTx[i].timeout = 0xFFFFFF;
        MCASP_submitTx(mcaspHandle, &gMcaspAudioTxnTx[i]);
    }

    /* Prepare and submit audio transaction receive objects */
    for (i = 0U; i < APP_MCASP_AUDIO_BUFF_COUNT; i++)
    {
        gMcaspAudioTxnRx[i].buf = (void*) &gMcaspAudioBufferRx[i][0];
        gMcaspAudioTxnRx[i].count = APP_MCASP_AUDIO_BUFF_SIZE/4;
        gMcaspAudioTxnRx[i].timeout = 0xFFFFFF;
        MCASP_submitRx(mcaspHandle, &gMcaspAudioTxnRx[i]);
    }

    /* Trigger McASP receive operation */
    status = MCASP_startTransferRx(mcaspHandle);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Trigger McASP transmit operation */
    status = MCASP_startTransferTx(mcaspHandle);
    DebugP_assert(status == SystemP_SUCCESS);

    MCASP_startTransferTx(gMcaspHandle[1]);
    MCASP_startTransferRx(gMcaspHandle[1]);

    return status;
}
void mcasp_txcb1(MCASP_Handle handle,
                          MCASP_Transaction *transaction)
{

}

void mcasp_txcb(MCASP_Handle handle,
                          MCASP_Transaction *transaction)
{
    MCASP_submitRx(handle, transaction);
}

void mcasp_rxcb(MCASP_Handle handle,
                          MCASP_Transaction *transaction)
{
    MCASP_submitTx(handle, transaction);
}