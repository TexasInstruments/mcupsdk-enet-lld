/*
 *  Copyright (c) Texas Instruments Incorporated 2024-2025
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
 * \file   media_clock_ctl.c
 *
 * \brief This file contains all functions related Media Clock Control Functions.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "media_clock_ctrl.h"
#include <enet_apputils.h>
#include "dp83tg721.h"
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define PHY_TRIGGER_INDEX     0
#define PHY_EVENT_INDEX       0
#define MCC_MAGIC             (0x12341234)
/* ========================================================================== */
/*                              Global Variables                              */
/* ========================================================================== */
MCC_Obj gMCCObject;
/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */

/* ========================================================================== */
/*                           Function Definitions                             */
/* ========================================================================== */

/* Initialize the MCC object. */
MCC_handle MCC_init(uint32_t mcFrequency, uint32_t timestampInterval)
{
    /* MCC Init function*/
    MCC_handle handle = &gMCCObject;

    if (handle->magic != MCC_MAGIC)
    {
        handle->magic = MCC_MAGIC;
        handle->MediaClockFrequency = mcFrequency;
        handle->timestampInterval = timestampInterval;

        Enet_Type enetType;
        uint32_t instId;
        EnetApp_getEnetInstInfo(CONFIG_ENET_CPSW0, &enetType, &instId);

        handle->hEnet = Enet_getHandle(enetType, instId);
        handle->coreId = EnetSoc_getCoreId();
        EnetAppUtils_assert(handle->hEnet != NULL);
        return handle;
    }
    else
    {
        return NULL;
    }
}

int32_t MCC_registerEvent(MCC_handle handle, uint32_t eventNum)
{
    int32_t status;
    Enet_IoctlPrms prms;
    EnetPhy_EnableEventCaptureInArgs inArgs;

    inArgs.macPort  = ENET_MAC_PORT_1;
    inArgs.eventIdx = eventNum;
    inArgs.falling  = false;
    inArgs.on       = true;

    ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);
    ENET_IOCTL(handle->hEnet, handle->coreId,
            ENET_PHY_IOCTL_ENABLE_EVENT_CAPTURE, &prms, status);

    if (status != ENET_SOK)
    {
        DebugP_logError("Failed to register event: %d\n", status);
    }

    return status;
}

int32_t MCC_getTimestamp(MCC_handle handle, uint32_t eventNum, uint64_t* timestamp)
{
    int32_t status = ENET_EFAIL;
    EnetPhy_GetEventTimestampOutArgs outArgs;
    do
    {
        Enet_IoctlPrms prms;
        EnetPhy_GenericInArgs inArgs;

        inArgs.macPort = ENET_MAC_PORT_1;

        ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs, &outArgs);
        ENET_IOCTL(handle->hEnet, handle->coreId,
                ENET_PHY_IOCTL_GET_EVENT_TIMESTAMP, &prms, status);

        if (status != ENET_SOK)
        {
            DebugP_logError("Failed to get timestamp: %d\n", status);
            break;
        }
    }
    while (outArgs.eventIdx != eventNum);

    if (status == ENET_SOK)
    {
        *timestamp = outArgs.ts64;
    }

    return status;
}

int32_t MCC_configMediaClock(MCC_handle handle, bool isMaster, uint8_t streamID[8])
{
    int32_t status = ENET_EFAIL;
    Enet_IoctlPrms prms;
    EnetPhy_ConfigMediaClockInArgs inArgs = {
        .macPort            = ENET_MAC_PORT_1,
        .isMaster           = isMaster,
        .enTrigOut          = true,
    };
    inArgs.streamIDMatchValue = (uint8_t*)streamID;

    ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);
    ENET_IOCTL(handle->hEnet, handle->coreId,
            ENET_PHY_IOCTL_CONFIG_MEDIA_CLOCK, &prms, status);

    return status;
}

int32_t MCC_enableEventCapture(MCC_handle handle, int eventIndex)
{
    int32_t status = ENET_EFAIL;
    Enet_IoctlPrms prms;
    EnetPhy_EnableEventCaptureInArgs inArgs = {
        .macPort  = ENET_MAC_PORT_1,
        .eventIdx = eventIndex,
        .falling  = false,
        .on       = true,
    };
    ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);
    ENET_IOCTL(handle->hEnet, handle->coreId,
            ENET_PHY_IOCTL_ENABLE_EVENT_CAPTURE, &prms, status);

    return status;
}

int32_t MCC_NudgeClock(MCC_handle handle, int8_t cycles)
{
    int32_t status = ENET_EFAIL;
    EnetPhy_NudgeCodecClockInArgs inArgs = {
        .macPort = ENET_MAC_PORT_1,
        .nudgeValue = cycles,
    };
    Enet_IoctlPrms prms;
    ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);
    ENET_IOCTL(handle->hEnet, handle->coreId,
            ENET_PHY_IOCTL_NUDGE_CODEC_CLOCK, &prms, status);
    return status;
}
