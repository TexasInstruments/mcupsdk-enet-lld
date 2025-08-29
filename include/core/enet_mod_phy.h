/*
 *  Copyright (c) Texas Instruments Incorporated 2020
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
 * \file  enet_mod_phy.h
 *
 * \brief This file contains the type definitions and helper macros for the
 *        Ethernet PHY interface.
 */

#ifndef ENET_MOD_PHY_H_
#define ENET_MOD_PHY_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <include/core/enet_types.h>
#include <include/core/enet_ioctl.h>
#include <include/phy/enetphy.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/*! \brief Helper macro to create PHY IOCTL commands. */
#define ENET_PHY_PUBLIC_IOCTL(x)              (ENET_IOCTL_TYPE_PUBLIC | \
                                               ENET_IOCTL_PHY_BASE |    \
                                               ENET_IOCTL_MIN(x))

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief PHY IOCTL commands.
 */
enum EnetPhy_Ioctl_e
{
    /*!
     * \brief Get PHY identification.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetPhy_GenericInArgs
     * - outArgs: #EnetPhy_Version
     */
    ENET_PHY_IOCTL_GET_ID = ENET_PHY_PUBLIC_IOCTL(0U),

    /*!
     * \brief Get PHY supported modes by local PHY device.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetPhy_GenericInArgs
     * - outArgs: uint32_t
     */
    ENET_PHY_IOCTL_GET_SUPPORTED_MODES = ENET_PHY_PUBLIC_IOCTL(1U),

    /*!
     * \brief Check if PHY is in loopback or not.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetPhy_GenericInArgs
     * - outArgs: bool
     */
    ENET_PHY_IOCTL_GET_LOOPBACK_STATE = ENET_PHY_PUBLIC_IOCTL(2U),

    /*!
     * \brief Check PHY alive status.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetPhy_GenericInArgs
     * - outArgs: bool
     */
    ENET_PHY_IOCTL_IS_ALIVE = ENET_PHY_PUBLIC_IOCTL(3U),

    /*!
     * \brief Check state-machine link state, that is, whether the state machine
     *        has reached the LINKED state.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetPhy_GenericInArgs
     * - outArgs: bool
     */
    ENET_PHY_IOCTL_IS_LINKED = ENET_PHY_PUBLIC_IOCTL(4U),

    /*!
     * \brief Get the link speed and duplexity state after the state machine
     *        has reached the LINKED state.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetPhy_GenericInArgs
     * - outArgs: #EnetMacPort_LinkCfg
     */
    ENET_PHY_IOCTL_GET_LINK_MODE = ENET_PHY_PUBLIC_IOCTL(5U),

    /*!
     * \brief Reset PHY.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetPhy_GenericInArgs
     * - outArgs: None
     */
    ENET_PHY_IOCTL_RESET = ENET_PHY_PUBLIC_IOCTL(6U),

    /*!
     * \brief Read PHY register.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetPhy_ReadRegInArgs
     * - outArgs: uint16_t
     */
    ENET_PHY_IOCTL_READ_REG = ENET_PHY_PUBLIC_IOCTL(7U),

    /*!
     * \brief Write PHY register.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetPhy_WriteRegInArgs
     * - outArgs: None
     */
    ENET_PHY_IOCTL_WRITE_REG = ENET_PHY_PUBLIC_IOCTL(8U),

    /*!
     * \brief Read PHY extended register.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetPhy_ReadRegInArgs
     * - outArgs: uint16_t
     */
    ENET_PHY_IOCTL_READ_EXT_REG = ENET_PHY_PUBLIC_IOCTL(9U),

    /*!
     * \brief Write PHY extended register.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetPhy_WriteRegInArgs
     * - outArgs: None
     */
    ENET_PHY_IOCTL_WRITE_EXT_REG = ENET_PHY_PUBLIC_IOCTL(10U),

    /*!
     * \brief Read PHY register using Clause-45 frame.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetPhy_C45ReadRegInArgs
     * - outArgs: uint16_t
     */
    ENET_PHY_IOCTL_C45_READ_REG = ENET_PHY_PUBLIC_IOCTL(11U),

    /*!
     * \brief Write PHY register using Clause-45 frame.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetPhy_C45WriteRegInArgs
     * - outArgs: None
     */
    ENET_PHY_IOCTL_C45_WRITE_REG = ENET_PHY_PUBLIC_IOCTL(12U),

    /*!
     * \brief Print PHY registers.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetPhy_GenericInArgs
     * - outArgs: None
     */
    ENET_PHY_IOCTL_PRINT_REGS = ENET_PHY_PUBLIC_IOCTL(13U),

    /*!
     * \brief Adjust PHY PTP clock Frequency.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetPhy_AdjPtpFreqInArgs
     * - outArgs: None
     */
    ENET_PHY_IOCTL_ADJ_PTP_FREQ = ENET_PHY_PUBLIC_IOCTL(14U),

    /*!
     * \brief Adjust PHY PTP clock Phase.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetPhy_AdjPtpPhaseInArgs
     * - outArgs: None
     */
    ENET_PHY_IOCTL_ADJ_PTP_PHASE = ENET_PHY_PUBLIC_IOCTL(15U),

    /*!
     * \brief Get current PHY PTP clock time.
     *
     * IOCTL parameters:
     * -  inArgs: EnetPhy_GenericInArgs
     * - outArgs: uint64_t
     */
    ENET_PHY_IOCTL_GET_PTP_TIME = ENET_PHY_PUBLIC_IOCTL(16U),

    /*!
     * \brief Set PHY PTP clock time.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetPhy_SetPtpTimeInArgs
     * - outArgs: None
     */
    ENET_PHY_IOCTL_SET_PTP_TIME = ENET_PHY_PUBLIC_IOCTL(17U),

    /*!
     * \brief Get PHY PTP TX packet timestamp.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetPhy_PtpPktTimestampInArgs
     * - outArgs: uint64_t
     */
    ENET_PHY_IOCTL_GET_PTP_TXTS = ENET_PHY_PUBLIC_IOCTL(18U),

    /*!
     * \brief Get PHY PTP RX packet timestamp.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetPhy_PtpPktTimestampInArgs
     * - outArgs: uint64_t
     */
    ENET_PHY_IOCTL_GET_PTP_RXTS = ENET_PHY_PUBLIC_IOCTL(19U),

    /*!
     * \brief Add PHY PTP TX packet info to a waiting TX timestamp list.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetPhy_PtpPktTimestampInArgs
     * - outArgs: None
     */
    ENET_PHY_IOCTL_WAIT_PTP_TXTS = ENET_PHY_PUBLIC_IOCTL(20U),

    /*!
     * \brief Process PHY status frame.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetPhy_ProcStatusFrameInArgs
     * - outArgs: #EnetPhy_ProcStatusFrameOutArgs
     */
    ENET_PHY_IOCTL_PROC_STATUS_FRAME = ENET_PHY_PUBLIC_IOCTL(21U),

    /*!
     * \brief Get PHY status frame header.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetPhy_GenericInArgs
     * - outArgs: #EnetPhy_GetStatusFrameEthdrOutArgs
     */
    ENET_PHY_IOCTL_GET_STATUS_FRAME_ETHDR = ENET_PHY_PUBLIC_IOCTL(22U),

    /*!
     * \brief Enable/Disable PHY PTP module.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetPhy_EnablePtpInArgs
     * - outArgs: None
     */
    ENET_PHY_IOCTL_ENABLE_PTP = ENET_PHY_PUBLIC_IOCTL(23U),

    /*!
     * \brief Enable/Disable an event capture on a PHY GPIO pin.
     *
     * IOCTL parameters:
     * -  inArgs: EnetPhy_EnableEventCaptureInArgs
     * - outArgs: None
     */
    ENET_PHY_IOCTL_ENABLE_EVENT_CAPTURE = ENET_PHY_PUBLIC_IOCTL(24U),

    /*!
     * \brief Enable/Disable clock trigger on a GPIO pin.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetPhy_EnableTriggerOutputInArgs
     * - outArgs: None
     */
    ENET_PHY_IOCTL_ENABLE_TRIGGER_OUTPUT = ENET_PHY_PUBLIC_IOCTL(25U),

    /*!
     * \brief Get event timestamp.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetPhy_GenericInArgs
     * - outArgs: #EnetPhy_GetEventTimestampOutArgs
     */
    ENET_PHY_IOCTL_GET_EVENT_TIMESTAMP = ENET_PHY_PUBLIC_IOCTL(26U),

    /*!
     * \brief Get Phy Link status. This IOTCL will not have longterm support and this functionality
     *  will eventually be merged to ENET_PHY_IOCTL_GET_LINK_MODE IOCTL in upcoming release.
     *  So ENET_PHY_IOCTL_GET_LINK_MODE IOCTL can be continued to use to get the phy link speed, duplexity
     *
     * IOCTL parameters:
     * -  inArgs: #EnetPhy_GenericInArgs
     * -  outArgs: #EnetMacPort_LinkCfg
     */
    ENET_PHY_IOCTL_GET_LINK_STATUS = ENET_PHY_PUBLIC_IOCTL(27U),

        /*!
     * \brief Config Media Clock
     *
     * IOCTL parameters:
     * -  inArgs: #EnetPhy_ConfigMediaClockInArgs
     * - outArgs: None
     */
    ENET_PHY_IOCTL_CONFIG_MEDIA_CLOCK = ENET_PHY_PUBLIC_IOCTL(28U),

    /*!
     * \brief Nudge Codec Clock
     *
     * IOCTL parameters:
     * -  inArgs: #EnetPhy_NudgeCodecClockInArgs
     * - outArgs: None
     */
    ENET_PHY_IOCTL_NUDGE_CODEC_CLOCK = ENET_PHY_PUBLIC_IOCTL(29U),
};

/*!
 * \brief Generic input args.
 */
typedef struct EnetPhy_GenericInArgs_s
{
    /*! Port number */
    Enet_MacPort macPort;
} EnetPhy_GenericInArgs;

/*!
 * \brief Input args for #ENET_PHY_IOCTL_READ_EXT_REG command.
 */
typedef struct EnetPhy_ReadRegInArgs_s
{
    /*! Port number */
    Enet_MacPort macPort;

    /*! Register address */
    uint16_t reg;
} EnetPhy_ReadRegInArgs;

/*!
 * \brief Input args for #ENET_PHY_IOCTL_WRITE_EXT_REG command.
 */
typedef struct EnetPhy_WriteRegInArgs_s
{
    /*! Port number */
    Enet_MacPort macPort;

    /*! Register address */
    uint16_t reg;

    /*! Value to be written */
    uint16_t val;
} EnetPhy_WriteRegInArgs;

/*!
 * \brief Input args for #ENET_PHY_IOCTL_C45_READ_REG command.
 */
typedef struct EnetPhy_C45ReadRegInArgs_s
{
    /*! Port number */
    Enet_MacPort macPort;

    /*! MMD */
    uint8_t mmd;

    /*! Register address */
    uint16_t reg;
} EnetPhy_C45ReadRegInArgs;

/*!
 * \brief Input args for #ENET_PHY_IOCTL_C45_WRITE_REG command.
 */
typedef struct EnetPhy_C45WriteRegInArgs_s
{
    /*! Port number */
    Enet_MacPort macPort;

    /*! MMD */
    uint8_t mmd;

    /*! Register address */
    uint16_t reg;

    /*! Value to be written */
    uint16_t val;
} EnetPhy_C45WriteRegInArgs;

/*!
 * \brief Input args for #ENET_PHY_IOCTL_GET_PTP_TXTS command.
 */
typedef struct EnetPhy_PtpPktTimestampInArgs_s
{
    Enet_MacPort macPort;
    uint32_t domain;
    uint32_t msgType;
    uint32_t seqId;
} EnetPhy_PtpPktTimestampInArgs;

/*!
 * \brief Input args for #ENET_PHY_IOCTL_ADJ_PTP_FREQ command.
 */
typedef struct EnetPhy_AdjPtpFreqInArgs_s
{
    Enet_MacPort macPort;
    int64_t ppb;
} EnetPhy_AdjPtpFreqInArgs;

/*!
 * \brief Input args for #ENET_PHY_IOCTL_ADJ_PTP_PHASE command.
 */
typedef struct EnetPhy_AdjPtpPhaseInArgs_s
{
    Enet_MacPort macPort;
    int64_t offset;
} EnetPhy_AdjPtpPhaseInArgs;

/*!
 * \brief Input args for #ENET_PHY_IOCTL_SET_PTP_TIME command.
 */
typedef struct EnetPhy_SetPtpTimeInArgs_s
{
    Enet_MacPort macPort;
    uint64_t ts64;
} EnetPhy_SetPtpTimeInArgs;

/*!
 * \brief Input args for #ENET_PHY_IOCTL_PROC_STATUS_FRAME command.
 */
typedef struct EnetPhy_ProcStatusFrameInArgs_s
{
    Enet_MacPort macPort;
    uint8_t *frame;
    uint32_t size;
} EnetPhy_ProcStatusFrameInArgs;

/*!
 * \brief Output args for #ENET_PHY_IOCTL_PROC_STATUS_FRAME command.
 */
typedef struct EnetPhy_ProcStatusFrameOutArgs_s
{
    /* combination of any #ENETPHY_STATUS_FRAME_TYPE_x */
    uint32_t types;
} EnetPhy_ProcStatusFrameOutArgs;

/*!
 * \brief Input args for #ENET_PHY_IOCTL_GET_STATUS_FRAME_ETHDR command.
 */
typedef struct EnetPhy_GetStatusFrameEthdrOutArgs_s
{
    uint8_t *ethhdr;
    uint32_t size;
} EnetPhy_GetStatusFrameEthdrOutArgs;

/*!
 * \brief Input args for #ENET_PHY_IOCTL_ENABLE_PTP command.
 */
typedef struct EnetPhy_EnablePtpInArgs_s
{
    Enet_MacPort macPort;
    bool on;
    /* This PHY-specific parameter is used to distinguish status frames
     * among multiple PHYs connected to the same SoC.
     * Please check the PHY datasheet for supported value. */
    uint32_t srcMacStatusFrameType;
} EnetPhy_EnablePtpInArgs;

/*!
 * \brief Input args for #ENET_PHY_IOCTL_ENABLE_EVENT_CAPTURE command.
 */
typedef struct EnetPhy_EnableEventCaptureInArgs_s
{
    Enet_MacPort macPort;
    uint32_t eventIdx;
    bool falling;
    bool on;
} EnetPhy_EnableEventCaptureInArgs;

/*!
 * \brief Input args for #ENET_PHY_IOCTL_ENABLE_TRIGGER_OUTPUT command.
 */
typedef struct EnetPhy_EnableTriggerOutputInArgs_s
{
    Enet_MacPort macPort;
    uint32_t triggerIdx;
    uint64_t startNsec;
    uint64_t periodNsec;
    bool repeat;
} EnetPhy_EnableTriggerOutputInArgs;

/*!
 * \brief Output args for #ENET_PHY_IOCTL_GET_EVENT_TIMESTAMP command.
 */
typedef struct EnetPhy_GetEventTimestampOutArgs_s
{
    uint32_t eventIdx;
    uint32_t seqId;
    uint64_t ts64;
} EnetPhy_GetEventTimestampOutArgs;
/*!
 * \brief Input args for #ENET_PHY_IOCTL_CONFIG_MEDIA_CLOCK command.
 */
typedef struct EnetPhy_ConfigMediaClockInArgs_s
{
    Enet_MacPort macPort;
    bool isMaster;
    uint8_t *streamIDMatchValue;
    bool enTrigOut;
}EnetPhy_ConfigMediaClockInArgs;

/*!
 * \brief Input args for #ENET_PHY_IOCTL_NUDGE_CODEC_CLOCK command.
 */
typedef struct EnetPhy_NudgeCodecClockInArgs_s
{
    Enet_MacPort macPort;
    int8_t nudgeValue;
}EnetPhy_NudgeCodecClockInArgs;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                        Deprecated Function Declarations                    */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* ENET_MOD_PHY_H_ */
