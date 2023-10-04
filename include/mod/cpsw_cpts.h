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
 * \file  cpsw_cpts.h
 *
 * \brief This file contains private type definitions and helper macros for the
 *        CPSW CPTS module.
 */

/*!
 * \ingroup  ENET_MOD_TIMESYNC
 * \defgroup CPSW_CPTS_MOD CPSW CPTS
 *
 * The CPSW CPTS module provides additional IOCTL commands than those supported
 * by the generic \ref ENET_MOD_TIMESYNC API set.
 *
 * CPSW CPTS clocks:
 *
 * Interrupts:
 * - CPSW_EVNT_PEND - Event pending interrupt.
 *
 * SoC parameters:
 * - CpswCpts_Obj::hwPushCnt - Maximum hardware push instances in hardware.
 *
 * Errata:
 * - CPSW_CPTS_ERRATA_GENFN_RECONFIG - GENFn/ESTFn reconfiguration bug.
 *
 * Compile-time configuration:
 * - #ENET_CFG_CPSW_CPTS_EVENTS_POOL_SIZE - Maximum size of CPTS event pool.
 * - #ENET_CFG_CPSW_CPTS_STATS - Event counter (software) statistics.  CPTS
 *   statistics can be printed via #ENET_TIMESYNC_IOCTL_PRINT_STATS.
 *
 * @{
 */

#ifndef CPSW_CPTS_H_
#define CPSW_CPTS_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <include/core/enet_mod_timesync.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/*! \brief Helper macro to create IOCTL commands for CPSW CPTS module. */
#define CPSW_CPTS_PUBLIC_IOCTL(x)             (ENET_IOCTL_TYPE_PUBLIC |   \
                                               ENET_IOCTL_TIMESYNC_BASE | \
                                               ENET_IOCTL_PER_CPSW |      \
                                               ENET_IOCTL_MIN(x))

/*! \brief Minimum nudge value. */
#define CPSW_CPTS_NUDGE_MIN_VAL           (-128)

/*! \brief Maximum nudge value. */
#define CPSW_CPTS_NUDGE_MAX_VAL           (127)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief CPSW CPTS IOCTL commands.
 */
typedef enum CpswCpts_Ioctl_s
{
    /*!
     * \brief CPTS register stack.
     *
     * IOCTL parameters:
     * -  inArgs: #CpswCpts_RegisterStackInArgs
     * - outArgs: None
     */
    CPSW_CPTS_IOCTL_REGISTER_STACK = CPSW_CPTS_PUBLIC_IOCTL(0U),

    /*!
     * \brief CPTS unregister stack.
     *
     * IOCTL parameters:
     * -  inArgs: None
     * - outArgs: None
     */
    CPSW_CPTS_IOCTL_UNREGISTER_STACK = CPSW_CPTS_PUBLIC_IOCTL(1U),

    /*!
     * \brief CPTS register hardware push event callback.
     *
     * IOCTL parameters:
     * -  inArgs: #CpswCpts_RegisterHwPushCbInArgs
     * - outArgs: None
     */
    CPSW_CPTS_IOCTL_REGISTER_HWPUSH_CALLBACK = CPSW_CPTS_PUBLIC_IOCTL(2U),

    /*!
     * \brief CPTS unregister hardware push event callback.
     *
     * IOCTL parameters:
     * -  inArgs: #CpswCpts_HwPush
     * - outArgs: None
     */
    CPSW_CPTS_IOCTL_UNREGISTER_HWPUSH_CALLBACK = CPSW_CPTS_PUBLIC_IOCTL(3U),

    /*!
     * \brief CPTS set timestamp nudge.
     *
     * Sets time nudge value.  Nudge value must be within the range of
     * #CPSW_CPTS_NUDGE_MIN_VAL and #CPSW_CPTS_NUDGE_MAX_VAL.
     *
     * IOCTL parameters:
     * -  inArgs: int32_t
     * - outArgs: None
     */
    CPSW_CPTS_IOCTL_SET_TS_NUDGE = CPSW_CPTS_PUBLIC_IOCTL(4U),

    /*!
     * \brief CPTS set compare value.
     *
     * IOCTL parameters:
     * -  inArgs: #CpswCpts_SetCompValInArgs
     * - outArgs: None
     */
    CPSW_CPTS_IOCTL_SET_COMP = CPSW_CPTS_PUBLIC_IOCTL(5U),

    /*!
     * \brief CPTS set compare nudge.
     *
     * IOCTL parameters:
     * -  inArgs: int32_t
     * - outArgs: None
     */
    CPSW_CPTS_IOCTL_SET_COMP_NUDGE = CPSW_CPTS_PUBLIC_IOCTL(6U),

    /*!
     * \brief CPTS set function generator.
     *
     * IOCTL parameters:
     * -  inArgs: #CpswCpts_SetFxnGenInArgs
     * - outArgs: None
     */
    CPSW_CPTS_IOCTL_SET_GENF = CPSW_CPTS_PUBLIC_IOCTL(7U),

    /*!
     * \brief CPTS set GenFn stamp nudge
     *
     * IOCTL parameters:
     * -  inArgs: #CpswCpts_SetFxnGenNudgeInArgs
     * - outArgs: None
     */
    CPSW_CPTS_IOCTL_SET_GENF_NUDGE = CPSW_CPTS_PUBLIC_IOCTL(8U),

    /*!
     * \brief CPTS set EST function generator.
     *
     * IOCTL parameters:
     * -  inArgs: #CpswCpts_SetFxnGenInArgs
     * - outArgs: None
     */
    CPSW_CPTS_IOCTL_SET_ESTF = CPSW_CPTS_PUBLIC_IOCTL(9U),

    /*!
     * \brief CPTS set ESTFn stamp nudge.
     *
     * IOCTL parameters:
     * -  inArgs: #CpswCpts_SetFxnGenNudgeInArgs
     * - outArgs: None
     */
    CPSW_CPTS_IOCTL_SET_ESTF_NUDGE = CPSW_CPTS_PUBLIC_IOCTL(10U),

    /*!
     * \brief CPTS TS output bit select.
     *
     * IOCTL parameters:
     * -  inArgs: #CpswCpts_OutputBitSel
     * - outArgs: None
     */
    CPSW_CPTS_IOCTL_SELECT_TS_OUTPUT_BIT = CPSW_CPTS_PUBLIC_IOCTL(11U),

    /*!
     * \brief CPTS lookup event.
     *
     * IOCTL parameters:
     * -  inArgs: #CpswCpts_Event
     * - outArgs: #CpswCpts_Event
     */
    CPSW_CPTS_IOCTL_LOOKUP_EVENT = CPSW_CPTS_PUBLIC_IOCTL(12U),

    /*!
     * \brief CPTS lookup EST event/.
     *
     * IOCTL parameters:
     * -  inArgs: #CpswCpts_EstEventMatchParams
     * - outArgs: #CpswCpts_EstEvent
     */
    CPSW_CPTS_IOCTL_LOOKUP_EST_EVENT = CPSW_CPTS_PUBLIC_IOCTL(13U),
} CpswCpts_Ioctl;


/*!
 * \brief CPTS clock frequency add value.
 */
typedef enum CpswCpts_RftClkFreq_e
{
    /*! CPTS reference clock frequency: 1000MHz */
    CPSW_CPTS_RFTCLK_FREQ_1000MHZ    = 0U,

    /*! CPTS reference clock frequency: 500MHz */
    CPSW_CPTS_RFTCLK_FREQ_500MHZ     = 1U,

    /*! CPTS reference clock frequency: 333.33MHz */
    CPSW_CPTS_RFTCLK_FREQ_333_33MHZ  = 2U,

    /*! CPTS reference clock frequency: 250MHz */
    CPSW_CPTS_RFTCLK_FREQ_250MHZ     = 3U,

    /*! CPTS reference clock frequency: 200MHz */
    CPSW_CPTS_RFTCLK_FREQ_200MHZ     = 4U,

    /*! CPTS reference clock frequency: 166.66MHz */
    CPSW_CPTS_RFTCLK_FREQ_166_66MHZ  = 5U,

    /*! CPTS reference clock frequency: 142.85MHz */
    CPSW_CPTS_RFTCLK_FREQ_142_85MHZ  = 6U,

    /*! CPTS reference clock frequency: 125MHz */
    CPSW_CPTS_RFTCLK_FREQ_125MHZ     = 7U,
} CpswCpts_RftClkFreq;

/*!
 * \brief CPTS event type.
 */
typedef enum CpswCpts_EventType_e
{
    /*! CPTS software time stamp push event */
    CPSW_CPTS_EVENTTYPE_TS_PUSH = 0U,

    /*! CPTS time stamp rollover event (32-bit mode only) */
    CPSW_CPTS_EVENTTYPE_TS_ROLLOVER,

    /*! CPTS time stamp half rollover event (32-bit mode only) */
    CPSW_CPTS_EVENTTYPE_TS_HALFROLLOVER,

    /*! CPTS hardware time stamp push event */
    CPSW_CPTS_EVENTTYPE_HW_TS_PUSH,

    /*! CPTS Ethernet receive event */
    CPSW_CPTS_EVENTTYPE_ETH_RECEIVE,

    /*! CPTS Ethernet transmit event */
    CPSW_CPTS_EVENTTYPE_ETH_TRANSMIT,

    /*! CPTS time stamp compare event */
    CPSW_CPTS_EVENTTYPE_TS_COMP,

    /*! CPTS time stamp host transmit event */
    CPSW_CPTS_EVENTTYPE_TS_HOST_TX,

    /*! Invalid event */
    CPSW_CPTS_EVENTTYPE_INVALID = -1,
} CpswCpts_EventType;

/*!
 * \brief CPTS time stamp output bit.
 */
typedef enum CpswCpts_OutputBitSel_e
{
    /*! CPTS time stamp output bit selection disabled */
    CPSW_CPTS_TS_OUTPUT_BIT_DISABLED = 0U,

    /*! CPTS select time stamp output bit 17 */
    CPSW_CPTS_TS_OUTPUT_BIT_17,

    /*! CPTS select time stamp output bit 18 */
    CPSW_CPTS_TS_OUTPUT_BIT_18,

    /*! CPTS select time stamp output bit 19 */
    CPSW_CPTS_TS_OUTPUT_BIT_19,

    /*! CPTS select time stamp output bit 20 */
    CPSW_CPTS_TS_OUTPUT_BIT_20,

    /*! CPTS select time stamp output bit 21 */
    CPSW_CPTS_TS_OUTPUT_BIT_21,

    /*! CPTS select time stamp output bit 22 */
    CPSW_CPTS_TS_OUTPUT_BIT_22,

    /*! CPTS select time stamp output bit 23 */
    CPSW_CPTS_TS_OUTPUT_BIT_23,

    /*! CPTS select time stamp output bit 24 */
    CPSW_CPTS_TS_OUTPUT_BIT_24,

    /*! CPTS select time stamp output bit 25 */
    CPSW_CPTS_TS_OUTPUT_BIT_25,

    /*! CPTS select time stamp output bit 26 */
    CPSW_CPTS_TS_OUTPUT_BIT_26,

    /*! CPTS select time stamp output bit 27 */
    CPSW_CPTS_TS_OUTPUT_BIT_27,

    /*! CPTS select time stamp output bit 28 */
    CPSW_CPTS_TS_OUTPUT_BIT_28,

    /*! CPTS select time stamp output bit 29 */
    CPSW_CPTS_TS_OUTPUT_BIT_29,

    /*! CPTS select time stamp output bit 30 */
    CPSW_CPTS_TS_OUTPUT_BIT_30,

    /*! CPTS select time stamp output bit 31 */
    CPSW_CPTS_TS_OUTPUT_BIT_31,
} CpswCpts_OutputBitSel;

/*!
 * \brief CPTS Hardware push instances.
 */
typedef enum CpswCpts_HwPush_e
{
    /*! First hardware push instance */
    CPSW_CPTS_HWPUSH_FIRST = 1U,

    /*! Hardware push instance 1 */
    CPSW_CPTS_HWPUSH_1 = CPSW_CPTS_HWPUSH_FIRST,

    /*! Hardware push instance 2 */
    CPSW_CPTS_HWPUSH_2,

    /*! Hardware push instance 3 */
    CPSW_CPTS_HWPUSH_3,

    /*! Hardware push instance 4 */
    CPSW_CPTS_HWPUSH_4,

    /*! Hardware push instance 5 */
    CPSW_CPTS_HWPUSH_5,

    /*! Hardware push instance 6 */
    CPSW_CPTS_HWPUSH_6,

    /*! Hardware push instance 7 */
    CPSW_CPTS_HWPUSH_7,

    /*! Hardware push instance 8 */
    CPSW_CPTS_HWPUSH_8,

    /*! Last hardware push instance */
    CPSW_CPTS_HWPUSH_LAST = CPSW_CPTS_HWPUSH_8,

    /*! Invalid hardware push instance */
    CPSW_CPTS_HWPUSH_INVALID  = -1,
} CpswCpts_HwPush;

/*! \brief Max number of hardware push instances. */
#define CPSW_CPTS_HWPUSH_COUNT_MAX            ((uint32_t)CPSW_CPTS_HWPUSH_LAST)

/*! \brief Helper macro to normalize CpswCpts_HwPush values. */
#define CPSW_CPTS_HWPUSH_NORM(x)              ((uint32_t)((x) - CPSW_CPTS_HWPUSH_FIRST))

/*!
 * \brief Genf adjustment direction.
 */
typedef enum CpswCpts_FxnGenAdjDir_e
{
    /*! Decrease the Genf frequency with adjustment value */
    CPSW_CPTS_GENF_PPM_ADJDIR_DECREASE = 0U,

    /*! Increase the Genf frequency with adjustment value */
    CPSW_CPTS_GENF_PPM_ADJDIR_INCREASE,
} CpswCpts_FxnGenAdjDir;

/*!
 * \brief CPTS event.
 */
typedef struct CpswCpts_Event_s
{
    /*! Time stamp value when the event occurred */
    uint64_t tsVal;

    /*! CPTS event type information */
    CpswCpts_EventType eventType;

    /*! CPTS event message type */
    EnetTimeSync_MsgType msgType;

    /*! Sequence id of the event */
    uint32_t seqId;

    /*! MAC port number corresponding to the CPTS event */
    uint32_t portNum;

    /*! Domain number of the event */
    uint32_t domain;

    /*! Hardware push instance number which triggered the event */
    CpswCpts_HwPush hwPushNum;
} CpswCpts_Event;

/*!
 * \brief CPTS EST event match params.
 */
typedef struct CpswCpts_EstEventMatchParams_s
{
    /*! MAC port number */
    Enet_MacPort macPort;

    /*! Domain number of the event. The generated EST events will have the domain
     *  set via \ref CpswMacPort_EstTimestampCfg::domain */
    uint32_t domain;
} CpswCpts_EstEventMatchParams;

/*!
 * \brief CPTS EST timestamp.
 */
typedef struct CpswCpts_EstEvent_s
{
    /*! Time stamp value when the event occurred */
    uint64_t tsVal;

    /*! Hardware switch priority */
    uint8_t priority;

    /*! Sequence number of the event */
    uint32_t seqNum;

    /*! Packet receive port number */
    uint32_t ingressPort;

    /*! Transmit port number */
    uint32_t egressPort;

    /*! Domain number of the event */
    uint32_t domain;
} CpswCpts_EstEvent;

/*!
 * \brief CPTS event notify callback.
 *
 * \param eventNotifyCbArg      Callback argument
 * \param eventInfo             CPTS event information
 *
 */
typedef void (*CpswCpts_EventNotifyCb)(void *eventNotifyCbArg,
                                       CpswCpts_Event *eventInfo);

/*!
 * \brief CPTS hardware push notify callback/
 *
 * \param hwPushNotifyCbArg      Callback argument
 * \param hwPushNum              CPTS hardware push instance number
 *
 */
typedef void (*CpswCpts_HwPushNotifyCb)(void *hwPushNotifyCbArg,
                                        CpswCpts_HwPush hwPushNum);

/*!
 * \brief Input args for #CPSW_CPTS_IOCTL_REGISTER_STACK command.
 *
 * Configuration information for registering a stack with the CPTS module.
 */
typedef struct CpswCpts_StackConfig_s
{
    /*! Pointer to event notify callback function */
    CpswCpts_EventNotifyCb eventNotifyCb;

    /*! Pointer to event notify callback function argument */
    void *eventNotifyCbArg;
} CpswCpts_RegisterStackInArgs;

/*!
 * \brief Input args for #CPSW_CPTS_IOCTL_REGISTER_HWPUSH_CALLBACK command.
 */
typedef struct CpswCpts_RegisterHwPushCbInArgs_s
{
    /*! Hardware push instance number */
    CpswCpts_HwPush hwPushNum;

    /*! Pointer to hardware push notify callback function */
    CpswCpts_HwPushNotifyCb hwPushNotifyCb;

    /*! Pointer to hardware push notify callback function argument */
    void *hwPushNotifyCbArg;
} CpswCpts_RegisterHwPushCbInArgs;

/*!
 * \brief Input args for #CPSW_CPTS_IOCTL_SET_COMP command.
 */
typedef struct CpswCpts_SetCompValInArgs_s
{
    /*! 64-bit time stamp compare value */
    uint64_t tsCompVal;

    /*! Time stamp comparison length value */
    uint32_t tsCompLen;

    /*! Time stamp compare toggle mode selection.
     *     true : Toggle mode
     *     false: Non-toggle mode */
    bool tsCompToggle;
} CpswCpts_SetCompValInArgs;

/*!
 * \brief Input args for #CPSW_CPTS_IOCTL_SET_GENF and #CPSW_CPTS_IOCTL_SET_ESTF
 *        commands.
 */
typedef struct CpswCpts_SetFxnGenInArgs_s
{
    /*! Function generator instance index value */
    uint32_t index;

    /*! Function generator length value */
    uint32_t length;

    /*! 64-bit function generator compare value */
    uint64_t compare;

    /*! Function generator polarity inversion flag
     *      true : Output of function generator asserts high
     *      false: Output of function generator asserts low */
    bool polarityInv;

    /*! Function generator parts-per-million value */
    uint64_t ppmVal;

    /*! Direction of parts-per-million correction value */
    CpswCpts_FxnGenAdjDir ppmDir;

    /*! Mode of correction value for GENF PPM: parts-per-million or parts-per-hour.
     *  If the ppmMode is set to CPSW_CPTS_PPM_DISABLE, the ppm value will be
     *  set to zero. */
    EnetTimeSync_AdjMode ppmMode;
} CpswCpts_SetFxnGenInArgs;

/*!
 * \brief Input args for #CPSW_CPTS_IOCTL_SET_GENF_NUDGE and
 *        #CPSW_CPTS_IOCTL_SET_ESTF_NUDGE commands.
 */
typedef struct CpswCpts_SetFxnGenNudgeInArgs_s
{
    /*! Function generator instance index value */
    uint32_t index;

    /*! Function generator nudge value. It must be within the range of
     *  #CPSW_CPTS_NUDGE_MIN_VAL and #CPSW_CPTS_NUDGE_MAX_VAL. */
    int32_t tsNudge;
} CpswCpts_SetFxnGenNudgeInArgs;

/*!
 * \brief CPTS configuration.
 *
 * Configuration information for the CPTS module.
 */
typedef struct CpswCpts_Cfg_s
{
    /*! Host Receive timestamp enable: When set, timestamps are enabled on
     *  received packets to host */
    bool hostRxTsEn;

    /*! TS_COMP polarity:
     *   true:  TS_COMP is asserted high
     *   false: TS_COMP is asserted low */
    bool tsCompPolarity;

    /*! Disable all timestamp Ethernet receive events */
    bool tsRxEventsDis;

    /*! GENF (and ESTF) clear enable:
     *   true:  A TS_GENFn (or TS_ESTFn) output is cleared when the associated
     *          ts_genf_length[31:0] (or ts_estf_length[31:0]) is cleared to zero
     *   false: A TS_GENFn (or TS_ESTFn) output is not cleared when the associated
     *          ts_genf_length[31:0] (or ts_estf_length[31:0]) is cleared to zero */
    bool tsGenfClrEn;

    /*! CPTS RFT clock frequency required to set TS_ADD VAL */
    CpswCpts_RftClkFreq cptsRftClkFreq;
} CpswCpts_Cfg;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief Initialize CPSW CPTS configuration parameters.
 *
 * \param cptsCfg   Configuration parameters to be initialized
 */
void CpswCpts_initCfg(CpswCpts_Cfg *cptsCfg);

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

#endif /* CPSW_CPTS_H_ */

/*! @} */
