/*
 * Copyright (C) 2026 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*!
 * \file  cli_timesync.c
 *
 * \brief This file contains scripts to timesync cli commands
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "cli_timesync.h"
#include <priv/per/cpsw_cpdma_priv.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define ENET_TIMESYNC_EVENTS_COUNT        (4)
#define ENET_TIMESYNC_MAX_EVENTS          (4)
#define ENET_TIMESYNC_EVENT_ARRAY_SIZE    (ENET_TIMESYNC_MAX_EVENTS-1)
#define ENET_TIMESYNC_NSEC_IN_SEC         (1000000000)
/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef struct PtpSendRequest_s
{
    uint16_t seqId;         /* Sequence ID for PTP packet */
    uint8_t msgType;        /* PTP message type */
    uint8_t domain;         /* PTP domain number */
    uint32_t count;         /* Remaining packets to send */
    uint8_t transport;      /* Transport type: 0=L2, 1=UDP */
    uint32_t totalCount;    /* Total packet count (for progress tracking) */
    int32_t macPort;        /* Macport number to send packet */
} PtpSendRequest_t;

typedef struct EventList_s
{
    CpswCpts_Event list[ENET_TIMESYNC_EVENT_ARRAY_SIZE];
    int8_t writeIdx;
    int8_t readIdx;
    int8_t lastIdx;
} EventList_t;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void EnetApp_timesyncGetVersion(void);
static void EnetApp_timesyncPrintRegs(void);
#if ENET_CFG_IS_ON(ENET_CFG_CPSW_CPTS_STATS)
static void EnetApp_timesyncPrintStats(void);
#endif
static void EnetApp_timesyncGetCurrentTimestamp(void);
static void EnetApp_timesyncSetTimestamp(const char *commandString);
static void EnetApp_timesyncAdjustTimestamp(const char *commandString);
static void EnetApp_timesyncGetEthRxTimestamp(const char *commandString);
static void EnetApp_timesyncGetEthTxTimestamp(const char *commandString);
static void EnetApp_timesyncReset(void);
static void EnetApp_timesyncRegisterEvents(const char *commandString);
static void EnetApp_timesyncUnregisterEvents(const char *commandString);
static void EnetCLI_dumpTSinfo(const char *commandString);
static void EnetApp_timesyncSendPtpPacket(const char *commandString);
static void EnetApp_cptsSetTsNudge(const char *commandString);
static void EnetApp_cptsSetComp(const char *commandString);
static void EnetApp_cptsSetCompNudge(const char *commandString);
static void EnetApp_cptsSelectTsOutputBit(const char *commandString);
static void EnetApp_cptsSetAddVal(const char *commandString);

static int32_t EnetApp_ptpSendPacketImpl(const PtpSendRequest_t *request, uint32_t *sentCount);

static int32_t EnetApp_registerEventStack(void);

static int32_t EnetApp_unregisterEventStack(void);

static int32_t EnetApp_enablePortEvents(void);

static void EnetApp_eventIsr(void *eventNotifyCbArg, CpswCpts_Event *eventInfo);

static void EnetApp_printCptsEvent(CpswCpts_Event* evt);

static void EnetApp_setPortTsEventPrms(CpswMacPort_TsEventCfg *tsPortEventCfg);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Store Rx Timestamp Events */
static EventList_t EnetApp_eventList[ENET_TIMESYNC_EVENTS_COUNT] = {
    [0] = {.writeIdx = 0, .readIdx = 0, .lastIdx = ENET_TIMESYNC_EVENT_ARRAY_SIZE-1},
    [1] = {.writeIdx = 0, .readIdx = 0, .lastIdx = ENET_TIMESYNC_EVENT_ARRAY_SIZE-1},
    [2] = {.writeIdx = 0, .readIdx = 0, .lastIdx = ENET_TIMESYNC_EVENT_ARRAY_SIZE-1},
    [3] = {.writeIdx = 0, .readIdx = 0, .lastIdx = ENET_TIMESYNC_EVENT_ARRAY_SIZE-1}
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

BaseType_t EnetCLI_timesyncCommands(char *writeBuffer, size_t writeBufferLen,
        const char *commandString)
{
    char *parameter;
    BaseType_t paramLen;
    uint32_t paramCnt = 1;

    parameter = (char*) FreeRTOS_CLIGetParameter(commandString, paramCnt,
                        &paramLen);
    if(parameter == NULL)
    {
        EnetAppUtils_print("Use 'help' command to see Usage.\r\n");
        return pdFALSE;
    }

    if(strncmp(parameter, "help", paramLen) == 0)
    {
        EnetAppUtils_print(
            "timesync Commands:\r\n"
            "  get_version\r\n"
            "  print_regs\r\n");
        EnetAppUtils_print(
#if ENET_CFG_IS_ON(ENET_CFG_CPSW_CPTS_STATS)
            "  print_stats\r\n"
#endif
            "  get_current_timestamp\r\n"
            "  set_timestamp <sec> <nsec>\r\n"
            "  adjust_timestamp <sec> <nsec>\r\n");
        EnetAppUtils_print(
            "  get_eth_rx_timestamp <msgtype> <seqid> <domain>\r\n"
            "  get_eth_tx_timestamp <msgtype> <seqid> <domain>\r\n"
            "  register_events\r\n"
            "  unregister_events\r\n"
            "  dump_events\r\n");
        EnetAppUtils_print(
            "  send_ptp <seqid> <msgtype> <domain> <count> <transport>\r\n"
            "    transport: 0=L2, 1=UDP\r\n"
            "  set_ts_nudge <nudge_val>\r\n"
            "  set_comp <sec> <nsec>\r\n");
        EnetAppUtils_print(
            "  set_comp_nudge <nudge_val>\r\n"
            "  select_ts_output <bit_val>\r\n"
            "  set_add_val <add_val>\r\n"
            "  reset\r\n");
        return pdFALSE;
    }

    if(strncmp(parameter, "get_version", paramLen) == 0)
    {
        EnetApp_timesyncGetVersion();
    }
    else if(strncmp(parameter, "print_regs", paramLen) == 0)
    {
        EnetApp_timesyncPrintRegs();
    }
#if ENET_CFG_IS_ON(ENET_CFG_CPSW_CPTS_STATS)
    else if(strncmp(parameter, "print_stats", paramLen) == 0)
    {
        EnetApp_timesyncPrintStats();
    }
#endif
    else if(strncmp(parameter, "get_current_timestamp", paramLen) == 0)
    {
        EnetApp_timesyncGetCurrentTimestamp();
    }
    else if(strncmp(parameter, "set_timestamp", paramLen) == 0)
    {
        EnetApp_timesyncSetTimestamp(commandString);
    }
    else if(strncmp(parameter, "adjust_timestamp", paramLen) == 0)
    {
        EnetApp_timesyncAdjustTimestamp(commandString);
    }
    else if(strncmp(parameter, "get_eth_rx_timestamp", paramLen) == 0)
    {
        EnetApp_timesyncGetEthRxTimestamp(commandString);
    }
    else if(strncmp(parameter, "get_eth_tx_timestamp", paramLen) == 0)
    {
        EnetApp_timesyncGetEthTxTimestamp(commandString);
    }
    else if(strncmp(parameter, "register_events", paramLen) == 0)
    {
        EnetApp_timesyncRegisterEvents(commandString);
    }
    else if(strncmp(parameter, "unregister_events", paramLen) == 0)
    {
        EnetApp_timesyncUnregisterEvents(commandString);
    }
    else if(strncmp(parameter, "reset", paramLen) == 0)
    {
        EnetApp_timesyncReset();
    }
    else if(strncmp(parameter, "dump_events", paramLen) == 0)
    {
        EnetCLI_dumpTSinfo(commandString);
    }
    else if(strncmp(parameter, "send_ptp", paramLen) == 0)
    {
        EnetApp_timesyncSendPtpPacket(commandString);
    }
    else if(strncmp(parameter, "set_ts_nudge", paramLen) == 0)
    {
        EnetApp_cptsSetTsNudge(commandString);
    }
    else if(strncmp(parameter, "set_comp", paramLen) == 0)
    {
        EnetApp_cptsSetComp(commandString);
    }
    else if(strncmp(parameter, "set_comp_nudge", paramLen) == 0)
    {
        EnetApp_cptsSetCompNudge(commandString);
    }
    else if(strncmp(parameter, "select_ts_output", paramLen) == 0)
    {
        EnetApp_cptsSelectTsOutputBit(commandString);
    }
    else if(strncmp(parameter, "set_add_val", paramLen) == 0)
    {
        EnetApp_cptsSetAddVal(commandString);
    }
    else
    {
        EnetAppUtils_print("Invalid Command.\r\n");
        return pdFALSE;
    }

    return pdFALSE;
}

static void EnetCLI_dumpTSinfo(const char *commandString)
{
    int8_t idx;
    int32_t eventCount = 0;
    uint32_t eventIdx;
    int32_t i;
    CpswCpts_Event* evt;
    uintptr_t key;
    int32_t eventTypeCounts[ENET_TIMESYNC_EVENTS_COUNT];

    key = EnetOsal_disableAllIntr();

    for (eventIdx = 0; eventIdx < ENET_TIMESYNC_EVENTS_COUNT; eventIdx++)
    {
        idx = EnetApp_eventList[eventIdx].readIdx;
        eventTypeCounts[eventIdx] = 0;
        while(idx != EnetApp_eventList[eventIdx].writeIdx)
        {
            eventTypeCounts[eventIdx]++;
            eventCount++;
            idx = (idx == EnetApp_eventList[eventIdx].lastIdx) ? 0 : idx + 1;
        }
    }

    EnetOsal_restoreAllIntr(key);

    EnetAppUtils_print("Results From last %d events.\r\n", eventCount);

    for (eventIdx = 0; eventIdx < ENET_TIMESYNC_EVENTS_COUNT; eventIdx++)
    {
        idx = EnetApp_eventList[eventIdx].writeIdx;

        for(i = 0; i < eventTypeCounts[eventIdx]; i++)
        {
            idx = (idx == 0) ? EnetApp_eventList[eventIdx].lastIdx : idx - 1;
            evt = &EnetApp_eventList[eventIdx].list[idx];
            EnetApp_printCptsEvent(evt);
        }
    }
}

/* ========================================================================== */
/*                   Static Function Definitions                              */
/* ========================================================================== */

static void EnetApp_timesyncGetVersion(void)
{
    Enet_IoctlPrms prms;
    Enet_Version version;
    int32_t status;

    ENET_IOCTL_SET_OUT_ARGS(&prms, &version);
    ENET_IOCTL(EnetApp_inst.hEnet, EnetApp_inst.coreId, ENET_TIMESYNC_IOCTL_GET_VERSION, &prms, status);

    if(status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to get version\r\n");
    }
    else
    {
        EnetAppUtils_print("Version === \r\n\tmajor = %d\r\n\tminor = %d\r\n"
            "\trtl = %d\r\n", version.maj, version.min, version.rtl);
    }
}

static void EnetApp_timesyncPrintRegs(void)
{
    Enet_IoctlPrms prms;
    int32_t status;

    /* Print CPTS (Precision Time Protocol Synchronization) hardware registers:
     * Includes timestamp counter registers, output pin selection, and synchronization control registers */
    ENET_IOCTL(EnetApp_inst.hEnet, EnetApp_inst.coreId, ENET_TIMESYNC_IOCTL_PRINT_REGS, &prms, status);

    if(status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to print registers\r\n");
    }
    else
    {
        EnetAppUtils_print("Registers printed\r\n");
    }
}

#if ENET_CFG_IS_ON(ENET_CFG_CPSW_CPTS_STATS)
static void EnetApp_timesyncPrintStats(void)
{
    Enet_IoctlPrms prms;
    int32_t status;

    /* Print CPTS statistics: RX/TX timestamp events captured, overflow counters,
     * and synchronization quality metrics */
    ENET_IOCTL(EnetApp_inst.hEnet, EnetApp_inst.coreId, ENET_TIMESYNC_IOCTL_PRINT_STATS, &prms, status);

    if(status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to print statistics\r\n");
    }
    else
    {
        EnetAppUtils_print("Statistics printed\r\n");
    }
}
#endif

static void EnetApp_timesyncGetCurrentTimestamp(void)
{
    Enet_IoctlPrms prms;
    uint64_t tsVal;
    int32_t status;

    ENET_IOCTL_SET_OUT_ARGS(&prms, &tsVal);
    ENET_IOCTL(EnetApp_inst.hEnet, EnetApp_inst.coreId, ENET_TIMESYNC_IOCTL_GET_CURRENT_TIMESTAMP, &prms, status);

    if(status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to get current timestamp\r\n");
    }
    else
    {
        uint64_t sec = tsVal/ENET_TIMESYNC_NSEC_IN_SEC;
        uint64_t nsec = tsVal%ENET_TIMESYNC_NSEC_IN_SEC;
        EnetAppUtils_print("Current timestamp = %llu sec %llu nsec\r\n", sec, nsec);
    }
}

static void EnetApp_timesyncSetTimestamp(const char *commandString)
{
    char *parameter;
    BaseType_t paramLen;
    int32_t status;
    uint64_t timestamp;
    int32_t sec, nsec;

    parameter = (char*) FreeRTOS_CLIGetParameter(commandString, 2, &paramLen);
    if(parameter == NULL)
    {
        EnetAppUtils_print( "Usage: timesync set_timestamp <sec> <nsec>\r\n");
        return;
    }

    sec = atoi(parameter);

    parameter = (char*) FreeRTOS_CLIGetParameter(commandString, 3, &paramLen);
    if(parameter == NULL)
    {
        EnetAppUtils_print( "Usage: timesync set_timestamp <sec> <nsec>\r\n");
        return;
    }

    nsec = atoi(parameter);

    if((nsec < 0) || (sec < 0))
    {
        EnetAppUtils_print( "Max sec/nsec value is %d\r\n", 0x7FFFFFFF);
        return;
    }
    timestamp = (uint64_t)sec * ENET_TIMESYNC_NSEC_IN_SEC;
    timestamp += nsec;

    Enet_IoctlPrms prms;
    ENET_IOCTL_SET_IN_ARGS(&prms, &timestamp);

    ENET_IOCTL(EnetApp_inst.hEnet, EnetApp_inst.coreId, ENET_TIMESYNC_IOCTL_SET_TIMESTAMP, &prms, status);

    if(status != ENET_SOK)
    {
        EnetAppUtils_print( "Failed to set timestamp\r\n");
    }
    else
    {
        EnetAppUtils_print( "Timestamp set to %u sec %u nsec\r\n", sec, nsec);
    }
}

static void EnetApp_timesyncAdjustTimestamp(const char *commandString)
{
    char *parameter;
    BaseType_t paramLen;
    int32_t status;
    uint32_t intervalInNsec;
    int32_t adjustValInNsec;
    EnetTimeSync_TimestampAdj inArgs;
    Enet_IoctlPrms prms;

    parameter = (char*) FreeRTOS_CLIGetParameter(commandString, 2, &paramLen);
    if(parameter == NULL)
    {
        EnetAppUtils_print( "Usage: timesync adjust_timestamp <intervalInNsec> <adjustInNsec>\r\n");
        return;
    }

    intervalInNsec = (uint32_t)atoll(parameter);

    parameter = (char*) FreeRTOS_CLIGetParameter(commandString, 3, &paramLen);
    if(parameter == NULL)
    {
        EnetAppUtils_print( "Usage: timesync adjust_timestamp <intervalInNsec> <adjustInNsec>\r\n");
        return;
    }

    adjustValInNsec = atoi(parameter);

    inArgs.adjValInNsecs = adjustValInNsec;
    inArgs.intervalInNsecs = intervalInNsec;

    ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);

    ENET_IOCTL(EnetApp_inst.hEnet, EnetApp_inst.coreId, ENET_TIMESYNC_IOCTL_ADJUST_TIMESTAMP, &prms, status);

    if(status != ENET_SOK)
    {
        EnetAppUtils_print( "Failed to adjust timestamp\r\n");
    }
    else
    {
        EnetAppUtils_print( "Timestamp adjusted %d ns in %d ns\r\n", adjustValInNsec, intervalInNsec);
    }
}

static int32_t EnetApp_getTimestampParams(const char *commandString, const char *cmdName,
                                          uint32_t *msgType, uint32_t *seqId,
                                          uint32_t *domain, int32_t *portNum)
{
    char *parameter;
    BaseType_t paramLen;

    /* Get msgType parameter */
    parameter = (char*) FreeRTOS_CLIGetParameter(commandString, 2, &paramLen);
    if(parameter == NULL)
    {
        EnetAppUtils_print( "Usage: timesync %s <msgtype> <seqid> <domain> <portNum>\r\n", cmdName);
        return -1;
    }
    *msgType = atoi(parameter);

    /* Get seqId parameter */
    parameter = (char*) FreeRTOS_CLIGetParameter(commandString, 3, &paramLen);
    if(parameter == NULL)
    {
        EnetAppUtils_print( "Usage: timesync %s <msgtype> <seqid> <domain> <portNum>\r\n", cmdName);
        return -1;
    }
    *seqId = atoi(parameter);

    /* Get domain parameter */
    parameter = (char*) FreeRTOS_CLIGetParameter(commandString, 4, &paramLen);
    if(parameter == NULL)
    {
        EnetAppUtils_print( "Usage: timesync %s <msgtype> <seqid> <domain> <portNum>\r\n", cmdName);
        return -1;
    }
    *domain = atoi(parameter);

    /* Get port number parameter */
    parameter = (char*) FreeRTOS_CLIGetParameter(commandString, 5, &paramLen);
    if(parameter == NULL)
    {
        EnetAppUtils_print( "Usage: timesync %s <msgtype> <seqid> <domain> <portNum>\r\n", cmdName);
        return -1;
    }
    *portNum = atoi(parameter);

    if((*portNum < 0) || (*portNum > 2))
    {
        EnetAppUtils_print( "Port number should be between %d and %d\r\n", 1, 2);
        return -1;
    }

    return 0;
}

static void EnetApp_timesyncGetEthRxTimestamp(const char *commandString)
{
    Enet_IoctlPrms prms;
    int32_t status;
    EnetTimeSync_GetEthTimestampInArgs inArgs;
    uint32_t msgType;
    uint32_t seqId;
    uint32_t domain;
    uint64_t tsVal;
    int32_t portNum;

    if (EnetApp_getTimestampParams(commandString, "get_eth_rx_timestamp",
                                    &msgType, &seqId, &domain, &portNum) != 0)
    {
        return;
    }

    inArgs.msgType = (EnetTimeSync_MsgType)msgType;
    inArgs.seqId = seqId;
    inArgs.portNum = portNum-1;
    inArgs.domain = domain;

    ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs, &tsVal);
    ENET_IOCTL(EnetApp_inst.hEnet, EnetApp_inst.coreId, ENET_TIMESYNC_IOCTL_GET_ETH_RX_TIMESTAMP, &prms, status);

    if(status != ENET_SOK)
    {
        EnetAppUtils_print( "Failed to get Ethernet RX timestamp\r\n");
    }
    else
    {
        EnetAppUtils_print( "Ethernet RX timestamp retrieved for MsgType:%u SeqId:%u Domain:%u\r\n"
                "\tTime stamp val %llu sec %llu nsec\r\n", msgType, seqId, domain, tsVal/ENET_TIMESYNC_NSEC_IN_SEC, tsVal%ENET_TIMESYNC_NSEC_IN_SEC);
    }
}

static void EnetApp_timesyncGetEthTxTimestamp(const char *commandString)
{
    Enet_IoctlPrms prms;
    int32_t status;
    EnetTimeSync_GetEthTimestampInArgs inArgs;
    uint32_t msgType;
    uint32_t seqId;
    uint32_t domain;
    uint64_t tsVal;
    int32_t portNum;

    if (EnetApp_getTimestampParams(commandString, "get_eth_tx_timestamp",
                                    &msgType, &seqId, &domain, &portNum) != 0)
    {
        return;
    }

    inArgs.msgType = (EnetTimeSync_MsgType)msgType;
    inArgs.seqId = seqId;
    inArgs.portNum = portNum-1;
    inArgs.domain = domain;

    ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs, &tsVal);
    ENET_IOCTL(EnetApp_inst.hEnet, EnetApp_inst.coreId, ENET_TIMESYNC_IOCTL_GET_ETH_TX_TIMESTAMP, &prms, status);

    if(status != ENET_SOK)
    {
        EnetAppUtils_print( "Failed to get Ethernet TX timestamp\r\n");
    }
    else
    {
        EnetAppUtils_print( "Ethernet TX timestamp retrieved for MsgType:%u SeqId:%u Domain:%u\r\n"
                "\tTime stamp val %llu sec %llu nsec\r\n", msgType, seqId, domain, tsVal/ENET_TIMESYNC_NSEC_IN_SEC, tsVal%ENET_TIMESYNC_NSEC_IN_SEC);
    }
}

static void EnetApp_timesyncReset(void)
{
    Enet_IoctlPrms prms;
    int32_t status;

    ENET_IOCTL(EnetApp_inst.hEnet, EnetApp_inst.coreId, ENET_TIMESYNC_IOCTL_RESET, &prms, status);

    if(status != ENET_SOK)
    {
        EnetAppUtils_print( "Failed to reset timesync\r\n");
    }
    else
    {
        EnetAppUtils_print( "Timesync reset successfully\r\n");
    }
}

static void EnetApp_timesyncRegisterEvents(const char *commandString)
{
    int32_t status;

    status = EnetApp_registerEventStack();

    if(status != ENET_SOK)
    {
        EnetAppUtils_print( "Failed to register events on MAC port 1\r\n");
    }
    else
    {
        EnetAppUtils_print( "Events registered on MAC port 1\r\n");
    }
}

static void EnetApp_timesyncUnregisterEvents(const char *commandString)
{
    int32_t status;

    status = EnetApp_unregisterEventStack();

    if(status != ENET_SOK)
    {
        EnetAppUtils_print( "Failed to unregister events\r\n");
    }
    else
    {
        EnetAppUtils_print( "Events unregistered\r\n");
    }
}

static int32_t EnetApp_registerEventStack(void)
{
    CpswCpts_RegisterStackInArgs enableTxEvtInArgs;
    Enet_IoctlPrms prms;
    int32_t status = ENET_SOK;

    status = EnetApp_enablePortEvents();
    if (status != ENET_SOK)
    {
        return status;
    }

    enableTxEvtInArgs.eventNotifyCb = &EnetApp_eventIsr;
    enableTxEvtInArgs.eventNotifyCbArg = NULL;
    ENET_IOCTL_SET_IN_ARGS(&prms, &enableTxEvtInArgs);

    ENET_IOCTL(EnetApp_inst.hEnet,
               EnetApp_inst.coreId,
               CPSW_CPTS_IOCTL_REGISTER_STACK,
               &prms,
               status);

    if (status != ENET_SOK)
    {
        EnetAppUtils_print("[ERR] Failed to register event stack: %d\r\n", status);
    }
    else
    {
        uint32_t i;
        for (i = 0; i < ENET_TIMESYNC_EVENTS_COUNT; i++)
        {
            EnetApp_eventList[i].writeIdx = 0;
            EnetApp_eventList[i].readIdx = 0;
        }
    }

    return status;
}

static int32_t EnetApp_unregisterEventStack(void)
{
    Enet_IoctlPrms prms;
    int32_t status = ENET_SOK;

    ENET_IOCTL(EnetApp_inst.hEnet,
               EnetApp_inst.coreId,
               CPSW_CPTS_IOCTL_UNREGISTER_STACK,
               &prms,
               status);

    if (status != ENET_SOK)
    {
        EnetAppUtils_print("[ERR] Failed to unregister event stack: %d\r\n", status);
    }
    else
    {
        uint32_t i;
        for (i = 0; i < ENET_TIMESYNC_EVENTS_COUNT; i++)
        {
            EnetApp_eventList[i].readIdx = EnetApp_eventList[i].writeIdx;
        }
    }

    return status;
}

static int32_t EnetApp_enablePortEvents(void)
{
    CpswMacPort_EnableTsEventInArgs enableTsEventInArgs1, enableTsEventInArgs2;
    Enet_IoctlPrms prms1, prms2;
    int32_t status = ENET_SOK;

    EnetApp_setPortTsEventPrms(&enableTsEventInArgs1.tsEventCfg);
    enableTsEventInArgs1.macPort = ENET_MAC_PORT_1;
    ENET_IOCTL_SET_IN_ARGS(&prms1, &enableTsEventInArgs1);
    ENET_IOCTL(EnetApp_inst.hEnet,
               EnetApp_inst.coreId,
               CPSW_MACPORT_IOCTL_ENABLE_CPTS_EVENT,
               &prms1,
               status);

    if (status != ENET_SOK)
    {
        EnetAppUtils_print("[ERR] Failed to enable port events on Macport %d : %d\r\n", ENET_MAC_PORT_1, status);
    }

    EnetApp_setPortTsEventPrms(&enableTsEventInArgs2.tsEventCfg);
    enableTsEventInArgs2.macPort = ENET_MAC_PORT_2;
    ENET_IOCTL_SET_IN_ARGS(&prms2, &enableTsEventInArgs2);
    ENET_IOCTL(EnetApp_inst.hEnet,
               EnetApp_inst.coreId,
               CPSW_MACPORT_IOCTL_ENABLE_CPTS_EVENT,
               &prms2,
               status);

    if (status != ENET_SOK)
    {
        EnetAppUtils_print("[ERR] Failed to enable port events on Macport %d : %d\r\n", ENET_MAC_PORT_2, status);
    }

    return status;
}

/* ========================================================================== */
/*                   Static Function Definitions                              */
/* ========================================================================== */

static void EnetApp_eventIsr(void *eventNotifyCbArg, CpswCpts_Event *eventInfo)
{
    uint32_t eventListIdx = 0;

    if (eventInfo == NULL)
    {
        return;
    }

    switch(eventInfo->eventType)
    {
        case CPSW_CPTS_EVENTTYPE_ETH_RECEIVE:
            eventListIdx = 0;
            break;
        case CPSW_CPTS_EVENTTYPE_ETH_TRANSMIT:
            eventListIdx = 1;
            break;
        case CPSW_CPTS_EVENTTYPE_TS_HOST_TX:
            eventListIdx = 2;
            break;
        case CPSW_CPTS_EVENTTYPE_TS_COMP:
            eventListIdx = 3;
            break;
        default:
            /* Unsupported event type, ignore */
            return;
    }

    CpswCpts_Event* evt = &EnetApp_eventList[eventListIdx].list[EnetApp_eventList[eventListIdx].writeIdx];
    evt->domain = eventInfo->domain;
    evt->eventType = eventInfo->eventType;
    evt->hwPushNum = eventInfo->hwPushNum;
    evt->msgType = eventInfo->msgType;
    evt->portNum = eventInfo->portNum;
    evt->seqId = eventInfo->seqId;
    evt->tsVal = eventInfo->tsVal;

    /* Move writeIdx forward */
    if(EnetApp_eventList[eventListIdx].writeIdx == EnetApp_eventList[eventListIdx].lastIdx)
    {
        EnetApp_eventList[eventListIdx].writeIdx = 0;
    }
    else
    {
        EnetApp_eventList[eventListIdx].writeIdx++;
    }

    /* If buffer is full, drop oldest event (advance readIdx) */
    if(EnetApp_eventList[eventListIdx].writeIdx == EnetApp_eventList[eventListIdx].readIdx)
    {
        if(EnetApp_eventList[eventListIdx].readIdx == EnetApp_eventList[eventListIdx].lastIdx)
        {
            EnetApp_eventList[eventListIdx].readIdx = 0;
        }
        else
        {
            EnetApp_eventList[eventListIdx].readIdx++;
        }
    }
}

static void EnetApp_printCptsEvent(CpswCpts_Event* evt)
{
    char* eventType;

    switch(evt->eventType){
        case CPSW_CPTS_EVENTTYPE_ETH_RECEIVE:
        {
            eventType = "Receive event";
            break;
        }
        case CPSW_CPTS_EVENTTYPE_ETH_TRANSMIT:
        {
            eventType = "Transmit event";
            break;
        }
        case CPSW_CPTS_EVENTTYPE_TS_HOST_TX:
        {
            eventType = "Host TX event";
            break;
        }
        case CPSW_CPTS_EVENTTYPE_TS_COMP:
        {
            eventType = "TS Comp event";
            break;
        }
        default:
        {
            eventType = "Event unknown";
            break;
        }
    }

    EnetAppUtils_print("------------------------------------\r\n");
    EnetAppUtils_print("Event Type : %s\r\n", eventType);
    EnetAppUtils_print("Time stamp value : %llu sec %llu nsec\r\n", evt->tsVal/ENET_TIMESYNC_NSEC_IN_SEC, evt->tsVal%ENET_TIMESYNC_NSEC_IN_SEC);
    EnetAppUtils_print("Sequence ID : %u\r\n", evt->seqId);
    EnetAppUtils_print("------------------------------------\r\n");
}

static int32_t EnetApp_ptpSendPacketImpl(const PtpSendRequest_t *request, uint32_t *sentCount)
{
    EnetDma_PktQ txSubmitQ;
    EnetDma_PktQ txFreeQ;
    EnetDma_Pkt *pktInfo;
    EnetDma_Pkt *retrievedPkt;
    EthFrame *frame;
    uint8_t *bufPtr;
    uint8_t *ptp;
    uint8_t *ipv4;
    uint8_t *udp;
    uint8_t *pLoad;
    int32_t status;
    int8_t dmaChNum = 0;
    uint16_t payloadLen = 60;
    uint8_t ptpMulticastMac[ENET_MAC_ADDR_LEN] = {0x01, 0x1B, 0x19, 0x00, 0x00, 0x00};
    uint32_t txCnt, retrieveTxCnt;
    uint32_t i;
    uint32_t frameSize;
    uint32_t ipv4HeaderSize = 20;
    uint32_t udpHeaderSize = 8;
    uint32_t ptpHeaderSize;
    uint32_t totalHeaderLength;
    uint16_t msgLen;
    uint16_t portNum;
    uint16_t seqIdNetOrder;
    uint16_t totLen;
    uint16_t ident;
    uint16_t flagsFrag;
    uint32_t checksum;
    uint16_t srcPort;
    uint16_t dstPort;
    uint16_t udpLen;
    int k;
    uint16_t j;
    uint32_t val;

    *sentCount = 0;

    for (i = 0; i < request->count; i++)
    {
        EnetQueue_initQ(&txSubmitQ);

        /* Dequeue one free TX packet */
        pktInfo = (EnetDma_Pkt*) EnetQueue_deq(&EnetApp_inst.txFreePktInfoQ[dmaChNum]);

        /* If no free packet available, try to retrieve transmitted packets first */
        if (NULL == pktInfo)
        {
            EnetQueue_initQ(&txFreeQ);
            status = EnetDma_retrieveTxPktQ(EnetApp_inst.hTxCh[dmaChNum], &txFreeQ);

            if (status == ENET_SOK)
            {
                retrievedPkt = (EnetDma_Pkt*) EnetQueue_deq(&txFreeQ);
                while (NULL != retrievedPkt)
                {
                    EnetDma_checkPktState(&retrievedPkt->pktState, ENET_PKTSTATE_MODULE_APP,
                            ENET_PKTSTATE_APP_WITH_DRIVER,
                            ENET_PKTSTATE_APP_WITH_FREEQ);

                    EnetQueue_enq(&EnetApp_inst.txFreePktInfoQ[dmaChNum],
                            &retrievedPkt->node);
                    retrievedPkt = (EnetDma_Pkt*) EnetQueue_deq(&txFreeQ);
                }
            }

            /* Try to dequeue again after retrieving transmitted packets */
            pktInfo = (EnetDma_Pkt*) EnetQueue_deq(&EnetApp_inst.txFreePktInfoQ[dmaChNum]);
        }

        /* Report error if still no packets available */
        if (NULL == pktInfo)
        {
            EnetAppUtils_print("Failed to get free TX packet for PTP (sent %u/%u)\r\n",
                    *sentCount, request->count);
        }
        else
        {
            bufPtr = pktInfo->sgList.list[0].bufPtr;

            if (request->transport == 0)
            {
                /* L2 Transport (PTP over Ethernet) */
                frame = (EthFrame*) bufPtr;

                /* Clean the Ethernet frame header */
                memset(frame, 0, sizeof(EthFrame));

                /* Fill Ethernet frame header */
                memcpy(frame->hdr.dstMac, ptpMulticastMac, ENET_MAC_ADDR_LEN);
                memcpy(frame->hdr.srcMac, EnetApp_inst.hostMacAddr, ENET_MAC_ADDR_LEN);
                frame->hdr.etherType = Enet_htons(0x88F7);

                /* Fill minimal PTP v2 header in payload */
                ptp = (uint8_t *)&frame->payload[0];
                ptpHeaderSize = 34;

                ptp[0] = (0 << 4) | (request->msgType & 0x0F);
                ptp[1] = 0x02;

                msgLen = Enet_htons(ptpHeaderSize + payloadLen);
                memcpy(&ptp[2], &msgLen, 2);

                ptp[4] = request->domain & 0xFF;
                ptp[5] = 0;
                ptp[6] = 0;
                ptp[7] = 0;

                ptp[8] = 0;
                ptp[9] = 0;

                memset(&ptp[10], 0, 8);

                memcpy(&ptp[18], EnetApp_inst.hostMacAddr, 6);
                ptp[24] = 0;
                ptp[25] = 0;
                ptp[26] = 0;
                ptp[27] = 0;

                portNum = Enet_htons(1);
                memcpy(&ptp[28], &portNum, 2);

                seqIdNetOrder = Enet_htons(request->seqId);
                memcpy(&ptp[30], &seqIdNetOrder, 2);

                ptp[32] = 0;
                ptp[33] = 0x7F;

                pLoad = &ptp[34];
                for (j = 0; j < payloadLen; j++)
                {
                    pLoad[j] = 0;
                }

                frameSize = sizeof(EthFrameHeader) + ptpHeaderSize + payloadLen;
            }
            else
            {
                /* UDP Transport (PTP over UDP/IP) */
                frame = (EthFrame*) bufPtr;

                memset(frame, 0, sizeof(EthFrame));

                memcpy(frame->hdr.dstMac, ptpMulticastMac, ENET_MAC_ADDR_LEN);
                memcpy(frame->hdr.srcMac, EnetApp_inst.hostMacAddr, ENET_MAC_ADDR_LEN);
                frame->hdr.etherType = Enet_htons(0x0800);

                ipv4 = (uint8_t *)&frame->payload[0];
                ptpHeaderSize = 34;
                totalHeaderLength = ipv4HeaderSize + udpHeaderSize + ptpHeaderSize;

                ipv4[0] = 0x45;
                ipv4[1] = 0;

                totLen = Enet_htons(totalHeaderLength + payloadLen);
                memcpy(&ipv4[2], &totLen, 2);

                ident = Enet_htons(0x1234);
                memcpy(&ipv4[4], &ident, 2);

                flagsFrag = Enet_htons(0x4000);
                memcpy(&ipv4[6], &flagsFrag, 2);

                ipv4[8] = 64;
                ipv4[9] = 17;

                ipv4[10] = 0;
                ipv4[11] = 0;

                ipv4[12] = 192;
                ipv4[13] = 168;
                ipv4[14] = 1;
                ipv4[15] = 100;

                ipv4[16] = 224;
                ipv4[17] = 0;
                ipv4[18] = 1;
                ipv4[19] = 129;

                /* Calculate IPv4 checksum */
                checksum = 0;
                for (k = 0; k < ipv4HeaderSize; k += 2)
                {
                    val = (ipv4[k] << 8) | ipv4[k + 1];
                    checksum += val;
                }
                checksum = (checksum & 0xFFFF) + (checksum >> 16);
                checksum = (checksum & 0xFFFF) + (checksum >> 16);
                checksum = (~checksum) & 0xFFFF;
                ipv4[10] = (checksum >> 8) & 0xFF;
                ipv4[11] = checksum & 0xFF;

                udp = &ipv4[20];

                srcPort = Enet_htons(319);
                memcpy(&udp[0], &srcPort, 2);

                dstPort = Enet_htons(319);
                memcpy(&udp[2], &dstPort, 2);

                udpLen = Enet_htons(udpHeaderSize + ptpHeaderSize + payloadLen);
                memcpy(&udp[4], &udpLen, 2);

                udp[6] = 0;
                udp[7] = 0;

                ptp = &udp[8];

                ptp[0] = (1 << 4) | (request->msgType & 0x0F);
                ptp[1] = 0x02;

                msgLen = Enet_htons(ptpHeaderSize + payloadLen);
                memcpy(&ptp[2], &msgLen, 2);

                ptp[4] = request->domain & 0xFF;
                ptp[5] = 0;
                ptp[6] = 0;
                ptp[7] = 0;

                ptp[8] = 0;
                ptp[9] = 0;

                memset(&ptp[10], 0, 8);

                memcpy(&ptp[18], EnetApp_inst.hostMacAddr, 6);
                ptp[24] = 0;
                ptp[25] = 0;
                ptp[26] = 0;
                ptp[27] = 0;

                portNum = Enet_htons(1);
                memcpy(&ptp[28], &portNum, 2);

                seqIdNetOrder = Enet_htons(request->seqId);
                memcpy(&ptp[30], &seqIdNetOrder, 2);

                ptp[32] = 0;
                ptp[33] = 0x7F;

                pLoad = &ptp[34];
                for (j = 0; j < payloadLen; j++)
                {
                    pLoad[j] = 0;
                }

                frameSize = sizeof(EthFrameHeader) + totalHeaderLength + payloadLen;
            }

            pktInfo->sgList.list[0].segmentFilledLen = frameSize;
            pktInfo->sgList.numScatterSegments = 1;
            pktInfo->chkSumInfo = 0U;
            pktInfo->appPriv = &EnetApp_inst;
            pktInfo->txPortNum = request->macPort;

            EnetDma_checkPktState(&pktInfo->pktState, ENET_PKTSTATE_MODULE_APP,
                    ENET_PKTSTATE_APP_WITH_FREEQ,
                    ENET_PKTSTATE_APP_WITH_DRIVER);

            /* Enqueue packet for transmission */
            EnetQueue_enq(&txSubmitQ, &pktInfo->node);

            /* Record the packet count to be submitted on HW */
            txCnt = EnetQueue_getQCount(&txSubmitQ);

            /* Submit packet to DMA */
            status = EnetDma_submitTxPktQ(EnetApp_inst.hTxCh[dmaChNum], &txSubmitQ);

            if (status == ENET_SOK)
            {
                (*sentCount)++;

                /* Wait for Transmission to complete */
                SemaphoreP_pend(&EnetApp_inst.txSemObj[dmaChNum], SystemP_WAIT_FOREVER);

                txCnt = txCnt - EnetQueue_getQCount(&txSubmitQ);
                retrieveTxCnt = 0U;

                while(retrieveTxCnt < txCnt)
                {
                    /* Wait for transmission completion */
                    ClockP_usleep(1000);

                    /* Retrieve free packets */
                    EnetQueue_initQ(&txFreeQ);
                    status = EnetDma_retrieveTxPktQ(EnetApp_inst.hTxCh[dmaChNum], &txFreeQ);

                    if (status == ENET_SOK)
                    {
                        retrieveTxCnt += EnetQueue_getQCount(&txFreeQ);
                        pktInfo = (EnetDma_Pkt*) EnetQueue_deq(&txFreeQ);
                        while (NULL != pktInfo)
                        {
                            EnetDma_checkPktState(&pktInfo->pktState, ENET_PKTSTATE_MODULE_APP,
                                    ENET_PKTSTATE_APP_WITH_DRIVER,
                                    ENET_PKTSTATE_APP_WITH_FREEQ);

                            EnetQueue_enq(&EnetApp_inst.txFreePktInfoQ[dmaChNum],
                                    &pktInfo->node);
                            pktInfo = (EnetDma_Pkt*) EnetQueue_deq(&txFreeQ);
                        }
                    }
                    else
                    {
                        EnetAppUtils_print("[ERR] %s: Failed to retrieve pkts: %d\r\n",
                                __func__, status);
                    }
                }
            }
        }
    }

    return ENET_SOK;
}

static void EnetApp_timesyncSendPtpPacket(const char *commandString)
{
    char *parameter;
    BaseType_t paramLen;
    uint16_t seqId;
    uint8_t msgType;
    uint8_t domain;
    uint32_t count;
    uint8_t transport;
    uint32_t sentCount;
    int32_t portNum;
    PtpSendRequest_t currentRequest;

    /* Get seqid parameter */
    parameter = (char*) FreeRTOS_CLIGetParameter(commandString, 2, &paramLen);
    if (parameter == NULL)
    {
        EnetAppUtils_print(
            "Usage: enet_timesync send_ptp <seqid> <msgtype> <domain> <count> <transport> <portNum>\r\n"
            "  transport: 0=L2, 1=UDP\r\n");
        return;
    }
    seqId = (uint16_t)atoi(parameter);

    /* Get msgtype parameter */
    parameter = (char*) FreeRTOS_CLIGetParameter(commandString, 3, &paramLen);
    if (parameter == NULL)
    {
        EnetAppUtils_print(
            "Usage: enet_timesync send_ptp <seqid> <msgtype> <domain> <count> <transport> <portNum>\r\n"
            "  transport: 0=L2, 1=UDP\r\n");
        return;
    }
    msgType = (uint8_t)atoi(parameter);

    /* Get domain parameter */
    parameter = (char*) FreeRTOS_CLIGetParameter(commandString, 4, &paramLen);
    if (parameter == NULL)
    {
        EnetAppUtils_print(
            "Usage: enet_timesync send_ptp <seqid> <msgtype> <domain> <count> <transport> <portNum>\r\n"
            "  transport: 0=L2, 1=UDP\r\n");
        return;
    }
    domain = (uint8_t)atoi(parameter);

    /* Get count parameter */
    parameter = (char*) FreeRTOS_CLIGetParameter(commandString, 5, &paramLen);
    if (parameter == NULL)
    {
        EnetAppUtils_print(
            "Usage: enet_timesync send_ptp <seqid> <msgtype> <domain> <count> <transport> <portNum>\r\n"
            "  transport: 0=L2, 1=UDP\r\n");
        return;
    }
    count = (uint32_t)atoi(parameter);

    /* Get transport parameter */
    parameter = (char*) FreeRTOS_CLIGetParameter(commandString, 6, &paramLen);
    if (parameter == NULL)
    {
        EnetAppUtils_print(
            "Usage: enet_timesync send_ptp <seqid> <msgtype> <domain> <count> <transport> <portNum>\r\n"
            "  transport: 0=L2, 1=UDP\r\n");
        return;
    }
    transport = (uint8_t)atoi(parameter);

    /* Validate transport type */
    if (transport > 1)
    {
        EnetAppUtils_print(
            "Invalid transport type. Use 0 for L2 or 1 for UDP\r\n");
        return;
    }

    /* Get Macport number */
    parameter = (char*) FreeRTOS_CLIGetParameter(commandString, 7 , &paramLen);
    if (parameter == NULL)
    {
        EnetAppUtils_print(
            "Usage: enet_timesync send_ptp <seqid> <msgtype> <domain> <count> <transport> <portNum>\r\n"
            "  transport: 0=L2, 1=UDP\r\n");
        return;
    }
    portNum = atoi(parameter);

    if((portNum < 0) || (portNum > 2))
    {
        EnetAppUtils_print( "Port number should be between %d and %d\r\n",1,2);
        return;
    }

    /* Validate count */
    if (count == 0)
    {
        EnetAppUtils_print( "Count must be greater than 0\r\n");
        return;
    }

    /* Prepare and queue the request */
    currentRequest.seqId = seqId;
    currentRequest.msgType = msgType;
    currentRequest.domain = domain;
    currentRequest.count = count;
    currentRequest.transport = transport;
    currentRequest.totalCount = count;
    currentRequest.macPort = portNum-1;

    EnetApp_ptpSendPacketImpl(&currentRequest, &sentCount);

    EnetAppUtils_print(
                "Sent %u/%u PTP %s packets\r\n"
                "  SeqID: %u, MsgType: %u, Domain: %u\r\n",
                sentCount, count,
                (transport == 0) ? "L2" : "UDP",
                seqId, msgType, domain);
}

static void EnetApp_setPortTsEventPrms(CpswMacPort_TsEventCfg *tsPortEventCfg)
{
    tsPortEventCfg->commonPortIpCfg.ttlNonzeroEn = true;
    tsPortEventCfg->commonPortIpCfg.tsIp107En = false;
    tsPortEventCfg->commonPortIpCfg.tsIp129En = true;
    tsPortEventCfg->commonPortIpCfg.tsIp130En = false;
    tsPortEventCfg->commonPortIpCfg.tsIp131En = false;
    tsPortEventCfg->commonPortIpCfg.tsIp132En = false;
    tsPortEventCfg->commonPortIpCfg.tsPort319En = true;
    tsPortEventCfg->commonPortIpCfg.tsPort320En = true;
    tsPortEventCfg->commonPortIpCfg.unicastEn = false;
    tsPortEventCfg->domainOffset = 4U;
    tsPortEventCfg->ltype2En = false;
    tsPortEventCfg->rxAnnexDEn = true;
    tsPortEventCfg->rxAnnexEEn = true;
    tsPortEventCfg->rxAnnexFEn = true;
    tsPortEventCfg->txAnnexDEn = true;
    tsPortEventCfg->txAnnexEEn = true;
    tsPortEventCfg->txAnnexFEn = true;
    tsPortEventCfg->txHostTsEn = true;
    tsPortEventCfg->mcastType = 0U;
    tsPortEventCfg->messageType = 0xFFFFU;
    tsPortEventCfg->seqIdOffset = 30U;
}

static void EnetApp_cptsSetTsNudge(const char *commandString)
{
    char *parameter;
    BaseType_t paramLen;
    int32_t status;
    int32_t nudge_val;

    parameter = (char*) FreeRTOS_CLIGetParameter(commandString, 2, &paramLen);
    if(parameter == NULL)
    {
        EnetAppUtils_print( "Usage: timesync set_ts_nudge <nudge_val>\r\n");
        return;
    }

    nudge_val = atoi(parameter);
    Enet_IoctlPrms prms;
    ENET_IOCTL_SET_IN_ARGS(&prms, &nudge_val);

    ENET_IOCTL(EnetApp_inst.hEnet, EnetApp_inst.coreId, CPSW_CPTS_IOCTL_SET_TS_NUDGE, &prms, status);

    if(status != ENET_SOK)
    {
        EnetAppUtils_print( "Failed to set timestamp nudge\r\n");
    }
    else
    {
        EnetAppUtils_print( "Timestamp nudge set to %d\r\n", nudge_val);
    }
}

static void EnetApp_cptsSetComp(const char *commandString)
{
    char *parameter;
    BaseType_t paramLen;
    int32_t status;
    uint64_t comp_val_sec;
    uint64_t comp_val_nsec;
    CpswCpts_SetCompValInArgs inArgs;
    EnetTimeSync_TimestampAdj ppmCorrection;
    Enet_IoctlPrms prms, prms1;
    Enet_Handle hEnet;
    Cpsw_Handle hCpsw;
    CpswCpts_Handle hCpts;
    CSL_cptsRegs *regs;
    int32_t paramError = 0;

    parameter = (char*) FreeRTOS_CLIGetParameter(commandString, 2, &paramLen);
    if(parameter == NULL)
    {
        EnetAppUtils_print("Usage: enet_timesync set_comp <sec> <nsec>\r\n");
        paramError = 1;
    }
    else
    {
        comp_val_sec = atoi(parameter);
        parameter = (char*) FreeRTOS_CLIGetParameter(commandString, 3, &paramLen);
        if(parameter == NULL)
        {
            EnetAppUtils_print("Usage: enet_timesync set_comp <sec> <nsec>\r\n");
            paramError = 1;
        }
        else
        {
            comp_val_nsec = atoi(parameter);
        }
    }

    if (paramError == 0)
    {
        hEnet = EnetSoc_getEnetHandle(EnetApp_inst.enetType, EnetApp_inst.instId);
        if (hEnet == NULL)
        {
            EnetAppUtils_print("Failed to get Enet handle\r\n");
            paramError = 1;
        }
    }

    if (paramError == 0)
    {
        hCpsw = (Cpsw_Handle)hEnet->enetPer;
        hCpts = &hCpsw->cptsObj;
        regs = (CSL_cptsRegs *)hCpts->virtAddr;
        CSL_CPTS_setTSAddVal(regs, 0);

        ppmCorrection.adjValInNsecs = 0;
        ENET_IOCTL_SET_IN_ARGS(&prms1, &ppmCorrection);
        ENET_IOCTL(EnetApp_inst.hEnet, EnetApp_inst.coreId, ENET_TIMESYNC_IOCTL_ADJUST_TIMESTAMP, &prms1, status);
        if(status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to change the PPM value to zero\r\n");
            paramError = 1;
        }
        else
        {
            EnetAppUtils_print("Changing Add value and PPM value to zero\r\n");
        }
    }

    if (paramError == 0)
    {
        uint64_t comp_val = (comp_val_sec * ENET_TIMESYNC_NSEC_IN_SEC) + comp_val_nsec;
        inArgs.tsCompVal = comp_val;
        inArgs.tsCompLen = ENET_TIMESYNC_NSEC_IN_SEC;
        inArgs.tsCompToggle = false;

        ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);
        ENET_IOCTL(EnetApp_inst.hEnet, EnetApp_inst.coreId, CPSW_CPTS_IOCTL_SET_COMP, &prms, status);

        if(status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to set comparison value\r\n");
        }
        else
        {
            EnetAppUtils_print("Comparison value set to %llu with comp length of 1s\r\n", comp_val);
        }
    }
}

static void EnetApp_cptsSetCompNudge(const char *commandString)
{
    char *parameter;
    BaseType_t paramLen;
    int32_t status;
    int32_t nudge_val;

    parameter = (char*) FreeRTOS_CLIGetParameter(commandString, 2, &paramLen);
    if(parameter == NULL)
    {
        EnetAppUtils_print( "Usage: timesync set_comp_nudge <nudge_val>\r\n");
        return;
    }

    nudge_val = atoi(parameter);
    Enet_IoctlPrms prms;
    ENET_IOCTL_SET_IN_ARGS(&prms, &nudge_val);

    ENET_IOCTL(EnetApp_inst.hEnet, EnetApp_inst.coreId, CPSW_CPTS_IOCTL_SET_COMP_NUDGE, &prms, status);

    if(status != ENET_SOK)
    {
        EnetAppUtils_print( "Failed to set comparison nudge\r\n");
    }
    else
    {
        EnetAppUtils_print( "Comparison nudge set to %d\r\n", nudge_val);
    }
}

static void EnetApp_cptsSelectTsOutputBit(const char *commandString)
{
    char *parameter;
    BaseType_t paramLen;
    int32_t status;
    int32_t bit_val;

    parameter = (char*) FreeRTOS_CLIGetParameter(commandString, 2, &paramLen);
    if(parameter == NULL)
    {
        EnetAppUtils_print( "Usage: timesync select_ts_output <bit_val>\r\n");
        return;
    }

    /* Select which bit of the CPTS timestamp counter to output on TS_output signal.
     * bit_val specifies which bit position (0-31) of the 64-bit timestamp to route to the TS output pin.
     * This is used for hardware synchronization and external timestamp signaling. */
    bit_val = atoi(parameter);
    Enet_IoctlPrms prms;
    ENET_IOCTL_SET_IN_ARGS(&prms, &bit_val);

    ENET_IOCTL(EnetApp_inst.hEnet, EnetApp_inst.coreId, CPSW_CPTS_IOCTL_SELECT_TS_OUTPUT_BIT, &prms, status);

    if(status != ENET_SOK)
    {
        EnetAppUtils_print( "Failed to select timestamp output bit\r\n");
    }
    else
    {
        EnetAppUtils_print( "Timestamp output bit selected: %d\r\n", bit_val);
    }
}

static void EnetApp_cptsSetAddVal(const char *commandString)
{
    char *parameter;
    BaseType_t paramLen;
    int32_t add_val, check;

    parameter = (char*) FreeRTOS_CLIGetParameter(commandString, 2, &paramLen);
    if(parameter == NULL)
    {
        EnetAppUtils_print( "Usage: timesync set_add_val {<add_val>, -1 for default value}\r\n");
        return;
    }

    add_val = atoi(parameter);
    if(add_val == -1)
    {
        Cpsw_Cfg *pCfg = EnetApp_getCpswCfg(EnetApp_inst.enetType, EnetApp_inst.instId);
        add_val = pCfg->cptsCfg.cptsRftClkFreq;
    }

    Enet_Handle hEnet = EnetSoc_getEnetHandle(EnetApp_inst.enetType, EnetApp_inst.instId);
    Cpsw_Handle hCpsw = (Cpsw_Handle)hEnet->enetPer;
    CpswCpts_Handle hCpts = &hCpsw->cptsObj;
    CSL_cptsRegs *regs = (CSL_cptsRegs *)hCpts->virtAddr;

    CSL_CPTS_setTSAddVal(regs, (uint32_t)add_val);
    check = CSL_CPTS_getTSAddVal(regs);

    if(check != add_val)
    {
        EnetAppUtils_print( "Failed to set add val %u\r\n", (uint32_t)add_val);
    }
    else
    {
        EnetAppUtils_print( "Add value set to %u\r\n", (uint32_t)add_val);
    }
}
