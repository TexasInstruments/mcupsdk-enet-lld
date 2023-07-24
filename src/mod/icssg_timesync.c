/*
 *  Copyright (c) Texas Instruments Incorporated 2021
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
 * \file  icssg_timesync.c
 *
 * \brief This file contains the implementation of the ICSSG TimeSync driver.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdlib.h>
#include <hw_include/cslr_icss.h>
#include <drivers/hw_include/hw_types.h>
#include <include/core/enet_base.h>
#include <include/core/enet_utils.h>
#include <include/core/enet_mod.h>
#include <include/per/icssg.h>
#include <src/per/icssg_utils.h>
#include <priv/mod/icssg_timesync_priv.h>
#include <priv/mod/icssg_timesync_ioctl_priv.h>
#include <src/per/firmware/icssg/fw_mem_map.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/*! \brief Max TX packet delay (in nanoseconds). */
#define MAX_PKTTXDELAY                          (1000U)

/*! \brief 23 bit cycle count, 24th bit used for wraparound detection. */
#define IEP_COUNT_HI_BIT_WIDTH_FW               (23U)

#define HWREG(x)         (*((volatile uint32_t *)(x)))
#define HWREGB(x)        (*((volatile uint8_t *)(x)))

#define ICSSG_TIMESYNC_IOCTL_HANDLER_ENTRY_INIT(x)    \
          {.cmd = x,                            \
           .fxn = &IcssgTimeSync_ioctl_handler_##x}

#define ICSSG_TIMESYNC_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(x)    \
          {.cmd = x,                            \
           .fxn = &IcssgTimeSync_ioctl_handler_default}
/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/*!
 * \brief 802.1AS-rev Timestamp structure
 */
typedef struct IcssgTimeSync_TimeStamp_s
{
    /*! Reserved */
    uint32_t reserved;

    /*! 48 bit seconds field */
    uint64_t seconds;

    /*! 32 bit nanoseconds field */
    uint32_t nanoseconds;
} IcssgTimeSync_Timestamp;

/*!
 * \brief Set clock descriptor.
 */
typedef struct IcssgTimeSync_SetClkDesc_s
{
    /*! Flag to indicate that a setClock request is issued to firmware */
    uint8_t request;

    /*! Flag used by firmware to indicate restoring cycle */
    uint8_t restore;

    /*! Flag used to signal to PNIO ISR that setClock is complete */
    uint8_t ack;

    /*! CMP status value */
    uint8_t cmpStatus;

    /*! Time window around cycle boundary where setClock request is kept pending */
    uint32_t margin;

    /*! Cycle counter value to set */
    uint32_t cycleCnt0Set;

    /*! Cycle counter value to set (1) */
    uint32_t cycleCnt1Set;

    /*! IEP value to set */
    uint32_t iepCntSet;

    /*! Reserved for internal/future use */
    uint32_t rsvd32;

    /*! Cycle count increment factor */
    uint32_t incFactor;

    /*! Current CMP0 value used to restore */
    uint32_t cmp0Curr;

    /*! Current IEP count */
    uint32_t iepCntCurr;

    /*! Extension */
    uint32_t difference;

    /*! Cycle counter value to update in next cycle */
    uint32_t cycleCnt0New;

    /*! Cycle counter value to update in next cycle (1) */
    uint32_t cycleCnt1New;

    /*! Extended cycle value for CMP0 */
    uint32_t cmp0New;
} IcssgTimeSync_SetClkDesc;

/*!
 * \brief Time Sync Ioctl Handler Function.
 *
 * \param hPer      Enet Peripheral handle
 * \param cmd       IOCTL command Id
 * \param prms      IOCTL parameters
 */
typedef int32_t IcssgTimeSyncIoctlHandlerFxn_t(EnetMod_Handle hMod,
                                       uint32_t cmd,
                                       Enet_IoctlPrms *prms);

/*!
 * \brief Time Sync IOCTL register Handler structure.
 */
typedef struct IcssgTimeSyncIoctlHandlerTableEntry_s
{
    uint32_t cmd;
    IcssgTimeSyncIoctlHandlerFxn_t *fxn;
} IcssgTimeSyncIoctlHandlerTableEntry_t;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief Set clock time.
 *
 * \param hTimeSync  TimeSync handle
 * \param clkMode    Absolute or relative setClock mode for WorkingClock
 * \param clkSign    Sign for relative setClock mode for WorkingClock
 * \param clkTime    Absolute/relative time to set for WorkingClock
 *
 * \return #ENET_SOK or \ref Enet_ErrorCodes in case of any failure
 */
int32_t IcssgTimeSync_setClockTime(IcssgTimeSync_Handle hTimeSync,
                                   uint8_t clkMode,
                                   uint8_t clkSign,
                                   IcssgTimeSync_Timestamp *clkTime);

/*!
 * \brief Adjust clock by doing slow compensation.
 *
 * \param hTimeSync     TimeSync handle
 * \param drift         Clock drift to adjust
 * \param syncInterval  Sync interval to perform compensation
 *
 * \return #ENET_SOK or \ref Enet_ErrorCodes in case of any failure
 */
int32_t IcssgTimeSync_adjustClock(IcssgTimeSync_Handle hTimeSync,
                                  int32_t drift,
                                  uint32_t syncInterval);

/*!
 * \brief Get current time.
 *
 * \param hTimeSync     TimeSync handle
 * \param tsVal         Timestamp in nanoseconds
 *
 * \return #ENET_SOK or \ref Enet_ErrorCodes in case of any failure
 */
int32_t IcssgTimeSync_getClockTime(IcssgTimeSync_Handle hTimeSync,
                                   uint64_t* tsVal);

/*!
 * \brief Configure the packet TX delay.
 *
 * \param hTimeSync  TimeSync handle
 * \param portSel    Port number
 * \param txDelay    IEP timestamp to wire (MDI) delay for transmitted packets.
 *                   Equivalent to PortTxDelay = PTCPTxDelay + PHYTxDelay
 *
 * \return #ENET_SOK or \ref Enet_ErrorCodes in case of any failure
 */
int32_t IcssgTimeSync_configurePktTxDelay(IcssgTimeSync_Handle hTimeSync,
                                          uint8_t portSel,
                                          uint32_t TxDelay);

static IcssgTimeSyncIoctlHandlerFxn_t * Icssg_getTimeSyncIoctlHandler(EnetMod_Handle hMod,
                                                                uint32_t cmd,
                                                                IcssgTimeSyncIoctlHandlerTableEntry_t ioctlTbl[],
                                                                uint32_t numEntries);

int32_t IcssgTimeSync_ioctl_handler_ICSSG_TIMESYNC_IOCTL_REGISTER_HANDLER(EnetMod_Handle hMod,
                                                                        uint32_t cmd,
                                                                        Enet_IoctlPrms *prms);

int32_t  IcssgTimeSync_ioctl_handler_default(EnetMod_Handle hMod,
                                                uint32_t cmd,
                                                Enet_IoctlPrms *prms);
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static IcssgTimeSyncIoctlHandlerTableEntry_t IcssgTimeSyncIoctlHandlerTable[] =
{
    ICSSG_TIMESYNC_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_TIMESYNC_IOCTL_GET_CURRENT_TIMESTAMP),
    ICSSG_TIMESYNC_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_TIMESYNC_IOCTL_SET_TIMESTAMP),
    ICSSG_TIMESYNC_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_TIMESYNC_IOCTL_ADJUST_TIMESTAMP),
    ICSSG_TIMESYNC_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_TIMESYNC_IOCTL_SET_TIMESTAMP_COMPLETE),
    ICSSG_TIMESYNC_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_TIMESYNC_IOCTL_GET_ETH_TX_TIMESTAMP),
    ICSSG_TIMESYNC_IOCTL_HANDLER_ENTRY_INIT(ICSSG_TIMESYNC_IOCTL_REGISTER_HANDLER)
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void IcssgTimeSync_initCfg(IcssgTimeSync_Cfg *timeSyncCfg)
{
    timeSyncCfg->enable  = true;
    timeSyncCfg->clkType = ICSSG_TIMESYNC_CLKTYPE_WORKING_CLOCK;

    /* 10us and 25us */
    timeSyncCfg->syncOut_start_WC = 10000U;
    timeSyncCfg->syncOut_pwidth_WC = 25000U;
}

int32_t IcssgTimeSync_open(EnetMod_Handle hMod,
                           Enet_Type enetType,
                           uint32_t instId,
                           const void *cfg,
                           uint32_t cfgSize)
{
    IcssgTimeSync_Handle hTimeSync = (IcssgTimeSync_Handle)hMod;
    IcssgTimeSync_Cfg *timeSyncCfg = (IcssgTimeSync_Cfg *)cfg;
    uintptr_t iep0Regs = (uintptr_t)hMod->virtAddr;
    int32_t status = ENET_SOK;

    Enet_devAssert(cfgSize == sizeof(IcssgTimeSync_Cfg),
                   "Invalid ICCSG TimeSync config params size %u (expected %u)\n",
                   cfgSize, sizeof(IcssgTimeSync_Cfg));

    if ((hTimeSync->clkType == ICSSG_TIMESYNC_CLKTYPE_SYSTEM_TIME) ||
        (hTimeSync->clkType == ICSSG_TIMESYNC_CLKTYPE_GLOBAL_TIME))
    {
        ENETTRACE_ERR("%s: Clock type (%u) not supported\n", hMod->name, hTimeSync->clkType);
        status = ENET_ENOTSUPPORTED;
    }

    /* Init WorkingClock timers */
    if (status == ENET_SOK)
    {
        // TODO - Is this also true for LLD? */
        /* Cycle time (CMP0) already initialized by PN_initDrv */
        HWREG(iep0Regs + CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG) = 0x441U;

        /* Init WorkingClock signals */

        /* Firmware needs to reprogram SYNC CTRL every cycle */
        HWREG(iep0Regs + CSL_ICSS_G_PR1_IEP0_SLV_SYNC_CTRL_REG) |= 0x03U;

        /* Enable CMP1 CFG */
        HWREG(iep0Regs + CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG) |= 0x00000004U;

        HWREG(iep0Regs + CSL_ICSS_G_PR1_IEP0_SLV_CMP1_REG0) = timeSyncCfg->syncOut_start_WC;

        /* REG1 is loaded to REG0 in shadow mode */
        HWREG(iep0Regs + CSL_ICSS_G_PR1_IEP0_SLV_CMP1_REG1) = timeSyncCfg->syncOut_start_WC;

        HWREG(iep0Regs + CSL_ICSS_G_PR1_IEP0_SLV_SYNC_PWIDTH_REG) = (timeSyncCfg->syncOut_pwidth_WC / 5U) - 1U;

        hTimeSync->clkType = timeSyncCfg->clkType;
    }

    return status;
}

void IcssgTimeSync_close(EnetMod_Handle hMod)
{
    uintptr_t iep0Regs = (uintptr_t)hMod->virtAddr;

    /* Deinit WorkingClock signals */

    /* Disable CMP1 CFG */
    HWREG(iep0Regs + CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG) &= ~0x00000004U;

    /* Deinit WorkingClock timers */
    // FIXME - Is this needed?
    //HWREG(iep0Regs + CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG) = 0U;
}

int32_t IcssgTimeSync_rejoin(EnetMod_Handle hMod,
                             Enet_Type enetType,
                             uint32_t instId)
{
    return ENET_ENOTSUPPORTED;
}

int32_t IcssgTimeSync_ioctl(EnetMod_Handle hMod,
                            uint32_t cmd,
                            Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;
    IcssgTimeSyncIoctlHandlerFxn_t * ioctlHandler;

    ioctlHandler = Icssg_getTimeSyncIoctlHandler(hMod, cmd, IcssgTimeSyncIoctlHandlerTable , ENET_ARRAYSIZE(IcssgTimeSyncIoctlHandlerTable));
    Enet_assert(ioctlHandler != NULL);
    status = ioctlHandler(hMod, cmd, prms);
    return status;
}

int32_t IcssgTimeSync_setClockTime(IcssgTimeSync_Handle hTimeSync,
                                   uint8_t clkMode,
                                   uint8_t clkSign,
                                   IcssgTimeSync_Timestamp *clkTime)
{
    EnetMod_Handle hMod = ENET_MOD(hTimeSync);
    uintptr_t sharedRam = (uintptr_t)hMod->virtAddr2;
    IcssgTimeSync_SetClkDesc setClkDesc;
    uint8_t *pSetClkDesc;
    uint64_t cycleCnt;
    uint32_t cycleTime;
    uint32_t i;
    int32_t status = ENET_SOK;

    if (hTimeSync->setClockOngoing)
    {
        ENETTRACE_ERR("%s: Cannot set new clock time, previous operation ongoing\n", hMod->name);
        status = ENET_EBUSY;
    }

    if (status == ENET_SOK)
    {
        /* Write setclock descriptor to ICSS */
        Icssg_Handle hIcssg = (Icssg_Handle)hTimeSync->hIcssg;
        cycleTime = hIcssg->cycleTimeNs;

        cycleCnt = ((clkTime->seconds * 1000000000) + clkTime->nanoseconds) / cycleTime;

        setClkDesc.request = 0x00U;  /* Write request later */
        setClkDesc.restore = 0x00U;
        setClkDesc.ack     = 0x00U;
        setClkDesc.cmpStatus = 0x00U;

        setClkDesc.margin = cycleTime - 1000U;
        setClkDesc.cycleCnt0Set = cycleCnt & 0x00000000FFFFFFFF;
        setClkDesc.cycleCnt1Set = (cycleCnt & 0xFFFFFFFF00000000) >> 32U;
        setClkDesc.iepCntSet = clkTime->nanoseconds % cycleTime;
        setClkDesc.incFactor = 1U;
        setClkDesc.cmp0Curr = cycleTime - 4U; /* Count from 0 to (cycle time)-4 */

        setClkDesc.iepCntCurr = 0U;
        setClkDesc.difference = 0U;

        setClkDesc.cycleCnt0New = 0U;
        setClkDesc.cycleCnt1New = 0U;
        setClkDesc.cmp0New = 0U;

        pSetClkDesc = (uint8_t *)(&setClkDesc);

        for (i = 0U; i < sizeof(IcssgTimeSync_SetClkDesc); i++)
        {
            HWREGB(sharedRam + TIMESYNC_FW_WC_SETCLOCK_DESC_OFFSET + i) = *(pSetClkDesc + i);
        }

        /* Writes to setClkDesc request. Request to set time */
        if (clkMode == 0U)
        {
            HWREGB(sharedRam + TIMESYNC_FW_WC_SETCLOCK_DESC_OFFSET) = 0x01U;
        }
        else if (clkMode == 1U)
        {
            if (clkSign == 0U)
            {
                HWREGB(sharedRam + TIMESYNC_FW_WC_SETCLOCK_DESC_OFFSET) = 0x11U;
            }
            else if (clkSign == 1U)
            {
                HWREGB(sharedRam + TIMESYNC_FW_WC_SETCLOCK_DESC_OFFSET) = 0x31U;
            }
            else
            {
                ENETTRACE_ERR("%s: Invalid clock sign %u\n", hMod->name, clkSign);
                status = ENET_EINVALIDPARAMS;
            }
        }
        else
        {
            ENETTRACE_ERR("%s: Invalid clock mode %u\n", hMod->name, clkMode);
            status = ENET_EINVALIDPARAMS;
        }

        hTimeSync->setClockOngoing = true;

        // FIXME - When is clock setting done?
        //hTimeSync->setClockOngoing = false;
    }

    return status;
}

int32_t IcssgTimeSync_adjustClock(IcssgTimeSync_Handle hTimeSync,
                                  int32_t drift,
                                  uint32_t syncInterval)
{
    EnetMod_Handle hMod = ENET_MOD(hTimeSync);
    uintptr_t iep0Regs = (uintptr_t)hMod->virtAddr;
    uint32_t globalCfg = 0U;
    uint32_t compensationPeriod = 0U;
    uint32_t driftAbs = (uint32_t)((drift > 0) ?  drift : -drift);
    int32_t status = ENET_SOK;

    /* Configure IEP rate compensation using 'Slow compensation' */
    if (drift != 0)
    {
        if (driftAbs > syncInterval)
        {
            ENETTRACE_ERR("%s: Drift is too large %d (exp abs(%d)< %u)\n", hMod->name, drift, syncInterval);
            status = ENET_EINVALIDPARAMS;
        }

        /*
         * Set compensation interval = 4 * sync interval/drift.
         * '4' for compensation increment 0ns (default increment - 4).
         * Example:
         *   Sync interval = 30ms
         *   Drift = -300ns
         *   Compensation interval = 4 * 30ms / 300ns = 4 * 30000000 / 300 = 4 * 100000ns*/
        if (status == ENET_SOK)
        {
            compensationPeriod = 4U * (syncInterval / abs(drift));

            /* Compensation increment = 8ns for positive drift, 0ns for negative drift */
            globalCfg = (drift > 0) ? 0x00000841U : 0x00000041U;

            /* Set compensation interval in IEP cycles */
            compensationPeriod = compensationPeriod / 4U;
        }

    }
    else
    {
        /* If drift is 0, set compensation increment to 4ns and disable slow compensation */

        /* Compensation increment = 4ns */
        globalCfg = 0x00000441U;

        /* Set compensation interval in IEP cycles */
        compensationPeriod = 0U;
    }

    /* Allow extension or shrinking of cycles by only 10% max */
    if (status == ENET_SOK)
    {
        if ((compensationPeriod > 0) && (compensationPeriod < 10))
        {
            ENETTRACE_ERR("%s: Invalid compensation period %u, exp < 10\n",
                          hMod->name, compensationPeriod);
            status = ENET_EINVALIDPARAMS;
        }
    }

    /* Write adjustment to slow compensation register */
    if (status == ENET_SOK)
    {
        /* Compensation increment */
        HWREG(iep0Regs + CSL_ICSSIEP_GLOBAL_CFG_REG) = globalCfg;

        /* Set compensation interval in IEP cycles */
        HWREG(iep0Regs + CSL_ICSSIEP_SLOW_COMPEN_REG) = compensationPeriod;

        /* Workaround for rate computation */
        hTimeSync->drift = drift;
        hTimeSync->syncInterval = syncInterval;
    }

    return status;
}

int32_t IcssgTimeSync_getClockTime(IcssgTimeSync_Handle hTimeSync,
                                   uint64_t *tsVal)
{
    EnetMod_Handle hMod = ENET_MOD(hTimeSync);
    uintptr_t iep0Regs = (uintptr_t)hMod->virtAddr;
    uintptr_t sharedRam = (uintptr_t)hMod->virtAddr2;
    uint32_t iepCntHi = 0U;
    uint32_t iepCntHiR = 0U;
    uint32_t iepCntLo = 0U;
    uint64_t rolloverCntHi = 0ULL;
    uint64_t rolloverCntHiR = 0ULL;
    int32_t status = ENET_SOK;

    if (hTimeSync->setClockOngoing)
    {
        ENETTRACE_ERR("%s: Cannot set get clock time, previous operation ongoing\n", hMod->name);
        status = ENET_EBUSY;
    }

    if (status == ENET_SOK)
    {
        /* Read IEP and cycle counter */
        /*
         * Pseudocode:
         *  1. Read the upper half of the timer into H.
         *  2. Read the lower half of the timer into L.
         *  3. Read the upper half of the timer again into H'.
         *  4. If H == H' then return {H, L}, otherwise go back to 1.
         */
        do
        {
            iepCntHi = (*((volatile uint32_t *)(iep0Regs + CSL_ICSS_G_PR1_IEP0_SLV_COUNT_REG1))) & 0x007FFFFF;
            iepCntHi += HWREG(sharedRam + TIMESYNC_FW_WC_COUNT_HI_SW_OFFSET_OFFSET);
            rolloverCntHi = *((uint32_t *)(sharedRam + TIMESYNC_FW_WC_HI_ROLLOVER_COUNT_OFFSET));
            iepCntLo = (*((volatile uint32_t *)(iep0Regs + CSL_ICSS_G_PR1_IEP0_SLV_COUNT_REG0))) & 0xFFFFFFFF;

            iepCntHiR = (*((volatile uint32_t *)(iep0Regs + CSL_ICSS_G_PR1_IEP0_SLV_COUNT_REG1))) & 0x007FFFFF;
            iepCntHiR += HWREG(sharedRam + TIMESYNC_FW_WC_COUNT_HI_SW_OFFSET_OFFSET);
            rolloverCntHiR = *((uint32_t *)(sharedRam + TIMESYNC_FW_WC_HI_ROLLOVER_COUNT_OFFSET));
        }
        while ((iepCntHiR != iepCntHi) || (rolloverCntHiR != rolloverCntHi));

        /* Compute 64 bit nanoseconds.
         * It can hold only 48 bit seconds. IEP count is 32 bit nanoseconds */
        *tsVal = ((uint64_t)(iepCntHi | (rolloverCntHi << IEP_COUNT_HI_BIT_WIDTH_FW)) * 1000000 /*hTimeSync->pn_handle->cycleTime*/) + iepCntLo;
    }

    return status;
}

int32_t IcssgTimeSync_configurePktTxDelay(IcssgTimeSync_Handle hTimeSync,
                                          uint8_t portSel,
                                          uint32_t txDelay)
{
    EnetMod_Handle hMod = ENET_MOD(hTimeSync);
    uintptr_t sharedRam = (uintptr_t)hMod->virtAddr2;
    int32_t status = ENET_SOK;

    ENETTRACE_DBG("%s: portSel=%u, txDelay=0x%08x\n", hMod->name, portSel, txDelay);

    if ((portSel != 1) && (portSel != 2))
    {
        ENETTRACE_ERR("%s: Invalid port number %u\n", hMod->name, portSel);
        status = ENET_EINVALIDPARAMS;
    }

    if (txDelay > MAX_PKTTXDELAY)
    {
        ENETTRACE_ERR("%s: Invalid TX delay %u (exp <= %u)\n", hMod->name, txDelay, MAX_PKTTXDELAY);
        status = ENET_EINVALIDPARAMS;
    }

    if (status == ENET_SOK)
    {
        if (portSel == 1U)
        {
            HWREG(sharedRam + TIMESYNC_FW_WC_PKTTXDELAY_P1_OFFSET) = txDelay;
        }
        else if (portSel == 2U)
        {
            HWREG(sharedRam + TIMESYNC_FW_WC_PKTTXDELAY_P2_OFFSET) = txDelay;
        }
    }

    return status;
}

int32_t  IcssgTimeSync_ioctl_handler_ENET_TIMESYNC_IOCTL_GET_CURRENT_TIMESTAMP(EnetMod_Handle hMod,
                                             uint32_t cmd,
                                             Enet_IoctlPrms *prms)
{
    IcssgTimeSync_Handle hTimeSync = (IcssgTimeSync_Handle)hMod;
    int32_t status = ENET_SOK;

    Enet_assert(cmd == ENET_TIMESYNC_IOCTL_GET_CURRENT_TIMESTAMP);

    uint64_t *tsVal = (uint64_t *)prms->outArgs;

    status = IcssgTimeSync_getClockTime(hTimeSync, tsVal);
    ENETTRACE_ERR_IF((status != ENET_SOK),
                        "%s: Failed to get clock time: %d\n", hMod->name, status);

    return status;

}

int32_t  IcssgTimeSync_ioctl_handler_ENET_TIMESYNC_IOCTL_SET_TIMESTAMP(EnetMod_Handle hMod,
                                             uint32_t cmd,
                                             Enet_IoctlPrms *prms)
{
    IcssgTimeSync_Handle hTimeSync = (IcssgTimeSync_Handle)hMod;
    int32_t status = ENET_SOK;

    Enet_assert(cmd == ENET_TIMESYNC_IOCTL_SET_TIMESTAMP);

    EnetTimeSync_setTimestamp *tsSet = (EnetTimeSync_setTimestamp *)prms->inArgs;
    const uint64_t tsLoadVal = tsSet->tsLoadVal;
    IcssgTimeSync_Timestamp clkTime;
    uint8_t clkMode = tsSet->clkMode; /* Absolute or relative setClock mode for WorkingClock*/
    uint8_t clkSign = tsSet->clkSign; // FIXME

    clkTime.nanoseconds = tsLoadVal % 1000000000;
    clkTime.seconds     = tsLoadVal / 1000000000;

    status = IcssgTimeSync_setClockTime(hTimeSync, clkMode, clkSign, &clkTime);
    ENETTRACE_ERR_IF((status != ENET_SOK),
                        "%s: Failed to set clock time: %d\n", hMod->name, status);

    return status;

}

int32_t  IcssgTimeSync_ioctl_handler_ENET_TIMESYNC_IOCTL_ADJUST_TIMESTAMP(EnetMod_Handle hMod,
                                             uint32_t cmd,
                                             Enet_IoctlPrms *prms)
{
    IcssgTimeSync_Handle hTimeSync = (IcssgTimeSync_Handle)hMod;
    int32_t status = ENET_SOK;

    Enet_assert(cmd == ENET_TIMESYNC_IOCTL_ADJUST_TIMESTAMP);

    const EnetTimeSync_TimestampAdj *tsAdj = (const EnetTimeSync_TimestampAdj *)prms->inArgs;

    status = IcssgTimeSync_adjustClock(hTimeSync,
                                        tsAdj->adjValInNsecs,
                                        tsAdj->intervalInNsecs);
    ENETTRACE_ERR_IF((status != ENET_SOK),
                        "%s: Failed to adjust clock: %d\n", hMod->name, status);

    return status;

}

int32_t  IcssgTimeSync_ioctl_handler_ENET_TIMESYNC_IOCTL_SET_TIMESTAMP_COMPLETE(EnetMod_Handle hMod,
                                             uint32_t cmd,
                                             Enet_IoctlPrms *prms)
{
    IcssgTimeSync_Handle hTimeSync = (IcssgTimeSync_Handle)hMod;
    int32_t status = ENET_SOK;

    Enet_assert(cmd == ENET_TIMESYNC_IOCTL_SET_TIMESTAMP_COMPLETE);

    hTimeSync->setClockOngoing = false;

    return status;

}

static int32_t IcssgTimeSync_getTxTs(IcssgTimeSync_Handle hTimeSync,
                                     Enet_MacPort macPort,
                                     uint32_t seqId,
                                     uint64_t *ts)
{
    int32_t hwQLevel;
    int32_t status = ENET_SOK;
    Icssg_Handle hIcssg = (Icssg_Handle)hTimeSync->hIcssg;
    EnetPer_Handle hPer = (EnetPer_Handle)hIcssg;
    uint32_t *pMgmtPkt = NULL;
    uint32_t txTsId = 0;
    uint64_t tsVal = 0;
    uint32_t slice;

    hwQLevel = IcssgUtils_hwqLevel(hIcssg, macPort, ICSSG_TXTS_RX_HWQA);
    if (hwQLevel != 0)
    {
        pMgmtPkt = (uint32_t*)IcssgUtils_hwqPop(hIcssg, macPort, ICSSG_TXTS_RX_HWQA);
        if (pMgmtPkt != NULL)
        {
             slice = ENET_GET_BIT(pMgmtPkt[0], 23);
             txTsId = pMgmtPkt[2];
             tsVal = pMgmtPkt[4];
             tsVal = tsVal << 32U | pMgmtPkt[3];
             tsVal = Icssg_convertTs(hPer, tsVal);

             /* Pop from port-dependent HwQ, but push into specific HwQ as indicated
              * by bit 23 of word 0, irrespective of port number */
             IcssgUtils_hwqPushForSlice(hIcssg, slice, ICSSG_TXTS_FREE_HWQA, pMgmtPkt);
        }
        if (txTsId != seqId)
        {
            status = ENET_ENOTFOUND;
        }
        *ts = tsVal;
    }
    else
    {
         status = ENET_ENOTFOUND;
    }
    return status;
}

int32_t IcssgTimeSync_ioctl_handler_ENET_TIMESYNC_IOCTL_GET_ETH_TX_TIMESTAMP(EnetMod_Handle hMod,
                                                                             uint32_t cmd,
                                                                             Enet_IoctlPrms *prms)
{
    IcssgTimeSync_Handle hTimeSync = (IcssgTimeSync_Handle)hMod;
    const EnetTimeSync_GetEthTimestampInArgs *inArgs =
           (const EnetTimeSync_GetEthTimestampInArgs *)prms->inArgs;
    uint64_t *ts = (uint64_t *)prms->outArgs;
    int32_t status = ENET_SOK;
    void *portNum = (void *) &(inArgs->portNum);
    Enet_MacPort macPort = *(Enet_MacPort *)portNum;

    Enet_assert(cmd == ENET_TIMESYNC_IOCTL_GET_ETH_TX_TIMESTAMP);
    /* Pop the top entry in the entry in the queue and return.
     * Driver is not maintaining any pool of timestamps, so application
     * should makesure to call this ioctl in proper order of tx pkts.
     * TODO: Driver to maintain the pool of timestamps using polling logic? */
    status = IcssgTimeSync_getTxTs(hTimeSync, macPort, inArgs->seqId, ts);

    return status;

}

static int32_t Icssg_getTimeSyncIoctlHandlerEntry(EnetMod_Handle hMod, uint32_t cmd, IcssgTimeSyncIoctlHandlerTableEntry_t ioctlTbl[], uint32_t numEntries, uint32_t *entryIdx)
{
    uint32_t i;
    int32_t status;

    for (i = 0; i < numEntries; i++)
    {
        if (ioctlTbl[i].cmd == cmd)
        {
            break;
        }
    }
    if (i < numEntries)
    {
        *entryIdx = i;
        status = ENET_SOK;
    }
    else
    {
        ENETTRACE_ERR("%s: failed to get ioctl handler for ICSSG STATS IOCTL cmd %x\r\n", cmd);
        status = ENET_EINVALIDPARAMS;
    }
    return status;
}

static IcssgTimeSyncIoctlHandlerFxn_t * Icssg_getTimeSyncIoctlHandler(EnetMod_Handle hMod, uint32_t cmd, IcssgTimeSyncIoctlHandlerTableEntry_t ioctlTbl[], uint32_t numEntries)
{
    uint32_t entryIndex;
    int32_t status;
    IcssgTimeSyncIoctlHandlerFxn_t *ioctlHandler = NULL;

    status = Icssg_getTimeSyncIoctlHandlerEntry(hMod, cmd, ioctlTbl, numEntries, &entryIndex);
    if (status == ENET_SOK)
    {
        Enet_assert(entryIndex < numEntries);
        ioctlHandler = ioctlTbl[entryIndex].fxn;
    }
    else
    {
        ENETTRACE_ERR("%s: failed to get ioctl handler for ICSSG STATS IOCTL cmd %x\r\n", cmd);
        ioctlHandler = &IcssgTimeSync_ioctl_handler_default;
    }
    return ioctlHandler;
}


int32_t IcssgTimeSync_ioctl_handler_default(EnetMod_Handle hMod,
                                    uint32_t cmd,
                                    Enet_IoctlPrms *prms)
{
    return ENET_ENOTSUPPORTED;
}

int32_t IcssgTimeSync_ioctl_handler_registerHandler(EnetMod_Handle hMod,
                                                    uint32_t cmd,
                                                    Enet_IoctlPrms *prms)
{

    int32_t status = ENET_SOK;
    IcssgTimeSyncIoctlHandlerTableEntry_t *ioctlHandlerToRegister  = (IcssgTimeSyncIoctlHandlerTableEntry_t *)prms->inArgs;
    IcssgTimeSyncIoctlHandlerTableEntry_t *currentIoctlTblEntry;
    uint32_t entryIndex;

    Enet_assert(cmd == ICSSG_TIMESYNC_IOCTL_REGISTER_HANDLER);
    status = Icssg_getTimeSyncIoctlHandlerEntry(hMod, ioctlHandlerToRegister->cmd,
                                        IcssgTimeSyncIoctlHandlerTable ,
                                        ENET_ARRAYSIZE(IcssgTimeSyncIoctlHandlerTable),
                                        &entryIndex);
    if (status == ENET_SOK)
    {
        Enet_assert(entryIndex < ENET_ARRAYSIZE(IcssgTimeSyncIoctlHandlerTable));
        currentIoctlTblEntry = &IcssgTimeSyncIoctlHandlerTable[entryIndex];
        Enet_assert(ioctlHandlerToRegister->cmd == currentIoctlTblEntry->cmd);
        currentIoctlTblEntry->fxn = (IcssgTimeSyncIoctlHandlerFxn_t *)ioctlHandlerToRegister->fxn;
    }
    else
    {
        ENETTRACE_ERR("%s: failed to get ioctl handler for ICSSG STATS IOCTL cmd %x\r\n", cmd);
        status = ENET_EINVALIDPARAMS;
    }
    return status;
}

int32_t IcssgTimeSync_ioctl_handler_ICSSG_TIMESYNC_IOCTL_REGISTER_HANDLER(EnetMod_Handle hMod,
                                                                        uint32_t cmd,
                                                                        Enet_IoctlPrms *prms)
{
    int32_t status;

    status = IcssgTimeSync_ioctl_handler_registerHandler(hMod, cmd, prms);
    return status;
}
